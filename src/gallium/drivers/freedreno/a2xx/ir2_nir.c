/*
 * Copyright (C) 2018 Jonathan Marek <jonathan@marek.ca>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Jonathan Marek <jonathan@marek.ca>
 */

#include "ir2_private.h"
#include "nir/tgsi_to_nir.h"

#include "freedreno_util.h"
#include "fd2_program.h"

static const nir_shader_compiler_options options = {
	.lower_fpow = true,
	.lower_scmp = true,
	.lower_flrp32 = true,
	.lower_flrp64 = true,
	.lower_fmod32 = true,
	.lower_fmod64 = true,
	.lower_fdiv = true,
	.lower_ldexp = true,
	.fuse_ffma = true,
	.vertex_id_zero_based = true,
	// XXX what does this change?
	// .fdot_replicates = true,

	.lower_extract_byte = true,
	.lower_extract_word = true,
	.lower_helper_invocation = true,
	.lower_all_io_to_temps = true,
};

struct nir_shader *ir2_tgsi_to_nir(const struct tgsi_token *tokens)
{
	return tgsi_to_nir(tokens, &options);
}

const nir_shader_compiler_options *ir2_get_compiler_options(void)
{
	return &options;
}

#define OPT(nir, pass, ...) ({                             \
   bool this_progress = false;                             \
   NIR_PASS(this_progress, nir, pass, ##__VA_ARGS__);      \
   this_progress;                                          \
})
#define OPT_V(nir, pass, ...) NIR_PASS_V(nir, pass, ##__VA_ARGS__)

static void ir2_optimize_loop(nir_shader * s)
{
	bool progress;
	do {
		progress = false;

		OPT_V(s, nir_lower_vars_to_ssa);
		progress |= OPT(s, nir_opt_copy_prop_vars);
		progress |= OPT(s, nir_copy_prop);
		progress |= OPT(s, nir_opt_dce);
		progress |= OPT(s, nir_opt_cse);
		static int gcm = -1;
		if (gcm == -1)
			gcm = env2u("GCM");
		if (gcm == 1)
			progress |= OPT(s, nir_opt_gcm, true);
		else if (gcm == 2)
			progress |= OPT(s, nir_opt_gcm, false);
		progress |= OPT(s, nir_opt_peephole_select, UINT_MAX);
		progress |= OPT(s, nir_opt_intrinsics);
		progress |= OPT(s, nir_opt_algebraic);
		progress |= OPT(s, nir_opt_constant_folding);
		progress |= OPT(s, nir_opt_dead_cf);
		if (OPT(s, nir_opt_trivial_continues)) {
			progress |= true;
			/* If nir_opt_trivial_continues makes progress, then we need to clean
			 * things up if we want any hope of nir_opt_if or nir_opt_loop_unroll
			 * to make progress.
			 */
			OPT(s, nir_copy_prop);
			OPT(s, nir_opt_dce);
		}
		progress |= OPT(s, nir_opt_loop_unroll, nir_var_all);
		progress |= OPT(s, nir_opt_if);
		progress |= OPT(s, nir_opt_remove_phis);
		progress |= OPT(s, nir_opt_undef);

	} while (progress);
}

// XXX
bool ir3_nir_apply_trig_workarounds(nir_shader * shader);

int ir2_optimize_nir(nir_shader * s)
{
	struct nir_lower_tex_options tex_options = {
		.lower_txp = ~0u,
		.lower_rect = 0,
	};

	if (fd_mesa_debug & FD_DBG_DISASM) {
		debug_printf("----------------------\n");
		nir_print_shader(s, stdout);
		debug_printf("----------------------\n");
	}

	OPT_V(s, nir_opt_global_to_local);
	OPT_V(s, nir_lower_regs_to_ssa);
	OPT_V(s, nir_lower_vars_to_ssa);

	OPT_V(s, ir3_nir_apply_trig_workarounds);

	OPT_V(s, nir_lower_tex, &tex_options);

	/* lower to scalar instructions that can only be scalar on a2xx */
	OPT_V(s, ir2_nir_lower_scalar);

	ir2_optimize_loop(s);

	OPT_V(s, nir_remove_dead_variables, nir_var_local);
	OPT_V(s, nir_move_load_const);

	/* postprocess */
	OPT_V(s, nir_opt_algebraic_late);

	OPT_V(s, nir_lower_to_source_mods);
	OPT_V(s, nir_copy_prop);
	OPT_V(s, nir_opt_dce);
	OPT_V(s, nir_opt_move_comparisons);

	OPT_V(s, nir_lower_locals_to_regs);

	OPT_V(s, nir_convert_from_ssa, true);

	OPT_V(s, nir_move_vec_src_uses_to_dest);
	OPT_V(s, nir_lower_vec_to_movs);

	OPT_V(s, nir_opt_dce);

	nir_sweep(s);

	if (fd_mesa_debug & FD_DBG_DISASM) {
		debug_printf("----------------------\n");
		nir_print_shader(s, stdout);
		debug_printf("----------------------\n");
	}

	/* check for unimplemented instrs and return failure */
	nir_foreach_block(block, nir_shader_get_entrypoint(s)) {
		nir_foreach_instr(instr, block) {
			switch (instr->type) {
			case nir_instr_type_alu:
			case nir_instr_type_deref:
			case nir_instr_type_intrinsic:
			case nir_instr_type_load_const:
			case nir_instr_type_tex:
				break;
			case nir_instr_type_ssa_undef:	// XXX should be implemented
			default:
				printf("Unhandled NIR instruction type: %d\n",
					   instr->type);
				return -1;
			}
		}
	}

	return 0;
}

/* instruction idx to ssa/reg idx */

static void
set_index(struct ir2_context *ctx, nir_dest * dst,
		  struct ir2_instruction *instr)
{
	if (dst->is_ssa) {
		ctx->ssa_map[dst->ssa.index] = instr->idx;
		return;
	}

	int idx = ctx->reg_map[dst->reg.reg->index];
	if (idx < 0) {
		ctx->reg_map[dst->reg.reg->index] = instr->idx;
		idx = instr->idx;
	}
	instr->reg_idx = idx;
}

static int get_index(struct ir2_context *ctx, nir_src * src)
{
	return src->is_ssa ?
		ctx->ssa_map[src->ssa->index] : ctx->reg_map[src->reg.reg->index];
}

static inline struct ir2_instruction *ir2_instr_create(struct ir2_context
													   *ctx,
													   int instr_type)
{
	struct ir2_instruction *instr;

	instr = &ctx->instr[ctx->instr_count++];
	instr->idx = ctx->instr_count - 1;
	instr->instr_type = instr_type;
	instr->reg_idx = -1;
	return instr;
}

static struct ir2_instruction *instr_create_alu(struct ir2_context *ctx,
												nir_op opcode,
												unsigned ncomp)
{
	static const struct ir2_opc {
		int8_t scalar, vector;
	} nir_ir2_opc[nir_num_opcodes] = {
		[0 ... nir_num_opcodes - 1] = {
		-1, -1},[nir_op_fmov] = {
		MAXs, MAXv},[nir_op_fnot] = {
		SETEs, SETEv},[nir_op_for] = {
		MAXs, MAXv},[nir_op_fand] = {
		MINs, MINv},[nir_op_fadd] = {
		ADDs, ADDv},[nir_op_fsub] = {
		ADDs, ADDv},[nir_op_fmul] = {
		MULs, MULv},[nir_op_ffma] = {
		-1, MULADDv},[nir_op_fmax] = {
		MAXs, MAXv},[nir_op_fmin] = {
		MINs, MINv},[nir_op_ffloor] = {
		FLOORs, FLOORv},[nir_op_ffract] = {
		FRACs, FRACv},[nir_op_fdot2] = {
		-1, DOT2ADDv},[nir_op_fdot3] = {
		-1, DOT3v},[nir_op_fdot4] = {
		-1, DOT4v},[nir_op_fge] = {
		-1, SETGTEv},[nir_op_flt] = {
		-1, SETGTv},[nir_op_fne] = {
		-1, SETNEv},[nir_op_feq] = {
		-1, SETEv},[nir_op_fcsel] = {
		-1, CNDEv},[nir_op_frsq] = {
		RECIPSQ_IEEE, -1},[nir_op_frcp] = {
		RECIP_CLAMP, -1},[nir_op_flog2] = {
		LOG_CLAMP, -1},[nir_op_fexp2] = {
		EXP_IEEE, -1},[nir_op_fsqrt] = {
		SQRT_IEEE, -1},[nir_op_fcos] = {
		COS, -1},[nir_op_fsin] = {
		SIN, -1},
			// XXX IEEE vs CLAMP?
			// XXX these should never happen with native_integers=false?
			[nir_op_imov] = {
		MAXs, MAXv},[nir_op_b2f] = {
		MAXs, MAXv},[nir_op_inot] = {
		SETEs, SETEv},[nir_op_ior] = {
		MAXs, MAXv},[nir_op_iand] = {
		MINs, MINv},[nir_op_bcsel] = {
	-1, CNDEv},};

	struct ir2_opc op = nir_ir2_opc[opcode];
	assert(op.vector >= 0 || op.scalar >= 0);

	struct ir2_instruction *instr = ir2_instr_create(ctx, IR2_ALU);
	instr->alu.vector_opc = op.vector;
	instr->alu.scalar_opc = op.scalar;
	instr->alu.export = -1;
	instr->alu.write_mask = (1 << ncomp) - 1;
	instr->src_reg_count = nir_op_infos[opcode].num_inputs;
	instr->num_components = ncomp;
	return instr;
}

static struct ir2_instruction *instr_create_alu_dest(struct ir2_context
													 *ctx, nir_op opcode,
													 nir_dest * dst)
{
	struct ir2_instruction *instr;
	instr = instr_create_alu(ctx, opcode, nir_dest_num_components(*dst));
	set_index(ctx, dst, instr);
	return instr;
}

static struct ir2_instruction *ir2_instr_create_fetch(struct ir2_context
													  *ctx, nir_dest * dst,
													  bool tex_fetch)
{
	struct ir2_instruction *instr = ir2_instr_create(ctx, IR2_FETCH);
	instr->fetch.opc = tex_fetch ? TEX_FETCH : VTX_FETCH;
	instr->src_reg_count = 1;
	instr->num_components = nir_dest_num_components(*dst);
	set_index(ctx, dst, instr);
	return instr;
}

static void emit_alu(struct ir2_context *ctx, nir_alu_instr * alu)
{
	const nir_op_info *info = &nir_op_infos[alu->op];
	nir_dest *dst = &alu->dest.dest;
	struct ir2_instruction *instr;
	struct ir2_src tmp;
	unsigned ncomp;

	assert(info->output_size <= 1);

	/* get the number of dst components */
	if (dst->is_ssa) {
		ncomp = dst->ssa.num_components;
	} else {
		ncomp = 0;
		for (int i = 0; i < 4; i++)
			ncomp += ! !(alu->dest.write_mask & 1 << i);
	}

	instr = instr_create_alu(ctx, alu->op, ncomp);
	set_index(ctx, dst, instr);
	instr->alu.saturate = alu->dest.saturate;
	instr->alu.write_mask = alu->dest.write_mask;

	for (int i = 0; i < info->num_inputs; i++) {
		nir_alu_src *src = &alu->src[i];

		/* compress swizzle with writemask when applicable */
		unsigned swiz = 0, j = 0;
		for (int i = 0; i < 4; i++) {
			if (!(alu->dest.write_mask & 1 << i) && !info->output_size)
				continue;
			swiz |= swiz_set(src->swizzle[i], j++);
		}

		instr->src_reg[i].num = get_index(ctx, &src->src);
		instr->src_reg[i].swizzle = swiz;
		instr->src_reg[i].flags = (src->negate ? IR2_REG_NEGATE : 0) |
			(src->abs ? IR2_REG_ABS : 0);
	}

	/* workarounds for NIR ops that don't map directly to a2xx ops */
	switch (alu->op) {
	case nir_op_flt:
		tmp = instr->src_reg[0];
		instr->src_reg[0] = instr->src_reg[1];
		instr->src_reg[1] = tmp;
		break;
	case nir_op_fcsel:
	case nir_op_bcsel:
		tmp = instr->src_reg[1];
		instr->src_reg[1] = instr->src_reg[2];
		instr->src_reg[2] = tmp;
		break;
	case nir_op_fsub:
		instr->src_reg[1].flags ^= IR2_REG_NEGATE;
		break;
	case nir_op_fdot2:
		instr->src_reg_count = 3;
		instr->src_reg[2] = ir2_zero();
		break;
	default:
		break;
	}
}

static unsigned output_slot(struct ir2_context *ctx, unsigned idx)
{
	int slot = -1;
	nir_foreach_variable(var, &ctx->so->nir->outputs) {
		if (var->data.driver_location == idx) {
			slot = var->data.location;
			break;
		}
	}
	assert(slot != -1);
	return slot;
}

static unsigned vertex_output_link(struct ir2_context *ctx, unsigned slot)
{
	unsigned i;
	for (i = 0; i < ctx->so->v.fp->f.inputs_count; i++) {
		if (ctx->so->v.fp->f.inputs[i].slot == slot)
			break;
	}
	if (i == ctx->so->v.fp->f.inputs_count)
		return ~0u;
	return i;
}

static void
emit_intrinsic(struct ir2_context *ctx, nir_intrinsic_instr * intr)
{
	struct ir2_instruction *instr;
	nir_const_value *const_offset;
	unsigned idx, slot;

	switch (intr->intrinsic) {
	case nir_intrinsic_load_input:
		idx = nir_intrinsic_base(intr);
		if (ctx->so->type == SHADER_VERTEX) {
			instr = ir2_instr_create_fetch(ctx, &intr->dest, 0);
			instr->src_reg[0] = ir2_src(0, 0, IR2_REG_INPUT);
			instr->fetch.idx = idx;
			instr->fetch.const_idx = 20 + (idx / 3);
			instr->fetch.const_idx_sel = idx % 3;
		} else {
			instr = instr_create_alu_dest(ctx, nir_op_fmov, &intr->dest);
			instr->src_reg[0] = ir2_src(idx, 0, IR2_REG_INPUT);
		}
		break;
	case nir_intrinsic_store_output:
		idx = nir_intrinsic_base(intr);
		slot = output_slot(ctx, idx);
		if (ctx->so->type == SHADER_VERTEX) {
			if (slot == VARYING_SLOT_POS) {
				ctx->position = get_index(ctx, &intr->src[0]);
				idx = 62;
			} else {
				idx = vertex_output_link(ctx, slot);
			}
		} else {
			if (slot == FRAG_RESULT_COLOR || slot == FRAG_RESULT_DATA0) {
				idx = 0;
			} else {
				idx = ~0u;
			}
		}

		if (idx == ~0u)
			break;

		instr = instr_create_alu(ctx, nir_op_fmov, intr->num_components);
		instr->src_reg[0] = ir2_src(get_index(ctx, &intr->src[0]), 0, 0);
		instr->alu.export = idx;
		break;
	case nir_intrinsic_load_uniform:
		const_offset = nir_src_as_const_value(intr->src[0]);
		// XXX how to deal with non-const case
		// use FETCH instruction?
		assert(const_offset);
		idx = nir_intrinsic_base(intr);
		idx += nir_src_as_const_value(intr->src[0])->u32[0];
		instr = instr_create_alu_dest(ctx, nir_op_fmov, &intr->dest);
		instr->src_reg[0] = ir2_src(idx, 0, IR2_REG_CONST);
		break;
	case nir_intrinsic_discard:
	case nir_intrinsic_discard_if:
		instr = ir2_instr_create(ctx, IR2_ALU);
		instr->alu.vector_opc = -1;
		if (intr->intrinsic == nir_intrinsic_discard_if) {
			// can do better if previous instruction is compare
			instr->alu.scalar_opc = KILLNEs;
			instr->src_reg[0] =
				ir2_src(get_index(ctx, &intr->src[0]), 0, 0);
		} else {
			instr->alu.scalar_opc = KILLEs;
			instr->src_reg[0] = ir2_zero();
		}
		instr->alu.export = -1;
		instr->src_reg_count = 1;
		instr->num_components = 0;
		break;
	default:
		compile_error(ctx, "unimplemented intr %d\n", intr->intrinsic);
		break;
	}
}

static struct ir2_src
load_const(struct ir2_context *ctx, uint32_t *value, unsigned ncomp)
{
	struct fd2_shader_stateobj *so = ctx->so;
	unsigned imm_ncomp, swiz, idx, i, j;

	/* try to merge with existing immediate */
	for (idx = 0; idx < so->num_immediates; idx++) {
		swiz = 0;
		imm_ncomp = so->immediates[idx].ncomp;
		for (i = 0; i < ncomp; i++) {
			for (j = 0; j < imm_ncomp; j++) {
				if (value[i] == so->immediates[idx].val[j])
					break;
			}
			if (j == imm_ncomp) {
				if (j == 4)
					break;
				so->immediates[idx].val[imm_ncomp++] = value[i];
			}
			swiz |= swiz_set(j, i);
		}
		/* matched all components */
		if (i == ncomp)
			break;
	}

	/* need to allocate new immediate */
	if (idx == so->num_immediates) {
		for (i = 0; i < ncomp; i++)
			ctx->so->immediates[idx].val[i] = value[i];
		so->num_immediates++;
		swiz = 0;
		imm_ncomp = ncomp;
	}
	so->immediates[idx].ncomp = imm_ncomp;

	return ir2_src(so->first_immediate + idx, swiz, IR2_REG_CONST);
}

static void emit_tex(struct ir2_context *ctx, nir_tex_instr * tex)
{
	bool has_bias = false, has_lod = false, is_rect = false, is_cube =
		false;
	struct ir2_instruction *instr;
	nir_src *coord, *lod;

	coord = lod = NULL;

	for (unsigned i = 0; i < tex->num_srcs; i++) {
		switch (tex->src[i].src_type) {
		case nir_tex_src_coord:
			coord = &tex->src[i].src;
			break;
		case nir_tex_src_bias:
			lod = &tex->src[i].src;
			has_bias = true;
			break;
		case nir_tex_src_lod:
			lod = &tex->src[i].src;
			has_lod = true;
			break;
		default:
			compile_error(ctx, "Unhandled NIR tex src type: %d\n",
						  tex->src[i].src_type);
			return;
		}
	}

	switch (tex->op) {
	case nir_texop_tex:
	case nir_texop_txb:
		break;
	default:
		compile_error(ctx, "unimplemented texop %d\n", tex->op);
		return;
	}

	switch (tex->sampler_dim) {
	case GLSL_SAMPLER_DIM_2D:
		break;
	case GLSL_SAMPLER_DIM_RECT:
		is_rect = true;
		break;
	case GLSL_SAMPLER_DIM_CUBE:
		is_cube = true;
		break;
	default:
		compile_error(ctx, "unimplemented sampler %d\n", tex->sampler_dim);
		return;
	}

	struct ir2_src src_coord = ir2_src(get_index(ctx, coord), 0, 0);
	if (is_cube) {
#define IR2_SWIZZLE_ZZXY ((2 << 0) | (1 << 2) | (2 << 4) | (2 << 6))
#define IR2_SWIZZLE_YXZZ ((1 << 0) | (3 << 2) | (0 << 4) | (3 << 6))
#define IR2_SWIZZLE_XYW ((0 << 0) | (0 << 2) | (1 << 4) | (0 << 6))

		instr = ir2_instr_create(ctx, IR2_ALU);
		instr->alu.vector_opc = CUBEv;
		instr->alu.scalar_opc = -1;
		instr->alu.export = -1;
		instr->alu.write_mask = 15;
		instr->src_reg_count = 2;
		instr->num_components = 4;

		instr->src_reg[0] = src_coord;
		instr->src_reg[0].swizzle = IR2_SWIZZLE_ZZXY;
		instr->src_reg[1] = src_coord;
		instr->src_reg[1].swizzle = IR2_SWIZZLE_YXZZ;

		src_coord = ir2_src(instr->idx, 0, 0);

		instr = instr_create_alu(ctx, nir_op_ffma, 4);
		instr->src_reg[0] = src_coord;
		//XXX multiply by 0.5 shouldn't be needed ??
		float val0[4] = {0.5f, 0.5f, 1.0f, 1.0f};
		instr->src_reg[1] = load_const(ctx, (uint32_t*) val0, 4);
		float val[4] = {1.5f, 1.5f, 0.0f, 0.0f};
		instr->src_reg[2] = load_const(ctx, (uint32_t*) val, 4);

		src_coord = ir2_src(instr->idx, 0, 0);
	}

	// XXX use lod/bias
	// what values go into: use_reg_lod, use_comp_lod (computed), lod_bias?
	assert(tex->texture_index == tex->sampler_index);	// whats the difference?

	instr = ir2_instr_create_fetch(ctx, &tex->dest, 1);
	instr->src_reg[0] = src_coord;
	instr->src_reg[0].swizzle = !is_cube ? IR2_SWIZZLE_XYXY : IR2_SWIZZLE_XYW;
	instr->fetch.is_cube = is_cube;
	instr->fetch.is_rect = is_rect;
	instr->fetch.samp_id = tex->sampler_index;
}

static void setup_input(struct ir2_context *ctx, nir_variable * in)
{
	struct fd2_shader_stateobj *so = ctx->so;
	unsigned array_len = MAX2(glsl_get_length(in->type), 1);
	unsigned n = in->data.driver_location;
	unsigned slot = in->data.location;

	assert(array_len == 1);

	/* handle later */
	if (ctx->so->type == SHADER_VERTEX)
		return;

	if (ctx->so->type != SHADER_FRAGMENT)
		compile_error(ctx, "unknown shader type: %d\n", ctx->so->type);

	if (slot == VARYING_SLOT_PNTC)
		compile_error(ctx, "gl_PointCoord not implemented");

	if (slot == VARYING_SLOT_POS)
		so->f.frag_coord = n;

	so->f.inputs[n].slot = slot;
	so->f.inputs[n].ncomp = glsl_get_components(in->type);
	so->f.inputs_count = MAX2(so->f.inputs_count, n + 1);
	/* in->data.interpolation?
	 * opengl ES 2.0 can't do flat mode, but we still get it from GALLIUM_HUD
	 */
}

static void
emit_load_const(struct ir2_context *ctx, nir_load_const_instr * instr)
{
	unsigned ncomp = instr->def.num_components, i;
	uint32_t value[4];

	/* XXX hacks - translate NIR_TRUE to 1.0f,
	 * and values that look like integers to float
	 */
	for (i = 0; i < ncomp; i++) {
		value[i] = instr->value.u32[i];
		if (instr->value.u32[i] == NIR_TRUE)
			value[i] = fui(1.0f);
		else if (instr->value.i32[i] >= -4096 && instr->value.i32[i] < 4096)
			value[i] = fui((float) instr->value.i32[i]);
	}

	/* mov instruction to load const */
	struct ir2_instruction *mov;
	mov = instr_create_alu_dest(ctx, nir_op_fmov,
		&(nir_dest) {.ssa = instr->def,.is_ssa = true});
	mov->src_reg[0] = load_const(ctx, value, ncomp);
}

static void emit_instr(struct ir2_context *ctx, nir_instr * instr)
{
	switch (instr->type) {
	case nir_instr_type_alu:
		emit_alu(ctx, nir_instr_as_alu(instr));
		break;
	case nir_instr_type_deref:
		/* ignored, handled as part of the intrinsic they are src to */
		break;
	case nir_instr_type_intrinsic:
		emit_intrinsic(ctx, nir_instr_as_intrinsic(instr));
		break;
	case nir_instr_type_load_const:
		emit_load_const(ctx, nir_instr_as_load_const(instr));
		break;
	case nir_instr_type_tex:
		emit_tex(ctx, nir_instr_as_tex(instr));
		break;
		// case nir_instr_type_ssa_undef: // XXX should be implemented
	default:
		break;
	}
}

static void vertex_shader_epilog(struct ir2_context *ctx)
{
	struct fd2_shader_stateobj *fp = ctx->so->v.fp;
	struct ir2_instruction *instr, *rcp, *sc, *wc, *off;

	rcp = instr_create_alu(ctx, nir_op_frcp, 1);
	rcp->src_reg[0] = ir2_src(ctx->position, IR2_SWIZZLE_WWWW, 0);

	sc = instr_create_alu(ctx, nir_op_fmul, 4);
	sc->src_reg[0] = ir2_src(ctx->position, 0, 0);
	sc->src_reg[1] = ir2_src(rcp->idx, IR2_SWIZZLE_XXXX, 0);

	if (fp->f.frag_coord >= 0) {
		instr = instr_create_alu(ctx, nir_op_ffma, 4);
		instr->src_reg[0] = ir2_src(66, 0, IR2_REG_CONST);
		instr->src_reg[1] = ir2_src(sc->idx, 0, 0);
		instr->src_reg[2] = ir2_src(65, 0, IR2_REG_CONST);
		instr->alu.export = fp->f.frag_coord;
		/*
		   // XXX: w component
		   instr = instr_create_alu(ctx, nir_op_imov, IR2_WRITE_W, -1);
		   instr->src_reg[0] = ir2_src(rcp->idx, IR2_SWIZZLE_XXXX, 0);
		   instr->src_reg[1] = ir2_src(rcp->idx, IR2_SWIZZLE_XXXX, 0); */
	}

	/* these two instructions could be avoided with constant folding
	 * but it would be hard to implement..
	 */
	wc = instr_create_alu(ctx, nir_op_ffma, 4);
	wc->src_reg[0] = ir2_src(66, 0, IR2_REG_CONST);
	wc->src_reg[1] = ir2_src(sc->idx, 0, 0);
	wc->src_reg[2] = ir2_src(65, 0, IR2_REG_CONST);

	off = instr_create_alu(ctx, nir_op_fadd, 1);
	off->src_reg[0] = ir2_src(64, 0, IR2_REG_CONST);
	off->src_reg[1] = ir2_src(2, 0, IR2_REG_INPUT);

	/* 8 max set in freedreno_screen.. unneeded instrs patched out */
	for (int i = 0; i < 8; i++) {
		instr = instr_create_alu(ctx, nir_op_ffma, 4);
		instr->src_reg[0] = ir2_src(1, IR2_SWIZZLE_WYWW, IR2_REG_CONST);
		instr->src_reg[1] = ir2_src(off->idx, IR2_SWIZZLE_XXXX, 0);
		instr->src_reg[2] = ir2_src(3 + i, 0, IR2_REG_CONST);
		instr->alu.export = 32;

		instr = instr_create_alu(ctx, nir_op_ffma, 4);
		instr->src_reg[0] = ir2_src(68 + i * 2, 0, IR2_REG_CONST);
		instr->src_reg[1] = ir2_src(wc->idx, 0, 0);
		instr->src_reg[2] = ir2_src(67 + i * 2, 0, IR2_REG_CONST);
		instr->alu.export = 33;
	}
}

void ir2_nir_compile(struct ir2_context *ctx)
{
	memset(ctx->ssa_map, 0xff, sizeof(ctx->ssa_map));
	memset(ctx->reg_map, 0xff, sizeof(ctx->reg_map));
	ctx->position = -1;

	/* fd2_shader_stateobj init */
	if (ctx->so->type == SHADER_FRAGMENT) {
		ctx->so->f.frag_coord = -1;
		ctx->so->f.inputs_count = 0;
		memset(ctx->so->f.inputs, 0, sizeof(ctx->so->f.inputs));
	}

	ctx->so->first_immediate = ctx->so->nir->num_uniforms;
	ctx->so->num_immediates = 0;

	/* Setup inputs: */
	nir_foreach_variable(in, &ctx->so->nir->inputs)
		setup_input(ctx, in);

	/* And emit the body: */
	nir_foreach_block(block, nir_shader_get_entrypoint(ctx->so->nir)) {
		nir_foreach_instr(instr, block)
			emit_instr(ctx, instr);
	}

	/* For vertex shaders, fragcoord / a20x hw binning instrs */
	if (ctx->so->type == SHADER_VERTEX)
		vertex_shader_epilog(ctx);
}
