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
	.lower_fceil = true,
	.fuse_ffma = true,
	.vertex_id_zero_based = true,

	// XXX what does this change?
	// .fdot_replicates = true,

	// XXX
	.lower_extract_byte = true,
	.lower_extract_word = true,
	.lower_helper_invocation = true,
	.lower_all_io_to_temps = true,
};

struct nir_shader *
ir2_tgsi_to_nir(const struct tgsi_token *tokens)
{
	return tgsi_to_nir(tokens, &options);
}

const nir_shader_compiler_options *
ir2_get_compiler_options(void)
{
	return &options;
}

#define OPT(nir, pass, ...) ({                             \
   bool this_progress = false;                             \
   NIR_PASS(this_progress, nir, pass, ##__VA_ARGS__);      \
   this_progress;                                          \
})
#define OPT_V(nir, pass, ...) NIR_PASS_V(nir, pass, ##__VA_ARGS__)

static void
ir2_optimize_loop(nir_shader * s)
{
	bool progress;
	do {
		progress = false;

		OPT_V(s, nir_lower_vars_to_ssa);
		progress |= OPT(s, nir_opt_copy_prop_vars);
		progress |= OPT(s, nir_copy_prop);
		progress |= OPT(s, nir_opt_dce);
		progress |= OPT(s, nir_opt_cse);
		/* progress |= OPT(s, nir_opt_gcm, true); */
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

	}
	while (progress);
}

/* trig workarounds is the same as ir3.. but we don't want to include ir3 */
bool ir3_nir_apply_trig_workarounds(nir_shader * shader);

int
ir2_optimize_nir(nir_shader *s, bool lower)
{
	struct nir_lower_tex_options tex_options = {
		.lower_txp = ~0u,
		.lower_rect = 0,
	};

	if (fd_mesa_debug & FD_DBG_DISASM & 0) {
		debug_printf("----------------------\n");
		nir_print_shader(s, stdout);
		debug_printf("----------------------\n");
	}

	OPT_V(s, nir_opt_global_to_local);
	OPT_V(s, nir_lower_regs_to_ssa);
	OPT_V(s, nir_lower_vars_to_ssa);

	if (lower) {
		OPT_V(s, ir3_nir_apply_trig_workarounds);
		OPT_V(s, nir_lower_tex, &tex_options);
	}

	ir2_optimize_loop(s);

	OPT_V(s, nir_remove_dead_variables, nir_var_local);
	OPT_V(s, nir_move_load_const);

	/* TODO we dont want to get shaders writing to depth for depth textures */
	if (s->info.stage == MESA_SHADER_FRAGMENT) {
		nir_foreach_variable(var, &s->outputs) {
			if (var->data.location == FRAG_RESULT_DEPTH)
				return -1;
		}
	}

	return 0;
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
		swiz = 0;
		imm_ncomp = 0;
		for (i = 0; i < ncomp; i++) {
			for (j = 0; j < imm_ncomp; j++) {
				if (value[i] == ctx->so->immediates[idx].val[j])
                    break;
			}
			if (j == imm_ncomp) {
				so->immediates[idx].val[imm_ncomp++] = value[i];
			}
			swiz |= swiz_set(j, i);
		}
		so->num_immediates++;
	}
	so->immediates[idx].ncomp = imm_ncomp;

	return ir2_src(so->first_immediate + idx, swiz, IR2_SRC_CONST);
}

static void
update_range(struct ir2_context *ctx, struct ir2_reg *reg)
{
	if (!reg->initialized) {
		reg->initialized = true;
        reg->loop_depth = ctx->loop_depth;
	}

	if (ctx->loop_depth > reg->loop_depth) {
		reg->block_idx_free = ctx->loop_last_block[reg->loop_depth + 1];
	} else {
		reg->loop_depth = ctx->loop_depth;
		reg->block_idx_free = -1;
	}

	/* for regs we want to free at the end of the loop in any case
	 * XXX dont do this for ssa
	 */
	if (reg->loop_depth)
		reg->block_idx_free = ctx->loop_last_block[reg->loop_depth];
}

static struct ir2_src
make_src(struct ir2_context *ctx, nir_src src, bool is_bool)
{
	struct ir2_src res = {};
	struct ir2_reg *reg;

	nir_const_value *const_value = nir_src_as_const_value(src);

	if (const_value) {
		uint32_t value[4];

		assert(src.is_ssa);

		/* workaround: detect integers and convert to the right value */
		for (int i = 0; i < src.ssa->num_components; i++) {
			value[i] = const_value->u32[i];
			if (const_value->u32[i] == NIR_TRUE && is_bool)
				value[i] = fui(1.0f);
			else if (const_value->i32[i] >= -4096 && const_value->i32[i] < 4096)
				value[i] = fui((float) const_value->i32[i]);
		}

		return load_const(ctx, value, src.ssa->num_components);
	}

	if (!src.is_ssa) {
		res.num = src.reg.reg->index;
		res.type = IR2_SRC_REG;
		reg = &ctx->reg[res.num];
	} else {
		assert(ctx->ssa_map[src.ssa->index] >= 0);
		res.num = ctx->ssa_map[src.ssa->index];
		res.type = IR2_SRC_SSA;
		reg = &ctx->instr[res.num].ssa;
	}

	update_range(ctx, reg);
	return res;
}

static void
set_index(struct ir2_context *ctx, nir_dest * dst,
		  struct ir2_instr *instr)
{
	struct ir2_reg *reg = &instr->ssa;

	if (dst->is_ssa) {
		ctx->ssa_map[dst->ssa.index] = instr->idx;
	} else {
		assert(instr->is_ssa);
		instr->reg_ncomp = reg->ncomp;

		reg = &ctx->reg[dst->reg.reg->index];

		instr->is_ssa = false;
		instr->reg = reg;
	}
    update_range(ctx, reg);
}

static struct ir2_instr *
ir2_instr_create(struct ir2_context *ctx, int type)
{
	struct ir2_instr *instr;

	instr = &ctx->instr[ctx->instr_count++];
	instr->idx = ctx->instr_count - 1;
	instr->type = type;
	instr->block_idx = ctx->block_idx;
	instr->pred = ctx->pred;
	instr->is_ssa = true;
	return instr;
}

static struct ir2_instr *
instr_create_alu(struct ir2_context *ctx, nir_op opcode, unsigned ncomp)
{
	static const struct ir2_opc {
		int8_t scalar, vector;
	} nir_ir2_opc[nir_num_opcodes] = {
		[0 ... nir_num_opcodes - 1] = {-1, -1},

		[nir_op_fmov] = {MAXs, MAXv},
		[nir_op_fsign] = {-1, CNDGTEv},
		[nir_op_fnot] = {SETEs, SETEv},
		[nir_op_f2b] = {SETNEs, SETNEv},
		[nir_op_for] = {MAXs, MAXv},
		[nir_op_fand] = {MINs, MINv},
		[nir_op_fxor] = {-1, SETNEv},
		[nir_op_fadd] = {ADDs, ADDv},
		[nir_op_fsub] = {ADDs, ADDv},
		[nir_op_fmul] = {MULs, MULv},
		[nir_op_ffma] = {-1, MULADDv},
		[nir_op_fmax] = {MAXs, MAXv},
		[nir_op_fmin] = {MINs, MINv},
		[nir_op_ffloor] = {FLOORs, FLOORv},
		[nir_op_ffract] = {FRACs, FRACv},
		[nir_op_ftrunc] = {TRUNCs, TRUNCv},
		[nir_op_fdot2] = {-1, DOT2ADDv},
		[nir_op_fdot3] = {-1, DOT3v},
		[nir_op_fdot4] = {-1, DOT4v},
		[nir_op_fge] = {-1, SETGTEv},
		[nir_op_flt] = {-1, SETGTv},
		[nir_op_fne] = {-1, SETNEv},
		[nir_op_feq] = {-1, SETEv},
		[nir_op_fcsel] = {-1, CNDEv},
		[nir_op_frsq] = {RECIPSQ_IEEE, -1},
		[nir_op_frcp] = {RECIP_IEEE, -1},
		[nir_op_flog2] = {LOG_IEEE, -1},
		[nir_op_fexp2] = {EXP_IEEE, -1},
		[nir_op_fsqrt] = {SQRT_IEEE, -1},
		[nir_op_fcos] = {COS, -1},
		[nir_op_fsin] = {SIN, -1},

		[nir_op_b2f] = {MAXs, MAXv},

		/* XXX these should never happen with native_integers=false? */
		[nir_op_imov] = {MAXs, MAXv},
		[nir_op_inot] = {SETEs, SETEv},
		[nir_op_ior] = {MAXs, MAXv},
		[nir_op_iand] = {MINs, MINv},
		[nir_op_bcsel] = {-1, CNDEv},
	};

	struct ir2_opc op = nir_ir2_opc[opcode];
	assert(op.vector >= 0 || op.scalar >= 0);

	struct ir2_instr *instr = ir2_instr_create(ctx, IR2_ALU);
	instr->alu.vector_opc = op.vector;
	instr->alu.scalar_opc = op.scalar;
	instr->alu.export = -1;
	instr->alu.write_mask = (1 << ncomp) - 1;
	instr->src_count = nir_op_infos[opcode].num_inputs;
	instr->ssa.ncomp = ncomp;
	return instr;
}

static struct ir2_instr *
instr_create_alu_dest(struct ir2_context *ctx, nir_op opcode, nir_dest *dst)
{
	struct ir2_instr *instr;
	instr = instr_create_alu(ctx, opcode, nir_dest_num_components(*dst));
	set_index(ctx, dst, instr);
	return instr;
}

static struct ir2_instr *
ir2_instr_create_fetch(struct ir2_context *ctx, nir_dest *dst,
		instr_fetch_opc_t opc)
{
	struct ir2_instr *instr = ir2_instr_create(ctx, IR2_FETCH);
	instr->fetch.opc = opc;
	instr->src_count = 1;
	instr->ssa.ncomp = nir_dest_num_components(*dst);
	set_index(ctx, dst, instr);
	return instr;
}

static struct ir2_src
make_src_noconst(struct ir2_context *ctx, nir_src src)
{
	struct ir2_instr *instr;

	if (nir_src_as_const_value(src)) {
		assert(src.is_ssa);
        instr = instr_create_alu(ctx, nir_op_fmov, src.ssa->num_components);
        instr->src[0] = make_src(ctx, src, false);
        return ir2_src(instr->idx, 0, IR2_SRC_SSA);
	}

	return make_src(ctx, src, false);
}

static void
emit_alu(struct ir2_context *ctx, nir_alu_instr * alu)
{
	const nir_op_info *info = &nir_op_infos[alu->op];
	nir_dest *dst = &alu->dest.dest;
	struct ir2_instr *instr;
	struct ir2_src tmp;
	unsigned ncomp;

	assert(info->output_size <= 1);

	/* get the number of dst components */
	if (dst->is_ssa) {
		ncomp = dst->ssa.num_components;
	} else {
		ncomp = 0;
		for (int i = 0; i < 4; i++)
			ncomp += !!(alu->dest.write_mask & 1 << i);
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

		instr->src[i] = make_src(ctx, src->src, info->input_types[i] == nir_type_bool);
		instr->src[i].swizzle = swiz_merge(instr->src[i].swizzle, swiz);
		instr->src[i].negate = src->negate;
		instr->src[i].abs = src->abs;
	}

	/* workarounds for NIR ops that don't map directly to a2xx ops */
	switch (alu->op) {
	case nir_op_flt:
		tmp = instr->src[0];
		instr->src[0] = instr->src[1];
		instr->src[1] = tmp;
		break;
	case nir_op_fcsel:
	case nir_op_bcsel:
		tmp = instr->src[1];
		instr->src[1] = instr->src[2];
		instr->src[2] = tmp;
		break;
	case nir_op_fsub:
		instr->src[1].negate = !instr->src[1].negate;
		break;
	case nir_op_fdot2:
		instr->src_count = 3;
		instr->src[2] = ir2_zero();
		break;
	case nir_op_fsign: {
		/* annoyingly, we need an extra instruction to handle the zero case
		 * maybe the blob has a smarter method? */
		struct ir2_instr *zero_one;

		zero_one = instr_create_alu(ctx, nir_op_fcsel, ncomp);
		zero_one->src[0] = instr->src[0];
		zero_one->src[1] = ir2_zero();
        zero_one->src[2] = load_const(ctx, (uint32_t[]) {fui(1.0f)}, 1);
        zero_one->src[2].swizzle = swiz_merge(instr->src[2].swizzle, IR2_SWIZZLE_XXXX);

        instr->src[1] = ir2_src(zero_one->idx, 0, IR2_SRC_SSA);
        instr->src[2] = instr->src[1];
        instr->src[2].negate = true;
		instr->src_count = 3;
	} break;
	default:
		break;
	}
}

static unsigned
input_slot(struct ir2_context *ctx, unsigned idx)
{
	int slot = -1;
	nir_foreach_variable(var, &ctx->nir->inputs) {
		if (var->data.driver_location == idx) {
			slot = var->data.location;
			break;
		}
	}
	assert(slot != -1);
	return slot;
}

static unsigned
output_slot(struct ir2_context *ctx, unsigned idx)
{
	int slot = -1;
	nir_foreach_variable(var, &ctx->nir->outputs) {
		if (var->data.driver_location == idx) {
			slot = var->data.location;
			break;
		}
	}
	assert(slot != -1);
	return slot;
}

static unsigned
vertex_output_link(struct ir2_context *ctx, unsigned slot)
{
	unsigned i;
	for (i = 0; i < ctx->so->f.inputs_count; i++) {
		if (ctx->so->f.inputs[i].slot == slot)
			break;
	}
	if (i == ctx->so->f.inputs_count)
		return ~0u;
	return i;
}

static void
emit_intrinsic(struct ir2_context *ctx, nir_intrinsic_instr * intr)
{
	struct ir2_instr *instr;
	nir_const_value *const_offset;
	unsigned idx, slot;

	switch (intr->intrinsic) {
	case nir_intrinsic_load_input:
		idx = nir_intrinsic_base(intr);
		if (ctx->so->type == SHADER_VERTEX) {
			instr = ir2_instr_create_fetch(ctx, &intr->dest, 0);
			instr->src[0] = ir2_src(0, 0, IR2_SRC_INPUT);
			instr->fetch.vtx.const_idx = 20 + (idx / 3);
			instr->fetch.vtx.const_idx_sel = idx % 3;
		} else {
			slot = input_slot(ctx, idx);
			if (slot == VARYING_SLOT_PNTC) {
				/* need to invert y */
				instr = instr_create_alu_dest(ctx, nir_op_ffma, &intr->dest);
				instr->src[0] = ir2_src(ctx->so->f.inputs_count, IR2_SWIZZLE_ZW, IR2_SRC_INPUT);
				instr->src[0].abs = true;
				instr->src[1] = load_const(ctx, (uint32_t[]) {fui(1.0f), fui(-1.0f)}, 2);
				instr->src[2] = load_const(ctx, (uint32_t[]) {fui(0.0f), fui(1.0f)}, 2);
			} else if (slot == VARYING_SLOT_POS) {
				/* TODO only components that are required by fs */
				unsigned reg_idx = ctx->reg_count++;
				struct ir2_reg *reg = &ctx->reg[reg_idx];

				reg->ncomp = 4;

				instr = instr_create_alu(ctx, nir_op_ffma, 2);
				instr->src[0] = ir2_src(ctx->so->f.inputs_count, 0, IR2_SRC_INPUT);
				instr->src[0].abs = true;
				instr->src[1] = ir2_src(64, 0, IR2_SRC_CONST);
				instr->src[2] = ir2_src(64, IR2_SWIZZLE_ZW, IR2_SRC_CONST);
				instr->is_ssa = false;
				instr->reg = reg;
				instr->reg_ncomp = 2;

				instr = instr_create_alu(ctx, nir_op_fmov, 1);
				instr->src[0] = ir2_src(ctx->so->f.fragcoord, 0, IR2_SRC_INPUT);
				instr->is_ssa = false;
				instr->reg = reg;
				instr->reg_ncomp = 1;
				instr->alu.write_mask = 0x4;

				instr = instr_create_alu(ctx, nir_op_frcp, 1);
				instr->src[0] = ir2_src(ctx->so->f.fragcoord, IR2_SWIZZLE_Y, IR2_SRC_INPUT);
				instr->is_ssa = false;
				instr->reg = reg;
				instr->reg_ncomp = 1;
				instr->alu.write_mask = 0x8;

				instr = instr_create_alu_dest(ctx, nir_op_fmov, &intr->dest);
				instr->src[0] = ir2_src(reg_idx, 0, IR2_SRC_REG);
			} else {
				for (idx = 0; idx < ctx->so->f.inputs_count; idx++)
					if (ctx->so->f.inputs[idx].slot == slot)
						break;

				assert(idx < ctx->so->f.inputs_count);
				instr = instr_create_alu_dest(ctx, nir_op_fmov, &intr->dest);
				instr->src[0] = ir2_src(idx, 0, IR2_SRC_INPUT);
			}
		}
		break;
	case nir_intrinsic_store_output:
		idx = nir_intrinsic_base(intr);
		slot = output_slot(ctx, idx);
		if (ctx->so->type == SHADER_VERTEX) {
			if (slot == VARYING_SLOT_POS) {
				ctx->position = make_src(ctx, intr->src[0], false);
				idx = 62;
			} else if (slot == VARYING_SLOT_PSIZ) {
				idx = 63;
				ctx->so->writes_psize = true;
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
		instr->src[0] = make_src(ctx, intr->src[0], false);
		instr->alu.export = idx;
		break;
	case nir_intrinsic_load_uniform:
		const_offset = nir_src_as_const_value(intr->src[0]);
		assert(const_offset); /* TODO can be false in ES2? */
		idx = nir_intrinsic_base(intr);
		idx += nir_src_as_const_value(intr->src[0])->u32[0];
		instr = instr_create_alu_dest(ctx, nir_op_fmov, &intr->dest);
		instr->src[0] = ir2_src(idx, 0, IR2_SRC_CONST);
		break;
	case nir_intrinsic_discard:
	case nir_intrinsic_discard_if:
		instr = ir2_instr_create(ctx, IR2_ALU);
		instr->alu.vector_opc = VECTOR_NONE;
		if (intr->intrinsic == nir_intrinsic_discard_if) {
			instr->alu.scalar_opc = KILLNEs;
			instr->src[0] = make_src(ctx, intr->src[0], false);
		} else {
			instr->alu.scalar_opc = KILLEs;
			instr->src[0] = ir2_zero();
		}
		instr->alu.export = -1;
		instr->src_count = 1;
		break;
	case nir_intrinsic_load_front_face:
		/* TODO
	      ALU:	MAXv	R0.____ = R0, R0
		    	RECIP_CLAMP	R3.x___ = R2.xyzx
	      ALU:	CNDGTEv	R3.x___ = C30.yyzw, R3, C30
	      ALU:	SETGTv	R4.x___ = R3, C0
	      ALU:	SETGTv	R3.x___ = C0, R3
	      ALU:	ADDv	R3.x___ = R4, -R3 CLAMP
	      ALU:	SETEv	R3.___w = R3.xyzx, C0
	EXEC ADDR(0x9) CNT(0x1)
	      ALU:	MAXv	R0.____ = R0, R0
		    	PRED_SETEs	R0.____ = R3
		 */
		ctx->so->need_param = true;
		instr = instr_create_alu_dest(ctx, nir_op_fge, &intr->dest);
		instr->src[0] = ir2_src(ctx->so->f.inputs_count, 0, IR2_SRC_INPUT);
		instr->src[1] = ir2_zero();
		break;
	default:
		compile_error(ctx, "unimplemented intr %d\n", intr->intrinsic);
		break;
	}
}

static void
emit_tex(struct ir2_context *ctx, nir_tex_instr * tex)
{
	bool is_rect = false, is_cube = false;
	struct ir2_instr *instr;
	nir_src *coord, *lod_bias;

	coord = lod_bias = NULL;

	for (unsigned i = 0; i < tex->num_srcs; i++) {
		switch (tex->src[i].src_type) {
		case nir_tex_src_coord:
			coord = &tex->src[i].src;
			break;
		case nir_tex_src_bias:
		case nir_tex_src_lod:
			assert(!lod_bias);
			lod_bias = &tex->src[i].src;
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
	case nir_texop_txl:
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

	struct ir2_src src_coord = make_src_noconst(ctx, *coord);

	if (is_cube) {
		unsigned reg_idx = ctx->reg_count++;
		struct ir2_reg *reg = &ctx->reg[reg_idx];

		reg->ncomp = 4;

		instr = ir2_instr_create(ctx, IR2_ALU);
		instr->alu.vector_opc = CUBEv;
		instr->alu.scalar_opc = SCALAR_NONE;
		instr->alu.export = -1;
		instr->alu.write_mask = 15;
		instr->src_count = 2;

		instr->reg = reg;
		instr->reg_ncomp = 4;
		instr->is_ssa = false;

		instr->src[0] = src_coord;
		instr->src[0].swizzle = IR2_SWIZZLE_ZZXY;
		instr->src[1] = src_coord;
		instr->src[1].swizzle = IR2_SWIZZLE_YXZZ;

		struct ir2_instr *rcp = instr_create_alu(ctx, nir_op_frcp, 1);
		rcp->src[0] = ir2_src(reg_idx, IR2_SWIZZLE_Z, IR2_SRC_REG);
		rcp->src[0].abs = true;

		struct ir2_instr *coord = instr_create_alu(ctx, nir_op_ffma, 2);
		coord->src[0] = ir2_src(reg_idx, 0, IR2_SRC_REG);
		coord->src[1] = ir2_src(rcp->idx, IR2_SWIZZLE_XXXX, IR2_SRC_SSA);
		coord->src[2] = load_const(ctx, (uint32_t[]) {fui(1.5f)}, 1);
		swiz_merge_p(&coord->src[2].swizzle, IR2_SWIZZLE_XXXX);

		coord->reg = reg;
		coord->reg_ncomp = 2;
		coord->is_ssa = false;

		src_coord = ir2_src(reg_idx, 0, IR2_SRC_REG);

		/* TODO: for LOD in cubemaps, we need to use an extra instruction
		 * FETCH:	SET_TEX_LOD	R0.____ = R0.www CONST(0) LOCATION(CENTER) */
	}


#if 0
	unsigned idx = 0;
	if (lod_bias) {
		instr = ir2_instr_create(ctx, IR2_FETCH);
		instr->fetch.opc = TEX_SET_TEX_LOD;
		instr->src[0] = make_src_noconst(ctx, *lod_bias);
		instr->src_count = 1;
		/* actually 0, but so we can hack this as a 2nd src to FETCH */
		instr->is_ssa = true;
		instr->ssa.ncomp = 2 + is_cube;
		idx = instr->idx;
	}
#endif

#if 0
	if (lod_bias) {
		unsigned reg_idx = ctx->reg_count++;
		struct ir2_reg *reg = &ctx->reg[reg_idx];

		reg->ncomp = 4;

		instr = instr_create_alu(ctx, nir_op_fmov, 2 + is_cube);
		instr->src[0] = make_src(ctx, *coord, false);
		instr->alu.write_mask = 3 | (is_cube ? 4 : 0);
		instr->reg = reg;
		instr->reg_ncomp = 2 + is_cube;
		instr->is_ssa = false;

		instr = instr_create_alu(ctx, nir_op_fmov, 1);
		instr->src[0] = make_src(ctx, *lod_bias, false);
		instr->alu.write_mask = 4;
		instr->reg = reg;
		instr->reg_ncomp = 1;
		instr->is_ssa = false;

		src_coord = ir2_src(reg_idx, 0, IR2_SRC_REG);

		instr = ir2_instr_create(ctx, IR2_FETCH);
		instr->fetch.opc = TEX_SET_TEX_LOD;
		instr->src[0] = ir2_src(reg_idx, IR2_SWIZZLE_ZZZZ, IR2_SRC_REG);
		instr->src_count = 1;
		/* actually 0, but so we can hack this as a 2nd src to FETCH */
		instr->is_ssa = true;
		instr->ssa.ncomp = 2 + is_cube;
		idx = instr->idx;
	}
#endif

	assert(tex->texture_index == tex->sampler_index);

	instr = ir2_instr_create_fetch(ctx, &tex->dest, TEX_FETCH);
	instr->src[0] = src_coord;
	instr->src[0].swizzle = is_cube ? IR2_SWIZZLE_XYW : 0;
	instr->fetch.tex.is_cube = is_cube;
	instr->fetch.tex.is_rect = is_rect;
	instr->fetch.tex.samp_id = tex->sampler_index;

	if (lod_bias) {
		//instr->src[1] = ir2_src(idx, 0, IR2_SRC_SSA);
		instr->src[1] = make_src_noconst(ctx, *lod_bias);
		swiz_merge_p(&instr->src[1].swizzle, IR2_SWIZZLE_XXXX);
		instr->src_count = 2;
	}
}

static void
setup_input(struct ir2_context *ctx, nir_variable * in)
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

	if (slot == VARYING_SLOT_PNTC) {
        so->need_param = true;
		return;
	}

	n = so->f.inputs_count++;

	/* half of fragcoord from param reg, half from a varying */
	if (slot == VARYING_SLOT_POS) {
		so->f.fragcoord = n;
		so->need_param = true;
	}

	so->f.inputs[n].slot = slot;
	so->f.inputs[n].ncomp = glsl_get_components(in->type);

	/* in->data.interpolation?
	 * opengl ES 2.0 can't do flat mode, but we still get it from GALLIUM_HUD
	 */
}

static void
emit_undef(struct ir2_context *ctx, nir_ssa_undef_instr * undef)
{
	/* TODO we don't want to emit anything for undefs */

	struct ir2_instr *instr;

	instr = instr_create_alu_dest(ctx, nir_op_fmov,
		&(nir_dest) {.ssa = undef->def,.is_ssa = true});
	instr->src[0] = ir2_src(0, 0, IR2_SRC_CONST);
}

static void
emit_instr(struct ir2_context *ctx, nir_instr * instr)
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
		/* dealt with when using nir_src */
		break;
	case nir_instr_type_tex:
		emit_tex(ctx, nir_instr_as_tex(instr));
		break;
	case nir_instr_type_jump:
		ctx->block_has_jump[ctx->block_idx] = true;
		break;
	case nir_instr_type_ssa_undef:
		emit_undef(ctx, nir_instr_as_ssa_undef(instr));
		break;
	default:
		break;
	}
}

/* fragcoord.zw and a20x hw binning outputs */
static void
extra_position_exports(struct ir2_context *ctx, unsigned variant)
{
	struct ir2_instr *instr, *rcp, *sc, *wincoord, *off;

	if (ctx->so->f.fragcoord < 0 && !variant)
		return;

	instr = instr_create_alu(ctx, nir_op_fmax, 1);
	instr->src[0] = ctx->position;
	instr->src[0].swizzle = IR2_SWIZZLE_W;
	instr->src[1] = ir2_zero();

	rcp = instr_create_alu(ctx, nir_op_frcp, 1);
	rcp->src[0] = ir2_src(instr->idx, 0, IR2_SRC_SSA);

	sc = instr_create_alu(ctx, nir_op_fmul, 4);
	sc->src[0] = ctx->position;
	sc->src[1] = ir2_src(rcp->idx, IR2_SWIZZLE_XXXX, IR2_SRC_SSA);

	wincoord = instr_create_alu(ctx, nir_op_ffma, 4);
	wincoord->src[0] = ir2_src(66, 0, IR2_SRC_CONST);
	wincoord->src[1] = ir2_src(sc->idx, 0, IR2_SRC_SSA);
	wincoord->src[2] = ir2_src(65, 0, IR2_SRC_CONST);

	/* fragcoord z/w */
	if (ctx->so->f.fragcoord >= 0 && !variant) {
		instr = instr_create_alu(ctx, nir_op_fmov, 1);
		instr->src[0] = ir2_src(wincoord->idx, IR2_SWIZZLE_Z, IR2_SRC_SSA);
		instr->alu.export = ctx->so->f.fragcoord;

		instr = instr_create_alu(ctx, nir_op_fmov, 1);
		instr->src[0] = ctx->position;
		instr->src[0].swizzle = IR2_SWIZZLE_W;
		instr->alu.export = ctx->so->f.fragcoord;
		instr->alu.write_mask = 2;
	}

	if (!variant)
		return;

	off = instr_create_alu(ctx, nir_op_fadd, 1);
	off->src[0] = ir2_src(64, 0, IR2_SRC_CONST);
	off->src[1] = ir2_src(2, 0, IR2_SRC_INPUT);

	/* 8 max set in freedreno_screen.. unneeded instrs patched out */
	for (int i = 0; i < 8; i++) {
		instr = instr_create_alu(ctx, nir_op_ffma, 4);
		instr->src[0] = ir2_src(1, IR2_SWIZZLE_WYWW, IR2_SRC_CONST);
		instr->src[1] = ir2_src(off->idx, IR2_SWIZZLE_XXXX, IR2_SRC_SSA);
		instr->src[2] = ir2_src(3 + i, 0, IR2_SRC_CONST);
		instr->alu.export = 32;

		instr = instr_create_alu(ctx, nir_op_ffma, 4);
		instr->src[0] = ir2_src(68 + i * 2, 0, IR2_SRC_CONST);
		instr->src[1] = ir2_src(wincoord->idx, 0, IR2_SRC_SSA);
		instr->src[2] = ir2_src(67 + i * 2, 0, IR2_SRC_CONST);
		instr->alu.export = 33;
	}
}

static bool emit_cf_list(struct ir2_context *ctx, struct exec_list *list);

static bool
emit_block(struct ir2_context *ctx, nir_block * block)
{
	struct ir2_instr *instr;
	nir_block *succs = block->successors[0];

	ctx->block_idx = block->index;

	nir_foreach_instr(instr, block)
		emit_instr(ctx, instr);

	if (!succs || !succs->index)
		return false;

	/* we want to be smart and always jump and have the backend cleanup
	 * but we are not, so there are two cases where jump is needed:
	 *  loops (succs index lower)
	 *  jumps (jump instruction seen in block)
	 */
	if (succs->index > block->index && !ctx->block_has_jump[block->index])
		return false;

	assert(block->successors[1] == NULL);

	instr = ir2_instr_create(ctx, IR2_CF);
	instr->cf.block_idx = succs->index;
	/* XXX we can't jump to a predicated block */
	return true;
}

static void
emit_if(struct ir2_context *ctx, nir_if * nif)
{
	struct ir2_instr *instr;
	bool jumps;
	unsigned pred = ctx->pred, pred_idx = ctx->pred_idx;

	/* XXX nested predicates are not fully implemented, but optimizations
	 * give us a form we can deal with easily
	 */
	assert(!pred || pred == 3);

	instr = ir2_instr_create(ctx, IR2_ALU);
	instr->src[0] = make_src(ctx, nif->condition, true);
	instr->src_count = 1;
	instr->ssa.ncomp = 1;
	instr->alu.vector_opc = VECTOR_NONE;
	instr->alu.scalar_opc = SCALAR_NONE;
	instr->alu.export = -1;
	instr->alu.write_mask = 1;
	instr->pred = 0;

	if (pred) {
		/* XXX we want this instr in its own block */
		instr->alu.vector_opc = PRED_SETNE_PUSHv;
		instr->src[1] = instr->src[0];
		instr->src[0] = ir2_src(pred_idx, 0, IR2_SRC_SSA);
		instr->src[0].swizzle = IR2_SWIZZLE_XXXX;
		instr->src[1].swizzle = IR2_SWIZZLE_XXXX;
		instr->src_count = 2;
	} else {
		instr->alu.scalar_opc = PRED_SETNEs;
	}

	ctx->pred_idx = instr->idx;
	ctx->pred = 3;

	jumps = emit_cf_list(ctx, &nif->then_list);

	/* nested predicate must jump away */
	assert(!pred || jumps);

	/* only need an "else" predicate when "then" doesn't jump away */
	if (pred) {
		/* XXX we want this instr in its own block */

		/* instr = ir2_instr_create(ctx, IR2_ALU);
		instr->src[0] = ir2_src(ctx->pred_idx, 0, 0);
		instr->src_count = 1;
		instr->ncomp = 1;
		instr->alu.vector_opc = VECTOR_NONE;
		instr->alu.scalar_opc = PRED_SET_INVs;
		instr->alu.export = -1;
		instr->alu.write_mask = 1;
		instr->pred = 0;
		instr->block_idx++; //XXX
		ctx->pred_idx = instr->idx;*/

		instr = ir2_instr_create(ctx, IR2_ALU);
		instr->src[0] = ir2_src(ctx->pred_idx, 0, IR2_SRC_SSA);
		instr->src_count = 1;
		instr->ssa.ncomp = 1;
		instr->alu.vector_opc = VECTOR_NONE;
		instr->alu.scalar_opc = PRED_SET_POPs;
		instr->alu.export = -1;
		instr->alu.write_mask = 1;
		instr->pred = 0;
		instr->block_idx++;
		ctx->pred_idx = instr->idx;
	}

	ctx->pred = !jumps ? 2 : pred;
	jumps = emit_cf_list(ctx, &nif->else_list);
	ctx->pred = pred;
}

/* get the highest block idx in the loop, so we know when
 * we can free registers that are allocated outside the loop
 */
static unsigned
loop_last_block(struct exec_list *list)
{
	nir_cf_node *node =
		exec_node_data(nir_cf_node, exec_list_get_tail(list), node);
	switch (node->type) {
	case nir_cf_node_block:
		return nir_cf_node_as_block(node)->index;
	case nir_cf_node_if:
		assert(0); /* XXX could this ever happen? */
		return 0;
	case nir_cf_node_loop:
		return loop_last_block(&nir_cf_node_as_loop(node)->body);
	default:
		compile_error(ctx, "Not supported\n");
		return 0;
	}
}

static void
emit_loop(struct ir2_context *ctx, nir_loop *nloop)
{
	ctx->loop_last_block[++ctx->loop_depth] = loop_last_block(&nloop->body);
	emit_cf_list(ctx, &nloop->body);
	ctx->loop_depth--;
}

static bool
emit_cf_list(struct ir2_context *ctx, struct exec_list *list)
{
	bool ret = false;
	foreach_list_typed(nir_cf_node, node, node, list) {
		ret = false;
		switch (node->type) {
		case nir_cf_node_block:
			ret = emit_block(ctx, nir_cf_node_as_block(node));
			break;
		case nir_cf_node_if:
			emit_if(ctx, nir_cf_node_as_if(node));
			break;
		case nir_cf_node_loop:
			emit_loop(ctx, nir_cf_node_as_loop(node));
			break;
		case nir_cf_node_function:
			compile_error(ctx, "Not supported\n");
			break;
		}
	}
	return ret;
}

static void
variant_opt(struct ir2_context *ctx, unsigned variant)
{
	if (!variant)
		return;

	assert(ctx->so->type == SHADER_VERTEX);

	/* kill non-position outputs for binning variant */
	nir_foreach_block(block, nir_shader_get_entrypoint(ctx->nir)) {
		nir_foreach_instr_safe(instr, block) {
			if (instr->type != nir_instr_type_intrinsic)
				continue;

			nir_intrinsic_instr *intr = nir_instr_as_intrinsic(instr);
			if (intr->intrinsic != nir_intrinsic_store_output)
				continue;

			unsigned idx = nir_intrinsic_base(intr);
			unsigned slot = output_slot(ctx, idx);
			if (slot != VARYING_SLOT_POS)
				nir_instr_remove(instr);
		}
	}

	ir2_optimize_nir(ctx->nir, false);
}

void
ir2_nir_compile(struct ir2_context *ctx, unsigned variant)
{
	struct fd2_shader_stateobj *so = ctx->so;

	memset(ctx->ssa_map, 0xff, sizeof(ctx->ssa_map));

	ctx->nir = nir_shader_clone(NULL, so->nir);

	variant_opt(ctx, variant);

	/* postprocess */
	OPT_V(ctx->nir, nir_opt_algebraic_late);

	/* lower to scalar instructions that can only be scalar on a2xx */
	OPT_V(ctx->nir, ir2_nir_lower_scalar);

	OPT_V(ctx->nir, nir_lower_to_source_mods);
	OPT_V(ctx->nir, nir_copy_prop);
	OPT_V(ctx->nir, nir_opt_dce);
	OPT_V(ctx->nir, nir_opt_move_comparisons);

	OPT_V(ctx->nir, nir_lower_locals_to_regs);

	OPT_V(ctx->nir, nir_convert_from_ssa, true);

	OPT_V(ctx->nir, nir_move_vec_src_uses_to_dest);
	OPT_V(ctx->nir, nir_lower_vec_to_movs);

	OPT_V(ctx->nir, nir_opt_dce);

	nir_sweep(ctx->nir);

	if (fd_mesa_debug & FD_DBG_DISASM) {
		debug_printf("----------------------\n");
		nir_print_shader(ctx->nir, stdout);
		debug_printf("----------------------\n");
	}

	/* fd2_shader_stateobj init */
	if (so->type == SHADER_FRAGMENT) {
		so->f.fragcoord = -1;
		so->f.inputs_count = 0;
		memset(so->f.inputs, 0, sizeof(so->f.inputs));
	}

	/* Setup inputs: */
	nir_foreach_variable(in, &ctx->nir->inputs)
		setup_input(ctx, in);

	if (so->type == SHADER_FRAGMENT) {
		unsigned idx;
        for (idx = 0; idx < so->f.inputs_count; idx++) {
			ctx->input[idx].ncomp = so->f.inputs[idx].ncomp;
			update_range(ctx, &ctx->input[idx]);
		}
		/* assume we have param input and kill it later if not */
		ctx->input[idx].ncomp = 4;
		update_range(ctx, &ctx->input[idx]);
	} else {
		ctx->input[0].ncomp = 1;
		ctx->input[2].ncomp = 1;
		update_range(ctx, &ctx->input[0]);
		update_range(ctx, &ctx->input[2]);
	}

	/* And emit the body: */
	nir_function_impl *fxn = nir_shader_get_entrypoint(ctx->nir);

	nir_foreach_register(reg, &fxn->registers) {
		ctx->reg[reg->index].ncomp = reg->num_components;
		ctx->reg_count = MAX2(ctx->reg_count, reg->index + 1);
	}

	nir_metadata_require(fxn, nir_metadata_block_index);
	emit_cf_list(ctx, &fxn->body);
	/* TODO emit_block(ctx, fxn->end_block); */

	if (so->type == SHADER_VERTEX)
		extra_position_exports(ctx, variant);

	ralloc_free(ctx->nir);

	/* kill unused param input */
	if (so->type == SHADER_FRAGMENT && !so->need_param)
		ctx->input[so->f.inputs_count].initialized = false;
}
