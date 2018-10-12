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

#define swiz_transform(swizzle, ncomp, fn) ({ \
	unsigned swiz = 0, c; \
	for (int i = 0; i < ncomp; i++) { \
		c = swiz_get(swizzle, i); \
		swiz |= swiz_set(fn, i); \
	} \
	swiz; \
})

static unsigned
src_swizzle(struct ir2_context *ctx, struct ir2_src *reg, unsigned ncomp)
{
	struct ir2_reg_component *comps;

	/* for const/input, nothing to do */
	if (is_const(reg) || is_input(reg))
		return reg->swizzle;

	/* we need to take into account where the src components were allocated */
	comps = get_components(ctx, &ctx->instr[reg->num]);
	return swiz_transform(reg->swizzle, ncomp, comps[c].comp);
}

/* alu instr need to take into how the output components are allocated */

static unsigned
alu_swizzle(struct ir2_context *ctx, struct ir2_instruction *instr,
			struct ir2_src *reg)
{
	struct ir2_reg_component *comp = get_components(ctx, instr);
	unsigned swiz0 = src_swizzle(ctx, reg, num_src_comps(instr));
	unsigned swiz = 0;

	for (int i = 0, j = 0; i < instr->num_components; j++) {
		if (instr->alu.write_mask & 1 << j)
			swiz |= swiz_set(i++, comp[j].comp);
	}

	return swiz_merge(swiz0, swiz);
}

/* scalar doesn't need to take into account dest swizzle */

static unsigned
alu_swizzle_scalar(struct ir2_context *ctx, struct ir2_src *reg)
{
	/* hardware seems to take from W, but swizzle everywhere just in case */
	return swiz_merge(src_swizzle(ctx, reg, 1), IR2_SWIZZLE_XXXX);
}

static unsigned
alu_swizzle_scalar2(struct ir2_context *ctx, struct ir2_src *reg)
{
	/* hardware seems to take from ZW, but swizzle everywhere (ABAB) */
	unsigned s0 = swiz_get(src_swizzle(ctx, &reg[0], 1), 0);
	unsigned s1 = swiz_get(src_swizzle(ctx, &reg[1], 1), 0);

	return swiz_merge(swiz_set(s0, 0) | swiz_set(s1, 1), IR2_SWIZZLE_XYXY);
}

/* write_mask needs to be transformed by allocation information */

static unsigned
alu_write_mask(struct ir2_context *ctx, struct ir2_instruction *instr)
{
	struct ir2_reg_component *comp = get_components(ctx, instr);
	unsigned write_mask = 0;

	for (int i = 0; i < 4; i++) {
		if (instr->alu.write_mask & 1 << i)
			write_mask |= 1 << comp[i].comp;
	}

	return write_mask;
}

/* fetch instructions can swizzle dest, but src swizzle needs conversion */

static unsigned
fetch_swizzle(struct ir2_context *ctx, struct ir2_src *reg, unsigned ncomp)
{
	unsigned alu_swiz = src_swizzle(ctx, reg, ncomp);
	unsigned swiz = 0;
	for (int i = 0; i < ncomp; i++)
		swiz |= swiz_get(alu_swiz, i) << i * 2;
	return swiz;
}

static unsigned
fetch_dst_swiz(struct ir2_context *ctx, struct ir2_instruction *instr)
{
	struct ir2_reg_component *comp = get_components(ctx, instr);
	unsigned dst_swiz = 0xfff;
	for (int i = 0; i < instr->num_components; i++) {
		dst_swiz &= ~(7 << comp[i].comp * 3);
		dst_swiz |= i << comp[i].comp * 3;
	}
	return dst_swiz;
}


/* register / export # for instr */
static unsigned
dst_to_reg(struct ir2_context *ctx, struct ir2_instruction *instr)
{
	if (is_export(instr))
		return instr->alu.export;

	return get_alloc(ctx, instr)->reg;
}

/* register # for src */
static unsigned src_to_reg(struct ir2_context *ctx, struct ir2_src *reg)
{
	assert(!is_const(reg));

	if (is_input(reg))
		return reg->num;

	return get_alloc(ctx, &ctx->instr[reg->num])->reg;
}

static unsigned src_reg_byte(struct ir2_context *ctx, struct ir2_src *reg)
{
	if (reg->flags & IR2_REG_CONST) {
		// XXX allow negate?
		assert((reg->flags & ~IR2_REG_NEGATE) == IR2_REG_CONST);
		return reg->num;
	}

	unsigned num = src_to_reg(ctx, reg);
	if (reg->flags & IR2_REG_ABS)
		num |= 0x80;
	return num;
}

/* produce the 12 byte binary instruction for a given sched_instr */
static void
fill_instr(struct ir2_context *ctx, struct ir2_sched_instr *sched,
		   instr_alu_t * alu, bool * is_fetch)
{
	struct ir2_instruction *instr = sched->instr, *instr_s, *instr_v;

	memset(alu, 0, sizeof(*alu));

	if (instr && instr->instr_type == IR2_FETCH) {
		*is_fetch = true;

		instr_fetch_t *fetch = (instr_fetch_t *) alu;

		fetch->opc = instr->fetch.opc;

		assert(instr->src_reg_count == 1);
		struct ir2_src *src_reg = instr->src_reg;

		if (instr->fetch.opc == VTX_FETCH) {
			instr_fetch_vtx_t *vtx = &fetch->vtx;

			assert(instr->fetch.stride <= 0xff);
			assert(instr->fetch.fmt <= 0x3f);
			assert(instr->fetch.const_idx <= 0x1f);
			assert(instr->fetch.const_idx_sel <= 0x3);

			vtx->src_reg = src_to_reg(ctx, src_reg);
			vtx->src_swiz = fetch_swizzle(ctx, src_reg, 1);
			vtx->dst_reg = dst_to_reg(ctx, instr);
			vtx->must_be_one = 1;
			vtx->const_index = instr->fetch.const_idx;
			vtx->const_index_sel = instr->fetch.const_idx_sel;

			/* other fields will be patched */

			/* XXX seems like every FETCH but the first has
			 * this bit set:
			 */
			vtx->reserved3 = instr->idx ? 0x1 : 0x0;
			vtx->reserved0 = instr->idx ? 0x2 : 0x3;
		} else if (instr->fetch.opc == TEX_FETCH) {
			instr_fetch_tex_t *tex = &fetch->tex;

			assert(instr->fetch.const_idx <= 0x1f);

			tex->src_reg = src_to_reg(ctx, src_reg);
			tex->src_swiz = fetch_swizzle(ctx, src_reg, 3);
			tex->dst_reg = dst_to_reg(ctx, instr);
			tex->dst_swiz = fetch_dst_swiz(ctx, instr);
			tex->const_idx = instr->fetch.const_idx;
			tex->mag_filter = TEX_FILTER_USE_FETCH_CONST;
			tex->min_filter = TEX_FILTER_USE_FETCH_CONST;
			tex->mip_filter = TEX_FILTER_USE_FETCH_CONST;
			tex->aniso_filter = ANISO_FILTER_USE_FETCH_CONST;
			tex->arbitrary_filter = ARBITRARY_FILTER_USE_FETCH_CONST;
			tex->vol_mag_filter = TEX_FILTER_USE_FETCH_CONST;
			tex->vol_min_filter = TEX_FILTER_USE_FETCH_CONST;
			tex->use_comp_lod = 1;
			tex->use_reg_lod = !instr->fetch.is_cube;
			tex->sample_location = SAMPLE_CENTER;
			tex->tx_coord_denorm = instr->fetch.is_rect;
		} else {
			assert(0);
		}
		return;
	}

	instr_v = sched->instr;
	instr_s = sched->instr_s;

	if (instr_v) {
		struct ir2_src src1, src2, *src3;

		src1 = instr_v->src_reg[0];
		src2 = instr_v->src_reg[instr_v->src_reg_count > 1];
		src3 = instr_v->src_reg_count == 3 ? &instr_v->src_reg[2] : NULL;

		alu->vector_opc = instr_v->alu.vector_opc;
		alu->vector_write_mask = alu_write_mask(ctx, instr_v);
		alu->vector_dest = dst_to_reg(ctx, instr_v);
		alu->vector_clamp = instr_v->alu.saturate;
		alu->export_data = instr_v->alu.export >= 0;

		/* single operand SETEv, use 0.0f as src2 */
		if (instr_v->src_reg_count == 1 && alu->vector_opc == SETEv)
			src2 = ir2_zero();

		/* export32 has this bit set.. it seems to do more than just set
		 * the base address of the constants used to zero
		 * XXX this is a hack
		 */
		alu->relative_addr = (alu->export_data && alu->vector_dest == 32);

		alu->src1_reg_byte = src_reg_byte(ctx, &src1);
		alu->src1_swiz = alu_swizzle(ctx, instr_v, &src1);
		alu->src1_reg_negate = ! !(src1.flags & IR2_REG_NEGATE);
		alu->src1_sel = !(src1.flags & IR2_REG_CONST);

		alu->src2_reg_byte = src_reg_byte(ctx, &src2);
		alu->src2_swiz = alu_swizzle(ctx, instr_v, &src2);
		alu->src2_reg_negate = ! !(src2.flags & IR2_REG_NEGATE);
		alu->src2_sel = !(src2.flags & IR2_REG_CONST);

		if (src3) {
			alu->src3_reg_byte = src_reg_byte(ctx, src3);
			alu->src3_swiz = alu_swizzle(ctx, instr_v, src3);
			alu->src3_reg_negate = ! !(src3->flags & IR2_REG_NEGATE);
			alu->src3_sel = !(src3->flags & IR2_REG_CONST);
		}
	}

	if (instr_s) {
		struct ir2_src *src = instr_s->src_reg;

		alu->scalar_opc = instr_s->alu.scalar_opc;
		alu->scalar_write_mask = alu_write_mask(ctx, instr_s);
		alu->scalar_dest = dst_to_reg(ctx, instr_s);
		alu->scalar_clamp = instr_s->alu.saturate;
		alu->export_data = instr_s->alu.export >= 0;

		if (instr_s->src_reg_count == 1) {
			alu->src3_reg_byte = src_reg_byte(ctx, src);
			alu->src3_swiz = alu_swizzle_scalar(ctx, src);
			alu->src3_reg_negate = is_neg(src);
			alu->src3_sel = !is_const(src);
		} else {
			assert(instr_s->src_reg_count == 2);

			alu->src3_reg_byte = src_reg_byte(ctx, src);
			alu->src3_swiz = alu_swizzle_scalar2(ctx, src);
			alu->src3_reg_negate = is_neg(src);
			alu->src3_sel = !is_const(src);
		}
	}

	*is_fetch = false;
	return;
}

static unsigned
write_cfs(struct ir2_context *ctx, instr_cf_t * cfs, unsigned cf_idx,
		  instr_cf_alloc_t * alloc, instr_cf_exec_t * exec)
{
	assert(exec->count);

	if (alloc)
		cfs[cf_idx++].alloc = *alloc;

	/* for memory alloc offset for patching */
	if (alloc && alloc->buffer_select == SQ_MEMORY &&
		ctx->info->export32_offset == -1)
		ctx->info->export32_offset = cf_idx / 2 * 3;

	cfs[cf_idx++].exec = *exec;
	exec->address += exec->count;
	exec->serialize = 0;
	exec->count = 0;

	return cf_idx;
}

/* assemble the final shader */
void assemble(struct ir2_context *ctx)
{
	/* hw seems to have a limit of 384 (num_cf/2+num_instr <= 384)
	 * address is 9 bits so could it be 512 ?
	 */
	instr_cf_t cfs[384];
	instr_alu_t instr[384];
	unsigned num_cf = 0;

	/* CF instr state */
	instr_cf_exec_t exec = {.opc = EXEC };
	instr_cf_alloc_t alloc = {.opc = ALLOC };

	bool is_fetch;
	bool prev_fetch = false;
	bool need_sync = true;
	bool need_alloc = false;

	ctx->info->export32_offset = -1;
	ctx->info->num_fetch_instrs = 0;

	if (ctx->info->max_export == -1) {
		alloc.buffer_select = SQ_PARAMETER_PIXEL;
		cfs[num_cf++].alloc = alloc;
	}

	for (int i = 0; i < ctx->instr_sched_count; i++) {
		/* fill the 3 dwords for the instruction */
		fill_instr(ctx, &ctx->instr_sched[i], &instr[i], &is_fetch);

		need_sync = !i || is_fetch != prev_fetch;
		prev_fetch = is_fetch;

		/* info for patching */
		if (is_fetch) {
			instr_fetch_t *fetch = (instr_fetch_t *) & instr[i];
			struct ir2_instruction *instr = ctx->instr_sched[i].instr;
			struct ir2_fetch_info *info =
				&ctx->info->fetch_info[ctx->info->num_fetch_instrs++];
			info->offset = i * 3;	/* add cf offset later */

			if (fetch->opc == VTX_FETCH) {
				struct ir2_reg_component *comp =
					get_components(ctx, instr);
				memset(info->vtx.swiz, 0xff, sizeof(info->vtx.swiz));
				for (int i = 0; i < instr->num_components; i++) {
					if (comp[i].comp >= 0)
						info->vtx.swiz[comp[i].comp] = i;
				}
			} else {
				info->tex.samp_id = instr->fetch.samp_id;
			}
		}

		/* exec cf after 6 instr or when switching between fetch / alu */
		if (exec.count == 6 || (exec.count && need_sync)) {
			num_cf =
				write_cfs(ctx, cfs, num_cf, need_alloc ? &alloc : NULL,
						  &exec);
			need_alloc = false;
		}

		/* export - fill alloc cf */
		if (!is_fetch && instr[i].export_data) {
			/* get the export buffer from either vector/scalar dest */
			instr_alloc_type_t buffer =
				export_buffer(instr[i].vector_dest);
			if (instr[i].scalar_write_mask) {
				if (instr[i].vector_write_mask)
					assert(buffer == export_buffer(instr[i].scalar_dest));
				buffer = export_buffer(instr[i].scalar_dest);
			}

			if (alloc.buffer_select == 0)
				assert(buffer == SQ_PARAMETER_PIXEL);

			/* flush previous alloc if the buffer changes */
			bool need_new_alloc = buffer != alloc.buffer_select;

			/* memory export always in 32/33 pair, new alloc on 32 */
			if (instr[i].vector_dest == 32)
				need_new_alloc = true;

			if (need_new_alloc && need_alloc) {
				num_cf = write_cfs(ctx, cfs, num_cf, &alloc, &exec);
				need_alloc = false;
			}

			need_alloc |= need_new_alloc;

			alloc.size = 0;		// XXX 1 for SQ_POSITION with point_size
			alloc.buffer_select = buffer;

			if (buffer == SQ_PARAMETER_PIXEL)
				alloc.size = ctx->info->max_export;
		}

		if (is_fetch)
			exec.serialize |= 0x1 << exec.count * 2;
		if (need_sync)
			exec.serialize |= 0x2 << exec.count * 2;

		need_sync = false;
		exec.count += 1;
	}

	/* final exec cf */
	exec.opc = EXEC_END;
	num_cf =
		write_cfs(ctx, cfs, num_cf, need_alloc ? &alloc : NULL, &exec);

	/* insert nop to get an even # of CFs */
	if (num_cf % 2)
		cfs[num_cf++] = (instr_cf_t) {
		.opc = NOP};

	/* offset cf addrs */
	for (int idx = 0; idx < num_cf; idx++) {
		switch (cfs[idx].opc) {
		case EXEC:
		case EXEC_END:
			cfs[idx].exec.address += num_cf / 2;
			break;
		default:
			break;
			/* XXX  and any other address using cf that gets implemented */
		}
	}

	/* concatenate cfs and alu/fetch */
	uint32_t cfdwords = num_cf / 2 * 3;
	uint32_t alufetchdwords = exec.address * 3;
	uint32_t sizedwords = cfdwords + alufetchdwords;
	uint32_t *dwords = malloc(sizedwords * 4);
	assert(dwords);
	memcpy(dwords, cfs, cfdwords * 4);
	memcpy(&dwords[cfdwords], instr, alufetchdwords * 4);

	/* finalize ir2_shader_info */
	ctx->info->dwords = dwords;
	ctx->info->sizedwords = sizedwords;
	for (int i = 0; i < ctx->info->num_fetch_instrs; i++)
		ctx->info->fetch_info[i].offset += cfdwords;

	if (fd_mesa_debug & FD_DBG_DISASM) {
		DBG("disassemble: type=%d", ctx->so->type);
		disasm_a2xx(dwords, sizedwords, 0, ctx->so->type);
	}
}
