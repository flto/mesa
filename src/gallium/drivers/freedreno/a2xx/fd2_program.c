/* -*- mode: C; c-file-style: "k&r"; tab-width 4; indent-tabs-mode: t; -*- */

/*
 * Copyright (C) 2012 Rob Clark <robclark@freedesktop.org>
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
 *    Rob Clark <robclark@freedesktop.org>
 *    Jonathan Marek <jonathan@marek.ca>
 */

#include "pipe/p_state.h"
#include "util/u_string.h"
#include "util/u_memory.h"
#include "util/u_inlines.h"
#include "util/u_format.h"
#include "tgsi/tgsi_dump.h"
#include "tgsi/tgsi_parse.h"

#include "freedreno_program.h"

#include "ir2.h"
#include "fd2_program.h"
#include "fd2_texture.h"
#include "fd2_util.h"

static struct fd2_shader_stateobj *
create_shader(enum shader_t type)
{
	struct fd2_shader_stateobj *so = CALLOC_STRUCT(fd2_shader_stateobj);
	if (!so)
		return NULL;
	so->type = type;
	return so;
}

static void
delete_shader(struct fd2_shader_stateobj *so)
{
	if (!so)
		return;
	ralloc_free(so->nir);
	free(so->info[0].dwords);
	free(so->info[1].dwords);
	free(so);
}

static void
emit(struct fd_batch *batch, struct fd_ringbuffer *ring,
		struct fd2_shader_stateobj *so)
{
	bool binning = ring == batch->binning;
	bool a20x_binning = binning && is_a20x(batch->ctx->screen);
	struct ir2_shader_info *info;
	unsigned i;

	info = &so->info[binning];

	assert(info->sizedwords);

	OUT_PKT3(ring, CP_IM_LOAD_IMMEDIATE, 2 + info->sizedwords);
	OUT_RING(ring, (so->type == SHADER_VERTEX) ? 0 : 1);
	OUT_RING(ring, info->sizedwords);
	for (i = 0; i < info->sizedwords; i++) {
		if (a20x_binning && i == info->export32_offset)
			OUT_RINGP(ring, info->dwords[i], &batch->draw_patches);
		else
			OUT_RING(ring, info->dwords[i]);
	}
}

static int
ir2_glsl_type_size(const struct glsl_type *type)
{
	return glsl_count_attribute_slots(type, false);
}

static void *
fd2_fp_state_create(struct pipe_context *pctx,
		const struct pipe_shader_state *cso)
{
	struct fd2_shader_stateobj *so = create_shader(SHADER_FRAGMENT);
	if (!so)
		return NULL;

	if (cso->type == PIPE_SHADER_IR_NIR) {
		so->nir = cso->ir.nir;
		NIR_PASS_V(so->nir, nir_lower_io, nir_var_all, ir2_glsl_type_size,
			   (nir_lower_io_options)0);
	} else {
		assert(cso->type == PIPE_SHADER_IR_TGSI);
        so->nir = ir2_tgsi_to_nir(cso->tokens);
	}

	if (ir2_optimize_nir(so->nir))
		goto fail;

	ir2_compile(so, 0);

	ralloc_free(so->nir);
	so->nir = NULL;

	return so;

fail:
	delete_shader(so);
	return NULL;
}

static void
fd2_fp_state_delete(struct pipe_context *pctx, void *hwcso)
{
	struct fd2_shader_stateobj *so = hwcso;
	delete_shader(so);
}

static void *
fd2_vp_state_create(struct pipe_context *pctx,
		const struct pipe_shader_state *cso)
{
	struct fd2_shader_stateobj *so = create_shader(SHADER_VERTEX);
	if (!so)
		return NULL;

	if (cso->type == PIPE_SHADER_IR_NIR) {
		so->nir = cso->ir.nir;
		NIR_PASS_V(so->nir, nir_lower_io, nir_var_all, ir2_glsl_type_size,
			   (nir_lower_io_options)0);
	} else {
		assert(cso->type == PIPE_SHADER_IR_TGSI);
        so->nir = ir2_tgsi_to_nir(cso->tokens);
	}

	if (ir2_optimize_nir(so->nir))
		goto fail;

	/* can't compile the vertex shader here, as it depends on fs */

	return so;

fail:
	delete_shader(so);
	return NULL;
}

static void
fd2_vp_state_delete(struct pipe_context *pctx, void *hwcso)
{
	struct fd2_shader_stateobj *so = hwcso;
	delete_shader(so);
}

static void
patch_vtx_fetch(struct fd_context *ctx, struct pipe_vertex_element *elem,
	instr_fetch_vtx_t *instr, int8_t *swizzle)
{
	struct pipe_vertex_buffer *vb =
				&ctx->vtx.vertexbuf.vb[elem->vertex_buffer_index];
	enum pipe_format format = elem->src_format;
	const struct util_format_description *desc =
			util_format_description(format);
	unsigned j;

	/* Find the first non-VOID channel. */
	for (j = 0; j < 4; j++)
		if (desc->channel[j].type != UTIL_FORMAT_TYPE_VOID)
			break;

	instr->format = fd2_pipe2surface(format);
	instr->num_format_all = !desc->channel[j].normalized;
	instr->format_comp_all = desc->channel[j].type == UTIL_FORMAT_TYPE_SIGNED;
	instr->stride = vb->stride;
	instr->offset = elem->src_offset;

	unsigned swiz = 0;
	for (int i = 0; i < 4; i++) {
		swiz |= (swizzle[i] < 0 ? 7 : desc->swizzle[swizzle[i]]) << i*3;
	}
	instr->dst_swiz = swiz;
}

static void
patch_fetches(struct fd_context *ctx, struct ir2_shader_info *info,
	struct fd_vertex_stateobj *vtx, struct fd_texture_stateobj *tex)
{
    for (int i = 0; i < info->num_fetch_instrs; i++) {
        struct ir2_fetch_info *fi = &info->fetch_info[i];

		instr_fetch_t *instr = (instr_fetch_t*) &info->dwords[fi->offset];
        if (instr->opc == VTX_FETCH) {
			unsigned idx = (instr->vtx.const_index - 20) * 3 +
				instr->vtx.const_index_sel;
			patch_vtx_fetch(ctx, &vtx->pipe[idx], &instr->vtx, fi->vtx.swiz);
			continue;
        }

        assert(instr->opc == TEX_FETCH);
		instr->tex.const_idx = fd2_get_const_idx(ctx, tex, fi->tex.samp_id);
    }
}

void
fd2_program_validate(struct fd_context *ctx)
{
	struct fd_program_stateobj *prog = &ctx->prog;
	struct fd2_shader_stateobj *fp = prog->fp, *vp = prog->vp;

	/* recompile vertex shader when fragment shader changes */
	if (vp->v.fp != fp) {
		vp->v.fp = fp;
		ir2_compile(vp, 0);
		ir2_compile(vp, 1);
	}

	/* patch fetch instructions */
	patch_fetches(ctx, &vp->info[0], ctx->vtx.vtx, &ctx->tex[PIPE_SHADER_VERTEX]);
	patch_fetches(ctx, &vp->info[1], ctx->vtx.vtx, &ctx->tex[PIPE_SHADER_VERTEX]);
	patch_fetches(ctx, &fp->info[0], NULL, &ctx->tex[PIPE_SHADER_FRAGMENT]);
}

void
fd2_program_emit(struct fd_batch *batch, struct fd_ringbuffer *ring,
		struct fd_program_stateobj *prog)
{
	struct fd2_shader_stateobj *fp, *vp;
	uint8_t vs_gprs, fs_gprs = 0, vs_export = 0;
	bool b;

	vp = prog->vp;
	fp = prog->fp;
	assert(vp->v.fp == fp);

	emit(batch, ring, vp);

	if (!(b = (ring == batch->binning))) {
		emit(batch, ring, fp);
		fs_gprs = (fp->info[0].max_reg < 0) ? 0x80 : fp->info[0].max_reg;
		vs_export = MAX2(1, fp->f.inputs_count) - 1;
	}

	vs_gprs = (vp->info[b].max_reg < 0) ? 0x80 : vp->info[b].max_reg;

	OUT_PKT3(ring, CP_SET_CONSTANT, 2);
	OUT_RING(ring, CP_REG(REG_A2XX_SQ_PROGRAM_CNTL));
	OUT_RING(ring, A2XX_SQ_PROGRAM_CNTL_PS_EXPORT_MODE(POSITION_2_VECTORS_SPRITE) |
			A2XX_SQ_PROGRAM_CNTL_VS_EXPORT_MODE(0) |
			A2XX_SQ_PROGRAM_CNTL_VS_RESOURCE |
			A2XX_SQ_PROGRAM_CNTL_PS_RESOURCE |
			A2XX_SQ_PROGRAM_CNTL_VS_EXPORT_COUNT(vs_export) |
			A2XX_SQ_PROGRAM_CNTL_PS_REGS(fs_gprs) |
			A2XX_SQ_PROGRAM_CNTL_VS_REGS(vs_gprs) |
			/* generate R2 index for a20x hw binning */
			((b && is_a20x(batch->ctx->screen)) ?
				A2XX_SQ_PROGRAM_CNTL_GEN_INDEX_VTX : 0));
}

void
fd2_prog_init(struct pipe_context *pctx)
{
	struct fd_context *ctx = fd_context(pctx);
	struct fd_program_stateobj *prog;
	struct fd2_shader_stateobj *so;
	instr_fetch_vtx_t *instr;

	pctx->create_fs_state = fd2_fp_state_create;
	pctx->delete_fs_state = fd2_fp_state_delete;

	pctx->create_vs_state = fd2_vp_state_create;
	pctx->delete_vs_state = fd2_vp_state_delete;

	fd_prog_init(pctx);

	// XXX make this less hacky?

	prog = &ctx->solid_prog;
	so = prog->vp;
	so->v.fp = prog->fp;
	ir2_compile(prog->vp, 0);

#define IR2_FETCH_SWIZ_XY01 0xb08
#define IR2_FETCH_SWIZ_XYZ1 0xa88

	instr = (instr_fetch_vtx_t*) &so->info[0].dwords[so->info[0].fetch_info[0].offset];
	instr->const_index = 26;
	instr->const_index_sel = 0;
	instr->format = FMT_32_32_32_FLOAT;
	instr->format_comp_all = false;
	instr->stride = 12;
	instr->num_format_all = true;
	instr->dst_swiz = IR2_FETCH_SWIZ_XYZ1;

	prog = &ctx->blit_prog[0];
	so = prog->vp;
	so->v.fp = prog->fp;
	ir2_compile(prog->vp, 0);

	instr = (instr_fetch_vtx_t*) &so->info[0].dwords[so->info[0].fetch_info[0].offset];
	instr->const_index = 26;
	instr->const_index_sel = 1;
	instr->format = FMT_32_32_FLOAT;
	instr->format_comp_all = false;
	instr->stride = 8;
	instr->num_format_all = false;
	instr->dst_swiz = IR2_FETCH_SWIZ_XY01;

	instr = (instr_fetch_vtx_t*) &so->info[0].dwords[so->info[0].fetch_info[1].offset];
	instr->const_index = 26;
	instr->const_index_sel = 0;
	instr->format = FMT_32_32_32_FLOAT;
	instr->format_comp_all = false;
	instr->stride = 12;
	instr->num_format_all = false;
	instr->dst_swiz = IR2_FETCH_SWIZ_XYZ1;
}
