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
#include "instr-a2xx.h"

static struct fd2_shader_stateobj *
create_shader(struct pipe_context *pctx, enum shader_t type)
{
	struct fd2_shader_stateobj *so = CALLOC_STRUCT(fd2_shader_stateobj);
	if (!so)
		return NULL;
	so->type = type;
	so->is_a20x = is_a20x(fd_context(pctx)->screen);
	return so;
}

static void
delete_shader(struct fd2_shader_stateobj *so)
{
	if (!so)
		return;
	ralloc_free(so->nir);
	for (int i = 0; i < ARRAY_SIZE(so->variant); i++)
		free(so->variant[i].info.dwords);
	free(so);
}

static void
emit(struct fd_batch *batch, struct fd_ringbuffer *ring,
		struct fd2_shader_stateobj *so, unsigned variant, bool patches)
{
	struct ir2_shader_info *info;
	unsigned i;

	info = &so->variant[variant].info;

	assert(info->sizedwords);

	OUT_PKT3(ring, CP_IM_LOAD_IMMEDIATE, 2 + info->sizedwords);
	OUT_RING(ring, so->type == SHADER_FRAGMENT);
	OUT_RING(ring, info->sizedwords);
	if (patches) {
		util_dynarray_append(&batch->shader_patches, struct fd_cs_patch, ((struct fd_cs_patch){
			.cs  = ring->cur + info->mem_export_ptr,
		}));
	}

	for (i = 0; i < info->sizedwords; i++)
		OUT_RING(ring, info->dwords[i]);
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
	struct fd2_shader_stateobj *so = create_shader(pctx, SHADER_FRAGMENT);
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

	if (ir2_optimize_nir(so->nir, true))
		goto fail;

	so->first_immediate = so->nir->num_uniforms;

	ir2_compile(so, 0, NULL);

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
	struct fd2_shader_stateobj *so = create_shader(pctx, SHADER_VERTEX);
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

	if (ir2_optimize_nir(so->nir, true))
		goto fail;

	so->first_immediate = so->nir->num_uniforms;

	/* compile binning variant now */
	ir2_compile(so, 0, NULL);

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
	instr_fetch_vtx_t *instr, uint16_t dst_swiz)
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
		unsigned s = dst_swiz >> i*3 & 7;
		swiz |= (s >= 4 ? s : desc->swizzle[s]) << i*3;
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
			patch_vtx_fetch(ctx, &vtx->pipe[idx], &instr->vtx, fi->vtx.dst_swiz);
			continue;
		}

		assert(instr->opc == TEX_FETCH);
		instr->tex.const_idx = fd2_get_const_idx(ctx, tex, fi->tex.samp_id);
		instr->tex.src_swiz = fi->tex.src_swiz;
		if (fd2_texture_swap_xy(tex, fi->tex.samp_id)) {
			unsigned x = instr->tex.src_swiz;
			instr->tex.src_swiz = (x & 0x30) | (x & 3) << 2 | (x >> 2 & 3);
		}
	}
}

void
fd2_program_emit(struct fd_batch *batch, struct fd_ringbuffer *ring,
		struct fd_program_stateobj *prog)
{
	struct fd_context *ctx = batch->ctx;
	struct fd2_shader_stateobj *fp, *vp;
	struct ir2_shader_info *fpi, *vpi;
	uint8_t vs_gprs, fs_gprs = 0, vs_export = 0;
	bool binning = (ring == batch->binning);
	unsigned i;

	vp = prog->vp;

	if (binning) {
		fp = NULL;
		fpi = NULL;
		i = 0;
		vpi = &vp->variant[i].info;
	} else {
		fp = prog->fp;
		if ((fd_mesa_debug & FD_DBG_FRAGS) && vp != batch->ctx->blit_prog[0].vp)
			fp = batch->ctx->solid_prog.fp;

		fpi = &fp->variant[0].info;

		for (i = 1; i < ARRAY_SIZE(vp->variant); i++) {
			vpi = &vp->variant[i].info;

			if (!vpi->sizedwords) {
				ir2_compile(vp, i, &fp->f);
				break;
			}

			if (!memcmp(&vp->variant[i].f, &fp->f, sizeof(fp->f)))
				break;
		}
		assert(i < ARRAY_SIZE(vp->variant));
	}

	/* clear/gmem2mem/mem2gmem need to be changed to remove this condition */
	if (prog != &batch->ctx->solid_prog && prog != &batch->ctx->blit_prog[0]) {
		patch_fetches(ctx, vpi, ctx->vtx.vtx, &ctx->tex[PIPE_SHADER_VERTEX]);
		if (fp)
			patch_fetches(ctx, fpi, NULL, &ctx->tex[PIPE_SHADER_FRAGMENT]);
	}

	emit(batch, ring, vp, i, binning);

	if (fp) {
		emit(batch, ring, fp, 0, false);
		fs_gprs = (fpi->max_reg < 0) ? 0x80 : fpi->max_reg;
		vs_export = MAX2(1, fp->f.inputs_count) - 1;
	}

	vs_gprs = (vpi->max_reg < 0) ? 0x80 : vpi->max_reg;

	enum a2xx_sq_ps_vtx_mode mode = POSITION_1_VECTOR;
	if (vp->writes_psize && !binning)
		mode = POSITION_2_VECTORS_SPRITE;

	/* set register to use for param (fragcoord/pointcoord/frontfacing) */
	OUT_PKT3(ring, CP_SET_CONSTANT, 2);
	OUT_RING(ring, CP_REG(REG_A2XX_SQ_CONTEXT_MISC));
	OUT_RING(ring, A2XX_SQ_CONTEXT_MISC_SC_SAMPLE_CNTL(CENTERS_ONLY) |
		COND(fp, A2XX_SQ_CONTEXT_MISC_PARAM_GEN_POS(fp->f.inputs_count)) |
		/* we need SCREEN_XY for both fragcoord and frontfacing */
		A2XX_SQ_CONTEXT_MISC_SC_OUTPUT_SCREEN_XY);

	OUT_PKT3(ring, CP_SET_CONSTANT, 2);
	OUT_RING(ring, CP_REG(REG_A2XX_SQ_PROGRAM_CNTL));
	OUT_RING(ring, A2XX_SQ_PROGRAM_CNTL_PS_EXPORT_MODE(2) |
			A2XX_SQ_PROGRAM_CNTL_VS_EXPORT_MODE(mode) |
			A2XX_SQ_PROGRAM_CNTL_VS_RESOURCE |
			A2XX_SQ_PROGRAM_CNTL_PS_RESOURCE |
			A2XX_SQ_PROGRAM_CNTL_VS_EXPORT_COUNT(vs_export) |
			A2XX_SQ_PROGRAM_CNTL_PS_REGS(fs_gprs) |
			A2XX_SQ_PROGRAM_CNTL_VS_REGS(vs_gprs) |
			COND(fp && fp->need_param, A2XX_SQ_PROGRAM_CNTL_PARAM_GEN) |
			COND(!fp, A2XX_SQ_PROGRAM_CNTL_GEN_INDEX_VTX));
}

void
fd2_prog_init(struct pipe_context *pctx)
{
	struct fd_context *ctx = fd_context(pctx);
	struct fd_program_stateobj *prog;
	struct fd2_shader_stateobj *so;
	struct ir2_shader_info *info;
	instr_fetch_vtx_t *instr;

	pctx->create_fs_state = fd2_fp_state_create;
	pctx->delete_fs_state = fd2_fp_state_delete;

	pctx->create_vs_state = fd2_vp_state_create;
	pctx->delete_vs_state = fd2_vp_state_delete;

	fd_prog_init(pctx);

	/* XXX maybe its possible to reuse patch_vtx_fetch somehow? */

	prog = &ctx->solid_prog;
	so = prog->vp;
	ir2_compile(prog->vp, 1, &((struct fd2_shader_stateobj*) prog->fp)->f);

#define IR2_FETCH_SWIZ_XY01 0xb08
#define IR2_FETCH_SWIZ_XYZ1 0xa88

	info = &so->variant[1].info;

	instr = (instr_fetch_vtx_t*) &info->dwords[info->fetch_info[0].offset];
	instr->const_index = 26;
	instr->const_index_sel = 0;
	instr->format = FMT_32_32_32_FLOAT;
	instr->format_comp_all = false;
	instr->stride = 12;
	instr->num_format_all = true;
	instr->dst_swiz = IR2_FETCH_SWIZ_XYZ1;

	prog = &ctx->blit_prog[0];
	so = prog->vp;
	ir2_compile(prog->vp, 1, &((struct fd2_shader_stateobj*) prog->fp)->f);

	info = &so->variant[1].info;

	instr = (instr_fetch_vtx_t*) &info->dwords[info->fetch_info[0].offset];
	instr->const_index = 26;
	instr->const_index_sel = 1;
	instr->format = FMT_32_32_FLOAT;
	instr->format_comp_all = false;
	instr->stride = 8;
	instr->num_format_all = false;
	instr->dst_swiz = IR2_FETCH_SWIZ_XY01;

	instr = (instr_fetch_vtx_t*) &info->dwords[info->fetch_info[1].offset];
	instr->const_index = 26;
	instr->const_index_sel = 0;
	instr->format = FMT_32_32_32_FLOAT;
	instr->format_comp_all = false;
	instr->stride = 12;
	instr->num_format_all = false;
	instr->dst_swiz = IR2_FETCH_SWIZ_XYZ1;
}
