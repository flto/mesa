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

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "ir2.h"
#include "fd2_program.h"
#include "instr-a2xx.h"

enum {
	IR2_REG_INPUT = 0x1,
	IR2_REG_CONST = 0x2,
	IR2_REG_NEGATE = 0x4,
	IR2_REG_ABS = 0x8,
};

struct ir2_src {
	uint16_t num;
	uint8_t swizzle, flags;
};

struct ir2_input_reg {
	int8_t alloc;
	uint8_t used_count;
};

struct ir2_reg_component {
	int8_t comp;
	uint8_t ref_count;
};

struct ir2_alloc {
	int8_t reg;
	uint8_t mask;
};

struct ir2_instruction {
	unsigned idx;
	enum {
		IR2_NONE,
		IR2_FETCH,
		IR2_ALU,
	} instr_type;

	struct ir2_src src_reg[4];
	unsigned src_reg_count;

	/* number of output components */
	uint8_t num_components;

	/* components for allocation */
	struct ir2_reg_component components[4];

	/* reg idx for non-ssa */
	int16_t reg_idx;

	/* */
	struct ir2_alloc alloc;
	bool required, emitted;

	union {
		/* FETCH specific: */
		struct {
			instr_fetch_opc_t opc;
			unsigned const_idx;
			/* texture fetch specific: */
			bool is_cube:1;
			bool is_rect:1;
			/* vertex fetch specific: */
			unsigned const_idx_sel;
			enum a2xx_sq_surfaceformat fmt;
			bool is_signed:1;
			bool is_normalized:1;
			uint32_t stride;
			uint32_t offset;

			unsigned idx;
			unsigned samp_id;
		} fetch;
		struct {
			int8_t scalar_opc, vector_opc;
			int8_t export;
			uint8_t write_mask;
			bool saturate;
		} alu;
	};
};

struct ir2_sched_instr {
	uint32_t reg_state[8];
	struct ir2_instruction *instr, *instr_s;
};

struct ir2_context {
	struct fd2_shader_stateobj *so;

	/* ssa index of position output */
	int position;

	/* to translate SSA ids to instruction ids */
	int16_t ssa_map[1024];
	/* to translate reg ids (registers after vec lower) */
	int16_t reg_map[128];

	struct ir2_shader_info *info;

	int prev_export;

	uint32_t reg_state[8];

	struct ir2_input_reg input[8];

	struct ir2_instruction instr[0x300];
	unsigned instr_count;

	struct ir2_sched_instr instr_sched[0x180];
	unsigned instr_sched_count;
};

void substitutions(struct ir2_context *ctx);
void late_substitutions(struct ir2_context *ctx);

void assemble(struct ir2_context *ctx);

bool ir2_nir_vectorize(nir_shader * shader);
bool ir2_nir_lower_scalar(nir_shader * shader);
void ir2_nir_compile(struct ir2_context *ctx);

/* utils */

enum {
	IR2_SWIZZLE_XXXX = 0 << 0 | 3 << 2 | 2 << 4 | 1 << 6,
	IR2_SWIZZLE_XYXY = 0 << 0 | 0 << 2 | 2 << 4 | 2 << 6,
	IR2_SWIZZLE_YYYY = 1 << 0 | 0 << 2 | 3 << 4 | 2 << 6,
	IR2_SWIZZLE_WWWW = 3 << 0 | 2 << 2 | 1 << 4 | 0 << 6,
	IR2_SWIZZLE_WYWW = 3 << 0 | 0 << 2 | 1 << 4 | 0 << 6,
	IR2_SWIZZLE_XYZW = 0
};

#define compile_error(ctx, args...) ({ \
	printf(args); \
	assert(0); \
})

static inline struct ir2_src
ir2_src(uint16_t num, uint8_t swizzle, uint8_t flags)
{
	return (struct ir2_src) {
	.num = num,.swizzle = swizzle,.flags = flags};
}

static inline struct ir2_src ir2_zero(void)
{
	return ir2_src(63, IR2_SWIZZLE_XXXX, IR2_REG_CONST);
}

#define ir2_foreach_instr(it, ctx) \
	for (struct ir2_instruction *it = (ctx)->instr; ({ \
		while (it != &(ctx)->instr[(ctx)->instr_count] && it->instr_type == IR2_NONE) it++; \
		 it != &(ctx)->instr[(ctx)->instr_count]; }); it++)

#define ir2_foreach_avail(it) \
	for (struct ir2_instruction **__instrp = avail, *it; \
		it = *__instrp,  __instrp != &avail[avail_count]; __instrp++)

#define ir2_foreach_src_reg(it, instr) \
	for (struct ir2_src *it = instr->src_reg; \
		 it != &instr->src_reg[instr->src_reg_count]; it++)

/* mask for register allocation
 * 64 registers with 4 components each = 256 bits
 */
/* typedef struct {
	uint64_t data[4];
} regmask_t; */

static inline bool mask_isset(uint32_t * mask, unsigned num)
{
	return ! !(mask[num / 32] & 1 << num % 32);
}

static inline void mask_set(uint32_t * mask, unsigned num)
{
	mask[num / 32] |= 1 << num % 32;
}

static inline void mask_unset(uint32_t * mask, unsigned num)
{
	mask[num / 32] &= ~(1 << num % 32);
}

static inline unsigned mask_reg(uint32_t * mask, unsigned num)
{
	return mask[num / 8] >> num % 8 * 4 & 0xf;
}

static inline bool is_input(struct ir2_src *reg)
{
	return ! !(reg->flags & IR2_REG_INPUT);
}

static inline bool is_const(struct ir2_src *reg)
{
	return ! !(reg->flags & IR2_REG_CONST);
}

static inline bool is_neg(struct ir2_src *reg)
{
	return ! !(reg->flags & IR2_REG_NEGATE);
}

static inline bool is_abs(struct ir2_src *reg)
{
	return ! !(reg->flags & IR2_REG_ABS);
}

static inline bool is_export(struct ir2_instruction *instr)
{
	return instr->instr_type == IR2_ALU && instr->alu.export >= 0;
}

static inline instr_alloc_type_t export_buffer(unsigned num)
{
	return num < 32 ? SQ_PARAMETER_PIXEL :
		num >= 62 ? SQ_POSITION : SQ_MEMORY;
}

static inline unsigned export_order(unsigned num)
{
	switch (export_buffer(num)) {
	default:
		return 0;
	case SQ_POSITION:
		return 1;
	case SQ_MEMORY:
		return 2;
	}
}

/* bitwisk to swizzle c in channel i */
static inline unsigned swiz_set(unsigned c, unsigned i)
{
	return ((c - i) & 3) << i * 2;
}

/* get swizzle in channel i */
static inline unsigned swiz_get(unsigned swiz, unsigned i)
{
	return ((swiz >> i * 2) + i) & 3;
}

static inline unsigned swiz_merge(unsigned swiz0, unsigned swiz1)
{
	unsigned swiz = 0;
	for (int i = 0; i < 4; i++)
		swiz |= swiz_set(swiz_get(swiz0, swiz_get(swiz1, i)), i);
	return swiz;
}

static inline
	struct ir2_alloc *get_alloc(struct ir2_context *ctx,
								struct ir2_instruction *p)
{
	return p->reg_idx < 0 ? &p->alloc : &ctx->instr[p->reg_idx].alloc;
}

static inline struct ir2_reg_component *get_components(struct ir2_context
													   *ctx,
													   struct
													   ir2_instruction *p)
{
	return p->reg_idx <
		0 ? p->components : ctx->instr[p->reg_idx].components;
}

/* for instructions we have "num_components" for the dst regs
 * util to get a num_components value for the src registers
 */
static inline unsigned num_src_comps(struct ir2_instruction *p)
{
	if (p->instr_type == IR2_FETCH) {
		switch (p->fetch.opc) {
		case VTX_FETCH:
			return 1;
		case TEX_FETCH:
			return 3;
		default:
			assert(0);
		}
	}

	assert(p->instr_type == IR2_ALU);

	switch (p->alu.vector_opc) {
	case DOT2ADDv:
		return 2;
	case DOT3v:
		return 3;
	case DOT4v:
		return 4;
	default:
		return p->num_components;
	}
}
