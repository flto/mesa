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

static bool is_single_mov(struct ir2_instruction *instr)
{
	return instr->instr_type == IR2_ALU && instr->alu.vector_opc == MAXv &&
		instr->src_reg_count == 1 && instr->reg_idx < 0;
}

static unsigned merge_flags(unsigned flags, unsigned flags_2)
{
	assert(!(flags_2 & IR2_REG_CONST));
	assert(!(flags_2 & IR2_REG_INPUT));
	/* abs cancels previous negate */
	if (flags_2 & IR2_REG_ABS)
		flags &= ~IR2_REG_NEGATE, flags |= IR2_REG_ABS;
	/* negate inverses previous negate */
	if (flags_2 & IR2_REG_NEGATE)
		flags ^= IR2_REG_NEGATE;

	return flags;
}

/* substitutions: replace src regs when they refer to a mov instruction
 * example:
 *	ALU:      MAXv    R7 = C7, C7
 *	ALU:      MULADDv R7 = R7, R10, R0.xxxx
 * becomes:
 *	ALU:      MULADDv R7 = C7, R10, R0.xxxx
 */
void substitutions(struct ir2_context *ctx)
{
	struct ir2_instruction *p;

	ir2_foreach_instr(instr, ctx) {
		ir2_foreach_src_reg(reg, instr) {
			do {
				if (is_const(reg) || is_input(reg))
					break;

				p = &ctx->instr[reg->num];
				if (!is_single_mov(p))
					break;

				/* cant apply abs to const reg
				 * XXX verify if const neg works or not
				 */
				if ((is_abs(reg) || is_neg(reg)) && is_const(p->src_reg))
					break;

				reg->num = p->src_reg[0].num;
				reg->swizzle =
					swiz_merge(p->src_reg[0].swizzle, reg->swizzle);
				reg->flags = merge_flags(p->src_reg[0].flags, reg->flags);
			} while (1);
		}
	}
}

/* late substitution: redirect directly to export when possible
 * in the substitution pass we bypass any mov instructions related
 * to the src registers, but for exports for need something different
 * example:
 *	ALU:      MAXv    R3.x___ = C9.x???, C9.x???
 *	ALU:      MAXv    R3._y__ = R0.?x??, C8.?x??
 *	ALU:      MAXv    export0 = R3.yyyx, R3.yyyx
 * becomes:
 *	ALU:      MAXv    export0.___w = C9.???x, C9.???x
 *	ALU:      MAXv    export0.xyz_ = R0.xxx?, C8.xxx?
 *
 * TODO: also do it in non-export cases
 */
void late_substitutions(struct ir2_context *ctx)
{
	struct ir2_instruction *p;
	struct ir2_src *reg;

	/* TODO there might be some cases where a partial redirect is possible */

	ir2_foreach_instr(instr, ctx) {
		if (!is_export(instr) || !is_single_mov(instr))
			continue;

		reg = &instr->src_reg[0];

		if (is_const(reg) || is_input(reg))
			continue;

		/* TODO add logic for this */
		if (is_neg(reg) || is_abs(reg))
			continue;

		p = &ctx->instr[reg->num];

		/* FETCH can't write to export */
		if (p->instr_type == IR2_FETCH)
			continue;

		bool redirect = true;
		unsigned cnt[4] = { };

		/* source components only used by this export */
		for (int i = 0; i < instr->num_components; i++)
			cnt[swiz_get(reg->swizzle, i)]++;

		for (int i = 0; i < 4; i++)
			redirect &= (p->components[i].ref_count == cnt[i]);

		/* no FETCH instructions writing to the register */
		ir2_foreach_instr(ins, ctx) {
			if (ins != p && (ins->reg_idx != p->reg_idx || p->reg_idx < 0))
				continue;
			redirect &= ins->instr_type != IR2_FETCH;
		}

		if (!redirect)
			continue;

		/* redirect the instructions writing to the register */
		ir2_foreach_instr(ins, ctx) {
			if (ins != p && (ins->reg_idx != p->reg_idx || p->reg_idx < 0))
				continue;

			struct ir2_reg_component *comp = get_components(ctx, ins);

			unsigned swiz = 0;
			for (int i = 0, k = 0; i < ins->num_components; k++) {
				if (!(ins->alu.write_mask & 1 << k))
					continue;
				swiz |= swiz_set(i++, k);
				comp[k].comp = k;
			}
			ins->alu.export = instr->alu.export;
		}

		/* set ref_counts to zero and export instruction to emitted */
		for (int i = 0; i < 4; i++)
			p->components[i].ref_count = 0;

		instr->emitted = true;
	}
}
