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

static bool
need_result(struct ir2_instruction *p, bool param_export, bool mem_export)
{
	if (p->instr_type == IR2_FETCH)
		return false;

	/* instructions that have side effects */
	switch (p->alu.scalar_opc) {
	case KILLEs:
	case KILLNEs:
		return true;
	}

	if (p->alu.export < 0)
		return false;

	if (!param_export && p->alu.export < 32)
		return false;

	if (!mem_export && (p->alu.export == 32 || p->alu.export == 33))
		return false;

	return true;
}

static unsigned reg_alloc(struct ir2_context *ctx)
{
	//assert(ctx->regmask != ~0ull);
	//unsigned idx;
	//idx = ffsll(~ctx->regmask) - 1;
	//ctx->regmask |= (1ull << idx);

	unsigned idx;
	for (idx = 0; idx < 64; idx++) {
		if (!mask_isset(ctx->reg_state, idx * 4 + 0) &&
			!mask_isset(ctx->reg_state, idx * 4 + 1) &&
			!mask_isset(ctx->reg_state, idx * 4 + 2) &&
			!mask_isset(ctx->reg_state, idx * 4 + 3))
			break;
	}
	assert(idx != 64);

	ctx->info->max_reg = MAX2(ctx->info->max_reg, (int) idx);

	return idx;
}

static unsigned reg_alloc_input(struct ir2_context *ctx, unsigned idx)
{
	mask_set(ctx->reg_state, idx * 4 + 0);
	mask_set(ctx->reg_state, idx * 4 + 1);
	mask_set(ctx->reg_state, idx * 4 + 2);
	mask_set(ctx->reg_state, idx * 4 + 3);

	ctx->info->max_reg = MAX2(ctx->info->max_reg, (int) idx);

	return idx;
}

static void
update_refs(struct ir2_context *ctx, bool param_export, bool mem_export)
{
	struct ir2_instruction *p;

	/* initializations */
	ir2_foreach_instr(instr, ctx) {
		instr->required = 0;
		instr->emitted = 0;
		instr->alloc.reg = -1;
		instr->alloc.mask = 0;

		for (int i = 0; i < 4; i++) {
			instr->components[i].comp = is_export(instr) ? i : -1;
			instr->components[i].ref_count = 0;
		}
	}

	/* mark instructions as required */
	for (int i = ctx->instr_count; i-- > 0;) {
		struct ir2_instruction *instr = &ctx->instr[i];

		if (need_result(instr, param_export, mem_export))
			instr->required = true;

		if (instr->reg_idx >= 0) {
			p = &ctx->instr[instr->reg_idx];
			if (p->required)
				instr->required = true;
		}

		if (!instr->required) {
			instr->instr_type = IR2_NONE;
			continue;
		}

		ir2_foreach_src_reg(reg, instr) {
			if (is_const(reg) || is_input(reg))
				continue;

			p = &ctx->instr[reg->num];
			p->required = true;
		}
	}

	/* compute per-component ref_counts */
	ir2_foreach_instr(instr, ctx) {
		unsigned nc = num_src_comps(instr);

		if (is_export(instr) && instr->alu.export < 32)
			ctx->info->max_export =
				MAX2(ctx->info->max_export, instr->alu.export);

		ir2_foreach_src_reg(reg, instr) {
			if (is_const(reg))
				continue;

			if (is_input(reg)) {
				assert(reg->num < ARRAY_SIZE(ctx->input));
				ctx->input[reg->num].used_count++;
				continue;
			}

			p = &ctx->instr[reg->num];

			for (int i = 0; i < nc; i++) {
				unsigned c = swiz_get(reg->swizzle, i);
				assert(c < p->num_components || p->reg_idx >= 0);
				p->components[c].ref_count++;
			}
		}
	}
}

static void comp_free(struct ir2_context *ctx, struct ir2_instruction *p,
					  unsigned i)
{
	struct ir2_alloc *alloc = get_alloc(ctx, p);
	struct ir2_reg_component *c = &p->components[i];

	assert(c->comp >= 0);
	assert(alloc->reg >= 0);
	assert(c->ref_count);
	assert(alloc->mask & 1 << c->comp);

	c->ref_count--;
	if (c->ref_count)
		return;

	alloc->mask &= ~(1 << c->comp);
	mask_unset(ctx->reg_state, alloc->reg * 4 + c->comp);
}

static void instr_ra_update(struct ir2_context *ctx,
							struct ir2_instruction *instr)
{
	struct ir2_instruction *p;
	struct ir2_alloc *alloc;
	unsigned nc;

	nc = num_src_comps(instr);

	ir2_foreach_src_reg(reg, instr) {
		if (is_const(reg))
			continue;

		// XXX input not using per component RA
		if (is_input(reg)) {
			struct ir2_input_reg *r = &ctx->input[reg->num];
			assert(r->used_count);
			--r->used_count;
			if (!r->used_count) {
				mask_unset(ctx->reg_state, r->alloc * 4 + 0);
				mask_unset(ctx->reg_state, r->alloc * 4 + 1);
				mask_unset(ctx->reg_state, r->alloc * 4 + 2);
				mask_unset(ctx->reg_state, r->alloc * 4 + 3);
			}
			continue;
		}

		p = &ctx->instr[reg->num];

		for (int i = 0; i < nc; i++) {
			unsigned c = swiz_get(reg->swizzle, i);
			assert(c < p->num_components || p->reg_idx >= 0);
			comp_free(ctx, p, c);
		}
	}

	if (is_export(instr))
		return;

	alloc = get_alloc(ctx, instr);

	if (alloc->reg < 0 || alloc->mask == 0)
		alloc->reg = reg_alloc(ctx);

	struct ir2_reg_component *comp =
		instr->reg_idx <
		0 ? instr->components : ctx->instr[instr->reg_idx].components;
	int j = 0;

	for (int i = 0; i < instr->num_components; j++, comp++) {
		assert(j < 4);

		if (instr->instr_type == IR2_ALU
			&& !(instr->alu.write_mask & 1 << j))
			continue;

		if (!comp->ref_count) {
			i++;
			continue;
		}

		assert(alloc->mask != 0xf);
		assert(comp->comp == -1);
		comp->comp = ffs(~alloc->mask) - 1;

		assert(!mask_isset(ctx->reg_state, alloc->reg * 4 + comp->comp));

		alloc->mask |= 1 << comp->comp;
		mask_set(ctx->reg_state, alloc->reg * 4 + comp->comp);
		i++;
	}
}

static bool scalar_possible(struct ir2_instruction *instr)
{
	if (instr->alu.scalar_opc < 0)
		return false;

	return instr->num_components == 1;
}

static bool is_compatible(struct ir2_instruction *a,
						  struct ir2_instruction *b)
{
	if (!a)
		return true;

	if (a == b)
		return false;

	return (is_export(a) == is_export(b));
}

/* do required magic to convert a 2 src instr to a scalar instr */
static bool
insert_scalar(struct ir2_context *ctx, struct ir2_instruction *instr)
{
	// XXX to be implemented
	if (is_input(&instr->src_reg[0]) || is_input(&instr->src_reg[1]))
		return false;

	// XXX to be implemented
	if (is_const(&instr->src_reg[0]) && is_const(&instr->src_reg[1]))
		return false;

	/* move the result of the 2nd reg into the first reg
	 * so that scalar operation is possible
	 * TODO when they are both registers we can be smarter
	 * -try both possible orders
	 * -redirect directly to avoid MOV
	 * TODO if there is an empty vec instr, use vector mov
	 */

	struct ir2_src *a, *b;
	struct ir2_alloc *alloc;
	struct ir2_sched_instr *sched = NULL, *s;
	int order = is_const(&instr->src_reg[0]);
	unsigned i, mask = 0xf;
	struct ir2_instruction *p;

	a = &instr->src_reg[order];
	b = &instr->src_reg[!order];

	p = &ctx->instr[a->num];
	alloc = get_alloc(ctx, p);

	// XXX to be implemented
	if (is_neg(a) || is_abs(a))
		return false;

	/* go first earliest point where the mov can be inserted */
	for (i = ctx->instr_sched_count; i > 0; i--) {
		s = &ctx->instr_sched[i - 1];

		if (!is_const(b)) {
			if (s->instr && s->instr->idx == b->num)
				break;
			if (s->instr_s && s->instr_s->idx == b->num)
				break;
		}

		unsigned mr = ~mask_reg(s->reg_state, alloc->reg);
		if ((mask & mr) == 0)
			break;

		mask &= mr;
		if (!s->instr_s && s->instr->src_reg_count != 3
			&& s->instr->instr_type == IR2_ALU && s->instr->alu.export < 0)
			sched = s;
	}
	if (!sched)
		return false;

	unsigned idx;
	assert(ctx->instr_count < ARRAY_SIZE(ctx->instr));
	struct ir2_instruction *x = &ctx->instr[idx = ctx->instr_count++];
	x->idx = idx;
	x->instr_type = IR2_ALU;
	x->src_reg[0] = *b;
	x->src_reg_count = 1;
	x->num_components = 1;
	x->reg_idx = -1;
	x->alu.scalar_opc = MAXs;
	x->alu.export = -1;
	x->alu.write_mask = 1;

	if (!is_const(b)) {
		struct ir2_instruction *p;
		p = &ctx->instr[b->num];

		unsigned c = swiz_get(b->swizzle, 0);
		assert(c < p->num_components || p->reg_idx >= 0);
		comp_free(ctx, p, c);

		/*
		   struct ir2_reg_component *c = &p->components[swiz_get(b->swizzle, 0)];
		   assert(*c->ref_count_p);
		   (*c->ref_count_p)--;
		   if (!*c->ref_count_p) {
		   get_alloc(ctx, p)->mask &= ~(1 << c->comp);
		   mask_unset(ctx->reg_state, c->comp + get_alloc(ctx, p)->alloc*4);
		   } */
	}

	x->components[0].comp = ffs(mask) - 1;
	x->components[0].ref_count = 1;

	unsigned reg = alloc->reg;
	alloc = get_alloc(ctx, x);
	alloc->reg = reg;
	assert(!(alloc->mask & 1 << x->components[0].comp));
	alloc->mask |= (1 << x->components[0].comp);

	sched->instr_s = x;

	b->num = x->idx;
	b->flags = 0;
	b->swizzle = 0;


	x->emitted = true;

	// XXX one loop too many?
	do {
		mask_set(sched->reg_state, x->components[0].comp + alloc->reg * 4);
	} while (++sched != &ctx->instr_sched[ctx->instr_sched_count]);

	return true;
}

/* scheduling: determine order of instructions */
static void schedule_instrs(struct ir2_context *ctx)
{
	struct ir2_instruction *instr_v, *instr_s;
	struct ir2_instruction *avail[0x100];
	struct ir2_sched_instr *sched;
	unsigned avail_count;
	bool no_instr;

	struct ir2_instruction *p;

	for (;; ctx->instr_sched_count++) {
		sched = &ctx->instr_sched[ctx->instr_sched_count];
		avail_count = 0;
		no_instr = true;

		/* find instructions that can be emitted */
		ir2_foreach_instr(instr, ctx) {
			if (instr->emitted)
				continue;

			no_instr = false;

			bool is_ok = true;
			unsigned nc = num_src_comps(instr);
			ir2_foreach_src_reg(reg, instr) {
				if (is_const(reg) || is_input(reg))
					continue;

				p = &ctx->instr[reg->num];

				for (int i = 0; i < nc; i++) {
					unsigned c = swiz_get(reg->swizzle, i);
					assert(c < p->num_components || p->reg_idx >= 0);
					if (p->components[c].comp < 0)
						is_ok = false;
				}
			}
			if (!is_ok)
				continue;

			/* check if export is compatible with current export */
			if (is_export(instr)) {
				unsigned export = instr->alu.export;
				if (ctx->prev_export == 32 && export != 33)
					continue;
				if (ctx->prev_export == 33 && export != 32)
					continue;

				bool compatible = true;
				ir2_foreach_instr(b, ctx) {
					if (b->emitted)
						continue;

					if (!is_export(b)) {
						if (export == 32 || export == 33) {
							compatible = false;
							break;
						}
						// XXX exports in same CF ?
						//continue;
						compatible = false;
						break;
					}

					if (export_order(instr->alu.export) >
						export_order(b->alu.export)) {
						compatible = false;
						break;
					}
				}

				if (!compatible)
					continue;

				ctx->prev_export = instr->alu.export;
			}

			avail[avail_count++] = instr;

			//XXX
			if (is_export(instr)
				&& export_buffer(instr->alu.export) == SQ_MEMORY)
				break;
		}

		if (!avail_count) {
			assert(no_instr);
			break;
		}

		ir2_foreach_avail(instr) {
			if (instr->instr_type != IR2_FETCH)
				continue;

			instr->emitted = true;
			sched->instr = instr;
			sched->instr_s = NULL;

			instr_ra_update(ctx, instr);
			goto continue_loop;
		}

		instr_v = NULL;
		instr_s = NULL;

		/* TODO the scheduling could be better.. */

		/* fill instr_v */
#define avail_match_v(res, cond) ({ \
	ir2_foreach_avail(instr) { \
	if (instr->alu.vector_opc >= 0 && (cond)) { \
		res = instr; \
		if (!is_export(instr)); break; \
	} \
} res; })
		do {
			/* pick vector instruction from 3 src set */
			if (avail_match_v(instr_v, instr->src_reg_count == 3))
				break;

			/* pick vector instruction from non-scalarizable set */
			if (avail_match_v(instr_v, !scalar_possible(instr)))
				break;

			/* pick vector instruction from 2 src set */
			if (avail_match_v(instr_v, instr->src_reg_count == 2))
				break;

			/* pick any vector instruction */
			avail_match_v(instr_v, 1);
		} while (0);

		/* fill instr_s */
		if (!instr_v || instr_v->src_reg_count < 3) {
			do {
				/* first try scalar only op */
				ir2_foreach_avail(instr) {
					if (!is_compatible(instr_v, instr))
						continue;
					if (instr->alu.vector_opc >= 0)
						continue;
					instr_s = instr;
					break;
				}
				if (instr_s)
					break;

				ir2_foreach_avail(instr) {
					if (!is_compatible(instr_v, instr)
						|| !scalar_possible(instr))
						continue;

					if (instr->src_reg_count == 2
						&& !insert_scalar(ctx, instr))
						continue;

					instr_s = instr;
					break;
				}
			} while (0);
		}

		/* TODO merge RA update so instr_v can use reg freed by instr_s */
		if (instr_v) {
			instr_v->emitted = true;
			instr_ra_update(ctx, instr_v);
		}

		if (instr_s) {
			instr_s->emitted = true;
			instr_ra_update(ctx, instr_s);
		}

		assert(instr_v || instr_s);

		sched->instr = instr_v;
		sched->instr_s = instr_s;
	  continue_loop:;
		memcpy(sched->reg_state, ctx->reg_state, sizeof(ctx->reg_state));
	};
}

void ir2_compile(struct fd2_shader_stateobj *so, unsigned variant)
{
	struct ir2_context ctx = { };
	struct ir2_shader_info *info = &so->info[variant];

	ctx.so = so;
	ctx.info = info;

	info->max_reg = -1;
	info->max_export = -1;

	/* convert nir to internal representation */
	ir2_nir_compile(&ctx);

	/* remove movs used for loading inputs/constants/uniforms */
	substitutions(&ctx);

	/* compute dependencies..? (do some work with nir?) */
	update_refs(&ctx, !variant, variant);

	/* remove movs used to write outputs */
	late_substitutions(&ctx);

	/* allocate input registers */
	memset(ctx.reg_state, 0, sizeof(ctx.reg_state));
	for (int i = 0; i < ARRAY_SIZE(ctx.input); i++) {
		if (ctx.input[i].used_count)
			ctx.input[i].alloc = reg_alloc_input(&ctx, i);
	}

	/* instruction order.. and vector->scalar conversions */
	schedule_instrs(&ctx);

	/* validation */
	ir2_foreach_instr(instr, &ctx) {
		for (int i = 0; i < 4; i++)
			assert(instr->components[i].ref_count == 0);
	}
	for (int i = 0; i < ARRAY_SIZE(ctx.reg_state); i++)
		assert(ctx.reg_state[i] == 0);

	/* finally, assemble to bitcode */
	assemble(&ctx);
}
