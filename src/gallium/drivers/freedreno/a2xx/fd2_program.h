/*
 * Copyright (C) 2012-2013 Rob Clark <robclark@freedesktop.org>
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
 */

#ifndef FD2_PROGRAM_H_
#define FD2_PROGRAM_H_

#include "pipe/p_context.h"

#include "freedreno_context.h"

#include "ir2.h"
#include "disasm.h"

struct fd2_fs_info {
	unsigned inputs_count;
	struct {
		uint8_t slot;
		uint8_t ncomp;
	} inputs[16];

	/* driver_location of fragcoord.zw, -1 if not used */
	int fragcoord;
};

struct fd2_shader_stateobj {
	nir_shader *nir;
	enum shader_t type;
	bool is_a20x;

	unsigned first_immediate;     /* const reg # of first immediate */
	unsigned num_immediates;
	struct {
		uint32_t val[4];
		unsigned ncomp;
	} immediates[64];

	bool writes_psize;
	bool need_param;

	struct fd2_fs_info f;
	struct {
		struct ir2_shader_info info;
		struct fd2_fs_info f;
	} variant[8];
};

void fd2_program_emit(struct fd_batch *batch, struct fd_ringbuffer *ring,
		struct fd_program_stateobj *prog);

void fd2_prog_init(struct pipe_context *pctx);

#endif /* FD2_PROGRAM_H_ */
