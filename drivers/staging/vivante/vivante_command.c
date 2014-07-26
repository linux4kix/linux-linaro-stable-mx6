/*
 * Copyright (C) 2014 2014 Etnaviv Project
 * Author: Christian Gmeiner <christian.gmeiner@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "vivante_gpu.h"
#include "vivante_gem.h"

#include "common.xml.h"
#include "state.xml.h"
#include "cmdstream.xml.h"

/*
 * Command Buffer helper:
 */


static inline void CMD_LOAD_STATE(struct vivante_ringbuffer *rb, u32 reg, u32 value)
{
	/* write a register via cmd stream */
	OUT_RING(rb, VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE | VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
			VIV_FE_LOAD_STATE_HEADER_OFFSET(reg >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR));
	OUT_RING(rb, value);
}

static inline void CMD_LOAD_STATES(struct vivante_ringbuffer *rb, u32 reg, u16 count, u32 *values)
{
	u16 i;

	OUT_RING(rb, VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE | VIV_FE_LOAD_STATE_HEADER_COUNT(count) |
			VIV_FE_LOAD_STATE_HEADER_OFFSET(reg >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR));

	for (i = 0; i < count; i++)
		OUT_RING(rb, values[i]);
}


static inline void CMD_END(struct vivante_ringbuffer *rb)
{
	OUT_RING(rb, VIV_FE_END_HEADER_OP_END);
}

static inline void CMD_NOP(struct vivante_ringbuffer *rb)
{
	OUT_RING(rb, VIV_FE_NOP_HEADER_OP_NOP);
}

static inline void CMD_WAIT(struct vivante_ringbuffer *rb)
{
	OUT_RING(rb, VIV_FE_WAIT_HEADER_OP_WAIT | 200);
}

static inline void CMD_LINK(struct vivante_ringbuffer *rb, u16 prefetch, u32 address)
{
	OUT_RING(rb, VIV_FE_LINK_HEADER_OP_LINK | VIV_FE_LINK_HEADER_PREFETCH(prefetch));
	OUT_RING(rb, address);
}

static inline void CMD_STALL(struct vivante_ringbuffer *rb, u32 from, u32 to)
{
	OUT_RING(rb, VIV_FE_STALL_HEADER_OP_STALL);
	OUT_RING(rb, VIV_FE_STALL_TOKEN_FROM(from) | VIV_FE_STALL_TOKEN_TO(to));
}

static void vivante_cmd_select_pipe(struct vivante_ringbuffer *rb, u8 pipe)
{
	u32 flush;
	u32 stall;

	if (pipe == VIVANTE_PIPE_2D)
		flush = VIVS_GL_FLUSH_CACHE_DEPTH | VIVS_GL_FLUSH_CACHE_COLOR;
	else
		flush = VIVS_GL_FLUSH_CACHE_TEXTURE;

	stall = VIVS_GL_SEMAPHORE_TOKEN_FROM(SYNC_RECIPIENT_FE) |
			VIVS_GL_SEMAPHORE_TOKEN_TO(SYNC_RECIPIENT_PE);

	CMD_LOAD_STATE(rb, VIVS_GL_FLUSH_CACHE, flush);
	CMD_LOAD_STATE(rb, VIVS_GL_SEMAPHORE_TOKEN, stall);

	CMD_STALL(rb, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_PE);

	CMD_LOAD_STATE(rb, VIVS_GL_PIPE_SELECT, VIVS_GL_PIPE_SELECT_PIPE(pipe));
}

static void vivante_cmd_dump(struct vivante_gem_object *obj, u32 len)
{
	u32 size = obj->base.size;
	u32 *ptr = vivante_gem_vaddr_locked(&obj->base);

	printk(KERN_INFO "free: 0x%08x\n", size - len * 4);

	print_hex_dump(KERN_INFO, "cmd ", DUMP_PREFIX_OFFSET, 16, 4,
			ptr, len * 4 * 4, 0);
}

u32 vivante_cmd_init(struct vivante_gpu *gpu)
{
	/* initialize ringbuffer */
	gpu->rb->written = 0;

	vivante_cmd_select_pipe(gpu->rb, gpu->pipe);

	CMD_WAIT(gpu->rb);
	CMD_LINK(gpu->rb, 1, vivante_gem_paddr_locked(gpu->rb->bo) + ((gpu->rb->written - 1) * 4));
	gpu->rb->ll = gpu->rb->cur - 1;

	return gpu->rb->written;
}
