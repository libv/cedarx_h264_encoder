/*
 * Copyright (c) 2014-2015 Jens Kuske <jenskuske@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "h264enc.h"
#include "ve.h"
#include "ve_regs.h"

#define IS_ALIGNED(x, a) (((x) & ((typeof(x))(a) - 1)) == 0)

struct h264enc_context {
	void *regs;

	unsigned int keyframe_interval;

	unsigned int current_frame_num;
	unsigned int frame_count;
	enum slice_type { SLICE_P = 0, SLICE_I = 2 } current_slice_type;

};

#define __maybe_unused  __attribute__((unused))
static void __maybe_unused cedar_io_write(struct h264enc_context *context,
					  int address, uint32_t value)
{
	writel(value, context->regs + address);
}

static uint32_t __maybe_unused cedar_io_read(struct h264enc_context *context,
					     int address)
{
	return readl(context->regs + address);
}

static void __maybe_unused cedar_io_mask(struct h264enc_context *context,
					 int address,
					 uint32_t value, uint32_t mask)
{
	uint32_t temp = readl(context->regs + address);

	temp &= ~mask;
	value &= mask;

	writel(value | temp, context->regs + address);
}

#define h264enc_read(a) \
	cedar_io_read(context, H264ENC_BASE + (a))
#define h264enc_write(a, v) \
	cedar_io_write(context, H264ENC_BASE + (a), (v))
#define h264enc_mask(a, v, m) \
	cedar_io_mask(context, H264ENC_BASE + (a), (v), (m))

void h264enc_free(struct h264enc_context *context)
{
	free(context);
}

struct h264enc_context *
h264enc_new(struct h264enc_params *params)
{
	struct h264enc_context *context;
	int ret;

	/* check parameter validity */
	if (!IS_ALIGNED(params->src_width, 16) || !IS_ALIGNED(params->src_height, 16) ||
	    !IS_ALIGNED(params->width, 2) || !IS_ALIGNED(params->height, 2) ||
	    params->width > params->src_width || params->height > params->src_height) {
		fprintf(stderr, "%s(): invalid picture size.\n", __func__);
		return NULL;
	}

	if (params->qp == 0 || params->qp > 47) {
		fprintf(stderr, "%s(): invalid QP.\n", __func__);
		return NULL;
	}

	if (params->src_format != H264_FMT_NV12 && params->src_format != H264_FMT_NV16) {
		fprintf(stderr, "%s(): invalid color format.\n", __func__);
		return NULL;
	}

	ret = ve_config(params);
	if (ret)
		return NULL;

	/* allocate memory for h264enc structure */
	context = calloc(1, sizeof(struct h264enc_context));
	if (context == NULL) {
		fprintf(stderr, "%s(): can't allocate h264enc context.\n",
			__func__);
		return NULL;
	}

	context->regs = ve_mmio_get();

	context->keyframe_interval = params->keyframe_interval;

	context->current_frame_num = 0;

	return context;
}

int h264enc_encode_picture(struct h264enc_context *context)
{
	int ret;

	context->current_slice_type = context->current_frame_num ? SLICE_P : SLICE_I;

	if (context->current_slice_type == SLICE_P)
		ret = ve_encode(true);
	else
		ret = ve_encode(false);

	/* next frame */
	context->current_frame_num++;
	if (context->current_frame_num >= context->keyframe_interval)
		context->current_frame_num = 0;
	context->frame_count++;

	if (ret < 0) {
		fprintf(stderr, "%s(): %d: encoding failed: %d\n",
			__func__, context->frame_count - 1, ret);
		return ret;
	} else
		printf("\rFrame %5d: %dbytes", context->frame_count, ret);

	return ret;
}
