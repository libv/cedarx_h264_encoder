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

#define ALIGN(x, a) (((x) + ((typeof(x))(a) - 1)) & ~((typeof(x))(a) - 1))
#define IS_ALIGNED(x, a) (((x) & ((typeof(x))(a) - 1)) == 0)
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

struct h264enc_context {
	unsigned int mb_width, mb_height, mb_stride;
	unsigned int crop_right, crop_bottom;

	uint32_t bytestream_buffer_phys;
	int bytestream_buffer_size;

	void *regs;

	unsigned int write_sps_pps;

	unsigned int profile_idc, level_idc, constraints;

	unsigned int entropy_coding_mode_flag;
	unsigned int pic_init_qp;

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

static void put_bits(struct h264enc_context *context, uint32_t x, int num)
{
	int i;

#define STATUS_WAIT_COUNT 10000
	for (i = 0; i < STATUS_WAIT_COUNT; i++) {
		uint32_t status = h264enc_read(H264ENC_STATUS);

		if ((status & 0x200) == 0x200)
			break;
	}

	if (i == STATUS_WAIT_COUNT)
		fprintf(stderr, "%s(): bytestream status not cleared.\n",
			__func__);

	h264enc_write(H264ENC_PUTBITSDATA, x);
	h264enc_write(H264ENC_STARTTRIG, 0x1 | ((num & 0x1f) << 8));
	/* again the problem, how to check for finish? */
}

static void put_ue(struct h264enc_context *context, uint32_t x)
{
	x++;
	put_bits(context, x, (32 - __builtin_clz(x)) * 2 - 1);
}

static void put_start_code(struct h264enc_context *context,
			   unsigned int nal_ref_idc, unsigned int nal_unit_type)
{
	uint32_t tmp = h264enc_read(H264ENC_PARA0);

	/* disable emulation_prevention_three_byte */
	h264enc_write(H264ENC_PARA0, tmp | (0x1 << 31));

	put_bits(context, 0, 24);
	put_bits(context, 0x100 | (nal_ref_idc << 5) | (nal_unit_type << 0), 16);

	h264enc_write(H264ENC_PARA0, tmp);
}

static void put_rbsp_trailing_bits(struct h264enc_context *context)
{
	unsigned int cur_bs_len = h264enc_read(H264ENC_STMLEN);

	int num_zero_bits = 8 - ((cur_bs_len + 1) & 0x7);
	put_bits(context, 1 << num_zero_bits, num_zero_bits + 1);
}

static void put_seq_parameter_set(struct h264enc_context *context)
{
	put_start_code(context, 3, 7);

	put_bits(context, context->profile_idc, 8);
	put_bits(context, context->constraints, 8);
	put_bits(context, context->level_idc, 8);

	put_ue(context, /* seq_parameter_set_id = */ 0);

	put_ue(context, /* log2_max_frame_num_minus4 = */ 0);
	put_ue(context, /* pic_order_cnt_type = */ 2);

	put_ue(context, /* max_num_ref_frames = */ 1);
	put_bits(context, /* gaps_in_frame_num_value_allowed_flag = */ 0, 1);

	put_ue(context, context->mb_width - 1);
	put_ue(context, context->mb_height - 1);

	put_bits(context, /* frame_mbs_only_flag = */ 1, 1);

	put_bits(context, /* direct_8x8_inference_flag = */ 0, 1);

	unsigned int frame_cropping_flag = context->crop_right || context->crop_bottom;
	put_bits(context, frame_cropping_flag, 1);
	if (frame_cropping_flag) {
		put_ue(context, 0);
		put_ue(context, context->crop_right);
		put_ue(context, 0);
		put_ue(context, context->crop_bottom);
	}

	put_bits(context, /* vui_parameters_present_flag = */ 0, 1);

	put_rbsp_trailing_bits(context);
}

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

	/* copy parameters */
	context->mb_width = DIV_ROUND_UP(params->width, 16);
	context->mb_height = DIV_ROUND_UP(params->height, 16);
	context->mb_stride = params->src_width / 16;

	context->crop_right = (context->mb_width * 16 - params->width) / 2;
	context->crop_bottom = (context->mb_height * 16 - params->height) / 2;

	context->profile_idc = params->profile_idc;
	context->level_idc = params->level_idc;

	context->entropy_coding_mode_flag = params->entropy_coding_mode ? 1 : 0;
	context->pic_init_qp = params->qp;
	context->keyframe_interval = params->keyframe_interval;

	context->write_sps_pps = 1;
	context->current_frame_num = 0;

	context->bytestream_buffer_phys =
		ve_bytestream_dma_addr_get(&context->bytestream_buffer_size);

	return context;
}

int h264enc_encode_picture(struct h264enc_context *context)
{
	int ret;

	context->current_slice_type = context->current_frame_num ? SLICE_P : SLICE_I;

	/* set output buffer */
	h264enc_write(H264ENC_STMOST, 0);
	h264enc_write(H264ENC_STMSTARTADDR, context->bytestream_buffer_phys);
	h264enc_write(H264ENC_STMENDADDR, context->bytestream_buffer_phys +
		      context->bytestream_buffer_size - 1);
	h264enc_write(H264ENC_STMVSIZE, context->bytestream_buffer_size * 8);

	/* write headers */
	if (context->write_sps_pps) {
		put_seq_parameter_set(context);
		context->write_sps_pps = 0;
	}

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
