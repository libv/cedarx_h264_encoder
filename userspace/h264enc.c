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

#include "h264enc.h"
#include "ve.h"
#include "ve_regs.h"

#define ALIGN(x, a) (((x) + ((typeof(x))(a) - 1)) & ~((typeof(x))(a) - 1))
#define IS_ALIGNED(x, a) (((x) & ((typeof(x))(a) - 1)) == 0)
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

struct h264enc_context {
	unsigned int mb_width, mb_height, mb_stride;
	unsigned int crop_right, crop_bottom;

	uint8_t *luma_buffer, *chroma_buffer;
	unsigned int input_buffer_size;
	enum color_format input_color_format;

	uint8_t *bytestream_buffer;
	unsigned int bytestream_buffer_size;
	unsigned int bytestream_length;

	struct h264enc_ref_pic {
		void *luma_buffer, *chroma_buffer;
		void *extra_buffer; /* unknown purpose, looks like smaller luma */
	} ref_picture[2];

	void *MB_info;
	void *extra_buffer_frame; /* unknown purpose */

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


#define h264isp_read(a) \
	cedar_io_read(context, H264ISP_BASE + (a))
#define h264isp_write(a, v) \
	cedar_io_write(context, H264ISP_BASE + (a), (v))
#define h264isp_mask(a, v, m) \
	cedar_io_mask(context, H264ISP_BASE + (a), (v), (m))

#define h264enc_read(a) \
	cedar_io_read(context, H264ENC_BASE + (a))
#define h264enc_write(a, v) \
	cedar_io_write(context, H264ENC_BASE + (a), (v))
#define h264enc_mask(a, v, m) \
	cedar_io_mask(context, H264ENC_BASE + (a), (v), (m))

static void put_bits(struct h264enc_context *context, uint32_t x, int num)
{
	h264enc_write(H264ENC_PUTBITSDATA, x);
	h264enc_write(H264ENC_STARTTRIG, 0x1 | ((num & 0x1f) << 8));
	/* again the problem, how to check for finish? */
}

static void put_ue(struct h264enc_context *context, uint32_t x)
{
	x++;
	put_bits(context, x, (32 - __builtin_clz(x)) * 2 - 1);
}

static void put_se(struct h264enc_context *context, int x)
{
	x = 2 * x - 1;
	x ^= (x >> 31);
	put_ue(context, x);
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

static void put_pic_parameter_set(struct h264enc_context *context)
{
	put_start_code(context, 3, 8);

	put_ue(context, /* pic_parameter_set_id = */ 0);
	put_ue(context, /* seq_parameter_set_id = */ 0);

	put_bits(context, context->entropy_coding_mode_flag, 1);

	put_bits(context, /* bottom_field_pic_order_in_frame_present_flag = */ 0, 1);
	put_ue(context, /* num_slice_groups_minus1 = */ 0);

	put_ue(context, /* num_ref_idx_l0_default_active_minus1 = */ 0);
	put_ue(context, /* num_ref_idx_l1_default_active_minus1 = */ 0);

	put_bits(context, /* weighted_pred_flag = */ 0, 1);
	put_bits(context, /* weighted_bipred_idc = */ 0, 2);

	put_se(context, (int)context->pic_init_qp - 26);
	put_se(context, (int)context->pic_init_qp - 26);
	put_se(context, /* chroma_qp_index_offset = */ 4);

	put_bits(context, /* deblocking_filter_control_present_flag = */ 1, 1);
	put_bits(context, /* constrained_intra_pred_flag = */ 0, 1);
	put_bits(context, /* redundant_pic_cnt_present_flag = */ 0, 1);

	put_rbsp_trailing_bits(context);
}

static void put_slice_header(struct h264enc_context *context)
{
	if (context->current_slice_type == SLICE_I)
		put_start_code(context, 3, 5);
	else
		put_start_code(context, 2, 1);

	put_ue(context, /* first_mb_in_slice = */ 0);
	put_ue(context, context->current_slice_type);
	put_ue(context, /* pic_parameter_set_id = */ 0);

	put_bits(context, context->current_frame_num & 0xf, 4);

	if (context->current_slice_type == SLICE_I)
		put_ue(context, /* idr_pic_id = */ 0);

	if (context->current_slice_type == SLICE_P) {
		put_bits(context, /* num_ref_idx_active_override_flag = */ 0, 1);
		put_bits(context, /* ref_pic_list_modification_flag_l0 = */ 0, 1);
		put_bits(context, /* adaptive_ref_pic_marking_mode_flag = */ 0, 1);
		if (context->entropy_coding_mode_flag)
			put_ue(context, /* cabac_init_idc = */ 0);
	}

	if (context->current_slice_type == SLICE_I) {
		put_bits(context, /* no_output_of_prior_pics_flag = */ 0, 1);
		put_bits(context, /* long_term_reference_flag = */ 0, 1);
	}

	put_se(context, /* slice_qp_delta = */ 0);

	put_ue(context, /* disable_deblocking_filter_idc = */ 0);
	put_se(context, /* slice_alpha_c0_offset_div2 = */ 0);
	put_se(context, /* slice_beta_offset_div2 = */ 0);
}

void h264enc_free(struct h264enc_context *context)
{
	int i;

	ve_free(context->MB_info);
	ve_free(context->extra_buffer_frame);

	for (i = 0; i < 2; i++) {
		ve_free(context->ref_picture[i].luma_buffer);
		ve_free(context->ref_picture[i].extra_buffer);
	}

	ve_free(context->bytestream_buffer);
	ve_free(context->luma_buffer);
	free(context);
}

struct h264enc_context *
h264enc_new(struct h264enc_params *params)
{
	struct h264enc_context *context;
	int ret, i;

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

	/* allocate input buffer */
	context->input_color_format = params->src_format;
	switch (context->input_color_format) {
	case H264_FMT_NV12:
		context->input_buffer_size = params->src_width * (params->src_height + params->src_height / 2);
		break;
	case H264_FMT_NV16:
		context->input_buffer_size = params->src_width * params->src_height * 2;
		break;
	}

	context->luma_buffer = ve_malloc(context->input_buffer_size);
	if (!context->luma_buffer) {
		fprintf(stderr, "%s(): failed to allocate input buffer.\n",
			__func__);
		goto error;
	}
	context->chroma_buffer = context->luma_buffer + params->src_width * params->src_height;

	/* allocate bytestream output buffer */
	context->bytestream_buffer_size = 1 * 1024 * 1024;
	context->bytestream_buffer = ve_malloc(context->bytestream_buffer_size);
	if (!context->bytestream_buffer) {
		fprintf(stderr, "%s(): failed to allocate bytestream buffer.\n",
			__func__);
		goto error;
	}

	/* allocate reference picture memory */
	unsigned int luma_size = ALIGN(context->mb_width * 16, 32) * ALIGN(context->mb_height * 16, 32);
	unsigned int chroma_size = ALIGN(context->mb_width * 16, 32) * ALIGN(context->mb_height * 8, 32);
	for (i = 0; i < 2; i++) {
		context->ref_picture[i].luma_buffer = ve_malloc(luma_size + chroma_size);
		if (!context->ref_picture[i].luma_buffer) {
			fprintf(stderr, "%s(): failed to allocate reference "
				"picture %d buffer.\n", __func__, i);
			goto error;
		}
		context->ref_picture[i].chroma_buffer = context->ref_picture[i].luma_buffer + luma_size;

		context->ref_picture[i].extra_buffer = ve_malloc(luma_size / 4);
		if (!context->ref_picture[i].extra_buffer) {
			fprintf(stderr, "%s(): failed to allocate reference "
				"picture %d extra buffer.\n", __func__, i);
			goto error;
		}
	}

	/* allocate unknown purpose buffers */
	context->extra_buffer_frame = ve_malloc(ALIGN(context->mb_width, 4) * context->mb_height * 8);
	if (!context->extra_buffer_frame) {
		fprintf(stderr, "%s(): failed to allocate extra buffer frame.\n",
			__func__);
		goto error;
	}

	int size = (context->mb_width << 4) * 8;
	context->MB_info = ve_malloc(size);
	if (!context->MB_info) {
		fprintf(stderr, "%s(): failed to allocate extra buffer line.\n",
			__func__);
		goto error;
	}

	return context;

 error:
	ve_free(context->MB_info);
	ve_free(context->extra_buffer_frame);
	ve_free(context->ref_picture[1].luma_buffer);
	ve_free(context->ref_picture[1].extra_buffer);
	ve_free(context->ref_picture[0].luma_buffer);
	ve_free(context->ref_picture[0].extra_buffer);
	ve_free(context->bytestream_buffer);
	ve_free(context->luma_buffer);
	h264enc_free(context);
	return NULL;
}

void *h264enc_get_input_buffer(struct h264enc_context *context)
{
	return context->luma_buffer;
}

void *h264enc_get_bytestream_buffer(struct h264enc_context *context)
{
	return context->bytestream_buffer;
}

unsigned int h264enc_get_bytestream_length(struct h264enc_context *context)
{
	return context->bytestream_length;
}

int h264enc_encode_picture(struct h264enc_context *context)
{
	context->current_slice_type = context->current_frame_num ? SLICE_P : SLICE_I;

	/* set output buffer */
	h264enc_write(H264ENC_STMOST, 0);
	h264enc_write(H264ENC_STMSTARTADDR, ve_virt2phys(context->bytestream_buffer));
	h264enc_write(H264ENC_STMENDADDR, ve_virt2phys(context->bytestream_buffer) +
		      context->bytestream_buffer_size - 1);
	h264enc_write(H264ENC_STMVSIZE, context->bytestream_buffer_size * 8);

	/* write headers */
	if (context->write_sps_pps) {
		put_seq_parameter_set(context);
		put_pic_parameter_set(context);
		context->write_sps_pps = 0;
	}
	put_slice_header(context);

	/* set input size */
	h264isp_write(H264ISP_STRIDE_CTRL, context->mb_stride << 16);
	h264isp_write(H264ISP_INPUT_SIZE, (context->mb_width << 16) | (context->mb_height << 0));

	/* set input format */
	h264isp_write(H264ISP_CTRL, context->input_color_format << 29);

	/* set input buffer */
	h264isp_write(H264ISP_INPUT_Y_ADDR, ve_virt2phys(context->luma_buffer));
	h264isp_write(H264ISP_INPUT_C0_ADDR, ve_virt2phys(context->chroma_buffer));

	/* set reconstruction buffers */
	struct h264enc_ref_pic *ref_pic = &context->ref_picture[context->current_frame_num % 2];
	h264enc_write(H264ENC_RECADDRY, ve_virt2phys(ref_pic->luma_buffer));
	h264enc_write(H264ENC_RECADDRC, ve_virt2phys(ref_pic->chroma_buffer));
	h264enc_write(H264ENC_SUBPIXADDRNEW, ve_virt2phys(ref_pic->extra_buffer));

	/* set reference buffers */
	if (context->current_slice_type != SLICE_I) {
		ref_pic = &context->ref_picture[(context->current_frame_num + 1) % 2];
		h264enc_write(H264ENC_REFADDRY, ve_virt2phys(ref_pic->luma_buffer));
		h264enc_write(H264ENC_REFADDRC, ve_virt2phys(ref_pic->chroma_buffer));
		h264enc_write(H264ENC_SUBPIXADDRLAST, ve_virt2phys(ref_pic->extra_buffer));
	}

	/* set unknown purpose buffers */
	h264enc_write(H264ENC_MBINFO, ve_virt2phys(context->MB_info));
	h264enc_write(H264ENC_MVBUFADDR, ve_virt2phys(context->extra_buffer_frame));

	/* set encoding parameters */
	uint32_t params = 0x0;
	if (context->entropy_coding_mode_flag)
		params |= 0x100;
	if (context->current_slice_type == SLICE_P)
		params |= 0x10;
	h264enc_write(H264ENC_PARA0, params);
	h264enc_write(H264ENC_PARA1, (4 << 16) | (context->pic_init_qp << 8) | context->pic_init_qp);

	h264enc_write(H264ENC_MEPARA, 0x00000104);

	int ret = ve_encode();
	if (ret >= 0)
		context->bytestream_length = ret;

	printf("\rFrame %5d: %dbytes", context->frame_count, context->bytestream_length);

	/* next frame */
	context->current_frame_num++;
	if (context->current_frame_num >= context->keyframe_interval)
		context->current_frame_num = 0;
	context->frame_count++;

	if (ret < 0) {
		fprintf(stderr, "%s(): %d: encoding failed: %d\n",
			__func__, context->frame_count - 1, ret);
		return ret;
	}

	return 0;
}
