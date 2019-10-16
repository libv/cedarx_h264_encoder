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

#define MSG(x) fprintf(stderr, "h264enc: " x "\n")

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

	void *extra_buffer_line, *extra_buffer_frame; /* unknown purpose */

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

static void put_bits(void* regs, uint32_t x, int num)
{
	writel(x, regs + VE_AVC_BASIC_BITS);
	writel(0x1 | ((num & 0x1f) << 8), regs + VE_AVC_TRIGGER);
	/* again the problem, how to check for finish? */
}

static void put_ue(void* regs, uint32_t x)
{
	x++;
	put_bits(regs, x, (32 - __builtin_clz(x)) * 2 - 1);
}

static void put_se(void* regs, int x)
{
	x = 2 * x - 1;
	x ^= (x >> 31);
	put_ue(regs, x);
}

static void put_start_code(void* regs, unsigned int nal_ref_idc, unsigned int nal_unit_type)
{
	uint32_t tmp = readl(regs + VE_AVC_PARAM);

	/* disable emulation_prevention_three_byte */
	writel(tmp | (0x1 << 31), regs + VE_AVC_PARAM);

	put_bits(regs, 0, 24);
	put_bits(regs, 0x100 | (nal_ref_idc << 5) | (nal_unit_type << 0), 16);

	writel(tmp, regs + VE_AVC_PARAM);
}

static void put_rbsp_trailing_bits(void* regs)
{
	unsigned int cur_bs_len = readl(regs + VE_AVC_VLE_LENGTH);

	int num_zero_bits = 8 - ((cur_bs_len + 1) & 0x7);
	put_bits(regs, 1 << num_zero_bits, num_zero_bits + 1);
}

static void put_seq_parameter_set(struct h264enc_context *context)
{
	put_start_code(context->regs, 3, 7);

	put_bits(context->regs, context->profile_idc, 8);
	put_bits(context->regs, context->constraints, 8);
	put_bits(context->regs, context->level_idc, 8);

	put_ue(context->regs, /* seq_parameter_set_id = */ 0);

	put_ue(context->regs, /* log2_max_frame_num_minus4 = */ 0);
	put_ue(context->regs, /* pic_order_cnt_type = */ 2);

	put_ue(context->regs, /* max_num_ref_frames = */ 1);
	put_bits(context->regs, /* gaps_in_frame_num_value_allowed_flag = */ 0, 1);

	put_ue(context->regs, context->mb_width - 1);
	put_ue(context->regs, context->mb_height - 1);

	put_bits(context->regs, /* frame_mbs_only_flag = */ 1, 1);

	put_bits(context->regs, /* direct_8x8_inference_flag = */ 0, 1);

	unsigned int frame_cropping_flag = context->crop_right || context->crop_bottom;
	put_bits(context->regs, frame_cropping_flag, 1);
	if (frame_cropping_flag) {
		put_ue(context->regs, 0);
		put_ue(context->regs, context->crop_right);
		put_ue(context->regs, 0);
		put_ue(context->regs, context->crop_bottom);
	}

	put_bits(context->regs, /* vui_parameters_present_flag = */ 0, 1);

	put_rbsp_trailing_bits(context->regs);
}

static void put_pic_parameter_set(struct h264enc_context *context)
{
	put_start_code(context->regs, 3, 8);

	put_ue(context->regs, /* pic_parameter_set_id = */ 0);
	put_ue(context->regs, /* seq_parameter_set_id = */ 0);

	put_bits(context->regs, context->entropy_coding_mode_flag, 1);

	put_bits(context->regs, /* bottom_field_pic_order_in_frame_present_flag = */ 0, 1);
	put_ue(context->regs, /* num_slice_groups_minus1 = */ 0);

	put_ue(context->regs, /* num_ref_idx_l0_default_active_minus1 = */ 0);
	put_ue(context->regs, /* num_ref_idx_l1_default_active_minus1 = */ 0);

	put_bits(context->regs, /* weighted_pred_flag = */ 0, 1);
	put_bits(context->regs, /* weighted_bipred_idc = */ 0, 2);

	put_se(context->regs, (int)context->pic_init_qp - 26);
	put_se(context->regs, (int)context->pic_init_qp - 26);
	put_se(context->regs, /* chroma_qp_index_offset = */ 4);

	put_bits(context->regs, /* deblocking_filter_control_present_flag = */ 1, 1);
	put_bits(context->regs, /* constrained_intra_pred_flag = */ 0, 1);
	put_bits(context->regs, /* redundant_pic_cnt_present_flag = */ 0, 1);

	put_rbsp_trailing_bits(context->regs);
}

static void put_slice_header(struct h264enc_context *context)
{
	if (context->current_slice_type == SLICE_I)
		put_start_code(context->regs, 3, 5);
	else
		put_start_code(context->regs, 2, 1);

	put_ue(context->regs, /* first_mb_in_slice = */ 0);
	put_ue(context->regs, context->current_slice_type);
	put_ue(context->regs, /* pic_parameter_set_id = */ 0);

	put_bits(context->regs, context->current_frame_num & 0xf, 4);

	if (context->current_slice_type == SLICE_I)
		put_ue(context->regs, /* idr_pic_id = */ 0);

	if (context->current_slice_type == SLICE_P) {
		put_bits(context->regs, /* num_ref_idx_active_override_flag = */ 0, 1);
		put_bits(context->regs, /* ref_pic_list_modification_flag_l0 = */ 0, 1);
		put_bits(context->regs, /* adaptive_ref_pic_marking_mode_flag = */ 0, 1);
		if (context->entropy_coding_mode_flag)
			put_ue(context->regs, /* cabac_init_idc = */ 0);
	}

	if (context->current_slice_type == SLICE_I) {
		put_bits(context->regs, /* no_output_of_prior_pics_flag = */ 0, 1);
		put_bits(context->regs, /* long_term_reference_flag = */ 0, 1);
	}

	put_se(context->regs, /* slice_qp_delta = */ 0);

	put_ue(context->regs, /* disable_deblocking_filter_idc = */ 0);
	put_se(context->regs, /* slice_alpha_c0_offset_div2 = */ 0);
	put_se(context->regs, /* slice_beta_offset_div2 = */ 0);
}

void h264enc_free(struct h264enc_context *context)
{
	int i;

	ve_free(context->extra_buffer_line);
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
	int i;

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

	/* allocate memory for h264enc structure */
	context = calloc(1, sizeof(struct h264enc_context));
	if (context == NULL) {
		fprintf(stderr, "%s(): can't allocate h264enc context.\n",
			__func__);
		return NULL;
	}

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
	context->extra_buffer_line = ve_malloc(context->mb_width * 32);
	if (!context->extra_buffer_line) {
		fprintf(stderr, "%s(): failed to allocate extra buffer line.\n",
			__func__);
		goto error;
	}

	return context;

 error:
	ve_free(context->extra_buffer_line);
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

	context->regs = ve_get(VE_ENGINE_AVC, 0);

	/* set output buffer */
	writel(0x0, context->regs + VE_AVC_VLE_OFFSET);
	writel(ve_virt2phys(context->bytestream_buffer), context->regs + VE_AVC_VLE_ADDR);
	writel(ve_virt2phys(context->bytestream_buffer) + context->bytestream_buffer_size - 1, context->regs + VE_AVC_VLE_END);
	writel(context->bytestream_buffer_size * 8, context->regs + VE_AVC_VLE_MAX);

	/* write headers */
	if (context->write_sps_pps) {
		put_seq_parameter_set(context);
		put_pic_parameter_set(context);
		context->write_sps_pps = 0;
	}
	put_slice_header(context);

	/* set input size */
	writel(context->mb_stride << 16, context->regs + VE_ISP_INPUT_STRIDE);
	writel((context->mb_width << 16) | (context->mb_height << 0), context->regs + VE_ISP_INPUT_SIZE);

	/* set input format */
	writel(context->input_color_format << 29, context->regs + VE_ISP_CTRL);

	/* set input buffer */
	writel(ve_virt2phys(context->luma_buffer), context->regs + VE_ISP_INPUT_LUMA);
	writel(ve_virt2phys(context->chroma_buffer), context->regs + VE_ISP_INPUT_CHROMA);

	/* set reconstruction buffers */
	struct h264enc_ref_pic *ref_pic = &context->ref_picture[context->current_frame_num % 2];
	writel(ve_virt2phys(ref_pic->luma_buffer), context->regs + VE_AVC_REC_LUMA);
	writel(ve_virt2phys(ref_pic->chroma_buffer), context->regs + VE_AVC_REC_CHROMA);
	writel(ve_virt2phys(ref_pic->extra_buffer), context->regs + VE_AVC_REC_SLUMA);

	/* set reference buffers */
	if (context->current_slice_type != SLICE_I) {
		ref_pic = &context->ref_picture[(context->current_frame_num + 1) % 2];
		writel(ve_virt2phys(ref_pic->luma_buffer), context->regs + VE_AVC_REF_LUMA);
		writel(ve_virt2phys(ref_pic->chroma_buffer), context->regs + VE_AVC_REF_CHROMA);
		writel(ve_virt2phys(ref_pic->extra_buffer), context->regs + VE_AVC_REF_SLUMA);
	}

	/* set unknown purpose buffers */
	writel(ve_virt2phys(context->extra_buffer_line), context->regs + VE_AVC_MB_INFO);
	writel(ve_virt2phys(context->extra_buffer_frame), context->regs + VE_AVC_UNK_BUF);

	/* enable interrupt and clear status flags */
	writel(readl(context->regs + VE_AVC_CTRL) | 0xf, context->regs + VE_AVC_CTRL);
	writel(readl(context->regs + VE_AVC_STATUS) | 0x7, context->regs + VE_AVC_STATUS);

	/* set encoding parameters */
	uint32_t params = 0x0;
	if (context->entropy_coding_mode_flag)
		params |= 0x100;
	if (context->current_slice_type == SLICE_P)
		params |= 0x10;
	writel(params, context->regs + VE_AVC_PARAM);
	writel((4 << 16) | (context->pic_init_qp << 8) | context->pic_init_qp, context->regs + VE_AVC_QP);
	writel(0x00000104, context->regs + VE_AVC_MOTION_EST);

	/* trigger encoding */
	writel(0x8, context->regs + VE_AVC_TRIGGER);
	ve_wait(1);

	/* check result */
	uint32_t status = readl(context->regs + VE_AVC_STATUS);
	writel(status, context->regs + VE_AVC_STATUS);

	/* save bytestream length */
	context->bytestream_length = readl(context->regs + VE_AVC_VLE_LENGTH) / 8;

	printf("\rFrame %5d, size 0x%04X, status 0x%08X",
	       context->frame_count, context->bytestream_length, status);

	/* next frame */
	context->current_frame_num++;
	if (context->current_frame_num >= context->keyframe_interval)
		context->current_frame_num = 0;
	context->frame_count++;

	ve_put();

	if ((status & 0x3) != 0x1) {
		fprintf(stderr, "%s(): decoding failed: status 0x%02X\n",
			__func__, status);
		return -1;
	}

	return 0;
}
