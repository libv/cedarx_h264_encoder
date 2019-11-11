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

#define IS_ALIGNED(x, a) (((x) & ((typeof(x))(a) - 1)) == 0)

static uint32_t frame_count;

int
h264enc_new(struct h264enc_params *params)
{
	int ret;

	/* check parameter validity */
	if (!IS_ALIGNED(params->src_width, 16) || !IS_ALIGNED(params->src_height, 16) ||
	    !IS_ALIGNED(params->width, 2) || !IS_ALIGNED(params->height, 2) ||
	    params->width > params->src_width || params->height > params->src_height) {
		fprintf(stderr, "%s(): invalid picture size.\n", __func__);
		return -1;
	}

	if (params->qp == 0 || params->qp > 47) {
		fprintf(stderr, "%s(): invalid QP.\n", __func__);
		return -1;
	}

	if (params->src_format != H264_FMT_NV12 && params->src_format != H264_FMT_NV16) {
		fprintf(stderr, "%s(): invalid color format.\n", __func__);
		return -1;
	}

	ret = ve_config(params);
	if (ret)
		return ret;

	return 0;
}

int h264enc_encode_picture(void)
{
	int ret;

	ret = ve_encode();
	if (ret < 0) {
		fprintf(stderr, "%s(): %d: encoding failed: %d\n",
			__func__, frame_count, ret);
		return ret;
	} else
		printf("\rFrame %5d: %dbytes", frame_count, ret);

	frame_count++;

	return ret;
}
