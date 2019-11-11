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

#define _GNU_SOURCE

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdbool.h>

#include "h264enc.h"
#include "ve.h"

#define IS_ALIGNED(x, a) (((x) & ((typeof(x))(a) - 1)) == 0)

static uint32_t frame_count;

static int
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

static int
h264enc_encode_picture(void)
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

static int
read_frame(int fd, void *buffer, int size)
{
	int total = 0, len;

	while (total < size) {
		len = read(fd, buffer + total, size - total);
		if (len == 0)
			return -1;
		total += len;
	}

	return 0;
}

int main(int argc, char **argv)
{
	struct h264enc_params params;
	void *input_buf, *output_buf;
	int width, height, input_size, ret;
	int in, out;

	if (argc != 5) {
		printf("Usage: %s <infile> <width> <height> <outfile>\n", argv[0]);
		return EXIT_FAILURE;
	}

	width = atoi(argv[2]);
	height = atoi(argv[3]);

	ret = ve_open();
	if (ret) {
		fprintf(stderr, "%s(): ve_open() failed.\n", __func__);
		return EXIT_FAILURE;
	}

	if (strcmp(argv[1], "-")) {
		in = open(argv[1], O_RDONLY | O_LARGEFILE);
		if (in == -1) {
			fprintf(stderr,
				"%s(): Failed to open input file %s: %s\n",
				__func__, argv[1], strerror(errno));
			return EXIT_FAILURE;
		}
	} else
		  in = 0;

	out = open(argv[4], O_CREAT | O_RDWR | O_TRUNC,
		   S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
	if (out == -1) {
		fprintf(stderr, "%s(): Failed to open output file %s\n",
			__func__, argv[4]);
		return EXIT_FAILURE;
	}

	params.src_width = (width + 15) & ~15;
	params.width = width;
	params.src_height = (height + 15) & ~15;
	params.height = height;
	params.src_format = H264_FMT_NV12;
	params.profile_idc = 77;
	params.level_idc = 41;
	params.entropy_coding_mode = H264_EC_CABAC;
	params.qp = 24;
	params.keyframe_interval = 25;

	input_size = params.src_width * (params.src_height + params.src_height / 2);

	ret = h264enc_new(&params);
	if (ret) {
		fprintf(stderr, "%s: could not create context\n", __func__);
		return EXIT_FAILURE;
	}

	input_buf = ve_input_buffer_virtual_get();
	output_buf = ve_bytestream_virtual_get();

	while (!read_frame(in, input_buf, input_size)) {
		ret = h264enc_encode_picture();
		if (ret <= 0)
			fprintf(stderr, "%s: encoding error.\n", __func__);
		else
			write(out, output_buf, ret);
	}

	return EXIT_SUCCESS;
}
