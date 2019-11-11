/*
 * Copyright (c) 2013-2015 Jens Kuske <jenskuske@gmail.com>
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
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "cedar_ioctl.h"

#define DEVICE "/dev/cedar_dev"

#define IS_ALIGNED(x, a) (((x) & ((typeof(x))(a) - 1)) == 0)

static void *input_buffer_virtual;
static int input_buffer_size;

static void *bytestream_virtual;
static int bytestream_size;

static int cedar_fd = -1;

struct h264enc_params {
	unsigned int width;
	unsigned int height;

	unsigned int src_width;
	unsigned int src_height;
	enum color_format { H264_FMT_NV12 = 0, H264_FMT_NV16 = 1 } src_format;

	unsigned int profile_idc, level_idc;

	enum { H264_EC_CAVLC = 0, H264_EC_CABAC = 1 } entropy_coding_mode;

	unsigned int qp;

	unsigned int keyframe_interval;
};

int ve_open(void)
{
	cedar_fd = open(DEVICE, O_RDWR);
	if (cedar_fd == -1) {
		fprintf(stderr, "%s(): failed to open %s: %s\n",
			__func__, DEVICE, strerror(errno));
		return -1;
	}

	return 0;
}

void
ve_close(void)
{
	close(cedar_fd);
	cedar_fd = -1;
}

int ve_config(struct h264enc_params *params)
{
	struct cedar_ioctl_config config = { 0 };
	int ret;

	config.src_width = params->src_width;
	config.src_height = params->src_height;

	if (params->src_format == H264_FMT_NV16)
		config.src_format = CEDAR_IOCTL_CONFIG_FORMAT_NV16;
	else
		config.src_format = CEDAR_IOCTL_CONFIG_FORMAT_NV12;

	config.dst_width = params->width;
	config.dst_height = params->height;

	config.profile = params->profile_idc;
	config.level = params->level_idc;
	config.qp = params->qp;
	config.keyframe_interval = params->keyframe_interval;

	if (params->entropy_coding_mode == H264_EC_CABAC)
		config.entropy_coding_mode = CEDAR_IOCTL_ENTROPY_CODING_CABAC;
	else
		config.entropy_coding_mode = CEDAR_IOCTL_ENTROPY_CODING_CAVLC;

	ret = ioctl(cedar_fd, CEDAR_IOCTL_CONFIG, &config);
	if (ret) {
		fprintf(stderr, "%s(): CEDAR_IOCTL_CONFIG failed: %s\n",
			__func__, strerror(errno));
		return ret;
	}

	input_buffer_size = config.input_size;
	input_buffer_virtual =
		mmap(NULL, input_buffer_size, PROT_READ | PROT_WRITE,
		     MAP_SHARED, cedar_fd, config.input_dma_addr);
	if (input_buffer_virtual == MAP_FAILED) {
		fprintf(stderr, "%s(): failed to mmap input buffer: %s.\n",
			__func__, strerror(errno));
		return -ENOMEM;
	}

	printf("Input: %dbytes at 0x%08X -> %p\n", input_buffer_size,
	       config.input_dma_addr, input_buffer_virtual);

	bytestream_size = config.bytestream_size;
	bytestream_virtual =
		mmap(NULL, bytestream_size, PROT_READ | PROT_WRITE,
		     MAP_SHARED, cedar_fd, config.bytestream_dma_addr);
	if (bytestream_virtual == MAP_FAILED) {
		fprintf(stderr, "%s(): failed to mmap input buffer: %s.\n",
			__func__, strerror(errno));
		return -ENOMEM;
	}

	printf("Bytestream: %dbytes at 0x%08X -> %p\n", bytestream_size,
	       config.bytestream_dma_addr, bytestream_virtual);

	return 0;
}

void *
ve_input_buffer_virtual_get(void)
{
	return input_buffer_virtual;
}

void *
ve_bytestream_virtual_get(void)
{
	return bytestream_virtual;
}

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
	uint32_t frame_count = 0;
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
		ret = ioctl(cedar_fd, CEDAR_IOCTL_ENCODE, NULL);
		if (ret < 0)
			fprintf(stderr, "%s(): %d: CEDAR_IOCTL_ENCODE failed: %s\n",
				__func__, frame_count, strerror(errno));
		else {
			printf("\rFrame %5d: %5dbytes", frame_count, ret);
			write(out, output_buf, ret);
			frame_count++;
		}
	}

	return EXIT_SUCCESS;
}
