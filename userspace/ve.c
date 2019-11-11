/*
 * Copyright (c) 2013-2014 Jens Kuske <jenskuske@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdbool.h>

#include "ve.h"
#include "h264enc.h"
#include "cedar_ioctl.h"
#include "ve_regs.h"

#define DEVICE "/dev/cedar_dev"

static void *input_buffer_virtual;
static int input_buffer_size;

static void *bytestream_virtual;
static int bytestream_size;

static int cedar_fd = -1;
static void *cedar_regs;

int ve_open(void)
{
	struct cedarv_env_infomation info;
	int ret;

	cedar_fd = open(DEVICE, O_RDWR);
	if (cedar_fd == -1) {
		fprintf(stderr, "%s(): failed to open %s: %s\n",
			__func__, DEVICE, strerror(errno));
		return 0;
	}

	ret = ioctl(cedar_fd, CEDAR_IOCTL_GET_ENV_INFO, (void *)(&info));
	if (ret == -1) {
		fprintf(stderr, "%s(): CEDAR_IOCTL_GET_ENV_INFO failed: %s\n",
			__func__, strerror(errno));
		goto error;
	}

	cedar_regs = mmap(NULL, 0x800, PROT_READ | PROT_WRITE, MAP_SHARED,
		       cedar_fd, info.address_macc);
	if (cedar_regs == MAP_FAILED) {
		fprintf(stderr,
			"%s(): register mmapping at 0x%08X failed: %s\n",
			__func__, info.address_macc, strerror(errno));
		goto error;
	}

	return 0;

error:
	close(cedar_fd);
	cedar_fd = -1;
	return -1;
}

void
ve_close(void)
{
	munmap(cedar_regs, 0x800);
	cedar_regs = NULL;

	close(cedar_fd);
	cedar_fd = -1;
}

int
ve_encode(bool frame_type_p)
{
	struct cedar_ioctl_encode encode = { 0 };
	int ret;

	if (frame_type_p)
		encode.frame_type = CEDAR_FRAME_TYPE_P;
	else
		encode.frame_type = CEDAR_FRAME_TYPE_I;

	ret = ioctl(cedar_fd, CEDAR_IOCTL_ENCODE, &encode);
	if (ret < 0)
		fprintf(stderr, "%s(): CEDAR_IOCTL_ENCODE failed: %s\n",
			__func__, strerror(errno));

	return ret;
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

void *
ve_mmio_get(void)
{
	return cedar_regs;
}

