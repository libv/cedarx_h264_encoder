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
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "cedar_ioctl.h"

#define ALIGN(x, a) (((x) + ((typeof(x))(a) - 1)) & ~((typeof(x))(a) - 1))

static void *input_luma_buffer;
static int input_luma_size;
static void *input_chroma_buffer;
static int input_chroma_size;

static void *bytestream;
static int bytestream_size;

static int cedar_fd = -1;

static int
ve_config(int width, int height)
{
	struct cedar_ioctl_config config = { 0 };
	int ret;

	config.src_width = width;
	config.src_height = height;

	config.src_format = CEDAR_IOCTL_CONFIG_FORMAT_NV12;

	config.dst_width = ALIGN(width, 16);
	config.dst_height = ALIGN(height, 16);

	config.profile = 77;
	config.level = 41;
	config.qp = 24;
	config.keyframe_interval = 25;

	config.entropy_coding_mode = CEDAR_IOCTL_ENTROPY_CODING_CABAC;

	ret = ioctl(cedar_fd, CEDAR_IOCTL_CONFIG, &config);
	if (ret) {
		fprintf(stderr, "%s(): CEDAR_IOCTL_CONFIG failed: %s\n",
			__func__, strerror(errno));
		return ret;
	}

	input_luma_size = config.input_luma_size;
	input_luma_buffer = mmap(NULL, input_luma_size,
				 PROT_READ | PROT_WRITE,
				 MAP_SHARED, cedar_fd,
				 config.input_luma_dma_addr);
	if (input_luma_buffer == MAP_FAILED) {
		fprintf(stderr, "%s(): failed to mmap input luma buffer: "
			"%s.\n", __func__, strerror(errno));
		return -ENOMEM;
	}

	printf("Input Y: %dbytes at 0x%08X -> %p\n", input_luma_size,
	       config.input_luma_dma_addr, input_luma_buffer);

	input_chroma_size = config.input_chroma_size;
	input_chroma_buffer = mmap(NULL, input_chroma_size,
				   PROT_READ | PROT_WRITE,
				   MAP_SHARED, cedar_fd,
				   config.input_chroma_dma_addr);
	if (input_chroma_buffer == MAP_FAILED) {
		fprintf(stderr, "%s(): failed to mmap input chroma buffer: "
			"%s.\n", __func__, strerror(errno));
		return -ENOMEM;
	}

	printf("Input C: %dbytes at 0x%08X -> %p\n", input_chroma_size,
	       config.input_chroma_dma_addr, input_chroma_buffer);


	bytestream_size = config.bytestream_size;
	bytestream = mmap(NULL, bytestream_size, PROT_READ | PROT_WRITE,
			  MAP_SHARED, cedar_fd, config.bytestream_dma_addr);
	if (bytestream == MAP_FAILED) {
		fprintf(stderr, "%s(): failed to mmap input buffer: %s.\n",
			__func__, strerror(errno));
		return -ENOMEM;
	}

	printf("Bytestream: %dbytes at 0x%08X -> %p\n", bytestream_size,
	       config.bytestream_dma_addr, bytestream);

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

	return total;
}

int main(int argc, char **argv)
{
	uint32_t frame_count = 0;
	int width, height;
	int fd_in, fd_out;
	int luma_size, chroma_size, ret;

	if (argc != 5) {
		printf("Usage: %s <infile> <width> <height> <outfile>\n", argv[0]);
		return -1;
	}

	width = atoi(argv[2]);
	height = atoi(argv[3]);

	cedar_fd = open(CEDAR_DEVICE_PATH, O_RDWR);
	if (cedar_fd == -1) {
		fprintf(stderr, "%s(): failed to open %s: %s\n",
			__func__, CEDAR_DEVICE_PATH, strerror(errno));
		return -1;
	}

	if (strcmp(argv[1], "-")) {
		fd_in = open(argv[1], O_RDONLY | O_LARGEFILE);
		if (fd_in == -1) {
			fprintf(stderr, "%s(): Failed to open input file %s: %s\n",
				__func__, argv[1], strerror(errno));
			return -1;
		}
	} else
		  fd_in = 0;

	fd_out = open(argv[4], O_CREAT | O_RDWR | O_TRUNC,
		      S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
	if (fd_out == -1) {
		fprintf(stderr, "%s(): Failed to open output file %s\n",
			__func__, argv[4]);
		return -1;
	}

	ret = ve_config(width, height);
	if (ret)
		return ret;

	luma_size = width * height;
	chroma_size = luma_size / 2;

	while (1) {
		ret = read_frame(fd_in, input_luma_buffer, luma_size);
		if (ret != luma_size)
			break;
		ret = read_frame(fd_in, input_chroma_buffer, chroma_size);
		if (ret != chroma_size)
			break;

		ret = ioctl(cedar_fd, CEDAR_IOCTL_ENCODE, NULL);
		if (ret < 0)
			fprintf(stderr, "%s(): %d: CEDAR_IOCTL_ENCODE failed: %s\n",
				__func__, frame_count, strerror(errno));
		else {
			printf("\rFrame %5d: %5dbytes", frame_count, ret);
			write(fd_out, bytestream, ret);
			frame_count++;
		}
	}

	return 0;
}
