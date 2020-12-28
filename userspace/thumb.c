/*
 * Copyright (c) 2020 Luc Verhaegen <libv@skynet.be>
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
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "cedar_ioctl.h"

#define ALIGN(x, a) (((x) + ((typeof(x))(a) - 1)) & ~((typeof(x))(a) - 1))

#define DEFAULT_WIDTH 64
#define DEFAULT_HEIGHT 32

static int cedar_fd = -1;

static int input_width;
static int input_height;
static int input_pitch;
static uint8_t *input_luma;
static size_t input_luma_size;
static uint8_t *input_chroma;
static size_t input_chroma_size;

static int
cedar_config(int width, int height)
{
	struct cedar_ioctl_config config[1] = {{
		.src_width = width,
		.src_height = height,
		.src_format = CEDAR_IOCTL_CONFIG_FORMAT_NV12,
		.dst_width = ALIGN(width, 16),
		.dst_height = ALIGN(height, 16),
		.profile = 77,
		.level = 41,
		.qp = 24,
		.keyframe_interval = 25,
		.entropy_coding_mode = CEDAR_IOCTL_ENTROPY_CODING_CABAC,
	}};
	int ret;

	ret = ioctl(cedar_fd, CEDAR_IOCTL_CONFIG, config);
	if (ret) {
		fprintf(stderr, "Error:%s():ioctl(CONFIG): %s.\n",
			__func__, strerror(errno));
		return errno;
	}

	input_width = width;
	input_height = height;
	input_pitch = width;

	input_luma_size = config->input_luma_size;
	input_luma = mmap(NULL, input_luma_size, PROT_READ | PROT_WRITE,
			  MAP_SHARED, cedar_fd, config->input_luma_dma_addr);
	if (input_luma == MAP_FAILED) {
		fprintf(stderr, "Error: %s(): mmap(luma): %s.\n",
			__func__, strerror(errno));
		return errno;
	}
	printf("Input Y: 0x%08X -> %p (%tdbytes).\n",
	       config->input_luma_dma_addr, input_luma, input_luma_size);

	input_chroma_size = config->input_chroma_size;
	input_chroma = mmap(NULL, input_chroma_size, PROT_READ | PROT_WRITE,
			  MAP_SHARED, cedar_fd, config->input_chroma_dma_addr);
	if (input_chroma == MAP_FAILED) {
		fprintf(stderr, "Error: %s(): mmap(chroma): %s.\n",
			__func__, strerror(errno));
		return errno;
	}
	printf("Input C: 0x%08X -> %p (%tdbytes).\n",
	       config->input_chroma_dma_addr, input_chroma, input_chroma_size);

	return 0;
}

static void
input_fill(void)
{
	int offset, x, y;

	/* ITU REC.709 black luminance */
	memset(input_luma, 0x10, input_luma_size);

	/* Set luminance to full on edge pixels. */
	offset = 0;
	for (y = 0; y < input_height; y++) {
		for (x = 0; x < input_width; x++) {
			if ((y < 4) ||
			    (y >= (input_height - 4)) ||
			    (x < 4) ||
			    (x >= (input_width - 4)))
				input_luma[offset + x] = 0xEB;
		}
		offset += input_pitch;
	}

	/* encode our position in all pixels' chrominance */
	offset = 0;
	for (y = 0; y < input_height; y += 2) {
		for (x = 0; x < input_width; x += 2) {
			input_chroma[offset + x] = x & 0xFF;
			input_chroma[offset + x + 1] = y & 0xFF;
		}
		offset += input_pitch;
	}
}

static void
buffer_print(const char *name, uint8_t *luma, int luma_size,
	     uint8_t *chroma, int chroma_size,
	     int width, int height, int pitch)
{
	int offset, x, y, i;

	/* print to verify our creation */
	printf("%s Y (%d(%d)x%d):\n", name, width, pitch, height);
	offset = 0;
	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++) {
			if (!(x & 0x1F)) {
				if (!x)
					printf(" %3d:\t", y);
				else
					printf("\t");
				for (i = 0; i < (x >> 5); i++)
					printf(" ");
			}
			printf(" %02X", luma[offset + x]);

			if ((x & 0x1F) == 0x1F)
				printf("\n");
		}
		if (x & 0x1F)
			printf("\n");
		offset += pitch;
	}

	printf("%s UV (%d(%d)x%d):\n", name, width, pitch, height);
	offset = 0;
	for (y = 0; y < height; y += 2) {
		for (x = 0; x < width; x += 2) {
			if (!(x & 0x1F)) {
				if (!x)
					printf(" %3d:\t", y);
				else
					printf("\t");
				for (i = 0; i < (x >> 5); i++)
					printf(" ");
			}
			printf(" %02X%02X",
			       chroma[offset + x + 1],
			       chroma[offset + x]);

			if ((x & 0x1F) == 0x1E)
				printf("\n");
		}
		if (x & 0x1F)
			printf("\n");
		offset += pitch;
	}
}

int main(int argc, char *argv[])
{
	int ret;

	cedar_fd = open(CEDAR_DEVICE_PATH, O_RDWR);
	if (cedar_fd == -1) {
		fprintf(stderr, "Error: %s():open(%s): %s\n",
			__func__, CEDAR_DEVICE_PATH, strerror(errno));
		return errno;
	}

	ret = cedar_config(DEFAULT_WIDTH, DEFAULT_HEIGHT);
	if (ret) {
		fprintf(stderr, "Error: %s(): cedar_config(): %s.\n",
			__func__, strerror(-ret));
		return ret;
	}

	input_fill();

	buffer_print("Input", input_luma, input_luma_size, input_chroma,
		     input_chroma_size, input_width, input_height, input_pitch);

	return 0;
}
