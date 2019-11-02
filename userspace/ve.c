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

#include "ve.h"
#include "h264enc.h"
#include "cedar_ioctl.h"
#include "ve_regs.h"

#define DEVICE "/dev/cedar_dev"

#define PAGE_OFFSET (0xc0000000) // from kernel
#define PAGE_SIZE (4096)

struct memchunk_t
{
	uint32_t phys_addr;
	int size;
	void *virt_addr;
	struct memchunk_t *next;
};

static struct
{
	int fd;
	void *regs;
	struct memchunk_t first_memchunk;
	pthread_rwlock_t memory_lock;
	pthread_mutex_t device_lock;
} ve = {
	.fd = -1,
	.memory_lock = PTHREAD_RWLOCK_INITIALIZER,
	.device_lock = PTHREAD_MUTEX_INITIALIZER,
};

int ve_open(void)
{
	struct cedarv_env_infomation info;
	int ret;

	ve.fd = open(DEVICE, O_RDWR);
	if (ve.fd == -1) {
		fprintf(stderr, "%s(): failed to open %s: %s\n",
			__func__, DEVICE, strerror(errno));
		return 0;
	}

	ret = ioctl(ve.fd, CEDAR_IOCTL_GET_ENV_INFO, (void *)(&info));
	if (ret == -1) {
		fprintf(stderr, "%s(): CEDAR_IOCTL_GET_ENV_INFO failed: %s\n",
			__func__, strerror(errno));
		goto error;
	}

	ve.regs = mmap(NULL, 0x800, PROT_READ | PROT_WRITE, MAP_SHARED,
		       ve.fd, info.address_macc);
	if (ve.regs == MAP_FAILED) {
		fprintf(stderr,
			"%s(): register mmapping at 0x%08X failed: %s\n",
			__func__, info.address_macc, strerror(errno));
		goto error;
	}

	ve.first_memchunk.phys_addr = info.phymem_start - PAGE_OFFSET;
	ve.first_memchunk.size = info.phymem_total_size;

	return 0;

error:
	close(ve.fd);
	ve.fd = -1;
	return -1;
}

void
ve_close(void)
{
	munmap(ve.regs, 0x800);
	ve.regs = NULL;

	close(ve.fd);
	ve.fd = -1;
}

int
ve_encode(void)
{
	int ret;

	ret = ioctl(ve.fd, CEDAR_IOCTL_ENCODE, NULL);
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

	ret = ioctl(ve.fd, CEDAR_IOCTL_CONFIG, &config);
	if (ret < 0)
		fprintf(stderr, "%s(): CEDAR_IOCTL_CONFIG failed: %s\n",
			__func__, strerror(errno));

	return ret;
}

void *
ve_get(int engine, uint32_t flags)
{
	pthread_mutex_lock(&ve.device_lock);

	return ve.regs;
}

void
ve_put(void)
{
	pthread_mutex_unlock(&ve.device_lock);
}

void *
ve_malloc(int size)
{
	struct memchunk_t *chunk, *best_chunk = NULL;
	int size_left;
	void *ptr;

	printf("%s(%d);\n", __func__, size);

	pthread_rwlock_wrlock(&ve.memory_lock);

	size = (size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);

	for (chunk = &ve.first_memchunk; chunk; chunk = chunk->next)
		if (!chunk->virt_addr && (chunk->size >= size)) {
			if (!best_chunk || (chunk->size < best_chunk->size))
				best_chunk = chunk;

			if (chunk->size == size)
				break;
		}

	if (!best_chunk) {
		fprintf(stderr, "%s(%d): failed to find free area.\n",
			__func__, size);
		goto error;
	}

	ptr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED,
		    ve.fd, best_chunk->phys_addr + PAGE_OFFSET);
	if (ptr == MAP_FAILED) {
		fprintf(stderr, "%s(%d): failed to find mmap(0x%08X): %s.\n",
			__func__, size, best_chunk->phys_addr,
			strerror(errno));
		goto error;
	}

	size_left = best_chunk->size - size;
	best_chunk->virt_addr = ptr;
	best_chunk->size = size;

	if (size_left > 0) {
		chunk = malloc(sizeof(struct memchunk_t));
		chunk->phys_addr = best_chunk->phys_addr + size;
		chunk->size = size_left;
		chunk->virt_addr = NULL;
		chunk->next = best_chunk->next;
		best_chunk->next = chunk;
	}

	pthread_rwlock_unlock(&ve.memory_lock);
	return ptr;

 error:
	pthread_rwlock_unlock(&ve.memory_lock);
	return NULL;
}

void
ve_free(void *ptr)
{
	struct memchunk_t *chunk, *next;

	if (!ptr)
		return;

	pthread_rwlock_wrlock(&ve.memory_lock);

	for (chunk = &ve.first_memchunk; chunk; chunk = chunk->next)
		if (chunk->virt_addr == ptr) {
			munmap(ptr, chunk->size);
			chunk->virt_addr = NULL;
			break;
		}

	for (chunk = &ve.first_memchunk; chunk; chunk = chunk->next)
		if (!chunk->virt_addr) {
			while ((chunk->next != NULL) && (chunk->next->virt_addr == NULL)) {
				next = chunk->next;
				chunk->size += next->size;
				chunk->next = next->next;
				free(next);
			}
		}

	pthread_rwlock_unlock(&ve.memory_lock);
}

uint32_t
ve_virt2phys(void *ptr)
{
	struct memchunk_t *chunk;
	uint32_t addr = 0;

	if (!ptr)
		return 0;

	if (pthread_rwlock_rdlock(&ve.memory_lock))
		return 0;

	for (chunk = &ve.first_memchunk; chunk; chunk = chunk->next) {
		if (!chunk->virt_addr)
			continue;

		if (chunk->virt_addr == ptr) {
			addr = chunk->phys_addr;
			break;
		} else if ((ptr > chunk->virt_addr) &&
			   (ptr < (chunk->virt_addr + chunk->size))) {
			addr = chunk->phys_addr + (ptr - chunk->virt_addr);
			break;
		}
	}

	pthread_rwlock_unlock(&ve.memory_lock);

	return addr;
}
