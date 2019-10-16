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
#include "ve_regs.h"

#define DEVICE "/dev/cedar_dev"

enum IOCTL_CMD {
	IOCTL_GET_ENV_INFO = 0x101,
	IOCTL_WAIT_VE_EN = 0x103,
};

struct cedarv_env_infomation {
	unsigned int phymem_start;
	int phymem_total_size;
	unsigned int address_macc;
};

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
	int version;
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

	ret = ioctl(ve.fd, IOCTL_GET_ENV_INFO, (void *)(&info));
	if (ret == -1) {
		fprintf(stderr, "%s(): IOCTL_GET_ENV_INFO failed: %s\n",
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

	writel(0x00130007, ve.regs + VE_CTRL);

	ve.version = readl(ve.regs + VE_VERSION) >> 16;
	printf("[VDPAU SUNXI] VE version 0x%04x opened.\n", ve.version);

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
ve_get_version(void)
{
	return ve.version;
}

int
ve_wait(int timeout)
{
	int ret;

	ret = ioctl(ve.fd, IOCTL_WAIT_VE_EN, timeout);
	if (ret) {
		fprintf(stderr, "%s(): IOCTL_WAIT_VE failed: %s\n",
			__func__, strerror(errno));
		return ret;
	}

	return 0;
}

void *
ve_get(int engine, uint32_t flags)
{
	pthread_mutex_lock(&ve.device_lock);

	writel(0x00130000 | (engine & 0xf) | (flags & ~0xf), ve.regs + VE_CTRL);

	return ve.regs;
}

void
ve_put(void)
{
	writel(0x00130007, ve.regs + VE_CTRL);

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
