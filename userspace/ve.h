/*
 * Copyright (c) 2013 Jens Kuske <jenskuske@gmail.com>
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

#ifndef __VE_H__
#define __VE_H__

#include <stdint.h>

int ve_open(void);
void ve_close(void);
void *ve_mmio_get(void);

void *ve_malloc(int size);
void ve_free(void *ptr);
uint32_t ve_virt2phys(void *ptr);

struct h264enc_params;
int ve_config(struct h264enc_params *params);
int ve_encode(bool frame_type_p);

void *ve_input_buffer_virtual_get(void);
uint32_t ve_input_buffer_dma_addr_get(void);

#endif
