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

#ifndef __VE_REGS_H__
#define __VE_REGS_H__

static inline void writel(uint32_t val, void *addr)
{
	*((volatile uint32_t *)addr) = val;
}

static inline uint32_t readl(void *addr)
{
	return *((volatile uint32_t *) addr);
}

#define VE_ENGINE_AVC			0xb

#define VE_CTRL				0x000
#define VE_VERSION			0x0f0

#define VE_ISP_INPUT_SIZE		0xa00
#define VE_ISP_INPUT_STRIDE		0xa04
#define VE_ISP_CTRL			0xa08
#define VE_ISP_INPUT_LUMA		0xa78
#define VE_ISP_INPUT_CHROMA		0xa7c

#define H264ENC_PARA0			0xb04
#define H264ENC_PARA1			0xb08
#define H264ENC_MEPARA			0xb10
#define H264ENC_INT_ENABLE		0xb14
#define H264ENC_STARTTRIG		0xb18
#define H264ENC_STATUS			0xb1c
#define H264ENC_PUTBITSDATA		0xb20
#define H264ENC_MVBUFADDR		0xb60
#define H264ENC_STMSTARTADDR		0xb80
#define H264ENC_STMENDADDR		0xb84
#define H264ENC_STMOST			0xb88
#define H264ENC_STMVSIZE		0xb8c
#define H264ENC_STMLEN			0xb90
#define H264ENC_REFADDRY		0xba0
#define H264ENC_REFADDRC		0xba4
#define H264ENC_RECADDRY		0xbb0
#define H264ENC_RECADDRC		0xbb4
#define H264ENC_SUBPIXADDRLAST		0xbb8
#define H264ENC_SUBPIXADDRNEW		0xbbc
#define H264ENC_MBINFO			0xbc0

#endif
