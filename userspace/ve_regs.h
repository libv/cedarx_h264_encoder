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

/*
 * Image signal processor
 */
#define H264ISP_BASE		0xA00

#define H264ISP_INPUT_SIZE		0x00
#define H264ISP_STRIDE_CTRL		0x04
#define H264ISP_CTRL			0x08
#define H264ISP_TRIG			0x0C
#define H264ISP_INT_STATUS		0x10
#define H264ISP_MB_STRIDE		0x14

#define H264ISP_SCALER_CTRL		0x2C
#define H264ISP_SCALER_YPHASE		0x30
#define H264ISP_SCALER_CPHASE		0x34
#define H264ISP_SCALER_RATIO		0x38

#define H264ISP_THUMB_Y_ADDR		0x70
#define H264ISP_THUMB_C_ADDR		0x74
#define H264ISP_INPUT_Y_ADDR		0x78
#define H264ISP_INPUT_C0_ADDR		0x7C
#define H264ISP_INPUT_C1_ADDR		0x80

#define H264ISP_COEFF_SRAM_INDEX	0xE0
#define H264ISP_COEFF_SRAM_DATA		0xE4

/*
 * H.264 encoder.
 */
#define H264ENC_BASE		0xB00

#define H264ENC_PICINFO			0x00
#define H264ENC_PARA0			0x04
#define H264ENC_PARA1			0x08
#define H264ENC_PARA2			0x0C
#define H264ENC_MEPARA			0x10
#define H264ENC_INT_ENABLE		0x14
#define H264ENC_STARTTRIG		0x18
#define H264ENC_STATUS			0x1C
#define H264ENC_PUTBITSDATA		0x20
#define H264ENC_OVERTIMEMB		0x24
#define H264ENC_CYCLICINTRAREFRESH	0x28
#define H264ENC_RC_INIT			0x2C
#define H264ENC_RC_MAD_TH0		0x30
#define H264ENC_RC_MAD_TH1		0x34
#define H264ENC_RC_MAD_TH2		0x38
#define H264ENC_RC_MAD_TH3		0x3C
#define H264ENC_RC_MAD_SUM		0x40
#define H264ENC_TEMPORAL_FILTER_PAR	0x44
#define H264ENC_DYNAMIC_ME_PAR0		0x48
#define H264ENC_DYNAMIC_ME_PAR1		0x4C
#define H264ENC_MAD			0x50
#define H264ENC_TXTBITS			0x54
#define H264ENC_HDRBITS			0x58
#define H264ENC_MEINFO			0x5C
#define H264ENC_MVBUFADDR		0x60

#define H264ENC_STMSTARTADDR		0x80
#define H264ENC_STMENDADDR		0x84
#define H264ENC_STMOST			0x88
#define H264ENC_STMVSIZE		0x8C
#define H264ENC_STMLEN			0x90

#define H264ENC_REFADDRY		0xA0
#define H264ENC_REFADDRC		0xA4
#define H264ENC_REF1ADDRY		0xA8
#define H264ENC_REF1ADDRC		0xAC
#define H264ENC_RECADDRY		0xB0
#define H264ENC_RECADDRC		0xB4
#define H264ENC_SUBPIXADDRLAST		0xB8
#define H264ENC_SUBPIXADDRNEW		0xBC
#define H264ENC_MBINFO			0xC0
#define H264ENC_DBLKADDR		0xC4
#define H264ENC_TFCNT_ADDR		0xC8
#define H264ENC_ROI_QP_OFFSET		0xCC
#define H264ENC_ROI_0_AREA		0xD0
#define H264ENC_ROI_1_AREA		0xD4
#define H264ENC_ROI_2_AREA		0xD8
#define H264ENC_ROI_3_AREA		0xDC
#define H264ENC_SRAM_OFFSET		0xE0
#define H264ENC_SRAM_DATA		0xE4

#endif
