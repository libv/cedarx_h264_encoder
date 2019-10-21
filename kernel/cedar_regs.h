#ifndef _CEDAR_REGS_H_
#define _CEDAR_REGS_H_ 1

#define CEDAR_H264ISP_BASE	0xA00


#define CEDAR_H264ENC_BASE	0xB00

#define CEDAR_H264ENC_PICINFO			0x00
#define CEDAR_H264ENC_PARA0			0x04
#define CEDAR_H264ENC_PARA1			0x08
#define CEDAR_H264ENC_PARA2			0x0C
#define CEDAR_H264ENC_MEPARA			0x10
#define CEDAR_H264ENC_INT_ENABLE		0x14
#define CEDAR_H264ENC_STARTTRIG			0x18
#define CEDAR_H264ENC_INT_STATUS		0x1C
#define CEDAR_H264ENC_PUTBITSDATA		0x20
#define CEDAR_H264ENC_OVERTIMEMB		0x24
#define CEDAR_H264ENC_CYCLICINTRAREFRESH	0x28
#define CEDAR_H264ENC_RC_INIT			0x2C
#define CEDAR_H264ENC_RC_MAD_TH0		0x30
#define CEDAR_H264ENC_RC_MAD_TH1		0x34
#define CEDAR_H264ENC_RC_MAD_TH2		0x38
#define CEDAR_H264ENC_RC_MAD_TH3		0x3C
#define CEDAR_H264ENC_RC_MAD_SUM		0x40
#define CEDAR_H264ENC_TEMPORAL_FILTER_PAR	0x44
#define CEDAR_H264ENC_DYNAMIC_ME_PAR0		0x48
#define CEDAR_H264ENC_DYNAMIC_ME_PAR1		0x4C
#define CEDAR_H264ENC_MAD			0x50
#define CEDAR_H264ENC_TXTBITS			0x54
#define CEDAR_H264ENC_HDRBITS			0x58
#define CEDAR_H264ENC_MEINFO			0x5C
#define CEDAR_H264ENC_MVBUFADDR			0x60

#define CEDAR_H264ENC_STMSTARTADDR		0x80
#define CEDAR_H264ENC_STMENDADDR		0x84
#define CEDAR_H264ENC_STMOST			0x88
#define CEDAR_H264ENC_STMVSIZE			0x8C
#define CEDAR_H264ENC_STMLEN			0x90

#define CEDAR_H264ENC_REFADDRY			0xA0
#define CEDAR_H264ENC_REFADDRC			0xA4
#define CEDAR_H264ENC_REF1ADDRY			0xA8
#define CEDAR_H264ENC_REF1ADDRC			0xAC
#define CEDAR_H264ENC_RECADDRY			0xB0
#define CEDAR_H264ENC_RECADDRC			0xB4
#define CEDAR_H264ENC_SUBPIXADDRLAST		0xB8
#define CEDAR_H264ENC_SUBPIXADDRNEW		0xBC
#define CEDAR_H264ENC_MBINFO			0xC0
#define CEDAR_H264ENC_DBLKADDR			0xC4
#define CEDAR_H264ENC_TFCNT_ADDR		0xC8
#define CEDAR_H264ENC_ROI_QP_OFFSET		0xCC
#define CEDAR_H264ENC_ROI_0_AREA		0xD0
#define CEDAR_H264ENC_ROI_1_AREA		0xD4
#define CEDAR_H264ENC_ROI_2_AREA		0xD8
#define CEDAR_H264ENC_ROI_3_AREA		0xDC
#define CEDAR_H264ENC_SRAM_OFFSET		0xE0
#define CEDAR_H264ENC_SRAM_DATA			0xE4

#endif /* _CEDAR_REGS_H_ */
