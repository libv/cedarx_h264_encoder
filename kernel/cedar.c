// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus VPU driver
 *
 * Actual bitbanging and such:
 *
 * Copyright (c) 2014-2015 Jens Kuske <jenskuske@gmail.com>
 * Copyright (c) 2019 Luc Verhaegen <libv@skynet.be>
 *
 * Cedrus resources code:
 *
 * Copyright (C) 2016 Florent Revest <florent.revest@free-electrons.com>
 * Copyright (C) 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright (C) 2018 Bootlin
 *
 * Based on the vim2m driver, that is:
 *
 * Copyright (c) 2009-2010 Samsung Electronics Co., Ltd.
 * Pawel Osciak, <pawel@osciak.com>
 * Marek Szyprowski, <m.szyprowski@samsung.com>
 */
/*
 * Contains glue code for handling devicetree and things.
 */
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/uaccess.h>
#include <linux/soc/sunxi/sunxi_sram.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>

#include "cedar_ioctl.h"
#include "cedar_regs.h"

#define MODULE_NAME "sunxi_cedar"

struct sunxi_cedar {
	struct device *dev;

	/* for our /dev/ node */
	struct cdev cdev;
	struct class *class;
	struct device *slashdev;
	dev_t majorminor;

	bool powered;

	struct clk *mod_clk;
	struct clk *ahb_clk;
	struct clk *ram_clk;

	struct reset_control *rstc;

	void __iomem *mmio;
	struct resource *mmio_resource;

	bool interrupt_received;
	wait_queue_head_t wait_queue;
	uint32_t int_status;

	uint64_t time_open;
	uint64_t time_waiting;

	bool configured;
	int src_width;
	int src_height;
	int src_format;
	int src_width_mb;
	int src_height_mb;
	int src_stride_mb;

	int dst_width;
	int dst_height;
	int dst_width_mb;
	int dst_height_mb;
	int dst_width_crop;
	int dst_height_crop;

	int profile;
	int level;
	int qp;
	int keyframe_interval;
	int frame_p_count;
	uint32_t frame_count;

	bool entropy_coding_mode_cabac;

	size_t input_size;
	void *input_luma_virtual;
	dma_addr_t input_luma_dma_addr;
	dma_addr_t input_chroma_dma_addr;

	struct cedar_reference_frame {
		void *luma_virtual;
		dma_addr_t luma_dma_addr;
		size_t luma_size;

		dma_addr_t chroma_dma_addr;
		size_t chroma_size;

		void *subpic_virtual;
		dma_addr_t subpic_dma_addr;
		size_t subpic_size;
	} reference_frame[2];
	struct cedar_reference_frame *reference_current;
	struct cedar_reference_frame *reference_previous;

	/* macroblock info buffer */
	void *mb_info_virtual;
	dma_addr_t mb_info_dma_addr;
	size_t mb_info_size;

	/* motion vector buffer */
	void *mv_buffer_virtual;
	dma_addr_t mv_buffer_dma_addr;
	size_t mv_buffer_size;

	/* h.264 encoded bytestream */
	void *bytestream_virtual;
	dma_addr_t bytestream_dma_addr;
	size_t bytestream_size;
};

#define CEDRUS_CLOCK_RATE_DEFAULT 320000000

static void __maybe_unused cedar_io_write(struct sunxi_cedar *cedar,
					  int address, uint32_t value)
{
	writel(value, cedar->mmio + address);
}

static uint32_t __maybe_unused cedar_io_read(struct sunxi_cedar *cedar,
					     int address)
{
	return readl(cedar->mmio + address);
}

static void __maybe_unused cedar_io_mask(struct sunxi_cedar *cedar,
					 int address,
					 uint32_t value, uint32_t mask)
{
	uint32_t temp = readl(cedar->mmio + address);

	temp &= ~mask;
	value &= mask;

	writel(value | temp, cedar->mmio + address);
}

#define cedarenc_read(a) \
	cedar_io_read(cedar, CEDAR_H264ENC_BASE + (a))
#define cedarenc_write(a, v) \
	cedar_io_write(cedar, CEDAR_H264ENC_BASE + (a), (v))
#define cedarenc_mask(a, v, m) \
	cedar_io_mask(cedar, CEDAR_H264ENC_BASE + (a), (v), (m))

#define cedarisp_read(a) \
	cedar_io_read(cedar, CEDAR_H264ISP_BASE + (a))
#define cedarisp_write(a, v) \
	cedar_io_write(cedar, CEDAR_H264ISP_BASE + (a), (v))
#define cedarisp_mask(a, v, m) \
	cedar_io_mask(cedar, CEDAR_H264ISP_BASE + (a), (v), (m))

static void
cedar_bytestream_write(struct sunxi_cedar *cedar,
		       uint32_t data, int size)
{
	int i;

#define STATUS_WAIT_COUNT 10000
	for (i = 0; i < STATUS_WAIT_COUNT; i++) {
		uint32_t status = cedarenc_read(CEDAR_H264ENC_INT_STATUS);

		if ((status & 0x200) == 0x200)
			break;
	}

	if (i == STATUS_WAIT_COUNT)
		dev_err(cedar->dev, "%s(): bytestream status not cleared.\n",
			__func__);

	cedarenc_write(CEDAR_H264ENC_PUTBITSDATA, data);
	cedarenc_write(CEDAR_H264ENC_STARTTRIG, 0x01 | ((size & 0x1f) << 8));
}

static void
cedar_bytestream_expgolomb(struct sunxi_cedar *cedar, uint32_t data)
{
	data++;
	cedar_bytestream_write(cedar, data,
			       (32 - __builtin_clz(data)) * 2 - 1);
}

static void
cedar_bytestream_expgolomb_signed(struct sunxi_cedar *cedar, int32_t data)
{
	data = (2 * data) - 1;
	data ^= (data >> 31);
	cedar_bytestream_expgolomb(cedar, data);
}

static irqreturn_t cedar_isr(int irq, void *dev_id)
{
	struct sunxi_cedar *cedar = (struct sunxi_cedar *) dev_id;
	uint32_t enable, status;

	enable = cedarenc_read(CEDAR_H264ENC_INT_ENABLE);
	enable &= 0x07;

	status = cedarenc_read(CEDAR_H264ENC_INT_STATUS);
	status &= 0x0F;

	if ((enable != 0) && (status != 0)) {
		cedar->int_status = cedarenc_read(CEDAR_H264ENC_INT_STATUS);

		/* clear interrupts */
		cedarenc_write(CEDAR_H264ENC_INT_STATUS, cedar->int_status);

		cedar->interrupt_received = true;
		wake_up_interruptible(&cedar->wait_queue);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int cedar_resources_get(struct sunxi_cedar *cedar,
			       struct platform_device *platform_dev)
{
	int irq, ret;

	dev_info(cedar->dev, "%s();\n", __func__);

	ret = of_reserved_mem_device_init(cedar->dev);
	if (ret && ret != -ENODEV) {
		dev_err(cedar->dev, "Failed to reserve memory\n");

		return ret;
	}

	ret = sunxi_sram_claim(cedar->dev);
	if (ret) {
		dev_err(cedar->dev, "Failed to claim SRAM\n");

		goto err_mem;
	}

	cedar->ahb_clk = devm_clk_get(cedar->dev, "ahb");
	if (IS_ERR(cedar->ahb_clk)) {
		dev_err(cedar->dev, "Failed to get AHB clock\n");

		ret = PTR_ERR(cedar->ahb_clk);
		goto err_sram;
	}

	cedar->mod_clk = devm_clk_get(cedar->dev, "mod");
	if (IS_ERR(cedar->mod_clk)) {
		dev_err(cedar->dev, "Failed to get MOD clock\n");

		ret = PTR_ERR(cedar->mod_clk);
		goto err_sram;
	}

	cedar->ram_clk = devm_clk_get(cedar->dev, "ram");
	if (IS_ERR(cedar->ram_clk)) {
		dev_err(cedar->dev, "Failed to get RAM clock\n");

		ret = PTR_ERR(cedar->ram_clk);
		goto err_sram;
	}

	cedar->rstc = devm_reset_control_get(cedar->dev, NULL);
	if (IS_ERR(cedar->rstc)) {
		dev_err(cedar->dev, "Failed to get reset control\n");

		ret = PTR_ERR(cedar->rstc);
		goto err_sram;
	}

	cedar->mmio_resource = platform_get_resource(platform_dev,
						     IORESOURCE_MEM, 0);
	cedar->mmio = devm_ioremap_resource(cedar->dev,
					    cedar->mmio_resource);
	if (IS_ERR(cedar->mmio)) {
		dev_err(cedar->dev, "Failed to map registers\n");

		ret = PTR_ERR(cedar->mmio);
		goto err_sram;
	}

	irq = platform_get_irq(platform_dev, 0);
	if (irq < 0) {
		dev_err(cedar->dev, "%s(): platform_get_irq() failed: %d.\n",
			__func__, -irq);
		ret = -irq;
		goto err_sram;
	}

	ret = devm_request_irq(cedar->dev, irq, cedar_isr, 0,
			       MODULE_NAME, cedar);
	if (ret) {
		dev_err(cedar->dev, "%s(): devm_request_irq() failed: %d.\n",
			__func__, ret);
		goto err_sram;
	}

	ret = clk_set_rate(cedar->mod_clk, CEDRUS_CLOCK_RATE_DEFAULT);
	if (ret) {
		dev_err(cedar->dev, "Failed to set clock rate\n");

		goto err_sram;
	}

	ret = clk_prepare_enable(cedar->ahb_clk);
	if (ret) {
		dev_err(cedar->dev, "Failed to enable AHB clock\n");

		goto err_sram;
	}

	ret = clk_prepare_enable(cedar->mod_clk);
	if (ret) {
		dev_err(cedar->dev, "Failed to enable MOD clock\n");

		goto err_ahb_clk;
	}

	ret = clk_prepare_enable(cedar->ram_clk);
	if (ret) {
		dev_err(cedar->dev, "Failed to enable RAM clock\n");

		goto err_mod_clk;
	}

	ret = reset_control_reset(cedar->rstc);
	if (ret) {
		dev_err(cedar->dev, "Failed to apply reset\n");

		goto err_ram_clk;
	}

	return 0;

 err_ram_clk:
	clk_disable_unprepare(cedar->ram_clk);
 err_mod_clk:
	clk_disable_unprepare(cedar->mod_clk);
 err_ahb_clk:
	clk_disable_unprepare(cedar->ahb_clk);
 err_sram:
	sunxi_sram_release(cedar->dev);
 err_mem:
	of_reserved_mem_device_release(cedar->dev);

	return ret;
}

static void cedar_resources_remove(struct sunxi_cedar *cedar)
{
	reset_control_assert(cedar->rstc);

	clk_disable_unprepare(cedar->ram_clk);
	clk_disable_unprepare(cedar->mod_clk);
	clk_disable_unprepare(cedar->ahb_clk);

	sunxi_sram_release(cedar->dev);

	of_reserved_mem_device_release(cedar->dev);
}


static int cedar_poweron(struct sunxi_cedar *cedar)
{
	dev_info(cedar->dev, "%s();\n", __func__);

	return 0;
}

static int cedar_poweroff(struct sunxi_cedar *cedar)
{
	dev_info(cedar->dev, "%s();\n", __func__);

	return 0;
}

static int cedar_resume(struct device *dev)
{
	struct sunxi_cedar *cedar = dev_get_drvdata(dev);

	dev_info(dev, "%s();\n", __func__);

	if (!cedar->powered)
		return 0;

	return cedar_poweron(cedar);
}

static int cedar_suspend(struct device *dev)
{
	struct sunxi_cedar *cedar = dev_get_drvdata(dev);

	dev_info(dev, "%s();\n", __func__);

	if (!cedar->powered)
		return 0;

	return cedar_poweroff(cedar);
}

static const struct dev_pm_ops cedar_pm_ops = {
	SET_RUNTIME_PM_OPS(cedar_suspend, cedar_resume, NULL)
};


static void
cedar_h264enc_enable(struct sunxi_cedar *cedar)
{
	cedar_io_write(cedar, CEDAR_VE_CTRL, 0x0013000b);
	/* enable interrupt and clear status flags */
	cedarenc_mask(CEDAR_H264ENC_INT_ENABLE, 0x0F, 0x0F);
	cedarenc_mask(CEDAR_H264ENC_INT_STATUS, 0x07, 0x07);
}

static void
cedar_h264enc_disable(struct sunxi_cedar *cedar)
{
	/* enable interrupt and clear status flags */
	cedarenc_mask(CEDAR_H264ENC_INT_ENABLE, 0, 0x0F);
	cedarenc_mask(CEDAR_H264ENC_INT_STATUS, 0x07, 0x07);
	cedar_io_write(cedar, CEDAR_VE_CTRL, 0);
}

static int
cedar_slashdev_open(struct inode *inode, struct file *filp)
{
	struct sunxi_cedar *cedar =
		container_of(inode->i_cdev, struct sunxi_cedar, cdev);

	dev_info(cedar->dev, "%s();\n", __func__);

	filp->private_data = cedar;

	cedar->time_open = ktime_get_raw_fast_ns();
	cedar->time_waiting = 0;
	cedar->configured = false;

	cedar_h264enc_enable(cedar);

	return 0;
}

static void
cedar_reference_frame_cleanup(struct sunxi_cedar *cedar,
			      struct cedar_reference_frame *frame)
{
	if (frame->luma_virtual)
		dma_free_coherent(cedar->dev,
				  frame->luma_size + frame->chroma_size,
				  frame->luma_virtual,
				  frame->luma_dma_addr);
	frame->luma_virtual = NULL;
	frame->luma_dma_addr = 0;
	frame->luma_size = 0;
	frame->chroma_dma_addr = 0;
	frame->chroma_size = 0;

	if (frame->subpic_virtual)
		dma_free_coherent(cedar->dev, frame->subpic_size,
				  frame->subpic_virtual,
				  frame->subpic_dma_addr);
	frame->subpic_size = 0;
	frame->subpic_virtual = NULL;
	frame->subpic_dma_addr = 0;
}

static int
cedar_reference_frame_init(struct sunxi_cedar *cedar,
			   struct cedar_reference_frame *frame)
{
	frame->luma_size =
		ALIGN(cedar->dst_width, 32) * ALIGN(cedar->dst_height, 64);
	frame->chroma_size =
		ALIGN(cedar->dst_width, 32) * ALIGN(cedar->dst_height, 128) / 2;

	frame->luma_virtual =
		dma_alloc_coherent(cedar->dev,
				   frame->luma_size + frame->chroma_size,
				   &frame->luma_dma_addr, GFP_KERNEL);
	if (!frame->luma_virtual) {
		dev_err(cedar->dev, "%s: failed to allocate luma.\n",
			__func__);
		goto error;
	}
	frame->chroma_dma_addr = frame->luma_dma_addr + frame->luma_size;

	frame->subpic_size = frame->luma_size >> 1;
	frame->subpic_virtual =
		dma_alloc_coherent(cedar->dev, frame->subpic_size,
				   &frame->subpic_dma_addr, GFP_KERNEL);
	if (!frame->subpic_virtual) {
		dev_err(cedar->dev, "%s: failed to allocate subpic.\n",
			__func__);
		goto error;
	}

	pr_info("%s(): %dbytes at 0x%08X, %dbytes at 0x%08X\n", __func__,
		frame->luma_size + frame->chroma_size, frame->luma_dma_addr,
		frame->subpic_size, frame->subpic_dma_addr);

	return 0;
 error:
	cedar_reference_frame_cleanup(cedar, frame);
	return -ENOMEM;
}

static void
cedar_buffers_cleanup(struct sunxi_cedar *cedar)
{
	if (cedar->input_luma_virtual)
		dma_free_coherent(cedar->dev, cedar->input_size,
				  cedar->input_luma_virtual,
				  cedar->input_luma_dma_addr);
	cedar->input_luma_virtual = NULL;
	cedar->input_luma_dma_addr = 0;
	cedar->input_chroma_dma_addr = 0;
	cedar->input_size = 0;

	cedar_reference_frame_cleanup(cedar, &cedar->reference_frame[0]);
	cedar_reference_frame_cleanup(cedar, &cedar->reference_frame[1]);
	cedar->reference_current = NULL;
	cedar->reference_previous = NULL;

	if (cedar->mv_buffer_virtual)
		dma_free_coherent(cedar->dev, cedar->mv_buffer_size,
				  cedar->mv_buffer_virtual,
				  cedar->mv_buffer_dma_addr);
	cedar->mv_buffer_size = 0;
	cedar->mv_buffer_virtual = NULL;
	cedar->mv_buffer_dma_addr = 0;

	if (cedar->mb_info_virtual)
		dma_free_coherent(cedar->dev, cedar->mb_info_size,
				  cedar->mb_info_virtual,
				  cedar->mb_info_dma_addr);
	cedar->mb_info_size = 0;
	cedar->mb_info_virtual = NULL;
	cedar->mb_info_dma_addr = 0;

	if (cedar->bytestream_virtual)
		dma_free_coherent(cedar->dev, cedar->bytestream_size,
				  cedar->bytestream_virtual,
				  cedar->bytestream_dma_addr);
	cedar->bytestream_size = 0;
	cedar->bytestream_virtual = NULL;
	cedar->bytestream_dma_addr = 0;
}

static int
cedar_buffers_init(struct sunxi_cedar *cedar)
{
	int size, ret;

	size = cedar->src_width * cedar->src_height;
	cedar->input_size = ALIGN(size + (size >> 1), 4096);
	cedar->input_luma_virtual =
		dma_alloc_coherent(cedar->dev, cedar->input_size,
				   &cedar->input_luma_dma_addr, GFP_KERNEL);
	if (!cedar->input_luma_virtual) {
		dev_err(cedar->dev, "%s: failed to allocate luma.\n",
			__func__);
		goto error;
	}
	cedar->input_chroma_dma_addr = cedar->input_luma_dma_addr + size;

	ret = cedar_reference_frame_init(cedar, &cedar->reference_frame[0]);
	if (ret)
		goto error;
	ret = cedar_reference_frame_init(cedar, &cedar->reference_frame[1]);
	if (ret)
		goto error;
	cedar->reference_current = &cedar->reference_frame[0];
	cedar->reference_previous = &cedar->reference_frame[1];

	cedar->mb_info_size = ALIGN(cedar->dst_width, 16) * 8;
	cedar->mb_info_virtual =
		dma_alloc_coherent(cedar->dev, cedar->mb_info_size,
				   &cedar->mb_info_dma_addr, GFP_KERNEL);
	if (!cedar->mb_info_virtual) {
		dev_err(cedar->dev, "%s: failed to allocate mb_info.\n",
			__func__);
		goto error;
	}

	cedar->mv_buffer_size =
		ALIGN(cedar->dst_width_mb, 4) * cedar->dst_height_mb * 8;
	cedar->mv_buffer_virtual =
		dma_alloc_coherent(cedar->dev, cedar->mv_buffer_size,
				   &cedar->mv_buffer_dma_addr, GFP_KERNEL);
	pr_info("%s(): mv buffer %d bytes at 0x%08X.\n", __func__,
		cedar->mv_buffer_size, cedar->mv_buffer_dma_addr);
	if (!cedar->mv_buffer_virtual) {
		dev_err(cedar->dev, "%s: failed to allocate mv_buffer.\n",
			__func__);
		goto error;
	}

	cedar->bytestream_size = 1 << 20;
	cedar->bytestream_virtual =
		dma_alloc_coherent(cedar->dev, cedar->bytestream_size,
				   &cedar->bytestream_dma_addr, GFP_KERNEL);
	if (!cedar->bytestream_virtual) {
		dev_err(cedar->dev, "%s: failed to allocate bytestream.\n",
			__func__);
		goto error;
	}

	return 0;
 error:
	cedar_buffers_cleanup(cedar);
	return -ENOMEM;
}

static int
cedar_slashdev_release(struct inode *inode, struct file *filp)
{
	uint64_t now, total;
	struct sunxi_cedar *cedar =
		container_of(inode->i_cdev, struct sunxi_cedar, cdev);

	dev_info(cedar->dev, "%s();\n", __func__);

	now = ktime_get_raw_fast_ns();
	total = now - cedar->time_open;

	dev_info(cedar->dev, "Time spent: %llu/%lluns\n",
		 cedar->time_waiting, total);

	cedar_h264enc_disable(cedar);

	cedar_buffers_cleanup(cedar);

	cedar->configured = false;

	filp->private_data = cedar;

	return 0;
}

static long
cedar_slashdev_ioctl_get_env_info(struct sunxi_cedar *cedar, void __user *to)
{
	struct cedarv_env_infomation info = {
		.address_macc = cedar->mmio_resource->start,
	};

	if (!to)
		return -EINVAL;

	if (copy_to_user(to, &info, sizeof(struct cedarv_env_infomation)))
		return -EFAULT;

	return 0;
}

static long
cedar_slashdev_ioctl_config(struct sunxi_cedar *cedar, void __user *from)
{
	struct cedar_ioctl_config config;
	int ret;

	if (!from)
		return -EINVAL;

	if (copy_from_user(&config, from, sizeof(struct cedar_ioctl_config)))
		return -EFAULT;

	if (cedar->configured) {
		dev_err(cedar->dev, "%s(): Already configured.\n", __func__);
		return -EINVAL;
	}

	cedar->src_width = config.src_width;
	cedar->src_height = config.src_height;
	cedar->src_format = config.src_format;

	cedar->src_width_mb = ALIGN(cedar->src_width, 16) >> 4;
	cedar->src_height_mb = ALIGN(cedar->src_height, 16) >> 4;
	cedar->src_stride_mb = cedar->src_width_mb;

	cedar->dst_width = config.dst_width;
	cedar->dst_height = config.dst_height;
	cedar->dst_width_mb = ALIGN(cedar->dst_width, 16) >> 4;
	cedar->dst_height_mb = ALIGN(cedar->dst_height, 16) >> 4;
	if (config.dst_width & 0x0F)
		cedar->dst_width_crop = 0x10 - (config.dst_width & 0x0F);
	else
		cedar->dst_width_crop = 0;
	if (config.dst_height & 0x0F)
		cedar->dst_height_crop = 0x10 - (config.dst_height & 0x0F);
	else
		cedar->dst_height_crop = 0;

	cedar->profile = config.profile;
	cedar->level = config.level;
	cedar->qp = config.qp;
	cedar->keyframe_interval = config.keyframe_interval;
	cedar->frame_p_count = 0;
	cedar->frame_count = 0;

	if (config.entropy_coding_mode == CEDAR_IOCTL_ENTROPY_CODING_CABAC)
		cedar->entropy_coding_mode_cabac = true;
	else
		cedar->entropy_coding_mode_cabac = false;

	ret = cedar_buffers_init(cedar);
	if (ret)
		return ret;

	cedar->configured = true;

	config.input_dma_addr = cedar->input_luma_dma_addr;
	config.input_size = cedar->input_size;

	config.bytestream_dma_addr = cedar->bytestream_dma_addr;
	config.bytestream_size = cedar->bytestream_size;

	if (copy_to_user(from, &config, sizeof(struct cedar_ioctl_config)))
		return -EFAULT;

	return 0;
}

static void
cedar_bytestream_nal_startcode(struct sunxi_cedar *cedar,
			       uint8_t ref_idc, uint8_t type)
{
	/* disable emulation_prevention_three_byte */
	cedarenc_mask(CEDAR_H264ENC_PARA0, 0x80000000, 0x80000000);

	cedar_bytestream_write(cedar, 0, 24);
	cedar_bytestream_write(cedar,
			       0x100 | ((ref_idc & 0x03) << 5) | (type & 0x1F),
			       16);

	cedarenc_mask(CEDAR_H264ENC_PARA0, 0, 0x80000000);
}

static void
cedar_bytestream_rbsp_trailing_bits(struct sunxi_cedar *cedar)
{
	uint32_t len = cedarenc_read(CEDAR_H264ENC_STMLEN);
	int pad = 8 - ((len + 1) & 0x7);

	cedar_bytestream_write(cedar, 1 << pad, pad + 1);
}

static void
cedar_bytestream_sequence_parameter_set(struct sunxi_cedar *cedar)
{
	cedar_bytestream_nal_startcode(cedar, 3, 7);

	cedar_bytestream_write(cedar, cedar->profile, 8);
	/* constraints */
	cedar_bytestream_write(cedar, 0, 8);
	cedar_bytestream_write(cedar, cedar->level, 8);

	/* seq_parameter_set_id */
	cedar_bytestream_expgolomb(cedar, 0);

	/* log2_max_frame_num_minus4 */
	cedar_bytestream_expgolomb(cedar, 0);
	/* pic_order_cnt_type */
	cedar_bytestream_expgolomb(cedar, 2);

	/* max_num_ref_frames */
	cedar_bytestream_expgolomb(cedar, 1);
	/* gaps_in_frame_num_value_allowed_flag */
	cedar_bytestream_write(cedar, 0, 1);

	cedar_bytestream_expgolomb(cedar, cedar->dst_width_mb - 1);
	cedar_bytestream_expgolomb(cedar, cedar->dst_height_mb - 1);

	/* frame_mbs_only_flag = */
	cedar_bytestream_write(cedar, 1, 1);

	/* direct_8x8_inference_flag */
	cedar_bytestream_write(cedar, 0, 1);

	if (cedar->dst_width_crop || cedar->dst_height_crop) {
		cedar_bytestream_write(cedar, 1, 1);
		cedar_bytestream_expgolomb(cedar, 0);
		cedar_bytestream_expgolomb(cedar, cedar->dst_width_crop);
		cedar_bytestream_expgolomb(cedar, 0);
		cedar_bytestream_expgolomb(cedar, cedar->dst_height_crop);
	} else
		cedar_bytestream_write(cedar, 0, 1);

	/* vui_parameters_present_flag */
	cedar_bytestream_write(cedar, 0, 1);

	cedar_bytestream_rbsp_trailing_bits(cedar);
}

static void
cedar_bytestream_picture_parameter_set(struct sunxi_cedar *cedar)
{
	cedar_bytestream_nal_startcode(cedar, 3, 8);

	/* pic_parameter_set_id */
	cedar_bytestream_expgolomb(cedar, 0);
	/* seq_parameter_set_id */
	cedar_bytestream_expgolomb(cedar, 0);

	if (cedar->entropy_coding_mode_cabac)
		cedar_bytestream_write(cedar, 1, 1);
	else
		cedar_bytestream_write(cedar, 0, 1);

	/* bottom_field_pic_order_in_frame_present_flag */
	cedar_bytestream_write(cedar, 0, 1);
	 /* num_slice_groups_minus1 */
	cedar_bytestream_expgolomb(cedar, 0);

	/* num_ref_idx_l0_default_active_minus1 */
	cedar_bytestream_expgolomb(cedar, 0);
	/* num_ref_idx_l1_default_active_minus1 */
	cedar_bytestream_expgolomb(cedar, 0);

	/* weighted_pred_flag */
	cedar_bytestream_write(cedar, 0, 1);
	/* weighted_bipred_idc */
	cedar_bytestream_write(cedar, 0, 2);

	cedar_bytestream_expgolomb_signed(cedar, cedar->qp - 26);
	cedar_bytestream_expgolomb_signed(cedar, cedar->qp - 26);
	/* chroma_qp_index_offset */
	cedar_bytestream_expgolomb_signed(cedar, 4);

	/* deblocking_filter_control_present_flag */
	cedar_bytestream_write(cedar, 1, 1);
	/* constrained_intra_pred_flag */
	cedar_bytestream_write(cedar, 0, 1);
	/* redundant_pic_cnt_present_flag */
	cedar_bytestream_write(cedar, 0, 1);

	cedar_bytestream_rbsp_trailing_bits(cedar);
}

static void
cedar_bytestream_sliceheader(struct sunxi_cedar *cedar, bool frame_i)
{
	if (frame_i)
		cedar_bytestream_nal_startcode(cedar, 3, 5);
	else
		cedar_bytestream_nal_startcode(cedar, 2, 1);

	/* first_mb_in_slice */
	cedar_bytestream_expgolomb(cedar, 0);
	if (frame_i)
		cedar_bytestream_expgolomb(cedar, 2);
	else
		cedar_bytestream_expgolomb(cedar, 0); /* P frame */
	/* pic_parameter_set_id */
	cedar_bytestream_expgolomb(cedar, 0);

	cedar_bytestream_write(cedar, cedar->frame_p_count & 0x0F, 4);

	if (frame_i) {
		/* idr_pic_id */
		cedar_bytestream_expgolomb(cedar, 0);
		/* no_output_of_prior_pics_flag */
		cedar_bytestream_write(cedar, 0, 1);
		/* long_term_reference_flag */
		cedar_bytestream_write(cedar, 0, 1);
	} else { /* P frame */
		/* num_ref_idx_active_override_flag */
		cedar_bytestream_write(cedar, 0, 1);
		/* ref_pic_list_modification_flag_l0 */
		cedar_bytestream_write(cedar, 0, 1);
		 /* adaptive_ref_pic_marking_mode_flag */
		cedar_bytestream_write(cedar, 0, 1);
		if (cedar->entropy_coding_mode_cabac) /* cabac_init_idc = */
			cedar_bytestream_expgolomb(cedar, 0);
	}

	/* slice_qp_delta */
	cedar_bytestream_expgolomb_signed(cedar, 0);

	/* disable_deblocking_filter_idc */
	cedar_bytestream_expgolomb(cedar, 0);
	/* slice_alpha_c0_offset_div2 */
	cedar_bytestream_expgolomb_signed(cedar, 0);
	/* slice_beta_offset_div2 */
	cedar_bytestream_expgolomb_signed(cedar, 0);
}

static long
cedar_slashdev_ioctl_encode(struct sunxi_cedar *cedar, void __user *from)
{
	struct cedar_ioctl_encode encode;
	struct cedar_reference_frame *reference_tmp;
	uint64_t start, stop;

	if (!from)
		return -EINVAL;

	if (copy_from_user(&encode, from, sizeof(struct cedar_ioctl_encode)))
		return -EFAULT;

	if (!cedar->configured) {
		dev_err(cedar->dev, "%s: CEDAR_IOCTL_CONFIG not run yet.\n",
			__func__);
		return -EINVAL;
	}

	start = ktime_get_raw_fast_ns();

	if (encode.frame_type == CEDAR_FRAME_TYPE_I)
		cedar->frame_p_count = 0;

	if (!cedar->frame_count) {
		cedar_bytestream_sequence_parameter_set(cedar);
		cedar_bytestream_picture_parameter_set(cedar);
	}

	if (encode.frame_type == CEDAR_FRAME_TYPE_P)
		cedar_bytestream_sliceheader(cedar, false);
	else
		cedar_bytestream_sliceheader(cedar, true);

	cedarisp_write(CEDAR_H264ISP_STRIDE_CTRL,
		       cedar->src_stride_mb << 16);
	cedarisp_write(CEDAR_H264ISP_INPUT_SIZE,
		       (cedar->src_width_mb << 16) |
		       cedar->src_height_mb);

	cedarisp_write(CEDAR_H264ISP_CTRL, 0);
	cedarisp_write(CEDAR_H264ISP_INPUT_Y_ADDR,
		       cedar->input_luma_dma_addr);
	cedarisp_write(CEDAR_H264ISP_INPUT_C0_ADDR,
		       cedar->input_chroma_dma_addr);

	cedarenc_write(CEDAR_H264ENC_RECADDRY,
		       cedar->reference_current->luma_dma_addr);
	cedarenc_write(CEDAR_H264ENC_RECADDRC,
		       cedar->reference_current->chroma_dma_addr);
	cedarenc_write(CEDAR_H264ENC_SUBPIXADDRNEW,
		       cedar->reference_current->subpic_dma_addr);

	if (encode.frame_type == CEDAR_FRAME_TYPE_P) {
		cedarenc_write(CEDAR_H264ENC_REFADDRY,
			       cedar->reference_previous->luma_dma_addr);
		cedarenc_write(CEDAR_H264ENC_REFADDRC,
			       cedar->reference_previous->chroma_dma_addr);
		cedarenc_write(CEDAR_H264ENC_SUBPIXADDRLAST,
			       cedar->reference_previous->subpic_dma_addr);
	}

	cedarenc_write(CEDAR_H264ENC_MBINFO, cedar->mb_info_dma_addr);
	cedarenc_write(CEDAR_H264ENC_MVBUFADDR,
		       cedar->mv_buffer_dma_addr);

	if (cedar->entropy_coding_mode_cabac)
		cedarenc_mask(CEDAR_H264ENC_PARA0, 0x100, 0x100);
	else
		cedarenc_mask(CEDAR_H264ENC_PARA0, 0, 0x100);

	if (encode.frame_type == CEDAR_FRAME_TYPE_P)
		cedarenc_mask(CEDAR_H264ENC_PARA0, 0x10, 0x70);
	else /* I */
		cedarenc_mask(CEDAR_H264ENC_PARA0, 0, 0x70);

	/* first chrome QP offset = 4,
	   set fixed QP and fixed intra QP to QP */
	cedarenc_write(CEDAR_H264ENC_PARA1, (4 << 16) |
		       (cedar->qp << 8) | cedar->qp);

	/* motion estimation parameters: disable ME, searchlevel 2 */
	cedarenc_write(CEDAR_H264ENC_MEPARA, 0x00000104);

	cedar->interrupt_received = false;

	/* trigger encoding */
	cedarenc_write(CEDAR_H264ENC_STARTTRIG, 0x08);

	wait_event_interruptible_timeout(cedar->wait_queue,
					 cedar->interrupt_received,
					 1 * HZ);

	if (!cedar->interrupt_received) {
		dev_err(cedar->dev, "%s(): encoding timed out.\n", __func__);
		return -1;
	}

	if ((cedar->int_status & 0x03) != 0x01) {
		dev_err(cedar->dev, "%s(): encoding failed: 0x%08X\n",
			__func__, cedar->int_status);
		return -1;
	}

	cedar->frame_p_count++;
	cedar->frame_count++;

	/* swap reference_frames around to prepare for a future frame */
	reference_tmp = cedar->reference_current;
	cedar->reference_current = cedar->reference_previous;
	cedar->reference_previous = reference_tmp;

	stop = ktime_get_raw_fast_ns();

	cedar->time_waiting += stop - start;

	/* size of encoded stream, in bytes. */
	return cedarenc_read(CEDAR_H264ENC_STMLEN) / 8; /* align? */
}

static long
cedar_slashdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct sunxi_cedar *cedar = filp->private_data;
	void __user *to = (void *) arg;

	switch (cmd) {
	case CEDAR_IOCTL_GET_ENV_INFO:
		return cedar_slashdev_ioctl_get_env_info(cedar, to);
	case CEDAR_IOCTL_ENCODE:
		return cedar_slashdev_ioctl_encode(cedar, to);
	case CEDAR_IOCTL_CONFIG:
		return cedar_slashdev_ioctl_config(cedar, to);
	default:
		dev_err(cedar->dev, "%s(0x%04X, 0x%lX): unhandled ioctl.\n",
			__func__, cmd, arg);
		return -1;
	}

	return 0;
}

static int
cedar_slashdev_mmap_io(struct sunxi_cedar *cedar, struct vm_area_struct *vma)
{
	size_t size = cedar->mmio_resource->end -
		cedar->mmio_resource->start;
	int ret;

	dev_info(cedar->dev, "%s();\n", __func__);

	/* Set reserved and I/O flag for the area. */
	vma->vm_flags |= VM_IO;

	/* Select uncached access. */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* clear offset, so vm_iomap_memory will not try to use it */
	vma->vm_pgoff = 0;

	ret = vm_iomap_memory(vma, cedar->mmio_resource->start, size);
	if (ret) {
		dev_err(cedar->dev, "%s(): vm_iomap_memory() failed: %d.\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int
cedar_slashdev_mmap_mem(struct sunxi_cedar *cedar, struct vm_area_struct *vma)
{
	phys_addr_t address = vma->vm_pgoff << 12;
	size_t size = vma->vm_end - vma->vm_start;
	int ret;

	dev_info(cedar->dev, "%s(0x%08X, 0x%02X);\n", __func__, address, size);

	/* Set reserved and I/O flag for the area. */
	vma->vm_flags |= VM_IO;

	/* Select uncached access. */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* clear offset, so vm_iomap_memory will not try to use it */
	vma->vm_pgoff = 0;

	ret = vm_iomap_memory(vma, address, size);
	if (ret) {
		dev_err(cedar->dev, "%s(): vm_iomap_memory() failed: %d.\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int
cedar_slashdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct sunxi_cedar *cedar = filp->private_data;
	phys_addr_t address = vma->vm_pgoff << 12;
	size_t size = vma->vm_end - vma->vm_start;

	dev_info(cedar->dev, "%s(0x%08X, 0x%04X);\n", __func__, address, size);

	if (address == cedar->mmio_resource->start)
		return cedar_slashdev_mmap_io(cedar, vma);
	else if ((address = cedar->input_luma_dma_addr) &&
		 (size == cedar->input_size))
		return cedar_slashdev_mmap_mem(cedar, vma);
	else if ((address = cedar->bytestream_dma_addr) &&
		 (size == cedar->bytestream_size))
		return cedar_slashdev_mmap_mem(cedar, vma);
	else {
		dev_err(cedar->dev, "%s(0x%08X): invalid offset;\n",
			__func__, address);
		dev_info(cedar->dev, "0x%lX -> 0x%lX\n", vma->vm_start, vma->vm_end);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations cedar_slashdev_fops = {
	.owner = THIS_MODULE,
	.open = cedar_slashdev_open,
	.release = cedar_slashdev_release,
	.llseek = no_llseek,
	.unlocked_ioctl = cedar_slashdev_ioctl,
	.mmap = cedar_slashdev_mmap,
};

static int cedar_slashdev_initialize(struct sunxi_cedar *cedar)
{
	int ret;

	dev_info(cedar->dev, "%s();\n", __func__);

	ret = alloc_chrdev_region(&cedar->majorminor, 0, 1, "cedar_dev");
	if (ret) {
		dev_err(cedar->dev, "%s(): alloc_chrdev_region() failed: %d\n",
			__func__, ret);
		return ret;
	}

	cdev_init(&cedar->cdev, &cedar_slashdev_fops);
	cedar->cdev.owner = THIS_MODULE;

	ret = cdev_add(&cedar->cdev, cedar->majorminor, 1);
	if (ret) {
		dev_err(cedar->dev, "%s(): cdev_add() failed: %d\n",
			__func__, ret);
		unregister_chrdev_region(cedar->majorminor, 1);
		return ret;
	}

	cedar->class = class_create(THIS_MODULE, "cedar_dev");
	if (IS_ERR(cedar->class)) {
		dev_err(cedar->dev, "%s(): class_create() failed: %ld\n",
			__func__, PTR_ERR(cedar->class));
		cdev_del(&cedar->cdev);
		unregister_chrdev_region(cedar->majorminor, 1);
		return ret;

	}

	cedar->slashdev = device_create(cedar->class, NULL, cedar->majorminor,
					NULL, "cedar_dev");
	if (!cedar->slashdev) {
		dev_err(cedar->dev, "%s(): device_create() failed: %d\n",
			__func__, ret);
		class_destroy(cedar->class);
		cedar->class = NULL;
		cdev_del(&cedar->cdev);
		unregister_chrdev_region(cedar->majorminor, 1);
		return ret;
	}

	return 0;
}

static int cedar_slashdev_cleanup(struct sunxi_cedar *cedar)
{
	dev_info(cedar->dev, "%s();\n", __func__);

	device_destroy(cedar->class, cedar->majorminor);
	cedar->slashdev = NULL;
	class_destroy(cedar->class);
	cedar->class = NULL;
	cdev_del(&cedar->cdev);
	unregister_chrdev_region(cedar->majorminor, 1);

	return 0;
}

static int cedar_probe(struct platform_device *platform_dev)
{
	struct device *dev = &platform_dev->dev;
	struct sunxi_cedar *cedar;
	int ret;

	dev_info(dev, "%s();\n", __func__);

	cedar = devm_kzalloc(dev, sizeof(struct sunxi_cedar), GFP_KERNEL);
	if (!cedar)
		return -ENOMEM;
	cedar->dev = dev;
	init_waitqueue_head(&cedar->wait_queue);

	ret = cedar_resources_get(cedar, platform_dev);
	if (ret)
		return ret;

	platform_set_drvdata(platform_dev, cedar);

	ret = cedar_slashdev_initialize(cedar);
	if (ret)
		return ret;

	dev_info(dev, "Cedar AVC driver loaded for hw 0x%04X.\n",
		 cedar_io_read(cedar, CEDAR_VE_VERSION) >> 16);

	return 0;
}

static int cedar_remove(struct platform_device *platform_dev)
{
	struct device *dev = &platform_dev->dev;
	struct sunxi_cedar *cedar = platform_get_drvdata(platform_dev);
	int ret;

	dev_info(dev, "%s();\n", __func__);

	ret = cedar_slashdev_cleanup(cedar);
	if (ret)
		return ret;

	cedar_resources_remove(cedar);

	return 0;
}

static const struct of_device_id cedar_of_match[] = {
	{ .compatible = "allwinner,sun7i-a20-video-engine", },
	{},
};
MODULE_DEVICE_TABLE(of, cedar_of_match);

static struct platform_driver cedar_platform_driver = {
	.probe = cedar_probe,
	.remove = cedar_remove,
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(cedar_of_match),
		.pm = &cedar_pm_ops,
	},
};
module_platform_driver(cedar_platform_driver);

MODULE_DESCRIPTION("Allwinner A20 CedarX media encoder/decoder driver");
MODULE_AUTHOR("Luc Verhaegen <libv@skynet.be>");
MODULE_LICENSE("GPL v2");
