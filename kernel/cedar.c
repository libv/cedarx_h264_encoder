// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus VPU driver
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

#include <linux/soc/sunxi/sunxi_sram.h>

#include "cedar_ve.h"

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
};

#define CEDRUS_CLOCK_RATE_DEFAULT 320000000

static int cedar_resources_get(struct sunxi_cedar *cedar,
			       struct platform_device *platform_dev)
{
	struct resource *res;
	int ret;

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

	res = platform_get_resource(platform_dev, IORESOURCE_MEM, 0);
	cedar->mmio = devm_ioremap_resource(cedar->dev, res);
	if (IS_ERR(cedar->mmio)) {
		dev_err(cedar->dev, "Failed to map registers\n");

		ret = PTR_ERR(cedar->mmio);
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


static int
cedar_slashdev_open(struct inode *inode, struct file *filp)
{
	struct sunxi_cedar *cedar =
		container_of(inode->i_cdev, struct sunxi_cedar, cdev);

	dev_info(cedar->dev, "%s();\n", __func__);

	filp->private_data = cedar;

	return 0;
}

static int
cedar_slashdev_release(struct inode *inode, struct file *filp)
{
	struct sunxi_cedar *cedar =
		container_of(inode->i_cdev, struct sunxi_cedar, cdev);

	dev_info(cedar->dev, "%s();\n", __func__);

	filp->private_data = cedar;

	return 0;
}

static long
cedar_slashdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct sunxi_cedar *cedar = filp->private_data;

	switch (cmd) {
	case IOCTL_GET_ENV_INFO:
		dev_info(cedar->dev, "%s(%s, 0x%lX);\n", __func__,
			 "GET_ENV_INFO", arg);
		return 0;
	case IOCTL_ENGINE_REQ:
		dev_info(cedar->dev, "%s(%s, 0x%lX);\n", __func__,
			 "ENGINE_REQ", arg);
		return 0;
	case IOCTL_ENGINE_REL:
		dev_info(cedar->dev, "%s(%s, 0x%lX);\n", __func__,
			 "ENGINE_REL", arg);
		return 0;
	case IOCTL_SET_REFCOUNT:
		dev_info(cedar->dev, "%s(%s, 0x%lX);\n", __func__,
			 "SET_REFCOUNT", arg);
		return 0;
	default:
		dev_err(cedar->dev, "%s(0x%04X, 0x%lX): unhandled ioctl.\n",
			__func__, cmd, arg);
		return -1;
	}

	return 0;
}

static int
cedar_slashdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct sunxi_cedar *cedar = filp->private_data;

	dev_info(cedar->dev, "%s();\n", __func__);

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

	ret = cedar_resources_get(cedar, platform_dev);
	if (ret)
		return ret;

	platform_set_drvdata(platform_dev, cedar);

	ret = cedar_slashdev_initialize(cedar);
	if (ret)
		return ret;

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
