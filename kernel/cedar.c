/*
 * Contains glue code for handling devicetree and things.
 */
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>

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
};

static int cedar_resources_get(struct sunxi_cedar *cedar,
			       struct platform_device *platform_dev)
{
	dev_info(cedar->dev, "%s();\n", __func__);

	return 0;
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
