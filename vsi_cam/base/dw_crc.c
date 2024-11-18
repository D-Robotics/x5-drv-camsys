// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include "cam_dev.h"

#include "dw_crc.h"

#define DW_CRC_DT_NAME    "verisilicon,dw-cr-ctrl"
#define DW_CRC_DEV_NAME   "vs-dw-crc"

struct dw_crc_device {
	struct device *dev;
	refcount_t vse_state_cnt, gdc_state_cnt;
	enum cam_state vse_state, gdc_state;
	struct reset_control *vse_rst, *gdc_rst;
	struct mutex lock; /* lock for state change and reset */
};

int dw_reset(struct dw_crc_device *dev, enum dw_mod mod)
{
#if 0
	struct reset_control *rst = NULL;
	int rc = 0;

	if (!dev)
		return -ENODEV;

	mutex_lock(&dev->lock);
	if (mod == DW_MOD_VSE) {
		rst = dev->vse_rst;
		if (dev->gdc_state != CAM_STATE_STARTED) {
			reset_control_assert(rst);
			udelay(2);
			reset_control_deassert(rst);
		} else {
			rc = -EBUSY;
		}
	} else if (mod == DW_MOD_GDC) {
		rst = dev->gdc_rst;
		if (dev->vse_state != CAM_STATE_STARTED) {
			reset_control_assert(rst);
			udelay(2);
			reset_control_deassert(rst);
		} else {
			rc = -EBUSY;
		}
	} else {
		rc = -EINVAL;
	}

	mutex_unlock(&dev->lock);
	return rc;
#else
	return 0;
#endif
}
EXPORT_SYMBOL(dw_reset);

int dw_set_state(struct dw_crc_device *dev, enum dw_mod mod,
		 enum cam_state state)
{
	refcount_t *refcnt;
	enum cam_state *p_state;
	int rc = 0;

	if (!dev)
		return -ENODEV;

	mutex_lock(&dev->lock);
	if (mod == DW_MOD_VSE) {
		refcnt = &dev->vse_state_cnt;
		p_state = &dev->vse_state;
	} else if (mod == DW_MOD_GDC) {
		refcnt = &dev->gdc_state_cnt;
		p_state = &dev->gdc_state;
	} else {
		mutex_unlock(&dev->lock);
		return -EINVAL;
	}

	if (state == CAM_STATE_STARTED) {
		if (refcount_read(refcnt) == REFCNT_INIT_VAL)
			*p_state = state;
		refcount_inc(refcnt);
	} else if (state == CAM_STATE_STOPPED) {
		if (refcount_read(refcnt) > REFCNT_INIT_VAL) {
			refcount_dec(refcnt);
			if (refcount_read(refcnt) == REFCNT_INIT_VAL)
				*p_state = state;
		}
	}
	mutex_unlock(&dev->lock);
	return rc;
}
EXPORT_SYMBOL(dw_set_state);

void put_dw_crc_device(struct dw_crc_device *dev)
{
	if (dev)
		put_device(dev->dev);
}
EXPORT_SYMBOL(put_dw_crc_device);

static int dw_crc_probe(struct platform_device *pdev)
{
	struct dw_crc_device *crc;
	struct reset_control *rst;

	crc = devm_kzalloc(&pdev->dev, sizeof(*crc), GFP_KERNEL);
	if (!crc)
		return -ENOMEM;

	rst = devm_reset_control_get(&pdev->dev, "vse");
	if (IS_ERR_OR_NULL(rst)) {
		dev_err(&pdev->dev, "cannot get vse reset control\n");
		return IS_ERR(rst) ? PTR_ERR(rst) : -ENXIO;
	}
	crc->vse_rst = rst;
	rst = devm_reset_control_get(&pdev->dev, "gdc");
	if (IS_ERR_OR_NULL(rst)) {
		dev_err(&pdev->dev, "cannot get gdc reset control\n");
		return IS_ERR(rst) ? PTR_ERR(rst) : -ENXIO;
	}
	crc->gdc_rst = rst;

	mutex_init(&crc->lock);
	refcount_set(&crc->vse_state_cnt, REFCNT_INIT_VAL);
	refcount_set(&crc->gdc_state_cnt, REFCNT_INIT_VAL);

	crc->dev = &pdev->dev;
	platform_set_drvdata(pdev, crc);
	return 0;
}

static int dw_crc_remove(struct platform_device *pdev)
{
	struct dw_crc_device *crc = platform_get_drvdata(pdev);

	if (crc)
		mutex_destroy(&crc->lock);
	devm_kfree(&pdev->dev, crc);
	return 0;
}

static const struct of_device_id dw_crc_of_match[] = {
	{ .compatible = DW_CRC_DT_NAME },
	{ },
};

MODULE_DEVICE_TABLE(of, dw_crc_of_match);

static struct platform_driver dw_crc_driver = {
	.probe	= dw_crc_probe,
	.remove = dw_crc_remove,
	.driver = {
		.name = DW_CRC_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = dw_crc_of_match,
	}
};

static int __init dw_crc_init_module(void)
{
	return platform_driver_register(&dw_crc_driver);
}

static void __exit dw_crc_exit_module(void)
{
	platform_driver_unregister(&dw_crc_driver);
}

module_init(dw_crc_init_module);
module_exit(dw_crc_exit_module);

MODULE_DESCRIPTION("VeriSilicon DW C/R Control Driver");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
MODULE_LICENSE("GPL");
MODULE_ALIAS("VS-DW-CRC");
