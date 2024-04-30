// SPDX-License-Identifier: GPL-2.0-only
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "csi_wrapper.h"

#define CSIW_DT_NAME    "verisilicon,csi-rx-sr"
#define CSIW_DEV_NAME   "vs-csi-wrapper"

#define DRX_REG0        (0x00)
#define DRX_REG2        (0x08)
#define DRX_REG4        (0x10)
#define PHY_TESTCODE0   (0x14)
#define PHY_TESTCODE1   (0x18)

#define PHY_RX_NUM      (4)

union drx_reg {
	struct {
		u32 p0_enableclk : 1;
		u32 p0_basedir_0 : 1;
		u32 p0_forcerxmode_0 : 1;
		u32 p0_forcerxmode_1 : 1;
		u32 p0_forcetxstopmode_0 : 1;
		u32 p0_hsfreqrange : 7;
		u32 p1_enableclk : 1;
		u32 p1_basedir_0 : 1;
		u32 p1_forcerxmode_0 : 1;
		u32 p1_forcerxmode_1 : 1;
		u32 p1_forcetxstopmode_0 : 1;
		u32 p1_hsfreqrange : 7;
		u32 p1_ctrl_sel : 1;
		u32 reserved : 7;
	};
	u32 value;
};

union drx_reg4 {
	struct {
		u32 rct : 2;
		u32 wct : 2;
		u32 kp : 3;
		u32 test_sel_4l_0 : 1;
		u32 test_sel_4l_1 : 1;
		u32 cfgclkfreqrange : 6;
		u32 ppipg_clk_enable : 1;
		u32 reserved1 : 16;
	};
	u32 value;
};

#define csiw_write(csiw, offset, value) \
	__raw_writel(value, (csiw)->base + (offset))

#define csiw_read(csiw, offset) \
	__raw_readl((csiw)->base + (offset))

struct csiw_device {
	struct device *dev;
	void __iomem *base;
};

int csi_phy_enableclk(struct csiw_device *dev, unsigned int id)
{
	union drx_reg drx_reg;
	u32 reg;

	if (!dev)
		return -ENODEV;

	if (!id || id == 1)
		reg = DRX_REG0;
	else if (id == 2 || id == 3)
		reg = DRX_REG2;
	else
		return -EINVAL;

	drx_reg.value = csiw_read(dev, reg);

	if (id % 2 == 0)
		drx_reg.p0_enableclk = 1;
	else
		drx_reg.p1_enableclk = 1;

	csiw_write(dev, reg, drx_reg.value);
	return 0;
}
EXPORT_SYMBOL(csi_phy_enableclk);

int csi_phy_force_rxmode(struct csiw_device *dev, unsigned int id, unsigned int lanes, bool force)
{
	union drx_reg drx_reg;
	u32 reg;

	if (!dev)
		return -ENODEV;

	if (!id || id == 1)
		reg = DRX_REG0;
	else if (id == 2 || id == 3)
		reg = DRX_REG2;
	else
		return -EINVAL;

	drx_reg.value = csiw_read(dev, reg);

	if (id % 2 == 0) {
		drx_reg.p0_forcerxmode_0 = (u32)force;
		drx_reg.p0_forcerxmode_1 = (u32)force;
		if (lanes > 2) {
			drx_reg.p1_forcerxmode_0 = (u32)force;
			drx_reg.p1_forcerxmode_1 = (u32)force;
		}
	} else {
		drx_reg.p1_forcerxmode_0 = (u32)force;
		drx_reg.p1_forcerxmode_1 = (u32)force;
	}

	csiw_write(dev, reg, drx_reg.value);
	return 0;
}
EXPORT_SYMBOL(csi_phy_force_rxmode);

int csi_phy_get_lane_mode(struct csiw_device *dev, unsigned int id, unsigned int *lane_merged)
{
	union drx_reg drx_reg;
	u32 reg;

	if (!dev)
		return -ENODEV;
	if (!lane_merged)
		return -EINVAL;
	if (!id || id == 1)
		reg = DRX_REG0;
	else if (id == 2 || id == 3)
		reg = DRX_REG2;
	else
		return -EINVAL;
	drx_reg.value = csiw_read(dev, reg);
	*lane_merged = drx_reg.p1_ctrl_sel;
	return 0;
}
EXPORT_SYMBOL(csi_phy_get_lane_mode);

int csi_phy_set_lane_mode(struct csiw_device *dev, unsigned int id, bool lane_merged)
{
	union drx_reg drx_reg;
	u32 reg;

	if (!dev)
		return -ENODEV;

	if (!id || id == 1)
		reg = DRX_REG0;
	else if (id == 2 || id == 3)
		reg = DRX_REG2;
	else
		return -EINVAL;

	drx_reg.value = csiw_read(dev, reg);
	drx_reg.p1_ctrl_sel = lane_merged ? 1 : 0;
	csiw_write(dev, reg, drx_reg.value);
	return 0;
}
EXPORT_SYMBOL(csi_phy_set_lane_mode);

static struct {
	unsigned int rate;
	unsigned int range;
} rr_list[] = {
	{ 2500, 0x49 },
	{ 2000, 0x0f },
	{ 1500, 0x2c },
	{ 1000, 0x0a },
	{  800, 0x09 },
	{  600, 0x07 },
	{  500, 0x26 },
	{  400, 0x05 },
	{  300, 0x14 },
	{  205, 0x03 },
	{  190, 0x32 },
	{  160, 0x02 },
};

int csi_phy_get_hsfreqrange(struct csiw_device *dev, unsigned int id, u32 *hsfreqrange)
{
	union drx_reg drx_reg;
	u32 reg;

	if (!dev)
		return -ENODEV;

	if (!id || id == 1)
		reg = DRX_REG0;
	else if (id == 2 || id == 3)
		reg = DRX_REG2;
	else
		return -EINVAL;

	drx_reg.value = csiw_read(dev, reg);

	if (id % 2 == 0)
		*hsfreqrange = drx_reg.p0_hsfreqrange;
	else
		*hsfreqrange = drx_reg.p1_hsfreqrange;
	return 0;
}
EXPORT_SYMBOL(csi_phy_get_hsfreqrange);

int csi_phy_set_hsfreqrange(struct csiw_device *dev, unsigned int id, unsigned int rate)
{
	union drx_reg drx_reg;
	u32 reg, i;
	unsigned int range = 0;

	if (!dev)
		return -ENODEV;

	if (!id || id == 1)
		reg = DRX_REG0;
	else if (id == 2 || id == 3)
		reg = DRX_REG2;
	else
		return -EINVAL;

	drx_reg.value = csiw_read(dev, reg);

	for (i = 0; i < ARRAY_SIZE(rr_list); i++) {
		if (rate >= rr_list[i].rate) {
			range = rr_list[i].range;
			break;
		}
	}

	if (!range)
		return -EINVAL;

	if (id % 2 == 0)
		drx_reg.p0_hsfreqrange = range;
	else
		drx_reg.p1_hsfreqrange = range;

	csiw_write(dev, reg, drx_reg.value);
	return 0;
}
EXPORT_SYMBOL(csi_phy_set_hsfreqrange);

int csi_phy_enable_ppipg_clk(struct csiw_device *dev, bool enable)
{
	union drx_reg4 drx_reg;

	if (!dev)
		return -ENODEV;

	drx_reg.value = csiw_read(dev, DRX_REG4);
	drx_reg.ppipg_clk_enable = enable ? 1 : 0;
	csiw_write(dev, DRX_REG4, drx_reg.value);
	return 0;
}
EXPORT_SYMBOL(csi_phy_enable_ppipg_clk);

void csi_phy_set_testcode(struct csiw_device *dev, unsigned int id, unsigned int code)
{
	u32 val, reg_offset;

	if (!dev || id >= PHY_RX_NUM)
		return;
	if (id < 2)
		reg_offset = PHY_TESTCODE0;
	else
		reg_offset = PHY_TESTCODE1;
	val = csiw_read(dev, reg_offset);
	if (id % 2)
		val = (code << 12) | (val & 0xfff);
	else
		val = (code & 0xfff) | (val & 0xfff000);
	csiw_write(dev, reg_offset, val);
}
EXPORT_SYMBOL(csi_phy_set_testcode);

void csi_phy_testcode_sel_other_phy(struct csiw_device *dev, unsigned int id, bool sel_other)
{
	union drx_reg4 reg4;

	if (!dev || id >= PHY_RX_NUM)
		return;
	reg4.value = csiw_read(dev, DRX_REG4);
	if (id < 2)
		reg4.test_sel_4l_0 = (u32)sel_other;
	else
		reg4.test_sel_4l_1 = (u32)sel_other;

	csiw_write(dev, DRX_REG4, reg4.value);
}
EXPORT_SYMBOL(csi_phy_testcode_sel_other_phy);

void put_csi_wrapper_device(struct csiw_device *dev)
{
	if (dev)
		put_device(dev->dev);
}
EXPORT_SYMBOL(put_csi_wrapper_device);

static int csiw_probe(struct platform_device *pdev)
{
	struct csiw_device *csiw;
	struct resource *res;

	csiw = devm_kzalloc(&pdev->dev, sizeof(*csiw), GFP_KERNEL);
	if (!csiw)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	csiw->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(csiw->base)) {
		dev_err(&pdev->dev, "cannot get reg mem resource\n");
		return PTR_ERR(csiw->base);
	}
	csiw->dev = &pdev->dev;
	platform_set_drvdata(pdev, csiw);
	return 0;
}

static int csiw_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id csiw_of_match[] = {
	{ .compatible = CSIW_DT_NAME },
	{ },
};

MODULE_DEVICE_TABLE(of, csiw_of_match);

static struct platform_driver csiw_driver = {
	.probe  = csiw_probe,
	.remove = csiw_remove,
	.driver = {
		.name = CSIW_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = csiw_of_match,
	}
};

static int __init csiw_init_module(void)
{
	return platform_driver_register(&csiw_driver);
}

static void __exit csiw_exit_module(void)
{
	platform_driver_unregister(&csiw_driver);
}

module_init(csiw_init_module);
module_exit(csiw_exit_module);

MODULE_DESCRIPTION("VeriSilicon CSI Wrapper Driver");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
MODULE_LICENSE("GPL");
MODULE_ALIAS("VS-CSI-Wrapper");
