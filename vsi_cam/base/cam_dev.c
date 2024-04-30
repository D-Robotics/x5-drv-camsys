// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include "cam_uapi.h"

#include "cam_dev.h"

static int parse_mem_res(struct platform_device *pdev, struct mem_res *mems)
{
	struct resource *res;
	int i = 0;

	if (!mems)
		return 0;

	while (mems[i].name) {
		res = platform_get_resource_byname
				(pdev, IORESOURCE_MEM, mems[i].name);
		mems[i].base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(mems[i].base)) {
			dev_err(&pdev->dev, "cannot get reg mem resource\n");
			return PTR_ERR(mems[i].base);
		}
		i++;
	}
	return 0;
}

static int parse_irq_res(struct platform_device *pdev, struct irq_res *irqs)
{
	int rc, i = 0;

	if (!irqs)
		return 0;

	while (irqs[i].name) {
		irqs[i].no = platform_get_irq_byname(pdev, irqs[i].name);
		if (irqs[i].no < 0) {
			dev_err(&pdev->dev, "cannot get %s irq resource\n",
				irqs[i].name);
			return irqs[i].no;
		}

		rc = devm_request_irq(&pdev->dev, irqs[i].no, irqs[i].handler,
				      IRQF_TRIGGER_HIGH | IRQF_SHARED,
				      irqs[i].name, irqs[i].arg);
		if (rc) {
			dev_err(&pdev->dev, "cannot request %s irq\n",
				irqs[i].name);
			return rc;
		}
		i++;
	}
	return 0;
}

static int parse_clk_res(struct platform_device *pdev, struct clk_res *clks)
{
	int i = 0;

	if (!clks)
		return 0;

	while (clks[i].name) {
		clks[i].clk = devm_clk_get(&pdev->dev, clks[i].name);
		if (IS_ERR_OR_NULL(clks[i].clk)) {
			dev_err(&pdev->dev, "cannot get %s clock\n", clks[i].name);
			return IS_ERR(clks[i].clk) ? PTR_ERR(clks[i].clk) : -ENXIO;
		}
		i++;
	}
	return 0;
}

static int parse_rst_res(struct platform_device *pdev, struct rst_res *rsts)
{
	int i = 0;

	if (!rsts)
		return 0;

	while (rsts[i].name) {
		rsts[i].rst = devm_reset_control_get(&pdev->dev, rsts[i].name);
		if (IS_ERR_OR_NULL(rsts[i].rst)) {
			dev_err(&pdev->dev, "cannot get %s reset control\n", rsts[i].name);
			return IS_ERR(rsts[i].rst) ? PTR_ERR(rsts[i].rst) : -ENXIO;
		}
		i++;
	}
	return 0;
}


int parse_cam_dt(struct platform_device *pdev, struct cam_dt *dt, void *arg)
{
	int rc;

	rc = of_property_read_u32(pdev->dev.of_node, "id", &dt->id);
	if (rc < 0)
		dt->id = 0;

	rc = of_property_read_u32(pdev->dev.of_node, "instances", &dt->num_insts);
	if (rc < 0)
		dt->num_insts = 1;

	rc = parse_mem_res(pdev, dt->mems);
	if (rc < 0)
		return rc;

	rc = parse_irq_res(pdev, dt->irqs);
	if (rc < 0)
		return rc;

	rc = parse_clk_res(pdev, dt->clks);
	if (rc < 0)
		return rc;

	return parse_rst_res(pdev, dt->rsts);
}

bool check_framesize(struct cam_res_cap *cap, u32 size, struct cam_format *fmt)
{
	u32 i;

	for (i = 0; i < size; i++) {
		if (cap[i].type == CAP_DC) {
			if (cap[i].dc.width == fmt->width &&
			    cap[i].dc.height == fmt->height)
				return true;
		} else if (cap[i].type == CAP_SW) {
			if (cap[i].sw.min_width <= fmt->width &&
			    cap[i].sw.max_width >= fmt->width &&
			    cap[i].sw.min_height <= fmt->height &&
			    cap[i].sw.max_height >= fmt->height &&
			    !((fmt->width - cap[i].sw.min_width) %
					cap[i].sw.step_width) &&
			    !((fmt->height - cap[i].sw.min_height) %
					cap[i].sw.step_height))
				return true;
		}
	}
	return false;
}

u32 get_framebuf_size(struct cam_format *fmt)
{
	switch (fmt->format) {
	case CAM_FMT_NV12:
		return fmt->stride * fmt->height * 3 / 2;
	case CAM_FMT_RAW8:
		return fmt->stride * fmt->height;
	case CAM_FMT_RAW10:
	case CAM_FMT_RAW12:
	case CAM_FMT_YUYV:
	case CAM_FMT_NV16:
		return fmt->stride * fmt->height * 2;
	default:
		return 0;
	}
}

static inline void *of_get_phandle(struct device_node *dn, const char *name)
{
	struct platform_device *pdev;
	struct device_node *ph_dn;
	void *ph;

	ph_dn = of_parse_phandle(dn, name, 0);
	if (!ph_dn)
		return ERR_PTR(-EINVAL);

	pdev = of_find_device_by_node(ph_dn);
	of_node_put(ph_dn);
	if (!pdev)
		return ERR_PTR(-ENODEV);

	ph = platform_get_drvdata(pdev);
	if (!ph) {
		platform_device_put(pdev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	return ph;
}

struct cam_ctrl_device *get_cam_ctrl_device(struct platform_device *pdev)
{
	return of_get_phandle(pdev->dev.of_node, "cam-ctrl");
}

struct csiw_device *get_csi_wrapper_device(struct platform_device *pdev)
{
	return of_get_phandle(pdev->dev.of_node, "csi-wrapper");
}
