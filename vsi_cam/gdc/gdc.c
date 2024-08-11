// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "cam_buf.h"
#include "cam_ctrl.h"
#include "cam_dev.h"
#include "isc.h"
#include "gdc_uapi.h"

#include "gdc.h"

static bool check_format(struct gdc_instance *ins, struct cam_format *fmt)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(ins->fmt_cap); i++) {
		struct gdc_format_cap *cap = &ins->fmt_cap[i];

		if (!cap->format)
			return false;
		if (cap->format == fmt->format)
			return check_framesize(cap->res, ARRAY_SIZE(cap->res), fmt);
	}
	return false;
}

void gdc_post(struct gdc_device *gdc, void *msg, u32 len)
{
	struct isc_post_param param = {
		.msg = msg,
		.msg_len = len,
		.lock = &gdc->isc_lock,
		.sync = false,
	};

	if (gdc->isc)
		isc_post(gdc->isc, &param);
}

void set_ibuffer(struct gdc_device *gdc, struct cam_format *fmt,
		 struct mem_buf *cfg, phys_addr_t buf)
{
	pr_info("gdc_read (0x00000060)=(0x%08x)\n", gdc_read(gdc, 0x00000060));
	gdc_write(gdc, 0x00000010, cfg->addr);
	gdc_write(gdc, 0x00000014, 0x733);//cfg->size);
	gdc_write(gdc, 0x00000020, fmt->width);
	gdc_write(gdc, 0x00000024, fmt->height);
	gdc_write(gdc, 0x00000028, buf);
	gdc_write(gdc, 0x0000002c, fmt->stride);
	gdc_write(gdc, 0x00000030, buf + fmt->stride * fmt->height);
	gdc_write(gdc, 0x00000034, fmt->stride);
}

void set_obuffer(struct gdc_device *gdc, struct cam_format *fmt, phys_addr_t buf)
{
	gdc_write(gdc, 0x00000040, fmt->width);
	gdc_write(gdc, 0x00000044, fmt->height);
	gdc_write(gdc, 0x00000048, buf);
	gdc_write(gdc, 0x0000004c, fmt->stride);
	gdc_write(gdc, 0x00000050, buf + fmt->stride * fmt->height);
	gdc_write(gdc, 0x00000054, fmt->stride);
}

void gdc_start(struct gdc_device *gdc)
{
	gdc_write(gdc, 0x00000064, 0x00000001);
	gdc_write(gdc, 0x00000064, 0x00000000);
}

void gdc_stop(struct gdc_device *gdc)
{
	gdc_write(gdc, 0x00000064, 0x00000002);
	gdc_write(gdc, 0x00000064, 0x00000000);
}

int gdc_set_format(struct gdc_device *gdc, u32 inst, struct gdc_format *fmt)
{
	struct gdc_instance *ins;
	struct gdc_msg msg;

	if (!gdc || !fmt)
		return -EINVAL;

	if (inst >= gdc->num_insts)
		return -EINVAL;

	ins = &gdc->insts[inst];

	if (!check_format(ins, &fmt->ofmt))
		return -EINVAL;

	memcpy(&ins->fmt, fmt, sizeof(ins->fmt));

	msg.id = CAM_MSG_FORMAT_CHANGED;
	msg.inst = inst;
	memcpy(&msg.fmt, fmt, sizeof(msg.fmt));
	gdc_post(gdc, &msg, sizeof(msg));
	return 0;
}

int gdc_set_state(struct gdc_device *gdc, u32 inst, int enable)
{
	struct gdc_msg msg;

	if (!gdc || inst >= gdc->num_insts)
		return -EINVAL;

	if (!enable)
		gdc_stop(gdc);

	msg.id = CAM_MSG_STATE_CHANGED;
	msg.inst = inst;
	if (enable)
		msg.state = CAM_STATE_STARTED;
	else
		msg.state = CAM_STATE_STOPPED;
	gdc_post(gdc, &msg, sizeof(msg));
	return 0;
}

int gdc_set_ctx(struct gdc_device *gdc, u32 inst, struct gdc_irq_ctx *ctx)
{
	struct gdc_instance *ins;
	unsigned long flags;

	if (!gdc || !ctx)
		return -EINVAL;

	if (inst >= gdc->num_insts)
		return -EINVAL;

	ins = &gdc->insts[inst];
	spin_lock_irqsave(&ins->lock, flags);
	ins->ctx = *ctx;
	spin_unlock_irqrestore(&ins->lock, flags);
	return 0;
}

int gdc_add_job(struct gdc_device *gdc, u32 inst)
{
	struct irq_job job = { inst };
	struct gdc_instance *ins;
	struct gdc_irq_ctx *ctx;
	phys_addr_t buf;
	int rc;

	rc = push_job(gdc->jq, &job);
	if (rc < 0) {
		dev_err(gdc->dev, "failed to push a job(err=%d)\n", rc);
		return rc;
	}

	if (gdc->error) {
		ins = &gdc->insts[gdc->next_irq_ctx];
		ctx = get_next_irq_ctx(gdc);
		if (ctx->sink_buf && ctx->src_buf) {
			gdc->error = 0;
			buf = get_phys_addr(ctx->sink_buf, 0);
			pr_info("%s sink_buf=%x\n", __func__, (u32)buf);
			set_ibuffer(gdc, &ins->fmt.ifmt, &ins->cfg_buf, buf);
			buf = get_phys_addr(ctx->src_buf, 0);
			pr_info("%s src_buf=%x\n", __func__, (u32)buf);
			set_obuffer(gdc, &ins->fmt.ofmt, buf);
			gdc_start(gdc);
		}
	}
	return 0;
}

static void gdc_bound(struct isc_handle *isc, void *arg)
{
	struct gdc_device *gdc = (struct gdc_device *)arg;
	unsigned long flags;

	if (gdc) {
		spin_lock_irqsave(&gdc->isc_lock, flags);
		if (!gdc->isc) {
			isc_get(isc);
			gdc->isc = isc;
		}
		spin_unlock_irqrestore(&gdc->isc_lock, flags);
	}
}

static void gdc_unbind(void *arg)
{
	struct gdc_device *gdc = (struct gdc_device *)arg;
	unsigned long flags;

	if (gdc) {
		spin_lock_irqsave(&gdc->isc_lock, flags);
		if (gdc->isc) {
			isc_put(gdc->isc);
			gdc->isc = NULL;
		}
		spin_unlock_irqrestore(&gdc->isc_lock, flags);
	}
}

static struct isc_notifier_ops gdc_notifier_ops = {
	.bound = gdc_bound,
	.unbind = gdc_unbind,
	.got = gdc_msg_handler,
};

int gdc_probe(struct platform_device *pdev, struct gdc_device *gdc)
{
	struct device *dev = &pdev->dev;
	int rc;
	struct mem_res gdc_mems[] = {
		{ "reg", NULL },
		{},
	};
	struct irq_res gdc_irqs[] = {
		{ "gdc", -1, gdc_irq_handler, gdc },
		{},
	};
	struct clk_res gdc_clks[] = {
		{ "core", NULL },
		{ "axi", NULL },
		{ "hclk", NULL },
		{ "vse_core", NULL },
		{ "vse_ups", NULL },
		{},
	};
	struct rst_res gdc_rsts[] = {
		{ "rst", NULL },
		{},
	};
	struct cam_dt gdc_dt = {
		.id = 0,
		.num_insts = 0,
		.mems = gdc_mems,
		.irqs = gdc_irqs,
		.clks = gdc_clks,
		.rsts = gdc_rsts,
	};
	u32 i;

	if (!gdc)
		return -EINVAL;

	rc = parse_cam_dt(pdev, &gdc_dt, gdc);
	if (rc < 0) {
		dev_err(dev, "failed to call parse_cam_dt (err=%d)\n", rc);
		return rc;
	}

	gdc->dev = dev;
	gdc->id = gdc_dt.id;
	gdc->num_insts = gdc_dt.num_insts;
	gdc->base = gdc_dt.mems[0].base;
	gdc->core = gdc_dt.clks[0].clk;
	gdc->axi = gdc_dt.clks[1].clk;
	gdc->hclk = gdc_dt.clks[2].clk;
	gdc->vse_core = gdc_dt.clks[3].clk;
	gdc->vse_ups = gdc_dt.clks[4].clk;
	gdc->rst = gdc_dt.rsts[0].rst;
	spin_lock_init(&gdc->isc_lock);

	gdc->error = 1;

	gdc->insts = devm_kcalloc(dev, gdc_dt.num_insts,
				  sizeof(*gdc->insts), GFP_KERNEL);
	if (!gdc->insts)
		return -ENOMEM;

	gdc->ctrl_dev = get_cam_ctrl_device(pdev);
	if (IS_ERR(gdc->ctrl_dev))
		return PTR_ERR(gdc->ctrl_dev);

	gdc->jq = create_job_queue(32);
	if (!gdc->jq) {
		dev_err(dev, "failed to call create_job_queue\n");
		return -ENOMEM;
	}

	rc = isc_register(GDC_UID(gdc->id), &gdc_notifier_ops, gdc);
	if (rc < 0) {
		dev_err(dev, "failed to call isc_register (err=%d)\n", rc);
		destroy_job_queue(gdc->jq);
		return rc;
	}

	for (i = 0; i < gdc_dt.num_insts; i++)
		spin_lock_init(&gdc->insts[i].lock);

	INIT_LIST_HEAD(&gdc->in_buf_list);
	dev_dbg(dev, "ARM GDC driver (base) probed done\n");
	return 0;
}

int gdc_remove(struct platform_device *pdev, struct gdc_device *gdc)
{
	int rc;

	rc = isc_unregister(GDC_UID(gdc->id));
	if (rc < 0)
		dev_err(&pdev->dev, "failed to call isc_unregister (err=%d)\n", rc);

	destroy_job_queue(gdc->jq);
	put_cam_ctrl_device(gdc->ctrl_dev);

	dev_dbg(&pdev->dev, "ARM GDC driver (base) removed\n");
	return rc;
}

#ifdef CONFIG_PM_SLEEP
int gdc_system_suspend(struct device *dev)
{
	return pm_runtime_force_suspend(dev);
}

int gdc_system_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}
#endif

#ifdef CONFIG_PM
int gdc_runtime_suspend(struct device *dev)
{
	struct gdc_device *gdc = dev_get_drvdata(dev);

	if (gdc->core)
		clk_disable_unprepare(gdc->core);
	if (gdc->vse_core)
		clk_disable_unprepare(gdc->vse_core);
	if (gdc->vse_ups)
		clk_disable_unprepare(gdc->vse_ups);
	if (gdc->hclk)
		clk_disable_unprepare(gdc->hclk);
	return 0;
}

int gdc_runtime_resume(struct device *dev)
{
	struct gdc_device *gdc = dev_get_drvdata(dev);
	int rc;

	if (gdc->core) {
		rc = clk_prepare_enable(gdc->core);
		if (rc)
			return rc;
	}
	if (gdc->vse_core) {
		rc = clk_prepare_enable(gdc->vse_core);
		if (rc)
			goto _vse_core_err;
	}
	if (gdc->vse_ups) {
		rc = clk_prepare_enable(gdc->vse_ups);
		if (rc)
			goto _vse_ups_err;
	}
	if (gdc->hclk) {
		rc = clk_prepare_enable(gdc->hclk);
		if (rc)
			goto _hclk_err;
	}
	return 0;
_hclk_err:
	if (gdc->vse_ups)
		clk_disable_unprepare(gdc->vse_ups);
_vse_ups_err:
	if (gdc->vse_core)
		clk_disable_unprepare(gdc->vse_core);
_vse_core_err:
	if (gdc->core)
		clk_disable_unprepare(gdc->core);
	return rc;
}
#endif
