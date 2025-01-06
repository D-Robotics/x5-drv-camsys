// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "[gdc_drv]: %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "cam_buf.h"
#include "cam_ctrl.h"
#include "cam_dev.h"
#include "dw_crc.h"
#include "gdc_uapi.h"

#include "gdc.h"

#ifdef EN_CHK_FMT
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
#endif

int gdc_set_format(struct gdc_device *gdc, u32 inst, struct gdc_format *fmt)
{
	struct gdc_instance *ins;

	if (!gdc || !fmt)
		return -EINVAL;

	if (inst >= gdc->num_insts)
		return -EINVAL;

	ins = &gdc->insts[inst];
#ifdef EN_CHK_FMT
	if (!check_format(ins, &fmt->ofmt))
		return -EINVAL;
#endif
	memcpy(&ins->fmt, fmt, sizeof(ins->fmt));

	return 0;
}

int gdc_get_format(struct gdc_device *gdc, u32 inst, struct gdc_format *fmt)
{
	struct gdc_instance ins = gdc->insts[inst];

	memcpy(fmt, &ins.fmt, sizeof(struct gdc_format));

	return 0;
}

int gdc_set_attr(struct gdc_device *gdc, u32 inst, phys_addr_t paddr, u32 size)
{
	if (!paddr || !size)
		return -EINVAL;

	gdc->cfg_bufs[inst].addr = paddr;
	gdc->cfg_bufs[inst].size = size;

	return 0;
}

int gdc_get_attr(struct gdc_device *gdc, u32 inst, phys_addr_t *paddr, u32 *size)
{
	*paddr = gdc->cfg_bufs[inst].addr;
	*size = gdc->cfg_bufs[inst].size;

	if (!*paddr || !*size)
		return -EINVAL;

	return 0;
}

int gdc_set_state(struct gdc_device *gdc, u32 inst, int enable)
{
	struct gdc_instance *ins;

	if (!gdc || inst >= gdc->num_insts)
		return -EINVAL;

	ins = &gdc->insts[inst];
	ins->state = enable ? CAM_STATE_STARTED : CAM_STATE_STOPPED;

	return dw_set_state(gdc->crc_dev, DW_MOD_GDC, ins->state);
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

void gdc_set_cmd(struct gdc_device *gdc, u32 inst)
{
	struct gdc_irq_ctx *ctx;
	struct gdc_instance *ins;

	if(!gdc)
		return;

	gdc->error = 0;
	ins = &gdc->insts[inst];
	ctx = &ins->ctx;
	if (ctx->sink_buf && ctx->src_buf) {
		gdc_hw_init(gdc);
		gdc_hw_set_format(gdc, inst, &ins->fmt);
		gdc_set_in_buffer(gdc, &ins->fmt.ifmt, ins->ctx.sink_buf);
		gdc_set_out_buffer(gdc, &ins->fmt.ofmt, ins->ctx.src_buf);
		gdc_set_cfg_buffer(gdc, gdc->cfg_bufs[inst].addr, gdc->cfg_bufs[inst].size);
		gdc_hw_start_process(gdc);
	}
}

int gdc_add_job(struct gdc_device *gdc, u32 inst)
{
	struct irq_job job = { inst };
	struct gdc_irq_ctx *ctx;
	struct gdc_instance *ins;
	unsigned long flags;
	int rc;

	rc = push_job(gdc->jq, &job);
	if (rc < 0) {
		dev_err(gdc->dev, "failed to push a job(err=%d)\n", rc);
		return rc;
	}
	ins = &gdc->insts[inst];
	ins->job_count++;

	spin_lock_irqsave(&gdc->err_lock, flags);
	if (gdc->error) {
		ctx = get_next_irq_ctx(gdc);
		if (ctx)
			gdc_set_cmd(gdc, gdc->next_irq_ctx);
	}
	spin_unlock_irqrestore(&gdc->err_lock, flags);
	return 0;
}

int gdc_wake_up(struct gdc_device *gdc, u32 inst)
{
	struct irq_job job = { inst };
	struct gdc_irq_ctx *ctx;
	struct gdc_instance *ins;
	unsigned long flags;
	int rc = 0;

	ins = &gdc->insts[inst];
	spin_lock_irqsave(&gdc->err_lock, flags);

	if (gdc->error || !ins->job_count) {
		rc = push_job(gdc->jq, &job);
		if (rc < 0)
			goto _exit;
		ins->job_count++;
		if (!gdc->error)
			goto _exit;

		ctx = get_next_irq_ctx(gdc);
		if (ctx)
			gdc_set_cmd(gdc, gdc->next_irq_ctx);
	}
_exit:
	spin_unlock_irqrestore(&gdc->err_lock, flags);
	return rc;
}

int gdc_open(struct gdc_device *gdc, u32 inst)
{
	int rc = 0;

	if (!gdc)
		return -EINVAL;

	mutex_lock(&gdc->open_lock);
	refcount_inc(&gdc->open_cnt);
	mutex_unlock(&gdc->open_lock);
	return rc;
}

int gdc_close(struct gdc_device *gdc, u32 inst)
{
	struct gdc_instance *ins;
	bool dis_gdc = false;
	int rc = 0;

	if (!gdc)
		return -EINVAL;

	if (inst >= gdc->num_insts)
		return -EINVAL;

	ins = &gdc->insts[inst];
	memset(&ins->fmt, 0, sizeof(ins->fmt));
	ins->error = 1;

	mutex_lock(&gdc->open_lock);
	if (refcount_read(&gdc->open_cnt) > REFCNT_INIT_VAL) {
		refcount_dec(&gdc->open_cnt);
		if (refcount_read(&gdc->open_cnt) == REFCNT_INIT_VAL)
			dis_gdc = true;
	}
	mutex_unlock(&gdc->open_lock);

	if (!dis_gdc)
		goto _exit;

	reset_job_queue(gdc->jq);
	gdc_stop(gdc);
	cam_iommu_unmap(gdc->cam_dev);

_exit:
	return rc;
}

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
	spin_lock_init(&gdc->err_lock);
	mutex_init(&gdc->open_lock);
	refcount_set(&gdc->open_cnt, REFCNT_INIT_VAL);
	gdc->error = 1;

	gdc->insts = devm_kcalloc(dev, gdc_dt.num_insts,
				  sizeof(*gdc->insts), GFP_KERNEL);
	if (!gdc->insts)
		return -ENOMEM;

	gdc->ctrl_dev = get_cam_ctrl_device(pdev);
	if (IS_ERR(gdc->ctrl_dev))
		return PTR_ERR(gdc->ctrl_dev);

	gdc->crc_dev = get_dw_crc_device(pdev);
	if (IS_ERR(gdc->crc_dev))
		return PTR_ERR(gdc->crc_dev);

	gdc->jq = create_job_queue(32);
	if (!gdc->jq) {
		dev_err(dev, "failed to call create_job_queue\n");
		return -ENOMEM;
	}

	for (i = 0; i < gdc_dt.num_insts; i++)
		spin_lock_init(&gdc->insts[i].lock);

	dev_dbg(dev, "ARM GDC driver (base) probed done\n");
	return 0;
}

int gdc_remove(struct platform_device *pdev, struct gdc_device *gdc)
{
	destroy_job_queue(gdc->jq);
	put_cam_ctrl_device(gdc->ctrl_dev);
	devm_kfree(&pdev->dev, gdc->insts);

	dev_dbg(&pdev->dev, "ARM GDC driver (base) removed\n");
	return 0;
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
	struct gdc_instance *ins;
	int inst;

	if (!gdc)
		return -EINVAL;

	for (inst = 0; inst < gdc->num_insts; inst++) {
		ins = &gdc->insts[inst];
		if (ins->state == CAM_STATE_STARTED)
			return -EBUSY;
	}

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
