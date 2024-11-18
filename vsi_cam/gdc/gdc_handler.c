// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "[gdc_drv]: %s: " fmt, __func__

#include <linux/io.h>
#include <linux/interrupt.h>

#include "cam_buf.h"
#include "cam_ctrl.h"
#include "dw_crc.h"
#include "isc.h"
#include "gdc_uapi.h"

#include "gdc.h"

static inline void frame_done(struct gdc_irq_ctx *ctx)
{
	if (ctx->sink_buf) {
		cam_qbuf_irq(ctx->sink_ctx, ctx->sink_buf, false);
		ctx->sink_buf = NULL;
	}

	if (ctx->src_buf) {
		cam_qbuf_irq(ctx->src_ctx, ctx->src_buf, true);
		ctx->src_buf = NULL;
	}
}

int new_frame(struct gdc_irq_ctx *ctx)
{
	struct cam_buf *buf;

	if (ctx->sink_ctx) {
		buf = cam_acqbuf_irq(ctx->sink_ctx, false);
		if (!buf)
			return -ENOMEM;
	}

	if (ctx->src_ctx) {
		ctx->src_buf = cam_dqbuf_irq(ctx->src_ctx, true);
		if (!ctx->src_buf)
			return -ENOMEM;
	}

	if (ctx->sink_ctx)
		ctx->sink_buf = cam_dqbuf_irq(ctx->sink_ctx, false);
	return 0;
}

struct gdc_irq_ctx *get_next_irq_ctx(struct gdc_device *gdc)
{
	struct gdc_instance *inst;
	struct gdc_irq_ctx *ctx = NULL;
	struct irq_job job;
	unsigned long flags;
	// u32 id = gdc->next_irq_ctx;
	int rc;

	for (;;) {
		rc = pop_job(gdc->jq, &job);
		if (rc < 0) {
#if 0
			inst = &gdc->insts[id];
			spin_lock_irqsave(&inst->lock, flags);
			ctx = &inst->ctx;
			new_frame(ctx);
			spin_unlock_irqrestore(&inst->lock, flags);
			gdc->next_irq_ctx = id;
#endif
			ctx = NULL;
			break;
		}

		inst = &gdc->insts[job.irq_ctx_index];
		if (inst->state != CAM_STATE_STARTED)
			continue;
		spin_lock_irqsave(&inst->lock, flags);
		ctx = &inst->ctx;
		rc = new_frame(ctx);
		spin_unlock_irqrestore(&inst->lock, flags);
		if (!rc) {
			gdc->next_irq_ctx = job.irq_ctx_index;
			break;
		}
	}
	return ctx;
}

irqreturn_t gdc_irq_handler(int irq, void *arg)
{
	struct gdc_device *gdc = (struct gdc_device *)arg;
	struct gdc_instance *ins;
	struct gdc_irq_ctx *ctx;
	unsigned long flags;
	bool is_done = false;

	get_gdc_intr_stat_and_clear(gdc->ctrl_dev, NULL, &is_done);
	if (is_done) {
		pr_debug("gdc%d frame done\n", gdc->next_irq_ctx);
		ins = &gdc->insts[gdc->next_irq_ctx];
		spin_lock_irqsave(&ins->lock, flags);
		frame_done(&ins->ctx);
		spin_unlock_irqrestore(&ins->lock, flags);

		ctx = get_next_irq_ctx(gdc);
		if (!ctx) {
			gdc->error = 1;
		} else {
			pr_debug("gdc%d set cmd\n", gdc->next_irq_ctx);
			gdc_set_cmd(gdc, gdc->next_irq_ctx);
		}
	}
	return IRQ_HANDLED;
}
