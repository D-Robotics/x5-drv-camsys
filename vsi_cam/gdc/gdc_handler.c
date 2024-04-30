// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/reset.h>

#include "cam_buf.h"
#include "cam_ctrl.h"
#include "isc.h"
#include "gdc_uapi.h"

#include "gdc.h"

static s32 handle_set_fmt_cap(struct gdc_device *gdc, struct gdc_msg *msg)
{
	struct gdc_instance *ins;
	struct gdc_format_cap *cap, *c = NULL;
	u32 i;

	if (msg->inst >= gdc->num_insts)
		return -EINVAL;

	ins = &gdc->insts[msg->inst];

	for (i = 0; i < ARRAY_SIZE(ins->fmt_cap); i++) {
		cap = &ins->fmt_cap[i];
		if (cap->format == msg->fcap.format) {
			c = cap;
			break;
		} else if (cap->format == CAM_FMT_NULL) {
			c = cap;
			c->format = msg->fcap.format;
			break;
		}
	}

	if (!c)
		return -EINVAL;

	if (msg->fcap.index > ARRAY_SIZE(c->res))
		return -EINVAL;

	c->res[msg->fcap.index] = msg->fcap.res;
	return 0;
}

static s32 handle_set_state(struct gdc_device *gdc, struct gdc_msg *msg)
{
	return 0;
}

static s32 handle_reset_control(struct gdc_device *gdc, struct gdc_msg *msg)
{
	if (gdc->rst) {
		reset_control_assert(gdc->rst);
		udelay(2);
		reset_control_deassert(gdc->rst);
	}
	return 0;
}
static s32 handle_change_input(struct gdc_device *gdc, struct gdc_msg *msg)
{
	struct gdc_instance *ins;

	if (msg->inst >= gdc->num_insts)
		return -EINVAL;

	ins = &gdc->insts[msg->inst];

	memset(ins->fmt_cap, 0, sizeof(ins->fmt_cap));
	return 0;
}

static s32 handle_set_cfg_buf(struct gdc_device *gdc, struct gdc_msg *msg)
{
	struct gdc_instance *ins;

	if (msg->inst >= gdc->num_insts)
		return -EINVAL;

	ins = &gdc->insts[msg->inst];

	ins->cfg_buf = msg->cfg_buf;
	return 0;
}

s32 gdc_msg_handler(void *msg, u32 len, void *arg)
{
	struct gdc_device *gdc = (struct gdc_device *)arg;
	struct gdc_msg *m = (struct gdc_msg *)msg;
	s32 rc = 0;

	if (!gdc || !msg || !len)
		return -EINVAL;

	switch (m->id) {
	case CAM_MSG_READ_REG:
		m->reg.value = gdc_read(gdc, m->reg.offset);
		break;
	case CAM_MSG_WRITE_REG:
		gdc_write(gdc, m->reg.offset, m->reg.value);
		break;
	case CAM_MSG_CHANGE_INPUT:
		rc = handle_change_input(gdc, m);
		break;
	case CAM_MSG_SET_FMT_CAP:
		rc = handle_set_fmt_cap(gdc, m);
		break;
	case CAM_MSG_SET_STATE:
		rc = handle_set_state(gdc, m);
		break;
	case CAM_MSG_RESET_CONTROL:
		rc = handle_reset_control(gdc, m);
		break;
	case GDC_MSG_SET_CFG_BUF:
		rc = handle_set_cfg_buf(gdc, m);
		break;
	default:
		return -EINVAL;
	}
	return rc;
}

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
		buf = cam_acqbuf_irq(ctx->sink_ctx);
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
	u32 id = gdc->next_irq_ctx;
	int rc;

	for (;;) {
		rc = pop_job(gdc->jq, &job);
		if (rc < 0) {
			inst = &gdc->insts[id];
			spin_lock_irqsave(&inst->lock, flags);
			ctx = &inst->ctx;
			new_frame(ctx);
			spin_unlock_irqrestore(&inst->lock, flags);
			gdc->next_irq_ctx = id;
			break;
		}

		inst = &gdc->insts[job.irq_ctx_index];
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
	struct gdc_msg msg = { .id = GDC_MSG_IRQ_STAT };
	struct gdc_instance *ins;
	struct gdc_irq_ctx *ctx;
	phys_addr_t buf;
	unsigned long flags;
	bool is_done = false;

	get_gdc_intr_stat_and_clear(gdc->ctrl_dev, NULL, &is_done);
	if (is_done) {
		ins = &gdc->insts[gdc->next_irq_ctx];
		spin_lock_irqsave(&ins->lock, flags);
		frame_done(&ins->ctx);
		spin_unlock_irqrestore(&ins->lock, flags);

		ctx = get_next_irq_ctx(gdc);
		ins = &gdc->insts[gdc->next_irq_ctx];
		if (ctx->sink_buf && ctx->src_buf) {
			buf = get_phys_addr(ctx->sink_buf, 0);
			set_ibuffer(gdc, &ins->fmt.ifmt, &ins->cfg_buf, buf);
			buf = get_phys_addr(ctx->src_buf, 0);
			set_obuffer(gdc, &ins->fmt.ofmt, buf);
			gdc_start(gdc);
		} else {
			gdc->error = 1;
		}

		msg.inst = 0;
		msg.irq.num = 0;
		msg.irq.stat = 1;
		gdc_post(gdc, &msg, sizeof(msg));
	}
	return IRQ_HANDLED;
}
