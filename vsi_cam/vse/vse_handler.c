// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "[vse_drv]: %s: " fmt, __func__

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>

#include "cam_ctx.h"
#include "dw230_vse_regs.h"
#include "isc.h"
#include "vse_uapi.h"

#include "vse.h"

static s32 handle_set_fmt_cap(struct vse_device *vse, struct vse_msg *msg)
{
	struct vse_instance *ins;
	struct vse_format_cap *cap, *c = NULL;
	u32 i;

	if (msg->inst >= vse->num_insts)
		return -EINVAL;

	ins = &vse->insts[msg->inst];

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

	if (msg->fcap.index >= ARRAY_SIZE(c->res))
		return -EINVAL;

	for (i = 0; i < msg->fcap.res_num; i++)
		c->res[i][msg->fcap.index] = msg->fcap.res[i];
	return 0;
}

static s32 handle_set_state(struct vse_device *vse, struct vse_msg *msg)
{
	return 0;
}

static s32 handle_reset_control(struct vse_device *vse, struct vse_msg *msg)
{
	vse_reset(vse);
	return 0;
}

static s32 handle_change_input(struct vse_device *vse, struct vse_msg *msg)
{
	struct vse_instance *ins;

	if (msg->inst >= vse->num_insts)
		return -EINVAL;

	ins = &vse->insts[msg->inst];

	memset(ins->fmt_cap, 0, sizeof(ins->fmt_cap));
	return 0;
}

static s32 handle_alloc_cmd_buf(struct vse_device *vse, struct vse_msg *msg)
{
	struct vse_instance *ins;

	if (msg->inst >= vse->num_insts)
		return -EINVAL;

	ins = &vse->insts[msg->inst];

	if (msg->cmd.size > 0) {
		ins->cmd_buf_va = dma_alloc_coherent(vse->dev, msg->cmd.size, &msg->cmd.addr,
						  GFP_KERNEL);
		if (!ins->cmd_buf_va)
			return -ENOMEM;
	} else {
		dma_free_coherent(vse->dev, ins->cmd_buf.size, ins->cmd_buf_va, ins->cmd_buf.addr);
		ins->cmd_buf_va = NULL;
	}
	ins->cmd_buf = msg->cmd;
	return 0;
}

static s32 handle_set_osd_buf(struct vse_device *vse, struct vse_msg *msg)
{
	struct vse_instance *ins;

	if (msg->inst >= vse->num_insts || msg->channel >= VSE_OUT_CHNL_MAX || msg->osd.id >= 4)
		return -EINVAL;

	ins = &vse->insts[msg->inst];

    ins->osd[msg->channel][msg->osd.id] = msg->osd.buf;
	return 0;
}

s32 vse_msg_handler(void *msg, u32 len, void *arg)
{
	struct vse_device *vse = (struct vse_device *)arg;
	struct vse_msg *m = (struct vse_msg *)msg;
	s32 rc = 0;

	if (!vse || !msg || !len)
		return -EINVAL;

	switch (m->id) {
	case CAM_MSG_READ_REG:
		m->reg.value = vse_read(vse, m->reg.offset);
		break;
	case CAM_MSG_WRITE_REG:
		vse_write(vse, m->reg.offset, m->reg.value);
		break;
	case CAM_MSG_CHANGE_INPUT:
		rc = handle_change_input(vse, m);
		break;
	case CAM_MSG_SET_FMT_CAP:
		rc = handle_set_fmt_cap(vse, m);
		break;
	case CAM_MSG_SET_STATE:
		rc = handle_set_state(vse, m);
		break;
	case CAM_MSG_RESET_CONTROL:
		rc = handle_reset_control(vse, m);
		break;
	case VSE_MSG_ALLOC_CMD_BUF:
		rc = handle_alloc_cmd_buf(vse, m);
		break;
	case VSE_MSG_SET_OSD_BUF:
		rc = handle_set_osd_buf(vse, m);
		break;
	default:
		return -EINVAL;
	}
	return rc;
}

static inline void frame_done(struct vse_instance *inst, bool timeout)
{
	struct vse_irq_ctx *ctx = &inst->ctx;
	ktime_t now_time = ktime_get_boottime();
	u32 i;

	if (ctx->sink_buf) {
		cam_qbuf_irq(ctx->sink_ctx, ctx->sink_buf, false);
		ctx->sink_buf = NULL;
	}

	for (i = 0; i < VSE_OUT_CHNL_MAX; i++) {
		if (ctx->src_buf[i]) {
			if (timeout) {
				cam_drop(ctx->src_ctx[i]);
				ctx->src_buf[i] = NULL;
			} else if (!cam_osd_update(ctx->src_ctx[i])) {
				cam_qbuf_irq(ctx->src_ctx[i], ctx->src_buf[i], true);
				ctx->src_buf[i] = NULL;
			}
			if (inst->last_frame_done[i])
				inst->frame_interval[i] += ktime_to_ms
						(ktime_sub(now_time, inst->last_frame_done[i]));
			inst->last_frame_done[i] = now_time;
			inst->frame_count[i]++;
		}
	}

	cam_set_stat_info(ctx->stat_ctx, CAM_STAT_FE);
}

int new_frame(struct vse_irq_ctx *ctx)
{
	struct cam_buf *buf = NULL;
	u32 i, count = 0;
	u32 enable = 0;

	if (!ctx) {
		pr_err("newframe vse_irq_ctx null\n");
		return -1;
	}

	if (!ctx->is_sink_online_mode && ctx->sink_ctx) {
		buf = cam_acqbuf_irq(ctx->sink_ctx);
		if (!buf)
			return -ENOMEM;
	}

	for (i = 0; i < VSE_OUT_CHNL_MAX; i++) {
		if (!ctx->src_ctx[i])
			continue;
		ctx->cur_fps[i].cnt += ctx->cur_fps[i].dst;
		if (ctx->cur_fps[i].cnt >= ctx->cur_fps[i].src) {
			ctx->cur_fps[i].cnt -= ctx->cur_fps[i].src;
			enable |= BIT(i);
		}
		if (!ctx->cur_fps[i].cnt && ctx->cur_fps[i].dst != ctx->fps[i].dst)
			ctx->cur_fps[i].dst = ctx->fps[i].dst;
		if (!(enable & BIT(i)))
			continue;
		ctx->src_buf[i] = cam_dqbuf_irq(ctx->src_ctx[i], true);
		if (ctx->src_buf[i]) {
			count++;
			if (ctx->is_sink_online_mode)
				sif_get_frame_des(ctx->src_ctx[i]);
		}
	}

	if (!enable) {
		/* keep sink buf in loop if all out channels are disabled at a time */
		if (buf) {
			buf = cam_dqbuf_irq(ctx->sink_ctx, false);
			if (buf)
				cam_qbuf_irq(ctx->sink_ctx, buf, false);
		}
		return 1;
	} else if (!count) {
		return -ENOMEM;
	}

	if (buf)
		ctx->sink_buf = cam_dqbuf_irq(ctx->sink_ctx, false);
	return 0;
}

struct vse_irq_ctx *get_next_irq_ctx(struct vse_device *vse)
{
	struct vse_instance *inst;
	struct vse_irq_ctx *ctx = NULL;
	struct irq_job job;
	unsigned long flags;
	// u32 id = vse->next_irq_ctx;
	int rc;

	if (vse->mode == VSE_SCM_MODE) {
		inst = &vse->insts[0];
		if (inst->state != CAM_STATE_STARTED)
			return NULL;
		spin_lock_irqsave(&inst->lock, flags);
		ctx = &inst->ctx;
		rc = new_frame(ctx);
		spin_unlock_irqrestore(&inst->lock, flags);
		if (!rc)
			return ctx;
		return NULL;
	}

	for (;;) {
		rc = pop_job(vse->jq, &job);
		if (rc < 0) {
#if 0
			inst = &vse->insts[id];
			spin_lock_irqsave(&inst->lock, flags);
			ctx = &inst->ctx;
			new_frame(ctx);
			spin_unlock_irqrestore(&inst->lock, flags);
			vse->next_irq_ctx = id;
#endif
			ctx = NULL;
			break;
		}

		inst = &vse->insts[job.irq_ctx_index];
		if (inst->state != CAM_STATE_STARTED)
			continue;
		spin_lock_irqsave(&inst->lock, flags);
		ctx = &inst->ctx;
		rc = new_frame(ctx);
		spin_unlock_irqrestore(&inst->lock, flags);
		if (rc == 1 && ctx->is_sink_online_mode) {
			struct vse_msg msg = { .id = VSE_MSG_SKIP_FRAME };

			msg.inst = -1;
			msg.channel = -1;
			vse_post(vse, &msg, false);
			pr_debug("vse fps skip frame\n");
		}
		if (!rc) {
			vse->next_irq_ctx = job.irq_ctx_index;
			break;
		}
	}
	return ctx;
}

irqreturn_t vse_irq_handler(int irq, void *arg)
{
	struct vse_device *vse = (struct vse_device *)arg;
	struct vse_msg msg = { .id = VSE_MSG_IRQ_STAT };
	struct vse_instance *inst;
	struct vse_irq_ctx *ctx;
	unsigned long flags;
	u32 mis, mis1, i, vse_ctrl, pre_ctrl, value, base;
	u8 chnl;

	mis = vse_read(vse, VSE_MI_MIS);
	if (mis) {
		vse_write(vse, VSE_MI_ICR, mis);
		pr_debug("+mi mis:0x%x\n", mis);
	}

	mis1 = vse_read(vse, VSE_MI_MIS1);
	if (mis1) {
		vse_write(vse, VSE_MI_ICR1, mis1);
		pr_debug("+mi mis1:0x%x\n", mis1);
	}
	if (mis1 & 0x5)
		mis = BIT(13);

	if (mis & BIT(13)) {
		inst = &vse->insts[vse->next_irq_ctx];
		if (inst->is_need_read_hist && !(mis1 & 0x5)) {
			for (i = 0; i < VSE_OUT_CHNL_MAX; i++) {
				if (inst->ctx.src_buf[i])
					cam_read_hist(inst->ctx.src_ctx[i], i);
			}
			inst->is_need_read_hist = false;
			inst->is_hist_num_updated = true;
		}

		value = vse_read(vse, VSE_MI0_BUS_CFG);
		value &= ~BIT(21);
		vse_write(vse, VSE_MI0_BUS_CFG, value);
		value = vse_read(vse, VSE_MI_IMSC1);
		value &= ~0x5;
		vse_write(vse, VSE_MI_IMSC1, value);
		spin_lock_irqsave(&inst->lock, flags);
		frame_done(inst, !!(mis1 & 0x5));
		spin_unlock_irqrestore(&inst->lock, flags);

		ctx = get_next_irq_ctx(vse);
		if (!ctx) {
			vse->is_completed = true;
			vse->error = 1;
		} else if (vse->mode == VSE_SCM_MODE) {
			pre_ctrl = vse_read(vse, VSE_CTRL);
			vse_ctrl = pre_ctrl;
			vse_ctrl &= ~0x3f;
			for (i = 0; i < VSE_OUT_CHNL_MAX; i++) {
				if (ctx->src_buf[i]) {
					phys_addr_t phys_addr = get_phys_addr(ctx->src_buf[i], 0);

					vse_set_mi_buffer(vse, i, phys_addr, &inst->ofmt[i]);
					vse_ctrl |= BIT(i);
				}
			}
			if (vse_ctrl != pre_ctrl) {
				vse_write(vse, VSE_CTRL, vse_ctrl);
				pr_debug("vse_ctrl %x->%x\n", pre_ctrl, vse_ctrl);
			}
			for (chnl = 0; chnl < VSE_OUT_CHNL_MAX; chnl++) {
				if (chnl == 4) /* exclude path4 in all scenarios */
					continue;
				base = VSE_OSDn_BASE(chnl);
				vse_write(vse, VSE_OSDn_CTRL(base), 0x00000c7f);
			}
		} else {
			vse_set_cmd(vse, vse->next_irq_ctx);
		}

		msg.inst = -1;
		msg.channel = -1;
		msg.irq.num = VSE_MI_MIS;
		msg.irq.stat = mis;
		vse_post(vse, &msg, false);
	}
	pr_debug("-\n");
	return IRQ_HANDLED;
}

irqreturn_t vse_fe_irq_handler(int irq, void *arg)
{
	struct vse_device *vse = (struct vse_device *)arg;
	struct vse_msg msg = { .id = VSE_MSG_IRQ_STAT };
	u32 value;

	value = vse_read(vse, VSE_FE_MIS);
	pr_debug("+fe mis:0x%x\n", value);
	if (value) {
		vse_write(vse, VSE_FE_ICR, value);
		if (value & BIT(0)) {
			msg.inst = -1;
			msg.channel = -1;
			msg.irq.num = VSE_FE_MIS;
			msg.irq.stat = value;
			vse_post(vse, &msg, false);
		}
	}
	pr_debug("-\n");
	return IRQ_HANDLED;
}
