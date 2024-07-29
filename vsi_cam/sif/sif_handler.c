// SPDX-License-Identifier: GPL-2.0-only

#define pr_fmt(fmt) "[sif_drv]: %s: " fmt, __func__

#include <linux/interrupt.h>

#include "cam_ctx.h"
#include "isc.h"
#include "sif_uapi.h"
#include "sif_regs.h"

#include "sif.h"

static s32 handle_set_fmt_cap(struct sif_device *sif, struct sif_msg *msg)
{
	struct sif_instance *ins;
	struct sif_format_cap *cap, *c = NULL;
	u32 i;

	if (msg->inst >= sif->num_insts)
		return -EINVAL;

	ins = &sif->insts[msg->inst];

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

	c->res[msg->fcap.index] = msg->fcap.res;
	return 0;
}

static s32 handle_set_state(struct sif_device *sif, struct sif_msg *msg)
{
	int rc;

	rc = sif_set_state(sif, msg->inst, msg->state, false);
	if (rc < 0) {
		pr_info("failed to call sif_set_state (rc=%d).\n", rc);
		return rc;
	}
	return 0;
}

static s32 handle_reset_control(struct sif_device *sif, struct sif_msg *msg)
{
	sif_reset(sif);
	return 0;
}

static s32 handle_set_format(struct sif_device *sif, struct sif_msg *msg)
{
	int rc;

	rc = sif_set_format(sif, msg->inst, &msg->fmt, false, BOTH_CHANNEL);
	if (rc < 0) {
		pr_info("failed to call sif_set_format (rc=%d).\n", rc);
		return rc;
	}
	return 0;
}

static s32 handle_change_input(struct sif_device *sif, struct sif_msg *msg)
{
	struct sif_instance *ins;

	if (msg->inst >= sif->num_insts)
		return -EINVAL;

	ins = &sif->insts[msg->inst];

	memset(ins->fmt_cap, 0, sizeof(ins->fmt_cap));
	ins->input_bayer_format = msg->in.sens.bayer_format;
	return 0;
}

static s32 handle_set_cfg(struct sif_device *sif, struct sif_msg *msg)
{
	struct sif_instance *ins;

	if (msg->inst >= sif->num_insts)
		return -EINVAL;

	ins = &sif->insts[msg->inst];

	memset(&ins->sif_cfg, 0, sizeof(ins->sif_cfg));
	ins->sif_cfg = msg->cfg;
	return 0;
}

s32 sif_msg_handler(void *msg, u32 len, void *arg)
{
	struct sif_device *sif = (struct sif_device *)arg;
	struct sif_msg *m = (struct sif_msg *)msg;
	s32 rc = 0;

	if (!sif || !msg || !len)
		return -EINVAL;

	switch (m->id) {
	case CAM_MSG_CHANGE_INPUT:
		rc = handle_change_input(sif, m);
		break;
	case CAM_MSG_SET_FMT_CAP:
		rc = handle_set_fmt_cap(sif, m);
		break;
	case CAM_MSG_SET_FORMAT:
		rc = handle_set_format(sif, m);
		break;
	case CAM_MSG_SET_STATE:
		rc = handle_set_state(sif, m);
		break;
	case CAM_MSG_RESET_CONTROL:
		rc = handle_reset_control(sif, m);
		break;
	case SIF_MSG_SET_CFG:
		rc = handle_set_cfg(sif, m);
		break;
	default:
		return -EINVAL;
	}
	return rc;
}

static void sif_handle_frame_size_err(struct sif_device *sif, u32 inst)
{
	u32 val;
	struct sif_instance *ins = &sif->insts[inst];

	if (ins->fmt.format == CAM_FMT_NV12)
		return;

	// Need to get error size before clear state.
	val = sif_read(sif, SIF_IPI_ERR_SIZE(inst));
	dev_err(sif->dev,"sif frame size incorrect, hsize count:0x%x vsize count:0x%x for inst:%d\n",
		SIF_HSIZE_ERR(val), SIF_VSIZE_ERR(val), inst);

	if (ins->hsize_err_count_pre != SIF_HSIZE_ERR(val)
	    && ins->vsize_err_count_pre == SIF_VSIZE_ERR(val)) {
		// meet Hsize error drop frame.
		cam_set_frame_status(ins->ctx.buf_ctx, HSIZE_ERR);
	} else if (ins->hsize_err_count_pre == SIF_HSIZE_ERR(val)
		   && ins->vsize_err_count_pre != SIF_VSIZE_ERR(val)) {
		// when hsync is more than exception, post warning.
		// otherwise, do nothing since where won't be dma done.
		if (SIF_VSIZE_ERR(val) > ins->fmt.height)
			dev_err(sif->dev,"WARNING:Reviced more hsync than expect,check configuration!\n");
	} else if (ins->hsize_err_count_pre == SIF_HSIZE_ERR(val)
		   && ins->vsize_err_count_pre == SIF_VSIZE_ERR(val)) {
		// corner case, drop frame for safety.
		cam_set_frame_status(ins->ctx.buf_ctx, BOTH_ERR);
	}

	ins->hsize_err_count_pre = SIF_HSIZE_ERR(val);
	ins->vsize_err_count_pre = SIF_VSIZE_ERR(val);
	ins->size_err_cnt++;
}

static void sif_handle_pps_irq(struct sif_device *sif, u32 inst)
{
	struct sif_instance *ins = &sif->insts[inst];
	struct sif_frame_des sif_frame = {0};
	u32 pps1_ts_l = 0, pps1_ts_h = 0, pps2_ts_l = 0, pps2_ts_h = 0;

	if (ins->sif_cfg.ts_ctrl.trigger_mode & PPS0_TRIGGER) {
		pps1_ts_h = sif_read(sif, SIF_PPS_TS_HI(0));
		pps1_ts_l = sif_read(sif, SIF_PPS_TS_LO(0));
		dev_dbg(sif->dev, "PPS Timpstamp TRIGGER0:0x%llx\n", ((u64)pps1_ts_h << 32 | pps1_ts_l));
	}
	if (ins->sif_cfg.ts_ctrl.trigger_mode & PPS1_TRIGGER) {
		pps2_ts_h = sif_read(sif, SIF_PPS_TS_HI(1));
		pps2_ts_l = sif_read(sif, SIF_PPS_TS_LO(1));
		dev_dbg(sif->dev, "PPS Timpstamp TRIGGER1:0x%llx\n", ((u64)pps2_ts_h << 32 | pps2_ts_l));
	}
	sif_frame.pps1_ts = pps1_ts_h;
	sif_frame.pps1_ts = (sif_frame.pps1_ts << 32) | pps1_ts_l;
	sif_frame.pps2_ts = pps2_ts_h;
	sif_frame.pps2_ts = (sif_frame.pps2_ts << 32) | pps2_ts_l;
	// TODO:add pps timestamp to vio framework.
}

static inline void sif_handle_frame_start(struct sif_device *sif, u32 inst)
{
	struct sif_instance *ins;
	struct sif_frame_des sif_frame = {0};
	u32 val = 0;
	u32 fs_l = 0, fs_h = 0, trigger_l = 0, trigger_h = 0;
	struct sif_irq_ctx *ctx;
	phys_addr_t p_addr = 0;
	phys_addr_t p_uv_addr = 0;
	unsigned long flags;
	ktime_t now_time = ktime_get_boottime();

	ins = &sif->insts[inst];
	if (ins->last_frame_done)
		ins->frame_interval += ktime_to_ms
				(ktime_sub(now_time, ins->last_frame_done));
	ins->last_frame_done = now_time;
	ins->frame_count++;
	if (ins->sif_cfg.ts_ctrl.time_stamp_en) {
		if (ins->sif_cfg.ts_ctrl.trigger_mode & IPI_VSYNC) {
			fs_l = sif_read(sif, SIF_IPI_TS_VSYNC_LO(inst));
			fs_h = sif_read(sif, SIF_IPI_TS_VSYNC_HI(inst));
			dev_dbg(sif->dev, "IPI Timpstamp VSYNC:0x%llx\n", ((u64)fs_h << 32) | fs_l);
		}
		if (ins->sif_cfg.ts_ctrl.trigger_mode & IPI_TRIGGER) {
			trigger_l = sif_read(sif, SIF_IPI_TS_TRIG_LO(inst));
			trigger_h = sif_read(sif, SIF_IPI_TS_TRIG_HI(inst));
			dev_dbg(sif->dev, "IPI Timpstamp TRIGGER:0x%llx\n", ((u64)trigger_h << 32) | trigger_l);
		}
	}

	if (ins->sif_cfg.f_id_ctrl.frame_id_en) {
		val = sif_read(sif, SIF_IPI_FRAME_ID(inst));
		sif_frame.frame_id = val + ins->frameid_cnt * (FRAME_ID_MAXIMUM + 1);
		if (val == FRAME_ID_MAXIMUM)
			ins->frameid_cnt++;
		dev_dbg(sif->dev, "sif(%d-%d) current frameid: %d, val: %d, cnt: %d\n", sif->id, inst,
			sif_frame.frame_id, val, ins->frameid_cnt);
	}
	sif_frame.trigger_freq = sif->timestamp_clk;
	sif_frame.timestamps = ktime_get_raw_ns();
	sif_frame.trigger_ts = trigger_h;
	sif_frame.trigger_ts = (sif_frame.trigger_ts << 32) | trigger_l;
	sif_frame.fs_ts = fs_h;
	sif_frame.fs_ts = (sif_frame.fs_ts << 32) | fs_l;
	sif_set_frame_des(ins->ctx.sink_ctx, (void *)&sif_frame);
	cam_set_stat_info(ins->ctx.sink_ctx, CAM_STAT_FS);

	spin_lock_irqsave(&ins->lock, flags);
	ctx = &ins->ctx;
	if (ctx->buf_ctx)
		ins->frame_start_cnt++;
	dev_dbg(sif->dev, "sif(%d-%d), FS sink ctx %p buf ctx %p buf %p\n",
		sif->id, inst, ctx->sink_ctx, ctx->buf_ctx, ctx->buf);
	if (ins->state == CAM_STATE_STARTED) {
		if (ctx->next_buf) {
			cam_drop(ctx->buf_ctx);
			ctx->buf = ctx->next_buf;
		}
		if (ctx->buf_ctx)
			ctx->next_buf = cam_dqbuf_irq(ctx->buf_ctx, true);
		if (ctx->next_buf) {
			p_addr = get_phys_addr(ctx->next_buf, 0);
			if (ins->fmt.format == CAM_FMT_NV12 || ins->fmt.format == CAM_FMT_NV16
				|| (sif->ipi_channel_num != 1 && inst == sif->ipi_base))
				p_uv_addr = p_addr + (ins->fmt.stride * ins->fmt.height);
		} else {
			cam_set_frame_status(ctx->buf_ctx, DQ_FAIL);
		}
	}
	spin_unlock_irqrestore(&ins->lock, flags);

	if (p_addr) {
		if (ins->fmt.format == CAM_FMT_NV12 || ins->fmt.format == CAM_FMT_NV16) {
			sif_write(sif, SIF_IPI_BADDR_Y(inst), p_addr);
			if (p_uv_addr)
				sif_write(sif, SIF_IPI_BADDR_UV(inst), p_uv_addr);
		} else if (sif->ipi_channel_num != 1 && inst == sif->ipi_base) {
			sif_write(sif, SIF_IPI_BADDR_Y(inst), p_addr);
			if (p_uv_addr)
				sif_write(sif, SIF_IPI_BADDR_Y(inst + 1), p_uv_addr);
		} else {
			sif_write(sif, SIF_IPI_BADDR_Y(inst), p_addr);
		}
	}
	spin_lock_irqsave(&sif->cfg_reg_lock, flags);
	val = sif_read(sif, SIF_DMA_CTL);
	val |= SIF_DMA_CONFIG_IPI[inst];
	sif_write(sif, SIF_DMA_CTL, val);
	spin_unlock_irqrestore(&sif->cfg_reg_lock, flags);
}

static inline void sif_handle_frame_done(struct sif_device *sif, u32 inst)
{
	unsigned long flags;
	struct sif_irq_ctx *ctx;
	struct sif_instance *ins;
	u8 frame_status = 0;

	ins = &sif->insts[inst];
	spin_lock_irqsave(&ins->lock, flags);
	ctx = &ins->ctx;
	if (ctx->buf_ctx)
		ins->frame_start_cnt--;
	dev_dbg(sif->dev, "sif(%d-%d), FE sink ctx %p buf ctx %p buf %p \n",
		sif->id, inst, ctx->sink_ctx, ctx->buf_ctx, ctx->buf);
	if (ctx->buf) {
		frame_status = cam_get_frame_status(ctx->buf_ctx);
		if (frame_status || (ins->frame_start_cnt != 0)) {
			cam_drop(ctx->buf_ctx);
			cam_dec_frame_status(ctx->buf_ctx);
		} else {
			cam_qbuf_irq(ctx->buf_ctx, ctx->buf, true);
		}
	}

	ctx->buf = ctx->next_buf;
	ctx->next_buf = NULL;
	if (ctx->buf_ctx)
		ins->frame_start_cnt = 0;
	spin_unlock_irqrestore(&ins->lock, flags);
}

static void sif_handle_emb_done(struct sif_device *sif, u32 inst)
{
	unsigned long flags;
	struct sif_irq_ctx *ctx;
	struct sif_instance *ins;
	phys_addr_t p_addr = 0;

	ins = &sif->insts[inst];
	spin_lock_irqsave(&ins->lock, flags);

	ctx = &ins->ctx;
	if (ctx->emb_buf) {
		cam_qbuf_irq(ctx->emb_buf_ctx, ctx->emb_buf, false);
		ctx->emb_buf = NULL;
	}

	if (ins->state == CAM_STATE_STARTED) {
		if (ctx->emb_buf_ctx)
			ctx->emb_buf = cam_dqbuf_irq(ctx->emb_buf_ctx, false);
		if (ctx->emb_buf)
			p_addr = get_phys_addr(ctx->emb_buf, 0);
	}

	spin_unlock_irqrestore(&ins->lock, flags);

	if (p_addr)
		sif_write(sif, SIF_IPI_EBD_BADDR_POST(inst), p_addr);
}

irqreturn_t sif_irq_handler(int irq, void *arg)
{
	struct sif_device *sif = (struct sif_device *)arg;
	u32 i, status, pps_status;

	pr_debug("+\n");
	for (i = 0; i < sif->num_insts; i++) {
		pps_status = sif_read(sif, SIF_PPS_IRQ_STATUS);
		if (pps_status & PPS1_TRIG_IRQ || pps_status & PPS2_TRIG_IRQ) {
			sif_handle_pps_irq(sif, i);
		}
		status = sif_read(sif, SIF_IPI_IRQ_STATUS(i));
		if (!status)
			continue;

		if (status & SIF_FRAME_SIZE_ERROR_EN)
			sif_handle_frame_size_err(sif, i);

		sif_write(sif, SIF_IPI_IRQ_CLR(i), status);

		if (status & (SIF_IRQ_OF | SIF_PIXEL_BUF_AFULL_IRQ_EN))
			cam_set_frame_status(sif->insts[i].ctx.buf_ctx, IPI_OF);

		if (status & SIF_IRQ_FS)
			sif_handle_frame_start(sif, i);
		if (status & SIF_IPI_FRAME_END_EN)
			cam_set_stat_info(sif->insts[i].ctx.sink_ctx, CAM_STAT_FE);

		if (!((status & SIF_IRQ_DONE) || (status & SIF_IRQ_EBD_DMA_DONE)))
			continue;

		if (status & SIF_IRQ_DONE)
			sif_handle_frame_done(sif, i);

		if (status & SIF_IRQ_EBD_DMA_DONE)
			sif_handle_emb_done(sif, i);
	}
	pr_debug("-\n");
	return IRQ_HANDLED;
}
