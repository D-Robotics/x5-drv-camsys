// SPDX-License-Identifier: GPL-2.0-only

#define pr_fmt(fmt) "[isp_drv]: %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/interrupt.h>

#include "cam_buf.h"
#include "isc.h"
#include "isp8000_regs.h"
#include "isp_uapi.h"

#include "isp.h"

static s32 handle_set_fmt_cap(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_instance *ins;
	struct isp_format_cap *cap, *c = NULL;
	u32 i;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];

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

static s32 handle_get_format(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_instance *ins;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];

	memcpy(&msg->fmt, &ins->fmt, sizeof(ins->fmt));
	return 0;
}

static s32 handle_set_state(struct isp_device *isp, struct isp_msg *msg)
{
	return 0;
}

static s32 handle_get_clock(struct isp_device *isp, struct isp_msg *msg)
{
	if (msg->clk.clk == CAM_CORE_CLOCK)
		msg->clk.rate = clk_get_rate(isp->core);
	else if (msg->clk.clk == CAM_AXI_CLOCK)
		msg->clk.rate = clk_get_rate(isp->axi);
	else if (msg->clk.clk == ISP_MCM_CLOCK)
		msg->clk.rate = clk_get_rate(isp->mcm);
	else
		return -EINVAL;
	return 0;
}

static s32 handle_set_clock(struct isp_device *isp, struct isp_msg *msg)
{
	if (msg->clk.clk == CAM_CORE_CLOCK)
		return clk_set_rate(isp->core, msg->clk.rate);
	else if (msg->clk.clk == CAM_AXI_CLOCK)
		return clk_set_rate(isp->axi, msg->clk.rate);
	else if (msg->clk.clk == ISP_MCM_CLOCK)
		return clk_set_rate(isp->mcm, msg->clk.rate);
	else
		return -EINVAL;
}

static s32 handle_reset_control(struct isp_device *isp, struct isp_msg *msg)
{
	isp_reset(isp);
	return 0;
}

static s32 handle_change_input(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_instance *ins;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];

	memset(ins->fmt_cap, 0, sizeof(ins->fmt_cap));

	ins->input_bayer_format = msg->in.sens.bayer_format;
	return 0;
}

static s32 handle_get_func(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_instance *ins;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];

	memset(&msg->func, 0, sizeof(msg->func));
	msg->func.work_mode = isp->mode;
	if (isp->mcm_en) {
		msg->func.mcm.enable = 1;
		if (ins->stream_idx < 0)
			msg->func.mcm.stream_idx = msg->inst;
		else
			msg->func.mcm.stream_idx = ins->stream_idx;
	}
	return 0;
}

static s32 handle_get_vi_info(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_instance *ins;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];

	memset(&msg->func, 0, sizeof(msg->func));
	msg->vinfo.sensor_id = ins->in.index;
	msg->vinfo.hdr_en = ins->hdr_en;
	return 0;
}

s32 isp_msg_handler(void *msg, u32 len, void *arg)
{
	struct isp_device *isp = (struct isp_device *)arg;
	struct isp_msg *m = (struct isp_msg *)msg;
	s32 rc = 0;

	if (!isp || !msg || !len)
		return -EINVAL;

	switch (m->id) {
	case CAM_MSG_READ_REG:
		m->reg.value = isp_read(isp, m->reg.offset);
		break;
	case CAM_MSG_WRITE_REG:
		isp_write(isp, m->reg.offset, m->reg.value);
		break;
	case CAM_MSG_CHANGE_INPUT:
		rc = handle_change_input(isp, m);
		break;
	case CAM_MSG_SET_FMT_CAP:
		rc = handle_set_fmt_cap(isp, m);
		break;
	case CAM_MSG_GET_FORMAT:
		rc = handle_get_format(isp, m);
		break;
	case CAM_MSG_SET_STATE:
		rc = handle_set_state(isp, m);
		break;
	case CAM_MSG_GET_CLOCK:
		rc = handle_get_clock(isp, m);
		break;
	case CAM_MSG_SET_CLOCK:
		rc = handle_set_clock(isp, m);
		break;
	case CAM_MSG_RESET_CONTROL:
		rc = handle_reset_control(isp, m);
		break;
	case ISP_MSG_UNIT_TEST:
		isp->unit_test = (m->unit_test == 1 ? true : false);
		break;
	case ISP_MSG_GET_FUNC:
		rc = handle_get_func(isp, m);
		break;
	case ISP_MSG_GET_VI_INFO:
		rc = handle_get_vi_info(isp, m);
		break;
	default:
		return -EINVAL;
	}
	return rc;
}

static inline void frame_done(struct isp_irq_ctx *ctx)
{
	struct cam_list_node *node;

	if (ctx->sink_buf) {
		cam_qbuf_irq(ctx->sink_ctx, ctx->sink_buf, false);
		ctx->sink_buf = NULL;
	}

	node = list_first_entry_or_null(ctx->src_buf_list3,
					struct cam_list_node, entry);
	if (node) {
		cam_qbuf_irq(ctx->src_ctx, node->data, true);
		list_del(&node->entry);
		list_add_tail(&node->entry, ctx->src_buf_list1);
	}
}

struct isp_irq_ctx *get_next_irq_ctx(struct isp_device *isp)
{
	struct isp_instance *inst;
	struct isp_irq_ctx *ctx = NULL;
	struct irq_job job;
	unsigned long flags;
	u32 id = isp->next_mi_irq_ctx;
	int rc;
	struct ibuf *mcm_ib;

	if (isp->mode == STRM_WORK_MODE) {
		inst = &isp->insts[id];
		if (inst->state != CAM_STATE_STARTED)
			return NULL;
		spin_lock_irqsave(&inst->lock, flags);
		ctx = &inst->ctx;
		rc = new_frame(ctx);
		spin_unlock_irqrestore(&inst->lock, flags);
		if (!rc)
			return ctx;
		else
			return NULL;

	}

	for (;;) {
		rc = pop_job(isp->jq, &job);
		if (rc < 0) {
			ctx = NULL;
			break;
		}

		id = job.irq_ctx_index;
		inst = &isp->insts[id];
		if (inst->state != CAM_STATE_STARTED)
			continue;
		if (isp->mcm_en) {
			mcm_ib = list_first_entry_or_null(&isp->ibm[id].list2,
							  struct ibuf, entry);
			if (!mcm_ib) {
				pr_warn("%s no mcm buf available!\n", __func__);
				continue;
			}
		}
		spin_lock_irqsave(&inst->lock, flags);
		ctx = &inst->ctx;
		rc = new_frame(ctx);
		spin_unlock_irqrestore(&inst->lock, flags);
		if (!rc) {
			isp->next_mi_irq_ctx = job.irq_ctx_index;
			break;
		}
	}
	return ctx;
}

int new_frame(struct isp_irq_ctx *ctx)
{
	struct cam_buf *buf;

	if (ctx->sink_ctx) {
		buf = cam_acqbuf_irq(ctx->sink_ctx);
		if (!buf) {
			ctx->sink_buf = NULL;
			return -ENOMEM;
		}
	}

	if (!ctx->is_src_online_mode && ctx->src_ctx) {
		struct cam_list_node *node;

		buf = cam_dqbuf_irq(ctx->src_ctx, true);
		if (!buf)
			return -ENOMEM;
		node = list_first_entry_or_null(ctx->src_buf_list1,
						struct cam_list_node, entry);
		if (WARN_ON(!node))
			return -ENOMEM;
		node->data = buf;
		list_del(&node->entry);
		list_add_tail(&node->entry, ctx->src_buf_list2);
	}

	if (ctx->sink_ctx)
		ctx->sink_buf = cam_dqbuf_irq(ctx->sink_ctx, false);
	return 0;
}

static inline int handle_mcm(struct isp_device *isp, u32 path)
{
	u32 inst = isp->stream_idx_mapping[path];

	if (unlikely(inst < 0))
		return -1;

	if (!isp->mcm_en)
		return -EFAULT;

#ifndef WITH_LEGACY_ISP
	{
		struct isp_instance *ins;
		struct ibuf *ib = list_first_entry_or_null(&isp->ibm[inst].list1,
							   struct ibuf, entry);

		ins = &isp->insts[inst];
		if (ib) {
			list_add_tail(&ins->mcm_ib->entry, &isp->ibm[inst].list2);
			list_del(&ib->entry);
			ins->mcm_ib = ib;
		} else {
			ib = ins->mcm_ib;
		}

		isp_set_mcm_raw_buffer(isp, path, ib->buf.addr, &ins->fmt.ifmt);
	}
#else
	isp->in_counts[inst]++;
#endif
	isp_add_job(isp, inst);
	return 0;
}

irqreturn_t mi_irq_handler(int irq, void *arg)
{
	struct isp_device *isp = (struct isp_device *)arg;
	struct isp_msg msg;
	struct isp_instance *ins;
	u32 miv2_mis, miv2_mis1, miv2_mis2, miv2_mis3, mi_mis_hdr1;
	struct isp_irq_ctx *ctx;

	pr_debug("+\n");
	miv2_mis = isp_read(isp, MIV2_MIS);
	if (miv2_mis)
		isp_write(isp, MIV2_ICR, miv2_mis);
	miv2_mis1 = isp_read(isp, MIV2_MIS1);
	if (miv2_mis1)
		isp_write(isp, MIV2_ICR1, miv2_mis1);
	miv2_mis2 = isp_read(isp, MIV2_MIS2);
	if (miv2_mis2)
		isp_write(isp, MIV2_ICR2, miv2_mis2);
	miv2_mis3 = isp_read(isp, MIV2_MIS3);
	if (miv2_mis3)
		isp_write(isp, MIV2_ICR3, miv2_mis3);
	mi_mis_hdr1 = isp_read(isp, MI_MIS_HDR1);
	if (mi_mis_hdr1)
		isp_write(isp, MI_ICR_HDR1, mi_mis_hdr1);

	pr_debug("mi mis:0x%x, mis1:0x%x, mis2:0x%x, mis3:0x%x, mis_hdr1:0x%x\n",
				miv2_mis, miv2_mis1, miv2_mis2, miv2_mis3, mi_mis_hdr1);

	if (miv2_mis & MIV2_MIS_MCM_RAW0_FRAME_END_MASK) {
		handle_mcm(isp, 0);
	}
	if (miv2_mis & MIV2_MIS_MCM_RAW1_FRAME_END_MASK) {
		if (!isp->unit_test)
			handle_mcm(isp, 1);
	}
	if (miv2_mis3 & MIV2_MIS3_MCM_G2RAW0_FRAME_END_MASK) {
		if (!isp->unit_test)
			handle_mcm(isp, 2);
	}
	if (miv2_mis3 & MIV2_MIS3_MCM_G2RAW1_FRAME_END_MASK) {
		if (!isp->unit_test)
			handle_mcm(isp, 3);
	}

	// skip buffer management if running unit test!
	if (!isp->unit_test) {
		if (miv2_mis & 0x1) {
			// handle isp frame end intr
			ins = &isp->insts[isp->next_mi_irq_ctx];
			frame_done(&ins->ctx);
			isp->rdma_busy = false;

			if (isp->mcm_en) {
				struct ibuf *ib;

				ib = list_first_entry_or_null
						(&isp->ibm[isp->next_mi_irq_ctx].list3,
						struct ibuf, entry);
				if (ib) {
					list_del(&ib->entry);
					list_add_tail(&ib->entry,
						      &isp->ibm[isp->next_mi_irq_ctx].list1);
				}
			}
		}

		if (!isp->rdma_busy) {
			// handle mcm_wr frame end intr
			u32 old_next_mi_irq_ctx = isp->next_mi_irq_ctx;
			bool is_completed = true;
			struct cam_list_node *node = NULL;

			ctx = get_next_irq_ctx(isp);
			if (!ctx) {
				isp->next_mi_irq_ctx = old_next_mi_irq_ctx;
				isp->error = 1;
				goto _post;
			}

			if (ctx->is_src_online_mode) {
				is_completed = cam_is_completed(ctx->src_ctx);
				pr_debug("isp:%d->%d,vse:%d\n", old_next_mi_irq_ctx,
					 isp->next_mi_irq_ctx, is_completed);
			} else {
				pr_debug("isp:%d->%d\n", old_next_mi_irq_ctx, isp->next_mi_irq_ctx);
			}
			if (!is_completed) {
				isp->next_mi_irq_ctx = old_next_mi_irq_ctx;
				goto _post;
			} else if (ctx->is_src_online_mode) {
				cam_trigger(ctx->src_ctx);
			}

			ins = &isp->insts[isp->next_mi_irq_ctx];
			if (ctx->src_ctx) {
				node = list_first_entry_or_null(ctx->src_buf_list2,
								struct cam_list_node, entry);
				if (node) {
					list_del(&node->entry);
					list_add_tail(&node->entry, ctx->src_buf_list3);
				}
			}
#ifdef WITH_LEGACY_ISP
			if (0)
#else
			if (isp->mode != STRM_WORK_MODE)
#endif
			{
				msg.id = ISP_MSG_MCM_SCH;
				msg.inst = isp->next_mi_irq_ctx;
				msg.sch.id = msg.inst;
				msg.sch.mp_buf.addr = get_phys_addr(node->data, 0);
				msg.sch.mp_buf.size = 0;
			} else if (node) {
				isp_set_mp_buffer(isp, get_phys_addr(node->data, 0), &ins->fmt.ofmt);
			}
			if (isp->mode != STRM_WORK_MODE) {
				if (isp->mcm_en) {
#ifdef WITH_LEGACY_ISP
					u32 count, size;

					count = isp->in_counts[isp->next_mi_irq_ctx] % MCM_BUF_NUM;
					if (count > 0)
						count--;
					else
						count = MCM_BUF_NUM - 1;
					size = ins->fmt.ifmt.stride * ins->fmt.ifmt.height;
					isp_set_rdma_buffer(isp, isp->in_bufs[isp->next_mi_irq_ctx].addr +
								count * size);
#else
					struct ibuf *mcm_ib;

					mcm_ib = list_first_entry_or_null
							(&isp->ibm[isp->next_mi_irq_ctx].list2,
							struct ibuf, entry);
					if (mcm_ib) {
						msg.sch.rdma_buf.addr = mcm_ib->buf.addr;
						msg.sch.rdma_buf.size = mcm_ib->buf.size;
						list_del(&mcm_ib->entry);
						list_add_tail(&mcm_ib->entry,
							      &isp->ibm[isp->next_mi_irq_ctx].list3);
						isp_post(isp, &msg, false);
					}
#endif
				} else {
					msg.sch.rdma_buf.addr = get_phys_addr(ctx->sink_buf, 0);
					msg.sch.rdma_buf.size = 0;
					isp_post(isp, &msg, false);
				}
				isp->error = 0;
			} else if (ctx->sink_buf) {
				isp_set_rdma_buffer(isp, get_phys_addr(ctx->sink_buf, 0));
				isp->error = 0;
			} else if (ctx->sink_ctx) {
				isp->error = 1;
			}
			isp->rdma_busy = true;
		}
	}

_post:
	if (miv2_mis || miv2_mis1 || miv2_mis2 || miv2_mis3 || mi_mis_hdr1) {
		msg.id = ISP_MSG_IRQ_MIS;
		msg.inst = 0;
		msg.irq.num = MI_IRQ_MIS;
		msg.irq.stat.mi_mis.miv2_mis = miv2_mis;
		msg.irq.stat.mi_mis.miv2_mis1 = miv2_mis1;
		msg.irq.stat.mi_mis.miv2_mis2 = miv2_mis2;
		msg.irq.stat.mi_mis.miv2_mis3 = miv2_mis3;
		msg.irq.stat.mi_mis.mi_mis_hdr1 = mi_mis_hdr1;
		isp_post(isp, &msg, false);
	}
	pr_debug("-\n");

	return IRQ_HANDLED;
}

irqreturn_t isp_irq_handler(int irq, void *arg)
{
	struct isp_device *isp = (struct isp_device *)arg;
	struct isp_msg msg = { .id = ISP_MSG_IRQ_MIS };
	struct isp_instance *ins;
	u32 isp_mis = 0;

	pr_debug("+\n");
	isp_mis = isp_read(isp, ISP_MIS);
	pr_debug("isp mis:0x%x\n", isp_mis);
	if (isp_mis) {
		isp_write(isp, ISP_ICR, isp_mis);
		isp_mis &= ~(BIT(26) | BIT(25) | BIT(24) | BIT(23)); /*sensor dataloss*/
		ins = &isp->insts[isp->next_mi_irq_ctx];
		if ((isp_mis & BIT(5)) && ins->ctx.is_sink_online_mode)
			sif_get_frame_des(ins->ctx.src_ctx);
		if (isp_mis & BIT(6)) {
			cam_set_stat_info(ins->ctx.stat_ctx, CAM_STAT_FS);
			isp_mis &= ~BIT(6);
		}
		if (isp_mis & BIT(1))
			cam_set_stat_info(ins->ctx.stat_ctx, CAM_STAT_FE);
		if (isp_mis & (BIT(2) | BIT(3)))
			cam_drop(ins->ctx.stat_ctx);
		if (isp_mis) {
			msg.irq.num = ISP_IRQ_MIS;
			msg.irq.stat.isp_mis = isp_mis;
			isp_post(isp, &msg, false);
		}
	} else {
		return IRQ_NONE;
	}
	pr_debug("-\n");

	return IRQ_HANDLED;
}

irqreturn_t fe_irq_handler(int irq, void *arg)
{
	struct isp_device *isp = (struct isp_device *)arg;
	struct isp_msg msg = { .id = ISP_MSG_IRQ_MIS };
	u32 isp_fe_mis, isp_fe_ctrl;

	isp_fe_mis = isp_read(isp, ISP_FE_MIS);
	pr_debug("+fe_mis:0x%x\n", isp_fe_mis);
	if (isp_fe_mis) {
		isp_write(isp, ISP_FE_ICR, isp_fe_mis);
		isp_fe_ctrl = isp_read(isp, ISP_FE_CTL);
		if ((isp_fe_ctrl & ISP_FE_CFG_SEL_MASK) == ISP_FE_SEL_CMDBUF) {
			isp_fe_ctrl &=
				~(ISP_FE_CFG_SEL_MASK | ISP_FE_AHB_WRITE_MASK);
			isp_fe_ctrl |= (ISP_FE_SEL_AHBBUF)
				       << ISP_FE_CFG_SEL_SHIFT;
			isp_fe_ctrl |= (ISP_FE_AHB_WR_ENABLE)
				       << ISP_FE_AHB_WRITE_SHIFT;
			isp_write(isp, ISP_FE_CTL, isp_fe_ctrl);
		}
		msg.irq.num = FE_IRQ_MIS;
		msg.irq.stat.fe_mis = isp_fe_mis;
		isp_post(isp, &msg, false);
	} else {
		return IRQ_NONE;
	}
	pr_debug("-\n");

	return IRQ_HANDLED;
}
