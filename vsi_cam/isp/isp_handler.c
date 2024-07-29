// SPDX-License-Identifier: GPL-2.0-only

#define pr_fmt(fmt) "[isp_drv]: %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/interrupt.h>

#include "cam_ctx.h"
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

	if (msg->fcap.index >= ARRAY_SIZE(c->res))
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
	if (msg->func.work_mode == ISP_MCM_MODE) {
		msg->func.mcm.online = ins->online_mcm ? 1 : 0;
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

static inline void isp_post_frame_end(struct isp_device *isp, u32 inst)
{
	struct isp_msg msg;
	u32 status;

	if (!isp)
		return;

	status = isp->frame_done_status;

	if (status & ISP_RDMA_END && status & ISP_MP_FRAME_END && status & ISP_MIS_FRAME_END) {
		memset(&msg, 0, sizeof(msg));
		msg.id = ISP_MSG_FRAME_DONE;
		msg.inst = inst;
		isp_post(isp, &msg, false);
		isp->frame_done_status = 0;
		pr_debug("post frame end inst:%d", inst);
	}
}

static s32 handle_get_frame_info(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_instance *ins;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];

	memset(&msg->frame_info, 0, sizeof(msg->frame_info));
	isp_update_frame_info(&msg->frame_info, ins->ctx.src_ctx);

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
	case ISP_MSG_GET_FRAME_INFO:
		rc = handle_get_frame_info(isp, m);
		break;
	default:
		return -EINVAL;
	}
	return rc;
}

static inline void frame_done(struct isp_instance *inst)
{
	struct isp_irq_ctx *ctx = &inst->ctx;
	struct cam_list_node *node;
	ktime_t now_time = ktime_get_boottime();

	if (ctx->sink_buf) {
		cam_qbuf_irq(ctx->sink_ctx, ctx->sink_buf, false);
		ctx->sink_buf = NULL;
	}

	node = list_first_entry_or_null(ctx->src_buf_list3,
					struct cam_list_node, entry);
	if (node) {
		if (cam_get_frame_status(ctx->src_ctx)) {
			cam_drop(ctx->src_ctx);
		} else {
			if (ctx->is_sink_online_mode)
				sif_get_frame_des(ctx->src_ctx);
			cam_qbuf_irq(ctx->src_ctx, node->data, true);
		}
		cam_set_frame_status(ctx->src_ctx, NO_ERR);
		list_del(&node->entry);
		list_add_tail(&node->entry, ctx->src_buf_list1);
	}
	if (inst->last_frame_done)
		inst->frame_interval += ktime_to_ms
				(ktime_sub(now_time, inst->last_frame_done));
	inst->last_frame_done = now_time;
	inst->frame_count++;
}

struct isp_irq_ctx *get_next_irq_ctx(struct isp_device *isp)
{
	struct isp_instance *inst;
	struct isp_irq_ctx *ctx = NULL;
	struct irq_job job;
	unsigned long flags;
	u32 id;
	int rc;
	struct ibuf *mcm_ib;

	if (isp->mode == ISP_STRM_MODE) {
		inst = &isp->insts[0];
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
		rc = pop_job(isp->jq, &job);
		if (rc < 0) {
			ctx = NULL;
			break;
		}
		pr_debug("isp %s: pop_job[%d]\n", __func__, job.irq_ctx_index);
		id = job.irq_ctx_index;
		inst = &isp->insts[id];
		if (inst->state != CAM_STATE_STARTED)
			continue;
		if (inst->online_mcm) {
			mcm_ib = list_first_entry_or_null(&isp->ibm[id].list2,
								struct ibuf, entry);
			if (!mcm_ib) {
				/* pr_warn("%s no mcm buf available! inst:%d\n", __func__, id); */
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

	if (ctx->src_ctx && (!ctx->is_src_online_mode || ctx->ddr_en)) {
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
		pr_debug("isp list_add_tail src_buf_list2\n");
	}

	if (ctx->sink_ctx)
		ctx->sink_buf = cam_dqbuf_irq(ctx->sink_ctx, false);
	return 0;
}

static inline int handle_mcm(struct isp_device *isp, u32 path, bool error)
{
	u32 inst = isp->stream_idx_mapping[path];
	struct isp_instance *ins;

	pr_debug("%s: path[%d], inst:%d\n", __func__, path, inst);
	if (unlikely(inst >= isp->num_insts))
		return -1;

	ins = &isp->insts[inst];
	if (!ins->online_mcm || ins->state != CAM_STATE_STARTED)
		return -EFAULT;

#ifndef WITH_LEGACY_ISP
	{
		struct ibuf *ib = list_first_entry_or_null(&isp->ibm[inst].list1,
							   struct ibuf, entry);
		if (ib) {
			if (!error) {
				list_add_tail(&ins->mcm_ib->entry, &isp->ibm[inst].list2);
			} else {
				list_add_tail(&ins->mcm_ib->entry, &isp->ibm[inst].list1);
				pr_debug("mcm%d frame done but buf full\n", path);
			}
			list_del(&ib->entry);
			ins->mcm_ib = ins->mcm_ib1;
			ins->mcm_ib1 = ib;
		} else {
			ib = ins->mcm_ib1;
		}

		isp_set_mcm_raw_buffer(isp, path, ib->buf.addr, &ins->fmt.ifmt);
	}
#else
	isp->in_counts[inst]++;
#endif
	pr_debug("isp_add_job:%d\n", inst);
	isp_add_job(isp, inst, true);
	return 0;
}

static inline void irq_notify(struct isp_device *isp, u32 inst, struct mi_mis_group *mi_mis)
{
	struct isp_msg msg;
	u32 miv2_mis, miv2_mis1, miv2_mis2, miv2_mis3, mi_mis_hdr1;

	miv2_mis = mi_mis->miv2_mis;
	miv2_mis1 = mi_mis->miv2_mis1;
	miv2_mis2 = mi_mis->miv2_mis2;
	miv2_mis3 = mi_mis->miv2_mis3;
	mi_mis_hdr1 = mi_mis->mi_mis_hdr1;
	if (miv2_mis || miv2_mis1 || miv2_mis2 || miv2_mis3 || mi_mis_hdr1) {
		memset(&msg, 0, sizeof(msg));
		msg.id = ISP_MSG_IRQ_MIS;
		msg.inst = 0;
		msg.irq.num = MI_IRQ_MIS;
		memcpy(&msg.irq.stat.mi_mis, mi_mis, sizeof(msg.irq.stat.mi_mis));
		isp_post(isp, &msg, false);
	}

	if (mi_mis->miv2_mis & BIT(24))
		isp->frame_done_status |= ISP_RDMA_END;

	if (mi_mis->miv2_mis & 0x1)
		isp->frame_done_status |= ISP_MP_FRAME_END;

	if (!isp->unit_test && (mi_mis->miv2_mis & 0x1) && inst != INVALID_MCM_SCH_INST)
		isp_post_frame_end(isp, inst);
}

irqreturn_t mi_irq_handler(int irq, void *arg)
{
	struct isp_device *isp = (struct isp_device *)arg;
	struct isp_mcm_sch sch;
	struct isp_instance *ins;
	struct mi_mis_group mi_mis;
	u32 cur_mi_irq_ctx = INVALID_MCM_SCH_INST;
	struct isp_irq_ctx *ctx;
	int rc;
	u32 ris, isp_ris;

	pr_debug("+\n");
	mi_mis.miv2_mis = isp_read(isp, MIV2_MIS);
	if (mi_mis.miv2_mis)
		isp_write(isp, MIV2_ICR, mi_mis.miv2_mis);
	mi_mis.miv2_mis1 = isp_read(isp, MIV2_MIS1);
	if (mi_mis.miv2_mis1)
		isp_write(isp, MIV2_ICR1, mi_mis.miv2_mis1);
	mi_mis.miv2_mis2 = isp_read(isp, MIV2_MIS2);
	if (mi_mis.miv2_mis2)
		isp_write(isp, MIV2_ICR2, mi_mis.miv2_mis2);
	mi_mis.miv2_mis3 = isp_read(isp, MIV2_MIS3);
	if (mi_mis.miv2_mis3)
		isp_write(isp, MIV2_ICR3, mi_mis.miv2_mis3);
	mi_mis.mi_mis_hdr1 = isp_read(isp, MI_MIS_HDR1);
	if (mi_mis.mi_mis_hdr1)
		isp_write(isp, MI_ICR_HDR1, mi_mis.mi_mis_hdr1);

	pr_debug("mi mis:0x%x, mis1:0x%x, mis2:0x%x, mis3:0x%x, mis_hdr1:0x%x\n",
		 mi_mis.miv2_mis, mi_mis.miv2_mis1, mi_mis.miv2_mis2, mi_mis.miv2_mis3,
		 mi_mis.mi_mis_hdr1);

	// skip buffer management if running unit test!
	if (!isp->unit_test) {
		isp_ris = isp_read(isp, ISP_RIS);
		if (mi_mis.miv2_mis & MIV2_MIS_MCM_RAW0_FRAME_END_MASK) {
			ris = isp_read(isp, MIV2_RIS1);
			if (isp_ris & BIT(26)) {
				if (!(ris & MIV2_MIS_MCM_RAW0_BUF_FULL_MASK)) {
					pr_info("%s sensor0 dataloss but no buf full\n", __func__);
					ris |= MIV2_MIS_MCM_RAW0_BUF_FULL_MASK;
				}
			}
			handle_mcm(isp, 0, ris & MIV2_MIS_MCM_RAW0_BUF_FULL_MASK);
			isp_write(isp, MIV2_ICR1, MIV2_MIS_MCM_RAW0_BUF_FULL_MASK);
		}
		if (mi_mis.miv2_mis & MIV2_MIS_MCM_RAW1_FRAME_END_MASK) {
			ris = isp_read(isp, MIV2_RIS1);
			if (isp_ris & BIT(25)) {
				if (!(ris & MIV2_MIS_MCM_RAW1_BUF_FULL_MASK)) {
					pr_info("%s sensor1 dataloss but no buf full\n", __func__);
					ris |= MIV2_MIS_MCM_RAW1_BUF_FULL_MASK;
				}
			}
			handle_mcm(isp, 1, ris & MIV2_MIS_MCM_RAW1_BUF_FULL_MASK);
			isp_write(isp, MIV2_ICR1, MIV2_MIS_MCM_RAW1_BUF_FULL_MASK);
		}
		if (mi_mis.miv2_mis3 & MIV2_MIS3_MCM_G2RAW0_FRAME_END_MASK) {
			ris = isp_read(isp, MIV2_RIS3);
			if (isp_ris & BIT(24)) {
				if (!(ris & MIV2_MIS3_MCM_G2RAW0_BUF_FULL_MASK)) {
					pr_info("%s sensor2 dataloss but no buf full\n", __func__);
					ris |= MIV2_MIS3_MCM_G2RAW0_BUF_FULL_MASK;
				}
			}
			handle_mcm(isp, 2, ris & MIV2_MIS3_MCM_G2RAW0_BUF_FULL_MASK);
			isp_write(isp, MIV2_ICR3, MIV2_MIS3_MCM_G2RAW0_BUF_FULL_MASK);
		}
		if (mi_mis.miv2_mis3 & MIV2_MIS3_MCM_G2RAW1_FRAME_END_MASK) {
			ris = isp_read(isp, MIV2_RIS3);
			if (isp_ris & BIT(23)) {
				if (!(ris & MIV2_MIS3_MCM_G2RAW1_BUF_FULL_MASK)) {
					pr_info("%s sensor3 dataloss but no buf full\n", __func__);
					ris |= MIV2_MIS3_MCM_G2RAW1_BUF_FULL_MASK;
				}
			}
			handle_mcm(isp, 3, ris & MIV2_MIS3_MCM_G2RAW1_BUF_FULL_MASK);
			isp_write(isp, MIV2_ICR3, MIV2_MIS3_MCM_G2RAW1_BUF_FULL_MASK);
		}

		if (mi_mis.miv2_mis & 0x1) {
			rc = isp_get_schedule(isp, &cur_mi_irq_ctx);
			if (rc) {
				pr_err("fail to get currect isp instance id!\n");
			} else {
				pr_debug("isp:%d, mi frame done!\n", cur_mi_irq_ctx);
				isp->cur_mi_irq_ctx = cur_mi_irq_ctx;
				// handle isp frame end intr
				ins = &isp->insts[isp->cur_mi_irq_ctx];
				if (ins->state == CAM_STATE_STARTED) {
					frame_done(ins);
					if (ins->online_mcm) {
						struct ibuf *ib;

						ib = list_first_entry_or_null
								(&isp->ibm[isp->cur_mi_irq_ctx].list3,
								struct ibuf, entry);
						if (ib) {
							list_del(&ib->entry);
							list_add_tail(&ib->entry,
								&isp->ibm[isp->cur_mi_irq_ctx].list1);
						}
					}
				}
			}
		}

		if (!isp->rdma_busy) {
			// handle mcm_wr frame end intr
			struct cam_list_node *node = NULL;

			ctx = get_next_irq_ctx(isp);
			if (!ctx)
				goto _post;

#ifdef WITH_LEGACY_ISP
			{
				bool is_completed = true;

				if (ctx->is_src_online_mode) {
					is_completed = cam_is_completed(ctx->src_ctx);
					pr_debug("isp:%d->%d,vse:%d\n", isp->cur_mi_irq_ctx,
						 isp->next_mi_irq_ctx, is_completed);
				} else {
					pr_debug("isp:%d->%d\n", isp->cur_mi_irq_ctx, isp->next_mi_irq_ctx);
				}

				if (!is_completed)
					goto _post;
				else if (ctx->is_src_online_mode)
					cam_trigger(ctx->src_ctx);
			}
#endif

			ins = &isp->insts[isp->next_mi_irq_ctx];
			if (isp->mode == ISP_MCM_MODE && !ins->online_mcm) {
				irq_notify(isp, cur_mi_irq_ctx, &mi_mis);
				isp_set_schedule_offline(isp, isp->next_mi_irq_ctx, true);
				goto _exit;
			}
			if (ctx->src_ctx && (!ctx->is_src_online_mode || ctx->ddr_en)) {
				node = list_first_entry_or_null(ctx->src_buf_list2,
								struct cam_list_node, entry);
				if (node) {
					list_del(&node->entry);
					list_add_tail(&node->entry, ctx->src_buf_list3);
					pr_debug("isp list_add_tail src_buf_list3\n");
				}
			}
#ifdef WITH_LEGACY_ISP
			if (0)
#else
			if (node)
#endif
			{
				if (isp->mode != ISP_STRM_MODE) {
					sch.id = isp->next_mi_irq_ctx;
					sch.mp_buf.addr = get_phys_addr(node->data, 0);
					sch.mp_buf.size = 0;
				} else {
					isp_set_mp_buffer(isp, get_phys_addr(node->data, 0), &ins->fmt.ofmt);
					irq_notify(isp, cur_mi_irq_ctx, &mi_mis);
					isp_set_schedule(isp, &sch, false);
					goto _exit;
				}
			} else if (ctx->is_src_online_mode) {
				if (isp->mode != ISP_STRM_MODE) {
					sch.id = isp->next_mi_irq_ctx;
					sch.mp_buf.addr = 0;
					sch.mp_buf.size = 0;
				} else {
					isp_set_mp_buffer(isp, 0, &ins->fmt.ofmt);
					irq_notify(isp, cur_mi_irq_ctx, &mi_mis);
					isp_set_schedule(isp, &sch, false);
					goto _exit;
				}
			} else {
				pr_err("fail to configure mp buffer!\n");
				goto _post;
			}
			if (isp->mode != ISP_STRM_MODE) {
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
					sch.rdma_buf.addr = mcm_ib->buf.addr;
					sch.rdma_buf.size = mcm_ib->buf.size;
					list_del(&mcm_ib->entry);
					list_add_tail(&mcm_ib->entry,
							&isp->ibm[isp->next_mi_irq_ctx].list3);
					irq_notify(isp, cur_mi_irq_ctx, &mi_mis);
					isp_set_schedule(isp, &sch, true);
					goto _exit;
				}
#endif
			} else if (ctx->sink_buf) {
				isp_set_rdma_buffer(isp, get_phys_addr(ctx->sink_buf, 0));
				irq_notify(isp, cur_mi_irq_ctx, &mi_mis);
				isp_set_schedule(isp, &sch, false);
				goto _exit;
			} else if (ctx->sink_ctx) {
				// TODO:
				goto _post;
			}
		}
	}

_post:
	irq_notify(isp, cur_mi_irq_ctx, &mi_mis);

_exit:
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
		if (isp_mis & BIT(6)) {
			cam_set_stat_info(ins->ctx.stat_ctx, CAM_STAT_FS);
			isp_mis &= ~BIT(6);
		}
		if (isp_mis & BIT(1)) {
			isp->frame_done_status |= ISP_MIS_FRAME_END;
			cam_set_stat_info(ins->ctx.stat_ctx, CAM_STAT_FE);
		}
		if (isp_mis & (BIT(2) | BIT(3))) {
			if (isp_mis & BIT(3))
				cam_set_frame_status(ins->ctx.src_ctx, VSIZE_ERR);
		}
		if (isp_mis) {
			msg.irq.num = ISP_IRQ_MIS;
			msg.irq.stat.isp_mis = isp_mis;
			isp_post(isp, &msg, false);
			/**
			 * if MI frame done is not ready, frame done message will not be sent to
			 * cam_service. in this case, there is no negative impact to call
			 * isp_post_frame_end() with incorrect instance isp id
			 **/
			if (isp_mis & BIT(1))
				isp_post_frame_end(isp, isp->cur_mi_irq_ctx);
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
	u32 miv2_ctrl;

	isp_fe_mis = isp_read(isp, ISP_FE_MIS);
	pr_debug("+fe_mis:0x%x\n", isp_fe_mis);
	if (isp_fe_mis) {
		isp_write(isp, ISP_FE_ICR, isp_fe_mis);
		isp_fe_ctrl = isp_read(isp, ISP_FE_CTL);
		miv2_ctrl = isp_read(isp, MIV2_CTRL);
		miv2_ctrl |= MIV2_CTRL_MCM_RAW_RDMA_START_MASK;
		isp_write(isp, MIV2_CTRL, miv2_ctrl);
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
