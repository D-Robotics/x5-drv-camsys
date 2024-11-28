// SPDX-License-Identifier: GPL-2.0-only

#define pr_fmt(fmt) "[isp_drv]: %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/interrupt.h>

#include "cam_ctx.h"
#include "isc.h"
#include "isp8000_regs.h"
#include "isp_uapi.h"

#include "isp.h"

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

static s32 handle_get_func(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_instance *ins;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];

	memset(&msg->func, 0, sizeof(msg->func));
	msg->func.work_mode = isp->mode;
	msg->func.tile_en = ins->tile_en;
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

static s32 handle_get_frame_info(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_instance *ins;
	u32 i;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];

	memset(&msg->frame_info, 0, sizeof(msg->frame_info));

	if (has_offline(ins->ctx.src_online_stat)) {
		i = get_offline(ins->ctx.src_online_stat);
		isp_update_frame_info(&msg->frame_info, ins->ctx.src_ctx[i]);
	}
	return 0;
}

static s32 handle_set_gamma_febe(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_instance *ins;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	if (!ins->febe_ctrl.flag) {
		memcpy(&ins->febe_ctrl, &msg->febe_ctrl, sizeof(msg->febe_ctrl));
		ins->febe_ctrl.flag = true;
	}

	return 0;
}

static s32 handle_set_sensor_ctrl(struct isp_device *isp, struct isp_msg *msg)
{
	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	isp_handle_set_sensor_ctrl((void *)isp, msg->inst, (void *)&msg->sen_ctrl);
	return 0;
}

static s32 handle_get_sensor_ctrl(struct isp_device *isp, struct isp_msg *msg)
{
	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	isp_handle_get_sensor_ctrl((void *)isp, msg->inst, (void *)&msg->sen_ctrl);
	return 0;
}

static s32 handle_get_metadata(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_instance *ins, *ins_meta;
	struct cam_buf *buf;
	struct isp_irq_ctx *ctx;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	ins_meta = &isp->insts[ins->meta_inst];
	ctx = &ins_meta->ctx;

	if (ctx->sink_buf) {
		cam_qbuf_irq(ctx->sink_ctx, ctx->sink_buf, false);
		ctx->sink_buf = NULL;
	}
	buf = cam_dqbuf_irq(ctx->sink_ctx, false);
	if (!buf)
		return -ENOMEM;
	ctx->sink_buf = buf;
	msg->meta.buf.addr = get_phys_addr(NULL, buf, 0);
	msg->meta.buf.size = get_buf_size(buf, 0);
	pr_debug("%s buf_addr: 0x%x, buff_size:%d\n", __func__,
		 (u32)msg->meta.buf.addr, (u32)msg->meta.buf.size);
	return 0;
}

static s32 handle_query_metadata(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_instance *ins;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	msg->meta_enabled = ins->meta_inst < isp->num_insts ? 1 : 0;

	return 0;
}

static s32 handle_isp_reset_schedule(struct isp_device *isp, struct isp_msg *msg)
{
	isp_reset_schedule(isp, INVALID_INST, true);
	// isp_reset_schedule(isp, msg->inst, false);
	return 0;
}

static s32 handle_iommu_map(struct isp_device *isp, struct isp_msg *msg)
{
	return mem_iommu_map(isp->dev, msg->map_buf.phys,
			     msg->map_buf.size, &msg->map_buf.iova);
}

static s32 handle_iommu_unmap(struct isp_device *isp, struct isp_msg *msg)
{
	return mem_iommu_unmap(isp->dev, msg->map_buf.iova,
			       msg->map_buf.size);
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
	case ISP_MSG_SET_GAMMA_FE_BE:
		rc = handle_set_gamma_febe(isp, m);
		break;
	case CAM_MSG_SET_SEN_CTRL:
		rc = handle_set_sensor_ctrl(isp, m);
		break;
	case CAM_MSG_GET_SEN_CTRL:
		rc = handle_get_sensor_ctrl(isp, m);
		break;
	case ISP_MSG_GET_METADATA:
		rc = handle_get_metadata(isp, m);
		break;
	case ISP_MSG_QRY_METADATA:
		rc = handle_query_metadata(isp, m);
		break;
	case ISP_MSG_RESET_SCH:
		rc = handle_isp_reset_schedule(isp, m);
		break;
	case CAM_MSG_IOMMU_MAP:
		rc = handle_iommu_map(isp, m);
		break;
	case CAM_MSG_IOMMU_UNMAP:
		rc = handle_iommu_unmap(isp, m);
		break;
	default:
		return -EINVAL;
	}
	return rc;
}

void frame_done(struct isp_device *isp, struct isp_instance *inst, bool timeout)
{
	struct isp_irq_ctx *ctx = &inst->ctx;
	struct cam_list_node *node;
	ktime_t now_time = ktime_get_boottime();
	struct cam_frame_info *info = NULL;
	struct ibuf *ib = NULL;
	struct cam_ctx *src_ctx = NULL;
	u32 i;

	if (inst->online_mcm) {
		ib = list_first_entry_or_null(&isp->ibm[isp->cur_mi_irq_ctx].list3, struct ibuf,
				      entry);
		if (ib) {
			info = &ib->info;
			list_del(&ib->entry);
			list_add_tail(&ib->entry, &isp->ibm[isp->cur_mi_irq_ctx].list1);
		}
	}

	if (ctx->sink_buf) {
		cam_qbuf_irq(ctx->sink_ctx, ctx->sink_buf, false);
		ctx->sink_buf = NULL;
	}

	node = list_first_entry_or_null(ctx->src_buf_list3,
					struct cam_list_node, entry);
	if (node) {
		if (has_offline(ctx->src_online_stat)) {
			i = get_offline(ctx->src_online_stat);
			src_ctx = ctx->src_ctx[i];
		}

		if (cam_get_frame_status(src_ctx) || timeout) {
			cam_drop_irq(src_ctx, node->data);
		} else {
			if (ctx->sink_online_en) {
				if (isp->mode == ISP_STRM_MODE)
					sif_get_frame_des(src_ctx);
				else
					cam_update_frame_info(src_ctx, info);
			}
			cam_qbuf_irq(src_ctx, node->data, true);
		}
		cam_set_frame_status(src_ctx, NO_ERR);
		list_del(&node->entry);
		list_add_tail(&node->entry, ctx->src_buf_list1);
	}

	if (inst->last_frame_done)
		inst->frame_interval += ktime_to_ms(ktime_sub(now_time, inst->last_frame_done));

	inst->last_frame_done = now_time;
	inst->frame_count++;
}

struct isp_irq_ctx *get_next_irq_ctx(struct isp_device *isp)
{
	struct isp_instance *inst;
	struct isp_irq_ctx *ctx = NULL;
	unsigned long flags;
	u32 id;
	int rc;
	struct ibuf *mcm_ib;

	for (;;) {
		rc = isp_fetch_job(isp, &id);
		if (rc < 0 || id == INVALID_INST) {
			ctx = NULL;
			break;
		}
		pr_debug("isp %s: pop_job[%d]\n", __func__, id);
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
			isp->next_mi_irq_ctx = id;
			break;
		}
	}
	return ctx;
}

int new_frame(struct isp_irq_ctx *ctx)
{
	struct cam_buf *buf;

	if (ctx->sink_ctx) {
		buf = cam_acqbuf_irq(ctx->sink_ctx, false);
		if (!buf) {
			ctx->sink_buf = NULL;
			return -ENOMEM;
		}
	}

	if (has_offline(ctx->src_online_stat)) {
		struct cam_list_node *node;
		u32 i = get_offline(ctx->src_online_stat);

		buf = cam_dqbuf_irq(ctx->src_ctx[i], true);
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

int handle_mcm(struct isp_device *isp, u32 path, bool error)
{
	int inst = isp->stream_idx_mapping[path];
	struct isp_instance *ins;
	struct ibuf *ib;
	struct cam_frame_info *info;

	pr_debug("%s: path[%d], inst:%d\n", __func__, path, inst);
	if (inst >= isp->num_insts || inst < 0)
		return -1;

	ins = &isp->insts[inst];
	if (!ins->online_mcm || ins->state != CAM_STATE_STARTED)
		return -EFAULT;

	ib = list_first_entry_or_null(&isp->ibm[inst].list1,
				      struct ibuf, entry);
	if (ib) {
		if (!error) {
			info = &ins->mcm_ib->info;
			cam_get_frame_info(ins->prev, info);
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

	isp_set_mcm_buffer(isp, path, ib->buf.addr);

	pr_debug("isp_add_job:%d\n", inst);
	isp_add_job(isp, inst);
	return 0;
}

static inline void irq_notify(struct isp_device *isp, struct mi_mis_group *mi_mis,
			      u32 isp_mis)
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
	// for MI bus timeout
	if (isp_mis) {
		memset(&msg, 0, sizeof(msg));
		msg.id = ISP_MSG_IRQ_MIS;
		msg.inst = 0;
		msg.irq.num = ISP_IRQ_MIS;
		msg.irq.stat.isp_mis = isp_mis;
		isp_post(isp, &msg, false);
	}
}

irqreturn_t mi_irq_handler(int irq, void *arg)
{
	struct isp_device *isp = (struct isp_device *)arg;
	struct isp_mcm_sch sch;
	struct mi_mis_group mi_mis;
	u32 isp_mis = 0, value;

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
		isp_add_schedule(isp, &mi_mis);
		if (mi_mis.miv2_mis1 & 0x1) {
			/** If mp bus timeout occurs, we report an artificial
			 *  interrupt status and drop the current frame data.
			 */
			mi_mis.miv2_mis = 0x1800025;
			isp_mis = 0x8002;
			pr_info("mp bus timed-out!\n");
		}
		if (mi_mis.miv2_mis & 0x1) {
			value = isp_read(isp, MI_MP_BUS_TIMEO);
			value |= 0x1;
			isp_write(isp, MI_MP_BUS_TIMEO, value);
			value = isp_read(isp, MIV2_IMSC1);
			value &= ~0x1;
			isp_write(isp, MIV2_IMSC1, value);
			isp_get_schedule(isp, &mi_mis);
		}
	}

	irq_notify(isp, &mi_mis, isp_mis);
	if (!isp->unit_test) {
		if (isp->mode == ISP_STRM_MODE) {
			if (isp->sch.mi_idle)
				isp_set_schedule(isp, &sch, 0, 0, true);
		} else {
			if (isp->sch.mi_idle || mi_mis.miv2_mis || isp_mis)
				isp_set_schedule(isp, &sch, mi_mis.miv2_mis, isp_mis, true);
		}
	}

	pr_debug("-\n");
	return IRQ_HANDLED;
}

irqreturn_t isp_irq_handler(int irq, void *arg)
{
	struct isp_device *isp = (struct isp_device *)arg;
	struct isp_msg msg = { .id = ISP_MSG_IRQ_MIS };
	struct isp_instance *ins;
	struct isp_mcm_sch sch;
	u32 isp_mis = 0, i;

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
		if (isp_mis & BIT(1))
			cam_set_stat_info(ins->ctx.stat_ctx, CAM_STAT_FE);
		if (isp_mis & (BIT(2) | BIT(3))) {
			if (isp_mis & BIT(3) && has_offline(ins->ctx.src_online_stat)) {
				i = get_offline(ins->ctx.src_online_stat);
				cam_set_frame_status(ins->ctx.src_ctx[i], VSIZE_ERR);
			}
		}
		if (isp_mis) {
			msg.irq.num = ISP_IRQ_MIS;
			msg.irq.stat.isp_mis = isp_mis;
			isp_post(isp, &msg, false);
			if (isp_mis & BIT(1)) {
				if (isp->mode != ISP_STRM_MODE) {
					isp_set_schedule(isp, &sch, 0, isp_mis, true);
				} else if (ins->febe_ctrl.flag) {
					pr_debug("config febe\n");
					isp_write(isp, ISP_GAMMA_FE_Y_ADDR, 0);
					isp_write(isp, ISP_GAMMA_BE_Y_ADDR, 0);
					for (i = 0; i < ISP_CTRL_FEBE_NUM; i++) {
						if (i % 20 == 0)
							pr_debug("%s i %d  %d %d\n", __func__, i,
								 ins->febe_ctrl.compress[i],
								 ins->febe_ctrl.expand[i]);
						isp_write(isp, ISP_GAMMA_FE_Y_WRITE_DATA,
							  ins->febe_ctrl.compress[i]);
						isp_write(isp, ISP_GAMMA_BE_Y_WRITE_DATA,
							  ins->febe_ctrl.expand[i]);
					}
					ins->febe_ctrl.flag = false;
				}
			}
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
	u32 isp_fe_mis;
	u32 miv2_ctrl;

	isp_fe_mis = isp_read(isp, ISP_FE_MIS);
	pr_debug("+fe_mis:0x%x\n", isp_fe_mis);
	if (isp_fe_mis) {
		isp_write(isp, ISP_FE_ICR, isp_fe_mis);
		miv2_ctrl = isp_read(isp, MIV2_CTRL);
		miv2_ctrl |= MIV2_CTRL_MCM_RAW_RDMA_START_MASK;
		isp_write(isp, MIV2_CTRL, miv2_ctrl);
		msg.irq.num = FE_IRQ_MIS;
		msg.irq.stat.fe_mis = isp_fe_mis;
		isp_post(isp, &msg, false);
	} else {
		return IRQ_NONE;
	}
	pr_debug("-\n");

	return IRQ_HANDLED;
}
