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
	msg->func.hdr_sram = isp_get_hdr_sram_enabled(isp, msg->inst);
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

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];

	memset(&msg->frame_info, 0, sizeof(msg->frame_info));

	isp_update_frame_info(&msg->frame_info, ins->ctx.info_ctx);
	return 0;
}

static s32 handle_set_gamma_febe(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_instance *ins;
	struct isp_gamma_febe_ctrl *febe_ctrl;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	febe_ctrl = &ins->febe_ctrl;
	if (!febe_ctrl->flag) {
		memcpy(febe_ctrl, &msg->febe_ctrl, sizeof(msg->febe_ctrl));
		febe_ctrl->flag = true;
	}
	return 0;
}

static s32 handle_set_rgbgamma(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_ctrl_ext *ctrl_ext = &msg->ctrl_ext;
	struct isp_instance *ins;
	struct isp_rgbgamma_data *rgbgamma_data;
	void *pdata;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	rgbgamma_data = &ins->rgbgamma_data;
	if (!rgbgamma_data->flag) {
		pr_debug("rgb gamma data size %lld %d\n",
			 ctrl_ext->buf.size, ctrl_ext->size);
		pdata = isc_get_extra_data(isp->isc, &ctrl_ext->buf);
		memcpy(rgbgamma_data, pdata, ctrl_ext->size);
		rgbgamma_data->flag = true;
		isc_put_extra_data(isp->isc, &ctrl_ext->buf);
	}
	return 0;
}

static s32 handle_set_wdr5_hist(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_ctrl_ext *ctrl_ext = &msg->ctrl_ext;
	struct isp_instance *ins;
	struct isp_wdr5_data *wdr5_data;
	void *pdata;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	wdr5_data = &ins->wdr5_data;
	if (!wdr5_data->histogram_w_data_changed) {
		pr_debug("wdr data size %lld %d\n",
			 ctrl_ext->buf.size, ctrl_ext->size);
		pdata = isc_get_extra_data(isp->isc, &ctrl_ext->buf);
		memcpy(wdr5_data->lut_histogram_write_data, pdata, ctrl_ext->size);
		wdr5_data->histogram_w_data_changed = true;
		isc_put_extra_data(isp->isc, &ctrl_ext->buf);
	}
	return 0;
}

static s32 handle_set_wdr5_shift(struct isp_device *isp, struct isp_msg *msg)
{

	struct isp_ctrl_ext *ctrl_ext = &msg->ctrl_ext;
	struct isp_instance *ins;
	struct isp_wdr5_data *wdr5_data;
	void *pdata;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	wdr5_data = &ins->wdr5_data;
	if (!wdr5_data->lut_shift_w_data_changed) {
		pdata = isc_get_extra_data(isp->isc, &ctrl_ext->buf);
		memcpy(wdr5_data->lut_shift_write_data, pdata, ctrl_ext->size);
		wdr5_data->lut_shift_w_data_changed = true;
		isc_put_extra_data(isp->isc, &ctrl_ext->buf);
	}
	return 0;
}

static s32 handle_set_wdr5_shift0(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_ctrl_ext *ctrl_ext = &msg->ctrl_ext;
	struct isp_instance *ins;
	struct isp_wdr5_data *wdr5_data;
	void *pdata;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	wdr5_data = &ins->wdr5_data;
	if (!wdr5_data->lut_shift0_w_data_changed) {
		pdata = isc_get_extra_data(isp->isc, &ctrl_ext->buf);
		memcpy(wdr5_data->lut_shift0_write_data, pdata, ctrl_ext->size);
		wdr5_data->lut_shift0_w_data_changed = true;
		isc_put_extra_data(isp->isc, &ctrl_ext->buf);
	}
	return 0;
}

static s32 handle_set_wdr5_gammapre(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_ctrl_ext *ctrl_ext = &msg->ctrl_ext;
	struct isp_instance *ins;
	struct isp_wdr5_data *wdr5_data;
	void *pdata;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	wdr5_data = &ins->wdr5_data;
	if (!wdr5_data->gammapre_w_data_changed) {
		pdata = isc_get_extra_data(isp->isc, &ctrl_ext->buf);
		memcpy(wdr5_data->lut_gammapre_write_data, pdata, ctrl_ext->size);
		wdr5_data->gammapre_w_data_changed = true;
		isc_put_extra_data(isp->isc, &ctrl_ext->buf);
	}
	return 0;
}

static s32 handle_set_wdr5_gammadown(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_ctrl_ext *ctrl_ext = &msg->ctrl_ext;
	struct isp_instance *ins;
	struct isp_wdr5_data *wdr5_data;
	void *pdata;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	wdr5_data = &ins->wdr5_data;
	if (!wdr5_data->gammadown_w_data_changed) {
		pdata = isc_get_extra_data(isp->isc, &ctrl_ext->buf);
		memcpy(wdr5_data->lut_gammadown_write_data, pdata, ctrl_ext->size);
		wdr5_data->gammadown_w_data_changed = true;
		isc_put_extra_data(isp->isc, &ctrl_ext->buf);
	}
	return 0;
}

static s32 handle_set_wdr5_entropy(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_ctrl_ext *ctrl_ext = &msg->ctrl_ext;
	struct isp_instance *ins;
	struct isp_wdr5_data *wdr5_data;
	void *pdata;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	wdr5_data = &ins->wdr5_data;
	if (!wdr5_data->entropy_w_data_changed) {
		pdata = isc_get_extra_data(isp->isc, &ctrl_ext->buf);
		memcpy(wdr5_data->lut_entropy_write_data, pdata, ctrl_ext->size);
		wdr5_data->entropy_w_data_changed = true;
		isc_put_extra_data(isp->isc, &ctrl_ext->buf);
	}
	return 0;
}

static s32 handle_set_wdr5_distance(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_ctrl_ext *ctrl_ext = &msg->ctrl_ext;
	struct isp_instance *ins;
	struct isp_wdr5_data *wdr5_data;
	void *pdata;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	wdr5_data = &ins->wdr5_data;
	if (!wdr5_data->lut_distance_weight_w_data_changed) {
		pdata = isc_get_extra_data(isp->isc, &ctrl_ext->buf);
		memcpy(wdr5_data->lut_distance_weight_write_data, pdata, ctrl_ext->size);
		wdr5_data->lut_distance_weight_w_data_changed = true;
		isc_put_extra_data(isp->isc, &ctrl_ext->buf);
	}
	return 0;
}

static s32 handle_set_wdr5_difference(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_ctrl_ext *ctrl_ext = &msg->ctrl_ext;
	struct isp_instance *ins;
	struct isp_wdr5_data *wdr5_data;
	void *pdata;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	wdr5_data = &ins->wdr5_data;
	if (!wdr5_data->difference_weight_w_data_changed) {
		pdata = isc_get_extra_data(isp->isc, &ctrl_ext->buf);
		memcpy(wdr5_data->lut_difference_weight_write_data, pdata, ctrl_ext->size);
		wdr5_data->difference_weight_w_data_changed = true;
		isc_put_extra_data(isp->isc, &ctrl_ext->buf);
	}
	return 0;
}

static s32 handle_set_wdr5_factor(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_ctrl_ext *ctrl_ext = &msg->ctrl_ext;
	struct isp_instance *ins;
	struct isp_wdr5_data *wdr5_data;
	void *pdata;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	wdr5_data = &ins->wdr5_data;
	if (!wdr5_data->flat_factor_w_data_changed) {
		pdata = isc_get_extra_data(isp->isc, &ctrl_ext->buf);
		memcpy(wdr5_data->lut_flat_factor_write_data, pdata, ctrl_ext->size);
		wdr5_data->flat_factor_w_data_changed = true;
		isc_put_extra_data(isp->isc, &ctrl_ext->buf);
	}
	return 0;
}

static s32 handle_set_wdr5_level(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_ctrl_ext *ctrl_ext = &msg->ctrl_ext;
	struct isp_instance *ins;
	struct isp_wdr5_data *wdr5_data;
	void *pdata;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	wdr5_data = &ins->wdr5_data;
	if (!wdr5_data->flat_level_w_data_changed) {
		pdata = isc_get_extra_data(isp->isc, &ctrl_ext->buf);
		memcpy(wdr5_data->lut_flat_level_write_data, pdata, ctrl_ext->size);
		wdr5_data->flat_level_w_data_changed = true;
		isc_put_extra_data(isp->isc, &ctrl_ext->buf);
	}
	return 0;
}

static s32 handle_set_wdr5_sat_shift(struct isp_device *isp, struct isp_msg *msg)
{
	struct isp_ctrl_ext *ctrl_ext = &msg->ctrl_ext;
	struct isp_instance *ins;
	struct isp_wdr5_data *wdr5_data;
	void *pdata;

	if (msg->inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[msg->inst];
	wdr5_data = &ins->wdr5_data;
	if (!wdr5_data->sat_shift_w_data_changed) {
		pdata = isc_get_extra_data(isp->isc, &ctrl_ext->buf);
		memcpy(wdr5_data->lut_sat_shift_write_data, pdata, ctrl_ext->size);
		wdr5_data->sat_shift_w_data_changed = true;
		isc_put_extra_data(isp->isc, &ctrl_ext->buf);
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
	struct isp_instance *ins;
	struct cam_buf *buf;
	struct isp_irq_ctx *ctx;

	ins = &isp->insts[msg->inst];
	ctx = &ins->ctx;

	if (ctx->pd_buf) {
		cam_qbuf_irq(ctx->pd_ctx, ctx->pd_buf, false);
		ctx->pd_buf = NULL;
	}

	buf = cam_dqbuf_irq(ctx->pd_ctx, false);
	if (!buf)
		return -ENOMEM;
	ctx->pd_buf = buf;
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

	if (ins->af_mode)
		msg->meta_enabled = true;

	return 0;
}

static s32 handle_isp_reset_schedule(struct isp_device *isp, struct isp_msg *msg)
{
	isp_reset_schedule(isp, INVALID_INST, true);
	// isp_reset_schedule(isp, msg->inst, false);
	return 0;
}

static s32 handle_ack_ctrl(struct isp_device *isp, struct isp_msg *msg)
{
	u64 curr, cond = 0;

	while (true) {
		curr = get_ctrl_timestamp(isp);
		if (curr - msg->ctrl.timestamp >=
		    ISP_CTRL_WAIT_TIME_MS * 1000000llu) {
			pr_warn("isp ctrl cmd: 0x%x timedout\n", msg->ctrl.ctrl_id);
			return -ETIMEDOUT;
		}
		mutex_lock(&isp->ctrl_lock);
		if (isp->ctrl_msg.type == ISP_CTRL_MSG) {
			if (curr - isp->ctrl_msg.ctrl.timestamp >=
			    ISP_CTRL_WAIT_TIME_MS * 1000000llu) {
				pr_warn("isp ctrl cmd: 0x%x timedout, cleared\n",
					isp->ctrl_msg.ctrl.ctrl_id);
				memset(&isp->ctrl_msg, 0, sizeof(isp->ctrl_msg));
				break;
			}
		} else if (!isp->ctrl_msg.type) {
			break;
		}
		cond = !cond ? isp->ctrl_msg_wait_cond + 1 : cond + 1;
		mutex_unlock(&isp->ctrl_lock);
		wait_event_timeout(isp->ctrl_msg_waitq,
				   cond == isp->ctrl_msg_wait_cond,
				   msecs_to_jiffies(ISP_CTRL_WAIT_TIME_MS));
		if (isp->ctrl_exit)
			return -EINTR;
	}
	isp->ctrl_msg.rc = (int)msg->group;
	memcpy(&isp->ctrl_msg.ctrl, &msg->ctrl, sizeof(msg->ctrl));
	isp->ctrl_msg.type = ISP_CTRL_MSG;
	wake_up_all(&isp->ctrl_ack_waitq);
	mutex_unlock(&isp->ctrl_lock);
	return 0;
}

static s32 handle_ack_ctrl_ext(struct isp_device *isp, struct isp_msg *msg)
{
	u64 curr, cond = 0;

	while (true) {
		curr = get_ctrl_timestamp(isp);
		if (curr - msg->ctrl_ext.timestamp >=
		    ISP_CTRL_WAIT_TIME_MS * 1000000llu) {
			pr_warn("isp ctrl ext cmd: 0x%x timedout\n",
				msg->ctrl_ext.ctrl_id);
			return -ETIMEDOUT;
		}
		mutex_lock(&isp->ctrl_lock);
		if (isp->ctrl_msg.type == ISP_CTRL_EXT_MSG) {
			if (curr - isp->ctrl_msg.ctrl_ext.timestamp >=
			    ISP_CTRL_WAIT_TIME_MS * 1000000llu) {
				pr_warn("isp ctrl ext cmd: 0x%x timedout, cleared\n",
					isp->ctrl_msg.ctrl_ext.ctrl_id);
				memset(&isp->ctrl_msg, 0, sizeof(isp->ctrl_msg));
				break;
			}
		} else if (!isp->ctrl_msg.type) {
			break;
		}
		cond = !cond ? isp->ctrl_msg_wait_cond + 1 : cond + 1;
		mutex_unlock(&isp->ctrl_lock);
		wait_event_timeout(isp->ctrl_msg_waitq,
				   cond == isp->ctrl_msg_wait_cond,
				   msecs_to_jiffies(ISP_CTRL_WAIT_TIME_MS));
		if (isp->ctrl_exit)
			return -EINTR;
	}
	isp->ctrl_msg.rc = (int)msg->group;
	memcpy(&isp->ctrl_msg.ctrl_ext, &msg->ctrl_ext, sizeof(msg->ctrl_ext));
	isp->ctrl_msg.type = ISP_CTRL_EXT_MSG;
	wake_up_all(&isp->ctrl_ack_waitq);
	mutex_unlock(&isp->ctrl_lock);
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
	case ISP_MSG_SET_RGBGAMMA:
		rc = handle_set_rgbgamma(isp, m);
		break;
	case ISP_MSG_SET_WDR5_HIST:
		rc = handle_set_wdr5_hist(isp, m);
		break;
	case ISP_MSG_SET_WDR5_SHIFT:
		rc = handle_set_wdr5_shift(isp, m);
		break;
	case ISP_MSG_SET_WDR5_SHIFT0:
		rc = handle_set_wdr5_shift0(isp, m);
		break;
	case ISP_MSG_SET_WDR5_GAMMAPRE:
		rc = handle_set_wdr5_gammapre(isp, m);
		break;
	case ISP_MSG_SET_WDR5_GAMMADOWN:
		rc = handle_set_wdr5_gammadown(isp, m);
		break;
	case ISP_MSG_SET_WDR5_ENTROPY:
		rc = handle_set_wdr5_entropy(isp, m);
		break;
	case ISP_MSG_SET_WDR5_DISTANCE:
		rc = handle_set_wdr5_distance(isp, m);
		break;
	case ISP_MSG_SET_WDR5_DIFFERENCE:
		rc = handle_set_wdr5_difference(isp, m);
		break;
	case ISP_MSG_SET_WDR5_FACTOR:
		rc = handle_set_wdr5_factor(isp, m);
		break;
	case ISP_MSG_SET_WDR5_LEVEL:
		rc = handle_set_wdr5_level(isp, m);
		break;
	case ISP_MSG_SET_WDR5_SAT_SHIFT:
		rc = handle_set_wdr5_sat_shift(isp, m);
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
	case ISP_MSG_ACK_CTRL:
		rc = handle_ack_ctrl(isp, m);
		break;
	case ISP_MSG_ACK_CTRL_EXT:
		rc = handle_ack_ctrl_ext(isp, m);
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

void frame_done(struct isp_device *isp, u32 inst, bool timeout)
{
	struct isp_instance *ins;
	struct isp_irq_ctx *ctx;
	struct cam_list_node *node;
	ktime_t now_time = ktime_get_boottime();
	struct cam_frame_info *info = NULL;
	struct ibuf *ib = NULL;
	struct cam_ctx *src_ctx = NULL;
	u32 i;

	ins = &isp->insts[inst];
	ctx = &ins->ctx;
	if (ins->online_mcm) {
		ib = list_first_entry_or_null(&isp->ibm[isp->cur_mi_irq_ctx].list3, struct ibuf,
				      entry);
		if (ib) {
			info = &ib->info;
			list_del(&ib->entry);
			list_add_tail(&ib->entry, &isp->ibm[isp->cur_mi_irq_ctx].list1);
		}
	}

	if (ctx->sink_online_en) {
		if (isp->mode == ISP_STRM_MODE)
			sif_get_frame_des(ctx->info_ctx);
		else
			cam_update_frame_info(ctx->info_ctx, info);
	}

	if (ctx->sink_buf) {
		cam_qbuf_irq(ctx->sink_ctx, ctx->sink_buf, false);
		ctx->sink_buf = NULL;
	}

	node = list_first_entry_or_null(ctx->src_buf_list3, struct cam_list_node, entry);
	if (node) {
		if (has_offline(ctx->src_online_stat)) {
			i = get_offline(ctx->src_online_stat);
			src_ctx = ctx->src_ctx[i];
		}

		if (cam_get_frame_status(src_ctx) || timeout) {
			cam_drop_irq(src_ctx, node->data);
		} else {
			if (isp->mode == ISP_STRM_MODE && ctx->src_buf &&
			    ctx->src_buf == ctx->next_src_buf)
				pr_debug("isp inst: %d, src_buf is the same with next_src_buf! skip "
					 "dequeueing mp buf!\n", inst);
			else
				cam_qbuf_irq(src_ctx, node->data, true);
		}
		cam_set_frame_status(src_ctx, NO_ERR);
		list_del(&node->entry);
		list_add_tail(&node->entry, ctx->src_buf_list1);

		if (isp->mode == ISP_STRM_MODE) {
			ctx->src_buf = ctx->next_src_buf;
			ctx->next_src_buf = NULL;
		}
	}

	node = list_first_entry_or_null(ctx->src_raw_buf_list3, struct cam_list_node, entry);
	if (node) {
		src_ctx = ctx->src_raw_ctx;

		if (cam_get_frame_status(src_ctx) || timeout) {
			cam_drop_irq(src_ctx, node->data);
		} else {
			if (isp->mode == ISP_STRM_MODE && ctx->src_raw_buf &&
			    ctx->src_raw_buf == ctx->next_src_raw_buf)
				pr_debug("isp inst: %d, src_raw_buf is the same with next_src_raw_buf! skip "
					 "dequeueing mp buf!\n", inst);
			else
				cam_qbuf_irq(src_ctx, node->data, true);
		}
		cam_set_frame_status(src_ctx, NO_ERR);
		list_del(&node->entry);
		list_add_tail(&node->entry, ctx->src_raw_buf_list1);

		if (isp->mode == ISP_STRM_MODE) {
			ctx->src_raw_buf = ctx->next_src_raw_buf;
			ctx->next_src_raw_buf = NULL;
		}
	}

	if (ins->last_frame_done)
		ins->frame_interval += ktime_to_ms(ktime_sub(now_time, ins->last_frame_done));

	ins->last_frame_done = now_time;
	ins->frame_count++;
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
		inst->job_count--;
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
	struct cam_buf *src_buf, *sink_buf = NULL;

	if (!ctx->sink_online_en && ctx->sink_ctx) {
		sink_buf = cam_acqbuf_irq(ctx->sink_ctx, false);
		if (!sink_buf) {
			ctx->sink_buf = NULL;
			return -ENOMEM;
		}
	}

	if (has_offline(ctx->src_online_stat)) {
		struct cam_list_node *node;
		u32 i = get_offline(ctx->src_online_stat);

		src_buf = cam_dqbuf_irq(ctx->src_ctx[i], true);
		if (!src_buf)
			return -ENOMEM;

		node = list_first_entry_or_null(ctx->src_buf_list1,
						struct cam_list_node, entry);
		if (WARN_ON(!node))
			return -ENOMEM;
		node->data = src_buf;
		list_del(&node->entry);
		list_add_tail(&node->entry, ctx->src_buf_list2);
		pr_debug("isp list_add_tail src_buf_list2\n");
	}

	if (ctx->src_raw_ctx) {
		struct cam_list_node *node;

		src_buf = cam_dqbuf_irq(ctx->src_raw_ctx, true);
		if (!src_buf)
			return -ENOMEM;
		node = list_first_entry_or_null(ctx->src_raw_buf_list1,
						struct cam_list_node, entry);
		if (WARN_ON(!node))
			return -ENOMEM;
		node->data = src_buf;
		list_del(&node->entry);
		list_add_tail(&node->entry, ctx->src_raw_buf_list2);
		pr_debug("isp list_add_tail src_raw_buf_list2\n");
	}

	if (sink_buf)
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
	struct isp_mcm_sch sch = {0};
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
		if (mi_mis.miv2_mis & 0x3) {
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
		memset(&sch, 0, sizeof(sch));
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

static int isp_wdr5_histogram(struct isp_device *isp, struct isp_wdr5_data *pwdr_data)
{
	struct isp_wdr5_data *wdr5 = pwdr_data;
	u32 k = 0;
	u32 val;

	isp_write(isp, ISP_WDR5_LUT_HISTOGRAM_ADDR, 0);

	for (k = 0; k < 21; k++) {
		val = (wdr5->lut_histogram_write_data[k * 3] |
				((wdr5->lut_histogram_write_data[k * 3 + 1] & 0x3FF) << 20));
		isp_write(isp, ISP_WDR5_LUT_HISTOGRAM_WRITE_DATA, val);
		val = (((wdr5->lut_histogram_write_data[k * 3 + 1] & 0XFFC00) << 10)
				| wdr5->lut_histogram_write_data[k * 3 + 2]);
		isp_write(isp, ISP_WDR5_LUT_HISTOGRAM_WRITE_DATA, val);
	}

	isp_write(isp, ISP_WDR5_LUT_HISTOGRAM_WRITE_DATA, wdr5->lut_histogram_write_data[63]);
	isp_write(isp, ISP_WDR5_LUT_HISTOGRAM_WRITE_DATA, wdr5->lut_histogram_write_data[64]);
	wdr5->histogram_w_data_changed = false;

	return 0;
}

static int isp_wdr5_gammapre(struct isp_device *isp, struct isp_wdr5_data *pwdr_data)
{
	struct isp_wdr5_data *wdr5 = pwdr_data;
	u32 k = 0;
	u32 val;

	isp_write(isp, ISP_WDR5_LUT_GAMMAPRE_ADDR, 0);

	for (k = 0; k < 21; k++) {
		val = (wdr5->lut_gammapre_write_data[k * 3] |
				((wdr5->lut_gammapre_write_data[k * 3 + 1] & 0x3FF) << 20));
		isp_write(isp, ISP_WDR5_LUT_GAMMAPRE_WRITE_DATA, val);
		val = (((wdr5->lut_gammapre_write_data[k * 3 + 1] & 0XFFC00) << 10)
				|  wdr5->lut_gammapre_write_data[k * 3 + 2]);
		isp_write(isp, ISP_WDR5_LUT_GAMMAPRE_WRITE_DATA, val);
	}

	isp_write(isp, ISP_WDR5_LUT_GAMMAPRE_WRITE_DATA, wdr5->lut_gammapre_write_data[63]);
	isp_write(isp, ISP_WDR5_LUT_GAMMAPRE_WRITE_DATA, wdr5->lut_gammapre_write_data[64]);
	wdr5->gammapre_w_data_changed = false;

	return 0;
}

int isp_wdr5_gammadown(struct isp_device *isp, struct isp_wdr5_data *pwdr_data)
{
	struct isp_wdr5_data *wdr5 = pwdr_data;
	u32 k = 0;
	u32 val;

	isp_write(isp, ISP_WDR5_LUT_GAMMADOWN_ADDR, 0);

	for (k = 0; k < 21; k++) {
		val = ((wdr5->lut_gammadown_write_data[k * 3] << 20)
				| (wdr5->lut_gammadown_write_data[k * 3 + 1] << 10)
				| wdr5->lut_gammadown_write_data[k * 3 + 2]);
		isp_write(isp, ISP_WDR5_LUT_GAMMADOWN_WRITE_DATA, val);
	}

	isp_write(isp, ISP_WDR5_LUT_GAMMADOWN_WRITE_DATA,
			((wdr5->lut_gammadown_write_data[63] << 20)
			 | (wdr5->lut_gammadown_write_data[64] << 10)));

	wdr5->gammadown_w_data_changed = false;

	return 0;
}

int isp_wdr5_entropy(struct isp_device *isp, struct isp_wdr5_data *pwdr_data)
{
	struct isp_wdr5_data *wdr5 = pwdr_data;
	u32 k = 0;
	u32 val;

	isp_write(isp, ISP_WDR5_LUT_ENTROPY_ADDR, 0);

	for (k = 0; k < 21; k++) {
		val = ((wdr5->lut_entropy_write_data[k * 3] << WDR5_ENTROPY_CONVERT0_SHIFT)
				| (wdr5->lut_entropy_write_data[k * 3 + 1] << WDR5_ENTROPY_CONVERT1_SHIFT)
				| wdr5->lut_entropy_write_data[k * 3 + 2]);
		isp_write(isp, ISP_WDR5_LUT_ENTROPY_WRITE_DATA, val);
	}

	isp_write(isp, ISP_WDR5_LUT_ENTROPY_WRITE_DATA,
			((wdr5->lut_entropy_write_data[63] << WDR5_ENTROPY_CONVERT18_SHIFT)
			 | (wdr5->lut_entropy_write_data[64] << WDR5_ENTROPY_CONVERT19_SHIFT)));

	wdr5->entropy_w_data_changed = false;

	return 0;
}

int isp_wdr5_lut_shift(struct isp_device *isp, struct isp_wdr5_data *pwdr_data)
{
	struct isp_wdr5_data *wdr5 = pwdr_data;
	u32 k = 0;
	u32 val;

	isp_write(isp, ISP_WDR5_LUT_SHIFT_ADDR, 0);

	for (k = 0; k < 10; k++) {
		val = ((wdr5->lut_shift_write_data[k * 6] << 25)
				| (wdr5->lut_shift_write_data[k * 6 + 1] << 20)
				| (wdr5->lut_shift_write_data[k * 6 + 2] << 15)
				| (wdr5->lut_shift_write_data[k * 6 + 3] << 10)
				| (wdr5->lut_shift_write_data[k * 6 + 4] << 5)
				| wdr5->lut_shift_write_data[k * 6 + 5]);
		isp_write(isp, ISP_WDR5_LUT_SHIFT_WRITE_DATA, val);
	}

	val = ((wdr5->lut_shift_write_data[60] << 25U)
			| (wdr5->lut_shift_write_data[61] << 20U)
			| (wdr5->lut_shift_write_data[62] << 15U)
			| (wdr5->lut_shift_write_data[63] << 10U)
			| (wdr5->lut_shift_write_data[64] << 5U));
	isp_write(isp, ISP_WDR5_LUT_SHIFT_WRITE_DATA, val);
	wdr5->lut_shift_w_data_changed = false;

	return 0;
}

int isp_wdr5_lut_shift0(struct isp_device *isp, struct isp_wdr5_data *pwdr_data)
{
	struct isp_wdr5_data *wdr5 = pwdr_data;
	u32 k = 0;
	u32 val;

	isp_write(isp, ISP_WDR5_LUT_SHIFT0_ADDR, 0);
	for (k = 0; k < 8; k++) {
		val = ((wdr5->lut_shift0_write_data[k * 8] << 28)
				| (wdr5->lut_shift0_write_data[k * 8 + 1] << 24)
				| (wdr5->lut_shift0_write_data[k * 8 + 2] << 20)
				| (wdr5->lut_shift0_write_data[k * 8 + 3] << 16)
				| (wdr5->lut_shift0_write_data[k * 8 + 4] << 12)
				| (wdr5->lut_shift0_write_data[k * 8 + 5] << 8)
				| (wdr5->lut_shift0_write_data[k * 8 + 6] << 4)
				| wdr5->lut_shift0_write_data[k * 8 + 7]);
		isp_write(isp, ISP_WDR5_LUT_SHIFT0_WRITE_DATA, val);
	}

	isp_write(isp, ISP_WDR5_LUT_SHIFT0_WRITE_DATA, wdr5->lut_shift0_write_data[64]);
	wdr5->lut_shift0_w_data_changed = false;
	return 0;
}

int isp_wdr5_distance_weight(struct isp_device *isp, struct isp_wdr5_data *pwdr_data)
{
	struct isp_wdr5_data *wdr5 = pwdr_data;
	u32 k = 0;
	u32 val;

	isp_write(isp, ISP_WDR5_LUT_DISTANCE_WEIGHT_ADDR, 0);

	for (k = 0; k < 16; k++) {
		val = ((wdr5->lut_distance_weight_write_data[k * 4] << 21)
				| (wdr5->lut_distance_weight_write_data[k * 4 + 1] << 14)
				| (wdr5->lut_distance_weight_write_data[k * 4 + 2] << 7)
				|  wdr5->lut_distance_weight_write_data[k * 4 + 3]);
		isp_write(isp, ISP_WDR5_LUT_DISTANCE_WEIGHT_WRITE_DATA, val);
	}

	isp_write(isp, ISP_WDR5_LUT_DISTANCE_WEIGHT_WRITE_DATA,  wdr5->lut_distance_weight_write_data[64]);
	wdr5->lut_distance_weight_w_data_changed = false;

	return 0;
}

int isp_wdr5_difference_weight(struct isp_device *isp, struct isp_wdr5_data *pwdr_data)
{
	struct isp_wdr5_data *wdr5 = pwdr_data;
	u32 k = 0;
	u32 val;

	isp_write(isp, ISP_WDR5_LUT_DIFFERENCE_WEIGHT_ADDR, 0);

	for (k = 0; k < 16; k++) {
		val = ((wdr5->lut_difference_weight_write_data[k * 4] << 21)
				| (wdr5->lut_difference_weight_write_data[k * 4 + 1] << 14)
				| (wdr5->lut_difference_weight_write_data[k * 4 + 2] << 7)
				| wdr5->lut_difference_weight_write_data[k * 4 + 3]);
		isp_write(isp, ISP_WDR5_LUT_DIFFERENCE_WEIGHT_WRITE_DATA, val);
	}

	isp_write(isp, ISP_WDR5_LUT_DIFFERENCE_WEIGHT_WRITE_DATA,
			wdr5->lut_difference_weight_write_data[64]);
	wdr5->difference_weight_w_data_changed = false;

	return 0;
}

int isp_wdr5_flat_factor(struct isp_device *isp, struct isp_wdr5_data *pwdr_data)
{
	struct isp_wdr5_data *wdr5 = pwdr_data;
	u32 k = 0;
	u32 val;

	isp_write(isp, ISP_WDR5_LUT_FLAT_FACTOR_ADDR, 0);

	for (k = 0; k < 90; k++) {
		val = ((wdr5->lut_flat_factor_write_data[k * 3] << 20)
				| (wdr5->lut_flat_factor_write_data[k * 3 + 1] << 10)
				|  wdr5->lut_flat_factor_write_data[k * 3 + 2]);
		isp_write(isp, ISP_WDR5_LUT_FLAT_FACTOR_WRITE_DATA, val);
	}

	isp_write(isp, ISP_WDR5_LUT_FLAT_FACTOR_WRITE_DATA,
			((wdr5->lut_flat_factor_write_data[270] << 20)
			 | (wdr5->lut_flat_factor_write_data[271] << 10)));

	wdr5->flat_factor_w_data_changed = false;

	return 0;
}

int isp_wdr5_flat_level(struct isp_device *isp, struct isp_wdr5_data *pwdr_data)
{
	struct isp_wdr5_data *wdr5 = pwdr_data;
	u32 k = 0;
	u32 val;

	isp_write(isp, ISP_WDR5_LUT_FLAT_LEVEL_ADDR, 0);

	for (k = 0; k < 8; k++) {
		val = ((wdr5->lut_flat_level_write_data[k * 8] << 28)
				| (wdr5->lut_flat_level_write_data[k * 8 + 1] << 24)
				| (wdr5->lut_flat_level_write_data[k * 8 + 2] << 20)
				| (wdr5->lut_flat_level_write_data[k * 8 + 3] << 16)
				| (wdr5->lut_flat_level_write_data[k * 8 + 4] << 12)
				| (wdr5->lut_flat_level_write_data[k * 8 + 5] << 8)
				| (wdr5->lut_flat_level_write_data[k * 8 + 6] << 4)
				|  wdr5->lut_flat_level_write_data[k * 8 + 7]);
		isp_write(isp, ISP_WDR5_LUT_FLAT_LEVEL_WRITE_DATA, val);
	}

	isp_write(isp, ISP_WDR5_LUT_FLAT_LEVEL_WRITE_DATA,
			((wdr5->lut_flat_level_write_data[64] << 28)
			 | (wdr5->lut_flat_level_write_data[65] << 24)
			 | (wdr5->lut_flat_level_write_data[66] << 20)
			 | (wdr5->lut_flat_level_write_data[67] << 16)));

	wdr5->flat_level_w_data_changed = false;

	return 0;
}

int isp_wdr5_sat_shift(struct isp_device *isp, struct isp_wdr5_data *pwdr_data)
{
	struct isp_wdr5_data *wdr5 = pwdr_data;
	u32 k = 0;
	u32 val;

	isp_write(isp, ISP_WDR5_LUT_SAT_SHIFT_ADDR, 0);

	for (k = 0; k < 2; k++) {
		val = ((wdr5->lut_sat_shift_write_data[k * 8] << 28)
				| (wdr5->lut_sat_shift_write_data[k * 8 + 1] << 24)
				| (wdr5->lut_sat_shift_write_data[k * 8 + 2] << 20)
				| (wdr5->lut_sat_shift_write_data[k * 8 + 3] << 16)
				| (wdr5->lut_sat_shift_write_data[k * 8 + 4] << 12)
				| (wdr5->lut_sat_shift_write_data[k * 8 + 5] << 8)
				| (wdr5->lut_sat_shift_write_data[k * 8 + 6] << 4)
				| wdr5->lut_sat_shift_write_data[k * 8 + 7]);
		isp_write(isp, ISP_WDR5_LUT_SAT_SHIFT_WRITE_DATA, val);
	}

	isp_write(isp, ISP_WDR5_LUT_SAT_SHIFT_WRITE_DATA,
			((wdr5->lut_sat_shift_write_data[16] << 28)
			 | (wdr5->lut_sat_shift_write_data[17] << 24)));

	wdr5->sat_shift_w_data_changed = false;

	return 0;
}

static int isp_s_rgbgammapx(struct isp_device *isp, struct isp_rgbgamma_data *data)
{
	u32 isp_r_px_reg = ISP_GCRGB_R_PX_0;
	u32 isp_g_px_reg = ISP_GCRGB_G_PX_0;
	u32 isp_b_px_reg = ISP_GCRGB_B_PX_0;
	u32 *p_r_table = NULL;
	u32 *p_g_table = NULL;
	u32 *p_b_table = NULL;
	int i;
	u32 gc_px_r_data = 0;
	u32 gc_px_g_data = 0;
	u32 gc_px_b_data = 0;

	p_r_table = (u32 *)&data->rgbgc_r_px;
	p_g_table = (u32 *)&data->rgbgc_g_px;
	p_b_table = (u32 *)&data->rgbgc_b_px;
	for (i = 0; i < 64; i++) {
		gc_px_r_data |= (*(p_r_table + i) << (i % 6 * 5));
		if (i % 6 == 5 || i == 63) {
			isp_write(isp, isp_r_px_reg, gc_px_r_data);
			isp_r_px_reg += 4;
			gc_px_r_data = 0;
		}

		gc_px_g_data |= (*(p_g_table + i) << (i % 6 * 5));
		if (i % 6 == 5 || i == 63) {
			isp_write(isp, isp_g_px_reg, gc_px_g_data);
			isp_g_px_reg += 4;
			gc_px_g_data = 0;
		}

		gc_px_b_data |= (*(p_b_table + i) << (i % 6 * 5));
		if (i % 6 == 5 || i == 63) {
			isp_write(isp, isp_b_px_reg, gc_px_b_data);
			isp_b_px_reg += 4;
			gc_px_b_data = 0;
		}
	}
	return 0;
}

static int isp_s_rgbgammaWriteData(struct isp_device *isp,
			    struct isp_rgbgamma_data *data)
{
	u32 isp_gc_x_data, isp_gc_y_data;

	int i;
	u32 *r_tblX, *r_tblY;
	u32 *g_tblX, *g_tblY;
	u32 *b_tblX, *b_tblY;

	isp_write(isp, ISP_GCRGB_R_Y_ADDR, 0);
	isp_write(isp, ISP_GCRGB_R_X_ADDR, 0);

	isp_write(isp, ISP_GCRGB_G_Y_ADDR, 0);
	isp_write(isp, ISP_GCRGB_G_X_ADDR, 0);

	isp_write(isp, ISP_GCRGB_B_Y_ADDR, 0);
	isp_write(isp, ISP_GCRGB_B_X_ADDR, 0);

	r_tblX = data->rgbgc_r_datax;
	r_tblY = data->rgbgc_r_datay;

	g_tblX = data->rgbgc_g_datax;
	g_tblY = data->rgbgc_g_datay;

	b_tblX = data->rgbgc_b_datax;
	b_tblY = data->rgbgc_b_datay;

	for (i = 0; i < 64; i++) {
		isp_gc_y_data = *(r_tblY + i);
		isp_write(isp, ISP_GCRGB_R_Y_WRITE_DATA,
				isp_gc_y_data);

		isp_gc_y_data = *(g_tblY + i);
		isp_write(isp, ISP_GCRGB_G_Y_WRITE_DATA,
				isp_gc_y_data);

		isp_gc_y_data = *(b_tblY + i);
		isp_write(isp, ISP_GCRGB_B_Y_WRITE_DATA,
				isp_gc_y_data);

		if (i < 63) {
			isp_gc_x_data = *(r_tblX + i);
			isp_write(isp, ISP_GCRGB_R_X_WRITE_DATA,
				isp_gc_x_data);

			isp_gc_x_data = *(g_tblX + i);
			isp_write(isp, ISP_GCRGB_G_X_WRITE_DATA,
				isp_gc_x_data);

			isp_gc_x_data = *(b_tblX + i);
			isp_write(isp, ISP_GCRGB_B_X_WRITE_DATA,
				isp_gc_x_data);

		}
	}
	return 0;
}

void isp_update_none_shd_regs(unsigned long data)
{
	struct isp_device *isp = (struct isp_device *)data;
	/* For using tasklet to update non-shading registers, it's just enabled
	 * in the stream mode, that's, there's just only isp instance 0 which
	 * should be used in the case. And for others, it's updated in camera
	 * service.
	 */
	struct isp_instance *ins = &isp->insts[0];
	struct isp_gamma_febe_ctrl *febe_ctrl = &ins->febe_ctrl;
	struct isp_rgbgamma_data *rgbgamma_data = &ins->rgbgamma_data;
	struct isp_wdr5_data *wdr5_data = &ins->wdr5_data;
	u32 i = 0;

	if (febe_ctrl->flag) {
		pr_debug("config febe\n");
		isp_write(isp, ISP_GAMMA_FE_Y_ADDR, 0);
		isp_write(isp, ISP_GAMMA_BE_Y_ADDR, 0);
		for (i = 0; i < ISP_CTRL_FEBE_NUM; i++) {
			isp_write(isp, ISP_GAMMA_FE_Y_WRITE_DATA,
					febe_ctrl->compress[i]);
			isp_write(isp, ISP_GAMMA_BE_Y_WRITE_DATA,
					febe_ctrl->expand[i]);
		}
		febe_ctrl->flag = false;
	}
	if (rgbgamma_data->flag) {
		pr_debug("config rgb gamma\n");
		isp_s_rgbgammapx(isp, rgbgamma_data);
		isp_s_rgbgammaWriteData(isp, rgbgamma_data);
		rgbgamma_data->flag = false;
	}

	pr_debug("wdr5_data->histogram_w_data_changed %d\n",
			wdr5_data->histogram_w_data_changed);
	if (wdr5_data->histogram_w_data_changed)
		isp_wdr5_histogram(isp, wdr5_data);

	if (wdr5_data->gammapre_w_data_changed)
		isp_wdr5_gammapre(isp, wdr5_data);

	if (wdr5_data->gammadown_w_data_changed)
		isp_wdr5_gammadown(isp, wdr5_data);

	if (wdr5_data->entropy_w_data_changed)
		isp_wdr5_entropy(isp, wdr5_data);

	if (wdr5_data->lut_shift_w_data_changed)
		isp_wdr5_lut_shift(isp, wdr5_data);

	if (wdr5_data->lut_shift0_w_data_changed)
		isp_wdr5_lut_shift0(isp, wdr5_data);

	if (wdr5_data->lut_distance_weight_w_data_changed)
		isp_wdr5_distance_weight(isp, wdr5_data);

	if (wdr5_data->difference_weight_w_data_changed)
		isp_wdr5_difference_weight(isp, wdr5_data);

	if (wdr5_data->flat_factor_w_data_changed)
		isp_wdr5_flat_factor(isp, wdr5_data);

	if (wdr5_data->flat_level_w_data_changed)
		isp_wdr5_flat_level(isp, wdr5_data);

	if (wdr5_data->sat_shift_w_data_changed)
		isp_wdr5_sat_shift(isp, wdr5_data);
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
				memset(&sch, 0, sizeof(sch));
				if (isp->mode != ISP_STRM_MODE)
					isp_set_schedule(isp, &sch, 0, isp_mis, true);
				else
					tasklet_schedule(&isp->update_lut_tbl);
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
