// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "[isp_drv]: %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include "cam_ctrl.h"
#include "cam_dev.h"
#include "cam_buf.h"
#include "isc.h"
#include "isp_uapi.h"

#include "isp.h"
#include "isp8000_regs.h"

#define REFCNT_INIT_VAL (1)

#ifdef EN_CHK_FMT
static bool check_format(struct isp_instance *ins, struct cam_format *fmt)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(ins->fmt_cap); i++) {
		struct isp_format_cap *cap = &ins->fmt_cap[i];

		if (!cap->format)
			return false;
		if (cap->format == fmt->format)
			return check_framesize(cap->res, ARRAY_SIZE(cap->res), fmt);
	}
	return false;
}
#endif

int isp_post(struct isp_device *isp, struct isp_msg *msg, bool sync)
{
	struct isc_post_param param = {
		.msg = msg,
		.msg_len = sizeof(*msg),
		.lock = &isp->isc_lock,
		.sync = sync,
	};
	int rc = -EINVAL;

	if (isp->isc)
		rc = isc_post(isp->isc, &param);
	return rc;
}

int isp_post_ex(struct isp_device *isp, struct isp_msg *msg,
		struct mem_buf *extra, bool sync)
{
	struct isc_post_param param = {
		.msg = msg,
		.msg_len = sizeof(*msg),
		.extra = extra,
		.lock = &isp->isc_lock,
		.sync = sync,
	};
	int rc = -EINVAL;

	if (isp->isc)
		rc = isc_post(isp->isc, &param);
	return rc;
}

int isp_set_input(struct isp_device *isp, u32 inst, struct cam_input *in)
{
	struct isp_instance *ins;
	struct isp_msg msg;

	if (!isp || !in)
		return -EINVAL;

	if (inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[inst];

	memcpy(&ins->in, in, sizeof(ins->in));
	msg.id = CAM_MSG_INPUT_CHANGED;
	msg.inst = inst;
	memcpy(&msg.in, in, sizeof(msg.in));
	return isp_post(isp, &msg, true);
}

int isp_set_input_select(struct isp_device *isp, u32 inst, u32 in_id, u32 in_chnl)
{
	int rc = -EINVAL;

	if (!isp)
		return rc;

	mutex_lock(&isp->set_input_lock);
	if (inst < ISP_SINK_ONLINE_PATH_MAX)
		rc = set_isp_input_select(isp->ctrl_dev, inst, in_id, in_chnl);
	mutex_unlock(&isp->set_input_lock);

	return rc;
}

int isp_set_subctrl(struct isp_device *isp, u32 inst, u32 cmd, void *data, u32 size)
{
	struct isp_msg msg;
	void *buf_va;
	int ret;

	if (!isp)
		return -EINVAL;

	if (!data || !size) {
		pr_info("%s: invalid ctrl data!\n", __func__);
		return -EINVAL;
	}

	memset(&msg, 0, sizeof(msg));

	msg.inst = inst;
	if (size < ISP_CTRL_DATA_LENGTH) {
		msg.id = CAM_MSG_CTRL_CHANGED;
		msg.ctrl.ctrl_id = cmd;
		msg.ctrl.dir = 1;
		msg.ctrl.size = size;

		ret = copy_from_user(msg.ctrl.ctrl_data, data, size);
		if (ret) {
			pr_info("%s: ctrl_data copy_from_user failed!\n", __func__);
			return ret;
		}
		ret = isp_post(isp, &msg, true);
	} else {
		msg.id = CAM_MSG_CTRL_EXT_CHANGED;
		msg.ctrl_ext.ctrl_id = cmd;
		msg.ctrl_ext.dir = 1;
		msg.ctrl_ext.size = size;
		msg.ctrl_ext.buf.size = size;

		buf_va = isc_alloc_extra_buf(isp->isc, &msg.ctrl_ext.buf);
		if (!buf_va) {
			pr_info("%s: isc_alloc_extra_buf failed!\n", __func__);
			return -ENOMEM;
		}

		ret = copy_from_user(buf_va, data, size);
		if (ret) {
			pr_info("%s: ctrl_data copy_from_user failed!\n", __func__);
			isc_free_extra_buf(isp->isc, &msg.ctrl_ext.buf);
			return ret;
		}
		ret = isp_post_ex(isp, &msg, &msg.ctrl_ext.buf, true);

		isc_free_extra_buf(isp->isc, &msg.ctrl_ext.buf);
	}

	return ret;
}

int isp_get_subctrl(struct isp_device *isp, u32 inst, u32 cmd, void *data, u32 size)
{
	struct isp_msg msg;
	void *buf_va;
	int ret;

	if (!isp)
		return -EINVAL;

	if (!data || !size) {
		pr_info("%s: invalid ctrl data!\n", __func__);
		return -EINVAL;
	}

	memset(&msg, 0, sizeof(msg));

	msg.inst = inst;
	if (size < ISP_CTRL_DATA_LENGTH) {
		msg.id = CAM_MSG_CTRL_CHANGED;
		msg.ctrl.ctrl_id = cmd;
		msg.ctrl.dir = 0;
		msg.ctrl.size = size;
		ret = copy_from_user(msg.ctrl.ctrl_data, data, size);
		if (ret) {
			pr_info("%s: ctrl_data copy_from_user failed!\n", __func__);
			return ret;
		}
		ret = isp_post(isp, &msg, true);
		if (ret < 0) {
			pr_info("%s: msg isp_post failed!\n", __func__);
			return ret;
		}

		ret = copy_to_user(data, msg.ctrl.ctrl_data, size);
		if (ret) {
			pr_info("%s: ctrl_data copy_to_user failed!\n", __func__);
			return ret;
		}
	} else {
		msg.id = CAM_MSG_CTRL_EXT_CHANGED;
		msg.ctrl_ext.ctrl_id = cmd;
		msg.ctrl_ext.dir = 0;
		msg.ctrl_ext.size = size;
		msg.ctrl_ext.buf.size = size;

		buf_va = isc_alloc_extra_buf(isp->isc, &msg.ctrl_ext.buf);
		if (!buf_va) {
			pr_info("%s: isc_alloc_extra_buf failed!\n", __func__);
			return -ENOMEM;
		}

		ret = copy_from_user(buf_va, data, size);
		if (ret) {
			pr_info("%s: ctrl_data copy_from_user failed!\n", __func__);
			isc_free_extra_buf(isp->isc, &msg.ctrl_ext.buf);
			return ret;
		}
		ret = isp_post_ex(isp, &msg, &msg.ctrl_ext.buf, true);
		if (ret < 0) {
			pr_info("%s: msg isp_post_ex failed!\n", __func__);
			isc_free_extra_buf(isp->isc, &msg.ctrl_ext.buf);
			return ret;
		}

		ret = copy_to_user(data, buf_va, size);
		if (ret) {
			pr_info("%s: ctrl_data copy_to_user failed!\n", __func__);
			isc_free_extra_buf(isp->isc, &msg.ctrl_ext.buf);
			return ret;
		}
		isc_free_extra_buf(isp->isc, &msg.ctrl_ext.buf);
	}

	return 0;
}

int isp_set_iformat(struct isp_device *isp, u32 inst, struct cam_format *fmt, struct cam_rect *crop,
		    bool hdr_en)
{
	struct isp_instance *ins;
	int rc = 0;

	if (!isp || !fmt || !crop)
		return -EINVAL;

	if (inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[inst];

	ins->hdr_en = hdr_en;
	memcpy(&ins->fmt.ifmt, fmt, sizeof(ins->fmt.ifmt));
	memcpy(&ins->fmt.icrop, crop, sizeof(ins->fmt.icrop));
	if (ins->fmt.ofmt.format != CAM_FMT_NULL)
		rc = isp_set_format(isp, inst, &ins->fmt);
	return rc;
}

int isp_set_oformat(struct isp_device *isp, u32 inst, struct cam_format *fmt)
{
	struct isp_instance *ins;
	int rc = 0;

	if (!isp || !fmt)
		return -EINVAL;

	if (inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[inst];

#ifdef EN_CHK_FMT
	if (!check_format(ins, fmt))
		return -EINVAL;
#endif

	memcpy(&ins->fmt.ofmt, fmt, sizeof(ins->fmt.ofmt));
	if (ins->fmt.ifmt.format != CAM_FMT_NULL)
		rc = isp_set_format(isp, inst, &ins->fmt);
	return rc;
}

static int alloc_mcm_buf(struct isp_device *isp, u32 id, struct cam_format *fmt)
{
	u32 size, i;
	struct mem_buf *buf;

	size = fmt->stride * fmt->height * MCM_BUF_NUM;
	if (!size)
		return -EINVAL;
	buf = &isp->in_bufs[id];
	if (buf->size > 0 && buf->size != size)
		mem_free(isp->dev, &isp->in_buf_list, buf);
	if (buf->size != size) {
		buf->size = size;
		mem_alloc(isp->dev, &isp->in_buf_list, buf);

		size = fmt->stride * fmt->height;
		for (i = 0; i < MCM_BUF_NUM; i++) {
			isp->ib[id][i].buf.addr = buf->addr + i * size;
			isp->ib[id][i].buf.size = size;
			list_add_tail(&isp->ib[id][i].entry, &isp->ibm[id].list1);
		}
	}
	return 0;
}

static int alloc_hdr_buf(struct isp_device *isp, u32 id, struct cam_format *fmt)
{
	u32 size;
	struct mem_buf *buf;

	size = fmt->stride * fmt->height;
	if (!size)
		return -EINVAL;

	buf = &isp->hdr_bufs[id];
	if (buf->size > 0 && buf->size != size)
		mem_free(isp->dev, &isp->hdr_buf_list, buf);
	if (buf->size != size) {
		buf->size = size;
		mem_alloc(isp->dev, &isp->hdr_buf_list, buf);
	}
	return 0;
}

int isp_set_format(struct isp_device *isp, u32 inst, struct isp_format *fmt)
{
	struct isp_instance *ins;
	struct isp_msg msg;

	if (!isp || !fmt)
		return -EINVAL;

	if (inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[inst];

#ifdef EN_CHK_FMT
	if (!check_format(ins, &fmt->ofmt))
		return -EINVAL;
#endif

	if (ins->hdr_en) {
		int i;

		for (i = 0; i < HDR_BUF_NUM; i++)
			alloc_hdr_buf(isp, i, &fmt->ifmt);
	}

	if (isp->mcm_en && ins->stream_idx > -1)
		alloc_mcm_buf(isp, inst, &fmt->ifmt);

	memcpy(&ins->fmt, fmt, sizeof(ins->fmt));
	msg.id = CAM_MSG_FORMAT_CHANGED;
	msg.inst = inst;
	memcpy(&msg.fmt, fmt, sizeof(msg.fmt));
	return isp_post(isp, &msg, true);
}

void isp_set_mcm_raw_buffer(struct isp_device *isp, u32 path_id,
			    phys_addr_t phys_addr, struct cam_format *fmt)
{
	u32 base = MI_MCMn_RAW_BASE(path_id);
	u32 value;

	isp_write(isp, MI_MCMn_RAW_ADDR(base), phys_addr);
#ifdef WITH_LEGACY_ISP
	isp_write(isp, MI_MCMn_RAW_SIZE(base), fmt->stride * fmt->height * MCM_BUF_NUM);
	value = 0x9;
#else
	isp_write(isp, MI_MCMn_RAW_SIZE(base), fmt->stride * fmt->height);
	value = 0x19;
#endif
	isp_write(isp, MI_MCMn_RAW_OFFS(base), 0x00000000);
	isp_write(isp, MI_MCM_CTRL, value);

	/* force update */
	switch (path_id) {
	case 0:
		value |= BIT(2);
		isp_write(isp, MI_MCM_CTRL, value);
		break;
	case 1:
		value |= BIT(7);
		isp_write(isp, MI_MCM_CTRL, value);
		break;
	case 2:
		value |= BIT(2);
		isp_write(isp, MI_MCM_G2_CTRL, value);
		break;
	case 3:
		value |= BIT(5);
		isp_write(isp, MI_MCM_G2_CTRL, value);
		break;
	default:
		break;
	}
}

void isp_set_rdma_buffer(struct isp_device *isp, phys_addr_t rdma_addr)
{
	u32 mcm_mi_ctrl;
	u32 mi_ctrl;

	isp_write(isp, MI_MCM_RDMA_START, rdma_addr);

	mcm_mi_ctrl = isp_read(isp, MI_MCM_CTRL);
	mcm_mi_ctrl |= BIT(6);
	isp_write(isp, MI_MCM_CTRL, mcm_mi_ctrl); /* force update */

	mi_ctrl = isp_read(isp, MI_CTRL);
	mi_ctrl |= BIT(15);
	isp_write(isp, MI_CTRL, mi_ctrl); /* start rdma */
}

static inline void isp_set_hdr_raw_buffer(struct isp_device *isp, u32 n,
					  phys_addr_t phys_addr, struct cam_format *fmt)
{
	if (n < HDR_BUF_NUM) {
		isp_write(isp, MI_HDR_RAW_ADDR(n), phys_addr & MP_RAW_BASE_AD_MASK);
		isp_write(isp, MI_HDR_RAW_SIZE(n), (fmt->stride * fmt->height) & MP_RAW_SIZE_MASK);
		isp_write(isp, MI_HDR_RAW_OFFS(n), 0);
		isp_write(isp, MI_HDR_DMA_ADDR(n), phys_addr & MP_RAW_BASE_AD_MASK);
	}
}

void isp_set_mp_buffer(struct isp_device *isp, phys_addr_t phys_addr, struct cam_format *fmt)
{
	if (phys_addr) {
		isp_write(isp, MI_MP_Y_ADDR, phys_addr);
		isp_write(isp, MI_MP_Y_SIZE, fmt->stride * fmt->height);
		isp_write(isp, MI_MP_CB_ADDR, phys_addr + fmt->stride * fmt->height);
		isp_write(isp, MI_MP_CB_SIZE, fmt->stride * fmt->height / 2);
	} else {
		isp_write(isp, MI_MP_Y_ADDR, 0x00000000);
		isp_write(isp, MI_MP_Y_SIZE, 0x00000000);
		isp_write(isp, MI_MP_CB_ADDR, 0x00000000);
		isp_write(isp, MI_MP_CB_SIZE, 0x00000000);
	}
	isp_write(isp, MI_MP_Y_OFFS, 0x00000000);
	isp_write(isp, MI_MP_CB_OFFS, 0x00000000);
	isp_write(isp, MI_MP_CR_ADDR, 0x00000000); /* yuv420 */
	isp_write(isp, MI_MP_CR_SIZE, 0x00000000);
	isp_write(isp, MI_MP_CR_OFFS, 0x00000000);

	isp_write(isp, MI_MP_CTRL, 0x00000038); /* force update */
}

int isp_set_state(struct isp_device *isp, u32 inst, int enable)
{
	struct isp_msg msg;
	struct isp_instance *ins;
	phys_addr_t phys_addr;
	int state, rc;

	if (!isp || inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[inst];
	if (enable) {
		if (ins->hdr_en) {
			int i = 0;

			for (i = 0; i < HDR_BUF_NUM; i++) {
				phys_addr = isp->hdr_bufs[i].addr;
				isp_set_hdr_raw_buffer(isp, i, phys_addr, &ins->fmt.ifmt);
			}
		}

		if (isp->mcm_en && ins->stream_idx > -1) {
			struct ibuf *ib = list_first_entry_or_null(&isp->ibm[inst].list1,
								   struct ibuf, entry);
			if (!ib)
				return -ENOMEM;
			phys_addr = ib->buf.addr;
			isp_set_mcm_raw_buffer(isp, ins->stream_idx, phys_addr, &ins->fmt.ifmt);
			ins->mcm_ib = ib;
			list_del(&ib->entry);
		}

		isp->rdma_busy = !isp->mcm_en;

		if (isp->rdma_busy) {
#if 0
			rc = new_frame(&ins->ctx);
			if (rc < 0) {
				dev_info(isp->dev, "failed to call new_frame (ignored).\n");
			} else if (ins->ctx.src_buf) {
				phys_addr = get_phys_addr(ins->ctx.src_buf, 0);
				isp_set_mp_buffer(isp, phys_addr, &ins->fmt.ofmt);
				isp->next_mi_irq_ctx = inst;
			}
#endif
		}
		state = CAM_STATE_STARTED;
		refcount_inc(&isp->set_state_refcnt);
	} else {
		state = CAM_STATE_STOPPED;
		if (refcount_read(&isp->set_state_refcnt) > REFCNT_INIT_VAL)
			refcount_dec(&isp->set_state_refcnt);
	}

	msg.id = CAM_MSG_STATE_CHANGED;
	msg.inst = inst;
	msg.state = state;
	rc = isp_post(isp, &msg, true);
	if (rc < 0)
		return rc;

	if (!enable) {
		list_splice_tail_init(&ins->src_buf_list2, &ins->src_buf_list1);
		list_splice_tail_init(&ins->src_buf_list3, &ins->src_buf_list1);
		if (isp->mcm_en) {
			list_splice_tail_init(&isp->ibm[inst].list2, &isp->ibm[inst].list1);
			list_splice_tail_init(&isp->ibm[inst].list3, &isp->ibm[inst].list1);
			if (ins->mcm_ib) {
				list_add_tail(&ins->mcm_ib->entry, &isp->ibm[inst].list1);
				ins->mcm_ib = NULL;
			}
		}
	}

	ins->state = state;
	return 0;
}

int isp_set_ctx(struct isp_device *isp, u32 inst, struct isp_irq_ctx *ctx)
{
	struct isp_instance *ins;
	unsigned long flags;

	if (!isp || !ctx)
		return -EINVAL;

	if (inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[inst];
	spin_lock_irqsave(&ins->lock, flags);
	ins->ctx = *ctx;
	ins->ctx.src_buf_list1 = &ins->src_buf_list1;
	ins->ctx.src_buf_list2 = &ins->src_buf_list2;
	ins->ctx.src_buf_list3 = &ins->src_buf_list3;
	spin_unlock_irqrestore(&ins->lock, flags);
	return 0;
}

int isp_set_stream_idx(struct isp_device *isp, u32 inst, int idx)
{
	struct isp_instance *ins;

	if (!isp)
		return -EINVAL;

	if (inst >= isp->num_insts)
		return -EINVAL;

	if (idx >= ARRAY_SIZE(isp->stream_idx_mapping))
		return -EINVAL;

	ins = &isp->insts[inst];

	if (idx > -1)
		isp->stream_idx_mapping[idx] = inst;
	ins->stream_idx = idx;
	return 0;
}

int isp_add_job(struct isp_device *isp, u32 inst)
{
	struct isp_instance *ins;
	struct isp_irq_ctx *ctx;
	struct irq_job job = { inst };
	struct cam_list_node *node = NULL;
	int rc;

	if (isp->mode == STRM_WORK_MODE)
		return -EFAULT;

	rc = push_job(isp->jq, &job);
	if (rc < 0) {
//		dev_err(isp->dev, "failed to push a job(err=%d)\n", rc);
		return rc;
	}

	if (isp->error && !isp->insts[inst].ctx.is_sink_online_mode) {
		ctx = get_next_irq_ctx(isp);
		if (!ctx)
			return 0;
		ins = &isp->insts[isp->next_mi_irq_ctx];
		if (ctx->src_ctx) {
			node = list_first_entry_or_null(ctx->src_buf_list2,
							struct cam_list_node, entry);
			if (!node)
				return 0;
		}
		dev_dbg(isp->dev, "%s try to start isp\n", __func__);
#ifdef WITH_LEGACY_ISP
		if (node)
			isp_set_mp_buffer(isp, get_phys_addr(node->data, 0), &ins->fmt.ofmt);
		if (ctx->sink_buf) {
			isp_set_rdma_buffer(isp, get_phys_addr(ctx->sink_buf, 0));
			isp->error = 0;
		}
#else
		if (ctx->sink_buf) {
			struct isp_msg msg;

			memset(&msg, 0, sizeof(msg));
			msg.id = ISP_MSG_MCM_SCH;
			msg.inst = isp->next_mi_irq_ctx;
			if (node)
				msg.sch.mp_buf.addr = get_phys_addr(node->data, 0);
			msg.sch.rdma_buf.addr = get_phys_addr(ctx->sink_buf, 0);
			rc = isp_post(isp, &msg, false);
			if (rc < 0) {
				dev_warn(isp->dev, "%s failed to post MCM_SCH (err=%d).\n", __func__, rc);
				return rc;
			}
			isp->error = 0;
		}
#endif
		if (node) {
			list_del(&node->entry);
			list_add_tail(&node->entry, ctx->src_buf_list3);
		}
	}
	return 0;
}

static void isp_bound(struct isc_handle *isc, void *arg)
{
	struct isp_device *isp = (struct isp_device *)arg;
	unsigned long flags;

	if (isp) {
		spin_lock_irqsave(&isp->isc_lock, flags);
		if (!isp->isc) {
			isc_get(isc);
			isp->isc = isc;
		}
		spin_unlock_irqrestore(&isp->isc_lock, flags);
	}
}

static void isp_unbind(void *arg)
{
	struct isp_device *isp = (struct isp_device *)arg;
	unsigned long flags;

	if (isp) {
		spin_lock_irqsave(&isp->isc_lock, flags);
		if (isp->isc) {
			isc_put(isp->isc);
			isp->isc = NULL;
		}
		spin_unlock_irqrestore(&isp->isc_lock, flags);
	}
}

static struct isc_notifier_ops isp_notifier_ops = {
	.bound = isp_bound,
	.unbind = isp_unbind,
	.got = isp_msg_handler,
};

int isp_open(struct isp_device *isp, u32 inst)
{
	bool en_clk = false;
	int rc = 0;

	dev_dbg(isp->dev, "inst %d+\n", inst);

	if (!isp)
		return -EINVAL;

	mutex_lock(&isp->open_lock);
	if (refcount_read(&isp->open_cnt) == REFCNT_INIT_VAL)
		en_clk = true;
	refcount_inc(&isp->open_cnt);
	mutex_unlock(&isp->open_lock);

	if (en_clk) {
		pm_runtime_set_active(isp->dev);
		rc = pm_runtime_get_sync(isp->dev);
		if (rc < 0)
			return rc;
		rc = isp_runtime_resume(isp->dev);
		if (rc < 0)
			return rc;
	}

	dev_dbg(isp->dev, "inst %d-\n", inst);

	return rc;
}

int isp_close(struct isp_device *isp, u32 inst)
{
	struct isp_instance *ins;
	struct isp_msg msg;
	bool dis_clk = false;
	int i, rc = 0;

	dev_dbg(isp->dev, "inst %d+\n", inst);

	if (!isp)
		return -EINVAL;

	if (inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[inst];
	memset(&ins->fmt, 0, sizeof(ins->fmt));
	memset(&ins->in, 0, sizeof(ins->in));
	ins->input_bayer_format = 0;
	ins->error = 1;
	ins->hdr_en = false;
	INIT_LIST_HEAD(&ins->src_buf_list1);
	INIT_LIST_HEAD(&ins->src_buf_list2);
	INIT_LIST_HEAD(&ins->src_buf_list3);
	for (i = 0; i < ARRAY_SIZE(ins->src_bufs); i++)
		list_add_tail(&ins->src_bufs[i].entry, &ins->src_buf_list1);
	if (inst < ISP_SINK_ONLINE_PATH_MAX) {
		INIT_LIST_HEAD(&isp->ibm[inst].list1);
		INIT_LIST_HEAD(&isp->ibm[inst].list2);
		INIT_LIST_HEAD(&isp->ibm[inst].list3);
		if (isp->in_bufs[inst].size > 0) {
			mem_free(isp->dev, &isp->in_buf_list, &isp->in_bufs[inst]);
			isp->in_bufs[inst].size = 0;
		}
	}
	if (isp->hdr_bufs[inst].size > 0)
		mem_free(isp->dev, &isp->hdr_buf_list, &isp->hdr_bufs[inst]);

	msg.id = CAM_MSG_STATE_CHANGED;
	msg.inst = inst;
	msg.state = CAM_STATE_CLOSED;
	rc = isp_post(isp, &msg, true);
	if (rc < 0)
		dev_err(isp->dev, "failed to call isp_post (err=%d)\n", rc);

	mutex_lock(&isp->open_lock);
	if (refcount_read(&isp->open_cnt) > REFCNT_INIT_VAL) {
		refcount_dec(&isp->open_cnt);
		if (refcount_read(&isp->open_cnt) == REFCNT_INIT_VAL)
			dis_clk = true;
	}
	mutex_unlock(&isp->open_lock);

	if (!dis_clk) {
		dev_dbg(isp->dev, "inst %d close\n", inst);
		return 0;
	}

	reset_job_queue(isp->jq);

	isp_reset(isp);
	rc = isp_runtime_suspend(isp->dev);
	if (rc < 0)
		return rc;

	rc = pm_runtime_put_sync(isp->dev);
	if (rc < 0)
		return rc;

	dev_dbg(isp->dev, "inst %d-\n", inst);

	return 0;
}

int isp_probe(struct platform_device *pdev, struct isp_device *isp)
{
	struct device *dev = &pdev->dev;
	int rc;
	struct mem_res isp_mems[] = {
		{ "reg", NULL },
		{},
	};
	struct irq_res isp_irqs[] = {
		{ "isp", -1, isp_irq_handler, isp },
		{ "mi", -1, mi_irq_handler, isp },
		{ "fe", -1, fe_irq_handler, isp },
		{},
	};
	struct clk_res isp_clks[] = {
		{ "core", NULL },
		{ "axi", NULL },
		{ "mcm", NULL },
		{ "hclk", NULL },
		{},
	};
	struct rst_res isp_rsts[] = {
		{"rst", NULL },
		{},
	};
	struct cam_dt isp_dt = {
		.id = 0,
		.num_insts = 0,
		.mems = isp_mems,
		.irqs = isp_irqs,
		.clks = isp_clks,
		.rsts = isp_rsts,
	};
	u32 i, j;

	if (!isp)
		return -EINVAL;

	rc = parse_cam_dt(pdev, &isp_dt, isp);
	if (rc < 0) {
		dev_err(dev, "failed to call parse_cam_dt (err=%d)\n", rc);
		return rc;
	}

	isp->dev = dev;
	isp->id = isp_dt.id;
	isp->num_insts = isp_dt.num_insts;
	isp->base = isp_dt.mems[0].base;
	isp->core = isp_dt.clks[0].clk;
	isp->axi = isp_dt.clks[1].clk;
	isp->mcm = isp_dt.clks[2].clk;
	isp->hclk = isp_dt.clks[3].clk;
	isp->rst = isp_dt.rsts[0].rst;
	spin_lock_init(&isp->isc_lock);
	refcount_set(&isp->set_state_refcnt, REFCNT_INIT_VAL);
	isp->error = 1;
	mutex_init(&isp->open_lock);
	mutex_init(&isp->set_input_lock);
	refcount_set(&isp->open_cnt, REFCNT_INIT_VAL);

	isp->insts = devm_kcalloc(dev, isp_dt.num_insts, sizeof(*isp->insts),
				  GFP_KERNEL);
	if (!isp->insts)
		return -ENOMEM;

	isp->ctrl_dev = get_cam_ctrl_device(pdev);
	if (IS_ERR(isp->ctrl_dev))
		return PTR_ERR(isp->ctrl_dev);

	isp->jq = create_job_queue(32);
	if (!isp->jq) {
		dev_err(dev, "failed to call create_job_queue\n");
		return -ENOMEM;
	}

	rc = isc_register(ISP_UID(isp->id), &isp_notifier_ops, isp);
	if (rc < 0) {
		dev_err(dev, "failed to call isc_register (err=%d)\n", rc);
		destroy_job_queue(isp->jq);
		return rc;
	}

	for (i = 0; i < isp_dt.num_insts; i++) {
		spin_lock_init(&isp->insts[i].lock);
		INIT_LIST_HEAD(&isp->insts[i].src_buf_list1);
		INIT_LIST_HEAD(&isp->insts[i].src_buf_list2);
		INIT_LIST_HEAD(&isp->insts[i].src_buf_list3);
		for (j = 0; j < ARRAY_SIZE(isp->insts[i].src_bufs); j++)
			list_add_tail(&isp->insts[i].src_bufs[j].entry,
				      &isp->insts[i].src_buf_list1);
	}

	for (i = 0; i < ISP_SINK_ONLINE_PATH_MAX; i++) {
		isp->stream_idx_mapping[i] = -1;
		INIT_LIST_HEAD(&isp->ibm[i].list1);
		INIT_LIST_HEAD(&isp->ibm[i].list2);
		INIT_LIST_HEAD(&isp->ibm[i].list3);
	}

	INIT_LIST_HEAD(&isp->in_buf_list);
	INIT_LIST_HEAD(&isp->hdr_buf_list);
	isp->unit_test = false;

	pm_runtime_enable(isp->dev);
	if (pm_runtime_active(isp->dev)) {
		rc = pm_runtime_put_sync(isp->dev);
		if (rc < 0)
			return rc;
	}

	dev_dbg(dev, "VS ISP driver (base) probed done\n");
	return 0;
}

int isp_remove(struct platform_device *pdev, struct isp_device *isp)
{
	int rc;

	rc = isc_unregister(ISP_UID(isp->id));
	if (rc < 0)
		dev_err(&pdev->dev, "failed to call isc_unregister (err=%d)\n",
			rc);

	destroy_job_queue(isp->jq);
	mem_free_all(isp->dev, &isp->in_buf_list);
	mem_free_all(isp->dev, &isp->hdr_buf_list);
	put_cam_ctrl_device(isp->ctrl_dev);
	pm_runtime_disable(isp->dev);

	dev_dbg(&pdev->dev, "VS ISP driver (base) removed\n");
	return rc;
}

void isp_reset(struct isp_device *isp)
{
	if (isp->rst) {
		reset_control_assert(isp->rst);
		udelay(2);
		reset_control_deassert(isp->rst);
	}
}

#ifdef CONFIG_DEBUG_FS
static ssize_t isp_debugfs_log_write(struct file *f, const char __user *buf,
				     size_t size, loff_t *pos)
{
	struct isp_device *isp = f->f_inode->i_private;
	char cmd[64], *str = cmd, *token;
	struct isp_msg msg;
	int rc;

	if (!size || size >= sizeof(cmd))
		return -EINVAL;

	rc = strncpy_from_user(cmd, buf, size);
	if (rc < 0)
		return rc;

	cmd[size] = '\0';
	memset(&msg, 0, sizeof(msg));
	msg.id = CAM_MSG_LOG_STATE_CHANGED;
	token = strsep(&str, " ");
	while (token) {
		msg.log.level = simple_strtoul(token, NULL, 10);
		pr_debug("%s [%d,%d]\n", __func__, msg.log.id, msg.log.level);
		rc = isp_post(isp, &msg, true);
		if (rc < 0) {
			pr_err("failed to post log state changed msg (err=%d)\n", rc);
			break;
		}
		msg.log.id++;
		token = strsep(&str, " ");
	}
	return size;
}

static const struct file_operations isp_debugfs_log_fops = {
	.owner  = THIS_MODULE,
	.write  = isp_debugfs_log_write,
	.llseek = seq_lseek,
};

static ssize_t isp_debugfs_tune_write(struct file *f, const char __user *buf,
				      size_t size, loff_t *pos)
{
	struct isp_device *isp = f->f_inode->i_private;
	char cmd[4];
	struct isp_msg msg;
	int rc;

	if (!size || size >= sizeof(cmd))
		return -EINVAL;

	rc = strncpy_from_user(cmd, buf, size);
	if (rc < 0)
		return rc;

	cmd[size] = '\0';
	memset(&msg, 0, sizeof(msg));
	msg.id = ISP_MSG_TUNE_EN;
	msg.tune_enabled = simple_strtoul(cmd, NULL, 10);
	pr_debug("%s [%d]\n", __func__, msg.tune_enabled);
	rc = isp_post(isp, &msg, true);
	if (rc < 0)
		pr_err("failed to post tune enabled msg (err=%d)\n", rc);
	return size;
}

static const struct file_operations isp_debugfs_tune_fops = {
	.owner  = THIS_MODULE,
	.write  = isp_debugfs_tune_write,
	.llseek = seq_lseek,
};

void isp_debugfs_init(struct isp_device *isp)
{
	if (!isp->debugfs_dir)
		isp->debugfs_dir = debugfs_create_dir("isp", NULL);
	if (!isp->debugfs_dir)
		return;
	if (!isp->debugfs_log_file)
		isp->debugfs_log_file = debugfs_create_file
				("log", 0222, isp->debugfs_dir, isp,
				&isp_debugfs_log_fops);
	if (!isp->debugfs_tune_file)
		isp->debugfs_tune_file = debugfs_create_file
				("tune", 0222, isp->debugfs_dir, isp,
				&isp_debugfs_tune_fops);
}

void isp_debugfs_remo(struct isp_device *isp)
{
	if (isp->debugfs_dir) {
		debugfs_remove_recursive(isp->debugfs_dir);
		isp->debugfs_dir = NULL;
		isp->debugfs_log_file = NULL;
		isp->debugfs_tune_file = NULL;
	}
}
#endif

#ifdef CONFIG_PM_SLEEP
int isp_system_suspend(struct device *dev)
{
	return pm_runtime_force_suspend(dev);
}

int isp_system_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}
#endif

#ifdef CONFIG_PM
int isp_runtime_suspend(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	if (isp->mcm)
		clk_disable_unprepare(isp->mcm);
	if (isp->core)
		clk_disable_unprepare(isp->core);
	if (isp->axi)
		clk_disable_unprepare(isp->axi);
	if (isp->hclk)
		clk_disable_unprepare(isp->hclk);
	return 0;
}

int isp_runtime_resume(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	int rc;

	if (isp->axi) {
		rc = clk_prepare_enable(isp->axi);
		if (rc)
			return rc;
	}
	if (isp->core) {
		rc = clk_prepare_enable(isp->core);
		if (rc)
			goto _core_err;
	}
	if (isp->mcm) {
		rc = clk_prepare_enable(isp->mcm);
		if (rc)
			goto _mcm_err;
	}
	if (isp->hclk) {
		rc = clk_prepare_enable(isp->hclk);
		if (rc)
			goto _hclk_err;
	}
	return 0;
_hclk_err:
	if (isp->mcm)
		clk_disable_unprepare(isp->mcm);
_mcm_err:
	if (isp->core)
		clk_disable_unprepare(isp->core);
_core_err:
	if (isp->axi)
		clk_disable_unprepare(isp->axi);
	return rc;
}
#endif
