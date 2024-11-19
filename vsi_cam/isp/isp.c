// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "[isp_drv]: %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/of_reserved_mem.h>

#include "cam_ctrl.h"
#include "cam_dev.h"
#include "cam_ctx.h"
#include "isc.h"
#include "isp_uapi.h"

#include "isp.h"
#include "isp8000_regs.h"

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

static int _isp_post(struct isp_device *isp, struct isp_msg *msg,
		     bool sync, int *result)
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
	if (result)
		*result = param.rc;
	return rc;
}

int isp_post(struct isp_device *isp, struct isp_msg *msg, bool sync)
{
	return _isp_post(isp, msg, sync, NULL);
}

int isp_post_ex(struct isp_device *isp, struct isp_msg *msg,
		struct mem_buf *extra, bool sync, int *result)
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
	if (result)
		*result = param.rc;
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
	int ret, result;

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
		ret = _isp_post(isp, &msg, true, &result);
		if (ret < 0 || result) {
			ret |= result;
			pr_info("%s: msg isp_post failed (err=%d)!\n", __func__, ret);
		}
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
		ret = isp_post_ex(isp, &msg, &msg.ctrl_ext.buf, true, &result);
		if (ret < 0 || result) {
			ret |= result;
			pr_info("%s: msg isp_post_ex failed (err=%d)!\n", __func__, ret);
		}

		isc_free_extra_buf(isp->isc, &msg.ctrl_ext.buf);
	}

	return ret;
}

int isp_get_subctrl(struct isp_device *isp, u32 inst, u32 cmd, void *data, u32 size)
{
	struct isp_msg msg;
	void *buf_va;
	int ret, result;

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
		ret = _isp_post(isp, &msg, true, &result);
		if (ret < 0 || result) {
			ret |= result;
			pr_info("%s: msg isp_post failed (err=%d)!\n", __func__, ret);
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
		ret = isp_post_ex(isp, &msg, &msg.ctrl_ext.buf, true, &result);
		if (ret < 0 || result) {
			ret |= result;
			pr_info("%s: msg isp_post_ex failed (err=%d)!\n", __func__, ret);
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
	u32 size, i, rc;
	struct mem_buf *buf;

	size = fmt->stride * fmt->height;
	if (!size)
		return -EINVAL;
	for (i = 0; i < MCM_BUF_NUM; i++) {
		buf = &isp->in_bufs[id][i];
		if (buf->size > 0 && buf->size != size) {
			rc = mem_free(isp->dev, &isp->in_buf_list, buf);
			if (unlikely(rc)) {
				pr_err("mem_free fail, (err=%d)\n", rc);
				return rc;
			}
		}
		if (buf->size != size) {
			buf->size = size;
			rc = mem_alloc(isp->dev, &isp->in_buf_list, buf);
			if (unlikely(rc)) {
				pr_err("mem_alloc fail, (err=%d)\n", rc);
				return rc;
			}

			isp->ib[id][i].buf.addr = buf->addr;
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
	int rc;

	size = fmt->stride * fmt->height;
	if (!size)
		return -EINVAL;

	buf = &isp->hdr_bufs[id];
	if (buf->size > 0 && buf->size != size) {
		rc = mem_free(isp->dev, &isp->hdr_buf_list, buf);
		if (unlikely(rc)) {
			pr_err("mem_free fail, (err=%d)\n", rc);
			return rc;
		}
	}
	if (buf->size != size) {
		buf->size = size;
		rc = mem_alloc(isp->dev, &isp->hdr_buf_list, buf);
		if (unlikely(rc)) {
			pr_err("mem_alloc fail, (err=%d)\n", rc);
			return rc;
		}
	}
	return 0;
}

int isp_set_format(struct isp_device *isp, u32 inst, struct isp_format *fmt)
{
	struct isp_instance *ins;
	struct isp_msg msg;
	int rc;

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

		for (i = 0; i < HDR_BUF_NUM; i++) {
			rc = alloc_hdr_buf(isp, i, &fmt->ifmt);
			if (unlikely(rc)) {
				mem_free_all(isp->dev, &isp->hdr_buf_list);
				pr_err("inst %d alloc_hdr_buf fail, (err=%d)\n", inst, rc);
				return rc;
			}
		}
	}

	if (ins->online_mcm && ins->stream_idx > -1) {
		rc = alloc_mcm_buf(isp, inst, &fmt->ifmt);
		if (unlikely(rc)) {
			pr_err("inst %d alloc_mcm_buf fail, (err=%d)\n", inst, rc);
			return rc;
		}
	}

	memcpy(&ins->fmt, fmt, sizeof(ins->fmt));
	msg.id = CAM_MSG_FORMAT_CHANGED;
	msg.inst = inst;
	memcpy(&msg.fmt, fmt, sizeof(msg.fmt));
	return isp_post(isp, &msg, true);
}

void isp_set_mcm_buffer(struct isp_device *isp, u32 path, phys_addr_t phys_addr)
{
	u32 base = MI_MCMn_RAW_BASE(path);

	isp_write(isp, MI_MCMn_RAW_ADDR(base), phys_addr);
}

static inline void isp_set_mcm_raw_buffer(struct isp_device *isp, u32 path,
					  phys_addr_t phys_addr, struct cam_format *fmt,
					  bool aligned)
{
	u32 base = MI_MCMn_RAW_BASE(path);
	u32 size;

	isp_write(isp, MI_MCMn_RAW_ADDR(base), phys_addr);
	/* configure mcm raw buffer size, unaligned by default. */
	switch (fmt->format) {
	case CAM_FMT_RAW10:
		if (aligned)
			size = fmt->width * fmt->height * 2;
		else
			size = fmt->width * fmt->height * 10 / 8;
		break;
	case CAM_FMT_RAW12:
		if (aligned)
			size = fmt->width * fmt->height * 2;
		else
			size = fmt->width * fmt->height * 12 / 8;
		break;
	default:
		size = fmt->stride * fmt->height;
		break;
	}
	isp_write(isp, MI_MCMn_RAW_SIZE(base), size);
	isp_write(isp, MI_MCMn_RAW_OFFS(base), 0x0);
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

	dev_dbg(isp->dev, "stride %d, height %d, phys_addr %llx\n", fmt->stride, fmt->height, phys_addr);
}

int isp_set_state(struct isp_device *isp, u32 inst, int state, enum group_type type)
{
	struct isp_msg msg;
	struct isp_instance *ins;
	phys_addr_t phys_addr;
	struct isp_mcm_sch sch;
	int rc, i;
	u32 value;
	bool state_check = false;
	struct ibuf *ib = NULL;
	struct isp_irq_ctx *ctx;
	unsigned long flags, flags2;
	struct cam_list_node *node = NULL;

	if (!isp || inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[inst];
	mutex_lock(&isp->set_state_lock);
	spin_lock_irqsave(&isp->sch.lock, flags);
	if (state != CAM_STATE_STARTED)
		ins->state = state;
	spin_unlock_irqrestore(&isp->sch.lock, flags);

	switch (state) {
	case CAM_STATE_INITED:
		ins->last_frame_done = 0;
		ins->frame_interval = 0;
		ins->frame_count = 0;
		break;
	case CAM_STATE_CLOSED:
		if (isp->mode != ISP_STRM_MODE) {
			isp_remove_job(isp, inst);
			isp_reset_schedule(isp, inst, false);
		}
		break;
	case CAM_STATE_STARTED:
		if (ins->hdr_en) {
			for (i = 0; i < HDR_BUF_NUM; i++) {
				phys_addr = isp->hdr_bufs[i].addr;
				isp_set_hdr_raw_buffer(isp, i, phys_addr, &ins->fmt.ifmt);
			}
		}

		if (isp->mode == ISP_STRM_MODE) {
			spin_lock_irqsave(&ins->lock, flags2);
			ctx = &ins->ctx;
			rc = new_frame(ctx);
			spin_unlock_irqrestore(&ins->lock, flags2);
			if (rc) {
				rc = -ENOMEM;
				goto _exit;
			} else {
				if (has_offline(ctx->is_src_online_mode)) {
					node = list_first_entry_or_null(ctx->src_buf_list2,
									struct cam_list_node, entry);
					if (!node) {
						rc = -EINVAL;
						goto _exit;
					}
					isp_set_mp_buffer(isp, get_phys_addr(isp->dev, node->data, 0),
							  &ins->fmt.ofmt);
					/* force update */
					value = isp_read(isp, MI_MP_CTRL);
					value |= 0x38;
					isp_write(isp, MI_MP_CTRL, value);
					isp_set_schedule(isp, &sch, 0, 0, false);
					list_del(&node->entry);
					list_add_tail(&node->entry, ctx->src_buf_list3);
					ins->shd_src_node = node;
				} else {
					isp_set_mp_buffer(isp, 0, &ins->fmt.ofmt);
					/* force update */
					value = isp_read(isp, MI_MP_CTRL);
					value |= 0x38;
					isp_write(isp, MI_MP_CTRL, value);
					isp_set_schedule(isp, &sch, 0, 0, false);
				}
			}

			spin_lock_irqsave(&ins->lock, flags2);
			rc = new_frame(ctx);
			spin_unlock_irqrestore(&ins->lock, flags2);
			if (rc) {
				if (ins->shd_src_node) {
					list_del(&ins->shd_src_node->entry);
					list_add_tail(&ins->shd_src_node->entry, ctx->src_buf_list1);
					ins->shd_src_node = NULL;
				}
				rc = -ENOMEM;
				goto _exit;
			} else {
				if (has_offline(ctx->is_src_online_mode)) {
					node = list_first_entry_or_null(ctx->src_buf_list2,
									struct cam_list_node, entry);
					if (!node) {
						if (ins->shd_src_node) {
							list_del(&ins->shd_src_node->entry);
							list_add_tail(&ins->shd_src_node->entry, ctx->src_buf_list1);
							ins->shd_src_node = NULL;
						}
						rc = -EINVAL;
						goto _exit;
					}
					list_del(&node->entry);
					list_add_tail(&node->entry, ctx->src_buf_list3);
					ins->src_node = node;
				}
			}
		} else {
			if (ins->online_mcm && ins->stream_idx > -1) {
				ib = list_first_entry_or_null(&isp->ibm[inst].list1,
							      struct ibuf, entry);
				if (!ib) {
					rc = -ENOMEM;
					goto _exit;
				}
				isp_set_mcm_raw_buffer(isp, ins->stream_idx, ib->buf.addr,
						       &ins->fmt.ifmt, ins->hdr_en || ins->tile_en);
				ins->mcm_ib = ib;
				list_del(&ib->entry);

				/* force update */
				switch (ins->stream_idx) {
				case 0:
					value = isp_read(isp, MI_MCM_CTRL);
					value |= 0x19;
					isp_write(isp, MI_MCM_CTRL, value);
					value |= BIT(2);
					isp_write(isp, MI_MCM_CTRL, value);
					break;
				case 1:
					value = isp_read(isp, MI_MCM_CTRL);
					value |= 0x19;
					isp_write(isp, MI_MCM_CTRL, value);
					value |= BIT(7);
					isp_write(isp, MI_MCM_CTRL, value);
					break;
				case 2:
					value = isp_read(isp, MI_MCM_G2_CTRL);
					value |= 0x19;
					isp_write(isp, MI_MCM_G2_CTRL, value);
					value |= BIT(2);
					isp_write(isp, MI_MCM_G2_CTRL, value);
					break;
				case 3:
					value = isp_read(isp, MI_MCM_G2_CTRL);
					value |= 0x19;
					isp_write(isp, MI_MCM_G2_CTRL, value);
					value |= BIT(5);
					isp_write(isp, MI_MCM_G2_CTRL, value);
					break;
				default:
					break;
				}

				ib = list_first_entry_or_null(&isp->ibm[inst].list1,
							      struct ibuf, entry);
				if (!ib) {
					list_add_tail(&ins->mcm_ib->entry, &isp->ibm[inst].list1);
					rc = -ENOMEM;
					goto _exit;
				}
			}

			state_check = false;
			for (i = 0; i < isp->num_insts; i++) {
				if (i == inst)
					continue;
				if (isp->insts[i].state == CAM_STATE_STARTED) {
					state_check = true;
					break;
				}
			}
			if (!state_check)
				isp_reset_schedule(isp, INVALID_INST, false);
		}

		refcount_inc(&isp->set_state_refcnt);
		//trigger vse
		if (isp->mode == ISP_STRM_MODE) {
			for (i = 0; i < ISP_OUT_CHNL_MAX; i++) {
				if (is_online(ins->ctx.is_src_online_mode, i)) {
					pr_debug("%s isp%d trigger vse...\n", __func__, inst);
					cam_trigger(ins->ctx.src_ctx[i]);
				}
			}
		}
		break;
	case CAM_STATE_STOPPED:
		if (refcount_read(&isp->set_state_refcnt) > REFCNT_INIT_VAL)
			refcount_dec(&isp->set_state_refcnt);

		list_splice_tail_init(&ins->src_buf_list2, &ins->src_buf_list1);
		list_splice_tail_init(&ins->src_buf_list3, &ins->src_buf_list1);
		// for stream mode, disable all buffer operation directly
		if (isp->mode == ISP_STRM_MODE) {
			isp_set_schedule(isp, &sch, 0, 0, false);
		} else {
			isp_remove_job(isp, inst);
			if (ins->online_mcm) {
				list_splice_tail_init(&isp->ibm[inst].list2, &isp->ibm[inst].list1);
				list_splice_tail_init(&isp->ibm[inst].list3, &isp->ibm[inst].list1);
				if (ins->mcm_ib) {
					list_add_tail(&ins->mcm_ib->entry, &isp->ibm[inst].list1);
					ins->mcm_ib = NULL;
				}
				if (ins->mcm_ib1) {
					list_add_tail(&ins->mcm_ib1->entry, &isp->ibm[inst].list1);
					ins->mcm_ib1 = NULL;
				}
			}
		}
		break;
	default:
		rc = -EINVAL;
		goto _exit;
	}
	pr_debug("%s: isp(%d) post STATE_CHANGED:%d\n", __func__, inst, state);
	msg.id = CAM_MSG_STATE_CHANGED;
	msg.inst = inst;
	msg.state = state;
	msg.group = type;
	rc = isp_post(isp, &msg, true);

	if (state == CAM_STATE_STARTED) {
		if (isp->mode != ISP_STRM_MODE) {
			if (ib) {
				isp_set_mcm_buffer(isp, ins->stream_idx, ib->buf.addr);
				ins->mcm_ib1 = ib;
				list_del(&ib->entry);
			}
			ins->tile_count = 0;
		} else {
			if (has_offline(ctx->is_src_online_mode)) {
				if (ins->src_node)
					isp_set_mp_buffer(isp, get_phys_addr(isp->dev, ins->src_node->data, 0),
							  &ins->fmt.ofmt);
			} else {
				isp_set_mp_buffer(isp, 0, &ins->fmt.ofmt);
			}
		}
		ins->state = CAM_STATE_STARTED;
	}

_exit:
	mutex_unlock(&isp->set_state_lock);
	return rc;
}

int isp_get_ctx(struct isp_device *isp, u32 inst, struct isp_irq_ctx *ctx)
{
	struct isp_instance *ins;
	unsigned long flags;

	if (!isp || !ctx)
		return -EINVAL;

	if (inst >= isp->num_insts)
		return -EINVAL;

	ins = &isp->insts[inst];
	spin_lock_irqsave(&ins->lock, flags);
	*ctx = ins->ctx;
	spin_unlock_irqrestore(&ins->lock, flags);
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

	ins = &isp->insts[inst];

	if (ins->online_mcm && idx > -1)
		isp->stream_idx_mapping[idx] = inst;
	ins->stream_idx = idx;
	return 0;
}

int isp_add_job(struct isp_device *isp, u32 inst)
{
	struct irq_job job = { inst };
	int rc = 0;
	struct isp_instance *ins;
	struct isp_mcm_sch sch;

	if (!isp)
		return -EINVAL;

	if (isp->mode == ISP_STRM_MODE)
		return -EFAULT;

	if (WARN_ON(inst >= isp->num_insts))
		return -EFAULT;

	ins = &isp->insts[inst];
	if (ins->state != CAM_STATE_STARTED) {
		// dev_err(isp->dev, "instance %d is not in CAM_STATE_STARTED!\n", inst);
		return -EFAULT;
	}

	rc = push_job(isp->jq, &job);
	if (rc < 0) {
		// dev_err(isp->dev, "failed to push a job %d (err=%d)\n", inst, rc);
		return rc;
	}

	if (!ins->online_mcm) {
		sch.id = inst;
		isp_set_schedule(isp, &sch, 0, 0, false);
	}

	return 0;
}

int isp_fetch_job(struct isp_device *isp, u32 *inst)
{
	struct irq_job job;
	int rc = 0;

	if (!isp || !inst)
		return -EINVAL;

	if (isp->mode == ISP_STRM_MODE)
		return -EFAULT;

	rc = pop_job(isp->jq, &job);
	if (rc < 0) {
		// dev_err(isp->dev, "failed to pop a job (err=%d)\n", rc);
		*inst = INVALID_INST;
		return rc;
	}

	if (WARN_ON(job.irq_ctx_index >= isp->num_insts)) {
		*inst = INVALID_INST;
		return -1;
	}

	*inst = job.irq_ctx_index;
	return 0;
}

int isp_remove_job(struct isp_device *isp, u32 inst)
{
	struct irq_job job = { inst };
	int rc;

	if (!isp)
		return -EINVAL;

	if (isp->mode == ISP_STRM_MODE)
		return -EFAULT;

	if (WARN_ON(inst >= isp->num_insts))
		return -EFAULT;

	rc = remove_job(isp->jq, &job);
	if (rc < 0) {
//		dev_err(isp->dev, "failed to remove job for inst %d (err=%d)\n", inst, rc);
		return rc;
	}

	return 0;
}

int isp_query_job(struct isp_device *isp, u32 *inst)
{
	struct irq_job job;
	int rc = 0;

	if (!isp || !inst)
		return -EINVAL;

	if (isp->mode == ISP_STRM_MODE)
		return -EFAULT;

	rc = query_job(isp->jq, &job);
	if (rc < 0) {
		// dev_err(isp->dev, "failed to query a job (err=%d)\n", rc);
		*inst = INVALID_INST;
		return rc;
	}

	if (WARN_ON(job.irq_ctx_index >= isp->num_insts)) {
		*inst = INVALID_INST;
		return -1;
	}

	*inst = job.irq_ctx_index;
	return 0;
}

static int isp_set_schedule_online_stream(struct isp_device *isp, bool isp_irq_call)
{
	struct isp_instance *ins;
	struct isp_irq_ctx *ctx;
	struct cam_list_node *node = NULL;
	int rc;
	unsigned long flags;

	if (!isp)
		return -EINVAL;

	if (!isp_irq_call) {
		isp->error = 0;
		isp->sch.mi_idle = false;
		return 0;
	}

	if (!isp->sch.mi_idle)
		return 0;

	ins = &isp->insts[0];
	if (ins->state != CAM_STATE_STARTED)
		return 0;
	spin_lock_irqsave(&ins->lock, flags);
	ctx = &ins->ctx;
	rc = new_frame(ctx);
	spin_unlock_irqrestore(&ins->lock, flags);
	if (has_offline(ctx->is_src_online_mode)) {
		if (!rc)
			ins->shd_src_node = ins->src_node;
		// if shd_src_node failed to update (there is no more mp buffer), use
		// src_node to update shd_src_node
		if (!ins->shd_src_node) {
			ins->shd_src_node = ins->src_node;
			node = ins->src_node;
		} else {
			node = list_first_entry_or_null(ctx->src_buf_list2,
							struct cam_list_node, entry);
			if (node) {
				list_del(&node->entry);
				list_add_tail(&node->entry, ctx->src_buf_list3);
				pr_debug("isp list_add_tail src_buf_list3\n");
				ins->src_node = node;
			}
		}
		if (!node) {
			pr_debug("isp fail to get valid node!\n");
			return 0;
		}
		isp_set_mp_buffer(isp, get_phys_addr(isp->dev, node->data, 0), &ins->fmt.ofmt);
	} else {
		isp_set_mp_buffer(isp, 0, &ins->fmt.ofmt);
	}

	isp->error = 0;
	isp->sch.mi_idle = false;
	return 0;
}

static int isp_set_schedule_online_mcm(struct isp_device *isp, struct isp_mcm_sch *sch)
{
	struct isp_msg msg;
	struct isp_irq_ctx *ctx;
	struct isp_instance *ins;
	struct cam_list_node *node = NULL;
	struct ibuf *mcm_ib = NULL;
	u32 i, inst;
	int rc = 0;

	if (!isp || !sch)
		return -EINVAL;

	inst = sch->id;
	ins = &isp->insts[inst];
	ctx = &ins->ctx;
	mcm_ib = list_first_entry_or_null(&isp->ibm[inst].list2, struct ibuf, entry);
	if (mcm_ib) {
		if (has_offline(ctx->is_src_online_mode)) {
			node = list_first_entry_or_null(ctx->src_buf_list2, struct cam_list_node,
							entry);
			if (node) {
				list_del(&node->entry);
				list_add_tail(&node->entry, ctx->src_buf_list3);
				pr_debug("isp list_add_tail src_buf_list3\n");
			} else {
				pr_debug("isp online job fail to get src buf node!\n");
				return -EFAULT;
			}

			sch->mp_buf.mem.addr = get_phys_addr(isp->dev, node->data, 0);
			sch->mp_buf.mem.size = 0;
		} else {
			sch->mp_buf.mem.addr = 0;
			sch->mp_buf.mem.size = 0;
		}
		memcpy(&sch->mp_buf.fmt, &ins->fmt.ofmt, sizeof(sch->mp_buf.fmt));
		sch->mp_buf.valid = 1;

		sch->rdma_buf.mem.addr = mcm_ib->buf.addr;
		sch->rdma_buf.mem.size = mcm_ib->buf.size;
		memcpy(&sch->rdma_buf.fmt, &ins->fmt.ifmt, sizeof(sch->rdma_buf.fmt));
		sch->rdma_buf.valid = 1;
		sch->hdr_en = ins->hdr_en ? 1 : 0;
		sch->tile_en = ins->tile_en ? 1 : 0;
		sch->online_mcm = ins->online_mcm;
		list_del(&mcm_ib->entry);
		list_add_tail(&mcm_ib->entry, &isp->ibm[inst].list3);
	} else {
		pr_debug("isp online job fail to get mcm ib node!\n");
		return -EFAULT;
	}

	for (i = 0; i < ISP_OUT_CHNL_MAX; i++) {
		if (is_online(ins->ctx.is_src_online_mode, i))
			cam_trigger(ins->ctx.src_ctx[i]);
	}

	isp->error = 0;
	isp->sch.next_mi_inst = inst;
	isp->sch.mi_idle = false;
	isp->sch.frame_done_mask = ISP_SW_FRAME_DONE;
	pr_debug("isp online mcm_sch inst: %d\n", sch->id);
	memset(&msg, 0, sizeof(msg));
	msg.id = ISP_MSG_MCM_SCH;
	msg.inst = sch->id;
	memcpy(&msg.sch, sch, sizeof(msg.sch));
	rc = isp_post(isp, &msg, false);
	return rc;
}

static int isp_set_schedule_offline_mcm(struct isp_device *isp, struct isp_mcm_sch *sch,
					bool isp_irq_call)
{
	struct isp_instance *ins;
	struct isp_irq_ctx *ctx;
	struct cam_list_node *node = NULL;
	struct isp_msg msg;
	int rc = 0;
	u32 i, inst;

	if (!isp || !sch)
		return -EINVAL;

	inst = sch->id;
	ins = &isp->insts[inst];
	ctx = &ins->ctx;
	if (!isp_irq_call) {
		rc = new_frame(ctx);
		if (rc)
			return -1;
	}

	if (has_offline(ctx->is_src_online_mode)) {
		node = list_first_entry_or_null(ctx->src_buf_list2, struct cam_list_node,
						entry);
		if (!node)
			return -1;
	}
	if (ctx->sink_buf) {
		if (node) {
			sch->mp_buf.mem.addr = get_phys_addr(isp->dev, node->data, 0);
			pr_debug("%s: isp list_add_tail src_buf_list3\n", __func__);
			list_del(&node->entry);
			list_add_tail(&node->entry, ctx->src_buf_list3);
		} else if (!has_offline(ctx->is_src_online_mode)) {
			sch->mp_buf.mem.addr = 0;
		} else {
			pr_err("%s: invalid node!\n", __func__);
			return -1;
		}
		sch->mp_buf.mem.size = 0;
		memcpy(&sch->mp_buf.fmt, &ins->fmt.ofmt, sizeof(struct cam_format));
		sch->mp_buf.valid = 1;
		sch->rdma_buf.mem.addr = get_phys_addr(isp->dev, ctx->sink_buf, 0);
		sch->rdma_buf.mem.size = 0;
		memcpy(&sch->rdma_buf.fmt, &ins->fmt.ifmt, sizeof(struct cam_format));
		sch->rdma_buf.valid = 1;
		sch->hdr_en = ins->hdr_en ? 1 : 0;
		sch->tile_en = ins->tile_en ? 1 : 0;
		sch->online_mcm = ins->online_mcm;

		for (i = 0; i < ISP_OUT_CHNL_MAX; i++) {
			if (is_online(ctx->is_src_online_mode, i))
				cam_trigger(ctx->src_ctx[i]);
		}

		isp->error = 0;
		isp->sch.next_mi_inst = sch->id;
		isp->sch.mi_idle = false;
		isp->sch.frame_done_mask = ISP_SW_FRAME_DONE;
		pr_debug("isp offline mcm_sch inst: %d\n", sch->id);
		memset(&msg, 0, sizeof(msg));
		msg.id = ISP_MSG_MCM_SCH;
		msg.inst = sch->id;
		memcpy(&msg.sch, sch, sizeof(msg.sch));
		rc = isp_post(isp, &msg, false);
	}

	return rc;
}

int isp_set_schedule(struct isp_device *isp, struct isp_mcm_sch *sch, u32 miv2_mis,
		     u32 isp_mis, bool isp_irq_call)
{
	struct isp_instance *ins;
	struct isp_irq_ctx *ctx;
	u32 inst, id = INVALID_INST;
	int rc = 0;
	unsigned long flags;

	if (!isp || !sch)
		return -EINVAL;

	spin_lock_irqsave(&isp->sch.lock, flags);
	if (isp->mode == ISP_STRM_MODE) {
		isp_set_schedule_online_stream(isp, isp_irq_call);
		goto _exit;
	}
	// judge frame end for MCM job (online/offline)
	if ((miv2_mis & BIT(0) || miv2_mis & BIT(24) || isp_mis & BIT(1)) &&
		isp->sch.frame_done_mask) {
		pr_debug("miv2_mis: 0x%x, isp_mis: 0x%x, frame_done_mask: 0x%x\n", miv2_mis,
			 isp_mis, isp->sch.frame_done_mask);
		if (miv2_mis & BIT(0))
			isp->sch.frame_done_mask &= ~ISP_MP_FRAME_END;
		if (miv2_mis & BIT(24))
			isp->sch.frame_done_mask &= ~ISP_RDMA_END;
		if (isp_mis & BIT(1))
			isp->sch.frame_done_mask &= ~ISP_MIS_FRAME_END;
		if (!isp->sch.frame_done_mask) {
			id = isp->sch.next_mi_inst;
			ins = &isp->insts[id];
			/**
			 * For tile mode, if tile count is not equal to target tile count, update
			 * frame done mask and just return. We can consider 1 tile frame is really
			 * done IF AND ONLY IF the number of times frame_done_mask completely being
			 * reset is equal to target tile count!
			 **/
			if (ins->tile_en) {
				if (ins->tile_count == TILE_COUNT) {
					ins->tile_count = 0;
					pr_debug("tile mode frame done!\n");
				} else {
					isp->sch.frame_done_mask = ISP_SW_FRAME_DONE;
					goto _exit;
				}
			}

			// frame done
			pr_debug("mcm_sch inst: %d, frame done!\n", isp->cur_mi_irq_ctx);
			{
				struct isp_msg msg;

				memset(&msg, 0, sizeof(msg));
				msg.id = ISP_MSG_FRAME_DONE;
				msg.inst = isp->cur_mi_irq_ctx;
				isp_post(isp, &msg, false);
				pr_debug("post frame end inst:%d", inst);
			}
			isp->sch.mi_idle = true;
			isp->sch.next_mi_inst = INVALID_INST;
		}
	}
	// if MI is busy or frame done mask has not been totally reset, just skip schedule
	if (!isp->sch.mi_idle) {
		rc = -1;
		goto _exit;
	}
	// start next isp schedule
	if (isp_irq_call) {
		ctx = get_next_irq_ctx(isp);
		if (!ctx) {
			rc = -1;
			goto _exit;
		}
		inst = isp->next_mi_irq_ctx;
		sch->id = inst;
	} else {
		/**
		 * for offline MCM job called by isp_add_job(), if MI is idle AND the first job in
		 * job queue is equal to input offline MCM job id, try to manually trigger isp
		 * schedule!
		 **/
		inst = sch->id;
		rc = isp_query_job(isp, &id);
		if (id != inst) {
			rc = -1;
			goto _exit;
		}
		isp_fetch_job(isp, &id);
		ins = &isp->insts[inst];
		if (ins->state != CAM_STATE_STARTED) {
			rc = -1;
			goto _exit;
		}
		isp->next_mi_irq_ctx = inst;
	}

	ins = &isp->insts[inst];
	if (ins->online_mcm)
		isp_set_schedule_online_mcm(isp, sch);
	else
		isp_set_schedule_offline_mcm(isp, sch, isp_irq_call);

_exit:
	spin_unlock_irqrestore(&isp->sch.lock, flags);
	return rc;
}

int isp_get_schedule(struct isp_device *isp, struct mi_mis_group *mi_mis)
{
	unsigned long flags;
	struct isp_instance *ins = NULL;
	int rc = 0;
	u32 id = INVALID_INST;

	if (!isp || !mi_mis)
		return -EINVAL;

	spin_lock_irqsave(&isp->sch.lock, flags);
	if (isp->mode == ISP_STRM_MODE) {
		ins = &isp->insts[0];
		isp->sch.mi_idle = true;
		goto _frame_done;
	}

	id = isp->sch.next_mi_inst;
	if (id == INVALID_INST || id >= isp->num_insts) {
		rc = -1;
		goto _frame_done;
	}

	ins = &isp->insts[id];
	if (ins->tile_en) {
		if (ins->tile_count == TILE_COUNT) {
			id = INVALID_INST;
			ins = NULL;
			rc = -1;
			goto _frame_done;
		}
		ins->tile_count++;
		if (ins->tile_count < TILE_COUNT) {
			pr_debug("isp:%d, tile mode hasn't been finished!\n", id);
			rc = -1;
		}
	}

_frame_done:
	if (!rc && ins) {
		isp->cur_mi_irq_ctx = id;
		pr_debug("isp:%d, mi frame done!\n", isp->cur_mi_irq_ctx);
		if (ins->state == CAM_STATE_STARTED) {
			frame_done(isp, ins, !!(mi_mis->miv2_mis1 & 0x1));
			if (isp->mode == ISP_STRM_MODE)
				ins->shd_src_node = NULL;
		}
	} else {
		if (!ins)
			pr_err("fail to get currect isp instance id!\n");
	}

	spin_unlock_irqrestore(&isp->sch.lock, flags);
	return rc;
}

int isp_reset_schedule(struct isp_device *isp, u32 inst, bool force_reset)
{
	unsigned long flags;

	if (!isp)
		return -EINVAL;

	spin_lock_irqsave(&isp->sch.lock, flags);
	if (force_reset || isp->sch.next_mi_inst == inst) {
		isp->error = 1;
		isp->sch.next_mi_inst = INVALID_INST;
		isp->sch.mi_idle = true;
		isp->sch.frame_done_mask = 0;
	}
	spin_unlock_irqrestore(&isp->sch.lock, flags);

	return 0;
}

int isp_add_schedule(struct isp_device *isp, struct mi_mis_group *mi_mis)
{
	u32 ris, isp_ris;
	unsigned long flags;

	if (!isp)
		return -EINVAL;

	spin_lock_irqsave(&isp->sch.lock, flags);
	isp_ris = isp_read(isp, ISP_RIS);
	if (mi_mis->miv2_mis & MIV2_MIS_MCM_RAW0_FRAME_END_MASK) {
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
	if (mi_mis->miv2_mis & MIV2_MIS_MCM_RAW1_FRAME_END_MASK) {
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
	if (mi_mis->miv2_mis3 & MIV2_MIS3_MCM_G2RAW0_FRAME_END_MASK) {
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
	if (mi_mis->miv2_mis3 & MIV2_MIS3_MCM_G2RAW1_FRAME_END_MASK) {
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
	spin_unlock_irqrestore(&isp->sch.lock, flags);

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

	if (en_clk) {
		pm_runtime_set_active(isp->dev);
		rc = pm_runtime_get_sync(isp->dev);
		if (rc < 0)
			goto _exit;
		rc = isp_runtime_resume(isp->dev);
		if (rc < 0)
			goto _exit;
	}

_exit:
	mutex_unlock(&isp->open_lock);
	dev_dbg(isp->dev, "inst %d-\n", inst);

	return rc;
}

int isp_close(struct isp_device *isp, u32 inst, enum group_type type)
{
	struct isp_instance *ins;
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
		for (i = 0; i < MCM_BUF_NUM; i++) {
			if (isp->in_bufs[inst][i].size > 0) {
				mem_free(isp->dev, &isp->in_buf_list, &isp->in_bufs[inst][i]);
				isp->in_bufs[inst][i].size = 0;
			}
		}
	}
	if (inst < HDR_BUF_NUM && isp->hdr_bufs[inst].size > 0) {
		mem_free(isp->dev, &isp->hdr_buf_list, &isp->hdr_bufs[inst]);
		isp->hdr_bufs[inst].size = 0;
	}

	ins->meta_inst = isp->num_insts;
	rc = isp_set_state(isp, inst, CAM_STATE_CLOSED, type);
	if (rc < 0)
		dev_err(isp->dev, "failed to call isp_set_state (err=%d)\n", rc);

	mutex_lock(&isp->open_lock);
	if (refcount_read(&isp->open_cnt) > REFCNT_INIT_VAL) {
		refcount_dec(&isp->open_cnt);
		if (refcount_read(&isp->open_cnt) == REFCNT_INIT_VAL)
			dis_clk = true;
	}

	if (!dis_clk) {
		dev_dbg(isp->dev, "inst %d close\n", inst);
		goto _exit;
	}

	reset_job_queue(isp->jq);
	isp_reset_schedule(isp, INVALID_INST, true);

	isp_reset(isp);
	rc = isp_runtime_suspend(isp->dev);
	if (rc < 0)
		goto _exit;

	rc = pm_runtime_put_sync(isp->dev);
	if (rc < 0)
		goto _exit;

_exit:
	mutex_unlock(&isp->open_lock);
	dev_dbg(isp->dev, "inst %d-\n", inst);

	return rc;
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
	mutex_init(&isp->open_lock);
	mutex_init(&isp->set_input_lock);
	mutex_init(&isp->set_state_lock);
	refcount_set(&isp->open_cnt, REFCNT_INIT_VAL);

	isp->insts = devm_kcalloc(dev, isp_dt.num_insts, sizeof(*isp->insts),
				  GFP_KERNEL);
	if (!isp->insts)
		return -ENOMEM;

	isp->ctrl_dev = get_cam_ctrl_device(pdev);
	if (IS_ERR(isp->ctrl_dev))
		return PTR_ERR(isp->ctrl_dev);

	isp->jq = create_job_queue(ISP_SINK_PATH_MAX * SRC_BUF_NUM);
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
		isp->insts[i].meta_inst = isp->num_insts;
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
	spin_lock_init(&isp->sch.lock);
	isp->sch.next_mi_inst = INVALID_INST;
	isp->sch.mi_idle = true;
	isp->sch.frame_done_mask = 0;
	isp->error = 1;

	pm_runtime_enable(isp->dev);
	if (pm_runtime_active(isp->dev)) {
		rc = pm_runtime_put_sync(isp->dev);
		if (rc < 0)
			return rc;
	}

	if (of_reserved_mem_device_init_by_idx(dev, dev->of_node, 0))
		dev_warn(dev, "no reserved DMA memory for ISP\n");

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
	rc = mem_free_all(isp->dev, &isp->in_buf_list);
	if (unlikely(rc))
		dev_err(&pdev->dev, "fail to free in_buf_list (err=%d)\n", rc);
	rc = mem_free_all(isp->dev, &isp->hdr_buf_list);
	if (unlikely(rc))
		dev_err(&pdev->dev, "fail to free hdr_buf_list (err=%d)\n", rc);
	put_cam_ctrl_device(isp->ctrl_dev);
	devm_kfree(&pdev->dev, isp->insts);
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
		if (msg.log.id < 3) {
			msg.log.level = simple_strtoul(token, NULL, 10);
			pr_debug("%s [%d,%d]\n", __func__, msg.log.id, msg.log.level);
		} else {
			msg.log.module = simple_strtoul(token, NULL, 10);
			pr_debug("%s [%d,%llu]\n", __func__, msg.log.id, msg.log.module);
		}
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

static ssize_t isp_debugfs_fps_read(struct file *f, char __user *buf,
				    size_t size, loff_t *pos)
{
	struct isp_device *isp = f->f_inode->i_private;
	struct isp_instance *ins;
	char *output = NULL;
	size_t output_size = 0;
	size_t output_len = 0;
	ssize_t targets_read;
	u32 i;
	u64 fps;

	output_size = 20 * isp->num_insts;
	output = kmalloc(output_size, GFP_KERNEL);
	if (!output)
		return -ENOMEM;

	for (i = 0; i < isp->num_insts; i++) {
		ins = &isp->insts[i];
		if (ins->frame_interval)
			fps = 1000 * (ins->frame_count - 1) / ins->frame_interval;
		else
			fps = 0;
		output_len += snprintf(output + output_len, output_size - output_len,
				       "isp[%d] fps:%lld\n", i, fps);
	}

	if (*pos >= output_len) {
		kfree(output);
		return 0;
	}

	targets_read = min(size, (size_t)(output_len - *pos));
	if (copy_to_user(buf, output + *pos, targets_read)) {
		kfree(output);
		return -EFAULT;
	}

	*pos += targets_read;
	kfree(output);
	return targets_read;
}

static const struct file_operations isp_debugfs_fps_fops = {
	.owner  = THIS_MODULE,
	.read  = isp_debugfs_fps_read,
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
	if (!isp->debugfs_fps_file)
		isp->debugfs_fps_file = debugfs_create_file
				("fps", 0444, isp->debugfs_dir, isp,
				&isp_debugfs_fps_fops);
}

void isp_debugfs_remo(struct isp_device *isp)
{
	if (isp->debugfs_dir) {
		debugfs_remove_recursive(isp->debugfs_dir);
		isp->debugfs_dir = NULL;
		isp->debugfs_log_file = NULL;
		isp->debugfs_tune_file = NULL;
		isp->debugfs_fps_file = NULL;
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
	struct isp_instance *ins;
	int inst;

	if (!isp)
		return -EINVAL;

	for (inst = 0; inst < isp->num_insts; inst++) {
		ins = &isp->insts[inst];
		if (ins->state == CAM_STATE_STARTED)
			return -EBUSY;
	}

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
