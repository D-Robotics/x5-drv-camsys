// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/timekeeping.h>

#include "cam_ops.h"
#include "vse_drv.h"

#define VSE_DT_NAME     "verisilicon,vse"
#define VSE_DEV_NAME    "vs-vse"

extern s32 isp_get_inst(struct vio_subdev *vdev, int *inst_id);

static struct vio_version_info g_vse_version = {
	.major = 1,
	.minor = 0
};

static s32 vse_video_get_version(struct vio_version_info *version)
{
	memcpy(version, &g_vse_version, sizeof(struct vio_version_info));
	return 0;
}

int vse_get_sta_val(void *arg, uint32_t chn_id, volatile uint16_t hist_num[VSE_HIST_MAX][BIN_LEVEL_NUM + 1])
{
	u32 inst_id;
	struct vse_nat_instance *nat_inst;
	struct vse_instance *vse_inst;
	struct vse_hist_num *vse_hist;
	unsigned long flags;
	int i, j;

	if (!arg)
		return -EINVAL;

	nat_inst = (struct vse_nat_instance *)arg;
	inst_id = nat_inst->id;
	vse_inst = &nat_inst->dev->vse_dev.insts[inst_id];
	vse_hist = &vse_inst->hist_num[chn_id];

	if (!vse_inst->is_hist_num_updated)
		return -EBUSY;

	spin_lock_irqsave(&vse_inst->hist_lock, flags);
	for(i = 0; i < VSE_HIST_MAX; i++) {
		for (j = 0; j < BIN_LEVEL_NUM + 1; j++) {
			hist_num[i][j] = vse_hist->range_num[i][j];
		}
	}
	spin_unlock_irqrestore(&vse_inst->hist_lock, flags);

	return 0;
}

int32_t vse_video_get_struct_size(struct vio_struct_size *vio_size)
{
	int32_t ret = 0;

	switch (vio_size->type)
	{
	case BASE_ATTR:
		vio_size->size = sizeof(vse_attr_t);
		break;
	case ICHN_ATTR:
		vio_size->size = sizeof(vse_ichn_attr_t);
		break;
	case OCHN_ATTR:
		vio_size->size = sizeof(vse_ochn_attr_t);
		break;
	case EX_ATTR:
		vio_size->size = 0;
		break;
	case OCHN_EX_ATTR:
		vio_size->size = sizeof(vse_ochn_attr_ex_t);
		break;
	default:
		ret = -EINVAL;
		vio_err("Unknown isp struct type-%d\n", vio_size->type);
		break;
	}

	return ret;
}

static s32 vse_allow_bind(struct vio_subdev *vdev, struct vio_subdev *remote_vdev, u8 online_mode)
{
	struct vse_nat_instance *inst;
	enum vio_bind_type bind_type = CHN_BIND_OTF;
	struct vio_node *remote_node;
	int cas_id;

	if (!vdev || !remote_vdev)
		return bind_type;

	inst = container_of(vdev, struct vse_nat_instance, vdev);
	inst->online_mode = online_mode;

	if (vdev->id >= VNODE_ID_CAP) {
		vdev->chn_attr.format = MEM_PIX_FMT_NV12;
		vdev->chn_attr.height = inst->ochn_attr.target_h;
		vdev->chn_attr.width = inst->ochn_attr.target_w;
		vdev->chn_attr.wstride = inst->ochn_attr.target_w;
	}

	remote_node = remote_vdev->vnode;
	if (online_mode && remote_node->id == ISP_MODULE && vdev->id == VNODE_ID_SRC) {
		bind_type = CHN_BIND_OTF;
		isp_get_inst(remote_vdev, &cas_id);
		vse_set_cascade(&inst->dev->vse_dev, inst->id, cas_id, online_mode);
	} else {
		bind_type = CHN_BIND_M2M;
	}

	pr_info("%s online_mode=%d,bind_type=%d\n", __func__, online_mode, bind_type);
	return bind_type;
}

static void vse_frame_work(struct vio_node *vnode)
{
	struct vio_subdev *vdev;
	struct vse_nat_instance *inst;

	vio_dbg("[S%d][C%d] %s start\n", vnode->flow_id, vnode->ctx_id, __func__);
	vdev = vnode->ich_subdev[0];
	inst = container_of(vdev, struct vse_nat_instance, vdev);
	vse_add_job(&inst->dev->vse_dev, inst->id);
	vio_set_hw_free(vnode);
	vio_dbg("[S%d][C%d] %s done\n", vnode->flow_id, vnode->ctx_id, __func__);
}

static s32 vse_nat_open(struct vio_video_ctx *vctx)
{
	struct vse_nat_device *nat_dev;
	int rc;

	nat_dev = (struct vse_nat_device *)vctx->device;

	if (nat_dev && vctx->id == VNODE_ID_CAP) {
		rc = vse_open(&nat_dev->vse_dev, vctx->ctx_id);
		if (rc < 0) {
			pr_err("%s failed to call isp_open(err=%d).\n", __func__, rc);
			return rc;
		}
	}
	return 0;
}

static s32 vse_nat_close(struct vio_video_ctx *vctx)
{
	struct vse_nat_instance *inst;
	int rc = 0;

	if (!vctx || !vctx->vdev) {
		pr_err("%s:vctx or vctx->vdev null\n", __func__);
		return -EINVAL;
	}

	inst = container_of(vctx->vdev, struct vse_nat_instance, vdev);
	if (!inst) {
		pr_err("%s:inst null\n", __func__);
		return -EINVAL;
	}

	if (vctx->id == VNODE_ID_CAP) {
		pr_info("%s set vse state to CLOSED\n", __func__);
		if (!inst->dev) {
			pr_err("%s:inst->dev null\n", __func__);
			return -EINVAL;
		}

		rc = vse_close(&inst->dev->vse_dev, vctx->ctx_id);
		if (rc < 0) {
			pr_err("%s vse_close(ret=%d).\n", __func__, rc);
			return rc;
		}
	}

	memset(&inst->attr, 0, sizeof(inst->attr));
	memset(&inst->ichn_attr, 0, sizeof(inst->ichn_attr));
	memset(&inst->ochn_attr, 0, sizeof(inst->ochn_attr));

	return rc;
}

static s32 vse_fps_param_check(frame_fps_ctrl_t *fps)
{
	vpf_param_range_check(fps->src, 0, 120);
	vpf_param_range_check(fps->dst, 0, 120);
	return 0;
}

static s32 vse_video_set_cfg(struct vio_video_ctx *vctx, unsigned long arg)
{
	struct vse_nat_instance *inst;
	vse_attr_t attr;
	struct vse_msg msg;
	int rc;

	if (!vctx || !vctx->vdev)
		return -EINVAL;

	inst = container_of(vctx->vdev, struct vse_nat_instance, vdev);

	rc = copy_from_user(&attr, (void *)arg, sizeof(attr));
	if (rc < 0)
		return rc;

	rc = vse_fps_param_check(&inst->attr.fps);
	if (rc)
		return rc;

	inst->attr = attr;

	if (vctx->id == VNODE_ID_SRC) {
		vctx->vdev->leader = 1;

		msg.id = CAM_MSG_STATE_CHANGED;
		msg.inst = vctx->ctx_id;
		msg.channel = -1;
		msg.state = CAM_STATE_INITED;
		pr_info("%s set vse state to INITED\n", __func__);
		return vse_post(&inst->dev->vse_dev, &msg, true);
	}
	return 0;
}

static s32 vse_video_get_cfg(struct vio_video_ctx *vctx, unsigned long arg)
{
	struct vse_nat_instance *inst;
	u64 copy_ret;

	pr_info("%s vctx->ctx_id=%d,vctx->id=%d\n", __func__, vctx->ctx_id, vctx->id);
	inst = container_of(vctx->vdev, struct vse_nat_instance, vdev);

	copy_ret = copy_to_user((void __user *)arg, (void *)&inst->attr, sizeof(vse_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	return 0;
}

static s32 vse_video_set_cfg_ex(struct vio_video_ctx *vctx, unsigned long arg)
{
	return 0;
}

static s32 vse_video_get_cfg_ex(struct vio_video_ctx *vctx, unsigned long arg)
{
	return 0;
}

static void vse_get_plane(u32 format, struct vbuf_group_info *group_attr)
{
	if (format == HW_FORMAT_YUV422_8BIT) {
		group_attr->info[0].buf_attr.planecount = 2;
		group_attr->info[0].buf_attr.format = MEM_PIX_FMT_NV12;
	} else {
		pr_err("error format %d\n", format);
	}
}

static s32 vse_video_reqbufs(struct vio_video_ctx *vctx,
			     struct vbuf_group_info *group_attr)
{
	struct vse_nat_instance *inst;
	int ochn_id;
	s32 ret = 0;
	u32 format;

	inst = container_of(vctx->vdev, struct vse_nat_instance, vdev);
	if (vctx->id == VNODE_ID_SRC) {
		group_attr->bit_map = 1;
		group_attr->is_contig = 1;
		if (inst->ichn_attr.fmt == FRM_FMT_NV12)
			format = HW_FORMAT_YUV422_8BIT;
		else
			return -EINVAL;
		group_attr->info[0].buf_attr.width = inst->ichn_attr.width;
		group_attr->info[0].buf_attr.height = inst->ichn_attr.height;
		group_attr->info[0].buf_attr.wstride = inst->ichn_attr.width;
		group_attr->info[0].buf_attr.vstride = inst->ichn_attr.height;
		vse_get_plane(format, group_attr);
		group_attr->is_alloc = 0;
	} else if (vctx->id >= VNODE_ID_CAP) {
		ochn_id = vctx->id - VNODE_ID_CAP;
		if(ochn_id < 0 || ochn_id > VSE_OUT_CHNL_MAX) {
			vio_err("%s: invalid ochn id\n", __func__);
			return -EFAULT;
		}
		if (inst->ochn_attr.fmt == FRM_FMT_NV12)
			format = HW_FORMAT_YUV422_8BIT;
		else
			return -EINVAL;
		group_attr->bit_map |= 1;
		group_attr->is_contig = 1;
		group_attr->info[0].buf_attr.width = inst->ochn_attr.target_w;
		group_attr->info[0].buf_attr.height = inst->ochn_attr.target_h;
		group_attr->info[0].buf_attr.wstride = inst->ochn_attr.target_w;
		group_attr->info[0].buf_attr.vstride = inst->ochn_attr.target_h;
		vse_get_plane(format, group_attr);
		group_attr->is_alloc = 0;
	}

	pr_info("%s done bit_map 0x%x planecount %d\n", __func__, group_attr->bit_map,
		group_attr->info[0].buf_attr.planecount);
	return ret;
}

static s32 vse_video_streamon(struct vio_video_ctx *vctx)
{
	struct vse_nat_instance *inst;
	struct vio_node *vnode;
	struct vse_irq_ctx ctx;
	int i;
	int rc;

	inst = container_of(vctx->vdev, struct vse_nat_instance, vdev);
	vnode = vctx->vdev->vnode;
	if (!vnode) {
		vio_err("%s: vnode is null\n", __func__);
		return -EINVAL;
	}
	vnode->leader = 1;
	if (vctx->id == VNODE_ID_SRC) {
		rc = vse_set_source(&inst->dev->vse_dev, vctx->ctx_id,
				    inst->online_mode ? VSE_SRC_STRM0 : VSE_SRC_RDMA);
		if (rc < 0) {
			pr_err("%s failed to call vse_set_source (rc=%d)!\n", __func__, rc);
			return rc;
		}

		memset(&ctx, 0, sizeof(ctx));
		ctx.is_sink_online_mode = inst->online_mode ? true : false;
		pr_info("%s ctx.is_sink_online_mode=%d\n", __func__, ctx.is_sink_online_mode);
		ctx.sink_ctx = (struct cam_ctx *)vnode->ich_subdev[0];
		ctx.stat_ctx = (struct cam_ctx *)vnode->ich_subdev[0];
		for (i = 0; i < VSE_OUT_CHNL_MAX; i++) {
			if (inst->dev->cap_instance[i][vctx->ctx_id].ochn_attr.chn_en)
				ctx.src_ctx[i] = (struct cam_ctx *)(vnode->och_subdev[i]);
		}

		rc = vse_set_ctx(&inst->dev->vse_dev, vctx->ctx_id, &ctx);
		if (rc < 0) {
			pr_err("%s failed to call vse_set_ctx (rc=%d)!\n", __func__, rc);
			return rc;
		}

		return vse_set_state(&inst->dev->vse_dev, vctx->ctx_id, 1, 1, 1);
	}
	return 0;
}

static s32 vse_video_streamoff(struct vio_video_ctx *vctx)
{
	struct vse_nat_instance *inst;
	struct vse_irq_ctx ctx;
	int rc;

	inst = container_of(vctx->vdev, struct vse_nat_instance, vdev);

	if (vctx->id == VNODE_ID_SRC) {
		memset(&ctx, 0, sizeof(ctx));
		rc = vse_set_ctx(&inst->dev->vse_dev, vctx->ctx_id, &ctx);
		if (rc < 0) {
			pr_err("%s failed to call vse_set_ctx(err=%d).\n", __func__, rc);
			return rc;
		}

		return vse_set_state(&inst->dev->vse_dev, vctx->ctx_id, 0, 1, 1);
	}
	return 0;
}

static s32 vse_ichn_attr_check(vse_ichn_attr_t *ichn_attr)
{
	vpf_param_range_check(ichn_attr->tpg_en, 0, CAM_TRUE);
	vpf_param_range_check(ichn_attr->width, 0, 5432);
	vpf_param_range_check(ichn_attr->height, 0, 3076);
	vpf_param_range_check(ichn_attr->fmt, FRM_FMT_NV12, FRM_FMT_NV12);
	vpf_param_range_check(ichn_attr->bit_width, 8, 8);

	return 0;
}

static s32 vse_video_set_ichn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	struct vse_nat_instance *inst;
	struct cam_format fmt;
	vse_ichn_attr_t attr;
	int rc;

	inst = container_of(vctx->vdev, struct vse_nat_instance, vdev);

	rc = copy_from_user(&attr, (void *)arg, sizeof(attr));
	if (rc < 0)
		return rc;

	rc = vse_ichn_attr_check(&attr);
	if (rc)
		return rc;

	if (attr.fmt == FRM_FMT_NV12)
		fmt.format = CAM_FMT_NV12;
	else
		return -EINVAL;
	fmt.width = attr.width;
	fmt.stride = attr.width;
	fmt.height = attr.height;

	pr_info("%s format=%d,width=%d,stride=%d,height=%d\n", __func__,
		fmt.format, fmt.width, fmt.stride, fmt.height);
	rc = vse_set_iformat(&inst->dev->vse_dev, vctx->ctx_id, &fmt);
	if (rc < 0)
		return rc;
	memcpy(&inst->ichn_attr, &attr, sizeof(inst->ichn_attr));
	return 0;
}

static s32 vse_ochn_attr_check(u32 ochn_id, vse_ichn_attr_t *vse_ichn_attr, vse_ochn_attr_t *attr)
{
	s32 max_output_h;
	s32 max_output_w;
	s32 right;
	s32 bottom;

	if (!attr->chn_en)
		return 0;

	/* use input size if roi == 0
	 * roi just used by o, so this set once is acceptable
	 */
	if (attr->roi.w == 0 || attr->roi.h == 0) {
		attr->roi.x = 0;
		attr->roi.y = 0;
		attr->roi.w = vse_ichn_attr->width;
		attr->roi.h = vse_ichn_attr->height;
		vio_dbg("vse chn-%d use input size instand of roi\n", ochn_id);
	}

	/* use roi size if output size == 0
	 * just modify this channel target
	 */
	if (ochn_id == VSE_DOWN_SCALE_4K || ochn_id == VSE_UP_SCALE_4K) {
		max_output_h = 3076;
		max_output_w = 4096;
	} else if (ochn_id == VSE_DOWN_SCALE_1080P0 || ochn_id == VSE_DOWN_SCALE_1080P1) {
		max_output_h = 1080;
		max_output_w = 1920;
	} else if (ochn_id == VSE_DOWN_SCALE_720P0 || ochn_id == VSE_DOWN_SCALE_720P1) {
		max_output_h = 720;
		max_output_w = 1280;
	} else {
		return -EINVAL;
	}
	if (attr->target_w == 0 || attr->target_h == 0) {
		attr->target_w = CFG_MIN(attr->roi.w, max_output_w);
		attr->target_h = CFG_MIN(attr->roi.h, max_output_h);
		vio_dbg("vse chn-%d use roi size instand of target size\n", ochn_id);
	}
	attr->target_w = ALIGN(attr->target_w, 16);

	vpf_param_range_check(attr->fmt, FRM_FMT_NV12, FRM_FMT_NV12);
	vpf_param_range_check(attr->bit_width, 8, 8);
	vpf_param_range_check(attr->roi.w, 0, 4096);
	vpf_param_range_check(attr->roi.h, 0, 3076);

	right = attr->roi.x + attr->roi.w;
	bottom = attr->roi.y + attr->roi.h;
	vpf_param_range_check(right, 0, vse_ichn_attr->width);
	vpf_param_range_check(bottom, 0, vse_ichn_attr->height);

	if (attr->target_w % 2 != 0 || attr->target_h % 2 != 0) {
		vio_err("vse chn-%d scale size(%d, %d) cannot be odd\n", ochn_id,
			attr->target_w, attr->target_h);
		return -EINVAL;
	}

	/* check downscale output size */
	if (ochn_id >= VSE_DOWN_SCALE_4K && ochn_id <= VSE_DOWN_SCALE_720P1) {
		vpf_param_range_check(attr->target_w, 64, CFG_MIN(attr->roi.w, max_output_w));
		vpf_param_range_check(attr->target_h, 64, CFG_MIN(attr->roi.h, max_output_h));
	}

	/* check upscale output size */
	if (ochn_id == VSE_UP_SCALE_4K) {
		uint32_t in_total_size = 0;
		uint32_t up_total_size = 0;

		vpf_param_range_check(attr->target_w, attr->roi.w, 4096);
		vpf_param_range_check(attr->target_h, attr->roi.h, 3076);

		in_total_size = attr->roi.h * attr->roi.w;
		up_total_size = attr->target_h * attr->target_w;

		if (up_total_size > in_total_size * 4) {
			vio_err("max upscale ratio is 4x, exceed valid range\n");
			return -EINVAL;
		}
	}


	return 0;
}

static s32 vse_video_set_ochn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	struct vse_nat_instance *inst;
	struct vse_nat_device *nat_dev;
	struct cam_format fmt;
	struct cam_rect crop;
	struct vse_fps_rate fps;
	vse_ochn_attr_t attr;
	int ochn_id;
	int rc;

	if (vctx->id < VNODE_ID_CAP)
		return -EINVAL;

	pr_info("%s vctx->ctx_id=%d,vctx->id=%d\n", __func__, vctx->ctx_id, vctx->id);
	inst = container_of(vctx->vdev, struct vse_nat_instance, vdev);
	nat_dev = inst->dev;

	rc = copy_from_user(&attr, (void *)arg, sizeof(attr));
	if (rc < 0)
		return rc;
	attr.fps.src = inst->attr.fps.src; //TODO: rm it if json changed
	ochn_id = vctx->id - VNODE_ID_CAP;
	if (ochn_id > VSE_OUT_CHNL_MAX) {
		vio_err("%s: Invalid output channel id\n", __func__);
		return -EINVAL;
	}

	if (!attr.chn_en) {
		vio_dbg("vse ochnid: %d disable\n", ochn_id);
		return 0;
	}

	/* should use src inst ichn attr */
	rc = vse_ochn_attr_check(ochn_id, &nat_dev->src_instance[vctx->ctx_id].ichn_attr, &attr);
	if (rc) {
		pr_err("vse_ochn_attr_check failed\n");
		return rc;
	}
	rc = vse_fps_param_check(&attr.fps);
	if (rc < 0) {
		pr_err("vse_fps_param_check failed\n");
		return rc;
	}

	if (attr.fmt == FRM_FMT_NV12)
		fmt.format = CAM_FMT_NV12;
	else
		return -EINVAL;
	fmt.width = attr.target_w;
	fmt.stride = attr.target_w;
	fmt.height = attr.target_h;
	crop.x = attr.roi.x;
	crop.y = attr.roi.y;
	crop.w = attr.roi.w;
	crop.h = attr.roi.h;

	pr_info("%s ochn_id=%d,format=%d,width=%d,stride=%d,height=%d\n", __func__,
		ochn_id, fmt.format, fmt.width, fmt.stride, fmt.height);
	rc = vse_set_oformat(&inst->dev->vse_dev, vctx->ctx_id, ochn_id, &fmt, &crop, true);
	if (rc < 0)
		return rc;

	fps.src = attr.fps.src;
	fps.dst = attr.fps.dst;
	rc = vse_set_fps_rate(&inst->dev->vse_dev, vctx->ctx_id, ochn_id, &fps);
	if (rc < 0) {
		pr_err("vse_set_fps_dst_rate failed");
		return rc;
	}

	memcpy(&inst->ochn_attr, &attr, sizeof(inst->ochn_attr));
	return 0;
}

static s32 vse_video_get_ichn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	struct vse_nat_instance *inst;
	u64 copy_ret;

	pr_info("%s vctx->ctx_id=%d,vctx->id=%d\n", __func__, vctx->ctx_id, vctx->id);
	inst = container_of(vctx->vdev, struct vse_nat_instance, vdev);

	copy_ret = copy_to_user((void __user *)arg, (void *)&inst->ichn_attr,
				sizeof(vse_ichn_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	return 0;
}

static s32 vse_video_get_ochn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	struct vse_nat_instance *inst;
	u64 copy_ret;

	pr_info("%s vctx->ctx_id=%d,vctx->id=%d\n", __func__, vctx->ctx_id, vctx->id);
	inst = container_of(vctx->vdev, struct vse_nat_instance, vdev);

	copy_ret = copy_to_user((void __user *)arg, (void *)&inst->ochn_attr,
				sizeof(vse_ochn_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	return 0;
}

static s32 vse_do_ex_attr_changed(struct vio_video_ctx *vctx, vse_ochn_attr_t *attr)
{
	struct vse_nat_instance *inst =
			container_of(vctx->vdev, struct vse_nat_instance, vdev);
	struct cam_format fmt;
	struct cam_rect crop;
	int ochn_id = vctx->id - VNODE_ID_CAP;
	int rc;

	/* should use src inst ichn attr */
	rc = vse_ochn_attr_check(ochn_id, &inst->dev->src_instance[vctx->ctx_id].ichn_attr,
				  attr);
	if (rc < 0) {
		pr_err("vse_ochn_attr_check failed\n");
		return rc;
	}

	if (attr->fmt == FRM_FMT_NV12)
		fmt.format = CAM_FMT_NV12;
	else
		return -EINVAL;
	fmt.width = attr->target_w;
	fmt.stride = attr->target_w;
	fmt.height = attr->target_h;
	crop.x = attr->roi.x;
	crop.y = attr->roi.y;
	crop.w = attr->roi.w;
	crop.h = attr->roi.h;

	pr_info("%s ochn_id=%d, chn_en=%d, format=%d, width=%d, stride=%d, height=%d, roi_w=%d, roi_h=%d\n", __func__,
		ochn_id, attr->chn_en, fmt.format, fmt.width, fmt.stride, fmt.height, crop.w, crop.h);
	rc = vse_set_oformat(&inst->dev->vse_dev, vctx->ctx_id, ochn_id, &fmt, &crop, attr->chn_en);
	if (rc < 0)
		return rc;

	memcpy(&inst->ochn_attr, attr, sizeof(inst->ochn_attr));
	return 0;
}

static s32 vse_do_fps_param_changed(struct vio_video_ctx *vctx, frame_fps_ctrl_t *fps)
{
	struct vse_nat_instance *inst =
			container_of(vctx->vdev, struct vse_nat_instance, vdev);
	struct vse_fps_rate fps_ctrl;
	int ochn_id = vctx->id - VNODE_ID_CAP;
	int rc;

	rc = vse_fps_param_check(fps);
	if (rc < 0) {
		pr_err("vse_ochn_attr_check failed\n");
		return rc;
	}

	fps_ctrl.src = fps->src;
	fps_ctrl.dst = fps->dst;
	rc = vse_set_fps_rate(&inst->dev->vse_dev, vctx->ctx_id, ochn_id, &fps_ctrl);
	if (rc < 0) {
		pr_err("vse_set_fps_dst_rate failed");
		return rc;
	}
	memcpy(&inst->ochn_attr.fps, &fps, sizeof(inst->ochn_attr.fps));
	return rc;
}

static s32 vse_video_set_ochn_attr_ex(struct vio_video_ctx *vctx, unsigned long arg)
{
	struct vse_nat_instance *inst =
			container_of(vctx->vdev, struct vse_nat_instance, vdev);
	vse_ochn_attr_ex_t attr_ex;
	vse_ochn_attr_t attr;
	bool attr_changed = false, fps_changed = false;
	int rc;

	if (vctx->id < VNODE_ID_CAP)
		return -EINVAL;

	if ((vctx->id - VNODE_ID_CAP) > VSE_OUT_CHNL_MAX) {
		vio_err("%s: Invalid output channel id\n", __func__);
		return -EINVAL;
	}

	pr_info("%s vctx->ctx_id=%d,vctx->id=%d\n", __func__, vctx->ctx_id, vctx->id);
	rc = copy_from_user(&attr_ex, (void *)arg, sizeof(attr_ex));
	if (rc < 0)
		return rc;

	memcpy(&attr, &inst->ochn_attr, sizeof(inst->ochn_attr));
	if (attr.chn_en != attr_ex.chn_en) {
		attr.chn_en = attr_ex.chn_en;
		attr_changed = true;
	}
	if (attr.target_w != attr_ex.target_w) {
		attr.target_w = attr_ex.target_w;
		attr_changed = true;
	}
	if (attr.target_h != attr_ex.target_h) {
		attr.target_h = attr_ex.target_h;
		attr_changed = true;
	}
	if (memcmp(&attr.roi, &attr_ex.roi, sizeof(attr.roi))) {
		attr.roi = attr_ex.roi;
		attr_changed =true;
	}
	if (attr.fps.src != attr_ex.src_fps) {
		attr.fps.src = attr_ex.src_fps;
		fps_changed = true;
	}
	if (attr.fps.dst != attr_ex.dst_fps) {
		attr.fps.dst = attr_ex.dst_fps;
		fps_changed = true;
	}
	if (!attr_changed && !fps_changed)
		return -EINVAL;

	if (attr_changed) {
		rc = vse_do_ex_attr_changed(vctx, &attr);
		if (rc < 0)
			return rc;
	}
	if (fps_changed) {
		rc = vse_do_fps_param_changed(vctx, &attr.fps);
		if (rc < 0)
			return rc;
	}
	return 0;
}

static struct vio_common_ops vse_vops = {
	.open = vse_nat_open,
	.close = vse_nat_close,
	.video_get_version = vse_video_get_version,
	.video_set_attr = vse_video_set_cfg,
	.video_get_attr = vse_video_get_cfg,
	.video_set_attr_ex = vse_video_set_cfg_ex,
	.video_get_attr_ex = vse_video_get_cfg_ex,
	.video_get_buf_attr = vse_video_reqbufs,
	.video_set_obuf = NULL,
	.video_set_ibuf = NULL,
	.video_start = vse_video_streamon,
	.video_stop = vse_video_streamoff,
	.video_set_ichn_attr = vse_video_set_ichn_attr,
	.video_set_ochn_attr = vse_video_set_ochn_attr,
	.video_get_ichn_attr = vse_video_get_ichn_attr,
	.video_get_ochn_attr = vse_video_get_ochn_attr,
	.video_set_ochn_attr_ex = vse_video_set_ochn_attr_ex,
	.video_get_struct_size = vse_video_get_struct_size,
};

static int vse_trigger(struct cam_ctx *ctx)
{
	struct vio_subdev *vdev = (struct vio_subdev *)ctx;
	struct vse_nat_instance *inst;

	inst = container_of(vdev, struct vse_nat_instance, vdev);
	if (vdev)
		vse_add_job(&inst->dev->vse_dev, inst->id);
	return 0;
}

static bool vse_is_completed(struct cam_ctx *ctx)
{
	struct vio_subdev *vdev = (struct vio_subdev *)ctx;
	struct vse_nat_instance *inst;
	bool rc = false;

	inst = container_of(vdev, struct vse_nat_instance, vdev);
	if (vdev)
		rc = inst->dev->vse_dev.is_completed;
	return rc;
}

static bool vse_osd_update(struct cam_ctx *ctx)
{
	struct vio_subdev *subdev = NULL;
	struct vse_nat_instance *vse_ins = NULL;
	struct vse_nat_device *nat_dev;
	bool ret;

	subdev = (struct vio_subdev *)ctx;
	vse_ins = container_of(subdev, struct vse_nat_instance, vdev);
	nat_dev = vse_ins->dev;
	if (((struct osd_interface_ops *)nat_dev->osd_cops->cops)->frame_process
		&& atomic_read(&vse_ins->osd_info.need_sw_osd)) {
		((struct osd_interface_ops *)nat_dev->osd_cops->cops)->frame_process(&vse_ins->osd_info);
		ret = true;
	} else {
		ret = false;
	}

	return ret;
}

static int vse_set_osd_cfg(struct cam_ctx *ctx, u32 ochn_id)
{
	struct vio_subdev *subdev = NULL;
	struct vse_nat_instance *vse_ins = NULL;
	struct vse_osd_cfg *osd_hw_cfg = NULL;
	struct vse_osd_info osd_info;
	struct vse_hist_info hist_info[VSE_HIST_MAX];
	int ret = 0;
	int i = 0;

	subdev = (struct vio_subdev *)ctx;
	vse_ins = container_of(subdev, struct vse_nat_instance, vdev);
	osd_hw_cfg = &vse_ins->osd_hw_cfg;

	if (osd_hw_cfg->osd_box_update) {
		for (i = 0; i < MAX_OSD_NUM; i++) {
			memset(&osd_info, 0, sizeof(struct vse_osd_info));
			osd_info.roiId = i;
			osd_info.roiEnable = osd_hw_cfg->osd_box[i].osd_en;
			osd_info.roiStartX = osd_hw_cfg->osd_box[i].start_x;
			osd_info.roiStartY = osd_hw_cfg->osd_box[i].start_y;
			osd_info.roiHsize  = osd_hw_cfg->osd_box[i].width;
			osd_info.roiVsize  = osd_hw_cfg->osd_box[i].height;
			ret |= vse_set_osd_info(&vse_ins->dev->vse_dev, vse_ins->id, ochn_id, &osd_info);
		}
		osd_hw_cfg->osd_box_update = false;
	}

	if (osd_hw_cfg->osd_buf_update) {
		for (i = 0; i < MAX_OSD_NUM; i++) {
			struct vse_osd_buf osd_buf;
			memset(&osd_buf, 0, sizeof(struct vse_osd_buf));
			osd_buf.id  = i;
			osd_buf.buf.addr = osd_hw_cfg->osd_buf[i];
			// osd_buf.buf.size = osd_hw_cfg->osd_buf[i];
			ret |= vse_set_osd_buf(&vse_ins->dev->vse_dev, vse_ins->id, ochn_id, &osd_buf);
		}
		osd_hw_cfg->osd_buf_update = false;
	}

	if (osd_hw_cfg->color_map.color_map_update) {
		struct vse_lut_tbl tbl;
		memset(&tbl, 0, sizeof(struct vse_lut_tbl));
		//TODO1: yuv color map sync with x5 osd
		//TODO2: config alpya lut table, depended on x5 osd
		memcpy(tbl.rgb_data, osd_hw_cfg->color_map.color_map, sizeof(osd_hw_cfg->color_map.color_map));
		ret |= vse_set_osd_lut(&vse_ins->dev->vse_dev, vse_ins->id, ochn_id, &tbl);
		osd_hw_cfg->color_map.color_map_update = false;
	}

	if (osd_hw_cfg->osd_sta_update) {
		memset(&hist_info, 0, sizeof(hist_info));
		for (i = 0; i < VSE_HIST_MAX; i++) {
			hist_info[i].histId = i;
			hist_info[i].histEnable = osd_hw_cfg->osd_sta[i].sta_en;
			hist_info[i].histStartX = osd_hw_cfg->osd_sta[i].start_x;
			hist_info[i].histStartY = osd_hw_cfg->osd_sta[i].start_y;
			hist_info[i].histHsize  = osd_hw_cfg->osd_sta[i].width;
			hist_info[i].histVsize  = osd_hw_cfg->osd_sta[i].height;
		}
		ret |= vse_set_hist_info(&vse_ins->dev->vse_dev, vse_ins->id, ochn_id, hist_info);
		osd_hw_cfg->osd_sta_update = false;
		vse_ins->dev->vse_dev.insts[vse_ins->id].is_need_read_hist = true;
		vse_ins->dev->vse_dev.insts[vse_ins->id].is_hist_num_updated = false;
	}

	if (osd_hw_cfg->osd_sta_level_update) {
		ret |= vse_set_bin_level(&vse_ins->dev->vse_dev, vse_ins->id, ochn_id, osd_hw_cfg->osd_sta_level);
		osd_hw_cfg->osd_sta_level_update = false;
		vse_ins->dev->vse_dev.insts[vse_ins->id].is_need_read_hist = true;
		vse_ins->dev->vse_dev.insts[vse_ins->id].is_hist_num_updated = false;
	}

	return ret;
}

static int vse_read_hist(struct cam_ctx *ctx, u32 ochn_id)
{
	struct vio_subdev *subdev = NULL;
	struct vse_nat_instance *nat_inst = NULL;
	struct vse_osd_cfg *osd_hw_cfg = NULL;
	struct vse_instance *vse_inst = NULL;
	unsigned long flags;
	int ret = 0;
	int i;

	subdev = (struct vio_subdev *)ctx;
	nat_inst = container_of(subdev, struct vse_nat_instance, vdev);
	osd_hw_cfg = &nat_inst->osd_hw_cfg;
	vse_inst = &nat_inst->dev->vse_dev.insts[nat_inst->id];

	spin_lock_irqsave(&vse_inst->hist_lock, flags);
	for (i = 0; i < MAX_STA_NUM; i++) {
			if (nat_inst->osd_hw_cfg.osd_sta[i].sta_en)
				ret |= vse_get_hist_num(&nat_inst->dev->vse_dev, nat_inst->id, ochn_id, i);
	}
	spin_unlock_irqrestore(&vse_inst->hist_lock, flags);

	return ret;
}


static int vse_set_mode(struct cam_ctx *ctx, u32 mode)
{
	struct vio_subdev *vdev = (struct vio_subdev *)ctx;
	struct vse_nat_instance *ins =
			container_of(vdev, struct vse_nat_instance, vdev);
	struct vse_msg msg;

	if (!vdev)
		return -EINVAL;

	if (mode == CAM_SIMPLEX_MODE)
		ins->dev->vse_dev.mode = VSE_SCM_MODE;
	else if (mode == CAM_MULTIPLEX_MODE)
		ins->dev->vse_dev.mode = VSE_MCM_MODE;
	else
		return -EINVAL;

	msg.id = VSE_MSG_MODE_CHANGED;
	msg.inst = ins->id;
	msg.channel = -1;
	msg.mode = ins->dev->vse_dev.mode;
	return vse_post(&ins->dev->vse_dev, &msg, true);
}

static const struct cam_ops vse_ops = {
	.trigger = vse_trigger,
	.is_completed = vse_is_completed,
	.osd_update = vse_osd_update,
	.osd_set_cfg = vse_set_osd_cfg,
	.read_hist = vse_read_hist,
	.set_mode = vse_set_mode,
};

static ssize_t vse_stat_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vse_nat_device *nat_dev;
	struct vse_nat_instance *src_ins, *cap_ins;
	struct vio_node *vnode;
	ssize_t size = PAGE_SIZE, len = 0;
	u32 i, j;

	nat_dev = (struct vse_nat_device *)dev_get_drvdata(dev);

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		vnode = &nat_dev->vnode[i];
		src_ins = &nat_dev->src_instance[i];
		if (!src_ins->ichn_attr.fmt)
			continue;

		len += snprintf(&buf[len], size - len,
			       "------------------- flow%d info -------------------\n",
			       vnode->flow_id);
		if (size - len <= 0)
			break;

		len += snprintf(&buf[len], size - len,
			       "input_fps:%d/%d\ninput_width:%d\ninput_height:%d\ninput_format:%d\n"
			       "input_bitwidth:%d\n",
			       src_ins->attr.fps.src, src_ins->attr.fps.dst,
			       src_ins->ichn_attr.width, src_ins->ichn_attr.height,
			       src_ins->ichn_attr.fmt, src_ins->ichn_attr.bit_width);
		if (size - len <= 0)
			break;

		for (j = 0; j < VSE_OUT_CHNL_MAX - 1; j++) {
			cap_ins = &nat_dev->cap_instance[j][i];
			if (!cap_ins->ochn_attr.chn_en)
				continue;
			len += snprintf(&buf[len], size - len,
				       "dns%d channel: roi [%d][%d][%d][%d], target [%d][%d], "
				       "fps [%d/%d]\n", j,
				       cap_ins->ochn_attr.roi.x, cap_ins->ochn_attr.roi.y,
				       cap_ins->ochn_attr.roi.w, cap_ins->ochn_attr.roi.h,
				       cap_ins->ochn_attr.target_w, cap_ins->ochn_attr.target_h,
				       cap_ins->ochn_attr.fps.src, cap_ins->ochn_attr.fps.dst);
			if (size - len <= 0)
				break;
		}

		if (size - len <= 0)
			break;
		cap_ins = &nat_dev->cap_instance[j][i];
		if (!cap_ins->ochn_attr.chn_en)
			continue;
		len += snprintf(&buf[len], size - len,
			       "ups channel: roi [%d][%d][%d][%d], target [%d][%d], "
			       "fps [%d/%d]\n",
			       src_ins->ochn_attr.roi.x, src_ins->ochn_attr.roi.y,
			       src_ins->ochn_attr.roi.w, src_ins->ochn_attr.roi.h,
			       src_ins->ochn_attr.target_w, src_ins->ochn_attr.target_h,
			       src_ins->ochn_attr.fps.src, cap_ins->ochn_attr.fps.dst);
		if (size - len <= 0)
			break;
	}
	return len;
}
static DEVICE_ATTR(stat, 0444, vse_stat_show, NULL);

static void empty_osd_frame_process(struct vio_osd_info *osd_info)
{
	pr_info("%s\n", __func__);
}

static void empty_osd_set_info(uint32_t chn_id, uint32_t ctx_id, struct vio_osd_info *info)
{
	pr_info("%s\n", __func__);
}

struct osd_interface_ops empty_osd_cb_ops = {
	.frame_process = empty_osd_frame_process,
	.osd_set_info = empty_osd_set_info,
};

static void vse_return_frame(struct vio_osd_info *osd_info, struct vio_frame *frame)
{
	struct vse_nat_instance *vse_ctx = NULL;

	vse_ctx = container_of(osd_info, struct vse_nat_instance, osd_info);
	vio_frame_done(&vse_ctx->vdev);

	pr_debug("%s frame done, %d %d\n", __func__, osd_info->chn_id, osd_info->ctx_id);
}

// struct vse_nat_device *g_nat_dev;
// void (*get_osd_info)(int32_t inst_id, int32_t ochn_id)
// {
// 	return g_nat_dev->cap_instance[inst_id][ochn_id].osd_info;
// }

// static struct vse_interface_ops vse_cb_ops = {
	// .get_osd_info = vse_get_osd_info,
// };

// DECLARE_VIO_CALLBACK_OPS(vse_cops, VSE_COPS_MAGIC, &VSE_cb_ops);

static int vse_nat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vse_nat_device *nat_dev;
	struct vio_osd_info *osd_info;
	char name[64];
	u32 i, j;
	s32 ret;
	int rc;
	ktime_t start, end;

	start = ktime_get_boottime();
	nat_dev = devm_kzalloc(dev, sizeof(*nat_dev), GFP_KERNEL);
	if (!nat_dev)
		return -ENOMEM;
	// g_nat_dev = nat_dev;

	rc = vse_probe(pdev, &nat_dev->vse_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call vse_probe (err=%d)\n", rc);
		return rc;
	}

	nat_dev->gtask.no_worker = 1;
	nat_dev->gtask.id = VSE_MODULE;

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		nat_dev->vnode[i].flow_id = INVALID_FLOW_ID;
		nat_dev->vnode[i].ctx_id = i;
		nat_dev->vnode[i].id = VSE_MODULE;
		nat_dev->vnode[i].gtask = &nat_dev->gtask;
		nat_dev->vnode[i].allow_bind = vse_allow_bind;
		nat_dev->vnode[i].frame_work = vse_frame_work;
		nat_dev->vnode[i].ich_subdev[VSE_MAIN_FRAME] = &nat_dev->src_instance[i].vdev;
		nat_dev->vnode[i].active_ich = 1 << VSE_MAIN_FRAME;
		nat_dev->vnode[i].no_online_support = 1;
		for (j = 0; j < VSE_OUT_CHNL_MAX; j++) {
			nat_dev->vnode[i].och_subdev[j] = &nat_dev->cap_instance[j][i].vdev;
			nat_dev->vnode[i].active_och |= 1 << j;
			nat_dev->cap_instance[j][i].vdev.vnode = &nat_dev->vnode[i];
			nat_dev->cap_instance[j][i].vdev.pingpong_ring = 1;
			nat_dev->cap_instance[j][i].dev = nat_dev;
			nat_dev->cap_instance[j][i].id = i;
			osd_info = &nat_dev->cap_instance[j][i].osd_info;
			osd_info->chn_id = j;
			osd_info->ctx_id = i;
			osd_info->return_frame = vse_return_frame;
			osd_info->get_sta_val = vse_get_sta_val;
		}
		nat_dev->src_instance[i].vdev.vnode = &nat_dev->vnode[i];
		nat_dev->src_instance[i].dev = nat_dev;
		nat_dev->src_instance[i].id = i;
	}

	nat_dev->vps_dev[SRC_INDEX].vps_ops = &vse_vops;
	nat_dev->vps_dev[SRC_INDEX].ip_dev = nat_dev;
	nat_dev->vps_dev[SRC_INDEX].vnode = nat_dev->vnode;
	// nat_dev->vps_dev[SRC_INDEX].vnode_id = VSE_MODULE;
	nat_dev->vps_dev[SRC_INDEX].max_ctx = VIO_MAX_STREAM;
	nat_dev->vps_dev[SRC_INDEX].iommu_dev = dev;
	nat_dev->vps_dev[SRC_INDEX].vid = VNODE_ID_SRC;
	snprintf(name, sizeof(name), "%s%d_src", VSE_DEV_NAME, nat_dev->vse_dev.id);
	ret = vio_register_device_node(name, &nat_dev->vps_dev[SRC_INDEX]);
	if (ret < 0) {
		dev_err(dev, "failed to call vio_register_device_node(err=%d).\n", ret);
		return -EFAULT;
	}

	for (i = CAP_INDEX_1ST; i <= CAP_INDEX_END; i++) {
		nat_dev->vps_dev[i].vps_ops = &vse_vops;
		nat_dev->vps_dev[i].ip_dev = nat_dev;
		nat_dev->vps_dev[i].vnode = nat_dev->vnode;
		// nat_dev->vps_dev[i].vnode_id = VSE_MODULE;
		nat_dev->vps_dev[i].max_ctx = VIO_MAX_STREAM;
		nat_dev->vps_dev[i].iommu_dev = dev;
		nat_dev->vps_dev[i].vid = VNODE_ID_CAP + i - CAP_INDEX_1ST;
		snprintf(name, sizeof(name), "%s%d_cap%d", VSE_DEV_NAME, nat_dev->vse_dev.id,
			 i - CAP_INDEX_1ST);
		ret = vio_register_device_node(name, &nat_dev->vps_dev[i]);
		if (ret < 0) {
			dev_err(dev, "failed to call vio_register_device_node(err=%d).\n", ret);
			for (j = CAP_INDEX_1ST; j < i; j++)
				vio_unregister_device_node(&nat_dev->vps_dev[j]);
			vio_unregister_device_node(&nat_dev->vps_dev[SRC_INDEX]);
			return -EFAULT;
		}
	}

	add_ops(VSE_MODULE, &vse_ops);
	platform_set_drvdata(pdev, nat_dev);

	nat_dev->osd_cops = (struct vio_callback_ops *)vio_get_callback_ops(&empty_osd_cb_ops, VSE_MODULE, COPS_0);
	for (i = VSE_DOWN_SCALE_4K; i < VSE_OCHN_MAX; i++) {
		for (j = 0; j < VIO_MAX_STREAM; j++) {
			((struct osd_interface_ops *)nat_dev->osd_cops->cops)->osd_set_info(i, j, &nat_dev->cap_instance[i][j].osd_info);
		}
	}

	ret = device_create_file(dev, &dev_attr_stat);
	if (ret < 0)
		dev_warn(dev, "failed to call device_create_file(err=%d).\n", ret);

#ifdef CONFIG_DEBUG_FS
	vse_debugfs_init(&nat_dev->vse_dev);
#endif

	end = ktime_get_boottime();
	dev_info(dev, "VS VSE driver (native) probed done, time used: %lldus\n", ktime_to_us(ktime_sub(end, start)));
	return 0;
}

static int vse_nat_remove(struct platform_device *pdev)
{
	struct vse_nat_device *nat_dev = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	u32 i;
	int rc;

	for (i = CAP_INDEX_1ST; i <= CAP_INDEX_END; i++)
		vio_unregister_device_node(&nat_dev->vps_dev[i]);
	vio_unregister_device_node(&nat_dev->vps_dev[SRC_INDEX]);

	rc = vse_remove(pdev, &nat_dev->vse_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call vse_remove (err=%d)\n", rc);
		return rc;
	}

	device_remove_file(dev, &dev_attr_stat);
#ifdef CONFIG_DEBUG_FS
	vse_debugfs_remo(&nat_dev->vse_dev);
#endif

	dev_dbg(dev, "VS VSE driver (native) removed\n");
	return 0;
}

static const struct dev_pm_ops vse_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(vse_system_suspend, vse_system_resume)
};

static const struct of_device_id vse_of_match[] = {
	{ .compatible = VSE_DT_NAME },
	{ },
};

MODULE_DEVICE_TABLE(of, vse_of_match);

static struct platform_driver vse_driver = {
	.probe  = vse_nat_probe,
	.remove = vse_nat_remove,
	.driver = {
		.name = VSE_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vse_of_match,
		.pm = &vse_pm_ops,
	}
};

static int __init vse_init_module(void)
{
	return platform_driver_register(&vse_driver);
}

static void __exit vse_exit_module(void)
{
	platform_driver_unregister(&vse_driver);
}

module_init(vse_init_module);
module_exit(vse_exit_module);

MODULE_DESCRIPTION("VeriSilicon VSE Driver");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
MODULE_LICENSE("GPL");
MODULE_ALIAS("VeriSilicon-VSE");
