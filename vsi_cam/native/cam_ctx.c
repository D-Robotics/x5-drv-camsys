// SPDX-License-Identifier: GPL-2.0-only
#include <linux/err.h>

#include "cam_buf.h"
#include "cam_ops.h"
#include "vio_node_api.h"
#include "isp.h"
#include "sif.h"

#include "cam_ctx.h"

int cam_trigger(struct cam_ctx *ctx)
{
	struct vio_subdev *vdev = (struct vio_subdev *)ctx;
	const struct cam_ops *ops = NULL;
	struct vio_node *vnode;

	if (unlikely(!vdev))
		return -EINVAL;

	if (vdev->next) {
		vdev = vdev->next;
		if (vdev->vnode->id < MODULE_NUM)
			ops = get_ops(vdev->vnode->id);
		if (ops && ops->trigger)
			return ops->trigger((struct cam_ctx *)vdev);
	} else if (vdev->vnode->next){
		vnode = vdev->vnode->next;
		if (vnode->id < MODULE_NUM)
			ops = get_ops(vnode->id);
		if (ops && ops->trigger)
			return ops->trigger((struct cam_ctx *)vnode->ich_subdev[0]);
	} else {
		pr_err("cam trigger null\n");
	}
	return -EBUSY;
}

bool cam_is_completed(struct cam_ctx *ctx)
{
	struct vio_subdev *vdev = (struct vio_subdev *)ctx;
	const struct cam_ops *ops = NULL;
	struct vio_node *vnode;

	if (unlikely(!vdev))
		return -EINVAL;

	if (vdev->next) {
		vdev = vdev->next;
		if (vdev->vnode->id < MODULE_NUM)
			ops = get_ops(vdev->vnode->id);
		if (ops && ops->is_completed)
			return ops->is_completed((struct cam_ctx *)vdev);
	} else if (vdev->vnode->next){
		vnode = vdev->vnode->next;
		if (vnode->id < MODULE_NUM)
			ops = get_ops(vnode->id);
		if (ops && ops->is_completed)
			return ops->is_completed((struct cam_ctx *)vnode->ich_subdev[0]);
	} else {
		pr_err("cam_is_completed null\n");
	}
	return false;
}

int cam_ctx_init(struct cam_ctx *ctx, struct device *dev, void *data,
		  bool has_internal_buf)
{
	return cam_ctx_init(ctx, dev, data, has_internal_buf);
}

void cam_ctx_release(struct cam_ctx *ctx)
{
	cam_ctx_release(ctx);
}

void sif_set_frame_des(struct cam_ctx *ctx, void *data)
{
	struct vio_subdev *subdev = (struct vio_subdev *)ctx;
	struct frame_id_desc frameid = {0};
	struct sif_frame_des *des = (struct sif_frame_des *)data;

	frameid.frame_id = des->frame_id;
	/* todo: timestamp temp use kernel api */
	frameid.timestamps = des->timestamps;
	frameid.tv_sec = des->fs_ts / des->trigger_freq;
	frameid.tv_usec = (des->fs_ts % des->trigger_freq) / (des->trigger_freq / 1000000u);
	frameid.trig_tv_sec = des->trigger_ts / des->trigger_freq;
	frameid.trig_tv_usec = (des->trigger_ts % des->trigger_freq) / (des->trigger_freq / 1000000u);

	// printk("kernel time:%lld fs time %llds %lldus, trigger time %llds %lldus\n",
	// 	   frameid.timestamps, frameid.tv_sec, frameid.tv_usec, frameid.trig_tv_sec, frameid.trig_tv_usec);

	if (subdev)
		memcpy(&subdev->vnode->frameid, &frameid, sizeof(struct frame_id_desc));
}

void sif_get_frame_des(struct cam_ctx *ctx)
{
	struct vio_subdev *subdev = (struct vio_subdev *)ctx;

	if (subdev)
		vio_get_frame_id(subdev->vnode);
}

void isp_update_frame_info(void *data,struct cam_ctx *ctx)
{
	struct vio_subdev *subdev         = (struct vio_subdev *)ctx;
	struct isp_frame_info *frame_info = (struct isp_frame_info *)data;

	if (unlikely(!subdev || !frame_info))
		return;

	frame_info->frame_id = subdev->vnode->frameid.frame_id;
	frame_info->time_stamp = subdev->vnode->frameid.timestamps;
}

void cam_set_stat_info(struct cam_ctx *ctx, u32 type)
{
	struct vio_subdev *subdev = (struct vio_subdev *)ctx;
	struct vio_node *vnode;

	if (unlikely(!subdev))
		return;

	vnode = (struct vio_node *)subdev->vnode;

	if (type == CAM_STAT_FS)
		vio_set_stat_info(vnode->flow_id, vnode->id, STAT_FS,
				  vnode->frameid.frame_id);
	else if (type == CAM_STAT_FE)
		vio_set_stat_info(vnode->flow_id, vnode->id, STAT_FE,
				  vnode->frameid.frame_id);
}

bool cam_osd_update(struct cam_ctx *ctx)
{
	struct vio_subdev *subdev = (struct vio_subdev *)ctx;
	struct vio_node *vnode;
	const struct cam_ops *ops = NULL;

	if (unlikely(!subdev))
		return false;

	vnode = (struct vio_node *)subdev->vnode;

	if (vnode && vnode->id < MODULE_NUM) {
		ops = get_ops(vnode->id);
		if (ops && ops->osd_update)
			return ops->osd_update(ctx);
	}

	return false;
}

int cam_osd_set_cfg(struct cam_ctx *ctx, u32 ochn_id)
{
	struct vio_subdev *subdev = (struct vio_subdev *)ctx;
	struct vio_node *vnode;
	const struct cam_ops *ops = NULL;
	int ret = 0;

	if (unlikely(!subdev))
		return -EINVAL;

	vnode = (struct vio_node *)subdev->vnode;

	if (vnode && vnode->id < MODULE_NUM) {
		ops = get_ops(vnode->id);
		if (ops && ops->osd_set_cfg)
			ret = ops->osd_set_cfg(ctx, ochn_id);
	}

	return ret;
}

int cam_set_mode(struct cam_ctx *ctx, u32 mode)
{
	struct vio_subdev *vdev = (struct vio_subdev *)ctx;
	const struct cam_ops *ops = NULL;

	if (unlikely(!vdev))
		return -EINVAL;

	if (vdev->vnode->id < MODULE_NUM)
		ops = get_ops(vdev->vnode->id);
	if (ops && ops->set_mode)
		return ops->set_mode((struct cam_ctx *)vdev, mode);
	return -EBUSY;
}

void cam_set_frame_status(void *cam_ctx, enum cam_frame_status status)
{
	struct vio_subdev *vdev = (struct vio_subdev *)cam_ctx;
	unsigned long flag;

	if (vdev) {
		spin_lock_irqsave(&vdev->slock, flag);
		vdev->frame_status = status;
		spin_unlock_irqrestore(&vdev->slock, flag);
	}
}

u8 cam_get_frame_status(void *cam_ctx)
{
	unsigned long flag;
	u8 status = 0;
	struct vio_subdev *vdev = (struct vio_subdev *)cam_ctx;

	if (vdev) {
		spin_lock_irqsave(&vdev->slock, flag);
		status = vdev->frame_status;
		spin_unlock_irqrestore(&vdev->slock, flag);
	}
	return status;
}

void cam_dec_frame_status(void *cam_ctx)
{
	struct vio_subdev *vdev = (struct vio_subdev *)cam_ctx;
	unsigned long flag;

	if (vdev) {
		spin_lock_irqsave(&vdev->slock, flag);
		if (vdev->frame_status)
			vdev->frame_status--;
		spin_unlock_irqrestore(&vdev->slock, flag);
	}
}
