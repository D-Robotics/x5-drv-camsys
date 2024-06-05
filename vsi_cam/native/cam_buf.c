// SPDX-License-Identifier: GPL-2.0-only
#include <linux/err.h>

#include "cam_online_ops.h"
#include "vio_node_api.h"
#include "sif.h"

#include "cam_buf.h"

int cam_reqbufs(struct cam_buf_ctx *ctx, unsigned int num,
		struct cam_buf_ops *ops)
{
	return -EBUSY;
}

int cam_qbuf_irq(struct cam_buf_ctx *ctx, struct cam_buf *buf, bool remote)
{
	struct vio_subdev *subdev = (struct vio_subdev *)ctx;
	struct vio_node *vnode;

	if (ctx) {
		vio_frame_done((struct vio_subdev *)ctx);
		vnode = (struct vio_node *)subdev->vnode;
		vio_set_stat_info(vnode->flow_id, vnode->id, STAT_QB,
				  vnode->frameid.frame_id);
		return 0;
	}
	return -EBUSY;
}

struct cam_buf *cam_dqbuf_irq(struct cam_buf_ctx *ctx, bool remote)
{
	struct vio_subdev *subdev = (struct vio_subdev *)ctx;
	struct vio_node *vnode;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame = NULL;
	u64 flags = 0;

	if (ctx) {
		framemgr = subdev->cur_fmgr;
		vio_e_barrier_irqs(framemgr, flags);
		frame = peek_frame(framemgr, FS_REQUEST);
		vio_x_barrier_irqr(framemgr, flags);

		if (frame) {
			vio_e_barrier_irqs(framemgr, flags);
			trans_frame(framemgr, frame, FS_PROCESS);
			vio_x_barrier_irqr(framemgr, flags);
			vnode = (struct vio_node *)subdev->vnode;
			vio_set_stat_info(vnode->flow_id, vnode->id, STAT_DQ,
					  vnode->frameid.frame_id);
		}
	}
	return (struct cam_buf *)frame;
}

struct cam_buf *cam_acqbuf_irq(struct cam_buf_ctx *ctx)
{
	struct vio_framemgr *framemgr;
	struct vio_frame *frame = NULL;
	u64 flags = 0;

	if (ctx) {
		framemgr = ((struct vio_subdev *)ctx)->cur_fmgr;
		vio_e_barrier_irqs(framemgr, flags);
		frame = peek_frame(framemgr, FS_REQUEST);
		vio_x_barrier_irqr(framemgr, flags);
	}
	return (struct cam_buf *)frame;
}

int cam_trigger(struct cam_buf_ctx *ctx)
{
	struct vio_subdev *vdev = (struct vio_subdev *)ctx;
	const struct cam_online_ops *ops = NULL;
	struct vio_node *vnode;

	if (vdev && vdev->next) {
		vdev = vdev->next;
		if (vdev->vnode->id < MODULE_NUM)
			ops = get_online_ops(vdev->vnode->id);
		if (ops && ops->trigger)
			return ops->trigger((struct cam_buf_ctx *)vdev);
	} else if (vdev && vdev->vnode->next){
		vnode = vdev->vnode->next;
		if (vnode->id < MODULE_NUM)
			ops = get_online_ops(vnode->id);
		if (ops && ops->trigger)
			return ops->trigger((struct cam_buf_ctx *)vnode->ich_subdev[0]);
	} else {
		pr_err("cam trigger null\n");
	}
	return -EBUSY;
}

bool cam_is_completed(struct cam_buf_ctx *ctx)
{
	struct vio_subdev *vdev = (struct vio_subdev *)ctx;
	const struct cam_online_ops *ops = NULL;
	struct vio_node *vnode;

	if (vdev && vdev->next) {
		vdev = vdev->next;
		if (vdev->vnode->id < MODULE_NUM)
			ops = get_online_ops(vdev->vnode->id);
		if (ops && ops->is_completed)
			return ops->is_completed((struct cam_buf_ctx *)vdev);
	} else if (vdev && vdev->vnode->next){
		vnode = vdev->vnode->next;
		if (vnode->id < MODULE_NUM)
			ops = get_online_ops(vnode->id);
		if (ops && ops->is_completed)
			return ops->is_completed((struct cam_buf_ctx *)vnode->ich_subdev[0]);
	} else {
		pr_err("cam_is_completed null\n");
	}
	return false;
}

int cam_qbuf(struct cam_buf_ctx *ctx, struct cam_buf *buf)
{
	return -EBUSY;
}

struct cam_buf *cam_dqbuf(struct cam_buf_ctx *ctx)
{
	return NULL;
}

int cam_buf_ctx_init(struct cam_buf_ctx *ctx, struct device *dev, void *data,
		     bool has_internal_buf)
{
	return 0;
}

void cam_buf_ctx_release(struct cam_buf_ctx *ctx)
{
}

phys_addr_t get_phys_addr(struct cam_buf *buf, unsigned int plane)
{
	if (unlikely(!buf))
		return 0;
	return (phys_addr_t)((struct vio_frame *)buf)->vbuf.iommu_paddr[0][plane];
}

void sif_set_frame_des(struct cam_buf_ctx *buf_ctx, void *data)
{
	struct vio_subdev *subdev = (struct vio_subdev *)buf_ctx;
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

void sif_get_frame_des(struct cam_buf_ctx *buf_ctx)
{
	struct vio_subdev *subdev = (struct vio_subdev *)buf_ctx;

	if (subdev)
		vio_get_frame_id(subdev->vnode);
}

void cam_drop(struct cam_buf_ctx *buf_ctx)
{
	struct vio_subdev *subdev = (struct vio_subdev *)buf_ctx;

	if (subdev)
		vio_frame_ndone(subdev);
}

void cam_set_stat_info(struct cam_buf_ctx *buf_ctx, u32 type)
{
	struct vio_subdev *subdev = (struct vio_subdev *)buf_ctx;
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

bool cam_osd_update(struct cam_buf_ctx *buf_ctx)
{
	struct vio_subdev *subdev = (struct vio_subdev *)buf_ctx;
	struct vio_node *vnode;
	const struct cam_online_ops *ops = NULL;

	if (unlikely(!subdev))
		return false;

	vnode = (struct vio_node *)subdev->vnode;

	if (vnode && vnode->id < MODULE_NUM) {
		ops = get_online_ops(vnode->id);
		if (ops && ops->osd_update)
			return ops->osd_update(buf_ctx);
	}

	return false;
}
