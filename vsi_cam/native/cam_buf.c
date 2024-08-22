// SPDX-License-Identifier: GPL-2.0-only
#include <linux/err.h>

#include "vio_node_api.h"

#include "cam_buf.h"

int cam_reqbufs(struct cam_ctx *ctx, unsigned int num,
		struct cam_buf_ops *ops)
{
	return -EBUSY;
}

int cam_qbuf_irq(struct cam_ctx *ctx, struct cam_buf *buf, bool remote)
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

struct cam_buf *cam_dqbuf_irq(struct cam_ctx *ctx, bool remote)
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
			if (subdev->id == VNODE_ID_SRC) {
				(void)memcpy(&vnode->frameid, &frame->frameinfo.frameid,
					sizeof(struct frame_id_desc));
			}
			vio_set_stat_info(vnode->flow_id, vnode->id, STAT_DQ,
					  vnode->frameid.frame_id);
		}
	}
	return (struct cam_buf *)frame;
}

struct cam_buf *cam_acqbuf_irq(struct cam_ctx *ctx)
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

int cam_qbuf(struct cam_ctx *ctx, struct cam_buf *buf)
{
	return -EBUSY;
}

struct cam_buf *cam_dqbuf(struct cam_ctx *ctx)
{
	return NULL;
}

int cam_buf_ctx_init(struct cam_ctx *ctx, struct device *dev, void *data,
		     bool has_internal_buf)
{
	return 0;
}

void cam_buf_ctx_release(struct cam_ctx *ctx)
{
}

phys_addr_t get_phys_addr(struct cam_buf *buf, unsigned int plane)
{
	if (unlikely(!buf))
		return 0;
	return (phys_addr_t)((struct vio_frame *)buf)->vbuf.iommu_paddr[0][plane];
}

void cam_drop(struct cam_ctx *ctx)
{
	struct vio_subdev *subdev = (struct vio_subdev *)ctx;

	if (subdev)
		vio_frame_ndone(subdev);
}
