// SPDX-License-Identifier: GPL-2.0-only
#include <linux/err.h>

#include "vio_framemgr.h"
#include "vio_node_api.h"

#include "cam_buf.h"

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
		if (frame == NULL) {
			frame = peek_frame(framemgr, FS_COMPLETE);
		}
		vio_x_barrier_irqr(framemgr, flags);

		if (frame == NULL) {
			pr_info("[WARN] [%s][S%d] %s: REQUEST and COMPLETE queue have no member\n",
				subdev->name, subdev->vnode->flow_id, __func__);
			framemgr_print_queues(framemgr);
			return NULL;
		}

		if (frame) {
			vio_e_barrier_irqs(framemgr, flags);
			trans_frame(framemgr, frame, FS_PROCESS);
			vio_x_barrier_irqr(framemgr, flags);
			vnode = (struct vio_node *)subdev->vnode;
			if (subdev->id == VNODE_ID_SRC) {
				(void)memcpy(&vnode->frameid, &frame->frameinfo.frameid,
					sizeof(struct frame_id_desc));
				pr_debug("[%s][S%d] dqbuf set frame_des %d to %s vnode\n",
					vnode->name, vnode->flow_id,
					vnode->frameid.frame_id, vnode->name);
			}
			vio_set_stat_info(vnode->flow_id, vnode->id, STAT_DQ,
					  vnode->frameid.frame_id);
		}
	}
	return (struct cam_buf *)frame;
}

struct cam_buf *cam_acqbuf_irq(struct cam_ctx *ctx, bool remote)
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

phys_addr_t get_phys_addr(struct device *dev, struct cam_buf *buf, unsigned int plane)
{
	if (unlikely(!buf))
		return 0;
	return (phys_addr_t)((struct vio_frame *)buf)->vbuf.iommu_paddr[0][plane];
}

unsigned long get_buf_size(struct cam_buf *buf, unsigned int plane)
{
	if (unlikely(!buf))
		return 0;
	return ((struct vio_frame *)buf)->vbuf.group_info.info[0].planeSize[plane];
}

int cam_drop_irq(struct cam_ctx *ctx, struct cam_buf *buf)
{
	struct vio_subdev *subdev = (struct vio_subdev *)ctx;

	if (subdev)
		vio_frame_ndone(subdev);
	return 0;
}

int cam_drop_irq_ext(struct cam_ctx *ctx, struct cam_buf *buf)
{
	return 0;
}

int cam_drop(struct cam_ctx *ctx, struct cam_buf *buf)
{
	return 0;
}

void cam_iommu_unmap(struct cam_buf *buf)
{
}
