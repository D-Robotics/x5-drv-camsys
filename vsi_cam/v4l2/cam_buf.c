// SPDX-License-Identifier: GPL-2.0-only
#include <media/videobuf2-dma-contig.h>

#include "mem_helper.h"
#include "utils.h"

#include "cam_buf.h"

struct local_buf_ctx {
	struct vb2_queue queue;
	struct mutex lock; /* lock for vb2 queue */
	unsigned int count;
	struct list_head queued_list;
	struct list_head done_list;
	spinlock_t buflock; /* lock for local buf */
	struct cam_buf_ops *ops;
	bool en_reqbufs;
};

static enum vb2_memory memory = VB2_MEMORY_MMAP;

static int cam_queue_setup(struct vb2_queue *vq,
			   unsigned int *num_buffers, unsigned int *num_planes,
			   unsigned int sizes[], struct device *alloc_devs[])
{
	struct cam_ctx *cbc = (struct cam_ctx *)vb2_get_drv_priv(vq);
	struct local_buf_ctx *lbc = (struct local_buf_ctx *)cbc->priv;

	if (lbc->ops && lbc->ops->queue_setup)
		return lbc->ops->queue_setup
				(cbc, num_buffers, num_planes, sizes, alloc_devs);
	return 0;
}

static void cam_buf_queue(struct vb2_buffer *vb)
{
	struct cam_buf *buf = vb2_buf_to_cam_buf(vb);
	struct cam_ctx *cbc = (struct cam_ctx *)vb2_get_drv_priv(vb->vb2_queue);
	struct local_buf_ctx *lbc = (struct local_buf_ctx *)cbc->priv;

	list_add_tail(&buf->entry, &lbc->queued_list);
}

static int cam_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	return 0;
}

static void cam_stop_streaming(struct vb2_queue *vq)
{
	struct cam_ctx *cbc = (struct cam_ctx *)vb2_get_drv_priv(vq);
	struct local_buf_ctx *lbc = (struct local_buf_ctx *)cbc->priv;
	struct cam_buf *buf, *node;
	struct vb2_buffer *vb;

	list_for_each_entry_safe(buf, node, &lbc->queued_list, entry)
		list_del(&buf->entry);

	list_for_each_entry_safe(buf, node, &lbc->done_list, entry)
		list_del(&buf->entry);

	list_for_each_entry(vb, &lbc->queue.queued_list, queued_entry)
		vb2_buffer_done(vb, VB2_BUF_STATE_QUEUED);
}

static void cam_buf_cleanup(struct vb2_buffer *vb)
{
	struct cam_buf *buf = vb2_buf_to_cam_buf(vb);

	cam_iommu_unmap(buf);
}

static const struct vb2_ops cam_vb2_ops = {
	.queue_setup = cam_queue_setup,
	.buf_queue = cam_buf_queue,
	.start_streaming = cam_start_streaming,
	.stop_streaming = cam_stop_streaming,
	.buf_cleanup = cam_buf_cleanup,
};

static struct local_buf_ctx *create_local_buf_ctx(struct init_attr *attr,
						  struct cam_ctx *arg)
{
	struct local_buf_ctx *ctx;
	int rc;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;

	INIT_LIST_HEAD(&ctx->queued_list);
	INIT_LIST_HEAD(&ctx->done_list);
	spin_lock_init(&ctx->buflock);
	ctx->en_reqbufs = attr->en_reqbufs;
	if (attr->en_reqbufs) {
		if (!attr->dev) {
			kfree(ctx);
			return NULL;
		}

		mutex_init(&ctx->lock);
		ctx->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		ctx->queue.drv_priv = arg;
		ctx->queue.ops = &cam_vb2_ops;
		ctx->queue.io_modes = VB2_MMAP;
		ctx->queue.mem_ops = &vb2_dma_contig_memops;
		ctx->queue.buf_struct_size = sizeof(struct cam_buf);
		ctx->queue.min_buffers_needed = 2;
		ctx->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
		ctx->queue.lock = &ctx->lock;
		ctx->queue.dev = attr->dev;

		rc = vb2_core_queue_init(&ctx->queue);
		if (rc < 0) {
			kfree(ctx);
			return NULL;
		}
	}
	return ctx;
}

static void destroy_local_buf_ctx(struct local_buf_ctx *ctx)
{
	if (ctx->en_reqbufs)
		vb2_core_queue_release(&ctx->queue);
	mutex_destroy(&ctx->lock);
	kfree(ctx);
}

int cam_reqbufs(struct cam_ctx *ctx, unsigned int num,
		struct cam_buf_ops *ops)
{
	struct local_buf_ctx *lbc;
	unsigned int i;
	int rc;

	if (!ctx || !ctx->priv)
		return -EINVAL;

	lbc = ctx->priv;

	if (!lbc->en_reqbufs)
		return -EINVAL;

	lbc->ops = ops;

	if (lbc->count && !num) {
		rc = vb2_core_streamoff(&lbc->queue, lbc->queue.type);
		if (rc < 0)
			return rc;
	}

	if (lbc->count == num)
		return 0;

	rc = vb2_core_reqbufs(&lbc->queue, memory, 0, &num);
	if (rc < 0)
		return rc;

	for (i = 0; i < num; i++) {
		rc = vb2_core_qbuf(&lbc->queue, i, NULL, NULL);
		if (rc < 0)
			goto _err;
	}

	if (num > 0) {
		rc = vb2_core_streamon(&lbc->queue, lbc->queue.type);
		if (rc < 0)
			goto _err;
	}
	lbc->count = num;
	return 0;

_err:
	num = 0;
	vb2_core_reqbufs(&lbc->queue, memory, 0, &num);
	return rc;
}

static inline int cam_remote_qbuf(struct cam_ctx *ctx, struct cam_buf *buf)
{
	struct video_device *vdev;
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	struct v4l2_buf_ctx *vctx = NULL;

	if (!ctx || !ctx->pad)
		return -EINVAL;

	pad = media_pad_remote_pad_first(ctx->pad);
	if (!pad)
		return -ENOLINK;

	if (is_media_entity_v4l2_video_device(pad->entity)) {
		vdev = media_entity_to_video_device(pad->entity);
		if (vdev)
			vctx = (struct v4l2_buf_ctx *)video_get_drvdata(vdev);
	} else if (is_media_entity_v4l2_subdev(pad->entity)) {
		sd = media_entity_to_v4l2_subdev(pad->entity);
		if (sd)
			vctx = (struct v4l2_buf_ctx *)v4l2_get_subdevdata(sd);
	}

	if (!vctx || !vctx->qbuf)
		return -EINVAL;

	return vctx->qbuf(vctx, pad->index, buf);
}

static inline int cam_local_qbuf(struct cam_ctx *ctx, struct cam_buf *buf)
{
	struct local_buf_ctx *lbc;
	unsigned long flags;

	if (!ctx || !ctx->priv || !buf)
		return -EINVAL;

	lbc = ctx->priv;

	spin_lock_irqsave(&lbc->buflock, flags);
	list_add_tail(&buf->entry, &lbc->queued_list);
	spin_unlock_irqrestore(&lbc->buflock, flags);
	return 0;
}

int cam_qbuf_irq(struct cam_ctx *ctx, struct cam_buf *buf, bool remote)
{
	if (remote)
		return cam_remote_qbuf(ctx, buf);
	return cam_local_qbuf(ctx, buf);
}

static inline struct cam_buf *cam_remote_dqbuf(struct cam_ctx *ctx)
{
	struct video_device *vdev;
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	struct v4l2_buf_ctx *vctx = NULL;

	if (!ctx || !ctx->pad)
		return NULL;

	pad = media_pad_remote_pad_first(ctx->pad);
	if (!pad)
		return NULL;

	if (is_media_entity_v4l2_video_device(pad->entity)) {
		vdev = media_entity_to_video_device(pad->entity);
		if (vdev)
			vctx = (struct v4l2_buf_ctx *)video_get_drvdata(vdev);
	} else if (is_media_entity_v4l2_subdev(pad->entity)) {
		sd = media_entity_to_v4l2_subdev(pad->entity);
		if (sd)
			vctx = (struct v4l2_buf_ctx *)v4l2_get_subdevdata(sd);
	}

	if (!vctx || !vctx->dqbuf)
		return NULL;

	return vctx->dqbuf(vctx, pad->index);
}

static inline struct cam_buf *cam_local_dqbuf(struct cam_ctx *ctx)
{
	struct local_buf_ctx *lbc;
	struct cam_buf *buf;
	unsigned long flags;

	if (!ctx || !ctx->priv)
		return NULL;

	lbc = ctx->priv;

	spin_lock_irqsave(&lbc->buflock, flags);
	buf = list_first_entry_or_null(&lbc->done_list, struct cam_buf, entry);
	if (buf)
		list_del(&buf->entry);
	spin_unlock_irqrestore(&lbc->buflock, flags);
	return buf;
}

struct cam_buf *cam_dqbuf_irq(struct cam_ctx *ctx, bool remote)
{
	if (remote)
		return cam_remote_dqbuf(ctx);
	return cam_local_dqbuf(ctx);
}

static inline struct cam_buf *cam_remote_acqbuf(struct cam_ctx *ctx)
{
	struct video_device *vdev;
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	struct v4l2_buf_ctx *vctx = NULL;

	if (!ctx || !ctx->pad)
		return NULL;

	pad = media_pad_remote_pad_first(ctx->pad);
	if (!pad)
		return NULL;

	if (is_media_entity_v4l2_video_device(pad->entity)) {
		vdev = media_entity_to_video_device(pad->entity);
		if (vdev)
			vctx = (struct v4l2_buf_ctx *)video_get_drvdata(vdev);
	} else if (is_media_entity_v4l2_subdev(pad->entity)) {
		sd = media_entity_to_v4l2_subdev(pad->entity);
		if (sd)
			vctx = (struct v4l2_buf_ctx *)v4l2_get_subdevdata(sd);
	}

	if (!vctx || !vctx->acqbuf)
		return NULL;

	return vctx->acqbuf(vctx, pad->index);
}

static inline struct cam_buf *cam_local_acqbuf(struct cam_ctx *ctx)
{
	struct local_buf_ctx *lbc = ctx->priv;
	struct cam_buf *buf;
	unsigned long flags;

	if (!lbc)
		return NULL;

	spin_lock_irqsave(&lbc->buflock, flags);
	buf = list_first_entry_or_null(&lbc->done_list, struct cam_buf, entry);
	spin_unlock_irqrestore(&lbc->buflock, flags);
	return buf;
}

struct cam_buf *cam_acqbuf_irq(struct cam_ctx *ctx, bool remote)
{
	if (remote)
		return cam_remote_acqbuf(ctx);
	return cam_local_acqbuf(ctx);
}

int cam_qbuf(struct cam_ctx *ctx, struct cam_buf *buf)
{
	struct local_buf_ctx *lbc;
	unsigned long flags;

	if (!ctx || !ctx->priv || !buf)
		return -EINVAL;

	lbc = ctx->priv;

	spin_lock_irqsave(&lbc->buflock, flags);
	list_add_tail(&buf->entry, &lbc->done_list);
	spin_unlock_irqrestore(&lbc->buflock, flags);
	return 0;
}

struct cam_buf *cam_dqbuf(struct cam_ctx *ctx)
{
	struct local_buf_ctx *lbc;
	struct cam_buf *buf;
	unsigned long flags;

	if (!ctx || !ctx->priv)
		return NULL;

	lbc = ctx->priv;

	spin_lock_irqsave(&lbc->buflock, flags);
	buf = list_first_entry_or_null(&lbc->queued_list, struct cam_buf, entry);
	if (buf)
		list_del(&buf->entry);
	spin_unlock_irqrestore(&lbc->buflock, flags);
	return buf;
}

struct cam_buf *cam_acqbuf(struct cam_ctx *ctx)
{
	struct local_buf_ctx *lbc;
	struct cam_buf *buf;
	unsigned long flags;

	if (!ctx || !ctx->priv)
		return NULL;

	lbc = ctx->priv;

	spin_lock_irqsave(&lbc->buflock, flags);
	buf = list_first_entry_or_null(&lbc->queued_list, struct cam_buf, entry);
	spin_unlock_irqrestore(&lbc->buflock, flags);
	return buf;
}

int cam_buf_ctx_init(struct cam_ctx *ctx, void *data, struct init_attr *attr)
{
	struct local_buf_ctx *lbc;

	if (!ctx)
		return -EINVAL;

	if (ctx->pad || ctx->priv)
		return -EINVAL;

	ctx->pad = (struct media_pad *)data;

	if (attr) {
		lbc = create_local_buf_ctx(attr, ctx);
		if (!lbc)
			return -EFAULT;
		ctx->priv = lbc;
	}
	return 0;
}

void cam_buf_ctx_release(struct cam_ctx *ctx)
{
	struct local_buf_ctx *lbc;

	if (!ctx)
		return;

	if (ctx->priv) {
		lbc = ctx->priv;
		if (lbc->count > 0)
			cam_reqbufs(ctx, 0, NULL);
		destroy_local_buf_ctx(lbc);
		ctx->priv = NULL;
	}
	ctx->pad = NULL;
}

static inline bool is_addr_mapped(struct device *dev, struct cam_buf *buf,
				  unsigned int plane, phys_addr_t *addr)
{
	u32 i;
	bool mapped = false;

	for (i = 0; i < ARRAY_SIZE(buf->iova); i++) {
		if (buf->iova[i].dev == dev && buf->iova[i].piova[plane].addr) {
			*addr = buf->iova[i].piova[plane].addr;
			mapped = true;
			break;
		}
	}
	return mapped;
}

static inline int add_mapped_addr(struct device *dev, struct cam_buf *buf,
				  unsigned int plane, phys_addr_t iova, u32 size)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(buf->iova); i++) {
		if (!buf->iova[i].dev) {
			buf->iova[i].piova[plane].addr = iova;
			buf->iova[i].piova[plane].size = size;
			buf->iova[i].dev = dev;
			break;
		}
	}

	if (i == ARRAY_SIZE(buf->iova))
		return -EINVAL;

	return 0;
}

phys_addr_t get_phys_addr(struct device *dev, struct cam_buf *buf,
			  unsigned int plane)
{
	struct vb2_buffer *vb;
	dma_addr_t addr, iova;
	size_t size;
	int rc;

	if (unlikely(!buf))
		return 0;

	vb = &buf->vb.vb2_buf;
	addr = vb2_dma_contig_plane_dma_addr(vb, plane);
	if (dev && addr) {
		if (is_addr_mapped(dev, buf, plane, &addr))
			goto _exit;
		size = vb2_plane_size(vb, plane);
		rc = mem_iommu_map(dev, addr, size, &iova);
		if (rc < 0) {
			dev_warn(dev, "failed to call mem_iommu_map (err=%d)\n", rc);
			goto _exit;
		}
		rc = add_mapped_addr(dev, buf, plane, iova, size);
		if (rc < 0) {
			mem_iommu_unmap(dev, iova, size);
			goto _exit;
		}
		return (phys_addr_t)iova;
	}
_exit:
	return (phys_addr_t)addr;
}

unsigned long get_buf_size(struct cam_buf *buf, unsigned int plane)
{
	return 0;
}

int cam_drop_irq(struct cam_ctx *ctx, struct cam_buf *buf)
{
	struct video_device *vdev;
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	struct v4l2_buf_ctx *vctx = NULL;

	if (!ctx || !ctx->pad)
		return -EINVAL;

	pad = media_pad_remote_pad_first(ctx->pad);
	if (!pad)
		return -ENOLINK;

	if (is_media_entity_v4l2_video_device(pad->entity)) {
		vdev = media_entity_to_video_device(pad->entity);
		if (vdev)
			vctx = (struct v4l2_buf_ctx *)video_get_drvdata(vdev);
	} else if (is_media_entity_v4l2_subdev(pad->entity)) {
		sd = media_entity_to_v4l2_subdev(pad->entity);
		if (sd)
			vctx = (struct v4l2_buf_ctx *)v4l2_get_subdevdata(sd);
	}

	if (!vctx || !vctx->drop)
		return -EINVAL;

	return vctx->drop(vctx, pad->index, buf);
}

int cam_drop_irq_ext(struct cam_ctx *ctx, struct cam_buf *buf)
{
	return cam_drop_irq(ctx, buf);
}

int cam_drop(struct cam_ctx *ctx, struct cam_buf *buf)
{
	struct local_buf_ctx *lbc;
	unsigned long flags;

	if (!ctx || !ctx->priv || !buf)
		return -EINVAL;

	lbc = ctx->priv;

	spin_lock_irqsave(&lbc->buflock, flags);
	list_add_tail(&buf->entry, &lbc->queued_list);
	spin_unlock_irqrestore(&lbc->buflock, flags);
	return 0;
}

int cam_ready(struct cam_ctx *ctx, int on)
{
	struct video_device *vdev;
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	struct v4l2_buf_ctx *vctx = NULL;

	if (!ctx || !ctx->pad)
		return -EINVAL;

	pad = media_pad_remote_pad_first(ctx->pad);
	if (!pad)
		return -ENOLINK;

	if (is_media_entity_v4l2_video_device(pad->entity)) {
		vdev = media_entity_to_video_device(pad->entity);
		if (vdev)
			vctx = (struct v4l2_buf_ctx *)video_get_drvdata(vdev);
	} else if (is_media_entity_v4l2_subdev(pad->entity)) {
		sd = media_entity_to_v4l2_subdev(pad->entity);
		if (sd)
			vctx = (struct v4l2_buf_ctx *)v4l2_get_subdevdata(sd);
	}

	if (!vctx || !vctx->ready)
		return -EINVAL;

	vctx->ready(vctx, pad->index, on);
	return 0;
}

void cam_iommu_unmap(struct cam_buf *buf)
{
	struct cam_iova *piova;
	struct device *dev;
	u32 i, j;
	int rc;

	for (i = 0; i < ARRAY_SIZE(buf->iova); i++) {
		if (!buf->iova[i].dev)
			continue;
		dev = buf->iova[i].dev;
		for (j = 0; j < ARRAY_SIZE(buf->iova[i].piova); j++) {
			piova = &buf->iova[i].piova[j];
			if (!piova->addr)
				continue;
			rc = mem_iommu_unmap(dev, piova->addr, piova->size);
			if (rc < 0)
				dev_warn(dev, "failed to call mem_iommu_unmap (err=%d)\n", rc);
		}
	}

}
