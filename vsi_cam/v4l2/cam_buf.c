// SPDX-License-Identifier: GPL-2.0-only
#include <media/videobuf2-dma-contig.h>

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
	bool enabled;
};

static enum vb2_memory memory = VB2_MEMORY_MMAP;

static int cam_queue_setup(struct vb2_queue *vq,
			   unsigned int *num_buffers, unsigned int *num_planes,
			   unsigned int sizes[], struct device *alloc_devs[])
{
	struct cam_buf_ctx *cbc = (struct cam_buf_ctx *)vq->drv_priv;
	struct local_buf_ctx *lbc = (struct local_buf_ctx *)cbc->priv;

	if (lbc->ops && lbc->ops->queue_setup)
		return lbc->ops->queue_setup
				(cbc, num_buffers, num_planes, sizes, alloc_devs);
	return 0;
}

static void cam_buf_queue(struct vb2_buffer *vb)
{
	struct cam_buf *buf = vb2_buf_to_cam_buf(vb);
	struct cam_buf_ctx *cbc = (struct cam_buf_ctx *)vb->vb2_queue->drv_priv;
	struct local_buf_ctx *lbc = (struct local_buf_ctx *)cbc->priv;

	list_add_tail(&buf->entry, &lbc->queued_list);
}

static int cam_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	return 0;
}

static void cam_stop_streaming(struct vb2_queue *vq)
{
	struct cam_buf_ctx *cbc = (struct cam_buf_ctx *)vq->drv_priv;
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

static const struct vb2_ops cam_vb2_ops = {
	.queue_setup = cam_queue_setup,
	.buf_queue = cam_buf_queue,
	.start_streaming = cam_start_streaming,
	.stop_streaming = cam_stop_streaming,
};

static struct local_buf_ctx *create_local_buf_ctx(struct device *dev,
						  void *priv)
{
	struct local_buf_ctx *ctx;
	int rc;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;

	INIT_LIST_HEAD(&ctx->queued_list);
	INIT_LIST_HEAD(&ctx->done_list);
	mutex_init(&ctx->lock);
	spin_lock_init(&ctx->buflock);

	ctx->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ctx->queue.drv_priv = priv;
	ctx->queue.ops = &cam_vb2_ops;
	ctx->queue.io_modes = VB2_MMAP;
	ctx->queue.mem_ops = &vb2_dma_contig_memops;
	ctx->queue.buf_struct_size = sizeof(struct cam_buf);
	ctx->queue.min_buffers_needed = 2;
	ctx->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	ctx->queue.lock = &ctx->lock;
	ctx->queue.dev = dev;

	rc = vb2_core_queue_init(&ctx->queue);
	if (rc < 0)
		return NULL;

	return ctx;
}

static void destroy_local_buf_ctx(struct local_buf_ctx *ctx)
{
	vb2_core_queue_release(&ctx->queue);
	kfree(ctx);
}

int cam_reqbufs(struct cam_buf_ctx *ctx, unsigned int num,
		struct cam_buf_ops *ops)
{
	struct local_buf_ctx *lbc;
	unsigned int i;
	int rc;

	if (!ctx || !ctx->priv)
		return -EINVAL;

	lbc = ctx->priv;

	lbc->ops = ops;

	if (lbc->enabled && lbc->count && !num) {
		rc = vb2_core_streamoff(&lbc->queue, lbc->queue.type);
		if (rc < 0)
			return rc;
		lbc->enabled = false;
	}

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
		lbc->enabled = true;
	}
	lbc->count = num;
	return 0;

_err:
	num = 0;
	vb2_core_reqbufs(&lbc->queue, memory, 0, &num);
	return rc;
}

static inline int cam_remote_qbuf(struct cam_buf_ctx *ctx, struct cam_buf *buf)
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

	if (!vctx)
		return -EINVAL;

	return vctx->qbuf(vctx, buf);
}

static inline int cam_local_qbuf(struct cam_buf_ctx *ctx, struct cam_buf *buf)
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

int cam_qbuf_irq(struct cam_buf_ctx *ctx, struct cam_buf *buf, bool remote)
{
	int rc;

	if (remote)
		rc = cam_remote_qbuf(ctx, buf);
	else
		rc = cam_local_qbuf(ctx, buf);
	return rc;
}

static inline struct cam_buf *cam_remote_dqbuf(struct cam_buf_ctx *ctx)
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

	if (!vctx)
		return NULL;

	return vctx->dqbuf(vctx);
}

static inline struct cam_buf *cam_local_dqbuf(struct cam_buf_ctx *ctx)
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

struct cam_buf *cam_dqbuf_irq(struct cam_buf_ctx *ctx, bool remote)
{
	struct cam_buf *buf;

	if (remote)
		buf = cam_remote_dqbuf(ctx);
	else
		buf = cam_local_dqbuf(ctx);
	return buf;
}

struct cam_buf *cam_acqbuf_irq(struct cam_buf_ctx *ctx)
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

int cam_trigger(struct cam_buf_ctx *ctx)
{
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	struct v4l2_buf_ctx *vctx = NULL;

	if (!ctx || !ctx->pad)
		return -EINVAL;

	pad = media_pad_remote_pad_first(ctx->pad);
	if (!pad)
		return -EINVAL;

	if (is_media_entity_v4l2_video_device(pad->entity)) {
		return -EINVAL;
	} else if (is_media_entity_v4l2_subdev(pad->entity)) {
		sd = media_entity_to_v4l2_subdev(pad->entity);
		if (sd)
			vctx = (struct v4l2_buf_ctx *)v4l2_get_subdevdata(sd);
	}

	if (!vctx)
		return -EINVAL;

	vctx->trigger(vctx);
	return 0;
}

bool cam_is_completed(struct cam_buf_ctx *ctx)
{
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	struct v4l2_buf_ctx *vctx = NULL;

	if (!ctx || !ctx->pad)
		return true;

	pad = media_pad_remote_pad_first(ctx->pad);
	if (!pad)
		return true;

	if (is_media_entity_v4l2_video_device(pad->entity)) {
		return true;
	} else if (is_media_entity_v4l2_subdev(pad->entity)) {
		sd = media_entity_to_v4l2_subdev(pad->entity);
		if (sd)
			vctx = (struct v4l2_buf_ctx *)v4l2_get_subdevdata(sd);
	}

	if (!vctx)
		return true;

	return vctx->is_completed(vctx);
}

int cam_qbuf(struct cam_buf_ctx *ctx, struct cam_buf *buf)
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

struct cam_buf *cam_dqbuf(struct cam_buf_ctx *ctx)
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

int cam_buf_ctx_init(struct cam_buf_ctx *ctx, struct device *dev, void *data,
		     bool has_internal_buf)
{
	struct local_buf_ctx *lbc;

	if (!ctx)
		return -EINVAL;

	if (ctx->pad || ctx->priv)
		return -EINVAL;

	ctx->pad = (struct media_pad *)data;

	if (has_internal_buf) {
		lbc = create_local_buf_ctx(dev, ctx);
		if (!lbc)
			return -EFAULT;
		ctx->priv = lbc;
	}
	return 0;
}

void cam_buf_ctx_release(struct cam_buf_ctx *ctx)
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

phys_addr_t get_phys_addr(struct cam_buf *buf, unsigned int plane)
{
	if (unlikely(!buf))
		return 0;
	return (phys_addr_t)
			vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, plane);
}

void sif_set_frame_des(struct cam_buf_ctx *buf_ctx, void *data)
{
}

void sif_get_frame_des(struct cam_buf_ctx *buf_ctx)
{
}

void cam_drop(struct cam_buf_ctx *buf_ctx)
{
}

void cam_set_stat_info(struct cam_buf_ctx *buf_ctx, u32 type)
{
}
