// SPDX-License-Identifier: GPL-2.0-only
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "cam_buf.h"
#include "cam_uapi.h"
#include "utils.h"
#include "video_fmt.h"
#include "video.h"
#include "v4l2_usr_api.h"

static int scene;
module_param(scene, int, 0644);

struct entity_link {
	const char *src_name;
	u16 src_pad;
	const char *sink_name;
	u16 sink_pad;
	u32 flags;
};

static struct entity_link links0[] = {
	{ SIF_DEV_NAME "0-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "1-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "2-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "3-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
};

static struct entity_link links2[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-0", 0, GDC_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-0", 0, VSE_DEV_NAME "0-4", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 5, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 2, "video", 0, MEDIA_LNK_FL_ENABLED },
};

static struct entity_link links6[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-0", 0, VSE_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 2, "video", 0, MEDIA_LNK_FL_ENABLED },
};

static struct entity_link links9[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "1-0", 0, ISP_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "2-0", 0, ISP_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-0", 0, VSE_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 5, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 3, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 4, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-1", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-2", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
};

static struct entity_link links10[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "2-0", 0, ISP_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-0", 0, VSE_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-2", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "3-0", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
};

static struct entity_link links11[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "1-0", 0, ISP_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "2-0", 0, ISP_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "3-0", 0, ISP_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "0-1", 0, ISP_DEV_NAME "0-4", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-1", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-2", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-3", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-4", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
};

static struct entity_link links12[] = {
	{ SIF_DEV_NAME "0-0", 0, VSE_DEV_NAME "0-4", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 5, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
};

static struct entity_link links13[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "1-0", 0, ISP_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "2-0", 0, ISP_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "3-0", 0, ISP_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-0", 0, VSE_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-1", 0, VSE_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-2", 0, VSE_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-3", 0, VSE_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-1", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-2", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-3", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
};

static struct entity_link *links[] = {
	links0,
	NULL,
	links2,
	NULL,
	NULL,
	NULL,
	links6,
	NULL,
	NULL,
	links9,
	links10,
	links11,
	links12,
	links13,
};

static u32 links_size[] = {
	ARRAY_SIZE(links0),
	0,
	ARRAY_SIZE(links2),
	0,
	0,
	0,
	ARRAY_SIZE(links6),
	0,
	0,
	ARRAY_SIZE(links9),
	ARRAY_SIZE(links10),
	ARRAY_SIZE(links11),
	ARRAY_SIZE(links12),
	ARRAY_SIZE(links13),
};

struct vid_video_device {
	struct video_device video;
	struct vb2_queue queue;
	struct mutex lock; /* lock for vb2 queue */
	struct media_pad pad;
	struct media_pipeline pipe;
	struct v4l2_format fmt;
	struct v4l2_buf_ctx bctx;
	spinlock_t irqlock; /* lock for cam buf */
	u32 buf_sequence;
	u32 id;
	struct list_head queued_list;
	struct list_head entry;
};

static int vid_qbuf(struct v4l2_buf_ctx *ctx, struct cam_buf *buf)
{
	struct vid_video_device *vdev =
		container_of(ctx, struct vid_video_device, bctx);

	if (!buf || buf->vb.vb2_buf.state != VB2_BUF_STATE_ACTIVE)
		return -EINVAL;

	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, vdev->fmt.fmt.pix.sizeimage);
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = vdev->buf_sequence++;
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	return 0;
}

static struct cam_buf *vid_dqbuf(struct v4l2_buf_ctx *ctx)
{
	struct vid_video_device *vdev =
		container_of(ctx, struct vid_video_device, bctx);
	unsigned long flags;
	struct cam_buf *buf;

	spin_lock_irqsave(&vdev->irqlock, flags);
	buf = list_first_entry_or_null(&vdev->queued_list, struct cam_buf,
				       entry);
	if (buf)
		list_del(&buf->entry);
	spin_unlock_irqrestore(&vdev->irqlock, flags);
	return buf;
}

static int vid_queue_setup(struct vb2_queue *vq, unsigned int *num_buffers,
			   unsigned int *num_planes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct vid_video_device *vdev = (struct vid_video_device *)vq->drv_priv;
	unsigned int size = vdev->fmt.fmt.pix.sizeimage;

	if (!size)
		return -ENOMEM;

	if (!*num_buffers)
		*num_buffers = 1;

	*num_planes = 1;
	sizes[0] = size;
	return 0;
}

static inline void notify_buf_ready(struct vid_video_device *dev, int on)
{
	struct media_pad *pad = media_pad_remote_pad_first(&dev->pad);
	struct v4l2_subdev *sd;
	struct v4l2_buf_ctx *ctx;

	if (pad) {
		sd = media_entity_to_v4l2_subdev(pad->entity);
		ctx = v4l2_get_subdevdata(sd);

		if (ctx && ctx->ready)
			ctx->ready(ctx, pad->index, on);
	}
}

static void vid_buf_queue(struct vb2_buffer *vb)
{
	struct vid_video_device *vdev =
		(struct vid_video_device *)vb->vb2_queue->drv_priv;
	struct cam_buf *buf = vb2_buf_to_cam_buf(vb);
	unsigned long flags;

	spin_lock_irqsave(&vdev->irqlock, flags);
	list_add_tail(&buf->entry, &vdev->queued_list);
	spin_unlock_irqrestore(&vdev->irqlock, flags);

	notify_buf_ready(vdev, 1);
}

static void vid_return_all_buffers(struct vid_video_device *dev,
				   enum vb2_buffer_state state)
{
	struct cam_buf *buf, *node;
	struct vb2_buffer *vb;
	unsigned long flags;

	spin_lock_irqsave(&dev->irqlock, flags);
	list_for_each_entry_safe(buf, node, &dev->queued_list, entry)
		list_del(&buf->entry);
	spin_unlock_irqrestore(&dev->irqlock, flags);

	list_for_each_entry(vb, &dev->queue.queued_list, queued_entry)
		if (vb->state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(vb, state);
}

#define pad_to_remote_sd(pad)                                             \
	({                                                                \
		struct media_pad *_pad = media_pad_remote_pad_first(pad); \
		struct v4l2_subdev *_sd = NULL;                           \
		if (_pad)                                                 \
			_sd = media_entity_to_v4l2_subdev(_pad->entity);  \
		_sd;                                                      \
	})

static int vid_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vid_video_device *vdev = (struct vid_video_device *)vq->drv_priv;
	struct v4l2_subdev *sd = pad_to_remote_sd(&vdev->pad);
	int rc;

	vdev->buf_sequence = 0;
	rc = v4l2_subdev_call(sd, video, s_stream, 1);
	if (rc < 0)
		return rc;

//	rc = media_pipeline_start(&vdev->pad, &vdev->pipe);
//	if (rc < 0)
//		vid_return_all_buffers(vdev, VB2_BUF_STATE_QUEUED);
	return rc;
}

static void vid_stop_streaming(struct vb2_queue *vq)
{
	struct vid_video_device *vdev = (struct vid_video_device *)vq->drv_priv;
	struct v4l2_subdev *sd = pad_to_remote_sd(&vdev->pad);
	int rc;

	rc = v4l2_subdev_call(sd, video, s_stream, 0);
	if (rc < 0)
		return;

	notify_buf_ready(vdev, 0);

//	media_pipeline_stop(&vdev->pad);

	vid_return_all_buffers(vdev, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops vid_vb2_ops = {
	.queue_setup = vid_queue_setup,
	.buf_queue = vid_buf_queue,
	.start_streaming = vid_start_streaming,
	.stop_streaming = vid_stop_streaming,
};

#define file_to_video_device(file) \
	container_of(video_drvdata(file), struct vid_video_device, bctx)

static int vid_querycap(struct file *file, void *fh,
			struct v4l2_capability *cap)
{
	struct vid_video_device *vdev = file_to_video_device(file);

	strscpy(cap->driver, VID_DEV_NAME, sizeof(cap->driver));
	strscpy(cap->card, VID_DEV_NAME, sizeof(cap->card));
	snprintf((char *)cap->bus_info, sizeof(cap->bus_info),
		 "platform:vscam%d", vdev->id);
	return 0;
}

static int vid_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct v4l2_subdev *sd = pad_to_remote_sd(&vdev->pad);
	struct v4l2_buf_ctx *ctx = v4l2_get_subdevdata(sd);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (ctx && ctx->enum_format)
		return ctx->enum_format(ctx, f->index, &f->pixelformat);
	return -EINVAL;
}

static int vid_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vid_video_device *vdev = file_to_video_device(file);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (!vdev->fmt.fmt.pix.width || !vdev->fmt.fmt.pix.height)
		return -EINVAL;

	*f = vdev->fmt;
	return 0;
}

static int try_s_fmt(struct vid_video_device *vdev, struct v4l2_format *f,
		     bool is_try)
{
	struct media_pad *pad;
	struct v4l2_subdev *sd;
	struct v4l2_buf_ctx *ctx;
	struct v4l2_subdev_state state;
	struct v4l2_subdev_format s_f;
	struct video_fmt *v_f;
	u32 bytesperline, sizeimage;
	int rc;

	pad = media_pad_remote_pad_first(&vdev->pad);
	if (!pad)
		return -ENOLINK;

	sd = media_entity_to_v4l2_subdev(pad->entity);
	ctx = v4l2_get_subdevdata(sd);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	v_f = get_fmt_by_pixelformat(f->fmt.pix.pixelformat);
	if (!v_f)
		return -EINVAL;

	if (ctx && ctx->set_format) {
		rc = ctx->set_format(ctx, f->fmt.pix.pixelformat, is_try);
		if (rc < 0)
			return rc;
	}

	memset(&state, 0, sizeof(state));
	memset(&s_f, 0, sizeof(s_f));
	if (is_try)
		s_f.which = V4L2_SUBDEV_FORMAT_TRY;
	else
		s_f.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	s_f.pad = pad->index;
	s_f.format.width = f->fmt.pix.width;
	s_f.format.height = f->fmt.pix.height;
	s_f.format.code = v_f->mbus_code;
	rc = v4l2_subdev_call(sd, pad, set_fmt, &state, &s_f);
	if (rc < 0)
		return rc;

	v4l_bound_align_image(&f->fmt.pix.width, MIN_W, MAX_W, ALIGN_W,
			      &f->fmt.pix.height, MIN_H, MAX_H, ALIGN_H, 0);
	bytesperline = ALIGN(f->fmt.pix.width * v_f->bpp, STRIDE_ALIGN);
	if (v_f->bpp == 1)
		sizeimage = f->fmt.pix.height * (bytesperline * v_f->bit_depth / 8);
	else
		sizeimage = f->fmt.pix.height * bytesperline;

	if (f->fmt.pix.bytesperline < bytesperline)
		f->fmt.pix.bytesperline = bytesperline;
	if (f->fmt.pix.sizeimage < sizeimage)
		f->fmt.pix.sizeimage = sizeimage;
	return 0;
}

static int vid_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vid_video_device *vdev = file_to_video_device(file);

	return try_s_fmt(vdev, f, true);
}

static int vid_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	int rc;

	rc = try_s_fmt(vdev, f, false);
	if (rc < 0)
		return rc;

	vdev->fmt = *f;
	return 0;
}

static int vid_enum_framesizes(struct file *file, void *fh,
			       struct v4l2_frmsizeenum *fsize)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct v4l2_subdev *sd = pad_to_remote_sd(&vdev->pad);
	struct media_pad *pad = media_pad_remote_pad_first(&vdev->pad);
	struct v4l2_buf_ctx *ctx = v4l2_get_subdevdata(sd);

	if (ctx && ctx->enum_framesize)
		return ctx->enum_framesize(ctx, pad->index, fsize);
	return -EINVAL;
}

static int vid_enum_frameintervals(struct file *file, void *fh,
				   struct v4l2_frmivalenum *fival)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct v4l2_subdev *sd = pad_to_remote_sd(&vdev->pad);
	struct media_pad *pad = media_pad_remote_pad_first(&vdev->pad);
	struct v4l2_buf_ctx *ctx = v4l2_get_subdevdata(sd);

	if (ctx && ctx->enum_frameinterval)
		return ctx->enum_frameinterval(ctx, pad->index, fival);
	return -EINVAL;
}

static int vid_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	return vb2_ioctl_streamon(file, priv, i);
}

static int vid_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
	return vb2_ioctl_streamoff(file, priv, i);
}

static long vid_ioctl(struct file *file, void *fh, bool valid_prio, unsigned int cmd, void *arg)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	int rc = 0;

	switch (cmd) {
	case VIDIOC_GET_BUF_PHYS: {
		unsigned long flags;
		struct vb2_buffer *vb;
		struct cam_buf *buf;
		unsigned int *phys = arg;

		spin_lock_irqsave(&vdev->irqlock, flags);
		vb = list_last_entry(&vdev->queue.queued_list, struct vb2_buffer, queued_entry);
		buf = vb2_buf_to_cam_buf(vb);
		if (buf)
			*phys = (unsigned int)get_phys_addr(buf, 0);
		spin_unlock_irqrestore(&vdev->irqlock, flags);
	}
		break;
	default:
		rc = -ENOTTY;
	}
	return rc;
}

static int vid_s_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	return 0;
}

static int vid_g_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	return 0;
}

static int vid_s_ext_ctrls(struct file *file, void *fh, struct v4l2_ext_controls *a)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct media_pad *pad;
	struct v4l2_subdev *sd;
	int rc = 0;

	pad = media_pad_remote_pad_first(&vdev->pad);
	if (!pad)
		return -ENOLINK;

	sd = media_entity_to_v4l2_subdev(pad->entity);

	rc = v4l2_subdev_call(sd, core, command, true, a->controls);

	return rc;
}

static int vid_g_ext_ctrls(struct file *file, void *fh, struct v4l2_ext_controls *a)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct media_pad *pad;
	struct v4l2_subdev *sd;
	int rc = 0;

	pad = media_pad_remote_pad_first(&vdev->pad);
	if (!pad)
		return -ENOLINK;

	sd = media_entity_to_v4l2_subdev(pad->entity);

	rc = v4l2_subdev_call(sd, core, command, false, a->controls);

	return rc;
}

static const struct v4l2_ioctl_ops vid_ioctl_ops = {
	.vidioc_querycap = vid_querycap,
	.vidioc_enum_fmt_vid_cap = vid_enum_fmt,
	.vidioc_g_fmt_vid_cap = vid_g_fmt,
	.vidioc_try_fmt_vid_cap = vid_try_fmt,
	.vidioc_s_fmt_vid_cap = vid_s_fmt,
	.vidioc_enum_framesizes = vid_enum_framesizes,
	.vidioc_enum_frameintervals = vid_enum_frameintervals,
	.vidioc_streamon = vid_streamon,
	.vidioc_streamoff = vid_streamoff,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_default = vid_ioctl,
	.vidioc_s_ctrl = vid_s_ctrl,
	.vidioc_g_ctrl = vid_g_ctrl,
	.vidioc_s_ext_ctrls = vid_s_ext_ctrls,
	.vidioc_g_ext_ctrls = vid_g_ext_ctrls,
};

static int vid_link_setup(struct media_entity *entity,
			  const struct media_pad *local,
			  const struct media_pad *remote, u32 flags)
{
	return 0;
}

static int vid_link_validate(struct media_link *link)
{
	return 0;
}

static const struct media_entity_operations vid_media_ops = {
	.link_setup = vid_link_setup,
	.link_validate = vid_link_validate,
};

static void vid_video_device_release(struct video_device *vdev)
{
}

static const struct v4l2_file_operations video_ops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

static struct vid_video_device *create_video_device(struct vid_device *vdev,
						    u32 id)
{
	struct device *dev = vdev->v4l2_dev.dev;
	struct vid_video_device *v;
	int rc;

	v = devm_kzalloc(dev, sizeof(*v), GFP_KERNEL);
	if (!v)
		return ERR_PTR(-ENOMEM);

	snprintf(v->video.name, sizeof(v->video.name), "%s%d", VID_DEV_NAME,
		 id);

	v->id = id;

	v->video.v4l2_dev = &vdev->v4l2_dev;
	v->video.release = vid_video_device_release;
	v->video.fops = &video_ops;
	v->video.ioctl_ops = &vid_ioctl_ops;
	v->video.minor = -1;
	v->video.vfl_type = VFL_TYPE_VIDEO;
	v->video.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	mutex_init(&v->lock);

	v->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	v->queue.drv_priv = v;
	v->queue.ops = &vid_vb2_ops;
	v->queue.io_modes = VB2_MMAP | VB2_DMABUF | VB2_USERPTR;
	v->queue.mem_ops = &vb2_dma_contig_memops;
	v->queue.buf_struct_size = sizeof(struct cam_buf);
	v->queue.min_buffers_needed = 2;
	v->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	v->queue.lock = &v->lock;
	v->queue.dev = dev;

	rc = vb2_queue_init(&v->queue);
	if (rc < 0) {
		dev_err(dev, "failed to init vb2 queue (err=%d).\n", rc);
		goto _vb2_queue_init_err;
	}
	v->video.queue = &v->queue;

	spin_lock_init(&v->irqlock);
	INIT_LIST_HEAD(&v->queued_list);
	v->bctx.qbuf = vid_qbuf;
	v->bctx.dqbuf = vid_dqbuf;

	video_set_drvdata(&v->video, &v->bctx);

	v->video.entity.name = v->video.name;
	v->video.entity.obj_type = MEDIA_ENTITY_TYPE_VIDEO_DEVICE;
	v->video.entity.function = MEDIA_ENT_F_IO_V4L;
	v->video.entity.ops = &vid_media_ops;

	v->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	v->fmt.fmt.pix.field = V4L2_FIELD_NONE;
	v->fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

	v->pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	rc = media_entity_pads_init(&v->video.entity, 1, &v->pad);
	if (rc < 0) {
		dev_err(dev, "failed to init media pads (err=%d).\n", rc);
		goto _media_pads_init_err;
	}

	rc = video_register_device(&v->video, v->video.vfl_type, -1);
	if (rc < 0) {
		dev_err(dev, "failed to register video device (err=%d).\n", rc);
		goto _video_dev_reg_err;
	}

	list_add_tail(&v->entry, &vdev->video_device_list);
	return v;

_video_dev_reg_err:
	media_entity_cleanup(&v->video.entity);

_media_pads_init_err:
	vb2_queue_release(&v->queue);

_vb2_queue_init_err:
	mutex_destroy(&v->lock);
	return ERR_PTR(rc);
}

static void destroy_video_device(struct vid_video_device *vdev)
{
	if (vdev) {
		vid_return_all_buffers(vdev, VB2_BUF_STATE_QUEUED);
		video_unregister_device(&vdev->video);
		media_entity_cleanup(&vdev->video.entity);
		vb2_queue_release(&vdev->queue);
		mutex_destroy(&vdev->lock);
	}
}

static struct media_entity *find_entity_by_name(struct v4l2_device *dev,
						const char *name)
{
	struct v4l2_subdev *sd;

	list_for_each_entry(sd, &dev->subdevs, list)
		if (!strcmp(sd->entity.name, name))
			return &sd->entity;
	return NULL;
}

static struct media_pad *get_pad(struct media_entity *ent, u16 pad,
				 bool is_sink)
{
	u16 i = 0, j = 0;
	u32 flags;

	if (unlikely(!ent->pads))
		return NULL;

	if (is_sink)
		flags = MEDIA_PAD_FL_SINK;
	else
		flags = MEDIA_PAD_FL_SOURCE;

	while (i < ent->num_pads) {
		if (ent->pads[i].flags & flags) {
			if (j == pad)
				return &ent->pads[i];
			j++;
		}
		i++;
	}
	return NULL;
}

static int create_link(struct device *dev, struct media_entity *src,
		       u16 src_pad_idx, struct media_entity *sink,
		       u16 sink_pad_idx, u32 flags)
{
	struct media_pad *src_pad, *sink_pad, *pad;
	int rc = 0;

	src_pad = get_pad(src, src_pad_idx, false);
	sink_pad = get_pad(sink, sink_pad_idx, true);
	if (!src_pad || !sink_pad)
		return -EINVAL;

	pad = media_pad_remote_pad_first(src_pad);
	if (pad) {
		dev_warn(dev,
			 "entity %s pad %d <-> entity %s pad %d existed!\n",
			 src_pad->entity->name, src_pad->index,
			 pad->entity->name, pad->index);
		return -EBUSY;
	}

	pad = media_pad_remote_pad_first(sink_pad);
	if (pad) {
		dev_warn(dev,
			 "entity %s pad %d <-> entity %s pad %d existed!\n",
			 pad->entity->name, pad->index, src_pad->entity->name,
			 src_pad->index);
		return -EBUSY;
	}

	rc = media_entity_call(src, link_setup, src_pad, sink_pad, flags);
	if (rc < 0 && rc != -ENOIOCTLCMD)
		return rc;

	rc = media_entity_call(sink, link_setup, sink_pad, src_pad, flags);
	if (rc < 0 && rc != -ENOIOCTLCMD) {
		flags &= ~MEDIA_LNK_FL_ENABLED;
		media_entity_call(src, link_setup, src_pad, sink_pad, flags);
		return rc;
	}

	rc = media_create_pad_link(src, src_pad->index, sink, sink_pad->index,
				   flags);
	if (rc < 0) {
		flags &= ~MEDIA_LNK_FL_ENABLED;
		media_entity_call(src, link_setup, src_pad, sink_pad, flags);
		media_entity_call(sink, link_setup, sink_pad, src_pad, flags);
	}
	return rc;
}

int create_default_links(struct vid_device *vdev)
{
	struct vid_video_device *v = NULL;
	struct media_entity *src, *sink;
	char name[64];
	u32 i = 0, j = 0;
	int rc = 0;

	for (;;) {
		snprintf(name, sizeof(name), "%s%d-%d", CSI_DEV_NAME, i, j);
		src = find_entity_by_name(&vdev->v4l2_dev, name);
		snprintf(name, sizeof(name), "%s%d-%d", SIF_DEV_NAME, i, j);
		sink = find_entity_by_name(&vdev->v4l2_dev, name);
		if (!src || !sink) {
			if (!j)
				break;

			j = 0;
			i++;
			continue;
		}

		rc = create_link(vdev->v4l2_dev.dev, src, 0, sink, 0,
				 MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
		if (rc < 0)
			return rc;

		j++;
	}

	j = 0;
	for (i = 0; i < links_size[scene]; i++) {
		struct entity_link *link = &links[scene][i];

		src = find_entity_by_name(&vdev->v4l2_dev, link->src_name);
		if (!src)
			continue;

		if (!strcmp(link->sink_name, "video")) {
			v = create_video_device(vdev, j++);
			if (IS_ERR(v))
				return (int)PTR_ERR(v);

			sink = &v->video.entity;
		} else {
			sink = find_entity_by_name(&vdev->v4l2_dev,
						   link->sink_name);
		}
		if (!sink)
			continue;

		rc = create_link(vdev->v4l2_dev.dev, src, link->src_pad, sink,
				 link->sink_pad, link->flags);
		if (rc < 0) {
			dev_err(vdev->v4l2_dev.dev, "failed to create link , src_pad=%d, sink_pad=%d", link->src_pad, link->sink_pad);
			destroy_links(vdev);
			return rc;
		}
	}
	return rc;
}

void destroy_links(struct vid_device *vdev)
{
	struct v4l2_subdev *sd;
	struct vid_video_device *v;

	if (unlikely(!vdev))
		return;

	list_for_each_entry(sd, &vdev->v4l2_dev.subdevs, list)
		media_entity_remove_links(&sd->entity);

	list_for_each_entry(v, &vdev->video_device_list, entry) {
		media_entity_remove_links(&v->video.entity);
		destroy_video_device(v);
	}
}
