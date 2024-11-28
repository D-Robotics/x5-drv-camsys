// SPDX-License-Identifier: GPL-2.0-only
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "cam_uapi.h"
#include "utils.h"
#include "video_com.h"

#include "video_m2m.h"

enum {
	V4L2_M2M_SRC = 0,
	V4L2_M2M_DST = 1,
};

struct vid_m2m_dev {
	struct v4l2_device v4l2_dev;
	struct video_device vfd;
	struct mutex vdev_lock; /* lock for video device */
	struct device *dev;
	struct mutex vb_lock; /* lock for vb2 queue */
	struct v4l2_format fmt[2];
	struct v4l2_buf_ctx bctx;
	struct media_pad pads[2];
	struct cam_ctx ctx;
	struct vid_m2m_video_device m2m_vdev;
	struct v4l2_m2m_ctx *m2m_ctx;
	struct v4l2_m2m_dev *m2m_dev;
	spinlock_t irqlock; /* lock for cam buf */
	u32 buf_sequence[2];
	u32 translen;
	int aborting;
};

struct vid_m2m_ctx {
	struct v4l2_fh fh;
	struct vid_m2m_dev *dev;
};

static inline struct vid_m2m_ctx *file2ctx(struct file *file)
{
	return container_of(file->private_data, struct vid_m2m_ctx, fh);
}

static inline struct vid_m2m_dev *file2dev(struct file *file)
{
	struct vid_m2m_ctx *ctx =
			container_of(file->private_data, struct vid_m2m_ctx, fh);

	if (ctx)
		return ctx->dev;
	return NULL;
}

#undef sink_pad
#define sink_pad(v) (&(v)->pads[V4L2_M2M_SRC])
#undef src_pad
#define src_pad(v)  (&(v)->pads[V4L2_M2M_DST])

static struct v4l2_format *get_fmt(struct vid_m2m_dev *dev,
				   enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return &dev->fmt[V4L2_M2M_SRC];
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &dev->fmt[V4L2_M2M_DST];
	default:
		return NULL;
	}
}

static const char *type_name(enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return "Output";
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return "Capture";
	default:
		return "Invalid";
	}
}

static int vid_src_dqbuf(struct vid_m2m_dev *dev)
{
	struct cam_buf *buf;

	buf = cam_dqbuf_irq(&dev->ctx, true);
	if (buf) {
		buf->m2m.vb.vb2_buf.timestamp = ktime_get_ns();
		buf->m2m.vb.sequence = dev->buf_sequence[V4L2_M2M_SRC]++;
		buf->m2m.vb.field = V4L2_FIELD_NONE;
		v4l2_m2m_buf_done(&buf->m2m.vb, VB2_BUF_STATE_DONE);
	}
	return 0;
}

static int vid_qbuf(struct v4l2_buf_ctx *ctx, u32 pad, struct cam_buf *buf)
{
	struct vid_m2m_dev *dev = container_of(ctx, struct vid_m2m_dev, bctx);
	struct v4l2_format *fmt;

	if (!buf || buf->m2m.vb.vb2_buf.state != VB2_BUF_STATE_ACTIVE)
		return -EINVAL;

	fmt = get_fmt(dev, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (!fmt)
		return -EINVAL;

	vb2_set_plane_payload(&buf->m2m.vb.vb2_buf, 0, fmt->fmt.pix.sizeimage);
	buf->m2m.vb.vb2_buf.timestamp = ktime_get_ns();
	buf->m2m.vb.sequence = dev->buf_sequence[V4L2_M2M_DST]++;
	buf->m2m.vb.field = V4L2_FIELD_NONE;
	v4l2_m2m_buf_done(&buf->m2m.vb, VB2_BUF_STATE_DONE);
	vid_src_dqbuf(dev);
	v4l2_m2m_job_finish(dev->m2m_dev, dev->m2m_ctx);
	return 0;
}

static int vid_drop(struct v4l2_buf_ctx *ctx, u32 pad, struct cam_buf *buf)
{
	struct vid_m2m_dev *dev = container_of(ctx, struct vid_m2m_dev, bctx);

	if (!ctx || !buf)
		return -EINVAL;

	v4l2_m2m_buf_queue(dev->m2m_ctx, &buf->m2m.vb);
	vid_src_dqbuf(dev);
	v4l2_m2m_job_finish(dev->m2m_dev, dev->m2m_ctx);
	return 0;
}

#define to_cam_buf(b) \
({ \
	struct v4l2_m2m_buffer *m2m = \
			container_of(b, struct v4l2_m2m_buffer, vb); \
	container_of(m2m, struct cam_buf, m2m); \
})

static struct cam_buf *vid_dqbuf(struct v4l2_buf_ctx *ctx, u32 pad)
{
	struct vid_m2m_dev *dev = container_of(ctx, struct vid_m2m_dev, bctx);
	struct vb2_v4l2_buffer *vb;
	struct cam_buf *buf = NULL;
	unsigned long flags;

	spin_lock_irqsave(&dev->irqlock, flags);
	vb = v4l2_m2m_dst_buf_remove(dev->m2m_ctx);
	if (vb)
		buf = to_cam_buf(vb);
	spin_unlock_irqrestore(&dev->irqlock, flags);
	return buf;
}

static struct cam_buf *vid_acqbuf(struct v4l2_buf_ctx *ctx, u32 pad)
{
	struct vid_m2m_dev *dev = container_of(ctx, struct vid_m2m_dev, bctx);
	struct vb2_v4l2_buffer *vb;
	unsigned long flags;

	spin_lock_irqsave(&dev->irqlock, flags);
	vb = v4l2_m2m_next_dst_buf(dev->m2m_ctx);
	spin_unlock_irqrestore(&dev->irqlock, flags);
	return vb ? to_cam_buf(vb) : NULL;
}

static int job_ready(void *priv)
{
	struct vid_m2m_dev *dev = priv;

	if (v4l2_m2m_num_src_bufs_ready(dev->m2m_ctx) < dev->translen
	    || v4l2_m2m_num_dst_bufs_ready(dev->m2m_ctx) < dev->translen) {
		dev_err(dev->dev, "no enough buffers available\n");
		return 0;
	}
	return 1;
}

static void job_abort(void *priv)
{
	struct vid_m2m_dev *dev = priv;

	dev->aborting = 1;
}

static inline void notify_buf_ready(struct vid_m2m_dev *dev, int on)
{
	struct v4l2_subdev *sd;
	struct media_pad *pad = get_remote_pad_sd(sink_pad(dev), &sd);

	if (sd)
		v4l2_subdev_ctx_call_no_return(sd, ready, pad->index, on);
}

static void device_run(void *priv)
{
	struct vid_m2m_dev *dev = priv;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	int rc;

	src_buf = v4l2_m2m_src_buf_remove(dev->m2m_ctx);
	rc = cam_qbuf_irq(&dev->ctx, (struct cam_buf *)src_buf, true);
	if (rc < 0) {
		dev_err(dev->dev, "failed to queue src buf (err=%d).\n", rc);
		return;
	}

	dst_buf = v4l2_m2m_next_dst_buf(dev->m2m_ctx);
	if (dst_buf)
		notify_buf_ready(dev, 1);
}

static int vid_enum_fmt_vid_cap(struct file *file, void *fh,
				struct v4l2_fmtdesc *f)
{
	struct vid_m2m_dev *dev = file2dev(file);
	struct v4l2_subdev *sd;
	struct media_pad *pad = get_remote_pad_sd(sink_pad(dev), &sd);

	if (!IS_CAPTURE_V4L2_TYPE(f->type))
		return -EINVAL;

	if (sd)
		return v4l2_subdev_ctx_call(sd, enum_format, pad->index, f->index,
					    &f->pixelformat);
	return -ENOLINK;
}

static int vid_enum_fmt_vid_out(struct file *file, void *fh,
				struct v4l2_fmtdesc *f)
{
	struct vid_m2m_dev *dev = file2dev(file);
	struct v4l2_subdev *sd;
	struct media_pad *pad = get_remote_pad_sd(sink_pad(dev), &sd);

	if (!IS_OUTPUT_V4L2_TYPE(f->type))
		return -EINVAL;

	if (sd)
		return v4l2_subdev_ctx_call(sd, enum_format, pad->index, f->index,
					    &f->pixelformat);
	return -ENOLINK;
}

static int vid_enum_framesizes(struct file *file, void *fh,
			       struct v4l2_frmsizeenum *fsize)
{
	struct vid_m2m_dev *dev = file2dev(file);
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	int rc;

	pad = sink_pad(dev);
	rc = vid_check_pixelformat(pad, fsize->pixel_format);
	if (rc)
		return -EINVAL;

	pad = get_remote_pad_sd(pad, &sd);
	if (sd)
		return v4l2_subdev_ctx_call(sd, enum_framesize, pad->index, fsize);
	return -ENOLINK;
}

static int vid_g_fmt_vid_out(struct file *file, void *fh,
			     struct v4l2_format *f)
{
	struct vid_m2m_dev *dev = file2dev(file);
	struct v4l2_format *fmt;
	int rc;

	fmt = get_fmt(dev, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	if (!fmt->fmt.pix.pixelformat) {
		rc = get_def_fmt(src_pad(dev), fmt);
		if (rc < 0)
			return rc;
	}

	*f = *fmt;
	return 0;
}

static int vid_g_fmt_vid_cap(struct file *file, void *fh,
			     struct v4l2_format *f)
{
	struct vid_m2m_dev *dev = file2dev(file);
	struct v4l2_format *fmt;
	int rc;

	fmt = get_fmt(dev, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (!fmt->fmt.pix.pixelformat) {
		rc = get_def_fmt(sink_pad(dev), fmt);
		if (rc < 0)
			return rc;
	}

	*f = *fmt;
	return 0;
}

static int try_fmt_vid(struct media_pad *pad, struct v4l2_format *f, bool is_try)
{
	struct v4l2_subdev *sd;
	struct media_pad *r_pad = get_remote_pad_sd(pad, &sd);
	int rc;

	rc = vid_check_pixelformat(pad, f->fmt.pix.pixelformat);
	if (rc < 0)
		return rc;
	if (rc) {
		rc = get_def_fmt(pad, f);
		if (rc < 0)
			return rc;
	}

	if (f->fmt.pix.width < MIN_W || f->fmt.pix.height < MIN_H ||
	    f->fmt.pix.width > MAX_W || f->fmt.pix.height > MAX_H) {
		rc = get_def_fmt(pad, f);
		if (rc < 0) {
			pr_err("%s get_def_fmt failed\n", __func__);
			return rc;
		}
	}

	return v4l2_subdev_ctx_call(sd, set_format, r_pad->index, f, is_try);
}

static int vid_try_fmt_vid_cap(struct file *file, void *fh,
			       struct v4l2_format *f)
{
	struct vid_m2m_dev *dev = file2dev(file);

	if (!IS_CAPTURE_V4L2_TYPE(f->type))
		return -EINVAL;

	return try_fmt_vid(sink_pad(dev), f, true);
}

static int vid_try_fmt_vid_out(struct file *file, void *fh,
			       struct v4l2_format *f)
{
	struct vid_m2m_dev *dev = file2dev(file);

	if (!IS_OUTPUT_V4L2_TYPE(f->type))
		return -EINVAL;

	return try_fmt_vid(src_pad(dev), f, true);
}

static int vid_s_fmt(struct vid_m2m_dev *dev, struct media_pad *pad,
		     struct v4l2_format *f)
{
	struct vb2_queue *vq;
	struct v4l2_format *fmt;

	vq = v4l2_m2m_get_vq(dev->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		dev_err(dev->dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	fmt = get_fmt(dev, f->type);
	if (!fmt)
		return -EINVAL;

	fmt->fmt.pix.pixelformat = f->fmt.pix.pixelformat;
	fmt->fmt.pix.width = f->fmt.pix.width;
	fmt->fmt.pix.height = f->fmt.pix.height;

	init_fmt(fmt);
	if (IS_OUTPUT_V4L2_TYPE(f->type)) {
		fmt->fmt.pix.colorspace = f->fmt.pix.colorspace;
		fmt->fmt.pix.xfer_func = f->fmt.pix.xfer_func;
		fmt->fmt.pix.ycbcr_enc = f->fmt.pix.ycbcr_enc;
		fmt->fmt.pix.quantization = f->fmt.pix.quantization;
	}

	return try_fmt_vid(pad, f, false);
}

static int vid_s_fmt_vid_cap(struct file *file, void *fh,
			     struct v4l2_format *f)
{
	struct vid_m2m_dev *dev = file2dev(file);

	return vid_s_fmt(dev, sink_pad(dev), f);
}

static int vid_s_fmt_vid_out(struct file *file, void *fh,
			     struct v4l2_format *f)
{
	struct vid_m2m_dev *dev = file2dev(file);
	struct v4l2_format *fmt;
	int rc;

	rc = vid_s_fmt(dev, src_pad(dev), f);
	if (!rc) {
		fmt = get_fmt(dev, V4L2_BUF_TYPE_VIDEO_CAPTURE);
		fmt->fmt.pix.colorspace = f->fmt.pix.colorspace;
		fmt->fmt.pix.xfer_func = f->fmt.pix.xfer_func;
		fmt->fmt.pix.ycbcr_enc = f->fmt.pix.ycbcr_enc;
		fmt->fmt.pix.quantization = f->fmt.pix.quantization;
	}
	return rc;
}

static int vid_s_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct vid_m2m_dev *dev = file2dev(file);
	struct media_pad *pad;
	struct v4l2_subdev *sd;
	struct sen_ctrl ctrl = {0};

	ctrl.ctrl_id = a->id;
	memcpy(&ctrl.ctrl_data, &a->value, sizeof(a->value));
	pad = get_remote_pad_sd(sink_pad(dev), &sd);
	if (sd)
		return v4l2_subdev_call(sd, core, command, CAM_SET_CTRL, &ctrl);
	return -ENOLINK;
}

static int vid_g_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct vid_m2m_dev *dev = file2dev(file);
	struct media_pad *pad;
	struct v4l2_subdev *sd;
	struct sen_ctrl ctrl = {0};
	int rc = 0;

	ctrl.ctrl_id = a->id;
	pad = get_remote_pad_sd(sink_pad(dev), &sd);

	rc = v4l2_subdev_call(sd, core, command, CAM_GET_CTRL, &ctrl);
	if (rc < 0)
		return -EINVAL;

	memcpy(&a->value, &ctrl.ctrl_data, sizeof(a->value));
	return rc;
}

static int vid_s_ext_ctrls(struct file *file, void *fh, struct v4l2_ext_controls *a)
{
	struct vid_m2m_dev *dev = file2dev(file);
	struct media_pad *pad;
	struct v4l2_subdev *sd;
	struct cam_v4l2_ext_control cam_ext_ctrl;
	bool sd_is_vse = false;
	int rc = 0;

	pad = get_remote_pad_sd(sink_pad(dev), &sd);

	sd_is_vse = (strncmp(sd->name, VSE_DEV_NAME, strlen(VSE_DEV_NAME)) == 0);
	for (int i = 0; i < a->count; i++) {
		if (sd_is_vse) {
			cam_ext_ctrl.pad = pad->index;
			cam_ext_ctrl.controls = &a->controls[i];
			rc = v4l2_subdev_call(sd, core, command, CAM_SET_EXT_CTRL, &cam_ext_ctrl);
		} else
			rc = v4l2_subdev_call(sd, core, command, CAM_SET_EXT_CTRL, &a->controls[i]);
		if (rc < 0)
			return rc;
	}

	return rc;
}

static int vid_g_ext_ctrls(struct file *file, void *fh, struct v4l2_ext_controls *a)
{
	struct vid_m2m_dev *dev = file2dev(file);
	struct media_pad *pad;
	struct v4l2_subdev *sd;
	struct cam_v4l2_ext_control cam_ext_ctrl;
	bool sd_is_vse = false;
	int rc = 0;

	pad = get_remote_pad_sd(sink_pad(dev), &sd);
	sd_is_vse = (strncmp(sd->name, VSE_DEV_NAME, strlen(VSE_DEV_NAME)) == 0);
	for (int i = 0; i < a->count; i++) {
		if (sd_is_vse) {
			cam_ext_ctrl.pad = pad->index;
			cam_ext_ctrl.controls = &a->controls[i];
			rc = v4l2_subdev_call(sd, core, command, CAM_GET_EXT_CTRL, &cam_ext_ctrl);
		} else {
			rc = v4l2_subdev_call(sd, core, command, CAM_GET_EXT_CTRL, &a->controls[i]);
		}

		if (rc < 0)
			return rc;
	}

	return rc;
}

static long vid_ioctl(struct file *file, void *fh, bool valid_prio,
		      unsigned int cmd, void *arg)
{
	struct vid_m2m_dev *dev = file2dev(file);
	int rc = 0;

	switch (cmd) {
	case VIDIOC_GET_BUF_PHYS: {
		struct vb2_queue *q =
				v4l2_m2m_get_vq(dev->m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);
		struct vb2_buffer *vb;
		struct cam_buf *buf;
		unsigned int *phys = arg;
		unsigned long flags;

		spin_lock_irqsave(&dev->irqlock, flags);
		vb = list_last_entry(&q->queued_list, struct vb2_buffer, queued_entry);
		buf = vb2_buf_to_cam_buf(vb);
		if (buf)
			*phys = (unsigned int)get_phys_addr(NULL, buf, 0);
		spin_unlock_irqrestore(&dev->irqlock, flags);
	}
		break;
	default:
		rc = -ENOTTY;
		break;
	}
	return rc;
}

static const struct v4l2_ioctl_ops vid_m2m_ioctl_ops = {
	.vidioc_querycap = vid_querycap,
	.vidioc_enum_fmt_vid_cap = vid_enum_fmt_vid_cap,
	.vidioc_enum_framesizes = vid_enum_framesizes,
	.vidioc_g_fmt_vid_cap = vid_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = vid_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = vid_s_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out = vid_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out = vid_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out = vid_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out = vid_s_fmt_vid_out,
	.vidioc_default = vid_ioctl,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_s_ctrl = vid_s_ctrl,
	.vidioc_g_ctrl = vid_g_ctrl,
	.vidioc_s_ext_ctrls = vid_s_ext_ctrls,
	.vidioc_g_ext_ctrls = vid_g_ext_ctrls,
};

static int vid_m2m_queue_setup(struct vb2_queue *vq, unsigned int *num_buffers,
			      unsigned int *num_planes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct vid_m2m_dev *dev = vb2_get_drv_priv(vq);
	struct media_pad *pad;
	struct v4l2_format *fmt;
	unsigned int size;
	int rc;

	if (IS_OUTPUT_V4L2_TYPE(vq->type)) {
		pad = src_pad(dev);
		fmt = &dev->fmt[V4L2_M2M_SRC];
	} else if (IS_CAPTURE_V4L2_TYPE(vq->type)) {
		pad = sink_pad(dev);
		fmt = &dev->fmt[V4L2_M2M_DST];
	} else {
		return -EINVAL;
	}

	if (!fmt->fmt.pix.sizeimage) {
		rc = get_def_fmt(pad, fmt);
		if (rc < 0)
			return rc;
	}
	size = fmt->fmt.pix.sizeimage;

	if (sizes[0] && sizes[0] < size)
		return -EINVAL;

	if (*num_planes > 1)
		return -EINVAL;

	if (!*num_buffers)
		*num_buffers = 1;
	if (!*num_planes)
		*num_planes = 1;
	if (!sizes[0])
		sizes[0] = size;

	dev_dbg(dev->dev, "%s: get %d buffer(s) of size %d each.\n",
		type_name(vq->type), *num_buffers, sizes[0]);
	return 0;
}

static int vid_m2m_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vid_m2m_dev *dev = vb2_get_drv_priv(vb->vb2_queue);

	if (vbuf->field == V4L2_FIELD_ANY)
		vbuf->field = V4L2_FIELD_NONE;
	if (vbuf->field != V4L2_FIELD_NONE) {
		dev_err(dev->dev, "%s field isn't supported\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int vid_m2m_buf_prepare(struct vb2_buffer *vb)
{
	struct vid_m2m_dev *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_format *fmt;

	dev_dbg(dev->dev, "type: %s\n", type_name(vb->vb2_queue->type));

	fmt = get_fmt(dev, vb->vb2_queue->type);
	if (!fmt)
		return -EINVAL;

	if (vb2_plane_size(vb, 0) < fmt->fmt.pix.sizeimage) {
		dev_err(dev->dev,
			"%s data will not fit into plane (%u < %u)\n",
			__func__, (unsigned int)vb2_plane_size(vb, 0),
			fmt->fmt.pix.sizeimage);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, fmt->fmt.pix.sizeimage);
	return 0;
}

static void vid_m2m_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vid_m2m_dev *dev = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(dev->m2m_ctx, vbuf);
}

static int vid_m2m_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vid_m2m_dev *dev = vb2_get_drv_priv(q);
	struct v4l2_format *fmt = get_fmt(dev, q->type);
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	int rc = 0;

	if (!fmt)
		return -EINVAL;

	if (IS_OUTPUT_V4L2_TYPE(q->type))
		dev->aborting = 0;

	dev->buf_sequence[V4L2_M2M_SRC] = 0;
	dev->buf_sequence[V4L2_M2M_DST] = 0;

	if (IS_CAPTURE_V4L2_TYPE(q->type)) {
		pad = get_remote_pad_sd(sink_pad(dev), &sd);
		if (!sd)
			return -ENOLINK;

		v4l2_subdev_ctx_call_no_return(sd, set_stream, pad->index, 1);
		rc = v4l2_subdev_call(sd, video, s_stream, 1);
	}
	return rc;
}

static void vid_m2m_stop_streaming(struct vb2_queue *q)
{
	struct vid_m2m_dev *dev = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vbuf;
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	int rc;

	if (IS_CAPTURE_V4L2_TYPE(q->type)) {
		pad = get_remote_pad_sd(sink_pad(dev), &sd);
		if (!sd)
			return;

		rc = v4l2_subdev_call(sd, video, s_stream, 0);
		if (rc < 0)
			return;

		v4l2_subdev_ctx_call_no_return(sd, set_stream, pad->index, 0);
		notify_buf_ready(dev, 0);
	}

	for (;;) {
		if (IS_OUTPUT_V4L2_TYPE(q->type))
			vbuf = v4l2_m2m_src_buf_remove(dev->m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(dev->m2m_ctx);
		if (!vbuf)
			return;
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	}
}

static void vid_m2m_buf_request_complete(struct vb2_buffer *vb)
{
}

static const struct vb2_ops vid_m2m_qops = {
	.queue_setup = vid_m2m_queue_setup,
	.buf_out_validate = vid_m2m_buf_out_validate,
	.buf_prepare = vid_m2m_buf_prepare,
	.buf_queue = vid_m2m_buf_queue,
	.start_streaming = vid_m2m_start_streaming,
	.stop_streaming = vid_m2m_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.buf_request_complete = vid_m2m_buf_request_complete,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct vid_m2m_dev *dev = priv;
	int rc;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF | VB2_USERPTR;
	src_vq->drv_priv = dev;
	src_vq->buf_struct_size = sizeof(struct cam_buf);
	src_vq->ops = &vid_m2m_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &dev->vb_lock;
	src_vq->supports_requests = true;
	src_vq->dev = dev->dev;

	rc = vb2_queue_init(src_vq);
	if (rc)
		return rc;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF | VB2_USERPTR;
	dst_vq->drv_priv = dev;
	dst_vq->buf_struct_size = sizeof(struct cam_buf);
	dst_vq->ops = &vid_m2m_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &dev->vb_lock;
	dst_vq->dev = dev->dev;

	return vb2_queue_init(dst_vq);
}

static int vid_m2m_open(struct file *file)
{
	struct v4l2_buf_ctx *bctx = video_drvdata(file);
	struct vid_m2m_dev *dev = container_of(bctx, struct vid_m2m_dev, bctx);
	struct vid_m2m_ctx *ctx = NULL;
	struct v4l2_subdev *sd;
	int rc = 0;

	ctx = devm_kzalloc(dev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	ctx->dev = dev;
	ctx->fh.m2m_ctx = dev->m2m_ctx;

	v4l2_fh_add(&ctx->fh);
	dev_dbg(dev->dev, "created instance: %p, m2m_ctx: %p\n",
		ctx, ctx->fh.m2m_ctx);

	(void)get_remote_pad_sd(sink_pad(dev), &sd);
	if (!sd) {
		v4l2_fh_release(file);
		return -ENOLINK;
	}
	if (sd->internal_ops && sd->internal_ops->open)
		sd->internal_ops->open(sd, NULL/*subdev_fh*/);
	return rc;
}

static int vid_m2m_release(struct file *file)
{
	struct v4l2_buf_ctx *bctx = video_drvdata(file);
	struct vid_m2m_dev *dev = container_of(bctx, struct vid_m2m_dev, bctx);
	struct vid_m2m_ctx *ctx = file2ctx(file);
	struct vb2_queue *out_vq, *cap_vq;
	struct v4l2_subdev *sd;

	dev_dbg(ctx->dev->dev, "releasing instance %p\n", ctx);
	(void)get_remote_pad_sd(sink_pad(dev), &sd);
	if (!sd) {
		v4l2_fh_release(file);
		return -ENOLINK;
	}

	if (sd->internal_ops && sd->internal_ops->close)
		sd->internal_ops->close(sd, NULL/*subdev_fh*/);

	/* video buffer is released here if it was not previously released */
	cap_vq = v4l2_m2m_get_vq(dev->m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	out_vq = v4l2_m2m_get_vq(dev->m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	if (cap_vq && (file->private_data == cap_vq->owner)) {
		vb2_queue_release(cap_vq);
		cap_vq->owner = NULL;
	}
	if (out_vq && (file->private_data == out_vq->owner)) {
		vb2_queue_release(out_vq);
		out_vq->owner = NULL;
	}

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	devm_kfree(ctx->dev->dev, ctx);

	return 0;
}

static const struct v4l2_file_operations vid_m2m_fops = {
	.owner = THIS_MODULE,
	.open = vid_m2m_open,
	.release = vid_m2m_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static const struct v4l2_m2m_ops m2m_ops = {
	.device_run = device_run,
	.job_ready = job_ready,
	.job_abort = job_abort,
};

static const struct media_device_ops m2m_media_ops = {
	.req_validate = vb2_request_validate,
	.req_queue = v4l2_m2m_request_queue,
};

struct vid_m2m_video_device *create_m2m_video_device(struct vid_device *vdev,
						     u32 id)
{
	struct vid_m2m_dev *dev;
	struct media_entity *entity;
	int rc;

	if (!vdev)
		return ERR_PTR(-EINVAL);

	dev = devm_kzalloc(vdev->v4l2_dev.dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return ERR_PTR(-ENOMEM);

	dev->dev = vdev->v4l2_dev.dev;
	dev->v4l2_dev.mdev = &vdev->mdev;

	spin_lock_init(&dev->irqlock);
	dev->bctx.qbuf = vid_qbuf;
	dev->bctx.dqbuf = vid_dqbuf;
	dev->bctx.acqbuf = vid_acqbuf;
	dev->bctx.drop = vid_drop;
	video_set_drvdata(&dev->vfd, &dev->bctx);
	video_set_drvdata(&dev->m2m_vdev.video, &dev->bctx);

	rc = v4l2_device_register(dev->dev, &dev->v4l2_dev);
	if (rc)
		goto _v4l_dev_reg_err;

	mutex_init(&dev->vdev_lock);

	snprintf(dev->vfd.name, sizeof(dev->vfd), "%s%d", VID_DEV_NAME, id);
	dev->vfd.vfl_dir = VFL_DIR_M2M,
	dev->vfd.fops = &vid_m2m_fops,
	dev->vfd.ioctl_ops = &vid_m2m_ioctl_ops,
	dev->vfd.minor = -1,
	dev->vfd.release = video_device_release_empty,
	dev->vfd.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING,

	dev->vfd.lock = &dev->vdev_lock;
	dev->vfd.v4l2_dev = &dev->v4l2_dev;

	entity = &dev->m2m_vdev.video.entity;
	entity->name = dev->vfd.name;
	entity->obj_type = MEDIA_ENTITY_TYPE_VIDEO_DEVICE;
	entity->function = MEDIA_ENT_F_IO_V4L;
	entity->ops = &vid_media_ops;

	sink_pad(dev)->flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	src_pad(dev)->flags = MEDIA_PAD_FL_SOURCE | MEDIA_PAD_FL_MUST_CONNECT;
	rc = media_entity_pads_init(entity, 2, dev->pads);
	if (rc < 0) {
		dev_err(dev->dev, "%s failed to init media pads (err=%d).\n",
			__func__, rc);
		goto _pads_init_err;
	}

	rc = cam_ctx_init(&dev->ctx, src_pad(dev), NULL);
	if (rc < 0) {
		dev_err(dev->dev, "%s failed to init buf ctx (err=%d).\n",
			__func__, rc);
		goto _pads_init_err;
	}

	rc = video_register_device(&dev->vfd, VFL_TYPE_VIDEO, -1);
	if (rc) {
		dev_err(dev->dev, "failed to register video device\n");
		goto _pads_init_err;
	}

	dev_dbg(dev->dev, "device registered as /dev/video%d\n", dev->vfd.num);

	dev->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(dev->m2m_dev)) {
		dev_err(dev->dev, "failed to init mem2mem device\n");
		rc = PTR_ERR(dev->m2m_dev);
		goto _m2m_init_err;
	}

	rc = v4l2_m2m_register_media_controller(dev->m2m_dev, &dev->vfd,
						 MEDIA_ENT_F_PROC_VIDEO_SCALER);
	if (rc) {
		dev_err(dev->dev, "failed to init mem2mem media controller\n");
		goto _m2m_reg_mc_err;
	}

	rc = media_device_register_entity(&vdev->mdev, entity);
	if (rc < 0) {
		dev_err(dev->dev, "failed to register media entity\n");
		goto _entity_reg_err;
	}

	mutex_init(&dev->vb_lock);

	dev->fmt[V4L2_M2M_SRC].type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	dev->fmt[V4L2_M2M_SRC].fmt.pix.field = V4L2_FIELD_NONE;
	dev->fmt[V4L2_M2M_SRC].fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
	dev->fmt[V4L2_M2M_DST].type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dev->fmt[V4L2_M2M_DST].fmt.pix.field = V4L2_FIELD_NONE;
	dev->fmt[V4L2_M2M_DST].fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

	dev->m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, dev, &queue_init);
	if (IS_ERR(dev->m2m_ctx)) {
		rc = PTR_ERR(dev->m2m_ctx);
		goto _m2m_ctx_init_err;
	}

	list_add_tail(&dev->m2m_vdev.entry, &vdev->m2m_video_device_list);
	return &dev->m2m_vdev;

_m2m_ctx_init_err:
	media_device_unregister_entity(&dev->m2m_vdev.video.entity);
_entity_reg_err:
	v4l2_m2m_unregister_media_controller(dev->m2m_dev);
_m2m_reg_mc_err:
	v4l2_m2m_release(dev->m2m_dev);
_m2m_init_err:
	video_unregister_device(&dev->vfd);
_pads_init_err:
	v4l2_device_unregister(&dev->v4l2_dev);
_v4l_dev_reg_err:
	devm_kfree(dev->dev, dev);
	return ERR_PTR(rc);
}
EXPORT_SYMBOL(create_m2m_video_device);

void destroy_m2m_video_device(struct vid_m2m_video_device *dev)
{
	struct vid_m2m_dev *m2m_dev =
			container_of(dev, struct vid_m2m_dev, m2m_vdev);

	if (!dev)
		return;

	v4l2_m2m_ctx_release(m2m_dev->m2m_ctx);
	media_device_unregister_entity(&dev->video.entity);
	v4l2_m2m_unregister_media_controller(m2m_dev->m2m_dev);
	v4l2_m2m_release(m2m_dev->m2m_dev);
	video_unregister_device(&m2m_dev->vfd);
	v4l2_device_unregister(&m2m_dev->v4l2_dev);
	mutex_destroy(&m2m_dev->vb_lock);
	mutex_destroy(&m2m_dev->vdev_lock);
	devm_kfree(m2m_dev->dev, m2m_dev);
}
EXPORT_SYMBOL(destroy_m2m_video_device);
