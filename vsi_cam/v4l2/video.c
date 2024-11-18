// SPDX-License-Identifier: GPL-2.0-only
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <linux/i2c.h>

#include "cam_buf.h"
#include "cam_uapi.h"
#include "utils.h"
#include "video.h"
#include "video_com.h"
#include "video_link.h"
#include "video_m2m.h"
#include "v4l2_usr_api.h"

static int scene;
module_param(scene, int, 0644);

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
	bool opened, closed;
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

static int vid_drop(struct v4l2_buf_ctx *ctx, struct cam_buf *buf)
{
	struct vid_video_device *vdev =
		container_of(ctx, struct vid_video_device, bctx);
	unsigned long flags;

	if (!ctx || !buf)
		return -EINVAL;

	spin_lock_irqsave(&vdev->irqlock, flags);
	list_add_tail(&buf->entry, &vdev->queued_list);
	spin_unlock_irqrestore(&vdev->irqlock, flags);
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

static struct cam_buf *vid_acqbuf(struct v4l2_buf_ctx *ctx)
{
	struct vid_video_device *vdev =
		container_of(ctx, struct vid_video_device, bctx);
	unsigned long flags;
	struct cam_buf *buf;

	spin_lock_irqsave(&vdev->irqlock, flags);
	buf = list_first_entry_or_null(&vdev->queued_list, struct cam_buf,
				       entry);
	spin_unlock_irqrestore(&vdev->irqlock, flags);
	return buf;
}

static int vid_queue_setup(struct vb2_queue *vq, unsigned int *num_buffers,
			   unsigned int *num_planes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct vid_video_device *vdev = (struct vid_video_device *)vq->drv_priv;
	unsigned int size;
	int rc;

	if (!vdev->fmt.fmt.pix.sizeimage) {
		rc = get_def_fmt(&vdev->pad, &vdev->fmt);
		if (rc < 0)
			return rc;
	}

	size = vdev->fmt.fmt.pix.sizeimage;

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
	return 0;
}

static inline void notify_buf_ready(struct vid_video_device *dev, int on)
{
	struct v4l2_subdev *sd;
	struct media_pad *pad = get_remote_pad_sd(&dev->pad, &sd);

	if (sd)
		v4l2_subdev_ctx_call_no_return(sd, ready, pad->index, on);
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

static int vid_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vid_video_device *vdev = (struct vid_video_device *)vq->drv_priv;
	struct v4l2_subdev *sd;
	struct media_pad *pad = get_remote_pad_sd(&vdev->pad, &sd);
	int rc;

	vdev->buf_sequence = 0;
	if (!sd)
		return -ENOLINK;

	v4l2_subdev_ctx_call_no_return(sd, set_stream, pad->index, 1);

	rc = v4l2_subdev_call(sd, video, s_stream, 1);
	if (rc < 0) {
		vid_return_all_buffers(vdev, VB2_BUF_STATE_QUEUED);
		return rc;
	}

//	rc = media_pipeline_start(&vdev->pad, &vdev->pipe);
//	if (rc < 0)
//		vid_return_all_buffers(vdev, VB2_BUF_STATE_QUEUED);
	return rc;
}

static void vid_stop_streaming(struct vb2_queue *vq)
{
	struct vid_video_device *vdev = (struct vid_video_device *)vq->drv_priv;
	struct v4l2_subdev *sd;
	struct media_pad *pad = get_remote_pad_sd(&vdev->pad, &sd);
	int rc;

	if (!sd)
		return;

	rc = v4l2_subdev_call(sd, video, s_stream, 0);
	if (rc < 0)
		return;

	v4l2_subdev_ctx_call_no_return(sd, set_stream, pad->index, 0);

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

static int vid_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct v4l2_subdev *sd;
	struct media_pad *pad = get_remote_pad_sd(&vdev->pad, &sd);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (sd)
		return v4l2_subdev_ctx_call(sd, enum_format, pad->index, f->index, &f->pixelformat);
	return -ENOLINK;
}

static int vid_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	int rc;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (!vdev->fmt.fmt.pix.pixelformat) {
		rc = get_def_fmt(&vdev->pad, &vdev->fmt);
		if (rc < 0)
			return rc;
	}

	*f = vdev->fmt;
	return 0;
}

static int try_s_fmt(struct vid_video_device *vdev, struct v4l2_format *f,
		     bool is_try)
{
	struct v4l2_subdev *sd;
	struct media_pad *pad = get_remote_pad_sd(&vdev->pad, &sd);
	struct video_fmt *v_f;
	int rc;

	if (!sd)
		return -ENOLINK;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	v_f = get_fmt_by_pixelformat(f->fmt.pix.pixelformat);
	if (!v_f) {
		pr_err("%s get_fmt_by_pixelformat failed\n", __func__);
		return -EINVAL;
	}

	if (f->fmt.pix.width < MIN_W || f->fmt.pix.height < MIN_H ||
	    f->fmt.pix.width > MAX_W || f->fmt.pix.height > MAX_H) {
		rc = get_def_fmt(&vdev->pad, f);
		if (rc < 0) {
			pr_err("%s get_def_fmt failed\n", __func__);
			return rc;
		}
	}

	rc = v4l2_subdev_ctx_call(sd, set_format, pad->index, f, is_try);
	if (rc < 0) {
		pr_err("%s v4l2_subdev_ctx_call failed\n", __func__);
		return rc;
	}

	init_fmt(f);
	return 0;
}

static int vid_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	int rc;

	rc = vid_check_pixelformat(&vdev->pad, f->fmt.pix.pixelformat);
	if (rc < 0)
		return rc;
	if (rc) {
		rc = get_def_fmt(&vdev->pad, f);
		if (rc < 0)
			return rc;
	}

	return try_s_fmt(vdev, f, true);
}

static int vid_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	int rc;

	if (vb2_is_busy(&vdev->queue)) {
		pr_warn("%s num_buffers of vdev queue is not 0 (%d)\n",
			__func__, vdev->queue.num_buffers);
		return -EBUSY;
	}

	rc = vid_check_pixelformat(&vdev->pad, f->fmt.pix.pixelformat);
	if (rc < 0)
		return rc;
	if (rc) {
		rc = get_def_fmt(&vdev->pad, f);
		if (rc < 0)
			return rc;
	}

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
	struct v4l2_subdev *sd;
	struct media_pad *pad = get_remote_pad_sd(&vdev->pad, &sd);
	int rc;

	if (fsize->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	rc = vid_check_pixelformat(&vdev->pad, fsize->pixel_format);
	if (rc)
		return -EINVAL;

	if (sd)
		return v4l2_subdev_ctx_call(sd, enum_framesize, pad->index, fsize);
	return -ENOLINK;
}

static int vid_enum_frameintervals(struct file *file, void *fh,
				   struct v4l2_frmivalenum *fival)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct v4l2_subdev *sd;
	struct media_pad *pad = get_remote_pad_sd(&vdev->pad, &sd);
	int rc;

	if (fival->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	rc = vid_check_pixelformat(&vdev->pad, fival->pixel_format);
	if (rc)
		return -EINVAL;

	if (sd)
		return v4l2_subdev_ctx_call(sd, enum_frameinterval, pad->index, fival);
	return -ENOLINK;
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
			*phys = (unsigned int)get_phys_addr(NULL, buf, 0);
		spin_unlock_irqrestore(&vdev->irqlock, flags);
	}
		break;
	default:
		rc = -ENOTTY;
	}
	return rc;
}

static int vid_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct v4l2_subdev *sd;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	(void)get_remote_pad_sd(&vdev->pad, &sd);
	if (sd)
		return v4l2_g_parm_cap(&vdev->video, sd, a);
	return -ENOLINK;
}

static int vid_s_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct v4l2_subdev *sd;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	(void)get_remote_pad_sd(&vdev->pad, &sd);
	if (sd)
		return v4l2_s_parm_cap(&vdev->video, sd, a);
	return -ENOLINK;
}

static int vid_enum_input(struct file *file, void *priv,
				struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;
	snprintf(input->name, sizeof(input->name), "vscam");
	return 0;
}

static int vid_g_input(struct file *file, void *priv, unsigned int *index)
{
	*index = 0;
	return 0;
}

static int vid_s_input(struct file *file, void *priv, unsigned int index)
{
	return index > 0 ? -EINVAL : 0;
}

static int vid_s_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct media_pad *pad;
	struct v4l2_subdev *sd;
	struct sen_ctrl ctrl = {0};

	ctrl.ctrl_id = a->id;
	memcpy(&ctrl.ctrl_data, &a->value, sizeof(a->value));
	pad = get_remote_pad_sd(&vdev->pad, &sd);
	if (sd)
		return v4l2_subdev_call(sd, core, command, CAM_SET_CTRL, &ctrl);
	return -ENOLINK;
}

static int vid_g_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct media_pad *pad;
	struct v4l2_subdev *sd;
	struct sen_ctrl ctrl = {0};
	int rc = 0;

	ctrl.ctrl_id = a->id;
	pad = get_remote_pad_sd(&vdev->pad, &sd);
	if (!pad)
		return -ENOLINK;

	rc = v4l2_subdev_call(sd, core, command, CAM_GET_CTRL, &ctrl);
	if (rc < 0)
		return -EINVAL;

	memcpy(&a->value, &ctrl.ctrl_data, sizeof(a->value));
	return rc;
}

static int vid_s_ext_ctrls(struct file *file, void *fh, struct v4l2_ext_controls *a)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct media_pad *pad;
	struct v4l2_subdev *sd;
	struct cam_v4l2_ext_control cam_ext_ctrl;
	bool sd_is_vse = false;
	int rc = 0;

	pad = get_remote_pad_sd(&vdev->pad, &sd);
	if (!pad)
		return -ENOLINK;

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
	struct vid_video_device *vdev = file_to_video_device(file);
	struct media_pad *pad;
	struct v4l2_subdev *sd;
	struct cam_v4l2_ext_control cam_ext_ctrl;
	bool sd_is_vse = false;
	int rc = 0;

	pad = get_remote_pad_sd(&vdev->pad, &sd);
	if (!pad)
		return -ENOLINK;

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

static int vid_queryctrl(struct file *file, void *fh, struct v4l2_queryctrl *a)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct v4l2_subdev *sd;

	(void)get_remote_pad_sd(&vdev->pad, &sd);
	if (sd)
		return v4l2_subdev_call(sd, core, command, CAM_QUERY_CTRL, a);
	return -ENOLINK;
}

static int vid_query_ext_ctrl(struct file *file, void *fh, struct v4l2_query_ext_ctrl *a)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct v4l2_subdev *sd;

	(void)get_remote_pad_sd(&vdev->pad, &sd);
	if (sd)
		return v4l2_subdev_call(sd, core, command, CAM_QUERY_EXT_CTRL, a);
	return -ENOLINK;
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
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_default = vid_ioctl,
	.vidioc_g_parm = vid_g_parm,
	.vidioc_s_parm = vid_s_parm,
	.vidioc_enum_input = vid_enum_input,
	.vidioc_s_input = vid_s_input,
	.vidioc_g_input = vid_g_input,
	.vidioc_s_ctrl = vid_s_ctrl,
	.vidioc_g_ctrl = vid_g_ctrl,
	.vidioc_s_ext_ctrls = vid_s_ext_ctrls,
	.vidioc_g_ext_ctrls = vid_g_ext_ctrls,
	.vidioc_queryctrl = vid_queryctrl,
	.vidioc_query_ext_ctrl = vid_query_ext_ctrl,
};

static int vid_open(struct file *file)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct v4l2_subdev *sd;
	int rc;

	if (vdev->video.vfl_type != VFL_TYPE_VIDEO)
		return 0;

	rc = v4l2_fh_open(file);
	if (rc < 0)
		return rc;

	if (!vdev->opened) {
		vdev->opened = true;
		return 0;
	}

	(void)get_remote_pad_sd(&vdev->pad, &sd);
	if (!sd) {
		v4l2_fh_release(file);
		return -ENOLINK;
	}

	if (sd->internal_ops && sd->internal_ops->open)
		sd->internal_ops->open(sd, NULL/*subdev_fh*/);
	return 0;
}

static int vid_release(struct file *file)
{
	struct vid_video_device *vdev = file_to_video_device(file);
	struct v4l2_subdev *sd;
	int rc;

	if (vdev->video.vfl_type != VFL_TYPE_VIDEO)
		return 0;

	if (!vdev->closed) {
		vdev->closed = true;
		return 0;
	}

	rc = vb2_fop_release(file);
	if (rc < 0) {
		pr_err("%s vb2_fop_release fail (err=%d)\n", vdev->video.name, rc);
		return rc;
	}

	(void)get_remote_pad_sd(&vdev->pad, &sd);
	if (!sd)
		return -ENOLINK;

	if (sd->internal_ops && sd->internal_ops->close)
		sd->internal_ops->close(sd, NULL/*subdev_fh*/);

	INIT_LIST_HEAD(&vdev->queued_list);
	memset(&vdev->fmt, 0, sizeof(vdev->fmt));
	vdev->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vdev->fmt.fmt.pix.field = V4L2_FIELD_NONE;
	vdev->fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
	return 0;
}
static const struct v4l2_file_operations video_ops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.open = vid_open,
	.release = vid_release,
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

	mutex_init(&v->lock);

	v->video.v4l2_dev = &vdev->v4l2_dev;
	v->video.release = video_device_release_empty;
	v->video.fops = &video_ops;
	v->video.ioctl_ops = &vid_ioctl_ops;
	v->video.minor = -1;
	v->video.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	v->video.lock = &v->lock;
	v->video.dev_parent = dev->parent;

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
	v->bctx.acqbuf = vid_acqbuf;
	v->bctx.drop = vid_drop;

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

	rc = video_register_device(&v->video, VFL_TYPE_VIDEO, -1);
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
		devm_kfree(vdev->queue.dev, vdev);
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

struct media_entity *get_sensor_media_entity(struct media_entity *csi_entity)
{
	struct v4l2_subdev *csi_subdev, *sensor_subdev;
	struct device *dev;
	struct fwnode_handle *csi_fwnode, *endpoint = NULL, *remote_endpoint = NULL;
	struct fwnode_handle *remote_port = NULL, *remote_node = NULL;
	struct media_entity *sensor_entity = NULL;
	struct i2c_client *i2c_dev;
	struct device_node *node;

	csi_subdev = media_entity_to_v4l2_subdev(csi_entity);
	if (!csi_subdev) {
		pr_err("Failed to get CSI subdev\n");
		return NULL;
	}

	dev = csi_subdev->dev;
	if (!dev) {
		pr_err("No device found for CSI subdev\n");
		return NULL;
	}

	csi_fwnode = dev_fwnode(dev);
	if (!csi_fwnode) {
		dev_err(dev, "No fwnode found for CSI device\n");
		return NULL;
	}

	endpoint = fwnode_graph_get_next_endpoint(csi_fwnode, NULL);
	if (!endpoint) {
		dev_err(dev, "No endpoint found for CSI\n");
		return NULL;
	}

	remote_endpoint = fwnode_graph_get_remote_endpoint(endpoint);
	if (!remote_endpoint) {
		dev_err(dev, "No remote endpoint found\n");
		goto out_put_endpoint;
	}

	remote_port = fwnode_get_parent(remote_endpoint);
	if (!remote_port) {
		dev_err(dev, "No remote port found\n");
		goto out_put_remote_endpoint;
	}

	remote_node = fwnode_get_parent(remote_port);
	if (!remote_node) {
		dev_err(dev, "No remote node found\n");
		goto out_put_remote_port;
	}

	node = to_of_node(remote_node);
	if (!node) {
		dev_err(dev, "Failed to convert remote node to device node\n");
		goto out_put_remote_node;
	}

	i2c_dev = of_find_i2c_device_by_node(node);
	if (!i2c_dev) {
		dev_err(dev, "No I2C device found for sensor\n");
		goto out_put_remote_node;
	}

	sensor_subdev = i2c_get_clientdata(i2c_dev);
	if (!sensor_subdev) {
		dev_err(dev, "No sensor subdev found\n");
		goto out_put_remote_node;
	}

	sensor_entity = &sensor_subdev->entity;
	dev_info(dev, "Successfully found sensor entity\n");

out_put_remote_node:
	fwnode_handle_put(remote_node);
out_put_remote_port:
	fwnode_handle_put(remote_port);
out_put_remote_endpoint:
	fwnode_handle_put(remote_endpoint);
out_put_endpoint:
	fwnode_handle_put(endpoint);

	return sensor_entity;
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
	struct media_entity *src, *sink, *sensor_src;
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

		if (j == 0) {
			sensor_src = get_sensor_media_entity(src);
			if (sensor_src) {
				rc = create_link(vdev->v4l2_dev.dev, sensor_src, 0, src, 0,
					MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
				if (rc < 0)
					return rc;
			}
		}

		j++;
	}

	j = 0;
	for (i = 0; i < links_size[scene]; i++) {
		struct entity_link *link = &links[scene][i];

		src = find_entity_by_name(&vdev->v4l2_dev, link->src_name);
		if (!src)
			continue;

		if (!strcmp(link->sink_name, "video-m2m")) {
			struct vid_m2m_video_device *v =
					create_m2m_video_device(vdev, j++);

			if (IS_ERR(v))
				return (int)PTR_ERR(v);

			sink = &v->video.entity;
			/* create video input (pad:0) to subdev (pad:0) link */
			rc = create_link(vdev->v4l2_dev.dev, sink, 0, src, 0, MEDIA_LNK_FL_ENABLED);
			if (rc < 0) {
				dev_err(vdev->v4l2_dev.dev,
					"failed to create link, src_pad=0, sink_pad=0\n");
				destroy_links(vdev);
				return rc;
			}
		} else if (!strcmp(link->sink_name, "video")) {
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
			dev_err(vdev->v4l2_dev.dev,
				"failed to create link, src_pad=%d, sink_pad=%d\n",
				link->src_pad, link->sink_pad);
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
	struct vid_m2m_video_device *vm;

	if (unlikely(!vdev))
		return;

	list_for_each_entry(sd, &vdev->v4l2_dev.subdevs, list)
		media_entity_remove_links(&sd->entity);

	list_for_each_entry(v, &vdev->video_device_list, entry) {
		media_entity_remove_links(&v->video.entity);
		destroy_video_device(v);
	}

	list_for_each_entry(vm, &vdev->m2m_video_device_list, entry) {
		media_entity_remove_links(&vm->video.entity);
		destroy_m2m_video_device(vm);
	}
}

int vid_subdev_set_cap(struct vid_device *vdev)
{
	struct vid_m2m_video_device *vm;
	struct vid_video_device *v;
	struct v4l2_subdev *sd;
	struct media_pad *pad;

	list_for_each_entry(vm, &vdev->m2m_video_device_list, entry) {
		pad = get_pad(&vm->video.entity, 0, true);
		(void)get_remote_pad_sd(pad, &sd);
		if (sd)
			v4l2_subdev_ctx_call_no_return(sd, set_cap);
	}

	list_for_each_entry(v, &vdev->video_device_list, entry) {
		(void)get_remote_pad_sd(&v->pad, &sd);
		if (sd)
			v4l2_subdev_ctx_call_no_return(sd, set_cap);
	}
	return 0;
}
