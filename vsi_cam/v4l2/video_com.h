/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _VIDEO_COM_H_
#define _VIDEO_COM_H_
#include <media/media-device.h>

#include "vid_drv.h"
#include "video_fmt.h"

#define IS_CAPTURE_V4L2_TYPE(type) \
	((type) == V4L2_BUF_TYPE_VIDEO_CAPTURE)

#define IS_OUTPUT_V4L2_TYPE(type) \
	((type) == V4L2_BUF_TYPE_VIDEO_OUTPUT)

static int vid_querycap(struct file *file, void *fh,
			struct v4l2_capability *cap)
{
	strscpy(cap->driver, VID_DEV_NAME, sizeof(cap->driver));
	strscpy(cap->card, VID_DEV_NAME, sizeof(cap->card));
	snprintf((char *)cap->bus_info, sizeof(cap->bus_info),
		 "%s", VID_BUS_INFO);
	return 0;
}

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

static void init_fmt(struct v4l2_format *f)
{
	u32 bytesperline, sizeimage, bpp, bit_depth;

	switch (f->fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		bit_depth = 8;
		bpp = 1;
		break;
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_YUYV:
		bit_depth = 16;
		bpp = 2;
		break;
	case V4L2_PIX_FMT_NV12:
		bit_depth = 12;
		bpp = 1;
		break;
	case V4L2_PIX_FMT_NV16:
		bit_depth = 16;
		bpp = 1;
		break;
	case V4L2_PIX_FMT_RGB32:
		bit_depth = 32;
		bpp = 4;
		break;
	default:
		break;
	}

	v4l_bound_align_image(&f->fmt.pix.width, MIN_W, MAX_W, ALIGN_W,
			      &f->fmt.pix.height, MIN_H, MAX_H, ALIGN_H, 0);
	bytesperline = ALIGN(f->fmt.pix.width * bpp, STRIDE_ALIGN);
	if (bpp == 1)
		sizeimage = f->fmt.pix.height * (bytesperline * bit_depth / 8);
	else
		sizeimage = f->fmt.pix.height * bytesperline;

	if (f->fmt.pix.bytesperline < bytesperline)
		f->fmt.pix.bytesperline = bytesperline;
	if (f->fmt.pix.sizeimage < sizeimage)
		f->fmt.pix.sizeimage = sizeimage;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
}

static int vid_check_pixelformat(struct media_pad *pad, u32 pixelformat)
{
	struct v4l2_subdev *sd;
	struct media_pad *r_pad = get_remote_pad_sd(pad, &sd);
	int i = 0, rc;
	u32 fmt;

	if (!sd)
		return -EINVAL;

	for (;;) {
		rc = v4l2_subdev_ctx_call(sd, enum_format, r_pad->index, i++, &fmt);
		if (rc == -EINVAL)
			return 1;

		if (rc < 0)
			return rc;

		if (fmt == pixelformat)
			return 0;
	}
	return 0;
}

static int get_def_fmt(struct media_pad *pad, struct v4l2_format *f)
{
	struct v4l2_subdev *sd;
	struct media_pad *r_pad = get_remote_pad_sd(pad, &sd);
	struct video_fmt *v_f;
	struct v4l2_frmsizeenum fsize;
	u32 pixelformat = f->fmt.pix.pixelformat, width, height;
	int rc;

	if (!sd)
		return -EINVAL;

	rc = vid_check_pixelformat(pad, pixelformat);
	if (rc < 0)
		return rc;

	if (rc) {
		rc = v4l2_subdev_ctx_call(sd, enum_format, r_pad->index, 0, &pixelformat);
		if (rc < 0)
			return rc;
	}

	memset(&fsize, 0, sizeof(fsize));
	fsize.pixel_format = pixelformat;
	rc = v4l2_subdev_ctx_call(sd, enum_framesize, r_pad->index, &fsize);
	if (rc < 0)
		return rc;

	if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
		width = fsize.discrete.width;
		height = fsize.discrete.height;
	} else if (fsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
		width = fsize.stepwise.min_width;
		height = fsize.stepwise.max_width;
	} else {
		return -EINVAL;
	}

	v_f = get_fmt_by_pixelformat(pixelformat);
	if (!v_f)
		return -EINVAL;

	f->fmt.pix.pixelformat = pixelformat;
	f->fmt.pix.width = width;
	f->fmt.pix.height = height;
	init_fmt(f);
	return 0;
}

#endif /* _VIDEO_COM_H_ */
