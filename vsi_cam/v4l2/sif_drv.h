/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _SIF_DRV_H_
#define _SIF_DRV_H_

#include "sif.h"
#include "utils.h"

struct sif_v4l_instance {
	struct subdev_node node;
	struct sif_device *dev;
	struct cam_ctx buf_ctx;
	struct v4l2_subdev_format out_fmt;
	u32 out_pixelformat;
	u32 id;
	int enabled;
	bool en_post;
};

struct sif_v4l_device {
	struct sif_device sif_dev;
	struct sif_v4l_instance *insts;
};

#endif /* _SIF_DRV_H_ */
