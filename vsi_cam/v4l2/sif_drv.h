/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _SIF_DRV_H_
#define _SIF_DRV_H_

#include "sif.h"
#include "utils.h"

#define SIF_FMT_MAX (15)

struct sif_v4l_instance {
	struct subdev_node node;
	struct sif_device *dev;
	struct cam_ctx src_ctx;
	struct cam_ctx buf_ctx;
	struct mutex open_lock;
	struct mutex fmt_lock;
	u32 id;
	int enabled;
	bool en_post;
	bool fmt_changed;
	struct cam_format fmt;
	u32 fmt_cap[SIF_FMT_MAX]; /* pixelformat */
	u32 fmt_cap_num;
	u32 conv_nv12;
	refcount_t start_refcnt;
	refcount_t open_count;
};

struct sif_v4l_device {
	struct sif_device sif_dev;
	struct sif_v4l_instance *insts;
};

#endif /* _SIF_DRV_H_ */
