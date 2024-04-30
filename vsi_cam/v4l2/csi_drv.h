/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CSI_DRV_H_
#define _CSI_DRV_H_

#include "csi.h"
#include "utils.h"

struct csi_v4l_instance {
	struct subdev_node node;
	struct csi_device *dev;
	bool is_idi_mode;
	u32 id;
};

struct csi_v4l_device {
	struct csi_device csi_dev;
	struct csi_v4l_instance *insts;
};

long csi_idi_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg);
#endif /* _CSI_DRV_H_ */
