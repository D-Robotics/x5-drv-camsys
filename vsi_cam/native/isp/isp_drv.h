/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ISP_DRV_H_
#define _ISP_DRV_H_

#include "isp_api.h"
#include "vio_node_api.h"

#include "isp.h"

#define ISP_NODE_MAX_DEVICE  2
#define SRC_INDEX 0
#define CAP_INDEX 1

enum isp_chn_type {
	ISP_MAIN_FRAME,
	ISP_TYPE_INVALID,
};

enum isp_ichn_type {
	ISP_DDRIN = 2,
	ISP_EMB,
	ISP_HDR_SUB
};

struct isp_nat_instance {
	u32 id;
	struct vio_subdev vdev;
	struct isp_nat_device *dev;
	struct isp_irq_ctx ctx;
	isp_attr_t attr;
	isp_ichn_attr_t ichn_attr;
	isp_ochn_attr_t ochn_attr;
	u8 online_mode;
	int stream_idx;
};

struct isp_nat_device {
	struct isp_device isp_dev;
	struct vio_group_task gtask;
	struct vio_node vnode[VIO_MAX_STREAM];
	struct isp_nat_instance src_instance[VIO_MAX_STREAM];
	struct isp_nat_instance cap_instance[VIO_MAX_STREAM];
	struct vpf_device vps_dev[ISP_NODE_MAX_DEVICE];
	struct vio_node *stream_vnodes[ISP_SINK_ONLINE_PATH_MAX];
	struct mutex stream_vnodes_lock; /* lock for stream_vnodes */
	struct vio_callback_ops *sensor_ops;
};

#endif /* _ISP_DRV_H_ */
