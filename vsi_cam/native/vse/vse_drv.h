/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _VSE_DRV_H_
#define _VSE_DRV_H_

#include "vio_node_api.h"
#include "vse_api.h"

#include "vse.h"

#define SRC_INDEX  0
#define CAP_INDEX_1ST 1
#define CAP_INDEX_END 6
#define VSE_NODE_MAX_DEVICE  (VSE_OUT_CHNL_MAX + 1)

enum vse_chn_type {
	VSE_MAIN_FRAME,
	VSE_TYPE_INVALID,
};

enum vse_ichn_type {
	VSE_DDRIN = 2,
};

enum vse_ochn_type_e {
	VSE_DOWN_SCALE_4K,
	VSE_DOWN_SCALE_1080P0,
	VSE_DOWN_SCALE_1080P1,
	VSE_DOWN_SCALE_720P0,
	VSE_DOWN_SCALE_720P1,
	VSE_UP_SCALE_4K,
	VSE_OCHN_MAX
};

struct vse_nat_instance {
	u32 id;
	struct vio_subdev vdev;
	struct vse_nat_device *dev;
	vse_attr_t attr;
	vse_ichn_attr_t ichn_attr;
	vse_ochn_attr_t ochn_attr;
	u8 online_mode;
	// struct vio_osd_info osd_info;
};

struct vse_nat_device {
	struct vse_device vse_dev;
	struct vio_group_task gtask;
	struct vio_node vnode[VIO_MAX_STREAM];
	struct vse_nat_instance src_instance[VIO_MAX_STREAM];
	struct vse_nat_instance cap_instance[VSE_OUT_CHNL_MAX][VIO_MAX_STREAM];
	struct vpf_device vps_dev[VSE_NODE_MAX_DEVICE];
};

// static struct vse_interface_ops vse_cb_ops {
// 	void (*get_osd_info)(int32_t inst_id, int32_t ochn_id);
// };

#endif /* _VSE_DRV_H_ */
