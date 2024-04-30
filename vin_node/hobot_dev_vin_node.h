/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef HOBOT_DEV_VIN_NODE_H
#define HOBOT_DEV_VIN_NODE_H

#ifdef HOBOT_MCU_CAMSYS

#else
#include <linux/cdev.h>
// #include <linux/hobot_diag.h>
#include <linux/interrupt.h>
#endif

#include "vio_config.h"
#include "vio_framemgr.h"
#include "vio_node_api.h"
#include "hobot_vin_common.h"
#include "vin_node_config.h"
#include "hobot_dev_cim.h"

/**
 * @def SRC_INDEX
 * Index of src node
 * @NO{S10E01C01}
 */
#define SRC_INDEX 0
/**
 * @def CAP_INDEX
 * Index of cap node
 * @NO{S10E01C01}
 */
#define CAP_INDEX 1
/**
 * @def EMB_INDEX
 * Index of emb node
 * @NO{S10E01C01}
 */
#define EMB_INDEX 3
/**
 * @def ROI_INDEX
 * Index of roi node
 * @NO{S10E01C01}
 */
#define ROI_INDEX 4


/**
 * @struct vin_node_subdev
 * @brief VIN_NODE sub-device definition.
 * @NO{S10E01C01}
 */
struct vin_node_subdev {
	/**
	 * @var vin_node_subdev::vdev
	 * Vio's sub-device abstraction.
	 * range:N/A; default: N/A
	 */
	struct vio_subdev vdev;
	/**
	 * @var vin_node_subdev::*vin_node_dev
	 * pointer to vin_node_dev.
	 * range:N/A; default: N/A
	 */
	struct j6_vin_node_dev *vin_node_dev;
	/**
	 * @var vin_node_subdev::vin_attr
	 * Initialization parameters of the vin_node.
	 * range:N/A; default: N/A
	 */
	vin_attr_t  vin_attr;
	/**
	 * @var vin_node_subdev::cim_private_attr
	 * Initialization private parameters of the cim.
	 * range:N/A; default: N/A
	 */
	vin_cim_private_t  cim_private_attr;  //保存CIM一些私有配置
	/**
	 * @var vin_node_subdev::pre_int_mask
	 * Early interruption of the cim.
	 * range:N/A; default: N/A
	 */
	u32 pre_int_mask;
	/**
	 * @var vin_node_subdev::cfg_index
	 * Use this index for multi-process scenarioss
	 * range:N/A; default: N/A
	 */
	u8  cfg_index;
	/**
	 * @var vin_node_subdev::online_mode
	 * Online mode
	 * range:N/A; default: N/A
	 */
	u8  online_mode;
	/**
	 * @var vin_node_subdev::state
	 * record state of modules in vin_node
	 * range:N/A; default: N/A
	 */
	volatile uint64_t state;
	/**
	 * @var vin_node_subdev::ctx_id
	 * context id number
	 * range:N/A; default: N/A
	 */
	u8  ctx_id;
};

/**
 * @struct j6_vin_node_dev
 * @brief VIN_NODE device definition.
 * @NO{S10E01C01}
 */
struct j6_vin_node_dev {
	/**
	 * @var j6_vin_node_dev::state
	 * vin_node's working status.
	 * range:N/A; default: N/A
	 */
	unsigned long state;
	struct platform_device *pdev;
	/**
	 * @var j6_vin_node_dev::hw_id
	 * vin_node's hardware id, not used.
	 * range:N/A; default: N/A
	 */
	u32 hw_id;
	u32 flow_id;
	/**
	 * @var j6_vin_node_dev::vps_device
	 * VPF device node.
	 * range:N/A; default: N/A
	 */
	struct vpf_device vps_device[VIN_NODE_MAX_DEVICE];
	/**
	 * @var j6_vin_node_dev::open_cnt
	 * The number of times vin_node was opened.
	 * range:N/A; default: N/A
	 */
	osal_atomic_t open_cnt;
	/**
	 * @var j6_vin_node_dev::vin_ops
	 * vin_node's registration function for other moduless
	 * range:N/A; default: N/A
	 */
	struct vin_common_ops *vin_ops[VIN_MAX_DEVICES];
	/**
	 * @var j6_vin_node_dev::mlock
	 * mutex lock.
	 * range:N/A; default: N/A
	 */
	osal_mutex_t mlock;
	/**
	 * @var j6_vin_node_dev::src_subdev
	 * Abstraction of src nodes
	 * range:N/A; default: N/A
	 */
	struct vin_node_subdev src_subdev[VIO_MAX_STREAM];
	/**
	 * @var j6_vin_node_dev::cap_subdev
	 * Abstraction of cap nodes
	 * range:N/A; default: N/A
	 */
	struct vin_node_subdev cap_subdev[VIO_MAX_STREAM];
	/**
	 * @var j6_vin_node_dev::emb_subdev
	 * Abstraction of emb nodes
	 * range:N/A; default: N/A
	 */
	struct vin_node_subdev emb_subdev[VIO_MAX_STREAM];
	/**
	 * @var j6_vin_node_dev::roi_subdev
	 * Abstraction of roi nodes
	 * range:N/A; default: N/A
	 */
	struct vin_node_subdev roi_subdev[VIO_MAX_STREAM];
	/**
	 * @var j6_vin_node_dev::ich_vdev
	 * Abstraction of input channels
	 * range:N/A; default: N/A
	 */
	struct vio_subdev ich_vdev[MAX_CHN_DEVICE];   //输入chn
	/**
	 * @var j6_vin_node_dev::och_vdev
	 * Abstraction of output channels
	 * range:N/A; default: N/A
	 */
	struct vio_subdev och_vdev[MAX_CHN_DEVICE];  // 输出chn
	/**
	 * @var j6_vin_node_dev::vnode
	 * One vnode along the way
	 * range:N/A; default: N/A
	 */
	struct vio_node  vnode[VIO_MAX_STREAM];   // 一路对应一个vnode
	/**
	 * @var j6_vin_node_dev::gtask
	 * vin_node worker thread.
	 * range:N/A; default: N/A
	 */
	struct vio_group_task gtask;
	/**
	 * @var j6_vin_node_dev::flag
	 * Save some global variables
	 * range:N/A; default: N/A
	 */
	u32 flag[VIO_MAX_STREAM];  //ctx_id 全局变量，输入和输出chn都需要的一些配置
	struct j6_cim_dev *cim_dev;

};

typedef void (*vin_get_frame_info_callback)(struct vio_node *vnode);
void vin_register_device_node(vin_type_e type, struct vin_common_ops *ops);
void vin_register_device_node_hwid(vin_type_e type, u8 hw_id, struct vin_common_ops *ops);
void vin_register_frame_info_func(vin_get_frame_info_callback cim_get_frame_info);
s32 vin_node_device_node_init(u8 hw_id);
void vin_node_device_node_deinit(u8 hw_id);
s32 vin_device_node_init(u32 hw_id, void *cim);
s32 vin_device_node_deinit(u32 hw_id);
void vin_unregister_device_node(vin_type_e type);
struct class* vin_node_get_class(void);
u32 vin_get_perline_size(u32 width, u8 pack_mode, u32 format);

#endif /*HOBOT_DEV_VIN_NODE_H*/
