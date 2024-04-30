/**
 * @file: hobot_vpf_manager.h
 * @
 * @NO{S09E05C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */
/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef HOBOT_VPF_MANAGER_H
#define HOBOT_VPF_MANAGER_H

#ifndef HOBOT_MCU_CAMSYS
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#endif
#include "vio_node_api.h"
#include "vio_chain_api.h"

/**
 * @def MAX_DEVICE
 * maximum vpf device used to register device node;
 */
#define MAXIMUM_DEVICE  128
#define MINOR_BITS	32
/**
 * @def MINOR_NUM
 * minor array length;
 */
#define MINOR_NUM (MAXIMUM_DEVICE / MINOR_BITS)

#define VPF_COPS_NUM COPS_NUM
/**
 * @struct hobot_vpf_dev
 * @brief Define the descriptor of vpf driver device.
 * @NO{S09E05C01}
 */
struct hobot_vpf_dev {
#ifdef HOBOT_MCU_CAMSYS
#else
	struct class *class;
	struct cdev cdev;
	struct device *dev;

	dev_t devno;
#endif

	unsigned long state;

	u32 minor[MINOR_NUM];	/* minor id. 128 columns. */
	struct vpf_device *vpf_device[MAXIMUM_DEVICE];
	struct vio_core iscore;
	struct vpf_device vflow_dev;
	osal_atomic_t open_cnt;
	osal_mutex_t mlock;
	u32 stat_flow_id;
	u32 flowid_mask;
	struct vio_callback_ops vio_cops[MODULE_NUM][VPF_COPS_NUM];

	/* debug sys */
	struct dentry *debug_root;
	struct dentry *debug_file_fmgr_stats;
};

void vpf_set_drvdata(struct hobot_vpf_dev *vpf_dev);
struct hobot_vpf_dev *vpf_get_drvdata(void);

s32 hobot_vpf_manager_probe(void);
void hobot_vpf_manager_remove(void);
#endif
