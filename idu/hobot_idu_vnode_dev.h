/*
 * hobot-drivers/camsys/idu/hobot_idu_vnode_dev.h
 *
 * Copyright (C) 2020 horizon
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef HOBOT_IDU_VNODE_DEV_H
#define HOBOT_IDU_VNODE_DEV_H
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include <osal_semaphore.h>
#include <osal_spinlock.h>
#include <osal_bitops.h>
#include <vio_mem.h>
#include <vio_framemgr.h>
#include <vio_config.h>

#include <hb_idu_hw.h>
#include "hobot_idu_vnode_ops.h"

/**
 * The maximum number of pipelines that idu can handle
 */
#define MAX_IDU_PROCESS_PIPELINE 1

/**
 * The number of output channels registered by idu
 */
#define IDU_OCH_NUM 3

/**
 * The maximum number of device nodes that idu can register
 */
#define IDU_MAX_DEVICE ((IDU_ICHN_NUM) + (IDU_OCH_NUM))

/**
 * The ID of idu feedback mode
 */
#define IDU_FEEDBACK_MODE 0

/**
 * The ID of idu flow mode
 */
#define IDU_FLOW_MODE 1

#define IDU_DEV_NUM 2

enum idu_ext_command {
	IDU_EXT_GET_DONE_FLAG = 0x1000U,
	IDU_EXT_GET_WAIT_VSYNC = 0x1001U,
};

typedef enum _idu_ochn_type_e {
	IDU_OCH_MIPI_CSI_DEV = 0,
	IDU_OCH_MIPI_DSI,
	IDU_OCH_WRITEBACK,
	IDU_OCH_MAX,
} idu_ochn_type_e;

typedef enum _idu_ichn_type_e {
	IDU_ICH_YUV_LAYER0 = 0,
	IDU_ICH_YUV_LAYER1,
	IDU_ICH_RGB_LAYER0,
	IDU_ICH_RGB_LAYER1,
#ifdef CONFIG_HOBOT_CHIP_J6X
	IDU_ICH_YUV_LAYER2,
	IDU_ICH_YUV_LAYER3,
#endif
	IDU_ICH_MAX,
} idu_ichn_type_e;

/**
 * @struct idu_subdev
 * @brief idu internal Channel Abstract Data Structure
 * @NO{S09E04C01}
 */
struct idu_subdev {
	struct vio_subdev   vdev;
	/**< VPF channel abstract data structure.
	 * range:[0, ); default: 0
	 */
	struct idu_subnode *subnode;
	/**< Points to the idu pipeline node which the current channel belongs.
	 * range:[NULL, ); default: NULL
	 */
	struct vio_node	   *vnode;
	/**< Points to the VPF pipeline node which the current channel belongs.
	 * range:[NULL, ); default: NULL
	*/
};

/**
 * @struct idu_subnode
 * @brief idu pipeline node related parameters
 * @NO{S09E04C01}
 */
struct idu_subnode {
	uint32_t	  flow_id;
	/**< current flow ID.
	 * range:[0, ); default: 0
	*/
	uint32_t	  mode;
	/**< flow mode or feedback mode.
	 * range:[0, 1]; default: 0
	 */
	struct idu_subdev idu_ichn_sdev[IDU_ICHN_NUM];
	/**< Data channels, four input channels and one output channel.
	 * range:[0, ); default: 0
	 */
	struct idu_subdev idu_ochn_sdev[IDU_OCH_NUM];

	struct disp_cfg_s idu_cfg;

	struct hobot_idu_dev *idu;
	/**< pointer to the idu ip struct
	 * range:[NULL, ); default: NULL
	*/
	uint64_t	      frame_state;
	/**< Mark the current status of each data frame
	 * Multi-channel synchronization status, the lower four bits are valid
	 * range:[0, 15]; default: 0
	 */

	struct vio_frame *src_frames[IDU_ICHN_NUM];
	/**< Point to the ready input frame data
	 * range:[NULL, ); default: NULL
	*/
	struct vio_frame *dst_frame;
	/**< Point to the ready output frame data
	 * range:[NULL, ); default: NULL
	*/
	struct hobot_idu_vsync vsync;

	osal_spinlock_t	vsync_lock;

	wait_queue_head_t done;

	uint32_t frame_end;
};

/**
 * @struct idu_cfg_buf
 * @brief Configuration word memory resource management data structure
 * @NO{S09E04C01}
 */
struct idu_cfg_buf {
	struct ion_handle *handle;
	/**< ION handle pointer
	 * range:[NULL, ); default: NULL
	*/
	size_t		   size;
	/**< Memory block size
	 * range:[0, ); default: 0
	*/
	u64		   paddr[MAX_IDU_PROCESS_PIPELINE];
	/**< Physical base address of each configuration word
	 * range:[0, ); default: 0
	*/  //Memory address
	u64		   iommu_addr[MAX_IDU_PROCESS_PIPELINE];
	/**< IOMMU-mapped base address of each configuration word
	 * range:[0, ); default: 0
	 */ //iommu map
	char		  *vaddr[MAX_IDU_PROCESS_PIPELINE];
	/**< Base address of each configuration word mapped to the kernel
	 * range:[0, ); default: 0
	*/ //map kernel address
};

struct hobot_idu_debug_priv {
	uint32_t	id;
	uint32_t	config_log;

};

/**
 * @struct hobot_idu_dev
 * @brief Stich IP attribute abstract data structure
 * @NO{S09E04C01}
 */
struct hobot_idu_dev {
	struct platform_device *pdev;
	/**< Linux platform device data structure pointer
	 * range:[0, ); default: 0
	*/

	int32_t port;

	void __iomem	 *base_reg;
	/**< Base address of IP register mapping
	 * range:[NULL, ); default: NULL
	*/
	resource_size_t	  regs_start;
	/**< Base address of IP register mapping
	 * range:[0, ); default: 0
	 */
	resource_size_t	  regs_end;
	/**< Base address of IP register mapping
	 * range:[0, ); default: 0
	*/
	int32_t		  irq;
	/**< Interrupt number in request func
	 * range:[0, ); default: 0
	 */
	struct vpf_device vps_device[IDU_MAX_DEVICE];
	/**< Character device data structure encapsulated by VPF
	 * range:[0, ); default: 0
	*/

	atomic_t	      open_cnt;
	/**< idu IP open conuter
	 * range:[0, ); default: 0
	 */
	uint32_t	      state;
	struct vio_group_task gtask;

	struct idu_cfg_buf cfg_buff;
	/**< Configuration word memory resource management data structure
	 * range:[0, ); default: 0
	 */
	struct idu_subnode subnode;
	/**< idu channel node abstract data structure
	 * range:[0, ); default: 0
	 */
	struct vio_node	   vnode;
	/**< VPF channel node abstract data structure
	 * range:[0, ); default: 0
	 */

	osal_mutex_t	 mlock;
	/**< Mutex lock for attribute access of idu IP
	 * range:[0, ); default: 0
	 */
	osal_sem_t hw_resource;
	/**< idu IP processing resource semaphore
	 * range:[0, ); default: 0
	 */
	osal_sem_t done_resource;

	struct dentry	*debugfs;

	struct hobot_idu_debug_priv debug;

	/**< idu interrupt synchronization semaphore
	 * range:[0, ); default: 0
	 */
	struct dc_hw	*hw;

	struct mipi_csi_dev_priv_s *csi_priv;

	struct mipi_dsi_host_priv_s *dsi_priv;
};

#endif //HOBOT_IDU_VNODE_DEV_H