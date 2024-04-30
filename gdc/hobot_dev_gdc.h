/**
 * @file: hobot_dev_gdc.h
 * @
 * @NO{S09E03C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */

#ifndef HOBOT_GDC_DEV_H
#define HOBOT_GDC_DEV_H

#ifdef HOBOT_MCU_CAMSYS

#else
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#endif
#include "gdc_config.h"
#include "vio_node_api.h"

/**
 * @def GDC_NAME
 * driver name
 * @NO{S09E03C01}
 */
#define GDC_NAME  "hobot-gdc"

/**
 * @def GDC_MAX_DEVICE
 * hardware quantity
 * @NO{S09E03C01}
 */
#define GDC_MAX_DEVICE  1

#define GDC0_HW_ID  0u
#define GDC1_HW_ID  1u
#define GDC_MAX_HW_ID  2u
/**
 * @enum gdc_dev_status
 * Describe gdc working mode
 * @NO{S09E03C01}
 */
enum gdc_dev_state {
	GDC_DEV_NONE, /**< NONE status */
	GDC_DEV_PROCESS, /**< PROCESS status */
	GDC_DEV_FREE, /**< FREE status */
};

struct gdc_iommu_addr {
     u32 bin_iommu_addr;
     u32 in_iommu_addr[VIO_BUFFER_MAX_PLANES];
     u32 out_iommu_addr[VIO_BUFFER_MAX_PLANES];
};
/**
 * @struct gdc_subdev
 * @brief gdc sub-device definition.
 * @NO{S09E03C01}
 */
struct gdc_subdev {
   /**
     * @var gdc_subdev::vdev
     * Vio's sub-device abstraction.
     * range:N/A; default: N/A
     */
	struct vio_subdev vdev;
    /**
     * @var gdc_subdev::*gdc
     * pointer to gdc.
     * range:N/A; default: N/A
     */
	struct hobot_gdc_dev *gdc;
    /**
     * @var gdc_subdev::gdc_cfg
     * gdc cfg parameters.
     * range:N/A; default: N/A
     */
	gdc_settings_t gdc_setting;

     struct vio_frame bin_frame;
     struct gdc_iommu_addr map_addr;
};

/* same as struct cam_ctrl_device MUST */
struct gdc_wrapper {
	struct device *dev;
	void __iomem *base;
};

/**
 * @struct hobot_gdc_dev
 * @brief gdc device definition.
 * @NO{S09E03C01}
 */
struct hobot_gdc_dev {
    #ifdef HOBOT_MCU_CAMSYS
    #else
   /**
     * @var hobot_gdc_dev::*pdev
     * pointers to platform device drivers.
     * range:N/A; default: N/A
     */
	struct platform_device *pdev;
    /**
     * @var hobot_gdc_dev::*class
     * device classes.
     * range:N/A; default: N/A
     */
	struct class *class;
   /**
     * @var hobot_gdc_dev::cdev
     * char device.
     * range:N/A; default: N/A
     */
	struct cdev cdev;
   /**
     * @var hobot_gdc_dev::cdev
     * char device.
     * range:N/A; default: N/A
     */
	dev_t devno;
   #endif
   /**
     * @var hobot_gdc_dev::*base_reg
     * gdc register base address.
     * range:N/A; default: N/A
     */
	void __iomem *base_reg;
   /**
     * @var hobot_gdc_dev::regs_start
     * The starting address of the gdc register.
     * range:N/A; default: N/A
     */
	resource_size_t regs_start;
   /**
     * @var hobot_gdc_dev::regs_end
     * The end address of the gdc register.
     * range:N/A; default: N/A
     */
	resource_size_t regs_end;
    /**
     * @var hobot_gdc_dev::irq
     * The interrupt number of the gdc register.
     * range:N/A; default: N/A
     */
	s32 irq;
   /**
     * @var hobot_gdc_dev::share_slock
     * spinlock.
     * range:N/A; default: N/A
     */
	osal_spinlock_t shared_slock;
    /**
     * @var hobot_gdc_dev::mlock
     * mutex lock.
     * range:N/A; default: N/A
     */
	osal_mutex_t mlock;
    /**
     * @var hobot_gdc_dev::ctx_id
     * pointers to platform device drivers.
     * range:N/A; default: N/A
     */
	osal_atomic_t ctx_id;
    /**
     * @var hobot_gdc_dev::open_cnt
     * The number of times gdc was opened.
     * range:N/A; default: N/A
     */
	osal_atomic_t open_cnt;
    /**
     * @var hobot_gdc_dev::hw_id
     * gdc's hardware id.
     * range:N/A; default: N/A
     */
	u32 hw_id;
	/**
	 * @var hobot_gdc_dev::state
	 * gdc's working status.
	 * range:N/A; default: N/A
	 */
	u32 state;
    /**
     * @var hobot_gdc_dev::gdc_hw_resource
     * hardware semaphore resource.
     * range:N/A; default: N/A
     */
	osal_sem_t gdc_hw_resource;
    /**
     * @var hobot_gdc_dev::gdc_done_resource
     * hardware complete semaphore resource.
     * range:N/A; default: N/A
     */
	osal_sem_t gdc_done_resource;
    /**
     * @var hobot_gdc_dev::gtask
     * gdc worker thread.
     * range:N/A; default: N/A
     */
	struct vio_group_task gtask;
    /**
     * @var hobot_gdc_dev::vps_device
     * VPF device node.
     * range:N/A; default: N/A
     */
	struct vpf_device vps_device[2];
    /**
     * @var hobot_gdc_dev::subdev
     * gdc srouce channel sub-device.
     * range:N/A; default: N/A
     */
	struct gdc_subdev subdev[VIO_MAX_STREAM][2];
    /**
     * @var hobot_gdc_dev::vnode
     * gdc abstract virtual node.
     * range:N/A; default: N/A
     */
	struct vio_node vnode[VIO_MAX_STREAM];

	struct gdc_wrapper *wrap;

	struct vio_hw_loading loading;

	struct vio_stl stl;
};

struct hobot_gdc_dev *gdc_get_drvdata(u32 hw_id);
#endif
