/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef HOBOT_DEV_CIM_H
#define HOBOT_DEV_CIM_H

#include <uapi/linux/types.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
// #include <linux/hobot_diag.h>
#include "vin_node_config.h"
#include "hobot_dev_vin_node.h"
#include "vio_config.h"
#include "vio_framemgr.h"
#include "vio_node_api.h"
//#include "acamera_aframe.h"

#include "hobot_vpf_manager.h"
// #include "cim_isp_param.h"
#include "osal.h"
#include "sif.h"

// #define CIM_DEBUG

/**
 * @def CIM_IPI_MAX_NUM
 * ipi max number
 * @NO{S10E04C01}
 */
#define CIM_IPI_MAX_NUM 4u

/**
 * @def FIFO_CH_MAX_NUM
 * chn max number
 * @NO{S10E04C01}
 */
#define FIFO_CH_MAX_NUM 16u
#define FIFO_W_CHN_MAX 6u
#define FIFO_R_CHN_MAX 2u
#define FIFO_DMA_NUM 2u

#define W_CHNS_EACH_IPI 3u

#define MAIN_WR_CH 0u
#define ROI_WR_CH 1u
#define EMB_WR_CH 2u
#define DDRIN_WR_CH 4u

#define IPI_SUPPORT_DDRIN 3u

#define FIFO_DMA0_REG_OFFSET 0x1000
#define FIFO_DMA1_REG_OFFSET 0x2000
#define FIFO_DMA_REG_SIZE 0x1000
#define FIFO_DMA_WCHN0_OFFSET 0x800
#define FIFO_DMA_RCHN0_OFFSET 0xC00
#define FIFO_DMA_CHN_REG_SIZE 0x40

#define IPI0_REG_OFFSET 0x3000
#define IPI_REG_SIZE 0x3000

#define DPC_OFFSET 0x200
#define RGBIR_OFFSET 0x400
#define RAWS_OFFSET 0x600
#define ISP_ROI_OFFSET 0x800
#define ROI_OFFSET 0xA00
#define EMB_OFFSET 0xC00
#define DEC_OFFSET 0x1000
#define CPD_OFFSET 0x2000

// #define CIM_ISP_COPS
// #define CIM_SENSOR_COPS

struct j6_cim_dev {
	struct sif_device sif;
	/**
	 * @var j6_cim_dev:s*pdev
	 * pointers to platform device drivers.
	 * range:N/A; default: N/A
	 */
	struct platform_device *pdev;
	/**
	 * @var j6_cim_dev::*base_reg
	 * ynr register base address.
	 * range:N/A; default: N/A
	 */
	void __iomem *base_reg;
	/**
	 * @var j6_cim_dev:regs_start
	 * The starting address of the cimdma register.
	 * range:N/A; default: N/A
	 */
	resource_size_t regs_start;
	/**
	 * @var j6_cim_dev:regs_end
	 * The end address of the cimdma register.
	 * range:N/A; default: N/A
	 */
	resource_size_t regs_end;
	/**
	 * @var j6_cim_dev:irq
	 * The interrupt number of the cimdma register.
	 * range:N/A; default: N/A
	 */
	s32 irq;
	/**
	 * @var j6_cim_dev:hw_id
	 * cimdma's hardware id, not used.
	 * range:N/A; default: N/A
	 */
	u32 hw_id;
	/**
	 * @var j6_cim_dev:state
	 * cimdma's working status.
	 * range:N/A; default: N/A
	 */
	u64 state;
	/**
	 * @var j6_cim_dev:*class
	 * device classes.
	 * range:N/A; default: N/A
	 */
	struct class *class;
	/**
	 * @var j6_cim_dev:cdev
	 * char device.
	 * range:N/A; default: N/A
	 */
	struct cdev cdev;
	/**
	 * @var j6_cim_dev:devno
	 * devno.
	 * range:N/A; default: N/A
	 */
	dev_t devno;
	/**
	 * @var j6_cim_dev:vin_node_dev
	 * point to vin_node_dev
	 * range:N/A; default: N/A
	 */
	void *vin_node_dev;
	//void *vnode[MAX_CHN_DEVICE];
	osal_atomic_t flow_id;
	/**
	 * @var j6_cim_dev:rsccount
	 * The number of times dimdma was started.
	 * range:N/A; default: N/A
	 */
	osal_atomic_t rsccount;
	/**
	 * @var j6_cim_dev:open_cnt
	 * The number of times dimdma was opened.
	 * range:N/A; default: N/A
	 */
	osal_atomic_t open_cnt;
	/**
	 * @var j6_cim_dev:slock
	 * slock.
	 * range:N/A; default: N/A
	 */
	osal_spinlock_t slock;
	/**
	 * @var j6_cim_dev:mlock
	 * mutex lock.
	 * range:N/A; default: N/A
	 */
	osal_mutex_t mlock;
	/**
	 * @var j6_cim_dev:sensor_fcount
	 *
	 * range:N/A; default: N/A
	 */
	osal_atomic_t sensor_fcount[CIM_IPI_MAX_NUM];
	/**
	 * @var j6_cim_dev:backup_fcount
	 *
	 * range:N/A; default: N/A
	 */
	osal_atomic_t backup_fcount[CIM_IPI_MAX_NUM];
	/**
	 * @var j6_cim_dev:enable_cnt
	 *
	 * range:N/A; default: N/A
	 */
	osal_atomic_t enable_cnt[CIM_IPI_MAX_NUM];
	/**
	 * @var j6_cim_dev:cur_output_flag
	 * flag
	 * range:N/A; default: N/A
	 */
	u32 cur_output_flag[CIM_IPI_MAX_NUM];
	/**
	 * @var j6_cim_dev:vnode
	 * point to the vin_node's vnode
	 * range:N/A; default: N/A
	 */
	struct vio_node *vnode[VIO_MAX_STREAM];

	u32 sw_drop_count[VIO_MAX_STREAM];
	u32 isp_drop_count[VIO_MAX_STREAM];
	u32 hw_drop_count[VIO_MAX_STREAM];
#ifdef CONFIG_HOBOT_VIO_STL
	u32 fusa_enable;
	u64 jiffi;
	struct vio_stl stl;
	u32 last_frameid[CIM_IPI_MAX_NUM];
	u32 error_cnt[CIM_IPI_MAX_NUM];
#endif
	struct vio_callback_ops *sensor_cops;
	struct vio_callback_ops *isp_cops;
	u32 global_time_type;
};
struct j6_cim_dev *cim_get_dev(u8 hw_id);
void cim_set_dev(struct j6_cim_dev *cim_dev, u8 hw_id);
void cim_fusa_setup_ops(struct j6_cim_dev *cim);
void cim_fusa_enable(const struct j6_cim_dev *cim, u32 enable);
// void cim_stl_send_diag_event(u16 error_code, enum diag_msg_level prio, u16 id);
void cim_set_clk_enable(u32 enable);
// void cim_send_diag_error_event(enum vio_event_id id,
// enum diag_msg_level prio, u8 sub_id, u8 pri_data, u16 line_num);
// int j6_cim_fault_injection_show(struct device *dev, char *buf, size_t size);
// int j6_cim_fault_injection_store(struct device *dev, const char *buf, size_t count);
// void cim_fusa_enable_rg_parity_irq(struct j6_cim_dev *cim, u32 enable);

#endif /*HOBOT_DEV_CIM_H*/
