/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/**
 * @file hobot_mipi_csi.h
 *
 * @NO{S10E03C01}
 * @ASIL{B}
 */

#ifndef __HOBOT_MIPI_CSI_H__
#define __HOBOT_MIPI_CSI_H__ /* PRQA S 0603 */ /* header file macro */

#include <linux/types.h>

#include "hobot_mipi_host_ops.h"
#include "hobot_mipi_host_regs.h"
#include "hobot_mipi_phy.h"
#include "hobot_mipi_host.h"
#include "hobot_mipi_dev.h"

/**
 * @def MIPI_CSI_RXNUM
 * max number of rx csi device
 */
#define MIPI_CSI_RXNUM	MIPI_HOST_MAX_NUM
/**
 * @def MIPI_CSI_TXNUM
 * max number of tx csi device
 */
#define MIPI_CSI_TXNUM	MIPI_DEV_MAX_NUM

/**
 * @def MIPI_CSI_RX_ATTACHED
 * flag for rx device attached
 */
#define MIPI_CSI_RX_ATTACHED	(0x1U)
/**
 * @def MIPI_CSI_TX_ATTACHED
 * flag for tx device attached
 */
#define MIPI_CSI_TX_ATTACHED	(0x2U)

/**
 * @def MIPI_CSI_IPI_RESET_ENABLE
 * ipi reset attr to ipi enable
 */
#define MIPI_CSI_IPI_RESET_ENABLE(n)	((n) & 0xFFFFU)
/**
 * @def MIPI_CSI_IPI_RESET_MASK
 * ipi reset attr to ipi mask
 */
#define MIPI_CSI_IPI_RESET_MASK(n)	(((((n) >> 16) & 0xFFFFU) != 0U) ? (((n) >> 16) & 0xFFFFU) : 0xFU)

/**
 * @enum _mipi_dbg_hook_e
 * mipi debug hook callback type
 * @NO{S10E03C01}
 */
typedef enum _mipi_dbg_hook_e {
	MIPI_CSI_DBG_HOOKCALL = 0,
	MIPI_CSI_DBG_HOOKPRE,
	MIPI_CSI_DBG_HOOKPOST,
	MIPI_CSI_DBG_HOOKERR,
	MIPI_CSI_DBG_HOOKINV,
} mipi_dbg_hook_e;

#if 1
#include "vin_node_config.h"
#else
/**
 * @struct mipi_attr_s
 * mipi base config struct as attr, include rx and tx
 * @NO{S10E03C01}
 */
typedef struct mipi_attr_s {
	int32_t attr_valid;
	int32_t attach;		// attach操作类型.
	int32_t rx_enable;	// RX设备使能.
	int32_t tx_enable;	// TX设备使能.
	int32_t tx_index;	// TX设备选择.

	struct mipi_host_cfg_s rx_attr;		// RX设备属性.
	struct mipi_dev_cfg_s tx_attr;		// TX设备属性.
} mipi_attr_t;  // mipi属性结构体.

/**
 * @struct mipi_attr_ex_s
 * mipi param config struct as extra attr, include rx and tx
 * @NO{S10E03C01}
 */
typedef struct mipi_attr_ex_s {
	int32_t attr_valid;
	int32_t reserved;
	uint64_t rx_ex_mask;	// RX增强属性掩码.
	uint64_t tx_ex_mask;	// TX增强属性掩码.

	struct mipi_host_param_s rx_attr_ex;// RX增强属性;
	struct mipi_dev_param_s tx_attr_ex; // TX增强属性;
} mipi_attr_ex_t;
#endif

/**
 * @struct mipi_csi_device_s
 * device struct of mipi csi driver
 * @NO{S10E03C01}
 */
typedef struct mipi_csi_device_s {
	osal_mutex_t mutex;
	int32_t index;
	uint32_t open_cnt;
	struct mipi_attr_s config;
	struct mipi_attr_ex_s param;
} mipi_csi_device_t;

/**
 * @struct mipi_sub_ops_s
 * sub driver ops struct of mipi csi
 * @NO{S10E03C01}
 */
typedef struct mipi_sub_ops_s {
	int32_t (*open)(int32_t index);
	int32_t (*close)(int32_t index);
	int32_t (*ioctl)(int32_t index, uint32_t cmd, mipi_ioc_arg_t arg);
	int32_t (*setcb)(int32_t index, MIPI_DROP_CB drop_cb, MIPI_INT_CB int_cb);
	int32_t (*sys)(int32_t index, int32_t type, int32_t sub, const char *name, char *buf, int32_t count);
	os_dev_t* (*osdev)(int32_t index);
} mipi_sub_ops_t;

/**
 * @struct mipi_dbg_ops_s
 * debug driver ops struct of mipi csi
 * @NO{S10E03C01}
 */
typedef struct mipi_dbg_ops_s {
	int32_t (*setup)(uint32_t mask, const mipi_sub_ops_t *ops);
	int32_t (*open)(mipi_dbg_hook_e type, int32_t index);
	int32_t (*close)(mipi_dbg_hook_e type, int32_t index);
	int32_t (*ioctl)(mipi_dbg_hook_e type, int32_t index, uint32_t cmd, mipi_ioc_arg_t arg);
	MIPI_INT_CB int_cb;
	void* (*class)(int32_t op);
} mipi_dbg_ops_t;

/**
 * @struct mipi_csi_s
 * global operation struct of mipi csi driver
 * @NO{S10E03C01}
 */
typedef struct mipi_csi_s {
	struct mipi_csi_device_s csi[MIPI_CSI_RXNUM];
	struct mipi_csi_device_s *tx[MIPI_CSI_TXNUM];
	int32_t rx_num;
	uint32_t rx_mask;
	int32_t tx_num;
	uint32_t tx_mask;
	const struct mipi_sub_ops_s *rx_ops;
	const struct mipi_sub_ops_s *tx_ops;
	const struct mipi_dbg_ops_s *rx_dbg_ops;
	const struct mipi_dbg_ops_s *tx_dbg_ops;
} mipi_csi_t;

extern int32_t hobot_mipi_host_setup(const struct mipi_sub_ops_s **rx_ops);
extern int32_t hobot_mipi_dev_setup(const struct mipi_sub_ops_s **tx_ops);

extern void* hobot_mipi_csi_debug_class(void);
extern int32_t hobot_mipi_csi_debug_setup(const mipi_dbg_ops_t *rx_dbg_ops, const mipi_dbg_ops_t *tx_dbg_ops);

#endif // __HOBOT_MIPI_CSI_H__
