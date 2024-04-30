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
 * @file hobot_mipi_host_ops.h
 *
 * @NO{S10E03C01}
 * @ASIL{B}
 */

#ifndef __HOBOT_MIPI_HOST_OPS_H__
#define __HOBOT_MIPI_HOST_OPS_H__ /* PRQA S 0603 */ /* header file macro */

#ifdef CONFIG_HOBOT_MIPI_HOST_SNRCLK
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#endif

#include "hobot_mipi_osal.h"

#include "hobot_mipi_host.h"
#include "hobot_mipi_utils.h"

#ifdef MODULE
#define CONFIG_HOBOT_MIPI_PHY
#ifndef CONFIG_ARCH_HOBOT
#define EX_MODULE
#endif
#endif

#ifdef CONFIG_HOBOT_FUSA_DIAG
#include "hobot_mipi_csi_stl.h"
#define MIPI_HOST_OV_IGNORE_S2US		(1000000UL)
#define MIPI_HOST_OV_IGNORE_DFT_FRAME	(1U)
#define MIPI_HOST_OV_IGNORE_DFT_FPS		(30U)
#define MIPI_HOST_OV_IGNORE_MIN_US		(5000U)
static inline uint32_t MIPI_HOST_OV_IGNORE_US(uint16_t fps, uint32_t frame) {
	uint16_t f = (fps == 0U) ? MIPI_HOST_OV_IGNORE_DFT_FPS : fps;
	return (uint32_t)(((MIPI_HOST_OV_IGNORE_S2US / f) * frame) + /* qacfix: conversion */
			MIPI_HOST_OV_IGNORE_MIN_US);
}
#define MIPI_HOST_ST_INVALID_S2US			(1000000UL)
#define MIPI_HOST_ST_INVALID_DFT_FRAME	(4U)
#define MIPI_HOST_ST_INVALID_DFT_FPS		(30U)
#define MIPI_HOST_ST_INVALID_MIN_US		(0U)
static inline uint32_t MIPI_HOST_ST_INVALID_US(uint16_t fps, uint32_t frame) {
	uint16_t f = (fps == 0U) ? MIPI_HOST_ST_INVALID_DFT_FPS : fps;
	return (uint32_t)(((MIPI_HOST_ST_INVALID_S2US / f) * frame) + /* qacfix: conversion */
			MIPI_HOST_ST_INVALID_MIN_US);
}
#define MIPI_HOST_IPIRST_INV_DFT_FRAME	(1U)
#endif

#if (defined CONFIG_HOBOT_J5 || defined CONFIG_HOBOT_FPGA_J5)
#define MIPI_HOST_J5_DROP
#define MIPI_HOST_J5_CIMDMA_OPTIMIZE
static inline int32_t MIPI_HOST_PORT_FOR_CIMDMA(int32_t port) {
	return ((port <= 1) ? 0 : 1);
}
#define MIPI_HOST_IRQ_CPUMASK (7)
#endif

/* only j5 with diag support error diag */
#if defined CONFIG_HOBOT_FUSA_DIAG && (defined CONFIG_HOBOT_J5 || defined CONFIG_HOBOT_FPGA_J5)
#define MIPI_HOST_ERR_DIAG_EVENT
#endif

/* host driver config */
#define MIPI_HOST_DNAME		    "mipi_host"
#define MIPI_HOST_CFGCLK_MHZ	24000000UL
#define MIPI_HOST_REFCLK_MHZ	24000000UL
#ifdef X5_CHIP
#define MIPI_HOST_IPICLK_DEFAULT 100000000UL
#else
#define MIPI_HOST_IPICLK_DEFAULT 650000000UL
#endif
#define MIPI_HOST_FSYNC_TYPE_DEFULT 2U
#define MIPI_HOST_CLOCK_NONE	"none"
#define MIPI_HOST_HW_MODE_DEF	(0)
#define MIPI_HOST_HW_IPI_MAX	(4)
#define MIPI_HOST_HW_VC_MAX		(4)

#ifdef X5_CHIP
#define MIPI_HOST_REG_ADDR (0x3d060000)
#define MIPI_HOST_REG_SIZE (0x10000)
#else
#ifdef EX_MODULE
/* driver as ko without dts platform */
#ifdef CONFIG_ARCH_HOBOT
#define MIPI_HOST_REG_ADDR (0xA4350000)
#else
#define MIPI_HOST_REG_ADDR (0xA0100000)
#endif
#define MIPI_HOST_REG_SIZE (0x00000800)
#endif
#endif

/* only hobot platform driver use irq */
#if defined EX_MODULE
#define MIPI_HOST_INT_USE_TIMER
#endif

/* defbug func macro */
#define MIPI_HOST_INT_DBG		   (1)
#define MIPI_HOST_INT_DBG_ERRSTR   (1)
#define MIPI_HOST_INT_DBG_ERRBIT   (32)
#define MIPI_HOST_INT_DBG_ERRBUF   (256)
#define MIPI_HOST_SYSFS_FATAL_EN   (1)

/* reg value macro */
/* INT for 1p3 */
#define MIPI_HOST_INT_PHY_FATAL    ((uint32_t)(0x1UL << 0))
#define MIPI_HOST_INT_PKT_FATAL    ((uint32_t)(0x1UL << 1))
#define MIPI_HOST_INT_FRM_FATAL    ((uint32_t)(0x1UL << 2))
#define MIPI_HOST_INT_PHY          ((uint32_t)(0x1UL << 16))
#define MIPI_HOST_INT_PKT          ((uint32_t)(0x1UL << 17))
#define MIPI_HOST_INT_LINE         ((uint32_t)(0x1UL << 18))
#define MIPI_HOST_INT_IPI          ((uint32_t)(0x1UL << 19))
#define MIPI_HOST_INT_IPI2         ((uint32_t)(0x1UL << 20))
#define MIPI_HOST_INT_IPI3         ((uint32_t)(0x1UL << 21))
#define MIPI_HOST_INT_IPI4         ((uint32_t)(0x1UL << 22))

/* INT for 1p4 */
#define MIPI_HOST_1P4_INT_PHY_FATAL       ((uint32_t)(0x1UL << 0))
#define MIPI_HOST_1P4_INT_PKT_FATAL       ((uint32_t)(0x1UL << 1))
#define MIPI_HOST_1P4_INT_BNDRY_FRM_FATAL ((uint32_t)(0x1UL << 2))
#define MIPI_HOST_1P4_INT_SEQ_FRM_FATAL   ((uint32_t)(0x1UL << 3))
#define MIPI_HOST_1P4_INT_CRC_FRM_FATAL   ((uint32_t)(0x1UL << 4))
#define MIPI_HOST_1P4_INT_PLD_CRC_FATAL   ((uint32_t)(0x1UL << 5))
#define MIPI_HOST_1P4_INT_DATA_ID         ((uint32_t)(0x1UL << 6))
#define MIPI_HOST_1P4_INT_ECC_CORRECTED   ((uint32_t)(0x1UL << 7))
#define MIPI_HOST_1P4_INT_PHY             ((uint32_t)(0x1UL << 16))
#define MIPI_HOST_1P4_INT_LINE            ((uint32_t)(0x1UL << 17))
#define MIPI_HOST_1P4_INT_IPI             ((uint32_t)(0x1UL << 18))
#define MIPI_HOST_1P4_INT_IPI2            ((uint32_t)(0x1UL << 19))
#define MIPI_HOST_1P4_INT_IPI3            ((uint32_t)(0x1UL << 20))
#define MIPI_HOST_1P4_INT_IPI4            ((uint32_t)(0x1UL << 21))

#define MIPI_HOST_1P4AP_INT_ST_AP_GENERIC    ((uint32_t)(0x1UL << 0))
#define MIPI_HOST_1P4AP_INT_PHY_FATAL        ((uint32_t)(0x1UL << 1))
#define MIPI_HOST_1P4AP_INT_PKT_FATAL        ((uint32_t)(0x1UL << 2))
#define MIPI_HOST_1P4AP_INT_BNDRY_FRM_FATAL  ((uint32_t)(0x1UL << 3))
#define MIPI_HOST_1P4AP_INT_SEQ_FRM_FATAL    ((uint32_t)(0x1UL << 4))
#define MIPI_HOST_1P4AP_INT_CRC_FRM_FATAL    ((uint32_t)(0x1UL << 5))
#define MIPI_HOST_1P4AP_INT_PHY              ((uint32_t)(0x1UL << 6))
#define MIPI_HOST_1P4AP_INT_PLD_CRC_FATAL    ((uint32_t)(0x1UL << 7))
#define MIPI_HOST_1P4AP_INT_DATA_ID          ((uint32_t)(0x1UL << 8))
#define MIPI_HOST_1P4AP_INT_ECC_CORRECTED    ((uint32_t)(0x1UL << 9))
#define MIPI_HOST_1P4AP_INT_LINE             ((uint32_t)(0x1UL << 10))
#define MIPI_HOST_1P4AP_INT_ST_AP_IPI        ((uint32_t)(0x1UL << 11))
#define MIPI_HOST_1P4AP_INT_IPI              ((uint32_t)(0x1UL << 12))
#define MIPI_HOST_1P4AP_INT_ST_AP_IPI2       ((uint32_t)(0x1UL << 13))
#define MIPI_HOST_1P4AP_INT_IPI2             ((uint32_t)(0x1UL << 14))
#define MIPI_HOST_1P4AP_INT_ST_AP_IPI3       ((uint32_t)(0x1UL << 15))
#define MIPI_HOST_1P4AP_INT_IPI3             ((uint32_t)(0x1UL << 16))
#define MIPI_HOST_1P4AP_INT_ST_AP_IPI4       ((uint32_t)(0x1UL << 17))
#define MIPI_HOST_1P4AP_INT_IPI4             ((uint32_t)(0x1UL << 18))

#define MIPI_HOST_1P4AP_INT_APB_APERR_MASK   ((uint32_t)(0x3UL << 0))

/* INT for 1p5 */
#define MIPI_HOST_1P5_INT_PHY_FATAL       ((uint32_t)(0x1UL << 0))
#define MIPI_HOST_1P5_INT_PKT_FATAL       ((uint32_t)(0x1UL << 1))
#define MIPI_HOST_1P5_INT_BNDRY_FRM_FATAL ((uint32_t)(0x1UL << 2))
#define MIPI_HOST_1P5_INT_SEQ_FRM_FATAL   ((uint32_t)(0x1UL << 3))
#define MIPI_HOST_1P5_INT_CRC_FRM_FATAL   ((uint32_t)(0x1UL << 4))
#define MIPI_HOST_1P5_INT_PLD_CRC_FATAL   ((uint32_t)(0x1UL << 5))
#define MIPI_HOST_1P5_INT_DATA_ID         ((uint32_t)(0x1UL << 6))
#define MIPI_HOST_1P5_INT_ECC_CORRECTED   ((uint32_t)(0x1UL << 7))
#define MIPI_HOST_1P5_INT_PHY             ((uint32_t)(0x1UL << 16))
#define MIPI_HOST_1P5_INT_LINE            ((uint32_t)(0x1UL << 17))
#define MIPI_HOST_1P5_INT_IPI             ((uint32_t)(0x1UL << 18))
#define MIPI_HOST_1P5_INT_IPI2            ((uint32_t)(0x1UL << 19))
#define MIPI_HOST_1P5_INT_IPI3            ((uint32_t)(0x1UL << 20))
#define MIPI_HOST_1P5_INT_IPI4            ((uint32_t)(0x1UL << 21))
#define MIPI_HOST_1P5_INT_IPI5            ((uint32_t)(0x1UL << 22))
#define MIPI_HOST_1P5_INT_IPI6            ((uint32_t)(0x1UL << 23))
#define MIPI_HOST_1P5_INT_IPI7            ((uint32_t)(0x1UL << 24))
#define MIPI_HOST_1P5_INT_IPI8            ((uint32_t)(0x1UL << 25))

#define MIPI_HOST_1P5AP_INT_ST_AP_GENERIC    ((uint32_t)(0x1UL << 0))
#define MIPI_HOST_1P5AP_INT_PHY_FATAL        ((uint32_t)(0x1UL << 1))
#define MIPI_HOST_1P5AP_INT_PKT_FATAL        ((uint32_t)(0x1UL << 2))
#define MIPI_HOST_1P5AP_INT_BNDRY_FRM_FATAL  ((uint32_t)(0x1UL << 3))
#define MIPI_HOST_1P5AP_INT_SEQ_FRM_FATAL    ((uint32_t)(0x1UL << 4))
#define MIPI_HOST_1P5AP_INT_CRC_FRM_FATAL    ((uint32_t)(0x1UL << 5))
#define MIPI_HOST_1P5AP_INT_PHY              ((uint32_t)(0x1UL << 6))
#define MIPI_HOST_1P5AP_INT_PLD_CRC_FATAL    ((uint32_t)(0x1UL << 7))
#define MIPI_HOST_1P5AP_INT_DATA_ID          ((uint32_t)(0x1UL << 8))
#define MIPI_HOST_1P5AP_INT_ECC_CORRECTED    ((uint32_t)(0x1UL << 9))
#define MIPI_HOST_1P5AP_INT_LINE             ((uint32_t)(0x1UL << 10))
#define MIPI_HOST_1P5AP_INT_ST_AP_IPI        ((uint32_t)(0x1UL << 11))
#define MIPI_HOST_1P5AP_INT_IPI              ((uint32_t)(0x1UL << 12))
#define MIPI_HOST_1P5AP_INT_ST_AP_IPI2       ((uint32_t)(0x1UL << 13))
#define MIPI_HOST_1P5AP_INT_IPI2             ((uint32_t)(0x1UL << 14))
#define MIPI_HOST_1P5AP_INT_ST_AP_IPI3       ((uint32_t)(0x1UL << 15))
#define MIPI_HOST_1P5AP_INT_IPI3             ((uint32_t)(0x1UL << 16))
#define MIPI_HOST_1P5AP_INT_ST_AP_IPI4       ((uint32_t)(0x1UL << 17))
#define MIPI_HOST_1P5AP_INT_IPI4             ((uint32_t)(0x1UL << 18))
#define MIPI_HOST_1P5AP_INT_ST_AP_IPI5       ((uint32_t)(0x1UL << 19))
#define MIPI_HOST_1P5AP_INT_IPI5             ((uint32_t)(0x1UL << 20))
#define MIPI_HOST_1P5AP_INT_ST_AP_IPI6       ((uint32_t)(0x1UL << 21))
#define MIPI_HOST_1P5AP_INT_IPI6             ((uint32_t)(0x1UL << 22))
#define MIPI_HOST_1P5AP_INT_ST_AP_IPI7       ((uint32_t)(0x1UL << 23))
#define MIPI_HOST_1P5AP_INT_IPI7             ((uint32_t)(0x1UL << 24))
#define MIPI_HOST_1P5AP_INT_ST_AP_IPI8       ((uint32_t)(0x1UL << 25))
#define MIPI_HOST_1P5AP_INT_IPI8             ((uint32_t)(0x1UL << 26))
#define MIPI_HOST_1P5AP_INT_ST_LOGGER_ERR    ((uint32_t)(0x1UL << 27))

#define MIPI_HOST_CSI2_RAISE	   (0x01)
#define MIPI_HOST_CSI2_RESETN	   (0x00)
#define MIPI_HOST_PHY_MODE_DPHY    (0x00)
#define MIPI_HOST_PHY_MODE_CPHY    (0x01)
#define MIPI_HOST_PPI_WIDTH_8BIT   (0x00)
#define MIPI_HOST_PPI_WIDTH_16BIT  (0x01)
#define MIPI_HOST_PPI_PG_DISABLE   (0x00)
#define MIPI_HOST_PPI_PG_ENABLE    (0x01)
#define MIPI_HOST_MEMFLUSN_ENABLE  ((uint32_t)(0x01UL << 8))
#define MIPI_HOST_MEMFLUSN_MANUAL  ((uint32_t)(0x01UL << 0))
#define MIPI_HOST_IPI_DT_MASK	   (0x3fUL)
#define MIPI_HOST_EMB_DATA		   ((uint32_t)(0x01UL << 8))
#define MIPI_HOST_BITWIDTH_48	   ((uint32_t)(0x00UL << 8))
#define MIPI_HOST_BITWIDTH_16	   ((uint32_t)(0x01UL << 8))
#define MIPI_HOST_CUT_THROUGH	   ((uint32_t)(0x01UL << 16))
#define MIPI_HOST_IPI_ENABLE	   ((uint32_t)(0x01UL << 24))
#define MIPI_HOST_IPI_DISABLE	   (0x00U)
#define MIPI_HOST_IPI_16BIT        (16)
#define MIPI_HOST_IPI_48BIT        (48)
#define MIPI_HOST_LEGCYMODE_ENABLE ((uint32_t)(0x01UL << 24))
#define MIPI_HOST_HSATIME		   (0x04)
#define MIPI_HOST_HBPTIME		   (0x04)
#define MIPI_HOST_HSDTIME		   (0x5f4)
#define MIPI_HOST_HSDTIME_MIN	   (0x04U)
#define MIPI_HOST_HSDTIME_MAX	   (0xfffU)
#define MIPI_HOST_HSATIME_P	   (0x08)
#define MIPI_HOST_HBPTIME_P	   (0x08)
#define MIPI_HOST_HBPTIME		   (0x04)
#define MIPI_HOST_HSD_BYTE_BITS    (8U)
#define MIPI_HOST_HSD_CAL_MIN      (6U)
#define MIPI_HOST_HBPTIME_MIN	   (0x01U)
#define MIPI_HOST_HBPTIME_MAX	   (0xfffU)
#define MIPI_HOST_HSATIME_MIN	   (0x01U)
#define MIPI_HOST_HSATIME_MAX	   (0xfffU)
#define MIPI_HOST_CFGCLK_DEFAULT   (0x1C)
#define MIPI_HOST_IPI1_SOFTRSTN	   ((uint32_t)(0x01UL << 0))
#define MIPI_HOST_IPI2_SOFTRSTN	   ((uint32_t)(0x01UL << 4))
#define MIPI_HOST_IPI3_SOFTRSTN	   ((uint32_t)(0x01UL << 8))
#define MIPI_HOST_IPI4_SOFTRSTN	   ((uint32_t)(0x01UL << 12))
#define MIPI_HOST_ALLE_SOFTRSTN	   ((uint32_t)(0x1111UL))
#define MIPI_HOST_VC_EXT_LEGACY	   ((uint32_t)(0x01UL))
#define MIPI_HOST_VC_EXT_ENABLE	   ((uint32_t)(0x00UL))

#ifdef X5_CHIP
/* x5 ISP need frame_start vsync, so we must choose legacy mode for IPI ADV
*/
#define MIPI_HOST_ADV_DEFAULT     ((uint32_t)(0x100UL << 16))
#else
#define MIPI_HOST_ADV_DEFAULT      ((uint32_t)(0x3UL << 16))
#endif
#define MIPI_HOST_ADV_EN_EMB       ((uint32_t)(0x1UL << 21))
#define MIPI_HOST_ADV_RAW10_OV     (0x2B01U)
#define MIPI_HOST_ADV_RAW12_OV     (0x2C01U)
#define MIPI_HOST_ADV_DT_OFFSET    (8U)
#define MIPI_HOST_ADV_DT_OV_ENABLE (0x1U)
#define MIPI_HOST_ADV_DT_OV_MASK   (0xFF01U)
#define MIPI_HOST_ADV_EN_LINE_START (0x40000U)
#define MIPI_HOST_CUT_THROUGH_EN   (0x1U)
#define MIPI_HOST_CUT_HSD_LEGACY   (0x2U)
#define MIPI_HOST_CUT_DEFAULT      (MIPI_HOST_CUT_THROUGH_EN)
#define MIPI_HOST_MEM_DEFAULT      (1)
#define MIPI_HOST_ERROR_DIAG_DEFAULT (1)
#define MIPI_HOST_IPI_OVERST_DEFAULT (1)
#define MIPI_HOST_IPILIMIT_DEFAULT (102000000U)
#define MIPI_HOST_IPIFORCE_MIN     (10000000U)
#define MIPI_HOST_IRQ_CNT          (10)
#define MIPI_HOST_IRQ_DEBUG_PRERR  (0x1U)
#define MIPI_HOST_IRQ_DEBUG_ERRSTR (0x2U)
#define MIPI_HOST_IRQ_DEBUG        (0x1U)
#define MIPI_HOST_SNRCLK_DISABLE   (0U)
#define MIPI_HOST_SNRCLK_ENABLE    (1U)
#define MIPI_HOST_SNRCLK_NOUSED    (2U)
#define MIPI_HOST_SNRCLK_IGNORE    (24U)
#define MIPI_HOST_SNRCLK_FREQ_MIN  (6375000UL)
#define MIPI_HOST_SNRCLK_FREQ_BASE (10000UL)
#define MIPI_HOST_SNRCLK_DIFF_PERC (10U)
#define MIPI_HOST_IRQ_CNT_MAX      (0xffffffffU)
#define MIPI_HOST_FREQ_MHZ         (1000000UL)
#define MIPI_HOST_FREQ_MHZ_TO_NS   (1000UL)
#define MIPI_HOST_PIXELS_RAW       (3UL)

#define HOST_DPHY_CLK_PPI8_MAX     (2500U)
#define HOST_DPHY_LANE_MAX         (4U)
#define HOST_DPHY_CHECK_MAX        (3000)
#define HOST_DPHY_LANE_STOP_ALL    (0xFU)
static inline uint32_t HOST_DPHY_LANE_STOP(uint16_t l) {
	return ((uint32_t)HOST_DPHY_LANE_STOP_ALL >> (HOST_DPHY_LANE_MAX - (l)));
}
#define HOST_DPHY_RX_HS            (0x030000U)
#define HOST_DPHY_RX_HS_OFFS       (16)
static inline uint8_t HOST_DPHY_RX_HS_STA(uint32_t s) {
	return (uint8_t)((s & HOST_DPHY_RX_HS) >> HOST_DPHY_RX_HS_OFFS);
}

#define HOST_BITS_PER_BYTE         (8)
#define HOST_US_TO_NS              (1000)
#define HOST_S_TO_US               (1000000)
#define HOST_S_TO_NS               (1000000000)
#define HOST_DFLT_F_SYNC_TYPE      (2)
#define HOST_IPI_FIFO_DEPTH        (64)

#define MIPI_HOST_PPIPGC_VC_MASK       (0x1FU)
#define MIPI_HOST_PPIPGC_VC_OFFS       (3)
static inline uint8_t MIPI_HOST_PPIPGC_VC(uint16_t ppi_pg) {
        return ((uint8_t)((ppi_pg >> MIPI_HOST_PPIPGC_VC_OFFS) & MIPI_HOST_PPIPGC_VC_MASK));
}
#define MIPI_HOST_PPIPGC_MODE_MASK     (0x1U)
#define MIPI_HOST_PPIPGC_MODE_OFFS     (2)
static inline uint8_t MIPI_HOST_PPIPGC_MODE(uint16_t ppi_pg) {
        return ((uint8_t)((ppi_pg >> MIPI_HOST_PPIPGC_MODE_OFFS) & MIPI_HOST_PPIPGC_MODE_MASK));
}
#define MIPI_HOST_PPIPGC_TYPE_MASK     (0x1U)
#define MIPI_HOST_PPIPGC_TYPE_OFFS     (1)
static inline uint8_t MIPI_HOST_PPIPGC_TYPE(uint16_t ppi_pg) {
        return ((uint8_t)((ppi_pg >> MIPI_HOST_PPIPGC_TYPE_OFFS) & MIPI_HOST_PPIPGC_TYPE_MASK));
}

static inline uint32_t MIPI_HOST_PPIPG_HRES(uint32_t pkt2pkt, uint16_t hres) {
	return (hres | ((pkt2pkt & 0x3FFU) << 16U));
}
static inline uint32_t MIPI_HOST_PPIPG_CONFIG(uint32_t pattern, uint32_t datatype, uint32_t vc) {
	return ((pattern & 0x1U) | ((datatype & 0x3FU) << 8U) | ((vc & 0x1FU) << 14U));
}

#define MIPI_IPI16_YUV_CYCLE    (2)
#define MIPI_IPI48_YUV_CYCLE    (1)
#define MIPI_IPI16_RAW_PIXEL    (1)
#define MIPI_IPI48_RAW_PIXEL    (3)
#define MIPI_IPI_RAW_CYCLEALIGN (2U)

/* ipi info macro */
#define HOST_IPI_INFO_B_SETV    ((uint32_t)(0x1UL << 15))
#define HOST_IPI_INFO_B_ALL     ((uint32_t)(0xffffUL))
enum {
	HOST_IPI_INFO_MODE = 0,
	HOST_IPI_INFO_VC,
	HOST_IPI_INFO_DT,
	HOST_IPI_INFO_MEM,
	HOST_IPI_INFO_HSA,
	HOST_IPI_INFO_HBP,
	HOST_IPI_INFO_HSD,
	HOST_IPI_INFO_ADV,
	HOST_IPI_INFO_FATAL,
	HOST_IPI_INFO_SOFTRSTN,
	HOST_IPI_INFO_NUM,
};

#define MIPI_HOST_IPI_INFO_REGS { \
	{ \
		REG_MIPI_HOST_IPI_MODE, \
		REG_MIPI_HOST_IPI_VCID, \
		REG_MIPI_HOST_IPI_DATA_TYPE, \
		REG_MIPI_HOST_IPI_MEM_FLUSH, \
		REG_MIPI_HOST_IPI_HSA_TIME, \
		REG_MIPI_HOST_IPI_HBP_TIME, \
		REG_MIPI_HOST_IPI_HSD_TIME, \
		REG_MIPI_HOST_IPI_ADV_FEATURES, \
		REG_MIPI_HOST_INT_ST_IPI, \
		MIPI_HOST_IPI1_SOFTRSTN, \
	}, \
	{ \
		REG_MIPI_HOST_IPI2_MODE, \
		REG_MIPI_HOST_IPI2_VCID, \
		REG_MIPI_HOST_IPI2_DATA_TYPE, \
		REG_MIPI_HOST_IPI2_MEM_FLUSH, \
		REG_MIPI_HOST_IPI2_HSA_TIME, \
		REG_MIPI_HOST_IPI2_HBP_TIME, \
		REG_MIPI_HOST_IPI2_HSD_TIME, \
		REG_MIPI_HOST_IPI2_ADV_FEATURES, \
		REG_MIPI_HOST_INT_ST_IPI2, \
		MIPI_HOST_IPI2_SOFTRSTN, \
	}, \
	{ \
		REG_MIPI_HOST_IPI3_MODE, \
		REG_MIPI_HOST_IPI3_VCID, \
		REG_MIPI_HOST_IPI3_DATA_TYPE, \
		REG_MIPI_HOST_IPI3_MEM_FLUSH, \
		REG_MIPI_HOST_IPI3_HSA_TIME, \
		REG_MIPI_HOST_IPI3_HBP_TIME, \
		REG_MIPI_HOST_IPI3_HSD_TIME, \
		REG_MIPI_HOST_IPI3_ADV_FEATURES, \
		REG_MIPI_HOST_INT_ST_IPI3, \
		MIPI_HOST_IPI3_SOFTRSTN, \
	}, \
	{ \
		REG_MIPI_HOST_IPI4_MODE, \
		REG_MIPI_HOST_IPI4_VCID, \
		REG_MIPI_HOST_IPI4_DATA_TYPE, \
		REG_MIPI_HOST_IPI4_MEM_FLUSH, \
		REG_MIPI_HOST_IPI4_HSA_TIME, \
		REG_MIPI_HOST_IPI4_HBP_TIME, \
		REG_MIPI_HOST_IPI4_HSD_TIME, \
		REG_MIPI_HOST_IPI4_ADV_FEATURES, \
		REG_MIPI_HOST_INT_ST_IPI4, \
		MIPI_HOST_IPI4_SOFTRSTN, \
	}, \
}

/* mipi host error diag event desp */
#define EventIdSwMipiHostErrEventBase		(33u)
#ifdef MIPI_HOST_ERR_DIAG_EVENT
enum {
	EventIdSwMipiHostProbeErr = EventIdSwMipiHostErrEventBase,
	EventIdSwMipiHostIocCheckErr,
	EventIdSwMipiHostIocUserErr,
	EventIdSwMipiHostParamCheckErr,
	EventIdSwMipiHostMutexLockErr,
	EventIdSwMipiHostStatusErr,
	EventIdSwMipiHostIomemErr,
	EventIdSwMipiHostHwConfigErr,
	EventIdSwMipiHostDphyOpErr,
	EventIdSwMipiHostDphyStateErr,
	EventIdSwMipiHostIpiOpErr,
	EventIdSwMipiHostSnrclkSetErr,
};
struct mipi_host_err_event_desp {
	uint8_t                 id;
	uint8_t                 prio;
	const char              *name;
};
#define MIPI_HOST_ERR_EVENT_D(n, e, p) { \
	.id = (uint8_t)(e), \
	.prio = (uint8_t)(p), \
	.name = (n), \
}
#define FCHM_ERR_CODE_SW (0xFFFFu)
#define LOW_8_BIT_MASK   (0x00FFu)
#define SHIFT_8			 (8u)
#endif
enum {
	ESW_MipiHostProbeErr = 0,
	ESW_MipiHostIocCheckErr,
	ESW_MipiHostIocUserErr,
	ESW_MipiHostParamCheckErr,
	ESW_MipiHostMutexLockErr,
	ESW_MipiHostStatusErr,
	ESW_MipiHostIomemErr,
	ESW_MipiHostHwConfigErr,
	ESW_MipiHostDphyOpErr,
	ESW_MipiHostDphyStateErr,
	ESW_MipiHostIpiOpErr,
	ESW_MipiHostSnrclkSetErr,
	ESW_MipiHostErrEventMax,
};
enum {
	SUB_ID_0,
	SUB_ID_1,
	SUB_ID_2,
	SUB_ID_3,
	SUB_ID_4,
	SUB_ID_5,
	SUB_ID_6,
	SUB_ID_7,
	SUB_ID_8,
	SUB_ID_9,
	SUB_ID_10,
	SUB_ID_11,
	SUB_ID_12,
	SUB_ID_13,
	SUB_ID_14,
/* not used
	SUB_ID_15,
*/
};

/* host pre-op state */
enum {
	MIPI_PRE_STATE_DEFAULT = 0,
	MIPI_PRE_STATE_INITING,
	MIPI_PRE_STATE_INITED,
	MIPI_PRE_STATE_STARTING,
	MIPI_PRE_STATE_STARTED,
	MIPI_PRE_STATE_MAX,
};
#define MIPI_PRE_STSTE_STRINGS { \
	"default", \
	"initing", \
	"inited", \
	"starting", \
	"started", \
}

/* host soc clock name struct */
struct mipi_host_socclk_s {
	const char *cfgclk;
	const char *refclk;
	const char *snrclk;
	const char *pclk;
	const char *ipiclk[MIPI_HOST_HW_IPI_MAX];
};

#ifdef CONFIG_HOBOT_MIPI_HOST_SNRCLK
/* host sensor mclk clock struct */
typedef int32_t (*mipi_pin_state_func)(void*, void *);
struct mipi_host_snrclk_s {
	int32_t index;
	uint32_t probe_init;
	uint32_t probe_freq;
	uint32_t reserved;
	struct pinctrl *pinctrl;
	struct pinctrl_state *enable;
	struct pinctrl_state *disable;
	mipi_pin_state_func pin_state;
};
#endif

/* debug loglevel by param->dbg_value */
#define mipi_dbg(param, dev, fmt, ...) do { \
		if (param->dbg_value != 0U) \
			mipi_debug(dev, fmt, ##__VA_ARGS__); \
	} while(0)

/* debug param set&save */
static inline void mipi_host_param_dbg_set_save(struct mipi_host_param_s *param,
		uint32_t *irq_debug, uint32_t *irq_cnt, uint32_t *dbg_value) {
		uint32_t irq_debug_save = param->irq_debug;
		uint32_t irq_cnt_save = param->irq_cnt;
		uint32_t dbg_value_save = param->dbg_value;

		param->irq_debug = *irq_debug;
		param->irq_cnt = *irq_cnt;
		param->dbg_value = *dbg_value;

		*irq_debug = irq_debug_save;
		*irq_cnt = irq_cnt_save;
		*dbg_value = dbg_value_save;
}

#if MIPI_HOST_INT_DBG
/* interrupt error counts */
struct mipi_host_icnt_s {
	/* type must be: uint32_t */
	uint32_t st_main;
	uint32_t phy_fatal;
	uint32_t pkt_fatal;
	uint32_t frm_fatal;
	uint32_t bndry_frm_fatal;
	uint32_t seq_frm_fatal;
	uint32_t crc_frm_fatal;
	uint32_t pld_crc_fatal;
	uint32_t data_id;
	uint32_t ecc_corrected;
	uint32_t phy;
	uint32_t pkt;
	uint32_t line;
	uint32_t ipi;
	uint32_t ipi2;
	uint32_t ipi3;
	uint32_t ipi4;
	uint32_t ipi5;
	uint32_t ipi6;
	uint32_t ipi7;
	uint32_t ipi8;
	uint32_t ap_generic;
	uint32_t ap_ipi;
	uint32_t ap_ipi2;
	uint32_t ap_ipi3;
	uint32_t ap_ipi4;
	uint32_t ap_ipi5;
	uint32_t ap_ipi6;
	uint32_t ap_ipi7;
	uint32_t ap_ipi8;
	uint32_t logger_err;
};
#define MIPI_HOST_ICNT_NUM (sizeof(struct mipi_host_icnt_s)/sizeof(uint32_t))
#define MIPI_HOST_ICNT_IPI	    (13U)
#define MIPI_HOST_ICNT_IPIE     (20U)
#define MIPI_HOST_ICNT_IPI_MASK	((uint32_t)(0xFFUL << 13))
#define MIPI_HOST_ICNT_APIPI	(22U)
#define MIPI_HOST_ICNT_APIPIE	(29U)
/* interrupt error counts of sysfs, see: struct mipi_host_icnt_s */
#define MIPI_HOST_ICNT_STRINGS { \
	"st_main", \
	"phy_fatal", \
	"pkt_fatal", \
	"frm_fatal", \
	"bndry_frm_fatal", \
	"seq_frm_fatal", \
	"crc_frm_fatal", \
	"pld_crc_fatal", \
	"data_id", \
	"ecc_corrected", \
	"phy", \
	"pkt", \
	"line", \
	"ipi", \
	"ipi2", \
	"ipi3", \
	"ipi4", \
	"ipi5", \
	"ipi6", \
	"ipi7", \
	"ipi8", \
	"ap_generic", \
	"ap_ipi", \
	"ap_ipi2", \
	"ap_ipi3", \
	"ap_ipi4", \
	"ap_ipi5", \
	"ap_ipi6", \
	"ap_ipi7", \
	"ap_ipi8", \
	"logger_err", \
}

/* host interrupt reg desp struct */
struct mipi_host_ireg_s {
	uint32_t icnt_n;
	uint32_t st_mask;
	uint32_t reg_st;
	uint32_t reg_mask;
	uint32_t reg_force;
	uint32_t err_mask;
#if MIPI_HOST_INT_DBG_ERRSTR
	const char* err_str[MIPI_HOST_INT_DBG_ERRBIT];
#endif
};

/* drop frame depend on icnt */
#define MIPI_HOST_PARAM_DROP_IN_STL		(0x0U)
#define MIPI_HOST_PARAM_DROP_IN_IRQ		(0x1U)

#define MIPI_HOST_DROP_IPI_1		(0x1U)
#define MIPI_HOST_DROP_IPI_2		(0x2U)
#define MIPI_HOST_DROP_IPI_3		(0x4U)
#define MIPI_HOST_DROP_IPI_4		(0x8U)
#define MIPI_HOST_DROP_IPI_5		(0x10U)
#define MIPI_HOST_DROP_IPI_6		(0x20U)
#define MIPI_HOST_DROP_IPI_7		(0x40U)
#define MIPI_HOST_DROP_IPI_8		(0x80U)
#define MIPI_HOST_DROP_IPI_ALL		(0xFFU)
#define MIPI_HOST_DROP_MVC_OFFSET	(16)
#define MIPI_HOST_DROP_MVC_MASK		(0xFFFFU)

#if 0
typedef struct mipi_host_s mipi_host_t;
struct mipi_host_icnt_drop {
	uint32_t icnt;
	uint32_t ipi_default;
	uint32_t (*mipi_host_icnt2ipi)(mipi_host_t*, uint32_t);
};
uint32_t mipi_host_drop_vc2ipi(mipi_host_t *host, uint32_t vc_mask);
uint32_t mipi_host_drop_mvc2ipi(mipi_host_t *host, uint32_t vc_mask);
#endif

/* host interrupt error struct */
struct mipi_host_ierr_s {
	const struct mipi_host_ireg_s *iregs;
	uint32_t num;
	uint32_t st_main;
};
#endif

/* host hardware lane mode */
enum {
	MIPIHOST_LANEMODE_ALONE = 0,
	MIPIHOST_LANEMODE_GROUP,
	MIPIHOST_LANEMODE_MAX,
};

/* host hardware mode(lane/grout/ipi) desp */
struct mipi_host_port_hw_s {
	uint16_t group;
	uint16_t index;
	uint16_t lane_alone;
	uint16_t lane_group;
	uint16_t ipi;
};

/* host hardware mode struct */
struct mipi_host_port_hw_mode_s {
	const struct mipi_host_port_hw_s *unit;
	uint16_t unum;
	uint16_t ugrp;
};

/* mipi host info struct */
struct mipi_host_s {
	void __iomem             *iomem;
	struct mipi_reg_s         reg;
	int32_t                   irq;
	int32_t                   ap;
	int32_t                   hw;
	enum mipi_state_e         state;
	mipi_host_cfg_t           cfg;
	struct mipi_host_param_s  param;
#ifdef CONFIG_HOBOT_MIPI_HOST_SNRCLK
	struct mipi_host_snrclk_s snrclk;
#endif
	struct mipi_host_socclk_s socclk;
#if MIPI_HOST_INT_DBG
	struct mipi_host_ierr_s   ierr;
	struct mipi_host_icnt_s   icnt;
	struct mipi_host_icnt_drop *drops;
#endif
#ifdef CONFIG_HOBOT_FUSA_DIAG
	struct mipi_host_ierr_s   ieap;
	struct mipi_csi_stl_priv  stl;
#endif
};

/* mipi host user struct */
struct mipi_user_s {
    /*
	 * mutex: user.open_mutex
	 * protect: user.open_cnt and operations when first open and last close.
	 * init: probe, see: hobot_mipi_host_probe_cdev.
	 * call: open/close, see: hobot_mipi_host_open, hobot_mipi_host_close.
	 */
	osal_mutex_t      open_mutex;
	/*
	 * mutex: user.mutex
	 * protect: user.init_cnt user.start_cnt user.pre_state and operations of mipi host.
	 * init: first open, see hobot_mipi_host_open.
	 * call: ioctl, see hobot_mipi_host_ioctl.
	*/
	osal_mutex_t      mutex;
	uint32_t          open_cnt;
	uint32_t          init_cnt;
	uint32_t          start_cnt;
	uint32_t          pre_state;
	bool              pre_done;
	osal_waitqueue_t pre_wq;
};

/* mipi host device struct */
struct mipi_hdev_s {
	int32_t            port;
	int32_t            lane_mode;
	struct os_dev      osdev;
	void              *ex_hdev;
	int32_t            is_ex;
	uint64_t           ipi_clock;
	struct mipi_host_s host;
	struct mipi_user_s user;
	struct mipi_cb_s   cb;
	const struct mipi_host_port_hw_mode_s *hw_mode;
#if MIPI_HOST_INT_DBG && defined MIPI_HOST_INT_USE_TIMER
	osal_timer_t       irq_timer;
	uint32_t           irq_timer_en;
	uint32_t           irq_st_main;
#ifdef CONFIG_ARCH_ZYNQMP
	uint32_t           fatal_ap_generic;
#endif
#endif
};

enum {
	MIPI_HOST_SYS_PARAM,
	MIPI_HOST_SYS_STATUS_CLOCK,
	MIPI_HOST_SYS_STATUS_INFO,
	MIPI_HOST_SYS_STATUS_CFG,
	MIPI_HOST_SYS_STATUS_REGS,
	MIPI_HOST_SYS_STATUS_SNRCLK,
	MIPI_HOST_SYS_STATUS_USER,
	MIPI_HOST_SYS_STATUS_ICNT,
	MIPI_HOST_SYS_FATAL,
	MIPI_HOST_SYS_FAULT_INJECT,
	MIPI_HOST_SYS_NUM,
};

extern void hobot_mipi_host_irq_func_do(struct mipi_hdev_s *hdev);
extern int32_t hobot_mipi_host_open_do(struct mipi_hdev_s *hdev);
extern int32_t hobot_mipi_host_close_do(struct mipi_hdev_s *hdev);
extern mipi_ioc_ret_t hobot_mipi_host_ioctl_do(struct mipi_hdev_s *hdev, uint32_t cmd, mipi_ioc_arg_t arg);
extern int32_t hobot_mipi_host_sys_do(int32_t type, int32_t sub,
		struct mipi_hdev_s *hdev, const char *name, char *buf, int32_t count);

extern int32_t hobot_mipi_host_suspend_do(struct mipi_hdev_s *hdev);
extern int32_t hobot_mipi_host_resume_do(struct mipi_hdev_s *hdev);

extern int32_t hobot_mipi_host_probe_do(struct mipi_hdev_s *hdev);
extern void hobot_mipi_host_remove_do(struct mipi_hdev_s *hdev);
extern int32_t hobot_mipi_host_setcb_do(struct mipi_hdev_s *hdev, MIPI_DROP_CB drop_cb, MIPI_INT_CB int_cb);
extern struct mipi_hdev_s *hobot_mipi_host_hdev(int32_t port);

#ifdef CONFIG_HOBOT_FUSA_DIAG
extern int32_t hobot_mipi_host_stl_setup_do(int32_t port,
		MIPI_CSI_STL_SETUP setup, const struct mipi_csi_stl_call *call);
#endif

#endif //__HOBOT_MIPI_HOST_OPS_H__

