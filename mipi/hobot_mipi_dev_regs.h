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

#ifndef __HOBOT_MIPI_DEV_REGS_H__
#define __HOBOT_MIPI_DEV_REGS_H__ /* PRQA S 0603 */ /* header file macro */

/*************************************************
*  MIPI device register offset
**************************************************/
#define REG_MIPI_DEV_VERSION                (0x00)
#define REG_MIPI_DEV_CSI2_RESETN            (0x04)
#define REG_MIPI_DEV_INT_ST_MAIN            (0x20)
#define REG_MIPI_DEV_INT_ST_VPG             (0x24)
#define REG_MIPI_DEV_INT_ST_IDI             (0x28)
#define REG_MIPI_DEV_INT_ST_IPI             (0x2C)
#define REG_MIPI_DEV_INT_ST_PHY             (0x30)
#define REG_MIPI_DEV_INT_ST_IDI_VCX         (0x34)
#define REG_MIPI_DEV_INT_ST_IDI_VCX2        (0x38)
#define REG_MIPI_DEV_INT_ST_MT_IPI          (0x3C)
#define REG_MIPI_DEV_INT_MASK_N_VPG         (0x40)
#define REG_MIPI_DEV_INT_FORCE_VPG          (0x44)
#define REG_MIPI_DEV_INT_MASK_N_IDI         (0x48)
#define REG_MIPI_DEV_INT_FORCE_IDI          (0x4C)
#define REG_MIPI_DEV_INT_MASK_N_IPI         (0x50)
#define REG_MIPI_DEV_INT_FORCE_IPI          (0x54)
#define REG_MIPI_DEV_INT_MASK_N_PHY         (0x58)
#define REG_MIPI_DEV_INT_FORCE_PHY          (0x5C)
#define REG_MIPI_DEV_INT_MASK_N_IDI_VCX     (0x60)
#define REG_MIPI_DEV_INT_FORCE_IDI_VCX      (0x64)
#define REG_MIPI_DEV_INT_MASK_N_IDI_VCX2    (0x68)
#define REG_MIPI_DEV_INT_FORCE_IDI_VCX2     (0x6C)
#define REG_MIPI_DEV_INT_MASK_N_MT_IPI      (0x70)
#define REG_MIPI_DEV_INT_FORCE_MT_IPI       (0x74)
#define REG_MIPI_DEV_VPG_CTRL               (0x80)
#define REG_MIPI_DEV_VPG_STATUS             (0x84)
#define REG_MIPI_DEV_VPG_MODE_CFG           (0x88)
#define REG_MIPI_DEV_VPG_PKT_CFG            (0x8C)
#define REG_MIPI_DEV_VPG_PKT_SIZE           (0x90)
#define REG_MIPI_DEV_VPG_HSA_TIME           (0x94)
#define REG_MIPI_DEV_VPG_HBP_TIME           (0x98)
#define REG_MIPI_DEV_VPG_HLINE_TIME         (0x9C)
#define REG_MIPI_DEV_VPG_VSA_LINES          (0xA0)
#define REG_MIPI_DEV_VPG_VBP_LINES          (0xA4)
#define REG_MIPI_DEV_VPG_VFP_LINES          (0xA8)
#define REG_MIPI_DEV_VPG_ACT_LINES          (0xAC)
#define REG_MIPI_DEV_VPG_MAX_FRAME_NUM      (0xB0)
#define REG_MIPI_DEV_VPG_START_LINE_NUM     (0xB4)
#define REG_MIPI_DEV_VPG_STEP_LINE_NUM      (0xB8)
#define REG_MIPI_DEV_VPG_BK_LINES           (0xBC)
#define REG_MIPI_DEV_PHY_RSTZ               (0xE0)
#define REG_MIPI_DEV_PHY_IF_CFG             (0xE4)
#define REG_MIPI_DEV_LPCLK_CTRL             (0xE8)
#define REG_MIPI_DEV_PHY_ULPS_CTRL          (0xEC)
#define REG_MIPI_DEV_CLKMGR_CFG             (0xF0)
#define REG_MIPI_DEV_PHY_TX_TRIGGERS        (0xF4)
#define REG_MIPI_DEV_PHY_CAL                (0xF8)
#define REG_MIPI_DEV_TO_CNT_CFG             (0xFC)
#define REG_MIPI_DEV_PHY_STATUS             (0x110)
#define REG_MIPI_DEV_PHY0_TST_CTRL0         (0x114)
#define REG_MIPI_DEV_PHY0_TST_CTRL1         (0x118)
#define REG_MIPI_DEV_PHY1_TST_CTRL0         (0x11C)
#define REG_MIPI_DEV_PHY1_TST_CTRL1         (0x120)
#define REG_MIPI_DEV_IPI_PKT_CFG            (0x140)
#define REG_MIPI_DEV_IPI_PIXELS             (0x144)
#define REG_MIPI_DEV_IPI_MAX_FRAME_NUM      (0x148)
#define REG_MIPI_DEV_IPI_START_LINE_NUM     (0x14C)
#define REG_MIPI_DEV_IPI_STEP_LINE_NUM      (0x150)
#define REG_MIPI_DEV_IPI_LINES              (0x154)
#define REG_MIPI_DEV_IPI_DATA_SEND_START    (0x158)
#define REG_MIPI_DEV_IPI_FIFO_STATUS        (0x15C)
#define REG_MIPI_DEV_IPI_TRANS_STATUS       (0x160)
#define REG_MIPI_DEV_IPI_HSA_HBP_PPI_TIME   (0x164)
#define REG_MIPI_DEV_IPI_HLINE_PPI_TIME     (0x168)
#define REG_MIPI_DEV_IPI_VSA_LINES          (0x16C)
#define REG_MIPI_DEV_IPI_VBP_LINES          (0x170)
#define REG_MIPI_DEV_IPI_VFP_LINES          (0x174)
#define REG_MIPI_DEV_IPI_ACT_LINES          (0x178)
#define REG_MIPI_DEV_IPI_FB_LINES           (0x17C)
#define REG_MIPI_DEV_IDI_FIFO_STATUS        (0x180)
#define REG_MIPI_DEV_IPI_INSERT_CTRL        (0x184)
#define REG_MIPI_DEV_MT_IPI_CFG             (0x1A0)
#define REG_MIPI_DEV_MT_IPI_DF_TIME         (0x1A4)
#define REG_MIPI_DEV_MT_IPI_FIFO_STATUS     (0x1A8)
#define REG_MIPI_DEV_MT_IPI1_TRANS_CFG      (0x1B0)
#define REG_MIPI_DEV_MT_IPI2_TRANS_CFG      (0x1B4)
#define REG_MIPI_DEV_MT_IPI3_TRANS_CFG      (0x1B8)
#define REG_MIPI_DEV_MT_IPI4_TRANS_CFG      (0x1BC)
#define REG_MIPI_DEV_IPI1_HSA_HBP_TIME      (0x1C0)
#define REG_MIPI_DEV_IPI1_LP_TIME           (0x1C4)
#define REG_MIPI_DEV_IPI2_HSA_HBP_TIME      (0x1C8)
#define REG_MIPI_DEV_IPI2_LP_TIME           (0x1CC)
#define REG_MIPI_DEV_IPI3_HSA_HBP_TIME      (0x1D0)
#define REG_MIPI_DEV_IPI3_LP_TIME           (0x1D4)
#define REG_MIPI_DEV_IPI4_HSA_HBP_TIME      (0x1D8)
#define REG_MIPI_DEV_IPI4_LP_TIME           (0x1DC)
#define REG_MIPI_DEV_IPI2_PKT_CFG           (0x200)
#define REG_MIPI_DEV_IPI2_PIXELS            (0x204)
#define REG_MIPI_DEV_IPI2_MAX_FRAME_NUM     (0x208)
#define REG_MIPI_DEV_IPI2_START_LINE_NUM    (0x20C)
#define REG_MIPI_DEV_IPI2_STEP_LINE_NUM     (0x210)
#define REG_MIPI_DEV_IPI2_LINES             (0x214)
#define REG_MIPI_DEV_IPI2_DATA_SEND_START   (0x218)
#define REG_MIPI_DEV_IPI2_FIFO_STATUS       (0x21C)
#define REG_MIPI_DEV_IPI3_PKT_CFG           (0x240)
#define REG_MIPI_DEV_IPI3_PIXELS            (0x244)
#define REG_MIPI_DEV_IPI3_MAX_FRAME_NUM     (0x248)
#define REG_MIPI_DEV_IPI3_START_LINE_NUM    (0x24C)
#define REG_MIPI_DEV_IPI3_STEP_LINE_NUM     (0x250)
#define REG_MIPI_DEV_IPI3_LINES             (0x254)
#define REG_MIPI_DEV_IPI3_DATA_SEND_START   (0x258)
#define REG_MIPI_DEV_IPI3_FIFO_STATUS       (0x25C)
#define REG_MIPI_DEV_IPI4_PKT_CFG           (0x280)
#define REG_MIPI_DEV_IPI4_PIXELS            (0x284)
#define REG_MIPI_DEV_IPI4_MAX_FRAME_NUM     (0x288)
#define REG_MIPI_DEV_IPI4_START_LINE_NUM    (0x28C)
#define REG_MIPI_DEV_IPI4_STEP_LINE_NUM     (0x290)
#define REG_MIPI_DEV_IPI4_LINES             (0x294)
#define REG_MIPI_DEV_IPI4_DATA_SEND_START   (0x298)
#define REG_MIPI_DEV_IPI4_FIFO_STATUS       (0x29C)
#define REG_MIPI_DEV_INT_ST_FAP_VPG         (0xA00)
#define REG_MIPI_DEV_INT_ST_FAP_IDI         (0xA04)
#define REG_MIPI_DEV_INT_ST_FAP_IPI         (0xA08)
#define REG_MIPI_DEV_INT_ST_FAP_PHY         (0xA0C)
#define REG_MIPI_DEV_INT_ST_FAP_MT_IPI      (0xA18)
#define REG_MIPI_DEV_INT_MASK_N_FAP_VPG     (0xA20)
#define REG_MIPI_DEV_INT_FORCE_FAP_VPG      (0xA24)
#define REG_MIPI_DEV_INT_MASK_N_FAP_IDI     (0xA28)
#define REG_MIPI_DEV_INT_FORCE_FAP_IDI      (0xA2C)
#define REG_MIPI_DEV_INT_MASK_N_FAP_IPI     (0xA30)
#define REG_MIPI_DEV_INT_FORCE_FAP_IPI      (0xA34)
#define REG_MIPI_DEV_INT_MASK_N_FAP_PHY     (0xA38)
#define REG_MIPI_DEV_INT_FORCE_FAP_PHY      (0xA3C)
#define REG_MIPI_DEV_INT_MASK_N_FAP_MT_IPI  (0xA50)
#define REG_MIPI_DEV_INT_FORCE_FAP_MT_IPI   (0xA54)
#define REG_MIPI_DEV_IPI_AP_STATUS          (0xA60)
#define REG_MIPI_DEV_IDI_AP_STATUS          (0xA64)
#define REG_MIPI_DEV_AMBAAPBINTF_AP_STATUS	(0xA68)
#define REG_MIPI_DEV_PHY_IF_CTRL_AP_STATUS  (0xA6C)
#define REG_MIPI_DEV_REG_BANK_AP_STATUS     (0xA70)
#define REG_MIPI_DEV_IPI_FIFOCTRL_AP_STATUS (0xA74)
#define REG_MIPI_DEV_IDI_FIFOCTRL_AP_STATUS (0xA78)
#define REG_MIPI_DEV_PKT_BUILDER_AP_STATUS  (0xA7C)
#define REG_MIPI_DEV_ERR_HANDLER_AP_STATUS  (0xA80)
#define REG_MIPI_DEV_SYNC_AP_STATUS         (0xA84)
#define REG_MIPI_DEV_PKT_IF_AP_STATUS       (0xA88)
#define REG_MIPI_DEV_ECF_AP_STATUS          (0xA8C)
#define REG_MIPI_DEV_CMU_AP_STATUS          (0xA90)
#define REG_MIPI_DEV_MT_IPI_CONTROL_AP_STATUS      (0xA94)
#define REG_MIPI_DEV_IPI2_AP_STATUS                (0xA98)
#define REG_MIPI_DEV_IPI2_FIFOCTRL_AP_STATUS       (0xA9C)
#define REG_MIPI_DEV_IPI3_AP_STATUS                (0xAA0)
#define REG_MIPI_DEV_IPI3_FIFOCTRL_AP_STATUS       (0xAA4)
#define REG_MIPI_DEV_IPI4_AP_STATUS                (0xAA8)
#define REG_MIPI_DEV_IPI4_FIFOCTRL_AP_STATUS       (0xAAC)
#define REG_MIPI_DEV_MASK_N_IPI_AP_STATUS          (0xAC0)
#define REG_MIPI_DEV_FORCE_IPI_AP_STATUS           (0xAC4)
#define REG_MIPI_DEV_MASK_N_IDI_AP_STATUS          (0xAC8)
#define REG_MIPI_DEV_FORCE_IDI_AP_STATUS           (0xACC)
#define REG_MIPI_DEV_MASK_N_AMBAAPBINTF_AP_STATUS  (0xAD0)
#define REG_MIPI_DEV_FORCE_AMBAAPBINTF_AP_STATUS   (0xAD4)
#define REG_MIPI_DEV_MASK_N_PHY_IF_CTRL_AP_STATUS  (0xAD8)
#define REG_MIPI_DEV_FORCE_PHY_IF_CTRL_AP_STATUS   (0xADC)
#define REG_MIPI_DEV_MASK_N_REG_BANK_AP_STATUS     (0xAE0)
#define REG_MIPI_DEV_FORCE_REG_BANK_AP_STATUS      (0xAE4)
#define REG_MIPI_DEV_MASK_N_IPI_FIFOCTRL_AP_STATUS (0xAE8)
#define REG_MIPI_DEV_FORCE_IPI_FIFOCTRL_AP_STATUS  (0xAEC)
#define REG_MIPI_DEV_MASK_N_IDI_FIFOCTRL_AP_STATUS (0xAF0)
#define REG_MIPI_DEV_FORCE_IDI_FIFOCTRL_AP_STATUS  (0xAF4)
#define REG_MIPI_DEV_MASK_N_PKT_BUILDER_AP_STATUS  (0xAF8)
#define REG_MIPI_DEV_FORCE_PKT_BUILDER_AP_STATUS   (0xAFC)
#define REG_MIPI_DEV_MASK_N_ERR_HANDLER_AP_STATUS  (0xB00)
#define REG_MIPI_DEV_FORCE_ERR_HANDLER_AP_STATUS   (0xB04)
#define REG_MIPI_DEV_MASK_N_SYNC_AP_STATUS         (0xB08)
#define REG_MIPI_DEV_FORCE_SYNC_AP_STATUS          (0xB0C)
#define REG_MIPI_DEV_MASK_N_PKT_IF_AP_STATUS       (0xB10)
#define REG_MIPI_DEV_FORCE_PKT_IF_AP_STATUS        (0xB14)
#define REG_MIPI_DEV_MASK_N_ECF_AP_STATUS          (0xB18)
#define REG_MIPI_DEV_FORCE_ECF_AP_STATUS           (0xB1C)
#define REG_MIPI_DEV_MASK_N_CMU_AP_STATUS          (0xB20)
#define REG_MIPI_DEV_FORCE_CMU_AP_STATUS           (0xB24)
#define REG_MIPI_DEV_MASK_N_MT_IPI_CONTROL_AP_STATUS (0xB28)
#define REG_MIPI_DEV_FORCE_MT_IPI_CONTROL_AP_STATUS  (0xB2C)
#define REG_MIPI_DEV_MASK_N_IPI2_AP_STATUS           (0xB30)
#define REG_MIPI_DEV_FORCE_IPI2_AP_STATUS            (0xB34)
#define REG_MIPI_DEV_MASK_N_IPI2_FIFOCTRL_AP_STATUS  (0xB38)
#define REG_MIPI_DEV_FORCE_IPI2_FIFOCTRL_AP_STATUS   (0xB3C)
#define REG_MIPI_DEV_MASK_N_IPI3_AP_STATUS           (0xB40)
#define REG_MIPI_DEV_FORCE_IPI3_AP_STATUS            (0xB44)
#define REG_MIPI_DEV_MASK_N_IPI3_FIFOCTRL_AP_STATUS  (0xB48)
#define REG_MIPI_DEV_FORCE_IPI3_FIFOCTRL_AP_STATUS   (0xB4C)
#define REG_MIPI_DEV_MASK_N_IPI4_AP_STATUS           (0xB50)
#define REG_MIPI_DEV_FORCE_IPI4_AP_STATUS            (0xB54)
#define REG_MIPI_DEV_MASK_N_IPI4_FIFOCTRL_AP_STATUS  (0xB58)
#define REG_MIPI_DEV_FORCE_IPI4_FIFOCTRL_AP_STATUS   (0xB5C)
#define REG_MIPI_DEV_INT_ST_DIAG_MAIN       (0xF00)
#define REG_MIPI_DEV_INT_ST_DIAG0           (0xF04)
#define REG_MIPI_DEV_INT_MASK_N_DIAG0       (0xF10)
#define REG_MIPI_DEV_INT_FORCE_DIAG0        (0xF14)
#define REG_MIPI_DEV_ERR_INJ_CTRL           (0xF40)
#define REG_MIPI_DEV_ERR_INJ_STATUS         (0xF44)
#define REG_MIPI_DEV_ERR_INJ_CHK_MASK       (0xF48)
#define REG_MIPI_DEV_ERR_INJ_DH32_MASK      (0xF4C)
#define REG_MIPI_DEV_ERR_INJ_DL32_MASK      (0xF50)
#define REG_MIPI_DEV_IDI_RAM_ERR_LOG_AP     (0xF60)
#define REG_MIPI_DEV_IDI_RAM_ERR_ADDR_AP    (0xF64)
#define REG_MIPI_DEV_IPI_RAM_ERR_LOG_AP     (0xF68)
#define REG_MIPI_DEV_IPI_RAM_ERR_ADDR_AP    (0xF6C)
#define REG_MIPI_DEV_IPI2_RAM_ERR_LOG_AP    (0xF70)
#define REG_MIPI_DEV_IPI2_RAM_ERR_ADDR_AP   (0xF74)
#define REG_MIPI_DEV_IPI3_RAM_ERR_LOG_AP    (0xF78)
#define REG_MIPI_DEV_IPI3_RAM_ERR_ADDR_AP   (0xF7C)
#define REG_MIPI_DEV_IPI4_RAM_ERR_LOG_AP    (0xF80)
#define REG_MIPI_DEV_IPI4_RAM_ERR_ADDR_AP   (0xF84)

#endif //__HOBOT_MIPI_DEV_REGS_H__
