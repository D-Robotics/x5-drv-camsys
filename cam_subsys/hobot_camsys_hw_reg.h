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

#ifndef HOBOT_CAMSYS_HW_REG_H
#define HOBOT_CAMSYS_HW_REG_H

#include "vio_hw_common_api.h"

enum ipe_reg {
	/*CAM_SYS*/
	CAM_MODULE_ENABLE,
	CAM_SOFT_RSTN1,
	CAM_SOFT_RSTN2,
	CAM_CGM_ENABLE,
	CAM_MEM_CFG00,
	CAM_MEM_CFG01,
	CAM_MEM_CFG02,
	CAM_MIPI_HOST0_CTRL,
	CAM_MIPI_HOST1_CTRL,
	CAM_MIPI_HOST2_CTRL,
	CAM_MIPI_HOST3_CTRL,
	CAM_MIPI_DEVICE0_CTRL0,
	CAM_MIPI_DEVICE0_CTRL1,
	CAM_MIPI_DEVICE0_CTRL2,
	CAM_MIPI_DEVICE0_CTRL3,
	CAM_MIPI_DEVICE0_CTRL4,
	CAM_MIPI_DEVICE1_CTRL0,
	CAM_MIPI_DEVICE1_CTRL1,
	CAM_MIPI_DEVICE1_CTRL2,
	CAM_MIPI_DEVICE1_CTRL3,
	CAM_MIPI_DEVICE1_CTRL4,
	CAM_CIM_DMA_QOS,
	CAM_LITE_MMU_EN,
	CAM_CIM_ADDR_CFG_0,
	CAM_CIM_ADDR_CFG_1,
	CAM_CIM_ADDR_CFG_2,
	CAM_CIM_ADDR_CFG_3,
	CAM_CIM_ADDR_CFG_4,
	CAM_CIM_ADDR_CFG_5,
	CAM_CIM_ADDR_CFG_6,
	CAM_CIM_ADDR_CFG_7,
	CAM_CIM_ADDR_CFG_8,
	CAM_CIM_ADDR_CFG_9,
	CAM_CIM_ADDR_CFG_A,
	CAM_CIM_ADDR_CFG_B,
	CAM_CIM_ADDR_CFG_C,
	CAM_CIM_ADDR_CFG_D,
	CAM_CIM_ADDR_CFG_E,
	CAM_CIM_ADDR_CFG_F,
	CAM_PYM2_ADDR_CFG_0,
	CAM_PYM2_ADDR_CFG_1,
	CAM_PYM2_ADDR_CFG_2,
	CAM_PYM2_ADDR_CFG_3,
	CAM_PYM2_ADDR_CFG_4,
	CAM_PYM2_ADDR_CFG_5,
	CAM_PYM2_ADDR_CFG_6,
	CAM_PYM2_ADDR_CFG_7,
	CAM_PYM2_ADDR_CFG_8,
	CAM_PYM2_ADDR_CFG_9,
	CAM_PYM2_ADDR_CFG_A,
	CAM_PYM2_ADDR_CFG_B,
	CAM_PYM2_ADDR_CFG_C,
	CAM_PYM2_ADDR_CFG_D,
	CAM_PYM2_ADDR_CFG_E,
	CAM_PYM2_ADDR_CFG_F,
	CAM_GDC_ADDR_CFG_0,
	CAM_GDC_ADDR_CFG_1,
	CAM_GDC_ADDR_CFG_2,
	CAM_GDC_ADDR_CFG_3,
	CAM_GDC_ADDR_CFG_4,
	CAM_GDC_ADDR_CFG_5,
	CAM_GDC_ADDR_CFG_6,
	CAM_GDC_ADDR_CFG_7,
	CAM_GDC_ADDR_CFG_8,
	CAM_GDC_ADDR_CFG_9,
	CAM_GDC_ADDR_CFG_A,
	CAM_GDC_ADDR_CFG_B,
	CAM_GDC_ADDR_CFG_C,
	CAM_GDC_ADDR_CFG_D,
	CAM_GDC_ADDR_CFG_E,
	CAM_GDC_ADDR_CFG_F,
	CAM_STH_ADDR_CFG_0,
	CAM_STH_ADDR_CFG_1,
	CAM_STH_ADDR_CFG_2,
	CAM_STH_ADDR_CFG_3,
	CAM_STH_ADDR_CFG_4,
	CAM_STH_ADDR_CFG_5,
	CAM_STH_ADDR_CFG_6,
	CAM_STH_ADDR_CFG_7,
	CAM_STH_ADDR_CFG_8,
	CAM_STH_ADDR_CFG_9,
	CAM_STH_ADDR_CFG_A,
	CAM_STH_ADDR_CFG_B,
	CAM_STH_ADDR_CFG_C,
	CAM_STH_ADDR_CFG_D,
	CAM_STH_ADDR_CFG_E,
	CAM_STH_ADDR_CFG_F,
	CAM_IAR0_ADDR_CFG_0,
	CAM_IAR0_ADDR_CFG_1,
	CAM_IAR0_ADDR_CFG_2,
	CAM_IAR0_ADDR_CFG_3,
	CAM_IAR0_ADDR_CFG_4,
	CAM_IAR0_ADDR_CFG_5,
	CAM_IAR0_ADDR_CFG_6,
	CAM_IAR0_ADDR_CFG_7,
	CAM_IAR0_ADDR_CFG_8,
	CAM_IAR0_ADDR_CFG_9,
	CAM_IAR0_ADDR_CFG_A,
	CAM_IAR0_ADDR_CFG_B,
	CAM_IAR0_ADDR_CFG_C,
	CAM_IAR0_ADDR_CFG_D,
	CAM_IAR0_ADDR_CFG_E,
	CAM_IAR0_ADDR_CFG_F,
	CAM_IAR1_ADDR_CFG_0,
	CAM_IAR1_ADDR_CFG_1,
	CAM_IAR1_ADDR_CFG_2,
	CAM_IAR1_ADDR_CFG_3,
	CAM_IAR1_ADDR_CFG_4,
	CAM_IAR1_ADDR_CFG_5,
	CAM_IAR1_ADDR_CFG_6,
	CAM_IAR1_ADDR_CFG_7,
	CAM_IAR1_ADDR_CFG_8,
	CAM_IAR1_ADDR_CFG_9,
	CAM_IAR1_ADDR_CFG_A,
	CAM_IAR1_ADDR_CFG_B,
	CAM_IAR1_ADDR_CFG_C,
	CAM_IAR1_ADDR_CFG_D,
	CAM_IAR1_ADDR_CFG_E,
	CAM_IAR1_ADDR_CFG_F,
	CAM_IAR0_DEBUG,
	CAM_IAR1_DEBUG,
	CAM_RX_DEBUG,
	CAM_TX_DEBUG,
	CAM_MIPI_HOST0_CTRL1,
	CAM_MIPI_HOST1_CTRL1,
	CAM_MIPI_HOST2_CTRL1,
	CAM_MIPI_HOST3_CTRL1,
	/* IPE */
	IPE_IPE_SEL,
	IPE_MAX_ADDR_SINTER1,
	IPE_MAX_ADDR_SINTER2,
	IPE_MAX_ADDR,
	IPE_MAX_ADDR_FIFO_IN,
	IPE_MAX_ADDR_FIFO_OUT,
	IPE_MAX_ADDR_FIFO,
	IPE_MAX_ADDR_PC,
	IPE_MAX_ADDR_PC_ALIGN,
	IPE_MAX_ADDR_EXP_FIFO,
	IPE_ISP_WRM0,
	IPE_ISP_WRM1,
	IPE_ISP_WRM2,
	IPE_ISP_WRM3,
	IPE_ISP_TAG_OUT,
	IPE_ISP_QOS,
	IPE_PYM_QOS,
	IPE_ISP_SRAM_ECC_CTRL,
	IPE_ISP_ADDR_CFG_0,
	IPE_ISP_ADDR_CFG_1,
	IPE_ISP_ADDR_CFG_2,
	IPE_ISP_ADDR_CFG_3,
	IPE_ISP_ADDR_CFG_4,
	IPE_ISP_ADDR_CFG_5,
	IPE_ISP_ADDR_CFG_6,
	IPE_ISP_ADDR_CFG_7,
	IPE_ISP_ADDR_CFG_8,
	IPE_ISP_ADDR_CFG_9,
	IPE_ISP_ADDR_CFG_A,
	IPE_ISP_ADDR_CFG_B,
	IPE_ISP_ADDR_CFG_C,
	IPE_ISP_ADDR_CFG_D,
	IPE_ISP_ADDR_CFG_E,
	IPE_ISP_ADDR_CFG_F,
	IPE_PYM_ADDR_CFG_0,
	IPE_PYM_ADDR_CFG_1,
	IPE_PYM_ADDR_CFG_2,
	IPE_PYM_ADDR_CFG_3,
	IPE_PYM_ADDR_CFG_4,
	IPE_PYM_ADDR_CFG_5,
	IPE_PYM_ADDR_CFG_6,
	IPE_PYM_ADDR_CFG_7,
	IPE_PYM_ADDR_CFG_8,
	IPE_PYM_ADDR_CFG_9,
	IPE_PYM_ADDR_CFG_A,
	IPE_PYM_ADDR_CFG_B,
	IPE_PYM_ADDR_CFG_C,
	IPE_PYM_ADDR_CFG_D,
	IPE_PYM_ADDR_CFG_E,
	IPE_PYM_ADDR_CFG_F,
	IPE_ISP_SRAM_MISSION_ERR_ST0,
	IPE_ISP_SRAM_MISSION_ERR_ST1,
	IPE_ISP_SRAM_MISSION_ERR_ST2,
	IPE_ISP_SRAM_LATENT_ERR_ST0,
	IPE_ISP_SRAM_LATENT_ERR_ST1,
	IPE_ISP_SRAM_LATENT_ERR_ST2,
	NUM_OF_IPE_REG,
};

enum ipe_field{
	CAM_F_CIM_DMA_APB_RST,
	CAM_F_CIM_DMA_RST,
	CAM_F_ERM_APB_RST,
	CAM_F_CMM_APB_RST,
	CAM_F_IAR1_RST,
	CAM_F_IAR0_RST,
	CAM_F_CIM_APB_RST,
	CAM_F_CIM_RST,
	CAM_F_DVP_RST,
	CAM_F_DEVICE1_RST,
	CAM_F_DEVICE0_RST,
	CAM_F_HOST3_RST,
	CAM_F_HOST2_RST,
	CAM_F_HOST1_RST,
	CAM_F_HOST0_RST,
	CAM_F_STH_PRST,
	CAM_F_STH_VRST,
	CAM_F_GDC_PRST,
	CAM_F_GDC_VRST,
	CAM_F_PYM2_PRST,
	CAM_F_PYM2_VRST,
	CAM_F_PYM1_PRST,
	CAM_F_PYM1_VRST,
	CAM_F_ISP1_ARST,
	CAM_F_ISP1_VRST,
	CAM_F_ISP1_PRST,
	CAM_F_IPE1_PRST,
	CAM_F_PYM0_PRST,
	CAM_F_PYM0_VRST,
	CAM_F_ISP0_ARST,
	CAM_F_ISP0_VRST,
	CAM_F_ISP0_PRST,
	CAM_F_IPE0_PRST,
	CAM_F_CGM_MIPI_SCAN_EN,
	CAM_F_CGM_DPHY_PLLEXT_EN,
	CAM_F_CGM_DPHY_REF_EN,
	CAM_F_CGM_DPHY_CFG_EN,
	CAM_F_CGM_DMA_EN,
	CAM_F_CGM_STH_EN,
	CAM_F_CGM_GDC_EN,
	CAM_F_MIPI_HOST3_IDI_CLK,
	CAM_F_MIPI_HOST2_IDI_CLK,
	CAM_F_CGM_RXBCLK_RX3_EN,
	CAM_F_CGM_RXBCLK_RX2_EN,
	CAM_F_CGM_IAR1_EN,
	CAM_F_CGM_IAR0_EN,
	CAM_F_CGM_CAM_NOC_EN,
	CAM_F_CGM_ISP_AXI_EN,
	CAM_F_CGM_PAD_24M_EN,
	CAM_F_CGM_PYM2_EN,
	CAM_F_CGM_SYS_EN,
	CAM_F_CGM_APB_EN,
	CAM_F_CGM_DVP_OCC_EN,
	CAM_F_CGM_TXBCLK_TX1_EN,
	CAM_F_CGM_TXBCLK_TX0_EN,
	CAM_F_CGM_RXBCLK_RX1_EN,
	CAM_F_CGM_RXBCLK_RX0_EN,
	CAM_F_MIPI_HOST1_IDI_CLK,
	CAM_F_MIPI_HOST0_IDI_CLK,
	IPE_F_ENABLE_ISP,
	IPE_F_ENABLE_PYM,
	IPE_F_CIM_PYM,
	IPE_F_ISP_OUT_UV,
	IPE_F_ISP_OUT_Y,
	IPE_F_ISP_TAG3_OUT,
	IPE_F_ISP_TAG2_OUT,
	IPE_F_ISP_TAG1_OUT,
	IPE_F_ISP_SRAM_ERR_CLEAR,
	IPE_F_ISP_SRAM_ERR_EN,
	IPE_F_ISP_ECC_ENCODER_PARITY_IN,
	IPE_F_ISP_ECC_DECODER_BYPASS,
	IPE_F_ISP_ECC_ENCODER_BYPASS,
	NUM_OF_IPE_FIELD,
};

#endif //__HOBOT_CAMSYS_HW_REG_H__
