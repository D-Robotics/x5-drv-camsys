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

#include "hobot_camsys_hw_reg.h"
#include "camsys_hw_api.h"

/**
 * Purpose: camsys register attribute
 * Value: fixed register name, address offset and WR/RD type
 * Range: hobot_camsys_hw_reg.c
 * Attention: NA
 */
static struct vio_reg_def camsys_regs[NUM_OF_IPE_REG]={
	/*CAM_SYS*/
	{"CAM_MODULE_ENABLE",             0x0000, RW, 0x7FFFFFFF},
	{"CAM_SOFT_RSTN1",                0x0004, RW, 0x0},
	{"CAM_SOFT_RSTN2",                0x0008, RW, 0x38E},
	{"CAM_CGM_ENABLE",                0x000C, RW, 0x2F9F6FC},
	{"CAM_MEM_CFG00",                 0x0010, RW, 0x15115595},
	{"CAM_MEM_CFG01",                 0x0014, RW, 0x8AAB5155},
	{"CAM_MEM_CFG02",                 0x0018, RW, 0xC7574150},
	{"CAM_MIPI_HOST0_CTRL",           0x0020, RW, 0xE0A},
	{"CAM_MIPI_HOST1_CTRL",           0x0024, RW, 0xE0A},
	{"CAM_MIPI_HOST2_CTRL",           0x0028, RW, 0xE0A},
	{"CAM_MIPI_HOST3_CTRL",           0x002C, RW, 0xE0A},
	{"CAM_MIPI_DEVICE0_CTRL0",        0x0030, RW, 0xE0A},
	{"CAM_MIPI_DEVICE0_CTRL1",        0x0034, RW, 0x0},
	{"CAM_MIPI_DEVICE0_CTRL2",        0x0038, RW, 0x0},
	{"CAM_MIPI_DEVICE0_CTRL3",        0x003C, RW, 0x0},
	{"CAM_MIPI_DEVICE0_CTRL4",        0x0040, RO, 0x0},
	{"CAM_MIPI_DEVICE1_CTRL0",        0x0048, RW, 0xE0A},
	{"CAM_MIPI_DEVICE1_CTRL1",        0x004C, RW, 0x0},
	{"CAM_MIPI_DEVICE1_CTRL2",        0x0050, RW, 0x0},
	{"CAM_MIPI_DEVICE1_CTRL3",        0x0054, RW, 0x0},
	{"CAM_MIPI_DEVICE1_CTRL4",        0x0058, RO, 0x0},
	{"CAM_CIM_DMA_QOS",               0x005C, RW, 0x0},
	{"CAM_LITE_MMU_EN",               0x0064, RW, 0x1},
	{"CAM_CIM_ADDR_CFG_0",            0x0068, RW, 0x0},
	{"CAM_CIM_ADDR_CFG_1",            0x006C, RW, 0x1},
	{"CAM_CIM_ADDR_CFG_2",            0x0070, RW, 0x2},
	{"CAM_CIM_ADDR_CFG_3",            0x0074, RW, 0x3},
	{"CAM_CIM_ADDR_CFG_4",            0x0078, RW, 0x4},
	{"CAM_CIM_ADDR_CFG_5",            0x007C, RW, 0x5},
	{"CAM_CIM_ADDR_CFG_6",            0x0080, RW, 0x6},
	{"CAM_CIM_ADDR_CFG_7",            0x0084, RW, 0x7},
	{"CAM_CIM_ADDR_CFG_8",            0x0088, RW, 0x8},
	{"CAM_CIM_ADDR_CFG_9",            0x008C, RW, 0x9},
	{"CAM_CIM_ADDR_CFG_A",            0x0090, RW, 0xA},
	{"CAM_CIM_ADDR_CFG_B",            0x0094, RW, 0xB},
	{"CAM_CIM_ADDR_CFG_C",            0x0098, RW, 0xC},
	{"CAM_CIM_ADDR_CFG_D",            0x009C, RW, 0xD},
	{"CAM_CIM_ADDR_CFG_E",            0x00A0, RW, 0xE},
	{"CAM_CIM_ADDR_CFG_F",            0x00A4, RW, 0xF},
	{"CAM_PYM2_ADDR_CFG_0",           0x00A8, RW, 0x0},
	{"CAM_PYM2_ADDR_CFG_1",           0x00AC, RW, 0x1},
	{"CAM_PYM2_ADDR_CFG_2",           0x00B0, RW, 0x2},
	{"CAM_PYM2_ADDR_CFG_3",           0x00B4, RW, 0x3},
	{"CAM_PYM2_ADDR_CFG_4",           0x00B8, RW, 0x4},
	{"CAM_PYM2_ADDR_CFG_5",           0x00BC, RW, 0x5},
	{"CAM_PYM2_ADDR_CFG_6",           0x00C0, RW, 0x6},
	{"CAM_PYM2_ADDR_CFG_7",           0x00C4, RW, 0x7},
	{"CAM_PYM2_ADDR_CFG_8",           0x00C8, RW, 0x88},
	{"CAM_PYM2_ADDR_CFG_9",           0x00CC, RW, 0x89},
	{"CAM_PYM2_ADDR_CFG_A",           0x00D0, RW, 0x8A},
	{"CAM_PYM2_ADDR_CFG_B",           0x00D4, RW, 0x8B},
	{"CAM_PYM2_ADDR_CFG_C",           0x00D8, RW, 0x8C},
	{"CAM_PYM2_ADDR_CFG_D",           0x00DC, RW, 0x8D},
	{"CAM_PYM2_ADDR_CFG_E",           0x00E0, RW, 0x8E},
	{"CAM_PYM2_ADDR_CFG_F",           0x00E4, RW, 0x8F},
	{"CAM_GDC_ADDR_CFG_0",            0x00E8, RW, 0x0},
	{"CAM_GDC_ADDR_CFG_1",            0x00EC, RW, 0x1},
	{"CAM_GDC_ADDR_CFG_2",            0x00F0, RW, 0x2},
	{"CAM_GDC_ADDR_CFG_3",            0x00F4, RW, 0x3},
	{"CAM_GDC_ADDR_CFG_4",            0x00F8, RW, 0x4},
	{"CAM_GDC_ADDR_CFG_5",            0x00FC, RW, 0x5},
	{"CAM_GDC_ADDR_CFG_6",            0x0100, RW, 0x6},
	{"CAM_GDC_ADDR_CFG_7",            0x0104, RW, 0x7},
	{"CAM_GDC_ADDR_CFG_8",            0x0108, RW, 0x88},
	{"CAM_GDC_ADDR_CFG_9",            0x010C, RW, 0x89},
	{"CAM_GDC_ADDR_CFG_A",            0x0110, RW, 0x8A},
	{"CAM_GDC_ADDR_CFG_B",            0x0114, RW, 0x8B},
	{"CAM_GDC_ADDR_CFG_C",            0x0118, RW, 0x8C},
	{"CAM_GDC_ADDR_CFG_D",            0x011C, RW, 0x8D},
	{"CAM_GDC_ADDR_CFG_E",            0x0120, RW, 0x8E},
	{"CAM_GDC_ADDR_CFG_F",            0x0124, RW, 0x8F},
	{"CAM_STH_ADDR_CFG_0",            0x0128, RW, 0x0},
	{"CAM_STH_ADDR_CFG_1",            0x012C, RW, 0x1},
	{"CAM_STH_ADDR_CFG_2",            0x0130, RW, 0x2},
	{"CAM_STH_ADDR_CFG_3",            0x0134, RW, 0x3},
	{"CAM_STH_ADDR_CFG_4",            0x0138, RW, 0x4},
	{"CAM_STH_ADDR_CFG_5",            0x013C, RW, 0x5},
	{"CAM_STH_ADDR_CFG_6",            0x0140, RW, 0x6},
	{"CAM_STH_ADDR_CFG_7",            0x0144, RW, 0x7},
	{"CAM_STH_ADDR_CFG_8",            0x0148, RW, 0x88},
	{"CAM_STH_ADDR_CFG_9",            0x014C, RW, 0x89},
	{"CAM_STH_ADDR_CFG_A",            0x0150, RW, 0x8A},
	{"CAM_STH_ADDR_CFG_B",            0x0154, RW, 0x8B},
	{"CAM_STH_ADDR_CFG_C",            0x0158, RW, 0x8C},
	{"CAM_STH_ADDR_CFG_D",            0x015C, RW, 0x8D},
	{"CAM_STH_ADDR_CFG_E",            0x0160, RW, 0x8E},
	{"CAM_STH_ADDR_CFG_F",            0x0164, RW, 0x8F},
	{"CAM_IAR0_ADDR_CFG_0",           0x0168, RW, 0x0},
	{"CAM_IAR0_ADDR_CFG_1",           0x016C, RW, 0x1},
	{"CAM_IAR0_ADDR_CFG_2",           0x0170, RW, 0x2},
	{"CAM_IAR0_ADDR_CFG_3",           0x0174, RW, 0x3},
	{"CAM_IAR0_ADDR_CFG_4",           0x0178, RW, 0x4},
	{"CAM_IAR0_ADDR_CFG_5",           0x017C, RW, 0x5},
	{"CAM_IAR0_ADDR_CFG_6",           0x0180, RW, 0x6},
	{"CAM_IAR0_ADDR_CFG_7",           0x0184, RW, 0x7},
	{"CAM_IAR0_ADDR_CFG_8",           0x0188, RW, 0x88},
	{"CAM_IAR0_ADDR_CFG_9",           0x018C, RW, 0x89},
	{"CAM_IAR0_ADDR_CFG_A",           0x0190, RW, 0x8A},
	{"CAM_IAR0_ADDR_CFG_B",           0x0194, RW, 0x8B},
	{"CAM_IAR0_ADDR_CFG_C",           0x0198, RW, 0x8C},
	{"CAM_IAR0_ADDR_CFG_D",           0x019C, RW, 0x8D},
	{"CAM_IAR0_ADDR_CFG_E",           0x01A0, RW, 0x8E},
	{"CAM_IAR0_ADDR_CFG_F",           0x01A4, RW, 0x8F},
	{"CAM_IAR1_ADDR_CFG_0",           0x01A8, RW, 0x0},
	{"CAM_IAR1_ADDR_CFG_1",           0x01AC, RW, 0x1},
	{"CAM_IAR1_ADDR_CFG_2",           0x01B0, RW, 0x2},
	{"CAM_IAR1_ADDR_CFG_3",           0x01B4, RW, 0x3},
	{"CAM_IAR1_ADDR_CFG_4",           0x01B8, RW, 0x4},
	{"CAM_IAR1_ADDR_CFG_5",           0x01BC, RW, 0x5},
	{"CAM_IAR1_ADDR_CFG_6",           0x01C0, RW, 0x6},
	{"CAM_IAR1_ADDR_CFG_7",           0x01C4, RW, 0x7},
	{"CAM_IAR1_ADDR_CFG_8",           0x01C8, RW, 0x88},
	{"CAM_IAR1_ADDR_CFG_9",           0x01CC, RW, 0x89},
	{"CAM_IAR1_ADDR_CFG_A",           0x01D0, RW, 0x8A},
	{"CAM_IAR1_ADDR_CFG_B",           0x01D4, RW, 0x8B},
	{"CAM_IAR1_ADDR_CFG_C",           0x01D8, RW, 0x8C},
	{"CAM_IAR1_ADDR_CFG_D",           0x01DC, RW, 0x8D},
	{"CAM_IAR1_ADDR_CFG_E",           0x01E0, RW, 0x8E},
	{"CAM_IAR1_ADDR_CFG_F",           0x01E4, RW, 0x8F},
	{"CAM_IAR0_DEBUG",                0x0200, RW, 0x0},
	{"CAM_IAR1_DEBUG",                0x0204, RW, 0x0},
	{"CAM_RX_DEBUG",                  0x0208, RO, 0x0},
	{"CAM_TX_DEBUG",                  0x020C, RO, 0x0},
	{"CAM_MIPI_HOST0_CTRL1",          0x0210, RW, 0x1},
	{"CAM_MIPI_HOST1_CTRL1",	  0x0214, RW, 0x1},
	{"CAM_MIPI_HOST2_CTRL1",	  0x0218, RW, 0x1},
	{"CAM_MIPI_HOST3_CTRL1",	  0x021C, RW, 0x1},
	/* IPE */
	{"IPE_IPE_SEL",                     0x0000, RW, 0x64},
	{"IPE_MAX_ADDR_SINTER1",            0x0004, RW, 0x55D},
	{"IPE_MAX_ADDR_SINTER2",            0x0008, RW, 0x2B2},
	{"IPE_MAX_ADDR",                    0x000C, RW, 0x807},
	{"IPE_MAX_ADDR_FIFO_IN",            0x0010, RW, 0x3FF},
	{"IPE_MAX_ADDR_FIFO_OUT",           0x0014, RW, 0x3FF},
	{"IPE_MAX_ADDR_FIFO",               0x0018, RW, 0xFF},
	{"IPE_MAX_ADDR_PC",                 0x001C, RW, 0x7F},
	{"IPE_MAX_ADDR_PC_ALIGN",           0x0020, RW, 0x205},
	{"IPE_MAX_ADDR_EXP_FIFO",           0x0024, RW, 0x102},
	{"IPE_ISP_WRM0",                    0x0028, RO, 0x0},
	{"IPE_ISP_WRM1",                    0x002C, RO, 0x0},
	{"IPE_ISP_WRM2",                    0x0030, RO, 0x0},
	{"IPE_ISP_WRM3",                    0x0034, RO, 0x0},
	{"IPE_ISP_TAG_OUT",                 0x0038, RO, 0x0},
	{"IPE_ISP_QOS",                     0x003C, RW, 0x0},
	{"IPE_PYM_QOS",                     0x0040, RW, 0x0},
	{"IPE_ISP_SRAM_ECC_CTRL",           0x0044, RW, 0x3},
	{"IPE_ISP_ADDR_CFG_0",              0x0048, RW, 0x0},
	{"IPE_ISP_ADDR_CFG_1",              0x004C, RW, 0x1},
	{"IPE_ISP_ADDR_CFG_2",              0x0050, RW, 0x2},
	{"IPE_ISP_ADDR_CFG_3",              0x0054, RW, 0x3},
	{"IPE_ISP_ADDR_CFG_4",              0x0058, RW, 0x4},
	{"IPE_ISP_ADDR_CFG_5",              0x005C, RW, 0x5},
	{"IPE_ISP_ADDR_CFG_6",              0x0060, RW, 0x6},
	{"IPE_ISP_ADDR_CFG_7",              0x0064, RW, 0x7},
	{"IPE_ISP_ADDR_CFG_8",              0x0068, RW, 0x8},
	{"IPE_ISP_ADDR_CFG_9",              0x006C, RW, 0x9},
	{"IPE_ISP_ADDR_CFG_A",              0x0070, RW, 0xA},
	{"IPE_ISP_ADDR_CFG_B",              0x0074, RW, 0xB},
	{"IPE_ISP_ADDR_CFG_C",              0x0078, RW, 0xC},
	{"IPE_ISP_ADDR_CFG_D",              0x007C, RW, 0xD},
	{"IPE_ISP_ADDR_CFG_E",              0x0080, RW, 0xE},
	{"IPE_ISP_ADDR_CFG_F",              0x0084, RW, 0xF},
	{"IPE_PYM_ADDR_CFG_0",              0x0088, RW, 0x0},
	{"IPE_PYM_ADDR_CFG_1",              0x008C, RW, 0x1},
	{"IPE_PYM_ADDR_CFG_2",              0x0090, RW, 0x2},
	{"IPE_PYM_ADDR_CFG_3",              0x0094, RW, 0x3},
	{"IPE_PYM_ADDR_CFG_4",              0x0098, RW, 0x4},
	{"IPE_PYM_ADDR_CFG_5",              0x009C, RW, 0x5},
	{"IPE_PYM_ADDR_CFG_6",              0x00A0, RW, 0x6},
	{"IPE_PYM_ADDR_CFG_7",              0x00A4, RW, 0x7},
	{"IPE_PYM_ADDR_CFG_8",              0x00A8, RW, 0x8},
	{"IPE_PYM_ADDR_CFG_9",	            0x00AC, RW, 0x9},
	{"IPE_PYM_ADDR_CFG_A",	            0x00B0, RW, 0xA},
	{"IPE_PYM_ADDR_CFG_B",	            0x00B4, RW, 0xB},
	{"IPE_PYM_ADDR_CFG_C",	            0x00B8, RW, 0xC},
	{"IPE_PYM_ADDR_CFG_D",	            0x00BC, RW, 0xD},
	{"IPE_PYM_ADDR_CFG_E",	            0x00C0, RW, 0xE},
	{"IPE_PYM_ADDR_CFG_F",	            0x00C4, RW, 0xF},
	{"IPE_ISP_SRAM_MISSION_ERR_ST0",    0x0200, RO, 0x0},
	{"IPE_ISP_SRAM_MISSION_ERR_ST1",    0x0204, RO, 0x0},
	{"IPE_ISP_SRAM_MISSION_ERR_ST2",    0x0208, RO, 0x0},
	{"IPE_ISP_SRAM_LATENT_ERR_ST0",     0x020C, RO, 0x0},
	{"IPE_ISP_SRAM_LATENT_ERR_ST1",	    0x0210, RO, 0x0},
	{"IPE_ISP_SRAM_LATENT_ERR_ST2",	    0x0214, RO, 0x0},
};

/**
 * Purpose: camsys register bit field
 * Value: fixed bit attributes
 * Range: hobot_camsys_hw_reg.c
 * Attention: NA
 */
static const struct vio_field_def camsys_fields[NUM_OF_IPE_FIELD] = {
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_CIM_DMA_APB_RST,	  14, 1 , 0x0},
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_CIM_DMA_RST,	  13, 1 , 0x0},
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_ERM_APB_RST,	  12, 1 , 0x0},
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_CMM_APB_RST,	  11, 1 , 0x0},
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_IAR1_RST,	  10, 1 , 0x0},
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_IAR0_RST,	  9 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_CIM_APB_RST,       8 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_CIM_RST,           7 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_DVP_RST,           6 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_DEVICE1_RST,       5 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_DEVICE0_RST,       4 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_HOST3_RST,         3 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_HOST2_RST,         2 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_HOST1_RST,         1 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN1, (u32)CAM_F_HOST0_RST,	  0 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_STH_PRST,          17, 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_STH_VRST,          16, 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_GDC_PRST,          15, 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_GDC_VRST,          14, 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_PYM2_PRST,         13, 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_PYM2_VRST,         12, 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_PYM1_PRST,         11, 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_PYM1_VRST,         10, 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_ISP1_ARST,         9 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_ISP1_VRST,         8 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_ISP1_PRST,         7 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_IPE1_PRST,         6 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_PYM0_PRST,         5 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_PYM0_VRST,         4 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_ISP0_ARST,         3 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_ISP0_VRST,         2 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_ISP0_PRST,         1 , 1 , 0x0},
	{(u32)CAM_SOFT_RSTN2, (u32)CAM_F_IPE0_PRST,         0 , 1 , 0x0},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_MIPI_SCAN_EN,     25, 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_DPHY_PLLEXT_EN,   24, 1 , 0x0},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_DPHY_REF_EN,      23, 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_DPHY_CFG_EN,      22, 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_DMA_EN,           21, 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_STH_EN,           20, 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_GDC_EN,           19, 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_MIPI_HOST3_IDI_CLK,   18, 1 , 0x0},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_MIPI_HOST2_IDI_CLK,   17, 1 , 0x0},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_RXBCLK_RX3_EN,    16, 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_RXBCLK_RX2_EN,    15, 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_IAR1_EN,          14, 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_IAR0_EN,          13, 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_CAM_NOC_EN,       12, 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_ISP_AXI_EN,       11, 1 , 0x0},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_PAD_24M_EN,       10, 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_PYM2_EN,          9 , 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_SYS_EN,           8 , 1 , 0x0},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_APB_EN,           7 , 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_DVP_OCC_EN,       6 , 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_TXBCLK_TX1_EN,    5 , 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_TXBCLK_TX0_EN,    4 , 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_RXBCLK_RX1_EN,    3 , 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_CGM_RXBCLK_RX0_EN,    2 , 1 , 0x1},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_MIPI_HOST1_IDI_CLK,	1 , 1 , 0x0},
	{(u32)CAM_CGM_ENABLE, (u32)CAM_F_MIPI_HOST0_IDI_CLK,	0 , 1 , 0x0},
	{(u32)IPE_IPE_SEL, (u32)IPE_F_ENABLE_ISP,			6, 1, 0x1},
	{(u32)IPE_IPE_SEL, (u32)IPE_F_ENABLE_PYM, 		5, 1, 0x1},
	{(u32)IPE_IPE_SEL, (u32)IPE_F_CIM_PYM,			4, 1, 0x0},
	{(u32)IPE_IPE_SEL, (u32)IPE_F_ISP_OUT_UV, 		2, 2, 0x1},
	{(u32)IPE_IPE_SEL, (u32)IPE_F_ISP_OUT_Y,  		0, 2, 0x0},
	{(u32)IPE_ISP_TAG_OUT, (u32)IPE_F_ISP_TAG3_OUT,	6, 3, 0x0},
	{(u32)IPE_ISP_TAG_OUT, (u32)IPE_F_ISP_TAG2_OUT,	3, 3, 0x0},
	{(u32)IPE_ISP_TAG_OUT, (u32)IPE_F_ISP_TAG1_OUT,	0, 3, 0x0},
	{(u32)IPE_ISP_SRAM_ECC_CTRL, (u32)IPE_F_ISP_SRAM_ERR_CLEAR,		13, 1 , 0x0},
	{(u32)IPE_ISP_SRAM_ECC_CTRL, (u32)IPE_F_ISP_SRAM_ERR_EN,			12, 1 , 0x0},
	{(u32)IPE_ISP_SRAM_ECC_CTRL, (u32)IPE_F_ISP_ECC_ENCODER_PARITY_IN,	2 , 10, 0x0},
	{(u32)IPE_ISP_SRAM_ECC_CTRL, (u32)IPE_F_ISP_ECC_DECODER_BYPASS,		1 , 1 , 0x1},
	{(u32)IPE_ISP_SRAM_ECC_CTRL, (u32)IPE_F_ISP_ECC_ENCODER_BYPASS,		0 , 1 , 0x1},
};


void ipe_set_isp_ecc_sram_code_reg(void __iomem *base_reg)
{
    vio_hw_set_field(base_reg, &camsys_regs[IPE_ISP_SRAM_ECC_CTRL],
                        &camsys_fields[IPE_F_ISP_ECC_DECODER_BYPASS], 0);

    vio_hw_set_field(base_reg, &camsys_regs[IPE_ISP_SRAM_ECC_CTRL],
                        &camsys_fields[IPE_F_ISP_ECC_ENCODER_BYPASS], 0);
}


s32 cam_sys_module_reset(void __iomem *base_reg, u32 module, u32 enable)
{
	s32 field_index[3] = {-1, -1, -1};
	s32 reg_index = -1;
	u32 i = 0;
	s32 ret = 0;

	switch (module) {
		case (u32)CIM_DMA_RST:
			reg_index = CAM_SOFT_RSTN1;
			field_index[0] = CAM_F_CIM_DMA_APB_RST;
			field_index[1] = CAM_F_CIM_DMA_RST;
			break;
		case (u32)ERM_RST:
			reg_index = CAM_SOFT_RSTN1;
			field_index[0] = CAM_F_ERM_APB_RST;
			break;
		case (u32)CMM_RST:
			reg_index = CAM_SOFT_RSTN1;
			field_index[0] = CAM_F_CMM_APB_RST;
			break;
		case (u32)IAR1_RST:
			reg_index = CAM_SOFT_RSTN1;
			field_index[0] = CAM_F_IAR1_RST;
			break;
		case (u32)IAR0_RST:
			reg_index = CAM_SOFT_RSTN1;
			field_index[0] = CAM_F_IAR0_RST;
			break;
		case (u32)CIM_RST:
			reg_index = CAM_SOFT_RSTN1;
			field_index[0] = CAM_F_CIM_APB_RST;
			field_index[1] = CAM_F_CIM_RST;
			break;
		case (u32)DVP_RST:
			reg_index = CAM_SOFT_RSTN1;
			field_index[0] = CAM_F_DVP_RST;
			break;
		case (u32)MIPI_TX1_RST:
			reg_index = CAM_SOFT_RSTN1;
			field_index[0] = CAM_F_DEVICE1_RST;
			break;
		case (u32)MIPI_TX0_RST:
			reg_index = CAM_SOFT_RSTN1;
			field_index[0] = CAM_F_DEVICE0_RST;
			break;
		case (u32)MIPI_RX3_RST:
			reg_index = CAM_SOFT_RSTN1;
			field_index[0] = CAM_F_HOST3_RST;
			break;
		case (u32)MIPI_RX2_RST:
			reg_index = CAM_SOFT_RSTN1;
			field_index[0] = CAM_F_HOST2_RST;
			break;
		case (u32)MIPI_RX1_RST:
			reg_index = CAM_SOFT_RSTN1;
			field_index[0] = CAM_F_HOST1_RST;
			break;
		case (u32)MIPI_RX0_RST:
			reg_index = CAM_SOFT_RSTN1;
			field_index[0] = CAM_F_HOST0_RST;
			break;
		case (u32)STL_RST:
			reg_index = CAM_SOFT_RSTN2;
			field_index[0] = CAM_F_STH_PRST;
			field_index[1] = CAM_F_STH_VRST;
			break;
		case (u32)GDC_RST:
			reg_index = CAM_SOFT_RSTN2;
			field_index[0] = CAM_F_GDC_PRST;
			field_index[1] = CAM_F_GDC_VRST;
			break;
		case (u32)PYM2_RST:
			reg_index = CAM_SOFT_RSTN2;
			field_index[0] = CAM_F_PYM2_PRST;
			field_index[1] = CAM_F_PYM2_VRST;
			break;
		case (u32)PYM1_RST:
			reg_index = CAM_SOFT_RSTN2;
			field_index[0] = CAM_F_PYM1_PRST;
			field_index[1] = CAM_F_PYM1_VRST;
			break;
		case (u32)ISP1_RST:
			reg_index = CAM_SOFT_RSTN2;
			field_index[0] = CAM_F_ISP1_PRST;
			field_index[1] = CAM_F_ISP1_VRST;
			field_index[2] = CAM_F_ISP1_ARST;
			break;
		case (u32)IPE1_RST:
			reg_index = CAM_SOFT_RSTN2;
			field_index[0] = CAM_F_IPE1_PRST;
			break;
		case (u32)PYM0_RST:
			reg_index = CAM_SOFT_RSTN2;
			field_index[0] = CAM_F_PYM0_PRST;
			field_index[1] = CAM_F_PYM0_VRST;
			break;
		case (u32)ISP0_RST:
			reg_index = CAM_SOFT_RSTN2;
			field_index[0] = CAM_F_ISP0_PRST;
			field_index[1] = CAM_F_ISP0_VRST;
			field_index[2] = CAM_F_ISP0_ARST;
			break;
		case (u32)IPE0_RST:
			reg_index = CAM_SOFT_RSTN2;
			field_index[0] = CAM_F_IPE0_PRST;
			break;
		default:
			vio_err("wrong reset module(%d)\n", module);
			return -EFAULT;
	}

	for (i = 0; i < 3u; i++) {
		if (field_index[i] >= 0) {
			if (enable & (u32)1u << i)
				vio_hw_set_field(base_reg, &camsys_regs[reg_index],
					&camsys_fields[field_index[i]], 1);
			else
				vio_hw_set_field(base_reg, &camsys_regs[reg_index],
					&camsys_fields[field_index[i]], 0);
		}
	}

	vio_dbg("%s module %d enable 0x%x\n", __func__, module, enable);
	return ret;
}

/* code review E1:register set operation, so no return */
void ipe_path_cim_pym(void __iomem *base_reg, u32 enable)
{
	vio_hw_set_field(base_reg, &camsys_regs[IPE_IPE_SEL],
		&camsys_fields[IPE_F_CIM_PYM], enable);
}

/* code review E1:register set operation, so no return */
u32 ipe_get_reg_value(void __iomem *base_reg)
{
	return vio_hw_get_reg(base_reg, &camsys_regs[IPE_IPE_SEL]);
}

/* code review E1:register set operation, so no return */
void ipe_set_reg_value(void __iomem *base_reg, u32 value)
{
	vio_hw_set_reg(base_reg, &camsys_regs[IPE_IPE_SEL], value);
}

/* code review E1: internal function and just assignment logic, so no need return */
void camsys_set_module_id(u16 module_id, u16 event_id)
{
	u32 i = 0;

	for(i = 0; i < (u32)NUM_OF_IPE_REG; i++) {
		camsys_regs[i].module_id = (u32)module_id << 16 | event_id;
	}
}

/* code review E1:register set operation, so no return */
void camsys_set_parity_inject_value(void __iomem *base_reg, u32 value)
{
	camsys_regs[CAM_STH_ADDR_CFG_F].attr = FUSA_RW;
	vio_hw_set_reg(base_reg, &camsys_regs[CAM_STH_ADDR_CFG_F], value);
	camsys_regs[CAM_STH_ADDR_CFG_F].attr = RW;
}

u32 camsys_get_parity_inject_value(void __iomem *base_reg)
{
	return vio_hw_get_reg(base_reg, &camsys_regs[CAM_STH_ADDR_CFG_F]);
}

/* code review E1:register set operation, so no return */
void ipe_set_parity_inject_value(void __iomem *base_reg, u32 value)
{
	camsys_regs[IPE_PYM_ADDR_CFG_F].attr = FUSA_RW;
	vio_hw_set_reg(base_reg, &camsys_regs[IPE_PYM_ADDR_CFG_F], value);
	camsys_regs[IPE_PYM_ADDR_CFG_F].attr = RW;
}

u32 ipe_get_parity_inject_value(void __iomem *base_reg)
{
	return vio_hw_get_reg(base_reg, &camsys_regs[IPE_PYM_ADDR_CFG_F]);
}

/* code review E1:register dump operation, so no return */
void ipe_hw_dump(void __iomem *base_reg)
{
	vio_hw_dump_regs(base_reg, &camsys_regs[IPE_IPE_SEL],
			NUM_OF_IPE_REG - IPE_IPE_SEL - 1);
}

/* code review E1:register dump operation, so no return */
void camsys_hw_dump(void __iomem *base_reg)
{
	vio_hw_dump_regs(base_reg, &camsys_regs[CAM_MODULE_ENABLE], (u32)IPE_IPE_SEL);
}

s32 camsys_check_default_value(void __iomem *base_reg, u32 hw_id)
{
	s32 ret = 0;
	s32 i = 0;
	s32 start = 0, end = 0;
	u32 default_value = 0;

	if (hw_id == DEV_HW_CAM_SYS) {
		start = (s32)CAM_MEM_CFG00;
		end = (s32)CAM_TX_DEBUG + 1;
	} else {
		start = (s32)IPE_IPE_SEL;
		end = (s32)NUM_OF_IPE_REG;
	}

	for(i = start; i < end; i++) {
		if (camsys_regs[i].attr != RW)
			continue;
		default_value = vio_hw_get_reg(base_reg, &camsys_regs[i]);
		if (default_value != camsys_regs[i].default_val)
			break;
	}

	if (i != end) {
		ret = -1;
		vio_info("%s %s\n", __func__, camsys_regs[i].reg_name);
	}

	return ret;
}
