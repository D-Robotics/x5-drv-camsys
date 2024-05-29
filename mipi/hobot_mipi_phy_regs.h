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
 * @file hobot_mipi_phy_regs.h
 *
 * @NO{S10E03C02}
 * @ASIL{B}
 */

#ifndef __HOBOT_MIPI_DPHY_REGS_H__
#define __HOBOT_MIPI_DPHY_REGS_H__ /* PRQA S 0603 */ /* header file macro */

/*************************************************
*  MIPI dphy register offset
**************************************************/
/* ips sw regs for mipi of x2 */
#define REG_X2IPS_MIPI_DEV_PLL_CTRL1        (0xE0)
#define REG_X2IPS_MIPI_DEV_PLL_CTRL2        (0xE4)
#define REG_X2IPS_MIPI_HOST_PLL_CTRL1       (0xE8)
#define REG_X2IPS_MIPI_HOST_PLL_CTRL2       (0xEC)
#define X2IPS_MIPI_PLL_SEL_CLR_OFFS         (20)
#define X2IPS_MIPI_PLL_SEL_CLR_MASK         (0x3U)

#define REG_X2IPS_MIPI_CTRL                 (0xF0)
#define X2IPS_MIPI_BPASS_GEN_DLY_OFFS       (4)
#define X2IPS_MIPI_BPASS_GEN_DLY_MASK       (0x3fU)
#define X2IPS_MIPI_BPASS_GEN_HSYNC_OFFS     (1)
#define X2IPS_MIPI_BPASS_GEN_HSYNC_MASK     (0x1U)
#define X2IPS_MIPI_DEV_SHADOW_CLR_OFFS      (0)
#define X2IPS_MIPI_DEV_SHADOW_CLR_MASK      (0x1U)

#define REG_X2IPS_MIPI_FREQRANGE            (0xF4)
#define X2IPS_MIPI_DEV_CFGCLK_FRANGE_OFFS   (24)
#define X2IPS_MIPI_DEV_CFGCLK_FRANGE_MASK   (0xffU)
#define X2IPS_MIPI_DEV_HS_FRANGE_OFFS       (16)
#define X2IPS_MIPI_DEV_HS_FRANGE_MASK       (0x7fU)
#define X2IPS_MIPI_HOST_CFGCLK_FRANGE_OFFS  (8)
#define X2IPS_MIPI_HOST_CFGCLK_FRANGE_MASK  (0xffU)
#define X2IPS_MIPI_HOST_HS_FRANGE_OFFS      (0)
#define X2IPS_MIPI_HOST_HS_FRANGE_MASK      (0x7fU)

/* vio sw regs for mipi of x3 */
#define REG_X3VIO_MIPI_DEV_PLL_CTRL1        (0x80)
#define REG_X3VIO_MIPI_DEV_PLL_CTRL2        (0x84)

#define REG_X3VIO_MIPI_DEV_CTRL             (0x88)
#define X3VIO_MIPI_BPASS_GEN_DLY_OFFS       (4)
#define X3VIO_MIPI_BPASS_GEN_DLY_MASK       (0x3fU)
#define X3VIO_MIPI_BPASS_GEN_HSYNC_OFFS     (1)
#define X3VIO_MIPI_BPASS_GEN_HSYNC_MASK     (0x1U)
#define X3VIO_MIPI_DEV_SHADOW_CLR_OFFS      (0)
#define X3VIO_MIPI_DEV_SHADOW_CLR_MASK      (0x1U)

#define REG_X3VIO_MIPI_DEV_FREQRANGE        (0x8C)
#define X3VIO_MIPI_DEV_CFGCLK_FRANGE_OFFS   (8)
#define X3VIO_MIPI_DEV_CFGCLK_FRANGE_MASK   (0xffU)
#define X3VIO_MIPI_DEV_HS_FRANGE_OFFS       (0)
#define X3VIO_MIPI_DEV_HS_FRANGE_MASK       (0x7fU)

#define REG_X3VIO_MIPI_RX0_PLL_CTRL1        (0xA0)
#define REG_X3VIO_MIPI_RX0_PLL_CTRL2        (0xA4)
#define REG_X3VIO_MIPI_RX1_PLL_CTRL1        (0xA8)
#define REG_X3VIO_MIPI_RX1_PLL_CTRL2        (0xAC)
#define REG_X3VIO_MIPI_RX2_PLL_CTRL1        (0xB0)
#define REG_X3VIO_MIPI_RX2_PLL_CTRL2        (0xB4)
#define REG_X3VIO_MIPI_RX3_PLL_CTRL1        (0xB8)
#define REG_X3VIO_MIPI_RX3_PLL_CTRL2        (0xBC)
#define X3VIO_MIPI_PLL_SEL_CLR_OFFS         (20)
#define X3VIO_MIPI_PLL_SEL_CLR_MASK         (0x3U)

#define REG_X3VIO_MIPI_RX0                  (0xC0)
#define REG_X3VIO_MIPI_FREQRANGE_RX0        (0xC4)
#define REG_X3VIO_MIPI_RX1                  (0xC8)
#define REG_X3VIO_MIPI_FREQRANGE_RX1        (0xCC)
#define REG_X3VIO_MIPI_RX2                  (0xD0)
#define REG_X3VIO_MIPI_FREQRANGE_RX2        (0xD4)
#define REG_X3VIO_MIPI_RX3                  (0xD8)
#define REG_X3VIO_MIPI_FREQRANGE_RX3        (0xDC)
#define X3VIO_MIPI_RXN_CFGCLK_FRANGE_OFFS   (8)
#define X3VIO_MIPI_RXN_CFGCLK_FRANGE_MASK   (0xffU)
#define X3VIO_MIPI_RXN_HS_FRANGE_OFFS       (0)
#define X3VIO_MIPI_RXN_HS_FRANGE_MASK       (0x7fU)

#define REG_X3VIO_MIPI_TX_DPHY_CTRL         (0xE0)
#define X3VIO_MIPI_TX_DPHY_SEL_OFFS         (0)
#define X3VIO_MIPI_TX_DPHY_SEL_MASK         (0x1U)

#define REG_X3VIO_MIPI_RX_DPHY_CTRL         (0xE4)
#define X3VIO_MIPI_RX_DPHY_MODE02_OFFS      (0)
#define X3VIO_MIPI_RX_DPHY_MODE02_MASK      (0x1U)
#define X3VIO_MIPI_RX_DPHY_MODE13_OFFS      (1)
#define X3VIO_MIPI_RX_DPHY_MODE13_MASK      (0x1U)

/* cam subsys regs for mipi of j5 */
#define REG_J5SYS_MIPI_DEV0_CTRL0           (0x30)
#define REG_J5SYS_MIPI_DEV0_CTRL1           (0x34)
#define REG_J5SYS_MIPI_DEV0_CTRL2           (0x38)
#define REG_J5SYS_MIPI_DEV0_CTRL3           (0x3C)
#define REG_J5SYS_MIPI_DEV0_CTRL4           (0x40)
#define REG_J5SYS_MIPI_DEV1_CTRL0           (0x48)
#define REG_J5SYS_MIPI_DEV1_CTRL1           (0x4C)
#define REG_J5SYS_MIPI_DEV1_CTRL2           (0x50)
#define REG_J5SYS_MIPI_DEV1_CTRL3           (0x54)
#define REG_J5SYS_MIPI_DEV1_CTRL4           (0x58)
#define REG_J5SYS_MIPI_TX_DEBUG             (0x20C)

#define J5SYS_MIPI_DEVN_CFGCLK_FRANGE_OFFS  (7)
#define J5SYS_MIPI_DEVN_CFGCLK_FRANGE_MASK  (0x3fU)
#define J5SYS_MIPI_DEVN_HS_FRANGE_OFFS      (0)
#define J5SYS_MIPI_DEVN_HS_FRANGE_MASK      (0x7fU)

#define J5SYS_MIPI_DEV_SEL_CLR_OFFS         (20)
#define J5SYS_MIPI_DEV_SEL_CLR_MASK         (0x3U)
#define J5SYS_MIPI_DEV_SHADOW_CLR_OFFS      (16)
#define J5SYS_MIPI_DEV_SHADOW_CLR_MASK      (0x1U)
#define J5SYS_MIPI_DEV_SHADOW_CTL_OFFS      (17)
#define J5SYS_MIPI_DEV_SHADOW_CTL_MASK      (0x1U)

#define REG_J5SYS_MIPI_HOST0_CTRL           (0x20)
#define REG_J5SYS_MIPI_HOST1_CTRL           (0x24)
#define REG_J5SYS_MIPI_HOST2_CTRL           (0x28)
#define REG_J5SYS_MIPI_HOST3_CTRL           (0x2C)
#define REG_J5SYS_MIPI_RX_DEBUG             (0x208)
#define REG_J5SYS_MIPI_HOST0_CTRL1          (0x210)
#define REG_J5SYS_MIPI_HOST1_CTRL1          (0x214)
#define REG_J5SYS_MIPI_HOST2_CTRL1          (0x218)
#define REG_J5SYS_MIPI_HOST3_CTRL1          (0x21C)

#define J5SYS_MIPI_HOSTN_CFGCLK_FRANGE_OFFS (7)
#define J5SYS_MIPI_HOSTN_CFGCLK_FRANGE_MASK (0x3fU)
#define J5SYS_MIPI_HOSTN_HS_FRANGE_OFFS     (0)
#define J5SYS_MIPI_HOSTN_HS_FRANGE_MASK     (0x7fU)

/* cam ap erm regs for mipi of j5 */
#define REG_MIPI_ERM_FAULT0                 (0x0)
#define REG_MIPI_ERM_PASSWORD               (0x300)
#define REG_MIPI_ERM_FUSA_CTRL              (0x304)
#define REG_MIPI_ERM_MISSION_INTMASK0       (0x310)
#define REG_MIPI_ERM_MISSION_INTSTATUSSET0  (0x320)
#define REG_MIPI_ERM_MISSION_INTSTATUS0     (0x330)

#ifdef X5_CHIP
/* x5 mipi csi phy regs */
#define REG_X5SYS_MIPI_PHY_CFG0      (0x0)  //port0 && port1
#define REG_X5SYS_MIPI_PHY_CFG1      (0x8)  //port2 && port3
#define REG_X5SYS_MIPI_PHY_CFG2      (0x10)
#define REG_X5SYS_MIPI_PHY_TESTCODE0	(0x14)
#define REG_X5SYS_MIPI_PHY_TESTCODE1	(0x18)

#define REG_X5SYS_MIPI_PHY_PORT02_ENABLE_CLK_OFFS	(0)
#define REG_X5SYS_MIPI_PHY_PORT13_ENABLE_CLK_OFFS	(12)
#define REG_X5SYS_MIPI_PHY_PORT02_ENABLE_CLK_MASK	(0x1)
#define REG_X5SYS_MIPI_PHY_PORT13_ENABLE_CLK_MASK	(0x1)

#define REG_X5SYS_MIPI_PHY_PORT02_HS_FRANGE_OFFS	(5)
#define REG_X5SYS_MIPI_PHY_PORT13_HS_FRANGE_OFFS	(17)
#define REG_X5SYS_MIPI_PHY_PORT02_HS_FRANGE_MASK	(0x7fU)
#define REG_X5SYS_MIPI_PHY_PORT13_HS_FRANGE_MASK	(0x7fU)

#define REG_X5SYS_MIPI_PHY_PORT02_FORCE_RXMODE_OFFS	(2)
#define REG_X5SYS_MIPI_PHY_PORT13_FORCE_RXMODE_OFFS	(14)
#define REG_X5SYS_MIPI_PHY_PORT02_FORCE_RXMODE_MASK	(0x3)
#define REG_X5SYS_MIPI_PHY_PORT13_FORCE_RXMODE_MASK	(0x3)

#define REG_X5SYS_MIPI_PHY_CFG_CLK_OFFS                 (9)
#define REG_X5SYS_MIPI_PHY_CFG_CLK_MASK                 (0x1FU)

#define REG_X5SYS_MIPI_PHY_CTRL_OFFS                 (24)
#define REG_X5SYS_MIPI_PHY_CTRL_MASK                 (0x1U)

#define REG_X5SYS_MIPI_PHY_TEST_SEL_4L_0_OFFS		(7)
#define REG_X5SYS_MIPI_PHY_TEST_SEL_4L_1_OFFS           (8)
#define REG_X5SYS_MIPI_PHY_TEST_SEL_4L_MASK           (0x1U)

#endif

#define PASSWD_KEY							(0x95F2D303U)
#define DEFAULT_PASSWD_KEY					(0x0badbeefU)

#endif //__HOBOT_MIPI_DPHY_REGS_H__
