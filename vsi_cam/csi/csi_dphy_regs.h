/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __CSI_DPHY_REGS_H__
#define __CSI_DPHY_REGS_H__

#define CSI_DPHY_RX_SYS_7            (0x08)
#define CSI_DPHY_RX_SYSTIMERS_0      (0x7A)
#define CSI_DPHY_RX_STARTUP_OVR_0    (0xE0)
#define CSI_DPHY_RX_STARTUP_OVR_1    (0xE1)
#define CSI_DPHY_RX_STARTUP_OVR_2    (0xE2)
#define CSI_DPHY_RX_STARTUP_OVR_3    (0xE3)
#define CSI_DPHY_RX_STARTUP_OVR_4    (0xE4)
#define CSI_DPHY_RX_STARTUP_OVR_5    (0xE5)
#define CSI_DPHY_RX_STARTUP_OVR_17   (0xF1)
#define CSI_DPHY_RX_DUAL_PHY_0       (0x133)
#define CSI_DPHY_RX_CB_0             (0x1AA)
#define CSI_DPHY_RX_CB_2             (0x1AC)
#define CSI_DPHY_RX_CLKLANE_LANE_3   (0x304)
#define CSI_DPHY_RX_CLKLANE_LANE_4   (0x305)
#define CSI_DPHY_RX_CLKLANE_LANE_6   (0x307)
#define CSI_DPHY_RX_CLKLANE_LANE_7   (0x308)
#define CSI_DPHY_RX_LANE0_LANE_7     (0x508)
#define CSI_DPHY_RX_LANE1_LANE_7     (0x708)
#define CSI_DPHY_RX_LANE0_DDL_4      (0x60A)
#define CSI_DPHY_RX_LANE0_DDL_5      (0x60B)
#define CSI_DPHY_RX_LANE0_DDL_6      (0x60C)
#define CSI_DPHY_RX_LANE1_DDL_4      (0x80A)
#define CSI_DPHY_RX_LANE1_DDL_5      (0x80B)
#define CSI_DPHY_RX_LANE1_DDL_6      (0x80C)

#define CSI_DPHY_RX_LANE2_LANE7      (0x908)
#define CSI_DPHY_RX_LANE3_LANE7      (0xB08)

#define CSI_DPHY_OSC_FREQ_DEFAULT    (460U)
#define CSI_DPHY_FREQ_SKEWRATE_MBPS  (1500U)

#define CSI_DPHY_RX_CLK_SETTLE_EN    BIT(0)
#define CSI_DPHY_RX_CLK_SETTLE       BIT(4)
#define CSI_DPHY_RX_OSCFREQ_EN       BIT(0)
#define CSI_DPHY_RX_HS_SETTLE(s)     (BIT(7) | ((s) & 0x7f))

#define RX_SYS_7_SYSTEM_CONFIG       (0x38)
#define RX_CLKLANE_6_PULLLONG        (0x80)
#define RX_CB2_BIAS_ATB              (0x40)

union csi_phy_rx_cb0 {
	struct {
		u32 chop_clk_en : 1;
		u32 sel_chop_clk_ext : 1;
		u32 sel_v400_prog : 3;
		u32 sel_vrefcd_lprx : 2;
		u32 reserved : 25;
	};
	u32 value;
};

#endif /* __CSI_DPHY_REGS_H__*/
