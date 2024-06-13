/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __CSI_REGS_H__
#define __CSI_REGS_H__

#define CSI_VERSION             0x00
#define CSI_N_LANES             0x04
#define CSI_RESETN              0x08
#define CSI_INT_ST_MAIN         0x0c
#define CSI_DATA_IDS_1          0x10
#define CSI_DATA_IDS_2          0x14
#define CSI_PHY_CFG             0x18
#define CSI_PHY_MODE            0x1c
#define CSI_INT_ST_AP_MAIN      0x2c
#define CSI_DATA_IDS_VC_1       0x30
#define CSI_DATA_IDS_VC_2       0x34
#define CSI_PHY_SHUTDOWNZ       0x40
#define CSI_PHY_RSTZ            0x44
#define CSI_PHY_RX              0x48
#define CSI_PHY_STOPSTATE       0x4c
#define CSI_PHY_TEST_CTRL0      0x50
#define CSI_PHY_TEST_CTRL1      0x54
#define PPI_PG_PATTERN_VRES     0x60
#define PPI_PG_PATTERN_HRES     0x64
#define PPI_PG_CONFIG           0x68
#define PPI_PG_ENABLE           0x6c
#define PPI_PG_STATUS           0x70
#define CSI_IPI_MODE            0x80
#define CSI_IPI_VCID            0x84
#define CSI_IPI_DATA_TYPE       0x88
#define CSI_IPI_MEM_FLUSH       0x8c
#define CSI_IPI_HSA_TIME        0x90
#define CSI_IPI_HBP_TIME        0x94
#define CSI_IPI_HSD_TIME        0x98
#define CSI_IPI_HLINE_TIME      0x9c
#define CSI_IPI_SOFTRSTN        0xa0
#define CSI_IPI_ADV_FEATURES    0xac
#define CSI_IPI_VSA_LINES       0xb0
#define CSI_IPI_VBP_LINES       0xb4
#define CSI_IPI_VFP_LINES       0xb8
#define CSI_IPI_VACTIVE_LINES   0xbc
#define CSI_VC_EXTENSION        0xc8
#define CSI_INT_ST_PHY_FATAL    0xe0
#define CSI_INT_MSK_PHY_FATAL   0xe4
#define CSI_INT_FORCE_PHY_FATAL 0xe8
#define CSI_INT_ST_PKT_FATAL    0xf0
#define CSI_INT_MSK_PKT_FATAL   0xf4
#define CSI_INT_FORCE_PKT_FATAL 0xf8
#define CSI_INT_ST_PHY          0x110
#define CSI_INT_MSK_PHY         0x114
#define CSI_INT_FORCE_PHY       0x118
#define CSI_INT_ST_LINE         0x130
#define CSI_INT_MSK_LINE        0x134
#define CSI_INT_FORCE_LINE      0x138

#define CSI_INT_ST_IPI_FATAL     0x140
#define CSI_INT_MSK_IPI_FATAL    0x144
#define CSI_INT_FORCE_IPI_FATAL  0x148
#define CSI_INT_ST_IPI2_FATAL    0x150
#define CSI_INT_MSK_IPI2_FATAL   0x154
#define CSI_INT_FORCE_IPI2_FATAL 0x158
#define CSI_INT_ST_IPI3_FATAL    0x160
#define CSI_INT_MSK_IPI3_FATAL   0x164
#define CSI_INT_FORCE_IPI3_FATAL 0x168
#define CSI_INT_ST_IPI4_FATAL    0x170
#define CSI_INT_MSK_IPI4_FATAL   0x174
#define CSI_INT_FORCE_IPI4_FATAL 0x178
#define CSI_IPI2_MODE            0x200
#define CSI_IPI2_VCID            0x204
#define CSI_IPI2_DATA_TYPE       0x208
#define CSI_IPI2_HSA_TIME        0x210
#define CSI_IPI2_HBP_TIME        0x214
#define CSI_IPI2_HSD_TIME        0x218
#define CSI_INT_ST_BNDRY_FRAME_FATAL    0x280
#define CSI_INT_MSK_BNDRY_FRAME_FATAL   0x284
#define CSI_INT_FORCE_BNDRY_FRAME_FATAL 0x288
#define CSI_INT_ST_SEQ_FRAME_FATAL      0x290
#define CSI_INT_MSK_SEQ_FRAME_FATAL     0x294
#define CSI_INT_FORCE_SEQ_FRAME_FATAL   0x298
#define CSI_INT_ST_CRC_FRAME_FATAL      0x2a0
#define CSI_INT_MSK_CRC_FRAME_FATAL     0x2a4
#define CSI_INT_FORCE_CRC_FRAME_FATAL   0x2a8
#define CSI_INT_ST_PLD_CRC_FATAL        0x2b0
#define CSI_INT_MSK_PLD_CRC_FATAL       0x2b4
#define CSI_INT_FORCE_PLD_CRC_FATAL     0x2b8
#define CSI_INT_ST_DATA_ID              0x2c0
#define CSI_INT_MSK_DATA_ID             0x2c4
#define CSI_INT_FORCE_DATA_ID           0x2c8
#define CSI_INT_ST_ECC_CORRECTED        0x2d0
#define CSI_INT_MSK_ECC_CORRECTED       0x2d4
#define CSI_INT_FORCE_ECC_CORRECTED     0x2d8

#define CSI_SCRAMBLING                  0x300
#define CSI_INT_ST_IPI5_FATAL           0x4e0
#define CSI_INT_MSK_IPI5_FATAL          0x4e4
#define CSI_INT_FORCE_IPI5_FATAL        0x4e8
#define CSI_INT_ST_IPI6_FATAL           0x4f0
#define CSI_INT_MSK_IPI6_FATAL          0x4f4
#define CSI_INT_ST_IPI7_FATAL           0x500
#define CSI_INT_MSK_IPI7_FATAL          0x504
#define CSI_INT_ST_IPI8_FATAL           0x510
#define CSI_INT_MSK_IPI8_FATAL          0x514

#define CSI_INT_ST_IPI1_4_FATAL(__n)    (CSI_INT_ST_IPI_FATAL + ((__n) - 1) * 0x10)
#define CSI_INT_MSK_IPI1_4_FATAL(__n)   (CSI_INT_MSK_IPI_FATAL + ((__n) - 1) * 0x10)
#define CSI_INT_FORCE_IPI1_4_FATAL(__n) (CSI_INT_FORCE_IPI_FATAL + ((__n) - 1) * 0x10)
#define CSI_INT_ST_IPI5_8_FATAL(__n)    (CSI_INT_ST_IPI5_FATAL + ((__n) - 5) * 0x10)
#define CSI_INT_MSK_IPI5_8_FATAL(__n)   (CSI_INT_MSK_IPI5_FATAL + ((__n) - 5) * 0x10)
#define CSI_INT_FORCE_IPI5_8_FATAL(__n) (CSI_INT_FORCE_IPI5_FATAL + ((__n) - 5) * 0x10)
#define CSI_INT_ST_IPI_OVERFLOW         (0x22)

#define CSI_INT_MAIN_ST_PHY_FATAL         BIT(0)
#define CSI_INT_MAIN_ST_PKT_FATAL         BIT(1)
#define CSI_INT_MAIN_ST_BNDRY_FRAME_FATAL BIT(2)
#define CSI_INT_MAIN_ST_SEQ_FRAME_FATAL   BIT(3)
#define CSI_INT_MAIN_ST_CRC_FRAME_FATAL   BIT(4)
#define CSI_INT_MAIN_ST_PLD_CRC_FATAL     BIT(5)
#define CSI_INT_MAIN_ST_DATA_ID           BIT(6)
#define CSI_INT_MAIN_ST_ECC_CORRECTED     BIT(7)
#define CSI_INT_MAIN_ST_PHY               BIT(16)
#define CSI_INT_MAIN_ST_LINE              BIT(17)
#define CSI_INT_MAIN_ST_IPI_FATAL         BIT(18)
#define CSI_INT_MAIN_ST_IPI1_8_FATAL(n)   BIT(18 + ((n) - 1))

#define CSI_IPIn_RSTN(n)                (1 << ((n) * 4))
#define CSI_IPI_ALL_RELEASE_RSTN        (0x1111)
#define CSI_IPI1_BASE(__n)              (CSI_IPI_MODE)
#define CSI_IPI2_4BASE(__n)             ((CSI_IPI2_MODE) + ((__n) - 2) * 0x20)
#define CSI_IPI5_8BASE(__n)             (0x5c0 + ((__n) - 5) * 0x20)

#define CSI_IPIn_MODE(__base)           (__base)
#define CSI_IPIn_VCID(__base)           ((__base) + 4)
#define CSI_IPIn_DATA_TYPE(__base)      ((__base) + 8)
#define CSI_IPIn_MEM_FLUSH(__base)      ((__base) + 0x0c)
#define CSI_IPIn_HSA_TIME(__base)       ((__base) + 0x10)
#define CSI_IPIn_HBP_TIME(__base)       ((__base) + 0x14)
#define CSI_IPIn_HSD_TIME(__base)       ((__base) + 0x18)
#define CSI_IPIn_ADV_FEATURES(__base)   ((__base) == CSI_IPI_MODE ? CSI_IPI_ADV_FEATURES : (__base) + 0x1c)

#define CSI_HSATIME_DEFAULT             (4)
#define CSI_HSATIME_MIN                 (0x1)
#define CSI_HSATIME_MAX                 (0xfff)
#define CSI_HBPTIME_DEFAULT             (4)
#define CSI_HBPTIME_MIN                 (0x1)
#define CSI_HBPTIME_MAX                 (0xfff)
#define CSI_HSDTIME_DEFAULT             (0x5f4)
#define CSI_HSDTIME_MIN                 (0x4)
#define CSI_HSDTIME_MAX                 (0xfff)
#define CSI_HSD_CAL_MIN                 (0x6)
#define CSI_IPICLK_DEFAULT              (40000000)

#define CSI_PHY_RX_CLK_ACTIVE_HS        BIT(17)
#define CSI_PHY_STOPSTATE_CLK           BIT(16)
#define CSI_IPI_MEM_AUTO_FLUSH          BIT(8)
#define CSI_IPI_MEM_MANUAL_FLUSH        BIT(1)

#define CSI_IDI_DATATYPE(n, dt)         (((dt) & 0x3f) << ((n) * 8))
#define CSI_IDI_DATATYPE_MASK(n)        CSI_IDI_DATATYPE((n), 0x3f)
#define CSI_IDI_VC(n, vc)               (((vc) & 0x3) << ((n) * 8))
#define CSI_IDI_VC_MASK(n)              (0x1f << ((n) * 8))

#define CSI_TPG_ENABLE                  BIT(0)
#define CSI_TPG_PKT2PKT_TIME(n)         ((n) << 16)

#define CSI_PHY_LANE_STOP_ALL           (0x0f)
#define CSI_PHY_LANE_STOP(l)            ((1 << (l)) - 1)

union phy_test_ctl {
	struct {
		u32 phy_testclr : 1;
		u32 phy_testclk : 1;
		u32 reserved : 30;
	} c0;
	struct {
		u32 phy_testdin : 8;
		u32 phy_testdout : 8;
		u32 phy_testen : 1;
		u32 reserved : 15;
	} c1;
	u32 value;
};

union ipi_data_type {
	struct {
		u32 ipi_data_type: 6;
		u32 reserved0: 2;
		u32 embedded_data: 1;
		u32 reserved1: 23;
	};
	u32 value;
};

union ipi_mode {
	struct {
		u32 ipi_mode_ctrl: 1;
		u32 reserved0: 7;
		u32 ipi_color16 : 1;
		u32 reserved1: 7;
		u32 ipi_cut_through: 1;
		u32 reserved2: 7;
		u32 ipi_enable: 1;
		u32 reserved3: 7;
	};
	u32 value;
};

union csi_ppi_pg_config {
	struct {
		u32 pattern_horizontal : 1;
		u32 reserved0 : 7;
		u32 data_type : 6;
		u32 vc : 2;
		u32 vcx_01 : 2;
		u32 vcx_2 : 1;
		u32 reserved1 : 13;
	};
	u32 value;
};

union csi_main_status {
	struct {
		u32 phy_fatal : 1;
		u32 pkt_fatal : 1;
		u32 bndry_frame_fatal : 1;
		u32 seq_frame_fatal : 1;
		u32 crc_frame_fatal : 1;
		u32 pld_crc_fatal : 1;
		u32 data_id : 1;
		u32 ecc_corrected : 1;
		u32 reserved : 8;
		u32 phy_status : 1;
		u32 line_status : 1;
		u32 ipi_fatal : 1;
		u32 ipi2_fatal : 1;
		u32 ipi3_fatal : 1;
		u32 ipi4_fatal : 1;
		u32 ipi5_fatal : 1;
		u32 ipi6_fatal : 1;
		u32 ipi7_fatal : 1;
		u32 ipi8_fatal : 1;
	};
	u32 value;
};

#endif /* __CSI_REGS_H__*/
