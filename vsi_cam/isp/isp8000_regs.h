/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ISP8000_REGS_H_
#define _ISP8000_REGS_H_

/* IRQ address offset */
#define ISP_IMSC    (0x000005bc)
#define ISP_RIS     (0x000005c0)
#define ISP_MIS     (0x000005c4)
#define ISP_ICR     (0x000005c8)

#define MIV2_CTRL   (0x00001300)
#define MIV2_IMSC   (0x000016c0)
#define MIV2_MIS    (0x000016d0)
#define MIV2_IMSC1  (0x000016c4)
#define MIV2_MIS1   (0x000016d4)
#define MIV2_ICR    (0x000016d8)
#define MIV2_ICR1   (0x000016dc)
#define MIV2_RIS1   (0x000016e4)
#define MIV2_IMSC2  (0x000016e8)
#define MIV2_MIS2   (0x000016f0)
#define MIV2_ICR2   (0x000016f4)
#define MIV2_IMSC3  (0x000056d0)
#define MIV2_MIS3   (0x000056d8)
#define MIV2_ICR3   (0x000056dc)
#define MIV2_RIS3   (0x000056e0)
#define MI_MIS_HDR1 (0x000072c8)
#define MI_ICR_HDR1 (0x000072cc)

#define ISP_FE_MIS  (0x00003d74)
#define ISP_FE_ICR  (0x00003d78)
#define ISP_FE_CTL  (0x00003d60)

/* MI MP buffer related */
#define MI_CTRL         (0x00001300)
#define MI_MP_CTRL      (0x00001310)
#define MI_MP_Y_ADDR    (0x00001324)
#define MI_MP_Y_SIZE    (0x00001328)
#define MI_MP_Y_OFFS    (0x0000132c)
#define MI_MP_CB_ADDR   (0x00001340)
#define MI_MP_CB_SIZE   (0x00001344)
#define MI_MP_CB_OFFS   (0x00001348)
#define MI_MP_CR_ADDR   (0x0000134c)
#define MI_MP_CR_SIZE   (0x00001350)
#define MI_MP_CR_OFFS   (0x00001354)

/* MCM buffer related */
#define MI_MCMn_RAW_BASE(n)         \
({ \
	typeof(n) _n = (n); \
	(_n < 2 ? (_n * 0x2c + 0x1614) : (_n * 0x2c + 0x4fb4)); \
})
#define MI_MCMn_RAW_ADDR(__base)    ((__base) + 0x00)
#define MI_MCMn_RAW_SIZE(__base)    ((__base) + 0x04)
#define MI_MCMn_RAW_OFFS(__base)    ((__base) + 0x08)
#define MI_MCM_RDMA_START           (0x0000166c)
#define MI_MCM_CTRL                 (0x00001600)
#define MI_MCM_G2_CTRL              (0x00005000)

/* IRQ register MASK */
#define MIV2_CTRL_MCM_RAW_RDMA_START_MASK (0x00008000)

#define MIV2_MIS_MCM_RAW_RADY_MASK (0x01000000)
#define MIV2_MIS_MCM_RAW1_FRAME_END_MASK (0x00000080)
#define MIV2_MIS_MCM_RAW0_FRAME_END_MASK (0x00000040)
#define MIV2_MIS_MCM_RAW_FRAME_END_MASK (0x000000c0)
#define MIV2_MIS_MCM_RAW1_BUF_FULL_MASK (0x00200000)
#define MIV2_MIS_MCM_RAW0_BUF_FULL_MASK (0x00100000)
#define MIV2_MIS_FRAME_END_MASK (0x0000003f)
#define MIV2_MIS_JPD_FRAME_END_MASK (0x00000004)

#define MIV2_MIS2_HDR_RDMA_READY_MASK (0x20000700)
#define MIV2_MIS2_HDR_FRAME_END_MASK (0x0800001c)

#define MIV2_MIS3_MCM_G2RAW1_FRAME_END_MASK (0x00008000)
#define MIV2_MIS3_MCM_G2RAW0_FRAME_END_MASK (0x00004000)
#define MIV2_MIS3_MCM_G2RAW1_BUF_FULL_MASK (0x00080000)
#define MIV2_MIS3_MCM_G2RAW0_BUF_FULL_MASK (0x00040000)

#define ISP_FE_CFG_SEL_MASK (0x00000001)
#define ISP_FE_CFG_SEL_SHIFT (0)
#define ISP_FE_AHB_WRITE_MASK (0x00000002)
#define ISP_FE_AHB_WRITE_SHIFT (1)

/* n = 0, 1 for long, short exposure path */
#define MI_HDR_RAW_ADDR(n) (0x00005740 + (n) * 0x40)
#define MI_HDR_RAW_SIZE(n) (0x00005744 + (n) * 0x40)
#define MI_HDR_RAW_OFFS(n) (0x00005748 + (n) * 0x40)
#define MI_HDR_DMA_ADDR(n) (0x00005820 + (n) * 0x30)

#define MP_RAW_BASE_AD_MASK (0xFFFFFFF0)
#define MP_RAW_SIZE_MASK (0x1FFFFFF0)

enum isp_fe_cfg_sel {
	ISP_FE_SEL_AHBBUF = 0,
	ISP_FE_SEL_CMDBUF,
};

enum isp_fe_ahb_wr_en {
	ISP_FE_AHB_WR_DISABLE = 0,
	ISP_FE_AHB_WR_ENABLE,
};

#endif /* _ISP8000_REGS_H_ */
