/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __SIF_REGS_H__
#define __SIF_REGS_H__

#include <linux/types.h>

#define SIF_EBD_PRE_TOTAL(x) ((x) & 0xffff)
#define SIF_EBD_POST_TOTAL(x) (((x) & 0xffff) << 16)
#define SIF_EBD_HSIZE_ALIGN (16)
#define SIF_EBD_HSIZE(x) (((x) & 0x1fff) - 1)
#define SIF_EBD_VSIZE(x) ((((x) & 0x1fff) - 1) << 16)
#define SIF_EBD_PRE_HSTRIDE(x) (((x) & 0xffff) - 1)
#define SIF_EBD_POST_HSTRIDE(x) ((((x) & 0xffff) - 1) << 16)
#define SIF_HSIZE_ERR(x) ((x) & 0x3fff)
#define SIF_VSIZE_ERR(x) (((x) >> 14) & 0x3fff)

#define SIF_DMA_CTL (0x00)
#define SIF_IMG_OUT_BLENTH (0x40)
#define SIF_DMA_FPS_CTRL (0x200)
#define SIF_IPI_RESET (0x104)
#define SIF_YUV_CONV_CTRL (0x204)
#define SIF_IPI_RAM_CTRL (0xec)
#define SIF_HDR_CTRL (0xc4)
#define SIF_HDR_EN   (0xc8)
#define SIF_ISP_CTRL (0xf0)
#define SIF_PPS_IRQ_EN (0x1dc)
#define SIF_PPS_IRQ_STATUS (0x1e0)
#define SIF_PPS_IRQ_CLR (0x1e4)

#define SIF_FMT_CLEAR_IPI_0 0xffffffd1
#define SIF_FMT_CLEAR_IPI_1 0xfffc3fff
#define SIF_FMT_CLEAR_IPI_2 0xff0fffff
#define SIF_FMT_CLEAR_IPI_3 0xc3ffffff

static inline u32 SIF_IPI_BADDR_Y(unsigned int n)
{
	return ((n) < 3 ? (0x04 + (n) * 0x14) : (0xcc));
}

static inline u32 SIF_IPI_BADDR_UV(unsigned int n)
{
	return ((n) < 3 ? (0x08 + (n) * 0x14) : (0xd0));
}

static inline u32 SIF_IPI_PIX_HSIZE(unsigned int n)
{
	return ((n) < 3 ? (0x0c + (n) * 0x14) : (0xd4));
}

static inline u32 SIF_IPI_PIX_VSIZE(unsigned int n)
{
	return ((n) < 3 ? (0x10 + (n) * 0x14) : (0xd8));
}

static inline u32 SIF_IPI_PIX_HSTRIDE(unsigned int n)
{
	return ((n) < 3 ? (0x14 + (n) * 0x14) : (0xdc));
}

static inline u32 SIF_IPI_IRQ_EN(unsigned int n)
{
	return ((n) < 3 ? (0x44 + (n) * 0x0c) : (0xe0));
}

static inline u32 SIF_IPI_IRQ_CLR(unsigned int n)
{
	return ((n) < 3 ? (0x48 + (n) * 0x0c) : (0xe4));
}

static inline u32 SIF_IPI_IRQ_STATUS(unsigned int n)
{
	return ((n) < 3 ? (0x4c + (n) * 0x0c) : (0xe8));
}

static inline u32 SIF_IPI_ERR_SIZE(unsigned int n)
{
	return (0xf4 + (n) * 0x4);
}

static inline u32 SIF_IPI_EBD_CTRL(unsigned int n)
{
	return (0x110 + (n) * 0x20);
}

static inline u32 SIF_IPI_EBD_TOTAL_BYTE(unsigned int n)
{
	return (0x114 + (n) * 0x20);
}

static inline u32 SIF_IPI_EBD_SIZE_PRE(unsigned int n)
{
	return (0x118 + (n) * 0x20);
}

static inline u32 SIF_IPI_EBD_SIZE_POST(unsigned int n)
{
	return (0x11c + (n) * 0x20);
}

static inline u32 SIF_IPI_EBD_HSTRIDE(unsigned int n)
{
	return (0x120 + (n) * 0x20);
}

static inline u32 SIF_IPI_EBD_BADDR_PRE(unsigned int n)
{
	return (0x124 + (n) * 0x20);
}

static inline u32 SIF_IPI_EBD_BADDR_POST(unsigned int n)
{
	return (0x128 + (n) * 0x20);
}

static inline u32 SIF_IPI_FRAME_ID_CTRL(unsigned int n)
{
	return (0x18c + (n) * 0x0c);
}

static inline u32 SIF_IPI_FRAME_ID_INITAL(unsigned int n)
{
	return (0x190 + (n) * 0x0c);
}

static inline u32 SIF_IPI_FRAME_ID(unsigned int n)
{
	return (0x194 + (n) * 0x0c);
}

static inline u32 SIF_IPI_TIMESTAMP_CTRL(unsigned int n)
{
	return (0x1bc + (n) * 0x04);
}

static inline u32 SIF_PPS_TIMESTAMP_CTRL(unsigned int n)
{
	return (0x1cc + (n) * 0x08);
}

static inline u32 SIF_PPS_TIMEOUT_INTVAL(unsigned int n)
{
	return (0x1d0 + (n) * 0x08);
}

static inline u32 SIF_IPI_TS_VSYNC_LO(unsigned int n)
{
	return (0x208 + (n) * 0x08);
}

static inline u32 SIF_IPI_TS_VSYNC_HI(unsigned int n)
{
	return (0x20c + (n) * 0x08);
}

static inline u32 SIF_IPI_TS_TRIG_LO(unsigned int n)
{
	return (0x210 + (n) * 0x08);
}

static inline u32 SIF_IPI_TS_TRIG_HI(unsigned int n)
{
	return (0x214 + (n) * 0x08);
}

static inline u32 SIF_PPS_TS_LO(unsigned int n)
{
	return (0x248 + (n) * 0x08);
}

static inline u32 SIF_PPS_TS_HI(unsigned int n)
{
	return (0x24c + (n) * 0x08);
}

#define SIF_IPI0_ERR_SIZE (0xf4)
#define SIF_IPI1_ERR_SIZE (0xf8)
#define SIF_BURST_LENGTH (0x3333)

#define SIF_IRQ_FS                 BIT(0)
#define SIF_IRQ_DONE               BIT(1)
#define SIF_IRQ_OF                 BIT(2)
#define SIF_IRQ_EBD_DMA_DONE       BIT(3)
#define SIF_BUF_ERROR_EBD_EN       BIT(4)
#define SIF_EBD_BUF_AFULL_IRQ_EN   BIT(5)
#define SIF_PIXEL_BUF_AFULL_IRQ_EN BIT(6)
#define SIF_IPI_FRAME_END_EN       BIT(7)
#define SIF_FRAME_SIZE_ERROR_EN    BIT(8)
#define SIF_IPI_HALT_INT_EN        BIT(9)

#define PPS1_TRIG_IRQ BIT(0)
#define PPS2_TRIG_IRQ BIT(2)

enum sif_ctrl_regval {
	SIF_ENABLE_IPI_0 = 1 << 0,
	SIF_FMT_NV12_IPI_0 = 0 << 1,
	SIF_FMT_YUV422_IPI_0 = 1 << 1,
	SIF_FMT_YUV422_RS_IPI_0 = 2 << 1,
	SIF_FMT_USER_IPI_0 = 3 << 1,
	SIF_FMT_RGB444_IPI_0 = 4 << 1,
	SIF_FMT_RGB555_IPI_0 = 5 << 1,
	SIF_FMT_RGB565_IPI_0 = 6 << 1,
	SIF_FMT_RGB888_IPI_0 = 7 << 1,
	SIF_WR_LIMIT_ENABLE = 1 << 4,
	SIF_FMT_RAW8_IPI_0 = (1 << 5) | (0 << 1),
	SIF_FMT_RAW10_IPI_0 = (1 << 5) | (1 << 1),
	SIF_FMT_RAW12_IPI_0 = (1 << 5) | (2 << 1),
	SIF_FMT_RAW14_IPI_0 = (1 << 5) | (3 << 1),
	SIF_FMT_RAW16_IPI_0 = (1 << 5) | (4 << 1),
	SIF_DMA_CONFIG_IPI_0 = 1 << 6,

	SIF_ENABLE_IPI_1 = 1 << 12,
	SIF_DMA_CONFIG_IPI_1 = 1 << 13,
	SIF_FMT_NV12_IPI_1 = 0 << 14,
	SIF_FMT_YUV422_IPI_1 = 1 << 14,
	SIF_FMT_YUV422_RS_IPI_1 = 2 << 14,
	SIF_FMT_USER_IPI_1 = 3 << 14,
	SIF_FMT_RGB444_IPI_1 = 4 << 14,
	SIF_FMT_RGB555_IPI_1 = 5 << 14,
	SIF_FMT_RGB565_IPI_1 = 6 << 14,
	SIF_FMT_RGB888_IPI_1 = 7 << 14,
	SIF_FMT_RAW8_IPI_1 = (1 << 17) | (0 << 14),
	SIF_FMT_RAW10_IPI_1 = (1 << 17) | (1 << 14),
	SIF_FMT_RAW12_IPI_1 = (1 << 17) | (2 << 14),
	SIF_FMT_RAW14_IPI_1 = (1 << 17) | (3 << 14),
	SIF_FMT_RAW16_IPI_1 = (1 << 17) | (4 << 14),

	SIF_ENABLE_IPI_2 = 1 << 18,
	SIF_DMA_CONFIG_IPI_2 = 1 << 19,
	SIF_FMT_NV12_IPI_2 = 0 << 20,
	SIF_FMT_YUV422_IPI_2 = 1 << 20,
	SIF_FMT_YUV422_RS_IPI_2 = 2 << 20,
	SIF_FMT_USER_IPI_2 = 3 << 20,
	SIF_FMT_RGB444_IPI_2 = 4 << 20,
	SIF_FMT_RGB555_IPI_2 = 5 << 20,
	SIF_FMT_RGB565_IPI_2 = 6 << 20,
	SIF_FMT_RGB888_IPI_2 = 7 << 20,
	SIF_FMT_RAW8_IPI_2 = (1 << 23) | (0 << 20),
	SIF_FMT_RAW10_IPI_2 = (1 << 23) | (1 << 20),
	SIF_FMT_RAW12_IPI_2 = (1 << 23) | (2 << 20),
	SIF_FMT_RAW14_IPI_2 = (1 << 23) | (3 << 20),
	SIF_FMT_RAW16_IPI_2 = (1 << 23) | (4 << 20),

	SIF_ENABLE_IPI_3 = 1 << 24,
	SIF_DMA_CONFIG_IPI_3 = 1 << 25,
	SIF_FMT_NV12_IPI_3 = 0 << 26,
	SIF_FMT_YUV422_IPI_3 = 1 << 26,
	SIF_FMT_YUV422_RS_IPI_3 = 2 << 26,
	SIF_FMT_USER_IPI_3 = 3 << 26,
	SIF_FMT_RGB444_IPI_3 = 4 << 26,
	SIF_FMT_RGB555_IPI_3 = 5 << 26,
	SIF_FMT_RGB565_IPI_3 = 6 << 26,
	SIF_FMT_RGB888_IPI_3 = 7 << 26,
	SIF_FMT_RAW8_IPI_3 = (1 << 29) | (0 << 26),
	SIF_FMT_RAW10_IPI_3 = (1 << 29) | (1 << 26),
	SIF_FMT_RAW12_IPI_3 = (1 << 29) | (2 << 26),
	SIF_FMT_RAW14_IPI_3 = (1 << 29) | (3 << 26),
	SIF_FMT_RAW16_IPI_3 = (1 << 29) | (4 << 26),
};

#define SIF_WR_LIMIT_OUTSTAND(x) (((x)-1) << 7)

static const int __maybe_unused SIF_FMT_CLEAR[] = { SIF_FMT_CLEAR_IPI_0,
						       SIF_FMT_CLEAR_IPI_1,
						       SIF_FMT_CLEAR_IPI_2,
						       SIF_FMT_CLEAR_IPI_3 };

static const int __maybe_unused SIF_FMT_NV12_IPI[] = { SIF_FMT_NV12_IPI_0,
						       SIF_FMT_NV12_IPI_1,
						       SIF_FMT_NV12_IPI_2,
						       SIF_FMT_NV12_IPI_3 };

static const int __maybe_unused SIF_FMT_YUY422SP_IPI[] = {
	SIF_FMT_YUV422_IPI_0, SIF_FMT_YUV422_IPI_1,
    SIF_FMT_YUV422_IPI_2, SIF_FMT_YUV422_IPI_3
};

static const int __maybe_unused SIF_FMT_YUYV_IPI[] = {
	SIF_FMT_YUV422_RS_IPI_0, SIF_FMT_YUV422_RS_IPI_1,
	SIF_FMT_YUV422_RS_IPI_2, SIF_FMT_YUV422_RS_IPI_3
};

static const int __maybe_unused SIF_FMT_RAW8_IPI[] = { SIF_FMT_RAW8_IPI_0,
						       SIF_FMT_RAW8_IPI_1,
						       SIF_FMT_RAW8_IPI_2,
						       SIF_FMT_RAW8_IPI_3 };

static const int __maybe_unused SIF_FMT_RAW10_IPI[] = { SIF_FMT_RAW10_IPI_0,
							SIF_FMT_RAW10_IPI_1,
							SIF_FMT_RAW10_IPI_2,
							SIF_FMT_RAW10_IPI_3 };

static const int __maybe_unused SIF_FMT_RAW12_IPI[] = { SIF_FMT_RAW12_IPI_0,
							SIF_FMT_RAW12_IPI_1,
							SIF_FMT_RAW12_IPI_2,
							SIF_FMT_RAW12_IPI_3 };

static const int __maybe_unused SIF_FMT_RGB888_IPI[] = { SIF_FMT_RGB888_IPI_0,
							 SIF_FMT_RGB888_IPI_1,
							 SIF_FMT_RGB888_IPI_2,
							 SIF_FMT_RGB888_IPI_3 };

static const int __maybe_unused SIF_DMA_CONFIG_IPI[] = { SIF_DMA_CONFIG_IPI_0,
							 SIF_DMA_CONFIG_IPI_1,
							 SIF_DMA_CONFIG_IPI_2,
							 SIF_DMA_CONFIG_IPI_3 };

static const int __maybe_unused SIF_ENABLE_IPI[] = {
	SIF_ENABLE_IPI_0, SIF_ENABLE_IPI_1, SIF_ENABLE_IPI_2, SIF_ENABLE_IPI_3
};

#endif /* __SIF_REGS_H__ */
