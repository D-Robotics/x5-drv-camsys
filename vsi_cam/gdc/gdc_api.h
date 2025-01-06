/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _GDC_API_H_
#define _GDC_API_H_

#include <linux/io.h>
#include <linux/types.h>

#define gdc_hw_write(base, offset, value) \
	__raw_writel(value, base + offset)

#define gdc_hw_read(base, offset) __raw_readl(base + offset)

#define GDC_DEFAULT_COLOR          0x008080

#define NUM_GDC_REG                38

#define GDC_HW_ID                          0x0000
#define GDC_HW_CONFIG_ADDR                 0x0010
#define GDC_HW_CONFIG_SIZE                 0x0014
#define GDC_HW_RDMA_IMG_WIDTH              0x0020
#define GDC_HW_RDMA_IMG_HEIGHT             0x0024
#define GDC_HW_RDMA0_IMG_ADDR              0x0028
#define GDC_HW_RDMA0_LINE_OFFSET           0x002c
#define GDC_HW_RDMA1_IMG_ADDR              0x0030
#define GDC_HW_RDMA1_LINE_OFFSET           0x0034
#define GDC_HW_RDMA2_IMG_ADDR              0x0038
#define GDC_HW_RDMA2_LINE_OFFSET           0x003c
#define GDC_HW_WDMA_IMG_WIDTH              0x0040
#define GDC_HW_WDMA_IMG_HEIGHT             0x0044
#define GDC_HW_WDMA0_IMG_ADDR              0x0048
#define GDC_HW_WDMA0_LINE_OFFSET           0x004c
#define GDC_HW_WDMA1_IMG_ADDR              0x0050
#define GDC_HW_WDMA1_LINE_OFFSET           0x0054
#define GDC_HW_WDMA2_IMG_ADDR              0x0058
#define GDC_HW_WDMA2_LINE_OFFSET           0x005c
#define GDC_HW_STATUS                      0x0060
#define GDC_HW_PROCESS_CONFIG              0x0064
#define GDC_HW_CAPABILITY_STATUS           0x0068
#define GDC_HW_DEFAULT_CH1                 0x0070
#define GDC_HW_DEFAULT_CH2                 0x0074
#define GDC_HW_DEFAULT_CH3                 0x0078
#define GDC_HW_DIAG_CFG_STALL_CNT0         0x0080
#define GDC_HW_DIAG_CFG_STALL_CNT1         0x0084
#define GDC_HW_DIAG_CFG_STALL_CNT2         0x0088
#define GDC_HW_DIAG_CFG_STALL_CNT3         0x008c
#define GDC_HW_DIAG_CFG_STALL_CNT4         0x0090
#define GDC_HW_DIAG_INT_READ_STALL_CNT     0x0094
#define GDC_HW_DIAG_INT_COORD_STALL_CNT    0x0098
#define GDC_HW_DIAG_INT_WRITE_WAIT_CNT     0x009c
#define GDC_HW_DIAG_WRT_WRITE_WAIT_CNT     0x00a0
#define GDC_HW_DIAG_INT_DUAL_CNT           0x00a4
#define GDC_HW_AXI_SETTING_CONFIG_READER   0x00a8
#define GDC_HW_AXI_SETTING_TILE_READER     0x00ac
#define GDC_HW_AXI_SETTING_TILE_WRITER     0x00b0

#define INT_GDC_BUSY                ((u32)1 << 0)
#define INT_GDC_ERROR               ((u32)1 << 1)
#define INT_GDC_CONF_ERROR          ((u32)1 << 8)
#define INT_GDC_USER_ABORT          ((u32)1 << 9)
#define INT_GDC_AXI_READER_ERROR    ((u32)1 << 10)
#define INT_GDC_AXI_WRITER_ERROR    ((u32)1 << 11)
#define INT_GDC_UNALIGNED_ACCESS    ((u32)1 << 12)
#define INT_GDC_INCOMPATIBLE_CONF   ((u32)1 << 13)

#endif /* _GDC_API_H_ */
