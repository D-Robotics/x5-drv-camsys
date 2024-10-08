/**
 * @file: hobot_gdc_hw_reg.h
 * @
 * @NO{S09E03C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */

#ifndef HOBOT_GDC_HW_REG_H
#define HOBOT_GDC_HW_REG_H

enum gdc_reg {
	GDC_ID,
	GDC_CONFIG_ADDR,
	GDC_CONFIG_SIZE,
	GDC_RDMA_IMG_WIDTH,
	GDC_RDMA_IMG_HEIGHT,
	GDC_RDMA0_IMG_ADDR,
	GDC_RDMA0_LINE_OFFSET,
	GDC_RDMA1_IMG_ADDR,
	GDC_RDMA1_LINE_OFFSET,
	GDC_RDMA2_IMG_ADDR,
	GDC_RDMA2_LINE_OFFSET,
	GDC_WDMA_IMG_WIDTH,
	GDC_WDMA_IMG_HEIGHT,
	GDC_WDMA0_IMG_ADDR,
	GDC_WDMA0_LINE_OFFSET,
	GDC_WDMA1_IMG_ADDR,
	GDC_WDMA1_LINE_OFFSET,
	GDC_WDMA2_IMG_ADDR,
	GDC_WDMA2_LINE_OFFSET,
	GDC_STATUS,
	GDC_PROCESS_CONFIG,
	GDC_CAPABILITY_STATUS,
	GDC_DEFAULT_CH1,
	GDC_DEFAULT_CH2,
	GDC_DEFAULT_CH3,
	GDC_DIAG_CFG_STALL_CNT0,
	GDC_DIAG_CFG_STALL_CNT1,
	GDC_DIAG_CFG_STALL_CNT2,
	GDC_DIAG_CFG_STALL_CNT3,
	GDC_DIAG_CFG_STALL_CNT4,
	GDC_DIAG_INT_READ_STALL_CNT,
	GDC_DIAG_INT_COORD_STALL_CNT,
	GDC_DIAG_INT_WRITE_WAIT_CNT,
	GDC_DIAG_WRT_WRITE_WAIT_CNT,
	GDC_DIAG_INT_DUAL_CNT,
	GDC_AXI_SETTING_CONFIG_READER,
	GDC_AXI_SETTING_TILE_READER,
	GDC_AXI_SETTING_TILE_WRITER,
	NUM_OF_GDC_REG,
};

enum gdc_reg_field{
	GDC_F_STOP_FLAG,
	GDC_F_START_FLAG,
	GDC_F_CONF_READER_RXACT_MAXOSTAND,
	GDC_F_CONF_READER_FIFO_WATERMARK,
	GDC_F_CONF_READER_MAX_ARLEN,
	GDC_F_TILE_READER_RXACT_MAXOSTAND,
	GDC_F_TILE_READER_FIFO_WATERMARK,
	GDC_F_TILE_READER_MAX_ARLEN,
	GDC_F_TILE_WRITER_RXACT_MAXOSTAND,
	GDC_F_TILE_WRITER_FIFO_WATERMARK,
	GDC_F_TILE_WRITER_MAX_ARLEN,
	GDC_F_IRQ_MASK,
	NUM_OF_GDC_FIELD,
};

#endif
