/**
 * @file: hobot_gdc_hw_reg.c
 * @
 * @NO{S09E03C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */
#define pr_fmt(fmt)    "[GDC regs]:" fmt
#include "vio_hw_common_api.h"
#include "hobot_gdc_hw_reg.h"
#include "gdc_hw_api.h"

/**
 * Purpose: gdc register attribute
 * Value: fixed register name, address offset and WR/RD type
 * Range: gdc_hw_reg.c
 * Attention: NA
 */

static struct vio_reg_def gdc_regs[NUM_OF_GDC_REG]={
	{"GDC_ID",                          0x0000, RO, 0xF02D45F0},
	{"GDC_CONFIG_ADDR",                 0x0010, RW, 0x0},
	{"GDC_CONFIG_SIZE",                 0x0014, RW, 0x0},
	{"GDC_RDMA_IMG_WIDTH",              0x0020, RW, 0x0},
	{"GDC_RDMA_IMG_HEIGHT",             0x0024, RW, 0x0},
	{"GDC_RDMA0_IMG_ADDR",              0x0028, RW, 0x0},
	{"GDC_RDMA0_LINE_OFFSET",           0x002c, RW, 0x0},
	{"GDC_RDMA1_IMG_ADDR",              0x0030, RW, 0x0},
	{"GDC_RDMA1_LINE_OFFSET",           0x0034, RW, 0x0},
	{"GDC_RDMA2_IMG_ADDR",              0x0038, RW, 0x0},
	{"GDC_RDMA2_LINE_OFFSET",           0x003c, RW, 0x0},
	{"GDC_WDMA_IMG_WIDTH",              0x0040, RW, 0x0},
	{"GDC_WDMA_IMG_HEIGHT",             0x0044, RW, 0x0},
	{"GDC_WDMA0_IMG_ADDR",              0x0048, RW, 0x0},
	{"GDC_WDMA0_LINE_OFFSET",           0x004c, RW, 0x0},
	{"GDC_WDMA1_IMG_ADDR",              0x0050, RW, 0x0},
	{"GDC_WDMA1_LINE_OFFSET",           0x0054, RW, 0x0},
	{"GDC_WDMA2_IMG_ADDR",              0x0058, RW, 0x0},
	{"GDC_WDMA2_LINE_OFFSET",           0x005c, RW, 0x0},
	{"GDC_STATUS",                      0x0060, RO, 0x0},
	{"GDC_PROCESS_CONFIG",              0x0064, RW, 0x0},
	{"GDC_CAPABILITY_STATUS",           0x0068, RO, 0x9452013D},
	{"GDC_DEFAULT_CH1",                 0x0070, RW, 0x0},
	{"GDC_DEFAULT_CH2",                 0x0074, RW, 0x0},
	{"GDC_DEFAULT_CH3",                 0x0078, RW, 0x0},
	{"GDC_DIAG_CFG_STALL_CNT0",         0x0080, RO, 0x0},
	{"GDC_DIAG_CFG_STALL_CNT1",         0x0084, RO, 0x0},
	{"GDC_DIAG_CFG_STALL_CNT2",         0x0088, RO, 0x0},
	{"GDC_DIAG_CFG_STALL_CNT3",         0x008c, RO, 0x0},
	{"GDC_DIAG_CFG_STALL_CNT4",         0x0090, RO, 0x0},
	{"GDC_DIAG_INT_READ_STALL_CNT",     0x0094, RO, 0x0},
	{"GDC_DIAG_INT_COORD_STALL_CNT",    0x0098, RO, 0x0},
	{"GDC_DIAG_INT_WRITE_WAIT_CNT",     0x009c, RO, 0x0},
	{"GDC_DIAG_WRT_WRITE_WAIT_CNT",     0x00a0, RO, 0x0},
	{"GDC_DIAG_INT_DUAL_CNT",           0x00a4, RO, 0x0},
	{"GDC_AXI_SETTING_CONFIG_READER",   0x00a8, RW, 0x100F},
	{"GDC_AXI_SETTING_TILE_READER",     0x00ac, RW, 0x100F},
	{"GDC_AXI_SETTING_TILE_WRITER",     0x00b0, RW, 0x100F},
	{"GDC_INT_STATUS",					0x0200, W1C, 0X0},
	{"GDC_INT_MASK",				 	0x0204, RW, 0X0},
	{"GDC_FUSA_PWD",				 	0x0300, RW, 0x0},
	{"GDC_FUSA_INT_MASK",				0x0308, RW, 0x0},
	{"GDC_FUSA_INT_STATUS",				0x0310, W1C, 0x0},
};

/**
 * Purpose: gdc register bit field
 * Value: fixed bit attributes
 * Range: gdc_hw_reg.c
 * Attention: NA
 */
static const struct vio_field_def gdc_fields[NUM_OF_GDC_FIELD] = {
	{(u32)GDC_PROCESS_CONFIG,	(u32)GDC_F_STOP_FLAG,  1 , 1 , 0},
	{(u32)GDC_PROCESS_CONFIG,	(u32)GDC_F_START_FLAG, 0 , 1 , 0},
	{(u32)GDC_AXI_SETTING_CONFIG_READER,	(u32)GDC_F_CONF_READER_RXACT_MAXOSTAND,  16, 8 , 0},
	{(u32)GDC_AXI_SETTING_CONFIG_READER,	(u32)GDC_F_CONF_READER_FIFO_WATERMARK,   8 , 8 , 16},
	{(u32)GDC_AXI_SETTING_CONFIG_READER,	(u32)GDC_F_CONF_READER_MAX_ARLEN,   		0 , 4 , 15},
	{(u32)GDC_AXI_SETTING_TILE_READER,	(u32)GDC_F_TILE_READER_RXACT_MAXOSTAND,  16, 8 , 0},
	{(u32)GDC_AXI_SETTING_TILE_READER,	(u32)GDC_F_TILE_READER_FIFO_WATERMARK,   8 , 8 , 16},
	{(u32)GDC_AXI_SETTING_TILE_READER,	(u32)GDC_F_TILE_READER_MAX_ARLEN,   		0 , 4 , 15},
	{(u32)GDC_AXI_SETTING_TILE_WRITER,	(u32)GDC_F_TILE_WRITER_RXACT_MAXOSTAND,  16, 8 , 0},
	{(u32)GDC_AXI_SETTING_TILE_WRITER,	(u32)GDC_F_TILE_WRITER_FIFO_WATERMARK,   8 , 8 , 16},
	{(u32)GDC_AXI_SETTING_TILE_WRITER,	(u32)GDC_F_TILE_WRITER_MAX_ARLEN,   		0 , 4 , 15},
	{(u32)GDC_FUSA_INT_MASK,	(u32)GDC_F_IRQ_MASK,   		27 , 1 , 1},
};

/* code review E1:register set operation, so no return */
void gdc_set_config_addr(void __iomem *base_addr, u32 config_addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_CONFIG_ADDR], config_addr);
}

/* code review E1:register set operation, so no return */
void gdc_set_config_size(void __iomem *base_addr, u32 config_addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_CONFIG_SIZE], config_addr);
}

/* code review E1:register set operation, so no return */
void gdc_set_rdma_img_width(void __iomem *base_addr, u32 width)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA_IMG_WIDTH], width);
}

/* code review E1:register set operation, so no return */
void gdc_set_rdma_img_height(void __iomem *base_addr, u32 height)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA_IMG_HEIGHT], height);
}

/* code review E1:register set operation, so no return */
void gdc_set_wdma_img_width(void __iomem *base_addr, u32 width)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA_IMG_WIDTH], width);
}

/* code review E1:register set operation, so no return */
void gdc_set_wdma_img_height(void __iomem *base_addr, u32 height)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA_IMG_HEIGHT], height);
}

/* code review E1:register set operation, so no return */
void gdc_process_enable(void __iomem *base_addr, u8 enable)
{
	vio_hw_set_field(base_addr, &gdc_regs[GDC_PROCESS_CONFIG],
		&gdc_fields[GDC_F_START_FLAG], enable);
}

/* code review E1:register set operation, so no return */
void gdc_process_reset(void __iomem *base_addr, u8 enable)
{
	vio_hw_set_field(base_addr, &gdc_regs[GDC_PROCESS_CONFIG],
		&gdc_fields[GDC_F_STOP_FLAG], enable);
}

/* code review E1:register set operation, so no return */
void gdc_set_rdma0_img_addr(void __iomem *base_addr, u32 addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA0_IMG_ADDR], addr);
}

/* code review E1:register set operation, so no return */
void gdc_set_rdma1_img_addr(void __iomem *base_addr, u32 addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA1_IMG_ADDR], addr);
}

/* code review E1:register set operation, so no return */
void gdc_set_rdma2_img_addr(void __iomem *base_addr, u32 addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA2_IMG_ADDR], addr);
}

/* code review E1:register set operation, so no return */
void gdc_set_rdma0_line_offset(void __iomem *base_addr, u32 lineoffset)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA0_LINE_OFFSET], lineoffset);
}

/* code review E1:register set operation, so no return */
void gdc_set_rdma1_line_offset(void __iomem *base_addr, u32 lineoffset)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA1_LINE_OFFSET], lineoffset);
}

/* code review E1:register set operation, so no return */
void gdc_set_rdma2_line_offset(void __iomem *base_addr, u32 lineoffset)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA2_LINE_OFFSET], lineoffset);
}

/* code review E1:register set operation, so no return */
void gdc_set_wdma0_img_addr(void __iomem *base_addr, u32 addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA0_IMG_ADDR], addr);
}

/* code review E1:register set operation, so no return */
void gdc_set_wdma1_img_addr(void __iomem *base_addr, u32 addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA1_IMG_ADDR], addr);
}

/* code review E1:register set operation, so no return */
void gdc_set_wdma2_img_addr(void __iomem *base_addr, u32 addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA2_IMG_ADDR], addr);
}

/* code review E1:register set operation, so no return */
void gdc_set_wdma0_line_offset(void __iomem *base_addr, u32 lineoffset)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA0_LINE_OFFSET], lineoffset);
}

/* code review E1:register set operation, so no return */
void gdc_set_wdma1_line_offset(void __iomem *base_addr, u32 lineoffset)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA1_LINE_OFFSET], lineoffset);
}

/* code review E1:register set operation, so no return */
void gdc_set_wdma2_line_offset(void __iomem *base_addr, u32 lineoffset)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA2_LINE_OFFSET], lineoffset);
}

/* code review E1:register set operation, so no return */
void gdc_set_default_ch1(void __iomem *base_addr, u32 default_ch)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_DEFAULT_CH1], default_ch);
}

/* code review E1:register set operation, so no return */
void gdc_set_default_ch2(void __iomem *base_addr, u32 default_ch)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_DEFAULT_CH2], default_ch);
}

/* code review E1:register set operation, so no return */
void gdc_set_default_ch3(void __iomem *base_addr, u32 default_ch)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_DEFAULT_CH3], default_ch);
}

u32 gdc_get_status(const void __iomem *base_addr)
{
	u32 status;

	status = vio_hw_get_reg(base_addr, &gdc_regs[GDC_STATUS]);
	return status;
}

/* code review E1:register set operation, so no return */
void gdc_set_irq_mask(void __iomem *base_addr, u32 enable)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_INT_MASK], enable);
}

u32 gdc_get_irq_status(void __iomem *base_addr)
{
	u32 status;

	status = vio_hw_get_reg(base_addr, &gdc_regs[GDC_INT_STATUS]);
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_INT_STATUS], status);

	return status;
}

void gdc_fusa_set_pwd(void __iomem *base_addr, u32 pwd)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_FUSA_PWD], pwd);
}

void gdc_fusa_set_irq_mask(void __iomem *base_addr, u32 mask)
{
	vio_hw_set_field(base_addr, &gdc_regs[GDC_FUSA_INT_MASK],
		&gdc_fields[GDC_F_IRQ_MASK], mask);
}

u32 gdc_fusa_get_irq_status(void __iomem *base_addr)
{
	u32 status;

	status = vio_hw_get_reg(base_addr, &gdc_regs[GDC_FUSA_INT_STATUS]);
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_FUSA_INT_STATUS], status);

	return status;
}
/* code review E1:register dump operation, so no return */
void gdc_hw_dump(const void __iomem *base_reg)
{
	vio_hw_dump_regs(base_reg, gdc_regs, (u32)NUM_OF_GDC_REG);
}

/* code review E1: internal function and just assignment logic, so no need return */
void gdc_set_module_id(u16 module_id, u16 event_id)
{
	u32 i;

	for(i = 0; i < (u32)NUM_OF_GDC_REG; i++) {
		gdc_regs[i].module_id = ((u32)module_id << SHIFT_16) | event_id;
	}
}

/* code review E1:register set operation, so no return */
void gdc_set_parity_inject_value(void __iomem *base_reg, u32 value)
{
	gdc_regs[GDC_INT_MASK].attr = FUSA_RW;
	vio_hw_set_reg(base_reg, &gdc_regs[GDC_INT_MASK], value);
	gdc_regs[GDC_INT_MASK].attr = RW;
}

u32 gdc_get_parity_inject_value(const void __iomem *base_reg)
{
	return vio_hw_get_reg(base_reg, &gdc_regs[GDC_INT_MASK]);
}

s32 gdc_check_default_value(const void __iomem *base_reg)
{
	s32 ret = 0;
	s32 i;
	u32 default_value;

	for(i = 0; i < (s32)NUM_OF_GDC_REG; i++) {
		default_value = vio_hw_get_reg(base_reg, &gdc_regs[i]);
		if (default_value != gdc_regs[i].default_val)
			break;
	}

	if (i != (s32)NUM_OF_GDC_REG) {
		ret = -1;
		vio_info("%s %s\n", __func__, gdc_regs[i].reg_name);
	}
	return ret;
}
