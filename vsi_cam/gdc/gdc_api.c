/* SPDX-License-Identifier: GPL-2.0-only */
#include <osal_time.h>

#include "cam_ctx.h"
#include "cam_buf.h"
#include "dw_crc.h"
#include "gdc.h"
#include "gdc_uapi.h"

#include "gdc_api.h"

void gdc_hw_set_config_addr(void __iomem *base_addr, uint32_t config_addr)
{
	gdc_hw_write(base_addr, GDC_HW_CONFIG_ADDR, config_addr);
}

void gdc_hw_set_config_size(void __iomem *base_addr, uint32_t config_addr)
{
	gdc_hw_write(base_addr, GDC_HW_CONFIG_SIZE, config_addr);
}

void gdc_hw_get_config(void __iomem *base_addr, uint32_t *config_addr, uint32_t *config_size)
{
	*config_addr = gdc_hw_read(base_addr, GDC_HW_CONFIG_ADDR);
	*config_size = gdc_hw_read(base_addr, GDC_HW_CONFIG_SIZE) * 4;
}

void gdc_hw_set_rdma_img_width(void __iomem *base_addr, uint32_t width)
{
	gdc_hw_write(base_addr, GDC_HW_RDMA_IMG_WIDTH, width);
}

void gdc_hw_set_rdma_img_height(void __iomem *base_addr, uint32_t height)
{
	gdc_hw_write(base_addr, GDC_HW_RDMA_IMG_HEIGHT, height);
}

void gdc_hw_set_wdma_img_width(void __iomem *base_addr, uint32_t width)
{
	gdc_hw_write(base_addr, GDC_HW_WDMA_IMG_WIDTH, width);
}

void gdc_hw_set_wdma_img_height(void __iomem *base_addr, uint32_t height)
{
	gdc_hw_write(base_addr, GDC_HW_WDMA_IMG_HEIGHT, height);
}

void gdc_hw_process_enable(void __iomem *base_addr, uint8_t enable)
{
	uint32_t value;

	value = gdc_hw_read(base_addr, GDC_HW_PROCESS_CONFIG);
	value &= 0xfffffffe;
	value |= (enable & 0x1);
	gdc_hw_write(base_addr, GDC_HW_PROCESS_CONFIG, value);
}

void gdc_hw_process_reset(void __iomem *base_addr, uint8_t reset)
{
	uint32_t value;

	value = gdc_hw_read(base_addr, GDC_HW_PROCESS_CONFIG);
	value &= 0xfffffffd;
	value |= ((reset & 0x1) << 1);
	gdc_hw_write(base_addr, GDC_HW_PROCESS_CONFIG, value);
}

void gdc_hw_set_rdma0_img_addr(void __iomem *base_addr, uint32_t addr)
{
	gdc_hw_write(base_addr, GDC_HW_RDMA0_IMG_ADDR, addr);
}

void gdc_hw_set_rdma1_img_addr(void __iomem *base_addr, uint32_t addr)
{
	gdc_hw_write(base_addr, GDC_HW_RDMA1_IMG_ADDR, addr);
}

void gdc_hw_set_rdma2_img_addr(void __iomem *base_addr, uint32_t addr)
{
	gdc_hw_write(base_addr, GDC_HW_RDMA2_IMG_ADDR, addr);
}

void gdc_hw_set_rdma0_line_offset(void __iomem *base_addr, uint32_t lineoffset)
{
	gdc_hw_write(base_addr, GDC_HW_RDMA0_LINE_OFFSET, lineoffset);
}

void gdc_hw_set_rdma1_line_offset(void __iomem *base_addr, uint32_t lineoffset)
{
	gdc_hw_write(base_addr, GDC_HW_RDMA1_LINE_OFFSET, lineoffset);
}

void gdc_hw_set_rdma2_line_offset(void __iomem *base_addr, uint32_t lineoffset)
{
	gdc_hw_write(base_addr, GDC_HW_RDMA2_LINE_OFFSET, lineoffset);
}

void gdc_hw_set_wdma0_img_addr(void __iomem *base_addr, uint32_t addr)
{
	gdc_hw_write(base_addr, GDC_HW_WDMA0_IMG_ADDR, addr);
}

void gdc_hw_set_wdma1_img_addr(void __iomem *base_addr, uint32_t addr)
{
	gdc_hw_write(base_addr, GDC_HW_WDMA1_IMG_ADDR, addr);
}

void gdc_hw_set_wdma2_img_addr(void __iomem *base_addr, uint32_t addr)
{
	gdc_hw_write(base_addr, GDC_HW_WDMA2_IMG_ADDR, addr);
}

void gdc_hw_set_wdma0_line_offset(void __iomem *base_addr, uint32_t lineoffset)
{
	gdc_hw_write(base_addr, GDC_HW_WDMA0_LINE_OFFSET, lineoffset);
}

void gdc_hw_set_wdma1_line_offset(void __iomem *base_addr, uint32_t lineoffset)
{
	gdc_hw_write(base_addr, GDC_HW_WDMA1_LINE_OFFSET, lineoffset);
}

void gdc_hw_set_wdma2_line_offset(void __iomem *base_addr, uint32_t lineoffset)
{
	gdc_hw_write(base_addr, GDC_HW_WDMA2_LINE_OFFSET, lineoffset);
}

void gdc_hw_set_default_ch1(void __iomem *base_addr, uint32_t default_ch)
{
	gdc_hw_write(base_addr, GDC_HW_DEFAULT_CH1, default_ch);
}

void gdc_hw_set_default_ch2(void __iomem *base_addr, uint32_t default_ch)
{
	gdc_hw_write(base_addr, GDC_HW_DEFAULT_CH2, default_ch);
}

void gdc_hw_set_default_ch3(void __iomem *base_addr, uint32_t default_ch)
{
	gdc_hw_write(base_addr, GDC_HW_DEFAULT_CH3, default_ch);
}

uint32_t gdc_hw_get_status(void __iomem *base_addr)
{
	return gdc_hw_read(base_addr, GDC_HW_STATUS);
}

uint32_t gdc_hw_force_stop(struct gdc_device *gdc)
{
	uint32_t status;

	gdc_hw_process_reset(gdc->base, 1);
	gdc_hw_process_reset(gdc->base, 0);

	status = gdc_hw_get_status(gdc->base);
	if ((status & INT_GDC_BUSY) != 0u) {
		osal_msleep(30);
		pr_debug("%s:status = 0x%x", __func__, status);
	}

	return 0;
}

int32_t gdc_hw_check_status(struct gdc_device *gdc)
{
	s32 status = gdc_hw_get_status(gdc->base);
	s32 ret = 0;

	if ((status & INT_GDC_BUSY) != 0u) {
		pr_err("%s GDC busy\n", __func__);
	}
	if ((status & INT_GDC_ERROR) != 0u) {
		if ((status & INT_GDC_CONF_ERROR) != 0u)
			pr_err("%s GDC configuration error\n", __func__);

		if ((status & INT_GDC_USER_ABORT) != 0u)
			pr_err("%s GDC user abort(stop/reset command)\n", __func__);

		if ((status & INT_GDC_AXI_READER_ERROR) != 0u)
			pr_err("%s GDC AXI reader error\n", __func__);

		if ((status & INT_GDC_AXI_WRITER_ERROR) != 0u)
			pr_err("%s GDC AXI writer error\n", __func__);

		if ((status & INT_GDC_UNALIGNED_ACCESS) != 0u)
			pr_err("%s GDC address pointer is not aligned\n", __func__);

		if ((status & INT_GDC_INCOMPATIBLE_CONF) != 0u)
			pr_err("%s GDC incopatible configuration\n", __func__);
		ret = -1;
	}
	return ret;
}

int32_t gdc_hw_set_format(struct gdc_device *gdc, uint32_t inst, struct gdc_format *fmt)
{
	struct cam_format ifmt = fmt->ifmt;
	struct cam_format ofmt = fmt->ofmt;

	gdc_hw_set_rdma_img_width(gdc->base, ifmt.width);
	gdc_hw_set_rdma_img_height(gdc->base, ifmt.height);
	gdc_hw_set_rdma0_line_offset(gdc->base, ifmt.stride);
	gdc_hw_set_rdma1_line_offset(gdc->base, ifmt.stride);

	gdc_hw_set_wdma_img_width(gdc->base, ofmt.width);
	gdc_hw_set_wdma_img_height(gdc->base, ofmt.height);
	gdc_hw_set_wdma0_line_offset(gdc->base, ofmt.stride);
	gdc_hw_set_wdma1_line_offset(gdc->base, ofmt.stride);

	return 0;
}

int32_t gdc_start(struct gdc_device *gdc)
{
	dw_set_state(gdc->crc_dev, DW_MOD_GDC, CAM_STATE_STARTED);

	return 0;
}

int32_t gdc_stop(struct gdc_device *gdc)
{
	gdc_hw_force_stop(gdc);
	dw_set_state(gdc->crc_dev, DW_MOD_GDC, CAM_STATE_STOPPED);

	return 0;
}

void gdc_set_in_buffer(struct gdc_device *gdc, struct cam_format *fmt, struct cam_buf *buf)
{
	phys_addr_t paddr;

	paddr = get_phys_addr(gdc->cam_dev, buf, 0);
	gdc_hw_set_rdma0_img_addr(gdc->base, paddr);
	gdc_hw_set_rdma1_img_addr(gdc->base, paddr + fmt->stride * fmt->height);
}

void gdc_set_out_buffer(struct gdc_device *gdc, struct cam_format *fmt, struct cam_buf *buf)
{
	phys_addr_t paddr;

	paddr = get_phys_addr(gdc->cam_dev, buf, 0);
	gdc_hw_set_wdma0_img_addr(gdc->base, paddr);
	gdc_hw_set_wdma1_img_addr(gdc->base, paddr + fmt->stride * fmt->height);
}

void gdc_set_cfg_buffer(struct gdc_device *gdc, phys_addr_t paddr, uint32_t size)
{
	gdc_hw_set_config_addr(gdc->base, paddr);
	gdc_hw_set_config_size(gdc->base, size / 4);
}

void gdc_get_cfg_buffer(struct gdc_device *gdc, phys_addr_t *paddr, uint32_t *size)
{
	gdc_hw_get_config(gdc->base, (uint32_t *)paddr, size);
}

/* gdc_init need called when process every frame */
void gdc_hw_init(struct gdc_device *gdc)
{
	gdc_hw_process_enable(gdc->base, 0);
	gdc_hw_process_reset(gdc->base, 1);
	gdc_hw_process_reset(gdc->base, 0);
	gdc_hw_set_default_ch1(gdc->base, ((u32)GDC_DEFAULT_COLOR >> 16) & 0xff);
	gdc_hw_set_default_ch2(gdc->base, ((u32)GDC_DEFAULT_COLOR >> 8) & 0xff);
	gdc_hw_set_default_ch3(gdc->base, (u32)GDC_DEFAULT_COLOR & 0xff);
}

/* gdc_hw_start_process need called when process every frame */
void gdc_hw_start_process(struct gdc_device *gdc)
{
	gdc_hw_process_enable(gdc->base, 0);
	gdc_hw_process_enable(gdc->base, 1);
}

/* NOTICE
 *
 * process one frame:
 *
 * gdc_hw_init();
 * gdc_set_in_buffer && gdc_set_out_buffer && gdc_set_cfg_buffer && gdc_set_formt // No order required
 * gdc_hw_start_process();
 *
 * process hw interrupt;
 *
 */
