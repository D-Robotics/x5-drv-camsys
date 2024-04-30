/**
 * @file: gdc_hw_api.h
 * @
 * @NO{S09E03C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */

#ifndef HOBOT_GDC_HW_API
#define HOBOT_GDC_HW_API

#define SHIFT_16 16u
#define SHIFT_8 8u
#define MASK_8 0xffu

#define PASSWD_KEY		(0x95F2D303u)
#define DEFAULT_PASSWD_KEY	(0x0badbeefu)

void gdc_set_config_addr(void __iomem *base_addr, u32 config_addr);
void gdc_set_config_size(void __iomem *base_addr, u32 config_addr);
void gdc_set_rdma_img_width(void __iomem *base_addr, u32 width);
void gdc_set_rdma_img_height(void __iomem *base_addr, u32 height);
void gdc_set_wdma_img_width(void __iomem *base_addr, u32 width);
void gdc_set_wdma_img_height(void __iomem *base_addr, u32 height);
void gdc_process_enable(void __iomem *base_addr, u8 enable);
void gdc_process_reset(void __iomem *base_addr, u8 enable);
void gdc_set_rdma0_img_addr(void __iomem *base_addr, u32 addr);
void gdc_set_rdma1_img_addr(void __iomem *base_addr, u32 addr);
void gdc_set_rdma2_img_addr(void __iomem *base_addr, u32 addr);
void gdc_set_rdma0_line_offset(void __iomem *base_addr, u32 lineoffset);
void gdc_set_rdma1_line_offset(void __iomem *base_addr, u32 lineoffset);
void gdc_set_rdma2_line_offset(void __iomem *base_addr, u32 lineoffset);
void gdc_set_wdma0_img_addr(void __iomem *base_addr, u32 addr);
void gdc_set_wdma1_img_addr(void __iomem *base_addr, u32 addr);
void gdc_set_wdma2_img_addr(void __iomem *base_addr, u32 addr);
void gdc_set_wdma0_line_offset(void __iomem *base_addr, u32 lineoffset);
void gdc_set_wdma1_line_offset(void __iomem *base_addr, u32 lineoffset);
void gdc_set_wdma2_line_offset(void __iomem *base_addr, u32 lineoffset);
void gdc_set_default_ch1(void __iomem *base_addr, u32 default_ch);
void gdc_set_default_ch2(void __iomem *base_addr, u32 default_ch);
void gdc_set_default_ch3(void __iomem *base_addr, u32 default_ch);
u32 gdc_get_status(const void __iomem *base_addr);
void gdc_set_irq_mask(void __iomem *base_addr, u32 enable);
u32 gdc_get_irq_status(void __iomem *base_addr);

//j6e
void gdc_fusa_set_pwd(void __iomem *base_addr, u32 pwd);
void gdc_fusa_set_irq_mask(void __iomem *base_addr, u32 enable);
u32 gdc_fusa_get_irq_status(void __iomem *base_addr);

void gdc_hw_dump(const void __iomem *base_reg);
void gdc_set_module_id(u16 module_id, u16 event_id);
void gdc_set_parity_inject_value(void __iomem *base_reg, u32 value);
u32 gdc_get_parity_inject_value(const void __iomem *base_reg);
s32 gdc_check_default_value(const void __iomem *base_reg);

#endif
