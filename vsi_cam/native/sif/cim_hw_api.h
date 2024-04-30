/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef HOBOT_CIM_HW_API_H
#define HOBOT_CIM_HW_API_H

#define REG_CFG_SIZE        0xCu

#include "vin_node_config.h"
#include "hobot_dev_cim.h"

struct ipi_intr_s {
	u32 cim_ipi_fs;
	u32 cim_ipi_fe;
	u32 cim_ipi_werr;
	u32 cim_ipi_herr;
	u32 cim_emb_werr;
	u32 cim_emb_herr;
};

struct fifo_dma_intr_s {
	u32 dma_cmn_status;				/* rch_status[0:1] wch_status[16:21] */
	u32 dma_cmn_size_err;				/* rq_size_err[15] wq_size_err[31] */
	u32 img_w_int_status[FIFO_W_CHN_MAX];
	u32 img_r_int_status[FIFO_R_CHN_MAX];
};

struct cim_irq_src_s {
	struct ipi_intr_s ipi_intr[CIM_IPI_MAX_NUM];	/* from cim_ipi_reg */
	u32 ipi_drop_status;				/* from cim_reg (ipi_drop[0:3])*/
	u32 ipi_drop_done_status;			/* from cim_reg (ipi_drop_done[0:3])*/
	u32 dma_status;					/* from cim_reg  (wdone[0:11])(rdone[12:15])(wdrop[16:27]) */
	struct fifo_dma_intr_s dma_intr[FIFO_DMA_NUM];	/* from fifo_dma_cmn_reg & fifo_dma_chn_reg */
};

struct cim_irq_ipi_s {
	u32 fs;
	u32 fe;
	u32 werr;
	u32 herr;
	u32 emb_werr;
	u32 emb_herr;

	/* from cim_reg */
	u32 ipi_drop;
	u32 ipi_drop_done;
	u32 wch_frame_done[W_CHNS_EACH_IPI];
	u32 wch_frame_drop_done[W_CHNS_EACH_IPI];
	u32 rch_frame_done;

	/* from cim_dma_cmn_reg */
	u32 wch_status[W_CHNS_EACH_IPI];
	u32 rch_status;

	/* from fifo_dma_cmn_reg */
	u32 img_w_int_status[W_CHNS_EACH_IPI];
	u32 img_r_int_status;
};

void cim_set_ipi_enable(void __iomem *base_addr, u8 channel, u32 enable);
void cim_set_ipi_hw_drop(void __iomem *base_reg, u8 ipi_chn, u8 enable);
void cim_set_ipi_drop(void __iomem *base_reg, u8 ipi_chn, u8 enable);
void cim_set_ipi_drop_done(void __iomem *base_reg, u8 ipi_chn, u8 enable);
void cim_set_ipis_enable(void __iomem *base_addr, u32 enable);
void cim_set_image_size(void __iomem *base_addr, u8 channel,
			u32 width, u32 height);
// void cim_set_shd_rdy(void __iomem *base_addr, u8 channel, u8 enable);
void cim_set_raw_wap(void __iomem *base_addr, u8 channel, u8 enable);
void cim_set_hdr_mod(void __iomem *base_addr, u8 channel, u32 dol);
void cim_set_data_type(void __iomem *base_addr, u8 channel, u32 type);
void cim_set_yuv_swap(void __iomem *base_addr, u8 channel, u8 enable);
void cim_set_raw16_splict(void __iomem *base_addr, u8 channel, u32 enable);
void cim_set_debug_clear(void __iomem *base_addr, u8 channel, u32 enable);
void cim_set_fs_mask(void __iomem *base_addr, u8 channel, u8 enable);
void cim_set_fe_mask(void __iomem *base_addr, u8 channel, u8 enable);
void cim_set_werr_mask(void __iomem *base_addr, u8 channel, u8 enable);
void cim_set_herr_mask(void __iomem *base_addr, u8 channel, u8 enable);
u32 cim_get_frame_id(void __iomem *base_addr, u8 channel);
void cim_set_timestamp_en(void __iomem *base_addr, u8 channel,
		u8 enable);
void cim_set_lpwm_timestamp_en(void __iomem *base_addr, u8 channel,
		u8 sel, u8 enable);
void cim_set_frame_id_set(void __iomem *base_addr, u8 channel,
		u8 enable);
u64 cim_get_timestamp(void __iomem *base_addr, u8 channel);
u64 cim_get_lpwm_timestamp(void __iomem *base_addr, u8 channel);
u32 cim_get_debug_hcnt(void __iomem *base_addr, u8 channel);
u32 cim_get_debug_vcnt(void __iomem *base_addr, u8 channel);
void cim_set_tpg_path(void __iomem *base_addr, u8 channel, u32 en);
void cim_enable_test_pattern(void __iomem *base_addr, u8 ipi_chn, u32 enable);

// void cim_set_test_pattern(void __iomem *base_reg, u8 ipi_chn,
// 		vin_ichn_attr_t *p_data, u32 fps);
void cim_set_frame_id(void __iomem *base_addr, u8 channel, u32 enable,
		u32 init_frame_id);
// void cim_set_wr_pack_mode(void __iomem *base_reg, u8 ipi_chn, u8 wr_chn, u32 enable);
void cim_set_wr_en(void __iomem *base_reg, u8 ipi_chn, u8 wr_chn, u32 enable);
void cim_set_rd_en(void __iomem *base_reg, u8 ipi_chn, u32 enable);
u32 cim_dma_get_ch_lines(void __iomem *base_reg, u8 channel);
void cim_set_wr_force_drop(void __iomem *base_reg, u8 ipi_chn, u8 wr_chn, u8 enable);
void cim_set_wr_shd_ready(void __iomem *base_reg, u8 channel, u8 enable);
void cim_set_wr_data_type(void __iomem *base_reg, u8 ipi_chn, u8 wr_chn, u32 cfg);
void dma_get_cmn_status(void __iomem *base_reg, struct fifo_dma_intr_s *src);
u32 dma_get_ch_status(void __iomem *base_reg, u8 channel);
void dma_frame_done_en(void __iomem *base_reg, u8 ipi_chn, u8 wr_ch, u8 enable);
void dma_frame_drop_done_en(void __iomem *base_reg, u8 ipi_chn, u8 wr_ch, u8 enable);
void dma_frame_drop_en(void __iomem *base_reg, u8 ipi_chn, u8 wr_ch, u8 enable);

void cim_set_emb_size(void __iomem *base_reg, u8 channel, u32 width, u32 height);
void cim_set_emb_roi_en(void __iomem *base_reg, u8 channel, u32 enable);
void cim_set_emb_roi_start(void __iomem *base_reg, u8 channel, u32 start_x, u32 start_y);
void cim_set_emb_werr_intr(void __iomem *base_reg, u8 channel, u32 enable);
void cim_set_emb_herr_intr(void __iomem *base_reg, u8 channel, u32 enable);
u32 cim_get_emb_size_err(void __iomem *base_reg, u8 channel);
u32 cim_get_emb_debug(void __iomem *base_reg, u8 channel);
// void cim_emb_debug_clear(void __iomem *base_reg, u8 channel, u8 clear);

u32 cim_get_emb_rx_en(void __iomem *base_reg, u8 channel);
void cim_set_emb_force_stop(void __iomem *base_reg, u8 channel, u8 enable);

void dma_set_intr_mask(void __iomem *base_reg, u8 ipi_chn, u8 wr_chn, u8 cfg);
void dma_set_intr_linenum(void __iomem *base_reg, u8 ipi_chn, u8 wr_chn, u8 index, u32 lines);
void dma_set_base_addr(void __iomem *base_reg, u8 ipi_chn, u8 wr_chn, u32 addr);
void dma_set_max_blen(void __iomem *base_reg, u8 ipi_chn, u8 wr_chn, u8 cfg);
void dma_set_stride_len(void __iomem *base_reg, u8 ipi_chn, u8 wr_chn, u32 len);
void dma_set_bus_ctrl(void __iomem *base_reg, u8 channel, u8 cfg);
void dma_set_res_size(void __iomem *base_reg, u8 ipi_chn, u8 wr_chn,
			u32 width, u32 height);
void cim_hw_dump(void __iomem *base_reg);
s32 cim_check_default_value(void __iomem *base_reg);
void cim_set_module_id(u16 module_id, u16 event_id);
void cim_get_irq_src(void __iomem *base_reg, struct cim_irq_src_s *src);
void cim_set_parity_inject_value(void __iomem *base_reg, u32 value);
u32 cim_get_parity_inject_value(void __iomem *base_reg);
void fifo_set_parity_inject_value(void __iomem *base_reg, u32 value);
u32 fifo_get_parity_inject_value(void __iomem *base_reg);
void cim_disable_tpg_path(void __iomem *base_addr);

// void cim_isp_set_cpd_param(void __iomem *base_reg, u8 ipi_chn, cim_isp_cpd_param_t *cpd_param);
// void cim_isp_set_dec_param(void __iomem *base_reg, u8 ipi_chn, cim_isp_dec_param_t *dec_param);
// void cim_isp_set_blc_param(struct j6_cim_dev *cim, u8 ipi_chn, cim_isp_blc_param_t *blc_param);
// void cim_isp_set_dpc_param(void __iomem *base_reg, u8 ipi_chn, cim_isp_dpc_param_t *dpc_param);
// void cim_isp_set_rgbir_param(struct j6_cim_dev *cim, u8 ipi_chn, cim_isp_rgbir_param_t *rgbir_param);
void cim_disable_all_isp_param(void __iomem *base_reg, u8 ipi_chn);

void cim_set_raws_en(void __iomem *base_reg, u8 channel, u8 enable);
void cim_set_raws_size(void __iomem *base_reg, u8 channel, u32 width, u32 height);
void cim_set_raws_weight(void __iomem *base_reg, u8 channel, u32 g0, u32 g1, u32 g2, u32 g3);

void cim_set_roi_en(void __iomem *base_addr, u8 ipi_chn, u8 enable);
void cim_set_roi_size(void __iomem *base_reg, u8 channel, u32 width, u32 height);
// void cim_set_roi_start(void __iomem *base_reg, u8 channel, u32 start_x, u32 start_y);

void cim_set_isp_roi_en(void __iomem *base_addr, u8 ipi_chn, u8 enable);
void cim_set_isp_roi_size(void __iomem *base_reg, u8 channel, u32 width, u32 height);
void cim_set_isp_roi_start(void __iomem *base_reg, u8 channel, u32 start_x, u32 start_y);

void cim_set_online_isp(void __iomem *base_addr, u8 ipi_chn, u8 enable);
void cim_enable_online_isp(void __iomem *base_addr, u8 ipi_chn, u8 enable);
void cim_set_online_pym(void __iomem *base_addr, u8 ipi_chn, u8 enable);
void cim_enable_online_pym(void __iomem *base_addr, u8 ipi_chn, u8 enable);
void cim_set_data_src(void __iomem *base_addr, u8 ipi_chn, u8 src);
void cim_set_dma_en(void __iomem *base_addr, u8 ipi_chn, u8 wr_chn, u8 enable);

#endif/*HOBOT_CIM_HW_API_H*/
