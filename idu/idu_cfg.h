/*
 * hobot-drivers/camsys/idu/idu_cfg.h
 *
 * Copyright (C) 2020 horizon
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef IDU_CFG_H
#define IDU_CFG_H
#include <linux/types.h>

#define MAX_DISP_NUM 		(2U)
#define DISP_DEV_MASK		(0x11U)
#define PALETTE_SIZE 256

typedef enum {
	IDU_ICHN1 = 0,
	IDU_ICHN2,
	IDU_ICHN3,
	IDU_ICHN4,
#ifdef CONFIG_HOBOT_CHIP_J6X
	IDU_ICHN5,
	IDU_ICHN6,
#endif
	IDU_ICHN_NUM,
} idu_input_channel_e;

typedef enum {
	OCHN_MIPI_CSI_DEV = 0,
	OCHN_MIPI_DSI,
	OCHN_WRITEBACK,
	OCHN_NUM,
} idu_output_type_e;

typedef struct type_ppcon1_cfg_s {
	uint32_t dithering_flag;
	uint32_t dithering_en;
	uint32_t gamma_en;
	uint32_t hue_en;
	uint32_t sat_en;
	uint32_t con_en;
	uint32_t bright_en;
	uint32_t theta_sign;
	uint32_t contrast;
} ppcon1_cfg_t;

typedef struct type_ppcon2_cfg_s {
	uint32_t theta_abs; //ppcon2
	uint32_t saturation;
	uint32_t off_contrast;
	uint32_t off_bright;
	float	 gamma_value;
} ppcon2_cfg_t;

typedef struct gamma_reg_bits_s {
	uint32_t part_a : 8;
	uint32_t part_b : 8;
	uint32_t part_c : 8;
	uint32_t part_d : 8;
} gamma_reg_bits_t;

typedef union gamma_para_s {
	uint32_t		value;
	struct gamma_reg_bits_s bit;
} gama_para_t;

typedef struct refresh_cfg_s {
	uint32_t dbi_refresh_mode; //refresh mode
	uint32_t panel_color_type;
	uint32_t interlace_sel;
	uint32_t odd_polarity;
	uint32_t pixel_rate;
	uint32_t ycbcr_out;
	uint32_t uv_sequence;
	uint32_t itu_r656_en;

	uint32_t auto_dbi_refresh_cnt;
	uint32_t auto_dbi_refresh_en;
} refresh_cfg_t;

typedef struct disp_timing_s {
	uint32_t hbp;
	uint32_t hfp;
	uint32_t hs;
	uint32_t vbp;
	uint32_t vfp;
	uint32_t vs;
	uint32_t vfp_cnt;
	uint32_t pix_rate;
} disp_timing_t;

typedef struct csi_dev_channel_cfg_s {
	uint16_t enable;
	uint16_t lanes; //lanes mode
	uint16_t fps; //mode to fps
	uint32_t datatype; //format to csi dt
	uint32_t bpp; //format to bpp
	uint16_t mipiclk; //
	uint16_t width; //w
	uint16_t height; //h
	uint16_t linelenth; //htotal
	uint16_t framelenth; //vtotal
	uint16_t settle; //
	uint16_t vpg; //
	uint16_t ipi_lines; //vstart+1
	uint16_t channel_num; //ipi channel
	// uint8_t channel_sel[MIPI_CSI_DEV_CHANNEL_NUM]; // vc
	uint8_t	 lpclk_mode; //
	uint8_t	 vpg_mode; //
	uint8_t	 vpg_hsyncpkt_en; //
} csi_dev_channel_cfg_t;

typedef struct dsi_channel_cfg_s {
	uint16_t enable;
	uint16_t lanes;
	uint32_t mipiclk;     /* units: Mbps*/
	uint32_t pixel_clock; /* units: khz*/
	uint16_t eotp_rx_en;
	uint16_t eotp_tx_en;
	uint16_t cntmode;

	uint16_t channel;
	uint16_t cmd_channel;
	uint16_t color_coding;
	uint8_t hsync_low;    /* 1: configures the dpihsync pin as active low*/
	uint8_t vsync_low;    /* 1: configures the dpivsync pin as active low*/
	uint8_t dataen_low;   /* 1: configures the dpidataen pin as active low*/

	uint16_t video_mode;
	uint16_t vpg;
	uint16_t width;
	uint16_t height;
	uint32_t hbp;   /* pixels*/
	uint32_t hsa;   /* pixels*/
	uint32_t hline; /* pixels*/
	uint32_t vfp;   /* lines*/
	uint32_t vbp;   /* lines*/
	uint32_t vsa;   /* lines*/
} dsi_channel_cfg_t;

typedef struct writeback_channel_cfg_s {
	uint32_t enable;
	uint32_t point;
	uint32_t format;
	uint32_t external_buf;
	uint32_t paddr[3];
} writeback_channel_cfg_t;

typedef struct upscaling_cfg_s {
	uint32_t enable;
	uint32_t layer_no;
	uint32_t src_width;
	uint32_t src_height;
	uint32_t tgt_width;
	uint32_t tgt_height;
} upscaling_cfg_t;

typedef struct gamma_cfg_s {
	gama_para_t gamma_xr[4];
	gama_para_t gamma_xg[4];
	gama_para_t gamma_xb[4];
	gama_para_t gamma_yr[4];
	gama_para_t gamma_yg[4];
	gama_para_t gamma_yb[4];
	gama_para_t gamma_y16rgb;
} gamma_cfg_t;

typedef struct output_bg_color_s {
	uint32_t mode;
	uint32_t bgcolor;
} output_bg_color;

typedef struct output_cfg_s {
	uint32_t      enable;
	uint32_t      out_sel;
	uint32_t      width;
	uint32_t      height;
	uint32_t      bgmode;
	uint32_t      bgcolor;
	uint32_t      out_format;
	ppcon1_cfg_t  ppcon1;
	ppcon2_cfg_t  ppcon2;
	refresh_cfg_t refresh_cfg;
	gamma_cfg_t   gamma_cfg;
	disp_timing_t timing;
	struct csi_dev_channel_cfg_s csi_tx_cfg;
	struct dsi_channel_cfg_s dsi_cfg;
	struct writeback_channel_cfg_s wb_cfg;
} output_cfg_t;

typedef struct channel_base_cfg_s {
	uint32_t channel;
	uint32_t enable;
	uint32_t pri;
	uint32_t width;
	uint32_t height;
	uint32_t xposition;
	uint32_t yposition;
	uint32_t format;
	uint32_t alpha;
#ifdef CONFIG_HOBOT_CHIP_J6X
	uint32_t keycolor_low;
	uint32_t keycolor_hig;
#elif CONFIG_HOBOT_CHIP_J5
	uint32_t keycolor;
#endif
	uint32_t alpha_sel;
	uint32_t ov_en;
	uint32_t ov_mode;
	uint32_t alpha_en;
	uint32_t global_alpha;
	uint32_t crop_x;
	uint32_t crop_y;
	uint32_t crop_width;
	uint32_t crop_height;
	uint32_t rotation;
#ifdef CONFIG_HOBOT_CHIP_J6X
	uint32_t up_scaling_enable;
	uint32_t dst_width;
	uint32_t dst_height;
#endif

	uint32_t argb_endian_sel;
	uint32_t rgb565_convert_sel;
	uint32_t bt601_709_sel;
	uint32_t palette[PALETTE_SIZE];
} channel_base_cfg_t;

typedef struct layer_ctrl_s {
	uint32_t layer_no;
	uint32_t enable;
	uint32_t width;
	uint32_t height;
	uint32_t pos_x;
	uint32_t pos_y;
} layer_ctrl_t;

typedef struct disp_cfg_s {
	uint32_t	   disp_id;
	channel_base_cfg_t channel_base_cfg[IDU_ICHN_NUM];
	output_cfg_t	   output_cfg;
} disp_cfg_t;

typedef enum {
	DISP_DYNAMIC_UPSCALE = 0,
	DISP_DYNAMIC_TIMING,
	DISP_DYNAMIC_GAMMA,
	DISP_DYNAMIC_LAYERCTRL,
	DISP_DYNAMIC_LAYERCFG,
	DISP_DYNAMIC_OUTPUT,
	DISP_DYNAMIC_CHANNEL,
	DISP_DYNAMIC_NUM,
} disp_dynamic_type_e;

typedef struct disp_dynamic_cfg_s {
	uint32_t	     disp_id;
	disp_dynamic_type_e  type;
	struct upscaling_cfg_s upscale_cfg;
	struct disp_timing_s timing_cfg;
	struct gamma_cfg_s   gamma_cfg;
	struct layer_ctrl_s  layer_ctrl;
	struct channel_base_cfg_s channel_cfg;
	struct output_cfg_s output_cfg;
} disp_dynamic_cfg_t;

#endif //IDU_CFG_H