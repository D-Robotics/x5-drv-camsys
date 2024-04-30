/*
 * hobot-drivers/camsys/idu/hobot_idu_vnode_ops.c
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

#include <linux/interrupt.h>
#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/wait.h>

#include <osal.h>
#include <vio_config.h>
#include <vio_framemgr.h>
#include <vio_node_api.h>
#include <hobot_vpf_manager.h>
#include <hb_idu_hw.h>
#include <hb_idu_common.h>

#include "hb_mipi_csi_device.h"
#include "idu_cfg.h"
#include "hobot_idu_vnode_dev.h"
#include "hobot_idu_vnode_ops.h"
#include "hobot_idu_vnode_debug.h"
#if IS_ENABLED(CONFIG_HOBOT_IDE_WRAPPER)
#include "hb_ide_wrapper_drv.h"
#endif

// #define pr_fmt(fmt)    "[IDU_VNODE ops]:" fmt
#define idu_MODULE (13)
#define RST_FLAG (0x3)

#ifdef CONFIG_HOBOT_CHIP_J5
#define MAX_TIMING_PAR (0x3FFu)
#define MAX_CONTRAST_VAL (63u)
#define MAX_UINT8_VAL (255u)
#define MAX_BRIGHT_VAL (0x7fu)
#define MIN_BRIGHT_VAL (0xFFFFFF80u)
#define MAX_CAM_IDU_FREQ (307200000U)
#endif

#ifdef CONFIG_HOBOT_CHIP_J6X
#define MAX_TIMING_PAR (0xFFFu)
#define MAX_CONTRAST_VAL (63u)
#define MAX_UINT8_VAL (255u)
#define MAX_BRIGHT_VAL (0x7fu)
#define MIN_BRIGHT_VAL (0xFFFFFF80u)
#define MAX_CAM_IDU_FREQ (307200000U)
#endif

void idu_process_work(struct idu_subnode *work);

static int32_t disp_channel_cfg_log(channel_base_cfg_t *channel)
{
	if (channel->channel < 0 || channel->channel >= IDU_ICHN_NUM) {
		vio_err("Invalid channel id %d\n", channel->channel);
		return -EINVAL;
	}

	vio_info("channel%d config:\n", channel->channel);
	vio_info("\t enable: %d\n", channel->enable);
	vio_info("\t pri: %d\n", channel->pri);
	vio_info("\t width: %d\n", channel->width);
	vio_info("\t height: %d\n", channel->height);
	vio_info("\t xposition: %d\n", channel->xposition);
	vio_info("\t yposition: %d\n", channel->yposition);
	vio_info("\t format: %d\n", channel->format);
	vio_info("\t alpha: %d\n", channel->alpha);
#ifdef CONFIG_HOBOT_CHIP_J6X
	vio_info("\t keycolor_low: %d\n", channel->keycolor_low);
	vio_info("\t keycolor_hig: %d\n", channel->keycolor_hig);
#endif
#ifdef CONFIG_HOBOT_CHIP_J5
	vio_info("\t keycolor_hig: %d\n", channel->keycolor);
#endif
	vio_info("\t alpha_sel: %d\n", channel->alpha_sel);
	vio_info("\t ov_en: %d\n", channel->ov_en);
	vio_info("\t ov_mode: %d\n", channel->ov_mode);
	vio_info("\t alpha_en: %d\n", channel->alpha_en);
	vio_info("\t global_alpha: %d\n", channel->global_alpha);
#ifdef CONFIG_HOBOT_CHIP_J6X
	vio_info("\t crop_x: %d\n", channel->crop_x);
	vio_info("\t crop_y: %d\n", channel->crop_y);
#endif
	vio_info("\t crop_width: %d\n", channel->crop_width);
	vio_info("\t crop_height: %d\n", channel->crop_height);
	vio_info("\t rotation: %d\n", channel->rotation);
#ifdef CONFIG_HOBOT_CHIP_J6X
	vio_info("\t up_scaling_enable: %d\n", channel->up_scaling_enable);
	vio_info("\t dst_width: %d\n", channel->dst_width);
	vio_info("\t dst_height: %d\n", channel->dst_height);
#endif

	return 0;
}

void disp_capture_cfg_log(struct writeback_channel_cfg_s *cfg)
{
	vio_info("capture config:\n");
	vio_info("\t enable: %d\n", cfg->enable);
	vio_info("\t point: %d\n", cfg->point);
	vio_info("\t format: %d\n", cfg->format);
	vio_info("\t external_buf: %d\n", 0);
	vio_info("\t buf paddr[0]: 0x%x\n", cfg->paddr[0]);
	vio_info("\t buf paddr[1]: 0x%x\n", cfg->paddr[1]);
}

void disp_output_cfg_log(struct output_cfg_s *cfg)
{
	vio_info("output config:\n");
	vio_info("\t enable: %d\n", cfg->enable);
	vio_info("\t out format: %d\n", cfg->out_format);
	vio_info("\t bgcolor: %d\n", cfg->bgcolor);
	vio_info("\t bgmode: %d\n", cfg->bgmode);
	vio_info("\t out_sel: %d\n", cfg->out_sel);
	vio_info("\t width: %d\n", cfg->width);
	vio_info("\t height: %d\n", cfg->height);

	vio_info("\t ppcon1 config\n");
	vio_info("\t\t dithering_flag: %d\n", cfg->ppcon1.dithering_flag);
	vio_info("\t\t dithering_en: %d\n", cfg->ppcon1.dithering_en);
	vio_info("\t\t gamma_en: %d\n", cfg->ppcon1.gamma_en);
	vio_info("\t\t hue_en: %d\n", cfg->ppcon1.hue_en);
	vio_info("\t\t sat_en: %d\n", cfg->ppcon1.sat_en);
	vio_info("\t\t con_en: %d\n", cfg->ppcon1.con_en);
	vio_info("\t\t bright_en: %d\n", cfg->ppcon1.bright_en);
	vio_info("\t\t theta_sign: %d\n", cfg->ppcon1.theta_sign);
	vio_info("\t\t contrast: %d\n", cfg->ppcon1.contrast);

	vio_info("\t ppcon2 config\n");
	vio_info("\t\t theta_abs: %d\n", cfg->ppcon2.theta_abs);
	vio_info("\t\t saturation: %d\n", cfg->ppcon2.saturation);
	vio_info("\t\t off_contrast: %d\n", cfg->ppcon2.off_contrast);
	vio_info("\t\t off_bright: %d\n", cfg->ppcon2.off_bright);
	// vio_info("\t\t gamma_value: %0.5f\n", cfg->ppcon2.gamma_value);

	vio_info("\t refresh config\n");
	vio_info("\t\t dbi_refresh_mode: %d\n", cfg->refresh_cfg.dbi_refresh_mode);
	vio_info("\t\t interlace_sel: %d\n", cfg->refresh_cfg.interlace_sel);
	vio_info("\t\t odd_polarity: %d\n", cfg->refresh_cfg.odd_polarity);
	vio_info("\t\t pixel_rate: %d\n", cfg->refresh_cfg.pixel_rate);
	vio_info("\t\t ycbcr_out: %d\n", cfg->refresh_cfg.ycbcr_out);
	vio_info("\t\t uv_sequence: %d\n", cfg->refresh_cfg.uv_sequence);
	vio_info("\t\t itu_r656_en: %d\n", cfg->refresh_cfg.itu_r656_en);
	vio_info("\t\t auto_dbi_refresh_cnt: %d\n", cfg->refresh_cfg.auto_dbi_refresh_cnt);
	vio_info("\t\t auto_dbi_refresh_en: %d\n", cfg->refresh_cfg.auto_dbi_refresh_en);

	vio_info("\t display timing config\n");
	vio_info("\t\t hbp: %d\n", cfg->timing.hbp);
	vio_info("\t\t hfp: %d\n", cfg->timing.hfp);
	vio_info("\t\t hs: %d\n", cfg->timing.hs);
	vio_info("\t\t vbp: %d\n", cfg->timing.vbp);
	vio_info("\t\t vfp: %d\n", cfg->timing.vfp);
	vio_info("\t\t vs: %d\n", cfg->timing.vs);
	vio_info("\t\t vfp_cnt: %d\n", cfg->timing.vfp_cnt);
	vio_info("\t\t pixel rate: %d\n", cfg->timing.pix_rate);

	if (OCHN_MIPI_CSI_DEV == cfg->out_sel) {
		vio_info("\t MIPI CSI DEV config\n");
		vio_info("\t\t enable: %d\n", cfg->csi_tx_cfg.enable);
		vio_info("\t\t lanes: %d\n", cfg->csi_tx_cfg.lanes);
		vio_info("\t\t fps: %d\n", cfg->csi_tx_cfg.fps);
		vio_info("\t\t datatype: %d\n", cfg->csi_tx_cfg.datatype);
		vio_info("\t\t bpp: %d\n", cfg->csi_tx_cfg.bpp);
		vio_info("\t\t mipiclk: %d\n", cfg->csi_tx_cfg.mipiclk);
		vio_info("\t\t width: %d\n", cfg->csi_tx_cfg.width);
		vio_info("\t\t height: %d\n", cfg->csi_tx_cfg.height);
		vio_info("\t\t linelenth: %d\n", cfg->csi_tx_cfg.linelenth);
		vio_info("\t\t framelenth: %d\n", cfg->csi_tx_cfg.framelenth);
		vio_info("\t\t settle: %d\n", cfg->csi_tx_cfg.settle);
		vio_info("\t\t vpg: %d\n", cfg->csi_tx_cfg.vpg);
		vio_info("\t\t ipi_lines: %d\n", cfg->csi_tx_cfg.ipi_lines);
		vio_info("\t\t channel_num: %d\n", cfg->csi_tx_cfg.channel_num);
		vio_info("\t\t lpclk_mode: %d\n", cfg->csi_tx_cfg.lpclk_mode);
		vio_info("\t\t vpg_mode: %d\n", cfg->csi_tx_cfg.vpg_mode);
		vio_info("\t\t vpg_hsyncpkt_en: %d\n", cfg->csi_tx_cfg.vpg_hsyncpkt_en);
	} else if (OCHN_MIPI_DSI == cfg->out_sel) {
		vio_info("\t MIPI DSI HOST config\n");
		vio_info("\t\t enable: %d\n", cfg->dsi_cfg.enable);
		vio_info("\t\t lanes: %d\n", cfg->dsi_cfg.lanes);
		vio_info("\t\t mipiclk: %d\n", cfg->dsi_cfg.mipiclk);
		vio_info("\t\t eotp_rx_en: %d\n", cfg->dsi_cfg.eotp_rx_en);
		vio_info("\t\t eotp_tx_en: %d\n", cfg->dsi_cfg.eotp_tx_en);
		vio_info("\t\t cntmode: %d\n", cfg->dsi_cfg.cntmode);
		vio_info("\t\t color_coding: %d\n", cfg->dsi_cfg.color_coding);
		vio_info("\t\t channel: %d\n", cfg->dsi_cfg.channel);
		vio_info("\t\t cmd_channel: %d\n", cfg->dsi_cfg.cmd_channel);
		vio_info("\t\t hsync_low: %d\n", cfg->dsi_cfg.hsync_low);
		vio_info("\t\t vsync_low: %d\n", cfg->dsi_cfg.vsync_low);
		vio_info("\t\t dataen_low: %d\n", cfg->dsi_cfg.dataen_low);
		vio_info("\t\t width: %d\n", cfg->dsi_cfg.width);
		vio_info("\t\t heigth: %d\n", cfg->dsi_cfg.height);
		vio_info("\t\t hsa: %d\n", cfg->dsi_cfg.hsa);
		vio_info("\t\t hbp: %d\n", cfg->dsi_cfg.hbp);
		vio_info("\t\t hline: %d\n", cfg->dsi_cfg.hline);
		vio_info("\t\t vsa: %d\n", cfg->dsi_cfg.vsa);
		vio_info("\t\t vbp: %d\n", cfg->dsi_cfg.vbp);
		vio_info("\t\t vfp: %d\n", cfg->dsi_cfg.vfp);
		vio_info("\t\t video_mode: %d\n", cfg->dsi_cfg.video_mode);
		vio_info("\t\t vpg: %d\n", cfg->dsi_cfg.vpg);
	}
	disp_capture_cfg_log(&cfg->wb_cfg);
}

// This function is a port of a J5-driven implementation
static int32_t
idu_channel_base_cfg_size_check(const struct channel_base_cfg_s *cfg)
{
	if ((cfg->width > MAX_FRAME_IN_WIDTH) ||
	    (cfg->xposition > MAX_FRAME_IN_WIDTH) ||
	    (cfg->crop_width > MAX_FRAME_IN_WIDTH)) {
		(void)vio_err("channel width exceed the max limit, exit!!\n");
		return -EINVAL;
	}
	if ((cfg->height > MAX_FRAME_IN_HIGHT) ||
	    (cfg->yposition > MAX_FRAME_IN_HIGHT) ||
	    (cfg->crop_height > MAX_FRAME_IN_HIGHT)) {
		(void)vio_err("%d %d %d channel height exceed the max limit, exit!!\n",
					cfg->height,
					cfg->yposition,
					cfg->crop_height);
		return -EINVAL;
	}

	if ((cfg->width * cfg->height) >
	    (MAX_FRAME_IN_WIDTH * MAX_FRAME_IN_HIGHT)) {
		(void)vio_err(
			"channel width*height exceed the max limit, exit!!\n");
		return -EINVAL;
	}

	return 0;
}

static int32_t hbmem_format_to_capture(uint32_t format)
{
	uint32_t fmt;

	switch (format) {
		case MEM_PIX_FMT_RGB24:
		case MEM_PIX_FMT_ARGB:
			fmt = CAPTURE_FORMAT_RGB888;
			break;
		case MEM_PIX_FMT_YUYV422:
			fmt = CAPTURE_FORMAT_YUV422_YUYV;
			break;
		case MEM_PIX_FMT_NV12:
			fmt = CAPTURE_FORMAT_YUV420SP_UV;
			break;
		case MEM_PIX_FMT_NV21:
			fmt = CAPTURE_FORMAT_YUV420SP_VU;
			break;
		default:
			vio_err("Unsupport capture format %d\n", format);
			fmt = -EINVAL;
			break;
	}

	return fmt;
}

int32_t idu_rgbformat_to_hbmem(uint32_t format)
{
	uint32_t img_fmt = 0;

	switch (format) {
		case FORMAT_RGB565:
			img_fmt = MEM_PIX_FMT_RGB565;
			break;
		case FORMAT_RGB888P:
			img_fmt = MEM_PIX_FMT_RGB24;
			break;
		case FORMAT_ARGB8888:
			img_fmt = MEM_PIX_FMT_ARGB;
			break;
		case FORMAT_RGBA8888:
			img_fmt = MEM_PIX_FMT_RGBA;
			break;
	}

	return img_fmt;
}

int32_t idu_yuvformat_to_hbmem(uint32_t format)
{
	uint32_t img_fmt = 0;

	// RGB format
	switch (format) {
		// YUV format
		case FORMAT_YUV422_UYVY:
			img_fmt = MEM_PIX_FMT_UYVY422;
			break;
		case FORMAT_YUV422_VYUY:
			img_fmt = MEM_PIX_FMT_VYUY422;
			break;
		case FORMAT_YUV422_YUYV:
			img_fmt = MEM_PIX_FMT_YUYV422;
			break;
		case FORMAT_YUV422_YVYU:
			img_fmt = MEM_PIX_FMT_YVYU422;
			break;
		case FORMAT_YUV422SP_UV:
			img_fmt = MEM_PIX_FMT_NV16;
			break;
		case FORMAT_YUV422SP_VU:
			img_fmt = MEM_PIX_FMT_NV61;
			break;
		case FORMAT_YUV420SP_UV:
			img_fmt = MEM_PIX_FMT_NV12;
			break;
		case FORMAT_YUV420SP_VU:
			img_fmt = MEM_PIX_FMT_NV21;
			break;
		case FORMAT_YUV422P_UV:
			img_fmt = MEM_PIX_FMT_YUV422P;
			break;
		case FORMAT_YUV420P_UV:
			img_fmt = MEM_PIX_FMT_YUV420P;
			break;
		default:
			img_fmt = -EINVAL;
			break;
	}

	return img_fmt;
}

static int32_t hbmem_format_to_idu(uint32_t format)
{
	uint32_t img_fmt = FORMAT_YUV420SP_UV;

	// RGB format
	switch (format) {
		case MEM_PIX_FMT_RGB565:
			img_fmt = FORMAT_RGB565;
			break;
		case MEM_PIX_FMT_RGB24:
			img_fmt = FORMAT_RGB888P;
			break;
		case MEM_PIX_FMT_ARGB:
			img_fmt = FORMAT_ARGB8888;
			break;
		case MEM_PIX_FMT_RGBA:
			img_fmt = FORMAT_RGBA8888;
			break;
		case MEM_PIX_FMT_RAW8:
			img_fmt = FORMAT_8BPP;
			break;

		// YUV format
		case MEM_PIX_FMT_UYVY422:
			img_fmt = FORMAT_YUV422_UYVY;
			break;
		case MEM_PIX_FMT_VYUY422:
			img_fmt = FORMAT_YUV422_VYUY;
			break;
		case MEM_PIX_FMT_YUYV422:
			img_fmt = FORMAT_YUV422_YUYV;
			break;
		case MEM_PIX_FMT_YVYU422:
			img_fmt = FORMAT_YUV422_YVYU;
			break;
		case MEM_PIX_FMT_NV16:
			img_fmt = FORMAT_YUV422SP_UV;
			break;
		case MEM_PIX_FMT_NV61:
			img_fmt = FORMAT_YUV422SP_VU;
			break;
		case MEM_PIX_FMT_NV12:
			img_fmt = FORMAT_YUV420SP_UV;
			break;
		case MEM_PIX_FMT_NV21:
			img_fmt = FORMAT_YUV420SP_VU;
			break;
		case MEM_PIX_FMT_YUV422P:
			img_fmt = FORMAT_YUV422P_UV;
			break;
		case MEM_PIX_FMT_YUV420P:
			img_fmt = FORMAT_YUV420P_UV;
			break;
		default:
			img_fmt = -EINVAL;
			break;
	}

	return img_fmt;
}

int32_t idu_capture_to_hbmem(uint32_t format)
{
	int32_t fmt;

	switch (format) {
		case CAPTURE_FORMAT_YUV422_UYVY:
			fmt = MEM_PIX_FMT_UYVY422;
			break;
		case CAPTURE_FORMAT_YUV422_VYUY:
			fmt = MEM_PIX_FMT_VYUY422;
			break;
		case CAPTURE_FORMAT_YUV422_YUYV:
			fmt = MEM_PIX_FMT_YUYV422;
			break;
		case CAPTURE_FORMAT_YUV422_YVYU:
			fmt = MEM_PIX_FMT_YVYU422;
			break;
		case CAPTURE_FORMAT_YUV420SP_UV:
			fmt = MEM_PIX_FMT_NV12;
			break;
		case CAPTURE_FORMAT_YUV420SP_VU:
			fmt = MEM_PIX_FMT_NV21;
			break;
		case CAPTURE_FORMAT_RGB888:
			fmt = MEM_PIX_FMT_ARGB;
			break;
		default:
			vio_err("Unsupport capture format %d.", format);
			fmt = -EINVAL;
	}

	return fmt;
}
#if IS_ENABLED(CONFIG_HOBOT_DRM_MIPI_DSI)
static int32_t dsi_color_coding_to_ide_format(uint32_t color_coding)
{
	int32_t fmt;

	switch (color_coding) {
		case  MIPI_DSI_DPI_COLOR_CODE_16BIT_CONFIG1:
			fmt = RGB2YUV_OUT_DT_RGB565_CONFIG1;
			break;
		case  MIPI_DSI_DPI_COLOR_CODE_16BIT_CONFIG2:
			fmt = RGB2YUV_OUT_DT_RGB565_CONFIG2;
			break;
		case MIPI_DSI_DPI_COLOR_CODE_16BIT_CONFIG3:
			fmt = RGB2YUV_OUT_DT_RGB565_CONFIG3;
			break;
		case  MIPI_DSI_DPI_COLOR_CODE_18BIT_CONFIG1:
			fmt = RGB2YUV_OUT_DT_RGB666_CONFIG1;
			break;
		case  MIPI_DSI_DPI_COLOR_CODE_18BIT_CONFIG2:
			fmt = RGB2YUV_OUT_DT_RGB666_CONFIG2;
			break;
		case  MIPI_DSI_DPI_COLOR_CODE_24BIT:
			fmt = RGB2YUV_OUT_DT_RGB888;
			break;
		case MIPI_DSI_DPI_COLOR_CODE_16BIT_YCC422:
			fmt = RGB2YUV_OUT_DT_YCC422_16BIT;
			break;
		default:
			fmt = -EINVAL;
			vio_err("DSI color no match rgb2yuv out type[%d]\n", color_coding);
			break;
	}

	return fmt;
}
#endif

// This function is a port of a J5-driven implementation
static int32_t
idu_channel_base_cfg_par_check(const struct channel_base_cfg_s *cfg)
{
	if (cfg->channel >= (uint32_t)IDU_ICHN_NUM) {
		(void)vio_err("%s error channel number, exit!!\n", __func__);
		return -EINVAL;
	}

	if (idu_channel_base_cfg_size_check(cfg) != 0) { // mark_hb
		(void)vio_err("%s error channel number, exit!!\n", __func__);
		return -EINVAL;
	}

	if (cfg->pri >= IDU_PRI_MAX) {
		(void)vio_err("%s error channel priority %d, exit!!\n", __func__, cfg->pri);
		return -EINVAL;
	}

	if ((cfg->channel == IDU_ICHN3) || (cfg->channel == IDU_ICHN4)) {
		if (hbmem_format_to_idu(cfg->format) > FORMAT_RGBA8888) {
			(void)vio_err(
				"%s error rgb channel format, excceed the max limit, exit!!\n",
				__func__);
			return -EINVAL;
		}
	} else {
		if (hbmem_format_to_idu(cfg->format) > FORMAT_YUV420P_VU) {
			(void)vio_err(
				"%s error yuv channel format, excceed the max limit,  exit!!\n",
				__func__);
			return -EINVAL;
		}
	}
	if (cfg->alpha > 255) {
		(void)vio_err("%s error channel alpha value, exit!!\n",
			      __func__);
		return -EINVAL;
	}
	if (cfg->ov_mode > OV_MODE_INV) {
		(void)vio_err("%s error channel ov mode, exit!!\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int32_t idu_ichn_attr_param_check(struct vio_video_ctx	   *vctx,
					 struct channel_base_cfg_s *cfg)
{
	int32_t ret;
	struct hobot_idu_dev *idu;

	if (vctx->id - VNODE_ID_SRC != cfg->channel) {
		vio_err("[%s]Invalid channel id.\n", __func__);
		return -EINVAL;
	}
	idu = (struct hobot_idu_dev *)vctx->device;

	if (g_debug_priv[idu->port]->config_log) {
		disp_channel_cfg_log(cfg);
	}
	ret = idu_channel_base_cfg_par_check(cfg);
	if (ret < 0) {
		(void)vio_err("%s , channel%d configuration is error!!\n",
			      __func__, cfg->channel);
		return -EINVAL;
	}

	return 0;
}

static int32_t idu_ichn_attr_param_update(struct vio_video_ctx	    *vctx,
					  struct channel_base_cfg_s *cfg)
{
	struct hobot_idu_dev *idu;
	struct dc_hw_plane   *hw_plane;

	idu = vctx->device;
	if (idu == NULL) {
		(void)vio_err("%s , idu%d_ich%d device is NULL!!\n", __func__,
			      vctx->id, cfg->channel);
		return -EINVAL;
	}

	// Copy the configuration of struct channel_base_cfg_s to the fields of B
	if (idu->hw == NULL) {
		(void)vio_err("%s , idu%d_ich%d open fail!!\n", __func__,
			      vctx->id, cfg->channel);
		return -EINVAL;
	}

	hw_plane = &idu->hw->plane[cfg->channel];

	// update base config
	hw_plane->chan_base.enable = (bool)cfg->enable;
	hw_plane->chan_base.format = hbmem_format_to_idu(cfg->format);
	hw_plane->chan_base.endian_sel = cfg->argb_endian_sel;
	hw_plane->chan_base.rgb565_convert_sel = cfg->rgb565_convert_sel;
	hw_plane->chan_base.bt601_709_sel = cfg->bt601_709_sel;
	memcpy(hw_plane->chan_base.palette, cfg->palette, sizeof(cfg->palette));
	hw_plane->chan_base.img_width = cfg->width;
	hw_plane->chan_base.img_height = cfg->height;
	hw_plane->chan_base.mirror_en = cfg->rotation & FLIP_X;
	hw_plane->chan_base.flip_en = cfg->rotation & FLIP_Y;
	hw_plane->chan_base.threshold = cfg->height;
	hw_plane->chan_base.dirty = true;


	// update position
	hw_plane->pos.xposition = cfg->xposition;
	hw_plane->pos.yposition = cfg->yposition;
	hw_plane->pos.dirty = true;

	// update roi
#ifdef CONFIG_HOBOT_CHIP_J6X
	hw_plane->roi.x = cfg->crop_x;
	hw_plane->roi.y = cfg->crop_y;
#endif
	hw_plane->roi.width = cfg->crop_width;
	hw_plane->roi.height = cfg->crop_height;
	hw_plane->roi.dirty = true;

	// update keycolor
#ifdef CONFIG_HOBOT_CHIP_J6X
	hw_plane->keycolor.low = cfg->keycolor_low;
	hw_plane->keycolor.high = cfg->keycolor_hig;
#elif CONFIG_HOBOT_CHIP_J5
	hw_plane->keycolor.keycolor = cfg->keycolor;
#endif
	hw_plane->keycolor.dirty = true;

	// update alpha_blending
	hw_plane->alpha_blending.priority = cfg->pri;
	hw_plane->alpha_blending.ov_en = 1;
	hw_plane->alpha_blending.ov_mode = 2;
	hw_plane->alpha_blending.alpha_en = cfg->alpha_en;
	hw_plane->alpha_blending.alpha_sel = cfg->alpha_sel;
	hw_plane->alpha_blending.global_alpha = cfg->alpha;
	hw_plane->alpha_blending.dirty = true;

	// update up-scaling
#ifdef CONFIG_HOBOT_CHIP_J6X
	hw_plane->scale.enable = cfg->up_scaling_enable;
	hw_plane->scale.src_width = cfg->crop_width;
	hw_plane->scale.src_height = cfg->crop_height;
	hw_plane->scale.dst_width = cfg->dst_width;
	hw_plane->scale.dst_height = cfg->dst_height;
#elif CONFIG_HOBOT_CHIP_J5
	// J5 do not support up-scaling in plane
#endif
	hw_plane->scale.dirty = true;

	return 0;
}

int32_t idu_vsync_disable(struct hobot_idu_vsync *vsync)
{
	int32_t ret = 0;
	struct idu_subnode *subnode = container_of(vsync, struct idu_subnode, vsync);

	if (vsync->enabled) {
		ret = dc_hw_enable_interrupt(subnode->idu->port, 0);
		if (0 != ret) {
			vio_err("Failed to disable vsync interrupt\n");
		} else {
			WRITE_ONCE(vsync->enabled, 0);
		}
	}

	return ret;
}

int32_t idu_vsync_enable(struct hobot_idu_vsync *vsync)
{
	int32_t ret = 0;
	struct idu_subnode *subnode = container_of(vsync, struct idu_subnode, vsync);

	if (!vsync->enabled) {
		ret = dc_hw_enable_interrupt(subnode->idu->port, 1);
		if (0 != ret) {
			vio_err("Failed to enable vsync interrupt\n");
		} else {
			WRITE_ONCE(vsync->enabled, 1);
		}
	}

	return ret;
}

int32_t idu_vsync_get(struct hobot_idu_vsync *vsync)
{
	uint64_t flags;
	int32_t ret = 0;
	struct idu_subnode *subnode = container_of(vsync, struct idu_subnode, vsync);

	osal_spin_lock_irqsave(&subnode->vsync_lock, &flags);
	if (atomic_add_return(1, &vsync->refcount) == 1) {
		if (!vsync->enabled) {
			ret = idu_vsync_enable(vsync);
		}
	} else {
		if (!vsync->enabled) {
			vio_err("Vsync refcount is not 0, but vsync is disables\n");
			atomic_dec(&vsync->refcount);
			ret = -EINVAL;
		}
	}
	osal_spin_unlock_irqrestore(&subnode->vsync_lock, &flags);

	return ret;
}

int32_t idu_vsync_put(struct hobot_idu_vsync *vsync)
{
	uint64_t flags;
	struct idu_subnode *subnode = container_of(vsync, struct idu_subnode, vsync);

	osal_spin_lock_irqsave(&subnode->vsync_lock, &flags);
	atomic_dec(&vsync->refcount);
	osal_spin_unlock_irqrestore(&subnode->vsync_lock, &flags);

	return 0;
}

void idu_vsync_init(struct hobot_idu_vsync *vsync)
{
	struct idu_subnode *subnode = container_of(vsync, struct idu_subnode, vsync);

	atomic64_set(&vsync->count, 0);
	atomic_set(&vsync->refcount, 0);
	init_waitqueue_head(&vsync->queue);
	osal_spin_init(&subnode->vsync_lock);

	return;
}

void idu_handle_vsync(struct hobot_idu_vsync *vsync)
{
	atomic64_add(1, &vsync->count);
	wake_up(&vsync->queue);
}

int32_t idu_wait_vsync(struct hobot_idu_vsync *vsync, uint64_t rel_count)
{
	int32_t ret, wait = 1;
	uint64_t seq, req_seq;

	if (!vsync->enabled) {
		vio_err("Vsync is disabled.\n");
		return -EINVAL;
	}

	ret = idu_vsync_get(vsync);
	if (ret) {
		vio_err("Failed to refer idu vsync\n");
		return ret;
	}

	seq = atomic64_read(&vsync->count);
	req_seq = seq + rel_count;
	vio_dbg("request %lld->%lld", seq, req_seq);
	if (req_seq != seq) {
		wait = wait_event_interruptible_timeout(vsync->queue,
					 atomic64_read(&vsync->count) >= req_seq,
					 msecs_to_jiffies(3000));
	}
	switch (wait) {
		case 0:
			/* timeout */
			ret = -EBUSY;
			break;
		case -ERESTARTSYS:
			/* interrupted by signal */
			ret = -EINTR;
			break;
		default:
			ret = 0;
			break;
	}
	idu_vsync_put(vsync);

	return ret;
}

/**
 * @NO{S12E01C11}
 * @ASIL{B}
 * @brief turn on frame stream Callback interface of vpf framework
 *
 * @param[in] vctx: vpf contex info
 * @param[out] None
 *
 * @retval "= 0": success
 * @retval "< 0": failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t idu_vpf_streamon(struct vio_video_ctx *vctx)
{
	int32_t		      ret = 0;
	struct vio_node	     *vnode;
	struct vio_subdev    *vdev;
	struct idu_subdev    *subdev;
	struct idu_subnode   *subnode;
	struct hobot_idu_dev *idu;
	struct output_cfg_s  *ochn_cfg;
	int32_t format;

	vctx->event = 0;
	vdev = vctx->vdev;
	vnode = vdev->vnode;
	subdev = container_of(vctx->vdev, struct idu_subdev, vdev);
	subnode = subdev->subnode;
	idu = subnode->idu;
	ochn_cfg = &subnode->idu_cfg.output_cfg;

	if (vctx->id == IDU_OCH_MIPI_CSI_DEV + VNODE_ID_CAP) {
		if (!idu->csi_priv->ops.init || !idu->csi_priv->ops.start) {
			vio_err("No available tx ops.\n");
			return -EINVAL;
		}
#ifndef CONFIG_DISP_SPARKLER_DEBUG
		ret = mipi_csi_clock_configure(idu->csi_priv, idu->csi_priv->refclk, MIPI_CSI_TX_REFCLK_MHZ);
		if (0 != ret) {
			vio_err("REFCLK init failed[%d].\n", ret);
			return ret;
		}
#endif

		if (idu->csi_priv->cfg.enable) {
			ret = idu->csi_priv->ops.init(idu->csi_priv->port,
					      idu->csi_priv->regs, NULL,
					      &idu->csi_priv->cfg);
			if (0 != ret) {
				vio_err("csi dev init failed[%d].\n", ret);
				return ret;
			}
			ret = idu->csi_priv->ops.start(idu->csi_priv->port,
							idu->csi_priv->regs,
							&idu->csi_priv->cfg);
			if (0 != ret) {
				vio_err("mipi csi tx start failed[%d].\n", ret);
				return 0;
			}
		}

	} else if (vctx->id == IDU_OCH_MIPI_DSI + VNODE_ID_CAP) {
#if IS_ENABLED(CONFIG_HOBOT_DRM_MIPI_DSI)
#ifndef CONFIG_DISP_SPARKLER_DEBUG
		ret = mipi_csi_clock_configure(idu->dsi_priv, idu->dsi_priv->refclk, MIPI_DSI_HOST_REFCLK_MHZ);
		if (0 != ret) {
			vio_err("MIPI DSI REFCLK init failed[%d].\n", ret);
			return ret;
		}
#endif
		if (idu->dsi_priv->cfg.enable) {
			ret = idu->dsi_priv->ops.init(idu->dsi_priv->port,
						idu->dsi_priv->regs, NULL,
						&idu->dsi_priv->cfg);
			if (0 != ret) {
				vio_err("dsi host init failed[%d]\n", ret);
				return ret;
			}

			if (idu->hw->display.enable) {
				format = dsi_color_coding_to_ide_format(ochn_cfg->dsi_cfg.color_coding);
				if (format < 0) {
					return -EINVAL;
				}
#if IS_ENABLED(CONFIG_HOBOT_IDE_WRAPPER)
				vio_dbg("ide format set %d\n", format);
				ret = hb_ide_wrapper_path_cfg(idu->port, TX_DPI, format);
				if (0 != ret) {
					vio_err("ide data path cfg failed.[%d]\n", ret);
					return ret;
				}
#endif
			}
		}
#endif

	} else if (vctx->id == IDU_OCH_WRITEBACK + VNODE_ID_CAP) {
#ifdef CONFIG_HOBOT_DISPLAY_WRITEBACK
		if (idu->hw->capture.base.enable) {
			dc_hw_setup_capture(&idu->hw->capture);
		} else {
			dc_hw_disable_capture(&idu->hw->capture);
		}
#endif
	}

	if (vctx->id == VNODE_ID_SRC) {
		idu_vsync_enable(&idu->subnode.vsync);
		idu->hw->display.enable = 1;
		ret = idu->hw->func->enable(idu->hw);
		if (0 != ret) {
			vio_err("IDU start failed.\n");
			return ret;
		}
	}

	osal_set_bit((int32_t)VIO_NODE_START, &vnode->state);

	vio_dbg("[S%d][V%d][C%d] %s leader %d\n", vnode->flow_id, vctx->id,
		 vdev->id, __func__, vdev->leader);

	return 0;
}

/**
 * @NO{S12E01C11}
 * @ASIL{B}
 * @brief turn off frame stream Callback interface of vpf framework
 *
 * @param[in] vctx: vpf contex info
 * @param[out] None
 *
 * @retval "= 0": success
 * @retval "< 0": failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t idu_vpf_streamoff(struct vio_video_ctx *vctx)
{
	int32_t ret = 0;
	struct vio_node	     *vnode;
	struct vio_subdev    *vdev;
	struct idu_subdev    *subdev;
	struct idu_subnode   *subnode;
	struct hobot_idu_dev *idu;

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	subdev = container_of(vctx->vdev, struct idu_subdev, vdev);
	subnode = subdev->subnode;
	idu = subnode->idu;

	if (atomic_read(&vnode->start_cnt) == 0 && idu->hw->display.enable) {
		idu_vsync_get(&idu->subnode.vsync);
		idu_wait_vsync(&idu->subnode.vsync, 1);
		idu_vsync_put(&idu->subnode.vsync);
		idu->subnode.frame_end = 0;
		idu->hw->display.enable = 0;
		idu->hw->func->enable(idu->hw);
		idu_vsync_disable(&idu->subnode.vsync);

		if (idu->csi_priv->cfg.enable) {
			ret = idu->csi_priv->ops.stop(idu->csi_priv->port,
						idu->csi_priv->regs);
			if (0 != ret) {
				vio_err("csi dev stop failed[%d].\n", ret);
				return ret;
			}

			idu->csi_priv->cfg.enable = 0;
		}
#if IS_ENABLED(CONFIG_HOBOT_DRM_MIPI_DSI)
		if (idu->dsi_priv->cfg.enable) {
			ret = idu->dsi_priv->ops.stop(idu->dsi_priv->port,
						idu->dsi_priv->regs);
			if (0 != ret) {
				vio_err("dsi stop failed[%d].\n", ret);
				return ret;
			}

			idu->dsi_priv->cfg.enable = 0;
		}
#endif
	}

	return ret;
}

/**
 * @NO{S09E04C01}
 * @ASIL{B}
 * @brief idu character device node open interface
 * @param[in] *vctx : vpf framework context
 * @param[in] rst_en : Decide whether to rest ip
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
int32_t idu_open(struct vio_video_ctx *vctx, uint32_t rst_en)
{
	int32_t		      ret = 0;
	struct hobot_idu_dev *idu;

	init_waitqueue_head(&vctx->done_wq); //给poll用的, poll具体实现在VPF中
	vctx->state = BIT((int32_t)VIO_VIDEO_OPEN);
	idu = (struct hobot_idu_dev *)vctx->device;

	ret = mutex_lock_interruptible(&idu->mlock);
	if (ret < 0) {
		vio_err("%s, mutex error\n", __func__);
		return ret;
	}
	if (atomic_inc_return(&idu->open_cnt) == 1) {
		idu->hw = dc_hw_register(idu->port, &idu->pdev->dev);
		if (IS_ERR_OR_NULL(idu->hw)) {
			vio_err("Register dc hw filed.[%d]",
				PTR_ERR_OR_ZERO(idu->hw));
			idu->hw = NULL;
			atomic_dec(&idu->open_cnt);
			osal_mutex_unlock(&idu->mlock);
			return -EBUSY;
		}

		idu->csi_priv = mipi_csi_dev_register(idu->port, &idu->pdev->dev);
		if (IS_ERR_OR_NULL(idu->csi_priv)) {
			vio_err("Register mipi csi tx priv data filed.[%d]",
				PTR_ERR_OR_ZERO(idu->csi_priv));
			idu->csi_priv = NULL;
			dc_hw_unregister(idu->hw);
			idu->hw = NULL;
			atomic_dec(&idu->open_cnt);
			osal_mutex_unlock(&idu->mlock);
			return -EBUSY;
		}
#ifdef CONFIG_HOBOT_DRM_MIPI_DSI
		idu->dsi_priv = hb_mipi_dsi_host_register(idu->port, &idu->pdev->dev);
		if (IS_ERR_OR_NULL(idu->csi_priv)) {
			vio_err("Register mipi dsi host priv data filed.[%d]",
				PTR_ERR_OR_ZERO(idu->csi_priv));
			idu->dsi_priv = NULL;
			dc_hw_unregister(idu->hw);
			idu->hw = NULL;
			mipi_csi_dev_unregister(idu->csi_priv);
			idu->csi_priv = NULL;
			atomic_dec(&idu->open_cnt);
			osal_mutex_unlock(&idu->mlock);
			return -EBUSY;
		}
#endif
		enable_irq((uint32_t)idu->irq);
		osal_sema_init(&idu->hw_resource, 1);
		osal_sema_init(&idu->done_resource, 0);
	}
	osal_mutex_unlock(&idu->mlock);
	vio_dbg("[S%d]%s cnt %d\n", vctx->flow_id, __func__,
		 atomic_read(&idu->open_cnt));

	return ret;
}

/**
 * @NO{S09E04C01}
 * @ASIL{B}
 * @brief idu character device node close interface
 * @param[in] *vctx : vpf framework context
 * @param[in] rst_en : Decide whether to rest ip
 * @retval "= 0": success
 * @retval "< 0": failure
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
int32_t idu_close(struct vio_video_ctx *vctx, uint32_t rst_en)
{
	int32_t		      ret = 0;
	struct hobot_idu_dev *idu;
	struct vio_subdev    *vdev;

	vdev = vctx->vdev;
	idu = (struct hobot_idu_dev *)vctx->device;

	if (atomic_dec_and_test(&idu->open_cnt)) {
		ret = idu_vpf_streamoff(vctx);
		if (0 != ret) {
			vio_err("OFF IDU stream error.[%d]", ret);
		}
		disable_irq((uint32_t)idu->irq);
		dc_hw_soft_reset_rd(idu->hw);

		dc_hw_unregister(idu->hw);
		idu->hw = NULL;
		mipi_csi_dev_unregister(idu->csi_priv);
		idu->csi_priv = NULL;
#ifdef CONFIG_HOBOT_DRM_MIPI_DSI
		hb_mipi_dsi_host_unregister(idu->dsi_priv);
		idu->dsi_priv = NULL;
#endif
	}

	return ret;
}

void idu_channel_frame_handle(struct vio_subdev *vdev, struct vio_frame *frame)
{
	int32_t layer, j;
	uint64_t flags;
	struct hobot_idu_dev *idu;
	struct idu_subnode *subnode;
	struct idu_subdev *subdev;
	struct vio_framemgr *framemgr;

	if (frame == NULL) {
		vio_err("Invalid vio frame\n");
		return;
	}

	subdev = container_of(vdev, struct idu_subdev, vdev);
	subnode = subdev->subnode;
	idu = subnode->idu;
	layer = vdev->id;

	if (idu->hw->plane[layer].chan_base.enable) {
		osal_set_bit(layer, &subnode->frame_state);
	}

	framemgr = &vdev->framemgr;
	vio_e_barrier_irqs(framemgr, flags);
	if (0 == osal_test_bit(layer, &subnode->frame_state)) {
		vio_x_barrier_irqr(framemgr, flags);
		return;
	}
	framemgr_print_queues(framemgr);
	frame = peek_frame(framemgr, FS_REQUEST);
	if(frame == NULL) {
		vio_err("no input frame\n");
		vio_x_barrier_irqr(framemgr, flags);
		return;
	}
	trans_frame(framemgr, frame, FS_PROCESS);
	vio_dbg("input frameid(%d), fd[0] = %d, fd[1] = %d  plane_cnt = %d\n",
                frame->frameinfo.frameid.frame_id,
                frame->frameinfo.ion_id[0],
                frame->frameinfo.ion_id[1],
		frame->frameinfo.num_planes);
	osal_clear_bit(layer, &subnode->frame_state);
	vio_x_barrier_irqr(framemgr, flags);

	subnode->src_frames[layer] = frame;

	idu->hw->plane[layer].chan_addr.display_id = idu->port;
	if (frame->frameinfo.num_planes == 0) {
		frame->frameinfo.num_planes = 2;
	}
	for (j = 0; j < frame->frameinfo.num_planes; j++) {
		vio_dbg("%s[%d]:src pddr[%d] = %x\n", __func__,vdev->id, j, frame->vbuf.iommu_paddr[0][j]);
		idu->hw->plane[layer].chan_addr.yuv_address[j] = frame->vbuf.iommu_paddr[0][j];
	}
	if (0 != dc_hw_plane_set_rdaddr(idu->hw, layer)) {
		vio_err("Set plane buffer addr failed\n");
	}

	return;
}

/**
 * @NO{S09E04C01}
 * @ASIL{B}
 * @brief idu configuration word generation interface, which generates configuration words according to configuration parameters
 * @param[in] *subnode : idu pipeline node
 * @param[out] None
 * @retval "= 0": success
 * @retval "< 0": failure
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
// static int32_t idu_generate_cfg_word(struct idu_subnode *subnode)
// {
// 	struct hobot_idu_dev *idu = subnode->idu;
// 	struct idu_cfg *idu_cfg = &subnode->idu_cfg;
// 	idu_init_info_t *init_info = &idu_cfg->init_info;

// 	vio_info("%s:%d\n", __func__, __LINE__);
// 	return ret;
// }

/**
 * @NO{S09E04C01}
 * @ASIL{B}
 * @brief dump idu cfg word for debug
 * @param[in] *vaddr
 * @param[in] roi_num
 * @param[out] None
 * @retval None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void dump_cfg_bin(void *vaddr, uint32_t roi_num)
{
	uint32_t *base = (uint32_t *)vaddr;
	uint32_t  i, j;

	for (i = 0; i < roi_num; i++) {
		vio_dbg("#####################roi[%d]############################\n",
			i);
		for (j = 0; j < 16; j++)
			vio_dbg("cfg word[%d]=%x\n", j, base[j + i * 16]);
	}
}

#define MAX_TIMESTAMPS_GAP 10

/**
 * @NO{S09E04C01}
 * @ASIL{B}
 * @brief The idu worker thread callback function is called by the worker thread created by vpf
 * @param[in] *vnode
 * @param[out] None
 * @retval None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void idu_frame_work(struct vio_node *vnode)
{
	struct idu_subnode   *subnode;
	struct hobot_idu_dev *idu;
	struct vio_framemgr *fmgr;
	struct vio_frame *frame;
	struct vio_subdev *capture_vdev;
	uint64_t flags;
	vio_dbg("%s: C%d frame work start\n", __func__, vnode->ctx_id);
	idu = container_of(vnode, struct hobot_idu_dev, vnode);
	if (!idu->hw->capture.base.enable) {
		vio_warn("%s: Capture is disabled\n", __func__);
		return;
	}

	subnode = &idu->subnode;
	capture_vdev = vnode->och_subdev[IDU_OCH_WRITEBACK];
	fmgr = capture_vdev->cur_fmgr;

	vio_e_barrier_irqs(fmgr, flags);
	frame = peek_frame(fmgr, FS_REQUEST);
	if (NULL != frame) {
		trans_frame(fmgr, frame, FS_PROCESS);
		vio_x_barrier_irqr(fmgr, flags);

		idu->hw->capture.paddr[0] = frame->vbuf.iommu_paddr[0][0];
		idu->hw->capture.paddr[1] = frame->vbuf.iommu_paddr[0][1];
		idu->hw->capture.base.width = idu->hw->display.h_active;
		idu->hw->capture.base.height = idu->hw->display.v_active;
		idu->hw->capture.base.dirty = true;
		vio_dbg("\t buf paddr[0]: 0x%x\n", idu->hw->capture.paddr[0]);
		vio_dbg("\t buf paddr[1]: 0x%x\n", idu->hw->capture.paddr[0]);
#ifdef CONFIG_HOBOT_DISPLAY_WRITEBACK
		if (idu->hw->capture.base.enable) {
			dc_hw_setup_capture(&idu->hw->capture);
		} else {
			dc_hw_disable_capture(&idu->hw->capture);
		}
#endif
	}
}

/**
 * @NO{S09E04C01}
 * @ASIL{B}
 * @brief idu public parameter setting interface
 * @param[in] *vctx : vpf framework context
 * @param[in] arg : input parameter
 * @param[out] None
 * @retval "= 0": success
 * @retval "< 0": failure
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
int32_t idu_set_init_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	int32_t		      ret = 0;
	struct hobot_idu_dev *idu;
	uint32_t	      flow_id = vctx->flow_id;
	struct idu_subnode   *subnode;
	struct idu_subdev    *subdev;
	struct vio_node	     *vnode;

	idu = (struct hobot_idu_dev *)vctx->device;
	subdev = container_of(vctx->vdev, struct idu_subdev, vdev);
	subnode = subdev->subnode;
	vnode = subdev->vnode;
	if (vctx->id == IDU_OCH_WRITEBACK + VNODE_ID_CAP) {
		vctx->vdev->leader = 1;
		vnode->leader = 1;
	}

	return ret;
}

static int32_t idu_set_channel_attr_ex(struct vio_video_ctx *vctx, struct channel_base_cfg_s *cfg)
{
	int32_t ret = 0;
	struct hobot_idu_dev *idu = (struct hobot_idu_dev *)vctx->device;

	ret = idu_ichn_attr_param_check(vctx, cfg);
	if (ret) {
		vio_err("Check idu%d_ich%d parameter check failed, ret %d.\n",
			idu->port, cfg->channel, ret);
		return -EFAULT;
	}

	// step1: set ichn attr to idu->dc_hw
	ret = idu_ichn_attr_param_update(vctx, cfg);
	if (ret) {
		vio_err("Update idu%d_ich%d parameter to plane failed, ret %d.\n",
			idu->port, cfg->channel, ret);
		return -EFAULT;
	}

	// step2: write register
	ret = plane_commit(idu->hw);
	if (ret) {
		vio_err("Commit idu%d_ich%d parameter failed, ret %d.\n",
			idu->port, cfg->channel, ret);
	}

	return ret;
}

/**
 * @NO{S09E04C01}
 * @ASIL{B}
 * @brief a Interface for dynamic modification of public parameters
 * @param[in] *vctx : vpf framework context
 * @param[in] arg : input parameter
 * @param[out] None
 * @retval "= 0": success
 * @retval "< 0": failure
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
int32_t idu_set_ichn_attr_ex(struct vio_video_ctx *vctx, unsigned long arg)
{
	int32_t			  ret = 0;
	struct hobot_idu_dev	 *idu;
	struct idu_subdev	 *subdev;
	struct idu_subnode	 *subnode;
	uint32_t		  copy_ret;
	struct disp_dynamic_cfg_s disp_attr_ex = { 0 };

	idu = (struct hobot_idu_dev *)vctx->device;
	subdev = container_of(vctx->vdev, struct idu_subdev, vdev);
	subnode = subdev->subnode;

	copy_ret = copy_from_user((void *)&disp_attr_ex, (void __user *)arg,
				  sizeof(disp_attr_ex));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %d\n", __func__,
			copy_ret);
		return -EFAULT;
	}

	switch (disp_attr_ex.type) {
		case DISP_DYNAMIC_CHANNEL:
			ret = idu_set_channel_attr_ex(vctx, &disp_attr_ex.channel_cfg);
			if (0 != ret) {
				vio_err("Failed to dynamic set channel base cfg.[%d]\n", ret);
			}
			break;
		default:
			ret = -EINVAL;
			vio_err("Invalid dynamic attr type %d\n", disp_attr_ex.type);
			break;
	}

	return ret;
}



/**
 * @NO{S09E04C01}
 * @ASIL{B}
 * @brief idu input channel parameter setting interface
 * @param[in] *vctx : vpf framework context
 * @param[in] arg : input parameter
 * @param[out] None
 * @retval "= 0": success
 * @retval "< 0": failure
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
int32_t idu_set_ichn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	int32_t			   ret = 0;
	uint32_t		   ichn_id;
	struct hobot_idu_dev	  *idu;
	struct idu_subdev	  *subdev;
	struct idu_subnode	  *subnode;
	struct channel_base_cfg_s *ichn_cfg;

	if (IS_ERR_OR_NULL((void __user *)arg)) {
		vio_err("Invalid arg.\n");
		return -EINVAL;
	}

	if (vctx->id >= IDU_ICH_MAX) {
		vio_err("Invalid ichn vctx->id.\n");
		return -ERANGE;
	}

	ichn_id = vctx->id - VNODE_ID_SRC;
	idu = (struct hobot_idu_dev *)vctx->device;
	subdev = container_of(vctx->vdev, struct idu_subdev, vdev);
	subnode = subdev->subnode;
	ichn_cfg = &subnode->idu_cfg.channel_base_cfg[ichn_id];
	ret = copy_from_user(ichn_cfg, (void __user *)arg,
			     sizeof(struct channel_base_cfg_s));
	if (ret) {
		vio_err("Copy idu%d_ich%d parameter from user failed, ret %d.\n",
			idu->port, ichn_id, ret);
		return -EFAULT;
	}

	vio_dbg("vctx->id=%d set channel %d.\n", vctx->id, ichn_cfg->channel);
	ret = idu_ichn_attr_param_check(vctx, ichn_cfg);
	if (ret) {
		vio_err("Check idu%d_ich%d parameter check failed, ret %d.\n",
			idu->port, ichn_id, ret);
		return -EFAULT;
	}

	// step1: set ichn attr to idu->dc_hw
	ret = idu_ichn_attr_param_update(vctx, ichn_cfg);
	if (ret) {
		vio_err("Update idu%d_ich%d parameter to plane failed, ret %d.\n",
			idu->port, ichn_id, ret);
		return -EFAULT;
	}

	// step2: write register
	ret = plane_commit(idu->hw);
	if (ret) {
		vio_err("Commit idu%d_ich%d parameter failed, ret %d.\n",
			idu->port, ichn_id, ret);
	}

	return ret;
}

static int32_t disp_check_timing_par(const struct disp_timing_s *timing)
{
	if ((timing->hbp > MAX_TIMING_PAR) || (timing->hfp > MAX_TIMING_PAR) ||
	    (timing->hs > MAX_TIMING_PAR) || (timing->vbp > MAX_TIMING_PAR) ||
	    (timing->vfp > MAX_TIMING_PAR) || (timing->vs > MAX_TIMING_PAR)) {
		vio_err("%s: Invalid timing paraters!!\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int32_t idu_output_cfg_ppcon_check(const struct output_cfg_s *cfg)
{
	if (cfg->ppcon1.contrast > MAX_CONTRAST_VAL) {
		(void)vio_err("ppcon1 contrast value exceed 63, exit.\n");
		return -EINVAL;
	}
	if (cfg->ppcon2.theta_abs > MAX_UINT8_VAL) {
		(void)vio_err("theta abs exceed 255, exit.\n");
		return -EINVAL;
	}
	if (cfg->ppcon2.saturation > MAX_UINT8_VAL) {
		(void)vio_err("saturation exceed 255, exit.\n");
		return -EINVAL;
	}
	if (cfg->ppcon2.off_contrast > MAX_UINT8_VAL) {
		(void)vio_err("off contrast exceed 255, exit.\n");
		return -EINVAL;
	}
	if ((cfg->ppcon2.off_bright > MAX_BRIGHT_VAL) &&
	    (cfg->ppcon2.off_bright < MIN_BRIGHT_VAL)) {
		(void)vio_err("off bright value error, exit.\n");
		return -EINVAL;
	}

	return 0;
}

static int32_t idu_ochn_attr_param_check(struct vio_video_ctx *vctx,
					 struct output_cfg_s  *cfg)
{
	int32_t ret = 0;

	if (vctx->id - VNODE_ID_CAP != cfg->out_sel && vctx->id - VNODE_ID_CAP != IDU_OCH_WRITEBACK) {
		vio_err("Invalid output channel select %d-%d.\n", vctx->id, cfg->out_sel);
		return -EINVAL;
	}

	ret = idu_output_cfg_ppcon_check(cfg);
	if (0 != ret) {
		return ret;
	}

	ret = disp_check_timing_par(&cfg->timing);
	if (0 != ret) {
		return ret;
	}

	return 0;
}

static void idu_ochn_display_attr_update(struct output_cfg_s  *cfg,
					 struct dc_hw_display *display)
{
	struct idu_bg_cfg     *bg_cfg = &display->bg;
	struct idu_bchs_cfg   *bchs = &display->bchs;
	struct idu_dither_cfg *dither = &display->dither;

	display->enable = cfg->enable;
	display->bus_format = cfg->out_format;
	display->h_active = cfg->width;
	display->h_total =
		cfg->timing.hbp + cfg->timing.hfp + cfg->timing.hs + cfg->width;
	display->h_sync_start = cfg->width + cfg->timing.hfp;
	display->h_sync_end = display->h_sync_start + cfg->timing.hs;

	display->v_active = cfg->height;
	display->v_total = cfg->timing.vbp + cfg->timing.vfp + cfg->timing.vs +
			   cfg->height;
	display->v_sync_start = cfg->height + cfg->timing.vfp;
	display->v_sync_end = display->v_sync_start + cfg->timing.vs;
	display->mode_clock = cfg->timing.pix_rate;

	bg_cfg->mode = cfg->bgmode;
	bg_cfg->color = cfg->bgcolor;
	bg_cfg->dirty = true;

	dither->dither_flag = cfg->ppcon1.dithering_flag;
	dither->enable = cfg->ppcon1.dithering_en;
	dither->dirty = true;

	bchs->bright_en = cfg->ppcon1.bright_en;
	bchs->off_bright = cfg->ppcon2.off_bright;
	bchs->contrast_en = cfg->ppcon1.con_en;
	bchs->contrast_val = cfg->ppcon1.contrast;
	bchs->off_contrast = cfg->ppcon2.off_contrast;
	bchs->hue_en = cfg->ppcon1.hue_en;
	bchs->theta_sign = cfg->ppcon1.theta_sign;
	bchs->theta_abs = cfg->ppcon2.theta_abs;
	bchs->saturation_en = cfg->ppcon1.sat_en;
	bchs->saturation_val = cfg->ppcon2.saturation;
	bchs->dirty = true;
}

static int32_t idu_ochn_writeback_attr_update(void *cfg, struct dc_hw_capture_cfg *cap)
{
	struct writeback_channel_cfg_s *usr_cfg = (struct writeback_channel_cfg_s *)cfg;

	cap->base.enable = usr_cfg->enable;
	cap->base.source = usr_cfg->point;
	cap->base.format = hbmem_format_to_capture(usr_cfg->format);
	cap->base.dirty = true;

	return 0;
}

static int32_t mipi_csi_tx_pixel_format_to_datatype_bpp(uint32_t  fmt,
							uint32_t *datatype,
							uint32_t *bpp)
{
	int32_t ret = 0;

	switch (fmt) {
	case 0:
		*datatype = 0x1e;
		*bpp = 16;
	}

	return ret;
}

static int32_t idu_ochn_csi_attr_update(void			   *cfg,
					struct mipi_csi_dev_priv_s *priv)
{
	int32_t			    ret = 0;
	struct csi_dev_channel_cfg_s *usr_cfg = (struct csi_dev_channel_cfg_s *)cfg;
	struct mipi_csi_dev_cfg_t  *set_cfg = &priv->cfg;

	set_cfg->enable = usr_cfg->enable;
	set_cfg->lanes = usr_cfg->lanes;
	set_cfg->fps = usr_cfg->fps;
	ret = mipi_csi_tx_pixel_format_to_datatype_bpp(
		usr_cfg->datatype, &set_cfg->datatype, &set_cfg->bpp);
	if (0 != ret) {
		vio_err("No match datatype.\n");
		return ret;
	}
	set_cfg->mipiclk = usr_cfg->mipiclk;
	set_cfg->width = usr_cfg->width;
	set_cfg->height = usr_cfg->height;
	set_cfg->linelenth = usr_cfg->linelenth;
	set_cfg->framelenth = usr_cfg->framelenth;
	set_cfg->settle = usr_cfg->settle;
	set_cfg->vpg = usr_cfg->vpg;
	set_cfg->ipi_lines = usr_cfg->ipi_lines;
	set_cfg->channel_num = usr_cfg->channel_num;
	set_cfg->vpg_mode = usr_cfg->vpg_mode;
	set_cfg->vpg_hsyncpkt_en = usr_cfg->vpg_hsyncpkt_en;
	set_cfg->lpclk_mode = usr_cfg->lpclk_mode;

	return ret;
}
#if IS_ENABLED(CONFIG_HOBOT_DRM_MIPI_DSI)
static int32_t idu_ochn_dsi_attr_update(void			   *cfg,
					struct mipi_dsi_host_priv_s *priv)
{
	struct dsi_channel_cfg_s *usr_cfg = (struct dsi_channel_cfg_s *)cfg;
	struct mipi_dsi_host_cfg_t  *set_cfg = &priv->cfg;

	if (usr_cfg->enable == 0) {
		vio_err("mipi dsi%d config is disable\n", priv->port);
		return -EBUSY;
	}
	set_cfg->enable = usr_cfg->enable;
	set_cfg->lanes = usr_cfg->lanes;
	set_cfg->mipiclk = usr_cfg->mipiclk;
	set_cfg->eotp_rx_en = usr_cfg->eotp_rx_en;
	set_cfg->eotp_tx_en = usr_cfg->eotp_tx_en;
	set_cfg->cntmode = usr_cfg->cntmode;
	set_cfg->color_coding = usr_cfg->color_coding;
	set_cfg->channel = usr_cfg->channel;
	set_cfg->cmd_channel = usr_cfg->cmd_channel;
	set_cfg->hsync_low = usr_cfg->hsync_low;
	set_cfg->vsync_low = usr_cfg->vsync_low;
	set_cfg->dataen_low = usr_cfg->dataen_low;
	set_cfg->width = usr_cfg->width;
	set_cfg->height = usr_cfg->height;
	set_cfg->hsa = usr_cfg->hsa;
	set_cfg->hbp = usr_cfg->hbp;
	set_cfg->hline = usr_cfg->hline;
	set_cfg->vsa = usr_cfg->vsa;
	set_cfg->vbp = usr_cfg->vbp;
	set_cfg->vfp = usr_cfg->vfp;
	set_cfg->video_mode = usr_cfg->video_mode;
	set_cfg->vpg = usr_cfg->vpg;

	return 0;
}
#endif
/**
 * @NO{S09E04C01}
 * @ASIL{B}
 * @brief idu output channel parameter setting interface
 * @param[in] *vctx : vpf framework context
 * @param[in] arg : input parameter
 * @param[out] None
 * @retval "= 0": success
 * @retval "< 0": failure
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
int32_t idu_set_ochn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	int32_t		      ret = 0;
	uint32_t	      ochn_id;
	struct hobot_idu_dev *idu;
	struct idu_subdev    *subdev;
	struct idu_subnode   *subnode;
	struct output_cfg_s  *ochn_cfg;
	struct dc_hw	     *hw;
	struct dc_hw_display *display;

	if (IS_ERR_OR_NULL((void __user *)arg)) {
		vio_err("Invalid arg.\n");
		return -EINVAL;
	}

	if (vctx->id < VNODE_ID_CAP || vctx->id > VNODE_ID_CAP + IDU_OCH_MAX) {
		vio_err("Invalid ochn vctx->id %d.\n", vctx->id);
		return -ERANGE;
	}

	ochn_id = vctx->id - VNODE_ID_CAP;
	idu = (struct hobot_idu_dev *)vctx->device;
	hw = idu->hw;
	subdev = container_of(vctx->vdev, struct idu_subdev, vdev);
	subnode = subdev->subnode;
	ochn_cfg = &subnode->idu_cfg.output_cfg;
	ret = copy_from_user(ochn_cfg, (void __user *)arg,
			     sizeof(struct output_cfg_s));
	if (ret) {
		vio_err("copy ochn attr from user failed.\n");
		return -EFAULT;
	}

	if (g_debug_priv[idu->port]->config_log) {
		disp_output_cfg_log(ochn_cfg);
	}

	ret = idu_ochn_attr_param_check(vctx, ochn_cfg);
	if (0 != ret) {
		vio_err("ochn cfg check failed.\n");
		return ret;
	}

	display = &hw->display;
	display->id = idu->port;
	idu_ochn_display_attr_update(ochn_cfg, display);

	ret = hw->func->display(hw);
	if (0 != ret) {
		vio_err("ochn display attribute setup failed.\n");
		return ret;
	}

	if (ochn_cfg->out_sel == IDU_OCH_MIPI_CSI_DEV) {
		ret = idu_ochn_csi_attr_update(&ochn_cfg->csi_tx_cfg,
					       idu->csi_priv);
		if (0 != ret) {
			vio_err("mipi csi dev channel cfg update failed.[%d]\n",
				ret);
			return ret;
		}
	} else if (ochn_cfg->out_sel == IDU_OCH_MIPI_DSI) {
#if IS_ENABLED(CONFIG_HOBOT_DRM_MIPI_DSI)
		ret = idu_ochn_dsi_attr_update(&ochn_cfg->dsi_cfg, idu->dsi_priv);
		if (0 != ret) {
			vio_err("mipi dsi dev channel cfg update failed.[%d]\n",
				ret);
			return ret;
		}
#endif
	} else {
		vio_err("idu output select failed.[%d]\n", ochn_cfg->out_sel);
		return -EINVAL;
	}

	if (ochn_id == IDU_OCH_WRITEBACK) {
		idu_ochn_writeback_attr_update(&ochn_cfg->wb_cfg, &idu->hw->capture);
		vio_dbg("writeback channel cfg update.\n");
	}

	return ret;
}

static int32_t idu_set_output_attr_ex(struct vio_video_ctx *vctx, struct output_cfg_s *cfg)
{
	int32_t ret = 0;
	struct hobot_idu_dev *idu = (struct hobot_idu_dev *)vctx->device;

	ret = idu_ochn_attr_param_check(vctx, cfg);
	if (ret) {
		vio_err("Check output parameter check failed, ret %d.\n", ret);
		return -EFAULT;
	}

	// step1: set display attr to idu->dc_hw
	idu_ochn_display_attr_update(cfg, &idu->hw->display);

	// step2: write register
	ret = idu->hw->func->display(idu->hw);
	if (0 != ret) {
		vio_err("Dynamic display attribute setup failed.\n");
		return ret;
	}

	return ret;
}

int32_t idu_set_ochn_attr_ex(struct vio_video_ctx *vctx, unsigned long arg)
{
	int32_t			  ret = 0;
	struct hobot_idu_dev	 *idu;
	struct idu_subdev	 *subdev;
	struct idu_subnode	 *subnode;
	uint32_t		  copy_ret;
	struct disp_dynamic_cfg_s disp_attr_ex = { 0 };

	idu = (struct hobot_idu_dev *)vctx->device;
	subdev = container_of(vctx->vdev, struct idu_subdev, vdev);
	subnode = subdev->subnode;

	copy_ret = copy_from_user((void *)&disp_attr_ex, (void __user *)arg,
				  sizeof(disp_attr_ex));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %d\n", __func__,
			copy_ret);
		return -EFAULT;
	}

	switch (disp_attr_ex.type) {
		case DISP_DYNAMIC_OUTPUT:
			ret = idu_set_output_attr_ex(vctx, &disp_attr_ex.output_cfg);
			if (0 != ret) {
				vio_err("Failed to dynamic set output cfg.[%d]\n", ret);
			}
			break;
		default:
			ret = -EINVAL;
			vio_err("Invalid dynamic attr type %d\n", disp_attr_ex.type);
			break;
	}

	return ret;
}

int32_t idu_allow_bind(struct vio_subdev *vdev, struct vio_subdev *remote_vdev, u8 online_mode)
{
	enum vio_bind_type bind_type;

	bind_type = CHN_BIND_M2M;
	if (osal_test_bit((s32)VIO_SUBDEV_BIND_DONE, &vdev->state) != 0)
		bind_type = CHN_BIND_REP;

	return bind_type;
}