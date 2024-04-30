// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include "cam_dev.h"
#include "cam_ctrl.h"
#include "csi_dphy_regs.h"
#include "csi_regs.h"
#include "csi_uapi.h"
#include "csi_wrapper.h"
#include "isc.h"

#include "csi.h"

static void csi_bound(struct isc_handle *isc, void *arg)
{
	struct csi_device *csi = (struct csi_device *)arg;
	unsigned long flags;

	if (csi) {
		spin_lock_irqsave(&csi->isc_lock, flags);
		if (!csi->isc) {
			isc_get(isc);
			csi->isc = isc;
		}
		spin_unlock_irqrestore(&csi->isc_lock, flags);
	}
}

static void csi_unbind(void *arg)
{
	struct csi_device *csi = (struct csi_device *)arg;
	unsigned long flags;

	if (csi) {
		spin_lock_irqsave(&csi->isc_lock, flags);
		if (csi->isc) {
			isc_put(csi->isc);
			csi->isc = NULL;
		}
		spin_unlock_irqrestore(&csi->isc_lock, flags);
	}
}

static struct isc_notifier_ops csi_notifier_ops = {
	.bound = csi_bound,
	.unbind = csi_unbind,
	.got = csi_msg_handler,
};

static void csi_set_base_ex(struct csi_device *csi)
{
	struct device_node *cam_node = of_get_parent(csi->dev->of_node);
	struct device_node *csi_node_ex = NULL;
	void __iomem *csi_base_ex = NULL;
	s32 csi_id_ex, id = -1;
	int rc;

	switch (csi->id) {
	case 0:
		csi_id_ex = 1;
		break;
	case 1:
		csi_id_ex = 0;
		break;
	case 2:
		csi_id_ex = 3;
		break;
	case 3:
		csi_id_ex = 2;
		break;
	default:
		csi_id_ex = -1;
		break;
	}

	if (csi_id_ex < 0)
		goto _exit;

	do {
		csi_node_ex = of_get_next_child(cam_node, csi_node_ex);
		if (!csi_node_ex)
			break;

		if (strncmp(csi_node_ex->name, "csi", 3))
			continue;
		rc = of_property_read_u32(csi_node_ex, "id", &id);
		if (rc < 0)
			continue;
		if (id == csi_id_ex)
			break;
	} while (csi_node_ex);

	if (id != csi_id_ex || !csi_node_ex)
		goto _exit;

	csi_base_ex = of_iomap(csi_node_ex, 0);
_exit:
	csi->base_ex = csi_base_ex;
}

int csi_probe(struct platform_device *pdev, struct csi_device *csi, bool is_native)
{
	struct device *dev = &pdev->dev;
	int rc;
	struct mem_res csi_mems[] = {
		{ "reg", NULL },
		{},
	};
	struct irq_res csi_irqs[] = {
		{ "csi", -1, csi_irq_handler, csi },
		{},
	};
	struct clk_res csi_clks[] = {
		{ "ipi", NULL },
		{ "pclk", NULL },
		{ "cfg", NULL },
		{},
	};
	struct rst_res csi_rsts[] = {
		{ "rst", NULL },
		{},
	};
	struct cam_dt csi_dt = {
		.id = 0,
		.num_insts = 0,
		.mems = csi_mems,
		.irqs = csi_irqs,
		.clks = csi_clks,
		.rsts = csi_rsts,
	};

	if (!csi)
		return -EINVAL;

	rc = parse_cam_dt(pdev, &csi_dt, csi);
	if (rc < 0) {
		dev_err(dev, "failed to call parse_cam_dt (err=%d)\n", rc);
		return rc;
	}

	csi->dev = dev;
	csi->id = csi_dt.id;
	csi->num_insts = csi_dt.num_insts;
	csi->base = csi_dt.mems[0].base;
	csi->ipi = csi_dt.clks[0].clk;
	csi->pclk = csi_dt.clks[1].clk;
	csi->cfg = csi_dt.clks[2].clk;
	csi->rst = csi_dt.rsts[0].rst;
	csi_set_base_ex(csi);
	csi->is_native = is_native;
	spin_lock_init(&csi->isc_lock);
	if (!csi->ipi)
		csi->ipiclk = CSI_IPICLK_DEFAULT;
	else
		csi->ipiclk = clk_get_rate(csi->ipi);

	csi->csiw_dev = get_csi_wrapper_device(pdev);
	if (IS_ERR(csi->csiw_dev))
		return PTR_ERR(csi->csiw_dev);

	csi->cam_ctl_dev = get_cam_ctrl_device(pdev);
	if (IS_ERR(csi->cam_ctl_dev))
		return PTR_ERR(csi->cam_ctl_dev);

	rc = isc_register(CSI_UID(csi->id), &csi_notifier_ops, csi);
	if (rc < 0) {
		dev_err(dev, "failed to call isc_register (err=%d)\n", rc);
		return rc;
	}

	dev_dbg(dev, "SNPS CSI driver #%d (base) probed done\n", csi->id);
	return 0;
}

int csi_remove(struct platform_device *pdev, struct csi_device *csi)
{
	int rc;

	rc = isc_unregister(CSI_UID(csi->id));
	if (rc < 0)
		dev_err(&pdev->dev, "failed to call isc_unregister (err=%d)\n", rc);

	put_csi_wrapper_device(csi->csiw_dev);
	dev_dbg(&pdev->dev, "SNPS CSI driver #%d (base) removed\n", csi->id);
	return 0;
}

static int csi_fmt_2_type_info(struct cam_format *fmt, bool *color_mode16,
			       u32 *data_type)
{
	bool color_m16;
	u32 dt;

	switch (fmt->format) {
	case CAM_FMT_RAW8:
		color_m16 = true;
		dt = DATA_TYPE_RAW8;
		break;
	case CAM_FMT_RAW10:
		color_m16 = true;
		dt = DATA_TYPE_RAW10;
		break;
	case CAM_FMT_RAW12:
		color_m16 = true;
		dt = DATA_TYPE_RAW12;
		break;
	case CAM_FMT_YUYV:
		color_m16 = false;
		dt = DATA_TYPE_YUV422_8;
		break;
	case CAM_FMT_NV12:
		color_m16 = false;
		dt = DATA_TYPE_YUV420_SHIFT;
		break;
	case CAM_FMT_RGB888X:
		color_m16 = false;
		dt = DATA_TYPE_RGB888;
		break;
	default:
		return -EINVAL;
	}

	if (color_mode16)
		*color_mode16 = color_m16;
	if (data_type)
		*data_type = dt;
	return 0;
}

static int csi_get_hsd_time(struct csi_device *dev, struct cam_format *fmt,
			    struct csi_ipi_base_cfg *ipi_cfg)
{
	/**
	 * Rxbyteclk = (Rxbitclk / Number_of_lanes) / 8
	 * Rxbyteclk_per = 1 / Rxbyteclk
	 * Bytes_to_transmi = BitsPerPixel * Line_size / 8
	 * time to transmit last pixel in PPI
	 * = (Bytes_to_transmit / Number_of_lanes) *  Rxbyteclk_per
	 * = ((BitsPerPixel * Line_size / 8) / Number_of_lanes) /((Rxbitclk /  Number_of_lanes) / 8)
	 * = (BitsPerPixel * Line_size) / Rxbitclk
	 *
	 * pixel_clk_per = 1 / pixel_clk
	 * time to transmit last pixel in IPI
	 * = (h_sa + hbp + hsd + cycles_to_transmit_data) * pixel_clk_per
	 * = (h_sa + hbp + hsd + cycles_to_transmit_data) / pixel_clk
	 *
	 * T(ppi) < T(ipi) ==>
	 * BitsPerPixel * Line_size * pixel_clk / Rxbitclk < (h_sa + hbp + hsd + cycles_to_trans) ==>
	 *  hsd > BitsPerPixel * Line_size * pixel_clk / Rxbitclk - (cycles_to_trans+h_sa+ hbp)
	 *  cycles_to_trans = Line_size in 48 bits IPI
	 *
	 */
	u64 rx_bit_clk = 0;
	u64 bits_per_pixel = 0;
	u64 line_size = 0;
	u64 cycles_to_trans = 0;
	u64 time_ppi = 0;
	u64 time_ipi = 0;
	u64 pixclk = dev->ipiclk;
	u32 datatype;
	int yuv_cycle, raw_pixel;
	bool color_mode16;

	if (csi_fmt_2_type_info(fmt, &color_mode16, &datatype)) {
		dev_err(dev->dev, "not supported format 0x%x\n", fmt->format);
		return -EINVAL;
	}
	if (color_mode16) {
		yuv_cycle = 2;
		raw_pixel = 1;
	} else {
		yuv_cycle = 1;
		raw_pixel = 3;
	}

	switch (datatype) {
	case DATA_TYPE_YUV420_8:
	case DATA_TYPE_YUV420_SHIFT:
		bits_per_pixel = 8 * 3 / 2;
		cycles_to_trans = (u64)fmt->width * yuv_cycle;
		break;
	case DATA_TYPE_YUV420_10:
		bits_per_pixel = 16 * 3 / 2;
		cycles_to_trans = (u64)fmt->width * yuv_cycle;
		break;
	case DATA_TYPE_YUV422_8:
		bits_per_pixel = 8 * 2;
		cycles_to_trans = (u64)fmt->width * yuv_cycle;
		break;
	case DATA_TYPE_YUV422_10:
		bits_per_pixel = 16 * 2;
		cycles_to_trans = (u64)fmt->width * yuv_cycle;
		break;
	case DATA_TYPE_RGB888:
		bits_per_pixel = 24;
		cycles_to_trans = (u64)(fmt->width + 2) / raw_pixel;
		break;
	case DATA_TYPE_RAW8:
		bits_per_pixel = 8;
		cycles_to_trans = (u64)(fmt->width + 2) / raw_pixel;
		break;
	case DATA_TYPE_RAW10:
		bits_per_pixel = 10;
		cycles_to_trans = (u64)(fmt->width + 2) / raw_pixel;
		break;
	case DATA_TYPE_RAW12:
		bits_per_pixel = 12;
		cycles_to_trans = (u64)(fmt->width + 2) / raw_pixel;
		break;
	case DATA_TYPE_RAW14:
		bits_per_pixel = 14;
		cycles_to_trans = (u64)(fmt->width + 2) / raw_pixel;
		break;
	default:
		bits_per_pixel = 16;
		break;
	}
	rx_bit_clk = (u64)dev->lane_rate * 1000000 * dev->lanes / 2;
	line_size = fmt->width;

	dev_dbg(dev->dev, "bits_per_pixel: %llu, ipiclk: %llu, rx_bit_clk: %llu\n",
		bits_per_pixel, pixclk, rx_bit_clk);
	time_ppi = bits_per_pixel * 1000 * line_size * 1000000 / rx_bit_clk;
	dev_dbg(dev->dev, "time to transmit last pixel in ppi: %llu\n", time_ppi);
	if ((bits_per_pixel * line_size * pixclk / rx_bit_clk) > ((u64)ipi_cfg->hsa + ipi_cfg->hbp + cycles_to_trans))
		ipi_cfg->hsd = (u32)((bits_per_pixel * line_size * pixclk / rx_bit_clk) - ((u64)ipi_cfg->hsa + ipi_cfg->hbp + cycles_to_trans));
	else
		ipi_cfg->hsd = 0;

	if (ipi_cfg->hsd == 0) {
		ipi_cfg->hsd = CSI_HSDTIME_DEFAULT;
		dev_dbg(dev->dev, "default hsdtime: %d", ipi_cfg->hsd);
	}

	if (ipi_cfg->hsd > CSI_HSDTIME_MAX) {
		ipi_cfg->hsd = CSI_HSDTIME_MAX;
		dev_dbg(dev->dev, "hsdtime max as: %d", ipi_cfg->hsd);
	}
	time_ipi = 1000 * (u64)(ipi_cfg->hsa + ipi_cfg->hbp + ipi_cfg->hsd + cycles_to_trans) * 1000000 / pixclk;
	dev_dbg(dev->dev, "time to transmit last pixel in ipi: %llu", time_ipi);
	return 0;
}

static int csi_get_hsd_time_cutthrough(struct csi_device *dev,
				       struct csi_ipi_base_cfg *ipi_cfg)
{
	/**
	 * ipi_hsd_time > ((2*idi_clk_period)/ipi_clk_period)/(nlanes_active/4)) + 6
	 * ipi_hsd_time > ((2*ipi_clk)/rx_lane_byte_clk)/(nlanes_active/4)) + 6
	 * ipi_hsd_time > ((8*ipi_clk)/rx_lane_byte_clk)/nlanes_active) + 6
	 * ipi_hsd_time > (8*ipi_clk/rx_lane_byte_clk*nlanes_active) + 6
	 * ipi_hsd_time > (8*ipi_clk/rx_byte_clk) + 6
	 */
	u64 pixclk = dev->ipiclk;
	u64 rx_bit_clk = (u64)dev->lane_rate * 1000000 * dev->lanes / 2;
	u16 hsd = (u16)(((pixclk * 8) / (rx_bit_clk / 8)) + CSI_HSD_CAL_MIN);
	u16 hbp = ipi_cfg->hbp;
	u16 hsa = ipi_cfg->hsa;

	dev_dbg(dev->dev, "ipiclk: %llu, rx_bit_clk: %llu\n", pixclk, rx_bit_clk);

	if(hsd > (CSI_HSDTIME_MAX - CSI_HSDTIME_MIN)) {
		hbp = hbp + (hsd - CSI_HSDTIME_MAX);
		hsd = CSI_HSDTIME_MAX;
		dev_dbg(dev->dev, "hsdtime max as: %u\n", hsd);
		if (hbp > (CSI_HBPTIME_MAX - CSI_HBPTIME_MIN)) {
			hsa = hsa + (hbp - CSI_HBPTIME_MAX);
			hbp = CSI_HBPTIME_MAX;
			dev_dbg(dev->dev, "hbptime max as: %u\n", hbp);
			if (hsa > (CSI_HSATIME_MAX - CSI_HSATIME_MIN)) {
				hsa = CSI_HSATIME_MAX;
				dev_dbg(dev->dev, "hsatime max as: %u\n", hsa);
			} else {
				hsa += CSI_HSATIME_MIN;
			}
		} else {
			hbp += CSI_HBPTIME_MIN;
		}
	} else {
		hsd += CSI_HSDTIME_MIN;
	}

	ipi_cfg->hbp = hbp;
	ipi_cfg->hsa = hsa;
	ipi_cfg->hsd = hsd;
	return 0;
}

static u32 csi_get_pkt2pkt_time(struct csi_device *dev)
{
	u64 ipiclk = dev->ipiclk;
	u32 bytes_per_hsclk = 1;
	u32 bits_per_byte = 8;
	u32 dflt_fsync_type = 2;
	u32 t_ppi_hs = 1000 * dev->lanes * (bytes_per_hsclk * bits_per_byte) / (dev->lane_rate / 2);

	return (4 + 2 * dflt_fsync_type) * t_ppi_hs / bytes_per_hsclk +
		(4 + dflt_fsync_type) * (1000 / ipiclk) + 1;
}

void csi_set_tpg_mode(struct csi_device *dev, u32 tpg_en, u32 pkt2pkt_time,
		      u8 horizontal, u8 vc, u8 datatype)
{
	dev->tpg_enable = tpg_en ? true : false;
	dev->tpg_horizontal = horizontal ? true : false;
	dev->tpg_vc = vc;
	dev->tpg_dt = datatype;
	dev->pkt2pkt_time = pkt2pkt_time;
	csi_phy_enable_ppipg_clk(dev->csiw_dev, dev->tpg_enable);
}

static u32 get_ipi_base_from_id(u32 id)
{
	if (id == 0)
		return CSI_IPI1_BASE(1);
	else if (id < 4)
		return CSI_IPI2_4BASE(id + 1);
	return CSI_IPI5_8BASE(id + 1);
}

int csi_ipi_set_vc_cfg(struct csi_device *csi, u32 ipi_id,
		       struct csi_vc_cfg *vc_cfg)
{
	u32 ipi_base;
	union ipi_mode mode;
	union ipi_data_type data_type;

	ipi_base = get_ipi_base_from_id(ipi_id);
	csi_write(csi, CSI_IPIn_VCID(ipi_base), vc_cfg->vc_id);

	data_type.value = csi_read(csi, CSI_IPIn_DATA_TYPE(ipi_base));
	data_type.embedded_data = !!vc_cfg->vc_ebd_en;
	csi_write(csi, CSI_IPIn_DATA_TYPE(ipi_base), data_type.value);

	if (vc_cfg->vc_mode == CSI_VC_OFF)
		return 0;
	mode.value = csi_read(csi, CSI_IPIn_MODE(ipi_base));

	if (vc_cfg->vc_mode == CSI_VC_CTRL && ipi_id != 0) {
		dev_err(csi->dev, "ipi%d not support ctrl mode\n", ipi_id);
		return -EINVAL;
	}
	if (vc_cfg->vc_mode == CSI_VC_CAMERA) {
		mode.ipi_mode_ctrl = 0;
		if (ipi_id == 0)
			csi->ipi0_ctrlmode = false;
	} else {
		mode.ipi_mode_ctrl = 1;
		if (ipi_id == 0)
			csi->ipi0_ctrlmode = true;
	}
	csi_write(csi, CSI_IPIn_MODE(ipi_base), mode.value);
	dev_dbg(csi->dev, "ipi%d vc %d, ctrlmode %d, embedded data en %d\n",
		ipi_id, vc_cfg->vc_id, vc_cfg->vc_mode, vc_cfg->vc_ebd_en);
	return 0;
}

#ifndef PLATFORM_ASIC
static void csi_phys_on_same_bank_release_reset(struct csi_device *csi)
{
	csiex_write(csi, CSI_PHY_RSTZ, 1);
	csiex_write(csi, CSI_PHY_SHUTDOWNZ, 1);
	csiex_write(csi, CSI_RESETN, 1);

	csi_write(csi, CSI_PHY_RSTZ, 1);
	csi_write(csi, CSI_PHY_SHUTDOWNZ, 1);
	csi_write(csi, CSI_RESETN, 1);
}
#endif

void csi_dphy_reset(struct csi_device *csi)
{
	csi_write(csi, CSI_RESETN, 0);
	csi_write(csi, CSI_PHY_RSTZ, 0);
	csi_write(csi, CSI_PHY_SHUTDOWNZ, 0);
}

int csi_set_lane_rate(struct csi_device *csi, u32 rate)
{
	dev_dbg(csi->dev, "phy%d lane rate %d\n", csi->id, rate);

	csi->lane_rate = rate;
	return 0;
}

void csi_set_lanes(struct csi_device *csi, u32 lanes, u32 vcext)
{
	int rc;

	dev_dbg(csi->dev, "phy%d lane num %d\n", csi->id, lanes);
	csi->lanes = lanes;

	csi_write(csi, CSI_N_LANES, csi->lanes - 1);
	csi_write(csi, CSI_PHY_CFG, 0);
	// 1: legacy mode, vc extension disable
	// 0: vc extension enable
	csi_write(csi, CSI_VC_EXTENSION, !!vcext);
#ifdef PLATFORM_ASIC
	csi_dphy_reset(csi);
	rc = csi_dphy_init(csi);
	if (rc)
		dev_err(csi->dev, "csi dphy init err(%d)\n", rc);
#else
	// for FPGA, PHY_RSTZ and SHUTDOWNZ can not both be set to 0 in the same bank
	csi_write(csi, CSI_PHY_RSTZ, 0);
	csiex_write(csi, CSI_PHY_RSTZ, 0);
	csi_write(csi, CSI_RESETN, 0);
	csiex_write(csi, CSI_RESETN, 0);
	msleep(1);

	csi_phys_on_same_bank_release_reset(csi);

	rc = csi_phy_enableclk(csi->csiw_dev, csi->id);
	if (rc)
		dev_err(csi->dev, "phy enable clk err(%d)\n", rc);
#endif
}

int csi_ipi_init(struct csi_device *csi, struct csi_ipi_base_cfg *ipi_cfg, struct cam_format *fmt)
{
	u32 ipi_base, data_type, val;
	bool color_mode16 = false;
	int rc = 0;

	if (!csi || !ipi_cfg || (ipi_cfg->id >= (csi->num_insts - 1)))
		return -EINVAL;

	// raw as 16bit, others as 48bit
	if (csi_fmt_2_type_info(fmt, &color_mode16, &data_type)) {
		dev_err(csi->dev, "not supported format 0x%x\n", fmt->format);
		return -EINVAL;
	}
	dev_dbg(csi->dev, "ipi%d resolution %dx%d, color16 %d, data_type 0x%x\n",
		ipi_cfg->id, fmt->width, fmt->height, color_mode16, data_type);

	ipi_base = get_ipi_base_from_id(ipi_cfg->id);

	{
		union ipi_data_type dt;

		if (csi->tpg_enable && (data_type != DATA_TYPE_RGB888)) {
			dev_dbg(csi->dev, "datatype 0x%x to RGB888 for tpg\n", data_type);
			data_type = DATA_TYPE_RGB888;
		}
		dt.value = csi_read(csi, CSI_IPIn_DATA_TYPE(ipi_base));
		dt.ipi_data_type = data_type;
		csi_write(csi, CSI_IPIn_DATA_TYPE(ipi_base), dt.value);
	}

	if (ipi_cfg->mem_auto_flush)
		csi_write(csi, CSI_IPIn_MEM_FLUSH(ipi_base), CSI_IPI_MEM_AUTO_FLUSH);
	else
		csi_write(csi, CSI_IPIn_MEM_FLUSH(ipi_base), 0);

	{
		union ipi_mode mode;

		mode.value = csi_read(csi, CSI_IPIn_MODE(ipi_base));
		mode.ipi_color16 = color_mode16 ? 1 : 0;
		mode.ipi_cut_through = ipi_cfg->cut_through ? 1 : 0;
		csi_write(csi, CSI_IPIn_MODE(ipi_base), mode.value);
	}

	if (!csi->is_native) {
		ipi_cfg->hsa = CSI_HSATIME_DEFAULT;
		ipi_cfg->hbp = CSI_HBPTIME_DEFAULT;
		if (ipi_cfg->cut_through)
			rc = csi_get_hsd_time_cutthrough(csi, ipi_cfg);
		else
			rc = csi_get_hsd_time(csi, fmt, ipi_cfg);
		if (rc)
			return rc;
	}
	csi_write(csi, CSI_IPIn_HSA_TIME(ipi_base), ipi_cfg->hsa);
	csi_write(csi, CSI_IPIn_HBP_TIME(ipi_base), ipi_cfg->hbp);
	csi_write(csi, CSI_IPIn_HSD_TIME(ipi_base), ipi_cfg->hsd);

	if (csi->tpg_enable) {
		union csi_ppi_pg_config pg_cfg;

		csi_write(csi, CSI_SCRAMBLING, 0);
		csi_write(csi, PPI_PG_PATTERN_VRES, fmt->height);

		if (csi->pkt2pkt_time == 0)
			csi->pkt2pkt_time = csi_get_pkt2pkt_time(csi);
		csi_write(csi, PPI_PG_PATTERN_HRES, fmt->width |
			  CSI_TPG_PKT2PKT_TIME(csi->pkt2pkt_time));

		pg_cfg.value = csi_read(csi, PPI_PG_CONFIG);
		pg_cfg.pattern_horizontal = csi->tpg_horizontal ? 1 : 0;
		pg_cfg.data_type = csi->tpg_dt;
		pg_cfg.vc = csi->tpg_vc;
		csi_write(csi, PPI_PG_CONFIG, pg_cfg.value);
	}
	csi_write(csi, CSI_IPIn_ADV_FEATURES(ipi_base), ipi_cfg->adv_val);

	val = csi_read(csi, CSI_IPI_SOFTRSTN);
	val &= ~CSI_IPIn_RSTN(ipi_cfg->id);
	csi_write(csi, CSI_IPI_SOFTRSTN, val);
	val |= CSI_IPIn_RSTN(ipi_cfg->id);
	csi_write(csi, CSI_IPI_SOFTRSTN, val);
	return rc;
}

#ifdef DEBUG
void csi_reg_dump(struct csi_device *csi)
{
	dev_dbg(csi->dev, "N_LANES %d\n", csi_read(csi, CSI_N_LANES));
	dev_dbg(csi->dev, "CSI2_RESETN %d\n", csi_read(csi, CSI_RESETN));
	dev_dbg(csi->dev, "CSI_INT_ST_MAIN 0x%x\n", csi_read(csi, CSI_INT_ST_MAIN));
	dev_dbg(csi->dev, "DATA_IDS_1 0x%x\n", csi_read(csi, CSI_DATA_IDS_1));
	dev_dbg(csi->dev, "PHY_CFG PPI16 %d\n", csi_read(csi, CSI_PHY_CFG));
	dev_dbg(csi->dev, "PHY_MODE CPHY %d\n", csi_read(csi, CSI_PHY_MODE));
	dev_dbg(csi->dev, "INT_ST_AP_MAIN 0x%x\n", csi_read(csi, CSI_INT_ST_AP_MAIN));
	dev_dbg(csi->dev, "DATA_IDS_VC_1 0x%x\n", csi_read(csi, CSI_DATA_IDS_VC_1));
	dev_dbg(csi->dev, "PHY_SHUTDOWNZ %d\n", csi_read(csi, CSI_PHY_SHUTDOWNZ));
	dev_dbg(csi->dev, "DPHY_RSTZ %d\n", csi_read(csi, CSI_PHY_RSTZ));
	dev_dbg(csi->dev, "PHY_RX 0x%x\n", csi_read(csi, CSI_PHY_RX));
	dev_dbg(csi->dev, "PHY_STOPSTATE 0x%x\n", csi_read(csi, CSI_PHY_STOPSTATE));
	dev_dbg(csi->dev, "PPI_PG_PATTERN_VRES %d\n", csi_read(csi, PPI_PG_PATTERN_VRES));
	dev_dbg(csi->dev, "PPI_PG_PATTERN_HRES %d\n", csi_read(csi, PPI_PG_PATTERN_HRES));
	dev_dbg(csi->dev, "PPI_PG_CONFIG 0x%x\n", csi_read(csi, PPI_PG_CONFIG));
	dev_dbg(csi->dev, "PPI_PG_ENABLE %d\n", csi_read(csi, PPI_PG_ENABLE));
	dev_dbg(csi->dev, "PPI_PG_STATUS %d\n", csi_read(csi, PPI_PG_STATUS));
	dev_dbg(csi->dev, "IPI_MODE 0x%x\n", csi_read(csi, CSI_IPI_MODE));
	dev_dbg(csi->dev, "IPI_VCID 0x%x\n", csi_read(csi, CSI_IPI_VCID));
	dev_dbg(csi->dev, "IPI_DATA_TYPE 0x%x\n", csi_read(csi, CSI_IPI_DATA_TYPE));
	dev_dbg(csi->dev, "IPI_HSA_TIME %d\n", csi_read(csi, CSI_IPI_HSA_TIME));
	dev_dbg(csi->dev, "IPI_HBP_TIME %d\n", csi_read(csi, CSI_IPI_HBP_TIME));
	dev_dbg(csi->dev, "IPI_HSD_TIME %d\n", csi_read(csi, CSI_IPI_HSD_TIME));
	dev_dbg(csi->dev, "IPI_HLINE_TIME %d\n", csi_read(csi, CSI_IPI_HLINE_TIME));
	dev_dbg(csi->dev, "IPI_SOFTRSTN 0x%x\n", csi_read(csi, CSI_IPI_SOFTRSTN));
	dev_dbg(csi->dev, "IPI_ADV_FEATURE 0x%x\n", csi_read(csi, CSI_IPI_ADV_FEATURES));
	dev_dbg(csi->dev, "IPI_VSA_LINES %d\n", csi_read(csi, CSI_IPI_VSA_LINES));
	dev_dbg(csi->dev, "IPI_VBP_LINES %d\n", csi_read(csi, CSI_IPI_VBP_LINES));
	dev_dbg(csi->dev, "IPI_VFP_LINES %d\n", csi_read(csi, CSI_IPI_VFP_LINES));
	dev_dbg(csi->dev, "IPI_VACTIVE_LINES %d\n", csi_read(csi, CSI_IPI_VACTIVE_LINES));
}
#endif

int csi_set_tpg(struct csi_device *csi, bool is_en)
{
	u32 val;

	if (!is_en) {
		csi_write(csi, PPI_PG_ENABLE, 0);
		return 0;
	}
	val = csi_read(csi, PPI_PG_STATUS);
	if (val)
		return 0;
	csi_write(csi, PPI_PG_ENABLE, CSI_TPG_ENABLE);
	return 0;
}

int csi_ipi_start(struct csi_device *csi, u32 ipi_id)
{
	u32 ipi_base;
	union ipi_mode mode;

	if (!csi || (ipi_id >= csi->num_insts - 1))
		return -EINVAL;

	ipi_base = get_ipi_base_from_id(ipi_id);
	mode.value = csi_read(csi, CSI_IPIn_MODE(ipi_base));
	mode.ipi_enable = 1;
	csi_write(csi, CSI_IPIn_MODE(ipi_base), mode.value);

	if (csi->tpg_enable)
		csi_set_tpg(csi, true);
	if (csi->is_bypass)
		set_idi_bypass_select(csi->cam_ctl_dev, csi->id, true);

	dev_dbg(csi->dev, "ipi%d start\n", ipi_id);
#ifdef DEBUG
	csi_reg_dump(csi);
#endif
	return 0;
}

int csi_ipi_stop(struct csi_device *csi, u32 ipi_id)
{
	u32 ipi_base;
	union ipi_mode mode;

	if (!csi || (ipi_id >= csi->num_insts - 1))
		return -EINVAL;

	if (csi->tpg_enable)
		csi_set_tpg(csi, false);

	ipi_base = get_ipi_base_from_id(ipi_id);
	mode.value = csi_read(csi, CSI_IPIn_MODE(ipi_base));
	mode.ipi_enable = 0;
	csi_write(csi, CSI_IPIn_MODE(ipi_base), mode.value);

	if (csi->is_bypass)
		set_idi_bypass_select(csi->cam_ctl_dev, csi->id, false);

	csi_write(csi, CSI_RESETN, 0);
	return 0;
}

void csi_ipi_reset_and_down(struct csi_device *csi, u32 ipi_id, bool enable)
{
	u32 ipi_base, val;
	union ipi_mode mode;

	if (!csi || (ipi_id >= csi->num_insts - 1))
		return;

	ipi_base = get_ipi_base_from_id(ipi_id);
	val = csi_read(csi, CSI_IPI_SOFTRSTN);
	mode.value = csi_read(csi, CSI_IPIn_MODE(ipi_base));
	if (enable) {
		csi_write(csi, CSI_IPI_SOFTRSTN, val | CSI_IPIn_RSTN(ipi_id));
		mode.ipi_enable = 1;
		csi_write(csi, CSI_IPIn_MODE(ipi_base), mode.value);
	} else {
		mode.ipi_enable = 0;
		csi_write(csi, CSI_IPIn_MODE(ipi_base), mode.value);
		csi_write(csi, CSI_IPI_SOFTRSTN, val & ~(CSI_IPIn_RSTN(ipi_id)));
	}
}

void csi_ipi_overflow_reset(struct csi_device *csi, u32 ipi_id)
{
	u32 ipi_base, val;

	if (!csi || (ipi_id >= csi->num_insts - 1))
		return;

	ipi_base = get_ipi_base_from_id(ipi_id);
	val = csi_read(csi, CSI_IPIn_MEM_FLUSH(ipi_base));
	val |= CSI_IPI_MEM_MANUAL_FLUSH;
	csi_write(csi, CSI_IPIn_MEM_FLUSH(ipi_base), val);

	csi_write(csi, CSI_IPI_SOFTRSTN, ~CSI_IPIn_RSTN(ipi_id));
	csi_write(csi, CSI_IPI_SOFTRSTN, CSI_IPI_ALL_RELEASE_RSTN);
	dev_dbg(csi->dev, "%s\n", __func__);
}

void csi_ipi_reg_dump(const struct csi_device *csi, u32 ipi_id, u32 *reg_vals)
{
	u32 ipi_base;

	ipi_base = get_ipi_base_from_id(ipi_id);
	reg_vals[CSI_IPI_INFO_MODE] = csi_read(csi, CSI_IPIn_MODE(ipi_base));
	reg_vals[CSI_IPI_INFO_VC] = csi_read(csi, CSI_IPIn_VCID(ipi_base));
	reg_vals[CSI_IPI_INFO_DT] = csi_read(csi, CSI_IPIn_DATA_TYPE(ipi_base));
	reg_vals[CSI_IPI_INFO_HSA] = csi_read(csi, CSI_IPIn_HSA_TIME(ipi_base));
	reg_vals[CSI_IPI_INFO_HBP] = csi_read(csi, CSI_IPIn_HBP_TIME(ipi_base));
	reg_vals[CSI_IPI_INFO_HSD] = csi_read(csi, CSI_IPIn_HSD_TIME(ipi_base));
	reg_vals[CSI_IPI_INFO_ADV] = csi_read(csi, CSI_IPIn_ADV_FEATURES(ipi_base));
	if (ipi_id < 4)
		reg_vals[CSI_IPI_INFO_FATAL] = csi_read(csi, CSI_INT_ST_IPI1_4_FATAL(ipi_id + 1));
	else
		reg_vals[CSI_IPI_INFO_FATAL] = csi_read(csi, CSI_INT_ST_IPI5_8_FATAL(ipi_id + 1));
}

void csi_ipi_reg_set(struct csi_device *csi, u32 ipi_id, u32 *reg_vals, u32 set_mask)
{
	u32 ipi_base;
	union ipi_mode mode;

	ipi_base = get_ipi_base_from_id(ipi_id);

	mode.value = 0;
	mode.ipi_enable = 1;
	if (((set_mask & (0x1U << CSI_IPI_INFO_MODE)) != 0u) && ((reg_vals[CSI_IPI_INFO_MODE] & mode.value) == 0))
		csi_write(csi, CSI_IPIn_MODE(ipi_base), reg_vals[CSI_IPI_INFO_MODE]);

	if ((set_mask & (0x1U << CSI_IPI_INFO_VC)) != 0U)
		csi_write(csi, CSI_IPIn_VCID(ipi_base), reg_vals[CSI_IPI_INFO_VC]);
	if ((set_mask & (0x1U << CSI_IPI_INFO_DT)) != 0U)
		csi_write(csi, CSI_IPIn_DATA_TYPE(ipi_base), reg_vals[CSI_IPI_INFO_DT]);
	if ((set_mask & (0x1U << CSI_IPI_INFO_HSA)) != 0U)
		csi_write(csi, CSI_IPIn_HSA_TIME(ipi_base), reg_vals[CSI_IPI_INFO_HSA]);
	if ((set_mask & (0x1U << CSI_IPI_INFO_HBP)) != 0U)
		csi_write(csi, CSI_IPIn_HBP_TIME(ipi_base), reg_vals[CSI_IPI_INFO_HBP]);
	if ((set_mask & (0x1U << CSI_IPI_INFO_HSD)) != 0U)
		csi_write(csi, CSI_IPIn_HSD_TIME(ipi_base), reg_vals[CSI_IPI_INFO_HSD]);
	if ((set_mask & (0x1U << CSI_IPI_INFO_ADV)) != 0U)
		csi_write(csi, CSI_IPIn_ADV_FEATURES(ipi_base), reg_vals[CSI_IPI_INFO_ADV]);

	if (((set_mask & (0x1U << CSI_IPI_INFO_MODE)) != 0U) && ((reg_vals[CSI_IPI_INFO_MODE] & mode.value) != 0))
		csi_write(csi, CSI_IPIn_MODE(ipi_base), reg_vals[CSI_IPI_INFO_MODE]);
}

int csi_idi_set_vc_cfg(struct csi_device *csi, u32 idi_id,
		       struct csi_vc_cfg *vc_cfg)
{
	u32 val;
	u32 vc_reg_offset, vc_bit_offset;

	if (idi_id < 4) {
		vc_reg_offset = CSI_DATA_IDS_VC_1;
		vc_bit_offset = idi_id;
	} else if (idi_id < 8) {
		vc_reg_offset = CSI_DATA_IDS_VC_2;
		vc_bit_offset = idi_id - 4;
	} else {
		dev_err(csi->dev, "idi%d not valid", idi_id);
		return -EINVAL;
	}
	val = csi_read(csi, vc_reg_offset);
	val &= ~CSI_IDI_VC_MASK(vc_bit_offset);
	val |= CSI_IDI_VC(vc_bit_offset, vc_cfg->vc_id);
	csi_write(csi, vc_reg_offset, val);
	dev_dbg(csi->dev, "idi%d vc %d\n", idi_id, vc_cfg->vc_id);
	return 0;
}

int csi_idi_set_data_type(struct csi_device *csi, u32 idi_id,
			  struct cam_format *fmt)
{
	u32 val;
	u32 dt_reg_offset, dt_bit_offset;
	u32 data_type;

	if (idi_id < 4) {
		dt_reg_offset = CSI_DATA_IDS_1;
		dt_bit_offset = idi_id;
	} else if (idi_id < 8) {
		dt_reg_offset = CSI_DATA_IDS_2;
		dt_bit_offset = idi_id - 4;
	} else {
		dev_err(csi->dev, "idi%d not valid", idi_id);
		return -EINVAL;
	}

	if (csi_fmt_2_type_info(fmt, NULL, &data_type)) {
		dev_err(csi->dev, "not supported format 0x%x\n", fmt->format);
		return -EINVAL;
	}
	val = csi_read(csi, dt_reg_offset);
	val &= ~CSI_IDI_DATATYPE_MASK(dt_bit_offset);
	val |= CSI_IDI_DATATYPE(dt_bit_offset, data_type);
	csi_write(csi, dt_reg_offset, val);
	dev_dbg(csi->dev, "idi%d data_type %d\n", idi_id, data_type);
	return 0;
}

static int csi_dphy_wait_pwr_up(struct csi_device *csi, u32 lanes)
{
	u32 timeout = 3000;
	u32 stopstate, expect_sta;

	expect_sta = (1 << lanes) - 1;
	while (--timeout) {
		stopstate = csi_read(csi, CSI_PHY_STOPSTATE);
		if ((stopstate & expect_sta) == expect_sta)
			return 0;
		msleep(1);
	}
	return -ETIMEDOUT;
}

int csi_wait_stop_state(struct csi_device *csi, u32 nowait, u32 wait_ms, u32 lanes, u32 *errstate)
{
	u32 stopstate;
	int ret = -1;

	csi_phy_force_rxmode(csi->csiw_dev, csi->id, lanes, true);
	ret = csi_dphy_wait_pwr_up(csi, lanes);
	stopstate = csi_read(csi, CSI_PHY_STOPSTATE);
	csi_phy_force_rxmode(csi->csiw_dev, csi->id, lanes, false);
	if (errstate)
		*errstate = stopstate;
	return ret;
}

u32 csi_get_phy_rx_status(struct csi_device *csi)
{
	return csi_read(csi, CSI_PHY_RX);
}

#ifdef PLATFORM_ASIC
static void csi_dphy_test_codes_init(struct csi_device *csi)
{
	union phy_test_ctl c0;

	csi_write(csi, CSI_PHY_TEST_CTRL1, 0);

	c0.value = 0;
	c0.c0.phy_testclr = 1;
	csi_write(csi, CSI_PHY_TEST_CTRL0, c0.value);
	c0.value = 0;
	csi_write(csi, CSI_PHY_TEST_CTRL0, c0.value);
}

static void csi_dphy_test_write(struct csi_device *csi, u16 code, u8 data)
{
	union phy_test_ctl c0, c1;

	csi_phy_set_testcode(csi->csiw_dev, csi->id, code);

	c1.value = 0;
	c1.c1.phy_testdin = data;
	csi_write(csi, CSI_PHY_TEST_CTRL1, c1.value);

	c0.value = 0;
	csi_write(csi, CSI_PHY_TEST_CTRL0, c0.value);
	c0.c0.phy_testclk = 1;
	csi_write(csi, CSI_PHY_TEST_CTRL0, c0.value);
	msleep(1);
	c0.c0.phy_testclk = 0;
	csi_write(csi, CSI_PHY_TEST_CTRL0, c0.value);
}

static void dphy_merge_write(struct csi_device *csi, u32 reg_addr, u32 major, u32 minor)
{
	csi_phy_testcode_sel_other_phy(csi->csiw_dev, csi->id, false);
	csi_dphy_test_write(csi, reg_addr, major);
	csi_phy_testcode_sel_other_phy(csi->csiw_dev, csi->id, true);
	csi_dphy_test_write(csi, reg_addr, minor);
}

int csi_dphy_merge(struct csi_device *csi)
{
	int rc = 0;
	int major = csi->id / 2 * 2;

	rc = csi_phy_set_lane_mode(csi->csiw_dev, major, true);
	if (rc < 0)
		return rc;

	csi_write(csi, CSI_PHY_RSTZ, 0);
	csi_write(csi, CSI_PHY_SHUTDOWNZ, 0);
	csiex_write(csi, CSI_PHY_RSTZ, 0);
	csiex_write(csi, CSI_PHY_SHUTDOWNZ, 0);

	csi_phy_set_hsfreqrange(csi->csiw_dev, major, csi->lane_rate);
	csi_phy_set_hsfreqrange(csi->csiw_dev, major + 1, csi->lane_rate);
	csi_phy_enableclk(csi->csiw_dev, major);
	csi_phy_enableclk(csi->csiw_dev, major + 1);

	dphy_merge_write(csi, CSI_DPHY_RX_DUAL_PHY_0, 1, 0);
	dphy_merge_write(csi, CSI_DPHY_RX_CLKLANE_LANE_6, 4, 0);
	dphy_merge_write(csi, CSI_DPHY_RX_LANE0_LANE_7, 0x20, 0x20);
	dphy_merge_write(csi, CSI_DPHY_RX_LANE1_LANE_7, 0x20, 0x20);

	dphy_merge_write(csi, CSI_DPHY_RX_LANE2_LANE7, 0x20, 0x20);
	dphy_merge_write(csi, CSI_DPHY_RX_LANE3_LANE7, 0x20, 0x20);
	dphy_merge_write(csi, CSI_DPHY_RX_CLKLANE_LANE_7, 0x00, 0x08);

	csi_dphy_test_write(csi, CSI_DPHY_RX_STARTUP_OVR_0, 0x03);
	csi_dphy_test_write(csi, CSI_DPHY_RX_STARTUP_OVR_1, 0x02);
	csi_dphy_test_write(csi, CSI_DPHY_RX_CLKLANE_LANE_6, 0x08);
	csi_dphy_test_write(csi, CSI_DPHY_RX_CLKLANE_LANE_3, 0x80);
	csi_dphy_test_write(csi, CSI_DPHY_RX_CLKLANE_LANE_4, 0xa);

	csi_write(csi, CSI_RESETN, 1);
	csi_write(csi, CSI_PHY_RSTZ, 1);
	csi_write(csi, CSI_PHY_SHUTDOWNZ, 1);

	csiex_write(csi, CSI_PHY_SHUTDOWNZ, 1);
	rc = csi_dphy_wait_pwr_up(csi, 2);
	if (rc)
		return rc;

	csiex_write(csi, CSI_PHY_RSTZ, 1);
	return csi_dphy_wait_pwr_up(csi, 4);
}

int csi_dphy_init(struct csi_device *csi)
{
	u32 merged = 0;
	int rc = 0;

	rc = csi_phy_get_lane_mode(csi->csiw_dev, csi->id, &merged);
	if (rc < 0)
		return rc;

	if (merged && (csi->id % 2))
		return -EBUSY;

	csi_dphy_test_codes_init(csi);

	if (!csi->tpg_enable && csi->lanes > 2)
		return csi_dphy_merge(csi);

	csi_phy_set_lane_mode(csi->csiw_dev, csi->id, false);
	csi_write(csi, CSI_PHY_RSTZ, 0);
	csi_write(csi, CSI_PHY_SHUTDOWNZ, 0);
	csi_phy_set_hsfreqrange(csi->csiw_dev, csi->id, csi->lane_rate);
	csi_phy_enableclk(csi->csiw_dev, csi->id);
	csi_write(csi, CSI_RESETN, 1);
	csi_write(csi, CSI_PHY_RSTZ, 1);
	csi_write(csi, CSI_PHY_SHUTDOWNZ, 1);
	return csi_dphy_wait_pwr_up(csi, csi->lanes);
}
#endif

#ifdef CONFIG_PM_SLEEP
int csi_system_suspend(struct device *dev)
{
	return pm_runtime_force_suspend(dev);
}

int csi_system_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}
#endif

#ifdef CONFIG_PM
int csi_runtime_suspend(struct device *dev)
{
	struct csi_device *csi = dev_get_drvdata(dev);

	if (csi->ipi)
		clk_disable_unprepare(csi->ipi);
	if (csi->pclk)
		clk_disable_unprepare(csi->pclk);
	if (csi->cfg)
		clk_disable_unprepare(csi->cfg);
	return 0;
}

int csi_runtime_resume(struct device *dev)
{
	struct csi_device *csi = dev_get_drvdata(dev);
	int rc;

	if (csi->ipi) {
		rc = clk_prepare_enable(csi->ipi);
		if (rc)
			return rc;
	}
	if (csi->pclk) {
		rc = clk_prepare_enable(csi->pclk);
		if (rc)
			goto _pclk_err;
	}
	if (csi->cfg) {
		rc = clk_prepare_enable(csi->cfg);
		if (rc)
			goto _cfg_err;
	}
	return 0;
_cfg_err:
	if (csi->pclk)
		clk_disable_unprepare(csi->pclk);
_pclk_err:
	if (csi->ipi)
		clk_disable_unprepare(csi->ipi);
	return rc;
}
#endif
