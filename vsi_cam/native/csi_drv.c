// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "cam_ctrl.h"
#include "csi_drv.h"

#define CSI_DT_NAME     "snps,designware-csi"
#define CSI_DEV_NAME    "vs-snps-csi"
#define PLATFORM_FPGA
#define CSI_DATAIDS_1_START (0)
#define CSI_DATAIDS_2_START (4)
#define CSI_DATAIDS_OFT     (8)
#define CSI_DT_MASK         (0x3f)
#define CSI_VC_MASK         (0x03)
#define CSI_DT_EBD_FLAG     BIT(8)

struct csi_nat_device {
	struct mipi_hdev_s hdev;
	struct csi_device csi_dev;
	csi_nat_irq_done_func_t irq_done_func;
	csi_nat_subirq_func_t subirq_func;
};

u32 rxdphy_vrefcd_lprx = 1;
u32 rxdphy_v400_prog = 4;
u32 rxdphy_deskew_cfg = 0;
module_param(rxdphy_vrefcd_lprx, uint, 0644);
module_param(rxdphy_v400_prog, uint, 0644);
module_param(rxdphy_deskew_cfg, uint, 0644);

static int mipi_datatype_2_csi_fmt(u32 data_type, struct cam_format *fmt)
{
	if (!fmt)
		return -1;
	switch (data_type) {
	case DATA_TYPE_RAW8:
		fmt->format = CAM_FMT_RAW8;
		break;
	case DATA_TYPE_RAW10:
		fmt->format = CAM_FMT_RAW10;
		break;
	case DATA_TYPE_RAW12:
		fmt->format = CAM_FMT_RAW12;
		break;
	case DATA_TYPE_YUV422_8:
		fmt->format = CAM_FMT_YUYV;
		break;
	case DATA_TYPE_YUV420_SHIFT:
		fmt->format = CAM_FMT_NV12;
		break;
	case DATA_TYPE_RGB888:
		fmt->format = CAM_FMT_RGB888X;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int csi_data_ids_set(struct csi_device *csi, u32 data_ids, u32 data_ids_vc, u32 id_start)
{
	struct csi_vc_cfg vc_cfg;
	struct cam_format fmt;
	u32 i;
	u8 dt;
	int rc = 0;

	for (i = id_start; data_ids != 0; i++, data_ids >>= CSI_DATAIDS_OFT, data_ids_vc >>= CSI_DATAIDS_OFT) {
		dt = data_ids & CSI_DT_MASK;
		if (!dt)
			continue;
		rc = mipi_datatype_2_csi_fmt(dt, &fmt);
		if (rc)
			continue;
		rc = csi_idi_set_data_type(csi, i, &fmt);
		if (rc)
			continue;
		vc_cfg.vc_id = data_ids_vc & CSI_VC_MASK;
		rc = csi_idi_set_vc_cfg(csi, i, &vc_cfg);
	}

	return rc;
}

static int csi_configure_ipi(struct mipi_hdev_s *hdev, const mipi_host_cfg_t *cfg)
{
	struct csi_device *csi = &((struct csi_nat_device *)hdev)->csi_dev;
	struct mipi_host_param_s *param = &hdev->host.param;
	struct csi_ipi_base_cfg ipi_cfg;
	struct csi_vc_cfg vc_cfg;
	struct cam_format fmt;
	u32 dt, dt_real, ipi_i = 0;
	u32 *dt_sys = &param->ipi1_dt;

	if (cfg->channel_num > csi->num_insts - 1)
		return -1;
	while (ipi_i < cfg->channel_num) {
		dt = dt_sys[ipi_i] != 0 ? dt_sys[ipi_i] : cfg->datatype;
		dt_real = dt &  CSI_DT_MASK;
		dev_dbg(csi->dev, "in %s ipi%d, datatype 0x%x\n", __func__, ipi_i, dt);
		if (mipi_datatype_2_csi_fmt(dt_real, &fmt)) {
			ipi_i++;
			continue;
		}
		fmt.width = cfg->width;
		fmt.height = cfg->height;

		vc_cfg.vc_id = cfg->channel_sel[ipi_i];
		if (vc_cfg.vc_id <= CSI_VC_MASK) {
			vc_cfg.vc_mode = CSI_VC_CAMERA;
			vc_cfg.vc_ebd_en = dt & CSI_DT_EBD_FLAG ? 1 : 0;
			csi_ipi_set_vc_cfg(csi, ipi_i, &vc_cfg);

			ipi_cfg.id = ipi_i;
			ipi_cfg.cut_through = param->cut_through;
			ipi_cfg.mem_auto_flush = param->mem_flush;
			ipi_cfg.adv_val = param->adv_value;
			ipi_cfg.hsa = cfg->hsaTime;
			ipi_cfg.hsd = cfg->hsdTime;
			ipi_cfg.hbp = cfg->hbpTime;
			if (dt & CSI_DT_EBD_FLAG)
				ipi_cfg.adv_val |= CSI_IPI_ADV_FEAT_EN_EBD;
			ipi_cfg.adv_val |= CSI_IPI_ADV_FEAT_EVSELPROG | CSI_IPI_ADV_FEAT_EN_VIDEO |
					CSI_IPI_ADV_FEAT_MODE_LEGACY;

			csi_ipi_init(csi, &ipi_cfg, &fmt);
			csi_ipi_start(csi, ipi_i);
		} else {
			csi_ipi_stop(csi, ipi_i);
		}
		ipi_i++;
	}
	return 0;
}

int csi_nat_init_common(struct mipi_hdev_s *hdev, const mipi_host_cfg_t *cfg)
{
	struct csi_device *csi = &((struct csi_nat_device *)hdev)->csi_dev;
	struct mipi_host_param_s *param = &hdev->host.param;
	u8 vc, dt, horizontal;

	csi->lanes = cfg->lane;
	csi->lane_rate = cfg->mipiclk;

	// ppipg clk should be set before RST release
	if (cfg->ppi_pg) {
		vc = MIPI_HOST_PPIPGC_VC(cfg->ppi_pg);
		horizontal = MIPI_HOST_PPIPGC_MODE(cfg->ppi_pg);
		if (MIPI_HOST_PPIPGC_TYPE(cfg->ppi_pg))
			dt = MIPI_CSI2_DT_EMB_8;
		else
			dt = DATA_TYPE_RGB888;
	} else {
		vc = 0;
		horizontal = 0;
		dt = 0;
	}
	csi_set_tpg_mode(csi, cfg->ppi_pg, 0, horizontal, vc, dt);

#ifndef PLATFORM_FPGA
	csi_dphy_reset(csi);
	csi_dphy_init(csi);
#else
	csi_set_lane_rate(csi, cfg->mipiclk);
#endif
	csi_set_lanes(csi, cfg->lane, param->vcext_en);
	if (param->data_ids_1)
		csi_data_ids_set(csi, param->data_ids_1, param->data_ids_vc1, CSI_DATAIDS_1_START);
	if (param->data_ids_2)
		csi_data_ids_set(csi, param->data_ids_2, param->data_ids_vc2, CSI_DATAIDS_2_START);
	return csi_configure_ipi(hdev, cfg);
}
EXPORT_SYMBOL(csi_nat_init_common);

int csi_nat_get_ipi_num(struct mipi_hdev_s *hdev)
{
	struct csi_nat_device *nat_dev = (struct csi_nat_device *)hdev;

	return nat_dev->csi_dev.num_insts - 1;
}
EXPORT_SYMBOL(csi_nat_get_ipi_num);

void csi_nat_ipi_reset(struct mipi_hdev_s *hdev, u32 ipi_id, bool enable)
{
	struct csi_nat_device *nat_dev = (struct csi_nat_device *)hdev;

	csi_ipi_reset_and_down(&nat_dev->csi_dev, ipi_id, enable);
}
EXPORT_SYMBOL(csi_nat_ipi_reset);

static void csi_nat_irq_done(struct csi_device *csi, u32 irq_main_st)
{
	struct csi_nat_device *nat_dev = container_of(csi, struct csi_nat_device, csi_dev);
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)nat_dev;
	struct mipi_host_icnt_s *icnt = &hdev->host.icnt;
	struct mipi_host_param_s *param = &hdev->host.param;
	int keep = 0;
	u32 mask = BIT(CSI_ERR_IPI1) | BIT(CSI_ERR_IPI2) | BIT(CSI_ERR_IPI3) | BIT(CSI_ERR_IPI4);

	icnt->st_main++;

	if (icnt->st_main <= param->irq_cnt)
		return;

	if (param->ipi_overst != 0U) {
		keep = csi_irq_disable_mask(csi, ~mask);
	} else {
		csi_irq_disable(csi);
		keep = 0;
	}
	if (nat_dev->irq_done_func)
		nat_dev->irq_done_func(&nat_dev->hdev, irq_main_st, keep);
}

static void csi_nat_subirq(struct csi_device *csi, u32 subirq,
			   const struct csi_irq_reg *err_reg, u32 reset_flag)
{
	struct csi_nat_device *nat_dev = container_of(csi, struct csi_nat_device, csi_dev);

	if (nat_dev->subirq_func)
		nat_dev->subirq_func(&nat_dev->hdev, subirq, err_reg->id, reset_flag);
}

void csi_nat_irq_enable(struct mipi_hdev_s *hdev,
			csi_nat_irq_done_func_t nat_irq_done,
			csi_nat_subirq_func_t nat_subirq)
{
	struct csi_nat_device *nat_dev = (struct csi_nat_device *)hdev;

	nat_dev->irq_done_func = nat_irq_done;
	nat_dev->subirq_func   = nat_subirq;

	nat_dev->csi_dev.irq_done_func = csi_nat_irq_done;
	nat_dev->csi_dev.subirq_func   = csi_nat_subirq;

	csi_irq_enable(&nat_dev->csi_dev);
}
EXPORT_SYMBOL(csi_nat_irq_enable);

int csi_nat_irq_disable(struct mipi_hdev_s *hdev, bool use_mask, u32 mask)
{
	struct csi_nat_device *nat_dev = (struct csi_nat_device *)hdev;

	if (use_mask)
		return csi_irq_disable_mask(&nat_dev->csi_dev, mask);
	else
		csi_irq_disable(&nat_dev->csi_dev);
	return 0;
}
EXPORT_SYMBOL(csi_nat_irq_disable);

int csi_nat_wait_stop_state(struct mipi_hdev_s *hdev,
			    const mipi_host_cfg_t *cfg, u32 *stopstate)
{
	struct csi_device *csi = &((struct csi_nat_device *)hdev)->csi_dev;
	struct mipi_host_param_s *param = &hdev->host.param;

	return csi_wait_stop_state(csi, param->notimeout, param->wait_ms,
				   cfg->lane, stopstate);
}
EXPORT_SYMBOL(csi_nat_wait_stop_state);

u32 csi_nat_get_phy_rx_status(struct mipi_hdev_s *hdev)
{
	struct csi_device *csi = &((struct csi_nat_device *)hdev)->csi_dev;

	return csi_get_phy_rx_status(csi);
}
EXPORT_SYMBOL(csi_nat_get_phy_rx_status);

void csi_nat_dphy_reset(struct mipi_hdev_s *hdev)
{
	struct csi_device *csi = &((struct csi_nat_device *)hdev)->csi_dev;

	csi_dphy_reset(csi);
}
EXPORT_SYMBOL(csi_nat_dphy_reset);

int csi_nat_ipi_get_info(struct mipi_hdev_s *hdev, mipi_host_ipi_info_t *ipi_info)
{
	struct csi_device *csi = &((struct csi_nat_device *)hdev)->csi_dev;
	struct mipi_host_s *host = &hdev->host;
	mipi_host_cfg_t *cfg = &host->cfg;
	u32 index;
	u32 regs[CSI_IPI_INFO_NUM];

	index = ipi_info->index;
	if (index >= (csi->num_insts - 1)) {
		dev_err(csi->dev, "%d:ipi%d get: not support error\n", index, index + 1);
		return -EPERM;
	}
	if (index >= (int32_t)cfg->channel_num)
		dev_dbg(csi->dev, "%d:ipi%d get: not inited warning\n", index, index + 1);
	csi_ipi_reg_dump(csi, index, regs);
	ipi_info->fatal = (u16)regs[CSI_IPI_INFO_FATAL];
	ipi_info->mode = (u16)regs[CSI_IPI_INFO_MODE];
	ipi_info->vc = (u8)regs[CSI_IPI_INFO_VC];
	ipi_info->datatype = (u16)regs[CSI_IPI_INFO_DT];
	ipi_info->hsa = (u16)regs[CSI_IPI_INFO_HSA];
	ipi_info->hbp = (u16)regs[CSI_IPI_INFO_HBP];
	ipi_info->hsd = (u16)regs[CSI_IPI_INFO_HSD];
	ipi_info->adv = (u16)regs[CSI_IPI_INFO_ADV];
	return 0;
}
EXPORT_SYMBOL(csi_nat_ipi_get_info);

int csi_nat_ipi_set_info(struct mipi_hdev_s *hdev, mipi_host_ipi_info_t *ipi_info)
{
	struct csi_device *csi = &((struct csi_nat_device *)hdev)->csi_dev;
	struct mipi_host_s *host = &hdev->host;
	mipi_host_cfg_t *cfg = &host->cfg;
	u32 index;
	u32 regs[CSI_IPI_INFO_NUM];
	u32 set_mask;

	index = ipi_info->index;
	if (index >= (csi->num_insts - 1)) {
		dev_err(csi->dev, "%d:ipi%d set: not support error\n", index, index + 1);
		return -EPERM;
	}
	if (index >= (int32_t)cfg->channel_num)
		dev_dbg(csi->dev, "%d:ipi%d set: not inited warning\n", index, index + 1);
	if ((ipi_info->fatal & HOST_IPI_INFO_B_SETV) != 0U)
		set_mask = ipi_info->fatal;
	else
		set_mask = HOST_IPI_INFO_B_ALL;
	regs[CSI_IPI_INFO_MODE] = ipi_info->mode;
	regs[CSI_IPI_INFO_VC] = ipi_info->vc;
	regs[CSI_IPI_INFO_DT] = ipi_info->datatype;
	regs[CSI_IPI_INFO_HSA] = ipi_info->hsa;
	regs[CSI_IPI_INFO_HBP] = ipi_info->hbp;
	regs[CSI_IPI_INFO_HSD] = ipi_info->hsd;
	regs[CSI_IPI_INFO_ADV] = ipi_info->adv;
	csi_ipi_reg_set(csi, index, regs, set_mask);
	return 0;
}
EXPORT_SYMBOL(csi_nat_ipi_set_info);

void csi_nat_ipi_overflow_reset(struct mipi_hdev_s *hdev, u32 ipi_id)
{
	struct csi_device *csi = &((struct csi_nat_device *)hdev)->csi_dev;

	csi_ipi_overflow_reset(csi, ipi_id);
}
EXPORT_SYMBOL(csi_nat_ipi_overflow_reset);

int csi_nat_bypass_select(struct mipi_hdev_s *hdev, u32 csi_id)
{
	struct csi_device *csi = &((struct csi_nat_device *)hdev)->csi_dev;
	int ret = 0;

	ret = set_idi_bypass_select(csi->cam_ctl_dev, csi_id, true);
	return ret;
}
EXPORT_SYMBOL(csi_nat_bypass_select);

int csi_nat_tpg_enable(struct mipi_hdev_s *hdev, bool enable)
{
	struct csi_device *csi = &((struct csi_nat_device *)hdev)->csi_dev;

	return csi_set_tpg(csi, enable);
}
EXPORT_SYMBOL(csi_nat_tpg_enable);

int csi_nat_probe(struct platform_device *pdev, struct mipi_hdev_s **phdev)
{
	struct csi_nat_device *nat_dev;
	int rc;

	nat_dev = devm_kzalloc(&pdev->dev, sizeof(*nat_dev), GFP_KERNEL);
	if (!nat_dev)
		return -ENOMEM;
	rc = csi_probe(pdev, &nat_dev->csi_dev, true);
	if (rc)
		return rc;
	nat_dev->hdev.port = nat_dev->csi_dev.id;
	nat_dev->hdev.osdev.devno = nat_dev->csi_dev.id;
	nat_dev->hdev.host.iomem = nat_dev->csi_dev.base;
	*phdev = (struct mipi_hdev_s *)nat_dev;
	return 0;
}
EXPORT_SYMBOL(csi_nat_probe);

int csi_nat_remove(struct platform_device *pdev, struct mipi_hdev_s *hdev)
{
	struct csi_nat_device *nat_dev = (struct csi_nat_device *)hdev;

	return csi_remove(pdev, &nat_dev->csi_dev);
}
EXPORT_SYMBOL(csi_nat_remove);

static int __init csi_init_module(void)
{
	return 0;
}

static void __exit csi_exit_module(void)
{
}

module_init(csi_init_module);
module_exit(csi_exit_module);

MODULE_DESCRIPTION("SNPS MIPI CSI Controller Driver");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
MODULE_LICENSE("GPL");
MODULE_ALIAS("SNPS-CSI");
