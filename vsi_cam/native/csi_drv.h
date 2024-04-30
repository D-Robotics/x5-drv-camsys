/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CSI_DRV_H_
#define _CSI_DRV_H_

#include "csi.h"
#include "hobot_mipi_host.h"
#include "hobot_mipi_host_ops.h"

typedef void (*csi_nat_subirq_func_t)(struct mipi_hdev_s *hdev, u32 subirq,
				      u32 ireg_id, u32 reset_flag);
typedef void (*csi_nat_irq_done_func_t)(struct mipi_hdev_s *hdev,
					u32 irq_main_st, u32 keep);

int csi_nat_init_common(struct mipi_hdev_s *hdev, const mipi_host_cfg_t *cfg);
int csi_nat_get_ipi_num(struct mipi_hdev_s *hdev);
void csi_nat_ipi_reset(struct mipi_hdev_s *hdev, u32 ipi_id, bool enable);
void csi_nat_irq_enable(struct mipi_hdev_s *hdev,
			csi_nat_irq_done_func_t nat_irq_done,
			csi_nat_subirq_func_t nat_subirq);
int csi_nat_irq_disable(struct mipi_hdev_s *hdev, bool use_mask, u32 mask);
int csi_nat_wait_stop_state(struct mipi_hdev_s *hdev,
			    const mipi_host_cfg_t *cfg, u32 *stopstate);
u32 csi_nat_get_phy_rx_status(struct mipi_hdev_s *hdev);
void csi_nat_dphy_reset(struct mipi_hdev_s *hdev);
int csi_nat_ipi_get_info(struct mipi_hdev_s *hdev, mipi_host_ipi_info_t *ipi_info);
int csi_nat_ipi_set_info(struct mipi_hdev_s *hdev, mipi_host_ipi_info_t *ipi_info);
void csi_nat_ipi_overflow_reset(struct mipi_hdev_s *hdev, u32 ipi_id);
int csi_nat_bypass_select(struct mipi_hdev_s *hdev, u32 csi_id);
int csi_nat_tpg_enable(struct mipi_hdev_s *hdev, bool enable);
int csi_nat_probe(struct platform_device *pdev, struct mipi_hdev_s **phdev);
int csi_nat_remove(struct platform_device *pdev, struct mipi_hdev_s *hdev);
#endif /* _CSI_DRV_H_ */
