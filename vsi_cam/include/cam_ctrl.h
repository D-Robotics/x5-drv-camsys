/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _CAM_CTRL_H_
#define _CAM_CTRL_H_

struct cam_ctrl_device;
int set_idi_bypass_select(struct cam_ctrl_device *dev, unsigned int csi_index, bool enable);
int set_isp_input_select(struct cam_ctrl_device *dev, unsigned int id, unsigned int csi_index,
			 unsigned int ipi_index);
int get_gdc_intr_stat_and_clear(struct cam_ctrl_device *dev, unsigned int *stat, bool *is_done);
void put_cam_ctrl_device(struct cam_ctrl_device *dev);

#endif /* _CAM_CTRL_H_ */
