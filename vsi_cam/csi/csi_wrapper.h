/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _CSI_WRAPPER_H_
#define _CSI_WRAPPER_H_

struct csiw_device;
int csi_phy_enableclk(struct csiw_device *dev, unsigned int id);
int csi_phy_force_rxmode(struct csiw_device *dev, unsigned int id, unsigned int lanes, bool force);
int csi_phy_get_lane_mode(struct csiw_device *dev, unsigned int id, unsigned int *lane_merged);
int csi_phy_set_lane_mode(struct csiw_device *dev, unsigned int id, bool lane_merged);
int csi_phy_set_hsfreqrange(struct csiw_device *dev, unsigned int id, unsigned int rate);
int csi_phy_get_hsfreqrange(struct csiw_device *dev, unsigned int id, u32 *hsfreqrange);
int csi_phy_enable_ppipg_clk(struct csiw_device *dev, bool enable);
void csi_phy_set_testcode(struct csiw_device *dev, unsigned int id, unsigned int code);
void csi_phy_testcode_sel_other_phy(struct csiw_device *dev, unsigned int id, bool sel_other);
void put_csi_wrapper_device(struct csiw_device *dev);

#endif /* _CSI_WRAPPER_H_ */
