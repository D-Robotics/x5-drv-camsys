/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _DW_CRC_H_
#define _DW_CRC_H_

#include "cam_uapi.h"

enum dw_mod {
	DW_MOD_VSE,
	DW_MOD_GDC
};

struct dw_crc_device;
int dw_reset(struct dw_crc_device *dev, enum dw_mod mod);
int dw_set_state(struct dw_crc_device *dev, enum dw_mod mod,
		 enum cam_state state);
void put_dw_crc_device(struct dw_crc_device *dev);

#endif /* _DW_CRC_H_ */
