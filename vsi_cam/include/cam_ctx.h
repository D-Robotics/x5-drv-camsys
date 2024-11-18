/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CAM_COM_H_
#define _CAM_COM_H_

#include <linux/device.h>

#include "cam_buf.h"

enum cam_frame_status {
	NO_ERR    = 0,
	VSIZE_ERR = 1,
	HSIZE_ERR = 1,
	BOTH_ERR  = 1,
	IPI_OF    = 2,
	DQ_FAIL   = 3,
};

int cam_trigger(struct cam_ctx *ctx);

bool cam_is_completed(struct cam_ctx *ctx);

struct init_attr {
	bool en_reqbufs;
	struct device *dev;
};

__weak
int cam_ctx_init(struct cam_ctx *ctx, void *data, struct init_attr *attr);

__weak
void cam_ctx_release(struct cam_ctx *ctx);

void sif_set_frame_des(struct cam_ctx *ctx, void *data);

void sif_get_frame_des(struct cam_ctx *ctx);

void sif_set_frame_event(struct cam_ctx *ctx, u32 type);

void isp_update_frame_info(void *data,struct cam_ctx *ctx);

void cam_set_stat_info(struct cam_ctx *ctx, u32 type);

bool cam_osd_update(struct cam_ctx *ctx);

int cam_osd_set_cfg(struct cam_ctx *ctx, u32 ochn_id);

int cam_read_hist(struct cam_ctx *ctx, u32 ochn_id);

int cam_set_mode(struct cam_ctx *ctx, u32 mode);

void cam_set_frame_status(void *cam_ctx, enum cam_frame_status status);

u8 cam_get_frame_status(void *cam_ctx);

void cam_dec_frame_status(void *cam_ctx);

void isp_handle_set_sensor_ctrl(void *isp_dev, uint32_t inst, void *data);

void isp_handle_get_sensor_ctrl(void *isp_dev, uint32_t inst, void *data);

int cam_get_frame_info(struct cam_ctx *ctx, struct cam_frame_info *info);

int cam_update_frame_info(struct cam_ctx *ctx, struct cam_frame_info *info);

bool vse_get_drop_status(struct cam_ctx *ctx);

int cam_check_datapath(struct cam_ctx *ctx, bool *online);

#endif /* _CAM_COM_H_ */
