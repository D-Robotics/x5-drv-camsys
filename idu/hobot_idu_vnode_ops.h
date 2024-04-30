/*
 * hobot-drivers/camsys/idu/hobot_idu_vnode_dev.h
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
#ifndef HOBOT_IDU_VNODE_OPS_H
#define HOBOT_IDU_VNODE_OPS_H
#include <linux/types.h>

#include <vio_node_api.h>
#include "idu_cfg.h"
#include <hb_idu_hw.h>
#include <hb_mipi_csi_device_ops.h>
#ifdef CONFIG_HOBOT_DRM_MIPI_DSI
#include <hb_mipi_dsi_host_ops.h>
#endif

struct hobot_idu_vsync {
	atomic_t refcount;
	wait_queue_head_t queue;
	ktime_t time;
	atomic64_t count;
	int32_t enabled;
};

void idu_vsync_init(struct hobot_idu_vsync *vsync);
int32_t idu_wait_vsync(struct hobot_idu_vsync *vsync, uint64_t rel_count);
void idu_handle_vsync(struct hobot_idu_vsync *vsync);
int32_t idu_vsync_enable(struct hobot_idu_vsync *vsync);
int32_t idu_vsync_disable(struct hobot_idu_vsync *vsync);
int32_t idu_vpf_streamon(struct vio_video_ctx *vctx);
int32_t idu_vpf_streamoff(struct vio_video_ctx *vctx);
int32_t idu_open(struct vio_video_ctx *vctx, uint32_t rst_en);
int32_t idu_close(struct vio_video_ctx *vctx, uint32_t rst_en);
void idu_frame_work(struct vio_node *vnode);
void idu_channel_frame_handle(struct vio_subdev *vdev, struct vio_frame *frame);
int32_t idu_set_init_attr(struct vio_video_ctx *vctx, unsigned long arg);
int32_t idu_set_ichn_attr_ex(struct vio_video_ctx *vctx, unsigned long arg);
int32_t idu_set_ochn_attr(struct vio_video_ctx *vctx, unsigned long arg);
int32_t idu_set_ochn_attr_ex(struct vio_video_ctx *vctx, unsigned long arg);
int32_t idu_set_ichn_attr(struct vio_video_ctx *vctx, unsigned long arg);
int32_t idu_yuvformat_to_hbmem(uint32_t format);
int32_t idu_rgbformat_to_hbmem(uint32_t format);
int32_t idu_capture_to_hbmem(uint32_t format);
int32_t idu_allow_bind(struct vio_subdev *vdev, struct vio_subdev *remote_vdev, u8 online_mode);
#endif //HOBOT_IDU_VNODE_OPS_H