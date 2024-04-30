/**
 * @file: hobot_vpf_ops.h
 * @
 * @NO{S09E05C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */
/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef HOBOT_VPF_OPS_H
#define HOBOT_VPF_OPS_H

#include "vio_node_api.h"

#define INPUT_HW_ID_MASK 0xF0000000
#define INPUT_HW_ID_SHIFT 28
#define OUTPUT_MODULE_MASK 0x0F000000
#define OUTPUT_MODULE_SHIFT 24
#define OUTPUT_HW_ID_MASK 0x00F00000
#define OUTPUT_HW_ID_SHIFT 20
#define OUTPUT_CH_MASK 0x000F0000
#define OUTPUT_CH_SHIFT 16
#define OUTPUT_CTXID_MASK 0x0000F000
#define OUTPUT_CTXID_SHIFT 12
#define INPUT_MODULE_MASK 0x00000F00
#define INPUT_MODULE_SHIFT 8
#define INPUT_CH_MASK 0x000000F0
#define INPUT_CH_SHIFT 4
#define INPUT_CTXID_MASK 0x0000000F
#define INPUT_CTXID_SHIFT 0

s32 vpf_device_close(struct vio_video_ctx *vctx);
s32 vpf_device_open(struct vio_video_ctx *vctx, struct vpf_device *vpf_device);
s32 vpf_subdev_close(struct vio_video_ctx *vctx);
s32 vpf_subdev_open(struct vio_video_ctx *vctx);
s32 vpf_subdev_get_version(struct vio_version_info *version);
s32 vpf_manager_ioctl(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg);
s32 vpf_device_probe(struct vpf_device *vpf_device);
#endif//HOBOT_VPF_OPS_H