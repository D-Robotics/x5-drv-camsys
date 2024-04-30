/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2023 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef HOBOT_CODEC_COMMON_H
#define HOBOT_CODEC_COMMON_H

#include <linux/cdev.h>
#include <linux/interrupt.h>
#include "vio_config.h"
#include "vio_framemgr.h"
#include "vio_node_api.h"
#include "vio_mem.h"

#define CODEC_NODE_MAX_DEVICE 1
#define CODEC_MAX_CHANNEL	VIO_MAX_STREAM
#define CODEC_MAX_CHN_DEVICE  1
#define USER_BIND 1
#define CODEC_MAX_SUBDEV (CODEC_MAX_CHANNEL*2)
#define BUF_BITMAP_1 1
#define BUF_CONTIG 1
#define BUF_NO_CONTIG 0
#define BUF_ALLOC 1
#define SRC_DQ_TIMEOUT 100
#define CAP_DQ_TIMEOUT 1

/**
 * @struct codec_common_ops
 * @brief Registration functions for other modules
 * @NO{S10E01C01}
 */
struct codec_common_ops {
    s32 (*open)(struct vio_video_ctx *vctx);
    s32 (*close)(struct vio_video_ctx *vctx);
    s32 (*video_start)(struct vio_video_ctx *vctx);
    s32 (*video_stop)(struct vio_video_ctx *vctx);
    s32 (*video_set_attr)(struct vio_video_ctx *vctx, void *attr);
    s32 (*video_get_attr)(struct vio_video_ctx *vctx, void *attr);
    s32 (*video_set_attr_ex)(struct vio_video_ctx *vctx, void *attr);
    s32 (*video_get_attr_ex)(struct vio_video_ctx *vctx, void *attr);
    s32 (*video_set_internal_attr)(struct vio_video_ctx *vctx, void *attr);
    s32 (*video_set_ichn_attr)(struct vio_video_ctx *vctx, void *attr);
    s32 (*video_set_ochn_attr)(struct vio_video_ctx *vctx, void *attr);
    s32 (*video_get_ichn_attr)(struct vio_video_ctx *vctx, void *attr);
    s32 (*video_get_ochn_attr)(struct vio_video_ctx *vctx, void *attr);
    s32 (*video_error_callback)(struct vio_video_ctx *vctx, void *error_info);
    s32 (*video_reset)(struct vio_video_ctx *vctx);
};

#endif /*HOBOT_CODEC_COMMON_H*/
