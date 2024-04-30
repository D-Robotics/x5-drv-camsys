/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2023 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef HOBOT_VIN_NODE_OPS_API
#define HOBOT_VIN_NODE_OPS_API

/**
 * @def HW_FORMAT_RAW
 * RAW_TYPE
 * @NO{S10E01C01}
 */
#define HW_FORMAT_RAW   0x2C

/**
 * @def HW_FORMAT_NV12
 * YUV_TYPE
 * @NO{S10E01C01}
 */
#define HW_FORMAT_NV12	2

s32 vin_node_get_attr(struct vio_video_ctx *vctx, vin_node_attr_t *vin_node_attr);
s32 vin_node_set_attr(struct vio_video_ctx *vctx, vin_node_attr_t *vin_node_attr);
s32 vin_node_set_attr_ex(struct vio_video_ctx *vctx, vin_attr_ex_t *vin_attr_ex);
s32 vin_node_set_internal_attr(struct vio_video_ctx *vctx, void *mipi_attr);
s32 vin_node_set_ochn_attr(struct vio_video_ctx *vctx, vin_ochn_attr_t *ochn_attr);
s32 vin_node_set_ichn_attr(struct vio_video_ctx *vctx, vin_ichn_attr_t *ichn_attr);
s32 vin_node_set_ochn_buff_attr(struct vio_video_ctx *vctx,
		vin_ochn_buff_attr_t *ochn_buff_attr);
s32 vin_node_start(struct vio_video_ctx *vctx);
s32 vin_node_stop(struct vio_video_ctx *vctx);
s32 vin_node_open(struct vio_video_ctx *vctx);
s32 vin_node_close(struct vio_video_ctx *vctx);
s32 vin_node_reqbufs(struct vio_video_ctx *vctx,
		struct vbuf_group_info *group_attr);
s32 vin_node_set_ctrl(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg);
s32 vin_node_bind_check(struct vio_subdev *vdev, struct vio_subdev *remote_vdev, u8 online);
void vin_node_set_ochn_bind_param(struct vio_video_ctx *vctx);


#endif /*HOBOT_VIN_NODE_OPS_API*/
