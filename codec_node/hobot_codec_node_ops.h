/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2023 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef HOBOT_CODEC_NODE_OPS_API
#define HOBOT_CODEC_NODE_OPS_API

s32 codec_node_get_attr(struct vio_video_ctx *vctx, struct codec_node_attr_s *codec_node_attr);
s32 codec_node_set_attr(struct vio_video_ctx *vctx, struct codec_node_attr_s *codec_node_attr);
s32 codec_node_set_attr_ex(struct vio_video_ctx *vctx, struct  codec_attr_ex_s *codec_attr_ex);
s32 codec_node_set_ochn_attr(struct vio_video_ctx *vctx, struct  codec_ochn_attr_s *ochn_attr);
s32 codec_node_set_ichn_attr(struct vio_video_ctx *vctx, struct  codec_ichn_attr_s *ichn_attr);
s32 codec_node_set_ochn_buff_attr(struct vio_video_ctx *vctx, struct  codec_ochn_buff_attr_s *ochn_buff_attr);
s32 codec_node_start(struct vio_video_ctx *vctx);
s32 codec_node_stop(struct vio_video_ctx *vctx);
s32 codec_node_open(struct vio_video_ctx *vctx);
s32 codec_node_close(struct vio_video_ctx *vctx);
s32 codec_node_reqbufs(struct vio_video_ctx *vctx, struct vbuf_group_info *group_attr);
s32 codec_node_dqbuf(struct vio_video_ctx *vctx, unsigned long arg);
s32 codec_node_qbuf(struct vio_video_ctx *vctx, unsigned long arg);
s32 codec_node_bind_flow_id(struct vio_video_ctx *vctx, unsigned long arg);
s32 codec_node_get_buf_cfg(struct vio_video_ctx *vctx, unsigned long arg);
s32 codec_node_querybuf(struct vio_video_ctx *vctx, unsigned long arg);

#endif /*HOBOT_CODEC_NODE_OPS_API*/