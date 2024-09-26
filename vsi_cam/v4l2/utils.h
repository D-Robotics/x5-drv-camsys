/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _UTILS_H_
#define _UTILS_H_
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>

#include "cam_ctx.h"

#define CSI_DT_NAME     "snps,designware-csi"
#define SIF_DT_NAME     "verisilicon,sif"
#define ISP_DT_NAME     "verisilicon,isp"
#define VSE_DT_NAME     "verisilicon,vse"
#define GDC_DT_NAME     "arm,gdc"

#define CSI_DEV_NAME    "vs-snps-csi"
#define SIF_DEV_NAME    "vs-sif"
#define ISP_DEV_NAME    "vs-isp"
#define VSE_DEV_NAME    "vs-vse"
#define GDC_DEV_NAME    "vs-arm-gdc"

#define STRIDE_ALIGN    (64)

#define vb2_buf_to_cam_buf(vb2) \
({ \
	struct vb2_v4l2_buffer *vbuf = \
			container_of(vb2, struct vb2_v4l2_buffer, vb2_buf); \
	container_of(vbuf, struct cam_buf, vb); \
})

struct cam_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head entry;
};

struct cam_ctx {
	struct media_pad *pad;
	enum cam_frame_status status;
	void *priv;
};

struct v4l2_buf_ctx {
	bool is_sink_online_mode, is_src_online_mode;
	u32 (*get_format)(struct v4l2_buf_ctx *ctx);
	int (*set_format)(struct v4l2_buf_ctx *ctx, u32 format, bool is_try);
	int (*enum_format)(struct v4l2_buf_ctx *ctx, u32 index, u32 *format);
	int (*enum_framesize)(struct v4l2_buf_ctx *ctx, u32 pad,
			      struct v4l2_frmsizeenum *fsize);
	int (*enum_frameinterval)(struct v4l2_buf_ctx *ctx, u32 pad,
				  struct v4l2_frmivalenum *fival);
	void (*ready)(struct v4l2_buf_ctx *ctx, u32 pad, int on);
	int (*qbuf)(struct v4l2_buf_ctx *ctx, struct cam_buf *buf);
	int (*drop)(struct v4l2_buf_ctx *ctx, struct cam_buf *buf);
	struct cam_buf *(*dqbuf)(struct v4l2_buf_ctx *ctx);
	void (*trigger)(struct v4l2_buf_ctx *ctx);
	bool (*is_completed)(struct v4l2_buf_ctx *ctx);
	void (*set_cap)(struct v4l2_buf_ctx *ctx);
};

struct subdev_node {
	struct v4l2_subdev sd;
	struct v4l2_buf_ctx bctx;
	struct device *dev;
	u32 num_pads;
	struct media_pad *pads;
	int (*async_bound)(struct subdev_node *sn);
};

enum v4l_core_ctrl_cmd {
	CAM_SET_CTRL = 0,
	CAM_GET_CTRL,
	CAM_SET_EXT_CTRL,
	CAM_GET_EXT_CTRL,
	CAM_QUERY_CTRL,
	CAM_QUERY_EXT_CTRL,
};

struct cam_v4l2_ext_control {
	__u32 pad;
	struct v4l2_ext_control *controls;
};

int subdev_init(struct subdev_node *n, const char *name, u32 hwid, int inst,
		const struct v4l2_subdev_ops *ops,
		const struct media_entity_operations *mops);
void subdev_deinit(struct subdev_node *n);
int subdev_set_fmt(struct v4l2_subdev *sd,
		   struct v4l2_subdev_state *state,
		   struct v4l2_subdev_format *fmt);
int subdev_set_stream(struct v4l2_subdev *sd, int enable);
int subdev_open(struct v4l2_subdev *sd);
int subdev_close(struct v4l2_subdev *sd);
u32 pixelformat_to_cam_format(u32 format);
u32 cam_format_to_pixelformat(u32 format, u32 bayer_format);
u32 mbus_code_to_cam_format(u32 format);
u32 cam_format_to_mbus_code(u32 format, u32 bayer_format);
int subdev_call_command(struct v4l2_subdev *sd, uint32_t cmd, void *arg);
int subdev_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_size_enum *fse);
#endif /* _UTILS_H_ */
