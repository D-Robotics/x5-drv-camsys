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

#define v4l2_subdev_ctx_call(sd, f, args...) \
({ \
	int _ret = -EINVAL; \
	struct v4l2_buf_ctx *_ctx; \
	if (sd) { \
    	_ctx = v4l2_get_subdevdata(sd); \
		if (_ctx && _ctx->f) { \
			_ret = _ctx->f(_ctx, ##args); \
		} else { \
			pr_err("%s: v4l2_subdev_ctx_call, \
				ctx or ctx->%s does not exist\n", __func__, #f); \
		} \
	} else { \
		pr_err("%s: v4l2_subdev_ctx_call, \
			v4l2_subdev does not exist\n", __func__); \
	} \
    _ret; \
})

#define v4l2_subdev_ctx_call_no_return(sd, f, args...) \
do { \
	struct v4l2_buf_ctx *_ctx; \
	if (sd) { \
    	_ctx = v4l2_get_subdevdata(sd); \
		if (_ctx && _ctx->f) \
			_ctx->f(_ctx, ##args); \
	} \
} while (0)

#define V4L2_MIN(x, y) ((x) > (y) ? (y) : (x))

struct cam_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head entry;
};

struct cam_ctx {
	struct media_pad *pad;
	bool online;
	enum cam_frame_status status;
	void *priv;
};

struct v4l2_buf_ctx {
	u32 magic;
	bool is_sink_online_mode, is_src_online_mode;
	u32 (*get_format)(struct v4l2_buf_ctx *ctx);
	int (*set_format)(struct v4l2_buf_ctx *ctx, u32 pad,
				  struct v4l2_format *format, bool is_try);
	int (*enum_format)(struct v4l2_buf_ctx *ctx, u32 index, u32 *format);
	int (*enum_framesize)(struct v4l2_buf_ctx *ctx, u32 pad,
			      struct v4l2_frmsizeenum *fsize);
	int (*enum_frameinterval)(struct v4l2_buf_ctx *ctx, u32 pad,
				  struct v4l2_frmivalenum *fival);
	int (*set_stream)(struct v4l2_buf_ctx *ctx, u32 pad, int enable);
	void (*ready)(struct v4l2_buf_ctx *ctx, u32 pad, int on);
	int (*qbuf)(struct v4l2_buf_ctx *ctx, struct cam_buf *buf);
	int (*drop)(struct v4l2_buf_ctx *ctx, struct cam_buf *buf);
	struct cam_buf *(*dqbuf)(struct v4l2_buf_ctx *ctx);
	struct cam_buf *(*acqbuf)(struct v4l2_buf_ctx *ctx);
	void (*trigger)(struct v4l2_buf_ctx *ctx);
	bool (*is_completed)(struct v4l2_buf_ctx *ctx);
	void (*set_cap)(struct v4l2_buf_ctx *ctx);
	int (*init_output_ctx)(struct v4l2_buf_ctx *ctx);
	bool (*is_standalone)(struct v4l2_buf_ctx *ctx);
	int (*map_info)(struct v4l2_buf_ctx *ctx, u32 *devid, u32 *insid);
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
	CAM_REQ_BUF,
	CAM_QUERY_BUF,
	CAM_Q_BUF,
	CAM_DQ_BUF,
	CAM_MMAP,
};

struct cam_v4l2_ext_control {
	__u32 pad;
	struct v4l2_ext_control *controls;
};

enum isi_bayer_pattern
{
	ISI_BPAT_RGGB      = 0x00,
	ISI_BPAT_GRBG      = 0x01,
	ISI_BPAT_GBRG      = 0x02,
	ISI_BPAT_BGGR      = 0x03,
	ISI_BPAT_BGGIR     = 0x10,
	ISI_BPAT_GRIRG     = 0x11,
	ISI_BPAT_RGGIR     = 0x12,
	ISI_BPAT_GBIRG     = 0x13,
	ISI_BPAT_GIRRG     = 0x14,
	ISI_BPAT_IRGGB     = 0x15,
	ISI_BPAT_GIRBG     = 0x16,
	ISI_BPAT_IRGGR     = 0x17,
	ISI_BPAT_RGIRB     = 0x18,
	ISI_BPAT_GRBIR     = 0x19,
	ISI_BPAT_IRBRG     = 0x20,
	ISI_BPAT_BIRGR     = 0x21,
	ISI_BPAT_BGIRR     = 0x22,
	ISI_BPAT_GBRIR     = 0x23,
	ISI_BPAT_IRRBG     = 0x24,
	ISI_BPAT_RIRGB     = 0x25,
	ISI_BPAT_RCCC      = 0x30,
	ISI_BPAT_RCCB      = 0x40,
	ISI_BPAT_RYYCY     = 0x50,
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
u32 mbus_code_to_bayer_pattern(u32 code, bool isISI);
int pixelformat_to_mbus_code(u32 format);
u32 mbus_code_to_pixelformat(u32 code);
int subdev_call_command(struct v4l2_subdev *sd, uint32_t cmd, void *arg);
int get_front_info(struct v4l2_subdev *sd, u32 *devid, u32 *insid);
struct v4l2_subdev *get_remote_src_subdev(struct v4l2_subdev *sd, struct media_pad **rpad);
bool is_standalone_datapath(struct v4l2_subdev *sd);
#endif /* _UTILS_H_ */
