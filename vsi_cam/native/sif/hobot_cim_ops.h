/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2023 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef HOBOT_CIM_OPS_H
#define HOBOT_CIM_OPS_H

#include "hobot_vpf_manager.h"
#include "vin_node_config.h"

#define CIM_MAX_SHADOW_NUM 8u
#define CIM_MAX_LINE_NUM 8u
#define CIM_TIMEOUT_CNT 20u
#define CIM_MSLEEP_TIME 2u

#define UV_PLANE_SHIFT 2u
#define UV_PLANE_CNT 2
#define MAX_HW_ID  6

#define VC_INDEX_MAX 7u
#define VC_CHANNELS_MAX 4u
#define WIDTH_MAX 4096u
#define WIDTH_MIN 128u
#define HEIGHT_MAX 4096u
#define HEIGHT_MIN 16u

#define ISP_BUF_CNT 3
#define FIFO_WATER_INT 0x7fu
#define FIFO_WATER_INT_NUM 8u

#define CIM_FRAME_START	1 << 1u
#define CIM_FRAME_DONE	1 << 2u
#define CIM_FRAME_DROP	1 << 3u
#define CIM_FRAME_END	1 << 4u

#define CIM_INT_CPU_MASK  (7)

#define TIMEOUT_STOP_WAIT 100u

enum cim_err_interrupt_map {
	INTR_CIM_IPI0_ERR,
	INTR_CIM_IPI1_ERR,
	INTR_CIM_IPI2_ERR,
	INTR_CIM_IPI3_ERR,
	INTR_CIM_IPI4_ERR,
	INTR_CIM_IPI5_ERR,
	INTR_CIM_IPI6_ERR,
	INTR_CIM_IPI7_ERR,
};

enum cim_fs_interrupt_map {
	INTR_CIM_IPI0_FS,
	INTR_CIM_IPI1_FS,
	INTR_CIM_IPI2_FS,
	INTR_CIM_IPI3_FS,
	INTR_CIM_IPI4_FS,
	INTR_CIM_IPI5_FS,
	INTR_CIM_IPI6_FS,
	INTR_CIM_IPI7_FS,
	INTR_CIM_DVP_FS,
};

enum cim_fe_interrupt_map {
	INTR_CIM_IPI0_FE,
	INTR_CIM_IPI1_FE,
	INTR_CIM_IPI2_FE,
	INTR_CIM_IPI3_FE,
	INTR_CIM_IPI4_FE,
	INTR_CIM_IPI5_FE,
	INTR_CIM_IPI6_FE,
	INTR_CIM_IPI7_FE,
	INTR_CIM_DVP_FE,
};

enum dma_status {
	INTR_IMG_STATUS0,
	INTR_IMG_STATUS1,
	INTR_IMG_STATUS2,
	INTR_IMG_STATUS3,
	INTR_IMG_STATUS4,
	INTR_IMG_STATUS5,
	INTR_IMG_STATUS6,
	INTR_IMG_STATUS7,
};

enum dma_status_interrupt_map {
	INTR_WCH_FRAME_DONE0,
	INTR_WCH_FRAME_DONE1,
	INTR_WCH_FRAME_DONE2,
	INTR_WCH_FRAME_DONE3,
	INTR_WCH_FRAME_DONE4,
	INTR_WCH_FRAME_DONE5,
	INTR_WCH_FRAME_DONE6,
	INTR_WCH_FRAME_DONE7,
	INTR_WCH_FRAME_DONE8,
	INTR_WCH_FRAME_DONE9,
	INTR_WCH_FRAME_DONE10,
	INTR_WCH_FRAME_DONE11,
	INTR_WCH_FRAME_DONE12,
	INTR_WCH_FRAME_DONE13,
	INTR_WCH_FRAME_DONE14,
	INTR_WCH_FRAME_DONE15,
	INTR_WCH_FRAME_DROP_DONE0,
	INTR_WCH_FRAME_DROP_DONE1,
	INTR_WCH_FRAME_DROP_DONE2,
	INTR_WCH_FRAME_DROP_DONE3,
	INTR_WCH_FRAME_DROP_DONE4,
	INTR_WCH_FRAME_DROP_DONE5,
	INTR_WCH_FRAME_DROP_DONE6,
	INTR_WCH_FRAME_DROP_DONE7,
	INTR_WCH_FRAME_DROP_DONE8,
	INTR_WCH_FRAME_DROP_DONE9,
	INTR_WCH_FRAME_DROP_DONE10,
	INTR_WCH_FRAME_DROP_DONE11,
	INTR_WCH_FRAME_DROP_DONE12,
	INTR_WCH_FRAME_DROP_DONE13,
	INTR_WCH_FRAME_DROP_DONE14,
	INTR_WCH_FRAME_DROP_DONE15,
};

enum cim_status {
	CIM_OTF_INPUT,
	CIM_DMA_INPUT,
	CIM_REUSE_SHADOW0,
	CIM_HW_CONFIG,
	CIM_HW_RUN,
	CIM_IPI0_USED,
};

enum cim_src_type_e {
	CIM_SRC_MIPI,
	CIM_SRC_RDMA,
	CIM_SRC_PATTERN
};

int cim_set_ichn_attr(struct vio_video_ctx *vctx, vin_ichn_attr_t *cim_ichn_attr);
int cim_set_ochn_buff_attr(struct vio_video_ctx *vctx,
		vin_ochn_buff_attr_t *cim_ochn_buff_attr);
void cim_set_group_frameid(struct vio_node *vnode);
s32 cim_subdev_init_ex(struct vio_video_ctx *vctx, vin_attr_ex_t *vin_attr_ex);
int cim_set_ochn_attr(struct vio_video_ctx *vctx, vin_ochn_attr_t *cim_ochn_attr);
s32 cim_subdev_init(struct vio_video_ctx *vctx, cim_attr_t *cim_attr);
s32 cim_subdev_reqbufs(struct vio_subdev *vdev, u32 buffers);
s32 cim_subdev_start(struct vio_video_ctx *vctx, u32 tpn_fps);
s32 cim_subdev_stop(struct vio_video_ctx *vctx);
void cim_set_drop_info(u32 rx, u32 vc_mask);
void j6_cim_subdev_init(struct j6_cim_dev *cim);
void cim_set_clk_enable(u32 enable);
void cim_sw_rst(struct j6_cim_dev *cim);
s32 cim_skip_fusa_check(struct j6_cim_dev *cim, u8 ipi_channel);
s32 cim_open(struct vio_video_ctx *vctx);
s32 cim_close(struct vio_video_ctx *vctx, u32 rst_en);
s32 cim_isp_read_reg(uint32_t flow_id, uint32_t offset, uint32_t *value);
void cim_isp_write_reg(uint32_t flow_id, uint32_t offset, uint32_t *value);
s32 cim_isp_update_calib_param(uint32_t flow_id,
		uint32_t module_type, void *param);
void cim_config_next_frame_addr(struct vio_node *vnode, u8 chn);
void cim_frame_work(struct vio_node *vnode);

#endif /*HOBOT_CIM_OPS_H*/
