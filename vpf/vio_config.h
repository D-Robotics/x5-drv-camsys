/**
 * @file: vio_config.h
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

#ifndef HOBOT_VIO_CONFIG_H
#define HOBOT_VIO_CONFIG_H

#include <linux/types.h>

#ifdef HOBOT_MCU_CAMSYS
#define vio_err(fmt, ...)       printf(fmt, ##__VA_ARGS__)
#define vio_warn(fmt, ...)      printf(fmt, ##__VA_ARGS__)
#define vio_dbg(fmt, ...)       printf(fmt, ##__VA_ARGS__)
#define vio_info(fmt, ...)      printf(fmt, ##__VA_ARGS__)
#else
#define vio_err(fmt, ...)	(void)pr_err("[L%d]"fmt, __LINE__, ##__VA_ARGS__)
#define vio_warn(fmt, ...)	(void)pr_warn("[L%d]"fmt, __LINE__, ##__VA_ARGS__)
#define vio_dbg(fmt, ...)	pr_debug("[L%d]"fmt, __LINE__, ##__VA_ARGS__)
#define vio_info(fmt, ...)	(void)pr_info("[L%d]"fmt, __LINE__, ##__VA_ARGS__)
#endif

#define CONFIG_QEMU_TEST 0

#define VIO_MAX_STREAM	24u
#define PROCESS_TIMEOUT		(3000) /* ms */

#define HW_FORMAT_YUV420_8BIT	0x18u
#define HW_FORMAT_YUV420_10BIT	0x19u
#define HW_FORMAT_YUV420_LEG_8BIT	0x1Au
#define HW_FORMAT_YUV420_SHIFT_8BIT	0x1Cu
#define HW_FORMAT_YUV420_SHIFT_10BIT 0x1Du
#define HW_FORMAT_YUV422_8BIT	0x1Eu
#define HW_FORMAT_YUV422_10BIT	0x1Fu
#define HW_FORMAT_RAW8		0x2Au
#define HW_FORMAT_RAW10 	0x2Bu
#define HW_FORMAT_RAW12 	0x2Cu
#define HW_FORMAT_RAW14 	0x2Du
#define HW_FORMAT_RAW16 	0x2Eu
#define HW_FORMAT_RAW20 	0x2Fu
#define HW_FORMAT_RAW24		0x27u

#define INVALID_FLOW_ID 	0xffff

#define MIPI_RX0 0u
#define MIPI_RX1 1u
#define MIPI_RX2 2u
#define MIPI_RX3 3u
#define MIPI_VC_MASK 0xf

#define DPU_HW_NUM 2
#define DPU_LAYER_NUM 2u


#define MAX_CAM_APB_FREQ	307200000
#define MAX_CAM_SYS_FREQ	648000000
#define MAX_CIM_DMA_FREQ	768000000

#define FUSA_RBACK_TIMES 3u
#define FUSA_SW_ERR_CODE 0xffffu
#define FUSA_ENV_LEN 4u

#define MASK_8BIT 0xffu
#define BIT_8 8u

#define ERR_SEQ_0 0
#define ERR_SEQ_1 1
#define ERR_SEQ_2 2
#define ERR_SEQ_3 3
#define ERR_SEQ_4 4
#define ERR_SEQ_5 5
#define ERR_SEQ_6 6
#define ERR_SEQ_7 7
#define ERR_SEQ_8 8
#define ERR_SEQ_9 9
#define ERR_SEQ_10 10

enum vio_event_id {
	EventIdParamCheckErr = 33,
	EventIdPymModeErr,
	EventIdBindGroupErr,
	EventIdSwFramedropErr,
	EventIdHwFramedropErr,
	EventIdSwTimeoutErr,
	EventIdHwTimeoutErr,
	EventIdOpenDeviceErr,
	EventIdReqbufferErr,
	EventIdMmapErr,
	EventIdFrameErr,
	EventIdUserSetParamErr,
	EventIdUserGetParamErr,
	EventIdCtxStateErr,
	EventIdSuspendErr,
	EventIdResumeErr,
	EventIdProbeErr,
};

enum vio_video_state {
	VIO_VIDEO_CLOSE,
	VIO_VIDEO_OPEN,
	VIO_VIDEO_S_INPUT,
	VIO_VIDEO_INIT_ATTR,
	VIO_VIDEO_CHN_ATTR,
	VIO_VIDEO_REBUFS,
	VIO_VIDEO_STOP,
	VIO_VIDEO_START,
};

enum vio_subdev_state {
	VIO_SUBDEV_INIT_ATTR,
	VIO_SUBDEV_CHN_ATTR,
	VIO_SUBDEV_REQBUF,
	VIO_SUBDEV_STREAM_ON,
	VIO_SUBDEV_STREAM_OFF,
	VIO_SUBDEV_BIND_DONE,
	VIO_SUBDEV_DMA_OUTPUT,
};

enum FrameEventType {
	VIO_FRAME_NONE = 0,
	VIO_FRAME_DONE = 1,
	VIO_FRAME_NDONE = 2,
	VIO_FRAME_PREINT = 3,
	VIO_FRAME_DROP = 4,
	VIO_FRAME_HUP = 5,
};

#define SOFT_PRST 0x1u
#define SOFT_VRST 0x2u
#define SOFT_ARST 0x4u
#define SOFT_RST_ALL 0xfu
enum J5_RST_id {
	CIM_DMA_RST,
	ERM_RST,
	CMM_RST,
	IAR1_RST,
	IAR0_RST,
	CIM_RST,
	DVP_RST,
	MIPI_TX1_RST,
	MIPI_TX0_RST,
	MIPI_RX3_RST,
	MIPI_RX2_RST,
	MIPI_RX1_RST,
	MIPI_RX0_RST,
	STL_RST,
	GDC_RST,
	PYM2_RST,
	PYM1_RST,
	ISP1_RST,
	IPE1_RST,
	PYM0_RST,
	ISP0_RST,
	IPE0_RST,
};

struct roi_rect {
    u32 roi_x;
    u32 roi_y;
    u32 roi_width;
    u32 roi_height;
};

struct vpf_ext_ctrl {
	u32 id;
	void *arg;
};

#define MAGIC_NUMBER	0x12345678u
#define VIO_IOC_MAGIC 'p'

#define VIO_IOC_BIND_CTX         _IOWR(VIO_IOC_MAGIC, 0, int)
#define VIO_IOC_SET_ATTR    	 _IOW(VIO_IOC_MAGIC, 1, int)
#define VIO_IOC_GET_ATTR         _IOR(VIO_IOC_MAGIC, 2, int)
#define VIO_IOC_SET_ATTR_EX      _IOW(VIO_IOC_MAGIC, 3, int)
#define VIO_IOC_GET_ATTR_EX      _IOW(VIO_IOC_MAGIC, 4, int)

#define VIO_IOC_SET_ICH_ATTR     _IOW(VIO_IOC_MAGIC, 5, int)
#define VIO_IOC_SET_ICH_ATTR_EX  _IOW(VIO_IOC_MAGIC, 6, int)
#define VIO_IOC_GET_ICH_ATTR     _IOR(VIO_IOC_MAGIC, 7, int)
#define VIO_IOC_SET_OCH_ATTR     _IOW(VIO_IOC_MAGIC, 8, int)
#define VIO_IOC_SET_OCH_ATTR_EX  _IOW(VIO_IOC_MAGIC, 9, int)
#define VIO_IOC_GET_OCH_ATTR	 _IOR(VIO_IOC_MAGIC, 10, int)
#define VIO_IOC_QUERYBUF       	 _IOWR(VIO_IOC_MAGIC, 11, int)
#define VIO_IOC_REQBUFS     	 _IOW(VIO_IOC_MAGIC, 12, int)
#define VIO_IOC_START			 _IOW(VIO_IOC_MAGIC, 13, int)
#define VIO_IOC_STOP      		 _IOW(VIO_IOC_MAGIC, 14, int)
#define VIO_IOC_DQBUF     		 _IOR(VIO_IOC_MAGIC, 15, int)
#define VIO_IOC_QBUF      		 _IOW(VIO_IOC_MAGIC, 16, int)
#define VIO_IOC_SET_CTRL     	 _IOW(VIO_IOC_MAGIC, 17, int)
#define VIO_IOC_GET_CTRL      	 _IOWR(VIO_IOC_MAGIC, 18, int)
#define VIO_IOC_FLUSH_FRAME    	 _IO(VIO_IOC_MAGIC, 19)
#define VIO_IOC_SET_INTER_ATTR   _IOW(VIO_IOC_MAGIC, 20, int)
#define VIO_IOC_GET_VERSION      _IOR(VIO_IOC_MAGIC, 21, int)
#define VIO_IOC_GET_STRUCT_SIZE	_IOR(VIO_IOC_MAGIC, 22, int)

#define VIO_IOC_BIND_FLOW        _IOWR(VIO_IOC_MAGIC, 30, int)
#define VIO_IOC_BIND_NODE        _IOWR(VIO_IOC_MAGIC, 31, int)
#define VIO_IOC_ADD_NODE         _IOW(VIO_IOC_MAGIC, 32, int)
#define VIO_IOC_SET_CALLBACK     _IOR(VIO_IOC_MAGIC, 33, int)
#define VIO_IOC_GET_HW_STATUS      _IOR(VIO_IOC_MAGIC, 34, int)

#define VIO_IOC_DBG_CTRL     	 _IOWR(VIO_IOC_MAGIC, 50, int)
#endif /* HOBOT_VIO_CONFIG_H */
