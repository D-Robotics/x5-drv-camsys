/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2023 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef HOBOT_VIN_COMMON_CIM_H
#define HOBOT_VIN_COMMON_CIM_H

#include "vio_config.h"
#include "vio_framemgr.h"
#include "vio_node_api.h"
#include "vio_mem.h"
#include "osal.h"

/**
 * @def VIN_NODE_MAX_DEVICE
 * The maximum number of devices supported by the node driver
 * @NO{S10E01C01}
 */
#define VIN_NODE_MAX_DEVICE  5u
/**
 * @def MAX_CHN_DEVICE
 * The maximum number of channels supported by the node driver
 * @NO{S10E01C01}
 */
#define MAX_CHN_DEVICE  4
/**
 * @def LPWM_CHN_NUM
 * The maximum number of lpwm channels supported by the node driver
 * @NO{S10E01C01}
 */
#define LPWM_CHN_NUM  4
/**
 * @def SENSOR_ERR_PIN_NUM
 * The maximum number of sensor err pins supported by the node driver
 * @NO{S10E01C01}
 */
#define SENSOR_ERR_PIN_NUM 4

#define MAX_VIN_HW_ID		6

/**
 * @enum vin_type_e
 * @brief Module types supported by vin
 * @NO{S10E01C01}
 */
typedef enum _vin_type_e {
	VIN_CIM,
	VIN_MIPI,
	VIN_VCON,
	VIN_LPWM,
	VIN_MAX_DEVICES,
} vin_type_e;

static const char * const vin_type_name[VIN_MAX_DEVICES] = {
	"VIN_CIM",
	"VIN_MIPI",
	"VIN_VCON",
	"VIN_LPWM"
};

static const u8 vin_start_order[VIN_MAX_DEVICES] = {
	VIN_CIM,
	VIN_LPWM,
	VIN_VCON,
	VIN_MIPI
};

static const u8 vin_stop_order[VIN_MAX_DEVICES] = {
	VIN_VCON,
	VIN_LPWM,
	VIN_MIPI,
	VIN_CIM
};

/**
 * @enum vin_ochn_type_e
 * @brief Channel types supported by vin
 * @NO{S10E01C01}
 */
typedef enum _vin_ochn_type_e {
	VIN_MAIN_FRAME,
	VIN_ONLINE,
	VIN_FRAME_TOGHER,
	VIN_EMB,
	VIN_ROI,
	VIN_TYPE_INVALID,
} vin_ochn_type_e;

enum vin_ichn_type_e {
	VIN_DDRIN = 5,
};

enum vin_node_state_e {
	CIM_INIT,
	CIM_OPEN,
	CIM_START,
	CIM_PAUSE,

	MIPI_OPEN,
	MIPI_INIT,
	MIPI_START,

	VCON_OPEN,
	VCON_INIT,
	VCON_START,

	LPWM_INIT,
	LPWM_START,

	LPWM_CHN0_OPEN,
	LPWM_CHN1_OPEN,
	LPWM_CHN2_OPEN,
	LPWM_CHN3_OPEN,
	LPWM_CHN0_INIT,
	LPWM_CHN1_INIT,
	LPWM_CHN2_INIT,
	LPWM_CHN3_INIT,
	LPWM_CHN0_START,
	LPWM_CHN1_START,
	LPWM_CHN2_START,
	LPWM_CHN3_START,
};

/**
 * @struct vin_common_ops
 * @brief Registration functions for other modules
 * @NO{S10E01C01}
 */
struct vin_common_ops {
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
	s32 (*video_s_ctrl)(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg);
};

#endif/*__HOBOT_VIN_COMMON_CIM_H__*/
