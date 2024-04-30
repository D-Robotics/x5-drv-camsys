/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/**
 * @file hobot_vin_vcon_sub.h
 *
 * @NO{S10E01C02}
 * @ASIL{B}
 */

#ifndef __HOBOT_VIN_VCON_SUB_H__
#define __HOBOT_VIN_VCON_SUB_H__ /* PRQA S 0603 */ /* header file macro */

#include <linux/types.h>

#include "vio_framemgr.h"
#include "vio_node_api.h"

/**
 * @def VCON_COP_MODULE
 * the module id of vcon sub drivers ops to used
 */
#define VCON_COP_MODULE		VIN_MODULE
/**
 * @def VCON_COPT_DESERIAL
 * the type id of vcon sub deserial drivers ops to used
 */
#define VCON_COPT_DESERIAL	COPS_2
/**
 * @def VCON_COPT_SENSOR
 * the type id of vcon sub sensor drivers ops to used
 */
#define VCON_COPT_SENSOR	COPS_3
/**
 * @def VCON_FLOW_INVALID
 * the invalid value of flow_id
 */
#define VCON_FLOW_INVALID	(-1)

/**
 * @def VCON_SUB_COPTS
 * the type ids of vcon sub drivers for init
 */
#define VCON_SUB_COPTS { \
	VCON_COPT_DESERIAL, \
	VCON_COPT_SENSOR, \
}

/**
 * @enum vcon_sub_type_e
 * vcon sub device type enum
 * @NO{S10E01C02}
 */
enum vcon_sub_type_e {
	VCON_DESERIAL,
	VCON_SENSOR,
	VCON_SUBNUM,
};

/**
 * @enum vcon_sub_event_e
 * vcon sub device event enum
 * @NO{S10E01C02}
 */
enum vcon_sub_event_e {
	VCON_EVENT_HW_SHOW,
	VCON_EVENT_ATTR_SHOW,
	VCON_EVENT_INFO_SHOW,
	VCON_EVENT_NUM,
};

/* vcon event callback for sub drivers to call */
typedef int32_t (*vcon_cb)(int32_t flow_id, int32_t event_id, void *event_data);

/**
 * @struct vcon_sub_ops_s
 * vcon sub device opration fucntions
 * @NO{S10E01C02}
 */
struct vcon_sub_ops_s {
	int32_t (*attach)(int32_t index, int32_t flow_id, int32_t add_id);
	int32_t (*detach)(int32_t index, int32_t flow_id, int32_t add_id);
	int32_t (*start)(int32_t index);
	int32_t (*stop)(int32_t index);
	int32_t (*event)(int32_t index, int32_t event_id, void *event_data);
	void (*setcb)(vcon_cb cb);
	void *reserved_func[4];
	int32_t end_magic;
};

#define VCON_SUB_OPS_END_MAGIC(t)		((0x26767330) | ((t) & 0xf))
#define VCON_SUB_OPS_CHECK(p, t)		(((p) != NULL) && ((p)->end_magic == VCON_SUB_OPS_END_MAGIC(t)))
#define VCON_SUB_OPS_CHECK_STRICT(p, t)		(VCON_SUB_OPS_CHECK(p, t) && ((p)->reserved_func[0] == NULL))

#endif // __HOBOT_VIN_VCON_SUB_H__
