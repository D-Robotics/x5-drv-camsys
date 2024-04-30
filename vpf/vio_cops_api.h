/**
 * @file: vio_cops_api.h
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

#ifndef VIO_COPS_API_H
#define VIO_COPS_API_H

#include "../sensor/inc/common_camera.h"
#include "vio_framemgr.h"

struct vio_callback_ops {
	u32 magic;
	char name[32];
	void *cops;
	void *empty_ops;
};

enum cops_type {
	COPS_0,
	COPS_1,
	COPS_2,
	COPS_3,
	COPS_4,
	COPS_5,
	COPS_6,
	COPS_7,
	COPS_8,
	COPS_9,
	COPS_NUM,
};

#define DECLARE_VIO_CALLBACK_OPS(_name, _magic, _cops)  \
    static struct vio_callback_ops cb_##_name = {            \
		.name = #_name,									\
		.magic = _magic,								\
		.cops = _cops,									\
    };                                                  \

s32 vio_register_callback_ops(struct vio_callback_ops *vpf_cops, u32 module, enum cops_type ctype);
void vio_unregister_callback_ops(u32 module, enum cops_type ctype);
void *vio_get_callback_ops(void *empty_ops, u32 module, enum cops_type ctype);

struct vio_node;

struct camsys_interface_ops {
	s32 (*ip_reset_func)(u32 module, u32 rst_flag);
};

struct cim_interface_ops {
	void (*get_frame_id)(struct vio_node *vnode, struct frame_id_desc *frameid);
	void (*cim_isp_write_reg)(uint32_t flow_id, uint32_t offset, uint32_t *value);
	s32 (*cim_isp_read_reg)(uint32_t flow_id, uint32_t offset, uint32_t *value);
	s32 (*cim_isp_update_calib_param)(uint32_t flow_id, uint32_t module_type, void *param);
	u64 (*cim_get_lpwm_timestamps)(uint32_t lpwm_chn);
};

struct isp_interface_ops {
	void (*ipi_lost_fe_state)(struct vio_node *vnode);
	void (*cim_trans_raw_frameid)(struct vio_node *vnode);
	s32 (*isp_trans_frameid)(uint32_t flow_id, struct frame_id_desc *frameid);
	int32_t (*get_isp_hw_fault_status)(struct vio_node *vnode);
};

struct dbg_interface_ops {
	int32_t (*vtrace_send)(uint32_t module_type, uint32_t param_type, uint32_t *param,
		uint32_t flow_id, uint32_t frame_id, uint32_t ctx_id, uint32_t chnid);
};
#endif
