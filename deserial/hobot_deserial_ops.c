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
 * @file hobot_deserial_ops.c
 *
 * @NO{S10E02C11}
 * @ASIL{B}
 */

#include "hobot_deserial_ops.h"

/**
 * @var g_des_pre_state_names
 * deserial pre state name string array
 */
static const char *g_des_pre_state_names[] = DESERIAL_PRE_STATE_NAMES;
/**
 * @var g_des_ope_state_names
 * deserial op state name string array
 */
static const char *g_des_op_state_names[] = DESERIAL_OP_STATE_NAMES;

/**
 * @var g_des_param_names
 * deserial param name string array
 */
static const char *g_des_param_names[] = DESERIAL_PARAM_NAMES;

/**
 * @var g_des
 * global deserial driver struct
 */
static struct deserial_s g_des = {
	.ver = { DESERIAL_VER_MAJOR, DESERIAL_VER_MINOR },
};

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief get deserial global struct
 *
 * @return !NULL: the global deserial_s struct pointer
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
struct deserial_s* deserial_global(void)
{
	return &g_des;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial attr info init do
 *
 * @param[in] des: deserial device struct
 * @param[in] des_info: deserial attr struct pointer
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_data_init_do(struct deserial_device_s *des, struct deserial_info_data_s *des_info)
{
	int32_t i;
	struct deserial_link_s *link;
	struct deserial_info_data_s *dinfo = &des->deserial_info;

	memcpy(dinfo, des_info, sizeof(struct deserial_info_data_s));

	for(i = 0; i < DESERIAL_LINK_NUM_MAX; i++) {
		link = &des->link[i];
		link->desp = dinfo->link_desp[i];
		link->sensor_id = dinfo->sensor_index[i];
		link->init_by_pid = 0;
		if ((dinfo->sensor_index[i] >= 0) || (dinfo->link_desp[i][0] != '\0'))
			link->linked = 1;
		else
			link->linked = 0;
	}

	return 0;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial attr info deinit do
 *
 * @param[in] des: deserial device struct
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_data_deinit_do(struct deserial_device_s *des)
{

	return 0;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial attr info compare
 *
 * @param[in] s_info: deserial info struct source
 * @param[in] d_info: deserial info struct destine
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_data_info_cmp(struct deserial_info_data_s *s_info, struct deserial_info_data_s *d_info)
{

	return 0;
}


/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial stream on/off use data_info reg
 *
 * @param[in] des: deserial device struct
 * @param[in] on: 0-off as stop, 1-on as start
 * @param[in] link: the stream link
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_stream_reg_do(struct deserial_device_s *des, int32_t on, int32_t link)
{

	return 0;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial init operation real done
 *
 * @param[in] des: deserial device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void deserial_init_done(struct deserial_device_s *des)
{
	struct deserial_user_s *user = &des->user;

	if (user->op_state == DES_OP_STATE_CANCEL)
		user->op_state = DES_OP_STATE_DEFAULT;


	return;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial deinit operation real done
 *
 * @param[in] des: deserial device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void deserial_deinit_done(struct deserial_device_s *des)
{

	return;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial start operation real done
 *
 * @param[in] des: deserial device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void deserial_start_done(struct deserial_device_s *des)
{

	return;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial stop operation real done
 *
 * @param[in] des: deserial device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void deserial_stop_done(struct deserial_device_s *des)
{

	return;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial device call vcon event callback function
 *
 * @param[in] flow_id: flow id
 * @param[in] event_id: event id
 * @param[in] event_data: event data pointer
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_vcon_event(int32_t flow_id, int32_t event_id, void *event_data)
{
	if (g_des.vcon_event_cb != NULL)
		return g_des.vcon_event_cb(flow_id, event_id, event_data);

	return 0;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial operation user wait (in nutex lock)
 *
 * @param[in] des: deserial device struct
 * @param[out] op_info: the opreation info to get
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_opu_do(struct deserial_device_s *des, deserial_op_info_t *op_info)
{
	struct deserial_user_s *user = &des->user;
	struct os_dev * dev = &des->osdev;
	int32_t ret;

	if ((user->op_state == DES_OP_STATE_DEFAULT) ||
		(user->op_state == DES_OP_STATE_FINISH)) {
		des_info(dev, "%s: %s to %s", __func__,
			g_des_op_state_names[user->op_state],
			g_des_op_state_names[DES_OP_STATE_WAIT]);
		user->op_state = DES_OP_STATE_WAIT;
	}
	osal_mutex_unlock(&user->mutex);

	ret = osal_wait_event_interruptible(user->opu_wq,
			((user->op_state == DES_OP_STATE_CANCEL) ||
			 (((user->op_state == DES_OP_STATE_DOING) ||
			   (user->op_state == DES_OP_STATE_DOING_NOWAIT)) &&
			  (user->op_info != NULL))));
	osal_mutex_lock(&user->mutex);
	if (ret < 0) {
		return ret;
	}
	if (user->op_state == DES_OP_STATE_CANCEL)
		return -ESRCH;
	if (user->op_info == NULL)
		return -EFAULT;

	memcpy(op_info, user->op_info, sizeof(deserial_op_info_t));

	return 0;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial operation user done (in nutex lock)
 *
 * @param[in] des: deserial device struct
 * @param[in] result: operation result: 0-done, others-error
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_opu_done(struct deserial_device_s *des, int32_t result)
{
	uint32_t ret = 0, op_state;
	struct deserial_user_s *user = &des->user;
	struct os_dev *dev = &des->osdev;

	op_state = user->op_state;
	user->op_state = (result == 0) ? DES_OP_STATE_DONE : DES_OP_STATE_ERROR;

	if (user->op_wait == 0U) {
		des_info(dev, "%s: %s to %s to %s\n", __func__,
			g_des_op_state_names[op_state],
			g_des_op_state_names[user->op_state],
			g_des_op_state_names[DES_OP_STATE_FINISH]);
		user->op_state = DES_OP_STATE_FINISH;
		if (op_state != DES_OP_STATE_DOING_NOWAIT)
			ret = -EFAULT;
	} else {
		osal_wake_up(&user->opk_wq);
	}

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial operation (in nutex lock)
 *
 * @param[in] des: deserial device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_opk_do(struct deserial_device_s *des, deserial_op_info_t *op_info)
{
	struct deserial_user_s *user = &des->user;
	struct deserial_param_s *pa = &des->param;
	struct os_dev * dev = &des->osdev;
	int32_t ret, op_state, toout = 0, retry = 0, finish = 0;
	int32_t op_timeout_ms = pa->op_timeout_ms;
	int32_t nowait = (op_timeout_ms == 0) ? 1 : 0;

	op_state = user->op_state;
	if ((op_state == DES_OP_STATE_WAIT) || (op_state == DES_OP_STATE_FINISH)) {
		/* if wait is empty */
		if (nowait == 0) {
			user->op_info = op_info;
			user->op_state = DES_OP_STATE_DOING;
		} else {
			memcpy(&user->op_nowait, op_info, sizeof(deserial_op_info_t));
			user->op_info = &user->op_nowait;
			user->op_state = DES_OP_STATE_DOING_NOWAIT;
		}
		des_info(dev, "%s: %s to %s\n", __func__,
			g_des_op_state_names[op_state],
			g_des_op_state_names[user->op_state]);
		osal_wake_up(&user->opu_wq);
		if (nowait != 0)
			return 0;
	}
	user->op_wait++;
	osal_mutex_unlock(&user->mutex);

	while (finish == 0) {
		if (op_timeout_ms == -1) {
			/* wait forever */
			ret = osal_wait_event_interruptible(user->opk_wq,
				((user->op_state == DES_OP_STATE_CANCEL) ||
				 (user->op_state == DES_OP_STATE_WAIT) ||
				 ((user->op_state > DES_OP_STATE_DOING) && (user->op_info == op_info))));
		} else {
			/* wait timeout */
			toout = 1;
			ret = osal_wait_event_interruptible_timeout(user->opk_wq,
				((user->op_state == DES_OP_STATE_CANCEL) ||
				 (user->op_state == DES_OP_STATE_WAIT) ||
				 ((user->op_state > DES_OP_STATE_DOING) && (user->op_info == op_info))),
				op_timeout_ms);
		}
		osal_mutex_lock(&user->mutex);
		if ((toout != 0) && (ret == 0)) {
			/* timeout */
			if (retry >= pa->op_retry_max) {
				user->op_wait--;
				ret = -ETIME;
				des_err(dev, "%s: %s %dms timeout failed %d\n", __func__,
					g_des_op_state_names[user->op_state], op_timeout_ms, ret);
				osal_mutex_unlock(&user->mutex);
				break;
			} else {
				retry++;
				des_warn(dev, "%s: %s %dms timeout retry %d\n", __func__,
					g_des_op_state_names[user->op_state], op_timeout_ms, retry);
				osal_mutex_unlock(&user->mutex);
				continue;
			}
		}
		/* wake up */
		if (user->op_state == DES_OP_STATE_CANCEL) {
			/* cancel */
			user->op_wait--;
			ret = -ESRCH;
			finish = 1;
		} else if ((user->op_state > DES_OP_STATE_DOING) && (user->op_info == op_info)) {
			/* this op finish */
			des_info(dev, "%s: %s to %s", __func__,
				g_des_op_state_names[user->op_state],
				g_des_op_state_names[DES_OP_STATE_FINISH]);
			user->op_wait--;
			ret = (user->op_state == DES_OP_STATE_DONE) ? 0 : -EFAULT;
			user->op_state = DES_OP_STATE_FINISH;
			finish = 1;
		} else if (user->op_state == DES_OP_STATE_WAIT) {
			if (nowait == 0) {
				user->op_info = op_info;
				user->op_state = DES_OP_STATE_DOING;
			} else {
				memcpy(&user->op_nowait, op_info, sizeof(deserial_op_info_t));
				user->op_info = &user->op_nowait;
				user->op_state = DES_OP_STATE_DOING_NOWAIT;
			}
			des_info(dev, "%s: %s to %s\n", __func__,
				g_des_op_state_names[DES_OP_STATE_WAIT],
				g_des_op_state_names[user->op_state]);
			/* this op ready to run */
			osal_wake_up(&user->opu_wq);
			if (nowait != 0) {
				user->op_wait--;
				ret = 0;
				finish = 1;
			}
		}
		osal_mutex_unlock(&user->mutex);
	}

	osal_mutex_lock(&user->mutex);
	if (user->op_wait > 0U) {
		/* someone waiting to op */
		osal_wake_up(&user->opk_wq);
	}

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial operation cancel (in mutex lock)
 *
 * @param[in] des: deserial device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void deserial_op_cancel(struct deserial_device_s *des)
{
	struct deserial_user_s *user = &des->user;

	if (user->op_state != DES_OP_STATE_DEFAULT) {
		user->op_state = DES_OP_STATE_CANCEL;
		osal_wake_up(&user->opu_wq);
		if (user->op_wait > 0U)
			osal_wake_up(&user->opk_wq);
	}

	return;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial streaming operation
 *
 * @param[in] des: deserial device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_streaming_do(struct deserial_device_s *des, int32_t state)
{
	int32_t ret;

	struct deserial_user_s *user = &des->user;
	deserial_op_info_t op_info = {
		.type = DES_OP_TYPE_STREAM,
		.data = state,
	};

	if (user->op_state != DES_OP_STATE_DEFAULT) {
		ret = deserial_opk_do(des, &op_info);
	} else {
		ret = deserial_stream_reg_do(des, state, 0);
	}

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial attr info init
 *
 * @param[in] des: deserial device struct
 * @param[in] arg: deserial attr struct
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_data_init(struct deserial_device_s *des, unsigned long arg)
{
	int32_t ret = 0;
	struct deserial_user_s *user = &des->user;
	struct os_dev * dev = &des->osdev;
	struct deserial_info_data_s des_info;

	if (arg == 0UL) {
		des_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}

	if (osal_copy_from_app((void *)&des_info,
				(void __user *)arg, sizeof(struct deserial_info_data_s)) != 0U) {
		des_err(dev, "%s error, config %px from user error",
				__func__, (void __user *)arg);
		return -EFAULT;
	}

	osal_mutex_lock(&user->mutex);
	des_info(dev, "%s cmd: %u %s", __func__, user->data_init,
		(user->data_init != 0U) ? "drop" : "real");
	if (user->data_init == 0U) {
		ret = deserial_data_init_do(des, &des_info);
		if (ret != 0) {
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
	} else if (deserial_data_info_cmp(&des->deserial_info, &des_info) != 0) {
		des_info(dev, "%s warning: init info mismatch", __func__);
	} else {
		/* init done */
	}
	user->data_init = 1U;
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial init operation request
 *
 * @param[in] des: deserial device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_init_req(struct deserial_device_s *des, unsigned long arg)
{
	int32_t ret = 0;
	struct deserial_user_s *user = &des->user;
	struct os_dev * dev = &des->osdev;
	uint32_t timeout_ms = 0U, wait = 0U;

	if (arg != 0U) {
		if (osal_copy_from_app(&timeout_ms, (uint32_t *)arg, sizeof(uint32_t)) != 0) {
			des_err(dev, "%s get data from user failed", __func__);
			return -EFAULT;
		}
	}
	osal_mutex_lock(&user->mutex);
	if (user->init_cnt == 0U) {
		if (user->pre_state == (uint32_t)DES_PRE_STATE_DEFAULT) {
			user->pre_state = (uint32_t)DES_PRE_STATE_INITING;
			des_info(dev, "%s cmd: %s", __func__,
				g_des_pre_state_names[user->pre_state]);
		} else if (user->pre_state == (uint32_t)DES_PRE_STATE_INITING) {
			wait = 1;
		} else {
			des_info(dev, "%s cmd: %s drop", __func__,
				g_des_pre_state_names[user->pre_state]);
			ret = -EACCES;
		}
	} else {
		des_info(dev, "%s cmd: %s drop", __func__,
			g_des_pre_state_names[user->pre_state]);
			ret = -EACCES;
	}
	osal_mutex_unlock(&user->mutex);
	if (wait != 0U) {
		user->pre_done = (bool)false;
		if (timeout_ms != 0U) {
			osal_wait_event_interruptible_timeout(user->pre_wq,
				user->pre_done, timeout_ms);
		} else {
			osal_wait_event_interruptible(user->pre_wq, user->pre_done);
		}
		osal_mutex_lock(&user->mutex);
		if ((user->init_cnt == 0U) &&
			(user->pre_state == (uint32_t)DES_PRE_STATE_DEFAULT)) {
			user->pre_state = (uint32_t)DES_PRE_STATE_INITING;
			des_info(dev, "%s cmd: wait & %s", __func__,
			       g_des_pre_state_names[user->pre_state]);
		} else {
			des_info(dev, "%s cmd: wait & drop", __func__);
			ret = -EACCES;
		}
		osal_mutex_unlock(&user->mutex);
	}

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial init operation result
 *
 * @param[in] des: deserial device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_init_result(struct deserial_device_s *des, unsigned long arg)
{
	int32_t ret = 0;
	struct deserial_user_s *user = &des->user;
	struct os_dev * dev = &des->osdev;
	uint32_t result = 0U, wake = 0U;

	if (arg == 0UL) {
		des_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}

	if (osal_copy_from_app(&result, (uint32_t *)arg, sizeof(uint32_t)) != 0U) {
		des_err(dev, "%s get data from user failed", __func__);
		return -EFAULT;
	}
	osal_mutex_lock(&user->mutex);
	if ((user->init_cnt == 0U) &&
		(user->pre_state == (uint32_t)DES_PRE_STATE_INITING)) {
		if (result != 0U) {
			user->pre_state = (uint32_t)DES_PRE_STATE_DEFAULT;
		} else {
			deserial_init_done(des);
			user->pre_state = (uint32_t)DES_PRE_STATE_INITED;
		}
		wake = 1U;
	}
	if (result == 0U) {
		user->init_cnt ++;
	}
	osal_mutex_unlock(&user->mutex);
	if (wake != 0U) {
		des_info(dev, "%s cmd: %s wake", __func__,
				(result != 0U) ? "falied" : "done");
		user->pre_done = (bool)true;
		osal_wake_up(&user->pre_wq);
	} else {
		des_info(dev, "%s cmd: %s drop", __func__,
				(result != 0U) ? "falied" : "done");
	}

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial deinit operation request and done
 *
 * @param[in] des: deserial device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_deinit_req(struct deserial_device_s *des, unsigned long arg)
{
	int32_t ret = 0;
	struct deserial_user_s *user = &des->user;
	struct os_dev * dev = &des->osdev;
	int32_t through = 0;

	if (arg != 0U) {
		if (osal_copy_from_app(&through, (uint32_t *)arg, sizeof(uint32_t)) != 0) {
			des_err(dev, "%s get data from user failed", __func__);
			return -EFAULT;
		}
	}
	osal_mutex_lock(&user->mutex);
	if (through != 0) {
		user->init_cnt = 0U;
	} else if (user->init_cnt > 0U) {
		user->init_cnt--;
	}
	des_info(dev, "%s cmd: %u %s", __func__, user->init_cnt,
		(user->init_cnt != 0U) ? "drop" : "real");
	if (user->init_cnt == 0U) {
		if (user->start_cnt > 0U) {
			deserial_stop_done(des);
			user->start_cnt = 0U;
		}
		deserial_op_cancel(des);
		user->pre_state = DES_PRE_STATE_DEFAULT;
		if (user->data_init != 0U) {
			deserial_data_deinit_do(des);
			user->data_init = 0U;
		}
		deserial_deinit_done(des);
	} else {
		ret = -EACCES;
	}
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial start operation request
 *
 * @param[in] des: deserial device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_start_req(struct deserial_device_s *des, unsigned long arg)
{
	int32_t ret = 0;
	struct deserial_user_s *user = &des->user;
	struct os_dev * dev = &des->osdev;
	uint32_t timeout_ms = 0U, wait = 0U;

	if (arg != 0U) {
		if (osal_copy_from_app(&timeout_ms, (uint32_t *)arg, sizeof(uint32_t)) != 0) {
			des_err(dev, "%s get data from user failed", __func__);
			return -EFAULT;
		}
	}
	osal_mutex_lock(&user->mutex);
	if (user->start_cnt == 0U) {
		if (user->pre_state == (uint32_t)DES_PRE_STATE_INITED) {
			user->pre_state = (uint32_t)DES_PRE_STATE_INITING;
			des_info(dev, "%s cmd: %s", __func__,
				g_des_pre_state_names[user->pre_state]);
		} else if (user->pre_state == (uint32_t)DES_PRE_STATE_INITING) {
			wait = 1;
		} else {
			des_info(dev, "%s cmd: %s drop", __func__,
				g_des_pre_state_names[user->pre_state]);
			ret = -EACCES;
		}
	} else {
		des_info(dev, "%s cmd: %s drop", __func__,
			g_des_pre_state_names[user->pre_state]);
			ret = -EACCES;
	}
	osal_mutex_unlock(&user->mutex);
	if (wait != 0U) {
		user->pre_done = (bool)false;
		if (timeout_ms != 0U) {
			osal_wait_event_interruptible_timeout(user->pre_wq,
				user->pre_done, timeout_ms);
		} else {
			osal_wait_event_interruptible(user->pre_wq, user->pre_done);
		}
		osal_mutex_lock(&user->mutex);
		if ((user->start_cnt == 0U) &&
			(user->pre_state == (uint32_t)DES_PRE_STATE_INITED)) {
			user->pre_state = (uint32_t)DES_PRE_STATE_STARTING;
			des_info(dev, "%s cmd: wait & %s", __func__,
			       g_des_pre_state_names[user->pre_state]);
		} else {
			des_info(dev, "%s cmd: wait & drop", __func__);
			ret = -EACCES;
		}
		osal_mutex_unlock(&user->mutex);
	}
	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial start operation result
 *
 * @param[in] des: deserial device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_start_result(struct deserial_device_s *des, unsigned long arg)
{
	int32_t ret = 0;
	struct deserial_user_s *user = &des->user;
	struct os_dev * dev = &des->osdev;
	uint32_t result = 0U, wake = 0U;

	if (arg == 0UL) {
		des_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}

	if (osal_copy_from_app(&result, (uint32_t *)arg, sizeof(uint32_t)) != 0U) {
		des_err(dev, "%s get data from user failed", __func__);
		return -EFAULT;
	}
	osal_mutex_lock(&user->mutex);
	if ((user->start_cnt == 0U) &&
		(user->pre_state == (uint32_t)DES_PRE_STATE_STARTING)) {
		if (result != 0U) {
			user->pre_state = (uint32_t)DES_PRE_STATE_INITED;
		} else {
			deserial_start_done(des);
			user->pre_state = (uint32_t)DES_PRE_STATE_STARTED;
		}
		wake = 1U;
	}
	if (result == 0U) {
		user->start_cnt ++;
	}
	osal_mutex_unlock(&user->mutex);
	if (wake != 0U) {
		des_info(dev, "%s cmd: %s wake", __func__,
				(result != 0U) ? "falied" : "done");
		user->pre_done = (bool)true;
		osal_wake_up(&user->pre_wq);
	} else {
		des_info(dev, "%s cmd: %s drop", __func__,
				(result != 0U) ? "falied" : "done");
	}

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial stop operation request and done
 *
 * @param[in] des: deserial device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_stop_req(struct deserial_device_s *des, unsigned long arg)
{
	int32_t ret = 0;
	struct deserial_user_s *user = &des->user;
	struct os_dev * dev = &des->osdev;
	int32_t through = 0;

	if (arg != 0U) {
		if (osal_copy_from_app(&through, (uint32_t *)arg, sizeof(uint32_t)) != 0) {
			des_err(dev, "%s get data from user failed", __func__);
			return -EFAULT;
		}
	}
	osal_mutex_lock(&user->mutex);
	if (through != 0) {
		user->start_cnt = 0U;
	} else if (user->start_cnt > 0U) {
		user->start_cnt--;
	}
	des_info(dev, "%s cmd: %u %s", __func__, user->start_cnt,
		(user->start_cnt != 0U) ? "drop" : "real");
	if (user->start_cnt == 0U) {
		deserial_stop_done(des);
		if (user->pre_state >= DES_PRE_STATE_INITED) {
			user->pre_state = DES_PRE_STATE_INITED;
		}
	} else {
		ret = -EACCES;
	}
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial stream operation wait and get call
 *
 * @param[in] des: deserial device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_stream_get(struct deserial_device_s *des, unsigned long arg)
{
	int32_t ret = 0;
	struct deserial_user_s *user = &des->user;
	struct os_dev * dev = &des->osdev;
	deserial_op_info_t op_info;

	if (arg == 0UL) {
		des_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}

	osal_mutex_lock(&user->mutex);

	ret = deserial_opu_do(des, &op_info);
	if (ret == 0) {
		if (osal_copy_to_app((void __user *)arg, (void *)&op_info, sizeof(deserial_op_info_t)) != 0U) {
			des_err(dev, "%s op_info to user %p error", __func__, (void __user *)arg);
			ret = -EFAULT;
		}
		des_debug(dev, "%s op %d:%d", __func__, op_info.type, op_info.data);
	} else {
		des_debug(dev, "%s op cancel", __func__);
	}

	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial stream operation done and put result
 *
 * @param[in] des: deserial device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_stream_put(struct deserial_device_s *des, unsigned long arg)
{
	int32_t ret = 0, result = 0;
	struct deserial_user_s *user = &des->user;
	struct os_dev * dev = &des->osdev;

	if (arg == 0UL) {
		des_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}

	if (osal_copy_from_app(&result, (uint32_t *)arg, sizeof(uint32_t)) != 0U) {
		des_err(dev, "%s get data from user failed", __func__);
		return -EFAULT;
	}
	osal_mutex_lock(&user->mutex);

	des_debug(dev, "%s op %d:%d %s", __func__,
		(user->op_info != NULL) ? (user->op_info->type) : -1,
		(user->op_info != NULL) ? (user->op_info->data) : -1,
		(result == 0) ? "done" : "error");

	ret = deserial_opu_done(des, result);

	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial stream on in driver by data_info start reg
 *
 * @param[in] des: deserial device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_stream_on(struct deserial_device_s *des, unsigned long arg)
{
	int32_t ret = 0, link = 0;
	struct deserial_user_s *user = &des->user;
	struct os_dev * dev = &des->osdev;

	if (arg != 0U) {
		if (osal_copy_from_app(&link, (uint32_t *)arg, sizeof(uint32_t)) != 0) {
			des_err(dev, "%s get data from user failed", __func__);
			return -EFAULT;
		}
	}
	osal_mutex_lock(&user->mutex);

	des_debug(dev, "%s stream on", __func__);
	ret = deserial_stream_reg_do(des, 1, link);

	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial stream off in driver by data_info start reg
 *
 * @param[in] des: deserial device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_stream_off(struct deserial_device_s *des, unsigned long arg)
{
	int32_t ret = 0, link = 0;
	struct deserial_user_s *user = &des->user;
	struct os_dev * dev = &des->osdev;

	if (arg != 0U) {
		if (osal_copy_from_app(&link, (uint32_t *)arg, sizeof(uint32_t)) != 0) {
			des_err(dev, "%s get data from user failed", __func__);
			return -EFAULT;
		}
	}
	osal_mutex_lock(&user->mutex);
	des_debug(dev, "%s stream off", __func__);
	ret = deserial_stream_reg_do(des, 0, link);

	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial param init as default
 *
 * @param[in] des: deserial device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void deserial_param_init(struct deserial_device_s *des)
{
	struct deserial_param_s *pa;

	if (des == NULL)
		return;
	pa = &des->param;

	/* param clear */
	memset(pa, 0, sizeof(struct deserial_param_s));
	/* param default not 0 */
	pa->op_timeout_ms = DESERIAL_PARAM_OP_TIMEOOUT_MS_DEFAULT;
	pa->op_retry_max = DESERIAL_PARAM_OP_RETRY_MAX_DEFAULT;

	return;
}
/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief camera deserial param reset for bit mask
 *
 * @param[in] des: camera deserial device struct
 * @param[in] mask: the param bit mask to reset
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void deserial_param_reset(struct deserial_device_s *des, int32_t mask)
{
	int32_t *param;
	int32_t i;

	if ((des == NULL) || (mask == 0))
		return;
	param = (int32_t *)((void *)(&des->param));
	for (i = 0; i < (sizeof(struct deserial_param_s)/sizeof(int32_t)); i++) {
		if ((mask & (0x1 << i)) == 0)
			continue;
		param[i] = 0;
	}
	return;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial device operation: open
 *
 * @param[in] des: deserial device struct
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t deserial_device_open(struct deserial_device_s *des)
{
	struct deserial_user_s *user;
	struct os_dev *dev;

	if (des == NULL)
		return -ENODEV;
	user = &des->user;
	dev = &des->osdev;

	osal_mutex_lock(&user->open_mutex);
	des_debug(dev, "open as %u", user->open_cnt);
	if (user->open_cnt == 0U) {
		osal_mutex_init(&user->mutex); /* PRQA S 3334 */ /* mutex_init macro */
		osal_waitqueue_init(&user->pre_wq);
		osal_waitqueue_init(&user->opu_wq);
		deserial_param_reset(des, DESERIAL_PARAM_OPEN_RESET_MASK);
		user->pre_state = DES_PRE_STATE_DEFAULT;
		user->op_state = DES_OP_STATE_DEFAULT;
		user->data_init = 0U;
		user->init_cnt = 0U;
		user->start_cnt = 0U;
	}
	user->open_cnt++;
	osal_mutex_unlock(&user->open_mutex);

	return 0;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial device operation: close
 *
 * @param[in] des: deserial device struct
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t deserial_device_close(struct deserial_device_s *des)
{
	struct deserial_user_s *user;
	struct deserial_link_s *link;
	struct os_dev *dev;
	int32_t i;

	if (des == NULL)
		return -ENODEV;
	user = &des->user;
	dev = &des->osdev;

	osal_mutex_lock(&user->open_mutex);
	if (user->open_cnt > 0U) {
		user->open_cnt--;
	}
	if (user->open_cnt == 0U) {
		if (des->attach_link != 0) {
			for (i = 0; i < DESERIAL_LINK_NUM_MAX; i++) {
				if ((des->attach_link & (0x1 << i)) == 0)
					continue;
				link = &des->link[i];
				des_debug(dev, "auto detach flow%d link%d from sensor%d",
					link->flow_id, i, link->sensor_id);
				link->flow_id = VCON_FLOW_INVALID;
				link->attach = 0;
				link->linked = 0;
				des->attach_link &= ~(0x1 << i);
			}
		}
		user->data_init = 0U;
	}
	des_debug(dev, "close as %u", user->open_cnt);
	osal_mutex_unlock(&user->open_mutex);

	return 0;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial get driver version
 *
 * @param[in] des: deserial device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_get_version(struct deserial_device_s *des, unsigned long arg)
{
	struct os_dev *dev;

	if (des == NULL)
		return -ENODEV;
	dev = &des->osdev;

	if (arg == 0UL) {
		des_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}
	if (osal_copy_to_app((void __user *)arg, (void *)&g_des.ver, sizeof(g_des.ver))) {
		des_err(dev, "%s deserial version to user error\n", __func__);
		return -ENOMEM;
	}
	return 0;
}

static int32_t deserial_state_check(struct deserial_device_s *des, unsigned long arg)
{
	struct deserial_user_s *user;
	struct os_dev *dev;
	uint32_t check_id;
	int32_t cur_pid = current->pid;
	int32_t init_pid;

	if (des == NULL)
		return -ENODEV;
	user = &des->user;
	dev = &des->osdev;

	if (arg == 0UL) {
		des_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}
	if (osal_copy_from_app(&check_id, (uint32_t *)arg, sizeof(uint32_t)) != 0) {
		des_err(dev, "%s check from user error\n", __func__);
		return -ENOMEM;
	}
	if (check_id >= DESERIAL_LINK_NUM_MAX) {
		des_err(dev, "%s link %d overrange error\n", __func__, check_id);
		return -ERANGE;
	}

	osal_mutex_lock(&user->mutex);
	init_pid = des->link[check_id].init_by_pid;
	if (init_pid == 0U)
		des_debug(dev, "%s proc %d check link%d idle\n",
			__func__, cur_pid, check_id);
	else if (init_pid == cur_pid)
		des_debug(dev, "%s proc %d check link%d by self\n",
			__func__, cur_pid, check_id);
	else
		des_info(dev, "%s proc %d check link%d by %d\n",
			__func__, cur_pid, check_id, init_pid);
	osal_mutex_unlock(&user->mutex);

	return init_pid;
}

static int32_t deserial_state_confirm(struct deserial_device_s *des, unsigned long arg)
{
	struct deserial_user_s *user;
	struct os_dev *dev;
	uint32_t confirm_id;
	int32_t cur_pid = current->pid;
	int32_t init_pid;

	if (des == NULL)
		return -ENODEV;
	user = &des->user;
	dev = &des->osdev;

	if (arg == 0UL) {
		des_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}
	if (osal_copy_from_app(&confirm_id, (uint32_t *)arg, sizeof(uint32_t)) != 0) {
		des_err(dev, "%s confirm from user error\n", __func__);
		return -ENOMEM;
	}
	if (confirm_id >= DESERIAL_LINK_NUM_MAX) {
		des_err(dev, "%s link %d overrange error\n", __func__, confirm_id);
		return -ERANGE;
	}

	osal_mutex_lock(&user->mutex);
	init_pid = des->link[confirm_id].init_by_pid;
	if (init_pid != 0)
		des_info(dev, "%s proc %d confirm link%d over %d\n",
			__func__, cur_pid, confirm_id, init_pid);
	else
		des_info(dev, "%s proc %d confirm link%d\n",
			__func__, cur_pid, confirm_id);
	des->link[confirm_id].init_by_pid = cur_pid;
	osal_mutex_unlock(&user->mutex);

	return 0;
}

static int32_t deserial_state_clear(struct deserial_device_s *des, unsigned long arg)
{
	struct deserial_user_s *user;
	struct os_dev *dev;
	uint32_t confirm_id;
	int32_t cur_pid = current->pid;
	int32_t init_pid;

	if (des == NULL)
		return -ENODEV;
	user = &des->user;
	dev = &des->osdev;

	if (arg == 0UL) {
		des_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}
	if (osal_copy_from_app(&confirm_id, (uint32_t *)arg, sizeof(uint32_t)) != 0) {
		des_err(dev, "%s deserial version to user error\n", __func__);
		return -ENOMEM;
	}
	if (confirm_id >= DESERIAL_LINK_NUM_MAX) {
		des_err(dev, "%s link %d overrange error\n", __func__, confirm_id);
		return -ERANGE;
	}

	osal_mutex_lock(&user->mutex);
	init_pid = des->link[confirm_id].init_by_pid;
	if (init_pid == cur_pid)
		des_info(dev, "%s proc %d clear link%d\n",
			__func__, cur_pid, confirm_id);
	if (init_pid != 0)
		des_info(dev, "%s proc %d clear link%d by %d\n",
			__func__, cur_pid, confirm_id, init_pid);
	else
		des_debug(dev, "%s proc %d clear link%d\n",
			__func__, cur_pid, confirm_id);
	des->link[confirm_id].init_by_pid = 0;
	osal_mutex_unlock(&user->mutex);

	return 0;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial device operation: ioctl
 *
 * @param[in] des: deserial device struct
 * @param[in] cmd: ioctl cmd
 * @param[in] arg: ioctl arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t deserial_device_ioctl(struct deserial_device_s *des, uint32_t cmd, unsigned long arg)
{
	struct os_dev *dev;
	int32_t ret;

	if (des == NULL)
		return -ENODEV;
	dev = &des->osdev;

	switch (cmd) {
	case DESERIAL_DATA_INIT:
		ret = deserial_data_init(des, arg);
		break;
	case DESERIAL_INIT_REQ:
		ret = deserial_init_req(des, arg);
		break;
	case DESERIAL_INIT_RESULT:
		ret = deserial_init_result(des, arg);
		break;
	case DESERIAL_DEINIT_REQ:
		ret = deserial_deinit_req(des, arg);
		break;
	case DESERIAL_START_REQ:
		ret = deserial_start_req(des, arg);
		break;
	case DESERIAL_START_RESULT:
		ret = deserial_start_result(des, arg);
		break;
	case DESERIAL_STOP_REQ:
		ret = deserial_stop_req(des, arg);
		break;
	case DESERIAL_STREAM_GET:
		ret = deserial_stream_get(des, arg);
		break;
	case DESERIAL_STREAM_PUT:
		ret = deserial_stream_put(des, arg);
		break;
	case DESERIAL_STREAM_ON:
		ret = deserial_stream_on(des, arg);
		break;
	case DESERIAL_STREAM_OFF:
		ret = deserial_stream_off(des, arg);
		break;
	case DESERIAL_GET_VERSION:
		ret = deserial_get_version(des, arg);
		break;
	case DESERIAL_STATE_CHECK:
		ret = deserial_state_check(des, arg);
		break;
	case DESERIAL_STATE_CONFIRM:
		ret = deserial_state_confirm(des, arg);
		break;
	case DESERIAL_STATE_CLEAR:
		ret = deserial_state_clear(des, arg);
		break;
	default:
		des_err(dev, "ioctl cmd 0x%x is err\n", cmd);
		ret = -1;
		break;
	}

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial device operation: get the status/info show
 *
 * @param[in] des: deserial device struct
 * @param[out] buf: the show string buffer to store
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t deserial_device_status_info_show(struct deserial_device_s *des, char *buf)
{
	struct deserial_miscdev_s *mdev;
	struct deserial_link_s *link;
	char *s = buf;
	int32_t l = 0, i;
	int32_t flow_id = -1;

	if ((des == NULL) || (s == NULL))
		return -EFAULT;
	mdev = &des->mdev;

	l += sprintf(&s[l], "%-15s: %s\n", "dev", mdev->name);
	if (mdev->client != NULL) {
		l += sprintf(&s[l], "%-15s: %s\n", "deserial", mdev->name);
		l += sprintf(&s[l], "%-15s: i2c%d@0x%02x\n", "device", mdev->bus_num, mdev->addr);
	} else {
		l += sprintf(&s[l], "%-15s: no client\n", "deserial");
	}
	for (i = 0; i < DESERIAL_LINK_NUM_MAX; i++) {
		link = &des->link[i];
		if (link->attach) {
			l += sprintf(&s[l], "%-14s%d: sensor%d - %s\n", "link", i, link->sensor_id, link->desp);
			l += sprintf(&s[l], "%-15s: %d\n", "flow_id", link->flow_id);
			l += sprintf(&s[l], "%-15s: %d\n", "poc_id", link->poc_id);
			if (flow_id < 0)
				flow_id = link->flow_id;
		} else if (link->linked == 0) {
			l += sprintf(&s[l], "%-14s%d: not maped\n", "link", i);
		} else {
			l += sprintf(&s[l], "%-14s%d: not attach\n", "link", i);
		}
	}
	if (flow_id >= 0) {
		i = deserial_vcon_event(flow_id, VCON_EVENT_INFO_SHOW, &s[l]);
		if (i >= 0)
			l += i;
	}
	i = deserial_vcon_event(des->index, VCON_EVENT_HW_SHOW, &s[l]);
	if (i >= 0)
		l += i;

	return l;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial device operation: get the status/cfg show
 *
 * @param[in] des: deserial device struct
 * @param[out] buf: the show string buffer to store
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t deserial_device_status_cfg_show(struct deserial_device_s *des, char *buf)
{
	struct deserial_user_s *user;
	struct deserial_info_data_s *des_d;
	char *s = buf;
	int32_t l = 0, i;

	if ((des == NULL) || (s == NULL))
		return -EFAULT;
	user = &des->user;
	des_d = &des->deserial_info;

	if (user->data_init == 0) {
		l += sprintf(&s[l], "%-15s: not init\n", des->mdev.name);
	} else {
		l += sprintf(&s[l], "%-15s: %s\n", "deserial_name", des_d->deserial_name);
		l += sprintf(&s[l], "%-15s: %d\n", "bus_num", des_d->bus_num);
		l += sprintf(&s[l], "%-15s: 0x%02x\n", "deserial_addr", des_d->deserial_addr);
		l += sprintf(&s[l], "%-15s: 0x%04x\n", "link_map", des_d->link_map);
		l += sprintf(&s[l], "%-15s: %d\n", "reg_width", des_d->reg_width);
		if (des_d->poc_name[0] != '\0') {
			l += sprintf(&s[l], "%-15s: %s\n", "poc_name", des_d->poc_name);
			l += sprintf(&s[l], "%-15s: 0x%02x\n", "poc_addr", des_d->poc_addr);
			l += sprintf(&s[l], "%-15s: 0x%04x\n", "poc_map", des_d->poc_map);
		}
		if (des_d->chip_addr != 0) {
			l += sprintf(&s[l], "%-15s: 0x%04x\n", "chip_addr", des_d->chip_addr);
			l += sprintf(&s[l], "%-15s: 0x%02x\n", "chip_id", des_d->chip_id);
		}
		if (des_d->gpio_enable != 0) {
			l += sprintf(&s[l], "%-15s: 0x%04x\n", "gpio_enable", des_d->gpio_enable);
			l += sprintf(&s[l], "%-15s: 0x%04x\n", "gpio_level", des_d->gpio_level);
			for (i = 0; i < DESERIAL_GPIO_NUM_MAX; i++) {
				if (des_d->gpio_enable & (0x1 << i))
					l += sprintf(&s[l], "%-14s%d: %d\n", "gpio", i, des_d->gpios[i]);
			}
		}
		for (i = 0; i < DESERIAL_LINK_NUM_MAX; i++) {
			if (des_d->link_desp[i][0] != '\0')
				l += sprintf(&s[l], "%-14s%d: %s\n", "link", i, des_d->link_desp[i]);
		}
	}

	return l;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial device operation: get the status/regs show
 *
 * @param[in] des: deserial device struct
 * @param[out] buf: the show string buffer to store
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t deserial_device_status_regs_show(struct deserial_device_s *des, char *buf)
{
	struct deserial_miscdev_s *mdev;
	char *s = buf;
	int32_t l = 0;

	if ((des == NULL) || (s == NULL))
		return -EFAULT;
	mdev = &des->mdev;

	if (mdev->client == NULL) {
		l += sprintf(&s[l], "%-15s: not init\n", mdev->name);
		return l;
	}

	l += sprintf(&s[l], "%s i2c%d@0x%02x: regs show to add\n",
		mdev->name, mdev->bus_num, mdev->addr);

	return l;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial device operation: get the status/user show
 *
 * @param[in] des: deserial device struct
 * @param[out] buf: the show string buffer to store
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t deserial_device_status_user_show(struct deserial_device_s *des, char *buf)
{
	struct deserial_user_s *user;
	struct deserial_link_s *link;
	char *s = buf;
	int32_t l = 0, i;

	if ((des == NULL) || (s == NULL))
		return -EFAULT;
	user = &des->user;

	l += sprintf(&s[l], "%-15s: %u\n", "user", user->open_cnt);
	l += sprintf(&s[l], "%-15s: %u\n", "init", user->init_cnt);
	l += sprintf(&s[l], "%-15s: %u\n", "start", user->start_cnt);
	l += sprintf(&s[l], "%-15s: %u\n", "data_init", user->data_init);
	l += sprintf(&s[l], "%-15s: %s\n", "pre_state", g_des_pre_state_names[user->pre_state]);
	l += sprintf(&s[l], "%-15s: %s\n", "op_state", g_des_op_state_names[user->op_state]);
	for (i = 0; i < DESERIAL_LINK_NUM_MAX; i++) {
		link = &des->link[i];
		if (link->init_by_pid != 0)
			l += sprintf(&s[l], "%-14s%d: PID %d\n", "link", i, link->init_by_pid);
	}

	return l;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial param index get for sysfs show/store
 *
 * @param[in] name: the param name string to match
 *
 * @return >=0:Success-the index, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_param_idx(const char *name)
{
	int32_t i;

	for (i = 0; i < ARRAY_SIZE(g_des_param_names); i++) {
		if (strcmp(g_des_param_names[i], name) == 0) {
			return i;
		}
	}

	return -1;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial device operation: get the param/xxx show
 *
 * @param[in] des: deserial device struct
 * @param[in] name: the param name string
 * @param[out] buf: the show string buffer to store
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t deserial_device_param_show(struct deserial_device_s *des, const char *name, char *buf)
{
	int32_t *param;
	char *s = buf;
	int32_t idx, l = 0;

	if ((des == NULL) || (name == NULL) || (s == NULL))
		return -EFAULT;

	param = (int32_t *)((void *)(&des->param));

	idx = deserial_param_idx(name);
	if (idx >= 0)
		l += sprintf(&s[l], "%d\n", param[idx]);

	return l;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial device operation: set the param/xxx store
 *
 * @param[in] des: deserial device struct
 * @param[in] name: the param name string
 * @param[in] buf: the store string to parse
 * @param[in] count: the store string buffer size
 *
 * @return >=0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t deserial_device_param_store(struct deserial_device_s *des, const char *name, const char *buf, int32_t count)
{
	uint32_t *param;
	int32_t ret = -EINVAL;
	int32_t idx;
	int32_t val;

	if ((des == NULL) || (buf == NULL))
		return -EFAULT;

	param = (int32_t *)((void *)(&des->param));

	idx = deserial_param_idx(name);
	if (idx >= 0) {
		ret = kstrtoint(buf, 0, &val);
		if (ret == 0)
			param[idx] = val;
	}

	return ((ret < 0) ? ret : (ssize_t)count);
}

/**
 * @NO{S10E02C11I}
 * @ASIL{B}
 * @brief deserial device attach to vpf flow
 *
 * @param[in] index: deserial device index
 * @param[in] flow_id: vpf flow id
 * @param[in] link_id: deserial link id to attach
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_attach(int32_t index, int32_t flow_id, int32_t link_id)
{
	struct deserial_device_s *des;
	struct deserial_user_s *user;
	struct deserial_link_s *link;
	struct os_dev * dev;

	if (index >= g_des.des_num)
		return -ENODEV;
	if (link_id >= DESERIAL_LINK_NUM_MAX)
		return -ERANGE;
	des = &g_des.des[index];
	user = &des->user;
	link = &des->link[link_id];
	dev = &des->osdev;

	osal_mutex_lock(&user->mutex);
	if (link->attach != 0) {
		des_err(dev, "%s error has attached by flow%d",
			__func__, link->flow_id);
		osal_mutex_unlock(&user->mutex);
		return -EBUSY;
	}
	link->flow_id = flow_id;
	link->attach = 1;
	des->attach_link |= (0x1 << link_id);
	if (link->linked == 0) {
		des_info(dev, "%s flow%d link%d warn not maped",
			__func__, flow_id, link_id);
	} else {
		if (link->sensor_id < 0)
			link->sensor_id = flow_id;
		des_info(dev, "%s flow%d link%d to sensor%d %s",
			__func__, flow_id, link_id, link->sensor_id, link->desp);
	}
	osal_mutex_unlock(&user->mutex);

	return 0;
}

/**
 * @NO{S10E02C11I}
 * @ASIL{B}
 * @brief deserial device detach to or remove from vpf flow
 *
 * @param[in] index: deserial device index
 * @param[in] flow_id: vpf flow id
 * @param[in] link_id: deserial link id to attach
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_detach(int32_t index, int32_t flow_id, int32_t link_id)
{
	struct deserial_device_s *des;
	struct deserial_user_s *user;
	struct deserial_link_s *link;
	struct os_dev * dev;

	if (index >= g_des.des_num)
		return -ENODEV;
	if (link_id >= DESERIAL_LINK_NUM_MAX)
		return -ERANGE;
	des = &g_des.des[index];
	user = &des->user;
	link = &des->link[link_id];
	dev = &des->osdev;

	osal_mutex_lock(&user->mutex);
	if (link->attach != 0) {
		if (link->flow_id != flow_id) {
			des_info(dev, "%s flow%d link%d warn not match attached flow%d",
				__func__, flow_id, link_id, link->flow_id);
		} else if (link->linked == 0) {
			des_info(dev, "%s flow%d link%d warn not maped",
				__func__, flow_id, link_id);
		} else {
			des_info(dev, "%s flow%d link%d from sensor%d %s",
				__func__, flow_id, link_id, link->sensor_id, link->desp);
		}
		link->flow_id = VCON_FLOW_INVALID;
		link->attach = 0;
		des->attach_link &= ~(0x1 << link_id);
	}
	osal_mutex_unlock(&user->mutex);

	return 0;
}

/**
 * @NO{S10E02C11I}
 * @ASIL{B}
 * @brief deserial device stream start callback function
 *
 * @param[in] index: deserial device index
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_start(int32_t index)
{
	struct deserial_device_s *des;
	struct deserial_user_s *user;
	struct os_dev * dev;
	int32_t ret;

	if (index >= g_des.des_num)
		return -ENODEV;
	des = &g_des.des[index];
	user = &des->user;
	dev = &des->osdev;

	osal_mutex_lock(&user->mutex);
	if (des->attach_link == 0) {
		des_err(dev, "%s error not attached", __func__);
		osal_mutex_unlock(&user->mutex);
		return -EACCES;
	}
	des_debug(dev, "%s links 0x%x starting ...",
		__func__, des->attach_link);
	ret = deserial_streaming_do(des, 1);
	if (ret < 0) {
		des_err(dev, "%s links 0x%x start error %d",
			__func__, des->attach_link, ret);
	} else {
		des_info(dev, "%s links 0x%x start done",
			__func__, des->attach_link);
	}
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C11I}
 * @ASIL{B}
 * @brief deserial device stream stop callback fucntion
 *
 * @param[in] index: deserial device index
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_stop(int32_t index)
{
	struct deserial_device_s *des;
	struct deserial_user_s *user;
	struct os_dev * dev;
	int32_t ret;

	if (index >= g_des.des_num)
		return -ENODEV;
	des = &g_des.des[index];
	user = &des->user;
	dev = &des->osdev;

	osal_mutex_lock(&user->mutex);
	if (des->attach_link == 0) {
		des_err(dev, "%s error not attached", __func__);
		osal_mutex_unlock(&user->mutex);
		return -EACCES;
	}
	des_debug(dev, "%s links 0x%x stopping ...",
		__func__, des->attach_link);
	ret = deserial_streaming_do(des, 0);
	if (ret < 0) {
		des_err(dev, "%s links 0x%x stop error %d",
			__func__, des->attach_link, ret);
	} else {
		des_info(dev, "%s links 0x%x stop done",
			__func__, des->attach_link);
	}
	osal_mutex_unlock(&user->mutex);

	return 0;
}

/**
 * @NO{S10E02C11I}
 * @ASIL{B}
 * @brief deserial device event callback function
 *
 * @param[in] index: deserial device index
 * @param[in] event_id: event id
 * @param[in] event_data: event data
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_event(int32_t index, int32_t event_id, void *event_data)
{
	struct deserial_device_s *des;
	struct deserial_user_s *user;

	if (index >= g_des.des_num)
		return -ENODEV;
	des = &g_des.des[index];
	user = &des->user;

	osal_mutex_lock(&user->mutex);

	osal_mutex_unlock(&user->mutex);

	return 0;
}

/**
 * @NO{S10E02C11I}
 * @ASIL{B}
 * @brief deserial device set vcon callback function
 *
 * @param[in] cb: vcon event callback function
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void deserial_setcb(vcon_cb cb)
{
	g_des.vcon_event_cb = cb;
	// des_info(NULL, "%s done", __func__);
}


/**
 * deserial driver ops for vin vcon
 */
static struct vcon_sub_ops_s deserial_vcon_ops = {
	.attach = deserial_attach,
	.detach = deserial_detach,
	.start = deserial_start,
	.stop = deserial_stop,
	.event = deserial_event,
	.setcb = deserial_setcb,
	.end_magic = VCON_SUB_OPS_END_MAGIC(VCON_COPT_DESERIAL),
};

DECLARE_VIO_CALLBACK_OPS(deserial_vcon_cops, DESERIAL_COPS_MAGIC, &deserial_vcon_ops);

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial cops init for other drivers
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t deserial_cops_init(void)
{
	int32_t ret;

	ret = vio_register_callback_ops(&cb_deserial_vcon_cops, DESERIAL_COP_MODULE, DESERIAL_VCON_COP_TYPE);
	if (ret < 0) {
		des_err(NULL, "deserial vcon cops register failed %d\n", ret);
		return ret;
	}

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial cops exit unregister
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void deserial_cops_exit(void)
{
	vio_unregister_callback_ops(DESERIAL_COP_MODULE, DESERIAL_VCON_COP_TYPE);
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial device init for probe
 *
 * @param[in] des: deserial device struct
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t deserial_device_init(struct deserial_device_s *des)
{
	int32_t i;

	if (des == NULL)
		return -EINVAL;

	osal_mutex_init(&des->user.open_mutex);
	osal_waitqueue_init(&des->user.opk_wq);
	deserial_param_init(des);

	for (i = 0; i < DESERIAL_LINK_NUM_MAX; i++) {
		des->link[i].flow_id = VCON_FLOW_INVALID;
	}

	return 0;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial device exit
 *
 * @param[in] des: deserial device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void deserial_device_exit(struct deserial_device_s *des)
{
	/* do nothing */
	return;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial driver init
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t deserial_driver_init(void)
{
	int32_t ret;

	ret = deserial_cops_init();
	if (ret < 0)
		return ret;

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial driver exit
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void deserial_driver_exit(void)
{
	deserial_cops_exit();
}
