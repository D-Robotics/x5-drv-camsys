/*   Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

/**
 * @file hobot_sensor_ops.c
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#include "hobot_sensor_ops.h"
#include "hobot_sensor_cops.h"
#include "camera_ctrl.h"

/**
 * @var g_sen_ev_state_names
 * sensor event op state name string array
 */
static const char *g_sen_ev_state_names[] = SENSOR_EV_STATE_NAMES;

/**
 * @var g_sen_pre_state_names
 * sensor pre state name string array
 */
static const char *g_sen_pre_state_names[] = SENSOR_PRE_STATE_NAMES;

/**
 * @var g_sen_param_names
 * sensor param name string array
 */
static const char *g_sen_param_names[] = SENSOR_PARAM_NAMES;

/**
 * @var g_sen
 * global sensor driver struct
 */
static struct sensor_s g_sen = {
	.ver = { SENSOR_VER_MAJOR, SENSOR_VER_MINOR },
};

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief get sensor global struct
 *
 * @return !NULL: the global sensor_s struct pointer
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
struct sensor_s* sensor_global(void)
{
	return &g_sen;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief get sensor struct pointer by port index
 *
 * @param[in] port: the sensor port index
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
struct sensor_device_s* sensor_dev_get(int32_t port)
{
	if (port >= g_sen.sen_num)
		return NULL;

	return &g_sen.sen[port];
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief get sensor osdev struct pointer by port index
 *
 * @param[in] port: the sensor port index
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
struct os_dev* sensor_osdev_get(int32_t port)
{
	if (port >= g_sen.sen_num)
		return NULL;

	return &g_sen.sen[port].osdev;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief get sensor struct pointer by flow id
 *
 * @param[in] flow_id: the sensor attached flow_id
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
struct sensor_device_s* sensor_dev_get_by_flow(int32_t flow_id)
{
	if (flow_id >= SENSOR_NUM_MAX)
		return NULL;

	return g_sen.sen_flow[flow_id];
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor device operation: open
 *
 * @param[in] sen: sensor device struct
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
int32_t sensor_device_open(struct sensor_device_s *sen)
{
	int32_t ret;

	if (sen == NULL)
		return -ENODEV;

	ret = camera_dev_open(sen);
	if (ret < 0)
		return ret;

	return ret;
}

static int32_t sensor_stop(int32_t index);
static int32_t sensor_detach(int32_t index, int32_t flow_id, int32_t add_id);
/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor device operation: release
 *
 * @param[in] sen: sensor device struct
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
int32_t sensor_device_close(struct sensor_device_s *sen)
{
	struct sensor_link_s *link;

	if (sen == NULL)
		return -ENODEV;
	link = &sen->link;

	if (link->attach != 0) {
		sensor_stop(sen->port);
		sensor_detach(sen->port, link->flow_id, link->ctx_id);
	}

	camera_dev_release(sen);

	return 0;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor get driver version
 *
 * @param[in] sen: camera sensor device struct
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
static int32_t sensor_get_version(struct sensor_device_s *sen, unsigned long arg)
{
	struct os_dev *dev;

	if (sen == NULL)
		return -ENODEV;
	dev = &sen->osdev;

	if (arg == 0UL) {
		sen_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}
	if (osal_copy_to_app((void __user *)arg, (void *)&g_sen.ver, sizeof(g_sen.ver))) {
		sen_err(dev, "%s sensor version to user error\n", __func__);
		return -ENOMEM;
	}
	return 0;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor operation: ioctl
 *
 * @param[in] sen: sensor device struct
 * @param[in] cmd: ioctl cmd
 * @param[in] arg: ioctl arg
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t sensor_device_ioctl(struct sensor_device_s *sen, uint32_t cmd, unsigned long arg)
{
	struct os_dev *dev;
	int32_t ret;

	if (sen == NULL)
		return -ENODEV;
	dev = &sen->osdev;

	switch (cmd) {
	case SENSOR_TUNING_PARAM:
		ret = camera_tuning_update_param(sen, arg);
		break;
	case SENSOR_OPEN_CNT:
	case SENSOR_SET_START_CNT:
	case SENSOR_GET_START_CNT:
	case SENSOR_USER_LOCK:
	case SENSOR_USER_UNLOCK:
	case SENSOR_SET_INIT_CNT:
	case SENSOR_GET_INIT_CNT:
		sen_err(dev, "ioctl cmd 0x%x not supported\n", cmd);
		ret = -EINVAL;
		break;
	case SENSOR_AE_SHARE:
		ret = camera_set_ae_share(sen, arg);
		break;
	case SENSOR_INPUT_PARAM:
		ret = camera_set_input_param(sen, arg);
		break;
	case SENSOR_SET_INTRINSIC_PARAM:
		ret = camera_set_intrinsic_param(sen, arg);
		break;
	case SENSOR_GET_INTRINSIC_PARAM:
		ret = camera_get_intrinsic_param(sen, arg);
		break;
	case SENSOR_INIT_REQ:
		ret = camera_init_req(sen, arg);
		break;
	case SENSOR_INIT_RESULT:
		ret = camera_init_result(sen, arg);
		break;
	case SENSOR_DEINIT_REQ:
		ret = camera_deinit_req(sen, arg);
		break;
	case SENSOR_START:
		ret = camera_start(sen, arg);
		break;
	case SENSOR_STOP:
		ret = camera_stop(sen, arg);
		break;
	case SENSOR_EVENT_GET:
		ret = camera_event_get(sen, arg);
		break;
	case SENSOR_EVENT_PUT:
		ret = camera_event_put(sen, arg);
		break;
	case SENSOR_UPDATE_AE_INFO:
		ret = camera_update_ae_info(sen, arg);
		break;
	case SENSOR_GET_VERSION:
		ret = sensor_get_version(sen, arg);
		break;
	default:
		sen_err(dev, "ioctl cmd 0x%x is err\n", cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device call vcon event callback function
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
static int32_t sensor_vcon_event(int32_t flow_id, int32_t event_id, void *event_data)
{
	if (g_sen.vcon_event_cb != NULL)
		return g_sen.vcon_event_cb(flow_id, event_id, event_data);

	return 0;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device operation: get the status/info show
 *
 * @param[in] sen: sensor device struct
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
int32_t sensor_device_status_info_show(struct sensor_device_s *sen, char *buf)
{
	struct sensor_miscdev_s *mdev;
	struct sensor_link_s *link;
	char *s = buf;
	int32_t l = 0, ret;

	if ((sen == NULL) || (s == NULL))
		return -EFAULT;
	mdev = &sen->mdev;
	link = &sen->link;

	l += sprintf(&s[l], "%-15s: %s\n", "dev", mdev->name);
	if (mdev->client != NULL) {
		l += sprintf(&s[l], "%-15s: %s\n", "sensor", sen->camera_param.sensor_name);
		l += sprintf(&s[l], "%-15s: i2c%d@0x%02x\n", "device", mdev->bus_num, mdev->addr);
	} else if (mdev->dummy_sensor != 0) {
		l += sprintf(&s[l], "%-15s: %s\n", "sensor", sen->camera_param.sensor_name);
		l += sprintf(&s[l], "%-15s: dummy\n", "device");
	} else {
		l += sprintf(&s[l], "%-15s: not open\n", "sensor");
	}
	if (link->attach != 0) {
		l += sprintf(&s[l], "%-15s: %d\n", "flow_id", link->flow_id);
		l += sprintf(&s[l], "%-15s: %d\n", "ctx_id", link->ctx_id);
		ret = sensor_vcon_event(link->flow_id, VCON_EVENT_ATTR_SHOW, &s[l]);
		if (ret >= 0)
			l += ret;
	} else {

		l += sprintf(&s[l], "%-15s: not attach\n", "flow_id");
	}

	return l;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device operation: get the status/cfg show
 *
 * @param[in] sen: sensor device struct
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
int32_t sensor_device_status_cfg_show(struct sensor_device_s *sen, char *buf)
{
	struct sensor_user_s *user;
	struct sensor_tuning_data_s *cam_p;
	struct sensor_data *sen_d;
	struct sensor_param_s *pa;
	char *s = buf;
	int32_t l = 0;

	if ((sen == NULL) || (s == NULL))
		return -EFAULT;
	user = &sen->user;
	cam_p = &sen->camera_param;
	sen_d = &cam_p->sensor_data;
	pa = &sen->param;

	if (user->data_init == 0) {
		l += sprintf(&s[l], "%-15s: not init\n", sen->mdev.name);
		if ((pa->frame_debug & SENSOR_FRAME_DEBUG_CFG_SHOW) == 0U)
			return l;
	}

	l += sprintf(&s[l], "%-15s: %s\n", "sensor_name", cam_p->sensor_name);
	l += sprintf(&s[l], "%-15s: %d\n", "bus_num", cam_p->bus_num);
	l += sprintf(&s[l], "%-15s: 0x%02x\n", "sensor_addr", cam_p->sensor_addr);
	l += sprintf(&s[l], "%-15s: %d\n", "reg_width", cam_p->reg_width);

	l += sprintf(&s[l], "%-15s: %d\n", "tuning_type", sen_d->tuning_type);
	l += sprintf(&s[l], "%-15s: %d\n", "step_gain", sen_d->step_gain);
	l += sprintf(&s[l], "%-15s: %d\n", "VMAX", sen_d->VMAX);
	l += sprintf(&s[l], "%-15s: %d\n", "HMAX", sen_d->HMAX);
	l += sprintf(&s[l], "%-15s: %d\n", "active_width", sen_d->active_width);
	l += sprintf(&s[l], "%-15s: %d\n", "active_height", sen_d->active_height);
	l += sprintf(&s[l], "%-15s: %d\n", "fps", sen_d->fps);
	l += sprintf(&s[l], "%-15s: %d\n", "data_width", sen_d->data_width);
	l += sprintf(&s[l], "%-15s: %d\n", "bayer_start", sen_d->bayer_start);
	l += sprintf(&s[l], "%-15s: %d\n", "bayer_pattern", sen_d->bayer_pattern);
	l += sprintf(&s[l], "%-15s: %d\n", "exposure_max_bit_width", sen_d->exposure_max_bit_width);
	l += sprintf(&s[l], "%-15s: %d\n", "gain_max", sen_d->gain_max);
	l += sprintf(&s[l], "%-15s: %d\n", "lines_per_second", sen_d->lines_per_second);
	l += sprintf(&s[l], "%-15s: %d\n", "analog_gain_max", sen_d->analog_gain_max);
	l += sprintf(&s[l], "%-15s: %d\n", "digital_gain_max", sen_d->digital_gain_max);
	l += sprintf(&s[l], "%-15s: %d\n", "exposure_time_max", sen_d->exposure_time_max);
	l += sprintf(&s[l], "%-15s: %d\n", "exposure_time_min", sen_d->exposure_time_min);
	l += sprintf(&s[l], "%-15s: %d\n", "exposure_time_long_max", sen_d->exposure_time_long_max);

	switch (cam_p->mode) {
	case SENSOR_LINEAR:
		l += sprintf(&s[l], "%-15s: %d - LINEAR\n", "mode", cam_p->mode);
		l += sprintf(&s[l], "%-15s: %d - 0x%04x\n", "param_hold",
			cam_p->normal.param_hold_length, cam_p->normal.param_hold);
		l += sprintf(&s[l], "%-15s: %d - 0x%04x\n", "s_line",
			cam_p->normal.s_line_length, cam_p->normal.s_line);
		l += sprintf(&s[l], "%-15s: %d\n", "again_control_num",
			cam_p->normal.again_control_num);
		break;
	case SENSOR_DOL2:
		l += sprintf(&s[l], "%-15s: %d - DOL2\n", "mode", cam_p->mode);
		l += sprintf(&s[l], "%-15s: %d - 0x%04x\n", "param_hold",
			cam_p->dol2.param_hold_length, cam_p->dol2.param_hold);
		l += sprintf(&s[l], "%-15s: %d - 0x%04x\n", "s_line",
			cam_p->dol2.s_line_length, cam_p->dol2.s_line);
		l += sprintf(&s[l], "%-15s: %d - 0x%04x\n", "m_line",
			cam_p->dol2.m_line_length, cam_p->dol2.m_line);
		l += sprintf(&s[l], "%-15s: %d\n", "again_control_num",
			cam_p->dol2.again_control_num);
		break;
	case SENSOR_DOL3:
		l += sprintf(&s[l], "%-15s: %d - DOL3\n", "mode", cam_p->mode);
		l += sprintf(&s[l], "%-15s: %d - 0x%04x\n", "param_hold",
			cam_p->dol3.param_hold_length, cam_p->dol3.param_hold);
		l += sprintf(&s[l], "%-15s: %d - 0x%04x\n", "s_line",
			cam_p->dol3.s_line_length, cam_p->dol3.s_line);
		l += sprintf(&s[l], "%-15s: %d - 0x%04x\n", "m_line",
			cam_p->dol3.m_line_length, cam_p->dol3.m_line);
		l += sprintf(&s[l], "%-15s: %d - 0x%04x\n", "l_line",
			cam_p->dol3.l_line_length, cam_p->dol3.l_line);
		l += sprintf(&s[l], "%-15s: %d\n", "again_control_num",
			cam_p->normal.again_control_num);
		break;
	case SENSOR_DOL4:
		l += sprintf(&s[l], "%-15s: %d - DOL4\n", "mode", cam_p->mode);
		break;
	case SENSOR_PWL:
		l += sprintf(&s[l], "%-15s: %d - PWL\n", "mode", cam_p->mode);
		l += sprintf(&s[l], "%-15s: %d - 0x%04x\n", "param_hold",
			cam_p->pwl.param_hold_length, cam_p->pwl.param_hold);
		l += sprintf(&s[l], "%-15s: %d\n", "l_s_mode", cam_p->pwl.l_s_mode);
		l += sprintf(&s[l], "%-15s: %d\n", "line_num", cam_p->pwl.line_num);
		l += sprintf(&s[l], "%-15s: %d - 0x%04x\n", "line",
				cam_p->pwl.line_length, cam_p->pwl.line);
		break;
	default:
		l += sprintf(&s[l], "%-15s: %d - UNKOWN\n", "mode", cam_p->mode);
	}

	return l;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device operation: get the status/iparam show
 *
 * @param[in] sen: sensor device struct
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
int32_t sensor_device_status_iparam_show(struct sensor_device_s *sen, char *buf)
{
	struct sensor_user_s *user;
	struct cam_usr_info_s *user_info;
	struct sensor_parameter_s *sp;
	struct sensor_intrinsic_parameter_s *sip;
	struct sensor_miscdev_s *mdev;
	struct sensor_param_s *pa;
	char *s = buf;
	int32_t l = 0, i;
	uint8_t *v;

	if ((sen == NULL) || (s == NULL))
		return -EFAULT;
	user = &sen->user;
	user_info = &sen->user_info;
	sp = &user_info->iparam.sns_param;
	sip = &user_info->iparam.sns_intrinsic_param;
	mdev = &sen->mdev;
	pa  = &sen->param;

#define SENSOR_STATUS_IPARAM_FLOAT(p, name, type) do {\
		v = (uint8_t *)&p->name; \
		i = sizeof(type); \
		l += sprintf(&s[l], "%-15s: %s 0x", #name, #type); \
		while (i > 0) { \
			i--; \
			l += sprintf(&s[l], "%02x", v[i]); \
		} \
		l += sprintf(&s[l], "\n"); \
	} while (0)

	if (user->devflag == 0) {
		l += sprintf(&s[l], "%-15s: iparam not set\n", sen->mdev.name);
		if ((pa->frame_debug & SENSOR_FRAME_DEBUG_CFG_SHOW) == 0U)
			return l;
	}
	l += sprintf(&s[l], "%-15s: %s iparam state %d\n",
		mdev->name, sen->camera_param.sensor_name, user_info->state);

	l += sprintf(&s[l], "%-15s: ------------\n", "sensor_param");
	l += sprintf(&s[l], "%-15s: %d\n", "frame_length", sp->frame_length);
	l += sprintf(&s[l], "%-15s: %d\n", "line_length", sp->line_length);
	l += sprintf(&s[l], "%-15s: %d\n", "width", sp->width);
	l += sprintf(&s[l], "%-15s: %d\n", "height", sp->height);
	SENSOR_STATUS_IPARAM_FLOAT(sp, fps, float);
	l += sprintf(&s[l], "%-15s: %d\n", "pclk", sp->pclk);
	l += sprintf(&s[l], "%-15s: %d\n", "exp_num", sp->exp_num);
	l += sprintf(&s[l], "%-15s: %d\n", "lines_per_second", sp->lines_per_second);
	l += sprintf(&s[l], "%-15s: %s\n", "version", sp->version);
	l += sprintf(&s[l], "%-15s: %s\n", "vendor", sp->vendor);
	l += sprintf(&s[l], "%-15s: %s\n", "sensor_name", sp->sensor_name);
	l += sprintf(&s[l], "%-15s: %s\n", "fov", sp->fov);
	l += sprintf(&s[l], "%-15s: %s\n", "bayer_pattern", sp->bayer_pattern);
	l += sprintf(&s[l], "%-15s: %s\n", "calb_name", sp->calb_name);
	l += sprintf(&s[l], "%-15s: %s\n", "sensor_hw_version", sp->sensor_hw_version);

	l += sprintf(&s[l], "%-15s: ------------\n", "intrinsic_param");
	l += sprintf(&s[l], "%-15s: %d.%d\n", "version maj.min", sip->major_version, sip->minor_version);
	l += sprintf(&s[l], "%-15s: 0x%04x.0x%04x\n", "id ven.module", sip->vendor_id, sip->module_id);
	l += sprintf(&s[l], "%-15s: 0x%08x\n", "module_serial", sip->module_serial);
	l += sprintf(&s[l], "%-15s: %d/%d/%d\n", "date", sip->year, sip->month, sip->day);
	l += sprintf(&s[l], "%-15s: %d\n", "cam_type", sip->cam_type);
	l += sprintf(&s[l], "%-15s: m-0x%x e-0x%x c-0x%x p-0x%x d-0x%x\n", "flags",
		sip->module_falg, sip->efl_flag, sip->cod_flag, sip->pp_flag, sip->distortion_flag);
	l += sprintf(&s[l], "%-15s: %dx%d\n", "image wxh", sip->image_width, sip->image_height);
	l += sprintf(&s[l], "%-15s: 0x%08x\n", "crc32_1", sip->crc32_1);
	l += sprintf(&s[l], "%-15s: 0x%08x\n", "crcgroup_1", sip->crc_group1);
	l += sprintf(&s[l], "%-15s: %d\n", "distort_params", sip->distort_params);
	l += sprintf(&s[l], "%-15s: %d\n", "distort_model_type", sip->distort_model_type);
	l += sprintf(&s[l], "%-15s: %s\n", "serial_num", sip->serial_num);
	SENSOR_STATUS_IPARAM_FLOAT(sip, pp_x, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, pp_y, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, cam_skew, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, focal_u, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, focal_v, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, center_u, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, center_v, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, hfov, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k1, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k2, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k3, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k4, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k5, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k6, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k7, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k8, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k9, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k10, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k11, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k12, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k13, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k14, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, focal_u_2, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, focal_v_2, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, center_u_2, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, center_v_2, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k1_2, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k2_2, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k3_2, double);
	SENSOR_STATUS_IPARAM_FLOAT(sip, k4_2, double);

	return l;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device operation: get the status/regs show
 *
 * @param[in] sen: sensor device struct
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
int32_t sensor_device_status_regs_show(struct sensor_device_s *sen, char *buf)
{
	struct sensor_miscdev_s *mdev;
	char *s = buf;
	int32_t l = 0;

	if ((sen == NULL) || (s == NULL))
		return -EFAULT;
	mdev = &sen->mdev;

	if (mdev->client == NULL) {
		l += sprintf(&s[l], "%-15s: not init\n", mdev->name);
		return l;
	}

	l += sprintf(&s[l], "%s i2c%d@0x%02x: regs show to add\n",
		mdev->name, mdev->bus_num, mdev->addr);

	return l;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device operation: get the status/user show
 *
 * @param[in] sen: sensor device struct
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
int32_t sensor_device_status_user_show(struct sensor_device_s *sen, char *buf)
{
	struct sensor_user_s *user;
	char *s = buf;
	int32_t l = 0;

	if ((sen == NULL) || (s == NULL))
		return -EFAULT;
	user = &sen->user;

	l += sprintf(&s[l], "%-15s: %u\n", "user", user->open_cnt);
	l += sprintf(&s[l], "%-15s: %u\n", "init", user->init_cnt);
	l += sprintf(&s[l], "%-15s: %u\n", "start", user->start_cnt);
	l += sprintf(&s[l], "%-15s: %u\n", "data_init", user->data_init);
	l += sprintf(&s[l], "%-15s: %s\n", "pre_state", g_sen_pre_state_names[user->pre_state]);
	l += sprintf(&s[l], "%-15s: %s\n", "ev_state", g_sen_ev_state_names[user->ev_state]);

	return l;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device operation: get the status/frame show
 *
 * @param[in] sen: sensor device struct
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
int32_t sensor_device_status_frame_show(struct sensor_device_s *sen, char *buf)
{
	struct sensor_tuning_data_s *cam_p;
	struct sensor_link_s *link;
	struct sensor_param_s *pa;
	struct sensor_frame_s frame;
	struct sensor_frame_record_s *r;
	uint32_t lost_count, avg, fpks = 0U;
	const char *ctrlm_names[] = SENSOR_CTRLM_NAMES;
	int32_t ctrl_mode;
	char *s = buf;
	int32_t l = 0;

	if ((sen == NULL) || (s == NULL))
		return -EFAULT;
	cam_p = &sen->camera_param;
	link = &sen->link;
	pa = &sen->param;
	memcpy(&frame, &sen->frame, sizeof(frame));

	/* sensor info */
	if (cam_p->sensor_name[0] != '\0') {
		l += sprintf(&s[l], "%-15s: %d@0x%02x\n", cam_p->sensor_name,
			cam_p->bus_num, cam_p->sensor_addr);
	}
	/* frame flow */
	if (link->attach) {
		l += sprintf(&s[l], "%-15s: %d\n", "flow_id", link->flow_id);
	} else {
		l += sprintf(&s[l], "%-15s: detached\n", "flow_id");
	}
	/* frame count */
	if ((pa->frame_debug & SENSOR_FRAME_DEBUG_TIME_EVENT) != 0U) {
		r = &frame.fs;
		avg = (r->count > 1U) ? (uint32_t)(r->diff_ns_all / (r->count - 1U)) : 0U;
		l += sprintf(&s[l], "%-15s: %-10u\t%llu.%09llu\t%u\n", "fs_count",
			r->count, SENSOR_NS2S(r->ts_ns_last), SENSOR_NS2SNS(r->ts_ns_last),
			r->warn);
		l += sprintf(&s[l], "%-15s: %u.%06u/%u\t%u.%06u/%u\t%u.%06u\n", "fs_diff",
			SENSOR_NS2MS(r->diff_ns_min), SENSOR_NS2MSNS(r->diff_ns_min), r->count_min,
			SENSOR_NS2MS(r->diff_ns_max), SENSOR_NS2MSNS(r->diff_ns_max), r->count_max,
			SENSOR_NS2MS(avg), SENSOR_NS2MSNS(avg));
		if (avg != 0U)
			fpks = SENSOR_NS2FPKS(avg);
		r = &frame.fe;
		avg = (r->count > 1U) ? (uint32_t)(r->diff_ns_all / (r->count - 1U)) : 0U;
		l += sprintf(&s[l], "%-15s: %-10u\t%llu.%09llu\t%u\n", "fe_count",
			r->count, SENSOR_NS2S(r->ts_ns_last), SENSOR_NS2SNS(r->ts_ns_last),
			r->warn);
		l += sprintf(&s[l], "%-15s: %u.%06u/%u\t%u.%06u/%u\t%u.%06u\n", "fe_diff",
			SENSOR_NS2MS(r->diff_ns_min), SENSOR_NS2MSNS(r->diff_ns_min), r->count_min,
			SENSOR_NS2MS(r->diff_ns_max), SENSOR_NS2MSNS(r->diff_ns_max), r->count_max,
			SENSOR_NS2MS(avg), SENSOR_NS2MSNS(avg));
		r = &frame.fs_fe;
		avg = (r->count > 0U) ? (uint32_t)(r->diff_ns_all / r->count) : 0U;
		l += sprintf(&s[l], "%-15s: %-10u\t%llu.%09llu\t%u\n", "frame",
			r->count, SENSOR_NS2S(r->ts_ns_last), SENSOR_NS2SNS(r->ts_ns_last),
			r->warn);
		l += sprintf(&s[l], "%-15s: %u.%06u/%u\t%u.%06u/%u\t%u.%06u\n", "frame_vtime",
			SENSOR_NS2MS(r->diff_ns_min), SENSOR_NS2MSNS(r->diff_ns_min), r->count_min,
			SENSOR_NS2MS(r->diff_ns_max), SENSOR_NS2MSNS(r->diff_ns_max), r->count_max,
			SENSOR_NS2MS(avg), SENSOR_NS2MSNS(avg));
		l += sprintf(&s[l], "%-15s: %u.%03u\n", "frame_fps", fpks / 1000, fpks % 1000);
	} else {
		r = &frame.fs;
		l += sprintf(&s[l], "%-15s: %u\t%u\n", "fs_count", r->count, r->warn);
		r = &frame.fe;
		l += sprintf(&s[l], "%-15s: %u\t%u\n", "fe_count", r->count, r->warn);
	}
	/* frame info */
	l += sprintf(&s[l], "%-15s: %u - %s\n", "frame_event", frame.event,
		(frame.event == SENSOR_FRAME_END) ? "FE" : "FS");
	if (frame.fs.count > frame.fe.count)
		lost_count = (frame.fs.count - frame.fe.count) + frame.event - 1U;
	else if (frame.fs.count < frame.fe.count)
		lost_count = (frame.fe.count - frame.fs.count) - frame.event + 1U;
	else
		lost_count = 1U - frame.event;
	if ((frame.fs.count > 0) && (lost_count != 0UL))
		l += sprintf(&s[l], "%-15s: %u\n", "frame_lost", lost_count);
	/* frame ae */
	if ((pa->frame_debug & SENSOR_FRAME_DEBUG_TIME_2A) != 0U) {
		r = &frame.update_2a;
		avg = (r->count > 1U) ? (uint32_t)(r->diff_ns_all / (r->count - 1U)) : 0U;
		l += sprintf(&s[l], "%-15s: %-10u\t%llu.%09llu\t%u\n", "2a_update",
			r->count, SENSOR_NS2S(r->ts_ns_last), SENSOR_NS2SNS(r->ts_ns_last),
			r->warn);
		l += sprintf(&s[l], "%-15s: %u.%06u/%u\t%u.%06u/%u\t%u.%06u\n", "loop_time",
			SENSOR_NS2MS(r->diff_ns_min), SENSOR_NS2MSNS(r->diff_ns_min), r->count_min,
			SENSOR_NS2MS(r->diff_ns_max), SENSOR_NS2MSNS(r->diff_ns_max), r->count_max,
			SENSOR_NS2MS(avg), SENSOR_NS2MSNS(avg));
		r = &frame.start_2a;
		avg = (r->count > 1U) ? (uint32_t)(r->diff_ns_all / (r->count - 1U)) : 0U;
		l += sprintf(&s[l], "%-15s: %-10u\t%llu.%09llu\t%u\n", "2a_start",
			r->count, SENSOR_NS2S(r->ts_ns_last), SENSOR_NS2SNS(r->ts_ns_last),
			r->warn);
		l += sprintf(&s[l], "%-15s: %u.%06u/%u\t%u.%06u/%u\t%u.%06u\n", "trig_time",
			SENSOR_NS2MS(r->diff_ns_min), SENSOR_NS2MSNS(r->diff_ns_min), r->count_min,
			SENSOR_NS2MS(r->diff_ns_max), SENSOR_NS2MSNS(r->diff_ns_max), r->count_max,
			SENSOR_NS2MS(avg), SENSOR_NS2MSNS(avg));
		r = &frame.trig_2a;
		avg = (r->count > 0U) ? (uint32_t)(r->diff_ns_all / r->count) : 0U;
		l += sprintf(&s[l], "%-15s: %-10u\t%llu.%09llu\t%u\n", "2a_trig",
			r->count, SENSOR_NS2S(r->ts_ns_last), SENSOR_NS2SNS(r->ts_ns_last),
			r->warn);
		l += sprintf(&s[l], "%-15s: %u.%06u/%u\t%u.%06u/%u\t%u.%06u\n", "wake_time",
			SENSOR_NS2MS(r->diff_ns_min), SENSOR_NS2MSNS(r->diff_ns_min), r->count_min,
			SENSOR_NS2MS(r->diff_ns_max), SENSOR_NS2MSNS(r->diff_ns_max), r->count_max,
			SENSOR_NS2MS(avg), SENSOR_NS2MSNS(avg));
		r = &frame.done_2a;
		avg = (r->count > 1U) ? (uint32_t)(r->diff_ns_all / (r->count - 1U)) : 0U;
		l += sprintf(&s[l], "%-15s: %-10u\t%llu.%09llu\t%u\n", "2a_work",
			r->count, SENSOR_NS2S(r->ts_ns_last), SENSOR_NS2SNS(r->ts_ns_last),
			r->warn);
		l += sprintf(&s[l], "%-15s: %u.%06u/%u\t%u.%06u/%u\t%u.%06u\n", "ctrl_time",
			SENSOR_NS2MS(r->diff_ns_min), SENSOR_NS2MSNS(r->diff_ns_min), r->count_min,
			SENSOR_NS2MS(r->diff_ns_max), SENSOR_NS2MSNS(r->diff_ns_max), r->count_max,
			SENSOR_NS2MS(avg), SENSOR_NS2MSNS(avg));
	} else {
		r = &frame.update_2a;
		l += sprintf(&s[l], "%-15s: %u\t%u\n", "2a_update", r->count, r->warn);
		r = &frame.start_2a;
		l += sprintf(&s[l], "%-15s: %u\t%u\n", "2a_start", r->count, r->warn);
		r = &frame.trig_2a;
		l += sprintf(&s[l], "%-15s: %u\t%u\n", "2a_trig", r->count, r->warn);
		r = &frame.done_2a;
		l += sprintf(&s[l], "%-15s: %u\t%u\n", "2a_work", r->count, r->warn);
	}
	ctrl_mode = (pa->ctrl_mode >= SENSOR_CTRLM_INVALID) ? SENSOR_CTRLM_AUTO : pa->ctrl_mode;
	l += sprintf(&s[l], "%-15s: %d - %s\n", "2a_ctrl", pa->ctrl_mode,
		ctrlm_names[ctrl_mode]);
	l += sprintf(&s[l], "%-15s: %d - %s\n", "2a_event", pa->ae_event_flag,
		(pa->ae_event_flag == SENSOR_FRAME_END) ? "FE" : "FS");

	return l;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device operation: get the status/fps show
 *
 * @param[in] sen: sensor device struct
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
int32_t sensor_device_status_fps_show(struct sensor_device_s *sen, char *buf)
{
	struct sensor_tuning_data_s *cam_p;
	struct sensor_link_s *link;
	struct sensor_param_s *pa;
	struct sensor_fps_s *fps;
	struct sensor_fps_record_s *r;
	char *s = buf;
	uint32_t last_index, f = 0U;
	int32_t l = 0;
	uint64_t now_s;

	if ((sen == NULL) || (s == NULL))
		return -EFAULT;
	cam_p = &sen->camera_param;
	fps = &sen->fps;
	link = &sen->link;
	pa = &sen->param;

	/* if enable? */
	if ((pa->frame_debug & SENSOR_FRAME_DEBUG_FPS_RECORD) == 0U) {
		l += sprintf(&s[l], "%-15s: %s \tfps not record\n",
			sen->mdev.name, cam_p->sensor_name);
		return l;
	}

	/* do nothing if not attached */
	if (link->attach == 0)
		return l;

	/* fps info */
	now_s = SENSOR_NS2S(osal_time_get_ns());
	r = &fps->record[fps->rec_index];
	if (now_s == r->ts_s) {
		last_index = (fps->rec_index == 0U) ?
			(SENSOR_FPS_RECORD_MAX - 1U) : (fps->rec_index - 1U);
		r = &fps->record[last_index];
		if (now_s == (r->ts_s + 1U))
			f = r->count;
	} else if (now_s == (r->ts_s + 1U)) {
		f = r->count;
	}

	l += sprintf(&s[l], "%-15s: %s \t%dfps\n",
		sen->mdev.name, cam_p->sensor_name, f);

	return l;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device operation: get the status/fps_record show
 *
 * @param[in] sen: sensor device struct
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
int32_t sensor_device_status_fps_record_show(struct sensor_device_s *sen, char *buf)
{
	struct sensor_tuning_data_s *cam_p;
	struct sensor_link_s *link;
	struct sensor_param_s *pa;
	struct sensor_fps_s *fps;
	struct sensor_fps_record_s *r;
	uint32_t s_run;
	char *s = buf;
	uint32_t i, n;
	int32_t l = 0;

	if ((sen == NULL) || (s == NULL))
		return -EFAULT;
	cam_p = &sen->camera_param;
	fps = &sen->fps;
	link = &sen->link;
	pa = &sen->param;

	/* if enable? */
	if ((pa->frame_debug & SENSOR_FRAME_DEBUG_FPS_RECORD) == 0U) {
		l += sprintf(&s[l], "%-15s: %s \tfps not record\n",
			sen->mdev.name, cam_p->sensor_name);
		return l;
	}

	/* base info show */
	s_run = SENSOR_NS2S(fps->ts_ns_last - fps->ts_ns_first);
	if (link->attach == 0) {
		l += sprintf(&s[l], "%-15s: %s \tdetached %ds\n",
			sen->mdev.name, cam_p->sensor_name, s_run);
	} else {
		l += sprintf(&s[l], "%-15s: %s \t%dfps %ds\n",
			sen->mdev.name, cam_p->sensor_name, fps->fps_target, s_run);
	}
	/* ts info show  */
	l += sprintf(&s[l], "%15s: %lld.%09lld\n",
		"ts_first", SENSOR_NS2S(fps->ts_ns_first), SENSOR_NS2SNS(fps->ts_ns_first));
	l += sprintf(&s[l], "%15s: %lld.%09lld\n",
		"ts_last", SENSOR_NS2S(fps->ts_ns_last), SENSOR_NS2SNS(fps->ts_ns_last));

	/* record info show */
	i = (fps->rec_index + 1U) % SENSOR_FPS_RECORD_MAX;
	n = 0U;
	while (n < SENSOR_FPS_RECORD_MAX) {
		r = &fps->record[i];
		if (r->ts_s != 0U) {
			l += sprintf(&s[l], "%15lld: frame %d \t%dfps\n",
				r->ts_s, r->start, r->count);
		}
		i = (i + 1U) % SENSOR_FPS_RECORD_MAX;
		n++;
	}
	/* record fps mismatch show? */
	if ((pa->frame_debug & SENSOR_FRAME_DEBUG_FPS_RECMIS) == 0U) {
		l += sprintf(&s[l], "%-15s: not record\n", "mismatch");
	} else {
		if (fps->mis_count == 0U) {
			if (fps->fps_target != 0U)
				l += sprintf(&s[l], "%-15s: all good \t%dfps\n",
					"mismatch", fps->fps_target);
		} else {
			l += sprintf(&s[l], "%-15s: %d record\n",
				"mismatch", fps->mis_count);
			i = (fps->mis_index + 1U) % SENSOR_FPS_RECMIS_MAX;
			n = 0U;
			while (n < SENSOR_FPS_RECMIS_MAX) {
				r = &fps->recmis[i];
				if (r->ts_s != 0U) {
					if ((r->count > (fps->fps_target + 1U)) ||
						(r->count < (fps->fps_target - 1U)))
						l += sprintf(&s[l], "%15lld: frame %d \t%dfps\t*\n",
							r->ts_s, r->start, r->count);
					else
						l += sprintf(&s[l], "%15lld: frame %d \t%dfps\n",
							r->ts_s, r->start, r->count);
				}
				i = (i + 1U) % SENSOR_FPS_RECMIS_MAX;
				n++;
			}
		}
	}

	return l;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor param index get for sysfs show/store
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
static int32_t sensor_param_idx(const char *name)
{
	int32_t i;

	for (i = 0; i < ARRAY_SIZE(g_sen_param_names); i++) {
		if (strcmp(g_sen_param_names[i], name) == 0) {
			return i;
		}
	}

	return -1;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device operation: get the param/xxx show
 *
 * @param[in] sen: sensor device struct
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
int32_t sensor_device_param_show(struct sensor_device_s *sen, const char *name, char *buf)
{
	int32_t *param;
	char *s = buf;
	int32_t idx, l = 0;

	if ((sen == NULL) || (name == NULL) || (s == NULL))
		return -EFAULT;

	param = (int32_t *)((void *)(&sen->param));

	idx = sensor_param_idx(name);
	if (idx >= 0)
		l += sprintf(&s[l], "%d\n", param[idx]);

	return l;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device operation: set the param/xxx store
 *
 * @param[in] sen: sensor device struct
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
int32_t sensor_device_param_store(struct sensor_device_s *sen, const char *name, const char *buf, int32_t count)
{
	uint32_t *param;
	int32_t ret = -EINVAL;
	int32_t idx;
	int32_t val;

	if ((sen == NULL) || (buf == NULL))
		return -EFAULT;

	param = (int32_t *)((void *)(&sen->param));

	idx = sensor_param_idx(name);
	if (idx >= 0) {
		ret = kstrtoint(buf, 0, &val);
		if (ret == 0)
			param[idx] = val;
	}

	return ((ret < 0) ? ret : (ssize_t)count);
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor operation (in mutex lock)
 *
 * @param[in] sen: sensor device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t sensor_evk_do(struct sensor_device_s *sen, sensor_event_info_t *ev_info)
{
	struct sensor_user_s *user = &sen->user;
	struct sensor_param_s *pa = &sen->param;
	struct os_dev * dev = &sen->osdev;
	int32_t ret, ev_state, toout = 0, retry = 0, finish = 0;
	int32_t ev_timeout_ms = pa->ev_timeout_ms;
	int32_t nowait = (ev_timeout_ms == 0) ? 1 : 0;

	ev_state = user->ev_state;
	if ((ev_state == SEN_EV_STATE_WAIT) || (ev_state == SEN_EV_STATE_FINISH)) {
		/* if wait is empty */
		if (nowait == 0) {
			user->ev_info = ev_info;
			user->ev_state = SEN_EV_STATE_DOING;
		} else {
			memcpy(&user->ev_nowait, ev_info, sizeof(sensor_event_info_t));
			user->ev_info = &user->ev_nowait;
			user->ev_state = SEN_EV_STATE_DOING_NOWAIT;
		}
		sen_info(dev, "%s: %s to %s\n", __func__,
			g_sen_ev_state_names[ev_state],
			g_sen_ev_state_names[user->ev_state]);
		osal_wake_up(&user->evu_wq);
		if (nowait != 0)
			return 0;
	}
	user->ev_wait++;
	osal_mutex_unlock(&user->mutex);

	while (finish == 0) {
		if (ev_timeout_ms == -1) {
			/* wait forever */
			ret = osal_wait_event_interruptible(user->evk_wq,
				((user->ev_state == SEN_EV_STATE_CANCEL) ||
				 (user->ev_state == SEN_EV_STATE_WAIT) ||
				 ((user->ev_state > SEN_EV_STATE_DOING) && (user->ev_info == ev_info))));
		} else {
			/* wait timeout */
			toout = 1;
			ret = osal_wait_event_interruptible_timeout(user->evk_wq,
				((user->ev_state == SEN_EV_STATE_CANCEL) ||
				 (user->ev_state == SEN_EV_STATE_WAIT) ||
				 ((user->ev_state > SEN_EV_STATE_DOING) && (user->ev_info == ev_info))),
				ev_timeout_ms);
		}
		osal_mutex_lock(&user->mutex);
		/* if interrupt by signal? */
		if (ret < 0) {
			user->ev_wait--;
			sen_err(dev, "%s: %s wait evk failed %d\n", __func__,
				g_sen_ev_state_names[user->ev_state], ret);
			osal_mutex_unlock(&user->mutex);
			break;
		}
		/* if wait timeout? */
		if ((toout != 0) && (ret == 0)) {
			/* timeout */
			if (retry >= pa->ev_retry_max) {
				user->ev_wait--;
				ret = -ETIME;
				sen_err(dev, "%s: %s %dms timeout failed %d\n", __func__,
					g_sen_ev_state_names[user->ev_state], ev_timeout_ms, ret);
				osal_mutex_unlock(&user->mutex);
				break;
			} else {
				retry++;
				sen_warn(dev, "%s: %s %dms timeout retry %d\n", __func__,
					g_sen_ev_state_names[user->ev_state], ev_timeout_ms, retry);
				osal_mutex_unlock(&user->mutex);
				continue;
			}
		}
		/* wake up */
		if (user->ev_state == SEN_EV_STATE_CANCEL) {
			/* cancel */
			user->ev_wait--;
			ret = -ESRCH;
			finish = 1;
		} else if ((user->ev_state > SEN_EV_STATE_DOING) && (user->ev_info == ev_info)) {
			/* this op finish */
			sen_info(dev, "%s: %s to %s\n", __func__,
				g_sen_ev_state_names[user->ev_state],
				g_sen_ev_state_names[SEN_EV_STATE_FINISH]);
			user->ev_wait--;
			ret = (user->ev_state == SEN_EV_STATE_DONE) ? 0 : -EFAULT;
			user->ev_state = SEN_EV_STATE_FINISH;
			finish = 1;
		} else if (user->ev_state == SEN_EV_STATE_WAIT) {
			if (nowait == 0) {
				user->ev_info = ev_info;
				user->ev_state = SEN_EV_STATE_DOING;
			} else {
				memcpy(&user->ev_nowait, ev_info, sizeof(sensor_event_info_t));
				user->ev_info = &user->ev_nowait;
				user->ev_state = SEN_EV_STATE_DOING_NOWAIT;
			}
			sen_info(dev, "%s: %s to %s\n", __func__,
				g_sen_ev_state_names[SEN_EV_STATE_WAIT],
				g_sen_ev_state_names[user->ev_state]);
			/* this op ready to run */
			osal_wake_up(&user->evu_wq);
			if (nowait != 0) {
				user->ev_wait--;
				ret = 0;
				finish = 1;
			}
		}
		osal_mutex_unlock(&user->mutex);
	}

	osal_mutex_lock(&user->mutex);
	if (user->ev_wait > 0U) {
		/* someone waiting to op */
		osal_wake_up(&user->evk_wq);
	}

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor streaming operation
 *
 * @param[in] sen: sensor device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t sensor_streaming_do(struct sensor_device_s *sen, int32_t state)
{
	int32_t ret;

	struct sensor_user_s *user = &sen->user;
	sensor_event_info_t ev_info = {
		.type = SEN_EVENT_TYPE_STREAM,
		.data = state,
	};

	/* wait fe before stop to streaming off */
	if (state == 0) {
		ret = sensor_frame_end_stop_wait(sen);
		if (ret < 0) {
			ret = sensor_stream_reg_do(sen, state);
			return ret;
		}
	}

	/* do streaming on/off */
	if (user->ev_state != SEN_EV_STATE_DEFAULT) {
		ret = sensor_evk_do(sen, &ev_info);
	} else {
		ret = sensor_stream_reg_do(sen, state);
	}

	return ret;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief vcon sub driver sensor operation: attach
 *
 * @param[in] index: camera sensor port index
 * @param[in] flow_id: vpf flow id
 * @param[in] add_id: vcon contex index
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
static int32_t sensor_attach(int32_t index, int32_t flow_id, int32_t add_id)
{
	struct sensor_device_s *sen;
	struct sensor_user_s *user;
	struct sensor_link_s *link;
	struct os_dev * dev;

	sen = sensor_dev_get(index);
	if (sen == NULL)
		return -ENODEV;
	if (flow_id >= SENSOR_NUM_MAX)
		return -EINVAL;
	user = &sen->user;
	link = &sen->link;
	dev = &sen->osdev;

	osal_mutex_lock(&user->mutex);
	if (link->attach != 0) {
		sen_err(dev, "%s error %s has attached by flow%d",
			__func__, sen->camera_param.sensor_name, link->flow_id);
		osal_mutex_unlock(&user->mutex);
		return -EBUSY;
	}
	link->flow_id = flow_id;
	link->ctx_id = add_id;
	link->attach = 1;
	g_sen.sen_flow[flow_id] = sen;
	sen_info(dev, "%s flow%d ctx%d to sensor%d %s\n",
		__func__, flow_id, add_id, sen->port, sen->camera_param.sensor_name);
	osal_mutex_unlock(&user->mutex);

	return 0;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief vcon sub driver sensor operation: detach
 *
 * @param[in] index: camera sensor port index
 * @param[in] flow_id: vpf flow id
 * @param[in] add_id: vcon contex index
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
static int32_t sensor_detach(int32_t index, int32_t flow_id, int32_t add_id)
{
	struct sensor_device_s *sen;
	struct sensor_user_s *user;
	struct sensor_link_s *link;
	struct os_dev * dev;

	sen = sensor_dev_get(index);
	if (sen == NULL)
		return -ENODEV;
	if (flow_id >= SENSOR_NUM_MAX)
		return -EINVAL;
	user = &sen->user;
	link = &sen->link;
	dev = &sen->osdev;

	osal_mutex_lock(&user->mutex);
	if (link->attach != 0) {
		if ((link->flow_id != flow_id) || (link->ctx_id != add_id)) {
			sen_info(dev, "%s flow%d ctx%d warn not match attached flow%d ctx%d\n",
				__func__, flow_id, add_id, link->flow_id, link->ctx_id);
		} else {
			sen_info(dev, "%s flow%d link%d from sensor%d %s\n",
				__func__, flow_id, add_id, sen->port, sen->camera_param.sensor_name);
		}
		link->flow_id = VCON_FLOW_INVALID;
		link->ctx_id = VCON_FLOW_INVALID;
		link->attach = 0;
		g_sen.sen_flow[flow_id] = NULL;
	}
	osal_mutex_unlock(&user->mutex);

	return 0;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief vcon sub driver sensor operation: start
 *
 * @param[in] index: camera sensor port index
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
static int32_t sensor_start(int32_t index)
{
	struct sensor_device_s *sen;
	struct sensor_user_s *user;
	struct sensor_link_s *link;
	struct os_dev * dev;
	int32_t ret;

	sen = sensor_dev_get(index);
	if (sen == NULL)
		return -ENODEV;

	user = &sen->user;
	link = &sen->link;
	dev = &sen->osdev;

	osal_mutex_lock(&user->mutex);
	if (link->attach == 0) {
		sen_err(dev, "%s %s error not attached",
			__func__, sen->camera_param.sensor_name);
		osal_mutex_unlock(&user->mutex);
		return -EACCES;
	}
	sen_debug(dev, "%s %s flow%d starting ...\n",
		__func__, sen->camera_param.sensor_name, link->flow_id);
	ret = sensor_streaming_do(sen, 1);
	if (ret < 0) {
		sen_debug(dev, "%s %s flow%d start error %d\n",
			__func__, sen->camera_param.sensor_name, link->flow_id, ret);
	} else {
		sen_info(dev, "%s %s flow%d start done\n",
			__func__, sen->camera_param.sensor_name, link->flow_id);
	}
	user->start_cnt++;
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief vcon sub driver sensor operation: stop
 *
 * @param[in] index: camera sensor port index
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
static int32_t sensor_stop(int32_t index)
{
	struct sensor_device_s *sen;
	struct sensor_user_s *user;
	struct sensor_link_s *link;
	struct os_dev * dev;
	int32_t ret = 0;

	sen = sensor_dev_get(index);
	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	link = &sen->link;
	dev = &sen->osdev;

	osal_mutex_lock(&user->mutex);
	if (user->start_cnt == 0) {
		sen_info(dev, "%s %s has stoped \n",
			__func__, sen->camera_param.sensor_name);
		osal_mutex_unlock(&user->mutex);
		return ret;
	}
	if (link->attach == 0) {
		sen_err(dev, "%s %s error not attached",
			__func__, sen->camera_param.sensor_name);
		osal_mutex_unlock(&user->mutex);
		return -EACCES;
	}
	sen_debug(dev, "%s %s flow%d stopping ...\n",
		__func__, sen->camera_param.sensor_name, link->flow_id);
	ret = sensor_streaming_do(sen, 0);
	if (ret < 0) {
		sen_debug(dev, "%s %s flow%d stop error %d\n",
			__func__, sen->camera_param.sensor_name, link->flow_id, ret);
	} else {
		sen_info(dev, "%s %s flow%d stop done\n",
			__func__, sen->camera_param.sensor_name, link->flow_id);
	}
	if (user->start_cnt > 0)
		user->start_cnt--;
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief vcon sub driver sensor operation: event(fs/fe,etc.)
 *
 * @param[in] index: camera sensor port index
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
static int32_t sensor_event(int32_t index, int32_t event_id, void *event_data)
{
	struct sensor_device_s *sen;
	struct sensor_user_s *user;
	struct os_dev * dev;
	int32_t ret = 0;

	sen = sensor_dev_get(index);
	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	osal_mutex_lock(&user->mutex);
	sen_debug(dev, "%s %s event %d\n",
		__func__, sen->camera_param.sensor_name, event_id);
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief vcon sub driver sensor operation: set vcon callback function
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
static void sensor_setcb(vcon_cb cb)
{
	g_sen.vcon_event_cb = cb;
	// sen_info(NULL, "%s done\n", __func__);
}

/**
 * sensor driver ops for vcon/isp/cim
 */
static struct vcon_sub_ops_s sensor_vcon_ops = {
	.attach = sensor_attach,
	.detach = sensor_detach,
	.start = sensor_start,
	.stop = sensor_stop,
	.event = sensor_event,
	.setcb = sensor_setcb,
	.end_magic = VCON_SUB_OPS_END_MAGIC(VCON_COPT_SENSOR),
};

DECLARE_VIO_CALLBACK_OPS(sensor_vcon_cops, SENSOR_COPS_MAGIC, &sensor_vcon_ops);
DECLARE_VIO_CALLBACK_OPS(sensor_isp_cops, SENSOR_COPS_MAGIC, &sensor_isp_ops);
DECLARE_VIO_CALLBACK_OPS(sensor_cim_cops, SENSOR_COPS_MAGIC, &sensor_cim_ops);
DECLARE_VIO_CALLBACK_OPS(sensor_isi_cops, SENSOR_COPS_MAGIC, &sensor_isi_ops);

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor cops init for other drivers
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
static int32_t sensor_cops_init(void)
{
	int32_t ret;

	ret = vio_register_callback_ops(&cb_sensor_vcon_cops, SENSOR_COP_MODULE, SENSOR_VCON_COP_TYPE);
	if (ret < 0) {
		sen_err(NULL, "sensor vcon cops register failed %d\n", ret);
		return ret;
	}
	ret = vio_register_callback_ops(&cb_sensor_isp_cops, SENSOR_COP_MODULE, SENSOR_ISP_COP_TYPE);
	if (ret < 0) {
		sen_err(NULL, "sensor isp cops register failed %d\n", ret);
		vio_unregister_callback_ops(SENSOR_COP_MODULE, SENSOR_VCON_COP_TYPE);
		return ret;
	}
	ret = vio_register_callback_ops(&cb_sensor_cim_cops, SENSOR_COP_MODULE, SENSOR_CIM_COP_TYPE);
	if (ret < 0) {
		sen_err(NULL, "sensor cim cops register failed %d\n", ret);
		vio_unregister_callback_ops(SENSOR_COP_MODULE, SENSOR_ISP_COP_TYPE);
		vio_unregister_callback_ops(SENSOR_COP_MODULE, SENSOR_VCON_COP_TYPE);
		return ret;
	}
        ret = vio_register_callback_ops(&cb_sensor_isi_cops, SENSOR_COP_MODULE, SENSOR_ISI_COP_TYPE);
        if (ret < 0) {
                sen_err(NULL, "sensor isi cops register failed %d\n", ret);
                vio_unregister_callback_ops(SENSOR_COP_MODULE, SENSOR_CIM_COP_TYPE);
                vio_unregister_callback_ops(SENSOR_COP_MODULE, SENSOR_ISP_COP_TYPE);
                vio_unregister_callback_ops(SENSOR_COP_MODULE, SENSOR_VCON_COP_TYPE);
                return ret;
	}

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor cops exit unregister
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void sensor_cops_exit(void)
{
	vio_unregister_callback_ops(SENSOR_COP_MODULE, SENSOR_CIM_COP_TYPE);
	vio_unregister_callback_ops(SENSOR_COP_MODULE, SENSOR_ISP_COP_TYPE);
	vio_unregister_callback_ops(SENSOR_COP_MODULE, SENSOR_VCON_COP_TYPE);
    vio_unregister_callback_ops(SENSOR_COP_MODULE, SENSOR_ISI_COP_TYPE);
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device init for probe
 *
 * @param[in] sen: sensor device struct
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
int32_t sensor_device_init(struct sensor_device_s *sen)
{
	if (sen == NULL)
		return -EINVAL;

	osal_mutex_init(&sen->user.open_mutex);
	osal_mutex_init(&sen->mdev.bus_mutex);
	osal_waitqueue_init(&sen->user.evk_wq);
	camera_dev_param_init(sen);

	sen->link.flow_id = VCON_FLOW_INVALID;

	return 0;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device exit
 *
 * @param[in] sen: sensor device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void sensor_device_exit(struct sensor_device_s *sen)
{
	/* do nothing */
	return;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor driver init
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
int32_t sensor_driver_init(void)
{
	int32_t ret;

	ret = camera_subdev_init();
	if (ret < 0) {
		sen_err(NULL, "sensor_dev_init register isp ops failed\n");
		return ret;
	}
	ret = camera_ctrldev_init();
	if (ret < 0) {
		sen_err(NULL, "sensor_dev_init ctrldev init failed\n");
		goto init_error_subexit;
	}

	ret = sensor_cops_init();
	if (ret < 0) {
		sen_err(NULL, "sensor cops register failed %d\n", ret);
		goto init_error_ctrlexit;
	}
	return ret;

init_error_ctrlexit:
	camera_ctrldev_exit();
init_error_subexit:
	camera_subdev_exit();
	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor driver exit
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void sensor_driver_exit(void)
{
	sensor_cops_exit();
	camera_ctrldev_exit();
	camera_subdev_exit();
}
