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
 * @file camera_subdev.c
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#include "inc/camera_dev.h"
#include "inc/camera_subdev.h"
#include "inc/camera_i2c.h"
#include "inc/camera_sys_api.h"
#include "inc/camera_ctrl.h"
#include "hobot_sensor_ops.h"

#define ARGS_TO_PTR(arg) ((struct sensor_arg *)arg)
sensor_priv_t sensor_ctl[FIRMWARE_CONTEXT_NUMBER];
sensor_data_t sensor_param[FIRMWARE_CONTEXT_NUMBER];
sensor_tuning_data_t tuning_param[FIRMWARE_CONTEXT_NUMBER];

// used for ae ctrl.
static sensor_event_header_t sensor_event_header[CAMERA_TOTAL_NUMBER];
static sensor_event_node_t *sensor_event_arr = NULL;
static uint32_t sensor_update_flag = 0u;
module_param(sensor_update_flag, uint, 0644); /* PRQA S 0605,0636,4501 */ /* module_param macro */

static int32_t common_alloc_analog_gain(uint32_t chn, int32_t *gain_ptr)
{
	int32_t ret = 0;
	struct os_dev* dev = sensor_osdev_get(chn);

	if ((chn < FIRMWARE_CONTEXT_NUMBER) && (gain_ptr != NULL)) {
		// Initial local parameters
		//ret = camera_sys_alloc_dgain(chn, gain_ptr);
	} else {
		sen_err(dev, "common subdev pointer is NULL");
		ret = -1;
	}
	return ret;
}

static int32_t common_alloc_digital_gain(uint32_t chn, int32_t *gain_ptr)
{
	int32_t ret = 0;
	struct os_dev* dev = sensor_osdev_get(chn);

	if ((chn < FIRMWARE_CONTEXT_NUMBER) && (gain_ptr != NULL)) {
		// Initial local parameters
		//ret = camera_sys_alloc_again(chn, gain_ptr);
	} else {
		sen_err(dev, "common subdev pointer is NULL");
		ret = -1;
	}
	return ret;
}

static int32_t common_alloc_integration_time(uint32_t chn, uint32_t *line_ptr)
{
	int32_t ret = 0;
	struct os_dev* dev = sensor_osdev_get(chn);

	if (chn < FIRMWARE_CONTEXT_NUMBER) {

		// Initial local parameters
		//camera_sys_alloc_intergration_time

		switch (sensor_ctl[chn].mode) {
		case (u8)SENSOR_LINEAR:
		case (u8)SENSOR_PWL:
			sensor_ctl[chn].line_num = 1;
		break;
		case (u8)SENSOR_DOL2:
			sensor_ctl[chn].line_num = 2;
		break;
		case (u8)SENSOR_DOL3:
			sensor_ctl[chn].line_num = 3;
		break;
		case (u8)SENSOR_DOL4:
			sen_err(dev, "common subdev pointer is NULL");
		break;
		default:
			sen_err(dev, "sensor mode is error");
		break;
		}
	} else {
		sen_err(dev, "common subdev pointer is NULL");
		ret = -1;
	}

	return ret;
}

static int32_t isi_alloc_analog_gain(uint32_t chn, int32_t *gain_ptr, uint32_t gain_num)
{
	int32_t ret = 0;
	struct os_dev* dev = sensor_osdev_get(chn);

	if ((chn < FIRMWARE_CONTEXT_NUMBER) && (gain_ptr != NULL)) {
		// Initial local parameters
		//ret = camera_sys_alloc_dgain(chn, gain_ptr);

		if (gain_num > 2u)
			sensor_ctl[chn].gain_buf[2] = (uint32_t)gain_ptr[2];
		if (gain_num > 1u)
			sensor_ctl[chn].gain_buf[1] = (uint32_t)gain_ptr[1];
		if (gain_num > 0u)
			sensor_ctl[chn].gain_buf[0] = (uint32_t)gain_ptr[0];
		sensor_ctl[chn].dgain_num = gain_num;
	} else {
		sen_err(dev, "common subdev pointer is NULL");
		ret = -1;
	}
	return ret;
}

static int32_t isi_alloc_digital_gain(uint32_t chn, int32_t *gain_ptr, uint32_t gain_num)
{
	int32_t ret = 0;
	struct os_dev* dev = sensor_osdev_get(chn);

	if ((chn < FIRMWARE_CONTEXT_NUMBER) && (gain_ptr != NULL)) {
		// Initial local parameters
		//ret = camera_sys_alloc_again(chn, gain_ptr);

		if (gain_num > 2u)
			sensor_ctl[chn].dgain_buf[2] = (uint32_t)gain_ptr[2];
		if (gain_num > 1u)
			sensor_ctl[chn].dgain_buf[1] = (uint32_t)gain_ptr[1];
		if (gain_num > 0u)
			sensor_ctl[chn].dgain_buf[0] = (uint32_t)gain_ptr[0];

		sensor_ctl[chn].dgain_num = gain_num;
	} else {
		sen_err(dev, "common subdev pointer is NULL");
		ret = -1;
	}
	return ret;
}

static int32_t isi_alloc_integration_time(uint32_t chn, uint32_t *int_time,
	uint32_t *int_time_M, uint32_t *int_time_L)
{
	int32_t ret = 0;
	uint32_t time_L = *int_time;
	uint32_t time_M = *int_time_M;
	uint32_t time_S = *int_time_L;
	struct os_dev* dev = sensor_osdev_get(chn);

	if (chn < FIRMWARE_CONTEXT_NUMBER) {

		// Initial local parameters
		//camera_sys_alloc_intergration_time

		switch (sensor_ctl[chn].mode) {
		case (u8)SENSOR_LINEAR:
		case (u8)SENSOR_PWL:
			sensor_ctl[chn].line_buf[0] = time_L;
			sensor_ctl[chn].line_num = 1;
			sen_debug(dev, "linear time is %d", time_L);/*PRQA S 0685,1294*/
		break;
		case (u8)SENSOR_DOL2:
			sensor_ctl[chn].line_buf[0] = time_L;
			sensor_ctl[chn].line_buf[1] = time_S;
			sensor_ctl[chn].line_num = 2;
		break;
		case (u8)SENSOR_DOL3:
			sensor_ctl[chn].line_buf[0] = time_L;
			sensor_ctl[chn].line_buf[1] = time_M;
			sensor_ctl[chn].line_buf[2] = time_S;
			sensor_ctl[chn].line_num = 3;
		break;
		case (u8)SENSOR_DOL4:
			sen_err(dev, "common subdev pointer is NULL");
		break;
		default:
			sen_err(dev, "sensor mode is error");
		break;
		}
	} else {
		sen_err(dev, "common subdev pointer is NULL");
		ret = -1;
	}

	*int_time = (uint16_t)(time_L);
	*int_time_M = (uint16_t)(time_M);
	*int_time_L =(uint16_t)(time_S);
	return ret;
}

// add list
static void cmd_add_to_work(uint8_t chn, uint32_t cmd, void *arg)
{
	osal_list_head_t *list;
	sensor_event_node_t *event_p;
	uint32_t pipeline = (uint32_t)chn;
	struct os_dev* dev = sensor_osdev_get(chn);

        osal_spin_lock(&sensor_event_header[pipeline].lock);
        if (!osal_list_empty(&sensor_event_header[pipeline].list_free)) {
                list = sensor_event_header[pipeline].list_free.next;
                osal_list_del(list);
                event_p = osal_list_entry(list, sensor_event_node_t, list_node);/*PRQA S 2810,0497*/
                event_p->port = chn;
                event_p->cmd = cmd;
                memcpy(&event_p->priv_param, arg, sizeof(sensor_priv_t));
                osal_list_add(list, &sensor_event_header[pipeline].list_busy);
        } else {
		sen_debug(dev, "%s chn %d buf is null\n", __func__, pipeline);/*PRQA S 0685,1294*/
	}
        osal_spin_unlock(&sensor_event_header[pipeline].lock);
}

// wake up from cim or cim_dma
void wake_up_ae_update(uint32_t pipeline)
{
	uint32_t triger_status = 0;
	struct os_dev* dev = sensor_osdev_get(pipeline);

	if (pipeline >= CAMERA_TOTAL_NUMBER)
		return;
	if (sensor_update_flag & ((uint32_t)1u << pipeline)) {
		sen_debug(dev, "%s chn %d ignore triger\n", __func__, pipeline);
	} else {
		sen_debug(dev, "%s chn %d triger\n", __func__, pipeline);
		triger_status = (uint32_t)schedule_work(&sensor_event_header[pipeline].updata_work);
		sensor_ctrl_wakeup(pipeline);
	}
}

// release work buf
void wake_up_release_work(uint32_t pipeline)
{
	osal_list_head_t *list;
	sensor_event_node_t *event_p;
	uint32_t count = 0;
	struct os_dev* dev = sensor_osdev_get(pipeline);

	if (pipeline >= CAMERA_TOTAL_NUMBER)
		return;
	osal_spin_lock(&sensor_event_header[pipeline].lock);
	do {
		if (!osal_list_empty(&sensor_event_header[pipeline].list_busy)) {
		        list = sensor_event_header[pipeline].list_busy.next;
		        osal_list_del(list);
		        event_p = osal_list_entry(list, sensor_event_node_t, list_node);/*PRQA S 2810,0497*/
			event_p->cmd = SENSOR_INVALID;
		        osal_list_add(list, &sensor_event_header[pipeline].list_free);
		} else {
			break;
		}
		count++;
	} while (count < HARDWARE_BUF_NUMBER);
	osal_spin_unlock(&sensor_event_header[pipeline].lock);
	sen_debug(dev, "%s chn %d is release success!\n", __func__, pipeline);/*PRQA S 0685,1294*/
}

// remove from list
static void write_isp_work(struct work_struct *data)
{
        int ret = 0;
	osal_list_head_t *list;
	sensor_event_node_t *event_p;
	sensor_event_header_t *sensor_event_temp;
	const uint32_t try_write_num = 3;
	uint32_t count = 0;
	uint32_t ae_write_flag = SENSOR_WRITE_ENABLE;
	uint32_t awb_write_flag = SENSOR_WRITE_ENABLE;
	struct os_dev* dev;

	sensor_event_temp = container_of(data, sensor_event_header_t, updata_work); /*PRQA S 2810,0497*/

	do {
		list = NULL;
		osal_spin_lock(&sensor_event_temp->lock);
		if (!osal_list_empty(&sensor_event_temp->list_busy)) {
		        list = sensor_event_temp->list_busy.next;
		        osal_list_del(list);
		}
		osal_spin_unlock(&sensor_event_temp->lock);

		if (list) {
		        event_p = osal_list_entry(list, sensor_event_node_t, list_node);/*PRQA S 2810,0497*/
			dev = sensor_osdev_get(event_p->port);
			if ((event_p->cmd == (uint32_t)SENSOR_UPDATE) && (ae_write_flag)) {
				sen_debug(dev, "%s chn %d write sensor ae param start\n", __func__, event_p->port);/*PRQA S 0685,1294*/
				ret = camera_sys_priv_set(event_p->port, &event_p->priv_param);
				if(ret < 0)
					sen_warn(dev, "SENSOR_AE_UPDATE drop port%d\n", event_p->port);
				sen_debug(dev, "%s chn %d write sensor ae param end\n", __func__, event_p->port);/*PRQA S 0685,1294*/
				ae_write_flag = SENSOR_WRITE_DISABLE;
			} else if ((event_p->cmd == (uint32_t)SENSOR_AWB_UPDATE) && (awb_write_flag)) {
				sen_debug(dev, "%s chn %d write sensor awb param start\n", __func__, event_p->port);/*PRQA S 0685,1294*/
				ret = camera_sys_priv_awb_set(event_p->port, &event_p->priv_param);
				if(ret < 0)
					sen_warn(dev, "SENSOR_AWB_UPDATE drop port%d\n", event_p->port);
				sen_debug(dev, "%s chn %d write sensor awb param end\n", __func__, event_p->port);/*PRQA S 0685,1294*/
				awb_write_flag = SENSOR_WRITE_DISABLE;
			} else {
				sen_debug(dev, "chn %d do not write cmd %d\n", event_p->port, event_p->cmd);/*PRQA S 0685,1294*/
			}
			event_p->cmd = SENSOR_INVALID;
			osal_spin_lock(&sensor_event_temp->lock);
			osal_list_add(list, &sensor_event_temp->list_free);
			osal_spin_unlock(&sensor_event_temp->lock);
		}
		count++;
	} while (count < try_write_num);
}

static int32_t common_update(uint32_t chn, struct sensor_priv_old *updata, int32_t effect)
{
	int32_t ret = 0;
	struct sensor_device_s *sen = sensor_dev_get(chn);
	struct os_dev* dev = sensor_osdev_get(chn);

	if (chn >= FIRMWARE_CONTEXT_NUMBER) {
		sen_err(dev, "%s chn %d beyond 16\n", __func__, chn);
		return -1;
	}

	if (sensor_param[chn].lines_per_second == 0u)
		return -1;
	sensor_frame_2a_record(sen, SENSOR_F2AS_UPDATE);

	// get current fs count as id.
	sensor_ctl[chn].id = sensor_frame_count_get(sen, SENSOR_FTYPE_FS);
	// fill ae param
	memcpy(sensor_ctl[chn].gain_buf, updata->gain_buf, sizeof(sensor_ctl[chn].gain_buf));
	memcpy(sensor_ctl[chn].dgain_buf, updata->dgain_buf, sizeof(sensor_ctl[chn].dgain_buf));
	memcpy(sensor_ctl[chn].line_buf, updata->line_buf, sizeof(sensor_ctl[chn].line_buf));

	if (sensor_ctrl_mode_get(sen) == SENSOR_CTRLM_USER) {
		// update hal ctrl info & wake_up
		set_sensor_aexp_info(chn, &sensor_ctl[chn]);
		sensor_ctrl_wakeup_flag(chn);
	} else {
		// Initial local parameters
		cmd_add_to_work(chn, (uint32_t)SENSOR_UPDATE, &sensor_ctl[chn]);
	}

	if (effect != 0) {
		sen_debug(dev, "%s chn %d update ae param and effect\n", __func__, chn);/*PRQA S 0685,1294*/
		wake_up_ae_update(chn);
	} else {
		sen_debug(dev, "%s chn %d update ae param\n", __func__, chn);/*PRQA S 0685,1294*/
	}
	return ret;
}

static int32_t isi_update(uint32_t chn, int32_t effect)
{
	int32_t ret = 0;
	struct os_dev* dev = sensor_osdev_get(chn);

	if (chn >= FIRMWARE_CONTEXT_NUMBER) {
		sen_err(dev, "%s chn %d beyond 16\n", __func__, chn);
		return -1;
	}

	if (sensor_param[chn].lines_per_second == 0u)
		return -1;

	// update hal ctrl info & wake_up
	set_sensor_aexp_info(chn, &sensor_ctl[chn]);
	sensor_ctrl_wakeup_flag(chn);
	// Initial local parameters
	cmd_add_to_work(chn, (uint32_t)SENSOR_UPDATE, &sensor_ctl[chn]);

	if (effect != 0) {
		sen_debug(dev, "%s chn %d update ae param and effect\n", __func__, chn);/*PRQA S 0685,1294*/
		wake_up_ae_update(chn);
	} else {
		sen_debug(dev, "%s chn %d update ae param\n", __func__, chn);/*PRQA S 0685,1294*/
	}
	return ret;
}

static void get_common_info(uint8_t chn)
{
	struct os_dev* dev = sensor_osdev_get(chn);

	if (chn < FIRMWARE_CONTEXT_NUMBER) {
		// Initial local parameters
		camera_sys_get_param(chn, &sensor_param[chn]);

	} else {
		sen_err(dev, "common subdev pointer is NULL");
	}
}

static void get_tuning_info(uint8_t chn)
{
	if (chn < FIRMWARE_CONTEXT_NUMBER) {
		// Initial local parameters
		camera_sys_get_tuning_param(chn, &tuning_param[chn]);

	} else {
		pr_err("common subdev pointer is NULL");
	}
}

int32_t common_init(uint8_t chn, uint8_t mode)
{
	int32_t ret = 0;

	//todo sensor tyep info
	sensor_ctl[chn].mode = mode;
	sensor_ctl[chn].gain_num = 1;
	sensor_ctl[chn].dgain_num = 1;
	sensor_ctl[chn].line_num = 1;
	// update hal ctrl info & wake_up
	get_common_info(chn);
	get_tuning_info(chn);

	return ret;
}

void common_exit(uint8_t chn)
{
	if (chn < FIRMWARE_CONTEXT_NUMBER) {
		memset(&sensor_param[chn], 0, sizeof(sensor_data_t));
	}
}

static uint32_t common_read_register(uint32_t chn, uint32_t address)
{
	uint32_t value = 0;
	struct os_dev* dev = sensor_osdev_get(chn);

	if (chn < FIRMWARE_CONTEXT_NUMBER) {
		// Initial local parameters
		camera_sys_sensor_read(chn, address, &value);

	} else {
		sen_err(dev, "common subdev pointer is NULL");
	}

	return value;
}

static int32_t common_write_register(uint32_t chn, uint32_t address, uint32_t data)
{
	int32_t ret = 0;
	struct os_dev* dev = sensor_osdev_get(chn);

	if (chn < FIRMWARE_CONTEXT_NUMBER) {
		// Initial local parameters
		ret = camera_sys_sensor_write(chn, address, data);
	} else {
		sen_err(dev, "common subdev pointer is NULL");
		ret = -1;
	}
	return ret;
}

int32_t common_get_param(uint32_t chn, struct _setting_param_t *user_para)
{
	struct os_dev* dev = sensor_osdev_get(chn);

	if (chn >= FIRMWARE_CONTEXT_NUMBER) {
		sen_err(dev, "%s chn %d beyond 8\n", __func__, chn);
		return -1;
	}

	user_para->lines_per_second = sensor_param[chn].lines_per_second;
	user_para->analog_gain_max = sensor_param[chn].analog_gain_max;
	user_para->digital_gain_max = sensor_param[chn].digital_gain_max;
	user_para->exposure_time_max = sensor_param[chn].exposure_time_max;
	user_para->exposure_time_min = sensor_param[chn].exposure_time_min;
	user_para->exposure_time_long_max = sensor_param[chn].exposure_time_long_max;
	user_para->active_width = (uint16_t)sensor_param[chn].active_width;
	user_para->active_height = (uint16_t)sensor_param[chn].active_height;
	user_para->fps = sensor_param[chn].fps;
	user_para->data_width = (uint8_t)sensor_param[chn].data_width;
	user_para->bayer_start = (bayer_start_u)sensor_param[chn].bayer_start;/*PRQA S 4342*/
	user_para->bayer_pattern = (bayer_pattern_e)sensor_param[chn].bayer_pattern;/*PRQA S 4342*/
	user_para->exposure_max_bit_width = (uint8_t)sensor_param[chn].exposure_max_bit_width;

	sen_debug(dev, "param [%d] l:%d g:%d/%d e:%d/%d %dx%d %dfps b:%d o:%d p:%d m:%d\n",/*PRQA S 0685,1294*/
		chn, user_para->lines_per_second,
		user_para->analog_gain_max, user_para->digital_gain_max,
		user_para->exposure_time_max, user_para->exposure_time_min,
		user_para->active_width, user_para->active_height, user_para->fps,
		user_para->data_width, user_para->bayer_start.rggb, user_para->bayer_pattern,
		user_para->exposure_max_bit_width);
	return 0;
}

static int32_t common_awb_param(uint32_t chn, uint32_t rgain, uint32_t bgain,
				uint32_t grgain, uint32_t gbgain, uint32_t temper)
{
	int32_t ret = 0;
	struct os_dev* dev = sensor_osdev_get(chn);

	if (chn < FIRMWARE_CONTEXT_NUMBER) {
		sensor_ctl[chn].rgain = rgain;
		sensor_ctl[chn].bgain = bgain;
		sensor_ctl[chn].grgain = grgain;
		sensor_ctl[chn].gbgain = gbgain;
		sensor_ctl[chn].temper = temper;

		// Initial local parameters
		cmd_add_to_work(chn, (uint32_t)SENSOR_AWB_UPDATE, &sensor_ctl[chn]);
		sen_debug(dev, "%s chn %d update awb param\n", __func__, chn);/*PRQA S 0685,1294*/
	} else {
		sen_err(dev, "common subdev pointer is NULL");
		ret = -1;
	}
	return ret;
}

static int32_t camera_set_led(uint32_t chn, uint32_t brightness, struct sensor_led_s *cam_led)
{
	int32_t ret = 0;
	uint32_t address;
	uint32_t type, value;
	struct os_dev* dev = sensor_osdev_get(chn);

	address = cam_led->reg_addr;
	type = cam_led->type;

	if ((type <= INVALID) || (type >= LED_TYPE_MAX)) {
		sen_err(dev, "%s, port %d invalid type %d\n", __func__, chn, type);
		return -1;
	}

	switch(type) {
		case ON_OFF:
			if (brightness <= 49) {
				ret = camera_sys_sensor_write(chn, address, cam_led->neg_value);
			} else {
				ret = camera_sys_sensor_write(chn, address, cam_led->pos_value);
			}
			if (ret < 0) {
				sen_err(dev, "%s, port %d type %d write led reg failed!\n", __func__, chn, type);
			}
		break;
		case LINEAR:
			value = (cam_led->pos_value - cam_led->neg_value) * brightness / 100;
			ret = camera_sys_sensor_write(chn, address, value);
			if (ret < 0) {
				sen_err(dev, "%s, port %d type %d write led reg failed!\n", __func__, chn, type);
			}
		break;
		case LUT:
			value = cam_led->led_lut[2*brightness + 1];
			ret = camera_sys_sensor_write(chn, address, value);
			if (ret < 0) {
				sen_err(dev, "%s, port %d type %d write led reg failed!\n", __func__, chn, type);
			}
		break;
		default:
		break;
	}

	return ret;
}

static int32_t common_set_led(uint32_t chn, int32_t brightness)
{
	int32_t ret = 0;
	struct sensor_device_s *sen = NULL;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;
	struct sensor_led_s *cam_led;

	sen = sensor_dev_get(chn);
	if (sen == NULL) {
		return -ENODEV;
	}

	cam_p = &sen->camera_param;
	cam_led = &sen->camera_param.led_info;
	dev = &sen->osdev;

	if (sen->mdev.client == NULL) {
		sen_err(dev, "%s, port %d cam client is null\n", __func__, chn);
		return -1;
	}

	if (brightness < 0 )
		brightness = 0;
	if (brightness > 100)
		brightness = 100;

	ret = camera_set_led(chn, brightness, cam_led);
	if (ret < 0) {
		sen_err(dev, "camera %d type %d set led failed\n", chn, cam_led->type);
	}

	return ret;
}

static int32_t common_get_base_info(uint32_t chn, struct isi_sensor_base_info_s *user_para)
{
	if (chn >= FIRMWARE_CONTEXT_NUMBER) {
		pr_err("%s chn %d beyond 16\n", __func__, chn);
		return -1;
	}

        // sync
        get_tuning_info(chn);

        memcpy(user_para->sensor_name, tuning_param[chn].sensor_name, sizeof(tuning_param[chn].sensor_name));
        user_para->chn = chn;
        user_para->sensor_addr = tuning_param[chn].sensor_addr;
        user_para->bus_num = tuning_param[chn].bus_num;
        user_para->bus_type = tuning_param[chn].bus_type;
        user_para->reg_width = tuning_param[chn].reg_width;
        user_para->mode = tuning_param[chn].mode;

        return 0;
}

struct sensor_isp_ops_s sensor_isp_ops = {
	.sensor_alloc_analog_gain = common_alloc_analog_gain,
	.sensor_alloc_digital_gain = common_alloc_digital_gain,
	.sensor_alloc_integration_time = common_alloc_integration_time,
	.sensor_update = common_update,
	.read_register = common_read_register,
	.write_register = common_write_register,
	.sensor_get_para = common_get_param,
	.sensor_get_base_info = common_get_base_info,
        .sensor_awb_para = common_awb_param,
	.sensor_set_led = common_set_led,
	.end_magic = SENSOR_OPS_END_MAGIC,
};

static int32_t common_get_analog_gain(uint32_t chn, struct isi_sensor_again_info_s *user_again)
{
        if (chn >= FIRMWARE_CONTEXT_NUMBER) {
                pr_err("%s chn %d beyond 8\n", __func__, chn);
                return -1;
        }

        user_again->again_buf[0] = sensor_ctl[chn].gain_buf[0];
        user_again->again_buf[1] = sensor_ctl[chn].gain_buf[1];
        user_again->again_buf[2] = sensor_ctl[chn].gain_buf[2];
        user_again->again_num = sensor_ctl[chn].gain_num;

        return 0;
}

static int32_t common_get_digital_gain(uint32_t chn, struct isi_sensor_dgain_info_s *user_dgain)
{
        if (chn >= FIRMWARE_CONTEXT_NUMBER) {
                pr_err("%s chn %d beyond 8\n", __func__, chn);
                return -1;
        }

        user_dgain->dgain_buf[0] = sensor_ctl[chn].dgain_buf[0];
        user_dgain->dgain_buf[1] = sensor_ctl[chn].dgain_buf[1];
        user_dgain->dgain_buf[2] = sensor_ctl[chn].dgain_buf[2];
        user_dgain->dgain_num = sensor_ctl[chn].dgain_num;

        return 0;
}

static int32_t common_get_integration_time(uint32_t chn, struct isi_sensor_line_info_s *user_line)
{
        if (chn >= FIRMWARE_CONTEXT_NUMBER) {
                pr_err("%s chn %d beyond 8\n", __func__, chn);
                return -1;
        }

        user_line->line_buf[0] = sensor_ctl[chn].line_buf[0];
        user_line->line_buf[1] = sensor_ctl[chn].line_buf[1];
        user_line->line_buf[2] = sensor_ctl[chn].line_buf[2];
        user_line->line_num = sensor_ctl[chn].line_num;

        return 0;
}

static int32_t common_get_awb_gain(uint32_t chn, struct isi_sensor_awb_info_s *user_awb)
{
        if (chn >= FIRMWARE_CONTEXT_NUMBER) {
                pr_err("%s chn %d beyond 8\n", __func__, chn);
                return -1;
        }

        user_awb->rgain = sensor_ctl[chn].rgain;
        user_awb->bgain = sensor_ctl[chn].bgain;
        user_awb->grgain = sensor_ctl[chn].grgain;
        user_awb->gbgain = sensor_ctl[chn].gbgain;
        user_awb->temper = sensor_ctl[chn].temper;

        return 0;
}


struct sensor_isi_ops_s sensor_isi_ops = {
        .sensor_alloc_analog_gain = isi_alloc_analog_gain,
        .sensor_alloc_digital_gain = isi_alloc_digital_gain,
        .sensor_alloc_integration_time = isi_alloc_integration_time,
        .sensor_get_analog_gain = common_get_analog_gain,
        .sensor_get_digital_gain = common_get_digital_gain,
        .sensor_get_integration_time = common_get_integration_time,
        .sensor_update = isi_update,
        .read_register = common_read_register,
        .write_register = common_write_register,
        .sensor_get_para = common_get_param,
        .sensor_get_base_info = common_get_base_info,
        .sensor_awb_para = common_awb_param,
        .sensor_get_awb_para = common_get_awb_gain,
        .end_magic = SENSOR_OPS_END_MAGIC,
};

static void sensor_frame_event_2a(int32_t flow_id, enum _sensor_frame_event_e event)
{
	struct sensor_device_s *sen = sensor_dev_get_by_flow(flow_id);

	if (sen == NULL)
		return;
	sensor_frame_event_record(sen, event);

	if (event == sen->param.ae_event_flag)
		wake_up_ae_update(flow_id);
}

static int32_t sensor_get_ts_compensate(int32_t flow_id)
{
	struct sensor_device_s *sen = sensor_dev_get_by_flow(flow_id);

	if (sen == NULL)
		return 0;

	return sen->param.ts_compensate;
}

struct sensor_cim_ops_s sensor_cim_ops = {
	.sensor_get_para = common_get_param,
	.sensor_frame_event = sensor_frame_event_2a,
	.sensor_get_ts_compensate = sensor_get_ts_compensate,
	.end_magic = SENSOR_OPS_END_MAGIC,
};

int32_t camera_subdev_init(void)
{
	uint32_t i = 0, j = 0;

	/* init event */
	sensor_event_arr = osal_kmalloc((HARDWARE_BUF_NUMBER * CAMERA_TOTAL_NUMBER * sizeof(sensor_event_node_t)), GFP_KERNEL);
	if (sensor_event_arr == NULL) {
		sen_err(NULL, "kmalloc event_node failed!");
		return -ENOMEM;
	}
	memset(sensor_event_arr, 0, (HARDWARE_BUF_NUMBER * CAMERA_TOTAL_NUMBER * sizeof(sensor_event_node_t)));
	/* init list & spin lock */
	for (i = 0; i < CAMERA_TOTAL_NUMBER; i++) {
		sensor_event_header[i].ctx_id = i;
		osal_list_head_init(&sensor_event_header[i].list_free);
		osal_list_head_init(&sensor_event_header[i].list_busy);
		osal_spin_init(&sensor_event_header[i].lock);	/*PRQA S 3334*/
		for (j = 0; j < HARDWARE_BUF_NUMBER; j++) {
			osal_list_add(&sensor_event_arr[i * HARDWARE_BUF_NUMBER + j].list_node, &sensor_event_header[i].list_free);
		}
		/* add work func */
		INIT_WORK(&sensor_event_header[i].updata_work, write_isp_work);
	}

	return 0;
}

void camera_subdev_exit(void)
{
	uint32_t i = 0;

	for (i = 0; i < CAMERA_TOTAL_NUMBER; i++) {
		cancel_work_sync(&sensor_event_header[i].updata_work);
	}

	/* deinit event */
	if (sensor_event_arr) {
		osal_kfree(sensor_event_arr);
		sensor_event_arr = NULL;
	}
}
