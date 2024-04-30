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
 * @file camera_sys_api.c
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#include "hobot_sensor_ops.h"
#include "inc/camera_dev.h"
#include "inc/camera_subdev.h"
#include "inc/camera_i2c.h"
#include "inc/camera_sys_api.h"

/*  TODO */
static void DOFFSET(uint32_t *x, uint32_t n)
{
	switch (n) {
	case 1:
	break;
	case 2:
	*x = ((((*x) & 0xff00u) >> 8) | (((*x) & 0xffu) << 8));
		break;
	case 3:
	 *x = (((*x) & 0x0000ff00u) + (((*x) & 0xff0000u) >> 16)
		+ (((*x) & 0xffu) << 16));
		break;
	case 4:
	 *x = ((((((*x) & 0xff000000u) >> 8) +
		(((*x) & 0xff0000u) << 8)) >> 16) |
	        (((((*x) & 0xff00u) >> 8) +
		(((*x) & 0xffu) << 8)) << 16));
		break;
	default:
		break;
	}
}

/*
 * sensor gain/line ctrl for sensor_lut and so on
 * note: have again/dgain
 * note:
 */
void camera_gain_conversion(uint32_t *gain, uint32_t *gain_lut,
	uint32_t gain_num, uint32_t gain_temp, uint32_t conversion)
{
	uint32_t i = 0;
	uint32_t temp = 0;

	for(i = 0; i < gain_num; i++) {
		if (gain_temp > 255u)
			gain_temp = 255;

		if (gain_lut) {
			if (conversion) {
				temp = gain_lut[i * 256u + gain_temp];
				DOFFSET(&temp, 2);
				sen_debug(NULL, "a_gain[%d] = 0x%x\n", i, temp);/*PRQA S 0685,1294*/
			} else {
				sen_debug(NULL, "a_gain[%d] = 0x%x\n", i, gain_lut[i * 256u + gain_temp]);/*PRQA S 0685,1294*/
			}
			gain[i] = gain_lut[i * 256u + gain_temp];
		} else {
			sen_err(NULL, "a_gain[%d] gain is null\n", i);
			gain[i] = 0;
		}
	}
}
void camera_sys_sensor_gain_turning_data(uint32_t port,
	sensor_priv_t *priv_param, uint32_t *a_gain,
	uint32_t *d_gain, uint32_t *line)
{
	uint32_t a_gain_num = 0;
	uint32_t d_gain_num = 0;
	uint32_t *again_lut = NULL;
	uint32_t *dgain_lut = NULL;
	uint32_t  conversion = 0;
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	//mode switch
	sen = sensor_dev_get(port);
	if (sen == NULL)
		return;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	switch (cam_p->mode) {
	case NORMAL_M :
		a_gain_num = cam_p->normal.again_control_num;
		d_gain_num = cam_p->normal.dgain_control_num;
		again_lut = cam_p->normal.again_lut;
		dgain_lut = cam_p->normal.dgain_lut;
	break;
	case DOL2_M :
		a_gain_num = cam_p->dol2.again_control_num;
		d_gain_num = cam_p->dol2.dgain_control_num;
		again_lut = cam_p->dol2.again_lut;
		dgain_lut = cam_p->dol2.dgain_lut;
	break;
	case DOL3_M :
		a_gain_num = cam_p->dol3.again_control_num;
		d_gain_num = cam_p->dol3.dgain_control_num;
		again_lut = cam_p->dol3.again_lut;
		dgain_lut = cam_p->dol3.dgain_lut;
	break;
	case DOL4_M :
	break;
	case PWL_M :
		a_gain_num = cam_p->pwl.again_control_num;
		d_gain_num = cam_p->pwl.dgain_control_num;
		again_lut = cam_p->pwl.again_lut;
		dgain_lut = cam_p->pwl.dgain_lut;
	break;
	default:
		sen_err(dev, "%s mode is error\n", __func__);
	break;
	}

	conversion = cam_p->sensor_data.conversion;
	camera_gain_conversion(a_gain, again_lut, a_gain_num,
			priv_param->gain_buf[0], conversion);

	camera_gain_conversion(d_gain, dgain_lut, d_gain_num,
			priv_param->dgain_buf[0], conversion);
}

static uint32_t sensor_line_calculation(int32_t ratio, uint32_t offset,
	uint32_t max, uint32_t input)
{
	uint32_t line = 0;
	uint32_t r_t = 256;

	if (input > max) {
		input = max;
	}

	if (ratio < 0) {
		ratio = -ratio;
		r_t = (uint32_t)(ratio);
		line = (uint32_t)((offset > ((r_t * input) >> 8)) ? (offset - ((r_t * input) >> 8)) : 0);
	} else {
		r_t = (uint32_t)(ratio);
		line = (uint32_t)(offset + ((r_t * input) >> 8));
	}

	sen_debug(NULL, "%s, ratio 0x%x, offset 0x%x, max 0x%x, input 0x%x, line 0x%x\n",/*PRQA S 0685,1294*/
		__func__, ratio, offset, max, input, line);
	return line;
}

static uint32_t sensor_line_calculation_func2(int32_t ratio, uint32_t offset,
	uint32_t max, uint32_t min, uint32_t input)
{
	uint32_t line = 0;
	uint32_t r_t = 256;

	if (ratio < 0) {
		ratio = -ratio;
		r_t = (uint32_t)(ratio);
		line = (uint32_t)((offset > ((r_t * input) >> 16)) ? (offset - ((r_t * input) >> 16)) : 0);
	} else {
		r_t = (uint32_t)(ratio);
		line = (uint32_t)(offset + ((r_t * input) >> 16));
	}

	if (input > max) {
		input = max;
	}

	if (line < min) {
		line = min;
	}

	sen_debug(NULL, "%s, ratio 0x%x, offset 0x%x, max 0x%x, input 0x%x, line 0x%x\n",/*PRQA S 0685,1294*/
		__func__, ratio, offset, max, input, line);
	return line;
}

/*
 * sensor gain/line ctrl for sensor_lut and so on
 * note: have again/dgain
 * note:
 */
void camera_sys_sensor_line_turning_data(uint32_t port,
	sensor_priv_t *priv_param, uint32_t *a_line)
{
	uint32_t i = 0;
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	//data calculation
	sen = sensor_dev_get(port);
	if (sen == NULL)
		return;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	switch (cam_p->mode) {
	case DOL4_M :
	case DOL3_M :
		for(i = 0; i < 3u; i++) {
			a_line[i] = sensor_line_calculation(
				cam_p->dol3.line_p[i].ratio,
				cam_p->dol3.line_p[i].offset,
				cam_p->dol3.line_p[i].max,
				priv_param->line_buf[i]);
		}
	break;
	case DOL2_M :
		for(i = 0; i < 2u; i++) {
			a_line[i] = sensor_line_calculation(
				cam_p->dol2.line_p[i].ratio,
				cam_p->dol2.line_p[i].offset,
				cam_p->dol2.line_p[i].max,
				priv_param->line_buf[i]);
		}
	break;
	case NORMAL_M :
		a_line[0] = sensor_line_calculation(
			cam_p->normal.line_p.ratio,
			cam_p->normal.line_p.offset,
			cam_p->normal.line_p.max,
			priv_param->line_buf[0]);
	break;
	case PWL_M :
		if (cam_p->pwl.l_s_mode == 0) {
			a_line[0] = sensor_line_calculation(
				cam_p->pwl.line_p.ratio,
				cam_p->pwl.line_p.offset,
				cam_p->pwl.line_p.max,
				priv_param->line_buf[0]);
		} else {
			a_line[0] = priv_param->line_buf[0];
		}
	break;
	default:
		sen_debug(dev, "sensor_mode is err!\n");/*PRQA S 0685,1294*/
	break;
	}

	//line show
	for(i = 0; i < priv_param->line_num; i++) {
		sen_debug(dev, "%s, a_line[%d] 0x%x\n", __func__, i, a_line[i]);/*PRQA S 0685,1294*/
	}

	//data conversion
	if (cam_p->sensor_data.conversion) {
		switch (cam_p->mode) {
		case DOL4_M:
		break;
		case DOL3_M:
			DOFFSET(&a_line[0], cam_p->dol3.s_line_length);
			DOFFSET(&a_line[1], cam_p->dol3.m_line_length);
			DOFFSET(&a_line[2], cam_p->dol3.l_line_length);
		break;
		case DOL2_M:
			DOFFSET(&a_line[0], cam_p->dol2.s_line_length);
			DOFFSET(&a_line[1], cam_p->dol2.m_line_length);
		break;
		case NORMAL_M:
			DOFFSET(&a_line[0], cam_p->normal.s_line_length);
		break;
		case PWL_M:
			if (cam_p->pwl.l_s_mode == 0) {
				DOFFSET(&a_line[0], cam_p->pwl.line_length);
			}
		break;
		default:
			sen_debug(dev, "sensor_mode is err!\n");/*PRQA S 0685,1294*/
		break;
		}
	}
}


void camera_common_alloc_again(uint32_t port, uint32_t *a_gain)
{
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return;
	cam_p = &sen->camera_param;

	if (*a_gain >= (cam_p->sensor_data.analog_gain_max >> 13))
		*a_gain = cam_p->sensor_data.analog_gain_max >> 13;
}

uint32_t camera_sys_sensor_gain_alloc(uint32_t port, uint32_t input,
	uint32_t gain_sw)
{
	uint32_t i = 0;
	uint32_t count = 0;
	uint32_t gain_temp = input;
	uint32_t a_gain_num = 0;
	uint32_t d_gain_num = 0;
	uint32_t *again_lut = NULL;
	uint32_t *dgain_lut = NULL;
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	//mode switch
	sen = sensor_dev_get(port);
	if (sen == NULL)
		return gain_temp;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	switch (cam_p->mode) {
	case NORMAL_M :
		a_gain_num = cam_p->normal.again_control_num;
		d_gain_num = cam_p->normal.dgain_control_num;
		again_lut = cam_p->normal.again_lut;
		dgain_lut = cam_p->normal.dgain_lut;
	break;
	case DOL2_M :
		a_gain_num = cam_p->dol2.again_control_num;
		d_gain_num = cam_p->dol2.dgain_control_num;
		again_lut = cam_p->dol2.again_lut;
		dgain_lut = cam_p->dol2.dgain_lut;
	break;
	case DOL3_M :
		a_gain_num = cam_p->dol3.again_control_num;
		d_gain_num = cam_p->dol3.dgain_control_num;
		again_lut = cam_p->dol3.again_lut;
		dgain_lut = cam_p->dol3.dgain_lut;
	break;
	case DOL4_M :
	break;
	case PWL_M :
		a_gain_num = cam_p->pwl.again_control_num;
		d_gain_num = cam_p->pwl.dgain_control_num;
		again_lut = cam_p->pwl.again_lut;
		dgain_lut = cam_p->pwl.dgain_lut;
	break;
	default:
		sen_debug(dev, "%s mode is error\n", __func__);/*PRQA S 0685,1294*/
	break;
	}

	count = 0;
	if (gain_sw == 1u) {
		while(gain_temp > 1u) {
			for(i = 0; i < a_gain_num; i++) {
				if (again_lut[i * 256u + gain_temp] !=
					again_lut[i * 256u + gain_temp - 1u]) {
					return gain_temp;
				}
			}
			gain_temp--;
		}
	} else if (gain_sw == 2u) {
		while(gain_temp > 1u) {
			for(i = 0; i < d_gain_num; i++) {
				if (dgain_lut[i * 256u + gain_temp] !=
					dgain_lut[i * 256u + gain_temp - 1u]) {
					return gain_temp;
				}
			}
			gain_temp--;
		}
	} else {
		sen_err(dev, "wrong gain_sw %d\n", gain_sw);
	}
	sen_debug(dev, "%s gain %d\n", __func__, gain_temp);/*PRQA S 0685,1294*/

	return gain_temp;
}

void camera_common_alloc_lut_again(uint32_t port, uint32_t *a_gain)
{
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return;
	cam_p = &sen->camera_param;

	if (*a_gain >= (cam_p->sensor_data.analog_gain_max >> 13)) {
		*a_gain = cam_p->sensor_data.analog_gain_max >> 13;
	}

	*a_gain = camera_sys_sensor_gain_alloc(port, *a_gain, 1);
}

void camera_common_alloc_dgain(uint32_t port, uint32_t *d_gain)
{
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return;
	cam_p = &sen->camera_param;

	if (*d_gain >= (cam_p->sensor_data.digital_gain_max >> 13))
		*d_gain = cam_p->sensor_data.digital_gain_max >> 13;
}

void camera_common_alloc_lut_dgain(uint32_t port, uint32_t *d_gain)
{
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return;
	cam_p = &sen->camera_param;

	if (*d_gain >= (cam_p->sensor_data.digital_gain_max >> 13)) {
		*d_gain = cam_p->sensor_data.digital_gain_max >> 13;
	}

	*d_gain = camera_sys_sensor_gain_alloc(port, *d_gain, 2);
}

struct sensor_ctrl_ops sensor_ops[] = {
	{
		.ctrl_name = "lut_mode",
		.camera_gain_control = camera_sys_sensor_gain_turning_data,
		.camera_line_control = camera_sys_sensor_line_turning_data,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},

	{
		.ctrl_name = "lut_mode",
		.camera_gain_control = camera_sys_sensor_gain_turning_data,
		.camera_line_control = camera_sys_sensor_line_turning_data,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},

	{
		.ctrl_name = "lut_mode",
		.camera_gain_control = camera_sys_sensor_gain_turning_data,
		.camera_line_control = camera_sys_sensor_line_turning_data,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},
	{
		.ctrl_name = "lut_mode",
		.camera_gain_control = camera_sys_sensor_gain_turning_data,
		.camera_line_control = camera_sys_sensor_line_turning_data,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},

	{
		.ctrl_name = "lut_mode",
		.camera_gain_control = camera_sys_sensor_gain_turning_data,
		.camera_line_control = camera_sys_sensor_line_turning_data,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},

	{
		.ctrl_name = "lut_mode",
		.camera_gain_control = camera_sys_sensor_gain_turning_data,
		.camera_line_control = camera_sys_sensor_line_turning_data,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},
	{
		.ctrl_name = "lut_mode_1",
		.camera_gain_control = camera_sys_sensor_gain_turning_data,
		.camera_line_control = camera_sys_sensor_line_turning_data,
		.camera_alloc_again = camera_common_alloc_lut_again,
		.camera_alloc_dgain = camera_common_alloc_lut_dgain,
	},
};


void camera_sys_printk_disturing(sensor_tuning_data_t *turing_param)
{
	uint32_t i = 0;
	sensor_data_t *sensor_data;
	//sensor info

	sen_debug(NULL, "sensor_addr 0x%x, bus_type %d, reg_width %d",/*PRQA S 0685,1294*/
		turing_param->sensor_addr, turing_param->bus_type, turing_param->reg_width);
	sen_debug(NULL, "sensor_mode %d \n", turing_param->mode);/*PRQA S 0685,1294*/
	sen_debug(NULL, "s_line 0x%x s_line_length %d \n", turing_param->normal.s_line,/*PRQA S 0685,1294*/
		turing_param->normal.s_line_length);
	for (i = 0; i < turing_param->normal.again_control_num; i++) {
		sen_debug(NULL, "again_control[%d] 0x%x  again_control_length[%d] %d \n",/*PRQA S 0685,1294*/
			i, turing_param->normal.again_control[i],
			i, turing_param->normal.again_control_length[i]);
	}
	for (i = 0; i < turing_param->normal.dgain_control_num; i++) {
		sen_debug(NULL, "dgain_control[%d] 0x%x  dgain_control_length[%d] %d \n",/*PRQA S 0685,1294*/
			i, turing_param->normal.dgain_control[i],
			i, turing_param->normal.dgain_control_length[i]);
	}

	sen_debug(NULL, "line 0x%x line_length %d \n", turing_param->pwl.line,/*PRQA S 0685,1294*/
		turing_param->pwl.line_length);
	sen_debug(NULL, "active_height %d active_width %d \n",/*PRQA S 0685,1294*/
		turing_param->sensor_data.active_height,
		turing_param->sensor_data.active_width);
	sen_debug(NULL, "normal param_hold 0x%x param_hold_length %d \n",/*PRQA S 0685,1294*/
		turing_param->normal.param_hold,
		turing_param->normal.param_hold_length);
	sen_debug(NULL, "dol2 param_hold 0x%x param_hold_length %d \n",/*PRQA S 0685,1294*/
		turing_param->dol2.param_hold,
		turing_param->dol2.param_hold_length);

	sensor_data = &turing_param->sensor_data;

	sen_debug(NULL, "exposure_time_max= %d, exposure_time_min = %d, exposure_time_long_max = %d\n",/*PRQA S 0685,1294*/
			sensor_data->exposure_time_max,
			sensor_data->exposure_time_min,
			sensor_data->exposure_time_long_max);

}
int32_t camera_sys_read(uint32_t port, uint32_t reg_addr,
		uint32_t reg_width, char *buf, uint32_t length)
{
	int32_t ret = 0;
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	if (!length) {
		return ret;
	}

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	if(cam_p->bus_type == I2C_BUS) {
		ret = camera_i2c_read(sen, reg_addr, reg_width, buf, length);
	} else {
		sen_err(dev, "wrong bus type %d\n", cam_p->bus_type);
	}

	return ret;
}
int32_t camera_sys_write(uint32_t port, uint32_t reg_addr,
		uint32_t reg_width, char *buf, uint32_t length)
{
	int32_t ret = 0;
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	if (!length) {
		return ret;
	}

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	if(cam_p->bus_type == I2C_BUS) {
		ret = camera_i2c_write(sen, reg_addr, reg_width, buf, length);
	} else {
		sen_err(dev, "wrong bus type %d\n", cam_p->bus_type);
	}

	return ret;
}
int32_t	camera_sys_alloc_again(uint32_t port, uint32_t *a_gain)
{
	int32_t ret = 0;
	uint32_t num = 0;
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;
	num = cam_p->sensor_data.tuning_type;

	if (num  > sizeof(sensor_ops)/sizeof(struct sensor_ctrl_ops)) {
		sen_err(dev, "gain line calculation out range\n");
	} else {
		if (sensor_ops[num - 1u].camera_alloc_again)
			sensor_ops[num - 1u].camera_alloc_again(port, a_gain);
	}

	return ret;
}

int32_t	camera_sys_alloc_dgain(uint32_t port, uint32_t *d_gain)
{
	int32_t ret = 0;
	uint32_t num = 0;
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;
	num = cam_p->sensor_data.tuning_type;

	if (num > sizeof(sensor_ops)/sizeof(struct sensor_ctrl_ops)) {
		sen_err(dev, "gain line calculation out range\n");
	} else {
		if (sensor_ops[num - 1u].camera_alloc_dgain)
			sensor_ops[num - 1u].camera_alloc_dgain(port, d_gain);
	}

	return ret;
}

int32_t camera_sys_alloc_intergration_time(uint32_t port,
		uint32_t *intergration_time)
{
	int32_t ret = 0;
	return ret;
}

static void camera_trans_value(uint32_t *input, uint8_t *output)
{
	output[0] = (uint8_t)(input[0] & 0xffu);
	output[1] = (uint8_t)((input[0] >> 8) & 0xffu);
	output[2] = (uint8_t)((input[0] >> 16) & 0x03u);
	return;
}
static int32_t camera_sys_set_normal_gain(uint32_t port, uint32_t *input_gain,
			uint32_t *dinput_gain)
{
	uint8_t a_gain[3] = {0};
	uint32_t i = 0;
	int32_t ret = 0;
	uint32_t reg_width, s_gain, s_gain_length;
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;

	for (i = 0; i < cam_p->normal.again_control_num; i++) {
		reg_width = cam_p->reg_width;
		s_gain_length = cam_p->normal.again_control_length[i];
		s_gain = cam_p->normal.again_control[i];
		camera_trans_value(&input_gain[i], a_gain);
		ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
	}

	for (i = 0; i < cam_p->normal.dgain_control_num; i++) {
		reg_width = cam_p->reg_width;
		s_gain_length = cam_p->normal.again_control_length[i];
		s_gain = cam_p->normal.dgain_control[i];
		camera_trans_value(&dinput_gain[i], a_gain);
		ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
	}

	return ret;
}

static int32_t camera_sys_set_normal_line(uint32_t port, uint32_t *input_line)
{
	char line_d[3] = {0};
	int32_t ret = 0;
	uint32_t reg_width, s_line, s_line_length;
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;

	reg_width = cam_p->reg_width;
	s_line = cam_p->normal.s_line;
	s_line_length = cam_p->normal.s_line_length;

	camera_trans_value(input_line, line_d);
	ret = camera_sys_write(port, s_line, reg_width, line_d, s_line_length);

	return ret;
}

static int32_t camera_sys_set_dol2_gain(uint32_t port, uint32_t gain_num,
		uint32_t *input_gain, uint32_t *dinput_gain)
{
	uint8_t a_gain[3] = {0};
	uint32_t i = 0;
	int32_t ret = 0;
	uint32_t reg_width, s_gain_length, s_gain;
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;

	for (i = 0; i < cam_p->dol2.again_control_num; i++) {
		reg_width = cam_p->reg_width;
		s_gain_length = cam_p->dol2.again_control_length[i];
		s_gain = cam_p->dol2.again_control[i];
		camera_trans_value(&input_gain[i], a_gain);
		ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
	}

	for (i = 0; i < cam_p->dol2.dgain_control_num; i++) {
		reg_width = cam_p->reg_width;
		s_gain_length = cam_p->dol2.again_control_length[i];
		s_gain = cam_p->dol2.dgain_control[i];
		camera_trans_value(&dinput_gain[i], a_gain);
		ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
	}

	return ret;
}

static int32_t camera_sys_set_dol2_line(uint32_t port, uint32_t gain_num,
		uint32_t *input_line)
{
	char line_d[3] = {0};
	//char rev_d[3] = {0};
	int32_t ret = 0;
	uint32_t reg_width, s_line_length, m_line_length, s_line, m_line;
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;

	reg_width = cam_p->reg_width;
	s_line = cam_p->dol2.s_line;
	m_line = cam_p->dol2.m_line;
	s_line_length = cam_p->dol2.s_line_length;
	m_line_length = cam_p->dol2.m_line_length;

	if (gain_num > 1u) {
		camera_trans_value(&input_line[1], line_d);
		ret = camera_sys_write(port, m_line, reg_width, line_d, m_line_length);
	}
	if (gain_num > 0u) {
		camera_trans_value(input_line, line_d);
		ret = camera_sys_write(port, s_line, reg_width, line_d, s_line_length);
	}

	return ret;
}

static int32_t camera_sys_set_dol3_gain(uint32_t port, uint32_t gain_num,
		uint32_t *input_gain, uint32_t *dinput_gain)
{
	uint8_t gain_d[3] = {0};
	uint32_t i = 0;
	int32_t ret = 0;
	uint32_t reg_width, s_gain_length, s_gain;
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;

	for (i = 0; i < cam_p->dol3.again_control_num; i++) {
		reg_width = cam_p->reg_width;
		s_gain_length = cam_p->dol3.again_control_length[i];
		s_gain = cam_p->dol3.again_control[i];
		camera_trans_value(&input_gain[i], gain_d);
		ret = camera_sys_write(port, s_gain, reg_width, gain_d, s_gain_length);
	}

	for (i = 0; i < cam_p->dol3.dgain_control_num; i++) {
		reg_width = cam_p->reg_width;
		s_gain_length = cam_p->dol3.again_control_length[i];
		s_gain = cam_p->dol3.dgain_control[i];
		camera_trans_value(&dinput_gain[i], gain_d);
		ret = camera_sys_write(port, s_gain, reg_width, gain_d, s_gain_length);
	}

	return ret;
}

static int32_t camera_sys_set_dol3_line(uint32_t port, uint32_t gain_num,
		uint32_t *input_line)
{
	uint8_t line_d[3] = {0};
	int32_t ret = 0;
	uint32_t reg_width, s_line_length, m_line_length, s_line, m_line;
	uint32_t l_line_length, l_line;
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;

	reg_width = cam_p->reg_width;
	s_line = cam_p->dol3.s_line;
	m_line = cam_p->dol3.m_line;
	l_line = cam_p->dol3.l_line;
	s_line_length = cam_p->dol3.s_line_length;
	m_line_length = cam_p->dol3.m_line_length;
	l_line_length = cam_p->dol3.l_line_length;

	if (gain_num > 2u) {
		camera_trans_value(&input_line[2], line_d);
		ret = camera_sys_write(port, l_line, reg_width, line_d, l_line_length);
	}
	if (gain_num > 1u) {
		camera_trans_value(&input_line[1], line_d);
		ret = camera_sys_write(port, m_line, reg_width, line_d, m_line_length);
	}
	if (gain_num > 0u) {
		camera_trans_value(input_line, line_d);
		ret = camera_sys_write(port, s_line, reg_width, line_d, s_line_length);
	}

	return ret;
}

int32_t camera_sys_gain_line_process(uint32_t port, sensor_priv_t *priv_param,
				uint32_t *a_gain, uint32_t *d_gain, uint32_t *a_line)
{
	uint32_t num = 0;
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	num = cam_p->sensor_data.tuning_type;
	if (cam_p->sensor_data.tuning_type >
		sizeof(sensor_ops)/sizeof(struct sensor_ctrl_ops)) {
		sen_err(dev, "gain line calculation out range\n");
	} else {

                 if (num <= 0)
                         num = 1;

                 if (sensor_ops[num - 1u].camera_gain_control) {
			sensor_ops[num - 1u].camera_gain_control(port,
				priv_param, a_gain, d_gain, a_line);
                }
		if (sensor_ops[num - 1u].camera_line_control)
			sensor_ops[num - 1u].camera_line_control(port, priv_param, a_line);
	}

	return 0;
}

static int32_t camera_sys_set_pwl_line(uint32_t port, uint32_t line_num,
						uint32_t *input_line)
{
	int32_t ret = 0;
	uint8_t line_data[3] = {0};
	uint32_t reg_width, line_addr, line_length, temp_line;
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	if (cam_p->pwl.l_s_mode == 0) {
	// user normal
		reg_width = cam_p->reg_width;
		line_addr = cam_p->pwl.line;
		line_length = cam_p->pwl.line_length;

		if (line_num == 1u) {
			camera_trans_value(&input_line[0], line_data);
			sen_debug(dev, "input_line[0] 0x%x input_line[1] 0x%x\n",/*PRQA S 0685,1294*/
					line_data[0], line_data[1]);
			ret += camera_sys_write(port, line_addr, reg_width, line_data, line_length);
		}
	} else {
	// user other
		if (cam_p->pwl.line_num >= 4) {
			reg_width = cam_p->reg_width;
			line_addr = cam_p->pwl.line_ext[3];
			line_length = cam_p->pwl.line_length_ext[3];

			temp_line = sensor_line_calculation_func2(cam_p->pwl.line_p_ext[3].ratio,
				cam_p->pwl.line_p_ext[3].offset, cam_p->pwl.line_p_ext[3].max,
				cam_p->pwl.line_p_ext[3].min, input_line[0]);
			if(cam_p->sensor_data.conversion) {
				DOFFSET(&temp_line, cam_p->pwl.line_length_ext[3]);
			}
			camera_trans_value(&temp_line, line_data);
			sen_debug(dev, "3-input_line 0x%x sensor 0x%x\n", line_data[0], temp_line);/*PRQA S 0685,1294*/
			ret += camera_sys_write(port, line_addr, reg_width, line_data, line_length);
		}

		if (cam_p->pwl.line_num >= 3) {
			reg_width = cam_p->reg_width;
			line_addr = cam_p->pwl.line_ext[2];
			line_length = cam_p->pwl.line_length_ext[2];

			temp_line = sensor_line_calculation_func2(cam_p->pwl.line_p_ext[2].ratio,
				cam_p->pwl.line_p_ext[2].offset, cam_p->pwl.line_p_ext[2].max,
				cam_p->pwl.line_p_ext[2].min, input_line[0]);
			if(cam_p->sensor_data.conversion) {
				DOFFSET(&temp_line, cam_p->pwl.line_length_ext[2]);
			}
			camera_trans_value(&temp_line, line_data);
			sen_debug(dev, "2-input_line 0x%x sensor 0x%x\n", line_data[0], temp_line);/*PRQA S 0685,1294*/
			ret += camera_sys_write(port, line_addr, reg_width, line_data, line_length);
		}

		if (cam_p->pwl.line_num >= 2) {
			reg_width = cam_p->reg_width;
			line_addr = cam_p->pwl.line_ext[1];
			line_length = cam_p->pwl.line_length_ext[1];

			temp_line = sensor_line_calculation_func2(cam_p->pwl.line_p_ext[1].ratio,
				cam_p->pwl.line_p_ext[1].offset, cam_p->pwl.line_p_ext[1].max,
				cam_p->pwl.line_p_ext[1].min, input_line[0]);
			if(cam_p->sensor_data.conversion) {
				DOFFSET(&temp_line, cam_p->pwl.line_length_ext[1]);
			}
			camera_trans_value(&temp_line, line_data);
			sen_debug(dev, "1- input_line 0x%x sensor 0x%x\n", line_data[0], temp_line);/*PRQA S 0685,1294*/
			ret += camera_sys_write(port, line_addr, reg_width, line_data, line_length);
		}

		if (cam_p->pwl.line_num >= 1) {
			reg_width = cam_p->reg_width;
			line_addr = cam_p->pwl.line_ext[0];
			line_length = cam_p->pwl.line_length_ext[0];

			temp_line = sensor_line_calculation_func2(cam_p->pwl.line_p_ext[0].ratio,
				cam_p->pwl.line_p_ext[0].offset, cam_p->pwl.line_p_ext[0].max,
				cam_p->pwl.line_p_ext[0].min, input_line[0]);
			if(cam_p->sensor_data.conversion) {
				DOFFSET(&temp_line, cam_p->pwl.line_length_ext[0]);
			}
			camera_trans_value(&temp_line, line_data);
			sen_debug(dev, "0- input_line 0x%x sensor 0x%x\n", line_data[0], temp_line);/*PRQA S 0685,1294*/
			ret += camera_sys_write(port, line_addr, reg_width, line_data, line_length);
		}
	}

	return ret;
}
static int32_t camera_sys_set_pwl_gain(uint32_t port, uint32_t gain_num,
					uint32_t *input_gain)

{
	int32_t ret = 0;
	uint32_t i = 0;
	uint8_t gain_data[3] = {0};
	uint32_t reg_width, gain_addr, gain_length;
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;

	for (i = 0; i < cam_p->pwl.again_control_num; i++) {
		reg_width = cam_p->reg_width;
		gain_length = cam_p->pwl.again_control_length[i];
		gain_addr = cam_p->pwl.again_control[i];
		camera_trans_value(&input_gain[i], gain_data);
		ret = camera_sys_write(port, gain_addr, reg_width, gain_data, gain_length);
	}
	return ret;
}

static int32_t camera_sys_set_pwl_dgain(uint32_t port, uint32_t gain_num,
					uint32_t *input_dgain)

{
	int32_t ret = 0;
	uint32_t i = 0;
	uint8_t gain_data[3] = {0};
	uint32_t reg_width, gain_addr, gain_length;
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;

	for (i = 0; i < cam_p->pwl.dgain_control_num; i++) {
		reg_width = cam_p->reg_width;
		gain_length = cam_p->pwl.dgain_control_length[i];
		gain_addr = cam_p->pwl.dgain_control[i];
		camera_trans_value(&input_dgain[i], gain_data);
		ret = camera_sys_write(port, gain_addr, reg_width, gain_data, gain_length);
	}
	return ret;
}

int32_t camera_sys_set_ex_gain_control(uint32_t port, sensor_priv_t *priv_param,
		uint32_t *input_gain, uint32_t *input_dgain, uint32_t *input_line)
{
	int32_t ret = 0;

	camera_sys_set_pwl_line(port, priv_param->line_num, input_line);
	camera_sys_set_pwl_gain(port, priv_param->gain_num, input_gain);
	camera_sys_set_pwl_dgain(port, priv_param->gain_num, input_dgain);

	return ret;
}

int32_t camera_sys_set_param_hold(uint32_t port, uint32_t value)
{
	int32_t ret = 0;
	uint32_t param_hold = 0, param_hold_length = 0;
	uint32_t  reg_width;
	uint8_t buf[2];
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	reg_width = cam_p->reg_width;
	switch(cam_p->mode) {
		case NORMAL_M:
			param_hold = cam_p->normal.param_hold;
			param_hold_length = cam_p->normal.param_hold_length;
			break;
		case DOL2_M:
			param_hold = cam_p->dol2.param_hold;
			param_hold_length = cam_p->dol2.param_hold_length;
			break;
		case DOL3_M:
			param_hold = cam_p->dol3.param_hold;
			param_hold_length = cam_p->dol3.param_hold_length;
			break;
		case PWL_M:
			param_hold = cam_p->pwl.param_hold;
			param_hold_length = cam_p->pwl.param_hold_length;
			break;
		default:
			sen_err(dev, "[%s -- %d ] mode is err %d !", __func__, __LINE__,
					cam_p->mode);
			ret = -1;
			break;
	}
	if(param_hold != 0u) {
		if(value) {
			buf[0] = 0x01;
			ret = camera_sys_write(port, param_hold, reg_width, buf, param_hold_length);
		} else {
			buf[0] = 0x00;
			ret = camera_sys_write(port, param_hold, reg_width, buf, param_hold_length);
		}
	}
	return ret;
}
int32_t camera_sys_set_gain_line_control(uint32_t port,
		sensor_priv_t *priv_param)
{
	int32_t ret = 0;
	uint32_t a_gain[3] = {0};
	uint32_t d_gain[6] = {0};
	uint32_t a_line[3] = {0};
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	ret = camera_sys_gain_line_process(port, priv_param, a_gain, d_gain, a_line);
	if (ret) {
		sen_err(dev, "[%s -- %d] param is err!", __func__, __LINE__);
		return -1;
	}
	switch(cam_p->mode) {
		case NORMAL_M:
			camera_sys_set_param_hold(port, 0x1);
			camera_sys_set_normal_gain(port, a_gain, d_gain);
			camera_sys_set_normal_line(port, a_line);
			camera_sys_set_param_hold(port, 0x0);
			break;
		case DOL2_M:
			camera_sys_set_param_hold(port, 0x1);
			camera_sys_set_dol2_gain(port, priv_param->gain_num, a_gain, d_gain);
			/*Determine the different exposure times of long and short frames to avoid sensor abnormalities*/
			if(a_line[0] != a_line[1]) {
				camera_sys_set_dol2_line(port, priv_param->line_num, a_line);
			}
			camera_sys_set_param_hold(port, 0x0);
			break;
		case DOL3_M:
			camera_sys_set_param_hold(port, 0x1);
			camera_sys_set_dol3_gain(port, priv_param->gain_num, a_gain, d_gain);
			camera_sys_set_dol3_line(port, priv_param->line_num, a_line);
			camera_sys_set_param_hold(port, 0x0);
			break;
		case PWL_M:
			camera_sys_set_param_hold(port, 0x1);
			camera_sys_set_ex_gain_control(port, priv_param, a_gain, d_gain, a_line);
			camera_sys_set_param_hold(port, 0x0);
			break;
		default:
			sen_err(dev, "[%s -- %d ] mode is err %d !", __func__, __LINE__,
					cam_p->mode);
			ret = -1;
			break;
	}

	return ret;
}

static uint32_t *camera_sys_lut_fill(uint32_t gain_num, uint32_t *tuning_pram)
{
	uint32_t *gain_ptr = NULL;

	gain_ptr = kzalloc(256u * (unsigned long)gain_num * sizeof(uint32_t), GFP_KERNEL);
	if (gain_ptr) {
		if (osal_copy_from_app((void *)gain_ptr, (void __user *)tuning_pram,
			256u * (unsigned long)gain_num * sizeof(uint32_t))) {
				osal_kfree(gain_ptr);
				gain_ptr = NULL;
				sen_err(NULL, "copy is err !\n");
		}
	}

	return gain_ptr;
}

static int32_t camera_tuning_lut_fill(uint32_t *src_gain_lut, uint32_t gain_num,
	uint32_t **dst_gain_lut)
{
	int32_t ret = 0;
	uint32_t *ptr = NULL;

	if (src_gain_lut) {
		ptr = camera_sys_lut_fill(gain_num, src_gain_lut);
		if (ptr) {
			*dst_gain_lut = ptr;
		} else {
			ret = -1;
			return ret;
		}
	}
	return ret;
}

static int32_t camera_tuning_lut_map(struct sensor_device_s *sen,
		sensor_tuning_data_t *tuning_pram)
{
	int32_t ret = 0;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	// tuning lut map
	switch (cam_p->mode) {
	case NORMAL_M:
		ret = camera_tuning_lut_fill(tuning_pram->normal.again_lut,
					tuning_pram->normal.again_control_num,
					&cam_p->normal.again_lut);
		if (ret)
			return ret;

		ret = camera_tuning_lut_fill(tuning_pram->normal.dgain_lut,
					tuning_pram->normal.dgain_control_num,
					&cam_p->normal.dgain_lut);
		break;
	case DOL2_M:
		ret = camera_tuning_lut_fill(tuning_pram->dol2.again_lut,
					tuning_pram->dol2.again_control_num,
					&cam_p->dol2.again_lut);
		if (ret)
			return ret;

		ret = camera_tuning_lut_fill(tuning_pram->dol2.dgain_lut,
					tuning_pram->dol2.dgain_control_num,
					&cam_p->dol2.dgain_lut);
		break;
	case DOL3_M:
		ret = camera_tuning_lut_fill(tuning_pram->dol3.again_lut,
					tuning_pram->dol3.again_control_num,
					&cam_p->dol3.again_lut);
		if (ret)
			return ret;

		ret = camera_tuning_lut_fill(tuning_pram->dol3.dgain_lut,
					tuning_pram->dol3.dgain_control_num,
					&cam_p->dol3.dgain_lut);

		break;
	case DOL4_M:
		break;
	case PWL_M:
		ret = camera_tuning_lut_fill(tuning_pram->pwl.again_lut,
					tuning_pram->pwl.again_control_num,
					&cam_p->pwl.again_lut);
		if (ret)
			return ret;

		ret = camera_tuning_lut_fill(tuning_pram->pwl.dgain_lut,
					tuning_pram->pwl.dgain_control_num,
					&cam_p->pwl.dgain_lut);
		break;
	default:
		sen_err(dev, "%s wrong mode %d", __func__, cam_p->mode);
		ret = -1;
		break;
	}

	if (tuning_pram->led_info.type == LUT) {
		ret = camera_tuning_lut_fill(tuning_pram->led_info.led_lut,
						1, &cam_p->led_info.led_lut);
	}

	return ret;
}

void camera_free_lut(uint32_t **gain_lut)
{
	if (*gain_lut != NULL) {
		osal_kfree(*gain_lut);
		*gain_lut = NULL;
	}
}
void camera_sys_lut_free(uint32_t port)
{
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return;
	cam_p = &sen->camera_param;

	camera_free_lut(&cam_p->normal.again_lut);
	camera_free_lut(&cam_p->normal.dgain_lut);
	camera_free_lut(&cam_p->dol2.again_lut);
	camera_free_lut(&cam_p->dol2.dgain_lut);
	camera_free_lut(&cam_p->dol3.again_lut);
	camera_free_lut(&cam_p->dol3.dgain_lut);
	camera_free_lut(&cam_p->pwl.again_lut);
	camera_free_lut(&cam_p->pwl.dgain_lut);
	camera_free_lut(&cam_p->led_info.led_lut);
}

int32_t camera_sys_tuning_set(uint32_t port, sensor_tuning_data_t *tuning_pram)
{
	int32_t ret = 0;
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;

	if (tuning_pram) {
		memcpy(cam_p, tuning_pram, sizeof(sensor_tuning_data_t));

		cam_p->normal.again_lut = NULL;
		cam_p->normal.dgain_lut = NULL;
		cam_p->dol2.again_lut = NULL;
		cam_p->dol2.dgain_lut = NULL;
		cam_p->dol3.again_lut = NULL;
		cam_p->dol3.dgain_lut = NULL;
		cam_p->pwl.again_lut = NULL;
		cam_p->pwl.dgain_lut = NULL;
		cam_p->led_info.led_lut = NULL;
	} else {
		return -1;
	}

	ret = camera_tuning_lut_map(sen, tuning_pram);
	if (ret)
		return ret;

	camera_sys_printk_disturing(cam_p);

	return ret;
}

int32_t camera_sys_priv_set(uint32_t port, sensor_priv_t *priv_param)
{
	int32_t ret = 0;
	uint32_t src_port = 0, dst_port = 0;
	struct sensor_device_s *sen;
	struct os_dev *dev = NULL;
	struct sensor_tuning_data_s *cam_p;
	int32_t check, dest_do = 1;

	sen = sensor_dev_get(port);
	if (sen == NULL) {
		sen_info(dev, "%s, port %d sen is null\n", __func__, port);
		return -1;
	}
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	if (camera_i2c_isdummy(sen))
		return ret;

	if (sen->mdev.client == NULL) {
		sen_info(dev, "%s, port %d cam client is null\n", __func__, port);
		return -1;
	}

	if (priv_param) {
		check = sensor_frame_2a_check(sen, priv_param->id);
		if (check < 0) {
			sen_warn(dev,"%s id %d over frame warn\n", __func__, priv_param->id);
			sensor_frame_2a_record(sen, SENSOR_F2AS_UPDATE_WARN);
			return -1;
		}

		sensor_frame_2a_record(sen, SENSOR_F2AS_TRIGGER);

		src_port = (sen->param.ae_share_flag >> 16) & 0xffu;
		dst_port = sen->param.ae_share_flag & 0xfu;

		camera_sys_set_gain_line_control(port, priv_param);

		/* ae share */
		if (src_port == dst_port) {
			dest_do = 0;
		} else if ((src_port - 0xA0u) >= CAMERA_TOTAL_NUMBER) {
			sen_warn(dev, "port src %d, dst %d exceed valid range.\n", src_port, dst_port);
			dest_do = 0;
		} else if ((src_port - 0xA0u) != port) {
			sen_warn(dev, "src port %d is not sharer, cur port %d\n", src_port, port);
			dest_do = 0;
		}
		if (dest_do != 0)
			camera_sys_set_gain_line_control(dst_port, priv_param);

		check = sensor_frame_2a_check(sen, priv_param->id);
		sensor_frame_2a_record(sen, SENSOR_F2AS_DONE);
		if (check < 0)  {
			sen_warn(dev, "%s set%s ctrl %d frame out warn",
				__func__, (dest_do) ? " share" : "", priv_param->id);
			sensor_frame_2a_record(sen, SENSOR_F2AS_DONE_WARN);
		} else {
			sen_debug(dev, "%s set%s ctrl %d",
				__func__, (dest_do) ? " share" : "", priv_param->id);
		}
	} else {
		sen_info(dev, "%s, port %d priv_param is null\n", __func__, port);
		ret = -1;
	}

	return ret;
}

int32_t camera_sys_get_param(uint32_t port, sensor_data_t *sensor_data)
{
	int32_t ret = 0;
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	memcpy(sensor_data, &cam_p->sensor_data, sizeof(sensor_data_t));

	sen_debug(dev, "%s [%d] l:%d ag:%d dg:%d eM:%d em:%d\n", __func__ , port,
		sensor_data->lines_per_second,
		sensor_data->analog_gain_max, sensor_data->digital_gain_max,
		sensor_data->exposure_time_max, sensor_data->exposure_time_min);
	return ret;
}

int32_t camera_sys_get_tuning_param(uint32_t port, sensor_tuning_data_t *tuning_data)
{
	int32_t ret = 0;
	struct sensor_device_s *sen;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;

	memcpy(tuning_data, &sen->camera_param, sizeof(sensor_tuning_data_t));

	return ret;
}

static int32_t camera_sys_set_awb_control(uint32_t port, sensor_priv_t *priv_param)
{
	int32_t ret = 0;
	char awb_gain[3];
	uint32_t i = 0;
	uint32_t data = 0;
	const uint32_t SENSOR_AWB_GAIN_NUM_MAX = 4;
	uint32_t reg_width, rgain_addr, rgain_length, bgain_addr, bgain_length;
	uint32_t grgain_addr, grgain_length, gbgain_addr, gbgain_length;
	uint32_t *again_lut;
	uint32_t gain_temp = priv_param->gain_buf[0];
	uint32_t a_gain = 1;
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	if (camera_i2c_isdummy(sen))
		return ret;
	again_lut = cam_p->pwl.again_lut;
	reg_width = cam_p->reg_width;

	if (gain_temp > 255)
		gain_temp = 255;
	if (again_lut) {
		a_gain = again_lut[gain_temp];
		DOFFSET(&a_gain, 2);
	}
	sen_debug(dev, "gain_temp = %d, again=%d, apply_lut_gain=%d",/*PRQA S 0685,1294*/
		gain_temp, a_gain, cam_p->sensor_awb.apply_lut_gain);
	ret = camera_sys_set_param_hold(port, 0x1);
	for (i = 0; i < SENSOR_AWB_GAIN_NUM_MAX; i++) {
	        rgain_addr = cam_p->sensor_awb.rgain_addr[i];
	        rgain_length = cam_p->sensor_awb.rgain_length[i];
	        if (rgain_length != 0) {
	                if (cam_p->sensor_awb.rb_prec > 8) {
	                        data = priv_param->rgain << (cam_p->sensor_awb.rb_prec - 8);
	                } else {
						if (cam_p->sensor_awb.apply_lut_gain) {
							// (min rgain) 0x100 * (min lut again) 0x80 >> 8 = (min reg 1x) 0x80
							// (min rgain) 0x100 * (min lut again) 0x200 >> 8 = (min reg 1x) 0x200
							data = (priv_param->rgain * a_gain) >> 8;
						} else {
							data = priv_param->rgain >>
								(8 - cam_p->sensor_awb.rb_prec);
						}
					}
	                if (cam_p->sensor_data.conversion) {
	                        DOFFSET(&data, rgain_length);
	                }
	                camera_trans_value(&data, awb_gain);
					sen_debug(dev, "rgain[%d] = %x, origin rgain = %d, again = %d\n",/*PRQA S 0685,1294*/
							 i, data, priv_param->rgain, a_gain);
	                ret += camera_sys_write(port, rgain_addr, reg_width, awb_gain, rgain_length);
	        }

	        bgain_addr = cam_p->sensor_awb.bgain_addr[i];
	        bgain_length = cam_p->sensor_awb.bgain_length[i];
	        if (bgain_length != 0) {
	                if (cam_p->sensor_awb.rb_prec > 8) {
	                        data = priv_param->bgain << (cam_p->sensor_awb.rb_prec - 8);
	                } else {
						if (cam_p->sensor_awb.apply_lut_gain) {
							data = (priv_param->bgain * a_gain) >> 8;
						} else {
							data = priv_param->bgain >>
								(8 - cam_p->sensor_awb.rb_prec);
						}
					}
	                if (cam_p->sensor_data.conversion) {
	                        DOFFSET(&data, bgain_length);
	                }
	                camera_trans_value(&data, awb_gain);
					sen_debug(dev, "bgain[%d] = %x, origin bgain = %d, again = %d\n",/*PRQA S 0685,1294*/
							 i, data, priv_param->bgain, a_gain);
	                ret += camera_sys_write(port, bgain_addr, reg_width, awb_gain, bgain_length);
	        }

	        grgain_addr = cam_p->sensor_awb.grgain_addr[i];
	        grgain_length = cam_p->sensor_awb.grgain_length[i];
	        if (grgain_length != 0) {
	                if (cam_p->sensor_awb.rb_prec > 8) {
	                        data = priv_param->grgain << (cam_p->sensor_awb.rb_prec - 8);
	                } else {
						if (cam_p->sensor_awb.apply_lut_gain) {
							data = (priv_param->grgain * a_gain) >> 8;
						} else {
							data = priv_param->grgain >>
								(8 - cam_p->sensor_awb.rb_prec);
						}
					}
	                if (cam_p->sensor_data.conversion) {
	                        DOFFSET(&data, grgain_length);
	                }
	                camera_trans_value(&data, awb_gain);
					sen_debug(dev, "grgain[%d] = %x, origin grgain = %d, again = %d\n",/*PRQA S 0685,1294*/
							 i, data, priv_param->grgain, a_gain);
	                ret += camera_sys_write(port, grgain_addr, reg_width, awb_gain, grgain_length);
	        }

	        gbgain_addr = cam_p->sensor_awb.gbgain_addr[i];
	        gbgain_length = cam_p->sensor_awb.gbgain_length[i];
	        if (gbgain_length != 0) {
	                if (cam_p->sensor_awb.rb_prec > 8) {
	                        data = priv_param->gbgain << (cam_p->sensor_awb.rb_prec - 8);
	                } else {
						if (cam_p->sensor_awb.apply_lut_gain) {
							data = (priv_param->gbgain * a_gain) >> 8;
						} else {
							data = priv_param->gbgain >>
								(8 - cam_p->sensor_awb.rb_prec);
						}
					}
	                if (cam_p->sensor_data.conversion) {
	                        DOFFSET(&data, gbgain_length);
	                }
	                camera_trans_value(&data, awb_gain);
					sen_debug(dev, "gbgain[%d] = %x, origin gbgain = %d, again = %d\n",/*PRQA S 0685,1294*/
							 i, data, priv_param->gbgain, a_gain);
	                ret += camera_sys_write(port, gbgain_addr, reg_width, awb_gain, gbgain_length);
	        }
	}
	ret += camera_sys_set_param_hold(port, 0x0);

	return ret;
}

int32_t camera_sys_priv_awb_set(uint32_t port, sensor_priv_t *priv_param)
{
	int32_t ret = 0;
	struct os_dev *dev = sensor_osdev_get(port);

	if (port >= CAMERA_TOTAL_NUMBER) {
		sen_err(dev, "not support %d max port is %d\n", port, CAMERA_TOTAL_NUMBER);
		ret = -1;
	} else {
        if (priv_param) {
            sensor_priv_t param;
            memcpy(&param, priv_param, sizeof(sensor_priv_t));
            sen_debug(dev, "rgain %d, bgain %d\n", param.rgain, param.bgain);/*PRQA S 0685,1294*/
            ret = camera_sys_set_awb_control(port, &param);
        } else {
			sen_err(dev, "priv_param is null!\n");
            ret = -1;
        }
	}
	return ret;
}

int32_t write_register(uint32_t port, uint8_t *pdata, uint32_t setting_size)
{
	int32_t ret = 0;
	uint16_t delay;
	uint32_t i = 0, len = 0, count = 0;
	struct i2c_client client;
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	if (sen->mdev.client == NULL)
		return -1;

	memset(&client, 0, sizeof(client));
	client.adapter = dev->client->adapter;
	while (i < setting_size) {
		len = pdata[i];
		if (len == 0) {
			delay = pdata[i + 1];
			osal_msleep(delay);
			i += 2;
		} else {
			client.addr = (uint16_t)pdata[i + 1] >> 1;
			count = len - 1;
			ret = i2c_master_send(&client, &pdata[i + 2], count);
			if (ret < 0) {
				sen_err(dev, "write sensor%d register error\n", port);
				return ret;
			}
			i = i + len + 1;
		}
	}

	return ret;
}

int32_t camera_sys_stream_on(uint32_t port)
{
	int32_t ret = 0;
	uint32_t i = 0;
	uint8_t buf[2];
	uint32_t reg_width, setting_size, data_length;
	uint32_t *stream_on = NULL;
	uint32_t size = 0;
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	stream_on = cam_p->stream_ctrl.stream_on;
	size = sizeof(cam_p->stream_ctrl.stream_on);

	if (cam_p->sensor_data.tuning_type == 5u) {
		ret = write_register(port, (uint8_t *)stream_on, size);
	} else {
		reg_width = cam_p->reg_width;
		data_length = cam_p->stream_ctrl.data_length;
		if (data_length <= 0) {	//FIXME just notice user.
			sen_info(dev, "stream on sensor%d, error \n", port);
			return ret;
		}
		setting_size = size / (uint32_t)sizeof(uint32_t);
		sen_info(dev, "stream on sensor%d: size %d\n", port, setting_size);/*PRQA S 0685,1294*/
		for (i = 0; i < setting_size; i += 2u) {
			if (stream_on[i]) {
				sen_debug(dev, " sensor%d on[%d]: 0x%x = 0x%x\n", port, i / 2,/*PRQA S 0685,1294*/
					stream_on[i], stream_on[i + 1u]);
				if(data_length == 1u) {
					buf[0] = (uint8_t)(stream_on[i + 1u] & 0xffu);
					camera_i2c_write(sen, stream_on[i], reg_width, buf, data_length);
				} else {
					buf[0] = (uint8_t)((stream_on[i + 1u] >> 8 ) & 0xffu);
					buf[1] = (uint8_t)(stream_on[i + 1u] & 0xffu);
					camera_i2c_write(sen, stream_on[i], reg_width, buf, data_length);
				}
			} else {
				break;
			}
		}
	}

	return ret;
}

int32_t camera_sys_stream_off(uint32_t port)
{
	int32_t ret = 0;
	uint8_t buf[2];
	uint32_t reg_width, setting_size, data_length;
	uint32_t i = 0;
	uint32_t *stream_off = NULL;
	uint32_t size = 0;
	struct sensor_device_s *sen;
	struct os_dev *dev;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;
	dev = &sen->osdev;

	stream_off = cam_p->stream_ctrl.stream_off;
	size = sizeof(cam_p->stream_ctrl.stream_off);
	if (cam_p->sensor_data.tuning_type == 5u) {
		ret = write_register(port, (uint8_t *)stream_off, size);
	} else {
		reg_width = cam_p->reg_width;
		data_length = cam_p->stream_ctrl.data_length;
		if (data_length <= 0) {	//FIXME just notice user.
			sen_info(dev, "stream off sensor%d, error \n", port);
			return ret;
		}
		setting_size = size / (uint32_t)sizeof(uint32_t);
		sen_info(dev, "stream off sensor%d: size %d\n", port, setting_size);/*PRQA S 0685,1294*/
		for(i = 0; i < setting_size; i += 2u) {
			if (stream_off[i]) {
				sen_debug(dev, " sensor%d off[%d]: 0x%x = 0x%x\n", port, i / 2,/*PRQA S 0685,1294*/
					stream_off[i], stream_off[i + 1u]);
				if(data_length == 1u) {
					buf[0] = (uint8_t)(stream_off[i + 1u] & 0xffu);
					camera_i2c_write(sen, stream_off[i], reg_width, buf, data_length);
				} else {
					buf[0] = (uint8_t)((stream_off[i + 1u] >> 8 ) & 0xffu);
					buf[1] = (uint8_t)(stream_off[i + 1u] & 0xffu);
					camera_i2c_write(sen, stream_off[i], reg_width, buf, data_length);
				}
			} else {
				break;
			}
		}
	}
	return ret;
}

int32_t camera_sys_sensor_write(uint32_t port, uint32_t address,
		uint32_t w_data)
{
	int32_t ret = 0;
	uint8_t buf[1];
	uint32_t reg_width;
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;

	reg_width = cam_p->reg_width;
	buf[0] = (uint8_t)(w_data & 0xffu);
	ret = camera_i2c_write(sen, address, reg_width, buf, 1);
	return ret;
}

int32_t camera_sys_sensor_read(uint32_t port, uint32_t address,
		uint32_t *r_data)
{
	int32_t ret = 0;
	char buf[1];
	uint32_t reg_width;
	struct sensor_device_s *sen;
	struct sensor_tuning_data_s *cam_p;

	sen = sensor_dev_get(port);
	if (sen == NULL)
		return -1;
	cam_p = &sen->camera_param;

	reg_width = cam_p->reg_width;
	ret = camera_i2c_read(sen, address, reg_width, buf, 1);
	*r_data = (uint32_t)(buf[0]);
	return ret;
}
