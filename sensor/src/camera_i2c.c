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
 * @file camera_i2c.c
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include "hobot_sensor_ops.h"
#include "camera_i2c.h"

/* #include "inc/camera_dev.h" */
/* #include "inc/camera_subdev.h" */
/* #include "inc/camera_i2c.h" */

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera driver i2c open: new i2c client
 *
 * @param[in] sen: the sensor driver struct
 * @param[in] bus_num: the i2c bus number
 * @param[in] sensor_name: the sensor name string
 * @param[in] sensor_addr: the sensor addr
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
int32_t camera_i2c_open(struct sensor_device_s *sen, uint32_t bus_num, char *sensor_name, uint32_t sensor_addr)
{
	int32_t ret = 0;
	struct sensor_miscdev_s *mdev;
	struct os_dev* dev;
	struct i2c_adapter *adap;
	struct i2c_client *client;
	int32_t bus = bus_num;

	if (sen == NULL)
		return -ENODEV;
	if (sensor_name == NULL)
		return -EINVAL;
	mdev = &sen->mdev;
	dev = &sen->osdev;

	if (sensor_addr == SENSOR_ADDR_IGNORE) {
		sen_info(dev, "%s open i2c%d@0x%02x ignore\n",
			sensor_name, bus, sensor_addr);
		mdev->dummy_sensor = 1;
		return 0;
	}

	osal_mutex_lock(&mdev->bus_mutex);
	if (dev->client != NULL) {
		if ((mdev->bus_num == bus_num) && (mdev->addr == sensor_addr) &&
			(strcmp(mdev->client_name, sensor_name) == 0)) {
			sen_debug(dev, "%s client has ready\n",
				sensor_name);
			osal_mutex_unlock(&mdev->bus_mutex);
			return 0;
		}
		i2c_unregister_device(dev->client);
		sen_info(dev, "%s release last client\n",
			sensor_name);
		dev->client = NULL;
		mdev->client = NULL;
		mdev->client_name = NULL;
	}

	mdev->dummy_sensor = 0;
	mdev->bus_num = bus_num;
	mdev->addr = sensor_addr;
	strncpy(dev->board_info.type, sensor_name, sizeof(dev->board_info.type));
	dev->board_info.addr = (uint16_t)sensor_addr;

	adap = i2c_get_adapter((int32_t)bus);
	if (adap == NULL) {
		sen_err(dev, "%s open get i2c%d adapt error\n",
			sensor_name, bus);
		ret = -ENODEV;
		goto i2c_open_err_unlock;
	}
	client = i2c_new_client_device(adap, &dev->board_info);
	if (IS_ERR_OR_NULL(client)) {
		sen_err(dev, "%s open i2c%d@0x%02x new client error\n",
			sensor_name, bus, sensor_addr);
		ret = -ENOMEM;
		goto i2c_open_err_put;
	}

	sen_info(dev, "%s open i2c%d@0x%02x\n",
		sensor_name, bus, sensor_addr);
	dev->client = client;
	mdev->client = client;
	mdev->client_name = dev->board_info.type;
	i2c_put_adapter(adap);
	osal_mutex_unlock(&mdev->bus_mutex);
	return ret;

i2c_open_err_put:
	i2c_put_adapter(adap);
i2c_open_err_unlock:
	osal_mutex_unlock(&mdev->bus_mutex);

	return ret;
}


/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera driver i2c release: free i2c client
 *
 * @param[in] sen: the sensor driver struct
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
int32_t camera_i2c_release(struct sensor_device_s *sen)
{
	struct sensor_miscdev_s *mdev;
	struct os_dev* dev;
	uint32_t bus;
	uint32_t sensor_addr;
	char *sensor_name;

	if (sen == NULL)
		return -ENODEV;
	mdev = &sen->mdev;
	dev = &sen->osdev;
	bus  = mdev->bus_num;
	sensor_addr = dev->board_info.addr;
	sensor_name = dev->board_info.type;

	osal_mutex_lock(&mdev->bus_mutex);
	if (dev->client == NULL) {
		sen_info(dev, "%s release client NULL\n",
			sensor_name);
		osal_mutex_unlock(&mdev->bus_mutex);
		return 0;
	}

	i2c_unregister_device(dev->client);
	sen_info(dev, "%s release i2c%d@0x%02x\n",
		sensor_name, bus, sensor_addr);
	dev->client = NULL;
	mdev->client = NULL;
	mdev->client_name = NULL;
	osal_mutex_unlock(&mdev->bus_mutex);

	return 0;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief get the camera sensor is dummy?
 *
 * @param[in] sen: the sensor driver struct
 *
 * @return 0:No, 1:Yes
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_i2c_isdummy(struct sensor_device_s *sen)
{
	return ((sen != NULL) ? (sen->mdev.dummy_sensor) : 1);
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief get the camera sensor is debug on?
 *
 * @param[in] sen: the sensor driver struct
 *
 * @return 0:No, 1:Yes
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t camera_i2c_is_debug(struct sensor_device_s *sen, uint32_t addr)
{
	struct sensor_param_s *pa = &sen->param;

	if (pa->i2c_debug <= 1)
		return pa->i2c_debug;
	return (pa->i2c_debug == addr) ? 1 : 0;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor i2c read: from sensor hw
 *
 * @param[in] sen: the sensor driver struct
 * @param[in] reg_addr: the i2c reg addr
 * @param[in] bit_width: the bit width of addr
 * @param[out] buf: the data buffer to read store
 * @param[in] count: the data byte count to read
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
int32_t camera_i2c_read(struct sensor_device_s *sen, uint32_t reg_addr,
		uint32_t bit_width, char *buf, uint32_t count)
{
	struct sensor_miscdev_s *mdev;
	struct os_dev* dev;
	uint32_t bus;
	uint32_t sensor_addr;
	char *sensor_name;
	struct i2c_client *client;
	struct i2c_adapter *adap;
	uint8_t tmp[4];
	struct i2c_msg msg[2];
	int32_t ret = 0;

	if (count == 0)
		return 0;
	if (sen == NULL)
		return -ENODEV;
	if (buf == NULL)
		return -EINVAL;
	mdev = &sen->mdev;
	dev = &sen->osdev;
	bus  = mdev->bus_num;
	sensor_addr = dev->board_info.addr;
	sensor_name = dev->board_info.type;

	if (camera_i2c_isdummy(sen)) {
		if (camera_i2c_is_debug(sen, reg_addr))
			sen_info(dev, "%s i2c%d@0x%02x R 0x%04x[%d]: ignore\n",
				sensor_name, bus, sensor_addr, reg_addr, count);
		return 0;
	}

	if (count > CAMERA_I2C_BYTE_MAX)
		count = CAMERA_I2C_BYTE_MAX;

	switch (bit_width) {
	case 8:
		tmp[0] = (uint8_t)(reg_addr & 0xFFu);
		msg[0].len = 1;
		break;
	case 16:
		tmp[0] = (uint8_t)((reg_addr >> 8) & 0xFFu);
		tmp[1] = (uint8_t)(reg_addr & 0xFFu);
		msg[0].len = 2;
		break;
	case 32:
		tmp[0] = (uint8_t)((reg_addr >> 24) & 0xFFu);
		tmp[1] = (uint8_t)((reg_addr >> 16) & 0xFFu);
		tmp[2] = (uint8_t)((reg_addr >> 8) & 0xFFu);
		tmp[3] = (uint8_t)(reg_addr & 0xFFu);
		msg[0].len = 4;
		break;
	default:
		sen_err(dev, "%s i2c%d@0x%02x R 0x%04x[%d]: bit_width %d error\n",
			sensor_name, bus, sensor_addr, reg_addr, count, bit_width);
		return -EINVAL;
	}

	osal_mutex_lock(&mdev->bus_mutex);
	client = mdev->client;
	if (client == NULL || client->adapter == NULL) {
		sen_err(dev, "%s i2c%d@0x%02x R 0x%04x[%d]: client NULL error\n",
			sensor_name, bus, sensor_addr, reg_addr, count);
		osal_mutex_unlock(&mdev->bus_mutex);
		return -ENODEV;
	}
	adap = client->adapter;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & (uint16_t)I2C_M_TEN;
	msg[0].buf = tmp;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & (uint16_t)I2C_M_TEN;
	msg[1].flags |= (uint16_t)I2C_M_RD;
	msg[1].len = (uint16_t)count;
	msg[1].buf = buf;

	if (camera_i2c_is_debug(sen, reg_addr))
		sen_info(dev, "%s i2c%d@0x%02x R 0x%04x[%d]: 0x%02x 0x%02x%s\n",
			sensor_name, bus, sensor_addr, reg_addr, count,
			buf[0], (count > 1) ? buf[1] : 0, (count > 2) ? " ..." : "");
	ret = i2c_transfer(adap, msg, 2);
	if (ret != 2) {
		sen_err(dev, "%s i2c%d@0x%02x R 0x%04x[%d]: 0x%02x 0x%02x%s error %d\n",
			sensor_name, bus, sensor_addr, reg_addr, count,
			buf[0], (count > 1) ? buf[1] : 0, (count > 2) ? " ..." : "", ret);
		osal_mutex_unlock(&mdev->bus_mutex);
		return -EIO;
	}
	osal_mutex_unlock(&mdev->bus_mutex);

	return 0;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor i2c write: to sensor hw
 *
 * @param[in] sen: the sensor driver struct
 * @param[in] reg_addr: the i2c reg addr
 * @param[in] bit_width: the bit width of addr
 * @param[in] buf: the data buffer to write
 * @param[in] count: the data byte count to write

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
int32_t camera_i2c_write(struct sensor_device_s *sen, uint32_t reg_addr, uint32_t bit_width,
		const char *buf, uint32_t count)
{
	struct sensor_miscdev_s *mdev;
	struct os_dev* dev;
	uint32_t bus;
	uint32_t sensor_addr;
	char *sensor_name;
	struct i2c_client *client;
	uint8_t tmp[CAMERA_I2C_BYTE_MAX + 4];
	int32_t wcount = 0;
	int32_t ret = 0;

	if (sen == NULL)
		return -ENODEV;
	mdev = &sen->mdev;
	dev = &sen->osdev;
	bus  = mdev->bus_num;
	sensor_addr = dev->board_info.addr;
	sensor_name = dev->board_info.type;

	if (camera_i2c_isdummy(sen)) {
		if (camera_i2c_is_debug(sen, reg_addr))
			sen_info(dev, "%s i2c%d@0x%02x W 0x%04x[%d]: 0x%02x 0x%02x%s ignore\n",
				sensor_name, bus, sensor_addr, reg_addr, count,
				buf[0], (count > 1) ? buf[1] : 0, (count > 2) ? " ..." : "");
		return 0;
	}

	if (count > CAMERA_I2C_BYTE_MAX)
		count = CAMERA_I2C_BYTE_MAX;

	switch (bit_width) {
	case 8:
		tmp[0] = (uint8_t)(reg_addr & 0xFFu);
		memcpy(&tmp[1], buf, count);
		wcount = 1 + count;
		break;
	case 16:
		tmp[0] = (uint8_t)((reg_addr >> 8) & 0xFFu);
		tmp[1] = (uint8_t)(reg_addr & 0xFFu);
		memcpy(&tmp[2], buf, count);
		wcount = 2 + count;
		break;
	case 32:
		tmp[0] = (uint8_t)((reg_addr >> 24) & 0xFFu);
		tmp[1] = (uint8_t)((reg_addr >> 16) & 0xFFu);
		tmp[2] = (uint8_t)((reg_addr >> 8) & 0xFFu);
		tmp[3] = (uint8_t)(reg_addr & 0xFFu);
		memcpy(&tmp[4], buf, count);
		wcount = 4 + count;
		break;
	default:
		sen_info(dev, "%s i2c%d@0x%02x W 0x%04x[%d]: 0x%02x 0x%02x%s bit_width %d error\n",
			sensor_name, bus, sensor_addr, reg_addr, count,
			buf[0], (count > 1) ? buf[1] : 0, (count > 2) ? " ..." : "", bit_width);
		return -EINVAL;
	}

	osal_mutex_lock(&mdev->bus_mutex);
	client = mdev->client;
	if (client == NULL) {
		sen_err(dev, "%s i2c%d@0x%02x W 0x%04x[%d]: 0x%02x 0x%02x%s client NULL error\n",
			sensor_name, bus, sensor_addr, reg_addr, count,
			buf[0], (count > 1) ? buf[1] : 0, (count > 2) ? " ..." : "");
		osal_mutex_unlock(&mdev->bus_mutex);
		return -ENODEV;
	}

	if (camera_i2c_is_debug(sen, reg_addr))
		sen_info(dev, "%s i2c%d@0x%02x R 0x%04x[%d]: 0x%02x 0x%02x%s\n",
			sensor_name, bus, sensor_addr, reg_addr, count,
			buf[0], (count > 1) ? buf[1] : 0, (count > 2) ? " ..." : "");
	ret = i2c_master_send(client, tmp, wcount);
	if (ret != wcount) {
		sen_err(dev, "%s i2c%d@0x%02x R 0x%04x[%d]: 0x%02x 0x%02x%s error %d\n",
			sensor_name, bus, sensor_addr, reg_addr, count,
			buf[0], (count > 1) ? buf[1] : 0, (count > 2) ? " ..." : "", ret);
		osal_mutex_unlock(&mdev->bus_mutex);
		return -EIO;
	}
	osal_mutex_unlock(&mdev->bus_mutex);

	return 0;
}
