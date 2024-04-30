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
 * @file camera_ctrl.c
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#include "camera_dev.h"
#include "camera_ctrl.h"
#include "camera_subdev.h"
#include "hobot_sensor_ops.h"

static camera_ctrlmod_s *camera_ctrl;
static sensor_ctrl_info_t sensor_info[CAMERA_TOTAL_NUMBER];
static sensor_ctrl_info_t sensor_info_b[CAMERA_TOTAL_NUMBER];
static const struct sensor_version_info_s camera_ctrl_ver = {
	SENSOR_CTRL_VER_MAJOR, SENSOR_CTRL_VER_MINOR,
};

static osal_waitqueue_t sensor_update;
static volatile uint64_t update_flag[CAMERA_TOTAL_NUMBER];

void set_sensor_aexp_info(uint32_t chn, void *ptr)
{
	struct sensor_device_s *sen = sensor_dev_get(chn);
	struct os_dev *dev = (sen != NULL) ? &sen->osdev : NULL;

	if ((ptr) && (chn < CAMERA_TOTAL_NUMBER)) {
		sensor_priv_t *data = (sensor_priv_t *)ptr;
		sensor_ctrl_info_t *ae_data = &sensor_info[chn];
		if (osal_test_bit((int32_t)CTRL_BUF_SELECT_FLAG, &update_flag[chn]) != CTRL_BUF_PING) {
			ae_data = &sensor_info_b[chn];
		}

		ae_data->gain_num = data->gain_num;
		memcpy(ae_data->gain_buf, data->gain_buf, sizeof(ae_data->gain_buf));
		ae_data->dgain_num = data->dgain_num;
		memcpy(ae_data->dgain_buf, data->dgain_buf, sizeof(ae_data->dgain_buf));
		ae_data->en_dgain = data->en_dgain;
		ae_data->line_num = data->line_num;
		memcpy(ae_data->line_buf, data->line_buf, sizeof(ae_data->line_buf));
		ae_data->rgain = data->rgain;
		ae_data->bgain = data->bgain;
		ae_data->grgain = data->grgain;
		ae_data->gbgain = data->gbgain;
		ae_data->color_temper = data->temper;
		ae_data->mode = data->mode;
		ae_data->port = chn;
		if (ae_data->id != 0U) {
			sen_warn(dev,"%s ctrl %d over write warn\n", __func__, ae_data->id);
			sensor_frame_2a_record(sen, SENSOR_F2AS_UPDATE_WARN);
		}
		ae_data->id = data->id;
	}
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief camera sensor control wait updata and wakeup
 *
 * @param[in] chn: sensor port index
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
int sensor_ctrl_completion_wait(struct sensor_device_s *sen)
{
	struct os_dev *dev = NULL;
	struct sensor_param_s *pa;
	uint32_t td = 0, chn;
	int ret = 0;
	int32_t ctrl_timeout_ms;

	if (sen == NULL) {
		sen_err(dev, "ctrl wait sen NULL error\n");
		return -1;
	}
	chn = sen->port;
	dev = &sen->osdev;
	pa = &sen->param;
	ctrl_timeout_ms = pa->ctrl_timeout_ms;

	td = osal_wait_event_interruptible_timeout(sensor_update,
		osal_test_bit((int32_t)CTRL_FS_FLAG, &update_flag[chn]), ctrl_timeout_ms);/*PRQA S 2996,4434,4460*/
	osal_clear_bit((int32_t)CTRL_FS_FLAG, &update_flag[chn]);
	osal_clear_bit((int32_t)CTRL_DATA_FLAG, &update_flag[chn]);
	if (td == 0) {
		sen_debug(dev, "ctrl wait trigger %dms timeout\n", ctrl_timeout_ms);
		ret = -1;
	} else if (td < 0) {
		sen_err(dev, "ctrl wait trigger error %d\n", td);
		ret = td;
	}

	return ret;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief camera sensor control done and sync result
 *
 * @param[in] res: sensor ctrl result struct
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
int sensor_ctrl_set_result(sensor_ctrl_result_t *res)
{
	struct sensor_device_s *sen;
	struct os_dev *dev;
	uint32_t check;

	if (res == NULL)
		return -1;
	sen = sensor_dev_get(res->port);
	if (sen == NULL)
		return -1;
	dev = &sen->osdev;

	if (res->result == 0) {
		check = sensor_frame_2a_check(sen, res->id);
		sensor_frame_2a_record(sen, SENSOR_F2AS_DONE);
		if (check < 0) {
			sen_warn(dev, "%s ops 0x%x ctrl %d frame out warn",
				__func__, res->ops, res->id);
			sensor_frame_2a_record(sen, SENSOR_F2AS_DONE_WARN);
		} else {
			sen_debug(dev, "%s ops 0x%x ctrl %d",
				__func__, res->ops, res->id);
		}
	} else {
		sen_err(dev, "%s ops 0x%x ctrl %d failed",
			__func__, res->ops, res->id);
	}

	return 0;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief camera sensor control wakeup the thread waited
 *
 * @param[in] chn: sensor port index
 *
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void sensor_ctrl_wakeup(uint32_t chn)
{
	if (osal_test_bit((int32_t)CTRL_DATA_FLAG, &update_flag[chn])) {
		osal_set_bit((int32_t)CTRL_FS_FLAG, &update_flag[chn]);
	}
	osal_wake_up(&sensor_update);
}

void sensor_ctrl_wakeup_flag(uint32_t chn)
{
	osal_set_bit((int32_t)CTRL_DATA_FLAG, &update_flag[chn]);
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief camera ctrl driver file operation: open
 *
 * @param[in] pinode: file node point
 * @param[in] pfile: file point
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
static int camera_ctrl_fop_open(struct inode *pinode, struct file *pfile)
{
	if (camera_ctrl == NULL) {
		sen_err(NULL, "[%s:%d] camera_fop_open failed, because param is NULL!\n", __func__, __LINE__);
		return -EFAULT;
	}

	camera_ctrl->user_num++;
	pfile->private_data = camera_ctrl;
	sen_debug(NULL, "camera_fop_open success %d\n", __LINE__);/*PRQA S 0685,1294*/
	return 0;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief camera ctrl driver file operation: release
 *
 * @param[in] pinode: file node point
 * @param[in] pfile: file point
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
static int camera_ctrl_fop_release(struct inode *pinode, struct file *pfile)
{
	camera_ctrlmod_s *camera_cdev = pfile->private_data;

	if (camera_cdev == NULL) {
		sen_err(NULL, "[%s:%d] camera_fop_release failed, because param is NULL!\n", __func__, __LINE__);
		return -EFAULT;
	}

	camera_cdev->user_num--;
	// pfile->private_data = NULL;
	sen_debug(NULL, "camera_fop_release success %d\n", __LINE__);/*PRQA S 0685,1294*/
	return 0;
}


/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief camera ctrl driver file operation: ioctl
 *
 * @param[in] pfile: file point
 * @param[in] cmd: ioctl cmd
 * @param[in] arg: ioctl arg
 *
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static long camera_ctrl_fop_ioctl(struct file *pfile, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;
	uint32_t chn = 0;
	uint32_t trigger_ok = 1U;
	sensor_ctrl_result_t res;
	struct os_dev *dev = NULL;
	struct sensor_device_s *sen;

	switch(cmd) {
		case SENSOR_CTRL_INFO_SYNC:/*PRQA S 0591*/
			if (osal_copy_from_app((void *)&chn, (void __user *)arg, sizeof(uint32_t))) {
				sen_err(dev, "copy chn is err !\n");
				ret = -EIO;
			} else {
				sen = sensor_dev_get(chn);
				dev = (sen != NULL) ? &sen->osdev : NULL;
				/* set ctrl_mode to user */
				sensor_ctrl_mode_set(sen, SENSOR_CTRLM_USER);
				ret = sensor_ctrl_completion_wait(sen);
				if (!ret) {
					if (osal_test_bit((int32_t)CTRL_BUF_SELECT_FLAG, &update_flag[chn]) == CTRL_BUF_PING) {
						if (osal_copy_to_app((void __user *)arg, (void *)(&sensor_info[chn]),
								sizeof(sensor_ctrl_info_t))) {
							sen_err(dev, "copy info is err !\n");
							ret = -EIO;
						}
						trigger_ok = sensor_info[chn].id;
						if (trigger_ok == 0U)
							sen_debug(dev, "%s ctrl info duplicate\n", __func__);
						else
							sensor_info[chn].id = 0U;
						osal_set_bit((int32_t)CTRL_BUF_SELECT_FLAG, &update_flag[chn]);
					} else {
						if (osal_copy_to_app((void __user *)arg, (void *)(&sensor_info_b[chn]),
								sizeof(sensor_ctrl_info_t))) {
							sen_err(dev, "copy info_b is err !\n");
							ret = -EIO;
						}
						trigger_ok = sensor_info[chn].id;
						if (trigger_ok == 0U)
							sen_debug(dev, "%s ctrl info duplicate\n", __func__);
						else
							sensor_info_b[chn].id = 0U;
						osal_clear_bit((int32_t)CTRL_BUF_SELECT_FLAG, &update_flag[chn]);
					}
					if  (!ret) {
						sensor_frame_2a_record(sen, SENSOR_F2AS_TRIGGER);
						if (trigger_ok != 0U)
							sensor_frame_2a_record(sen, SENSOR_F2AS_TRIGGER_WARN);
					}
				}
			}
			break;
		case SENSOR_CTRL_RESULT:
			if (osal_copy_from_app((void *)&res, (void __user *)arg, sizeof(sensor_ctrl_result_t))) {
				sen_err(dev, "copy result is err !\n");
				ret = -EIO;
			} else {
				ret = sensor_ctrl_set_result(&res);
			}
			break;
		case SENSOR_CTRL_GET_VERSION:
			if (osal_copy_to_app((void __user *)arg, (void *)&camera_ctrl_ver, sizeof(camera_ctrl_ver))) {
				sen_err(dev, "sensor iq version to user error\n");
				ret = -EIO;
			}
			break;
		default:
			sen_err(dev, "ioctl cmd is err \n");
			ret = -EINVAL;
			break;
	}

	return ret;
}

const struct file_operations camera_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = camera_ctrl_fop_open,
	.release = camera_ctrl_fop_release,
	.unlocked_ioctl = camera_ctrl_fop_ioctl,
	.compat_ioctl = camera_ctrl_fop_ioctl,
};

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief camera ctrl device register and init
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
int camera_ctrl_init(void)
{
	int ret = 0;

	memset((void *)update_flag, 0 , sizeof(update_flag));
	camera_ctrl = osal_kmalloc(sizeof(camera_ctrlmod_s), GFP_ATOMIC);
	if (camera_ctrl == NULL) {
		sen_err(NULL, "[%s:%d] kmalloc failed!\n", __func__, __LINE__);
		return -ENOMEM;
	}
	memset(camera_ctrl, 0, sizeof(camera_ctrlmod_s));
	snprintf(camera_ctrl->name, CTRL_DEVNAME_LEN, "sensor_ctrl");
	camera_ctrl->camera_chardev.name = camera_ctrl->name;
	camera_ctrl->camera_chardev.minor = MISC_DYNAMIC_MINOR;
	camera_ctrl->camera_chardev.fops = &camera_ctrl_fops;

	ret = misc_register(&camera_ctrl->camera_chardev);
	if (ret) {
		sen_err(NULL, "[%s:%d], register failed, err %d !\n",
					__func__, __LINE__, ret);
		goto register_err;
	}
	camera_ctrl->dev_minor_id = camera_ctrl->camera_chardev.minor;
	osal_waitqueue_init(&sensor_update);
	sen_debug(NULL, "%s v%u.%u register success !\n",
		camera_ctrl->name,
		camera_ctrl_ver.major, camera_ctrl_ver.minor);/*PRQA S 0685,1294*/
	return ret;

register_err:
	osal_kfree(camera_ctrl);
	return ret;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief camera ctrl device deregister and exit
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void camera_ctrl_exit(void)
{
	misc_deregister(&camera_ctrl->camera_chardev);
	osal_kfree(camera_ctrl);
	camera_ctrl = NULL;
	sen_debug(NULL, "camera_ctrl_exit success\n");/*PRQA S 0685,1294*/
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief camera ctrl driver exit
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void camera_ctrldev_exit(void)
{
	camera_ctrl_exit();
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief camera ctrl driver init
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
int camera_ctrldev_init(void)
{
	int ret = 0;

	ret = camera_ctrl_init();
	if (ret < 0) {
		sen_err(NULL, "[%s:%d] camera_ctrl_init is failed\n", __func__, __LINE__);
	}

	return ret;
}

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("camera_ctrl dev of J5");
MODULE_LICENSE("GPL");

