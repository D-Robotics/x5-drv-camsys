/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright TIMEOUT_CNT19 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#define pr_fmt(fmt)    "[ISI_Sensor]:" fmt

#include "hobot_isi_dev_sensor.h"
#include <linux/slab.h>

/**
 * global sensor driver struct
 */
extern int32_t chn_index[CAMERA_TOTAL_NUMBER];

static struct _isi_sensor_t *g_isi_sen = NULL;
struct isi_sensor_base_info_s g_sensor_base_info[FIRMWARE_CONTEXT_NUMBER];
struct isi_sensor_setting_param_s g_sensor_setting_param[FIRMWARE_CONTEXT_NUMBER];
sensor_priv_t g_sensor_priv[FIRMWARE_CONTEXT_NUMBER];

struct isi_sensor_again_param_s g_sensor_again_param[FIRMWARE_CONTEXT_NUMBER];
struct isi_sensor_dgain_param_s g_sensor_dgain_param[FIRMWARE_CONTEXT_NUMBER];
struct isi_sensor_line_param_s g_sensor_line_param[FIRMWARE_CONTEXT_NUMBER];
struct isi_sensor_awb_param_s g_sensor_awb_param[FIRMWARE_CONTEXT_NUMBER];

static bool g_base_info_updated = false;

static int16_t isi_get_sensor_again_param(uint32_t chn)
{
        int32_t ret = 0;
        pr_debug("%s \n", __func__);

        if (g_isi_sen == NULL)
                return -EINVAL;

        if (chn >= CAMERA_TOTAL_NUMBER || chn < 0) {
                pr_err("%s chn %d is error \n", __func__, chn);
                return -1;
        }

        if (g_isi_sen->isi_sensor_cops != NULL) {
                ret = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->sensor_get_analog_gain(chn, &g_sensor_again_param[chn].isi_sensor_again_info);
                if (ret < 0) {
                        pr_err("isi callback sensor_get_again error \n");
                        return -1;
                }

                g_sensor_again_param[chn].chn = chn;

        } else {
                pr_err("isi_sensor_cops is NULL \n");
                return -EINVAL;
        }

        return ret;
}

static void isi_set_sensor_again_param(struct isi_sensor_again_param_s *sensor_again_param)
{
        int32_t ret = 0;
        pr_debug("%s \n", __func__);

        if (g_isi_sen == NULL)
                return;

        if (sensor_again_param->chn >= CAMERA_TOTAL_NUMBER || sensor_again_param->chn < 0) {
                pr_err("%s chn %d is error \n", __func__, sensor_again_param->chn);
                return;
        }

        if (g_isi_sen->isi_sensor_cops != NULL) {
                ret = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->sensor_alloc_analog_gain(sensor_again_param->chn, sensor_again_param->isi_sensor_again_info.again_buf,
                                sensor_again_param->isi_sensor_again_info.again_num);
                if (ret < 0) {
                        pr_err("isi callback sensor_set_again error \n");
                        return;
                }
        } else {
                pr_err("isi_sensor_cops is NULL \n");
        }
}

static int16_t isi_get_sensor_dgain_param(uint32_t chn)
{
        int32_t ret = 0;
        pr_debug("%s \n", __func__);

        if (g_isi_sen == NULL)
                return -EINVAL;

        if (chn >= CAMERA_TOTAL_NUMBER || chn < 0) {
                pr_err("%s chn %d is error \n", __func__, chn);
                return -1;
        }

        if (g_isi_sen->isi_sensor_cops != NULL) {
                ret = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->sensor_get_digital_gain(chn, &g_sensor_dgain_param[chn].isi_sensor_dgain_info);
                if (ret < 0) {
                        pr_err("isi callback sensor_get_digital_gain error \n");
                        return -1;
                }

                g_sensor_dgain_param[chn].chn = chn;

        } else {
                pr_err("isi_sensor_cops is NULL \n");
                return -EINVAL;
        }

        return ret;
}

static void isi_set_sensor_dgain_param(struct isi_sensor_dgain_param_s *sensor_dgain_param)
{
        int32_t ret = 0;
        pr_debug("%s \n", __func__);

        if (g_isi_sen == NULL)
                return;

        if (sensor_dgain_param->chn >= CAMERA_TOTAL_NUMBER || sensor_dgain_param->chn < 0) {
                pr_err("%s chn %d is error \n", __func__, sensor_dgain_param->chn);
                return;
        }

        if (g_isi_sen->isi_sensor_cops != NULL) {
                ret = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->sensor_alloc_digital_gain(sensor_dgain_param->chn, sensor_dgain_param->isi_sensor_dgain_info.dgain_buf,
                                sensor_dgain_param->isi_sensor_dgain_info.dgain_num);
                if (ret < 0) {
                        pr_err("isi callback sensor_set_dgain error \n");
                }
        } else {
                pr_err("isi_sensor_cops is NULL \n");
        }

	//X5 TODO FIXME
	if (g_isi_sen->isi_sensor_cops != NULL) {
		ret = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->sensor_update(sensor_dgain_param->chn, 1);
		if (ret < 0) {
			pr_err("isi callback sensor_update error \n");
		}
	} else {
		pr_err("isi_sensor_cops is NULL \n");
	}
}

static int16_t isi_get_sensor_line_param(uint32_t chn)
{
        int32_t ret = 0;
        pr_debug("%s \n", __func__);

        if (g_isi_sen == NULL)
                return -EINVAL;

        if (chn >= CAMERA_TOTAL_NUMBER || chn < 0) {
                pr_err("%s chn %d is error \n", __func__, chn);
                return -1;
        }

        if (g_isi_sen->isi_sensor_cops != NULL) {
                ret = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->sensor_get_integration_time(chn, &g_sensor_line_param[chn].isi_sensor_line_info);
                if (ret < 0) {
                        pr_err("isi callback sensor_get_integration_time error \n");
                        return -1;
                }

                g_sensor_line_param[chn].chn = chn;

        } else {
                pr_err("isi_sensor_cops is NULL \n");
                return -EINVAL;
        }

        return ret;
}

static void isi_set_sensor_line_param(struct isi_sensor_line_param_s *sensor_line_param)
{
        int32_t ret = 0;
        pr_debug("%s \n", __func__);

        if (g_isi_sen == NULL)
                return;

        if (sensor_line_param->chn >= CAMERA_TOTAL_NUMBER || sensor_line_param->chn < 0) {
                pr_err("%s chn %d is error \n", __func__, sensor_line_param->chn);
                return;
        }

        // hobot_sensor 中对 mode 进行了区分，上层 ISP 也应该有区分
        if (g_isi_sen->isi_sensor_cops != NULL) {
                ret = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->sensor_alloc_integration_time(sensor_line_param->chn, &sensor_line_param->isi_sensor_line_info.line_buf[0],
                                &sensor_line_param->isi_sensor_line_info.line_buf[1],
                                &sensor_line_param->isi_sensor_line_info.line_buf[2]);
                if (ret < 0) {
                        pr_err("isi callback sensor_set_line_param error \n");
                }
        } else {
                pr_err("isi_sensor_cops is NULL \n");
        }

	//X5 TODO FIXME
	if (g_isi_sen->isi_sensor_cops != NULL) {
		ret = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->sensor_update(sensor_line_param->chn, 1);
		if (ret < 0) {
			pr_err("isi callback sensor_update error \n");
		}
	} else {
		pr_err("isi_sensor_cops is NULL \n");
	}
}

static int16_t isi_get_sensor_awb_param(uint32_t chn)
{
        int32_t ret = 0;
        pr_debug("%s \n", __func__);

        if (g_isi_sen == NULL)
                return -EINVAL;

        if (chn >= CAMERA_TOTAL_NUMBER || chn < 0) {
                pr_err("%s chn %d is error \n", __func__, chn);
                return -1;
        }

        if (g_isi_sen->isi_sensor_cops != NULL) {
                ret = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->sensor_get_awb_para(chn, &g_sensor_awb_param[chn].isi_sensor_awb_info);
                if (ret < 0) {
                        pr_err("isi callback sensor_get_awb_para error \n");
                        return -1;
                }

                g_sensor_awb_param[chn].chn = chn;

        } else {
                pr_err("isi_sensor_cops is NULL \n");
                return -EINVAL;
        }

        return ret;
}

static void isi_set_sensor_awb_param(struct isi_sensor_awb_param_s *sensor_awb_param)
{
        int32_t ret = 0;
        pr_debug("%s \n", __func__);

        if (g_isi_sen == NULL)
                return;

        if (sensor_awb_param->chn >= CAMERA_TOTAL_NUMBER || sensor_awb_param->chn < 0) {
                pr_err("%s chn %d is error \n", __func__, sensor_awb_param->chn);
                return;
        }

        if (g_isi_sen->isi_sensor_cops != NULL) {
                ret = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->sensor_awb_para(sensor_awb_param->chn, sensor_awb_param->isi_sensor_awb_info.rgain,
                  sensor_awb_param->isi_sensor_awb_info.bgain,
                  sensor_awb_param->isi_sensor_awb_info.grgain,
                  sensor_awb_param->isi_sensor_awb_info.gbgain,
                  sensor_awb_param->isi_sensor_awb_info.temper);
                if (ret < 0) {
                        pr_err("isi callback sensor_set_awb_param error \n");
                }
        } else {
                pr_err("isi_sensor_cops is NULL \n");
        }
}


static int16_t isi_get_sensor_setting_param(uint32_t chn)
{
        int32_t ret = 0;
        pr_debug("%s\n", __func__);

        if (g_isi_sen == NULL)
                return -EINVAL;

        if (g_isi_sen->isi_sensor_cops != NULL) {
                ret = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->sensor_get_para(chn, &g_sensor_setting_param[chn].setting_param_t);
                if (ret < 0) {
                        pr_err("isi callback sensor_get_para error \n");
                        return -1;
                }

                g_sensor_setting_param[chn].chn = chn;
        } else {
                pr_err("isi_sensor_cops is NULL \n");
                return -EINVAL;
        }

        return ret;
}

static void isi_set_sensor_cali_name(uint32_t chn, char *cali_name)
{
        int32_t ret = 0;
        pr_debug("%s \n", __func__);

        if (g_isi_sen == NULL)
                return;

        if (chn >= CAMERA_TOTAL_NUMBER || chn < 0) {
                pr_err("%s chn %d is error \n", __func__, chn);
                return;
        }

        if (g_isi_sen->isi_sensor_cops != NULL) {
                ret = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->sensor_set_cali_name(chn, cali_name);
                if (ret < 0) {
                        pr_err("isi callback sensor_set_cali_name error \n");
                        return;
                }
        } else {
                pr_err("isi_sensor_cops is NULL \n");
        }
}

static int32_t empty_common_alloc_analog_gain(uint32_t chn, int32_t *gain_ptr, uint32_t gain_num)
{
        pr_debug("%s\n", __func__);
        return 0;
}

static int32_t empty_common_alloc_digital_gain(uint32_t chn, int32_t *gain_ptr, uint32_t gain_num)
{
        pr_debug("%s\n", __func__);
        return 0;
}

static int32_t empty_common_alloc_integration_time(uint32_t chn, uint32_t *int_time,
    uint32_t *int_time_M, uint32_t *int_time_L)
{
        pr_debug("%s\n", __func__);
        return 0;
}


static int32_t empty_common_update(uint32_t chn, int32_t effect)
{
        pr_debug("%s\n", __func__);
        return 0;
}


static uint32_t empty_common_read_register(uint32_t chn, uint32_t address)
{
        pr_debug("%s\n", __func__);
        return 0;
}

static int32_t empty_common_write_register(uint32_t chn, uint32_t address, uint32_t data)
{
        pr_debug("%s\n", __func__);
        return 0;
}


static int32_t empty_common_get_param(uint32_t chn, struct _setting_param_t *user_para)
{
        pr_debug("%s\n", __func__);
        return 0;
}

static int32_t empty_common_awb_param(uint32_t chn, uint32_t rgain, uint32_t bgain,
                uint32_t grgain, uint32_t gbgain, uint32_t temper)
{
        pr_debug("%s\n", __func__);
        return 0;
}

static int32_t empty_sensor_get_base_info(uint32_t chn, struct isi_sensor_base_info_s *user_para)
{
        pr_debug("%s\n", __func__);
        return 0;
}

static int32_t empty_sensor_get_analog_gain(uint32_t chn, struct isi_sensor_again_info_s *user_again)
{
        pr_debug("%s\n", __func__);
        return 0;
}

static int32_t empty_sensor_get_digital_gain(uint32_t chn, struct isi_sensor_dgain_info_s *user_dgain)
{
        pr_debug("%s\n", __func__);
        return 0;
}

static int32_t empty_sensor_get_integration_time(uint32_t chn, struct isi_sensor_line_info_s *user_line)
{
        pr_debug("%s\n", __func__);
        return 0;
}

static int32_t empty_sensor_get_awb_para(uint32_t chn, struct isi_sensor_awb_info_s *user_awb)
{
        pr_debug("%s\n", __func__);
        return 0;
}

static int32_t empty_sensor_set_cali_name(uint32_t chn, char *cali_name)
{
        pr_debug("%s\n", __func__);
        return 0;
}

struct sensor_isi_ops_s g_isi_sensor_cops = {
        .sensor_alloc_analog_gain = empty_common_alloc_analog_gain,
        .sensor_alloc_digital_gain = empty_common_alloc_digital_gain,
        .sensor_alloc_integration_time = empty_common_alloc_integration_time,
        .sensor_get_analog_gain = empty_sensor_get_analog_gain,
        .sensor_get_digital_gain = empty_sensor_get_digital_gain,
        .sensor_get_integration_time = empty_sensor_get_integration_time,
        .sensor_update = empty_common_update,
        .read_register = empty_common_read_register,
        .write_register = empty_common_write_register,
        .sensor_get_para = empty_common_get_param,
        .sensor_get_base_info = empty_sensor_get_base_info,
        .sensor_awb_para = empty_common_awb_param,
        .sensor_get_awb_para = empty_sensor_get_awb_para,
	.sensor_set_cali_name = empty_sensor_set_cali_name,
        .end_magic = SENSOR_OPS_END_MAGIC,
};

/**
 * @brief camera sensor param show of sysfs
 *
 * @param[in] dev: the sensor device struct
 * @param[in] attr: the sysfs attr struct
 * @param[out] buf: the buffer to show string store
 *
 * @return >=0:Success-string length, <0:Failure
 */
static ssize_t sensor_param_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        pr_debug("sensor_param_show \n");
        return 0;
}

/**
 * @brief camera sensor param store of sysfs
 *
 * @param[in] dev: the sensor device struct
 * @param[in] attr: the sysfs attr struct
 * @param[in] buf: the buffer to store string
 * @param[in] count: the buffer string length
 *
 * @return >0:Success-the count, <0:Failure
 */
static ssize_t sensor_param_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        pr_debug("sensor_param_store \n");
        return 0;
}

/* sysfs for sensor devices' param */
static DEVICE_ATTR(isi_debug, (S_IWUSR | S_IRUGO), sensor_param_show, sensor_param_store);

static struct attribute *param_attr[] = {
        &dev_attr_isi_debug.attr,
        NULL,
};

static const struct attribute_group param_attr_group = {
        .name = __stringify(param),
        .attrs = param_attr,
};

/* sysfs attr groups */
static const struct attribute_group *attr_groups[] = {
        &param_attr_group,
        // &status_attr_group,
        NULL,
};

/**
 * @brief camera sensor device file operation: open
 *
 * @param[in] pinode: file node point
 * @param[in] pfile: file point
 *
 * @return 0:Success, <0:Failure
 */
static int32_t isi_sensor_fop_open(struct inode *pinode, struct file *pfile)
{
        pr_debug("isi_sensor_fop_open \n");
        return 0;
}
/**
 * @brief camera sensor device file operation: release
 *
 * @param[in] pinode: file node point
 * @param[in] pfile: file point
 *
 * @return 0:Success, <0:Failure
 */
static int32_t isi_sensor_fop_release(struct inode *pinode, struct file *pfile)
{
        pr_debug("%s\n", __func__);
        return 0;
}

static int32_t isi_snesor_wirte_reg(struct isi_sensor_reg_s *reg_info)
{
        int32_t ret = 0;

        if (g_isi_sen == NULL)
                return -EINVAL;

        if (g_isi_sen->isi_sensor_cops != NULL) {
                ret = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->
                write_register(reg_info->chn, reg_info->address, reg_info->data);
                if (ret != 0) {
                        pr_err("isi_snesor_wirte_reg error, ret = %d\n", ret);
                }
        } else {
                pr_err("isi_sensor_cops is NULL \n");
                ret = -EINVAL;
        }

        return ret;
}

/*
 *  reg width should be setted in sensor driver, and now it support 8\16\32 width
 * */
static uint32_t isi_sensor_read_reg(struct isi_sensor_reg_s *reg_info)
{
        uint32_t value = 0;

        if (g_isi_sen == NULL)
                return value;

        pr_debug("isi_sensor_read_reg \n");

        if (g_isi_sen->isi_sensor_cops != NULL) {
                value = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->
                read_register(reg_info->chn, reg_info->address);
        } else {
                pr_err("isi_sensor_cops is NULL \n");
        }

        return value;
}

static int32_t isi_sensor_get_base_info(uint32_t chn)
{
        int32_t ret = 0;

        if (g_isi_sen == NULL)
                return -EINVAL;

        pr_debug("isi_sensor_get_base_info \n");

        if (chn >= CAMERA_TOTAL_NUMBER || chn < 0) {
                pr_err("%s chn %d is error \n", __func__, chn);
                return -1;
        }

        if (g_isi_sen->isi_sensor_cops != NULL) {
                ret = ((struct sensor_isi_ops_s *)(g_isi_sen->isi_sensor_cops->cops))->
                sensor_get_base_info(chn, &g_sensor_base_info[chn]);
                if (ret != 0) {
                        pr_err("%s, g_sensor_base_info update failed \n",__func__);
                        return ret;
                } else {
                        pr_debug("%s, g_sensor_base_info update success \n", __func__);
                        g_base_info_updated = true;
                }
        } else {
                pr_err("isi_sensor_cops is NULL \n");
        }

        return ret;
}

/**
 * @brief camera sensor file operation: ioctl
 *
 * @param[in] pfile: file point
 * @param[in] cmd: ioctl cmd
 * @param[in] arg: ioctl arg
 */
static long isi_sensor_fop_ioctl(struct file *pfile, uint32_t cmd, unsigned long arg)
{
        int ret = 0;
        uint32_t chn_number = 0;

        struct isi_sensor_reg_s reg_info = {0};

        struct isi_sensor_base_info_s base_info = {0};
        struct isi_sensor_setting_param_s setting_param = {0};

        struct isi_sensor_again_param_s again_param = {0};
        struct isi_sensor_dgain_param_s dgain_param = {0};
        struct isi_sensor_line_param_s line_param = {0};
        struct isi_sensor_awb_param_s awb_param = {0};

	camera_calib_t calib_param = {0};

        switch (cmd) {
                case ISI_SENSOR_IOCTL_READ_PORT:
                        chn_number = camera_isi_sensor_chn_number();
                        if (chn_number < 0 || chn_number >= CAMERA_TOTAL_NUMBER) {
                                pr_err("%s %d chn_number is error \n", __func__, cmd);
                                return -EIO;
                        }
                        if (copy_to_user((void __user *)arg, (void *)&chn_number, sizeof(uint32_t))) {
                                pr_err("%s %d arg send error \n", __func__, cmd);
                                ret = -EIO;
                        }
                        break;
                case ISI_SENSOR_IOCTL_READ_BASE_INFO:
                        if (copy_from_user((void *)&base_info, (void __user *)arg, sizeof(struct isi_sensor_base_info_s))) {
                                pr_err("%s %d arg copy error \n",__func__, cmd);
                                ret = -EIO;
                        } else {
                                ret = isi_sensor_get_base_info(base_info.chn);
                                if (ret < 0) {
                                        pr_err("%s %d arg handle error \n", __func__, cmd);
                                        return -EIO;
                                }
                                if (copy_to_user((void __user *)arg, (void *)&g_sensor_base_info[base_info.chn],
                                        sizeof(struct isi_sensor_base_info_s))) {
                                        pr_err("%s %d data send error \n", __func__, cmd);
                                        ret = -EIO;
                                }
                        }
                        break;
                case ISI_SENSOR_IOCTL_GET_SETTING_PARAM:
                       if (copy_from_user((void *)&setting_param, (void __user *)arg, sizeof(struct isi_sensor_setting_param_s))) {
                                pr_err("%s %d arg copy error \n",__func__, cmd);
                                ret = -EIO;
                        } else {
                                ret = isi_get_sensor_setting_param(setting_param.chn);
                                if (ret < 0) {
                                    pr_err("%s %d arg handle error \n", __func__, cmd);
                                    return -EIO;
                                }
                                if (copy_to_user((void __user *)arg, (void *)&g_sensor_setting_param[setting_param.chn],
                                        sizeof(struct isi_sensor_setting_param_s))) {
                                        pr_err("%s %d data send error \n", __func__, cmd);
                                        ret = -EIO;
                                }
                        }
                        break;
                case ISI_SENSOR_IOCTL_READ_REG:
                        if (copy_from_user((void*)&reg_info, (void __user *)arg, sizeof(struct isi_sensor_reg_s))) {
                                pr_err("%s arg copy error \n", __func__);
                                ret = -EIO;
                        } else {
                                reg_info.data = isi_sensor_read_reg(&reg_info);
                                if(reg_info.data < 0) {
                                    pr_err("%s %d arg handle error \n", __func__, cmd);
                                    return -EIO;
                                }
                                if (copy_to_user((void __user *)arg, (void *)&reg_info, sizeof(struct isi_sensor_reg_s))) {
                                        pr_err("%s data send error \n", __func__);
                                        ret = -EIO;
                                }
                        }
                        break;
                case ISI_SENSOR_IOCTL_WRITE_REG:
                        if (copy_from_user((void *)&reg_info, (void __user *)arg, sizeof(struct isi_sensor_reg_s))) {
                                pr_err("%s %d arg copy error \n", __func__, cmd);
                                ret = -EIO;
                        } else {
                                ret = isi_snesor_wirte_reg(&reg_info);
                                if (ret < 0) {
                                        pr_err("%s %d handle error \n", __func__, cmd);
                                        ret = -EIO;
                                }
                        }
                        break;
                case ISI_SENSOR_IOCTL_GET_AGAIN:
                        if (copy_from_user((void *)&again_param, (void __user *)arg, sizeof(struct isi_sensor_again_param_s))) {
                                pr_err("%s %d arg copy error \n",__func__, cmd);
                                ret = -EIO;
                        } else {
                                ret = isi_get_sensor_again_param(again_param.chn);
                                if (ret < 0) {
                                    pr_err("%s %d arg handle error \n", __func__, cmd);
                                    return -EIO;
                                }
                                if (copy_to_user((void __user *)arg, (void *)&g_sensor_again_param[again_param.chn],
                                        sizeof(struct isi_sensor_again_param_s))) {
                                        pr_err("%s %d data send error \n", __func__, cmd);
                                        ret = -EIO;
                                }
                        }
                        break;
                case ISI_SENSOR_IOCTL_SET_AGAIN:
                       if (copy_from_user((void *)&again_param, (void __user *)arg, sizeof(struct isi_sensor_again_param_s))) {
                                pr_err("%s %d arg copy error \n",__func__, cmd);
                                ret = -EIO;
                        } else {
                                isi_set_sensor_again_param(&again_param);
                        }
                        break;
                case ISI_SENSOR_IOCTL_GET_DGAIN:
                       if (copy_from_user((void *)&dgain_param, (void __user *)arg, sizeof(struct isi_sensor_dgain_param_s))) {
                                pr_err("%s %d arg copy error \n",__func__, cmd);
                                ret = -EIO;
                        } else {
                                ret = isi_get_sensor_dgain_param(dgain_param.chn);
                                if (ret < 0) {
                                    pr_err("%s %d arg handle error \n", __func__, cmd);
                                    return -EIO;
                                }
                                if (copy_to_user((void __user *)arg, (void *)&g_sensor_dgain_param[dgain_param.chn],
                                        sizeof(struct isi_sensor_dgain_param_s))) {
                                        pr_err("%s %d data send error \n", __func__, cmd);
                                        ret = -EIO;
                                }
                        }
                        break;
                case ISI_SENSOR_IOCTL_SET_DGAIN:
                        if (copy_from_user((void *)&dgain_param, (void __user *)arg, sizeof(struct isi_sensor_dgain_param_s))) {
                                pr_err("%s %d arg copy error \n",__func__, cmd);
                                ret = -EIO;
                        } else {
                                isi_set_sensor_dgain_param(&dgain_param);
                        }
                        break;
                case ISI_SENSOR_IOCTL_GET_LINE:
                        if (copy_from_user((void *)&line_param, (void __user *)arg, sizeof(struct isi_sensor_line_param_s))) {
                                pr_err("%s %d arg copy error \n",__func__, cmd);
                                ret = -EIO;
                        } else {
                                ret = isi_get_sensor_line_param(line_param.chn);
                                if (ret < 0) {
                                    pr_err("%s %d arg handle error \n", __func__, cmd);
                                    return -EIO;
                                }
                                if (copy_to_user((void __user *)arg, (void *)&g_sensor_line_param[line_param.chn],
                                        sizeof(struct isi_sensor_line_param_s))) {
                                        pr_err("%s %d data send error \n", __func__, cmd);
                                        ret = -EIO;
                                }
                        }
                        break;
                case ISI_SENSOR_IOCTL_SET_LINE:
                        if (copy_from_user((void *)&line_param, (void __user *)arg, sizeof(struct isi_sensor_line_param_s))) {
                                pr_err("%s %d arg copy error \n",__func__, cmd);
                                ret = -EIO;
                        } else {
                                isi_set_sensor_line_param(&line_param);
                        }
                        break;
                case ISI_SENSOR_IOCTL_GET_AWB:
                        if (copy_from_user((void *)&awb_param, (void __user *)arg, sizeof(struct isi_sensor_awb_param_s))) {
                                pr_err("%s %d arg copy error \n",__func__, cmd);
                                ret = -EIO;
                        } else {
                                ret = isi_get_sensor_awb_param(awb_param.chn);
                                if (ret < 0) {
                                    pr_err("%s %d arg handle error \n", __func__, cmd);
                                    return -EIO;
                                }
                                if (copy_to_user((void __user *)arg, (void *)&g_sensor_awb_param[awb_param.chn],
                                        sizeof(struct isi_sensor_awb_param_s))) {
                                        pr_err("%s %d data send error \n", __func__, cmd);
                                        ret = -EIO;
                                }
                        }
                        break;
                case ISI_SENSOR_IOCTL_SET_AWB:
                        if (copy_from_user((void *)&awb_param, (void __user *)arg, sizeof(struct isi_sensor_awb_param_s))) {
                                pr_err("%s %d arg copy error \n",__func__, cmd);
                                ret = -EIO;
                        } else {
                                isi_set_sensor_awb_param(&awb_param);
                        }
                        break;
		case ISI_SENSOR_IOCTL_GET_LNAME:
			if (copy_from_user((void *)&calib_param, (void __user *)arg, sizeof(camera_calib_t))) {
				pr_err("%s %d arg copy error \n",__func__, cmd);
				ret = -EIO;
                        } else {
				pr_debug("%s port = %d, name = %s \n", __func__, calib_param.port, calib_param.name);
				isi_set_sensor_cali_name(calib_param.port, calib_param.name);
			}
			break;
                default:
                    pr_err("ISI sensor ioctl cmd 0x%x not supported\n", cmd);
                    break;
            }

        pr_debug("isi_sensor_fop_ioctl \n");
        return ret;
}


/**
 * isi sensor driver file operation functions
 */
const struct file_operations isi_sensor_fops = {
        .owner = THIS_MODULE,
        .open = isi_sensor_fop_open,
        .release = isi_sensor_fop_release,
        .unlocked_ioctl = isi_sensor_fop_ioctl,
        .compat_ioctl = isi_sensor_fop_ioctl,
};

static int32_t __init isi_sensor_init(void)
{
        int32_t ret = 0;
        struct _isi_sensor_t *isi;

        isi = kzalloc(sizeof(struct _isi_sensor_t), GFP_KERNEL);
        if (isi == NULL) {
                pr_err("kzalloc isi sensor device error \n");
                return -ENOMEM;
        }

        snprintf(isi->isi_sensor_miscdev.name, ISI_SENSOR_NAME_LEN_MAX, "%s", ISI_SENSOR_DEV_NAME);
        isi->isi_sensor_miscdev.miscdev.name = isi->isi_sensor_miscdev.name;
        isi->isi_sensor_miscdev.miscdev.minor = MISC_DYNAMIC_MINOR;
        isi->isi_sensor_miscdev.miscdev.fops = &isi_sensor_fops;
        isi->isi_sensor_miscdev.miscdev.groups = attr_groups;

        ret = misc_register(&isi->isi_sensor_miscdev.miscdev);
        if (ret < 0) {
                pr_err("isi sensor misc_register failed %d\n", ret);
                goto init_error_devexit;
        }

        isi->isi_sensor_cops = vio_get_callback_ops(&g_isi_sensor_cops, VIN_MODULE, COPS_8);

        g_isi_sen = isi;

        pr_info("isi_sensor_init end\n");

	return ret;

init_error_devexit:
        kfree(isi);

        return ret;
}

static void __exit isi_sensor_exit(void)
{
        struct _isi_sensor_t *isi = g_isi_sen;

        misc_deregister(&isi->isi_sensor_miscdev.miscdev);
        kfree(isi);

        pr_info("isi_sensor_exit end\n");
}

late_initcall_sync(isi_sensor_init);
module_exit(isi_sensor_exit);
MODULE_AUTHOR("liu xukuai<xukuai.liu@horizon.cc>");
MODULE_DESCRIPTION("hobot isi dev sensor driver");
MODULE_LICENSE("GPL");
