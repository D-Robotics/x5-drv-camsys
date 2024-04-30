/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef HOBOT_ISI_DEV_SENSOR_H
#define HOBOT_ISI_DEV_SENSOR_H

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include "osal_list.h"
#include "common_camera.h"
#include "vio_framemgr.h"
#include "vio_node_api.h"
//#include "hobot_sensor_ops.h"
#include "camera_subdev.h"
#include "camera_dev.h"

/**
 * @def ISI_SENSOR_DEV_NAME_LEN
 * sensor dev name string length
 */
#define ISI_SENSOR_DEV_NAME_LEN (32)

/**
 * @def ISI_SENSOR_NAME_LEN_MA
 * sensor chip name string length
 */
#define ISI_SENSOR_NAME_LEN_MAX (32)

#define ISI_LINEAR_PARAS               0
#define ISI_DUAL_EXP_L_PARAS           0
#define ISI_DUAL_EXP_S_PARAS           1


struct isi_sensor_reg_s {
        uint32_t chn;
        uint32_t address;
        uint32_t data;
};


struct isi_sensor_setting_param_s {
        uint32_t chn;
        struct _setting_param_t setting_param_t;
};


struct isi_sensor_again_param_s {
        uint32_t chn;
        struct isi_sensor_again_info_s isi_sensor_again_info;
};

struct isi_sensor_dgain_param_s {
        uint32_t chn;
        struct isi_sensor_dgain_info_s isi_sensor_dgain_info;
};

struct isi_sensor_line_param_s {
        uint32_t chn;
        struct isi_sensor_line_info_s isi_sensor_line_info;
};

struct isi_sensor_awb_param_s {
        uint32_t chn;
        struct isi_sensor_awb_info_s isi_sensor_awb_info;
};

/**
 * @def ISI_SENSOR_DEV_NAME
 * sensor device name string
 */
#define ISI_SENSOR_DEV_NAME "isi_sensor"


#define ISI_SENSOR_IOC_MAGIC      's'
// 接了几路 sensor
#define ISI_SENSOR_IOCTL_READ_PORT  _IOR(ISI_SENSOR_IOC_MAGIC, 0, int32_t)
// 获取 对应 chn 的 sensor 基本信息，name、mode、分辨率、bitwidth 等
#define ISI_SENSOR_IOCTL_READ_BASE_INFO _IOWR(ISI_SENSOR_IOC_MAGIC, 1, struct isi_sensor_base_info_s)
// 获取tuning 相关的一些 setting 数据
#define ISI_SENSOR_IOCTL_GET_SETTING_PARAM _IOWR(ISI_SENSOR_IOC_MAGIC, 2, struct isi_sensor_setting_param_s)
// 读寄存器
#define ISI_SENSOR_IOCTL_READ_REG _IOWR(ISI_SENSOR_IOC_MAGIC, 3, struct isi_sensor_reg_s)
// 写寄存器
#define ISI_SENSOR_IOCTL_WRITE_REG _IOW(ISI_SENSOR_IOC_MAGIC, 4, struct isi_sensor_reg_s)
// 获取A gain
#define ISI_SENSOR_IOCTL_GET_AGAIN _IOWR(ISI_SENSOR_IOC_MAGIC, 5, struct isi_sensor_again_param_s)
// 设置A gain
#define ISI_SENSOR_IOCTL_SET_AGAIN _IOW(ISI_SENSOR_IOC_MAGIC, 6, struct isi_sensor_again_param_s)
// 获取D gain
#define ISI_SENSOR_IOCTL_GET_DGAIN _IOWR(ISI_SENSOR_IOC_MAGIC, 7, struct isi_sensor_dgain_param_s)
// 设置D gain
#define ISI_SENSOR_IOCTL_SET_DGAIN _IOW(ISI_SENSOR_IOC_MAGIC, 8, struct isi_sensor_dgain_param_s)
// 获取Int time/line
#define ISI_SENSOR_IOCTL_GET_LINE _IOWR(ISI_SENSOR_IOC_MAGIC, 9, struct isi_sensor_line_param_s)
// 设置Int time/line
#define ISI_SENSOR_IOCTL_SET_LINE _IOW(ISI_SENSOR_IOC_MAGIC, 10, struct isi_sensor_line_param_s)
// 获取 AWB
#define ISI_SENSOR_IOCTL_GET_AWB _IOWR(ISI_SENSOR_IOC_MAGIC, 11, struct isi_sensor_awb_param_s)
// 设置 AWB
#define ISI_SENSOR_IOCTL_SET_AWB _IOW(ISI_SENSOR_IOC_MAGIC, 12, struct isi_sensor_awb_param_s)


/**
 * @struct isi_sensor_miscdev_s
 * sensor misc device struct for hw operation
 */
struct isi_sensor_miscdev_s {
        char name[ISI_SENSOR_DEV_NAME_LEN];
        int32_t minor_id;
        struct miscdevice miscdev;
};

struct _isi_sensor_t {
        struct isi_sensor_miscdev_s isi_sensor_miscdev;
        struct vio_callback_ops *isi_sensor_cops;  // isi sensor ops func
};

#endif/*HOBOT_ISI_DEV_SENSOR_H*/
