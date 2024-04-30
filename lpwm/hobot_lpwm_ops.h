/*************************************************************
  ****                    COPYRIGHT NOTICE
  ****            Copyright      2020 Horizon Robotics, Inc.
  ****                    All rights reserved.
  *************************************************************/

#ifndef HOBOT_LPWM_OPS_J6_H_
#define HOBOT_LPWM_OPS_J6_H_

#include <linux/cdev.h>
#include <linux/pwm.h>
#include "vin_node_config.h"
#include "hobot_lpwm_hw_reg.h"

#define LPWM_CLK_MUL_FACTOR	1000		/**< Factor to get us from ns */
#define LPWM_STREAM_ON		1
#define LPWM_STREAM_OFF		0

#define lpwm_check_and_return(lpwm, i, c) do { \
		if (LPWM_CHANNELID_CHECK(LPWM_TO_CHANNELID((i), (c)))) { \
			lpwm_err(NULL, "Id(%d-%d) check fail, confirm HW support!\n", (i), (c)); \
			(lpwm) = NULL; \
		} else { \
			(lpwm) = lpwm_ins_ptr(i); \
		} \
	} while(0)

#define lpwm_check_utype(lpwm, type) do { \
		if ((lpwm)->utype != (type)) { \
			lpwm_err(lpwm, "Camsys/Backlight cannot handle the same device! \
				 current utype: %d\n", lpwm->utype); \
			return -EPERM; \
		} \
	} while(0)

struct hobot_lpwm_cdev_config {
	uint32_t	channel_id;
	lpwm_attr_t	config;
};

struct hobot_lpwm_cdev_chg_config {
	uint32_t		channel_id;
	lpwm_dynamic_fps_t	config;
};

/* chardev Macro */
#define LPWM_CDEV_MAGIC			'L'
#define LPWM_CDEV_SIGNAL_CHECK		_IOR(LPWM_CDEV_MAGIC, 0x11, unsigned int)
#define LPWM_CDEV_INIT_ENABLE		_IOW(LPWM_CDEV_MAGIC, 0x12, struct hobot_lpwm_cdev_config)
#define LPWM_CDEV_DEINIT_DISABLE	_IOW(LPWM_CDEV_MAGIC, 0x13, unsigned int)
#define LPWM_CDEV_GET_STATE		_IOWR(LPWM_CDEV_MAGIC, 0x14, struct hobot_lpwm_cdev_config)
#define LPWM_CDEV_CHANGE_CONFIG		_IOW(LPWM_CDEV_MAGIC, 0x15, struct hobot_lpwm_cdev_chg_config)
#define LPWM_CDEV_RST			_IOW(LPWM_CDEV_MAGIC, 0x16, unsigned int)
#define LPWM_CDEV_PWM_REQUEST		_IOW(LPWM_CDEV_MAGIC, 0x17, unsigned int)
#define LPWM_CDEV_PWM_FREE		_IOW(LPWM_CDEV_MAGIC, 0x18, unsigned int)
#define LPWM_CDEV_PWM_APPLY		_IOWR(LPWM_CDEV_MAGIC, 0x19, unsigned int)

/* VPF VIN module common ops */
extern int32_t lpwm_open(struct vio_video_ctx *vctx);
extern int32_t lpwm_close(struct vio_video_ctx *vctx);
extern int32_t lpwm_init(struct vio_video_ctx *vctx, void *attr);
extern int32_t lpwm_get_attr(struct vio_video_ctx *vctx, void *attr);
extern int32_t lpwm_change_attr(struct vio_video_ctx *vctx, void *attr);
extern int32_t lpwm_start(struct vio_video_ctx *vctx);
extern int32_t lpwm_stop(struct vio_video_ctx *vctx);
extern int32_t lpwm_reset(struct vio_video_ctx *vctx);
/* Linux pwm ops */
extern int32_t hobot_lpwm_request(struct pwm_chip *chip, struct pwm_device *pwm);
extern void hobot_lpwm_free(struct pwm_chip *chip, struct pwm_device *pwm);
extern int32_t hobot_lpwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
				const struct pwm_state *state);
/* Linux char dev file ops */
extern int lpwm_chip_open(struct inode *pinode, struct file *pfile);
extern int lpwm_chip_release(struct inode *pinode, struct file *pfile);
extern long lpwm_chip_ioctl(struct file *pfile, unsigned int cmd, unsigned long args);


#endif // HOBOT_LPWM_OPS_J6_H_