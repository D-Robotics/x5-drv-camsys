/*************************************************************
  ****                    COPYRIGHT NOTICE
  ****            Copyright      2020 Horizon Robotics, Inc.
  ****                    All rights reserved.
  *************************************************************/

#ifndef HOBOT_LPWM_DEV_J6_H_
#define HOBOT_LPWM_DEV_J6_H_

#include <linux/cdev.h>
#include <linux/pwm.h>
#include <osal.h>
#include "hobot_vin_common.h"
#include "vin_node_config.h"

#define LPWM_NAME		"hobot-lpwm"	/**< Name of lpwm */
#define LPWM_CNUM		4		/**< Core number of lpwm */
#ifdef LPWM_J6E
#define LPWM_INUM		3		/**< Instance number of lpwm */
#else
#define LPWM_INUM		4		/**< Instance number of lpwm */
#endif
#define LPWM_ID_MAX 		(LPWM_INUM * LPWM_CNUM)
#define INS_ID(id)		((id) / LPWM_CNUM)
#define COR_ID(id)		((id) % LPWM_CNUM)
#define LPWM_TO_CHANNELID(i, c)	(((i) * LPWM_CNUM) + (c))
#define LPWM_CHANNELID_CHECK(c)	((c) >= LPWM_ID_MAX)

#define LPWM_ATTR_MAX_SIZE	256		/**< Max size to snprintf */
#define LPWM_RET_OK		0


#define lpwm_fmtw(fmt)			"[HOBOT_LPWM-%d] (%s): " fmt
#define lpwm_fmtn(fmt)			"[HOBOT_LPWM] (%s): " fmt
#define lpwm_pr_warp(p_func_, ins, fmt, ...) do { \
		if ((ins) == NULL) { \
			p_func_(lpwm_fmtn(fmt), __func__, ##__VA_ARGS__); \
		} else { \
			p_func_(lpwm_fmtw(fmt), \
				 (((struct hobot_lpwm_ins *)(ins))->dev_idx), \
				 __func__, ##__VA_ARGS__); \
		} \
	} while(0)
#define lpwm_debug(ins, fmt, ...)	lpwm_pr_warp(osal_pr_debug, ins, fmt, ##__VA_ARGS__)
#define lpwm_info(ins, fmt, ...)	lpwm_pr_warp(osal_pr_info, ins, fmt, ##__VA_ARGS__)
#define lpwm_warn(ins, fmt, ...)	lpwm_pr_warp(osal_pr_warn, ins, fmt, ##__VA_ARGS__)
#define lpwm_err(ins, fmt, ...)		lpwm_pr_warp(osal_pr_err, ins, fmt, ##__VA_ARGS__)

/**
 * @struct lpwm_chip_cdev
 * Define the descriptor of lpwm char device
 * @NO{S10E05C01}
 */
struct lpwm_chip_cdev {
	char			name[16];	/**< Name of lpwm */
	int			major;		/**< Major of devid */
	int			minor;		/**< Minor of devid */
	struct cdev		cdev;		/**< Struct of char dev */
	struct device		*dev;		/**< Pointer of dev */
	dev_t			devt;		/**< ID of dev */
	struct hobot_lpwm_ins	*lpwm;		/**< Pointer of lpwm instance */
};

/**
 * @enum chip_type
 * List the current type of lpwm
 * @NO{S10E05C01}
 */
typedef enum chip_type {
	INIT_STATE,
	CAMSYS,
	BACKLIGHT,
} chip_type_t;

/**
 * @struct hobot_lpwm_ins
 * Define the descriptor of lpwm instance device
 * @NO{S10E05C01}
 */
struct hobot_lpwm_ins {
	struct pwm_chip		chip;			/**< Struct of pwm chip dev */
	char			name[16];		/**< Name of lpwm */
	struct clk		*sclk;			/**< Pointer of lpwm sclock */
	struct clk		*pclk;			/**< Pointer of lpwm pclock */
	void __iomem		*base;			/**< The virtual address of base */
	lpwm_chn_attr_t		lpwm_attr[LPWM_CNUM];	/**< Struct of lpwm attr */
	struct pinctrl		*pinctrl;		/**< Pointer of pinctrl */
	struct pinctrl_state	*pins_default;		/**< Pointer of pinctrl state */
	struct lpwm_chip_cdev	*lpwm_cdev;		/**< Pointer of char dev */
	osal_atomic_t		rcount[LPWM_CNUM];	/**< request count */
	osal_atomic_t		enable_cnt[LPWM_CNUM];	/**< enable count */
	int32_t			irq;			/**< Interrupt id */
	int32_t			dev_idx;		/**< instance id */
	chip_type_t		utype;			/**< which sys occupy lpwm */
	osal_mutex_t		con_lock;		/**< protect lpwm_attr */
	struct hrtimer		swtrigger_timer;	/**< software trigger timer struct */
	struct vio_callback_ops	*cim_cops;		/**< callback ops register by cim */
};

extern inline struct hobot_lpwm_ins *lpwm_ins_ptr(uint32_t i_id);

#endif  // HOBOT_LPWM_DEV_J6_H_