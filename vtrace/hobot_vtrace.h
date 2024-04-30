#ifndef __VIO_TRACE_H__
#define __VIO_TRACE_H__

#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/mod_devicetable.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include "osal.h"
#include "../vpf/vio_cops_api.h"
#include "vio_node_api.h"

#define uint32_t 		unsigned int
#define int32_t 		int
#define VTRACE_NAME 		"hobot_vtrace"
#define VTRACE_NAME_LEN_MAX	32
#define VTRACE_SYSFS_SIZE_MAX	128
#define VTRACE_SNP_MAX_SIZE	128
#define VTRACE_PING_PONG_SIZE	PAGE_SIZE
#define VTARCE_MEM_SIZE		VTRACE_PING_PONG_SIZE*2
#define VTRACE_TEMP_BUF_SIZE	384
#define VTRACE_SAFE_SIZE	50
#define VTRACE_ALL		0xFF
#define VTRACE_TIMEOUT		msecs_to_jiffies(3000)
#define VTRACE_COPS_MAGIC	0x1234

// vtrace pr
#define vtrace_fmt(fmt)			"[HOBOT_VTRACE] (%s_%d): " fmt
#define vtrace_pr_warp(p_func_, fmt, ...) do { \
		p_func_(vtrace_fmt(fmt), __func__, __LINE__, ##__VA_ARGS__); \
	} while(0)
#define vtrace_debug(fmt, ...)	vtrace_pr_warp(osal_pr_debug, fmt, ##__VA_ARGS__)
#define vtrace_info(fmt, ...)	vtrace_pr_warp(osal_pr_info, fmt, ##__VA_ARGS__)
#define vtrace_warn(fmt, ...)	vtrace_pr_warp(osal_pr_warn, fmt, ##__VA_ARGS__)
#define vtrace_err(fmt, ...)	vtrace_pr_warp(osal_pr_err, fmt, ##__VA_ARGS__)


#define VTRACE_MODULE_NAME { \
	"VIN_MODULE", \
	"ISP_MODULE", \
	"VNODE_ID_ISP1", \
	"VNODE_ID_YNR", \
	"VNODE_ID_PYM0", \
	"VNODE_ID_PYM1", \
	"VNODE_ID_PYM2", \
	"VNODE_ID_GDC", \
	"VNODE_ID_STH", \
	"VNODE_ID_CODEC", \
	"VNODE_ID_IDU0", \
	"VNODE_ID_IDU1", \
}

// public param
#define VTRACE_PUBLIC_TEST	0u
#define VTRACE_PUBLIC_FS	1u
#define VTRACE_PUBLIC_FE	2u
#define VTRACE_PUBLIC_DROP	3u
#define VTRACE_PUBTYPE_MAX	10u
#define VTRACE_PARAM_MAX	32u
#define VTRACE_PUBLIC_NAME { \
	"PUBLIC_TEST", \
	"PUBLIC_FS", \
	"PUBLIC_FE", \
	"PUBLIC_DROP", \
}

// 0. public test info
#define PUBLIC_TEST_PARAM_NAME { \
	"test_info", \
}

typedef struct vtrace_public_test {
	uint32_t testinfo;
} vtrace_public_test_s;
#define PUBLIC_TEST_PARAM_NUM	(sizeof(vtrace_public_test_s)/sizeof(uint32_t))

// 1. public frame start
#define PUBLIC_FS_PARAM_NAME { \
	"timestamp", \
}

typedef struct vtrace_public_fs {
	uint32_t timestamp;
} vtrace_public_fs_s;
#define PUBLIC_FS_PARAM_NUM	(sizeof(vtrace_public_fs_s)/sizeof(uint32_t))

// 2. public frame end
#define PUBLIC_FE_PARAM_NAME { \
	"timestamp", \
}

typedef struct vtrace_public_fe {
	uint32_t timestamp;
} vtrace_public_fe_s;
#define PUBLIC_FE_PARAM_NUM	(sizeof(vtrace_public_fe_s)/sizeof(uint32_t))

// 3. public frame start
#define PUBLIC_DROP_PARAM_NAME { \
	"timestamp", \
}

typedef struct vtrace_public_drop {
	uint32_t timestamp;
} vtrace_public_drop_s;
#define PUBLIC_DROP_PARAM_NUM	(sizeof(vtrace_public_drop_s)/sizeof(uint32_t))

typedef struct vtrace_zone_info {
	char (*name)[VTRACE_NAME_LEN_MAX];
	uint32_t param_nums;
	uint32_t level;
} vtrace_zone_info_s;

typedef struct vtrace_buf_head {
	vtrace_zone_info_s zone[VTRACE_PARAM_MAX];
	bool is_init[VTRACE_PARAM_MAX];
} vtrace_buf_head_s;

typedef enum vtrace_debug_level {
	LEVEL0 = 0,
	LEVEL1,
	LEVEL2,
} vtrace_debug_level_e;

typedef struct vtrace_cdev {
	dev_t devt;
	int major;
	int minor;
	struct cdev cdev;
	struct device *dev;
	char name[16];
} vtrace_cdev_s;

typedef struct vtrace_ctx {
	vtrace_cdev_s vcdev;
	uint32_t hal_on;		/**< whether hal is turn on */
	uint32_t new_data_comein;	/**< there is new message send to vtrace */
	void *base;			/**< the only base of pingpong buffer */
	osal_atomic_t cur_off;		/**< current offset to the base */
	int32_t pp_flag;		/**< which buffer is in use, 0-ping, 1-pong */
	uint32_t message_valid;		/**< means whether has message valid can be read */
	uint32_t halused_mask;		/**< 2bit mask, means which buffer is using by hal */
	uint32_t message_end;		/**< used by HAL to sign the end of valid message */
	uint32_t error_cnt;		/**< ++ when one message droped */
	uint32_t pop_swap;		/**< turn to 1 if in pop, to swap buffer */
	vtrace_buf_head_s buf_head[VNODE_ID_MAX];
	wait_queue_head_t irq_wait;
} vtrace_ctx_s;

typedef struct vtrace_ctl_ctx {
	uint32_t vtrace_on;
	uint32_t flowid_mask;
	uint32_t ctxid_mask;
	uint32_t module_mask;
	uint32_t level;
} vtrace_ctl_ctx_s;

int32_t vtrace_register(uint32_t module_type, uint32_t param_type, char **param_name,
			uint32_t param_nums, uint32_t level);
int32_t vtrace_send(uint32_t module_type, uint32_t param_type, uint32_t *param,
		    uint32_t flow_id, uint32_t frame_id, uint32_t ctx_id, uint32_t chnid);

#define VTRACE_CDEV_MAGIC	'V'
#define VTRACE_MODULE_MASK_SET	_IOW(VTRACE_CDEV_MAGIC, 0x13, unsigned int)
#define VTRACE_FLOWID_MASK_SET	_IOW(VTRACE_CDEV_MAGIC, 0x14, unsigned int)
#define VTRACE_CTXID_MASK_SET	_IOW(VTRACE_CDEV_MAGIC, 0x15, unsigned int)
#define VTRACE_LEVEL_SET	_IOW(VTRACE_CDEV_MAGIC, 0x16, unsigned int)
#define VTARCE_MEM_SIZE_GET	_IOW(VTRACE_CDEV_MAGIC, 0x17, unsigned int)
#define VTRACE_READ_START	_IOW(VTRACE_CDEV_MAGIC, 0x19, unsigned int)
#define VTRACE_READ_END		_IOW(VTRACE_CDEV_MAGIC, 0x20, unsigned int)
#define VTRACE_TIMEOUT_POP	_IOW(VTRACE_CDEV_MAGIC, 0x21, unsigned int)
#define VTRACE_TIMEOUT_POP_END	_IOW(VTRACE_CDEV_MAGIC, 0x22, unsigned int)
#define VTRACE_TEST_SEND	_IOW(VTRACE_CDEV_MAGIC, 0x23, unsigned int)

#endif
