/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_OSD_DEV_H__
#define __HOBOT_OSD_DEV_H__

#include <uapi/linux/types.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/types.h>

#include "vio_config.h"
#include "vio_framemgr.h"
#include "osd_config.h"
#include "hobot_osd_mem.h"
#include "hobot_osd_ops.h"
#include "hobot_osd_process.h"
#include "vio_cops_api.h"
#include "vio_node_api.h"

#define MODULE_NAME		"hobot_osd"
#define OSD_NAM			"osd"
#define OSD_MAX_DEVICE		1
#define OSD_HANDLE_MAX		256
#define OSD_HW_PROC_NUM		4
#define OSD_COLOR_NUM		15
#define OSD_LEVEL_NUM		4
#define MAX_STA_BIN_NUM		4
#define OSD_TASK_PRIORITY	34
#define OSD_COPS_MAGIC		0x1234
#define OSD_CHN_NAME {"osd_vse_ds0 ",\
		"osd_vse_ds1",\
		"osd_vse_ds2",\
		"osd_vse_ds3",\
		"osd_vse_ds4",\
		"osd_vse_us"}

#define OSD_IOC_MAGIC 'c'
#define OSD_IOC_CREATE_HANDLE           _IOW(OSD_IOC_MAGIC, 0, struct osd_handle_info)
#define OSD_IOC_DESTROY_HANDLE          _IOW(OSD_IOC_MAGIC, 1, int32_t)
#define OSD_IOC_GET_ATTR                _IOWR(OSD_IOC_MAGIC, 2, struct osd_handle_info)
#define OSD_IOC_SET_ATTR                _IOW(OSD_IOC_MAGIC, 3, struct osd_handle_info)
#define OSD_IOC_GET_BUF                 _IOWR(OSD_IOC_MAGIC, 4, struct osd_buffer_info)
#define OSD_IOC_SET_BUF                 _IOW(OSD_IOC_MAGIC, 5, struct osd_buffer_info)
#define OSD_IOC_ATTACH                  _IOW(OSD_IOC_MAGIC, 6, struct osd_bind_info)
#define OSD_IOC_DETACH                  _IOW(OSD_IOC_MAGIC, 7, struct osd_bind_info)
#define OSD_IOC_GET_BIND_ATTR           _IOWR(OSD_IOC_MAGIC, 8, struct osd_bind_info)
#define OSD_IOC_SET_BIND_ATTR           _IOW(OSD_IOC_MAGIC, 9, struct osd_bind_info)
#define OSD_IOC_STA                     _IOWR(OSD_IOC_MAGIC, 10, struct osd_sta_info)
#define OSD_IOC_STA_LEVEL               _IOWR(OSD_IOC_MAGIC, 11, struct osd_sta_info)
#define OSD_IOC_STA_BIN                 _IOWR(OSD_IOC_MAGIC, 12, struct osd_sta_bin_info)
#define OSD_IOC_COLOR_MAP               _IOWR(OSD_IOC_MAGIC, 13, struct osd_color_map)
#define OSD_IOC_PROC_BUF                _IOW(OSD_IOC_MAGIC, 14, struct osd_proc_buf_info)

#define MAX_OSD_NUM 4
#define MAX_STA_NUM 8
#define MAX_OSD_STA_LEVEL_NUM 3
#define MAX_OSD_COLOR_NUM 16

struct osd_box {
	u8 osd_en;
	u8 overlay_mode;
	u16 start_x;
	u16 start_y;
	u16 width;
	u16 height;
};

//for osd draw, Y info sta
struct osd_sta_box {
	u8 sta_en;
	u16 start_x;
	u16 start_y;
	u16 width;
	u16 height;
};

struct osd_color_map {
	u8 color_map_update;
	u32 color_map[MAX_OSD_COLOR_NUM];	//colour map buffer addr
};

// vse hw osd cfg
// todo: fix to vse hw cfg
struct vse_osd_cfg {
	bool osd_box_update;
	struct osd_box osd_box[MAX_OSD_NUM];
	bool osd_buf_update;
	u32 osd_buf[MAX_OSD_NUM];
	bool osd_sta_update;
	struct osd_sta_box osd_sta[MAX_STA_NUM];
	bool osd_sta_level_update;
	u8 osd_sta_level[MAX_OSD_STA_LEVEL_NUM];
	struct osd_color_map color_map;
	spinlock_t osd_cfg_slock;
};

enum osd_chn {
	OSD_VSE_DS0,
	OSD_VSE_DS1,
	OSD_VSE_DS2,
	OSD_VSE_DS3,
	OSD_VSE_DS4,
	OSD_VSE_US,
	OSD_CHN_MAX
};

enum yuv_buf_type {
	Y_UV_NON_CONTINUOUS_ALIGN_4K,
	Y_UV_CONTINUOUS_ALIGN
};

struct mp_vio_frame {
	struct vio_frame *vse_frame;
	struct list_head vse_entry;

	// ipu/pym multprocess share
	// support max 3 planar
	// phy addr for ion reserverd in xj3 is never greater than 32bit
	int32_t plane_count;
	u64 addr[3];
	struct ion_handle *ion_handle[3];

	// which process this vio_frame belongs to
	// the process alloc how may ion buffers and it's first indx in vio_framemgr
	// for multiprocess alloc/free tracking
	int32_t proc_id;
	int32_t first_indx;
	int32_t ion_bufffer_num;

	// if Y and UV are continous allocated
	// used only in ipu driver
	enum yuv_buf_type ion_alloc_type;
};

struct vio_osd_info {
	atomic_t need_sw_osd;
	uint32_t chn_id;
	uint32_t ctx_id;
	atomic_t frame_count;
	void (*return_frame)(struct vio_osd_info *osd_info, struct vio_frame *frame);
};

struct osd_interface_ops {
	void (*frame_process)(struct vio_osd_info *osd_info);
	void (*osd_set_info)(uint32_t chn_id, uint32_t ctx_id, struct vio_osd_info *info);
};

enum osd_sta_state {
	OSD_STA_NULL = 0,
	OSD_STA_REQUEST,
	OSD_STA_PROCESS,
	OSD_STA_DONE,
};

enum osd_task_state {
	OSD_TASK_NULL,
	OSD_TASK_START,
	OSD_TASK_REQUEST_STOP,
	OSD_TASK_STOP,
};

/****sync with hal****/
struct osd_handle_info {
	uint32_t handle_id;

	// for polygon
	uint32_t side_num;
	struct osd_point point[OSD_POLYGON_MAX_SIDE];
	uint32_t *polygon_buf;

	struct osd_size size;
	uint32_t fill_color;
	uint32_t alpha;
	// when enable and a pixel yuv color is key color, it will use origin image
	// 31-24: 0/1  enable/diable
	// 23-16: y value in yuv
	// 15- 8: u value in yuv
	// 7 - 0: v value in yuv
	uint32_t yuv_bg_transparent;   // yuv420 pixel format background transparent
	enum osd_process_type proc_type;
};

struct osd_buffer_info {
	int32_t handle_id;
	uint32_t share_id;
	// ping-pong index
	int32_t index;
	struct osd_size size;
	uint64_t paddr;
	uint8_t *vaddr;
	enum osd_process_type proc_type;
};

struct osd_bind_info {
	int32_t chn_id;
	int32_t ctx_id;
	int32_t handle_id;
	struct osd_handle_info handle_info;

	uint8_t show_en;
	uint8_t invert_en;
	// hw process level:0; sw process level :1-3
	uint32_t osd_level;
	// for pym buffer, osd in which layer
	int32_t buf_layer;
	struct osd_point start_point;
};

struct osd_sta_info {
	int32_t chn_id;
	int32_t ctx_id;
	int32_t buf_layer;
	uint8_t sta_level[MAX_OSD_STA_LEVEL_NUM];
	struct osd_sta_box sta_box[MAX_STA_NUM];
};

struct osd_sta_bin_info {
	int32_t chn_id;
	int32_t ctx_id;
	uint16_t sta_value[MAX_STA_NUM][MAX_STA_BIN_NUM];
};

struct osd_proc_buf_info {
	int32_t handle_id;
	uint8_t invert_en;
	struct osd_point start_point;
	struct osd_size size;
	uint32_t fill_color;
	uint32_t yuv_bg_transparent;   // yuv420 pixel format background transparent
	uint32_t image_width;
	uint32_t image_height;
	uint64_t y_paddr;
	uint32_t y_length;
	uint64_t uv_paddr;
	uint32_t uv_length;
	uint32_t *polygon_buf;
	enum osd_process_type proc_type;
};
/****sync with hal end****/


struct osd_handle {
	struct list_head node;
	struct osd_handle_info info;
	struct osd_buffer buffer;
	atomic_t bind_cnt;
	atomic_t ref_cnt;
};

struct osd_bind {
	struct list_head node;
	struct osd_bind_info bind_info;
	struct osd_process_info proc_info;
	atomic_t need_update;
	atomic_t ref_cnt;
};

struct osd_sta {
	enum osd_sta_state sta_state;
	uint32_t enable_index;
	int32_t buf_layer;
	uint8_t sta_level[MAX_OSD_STA_LEVEL_NUM];
	struct osd_sta_box sta_box[MAX_STA_NUM];
	volatile uint16_t sta_value[MAX_STA_NUM][MAX_STA_BIN_NUM];
	struct osd_process_info sta_proc[MAX_STA_NUM];
	struct kthread_work work;
};

struct osd_video_ctx{
	struct osd_dev *osd_dev;
};

struct osd_frame {
	void *frame;
	struct list_head node;
};

struct osd_frame_queue {
	osal_spinlock_t lock;
	struct osd_frame *frames;
	struct list_head incoming_list;
	struct list_head process_list;
	struct list_head request_list;
};

struct osd_subdev {
	int32_t chn_id;
	int32_t ctx_id;
	struct osd_frame_queue queue;
	struct vse_osd_cfg *osd_hw_cfg;
	struct osd_sta osd_sta;
	struct list_head bind_list;
	struct mutex bind_mutex;
	struct mutex sta_mutex;
	struct osd_dev *osd_dev;
	struct vio_osd_info *osd_info;
	uint32_t osd_hw_limit_y;
	atomic_t osd_hw_need_update;
	atomic_t osd_hw_cnt;
	struct list_head input_frame_list;
	osal_spinlock_t frame_slock;
	struct kthread_work work;
};

struct osd_dev {
	atomic_t open_cnt;
	struct class *class;
	struct cdev cdev;
	dev_t devno;
	struct kthread_work work;
	struct kthread_worker worker;
	struct task_struct *task;
	enum osd_task_state task_state;
	struct ion_client *ion_client;
	struct osd_subdev subdev[OSD_CHN_MAX][VIO_MAX_STREAM];
	struct list_head osd_list;
	struct mutex osd_list_mutex;
	struct mutex osd_mutex;
};

#define osd_fmt(fmt) "[HOBOT_OSD](%s) " fmt
#define osd_pr_warp(p_func_, str, fmt, ...) do { \
		p_func_(str osd_fmt(fmt), __func__, ##__VA_ARGS__); \
	} while(0)
#define osd_debug(fmt, ...)	osd_pr_warp(pr_debug, "[DBG]", fmt, ##__VA_ARGS__)
#define osd_info(fmt, ...)	osd_pr_warp(pr_info, "[INFO]", fmt, ##__VA_ARGS__)
#define osd_warn(fmt, ...)	osd_pr_warp(pr_warn, "[WARN]", fmt, ##__VA_ARGS__)
#define osd_err(fmt, ...)	osd_pr_warp(pr_err, "[ERR]", fmt, ##__VA_ARGS__)

struct vio_osd_info* osd_get_info(uint32_t chn_id, uint32_t ctx_id);

#endif // __HOBOT_OSD_DEV_H__
