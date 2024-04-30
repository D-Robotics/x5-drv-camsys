/**
 * @file: vio_node_api.h
 * @
 * @NO{S09E05C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */
/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef VIO_NODE_API_H
#define VIO_NODE_API_H

#ifndef HOBOT_MCU_CAMSYS
#include <linux/platform_device.h>
#endif

#include "osal.h"
#include "vio_framemgr.h"
#include "vio_cops_api.h"
#include "vio_debug_api.h"
#include "vio_config.h"
#include "vio_metadata_api.h"

#define MAX_SUB_DEVICE  8u

#define VIN_MODULE 0u
#define ISP_MODULE 1u
#define VSE_MODULE 2u
#define GDC_MODULE 3u
#define N2D_MODULE 4u
#define DPU_MODULE 5u
#define CODEC_MODULE 6u
#define MODULE_NUM 7u

#define FRAME_PRE_INT0  0u
#define FRAME_PRE_INT1  1u
#define FRAME_PRE_INT2  2u
#define FRAME_PRE_INT3  3u
#define FRAME_PRE_INT4  4u
#define FRAME_PRE_INT5  5u
#define FRAME_PRE_INT6  6u
#define FRAME_PRE_INT7  7u
#define FRAME_DONE  8u
#define MAXIMUM_CHN 8

#define	VNODE_ID_SRC 0u
#define	VNODE_ID_CAP MAXIMUM_CHN
#define	VNODE_ID_MAX 16u

#define AUTO_CTX_ID -1

#define MAX_DELAY_FRAMES 5u

#define SHOT_TIMEOUT 100

#define GTASK_FIFO_SCHED 0
#define GTASK_ROLL_SCHED 1

#define vio_e_barrier_irqs(this, flag)	osal_spin_lock_irqsave(&this->slock, &flag)
#define vio_x_barrier_irqr(this, flag)	osal_spin_unlock_irqrestore(&this->slock, &flag)

#define vpf_param_range_check(p, min, max) do { \
		if ((p) < (min) || (p) > (max)) { \
			vio_err("%s: Invalid Param %s: %d, min: %d, max: %d\n", __func__, #p, (p), (min), (max)); \
			return -EINVAL; \
		} \
	} while(0)

#define CFG_MIN(x, y) ((x) > (y) ? (y) : (x))

struct vio_version_info {
	uint32_t major;/**< the major version number >*/
	uint32_t minor;/**< the minor version number >*/
};

typedef enum vpf_struct_type_e {
	BASE_ATTR,
	ICHN_ATTR,
	OCHN_ATTR,
	EX_ATTR,
	STRUCT_ATTR_MAX,
} vpf_struct_type_t;

struct vio_struct_size {
	vpf_struct_type_t type;/**< the type of struct >*/
	int32_t size;/**< the size of struct >*/
};

/**
 * @enum vio_node_task_state
 * Describe state of vio group task thread
 */
enum vio_node_task_state {
	VIO_GTASK_START,
	VIO_GTASK_REQUEST_STOP,
	VIO_GTASK_SHOT,
	VIO_GTASK_SHOT_STOP,
};

/**
 * @enum vio_node_state
 * Describe state of vio node
 */
enum vio_node_state {
	VIO_NODE_OPEN,
	VIO_NODE_INIT,
	VIO_NODE_START,
	VIO_NODE_BIND_CTX,
	VIO_NODE_OTF_INPUT,
	VIO_NODE_OTF_OUTPUT,
	VIO_NODE_DMA_INPUT,
	VIO_NODE_DMA_OUTPUT,
	VIO_NODE_M2M_INPUT,
	VIO_NODE_M2M_OUTPUT,
	VIO_NODE_LEADER,
	VIO_NODE_SHOT,
};

enum vio_bind_type {
	CHN_BIND_NONE,
	CHN_BIND_REP,
	CHN_BIND_OTF,
	CHN_BIND_M2M,
};

/**
 * @enum vio_group_task_state
 * Describe state of vio chain
 */

enum vio_chain_state {
	VIO_CHAIN_OPEN,
	VIO_CHAIN_INIT,
};

//#ifdef CONFIG_HOBOT_VIO_STL
/* Helpers to program the hobot cimdma fusa */
struct vio_stl_ops {
	void (*stl_enable)(void *dev);
	void (*stl_disable)(void *dev);
};

struct vio_stl {
	osal_atomic_t stl_enable;
	u32 polling_value;
	struct vio_stl_ops stl_ops;
};
//#endif

#define NODE_NAME_SIZE 16u
/**
 * @struct vpf_device
 * @brief Define the descriptor of vpf device used to register device nodes.
 * @NO{S09E05C01}
 */
struct vpf_device {
	s8 name[NODE_NAME_SIZE];
	u32 vid;
#ifdef HOBOT_MCU_CAMSYS
#else
	struct device dev;
	struct cdev *cdev;
	struct module *owner;
#endif
	void *iommu_dev;
	struct vio_common_ops *vps_ops;
	struct vio_node *vnode;
	struct vio_group_task *gtask;
	void *ip_dev;
	s32 minor;
	s32 max_ctx;
};

/**
 * @struct vio_video_ctx
 * @brief Define the descriptor of vio video context as the private data of fd handle.
 * @NO{S09E05C01}
 */
struct vio_video_ctx {
	s8 *name;
	u32 id;
	osal_waitqueue_t		done_wq;
	struct vio_framemgr 	*framemgr;
	struct vio_subdev 		*vdev;
	void *device;
	u64	state;
	u32 event;
	u32 ctx_index;
	u32 flow_id;
	u32 ctx_id;
	struct vpf_device *dev;
	void *file;// only for isp;
};

struct chn_attr {
	u32 format;
	u32 width;
	u32 height;
	u32 wstride;
	u32 vstride;
	u32 slot_id;
	u32 work_mode; //0: auto; 1: manual; 2: passthrough; 3: offline
};

/**
 * @struct vio_subdev
 * @brief Describes the input and output channels of IP driver and can be
 * used to bind.
 * @NO{S09E05C01}
 */
struct vio_subdev {
	s8 *name;
	u32 id;
	osal_spinlock_t slock;
	osal_mutex_t mlock;
	struct vio_framemgr	framemgr;
	struct vio_node *vnode;

	struct vio_video_ctx *vctx[VIO_MAX_SUB_PROCESS];
	u64 val_ctx_mask;
	osal_atomic_t refcount;
	u64 state;
	u8 leader;
	u8 reqbuf_flag;
	u8 pingpong_ring;
	u32 crc_value[VIO_BUFFER_MAX_PLANES];
	u32 threshold_time;

	/* static property */
	u8 multi_process;

	struct vio_subdev *prev;
	struct vio_subdev *next;

	struct frame_info curinfo;
	struct vio_framemgr *cur_fmgr;
	void *iommu_dev;
	void (*vdev_work)(struct vio_subdev *vdev, struct vio_frame *frame);

	struct chn_attr chn_attr;
	/* debug */
	struct frame_debug fdebug;
};

/**
 * @struct vio_group_task
 * @brief Define the descriptor of vio task thread.
 * @NO{S09E05C01}
 */
struct vio_group_task {
	s8 *name;
	u32 id;
	u64	state;
	osal_spinlock_t slock;
	osal_atomic_t refcount;

	u8 no_worker;
	u8 sched_mode;
	u8 last_work;
	u8 hw_resource_en;
	osal_list_head_t list;
	u32 rcount; /* request count */
};

/**
 * @struct vio_node
 * @brief Describes software context of IP driver and manager the input
 * and output channels of IP;
 * @NO{S09E05C01}
 */
struct vio_node {
	s8 name[8];
	u32 id;
	u32 hw_id;
	u32 flow_id;
	u32 ctx_id;
	osal_spinlock_t slock;
    struct vio_subdev *ich_subdev[MAXIMUM_CHN];
    struct vio_subdev *och_subdev[MAXIMUM_CHN];
    u32 active_ich;
    u32 active_och;

	struct frame_id_desc frameid; /* transfer frame id */
	u64 state;
	osal_atomic_t rcount; /* request count */
	u32 frame_done_flag;
	u32 leader;
	struct vio_node		*next;
	struct vio_node		*prev;
	struct vio_node		*head;
	struct vio_chain	*vchain;
	struct vio_group_task *gtask;
	u8 path_print;

	osal_atomic_t start_cnt; /* resource count */
	void (*frame_work)(struct vio_node *vnode);
	s32 (*allow_bind)(struct vio_subdev *vdev, struct vio_subdev *remote_vdev, u8 online_mode);
};

/**
 * @struct vio_common_ops
 * @brief Define the descriptor of common callback funtions.
 * @NO{S09E05C01}
 */
struct vio_common_ops {
	s32 (*open)(struct vio_video_ctx *vctx);
	s32 (*close)(struct vio_video_ctx *vctx);
	s32 (*video_set_attr)(struct vio_video_ctx *vctx, unsigned long arg);
	s32 (*video_get_attr)(struct vio_video_ctx *vctx, unsigned long arg);
	s32 (*video_set_attr_ex)(struct vio_video_ctx *vctx, unsigned long arg);
	s32 (*video_get_attr_ex)(struct vio_video_ctx *vctx, unsigned long arg);
	s32 (*video_get_buf_attr)(struct vio_video_ctx *vctx, struct vbuf_group_info *group_attr);
	s32 (*video_set_obuf)(struct vio_video_ctx *vctx, void *buf);
	s32 (*video_set_ibuf)(struct vio_video_ctx *vctx, void *buf);
	s32 (*video_start)(struct vio_video_ctx *vctx);
	s32 (*video_stop)(struct vio_video_ctx *vctx);
	s32 (*video_reset)(struct vio_video_ctx *vctx);
	s32 (*video_s_ctrl)(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg);
	s32 (*video_g_ctrl)(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg);
	s32 (*video_error_callback)(struct vio_video_ctx *vctx, void *error_info);
	s32 (*video_get_version)(struct vio_version_info *version);
	s32 (*video_get_struct_size)(struct vio_struct_size *vio_size);
	u32 (*video_get_hw_status)(struct vio_video_ctx *vctx);

	s32 (*video_set_inter_attr)(struct vio_video_ctx *vctx, unsigned long arg);
	s32 (*video_set_ichn_attr)(struct vio_video_ctx *vctx, unsigned long arg);
	s32 (*video_set_ochn_attr)(struct vio_video_ctx *vctx, unsigned long arg);
	s32 (*video_get_ichn_attr)(struct vio_video_ctx *vctx, unsigned long arg);
	s32 (*video_get_ochn_attr)(struct vio_video_ctx *vctx, unsigned long arg);
	s32 (*video_set_ichn_attr_ex)(struct vio_video_ctx *vctx, unsigned long arg);
	s32 (*video_set_ochn_attr_ex)(struct vio_video_ctx *vctx, unsigned long arg);
};

s32 vio_register_device_node(char *node_name, struct vpf_device *vpf_device);
s32 vio_unregister_device_node(struct vpf_device *vpf_device);

void vio_vnode_init(struct vio_node *vnode);
s32 vio_group_task_start(struct vio_group_task *group_task);
s32 vio_group_task_stop(struct vio_group_task *group_task);
void vio_group_start_trigger(struct vio_node *vnode, struct vio_frame *frame);
void vio_group_cancel_work(struct vio_node *vnode, struct vio_frame *frame);

void vio_get_frame_id(struct vio_node *vnode);
void vio_get_frame_id_by_flowid(u32 flow_id, struct frame_id_desc *frameid);
void vio_get_head_frame_id(struct vio_node *vnode);
void vio_reset_module(u32 module, u32 cfg);
void vio_set_hw_free(struct vio_node *vnode);

s32 vio_subdev_qbuf(struct vio_subdev *vdev, const struct frame_info *frameinfo);
s32 vio_subdev_dqbuf(struct vio_subdev *vdev, struct frame_info *frameinfo);
s32 vio_push_buf_to_next(struct vio_subdev *vdev);
s32 vio_return_buf_to_prev(struct vio_subdev *vdev);

struct vio_chain *vio_get_chain(u32 flow_id);
struct vio_node *vio_get_vnode(u32 flow_id, u32 module_id);
s32 vio_check_drop_info(u32 flow_id, u32 frame_id);
void vio_set_drop_info(u32 flow_id, u32 frame_id);
void *vio_get_metadata(u32 flow_id, u32 frame_id);

void vio_frame_done(struct vio_subdev *vdev);
void vio_frame_ndone(struct vio_subdev *vdev);
void vio_frame_predone(struct vio_subdev *vdev, u32 pre);
s32 vio_hw_format_cov_hbmem_format(u32 hw_format);

s32 vio_debug_ctrl(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg);
void vio_vtrace_send(uint32_t module_id, uint32_t param_type, uint32_t *param,
		    uint32_t flow_id, uint32_t frame_id, uint32_t ctx_id, uint32_t chnid);
s32 vio_clk_enable(const char *name);
s32 vio_clk_disable(const char *name);
s32 vio_set_clk_rate(const char *name, u64 frequency);
u64 vio_get_clk_rate(const char *name);

#endif
