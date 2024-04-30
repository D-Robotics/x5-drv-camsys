/**
 * @file: vio_chain_api.h
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

#ifndef VIO_CHAIN_API_H
#define VIO_CHAIN_API_H

#include "vio_node_api.h"

#define DROP_INFO_NUM 6u
#define CMN_META_NUM 10u
#define META_OVERFLOW_EWARN (CMN_META_NUM - 2u)
#define METADATA_SIZE (4 * 1024) //4KB

#define PATH_SIZE 128
/**
 * @struct vio_drop_mgr
 * @brief Define the descriptor of frame drop information manager and manager vio_dropinfo.
 * @NO{S09E05C01}
 */
struct vio_drop_mgr {
	osal_spinlock_t slock;
	u32	drop_frameid[DROP_INFO_NUM];
	u32	head_index;
	u32 last_drop_frameid;
};

struct vio_metadata_mgr {
	osal_spinlock_t slock;
	u32 frameid[CMN_META_NUM];
	u32 max_frameid;
	u32 metadata_size;
	void *metadata;
};

#define MAX_VNODE_NUM 8u
struct vio_node_mgr {
	u32 id;
	osal_spinlock_t slock;
	struct vio_node *vnode[MAX_VNODE_NUM];
};

/**
 * @struct vio_chain
 * @brief Describes one flow chain;
 * @NO{S09E05C01}
 */
struct vio_chain {
    u32 id;
	u64 state;
	struct vio_node_mgr vnode_mgr[MODULE_NUM];
	struct vio_drop_mgr drop_mgr;
    struct vio_metadata_mgr meta_mgr;
	struct module_stat mstat[MODULE_NUM][MAX_DELAY_FRAMES];
	u32 info_idx[MODULE_NUM][STAT_NUM];
    u8 path[PATH_SIZE];
    u8 path_print;
};

/**
 * @struct vio_chain
 * @brief Describes the manager of all flow chains;
 * @NO{S09E05C01}
 */
struct vio_core {
	struct vio_chain vchain[VIO_MAX_STREAM];
	osal_mutex_t mlock;
	osal_atomic_t rsccount;
};

s32 vio_chain_init(struct vio_chain *vchain, u32 id);
char *vchain_get_module_name(u32 vnode_id);
s32 vnode_mgr_add_member(struct vio_node_mgr *vnode_mgr, struct vio_node *vnode);
struct vio_node *vnode_mgr_find_member(struct vio_node_mgr *vnode_mgr, u32 vnode_id, u32 ctx_id);
void vio_chain_path_show(struct vio_chain *vchain);

#endif
