/**
 * @file: vio_chain_api.c
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
#define pr_fmt(fmt)    "[VPF chain]:" fmt
#include "vio_config.h"
#include "vio_chain_api.h"
#include "vio_node_api.h"
#include "hobot_vpf_manager.h"

s32 vio_chain_init(struct vio_chain *vchain, u32 id)
{
	s32 i;
	struct vio_metadata_mgr *meta_mgr;
	struct vio_node_mgr *vnode_mgr;
    struct vio_drop_mgr *drop_mgr;

    vchain->id = id;
	meta_mgr = &vchain->meta_mgr;
	osal_spin_init(&meta_mgr->slock);
	meta_mgr->max_frameid = 0;
	(void)memset(meta_mgr->frameid, 0, sizeof(meta_mgr->frameid));
	if (meta_mgr->metadata == NULL) {
		meta_mgr->metadata_size = METADATA_SIZE;
		meta_mgr->metadata = osal_kzalloc(METADATA_SIZE * CMN_META_NUM, GFP_ATOMIC);
        if (meta_mgr->metadata == NULL) {
            vio_err("%s: metadata alloc failed\n", __func__);
            return -ENOMEM;
        }
	} else {
        (void)memset(meta_mgr->metadata, 0, METADATA_SIZE * CMN_META_NUM);
    }

    drop_mgr = &vchain->drop_mgr;
    (void)memset(drop_mgr, 0, sizeof(struct vio_drop_mgr));
	osal_spin_init(&drop_mgr->slock);

	for (i = 0; i < MODULE_NUM; i++) {
		vnode_mgr = &vchain->vnode_mgr[i];
		(void)memset(vnode_mgr, 0, sizeof(struct vio_node_mgr));
		vnode_mgr->id = id;
		osal_spin_init(&vnode_mgr->slock);
	}

	(void)memset(vchain->mstat, 0, sizeof(vchain->mstat));
	(void)memset(vchain->info_idx, 0, sizeof(vchain->info_idx));

    (void)memset(vchain->path, 0, sizeof(vchain->path));
    vchain->path_print = 0;

    return 0;
}

struct vio_chain *vio_get_chain(u32 flow_id)
{
	struct vio_chain *vchain = NULL;
	struct hobot_vpf_dev *vpf_dev;

	if (flow_id == INVALID_FLOW_ID)
		goto err;

	if (flow_id >= VIO_MAX_STREAM) {
		vio_err("%s: wrong flow_id %d\n", __func__, flow_id);
		goto err;
	}

	vpf_dev = vpf_get_drvdata();
	vchain = &vpf_dev->iscore.vchain[flow_id];
	if (osal_test_bit(VIO_CHAIN_OPEN, &vchain->state) == 0) {
		vio_dbg("[S%d] %s: current flow not work, vchain state 0x%llx\n",
			flow_id, __func__, vchain->state);
		vchain = NULL;
		goto err;
	}

err:
	return vchain;
}

s32 vnode_mgr_add_member(struct vio_node_mgr *vnode_mgr, struct vio_node *vnode)
{
	s32 ret = 0;
	u32 i;
	u64 flags;

	vio_e_barrier_irqs(vnode_mgr, flags);
	for (i = 0; i < MAX_VNODE_NUM; i++) {
		if (vnode_mgr->vnode[i] == NULL) {
			vnode_mgr->vnode[i] = vnode;
			break;
		}
	}
	vio_x_barrier_irqr(vnode_mgr, flags);

	if (i == MAX_VNODE_NUM) {
		vio_err("[%s][S%d] %s: vnode_mgr is full\n", vnode->name, vnode_mgr->id, __func__);
		ret = -EFAULT;
	}

	return ret;
}


struct vio_node *vnode_mgr_find_member(struct vio_node_mgr *vnode_mgr, u32 hw_id, u32 ctx_id)
{
	u32 i;
	u64 flags;
	struct vio_node *vnode = NULL, *tmp;

	vio_e_barrier_irqs(vnode_mgr, flags);
	for (i = 0; i < MAX_VNODE_NUM; i++) {
		tmp = vnode_mgr->vnode[i];
		if (tmp != NULL && tmp->hw_id == hw_id && tmp->ctx_id == ctx_id) {
			vnode = tmp;
			break;
		}
	}
	vio_x_barrier_irqr(vnode_mgr, flags);

	return vnode;
}

static u64 vnode_path_show(u8 *buf, u64 offset, struct vio_node *vnode)
{
    u64 len;

    if (vnode->path_print == 0) {
        vnode->path_print = 1;
        if (osal_test_bit(VIO_NODE_DMA_INPUT, &vnode->state) != 0) {
            len = snprintf(&buf[offset], PATH_SIZE - offset, "(dma)");
            offset += len;
        }

        len = snprintf(&buf[offset], PATH_SIZE - offset, "%s_C%d", vnode->name, vnode->ctx_id);
        offset += len;

        if (vnode->leader == 1) {
            len = snprintf(&buf[offset], PATH_SIZE - offset, "*");
            offset += len;
        }

        if (osal_test_bit(VIO_NODE_DMA_OUTPUT, &vnode->state) != 0) {
            len = snprintf(&buf[offset], PATH_SIZE - offset, "(dma)");
            offset += len;
        }

        if (osal_test_bit(VIO_NODE_M2M_OUTPUT, &vnode->state) != 0) {
            len = snprintf(&buf[offset], PATH_SIZE - offset, "-m2m-");
            offset += len;
        }

        if (osal_test_bit(VIO_NODE_OTF_OUTPUT, &vnode->state) != 0) {
            len = snprintf(&buf[offset], PATH_SIZE - offset, "-otf-");
            offset += len;
        }
    }

    return offset;
}

static s32 vio_search_vnode_path(char *buf, struct vio_node *vnode)
{
	u32 i;
	s32 len;
	s32 offset = 0;
	struct vio_node *tmp_vnode;
	struct vio_subdev *vdev;

	len = vnode_path_show(buf, (size_t)offset, vnode);
	offset += len;
	if (osal_test_bit((u32)VIO_NODE_OTF_OUTPUT, &vnode->state)) {
		tmp_vnode = vnode->next;
		len = vio_search_vnode_path(&buf[offset], tmp_vnode);
		offset += len;

	}

	if (osal_test_bit((u32)VIO_NODE_M2M_OUTPUT, &vnode->state)) {
		for (i = 0; i < MAXIMUM_CHN; i++) {
			if ((vnode->active_och & (u32)1u << i) == 0u)
				continue;
			vdev = vnode->och_subdev[i];
			if (vdev->next != NULL) {
				tmp_vnode = vdev->next->vnode;
				len = vio_search_vnode_path(&buf[offset], tmp_vnode);
				offset += len;
			}
		}
	}

	return offset;
}

void vio_chain_path_show(struct vio_chain *vchain)
{
	s32 i, j;
	u64 offset = 0;
	u64 len;
	struct vio_node_mgr *vnode_mgr;
	struct vio_node *vnode;

	if (vchain == NULL) {
		vio_info("%s: feedback independent mode", __func__);
		return;
	}

	if (vchain->path_print == 1)
		return;

	vchain->path_print = 1;
	len = snprintf(&vchain->path[offset], PATH_SIZE - offset, "[S%d] ", vchain->id);
	offset += len;

	for (i = 0; i < MODULE_NUM; i++) {
		vnode_mgr = &vchain->vnode_mgr[i];
		for (j = 0; j < MAX_VNODE_NUM; j++) {/*PRQA S 2810,0497*/
			vnode = vnode_mgr->vnode[j];
			if (vnode == NULL)
				continue;

			if (osal_test_bit((u32)VIO_NODE_OTF_INPUT, &vnode->state) ||
					osal_test_bit((u32)VIO_NODE_M2M_INPUT, &vnode->state))
				continue;

			len = vio_search_vnode_path(&vchain->path[offset], vnode);
			offset += (size_t)len;
		}
	}
	len = snprintf(&vchain->path[offset], PATH_SIZE - offset, "\n");
	offset += len;

	vio_info("%s: %s", __func__, vchain->path);
}

struct vio_node *vio_get_vnode(u32 flow_id, u32 module_id)
{
	u32 i;
	struct vio_chain *vchain;
	struct vio_node_mgr *vnode_mgr;
	struct vio_node *vnode = NULL;

	if (module_id >= MODULE_NUM) {
		vio_err("[S%d] %s: wrong vnode_id %d\n", flow_id, __func__, module_id);
		return NULL;
	}

	vchain = vio_get_chain(flow_id);
	if (vchain == NULL) {
		vio_err("[S%d][M%d] %s: chain is not work", flow_id, module_id, __func__);
		return NULL;
	}

	vnode_mgr = &vchain->vnode_mgr[module_id];
	for (i = 0; i < MAX_VNODE_NUM; i++) {
		vnode = vnode_mgr->vnode[i];
		if (vnode != NULL)
			break;
	}

	return vnode;
}
EXPORT_SYMBOL(vio_get_vnode);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Check the frame whether should drop or not;
 * @param[in] flow_id: pipe id;
 * @param[in] frame_id: frame id;
 * @retval "= 0": not drop
 * @retval "= 1": drop
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vio_check_drop_info(u32 flow_id, u32 frame_id)
{
	u32 i, index;
	s32 drop_flag = 0;
	u64 flags = 0;
	struct vio_chain *vchain;
	struct vio_drop_mgr *drop_mgr;

	if (flow_id == INVALID_FLOW_ID)
		return drop_flag;

	vchain = vio_get_chain(flow_id);
	if (vchain == NULL) {
		vio_err("S%d %s: chain not work\n", flow_id, __func__);
		return drop_flag;
	}

	drop_mgr = &vchain->drop_mgr;
	vio_e_barrier_irqs(drop_mgr, flags);
	index = drop_mgr->head_index;
	for (i = 0; i < DROP_INFO_NUM; i++) {
		if (frame_id > drop_mgr->drop_frameid[index])
			break;

		if (frame_id == drop_mgr->drop_frameid[index]) {
			drop_flag = 1;
			drop_mgr->last_drop_frameid = frame_id;
			vio_warn("[S%d] %s: frame id %d, drop_flag %d\n", flow_id, __func__, frame_id, drop_flag);
			break;
		}
		index = (index + DROP_INFO_NUM - 1) % DROP_INFO_NUM;
	}
	vio_x_barrier_irqr(drop_mgr, flags);

	if (i == DROP_INFO_NUM)
		vio_warn("[S%d] %s: frame id %d < all drop info, please check!\n", flow_id, __func__, frame_id);

	return drop_flag;
}
EXPORT_SYMBOL(vio_check_drop_info);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Store frame id of drop frame which should drop;
 * @param[in] flow_id: pipe id;
 * @param[in] frame_id: frame id;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_set_drop_info(u32 flow_id, u32 frame_id)
{
	u32 next_index, cur_index, prev_index;
	u64 flags = 0;
	struct vio_drop_mgr *drop_mgr;
	struct vio_chain *vchain;
	struct frame_id_desc frameid;

	if (flow_id == INVALID_FLOW_ID)
		return;

	vchain = vio_get_chain(flow_id);
	if (vchain == NULL) {
		vio_err("S%d %s: chain not work\n", flow_id, __func__);
		return;
	}

	if (frame_id == 0xffffffff) {
		vio_get_frame_id_by_flowid(flow_id, &frameid);
		frame_id = frameid.frame_id;
	}

	drop_mgr = &vchain->drop_mgr;
	vio_e_barrier_irqs(drop_mgr, flags);
	cur_index = drop_mgr->head_index;
	next_index = (cur_index + 1) % DROP_INFO_NUM;
	prev_index = (cur_index - 1) % DROP_INFO_NUM;
	if(drop_mgr->drop_frameid[cur_index] == frame_id ||
		drop_mgr->drop_frameid[prev_index] == frame_id) {
		vio_dbg("%s: drop frame %d already exists\n", __func__, frame_id);
		vio_x_barrier_irqr(drop_mgr, flags);
		return;
	}

	if (drop_mgr->last_drop_frameid < drop_mgr->drop_frameid[next_index])
		vio_warn("%s: drop pool is overflow, drop frame id %d\n", __func__,
            drop_mgr->drop_frameid[next_index]);

	if (drop_mgr->drop_frameid[cur_index] < frame_id) {
		drop_mgr->drop_frameid[next_index] = frame_id;
	} else {
		drop_mgr->drop_frameid[next_index] = drop_mgr->drop_frameid[cur_index];
		drop_mgr->drop_frameid[cur_index] = frame_id;
	}
	drop_mgr->head_index = next_index;

	vio_x_barrier_irqr(drop_mgr, flags);

	vio_warn("[S%d] %s: frame id %d\n", flow_id, __func__, frame_id);
}
EXPORT_SYMBOL(vio_set_drop_info);/*PRQA S 0605,0307*/

void *vio_get_metadata(u32 flow_id, u32 frame_id)
{
	s32 cur_index;
	u64 flags = 0;
	void *metadata = NULL;
	struct vio_metadata_mgr *meta_mgr;
	struct vio_chain *vchain;

	if (flow_id == INVALID_FLOW_ID)
		return NULL;

	vchain = vio_get_chain(flow_id);
	if (vchain == NULL) {
		vio_err("S%d %s: chain not work\n", flow_id, __func__);
		return NULL;
	}

	meta_mgr = &vchain->meta_mgr;
	vio_e_barrier_irqs(meta_mgr, flags);
	if (meta_mgr->max_frameid <= frame_id) {
		meta_mgr->max_frameid = frame_id;
	} else {
		if ((meta_mgr->max_frameid - frame_id) >= META_OVERFLOW_EWARN)
			vio_warn("%s: overflow early warning, max frameid %d, frame id %d, please check!\n",
				__func__, meta_mgr->max_frameid, frame_id);
	}

	cur_index = frame_id % CMN_META_NUM;
	if (meta_mgr->frameid[cur_index] < frame_id) {
		meta_mgr->frameid[cur_index] = frame_id;
		metadata = meta_mgr->metadata + cur_index * METADATA_SIZE;
		vio_init_metadata(flow_id, metadata, frame_id);
	} else if (meta_mgr->frameid[cur_index] == frame_id) {
		metadata = meta_mgr->metadata + cur_index * METADATA_SIZE;
	} else {
		vio_warn("%s: frame id %d not match\n", __func__, frame_id);
	}
	vio_x_barrier_irqr(meta_mgr, flags);

	return metadata;
}
EXPORT_SYMBOL(vio_get_metadata);/*PRQA S 0605,0307*/
