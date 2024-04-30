/**
 * @file: vio_group_api.c
 * @
 * @NO{S090E05C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */
/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)    "[VIO node]:" fmt
#include "hobot_vpf_manager.h"

/**
 * Purpose: vps module name
 * Value: NA
 * Range: vio_chain_api.c
 * Attention: NA
 */
static char* vps_module_name[] = {
	"vin",
	"isp",
	"vse",
	"gdc",
	"n2d",
	"ipu",
	"codec",
};

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Initialize struct vio_node instance
 * @param[in] *vnode: point to struct vio_node instance
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_vnode_init(struct vio_node *vnode)
{
	if (strlen(vnode->name) == 0) {
		snprintf(vnode->name, sizeof(vnode->name), "%s%d",
			vps_module_name[vnode->id], vnode->hw_id);
		if (vnode->gtask != NULL)
			vnode->gtask->name = vnode->name;
	}

	if (osal_test_and_set_bit(VIO_NODE_INIT, &vnode->state) == 0) {
		vnode->flow_id = INVALID_FLOW_ID;
		vnode->leader = 0;
		vnode->path_print = 0;
		vnode->next = NULL;
		vnode->vchain = NULL;
		vnode->prev = vnode;
		vnode->head = vnode;
		osal_atomic_set(&vnode->rcount, 0);
		osal_atomic_set(&vnode->start_cnt, 0);
		(void)memset(&vnode->frameid, 0, sizeof(struct frame_id_desc));
		osal_spin_init(&vnode->slock);/*PRQA S 3334*/
	}
	vio_dbg("[%s] %s: done\n", vnode->name, __func__);
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Start gtask thread;
 * @param[in] *gtask: point to struct group_task instance
 * @retval "= 0": failure
 * @retval "> 0": success
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vio_group_task_start(struct vio_group_task *gtask)
{
	s32 ret = 0;

	if (IS_ERR_OR_NULL((void *)gtask)) {
		vio_err("%s: group_task = 0x%p\n", __func__, gtask);
		return -EINVAL;
	}

	if (osal_atomic_inc_return(&gtask->refcount) > 1)
		return ret;

	gtask->hw_resource_en = 1;
	gtask->sched_mode = GTASK_ROLL_SCHED;
	gtask->rcount = 0;
	osal_spin_init(&gtask->slock);/*PRQA S 3334*/
	osal_list_head_init(&gtask->list);
	osal_set_bit((s32)VIO_GTASK_START, &gtask->state);
	vio_info("[%s] %s: sched_mode %d\n", gtask->name, __func__, gtask->sched_mode);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Stop gtask thread;
 * @param[in] *group_task: point to struct group_task instance;
 * @retval "= 0": failure
 * @retval "> 0": success
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vio_group_task_stop(struct vio_group_task *gtask)
{
	s32 ret = 0;

	if (IS_ERR_OR_NULL((void *)gtask)) {
		vio_err("%s: gtask = 0x%p\n", __func__, gtask);
		return -EINVAL;
	}

	if (osal_atomic_read(&gtask->refcount) == 0 ||
			osal_atomic_dec_return(&gtask->refcount) > 0)
		return ret;

	if (osal_list_empty(&gtask->list) == 0)
		vio_err("%s: work list is not empty, please check\n", __func__);

	osal_clear_bit((s32)VIO_GTASK_START, &gtask->state);
	vio_info("[%s] %s: done\n", gtask->name, __func__);

	return ret;

}

static struct vio_frame *vpf_get_sched_work(struct vio_group_task *gtask)
{
	struct vio_frame *frame, *tmp;
	struct vio_node *vnode;

	frame = osal_list_first_entry(&gtask->list, struct vio_frame, work_list);
	if (gtask->sched_mode == GTASK_ROLL_SCHED) {
		osal_list_for_each_entry(tmp, &gtask->list, work_list) {
			vnode = tmp->vnode;
			if (gtask->last_work < vnode->ctx_id) {
				frame = tmp;
				break;
			}
		}
	}

	return frame;
}

static struct vio_node *vpf_get_ready_vnode(struct vio_group_task *gtask)
{
	u64 flags = 0;
	struct vio_node *vnode = NULL;
	struct vio_frame *frame;

	vio_e_barrier_irqs(gtask, flags);/*PRQA S 2996*/
	if (gtask->hw_resource_en == 0 || gtask->rcount == 0) {
		vio_x_barrier_irqr(gtask, flags);/*PRQA S 2996*/
		return vnode;
	}

	frame = vpf_get_sched_work(gtask);
	osal_list_del(&frame->work_list);
	frame->work_queued = 0;
	vnode = frame->vnode;
	osal_set_bit((s32)VIO_NODE_SHOT, &vnode->state);
	osal_set_bit((s32)VIO_GTASK_SHOT, &gtask->state);
	gtask->hw_resource_en = 0;
	gtask->rcount--;
	gtask->last_work = vnode->ctx_id;
	vio_x_barrier_irqr(gtask, flags);/*PRQA S 2996*/
	(void)memcpy(&vnode->frameid, &frame->frameinfo.frameid, sizeof(struct frame_id_desc));

	vio_dbg("[C%d] %s next config ready, rcount %d\n", vnode->ctx_id, __func__, gtask->rcount);

	return vnode;
}

static void vio_update_hw_config(struct vio_group_task *gtask)
{
	struct vio_node *leader, *vnode;

	leader = vpf_get_ready_vnode(gtask);
	if (leader != NULL) {
		osal_atomic_dec(&leader->rcount);
		vnode = leader->next;
		while (vnode != NULL && vnode->leader == 0) {
			(void)memcpy(&vnode->frameid, &leader->frameid, sizeof(vnode->frameid));
			vio_dbg("[S%d][%s]%s #1\n", leader->flow_id, leader->name, __func__);/*PRQA S 0685,1294*/
			if (vnode->frame_work != NULL)
				vnode->frame_work(vnode);
			vnode = vnode->next;
		}
		vio_dbg("[S%d][%s]%s #2\n", leader->flow_id, leader->name, __func__);/*PRQA S 0685,1294*/
		leader->frame_work(leader);
	}
}

static void vpf_roll_sched_work(struct vio_group_task *gtask, struct vio_frame *frame)
{
	u32 cur_index;
	u8 queued = 0;
	struct vio_frame *tmp;
	struct vio_node *vnode;

	vnode = frame->vnode;
	cur_index = vnode->ctx_id;
	osal_list_for_each_entry_reverse(tmp, &gtask->list, work_list) {/*PRQA S 2810,0497*/
		vnode = tmp->vnode;
		if (vnode->ctx_id <= cur_index) {
			osal_list_add(&frame->work_list, &tmp->work_list);
			queued = 1;
			break;
		}
	}

	if (queued == 0)
		osal_list_add(&frame->work_list, &gtask->list);
}

static void vpf_fifo_sched_work(struct vio_group_task *gtask, struct vio_frame *frame)
{
	osal_list_add_tail(&frame->work_list, &gtask->list);
}
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Queue work into worker queue;
 * @param[in] *vnode: point to struct vio_node instance;
 * @param[in] *frame: point to struct vio_frame instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_group_start_trigger(struct vio_node *vnode, struct vio_frame *frame)
{
	u64 flags = 0;
	struct vio_group_task *gtask;

	if (vnode == NULL || frame == NULL) {
		vio_err("%s: vnode is 0x%p and frame is 0x%p\n", __func__, vnode, frame);
		return;
	}

	gtask = vnode->gtask;
	vio_e_barrier_irqs(gtask, flags);/*PRQA S 2996*/
	if (osal_test_bit((s32)VIO_NODE_START, &vnode->state) != 0) {
		osal_atomic_inc(&vnode->rcount);
		gtask->rcount++;
		frame->work_queued = 1;
		if (gtask->sched_mode == GTASK_ROLL_SCHED)
			vpf_roll_sched_work(gtask, frame);
		else
			vpf_fifo_sched_work(gtask, frame);
	}
	vio_x_barrier_irqr(gtask, flags);/*PRQA S 2996*/

	vio_update_hw_config(gtask);

	vio_dbg("[S%d][%s] %s\n", vnode->flow_id, vnode->name, __func__);
}
EXPORT_SYMBOL(vio_group_start_trigger);/*PRQA S 0605,0307*/

void vio_group_cancel_work(struct vio_node *vnode, struct vio_frame *frame)
{
	u64 flags = 0;
	struct vio_group_task *gtask;

	gtask = vnode->gtask;
	vio_e_barrier_irqs(gtask, flags);/*PRQA S 2996*/
	if (frame->work_queued == 1) {
		osal_atomic_dec(&vnode->rcount);
		osal_list_del(&frame->work_list);
		gtask->rcount--;
		frame->work_queued = 0;
	}
	vio_x_barrier_irqr(gtask, flags);/*PRQA S 2996*/
}

static void vpf_clear_done_flag(struct vio_node *vnode_leader)
{
	struct vio_node *next_vnode;

	vnode_leader->frame_done_flag = 0;
	next_vnode = vnode_leader->next;
	while (next_vnode != NULL && next_vnode->leader == 0) {
		next_vnode->frame_done_flag = 0;
		next_vnode = next_vnode->next;
	}
}
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: IP done notify function;
 * @param[in] *vnode: point to struct vio_node instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_set_hw_free(struct vio_node *vnode)
{
	u64 flags = 0;
	struct vio_group_task *gtask;
	struct vio_node *vnode_leader;
	struct vio_node *next_vnode;

	if (vnode == NULL) {
		vio_err("%s: vnode is NULL\n", __func__);
		return;
	}

	vnode->frame_done_flag = 1;
	if (vnode->leader == 1)
		vnode_leader = vnode;
	else
		vnode_leader = vnode->head;

	gtask = vnode_leader->gtask;

	vio_e_barrier_irqs(gtask, flags);/*PRQA S 2996*/
	next_vnode = vnode_leader;
	do {
		if (osal_test_bit((s32)VIO_NODE_DMA_OUTPUT, &next_vnode->state) != 0 &&
			next_vnode->frame_done_flag == 0)
			break;

		next_vnode = next_vnode->next;
		if (next_vnode == NULL || next_vnode->leader == 1) {
			gtask->hw_resource_en = 1;
			vpf_clear_done_flag(vnode_leader);
		}
	} while (next_vnode != NULL && next_vnode->leader == 0);
	vio_x_barrier_irqr(gtask, flags);/*PRQA S 2996*/
	vio_dbg("[%s][S%d] %s: %s up hw_resource %d\n",
		vnode->name, vnode->flow_id, __func__,
		vnode_leader->name, gtask->hw_resource_en);

	vio_update_hw_config(gtask);
}
EXPORT_SYMBOL(vio_set_hw_free);/*PRQA S 0605,0307*/

void vio_get_frame_id(struct vio_node *vnode)
{
	struct vio_node *vin_vnode;
	struct cim_interface_ops *cim_cops;
	struct hobot_vpf_dev *vpf_dev;

	if (vnode != NULL) {
		vpf_dev = vpf_get_drvdata();
		cim_cops = vpf_dev->vio_cops[VIN_MODULE][COPS_0].cops;
		vin_vnode = vio_get_vnode(vnode->flow_id, VIN_MODULE);
		cim_cops->get_frame_id(vin_vnode, &vnode->frameid);
	}
}
EXPORT_SYMBOL(vio_get_frame_id);/*PRQA S 0605,0307*/

void vio_get_frame_id_by_flowid(u32 flow_id, struct frame_id_desc *frameid)
{
	struct vio_node *vnode;
	struct cim_interface_ops *cim_cops;
	struct hobot_vpf_dev *vpf_dev;

	vpf_dev = vpf_get_drvdata();
	cim_cops = vpf_dev->vio_cops[VIN_MODULE][COPS_0].cops;
	vnode = vio_get_vnode(flow_id, VIN_MODULE);
	cim_cops->get_frame_id(vnode, frameid);
	vio_dbg("[S%d] %s: frameid = %d\n", flow_id, __func__, frameid->frame_id);/*PRQA S 0685,1294*/
}
EXPORT_SYMBOL(vio_get_frame_id_by_flowid);/*PRQA S 0605,0307*/

void vio_get_head_frame_id(struct vio_node *vnode)
{
	struct vio_node *head;

	if (vnode == NULL){
		vio_err("%s: vnode is null\n", __func__);
		return;
	}

	head = vnode->head;
	if (head != vnode) {
		if (vnode->frameid.frame_id == head->frameid.frame_id)
			vio_warn("[S%d] %s: repeat frame id(%d)\n", vnode->flow_id,
				 __func__, vnode->frameid.frame_id);
		(void)memcpy(&vnode->frameid, &head->frameid, sizeof(struct frame_id_desc));
	}
}
EXPORT_SYMBOL(vio_get_head_frame_id);/*PRQA S 0605,0307*/
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: IP reset function;
 * @param[in] module: module id of ip;
 * @param[in] cfg: reset config;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_reset_module(u32 module, u32 cfg)
{
	s32 ret;
	struct camsys_interface_ops *camsys_cops;
	struct hobot_vpf_dev *vpf_dev;

	vpf_dev = vpf_get_drvdata();
	camsys_cops = vpf_dev->vio_cops[VIN_MODULE][COPS_7].cops;
	ret = camsys_cops->ip_reset_func(module, cfg);
	if (ret < 0)
		vio_err("%s: module %d failed\n", __func__, module);
}
EXPORT_SYMBOL(vio_reset_module);/*PRQA S 0605,0307*/

void vio_vtrace_send(uint32_t module_type, uint32_t param_type, uint32_t *param,
		    uint32_t flow_id, uint32_t frame_id, uint32_t ctx_id, uint32_t chnid)
{
	struct dbg_interface_ops *dbg_cops;
	struct hobot_vpf_dev *vpf_dev;

	vpf_dev = vpf_get_drvdata();
	dbg_cops = vpf_dev->vio_cops[DPU_MODULE][COPS_0].cops;
	dbg_cops->vtrace_send(module_type, param_type, param, flow_id, frame_id, ctx_id, chnid);
}