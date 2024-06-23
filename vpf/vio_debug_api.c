/**
 * @file: vio_debug_api.c
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
#define pr_fmt(fmt)    "[VIO dbg api]:" fmt
#include "vio_node_api.h"
#include "vio_chain_api.h"
#include "vio_debug_api.h"

static void frame_delay_calcalate(struct frame_delay *fdelay, struct frame_id_desc *frameid)
{
	u64 timestamps;

	timestamps = osal_time_get_ns();
	fdelay->cur_delay_ns = timestamps - frameid->timestamps;
	if (fdelay->min_delay_ns == 0 || fdelay->min_delay_ns > fdelay->cur_delay_ns)
		fdelay->min_delay_ns = fdelay->cur_delay_ns;

	if (fdelay->max_delay_ns == 0 || fdelay->max_delay_ns < fdelay->cur_delay_ns)
		fdelay->max_delay_ns = fdelay->cur_delay_ns;

	if (fdelay->avg_delay_ns == 0)
		fdelay->avg_delay_ns = fdelay->cur_delay_ns;
	else
		fdelay->avg_delay_ns = (fdelay->avg_delay_ns * fdelay->fcount + fdelay->cur_delay_ns) /
			(fdelay->fcount + 1);

	fdelay->fcount++;
}

void vio_loading_calculate(struct vio_hw_loading* loading, enum vio_stat_type stype)
{
	u64 fe_timestamp = 0;
	u64 stop_timestamp = 0;

	if (stype == STAT_FS) {
		loading->pre_fs_timestamp = osal_time_get_ns();
	} else if (stype == STAT_FE) {
		if (osal_atomic_read(&loading->enable_loading) == 1) {
			fe_timestamp = osal_time_get_ns();
			if (loading->start_timestamp < loading->pre_fs_timestamp)
				loading->loading_time += fe_timestamp - loading->pre_fs_timestamp;
			else
				loading->loading_time += fe_timestamp - loading->start_timestamp;
		}
		loading->pre_fs_timestamp = 0;
	} else if (stype == STAT_QB){
		loading->loading_time = 0;
		loading->start_timestamp = osal_time_get_ns();
		osal_atomic_set(&loading->enable_loading, 1);
	} else if (stype == STAT_DQ) {
		osal_atomic_set(&loading->enable_loading, 0);
		stop_timestamp = osal_time_get_ns();
		if (loading->pre_fs_timestamp != 0) {
			if (loading->start_timestamp < loading->pre_fs_timestamp)
				loading->loading_time += stop_timestamp - loading->pre_fs_timestamp;
			else
				loading->loading_time += stop_timestamp - loading->start_timestamp;
		}
		loading->loading_value = loading->loading_time * 10000 / (stop_timestamp - loading->start_timestamp);
	}
}
EXPORT_SYMBOL(vio_loading_calculate);
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Calculate fps in 1s
 * @param[in] *vdev: point to struct vio_subdev instance;
 * @param[in] *frameid: point to struct frame_id_desc instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */

/* code review E1: internal logic function, no need error return */
void vio_fps_calculate(struct frame_debug *fdebug, struct frame_id_desc *frameid)
{
	u32 type;
	osal_time_t ts;
	struct id_set *idset;

	if (fdebug == NULL || frameid == NULL) {
		vio_err("%s: fdebug is 0x%p and frameid is 0x%p\n", __func__, fdebug, frameid);
		return;
	}

	if (frameid->timestamps == 0 && !frameid->tv_sec) {
		osal_time_get(&ts);
		frameid->tv_sec = ts.tv_sec;
	}

	fdebug->fps.cur_frame_count++;
	if (frameid->tv_sec < fdebug->fps.last_time) {
		// vio_warn("%s: cur sec < last sec, timestamps abnormal\n", __func__);
		fdebug->fps.last_time = frameid->tv_sec;
	}

	if (frameid->tv_sec > fdebug->fps.last_time) {
		fdebug->fps.last_frame_count = fdebug->fps.cur_frame_count;
		fdebug->fps.last_time = frameid->tv_sec;
		fdebug->fps.cur_frame_count = 0;
		fdebug->fps.last_update = 1;
	}
	fdebug->fcount++;
	fdebug->frame_id = frameid->frame_id;

	if (fdebug->init_timestamps == 0)
		fdebug->init_timestamps = osal_time_get_ns();

	frame_delay_calcalate(&fdebug->fdelay, frameid);

	idset = &fdebug->idset;
#ifdef HOBOT_MCU_CAMSYS
#else
	if (idset->chn_id < VNODE_ID_CAP)
		type = FS_EVENT;
	else
		type = FE_EVENT;

	vio_vtrace_send(idset->module_id, type, NULL, idset->flow_id, fdebug->frame_id,
		idset->ctx_id, idset->chn_id);
#endif
}
EXPORT_SYMBOL(vio_fps_calculate);

void vio_drop_calculate(struct frame_debug *fdebug, enum vio_drop_type drop_type,
	struct frame_id_desc *frameid)
{
	struct frame_drop_stats *drop_stats;
	struct id_set *idset;

	if (fdebug == NULL || frameid == NULL) {
		vio_err("%s: fdebug or frameid is null\n", __func__);
		return;
	}

	if (drop_type >= DROP_TYPE_NUM) {
		vio_err("%s: wrong input parameters drop_type(%d)", __func__, drop_type);
		return;
	}

	drop_stats = &fdebug->drop_stats[drop_type];
	drop_stats->drop_count++;
	drop_stats->last_drop_id = frameid->frame_id;
	drop_stats->last_timestamps = drop_stats->cur_timestamps;
	drop_stats->cur_timestamps = osal_time_get_ns();

	idset = &fdebug->idset;
#ifdef HOBOT_MCU_CAMSYS
#else
	vio_vtrace_send(idset->module_id, DROP_EVENT, NULL, idset->flow_id, fdebug->frame_id,
		idset->ctx_id, idset->chn_id);
#endif
}
EXPORT_SYMBOL(vio_drop_calculate);

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: IP driver call this function to store frame id and current timestamps
 * @param[in] flow_id: pipeline id;
 * @param[in] stat_type: statistics type;
 * @param[in] frameid: frame id;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_set_stat_info(u32 flow_id, u32 module, enum vio_stat_type stype, u32 frameid)
{
	u32 index;
	struct vio_chain *chain;
	struct module_stat *mstat;
	osal_time_t ts;

	chain = vio_get_chain(flow_id);
	if (chain == NULL)
		return;

	if (module >= MODULE_NUM || stype >= STAT_NUM) {
		vio_err("%s: wrong input parameters module(%d) and stype(%d)", __func__, module, stype);
		return;
	}

	if (frameid == 0) {
		chain->info_idx[module][stype]++;
		index = chain->info_idx[module][stype] % MAX_DELAY_FRAMES;
		mstat = &chain->mstat[module][index];
		mstat->sinfo[stype].frameid = index;
	} else {
		index = frameid % MAX_DELAY_FRAMES;
		mstat = &chain->mstat[module][index];
		if (stype == STAT_FS) {
			mstat->sinfo[STAT_FE].tv_sec = 0;
			mstat->sinfo[STAT_FE].tv_usec = 0;
			mstat->sinfo[STAT_QB].tv_sec = 0;
			mstat->sinfo[STAT_QB].tv_usec = 0;
			mstat->sinfo[STAT_DQ].tv_sec = 0;
			mstat->sinfo[STAT_DQ].tv_usec = 0;
		}
		mstat->sinfo[stype].frameid = frameid;
		chain->info_idx[module][stype] = frameid;
	}

	osal_time_get(&ts);
	mstat->sinfo[stype].tv_sec = ts.tv_sec;
	mstat->sinfo[stype].tv_usec = ts.tv_nsec / 1000;
}
EXPORT_SYMBOL(vio_set_stat_info);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Get statistic information of one pipeline chain;
 * @param[in] flow_id: pipeline id;
 * @retval base address of stat;
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
struct module_stat* vio_get_stat_info(u32 flow_id)
{
	struct vio_chain *chain;

	chain = vio_get_chain(flow_id);
	if (chain == NULL)
		return NULL;

	return &chain->mstat[0][0];
}

u32 vio_fps_snapshot(char* buf, u32 size, u32 flowid_mask)
{
	u32 i, j, k;
	u32 flow_id;
	ssize_t len;
	ssize_t offset = 0;
	struct vio_node *vnode;
	struct vio_node_mgr *vnode_mgr;
	struct vio_chain *vchain;
	struct frame_debug *fdebug;

	for (flow_id = 0; flow_id < VIO_MAX_STREAM; flow_id++) {
		if ((1 << flow_id & flowid_mask) == 0)
			continue;
		vchain = vio_get_chain(flow_id);
		if (vchain == NULL)
			continue;

		len = snprintf(&buf[offset], size - (size_t)offset,
					"-------------------------------------------------------------------\n");
		offset += len;
		len = snprintf(&buf[offset], size - (size_t)offset,
					"                              Flow%d FPS                            \n", flow_id);
		offset += len;
		len = snprintf(&buf[offset], size - (size_t)offset,
					"-------------------------------------------------------------------\n");
		offset += len;

		for (i = 0; i < MODULE_NUM; i++) {
			vnode_mgr = &vchain->vnode_mgr[i];
			for (j = 0; j < MAX_VNODE_NUM; j++) {
				vnode = vnode_mgr->vnode[j];
				if (vnode == NULL || osal_test_bit(VIO_NODE_START, &vnode->state) == 0)
					continue;

				if (size <= offset)
					break;
				len = snprintf(&buf[offset], size - (size_t)offset,
							"%6s ctx%2d: ", vnode->name, vnode->ctx_id);
				offset += len;
				for (k = 0; k < MAXIMUM_CHN; k++) {
					if ((vnode->active_ich & 1 << k) == 0)
						continue;
					fdebug = &vnode->ich_subdev[k]->fdebug;

					if (size <= offset)
						break;
					len = snprintf(&buf[offset], size - (size_t)offset,
								"| ich%d %3d | ", k, fdebug->fps.last_frame_count);
					offset += len;
					if (fdebug->fps.last_update == 0)
						fdebug->fps.last_frame_count = 0;
					fdebug->fps.last_update = 0;
				}
				for (k = 0; k < MAXIMUM_CHN; k++) {
					if ((vnode->active_och & 1 << k) == 0)
						continue;
					fdebug = &vnode->och_subdev[k]->fdebug;

					if (size <= offset)
						break;
					len = snprintf(&buf[offset], size - (size_t)offset,
								"och%d %3d | ", k, fdebug->fps.last_frame_count);
					offset += len;
					if (fdebug->fps.last_update == 0)
						fdebug->fps.last_frame_count = 0;
					fdebug->fps.last_update = 0;
				}

				if (size <= offset)
					break;
				len = snprintf(&buf[offset], size - (size_t)offset, "\n");
				offset += len;
			}
		}
	}

	return offset;
}

u32 vio_fps_stats(char* buf, u32 size, u32 flowid_mask)
{
	u32 i, j, k;
	u32 flow_id;
	u32 avg_fps = 0, cur_fps = 0;
	u64 cur_timestamps, dura_timestamps;
	u32 len;
	u32 offset = 0;
	struct vio_node *vnode;
	struct vio_node_mgr *vnode_mgr;
	struct vio_chain *vchain;
	struct frame_debug *fdebug;

	len = snprintf(&buf[offset], size - offset,
				"-------------------------------------------------------------------\n");
	offset += len;
	len = snprintf(&buf[offset], size - offset,
				"%-10s%-10s%-10s%-5s%10s%10s%10s\n",
				"flowid", "module", "ctx_id", "chn", "fcount", "avg_fps", "cur_fps");
	offset += len;
	len = snprintf(&buf[offset], size - offset,
				"-------------------------------------------------------------------\n");
	offset += len;

	cur_timestamps = osal_time_get_ns();
	for (flow_id = 0; flow_id < VIO_MAX_STREAM; flow_id++) {
		if ((1 << flow_id & flowid_mask) == 0)
			continue;
		vchain = vio_get_chain(flow_id);
		if (vchain == NULL)
			continue;

		for (i = 0; i < MODULE_NUM; i++) {
			vnode_mgr = &vchain->vnode_mgr[i];
			for (j = 0; j < MAX_VNODE_NUM; j++) {
				vnode = vnode_mgr->vnode[j];
				if (vnode == NULL || osal_test_bit(VIO_NODE_START, &vnode->state) == 0)
					continue;

				if (size <= offset)
					break;
				len = snprintf(&buf[offset], size - offset,
							"%-10d%-10s%-10d\n", flow_id, vnode->name, vnode->ctx_id);
				offset += len;
				for (k = 0; k < 16; k++) {
					if (k < MAXIMUM_CHN) {
						if ((vnode->active_ich & 1 << k) == 0)
							continue;
						fdebug = &vnode->ich_subdev[k]->fdebug;
					} else {
						if ((vnode->active_och & 1 << (k - MAXIMUM_CHN)) == 0)
							continue;
						fdebug = &vnode->och_subdev[k - MAXIMUM_CHN]->fdebug;
					}

					if (size <= offset)
						break;
					len = snprintf(&buf[offset], size - offset,
								"%-10s%-10s%-10s", " ", " ", " ");
					offset += len;

					dura_timestamps = cur_timestamps - fdebug->init_timestamps;
					if (dura_timestamps != 0)
						avg_fps = (u64)fdebug->fcount * 1000000000000UL / dura_timestamps;

					dura_timestamps = fdebug->fps.cur_timestamps - fdebug->fps.last_timestamps;
					if (dura_timestamps != 0)
						cur_fps = (u64)1000000000000UL / dura_timestamps;

					if (size <= offset)
						break;
					len = snprintf(&buf[offset], size - offset,
								"%-5d%10d%6d.%03d%6d.%03d\n", k, fdebug->fcount,
								avg_fps / 1000, avg_fps % 1000, cur_fps / 1000, cur_fps % 1000);
					offset += len;
				}

				if (size <= offset)
					break;
				len = snprintf(&buf[offset], size - offset, "\n");
				offset += len;
			}
		}
	}

	return offset;
}

u32 vio_delay_stats(char* buf, u32 size, u32 flowid_mask)
{
	u32 i, j, k;
	u32 flow_id;
	u32 len;
	u32 offset = 0;
	struct vio_node *vnode;
	struct vio_node_mgr *vnode_mgr;
	struct vio_chain *vchain;
	struct frame_debug *fdebug;
	char chn[10] = {0};

	len = snprintf(&buf[offset], size - offset,
				"---------------------------------------------------------------------------------------\n");
	offset += len;
	len = snprintf(&buf[offset], size - offset,
				"%-10s%-10s%-10s%-5s%13s%13s%13s%13s\n",
				"flowid", "module", "ctx_id", "chn", "cur_delay_ms", "min_delay_ms", "avg_delay_ms", "max_delay_ms");
	offset += len;
	len = snprintf(&buf[offset], size - offset,
				"---------------------------------------------------------------------------------------\n");
	offset += len;

	for (flow_id = 0; flow_id < VIO_MAX_STREAM; flow_id++) {
		if ((1 << flow_id & flowid_mask) == 0)
			continue;
		vchain = vio_get_chain(flow_id);
		if (vchain == NULL)
			continue;

		for (i = 0; i < MODULE_NUM; i++) {
			vnode_mgr = &vchain->vnode_mgr[i];
			for (j = 0; j < MAX_VNODE_NUM; j++) {
				vnode = vnode_mgr->vnode[j];
				if (vnode == NULL || osal_test_bit(VIO_NODE_START, &vnode->state) == 0)
					continue;

				if (size <= offset)
					break;
				len = snprintf(&buf[offset], size - offset,
							"%-10d%-10s%-10d\n", flow_id, vnode->name, vnode->ctx_id);
				offset += len;
				for (k = 0; k < 16; k++) {
					if (k < MAXIMUM_CHN) {
						if ((vnode->active_ich & 1 << k) == 0)
							continue;
						fdebug = &vnode->ich_subdev[k]->fdebug;
					} else {
						if ((vnode->active_och & 1 << (k - MAXIMUM_CHN)) == 0)
							continue;
						fdebug = &vnode->och_subdev[k - MAXIMUM_CHN]->fdebug;
					}

					if (size <= offset)
						break;
					len = snprintf(&buf[offset], size - offset,
								"%-10s%-10s%-10s", " ", " ", " ");
					offset += len;

					if (size <= offset)
						break;
					if (k < 8) {
						snprintf(chn, 10, "ich%d", k);
					} else {
						snprintf(chn, 10, "och%d", k - 8);
					}
					len = snprintf(&buf[offset], size - offset,
								"%-5s%13d%13d%13lld%13d\n", chn, fdebug->fdelay.cur_delay_ns / 1000000,
								fdebug->fdelay.min_delay_ns / 1000000, fdebug->fdelay.avg_delay_ns / 1000000,
								fdebug->fdelay.max_delay_ns / 1000000);
					offset += len;
				}

				if (size <= offset)
					break;
				len = snprintf(&buf[offset], size - offset, "\n");
				offset += len;
			}
		}
	}

	return offset;
}

u32 vio_drop_stats(char* buf, u32 size, u32 flowid_mask)
{
	u32 i, j, k, chn;
	u32 flow_id;
	u32 avg_fps = 0, cur_fps = 0;
	u64 cur_timestamps, dura_timestamps;
	char *drop_name[DROP_TYPE_NUM] = {"hw drop", "sw drop", "user drop"};
	u32 len;
	u32 offset = 0;
	struct vio_node *vnode;
	struct vio_node_mgr *vnode_mgr;
	struct vio_chain *vchain;
	struct frame_debug *fdebug;

	len = snprintf(&buf[offset], size - offset,
				"------------------------------------------------------------------------------\n");
	offset += len;
	len = snprintf(&buf[offset], size - offset,
				"%-10s%-10s%-10s%-5s%-10s%10s%10s%10s\n",
				"flowid", "module", "ctx_id", "chn", "drop_type", "drop_cnt", "avg_fps", "cur_fps");
	offset += len;
	len = snprintf(&buf[offset], size - offset,
				"------------------------------------------------------------------------------\n");
	offset += len;
	cur_timestamps = osal_time_get_ns();
	for (flow_id = 0; flow_id < VIO_MAX_STREAM; flow_id++) {
		if ((1 << flow_id & flowid_mask) == 0)
			continue;
		vchain = vio_get_chain(flow_id);
		if (vchain == NULL)
			continue;

		for (i = 0; i < MODULE_NUM; i++) {
			vnode_mgr = &vchain->vnode_mgr[i];
			for (j = 0; j < MAX_VNODE_NUM; j++) {
				vnode = vnode_mgr->vnode[j];
				if (vnode == NULL || osal_test_bit(VIO_NODE_START, &vnode->state) == 0)
					continue;

				if (size <= offset)
					break;
				len = snprintf(&buf[offset], size - offset,
							"%-10d%-10s%-10d\n", flow_id, vnode->name, vnode->ctx_id);
				offset += len;
				for (chn = 0; chn < MAXIMUM_CHN; chn++) {
					if ((vnode->active_och & 1 << chn) == 0)
						continue;
					fdebug = &vnode->och_subdev[chn]->fdebug;

					for (k = 0; k < DROP_TYPE_NUM; k++) {
						if (size <= offset)
							break;
						len = snprintf(&buf[offset], size - offset,
									"%-10s%-10s%-10s", " ", " ", " ");
						offset += len;

						dura_timestamps = cur_timestamps - fdebug->init_timestamps;
						if (dura_timestamps != 0)
							avg_fps = (u64)fdebug->drop_stats[k].drop_count * 1000000000000UL / dura_timestamps;

						dura_timestamps = fdebug->drop_stats[k].cur_timestamps - fdebug->drop_stats[k].last_timestamps;
						if (dura_timestamps != 0)
							cur_fps = (u64)1000000000000UL / dura_timestamps;

						if (size <= offset)
							break;
						len = snprintf(&buf[offset], size - offset,
									"%-5d%-10s%10d%6d.%03d%6d.%03d\n", k, drop_name[k],
									fdebug->drop_stats[k].drop_count, avg_fps / 1000, avg_fps % 1000,
									cur_fps / 1000, cur_fps % 1000);
						offset += len;
						cur_fps = 0;
					}
				}

				if (size <= offset)
					break;
				len = snprintf(&buf[offset], size - offset, "\n");
				offset += len;
			}
		}
	}

	return offset;
}

u32 vio_fmgr_stats(char* buf, u32 size, u32 head_index, u32 flowid_mask)
{
	u32 i, j, k;
	u32 flow_id;
	u32 len;
	u32 offset = 0;
	struct vio_node *vnode;
	struct vio_node_mgr *vnode_mgr;
	struct vio_chain *vchain;
	struct vio_framemgr *framemgr;
	char chn[10] = {0};

	if (head_index == 0) {
		len = snprintf(&buf[offset], size - offset,
					"------------------------------------------------------------------------------\n");
		offset += len;
		len = snprintf(&buf[offset], size - offset,
					"%-10s%-10s%-10s%-5s%6s%10s%10s%10s%6s\n",
					"flowid", "module", "ctx_id", "chn", "FREE", "REQUEST", "PROCESS", "COMPLETE", "USED");
		offset += len;
		len = snprintf(&buf[offset], size - offset,
					"------------------------------------------------------------------------------\n");
		offset += len;
	}

	for (flow_id = 0; flow_id < VIO_MAX_STREAM; flow_id++) {
		if ((1 << flow_id & flowid_mask) == 0)
			continue;
		vchain = vio_get_chain(flow_id);
		if (vchain == NULL)
			continue;

		for (i = 0; i < MODULE_NUM; i++) {
			vnode_mgr = &vchain->vnode_mgr[i];
			for (j = 0; j < MAX_VNODE_NUM; j++) {
				vnode = vnode_mgr->vnode[j];
				if (vnode == NULL || osal_test_bit(VIO_NODE_START, &vnode->state) == 0)
					continue;

				if (size <= offset)
					break;
				len = snprintf(&buf[offset], size - offset,
							"%-10d%-10s%-10d\n", flow_id, vnode->name, vnode->ctx_id);
				offset += len;
				for (k = 0; k < 16; k++) {
					if (k < MAXIMUM_CHN) {
						if ((vnode->active_ich & 1 << k) == 0)
							continue;
						framemgr = &vnode->ich_subdev[k]->framemgr;
					} else {
						if ((vnode->active_och & 1 << (k - MAXIMUM_CHN)) == 0)
							continue;
						framemgr = &vnode->och_subdev[k - MAXIMUM_CHN]->framemgr;
					}

					if (framemgr->num_frames == 0)
						continue;

					if (size <= offset)
						break;
					len = snprintf(&buf[offset], size - offset,
								"%-10s%-10s%-10s", " ", " ", " ");
					offset += len;

					if (size <= offset)
						break;
					if (k < 8) {
						snprintf(chn, 10, "ich%d", k);
					} else {
						snprintf(chn, 10, "och%d", k - 8);
					}
					len = snprintf(&buf[offset], size - offset,
								"%-5s%6d%10d%10d%10d%6d\n", chn, framemgr->queued_count[FS_FREE],
								framemgr->queued_count[FS_REQUEST], framemgr->queued_count[FS_PROCESS],
								framemgr->queued_count[FS_COMPLETE], framemgr->queued_count[FS_USED]);
					offset += len;
				}

				if (size <= offset)
					break;
				len = snprintf(&buf[offset], size - offset, "\n");
				offset += len;
			}
		}
	}

	return offset;
}

static s32 vpf_dbg_query_active_ctx(struct vio_video_ctx *vctx, unsigned long arg)
{
	u32 i, dump_bitmap = 0;
	s64 copy_ret = 0;
	struct vpf_device *dev;
	struct vio_node *vnode;

	dev = vctx->dev;
	for (i = 0; i < dev->max_ctx; i++) {
		vnode = &dev->vnode[i];
		if (osal_test_bit((s32)VIO_NODE_START, &vnode->state) != 0)
			dump_bitmap |= 1 << i;
	}

	copy_ret = osal_copy_to_app((void __user *)arg, (void *)&dump_bitmap, sizeof(u32));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy to user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	return 0;
}

static s32 vpf_dbg_vctx_bind_vdev(struct vio_video_ctx *vctx, unsigned long arg)
{
	u32 i = 0;
	s32 ret = 0;
	u32 ctx_id;
	s64 copy_ret;
	u64 flags;
	struct vio_subdev *vdev;
	struct vpf_device *dev;
	struct vio_node *vnode;

	copy_ret = osal_copy_from_app(&ctx_id, (void __user *) arg, sizeof(u32));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	dev = vctx->dev;
	if (ctx_id >= dev->max_ctx) {
		vio_err("%s: wrong ctx_id = %d\n", __func__, ctx_id);
		return -EFAULT;
	}

	vnode = &dev->vnode[ctx_id];
	if (vctx->id < VNODE_ID_CAP)
		vdev = vnode->ich_subdev[vctx->id];
	else
		vdev = vnode->och_subdev[vctx->id - VNODE_ID_CAP];

	vio_e_barrier_irqs(vdev, flags);
	for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
		if (osal_test_and_set_bit(i, &vdev->val_ctx_mask) == 0) {
			vdev->vctx[i] = vctx;
			vctx->ctx_index = i;
			vctx->vdev = vdev;
			osal_atomic_inc(&vdev->refcount);
			vctx->framemgr = &vdev->framemgr;
			vdev->cur_fmgr = &vdev->framemgr;
			vio_info("[C%d][V%d] %s: done, ctx_index(%d) refcount(%d)\n",
				vctx->ctx_id, vctx->id, __func__,
				i, osal_atomic_read(&vdev->refcount));
			break;
		}
	}
	vio_x_barrier_irqr(vdev, flags);

	if (i == VIO_MAX_SUB_PROCESS) {
		vio_err("[%s][V%d] %s: failed\n", vctx->name, vctx->id, __func__);
		return -EFAULT;
	}
	vctx->state = BIT((s32)VIO_VIDEO_REBUFS) | BIT((s32)VIO_VIDEO_START);

	return ret;
}

static u32 vpf_gtask_dbg_info(struct vio_chain *vchain, s8 *buf)
{
	s32 i, j;
	u32 offset = 0;
    u32 len;
    struct vio_node_mgr *vnode_mgr;
    struct vio_node *vnode, *tmp_vnode;
	struct vio_group_task *gtask;

	for (i = 0; i < MODULE_NUM; i++) {
        vnode_mgr = &vchain->vnode_mgr[i];
        for (j = 0; j < MAX_VNODE_NUM; j++) {/*PRQA S 2810,0497*/
			vnode = vnode_mgr->vnode[j];
			if (vnode == NULL)
				continue;

            if (vnode->leader == 1) {
				gtask = vnode->gtask;
				len = snprintf(&buf[offset], DEBUG_SIZE, "gtask-%s: res %d rcnt %d ",
					gtask->name, gtask->hw_resource_en, gtask->rcount);
				offset += len;
				tmp_vnode = vnode;
				do {
					len = snprintf(&buf[offset], DEBUG_SIZE, "[%s:%d]",
						tmp_vnode->name, tmp_vnode->frame_done_flag);
					offset += len;
					tmp_vnode = tmp_vnode->next;
				} while (tmp_vnode != NULL && tmp_vnode->leader == 0);
			}
			len = snprintf(&buf[offset], DEBUG_SIZE, "\n");
			offset += len;
		}
    }

	return offset;
}
static s32 vpf_dbg_get_fmgr_stats(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u32 len = 0, offset = 0;
	u32 flowid_mask;
	s64 copy_ret = 0;
	char buf[DEBUG_SIZE];
	struct vio_chain *vchain;

	vchain = vio_get_chain(vctx->flow_id);
	if (vchain != NULL) {
		len = snprintf(&buf[offset], DEBUG_SIZE, "%s", vchain->path);
		offset += len;
		len = vpf_gtask_dbg_info(vchain, &buf[offset]);
		offset += len;
	}

	flowid_mask = 1 << vctx->flow_id;
	len = vio_fmgr_stats(&buf[offset], DEBUG_SIZE - offset, 0, flowid_mask);
	offset += len;
	if (offset > DEBUG_SIZE)
		offset = DEBUG_SIZE;

	copy_ret = osal_copy_to_app((void __user *)arg, (void *)buf, offset);
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy to user, ret = %lld\n", __func__, copy_ret);
		ret = -EFAULT;
	}

	return ret;
}

static s32 vpf_dbg_check_dma_output(struct vio_video_ctx *vctx, unsigned long arg)
{
	u32 dma_output = 0;
	s64 copy_ret = 0;
	s32 ret = 0;
	struct vio_subdev *vdev;

	if ((vctx->state & BIT((s32)VIO_VIDEO_START)) == 0) {
		vio_err("[%s][C%d][V%d] %s: invalid debug is requested(0x%llX)", vctx->name,
			vctx->ctx_id, vctx->id, __func__, vctx->state);
		return -EFAULT;
	}

	vdev = vctx->vdev;
	if (osal_test_bit((s32)VIO_SUBDEV_DMA_OUTPUT, &vdev->state) != 0)
		dma_output = 1;

	copy_ret = osal_copy_to_app((void __user *)arg, (void *)&dma_output, sizeof(u32));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy to user, ret = %lld\n", __func__, copy_ret);
		ret = -EFAULT;
	}

	return ret;
}

s32 vio_debug_ctrl(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg)
{
	s32 ret = 0;

	switch (cmd) {
		case VIO_DBG_GET_FMGR_STATS:
			ret = vpf_dbg_get_fmgr_stats(vctx, arg);
			break;
		case VIO_DBG_GET_ACTIVE_CTX:
			ret = vpf_dbg_query_active_ctx(vctx, arg);
			break;
		case VIO_DBG_SET_CTX_ID:
			ret = vpf_dbg_vctx_bind_vdev(vctx, arg);
			break;
		case VIO_DBG_CHK_DMA_OUT:
			ret = vpf_dbg_check_dma_output(vctx, arg);
			break;
		default:
			vio_err("%s: wrong cmd(0x%x)\n", __func__, cmd);
			ret = -EFAULT;
			break;
	}

	return ret;
}