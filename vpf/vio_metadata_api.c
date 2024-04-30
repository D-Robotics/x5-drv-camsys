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
#define pr_fmt(fmt)    "[VPF metadata]:" fmt
#include "vio_metadata_api.h"
#include "vio_chain_api.h"
#include "vio_config.h"

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Store frame id of drop frame which should drop;
 * @param[in] flow_id: pipe id;
 * @param[in] metadata: metadata buf;
 * @param[in] itemdata: itemdata buf;
 * @param[in] size: itemdata buf size;
 * @param[in] tag: tag idx;
 * @param[in] idx: idx;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vio_fill_metadata(u32 flow_id, void *metadata, void *itemdata, u32 size, u8 tag, u8 idx)
{
	s32 ret = 0;
	u32 new_offset = 0;
	const u32 alignment_size = 4;
	u32 temp_offset = 0;
	u32 temp_size = 0;
	void *temp_ptr = NULL;
	struct metadata_header_s * meta_header = (struct metadata_header_s *)metadata;
	struct item_header_s *item_header = NULL;

	if ((meta_header == NULL) || (itemdata == NULL)) {
		vio_err("S%d %s: metadat buf is null!\n", flow_id, __func__);
		return -EINVAL;
	}

	// alignment 4
	temp_size = (size + alignment_size - 1) & (~(alignment_size - 1));
	temp_offset = osal_atomic_read(&meta_header->offset);
	#ifndef HOBOT_MCU_CAMSYS
	do {
		new_offset = temp_offset + temp_size + sizeof(struct item_header_s);
		if (new_offset >= METADATA_SIZE) {
			return -EINVAL;
		}
	} while(!atomic_try_cmpxchg(&meta_header->offset, &temp_offset, new_offset));
	#endif
	item_header = (struct item_header_s *)(metadata + new_offset - temp_size - sizeof(struct item_header_s));
	item_header->magic = METADATA_ITEM_MAGIC;
	item_header->size = temp_size + sizeof(struct item_header_s);
	item_header->tag = tag;
	item_header->idx = idx;
	temp_ptr = metadata + new_offset - temp_size;
	(void)memcpy(temp_ptr, itemdata, size);

	return ret;
}
EXPORT_SYMBOL(vio_fill_metadata);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Init metadata buf header;
 * @param[in] flow_id: pipe id;
 * @param[in] metadata: metadata buf;
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
s32 vio_init_metadata(u32 flow_id, void *metadata, u32 frame_id)
{
	s32 ret = 0;
	struct metadata_header_s * meta_header = (struct metadata_header_s *)metadata;

	if (meta_header == NULL) {
		vio_err("S%d %s: metadat buf is null!\n", flow_id, __func__);
		return -EINVAL;
	}

	meta_header->version = METADATA_HEADER_VERSION;
	meta_header->header_size = sizeof(struct metadata_header_s);
	meta_header->crc_info = 0;
	meta_header->crc_data = 0;
	meta_header->frame_id = frame_id;
	osal_atomic_set(&meta_header->offset, sizeof(struct metadata_header_s));

	return ret;
}
EXPORT_SYMBOL(vio_init_metadata);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Init metadata buf header;
 * @param[in] flow_id: pipe id;
 * @param[in] metadata: metadata buf;
 * @param[in] tag: item module;
 * @param[in] idx: item id;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void *vio_analysis_metadata(u32 flow_id, void *metadata, u8 tag, u8 idx)
{
	void *ret_buf = NULL;
	u32 temp_offset = sizeof(struct metadata_header_s);
	struct item_header_s * item_header = NULL;
	struct metadata_header_s * meta_header = (struct metadata_header_s *)metadata;

	if (meta_header == NULL) {
		vio_err("S%d %s: metadat buf is null!\n", flow_id, __func__);
		return NULL;
	}

	while (temp_offset < osal_atomic_read(&meta_header->offset)) {
		item_header = (struct item_header_s *)(metadata + temp_offset);
		if ((tag == item_header->tag) && (idx == item_header->idx) &&
				(METADATA_ITEM_MAGIC == item_header->magic)) {
			ret_buf = (void *)item_header + sizeof(struct item_header_s);
			break;
		}
		temp_offset += item_header->size;
	}

	return ret_buf;
}
EXPORT_SYMBOL(vio_analysis_metadata);/*PRQA S 0605,0307*/
