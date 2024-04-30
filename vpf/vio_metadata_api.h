/**
 * @file: vio_metadata_api.h
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

#ifndef VIO_METADATA_API_H
#define VIO_METADATA_API_H

#ifndef HOBOT_MCU_CAMSYS
#include <linux/platform_device.h>
#endif
#include <linux/types.h>
#include "osal.h"

// 0x00.00.00.00===>0x00.00.major.minor
#define METADATA_HEADER_VERSION (0x00000100)

#define METADATA_ITEM_MAGIC	(0x11223344)

/**
 * @struct metadata_header_s
 * @brief Define the descriptor of metadata_header.
 * @NO{S09E05C01}
 */
struct metadata_header_s {
	u16 version;    // version
	u8 header_size; // metadata header size
	u8 crc_info;    // reserved crc cfg
	u32 crc_data;   // reserved crc data
	u32 frame_id;
	union {
		osal_atomic_t offset;
		u64 item_offset;
	};
	u32 reserved[10]; // reserved
};

struct item_header_s {
	u32 magic; // item magic
	u8 tag; // module
	u8 idx; // id
	u16 reserved; // reserved
	u32 size; // item_header_size + item_buf_size
};

// cim item idx
#define CIM_ITEM_RGBIR_IDX	(0x0)
struct cim_item_s {
	u32 rgbir_index;
};

int32_t vio_fill_metadata(u32 flow_id, void *metadata, void *itemdata, u32 size, u8 tag, u8 idx);
int32_t vio_init_metadata(u32 flow_id, void *metadata, u32 frame_id);
void *vio_analysis_metadata(u32 flow_id, void *metadata, u8 tag, u8 idx);

#endif
