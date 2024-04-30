/**
 * @file: vio_mem.h
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

#ifndef VIO_MEM_API_H
#define VIO_MEM_API_H

#ifdef HOBOT_MCU_CAMSYS
#include <camsys_common.h>
#else
#ifdef CONFIG_HOBOT_J5
#include <linux/ion.h>
#else
#ifdef CONFIG_PCIE_HOBOT_EP_DEV_AI
#include "hobot-ep-vf-PAC-hbmem.h"
#include "vio_mem_pac.h"
#else
#include <linux/ion.h>
#include <hobot_ion_iommu.h>
#endif //CONFIG_PCIE_HOBOT_EP_AI
#endif //CONFIG_HOBOT_J5
#endif //HOBOT_MCU_CAMSYS

#include "vio_config.h"

#define HBN_LAYER_MAXIMUM 6u

#define VIO_ION_MEM_TYPE_MASK 0xFFF
#define VIO_ION_MEM_TYPE_BIT_SHIFT 16u

#define VIO_BUFFER_MAX_PLANES 4
#define VIO_META_PLANE 3

#define BUF_NO_ALLOC 0u
#define BUF_ALLOC_AND_MAP 1u
#define BUF_ONLY_ALLOC 2u
struct hbn_buf_alloc_attr {
    s64 flags;
    u32 buffers_num;
	u32 is_contig;
};

struct vbuf_attr {
    s32 format;
	u32 planecount;
	u32 width;
	u32 height;
	u32 wstride;
	u32 vstride;
};

struct vbuf_info {
	struct vbuf_attr buf_attr;
	u32 share_id[VIO_BUFFER_MAX_PLANES];
	size_t planeSize[VIO_BUFFER_MAX_PLANES];
	u64 paddr[VIO_BUFFER_MAX_PLANES];
	char *addr[VIO_BUFFER_MAX_PLANES];
};

struct vbuf_group_info {
	u32 index;
	struct vbuf_info info[HBN_LAYER_MAXIMUM];
	u32 bit_map;
	u32 is_alloc;
	u32 vbuf_type_mask;
	s64 flags;
	u32 is_contig;
	u32 buffers_num;
	u32 metadata_en;
	u32 dev_num;
	u32 reserved[5];
};

struct vio_buffer {
	u8 ion_alloced;
	u8 ion_cached;
	u8 ion_cachesync;
	u8 ion_mmap;
	u8 iommu_map;
	struct vbuf_group_info group_info;
	u32 iommu_paddr[HBN_LAYER_MAXIMUM][VIO_BUFFER_MAX_PLANES];
	void *metadata;
	void *ion_priv;
};

s32 vio_ion_create(void);
void vio_ion_destroy(void);
s32 vio_ion_alloc(struct vio_buffer *vbuf);
void vio_ion_free(struct vio_buffer *vbuf);
void vio_ion_sync_for_device(const struct vio_buffer *vbuf);
void vio_ion_sync_for_cpu(const struct vio_buffer *vbuf);
s32 vio_iommu_map(void *iommu_dev, struct vio_buffer *vbuf);
void vio_iommu_unmap(void *iommu_dev, struct vio_buffer *vbuf);
s32 vio_handle_ext_buffer(struct vio_buffer *vbuf, s32 *ion_id);

#endif
