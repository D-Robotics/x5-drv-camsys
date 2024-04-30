/**
 * @file: vio_mem.c
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
#define pr_fmt(fmt)    "[VIO mem]:" fmt
#include <asm/cacheflush.h>
#include "osal.h"
#include "vio_mem.h"
#include "vio_config.h"

/**
 * Purpose: global point to struct ion_client
 * Value: NULL
 * Range: vio_mem.c
 * Attention: NA
 */
static struct ion_client *g_ion_client;
static void free_vbuf(struct ion_client *ion_client, struct ion_handle **ion_handle,
	struct vbuf_info *info, u8 mmap, u8 is_contig);

struct vio_ion_priv {
	struct ion_handle *ion_handle[HBN_LAYER_MAXIMUM][VIO_BUFFER_MAX_PLANES];
	struct ion_dma_buf_data ion_data[HBN_LAYER_MAXIMUM][VIO_BUFFER_MAX_PLANES];
};

#ifndef CONFIG_PCIE_HOBOT_EP_AI
struct ion_client *vio_get_ion_client(s32 dev_num)
{
	return g_ion_client;
}
#endif
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Create ion client in probe;
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vio_ion_create(void)
{
	s32 ret = 0;
	struct ion_device *hb_ion_dev;

	hb_ion_dev = hobot_ion_get_ion_device();
	if (hb_ion_dev == NULL) {
		vio_err("%s: hb_ion_dev is null.\n", __func__);
		ret = -EFAULT;
	}

	if (g_ion_client == NULL) {
		g_ion_client = ion_client_create(hb_ion_dev, "vio_driver_ion");
		if (IS_ERR((void *)g_ion_client)) {
			vio_err("%s: ion client create failed.\n", __func__);
			ret = -EFAULT;
		}
	}

	return ret;
}

void vio_ion_destroy(void)
{
	ion_client_destroy(g_ion_client);
	g_ion_client = NULL;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Alloc one buffer layer in vio frame by ion;
 * @param[in] *info: piont to struct vbuf_info instance;
 * @param[in] ionFlags: ion flag;
 * @param[in] mmap: 1 means map virsual address, 0 means ummap;
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] **ion_handle: store ion handle point;
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 alloc_vbuf(struct ion_client *ion_client, struct ion_handle **ion_handle,
	struct vbuf_info *info, u32 ionHeapMask,
	u32 ionFlags, u8 mmap, u8 is_contig)
{
	s32 ret = 0;
	s32 i, j;
	size_t size, planeSize;
	u32 plane_count;
	struct vbuf_attr *buf_attr;

	if (info->planeSize[VIO_META_PLANE] > 0) {
		ion_handle[VIO_META_PLANE] = ion_alloc(ion_client,
			info->planeSize[VIO_META_PLANE], PAGE_SIZE,
			ionHeapMask, ionFlags);
		if (IS_ERR(ion_handle[VIO_META_PLANE])) {
			vio_err("%s: metadata buf failed\n", __func__);
			return -EFAULT;
		}
		ion_phys(ion_client, ion_handle[VIO_META_PLANE]->id,
			&info->paddr[VIO_META_PLANE], &info->planeSize[VIO_META_PLANE]);
		info->share_id[VIO_META_PLANE] = ion_handle[VIO_META_PLANE]->share_id;
		info->addr[VIO_META_PLANE] = ion_map_kernel(ion_client, ion_handle[VIO_META_PLANE]);
	}

	buf_attr = &info->buf_attr;
	plane_count = buf_attr->planecount;
	if (is_contig == 1) {
		planeSize = info->planeSize[0];
		for (i = 1; i < plane_count; i++)
			info->planeSize[0] += info->planeSize[i];
		plane_count = 1;
	}

	for (i = 0; i < plane_count; i++) {
		ion_handle[i] = ion_alloc(ion_client,
					info->planeSize[i], PAGE_SIZE,
					ionHeapMask, ionFlags);
		if (IS_ERR(ion_handle[i])) {
			vio_err("%s: failed\n", __func__);
			for (j = i - 1; j >= 0; j--)
				ion_free(ion_client, ion_handle[j]);

			if(ion_handle[VIO_META_PLANE] != NULL)
				ion_free(ion_client, ion_handle[VIO_META_PLANE]);
			return -EFAULT;
		}
		size = info->planeSize[i];
		info->share_id[i] = ion_handle[i]->share_id;
		ret = ion_phys(ion_client, ion_handle[i]->id, &info->paddr[i], &size);
		if (mmap == 1u)
			info->addr[i] = ion_map_kernel(ion_client, ion_handle[i]);

		vio_dbg("%s: paddr[%d] = 0x%llx addr[%d] = 0x%p reqsize = %ld, active size = %ld\n",/*PRQA S 0685,1294*/
				__func__, i, info->paddr[i], i, info->addr[i], info->planeSize[i], size);
	}

	if (is_contig == 1) {
		info->planeSize[0] = planeSize;
		for (i = 1; i < buf_attr->planecount; i++)
			info->paddr[i] = info->paddr[i - 1] + info->planeSize[i - 1];
	}

	return ret;
}

static s32 vio_buffer_check(struct vio_buffer *vbuf)
{
	s32 i, j;
	struct vbuf_group_info *group_info;
	struct vbuf_attr *buf_attr;

	group_info = &vbuf->group_info;
	if (group_info->bit_map == 0) {
		vio_err("[F%d] %s: invalid parameter bitmap = 0\n",
			group_info->index, __func__);
		return -EFAULT;
	}

	for (i = 0; i < HBN_LAYER_MAXIMUM; i++) {
		if ((1 << i & group_info->bit_map) == 0)
			continue;

		buf_attr = &group_info->info[i].buf_attr;
		if (buf_attr->planecount >= VIO_BUFFER_MAX_PLANES
			|| buf_attr->planecount == 0) {
			vio_err("[F%d] %s: L%d invalid parameter plane count %d\n",
				group_info->index, __func__, i, buf_attr->planecount);
			return -EFAULT;
		}

		for (j = 0; j < buf_attr->planecount; j++) {
			if (group_info->info[i].planeSize[j] == 0) {
				vio_err("[F%d] %s: L%d P%d invalid parameter plansize = 0\n",
					group_info->index, __func__, i, j);
				return -EFAULT;
			}
		}
	}

	return 0;
}
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Alloc all buffer layers in vio frame by ion;
 * @param[in] *buffer: point to struct vio_buffer instance;
 * @param[in] buf_type_mask: buffer type;
 * @param[in] mmap: 1 means map virsual address, 0 means ummap;
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vio_ion_alloc(struct vio_buffer *vbuf)
{
	s32 ret = 0;
	s32 i, j;
	u32 m_ionFlags, m_ionHeapMask;
	s32 port;
	struct ion_client *ion_client;
	struct vbuf_group_info *group_info;
	struct vio_ion_priv *ion_priv;

	if (vbuf == NULL) {
		vio_err("%s: vbuf is NULL\n", __func__);
		return -EFAULT;
	}

	if (vbuf->ion_alloced == 1)
		return ret;

	ret = vio_buffer_check(vbuf);
	if (ret < 0)
		return ret;

	vbuf->ion_priv = osal_kzalloc(sizeof(struct vio_ion_priv), GFP_ATOMIC);
	if (vbuf->ion_priv == NULL) {
		vio_err("%s: ion_priv osal_kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	ion_client = vio_get_ion_client(vbuf->group_info.dev_num);
	ion_priv = vbuf->ion_priv;
	group_info = &vbuf->group_info;
	m_ionFlags = hbmem_flag_to_ion_flag(group_info->flags, &m_ionHeapMask, &port);
	if ((group_info->flags & HB_MEM_USAGE_HW_MASK) == 0 && group_info->vbuf_type_mask != 0)
		m_ionFlags = (m_ionFlags & (~(VIO_ION_MEM_TYPE_MASK << VIO_ION_MEM_TYPE_BIT_SHIFT)))
					| (group_info->vbuf_type_mask << VIO_ION_MEM_TYPE_BIT_SHIFT);

	m_ionHeapMask = ION_HEAP_TYPE_CMA_RESERVED_MASK;
	if (vbuf->ion_cached == 1)
		m_ionFlags |= ION_FLAG_CACHED | ION_FLAG_CACHED_NEEDS_SYNC;

	for (i = 0; i < HBN_LAYER_MAXIMUM; i++) {
		if ((1 << i & group_info->bit_map) == 0)
			continue;

		ret = alloc_vbuf(ion_client, ion_priv->ion_handle[i],
				&group_info->info[i], m_ionHeapMask, m_ionFlags,
				vbuf->ion_mmap, group_info->is_contig);
		if (ret < 0) {
			vio_err("[F%d] %s: L%d failed\n", group_info->index, __func__, i);
			for (j = i - 1; j >= 0; j--) {
				if ((1 << j & group_info->bit_map) == 0)
					continue;
				free_vbuf(ion_client, ion_priv->ion_handle[j],
					&group_info->info[j], vbuf->ion_mmap, group_info->is_contig);
			}
			osal_kfree(vbuf->ion_priv);
			vbuf->ion_priv = NULL;
			return ret;
		}

		if (group_info->metadata_en == 1 && vbuf->metadata == NULL)
			vbuf->metadata = group_info->info[i].addr[VIO_META_PLANE];

		vio_dbg("[F%d] %s: [L%d] done", group_info->index, __func__, i);/*PRQA S 0685,1294*/
	}

	vbuf->ion_alloced = 1;
	vio_dbg("%s: m_ionFlags = 0x%x", __func__, m_ionFlags);/*PRQA S 0685,1294*/

	return ret;
}
EXPORT_SYMBOL(vio_ion_alloc);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Sync cache data to ddr;
 * @param[in] *buffer: point to struct vio_buffer instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_ion_sync_for_device(const struct vio_buffer *vbuf)
{
	s32 i, j;
	u64 paddr;
	u32 plane_count;
	size_t length;
	const struct vbuf_group_info *group_info;
	struct device dev = {0};

	group_info = &vbuf->group_info;
	if (vbuf->ion_cachesync == 1u) {
		for (i = 0; i < HBN_LAYER_MAXIMUM; i++) {
			if ((1 << i & group_info->bit_map) == 0)
				continue;
			plane_count = group_info->info[i].buf_attr.planecount;
			if (group_info->is_contig)
				plane_count = 1;

			for (j = 0; j < plane_count; j++) {
				paddr = group_info->info[i].paddr[j];
				length =group_info->info[i].planeSize[j];
				dma_sync_single_for_device(&dev, paddr, length, DMA_TO_DEVICE);
			}
			vio_dbg("%s\n", __func__);/*PRQA S 0685,1294*/
		}
	}
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Sync ddr data to cache;
 * @param[in] *buffer: point to struct vio_buffer instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_ion_sync_for_cpu(const struct vio_buffer *vbuf)
{
	s32 i, j;
	u64 paddr;
	u32 plane_count;
	size_t length;
	const struct vbuf_group_info *group_info;
	struct device dev = {0};

	group_info = &vbuf->group_info;
	if (vbuf->ion_cachesync == 1u) {
		for (i = 0; i < HBN_LAYER_MAXIMUM; i++) {
			if ((1 << i & group_info->bit_map) == 0)
				continue;
			plane_count = group_info->info[i].buf_attr.planecount;
			if (group_info->is_contig)
				plane_count = 1;

			for (j = 0; j < group_info->info[i].buf_attr.planecount; j++) {
				paddr = group_info->info[i].paddr[j];
				length =group_info->info[i].planeSize[j];
				dma_sync_single_for_device(&dev, paddr, length, DMA_FROM_DEVICE);
			}
			vio_dbg("%s\n", __func__);/*PRQA S 0685,1294*/
		}
	}
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Transfer ion share id to ion handle instance;
 * @param[in] *ion_id: ion share id
 * @param[in] plane_num: plane count;
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] **ion_handle: store ion handle point;
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 vio_ion_handle_by_id(struct ion_client *ion_client,
	struct ion_handle **ion_handle, s32 *ion_id, u32 plane_num)
{
	s32 i, j;
	s32 ret = 0;

	for (i = 0; i < plane_num; i++) {
		if (i > 0 && ion_id[i] == 0)
			continue;

		ion_handle[i] = ion_import_dma_buf_with_shareid(ion_client, ion_id[i]);
		if (IS_ERR(ion_handle[i])) {
			vio_err("%s failed ion_fd[%d] %d\n", __func__, i, ion_id[i]);
			for (j = i - 1; j >= 0; j--)
				ion_free(ion_client, ion_handle[j]);
			ret = -EFAULT;
			break;
		}
		vio_dbg("%s: fd[%d] %d ion_handle[%d] 0x%p\n", __func__, i, ion_id[i], i, ion_handle[i]);
	}

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Free alloced ion buffer of one layer in vio frame instance;
 * @param[in] *info: piont to struct vbuf_info instance;
 * @param[in] **ion_handle: point to struct ion_handle array;
 * @param[in] mmap:1 means mapped, 0 means ummapped;
 * @retval {*}
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void free_vbuf(struct ion_client *ion_client, struct ion_handle **ion_handle,
	struct vbuf_info *info, u8 mmap, u8 is_contig)
{
	u32 i;
	u32 plane_count;
	struct vbuf_attr *buf_attr;

	buf_attr = &info->buf_attr;
	plane_count = buf_attr->planecount;
	if (is_contig == 1)
		plane_count = 1;

	for (i = 0; i < plane_count; i++) {
		if (ion_handle[i] != NULL) {
			if (mmap == 1u)
				ion_unmap_kernel(ion_client, ion_handle[i]);
			ion_free(ion_client, ion_handle[i]);
			ion_handle[i] = NULL;
		}
	}

	if (ion_handle[VIO_META_PLANE] != NULL) {
		ion_unmap_kernel(ion_client, ion_handle[VIO_META_PLANE]);
		ion_free(ion_client, ion_handle[VIO_META_PLANE]);
		ion_handle[VIO_META_PLANE] = NULL;
	}
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Get ion handle by ion id and store ion handle in vio frame instance;
 * @param[in] *frame: point to struct vio_frame instance;
 * @param[in] *ion_id: ion share id;
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vio_handle_ext_buffer(struct vio_buffer *vbuf, s32 *ion_id)
{
	s32 ret = 0;
	s32 i, j;
	u32 plane_count;
	struct vbuf_group_info *group_info;
	struct vio_ion_priv *ion_priv;
	struct ion_client *ion_client;

	if (vbuf->ion_priv == NULL) {
		vbuf->ion_priv = osal_kzalloc(sizeof(struct vio_ion_priv), GFP_ATOMIC);
		if (vbuf->ion_priv == NULL) {
			vio_err("%s: ion_priv osal_kzalloc failed\n", __func__);
			return -ENOMEM;
		}
	}

	ion_client = vio_get_ion_client(vbuf->group_info.dev_num);;
	ion_priv = (struct vio_ion_priv *)vbuf->ion_priv;
	group_info = &vbuf->group_info;
	for (i = 0; i < HBN_LAYER_MAXIMUM; i++) {
		if ((1 << i & group_info->bit_map) != 0)
			break;
	}// one buffer selected;

	if (i == HBN_LAYER_MAXIMUM) {
		vio_err("%s: bit_map(0x%x) is wrong\n", __func__, group_info->bit_map);
		return -EFAULT;
	}

	plane_count = group_info->info[i].buf_attr.planecount;
	if (group_info->is_contig == 1)
		plane_count = 1;

	ret = vio_ion_handle_by_id(ion_client, ion_priv->ion_handle[i], ion_id, plane_count);
	if (ret < 0) {
		vio_err("[F%d] %s: failed ret %d\n", group_info->index, __func__, ret);
		for (j = i - 1; j >= 0; j--) {
			if ((1 << j & group_info->bit_map) == 0)
				continue;
			free_vbuf(ion_client, ion_priv->ion_handle[j],
				&group_info->info[j], 0, group_info->is_contig);
		}
		osal_kfree(vbuf->ion_priv);
		vbuf->ion_priv = NULL;
		return ret;
	}
	vbuf->ion_alloced = 1;
	vio_dbg("[F%d] %s: done\n", group_info->index, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Free all alloced ion buffer in vio frame instance;
 * @param[in] *buffer:point to struct vio_buffer instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_ion_free(struct vio_buffer *vbuf)
{
	u32 i;
	struct vbuf_group_info *group_info;
	struct vio_ion_priv *ion_priv;
	struct ion_client *ion_client;

	if (vbuf == NULL) {
		vio_err("%s: vbuf is NULL\n", __func__);
		return;
	}

	if (vbuf->ion_alloced == 0) {
		vio_dbg("%s: vio buffer not alloc or already free\n", __func__);
		return;
	}

	ion_client = vio_get_ion_client(vbuf->group_info.dev_num);;
	ion_priv = (struct vio_ion_priv *)vbuf->ion_priv;
	group_info = &vbuf->group_info;
	for (i = 0; i < HBN_LAYER_MAXIMUM; i++) {
		if ((1 << i & group_info->bit_map) == 0)
			continue;
		free_vbuf(ion_client, ion_priv->ion_handle[i],
			&group_info->info[i], vbuf->ion_mmap, group_info->is_contig);
		vio_dbg("[F%d] %s: L%d free done\n", group_info->index,  __func__, i);/*PRQA S 0685,1294*/
	}
	osal_kfree(vbuf->ion_priv);
	vbuf->ion_priv = NULL;
	vbuf->ion_alloced = 0;
	vbuf->ion_cached = 0;
	vbuf->ion_mmap = 0;
	vio_dbg("[F%d] %s: done\n", group_info->index,  __func__);/*PRQA S 0685,1294*/
}
EXPORT_SYMBOL(vio_ion_free);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Iommu map vio_buffer
 * @param[in] *iommu_dev: point to structe device instance;
 * @param[in] *buffer: point to struct vio_buffer instance;
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vio_iommu_map(void *iommu_dev, struct vio_buffer *vbuf)
{
	s32 ret = 0;
	s32 i, j;
	u32 plane_count;
	struct vbuf_group_info *group_info;
	struct vbuf_attr *buf_attr;
	struct vio_ion_priv *ion_priv;
	struct ion_client *ion_client;

	if (vbuf == NULL) {
		vio_err("%s: vbuf is NULL\n", __func__);
		ret = -EFAULT;
		goto err;
	}

	if (vbuf->ion_alloced == 0u) {
		vio_err("%s: vio buffer not alloc or free\n", __func__);
		ret = -EFAULT;
		goto err;
	}

	if (vbuf->iommu_map == 1) {
		vio_err("%s: vio buffer double iommu map\n", __func__);
		ret = -EFAULT;
		goto err;
	}

	ion_client = vio_get_ion_client(vbuf->group_info.dev_num);;
	ion_priv = (struct vio_ion_priv *)vbuf->ion_priv;
	group_info = &vbuf->group_info;
	for (i = 0; i < HBN_LAYER_MAXIMUM; i++) {
		if ((1 << i & group_info->bit_map) == 0)
			continue;
		buf_attr = &group_info->info[i].buf_attr;
		plane_count = buf_attr->planecount;
		if (group_info->is_contig == 1)
			plane_count = 1;

		for (j = 0; j < plane_count; j++) {
			ret = ion_iommu_map_ion_handle(iommu_dev, &ion_priv->ion_data[i][j],
								ion_client, ion_priv->ion_handle[i][j],
								IOMMU_READ | IOMMU_WRITE);
			if (ret < 0) {
				vio_err("[F%d] %s: Failed to map ion handle.", group_info->index, __func__);
				for (j = j - 1; j >= 0; j--)
					ion_iommu_unmap_ion_handle(iommu_dev, &ion_priv->ion_data[i][j]);
				goto unmap;
			}

			vbuf->iommu_paddr[i][j] = (u32)ion_priv->ion_data[i][j].dma_addr;
			vio_dbg("[F%d] %s: phaddr = 0x%llx, map_addr 0x%llx\n", group_info->index,
					__func__, group_info->info[i].paddr[j], ion_priv->ion_data[i][j].dma_addr);
		}

		if (group_info->is_contig == 1) {
			for (j = 1; j < buf_attr->planecount; j++)
				vbuf->iommu_paddr[i][j] = vbuf->iommu_paddr[i][j - 1] +
					group_info->info[i].planeSize[j - 1];
		}
	}
	vbuf->iommu_map = 1;

	return ret;
unmap:
	for (i = i - 1; i >= 0; i--) {
		if ((1 << i & group_info->bit_map) == 0)
			continue;
		buf_attr = &group_info->info[i].buf_attr;
		plane_count = buf_attr->planecount;
		if (group_info->is_contig)
			plane_count = 1;
		for (j = 0; j < plane_count; j++)
			ion_iommu_unmap_ion_handle(iommu_dev, &ion_priv->ion_data[i][j]);
	}
err:
	return ret;
}
EXPORT_SYMBOL(vio_iommu_map);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Iommu ummap vio_buffer
 * @param[in] *iommu_dev: point to structe device instance;
 * @param[in] *buffer: point to struct vio_buffer instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_iommu_unmap(void *iommu_dev, struct vio_buffer *vbuf)
{
	s32 i, j;
	u32 plane_count;
	struct vbuf_group_info *group_info;
	struct vbuf_attr *buf_attr;
	struct vio_ion_priv *ion_priv;

	if (vbuf == NULL) {
		vio_err("%s: vbuf is NULL\n", __func__);
		return;
	}

	if (vbuf->ion_alloced == 0u) {
		vio_dbg("%s: vio buffer not alloc or free\n", __func__);
		return;
	}

	if (vbuf->iommu_map == 0) {
		vio_dbg("%s: vio buffer not iommu map\n", __func__);
		return;
	}

	ion_priv = (struct vio_ion_priv *)vbuf->ion_priv;
	group_info = &vbuf->group_info;
	for (i = 0; i < HBN_LAYER_MAXIMUM; i++) {
		if ((1 << i & group_info->bit_map) == 0)
			continue;
		buf_attr = &group_info->info[i].buf_attr;
		plane_count = buf_attr->planecount;
		if (group_info->is_contig)
			plane_count = 1;

		for (j = 0; j < plane_count; j++)
			ion_iommu_unmap_ion_handle(iommu_dev, &ion_priv->ion_data[i][j]);
	}
	vbuf->iommu_map = 0;
}
EXPORT_SYMBOL(vio_iommu_unmap);/*PRQA S 0605,0307*/