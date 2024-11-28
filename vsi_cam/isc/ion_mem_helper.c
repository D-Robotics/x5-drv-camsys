// SPDX-License-Identifier: GPL-2.0-only
#include <linux/ion.h>
#include <hobot_ion_iommu.h>
#include <linux/module.h>

#include "mem_helper.h"

static struct ion_client *g_ion_client;

struct _mem_buf {
	dma_addr_t addr;
	void *vaddr;
	size_t size;
	struct ion_handle *ion_handle;
	struct list_head entry;
};

static int mem_create(void)
{
	int rc = 0;
	struct ion_device *hb_ion_dev;

	hb_ion_dev = hobot_ion_get_ion_device();
	if (!hb_ion_dev) {
		pr_err("%s hb_ion_dev is null.\n", __func__);
		rc = -EFAULT;
	}

	if (!g_ion_client) {
		g_ion_client = ion_client_create(hb_ion_dev, "vsi_cam_drv_ion");
		if (IS_ERR((void *)g_ion_client)) {
			pr_err("%s ion client create failed.\n", __func__);
			rc = -EFAULT;
		}
	}
	return rc;
}

#if 0
static void mem_destroy(void)
{
	ion_client_destroy(g_ion_client);
	g_ion_client = NULL;
}
#endif

int mem_alloc(struct device *dev, struct mem_list *list, struct mem_buf *buf)
{
	struct _mem_buf *_buf;
	const u32 ion_heap_mask = ION_HEAP_TYPE_CMA_RESERVED_MASK;
	//const u32 ion_flags = ION_FLAG_CACHED_NEEDS_SYNC | ION_FLAG_CACHED;
	const u32 ion_flags = 0;
	size_t size;

	if (!dev || !list || !buf || !buf->size)
		return -EINVAL;

	if (mem_create() < 0)
		return -EFAULT;

	_buf = devm_kzalloc(dev, sizeof(*_buf), GFP_KERNEL);
	if (!_buf)
		return -ENOMEM;

	_buf->size = buf->size;
	_buf->ion_handle = ion_alloc(g_ion_client, _buf->size, PAGE_SIZE,
				     ion_heap_mask, ion_flags);
	if (IS_ERR(_buf->ion_handle)) {
		pr_err("%s ion_alloc buf failed\n", __func__);
		devm_kfree(dev, _buf);
		return -EFAULT;
	}
	size = buf->size;
	ion_phys(g_ion_client, _buf->ion_handle->id, &_buf->addr, &size);
	_buf->vaddr = ion_map_kernel(g_ion_client, _buf->ion_handle);

	if (!_buf->vaddr) {
		ion_free(g_ion_client, _buf->ion_handle);
		devm_kfree(dev, _buf);
		return -ENOMEM;
	}

	buf->addr = _buf->addr;
	mutex_lock(&list->lock);
	list_add_tail(&_buf->entry, &list->list);
	mutex_unlock(&list->lock);
	return 0;
}
EXPORT_SYMBOL(mem_alloc);

int mem_free(struct device *dev, struct mem_list *list, struct mem_buf *buf)
{
	struct _mem_buf *b, *_buf = NULL;

	if (!dev || !list || !buf || !buf->addr || !buf->size)
		return -EINVAL;

	mutex_lock(&list->lock);
	list_for_each_entry(b, &list->list, entry) {
		if (b->addr == buf->addr && b->size == buf->size) {
			_buf = b;
			break;
		}
	}
	mutex_unlock(&list->lock);

	if (unlikely(!_buf))
		return -EINVAL;

	ion_unmap_kernel(g_ion_client, _buf->ion_handle);
	ion_free(g_ion_client, _buf->ion_handle);
	mutex_lock(&list->lock);
	list_del(&_buf->entry);
	mutex_unlock(&list->lock);
	devm_kfree(dev, _buf);
	return 0;
}
EXPORT_SYMBOL(mem_free);

int mem_free_all(struct device *dev, struct mem_list *list)
{
	struct _mem_buf *_buf;

	if (!dev || !list)
		return -EINVAL;

	mutex_lock(&list->lock);
	while (!list_empty(&list->list)) {
		_buf = list_first_entry(&list->list, struct _mem_buf, entry);
		list_del(&_buf->entry);
		mutex_unlock(&list->lock);
		ion_unmap_kernel(g_ion_client, _buf->ion_handle);
		ion_free(g_ion_client, _buf->ion_handle);
		devm_kfree(dev, _buf);
		mutex_lock(&list->lock);
	}
	mutex_unlock(&list->lock);
	return 0;
}
EXPORT_SYMBOL(mem_free_all);

int mem_mmap(struct device *dev, struct mem_list *list,
	     struct vm_area_struct *vma)
{
	return 0;
}
EXPORT_SYMBOL(mem_mmap);

void *get_virt_addr(struct device *dev, struct mem_list *list,
		    struct mem_buf *buf)
{
	struct _mem_buf *b, *_buf = NULL;

	if (!dev || !list || !buf || !buf->addr || !buf->size)
		return ERR_PTR(-EINVAL);

	mutex_lock(&list->lock);
	list_for_each_entry(b, &list->list, entry) {
		if (b->addr == buf->addr && b->size == buf->size) {
			_buf = b;
			break;
		}
	}
	mutex_unlock(&list->lock);

	if (unlikely(!_buf))
		return ERR_PTR(-EINVAL);

	return _buf->vaddr;
}
EXPORT_SYMBOL(get_virt_addr);

int mem_cache_flush(struct device *dev, struct mem_list *list, struct mem_buf *buf)
{
	struct _mem_buf *b, *_buf = NULL;

	if (!dev || !list || !buf || !buf->addr || !buf->size)
		return -EINVAL;

	mutex_lock(&list->lock);
	list_for_each_entry(b, &list->list, entry) {
		if (b->addr == buf->addr/* && b->size == buf->size*/) {
			_buf = b;
			break;
		}
	}
	mutex_unlock(&list->lock);

	if (unlikely(!_buf))
		return -EINVAL;

	dma_sync_single_for_device(g_ion_client->dev->dev.this_device,
						_buf->addr, _buf->size, DMA_TO_DEVICE);

	return 0;
}
EXPORT_SYMBOL(mem_cache_flush);

int mem_cache_invalid(struct device *dev, struct mem_list *list, struct mem_buf *buf)
{
	struct _mem_buf *b, *_buf = NULL;

	if (!dev || !list || !buf || !buf->addr || !buf->size)
		return -EINVAL;

	mutex_lock(&list->lock);
	list_for_each_entry(b, &list->list, entry) {
		if (b->addr == buf->addr/* && b->size == buf->size*/) {
			_buf = b;
			break;
		}
	}
	mutex_unlock(&list->lock);

	if (unlikely(!_buf))
		return -EINVAL;

	dma_sync_single_for_cpu(g_ion_client->dev->dev.this_device, _buf->addr,
					_buf->size, DMA_FROM_DEVICE);

	return 0;
}
EXPORT_SYMBOL(mem_cache_invalid);

/* referring to ion_iommu_map_ion_phys in hobot_ion_iommu.c */
int mem_iommu_map(struct device *dev, phys_addr_t phys_addr, size_t size,
		  phys_addr_t *iova)
{
	dma_addr_t start;
	phys_addr_t phys, mapped_phys;
	size_t len;
	struct lite_mmu_iommu *iommu = dev_iommu_priv_get(dev);
	struct iommu_domain *domain;
	const int prot = IOMMU_READ | IOMMU_WRITE;
	int rc;

	if (IS_ERR_OR_NULL(iommu)) {
		dev_warn_once(dev,
			"not attached to any iommu, using physical address!\n");
		*iova = phys_addr;
		return 0;
	}

	if (!iommu->domain)
		return -EINVAL;

	domain = iommu->domain;

	phys = phys_addr;
	start = phys & dma_get_mask(dev);
	len = PAGE_ALIGN(size);
	dev_dbg(dev, "mapping phys:%#llx(%#lx)\n", phys, len);
	if (iommu->iommu.ops &&
	    iommu->iommu.ops->default_domain_ops &&
	    iommu->iommu.ops->default_domain_ops->map)
		rc = iommu->iommu.ops->default_domain_ops->map
				(domain, start, phys, len, prot, GFP_KERNEL);
	else
		rc = -EINVAL;
	if (rc < 0) {
		dev_err(dev, "iommu map failed (err=%d)\n", rc);
		return rc;
	}

	mapped_phys = iommu_iova_to_phys(domain, start);
	if (mapped_phys != phys) {
		(void)mem_iommu_unmap(dev, start, len);
		dev_err(dev,
			"iommu map failed (size: %#zx, mapped: 0x%llx, phys: 0x%llx)\n",
			len, mapped_phys, phys);
		return -EINVAL;
	}
	*iova = (dma_addr_t)start;
	return 0;
}
EXPORT_SYMBOL(mem_iommu_map);

int mem_iommu_unmap(struct device *dev, phys_addr_t iova, size_t size)
{
	size_t len;
	struct lite_mmu_iommu *iommu = dev_iommu_priv_get(dev);
	struct iommu_domain *domain;
	struct iommu_iotlb_gather iotlb_gather;
	dma_addr_t start;
	int rc;

	if (IS_ERR_OR_NULL(iommu)) {
		dev_warn_once(dev,
			"not attached to any iommu, no need to unmap anything!\n");
		return 0;
	}

	if (!iommu->domain)
		return -EINVAL;

	domain = iommu->domain;
	start = (dma_addr_t)iova;
	len = PAGE_ALIGN(size);
	iommu_iotlb_gather_init(&iotlb_gather);
	if (iommu->iommu.ops &&
	    iommu->iommu.ops->default_domain_ops &&
	    iommu->iommu.ops->default_domain_ops->unmap)
		rc = iommu->iommu.ops->default_domain_ops->unmap
				(domain, start, len, &iotlb_gather);
	else
		rc = -EINVAL;
	if (rc < 0)
		dev_err(dev, "iommu unmap failed (err=%d)\n", rc);
	iommu_iotlb_sync(domain, &iotlb_gather);
	return rc;
}
EXPORT_SYMBOL(mem_iommu_unmap);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
