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

int mem_alloc(struct device *dev, struct list_head *list, struct mem_buf *buf)
{
	struct _mem_buf *_buf;
	const u32 ion_heap_mask = ION_HEAP_TYPE_CMA_RESERVED_MASK;
	const u32 ion_flags = ION_FLAG_CACHED_NEEDS_SYNC | ION_FLAG_CACHED;

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
		return -EFAULT;
	}
	ion_phys(g_ion_client, _buf->ion_handle->id, &_buf->addr, &_buf->size);
	_buf->vaddr = ion_map_kernel(g_ion_client, _buf->ion_handle);

	if (!_buf->vaddr) {
		ion_free(g_ion_client, _buf->ion_handle);
		return -ENOMEM;
	}

	buf->addr = _buf->addr;
	list_add_tail(&_buf->entry, list);
	return 0;
}
EXPORT_SYMBOL(mem_alloc);

int mem_free(struct device *dev, struct list_head *list, struct mem_buf *buf)
{
	struct _mem_buf *b, *_buf = NULL;

	if (!dev || !list || !buf || !buf->addr || !buf->size)
		return -EINVAL;

	list_for_each_entry(b, list, entry) {
		if (b->addr == buf->addr && b->size == buf->size) {
			_buf = b;
			break;
		}
	}

	if (unlikely(!_buf))
		return -EINVAL;

	ion_unmap_kernel(g_ion_client, _buf->ion_handle);
	ion_free(g_ion_client, _buf->ion_handle);
	list_del(&_buf->entry);
	devm_kfree(dev, _buf);
	return 0;
}
EXPORT_SYMBOL(mem_free);

int mem_free_all(struct device *dev, struct list_head *list)
{
	struct _mem_buf *_buf;

	if (!dev || !list)
		return -EINVAL;

	while (!list_empty(list)) {
		_buf = list_first_entry(list, struct _mem_buf, entry);
		ion_unmap_kernel(g_ion_client, _buf->ion_handle);
		ion_free(g_ion_client, _buf->ion_handle);
		list_del(&_buf->entry);
		devm_kfree(dev, _buf);
	}
	return 0;
}
EXPORT_SYMBOL(mem_free_all);

int mem_mmap(struct device *dev, struct list_head *list,
	     struct vm_area_struct *vma)
{
	return 0;
}
EXPORT_SYMBOL(mem_mmap);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
