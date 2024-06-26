// SPDX-License-Identifier: GPL-2.0-only
#include <linux/dma-mapping.h>
#include <linux/module.h>

#include "mem_helper.h"

struct _mem_buf {
	dma_addr_t addr;
	void *vaddr;
	size_t size;
	struct list_head entry;
};

int mem_alloc(struct device *dev, struct list_head *list, struct mem_buf *buf)
{
	struct _mem_buf *_buf;

	if (!dev || !list || !buf || !buf->size)
		return -EINVAL;

	_buf = devm_kzalloc(dev, sizeof(*_buf), GFP_KERNEL);
	if (!_buf)
		return -ENOMEM;

	_buf->size = buf->size;
	_buf->vaddr =
		dma_alloc_coherent(dev, _buf->size, &_buf->addr, GFP_KERNEL);
	if (!_buf->vaddr) {
		devm_kfree(dev, _buf);
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

	dma_free_coherent(dev, _buf->size, _buf->vaddr, _buf->addr);
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
		dma_free_coherent(dev, _buf->size, _buf->vaddr, _buf->addr);
		list_del(&_buf->entry);
		devm_kfree(dev, _buf);
	}
	return 0;
}
EXPORT_SYMBOL(mem_free_all);

int mem_mmap(struct device *dev, struct list_head *list,
	     struct vm_area_struct *vma)
{
	dma_addr_t addr = ((dma_addr_t)vma->vm_pgoff << PAGE_SHIFT);
	size_t size = vma->vm_end - vma->vm_start;
	struct _mem_buf *b, *_buf = NULL;

	if (!addr || !size)
		return -EINVAL;

	list_for_each_entry(b, list, entry) {
		if (b->addr <= addr && b->addr + b->size >= addr + size) {
			_buf = b;
			break;
		}
	}

	if (unlikely(!_buf))
		return -EINVAL;

	vma->vm_pgoff = 0;
	return dma_mmap_coherent(dev, vma, _buf->vaddr, _buf->addr, _buf->size);
}
EXPORT_SYMBOL(mem_mmap);

int mem_cache_flush(struct device *dev, struct list_head *list, struct mem_buf *buf)
{
	struct _mem_buf *b, *_buf = NULL;
	unsigned long pfn;
	struct page *page;
	unsigned long offset;
	size_t size;
	dma_addr_t dma_addr = 0;

	if (!dev || !list || !buf || !buf->addr || !buf->size)
		return -EINVAL;

	list_for_each_entry(b, list, entry) {
		if (b->addr == buf->addr/* && b->size == buf->size*/) {
			_buf = b;
			break;
		}
	}

	if (unlikely(!_buf))
		return -EINVAL;

	pfn = __phys_to_pfn(_buf->addr);
	page = pfn_to_page(pfn);
	offset = offset_in_page(_buf->addr);
	size = PAGE_ALIGN(_buf->size);
	dma_addr = dma_map_page(dev, page, offset, size, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, dma_addr)) {
		// dev_err(dev, "ma map page failed.\n");
		return -EINVAL;
	}

	dma_sync_single_for_device(dev, dma_addr, size, DMA_TO_DEVICE);
	dma_unmap_page(dev, dma_addr, size, DMA_TO_DEVICE);
	return 0;
}
EXPORT_SYMBOL(mem_cache_flush);

int mem_cache_invalid(struct device *dev, struct list_head *list, struct mem_buf *buf)
{
	struct _mem_buf *b, *_buf = NULL;
	unsigned long pfn;
	struct page *page;
	unsigned long offset;
	size_t size;
	dma_addr_t dma_addr = 0;

	if (!dev || !list || !buf || !buf->addr || !buf->size)
		return -EINVAL;

	list_for_each_entry(b, list, entry) {
		if (b->addr == buf->addr/* && b->size == buf->size*/) {
			_buf = b;
			break;
		}
	}

	if (unlikely(!_buf))
		return -EINVAL;

	pfn = __phys_to_pfn(_buf->addr);
	page = pfn_to_page(pfn);
	offset = offset_in_page(_buf->addr);
	size = PAGE_ALIGN(_buf->size);
	dma_addr = dma_map_page(dev, page, offset, size, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, dma_addr)) {
		// dev_err(dev, "ma map page failed.\n");
		return -EINVAL;
	}

	dma_sync_single_for_cpu(dev, dma_addr, size, DMA_FROM_DEVICE);
	dma_unmap_page(dev, dma_addr, size, DMA_FROM_DEVICE);
	return 0;
}
EXPORT_SYMBOL(mem_cache_invalid);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
