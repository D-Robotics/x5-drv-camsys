/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _MEM_HELPER_H_
#define _MEM_HELPER_H_

#include <linux/list.h>

#include "mem_helper_uapi.h"

int mem_alloc(struct device *dev, struct list_head *list, struct mem_buf *buf);
int mem_free(struct device *dev, struct list_head *list, struct mem_buf *buf);
int mem_free_all(struct device *dev, struct list_head *list);
int mem_mmap(struct device *dev, struct list_head *list,
	     struct vm_area_struct *vma);
void *get_virt_addr(struct device *dev, struct list_head *list,
		    struct mem_buf *buf);
int mem_cache_flush(struct device *dev, struct list_head *list, struct mem_buf *buf);
int mem_cache_invalid(struct device *dev, struct list_head *list, struct mem_buf *buf);
int mem_iommu_map(struct device *dev, phys_addr_t addr, size_t size, phys_addr_t *iova);
int mem_iommu_unmap(struct device *dev, phys_addr_t iova, size_t size);

#endif /* _MEM_HELPER_H_ */
