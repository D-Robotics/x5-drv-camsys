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

#endif /* _MEM_HELPER_H_ */
