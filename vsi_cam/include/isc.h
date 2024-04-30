/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Author: Chao Fang <chao.fang@verisilicon.com>
 */

#ifndef _ISC_H_
#define _ISC_H_

struct isc_handle;
struct mem_buf;

struct isc_notifier_ops {
	void (*bound)(struct isc_handle *isc, void *arg);
	void (*unbind)(void *arg);
	s32  (*got)(void *msg, u32 len, void *arg);
};

struct isc_post_param {
	void *msg;
	u32 msg_len;
	struct mem_buf *extra;
	spinlock_t *lock; /* lock for post */
	bool sync;
};

int isc_register(u32 uid, struct isc_notifier_ops *ops, void *arg);

int isc_unregister(u32 uid);

int isc_post(struct isc_handle *isc, struct isc_post_param *param);

void isc_get(struct isc_handle *isc);

void isc_put(struct isc_handle *isc);

void *isc_alloc_extra_buf(struct isc_handle *isc, struct mem_buf *buf);

void isc_free_extra_buf(struct isc_handle *isc, struct mem_buf *buf);

void *isc_get_extra_data(struct isc_handle *isc, struct mem_buf *ext);

void isc_put_extra_data(struct isc_handle *isc, struct mem_buf *ext);

#endif /* _ISC_H_ */
