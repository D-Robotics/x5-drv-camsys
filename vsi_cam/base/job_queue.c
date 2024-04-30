// SPDX-License-Identifier: GPL-2.0-only
#include <linux/slab.h>

#include "job_queue.h"

struct job_queue {
	struct list_head idle_queue;
	struct list_head done_queue;
	spinlock_t lock; /* lock for job queue */
	struct job *jobs;
	void *job_data_space;
};

struct job {
	void *data;
	struct list_head entry;
};

int push_job(struct job_queue *q, struct irq_job *ij)
{
	struct job *job;
	unsigned long flags;
	int rc = 0;

	if (!q || !ij)
		return -EINVAL;

	spin_lock_irqsave(&q->lock, flags);

	job = list_first_entry_or_null(&q->idle_queue, struct job, entry);
	if (!job) {
		rc = -EBUSY;
		goto _exit;
	}

	memcpy(job->data, ij, sizeof(*ij));

	list_del(&job->entry);

	list_add_tail(&job->entry, &q->done_queue);

_exit:
	spin_unlock_irqrestore(&q->lock, flags);
	return rc;
}

int pop_job(struct job_queue *q, struct irq_job *ij)
{
	struct job *job;
	unsigned long flags;
	int rc = 0;

	if (!q || !ij)
		return -EINVAL;

	spin_lock_irqsave(&q->lock, flags);

	job = list_first_entry_or_null(&q->done_queue, struct job, entry);
	if (!job) {
		rc = -EBUSY;
		goto _exit;
	}

	memcpy(ij, job->data, sizeof(*ij));

	list_del(&job->entry);

	list_add_tail(&job->entry, &q->idle_queue);

_exit:
	spin_unlock_irqrestore(&q->lock, flags);
	return rc;
}

struct job_queue *create_job_queue(unsigned int nmem)
{
	struct job_queue *q;
	unsigned int i;

	if (!nmem)
		return NULL;

	q = kzalloc(sizeof(*q), GFP_KERNEL);
	if (!q)
		return NULL;

	q->jobs = kcalloc(nmem, sizeof(*q->jobs), GFP_KERNEL);
	if (!q->jobs) {
		kfree(q);
		return NULL;
	}

	q->job_data_space = kcalloc(nmem, sizeof(struct irq_job), GFP_KERNEL);
	if (!q->job_data_space) {
		kfree(q->jobs);
		kfree(q);
		return NULL;
	}

	INIT_LIST_HEAD(&q->idle_queue);
	INIT_LIST_HEAD(&q->done_queue);
	spin_lock_init(&q->lock);

	for (i = 0; i < nmem; i++) {
		q->jobs[i].data = q->job_data_space + i * sizeof(struct irq_job);
		list_add_tail(&q->jobs[i].entry, &q->idle_queue);
	}
	return q;
}

void destroy_job_queue(struct job_queue *q)
{
	unsigned long flags;

	if (!q)
		return;

	spin_lock_irqsave(&q->lock, flags);
	kfree(q->jobs);
	kfree(q->job_data_space);
	spin_unlock_irqrestore(&q->lock, flags);
	kfree(q);
}

void reset_job_queue(struct job_queue *q)
{
	unsigned long flags;

	if (!q)
		return;

	spin_lock_irqsave(&q->lock, flags);
	list_splice_tail_init(&q->done_queue, &q->idle_queue);
	spin_unlock_irqrestore(&q->lock, flags);
}
