/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _JOB_QUEUE_H_
#define _JOB_QUEUE_H_

struct irq_job {
	u32 irq_ctx_index;
};

struct job_queue;

int push_job(struct job_queue *q, struct irq_job *job);
int pop_job(struct job_queue *q, struct irq_job *job);
int remove_job(struct job_queue *q, u32 index);
struct job_queue *create_job_queue(unsigned int nmem);
void destroy_job_queue(struct job_queue *q);
void reset_job_queue(struct job_queue *q);

#endif /* _JOB_QUEUE_H_ */
