/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Utility module for doubly linked queues.
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Lakshmi Sankar <lakshmisankar-t@ti.com>
 */
#ifndef DQ_H
#define DQ_H

/* dq structure */
struct dq_linkage_t {
	struct dq_linkage_t *fwd;
	struct dq_linkage_t *back;
};

/* Function Prototypes */
void dq_addafter(void *predecessor, void *item);
void dq_addbefore(void *successor, void *item);
void dq_addhead(struct dq_linkage_t *queue, void *item);
void dq_addtail(struct dq_linkage_t *queue, void *item);
int dq_empty(struct dq_linkage_t *queue);
void *dq_first(struct dq_linkage_t *queue);
void *dq_last(struct dq_linkage_t *queue);
void dq_init(struct dq_linkage_t *queue);
void dq_move(struct dq_linkage_t *from, struct dq_linkage_t *to);
void *dq_next(void *item);
void *dq_previous(void *item);
void dq_remove(void *item);
void *dq_removehead(struct dq_linkage_t *queue);
void *dq_removetail(struct dq_linkage_t *queue);

#endif /* #define DQ_H */
