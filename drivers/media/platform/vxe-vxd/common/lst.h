/* SPDX-License-Identifier: GPL-2.0 */
/*
 * List processing primitives.
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Author:
 *	Lakshmi Sankar <lakshmisankar-t@ti.com>
 */
#ifndef __LIST_H__
#define __LIST_H__

#include <linux/types.h>

struct lst_t {
	void **first;
	void **last;
};

void lst_add(struct lst_t *list, void *item);
void lst_addhead(struct lst_t *list, void *item);

/**
 * lst_empty- Is list empty?
 * @list: pointer to list
 */
int  lst_empty(struct lst_t *list);
void *lst_first(struct lst_t *list);
void lst_init(struct lst_t *list);
void *lst_last(struct lst_t *list);
void *lst_next(void *item);
void *lst_remove(struct lst_t *list, void *item);
void *lst_removehead(struct lst_t *list);
int lst_check(struct lst_t *list, void *item);

#endif /* __LIST_H__ */
