// SPDX-License-Identifier: GPL-2.0
/*
 * List processing primitives.
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Author:
 *	Lakshmi Sankar <lakshmisankar-t@ti.com>
 */

#include "lst.h"

#ifndef NULL
#define NULL ((void *)0)
#endif

void lst_add(struct lst_t *list, void *item)
{
	if (!list->first) {
		list->first = item;
		list->last = item;
	} else {
		*list->last = item;
		list->last = item;
	}
	*((void **)item) = NULL;
}

void lst_addhead(struct lst_t *list, void *item)
{
	if (!list->first) {
		list->first = item;
		list->last = item;
		*((void **)item) = NULL;
	} else {
		*((void **)item) = list->first;
		list->first = item;
	}
}

int lst_empty(struct lst_t *list)
{
	if (!list->first)
		return 1;
	else
		return 0;
}

void *lst_first(struct lst_t *list)
{
	return list->first;
}

void lst_init(struct lst_t *list)
{
	list->first = NULL;
	list->last = NULL;
}

void *lst_last(struct lst_t *list)
{
	return list->last;
}

void *lst_next(void *item)
{
	return *((void **)item);
}

void *lst_removehead(struct lst_t *list)
{
	void **temp = list->first;

	if (temp) {
		list->first = *temp;
		if (!list->first)
			list->last = NULL;
	}
	return temp;
}

void *lst_remove(struct lst_t *list, void *item)
{
	void **p;
	void **q;

	p = (void **)list;
	q = *p;
	while (q) {
		if (q == item) {
			*p = *q;
			if (list->last == q)
				list->last = p;
			return item;
		}
		p = q;
		q = *p;
	}

	return NULL;
}

int lst_check(struct lst_t *list, void *item)
{
	void **p;
	void **q;

	p = (void **)list;
	q = *p;
	while (q) {
		if (q == item)
			return 1;
		p = q;
		q = *p;
	}

	return 0;
}
