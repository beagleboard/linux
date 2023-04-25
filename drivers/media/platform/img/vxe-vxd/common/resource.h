/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD DEC SYSDEV and UI Interface header
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Sunita Nadampalli <sunitan@ti.com>
 *
 * Re-written for upstream
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 */

#ifndef _VXD_RESOURCE_H
#define _VXD_RESOURCE_H

typedef int (*resource_pfn_freeitem)(void *item, void *free_cb_param);

int resource_item_use(unsigned int *refcnt);

void resource_item_return(unsigned int *refcnt);

int resource_item_release(unsigned int *refcnt);

int resource_item_isavailable(unsigned int *refcnt);

int resource_list_add_img(struct lst_t *list, void *item, unsigned int id, unsigned int *refcnt);

void *resource_list_pickhead(struct lst_t *list);

int resource_list_remove(struct lst_t *list, void *item);

/**
 * resource_list_removehead - removes item at head of resource list
 * @list: head of resource list
 */

void *resource_list_removehead(struct lst_t *list);

int resource_list_remove_nextavail(struct lst_t *list,
				   resource_pfn_freeitem fn_freeitem,
				   void *free_cb_param);

void *resource_list_get_avail(struct lst_t *list);

void *resource_list_reuseitem(struct lst_t *list, void *item);

void *resource_list_getbyid(struct lst_t *list, unsigned int id);

int resource_list_getnumavail(struct lst_t *list);

int resource_list_getnum(struct lst_t *list);

int resource_list_replace(struct lst_t *list, void *item, unsigned int id, unsigned int *refcnt,
			  resource_pfn_freeitem fn_freeitem,
			  void *free_cb_param);

int resource_list_empty(struct lst_t *list, unsigned int release_item,
			resource_pfn_freeitem fn_freeitem,
			void *free_cb_param);

int resource_getnumpict(struct lst_t *list);

#endif /* _VXD_RESOURCE_H */
