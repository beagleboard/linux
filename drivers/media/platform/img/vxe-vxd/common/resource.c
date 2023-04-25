// SPDX-License-Identifier: GPL-2.0
/*
 * VXD DEC Resource manager implementations
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

#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "dq.h"
#include "img_errors.h"
#include "lst.h"
#include "resource.h"

struct resource_list_elem {
	struct dq_linkage_t	link;
	void *item;
	unsigned int id;
	unsigned int *refcnt;
};

/*
 * marks an item as used by incrementing the reference count
 */
int resource_item_use(unsigned int *refcnt)
{
	if (refcnt)
		(*refcnt)++;

	return 0;
}

/*
 * returns an item by decrementing the reference count
 */
void resource_item_return(unsigned int *refcnt)
{
	if (refcnt && *refcnt > 0)
		(*refcnt)--;
}

/*
 * releases an item by setting reference count to 1 (original owner)
 */
int resource_item_release(unsigned int *refcnt)
{
	if (refcnt)
		*refcnt = 1;

	return 0;
}

/*
 * indicates whether an item is free to be used (no owners)
 */
int resource_item_isavailable(unsigned int *refcnt)
{
	if (refcnt)
		return (*refcnt == 0) ? 1 : 0;
	else
		return 0;
}

/*
 * adds an item (and associated id) to a resource list
 */
int resource_list_add_img(struct lst_t *list, void *item, unsigned int id, unsigned int *refcnt)
{
	struct resource_list_elem *listelem = NULL;
	int bfound = 0;
	unsigned int result = 0;

	if (!list || !item) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error;
	}

	/*
	 * Decrement the reference count on the item
	 * to signal that the owner has relinquished it.
	 */
	resource_item_return(refcnt);

	/*
	 *  Determine whether this buffer is already in the list.
	 */
	listelem = lst_first(list);
	while (listelem) {
		if (listelem->item == item) {
			bfound = 1;
			break;
		}

		listelem = lst_next(listelem);
	}

	if (!bfound) {
		/*
		 * allocate the image buffer list element structure.
		 */
		listelem = kmalloc(sizeof(*(listelem)), GFP_KERNEL);
		if (!listelem) {
			result = IMG_ERROR_OUT_OF_MEMORY;
			goto error;
		}
		memset(listelem, 0, sizeof(*(listelem)));

		/*
		 * setup the list element.
		 */
		listelem->item = item;
		listelem->id = id;
		listelem->refcnt = refcnt;

		/*
		 * add the element to the list.
		 */
		lst_add(list, (void *)listelem);
	}

	return 0;

error:
	return result;
}

/*
 * obtains pointer to item at head of resource list
 */
void *resource_list_pickhead(struct lst_t *list)
{
	struct resource_list_elem *listelem = NULL;
	void *item = NULL;

	if (!list)
		goto error;
	/*
	 * peek the head item of the list.
	 */
	listelem = lst_first(list);
	if (listelem)
		item = listelem->item;

error:
	return item;
}

/*
 * removes item from resource list
 */
int resource_list_remove(struct lst_t *list, void *item)
{
	struct resource_list_elem *listelem = NULL;
	unsigned int result = 0;

	if (!list || !item) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error;
	}

	/*
	 * find the specified item in the list.
	 */
	listelem = lst_first(list);
	while (listelem) {
		if (listelem->item == item) {
			if (*listelem->refcnt != 0)
				pr_warn("item remove from list still in use\n");

			/*
			 * Remove the item from the list.
			 */
			lst_remove(list, listelem);
			/*
			 * Free the stream unit queue element.
			 */
			kfree(listelem);
			listelem = NULL;
			return 0;
		}

		listelem = lst_next(listelem);
	}

#if defined(DEBUG_DECODER_DRIVER) || defined(DEBUG_ENCODER_DRIVER)
	pr_info("item could not be located to remove from RESOURCE list\n");
#endif

	return IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;

error:
	return result;
}

/*
 * resource_list_removehead - removes item at head of resource list
 * @list: head of resource list
 */
void *resource_list_removehead(struct lst_t *list)
{
	struct resource_list_elem *listelem = NULL;
	void *item = NULL;

	if (!list)
		goto error;

	/*
	 * peek the head item of the list.
	 */
	listelem = lst_removehead(list);
	if (listelem) {
		item = listelem->item;
		kfree(listelem);
		listelem = NULL;
	}

error:
	return item;
}

/*
 * removes next available item from resource list.
 * item is freed if no longer used
 */
int resource_list_remove_nextavail(struct lst_t *list,
				   resource_pfn_freeitem fn_freeitem,
				   void *free_cb_param)
{
	struct resource_list_elem *listelem = NULL;
	unsigned int result = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;

	if (!list) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error;
	}

	/*
	 * find the next unused item in the list.
	 */
	listelem = lst_first(list);
	while (listelem) {
		if (resource_item_isavailable(listelem->refcnt)) {
			resource_item_return(listelem->refcnt);

			if (*listelem->refcnt == 0) {
				if (fn_freeitem)
					fn_freeitem(listelem->item, free_cb_param);
				else
					kfree(listelem->item);

				listelem->item = NULL;
			}

			/*
			 * get the next element from the list.
			 */
			lst_remove(list, listelem);

			/*
			 * free the buffer list element.
			 */
			kfree(listelem);
			listelem = NULL;

			result = 0;
			break;
		}

		listelem = lst_next(listelem);
	}

	if (result == IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE)
		pr_debug("failed to locate an available resource element to remove\n");

error:
	return result;
}

/*
 * obtains pointer to an available item from the resource list
 */
void *resource_list_get_avail(struct lst_t *list)
{
	struct resource_list_elem *listelem = NULL;
	void *item = NULL;

	if (!list)
		goto error;

	/*
	 * find the next unused item in the list.
	 */
	listelem = lst_first(list);
	while (listelem) {
		if (resource_item_isavailable(listelem->refcnt)) {
			resource_item_use(listelem->refcnt);
			item = listelem->item;
			break;
		}
		listelem = lst_next(listelem);
	}

error:
	return item;
}

/*
 * signal duplicate use of specified item with resource list
 */
void *resource_list_reuseitem(struct lst_t *list, void *item)
{
	struct resource_list_elem *listelem = NULL;
	void *ret_item  = NULL;

	if (!list || !item)
		goto error;

	/*
	 * find the specified item in the list.
	 */
	listelem = lst_first(list);

	while (listelem) {
		if (listelem->item == item) {
			resource_item_use(listelem->refcnt);
			ret_item = item;
			break;
		}

		listelem = lst_next(listelem);
	}

error:
	return ret_item;
}

/*
 * obtain pointer to item from resource list with id
 */
void *resource_list_getbyid(struct lst_t *list, unsigned int id)
{
	struct resource_list_elem *listelem = NULL;
	void *item  = NULL;

	if (!list)
		goto error;

	/*
	 * find the next unused buffer in the list.
	 */
	listelem = lst_first(list);
	while (listelem) {
		if (listelem->id == id) {
			resource_item_use(listelem->refcnt);
			item = listelem->item;
			break;
		}

		listelem = lst_next(listelem);
	}

error:
	return item;
}

/*
 * obtain the number of available (unused) items within list.
 */
int resource_list_getnumavail(struct lst_t *list)
{
	struct resource_list_elem *listelem = NULL;
	unsigned int num_items = 0;

	if (!list)
		goto error;

	/*
	 * find the next unused buffer in the list.
	 */
	listelem = lst_first(list);
	while (listelem) {
		if (resource_item_isavailable(listelem->refcnt))
			num_items++;

		listelem = lst_next(listelem);
	}

error:
	return num_items;
}

/*
 * Obtain the number of items within list
 */
int resource_list_getnum(struct lst_t *list)
{
	struct resource_list_elem *listelem = NULL;
	unsigned int num_items = 0;

	if (!list)
		goto error;

	/*
	 * find the next unused buffer in the list.
	 */
	listelem = lst_first(list);
	while (listelem) {
		num_items++;
		listelem = lst_next(listelem);
	}

error:
	return num_items;
}

/*
 * replaces an item (of specified id) within a resource list
 */
int resource_list_replace(struct lst_t *list, void *item, unsigned int id, unsigned int *refcnt,
			  resource_pfn_freeitem fn_freeitem,
			  void *free_cb_param)
{
	struct resource_list_elem *listelem = NULL;
	unsigned int result = 0;

	if (!list || !item) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error;
	}

	/*
	 * determine whether this sequence header is already in the list
	 */
	listelem = lst_first(list);
	while (listelem) {
		if (listelem->id == id) {
			resource_item_return(listelem->refcnt);
			if (*listelem->refcnt == 0) {
				if (fn_freeitem)
					fn_freeitem(listelem->item,
						    free_cb_param);
				else {
					if (!listelem->item)
						kfree(listelem->item);
				}
				listelem->item = NULL;
			}

			lst_remove(list, listelem);
			break;
		}

		listelem = lst_next(listelem);
	}

	if (!listelem) {
		/*
		 * Allocate the sequence header list element structure.
		 */
		listelem = kmalloc(sizeof(*(listelem)), GFP_KERNEL);
		if (!listelem) {
			result = IMG_ERROR_OUT_OF_MEMORY;
			goto error;
		}
		memset(listelem, 0, sizeof(*(listelem)));
	}

	/*
	 * setup the sequence header list element.
	 */
	resource_item_use(refcnt);

	listelem->item = item;
	listelem->id = id;
	listelem->refcnt = refcnt;

	/*
	 * Add the sequence header list element to the sequence header list.
	 */
	lst_add(list, (void *)listelem);

	return 0;

error:
	return result;
}

/*
 * removes all items from a resource list.
 */
int resource_list_empty(struct lst_t *list, unsigned int release_item,
			resource_pfn_freeitem fn_freeitem,
			void *free_cb_param)
{
	struct resource_list_elem *listelem = NULL;
	unsigned int result = 0;

	if (!list) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error;
	}

	/*
	 * remove all the buffer list elements from the image buffer list
	 */
	listelem = lst_removehead(list);
	while (listelem) {
		if (release_item) {
			resource_item_release(listelem->refcnt);
		} else {
			/*
			 * Return and free.
			 */
			resource_item_return(listelem->refcnt);

			if (!listelem->refcnt || *listelem->refcnt == 0) {
				if (fn_freeitem)
					fn_freeitem(listelem->item,
						    free_cb_param);
				else
					kfree(listelem->item);
				listelem->item = NULL;
			}
		}

		/*
		 * free the buffer list element.
		 */
		kfree(listelem);
		listelem = NULL;

		/*
		 * Get the next element from the list.
		 */
		listelem = lst_removehead(list);
	}

	return 0;

error:
	return result;
}

/*
 * obtain the number of pictures within list
 */
int resource_getnumpict(struct lst_t *list)
{
	struct resource_list_elem *listelem = NULL;
	unsigned int num_pict = 0;

	if (!list)
		goto error;

	/*
	 * find the next unused buffer in the list.
	 */
	listelem = lst_first(list);
	while (listelem) {
		num_pict++;
		listelem = lst_next(listelem);
	}

error:
	return num_pict;
}
