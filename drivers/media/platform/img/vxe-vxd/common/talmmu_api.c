// SPDX-License-Identifier: GPL-2.0
/*
 * TAL MMU Extensions.
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Lakshmi Sankar <lakshmisankar-t@ti.com>
 *
 * Re-written for upstream
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "img_errors.h"
#include "lst.h"
#include "talmmu_api.h"

static int global_init;
static struct lst_t gl_dmtmpl_lst = {0};
static struct mutex *global_lock;

static int talmmu_devmem_free(void *mem_hndl)
{
	struct talmmu_memory *mem = mem_hndl;
	struct talmmu_devmem_heap *mem_heap;

	if (!mem_hndl)
		return IMG_ERROR_INVALID_PARAMETERS;

	mem_heap = mem->devmem_heap;

	if (!mem->ext_dev_virtaddr)
		addr_cx_free(&mem_heap->ctx, "", mem->dev_virtoffset);

	mutex_lock_nested(global_lock, SUBCLASS_TALMMU);

	lst_remove(&mem_heap->memory_list, mem);

	mutex_unlock(global_lock);

	kfree(mem);

	return IMG_SUCCESS;
}

/*
 * talmmu_devmem_heap_empty - talmmu_devmem_heap_empty
 * @devmem_heap_hndl: device memory heap handle
 *
 * This function is used for emptying the device memory heap list
 */
int talmmu_devmem_heap_empty(void *devmem_heap_hndl)
{
	struct talmmu_devmem_heap *devmem_heap = devmem_heap_hndl;

	if (!devmem_heap)
		return IMG_ERROR_INVALID_PARAMETERS;

	while (!lst_empty(&devmem_heap->memory_list))
		talmmu_devmem_free(lst_first(&devmem_heap->memory_list));

	addr_cx_deinitialise(&devmem_heap->ctx);

	return IMG_SUCCESS;
}

/*
 * @Function	talmmu_devmem_heap_destroy
 *
 * @Description	This function is used for freeing the device memory heap
 *
 * @Input
 *
 * @Output
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
static void talmmu_devmem_heap_destroy(void *devmem_heap_hndl)
{
	struct talmmu_devmem_heap *devmem_heap = devmem_heap_hndl;

	talmmu_devmem_heap_empty(devmem_heap_hndl);
	kfree(devmem_heap);
}

/*
 * @Function	talmmu_init
 *
 * @Description	This function is used to initialize the TALMMU component.
 *
 * @Input	None.
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
int talmmu_init(void)
{
	if (!global_init) {
		/* If no mutex associated with this resource */
		if (!global_lock) {
			/* Create one */
			global_lock = kzalloc(sizeof(*global_lock), GFP_KERNEL);
			if (!global_lock)
				return IMG_ERROR_OUT_OF_MEMORY;

			mutex_init(global_lock);
		}

		lst_init(&gl_dmtmpl_lst);
		global_init = 1;
	}

	return IMG_SUCCESS;
}

/*
 * @Function	talmmu_deinit
 *
 * @Description	This function is used to de-initialize the TALMMU component.
 *
 * @Input	None.
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
int talmmu_deinit(void)
{
	struct talmmu_dm_tmpl *t;

	if (global_init) {
		while (!lst_empty(&gl_dmtmpl_lst)) {
			t = (struct talmmu_dm_tmpl *)lst_first(&gl_dmtmpl_lst);
			talmmu_devmem_template_destroy((void *)t);
		}
		mutex_destroy(global_lock);
		kfree(global_lock);
		global_lock = NULL;
		global_init = 0;
	}

	return IMG_SUCCESS;
}

/*
 * @Function	talmmu_devmem_template_create
 *
 * @Description	This function is used to create a device memory template
 *
 * @Input	devmem_info:  A pointer to a talmmu_devmem_info structure.
 *
 * @Output	devmem_template_hndl: A pointer used to return the template
 *		handle
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
int talmmu_devmem_template_create(struct talmmu_devmem_info *devmem_info,
				  void **devmem_template_hndl)
{
	struct talmmu_dm_tmpl *devmem_template;
	struct talmmu_dm_tmpl *tmp_devmem_template;

	if (!devmem_info)
		return IMG_ERROR_INVALID_PARAMETERS;

	devmem_template = kzalloc(sizeof(*devmem_template), GFP_KERNEL);
	if (!devmem_template)
		return IMG_ERROR_OUT_OF_MEMORY;

	devmem_template->devmem_info = *devmem_info;

	lst_init(&devmem_template->devmem_ctx_list);

	mutex_lock_nested(global_lock, SUBCLASS_TALMMU);

	tmp_devmem_template = lst_first(&gl_dmtmpl_lst);
	while (tmp_devmem_template)
		tmp_devmem_template = lst_next(tmp_devmem_template);

	devmem_template->page_num_shift = 12;
	devmem_template->byte_in_pagemask = 0xFFF;
	devmem_template->heap_alignment = 0x400000;
	devmem_template->pagetable_entries_perpage =
		(devmem_template->devmem_info.page_size / sizeof(unsigned int));
	devmem_template->pagetable_num_shift = 10;
	devmem_template->index_in_pagetable_mask = 0x3FF;
	devmem_template->pagedir_num_shift = 22;

	lst_add(&gl_dmtmpl_lst, devmem_template);

	mutex_unlock(global_lock);

	*devmem_template_hndl = devmem_template;

	return IMG_SUCCESS;
}

/*
 * @Function	talmmu_devmem_template_destroy
 *
 * @Description This function is used to obtain the template from the list and
 *		destroy
 *
 * @Input	devmem_tmplt_hndl: Device memory template handle
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
int talmmu_devmem_template_destroy(void *devmem_tmplt_hndl)
{
	struct talmmu_dm_tmpl *dm_tmpl = devmem_tmplt_hndl;
	unsigned int i;

	if (!devmem_tmplt_hndl)
		return IMG_ERROR_INVALID_PARAMETERS;

	while (!lst_empty(&dm_tmpl->devmem_ctx_list))
		talmmu_devmem_ctx_destroy(lst_first(&dm_tmpl->devmem_ctx_list));

	for (i = 0; i < dm_tmpl->num_heaps; i++)
		talmmu_devmem_heap_destroy(dm_tmpl->devmem_heap[i]);

	mutex_lock_nested(global_lock, SUBCLASS_TALMMU);

	lst_remove(&gl_dmtmpl_lst, dm_tmpl);

	mutex_unlock(global_lock);

	kfree(dm_tmpl);

	return IMG_SUCCESS;
}

/*
 * @Function	talmmu_create_heap
 *
 * @Description	This function is used to create a device memory heap
 *
 * @Input
 *
 * @Output
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
static int talmmu_create_heap(void *devmem_tmplt_hndl,
			      struct talmmu_heap_info *heap_info_arg,
			      unsigned char isfull,
			      struct talmmu_devmem_heap **devmem_heap_arg)
{
	struct talmmu_dm_tmpl *devmem_template = devmem_tmplt_hndl;
	struct talmmu_devmem_heap *devmem_heap;

	/* Allocating memory for device memory heap */
	devmem_heap = kzalloc(sizeof(*devmem_heap), GFP_KERNEL);
	if (!devmem_heap)
		return IMG_ERROR_OUT_OF_MEMORY;

	/*
	 * Update the device memory heap structure members
	 * Update the device memory template
	 */
	devmem_heap->devmem_template = devmem_template;
	/* Update the device memory heap information */
	devmem_heap->heap_info = *heap_info_arg;

	/* Initialize the device memory heap list */
	lst_init(&devmem_heap->memory_list);

	/* If full structure required */
	if (isfull) {
		addr_cx_initialise(&devmem_heap->ctx);
		devmem_heap->regions.base_addr = 0;
		devmem_heap->regions.size = devmem_heap->heap_info.size;
		addr_cx_define_mem_region(&devmem_heap->ctx,
					  &devmem_heap->regions);
	}

	*devmem_heap_arg = devmem_heap;

	return IMG_SUCCESS;
}

/*
 * @Function	talmmu_devmem_heap_add
 *
 * @Description	This function is for creating and adding the heap to the
 *		device memory template
 *
 * @Input	devmem_tmplt_hndl: device memory template handle
 *
 * @Input	heap_info_arg: pointer to the heap info structure
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
int talmmu_devmem_heap_add(void *devmem_tmplt_hndl,
			   struct talmmu_heap_info *heap_info_arg)
{
	struct talmmu_dm_tmpl *devmem_template = devmem_tmplt_hndl;
	struct talmmu_devmem_heap *devmem_heap;
	unsigned int res;

	if (!devmem_tmplt_hndl)
		return IMG_ERROR_INVALID_PARAMETERS;

	if (!heap_info_arg)
		return IMG_ERROR_INVALID_PARAMETERS;

	res = talmmu_create_heap(devmem_tmplt_hndl,
				 heap_info_arg,
				 1,
				 &devmem_heap);
	if (res != IMG_SUCCESS)
		return res;

	devmem_template->devmem_heap[devmem_template->num_heaps] = devmem_heap;
	devmem_template->num_heaps++;

	return IMG_SUCCESS;
}

/*
 * @Function	talmmu_devmem_ctx_create
 *
 * @Description	This function is used to create a device memory context
 *
 * @Input	devmem_tmplt_hndl: pointer to the device memory template handle
 *
 * @Input	mmu_ctx_id: MMU context ID used with the TAL
 *
 * @Output	devmem_ctx_hndl: pointer to the device memory context handle
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
int talmmu_devmem_ctx_create(void *devmem_tmplt_hndl,
			     unsigned int mmu_ctx_id,
			     void **devmem_ctx_hndl)
{
	struct talmmu_dm_tmpl *dm_tmpl = devmem_tmplt_hndl;
	struct talmmu_devmem_ctx *dm_ctx;
	struct talmmu_devmem_heap *dm_heap;
	int i;
	unsigned int res = IMG_SUCCESS;

	if (!devmem_tmplt_hndl)
		return IMG_ERROR_INVALID_PARAMETERS;

	/* Allocate memory for device memory context */
	dm_ctx = kzalloc((sizeof(struct talmmu_devmem_ctx)), GFP_KERNEL);
	if (!dm_ctx)
		return IMG_ERROR_OUT_OF_MEMORY;

	/*
	 * Update the device memory context structure members
	 * Update the device memory template
	 */
	dm_ctx->devmem_template = dm_tmpl;
	/* Update MMU context ID */
	dm_ctx->mmu_ctx_id = mmu_ctx_id;

	/* Check for PTD Alignment */
	if (dm_tmpl->devmem_info.ptd_alignment == 0)
		/*
		 * Make sure alignment is a multiple of page size.
		 * Set up PTD alignment to Page Size
		 */
		dm_tmpl->devmem_info.ptd_alignment =
			dm_tmpl->devmem_info.page_size;

	/* Reference or create heaps for this context */
	for (i = 0; i < dm_tmpl->num_heaps; i++) {
		dm_heap = dm_tmpl->devmem_heap[i];
		if (!dm_heap)
			goto error_heap_create;

		switch (dm_heap->heap_info.heap_type) {
		case TALMMU_HEAP_PERCONTEXT:
			res = talmmu_create_heap(dm_tmpl,
						 &dm_heap->heap_info,
						 1,
						 &dm_ctx->devmem_heap[i]);
			if (res != IMG_SUCCESS)
				goto error_heap_create;
			break;

		default:
			break;
		}

		dm_ctx->num_heaps++;
	}

	mutex_lock_nested(global_lock, SUBCLASS_TALMMU);

	/* Add the device memory context to the list */
	lst_add(&dm_tmpl->devmem_ctx_list, dm_ctx);

	dm_tmpl->num_ctxs++;

	mutex_unlock(global_lock);

	*devmem_ctx_hndl = dm_ctx;

	return IMG_SUCCESS;

error_heap_create:
	/* Destroy the device memory heaps which were already created */
	for (i--; i >= 0; i--) {
		dm_heap = dm_ctx->devmem_heap[i];
		if (dm_heap->heap_info.heap_type == TALMMU_HEAP_PERCONTEXT)
			talmmu_devmem_heap_destroy(dm_heap);

		dm_ctx->num_heaps--;
	}
	kfree(dm_ctx);
	return res;
}

/*
 * @Function	talmmu_devmem_ctx_destroy
 *
 * @Description	This function is used to get the device memory context from
 *		the list and destroy
 *
 * @Input	devmem_ctx_hndl: device memory context handle
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
int talmmu_devmem_ctx_destroy(void *devmem_ctx_hndl)
{
	struct talmmu_devmem_ctx *devmem_ctx = devmem_ctx_hndl;
	struct talmmu_dm_tmpl *devmem_template;
	struct talmmu_devmem_heap *devmem_heap;
	unsigned int i;

	if (!devmem_ctx_hndl)
		return IMG_ERROR_INVALID_PARAMETERS;

	devmem_template = devmem_ctx->devmem_template;

	for (i = 0; i < devmem_ctx->num_heaps; i++) {
		devmem_heap = devmem_ctx->devmem_heap[i];
		if (!devmem_heap)
			return IMG_ERROR_INVALID_PARAMETERS;

		talmmu_devmem_heap_destroy(devmem_heap);
	}

	devmem_ctx->pagedir = NULL;

	mutex_lock_nested(global_lock, SUBCLASS_TALMMU);

	lst_remove(&devmem_template->devmem_ctx_list, devmem_ctx);

	devmem_ctx->devmem_template->num_ctxs--;

	mutex_unlock(global_lock);

	kfree(devmem_ctx);

	return IMG_SUCCESS;
}

/*
 * @Function	talmmu_get_heap_handle
 *
 * @Description	This function is used to get the device memory heap handle
 *
 * @Input	hid: heap id
 *
 * @Input	devmem_ctx_hndl: device memory context handle
 *
 * @Output	devmem_heap_hndl: pointer to the device memory heap handle
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
int talmmu_get_heap_handle(unsigned int hid,
			   void *devmem_ctx_hndl,
			   void **devmem_heap_hndl)
{
	struct talmmu_devmem_ctx *devmem_ctx = devmem_ctx_hndl;
	unsigned int i;

	if (!devmem_ctx_hndl)
		return IMG_ERROR_INVALID_PARAMETERS;

	for (i = 0; i < devmem_ctx->num_heaps; i++) {
		/*
		 * Checking for requested heap id match and return the device
		 * memory heap handle
		 */
		if (devmem_ctx->devmem_heap[i]->heap_info.heap_id == hid) {
			*devmem_heap_hndl = devmem_ctx->devmem_heap[i];
			return IMG_SUCCESS;
		}
	}

	return IMG_ERROR_GENERIC_FAILURE;
}

/*
 * @Function	talmmu_devmem_heap_options
 *
 * @Description	This function is used to set additional heap options
 *
 * @Input	devmem_heap_hndl: Handle for heap
 *
 * @Input	heap_opt_id: Heap options ID
 *
 * @Input	heap_options: Heap options
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
void talmmu_devmem_heap_options(void *devmem_heap_hndl,
				enum talmmu_heap_option_id heap_opt_id,
				union talmmu_heap_options heap_options)
{
	struct talmmu_devmem_heap *dm_heap = devmem_heap_hndl;

	switch (heap_opt_id) {
	case TALMMU_HEAP_OPT_ADD_GUARD_BAND:
		dm_heap->guardband = heap_options.guardband_opt.guardband;
		break;
	default:
		break;
	}
}

/*
 * @Function	talmmu_devmem_malloc_nonmap
 *
 * @Description
 *
 * @Input
 *
 * @Output
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
static int talmmu_devmem_alloc_nonmap(void *devmem_ctx_hndl,
				      void *devmem_heap_hndl,
				      unsigned int size,
				      unsigned int align,
				      unsigned int dev_virt_ofset,
				      unsigned char ext_dev_vaddr,
				      void **mem_hndl)
{
	struct talmmu_devmem_ctx *dm_ctx = devmem_ctx_hndl;
	struct talmmu_dm_tmpl *dm_tmpl;
	struct talmmu_devmem_heap *dm_heap = devmem_heap_hndl;
	struct talmmu_memory *mem;
	unsigned long long ui64_dev_offset = 0;
	int res = IMG_SUCCESS;

	if (!dm_ctx)
		return IMG_ERROR_INVALID_PARAMETERS;

	if (!devmem_heap_hndl)
		return IMG_ERROR_INVALID_PARAMETERS;

	dm_tmpl = dm_ctx->devmem_template;

	/* Allocate memory for memory structure */
	mem = kzalloc((sizeof(struct talmmu_memory)), GFP_KERNEL);
	if (!mem)
		return IMG_ERROR_OUT_OF_MEMORY;

	mem->devmem_heap = dm_heap;
	mem->devmem_ctx = dm_ctx;
	mem->ext_dev_virtaddr = ext_dev_vaddr;

	/* We always for to be at least page aligned */
	if (align >= dm_tmpl->devmem_info.page_size)
		/*
		 * alignment is larger than page size - make sure alignment is
		 * a multiple of page size
		 */
		mem->alignment = align;
	else
		/*
		 * alignment is smaller than page size - make sure page size is
		 * a multiple of alignment. Now round up alignment to one page
		 */
		mem->alignment = dm_tmpl->devmem_info.page_size;

	/* Round size up to next multiple of physical pages */
	if ((size % dm_tmpl->devmem_info.page_size) != 0)
		mem->size = ((size / dm_tmpl->devmem_info.page_size)
			+ 1) * dm_tmpl->devmem_info.page_size;
	else
		mem->size = size;

	/* If the device virtual address was externally defined */
	if (mem->ext_dev_virtaddr) {
		res = IMG_ERROR_INVALID_PARAMETERS;
		goto free_mem;
	}

	res = addr_cx_malloc_align_res(&dm_heap->ctx, "",
				       (mem->size + dm_heap->guardband),
				       mem->alignment,
				       &ui64_dev_offset);

	mem->dev_virtoffset = (unsigned int)ui64_dev_offset;
	if (res != IMG_SUCCESS)
		/*
		 * If heap space is unavaliable return NULL, the caller must
		 * handle this condition
		 */
		goto free_virt;

	mutex_lock_nested(global_lock, SUBCLASS_TALMMU);

	/*
	 * Add memory allocation to the list for this heap...
	 * If the heap is empty...
	 */
	if (lst_empty(&dm_heap->memory_list))
		/*
		 * Save flag to indicate whether the device virtual address
		 * is allocated internally or externally...
		 */
		dm_heap->ext_dev_virtaddr = mem->ext_dev_virtaddr;

	/*
	 * Once we have started allocating in one way ensure that we continue
	 * to do this...
	 */
	lst_add(&dm_heap->memory_list, mem);

	mutex_unlock(global_lock);

	*mem_hndl = mem;

	return IMG_SUCCESS;

free_virt:
	addr_cx_free(&dm_heap->ctx, "", mem->dev_virtoffset);
free_mem:
	kfree(mem);

	return res;
}

/*
 * @Function	talmmu_devmem_addr_alloc
 *
 * @Description
 *
 * @Input
 *
 * @Output
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
int talmmu_devmem_addr_alloc(void *devmem_ctx_hndl,
			     void *devmem_heap_hndl,
			     unsigned int size,
			     unsigned int align,
			     void **mem_hndl)
{
	unsigned int res;
	void *mem;

	res = talmmu_devmem_alloc_nonmap(devmem_ctx_hndl,
					 devmem_heap_hndl,
					 size,
					 align,
					 0,
					 0,
					 &mem);
	if (res != IMG_SUCCESS)
		return res;

	*mem_hndl = mem;

	return IMG_SUCCESS;
}

/*
 * @Function	talmmu_devmem_addr_free
 *
 * @Description	This function is used to free device memory allocated using
 *		talmmu_devmem_addr_alloc().
 *
 * @Input	mem_hndl : Handle for the memory object
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
int talmmu_devmem_addr_free(void *mem_hndl)
{
	unsigned int res;

	if (!mem_hndl)
		return IMG_ERROR_INVALID_PARAMETERS;

	/* free device memory allocated by calling talmmu_devmem_free() */
	res = talmmu_devmem_free(mem_hndl);

	return res;
}

/*
 * @Function	talmmu_get_dev_virt_addr
 *
 * @Description	This function is use to obtain the device (virtual) memory
 *		address which may be required for as a device virtual address
 *		in some of the TAL image functions
 *
 * @Input	mem_hndl : Handle for the memory object
 *
 * @Output	dev_virt: A piointer used to return the device virtual address
 *
 * @Return	IMG_SUCCESS or an error code
 *
 */
int talmmu_get_dev_virt_addr(void *mem_hndl,
			     unsigned int *dev_virt)
{
	struct talmmu_memory *mem = mem_hndl;
	struct talmmu_devmem_heap *devmem_heap;

	if (!mem_hndl)
		return IMG_ERROR_INVALID_PARAMETERS;

	devmem_heap = mem->devmem_heap;

	/*
	 * Device virtual address is addition of the specific device virtual
	 * offset and the base device virtual address from the heap information
	 */
	*dev_virt = (devmem_heap->heap_info.basedev_virtaddr +
		mem->dev_virtoffset);

	return IMG_SUCCESS;
}
