/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VDEC MMU Functions
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

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "img_errors.h"
#include "img_mem.h"
#include "lst.h"
#include "mmu_defs.h"
#include "vid_buf.h"

#ifndef _VXD_MMU_H_
#define _VXD_MMU_H_

/* Page size of the device MMU */
#define DEV_MMU_PAGE_SIZE (0x1000)
/* Page alignment of the device MMU */
#define DEV_MMU_PAGE_ALIGNMENT  (0x1000)

#define HOST_MMU_PAGE_SIZE	PAGE_SIZE

/*
 * @Function	mmu_stream_get_ptd_handle
 * @Description
 * This function is used to obtain the stream PTD (Page Table Directory)handle
 * @Input	mmu_str_handle : MMU stream handle.
 * @Output	str_ptd : Pointer to stream PTD handle.
 * @Return	IMG_SUCCESS or an error code.
 */
int mmu_stream_get_ptd_handle(void *mmu_str_handle, void **str_ptd);

/*
 * @Function	mmu_device_create
 * @Description
 * This function is used to create and initialise the MMU device context.
 * @Input	mmu_type : MMU type.
 * @Input	ptd_alignment : Alignment of Page Table directory.
 * @Output	mmudev_hndl : A pointer used to return the
 *		MMU device handle.
 * @Return	IMG_SUCCESS or an error code.
 */
int mmu_device_create(enum mmu_etype mmu_type,
		      unsigned int ptd_alignment,
		      void **mmudev_hndl);

/*
 * @Function	mmu_device_destroy
 * @Description
 * This function is used to create and initialise the MMU device context.
 * NOTE: Destroy device automatically destroys any streams and frees and
 * memory allocated using MMU_StreamMalloc().
 * @Input	mmudev_hndl : The MMU device handle.
 * @Return	IMG_SUCCESS or an error code.
 */
int mmu_device_destroy(void *mmudev_hndl);

/*
 * @Function	mmu_stream_create
 * @Description
 * This function is used to create and initialise the MMU stream context.
 * @Input	mmudev_hndl : The MMU device handle.
 * @Input	km_str_id : Stream Id used in communication with KM driver.
 * @Output	mmustr_hndl : A pointer used to return the MMU stream handle.
 * @Return	IMG_SUCCESS or an error code.
 */
int mmu_stream_create(void *mmudev_hndl, unsigned int km_str_id, void *vxd_dec_ctx,
		      void **mmustr_hndl);

/**
 * mmu_stream_destroy - This function is used to create and initialise the MMU stream context.
 * @mmustr_hndl : The MMU stream handle.
 * Return	IMG_SUCCESS or an error code.
 *
 * NOTE: Destroy automatically frees and memory allocated using
 *	mmu_stream_malloc().
 */
int mmu_stream_destroy(void *mmustr_hndl);

/*
 * @Function	mmu_stream_alloc
 * @Description
 * This function is used to allocate stream memory.
 * @Input	mmustr_hndl : The MMU stream handle.
 * @Input	heap_id : The MMU heap Id.
 * @Input	mem_heap_id : Memory heap id
 * @Input	mem_attrib : Memory attributes
 * @Input	size : The size, in bytes, to be allocated.
 * @Input	alignment : The required byte alignment
 *		(1, 2, 4, 8, 16 etc).
 * @Output	ddbuf_info : A pointer to a #vidio_ddbufinfo structure
 *		used to return the buffer info.
 * @Return	IMG_SUCCESS or an error code.
 */
int  mmu_stream_alloc(void *mmustr_hndl,
		      enum mmu_eheap_id heap_id,
		      unsigned int mem_heap_id,
		      enum sys_emem_attrib mem_attrib,
		      unsigned int size,
		      unsigned int alignment,
		      struct vidio_ddbufinfo *ddbuf_info);

/*
 * @Function	mmu_stream_map_ext
 * @Description
 * This function is used to malloc device memory (virtual memory), but mapping
 * this to memory that has already been allocated (externally).
 * NOTE: Memory can be freed using MMU_Free().  However, this does not
 *	free the memory provided by the caller via pvCpuLinearAddr.
 * @Input	mmustr_hndl    : The MMU stream handle.
 * @Input	heap_id : The heap Id.
 * @Input	buff_id : The buffer Id.
 * @Input	size : The size, in bytes, to be allocated.
 * @Input	alignment : The required byte alignment (1, 2, 4, 8, 16 etc).
 * @Input	mem_attrib : Memory attributes
 * @Input	cpu_linear_addr : CPU linear address of the memory
 *		to be allocated for the device.
 * @Output	ddbuf_info : A pointer to a #vidio_ddbufinfo structure
 *		used to return the buffer info.
 * @Return	IMG_SUCCESS or an error code.
 */
int mmu_stream_map_ext(void *mmustr_hndl,
		       enum mmu_eheap_id heap_id,
		       unsigned int buff_id,
		       unsigned int size,
		       unsigned int alignment,
		       enum sys_emem_attrib mem_attrib,
		       void *cpu_linear_addr,
		       struct vidio_ddbufinfo *ddbuf_info);

int mmu_stream_map_ext_sg(void *mmustr_hndl,
			  enum mmu_eheap_id heap_id,
			  void *sgt,
			  unsigned int size,
			  unsigned int alignment,
			  enum sys_emem_attrib mem_attrib,
			  void *cpu_linear_addr,
			  struct vidio_ddbufinfo *ddbuf_info,
			  unsigned int *buff_id);

/*
 * @Function	mmu_free_mem
 * @Description
 * This function is used to free device memory.
 * @Input	ps_dd_buf_info : A pointer to a #vidio_ddbufinfo structure.
 * @Return	IMG_SUCCESS or an error code.
 */
int mmu_free_mem(void *mmustr_hndl, struct vidio_ddbufinfo *ddbuf_info);

/*
 * @Function	mmu_free_mem
 * @Description
 * This function is used to free device memory.
 * @Input	ps_dd_buf_info : A pointer to a #vidio_ddbufinfo structure.
 * @Return	IMG_SUCCESS or an error code.
 */
int mmu_free_mem_sg(void *mmustr_hndl, struct vidio_ddbufinfo *ddbuf_info);

int mmu_get_heap(unsigned int image_stride, enum mmu_eheap_id *heap_id);

#endif /* _VXD_MMU_H_ */
