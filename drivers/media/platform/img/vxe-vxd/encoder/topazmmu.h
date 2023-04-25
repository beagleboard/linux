/* SPDX-License-Identifier: GPL-2.0 */
/*
 * topaz mmu header
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Sunita Nadampalli <sunitan@ti.com>
 *
 * Re-written for upstreming
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#ifndef TOPAZZ_MMU_H_
#define TOPAZZ_MMU_H_

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "talmmu_api.h"
#include "vxe_enc.h"
#include "img_mem.h"
#include "target_config.h"

/* Page size of the device MMU */
#define DEV_MMU_PAGE_SIZE (0x1000)
/* Page alignment of the device MMU */
#define DEV_MMU_PAGE_ALIGNMENT  (0x1000)

#define HOST_MMU_PAGE_SIZE PAGE_SIZE

/*
 * This structure contains the stream context.
 * @brief MMU Stream Context
 * @devmem_ctx_hndl: Handle for MMU context.
 * @dev_ctx: Pointer to device context.
 * @ctx_id: MMU context Id.
 * km_str_id: Stream ID used in communication with new KM interface
 */
struct mmu_str_context {
	void **link; // to be able to maintain in single linked list.
	void *mmu_context_handle;
	unsigned int int_reg_num;
	unsigned int km_str_id;
	/* vxe encoder context. Need in stream context to access mem_ctx. */
	struct vxe_enc_ctx *vxe_enc_context;
	struct lst_t ddbuf_list;
};

struct topaz_mmu_context {
	void *mmu_context_handle;
	unsigned int ptd_phys_addr;
	struct lst_t str_list;
};

/*
 * This type defines the MMU heaps.
 * @0:	General heap ID.
 */
enum topaz_mmu_eheap_id {
	MMU_GENERAL_HEAP_ID = 0x00,
	/* Do not remove - keeps count of size */
	HEAP_ID_NO_OF_HEAPS
};

/* Function definitions */

/*
 * Called once during initialization to initialize the MMU hardware, create
 * the template and define the MMU heap.
 * This is where talmmu initialization and template will be created.
 *
 * NOTE : We are not taking care of alignment here, need to be updated in
 * mmu_device_memory_info.
 */
int topaz_mmu_device_create(struct topaz_mmu_context *mmu_context, unsigned int mmu_flags);

/*
 * @Function	mmu_device_destroy
 * @Description
 * This function is used to destroy the MMU device context.
 * NOTE: Destroy device automatically destroys any streams and frees and
 * memory allocated using MMU_StreamMalloc().
 * @Return	IMG_SUCCESS or an error code.
 */
int topaz_mmu_device_destroy(struct topaz_mmu_context *mmu_context);

/*
 * @Function	mmu_stream_create
 * @Description
 * This function is used to create and initialize the MMU stream context.
 * @Input	km_str_id : Stream Id used in communication with KM driver.
 * @Return	IMG_SUCCESS or an error code.
 *
 * Context ID is 1, since we are creating single stream.
 */
int topaz_mmu_stream_create(struct topaz_mmu_context *mmu_context, unsigned int km_str_id,
			    void *vxe_enc_ctx_arg, void **mmu_str_ctx);

/*
 * @Function	mmu_stream_destroy
 * @Description
 * This function is used to destroy the MMU stream context.
 * NOTE: Destroy automatically frees and memory allocated using
 *	mmu_stream_malloc().
 * @Input	str_ctx : The MMU stream handle.
 * @Return	IMG_SUCCESS or an error code.
 */
int topaz_mmu_stream_destroy(struct topaz_mmu_context *mmu_context,
			     struct mmu_str_context *str_ctx);

int topaz_mmu_alloc(void *mmu_context_handle, struct vxe_enc_ctx *vxe_enc_ctx_arg,
		    enum topaz_mmu_eheap_id heap_id, unsigned int mem_heap_id,
		    enum sys_emem_attrib mem_attrib, unsigned int size, unsigned int alignment,
		    struct vidio_ddbufinfo *ddbuf_info);
/*
 * @Function	mmu_stream_malloc
 */
int topaz_mmu_stream_alloc(void *mmu_str_hndl, enum topaz_mmu_eheap_id heap_id,
			   unsigned int mem_heap_id,
			   enum sys_emem_attrib mem_attrib,
			   unsigned int size,
			   unsigned int alignment,
			   struct vidio_ddbufinfo *ddbuf_info);

/*
 * @Function	mmu_stream_map_ext_sg
 */
int topaz_mmu_stream_map_ext_sg(void *mmu_str_hndl, enum topaz_mmu_eheap_id heap_id,
				void *sgt,
				unsigned int size,
				unsigned int alignment,
				enum sys_emem_attrib mem_attrib,
				void *cpu_linear_addr,
				struct vidio_ddbufinfo *ddbuf_info,
				unsigned int *buff_id);

/*
 * @Function	mmu_stream_map_ext
 */
int topaz_mmu_stream_map_ext(void *mmu_str_hndl, enum topaz_mmu_eheap_id heap_id,
			     unsigned int buff_id, unsigned int size,
			     unsigned int alignment,
			     enum sys_emem_attrib mem_attrib,
			     void *cpu_linear_addr,
			     struct vidio_ddbufinfo *ddbuf_info);

/* topaz core mmu hardware setup */
int topaz_core_mmu_hw_setup(struct topaz_mmu_context *mmu_context, void *core_reg);

/* topaz core mmu flush cache */
int topaz_core_mmu_flush_cache(void);

/*
 * @Function    mmu_free
 *
 * Free memory allocated with mmu_alloc
 */
int topaz_mmu_free(struct vxe_enc_ctx *vxe_enc_ctx_arg,
		   struct vidio_ddbufinfo *ddbuf_info);

/*
 * @Function	mmu_free_mem.
 *
 * NOTE : This should be used only to free the stream memory.
 */
int topaz_mmu_stream_free(void *mmu_str_hndl, struct vidio_ddbufinfo *ddbuf_info);

/*
 * @Function	mmu_free_mem_sg.
 *
 * NOTE : This should be used only to free the stream memory.
 */
int topaz_mmu_stream_free_sg(void *mmu_str_hndl, struct vidio_ddbufinfo *ddbuf_info);

/*
 * @Function    update_device_mem
 *
 * Update the memory to the device
 */
int topaz_update_device_mem(struct vxe_enc_ctx *vxe_enc_ctx_arg,
			    struct vidio_ddbufinfo *ddbuf_info);

/*
 * @Function    update_host_mem
 *
 * Update the memory to the host
 */
int topaz_update_host_mem(struct vxe_enc_ctx *vxe_enc_ctx_arg,
			  struct vidio_ddbufinfo *ddbuf_info);

/* Global */
extern struct mem_space topaz_mem_space[];
extern void *g_lock;

#endif /* TOPAZZ_MMU_H_ */
