// SPDX-License-Identifier: GPL-2.0
/*
 * VXD Decoder resource allocation and tracking function implementations
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

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "decoder.h"
#include "dec_resources.h"
#include "hw_control.h"
#include "h264fw_data.h"
#include "h264_idx.h"
#include "h264_vlc.h"
#include "img_mem.h"
#include "pool_api.h"
#include "vdecdd_utils.h"
#include "vdec_mmu_wrapper.h"
#include "vid_buf.h"
#include "vxd_mmu_defs.h"

#define DECODER_END_BYTES_SIZE  40

#define BATCH_MSG_BUFFER_SIZE   (8 * 4096)
#define INTRA_BUF_SIZE          (1024 * 32)
#define AUX_LINE_BUFFER_SIZE    (512 * 1024)

static void decres_pack_vlc_tables(unsigned short *packed,
				   unsigned short *unpacked,
				   unsigned short size)
{
	unsigned short i, j;

	for (i = 0; i < size; i++) {
		j = i * 3;
		/*
		 * opcode 14:12
		 * width 11:9
		 * symbol 8:0
		 */
		packed[i] = 0 | ((unpacked[j]) << 12) |
			((unpacked[j + 1]) << 9) | (unpacked[j + 2]);
	}
}

struct dec_vlctable {
	void    *data;
	unsigned int num_entries;
	void    *index_table;
	unsigned int num_tables;
};

/*
 * Union with sizes of firmware parser header structure sizes. Dec_resources
 * uses the largest to allocate the header buffer.
 */
union decres_fw_hdrs {
	struct h264fw_header_data h264_header;
};

/*
 * This array contains the size of each resource allocation.
 * @brief  Resource Allocation Sizes
 * NOTE: This should be kept in step with #DECODER_eResType.
 */
static const unsigned int res_size[DECODER_RESTYPE_MAX] = {
	sizeof(struct vdecfw_transaction),
	sizeof(union decres_fw_hdrs),
	BATCH_MSG_BUFFER_SIZE,
#ifdef HAS_HEVC
	MEM_TO_REG_BUF_SIZE + SLICE_PARAMS_BUF_SIZE + ABOVE_PARAMS_BUF_SIZE,
#endif
};

static const unsigned char start_code[] = {
	0x00, 0x00, 0x01, 0x00,
};

static void decres_get_vlc_data(struct dec_vlctable *vlc_table,
				enum vdec_vid_std vid_std)
{
	switch (vid_std) {
	case VDEC_STD_H264:
		vlc_table->data = h264_vlc_table_data;
		vlc_table->num_entries = h264_vlc_table_size;
		vlc_table->index_table = h264_vlc_index_data;
		vlc_table->num_tables = h264_vlc_index_size;
		break;

	default:
		memset(vlc_table, 0x0, sizeof(*vlc_table));
		break;
	}
}

static void decres_fnbuf_info_destructor(void *param, void *cb_handle)
{
	struct vidio_ddbufinfo *dd_bufinfo = (struct vidio_ddbufinfo *)param;
	int ret;
	void *mmu_handle = cb_handle;

	VDEC_ASSERT(dd_bufinfo);

	ret = mmu_free_mem(mmu_handle, dd_bufinfo);
	VDEC_ASSERT(ret == IMG_SUCCESS);

	kfree(dd_bufinfo);
	dd_bufinfo = NULL;
}

int dec_res_picture_detach(void **res_ctx, struct dec_decpict *dec_pict)
{
	struct dec_res_ctx *local_res_ctx;

	VDEC_ASSERT(res_ctx);
	VDEC_ASSERT(res_ctx && *res_ctx);
	VDEC_ASSERT(dec_pict);
	VDEC_ASSERT(dec_pict && dec_pict->transaction_info);

	if (!res_ctx || !(*res_ctx) || !dec_pict ||
	    !dec_pict->transaction_info) {
		pr_err("Invalid parameters\n");
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	local_res_ctx = (struct dec_res_ctx *)*res_ctx;

	/* return transaction buffer */
	lst_add(&local_res_ctx->pool_data_list[DECODER_RESTYPE_TRANSACTION],
		dec_pict->transaction_info);
	pool_resfree(dec_pict->transaction_info->res);

	/* return picture header information buffer */
	lst_add(&local_res_ctx->pool_data_list[DECODER_RESTYPE_HDR],
		dec_pict->hdr_info);
	pool_resfree(dec_pict->hdr_info->res);

	/* return batch message buffer */
	lst_add(&local_res_ctx->pool_data_list[DECODER_RESTYPE_BATCH_MSG],
		dec_pict->batch_msginfo);
	pool_resfree(dec_pict->batch_msginfo->res);

#ifdef HAS_HEVC
	if (dec_pict->pvdec_info) {
		lst_add(&local_res_ctx->pool_data_list[DECODER_RESTYPE_PVDEC_BUF],
			dec_pict->pvdec_info);
		pool_resfree(dec_pict->pvdec_info->res);
	}
#endif

	return IMG_SUCCESS;
}

static int decres_get_resource(struct dec_res_ctx *res_ctx,
			       enum dec_res_type res_type,
			       struct res_resinfo **res_info,
			       unsigned char fill_zeros)
{
	struct res_resinfo *local_res_info = NULL;
	unsigned int ret = IMG_SUCCESS;

	VDEC_ASSERT(res_ctx);
	VDEC_ASSERT(res_info);

	local_res_info = lst_removehead(&res_ctx->pool_data_list[res_type]);
	VDEC_ASSERT(local_res_info);
	if (local_res_info) {
		VDEC_ASSERT(local_res_info->ddbuf_info);
		if (local_res_info->ddbuf_info) {
			ret = pool_resalloc(res_ctx->res_pool[res_type], local_res_info->res);
			VDEC_ASSERT(ret == IMG_SUCCESS);
			if (ret != IMG_SUCCESS) {
				ret = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
				return ret;
			}

			if (fill_zeros)
				memset(local_res_info->ddbuf_info->cpu_virt, 0,
				       local_res_info->ddbuf_info->buf_size);

			*res_info = local_res_info;
		} else {
			ret = IMG_ERROR_FATAL;
			return ret;
		}
	} else {
		ret = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
		return ret;
	}

	return ret;
}

int dec_res_picture_attach(void **res_ctx, enum vdec_vid_std vid_std,
			   struct dec_decpict *dec_pict)
{
	struct dec_res_ctx *local_res_ctx;
	int ret;

	VDEC_ASSERT(res_ctx);
	VDEC_ASSERT(res_ctx && *res_ctx);
	VDEC_ASSERT(dec_pict);
	if (!res_ctx || !(*res_ctx) || !dec_pict) {
		pr_err("Invalid parameters");
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	local_res_ctx = (struct dec_res_ctx *)*res_ctx;

	/* Obtain transaction buffer. */
	ret = decres_get_resource(local_res_ctx, DECODER_RESTYPE_TRANSACTION,
				  &dec_pict->transaction_info, TRUE);

	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	/* Obtain picture header information buffer */
	ret = decres_get_resource(local_res_ctx, DECODER_RESTYPE_HDR,
				  &dec_pict->hdr_info, TRUE);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

#ifdef HAS_HEVC
	/* Obtain HEVC buffer */
	if (vid_std == VDEC_STD_HEVC) {
		ret = decres_get_resource(local_res_ctx, DECODER_RESTYPE_PVDEC_BUF,
					  &dec_pict->pvdec_info, TRUE);

		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;
	}
#endif
	/* Obtain picture batch message buffer */
	ret = decres_get_resource(local_res_ctx, DECODER_RESTYPE_BATCH_MSG,
				  &dec_pict->batch_msginfo, TRUE);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dec_pict->intra_bufinfo = &local_res_ctx->intra_bufinfo;
	dec_pict->auxline_bufinfo = &local_res_ctx->auxline_bufinfo;
	dec_pict->vlc_tables_bufinfo =
		&local_res_ctx->vlc_tables_bufinfo[vid_std];
	dec_pict->vlc_idx_tables_bufinfo =
		&local_res_ctx->vlc_idxtables_bufinfo[vid_std];
	dec_pict->start_code_bufinfo = &local_res_ctx->start_code_bufinfo;

	return IMG_SUCCESS;
}

int dec_res_create(void *mmu_handle, struct vxd_coreprops *core_props,
		   unsigned int num_dec_slots,
		   unsigned int mem_heap_id, void **resources)
{
	struct dec_res_ctx *local_res_ctx;
	int ret;
	unsigned int i = 0;
	struct dec_vlctable vlc_table;
	enum sys_emem_attrib mem_attrib;

	VDEC_ASSERT(core_props);
	VDEC_ASSERT(resources);
	if (!core_props || !resources) {
		pr_err("Invalid parameters");
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	mem_attrib = (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED | SYS_MEMATTRIB_WRITECOMBINE);
	mem_attrib |= (enum sys_emem_attrib)SYS_MEMATTRIB_INTERNAL;

	local_res_ctx = kzalloc(sizeof(*local_res_ctx), GFP_KERNEL);
	VDEC_ASSERT(local_res_ctx);
	if (!local_res_ctx)
		return IMG_ERROR_OUT_OF_MEMORY;

	/* Allocate Intra buffer. */
#ifdef DEBUG_DECODER_DRIVER
	pr_info("%s:%d call MMU_StreamMalloc", __func__, __LINE__);
#endif

	ret = mmu_stream_alloc(mmu_handle, MMU_HEAP_STREAM_BUFFERS, mem_heap_id,
			       mem_attrib,
			       core_props->num_pixel_pipes *
			       INTRA_BUF_SIZE * 3,
			       DEV_MMU_PAGE_ALIGNMENT,
			       &local_res_ctx->intra_bufinfo);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	/* Allocate aux line buffer. */
#ifdef DEBUG_DECODER_DRIVER
	pr_info("%s:%d call MMU_StreamMalloc", __func__, __LINE__);
#endif
	ret = mmu_stream_alloc(mmu_handle, MMU_HEAP_STREAM_BUFFERS, mem_heap_id,
			       mem_attrib,
			       AUX_LINE_BUFFER_SIZE * 3 *
			       core_props->num_pixel_pipes,
			       DEV_MMU_PAGE_ALIGNMENT,
			       &local_res_ctx->auxline_bufinfo);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	/* Allocate standard-specific buffers. */
	for (i = VDEC_STD_UNDEFINED + 1; i < VDEC_STD_MAX; i++) {
		decres_get_vlc_data(&vlc_table, (enum vdec_vid_std)i);

		if (vlc_table.num_tables > 0) {
			/*
			 * Size of VLC IDX table in bytes. Has to be aligned
			 * to 4, so transfer to MTX succeeds.
			 * (VLC IDX is copied to local RAM of MTX)
			 */
			unsigned int vlc_idxtable_sz =
				ALIGN((sizeof(unsigned short) * vlc_table.num_tables * 3), 4);

#ifdef DEBUG_DECODER_DRIVER
			pr_info(" %s:%d calling MMU_StreamMalloc", __func__, __LINE__);
#endif

			ret = mmu_stream_alloc(mmu_handle,
					       MMU_HEAP_STREAM_BUFFERS,
					       mem_heap_id, (enum sys_emem_attrib)(mem_attrib |
					       SYS_MEMATTRIB_CORE_READ_ONLY |
					       SYS_MEMATTRIB_CPU_WRITE),
					       sizeof(unsigned short) * vlc_table.num_entries,
					       DEV_MMU_PAGE_ALIGNMENT,
					       &local_res_ctx->vlc_tables_bufinfo[i]);
			VDEC_ASSERT(ret == IMG_SUCCESS);
			if (ret != IMG_SUCCESS)
				goto error;

			if (vlc_table.data)
				decres_pack_vlc_tables
					(local_res_ctx->vlc_tables_bufinfo[i].cpu_virt,
					 vlc_table.data,
					 vlc_table.num_entries);

			/* VLC index table */
#ifdef DEBUG_DECODER_DRIVER
			pr_info("%s:%d calling MMU_StreamMalloc",
				__func__, __LINE__);
#endif
			ret = mmu_stream_alloc(mmu_handle,
					       MMU_HEAP_STREAM_BUFFERS,
					       mem_heap_id, (enum sys_emem_attrib)(mem_attrib |
					       SYS_MEMATTRIB_CORE_READ_ONLY |
					       SYS_MEMATTRIB_CPU_WRITE),
					       vlc_idxtable_sz,
					       DEV_MMU_PAGE_ALIGNMENT,
					       &local_res_ctx->vlc_idxtables_bufinfo[i]);
			VDEC_ASSERT(ret == IMG_SUCCESS);
			if (ret != IMG_SUCCESS)
				goto error;

			if (vlc_table.index_table)
				memcpy(local_res_ctx->vlc_idxtables_bufinfo[i].cpu_virt,
				       vlc_table.index_table,
				       local_res_ctx->vlc_idxtables_bufinfo[i].buf_size);
		}
	}

	/* Start code */
#ifdef DEBUG_DECODER_DRIVER
	pr_info("%s:%d calling MMU_StreamMalloc", __func__, __LINE__);
#endif
	ret = mmu_stream_alloc(mmu_handle, MMU_HEAP_STREAM_BUFFERS, mem_heap_id,
			       (enum sys_emem_attrib)(mem_attrib |
			       SYS_MEMATTRIB_CORE_READ_ONLY |
			       SYS_MEMATTRIB_CPU_WRITE),
			       sizeof(start_code),
			       DEV_MMU_PAGE_ALIGNMENT,
			       &local_res_ctx->start_code_bufinfo);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	memcpy(local_res_ctx->start_code_bufinfo.cpu_virt, start_code, sizeof(start_code));

	for (i = 0; i < DECODER_RESTYPE_MAX; i++) {
		unsigned int j;

		ret = pool_api_create(&local_res_ctx->res_pool[i]);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			goto error;

		lst_init(&local_res_ctx->pool_data_list[i]);

		for (j = 0; j < num_dec_slots; j++) {
			struct res_resinfo *local_res_info;

			local_res_info = kzalloc(sizeof(*local_res_info), GFP_KERNEL);

			VDEC_ASSERT(local_res_info);
			if (!local_res_info) {
				pr_err("Failed to allocate memory\n");
				ret = IMG_ERROR_OUT_OF_MEMORY;
				goto error_local_res_info_alloc;
			}

			local_res_info->ddbuf_info = kzalloc(sizeof(*local_res_info->ddbuf_info),
							     GFP_KERNEL);
			VDEC_ASSERT(local_res_info->ddbuf_info);
			if (!local_res_info->ddbuf_info) {
				pr_err("Failed to allocate memory for resource buffer information structure");
				ret = IMG_ERROR_OUT_OF_MEMORY;
				goto error_local_dd_buf_alloc;
			}

#ifdef DEBUG_DECODER_DRIVER
			pr_info("%s:%d calling MMU_StreamMalloc", __func__, __LINE__);
#endif
			ret = mmu_stream_alloc(mmu_handle, MMU_HEAP_STREAM_BUFFERS,
					       mem_heap_id, (enum sys_emem_attrib)(mem_attrib |
					       SYS_MEMATTRIB_CPU_READ |
					       SYS_MEMATTRIB_CPU_WRITE),
					       res_size[i],
					       DEV_MMU_PAGE_ALIGNMENT,
					       local_res_info->ddbuf_info);
			VDEC_ASSERT(ret == IMG_SUCCESS);
			if (ret != IMG_SUCCESS)
				goto error_local_res_alloc;

			/* Register with the buffer pool */
			ret = pool_resreg(local_res_ctx->res_pool[i],
					  decres_fnbuf_info_destructor,
					  local_res_info->ddbuf_info,
					  sizeof(*local_res_info->ddbuf_info),
					  FALSE, NULL,
					  &local_res_info->res, mmu_handle);
			VDEC_ASSERT(ret == IMG_SUCCESS);
			if (ret != IMG_SUCCESS)
				goto error_local_res_register;

			lst_add(&local_res_ctx->pool_data_list[i],
				local_res_info);
			continue;

/* Roll back in case of local errors. */
error_local_res_register: mmu_free_mem(mmu_handle, local_res_info->ddbuf_info);

error_local_res_alloc:  kfree(local_res_info->ddbuf_info);

error_local_dd_buf_alloc: kfree(local_res_info);

error_local_res_info_alloc: goto error;
		}
	}

	*resources = (void *)local_res_ctx;

	return IMG_SUCCESS;

/* Roll back in case of errors. */
error:  dec_res_destroy(mmu_handle, (void *)local_res_ctx);

	return ret;
}

/*
 *@Function RESOURCES_Destroy
 *
 */
int dec_res_destroy(void *mmudev_handle, void *res_ctx)
{
	int ret = IMG_SUCCESS;
	int ret1 = IMG_SUCCESS;
	unsigned int i = 0;
	struct res_resinfo *local_res_info;
	struct res_resinfo *next_res_info;

	struct dec_res_ctx *local_res_ctx = (struct dec_res_ctx *)res_ctx;

	if (!local_res_ctx) {
		pr_err("Invalid parameters");
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (local_res_ctx->intra_bufinfo.hndl_memory) {
		ret1 = mmu_free_mem(mmudev_handle, &local_res_ctx->intra_bufinfo);
		VDEC_ASSERT(ret1 == IMG_SUCCESS);
		if (ret1 != IMG_SUCCESS)
			ret = ret1;
	}

	if (local_res_ctx->auxline_bufinfo.hndl_memory) {
		ret1 = mmu_free_mem(mmudev_handle, &local_res_ctx->auxline_bufinfo);
		VDEC_ASSERT(ret1 == IMG_SUCCESS);
		if (ret1 != IMG_SUCCESS)
			ret = ret1;
	}

	for (i = 0; i < VDEC_STD_MAX; i++) {
		if (local_res_ctx->vlc_tables_bufinfo[i].hndl_memory) {
			ret1 = mmu_free_mem(mmudev_handle, &local_res_ctx->vlc_tables_bufinfo[i]);
			VDEC_ASSERT(ret1 == IMG_SUCCESS);
			if (ret1 != IMG_SUCCESS)
				ret = ret1;
		}

		if (local_res_ctx->vlc_idxtables_bufinfo[i].hndl_memory) {
			ret1 = mmu_free_mem(mmudev_handle,
					    &local_res_ctx->vlc_idxtables_bufinfo[i]);
			VDEC_ASSERT(ret1 == IMG_SUCCESS);
			if (ret1 != IMG_SUCCESS)
				ret = ret1;
		}
	}

	if (local_res_ctx->start_code_bufinfo.hndl_memory) {
		ret1 = mmu_free_mem(mmudev_handle, &local_res_ctx->start_code_bufinfo);
		VDEC_ASSERT(ret1 == IMG_SUCCESS);
		if (ret1 != IMG_SUCCESS)
			ret = ret1;
	}

	for (i = 0; i < DECODER_RESTYPE_MAX; i++) {
		if (local_res_ctx->res_pool[i]) {
			local_res_info =
				lst_first(&local_res_ctx->pool_data_list[i]);
			while (local_res_info) {
				next_res_info = lst_next(local_res_info);
				lst_remove(&local_res_ctx->pool_data_list[i], local_res_info);
				ret1 = pool_resdestroy(local_res_info->res, TRUE);
				VDEC_ASSERT(ret1 == IMG_SUCCESS);
				if (ret1 != IMG_SUCCESS)
					ret = ret1;
				kfree(local_res_info);
				local_res_info = next_res_info;
			}
			pool_destroy(local_res_ctx->res_pool[i]);
		}
	}

	kfree(local_res_ctx);
	return ret;
}
