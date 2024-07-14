// SPDX-License-Identifier: GPL-2.0
/*
 * Encoder Core API function implementations
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

#include <linux/string.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/mutex.h>
#include <linux/types.h>

#include "fw_headers/coreflags.h"
#include "fw_headers/topazscfwif.h"
#include "header_gen.h"
#include "img_errors.h"
#include "img_mem_man.h"
#include "lst.h"
#include "reg_headers/topaz_coreext_regs.h"
#include "reg_headers/topazhp_core_regs.h"
#include "reg_headers/topaz_vlc_regs.h"
#include "reg_headers/topaz_db_regs.h"
#include "topaz_color_formats.h"
#include "topaz_device.h"
#include "topaz_api.h"
#include "topaz_api_utils.h"
#include "topazmmu.h"
#include "vxe_public_regdefs.h"
#include "img_errors.h"

#define TOPAZ_TIMEOUT_RETRIES (5000000)
#define TOPAZ_TIMEOUT_WAIT_FOR_SPACE (500)

#define COMM_WB_DATA_BUF_SIZE (64)

/*
 * All contexts should be able to send as many commands as possible before waiting for a response.
 * There must be enough command memory buffers for all applicable commands, that is:
 * -To fill all source slots
 * -To supply custom quant data
 */
#define TOPAZ_CMD_DATA_BUF_NUM  ((MAX_SOURCE_SLOTS_SL + 1) * TOPAZHP_MAX_POSSIBLE_STREAMS)
#define TOPAZ_CMD_DATA_BUF_SIZE (64)
#define COMM_CMD_DATA_BUF_SLOT_NONE 0xFF

struct topaz_core_context *global_topaz_core_context;

static unsigned char global_cmd_data_busy[TOPAZ_CMD_DATA_BUF_NUM];
struct vidio_ddbufinfo global_cmd_data_dev_addr; /* Data section */
struct vidio_ddbufinfo global_cmd_data_info[TOPAZ_CMD_DATA_BUF_NUM]; /* Data section */
static unsigned char global_pipe_usage[TOPAZHP_MAX_NUM_PIPES] = { 0 };

struct vidio_ddbufinfo *global_wb_data_info;
static unsigned char is_topaz_core_initialized;

/*
 * Get a buffer reference
 */
static int topaz_get_buffer(struct topaz_stream_context *str_ctx,
			    struct img_buffer *buffer, void **lin_address,
	unsigned char update_host_memory)
{
	if (buffer->lock == NOTDEVICEMEMORY) {
		*lin_address = buffer->mem_info.cpu_virt;
		return IMG_SUCCESS;
	}

	if (buffer->lock == SW_LOCK)
		return IMG_ERROR_SURFACE_LOCKED;

	if (update_host_memory)
		topaz_update_host_mem(str_ctx->vxe_ctx, &buffer->mem_info);

	*lin_address    = buffer->mem_info.cpu_virt;
	buffer->lock    = SW_LOCK;

	return IMG_SUCCESS;
}

static int topaz_release_buffer(struct topaz_stream_context *str_ctx,
				struct img_buffer *buffer, unsigned char update_device_memory)
{
	if (buffer->lock == NOTDEVICEMEMORY)
		return IMG_SUCCESS;

	if (buffer->lock == HW_LOCK)
		return IMG_ERROR_SURFACE_LOCKED;

	buffer->lock = BUFFER_FREE;

	if (update_device_memory)
		topaz_update_device_mem(str_ctx->vxe_ctx, &buffer->mem_info);

	return IMG_SUCCESS;
}

static int topaz_get_cmd_data_buffer(struct vidio_ddbufinfo **mem_info)
{
	int index = 0;
	int res = IMG_SUCCESS;

	mutex_lock_nested(global_topaz_core_context->mutex, SUBCLASS_TOPAZ_API);

	do {
		if (!global_cmd_data_busy[index])
			break;
		index++;
	} while (index < ARRAY_SIZE(global_cmd_data_info));

	if (index == ARRAY_SIZE(global_cmd_data_info)) {
		res = IMG_ERROR_UNEXPECTED_STATE;
	} else {
		global_cmd_data_busy[index] = TRUE;
		*mem_info = &global_cmd_data_info[index];
	}

	mutex_unlock((struct mutex *)global_topaz_core_context->mutex);

	return res;
}

static int topaz_release_cmd_data_buffer(struct vidio_ddbufinfo *mem_info)
{
	int index = 0;
	int res = IMG_ERROR_UNEXPECTED_STATE;

	mutex_lock_nested(global_topaz_core_context->mutex, SUBCLASS_TOPAZ_API);

	do {
		if (mem_info == &global_cmd_data_info[index]) {
			global_cmd_data_busy[index] = FALSE;
			res = IMG_SUCCESS;
			break;
		}
		index++;
	} while (index < ARRAY_SIZE(global_cmd_data_info));

	mutex_unlock((struct mutex *)global_topaz_core_context->mutex);

	return res;
}

/*
 * Get a buffer reference
 */
static int get_coded_buffer(struct topaz_stream_context *str_ctx, void **lin_address,
			    unsigned char update_host_memory, unsigned char coded_package_idx)
{
	struct img_enc_context *enc;
	struct img_video_context *video;
	unsigned char coded_buffer_idx;
	unsigned char found = FALSE;
	unsigned int *address;
	struct coded_data_hdr *coded_datahdr = NULL;
	unsigned int offset_buffer_header = 0, offset_coded_buffer = 0;
	/* Tells if all the slices have been retrieved */
	unsigned char all_slice_retrieved = FALSE;
	/* Tells if we have reach the last coded buffer used or not */
	unsigned char slice_break = FALSE;
	/* Tells if we are at the beginning of a slice or not */
	unsigned char new_coded_header = TRUE;
	/* Tells the number of bytes remaining to be retrieved */
	unsigned int total_byte_written = 0;
	unsigned int coded_slices_so_far = 0;
	unsigned int coded_slices_in_buffer = 0;

	enc = str_ctx->enc_ctx;
	video = enc->video;

	if (video->coded_package[coded_package_idx]->header_buffer->lock == SW_LOCK)
		return IMG_ERROR_UNDEFINED;

	/* Retrieve the FW Package memory. Get linear address */
	video->coded_package[coded_package_idx]->mtx_info.coded_package_fw =
		(struct coded_package_dma_info *)(&video->coded_package[coded_package_idx]->mtx_info
		.code_package_fw_buffer->mem_info);

	if (update_host_memory) {
		/* Go through all the coded buffers */
		for (coded_buffer_idx = 0; coded_buffer_idx < MAX_CODED_BUFFERS_PER_PACKAGE;
			coded_buffer_idx++) {
			/* Reset the Offset */
			offset_coded_buffer = 0;
			do {
				if (new_coded_header) { // beginning of a slice
					slice_break = FALSE;
					/* Get the coded header information */
					*lin_address = video->coded_package
					[coded_package_idx]->header_buffer->mem_info.cpu_virt;
					address = *lin_address;
					/* Getting the nth buffer header */
					coded_datahdr = (struct coded_data_hdr *)(address +
							(offset_buffer_header / 4));
					total_byte_written = coded_datahdr->bytes_written;
					coded_slices_so_far =
						F_DECODE(coded_datahdr->extra_feedback,
							 CODED_SLICES_SO_FAR);
					coded_slices_in_buffer =
						F_DECODE(coded_datahdr->extra_feedback,
							 CODED_SLICES_IN_BUFFER);

					/* Increment the offset in the coded header information
					 * buffer in order to point on the next header
					 */
					offset_buffer_header += CODED_BUFFER_INFO_SECTION_SIZE;
				}

				if (!new_coded_header) {
					/* Retrieve the last coded data */
					offset_coded_buffer = ALIGN_16(offset_coded_buffer +
								       total_byte_written);
					slice_break = TRUE;
					/* On next loop we will be at the start of a new slice */
					new_coded_header = TRUE;
				} else {
					/*
					 * New slice : Read all the bytes written for this slice
					 * Go after what we read, next 16bit align address
					 */
					offset_coded_buffer =
						ALIGN_16(offset_coded_buffer +
							 coded_datahdr->bytes_written);
					if (F_DECODE(coded_datahdr->extra_feedback,
						     CODED_SLICES_SO_FAR) ==
							F_DECODE(coded_datahdr->extra_feedback,
								 CODED_SLICES_IN_BUFFER)) {
						/* We now have all the slices for this coded buffer,
						 * we should not try to read further.
						 */
						all_slice_retrieved = TRUE;
						break;
					}
				}
			} while (coded_slices_so_far != coded_slices_in_buffer);

			if (all_slice_retrieved || slice_break) {
				/* If we are NOT in the middle of a slice */
				found = TRUE;
				/* We lock this last buffer */
				video->coded_package[coded_package_idx]->coded_buffer
								[coded_buffer_idx]->lock = SW_LOCK;
				/* This function will do nothing if -debugCRCs (1 or 2) has not
				 * been specified on the command line
				 */
				break;
			}
		}

		if (!found)
			topaz_update_host_mem(str_ctx->vxe_ctx, &video->coded_package
						[coded_package_idx]->header_buffer->mem_info);
	}

	/* address of first  header if all buffer finish in middle of
	 * slice or !bUpdateHostMemory, last red header otherwise
	 */
	*lin_address = video->coded_package[coded_package_idx]->header_buffer->mem_info.cpu_virt;
	/* Lock-it */
	video->coded_package[coded_package_idx]->header_buffer->lock = SW_LOCK;

	return IMG_SUCCESS;
}

static void combine_feedback(struct topaz_stream_context *str_ctx,
			     unsigned char active_coded_package_idx, unsigned int *feedback,
			     unsigned int *extra_feedback, unsigned int *bytes_coded)
{
	struct img_enc_context *enc = str_ctx->enc_ctx;
	struct coded_data_hdr *coded_datahdr;
	unsigned int offset = 0;
	unsigned int min_bu = 0xFFFFFFFF;
	unsigned int coded_bytes = 0;
	unsigned int bu;
	unsigned int coded_slices_so_far;
	unsigned int coded_slices_in_buffer;

	do {
		/* we should be able to rely on the linear pointer here
		 * as the coded data header should have been updated.
		 */
		coded_datahdr = (struct coded_data_hdr *)((unsigned long)(enc->video->coded_package
				[active_coded_package_idx]->header_buffer->mem_info.cpu_virt) +
				offset);

		IMG_DBG_ASSERT(coded_datahdr);
		if (!coded_datahdr)
			return;

		bu = F_DECODE(coded_datahdr->feedback, CODED_FIRST_BU);
		coded_slices_so_far = F_DECODE(coded_datahdr->extra_feedback, CODED_SLICES_SO_FAR);
		coded_slices_in_buffer = F_DECODE(coded_datahdr->extra_feedback,
						  CODED_SLICES_IN_BUFFER);

		if (bu < min_bu)
			min_bu = bu;

		coded_bytes += coded_datahdr->bytes_written;
		offset += CODED_BUFFER_INFO_SECTION_SIZE;
	} while (coded_slices_so_far != coded_slices_in_buffer);

	*bytes_coded = coded_bytes;
	*feedback = F_INSERT(coded_datahdr->feedback, min_bu, CODED_FIRST_BU);
	*extra_feedback = coded_datahdr->extra_feedback;
}

/*
 * Move around the reconstructed data and handle the list for frame reordering
 */
static void process_reconstructed(struct topaz_stream_context *str_ctx, unsigned char is_coded,
				  enum img_frame_type frame_type, struct list_item **recon_list)
{
	struct img_video_context *video = str_ctx->enc_ctx->video;
	unsigned char *tmp_buffer;
	unsigned short width, height;
	struct list_item *new_item;
	struct img_recon_node *new_node;
	struct list_item *current_item;

	*recon_list = NULL;

	if (!video->output_reconstructed)
		return;

	/* Create new reconstructed node */
	new_item = kzalloc(sizeof(*new_item), GFP_KERNEL);
	if (!new_item)
		return;

	new_item->data = kzalloc(sizeof(*new_node), GFP_KERNEL);
	if (!new_item->data) {
		kfree(new_item);
		new_item = NULL;
		return;
	}

	new_node = (struct img_recon_node *)new_item->data;

	if (is_coded) {
		topaz_update_host_mem(str_ctx->vxe_ctx, video->recon_buffer);
		tmp_buffer = (unsigned char *)video->recon_buffer->cpu_virt;
		width = ALIGN_64(video->width);
		height = ALIGN_64(video->frame_height);

		new_node->buffer = kzalloc(width * height * 3 / 2, GFP_KERNEL);
		if (!new_node->buffer) {
			kfree(new_item->data);
			kfree(new_item);
			new_item = NULL;
			new_node = NULL;
			return;
		}
		memcpy(new_node->buffer, tmp_buffer, width * height * 3 / 2);

	} else {
		new_node->buffer = NULL;
	}
	new_node->poc = video->recon_poc;

	/* Add new node to the queue */
	if (!video->ref_frame) {
		/* First element */
		new_item->next = NULL;
		video->ref_frame = new_item;
	} else if (new_node->poc == 0) {
		/* First element after aborted sequence */
		current_item = video->ref_frame;

		while (current_item->next)
			current_item = current_item->next;

		/* Insert at end */
		new_item->next = NULL;
		current_item->next = new_item;
	} else {
		struct img_recon_node *head_node = (struct img_recon_node *)video->ref_frame->data;

		if (head_node->poc > new_node->poc) {
			/* Insert at start */
			new_item->next = video->ref_frame;
			video->ref_frame = new_item;
		} else {
			struct img_recon_node *next_node = NULL;

			current_item = video->ref_frame;
			while (current_item->next) {
				next_node = (struct img_recon_node *)current_item->next->data;

				if (next_node->poc > new_node->poc) {
					/* Insert between current and next */
					new_item->next = current_item->next;
					current_item->next = new_item;
					break;
				}
				current_item = current_item->next;
			}

			if (!current_item->next) {
				/* Insert at end */
				new_item->next = NULL;
				current_item->next = new_item;
			}
		}
	}

	if (video->next_recon == 0) {
		video->next_recon++;
		/* Flush all frames */
		*recon_list = video->ref_frame;
		video->ref_frame = NULL;
	} else if (new_node->poc == video->next_recon) {
		struct list_item *flush_tail = video->ref_frame;
		struct img_recon_node *next_node;

		video->next_recon++;

		/* Find all flushable frames */
		while (flush_tail->next) {
			next_node = (struct img_recon_node *)flush_tail->next->data;

			/* Flushing sequence ends when POCs no longer match */
			if (next_node->poc != video->next_recon)
				break;

			video->next_recon++;

			flush_tail = flush_tail->next;
		}

		/* Flush consecutive sequence */
		*recon_list = video->ref_frame;

		/* Set new head */
		video->ref_frame = flush_tail->next;

		/* Separate sequences */
		flush_tail->next = NULL;
	}
}

int topaz_process_message(struct topaz_stream_context *str_ctx, struct mtx_tohost_msg tohost_msg)
{
	struct driver_tohost_msg *driver_msg;
	struct list_item *current_el = NULL;
	struct img_enc_context *enc;
	struct img_video_context *video;
	struct list_item *message_list = NULL;
	unsigned int index = 0;

	enc = str_ctx->enc_ctx;
	video = enc->video;

	/* add a new element */
	current_el = kzalloc(sizeof(*current_el), GFP_KERNEL);
	if (!current_el)
		return IMG_ERROR_OUT_OF_MEMORY;

	current_el->data = kzalloc(sizeof(*driver_msg), GFP_KERNEL);
	if (!current_el->data) {
		kfree(current_el);
		current_el = NULL;
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	/* adding to head */
	current_el->next = message_list;
	message_list = current_el;

	driver_msg = (struct driver_tohost_msg *)current_el->data;
	driver_msg->cmd_id = tohost_msg.cmd_id;
	driver_msg->data = tohost_msg.data;
	driver_msg->command_data_buf = tohost_msg.command_data_buf;

	switch (tohost_msg.cmd_id) {
	case MTX_MESSAGE_ACK:
		driver_msg->input_cmd_id = (enum mtx_cmd_id)F_DECODE(tohost_msg.input_cmd_word,
					    MTX_MSG_CMD_ID);
		break;

	case MTX_MESSAGE_CODED:
	{
		struct coded_data_hdr *coded_datahdr = NULL;
		unsigned int feedback, extra_feedback;
		unsigned char active_coded_package_idx;
		struct img_feedback_element *feedback_struct;

		active_coded_package_idx = tohost_msg.coded_pkg_idx;

		get_coded_buffer(str_ctx, (void **)&coded_datahdr, TRUE,
				 active_coded_package_idx);

		feedback = coded_datahdr->feedback;
		extra_feedback = coded_datahdr->extra_feedback;

		/* detect the FrameNum of the coded buffer */
		feedback_struct = (struct img_feedback_element *)&driver_msg->feedback;

		combine_feedback(str_ctx, active_coded_package_idx, &feedback, &extra_feedback,
				 &feedback_struct->bytes_coded);

		feedback_struct->coded_buffer_count = F_DECODE(extra_feedback,
							       CODED_BUFFER_NUMBER_USED);

		/* Give the header buffer to the feedback structure */
		feedback_struct->coded_package = video->coded_package[active_coded_package_idx];
		feedback_struct->active_coded_package_idx       = active_coded_package_idx;
		/* update this frame, using the info from the coded buffer */
		feedback_struct->coded_package->coded_buffer[feedback_struct->coded_slot_num] =
			video->coded_package[active_coded_package_idx]->coded_buffer[feedback_struct
			->coded_slot_num];

		feedback_struct->first_bu = F_DECODE(feedback, CODED_FIRST_BU);
		feedback_struct->storage_frame_num = F_DECODE(feedback, CODED_STORAGE_FRAME_NUM);
		feedback_struct->entire_frame = F_DECODE(feedback, CODED_ENTIRE_FRAME);
		feedback_struct->is_skipped = F_DECODE(feedback, CODED_IS_SKIPPED);
		feedback_struct->is_coded = F_DECODE(feedback, CODED_IS_CODED);
		feedback_struct->recon_idx = F_DECODE(feedback, CODED_RECON_IDX);
		feedback_struct->source_slot = F_DECODE(feedback, CODED_SOURCE_SLOT);
		feedback_struct->frame_type = (enum img_frame_type)F_DECODE
						(feedback, CODED_FRAME_TYPE);
		feedback_struct->slice_num = F_DECODE(feedback, CODED_SLICE_NUM);
		feedback_struct->poc = video->source_slot_poc[feedback_struct->source_slot];

		feedback_struct->slices_in_buffer = F_DECODE(extra_feedback,
							     CODED_SLICES_IN_BUFFER);
		feedback_struct->field = F_DECODE(extra_feedback, CODED_FIELD);
		feedback_struct->patched_recon = F_DECODE(extra_feedback,
							  CODED_PATCHED_RECON);
		feedback_struct->bytes_coded = coded_datahdr->bytes_written;
		feedback_struct->host_ctx = coded_datahdr->host_ctx;

		if (video->highest_storage_number != feedback_struct->storage_frame_num &&
		    video->standard != IMG_STANDARD_H263) {
			if (feedback_struct->storage_frame_num ==
				((video->highest_storage_number + 1) & 0x03)) {
				/* it is piece of the next frame */
				video->highest_storage_number = feedback_struct->storage_frame_num;
				/* retrieve next WB */
				video->encode_pic_processing--;
				video->extra_wb_retrieved++;
			} else if (feedback_struct->storage_frame_num ==
				((video->highest_storage_number + 2) & 0x03)) {
				/* it is piece of the next frame */
				video->highest_storage_number = feedback_struct->storage_frame_num;

				video->encode_pic_processing -= 2;
				video->extra_wb_retrieved += 2;
			}
		}

		while (index < feedback_struct->coded_buffer_count) {
			if (video->coded_package
			[active_coded_package_idx]->coded_buffer[index]->lock == SW_LOCK)
				/* Unlock coded buffers used*/
				topaz_release_buffer(str_ctx,
						     (struct img_buffer *)(video->coded_package
						[active_coded_package_idx]->coded_buffer[index]),
					FALSE);
			index++;
		}

		/* Unlock header buffer */
		topaz_release_buffer(str_ctx, video->coded_package
				     [feedback_struct->active_coded_package_idx]->header_buffer,
				     FALSE);

		/* Release the coded slot */
		video->coded_package[feedback_struct->active_coded_package_idx]->busy = FALSE;

		feedback_struct->src_frame = video->source_slot_buff[feedback_struct->source_slot];

		/* Detect the slice number based on the Slice Map and the first BU in a slice */
		if (feedback_struct->bytes_coded) {
			struct img_buffer *output_slice_map;
			unsigned char *src_buffer = NULL;
			unsigned char slices_per_picture;
			unsigned short first_bu_in_slice;
			unsigned char slice_number;
			unsigned char index;
			unsigned char slice_size_in_bu[MAX_SLICESPERPIC];

			/* postion the start of the slice map */
			output_slice_map = &video->slice_map[feedback_struct->source_slot];

			topaz_get_buffer(str_ctx, output_slice_map, (void **)&src_buffer, FALSE);

			/* retrieve slices per field */
			slices_per_picture = *src_buffer;
			src_buffer++;

			/* retrieve first BU in slices and Slice sizes in BUs */
			first_bu_in_slice = 0;

			for (index = 0; index < slices_per_picture; index++) {
				slice_number = src_buffer[index * 2];
				slice_size_in_bu[slice_number] = src_buffer[index * 2 + 1];

				first_bu_in_slice += slice_size_in_bu[slice_number];
			}
			topaz_release_buffer(str_ctx, output_slice_map, FALSE);

			feedback_struct->slices_per_picture = slices_per_picture;
		}

		if (feedback_struct->entire_frame) {
			/* we encoded the entire frame  */
			video->frames_encoded++;
#ifdef DEBUG_ENCODER_DRIVER
			pr_info("FRAMES_CODED[%d]\n", video->frames_encoded);
#endif

			if (feedback_struct->coded_package->coded_buffer[0]) {
				/* Send callback for coded_buffer ready */
				global_topaz_core_context->vxe_str_processed_cb(str_ctx->vxe_ctx,
					VXE_CB_CODED_BUFF_READY,
					(void *)(feedback_struct->coded_package->coded_buffer[0]),
					feedback_struct->bytes_coded, video->frames_encoded,
					feedback_struct->frame_type);
			}

			if (!str_ctx->vxe_ctx->eos) {
				if (feedback_struct->src_frame) {
					/* Send callback for src ready */
					global_topaz_core_context->vxe_str_processed_cb(
						str_ctx->vxe_ctx,
						VXE_CB_SRC_FRAME_RELEASE,
						(void *)(feedback_struct
								 ->src_frame),
						0, 0, 0);
				}
			}
			if (video->flush_at_frame > 0 &&
			    video->frames_encoded >= video->flush_at_frame)
				feedback_struct->last_frame_encoded = TRUE;

			if (feedback_struct->patched_recon && video->patched_recon_buffer) {
				video->recon_buffer = video->patched_recon_buffer;
				video->patched_recon_buffer = NULL;
			} else {
				video->recon_buffer =
					&video->recon_pictures[feedback_struct->recon_idx];
			}
			video->recon_poc = feedback_struct->poc;

			video->frame_type = feedback_struct->frame_type;

			process_reconstructed(str_ctx, feedback_struct->is_coded, video->frame_type,
					      &feedback_struct->recon_list);

			/* If there are more frames to be encoded, release the source slot */
			if (video->frame_count == 0 ||
			    video->encode_requested < video->frame_count)
				video->source_slot_buff[feedback_struct->source_slot] = NULL;

			if (!video->extra_wb_retrieved) {
				video->encode_pic_processing--;
				video->highest_storage_number =
					(video->highest_storage_number + 1) & 0x03;
			} else {
				video->extra_wb_retrieved--;
			}
		} else {
			if (feedback_struct->coded_package->coded_buffer[0]) {
				/* Send callback for coded_buffer ready */
				global_topaz_core_context->vxe_str_processed_cb(str_ctx->vxe_ctx,
					VXE_CB_CODED_BUFF_READY,
					(void *)(feedback_struct->coded_package->coded_buffer[0]),
					feedback_struct->bytes_coded, video->frames_encoded,
					feedback_struct->frame_type);
			}
		}

		if (feedback_struct->entire_frame &&
		    (video->enable_sel_stats_flags & ESF_FIRST_STAGE_STATS))
			feedback_struct->motion_search_statistics_buf =
			&video->firstpass_out_param_buf[feedback_struct->source_slot];
		else
			feedback_struct->motion_search_statistics_buf = NULL;

		if (video->frame_count > 0 && video->frames_encoded >= video->frame_count)
			feedback_struct->last_frame_encoded = TRUE;

		if (feedback_struct->entire_frame &&
		    (video->enable_sel_stats_flags & ESF_MP_BEST_MB_DECISION_STATS ||
		     video->enable_sel_stats_flags & ESF_MP_BEST_MOTION_VECTOR_STATS))
			feedback_struct->best_multipass_statistics_buf =
				&video->firstpass_out_best_multipass_param_buf
							[feedback_struct->source_slot];
		else
			feedback_struct->best_multipass_statistics_buf = NULL;
		break;
	}
	default:
		break;
	}

	kfree(current_el->data);
	kfree(current_el);

	return IMG_SUCCESS;
}

void handle_encoder_firmware_response(struct img_writeback_msg *wb_msg, void *priv)
{
	struct topaz_stream_context *str_ctx;
	struct mtx_tohost_msg tohost_msg;
	int index;
	unsigned int cmd_buf_slot = COMM_CMD_DATA_BUF_SLOT_NONE;
	unsigned int *cmdbuf_devaddr;

	str_ctx = (struct topaz_stream_context *)priv;

	if (!str_ctx)
		return;

	memset(&tohost_msg, 0, sizeof(tohost_msg));
	tohost_msg.cmd_id = (enum mtx_message_id)F_DECODE(wb_msg->cmd_word, MTX_MSG_MESSAGE_ID);

	switch (tohost_msg.cmd_id) {
	case MTX_MESSAGE_ACK:
#ifdef DEBUG_ENCODER_DRIVER
		pr_info("MTX_MESSAGE_ACK received\n");
#endif

		tohost_msg.wb_val = wb_msg->writeback_val;
		tohost_msg.input_cmd_word = wb_msg->cmd_word;
		tohost_msg.data = wb_msg->data;
		break;
	case MTX_MESSAGE_CODED:
#ifdef DEBUG_ENCODER_DRIVER
		pr_info("MTX_MESSAGE_CODED Received\n");
#endif
		tohost_msg.input_cmd_word = wb_msg->cmd_word;
		tohost_msg.coded_pkg_idx = wb_msg->coded_package_consumed_idx;
		break;
	default:
		break;
	}

	cmdbuf_devaddr = global_cmd_data_dev_addr.cpu_virt;

	for (index = 0; index < TOPAZ_CMD_DATA_BUF_NUM; index++) {
		if (*cmdbuf_devaddr == wb_msg->extra_data) {
			/* Input cmd buffer found */
			cmd_buf_slot = index;
			break;
		}
		cmdbuf_devaddr++;
	}

	if (cmd_buf_slot != COMM_CMD_DATA_BUF_SLOT_NONE) {
		tohost_msg.command_data_buf = &global_cmd_data_info[cmd_buf_slot];
		topaz_release_cmd_data_buffer(tohost_msg.command_data_buf);

	} else {
		tohost_msg.command_data_buf = NULL;
	}

	mutex_lock_nested(str_ctx->vxe_ctx->mutex, SUBCLASS_VXE_V4L2);
	topaz_process_message(str_ctx, tohost_msg);
	mutex_unlock(str_ctx->vxe_ctx->mutex);
}

static inline void populate_firmware_message(struct vidio_ddbufinfo *dest, unsigned int dest_offset,
					     struct vidio_ddbufinfo *src, unsigned int src_offset)
{
	*(unsigned int *)((unsigned long)dest->cpu_virt + dest_offset) =
		src->dev_virt + src_offset;
}

/*
 * init_hardware
 */
int init_topaz_core(void *device_handle, unsigned int *num_pipes,
		    unsigned int mmu_flags, void *callback)
{
	unsigned int index;

	if (is_topaz_core_initialized)
		return IMG_ERROR_INVALID_PARAMETERS;

	is_topaz_core_initialized = TRUE;

	global_topaz_core_context = kzalloc(sizeof(*global_topaz_core_context), GFP_KERNEL);
	if (!global_topaz_core_context) {
		is_topaz_core_initialized = FALSE;
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	/* Initialise device context. */
	global_topaz_core_context->dev_handle = (struct topaz_dev_ctx *)device_handle;
	global_topaz_core_context->vxe_str_processed_cb = (vxe_cb)callback;

	lst_init(&global_topaz_core_context->topaz_stream_list);

	*num_pipes = topazdd_get_num_pipes(device_handle);

	/* allocate memory for HighCmd FIFO data section */
	if (topaz_mmu_alloc(global_topaz_core_context->dev_handle->topaz_mmu_ctx.mmu_context_handle,
			    global_topaz_core_context->dev_handle->vxe_arg, MMU_GENERAL_HEAP_ID,
			    1, (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
						      SYS_MEMATTRIB_WRITECOMBINE),
			    4 * TOPAZ_CMD_DATA_BUF_NUM, 64, &global_cmd_data_dev_addr)) {
		IMG_DBG_ASSERT("Global command data info buff alloc failed\n" != NULL);
		kfree(global_topaz_core_context);
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	for (index = 0; index < ARRAY_SIZE(global_cmd_data_info); index++) {
		if (topaz_mmu_alloc
			(global_topaz_core_context->dev_handle->topaz_mmu_ctx.mmu_context_handle,
			 global_topaz_core_context->dev_handle->vxe_arg, MMU_GENERAL_HEAP_ID, 1,
			 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
				 SYS_MEMATTRIB_WRITECOMBINE),
			 TOPAZ_CMD_DATA_BUF_SIZE, 64, &global_cmd_data_info[index])) {
			IMG_DBG_ASSERT("Global command data info buff alloc failed\n" != NULL);
			topaz_mmu_free(global_topaz_core_context->dev_handle->vxe_arg,
				       &global_cmd_data_dev_addr);
			kfree(global_topaz_core_context);
			return IMG_ERROR_OUT_OF_MEMORY;
		}
		populate_firmware_message(&global_cmd_data_dev_addr, 4 * index,
					  &global_cmd_data_info[index], 0);
		global_cmd_data_busy[index] = FALSE;
	}

	/*Lock for locking critical section in TopazAPI*/
	global_topaz_core_context->mutex = kzalloc(sizeof(*global_topaz_core_context->mutex),
						   GFP_KERNEL);
	if (!global_topaz_core_context->mutex)
		return IMG_ERROR_OUT_OF_MEMORY;

	mutex_init(global_topaz_core_context->mutex);
	return IMG_SUCCESS;
}

/*
 * deinit_topaz_core
 */
int deinit_topaz_core(void)
{
	unsigned int index;

	mutex_destroy(global_topaz_core_context->mutex);
	kfree(global_topaz_core_context->mutex);
	global_topaz_core_context->mutex = NULL;

	if (topaz_mmu_free(global_topaz_core_context->dev_handle->vxe_arg,
			   &global_cmd_data_dev_addr))
		IMG_DBG_ASSERT("Free failed" == NULL);

	for (index = 0; index < ARRAY_SIZE(global_cmd_data_info); index++)
		if (topaz_mmu_free(global_topaz_core_context->dev_handle->vxe_arg,
				   &global_cmd_data_info[index]))
			IMG_DBG_ASSERT("Free failed"  == NULL);

	return IMG_SUCCESS;
}

static unsigned short create_gop_frame(unsigned char *level, unsigned char reference,
				       unsigned char pos, unsigned char ref0_level,
				       unsigned char ref1_level, enum img_frame_type frame_type)
{
	*level = max(ref0_level, ref1_level) + 1;

	return F_ENCODE(reference, GOP_REFERENCE) |
	       F_ENCODE(pos, GOP_POS) |
	       F_ENCODE(ref0_level, GOP_REF0) |
	       F_ENCODE(ref1_level, GOP_REF1) |
	       F_ENCODE(frame_type, GOP_FRAMETYPE);
}

static void gop_split(unsigned short **gop_structure, signed char ref0,
		      signed char ref1, unsigned char ref0_level,
		      unsigned char ref1_level, unsigned char pic_on_level[])
{
	unsigned char distance = ref1 - ref0;
	unsigned char position = ref0 + (distance >> 1);
	unsigned char level;

	if (distance == 1)
		return;

	/* mark middle as this level */
	(*gop_structure)++;
	**gop_structure = create_gop_frame(&level, distance >= 3, position, ref0_level, ref1_level,
					   IMG_INTER_B);
	pic_on_level[level]++;

	if (distance >= 4)
		gop_split(gop_structure, ref0, position, ref0_level, level, pic_on_level);

	if (distance >= 3)
		gop_split(gop_structure, position, ref1, level, ref1_level, pic_on_level);
}

static void mini_gop_generate_hierarchical(unsigned short gop_structure[],
					   unsigned int bframe_count,
					   unsigned int ref_spacing,
					   unsigned char pic_on_level[])
{
	unsigned char level;

	gop_structure[0] = create_gop_frame(&level, TRUE, bframe_count, ref_spacing, 0,
					    IMG_INTER_P);
	pic_on_level[level]++;

	gop_split(&gop_structure, -1, bframe_count, ref_spacing, ref_spacing + 1, pic_on_level);
}

static void mini_gop_generate_flat(unsigned short gop_structure[],
				   unsigned int bframe_count,
				   unsigned int ref_spacing,
	unsigned char pic_on_level[])
{
	/* B B B B P */
	unsigned char encode_order_pos;
	unsigned char level;

	gop_structure[0] = create_gop_frame(&level, TRUE, MAX_BFRAMES, ref_spacing, 0,
					    IMG_INTER_P);
	pic_on_level[level]++;

	for (encode_order_pos = 1; encode_order_pos < MAX_GOP_SIZE; encode_order_pos++) {
		gop_structure[encode_order_pos] = create_gop_frame(&level,
								   FALSE, encode_order_pos - 1,
								   ref_spacing, ref_spacing + 1,
								   IMG_INTER_B);
		pic_on_level[level] = bframe_count;
	}
}

/*
 * Create the MTX-side encoder context
 */
static int topaz_video_create_mtx_context(struct topaz_stream_context *str_ctx,
					  struct img_video_params *video_params)
{
	struct img_video_context *video;
	struct img_enc_context *enc;
	int index, i, j;
	void *mtx_enc_context_mem;
	struct img_mtx_video_context *mtx_enc_context;
	unsigned char flag;
	unsigned int max_cores;
	unsigned int bit_limit;
	unsigned int vert_mv_limit;
	unsigned int packed_strides;
	unsigned short *gop_structure;

	max_cores = topazdd_get_num_pipes(global_topaz_core_context->dev_handle);

	enc = str_ctx->enc_ctx;
	video = enc->video;

	mtx_enc_context = (struct img_mtx_video_context *)(video->mtx_enc_ctx_mem.cpu_virt);

	/* clear the context region */
	memset(mtx_enc_context, 0x00, MTX_CONTEXT_SIZE);

	mtx_enc_context_mem = (void *)(&enc->video->mtx_enc_ctx_mem);

	mtx_enc_context->initial_qp_i = video->rc_params.initial_qp_i;
	mtx_enc_context->initial_qp_p = video->rc_params.initial_qp_p;
	mtx_enc_context->initial_qp_b = video->rc_params.initial_qp_b;

	mtx_enc_context->cqp_offset = (video->rc_params.qcp_offset & 0x1f) |
				       ((video->rc_params.qcp_offset & 0x1f) << 8);
	mtx_enc_context->standard = video->standard;
	mtx_enc_context->width_in_mbs = video->width >> 4;
	mtx_enc_context->picture_height_in_mbs = video->picture_height >> 4;

	mtx_enc_context->kick_size = video->kick_size;
	mtx_enc_context->kicks_per_bu = video->kicks_per_bu;
	mtx_enc_context->kicks_per_picture = (mtx_enc_context->width_in_mbs *
		mtx_enc_context->picture_height_in_mbs) / video->kick_size;

	mtx_enc_context->output_reconstructed = video->output_reconstructed;

	mtx_enc_context->vop_time_resolution = video->vop_time_resolution;

	mtx_enc_context->max_slices_per_picture = video->slices_per_picture;

	mtx_enc_context->is_interlaced = video->is_interlaced;
	mtx_enc_context->top_field_first = video->top_field_first;
	mtx_enc_context->arbitrary_so = video->arbitrary_so;

	mtx_enc_context->idr_period = video->idr_period;
	mtx_enc_context->bframe_count = video->rc_params.bframes;
	mtx_enc_context->hierarchical = (unsigned char)video->rc_params.hierarchical;
	mtx_enc_context->intra_loop_cnt = video->intra_cnt;
	mtx_enc_context->ref_spacing = video_params->ref_spacing;

	mtx_enc_context->debug_crcs = video_params->debug_crcs;

	mtx_enc_context->fw_num_pipes = enc->pipes_to_use;
	mtx_enc_context->fw_first_pipe = enc->base_pipe;
	mtx_enc_context->fw_last_pipe = enc->base_pipe + enc->pipes_to_use - 1;
	mtx_enc_context->fw_pipes_to_use_flags = 0;

	flag = 0x1 << mtx_enc_context->fw_first_pipe;
	/* Pipes used MUST be contiguous from the BasePipe offset */
	for (index = 0; index < mtx_enc_context->fw_num_pipes; index++, flag <<= 1)
		mtx_enc_context->fw_pipes_to_use_flags |= flag;

	mtx_enc_context->format = video_params->format;

	/* copy scaler values to context in case we need them later */
	video->enable_scaler            = video_params->enable_scaler;
	video->crop_left                = video_params->crop_left;
	video->crop_right               = video_params->crop_right;
	video->crop_top                 = video_params->crop_top;
	video->crop_bottom              = video_params->crop_bottom;
	video->source_width             = video_params->source_width;
	video->source_frame_height      = video_params->source_frame_height;
	video->intra_pred_modes         = video_params->intra_pred_modes;

	topaz_setup_input_format(video, &mtx_enc_context->scaler_setup);
	topaz_setup_input_csc(video, &mtx_enc_context->scaler_setup, &mtx_enc_context->csc_setup,
			      video_params->csc_preset);

	mtx_enc_context->enable_mvc = video->enable_mvc;
	mtx_enc_context->mvc_view_idx = video->mvc_view_idx;

	if (video->standard == IMG_STANDARD_H264)
		mtx_enc_context->no_sequence_headers = video->no_sequence_headers;

	mtx_enc_context->coded_header_per_slice = video->coded_header_per_slice;

	packed_strides = topaz_get_packed_buffer_strides
		(video->buffer_stride_bytes, video->format, video_params->enable_scaler,
		 video_params->is_interlaced, video_params->is_interleaved);

	mtx_enc_context->pic_row_stride_bytes =
		F_ENCODE(F_DECODE(packed_strides, MTX_MSG_PICMGMT_STRIDE_Y),
			 TOPAZHP_CR_CUR_PIC_LUMA_STRIDE) |
		F_ENCODE(F_DECODE(packed_strides, MTX_MSG_PICMGMT_STRIDE_UV),
			 TOPAZHP_CR_CUR_PIC_CHROMA_STRIDE);

	mtx_enc_context->rc_mode = video->rc_params.rc_mode;
	if (mtx_enc_context->rc_mode == IMG_RCMODE_VCM) {
		mtx_enc_context->rc_vcm_mode = video->rc_params.rc_vcm_mode;
		mtx_enc_context->rc_cfs_max_margin_perc = video->rc_params.rc_cfs_max_margin_perc;
	}

	mtx_enc_context->disable_bit_stuffing = (unsigned char)video_params->disable_bit_stuffing;

	mtx_enc_context->first_pic = TRUE;

	/*Content Adaptive Rate Control Parameters*/
	if (video_params->carc) {
		mtx_enc_context->jmcomp_rc_reg0 =
			F_ENCODE(video_params->carc_pos_range, TOPAZHP_CR_CARC_POS_RANGE)  |
			F_ENCODE(video_params->carc_pos_scale,  TOPAZHP_CR_CARC_POS_SCALE)   |
			F_ENCODE(video_params->carc_neg_range, TOPAZHP_CR_CARC_NEG_RANGE)    |
			F_ENCODE(video_params->carc_neg_scale,  TOPAZHP_CR_CARC_NEG_SCALE);

		mtx_enc_context->jmcomp_rc_reg1 =
			F_ENCODE(video_params->carc_threshold, TOPAZHP_CR_CARC_THRESHOLD) |
			F_ENCODE(video_params->carc_cutoff, TOPAZHP_CR_CARC_CUTOFF)  |
			F_ENCODE(video_params->carc_shift, TOPAZHP_CR_CARC_SHIFT);
	} else {
		mtx_enc_context->jmcomp_rc_reg0 = 0;
		mtx_enc_context->jmcomp_rc_reg1 = 0;
	}

	mtx_enc_context->mv_clip_config =
		F_ENCODE(video_params->no_offscreen_mv, TOPAZHP_CR_MVCALC_RESTRICT_PICTURE);

	mtx_enc_context->lritc_cache_chunk_config = 0;

	mtx_enc_context->ipcm_0_config =
		F_ENCODE(enc->video->cabac_bin_flex, TOPAZ_VLC_CR_CABAC_BIN_FLEX) |
		F_ENCODE(DEFAULT_CABAC_DB_MARGIN, TOPAZ_VLC_CR_CABAC_DB_MARGIN);

	bit_limit = 3100;

	mtx_enc_context->ipcm_1_config = F_ENCODE(bit_limit, TOPAZ_VLC_CR_IPCM_THRESHOLD) |
		F_ENCODE(enc->video->cabac_bin_limit, TOPAZ_VLC_CR_CABAC_BIN_LIMIT);

	/* leave alone until high profile and constrained modes are defined. */
	mtx_enc_context->h264_comp_control  = F_ENCODE((video->cabac_enabled ? 0 : 1),
						       TOPAZHP_CR_H264COMP_8X8_CAVLC);
	mtx_enc_context->h264_comp_control |=
		F_ENCODE(video_params->use_default_scaling_list ? 1 : 0,
			 TOPAZHP_CR_H264COMP_DEFAULT_SCALING_LIST);
	mtx_enc_context->h264_comp_control |= F_ENCODE(video->h264_8x8_transform ? 1 : 0,
						       TOPAZHP_CR_H264COMP_8X8_TRANSFORM);
	mtx_enc_context->h264_comp_control |= F_ENCODE(video->h264_intra_constrained ? 1 : 0,
						       TOPAZHP_CR_H264COMP_CONSTRAINED_INTRA);

	mtx_enc_context->mc_adaptive_rounding_disable = video_params->vp_adaptive_rounding_disable;
	mtx_enc_context->h264_comp_control |=
		F_ENCODE(mtx_enc_context->mc_adaptive_rounding_disable ? 0 : 1,
			 TOPAZHP_CR_H264COMP_ADAPT_ROUND_ENABLE);

	if (!mtx_enc_context->mc_adaptive_rounding_disable)
		for (i = 0; i < 4; i++)
			for (j = 0; j < AR_REG_SIZE; j++)
				mtx_enc_context->mc_adaptive_rounding_offsets[j][i] =
					video_params->vp_adaptive_rounding_offsets[j][i];

	if (video->standard == IMG_STANDARD_H264)
		mtx_enc_context->h264_comp_control |=
			F_ENCODE(USE_VCM_HW_SUPPORT, TOPAZHP_CR_H264COMP_VIDEO_CONF_ENABLE);

	mtx_enc_context->h264_comp_control |=
		F_ENCODE(video_params->use_custom_scaling_lists & 0x01 ? 1 : 0,
			 TOPAZHP_CR_H264COMP_CUSTOM_QUANT_4X4_INTRA_LUMA_ENABLE) |
		F_ENCODE(video_params->use_custom_scaling_lists & 0x02 ? 1 : 0,
			 TOPAZHP_CR_H264COMP_CUSTOM_QUANT_4X4_INTRA_CB_ENABLE) |
		F_ENCODE(video_params->use_custom_scaling_lists & 0x04 ? 1 : 0,
			 TOPAZHP_CR_H264COMP_CUSTOM_QUANT_4X4_INTRA_CR_ENABLE) |
		F_ENCODE(video_params->use_custom_scaling_lists & 0x08 ? 1 : 0,
			 TOPAZHP_CR_H264COMP_CUSTOM_QUANT_4X4_INTER_LUMA_ENABLE) |
		F_ENCODE(video_params->use_custom_scaling_lists & 0x10 ? 1 : 0,
			 TOPAZHP_CR_H264COMP_CUSTOM_QUANT_4X4_INTER_CB_ENABLE) |
		F_ENCODE(video_params->use_custom_scaling_lists & 0x20 ? 1 : 0,
			 TOPAZHP_CR_H264COMP_CUSTOM_QUANT_4X4_INTER_CR_ENABLE) |
		F_ENCODE(video_params->use_custom_scaling_lists & 0x40 ? 1 : 0,
			 TOPAZHP_CR_H264COMP_CUSTOM_QUANT_8X8_INTRA_LUMA_ENABLE) |
		F_ENCODE(video_params->use_custom_scaling_lists & 0x80 ? 1 : 0,
			 TOPAZHP_CR_H264COMP_CUSTOM_QUANT_8X8_INTER_LUMA_ENABLE);

	mtx_enc_context->h264_comp_control |=
		F_ENCODE(video_params->enable_lossless ? 1 : 0, TOPAZHP_CR_H264COMP_LOSSLESS) |
		F_ENCODE(video_params->lossless_8x8_prefilter ?
			 TOPAZHP_CR_H264COMP_LOSSLESS_8X8_PREFILTER_BYPASS :
			 TOPAZHP_CR_H264COMP_LOSSLESS_8X8_PREFILTER_FILTER,
			 TOPAZHP_CR_H264COMP_LOSSLESS_8X8_PREFILTER);

	mtx_enc_context->h264_comp_intra_pred_modes = 0x3ffff;// leave at default for now.

	if (video->intra_pred_modes != 0)
		mtx_enc_context->h264_comp_intra_pred_modes = video->intra_pred_modes;

	mtx_enc_context->pred_comb_control = video->pred_comb_control;

	mtx_enc_context->skip_coded_inter_intra =
		F_ENCODE(video->inter_intra_index, TOPAZHP_CR_INTER_INTRA_SCALE_IDX) |
		F_ENCODE(video->coded_skipped_index, TOPAZHP_CR_SKIPPED_CODED_SCALE_IDX);

	if (video->enable_inp_ctrl) {
		mtx_enc_context->mb_host_ctrl =
			F_ENCODE(video->enable_host_qp, TOPAZHP_CR_MB_HOST_QP) |
			F_ENCODE(video->enable_host_bias, TOPAZHP_CR_MB_HOST_SKIPPED_CODED_SCALE) |
			F_ENCODE(video->enable_host_bias, TOPAZHP_CR_MB_HOST_INTER_INTRA_SCALE);
		mtx_enc_context->pred_comb_control |= F_ENCODE(1,
				TOPAZHP_CR_INTER_INTRA_SCALE_ENABLE)
			| F_ENCODE(1, TOPAZHP_CR_SKIPPED_CODED_SCALE_ENABLE);
	}

	if (video_params->enable_cumulative_biases)
		mtx_enc_context->pred_comb_control |=
					F_ENCODE(1, TOPAZHP_CR_CUMULATIVE_BIASES_ENABLE);

	mtx_enc_context->pred_comb_control |=
		F_ENCODE((((video->inter_intra_index == 3) && (video->coded_skipped_index == 3))
					? 0 : 1), TOPAZHP_CR_INTER_INTRA_SCALE_ENABLE) |
		F_ENCODE((video->coded_skipped_index == 3 ? 0 : 1),
			 TOPAZHP_CR_SKIPPED_CODED_SCALE_ENABLE);

	mtx_enc_context->deblock_ctrl =
		F_ENCODE(video->deblock_idc, TOPAZ_DB_CR_DISABLE_DEBLOCK_IDC);

	/* Set up VLC Control Register */
	mtx_enc_context->vlc_control = 0;

	switch (video->standard) {
	case IMG_STANDARD_H264:
		/* 1 for H.264 note this is inconsistent with the sequencer value */
		mtx_enc_context->vlc_control |= F_ENCODE(1, TOPAZ_VLC_CR_CODEC);
		mtx_enc_context->vlc_control |= F_ENCODE(0, TOPAZ_VLC_CR_CODEC_EXTEND);
		break;

	default:
		break;
	}

	if (video->cabac_enabled)
		/* 2 for Mpeg4 note this is inconsistent with the sequencer value */
		mtx_enc_context->vlc_control |= F_ENCODE(1, TOPAZ_VLC_CR_CABAC_ENABLE);

	mtx_enc_context->vlc_control |= F_ENCODE(video->is_interlaced ? 1 : 0,
						 TOPAZ_VLC_CR_VLC_FIELD_CODED);
	mtx_enc_context->vlc_control |= F_ENCODE(video->h264_8x8_transform ? 1 : 0,
						 TOPAZ_VLC_CR_VLC_8X8_TRANSFORM);
	mtx_enc_context->vlc_control |= F_ENCODE(video->h264_intra_constrained ? 1 : 0,
						 TOPAZ_VLC_CR_VLC_CONSTRAINED_INTRA);

	mtx_enc_context->vlc_slice_control = F_ENCODE(video->rc_params.slice_byte_limit,
						      TOPAZ_VLC_CR_SLICE_SIZE_LIMIT);
	mtx_enc_context->vlc_slice_mb_control = F_ENCODE(video->rc_params.slice_mb_limit,
							 TOPAZ_VLC_CR_SLICE_MBS_LIMIT);

	switch (video->standard) {
	case IMG_STANDARD_H264:
		vert_mv_limit = 255; /* default to no clipping */
		if (video->vert_mv_limit)
			vert_mv_limit = enc->video->vert_mv_limit;

		/* as topaz can only cope with at most 255 (in the register field) */
		vert_mv_limit = min(255U, vert_mv_limit);
		mtx_enc_context->ipe_vector_clipping =
			F_ENCODE(1, TOPAZHP_CR_IPE_VECTOR_CLIPPING_ENABLED) |
			F_ENCODE(255, TOPAZHP_CR_IPE_VECTOR_CLIPPING_X) |
			F_ENCODE(vert_mv_limit, TOPAZHP_CR_IPE_VECTOR_CLIPPING_Y);

		mtx_enc_context->spe_mvd_clip_range = F_ENCODE(0, TOPAZHP_CR_SPE_MVD_CLIP_ENABLE);
		break;
	default:
		break;
	}

	/* Update MV Scaling settings: IDR */
	memcpy(&mtx_enc_context->mv_settings_idr, &video->mv_settings_idr,
	       sizeof(struct img_mv_settings));

	/* NonB (I or P) */
	for (i = 0; i <= MAX_BFRAMES; i++)
		memcpy(&mtx_enc_context->mv_settings_non_b[i], &video->mv_settings_non_b[i],
		       sizeof(struct img_mv_settings));

	/* WEIGHTED PREDICTION */
	mtx_enc_context->weighted_prediction_enabled = video_params->weighted_prediction;
	mtx_enc_context->mtx_weighted_implicit_bi_pred = video_params->vp_weighted_implicit_bi_pred;

	/* SEI_INSERTION */
	mtx_enc_context->insert_hrd_params = video_params->insert_hrd_params;
	if (mtx_enc_context->insert_hrd_params & enc->video->rc_params.bits_per_second)
		/* HRD parameters are meaningless without a bitrate */
		mtx_enc_context->insert_hrd_params = FALSE;

	if (mtx_enc_context->insert_hrd_params) {
		mtx_enc_context->clock_div_bitrate = (90000 * 0x100000000LL);
		mtx_enc_context->clock_div_bitrate /= enc->video->rc_params.bits_per_second;
		mtx_enc_context->max_buffer_mult_clock_div_bitrate =
			(unsigned int)(((unsigned long long)(video->rc_params.buffer_size) *
						90000ULL) /
			(unsigned long long)enc->video->rc_params.bits_per_second);
	}

	memcpy(&mtx_enc_context->in_params, &video->pic_params.in_params,
	       sizeof(struct in_rc_params));

	mtx_enc_context->lritc_cache_chunk_config =
		F_ENCODE(enc->video->chunks_per_mb,
			 TOPAZHP_CR_CACHE_CHUNKS_PER_MB)
		| F_ENCODE(enc->video->max_chunks, TOPAZHP_CR_CACHE_CHUNKS_MAX)
		| F_ENCODE(enc->video->max_chunks - enc->video->priority_chunks,
			   TOPAZHP_CR_CACHE_CHUNKS_PRIORITY);

	mtx_enc_context->first_pic_flags = video->first_pic_flags;
	mtx_enc_context->non_first_pic_flags = video->non_first_pic_flags;

	mtx_enc_context->slice_header_slot_num = -1;

	memset(mtx_enc_context->pic_on_level, 0, sizeof(mtx_enc_context->pic_on_level));

	gop_structure = (unsigned short *)(video->flat_gop_struct.cpu_virt);

	mini_gop_generate_flat(gop_structure, mtx_enc_context->bframe_count,
			       mtx_enc_context->ref_spacing, mtx_enc_context->pic_on_level);
	topaz_update_device_mem(str_ctx->vxe_ctx, &video->flat_gop_struct);

	if (video->rc_params.hierarchical) {
		memset(mtx_enc_context->pic_on_level, 0, sizeof(mtx_enc_context->pic_on_level));
		gop_structure = (unsigned short *)(video->hierar_gop_struct.cpu_virt);

		mini_gop_generate_hierarchical(gop_structure, mtx_enc_context->bframe_count,
					       mtx_enc_context->ref_spacing,
					       mtx_enc_context->pic_on_level);
		topaz_update_device_mem(str_ctx->vxe_ctx, &video->hierar_gop_struct);
	}

	topaz_update_device_mem(str_ctx->vxe_ctx, &video->mtx_enc_ctx_mem);

	populate_firmware_message
		(mtx_enc_context_mem,
		 (unsigned int)((unsigned char *)&mtx_enc_context->mv_settings_b_table -
		(unsigned char *)mtx_enc_context),
		&video->mv_settings_btable, 0);

	if (video->rc_params.hierarchical)
		populate_firmware_message
		(mtx_enc_context_mem,
		 (unsigned int)((unsigned char *)&mtx_enc_context->mv_settings_hierarchical -
			 (unsigned char *)mtx_enc_context),
		 &video->mv_settings_hierarchical, 0);

	for (i = 0; i < video->pic_nodes; i++) {
		populate_firmware_message
			(mtx_enc_context_mem,
			 (unsigned int)((unsigned char *)&mtx_enc_context->reconstructed[i] -
				 (unsigned char *)mtx_enc_context),
			 &video->recon_pictures[i], 0);

		populate_firmware_message
			(mtx_enc_context_mem,
			 (unsigned int)((unsigned char *)&mtx_enc_context->colocated[i] -
				 (unsigned char *)mtx_enc_context),
			 &video->colocated[i], 0);
	}

	for (i = 0; i < WB_FIFO_SIZE; i++)
		populate_firmware_message
			(mtx_enc_context_mem,
			 (unsigned int)((unsigned char *)&mtx_enc_context->writeback_regions[i] -
				 (unsigned char *)mtx_enc_context),
			 &global_wb_data_info[i], 0);

	for (i = 0; i < video->mv_stores; i++)
		populate_firmware_message
			(mtx_enc_context_mem,
			 (unsigned int)((unsigned char *)
				 &mtx_enc_context->mv[i] - (unsigned char *)mtx_enc_context),
			 &video->mv[i], 0);

	if (video->enable_mvc) {
		for (i = 0; i < 2; i++)
			populate_firmware_message
			(mtx_enc_context_mem,
			 (unsigned int)((unsigned char *)&mtx_enc_context->inter_view_mv[i] -
				 (unsigned char *)mtx_enc_context),
			 &video->inter_view_mv[i], 0);
	}

	for (i = 0; i < (int)max_cores; i++)
		populate_firmware_message
			(mtx_enc_context_mem,
			 (unsigned int)((unsigned char *)&mtx_enc_context->above_params[i] -
				 (unsigned char *)mtx_enc_context),
			 &video->above_params[i], 0);

	/* SEI insertion */
	if (video_params->insert_hrd_params) {
		populate_firmware_message
			(mtx_enc_context_mem,
			 (unsigned int)((unsigned char *)
				 &mtx_enc_context->sei_buffering_period_template -
				 (unsigned char *)mtx_enc_context),
			 &video->sei_buffering_period_header_mem, 0);

		populate_firmware_message
			(mtx_enc_context_mem,
			 (unsigned int)((unsigned char *)
				 &mtx_enc_context->sei_picture_timing_template -
				 (unsigned char *)mtx_enc_context),
			 &video->sei_picture_timing_header_mem, 0);
	}

	for (i = 0; i < ARRAY_SIZE(video->slice_params_template_mem); i++)
		populate_firmware_message
			(mtx_enc_context_mem,
			 (unsigned int)((unsigned char *)
				 &mtx_enc_context->slice_params_templates[i] -
				 (unsigned char *)mtx_enc_context),
			 &video->slice_params_template_mem[i], 0);

	for (i = 0; i < video->slots_in_use; i++) {
		populate_firmware_message
		(mtx_enc_context_mem,
		 (unsigned int)((unsigned char *)&mtx_enc_context->slice_map[i] -
			 (unsigned char *)mtx_enc_context),
		 &video->slice_map[i].mem_info, 0);

		/* WEIGHTED PREDICTION */
		if (video_params->weighted_prediction ||
		    video_params->vp_weighted_implicit_bi_pred == WBI_EXPLICIT) {
			populate_firmware_message
				(mtx_enc_context_mem,
				 (unsigned int)((unsigned char *)
					 &mtx_enc_context->weighted_prediction_virt_addr[i] -
					 (unsigned char *)mtx_enc_context),
				 &video->weighted_prediction_mem[i], 0);
		}
	}

	populate_firmware_message
		(mtx_enc_context_mem,
		 (unsigned int)((unsigned char *)&mtx_enc_context->flat_gop_struct -
			(unsigned char *)mtx_enc_context), &video->flat_gop_struct, 0);

	populate_firmware_message
		(mtx_enc_context_mem,
		 (unsigned int)((unsigned char *)&mtx_enc_context->flat_gop_struct -
			(unsigned char *)mtx_enc_context),
			&video->flat_gop_struct, 0);

	for (i = 0; i < video->slots_in_use; i++) {
		populate_firmware_message
			(mtx_enc_context_mem,
			 (unsigned int)((unsigned char *)&mtx_enc_context->ltref_header[i] -
			(unsigned char *)mtx_enc_context),
			&video->ltref_header[i], 0);
	}

	if (mtx_enc_context->hierarchical)
		populate_firmware_message
			(mtx_enc_context_mem,
			 (unsigned int)((unsigned char *)&mtx_enc_context->hierar_gop_struct -
			(unsigned char *)mtx_enc_context),
			&video->hierar_gop_struct, 0);

	for (i = 0; i < ARRAY_SIZE(video->pichdr_template_mem); i++)
		populate_firmware_message
			(mtx_enc_context_mem,
			 (unsigned int)((unsigned char *)&mtx_enc_context->pichdr_templates[i] -
			(unsigned char *)mtx_enc_context),
			&video->pichdr_template_mem[i], 0);

	if (video->standard == IMG_STANDARD_H264) {
		populate_firmware_message(mtx_enc_context_mem, (unsigned int)((unsigned char *)
			&mtx_enc_context->seq_header - (unsigned char *)mtx_enc_context),
			&video->seq_header_mem, 0);

		if (video->enable_mvc)
			populate_firmware_message
			(mtx_enc_context_mem,
			 (unsigned int)((unsigned char *)&mtx_enc_context->subset_seq_header -
				 (unsigned char *)mtx_enc_context),
				&video->subset_seq_header_mem, 0);
	}

	/* Store the feedback memory address for all "5" slots in the context */
	if (video->enable_sel_stats_flags & ESF_FIRST_STAGE_STATS) {
		for (i = 0; i < video->slots_in_use; i++)
			populate_firmware_message
				(mtx_enc_context_mem,
				 (unsigned int)((unsigned char *)
					 &mtx_enc_context->firstpass_out_param_addr[i] -
				(unsigned char *)mtx_enc_context),
				&video->firstpass_out_param_buf[i].mem_info, 0);
	}

	/* Store the feedback memory address for all "5" slots in the context */
	if (video->enable_sel_stats_flags & ESF_MP_BEST_MB_DECISION_STATS ||
	    video->enable_sel_stats_flags & ESF_MP_BEST_MOTION_VECTOR_STATS) {
		for (i = 0; i < video->slots_in_use; i++) {
			populate_firmware_message
				(mtx_enc_context_mem,
				 (unsigned int)((unsigned char *)
				 &mtx_enc_context->firstpass_out_best_multipass_param_addr[i] -
				(unsigned char *)mtx_enc_context),
				&video->firstpass_out_best_multipass_param_buf[i].mem_info, 0);
		}
	}

	/* Store the MB-Input control parameter memory for all the 5-slots in the context */
	if (video->enable_inp_ctrl) {
		for (i = 0; i < video->slots_in_use; i++)
			populate_firmware_message
		(mtx_enc_context_mem,
		 (unsigned int)((unsigned char *)&mtx_enc_context->mb_ctrl_in_params_addr[i] -
			 (unsigned char *)mtx_enc_context),
				&video->mb_ctrl_in_params_buf[i].mem_info, 0);
	}

	topaz_update_device_mem(str_ctx->vxe_ctx, &video->mtx_enc_ctx_mem);

	return IMG_SUCCESS;
}

/*
 * Prepares the header templates for the encode for H.264
 */
static int h264_prepare_templates(struct topaz_stream_context *str_ctx,
				  struct img_rc_params *rc_params,
				  int fine_y_search_size)
{
	struct img_enc_context *enc;
	struct img_video_context *video_ctx;
	struct pic_params *pic_params;

	enc = str_ctx->enc_ctx;
	video_ctx = enc->video;

	prepare_mv_estimates(enc);

	pic_params = &enc->video->pic_params;

	pic_params->flags = 0;

	if (rc_params->rc_enable) {
		pic_params->flags |= ISRC_FLAGS;
		setup_rc_data(enc->video, pic_params, rc_params);
	} else {
		pic_params->in_params.se_init_qp_i = rc_params->initial_qp_i;
		pic_params->in_params.mb_per_row   = (enc->video->width >> 4);
		pic_params->in_params.mb_per_bu    = rc_params->bu_size;
		pic_params->in_params.mb_per_frm   = ((unsigned int)(enc->video->width >> 4)) *
			(enc->video->frame_height >> 4);
		pic_params->in_params.bu_per_frm   = (pic_params->in_params.mb_per_frm) /
			rc_params->bu_size;
	}

	/* Prepare Slice header templates */
	generate_slice_params_template(enc, &enc->video->slice_params_template_mem[IMG_FRAME_IDR],
				       IMG_FRAME_IDR, enc->video->is_interlaced,
				       fine_y_search_size);
	topaz_update_device_mem(str_ctx->vxe_ctx, &enc->video->slice_params_template_mem
									[IMG_FRAME_IDR]);

	generate_slice_params_template(enc, &enc->video->slice_params_template_mem[IMG_FRAME_INTRA],
				       IMG_FRAME_INTRA, enc->video->is_interlaced,
				       fine_y_search_size);
	topaz_update_device_mem(str_ctx->vxe_ctx, &enc->video->slice_params_template_mem
									[IMG_FRAME_INTRA]);

	generate_slice_params_template(enc,
				       &enc->video->slice_params_template_mem[IMG_FRAME_INTER_P],
				       IMG_FRAME_INTER_P, enc->video->is_interlaced,
				       fine_y_search_size);
	topaz_update_device_mem(str_ctx->vxe_ctx, &enc->video->slice_params_template_mem
									[IMG_FRAME_INTER_P]);

	generate_slice_params_template(enc, &enc->video->slice_params_template_mem
									[IMG_FRAME_INTER_B],
		IMG_FRAME_INTER_B, enc->video->is_interlaced,
		fine_y_search_size);
	topaz_update_device_mem(str_ctx->vxe_ctx, &enc->video->slice_params_template_mem
									[IMG_FRAME_INTER_B]);

	if (video_ctx->enable_mvc) {
		generate_slice_params_template(enc, &enc->video->slice_params_template_mem
									[IMG_FRAME_INTER_P_IDR],
			IMG_FRAME_INTER_P_IDR, enc->video->is_interlaced, fine_y_search_size);
		topaz_update_device_mem(str_ctx->vxe_ctx, &enc->video->slice_params_template_mem
									[IMG_FRAME_INTER_P_IDR]);
	}

	/* Prepare Pic Params Templates */
	adjust_pic_flags(enc, rc_params, TRUE, &video_ctx->first_pic_flags);
	adjust_pic_flags(enc, rc_params, FALSE, &video_ctx->non_first_pic_flags);

	return IMG_SUCCESS;
}

/*
 * Prepares the header templates for the encode.
 */
static int topaz_video_prepare_templates(struct topaz_stream_context *str_ctx,
					 unsigned char search_range,
					 int fine_y_search_size)
{
	struct img_enc_context *enc = str_ctx->enc_ctx;
	struct img_video_context *video = enc->video;
	int err_value = IMG_ERROR_UNEXPECTED_STATE;

	switch (video->standard) {
	case IMG_STANDARD_H264:
		err_value = h264_prepare_templates(str_ctx, &video->rc_params, fine_y_search_size);
		break;
	default:
		break;
	}

	return err_value;
}

/*
 * Prepare the sequence header for h.264
 */
int topaz_h264_prepare_sequence_header(void *topaz_str_ctx, unsigned int mb_width,
				       unsigned int mb_height,
				       unsigned char vui_params_present,
				       struct h264_vui_params *params,
				       struct h264_crop_params *crop_params,
				       struct h264_sequence_header_params *sh_params,
				       unsigned char mvc_sps)
{
	struct mtx_header_params *seq_header;
	struct img_enc_context *enc;
	struct topaz_stream_context *str_ctx;

	if (!topaz_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;

	enc = str_ctx->enc_ctx;

	/* Ensure parameters are consistent with context */
	if (!enc->video->custom_scaling)
		sh_params->seq_scaling_matrix_present_flag = FALSE;

	/* Get a pointer to the memory the header will be written to */
	seq_header = (struct mtx_header_params *)(enc->video->seq_header_mem.cpu_virt);
	h264_prepare_sequence_header(seq_header, mb_width, mb_height, vui_params_present,
				     params, crop_params, sh_params, enc->video->arbitrary_so);

	topaz_update_device_mem(str_ctx->vxe_ctx, &enc->video->seq_header_mem);

	if (mvc_sps) {
		/* prepare subset sequence parameter header */
		struct mtx_header_params *subset_seq_header;

		subset_seq_header =
			(struct mtx_header_params *)(enc->video->subset_seq_header_mem.cpu_virt);
		h264_prepare_mvc_sequence_header(subset_seq_header, mb_width, mb_height,
						 vui_params_present, params, crop_params,
			sh_params);
		topaz_update_device_mem(str_ctx->vxe_ctx, &enc->video->subset_seq_header_mem);
	}

	return IMG_SUCCESS;
}

/*
 * Prepare the picture header for h.264
 */
int topaz_h264_prepare_picture_header(void *topaz_str_ctx, signed char cqp_offset)
{
	struct mtx_header_params *pic_header;
	struct topaz_stream_context *str_ctx;
	struct img_enc_context *enc;
	unsigned char dep_view_pps = FALSE;

	if (!topaz_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;
	enc = str_ctx->enc_ctx;

	/* Get a pointer to the memory the header will be written to */
	pic_header = (struct mtx_header_params *)(enc->video->pichdr_template_mem[0].cpu_virt);

	if (enc->video->enable_mvc && enc->video->mvc_view_idx != 0 &&
	    (enc->video->mvc_view_idx != (unsigned short)(NON_MVC_VIEW)))
		dep_view_pps = TRUE;

	h264_prepare_picture_header(pic_header, enc->video->cabac_enabled,
				    enc->video->h264_8x8_transform,
				    enc->video->h264_intra_constrained,
				    cqp_offset, enc->video->weighted_prediction,
				    enc->video->weighted_bi_pred,
				    dep_view_pps, enc->video->pps_scaling,
				    enc->video->pps_scaling && enc->video->custom_scaling);

	topaz_update_device_mem(str_ctx->vxe_ctx, &enc->video->pichdr_template_mem[0]);

	return IMG_SUCCESS;
}

/*
 * Prepare the AUD header for H264
 */
int topaz_h264_prepare_aud_header(void *str_context)
{
	struct mtx_header_params *aud_header;
	struct img_enc_context *enc;
	struct topaz_stream_context *str_ctx;

	str_ctx = (struct topaz_stream_context *)str_context;
	if (!str_ctx)
		return IMG_ERROR_INVALID_PARAMETERS;

	enc = str_ctx->enc_ctx;

	/* Get a pointer to the memory the header will be written to */
	aud_header = (struct mtx_header_params *)(&enc->video->aud_header_mem);

	h264_prepare_aud_header(aud_header);

	topaz_update_device_mem(str_ctx->vxe_ctx, &enc->video->aud_header_mem);

	return IMG_SUCCESS;
}

static unsigned int topaz_get_max_coded_data_size(enum img_standard standard, unsigned short width,
						  unsigned short height, unsigned int initial_qp_i)
{
	unsigned int worst_qp_size;

	if (standard == IMG_STANDARD_H264) {
		/* allocate based on worst case qp size */
		worst_qp_size = 400;
		return ((unsigned int)(width / 16) * (unsigned int)(height / 16) * worst_qp_size);
	}

	if (initial_qp_i <= 5)
		return ((unsigned int)width * (unsigned int)height * 1600) / (16 * 16);

	return ((unsigned int)width * (unsigned int)height * 900) / (16 * 16);
}

static int topaz_get_context_coded_buffer_size(struct img_enc_context *enc,
					       struct img_rc_params *rc_params,
	unsigned int *coded_buffer_size)
{
	struct img_video_context *video;

	video = enc->video;

	*coded_buffer_size = topaz_get_max_coded_data_size(video->standard, video->width,
							   video->picture_height,
							   rc_params->initial_qp_i);

	if (!video->disable_bit_stuffing && rc_params->rc_mode == IMG_RCMODE_CBR)
		*coded_buffer_size = max(*coded_buffer_size,
					 ((rc_params->bits_per_second + rc_params->frame_rate / 2) /
					  rc_params->frame_rate) * 2);

	if (video->coded_header_per_slice)
		*coded_buffer_size += CODED_BUFFER_INFO_SECTION_SIZE * video->slices_per_picture;
	else
		*coded_buffer_size += CODED_BUFFER_INFO_SECTION_SIZE;
	/* Ensure coded buffer sizes are always aligned to 1024 */
	*coded_buffer_size = ALIGN_1024(*coded_buffer_size);

	return IMG_SUCCESS;
}

/*
 * Description:	Allocate a coded package
 */
static int topaz_allocate_coded_package(struct topaz_stream_context *str_ctx,
					unsigned int coded_buffersize_bytes,
					struct coded_package_host **package)
{
	struct coded_package_host *this_package;
	struct img_video_context *video = str_ctx->enc_ctx->video;

	*package = kzalloc(sizeof(struct coded_package_host), GFP_KERNEL);

	this_package = *package;

	if (!this_package)
		return IMG_ERROR_OUT_OF_MEMORY;

	this_package->busy = 0;

	this_package->num_coded_buffers = 1;

	/* Allocate FW Buffer  IMG_BUFFER  memory */
	this_package->mtx_info.code_package_fw_buffer =
		kzalloc(sizeof(struct img_buffer), GFP_KERNEL);

	if (!this_package->mtx_info.code_package_fw_buffer)
		goto error_handling;

	/* Allocate header IMG_BUFFER memory */
	this_package->header_buffer = kzalloc(sizeof(*this_package->header_buffer), GFP_KERNEL);

	if (!this_package->header_buffer)
		goto error_handling;

	/* Allocate the FW Package (this will provide addresses
	 * of header and the coded buffer array)
	 */
	if (topaz_mmu_stream_alloc
		(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
		 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
					SYS_MEMATTRIB_WRITECOMBINE),
		 sizeof(struct coded_package_dma_info), 64,
		 &this_package->mtx_info.code_package_fw_buffer->mem_info))
		goto error_handling;

	if (topaz_mmu_stream_alloc
		(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
		 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED | SYS_MEMATTRIB_WRITECOMBINE),
		 (video->coded_header_per_slice ? video->slices_per_picture : 1) *
		 CODED_BUFFER_INFO_SECTION_SIZE,
		 64, &this_package->header_buffer->mem_info))
		goto error_handling;

	this_package->header_buffer->size =
		(video->coded_header_per_slice ? video->slices_per_picture : 1) *
		CODED_BUFFER_INFO_SECTION_SIZE;

	return IMG_SUCCESS;

error_handling:
	if (*package) {
		kfree(*package);
		*package = NULL;
	}

	if (this_package->mtx_info.code_package_fw_buffer) {
		if (this_package->mtx_info.code_package_fw_buffer->mem_info.dev_virt)
			topaz_mmu_stream_free
				(str_ctx->mmu_ctx,
				 &this_package->mtx_info.code_package_fw_buffer->mem_info);

		kfree(this_package->mtx_info.code_package_fw_buffer);
		this_package->mtx_info.code_package_fw_buffer = NULL;
	}

	kfree(this_package->header_buffer);
	this_package->header_buffer = NULL;

	return IMG_ERROR_OUT_OF_MEMORY;
}

/*
 * Create the Video Encoder context
 */
static int topaz_video_create_context(struct topaz_stream_context *str_ctx,
				      struct img_video_params *video_params,
				      struct img_rc_params *rc_params)
{
	struct img_enc_context  *enc;
	struct img_video_context        *video;
	unsigned int alloc_size;
	int index, i;
	unsigned short picture_height;
	unsigned int coded_buffer_size;
	unsigned short width_in_mbs;
	unsigned short frame_height_in_mbs;
	unsigned char pipes_to_use;
	unsigned int max_cores;
	unsigned int min_slice_height;
	unsigned int factor = 1;
	unsigned int kick_size, kicks_per_bu;
	int ret;

	max_cores = topazdd_get_num_pipes(str_ctx->core_ctx->dev_handle);

	enc = str_ctx->enc_ctx;

	picture_height =
		((video_params->frame_height >> (video_params->is_interlaced ? 1 : 0)) + 15) & ~15;
	width_in_mbs = (video_params->width + 15) >> 4;
	frame_height_in_mbs =  ((picture_height + 15) >> 4) <<
		(video_params->is_interlaced ? 1 : 0);

	if (topaz_get_encoder_caps(video_params->standard, video_params->width, picture_height,
				   &enc->caps) != IMG_SUCCESS) {
		pr_err("\nERROR: Unable to encode the size %dx%d with current hardware version\n\n",
		       video_params->width, picture_height);
		return IMG_ERROR_NOT_SUPPORTED;
	}

	/*scaler input W/H limit is 4K*/
	if (video_params->source_width > 4096) {
		pr_err("\nERROR: Source Width is bigger than the maximum supported Source Width(4096)\n");
		return IMG_ERROR_NOT_SUPPORTED;
	}

	if (video_params->source_frame_height > 4096) {
		pr_err("\nERROR: Source Height is bigger than the maximum supported Source Height(4096)\n");
		return IMG_ERROR_NOT_SUPPORTED;
	}

	if (video_params->width > enc->caps.max_width) {
		pr_err("\n ERROR: Width too big for given core revision 0x%x. Maximum width is %d.\n",
		       enc->caps.core_revision, enc->caps.max_width);
		return IMG_ERROR_NOT_SUPPORTED;
	}

	if (picture_height > enc->caps.max_height) {
		pr_err("\n ERROR: Height too big for given core revision 0x%x. Maximum height is %d.\n",
		       enc->caps.core_revision, enc->caps.max_height);
		return IMG_ERROR_NOT_SUPPORTED;
	}

	if (video_params->width < enc->caps.min_width) {
		pr_err("\n ERROR: Width too small for given core revision 0x%x. Minimum width is %d.\n",
		       enc->caps.core_revision, enc->caps.min_width);
		return IMG_ERROR_NOT_SUPPORTED;
	}

	if (video_params->standard == IMG_STANDARD_H264) {
		if (video_params->slices_per_picture < enc->caps.min_slices) {
			pr_err("WARNING: Minimum slices supported for this resolution is %d. Increasing slices per frame to %d\n",
			       enc->caps.min_slices, video_params->slices_per_picture);
			video_params->slices_per_picture =  (unsigned char)enc->caps.min_slices;
		}
		factor = min(enc->pipes_to_use, video_params->slices_per_picture);
	}

	if (video_params->standard == IMG_STANDARD_H264)
		pipes_to_use = min(enc->pipes_to_use, video_params->slices_per_picture);
	else
		pipes_to_use = 1;

	if (picture_height < (enc->caps.min_height * factor)) {
		pr_err("\n ERROR: Height too small for given core revision 0x%x. Minimum height is %d.\n",
		       enc->caps.core_revision, enc->caps.min_height * factor);
		return IMG_ERROR_NOT_SUPPORTED;
	}

	if ((unsigned int)((width_in_mbs) * (picture_height >> 4)) > enc->caps.max_mb_num) {
		pr_err("\n ERROR: Number of macroblocks too high. It should not be bigger than %d.\n",
		       enc->caps.max_mb_num);
		return IMG_ERROR_NOT_SUPPORTED;
	}

	calculate_kick_and_bu_size(width_in_mbs, picture_height / 16, video_params->is_interlaced,
				   enc->caps.max_bu_per_frame, &kick_size, &kicks_per_bu,
				   &min_slice_height);

	if (enc->caps.min_slice_height > min_slice_height)
		min_slice_height = enc->caps.min_slice_height;

	if ((unsigned int)(video_params->slices_per_picture * min_slice_height) >
	    (unsigned int)(picture_height / 16)) {
		/* we have too many slices for this resolution */
		pr_err("\n ERROR: Too many slices for this resolution.\n");
		return IMG_ERROR_NOT_SUPPORTED;
	}

	video = kzalloc(sizeof(*video), GFP_KERNEL);
	if (!video)
		return IMG_ERROR_OUT_OF_MEMORY;

	enc->video = video;

	memcpy(&video->rc_params, rc_params, sizeof(*rc_params));

	/* Setup BU size for rate control */
	video->rc_params.bu_size = kick_size * kicks_per_bu;
	rc_params->bu_size = video->rc_params.bu_size;

	video->kick_size = kick_size;
	video->kicks_per_bu = kicks_per_bu;

	video->debug_crcs = video_params->debug_crcs;

	/* stream level params */
	video->standard = video_params->standard;
	video->format = video_params->format;
	video->csc_preset = video_params->csc_preset;
	video->width = width_in_mbs << 4;
	video->frame_height = frame_height_in_mbs << 4;
	video->unrounded_width = video_params->width;
	video->unrounded_frame_height = video_params->frame_height;

	video->picture_height = picture_height;
	video->is_interlaced = video_params->is_interlaced;
	video->is_interleaved = video_params->is_interleaved;
	video->top_field_first = !(video_params->bottom_field_first);
	video->encode_requested = 0;
	video->limit_num_vectors = video_params->limit_num_vectors;
	video->disable_bit_stuffing = video_params->disable_bit_stuffing;
	video->vert_mv_limit = video_params->vert_mv_limit;
	/* Cabac Parameters */
	video->cabac_enabled = video_params->cabac_enabled;
	video->cabac_bin_limit  =  video_params->cabac_bin_limit;
	video->cabac_bin_flex   =  video_params->cabac_bin_flex;

	video->frame_count = 0;
	video->flush_at_frame = 0;
	video->flushed_at_frame = 0;
	video->encoder_idle = TRUE;
	video->high_latency = video_params->high_latency;
	video->slices_per_picture = (unsigned char)video_params->slices_per_picture;
	video->deblock_idc = video_params->deblock_idc;
	video->output_reconstructed = video_params->output_reconstructed;
	video->arbitrary_so = video_params->arbitrary_so;
	video->f_code = video_params->f_code;

	/* Default f_code is 4 */
	if (!video->f_code)
		video->f_code = 4;

	video->vop_time_resolution = video_params->vop_time_resolution;
	video->frames_encoded = 0;
	video->idr_period = video_params->idr_period;

	video->intra_cnt = video_params->intra_cnt;
	video->multi_reference_p = video_params->multi_reference_p;
	video->spatial_direct = video_params->spatial_direct;
	video->enable_sel_stats_flags = video_params->enable_sel_stats_flags;
	video->enable_inp_ctrl = video_params->enable_inp_ctrl;
	video->enable_host_bias = video_params->enable_host_bias;
	video->enable_host_qp = video_params->enable_host_qp;
	/* Line counter */
	video->line_counter = video_params->line_counter_enabled;

	video->enable_air = video_params->enable_air;
	video->num_air_mbs = video_params->num_air_mbs;
	video->air_threshold = video_params->air_threshold;
	video->air_skip_cnt = video_params->air_skip_cnt;

	video->extra_wb_retrieved = 0;
	video->highest_storage_number = 0;

	video->buffer_stride_bytes = calculate_stride(video_params->format,
						      video_params->buffer_stride_bytes,
						      video_params->source_width);
	video->buffer_height = ((video_params->buffer_height ? video_params->buffer_height :
		video_params->source_frame_height));

	if (!video_params->disable_bh_rounding)
		video->buffer_height =
			(((video->buffer_height >> (video_params->is_interlaced ? 1 : 0)) + 15)  &
			~15) << (video_params->is_interlaced ? 1 : 0);

	video_params->buffer_stride_bytes = video->buffer_stride_bytes;
	video_params->buffer_height = video->buffer_height;

	video->next_recon = 0;

	video->enable_mvc = video_params->enable_mvc;
	video->mvc_view_idx = video_params->mvc_view_idx;

	enc->pipes_to_use = pipes_to_use;

	enc->requested_pipes_to_use = pipes_to_use;
	video->slots_in_use = rc_params->bframes + 2;
	enc->video->slots_required = enc->video->slots_in_use;

	video->h264_8x8_transform = video_params->h264_8x8;
	video->h264_intra_constrained = video_params->constrained_intra;
	video->custom_scaling = (video_params->use_custom_scaling_lists != 0);
	video->pps_scaling =
		(video_params->pps_scaling &&
		(video_params->use_default_scaling_list || video->custom_scaling));

	video->encode_pic_processing = 0;
	video->next_slice = 0;
	video->ref_frame = NULL;

	/* create topaz device context */
	ret = topazdd_create_stream_context(global_topaz_core_context->dev_handle,
					    str_ctx->enc_ctx->codec,
					    handle_encoder_firmware_response, str_ctx,
					    &str_ctx->enc_ctx->video->dd_str_ctx,
					    &global_wb_data_info);

	if (ret != IMG_SUCCESS)
		return ret;

	ret = topazdd_setup_stream_ctx
		(str_ctx->enc_ctx->video->dd_str_ctx, video->frame_height,
		 video->width, (unsigned char *)&video->dd_ctx_num, &video->dd_ctx_num);

	if (ret != IMG_SUCCESS)
		return ret;

	/* Create MMU stream context */
	ret = topaz_mmu_stream_create(&global_topaz_core_context->dev_handle->topaz_mmu_ctx,
				      0x1 /*stream_id*/, str_ctx->vxe_ctx, &str_ctx->mmu_ctx);
	if (ret)
		return ret;

	/* WEIGHTED PREDICTION */
	if (video_params->weighted_prediction ||
	    video_params->vp_weighted_implicit_bi_pred == WBI_EXPLICIT) {
		video->weighted_prediction = TRUE;

		for (i = 0; i < video->slots_in_use; i++) {
			if (topaz_mmu_stream_alloc
				(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
				 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
							SYS_MEMATTRIB_WRITECOMBINE),
				 sizeof(struct weighted_prediction_values), 64,
				 &video->weighted_prediction_mem[i]))
				IMG_DBG_ASSERT("Allocation failed (A)" == NULL);
		}
	} else {
		video->weighted_prediction = FALSE;
	}

	video->weighted_bi_pred = video_params->vp_weighted_implicit_bi_pred;

	video->coded_skipped_index = video_params->coded_skipped_index;
	video->inter_intra_index = video_params->inter_intra_index;

	/*
	 * patch video parameters is the user has specified a profile
	 * calculate the number of macroblocks per second
	 */
	video->mbps = width_in_mbs * frame_height_in_mbs * video->rc_params.frame_rate;

	patch_hw_profile(video_params, video);

	enc->auto_expand_pipes = video_params->auto_expand_pipes;

	/* As ui32Vp8RefStructMode is not in use the worst case
	 * would have to be taken and hence 5 pic nodes
	 */
	video->pic_nodes = (rc_params->hierarchical ? MAX_REF_B_LEVELS : 0) +
		video_params->ref_spacing + 4;
	video->mv_stores = (video->pic_nodes * 2);

	/* We're using a common MACRO here so we can guarantee the same calculation
	 * when managing buffers either from host or within drivers
	 */
	video->coded_package_max_num = CALC_NUM_CODED_PACKAGES_ENCODE
					(video_params->slice_level,
					 video_params->slices_per_picture, pipes_to_use,
					 video->is_interlaced);

	alloc_size = MVEA_ABOVE_PARAM_REGION_SIZE * (ALIGN_64(width_in_mbs));

	for (index = 0; index < (int)max_cores; index++) {
		if (str_ctx->vxe_ctx->above_mb_params_sgt[index].sgl) {
			video->above_params[index].buf_size = alloc_size;

			topaz_mmu_stream_map_ext_sg
			(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID,
			 &str_ctx->vxe_ctx->above_mb_params_sgt[index],
			 video->above_params[index].buf_size,
			 64, (enum sys_emem_attrib)0, video->above_params[index].cpu_virt,
			 &video->above_params[index],
			 &video->above_params[index].buff_id);
		} else {
			if (topaz_mmu_stream_alloc
				(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
				 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
							SYS_MEMATTRIB_WRITECOMBINE),
				 ALIGN_64(alloc_size), 64, &video->above_params[index]))
				IMG_DBG_ASSERT("Allocation failed (C)" == NULL);
		}
	}

	alloc_size = MVEA_MV_PARAM_REGION_SIZE * ALIGN_4(width_in_mbs) * frame_height_in_mbs;

	for (index = 0; index < video->pic_nodes; index++) {
		if (topaz_mmu_stream_alloc
			(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
			 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
						SYS_MEMATTRIB_WRITECOMBINE),
			 alloc_size, 64, &video->colocated[index]))
			IMG_DBG_ASSERT("Allocation failed (D)" == NULL);
	}

	alloc_size = (ALIGN_64(video->width)) * (ALIGN_64(video->frame_height)) * 3 / 2;

	for (index = 0; index < video->pic_nodes; index++) {
		void *data;

		if (topaz_mmu_stream_alloc
			(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
			 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
						SYS_MEMATTRIB_WRITECOMBINE),
			 alloc_size, 256, &video->recon_pictures[index]))
			IMG_DBG_ASSERT("Allocation failed (E)" == NULL);

		data = video->recon_pictures[index].cpu_virt;
		memset(data, 0, alloc_size);

		topaz_update_device_mem(str_ctx->vxe_ctx, &video->recon_pictures[index]);
	}

	video->patched_recon_buffer = NULL;

	alloc_size = MVEA_MV_PARAM_REGION_SIZE * ALIGN_4(width_in_mbs) * frame_height_in_mbs;
	for (i = 0; i < video->mv_stores; i++) {
		if (topaz_mmu_stream_alloc
			(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
			 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
						SYS_MEMATTRIB_WRITECOMBINE),
			 alloc_size, 64, &video->mv[i]))
			IMG_DBG_ASSERT("Allocation failed (F)" == NULL);
		topaz_update_device_mem(str_ctx->vxe_ctx, &video->mv[i]);
	}

	if (video->enable_mvc) {
		for (i = 0; i < 2; i++) {
			if (topaz_mmu_stream_alloc
				(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
				 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
							SYS_MEMATTRIB_WRITECOMBINE),
				 alloc_size, 64, &video->inter_view_mv[i]))
				IMG_DBG_ASSERT("Allocation failed (G)" == NULL);
		}
	}

	/* memory for encoder context */
	if (topaz_mmu_stream_alloc
		(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
		 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
					SYS_MEMATTRIB_WRITECOMBINE),
		 ALIGN_64(MTX_CONTEXT_SIZE), 64, &video->mtx_enc_ctx_mem))
		IMG_DBG_ASSERT("Allocation failed (H)" == NULL);

	video->no_sequence_headers = video_params->no_sequence_headers;
	video->auto_encode = video_params->auto_encode;
	video->slice_level = video_params->slice_level;
	video->coded_header_per_slice = video_params->coded_header_per_slice;

	/* partially coded headers supplied to HW */
	if (topaz_mmu_stream_alloc
		(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
		 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
					SYS_MEMATTRIB_WRITECOMBINE),
		 MAX_HEADERSIZEBYTES, 64, &video->seq_header_mem))
		IMG_DBG_ASSERT("Allocation failed (I)\n" == NULL);

	/* partially coded subset sequence parameter headers supplied to HW */
	if (video->enable_mvc) {
		if (topaz_mmu_stream_alloc
			(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
			 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
						SYS_MEMATTRIB_WRITECOMBINE),
			 MAX_HEADERSIZEBYTES, 64, &video->subset_seq_header_mem))
			IMG_DBG_ASSERT("Allocation failed (J)" == NULL);
	}

	if (topaz_mmu_stream_alloc
		(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
		 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
					SYS_MEMATTRIB_WRITECOMBINE),
		 MAX_BFRAMES * MV_ROW_STRIDE, 64, &video->mv_settings_btable))

		IMG_DBG_ASSERT("Allocation failed (K)" == NULL);

	if (video->rc_params.hierarchical) {
		if (topaz_mmu_stream_alloc
			(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
			 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
						SYS_MEMATTRIB_WRITECOMBINE),
			 MAX_BFRAMES * sizeof(struct img_mv_settings), 64,
			 &video->mv_settings_hierarchical))
			IMG_DBG_ASSERT("Allocation failed (L)" == NULL);
	} else {
		video->mv_settings_hierarchical.cpu_virt = NULL;
	}

	video->insert_hrd_params = video_params->insert_hrd_params;
	if (video_params->insert_hrd_params) {
		alloc_size      = 64;
		if (topaz_mmu_stream_alloc
			(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
			 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
						SYS_MEMATTRIB_WRITECOMBINE),
			 alloc_size, 64, &video->aud_header_mem))
			IMG_DBG_ASSERT("Allocation failed (M)" == NULL);

		if (topaz_mmu_stream_alloc
			(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
			 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
						SYS_MEMATTRIB_WRITECOMBINE),
			 alloc_size, 64, &video->sei_buffering_period_header_mem))
			IMG_DBG_ASSERT("Allocation failed (N)" == NULL);

		if (topaz_mmu_stream_alloc
			(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
			 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
						SYS_MEMATTRIB_WRITECOMBINE),
			 alloc_size, 64, &video->sei_picture_timing_header_mem))
			IMG_DBG_ASSERT("Allocation failed (O)" == NULL);
	}

	for (index = 0; index < ARRAY_SIZE(video->pichdr_template_mem); index++) {
		if (topaz_mmu_stream_alloc
			(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
			 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
						SYS_MEMATTRIB_WRITECOMBINE),
			 MAX_HEADERSIZEBYTES, 64, &video->pichdr_template_mem[index]))
			IMG_DBG_ASSERT("Allocation failed (P)" == NULL);
	}

	for (index = 0; index < ARRAY_SIZE(video->slice_params_template_mem); index++) {
		if (topaz_mmu_stream_alloc
			(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
			 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
						SYS_MEMATTRIB_WRITECOMBINE),
			 ALIGN_64(sizeof(struct slice_params)), 64,
			 &video->slice_params_template_mem[index]))
			IMG_DBG_ASSERT("Allocation failed (Q)" == NULL);
	}

	for (index = 0; index < video->slots_in_use; index++) {
		if (topaz_mmu_stream_alloc
			(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
			 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
						SYS_MEMATTRIB_WRITECOMBINE),
			 ALIGN_64(sizeof(struct mtx_header_params)), 64,
			 &video->ltref_header[index]))
			IMG_DBG_ASSERT("Allocation failed (R)" == NULL);
	}

	if (topaz_mmu_stream_alloc
		(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
		 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED | SYS_MEMATTRIB_WRITECOMBINE),
		 ALIGN_64(4), 64, &video->src_phys_addr))
		IMG_DBG_ASSERT("Allocation failed (S)" == NULL);

	for (index = 0; index < video->slots_in_use; index++) {
		if (topaz_mmu_stream_alloc
			(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
			 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
						SYS_MEMATTRIB_WRITECOMBINE),
			 (1 + MAX_SLICESPERPIC * 2 + 15) & ~15, 64,
			 &video->slice_map[index].mem_info) != IMG_SUCCESS)
			IMG_DBG_ASSERT("Allocation failed (T)" == NULL);
	}

	if (topaz_mmu_stream_alloc
		(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
		 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED | SYS_MEMATTRIB_WRITECOMBINE),
		 ALIGN_64(sizeof(unsigned short) * MAX_GOP_SIZE), 64, &video->flat_gop_struct))
		IMG_DBG_ASSERT("Allocation failed (U)" == NULL);

	if (video->rc_params.hierarchical) {
		if (topaz_mmu_stream_alloc
			(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
			 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
						SYS_MEMATTRIB_WRITECOMBINE),
			 ALIGN_64(sizeof(unsigned short) * MAX_GOP_SIZE), 64,
			 &video->hierar_gop_struct))
			IMG_DBG_ASSERT("Allocation failed (V)" == NULL);
	}

	if (video->custom_scaling) {
		for (index = 0; index < 2; index++) {
			if (topaz_mmu_stream_alloc
				(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
				 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
					 SYS_MEMATTRIB_WRITECOMBINE),
				 ALIGN_64(QUANT_LISTS_SIZE), 64, &video->custom_quant[index]))
				IMG_DBG_ASSERT("Allocation failed (W)" == NULL);

			if (topaz_mmu_stream_alloc
				(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
				 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
					 SYS_MEMATTRIB_WRITECOMBINE),
				 192, 64, &video->custom_quant_regs4x4_sp[index]))
				IMG_DBG_ASSERT("Allocation failed (X)" == NULL);

			if (topaz_mmu_stream_alloc
				(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
				 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
							SYS_MEMATTRIB_WRITECOMBINE),
				 128 * CUSTOM_QUANT_PARAMSIZE_8x8, 64,
				 &video->custom_quant_regs8x8_sp[index]))
				IMG_DBG_ASSERT("Allocation failed (Y)" == NULL);

			if (topaz_mmu_stream_alloc
				(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
				 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
				 SYS_MEMATTRIB_WRITECOMBINE),
				 128, 64, &video->custom_quant_regs4x4_q[index]))
				IMG_DBG_ASSERT("Allocation failed (Z)" == NULL);

			if (topaz_mmu_stream_alloc
				(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
				 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
				 SYS_MEMATTRIB_WRITECOMBINE),
				 64 * CUSTOM_QUANT_PARAMSIZE_8x8, 64,
				 &video->custom_quant_regs8x8_q[index]))
				IMG_DBG_ASSERT("Allocation failed (0)" == NULL);
		}
		video->custom_quant_slot = 0;
	}

	/* Allocate device memory for storing feedback information for all "5" slots */
	if (video->enable_sel_stats_flags & ESF_FIRST_STAGE_STATS) {
		for (index = 0; index < video->slots_in_use; index++) {
			unsigned int row_size =
				ALIGN_64(width_in_mbs * sizeof(struct img_first_stage_mb_params));

			/* Allocate memory padding size of each row to be multiple of 64-bytes */
			if (topaz_mmu_stream_alloc
				(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
				 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
				 SYS_MEMATTRIB_WRITECOMBINE),
				 frame_height_in_mbs * row_size, 64,
				 &video->firstpass_out_param_buf[index].mem_info))
				IMG_DBG_ASSERT("Allocation failed (1)" == NULL);

			video->firstpass_out_param_buf[index].lock              = BUFFER_FREE;
			video->firstpass_out_param_buf[index].bytes_written     = 0;
			video->firstpass_out_param_buf[index].size              =
				frame_height_in_mbs * row_size;
		}
	} else {
		/* Set buffer pointers to NULL */
		for (index = 0; index < video->slots_in_use; index++) {
			video->firstpass_out_param_buf[index].mem_info.cpu_virt = NULL;
			video->firstpass_out_param_buf[index].lock              = BUFFER_FREE;
			video->firstpass_out_param_buf[index].bytes_written     = 0;
			video->firstpass_out_param_buf[index].size              = 0;
		}
	}

	/* Allocate device memory for storing feedback information for all "5" slots */
	if (video->enable_sel_stats_flags & ESF_MP_BEST_MB_DECISION_STATS ||
	    video->enable_sel_stats_flags & ESF_MP_BEST_MOTION_VECTOR_STATS) {
		for (index = 0; index < video->slots_in_use; index++) {
			unsigned int best_multipass_size = frame_height_in_mbs *
				//From TRM (4.5.2)
				(((5 * width_in_mbs) + 3) >> 2) * 64;

			/* Allocate memory padding size of each row to be multiple of 64-bytes */
			if (topaz_mmu_stream_alloc
				(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
				 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
				 SYS_MEMATTRIB_WRITECOMBINE),
				 best_multipass_size, 64,
				 &video->firstpass_out_best_multipass_param_buf[index].mem_info))
				IMG_DBG_ASSERT("Allocation failed (2)" == NULL);

			video->firstpass_out_best_multipass_param_buf[index].lock               =
				BUFFER_FREE;
			video->firstpass_out_best_multipass_param_buf[index].bytes_written      = 0;
			video->firstpass_out_best_multipass_param_buf[index].size               =
				best_multipass_size;
		}
	} else {
		/* Set buffer pointers to NULL */
		for (index = 0; index < video->slots_in_use; index++) {
			video->firstpass_out_best_multipass_param_buf[index].mem_info.cpu_virt  =
				NULL;
			video->firstpass_out_best_multipass_param_buf[index].lock               =
				BUFFER_FREE;
			video->firstpass_out_best_multipass_param_buf[index].bytes_written      = 0;
			video->firstpass_out_best_multipass_param_buf[index].size               = 0;
		}
	}

	if (video->enable_inp_ctrl) {
		for (index = 0; index < video->slots_in_use; index++) {
			alloc_size =  frame_height_in_mbs *  width_in_mbs * 2;

			/*
			 * Allocate memory for worst case slice structure
			 * i.e. assume number-of-slices == number-of-rows
			 */
			if (topaz_mmu_stream_alloc
				(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, 1,
				 (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED |
				 SYS_MEMATTRIB_WRITECOMBINE), alloc_size + 64, 64,
				 &video->mb_ctrl_in_params_buf[index].mem_info))
				IMG_DBG_ASSERT("Allocation failed (3)" == NULL);

			video->mb_ctrl_in_params_buf[index].lock = BUFFER_FREE;
			video->mb_ctrl_in_params_buf[index].bytes_written = 0;
			video->mb_ctrl_in_params_buf[index].size = alloc_size;
		}
	} else {
		for (index = 0; index < video->slots_in_use; index++) {
			video->mb_ctrl_in_params_buf[index].mem_info.cpu_virt   = NULL;
			video->mb_ctrl_in_params_buf[index].lock        = BUFFER_FREE;
			video->mb_ctrl_in_params_buf[index].bytes_written = 0;
			video->mb_ctrl_in_params_buf[index].size        = 0;
		}
	}

	for (index = 0; index < video->slots_in_use; index++)
		video->source_slot_buff[index] = NULL;

	/* Allocate coded package */
	topaz_get_context_coded_buffer_size(enc, rc_params, &coded_buffer_size);

	video->coded_buffer_max_size = coded_buffer_size;

	for (i = 0; i < video->coded_package_max_num; i++) {
		if (topaz_allocate_coded_package(str_ctx, coded_buffer_size,
						 &video->coded_package[i]) != IMG_SUCCESS)
			IMG_DBG_ASSERT("Coded package Allocation failed\n" == NULL);
	}

	video->encode_sent = 0;

	topaz_video_prepare_templates(str_ctx, video_params->f_code,
				      video_params->fine_y_search_size);

	enc->video->max_chunks = video_params->max_chunks;
	enc->video->chunks_per_mb = video_params->chunks_per_mb;
	enc->video->priority_chunks = video_params->priority_chunks;

	return topaz_video_create_mtx_context(str_ctx, video_params);
}

unsigned char topaz_validate_params(struct img_video_params *video_params,
				    struct img_rc_params *rc_params)
{
	unsigned char modified = FALSE;
	unsigned int required_core_des1 = 0;
	unsigned int core_des1 = topazdd_get_core_des1();

	if (video_params) {
		/* Validate video params */
		if (video_params->standard == IMG_STANDARD_H264) {
			if (video_params->is_interlaced) {
				if ((core_des1 &
				    F_ENCODE
				    (1, TOPAZHP_TOP_CR_TOPAZHP_H264_INTERLACED_SUPPORTED)) == 0) {
					video_params->is_interlaced = FALSE;

					if (!video_params->is_interleaved) {
					/* Non-interleaved source.
					 * Encode field pictures as frames.
					 */
						video_params->frame_height >>= 1;
						video_params->buffer_height >>= 1;
						video_params->source_frame_height >>= 1;
					} else {
					/* Interleaved source. Unite fields into single picture. */
						video_params->is_interleaved = FALSE;
					}

					video_params->bottom_field_first = FALSE;
					modified = TRUE;
				} else {
					required_core_des1 |= F_ENCODE(1,
					TOPAZHP_TOP_CR_TOPAZHP_H264_INTERLACED_SUPPORTED);
				}
			}

			if (video_params->h264_8x8) {
				if ((core_des1 &
				F_ENCODE(1, TOPAZHP_TOP_CR_TOPAZHP_H264_8X8_TRANSFORM_SUPPORTED)) ==
					0) {
					video_params->h264_8x8 = FALSE;
					modified = TRUE;
				} else {
					required_core_des1 |= F_ENCODE(1,
					TOPAZHP_TOP_CR_TOPAZHP_H264_8X8_TRANSFORM_SUPPORTED);
				}
			}

			if (video_params->cabac_enabled) {
				if ((core_des1 &
				    F_ENCODE
				    (1, TOPAZHP_TOP_CR_TOPAZHP_H264_CABAC_SUPPORTED)) == 0) {
					video_params->cabac_enabled = FALSE;
					modified = TRUE;
				} else {
					required_core_des1 |= F_ENCODE(1,
					TOPAZHP_TOP_CR_TOPAZHP_H264_CABAC_SUPPORTED);
				}
			}

			if (!video_params->enc_features.disable_bframes) {
				if ((core_des1 &
				    F_ENCODE
				    (1, TOPAZHP_TOP_CR_TOPAZHP_H264_B_PIC_SUPPORTED)) == 0) {
					video_params->enc_features.disable_bframes = FALSE;
					modified = TRUE;
				}
			}

			if (video_params->enable_sel_stats_flags) {
				if ((core_des1 &
				    F_ENCODE
				    (1, TOPAZHP_TOP_CR_TOPAZHP_H264_MULTIPASS_SUPPORTED)) == 0) {
					video_params->enable_sel_stats_flags = 0;
					modified = TRUE;
				} else {
					required_core_des1 |= F_ENCODE(1,
					TOPAZHP_TOP_CR_TOPAZHP_H264_MULTIPASS_SUPPORTED);
				}
			}

			if (video_params->use_default_scaling_list) {
				if ((core_des1 &
				    F_ENCODE
				    (1, TOPAZHP_TOP_CR_TOPAZHP_H264_DEFAULT_TABLES_SUPPORTED)) ==
					0) {
					video_params->use_default_scaling_list = FALSE;
					modified = TRUE;
				} else {
					required_core_des1 |= F_ENCODE(1,
					TOPAZHP_TOP_CR_TOPAZHP_H264_DEFAULT_TABLES_SUPPORTED);
				}
			}

			if (video_params->use_custom_scaling_lists) {
				if ((core_des1 &
				    F_ENCODE
				    (1, TOPAZHP_TOP_CR_TOPAZHP_H264_CUSTOM_QUANT_SUPPORTED)) == 0) {
					video_params->use_custom_scaling_lists = 0;
					modified = TRUE;
				} else {
					required_core_des1 |= F_ENCODE(1,
					TOPAZHP_TOP_CR_TOPAZHP_H264_CUSTOM_QUANT_SUPPORTED);
				}
			}

			if ((video_params->weighted_prediction ||
			     video_params->vp_weighted_implicit_bi_pred)) {
				if ((core_des1 &
				    F_ENCODE
				    (1, TOPAZHP_TOP_CR_TOPAZHP_H264_WEIGHTED_PRED_SUPPORTED)) ==
				     0) {
					video_params->weighted_prediction = FALSE;
					video_params->vp_weighted_implicit_bi_pred = 0;
					modified = TRUE;
				} else {
					required_core_des1 |= F_ENCODE(1,
					TOPAZHP_TOP_CR_TOPAZHP_H264_WEIGHTED_PRED_SUPPORTED);
				}
			}

			if (video_params->multi_reference_p || video_params->enable_mvc) {
				if ((core_des1 &
				    F_ENCODE
				    (1, TOPAZHP_TOP_CR_TOPAZHP_H264_2_REF_ON_P_PIC_SUPPORTED)) ==
					0) {
					video_params->multi_reference_p = FALSE;
					video_params->enable_mvc = FALSE;
					modified = TRUE;
				} else {
					required_core_des1 |= F_ENCODE(1,
					TOPAZHP_TOP_CR_TOPAZHP_H264_2_REF_ON_P_PIC_SUPPORTED);
				}
			}

			if (video_params->spatial_direct) {
				if ((core_des1 &
					F_ENCODE
					(1,
					 TOPAZHP_TOP_CR_TOPAZHP_H264_SPATIAL_DIRECT_SUPPORTED)) ==
						0) {
					video_params->spatial_direct = FALSE;
					modified = TRUE;
				} else {
					required_core_des1 |= F_ENCODE(1,
					TOPAZHP_TOP_CR_TOPAZHP_H264_SPATIAL_DIRECT_SUPPORTED);
				}
			}

			if (video_params->enable_lossless) {
				if ((core_des1 &
					F_ENCODE
					(1, TOPAZHP_TOP_CR_TOPAZHP_H264_LOSSLESS_SUPPORTED)) == 0) {
					video_params->enable_lossless = FALSE;
					modified = TRUE;
				} else {
					required_core_des1 |= F_ENCODE(1,
						TOPAZHP_TOP_CR_TOPAZHP_H264_LOSSLESS_SUPPORTED);
				}
			}
		}

		if (video_params->enable_scaler) {
			if ((core_des1 &
				F_ENCODE(1, TOPAZHP_TOP_CR_TOPAZHP_SCALER_SUPPORTED)) == 0) {
				video_params->enable_scaler = FALSE;
				modified = TRUE;
			} else {
				required_core_des1 |= F_ENCODE(1,
						TOPAZHP_TOP_CR_TOPAZHP_SCALER_SUPPORTED);
			}
		}

		if (rc_params) {
			/* Validate RC params */
			if (video_params->standard == IMG_STANDARD_H264) {
				if (rc_params->bframes) {
					if ((core_des1 &
						F_ENCODE
						(1, TOPAZHP_TOP_CR_TOPAZHP_H264_B_PIC_SUPPORTED)) ==
						0) {
						rc_params->bframes = 0;
						rc_params->hierarchical = FALSE;
						modified = TRUE;
					} else {
						required_core_des1 |= F_ENCODE(1,
						TOPAZHP_TOP_CR_TOPAZHP_H264_B_PIC_SUPPORTED);
					}
				}

				if (rc_params->hierarchical && rc_params->bframes > 1) {
					if ((core_des1 &
					    F_ENCODE
					(1, TOPAZHP_TOP_CR_TOPAZHP_H264_SPATIAL_DIRECT_SUPPORTED))
							== 0) {
						rc_params->hierarchical = FALSE;
						modified = TRUE;
					} else {
						required_core_des1 |= F_ENCODE(1,
					TOPAZHP_TOP_CR_TOPAZHP_H264_SPATIAL_DIRECT_SUPPORTED);
					}
				}
			}
		}
	}

	return modified;
}

/*
 * Creat an encoder context
 */
int topaz_stream_create(void *vxe_ctx, struct img_video_params *video_params,
			unsigned char base_pipe, unsigned char pipes_to_use,
			struct img_rc_params *rc_params, void **topaz_str_context)
{
	struct img_enc_context *enc;
	struct topaz_stream_context *str_ctx;

	if (!is_topaz_core_initialized)
		return IMG_ERROR_NOT_INITIALISED;

	str_ctx = kzalloc(sizeof(*str_ctx), GFP_KERNEL);
	if (!str_ctx)
		return IMG_ERROR_OUT_OF_MEMORY;

	enc = kzalloc(sizeof(*enc), GFP_KERNEL);
	if (!enc) {
		kfree(str_ctx);
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	*topaz_str_context = str_ctx;
	str_ctx->enc_ctx = enc;
	str_ctx->core_ctx = global_topaz_core_context;
	str_ctx->vxe_ctx = (struct vxe_enc_ctx *)vxe_ctx;

	enc->core_rev = topazdd_get_core_rev();
	enc->sync_first_pass = true;

	enc->requested_base_pipe = base_pipe;
	enc->base_pipe = base_pipe;
	enc->requested_pipes_to_use = pipes_to_use;
	enc->pipes_to_use = pipes_to_use;

	topaz_validate_params(video_params, rc_params);

	switch (video_params->standard) {
	case IMG_STANDARD_H264:
		if (video_params->enable_mvc) {
			switch (rc_params->rc_mode) {
			case IMG_RCMODE_NONE:
				enc->codec = IMG_CODEC_H264MVC_NO_RC;
				break;
			case IMG_RCMODE_CBR:
				enc->codec = IMG_CODEC_H264MVC_CBR;
				break;
			case IMG_RCMODE_VBR:
				enc->codec = IMG_CODEC_H264MVC_VBR;
				break;
			case IMG_RCMODE_ERC:
				enc->codec = IMG_CODEC_H264MVC_ERC;
				break;
			case IMG_RCMODE_VCM:
				IMG_DBG_ASSERT("VCM mode is not supported for MVC" == NULL);
				break;
			default:
				break;
			}
		} else {
			switch (rc_params->rc_mode) {
			case IMG_RCMODE_NONE:
				enc->codec = IMG_CODEC_H264_NO_RC;
				break;
			case IMG_RCMODE_CBR:
				enc->codec = IMG_CODEC_H264_CBR;
				break;
			case IMG_RCMODE_VBR:
				enc->codec = IMG_CODEC_H264_VBR;
				break;
			case IMG_RCMODE_VCM:
				enc->codec = IMG_CODEC_H264_VCM;
				break;
			case IMG_RCMODE_ERC:
				enc->codec = IMG_CODEC_H264_ERC;
				break;
			default:
				break;
			}
		}
		break;
	default:
		IMG_DBG_ASSERT("Only H264 encode is supported" == NULL);
	}

	/* initialise video context structure */
	return (topaz_video_create_context(str_ctx, video_params, rc_params));
}

/*
 * Sends a command to the specified core.
 * The function returns a writeback value.  This is a unique value that will be
 * written back by the target core after it completes its command.
 */
unsigned int topaz_insert_command(struct img_enc_context *enc_ctx,
				  enum mtx_cmd_id cmd_id, unsigned int data)
{
	unsigned int writeback_val;

	if (enc_ctx->debug_settings &&
	    enc_ctx->debug_settings->serialized_communication_mode ==
	     VXE_SERIALIZED_MODE_SERIAL)
		/* in serial mode do not use the priority bit */
		cmd_id &= ~MTX_CMDID_PRIORITY;

	topazdd_send_msg(enc_ctx->video->dd_str_ctx, cmd_id, data, NULL, &writeback_val);

	return writeback_val;
}

/*
 * Sends a command to the specified core.
 */
unsigned int topaz_insert_command_with_sync(struct img_enc_context *enc_ctx,
					    enum mtx_cmd_id cmd_id, unsigned int data)
{
	int ret;

	if (enc_ctx->debug_settings &&
	    enc_ctx->debug_settings->serialized_communication_mode ==
	     VXE_SERIALIZED_MODE_SERIAL)
		/* in serial mode do not use the priority bit */
		cmd_id &= ~MTX_CMDID_PRIORITY;

	ret = topazdd_send_msg_with_sync(enc_ctx->video->dd_str_ctx, cmd_id, data, NULL);

	return ret;
}

/*
 * Sends a command to the specified core.
 * The data specified in psCommandData will be read via DMA by the MTX,
 * so this memory must remain in scope for the duration of the execution
 * of the command.
 * The function returns a writeback value.  This is a unique value that will be
 * written back by the target core after it completes its command.
 */
unsigned int topaz_insert_mem_command(struct img_enc_context *enc_ctx,
				      enum mtx_cmd_id cmd_id,
				      unsigned int data,
				      struct vidio_ddbufinfo *command_data)
{
	unsigned int writeback_val;

	/* Priority bit is not supported for MEM commands */
	cmd_id &= ~MTX_CMDID_PRIORITY;

	topazdd_send_msg(enc_ctx->video->dd_str_ctx, cmd_id, data, command_data, &writeback_val);

	return writeback_val;
}

/*
 * Sends a command to the specified core.
 * The data specified in psCommandData will be read via DMA by the MTX,
 * so this memory must remain in scope for the duration of the execution
 * of the command.
 */
unsigned int topaz_insert_mem_command_with_sync(struct img_enc_context *enc_ctx,
						enum mtx_cmd_id cmd_id,
						unsigned int data,
						struct vidio_ddbufinfo *command_data)
{
	int ret;

	/* Priority bit is not supported for MEM commands */
	cmd_id &= ~MTX_CMDID_PRIORITY;

	ret = topazdd_send_msg_with_sync(enc_ctx->video->dd_str_ctx, cmd_id,
					 data, command_data);
	return ret;
}

/*
 * Send the Access Unit Delimiter to the stream
 */
static int topaz_send_aud_header(struct img_enc_context *enc)
{
	if (enc->video->aborted)
		return IMG_ERROR_UNDEFINED;

	/* must use unique writeback word */
	topaz_insert_mem_command(enc, MTX_CMDID_DO_HEADER, 0,
				 enc->video->aud_header_mem.cpu_virt);

	return IMG_SUCCESS;
}

/*
 * Transmit the picture headerts to MTX
 */
static int topaz_send_picture_headers(struct img_enc_context *enc)
{
	/* send Seqence headers only for IDR (I-frames) and only once in the beginning */
	struct img_video_context *video = enc->video;

	/* SEI_INSERTION */
	if (video->insert_hrd_params) {
		/* Access unit delimiter */
		if (!video->enable_mvc || (video->enable_mvc && video->mvc_view_idx == 0))
			/* in case of MVC, both views are a single access unit.
			 *  delimiter should be inserted by view 0 only.
			 */
			topaz_send_aud_header(enc);
	}

	if (video->insert_seq_header && !video->no_sequence_headers) {
		switch (video->standard) {
		case IMG_STANDARD_H264:
			IMG_DBG_ASSERT("SPS and PPS will be send from firmware." != NULL);
			break;
		default:
			IMG_DBG_ASSERT("only H264 encode is supported." == NULL);
			break;
		}
	}

	return IMG_SUCCESS;
}

/*
 * Encode a frame
 */
int topaz_encode_frame(void *topaz_str_ctx)
{
	struct img_enc_context *enc;
	struct img_video_context *video;
	struct topaz_stream_context *str_ctx;
	/* If line counter is enabled, we add one more bit in the command data
	 * to inform the firmware context whether it should proceed
	 */
	unsigned int encode_cmd_data;

	if (!topaz_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;

	enc = str_ctx->enc_ctx;
	video = enc->video;

	if (video->aborted)
		return IMG_ERROR_UNEXPECTED_STATE;

	video->insert_seq_header = (video->encode_sent == 0);

	topaz_send_picture_headers(enc);

	encode_cmd_data = F_ENCODE(1, MTX_MSG_ENCODE_CODED_INTERRUPT);

	if (video->line_counter)
		/* Set bit 20 to 1 to inform FW that we are using the line counter feature */
		encode_cmd_data |= F_ENCODE(1, MTX_MSG_ENCODE_USE_LINE_COUNTER);

	topaz_insert_command(enc, (enum mtx_cmd_id)
				       (MTX_CMDID_ENCODE_FRAME | MTX_CMDID_WB_INTERRUPT),
		encode_cmd_data);

	video->encode_pic_processing++;
	video->encode_sent++;

	return IMG_SUCCESS;
}

int topaz_get_pipe_usage(unsigned char pipe, unsigned char *ctx_id)
{
	IMG_DBG_ASSERT(pipe < TOPAZHP_MAX_NUM_PIPES);

	if (pipe >= TOPAZHP_MAX_NUM_PIPES)
		return 0;

	return global_pipe_usage[pipe];
}

void topaz_set_pipe_usage(unsigned char pipe, unsigned char val)
{
	IMG_DBG_ASSERT(pipe < TOPAZHP_MAX_NUM_PIPES);

	if (pipe < TOPAZHP_MAX_NUM_PIPES)
		global_pipe_usage[pipe] = val;
}

/*
 * Set the mtx context to the one implicit in the encoder context
 */
static int topaz_video_setup_mtx_context(struct img_enc_context *enc)
{
	struct img_video_context *video_context;
	unsigned char index;

	video_context = enc->video;

	for (index = 0; index < enc->pipes_to_use; index++)
		topaz_set_pipe_usage(enc->base_pipe + index, enc->ctx_num);

	if (topaz_insert_mem_command_with_sync(enc, (enum mtx_cmd_id)
					       (MTX_CMDID_SETVIDEO | MTX_CMDID_WB_INTERRUPT),
		enc->base_pipe, &video_context->mtx_enc_ctx_mem)) {
		pr_err("topaz mtx context setup command failed\n");
		return IMG_ERROR_UNDEFINED;
	}

	video_context->aborted = FALSE;

	return IMG_SUCCESS;
}

/*
 * Load the encoder and MTX context
 */
int topaz_load_context(void *topaz_str_ctx)
{
	struct topaz_stream_context *str_ctx;
	struct img_enc_context *enc;

	if (!topaz_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;
	enc = str_ctx->enc_ctx;

	enc->video->vid_ctx_num = 0;

	enc->ctx_num++;

	return topaz_video_setup_mtx_context(enc);
}

/*
 * Store the encoder and MTX context
 */
int topaz_store_context(void *topaz_str_ctx)
{
	struct img_enc_context *enc;
	struct topaz_stream_context *str_ctx;

	if (!topaz_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;
	enc = str_ctx->enc_ctx;

	/* Update Globals */
	if (enc->codec != IMG_CODEC_NONE && enc->codec != IMG_CODEC_JPEG) {
		struct img_video_context *video_context;

		video_context = enc->video;

		if (!topaz_insert_mem_command_with_sync(enc, (enum mtx_cmd_id)
							(MTX_CMDID_GETVIDEO |
							 MTX_CMDID_WB_INTERRUPT),
			enc->base_pipe, &video_context->mtx_enc_ctx_mem)) {
			pr_err("MTX message for GETVIDEO failed\n");
			return IMG_ERROR_UNDEFINED;
		}
	}

	return IMG_SUCCESS;
}

/*
 * Flush video stream
 */
int topaz_flush_stream(void *topaz_str_ctx, unsigned int frame_cnt)
{
	struct topaz_stream_context *str_ctx;
	struct img_enc_context *enc;
	int index;

	if (!topaz_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;

	enc = str_ctx->enc_ctx;

	if (enc->video->aborted)
		return IMG_ERROR_UNDEFINED;

	/* flush the internal queues */
	/* Check source slots */
	for (index = 0; index < enc->video->slots_in_use; index++) {
		if (enc->video->source_slot_buff[index]) {
			/* Found a valid src_frame, so signal callback for the same. */
			global_topaz_core_context->vxe_str_processed_cb(str_ctx->vxe_ctx,
				VXE_CB_SRC_FRAME_RELEASE,
				(void *)(enc->video->source_slot_buff[index]),
				0, 0, 0);
			enc->video->source_slot_buff[index] = NULL;
		}
	}

	/* Check coded package slots */
	for (index = 0; index < enc->video->coded_package_max_num; index++) {
		if (enc->video->coded_package[index]->busy) {
			/* Found a valid coded package, so, signal callback for the same */
			global_topaz_core_context->vxe_str_processed_cb(str_ctx->vxe_ctx,
				VXE_CB_CODED_BUFF_READY,
				(void *)(enc->video->coded_package[index]->coded_buffer[0]),
				0, 0, 0);
			enc->video->coded_package[index]->busy = FALSE;
		}
	}

	return IMG_SUCCESS;
}

/*
 * Destroy the Video Encoder context
 */
static int topaz_video_destroy_context(struct topaz_stream_context *str_ctx)
{
	struct img_enc_context *enc;
	struct img_video_context *video;
	int i;
	unsigned int max_cores;

	max_cores = topazdd_get_num_pipes(str_ctx->core_ctx->dev_handle);
	enc = str_ctx->enc_ctx;
	video = enc->video;

	for (i = 0; i < enc->pipes_to_use; i++)
		if (topaz_get_pipe_usage(enc->base_pipe + i, NULL) == enc->ctx_num)
			topaz_set_pipe_usage(enc->base_pipe + i, 0);

	if (video->standard == IMG_STANDARD_H264 && video->weighted_prediction) {
		for (i = 0; i < video->slots_in_use; i++) {
			if (topaz_mmu_stream_free(str_ctx->mmu_ctx,
						  &video->weighted_prediction_mem[i]))
				IMG_DBG_ASSERT("Free failed" == NULL);
		}
	}

	for (i = 0; i < video->coded_package_max_num; i++) {
		if (topaz_mmu_stream_free
			(str_ctx->mmu_ctx,
			 &video->coded_package[i]->mtx_info.code_package_fw_buffer->mem_info))
			IMG_DBG_ASSERT("Free failed" == NULL);

		kfree(video->coded_package[i]->mtx_info.code_package_fw_buffer);
		video->coded_package[i]->mtx_info.code_package_fw_buffer = NULL;

		if (topaz_mmu_stream_free(str_ctx->mmu_ctx,
					  &video->coded_package[i]->header_buffer->mem_info))
			IMG_DBG_ASSERT("Free failed" == NULL);

		kfree(video->coded_package[i]->header_buffer);
		video->coded_package[i]->header_buffer = NULL;

		kfree(video->coded_package[i]);
		video->coded_package[i] = NULL;
	}

	if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->flat_gop_struct))
		IMG_DBG_ASSERT("Free failed" == NULL);

	if (video->rc_params.hierarchical)
		if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->hierar_gop_struct))
			IMG_DBG_ASSERT("Free failed" == NULL);

	for (i = 0; i < video->slots_in_use; i++) {
		if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->slice_map[i].mem_info))
			IMG_DBG_ASSERT("slice map free failed" == NULL);
	}

	for (i = 0; i < (int)max_cores; i++) {
		if (str_ctx->vxe_ctx->above_mb_params_sgt[i].sgl) {
			topaz_mmu_stream_free_sg(str_ctx->mmu_ctx, &video->above_params[i]);
		} else {
			if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->above_params[i]))
				IMG_DBG_ASSERT("Free failed" == NULL);
		}
	}

	for (i = 0; i < video->pic_nodes; i++)
		if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->colocated[i]))
			IMG_DBG_ASSERT("Free failed" == NULL);

	for (i = 0; i < video->pic_nodes; i++)
		if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->recon_pictures[i]))
			IMG_DBG_ASSERT("Free failed" == NULL);

	for (i = 0; i < video->mv_stores; i++)
		if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->mv[i]))
			IMG_DBG_ASSERT("Free failed" == NULL);

	if (video->enable_mvc) {
		for (i = 0; i < 2; i++) {
			if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->inter_view_mv[i]))
				IMG_DBG_ASSERT("Free failed" == NULL);
		}
	}

	if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->mtx_enc_ctx_mem))
		IMG_DBG_ASSERT("Free failed" == NULL);

	if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->mv_settings_btable))
		IMG_DBG_ASSERT("Free failed" == NULL);

	if (video->mv_settings_hierarchical.cpu_virt)
		if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->mv_settings_hierarchical))
			IMG_DBG_ASSERT("Free failed" == NULL);

	/* partially coded headers supplied to HW */
	/* SEI_INSERTION */
	if (video->insert_hrd_params) {
		if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->aud_header_mem))
			IMG_DBG_ASSERT("Free failed" == NULL);

		if (topaz_mmu_stream_free(str_ctx->mmu_ctx,
					  &video->sei_buffering_period_header_mem))
			IMG_DBG_ASSERT("Free failed" == NULL);

		if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->sei_picture_timing_header_mem))
			IMG_DBG_ASSERT("Free failed" == NULL);
	}

	if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->seq_header_mem))
		IMG_DBG_ASSERT("Free failed" == NULL);

	/* FREE subset sequence parameter header */
	if (video->enable_mvc)
		if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->subset_seq_header_mem))
			IMG_DBG_ASSERT("Free failed" == NULL);

	for (i = 0; i < ARRAY_SIZE(video->pichdr_template_mem); i++) {
		if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->pichdr_template_mem[i]))
			IMG_DBG_ASSERT("Free failed" == NULL);
	}

	for (i = 0; i < ARRAY_SIZE(video->slice_params_template_mem); i++) {
		if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->slice_params_template_mem[i]))
			IMG_DBG_ASSERT("Free failed" == NULL);
	}

	if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->src_phys_addr))
		IMG_DBG_ASSERT("Free failed" == NULL);

	/* de-allocate memory corresponding to the output parameters */
	for (i = 0; i < video->slots_in_use; i++) {
		if (video->firstpass_out_param_buf[i].mem_info.cpu_virt)
			if (topaz_mmu_stream_free(str_ctx->mmu_ctx,
						  &video->firstpass_out_param_buf[i].mem_info))
				IMG_DBG_ASSERT("Free failed" == NULL);

		if (video->mb_ctrl_in_params_buf[i].mem_info.cpu_virt)
			if (topaz_mmu_stream_free(str_ctx->mmu_ctx,
						  &video->mb_ctrl_in_params_buf[i].mem_info))
				IMG_DBG_ASSERT("Free failed" == NULL);
	}

	/* de-allocate memory corresponding to the selectable best MV parameters */
	for (i = 0; i < video->slots_in_use; i++) {
		if (video->firstpass_out_best_multipass_param_buf[i].mem_info.cpu_virt)
			if (topaz_mmu_stream_free
				(str_ctx->mmu_ctx,
				 &video->firstpass_out_best_multipass_param_buf[i].mem_info))
				IMG_DBG_ASSERT("Free failed" == NULL);
	}

	for (i = 0; i < video->slots_in_use; i++) {
		if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->ltref_header[i]))
			IMG_DBG_ASSERT("Free failed" == NULL);
	}

	if (video->custom_scaling) {
		for (i = 0; i < 2; i++) {
			if (topaz_mmu_stream_free(str_ctx->mmu_ctx, &video->custom_quant[i]))
				IMG_DBG_ASSERT("Free failed" == NULL);

			if (topaz_mmu_stream_free(str_ctx->mmu_ctx,
						  &video->custom_quant_regs4x4_sp[i]))
				IMG_DBG_ASSERT("Free failed" == NULL);

			if (topaz_mmu_stream_free(str_ctx->mmu_ctx,
						  &video->custom_quant_regs8x8_sp[i]))
				IMG_DBG_ASSERT("Free failed" == NULL);

			if (topaz_mmu_stream_free(str_ctx->mmu_ctx,
						  &video->custom_quant_regs4x4_q[i]))
				IMG_DBG_ASSERT("Free failed" == NULL);

			if (topaz_mmu_stream_free(str_ctx->mmu_ctx,
						  &video->custom_quant_regs8x8_q[i]))
				IMG_DBG_ASSERT("Free failed" == NULL);
		}
	}

	topazdd_destroy_stream_ctx(video->dd_str_ctx);

	topaz_mmu_stream_destroy(&global_topaz_core_context->dev_handle->topaz_mmu_ctx,
				 str_ctx->mmu_ctx);

	/* free the video encoder structure itself */
	kfree(video);

	return IMG_SUCCESS;
}

/*
 * Destroy an Encoder Context
 */
int topaz_stream_destroy(void *str_context)
{
	struct img_enc_context *enc;
	struct topaz_stream_context *str_ctx;
	int ret;

	str_ctx = (struct topaz_stream_context *)str_context;
	if (!str_ctx)
		return IMG_ERROR_INVALID_PARAMETERS;

	enc = str_ctx->enc_ctx;

	ret = topaz_video_destroy_context(str_ctx);

	kfree(enc->debug_settings);
	enc->debug_settings = NULL;

	kfree(enc);
	kfree(str_context);

	return ret;
}

/*
 * Get the capabilities of the encoder for the given codec
 */
int topaz_get_encoder_caps(enum img_standard standard,
			   unsigned short width, unsigned short height,
			   struct img_enc_caps *caps)
{
	unsigned int width_in_mbs, height_in_mbs, kick_size, kicks_per_bu, min_slice_height, mbs;

	/* get the actual number of cores */
	caps->num_cores = topazdd_get_num_pipes(global_topaz_core_context->dev_handle);

	if (caps->num_cores < 3)
		caps->max_bu_per_frame = TOPAZHP_MAX_BU_SUPPORT_HD;
	else
		caps->max_bu_per_frame = TOPAZHP_MAX_BU_SUPPORT_4K;

	caps->core_features = topazdd_get_core_des1();
	caps->core_revision = topazdd_get_core_rev();

	width_in_mbs = (width + 15) / 16;
	height_in_mbs = (height + 15) / 16;

	switch (standard) {
	case IMG_STANDARD_H264:
		/* Assume progressive video for now as we don't know either way */
		calculate_kick_and_bu_size(width_in_mbs, height_in_mbs, FALSE,
					   caps->max_bu_per_frame, &kick_size, &kicks_per_bu,
					   &min_slice_height);
		caps->max_slices = height_in_mbs / min_slice_height;

		/*
		 * Limit for number of MBs in slices is 32K-2 = 32766
		 * Here we will limit it to 16K per slice = 16384
		 */
		caps->min_slices = 1;
		mbs = width_in_mbs * height_in_mbs;
		if (mbs >= 32768)
			caps->min_slices = 3;
		else if (mbs >= 16384)
			caps->min_slices = 2;

		/* if height is bigger or equal to 4000, use at least two slices */
		if (height_in_mbs >= 250 && caps->min_slices == 1)
			caps->min_slices = 2;

		caps->recommended_slices = min(caps->num_cores, caps->max_slices);
		caps->min_slice_height = min_slice_height;

		caps->max_height = 2048;
		caps->max_width = 2048;
		caps->min_height = 48;
		caps->min_width = 144;
		caps->max_mb_num = (2048 * 2048) >> 8;
		break;
	default:
		IMG_DBG_ASSERT("Only H264 encoder is supported" == NULL);
	}

	if (caps->recommended_slices < caps->min_slices)
		caps->recommended_slices = caps->min_slices;
	if (caps->recommended_slices > caps->max_slices)
		caps->recommended_slices = caps->max_slices;

	return IMG_SUCCESS;
}

/*
 * Supply a source frame to the encode process
 */
int topaz_send_source_frame(void *topaz_str_ctx, struct img_frame *src_frame,
			    unsigned int frame_num, unsigned long long ctx)
{
	struct topaz_stream_context *str_ctx;
	struct img_source_buffer_params *buffer_params;

	struct img_enc_context *enc;
	struct img_video_context *video;
	unsigned char slot_number;
	void *data;
	unsigned int y_plane_base = 0;
	unsigned int u_plane_base = 0;
	unsigned int v_plane_base = 0;
	struct vidio_ddbufinfo *cmd_data_mem_info = NULL;
	unsigned char *slice_map_addr = NULL;
	unsigned char index;
	unsigned char round;
	unsigned char slice_number;
	unsigned char first_bu_in_slice;
	unsigned char size_in_bus;
	unsigned int slice_height;
	unsigned char halfway_slice;
	unsigned int halfway_bu;
	unsigned char slices_per_picture;
	unsigned int picture_height_remaining;

	if (!topaz_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	/* if source slot is NULL then it's just a next portion of slices */
	if (!src_frame)
		return IMG_ERROR_UNEXPECTED_STATE;

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;

	enc = str_ctx->enc_ctx;
	video = enc->video;

	if (video->aborted)
		return IMG_ERROR_UNEXPECTED_STATE;

	slot_number = video->source_slot_reserved;

	/* mark the appropriate slot as filled */
	video->source_slot_buff[slot_number] = src_frame;
	video->source_slot_poc[slot_number] = frame_num;

	topaz_get_cmd_data_buffer(&cmd_data_mem_info);

	if (!cmd_data_mem_info)
		return IMG_ERROR_UNEXPECTED_STATE;

	data = cmd_data_mem_info->cpu_virt;
	buffer_params = (struct img_source_buffer_params *)data;

	/* Prepare data */
	if (src_frame->y_plane_buffer) {
		populate_firmware_message(&video->src_phys_addr, 0,
					  &src_frame->y_plane_buffer->mem_info, 0);

		data = video->src_phys_addr.cpu_virt;
		y_plane_base = *((unsigned int *)data);
	}

	if (src_frame->u_plane_buffer) {
		populate_firmware_message(&video->src_phys_addr, 0,
					  &src_frame->u_plane_buffer->mem_info, 0);

		data = video->src_phys_addr.cpu_virt;
		u_plane_base = *((unsigned int *)data);
	} else {
		u_plane_base = y_plane_base;
	}

	if (src_frame->v_plane_buffer) {
		populate_firmware_message(&video->src_phys_addr, 0,
					  &src_frame->v_plane_buffer->mem_info, 0);

		data = video->src_phys_addr.cpu_virt;
		v_plane_base = *((unsigned int *)data);
	} else {
		v_plane_base = u_plane_base;
	}

	buffer_params->slot_num = slot_number;
	buffer_params->display_order_num = (unsigned char)(frame_num & 0xFF);
	buffer_params->host_context = ctx;

	buffer_params->phys_addr_y_plane_field_0 = y_plane_base + src_frame->y_component_offset +
		src_frame->field0_y_offset;
	buffer_params->phys_addr_u_plane_field_0 = u_plane_base + src_frame->u_component_offset +
		src_frame->field0_u_offset;
	buffer_params->phys_addr_v_plane_field_0 = v_plane_base + src_frame->v_component_offset +
		src_frame->field0_v_offset;

	buffer_params->phys_addr_y_plane_field_1 = y_plane_base + src_frame->y_component_offset +
		src_frame->field1_y_offset;
	buffer_params->phys_addr_u_plane_field_1 = u_plane_base + src_frame->u_component_offset +
		src_frame->field1_u_offset;
	buffer_params->phys_addr_v_plane_field_1 = v_plane_base + src_frame->v_component_offset +
		src_frame->field1_v_offset;

	topaz_update_device_mem(str_ctx->vxe_ctx, cmd_data_mem_info);

	topaz_get_buffer(str_ctx, &video->slice_map[slot_number], (void **)&slice_map_addr,
			 FALSE);

	/* Fill standard Slice Map (non arbitrary) */
	halfway_bu = 0;
	first_bu_in_slice = 0;
	slice_number = 0;
	slices_per_picture = video->slices_per_picture;
	picture_height_remaining = video->picture_height;
	halfway_slice = slices_per_picture / 2;
	*slice_map_addr = slices_per_picture;
	slice_map_addr++;
	round = 16 * enc->caps.min_slice_height - 1;

	for (index = 0; index < slices_per_picture - 1; index++) {
		if (index == halfway_slice)
			halfway_bu = first_bu_in_slice;

		slice_height = (picture_height_remaining / (video->slices_per_picture - index)) &
			~round;
		picture_height_remaining -= slice_height;
		size_in_bus = ((slice_height / 16) * (video->width / 16)) /
			video->rc_params.bu_size;

		/* slice number */
		*slice_map_addr = slice_number;
		slice_map_addr++;

		/* SizeInKicks BU */
		*slice_map_addr = size_in_bus;
		slice_map_addr++;

		slice_number++;

		first_bu_in_slice += (unsigned int)size_in_bus;
	}

	slice_height = picture_height_remaining;
	if (index == halfway_slice)
		halfway_bu = first_bu_in_slice;

	/* round up for case where the last BU is smaller */
	size_in_bus = ((slice_height / 16) * (video->width / 16) + video->rc_params.bu_size - 1) /
		video->rc_params.bu_size;

	/* slice number */
	*slice_map_addr = slice_number;
	slice_map_addr++;

	/* last BU */
	*slice_map_addr = size_in_bus;
	slice_map_addr++;

	topaz_release_buffer(str_ctx, &video->slice_map[slot_number], TRUE);

#ifdef DEBUG_ENCODER_DRIVER
	pr_info("\n\nAPI - IMG_V_SendSourceFrame - Sending a source slot %i to FW\n\n",
		slot_number);
#endif

	/* Send command */
	topaz_insert_mem_command(enc, MTX_CMDID_PROVIDE_SOURCE_BUFFER, 0, cmd_data_mem_info);

	video->encode_requested++;

	return IMG_SUCCESS;
}

/*
 * Supply a header buffer and an optional number of coded data buffers as part of a package
 */
int topaz_send_coded_package(void *topaz_str_ctx, struct img_coded_buffer *coded_buffer)
{
	struct img_enc_context *enc;
	struct img_video_context *video;
	struct topaz_stream_context *str_ctx;
	unsigned char coded_buffer_idx;
	unsigned int *address = NULL;
	struct coded_package_dma_info *this_coded_header_node;

	if (!topaz_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;
	if (!coded_buffer)
		return IMG_ERROR_INVALID_PARAMETERS;

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;

	enc = str_ctx->enc_ctx;
	video = enc->video;

	if (video->aborted)
		return IMG_ERROR_UNEXPECTED_STATE;

	video->coded_package[video->coded_package_slot_reserved]->coded_buffer[0] = coded_buffer;

#ifdef DEBUG_ENCODER_DRIVER
	pr_info("\n\nEncode Context [%i] sending coded package [%i]\n", enc->ctx_num,
		video->coded_package_slot_reserved);
#endif

	/* Get the FW buffer */
	topaz_get_buffer
	(str_ctx,
	 video->coded_package[video->coded_package_slot_reserved]->mtx_info.code_package_fw_buffer,
	 (void **)&address, FALSE);

	this_coded_header_node =
	video->coded_package[video->coded_package_slot_reserved]->mtx_info.coded_package_fw =
							(struct coded_package_dma_info *)address;

	this_coded_header_node->coded_buffer_info =
		F_ENCODE
		(video->coded_package[video->coded_package_slot_reserved]->num_coded_buffers,
		 MTX_MSG_NUM_CODED_BUFFERS_PER_HEADER);

	/* Inverted function: From host to MTX */
	populate_firmware_message(&(video->coded_package
		[video->coded_package_slot_reserved]->mtx_info.code_package_fw_buffer->mem_info),
		(unsigned char *)&this_coded_header_node->coded_header_addr -
		(unsigned char *)this_coded_header_node,
		(struct vidio_ddbufinfo *)
		(&(video->coded_package[video->coded_package_slot_reserved]->header_buffer->mem_info
		)), 0);

	/* Normal mode - An array of consecutive memory addresses */
	for (coded_buffer_idx = 0; coded_buffer_idx <
		video->coded_package[video->coded_package_slot_reserved]->num_coded_buffers;
		coded_buffer_idx++) {
		if (video->coded_package[video->coded_package_slot_reserved]->coded_buffer
							[coded_buffer_idx]) {
			/* Write coded buffer memory address into the structure (host to MTX) */
			populate_firmware_message(&(video->coded_package
		[video->coded_package_slot_reserved]->mtx_info.code_package_fw_buffer->mem_info),
		(unsigned char *)&this_coded_header_node->coded_mem_addr[coded_buffer_idx] -
		(unsigned char *)this_coded_header_node, (struct vidio_ddbufinfo *)
		(&(video->coded_package
		[video->coded_package_slot_reserved]->coded_buffer[coded_buffer_idx]->mem_info)),
		0);
		} else {
			this_coded_header_node->coded_mem_addr[coded_buffer_idx] = 0;
			break;
		}
	}

	/* Release the FW buffer */
	topaz_release_buffer(str_ctx, video->coded_package
	[video->coded_package_slot_reserved]->mtx_info.code_package_fw_buffer, TRUE);

	/* Send header buffers to the MTX */
	topaz_insert_mem_command(enc, (enum mtx_cmd_id)(MTX_CMDID_PROVIDE_CODEDPACKAGE_BUFFER |
							MTX_CMDID_WB_INTERRUPT),
	F_ENCODE(video->coded_package[video->coded_package_slot_reserved]->coded_buffer[0]->size >>
	10, MTX_MSG_PROVIDE_CODED_BUFFER_SIZE) |
	F_ENCODE(video->coded_package_slot_reserved, MTX_MSG_PROVIDE_CODEDPACKAGE_BUFFER_SLOT),
	&(video->coded_package
		[video->coded_package_slot_reserved]->mtx_info.code_package_fw_buffer->mem_info));

	return IMG_SUCCESS;
}

unsigned int topaz_get_coded_buffer_max_size(void *topaz_str_ctx, enum img_standard standard,
					     unsigned short width, unsigned short height,
					     struct img_rc_params *rc_params)
{
	/* TODO: Determine if we want to make this api str_ctx dependent
	 * struct topaz_stream_context *str_ctx;
	 * if (!topaz_str_ctx)
	 * return IMG_ERROR_INVALID_CONTEXT;
	 */
	/* Worst-case coded buffer size: All MBs maximum size,
	 * and a coded buffer header for each row
	 */
	return topaz_get_max_coded_data_size(standard, width, height, rc_params->initial_qp_i) +
	       ((height >> 4) * CODED_BUFFER_INFO_SECTION_SIZE);
}

unsigned int topaz_get_coded_package_max_num(void *topaz_str_ctx, enum img_standard standard,
					     unsigned short width, unsigned short height,
					     struct img_rc_params *rc_params)
{
	struct topaz_stream_context *str_ctx;

	if (!topaz_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;

	return str_ctx->enc_ctx->video->coded_package_max_num;
}

/*
 * Get a source slot to fill
 */
int topaz_reserve_source_slot(void *topaz_str_ctx, unsigned char *src_slot_num)
{
	struct img_enc_context *enc;
	struct img_video_context *video;
	struct topaz_stream_context *str_ctx;
	signed char index;

	if (!topaz_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;
	enc = str_ctx->enc_ctx;
	video = enc->video;

	if (video->aborted)
		return IMG_ERROR_UNEXPECTED_STATE;

	for (index = 0; index < video->slots_in_use; index++) {
		if (!video->source_slot_buff[index]) {
			/* Found an empty slot, Mark the slot as reserved */
			video->source_slot_reserved = index;
			*src_slot_num = index;
			return IMG_SUCCESS;
		}
	}

	return IMG_ERROR_UNEXPECTED_STATE;
}

/*
 * Get a coded slot to fill
 */
int topaz_reserve_coded_package_slot(void *topaz_str_ctx)
{
	struct img_enc_context *enc;
	struct img_video_context *video;
	struct topaz_stream_context *str_ctx;
	signed char index;

	if (!topaz_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;

	enc = str_ctx->enc_ctx;
	video = enc->video;

	if (video->aborted)
		return IMG_ERROR_UNEXPECTED_STATE;

	for (index = 0; index < video->coded_package_max_num; index++) {
		if (!video->coded_package[index]->busy) {
			/* Found an empty slot, Mark the slot as reserved */
			video->coded_package_slot_reserved = index;
			video->coded_package[index]->busy = TRUE;
			return IMG_SUCCESS;
		}
	}

	return IMG_ERROR_UNEXPECTED_STATE;
}

/*
 * Returns number of empty source slots
 */
signed char topaz_query_empty_source_slots(void *topaz_str_ctx)
{
	struct topaz_stream_context *str_ctx;
	struct img_enc_context *enc;
	struct img_video_context *video;

	unsigned char slot_number;
	unsigned char empty_source_slots = 0;

	if (!topaz_str_ctx) {
		pr_err("ERROR: Invalid context handle provides to IMG_V_QueryEmptySourceSlots\n");
		return IMG_ERROR_INVALID_CONTEXT;
	}

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;
	enc = str_ctx->enc_ctx;
	video = enc->video;

	if (video->aborted)
		return -2;

	for (slot_number = 0; slot_number < video->slots_in_use; slot_number++) {
		if (!video->source_slot_buff[slot_number])
			empty_source_slots++;
	}

	return empty_source_slots;
}

/*
 * Returns number of empty coded buffer slots
 */
signed char topaz_query_empty_coded_slots(void *topaz_str_ctx)
{
	struct topaz_stream_context *str_ctx;
	struct img_enc_context *enc;
	struct img_video_context *video;

	unsigned char slot_number;
	unsigned char empty_coded_slots = 0;

	if (!topaz_str_ctx) {
		pr_err("ERROR: Invalid context handle provides to IMG_V_QueryEmptyCodedSlots\n");
		return IMG_ERROR_INVALID_CONTEXT;
	}

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;
	enc = str_ctx->enc_ctx;
	video = enc->video;

	if (video->aborted)
		return -2;

	for (slot_number = 0; slot_number < video->coded_package_max_num; slot_number++) {
		if (!video->coded_package[slot_number]->busy)
			empty_coded_slots++;
	}

	return empty_coded_slots;
}

/*
 * topaz_stream_map_buf_sg
 */
int topaz_stream_map_buf_sg(void *topaz_str_ctx, enum venc_buf_type buf_type,
			    struct vidio_ddbufinfo *buf_info, void *sgt)
{
	int ret;
	struct topaz_stream_context *str_ctx;

	/*
	 * Resource stream ID cannot be zero. If zero just warning and
	 * proceeding further will break the code. Return IMG_ERROR_INVALID_ID.
	 */
	if (!topaz_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	IMG_DBG_ASSERT(buf_type < VENC_BUFTYPE_MAX);
	IMG_DBG_ASSERT(buf_info);
	IMG_DBG_ASSERT(sgt);

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;

	/* Map heap from VENC to MMU. Currently only one heap is used for all buffer types */
	switch (buf_type) {
	case VENC_BUFTYPE_BITSTREAM:
	case VENC_BUFTYPE_PICTURE:
		/* TODO: add logic to cache these buffers into str context list */
		break;

	default:
		IMG_DBG_ASSERT(FALSE);
	}

	/* Map this buffer into the MMU. */
	ret = topaz_mmu_stream_map_ext_sg(str_ctx->mmu_ctx, MMU_GENERAL_HEAP_ID, sgt,
					  buf_info->buf_size, 64,
					  (enum sys_emem_attrib)0, buf_info->cpu_virt, buf_info,
					  &buf_info->buff_id);
	IMG_DBG_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return IMG_ERROR_OUT_OF_MEMORY;

	return IMG_SUCCESS;
}

/*
 * core_stream_unmap_buf_sg
 */
int topaz_stream_unmap_buf_sg(void *topaz_str_ctx, struct vidio_ddbufinfo *buf_info)
{
	int ret;
	struct topaz_stream_context *str_ctx;

	if (!topaz_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;

	/* Unmap this buffer from the MMU. */
	ret = topaz_mmu_stream_free_sg(str_ctx->mmu_ctx, buf_info);

	IMG_DBG_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	return IMG_SUCCESS;
}

/*
 * End Of Video stream
 */
int topaz_end_of_stream(void *topaz_str_ctx, unsigned int frame_cnt)
{
	struct topaz_stream_context *str_ctx;
	struct img_enc_context *enc;

	if (!topaz_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	str_ctx = (struct topaz_stream_context *)topaz_str_ctx;
	enc = str_ctx->enc_ctx;

	if (enc->video->aborted)
		return IMG_ERROR_UNDEFINED;

	enc->video->frame_count = frame_cnt;

	if (frame_cnt - enc->video->flushed_at_frame < enc->video->slots_in_use)
		enc->video->slots_required = frame_cnt - enc->video->flushed_at_frame;

	/* Send PicMgmt Command */
	topaz_insert_command(enc, (enum mtx_cmd_id)(MTX_CMDID_PICMGMT | MTX_CMDID_PRIORITY),
			     F_ENCODE(IMG_PICMGMT_EOS, MTX_MSG_PICMGMT_SUBTYPE) |
			     F_ENCODE(frame_cnt, MTX_MSG_PICMGMT_DATA));

	return IMG_SUCCESS;
}
