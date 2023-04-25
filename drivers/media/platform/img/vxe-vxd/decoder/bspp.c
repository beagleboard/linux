// SPDX-License-Identifier: GPL-2.0
/*
 * VXD Bitstream Buffer Pre-Parser
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Lakshmi Sankar <lakshmisankar-t@ti.com>
 *
 * Re-written for upstreming
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "bspp.h"
#include "h264_secure_parser.h"
#include "hevc_secure_parser.h"
#ifdef HAS_JPEG
#include "jpeg_secure_parser.h"
#endif
#include "lst.h"
#include "swsr.h"
#include "vdecdd_defs.h"
#include "img_errors.h"

#define BSPP_ERR_MSG_LENGTH     1024

/*
 * This type defines the exception flag to catch the error if more catch block
 * is required to catch different kind of error then more enum can be added
 * @breif BSPP exception handler to catch the errors
 */
enum bspp_exception_handler {
	/* BSPP parse exception handler */
	BSPP_EXCEPTION_HANDLER_NONE = 0x00,
	/* Jump at exception (external use) */
	BSPP_EXCEPTION_HANDLER_JUMP,
	BSPP_EXCEPTION_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This structure contains bitstream buffer information.
 * @brief  BSPP Bitstream Buffer Information
 */
struct bspp_bitstream_buffer {
	void **lst_link;
	struct bspp_ddbuf_info ddbuf_info;
	unsigned int data_size;
	unsigned int bufmap_id;
	enum vdec_bstr_element_type bstr_element_type;
	unsigned long long bytes_read;
	void *pict_tag_param;
};

/*
 * This structure contains shift-register state.
 * @brief  BSPP Shift-register State
 */
struct bspp_parse_ctx {
	void *swsr_context;
	enum swsr_exception exception;
};

/*
 * This structure contains context for the current picture.
 * @brief  BSPP Picture Context
 */
struct bspp_pict_ctx {
	struct bspp_sequence_hdr_info *sequ_hdr_info;
	int closed_gop;
	struct bspp_pict_hdr_info pict_hdr_info[VDEC_H264_MVC_MAX_VIEWS];
	struct bspp_sequence_hdr_info *ext_sequ_hdr_info;
	int present;
	int invalid;
	int unsupported;
	int finished;
	unsigned int new_pict_signalled;
};

/*
 * This structure contains resources allocated for the stream.
 * @brief  BSPP Stream Resource Allocations
 */
struct bspp_stream_alloc_data {
	struct lst_t sequence_data_list[SEQUENCE_SLOTS];
	struct lst_t pps_data_list[PPS_SLOTS];
	struct lst_t available_sequence_list;
	struct lst_t available_ppss_list;
	struct lst_t raw_data_list_available;
	struct lst_t raw_data_list_used;
	struct lst_t vps_data_list[VPS_SLOTS];
	struct lst_t raw_sei_alloc_list;
	struct lst_t available_vps_list;
};

struct bspp_raw_sei_alloc {
	void **lst_link;
	struct vdec_raw_bstr_data raw_sei_data;
};

/*
 * This structure contains bitstream parsing state information for the current
 * group of buffers.
 * @brief  BSPP Bitstream Parsing State Information
 */
struct bspp_grp_bstr_ctx {
	enum vdec_vid_std vid_std;
	int disable_mvc;
	int delim_present;
	void *swsr_context;
	enum bspp_unit_type unit_type;
	enum bspp_unit_type last_unit_type;
	int not_pic_unit_yet;
	int not_ext_pic_unit_yet;
	unsigned int total_data_size;
	unsigned int total_bytes_read;
	struct lst_t buffer_chain;
	struct lst_t in_flight_bufs;
	struct lst_t *pre_pict_seg_list[3];
	struct lst_t *pict_seg_list[3];
	void **pict_tag_param_array[3];
	struct lst_t *segment_list;
	void **pict_tag_param;
	struct lst_t *free_segments;
	unsigned int segment_offset;
	int insert_start_code;
	unsigned char start_code_suffix;
	unsigned char current_view_idx;
};

/*
 * This structure contains the stream context information.
 * @brief  BSPP Stream Context Information
 */
struct bspp_str_context {
	enum vdec_vid_std vid_std;
	int disable_mvc;
	int full_scan;
	int immediate_decode;
	enum vdec_bstr_format bstr_format;
	struct vdec_codec_config codec_config;
	unsigned int user_str_id;
	struct bspp_vid_std_features vid_std_features;
	struct bspp_swsr_ctx swsr_ctx;
	struct bspp_parser_callbacks parser_callbacks;
	struct bspp_stream_alloc_data str_alloc;
	unsigned int sequ_hdr_id;
	unsigned char *sequ_hdr_info;
	unsigned char *secure_sequence_info;
	unsigned char *pps_info;
	unsigned char *secure_pps_info;
	unsigned char *raw_data;
	struct bspp_grp_bstr_ctx grp_bstr_ctx;
	struct bspp_parse_ctx parse_ctx;
	struct bspp_inter_pict_data inter_pict_data;
	struct lst_t decoded_pictures_list;
	/* Mutex for secure access */
	struct mutex *bspp_mutex;
	int intra_frame_closed_gop;
	struct bspp_pict_ctx pict_ctx;
	struct bspp_parse_state parse_state;
};

/*
 * This structure contains the standard related parser functions.
 * @brief  BSPP Standard Related Functions
 */
struct bspp_parser_functions {
	/* Pointer to standard-specific parser configuration function */
	bspp_cb_set_parser_config set_parser_config;
	/* Pointer to standard-specific unit type determining function */
	bspp_cb_determine_unit_type determine_unit_type;
};

static struct bspp_parser_functions parser_fxns[VDEC_STD_MAX] = {
	/* VDEC_STD_UNDEFINED */
	{ .set_parser_config = NULL, .determine_unit_type = NULL },
	/* VDEC_STD_MPEG2 */
	{ .set_parser_config = NULL, .determine_unit_type = NULL },
	/* VDEC_STD_MPEG4 */
	{ .set_parser_config = NULL, .determine_unit_type = NULL },
	/* VDEC_STD_H263 */
	{ .set_parser_config = NULL, .determine_unit_type = NULL },
	/* VDEC_STD_H264 */
	{ .set_parser_config = bspp_h264_set_parser_config,
		.determine_unit_type = bspp_h264_determine_unittype },
	/* VDEC_STD_VC1 */
	{ .set_parser_config = NULL, .determine_unit_type = NULL },
	/* VDEC_STD_AVS */
	{ .set_parser_config = NULL, .determine_unit_type = NULL },
	/* VDEC_STD_REAL */
	{ .set_parser_config = NULL, .determine_unit_type = NULL },
	/* VDEC_STD_JPEG */
#ifdef HAS_JPEG
	{ .set_parser_config = bspp_jpeg_setparser_config,
		.determine_unit_type = bspp_jpeg_determine_unit_type },
#else
	{ .set_parser_config = NULL, .determine_unit_type = NULL },
#endif
	/* VDEC_STD_VP6 */
	{ .set_parser_config = NULL, .determine_unit_type = NULL },
	/* VDEC_STD_VP8 */
	{ .set_parser_config = NULL, .determine_unit_type = NULL },
	/* VDEC_STD_SORENSON */
	{ .set_parser_config = NULL, .determine_unit_type = NULL },
	/* VDEC_STD_HEVC */
	{ .set_parser_config = bspp_hevc_set_parser_config,
		.determine_unit_type = bspp_hevc_determine_unittype },
};

/*
 * @Function	bspp_get_pps_hdr
 * @Description	Obtains the most recent PPS header of a given Id.
 */
struct bspp_pps_info *bspp_get_pps_hdr(void *str_res_handle, unsigned int pps_id)
{
	struct bspp_stream_alloc_data *alloc_data =
		(struct bspp_stream_alloc_data *)str_res_handle;

	if (pps_id >= PPS_SLOTS || !alloc_data)
		return NULL;

	return lst_last(&alloc_data->pps_data_list[pps_id]);
}

/*
 * @Function	bspp_get_sequ_hdr
 * @Description	Obtains the most recent sequence header of a given Id.
 */
struct bspp_sequence_hdr_info *bspp_get_sequ_hdr(void *str_res_handle,
						 unsigned int sequ_id)
{
	struct bspp_stream_alloc_data *alloc_data =
		(struct bspp_stream_alloc_data *)str_res_handle;
	if (sequ_id >= SEQUENCE_SLOTS || !alloc_data)
		return NULL;

	return lst_last(&alloc_data->sequence_data_list[sequ_id]);
}

/*
 * @Function	bspp_free_bitstream_elem
 * @Description	Frees a bitstream chain element.
 */
static void bspp_free_bitstream_elem(struct bspp_bitstream_buffer *bstr_buf)
{
	memset(bstr_buf, 0, sizeof(struct bspp_bitstream_buffer));

	kfree(bstr_buf);
}

/*
 * @Function	bspp_create_segment
 * @Description Constructs a bitstream segment for the current unit and adds
 *		it to the list.
 */
static int bspp_create_segment(struct bspp_grp_bstr_ctx *grp_btsr_ctx,
			       struct bspp_bitstream_buffer *cur_buf)
{
	struct bspp_bitstr_seg *segment;
	unsigned int result;

	/*
	 * Only create a segment when data (not in a previous segment) has been
	 * parsed from the buffer.
	 */
	if (cur_buf->bytes_read != grp_btsr_ctx->segment_offset) {
		/* Allocate a software shift-register context structure */
		segment = lst_removehead(grp_btsr_ctx->free_segments);
		if (!segment) {
			result = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
			goto error;
		}
		memset(segment, 0, sizeof(struct bspp_bitstr_seg));

		segment->bufmap_id = cur_buf->bufmap_id;
		segment->data_size = (unsigned int)cur_buf->bytes_read
			- grp_btsr_ctx->segment_offset;
		segment->data_byte_offset = grp_btsr_ctx->segment_offset;

		if (cur_buf->bytes_read == cur_buf->data_size) {
			/* This is the last segment in the buffer. */
			segment->bstr_seg_flag |= VDECDD_BSSEG_LASTINBUFF;
		}

		/*
		 * Next segment will start part way through the buffer
		 * (current read position).
		 */
		grp_btsr_ctx->segment_offset = (unsigned int)cur_buf->bytes_read;

		if (grp_btsr_ctx->insert_start_code) {
			segment->bstr_seg_flag |= VDECDD_BSSEG_INSERT_STARTCODE;
			segment->start_code_suffix = grp_btsr_ctx->start_code_suffix;
			grp_btsr_ctx->insert_start_code = 0;
		}

		lst_add(grp_btsr_ctx->segment_list, segment);

		/*
		 * If multiple segments correspond to the same (picture)
		 * stream-unit, update it only the first time
		 */
		if (cur_buf->pict_tag_param && grp_btsr_ctx->pict_tag_param &&
		    (grp_btsr_ctx->segment_list ==
		     grp_btsr_ctx->pict_seg_list[0] ||
		     grp_btsr_ctx->segment_list ==
		     grp_btsr_ctx->pict_seg_list[1] ||
		     grp_btsr_ctx->segment_list ==
		     grp_btsr_ctx->pict_seg_list[2]))
			*grp_btsr_ctx->pict_tag_param = cur_buf->pict_tag_param;
	}

	return IMG_SUCCESS;
error:
	return result;
}

/*
 * @Function bspp_DetermineUnitType
 *
 */
static int bspp_determine_unit_type(enum vdec_vid_std vid_std,
				    unsigned char unit_type,
				    int disable_mvc,
				    enum bspp_unit_type *unit_type_enum)
{
	/* Determine the unit type from the NAL type. */
	if (vid_std < VDEC_STD_MAX && parser_fxns[vid_std].determine_unit_type)
		parser_fxns[vid_std].determine_unit_type(unit_type, disable_mvc, unit_type_enum);
	else
		return IMG_ERROR_INVALID_PARAMETERS;

	return IMG_SUCCESS;
}

/*
 * @Function	bspp_shift_reg_cb
 *
 */
static void bspp_shift_reg_cb(enum swsr_cbevent event,
			      struct bspp_grp_bstr_ctx *grp_btsr_ctx,
			      unsigned char nal_type,
			      unsigned char **data_buffer,
			      unsigned long long *data_size)
{
	unsigned int result;

	switch (event) {
	case SWSR_EVENT_INPUT_BUFFER_START: {
		struct bspp_bitstream_buffer *next_buf;

		/* Take the next bitstream buffer for use in shift-register. */
		next_buf = lst_removehead(&grp_btsr_ctx->buffer_chain);

		if (next_buf && data_buffer && data_size) {
			lst_add(&grp_btsr_ctx->in_flight_bufs, next_buf);

			*data_buffer = next_buf->ddbuf_info.cpu_virt_addr;
			*data_size = next_buf->data_size;

			next_buf->bytes_read = 0;
		} else {
			goto error;
		}
	}
	break;
	case SWSR_EVENT_OUTPUT_BUFFER_END: {
		struct bspp_bitstream_buffer *cur_buf;

		cur_buf = lst_removehead(&grp_btsr_ctx->in_flight_bufs);

		if (cur_buf) {
			/*
			 * Indicate that the whole buffer content has been
			 * used.
			 */
			cur_buf->bytes_read = cur_buf->data_size;
			grp_btsr_ctx->total_bytes_read += (unsigned int)cur_buf->bytes_read;

			/*
			 * Construct segment for current buffer and add to
			 * active list.
			 */
			result = bspp_create_segment(grp_btsr_ctx, cur_buf);
			if (result != IMG_SUCCESS)
				goto error;

			/*
			 * Next segment will start at the beginning of the next
			 * buffer.
			 */
			grp_btsr_ctx->segment_offset = 0;

			/* Destroy the bitstream element. */
			bspp_free_bitstream_elem(cur_buf);
		} else {
			goto error;
		}
	}
	break;

	case SWSR_EVENT_DELIMITER_NAL_TYPE:
		/*
		 * Initialise the unit type with the last (unclassified or
		 * unsupported types are not retained since they.
		 */
		grp_btsr_ctx->unit_type = grp_btsr_ctx->last_unit_type;

		/*
		 * Determine the unit type without consuming any data (start
		 * code) from shift-register. Segments are created automatically
		 * when a new buffer is requested by the shift-register so the
		 * unit type must be known in order to switch over the segment
		 * list.
		 */
		result = bspp_determine_unit_type(grp_btsr_ctx->vid_std, nal_type,
						  grp_btsr_ctx->disable_mvc,
						  &grp_btsr_ctx->unit_type);

		/*
		 * Only look to change bitstream segment list when the unit type
		 * is different and the current unit contains data that could be
		 * placed in a new list.
		 */
		if (grp_btsr_ctx->last_unit_type != grp_btsr_ctx->unit_type &&
		    grp_btsr_ctx->unit_type != BSPP_UNIT_UNSUPPORTED &&
		    grp_btsr_ctx->unit_type != BSPP_UNIT_UNCLASSIFIED) {
			int prev_pict_data;
			int curr_pict_data;

			prev_pict_data = (grp_btsr_ctx->last_unit_type == BSPP_UNIT_PICTURE ||
					  grp_btsr_ctx->last_unit_type ==
					  BSPP_UNIT_SKIP_PICTURE) ? 1 : 0;

			curr_pict_data = (grp_btsr_ctx->unit_type == BSPP_UNIT_PICTURE ||
					  grp_btsr_ctx->unit_type ==
					  BSPP_UNIT_SKIP_PICTURE) ? 1 : 0;

			/*
			 * When switching between picture and non-picture
			 * units.
			 */
			if ((prev_pict_data && !curr_pict_data) ||
			    (!prev_pict_data && curr_pict_data)) {
				/*
				 * Only delimit unit change when we're not the
				 * first unit and  we're not already in the last
				 * segment list.
				 */
				if (grp_btsr_ctx->last_unit_type != BSPP_UNIT_NONE &&
				    grp_btsr_ctx->segment_list !=
				    grp_btsr_ctx->pict_seg_list[2]) {
					struct bspp_bitstream_buffer *cur_buf =
						lst_first(&grp_btsr_ctx->in_flight_bufs);
					if (!cur_buf)
						goto error;

					/*
					 * Update the offset within current buf.
					 */
					swsr_get_byte_offset_curbuf(grp_btsr_ctx->swsr_context,
								    &cur_buf->bytes_read);

					/*
					 * Create the last segment of the
					 * previous type (which may split a
					 * buffer into two). If the unit is
					 * exactly at the start of a buffer this
					 * will not create a zero-byte segment.
					 */
					result = bspp_create_segment(grp_btsr_ctx, cur_buf);
					if (result != IMG_SUCCESS)
						goto error;
				}

				/* Point at the next segment list. */
				if (grp_btsr_ctx->segment_list
					== grp_btsr_ctx->pre_pict_seg_list[0]) {
					grp_btsr_ctx->segment_list =
						grp_btsr_ctx->pict_seg_list[0];
					grp_btsr_ctx->pict_tag_param =
						grp_btsr_ctx->pict_tag_param_array[0];
				} else if (grp_btsr_ctx->segment_list
					== grp_btsr_ctx->pict_seg_list[0])
					grp_btsr_ctx->segment_list =
						grp_btsr_ctx->pre_pict_seg_list[1];
				else if (grp_btsr_ctx->segment_list
					== grp_btsr_ctx->pre_pict_seg_list[1]) {
					grp_btsr_ctx->segment_list =
						grp_btsr_ctx->pict_seg_list[1];
					grp_btsr_ctx->pict_tag_param =
						grp_btsr_ctx->pict_tag_param_array[1];
				} else if (grp_btsr_ctx->segment_list
					== grp_btsr_ctx->pict_seg_list[1])
					grp_btsr_ctx->segment_list =
						grp_btsr_ctx->pre_pict_seg_list[2];
				else if (grp_btsr_ctx->segment_list
					== grp_btsr_ctx->pre_pict_seg_list[2]) {
					grp_btsr_ctx->segment_list =
						grp_btsr_ctx->pict_seg_list[2];
					grp_btsr_ctx->pict_tag_param =
						grp_btsr_ctx->pict_tag_param_array[2];
				}
			}

			grp_btsr_ctx->last_unit_type = grp_btsr_ctx->unit_type;
		}
		break;

	default:
		break;
	}

error:
	return;
}

/*
 * @Function	bspp_exception_handler
 *
 */
static void bspp_exception_handler(enum swsr_exception exception, void *parse_ctx_handle)
{
	struct bspp_parse_ctx *parse_ctx = (struct bspp_parse_ctx *)parse_ctx_handle;

	/* Store the exception. */
	parse_ctx->exception = exception;

	switch (parse_ctx->exception) {
	case SWSR_EXCEPT_NO_EXCEPTION:
		break;
	case SWSR_EXCEPT_ENCAPULATION_ERROR1:
		break;
	case SWSR_EXCEPT_ENCAPULATION_ERROR2:
		break;
	case SWSR_EXCEPT_ACCESS_INTO_SCP:
		break;
	case SWSR_EXCEPT_ACCESS_BEYOND_EOD:
		break;
	case SWSR_EXCEPT_EXPGOULOMB_ERROR:
		break;
	case SWSR_EXCEPT_WRONG_CODEWORD_ERROR:
		break;
	case SWSR_EXCEPT_NO_SCP:
		break;
	case SWSR_EXCEPT_INVALID_CONTEXT:
		break;

	default:
		break;
	}

	/* Clear the exception. */
	swsr_check_exception(parse_ctx->swsr_context);
}

/*
 * @Function	bspp_reset_sequence
 *
 */
static void bspp_reset_sequence(struct bspp_str_context *str_ctx,
				struct bspp_sequence_hdr_info *sequ_hdr_info)
{
	/* Temporarily store relevant sequence fields. */
	struct bspp_ddbuf_array_info aux_fw_sequence = sequ_hdr_info->fw_sequence;
	void *aux_secure_sequence_info_hndl = sequ_hdr_info->secure_sequence_info;

	struct bspp_ddbuf_array_info *tmp = &sequ_hdr_info->fw_sequence;

	/* Reset all related structures. */
	memset(((unsigned char *)tmp->ddbuf_info.cpu_virt_addr + tmp->buf_offset), 0x00,
	       sequ_hdr_info->fw_sequence.buf_element_size);

	if (str_ctx->parser_callbacks.reset_data_cb)
		str_ctx->parser_callbacks.reset_data_cb(BSPP_UNIT_SEQUENCE,
			sequ_hdr_info->secure_sequence_info);
	else
		memset(aux_secure_sequence_info_hndl, 0, str_ctx->vid_std_features.seq_size);

	memset(sequ_hdr_info, 0, sizeof(*sequ_hdr_info));

	/* Restore relevant sequence fields. */
	sequ_hdr_info->fw_sequence = aux_fw_sequence;
	sequ_hdr_info->sequ_hdr_info.bufmap_id = aux_fw_sequence.ddbuf_info.bufmap_id;
	sequ_hdr_info->sequ_hdr_info.buf_offset = aux_fw_sequence.buf_offset;
	sequ_hdr_info->secure_sequence_info = aux_secure_sequence_info_hndl;
}

/*
 * @Function	bspp_reset_pps
 *
 */
static void bspp_reset_pps(struct bspp_str_context *str_ctx,
			   struct bspp_pps_info *pps_info)
{
	/* Temporarily store relevant PPS fields. */
	struct bspp_ddbuf_array_info aux_fw_pps = pps_info->fw_pps;
	void *aux_secure_pps_info_hndl = pps_info->secure_pps_info;
	struct bspp_ddbuf_array_info *tmp = &pps_info->fw_pps;

	/* Reset all related structures. */
	memset(((unsigned char *)tmp->ddbuf_info.cpu_virt_addr + tmp->buf_offset), 0x00,
	       pps_info->fw_pps.buf_element_size);

	/* Reset the parser specific data. */
	if (str_ctx->parser_callbacks.reset_data_cb)
		str_ctx->parser_callbacks.reset_data_cb(BSPP_UNIT_PPS, pps_info->secure_pps_info);

	/* Reset the common data. */
	memset(pps_info, 0, sizeof(*pps_info));

	/* Restore relevant PPS fields. */
	pps_info->fw_pps = aux_fw_pps;
	pps_info->bufmap_id = aux_fw_pps.ddbuf_info.bufmap_id;
	pps_info->buf_offset = aux_fw_pps.buf_offset;
	pps_info->secure_pps_info = aux_secure_pps_info_hndl;
}

/*
 * @Function	bspp_stream_submit_buffer
 *
 */
int bspp_stream_submit_buffer(void *str_context_handle,
			      const struct bspp_ddbuf_info *ddbuf_info,
			      unsigned int bufmap_id,
			      unsigned int data_size,
			      void *pict_tag_param,
			      enum vdec_bstr_element_type bstr_element_type)
{
	struct bspp_str_context *str_ctx = (struct bspp_str_context *)str_context_handle;
	struct bspp_bitstream_buffer *bstr_buf;
	unsigned int result = IMG_SUCCESS;

	if (!str_context_handle) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error;
	}

	if (bstr_element_type == VDEC_BSTRELEMENT_UNDEFINED ||
	    bstr_element_type >= VDEC_BSTRELEMENT_MAX) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error;
	}

	/*
	 * Check that the new bitstream buffer is compatible with those
	 * before.
	 */
	bstr_buf = lst_last(&str_ctx->grp_bstr_ctx.buffer_chain);
	if (bstr_buf && bstr_buf->bstr_element_type != bstr_element_type) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error;
	}

	/* Allocate a bitstream buffer chain element structure */
	bstr_buf = kmalloc(sizeof(*bstr_buf), GFP_KERNEL);
	if (!bstr_buf) {
		result = IMG_ERROR_OUT_OF_MEMORY;
		goto error;
	}
	memset(bstr_buf, 0, sizeof(*bstr_buf));

	/* Queue buffer in a chain since units might span buffers. */
	if (ddbuf_info)
		bstr_buf->ddbuf_info = *ddbuf_info;

	bstr_buf->data_size = data_size;
	bstr_buf->bstr_element_type = bstr_element_type;
	bstr_buf->pict_tag_param = pict_tag_param;
	bstr_buf->bufmap_id = bufmap_id;
	lst_add(&str_ctx->grp_bstr_ctx.buffer_chain, bstr_buf);

	str_ctx->grp_bstr_ctx.total_data_size += data_size;

error:
	return result;
}

/*
 * @Function	bspp_sequence_hdr_info
 *
 */
static struct bspp_sequence_hdr_info *bspp_obtain_sequence_hdr(struct bspp_str_context *str_ctx)
{
	struct bspp_stream_alloc_data *str_alloc = &str_ctx->str_alloc;
	struct bspp_sequence_hdr_info *sequ_hdr_info;

	/*
	 * Obtain any partially filled sequence data else provide a new one
	 * (always new for H.264 and HEVC)
	 */
	sequ_hdr_info = lst_last(&str_alloc->sequence_data_list[BSPP_DEFAULT_SEQUENCE_ID]);
	if (!sequ_hdr_info || sequ_hdr_info->ref_count > 0 || str_ctx->vid_std == VDEC_STD_H264 ||
	    str_ctx->vid_std == VDEC_STD_HEVC) {
		/* Get Sequence resource. */
		sequ_hdr_info = lst_removehead(&str_alloc->available_sequence_list);
		if (sequ_hdr_info) {
			bspp_reset_sequence(str_ctx, sequ_hdr_info);
			sequ_hdr_info->sequ_hdr_info.sequ_hdr_id = BSPP_INVALID;
		}
	}

	return sequ_hdr_info;
}

/*
 * @Function	bspp_submit_picture_decoded
 *
 */
int bspp_submit_picture_decoded(void *str_context_handle,
				struct bspp_picture_decoded *picture_decoded)
{
	struct bspp_picture_decoded *picture_decoded_elem;
	struct bspp_str_context *str_ctx = (struct bspp_str_context *)str_context_handle;

	/* Validate input arguments. */
	if (!str_context_handle)
		return IMG_ERROR_INVALID_PARAMETERS;

	picture_decoded_elem = kmalloc(sizeof(*picture_decoded_elem), GFP_KERNEL);
	if (!picture_decoded_elem)
		return IMG_ERROR_MALLOC_FAILED;

	*picture_decoded_elem = *picture_decoded;

	/* Lock access to the list for adding a picture - HIGH PRIORITY */
	mutex_lock_nested(str_ctx->bspp_mutex, SUBCLASS_BSPP);

	lst_add(&str_ctx->decoded_pictures_list, picture_decoded_elem);

	/* Unlock access to the list for adding a picture - HIGH PRIORITY */
	mutex_unlock(str_ctx->bspp_mutex);

	return IMG_SUCCESS;
}

/*
 * @Function	bspp_check_and_detach_pps_info
 *
 */
static void bspp_check_and_detach_pps_info(struct bspp_stream_alloc_data *str_alloc,
					   unsigned int pps_id)
{
	if (pps_id != BSPP_INVALID) {
		struct bspp_pps_info *pps_info = lst_first(&str_alloc->pps_data_list[pps_id]);

		if (!pps_info) /* Invalid id */
			return;

		pps_info->ref_count--;
		/* If nothing references it any more */
		if (pps_info->ref_count == 0) {
			struct bspp_pps_info *next_pps_info = lst_next(pps_info);

			/*
			 * If it is not the last sequence in the slot list
			 * remove it and return it to the pool-list
			 */
			if (next_pps_info) {
				lst_remove(&str_alloc->pps_data_list[pps_id], pps_info);
				lst_addhead(&str_alloc->available_ppss_list, pps_info);
			}
		}
	}
}

/*
 * @Function	bspp_picture_decoded
 *
 */
static int bspp_picture_decoded(struct bspp_str_context *str_ctx,
				struct bspp_picture_decoded *picture_decoded)
{
	struct bspp_stream_alloc_data *str_alloc = &str_ctx->str_alloc;

	/* Manage Sequence */
	if (picture_decoded->sequ_hdr_id != BSPP_INVALID) {
		struct bspp_sequence_hdr_info *seq =
			lst_first(&str_alloc->sequence_data_list[picture_decoded->sequ_hdr_id]);

		if (!seq)
			return IMG_ERROR_INVALID_ID;

		if (picture_decoded->not_decoded) {
			/* Release sequence data. */
			if (str_ctx->parser_callbacks.release_data_cb)
				str_ctx->parser_callbacks.release_data_cb((void *)str_alloc,
					BSPP_UNIT_SEQUENCE, seq->secure_sequence_info);
		}

		seq->ref_count--;
		/* If nothing references it any more */
		if (seq->ref_count == 0) {
			struct bspp_sequence_hdr_info *next_sequ_hdr_info = lst_next(seq);

			/*
			 * If it is not the last sequence in the slot list
			 * remove it and return it to the pool-list
			 */
			if (next_sequ_hdr_info) {
				lst_remove(&str_alloc->sequence_data_list
					   [picture_decoded->sequ_hdr_id], seq);
				/* Release sequence data. */
				if (str_ctx->parser_callbacks.release_data_cb)
					str_ctx->parser_callbacks.release_data_cb((void *)str_alloc,
						BSPP_UNIT_SEQUENCE, seq->secure_sequence_info);

				lst_addhead(&str_alloc->available_sequence_list, seq);
			}
		}
	}

	/*
	 * Expect at least one valid PPS for H.264 and always invalid for all
	 * others
	 */
	bspp_check_and_detach_pps_info(str_alloc, picture_decoded->pps_id);
	bspp_check_and_detach_pps_info(str_alloc, picture_decoded->second_pps_id);

	return IMG_SUCCESS;
}

/*
 * @Function	bspp_service_pictures_decoded
 *
 */
static int bspp_service_pictures_decoded(struct bspp_str_context *str_ctx)
{
	struct bspp_picture_decoded *picture_decoded;

	while (1) {
		/*
		 * Lock access to the list for removing a picture -
		 * LOW PRIORITY
		 */
		mutex_lock_nested(str_ctx->bspp_mutex, SUBCLASS_BSPP);

		picture_decoded = lst_removehead(&str_ctx->decoded_pictures_list);

		/*
		 * Unlock access to the list for removing a picture -
		 * LOW PRIORITY
		 */
		mutex_unlock(str_ctx->bspp_mutex);

		if (!picture_decoded)
			break;

		bspp_picture_decoded(str_ctx, picture_decoded);
		kfree(picture_decoded);
	}

	return IMG_SUCCESS;
}

static void bspp_remove_unused_vps(struct bspp_str_context *str_ctx, unsigned int vps_id)
{
	struct bspp_stream_alloc_data *str_alloc = &str_ctx->str_alloc;
	struct bspp_vps_info *temp_vps_info = NULL;
	struct bspp_vps_info *next_temp_vps_info = NULL;

	/*
	 * Check the whole Vps slot list for any unused Vpss
	 * BEFORE ADDING THE NEW ONE, if found remove them
	 */
	next_temp_vps_info = lst_first(&str_alloc->vps_data_list[vps_id]);
	while (next_temp_vps_info) {
		/* Set Temp, it is the one which we will potentially remove */
		temp_vps_info = next_temp_vps_info;
		/*
		 *  Set Next Temp, it is the one for the next iteration
		 * (we cannot ask for next after removing it)
		 */
		next_temp_vps_info = lst_next(temp_vps_info);
		/* If it is not used remove it */
		if (temp_vps_info->ref_count == 0 && next_temp_vps_info) {
			/* Return resource to the available pool */
			lst_remove(&str_alloc->vps_data_list[vps_id], temp_vps_info);
			lst_addhead(&str_alloc->available_vps_list, temp_vps_info);
		}
	}
}

static void bspp_remove_unused_pps(struct bspp_str_context *str_ctx, unsigned int pps_id)
{
	struct bspp_stream_alloc_data *str_alloc = &str_ctx->str_alloc;
	struct bspp_pps_info *temp_pps_info = NULL;
	struct bspp_pps_info *next_temp_pps_info = NULL;

	/*
	 * Check the whole PPS slot list for any unused PPSs BEFORE ADDING
	 * THE NEW ONE, if found remove them
	 */
	next_temp_pps_info = lst_first(&str_alloc->pps_data_list[pps_id]);
	while (next_temp_pps_info) {
		/* Set Temp, it is the one which we will potentially remove */
		temp_pps_info = next_temp_pps_info;
		/*
		 * Set Next Temp, it is the one for the next iteration
		 * (we cannot ask for next after removing it)
		 */
		next_temp_pps_info = lst_next(temp_pps_info);
		/* If it is not used remove it */
		if (temp_pps_info->ref_count == 0 && next_temp_pps_info) {
			/* Return resource to the available pool */
			lst_remove(&str_alloc->pps_data_list[pps_id], temp_pps_info);
			lst_addhead(&str_alloc->available_ppss_list, temp_pps_info);
		}
	}
}

static void bspp_remove_unused_sequence(struct bspp_str_context *str_ctx, unsigned int sps_id)
{
	struct bspp_stream_alloc_data *str_alloc = &str_ctx->str_alloc;
	struct bspp_sequence_hdr_info *seq = NULL;
	struct bspp_sequence_hdr_info *next_seq = NULL;

	/*
	 * Check the whole sequence slot list for any unused sequences,
	 * if found remove them
	 */
	next_seq = lst_first(&str_alloc->sequence_data_list[sps_id]);
	while (next_seq) {
		/* Set Temp, it is the one which we will potentially remove */
		seq = next_seq;
		/*
		 * Set Next Temp, it is the one for the next iteration (we
		 * cannot ask for next after removing it)
		 */
		next_seq = lst_next(seq);

		/*
		 * If the head is no longer used and there is something after,
		 * remove it
		 */
		if (seq->ref_count == 0 && next_seq) {
			/* Return resource to the pool-list */
			lst_remove(&str_alloc->sequence_data_list[sps_id], seq);
			if (str_ctx->parser_callbacks.release_data_cb) {
				str_ctx->parser_callbacks.release_data_cb
							((void *)str_alloc,
							 BSPP_UNIT_SEQUENCE,
							 seq->secure_sequence_info);
			}
			lst_addhead(&str_alloc->available_sequence_list, seq);
		}
	}
}

/*
 * @Function	bspp_return_or_store_sequence_hdr
 *
 */
static int bspp_return_or_store_sequence_hdr(struct bspp_str_context *str_ctx,
					     enum bspp_error_type parse_error,
					     struct bspp_sequence_hdr_info *sequ_hdr_info)
{
	struct bspp_stream_alloc_data *str_alloc = &str_ctx->str_alloc;
	struct bspp_sequence_hdr_info *prev_sequ_hdr_info;

	if (((parse_error & BSPP_ERROR_UNRECOVERABLE) || (parse_error & BSPP_ERROR_UNSUPPORTED)) &&
	    sequ_hdr_info->sequ_hdr_info.sequ_hdr_id != BSPP_INVALID) {
		prev_sequ_hdr_info =
			lst_last(&str_alloc->sequence_data_list
					[sequ_hdr_info->sequ_hdr_info.sequ_hdr_id]);

		/* check if it's not the same pointer */
		if (prev_sequ_hdr_info && prev_sequ_hdr_info != sequ_hdr_info) {
			/*
			 * Throw away corrupted sequence header if a previous "good" one exists.
			 */
			sequ_hdr_info->sequ_hdr_info.sequ_hdr_id = BSPP_INVALID;
		}
	}

	/* Store or return Sequence resource. */
	if (sequ_hdr_info->sequ_hdr_info.sequ_hdr_id != BSPP_INVALID) {
		/* Only add when not already in list. */
		if (sequ_hdr_info != lst_last(&str_alloc->sequence_data_list
				[sequ_hdr_info->sequ_hdr_info.sequ_hdr_id])) {
			/*
			 * Add new sequence header (not already in list) to end
			 * of the slot-list.
			 */
			lst_add(&str_alloc->sequence_data_list
				[sequ_hdr_info->sequ_hdr_info.sequ_hdr_id], sequ_hdr_info);
		}

		bspp_remove_unused_sequence(str_ctx, sequ_hdr_info->sequ_hdr_info.sequ_hdr_id);
	} else {
		/*
		 * if unit was not a sequnce info, add resource to the
		 * pool-list
		 */
		lst_addhead(&str_alloc->available_sequence_list, sequ_hdr_info);
	}

	return IMG_SUCCESS;
}

/*
 * @Function	bspp_get_resource
 *
 */
static int bspp_get_resource(struct bspp_str_context *str_ctx,
			     struct bspp_pict_hdr_info *pict_hdr_info,
			     struct bspp_unit_data *unit_data)
{
	int result = IMG_SUCCESS;
	struct bspp_stream_alloc_data *str_alloc = &str_ctx->str_alloc;

	switch (unit_data->unit_type) {
	case BSPP_UNIT_VPS:
		/* Get VPS resource (HEVC only). */
		if (unit_data->vid_std != VDEC_STD_HEVC)
			break;
		unit_data->out.vps_info = lst_removehead(&str_alloc->available_vps_list);
		if (!unit_data->out.vps_info) {
			result = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
		} else {
			unit_data->out.vps_info->vps_id = BSPP_INVALID;
			unit_data->out.vps_info->ref_count = 0;
		}
		break;
	case BSPP_UNIT_SEQUENCE:
		unit_data->out.sequ_hdr_info = bspp_obtain_sequence_hdr(str_ctx);
		if (!unit_data->out.sequ_hdr_info)
			result = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;

		break;

	case BSPP_UNIT_PPS:
		/* Get PPS resource (H.264 only). */
		unit_data->out.pps_info = lst_removehead(&str_alloc->available_ppss_list);
		/* allocate and return extra resources */
		if (!unit_data->out.pps_info) {
			result = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
		} else {
			bspp_reset_pps(str_ctx, unit_data->out.pps_info);
			unit_data->out.pps_info->pps_id = BSPP_INVALID;
		}
		break;

	case BSPP_UNIT_PICTURE:
	case BSPP_UNIT_SKIP_PICTURE:
		unit_data->out.pict_hdr_info = pict_hdr_info;
#ifdef HAS_JPEG
		if (unit_data->vid_std == VDEC_STD_JPEG) {
			unit_data->impl_sequ_hdr_info = bspp_obtain_sequence_hdr(str_ctx);
			if (!unit_data->impl_sequ_hdr_info)
				result = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
		}
#endif
		break;

	default:
		break;
	}

	return result;
}

/*
 * @Function	bspp_file_resource
 * @Description	Stores or returns all resources provided to parse unit.
 */
static int bspp_file_resource(struct bspp_str_context *str_ctx, struct bspp_unit_data *unit_data)
{
	unsigned int result = IMG_SUCCESS;
	struct bspp_stream_alloc_data *str_alloc = &str_ctx->str_alloc;

	switch (unit_data->unit_type) {
	case BSPP_UNIT_VPS:
		/* Store or return VPS resource (HEVC only) */
		if (unit_data->vid_std != VDEC_STD_HEVC)
			break;

		if (unit_data->out.vps_info->vps_id != BSPP_INVALID) {
			lst_add(&str_alloc->vps_data_list[unit_data->out.vps_info->vps_id],
				unit_data->out.vps_info);

			bspp_remove_unused_vps(str_ctx, unit_data->out.vps_info->vps_id);
		} else {
			lst_addhead(&str_alloc->available_vps_list, unit_data->out.vps_info);
		}
		break;
	case BSPP_UNIT_SEQUENCE:
		result = bspp_return_or_store_sequence_hdr(str_ctx, unit_data->parse_error,
							   unit_data->out.sequ_hdr_info);
		VDEC_ASSERT(result == IMG_SUCCESS);
		break;

	case BSPP_UNIT_PPS:
		/* Store or return PPS resource (H.264 only). */
		if (unit_data->out.pps_info->pps_id != BSPP_INVALID) {
			/*
			 * if unit was a PPS info, add resource to the slot-list
			 * AFTER REMOVING THE UNUSED ONES otherwise this will be
			 * removed along the rest unless special provision for
			 * last is made
			 */
			lst_add(&str_alloc->pps_data_list[unit_data->out.pps_info->pps_id],
				unit_data->out.pps_info);

			bspp_remove_unused_pps(str_ctx, unit_data->out.pps_info->pps_id);
		} else {
			/*
			 * if unit was not a PPS info, add resource to the
			 * pool-list
			 */
			lst_addhead(&str_alloc->available_ppss_list, unit_data->out.pps_info);
		}
		break;

	case BSPP_UNIT_PICTURE:
	case BSPP_UNIT_SKIP_PICTURE:
#ifdef HAS_JPEG
		if (unit_data->vid_std == VDEC_STD_JPEG) {
			result = bspp_return_or_store_sequence_hdr(str_ctx,
								   unit_data->parse_error,
								   unit_data->impl_sequ_hdr_info);
			VDEC_ASSERT(result == IMG_SUCCESS);
		}
#endif
		break;

	default:
		break;
	}

	return result;
}

/*
 * @Function	bspp_process_unit
 *
 */
static int bspp_process_unit(struct bspp_str_context *str_ctx,
			     unsigned int size_delim_bits,
			     struct bspp_pict_ctx *pict_ctx,
			     struct bspp_parse_state *parse_state)
{
	struct bspp_unit_data unit_data;
	unsigned long long unit_size = 0; /* Unit size (in bytes, size delimited only). */
	unsigned int result;
	unsigned char vidx = str_ctx->grp_bstr_ctx.current_view_idx;
	struct bspp_pict_hdr_info *curr_pict_hdr_info;

	/*
	 * during call to swsr_consume_delim(), above.
	 * Setup default unit data.
	 */
	memset(&unit_data, 0, sizeof(struct bspp_unit_data));

	if (str_ctx->grp_bstr_ctx.delim_present) {
		/* Consume delimiter and catch any exceptions. */
		/*
		 * Consume the bitstream unit delimiter (size or
		 * start code prefix).
		 * When size-delimited the unit size is also returned
		 * so that the next unit can be found.
		 */
		result = swsr_consume_delim(str_ctx->swsr_ctx.swsr_context,
					    str_ctx->swsr_ctx.emulation_prevention,
					    size_delim_bits, &unit_size);
		if (result != IMG_SUCCESS)
			goto error;
	}

	unit_data.unit_type = str_ctx->grp_bstr_ctx.unit_type;
	unit_data.vid_std = str_ctx->vid_std;
	unit_data.delim_present = str_ctx->grp_bstr_ctx.delim_present;
	unit_data.codec_config = &str_ctx->codec_config;
	unit_data.parse_state = parse_state;
	unit_data.pict_sequ_hdr_id = str_ctx->sequ_hdr_id;
	unit_data.str_res_handle = &str_ctx->str_alloc;
	unit_data.unit_data_size = str_ctx->grp_bstr_ctx.total_data_size;
	unit_data.intra_frm_as_closed_gop = str_ctx->intra_frame_closed_gop;

	/* ponit to picture headers, check boundaries */
	curr_pict_hdr_info = vidx < VDEC_H264_MVC_MAX_VIEWS ?
		&pict_ctx->pict_hdr_info[vidx] : NULL;
	unit_data.parse_state->next_pict_hdr_info =
		vidx + 1 < VDEC_H264_MVC_MAX_VIEWS ?
		&pict_ctx->pict_hdr_info[vidx + 1] : NULL;
	unit_data.parse_state->is_prefix = 0;

	/* Obtain output data containers. */
	result = bspp_get_resource(str_ctx, curr_pict_hdr_info, &unit_data);
	if (result != IMG_SUCCESS)
		return result;

	/* Process Unit and catch any exceptions. */
	/*
	 * Call the standard-specific function to parse the bitstream
	 * unit.
	 */
	result = str_ctx->parser_callbacks.parse_unit_cb(str_ctx->swsr_ctx.swsr_context,
			&unit_data);
	if (result != IMG_SUCCESS) {
		pr_err("Failed to process unit, error = %d", unit_data.parse_error);
		goto error;
	}

	if (unit_data.parse_error != BSPP_ERROR_NONE)
		pr_err("Issues found while processing unit, error = %d\n", unit_data.parse_error);

	/* Store or return resource used for parsing unit. */
	result = bspp_file_resource(str_ctx, &unit_data);

	if (!str_ctx->inter_pict_data.seen_closed_gop &&
	    str_ctx->grp_bstr_ctx.unit_type == BSPP_UNIT_PICTURE &&
		unit_data.slice &&
		(unit_data.out.pict_hdr_info &&
		unit_data.out.pict_hdr_info->intra_coded) &&
		str_ctx->vid_std != VDEC_STD_H264)
		unit_data.new_closed_gop = 1;

	if (unit_data.new_closed_gop) {
		str_ctx->inter_pict_data.seen_closed_gop = 1;
		str_ctx->inter_pict_data.new_closed_gop = 1;
	}

	/*
	 * Post-process unit (use local context in case
	 * parse function tried to change the unit type.
	 */
	if (str_ctx->grp_bstr_ctx.unit_type == BSPP_UNIT_PICTURE ||
	    str_ctx->grp_bstr_ctx.unit_type == BSPP_UNIT_SKIP_PICTURE) {
		if (str_ctx->inter_pict_data.new_closed_gop) {
			pict_ctx->closed_gop = 1;
			str_ctx->inter_pict_data.new_closed_gop = 0;
		}

		if (unit_data.ext_slice && str_ctx->grp_bstr_ctx.not_ext_pic_unit_yet &&
		    unit_data.pict_sequ_hdr_id != BSPP_INVALID) {
			unsigned int id = unit_data.pict_sequ_hdr_id;

			str_ctx->grp_bstr_ctx.not_ext_pic_unit_yet = 0;
			pict_ctx->ext_sequ_hdr_info =
				lst_last(&str_ctx->str_alloc.sequence_data_list[id]);
		}

		if (unit_data.slice) {
			if (!curr_pict_hdr_info) {
				VDEC_ASSERT(0);
				return -EINVAL;
			}
			if (str_ctx->grp_bstr_ctx.not_pic_unit_yet &&
			    unit_data.pict_sequ_hdr_id != BSPP_INVALID) {
				str_ctx->grp_bstr_ctx.not_pic_unit_yet = 0;

				/*
				 * depend upon the picture header being
				 * populated (in addition to slice data).
				 */
				pict_ctx->present = 1;

				/*
				 * Update the picture context from the last unit parsed.
				 * This context must be stored since a non-picture unit may follow.
				 * Obtain current instance of sequence data for given ID.
				 */
				if (!pict_ctx->sequ_hdr_info) {
					unsigned int id = unit_data.pict_sequ_hdr_id;

					pict_ctx->sequ_hdr_info =
					lst_last(&str_ctx->str_alloc.sequence_data_list[id]);

					/* Do the sequence flagging/reference-counting */
					pict_ctx->sequ_hdr_info->ref_count++;
				}

				/* Override the field here. */
				if (str_ctx->swsr_ctx.sr_config.delim_type == SWSR_DELIM_NONE) {
					if (str_ctx->grp_bstr_ctx.unit_type ==
						BSPP_UNIT_SKIP_PICTURE) {
						/* VDECFW_SKIPPED_PICTURE; */
						curr_pict_hdr_info->parser_mode =
							VDECFW_SKIPPED_PICTURE;
						curr_pict_hdr_info->pic_data_size = 0;
					} else {
						/* VDECFW_SIZE_SIDEBAND; */
						curr_pict_hdr_info->parser_mode =
							VDECFW_SIZE_SIDEBAND;
						curr_pict_hdr_info->pic_data_size =
							str_ctx->grp_bstr_ctx.total_data_size;
					}
				} else if (str_ctx->swsr_ctx.sr_config.delim_type ==
					   SWSR_DELIM_SIZE) {
					if (str_ctx->swsr_ctx.sr_config.delim_length <= 8)
						/* VDECFW_SIZE_DELIMITED_1_ONLY; */
						curr_pict_hdr_info->parser_mode =
							VDECFW_SIZE_DELIMITED_1_ONLY;
					else if (str_ctx->swsr_ctx.sr_config.delim_length <= 16)
						/* VDECFW_SIZE_DELIMITED_2_ONLY; */
						curr_pict_hdr_info->parser_mode =
							VDECFW_SIZE_DELIMITED_2_ONLY;
					else if (str_ctx->swsr_ctx.sr_config.delim_length <= 32)
						/* VDECFW_SIZE_DELIMITED_4_ONLY; */
						curr_pict_hdr_info->parser_mode =
							VDECFW_SIZE_DELIMITED_4_ONLY;

					curr_pict_hdr_info->pic_data_size +=
						((unsigned int)unit_size
						+ (size_delim_bits / 8));
				} else if (str_ctx->swsr_ctx.sr_config.delim_type == SWSR_DELIM_SCP)
					/* VDECFW_SCP_ONLY; */
					curr_pict_hdr_info->parser_mode = VDECFW_SCP_ONLY;
			}

			/*
			 * for MVC, the Slice Extension should also have the
			 * same ParserMode as the Base view.
			 */
			if (unit_data.parse_state->next_pict_hdr_info) {
				unit_data.parse_state->next_pict_hdr_info->parser_mode =
					curr_pict_hdr_info->parser_mode;
			}

			if (unit_data.parse_error & BSPP_ERROR_UNSUPPORTED) {
				pict_ctx->invalid = 1;
				pict_ctx->unsupported = 1;
			} else if (!str_ctx->full_scan) {
				/*
				 * Only parse up to and including the first
				 * valid video slice unless full scanning.
				 */
				pict_ctx->finished = 1;
			}
		}
	}

	if (unit_data.extracted_all_data) {
		enum swsr_found found;

		swsr_byte_align(str_ctx->swsr_ctx.swsr_context);

		found = swsr_check_delim_or_eod(str_ctx->swsr_ctx.swsr_context);
		if (found != SWSR_FOUND_DELIM && found != SWSR_FOUND_EOD) {
			/*
			 * Should already be at the next delimiter or EOD.
			 * Any bits left at the end of the unit could indicate
			 * corrupted syntax or erroneous parsing.
			 */
		}
	}

	return IMG_SUCCESS;

error:
	if (unit_data.unit_type == BSPP_UNIT_PICTURE ||
	    unit_data.unit_type == BSPP_UNIT_SKIP_PICTURE)
		pict_ctx->invalid = 1;

	/*
	 * Tidy-up resources.
	 * Store or return resource used for parsing unit.
	 */
	bspp_file_resource(str_ctx, &unit_data);

	return result;
}

/*
 * @Function	bspp_terminate_buffer
 *
 */
static int bspp_terminate_buffer(struct bspp_grp_bstr_ctx *grp_btsr_ctx,
				 struct bspp_bitstream_buffer *buf)
{
	int result = -1;

	/* Indicate that all the data in buffer should be added to segment. */
	buf->bytes_read = buf->data_size;

	result = bspp_create_segment(grp_btsr_ctx, buf);
	if (result != IMG_SUCCESS)
		return result;

	/* Next segment will start at the beginning of the next buffer. */
	grp_btsr_ctx->segment_offset = 0;

	bspp_free_bitstream_elem(buf);

	return result;
}

/*
 * @Function	bspp_jump_to_next_view
 *
 */
static int bspp_jump_to_next_view(struct bspp_grp_bstr_ctx *grp_btsr_ctx,
				  struct bspp_preparsed_data *preparsed_data,
				  struct bspp_parse_state *parse_state)
{
	struct bspp_bitstream_buffer *cur_buf;
	int result;
	unsigned int i;
	unsigned char vidx;

	if (!grp_btsr_ctx || !parse_state || !preparsed_data) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error;
	}

	vidx = grp_btsr_ctx->current_view_idx;

	if (vidx >= VDEC_H264_MVC_MAX_VIEWS) {
		result = IMG_ERROR_NOT_SUPPORTED;
		goto error;
	}

	/* get current buffer */
	cur_buf = (struct bspp_bitstream_buffer *)lst_first(&grp_btsr_ctx->in_flight_bufs);
	if (!cur_buf) {
		result = IMG_ERROR_CANCELLED;
		goto error;
	}

	if (cur_buf->bufmap_id != parse_state->prev_buf_map_id) {
		/*
		 * If we moved to the next buffer while parsing the slice
		 * header of the new view we have to reduce the size of
		 * the last segment up to the beginning of the new view slice
		 * and create a new segment from that point up to the end of
		 * the buffer. The new segment should belong to the new view.
		 * THIS ONLY WORKS IF THE SLICE HEADER DOES NOT SPAN MORE THAN
		 * TWO BUFFERS. If we want to support the case that the slice
		 * header of the new view spans multiple buffer we either have
		 * here remove all the segments up to the point were we find
		 * the buffer we are looking for, then adjust the size of this
		 * segment and then add the segments we removed to the next
		 * view list or we can implement a mechanism like the one that
		 * peeks for the NAL unit type and delimit the next view
		 * segment before parsing the first slice of the view.
		 */
		struct bspp_bitstr_seg *segment;

		segment = lst_last(grp_btsr_ctx->segment_list);
		if (segment && segment->bufmap_id == parse_state->prev_buf_map_id) {
			struct bspp_bitstream_buffer prev_buf;

			segment->data_size -= parse_state->prev_buf_data_size
				- parse_state->prev_byte_offset_buf;
			segment->bstr_seg_flag &= ~VDECDD_BSSEG_LASTINBUFF;

			/*
			 * Change the segmenOffset value with the value it
			 * would have if we had delemited the segment correctly
			 * beforehand.
			 */
			grp_btsr_ctx->segment_offset = parse_state->prev_byte_offset_buf;

			/* set lists of segments to new view... */
			for (i = 0; i < BSPP_MAX_PICTURES_PER_BUFFER; i++) {
				grp_btsr_ctx->pre_pict_seg_list[i] =
					&preparsed_data->ext_pictures_data[vidx].pre_pict_seg_list
					[i];
				grp_btsr_ctx->pict_seg_list[i] =
					&preparsed_data->ext_pictures_data[vidx].pict_seg_list[i];

				lst_init(grp_btsr_ctx->pre_pict_seg_list[i]);
				lst_init(grp_btsr_ctx->pict_seg_list[i]);
			}
			/* and current segment list */
			grp_btsr_ctx->segment_list = grp_btsr_ctx->pict_seg_list[0];

			memset(&prev_buf, 0, sizeof(struct bspp_bitstream_buffer));
			prev_buf.bufmap_id = segment->bufmap_id;
			prev_buf.data_size = parse_state->prev_buf_data_size;
			prev_buf.bytes_read = prev_buf.data_size;

			/* Create the segment the first part of the next view */
			result = bspp_create_segment(grp_btsr_ctx, &prev_buf);
			if (result != IMG_SUCCESS)
				goto error;
		} else {
			result = IMG_ERROR_NOT_SUPPORTED;
			goto error;
		}
	} else {
		/*
		 * the data just parsed belongs to new view, so use previous byte
		 * offset
		 */
		cur_buf->bytes_read = parse_state->prev_byte_offset_buf;

		/* Create the segment for previous view */
		result = bspp_create_segment(grp_btsr_ctx, cur_buf);
		if (result != IMG_SUCCESS)
			goto error;

		/* set lists of segments to new view */
		for (i = 0; i < BSPP_MAX_PICTURES_PER_BUFFER; i++) {
			grp_btsr_ctx->pre_pict_seg_list[i] =
				&preparsed_data->ext_pictures_data[vidx].pre_pict_seg_list[i];
			grp_btsr_ctx->pict_seg_list[i] =
				&preparsed_data->ext_pictures_data[vidx].pict_seg_list[i];

			lst_init(grp_btsr_ctx->pre_pict_seg_list[i]);
			lst_init(grp_btsr_ctx->pict_seg_list[i]);
		}
		/* and current segment list */
		grp_btsr_ctx->segment_list = grp_btsr_ctx->pict_seg_list[0];
	}

	/* update prefix flag */
	preparsed_data->ext_pictures_data[vidx].is_prefix = parse_state->is_prefix;
	/* and view index */
	grp_btsr_ctx->current_view_idx++;

	/* set number of extended pictures */
	preparsed_data->num_ext_pictures = grp_btsr_ctx->current_view_idx;

error:
	return result;
}

static void bspp_reset_pict_state(struct bspp_str_context *str_ctx, struct bspp_pict_ctx *pict_ctx,
				  struct bspp_parse_state *parse_state)
{
	memset(pict_ctx, 0, sizeof(struct bspp_pict_ctx));
	memset(parse_state, 0, sizeof(struct bspp_parse_state));

	/* Setup group buffer processing state. */
	parse_state->inter_pict_ctx = &str_ctx->inter_pict_data;
	parse_state->prev_bottom_pic_flag = (unsigned char)BSPP_INVALID;
	parse_state->next_pic_is_new = 1;
	parse_state->prev_frame_num = BSPP_INVALID;
	parse_state->second_field_flag = 0;
	parse_state->first_chunk = 1;
}

/*
 * @Function	bspp_stream_preparse_buffers
 * @Description	Buffer list cannot be processed since units in this last buffer
 * may not be complete. Must wait until a buffer is provided with end-of-picture
 * signalled. When the buffer indicates that units won't span then we can
 * process the bitstream buffer chain.
 */
int bspp_stream_preparse_buffers(void *str_context_handle,
				 const struct bspp_ddbuf_info *contig_buf_info,
				 unsigned int contig_buf_map_id, struct lst_t *segments,
				 struct bspp_preparsed_data *preparsed_data,
				 int end_of_pic)
{
	struct bspp_str_context *str_ctx = (struct bspp_str_context *)str_context_handle;
	struct bspp_pict_ctx *pict_ctx = &str_ctx->pict_ctx;
	struct bspp_parse_state *parse_state = &str_ctx->parse_state;
	int i;
	unsigned int unit_count = 0, num_arrays = 0;
	unsigned int size_delim_bits = 0;
	enum swsr_found found = SWSR_FOUND_NONE;
	unsigned int result;
	struct bspp_bitstr_seg *segment;
	struct lst_t temp_list;

	/*
	 * since it is new picture, resetting the context status to
	 * beginning
	 */
	/* TODO: revisit this */
	pict_ctx->finished = 0;
	pict_ctx->new_pict_signalled = 0;

	if (!str_context_handle)
		return IMG_ERROR_INVALID_PARAMETERS;

	if (!segments || !preparsed_data)
		return IMG_ERROR_INVALID_PARAMETERS;

	/* Check that bitstream buffers have been registered. */
	if (!lst_last(&str_ctx->grp_bstr_ctx.buffer_chain))
		return IMG_ERROR_OPERATION_PROHIBITED;

	/* Initialise the output data. */
	memset(preparsed_data, 0, sizeof(struct bspp_preparsed_data));

	if (!parse_state->initialised) {
		bspp_reset_pict_state(str_ctx, pict_ctx, parse_state);
		parse_state->initialised = 1;
	}

	for (i = 0; i < 3; i++) {
		lst_init(&preparsed_data->picture_data.pre_pict_seg_list[i]);
		lst_init(&preparsed_data->picture_data.pict_seg_list[i]);
	}

	/* Initialise parsing for this video standard. */
	if (str_ctx->parser_callbacks.initialise_parsing_cb && parse_state->first_chunk)
		str_ctx->parser_callbacks.initialise_parsing_cb(parse_state);

	parse_state->first_chunk = 0;

	for (i = 0; i < VDEC_H264_MVC_MAX_VIEWS; i++) {
		pict_ctx->pict_hdr_info[i].pict_aux_data.id = BSPP_INVALID;
		pict_ctx->pict_hdr_info[i].second_pict_aux_data.id = BSPP_INVALID;
	}

	/* Setup buffer group bitstream context. */
	str_ctx->grp_bstr_ctx.vid_std = str_ctx->vid_std;
	str_ctx->grp_bstr_ctx.disable_mvc = str_ctx->disable_mvc;
	str_ctx->grp_bstr_ctx.delim_present = 1;
	str_ctx->grp_bstr_ctx.swsr_context = str_ctx->swsr_ctx.swsr_context;
	str_ctx->grp_bstr_ctx.unit_type = BSPP_UNIT_NONE;
	str_ctx->grp_bstr_ctx.last_unit_type = BSPP_UNIT_NONE;
	str_ctx->grp_bstr_ctx.not_pic_unit_yet = 1;
	str_ctx->grp_bstr_ctx.not_ext_pic_unit_yet = 1;
	str_ctx->grp_bstr_ctx.total_bytes_read = 0;
	str_ctx->grp_bstr_ctx.current_view_idx = 0;

	for (i = 0; i < 3; i++) {
		str_ctx->grp_bstr_ctx.pre_pict_seg_list[i] =
			&preparsed_data->picture_data.pre_pict_seg_list[i];
		str_ctx->grp_bstr_ctx.pict_seg_list[i] =
			&preparsed_data->picture_data.pict_seg_list[i];
		str_ctx->grp_bstr_ctx.pict_tag_param_array[i] =
			&preparsed_data->picture_data.pict_tag_param[i];
	}
	str_ctx->grp_bstr_ctx.segment_list = str_ctx->grp_bstr_ctx.pre_pict_seg_list[0];
	str_ctx->grp_bstr_ctx.pict_tag_param = str_ctx->grp_bstr_ctx.pict_tag_param_array[0];
	str_ctx->grp_bstr_ctx.free_segments = segments;
	str_ctx->grp_bstr_ctx.segment_offset = 0;
	str_ctx->grp_bstr_ctx.insert_start_code = 0;

	/*
	 * Before processing the units service all the picture decoded events
	 * to free the resources1794
	 */
	bspp_service_pictures_decoded(str_ctx);

	/*
	 * A picture currently being parsed is already decoded (may happen
	 * after dwr in low latency mode) and its recourses were freed. Skip
	 * the rest of the picture.
	 */
	if (pict_ctx->sequ_hdr_info && pict_ctx->sequ_hdr_info->ref_count == 0) {
		pict_ctx->present = 0;
		pict_ctx->finished = 1;
	}

	/*
	 * For bitstreams without unit delimiters treat all the buffers as
	 * a single unit whose type is defined by the first buffer element.
	 */
	if (str_ctx->swsr_ctx.sr_config.delim_type == SWSR_DELIM_NONE) {
		struct bspp_bitstream_buffer *cur_buf =
			lst_first(&str_ctx->grp_bstr_ctx.buffer_chain);

		/* if there is no picture data we must be skipped. */
		if (!cur_buf || cur_buf->data_size == 0) {
			str_ctx->grp_bstr_ctx.unit_type = BSPP_UNIT_SKIP_PICTURE;
		} else if (cur_buf->bstr_element_type == VDEC_BSTRELEMENT_CODEC_CONFIG) {
			str_ctx->grp_bstr_ctx.unit_type = BSPP_UNIT_SEQUENCE;
		} else if (cur_buf->bstr_element_type == VDEC_BSTRELEMENT_PICTURE_DATA ||
			 cur_buf->bstr_element_type == VDEC_BSTRELEMENT_UNSPECIFIED) {
			str_ctx->grp_bstr_ctx.unit_type = BSPP_UNIT_PICTURE;
			str_ctx->grp_bstr_ctx.segment_list = str_ctx->grp_bstr_ctx.pict_seg_list[0];
		}

		str_ctx->grp_bstr_ctx.delim_present = 0;
	}

	/*
	 * Load the first section (buffer) of biststream into the software
	 * shift-register. BSPP maps "buffer" to "section" and allows for
	 * contiguous parsing of all buffers since unit boundaries are not
	 * known up-front. Unit parsing and segment creation is happening in a
	 * single pass.
	 */
	result = swsr_start_bitstream(str_ctx->swsr_ctx.swsr_context,
				      &str_ctx->swsr_ctx.sr_config,
				      str_ctx->grp_bstr_ctx.total_data_size,
				      str_ctx->swsr_ctx.emulation_prevention);

	/* Seek for next delimiter or end of data and catch any exceptions. */
	if (str_ctx->grp_bstr_ctx.delim_present) {
		/* Locate the first bitstream unit. */
		found = swsr_seek_delim_or_eod(str_ctx->swsr_ctx.swsr_context);
	}

	if (str_ctx->swsr_ctx.sr_config.delim_type == SWSR_DELIM_SIZE) {
		struct bspp_bitstream_buffer *cur_buf =
			lst_first(&str_ctx->grp_bstr_ctx.in_flight_bufs);

		if (cur_buf->bstr_element_type == VDEC_BSTRELEMENT_CODEC_CONFIG &&
		    str_ctx->parser_callbacks.parse_codec_config_cb) {
			/* Parse codec config header and catch any exceptions */
			str_ctx->parser_callbacks.parse_codec_config_cb
						(str_ctx->swsr_ctx.swsr_context,
						 &unit_count,
						 &num_arrays,
						 &str_ctx->swsr_ctx.sr_config.delim_length,
						 &size_delim_bits);
		} else {
			size_delim_bits = str_ctx->swsr_ctx.sr_config.delim_length;
		}
	}

	/* Process all the bitstream units until the picture is located. */
	while (found != SWSR_FOUND_EOD && !pict_ctx->finished) {
		struct bspp_bitstream_buffer *cur_buf =
			lst_first(&str_ctx->grp_bstr_ctx.in_flight_bufs);

		if (!cur_buf) {
			pr_err("%s: cur_buf pointer is NULL\n", __func__);
			result = IMG_ERROR_INVALID_PARAMETERS;
			goto error;
		}

		if (str_ctx->swsr_ctx.sr_config.delim_type ==
			SWSR_DELIM_SIZE && cur_buf->bstr_element_type ==
			VDEC_BSTRELEMENT_CODEC_CONFIG &&
			str_ctx->parser_callbacks.update_unit_counts_cb) {
			/*
			 * Parse middle part of codec config header and catch
			 * any exceptions.
			 */
			str_ctx->parser_callbacks.update_unit_counts_cb
						(str_ctx->swsr_ctx.swsr_context,
						 &unit_count,
						 &num_arrays);
		}

		/* Process the next unit. */
		result = bspp_process_unit(str_ctx, size_delim_bits, pict_ctx, parse_state);
		if (result == IMG_ERROR_NOT_SUPPORTED)
			goto error;

		if (str_ctx->swsr_ctx.sr_config.delim_type != SWSR_DELIM_NONE)
			str_ctx->grp_bstr_ctx.delim_present = 1;

		/* jump to the next view */
		if (parse_state->new_view) {
			result = bspp_jump_to_next_view(&str_ctx->grp_bstr_ctx,
							preparsed_data,
							parse_state);
			if (result != IMG_SUCCESS)
				goto error;

			parse_state->new_view = 0;
		}

		if (!pict_ctx->finished) {
			/*
			 * Seek for next delimiter or end of data and catch any
			 * exceptions.
			 */
			/* Locate the next bitstream unit or end of data */
			found = swsr_seek_delim_or_eod(str_ctx->swsr_ctx.swsr_context);

			{
				struct bspp_bitstream_buffer *buf;
				/* Update the offset within current buffer. */
				swsr_get_byte_offset_curbuf(str_ctx->grp_bstr_ctx.swsr_context,
							    &parse_state->prev_byte_offset_buf);
				buf = lst_first(&str_ctx->grp_bstr_ctx.in_flight_bufs);
				if (buf) {
					parse_state->prev_buf_map_id = buf->bufmap_id;
					parse_state->prev_buf_data_size = buf->data_size;
				}
			}
		}
	}

	/* Finalize parsing for this video standard. */
	if (str_ctx->parser_callbacks.finalise_parsing_cb && end_of_pic) {
		str_ctx->parser_callbacks.finalise_parsing_cb((void *)&str_ctx->str_alloc,
			parse_state);
	}

	/*
	 * Create segments for each buffer held by the software shift register
	 * (and not yet processed).
	 */
	while (lst_first(&str_ctx->grp_bstr_ctx.in_flight_bufs)) {
		struct bspp_bitstream_buffer *buf =
			lst_removehead(&str_ctx->grp_bstr_ctx.in_flight_bufs);

		result = bspp_terminate_buffer(&str_ctx->grp_bstr_ctx, buf);
	}

	/*
	 * Create segments for each buffer not yet requested by the shift
	 * register.
	 */
	while (lst_first(&str_ctx->grp_bstr_ctx.buffer_chain)) {
		struct bspp_bitstream_buffer *buf =
			lst_removehead(&str_ctx->grp_bstr_ctx.buffer_chain);

		result = bspp_terminate_buffer(&str_ctx->grp_bstr_ctx, buf);
	}

	/*
	 * Populate the parsed data information for picture only if one is
	 * present. The anonymous data has already been added to the
	 * appropriate segment list.
	 */
	if (pict_ctx->present && !pict_ctx->invalid) {
		if (!pict_ctx->new_pict_signalled) {
			/*
			 * Provide data about sequence used by picture.
			 * Signal "new sequence" if the sequence header is new
			 * or has changed. always switch seq when changing base
			 * and additional views
			 */
			if (pict_ctx->sequ_hdr_info) {
				if (pict_ctx->sequ_hdr_info->sequ_hdr_info.sequ_hdr_id !=
					str_ctx->sequ_hdr_id ||
					pict_ctx->sequ_hdr_info->ref_count == 1 ||
					pict_ctx->ext_sequ_hdr_info ||
					pict_ctx->closed_gop) {
					preparsed_data->new_sequence = 1;
					preparsed_data->sequ_hdr_info =
						pict_ctx->sequ_hdr_info->sequ_hdr_info;
				}
			}

			/* Signal "new subsequence" and its common header information. */
			if (pict_ctx->ext_sequ_hdr_info) {
				preparsed_data->new_sub_sequence = 1;
				preparsed_data->ext_sequ_hdr_info =
					pict_ctx->ext_sequ_hdr_info->sequ_hdr_info;

				for (i = 0; i < VDEC_H264_MVC_MAX_VIEWS - 1;
					i++) {
					/*
					 * prefix is always the last one
					 * do not attach any header info to it
					 */
					if (preparsed_data->ext_pictures_data[i].is_prefix)
						break;

					/* attach headers */
					preparsed_data->ext_pictures_data[i].sequ_hdr_id =
					pict_ctx->ext_sequ_hdr_info->sequ_hdr_info.sequ_hdr_id;
					pict_ctx->ext_sequ_hdr_info->ref_count++;
					preparsed_data->ext_pictures_data[i].pict_hdr_info =
						pict_ctx->pict_hdr_info[i + 1];
				}

				preparsed_data->ext_pictures_data
					[0].pict_hdr_info.first_pic_of_sequence =
					preparsed_data->new_sub_sequence;

				/*
				 * Update the base view common sequence info
				 * with the number of views that the stream has.
				 * Otherwise the number of views is inconsistent
				 * between base view sequence and dependent view
				 * sequences. Also base view sequence appears
				 * with one view and the driver calculates the
				 * wrong number of resources.
				 */
				preparsed_data->sequ_hdr_info.com_sequ_hdr_info.num_views =
				preparsed_data->ext_sequ_hdr_info.com_sequ_hdr_info.num_views;
			}

			/* Signal if this picture is the first in a closed GOP */
			if (pict_ctx->closed_gop) {
				preparsed_data->closed_gop = 1;
				preparsed_data->sequ_hdr_info.com_sequ_hdr_info.not_dpb_flush =
					str_ctx->inter_pict_data.not_dpb_flush;
			}

			/*
			 * Signal "new picture" and its common header
			 * information.
			 */
			preparsed_data->new_picture = 1;
			if (pict_ctx->sequ_hdr_info) {
				preparsed_data->picture_data.sequ_hdr_id =
					pict_ctx->sequ_hdr_info->sequ_hdr_info.sequ_hdr_id;
			}
			preparsed_data->picture_data.pict_hdr_info = pict_ctx->pict_hdr_info[0];

			preparsed_data->picture_data.pict_hdr_info.first_pic_of_sequence =
				preparsed_data->new_sequence;
			if (contig_buf_info)
				preparsed_data->picture_data.pict_hdr_info.fragmented_data = 1;
			else
				preparsed_data->picture_data.pict_hdr_info.fragmented_data = 0;

			str_ctx->sequ_hdr_id = preparsed_data->picture_data.sequ_hdr_id;

			pict_ctx->new_pict_signalled = 1;

			/*
			 * aso/fmo supported only when a frame is submitted as
			 * a whole
			 */
			if (parse_state->discontinuous_mb && !end_of_pic)
				result = IMG_ERROR_NOT_SUPPORTED;
		} else {
			preparsed_data->new_fragment = 1;

			if (parse_state->discontinuous_mb)
				result = IMG_ERROR_NOT_SUPPORTED;
		}

		lst_init(&temp_list);

		segment = lst_removehead(&preparsed_data->picture_data.pict_seg_list[0]);
		while (segment) {
			lst_add(&temp_list, segment);
			segment = lst_removehead(&preparsed_data->picture_data.pict_seg_list[0]);
		}

		segment = lst_removehead(&str_ctx->inter_pict_data.pic_prefix_seg);
		while (segment) {
			lst_add(&preparsed_data->picture_data.pict_seg_list[0],
				segment);
			segment = lst_removehead(&str_ctx->inter_pict_data.pic_prefix_seg);
		}

		segment = lst_removehead(&temp_list);
		while (segment) {
			lst_add(&preparsed_data->picture_data.pict_seg_list[0],
				segment);
			segment = lst_removehead(&temp_list);
		}

		for (i = 0; i < VDEC_H264_MVC_MAX_VIEWS; i++) {
			unsigned int j;
			struct bspp_picture_data *ext_pic_data =
				&preparsed_data->ext_pictures_data[i];

			if (preparsed_data->ext_pictures_data[i].is_prefix) {
				for (j = 0; j < BSPP_MAX_PICTURES_PER_BUFFER;
					j++) {
					segment = lst_removehead(&ext_pic_data->pict_seg_list[j]);
					while (segment) {
						lst_add(&str_ctx->inter_pict_data.pic_prefix_seg,
							segment);
						segment = lst_removehead
								(&ext_pic_data->pict_seg_list[j]);
					}
				}
				preparsed_data->num_ext_pictures--;
				break;
			}
		}
	} else if (pict_ctx->present && pict_ctx->sequ_hdr_info) {
		/*
		 * Reduce the reference count since this picture will not be
		 * decoded.
		 */
		pict_ctx->sequ_hdr_info->ref_count--;
		/* Release sequence data. */
		if (str_ctx->parser_callbacks.release_data_cb) {
			str_ctx->parser_callbacks.release_data_cb((void *)&str_ctx->str_alloc,
				BSPP_UNIT_SEQUENCE,
				pict_ctx->sequ_hdr_info->secure_sequence_info);
		}
	}

	/* Reset the group bitstream context */
	lst_init(&str_ctx->grp_bstr_ctx.buffer_chain);
	memset(&str_ctx->grp_bstr_ctx, 0, sizeof(str_ctx->grp_bstr_ctx));

	/*
	 * for now: return IMG_ERROR_NOT_SUPPORTED only if explicitly set by
	 * parser
	 */
	result = (result == IMG_ERROR_NOT_SUPPORTED) ?
		IMG_ERROR_NOT_SUPPORTED : IMG_SUCCESS;

	if (end_of_pic)
		parse_state->initialised = 0;

	return result;

error:
	/* Free the SWSR list of buffers */
	while (lst_first(&str_ctx->grp_bstr_ctx.in_flight_bufs))
		lst_removehead(&str_ctx->grp_bstr_ctx.in_flight_bufs);

	return result;
}

/*
 * @Function	bspp_stream_destroy
 *
 */
int bspp_stream_destroy(void *str_context_handle)
{
	struct bspp_str_context *str_ctx = (struct bspp_str_context *)str_context_handle;
	unsigned int i;
	unsigned int sps_id;
	unsigned int pps_id;
	struct bspp_sequence_hdr_info *sequ_hdr_info;
	struct bspp_pps_info *pps_info;
	unsigned int result;

	/* Validate input arguments. */
	if (!str_context_handle) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error;
	}

	swsr_deinitialise(str_ctx->swsr_ctx.swsr_context);

	/*
	 * Service all the picture decoded events and free any unused
	 * resources.
	 */
	bspp_service_pictures_decoded(str_ctx);
	for (sps_id = 0; sps_id < SEQUENCE_SLOTS; sps_id++)
		bspp_remove_unused_sequence(str_ctx, sps_id);

	if (str_ctx->vid_std_features.uses_pps) {
		for (pps_id = 0; pps_id < PPS_SLOTS; pps_id++)
			bspp_remove_unused_pps(str_ctx, pps_id);
	}

	if (str_ctx->vid_std_features.uses_vps) {
		struct bspp_vps_info *vps_info;

		for (i = 0; i < VPS_SLOTS; ++i) {
			vps_info = lst_removehead(&str_ctx->str_alloc.vps_data_list[i]);

			if (vps_info)
				lst_add(&str_ctx->str_alloc.available_vps_list, vps_info);

			/*
			 * when we are done with the stream we should have MAXIMUM 1 VPS
			 * per slot, so after removing this one we should have none
			 * In case of "decodenframes" this is not true because we send more
			 * pictures for decode than what we expect to receive back, which
			 * means that potentially additional sequences/PPS are in the list
			 */
			vps_info = lst_removehead(&str_ctx->str_alloc.vps_data_list[i]);
			if (vps_info) {
				do {
					lst_add(&str_ctx->str_alloc.available_vps_list, vps_info);
					vps_info =
					lst_removehead(&str_ctx->str_alloc.vps_data_list[i]);
				} while (vps_info);
			}
			VDEC_ASSERT(lst_empty(&str_ctx->str_alloc.vps_data_list[i]));
		}

		vps_info = NULL;
		for (i = 0; i < MAX_VPSS; ++i) {
			VDEC_ASSERT(!lst_empty(&str_ctx->str_alloc.available_vps_list));
			vps_info = lst_removehead(&str_ctx->str_alloc.available_vps_list);
			if (vps_info) {
				kfree(vps_info->secure_vpsinfo);
				kfree(vps_info);
			} else {
				VDEC_ASSERT(vps_info);
				pr_err("vps still active at shutdown\n");
			}
		}
		VDEC_ASSERT(lst_empty(&str_ctx->str_alloc.available_vps_list));
	}

	/* Free the memory required for this stream. */
	for (i = 0; i < SEQUENCE_SLOTS; i++) {
		sequ_hdr_info = lst_removehead(&str_ctx->str_alloc.sequence_data_list[i]);
		if (sequ_hdr_info) {
			if (str_ctx->parser_callbacks.release_data_cb)
				str_ctx->parser_callbacks.release_data_cb
					((void *)&str_ctx->str_alloc,
					BSPP_UNIT_SEQUENCE,
					sequ_hdr_info->secure_sequence_info);
			lst_add(&str_ctx->str_alloc.available_sequence_list,
				sequ_hdr_info);
		}

		/*
		 * when we are done with the stream we should have MAXIMUM 1
		 * sequence per slot, so after removing this one we should have
		 * none In case of "decoded frames" this is not true because we
		 * send more pictures for decode than what we expect to receive
		 * back, which means that potentially additional sequences/PPS
		 * are in the list
		 */
		sequ_hdr_info = lst_removehead(&str_ctx->str_alloc.sequence_data_list[i]);
		if (sequ_hdr_info) {
			unsigned int count_extra_sequences = 0;

			do {
				count_extra_sequences++;
				if (str_ctx->parser_callbacks.release_data_cb) {
					str_ctx->parser_callbacks.release_data_cb
						((void *)&str_ctx->str_alloc,
						 BSPP_UNIT_SEQUENCE,
						 sequ_hdr_info->secure_sequence_info);
				}
				lst_add(&str_ctx->str_alloc.available_sequence_list,
					sequ_hdr_info);
				sequ_hdr_info =
					lst_removehead(&str_ctx->str_alloc.sequence_data_list[i]);
			} while (sequ_hdr_info);
		}
	}

	if (str_ctx->vid_std_features.uses_pps) {
		for (i = 0; i < PPS_SLOTS; i++) {
			pps_info = lst_removehead(&str_ctx->str_alloc.pps_data_list[i]);
			if (pps_info)
				lst_add(&str_ctx->str_alloc.available_ppss_list, pps_info);

			/*
			 * when we are done with the stream we should have
			 * MAXIMUM 1 PPS per slot, so after removing this one
			 * we should have none
			 * In case of "decodedframes" this is not true because
			 * we send more pictures for decode than what we expect
			 * to receive back, which means that potentially
			 * additional sequences/PPS are in the list
			 */
			pps_info = lst_removehead(&str_ctx->str_alloc.pps_data_list[i]);
			if (pps_info) {
				unsigned int count_extra_ppss = 0;

				do {
					count_extra_ppss++;
					lst_add(&str_ctx->str_alloc.available_ppss_list,
						pps_info);
					pps_info =
					lst_removehead(&str_ctx->str_alloc.pps_data_list[i]);
				} while (pps_info);
			}
		}
	}

	for (i = 0; i < MAX_SEQUENCES; i++) {
		sequ_hdr_info = lst_removehead(&str_ctx->str_alloc.available_sequence_list);
		if (sequ_hdr_info && str_ctx->parser_callbacks.destroy_data_cb)
			str_ctx->parser_callbacks.destroy_data_cb
				(BSPP_UNIT_SEQUENCE, sequ_hdr_info->secure_sequence_info);
	}

	kfree(str_ctx->secure_sequence_info);
	str_ctx->secure_sequence_info = NULL;
	kfree(str_ctx->sequ_hdr_info);
	str_ctx->sequ_hdr_info = NULL;

	if (str_ctx->vid_std_features.uses_pps) {
		for (i = 0; i < MAX_PPSS; i++) {
			pps_info = lst_removehead(&str_ctx->str_alloc.available_ppss_list);
			if (pps_info && str_ctx->parser_callbacks.destroy_data_cb)
				str_ctx->parser_callbacks.destroy_data_cb
							(BSPP_UNIT_PPS, pps_info->secure_pps_info);
		}

		kfree(str_ctx->secure_pps_info);
		str_ctx->secure_pps_info = NULL;
		kfree(str_ctx->pps_info);
		str_ctx->pps_info = NULL;
	}

	/* destroy mutex */
	mutex_destroy(str_ctx->bspp_mutex);
	kfree(str_ctx->bspp_mutex);
	str_ctx->bspp_mutex = NULL;

	kfree(str_ctx);

	return IMG_SUCCESS;
error:
	return result;
}

/*
 * @Function	bspp_set_codec_config
 *
 */
int bspp_set_codec_config(const void *str_context_handle,
			  const struct vdec_codec_config *codec_config)
{
	struct bspp_str_context *str_ctx = (struct bspp_str_context *)str_context_handle;
	unsigned int result = IMG_SUCCESS;

	/* Validate input arguments. */
	if (!str_context_handle || !codec_config) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error;
	}

	switch (str_ctx->vid_std) {
	default:
		result = IMG_ERROR_NOT_SUPPORTED;
		break;
	}
error:
	return result;
}

/*
 * @Function	bspp_stream_create
 *
 */
int bspp_stream_create(const struct vdec_str_configdata *str_config_data,
		       void **str_ctx_handle,
		       struct bspp_ddbuf_array_info fw_sequence[],
		       struct bspp_ddbuf_array_info fw_pps[])
{
	struct bspp_str_context *str_ctx;
	unsigned int result = IMG_SUCCESS;
	unsigned int i;
	struct bspp_sequence_hdr_info *sequ_hdr_info;
	struct bspp_pps_info *pps_info;
	struct bspp_parse_state *parse_state;

	/* Allocate a stream structure */
	str_ctx = kmalloc(sizeof(*str_ctx), GFP_KERNEL);
	if (!str_ctx) {
		result = IMG_ERROR_OUT_OF_MEMORY;
		goto error;
	}
	memset(str_ctx, 0, sizeof(*str_ctx));

	/* Initialise the stream context structure. */
	str_ctx->sequ_hdr_id = BSPP_INVALID;
	str_ctx->vid_std = str_config_data->vid_std;
	str_ctx->bstr_format = str_config_data->bstr_format;
	str_ctx->disable_mvc = str_config_data->disable_mvc;
	str_ctx->full_scan = str_config_data->full_scan;
	str_ctx->immediate_decode = str_config_data->immediate_decode;
	str_ctx->intra_frame_closed_gop = str_config_data->intra_frame_closed_gop;

	parse_state = &str_ctx->parse_state;

	/* Setup group buffer processing state. */
	parse_state->inter_pict_ctx = &str_ctx->inter_pict_data;
	parse_state->prev_bottom_pic_flag = (unsigned char)BSPP_INVALID;
	parse_state->next_pic_is_new = 1;
	parse_state->prev_frame_num = BSPP_INVALID;
	parse_state->second_field_flag = 0;

	lst_init(&str_ctx->grp_bstr_ctx.buffer_chain);

	if (str_ctx->vid_std < VDEC_STD_MAX && parser_fxns[str_ctx->vid_std].set_parser_config) {
		parser_fxns[str_ctx->vid_std].set_parser_config(str_ctx->bstr_format,
			&str_ctx->vid_std_features,
			&str_ctx->swsr_ctx,
			&str_ctx->parser_callbacks,
			&str_ctx->inter_pict_data);
	} else {
		result = IMG_ERROR_NOT_SUPPORTED;
		goto error;
	}

	/* Allocate the memory required for this stream for Sequence/PPS info */
	lst_init(&str_ctx->str_alloc.available_sequence_list);

	str_ctx->sequ_hdr_info = kmalloc((MAX_SEQUENCES * sizeof(struct bspp_sequence_hdr_info)),
					 GFP_KERNEL);
	if (!str_ctx->sequ_hdr_info) {
		result = IMG_ERROR_OUT_OF_MEMORY;
		goto error;
	}
	memset(str_ctx->sequ_hdr_info, 0x00,
	       (MAX_SEQUENCES * sizeof(struct bspp_sequence_hdr_info)));

	str_ctx->secure_sequence_info =
		kmalloc((MAX_SEQUENCES * str_ctx->vid_std_features.seq_size),
			GFP_KERNEL);
	if (!str_ctx->secure_sequence_info) {
		result = IMG_ERROR_OUT_OF_MEMORY;
		goto error;
	}
	memset(str_ctx->secure_sequence_info, 0x00,
	       (MAX_SEQUENCES * str_ctx->vid_std_features.seq_size));

	sequ_hdr_info = (struct bspp_sequence_hdr_info *)(str_ctx->sequ_hdr_info);
	for (i = 0; i < MAX_SEQUENCES; i++) {
		/* Deal with the device memory for FW SPS data */
		sequ_hdr_info->fw_sequence = fw_sequence[i];
		sequ_hdr_info->sequ_hdr_info.bufmap_id =
			fw_sequence[i].ddbuf_info.bufmap_id;
		sequ_hdr_info->sequ_hdr_info.buf_offset =
			fw_sequence[i].buf_offset;
		sequ_hdr_info->secure_sequence_info = (void *)(str_ctx->secure_sequence_info +
			(i * str_ctx->vid_std_features.seq_size));

		lst_add(&str_ctx->str_alloc.available_sequence_list,
			sequ_hdr_info);
		sequ_hdr_info++;
	}

	if (str_ctx->vid_std_features.uses_pps) {
		lst_init(&str_ctx->str_alloc.available_ppss_list);
		str_ctx->pps_info = kmalloc((MAX_PPSS * sizeof(struct bspp_pps_info)), GFP_KERNEL);
		if (!str_ctx->pps_info) {
			result = IMG_ERROR_OUT_OF_MEMORY;
			goto error;
		}
		memset(str_ctx->pps_info, 0x00, (MAX_PPSS * sizeof(struct bspp_pps_info)));
		str_ctx->secure_pps_info = kmalloc((MAX_PPSS * str_ctx->vid_std_features.pps_size),
						   GFP_KERNEL);
		if (!str_ctx->secure_pps_info) {
			result = IMG_ERROR_OUT_OF_MEMORY;
			goto error;
		}
		memset(str_ctx->secure_pps_info, 0x00,
		       (MAX_PPSS * str_ctx->vid_std_features.pps_size));

		pps_info = (struct bspp_pps_info *)(str_ctx->pps_info);
		for (i = 0; i < MAX_PPSS; i++) {
			/* Deal with the device memory for FW PPS data */
			pps_info->fw_pps = fw_pps[i];
			pps_info->bufmap_id = fw_pps[i].ddbuf_info.bufmap_id;
			pps_info->buf_offset = fw_pps[i].buf_offset;

			/*
			 * We have no container for the PPS that passes down to the kernel,
			 * for this reason the h264 secure parser needs to populate that
			 * info into the picture header (Second)PictAuxData.
			 */
			pps_info->secure_pps_info = (void *)(str_ctx->secure_pps_info + (i *
							str_ctx->vid_std_features.pps_size));

			lst_add(&str_ctx->str_alloc.available_ppss_list, pps_info);
			pps_info++;
		}

		/* As only standards that use PPS also use VUI, initialise
		 * the appropriate data structures here.
		 * Initialise the list of raw bitstream data containers.
		 */
		lst_init(&str_ctx->str_alloc.raw_data_list_available);
		lst_init(&str_ctx->str_alloc.raw_data_list_used);
	}

	if (str_ctx->vid_std_features.uses_vps) {
		struct bspp_vps_info *vps_info;

		lst_init(&str_ctx->str_alloc.available_vps_list);
		for (i = 0; i < MAX_VPSS; ++i) {
			vps_info = kmalloc(sizeof(*vps_info), GFP_KERNEL);
			VDEC_ASSERT(vps_info);
			if (!vps_info) {
				result = IMG_ERROR_OUT_OF_MEMORY;
				goto error;
			}

			memset(vps_info, 0x00, sizeof(struct bspp_vps_info));
			/*
			 * for VPS we do not allocate device memory since (at least for now)
			 * there is no need to pass any data from VPS directly to FW
			 */
			/* Allocate memory for BSPP local VPS data structure. */
			vps_info->secure_vpsinfo =
				kmalloc(str_ctx->vid_std_features.vps_size, GFP_KERNEL);

			VDEC_ASSERT(vps_info->secure_vpsinfo);
			if (!vps_info->secure_vpsinfo) {
				result = IMG_ERROR_OUT_OF_MEMORY;
				goto error;
			}
			memset(vps_info->secure_vpsinfo, 0, str_ctx->vid_std_features.vps_size);

			lst_add(&str_ctx->str_alloc.available_vps_list, vps_info);
		}
	}

	/* ... and initialise the lists that will use this data */
	for (i = 0; i < SEQUENCE_SLOTS; i++)
		lst_init(&str_ctx->str_alloc.sequence_data_list[i]);

	if (str_ctx->vid_std_features.uses_pps)
		for (i = 0; i < PPS_SLOTS; i++)
			lst_init(&str_ctx->str_alloc.pps_data_list[i]);

	str_ctx->bspp_mutex = kzalloc(sizeof(*str_ctx->bspp_mutex), GFP_KERNEL);
	if (!str_ctx->bspp_mutex) {
		result = -ENOMEM;
		goto error;
	}
	mutex_init(str_ctx->bspp_mutex);

	/* Initialise the software shift-register */
	swsr_initialise(bspp_exception_handler, &str_ctx->parse_ctx,
			(swsr_callback_fxn) bspp_shift_reg_cb,
			&str_ctx->grp_bstr_ctx,
			&str_ctx->swsr_ctx.swsr_context);

	/* Setup the parse context */
	str_ctx->parse_ctx.swsr_context = str_ctx->swsr_ctx.swsr_context;

	*str_ctx_handle = str_ctx;

	return IMG_SUCCESS;

error:
	if (str_ctx) {
		kfree(str_ctx->sequ_hdr_info);
		kfree(str_ctx->secure_sequence_info);
		kfree(str_ctx->pps_info);
		kfree(str_ctx->secure_pps_info);
		kfree(str_ctx);
	}

	return result;
}

void bspp_freeraw_sei_datacontainer(const void *str_res,
				    struct vdec_raw_bstr_data *rawsei_datacontainer)
{
	struct bspp_raw_sei_alloc *rawsei_alloc = NULL;

	/* Check input params. */
	if (str_res && rawsei_datacontainer) {
		struct bspp_stream_alloc_data *alloc_data =
			(struct bspp_stream_alloc_data *)str_res;

		rawsei_alloc = container_of(rawsei_datacontainer,
					    struct bspp_raw_sei_alloc,
					    raw_sei_data);
		memset(&rawsei_alloc->raw_sei_data, 0, sizeof(rawsei_alloc->raw_sei_data));
		lst_remove(&alloc_data->raw_sei_alloc_list, rawsei_alloc);
		kfree(rawsei_alloc);
	}
}

void bspp_freeraw_sei_datalist(const void *str_res, struct vdec_raw_bstr_data *rawsei_datalist)
{
	/* Check input params. */
	if (rawsei_datalist && str_res) {
		struct vdec_raw_bstr_data *sei_raw_datacurr = NULL;

		/* Start fromm the first element... */
		sei_raw_datacurr = rawsei_datalist;
		/* Free all the linked raw SEI data containers. */
		while (sei_raw_datacurr) {
			struct vdec_raw_bstr_data *seiraw_datanext =
				sei_raw_datacurr->next;
			bspp_freeraw_sei_datacontainer(str_res, sei_raw_datacurr);
			sei_raw_datacurr = seiraw_datanext;
		}
	}
}

void bspp_streamrelese_rawbstrdataplain(const void *str_res, const void *rawdata)
{
	struct bspp_stream_alloc_data *str_alloc =
		(struct bspp_stream_alloc_data *)str_res;
	struct bspp_raw_bitstream_data *rawbstrdata =
		(struct bspp_raw_bitstream_data *)rawdata;

	if (rawbstrdata) {
		/* Decrement the raw bitstream data reference count. */
		rawbstrdata->ref_count--;
		/* If no entity is referencing the raw
		 * bitstream data any more
		 */
		if (rawbstrdata->ref_count == 0) {
			/* ... free the raw bistream data buffer... */
			kfree(rawbstrdata->raw_bitstream_data.data);
			memset(&rawbstrdata->raw_bitstream_data, 0,
			       sizeof(rawbstrdata->raw_bitstream_data));
			/* ...and return it to the list. */
			lst_remove(&str_alloc->raw_data_list_used, rawbstrdata);
			lst_add(&str_alloc->raw_data_list_available, rawbstrdata);
		}
	}
}

struct bspp_vps_info *bspp_get_vpshdr(void *str_res, unsigned int vps_id)
{
	struct bspp_stream_alloc_data *alloc_data =
		(struct bspp_stream_alloc_data *)str_res;

	if (vps_id >= VPS_SLOTS || !alloc_data)
		return NULL;

	return lst_last(&alloc_data->vps_data_list[vps_id]);
}
