/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD Bitstream Buffer Pre-Parser
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Lakshmi Sankar <lakshmisankar-t@ti.com>
 * Re-written for upstreming
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */
#ifndef __BSPP_H__
#define __BSPP_H__

#include <linux/types.h>

#include "h264fw_data.h"
#include "lst.h"
#include "vdec_defs.h"

/*
 * There are up to 2 pictures in each buffer
 * (plus trailing data for the next picture, e.g. PPS).
 */
#define BSPP_MAX_PICTURES_PER_BUFFER 3

#define BSPP_INVALID ((unsigned int)(-1))

/*
 * This enables signalling of closed gop at every I frame. Add resilience to
 * seeking functionality.
 */
#define I_FRAME_SIGNALS_CLOSED_GOP

/*
 * enum bspp_error_type - enumeration of parsing error , different error flag
 *	for different data unit
 */
enum bspp_error_type {
	/* No Error in parsing. */
	BSPP_ERROR_NONE                      = (0),
	/* Correction in VSH, Replaced VSH with faulty one */
	BSPP_ERROR_CORRECTION_VSH            = (1 << 0),
	/*
	 * Correction in parsed Value, clamp the value if it goes beyond
	 * the limit
	 */
	BSPP_ERROR_CORRECTION_VALIDVALUE     = (1 << 1),
	/* Error in Aux data (i.e. PPS in H.264) parsing */
	BSPP_ERROR_AUXDATA                   = (1 << 2),
	/* Error in  parsing, more data remains in VSH data unit after parsing */
	BSPP_ERROR_DATA_REMAINS              = (1 << 3),
	/* Error in  parsing, parsed codeword is invalid */
	BSPP_ERROR_INVALID_VALUE             = (1 << 4),
	/* Error in  parsing, parsing error */
	BSPP_ERROR_DECODE                    = (1 << 5),
	/* reference frame is not available for decoding */
	BSPP_ERROR_NO_REF_FRAME              = (1 << 6),
	/* Non IDR frame loss detected */
	BSPP_ERROR_NONIDR_FRAME_LOSS         = (1 << 7),
	/* IDR frame loss detected */
	BSPP_ERROR_IDR_FRAME_LOSS            = (1 << 8),
	/* Error in  parsing, insufficient data to complete parsing */
	BSPP_ERROR_INSUFFICIENT_DATA         = (1 << 9),
	/* Severe Error, Error indicates, no support for this picture data */
	BSPP_ERROR_UNSUPPORTED               = (1 << 10),
	/* Severe Error, Error in which could not be recovered */
	BSPP_ERROR_UNRECOVERABLE             = (1 << 11),
	/* Severe Error, to indicate that NAL Header is absent after SCP */
	BSPP_ERROR_NO_NALHEADER              = (1 << 12),
	BSPP_ERROR_NO_SEQUENCE_HDR           = (1 << 13),
	BSPP_ERROR_SIGNALED_IN_STREAM        = (1 << 14),
	BSPP_ERROR_UNKNOWN_DATAUNIT_DETECTED = (1 << 15),
	BSPP_ERROR_NO_PPS                    = (1 << 16),
	BSPP_ERROR_NO_VPS                    = (1 << 17),
	BSPP_ERROR_OUT_OF_MEMORY             = (1 << 18),
	/* The shift value of the last error bit */
	BSPP_ERROR_MAX_SHIFT                 = 18,
	BSPP_ERROR_FORCE32BITS               = 0x7FFFFFFFU
};

/*
 * struct bspp_ddbuf_info - Buffer info
 * @buf_size: The size of the buffer (in bytes)
 * @cpu_virt_addr: The CPU virtual address  (mapped into the local cpu MMU)
 * @mem_attrib: Memory attributes
 * @bufmap_id: buffer mappind id
 */
struct bspp_ddbuf_info {
	unsigned int buf_size;
	void *cpu_virt_addr;
	enum sys_emem_attrib mem_attrib;
	unsigned int buf_id;
	unsigned int bufmap_id;
};

/*
 * struct bspp_ddbuf_array_info - Buffer array info
 * @ddbuf_info: Buffer info (container)
 * @buf_element_size: Size of each element
 * @buf_offset: Offset for each element
 */
struct bspp_ddbuf_array_info {
	struct bspp_ddbuf_info ddbuf_info;
	unsigned int buf_element_size;
	unsigned int buf_offset;
};

/**
 * struct bspp_bitstr_seg - Bitstream segment
 * @lst_padding:
 * @data_size: Size of data
 * @data_byte_offset: Offset for data
 * @bstr_seg_flag: flag indicates the bitstream segment type
 * @start_code_suffix: start code prefix
 * @bufmap_id: Buffer map ID
 */
struct bspp_bitstr_seg {
	void *lst_padding;
	unsigned int data_size;
	unsigned int data_byte_offset;
	unsigned int bstr_seg_flag;
	unsigned char start_code_suffix;
	unsigned int bufmap_id;
};

/*
 * struct bspp_pict_data - Picture Header Data Information
 * @bufmap_id: Buffer ID to use inside kernel #VXDIO_sDdBufInfo
 * @buf_offset: Buffer offset (for packed device buffers, e.g. PPS)
 * @pic_data: Picture data
 * @size: Size (in bytes) of data.
 * @data_id: Data identifier.
 */
struct bspp_pict_data {
	unsigned int bufmap_id;
	unsigned int buf_offset;
	void *pic_data;
	unsigned int size;
	unsigned int id;
};

/*
 * struct bspp_pict_hdr_info - Picture Header Information
 */
struct bspp_pict_hdr_info {
	/*
	 * Picture is entirely intra-coded and doesn't use any reference data.
	 * NOTE: should be IMG_FALSE if this cannot be determined.
	 */
	int intra_coded;
	/* Picture might be referenced by subsequent pictures. */
	int ref;
	/* Picture is a field as part of a frame. */
	int field;
	/* Emulation prevention bytes are present in picture data. */
	int emulation_prevention;
	/* Post Processing */
	int post_processing;
	/* Macroblocks within the picture may not occur in raster-scan order */
	int discontinuous_mbs;
	/* Flag to indicate data is span across mulitple buffer. */
	int fragmented_data;
	/* SOS fields count value */
	unsigned char sos_count;
	/* This picture is the first of the sequence or not */
	int first_pic_of_sequence;

	enum vdecfw_parsermode parser_mode;
	/* Total size of picture data which is going to be submitted. */
	unsigned int pic_data_size;
	/* Size of coded frame as specified in the bitstream. */
	struct vdec_pict_size coded_frame_size;
	/* Display information for picture */
	struct vdec_pict_disp_info disp_info;

	/* Picture auxiliary data (e.g. H.264 SPS/PPS) */
	struct bspp_pict_data pict_aux_data;
	/* Picture auxiliary data (e.g. H.264 SPS/PPS) for 2nd picture */
	struct bspp_pict_data second_pict_aux_data;
	/* Slice group-map data. */
	struct bspp_pict_data pict_sgm_data;
#ifdef HAS_JPEG
	/* JPEG specific picture header information.*/
	struct vdec_jpeg_pict_hdr_info jpeg_pict_hdr_info;
#endif

	struct h264_pict_hdr_info {
		void *raw_vui_data;
		void *raw_sei_data_list_first_field;
		void *raw_sei_data_list_second_field;
		unsigned char nal_ref_idc;
		unsigned short frame_num;
	} h264_pict_hdr_info;

	struct {        /* HEVC specific frame information.*/
		int range_ext_present;
		int is_full_range_ext;
		void *raw_vui_data;
		void *raw_sei_datalist_firstfield;
		void *raw_sei_datalist_secondfield;
	} hevc_pict_hdr_info;
};

/*
 * struct bspp_sequ_hdr_info - Sequence header information
 */
struct bspp_sequ_hdr_info {
	unsigned int sequ_hdr_id;
	unsigned int ref_count;
	struct vdec_comsequ_hdrinfo com_sequ_hdr_info;
	unsigned int bufmap_id;
	unsigned int buf_offset;
};

/*
 * struct bspp_picture_data - Picture data
 */
struct bspp_picture_data {
	/* Anonymous */
	/*
	 * Bitstream segments that contain other (non-picture) data before
	 * the picture in the buffer (elements of type #VDECDD_sBitStrSeg).
	 */
	struct lst_t pre_pict_seg_list[BSPP_MAX_PICTURES_PER_BUFFER];
	/* Picture */
	unsigned int sequ_hdr_id;
	struct bspp_pict_hdr_info pict_hdr_info;
	/*
	 * Bitstream segments that contain picture data, one for each field
	 * (if present in same group of buffers (elements of type
	 * #VDECDD_sBitStrSeg).
	 */
	struct lst_t pict_seg_list[BSPP_MAX_PICTURES_PER_BUFFER];
	void *pict_tag_param[BSPP_MAX_PICTURES_PER_BUFFER];
	int is_prefix;
};

/*
 * struct bspp_preparsed_data - Pre-parsed buffer information
 */
struct bspp_preparsed_data {
	/* Sequence */
	int new_sequence;
	struct bspp_sequ_hdr_info sequ_hdr_info;
	int sequence_end;

	/* Closed GOP */
	int closed_gop;

	/* Picture */
	int new_picture;
	int new_fragment;
	struct bspp_picture_data picture_data;

	/* Additional pictures (MVC extension) */
	int new_sub_sequence;
	struct bspp_sequ_hdr_info ext_sequ_hdr_info;
	/* non-base view pictures + picture prefix for next frame */
	struct bspp_picture_data ext_pictures_data[VDEC_H264_MVC_MAX_VIEWS];
	unsigned int num_ext_pictures;

	/*
	 * Additional information
	 * Flags word to indicate error in parsing/decoding - see
	 * #VDEC_eErrorType
	 */
	unsigned int error_flags;
};

/*
 * struct bspp_picture_decoded - used to store picture-decoded information for
 * resource handling (sequences/PPSs)
 */
struct bspp_picture_decoded {
	void **lst_link;
	unsigned int sequ_hdr_id;
	unsigned int pps_id;
	unsigned int second_pps_id;
	int not_decoded;
	struct vdec_raw_bstr_data *sei_raw_data_first_field;
	struct vdec_raw_bstr_data *sei_raw_data_second_field;
};

/*
 * @Function	bspp_stream_create
 * @Description	Creates a stream context for which to pre-parse bitstream
 *		buffers. The following allocations will take place:
 *		- Local storage for high-level header parameters (secure)
 *		- Host memory for common sequence information (insecure)
 *		- Device memory for Sequence information (secure)
 *		- Device memory for PPS (secure, H.264 only)
 * @Input	vdec_str_configdata : config data corresponding to bitstream
 * @Output	str_context : A pointer used to return the stream context handle
 * @Input	fw_sequ: FW sequence data
 * @Input	fw_pps: FW pps data
 * @Return	This function returns either IMG_SUCCESS or an error code.
 */
int bspp_stream_create(const struct vdec_str_configdata *str_config_data,
		       void **str_context,
		       struct bspp_ddbuf_array_info fw_sequ[],
		       struct bspp_ddbuf_array_info fw_pps[]);

/*
 * @Function	bspp_set_codec_config
 * @Description	This function is used to set the out-of-band codec config data.
 * @Input	str_context_handle     : Stream context handle.
 * @Input	codec_config   : Codec-config data
 * @Return	This function returns either IMG_SUCCESS or an error code.
 */
int bspp_set_codec_config(const void *str_context_handle,
			  const struct vdec_codec_config *codec_config);

/*
 * @Function	bspp_stream_destroy
 * @Description	Destroys a stream context used to pre-parse bitstream buffers.
 * @Input	str_context_handle  : Stream context handle.
 * @Return	This function returns either IMG_SUCCESS or an error code.
 */
int bspp_stream_destroy(void *str_context_handle);

/*
 * @Function	bspp_submit_picture_decoded
 */
int bspp_submit_picture_decoded(void *str_context_handle,
				struct bspp_picture_decoded *picture_decoded);

/*
 * @Function	bspp_stream_submit_buffer
 */
int bspp_stream_submit_buffer(void *str_context_handle,
			      const struct bspp_ddbuf_info *ddbuf_info,
			      unsigned int bufmap_id,
			      unsigned int data_size,
			      void *pict_tag_param,
			      enum vdec_bstr_element_type bstr_element_type);

/*
 * @Function	bspp_stream_preparse_buffers
 * @Description	Pre-parses bistream buffer and returns picture information in
 *		structure that also signals when the buffer is last in picture.
 * @Input	str_context_handle: Stream context handle.
 * @Input	contiguous_buf_info : Contiguous buffer information
 *		multiple segments that may be non contiguous in memory
 * @Input	contiguous_buf_map_id : Contiguous Buffer Map id
 * @Input	segments: Pointer to a list of segments (see #VDECDD_sBitStrSeg)
 * @Output	preparsed_data: Container to return picture information. Only
 *		provide when buffer is last in picture (see #bForceEop in
 *		function #VDEC_StreamSubmitBstrBuf)
 * @Output	eos_flag: flag indicates end of stream
 * @Return	int : This function returns either IMG_SUCCESS or an error code.
 */
int bspp_stream_preparse_buffers
	(void *str_context_handle,
	const struct bspp_ddbuf_info *contiguous_buf_info,
	unsigned int contiguous_buf_map_id,
	struct lst_t *segments,
	struct bspp_preparsed_data *preparsed_data,
	int eos_flag);

#endif /* __BSPP_H__   */
