/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD Bitstream Buffer Pre-Parser Internal
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Amit Makani <amit.makani@ti.com>
 *
 * Re-written for upstreamimg
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */
#ifndef __BSPP_INT_H__
#define __BSPP_INT_H__

#include "bspp.h"
#include "swsr.h"

#define VDEC_MB_DIMENSION  (16)
#define MAX_COMPONENTS  (4)

#define print_value(a, ...)

#define BSPP_DEFAULT_SEQUENCE_ID   (0)

enum bspp_unit_type {
	BSPP_UNIT_NONE = 0,
	/* Only relevant for HEVC. */
	BSPP_UNIT_VPS,
	/* Only relevant for h.264 and HEVC */
	BSPP_UNIT_SEQUENCE, BSPP_UNIT_PPS,
	/*
	 * !< Data from these units should be placed in non-picture bitstream
	 *  segment lists. In conformant streams these units should not occur
	 *  in-between the picture data.
	 */
	BSPP_UNIT_PICTURE,
	BSPP_UNIT_SKIP_PICTURE,
	BSPP_UNIT_NON_PICTURE,
	BSPP_UNIT_UNCLASSIFIED,
	/* Unit is unsupported, don't change segment list */
	BSPP_UNIT_UNSUPPORTED,
	BSPP_UNIT_MAX,
	BSPP_UNIT_FORCE32BITS = 0x7FFFFFFFU
};

struct bspp_raw_bitstream_data {
	void **lst_link;
	unsigned int ref_count;
	struct vdec_raw_bstr_data raw_bitstream_data;
};

/*
 * struct bspp_h264_inter_pict_ctx
 * @Brief: This structure contains H264 state to be retained between pictures.
 */
struct bspp_h264_inter_pict_ctx {
	/*
	 *  The following get applied to every picture until updated
	 * (bitstream properties)
	 */
	int disable_vdmc_filt;
	int b4x4transform_mb_unavailable;
	/*
	 *  The following get applied to the next picture only
	 * (picture properties)
	 */
	int repeat_first_field;
	unsigned int max_frm_repeat;
	/*
	 *  Control variable to decide when to attach the SEI info
	 * (picture properties) to a picture
	 */
	int sei_info_attached_to_pic;
	/*
	 * The following variable is an approximation because we cannot
	 * parse out-of-order, it takes value as described:
	 *	 1) Initially it is BSPP_INVALID
	 *	 2) The first SPS sets it to its SPSid
	 *	 3) The last bspp_H264SeiBufferingPeriod sets it, and it is used
	 * for every SEI parsing until updated by another
	 * bspp_H264SeiBufferingPeriod message
	 */
	unsigned int active_sps_for_sei_parsing;
	unsigned short current_view_id;
	struct vdec_raw_bstr_data *sei_raw_data_list;
};

/* This structure contains HEVC state to be retained between pictures. */
struct bspp_hevc_inter_pict_ctx {
	/* Picture count in a sequence */
	unsigned int seq_pic_count;
	struct {
		/* There was EOS NAL detected and no new picture yet */
		unsigned eos_detected : 1;
		/* This is first picture after EOS NAL */
		unsigned first_after_eos : 1;
	};

	/* control variable to decide when to attach the SEI info
	 * (picture properties) to a picture.
	 */
	unsigned char sei_info_attached_to_pic;
	/* Raw SEI list to be attached to a picture. */
	struct vdec_raw_bstr_data *sei_rawdata_list;
	/* Handle to a picture header field to attach the raw SEI list to. */
	void **hndl_pichdr_sei_rawdata_list;
};

/*
 * struct bspp_inter_pict_data
 * @Brief	This structure contains state to be retained between pictures.
 */
struct bspp_inter_pict_data {
	/* A closed GOP has occurred in the bitstream. */
	int seen_closed_gop;
	/* Closed GOP has been signaled by a unit before the next picture */
	int new_closed_gop;
	/* Indicates whether or not DPB flush is needed */
	int not_dpb_flush;
	struct lst_t pic_prefix_seg;
	union {
		struct bspp_h264_inter_pict_ctx h264_ctx;
		struct bspp_hevc_inter_pict_ctx hevc_ctx;
	};
};

/*
 * struct bspp_parse_state
 * @Brief	This structure contains parse state
 */
struct bspp_parse_state {
	struct bspp_inter_pict_data *inter_pict_ctx;
	int initialised;

	/* Input/Output (H264 etc. state). */
	/* For SCP ASO detection we need to log 3 components */
	unsigned int prev_first_mb_in_slice[MAX_COMPONENTS];
	struct bspp_pict_hdr_info *next_pict_hdr_info;
	unsigned char prev_bottom_pic_flag;
	unsigned char second_field_flag;
	unsigned char next_pic_is_new;
	unsigned int prev_frame_num;
	unsigned int prev_pps_id;
	unsigned int prev_field_pic_flag;
	unsigned int prev_nal_ref_idc;
	unsigned int prev_pic_order_cnt_lsb;
	int prev_delta_pic_order_cnt_bottom;
	int prev_delta_pic_order_cnt[2];
	int prev_nal_unit_type;
	int prev_idr_pic_id;
	int discontinuous_mb;
	/* Position in bitstream before parsing a unit */
	unsigned long long prev_byte_offset_buf;
	unsigned int prev_buf_map_id;
	unsigned int prev_buf_data_size;
	/*
	 * !< Flags word to indicate error in parsing/decoding
	 * - see #VDEC_eErrorType.
	 */
	unsigned int error_flags;
	/* Outputs. */
	int new_closed_gop;
	unsigned char new_view;
	unsigned char is_prefix;
	int first_chunk;
};

/*
 * struct bspp_pps_info
 * @Brief	Contains PPS information
 */
struct bspp_pps_info {
	void **lst_link;
	/* PPS Id. INSECURE MEMORY HOST */
	unsigned int pps_id;
	/* Reference count for PPS. INSECURE MEMORY HOST */
	unsigned int ref_count;
	struct bspp_ddbuf_array_info fw_pps;
	/* Buffer ID to be used in Kernel */
	unsigned int bufmap_id;
	/* Parsing Info.    SECURE MEMORY HOST   */
	void *secure_pps_info;
	/* Buffer Offset to be used in kernel */
	unsigned int buf_offset;
};

/*
 * struct bspp_sequence_hdr_info
 * @Brief	Contains SPS information
 */
struct bspp_sequence_hdr_info {
	void **lst_link;
	/* Reference count for sequence header */
	unsigned int ref_count;
	struct bspp_sequ_hdr_info sequ_hdr_info;
	struct bspp_ddbuf_array_info fw_sequence;
	/* Parsing Info.  SECURE MEMORY HOST */
	void *secure_sequence_info;
};

enum bspp_element_status {
	BSPP_UNALLOCATED = 0,
	BSPP_AVAILABLE,
	BSPP_UNAVAILABLE,
	BSPP_STATUSMAX,
	BSPP_FORCE32BITS = 0x7FFFFFFFU
};

struct bspp_vps_info {
	void **lst_link;
	/* VPS Id   INSECURE MEMORY HOST */
	unsigned int vps_id;
	/* Reference count for video header. INSECURE MEMORY HOST */
	unsigned int ref_count;
	/*!< Parsing Info. SECURE MEMORY HOST */
	void *secure_vpsinfo;
};

/*
 * struct bspp_unit_data
 * @Brief	Contains bitstream unit data
 */
struct bspp_unit_data {
	/* Input. */
	/* Indicates which output data to populate */
	enum bspp_unit_type unit_type;
	/* Video Standard of unit to parse */
	enum vdec_vid_std vid_std;
	/* Indicates whether delimiter is present for unit */
	int delim_present;
	/* Codec configuration used by this stream */
	const struct vdec_codec_config *codec_config;
	void *str_res_handle;
	/* Needed for calculating the size of the last fragment */
	unsigned int unit_data_size;
	/* Input/Output. */
	struct bspp_parse_state *parse_state;
	/* Output */
	/* eVidStd == VDEC_STD_H263 && BSPP_UNIT_PICTURE. */
	struct bspp_sequence_hdr_info *impl_sequ_hdr_info;
	/* Union of output data for each of the unit types. */
	union {
		/* BSPP_UNIT_SEQUENCE. */
		struct bspp_sequence_hdr_info *sequ_hdr_info;
		/* BSPP_UNIT_PPS. */
		struct bspp_pps_info *pps_info;
		/* BSPP_UNIT_PICTURE. */
		struct bspp_pict_hdr_info *pict_hdr_info;
		/* For Video Header (HEVC) */
		struct bspp_vps_info *vps_info;
	} out;

	/*
	 * For picture it should give the SequenceHdrId, for anything
	 * else it should contain BSPP_INVALID. This value is pre-loaded
	 * with the sequence ID of the last picture.
	 */
	unsigned int pict_sequ_hdr_id;
	/* State: output. */
	/*
	 * Picture unit (BSPP_UNIT_PICTURE) contains slice data.
	 * Picture header information must be populated once this unit has been
	 * parsed.
	 */
	int slice;
	int ext_slice; /* Current slice belongs to non-base view (MVC only) */
	/*
	 * True if we meet a unit that signifies closed gop, different
	 * for each standard.
	 */
	int new_closed_gop;
	/* True if the end of a sequence of pictures has been reached. */
	int sequence_end;
	/*
	 * Extracted all data from unit whereby shift-register should now
	 * be at the next delimiter or end of data (when byte-aligned).
	 */
	int extracted_all_data;
	/* Indicates the presence of any errors while processing this unit. */
	enum bspp_error_type parse_error;
	/* To turn on/off considering I-Frames as ClosedGop boundaries. */
	int intra_frm_as_closed_gop;

	/*
	 * constrain the amount of DPB's allowed
	 * a value of 0 means let the firmware decide
	 */
	unsigned int max_dec_frame_buffering;
};

/*
 * struct bspp_swsr_ctx
 * @brief	BSPP Software Shift Register Context Information
 */
struct bspp_swsr_ctx {
	/*
	 * Default configuration for the shift-register for this
	 * stream. The delimiter type may be adjusted for each unit
	 * where the buffer requires it. Information about how to
	 * process each unit will be passed down with the picture
	 * header information.
	 */
	struct swsr_config sr_config;
	/*
	 * Emulation prevention scheme present in bitstream. This is
	 * sometimes not ascertained (e.g. VC-1) until the first
	 * bitstream buffer (often codec configuration) has been
	 * received.
	 */
	enum swsr_emprevent emulation_prevention;
	/* Software shift-register context. */
	void *swsr_context;
};

/*
 * struct bspp_vid_std_features
 * @brief  BSPP Video Standard Specific Features and Information
 */
struct bspp_vid_std_features {
	/* The size of the sequence header structure for this video standard */
	unsigned long seq_size;
	/* This video standard uses Picture Parameter Sets. */
	int uses_pps;
	/*
	 * The size of the Picture Parameter Sets structure for
	 * this video standard.
	 */
	unsigned long pps_size;
	/* This video standard uses Video Parameter Sets. */
	int uses_vps;
	/*
	 * The size of the Video Parameter Sets structure for
	 * this video standard
	 */
	unsigned long vps_size;
};

/*
 * @Function	bspp_cb_parse_unit
 * @Description	Function prototype for the parse unit callback functions.
 * @Input	swsr_context_handle: A handle to software shift-register context
 * @InOut	unit_data: A pointer to unit data which includes input & output
 *		parameters as defined by structure.
 * @Return	int : This function returns either IMG_SUCCESS or an error code.
 */
typedef int (*bspp_cb_parse_unit)(void *swsr_context_handle,
				    struct bspp_unit_data *unit_data);

/*
 * @Function	bspp_pfnReleaseData
 * @Description	This is a function prototype for the data releasing callback
 *		functions.
 * @Input	str_alloc_handle   : A handle to stream related resources.
 * @Input	data_type   : A type of data which is to be released.
 * @Input	data_handle : A handle for data which is to be released.
 * @Return	int : This function returns either IMG_SUCCESS or an error code.
 */
typedef int (*bspp_cb_release_data)(void *str_alloc_handle,
				      enum bspp_unit_type data_type,
				      void *data_handle);

/*
 * @Function	bspp_cb_reset_data
 * @Description	This is a function prototype for the data resetting callback
 *		functions.
 * @Input	data_type   : A type of data which is to be reset.
 * @InOut	data_handle : A handle for data which is to be reset.
 * @Return	int : This function returns either IMG_SUCCESS or an error code.
 */
typedef int (*bspp_cb_reset_data)(enum bspp_unit_type data_type,
				    void *data_handle);

/*
 * @Function	bspp_cb_destroy_data
 * @Description	This is a function prototype for the data destruction callback
 *		functions.
 * @Input	data_type   : A type of data which is to be destroyed.
 * @InOut	data_handle : A handle for data which is to be destroyed.
 * @Return	int : This function returns either IMG_SUCCESS or an error code.
 */
typedef int (*bspp_cb_destroy_data)(enum bspp_unit_type data_type,
				      void *data_handle);

/*
 * @Function	bspp_cb_parse_codec_config
 * @Description	This is a function prototype for parsing codec config bitstream
 *		element for size delimited bitstreams.
 * @Input	swsr_context_handle: A handle to Shift Register processing
 *		current bitstream.
 * @Output	unit_count: A pointer to variable in which to return unit count.
 * @Output	unit_array_count: A pointer to variable in which to return unit
 *		array count.
 * @Output	delim_length: A pointer to variable in which to return NAL
 *		delimiter length in bits.
 * @Output	size_delim_length: A pointer to variable in which to return size
 *		delimiter length in bits.
 * @Return	None.
 */
typedef void (*bspp_cb_parse_codec_config)(void *swsr_context_handle,
					   unsigned int *unit_count,
					   unsigned int *unit_array_count,
					   unsigned int *delim_length,
					   unsigned int *size_delim_length);

/*
 * @Function	bspp_cb_update_unit_counts
 * @Description	This is a function prototype for updating unit counts for size
 *		delimited bitstreams.
 * @Input	swsr_context_handle: A handle to Shift Register processing
 *		current bitstream.
 * @InOut	unit_count: A pointer to variable holding current unit count
 * @InOut	unit_array_count: A pointer to variable holding current unit
 *		array count.
 * @Return	None.
 */
typedef void (*bspp_cb_update_unit_counts)(void *swsr_context_handle,
					   unsigned int *unit_count,
					   unsigned int *unit_array_count);

/*
 * @Function	bspp_cb_initialise_parsing
 * @Description	This prototype is for unit group parsing initialization.
 * @InOut	parse_state: The current unit group parsing state.
 * @Return	None.
 */
typedef void (*bspp_cb_initialise_parsing)(struct bspp_parse_state *prs_state);

/*
 * @Function	bspp_cb_finalise_parsing
 * @Description	This is prototype is for unit group parsing finalization.
 * @Input	str_alloc_handle: A handle to stream related resources.
 * @InOut	parse_state: The current unit group parsing state.
 * @Return	None.
 */
typedef void (*bspp_cb_finalise_parsing)(void *str_alloc_handle,
					 struct bspp_parse_state *parse_state);

/*
 * struct bspp_parser_callbacks
 * @brief	BSPP Standard Related Parser Callback Functions
 */
struct bspp_parser_callbacks {
	/* Pointer to standard-specific unit parsing callback function. */
	bspp_cb_parse_unit parse_unit_cb;
	/* Pointer to standard-specific data releasing callback function. */
	bspp_cb_release_data release_data_cb;
	/* Pointer to standard-specific data resetting callback function. */
	bspp_cb_reset_data reset_data_cb;
	/* Pointer to standard-specific data destruction callback function. */
	bspp_cb_destroy_data destroy_data_cb;
	/* Pointer to standard-specific codec config parsing callback function */
	bspp_cb_parse_codec_config parse_codec_config_cb;
	/* Pointer to standard-specific unit count updating callback function */
	bspp_cb_update_unit_counts update_unit_counts_cb;
	/*
	 * Pointer to standard-specific unit group parsing initialization
	 * function.
	 */
	bspp_cb_initialise_parsing initialise_parsing_cb;
	/*
	 * Pointer to standard-specific unit group parsing finalization
	 * function
	 */
	bspp_cb_finalise_parsing finalise_parsing_cb;
};

/*
 * @Function	bspp_cb_set_parser_config
 * @Description	Prototype is for the setting parser config callback functions.
 * @Input	bstr_format: Input bitstream format.
 * @Output	vid_std_features: Features of video standard for this bitstream.
 * @Output	swsr_ctx: Software Shift Register settings for this bitstream.
 * @Output	parser_callbacks: Parser functions to be used for parsing this
 *		bitstream.
 * @Output	inter_pict_data: Inter-picture settings specific for this
 *		bitstream.
 * @Return	int : This function returns either IMG_SUCCESS or an error code.
 */
typedef int (*bspp_cb_set_parser_config)(enum vdec_bstr_format bstr_format,
					   struct bspp_vid_std_features *vid_std_features,
					   struct bspp_swsr_ctx *swsr_ctx,
					   struct bspp_parser_callbacks *parser_callbacks,
					   struct bspp_inter_pict_data *inter_pict_data);

/*
 * @Function	bspp_cb_determine_unit_type
 * @Description	This is a function prototype for determining the BSPP unit type
 *		based on the bitstream (video standard specific) unit type
 *		callback functions.
 * @Input	bitstream_unit_type: Bitstream (video standard specific) unit
 *		type.
 * @Input	disable_mvc: Skip MVC related units (relevant for standards
 *		that support it).
 * @InOut	bspp_unit_type *: Last BSPP unit type on input. Current BSPP
 *		unit type on output.
 * @Return	None.
 */
typedef void (*bspp_cb_determine_unit_type)(unsigned char bitstream_unit_type,
					    int disable_mvc,
					    enum bspp_unit_type *bspp_unit_type);

struct bspp_pps_info *bspp_get_pps_hdr(void *str_res_handle, unsigned int pps_id);

struct bspp_sequence_hdr_info *bspp_get_sequ_hdr(void *str_res_handle,
						 unsigned int sequ_id);

struct bspp_vps_info *bspp_get_vpshdr(void *str_res, unsigned int vps_id);

void bspp_streamrelese_rawbstrdataplain(const void *str_res,
					const void *rawdata);

void bspp_freeraw_sei_datacontainer(const void *str_res,
				    struct vdec_raw_bstr_data *rawsei_datacontainer);

void bspp_freeraw_sei_datalist(const void *str_res,
			       struct vdec_raw_bstr_data *rawsei_datalist);

#endif /* __BSPP_INT_H__   */
