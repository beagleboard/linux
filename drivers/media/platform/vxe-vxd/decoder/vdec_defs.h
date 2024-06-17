/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD Decoder common header
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

#ifndef __VDEC_DEFS_H__
#define __VDEC_DEFS_H__

#include "img_mem.h"
#include "img_pixfmts.h"
#ifdef HAS_JPEG
#include "jpegfw_data.h"
#endif
#include "pixel_api.h"
#include "vdecfw_shared.h"

#define VDEC_MAX_PANSCAN_WINDOWS        4
#define VDEC_MB_DIMENSION               (16)

#define MAX_PICS_IN_SYSTEM      (8)
#define SEQUENCE_SLOTS          (8)
#define PPS_SLOTS               (8)
/* Only for HEVC */
#define VPS_SLOTS               (16)
#define MAX_VPSS                (MAX_PICS_IN_SYSTEM + VPS_SLOTS)
#define MAX_SEQUENCES           (MAX_PICS_IN_SYSTEM + SEQUENCE_SLOTS)
#define MAX_PPSS                (MAX_PICS_IN_SYSTEM + PPS_SLOTS)

#define VDEC_H264_MAXIMUMVALUEOFCPB_CNT 32
#define VDEC_H264_MVC_MAX_VIEWS         (H264FW_MAX_NUM_VIEWS)

#define VDEC_ASSERT(expected) ({ WARN_ON(!(expected)); 0; })

#define VDEC_ALIGN_SIZE(_val, _alignment, val_type, align_type) \
	({ \
		val_type val = _val; \
		align_type alignment = _alignment; \
		(((val) + (alignment) - 1) & ~((alignment) - 1)); })

/*
 * This type defines the video standard.
 * @brief  VDEC Video Standards
 */
enum vdec_vid_std {
	VDEC_STD_UNDEFINED = 0,
	VDEC_STD_MPEG2,
	VDEC_STD_MPEG4,
	VDEC_STD_H263,
	VDEC_STD_H264,
	VDEC_STD_VC1,
	VDEC_STD_AVS,
	VDEC_STD_REAL,
	VDEC_STD_JPEG,
	VDEC_STD_VP6,
	VDEC_STD_VP8,
	VDEC_STD_SORENSON,
	VDEC_STD_HEVC,
	VDEC_STD_MAX,
	VDEC_STD_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This type defines the bitstream format. Should be done at the
 * start of decoding.
 * @brief  VDEC Bitstream Format
 */
enum vdec_bstr_format {
	VDEC_BSTRFORMAT_UNDEFINED = 0,
	VDEC_BSTRFORMAT_ELEMENTARY,
	VDEC_BSTRFORMAT_DEMUX_BYTESTREAM,
	VDEC_BSTRFORMAT_DEMUX_SIZEDELIMITED,
	VDEC_BSTRFORMAT_MAX,
	VDEC_BSTRFORMAT_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This type defines the Type of payload. Could change with every buffer.
 * @brief  VDEC Bitstream Element Type
 */
enum vdec_bstr_element_type {
	VDEC_BSTRELEMENT_UNDEFINED = 0,
	VDEC_BSTRELEMENT_UNSPECIFIED,
	VDEC_BSTRELEMENT_CODEC_CONFIG,
	VDEC_BSTRELEMENT_PICTURE_DATA,
	VDEC_BSTRELEMENT_MAX,
	VDEC_BSTRELEMENT_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This structure contains the stream configuration details.
 * @brief  VDEC Stream Configuration Information
 */
struct vdec_str_configdata {
	enum vdec_vid_std vid_std;
	enum vdec_bstr_format bstr_format;
	unsigned int user_str_id;
	unsigned char update_yuv;
	unsigned char bandwidth_efficient;
	unsigned char disable_mvc;
	unsigned char full_scan;
	unsigned char immediate_decode;
	unsigned char intra_frame_closed_gop;
};

/*
 * This type defines the buffer type categories.
 * @brief  Buffer Types
 */
enum vdec_buf_type {
	VDEC_BUFTYPE_BITSTREAM,
	VDEC_BUFTYPE_PICTURE,
	VDEC_BUFTYPE_ALL,
	VDEC_BUFTYPE_MAX,
	VDEC_BUFTYPE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This structure contains information related to a picture plane.
 * @brief  Picture Plane Information
 */
struct vdec_plane_info {
	unsigned int offset;
	unsigned int stride;
	unsigned int size;
};

/*
 * This structure describes the VDEC picture dimensions.
 * @brief  VDEC Picture Size
 */
struct vdec_pict_size {
	unsigned int width;
	unsigned int height;
};

/*
 * This enumeration defines the colour plane indices.
 * @brief  Colour Plane Indices
 */
enum vdec_color_planes {
	VDEC_PLANE_VIDEO_Y     = 0,
	VDEC_PLANE_VIDEO_YUV   = 0,
	VDEC_PLANE_VIDEO_U     = 1,
	VDEC_PLANE_VIDEO_UV    = 1,
	VDEC_PLANE_VIDEO_V     = 2,
	VDEC_PLANE_VIDEO_A     = 3,
	VDEC_PLANE_LIGHT_R     = 0,
	VDEC_PLANE_LIGHT_G     = 1,
	VDEC_PLANE_LIGHT_B     = 2,
	VDEC_PLANE_INK_C       = 0,
	VDEC_PLANE_INK_M       = 1,
	VDEC_PLANE_INK_Y       = 2,
	VDEC_PLANE_INK_K       = 3,
	VDEC_PLANE_MAX         = 4,
	VDEC_PLANE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This structure describes the rendered region of a picture buffer (i.e. where
 * the image data is written.
 * @brief  Picture Buffer Render Information
 */
struct vdec_pict_rendinfo {
	unsigned int rendered_size;
	struct vdec_plane_info plane_info[VDEC_PLANE_MAX];
	unsigned int stride_alignment;
	struct vdec_pict_size rend_pict_size;
};

/*
 * This structure contains information required to configure the picture
 * buffers
 * @brief  Picture Buffer Configuration
 */
struct vdec_pict_bufconfig {
	unsigned int coded_width;
	unsigned int coded_height;
	enum img_pixfmt pixel_fmt;
	unsigned int stride[IMG_MAX_NUM_PLANES];
	unsigned int stride_alignment;
	unsigned char byte_interleave;
	unsigned int buf_size;
	unsigned char packed;
	unsigned int chroma_offset[IMG_MAX_NUM_PLANES];
	unsigned int plane_size[IMG_MAX_NUM_PLANES];
};

/*
 * This structure describes the VDEC Display Rectangle.
 * @brief  VDEC Display Rectangle
 */
struct vdec_rect {
	unsigned int top_offset;
	unsigned int left_offset;
	unsigned int width;
	unsigned int height;
};

/*
 * This structure contains the Color Space Description that may be present
 * in SequenceDisplayExtn(MPEG2), VUI parameters(H264), Visual Object(MPEG4)
 * for the application to use.
 * @brief  Stream Color Space Properties
 */
struct vdec_color_space_desc {
	unsigned char is_present;
	unsigned char color_primaries;
	unsigned char transfer_characteristics;
	unsigned char matrix_coefficients;
};

/*
 * This structure contains common (standard agnostic) sequence header
 * information, which is required for image buffer allocation and display.
 * @brief  Sequence Header Information (common)
 */
struct vdec_comsequ_hdrinfo {
	unsigned int codec_profile;
	unsigned int codec_level;
	unsigned int bitrate;
	long frame_rate;
	unsigned int frame_rate_num;
	unsigned int frame_rate_den;
	unsigned int aspect_ratio_num;
	unsigned int aspect_ratio_den;
	unsigned char interlaced_frames;
	struct pixel_pixinfo pixel_info;
	struct vdec_pict_size max_frame_size;
	unsigned int max_ref_frame_num;
	struct vdec_pict_size frame_size;
	unsigned char field_codec_mblocks;
	unsigned int min_pict_buf_num;
	unsigned char picture_reordering;
	unsigned char post_processing;
	struct vdec_rect orig_display_region;
	struct vdec_rect raw_display_region;
	unsigned int num_views;
	unsigned int max_reorder_picts;
	unsigned char separate_chroma_planes;
	unsigned char not_dpb_flush;
	struct vdec_color_space_desc color_space_info;
};

/*
 * This structure contains the standard specific codec configuration
 * @brief Codec configuration
 */
struct vdec_codec_config {
	unsigned int default_height;
	unsigned int default_width;
};

/*
 * This structure describes the decoded picture attributes (relative to the
 * encoded, where necessary, e.g. rotation angle).
 * @brief  Stream Output Configuration
 */
struct vdec_str_opconfig {
	struct pixel_pixinfo pixel_info;
	unsigned char force_oold;
};

/*
 * This type defines the "play" mode.
 * @brief  Play Mode
 */
enum vdec_play_mode {
	VDEC_PLAYMODE_PARSE_ONLY,
	VDEC_PLAYMODE_NORMAL_DECODE,
	VDEC_PLAYMODE_MAX,
	VDEC_PLAYMODE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This type defines the bitstream processing error info.
 * @brief  Bitstream Processing Error Info
 */
struct vdec_bstr_err_info {
	unsigned int sequence_err;
	unsigned int picture_err;
	unsigned int other_err;
};

/*
 * This structure describes the VDEC Pan Scan Window.
 * @brief  VDEC Pan Scan Window
 */
struct vdec_window {
	unsigned int ui32topoffset;
	unsigned int ui32leftoffset;
	unsigned int ui32width;
	unsigned int ui32height;
};

/*
 * This structure contains the VDEC picture display properties.
 * @brief  VDEC Picture Display Properties
 */
struct vdec_pict_disp_info {
	struct vdec_rect enc_disp_region;
	struct vdec_rect disp_region;
	struct vdec_rect raw_disp_region;
	unsigned char top_fld_first;
	unsigned char out_top_fld_first;
	unsigned int max_frm_repeat;
	unsigned int repeat_first_fld;
	unsigned int num_pan_scan_windows;
	struct vdec_window pan_scan_windows[VDEC_MAX_PANSCAN_WINDOWS];
};

/*
 * This structure contains VXD hardware signatures.
 * @brief  VXD Hardware signatures
 */
struct vdec_pict_hwcrc {
	unsigned char first_fld_rcvd;
	unsigned int crc_vdmc_pix_recon;
	unsigned int vdeb_sysmem_wrdata;
};

struct vdec_features {
	unsigned char valid;
	unsigned char mpeg2;
	unsigned char mpeg4;
	unsigned char h264;
	unsigned char vc1;
	unsigned char avs;
	unsigned char real;
	unsigned char jpeg;
	unsigned char vp6;
	unsigned char vp8;
	unsigned char hevc;
	unsigned char hd;
	unsigned char rotation;
	unsigned char scaling;
	unsigned char scaling_oold;
	unsigned char scaling_extnd_strides;
};

/*
 * This type defines the auxiliary info for picture queued for decoding.
 * @brief  Auxiliary Decoding Picture Info
 */
struct vdec_dec_pict_auxinfo {
	unsigned int seq_hdr_id;
	unsigned int pps_id;
	unsigned int second_pps_id;
	unsigned char not_decoded;
};

/*
 * This type defines the decoded picture state.
 * @brief  Decoded Picture State
 */
enum vdec_pict_state {
	VDEC_PICT_STATE_NOT_DECODED,
	VDEC_PICT_STATE_DECODED,
	VDEC_PICT_STATE_TERMINATED,
	VDEC_PICT_STATE_MAX,
	VDEC_PICT_STATE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This type defines the container for various picture tags.
 * @brief  Picture Tag Container
 */
struct vdec_pict_tag_container {
	enum img_buffer_type pict_type;
	unsigned long long pict_tag_param;
	unsigned long long sideband_info;
	struct vdec_pict_hwcrc pict_hwcrc;
};

/*
 * This structure describes raw bitstream data chunk.
 * @brief  Raw Bitstream Data Chunk
 */
struct vdec_raw_bstr_data {
	unsigned int size;
	unsigned int bit_offset;
	unsigned char *data;
	struct vdec_raw_bstr_data *next;
};

/*
 * This type defines the supplementary picture data.
 * @brief  Supplementary Picture Data
 */
struct vdec_pict_supl_data {
	struct vdec_raw_bstr_data *raw_vui_data;
	struct vdec_raw_bstr_data *raw_sei_list_first_fld;
	struct vdec_raw_bstr_data *raw_sei_list_second_fld;
	union {
		struct h264_pict_supl_data {
			unsigned char nal_ref_idc;
			unsigned short frame_num;
		} data;
	};
};

/*
 * This structure contains decoded picture information for display.
 * @brief  Decoded Picture Information
 */
struct vdec_dec_pict_info {
	enum vdec_pict_state pict_state;
	enum img_buffer_type buf_type;
	unsigned char interlaced_flds;
	unsigned int err_flags;
	unsigned int err_level;
	struct vdec_pict_tag_container first_fld_tag_container;
	struct vdec_pict_tag_container second_fld_tag_container;
	struct vdec_str_opconfig op_config;
	struct vdec_pict_rendinfo rend_info;
	struct vdec_pict_disp_info disp_info;
	unsigned int last_in_seq;
	unsigned int decode_id;
	unsigned int id_for_hwcrc_chk;
	unsigned short view_id;
	unsigned int timestamp;
	struct vdec_pict_supl_data pict_supl_data;
};

struct vdec_pict_rend_config {
	struct vdec_pict_size coded_pict_size;
	unsigned char packed;
	unsigned char byte_interleave;
	unsigned int stride_alignment;
};

/*
 * This structure contains unsupported feature flags.
 * @brief  Unsupported Feature Flags
 */
struct vdec_unsupp_flags {
	unsigned int str_cfg;
	unsigned int str_opcfg;
	unsigned int op_bufcfg;
	unsigned int seq_hdr;
	unsigned int pict_hdr;
};

/*
 * This type defines the error , error in parsing, error in decoding etc.
 * @brief  VDEC parsing/decoding error  Information
 */
enum vdec_error_type {
	VDEC_ERROR_NONE                 = (0),
	VDEC_ERROR_SR_ERROR             = (1 << 0),
	VDEC_ERROR_FEHW_TIMEOUT         = (1 << 1),
	VDEC_ERROR_FEHW_DECODE          = (1 << 2),
	VDEC_ERROR_BEHW_TIMEOUT         = (1 << 3),
	VDEC_ERROR_SERVICE_TIMER_EXPIRY = (1 << 4),
	VDEC_ERROR_MISSING_REFERENCES   = (1 << 5),
	VDEC_ERROR_MMU_FAULT            = (1 << 6),
	VDEC_ERROR_DEVICE               = (1 << 7),
	VDEC_ERROR_CORRUPTED_REFERENCE  = (1 << 8),
	VDEC_ERROR_MMCO                 = (1 << 9),
	VDEC_ERROR_MBS_DROPPED          = (1 << 10),
	VDEC_ERROR_MAX                  = (1 << 11),
	VDEC_ERROR_FORCE32BITS          = 0x7FFFFFFFU
};

/*
 * This structure contains information relating to a buffer.
 * @brief  Buffer Information
 */
struct vdec_buf_info {
	void *cpu_linear_addr;
	unsigned int buf_id;
	struct vdec_pict_bufconfig pictbuf_cfg;
	int fd;
	/* The following are fields used internally within VDEC... */
	unsigned int buf_size;
	enum sys_emem_attrib mem_attrib;
	void *buf_alloc_handle;
	void *buf_map_handle;
	unsigned long  dma_addr;
};

#ifdef HAS_JPEG
/*
 * This structure contains JPEG sequence header information.
 * NOTE: Should only contain JPEG specific information.
 * @brief  JPEG sequence header Information
 */
struct vdec_jpeg_sequ_hdr_info {
	/* total component in jpeg */
	unsigned char num_component;
	/* precision */
	unsigned char precision;
};

/*
 * This structure contains JPEG start of frame segment header
 * NOTE: Should only contain JPEG specific information.
 * @brief  JPEG SOF header Information
 */
struct vdec_jpeg_sof_component_hdr {
	/* component identifier. */
	unsigned char identifier;
	/* Horizontal scaling. */
	unsigned char horz_factor;
	/* Verticale scaling */
	unsigned char vert_factor;
	/* Qunatisation tables . */
	unsigned char quant_table;
};

/*
 * This structure contains JPEG start of scan segment header
 * NOTE: Should only contain JPEG specific information.
 * @brief  JPEG SOS header Information
 */
struct vdec_jpeg_sos_component_hdr {
	/* component identifier. */
	unsigned char component_index;
	/* Huffman DC tables. */
	unsigned char dc_table;
	/* Huffman AC table .*/
	unsigned char ac_table;
};

struct vdec_jpeg_pict_hdr_info {
	/* Start of frame component header */
	struct vdec_jpeg_sof_component_hdr sof_comp[JPEG_VDEC_MAX_COMPONENTS];
	/* Start of Scan component header */
	struct vdec_jpeg_sos_component_hdr sos_comp[JPEG_VDEC_MAX_COMPONENTS];
	/* Huffman tables */
	struct vdec_jpeg_huffman_tableinfo huff_tables[JPEG_VDEC_TABLE_CLASS_NUM]
						[JPEG_VDEC_MAX_SETS_HUFFMAN_TABLES];
	/* Quantization tables */
	struct vdec_jpeg_de_quant_tableinfo quant_tables[JPEG_VDEC_MAX_QUANT_TABLES];
	/* Number of MCU in the restart interval */
	unsigned short interval;
	unsigned int test;
};
#endif

#endif
