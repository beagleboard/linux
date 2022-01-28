/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave5 series multi-standard codec IP - helper definitions
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#ifndef VPUAPI_H_INCLUDED
#define VPUAPI_H_INCLUDED

#include <linux/kfifo.h>
#include <linux/idr.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ctrls.h>
#include "wave5-vpuerror.h"
#include "wave5-vpuconfig.h"
#include "wave5-vdi.h"

enum product_id {
	PRODUCT_ID_521,
	PRODUCT_ID_511,
	PRODUCT_ID_517,
	PRODUCT_ID_NONE,
};

struct vpu_attr;

enum vpu_instance_type {
	VPU_INST_TYPE_DEC = 0,
	VPU_INST_TYPE_ENC = 1
};

enum vpu_instance_state {
	VPU_INST_STATE_NONE = 0,
	VPU_INST_STATE_OPEN = 1,
	VPU_INST_STATE_INIT_SEQ = 2,
	VPU_INST_STATE_PIC_RUN = 3,
	VPU_INST_STATE_STOP = 4,
	VPU_INST_STATE_WAIT_BUF = 5
};

#define WAVE5_MAX_FBS 32

#define MAX_REG_FRAME (WAVE5_MAX_FBS * 2)

#define WAVE5_DEC_HEVC_BUF_SIZE(_w, _h) (DIV_ROUND_UP(_w, 64) * DIV_ROUND_UP(_h, 64) * 256 + 64)
#define WAVE5_DEC_AVC_BUF_SIZE(_w, _h) ((((ALIGN(_w, 256) / 16) * (ALIGN(_h, 16) / 16)) + 16) * 80)
#define WAVE5_DEC_VP9_BUF_SIZE(_w, _h) (((ALIGN(_w, 64) * ALIGN(_h, 64)) >> 2))
#define WAVE5_DEC_AVS2_BUF_SIZE(_w, _h) (((ALIGN(_w, 64) * ALIGN(_h, 64)) >> 5))
// AV1 BUF SIZE : MFMV + segment ID + CDF probs table + film grain param Y+ film graim param C
#define WAVE5_DEC_AV1_BUF_SZ_1(_w, _h)	\
	(((ALIGN(_w, 64) / 64) * (ALIGN(_h, 64) / 64) * 512) + 41984 + 8192 + 4864)
#define WAVE5_DEC_AV1_BUF_SZ_2(_w1, _w2, _h)	\
	(((ALIGN(_w1, 64) / 64) * 256 + (ALIGN(_w2, 256) / 64) * 128) * (ALIGN(_h, 64) / 64))

#define WAVE5_FBC_LUMA_TABLE_SIZE(_w, _h) (ALIGN(_h, 64) * ALIGN(_w, 256) / 32)
#define WAVE5_FBC_CHROMA_TABLE_SIZE(_w, _h) (ALIGN((_h), 64) * ALIGN((_w) / 2, 256) / 32)
#define WAVE5_ENC_AVC_BUF_SIZE(_w, _h) (ALIGN(_w, 64) * ALIGN(_h, 64) / 32)
#define WAVE5_ENC_HEVC_BUF_SIZE(_w, _h) (ALIGN(_w, 64) / 64 * ALIGN(_h, 64) / 64 * 128)

/*
 * common struct and definition
 */
enum cod_std {
	STD_AVC = 0,
	STD_VC1 = 1,
	STD_MPEG2 = 2,
	STD_MPEG4 = 3,
	STD_H263 = 4,
	STD_DIV3 = 5,
	STD_RV = 6,
	STD_AVS = 7,
	STD_THO = 9,
	STD_VP3 = 10,
	STD_VP8 = 11,
	STD_HEVC = 12,
	STD_VP9 = 13,
	STD_AVS2 = 14,
	STD_AV1 = 16,
	STD_MAX
};

enum wave_std {
	W_HEVC_DEC = 0x00,
	W_HEVC_ENC = 0x01,
	W_AVC_DEC = 0x02,
	W_AVC_ENC = 0x03,
	W_VP9_DEC = 0x16,
	W_AVS2_DEC = 0x18,
	W_AV1_DEC = 0x1A,
	STD_UNKNOWN = 0xFF
};

enum SET_PARAM_OPTION {
	OPT_COMMON = 0, /* SET_PARAM command option for encoding sequence */
	OPT_CUSTOM_GOP = 1, /* SET_PARAM command option for setting custom GOP */
	OPT_CUSTOM_HEADER = 2, /* SET_PARAM command option for setting custom VPS/SPS/PPS */
	OPT_VUI = 3, /* SET_PARAM command option for encoding VUI */
	OPT_CHANGE_PARAM = 0x10,
};

enum DEC_PIC_HDR_OPTION {
	INIT_SEQ_NORMAL = 0x01,
	INIT_SEQ_W_THUMBNAIL = 0x11,
};

enum DEC_PIC_OPTION {
	DEC_PIC_NORMAL = 0x00, /* it is normal mode of DEC_PIC command. */
	DEC_PIC_W_THUMBNAIL = 0x10, /* thumbnail mode (skip non-IRAP without reference reg.) */
	SKIP_NON_IRAP = 0x11, /* it skips to decode non-IRAP pictures. */
	SKIP_NON_REF_PIC = 0x13
};

/************************************************************************/
/* PROFILE & LEVEL */
/************************************************************************/
/* HEVC */
#define HEVC_PROFILE_MAIN 1
#define HEVC_PROFILE_MAIN10 2
#define HEVC_PROFILE_STILLPICTURE 3
#define HEVC_PROFILE_MAIN10_STILLPICTURE 2

/* H.264 profile for encoder*/
#define H264_PROFILE_BP 1
#define H264_PROFILE_MP 2
#define H264_PROFILE_EXTENDED 3
#define H264_PROFILE_HP 4
#define H264_PROFILE_HIGH10 5
#define H264_PROFILE_HIGH422 6
#define H264_PROFILE_HIGH444 7

/************************************************************************/
/* error codes */
/************************************************************************/

/************************************************************************/
/* utility macros */
/************************************************************************/

/************************************************************************/
/* */
/************************************************************************/
/**
 * \brief parameters of DEC_SET_SEQ_CHANGE_MASK
 */
#define SEQ_CHANGE_ENABLE_PROFILE BIT(5)
#define SEQ_CHANGE_CHROMA_FORMAT_IDC BIT(15) /* AV1 */
#define SEQ_CHANGE_ENABLE_SIZE BIT(16)
#define SEQ_CHANGE_INTER_RES_CHANGE BIT(17) /* VP9 */
#define SEQ_CHANGE_ENABLE_BITDEPTH BIT(18)
#define SEQ_CHANGE_ENABLE_DPB_COUNT BIT(19)

#define SEQ_CHANGE_ENABLE_ALL_VP9 (SEQ_CHANGE_ENABLE_PROFILE | \
		SEQ_CHANGE_ENABLE_SIZE | \
		SEQ_CHANGE_INTER_RES_CHANGE | \
		SEQ_CHANGE_ENABLE_BITDEPTH | \
		SEQ_CHANGE_ENABLE_DPB_COUNT)

#define SEQ_CHANGE_ENABLE_ALL_HEVC (SEQ_CHANGE_ENABLE_PROFILE | \
		SEQ_CHANGE_ENABLE_SIZE | \
		SEQ_CHANGE_ENABLE_BITDEPTH | \
		SEQ_CHANGE_ENABLE_DPB_COUNT)

#define SEQ_CHANGE_ENABLE_ALL_AVS2 (SEQ_CHANGE_ENABLE_PROFILE | \
		SEQ_CHANGE_ENABLE_SIZE | \
		SEQ_CHANGE_ENABLE_BITDEPTH | \
		SEQ_CHANGE_ENABLE_DPB_COUNT)

#define SEQ_CHANGE_ENABLE_ALL_AVC (SEQ_CHANGE_ENABLE_SIZE | \
		SEQ_CHANGE_ENABLE_BITDEPTH | \
		SEQ_CHANGE_ENABLE_DPB_COUNT)

#define SEQ_CHANGE_ENABLE_ALL_AV1 (SEQ_CHANGE_ENABLE_PROFILE | \
		SEQ_CHANGE_CHROMA_FORMAT_IDC | \
		SEQ_CHANGE_ENABLE_SIZE | \
		SEQ_CHANGE_ENABLE_BITDEPTH | \
		SEQ_CHANGE_ENABLE_DPB_COUNT)

#define DISPLAY_IDX_FLAG_SEQ_END -1
#define DISPLAY_IDX_FLAG_NO_FB -3
#define DECODED_IDX_FLAG_NO_FB -1
#define DECODED_IDX_FLAG_SKIP -2

#define RECON_IDX_FLAG_ENC_END -1
#define RECON_IDX_FLAG_ENC_DELAY -2
#define RECON_IDX_FLAG_HEADER_ONLY -3
#define RECON_IDX_FLAG_CHANGE_PARAM -4

enum codec_command {
	ENABLE_ROTATION,
	ENABLE_MIRRORING,
	SET_MIRROR_DIRECTION,
	SET_ROTATION_ANGLE,
	ENABLE_DEC_THUMBNAIL_MODE,
	DEC_GET_QUEUE_STATUS,
	ENC_GET_QUEUE_STATUS,
};

enum error_conceal_mode {
	ERROR_CONCEAL_MODE_OFF = 0, /* conceal off */
	ERROR_CONCEAL_MODE_INTRA_ONLY = 1, /* intra conceal in intra-picture, inter-picture */
	ERROR_CONCEAL_MODE_INTRA_INTER = 2
};

enum error_conceal_unit {
	ERROR_CONCEAL_UNIT_PICTURE = 0, /* picture-level error conceal */
	ERROR_CONCEAL_UNIT_SLICE_TILE = 1, /* slice/tile-level error conceal */
	ERROR_CONCEAL_UNIT_BLOCK_ROW = 2, /* block-row-level error conceal */
	ERROR_CONCEAL_UNIT_BLOCK = 3 /* block-level conceal */
};

enum cb_cr_order {
	CBCR_ORDER_NORMAL,
	CBCR_ORDER_REVERSED
};

enum mirror_direction {
	MIRDIR_NONE, /* no mirroring */
	MIRDIR_VER, /* vertical mirroring */
	MIRDIR_HOR, /* horizontal mirroring */
	MIRDIR_HOR_VER /* horizontal and vertical mirroring */
};

enum frame_buffer_format {
	FORMAT_ERR = -1,
	FORMAT_420 = 0, /* 8bit */
	FORMAT_422, /* 8bit */
	FORMAT_224, /* 8bit */
	FORMAT_444, /* 8bit */
	FORMAT_400, /* 8bit */

	/* little endian perspective */
	/* | addr 0 | addr 1 | */
	FORMAT_420_P10_16BIT_MSB = 5, /* lsb |000000xx|xxxxxxxx | msb */
	FORMAT_420_P10_16BIT_LSB, /* lsb |xxxxxxx |xx000000 | msb */
	FORMAT_420_P10_32BIT_MSB, /* lsb |00xxxxxxxxxxxxxxxxxxxxxxxxxxx| msb */
	FORMAT_420_P10_32BIT_LSB, /* lsb |xxxxxxxxxxxxxxxxxxxxxxxxxxx00| msb */

	/* 4:2:2 packed format */
	/* little endian perspective */
	/* | addr 0 | addr 1 | */
	FORMAT_422_P10_16BIT_MSB, /* lsb |000000xx |xxxxxxxx | msb */
	FORMAT_422_P10_16BIT_LSB, /* lsb |xxxxxxxx |xx000000 | msb */
	FORMAT_422_P10_32BIT_MSB, /* lsb |00xxxxxxxxxxxxxxxxxxxxxxxxxxx| msb */
	FORMAT_422_P10_32BIT_LSB, /* lsb |xxxxxxxxxxxxxxxxxxxxxxxxxxx00| msb */

	FORMAT_YUYV, /* 8bit packed format : Y0U0Y1V0 Y2U1Y3V1 ... */
	FORMAT_YUYV_P10_16BIT_MSB,
	FORMAT_YUYV_P10_16BIT_LSB,
	FORMAT_YUYV_P10_32BIT_MSB,
	FORMAT_YUYV_P10_32BIT_LSB,

	FORMAT_YVYU, /* 8bit packed format : Y0V0Y1U0 Y2V1Y3U1 ... */
	FORMAT_YVYU_P10_16BIT_MSB,
	FORMAT_YVYU_P10_16BIT_LSB,
	FORMAT_YVYU_P10_32BIT_MSB,
	FORMAT_YVYU_P10_32BIT_LSB,

	FORMAT_UYVY, /* 8bit packed format : U0Y0V0Y1 U1Y2V1Y3 ... */
	FORMAT_UYVY_P10_16BIT_MSB,
	FORMAT_UYVY_P10_16BIT_LSB,
	FORMAT_UYVY_P10_32BIT_MSB,
	FORMAT_UYVY_P10_32BIT_LSB,

	FORMAT_VYUY, /* 8bit packed format : V0Y0U0Y1 V1Y2U1Y3 ... */
	FORMAT_VYUY_P10_16BIT_MSB,
	FORMAT_VYUY_P10_16BIT_LSB,
	FORMAT_VYUY_P10_32BIT_MSB,
	FORMAT_VYUY_P10_32BIT_LSB,

	FORMAT_MAX,
};

enum packed_format_num {
	NOT_PACKED = 0,
	PACKED_YUYV,
	PACKED_YVYU,
	PACKED_UYVY,
	PACKED_VYUY,
};

enum wave5_interrupt_bit {
	INT_WAVE5_INIT_VPU = 0,
	INT_WAVE5_WAKEUP_VPU = 1,
	INT_WAVE5_SLEEP_VPU = 2,
	INT_WAVE5_CREATE_INSTANCE = 3,
	INT_WAVE5_FLUSH_INSTANCE = 4,
	INT_WAVE5_DESTROY_INSTANCE = 5,
	INT_WAVE5_INIT_SEQ = 6,
	INT_WAVE5_SET_FRAMEBUF = 7,
	INT_WAVE5_DEC_PIC = 8,
	INT_WAVE5_ENC_PIC = 8,
	INT_WAVE5_ENC_SET_PARAM = 9,
	INT_WAVE5_DEC_QUERY = 14,
	INT_WAVE5_BSBUF_EMPTY = 15,
	INT_WAVE5_BSBUF_FULL = 15,
};

enum pic_type {
	PIC_TYPE_I = 0, /* I picture */
	PIC_TYPE_KEY = 0, /* KEY frame for AV1*/
	PIC_TYPE_P = 1, /* P picture */
	PIC_TYPE_INTER = 1, /* inter frame for AV1*/
	PIC_TYPE_B = 2, /* B picture (except VC1) */
	PIC_TYPE_REPEAT = 2, /* repeat frame (VP9 only) */
	PIC_TYPE_AV1_INTRA = 2, /* intra only frame (AV1 only) */
	PIC_TYPE_VC1_BI = 2, /* VC1 BI picture (VC1 only) */
	PIC_TYPE_VC1_B = 3, /* VC1 B picture (VC1 only) */
	PIC_TYPE_D = 3,
	PIC_TYPE_S = 3,
	PIC_TYPE_AVS2_F = 3, /* F picture in AVS2 */
	PIC_TYPE_AV1_SWITCH = 3, /* switch frame (AV1 only) */
	PIC_TYPE_VC1_P_SKIP = 4, /* VC1 P skip picture (VC1 only) */
	PIC_TYPE_MP4_P_SKIP_NOT_CODED = 4, /* not coded P picture in MPEG4 packed mode */
	PIC_TYPE_AVS2_S = 4, /* S picture in AVS2 */
	PIC_TYPE_IDR = 5, /* H.264/H.265 IDR picture */
	PIC_TYPE_AVS2_G = 5, /* G picture in AVS2 */
	PIC_TYPE_AVS2_GB = 6, /* GB picture in AVS2 */
	PIC_TYPE_MAX /* no meaning */
};

enum bit_stream_mode {
	BS_MODE_INTERRUPT,
	BS_MODE_RESERVED, /* reserved for the future */
	BS_MODE_PIC_END,
};

enum sw_reset_mode {
	SW_RESET_SAFETY,
	SW_RESET_FORCE,
	SW_RESET_ON_BOOT
};

enum tiled_map_type {
	LINEAR_FRAME_MAP = 0, /* linear frame map type */
	COMPRESSED_FRAME_MAP = 17, /* compressed frame map type*/
};

enum temporal_id_mode {
	TEMPORAL_ID_MODE_ABSOLUTE,
	TEMPORAL_ID_MODE_RELATIVE,
};

struct vpu_attr {
	u32 product_id; /* the product ID */
	char product_name[8]; /* the product name in ascii code */
	u32 product_version; /* the product version number */
	u32 fw_version; /* the F/W version */
	u32 customer_id; /* customer ID number */
	u32 support_decoders; /* bitmask: see <<vpuapi_h_cod_std>> */
	u32 support_encoders; /* bitmask: see <<vpuapi_h_cod_std>> */
	bool support_bitstream_mode;
	bool support_backbone;
	bool support_avc10bit_enc;
	bool support_hevc10bit_enc;
	u32 support_endian_mask; /* A variable of supported endian mode in product */
	bool support_dual_core; /* this indicates whether a product has two vcores. */
	bool support_vcore_backbone;
	bool support_vcpu_backbone;
};

struct frame_buffer {
	dma_addr_t buf_y;
	dma_addr_t buf_cb;
	dma_addr_t buf_cr;
	u32 buf_y_size;
	u32 buf_cb_size;
	u32 buf_cr_size;
	int cbcr_interleave;
	int endian;
	enum tiled_map_type map_type;
	int stride; /* A horizontal stride for given frame buffer */
	int width; /* A width for given frame buffer */
	int height; /* A height for given frame buffer */
	int size; /* A size for given frame buffer */
	int sequence_no;
	bool update_fb_info;
};

struct vpu_rect {
	u32 left; /* A horizontal pixel offset of top-left corner of rectangle from (0, 0) */
	u32 top; /* A vertical pixel offset of top-left corner of rectangle from (0, 0) */
	u32 right; /* A horizontal pixel offset of bottom-right corner of rectangle from (0, 0) */
	u32 bottom; /* A vertical pixel offset of bottom-right corner of rectangle from (0, 0) */
};

/*
 * decode struct and definition
 */

struct dec_open_param {
	dma_addr_t bitstream_buffer;
	int bitstream_buffer_size;
	int cbcr_interleave;
	int nv21;
	int cbcr_order;
	enum endian_mode frame_endian;
	enum endian_mode stream_endian;
	enum bit_stream_mode bitstream_mode;
	bool enable_non_ref_fbc_write;
	int av1_format;
	enum error_conceal_unit error_conceal_unit;
	enum error_conceal_mode error_conceal_mode;
};

struct dec_initial_info {
	s32 pic_width;
	s32 pic_height;
	s32 f_rate_numerator; /* the numerator part of frame rate fraction */
	s32 f_rate_denominator; /* the denominator part of frame rate fraction */
	struct vpu_rect pic_crop_rect;
	u32 min_frame_buffer_count; /* between 1 to 16 */
	s32 frame_buf_delay;

	s32 max_temporal_layers; /* it indicates the max number of temporal sub-layers. */
	s32 profile;
	s32 level;
	u32 tier;
	bool is_ext_sar;
	u32 aspect_rate_info;
	s32 bit_rate;
	u32 user_data_header;
	s32 user_data_size;
	bool user_data_buf_full;
	s32 chroma_format_idc;/* A chroma format indicator */
	s32 luma_bitdepth; /* A bit-depth of luma sample */
	s32 chroma_bitdepth; /* A bit-depth of chroma sample */
	u32 seq_init_err_reason;
	s32 warn_info;
	dma_addr_t rd_ptr; /* A read pointer of bitstream buffer */
	dma_addr_t wr_ptr; /* A write pointer of bitstream buffer */
	u32 sequence_no;
	u32 output_bit_depth;
	u32 vlc_buf_size; /* the size of vlc buffer */
	u32 param_buf_size; /* the size of param buffer */
};

#define WAVE_SKIPMODE_WAVE_NONE 0
#define WAVE_SKIPMODE_NON_IRAP 1
#define WAVE_SKIPMODE_NON_REF 2

struct dec_param {
	s32 skipframe_mode;
	bool cra_as_bla_flag;
	bool disable_film_grain;
};

struct dec_output_ext_data {
	u32 user_data_header;
	u32 user_data_size; /* this is the size of user data. */
	bool user_data_buf_full;
	u32 active_format;
};

struct h265_rp_sei {
	unsigned int exist;
	int recovery_poc_cnt; /* recovery_poc_cnt */
	int exact_match_flag; /* exact_match_flag */
	int broken_link_flag; /* broken_link_flag */
};

struct avs2_info {
	int decoded_poi;
	int display_poi;
};

struct av1_info {
	int allow_screen_content_tools; /* it indicates whether screen content tool is enabled. */
	int allow_intra_bc; /* it indicates whether intra block copy is enabled. */
	int spatial_id; /* it indicates a spatial ID of the picture. */
};

struct dec_output_info {
	/**
	 * this is a frame buffer index for the picture to be displayed at the moment among
	 * frame buffers which are registered using vpu_dec_register_frame_buffer(). frame
	 * data to be displayed are stored into the frame buffer with this index.
	 * when there is no display delay, this index is always
	 * the same with index_frame_decoded. however, if display delay does exist for display
	 * reordering in AVC
	 * or B-frames in VC1), this index might be different with index_frame_decoded.
	 * by checking this index, HOST application can easily know whether sequence decoding
	 * has been finished or not.
	 *
	 * -3(0xFFFD) or -2(0xFFFE) : it is when a display output cannot be given due to picture
	 * reordering or skip option.
	 * -1(0xFFFF) : it is when there is no more output for display at the end of sequence
	 * decoding.
	 */
	int index_frame_display;
	int index_frame_display_for_tiled; /* in case of WTL mode, this index indicates a display
					    * index of tiled or compressed framebuffer.
					    */
	/**
	 * this is a frame buffer index of decoded picture among frame buffers which were
	 * registered using vpu_dec_register_frame_buffer(). the currently decoded frame is stored
	 * into the frame buffer specified by
	 * this index.
	 *
	 * -2 : it indicates that no decoded output is generated because decoder meets EOS
	 * (end of sequence) or skip.
	 * -1 : it indicates that decoder fails to decode a picture because there is no available
	 * frame buffer.
	 */
	int index_frame_decoded;
	int index_inter_frame_decoded;
	int index_frame_decoded_for_tiled;
	int nal_type;
	int pic_type;
	int num_of_err_m_bs;
	int num_of_tot_m_bs;
	int num_of_err_m_bs_in_disp;
	int num_of_tot_m_bs_in_disp;
	u32 decoding_success;
	struct vpu_rect rc_display;
	int disp_pic_width;
	int disp_pic_height;
	struct vpu_rect rc_decoded;
	int dec_pic_width;
	int dec_pic_height;
	struct avs2_info avs2_info;
	struct av1_info av1_info;
	int decoded_poc;
	int display_poc;
	int temporal_id; /* A temporal ID of the picture */
	struct h265_rp_sei h265_rp_sei;
	struct dec_output_ext_data dec_output_ext_data;
	int consumed_byte; /* the number of bytes that are consumed by VPU. */
	int rd_ptr; /* A stream buffer read pointer for the current decoder instance */
	int wr_ptr; /* A stream buffer write pointer for the current decoder instance */
	dma_addr_t byte_pos_frame_start;
	dma_addr_t byte_pos_frame_end;
	struct frame_buffer disp_frame;
	int frame_display_flag; /* it reports a frame buffer flag to be displayed. */
	/**
	 * this variable reports that sequence has been changed while H.264/AVC stream decoding.
	 * if it is 1, HOST application can get the new sequence information by calling
	 * vpu_dec_get_initial_info() or wave5_vpu_dec_issue_seq_init().
	 *
	 * for H.265/HEVC decoder, each bit has a different meaning as follows.
	 *
	 * sequence_changed[5] : it indicates that the profile_idc has been changed.
	 * sequence_changed[16] : it indicates that the resolution has been changed.
	 * sequence_changed[19] : it indicates that the required number of frame buffer has
	 * been changed.
	 */
	int sequence_changed;
	int frame_cycle; /* this variable reports the number of cycles for processing a frame. */
	int error_reason;
	u32 error_reason_ext;
	int warn_info;
	u32 sequence_no;

	u32 dec_host_cmd_tick; /* tick of DEC_PIC command for the picture */
	u32 dec_seek_start_tick; /* start tick of seeking slices of the picture */
	u32 dec_seek_end_tick; /* end tick of seeking slices of the picture */
	u32 dec_parse_start_tick; /* start tick of parsing slices of the picture */
	u32 dec_parse_end_tick; /* end tick of parsing slices of the picture */
	u32 dec_decode_start_tick; /* start tick of decoding slices of the picture */
	u32 dec_decode_end_tick; /* end tick of decoding slices of the picture */

	u32 seek_cycle;
	u32 parse_cycle;
	u32 decoded_cycle;

	s32 ctu_size;
	s32 output_flag;
};

struct queue_status_info {
	u32 instance_queue_count;
	u32 report_queue_count;
};

/*
 * encode struct and definition
 */

#define MAX_NUM_TEMPORAL_LAYER 7
#define MAX_NUM_SPATIAL_LAYER 3
#define MAX_GOP_NUM 8

struct custom_gop_pic_param {
	int pic_type; /* A picture type of nth picture in the custom GOP */
	int poc_offset; /* A POC of nth picture in the custom GOP */
	int pic_qp; /* A quantization parameter of nth picture in the custom GOP */
	int use_multi_ref_p; /* use multiref pic for P picture. valid only if PIC_TYPE is P */
	int ref_poc_l0; /* A POC of reference L0 of nth picture in the custom GOP */
	int ref_poc_l1; /* A POC of reference L1 of nth picture in the custom GOP */
	int temporal_id; /* A temporal ID of nth picture in the custom GOP */
};

struct custom_gop_param {
	int custom_gop_size; /* the size of custom GOP (0~8) */
	struct custom_gop_pic_param pic_param[MAX_GOP_NUM];
};

struct wave_custom_map_opt {
	int roi_avg_qp; /* it sets an average QP of ROI map. */
	int custom_roi_map_enable; /* it enables ROI map. */
	int custom_lambda_map_enable; /* it enables custom lambda map. */
	int custom_mode_map_enable;
	int custom_coef_drop_enable;
	dma_addr_t addr_custom_map;
};

struct enc_wave_param {
	/*
	 * A profile indicator (HEVC only)
	 *
	 * 0 : the firmware determines a profile according to internalbitdepth.
	 * 1 : main profile
	 * 2 : main10 profile
	 * 3 : main still picture profile
	 * in AVC encoder, a profile cannot be set by host application. the firmware decides it
	 * based on internalbitdepth. it is HIGH profile for bitdepth of 8 and HIGH10 profile for
	 * bitdepth of 10.
	 */
	int profile;
	int en_still_picture; /* still picture profile */
	int level; /* A level indicator (level * 10) */
	unsigned int tier; /* 0=main, 1=high */
	int internal_bit_depth;
	int gop_preset_idx;
	int decoding_refresh_type; /* 0=non-IRAP, 1=CRA, 2=IDR */
	int intra_qp; /* A quantization parameter of intra picture */
	int intra_period; /* A period of intra picture in GOP size */
	int forced_idr_header_enable;
	int conf_win_top; /* A top offset of conformance window */
	int conf_win_bot; /* A bottom offset of conformance window */
	int conf_win_left; /* A left offset of conformance window */
	int conf_win_right; /* A right offset of conformance window */
	int independ_slice_mode_arg;
	int depend_slice_mode_arg;
	int intra_refresh_mode;
	/*
	 * it specifies an intra CTU refresh interval. depending on intra_refresh_mode,
	 * it can mean one of the following.
	 *
	 * the number of consecutive CTU rows for intra_ctu_refresh_mode of 1
	 * the number of consecutive CTU columns for intra_ctu_refresh_mode of 2
	 * A step size in CTU for intra_ctu_refresh_mode of 3
	 * the number of intra ct_us to be encoded in a picture for intra_ctu_refresh_mode of 4
	 */
	int intra_refresh_arg;
	/*
	 * 0 : custom setting
	 * 1 : recommended encoder parameters (slow encoding speed, highest picture quality)
	 * 2 : boost mode (normal encoding speed, moderate picture quality)
	 * 3 : fast mode (fast encoding speed, low picture quality)
	 */
	uint32_t depend_slice_mode : 2;
	uint32_t independ_slice_mode : 1; /* 0=no-multi-slice, 1=slice-in-ctu-number*/
	uint32_t use_recommend_enc_param: 2;
	uint32_t max_num_merge: 2;
	uint32_t scaling_list_enable: 2;
	uint32_t bit_alloc_mode: 2; /* 0=ref-pic-priority, 1=uniform, 2=fixed_bit_ratio */
	int beta_offset_div2: 4; /* it sets beta_offset_div2 for deblocking filter. */
	int tc_offset_div2: 4; /* it sets tc_offset_div3 for deblocking filter. */
	int hvs_qp_scale: 4; /* QP scaling factor for CU QP adjust if hvs_qp_scale_enable is 1 */
	int hvs_max_delta_qp; /* A maximum delta QP for HVS */
	/*
	 * A fixed bit ratio (1 ~ 255) for each picture of GOP's bit
	 * allocation
	 *
	 * N = 0 ~ (MAX_GOP_SIZE - 1)
	 * MAX_GOP_SIZE = 8
	 *
	 * for instance when MAX_GOP_SIZE is 3, fixed_bit_ratio0, fixed_bit_ratio1, and
	 * fixed_bit_ratio2 can be set as 2, 1, and 1 respectively for
	 * the fixed bit ratio 2:1:1. this is only valid when bit_alloc_mode is 2.
	 */
	u8 fixed_bit_ratio[MAX_GOP_NUM];
	struct custom_gop_param gop_param; /* <<vpuapi_h_custom_gop_param>> */
	u32 num_units_in_tick;
	u32 time_scale;
	u32 num_ticks_poc_diff_one;
	int chroma_cb_qp_offset; /* the value of chroma(cb) QP offset */
	int chroma_cr_qp_offset; /* the value of chroma(cr) QP offset */
	int initial_rc_qp;
	u32 nr_intra_weight_y;
	u32 nr_intra_weight_cb; /* A weight to cb noise level for intra picture (0 ~ 31) */
	u32 nr_intra_weight_cr; /* A weight to cr noise level for intra picture (0 ~ 31) */
	u32 nr_inter_weight_y;
	u32 nr_inter_weight_cb; /* A weight to cb noise level for inter picture (0 ~ 31) */
	u32 nr_inter_weight_cr; /* A weight to cr noise level for inter picture (0 ~ 31) */
	u32 nr_noise_sigma_y; /* Y noise standard deviation if nr_noise_est_enable is 0. */
	u32 nr_noise_sigma_cb;/* cb noise standard deviation if nr_noise_est_enable is 0. */
	u32 nr_noise_sigma_cr;/* cr noise standard deviation if nr_noise_est_enable is 0. */
	u32 bg_thr_diff;
	u32 bg_thr_mean_diff;
	u32 bg_lambda_qp;
	int bg_delta_qp;
	int pu04_delta_rate: 8; /* added to the total cost of 4x4 blocks */
	int pu08_delta_rate: 8; /* added to the total cost of 8x8 blocks */
	int pu16_delta_rate: 8; /* added to the total cost of 16x16 blocks */
	int pu32_delta_rate: 8; /* added to the total cost of 32x32 blocks */
	int pu04_intra_planar_delta_rate: 8;
	int pu04_intra_dc_delta_rate: 8;
	int pu04_intra_angle_delta_rate: 8;
	int pu08_intra_planar_delta_rate: 8;
	int pu08_intra_dc_delta_rate: 8;
	int pu08_intra_angle_delta_rate: 8;
	int pu16_intra_planar_delta_rate: 8;
	int pu16_intra_dc_delta_rate: 8;
	int pu16_intra_angle_delta_rate: 8;
	int pu32_intra_planar_delta_rate: 8;
	int pu32_intra_dc_delta_rate: 8;
	int pu32_intra_angle_delta_rate: 8;
	int cu08_intra_delta_rate: 8;
	int cu08_inter_delta_rate: 8;
	int cu08_merge_delta_rate: 8;
	int cu16_intra_delta_rate: 8;
	int cu16_inter_delta_rate: 8;
	int cu16_merge_delta_rate: 8;
	int cu32_intra_delta_rate: 8;
	int cu32_inter_delta_rate: 8;
	int cu32_merge_delta_rate: 8;
	int coef_clear_disable: 8;
	int min_qp_i; /* A minimum QP of I picture for rate control */
	int max_qp_i; /* A maximum QP of I picture for rate control */
	int min_qp_p; /* A minimum QP of P picture for rate control */
	int max_qp_p; /* A maximum QP of P picture for rate control */
	int min_qp_b; /* A minimum QP of B picture for rate control */
	int max_qp_b; /* A maximum QP of B picture for rate control */
	dma_addr_t custom_lambda_addr; /* it specifies the address of custom lambda map. */
	dma_addr_t user_scaling_list_addr; /* it specifies the address of user scaling list file. */
	int avc_idr_period;/* A period of IDR picture (0 ~ 1024). 0 - implies an infinite period */
	int avc_slice_mode; /* 0=none, 1=slice-in-mb-number */
	int avc_slice_arg;	/* the number of MB for a slice when avc_slice_mode is set with 1 */
	int intra_mb_refresh_mode; /* 0=none, 1=row, 2=column, 3=step-size-in-mb */
	/**
	 * it specifies an intra MB refresh interval. depending on intra_mb_refresh_mode,
	 * it can mean one of the following.
	 *
	 * the number of consecutive MB rows for intra_mb_refresh_mode of 1
	 * the number of consecutive MB columns for intra_mb_refresh_mode of 2
	 * A step size in MB for intra_mb_refresh_mode of 3
	 */
	int intra_mb_refresh_arg;
	int entropy_coding_mode; /* 0=CAVLC, 1=CABAC */
	u32 rc_weight_param;
	u32 rc_weight_buf;

	/* flags */
	uint32_t lossless_enable: 1; /*enables lossless coding. */
	uint32_t const_intra_pred_flag: 1; /*enables constrained intra prediction. */
	uint32_t tmvp_enable: 1; /*enables temporal motion vector prediction. */
	uint32_t wpp_enable: 1;
	uint32_t disable_deblk: 1; /* it disables in-loop deblocking filtering. */
	uint32_t lf_cross_slice_boundary_enable: 1;
	uint32_t skip_intra_trans: 1;
	uint32_t sao_enable: 1; /* it enables SAO (sample adaptive offset). */
	uint32_t intra_nx_n_enable: 1; /* it enables intra nx_n p_us. */
	uint32_t cu_level_rc_enable: 1; /* it enable CU level rate control. */
	uint32_t hvs_qp_enable: 1; /* enable CU QP adjustment for subjective quality enhancement. */
	uint32_t roi_enable: 1; /* it enables ROI map. NOTE: it is valid when rate control is on. */
	uint32_t nr_y_enable: 1; /* it enables noise reduction algorithm to Y component. */
	uint32_t nr_noise_est_enable: 1;
	uint32_t nr_cb_enable: 1; /* it enables noise reduction algorithm to cb component. */
	uint32_t nr_cr_enable: 1; /* it enables noise reduction algorithm to cr component. */
	uint32_t use_long_term: 1; /* it enables long-term reference function. */
	uint32_t monochrome_enable: 1; /* it enables monochrom encoding mode. */
	uint32_t strong_intra_smooth_enable: 1; /* it enables strong intra smoothing. */
	uint32_t weight_pred_enable: 1; /* it enables to use weighted prediction.*/
	uint32_t bg_detect_enable: 1; /* it enables background detection. */
	uint32_t custom_lambda_enable: 1; /* it enables custom lambda table. */
	uint32_t custom_md_enable: 1; /* it enables custom mode decision. */
	uint32_t rdo_skip: 1;	/* it skips RDO(rate distortion optimization). */
	uint32_t lambda_scaling_enable: 1;	/* it enables lambda scaling using custom GOP. */
	uint32_t transform8x8_enable: 1; /* it enables 8x8 intra prediction and 8x8 transform. */
	uint32_t mb_level_rc_enable: 1;			/* it enables MB-level rate control. */
	uint32_t s2fme_disable: 1; /* it disables s2me_fme (only for AVC encoder). */
};

struct enc_sub_frame_sync_config {
	u32 sub_frame_sync_mode; /* 0=wire-based, 1=register-based */
	u32 sub_frame_sync_on;
};

struct enc_open_param {
	dma_addr_t bitstream_buffer;
	u32 bitstream_buffer_size;
	int ring_buffer_enable;
	int pic_width; /* the width of a picture to be encoded in unit of sample. */
	int pic_height; /* the height of a picture to be encoded in unit of sample. */
	int frame_rate_info;/* desired fps */
	int vbv_buffer_size;
	int bit_rate; /* target bitrate in bps */
	u32 rc_enable : 1; /* rate control */
	struct enc_wave_param wave_param;
	int cbcr_interleave;
	int cbcr_order;
	int stream_endian;
	int source_endian;
	int line_buf_int_en;
	int packed_format; /* <<vpuapi_h_packed_format_num>> */
	enum frame_buffer_format src_format;
	enum frame_buffer_format output_format;
	int nv21;
	bool enable_pts; /* an enable flag to report PTS(presentation timestamp) */
	bool enable_non_ref_fbc_write;
	int sub_frame_sync_enable;
	int sub_frame_sync_mode;
	u32 enc_hrd_rbsp_in_vps; /* it encodes the HRD syntax rbsp into VPS. */
	u32 hrd_rbsp_data_size; /* the bit size of the HRD rbsp data */
	dma_addr_t hrd_rbsp_data_addr; /* the address of the HRD rbsp data */
	u32 encode_vui_rbsp;
	u32 vui_rbsp_data_size; /* the bit size of the VUI rbsp data */
	dma_addr_t vui_rbsp_data_addr; /* the address of the VUI rbsp data */
};

struct enc_initial_info {
	u32 min_frame_buffer_count; /* minimum number of frame buffer */
	int min_src_frame_count; /* minimum number of source buffer */
	int max_latency_pictures; /* maximum number of picture latency */
	int seq_init_err_reason; /* error information */
	int warn_info; /* warn information */
	u32 vlc_buf_size; /* the size of task buffer */
	u32 param_buf_size; /* the size of task buffer */
};

struct enc_code_opt {
	int implicit_header_encode;
	int encode_vcl; /* A flag to encode VCL nal unit explicitly */
	int encode_vps; /* A flag to encode VPS nal unit explicitly */
	int encode_sps; /* A flag to encode SPS nal unit explicitly */
	int encode_pps; /* A flag to encode PPS nal unit explicitly */
	int encode_aud; /* A flag to encode AUD nal unit explicitly */
	int encode_eos;
	int encode_eob;
	int encode_vui; /* A flag to encode VUI nal unit explicitly */
};

struct enc_param {
	struct frame_buffer *source_frame;
	unsigned int skip_picture : 1;
	dma_addr_t pic_stream_buffer_addr;
	int pic_stream_buffer_size;
	int force_pic_qp_enable; /* flag used to force picture quantization parameter */
	int force_pic_qp_i;
	int force_pic_qp_p;
	int force_pic_qp_b;
	int force_pic_type_enable; /* A flag to use a force picture type */
	int force_pic_type;
	int src_idx; /* A source frame buffer index */
	int src_end_flag;
	struct enc_code_opt code_option;
	u32 use_cur_src_as_longterm_pic;
	u32 use_longterm_ref;
	u64 pts; /* the presentation timestamp (PTS) of input source */
	struct wave_custom_map_opt custom_map_opt;
	u32 wp_pix_sigma_y; /* pixel variance of Y component for weighted prediction */
	u32 wp_pix_sigma_cb; /* pixel variance of cb component for weighted prediction */
	u32 wp_pix_sigma_cr; /* pixel variance of cr component for weighted prediction */
	u32 wp_pix_mean_y; /* pixel mean value of Y component for weighted prediction */
	u32 wp_pix_mean_cb; /* pixel mean value of cb component for weighted prediction */
	u32 wp_pix_mean_cr; /* pixel mean value of cr component for weighted prediction */
	u32 force_all_ctu_coef_drop_enable; /* it forces all coefficients to be zero after TQ . */
};

struct enc_output_info {
	dma_addr_t bitstream_buffer;
	u32 bitstream_size; /* the byte size of encoded bitstream */
	int bitstream_wrap_around;
	int pic_type; /* <<vpuapi_h_pic_type>> */
	int num_of_slices; /* the number of slices of the currently being encoded picture */
	int recon_frame_index;
	struct frame_buffer recon_frame;
	int rd_ptr;
	int wr_ptr;
	int pic_skipped; /* whether the current encoding has been skipped or not */
	int num_of_intra; /* the number of intra coded block */
	int num_of_merge; /* the number of merge block in 8x8 */
	int num_of_skip_block; /* the number of skip block in 8x8 */
	int avg_ctu_qp; /* the average value of CTU q_ps */
	int enc_pic_byte; /* the number of encoded picture bytes */
	int enc_gop_pic_idx; /* the GOP index of the currently encoded picture */
	int enc_pic_poc; /* the POC(picture order count) of the currently encoded picture */
	int release_src_flag; /* the source buffer indicator of the encoded pictures */
	int enc_src_idx; /* the source buffer index of the currently encoded picture */
	int enc_vcl_nut;
	int enc_pic_cnt; /* the encoded picture number */
	int error_reason; /* the error reason of the currently encoded picture */
	int warn_info; /* the warning information of the currently encoded picture */
	int frame_cycle; /* the param for reporting the cycle number of encoding one frame.*/
	u64 pts;
	u32 enc_host_cmd_tick; /* tick of ENC_PIC command for the picture */
	u32 enc_prepare_start_tick; /* start tick of preparing slices of the picture */
	u32 enc_prepare_end_tick; /* end tick of preparing slices of the picture */
	u32 enc_processing_start_tick; /* start tick of processing slices of the picture */
	u32 enc_processing_end_tick; /* end tick of processing slices of the picture */
	u32 enc_encode_start_tick; /* start tick of encoding slices of the picture */
	u32 enc_encode_end_tick; /* end tick of encoding slices of the picture */
	u32 prepare_cycle; /* the number of cycles for preparing slices of a picture */
	u32 processing; /* the number of cycles for processing slices of a picture */
	u32 encoded_cycle; /* the number of cycles for encoding slices of a picture */
	u32 pic_distortion_low;
	u32 pic_distortion_high;
};

enum ENC_PIC_CODE_OPTION {
	CODEOPT_ENC_HEADER_IMPLICIT = BIT(0),
	CODEOPT_ENC_VCL = BIT(1), /* A flag to encode VCL nal unit explicitly */
};

enum GOP_PRESET_IDX {
	PRESET_IDX_CUSTOM_GOP = 0, /* user defined GOP structure */
	PRESET_IDX_ALL_I = 1, /* all intra, gopsize = 1 */
	PRESET_IDX_IPP = 2, /* consecutive P, cyclic gopsize = 1 */
	PRESET_IDX_IBBB = 3, /* consecutive B, cyclic gopsize = 1 */
	PRESET_IDX_IBPBP = 4, /* gopsize = 2 */
	PRESET_IDX_IBBBP = 5, /* gopsize = 4 */
	PRESET_IDX_IPPPP = 6, /* consecutive P, cyclic gopsize = 4 */
	PRESET_IDX_IBBBB = 7, /* consecutive B, cyclic gopsize = 4 */
	PRESET_IDX_RA_IB = 8, /* random access, cyclic gopsize = 8 */
	PRESET_IDX_IPP_SINGLE = 9, /* consecutive P, cyclic gopsize = 1, with single ref */
};

struct sec_axi_info {
	struct {
		int use_ip_enable;
		int use_lf_row_enable;
		int use_bit_enable;
		int use_enc_rdo_enable;
		int use_enc_lf_enable;
	} wave;
	int buf_size;
	dma_addr_t buf_base;
};

struct dec_info {
	struct dec_open_param open_param;
	struct dec_initial_info initial_info;
	struct dec_initial_info new_seq_info; /* temporal new sequence information */
	dma_addr_t stream_wr_ptr;
	dma_addr_t stream_rd_ptr;
	int stream_endflag;
	int frame_display_flag;
	dma_addr_t stream_buf_start_addr;
	dma_addr_t stream_buf_end_addr;
	int stream_buf_size;
	struct vpu_buf vb_mv[MAX_REG_FRAME];
	struct vpu_buf vb_fbc_y_tbl[MAX_REG_FRAME];
	struct vpu_buf vb_fbc_c_tbl[MAX_REG_FRAME];
	int num_fbs_for_decoding;
	int num_fbs_for_wtl;
	int stride;
	bool rotation_enable;
	bool mirror_enable;
	int dering_enable;
	enum mirror_direction mirror_direction;
	int rotation_angle;
	struct frame_buffer rotator_output;
	int rotator_stride;
	bool initial_info_obtained;
	struct sec_axi_info sec_axi_info;
	int seq_init_escape;
	dma_addr_t user_data_buf_addr;
	u32 user_data_enable;
	int user_data_buf_size;
	int frame_start_pos;
	int frame_end_pos;
	struct vpu_buf vb_work;
	struct vpu_buf vb_task;
	struct dec_output_info dec_out_info[WAVE5_MAX_FBS];
	int reorder_enable;
	int thumbnail_mode;
	int seq_change_mask;
	enum temporal_id_mode temp_id_select_mode;
	u32 target_temp_id;
	u32 target_spatial_id;
	u32 instance_queue_count;
	u32 report_queue_count;
	bool first_cycle_check;
	u32 cycle_per_tick;
	u32 product_code;
	u32 vlc_buf_size;
	u32 param_buf_size;
};

struct enc_info {
	struct enc_open_param open_param;
	struct enc_initial_info initial_info;
	dma_addr_t stream_rd_ptr;
	dma_addr_t stream_wr_ptr;
	dma_addr_t stream_buf_start_addr;
	dma_addr_t stream_buf_end_addr;
	int stream_buf_size;
	int num_frame_buffers;
	int stride;
	bool rotation_enable;
	bool mirror_enable;
	enum mirror_direction mirror_direction;
	int rotation_angle;
	bool initial_info_obtained;
	int ring_buffer_enable;
	struct sec_axi_info sec_axi_info;
	struct enc_sub_frame_sync_config sub_frame_sync_config;
	int line_buf_int_en;
	struct vpu_buf vb_work;
	struct vpu_buf vb_mv; /* col_mv buffer */
	struct vpu_buf vb_fbc_y_tbl; /* FBC luma table buffer */
	struct vpu_buf vb_fbc_c_tbl; /* FBC chroma table buffer */
	struct vpu_buf vb_sub_sam_buf; /* sub-sampled buffer for ME */
	struct vpu_buf vb_task;
	u64 cur_pts; /* current timestamp in 90_k_hz */
	u64 pts_map[32]; /* PTS mapped with source frame index */
	u32 instance_queue_count;
	u32 report_queue_count;
	bool first_cycle_check;
	u32 cycle_per_tick;
	u32 product_code;
	u32 vlc_buf_size;
	u32 param_buf_size;
};

struct vpu_device {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *v4l2_m2m_dev;
	struct video_device *video_dev_dec;
	struct video_device *video_dev_enc;
	struct mutex dev_lock; /* the lock for the src,dst v4l2 queues */
	struct mutex	 hw_lock; /* lock hw configurations */
	struct kfifo irq_status;
	int irq;
	struct completion irq_done;
	enum product_id	 product;
	struct vpu_attr	 attr;
	struct vpu_buf common_mem;
	u32 last_performance_cycles;
	struct dma_vpu_buf sram_buf;
	void __iomem *vdb_register;
	int product_code;
	struct ida inst_ida;
	struct clk_bulk_data *clks;
	int num_clks;
};

struct vpu_instance;

struct vpu_instance_ops {
	void (*start_process)(struct vpu_instance *inst);
	void (*stop_process)(struct vpu_instance *inst);
	void (*finish_process)(struct vpu_instance *inst);
};

struct vpu_instance {
	struct v4l2_fh v4l2_fh;
	struct v4l2_ctrl_handler v4l2_ctrl_hdl;
	struct vpu_device *dev;

	struct v4l2_pix_format_mplane src_fmt;
	struct v4l2_pix_format_mplane dst_fmt;
	enum v4l2_colorspace colorspace;
	enum v4l2_xfer_func xfer_func;
	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_quantization quantization;
	enum v4l2_hsv_encoding hsv_enc;

	enum vpu_instance_state state;
	enum vpu_instance_type type;
	const struct vpu_instance_ops *ops;

	enum wave_std		 std;
	int			 id;
	union {
		struct enc_info enc_info;
		struct dec_info dec_info;
	} *codec_info;
	struct frame_buffer frame_buf[MAX_REG_FRAME];
	struct vpu_buf frame_vbuf[MAX_REG_FRAME];
	u32 min_dst_frame_buf_count;
	u32 queued_src_buf_num;
	u32 queued_dst_buf_num;
	u64 timestamp;

	spinlock_t bitstream_lock; /* lock the src buf queue of the m2m ctx */
	struct vpu_buf bitstream_vbuf;
	bool thumbnail_mode;

	unsigned int min_src_frame_buf_count;
	unsigned int rot_angle;
	unsigned int mirror_direction;
	unsigned int profile;
	unsigned int level;
	unsigned int bit_depth;
	unsigned int frame_rate;
	unsigned int vbv_buf_size;
	unsigned int min_qp_i;
	unsigned int max_qp_i;
	unsigned int min_qp_p;
	unsigned int max_qp_p;
	unsigned int min_qp_b;
	unsigned int max_qp_b;
};

void wave5_vdi_write_register(struct vpu_device *vpu_device, unsigned int addr, unsigned int data);
unsigned int wave5_vdi_readl(struct vpu_device *vpu_dev, unsigned int addr);
int wave5_vdi_clear_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb);
int wave5_vdi_allocate_dma_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb);
int wave5_vdi_write_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb, size_t offset,
			   u8 *data, int len, int endian);
int wave5_vdi_convert_endian(struct vpu_device *vpu_dev, unsigned int endian);
void wave5_vdi_free_dma_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb);

int wave5_vpu_init_with_bitcode(struct device *dev, u8 *bitcode, uint32_t size);
void wave5_vpu_clear_interrupt_ex(struct vpu_instance *inst, int32_t intr_flag);
int wave5_vpu_get_version_info(struct device *dev, uint32_t *version_info, uint32_t *revision,
			       uint32_t *product_id);
int wave5_vpu_dec_open(struct vpu_instance *vpu_inst, struct dec_open_param *pop);
int wave5_vpu_dec_close(struct vpu_instance *inst, u32 *fail_res);
int wave5_vpu_dec_issue_seq_init(struct vpu_instance *inst);
int wave5_vpu_dec_complete_seq_init(struct vpu_instance *inst, struct dec_initial_info *info);
int wave5_vpu_dec_register_frame_buffer_ex(struct vpu_instance *inst, int num_of_dec_fbs,
					   int num_of_display_fbs, int stride, int height,
					   int map_type);
int wave5_vpu_dec_start_one_frame(struct vpu_instance *inst, struct dec_param *param,
				  u32 *res_fail);
int wave5_vpu_dec_get_output_info(struct vpu_instance *inst, struct dec_output_info *info);
int wave5_vpu_dec_give_command(struct vpu_instance *inst, enum codec_command cmd, void *parameter);
int wave5_vpu_dec_get_bitstream_buffer(struct vpu_instance *inst, dma_addr_t *prd_prt,
				       dma_addr_t *pwr_ptr, uint32_t *size);
int wave5_vpu_dec_update_bitstream_buffer(struct vpu_instance *inst, int size);
int wave5_vpu_dec_clr_disp_flag(struct vpu_instance *inst, int index);

int wave5_vpu_enc_open(struct vpu_instance *vpu_inst, struct enc_open_param *enc_op_param);
int wave5_vpu_enc_close(struct vpu_instance *inst, u32 *fail_res);
int wave5_vpu_enc_issue_seq_init(struct vpu_instance *inst);
int wave5_vpu_enc_complete_seq_init(struct vpu_instance *inst, struct enc_initial_info *info);
int wave5_vpu_enc_register_frame_buffer(struct vpu_instance *inst, int num, unsigned int stride,
					int height, enum tiled_map_type map_type);
int wave5_vpu_enc_start_one_frame(struct vpu_instance *inst, struct enc_param *param,
				  u32 *fail_res);
int wave5_vpu_enc_get_output_info(struct vpu_instance *inst, struct enc_output_info *info);
int wave5_vpu_enc_give_command(struct vpu_instance *inst, enum codec_command cmd, void *parameter);

#endif

