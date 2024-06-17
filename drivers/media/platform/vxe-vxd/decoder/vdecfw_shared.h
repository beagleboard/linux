/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Public data structures and enums for the firmware
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

#ifdef USE_SHARING
#endif

#ifndef _VDECFW_H_
#define _VDECFW_H_

#include "img_msvdx_core_regs.h"
#include "vdecfw_share.h"

/* brief This type defines the buffer type */
enum img_buffer_type {
	IMG_BUFFERTYPE_FRAME       = 0,
	IMG_BUFFERTYPE_FIELD_TOP,
	IMG_BUFFERTYPE_FIELD_BOTTOM,
	IMG_BUFFERTYPE_PAIR,
	IMG_BUFFERTYPE_FORCE32BITS = 0x7FFFFFFFU
};

/* Number of scaling coefficients */
#define VDECFW_NUM_SCALE_COEFFS 4

/*
 * maximum number of pictures handled by the firmware
 * for H.264 (largest requirement): 32 for 4 view MVC
 */
#define VDECFW_MAX_NUM_PICTURES 32
#define VDECFW_MAX_NUM_VIEWS 4
#define EMERALD_CORE    6

/*
 * maximum number of colocated pictures handled by
 * firmware in FWBSP mode
 */
#define VDECFWBSP_MAX_NUM_COL_PICS 16

/* Maximum number of colour planes. */
#define VDECFW_PLANE_MAX 4

#define VDECFW_NON_EXISTING_PICTURE_TID (0xffffffff)

#define NO_VALUE    0

/* Indicates whether a cyclic sequence number (x) has reached another (y). */
#define HAS_X_REACHED_Y(x, y, range, type) \
	({ \
		type __x = x; \
		type __y = y; \
		type __range = range; \
		(((((__x) - (__y) + (__range)) % (__range)) <= \
		  (((__y) - (__x) + (__range)) % (__range))) ? TRUE : FALSE); })

/* Indicates whether a cyclic sequence number (x) has passed another (y). */
#define HAS_X_PASSED_Y(x, y, range, type) \
	({ \
		type __x = x; \
		type __y = y; \
		type __range = range; \
		(((((__x) - (__y) + (__range)) % (__range)) < \
		  (((__y) - (__x) + (__range)) % (__range))) ? TRUE : FALSE); })

#define FWIF_BIT_MASK(num)                      ((1 << (num)) - 1)

/*
 * Number of bits in transaction ID used to represent picture number in stream.
 */
#define FWIF_NUMBITS_STREAM_PICTURE_ID          16
/* Number of bits in transaction ID used to represent picture number in core. */
#define FWIF_NUMBITS_CORE_PICTURE_ID            4
/* Number of bits in transaction ID used to represent stream id. */
#define FWIF_NUMBITS_STREAM_ID                  8
/* Number of bits in transaction ID used to represent core id. */
#define FWIF_NUMBITS_CORE_ID                    4

/* Offset in transaction ID to picture number in stream. */
#define FWIF_OFFSET_STREAM_PICTURE_ID           0
/* Offset in transaction ID to picture number in core. */
#define FWIF_OFFSET_CORE_PICTURE_ID                             \
	(FWIF_OFFSET_STREAM_PICTURE_ID + FWIF_NUMBITS_STREAM_PICTURE_ID)
/* Offset in transaction ID to stream id. */
#define FWIF_OFFSET_STREAM_ID                                   \
	(FWIF_OFFSET_CORE_PICTURE_ID + FWIF_NUMBITS_CORE_PICTURE_ID)
/* Offset in transaction ID to core id. */
#define FWIF_OFFSET_CORE_ID                                     \
	(FWIF_OFFSET_STREAM_ID + FWIF_NUMBITS_STREAM_ID)

/* Picture id (stream) from transaction id. */
#define GET_STREAM_PICTURE_ID(transaction_id)                   \
	((transaction_id) & FWIF_BIT_MASK(FWIF_NUMBITS_STREAM_PICTURE_ID))
/* Picture id (core) from transaction id. */
#define GET_CORE_PICTURE_ID(transaction_id)                     \
	(((transaction_id) >> FWIF_OFFSET_CORE_PICTURE_ID) &    \
	 FWIF_BIT_MASK(FWIF_NUMBITS_CORE_PICTURE_ID))
/* Stream id from transaction id. */
#define GET_STREAM_ID(transaction_id)                           \
	(((transaction_id) >> FWIF_OFFSET_STREAM_ID) &          \
	 FWIF_BIT_MASK(FWIF_NUMBITS_STREAM_ID))
/* Core id from transaction id. */
#define GET_CORE_ID(transaction_id)                             \
	(((transaction_id) >> FWIF_OFFSET_CORE_ID) &            \
	 FWIF_BIT_MASK(FWIF_NUMBITS_CORE_ID))

/* Picture id (stream) for transaction id. */
#define SET_STREAM_PICTURE_ID(str_pic_id)                       \
	(((str_pic_id) & FWIF_BIT_MASK(FWIF_NUMBITS_STREAM_PICTURE_ID)) << \
		FWIF_OFFSET_STREAM_PICTURE_ID)
/* Picture id (core) for transaction id. */
#define SET_CORE_PICTURE_ID(core_pic_id)                                \
	(((core_pic_id) % (1 << FWIF_NUMBITS_CORE_PICTURE_ID)) <<       \
		FWIF_OFFSET_CORE_PICTURE_ID)
/* Stream id for transaction id. */
#define SET_STREAM_ID(stream_id)                                \
	(((stream_id) & FWIF_BIT_MASK(FWIF_NUMBITS_STREAM_ID)) <<       \
		FWIF_OFFSET_STREAM_ID)
/* Core id for transaction id. */
#define SET_CORE_ID(core_id)                                    \
	(((core_id) & FWIF_BIT_MASK(FWIF_NUMBITS_CORE_ID)) <<   \
		FWIF_OFFSET_CORE_ID)
/* flag checking */
#define FLAG_MASK(_flagname_)                   ((1 << _flagname_ ## _SHIFT))
#define FLAG_IS_SET(_flagsword_, _flagname_)                    \
	(((_flagsword_) & FLAG_MASK(_flagname_)) ? TRUE : FALSE)

/* This type defines the parser component types */
enum vdecfw_codectype {
	VDECFW_CODEC_H264 = 0,       /* H.264, AVC, MVC */
	VDECFW_CODEC_MPEG4,          /* MPEG4, H.263, DivX, Sorenson */
	VDECFW_CODEC_VP8,            /* VP8 */

	VDECFW_CODEC_VC1,            /* VC1 (includes WMV9) */
	VDECFW_CODEC_MPEG2,          /* MPEG2 */

	VDECFW_CODEC_JPEG,           /* JPEG */

	VDECFW_CODEC_VP6,            /* VP6 */
	VDECFW_CODEC_AVS,            /* AVS */
	VDECFW_CODEC_RV,             /* RV30, RV40 */

	VDECFW_CODEC_HEVC,           /* HEVC/H265 */

	VDECFW_CODEC_VP9,            /* VP9 */

	VDECFW_CODEC_MAX,            /* End Marker */

	VDEC_CODEC_NONE        = -1, /* No codec */
	VDEC_CODEC_FORCE32BITS = 0x7FFFFFFFU
};

/* This type defines the FW parser mode - SCP, size delimited, etc. */
enum vdecfw_parsermode {
	/* Every NAL is expected to have SCP */
	VDECFW_SCP_ONLY = 0,
	/* Every NAL is expect to be size delimited with field size 4 */
	VDECFW_SIZE_DELIMITED_4_ONLY,
	/* Every NAL is expect to be size delimited with field size 2 */
	VDECFW_SIZE_DELIMITED_2_ONLY,
	/* Every NAL is expect to be size delimited with field size 1 */
	VDECFW_SIZE_DELIMITED_1_ONLY,
	/* Size of NAL is provided in the picture header */
	VDECFW_SIZE_SIDEBAND,
	/* Unit is a skipped picture with no data to process */
	VDECFW_SKIPPED_PICTURE,
	VDECFW_SKIPPED_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This enum defines values of ENTDEC_BE_MODE field of VEC_ENTDEC_BE_CONTROL
 * register and ENTDEC_FE_MODE field of VEC_ENTDEC_FE_CONTROL register.
 */
enum vdecfw_msvdxentdecmode {
	/* JPEG */
	VDECFW_ENTDEC_MODE_JPEG        = 0x0,
	/* H264 (MPEG4/AVC) */
	VDECFW_ENTDEC_MODE_H264        = 0x1,
	/* VC1 */
	VDECFW_ENTDEC_MODE_VC1         = 0x2,
	/* MPEG2 */
	VDECFW_ENTDEC_MODE_MPEG2       = 0x3,
	/* MPEG4 */
	VDECFW_ENTDEC_MODE_MPEG4       = 0x4,
	/* AVS */
	VDECFW_ENTDEC_MODE_AVS         = 0x5,
	/* WMV9 */
	VDECFW_ENTDEC_MODE_WMV9        = 0x6,
	/* MPEG1 */
	VDECFW_ENTDEC_MODE_MPEG1       = 0x7,
	/* RealVideo8, with ENTDEC_[BE|FE]_EXTENDED_MODE bit set */
	VDECFW_ENTDEC_MODE_EXT_REAL8   = 0x0,
	/* RealVideo9, with ENTDEC_[BE|FE]_EXTENDED_MODE bit set */
	VDECFW_ENTDEC_MODE_EXT_REAL9   = 0x1,
	/* VP6, with ENTDEC_[BE|FE]_EXTENDED_MODE bit set */
	VDECFW_ENTDEC_MODE_EXT_VP6     = 0x2,
	/* VP8, with ENTDEC_[BE|FE]_EXTENDED_MODE bit set */
	VDECFW_ENTDEC_MODE_EXT_VP8     = 0x3,
	/* SVC, with ENTDEC_[BE|FE]_EXTENDED_MODE bit set */
	VDECFW_ENTDEC_MODE_EXT_SVC     = 0x4,
	VDECFW_ENTDEC_MODE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This describes the Firmware Parser checkpoints in VEC Local RAM.
 * Each checkpoint is updated with the TransactionID of the picture as it passes
 * that point in its decode. Together they describe the current position of
 * pictures in the VXD/Firmware pipeline.
 *
 * Numbers indicate point in the "VDEC Firmware Component Timing" diagram.
 */
enum vdecfw_progresscheckpoint {
	/* Decode message has been read */
	VDECFW_CHECKPOINT_PICTURE_STARTED     = 1,
	/* Firmware has been loaded and bitstream DMA started */
	VDECFW_CHECKPOINT_FIRMWARE_READY      = 2,
	/* Picture management operations have completed */
	VDECFW_CHECKPOINT_PICMAN_COMPLETE     = 3,
	/* Firmware context for this picture has been saved */
	VDECFW_CHECKPOINT_FIRMWARE_SAVED      = 4,
	/*
	 * 1st Picture/Slice header has been read,
	 * registers written and Entdec started
	 */
	VDECFW_CHECKPOINT_ENTDEC_STARTED      = 5,
	/* 1st Slice has been completed by Entdec */
	VDECFW_CHECKPOINT_FE_1SLICE_DONE      = 6,
	/* Parsing of picture has completed on FE */
	VDECFW_CHECKPOINT_FE_PARSE_DONE       = 7,
	/* Picture end code has been read and picture closed */
	VDECFW_CHECKPOINT_FE_PICTURE_COMPLETE = 8,
	/* Picture has started decoding on VXD Backend */
	VDECFW_CHECKPOINT_BE_PICTURE_STARTED  = 9,
	/* 1st Slice has completed on VXD Backend */
	VDECFW_CHECKPOINT_BE_1SLICE_DONE      = 10,
	/* Picture decode has completed and done message sent to the Host */
	VDECFW_CHECKPOINT_BE_PICTURE_COMPLETE = 11,
#ifndef FW_STACK_USAGE_TRACKING
	/* General purpose check point 1 */
	VDECFW_CHECKPOINT_AUX1                = 12,
	/* General purpose check point 2 */
	VDECFW_CHECKPOINT_AUX2                = 13,
	/* General purpose check point 3 */
	VDECFW_CHECKPOINT_AUX3                = 14,
	/* General purpose check point 4 */
	VDECFW_CHECKPOINT_AUX4                = 15,
#endif  /* ndef FW_STACK_USAGE_TRACKING */
	VDECFW_CHECKPOINT_MAX,
	/*
	 * Indicate which checkpoints mark the start and end of each
	 * group (FW, FE and BE).
	 * The start and end values should be updated if new checkpoints are
	 * added before the current start or after the current end of any group.
	 */
	VDECFW_CHECKPOINT_FW_START            = VDECFW_CHECKPOINT_PICTURE_STARTED,
	VDECFW_CHECKPOINT_FW_END              = VDECFW_CHECKPOINT_FIRMWARE_SAVED,
	VDECFW_CHECKPOINT_FE_START            = VDECFW_CHECKPOINT_ENTDEC_STARTED,
	VDECFW_CHECKPOINT_FE_END              = VDECFW_CHECKPOINT_FE_PICTURE_COMPLETE,
	VDECFW_CHECKPOINT_BE_START            = VDECFW_CHECKPOINT_BE_PICTURE_STARTED,
	VDECFW_CHECKPOINT_BE_END              = VDECFW_CHECKPOINT_BE_PICTURE_COMPLETE,
	VDECFW_CHECKPOINT_FORCE32BITS         = 0x7FFFFFFFU
};

/* Number of auxiliary firmware checkpoints. */
#define VDECFW_CHECKPOINT_AUX_COUNT  4
/* This describes the action currently being done by the Firmware. */
enum vdecfw_firmwareaction {
	VDECFW_FWACT_IDLE = 1,           /* Firmware is currently doing nothing */
	VDECFW_FWACT_BASE_LOADING_PSR,   /* Loading parser context */
	VDECFW_FWACT_BASE_SAVING_PSR,    /* Saving parser context */
	VDECFW_FWACT_BASE_LOADING_BEMOD, /* Loading Backend module */
	VDECFW_FWACT_BASE_LOADING_FEMOD, /* Loading Frontend module */
	VDECFW_FWACT_PARSER_SLICE,       /* Parser active: parsing slice */
	VDECFW_FWACT_PARSER_PM,          /* Parser active: picture management */
	VDECFE_FWACT_BEMOD_ACTIVE,       /* Backend module active */
	VDECFE_FWACT_FEMOD_ACTIVE,       /* Frontend module active */
	VDECFW_FWACT_MAX,
	VDECFW_FWACT_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This describes the FE_ERR flags word in the VDECFW_MSGID_PIC_DECODED message
 */
enum vdecfw_msgflagdecodedfeerror {
	/* Front-end hardware watchdog timeout (FE_WDT_CM0) */
	VDECFW_MSGFLAG_DECODED_FEERROR_HWWDT_SHIFT = 0,
	/* Front-end entdec error (VEC_ERROR_DETECTED_ENTDEC) */
	VDECFW_MSGFLAG_DECODED_FEERROR_ENTDECERROR_SHIFT,
	/* Shift-register error (VEC_ERROR_DETECTED_SR) */
	VDECFW_MSGFLAG_DECODED_FEERROR_SRERROR_SHIFT,
	/* For cases when B frame comes after I without P. */
	VDECFW_MSGFLAG_DECODED_MISSING_REFERENCES_SHIFT,
	/* MMCO operation failed. */
	VDECFW_MSGFLAG_DECODED_MMCO_ERROR_SHIFT,
	/* Back-end WDT timeout */
	VDECFW_MSGFLAG_DECODED_BEERROR_HWWDT_SHIFT,
	/* Some macroblocks were dropped */
	VDECFW_MSGFLAG_DECODED_MBS_DROPPED_ERROR_SHIFT,
	VDECFW_MSGFLAG_DECODED_FEERROR_MAX,
	VDECFW_MSGFLAG_DECODED_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This type defines the IDs of the messages used to communicate with the
 * Firmware.
 *
 * The Firmware has 3 message buffers, each buffer uses a different set of IDs.
 * The buffers are:
 *   Host -> FW -Control messages(High Priority: processed in interrupt context)
 *   Host -> FW -Decode commands and associated information
 *   (Normal Priority: processed in baseloop)
 *   FW -> Host -Completion message
 */
enum vdecfw_message_id {
	/* Control Messages */
	/*
	 * Host -> FW Padding message
	 * Sent to optionally pad the message buffer
	 */
	VDECFW_MSGID_BASE_PADDING = 0x01,
	/*
	 * Host -> FW Initialisation message Initialisation should be
	 * sent *immediately* after loading the base component
	 *  ie. while the FW is idle
	 */
	VDECFW_MSGID_FIRMWARE_INIT,
	/*
	 * Host -> FW Configuration message
	 * Configuration should be setup after loading the base component
	 * and before decoding the next picture ie. while the FW is idle
	 */
	VDECFW_MSGID_FIRMWARE_CONFIG,
	/*
	 * Host -> FW Control message
	 * Firmware control command to have immediate affect
	 * eg. Stop stream, return CRCs, return Performance Data
	 */
	VDECFW_MSGID_FIRMWARE_CONTROL,
	VDECFW_MSGID_CONTROL_MAX,
	/* Decode Commands */
	/*
	 * Host -> FW Padding message
	 * Sent to optionally pad the message buffer
	 */
	VDECFW_MSGID_PSR_PADDING = 0x40,
	/*
	 * Host -> FW Decode message
	 * Describes the picture to decode
	 */
	VDECFW_MSGID_DECODE_PICTURE,
	/*
	 * Host -> FW Bitstream buffer information
	 * Information describing a bitstream buffer to DMA to VXD
	 */
	VDECFW_MSGID_BITSTREAM_BUFFER,
	/*
	 * Host -> FW Fence message
	 * Generate an interrupt when this is read,
	 * FenceID should be written to a location in VLR
	 */
	VDECFW_MSGID_FENCE,
	/*
	 * Host -> FW Batch message
	 * Contains a pointer to a host memory buffer
	 * containing a batch of decode command FW messages
	 */
	VDECFW_MSGID_BATCH,
	VDECFW_MSGID_DECODE_MAX,
	/* Completion Messages */
	/*
	 * FW -> Host Padding message
	 * Sent to optionally pad the message buffer
	 */
	VDECFW_MSGID_BE_PADDING = 0x80,
	/*
	 * FW -> Host Decoded Picture message
	 * Notification of decoded picture including errors recorded
	 */
	VDECFW_MSGID_PIC_DECODED,
	/*
	 * FW -> Host CRC message
	 * Optionally sent with Decoded Picture message, contains VXD CRCs
	 */
	VDECFW_MSGID_PIC_CRCS,
	/*
	 * FW -> Host Performance message
	 * Optional timestamps at the decode checkpoints and other information
	 * about the image to assist in measuring performance
	 */
	VDECFW_MSGID_PIC_PERFORMANCE,
	/* FW -> Host POST calculation test message */
	VDECFW_MSGID_PIC_POST_RESP,
	VDECFW_MSGID_COMPLETION_MAX,
	VDECFW_MSGID_FORCE32BITS = 0x7FFFFFFFU
};

#define VDECFW_MSGID_CONTROL_TYPES \
	(VDECFW_MSGID_CONTROL_MAX - VDECFW_MSGID_BASE_PADDING)
#define VDECFW_MSGID_DECODE_TYPES \
	(VDECFW_MSGID_DECODE_MAX - VDECFW_MSGID_PSR_PADDING)
#define VDECFW_MSGID_COMPLETION_TYPES \
	(VDECFW_MSGID_COMPLETION_MAX - VDECFW_MSGID_BE_PADDING)

/* This describes the layout of PVDEC Firmware state indicators in Comms RAM. */

/* Maximum number of PVDEC decoding pipes per core supported. */
#define VDECFW_MAX_DP  3

struct vdecfw_pvdecpipestate {
	/* TransactionID at each checkpoint */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, check_point[VDECFW_CHECKPOINT_MAX]);
	/* VDECFW_eFirmwareAction (UINT32 used to guarantee size) */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, firmware_action);
	/* Number of FE Slices processed for the last picture in FE */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, fe_slices);
	/* Number of BE Slices processed for the last picture in BE */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, be_slices);
	/*
	 * Number of FE Slices being detected as erroed for the last picture
	 * in FE
	 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, fe_errored_slices);
	/*
	 * Number of BE Slices being detected as erroed for the last picture
	 * in BE
	 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, be_errored_slices);
	/* Number of BE macroblocks dropped for the last picture */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, be_mbs_dropped);
	/* Number of BE macroblocks recovered for the last picture */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, be_mbs_recovered);
	/* Number of FE macroblocks processed for the last picture in FE */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, last_fe_mb_xy);
	/* Number of BE macroblocks processed for the last picture in BE */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, last_be_mb_xy);
	/* VDECFW_eCodecType - Codec currently loaded */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, curr_codec);
	/* TRUE if this pipe is available for processing */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, pipe_present);
};

#ifdef FW_STACK_USAGE_TRACKING
/* Stack usage info array size. */
#define VDECFW_STACK_INFO_SIZE (VDECFW_MAX_DP * VDECFW_CHECKPOINT_AUX_COUNT)
#endif /* FW_STACK_USAGE_TRACKING */
struct vdecfw_pvdecfirmwarestate {
	/*
	 * Indicates generic progress taken by firmware
	 * (must be the first item)
	 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, unsigned int, fwstep);
	/* Pipe state array. */
	struct vdecfw_pvdecpipestate pipestate[VDECFW_MAX_DP];
#ifdef FW_STACK_USAGE_TRACKING
	/* Stack usage info array. */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, unsigned int,
			stackinfo[VDECFW_STACK_INFO_SIZE]);
#endif  /* FW_STACK_USAGE_TRACKING */
};

/*
 * This describes the flags word in the aui8DisplayFlags
 * in VDECFW_sBufferControl
 */
enum vdecfw_bufflagdisplay {
	/* TID has been flushed with a "no display" indication */
	VDECFW_BUFFLAG_DISPLAY_NODISPLAY_SHIFT         = 0,
	/* TID contains an unpaired field */
	VDECFW_BUFFLAG_DISPLAY_SINGLE_FIELD_SHIFT      = 1,
	/* TID contains field coded picture(s) - single field or pair */
	VDECFW_BUFFLAG_DISPLAY_FIELD_CODED_SHIFT       = 2,
	/* if TID contains a single field, this defines which field that is */
	VDECFW_BUFFLAG_DISPLAY_BOTTOM_FIELD_SHIFT      = 3,
	/* if TID contains a frame with two interlaced fields */
	VDECFW_BUFFLAG_DISPLAY_INTERLACED_FIELDS_SHIFT = 4,
	/* End marker */
	VDECFW_BUFFLAG_DISPLAY_MAX                     = 8,
	VDECFW_BUFFLAG_DISPLAY_FORCE32BITS             = 0x7FFFFFFFU
};

/*
 * This describes the flags in the ui8PictMgmtFlags field in
 * VDECFW_sBufferControl
 */
enum vdecfw_picmgmflags {
	/* Picture management for this picture successfully executed */
	VDECFW_PICMGMTFLAG_PICTURE_EXECUTED_SHIFT   = 0,
	/*
	 * Picture management for the first field of this picture
	 * successfully executed
	 */
	VDECFW_PICMGMTFLAG_1ST_FIELD_EXECUTED_SHIFT = 0,
	/*
	 * Picture management for the second field of this picture
	 * successfully executed
	 */
	VDECFW_PICMGMTFLAG_2ND_FIELD_EXECUTED_SHIFT = 1,
	VDECFW_PICMGMTFLAG_FORCE32BITS              = 0x7FFFFFFFU
};

/*
 * Macro for checking if picture management was successfully executed for
 *  field coded picture
 */
#define VDECFW_PICMGMT_FIELD_CODED_PICTURE_EXECUTED(_flagsword_) \
	((FLAG_IS_SET(buf_control->picmgmt_flags, \
		      VDECFW_PICMGMTFLAG_1ST_FIELD_EXECUTED) && \
	  FLAG_IS_SET(buf_control->picmgmt_flags, \
		      VDECFW_PICMGMTFLAG_2ND_FIELD_EXECUTED)) ? \
	 TRUE : FALSE)
/* This describes the REAL related data for the current picture. */
struct vdecfw_real_data {
	/* Picture width */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, width);
	/* Picture height */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, height);
	/* Scaled Picture Width */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, scaled_width);
	/* Scaled Picture Height */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, scaled_height);
	/* Timestamp parsed in the firmware */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, timestamp);
};

/* This describes the HEVC related data for the current picture. */
struct vdecfw_hevcdata {
	/* POC */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, int, pic_order_count);
};

/*
 * This describes the buffer control structure that is used by the firmware to
 * signal to the Host to control the display and release of buffers.
 */
struct vdecfw_buffer_control {
	/*
	 * List of TransactionIDs indicating buffers ready to display,
	 * in display order
	 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, display_list[VDECFW_MAX_NUM_PICTURES]);
	/*
	 * List of TransactionIDs indicating buffers that are no longer
	 * required for reference
	 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int,
			release_list[VDECFW_MAX_NUM_PICTURES +
				     VDECFW_MAX_NUM_VIEWS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short,
			display_view_ids[VDECFW_MAX_NUM_PICTURES]);
	/* List of flags for each TID in the DisplayList */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, display_flags[VDECFW_MAX_NUM_PICTURES]);
	/* Number of TransactionIDs in aui32DisplayList */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, display_list_length);
	/* Number of TransactionIDs in aui32ReleaseList */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, release_list_length);
	union {
		struct vdecfw_real_data real_data;
		struct vdecfw_hevcdata hevc_data;
	};
	/*
	 * Refers to the picture decoded with the current transaction ID
	 * (not affected by merge with field of previous transaction ID)
	 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			enum img_buffer_type, dec_pict_type);
	/* Set if the current field is a pair to the previous field */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, second_field_of_pair);
	/*
	 * Set if for a pair we decoded first the top field or
	 * if we have only top field
	 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, top_field_first);
	/* Top field is first to be displayed */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, out_top_field_first);
	/* Picture management flags for this picture */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, picmgmt_flags);
	/*
	 * List of TransactionIDs indicating buffers used as references
	 * when decoding current picture
	 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, ref_list[VDECFW_MAX_NUM_PICTURES]);
};

/*
 * This describes an image buffer for one picture supplied to
 * the firmware by the host
 */
struct vdecfw_image_buffer {
	/* Virtual Address of each plane */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, byte_offset[VDECFW_PLANE_MAX]);
};

/* This type defines the picture commands that are prepared for the firmware. */
enum vdecfw_picture_cmds {
	/* Reconstructed buffer */
	/* DISPLAY_PICTURE_SIZE */
	VDECFW_CMD_DISPLAY_PICTURE,
	/* CODED_PICTURE_SIZE */
	VDECFW_CMD_CODED_PICTURE,
	/* OPERATING_MODE */
	VDECFW_CMD_OPERATING_MODE,
	/* LUMA_RECONSTRUCTED_PICTURE_BASE_ADDRESSES */
	VDECFW_CMD_LUMA_RECONSTRUCTED_PICTURE_BASE_ADDRESS,
	/* CHROMA_RECONSTRUCTED_PICTURE_BASE_ADDRESSES */
	VDECFW_CMD_CHROMA_RECONSTRUCTED_PICTURE_BASE_ADDRESS,
	/* CHROMA2_RECONSTRUCTED_PICTURE_BASE_ADDRESSES */
	VDECFW_CMD_CHROMA2_RECONSTRUCTED_PICTURE_BASE_ADDRESS,
	/* VC1_LUMA_RANGE_MAPPING_BASE_ADDRESS */
	VDECFW_CMD_LUMA_ALTERNATIVE_PICTURE_BASE_ADDRESS,
	/* VC1_CHROMA_RANGE_MAPPING_BASE_ADDRESS */
	VDECFW_CMD_CHROMA_ALTERNATIVE_PICTURE_BASE_ADDRESS,
	/* CHROMA2_ALTERNATIVE_PICTURE_BASE_ADDRESS */
	VDECFW_CMD_CHROMA2_ALTERNATIVE_PICTURE_BASE_ADDRESS,
	/* LUMA_ERROR_PICTURE_BASE_ADDRESSES */
	VDECFW_CMD_LUMA_ERROR_PICTURE_BASE_ADDRESS,
	/* CHROMA_ERROR_PICTURE_BASE_ADDRESSES */
	VDECFW_CMD_CHROMA_ERROR_PICTURE_BASE_ADDRESS,
	/* AUX_MSB_BUFFER_BASE_ADDRESSES (VC-1 only) */
	VDECFW_CMD_AUX_MSB_BUFFER,
	/* INTRA_BUFFER_BASE_ADDRESS (various) */
	VDECFW_CMD_INTRA_BUFFER_BASE_ADDRESS,
	/* AUX_LINE_BUFFER_BASE_ADDRESS */
	VDECFW_CMD_AUX_LINE_BUFFER_BASE_ADDRESS,
	/* MBFLAGS_BUFFER_BASE_ADDRESSES (VP8 only) */
	VDECFW_CMD_MBFLAGS_BUFFER_BASE_ADDRESS,
	/* FIRST_PARTITION_BASE_ADDRESSES (VP8 only) */
	VDECFW_CMD_FIRST_PARTITION_BUFFER_BASE_ADDRESS,
	/* CURRENT_PICTURE_BUFFER_BASE_ADDRESSES (VP8 only) */
	VDECFW_CMD_CURRENT_PICTURE_BUFFER_BASE_ADDRESS,
	/* SEGMENTID_BUFFER_BASE_ADDRESSES (VP8 only) */
	VDECFW_CMD_SEGMENTID_BASE_ADDRESS,
	/* EXT_OP_MODE (H.264 only) */
	VDECFW_CMD_EXT_OP_MODE,
	/* MC_CACHE_CONFIGURATION */
	VDECFW_CMD_MC_CACHE_CONFIGURATION,
	/* Alternative output buffer (rotation etc.) */
	/* ALTERNATIVE_OUTPUT_PICTURE_ROTATION */
	VDECFW_CMD_ALTERNATIVE_OUTPUT_PICTURE_ROTATION,
	/* EXTENDED_ROW_STRIDE */
	VDECFW_CMD_EXTENDED_ROW_STRIDE,
	/* CHROMA_ROW_STRIDE (H.264 only) */
	VDECFW_CMD_CHROMA_ROW_STRIDE,
	/* ALTERNATIVE_OUTPUT_CONTROL */
	VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL,
	/* RPR specific commands */
	/* RPR_AX_INITIAL */
	VDECFW_CMD_RPR_AX_INITIAL,
	/* RPR_AX_INCREMENT */
	VDECFW_CMD_RPR_AX_INCREMENT,
	/* RPR_AY_INITIAL */
	VDECFW_CMD_RPR_AY_INITIAL,
	/* RPR_AY_INCREMENT */
	VDECFW_CMD_RPR_AY_INCREMENT,
	/* RPR_PICTURE_SIZE */
	VDECFW_CMD_RPR_PICTURE_SIZE,
	/* Scaling specific params */
	/* SCALED_DISPLAY_SIZE */
	VDECFW_CMD_SCALED_DISPLAY_SIZE,
	/* HORIZONTAL_SCALE_CONTROL */
	VDECFW_CMD_HORIZONTAL_SCALE_CONTROL,
	/* SCALE_HORIZONTAL_CHROMA (H.264 only) */
	VDECFW_CMD_SCALE_HORIZONTAL_CHROMA,
	/* VERTICAL_SCALE_CONTROL */
	VDECFW_CMD_VERTICAL_SCALE_CONTROL,
	/* SCALE_VERTICAL_CHROMA (H.264 only) */
	VDECFW_CMD_SCALE_VERTICAL_CHROMA,
	/* HORIZONTAL_LUMA_COEFFICIENTS_0 */
	VDECFW_CMD_HORIZONTAL_LUMA_COEFFICIENTS_0,
	/* HORIZONTAL_LUMA_COEFFICIENTS_1 */
	VDECFW_CMD_HORIZONTAL_LUMA_COEFFICIENTS_1,
	/* HORIZONTAL_LUMA_COEFFICIENTS_2 */
	VDECFW_CMD_HORIZONTAL_LUMA_COEFFICIENTS_2,
	/* HORIZONTAL_LUMA_COEFFICIENTS_3 */
	VDECFW_CMD_HORIZONTAL_LUMA_COEFFICIENTS_3,
	/* VERTICAL_LUMA_COEFFICIENTS_0 */
	VDECFW_CMD_VERTICAL_LUMA_COEFFICIENTS_0,
	/* VERTICAL_LUMA_COEFFICIENTS_1 */
	VDECFW_CMD_VERTICAL_LUMA_COEFFICIENTS_1,
	/* VERTICAL_LUMA_COEFFICIENTS_2 */
	VDECFW_CMD_VERTICAL_LUMA_COEFFICIENTS_2,
	/* VERTICAL_LUMA_COEFFICIENTS_3 */
	VDECFW_CMD_VERTICAL_LUMA_COEFFICIENTS_3,
	/* HORIZONTAL_CHROMA_COEFFICIENTS_0 */
	VDECFW_CMD_HORIZONTAL_CHROMA_COEFFICIENTS_0,
	/* HORIZONTAL_CHROMA_COEFFICIENTS_1 */
	VDECFW_CMD_HORIZONTAL_CHROMA_COEFFICIENTS_1,
	/* HORIZONTAL_CHROMA_COEFFICIENTS_2 */
	VDECFW_CMD_HORIZONTAL_CHROMA_COEFFICIENTS_2,
	/* HORIZONTAL_CHROMA_COEFFICIENTS_3 */
	VDECFW_CMD_HORIZONTAL_CHROMA_COEFFICIENTS_3,
	/* VERTICAL_CHROMA_COEFFICIENTS_0 */
	VDECFW_CMD_VERTICAL_CHROMA_COEFFICIENTS_0,
	/* VERTICAL_CHROMA_COEFFICIENTS_1 */
	VDECFW_CMD_VERTICAL_CHROMA_COEFFICIENTS_1,
	/* VERTICAL_CHROMA_COEFFICIENTS_2 */
	VDECFW_CMD_VERTICAL_CHROMA_COEFFICIENTS_2,
	/* VERTICAL_CHROMA_COEFFICIENTS_3 */
	VDECFW_CMD_VERTICAL_CHROMA_COEFFICIENTS_3,
	/* SCALE_OUTPUT_SIZE */
	VDECFW_CMD_SCALE_OUTPUT_SIZE,
	/* VDECFW_CMD_INTRA_BUFFER_PLANE_SIZE */
	VDECFW_CMD_INTRA_BUFFER_PLANE_SIZE,
	/* VDECFW_CMD_INTRA_BUFFER_SIZE_PER_PIPE */
	VDECFW_CMD_INTRA_BUFFER_SIZE_PER_PIPE,
	/* VDECFW_CMD_AUX_LINE_BUFFER_SIZE_PER_PIPE */
	VDECFW_CMD_AUX_LINE_BUFFER_SIZE_PER_PIPE,
	VDECFW_SLICE_X_MB_OFFSET,
	VDECFW_SLICE_Y_MB_OFFSET,
	VDECFW_SLICE_TYPE,
	VDECFW_CMD_MAX,
	VDECFW_CMD_FORCE32BITS = 0x7FFFFFFFU
};

/* Size of relocation data attached to VDECFW_sTransaction message in words */
#define VDECFW_RELOC_SIZE 125

/* This structure defines the MMU Tile configuration. */
struct vdecfw_mmu_tile_config {
	/* MMU_CONTROL2 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, tilig_scheme);
	/* MMU_TILE */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int,
			mmu_tiling[MSVDX_CORE_CR_MMU_TILE_NO_ENTRIES]);
	/* MMU_TILE_EXT */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int,
			mmu_tiling_ext[MSVDX_CORE_CR_MMU_TILE_EXT_NO_ENTRIES]);
};

/*
 * This structure contains the transaction attributes to be given to the
 * firmware
 * @brief  Transaction Attributes
 */
struct vdecfw_transaction {
	/* Unique identifier for the picture (driver-wide). */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, transation_id);
	/* Codec */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			enum vdecfw_codectype, codec);
	/*
	 * Flag to indicate that the stream needs to ge handled
	 * in secure memory (if available)
	 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, secure_stream);
	/* Unique identifier for the current stream */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, stream_id);
	/* Dictates to the FW parser how the NALs are delimited */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			enum vdecfw_parsermode, parser_mode);
	/* Address from which to load the parser context data. */
	IMG_ALIGN_FIELD(VDECFW_SHARE_PTR_ALIGNMENT,
			unsigned int, ctx_load_addr);
	/*
	 * Address to save the parser state data including the updated
	 * "parser context data".
	 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_PTR_ALIGNMENT,
			unsigned int, ctx_save_addr);
	/* Size of the parser context data in bytes. */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, ctx_size);
	/* Address to save the "buffer control" data. */
	IMG_ALIGN_FIELD(VDECFW_SHARE_PTR_ALIGNMENT,
			unsigned int, ctrl_save_addr);
	/* Size of the buffer control data in bytes. */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, ctrl_size);

	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, pict_cmds[VDECFW_CMD_MAX]);

	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, pic_width_inmbs);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, pic_height_inmbs);

	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, mbparams_base_addr);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, mbparams_size_per_plane);
	/* Address of VLC table data. */
	IMG_ALIGN_FIELD(VDECFW_SHARE_PTR_ALIGNMENT,
			unsigned int, vlc_tables_addr);
	/* Size of the VLC table data in bytes. */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, vlc_tables_size);
	/* Address of VLC index table data. */
	IMG_ALIGN_FIELD(VDECFW_SHARE_PTR_ALIGNMENT,
			unsigned int, vlc_index_table_addr);
	/* Size of the VLC index table data in bytes. */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, vlc_index_table_size);
	/* Address of parser picture header. */
	IMG_ALIGN_FIELD(VDECFW_SHARE_PTR_ALIGNMENT,
			unsigned int, psr_hdr_addr);
	/* Size of the parser picture header in bytes. */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, psr_hdr_size);
	/* Address of Sequence Info in the Host (secure) */
	IMG_ALIGN_FIELD(VDECFW_SHARE_PTR_ALIGNMENT,
			unsigned int, sequence_info_source);
	/* Address of PPS Info in the Host (secure) */
	IMG_ALIGN_FIELD(VDECFW_SHARE_PTR_ALIGNMENT,
			unsigned int, pps_info_source);
	/* Address of Second PPS Info in the Host (secure) */
	IMG_ALIGN_FIELD(VDECFW_SHARE_PTR_ALIGNMENT,
			unsigned int, second_pps_info_source);
	/* MMU Tile config comes down with each transaction. */
	struct vdecfw_mmu_tile_config mmu_tile_config;
};

/*
 * This structure contains the info for extracting a subset of VLC tables
 * indexed inside the index table.
 * aui32VlcTablesOffset is the offset to the first table inside the index table
 * aui32VlcConsecutiveTables indicates the consecutive number of entries (from
 * aui32VlcTablesOffset to aui32VlcTablesOffset+aui32VlcConsecutiveTables)
 * which will be copied.
 */
struct vdecfw_vlc_table_info {
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, vlc_table_offset);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, vlc_consecutive_tables);
};

/* This structure defines the RENDEC buffer configuration. */
struct vdecfw_rendec_config {
	/* VEC_RENDEC_CONTROL0 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, regvec_rendec_control0);
	/* VEC_RENDEC_CONTROL1 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, regvec_rendec_control1);
	/* VEC_RENDEC_BASE_ADDR0 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, rendec_buffer_baseaddr0);
	/* VEC_RENDEC_BASE_ADDR1 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, rendec_buffer_baseaddr1);
	/* VEC_RENDEC_BUFFER_SIZE */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, regvec_rendec_buffer_size);
	/* VEC_RENDEC_CONTEXT0 - VEC_RENDEC_CONTEXT5 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, rendec_initial_ctx[6]);
};

#endif /* _VDECFW_H_ */
