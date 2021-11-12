/* SPDX-License-Identifier: GPL-2.0 */
/*
 * firmware header
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

#ifndef _TOPAZSCFWIF_H_
#define _TOPAZSCFWIF_H_

#include "coreflags.h"
#include <linux/types.h>

#define MAX_QP_H264			(51)

/*
 * The number of bytes used by each MVEA MV param & above param region
 */
#define MVEA_MV_PARAM_REGION_SIZE 16
#define MVEA_ABOVE_PARAM_REGION_SIZE 96

/*
 * Macros to align to the correct number of bytes
 */
#define ALIGN_4(X)  (((X) + 3) & ~3)
#define ALIGN_16(X)  (((X) + 15) & ~15)
#define ALIGN_64(X)  (((X) + 63) & ~63)
#define ALIGN_128(X)  (((X) + 127) & ~127)
#define ALIGN_1024(X)	(((X) + 1023) & ~1023)

/*
 * Context size allocated from host application
 */
#define MTX_CONTEXT_SIZE (13 * 1024)

/*
 * SEI (Buffering Period and Picture Timing) Constants shared
 * between host and firmware
 */
#define BPH_SEI_NAL_INITIAL_CPB_REMOVAL_DELAY_SIZE 23
#define PTH_SEI_NAL_CPB_REMOVAL_DELAY_SIZE 23
#define PTH_SEI_NAL_DPB_OUTPUT_DELAY_SIZE 7

/*
 * Size of the header in output coded buffer. This varies based on
 *              whether data logging is enabled/disabled
 */
#if defined(INCLUDE_CRC_REGISTER_CHECKS)
#define CRC_REGISTER_FEEDBACK_SIZE	(80 * 4)
#else
#define CRC_REGISTER_FEEDBACK_SIZE	0
#endif

/* MUST be aligned to the DMA 64 byte boundary condition
 * (CRC data is DMA'd after the coded buffer header)
 */
#define CODED_BUFFER_HEADER_SIZE    64
#define CODED_BUFFER_INFO_SECTION_SIZE (CODED_BUFFER_HEADER_SIZE + CRC_REGISTER_FEEDBACK_SIZE)

/*
 * Mask defines for the -ui8EnableSelStatsFlags variable
 */
#define ESF_FIRST_STAGE_STATS 1
#define ESF_MP_BEST_MB_DECISION_STATS 2
#define ESF_MP_BEST_MOTION_VECTOR_STATS 4

#define CUSTOM_QUANT_PARAMSIZE_8x8 2

/*
 * Combined size of H.264 quantization lists (6 * 16 + {2 or 6} * 64)
 */
#define QUANT_LISTS_SIZE		(6 * 16 + CUSTOM_QUANT_PARAMSIZE_8x8 * 64)

/*
 * Size in bytes and words of memory to transfer partially coded header data
 */
#define MAX_HEADERSIZEBYTES		(128)
#define MAX_HEADERSIZEWORDS		(32)

/*
 * Maximum number of slices per field
 */
#define MAX_SLICESPERPIC		(128)

/*
 * Picture parameter flags used in the PIC_PARAM structure
 */
#define ISINTERP_FLAGS					(0x00000001)
#define ISRC_FLAGS					(0x00000010)
#define ISRC_I16BIAS					(0x00000020)
#define ISINTERB_FLAGS					(0x00000080)
#define ISSCENE_DISABLED				(0x00000100)
#define ISMULTIREF_FLAGS				(0x00000200)
#define SPATIALDIRECT_FLAGS				(0x00000400)

/*
 * Enum describing contents of scratch registers
 */
enum mtx_scratch_regdata {
	MTX_SCRATCHREG_BOOTSTATUS = 0,
	MTX_SCRATCHREG_UNUSED = 0,
	MTX_SCRATCHREG_TOHOST,			//!< Reg for MTX->Host data
	MTX_SCRATCHREG_TOMTX,			//!< Reg for Host->MTX data

	MTX_SCRATCHREG_SIZE,				//!< End marker for enum
	MTX_SCRATCHREG_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * `MTX_SCRATCHREG_IDLE` register that is used for synchronous communication and debug.
 *
 * Current register usage:
 * <bits, inclusive boundaries> : <usage>
 * 2-10 : Number of executed commands (mod 255)
 * 0-1 : FW idle status
 */
#define MTX_SCRATCHREG_IDLE	TOPAZHP_TOP_CR_FIRMWARE_REG_4 //!< Reg for firmware IDLE status

/* Flags relating to MTX_SCRATCHREG_IDLE */
/* Bits [10-22] are used for the line information */
/* TOPAZHP_LINE_COUNTER (see TRM 8.1.1) uses 12 bits for the line count */
#define SHIFT_FW_IDLE_REG_STATUS		(0)
#define MASK_FW_IDLE_REG_STATUS			(3)
#define FW_IDLE_STATUS_IDLE			(1)

/*
 * In secure FW mode the first value written to the command FIFO is copied to MMU_CONTROL_0
 * by the firmware. When we don't want that to happen we can write this value instead.
 * The firmware will know to ignore it as
 * long as it is written BEFORE the firmware starts up
 */
#define TOPAZHP_NON_SECURE_FW_MARKER	(0xffffffff)

/*
 * This value is an arbitrary value that the firmware will write to TOPAZHP_TOP_CR_FIRMWARE_REG_1
 * (MTX_SCRATCHREG_BOOTSTATUS)
 * when it has completed the boot process to indicate that it is ready
 */
#define TOPAZHP_FW_BOOT_SIGNAL		(0x12345678)

/*
 * Sizes for arrays that depend on reference usage pattern
 */
#define MAX_REF_B_LEVELS		3
#define MAX_REF_SPACING			1
#define MAX_REF_I_OR_P_LEVELS	(MAX_REF_SPACING + 2)
#define MAX_REF_LEVELS		(MAX_REF_B_LEVELS + MAX_REF_I_OR_P_LEVELS)
#define MAX_PIC_NODES		(MAX_REF_LEVELS + 2)
#define MAX_MV			(MAX_PIC_NODES * 2)

#define MAX_BFRAMES	7           //B-frame count limit for Hierarchical mode
#define MAX_GOP_SIZE		(MAX_BFRAMES + 1)
#define MAX_SOURCE_SLOTS_SL	(MAX_GOP_SIZE + 1)

#define MV_ROW_STRIDE (ALIGN_64(sizeof(struct img_mv_settings) * MAX_BFRAMES))

/*
 * MTX -> host message FIFO
 */
#define LOG2_WB_FIFO_SIZE	(5)

#define WB_FIFO_SIZE		(1 << (LOG2_WB_FIFO_SIZE))

#define SHIFT_WB_PRODUCER	(0)
#define MASK_WB_PRODUCER	(((1 << LOG2_WB_FIFO_SIZE) - 1) << SHIFT_WB_PRODUCER)

#define SHIFT_WB_CONSUMER	(0)
#define MASK_WB_CONSUMER	(((1 << LOG2_WB_FIFO_SIZE) - 1) << SHIFT_WB_CONSUMER)

/*
 * Number of buffers per encode task (default: 2 - double bufferring)
 */
#define CODED_BUFFERING_CNT 2 //default to double-buffering

/*
 * Calculates the ideal minimun coded buffers for a frame level encode
 */
#define CALC_OPTIMAL_CODED_PACKAGES_FRAME_ENCODE(numcores, isinterlaced) \
	((((isinterlaced) ? 2 : 1) * (numcores)) * CODED_BUFFERING_CNT)

/*
 * Calculates the ideal minimum coded buffers for a slice level encode
 */
#define CALC_OPTIMAL_CODED_PACKAGES_SLICE_ENCODE(slicesperpic) \
						((slicesperpic) * CODED_BUFFERING_CNT)

/*
 * Calculates the ideal minimum coded buffers for an encode
 */
#define CALC_OPTIMAL_CODED_PACKAGES_ENCODE(bis_slice_level, slicesperpic, numcores, isinterlaced) \
	(bis_slice_level ? CALC_OPTIMAL_CODED_PACKAGES_SLICE_ENCODE(slicesperpic) \
	 : CALC_OPTIMAL_CODED_PACKAGES_FRAME_ENCODE(numcores, isinterlaced))

/*
 * Calculates the actual number of coded buffers that can be used for an encode
 */
#define CALC_NUM_CODED_PACKAGES_ENCODE(bis_slice_level, slicesperpic, numcores, isinterlaced) \
	(CALC_OPTIMAL_CODED_PACKAGES_ENCODE(bis_slice_level, slicesperpic, numcores, isinterlaced))

/*
 * Maximum number of coded packages
 */
#define MAX_CODED_PACKAGES	CALC_NUM_CODED_PACKAGES_ENCODE(0, 0, TOPAZHP_MAX_NUM_PIPES, 1)

/*
 * DMA configuration parameters
 */
#define MTX_DMA_BURSTSIZE_BYTES 32

/*
 * types that should be in DMAC header file
 */
enum dmac_acc_del {
	DMAC_ACC_DEL_0 = 0x0,			//!< Access delay zero clock cycles
	DMAC_ACC_DEL_256 = 0x1,			//!< Access delay 256 clock cycles
	DMAC_ACC_DEL_512 = 0x2,			//!< Access delay 512 clock cycles
	DMAC_ACC_DEL_768 = 0x3,			//!< Access delay 768 clock cycles
	DMAC_ACC_DEL_1024 = 0x4,		//!< Access delay 1024 clock cycles
	DMAC_ACC_DEL_1280 = 0x5,		//!< Access delay 1280 clock cycles
	DMAC_ACC_DEL_1536 = 0x6,		//!< Access delay 1536 clock cycles
	DMAC_ACC_DEL_1792 = 0x7,		//!< Access delay 1792 clock cycles
	DMAC_ACC_FORCE32BITS = 0x7FFFFFFFU
};

enum dmac_bswap {
	DMAC_BSWAP_NO_SWAP = 0x0,		//!< No byte swapping will be performed.
	DMAC_BSWAP_REVERSE = 0x1,		//!< Byte order will be reversed.
	DMAC_BSWAP_FORCE32BITS = 0x7FFFFFFFU
};

enum dmac_burst {
	DMAC_BURST_0 = 0x0,				//!< burst size of 0
	DMAC_BURST_1 = 0x1,				//!< burst size of 1
	DMAC_BURST_2 = 0x2,				//!< burst size of 2
	DMAC_BURST_3 = 0x3,				//!< burst size of 3
	DMAC_BURST_4 = 0x4,				//!< burst size of 4
	DMAC_BURST_5 = 0x5,				//!< burst size of 5
	DMAC_BURST_6 = 0x6,				//!< burst size of 6
	DMAC_BURST_7 = 0x7,				//!< burst size of 7
	DMAC_BURST_FORCE32BITS = 0x7FFFFFFFU
};

#define DMAC_VALUE_COUNT(BSWAP, PW, DIR, PERIPH_INCR, COUNT)			 \
	((((BSWAP)		<< SHIFT_IMG_SOC_BSWAP)	& MASK_IMG_SOC_BSWAP)	|\
	(((PW)			<< SHIFT_IMG_SOC_PW)	& MASK_IMG_SOC_PW)		|\
	(((DIR)			<< SHIFT_IMG_SOC_DIR)	& MASK_IMG_SOC_DIR)		|\
	(((PERIPH_INCR)	<< SHIFT_IMG_SOC_PI)	& MASK_IMG_SOC_PI)		|\
	(((COUNT)		<< SHIFT_IMG_SOC_CNT)	& MASK_IMG_SOC_CNT))

#define DMAC_VALUE_PERIPH_PARAM(ACC_DEL, INCR, BURST)					 \
	((((ACC_DEL)	<< SHIFT_IMG_SOC_ACC_DEL)	& MASK_IMG_SOC_ACC_DEL)	|\
	(((INCR)	<< SHIFT_IMG_SOC_INCR)		& MASK_IMG_SOC_INCR)	|\
	(((BURST)	<< SHIFT_IMG_SOC_BURST)		& MASK_IMG_SOC_BURST))

enum dmac_pw {
	DMAC_PWIDTH_32_BIT = 0x0,		//!< Peripheral width 32-bit.
	DMAC_PWIDTH_16_BIT = 0x1,		//!< Peripheral width 16-bit.
	DMAC_PWIDTH_8_BIT = 0x2,		//!< Peripheral width 8-bit.
	DMAC_PWIDTH_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Enum describing Command IDs.  Some commands require data to be DMA'd in
 * from the Host, with the base address of the data specified in the Command
 * Data Address word of the command.  The data required is specific to each
 * command type.
 */
enum mtx_cmd_id {
	// Common Commands
	MTX_CMDID_NULL,			//!< (no data)\n Null command does nothing\n
	MTX_CMDID_SHUTDOWN,		//!< (no data)\n shutdown the MTX\n

	// Video Commands
	/* !< (extra data: #MTX_HEADER_PARAMS) Command for Sequence, Picture and Slice headers */
	MTX_CMDID_DO_HEADER,
	/* !< (data: low latency encode activation, HBI usage) Encode frame data*/
	MTX_CMDID_ENCODE_FRAME,
	MTX_CMDID_START_FRAME,				//!< (no data)\n Prepare to encode frame\n
	MTX_CMDID_ENCODE_SLICE,				//!< (no data)\n Encode slice data\n
	MTX_CMDID_END_FRAME,				//!< (no data)\n Complete frame encoding\n
	/* !< (data: pipe number, extra data: #IMG_MTX_VIDEO_CONTEXT)\n Set MTX Video Context */
	MTX_CMDID_SETVIDEO,
	/* !< (data: pipe number, extra data: #IMG_MTX_VIDEO_CONTEXT)
	 * Get MTX Video Context
	 */
	MTX_CMDID_GETVIDEO,
	/* !< (data: new pipe allocations for the context)
	 * Change pipe allocation for a Video Context
	 */
	MTX_CMDID_DO_CHANGE_PIPEWORK,
#if SECURE_IO_PORTS
	MTX_CMDID_SECUREIO,					//!< (data: )\n Change IO security\n
#endif
	/* !< (data: subtype and parameters, extra data: #IMG_PICMGMT_CUSTOM_QUANT_DATA
	 * (optional))\n Change encoding parameters
	 */
	MTX_CMDID_PICMGMT,
	/* !< (data: QP and bitrate)\n Change encoding parameters */
	MTX_CMDID_RC_UPDATE,
	/* !< (extra data: #IMG_SOURCE_BUFFER_PARAMS)
	 * Transfer source buffer from host
	 */
	MTX_CMDID_PROVIDE_SOURCE_BUFFER,
	/* !< (data: buffer parameters, extra data: reference buffer)
	 * Transfer reference buffer from host
	 */
	MTX_CMDID_PROVIDE_REF_BUFFER,
	/* !< (data: slot and size, extra data: coded package)\n Transfer coded package from host
	 *(coded package contains addresses of header and coded output buffers/1st linked list node)
	 */
	MTX_CMDID_PROVIDE_CODEDPACKAGE_BUFFER,
	MTX_CMDID_ABORT,		//!< (no data)\n Stop encoding and release all buffers\n

	// JPEG commands
	MTX_CMDID_SETQUANT,			//!< (extra data: #JPEG_MTX_QUANT_TABLE)\n
	MTX_CMDID_SETUP_INTERFACE,		//!< (extra data: #JPEG WRITEBACK POINTERS)\n
	MTX_CMDID_ISSUEBUFF,			//!< (extra data: #MTX_ISSUE_BUFFERS)\n
	MTX_CMDID_SETUP,			//!< (extra data: #JPEG_MTX_DMA_SETUP)\n\n
	/* !< (extra data: #IMG_VXE_SCALER_SETUP)\nChange source
	 * pixel format after context creation\
	 */
	MTX_CMDID_UPDATE_SOURCE_FORMAT,
	/* !< (extra data: #IMG_VXE_CSC_SETUP)\nChange Colour Space Conversion setup dynamically */
	MTX_CMDID_UPDATE_CSC,

	MTX_CMDID_ENDMARKER,					//!< end marker for enum
	MTX_CMDID_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Priority for the command.
 * Each Command ID will only work with the correct priority.
 */
#define MTX_CMDID_PRIORITY 0x80

/*
 * Indicates whether or not to issue an interrupt when the firmware sends the
 * command's writeback message.
 */
#define MTX_CMDID_WB_INTERRUPT 0x8000

/*
 * Enum describing response IDs
 */
enum mtx_message_id {
	MTX_MESSAGE_ACK,
	MTX_MESSAGE_CODED,
	MTX_MESSAGE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Mask and shift values for command word
 */
#define SHIFT_MTX_MSG_CMD_ID		(0)
#define MASK_MTX_MSG_CMD_ID			(0x7f << SHIFT_MTX_MSG_CMD_ID)
#define SHIFT_MTX_MSG_PRIORITY		(7)
#define MASK_MTX_MSG_PRIORITY		(0x1 << SHIFT_MTX_MSG_PRIORITY)
#define SHIFT_MTX_MSG_CORE			(8)
#define MASK_MTX_MSG_CORE			(0x7f << SHIFT_MTX_MSG_CORE)
#define SHIFT_MTX_MSG_COUNT			(16)
#define MASK_MTX_MSG_COUNT			(0xffffU << SHIFT_MTX_MSG_COUNT)
#define SHIFT_MTX_MSG_MESSAGE_ID	(16)
#define MASK_MTX_MSG_MESSAGE_ID		(0xff << SHIFT_MTX_MSG_MESSAGE_ID)

/*
 * Mask and shift values for data word
 */
#define SHIFT_MTX_MSG_ENCODE_CODED_INTERRUPT		(0)
#define MASK_MTX_MSG_ENCODE_CODED_INTERRUPT \
	(0xff << SHIFT_MTX_MSG_ENCODE_CODED_INTERRUPT)
#define SHIFT_MTX_MSG_ENCODE_USE_LINE_COUNTER		(20)
#define MASK_MTX_MSG_ENCODE_USE_LINE_COUNTER \
	(0x1 << SHIFT_MTX_MSG_ENCODE_USE_LINE_COUNTER)

#define SHIFT_MTX_MSG_PICMGMT_SUBTYPE			(0)
#define MASK_MTX_MSG_PICMGMT_SUBTYPE			(0xff << SHIFT_MTX_MSG_PICMGMT_SUBTYPE)
#define SHIFT_MTX_MSG_PICMGMT_DATA			(8)
#define MASK_MTX_MSG_PICMGMT_DATA			(0xffffffU << SHIFT_MTX_MSG_PICMGMT_DATA)
#define SHIFT_MTX_MSG_PICMGMT_STRIDE_Y			(0)
#define MASK_MTX_MSG_PICMGMT_STRIDE_Y			(0x3ff << SHIFT_MTX_MSG_PICMGMT_STRIDE_Y)
#define SHIFT_MTX_MSG_PICMGMT_STRIDE_UV			(10)
#define MASK_MTX_MSG_PICMGMT_STRIDE_UV			(0x3ff << SHIFT_MTX_MSG_PICMGMT_STRIDE_UV)

/*Values for updating static Qp values when Rate Control is disabled*/
#define SHIFT_MTX_MSG_NUM_CODED_BUFFERS_PER_HEADER			(5)
#define MASK_MTX_MSG_NUM_CODED_BUFFERS_PER_HEADER \
	(0xf << SHIFT_MTX_MSG_NUM_CODED_BUFFERS_PER_HEADER)

#define SHIFT_MTX_MSG_PROVIDE_CODEDPACKAGE_BUFFER_SLOT	(0)
#define MASK_MTX_MSG_PROVIDE_CODEDPACKAGE_BUFFER_SLOT \
	(0x0f << SHIFT_MTX_MSG_PROVIDE_CODEDPACKAGE_BUFFER_SLOT)
#define SHIFT_MTX_MSG_PROVIDE_CODED_BUFFER_SIZE		(4)
#define MASK_MTX_MSG_PROVIDE_CODED_BUFFER_SIZE \
	(0x3fffff << SHIFT_MTX_MSG_PROVIDE_CODED_BUFFER_SIZE)

/*
 * Enum describing partially coded header element types
 */
enum header_element_type {
	ELEMENT_STARTCODE_RAWDATA = 0,	//!< Raw data that includes a start code
	/*!< Raw data that includes a start code in the middle of the header */
	ELEMENT_STARTCODE_MIDHDR,
	ELEMENT_RAWDATA,	//!< Raw data
	ELEMENT_QP,		//!< Insert the H264 Picture Header QP parameter
	ELEMENT_SQP,		//!< Insert the H264 Slice Header QP parameter
	/* Insert the H263/MPEG4 Frame Q_scale parameter (vob_quant field) */
	ELEMENT_FRAMEQSCALE,
	/* !< Insert the H263/MPEG4 Slice Q_scale parameter (quant_scale field) */
	ELEMENT_SLICEQSCALE,
	ELEMENT_INSERTBYTEALIGN_H264,	//!< Insert the byte alignment bits for H264
	ELEMENT_INSERTBYTEALIGN_MPG4,	//!< Insert the byte alignment bits for MPEG4
	ELEMENT_INSERTBYTEALIGN_MPG2,	//!< Insert the byte alignment bits for MPEG2
	ELEMENT_VBV_MPG2,
	ELEMENT_TEMPORAL_REF_MPG2,
	ELEMENT_CURRMBNR,	//!< Insert the current macrloblock number for a slice.

	/* !< Insert frame_num field (used as ID for ref. pictures in H264) */
	ELEMENT_FRAME_NUM,	//!< Insert frame_num field (used as ID for ref. pictures in H264)
	/* !< Insert Temporal Reference field (used as ID for ref. pictures in H263) */
	ELEMENT_TEMPORAL_REFERENCE,
	ELEMENT_EXTENDED_TR,		//!< Insert Extended Temporal Reference field
	/*//!< Insert idr_pic_id field (used to distinguish consecutive IDR frames) */
	ELEMENT_IDR_PIC_ID,
	/* !< Insert pic_order_cnt_lsb field (used for display ordering in H264) */
	ELEMENT_PIC_ORDER_CNT,
	/* !< Insert gob_frame_id field (used for display ordering in H263) */
	ELEMENT_GOB_FRAME_ID,
	/* !< Insert vop_time_increment field (used for display ordering in MPEG4) */
	ELEMENT_VOP_TIME_INCREMENT,
	/* !< Insert modulo_time_base used in MPEG4 (depends on vop_time_increment_resolution) */
	ELEMENT_MODULO_TIME_BASE,

	ELEMENT_BOTTOM_FIELD,			//!< Insert bottom_field flag
	ELEMENT_SLICE_NUM,			//!< Insert slice num (used for GOB headers in H263)
	ELEMENT_MPEG2_SLICE_VERTICAL_POS,	//!< Insert slice vertical pos (MPEG2 slice header)
	/* !< Insert 1 bit flag indicating if slice is Intra or not (MPEG2 slice header) */
	ELEMENT_MPEG2_IS_INTRA_SLICE,
	/* !< Insert 2 bit field indicating if the current header is for a frame picture (11),
	 * top field (01) or bottom field (10) - (MPEG2 picture header
	 */
	ELEMENT_MPEG2_PICTURE_STRUCTURE,
	/* !< Insert flag indicating whether or not this picture is a reference */
	ELEMENT_REFERENCE,
	ELEMENT_ADAPTIVE,		//!< Insert reference picture marking
	ELEMENT_DIRECT_SPATIAL_MV_FLAG,	//!< Insert spatial direct mode flag
	ELEMENT_NUM_REF_IDX_ACTIVE,	//!< Insert number of active references
	ELEMENT_REORDER_L0,		//!< Insert reference list 0 reordering
	ELEMENT_REORDER_L1,		//!< Insert reference list 1 reordering
	ELEMENT_TEMPORAL_ID,		//!< Insert temporal ID of the picture, used for MVC header
	/*!< Insert flag indicating whether or not this picture is an anchor picture */
	ELEMENT_ANCHOR_PIC_FLAG,

	BPH_SEI_NAL_INITIAL_CPB_REMOVAL_DELAY,	//!< Insert nal_initial_cpb_removal_delay
	/* !< Insert nal_initial_cpb_removal_delay_offset */
	BPH_SEI_NAL_INITIAL_CPB_REMOVAL_DELAY_OFFSET,
	PTH_SEI_NAL_CPB_REMOVAL_DELAY,				//!< Insert cpb_removal_delay
	PTH_SEI_NAL_DPB_OUTPUT_DELAY,				//!< Insert dpb_output_delay

	ELEMENT_SLICEWEIGHTEDPREDICTIONSTRUCT,	//!< Insert weighted prediciton parameters
	ELEMENT_CUSTOM_QUANT,			//!< Insert custom quantization values
	ELEMENT_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Struct describing a partially coded header element
 */
struct mtx_header_element {
	enum header_element_type element_type;	//!< Element type
	/* !< Number of bits of coded data to be inserted */
	unsigned char			size;
	unsigned char			bits;		//!< Raw data to be inserted.
};

/*
 * Struct describing partially coded header parameters
 */
struct mtx_header_params {
	unsigned int		elements;	//!< Number of header elements
	/*!< array of element data */
	struct mtx_header_element element_stream[MAX_HEADERSIZEWORDS - 1];
};

/*
 * Enum describing threshold values for skipped MB biasing
 */
enum th_skip_scale {
	TH_SKIP_0 = 0,		//!< Bias threshold for QP 0 to 12
	TH_SKIP_12 = 1,		//!< Bias threshold for QP 12 to 24
	TH_SKIP_24 = 2,		//!< Bias threshold for QP 24 and above
	TH_SKIP_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Struct describing rate control input parameters
 */
struct in_rc_params {
	unsigned int	mb_per_frm;		//!< Number of MBs Per Frame
	unsigned int	mb_per_bu;		//!< Number of MBs Per BU
	unsigned short	bu_per_frm;		//!< Number of BUs Per Frame

	unsigned short	intra_period;	//!< Intra frame frequency
	unsigned short	bframes;		//!< B frame frequency

	int	bits_per_frm;		//!< Bits Per Frame
	int	bits_per_bu;		//!< Bits Per BU

	int	bit_rate;			//!< Bit Rate (bps)
	int	buffer_size;		//!< Size of Buffer in bits
	int	buffer_size_frames;//!< Size of Buffer in frames, to be used in VCM
	int	initial_level;	//!< Initial Level of Buffer
	int	initial_delay;	//!< Initial Delay of Buffer

	unsigned short	frm_skip_disable;	//!< Disable Frame skipping

	unsigned char	se_init_qp_i;		//!< Initial QP for sequence (I frames)
	unsigned char	se_init_qp_p;		//!< Initial QP for sequence (P frames)
	unsigned char	se_init_qp_b;		//!< Initial QP for sequence (B frames)

	unsigned char	min_qp;		//!< Minimum QP value to use
	unsigned char	max_qp;		//!< Maximum QP value to use

	/* !< Scale Factor used to limit the range
	 * of arithmetic with high resolutions and bitrates
	 */
	unsigned char	scale_factor;
	unsigned short	mb_per_row;		//!< Number of MBs Per Row

	unsigned short	disable_vcm_hardware; //!< Disable using vcm hardware in RC modes.

	union {
		struct {
			/* !< Rate at which bits are sent from encoder
			 * to the output after each frame finished encoding
			 */
			int	transfer_rate;
			/* !< Disable Scene Change detection */
			unsigned short	sc_detect_disable;
			/* !< Flag indicating Hierarchical B Pic or Flat mode rate control */
			unsigned short	hierarchical_mode;
			/* !< Constant used in rate control =
			 * (GopSize/(BufferSize-InitialLevel))*256
			 */
			unsigned int	rc_scale_factor;
			/* !< Enable movement of slice boundary when Qp is high */
			unsigned short	enable_slice_bob;
			/* !< Maximum number of rows the slice boundary can be moved */
			unsigned char  max_slice_bob;
			/* !< Minimum Qp at which slice bobbing should take place */
			unsigned char  slice_bob_qp;
		} h264;
		struct {
			unsigned char	half_framerate;	//!< Half Frame Rate (MP4 only)
			unsigned char	f_code;			//!< F Code (MP4 only)
			int	bits_pergop;		//!< Bits Per GOP (MP4 only)
			unsigned short	bu_skip_disable;		//!< Disable BU skipping
			int	bits_per_mb;		//!< Bits Per MB
			unsigned short	avg_qp_val;		//!< Average QP in Current Picture
			unsigned short	initial_qp;		//!< Initial Quantizer
		} other;
	} mode;
};

/*
 * Enum describing MTX firmware version (codec and rate control)
 */
enum img_codec {
	IMG_CODEC_NONE = 0,		//!< There is no FW in MTX memory
	IMG_CODEC_JPEG,			//!< JPEG
	IMG_CODEC_H264_NO_RC,		//!< H264 with no rate control
	IMG_CODEC_H264_VBR,		//!< H264 variable bitrate
	IMG_CODEC_H264_CBR,		//!< H264 constant bitrate
	IMG_CODEC_H264_VCM,		//!< H264 video conferance mode
	IMG_CODEC_H263_NO_RC,		//!< H263 with no rate control
	IMG_CODEC_H263_VBR,		//!< H263 variable bitrate
	IMG_CODEC_H263_CBR,		//!< H263 constant bitrate
	IMG_CODEC_MPEG4_NO_RC,		//!< MPEG4 with no rate control
	IMG_CODEC_MPEG4_VBR,		//!< MPEG4 variable bitrate
	IMG_CODEC_MPEG4_CBR,		//!< MPEG4 constant bitrate
	IMG_CODEC_MPEG2_NO_RC,		//!< MPEG2 with no rate control
	IMG_CODEC_MPEG2_VBR,		//!< MPEG2 variable bitrate
	IMG_CODEC_MPEG2_CBR,		//!< MPEG2 constant bitrate
	IMG_CODEC_H264_ERC,		//!< H264 example rate control
	IMG_CODEC_H263_ERC,		//!< H263 example rate control
	IMG_CODEC_MPEG4_ERC,		//!< MPEG4 example rate control
	IMG_CODEC_MPEG2_ERC,		//!< MPEG2 example rate control
	IMG_CODEC_H264MVC_NO_RC,	//!< MVC H264 with no rate control
	IMG_CODEC_H264MVC_CBR,		//!< MVC H264 constant bitrate
	IMG_CODEC_H264MVC_VBR,		//!< MVC H264 variable bitrate
	IMG_CODEC_H264MVC_ERC,		//!< MVC H264 example rate control
	IMG_CODEC_H264_ALL_RC,		//!< H264 with multiple rate control modes
	IMG_CODEC_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Enum describing encoding standard (codec)
 */
enum img_standard {
	IMG_STANDARD_NONE = 0,	//!< There is no FW in MTX memory
	IMG_STANDARD_JPEG,		//!< JPEG
	IMG_STANDARD_H264,		//!< H264 with no rate control
	IMG_STANDARD_H263,		//!< H263 with no rate control
	IMG_STANDARD_MPEG4,		//!< MPEG4 with no rate control
	IMG_STANDARD_MPEG2,		//!< MPEG2 with no rate control
	IMG_STANDARD_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Enum describing image surface format types
 */
enum img_format {
	IMG_CODEC_420_YUV = 100,			//!< Planar Y U V
	IMG_CODEC_420_YV12 = 44,			//!< YV12 format Data
	IMG_CODEC_420_IMC2 = 36,			//!< IMC2 format Data
	IMG_CODEC_420_PL8 = 47,			//!< PL8 format YUV data
	IMG_CODEC_420_PL12 = 101,		//!< PL12 format YUV data
	/* |< PL12 format packed into a single plane (not currently supported by JPEG) */
	IMG_CODEC_420_PL12_PACKED = 25,
	/* !< PL21 format packed into a single plane (not currently supported by JPEG) */
	IMG_CODEC_420_PL21_PACKED = 26,
	/* !< YUV format 4:2:2 data; start the incrementing auto enumeration
	 * values after the last ones we have used.
	 */
	IMG_CODEC_422_YUV = 102,
	IMG_CODEC_422_YV12,			//!< YV12 format 4:2:2 data
	IMG_CODEC_422_PL8,			//!< PL8 format 4:2:2 data
	IMG_CODEC_422_IMC2,			//!< IMC2 format 4:2:2 data
	IMG_CODEC_422_PL12,			//!< PL12 format 4:2:2 data
	IMG_CODEC_Y0UY1V_8888,		//!< 4:2:2 YUYV data
	IMG_CODEC_Y0VY1U_8888,		//!< 4:2:2 YVYU data
	IMG_CODEC_UY0VY1_8888,		//!< 4:2:2 UYVY data
	IMG_CODEC_VY0UY1_8888,		//!< 4:2:2 VYUY data
	IMG_CODEC_444_YUV,	//!< YUV format 4:4:4 data (not currently supported by JPEG)
	IMG_CODEC_444_YV12,	//!< YV12 format 4:4:4 data (not currently supported by JPEG)
	IMG_CODEC_444_PL8,	//!< PL8 format 4:4:4 data (not currently supported by JPEG)
	IMG_CODEC_444_IMC2,	//!< PL8 format 4:4:4 data (not currently supported by JPEG)
	IMG_CODEC_444_PL12,	//!< PL12 format 4:4:4 data (not currently supported by JPEG)
	IMG_CODEC_ABCX,		//!< Interleaved 4:4:4 data (not currently supported by JPEG)
	IMG_CODEC_XBCA,		//!< Interleaved 4:4:4 data (not currently supported by JPEG)
	IMG_CODEC_ABC565,	//!< Packed 4:4:4 data (not currently supported by JPEG)

	IMG_CODEC_420_PL21,		//!< PL21 format YUV data
	IMG_CODEC_422_PL21,		//!< 4:2:2 PL21 format YUV data
	/* !< 4:4:4 PL21 format YUV data (not currently supported by JPEG) */
	IMG_CODEC_444_PL21,

	PVR_SURF_UNSPECIFIED,		//!< End of the enum
	IMG_CODEC_FORMAT_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Enum describing presets for source image colour space conversion
 */
enum img_csc_preset {
	IMG_CSC_NONE,			//!< No colour space conversion
	IMG_CSC_709_TO_601,		//!< ITU BT.709 YUV to be converted to ITU BT.601 YUV
	IMG_CSC_601_TO_709,		//!< ITU BT.601 YUV to be converted to ITU BT.709 YUV
	IMG_CSC_RGB_TO_601_ANALOG,	//!< RGB to be converted to ITU BT.601 YUV
	/* !< RGB to be converted to ITU BT.601 YCbCr for SDTV (reduced scale - 16-235) */
	IMG_CSC_RGB_TO_601_DIGITAL,
	/*  !< RGB to be converted to ITU BT.601 YCbCr for HDTV (full range - 0-255) */
	IMG_CSC_RGB_TO_601_DIGITAL_FS,
	IMG_CSC_RGB_TO_709,		//!< RGB to be converted to ITU BT.709 YUV
	IMG_CSC_YIQ_TO_601,		//!< YIQ to be converted to ITU BT.601 YUV
	IMG_CSC_YIQ_TO_709,		//!< YIQ to be converted to ITU BT.709 YUV
	IMG_CSC_BRG_TO_601,		//!< BRG to be converted to ITU BT.601 YUV (for XRGB format)
	IMG_CSC_RBG_TO_601,		//!< RBG to be converted to ITU BT.601 YUV (for XBGR format)
	IMG_CSC_BGR_TO_601,     //!< BGR to be converted to ITU BT.601 YUV (for BGRX format)
	IMG_CSC_UYV_TO_YUV,     //!< UYV to be converted to YUV (BT.601 or BT.709)
	IMG_CSC_PRESETS,			//!< End of the enum
	IMG_CSC_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * GOP structure information
 */
#define SHIFT_GOP_FRAMETYPE		(0)
#define MASK_GOP_FRAMETYPE		(0x3 << SHIFT_GOP_FRAMETYPE)
#define SHIFT_GOP_REFERENCE		(2)
#define MASK_GOP_REFERENCE		(0x1 << SHIFT_GOP_REFERENCE)
#define SHIFT_GOP_POS			(3)
#define MASK_GOP_POS			(0x1f << SHIFT_GOP_POS)
#define SHIFT_GOP_REF0			(0 + 8)
#define MASK_GOP_REF0			(0xf << SHIFT_GOP_REF0)
#define SHIFT_GOP_REF1			(4 + 8)
#define MASK_GOP_REF1			(0xf << SHIFT_GOP_REF1)

/*
 * Frame types
 */
enum img_frame_type {
	IMG_INTRA_IDR = 0,
	IMG_INTRA_FRAME,
	IMG_INTER_P,
	IMG_INTER_B,
	IMG_INTER_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Motion vector calculation register settings
 */
struct img_mv_settings {
	unsigned int	mv_calc_below;
	unsigned int	mv_calc_colocated;
	unsigned int	mv_calc_config;
};

/*
 * Frame template types
 */
enum img_frame_template_type {
	IMG_FRAME_IDR = 0,
	IMG_FRAME_INTRA,
	IMG_FRAME_INTER_P,
	IMG_FRAME_INTER_B,
	IMG_FRAME_INTER_P_IDR,
	IMG_FRAME_UNDEFINED,
	IMG_FRAME_TYPE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Rate control modes
 */
enum img_rcmode {
	IMG_RCMODE_NONE = 0,
	IMG_RCMODE_CBR,
	IMG_RCMODE_VBR,
	IMG_RCMODE_ERC,			// Example Rate Control
	IMG_RCMODE_VCM,
	IMG_RCMODE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Video Conferencing Mode (VCM) rate control method's sub modes
 */
enum img_rc_vcm_mode {
	IMG_RC_VCM_MODE_DEFAULT = 0,
	IMG_RC_VCM_MODE_CFS_NONIFRAMES,
	IMG_RC_VCM_MODE_CFS_ALLFRAMES,
	IMG_RC_VCM_MODE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Weighted prediction values
 */
struct weighted_prediction_values {
	unsigned char	frame_type;
	unsigned char	weighted_pred_flag;	// Corresponds to field in the pps
	unsigned char	weighted_bipred_idc;
	unsigned int	luma_log2_weight_denom;
	unsigned int	chroma_log2_weight_denom;
	/*  Y, Cb, Cr Support for 2 ref pictures on P, or 1 pic in each direction on B. */
	unsigned char	weight_flag[3][2];
	int		weight[3][2];
	int		offset[3][2];
};

enum weighted_bipred_idc {
	WBI_NONE = 0x0,
	WBI_EXPLICIT,
	WBI_IMPLICIT,
	WBI_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Registers required to configure input scaler
 */
struct img_vxe_scaler_setup {
	unsigned int	input_scaler_control;
	unsigned int	scaler_input_size_reg;
	unsigned int	scaler_crop_reg;
	unsigned int	scaler_pitch_reg;
	unsigned int	scaler_control;
	unsigned int	hor_scaler_coeff_regs[4];
	unsigned int	ver_scaler_coeff_regs[4];
};

/*
 * Registers required to configure input Colour Space conversion
 */
struct img_vxe_csc_setup {
	unsigned int	csc_source_y[3];
	unsigned int	csc_output_clip[2];
	unsigned int	csc_source_cbcr[3];
};

/*
 * SETVIDEO & GETVIDEO - Video encode context
 */
struct img_mtx_video_context {
	/* // keep this at the top as it has alignment issues */
	unsigned long long	clock_div_bitrate;
	unsigned int	width_in_mbs;		//!< target output width
	unsigned int	picture_height_in_mbs;	//!< target output height
	unsigned int	tmp_reconstructed[MAX_PIC_NODES];
	unsigned int	reconstructed[MAX_PIC_NODES];
	unsigned int	colocated[MAX_PIC_NODES];
	unsigned int	mv[MAX_MV];
	unsigned int	inter_view_mv[2];
	/* !< Send debug information from Register CRCs to Host with the coded buffer */
	unsigned int	debug_crcs;
	unsigned int	writeback_regions[WB_FIFO_SIZE];		//!< Data section
	unsigned int	initial_cpb_removal_delayoffset;
	unsigned int	max_buffer_mult_clock_div_bitrate;
	unsigned int	sei_buffering_period_template;
	unsigned int	sei_picture_timing_template;
	unsigned short	enable_mvc;
	unsigned short	mvc_view_idx;
	unsigned int	slice_params_templates[5];
	unsigned int	pichdr_templates[4];
	unsigned int	seq_header;
	unsigned int	subset_seq_header;
	unsigned short	no_sequence_headers;

	/* !< Slice map of the source picture */
	unsigned int	slice_map[MAX_SOURCE_SLOTS_SL];
	unsigned int	flat_gop_struct;	//!< Address of Flat MiniGop structure
	unsigned char	weighted_prediction_enabled;
	unsigned char	mtx_weighted_implicit_bi_pred;
	unsigned int	weighted_prediction_virt_addr[MAX_SOURCE_SLOTS_SL];
	/* !< Address of hierarchical MiniGop structure */
	unsigned int	hierar_gop_struct;
	/* Output Parameters of the First Pass */
	unsigned int	firstpass_out_param_addr[MAX_SOURCE_SLOTS_SL];
	/* !< Selectable Output Best MV Parameters data of the First Pass */
	unsigned int	firstpass_out_best_multipass_param_addr[MAX_SOURCE_SLOTS_SL];
	/* !< Input Parameters to the second pass */
	unsigned int	mb_ctrl_in_params_addr[MAX_SOURCE_SLOTS_SL];
	/* !< Strides of source Y data and chroma data */
	unsigned int	pic_row_stride_bytes;
	/* !< Picture level parameters (supplied by driver) */
	unsigned int	above_params[TOPAZHP_MAX_NUM_PIPES];
	unsigned int	idr_period;
	unsigned int	intra_loop_cnt;
	unsigned int	bframe_count;
	unsigned char	hierarchical;
	/* !< Only used in MPEG2, 2 bit field (0 = 8 bit, 1 = 9 bit, 2 = 10 bit and 3=11 bit
	 * precision). Set to zero for other encode standards.
	 */
	unsigned char	mpeg2_intra_dc_precision;
	unsigned char	pic_on_level[MAX_REF_LEVELS];
	unsigned int	vop_time_resolution;
	unsigned short	kick_size;		//!< Number of Macroblocks per kick
	unsigned short	kicks_per_bu;		//!< Number of kicks per BU
	unsigned short	kicks_per_picture;	//!< Number of kicks per picture
	struct img_mv_settings mv_settings_idr;
	struct img_mv_settings mv_settings_non_b[MAX_BFRAMES + 1];
	unsigned int	mv_settings_b_table;
	unsigned int	mv_settings_hierarchical;
	enum img_format format;		//!< Pixel format of the source surface
	enum img_standard standard;	//!< Encoder standard (H264 / H263 / MPEG4 / JPEG)
	enum img_rcmode rc_mode;	//!< RC flavour
	enum img_rc_vcm_mode rc_vcm_mode;	//!< RC VCM flavour
	/* !< RC VCM maximum frame size percentage allowed to exceed in CFS */
	unsigned int	rc_cfs_max_margin_perc;
	unsigned char	first_pic;
	unsigned char	is_interlaced;
	unsigned char	top_field_first;
	unsigned char	arbitrary_so;
	unsigned char	output_reconstructed;
	unsigned char	disable_bit_stuffing;
	unsigned char	insert_hrd_params;
	unsigned char	max_slices_per_picture;
	unsigned int	f_code;
	/* Contents Adaptive Rate Control parameters*/
	unsigned int	jmcomp_rc_reg0;
	unsigned int	jmcomp_rc_reg1;
	/* !< Value to use for MVClip_Config  register */
	unsigned int	mv_clip_config;
	/* !< Value to use for Predictor combiner register */
	unsigned int	pred_comb_control;
	/*  !< Value to use for LRITC_Cache_Chunk_Config register */
	unsigned int	lritc_cache_chunk_config;
	/* !< Value to use for IPEVectorClipping register */
	unsigned int	ipe_vector_clipping;
	/* !< Value to use for H264CompControl register */
	unsigned int	h264_comp_control;
	/* !< Value to use for H264CompIntraPredMode register */
	unsigned int	h264_comp_intra_pred_modes;
	/* !< Value to use for IPCM_0 Config register */
	unsigned int	ipcm_0_config;
	/* !< Value to use for IPCM_1 Config register */
	unsigned int	ipcm_1_config;
	/* !< Value to use for SPEMvdClipRange register */
	unsigned int	spe_mvd_clip_range;
	/* !< Value to use for MB_HOST_CONTROL register */
	unsigned int	mb_host_ctrl;
	/* !< Value for the CR_DB_DISABLE_DEBLOCK_IDC register */
	unsigned int	deblock_ctrl;
	/* !< Value for the CR_DB_DISABLE_DEBLOCK_IDC register */
	unsigned int	skip_coded_inter_intra;
	unsigned int	vlc_control;
	/* !< Slice control register value. Configures the size of a slice */
	unsigned int	vlc_slice_control;
	/* !< Slice control register value. Configures the size of a slice */
	unsigned int	vlc_slice_mb_control;
	/* !< Chroma QP offset to use (when PPS id = 0)*/
	unsigned short	cqp_offset;
	unsigned char	coded_header_per_slice;
	unsigned char	initial_qp_i;			//!< Initial QP I frames
	unsigned char	initial_qp_p;			//!< Initial QP P frames
	unsigned char	initial_qp_b;			//!< Initial QP B frames
	unsigned int	first_pic_flags;
	unsigned int	non_first_pic_flags;
	unsigned char	mc_adaptive_rounding_disable;
#define AR_REG_SIZE	18
#define AR_DELTA_SIZE 7
	unsigned short	mc_adaptive_rounding_offsets[AR_REG_SIZE][4];
	short		mc_adaptive_rounding_offsets_delta[AR_DELTA_SIZE][4];
	/* !< Reconstructed address to allow host picture management */
	unsigned int	patched_recon_address;
	/* !< Reference 0 address to allow host picture management */
	unsigned int	patched_ref0_address;
	/* !< Reference 1 address to allow host picture management */
	unsigned int	patched_ref1_address;
	unsigned int	ltref_header[MAX_SOURCE_SLOTS_SL];
	signed char	slice_header_slot_num;
	unsigned char	recon_is_longterm;
	unsigned char	ref0_is_longterm;
	unsigned char	ref1_is_longterm;
	unsigned char	ref_spacing;
	unsigned char	fw_num_pipes;
	unsigned char	fw_first_pipe;
	unsigned char	fw_last_pipe;
	unsigned char	fw_pipes_to_use_flags;
#if SECURE_IO_PORTS
	unsigned int	secure_io_control;
#endif
	struct img_vxe_scaler_setup scaler_setup;
	struct img_vxe_csc_setup csc_setup;

	struct in_rc_params in_params;
};

/*
 * PICMGMT - Command sub-type
 */
enum img_picmgmt_type {
	IMG_PICMGMT_REF_TYPE = 0,
	IMG_PICMGMT_GOP_STRUCT,
	IMG_PICMGMT_SKIP_FRAME,
	IMG_PICMGMT_EOS,
	IMG_PICMGMT_FLUSH,
	IMG_PICMGMT_QUANT,
	IMG_PICMGMT_STRIDE,
	IMG_PICMGMT_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * MTX- > host message structure
 */
struct img_writeback_msg {
	unsigned int cmd_word;
	union {
		struct {
			unsigned int	data;
			unsigned int	extra_data;
			unsigned int	writeback_val;
		};
		unsigned int	coded_package_consumed_idx;
	};
};

/*
 * PROVIDE_SOURCE_BUFFER - Details of the source picture buffer
 */
struct img_source_buffer_params {
	/* !< Host context value. Keep at start for alignment. */
	unsigned long long	host_context;
	unsigned int	phys_addr_y_plane_field_0;	//!< Source pic phys addr (Y plane, Field 0)
	unsigned int	phys_addr_u_plane_field_0;	//!< Source pic phys addr (U plane, Field 0)
	unsigned int	phys_addr_v_plane_field_0;	//!< Source pic phys addr (V plane, Field 0)
	unsigned int	phys_addr_y_plane_field_1;	//!< Source pic phys addr (Y plane, Field 1)
	unsigned int	phys_addr_u_plane_field_1;	//!< Source pic phys addr (U plane, Field 1)
	unsigned int	phys_addr_v_plane_field_1;	//!< Source pic phys addr (V plane, Field 1)
	/* !< Number of frames in the stream (incl. skipped) */
	unsigned char	display_order_num;
	unsigned char	slot_num;					//!< Source slot number
	unsigned char   reserved1;
	unsigned char   reserved2;
};

/*
 * Struct describing input parameters to encode a video slice
 */
struct slice_params {
	unsigned int	flags;					//!< Flags for slice encode

	/* Config registers. These are passed straight
	 *  through from drivers to hardware.
	 */
	unsigned int	slice_config;			//!< Value to use for Slice Config register
	unsigned int	ipe_control;			//!< Value to use for IPEControl register
	/* !<Value to use for Sequencer Config register */
	unsigned int	seq_config;

	enum img_frame_template_type template_type;	//!< Slice header template type
	/* !< Template of corresponding slice header */
	struct mtx_header_params slice_hdr_tmpl;
};

/*
 * Structure describing coded header data returned by the firmware.
 * The size of the structure should not be more than 64 bytes (needs to have 64 byte alignment)
 *              (i.e. CODED_BUFFER_HEADER_SIZE)
 */
struct coded_data_hdr {
	unsigned long long	host_ctx;	//!< Host context value. Keep at top for alignment.
	unsigned int	bytes_written;	//!< Bytes in this coded buffer excluding this header
	unsigned int	feedback;		//!< Feedback word for this coded buffers
	unsigned int	extra_feedback;	//!< Extra feedback word for this coded buffers

	unsigned short	i_mb_cnt;	//!< Number of MBs coded as I-macroblocks in this slice
	unsigned short	p_mb_cnt;	//!< Number of MBs coded as P-macroblocks in this slice

	unsigned short	b_mb_cnt;	//!< Number of MBs coded as B-macroblocks in this slice
	unsigned short	skip_mb_cnt;	//!< Number of MBs coded as skipped in this slice

	unsigned short	ipcm_mb_cnt;	//!< Number of macroblocks coded as IPCM in this slice
	unsigned char	inter_sum_satd_hi;	//!< High 8 bits for the inter sum satd
	unsigned char	intra_sum_satd_hi;	//!< High 8 bits for the intra sum satd
	/* !< Number of bits use for coding DC coefficients in this slice  */
	unsigned int	dc_bits;
	/* !< Number of bits used for coding all Motion vector data in this slice  */
	unsigned int	mv_bits;
	/* !< Number of bits used for coding all MB level symbols in this slice */
	unsigned int	symbols_bits;
	/* !< Number of bits used for coding residual data in all MBs in this slice  */
	unsigned int	residual_bits;

	/* !< Sum of QPy/Qscale for all Inter-MBs in the slice */
	unsigned int	qpy_inter;
	/* !< Sum of QPy/Qscale for all Intra-MBs in the slice */
	unsigned int	qpy_intra;
	unsigned int	inter_sum_satd;	//!< Sum of SATD for all Inter-MBs in the slice
	unsigned int	intra_sum_satd;	//!< Sum of SATD for all Intra-MBs in the slice
};

#define MAX_CODED_BUFFERS_PER_PACKAGE_FW 1
#define MAX_CODED_BUFFERS_PER_PACKAGE 1

// This structure is temporarily used during the 64 byte minimum DMA transfer from driver to FW
struct coded_package_dma_info {
	unsigned int coded_mem_addr[MAX_CODED_BUFFERS_PER_PACKAGE_FW];
	//////////////////
	// 2 Info words //
	//////////////////
	unsigned int coded_header_addr;
	/* Combined field Host->MTX = IsLinkedList, list segment
	 * (CB memory) size, number of list segments per coded buffer
	 */
	unsigned int coded_buffer_info;

	// PAD TO 64 BYTES
	unsigned int padding[16 - MAX_CODED_BUFFERS_PER_PACKAGE_FW - 2];
};

/*
 * Contents of the coded data buffer header feedback word
 */
#define SHIFT_CODED_FIRST_BU			(24)
#define MASK_CODED_FIRST_BU			(0xFFU << SHIFT_CODED_FIRST_BU)
#define SHIFT_CODED_SLICE_NUM			(16)
#define MASK_CODED_SLICE_NUM			(0xFF << SHIFT_CODED_SLICE_NUM)
#define SHIFT_CODED_STORAGE_FRAME_NUM		(14)
#define MASK_CODED_STORAGE_FRAME_NUM		(0x03 << SHIFT_CODED_STORAGE_FRAME_NUM)
#define SHIFT_CODED_ENTIRE_FRAME		(12)
#define MASK_CODED_ENTIRE_FRAME			(0x01 << SHIFT_CODED_ENTIRE_FRAME)
#define SHIFT_CODED_IS_SKIPPED			(11)
#define MASK_CODED_IS_SKIPPED			(0x01 << SHIFT_CODED_IS_SKIPPED)
#define SHIFT_CODED_IS_CODED			(10)
#define MASK_CODED_IS_CODED			(0x01 << SHIFT_CODED_IS_CODED)
#define SHIFT_CODED_RECON_IDX			(6)
#define MASK_CODED_RECON_IDX			(0x0F << SHIFT_CODED_RECON_IDX)
#define SHIFT_CODED_SOURCE_SLOT			(2)
#define MASK_CODED_SOURCE_SLOT			(0x0F << SHIFT_CODED_SOURCE_SLOT)
#define SHIFT_CODED_FRAME_TYPE			(0)
#define MASK_CODED_FRAME_TYPE			(0x03 << SHIFT_CODED_FRAME_TYPE)

/*
 * Contents of the coded data buffer header extra feedback word
 */
#define SHIFT_CODED_SLICES_SO_FAR		(24)
#define MASK_CODED_SLICES_SO_FAR		(0xFFU << SHIFT_CODED_SLICES_SO_FAR)

#define SHIFT_CODED_SLICES_IN_BUFFER		(16)
#define MASK_CODED_SLICES_IN_BUFFER		(0xFF << SHIFT_CODED_SLICES_IN_BUFFER)

#define SHIFT_CODED_BUFFER_NUMBER_USED		(2)
#define MASK_CODED_BUFFER_NUMBER_USED		(0xFF << SHIFT_CODED_BUFFER_NUMBER_USED)

#define SHIFT_CODED_FIELD			(1)
#define MASK_CODED_FIELD			(0x01 << SHIFT_CODED_FIELD)

#define SHIFT_CODED_PATCHED_RECON		(0)
#define MASK_CODED_PATCHED_RECON		(0x01 << SHIFT_CODED_PATCHED_RECON)

#endif /* _TOPAZSCFWIF_H_ */
