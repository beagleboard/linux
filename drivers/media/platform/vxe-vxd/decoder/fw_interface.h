/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IMG MSVDX core Registers
 * This file contains the MSVDX_CORE_REGS_H Definitions
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

#ifndef FW_INTERFACE_H_
#define FW_INTERFACE_H_

/* TODO For now this macro defined, need to think and enable */
#define VDEC_USE_PVDEC_COMPATIBILITY    1

#define MSG_TYPE_PADDING                                (0x00)
/* Start of parser specific Host->MTX messages */
#define MSG_TYPE_START_PSR_HOSTMTX_MSG  (0x80)
/* Start of parser specific MTX->Host message */
#define MSG_TYPE_START_PSR_MTXHOST_MSG  (0xC0)

enum {
	FW_DEVA_INIT = MSG_TYPE_START_PSR_HOSTMTX_MSG,
	FW_DEVA_DECODE_FE,
	FW_DEVA_RES_0,
	FW_DEVA_RES_1,
	FW_DEVA_DECODE_BE,
	FW_DEVA_HOST_BE_OPP,
	FW_DEVA_DEBLOCK,
	FW_DEVA_INTRA_OOLD,
	FW_DEVA_ENDFRAME,

	FW_DEVA_PARSE,
	FW_DEVA_PARSE_FRAGMENT,
	FW_DEVA_BEGINFRAME,

#ifdef VDEC_USE_PVDEC_COMPATIBILITY
#ifdef VDEC_USE_PVDEC_SEC
	FWBSP_INIT,
	FWBSP_PARSE_BITSTREAM,
	FWDEC_DECODE,
#endif /* VDEC_USE_PVDEC_SEC */
#endif /* VDEC_USE_PVDEC_COMPATIBILITY */

	/* Sent by the firmware on the MTX to the host. */
	FW_DEVA_COMPLETED = MSG_TYPE_START_PSR_MTXHOST_MSG,
#ifndef VDEC_USE_PVDEC_COMPATIBILITY
	FW_DEVA_RES_2,
	FW_DEVA_RES_3,
	FW_DEVA_RES_4,
	FW_DEVA_RES_5,

	FW_DEVA_RES_6,
	FW_DEVA_CONTIGUITY_WARNING,
	FW_DEVA_PANIC,
	FW_DEVA_RES_7,
	FW_DEVA_RES_8,
#else   /* ndef VDEC_USE_PVDEC_COMPATIBILITY */
	FW_DEVA_PANIC,
	FW_ASSERT,
	FW_PERF,
	/* An empty completion message sent by new vxd driver */
	FW_VXD_EMPTY_COMPL,
	FW_DEC_REQ_RECEIVED,
	FW_SO,
#ifdef VDEC_USE_PVDEC_SEC
	FWBSP_NEW_SEQ,
	FWBSP_NEW_PIC,
	FWBSP_BUF_EMPTY,
	FWBSP_ERROR,
	FWDEC_COMPLETED,
#endif /* VDEC_USE_PVDEC_SEC */
#endif /* VDEC_USE_PVDEC_COMPATIBILITY */
	FW_DEVA_SIGNATURES_LEGACY      = 0xD0,
	FW_DEVA_SIGNATURES_HEVC        = 0xE0,
	FW_DEVA_SIGNATURES_FORCE32BITS = 0x7FFFFFFFU
};

/* Defines the Host/Firmware communication area */
#ifndef VDEC_USE_PVDEC_COMPATIBILITY
#define COMMS_HEADER_SIZE               (0x34)
#else /* def VDEC_USE_PVDEC_COMPATIBILITY */
#define COMMS_HEADER_SIZE       (0x40)
#endif /* def VDEC_USE_PVDEC_COMPATIBILITY */
/* dwords */
#define PVDEC_COM_RAM_FW_STATUS_OFFSET                          0x00
#define PVDEC_COM_RAM_TASK_STATUS_OFFSET                        0x04
#define PVDEC_COM_RAM_FW_ID_OFFSET                              0x08
#define PVDEC_COM_RAM_FW_MTXPC_OFFSET                           0x0c
#define PVDEC_COM_RAM_MSG_COUNTER_OFFSET                        0x10
#define PVDEC_COM_RAM_SIGNATURE_OFFSET                          0x14
#define PVDEC_COM_RAM_TO_HOST_BUF_SIZE_AND_OFFSET_OFFSET        0x18
#define PVDEC_COM_RAM_TO_HOST_RD_INDEX_OFFSET                   0x1c
#define PVDEC_COM_RAM_TO_HOST_WRT_INDEX_OFFSET                  0x20
#define PVDEC_COM_RAM_TO_MTX_BUF_SIZE_AND_OFFSET_OFFSET         0x24
#define PVDEC_COM_RAM_TO_MTX_RD_INDEX_OFFSET                    0x28
#define PVDEC_COM_RAM_FLAGS_OFFSET                              0x2c
#define PVDEC_COM_RAM_TO_MTX_WRT_INDEX_OFFSET                   0x30
#ifdef VDEC_USE_PVDEC_COMPATIBILITY
#define PVDEC_COM_RAM_STATE_BUF_SIZE_AND_OFFSET_OFFSET          0x34
#define PVDEC_COM_RAM_FW_MMU_REPORT_OFFSET                      0x38
#endif /* VDEC_USE_PVDEC_COMPATIBILITY */
/* fields */
#define PVDEC_COM_RAM_TO_HOST_BUF_SIZE_AND_OFFSET_SIZE_MASK     0xFFFF
#define PVDEC_COM_RAM_TO_HOST_BUF_SIZE_AND_OFFSET_SIZE_SHIFT    0
#define PVDEC_COM_RAM_TO_HOST_BUF_SIZE_AND_OFFSET_OFFSET_MASK   0xFFFF0000
#define PVDEC_COM_RAM_TO_HOST_BUF_SIZE_AND_OFFSET_OFFSET_SHIFT  16

#define PVDEC_COM_RAM_TO_MTX_BUF_SIZE_AND_OFFSET_SIZE_MASK      0xFFFF
#define PVDEC_COM_RAM_TO_MTX_BUF_SIZE_AND_OFFSET_SIZE_SHIFT     0
#define PVDEC_COM_RAM_TO_MTX_BUF_SIZE_AND_OFFSET_OFFSET_MASK    0xFFFF0000
#define PVDEC_COM_RAM_TO_MTX_BUF_SIZE_AND_OFFSET_OFFSET_SHIFT   16
#ifdef VDEC_USE_PVDEC_COMPATIBILITY
#define PVDEC_COM_RAM_STATE_BUF_SIZE_AND_OFFSET_SIZE_MASK       0xFFFF
#define PVDEC_COM_RAM_STATE_BUF_SIZE_AND_OFFSET_SIZE_SHIFT      0
#define PVDEC_COM_RAM_STATE_BUF_SIZE_AND_OFFSET_OFFSET_MASK     0xFFFF0000
#define PVDEC_COM_RAM_STATE_BUF_SIZE_AND_OFFSET_OFFSET_SHIFT    16
#endif /* VDEC_USE_PVDEC_COMPATIBILITY */
#define PVDEC_COM_RAM_BUF_GET_SIZE(_reg_, _name_) \
	(((_reg_) & PVDEC_COM_RAM_ ## _name_ ## _BUF_SIZE_AND_OFFSET_SIZE_MASK) >> \
	 PVDEC_COM_RAM_ ## _name_ ## _BUF_SIZE_AND_OFFSET_SIZE_SHIFT)
#define PVDEC_COM_RAM_BUF_GET_OFFSET(_reg_, _name_) \
	(((_reg_) & \
	  PVDEC_COM_RAM_ ## _name_ ## _BUF_SIZE_AND_OFFSET_OFFSET_MASK) >> \
	 PVDEC_COM_RAM_ ## _name_ ## _BUF_SIZE_AND_OFFSET_OFFSET_SHIFT)
#define PVDEC_COM_RAM_BUF_SET_SIZE_AND_OFFSET(_name_, _size_, _offset_) \
	((((_size_) << \
		PVDEC_COM_RAM_ ## _name_ ## _BUF_SIZE_AND_OFFSET_SIZE_SHIFT) \
	  & PVDEC_COM_RAM_ ## _name_ ## _BUF_SIZE_AND_OFFSET_SIZE_MASK) | \
	 (((_offset_) << \
		PVDEC_COM_RAM_ ## _name_ ## _BUF_SIZE_AND_OFFSET_OFFSET_SHIFT) \
	  & PVDEC_COM_RAM_ ## _name_ ## _BUF_SIZE_AND_OFFSET_OFFSET_MASK))
/* values */
/* Firmware ready signature value */
	#define FW_READY_SIGNATURE                              (0xA5A5A5A5)

/* Firmware status values */
	#define FW_STATUS_BUSY                                  0
	#define FW_STATUS_IDLE                                  1
	#define FW_STATUS_PANIC                                 2
	#define FW_STATUS_ASSERT                                3
	#define FW_STATUS_GAMEOVER                              4
	#define FW_STATUS_FEWATCHDOG                            5
	#define FW_STATUS_EPWATCHDOG                            6
	#define FW_STATUS_BEWATCHDOG                            7
#ifdef VDEC_USE_PVDEC_COMPATIBILITY
	#define FW_STATUS_SO                                    8
	#define FW_STATUS_INIT                                  0xF
#endif

/* Decode Message Flags */
	#define FW_DEVA_RENDER_IS_FIRST_SLICE                   (0x00000001)
/* This is H264 Mbaff - required for state store */
	#define FW_DEVA_FORCE_RECON_WRITE_DISABLE               (0x00000002)
	#define FW_DEVA_RENDER_IS_LAST_SLICE                    (0x00000004)
/* Prevents insertion of end of picture or flush at VEC EOS */
	#define FW_DEVA_DECODE_DISABLE_EOF_DETECTION            (0x00000008)

	#define FW_DEVA_CONTEXT_BUFFER_INVALID                  (0x00000010)
	#define FW_DEVA_FORCE_ALT_OUTPUT                        (0x00000020)
	#define FW_SECURE_STREAM                                (0x00000040)
	#define FW_LOW_LATENCY                                  (0x00000080)

	#define FW_DEVA_CONTIGUITY_DETECTION                    (0x00000100)
	#define FW_DEVA_FORCE_INIT_CMDS                         (0x00000200)
	#define FW_DEVA_DEBLOCK_ENABLE                          (0x00000400)
#ifdef VDEC_USE_PVDEC_COMPATIBILITY
	#define FW_VDEC_SEND_SIGNATURES                         (0x00000800)
#else
/*							(0x00000800) */
#endif /* VDEC_USE_PVDEC_COMPATIBILITY */

	#define FW_DEVA_FORCE_AUX_LINE_BUF_DISABLE              (0x00001000)
/*
 * Cause no response message to be sent, and no interrupt
 * generation on successful completion
 */
	#define FW_DEVA_RENDER_NO_RESPONSE_MSG                  (0x00002000)
/*
 * Cause an interrupt if a response message is generated
 * on successful completion
 */
	#define FW_DEVA_RENDER_HOST_INT                         (0x00004000)
/* Report contiguity errors to host */
	#define FW_DEVA_CONTIGUITY_REPORTING                    (0x00008000)

	#define FW_DEVA_VC1_SKIPPED_PICTURE                     (0x00010000)
	#define FW_INTERNAL_RENDER_SWITCH                       (0x00020000)
	#define FW_DEVA_UNSUPPORTED                             (0x00040000)
	#define DEBLOCKING_FORCED_OFF                           (0x00080000)
#ifdef VDEC_USE_PVDEC_COMPATIBILITY
	#define FW_VDEC_CMD_PENDING                             (0x00100000)
#else
/*							(0x00100000) */
#endif
/* Only for debug */
	#define DETECTED_RENDEC_FULL                            (0x00200000)
/* Only for debug */
	#define DETECTED_RENDEC_EMPTY                           (0x00400000)
	#define FW_ONE_PASS_PARSE                               (0x00800000)

	#define FW_DEVA_EARLY_COMPLETE                          (0x01000000)
	#define FW_DEVA_FE_EP_SIGNATURES_READY                  (0x02000000)
	#define FW_VEC_EOS                                      (0x04000000)
/* hardware has reported an error relating to this command */
	#define FW_DEVA_ERROR_DETECTED_ENT                      (0x08000000)

	#define FW_DEVA_ERROR_DETECTED_PIX                      (0x10000000)
	#define FW_DEVA_MP_SYNC                                 (0x20000000)
	#define MORE_THAN_ONE_MB                                (0x40000000)
	#define REATTEMPT_SINGLEPIPE                            (0x80000000)
/* end of message flags */
#ifdef VDEC_USE_PVDEC_COMPATIBILITY
/* VDEC Decode Message Flags */
/*
 * H.264/H.265 are to be configured in SIZE_DELIMITED mode rather than SCP mode.
 */
#define FW_VDEC_NAL_SIZE_DELIM                  (0x00000001)
/* Indicates if MMU cache shall be flushed. */
#define FW_VDEC_MMU_FLUSH_CACHE                 (0x00000002)
/* end of message flags */
#endif /* VDEC_USE_PVDEC_COMPATIBILITY */

/* FW flags */
/* TODO : Temporary for HW testing */
	#define FWFLAG_DISABLE_VDEB_PRELOAD             (0x00000001)
	#define FWFLAG_BIG_TO_HOST_BUFFER               (0x00000002)
/* FS is default regarless of this flag */
	#define FWFLAG_FORCE_FS_FLOW                    (0x00000004)
	#define FWFLAG_DISABLE_WATCHDOG_TIMERS          (0x00000008)

	#define FWFLAG_DISABLE_AEH                      (0x00000020)
	#define FWFLAG_DISABLE_AUTONOMOUS_RESET         (0x00000040)
	#define FWFLAG_NON_ACCUMULATING_HWSIGS          (0x00000080)

	#define FWFLAG_DISABLE_2PASS_DEBLOCK            (0x00000100)
	#define FWFLAG_NO_INT_ON_TOHOST_FULL            (0x00000200)
	#define FWFLAG_RETURN_VDEB_CR                   (0x00000800)

	#define FWFLAG_DISABLE_AUTOCLOCKGATING          (0x00001000)
	#define FWFLAG_DISABLE_IDLE_GPIO                (0x00002000)
	#define FWFLAG_XPL                              (0x00004000)
	#define FWFLAG_INFINITE_MTX_TIMEOUT             (0x00008000)

	#define FWFLAG_DECOUPLE_BE_FE                   (0x00010000)
	#define FWFLAG_ENABLE_SECURITY                  (0x00080000)

	#define FWFLAG_ENABLE_CONCEALMENT               (0x00100000)
/* Not currently supported */
/*	#define	FWFLAG_PREEMPT				(0x00200000) */
/* NA in FS */
	#define FWFLAG_FORCE_FLUSHING                   (0x00400000)
/* NA in FS */
	#define FWFLAG_DISABLE_GENC_FLUSHING            (0x00800000)

	#define FWFLAG_DISABLE_COREWDT_TIMERS           (0x01000000)
	#define FWFLAG_DISABLE_RENDEC_AUTOFLUSH         (0x02000000)
	#define FWFLAG_FORCE_STRICT_SINGLEPIPE          (0x04000000)
	#define FWFLAG_CONSISTENT_MULTIPIPE_FLOW        (0x08000000)

	#define FWFLAG_DISABLE_IDLE_FAST_EVAL           (0x10000000)
	#define FWFLAG_FAKE_COMPLETION                  (0x20000000)
	#define FWFLAG_MAN_PP_CLK                       (0x40000000)
	#define FWFLAG_STACK_CHK                        (0x80000000)

/* end of FW flags */

#ifdef FW_STACK_USAGE_TRACKING
/* FW task identifiers */
enum task_id {
	TASK_ID_RX = 0,
	TASK_ID_TX,
	TASK_ID_EP1,
	TASK_ID_FE1,
	TASK_ID_FE2,
	TASK_ID_FE3,
	TASK_ID_BE1,
	TASK_ID_BE2,
	TASK_ID_BE3,
	TASK_ID_PARSER,
	TASK_ID_MAX,
	TASK_ID_FORCE32BITS = 0x7FFFFFFFU
};

/* FW task stack info utility macros */
#define TASK_STACK_SIZE_MASK   0xFFFF
#define TASK_STACK_SIZE_SHIFT  0
#define TASK_STACK_USED_MASK   0xFFFF0000
#define TASK_STACK_USED_SHIFT  16
#define TASK_STACK_SET_INFO(_task_id_, _stack_info_, _size_, _used_) \
	(_stack_info_[_task_id_] = \
		 ((_size_) << TASK_STACK_SIZE_SHIFT) | \
		 ((_used_) << TASK_STACK_USED_SHIFT))
#define TASK_STACK_GET_SIZE(_task_id_, _stack_info_) \
	((_stack_info_[_task_id_] & TASK_STACK_SIZE_MASK) >> \
	 TASK_STACK_SIZE_SHIFT)
#define TASK_STACK_GET_USED(_task_id_, _stack_info_) \
	((_stack_info_[_task_id_] & TASK_STACK_USED_MASK) >> \
	 TASK_STACK_USED_SHIFT)
#endif /* FW_STACK_USAGE_TRACKING */

/* Control Allocation */
#define CMD_MASK                                (0xF0000000)

/* Ctrl Allocation Header */
#define CMD_CTRL_ALLOC_HEADER                   (0x90000000)

struct ctrl_alloc_header {
	unsigned int cmd_additional_params;
	unsigned int slice_params;
	union {
		unsigned int vp8_probability_data;
		unsigned int h264_pipeintra_buffersize;
	};
	unsigned int chroma_strides;
	unsigned int slice_first_mb_yx;
	unsigned int pic_last_mb_yx;
	/* VC1 only : Store Range Map flags in bottom bits of [0] */
	unsigned int alt_output_addr[2];
	unsigned int alt_output_flags;
	/* H264 Only : Extended Operating Mode */
	unsigned int ext_opmode;
};

#define CMD_CTRL_ALLOC_HEADER_DWSIZE \
	(sizeof(struct ctrl_alloc_header) / sizeof(unsigned int))

/* Additional Parameter flags */
#define VC1_PARSEHDR_MASK               (0x00000001)
#define VC1_SKIPPIC_MASK                (0x00000002)

#define VP6_BUFFOFFSET_MASK             (0x0000ffff)
#define VP6_MULTISTREAM_MASK            (0x01000000)
#define VP6_FRAMETYPE_MASK              (0x02000000)

#define VP8_BUFFOFFSET_MASK             (0x00ffffff)
#define VP8_PARTITIONSCOUNT_MASK        (0x0f000000)
#define VP8_PARTITIONSCOUNT_SHIFT       (24)

/* Nop Command */
#define CMD_NOP                         (0x00000000)
#define CMD_NOP_DWSIZE                  (1)

/* Register Block */
#define CMD_REGISTER_BLOCK                      (0x10000000)
#define CMD_REGISTER_BLOCK_PATCHING_REQUIRED    (0x01000000)
#define CMD_REGISTER_BLOCK_FLAG_PRELOAD         (0x04000000)
#define CMD_REGISTER_BLOCK_FLAG_VLC_DATA        (0x08000000)

/* Rendec Command */
#define CMD_RENDEC_BLOCK                        (0x50000000)
#define CMD_RENDEC_BLOCK_FLAG_MASK              (0x0F000000)
#define CMD_RENDEC_FORCE                        (0x08000000)
#define CMD_RENDEC_PATCHING_REQUIRED            (0x01000000)
#define CMD_RENDEC_WORD_COUNT_MASK              (0x00ff0000)
#define CMD_RENDEC_WORD_COUNT_SHIFT             (16)
#define CMD_RENDEC_ADDRESS_MASK                 (0x0000ffff)
#define CMD_RENDEC_ADDRESS_SHIFT                (0)

#ifndef VDEC_USE_PVDEC_SEC
/* Deblock */
#define CMD_DEBLOCK                             (0x70000000)
#define CMD_DEBLOCK_TYPE_STD                    (0x00000000)
#define CMD_DEBLOCK_TYPE_OOLD                   (0x00000001)
#define CMD_DEBLOCK_TYPE_SKIP                   (0x00000002)
/* End Of Frame */
#define CMD_DEBLOCK_TYPE_EF                     (0x00000003)

struct deblock_cmd {
	unsigned int cmd; /* 0x70000000 */
	unsigned int source_mb_data;
	unsigned int address_a[2];
};

#define CMD_DEBLOCK_DWSIZE              (sizeof(DEBLOCK_CMD) / sizeof(unsigned int))
#endif /* !VDEC_USE_PVDEC_SEC */

/* Skip */
#define CMD_CONDITIONAL_SKIP                    (0x80000000)
#define CMD_CONDITIONAL_SKIP_DWSIZE             (1)
#define CMD_CONDITIONAL_SKIP_DWORDS             (0x0000ffff)
#define CMD_CONDITIONAL_SKIP_CONTEXT_SWITCH     BIT(20)

/* DMA */
#define CMD_DMA                                 (0xE0000000)
#define CMD_DMA_DMA_TYPE_MASK                   (0x03000000)
#define CMD_DMA_DMA_TYPE_SHIFT                  (24)
#define CMD_DMA_FLAG_MASK                       (0x00100000)
#define CMD_DMA_FLAG_SHIFT                      (20)
#define CMD_DMA_DMA_SIZE_MASK                   (0x000fffff)

#define CMD_DMA_OFFSET_FLAG                     (0x00100000)

#define CMD_DMA_MAX_OFFSET                      (0xFFF)
#define CMD_DMA_TYPE_VLC_TABLE                  (0 << CMD_DMA_DMA_TYPE_SHIFT)
#define CMD_DMA_TYPE_PROBABILITY_DATA           BIT(CMD_DMA_DMA_TYPE_SHIFT)

struct dma_cmd {
	unsigned int cmd;
	unsigned int dev_virt_add;
};

#define CMD_DMA_DWSIZE                          (sizeof(DMA_CMD) / sizeof(unsigned int))

struct dma_cmd_offset_dwsize {
	unsigned int cmd;
	unsigned int dev_virt_add;
	unsigned int byte_offset;
};

#define CMD_DMA_OFFSET_DWSIZE   (sizeof(DMA_CMD_WITH_OFFSET) / sizeof(unsigned int))

/* HOST COPY */
#define CMD_HOST_COPY                           (0xF0000000)
#define CMD_HOST_COPY_SIZE_MASK                 (0x000fffff)

struct host_copy_cmd {
	unsigned int cmd;
	unsigned int src_dev_virt_add;
	unsigned int dst_dev_virt_add;
};

#define CMD_HOST_COPY_DWSIZE            (sizeof(HOST_COPY_CMD) / sizeof(unsigned int))

/* Shift register setup and Bitstream DMA */
#define CMD_SR_SETUP                            (0xB0000000)
#define CMD_SR_ENABLE_RBDU_EXTRACTION           (0x00000001)
#define CMD_SR_ENABLE_AES_COUNTER               (0x00000002)
#define CMD_SR_VERIFY_STARTCODE                 (0x00000004)
#define CMD_SR_BITSTR_ADDR_DEREF                (0x00000008)
#define CMD_SR_BITSTR_PARSE_KEY                 (0x00000010)

struct sr_setup_cmd {
	unsigned int cmd;
	unsigned int bitstream_offset_bits;
	unsigned int bitstream_size_bytes;
};

#define CMD_SR_DWSIZE                   (sizeof(SR_SETUP_CMD) / sizeof(unsigned int))

#define CMD_BITSTREAM_DMA                       (0xA0000000)
#define CMD_BITSTREAM_DMA_DWSIZE                (2)
/* VC1 Parse Header Command */
#define CMD_PARSE_HEADER                        (0x30000000)
#define CMD_PARSE_HEADER_CONTEXT_MASK           (0x000000ff)
#define CMD_PARSE_HEADER_NEWSLICE               (0x00000001)
#define CMD_PARSE_HEADER_SKIP_PIC               (0x00000002)
#define CMD_PARSE_HEADER_ONEPASSPARSE           (0x00000004)
#define CMD_PARSE_HEADER_NUMSLICE_MINUS1        (0x00ffff00)

struct parse_header_cmd {
	unsigned int cmd;
	unsigned int seq_hdr_data;
	unsigned int pic_dimensions;
	unsigned int bitplane_addr[3];
	unsigned int vlc_table_addr;
};

#define CMD_PARSE_DWSIZE                (sizeof(PARSE_HEADER_CMD) / sizeof(unsigned int))

#define CMD_SLICE_INFO                          (0x20000000)
#define CMD_SLICE_INFO_SLICENUM                 (0xff000000)
#define CMD_SLICE_INFO_FIRSTMBY                 (0x00ff0000)
#define CMD_SLICE_INFO_MBBITOFFSET              (0x0000ffff)

struct slice_info {
	unsigned char slice_num;
	unsigned char slice_first_mby;
	unsigned short slice_mb_bitoffset;
};

#ifdef VDEC_USE_PVDEC_COMPATIBILITY
/* VDEC extension */
#define CMD_VDEC_EXT                            (0xC0000000)
#ifdef VDEC_USE_PVDEC_SEC
/*
 * Used only between firmware secure modules FWBSP->FWDEC,
 * thus the structure is defined in firmware structures.h
 */
#define CMD_VDEC_SECURE_EXT                     (0x40000000)
#endif/* VDEC_USE_PVDEC_SEC */

#define MEM2REG_SIZE_HOST_PART_MASK 0x0000FFFF
#define MEM2REG_SIZE_HOST_PART_SHIFT 0

#define MEM2REG_SIZE_BUF_TOTAL_MASK 0xFFFF0000
#define MEM2REG_SIZE_BUF_TOTAL_SHIFT 16

struct vdec_ext_cmd {
	unsigned int cmd;
	unsigned int trans_id;
	unsigned int hdr_addr;
	unsigned int hdr_size;
	unsigned int ctx_save_addr;
	unsigned int ctx_load_addr;
	unsigned int buf_ctrl_addr;
	unsigned int seq_addr;
	unsigned int pps_addr;
	unsigned int pps_2addr;
	unsigned int mem_to_reg_addr;
	/* 31-16: buff size, 15-0: size filled by host; dwords */
	unsigned int mem_to_reg_size;
	unsigned int slice_params_addr;
	unsigned int slice_params_size;  /* dwords */
	unsigned int last_luma_recon;
	unsigned int last_chroma_recon;
	unsigned int luma_err_base;
	unsigned int chroma_err_base;
	unsigned int scaled_display_size;
	unsigned int horz_scale_control;
	unsigned int vert_scale_control;
	unsigned int scale_output_size;
	unsigned int vlc_idx_table_size;
	unsigned int vlc_idx_table_addr;
	unsigned int vlc_tables_size;
	unsigned int vlc_tables_addr;
	unsigned int display_picture_size;
	unsigned int parser_mode;
	/* needed for separate colour planes */
	unsigned int intra_buf_base_addr;
	unsigned int intra_buf_size_per_plane;
	unsigned int intra_buf_size_per_pipe;
	unsigned int chroma2reconstructed_addr;
	unsigned int luma_alt_addr;
	unsigned int chroma_alt_addr;
	unsigned int chroma2alt_addr;
	unsigned int aux_line_buf_size_per_pipe;
	unsigned int aux_line_buffer_base_addr;
	unsigned int alt_output_pict_rotation;
	/* miscellaneous flags */
	struct {
		unsigned is_chromainterleaved   : 1;
		unsigned is_packedformat        : 1;
		unsigned is_discontinuousmbs    : 1;
	};
};

#define CMD_VDEC_EXT_DWSIZE             (sizeof(VDEC_EXT_CMD) / sizeof(unsigned int))
#endif /* VDEC_USE_PVDEC_COMPATIBILITY */

/* Completion */
#define CMD_COMPLETION                          (0x60000000)
#define CMD_COMPLETION_DWSIZE                   (1)

#ifdef VDEC_USE_PVDEC_SEC
/* Slice done */
#define CMD_SLICE_DONE                          (0x70000000)
#define CMD_SLICE_DONE_DWSIZE                   (1)
#endif /* VDEC_USE_PVDEC_SEC */

/* Bitstream segments */
#define CMD_BITSTREAM_SEGMENTS                  (0xD0000000)
#define CMD_BITSTREAM_SEGMENTS_MINUS1_MASK      (0x0000001F)
#define CMD_BITSTREAM_PARSE_BLK_MASK            (0x0000FF00)
#ifdef VDEC_USE_PVDEC_COMPATIBILITY
#define CMD_BITSTREAM_SEGMENTS_MORE_FOLLOW_MASK (0x00000020)
#define CMD_BITSTREAM_EOP_MASK                  (0x00000040)
#define CMD_BITSTREAM_BS_TOT_SIZE_WORD_OFFSET   (1)
#define CMD_BITSTREAM_BS_SEG_LIST_WORD_OFFSET   (2)
#define CMD_BITSTREAM_HDR_DW_SIZE       CMD_BITSTREAM_BS_SEG_LIST_WORD_OFFSET

#define CMD_BITSTREAM_SEGMENTS_MAX_NUM          (60)
#endif /* VDEC_USE_PVDEC_COMPATIBILITY */

#ifdef VDEC_USE_PVDEC_COMPATIBILITY
/* Signatures */
/* Signature set ids (see hwSignatureModules.c for exact order). */
/* -- FRONT END/ENTROPY_PIPE ----------------------------------- */
/*
 * Signature group 0:
 * REG(PVDEC_ENTROPY, CR_SR_SIGNATURE)
 * REG(MSVDX_VEC,     CR_SR_CRC)
 */
#define PVDEC_SIGNATURE_GROUP_0  BIT(0)
/*
 * Signature group 1:
 * REG(PVDEC_ENTROPY, CR_HEVC_PARSER_SIGNATURE)
 * REG(PVDEC_ENTROPY, CR_ENCAP_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_1  BIT(1)
/*
 * Signature group 2:
 * REG(PVDEC_ENTROPY, CR_GENC_ENGINE_OUTPUT_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_2  BIT(2)
/*
 * Signature group 3:
 * REGREP(PVDEC_ENTROPY, CR_GENC_BUFFER_SIGNATURE, 0)
 * REGREP(PVDEC_ENTROPY, CR_GENC_BUFFER_SIGNATURE, 1)
 * REGREP(PVDEC_ENTROPY, CR_GENC_BUFFER_SIGNATURE, 2)
 * REGREP(PVDEC_ENTROPY, CR_GENC_BUFFER_SIGNATURE, 3)
 * REG(   PVDEC_ENTROPY, CR_GENC_FRAGMENT_SIGNATURE)
 * REG(   PVDEC_ENTROPY, CR_GENC_FRAGMENT_READ_SIGNATURE)
 * REG(   PVDEC_ENTROPY, CR_GENC_FRAGMENT_WRADDR_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_3  BIT(3)
/* -- GENC_DEC -------------------------------------------------- */
/*
 * Signature group 4:
 * REG(   PVDEC_VEC_BE, CR_GDEC_FRAGMENT_REQ_SIGNATURE)
 * REG(   PVDEC_VEC_BE, CR_GDEC_SYS_WR_SIGNATURE)
 * REG(   PVDEC_VEC_BE, CR_GDEC_MEM2REG_SYS_WR_SIGNATURE)
 * REG(   PVDEC_VEC_BE, CR_SLICE_STRUCTURE_REQ_SIGNATURE)
 * REG(   PVDEC_VEC_BE, CR_SLICE_STRUCTURE_OVER1K_REQ_SIGNATURE)
 * REG(   PVDEC_VEC_BE, CR_MEM_STRUCTURE_REQ_SIGNATURE)
 * REGREP(PVDEC_VEC_BE, CR_GDEC_DATA_REQ_SIGNATURE, 0)
 * REGREP(PVDEC_VEC_BE, CR_GDEC_DATA_REQ_SIGNATURE, 1)
 * REGREP(PVDEC_VEC_BE, CR_GDEC_DATA_REQ_SIGNATURE, 2)
 * REGREP(PVDEC_VEC_BE, CR_GDEC_DATA_REQ_SIGNATURE, 3)
 */
#define PVDEC_SIGNATURE_GROUP_4  BIT(4)
/*
 * Signature group 5:
 * REG(   PVDEC_VEC_BE, CR_GDEC_FRAGMENT_SIGNATURE)
 * REG(   PVDEC_VEC_BE, CR_SLICE_STRUCTURE_SIGNATURE)
 * REG(   PVDEC_VEC_BE, CR_SLICE_STRUCTURE_OVER1K_SIGNATURE)
 * REG(   PVDEC_VEC_BE, CR_MEM_STRUCTURE_SIGNATURE)
 * REGREP(PVDEC_VEC_BE, CR_GDEC_BUFFER_SIGNATURE, 0)
 * REGREP(PVDEC_VEC_BE, CR_GDEC_BUFFER_SIGNATURE, 1)
 * REGREP(PVDEC_VEC_BE, CR_GDEC_BUFFER_SIGNATURE, 2)
 * REGREP(PVDEC_VEC_BE, CR_GDEC_BUFFER_SIGNATURE, 3)
 */
#define PVDEC_SIGNATURE_GROUP_5  BIT(5)
/* -- RESIDUAL AND COMMAND DEBUG--------------------------------- */
/*
 * Signature group 12:
 * REG(PVDEC_VEC_BE, CR_DECODE_TO_COMMAND_PRIME_SIGNATURE)
 * REG(PVDEC_VEC_BE, CR_DECODE_TO_COMMAND_SECOND_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_12  BIT(12)
/*
 * Signature group 13:
 * REG(PVDEC_VEC_BE, CR_DECODE_TO_RESIDUAL_PRIME_SIGNATURE)
 * REG(PVDEC_VEC_BE, CR_DECODE_TO_RESIDUAL_SECOND_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_13  BIT(13)
/*
 * Signature group 14:
 * REG(PVDEC_VEC_BE, CR_COMMAND_ABOVE_READ_SIGNATURE)
 * REG(PVDEC_VEC_BE, CR_COMMAND_ABOVE_WRITE_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_14  BIT(14)
/*
 * Signature group 15:
 * REG(PVDEC_VEC_BE, CR_TEMPORAL_READ_SIGNATURE)
 * REG(PVDEC_VEC_BE, CR_TEMPORAL_WRITE_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_15  BIT(15)
/* --VEC--------------------------------------------------------- */
/*
 * Signature group 16:
 * REG(PVDEC_VEC_BE, CR_COMMAND_OUTPUT_SIGNATURE)
 * REG(MSVDX_VEC,    CR_VEC_IXFORM_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_16  BIT(16)
/*
 * Signature group 17:
 * REG(PVDEC_VEC_BE, CR_RESIDUAL_OUTPUT_SIGNATURE)
 * REG(MSVDX_VEC,    CR_VEC_COMMAND_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_17  BIT(17)
/* --VDMC-------------------------------------------------------- */
/*
 * Signature group 18:
 * REG(MSVDX_VDMC, CR_VDMC_REFERENCE_CACHE_SIGNATURE)
 * REG(MSVDX_VDMC, CR_VDMC_REFERENCE_CACHE_MEM_WADDR_SIGNATURE)
 * REG(MSVDX_VDMC, CR_VDMC_REFERENCE_CACHE_MEM_RADDR_SIGNATURE)
 * REG(MSVDX_VDMC, CR_VDMC_REFERENCE_CACHE_MEM_WDATA_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_18  BIT(18)
/*
 * Signature group 19:
 * REG(MSVDX_VDMC, CR_VDMC_2D_FILTER_PIPELINE_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_19  BIT(19)
/*
 * Signature group 20:
 * REG(MSVDX_VDMC, CR_VDMC_PIXEL_RECONSTRUCTION_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_20  BIT(20)
/*
 * Signature group 21:
 * REG(MSVDX_VDMC, CR_VDMC_MCU_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_21  BIT(21)
/* ---VDEB------------------------------------------------------- */
/*
 * Signature group 22:
 * REG(MSVDX_VDEB, CR_VDEB_SYS_MEM_RDATA_LUMA_SIGNATURE)
 * REG(MSVDX_VDEB, CR_VDEB_SYS_MEM_RDATA_CHROMA_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_22  BIT(22)
/*
 * Signature group 23:
 * REG(MSVDX_VDEB, CR_VDEB_SYS_MEM_ADDR_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_23  BIT(23)
/*
 * Signature group 24:
 * REG(MSVDX_VDEB, CR_VDEB_SYS_MEM_WDATA_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_24  BIT(24)
/* ---SCALER----------------------------------------------------- */
/*
 * Signature group 25:
 * REG(MSVDX_VDEB, CR_VDEB_SCALE_ADDR_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_25  BIT(25)
/*
 * Signature group 26:
 * REG(MSVDX_VDEB, CR_VDEB_SCALE_WDATA_SIGNATURE)
 */
#define PVDEC_SIGNATURE_GROUP_26  BIT(26)
/* ---PICTURE CHECKSUM------------------------------------------- */
/*
 * Signature group 27:
 * REG(MSVDX_VDEB, CR_VDEB_HEVC_CHECKSUM_LUMA)
 * REG(MSVDX_VDEB, CR_VDEB_HEVC_CHECKSUM_CB)
 * REG(MSVDX_VDEB, CR_VDEB_HEVC_CHECKSUM_CR)
 */
#define PVDEC_SIGNATURE_GROUP_27  BIT(27)
#define PVDEC_SIGNATURE_NEW_METHOD  BIT(31)

/* Debug messages */
#define DEBUG_DATA_TYPE_MASK             0xF
#define DEBUG_DATA_TYPE_SHIFT            28

#define DEBUG_DATA_MSG_TYPE_MASK         0x1
#define DEBUG_DATA_MSG_TYPE_SHIFT        15

#define DEBUG_DATA_MSG_ARG_COUNT_MASK    0x7
#define DEBUG_DATA_MSG_ARG_COUNT_SHIFT   12

#define DEBUG_DATA_MSG_LINE_NO_MASK      0xFFF
#define DEBUG_DATA_MSG_LINE_NO_SHIFT     0

#define DEBUG_DATA_TYPE_HEADER  (0)
#define DEBUG_DATA_TYPE_STRING  (1)
#define DEBUG_DATA_TYPE_PARAMS  (2)
#define DEBUG_DATA_TYPE_MSG     (3)
#define DEBUG_DATA_TYPE_PERF    (6)

#define DEBUG_DATA_MSG_TYPE_LOG     0
#define DEBUG_DATA_MSG_TYPE_ASSERT  1

#define DEBUG_DATA_TAPE_PERF_INC_TIME_MASK   0x1
#define DEBUG_DATA_TYPE_PERF_INC_TIME_SHIFT  28
#define DEBUG_DATA_TYPE_PERF_INC_TIME        0x1

#define DEBUG_DATA_SET_TYPE(val, type, data_type) \
	({ \
		data_type __val = val; \
		((__val) = (__val & ~(DEBUG_DATA_TYPE_MASK << DEBUG_DATA_TYPE_SHIFT)) | \
			   ((type) << DEBUG_DATA_TYPE_SHIFT)); })

#define DEBUG_DATA_MSG_SET_ARG_COUNT(val, ac, data_type) \
	({ \
		data_type __val = val; \
		(__val = (__val & \
			  ~(DEBUG_DATA_MSG_ARG_COUNT_MASK << DEBUG_DATA_MSG_ARG_COUNT_SHIFT)) \
			 | ((ac) << DEBUG_DATA_MSG_ARG_COUNT_SHIFT)); })

#define DEBUG_DATA_MSG_SET_LINE_NO(val, ln, type) \
	({ \
		type __val = val; \
		(__val = (__val & \
			  ~(DEBUG_DATA_MSG_LINE_NO_MASK << DEBUG_DATA_MSG_LINE_NO_SHIFT)) \
			 | ((ln) << DEBUG_DATA_MSG_LINE_NO_SHIFT)); })

#define DEBUG_DATA_MSG_SET_TYPE(val, tp, type) \
	({ \
		type __val = val; \
		(__val = (__val & \
			  ~(DEBUG_DATA_MSG_TYPE_MASK << DEBUG_DATA_MSG_TYPE_SHIFT)) \
			 | ((tp) << DEBUG_DATA_MSG_TYPE_SHIFT)); })

#define DEBUG_DATA_GET_TYPE(val) \
	(((val) >> DEBUG_DATA_TYPE_SHIFT) & DEBUG_DATA_TYPE_MASK)
#define DEBUG_DATA_TYPE_PERF_IS_INC_TIME(val) \
	(((val) >> DEBUG_DATA_TYPE_PERF_INC_TIME_SHIFT) \
	 & DEBUG_DATA_TAPE_PERF_INC_TIME_MASK)
#define DEBUG_DATA_MSG_GET_ARG_COUNT(val) \
	(((val) >> DEBUG_DATA_MSG_ARG_COUNT_SHIFT) \
	 & DEBUG_DATA_MSG_ARG_COUNT_MASK)
#define DEBUG_DATA_MSG_GET_LINE_NO(val) \
	(((val) >> DEBUG_DATA_MSG_LINE_NO_SHIFT) \
	 & DEBUG_DATA_MSG_LINE_NO_MASK)
#define DEBUG_DATA_MSG_GET_TYPE(val) \
	(((val) >> DEBUG_DATA_MSG_TYPE_SHIFT) & DEBUG_DATA_MSG_TYPE_MASK)
#define DEBUG_DATA_MSG_TYPE_IS_ASSERT(val) \
	(DEBUG_DATA_MSG_GET_TYPE(val) == DEBUG_DATA_MSG_TYPE_ASSERT \
	 ? IMG_TRUE : IMG_FALSE)
#define DEBUG_DATA_MSG_TYPE_IS_LOG(val) \
	(DEBUG_DATA_MSG_GET_TYPE(val) == DEBUG_DATA_MSG_TYPE_LOG ? \
	 IMG_TRUE : IMG_FALSE)

#define DEBUG_DATA_MSG_LAT(ln, ac, tp)            \
	(((ln) << DEBUG_DATA_MSG_LINE_NO_SHIFT) | \
	 ((ac) << DEBUG_DATA_MSG_ARG_COUNT_SHIFT) | \
	 ((tp) << DEBUG_DATA_MSG_TYPE_SHIFT))
/* FWBSP-mode specific defines. */
#ifdef VDEC_USE_PVDEC_SEC
/**
 * FWBSP_ENC_BSTR_BUF_QUEUE_LEN - Suggested number of bitstream buffers submitted (queued)
 * to firmware for processing at the same time.
 */
#define FWBSP_ENC_BSTR_BUF_QUEUE_LEN 1

#endif /* VDEC_USE_PVDEC_SEC */

#endif /* VDEC_USE_PVDEC_COMPATIBILITY */
#endif /* FW_INTERFACE_H_ */
