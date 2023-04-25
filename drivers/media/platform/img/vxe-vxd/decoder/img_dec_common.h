/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IMG DEC common header
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Amit Makani <amit.makani@ti.com>
 *
 * Re-written for upstreamimg
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 */

#ifndef _IMG_DEC_COMMON_H
#define _IMG_DEC_COMMON_H

#include <linux/types.h>

#define VXD_MAX_PIPES 2
#define MAX_DST_BUFFERS 32

/* Helpers for parsing core properties. Based on HW registers layout. */
#define VXD_GET_BITS(v, lb, rb, type) \
	({                                \
		type __rb = (rb);                                       \
		(((v) >> (__rb)) & ((1 << ((lb) - __rb + 1)) - 1)); })
#define VXD_GET_BIT(v, b) (((v) >> (b)) & 1)

/* Get major core revision. */
#define VXD_MAJ_REV(props) (VXD_GET_BITS((props).core_rev, 23, 16, unsigned int))
/* Get minor core revision. */
#define VXD_MIN_REV(props) (VXD_GET_BITS((props).core_rev, 15, 8, unsigned int))
/* Get maint core revision. */
#define VXD_MAINT_REV(props) (VXD_GET_BITS((props).core_rev, 7, 0, unsigned int))
/* Get number of entropy pipes available (HEVC). */
#define VXD_NUM_ENT_PIPES(props) ((props).pvdec_core_id & 0xF)
/* Get number of pixel pipes available (other standards). */
#define VXD_NUM_PIX_PIPES(props) (((props).pvdec_core_id & 0xF0) >> 4)
/* Get number of bits used by external memory interface. */
#define VXD_EXTRN_ADDR_WIDTH(props) ((((props).mmu_config0 & 0xF0) >> 4) + 32)

/* Check whether specific standard is supported by the pixel pipe. */
#define VXD_HAS_MPEG2(props, pipe) VXD_GET_BIT(props.pixel_pipe_cfg[pipe], 0)
#define VXD_HAS_MPEG4(props, pipe) VXD_GET_BIT(props.pixel_pipe_cfg[pipe], 1)
#define VXD_HAS_H264(props, pipe) VXD_GET_BIT(props.pixel_pipe_cfg[pipe], 2)
#define VXD_HAS_VC1(props, pipe) VXD_GET_BIT(props.pixel_pipe_cfg[pipe], 3)
#define VXD_HAS_WMV9(props, pipe) VXD_GET_BIT(props.pixel_pipe_cfg[pipe], 4)
#define VXD_HAS_JPEG(props, pipe) VXD_GET_BIT(props.pixel_pipe_cfg[pipe], 5)
#define VXD_HAS_MPEG4_DATA_PART(props, pipe) \
	VXD_GET_BIT(props.pixel_pipe_cfg[pipe], 6)
#define VXD_HAS_AVS(props, pipe) VXD_GET_BIT(props.pixel_pipe_cfg[pipe], 7)
#define VXD_HAS_REAL(props, pipe) VXD_GET_BIT(props.pixel_pipe_cfg[pipe], 8)
#define VXD_HAS_VP6(props, pipe) VXD_GET_BIT(props.pixel_pipe_cfg[pipe], 9)
#define VXD_HAS_VP8(props, pipe) VXD_GET_BIT(props.pixel_pipe_cfg[pipe], 10)
#define VXD_HAS_SORENSON(props, pipe) \
	VXD_GET_BIT(props.pixel_pipe_cfg[pipe], 11)
#define VXD_HAS_HEVC(props, pipe) VXD_GET_BIT(props.pixel_pipe_cfg[pipe], 22)

/* Check whether specific feature is supported by the pixel pipe */

/*
 * Max picture size for HEVC still picture profile is 64k wide and/or 64k
 * high.
 */
#define VXD_HAS_HEVC_64K_STILL(props, pipe) \
	(VXD_GET_BIT((props).pixel_misc_cfg[pipe], 24))

/* Pixel processing pipe index. */
#define VXD_PIX_PIPE_ID(props, pipe) \
	(VXD_GET_BITS((props).pixel_misc_cfg[pipe], 18, 16, unsigned int))

/* Number of stream supported by the pixel pipe DMAC and shift register. */
#define VXD_PIX_NUM_STRS(props, pipe) \
	(VXD_GET_BITS((props).pixel_misc_cfg[pipe], 13, 12, unsigned int) + 1)

/* Is scaling supported. */
#define VXD_HAS_SCALING(props, pipe) \
	(VXD_GET_BIT((props).pixel_misc_cfg[pipe], 9))

/* Is rotation supported. */
#define VXD_HAS_ROTATION(props, pipe) \
	(VXD_GET_BIT((props).pixel_misc_cfg[pipe], 8))

/* Are HEVC range extensions supported. */
#define VXD_HAS_HEVC_REXT(props, pipe) \
	(VXD_GET_BIT((props).pixel_misc_cfg[pipe], 7))

/* Maximum bit depth supported by the pipe. */
#define VXD_MAX_BIT_DEPTH(props, pipe) \
	(VXD_GET_BITS((props).pixel_misc_cfg[pipe], 6, 4, unsigned int) + 8)

/*
 * Maximum chroma fomar supported by the pipe in HEVC mode.
 * 0x1 - 4:2:0
 * 0x2 - 4:2:2
 * 0x3 - 4:4:4
 */
#define VXD_MAX_HEVC_CHROMA_FMT(props, pipe) \
	(VXD_GET_BITS((props).pixel_misc_cfg[pipe], 3, 2, unsigned int))

/*
 * Maximum chroma format supported by the pipe in H264 mode.
 * 0x1 - 4:2:0
 * 0x2 - 4:2:2
 * 0x3 - 4:4:4
 */
#define VXD_MAX_H264_CHROMA_FMT(props, pipe) \
	(VXD_GET_BITS((props).pixel_misc_cfg[pipe], 1, 0, unsigned int))

/*
 * Maximum frame width and height supported in MSVDX pipeline.
 */
#define VXD_MAX_WIDTH_MSVDX(props) \
	(2 << (VXD_GET_BITS((props).pixel_max_frame_cfg, 4, 0, unsigned int)))
#define VXD_MAX_HEIGHT_MSVDX(props) \
	(2 << (VXD_GET_BITS((props).pixel_max_frame_cfg, 12, 8, unsigned int)))

/*
 * Maximum frame width and height supported in PVDEC pipeline.
 */
#define VXD_MAX_WIDTH_PVDEC(props) \
	(2 << (VXD_GET_BITS((props).pixel_max_frame_cfg, 20, 16, unsigned int)))
#define VXD_MAX_HEIGHT_PVDEC(props) \
	(2 << (VXD_GET_BITS((props).pixel_max_frame_cfg, 28, 24, unsigned int)))

#define PVDEC_COMMS_RAM_OFFSET      0x00002000
#define PVDEC_COMMS_RAM_SIZE        0x00001000
#define PVDEC_ENTROPY_OFFSET        0x00003000
#define PVDEC_ENTROPY_SIZE          0x1FF
#define PVDEC_VEC_BE_OFFSET         0x00005000
#define PVDEC_VEC_BE_SIZE           0x3FF
#define PVDEC_VEC_BE_CODEC_OFFSET   0x00005400
#define MSVDX_VEC_OFFSET            0x00006000
#define MSVDX_VEC_SIZE              0x7FF
#define MSVDX_CMD_OFFSET            0x00007000

/*
 * Virtual memory heap address ranges for tiled
 * and non-tiled buffers. Addresses within each
 * range should be assigned to the appropriate
 * buffers by the UM driver and mapped into the
 * device using the corresponding KM driver ioctl.
 */
#define PVDEC_HEAP_UNTILED_START    0x00400000ul
#define PVDEC_HEAP_UNTILED_SIZE     0x3FC00000ul
#define PVDEC_HEAP_TILE512_START    0x40000000ul
#define PVDEC_HEAP_TILE512_SIZE     0x10000000ul
#define PVDEC_HEAP_TILE1024_START   0x50000000ul
#define PVDEC_HEAP_TILE1024_SIZE    0x20000000ul
#define PVDEC_HEAP_TILE2048_START   0x70000000ul
#define PVDEC_HEAP_TILE2048_SIZE    0x30000000ul
#define PVDEC_HEAP_TILE4096_START   0xA0000000ul
#define PVDEC_HEAP_TILE4096_SIZE    0x30000000ul
#define PVDEC_HEAP_BITSTREAM_START  0xD2000000ul
#define PVDEC_HEAP_BITSTREAM_SIZE   0x0A000000ul
#define PVDEC_HEAP_STREAM_START     0xE4000000ul
#define PVDEC_HEAP_STREAM_SIZE      0x1C000000ul

/*
 * Max size of the message payload, in bytes. There are 7 bits used to encode
 * the message size in the firmware interface.
 */
#define VXD_MAX_PAYLOAD_SIZE (127 * sizeof(unsigned int))
/* Max size of the input message in bytes. */
#define VXD_MAX_INPUT_SIZE (VXD_MAX_PAYLOAD_SIZE + sizeof(struct vxd_fw_msg))
/*
 * Min size of the input message. Two words needed for message header and
 * stream PTD
 */
#define VXD_MIN_INPUT_SIZE 2
/*
 * Offset of the stream PTD within message. This word has to be left null in
 * submitted message, driver will fill it in with an appropriate value.
 */
#define VXD_PTD_MSG_OFFSET 1

/* Read flags */
#define VXD_FW_MSG_RD_FLAGS_MASK 0xffff
/* Driver watchdog interrupted processing of the message. */
#define VXD_FW_MSG_FLAG_DWR 0x1
/* VXD MMU fault occurred when the message was processed. */
#define VXD_FW_MSG_FLAG_MMU_FAULT 0x2
/* Invalid input message, e.g. the message was too large. */
#define VXD_FW_MSG_FLAG_INV 0x4
/* I/O error occurred when the message was processed. */
#define VXD_FW_MSG_FLAG_DEV_ERR 0x8
/*
 * Driver error occurred when the message was processed, e.g. failed to
 * allocate memory.
 */
#define VXD_FW_MSG_FLAG_DRV_ERR 0x10
/*
 * Item was canceled, without being fully processed
 * i.e. corresponding stream was destroyed.
 */
#define VXD_FW_MSG_FLAG_CANCELED 0x20
/* Firmware internal error occurred when the message was processed */
#define VXD_FW_MSG_FLAG_FATAL 0x40

/* Write flags */
#define VXD_FW_MSG_WR_FLAGS_MASK 0xffff0000
/* Indicates that message shall be dropped after sending it to the firmware. */
#define VXD_FW_MSG_FLAG_DROP 0x10000
/*
 * Indicates that message shall be exclusively handled by
 * the firmware/hardware. Any other pending messages are
 * blocked until such message is handled.
 */
#define VXD_FW_MSG_FLAG_EXCL 0x20000

#define VXD_MSG_SIZE(msg) (sizeof(struct vxd_fw_msg) + ((msg).payload_size))

/* Header included at the beginning of firmware binary */
struct vxd_fw_hdr {
	unsigned int core_size;
	unsigned int blob_size;
	unsigned int firmware_id;
	unsigned int timestamp;
};

/*
 * struct vxd_dev_fw - Core component will allocate a buffer for firmware.
 *                     This structure holds the information about the firmware
 *                     binary.
 * @buf_id: The buffer id allocation
 * @hdr: firmware header information
 * @fw_size: The size of the fw. Set after successful firmware request.
 */
struct vxd_dev_fw {
	int buf_id;
	struct vxd_fw_hdr *hdr;
	unsigned int fw_size;
	unsigned char ready;
};

/*
 * struct vxd_core_props - contains HW core properties
 * @core_rev: Core revision based on register CR_PVDEC_CORE_REV
 * @pvdec_core_id: PVDEC Core id based on register CR_PVDEC_CORE_ID
 * @mmu_config0: MMU configuration 0 based on register MMU_CONFIG0
 * @mmu_config1: MMU configuration 1 based on register MMU_CONFIG1
 * @mtx_ram_size: size of the MTX RAM based on register CR_PROC_DEBUG
 * @pixel_max_frame_cfg: indicates the max frame height and width for
 *                       PVDEC pipeline and MSVDX pipeline based on register
 *                       MAX_FRAME_CONFIG
 * @pixel_pipe_cfg: pipe configuration which codecs are supported in a
 *                  Pixel Processing Pipe, based on register
 *                  PIXEL_PIPE_CONFIG
 * @pixel_misc_cfg: Additional pipe configuration eg. supported scaling
 *                  or rotation, based on register PIXEL_MISC_CONFIG
 * @dbg_fifo_size: contains the depth of the Debug FIFO, based on
 *                 register CR_PROC_DEBUG_FIFO_SIZE
 */
struct vxd_core_props {
	unsigned int core_rev;
	unsigned int pvdec_core_id;
	unsigned int mmu_config0;
	unsigned int mmu_config1;
	unsigned int mtx_ram_size;
	unsigned int pixel_max_frame_cfg;
	unsigned int pixel_pipe_cfg[VXD_MAX_PIPES];
	unsigned int pixel_misc_cfg[VXD_MAX_PIPES];
	unsigned int dbg_fifo_size;
};

struct vxd_alloc_data {
	unsigned int heap_id;       /* [IN] Heap ID of allocator                */
	unsigned int size;          /* [IN] Size of device memory (in bytes)    */
	unsigned int attributes;    /* [IN] Attributes of buffer */
	unsigned int buf_id;        /* [OUT] Generated buffer ID                */
};

struct vxd_free_data {
	unsigned int buf_id;        /* [IN] ID of device buffer to free */
};
#endif /* _IMG_DEC_COMMON_H */
