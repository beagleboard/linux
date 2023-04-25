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

#ifndef _VXECOMMON_H_
#define _VXECOMMON_H_

#include "topazscfwif.h"
#include "../common/vid_buf.h"

/*
 * Enum describing buffer lock status
 */
enum lock_status {
	BUFFER_FREE	= 1,  //!< Buffer is not locked
	HW_LOCK,          //!< Buffer is locked by hardware
	SW_LOCK,          //!< Buffer is locked by software
	NOTDEVICEMEMORY,  //!< Buffer is not a device memory buffer
	LOCK_ST_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Struct describing a data buffer
 */
struct img_buffer {
	struct vidio_ddbufinfo mem_info;	//!< Pointer to the memory handle for the buffer
	enum lock_status lock;                  //!< Lock status for the buffer
	unsigned int	size;               //!< Size in bytes of the buffer
	unsigned int	bytes_written;       //!< Number of bytes written into buffer
};

/*
 * Struct describing a coded data buffer
 */
struct img_coded_buffer {
	struct vidio_ddbufinfo mem_info;	//!< Pointer to the memory handle for the buffer
	enum lock_status lock;                  //!< Lock status for the buffer
	unsigned int	size;               //!< Size in bytes of the buffer
	unsigned int	bytes_written;       //!< Number of bytes written into buffer
};

struct coded_info {
	struct img_buffer *code_package_fw_buffer;
	struct coded_package_dma_info *coded_package_fw;
};

// This structure is used by the Drivers
struct coded_package_host {
	struct coded_info mtx_info;
	/* Array of pointers to buffers */
	struct img_coded_buffer *coded_buffer[MAX_CODED_BUFFERS_PER_PACKAGE];
	struct img_buffer *header_buffer;
	unsigned char num_coded_buffers;
	unsigned char busy;
};

/*
 * Struct describing surface component info
 */
struct img_surf_component_info {
	unsigned int	step;
	unsigned int	width;
	unsigned int	height;
	unsigned int	phys_width;
	unsigned int	phys_height;
};

/*
 * Struct describing a frame
 */
struct img_frame {
	struct	img_buffer *y_plane_buffer;		//!< pointer to the image buffer
	struct	img_buffer *u_plane_buffer;		//!< pointer to the image buffer
	struct	img_buffer *v_plane_buffer;		//!< pointer to the image buffer
	unsigned int	width_bytes;			//!< stride of pBuffer
	unsigned int	height;				//!< height of picture in pBuffer

	unsigned int	component_count;	//!< number of colour components used
	enum img_format	format;
	unsigned int	component_offset[3];
	unsigned int	bottom_component_offset[3];
	struct img_surf_component_info	component_info[3];
	int	y_component_offset;
	int	u_component_offset;
	int	v_component_offset;
	int	field0_y_offset, field1_y_offset;
	int	field0_u_offset, field1_u_offset;
	int	field0_v_offset, field1_v_offset;
	unsigned short	src_y_stride_bytes, src_uv_stride_bytes;
	unsigned char	imported;
};

/*
 * Struct describing an array of frames
 */
struct img_frame_array {
	unsigned int array_size;   //!< Number of frames in array
	struct img_frame *frame;          //!< Pointer to start of frame array
};

/*
 * Struct describing list items
 */
struct list_item {
	struct list_item *next;		//!< Next item in the list
	void *data;				//!< pointer to list item data
};

/*
 * Struct describing rate control params
 */
struct img_rc_params {
	unsigned int	bits_per_second;		//!< Bit rate
	/* !< Transfer rate of encoded data from encoder to the output */
	unsigned int	transfer_bits_per_second;
	unsigned int	initial_qp_i;		//!< Initial QP I frames (only field used by JPEG)
	unsigned int	initial_qp_p;		//!< Initial QP P frames (only field used by JPEG)
	unsigned int	initial_qp_b;		//!< Initial QP B frames (only field used by JPEG)
	unsigned int	bu_size;					//!< Basic unit size
	unsigned int	frame_rate;
	unsigned int	buffer_size;
	unsigned int	intra_freq;
	short	min_qp;
	short	max_qp;
	unsigned char rc_enable;
	int	initial_level;
	int	initial_delay;
	unsigned short	bframes;
	unsigned char	hierarchical;

	/* !< Enable movement of slice boundary when Qp is high */
	unsigned char  enable_slice_bob;
	/* !< Maximum number of rows the slice boundary can be moved */
	unsigned char  max_slice_bob;
	/* !< Minimum Qp at which slice bobbing should take place */
	unsigned char  slice_bob_qp;

	signed char	qcp_offset;
	unsigned char	sc_detect_disable;
	unsigned int	slice_byte_limit;
	unsigned int	slice_mb_limit;
	enum img_rcmode rc_mode;
	enum img_rc_vcm_mode rc_vcm_mode;
	unsigned int	rc_cfs_max_margin_perc;
	unsigned char	disable_frame_skipping;
	unsigned char	disable_vcm_hardware;
};

/*
 * Bit fields for ui32MmuFlags
 */
#define MMU_USE_MMU_FLAG	0x00000001
#define MMU_EXTENDED_ADDR_FLAG	0x00000004

#endif
