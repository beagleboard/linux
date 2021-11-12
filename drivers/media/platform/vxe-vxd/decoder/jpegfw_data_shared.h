/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Public data structures for the hevc parser firmware module
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Sunita Nadampalli <sunitan@ti.com>
 *
 * Re-written for upstream
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */
#ifdef USE_SHARING
#endif

#ifndef _JPEGFW_DATA_H_
#define _JPEGFW_DATA_H_

#include "vdecfw_share.h"
#include "vdecfw_shared.h"

#define JPEG_VDEC_8x8_DCT_SIZE             64 //!< Number of elements in 8x8 DCT
#define JPEG_VDEC_MAX_COMPONENTS           4  //!< Maximum number of component in JPEG
#define JPEG_VDEC_MAX_SETS_HUFFMAN_TABLES  2  //!< Maximum set of huffman table in JPEG
#define JPEG_VDEC_MAX_QUANT_TABLES         4  //!< Maximum set of quantisation table in JPEG
#define JPEG_VDEC_TABLE_CLASS_NUM          2  //!< Maximum set of class of huffman table in JPEG
#define JPEG_VDEC_PLANE_MAX                4  //!< Maximum number of planes

struct hentry {
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, unsigned short, code);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, unsigned char, codelen);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, unsigned char, value);
};

/*
 * This structure contains JPEG huffmant table
 * NOTE: Should only contain JPEG specific information.
 * @brief  JPEG Huffman Table Information
 */
struct vdec_jpeg_huffman_tableinfo {
	/* number of bits */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, unsigned char, bits[16]);
	/* codeword value */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, unsigned char, values[256]);
};

/*
 * This structure contains JPEG DeQunatisation table
 * NOTE: Should only contain JPEG specific information.
 * @brief  JPEG Dequantisation Table Information
 */
struct vdec_jpeg_de_quant_tableinfo {
	/* Qunatisation precision */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, unsigned char, precision);
	/* Qunatisation Value for 8x8 DCT  */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, unsigned short, elements[64]);
};

/*
 * This describes the JPEG parser component "Header data", shown in the
 * Firmware Memory Layout diagram. This data is required by the JPEG firmware
 * and should be supplied by the Host.
 */
struct jpegfw_header_data {
	/* Primary decode buffer base addresses */
	struct vdecfw_image_buffer primary;
	/* Reference (output) picture base addresses */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, unsigned int,
			plane_offsets[JPEG_VDEC_PLANE_MAX]);
	/* SOS fields count value */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, unsigned char, hdr_sos_count);
};

/*
 * This describes the JPEG  parser component "Context data".
 * JPEG does not need any data to be saved between pictures, this structure
 * is needed only to fit in firmware framework.
 */
struct jpegfw_context_data {
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, unsigned int, dummy);
};

#endif /* _JPEGFW_DATA_H_ */
