/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Public data structures for the h264 parser firmware module.
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

#include "jpegfw_data_shared.h"

#ifndef _JPEGFW_DATA_H_
#define _JPEGFW_DATA_H_

#define JPEG_VDEC_8x8_DCT_SIZE             64 //!< Number of elements in 8x8 DCT
#define JPEG_VDEC_MAX_COMPONENTS           4  //!< Maximum number of component in JPEG
#define JPEG_VDEC_MAX_SETS_HUFFMAN_TABLES  2  //!< Maximum set of huffman table in JPEG
#define JPEG_VDEC_MAX_QUANT_TABLES         4  //!< Maximum set of quantisation table in JPEG
#define JPEG_VDEC_TABLE_CLASS_NUM          2  //!< Maximum set of class of huffman table in JPEG
#define JPEG_VDEC_PLANE_MAX                4  //!< Maximum number of planes

struct hentry {
	unsigned short code;
	unsigned char codelen;
	unsigned char value;
};

/**
 * struct vdec_jpeg_huffman_tableinfo - This structure contains JPEG huffmant table
 * @bits: number of bits
 * @values: codeword value
 *
 * NOTE: Should only contain JPEG specific information.
 * JPEG Huffman Table Information
 */
struct vdec_jpeg_huffman_tableinfo {
	/* number of bits */
	unsigned char bits[16];
	/* codeword value */
	unsigned char values[256];
};

/*
 * This structure contains JPEG DeQunatisation table
 * NOTE: Should only contain JPEG specific information.
 * @brief  JPEG Dequantisation Table Information
 */
struct vdec_jpeg_de_quant_tableinfo {
	/* Qunatisation precision */
	unsigned char precision;
	/* Qunatisation Value for 8x8 DCT  */
	unsigned short elements[64];
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
	unsigned int plane_offsets[JPEG_VDEC_PLANE_MAX];
	/* SOS fields count value */
	unsigned char hdr_sos_count;
};

/*
 * This describes the JPEG  parser component "Context data".
 * JPEG does not need any data to be saved between pictures, this structure
 * is needed only to fit in firmware framework.
 */
struct jpegfw_context_data {
	unsigned int dummy;
};

#endif /* _JPEGFW_DATA_H_ */
