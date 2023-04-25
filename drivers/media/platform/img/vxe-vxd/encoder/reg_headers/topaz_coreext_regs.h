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

#ifndef _REGCONV_H_topazhp_coreext_regs_h
#define _REGCONV_H_topazhp_coreext_regs_h

/* Register CR_SCALER_INPUT_SIZE */
#define MASK_TOPAZHP_EXT_CR_SCALER_INPUT_WIDTH_MIN1 0x00000FFF
#define SHIFT_TOPAZHP_EXT_CR_SCALER_INPUT_WIDTH_MIN1 0
#define MASK_TOPAZHP_EXT_CR_SCALER_INPUT_HEIGHT_MIN1 0x0FFF0000
#define SHIFT_TOPAZHP_EXT_CR_SCALER_INPUT_HEIGHT_MIN1 16

/* Register CR_SCALER_PITCH */
#define MASK_TOPAZHP_EXT_CR_SCALER_INPUT_HOR_PITCH 0x00007FFF
#define SHIFT_TOPAZHP_EXT_CR_SCALER_INPUT_HOR_PITCH 0
#define MASK_TOPAZHP_EXT_CR_SCALER_HOR_BILINEAR_FILTER 0x00008000
#define SHIFT_TOPAZHP_EXT_CR_SCALER_HOR_BILINEAR_FILTER 15
#define MASK_TOPAZHP_EXT_CR_SCALER_INPUT_VER_PITCH 0x7FFF0000
#define SHIFT_TOPAZHP_EXT_CR_SCALER_INPUT_VER_PITCH 16
#define MASK_TOPAZHP_EXT_CR_SCALER_VER_BILINEAR_FILTER 0x80000000
#define SHIFT_TOPAZHP_EXT_CR_SCALER_VER_BILINEAR_FILTER 31

/* Register CR_SCALER_CROP */
#define MASK_TOPAZHP_EXT_CR_SCALER_INPUT_CROP_VER 0x000000FF
#define SHIFT_TOPAZHP_EXT_CR_SCALER_INPUT_CROP_VER 0
#define MASK_TOPAZHP_EXT_CR_SCALER_INPUT_CROP_HOR 0x0000FF00
#define SHIFT_TOPAZHP_EXT_CR_SCALER_INPUT_CROP_HOR 8

/* Register CR_SCALER_CONTROL */
#define MASK_TOPAZHP_EXT_CR_SCALER_ENABLE 0x00000001
#define SHIFT_TOPAZHP_EXT_CR_SCALER_ENABLE 0
#define MASK_TOPAZHP_EXT_CR_ENABLE_COLOUR_SPACE_CONVERSION 0x00000002
#define SHIFT_TOPAZHP_EXT_CR_ENABLE_COLOUR_SPACE_CONVERSION 1
#define MASK_TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT 0x007F0000
#define SHIFT_TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT 16

/* 4:4:4, Any 3 colour space components plus reserved byte (e.g.
 * RGB), 8-bit components, packed 32-bit per pixel in a single plane, 8 LSBits not used
 */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_444IL4XBCA8		0x0000007E

/* 4:4:4, Any 3 colour space components plus reserved byte (e.g.
 * RGB), 8-bit components, packed 32-bit per pixel in a single plane, 8 MSBits not used
 */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_444IL4ABCX8		0x0000007C

/* RGB with 5 bits for R, 6 bits for G and 5 bits for B */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_444IL3RGB565		0x00000070

/* 4:4:4, Y in 1 plane, CrCb interleaved in 2nd plane, 8-bit components */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_444PL12YCRCB8		0x0000006A

/* 4:4:4, Y in 1 plane, CbCr interleaved in 2nd plane, 8-bit components */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_444PL12YCBCR8		0x00000068

/* 4:4:4, Y Cb Cr in 3 separate planes, 8-bit components
 * (could also be ABC, but colour space conversion is not supported by input scaler
 */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_444PL111YCBCR8		0x00000060

/* 4:2:2, CrYCbY interleaved in a single plane, 8-bit components */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_422IL3CRYCBY8		0x00000056

/* 4:2:2, CbYCrY interleaved in a single plane, 8-bit components */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_422IL3CBYCRY8		0x00000054

/* 4:2:2, YCrYCb interleaved in a single plane, 8-bit components */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_422IL3YCRYCB8		0x00000052

/* 4:2:2, YCbYCr interleaved in a single plane, 8-bit components */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_422IL3YCBYCR8		0x00000050

/* 4:2:2, Y in 1 plane, CrCb interleaved in 2nd plane, 8-bit components */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_422PL12YCRCB8		0x0000004A

/* 4:2:2, Y in 1 plane, CbCr interleaved in 2nd plane, 8-bit components */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_422PL12YCBCR8		0x00000048

/* 4:2:2, Y Cb Cr in 3 separate planes, 8-bit components */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_422PL111YCBCR8		0x00000040

/* 4:2:0, Y in 1 plane, CrCb interleaved in 2nd plane, 8-bit components */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_420PL12YCRCB8		0x0000002A

/* 4:2:0, Y in 1 plane, CbCr interleaved in 2nd plane, 8-bit components */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_420PL12YCBCR8		0x00000028

/* 4:2:0, Y Cb Cr in 3 separate planes, 8-bit components */
#define TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_420PL111YCBCR8		0x00000020

/* Register CR_CSC_SOURCE_MOD_Y_0 */
#define MASK_TOPAZHP_EXT_CR_CSC_SOURCE_MOD_00 0x00000003
#define SHIFT_TOPAZHP_EXT_CR_CSC_SOURCE_MOD_00 0

/* Subtract 1/2 maximum value from unsigned pixel component */
#define TOPAZHP_EXT_CR_CSC_SOURCE_MOD_00_MINUS_1_2		0x00000003

/* Subtract 1/16th maximum value from unsigned pixel component */
#define TOPAZHP_EXT_CR_CSC_SOURCE_MOD_00_MINUS_1_16		0x00000002

/* Source pixel component is unsigned */
#define TOPAZHP_EXT_CR_CSC_SOURCE_MOD_00_UNSIGNED		0x00000000

/* Register CR_CSC_SOURCE_MOD_Y_1 */
#define MASK_TOPAZHP_EXT_CR_CSC_SOURCE_MOD_01 0x00000003
#define SHIFT_TOPAZHP_EXT_CR_CSC_SOURCE_MOD_01 0

/* Subtract 1/2 maximum value from unsigned pixel component */
#define TOPAZHP_EXT_CR_CSC_SOURCE_MOD_01_MINUS_1_2		0x00000003

/* Subtract 1/16th maximum value from unsigned pixel component */
#define TOPAZHP_EXT_CR_CSC_SOURCE_MOD_01_MINUS_1_16		0x00000002

/* Source pixel component is unsigned */
#define TOPAZHP_EXT_CR_CSC_SOURCE_MOD_01_UNSIGNED		0x00000000

/* Register CR_CSC_SOURCE_CB_CR_1 */
#define MASK_TOPAZHP_EXT_CR_CSC_SOURCE_SRC_TO_CB_01 0x00000FFF
#define SHIFT_TOPAZHP_EXT_CR_CSC_SOURCE_SRC_TO_CB_01 0
#define SHIFT_TOPAZHP_EXT_CR_CSC_SOURCE_SRC_TO_CR_01 16

/* Register CR_CSC_SOURCE_MOD_Y_2 */
#define MASK_TOPAZHP_EXT_CR_CSC_SOURCE_MOD_02 0x00000003
#define SHIFT_TOPAZHP_EXT_CR_CSC_SOURCE_MOD_02 0

/* Subtract 1/2 maximum value from unsigned pixel component */
#define TOPAZHP_EXT_CR_CSC_SOURCE_MOD_02_MINUS_1_2		0x00000003

/* Subtract 1/16th maximum value from unsigned pixel component */
#define TOPAZHP_EXT_CR_CSC_SOURCE_MOD_02_MINUS_1_16		0x00000002

/* Source pixel component is unsigned */
#define TOPAZHP_EXT_CR_CSC_SOURCE_MOD_02_UNSIGNED		0x00000000

/* Register CR_CSC_SOURCE_CB_CR_2 */
#define MASK_TOPAZHP_EXT_CR_CSC_SOURCE_SRC_TO_CB_02 0x00000FFF
#define SHIFT_TOPAZHP_EXT_CR_CSC_SOURCE_SRC_TO_CR_02 16

/* Register CR_CSC_OUTPUT_COEFF_0 */
#define SHIFT_TOPAZHP_EXT_CR_CSC_OUTPUT_MIN_CLIP_00 0
#define SHIFT_TOPAZHP_EXT_CR_CSC_OUTPUT_MAX_CLIP_00 16
#define MASK_TOPAZHP_EXT_CR_CSC_OUTPUT_MOD_00 0x30000000
#define SHIFT_TOPAZHP_EXT_CR_CSC_OUTPUT_MOD_00 28

/* Add 1/16th maximum value prior to applying unsigned clamping */
#define TOPAZHP_EXT_CR_CSC_OUTPUT_MOD_00_ADD_1_16		0x00000002

/* Register CR_CSC_OUTPUT_COEFF_1 */
#define MASK_TOPAZHP_EXT_CR_CSC_OUTPUT_MIN_CLIP_01 0x000003FF
#define SHIFT_TOPAZHP_EXT_CR_CSC_OUTPUT_MIN_CLIP_01 0
#define SHIFT_TOPAZHP_EXT_CR_CSC_OUTPUT_MAX_CLIP_01 16
#define MASK_TOPAZHP_EXT_CR_CSC_OUTPUT_MOD_01 0x30000000
#define SHIFT_TOPAZHP_EXT_CR_CSC_OUTPUT_MOD_01 28

/* Add 1/2 maximum value prior to applying unsigned clamping */
#define TOPAZHP_EXT_CR_CSC_OUTPUT_MOD_01_ADD_1_2		0x00000003
#define MASK_TOPAZHP_EXT_CR_CSC_SOURCE_SRC_TO_Y 0x0FFF0000
#define SHIFT_TOPAZHP_EXT_CR_CSC_SOURCE_SRC_TO_Y 16

/* Register CR_CSC_SOURCE_CB_CR */
#define MASK_TOPAZHP_EXT_CR_CSC_SOURCE_SRC_TO_CB 0x00000FFF
#define SHIFT_TOPAZHP_EXT_CR_CSC_SOURCE_SRC_TO_CB 0
#define MASK_TOPAZHP_EXT_CR_CSC_SOURCE_SRC_TO_CR 0x0FFF0000
#define SHIFT_TOPAZHP_EXT_CR_CSC_SOURCE_SRC_TO_CR 16

/* Register CR_CSC_OUTPUT_COEFF */
#define MASK_TOPAZHP_EXT_CR_CSC_OUTPUT_MIN_CLIP 0x000003FF
#define SHIFT_TOPAZHP_EXT_CR_CSC_OUTPUT_MIN_CLIP 0
#define MASK_TOPAZHP_EXT_CR_CSC_OUTPUT_MAX_CLIP 0x03FF0000
#define SHIFT_TOPAZHP_EXT_CR_CSC_OUTPUT_MAX_CLIP 16

#endif
