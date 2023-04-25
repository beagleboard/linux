/* SPDX-License-Identifier: GPL-2.0 */
/*
 * h264 idx table definitions
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Angela Stegmaier <angelabaker@ti.com>
 */

#ifndef __H264_IDX_H__
#define __H264_IDX_H__

#include <linux/types.h>

static unsigned short h264_vlc_index_data[38][3] = {
	{ 2, 5, 0   },  /* NumCoeffTrailingOnes_Table9-5_nC_0-1.out */
	{ 0, 3, 76  },  /* NumCoeffTrailingOnes_Table9-5_nC_2-3.out */
	{ 0, 3, 160 },  /* NumCoeffTrailingOnes_Table9-5_nC_4-7.out */
	{ 0, 2, 231 },  /* NumCoeffTrailingOnesFixedLen.out */
	{ 2, 2, 244 },  /* NumCoeffTrailingOnesChromaDC_YUV420.out */
	{ 2, 5, 261 },  /* NumCoeffTrailingOnesChromaDC_YUV422.out */
	{ 2, 5, 301 },  /* TotalZeros_00.out */
	{ 0, 2, 326 },  /* TotalZeros_01.out */
	{ 0, 2, 345 },  /* TotalZeros_02.out */
	{ 0, 2, 363 },  /* TotalZeros_03.out */
	{ 0, 2, 379 },  /* TotalZeros_04.out */
	{ 0, 2, 394 },  /* TotalZeros_05.out */
	{ 0, 2, 406 },  /* TotalZeros_06.out */
	{ 0, 1, 418 },  /* TotalZeros_07.out */
	{ 0, 1, 429 },  /* TotalZeros_08.out */
	{ 0, 1, 438 },  /* TotalZeros_09.out */
	{ 2, 2, 446 },  /* TotalZeros_10.out */
	{ 2, 2, 452 },  /* TotalZeros_11.out */
	{ 2, 1, 456 },  /* TotalZeros_12.out */
	{ 0, 0, 459 },  /* TotalZeros_13.out */
	{ 0, 0, 461 },  /* TotalZeros_14.out */
	{ 2, 2, 463 },  /* TotalZerosChromaDC_YUV420_00.out */
	{ 2, 1, 467 },  /* TotalZerosChromaDC_YUV420_01.out */
	{ 0, 0, 470 },  /* TotalZerosChromaDC_YUV420_02.out */
	{ 0, 0, 472 },  /* Run_00.out */
	{ 2, 1, 474 },  /* Run_01.out */
	{ 0, 1, 477 },  /* Run_02.out */
	{ 0, 1, 481 },  /* Run_03.out */
	{ 1, 1, 487 },  /* Run_04.out */
	{ 0, 2, 494 },  /* Run_05.out */
	{ 0, 2, 502 },  /* Run_06.out */
	{ 2, 4, 520 },  /* TotalZerosChromaDC_YUV422_00.out */
	{ 2, 2, 526 },  /* TotalZerosChromaDC_YUV422_01.out */
	{ 0, 1, 530 },  /* TotalZerosChromaDC_YUV422_02.out */
	{ 1, 2, 534 },  /* TotalZerosChromaDC_YUV422_03.out */
	{ 0, 0, 538 },  /* TotalZerosChromaDC_YUV422_04.out */
	{ 0, 0, 540 },  /* TotalZerosChromaDC_YUV422_05.out */
	{ 0, 0, 542 },  /* TotalZerosChromaDC_YUV422_06.out */
};

static const unsigned char h264_vlc_index_size = 38;

#endif
