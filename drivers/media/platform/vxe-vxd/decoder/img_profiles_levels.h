/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD DEC SYSDEV and UI Interface header
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

#ifndef __IMG_PROFILES_LEVELS_H
#define __IMG_PROFILES_LEVELS_H

#include "vdecdd_utils.h"

/* Minimum level value for h.264 */
#define H264_LEVEL_MIN              (9)
/* Maximum level value for h.264 */
#define H264_LEVEL_MAX             (52)
/* Number of major levels for h.264 (5 + 1 for special levels) */
#define H264_LEVEL_MAJOR_NUM        (6)
/* Number of minor levels for h.264 */
#define H264_LEVEL_MINOR_NUM            (4)
/* Number of major levels for HEVC */
#define HEVC_LEVEL_MAJOR_NUM        (6)
/* Number of minor levels for HEVC */
#define HEVC_LEVEL_MINOR_NUM        (3)

#endif /*__IMG_PROFILES_LEVELS_H */
