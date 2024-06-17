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

#ifndef _COREFLAGS_H_
#define _COREFLAGS_H_

#define SERIALIZED_PIPES (1)

/* The number of TOPAZ cores present in the system */
#define TOPAZHP_MAX_NUM_PIPES	(4)

#define TOPAZHP_MAX_POSSIBLE_STREAMS (8)
#define TOPAZHP_MAX_BU_SUPPORT_HD	90
#define TOPAZHP_MAX_BU_SUPPORT_4K	128

#define USE_VCM_HW_SUPPORT (1)
/* controls the firmwares ability to support the optional hardware input scaler */
#define INPUT_SCALER_SUPPORTED (1)
/* controls the firmwares ability to support secure mode firmware upload */
#define SECURE_MODE_POSSIBLE (1)

/* controls the firmwares ability to support secure input/output ports */
#define SECURE_IO_PORTS (1)

/* Line counter feature is not ready for Onyx yet
 * (comment the define to remove the feature from builds)
 */
#define LINE_COUNTER_SUPPORTED (1)

#endif
