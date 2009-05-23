/*
 * SDRC register values for the Micron MT46H32M32LF-6
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 * Copyright (C) 2008 Nokia Corporation
 *
 * Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ARCH_ARM_MACH_OMAP2_SDRAM_MICRON_MT46H32M32LF
#define ARCH_ARM_MACH_OMAP2_SDRAM_MICRON_MT46H32M32LF

#include <mach/sdrc.h>

/* Micron MT46H32M32LF-6 */
/* XXX Using ARE = 0x1 (no autorefresh burst) -- can this be changed? */
static struct omap_sdrc_params mt46h32m32lf6_sdrc_params[] = {
	[0] = {
		.rate	     = 166000000,
		.actim_ctrla = 0x9a9db4c6,
		.actim_ctrlb = 0x00011217,
		.rfr_ctrl    = 0x0004dc01,
		.mr	     = 0x00000032,
	},
	[1] = {
		.rate	     = 165941176,
		.actim_ctrla = 0x9a9db4c6,
		.actim_ctrlb = 0x00011217,
		.rfr_ctrl    = 0x0004dc01,
		.mr	     = 0x00000032,
	},
	[2] = {
		.rate	     = 133333333,
		.actim_ctrla = 0x7a19b485,
		.actim_ctrlb = 0x00011213,
		.rfr_ctrl    = 0x0003de01,
		.mr	     = 0x00000032,
	},
	[3] = {
		.rate	     = 83000000,
		.actim_ctrla = 0x51512283,
		.actim_ctrlb = 0x0001120c,
		.rfr_ctrl    = 0x00025501,
		.mr	     = 0x00000032,
	},
	[4] = {
		.rate	     = 82970588,
		.actim_ctrla = 0x51512283,
		.actim_ctrlb = 0x0001120c,
		.rfr_ctrl    = 0x00025501,
		.mr	     = 0x00000032,
	},
	[5] = {
		.rate	     = 66666666,
		.actim_ctrla = 0x410d2243,
		.actim_ctrlb = 0x0001120a,
		.rfr_ctrl    = 0x0001d601,
		.mr	     = 0x00000032,
	},
	[6] = {
		.rate	     = 0
	},
};

#endif
