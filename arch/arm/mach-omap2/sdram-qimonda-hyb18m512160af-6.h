/*
 * SDRC register values for the Qimonda HYB18M512160AF-6
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

#ifndef ARCH_ARM_MACH_OMAP2_SDRAM_QIMONDA_HYB18M512160AF6
#define ARCH_ARM_MACH_OMAP2_SDRAM_QIMONDA_HYB18M512160AF6

#include <mach/sdrc.h>

/* Qimonda HYB18M512160AF-6 */
/* XXX Using ARE = 0x1 (no autorefresh burst) -- can this be changed? */
static struct omap_sdrc_params hyb18m512160af6_sdrc_params[] = {
	[0] = {
		.rate	     = 165941176,
		.actim_ctrla = 0x629db4c6,
		.actim_ctrlb = 0x00012214,
		.rfr_ctrl    = 0x0004dc01,
		.mr	     = 0x00000032,
	},
	[1] = {
		.rate	     = 133333333,
		.actim_ctrla = 0x5219b485,
		.actim_ctrlb = 0x00012210,
		.rfr_ctrl    = 0x0003de01,
		.mr	     = 0x00000032,
	},
	[2] = {
		.rate	     = 82970588,
		.actim_ctrla = 0x31512283,
		.actim_ctrlb = 0x0001220a,
		.rfr_ctrl    = 0x00025501,
		.mr	     = 0x00000022,
	},
	[3] = {
		.rate	     = 66666666,
		.actim_ctrla = 0x290d2243,
		.actim_ctrlb = 0x00012208,
		.rfr_ctrl    = 0x0001d601,
		.mr	     = 0x00000022,
	},
	[4] = {
		.rate	     = 0
	},
};

#endif
