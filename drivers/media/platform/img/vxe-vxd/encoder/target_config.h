/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Device specific memory configuration
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

#ifndef __TARGET_CONFIG_H__
#define __TARGET_CONFIG_H__

#include "target.h"

/* Order MUST match with topaz_mem_space definition */
enum topaz_mem_space_idx {
	REG_TOPAZHP_MULTICORE = 0,
	REG_DMAC,
	REG_COMMS,
	REG_MTX,
	REG_MMU,
	REG_TOPAZHP_TEST,
	REG_MTX_RAM,
	REG_TOPAZHP_CORE_0,
	REG_TOPAZHP_VLC_CORE_0,
	REG_TOPAZHP_DEBLOCKER_CORE_0,
	REG_TOPAZHP_COREEXT_0,
	REG_TOPAZHP_CORE_1,
	REG_TOPAZHP_VLC_CORE_1,
	REG_TOPAZHP_DEBLOCKER_CORE_1,
	REG_TOPAZHP_COREEXT_1,
	REG_TOPAZHP_CORE_2,
	REG_TOPAZHP_VLC_CORE_2,
	REG_TOPAZHP_DEBLOCKER_CORE_2,
	REG_TOPAZHP_COREEXT_2,
	REG_TOPAZHP_CORE_3,
	REG_TOPAZHP_VLC_CORE_3,
	REG_TOPAZHP_DEBLOCKER_CORE_3,
	REG_TOPAZHP_COREEXT_3,
	FW,
	SYSMEM,
	MEMSYSMEM,
	MEM,
	FB,
	MEMDMAC_00,
	MEMDMAC_01,
	MEMDMAC_02,
	MEM_SPACE_FORCE32BITS = 0x7FFFFFFFU
};

#endif
