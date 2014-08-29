/*
 * Defines for the EMIF driver
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 *
 * Benoit Cousson (b-cousson@ti.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __EMIF_H
#define __EMIF_H

#include <linux/ti_emif.h>

#ifndef __ASSEMBLY__
/*
 * Structure containing shadow of important registers in EMIF
 * The calculation function fills in this structure to be later used for
 * initialisation and DVFS
 */
struct emif_regs {
	u32 freq;
	u32 ref_ctrl_shdw;
	u32 ref_ctrl_shdw_derated;
	u32 sdram_tim1_shdw;
	u32 sdram_tim1_shdw_derated;
	u32 sdram_tim2_shdw;
	u32 sdram_tim3_shdw;
	u32 sdram_tim3_shdw_derated;
	u32 pwr_mgmt_ctrl_shdw;
	union {
		u32 read_idle_ctrl_shdw_normal;
		u32 dll_calib_ctrl_shdw_normal;
	};
	union {
		u32 read_idle_ctrl_shdw_volt_ramp;
		u32 dll_calib_ctrl_shdw_volt_ramp;
	};

	u32 phy_ctrl_1_shdw;
	u32 ext_phy_ctrl_2_shdw;
	u32 ext_phy_ctrl_3_shdw;
	u32 ext_phy_ctrl_4_shdw;
};
#endif /* __ASSEMBLY__ */
#endif /* __EMIF_H */
