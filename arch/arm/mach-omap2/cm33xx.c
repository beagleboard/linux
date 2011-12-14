/*
 * AM33XX CM module functions
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Based on arch/arm/mach-omap2/cm4xxx.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/io.h>

#include "common.h"

#include "cm.h"
#include "cm-regbits-33xx.h"
#include "cm33xx.h"

/**
 * am33xx_cm_wait_module_ready - wait for a module to be in 'func' state
 * @inst: Offset of CM instance associated with
 * @clkctrl_reg: CLKCTRL offset from CM instance base
 *
 * Wait for the module IDLEST to be functional. If the idle state is in any
 * the non functional state (trans, idle or disabled), module and thus the
 * sysconfig cannot be accessed and will probably lead to an "imprecise
 * external abort"
 *
 * Module idle state:
 *   0x0 func:     Module is fully functional, including OCP
 *   0x1 trans:    Module is performing transition: wakeup, or sleep, or sleep
 *                 abortion
 *   0x2 idle:     Module is in Idle mode (only OCP part). It is functional if
 *                 using separate functional clock
 *   0x3 disabled: Module is disabled and cannot be accessed
 *
 */
int am33xx_cm_wait_module_ready(u16 inst, u16 clkctrl_reg)
{
	int i = 0;

	omap_test_timeout((
		((__raw_readl(AM33XX_CM_REGADDR(inst, clkctrl_reg)) &
		  AM33XX_IDLEST_MASK) == 0)), MAX_MODULE_READY_TIME, i);

	return (i < MAX_MODULE_READY_TIME) ? 0 : -EBUSY;
}
