/*
 * AM33XX Power domains framework
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>

#include "powerdomain.h"
#include "prcm-common.h"
#include "prm33xx.h"
#include "prcm44xx.h"

static struct powerdomain gfx_33xx_pwrdm = {
	.name			= "gfx_pwrdm",
	.voltdm			= { .name = "core" },
	.prcm_partition		= AM33XX_PRM_PARTITION,
	.prcm_offs		= AM33XX_PRM_GFX_MOD,
	.pwrsts			= PWRSTS_OFF_ON,
	.pwrsts_logic_ret	= PWRSTS_OFF_RET,
	.pwrstctrl_offs		= AM33XX_PM_GFX_PWRSTCTRL_OFFSET,
	.pwrstst_offs		= AM33XX_PM_GFX_PWRSTST_OFFSET,
	.flags			= PWRDM_HAS_LOWPOWERSTATECHANGE,
	.banks			= 1,
	.pwrsts_mem_ret		= {
		[0]	= PWRSTS_OFF_RET,	/* gfx_mem */
	},
	.pwrsts_mem_on		= {
		[0]	= PWRSTS_ON,		/* gfx_mem */
	},
};

static struct powerdomain rtc_33xx_pwrdm = {
	.name			= "rtc_pwrdm",
	.voltdm			= { .name = "rtc" },
	.prcm_partition		= AM33XX_PRM_PARTITION,
	.prcm_offs		= AM33XX_PRM_RTC_MOD,
	.pwrsts			= PWRSTS_ON,
	.pwrstctrl_offs		= AM33XX_PM_RTC_PWRSTCTRL_OFFSET,
	.pwrstst_offs		= AM33XX_PM_RTC_PWRSTST_OFFSET,
};

static struct powerdomain wkup_33xx_pwrdm = {
	.name			= "wkup_pwrdm",
	.voltdm			= { .name = "core" },
	.prcm_partition		= AM33XX_PRM_PARTITION,
	.prcm_offs		= AM33XX_PRM_WKUP_MOD,
	.pwrsts			= PWRSTS_ON,
	.pwrstctrl_offs		= AM33XX_PM_WKUP_PWRSTCTRL_OFFSET,
	.pwrstst_offs		= AM33XX_PM_WKUP_PWRSTST_OFFSET,
};

static struct powerdomain per_33xx_pwrdm = {
	.name			= "per_pwrdm",
	.voltdm			= { .name = "core" },
	.prcm_partition		= AM33XX_PRM_PARTITION,
	.prcm_offs		= AM33XX_PRM_PER_MOD,
	.pwrsts			= PWRSTS_OFF_RET_ON,
	.pwrsts_logic_ret	= PWRSTS_OFF_RET,
	.pwrstctrl_offs		= AM33XX_PM_PER_PWRSTCTRL_OFFSET,
	.pwrstst_offs		= AM33XX_PM_PER_PWRSTST_OFFSET,
	.flags			= PWRDM_HAS_LOWPOWERSTATECHANGE,
	.banks			= 3,
	.pwrsts_mem_ret		= {
		[0]	= PWRSTS_OFF_RET,	/* pruss_mem */
		[1]	= PWRSTS_OFF_RET,	/* per_mem */
		[2]	= PWRSTS_OFF_RET,	/* ram_mem */
	},
	.pwrsts_mem_on		= {
		[0]	= PWRSTS_ON,		/* pruss_mem */
		[1]	= PWRSTS_ON,		/* per_mem */
		[2]	= PWRSTS_ON,		/* ram_mem */
	},
};

static struct powerdomain mpu_33xx_pwrdm = {
	.name			= "mpu_pwrdm",
	.voltdm			= { .name = "mpu" },
	.prcm_partition		= AM33XX_PRM_PARTITION,
	.prcm_offs		= AM33XX_PRM_MPU_MOD,
	.pwrsts			= PWRSTS_OFF_RET_ON,
	.pwrsts_logic_ret	= PWRSTS_OFF_RET,
	.pwrstctrl_offs		= AM33XX_PM_MPU_PWRSTCTRL_OFFSET,
	.pwrstst_offs		= AM33XX_PM_MPU_PWRSTST_OFFSET,
	.flags			= PWRDM_HAS_LOWPOWERSTATECHANGE,
	.banks			= 3,
	.pwrsts_mem_ret		= {
		[0]	= PWRSTS_OFF_RET,	/* mpu_l1 */
		[1]	= PWRSTS_OFF_RET,	/* mpu_l2 */
		[2]	= PWRSTS_OFF_RET,	/* mpu_ram */
	},
	.pwrsts_mem_on		= {
		[0]	= PWRSTS_ON,		/* mpu_l1 */
		[1]	= PWRSTS_ON,		/* mpu_l2 */
		[2]	= PWRSTS_ON,		/* mpu_ram */
	},
};

static struct powerdomain cefuse_33xx_pwrdm = {
	.name			= "cefuse_pwrdm",
	.voltdm			= { .name = "core" },
	.prcm_partition		= AM33XX_PRM_PARTITION,
	.prcm_offs		= AM33XX_PRM_CEFUSE_MOD,
	.pwrsts			= PWRSTS_OFF_ON,
	.pwrstctrl_offs		= AM33XX_PM_CEFUSE_PWRSTCTRL_OFFSET,
	.pwrstst_offs		= AM33XX_PM_CEFUSE_PWRSTST_OFFSET,
};

static struct powerdomain *powerdomains_am33xx[] __initdata = {
	&gfx_33xx_pwrdm,
	&rtc_33xx_pwrdm,
	&wkup_33xx_pwrdm,
	&per_33xx_pwrdm,
	&mpu_33xx_pwrdm,
	&cefuse_33xx_pwrdm,
	NULL,
};

void __init am33xx_powerdomains_init(void)
{
	pwrdm_register_platform_funcs(&omap4_pwrdm_operations);
	pwrdm_register_pwrdms(powerdomains_am33xx);
	pwrdm_complete_init();
}
