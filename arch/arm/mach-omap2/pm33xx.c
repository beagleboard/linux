/*
 * AM33XX Power Management Routines
 *
 * Copyright (C) 2012-2014 Texas Instruments Incorporated - http://www.ti.com/
 *	Vaibhav Bedia, Dave Gerlach
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
#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ti_emif.h>
#include <linux/omap-mailbox.h>
#include <linux/sizes.h>

#include <asm/suspend.h>
#include <asm/proc-fns.h>
#include <asm/fncpy.h>
#include <asm/system_misc.h>
#include <asm/smp_scu.h>

#include "control.h"
#include "pm.h"
#include "cm33xx.h"
#include "pm33xx.h"
#include "common.h"
#include "clockdomain.h"
#include "powerdomain.h"
#include "soc.h"
#include "sram.h"

#ifdef CONFIG_SUSPEND
static void __iomem *scu_base;
static struct powerdomain *cefuse_pwrdm, *gfx_pwrdm, *per_pwrdm, *mpu_pwrdm;
static struct clockdomain *gfx_l4ls_clkdm;
#endif /* CONFIG_SUSPEND */

#ifdef CONFIG_CPU_PM
static void __iomem *am33xx_emif_base;
static struct am33xx_pm_context *am33xx_pm;

static DECLARE_COMPLETION(am33xx_pm_sync);

static void (*am33xx_do_wfi_sram)(struct am33xx_suspend_params *);

static struct am33xx_suspend_params susp_params;

int am33xx_do_sram_cpuidle(u32 wfi_flags, u32 m3_flags)
{
	struct am33xx_suspend_params params;
	int ret;

	/* Start with the default flags */
	memcpy(&params, &susp_params, sizeof(params));

	/* Clear bits configurable through this call */
	params.wfi_flags &= ~(WFI_SELF_REFRESH | WFI_WAKE_M3 | WFI_SAVE_EMIF |
							WFI_DISABLE_EMIF);

	/* Don't enter these states if the M3 isn't available */
	if (am33xx_pm->state != M3_STATE_INITED)
		wfi_flags &= ~WFI_WAKE_M3;

	/* Set bits that have been passed */
	params.wfi_flags |= wfi_flags;

	if (wfi_flags & WFI_WAKE_M3) {
		am33xx_pm->ipc.reg1 = IPC_CMD_IDLE;
		am33xx_pm->ipc.reg2 = DS_IPC_DEFAULT;
		am33xx_pm->ipc.reg3 = m3_flags;
		am33xx_pm->ipc.reg5 = DS_IPC_DEFAULT;
		wkup_m3_set_cmd(&am33xx_pm->ipc);
		ret = wkup_m3_ping();
		if (ret < 0)
			return ret;
	}

	am33xx_do_wfi_sram(&params);
	return 0;
}

#ifdef CONFIG_SUSPEND
static int am33xx_do_sram_idle(unsigned long int arg)
{
	am33xx_do_wfi_sram((struct am33xx_suspend_params *)arg);
	return 0;
}

static int am33xx_pm_suspend(unsigned int state)
{
	int i, ret = 0;
	int status = 0;
	struct wkup_m3_wakeup_src wakeup_src = {.irq_nr = 0,
						.src = "Unknown",};

	omap_set_pwrdm_state(gfx_pwrdm, PWRDM_POWER_OFF);

	am33xx_pm->ops->pre_suspend(state);

	ret = cpu_suspend((unsigned long int)&susp_params,
			  am33xx_do_sram_idle);

	/*
	 * Because gfx_pwrdm is the only one under MPU control,
	 * comment on transition status
	 */
	status = pwrdm_read_pwrst(gfx_pwrdm);
	if (status != PWRDM_POWER_OFF)
		pr_err("PM: GFX domain did not transition\n");

	am33xx_pm->ops->post_suspend(state);

	if (ret) {
		pr_err("PM: Kernel suspend failure\n");
	} else {
		i = wkup_m3_pm_status();

		switch (i) {
		case 0:
			pr_info("PM: Successfully put all powerdomains to target state\n");

			/*
			 * The PRCM registers on AM335x do not contain
			 * previous state information like those present on
			 * OMAP4 so we must manually indicate transition so
			 * state counters are properly incremented
			 */
			pwrdm_post_transition(mpu_pwrdm);
			pwrdm_post_transition(per_pwrdm);
			break;
		case 1:
			pr_err("PM: Could not transition all powerdomains to target state\n");
			ret = -1;
			break;
		default:
			pr_err("PM: CM3 returned unknown result = %d\n", i);
			ret = -1;
		}
		/* print the wakeup reason */
		wkup_m3_wake_src(&wakeup_src);

		pr_info("PM: Wakeup source %s\n", wakeup_src.src);
	}

	return ret;
}

static int am33xx_pm_enter(suspend_state_t suspend_state)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = am33xx_pm_suspend(suspend_state);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}


static void am33xx_m3_state_machine_reset(void)
{
	int i;

	am33xx_pm->ipc.reg1 = IPC_CMD_RESET;

	wkup_m3_set_cmd(&am33xx_pm->ipc);

	am33xx_pm->state = M3_STATE_MSG_FOR_RESET;

	if (!wkup_m3_ping()) {
		i = wait_for_completion_timeout(&am33xx_pm_sync,
						msecs_to_jiffies(500));
		if (!i) {
			WARN(1, "PM: MPU<->CM3 sync failure\n");
			am33xx_pm->state = M3_STATE_UNKNOWN;
		}
	} else {
		pr_warn("PM: Unable to ping CM3\n");
	}
}

static int am33xx_pm_begin(suspend_state_t state)
{
	int i;


	switch (state) {
	case PM_SUSPEND_MEM:
		am33xx_pm->ipc.reg1	= IPC_CMD_DS0;
		break;
	case PM_SUSPEND_STANDBY:
		am33xx_pm->ipc.reg1	= IPC_CMD_STANDBY;
		break;
	}

	am33xx_pm->ipc.reg2		= DS_IPC_DEFAULT;
	am33xx_pm->ipc.reg3		= DS_IPC_DEFAULT;

	wkup_m3_set_cmd(&am33xx_pm->ipc);

	am33xx_pm->state = M3_STATE_MSG_FOR_LP;

	if (!wkup_m3_ping()) {
		i = wait_for_completion_timeout(&am33xx_pm_sync,
						msecs_to_jiffies(500));
		if (!i) {
			WARN(1, "PM: MPU<->CM3 sync failure\n");
			return -1;
		}
	} else {
		pr_warn("PM: Unable to ping CM3\n");
		return -1;
	}

	return 0;
}

static void am33xx_pm_end(void)
{
	am33xx_m3_state_machine_reset();
}

static int am33xx_pm_valid(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		return 1;
	default:
		return 0;
	}
}

static const struct platform_suspend_ops am33xx_pm_ops = {
	.begin		= am33xx_pm_begin,
	.end		= am33xx_pm_end,
	.enter		= am33xx_pm_enter,
	.valid		= am33xx_pm_valid,
};
#endif /* CONFIG_SUSPEND */

static void am33xx_txev_handler(void)
{
	switch (am33xx_pm->state) {
	case M3_STATE_RESET:
		am33xx_pm->state = M3_STATE_INITED;
		complete(&am33xx_pm_sync);
		break;
	case M3_STATE_MSG_FOR_RESET:
		am33xx_pm->state = M3_STATE_INITED;
		complete(&am33xx_pm_sync);
		break;
	case M3_STATE_MSG_FOR_LP:
		complete(&am33xx_pm_sync);
		break;
	case M3_STATE_UNKNOWN:
		pr_warn("PM: Unknown CM3 State\n");
	}
}

static void am33xx_m3_ready_cb(void)
{
	am33xx_pm->ver = wkup_m3_fw_version_read();

	if (am33xx_pm->ver == M3_VERSION_UNKNOWN ||
	    am33xx_pm->ver < M3_BASELINE_VERSION) {
		pr_warn("PM: CM3 Firmware Version %x not supported\n",
			am33xx_pm->ver);
		return;
	} else {
		pr_info("PM: CM3 Firmware Version = 0x%x\n",
			am33xx_pm->ver);
	}

	if (soc_is_am33xx())
		am33xx_idle_init(susp_params.wfi_flags & WFI_MEM_TYPE_DDR3);
	else if (soc_is_am437x())
		am437x_idle_init();

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&am33xx_pm_ops);
#endif /* CONFIG_SUSPEND */
}

static struct wkup_m3_ops am33xx_wkup_m3_ops = {
	.txev_handler = am33xx_txev_handler,
	.rproc_ready = am33xx_m3_ready_cb,
};

/*
 * Push the minimal suspend-resume code to SRAM
 */
#ifdef CONFIG_SOC_AM33XX
void am33xx_push_sram_idle(void)
{
	am33xx_do_wfi_sram = (void *)omap_sram_push
					(am33xx_do_wfi, am33xx_do_wfi_sz);
}
#endif

#ifdef CONFIG_SOC_AM43XX
void am43xx_push_sram_idle(void)
{
	am33xx_do_wfi_sram = (void *)omap_sram_push
					(am43xx_do_wfi, am43xx_do_wfi_sz);
}
#endif

static int __init am33xx_map_emif(void)
{
	am33xx_emif_base = ioremap(AM33XX_EMIF_BASE, SZ_32K);

	if (!am33xx_emif_base)
		return -ENOMEM;

	return 0;
}

#ifdef CONFIG_SUSPEND
static int __init am43xx_map_scu(void)
{
	scu_base = ioremap(scu_a9_get_base(), SZ_256);

	if (!scu_base)
		return -ENOMEM;

	return 0;
}


static int am33xx_suspend_init(void)
{
	u32 temp;

	gfx_l4ls_clkdm = clkdm_lookup("gfx_l4ls_gfx_clkdm");

	if (!gfx_l4ls_clkdm) {
		pr_err("PM: Cannot lookup gfx_l4ls_clkdm clockdomains\n");
		return -ENODEV;
	}

	/* Physical resume address to be used by ROM code */
	am33xx_pm->ipc.reg0 = (AM33XX_OCMC_END -
		am33xx_do_wfi_sz + am33xx_resume_offset + 0x4);

	/*
	 * Save SDRAM config in shadow register.
	 * When the EMIF gets powered back up, its SDRAM_CONFIG register gets
	 * loaded from the SECURE_SDRAM_CONFIG register.
	 */
	temp = readl(am33xx_emif_base + EMIF_SDRAM_CONFIG);
	omap_ctrl_writel(temp, AM33XX_CONTROL_SECURE_SDRAM_CONFIG);

	return 0;
}

static int am43xx_suspend_init(void)
{
	int ret = 0;

	ret = am43xx_map_scu();
	if (ret) {
			pr_err("PM: Could not ioremap SCU\n");
			return ret;
	}

	susp_params.l2_base_virt = omap4_get_l2cache_base();

	if (!susp_params.l2_base_virt) {
		pr_err("PM: Could not get l2 cache base address\n");
		return -ENOMEM;
	}

	susp_params.cke_override_virt =
		ioremap(AM43XX_CTRL_CKE_OVERRIDE, SZ_4);

	if (!susp_params.cke_override_virt) {
		pr_err("PM: Could not ioremap CKE override in Control Module\n");
		return -ENOMEM;
	}

	/* Physical resume address to be used by ROM code */
	am33xx_pm->ipc.reg0 = (AM33XX_OCMC_END -
		am43xx_do_wfi_sz + am43xx_resume_offset + 0x4);

	return ret;
}

static void am33xx_pre_suspend(unsigned int state)
{
	return;
}

static void am43xx_pre_suspend(unsigned int state)
{
	scu_power_mode(scu_base, SCU_PM_POWEROFF);
}

static void am33xx_post_suspend(unsigned int state)
{
	/*
	 * BUG: GFX_L4LS clock domain needs to be woken up to
	 * ensure thet L4LS clock domain does not get stuck in
	 * transition. If that happens L3 module does not get
	 * disabled, thereby leading to PER power domain
	 * transition failing
	 */
	clkdm_wakeup(gfx_l4ls_clkdm);
	clkdm_sleep(gfx_l4ls_clkdm);
}

static void am43xx_post_suspend(unsigned int state)
{
	scu_power_mode(scu_base, SCU_PM_NORMAL);
}

static struct am33xx_pm_ops am33xx_ops = {
	.init = am33xx_suspend_init,
	.pre_suspend = am33xx_pre_suspend,
	.post_suspend = am33xx_post_suspend,
};

static struct am33xx_pm_ops am43xx_ops = {
	.init = am43xx_suspend_init,
	.pre_suspend = am43xx_pre_suspend,
	.post_suspend = am43xx_post_suspend,
};
#endif /* CONFIG_SUSPEND */
#endif /* CONFIG_CPU_PM */

int __init am33xx_pm_init(void)
{
#ifdef CONFIG_CPU_PM
	int ret;
	u32 temp;
	struct device_node *np;
#endif /* CONFIG_CPU_PM */

	if (!soc_is_am33xx() && !soc_is_am43xx())
		return -ENODEV;

#ifdef CONFIG_CPU_PM
	am33xx_pm = kzalloc(sizeof(*am33xx_pm), GFP_KERNEL);
	if (!am33xx_pm) {
		pr_err("Memory allocation failed\n");
		ret = -ENOMEM;
		return ret;
	}

	ret = am33xx_map_emif();
	if (ret) {
		pr_err("PM: Could not ioremap EMIF\n");
		goto err;
	}

#ifdef CONFIG_SUSPEND
	gfx_pwrdm = pwrdm_lookup("gfx_pwrdm");
	per_pwrdm = pwrdm_lookup("per_pwrdm");
	mpu_pwrdm = pwrdm_lookup("mpu_pwrdm");

	if ((!gfx_pwrdm) || (!per_pwrdm) || (!mpu_pwrdm))
		return -ENODEV;

	/*
	 * Code paths for each SoC are nearly the same but set ops
	 * handle differences during init, pre-suspend, and post-suspend
	 */

	if (soc_is_am33xx())
		am33xx_pm->ops = &am33xx_ops;
	else if (soc_is_am43xx())
		am33xx_pm->ops = &am43xx_ops;

	ret = am33xx_pm->ops->init();

	if (ret)
		goto err;
#endif /* CONFIG_SUSPEND */

	/* Determine Memory Type */
	temp = readl(am33xx_emif_base + EMIF_SDRAM_CONFIG);
	temp = (temp & SDRAM_TYPE_MASK) >> SDRAM_TYPE_SHIFT;
	/* Parameters to pass to assembly code */
	susp_params.wfi_flags = 0;
	susp_params.emif_addr_virt = am33xx_emif_base;
	susp_params.dram_sync = am33xx_dram_sync;

	switch (temp) {
	case MEM_TYPE_DDR2:
		susp_params.wfi_flags |= WFI_MEM_TYPE_DDR2;
		break;
	case MEM_TYPE_DDR3:
		susp_params.wfi_flags |= WFI_MEM_TYPE_DDR3;
		break;
	}
	susp_params.wfi_flags |= WFI_SELF_REFRESH;
	susp_params.wfi_flags |= WFI_SAVE_EMIF;
	susp_params.wfi_flags |= WFI_DISABLE_EMIF;
	susp_params.wfi_flags |= WFI_WAKE_M3;

	am33xx_pm->ipc.reg4 = temp & MEM_TYPE_MASK;

	np = of_find_compatible_node(NULL, NULL, "ti,am3353-wkup-m3");
	if (np) {
		if (of_find_property(np, "ti,needs-vtt-toggle", NULL) &&
		    (!(of_property_read_u32(np, "ti,vtt-gpio-pin",
							&temp)))) {
			if (temp >= 0 && temp <= 31)
				am33xx_pm->ipc.reg4 |=
					((1 << VTT_STAT_SHIFT) |
					(temp << VTT_GPIO_PIN_SHIFT));
			else
				pr_warn("PM: Invalid VTT GPIO(%d) pin\n", temp);
		}

		if (of_find_property(np, "ti,set-io-isolation", NULL))
			am33xx_pm->ipc.reg4 |= (1 << IO_ISOLATION_STAT_SHIFT);
	}
#endif /* CONFIG_CPU_PM */

	(void) clkdm_for_each(omap_pm_clkdms_setup, NULL);

	/* CEFUSE domain can be turned off post bootup */
	cefuse_pwrdm = pwrdm_lookup("cefuse_pwrdm");
	if (cefuse_pwrdm)
		omap_set_pwrdm_state(cefuse_pwrdm, PWRDM_POWER_OFF);
	else
		pr_err("PM: Failed to get cefuse_pwrdm\n");

#ifdef CONFIG_CPU_PM
	am33xx_pm->state = M3_STATE_RESET;

	wkup_m3_set_ops(&am33xx_wkup_m3_ops);

	return 0;

err:
	kfree(am33xx_pm);
	return ret;
#endif /* CONFIG_CPU_PM */
}
