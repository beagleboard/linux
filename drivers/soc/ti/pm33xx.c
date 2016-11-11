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

#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_data/pm33xx.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/suspend.h>
#include <linux/ti-emif-sram.h>
#include <linux/wkup_m3_ipc.h>
#include <linux/rtc.h>

#include <asm/fncpy.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include <asm/system_misc.h>

#define RTC_SCRATCH_RESUME_REG	0
#define RTC_SCRATCH_MAGIC_REG	1
#define RTC_REG_BOOT_MAGIC	0x8cd0 /* RTC */
#define GIC_INT_SET_PENDING_BASE 0x200
#define AM43XX_GIC_DIST_BASE	0x48241000

static u32 rtc_magic_val;
static int (*am33xx_do_wfi_sram)(unsigned long unused);
static phys_addr_t am33xx_do_wfi_sram_phys;

static struct gen_pool *sram_pool;
static phys_addr_t ocmcram_location;

static struct rtc_device *omap_rtc;
static void __iomem *gic_dist_base;
static struct am33xx_pm_platform_data *pm_ops;
static struct am33xx_pm_sram_addr *pm_sram;

static struct wkup_m3_ipc *m3_ipc;

#ifdef CONFIG_SUSPEND
static unsigned long suspend_wfi_flags;
static int rtc_only_idle;
static int retrigger_irq;

static struct wkup_m3_wakeup_src wakeup_src = {.irq_nr = 0,
	.src = "Unknown",
};

static struct wkup_m3_wakeup_src rtc_alarm_wakeup = {
	.irq_nr = 108, .src = "RTC Alarm",
};

static struct wkup_m3_wakeup_src rtc_ext_wakeup = {
	.irq_nr = 0, .src = "Ext wakeup",
};
#endif /* CONFIG_SUSPEND */

static int am33xx_do_sram_idle(u32 wfi_flags)
{
	int ret = 0;

	if (!m3_ipc || !pm_ops)
		return 0;

	if (wfi_flags & WFI_FLAG_WAKE_M3)
		ret = m3_ipc->ops->prepare_low_power(m3_ipc, WKUP_M3_IDLE);

	return pm_ops->cpu_suspend(am33xx_do_wfi_sram, wfi_flags);
}

/*
 * Push the minimal suspend-resume code to SRAM
 */
static int am33xx_prepare_push_sram_idle(void)
{
	struct device_node *np;
	int ret;

	ret = ti_emif_copy_pm_function_table(pm_sram->emif_sram_table);
	if (ret) {
		pr_err("PM: %s: EMIF function copy failed\n", __func__);
		return -EPROBE_DEFER;
	}

	np = of_find_compatible_node(NULL, NULL, "ti,omap3-mpu");

	if (!np) {
		np = of_find_compatible_node(NULL, NULL, "ti,omap4-mpu");
		if (!np) {
			pr_warn("PM: %s: Unable to find device node for mpu\n",
				__func__);
			return -ENODEV;
		}
	}

	sram_pool = of_gen_pool_get(np, "sram", 0);

	if (!sram_pool) {
		pr_warn("PM: %s: Unable to get sram pool for ocmcram\n",
			__func__);
		return -ENODEV;
	}

	ocmcram_location = gen_pool_alloc(sram_pool, *pm_sram->do_wfi_sz);
	if (!ocmcram_location) {
		pr_warn("PM: %s: Unable to allocate memory from ocmcram\n",
			__func__);
		return -EINVAL;
	}

	/* Save physical address to calculate resume offset during pm init */
	am33xx_do_wfi_sram_phys = gen_pool_virt_to_phys(sram_pool,
							ocmcram_location);

	return 0;
}

/*
 * Push the minimal suspend-resume code to SRAM
 */
int am33xx_push_sram_idle(void)
{
	am33xx_do_wfi_sram = (void *)fncpy((void *)ocmcram_location,
						pm_sram->do_wfi,
						*pm_sram->do_wfi_sz);

	return 0;
}

static int __init am43xx_map_gic(void)
{
	gic_dist_base = ioremap(AM43XX_GIC_DIST_BASE, SZ_4K);

	if (!gic_dist_base)
		return -ENOMEM;

	return 0;
}

#ifdef CONFIG_SUSPEND
struct wkup_m3_wakeup_src rtc_wake_src(void)
{
	u32 i;

	i = __raw_readl(pm_ops->get_rtc_base_addr() + 0x44) & 0x40;

	if (i) {
		retrigger_irq = rtc_alarm_wakeup.irq_nr;
		return rtc_alarm_wakeup;
	}

	retrigger_irq = rtc_ext_wakeup.irq_nr;

	return rtc_ext_wakeup;
}

int am33xx_rtc_only_idle(unsigned long wfi_flags)
{
	rtc_power_off_program(omap_rtc);
	am33xx_do_wfi_sram(wfi_flags);
	return 0;
}

static int am33xx_pm_suspend(suspend_state_t suspend_state)
{
	int i, ret = 0;

	if (suspend_state == PM_SUSPEND_MEM &&
	    pm_ops->check_off_mode_enable()) {
		pm_ops->prepare_rtc_suspend();
		pm_ops->save_context();
		suspend_wfi_flags |= WFI_FLAG_RTC_ONLY;
		ret = pm_ops->soc_suspend(suspend_state, am33xx_rtc_only_idle,
					  suspend_wfi_flags);

		suspend_wfi_flags &= ~WFI_FLAG_RTC_ONLY;

		if (!ret) {
			pm_ops->restore_context();
			m3_ipc->ops->set_rtc_only(m3_ipc);
			am33xx_push_sram_idle();
		}
	} else {
		ret = pm_ops->soc_suspend(suspend_state, am33xx_do_wfi_sram,
				  suspend_wfi_flags);
	}

	if (ret) {
		pr_err("PM: Kernel suspend failure\n");
	} else {
		i = m3_ipc->ops->request_pm_status(m3_ipc);

		switch (i) {
		case 0:
			pr_info("PM: Successfully put all powerdomains to target state\n");
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
		if (rtc_only_idle) {
		wakeup_src = rtc_wake_src();
			pr_info("PM: Wakeup source %s\n", wakeup_src.src);
		}
	}

	if (suspend_state == PM_SUSPEND_MEM && pm_ops->check_off_mode_enable())
		pm_ops->prepare_rtc_resume();

	return ret;
}

static int am33xx_pm_enter(suspend_state_t suspend_state)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_MEM:
	case PM_SUSPEND_STANDBY:
		ret = am33xx_pm_suspend(suspend_state);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int am33xx_pm_begin(suspend_state_t state)
{
	int ret = -EINVAL;

	if (state == PM_SUSPEND_MEM && pm_ops->check_off_mode_enable()) {
		rtc_write_scratch(omap_rtc, RTC_SCRATCH_MAGIC_REG,
				  rtc_magic_val);
		rtc_only_idle = 1;
	} else {
		rtc_only_idle = 0;
	}

	cpu_idle_poll_ctrl(true);

	switch (state) {
	case PM_SUSPEND_MEM:
		ret = m3_ipc->ops->prepare_low_power(m3_ipc, WKUP_M3_DEEPSLEEP);
		break;
	case PM_SUSPEND_STANDBY:
		ret = m3_ipc->ops->prepare_low_power(m3_ipc, WKUP_M3_STANDBY);
		break;
	}

	return ret;
}

static void am33xx_pm_end(void)
{
	m3_ipc->ops->finish_low_power(m3_ipc);

	if (rtc_only_idle) {
		if (retrigger_irq)
			/*
			 * 32 bits of Interrupt Set-Pending correspond to 32
			 * 32 interupts. Compute the bit offset of the
			 * Interrupt and set that particular bit.
			 * Compute the register offset by dividing interrupt
			 * number by 32 and mutiplying by 4
			 */
			writel_relaxed(1 << (retrigger_irq & 31),
				       gic_dist_base + GIC_INT_SET_PENDING_BASE
				       + retrigger_irq / 32 * 4);
		rtc_write_scratch(omap_rtc, RTC_SCRATCH_MAGIC_REG, 0);
	}

	rtc_only_idle = 0;

	cpu_idle_poll_ctrl(false);
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

static void am33xx_pm_set_ipc_ops(void)
{
	void *resume_address;
	u32 temp;

	temp = ti_emif_get_mem_type();
	if (temp < 0) {
		pr_err("PM: Cannot determine memory type, no PM available\n");
		return;
	}
	m3_ipc->ops->set_mem_type(m3_ipc, temp);

	/* Physical resume address to be used by ROM code */
	resume_address = (void *)am33xx_do_wfi_sram_phys +
			 *pm_sram->resume_offset + 0x4;

	m3_ipc->ops->set_resume_address(m3_ipc, resume_address);
}

static int am33xx_pm_rtc_setup(void)
{
	struct device_node *np;

	np = of_find_node_by_name(NULL, "rtc");

	if (of_device_is_available(np)) {
		omap_rtc = rtc_class_open("rtc0");
		if (!omap_rtc) {
			pr_warn("PM: rtc0 not available");
			return -EPROBE_DEFER;
		}

		rtc_read_scratch(omap_rtc, RTC_SCRATCH_MAGIC_REG,
				 &rtc_magic_val);

		if ((rtc_magic_val & 0xffff) != RTC_REG_BOOT_MAGIC)
			pr_warn("PM: bootloader does not support rtc-only!\n");

		pm_sram->rtc_base_virt = pm_ops->get_rtc_base_addr();
		rtc_write_scratch(omap_rtc, RTC_SCRATCH_MAGIC_REG, 0);
		rtc_write_scratch(omap_rtc, RTC_SCRATCH_RESUME_REG,
				  pm_sram->rtc_resume_phys_addr);
	} else {
		pr_warn("PM: no-rtc available, rtc-only mode disabled.\n");
	}

	return 0;
}

static int am33xx_pm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	if (!of_machine_is_compatible("ti,am33xx") &&
	    !of_machine_is_compatible("ti,am43"))
		return -ENODEV;

	pm_ops = dev->platform_data;
	if (!pm_ops) {
		pr_err("PM: Cannot get core PM ops!\n");
		return -ENODEV;
	}

	ret = am43xx_map_gic();
	if (ret) {
		pr_err("PM: Could not ioremap GIC base\n");
		return ret;
	}

	pm_sram = pm_ops->pm_sram_addr;
	if (!pm_sram) {
		pr_err("PM: Cannot get PM asm function addresses!!\n");
		return -ENODEV;
	}

	m3_ipc = wkup_m3_ipc_get();
	if (!m3_ipc) {
		pr_err("PM: Cannot get wkup_m3_ipc handle\n");
		return -EPROBE_DEFER;
	}

	ret = am33xx_prepare_push_sram_idle();
	if (ret)
		return ret;

	ret = am33xx_pm_rtc_setup();
	if (ret)
		goto err_free_sram;

	am33xx_push_sram_idle();

	am33xx_pm_set_ipc_ops();

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&am33xx_pm_ops);

	/*
	 * For a system suspend we must flush the caches, we want
	 * the DDR in self-refresh, we want to save the context
	 * of the EMIF, and we want the wkup_m3 to handle low-power
	 * transition.
	 */
	suspend_wfi_flags |= WFI_FLAG_FLUSH_CACHE;
	suspend_wfi_flags |= WFI_FLAG_SELF_REFRESH;
	suspend_wfi_flags |= WFI_FLAG_SAVE_EMIF;
	suspend_wfi_flags |= WFI_FLAG_WAKE_M3;
#endif /* CONFIG_SUSPEND */

	ret = pm_ops->init(am33xx_do_sram_idle);
	if (ret) {
		pr_err("Unable to call core pm init!\n");
		ret = -ENODEV;
		goto err_put_wkup_m3_ipc;
	}

	return 0;

err_put_wkup_m3_ipc:
	wkup_m3_ipc_put(m3_ipc);
err_free_sram:
	gen_pool_free(sram_pool, ocmcram_location, *pm_sram->do_wfi_sz);

	return ret;
}

static int am33xx_pm_remove(struct platform_device *pdev)
{
	if (pm_ops->deinit)
		pm_ops->deinit();
	suspend_set_ops(NULL);
	wkup_m3_ipc_put(m3_ipc);
	gen_pool_free(sram_pool, ocmcram_location, *pm_sram->do_wfi_sz);
	return 0;
}

static struct platform_driver am33xx_pm_driver = {
	.driver = {
		.name   = "pm33xx",
	},
	.probe = am33xx_pm_probe,
	.remove = am33xx_pm_remove,
};

module_platform_driver(am33xx_pm_driver);

MODULE_ALIAS("platform:pm33xx");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("am33xx power management driver");
