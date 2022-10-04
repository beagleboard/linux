/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2022 Alibaba Group Holding Limited.
 */
#include <asm/cpuidle.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/suspend.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/types.h>

#undef pr_fmt
#define pr_fmt(fmt) "light-system-suspend" ": " fmt

#ifdef CONFIG_PLIC_INT_CLEAR
void __iomem *hart0_sbase;
/*
 * Each hart context has a set of control registers associated with it.  Right
 * now there's only two: a source priority threshold over which the hart will
 * take an interrupt, and a register to claim interrupts.
 */
#define CONTEXT_BASE			0x200000
#define CONTEXT_PER_HART		0x1000
#define CONTEXT_THRESHOLD		0x00
#define CONTEXT_CLAIM			0x04
#endif

static int light_suspend_prepare_late(void)
{
#ifdef CONFIG_PLIC_INT_CLEAR
	void __iomem *claim = hart0_sbase + CONTEXT_CLAIM;
	irq_hw_number_t hwirq;

	pr_debug("clear plic pending interrupt\n");
	pr_debug("claim base = 0x%lx\n", (unsigned long)claim);

	while ((hwirq = readl(claim))) {
		pr_debug("claim hwirq%ld\n", hwirq);
		writel(hwirq, claim);
	}
#endif
	/*
	 * Two-level switch for interrupt control: it needs to disable interrupts in SIE, not just in SSTATUS,
	 * otherwise the pending interrupts will wakup the cpu immediately after wfi.
	 */
	csr_write(CSR_IE, 0);

	return 0;
}

static int light_suspend_enter(suspend_state_t state)
{
	if (!IS_ENABLED(CONFIG_PM))
		return 0;

	switch (state) {
	case PM_SUSPEND_MEM:
		pr_debug("enter platform system suspend...\n");
		cpu_do_idle();
		pr_debug("wakeup from wfi...\n");
	break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct platform_suspend_ops light_suspend_ops = {
	.enter = light_suspend_enter,
	.valid = suspend_valid_only_mem,
	.prepare_late = light_suspend_prepare_late,
};

static int __init pm_light_init(void)
{
#ifdef CONFIG_PLIC_INT_CLEAR
	struct device_node *np;
	void __iomem *regs;

	np = of_find_node_by_path("/soc/interrupt-controller@ffd8000000");
	if (!np) {
		pr_err("no plic interrupt controller found\n");
		return -EINVAL;
	}

	regs = of_iomap(np, 0);
	if (WARN_ON(!regs)) {
		pr_err("failed to ioremap interrupt regs\n");
		return -EIO;
	}

	hart0_sbase = regs + CONTEXT_BASE + 1 * CONTEXT_PER_HART; /* 1 means s mode */

	pr_debug("hart0_sbase = 0x%lx\n", (unsigned long)hart0_sbase);
#endif

	pr_info("set system suspend platform callbacks\n");

	suspend_set_ops(&light_suspend_ops);

	return 0;
}

late_initcall(pm_light_init);
