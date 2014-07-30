/*
 * OMAP clockdomain support
 *
 * Copyright (C) 2013 Texas Instruments, Inc.
 *
 * Tero Kristo <t-kristo@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk/ti.h>

#undef pr_fmt
#define pr_fmt(fmt) "%s: " fmt, __func__

struct clkdm_init_item {
	struct device_node *node;
	int index;
	struct list_head link;
};

static LIST_HEAD(retry_list);

static int of_ti_init_clk_clkdm(struct device_node *node, int index)
{
	struct clk *clk;
	struct clk_hw *clk_hw;

	clk = of_clk_get(node, index);

	if (IS_ERR_OR_NULL(clk)) {
		pr_debug("%s[%d] = %08x\n", node->name, index, (u32)clk);
		return -EBUSY;
	}

	if (__clk_get_flags(clk) & CLK_IS_BASIC) {
		pr_warn("can't setup clkdm for basic clk %s\n",
			__clk_get_name(clk));
		return -EINVAL;
	}

	clk_hw = __clk_get_hw(clk);
	to_clk_hw_omap(clk_hw)->clkdm_name = node->name;
	omap2_init_clk_clkdm(clk_hw);

	return 0;
}

static void __init of_ti_clockdomain_setup(struct device_node *node)
{
	int i;
	int num_clks;
	struct clkdm_init_item *retry;
	int ret;

	num_clks = of_count_phandle_with_args(node, "clocks", "#clock-cells");

	for (i = 0; i < num_clks; i++) {
		ret = of_ti_init_clk_clkdm(node, i);

		if (ret == -EBUSY) {
			retry = kzalloc(sizeof(*retry), GFP_KERNEL);
			if (!retry)
				return;
			retry->node = node;
			retry->index = i;
			list_add(&retry->link, &retry_list);
			continue;
		}
	}
}

static struct of_device_id ti_clkdm_match_table[] __initdata = {
	{ .compatible = "ti,clockdomain" },
	{ }
};

/**
 * ti_dt_clockdomains_setup - setup device tree clockdomains
 *
 * Initializes clockdomain nodes for a SoC. This parses through all the
 * nodes with compatible = "ti,clockdomain", and add the clockdomain
 * info for all the clocks listed under these. This function shall be
 * called after rest of the DT clock init has completed and all
 * clock nodes have been registered.
 */
void __init ti_dt_clockdomains_setup(struct device_node *node)
{
	struct device_node *np;
	struct device_node *clkdms;
	struct clkdm_init_item *retry, *tmp;
	int ret;

	clkdms = of_get_child_by_name(node, "clockdomains");
	if (!clkdms)
		return;

	list_for_each_entry_safe(retry, tmp, &retry_list, link) {
		pr_debug("retry-init: %s [%d]\n", retry->node->name,
			 retry->index);
		ret = of_ti_init_clk_clkdm(retry->node, retry->index);
		if (!ret) {
			list_del(&retry->link);
			kfree(retry);
		}
	}

	for_each_child_of_node(clkdms, np) {
		if (!of_match_node(ti_clkdm_match_table, np))
			continue;

		of_ti_clockdomain_setup(np);
	}
}
