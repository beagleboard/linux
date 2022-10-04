/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2022 Alibaba Group Holding Limited.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include "clk-gate.h"

#define to_thead_clk_gate(_hw)	container_of(_hw, struct thead_clk_gate, hw)

static int thead_clk_gate_is_enabled(struct clk_hw *hw)
{
	struct thead_clk_gate *tcg = to_thead_clk_gate(hw);
	u32 val;

	regmap_read(tcg->regmap, tcg->offset, &val);

	val &= BIT(tcg->bit);

	return val != 0;
}

static void thead_clk_gate_disable(struct clk_hw *hw)
{
	struct thead_clk_gate *tcg = to_thead_clk_gate(hw);

	if (!tcg->shared)
		goto out;

	if (tcg->share_count) {
		if (WARN_ON(*tcg->share_count == 0))
			return;
		else if (--(*tcg->share_count) > 0) {
			pr_info("[%s,%d]share_count = %d\n", __func__, __LINE__,
					(*tcg->share_count));
			return;
		}
	}

out:
	regmap_update_bits(tcg->regmap, tcg->offset,
			   BIT(tcg->bit), 0);
}

static int thead_clk_gate_enable(struct clk_hw *hw)
{
	struct thead_clk_gate *tcg = to_thead_clk_gate(hw);

	if (!tcg->shared)
		goto out;

	if (tcg->share_count && (*tcg->share_count)++ > 0) {
		pr_info("[%s,%d]share_count = %d\n", __func__, __LINE__, (*tcg->share_count));
		return 0;
	}

out:
	return regmap_update_bits(tcg->regmap, tcg->offset,
				  BIT(tcg->bit), BIT(tcg->bit));
}

const struct clk_ops thead_gate_clk_ops = {
	.enable = thead_clk_gate_enable,
	.disable = thead_clk_gate_disable,
	.is_enabled = thead_clk_gate_is_enabled,
};

struct clk *thead_gate_clk_register(const char *name,
				    const char *parent_name,
				    struct regmap *regmap,
				    int offset,
				    u8 bit,
				    bool shared,
				    u32 *share_count,
				    struct device *dev)
{
	struct thead_clk_gate *tcg;
	struct clk *clk;
	struct clk_init_data init = {};

	tcg = kzalloc(sizeof(*tcg), GFP_KERNEL);
	if (!tcg)
		return ERR_PTR(-ENOMEM);

	tcg->regmap = regmap;
	tcg->offset = offset;
	tcg->bit = bit;
	tcg->shared = shared;
	tcg->share_count = share_count;

	init.name = name;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;
	init.ops = &thead_gate_clk_ops;

	tcg->hw.init = &init;

	clk = clk_register(dev, &tcg->hw);
	if (IS_ERR(clk))
		kfree(tcg);

	return clk;
}
