/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2022 Alibaba Group Holding Limited.
 */

#ifndef CLK_GATE_H
#define CLK_GATE_H

#include <linux/clk-provider.h>
#include <linux/mfd/syscon.h>

enum clk_gate_type {
	GATE_NOT_SHARED,
	GATE_SHARED,
};

struct thead_clk_gate {
	struct clk_hw hw;
	struct regmap *regmap;
	u32 offset;
	u8 bit;
	bool shared;
	u32 *share_count;
};

struct clk *thead_gate_clk_register(const char *name,
				    const char *parent_name,
				    struct regmap *regmap,
				    int offset,
				    u8 bit,
				    bool shared,
				    u32 *share_count,
				    struct device *dev);

#endif
