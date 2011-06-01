/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "clock.h"

static int dummy_clk_enable(unsigned id)
{
	return 0;
}

static void dummy_clk_disable(unsigned id)
{
}

static int dummy_clk_reset(unsigned id, enum clk_reset_action action)
{
	return 0;
}

static int dummy_clk_set_rate(unsigned id, unsigned rate)
{
	return 0;
}

static int dummy_clk_set_min_rate(unsigned id, unsigned rate)
{
	return 0;
}

static int dummy_clk_set_max_rate(unsigned id, unsigned rate)
{
	return 0;
}

static int dummy_clk_set_flags(unsigned id, unsigned flags)
{
	return 0;
}

static unsigned dummy_clk_get_rate(unsigned id)
{
	return 0;
}

static unsigned dummy_clk_is_enabled(unsigned id)
{
	return 0;
}

static long dummy_clk_round_rate(unsigned id, unsigned rate)
{
	return rate;
}

static bool dummy_clk_is_local(unsigned id)
{
	return true;
}

struct clk_ops clk_ops_dummy = {
	.enable = dummy_clk_enable,
	.disable = dummy_clk_disable,
	.reset = dummy_clk_reset,
	.set_rate = dummy_clk_set_rate,
	.set_min_rate = dummy_clk_set_min_rate,
	.set_max_rate = dummy_clk_set_max_rate,
	.set_flags = dummy_clk_set_flags,
	.get_rate = dummy_clk_get_rate,
	.is_enabled = dummy_clk_is_enabled,
	.round_rate = dummy_clk_round_rate,
	.is_local = dummy_clk_is_local,
};
