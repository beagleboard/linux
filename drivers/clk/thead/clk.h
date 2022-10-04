/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#ifndef __MACH_THEAD_CLK_H
#define __MACH_THEAD_CLK_H

#include <linux/spinlock.h>
#include <linux/clk-provider.h>

extern spinlock_t thead_light_clk_lock;

#define LIGHT_PLL_RATE(_vco, _rate, _r, _b, _f, _p, _k)	\
	{						\
		.vco_rate	=	(_vco),		\
		.rate		=	(_rate),	\
		.refdiv		=	(_r),		\
		.fbdiv		=	(_b),		\
		.frac		=	(_f),		\
		.postdiv1	=	(_p),		\
		.postdiv2	=	(_k),		\
	}

enum light_pll_outtype {
	LIGHT_PLL_VCO,
	LIGHT_PLL_DIV,
};

enum light_div_type {
        MUX_TYPE_DIV,
        MUX_TYPE_CDE,
};

enum light_pll_clktype {
	LIGHT_AUDIO_PLL,
	LIGHT_SYS_PLL,
	LIGHT_CPU_PLL0,
	LIGHT_CPU_PLL1,
	LIGHT_GMAC_PLL,
	LIGHT_VIDEO_PLL,
	LIGHT_DDR_PLL,
	LIGHT_DPU0_PLL,
	LIGHT_DPU1_PLL,
};

struct light_pll_rate_table {
	unsigned long vco_rate;
	unsigned long rate;
	unsigned int refdiv;
	unsigned int fbdiv;
	unsigned int frac;
	unsigned int postdiv1;
	unsigned int postdiv2;
};

struct light_pll_clk {
	enum light_pll_outtype out_type;
	enum light_pll_clktype clk_type;
	const struct light_pll_rate_table *rate_table;
	int rate_count;
	int flags;
};

static inline struct clk *thead_light_clk_fixed_factor(const char *name,
		const char *parent, unsigned int mult, unsigned int div)
{
	return clk_register_fixed_factor(NULL, name, parent,
			CLK_SET_RATE_PARENT, mult, div);
}

struct clk *thead_light_pll(const char *name, const char *parent_name,
			    void __iomem *base,
			    const struct light_pll_clk *pll_clk);

static inline struct clk *thead_clk_light_gate(const char *name, const char *parent,
					       void __iomem *reg, u8 shift)
{
	return clk_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
			shift, 0, &thead_light_clk_lock);
}

struct clk *thead_clk_light_register_gate_shared(const char *name, const char *parent,
						 unsigned long flags, void __iomem *reg,
						 u8 shift, spinlock_t *lock,
						 unsigned int *share_count);

struct clk *thead_clk_light_divider(const char *name, const char *parent,
				    void __iomem *reg, u8 shift, u8 width,
				    u8 sync, enum light_div_type div_type,
				    u16 min, u16 max);

void thead_unregister_clocks(struct clk *clks[], unsigned int count);

static inline struct clk *thead_clk_fixed(const char *name, unsigned long rate)
{
	return clk_register_fixed_rate(NULL, name, NULL, 0, rate);
}

static inline struct clk *thead_clk_light_gate_shared(const char *name, const char *parent,
					void __iomem *reg, u8 shift,
					unsigned int *share_count)
{
	return thead_clk_light_register_gate_shared(name, parent, CLK_SET_RATE_PARENT, reg,
						    shift, &thead_light_clk_lock, share_count);
}

static inline struct clk *thead_light_clk_mux_flags(const char *name,
			void __iomem *reg, u8 shift, u8 width,
			const char * const *parents, int num_parents,
			unsigned long flags)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			flags | CLK_SET_RATE_NO_REPARENT, reg, shift, width, 0,
			&thead_light_clk_lock);
}
#endif
