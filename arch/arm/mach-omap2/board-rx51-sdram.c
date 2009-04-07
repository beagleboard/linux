/*
 * SDRC register values for the Samsung K4X1G323PC
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Lauri Leukkunen <lauri.leukkunen@nokia.com>
 *
 * Original code by Juha Yrjölä <juha.yrjola@solidboot.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>

#include <mach/io.h>
#include <mach/common.h>
#include <mach/clock.h>
#include <mach/sdrc.h>


/* In picoseconds, except for tREF */
struct sdram_timings {
	u32 casl;
	u32 tDAL;
	u32 tDPL;
	u32 tRRD;
	u32 tRCD;
	u32 tRP;
	u32 tRAS;
	u32 tRC;
	u32 tRFC;
	u32 tXSR;

	u32 tREF; /* in ms */
};

struct sdram_info {
	u8 row_lines;
};


struct omap_sdrc_params rx51_sdrc_params[2];

static const struct sdram_timings rx51_timings[] = {
	{
		.casl = 3,
		.tDAL = 15000 + 18000,
		.tDPL = 15000,
		.tRRD = 12000,
		.tRCD = 18000,
		.tRP = 18000,
		.tRAS = 42000,
		.tRC = 66000,
		.tRFC = 97500,
		.tXSR = 120000,

		.tREF = 64,
	},
};

static const struct sdram_info rx51_info = {
	.row_lines = 13,
};

#define CM_BASE		    0x48004000

#define CM_CLKSEL_CORE      0x0a40
#define CM_CLKSEL1_PLL      0x0d40

#define PRM_CLKSEL          0x48306d40
#define PRM_CLKSRC_CTRL     0x48307270

static u32 cm_base = CM_BASE;

static inline u32 cm_read_reg(int idx)
{
	return *(u32 *)OMAP2_IO_ADDRESS(cm_base + idx);
}

static const unsigned long sys_clk_rate_table[] = {
	12000, 13000, 19200, 26000, 38400, 16800
};

static unsigned long get_sys_clk_rate(void)
{
	unsigned long rate;

	rate = sys_clk_rate_table[*(u32 *)OMAP2_IO_ADDRESS(PRM_CLKSEL) & 0x07];
	if (((*(u32 *)OMAP2_IO_ADDRESS(PRM_CLKSRC_CTRL) >> 6) & 0x03) == 0x02)
		rate /= 2;
	return rate;
}

static unsigned long get_core_rate(void)
{
	unsigned long rate;
	u32 l;

	l = cm_read_reg(CM_CLKSEL1_PLL);
	rate = get_sys_clk_rate();
	rate *= ((l >> 16) & 0x7ff);
	rate /= ((l >> 8) & 0x7f) + 1;
	rate /= (l >> 27) & 0x1f;

	return rate;
}

static unsigned long get_l3_rate(void)
{
	u32 l;

	l = cm_read_reg(CM_CLKSEL_CORE);
	return get_core_rate() / (l & 0x03);
}



static unsigned long sdrc_get_fclk_period(void)
{
	/* In picoseconds */
	return 1000000000 / get_l3_rate();
}

static unsigned int sdrc_ps_to_ticks(unsigned int time_ps)
{
	unsigned long tick_ps;

	/* Calculate in picosecs to yield more exact results */
	tick_ps = sdrc_get_fclk_period();

	return (time_ps + tick_ps - 1) / tick_ps;
}
#undef DEBUG
#ifdef DEBUG
static int set_sdrc_timing_regval(u32 *regval, int st_bit, int end_bit,
			       int time, const char *name)
#else
static int set_sdrc_timing_regval(u32 *regval, int st_bit, int end_bit,
			       int time)
#endif
{
	int ticks, mask, nr_bits;

	if (time == 0)
		ticks = 0;
	else
		ticks = sdrc_ps_to_ticks(time);
	nr_bits = end_bit - st_bit + 1;
	if (ticks >= 1 << nr_bits)
		return -1;
	mask = (1 << nr_bits) - 1;
	*regval &= ~(mask << st_bit);
	*regval |= ticks << st_bit;
#ifdef DEBUG
	printk("SDRC %s: %i ticks %i ns\n", name, ticks,
			(unsigned int)sdrc_get_fclk_period() * ticks / 1000);
#endif

	return 0;
}

#ifdef DEBUG
#define SDRC_SET_ONE(reg, st, end, field) \
	if (set_sdrc_timing_regval((reg), (st), (end), rx51_timings->field, #field) < 0) \
		err = -1
#else
#define SDRC_SET_ONE(reg, st, end, field) \
	if (set_sdrc_timing_regval((reg), (st), (end), rx51_timings->field) < 0) \
		err = -1
#endif

struct omap_sdrc_params *rx51_get_sdram_timings(void)
{
	u32 ticks_per_ms;
	u32 rfr, l;
	u32 actim_ctrla, actim_ctrlb;
	u32 rfr_ctrl;
	int err = 0;

	SDRC_SET_ONE(&actim_ctrla,  0,  4, tDAL);
	SDRC_SET_ONE(&actim_ctrla,  6,  8, tDPL);
	SDRC_SET_ONE(&actim_ctrla,  9, 11, tRRD);
	SDRC_SET_ONE(&actim_ctrla, 12, 14, tRCD);
	SDRC_SET_ONE(&actim_ctrla, 15, 17, tRP);
	SDRC_SET_ONE(&actim_ctrla, 18, 21, tRAS);
	SDRC_SET_ONE(&actim_ctrla, 22, 26, tRC);
	SDRC_SET_ONE(&actim_ctrla, 27, 31, tRFC);

	SDRC_SET_ONE(&actim_ctrlb,  0,  7, tXSR);

	ticks_per_ms = sdrc_ps_to_ticks(1000000000);
	rfr = rx51_timings[0].tREF * ticks_per_ms / (1 << rx51_info.row_lines);
	if (rfr > 65535 + 50)
		rfr = 65535;
	else
		rfr -= 50;

	l = rfr << 8;
	rfr_ctrl = l | 0x3; /* autorefresh, reload counter with 8xARCV */

	rx51_sdrc_params[0].rate = 133333333;
	rx51_sdrc_params[0].actim_ctrla = actim_ctrla;
	rx51_sdrc_params[0].actim_ctrlb = actim_ctrlb;
	rx51_sdrc_params[0].rfr_ctrl = rfr_ctrl;
	rx51_sdrc_params[0].mr = 0x32;

	rx51_sdrc_params[1].rate = 0;

	if (err < 0)
		return NULL;

	return &rx51_sdrc_params[0];
}

