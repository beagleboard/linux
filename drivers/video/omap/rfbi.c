/*
 * File: drivers/video/omap/omap2/rfbi.c
 *
 * OMAP2 Remote Frame Buffer Interface support
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Juha Yrjölä <juha.yrjola@nokia.com>
 *	   Imre Deak <imre.deak@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/clk.h>

#include <asm/io.h>

#include <asm/arch/omapfb.h>

#include "dispc.h"

#define MODULE_NAME "omapfb-rfbi"

#define pr_err(fmt, args...) printk(KERN_ERR MODULE_NAME ": " fmt, ## args)

#define RFBI_BASE		0x48050800
#define RFBI_REVISION		0x0000
#define RFBI_SYSCONFIG		0x0010
#define RFBI_SYSSTATUS		0x0014
#define RFBI_CONTROL		0x0040
#define RFBI_PIXEL_CNT		0x0044
#define RFBI_LINE_NUMBER	0x0048
#define RFBI_CMD		0x004c
#define RFBI_PARAM		0x0050
#define RFBI_DATA		0x0054
#define RFBI_READ		0x0058
#define RFBI_STATUS		0x005c
#define RFBI_CONFIG0		0x0060
#define RFBI_ONOFF_TIME0	0x0064
#define RFBI_CYCLE_TIME0	0x0068
#define RFBI_DATA_CYCLE1_0	0x006c
#define RFBI_DATA_CYCLE2_0	0x0070
#define RFBI_DATA_CYCLE3_0	0x0074
#define RFBI_VSYNC_WIDTH	0x0090
#define RFBI_HSYNC_WIDTH	0x0094

#define DISPC_BASE		0x48050400
#define DISPC_CONTROL		0x0040

static struct {
	u32		base;
	void		(*lcdc_callback)(void *data);
	void		*lcdc_callback_data;
	unsigned long	l4_khz;
} rfbi;

static inline void rfbi_write_reg(int idx, u32 val)
{
	__raw_writel(val, rfbi.base + idx);
}

static inline u32 rfbi_read_reg(int idx)
{
	return __raw_readl(rfbi.base + idx);
}

static int ns_to_l4_ticks(int time)
{
	unsigned long tick_ps;
	int ret;

	/* Calculate in picosecs to yield more exact results */
	tick_ps = 1000000000 / (rfbi.l4_khz);

	ret = (time * 1000 + tick_ps - 1) / tick_ps;

	return ret * 2;
}

#ifdef OMAPFB_DBG
static void print_timings(void)
{
	u32 l;

	DBGPRINT(1, "Tick time %lu ps\n", 1000000000 / rfbi.l4_khz);
	l = rfbi_read_reg(RFBI_ONOFF_TIME0);
	DBGPRINT(1, "CSONTIME %d, CSOFFTIME %d, WEONTIME %d, WEOFFTIME %d, "
	       "REONTIME %d, REOFFTIME %d\n",
	       l & 0x0f, (l >> 4) & 0x3f, (l >> 10) & 0x0f, (l >> 14) & 0x3f,
	       (l >> 20) & 0x0f, (l >> 24) & 0x3f);
	l = rfbi_read_reg(RFBI_CYCLE_TIME0);
	DBGPRINT(1, "WECYCLETIME %d, RECYCLETIME %d, CSPULSEWIDTH %d, "
	       "ACCESSTIME %d\n",
	       (l & 0x3f), (l >> 6) & 0x3f, (l >> 12) & 0x3f, (l >> 22) & 0x3f);
}
#endif

static void rfbi_set_timings(const struct extif_timings *t)
{
	u32 l;
	int on, off;

	on = ns_to_l4_ticks(t->cs_on_time) & 0x0f;
	l = on;
	off = ns_to_l4_ticks(t->cs_off_time) & 0x3f;
	if (off <= on)
		off = on + 2;
	l |= off << 4;

	on = ns_to_l4_ticks(t->we_on_time) & 0x0f;
	l |= on << 10;
	off = ns_to_l4_ticks(t->we_off_time) & 0x3f;
	if (off <= on)
		off = on + 2;
	l |= off << 14;

	l |= (ns_to_l4_ticks(t->re_on_time) & 0x0f) << 20;
	l |= (ns_to_l4_ticks(t->re_off_time) & 0x3f) << 24;
	rfbi_write_reg(RFBI_ONOFF_TIME0, l);

	l = ns_to_l4_ticks(t->we_cycle_time) & 0x3f;
	l |= (ns_to_l4_ticks(t->re_cycle_time) & 0x3f) << 6;
	l |= (ns_to_l4_ticks(t->cs_pulse_width) & 0x3f) << 12;
	l |= (ns_to_l4_ticks(t->access_time) & 0x3f) << 22;
	rfbi_write_reg(RFBI_CYCLE_TIME0, l);
}

static void rfbi_write_command(u32 cmd)
{
	rfbi_write_reg(RFBI_CMD, cmd);
}

static u32 rfbi_read_data(void)
{
	u32 val;

	rfbi_write_reg(RFBI_READ, 0);
	val = rfbi_read_reg(RFBI_READ);
	return val;
}

static void rfbi_write_data(u32 val)
{
	rfbi_write_reg(RFBI_PARAM, val);
}

static void rfbi_transfer_area(int width, int height,
				void (callback)(void * data), void *data)
{
	u32 w;

	BUG_ON(callback == NULL);

	omap_dispc_set_lcd_size(width, height);

	rfbi.lcdc_callback = callback;
	rfbi.lcdc_callback_data = data;

	rfbi_write_reg(RFBI_PIXEL_CNT, width * height);

	w = rfbi_read_reg(RFBI_CONTROL);
	/* Enable, Internal trigger */
	rfbi_write_reg(RFBI_CONTROL, w | (1 << 0) | (1 << 4));

	omap_dispc_enable_lcd_out(1);
}

static inline void _stop_transfer(void)
{
	u32 w;

	w = rfbi_read_reg(RFBI_CONTROL);
	rfbi_write_reg(RFBI_CONTROL, w & ~(1 << 0));
}

static void rfbi_dma_callback(void *data)
{
	_stop_transfer();
	rfbi.lcdc_callback(rfbi.lcdc_callback_data);
}

static int rfbi_init(void)
{
	u32 l;
	int r;
	struct clk *dss_ick;

	memset(&rfbi, 0, sizeof(rfbi));
	rfbi.base = io_p2v(RFBI_BASE);

	l = rfbi_read_reg(RFBI_REVISION);
	pr_info(MODULE_NAME ": version %d.%d\n", (l >> 4) & 0x0f, l & 0x0f);

	dss_ick = clk_get(NULL, "dss_ick");
	if (IS_ERR(dss_ick)) {
		pr_err("can't get dss_ick\n");
		return PTR_ERR(dss_ick);
	}
	rfbi.l4_khz = clk_get_rate(dss_ick) / 1000;
	clk_put(dss_ick);

	/* Reset */
	rfbi_write_reg(RFBI_SYSCONFIG, 1 << 1);
	while (!(rfbi_read_reg(RFBI_SYSSTATUS) & (1 << 0)));

	l = rfbi_read_reg(RFBI_SYSCONFIG);
	/* Enable autoidle and smart-idle */
	l |= (1 << 0) | (2 << 3);
	rfbi_write_reg(RFBI_SYSCONFIG, l);

	/* 16-bit interface, ITE trigger mode, 16-bit data */
	l = (0x03 << 0) | (0x00 << 2) | (0x01 << 5) | (0x02 << 7);
	l |= (0 << 9) | (1 << 20) | (1 << 21);
	rfbi_write_reg(RFBI_CONFIG0, l);

	l = 0x10;
	rfbi_write_reg(RFBI_DATA_CYCLE1_0, l);
	rfbi_write_reg(RFBI_DATA_CYCLE2_0, l);
	rfbi_write_reg(RFBI_DATA_CYCLE3_0, l);

	l = rfbi_read_reg(RFBI_CONTROL);
	/* Select CS0 */
	l = (0x01 << 2);
	rfbi_write_reg(RFBI_CONTROL, l);

	if ((r = omap_dispc_request_irq(rfbi_dma_callback, NULL)) < 0) {
		pr_err("can't get DISPC irq\n");
		return r;
	}

	return 0;
}

static void rfbi_cleanup(void)
{
	omap_dispc_free_irq();
}

struct lcd_ctrl_extif rfbi_extif = {
	.init			= rfbi_init,
	.cleanup		= rfbi_cleanup,
	.set_timings		= rfbi_set_timings,
	.write_command		= rfbi_write_command,
	.read_data		= rfbi_read_data,
	.write_data		= rfbi_write_data,
	.transfer_area		= rfbi_transfer_area,
};

