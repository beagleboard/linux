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
	int		bits_per_cycle;
	struct omapfb_device *fbdev;
	struct clk	*dss_ick;
	struct clk	*dss1_fck;
} rfbi;

static inline void rfbi_write_reg(int idx, u32 val)
{
	__raw_writel(val, rfbi.base + idx);
}

static inline u32 rfbi_read_reg(int idx)
{
	return __raw_readl(rfbi.base + idx);
}

static int rfbi_get_clocks(void)
{
	if (IS_ERR((rfbi.dss_ick = clk_get(rfbi.fbdev->dev, "dss_ick")))) {
		dev_err(rfbi.fbdev->dev, "can't get dss_ick");
		return PTR_ERR(rfbi.dss_ick);
	}

	if (IS_ERR((rfbi.dss1_fck = clk_get(rfbi.fbdev->dev, "dss1_fck")))) {
		dev_err(rfbi.fbdev->dev, "can't get dss1_fck");
		clk_put(rfbi.dss_ick);
		return PTR_ERR(rfbi.dss1_fck);
	}

	return 0;
}

static void rfbi_put_clocks(void)
{
	clk_put(rfbi.dss1_fck);
	clk_put(rfbi.dss_ick);
}

static void rfbi_enable_clocks(int enable)
{
	if (enable) {
		clk_enable(rfbi.dss_ick);
		clk_enable(rfbi.dss1_fck);
	} else {
		clk_disable(rfbi.dss1_fck);
		clk_disable(rfbi.dss_ick);
	}
}


#ifdef VERBOSE
static void rfbi_print_timings(void)
{
	u32 l;
	u32 time;

	l = rfbi_read_reg(RFBI_CONFIG0);
	time = 1000000000 / rfbi.l4_khz;
	if (l & (1 << 4))
		time *= 2;

	dev_dbg(rfbi.fbdev->dev, "Tick time %u ps\n", time);
	l = rfbi_read_reg(RFBI_ONOFF_TIME0);
	dev_dbg(rfbi.fbdev->dev,
		"CSONTIME %d, CSOFFTIME %d, WEONTIME %d, WEOFFTIME %d, "
		"REONTIME %d, REOFFTIME %d\n",
		l & 0x0f, (l >> 4) & 0x3f, (l >> 10) & 0x0f, (l >> 14) & 0x3f,
		(l >> 20) & 0x0f, (l >> 24) & 0x3f);

	l = rfbi_read_reg(RFBI_CYCLE_TIME0);
	dev_dbg(rfbi.fbdev->dev,
		"WECYCLETIME %d, RECYCLETIME %d, CSPULSEWIDTH %d, "
		"ACCESSTIME %d\n",
		(l & 0x3f), (l >> 6) & 0x3f, (l >> 12) & 0x3f,
		(l >> 22) & 0x3f);
}
#else
static void rfbi_print_timings(void) {}
#endif

static void rfbi_set_timings(const struct extif_timings *t)
{
	u32 l;

	BUG_ON(!t->converted);

	rfbi_enable_clocks(1);
	rfbi_write_reg(RFBI_ONOFF_TIME0, t->tim[0]);
	rfbi_write_reg(RFBI_CYCLE_TIME0, t->tim[1]);

	l = rfbi_read_reg(RFBI_CONFIG0);
	l &= ~(1 << 4);
	l |= (t->tim[2] ? 1 : 0) << 4;
	rfbi_write_reg(RFBI_CONFIG0, l);

	rfbi_print_timings();
	rfbi_enable_clocks(0);
}

static void rfbi_get_clk_info(u32 *clk_period, u32 *max_clk_div)
{
	*clk_period = 1000000000 / rfbi.l4_khz;
	*max_clk_div = 2;
}

static int ps_to_rfbi_ticks(int time, int div)
{
	unsigned long tick_ps;
	int ret;

	/* Calculate in picosecs to yield more exact results */
	tick_ps = 1000000000 / (rfbi.l4_khz) * div;

	ret = (time + tick_ps - 1) / tick_ps;

	return ret;
}

static int rfbi_convert_timings(struct extif_timings *t)
{
	u32 l;
	int reon, reoff, weon, weoff, cson, csoff, cs_pulse;
	int actim, recyc, wecyc;
	int div = t->clk_div;

	if (div <= 0 || div > 2)
		return -1;

	/* Make sure that after conversion it still holds that:
	 * weoff > weon, reoff > reon, recyc >= reoff, wecyc >= weoff,
	 * csoff > cson, csoff >= max(weoff, reoff), actim > reon
	 */
	weon = ps_to_rfbi_ticks(t->we_on_time, div);
	weoff = ps_to_rfbi_ticks(t->we_off_time, div);
	if (weoff <= weon)
		weoff = weon + 1;
	if (weon > 0x0f)
		return -1;
	if (weoff > 0x3f)
		return -1;

	reon = ps_to_rfbi_ticks(t->re_on_time, div);
	reoff = ps_to_rfbi_ticks(t->re_off_time, div);
	if (reoff <= reon)
		reoff = reon + 1;
	if (reon > 0x0f)
		return -1;
	if (reoff > 0x3f)
		return -1;

	cson = ps_to_rfbi_ticks(t->cs_on_time, div);
	csoff = ps_to_rfbi_ticks(t->cs_off_time, div);
	if (csoff <= cson)
		csoff = cson + 1;
	if (csoff < max(weoff, reoff))
		csoff = max(weoff, reoff);
	if (cson > 0x0f)
		return -1;
	if (csoff > 0x3f)
		return -1;

	l =  cson;
	l |= csoff << 4;
	l |= weon  << 10;
	l |= weoff << 14;
	l |= reon  << 20;
	l |= reoff << 24;

	t->tim[0] = l;

	actim = ps_to_rfbi_ticks(t->access_time, div);
	if (actim <= reon)
		actim = reon + 1;
	if (actim > 0x3f)
		return -1;

	wecyc = ps_to_rfbi_ticks(t->we_cycle_time, div);
	if (wecyc < weoff)
		wecyc = weoff;
	if (wecyc > 0x3f)
		return -1;

	recyc = ps_to_rfbi_ticks(t->re_cycle_time, div);
	if (recyc < reoff)
		recyc = reoff;
	if (recyc > 0x3f)
		return -1;

	cs_pulse = ps_to_rfbi_ticks(t->cs_pulse_width, div);
	if (cs_pulse > 0x3f)
		return -1;

	l =  wecyc;
	l |= recyc    << 6;
	l |= cs_pulse << 12;
	l |= actim    << 22;

	t->tim[1] = l;

	t->tim[2] = div - 1;

	t->converted = 1;

	return 0;
}

static void rfbi_write_command(const void *buf, unsigned int len)
{
	rfbi_enable_clocks(1);
	if (rfbi.bits_per_cycle == 16) {
		const u16 *w = buf;
		BUG_ON(len & 1);
		for (; len; len -= 2)
			rfbi_write_reg(RFBI_CMD, *w++);
	} else {
		const u8 *b = buf;
		BUG_ON(rfbi.bits_per_cycle != 8);
		for (; len; len--)
			rfbi_write_reg(RFBI_CMD, *b++);
	}
	rfbi_enable_clocks(0);
}

static void rfbi_read_data(void *buf, unsigned int len)
{
	rfbi_enable_clocks(1);
	if (rfbi.bits_per_cycle == 16) {
		u16 *w = buf;
		BUG_ON(len & ~1);
		for (; len; len -= 2) {
			rfbi_write_reg(RFBI_READ, 0);
			*w++ = rfbi_read_reg(RFBI_READ);
		}
	} else {
		u8 *b = buf;
		BUG_ON(rfbi.bits_per_cycle != 8);
		for (; len; len--) {
			rfbi_write_reg(RFBI_READ, 0);
			*b++ = rfbi_read_reg(RFBI_READ);
		}
	}
	rfbi_enable_clocks(0);
}

static void rfbi_write_data(const void *buf, unsigned int len)
{
	rfbi_enable_clocks(1);
	if (rfbi.bits_per_cycle == 16) {
		const u16 *w = buf;
		BUG_ON(len & 1);
		for (; len; len -= 2)
			rfbi_write_reg(RFBI_PARAM, *w++);
	} else {
		const u8 *b = buf;
		BUG_ON(rfbi.bits_per_cycle != 8);
		for (; len; len--)
			rfbi_write_reg(RFBI_PARAM, *b++);
	}
	rfbi_enable_clocks(0);
}

static void rfbi_transfer_area(int width, int height,
				void (callback)(void * data), void *data)
{
	u32 w;

	BUG_ON(callback == NULL);

	rfbi_enable_clocks(1);
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
	rfbi_enable_clocks(0);
}

static void rfbi_dma_callback(void *data)
{
	_stop_transfer();
	rfbi.lcdc_callback(rfbi.lcdc_callback_data);
}

static void rfbi_set_bits_per_cycle(int bpc)
{
	u32 l;

	rfbi_enable_clocks(1);
	l = rfbi_read_reg(RFBI_CONFIG0);
	l &= ~(0x03 << 0);
	switch (bpc)
	{
	case 8:
		break;
	case 16:
		l |= 3;
		break;
	default:
		BUG();
	}
	rfbi_write_reg(RFBI_CONFIG0, l);
	rfbi.bits_per_cycle = bpc;
	rfbi_enable_clocks(0);
}

static int rfbi_init(struct omapfb_device *fbdev)
{
	u32 l;
	int r;

	rfbi.fbdev = fbdev;
	rfbi.base = io_p2v(RFBI_BASE);

	if ((r = rfbi_get_clocks()) < 0)
		return r;
	rfbi_enable_clocks(1);

	rfbi.l4_khz = clk_get_rate(rfbi.dss_ick) / 1000;

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

	rfbi_write_reg(RFBI_DATA_CYCLE1_0, 0x00000010);

	l = rfbi_read_reg(RFBI_CONTROL);
	/* Select CS0, clear bypass mode */
	l = (0x01 << 2);
	rfbi_write_reg(RFBI_CONTROL, l);

	if ((r = omap_dispc_request_irq(rfbi_dma_callback, NULL)) < 0) {
		dev_err(fbdev->dev, "can't get DISPC irq\n");
		rfbi_enable_clocks(0);
		return r;
	}

	l = rfbi_read_reg(RFBI_REVISION);
	pr_info("omapfb: RFBI version %d.%d initialized\n",
		(l >> 4) & 0x0f, l & 0x0f);

	rfbi_enable_clocks(0);

	return 0;
}

static void rfbi_cleanup(void)
{
	omap_dispc_free_irq();
	rfbi_put_clocks();
}

const struct lcd_ctrl_extif omap2_ext_if = {
	.init			= rfbi_init,
	.cleanup		= rfbi_cleanup,
	.get_clk_info		= rfbi_get_clk_info,
	.set_bits_per_cycle	= rfbi_set_bits_per_cycle,
	.convert_timings	= rfbi_convert_timings,
	.set_timings		= rfbi_set_timings,
	.write_command		= rfbi_write_command,
	.read_data		= rfbi_read_data,
	.write_data		= rfbi_write_data,
	.transfer_area		= rfbi_transfer_area,

	.max_transmit_size	= (u32)~0,
};

