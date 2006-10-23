/*
 * File: drivers/video/omap/omap1/sossi.c
 *
 * OMAP1 Special OptimiSed Screen Interface support
 *
 * Copyright (C) 2004-2005 Nokia Corporation
 * Author: Juha Yrjölä <juha.yrjola@nokia.com>
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
#include <linux/mm.h>
#include <linux/clk.h>

#include <asm/io.h>

#include <asm/arch/dma.h>
#include <asm/arch/omapfb.h>

#include "lcdc.h"

#define MODULE_NAME		"omapfb-sossi"

#define OMAP_SOSSI_BASE         0xfffbac00
#define SOSSI_ID_REG		0x00
#define SOSSI_INIT1_REG		0x04
#define SOSSI_INIT2_REG		0x08
#define SOSSI_INIT3_REG		0x0c
#define SOSSI_FIFO_REG		0x10
#define SOSSI_REOTABLE_REG	0x14
#define SOSSI_TEARING_REG	0x18
#define SOSSI_INIT1B_REG	0x1c
#define SOSSI_FIFOB_REG		0x20

#define DMA_GSCR          0xfffedc04
#define DMA_LCD_CCR       0xfffee3c2
#define DMA_LCD_CTRL      0xfffee3c4
#define DMA_LCD_LCH_CTRL  0xfffee3ea

#define RD_ACCESS		0
#define WR_ACCESS		1

#define SOSSI_MAX_XMIT_BYTES	(512 * 1024)

static struct {
	void __iomem	*base;
	unsigned long	dpll_khz;
	int		bus_pick_width;
	void		(*lcdc_callback)(void *data);
	void		*lcdc_callback_data;
	/* timing for read and write access */
	int		clk_div;
	u8		clk_tw0[2];
	u8		clk_tw1[2];
	/* if last_access is the same as current we don't have to change
	 * the timings
	 */
	int		last_access;

	struct omapfb_device	*fbdev;
	struct lcd_ctrl_extif	*extif;
} sossi;

static inline u32 sossi_read_reg(int reg)
{
        return readl(sossi.base + reg);
}

static inline u16 sossi_read_reg16(int reg)
{
        return readw(sossi.base + reg);
}

static inline u8 sossi_read_reg8(int reg)
{
        return readb(sossi.base + reg);
}

static inline void sossi_write_reg(int reg, u32 value)
{
        writel(value, sossi.base + reg);
}

static inline void sossi_write_reg16(int reg, u16 value)
{
        writew(value, sossi.base + reg);
}

static inline void sossi_write_reg8(int reg, u8 value)
{
        writeb(value, sossi.base + reg);
}

static void sossi_set_bits(int reg, u32 bits)
{
        sossi_write_reg(reg, sossi_read_reg(reg) | bits);
}

static void sossi_clear_bits(int reg, u32 bits)
{
        sossi_write_reg(reg, sossi_read_reg(reg) & ~bits);
}

#define MOD_CONF_CTRL_1   0xfffe1110
#define CONF_SOSSI_RESET_R      (1 << 23)
#define CONF_MOD_SOSSI_CLK_EN_R (1 << 16)

static void sossi_dma_callback(void *data);

#define KHZ_TO_PS(x)	(1000000000 / (x))

static u32 ps_to_sossi_ticks(u32 ps, int div)
{
	u32 clk_period = KHZ_TO_PS(sossi.dpll_khz) * div;
	return (clk_period + ps - 1) / clk_period;
}

static int calc_rd_timings(struct extif_timings *t)
{
	u32 tw0, tw1;
	int reon, reoff, recyc, actim;
	int div = t->clk_div;

	/* Make sure that after conversion it still holds that:
	 * reoff > reon, recyc >= reoff, actim > reon
	 */
	reon = ps_to_sossi_ticks(t->re_on_time, div);
	/* reon will be exactly one sossi tick */
	if (reon > 1)
		return -1;

	reoff = ps_to_sossi_ticks(t->re_off_time, div);

	if (reoff <= reon)
		reoff = reon + 1;

	tw0 = reoff - reon;
	if (tw0 > 0x10)
		return -1;

	recyc = ps_to_sossi_ticks(t->re_cycle_time, div);
	if (recyc <= reoff)
		recyc = reoff + 1;

	tw1 = recyc - tw0;
	/* values less then 3 result in the SOSSI block resetting itself */
	if (tw1 < 3)
		tw1 = 3;
	if (tw1 > 0x40)
		return -1;

	actim = ps_to_sossi_ticks(t->access_time, div);
	if (actim < reoff)
		actim++;
	/* access time (data hold time) will be exactly one sossi
	 * tick
	 */
	if (actim - reoff > 1)
		return -1;

	t->tim[0] = tw0 - 1;
	t->tim[1] = tw1 - 1;

	return 0;
}

static int calc_wr_timings(struct extif_timings *t)
{
	u32 tw0, tw1;
	int weon, weoff, wecyc;
	int div = t->clk_div;

	/* Make sure that after conversion it still holds that:
	 * weoff > weon, wecyc >= weoff
	 */
	weon = ps_to_sossi_ticks(t->we_on_time, div);
	/* weon will be exactly one sossi tick */
	if (weon > 1)
		return -1;

	weoff = ps_to_sossi_ticks(t->we_off_time, div);
	if (weoff <= weon)
		weoff = weon + 1;
	tw0 = weoff - weon;
	if (tw0 > 0x10)
		return -1;

	wecyc = ps_to_sossi_ticks(t->we_cycle_time, div);
	if (wecyc <= weoff)
		wecyc = weoff + 1;

	tw1 = wecyc - tw0;
	/* values less then 3 result in the SOSSI block resetting itself */
	if (tw1 < 3)
		tw1 = 3;
	if (tw1 > 0x40)
		return -1;

	t->tim[2] = tw0 - 1;
	t->tim[3] = tw1 - 1;

	return 0;
}

static void _set_timing(int div, int tw0, int tw1)
{
	u32 l;

#ifdef VERBOSE
	dev_dbg(sossi.fbdev->dev, "Using TW0 = %d, TW1 = %d, div = %d\n",
		 tw0 + 1, tw1 + 1, div + 1);
#endif

	l = omap_readl(MOD_CONF_CTRL_1);
	l &= ~(7 << 17);
	l |= div << 17;
	omap_writel(l, MOD_CONF_CTRL_1);

	l = sossi_read_reg(SOSSI_INIT1_REG);
	l &= ~((0x0f << 20) | (0x3f << 24));
	l |= (tw0 << 20) | (tw1 << 24);
	sossi_write_reg(SOSSI_INIT1_REG, l);
}

static inline void set_timing(int access)
{
	if (access != sossi.last_access) {
		sossi.last_access = access;
		_set_timing(sossi.clk_div,
			    sossi.clk_tw0[access], sossi.clk_tw1[access]);
	}
}

static void sossi_start_transfer(void)
{
	/* WE */
	sossi_clear_bits(SOSSI_INIT2_REG, 1 << 4);
	/* CS active low */
	sossi_clear_bits(SOSSI_INIT1_REG, 1 << 30);
	/* FIXME: locking? */
}

static void sossi_stop_transfer(void)
{
	/* WE */
	sossi_set_bits(SOSSI_INIT2_REG, 1 << 4);
	/* CS active low */
	sossi_set_bits(SOSSI_INIT1_REG, 1 << 30);
	/* FIXME: locking? */
}

static void wait_end_of_write(void)
{
	/* Before reading we must check if some writings are going on */
	while (!(sossi_read_reg(SOSSI_INIT2_REG) & (1 << 3)));
}

static void send_data(const void *data, unsigned int len)
{
	while (len >= 4) {
		sossi_write_reg(SOSSI_FIFO_REG, *(const u32 *) data);
		len -= 4;
		data += 4;
	}
	while (len >= 2) {
		sossi_write_reg16(SOSSI_FIFO_REG, *(const u16 *) data);
		len -= 2;
		data += 2;
	}
	while (len) {
		sossi_write_reg8(SOSSI_FIFO_REG, *(const u8 *) data);
		len--;
		data++;
	}
}

static void set_cycles(unsigned int len)
{
	unsigned long nr_cycles = len / (sossi.bus_pick_width / 8);

	BUG_ON((nr_cycles - 1) & ~0x3ffff);

	sossi_clear_bits(SOSSI_INIT1_REG, 0x3ffff);
	sossi_set_bits(SOSSI_INIT1_REG, (nr_cycles - 1) & 0x3ffff);
}

static int sossi_convert_timings(struct extif_timings *t)
{
	int r = 0;
	int div = t->clk_div;

	t->converted = 0;

	if (div <= 0 || div > 8)
		return -1;

	/* no CS on SOSSI, so ignore cson, csoff, cs_pulsewidth */
	if ((r = calc_rd_timings(t)) < 0)
		return r;

	if ((r = calc_wr_timings(t)) < 0)
		return r;

	t->tim[4] = div - 1;

	t->converted = 1;

	return 0;
}

static void sossi_set_timings(const struct extif_timings *t)
{
	BUG_ON(!t->converted);

	sossi.clk_tw0[RD_ACCESS] = t->tim[0];
	sossi.clk_tw1[RD_ACCESS] = t->tim[1];

	sossi.clk_tw0[WR_ACCESS] = t->tim[2];
	sossi.clk_tw1[WR_ACCESS] = t->tim[3];

	sossi.clk_div = t->tim[4];
}

static void sossi_get_clk_info(u32 *clk_period, u32 *max_clk_div)
{
	*clk_period = KHZ_TO_PS(sossi.dpll_khz);
	*max_clk_div = 8;
}

static void sossi_set_bits_per_cycle(int bpc)
{
	u32 l;
	int bus_pick_count, bus_pick_width;

	/* We set explicitly the the bus_pick_count as well, although
	 * with remapping/reordering disabled it will be calculated by HW
	 * as (32 / bus_pick_width).
	 */
	switch (bpc) {
	case 8:
		bus_pick_count = 4;
		bus_pick_width = 8;
		break;
	case 16:
		bus_pick_count = 2;
		bus_pick_width = 16;
		break;
	default:
		BUG();
		return;
	}
	l = sossi_read_reg(SOSSI_INIT3_REG);
	sossi.bus_pick_width = bus_pick_width;
	l &= ~0x3ff;
	l |= ((bus_pick_count - 1) << 5) | ((bus_pick_width - 1) & 0x1f);
	sossi_write_reg(SOSSI_INIT3_REG, l);
}

static void sossi_write_command(const void *data, unsigned int len)
{
	set_timing(WR_ACCESS);
	/* CMD#/DATA */
	sossi_clear_bits(SOSSI_INIT1_REG, 1 << 18);
	set_cycles(len);
	sossi_start_transfer();
	send_data(data, len);
	sossi_stop_transfer();
	wait_end_of_write();
}

static void sossi_write_data(const void *data, unsigned int len)
{
	set_timing(WR_ACCESS);
	/* CMD#/DATA */
	sossi_set_bits(SOSSI_INIT1_REG, 1 << 18);
	set_cycles(len);
	sossi_start_transfer();
	send_data(data, len);
	sossi_stop_transfer();
	wait_end_of_write();
}

static void sossi_transfer_area(int width, int height,
				void (callback)(void *data), void *data)
{
	BUG_ON(callback == NULL);

	sossi.lcdc_callback = callback;
	sossi.lcdc_callback_data = data;

	set_timing(WR_ACCESS);
	/* CMD#/DATA */
	sossi_set_bits(SOSSI_INIT1_REG, 1 << 18);
	set_cycles(width * height * sossi.bus_pick_width / 8);

	sossi_start_transfer();
	omap_enable_lcd_dma();
}

static void sossi_dma_callback(void *data)
{
	omap_stop_lcd_dma();
	sossi_stop_transfer();
	sossi.lcdc_callback(sossi.lcdc_callback_data);
}

static void sossi_read_data(void *data, unsigned int len)
{
	set_timing(RD_ACCESS);
	/* CMD#/DATA */
	sossi_set_bits(SOSSI_INIT1_REG, 1 << 18);
	set_cycles(len);
	sossi_start_transfer();
	while (len >= 4) {
		*(u32 *) data = sossi_read_reg(SOSSI_FIFO_REG);
		len -= 4;
		data += 4;
	}
	while (len >= 2) {
		*(u16 *) data = sossi_read_reg16(SOSSI_FIFO_REG);
		len -= 2;
		data += 2;
	}
	while (len) {
		*(u8 *) data = sossi_read_reg8(SOSSI_FIFO_REG);
		len--;
		data++;
	}
	sossi_stop_transfer();
}

static int sossi_init(struct omapfb_device *fbdev)
{
	u32 l, k;
	struct clk *dpll_clk;
	int r;

	sossi.fbdev = fbdev;

	sossi.base = (void __iomem *)IO_ADDRESS(OMAP_SOSSI_BASE);
	dpll_clk = clk_get(fbdev->dev, "ck_dpll1");
	if (IS_ERR(dpll_clk)) {
		dev_err(fbdev->dev, "can't get dpll1 clock\n");
		return PTR_ERR(dpll_clk);
	}

	sossi.dpll_khz = clk_get_rate(dpll_clk) / 1000;
	clk_put(dpll_clk);

	/* Reset and enable the SoSSI module */
	l = omap_readl(MOD_CONF_CTRL_1);
	l |= CONF_SOSSI_RESET_R;
	omap_writel(l, MOD_CONF_CTRL_1);
	l &= ~CONF_SOSSI_RESET_R;
	omap_writel(l, MOD_CONF_CTRL_1);

	l |= CONF_MOD_SOSSI_CLK_EN_R;
	omap_writel(l, MOD_CONF_CTRL_1);

	omap_writel(omap_readl(ARM_IDLECT2) | (1 << 11), ARM_IDLECT2);
	omap_writel(omap_readl(ARM_IDLECT1) | (1 << 6), ARM_IDLECT1);

	l = sossi_read_reg(SOSSI_INIT2_REG);
	/* Enable and reset the SoSSI block */
	l |= (1 << 0) | (1 << 1);
	sossi_write_reg(SOSSI_INIT2_REG, l);
	/* Take SoSSI out of reset */
	l &= ~(1 << 1);
	sossi_write_reg(SOSSI_INIT2_REG, l);

	sossi_write_reg(SOSSI_ID_REG, 0);
	l = sossi_read_reg(SOSSI_ID_REG);
	k = sossi_read_reg(SOSSI_ID_REG);

	if (l != 0x55555555 || k != 0xaaaaaaaa) {
		dev_err(fbdev->dev,
			"invalid SoSSI sync pattern: %08x, %08x\n", l, k);
		return -ENODEV;
	}

	if ((r = omap_lcdc_set_dma_callback(sossi_dma_callback, NULL)) < 0) {
		dev_err(fbdev->dev, "can't get LCDC IRQ\n");
		return r;
	}

	l = sossi_read_reg(SOSSI_ID_REG); /* Component code */
	l = sossi_read_reg(SOSSI_ID_REG);
	pr_info("omapfb: SoSSI version %d.%d initialized\n",
		l >> 16, l & 0xffff);

	l = sossi_read_reg(SOSSI_INIT1_REG);
	l |= (1 << 19); /* DMA_MODE */
	l &= ~(1 << 31); /* REORDERING */
	sossi_write_reg(SOSSI_INIT1_REG, l);

	return 0;
}

static void sossi_cleanup(void)
{
	omap_lcdc_free_dma_callback();
}

const struct lcd_ctrl_extif omap1_ext_if = {
	.init			= sossi_init,
	.cleanup		= sossi_cleanup,
	.get_clk_info		= sossi_get_clk_info,
	.convert_timings	= sossi_convert_timings,
	.set_timings		= sossi_set_timings,
	.set_bits_per_cycle	= sossi_set_bits_per_cycle,
	.write_command		= sossi_write_command,
	.read_data		= sossi_read_data,
	.write_data		= sossi_write_data,
	.transfer_area		= sossi_transfer_area,

	.max_transmit_size	= SOSSI_MAX_XMIT_BYTES,
};

