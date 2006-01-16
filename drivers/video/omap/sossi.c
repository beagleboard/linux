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

#include <linux/config.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/clk.h>

#include <asm/io.h>

#include "sossi.h"

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

#define pr_err(fmt, args...) printk(KERN_ERR MODULE_NAME ": " fmt, ## args)

static int sossi_base = IO_ADDRESS(OMAP_SOSSI_BASE);

static inline u32 sossi_read_reg(int reg)
{
        return readl(sossi_base + reg);
}

static inline u16 sossi_read_reg16(int reg)
{
        return readw(sossi_base + reg);
}

static inline u8 sossi_read_reg8(int reg)
{
        return readb(sossi_base + reg);
}

static inline void sossi_write_reg(int reg, u32 value)
{
        writel(value, sossi_base + reg);
}

static inline void sossi_write_reg16(int reg, u16 value)
{
        writew(value, sossi_base + reg);
}

static inline void sossi_write_reg8(int reg, u8 value)
{
        writeb(value, sossi_base + reg);
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

static struct clk *dpll_clk;

int sossi_init(void)
{
	u32 l, k;

	dpll_clk = clk_get(NULL, "ck_dpll1");
	BUG_ON(dpll_clk == NULL);

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
		pr_err("Invalid SoSSI sync pattern: %08x, %08x\n", l, k);
		return -ENODEV;
	}
	l = sossi_read_reg(SOSSI_ID_REG); /* Component code */
	l = sossi_read_reg(SOSSI_ID_REG);
	pr_info(KERN_INFO MODULE_NAME ": version %d.%d initialized\n",
			l >> 16, l & 0xffff);

	l = sossi_read_reg(SOSSI_INIT1_REG);
	l |= (1 << 19); /* DMA_MODE */
	l &= ~(1 << 31); /* REORDERING */
	sossi_write_reg(SOSSI_INIT1_REG, l);

	return 0;
}

static unsigned long get_sossi_clk_rate(int div)
{
	return (clk_get_rate(dpll_clk)) / div;
}

static unsigned long get_sossi_clk_period(int div)
{
	/* In picoseconds */
	return 1000000000 / (get_sossi_clk_rate(div) / 1000);
}

static int ns_to_sossi_ticks(int time, int div)
{
	unsigned long tick_ps;

	/* Calculate in picosecs to yield more exact results */
	tick_ps = get_sossi_clk_period(div);

	return (time * 1000 + tick_ps - 1) / tick_ps;
}

static int set_timings(int div, int tw0, int tw1)
{
	u32 l;

	if (tw1 * 1000 > 64 * get_sossi_clk_period(div))
		return -1;
	if (tw0 * 1000 > 16 * get_sossi_clk_period(div))
		return -1;

	l = omap_readl(MOD_CONF_CTRL_1);
	l &= ~(7 << 17);
	l |= (div - 1) << 17;
	omap_writel(l, MOD_CONF_CTRL_1);

	tw0 = ns_to_sossi_ticks(tw0, div) - 1;
	tw1 = ns_to_sossi_ticks(tw1, div) - 1;
	if (tw0 < 0)
		tw0 = 0;
	if (tw1 < 0)
		tw1 = 0;
#if 0
	printk("Using TW0 = %d, TW1 = %d, div = %d, period = %d ps\n",
	       tw0, tw1, div, get_sossi_clk_period(div));
#endif
	l = sossi_read_reg(SOSSI_INIT1_REG);
	l &= ~((0x0f << 20) | (0x3f << 24));
	l |= ((tw0 & 0x0f) << 20) | ((tw1 & 0x3f) << 24);
	sossi_write_reg(SOSSI_INIT1_REG, l);

	return 0;
}

static struct sossi {
	int bus_pick_width;
} sossi;

void sossi_set_timings(int min_time, int min_tw0, int min_tw1)
{
	int div;

	for (div = 1; div <= 8; div++) {
		if (min_time * 1000 > get_sossi_clk_period(div))
			continue;
		if (set_timings(div, min_tw0, min_tw1) == 0)
			break;
	}
	if (div == 9) {
		pr_err("DPLL frequency too high for SoSSI\n");
		BUG();
	}
}

void sossi_set_xfer_params(int bus_pick_count, int bus_pick_width)
{
	u32 l;

	sossi.bus_pick_width = bus_pick_width;
	l = ((bus_pick_count - 1) << 5) | ((bus_pick_width - 1) & 0x1f);
	sossi_write_reg(SOSSI_INIT3_REG, l);
}

void sossi_start_transfer(void)
{
	/* WE */
	sossi_clear_bits(SOSSI_INIT2_REG, 1 << 4);
	/* CS active low */
	sossi_clear_bits(SOSSI_INIT1_REG, 1 << 30);
	/* FIXME: locking? */
}

void sossi_stop_transfer(void)
{
	/* WE */
	sossi_set_bits(SOSSI_INIT2_REG, 1 << 4);
	/* CS active low */
	sossi_set_bits(SOSSI_INIT1_REG, 1 << 30);
	/* FIXME: locking? */
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
	int nr_cycles = len / (sossi.bus_pick_width / 8);

	sossi_clear_bits(SOSSI_INIT1_REG, 0x3ffff);
	sossi_set_bits(SOSSI_INIT1_REG, (nr_cycles - 1) & 0x3ffff);
}

void sossi_send_cmd(const void *data, unsigned int len)
{
	sossi_clear_bits(SOSSI_INIT1_REG, 1 << 18);
	set_cycles(len);
	send_data(data, len);
}

void sossi_send_data(const void *data, unsigned int len)
{
	sossi_set_bits(SOSSI_INIT1_REG, 1 << 18);
	set_cycles(len);
	send_data(data, len);
}

void sossi_prepare_dma_transfer(unsigned int count)
{
	sossi_set_bits(SOSSI_INIT1_REG, 1 << 18);
	set_cycles(count);
}

void sossi_send_data_const32(u32 data, unsigned int count)
{
	sossi_set_bits(SOSSI_INIT1_REG, 1 << 18);
	set_cycles(count * 4);
	while (count > 0) {
		sossi_write_reg(SOSSI_FIFO_REG, data);
		count--;
	}
}

void sossi_set_tearing(int mode, int hs_counter, int detect_limit,
		       int vs_counter, int vs_detect_limit, int flags)
{
	u32 l = 0;

	l |= vs_counter << 30;
	if (flags & SOSSI_FLAG_HS_INVERTED)
		l |= 1 << 29;
	if (flags & SOSSI_FLAG_VS_INVERTED)
		l |= 1 << 28;
	l |= mode << 26;
	l |= hs_counter << 15;
	l |= vs_detect_limit << 3;
	l |= detect_limit;
	sossi_write_reg(SOSSI_TEARING_REG, l);
}

void sossi_read_data(void *data, unsigned int len)
{
	/* Before reading we must check if some writings are going on */
	while (!(sossi_read_reg(SOSSI_INIT2_REG) & (1 << 3)));
	sossi_set_bits(SOSSI_INIT1_REG, 1 << 18);
	set_cycles(len);
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
}
