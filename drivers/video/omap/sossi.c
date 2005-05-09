/*
 * File: drivers/video/omap_new/omapfb_main.c
 *
 * Special optimiSed Screen Interface driver for TI OMAP boards
 *
 * Copyright (C) 2004 Nokia Corporation
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
#include <asm/io.h>

#include "sossi.h"

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

#if 1
void sossi_dump(void)
{
	printk("  INIT1:    0x%08x\n", sossi_read_reg(SOSSI_INIT1_REG));
	printk("  INIT2:    0x%08x\n", sossi_read_reg(SOSSI_INIT2_REG));
	printk("  INIT3:    0x%08x\n", sossi_read_reg(SOSSI_INIT3_REG));
	printk("  TEARING:  0x%08x\n", sossi_read_reg(SOSSI_TEARING_REG));
	printk("  INIT1B:   0x%08x\n", sossi_read_reg(SOSSI_INIT1B_REG));
}
#endif

static void sossi_dma_init(void)
{
	/* OMAP3.1 mapping disable */
	omap_writel(omap_readl(DMA_GSCR) | (1 << 3), DMA_GSCR);
	/* Logical channel type to b0100 */
	omap_writew(omap_readw(DMA_LCD_LCH_CTRL) | (1 << 2), DMA_LCD_LCH_CTRL);
	/* LCD_DMA dest port to 1 */
	omap_writew(omap_readw(DMA_LCD_CTRL) | (1 << 8), DMA_LCD_CTRL);
	/* LCD_CCR OMAP31 comp mode */
	omap_writew(omap_readw(DMA_LCD_CCR) | (1 << 10), DMA_LCD_CCR);
}

#define MOD_CONF_CTRL_1   0xfffe1110
#define CONF_SOSSI_RESET_R      (1 << 23)
#define CONF_MOD_SOSSI_CLK_EN_R (1 << 16)

int sossi_init(void)
{
	u32 l, k;

	/* Reset and enable the SoSSI module */
	l = omap_readl(MOD_CONF_CTRL_1);
	l |= CONF_SOSSI_RESET_R;
	omap_writel(l, MOD_CONF_CTRL_1);
	l &= ~CONF_SOSSI_RESET_R;
	omap_writel(l, MOD_CONF_CTRL_1);

	l |= CONF_MOD_SOSSI_CLK_EN_R;
	/* FIXME: Hardcode divide ratio 3 */
	l |= 2 << 17;
	omap_writel(l, MOD_CONF_CTRL_1);

	omap_writel(omap_readl(ARM_IDLECT2) | (1 << 11), ARM_IDLECT2);
	omap_writel(omap_readl(ARM_IDLECT1) | (1 << 6), ARM_IDLECT1);

	sossi_dma_init();

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
		printk(KERN_ERR "Invalid SoSSI sync pattern: %08x, %08x\n", l, k);
		return -ENODEV;
	}
	l = sossi_read_reg(SOSSI_ID_REG); /* Component code */
	l = sossi_read_reg(SOSSI_ID_REG);
	printk(KERN_INFO "SoSSI rev. %d.%d initialized\n", l >> 16, l & 0xffff);

	l = sossi_read_reg(SOSSI_INIT1_REG);
	l |= (1 << 19); /* DMA_MODE */
	l &= ~(1 << 31); /* REORDERING */
	sossi_write_reg(SOSSI_INIT1_REG, l);

	return 0;
}

static void set_timings(int tw0, int tw1)
{
	u32 l;

	l = sossi_read_reg(SOSSI_INIT1_REG);
	l &= ~((0x0f << 20) | (0x3f << 24));
	l |= ((tw0 & 0x0f) << 20) | ((tw1 & 0x3f) << 24);
	sossi_write_reg(SOSSI_INIT1_REG, l);
}

static struct sossi {
	int bus_pick_width;
} sossi;

void sossi_set_xfer_params(int tw0, int tw1, int bus_pick_count, int bus_pick_width)
{
	u32 l;

	set_timings(tw0, tw1);	
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
