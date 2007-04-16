/*
 * linux/arch/arm/mach-omap2/board-n800-flash.c
 *
 * Copyright (C) 2006 Nokia Corporation
 * Author: Juha Yrjola
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <asm/mach/flash.h>
#include <linux/mtd/onenand_regs.h>

#include <asm/io.h>
#include <asm/arch/onenand.h>
#include <asm/arch/board.h>
#include <asm/arch/gpmc.h>

static struct mtd_partition n800_partitions[8];

static int n800_onenand_setup(void __iomem *);

static struct omap_onenand_platform_data n800_onenand_data = {
	.cs = 0,
	.gpio_irq = 26,
	.parts = n800_partitions,
	.nr_parts = 0, /* filled later */
	.onenand_setup = n800_onenand_setup,
};

static struct platform_device n800_onenand_device = {
	.name		= "omap2-onenand",
	.id		= -1,
	.dev = {
		.platform_data = &n800_onenand_data,
	},
};

static unsigned short omap2_onenand_readw(void __iomem *addr)
{
	return readw(addr);
}

static void omap2_onenand_writew(unsigned short value, void __iomem *addr)
{
	writew(value, addr);
}

static int omap2_onenand_set_sync_mode(int cs, void __iomem *onenand_base)
{
	const int min_gpmc_clk_period = 18;
	struct gpmc_timings t;
	int tick_ns, div, fclk_offset_ns, fclk_offset, gpmc_clk_ns, latency;
	u32 reg;

	tick_ns = gpmc_round_ns_to_ticks(1);
	div = gpmc_cs_calc_divider(cs, min_gpmc_clk_period);
	gpmc_clk_ns = div * tick_ns;
	if (gpmc_clk_ns >= 24)
		latency = 3;
	else
		latency = 4;

	/* Configure OneNAND for sync read */
	reg = omap2_onenand_readw(onenand_base + ONENAND_REG_SYS_CFG1);
	reg &= ~((0x7 << ONENAND_SYS_CFG1_BRL_SHIFT) | (0x7 << 9));
	reg |=	(latency << ONENAND_SYS_CFG1_BRL_SHIFT) |
		ONENAND_SYS_CFG1_SYNC_READ |
		ONENAND_SYS_CFG1_BL_16;
	omap2_onenand_writew(reg, onenand_base + ONENAND_REG_SYS_CFG1);

	/* FIXME: Get timings from platform data */
	/* Set syncronous read timings */
	memset(&t, 0, sizeof(t));
	t.sync_clk = min_gpmc_clk_period;
	t.cs_on = 0;
	t.adv_on = gpmc_round_ns_to_ticks(7);
	fclk_offset_ns = t.adv_on + gpmc_round_ns_to_ticks(7);
	fclk_offset = fclk_offset_ns / gpmc_round_ns_to_ticks(1);
	t.page_burst_access = gpmc_clk_ns;

	/* Read */
	t.adv_rd_off = fclk_offset_ns + gpmc_round_ns_to_ticks(7);
	t.oe_on = t.adv_rd_off;
	t.access = fclk_offset_ns + (latency + 1) * gpmc_clk_ns;
	t.oe_off = t.access + gpmc_round_ns_to_ticks(1);
	t.cs_rd_off = t.oe_off;
	t.rd_cycle = t.cs_rd_off + gpmc_round_ns_to_ticks(17);

	/* Write */
	t.adv_wr_off = t.adv_on + gpmc_round_ns_to_ticks(12);
	t.we_on = t.adv_wr_off + gpmc_round_ns_to_ticks(1);
	t.we_off = t.we_on + gpmc_round_ns_to_ticks(40);
	t.cs_wr_off = t.we_off + gpmc_round_ns_to_ticks(1);
	t.wr_cycle = t.cs_wr_off + gpmc_round_ns_to_ticks(1);

	/* Configure GPMC for synchronous read */
	fclk_offset %= div;
	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG1,
			  GPMC_CONFIG1_WRAPBURST_SUPP |
			  GPMC_CONFIG1_READMULTIPLE_SUPP |
			  GPMC_CONFIG1_READTYPE_SYNC |
			  GPMC_CONFIG1_CLKACTIVATIONTIME(fclk_offset) |
			  GPMC_CONFIG1_PAGE_LEN(2) |
			  GPMC_CONFIG1_WAIT_READ_MON |
			  GPMC_CONFIG1_WAIT_PIN_SEL(0) |
			  GPMC_CONFIG1_DEVICESIZE_16 |
			  GPMC_CONFIG1_DEVICETYPE_NOR |
			  GPMC_CONFIG1_MUXADDDATA);

	return gpmc_cs_set_timings(cs, &t);
}

static int n800_onenand_setup(void __iomem *onenand_base)
{
	struct omap_onenand_platform_data *datap = &n800_onenand_data;
	struct device *dev = &n800_onenand_device.dev;

	/* Set sync timings in GPMC */
	if (omap2_onenand_set_sync_mode(datap->cs, onenand_base) < 0) {
		dev_err(dev, "Unable to set synchronous mode\n");
		return -EINVAL;
	}

	return 0;
}

void __init n800_flash_init(void)
{
	const struct omap_partition_config *part;
	int i = 0;

	while ((part = omap_get_nr_config(OMAP_TAG_PARTITION,
				struct omap_partition_config, i)) != NULL) {
		struct mtd_partition *mpart;

		mpart = n800_partitions + i;
		mpart->name = (char *) part->name;
		mpart->size = part->size;
		mpart->offset = part->offset;
		mpart->mask_flags = part->mask_flags;
		i++;
		if (i == ARRAY_SIZE(n800_partitions)) {
			printk(KERN_ERR "Too many partitions supplied\n");
			return;
		}
	}
	n800_onenand_data.nr_parts = i;
	if (platform_device_register(&n800_onenand_device) < 0) {
		printk(KERN_ERR "Unable to register OneNAND device\n");
		return;
	}
}
