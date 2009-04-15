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
#include <mach/onenand.h>
#include <mach/board.h>
#include <mach/gpmc.h>

struct mtd_partition n800_partitions[ONENAND_MAX_PARTITIONS];

int n800_onenand_setup(void __iomem *, int freq);

static struct omap_onenand_platform_data n800_onenand_data = {
	.cs = 0,
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

static int omap2_onenand_set_async_mode(int cs, void __iomem *onenand_base)
{
	struct gpmc_timings t;

	const int t_cer = 15;
	const int t_avdp = 12;
	const int t_aavdh = 7;
	const int t_ce = 76;
	const int t_aa = 76;
	const int t_oe = 20;
	const int t_cez = 20; /* max of t_cez, t_oez */
	const int t_ds = 30;
	const int t_wpl = 40;
	const int t_wph = 30;

	memset(&t, 0, sizeof(t));
	t.sync_clk = 0;
	t.cs_on = 0;
	t.adv_on = 0;

	/* Read */
	t.adv_rd_off = gpmc_round_ns_to_ticks(max_t(int, t_avdp, t_cer));
	t.oe_on  = t.adv_rd_off + gpmc_round_ns_to_ticks(t_aavdh);
	t.access = t.adv_on + gpmc_round_ns_to_ticks(t_aa);
	t.access = max_t(int, t.access, t.cs_on + gpmc_round_ns_to_ticks(t_ce));
	t.access = max_t(int, t.access, t.oe_on + gpmc_round_ns_to_ticks(t_oe));
	t.oe_off = t.access + gpmc_round_ns_to_ticks(1);
	t.cs_rd_off = t.oe_off;
	t.rd_cycle  = t.cs_rd_off + gpmc_round_ns_to_ticks(t_cez);

	/* Write */
	t.adv_wr_off = t.adv_rd_off;
	t.we_on  = t.oe_on;
	if (cpu_is_omap34xx()) {
		t.wr_data_mux_bus = t.we_on;
		t.wr_access = t.we_on + gpmc_round_ns_to_ticks(t_ds);
	}
	t.we_off = t.we_on + gpmc_round_ns_to_ticks(t_wpl);
	t.cs_wr_off = t.we_off + gpmc_round_ns_to_ticks(t_wph);
	t.wr_cycle  = t.cs_wr_off + gpmc_round_ns_to_ticks(t_cez);

	/* Configure GPMC for asynchronous read */
	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG1,
			  GPMC_CONFIG1_DEVICESIZE_16 |
			  GPMC_CONFIG1_MUXADDDATA);

	return gpmc_cs_set_timings(cs, &t);
}

static unsigned short omap2_onenand_readw(void __iomem *addr)
{
	return readw(addr);
}

static void omap2_onenand_writew(unsigned short value, void __iomem *addr)
{
	writew(value, addr);
}

static void set_onenand_cfg(void __iomem *onenand_base, int latency,
			    int sync_write, int hf)
{
	u32 reg;

	reg = omap2_onenand_readw(onenand_base + ONENAND_REG_SYS_CFG1);
	reg &= ~((0x7 << ONENAND_SYS_CFG1_BRL_SHIFT) | (0x7 << 9));
	reg |=	(latency << ONENAND_SYS_CFG1_BRL_SHIFT) |
		ONENAND_SYS_CFG1_SYNC_READ |
		ONENAND_SYS_CFG1_BL_16;
	if (sync_write)
		reg |= ONENAND_SYS_CFG1_SYNC_WRITE;
	else
		reg &= ~ONENAND_SYS_CFG1_SYNC_WRITE;
	if (hf)
		reg |= ONENAND_SYS_CFG1_HF;
	else
		reg &= ~ONENAND_SYS_CFG1_HF;
	omap2_onenand_writew(reg, onenand_base + ONENAND_REG_SYS_CFG1);
}

static int omap2_onenand_set_sync_mode(int cs, void __iomem *onenand_base,
				       int freq)
{
	struct gpmc_timings t;
	const int t_cer  = 15;
	const int t_avdp = 12;
	const int t_cez  = 20; /* max of t_cez, t_oez */
	const int t_ds   = 30;
	const int t_wpl  = 40;
	const int t_wph  = 30;
	int min_gpmc_clk_period, t_ces, t_avds, t_avdh, t_ach, t_aavdh, t_rdyo;
	int tick_ns, div, fclk_offset_ns, fclk_offset, gpmc_clk_ns, latency;
	int err, ticks_cez, sync_write = 0, first_time = 0, hf = 0;
	u32 reg;

	if (!freq) {
		/* Very first call freq is not known */
		err = omap2_onenand_set_async_mode(cs, onenand_base);
		if (err)
			return err;
		reg = omap2_onenand_readw(onenand_base +
					  ONENAND_REG_VERSION_ID);
		switch ((reg >> 4) & 0xf) {
		case 0:
			freq = 40;
			break;
		case 1:
			freq = 54;
			break;
		case 2:
			freq = 66;
			break;
		case 3:
			freq = 83;
			break;
		case 4:
			freq = 104;
			break;
		default:
			freq = 54;
			break;
		}
		first_time = 1;
	}

	switch (freq) {
	case 83:
		min_gpmc_clk_period = 12; /* 83 MHz */
		t_ces   = 5;
		t_avds  = 4;
		t_avdh  = 2;
		t_ach   = 6;
		t_aavdh = 6;
		t_rdyo  = 9;
		if (cpu_is_omap34xx())
			sync_write = 1;
		break;
	case 66:
		min_gpmc_clk_period = 15; /* 66 MHz */
		t_ces   = 6;
		t_avds  = 5;
		t_avdh  = 2;
		t_ach   = 6;
		t_aavdh = 6;
		t_rdyo  = 11;
		if (cpu_is_omap34xx())
			sync_write = 1;
		break;
	default:
		min_gpmc_clk_period = 18; /* 54 MHz */
		t_ces   = 7;
		t_avds  = 7;
		t_avdh  = 7;
		t_ach   = 9;
		t_aavdh = 7;
		t_rdyo  = 15;
		break;
	}

	tick_ns = gpmc_ticks_to_ns(1);
	div = gpmc_cs_calc_divider(cs, min_gpmc_clk_period);
	gpmc_clk_ns = gpmc_ticks_to_ns(div);
	if (gpmc_clk_ns < 15) /* >66Mhz */
		hf = 1;
	if (hf)
		latency = 6;
	else if (gpmc_clk_ns >= 25) /* 40 MHz*/
		latency = 3;
	else
		latency = 4;

	if (first_time)
		set_onenand_cfg(onenand_base, latency, sync_write, hf);

	if (div == 1) {
		reg = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG2);
		reg |= (1 << 7);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG2, reg);
		reg = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG3);
		reg |= (1 << 7);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG3, reg);
		reg = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG4);
		reg |= (1 << 7);
		reg |= (1 << 23);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG4, reg);
	} else {
		reg = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG2);
		reg &= ~(1 << 7);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG2, reg);
		reg = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG3);
		reg &= ~(1 << 7);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG3, reg);
		reg = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG4);
		reg &= ~(1 << 7);
		reg &= ~(1 << 23);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG4, reg);
	}

	/* Set synchronous read timings */
	memset(&t, 0, sizeof(t));
	t.sync_clk = min_gpmc_clk_period;
	t.cs_on = 0;
	t.adv_on = 0;
	fclk_offset_ns = gpmc_round_ns_to_ticks(max_t(int, t_ces, t_avds));
	fclk_offset = gpmc_ns_to_ticks(fclk_offset_ns);
	t.page_burst_access = gpmc_clk_ns;

	/* Read */
	t.adv_rd_off = gpmc_ticks_to_ns(fclk_offset + gpmc_ns_to_ticks(t_avdh));
	t.oe_on = gpmc_ticks_to_ns(fclk_offset + gpmc_ns_to_ticks(t_ach));
	t.access = gpmc_ticks_to_ns(fclk_offset + (latency + 1) * div);
	t.oe_off = t.access + gpmc_round_ns_to_ticks(1);
	t.cs_rd_off = t.oe_off;
	ticks_cez = ((gpmc_ns_to_ticks(t_cez) + div - 1) / div) * div;
	t.rd_cycle = gpmc_ticks_to_ns(fclk_offset + (latency + 1) * div +
		     ticks_cez);

	/* Write */
	if (sync_write) {
		t.adv_wr_off = t.adv_rd_off;
		t.we_on  = 0;
		t.we_off = t.cs_rd_off;
		t.cs_wr_off = t.cs_rd_off;
		t.wr_cycle  = t.rd_cycle;
		if (cpu_is_omap34xx()) {
			t.wr_data_mux_bus = gpmc_ticks_to_ns(fclk_offset +
					gpmc_ns_to_ticks(min_gpmc_clk_period +
					t_rdyo));
			t.wr_access = t.access;
		}
	} else {
		t.adv_wr_off = gpmc_round_ns_to_ticks(max_t(int, t_avdp, t_cer));
		t.we_on  = t.adv_wr_off + gpmc_round_ns_to_ticks(t_aavdh);
		t.we_off = t.we_on + gpmc_round_ns_to_ticks(t_wpl);
		t.cs_wr_off = t.we_off + gpmc_round_ns_to_ticks(t_wph);
		t.wr_cycle  = t.cs_wr_off + gpmc_round_ns_to_ticks(t_cez);
		if (cpu_is_omap34xx()) {
			t.wr_data_mux_bus = t.we_on;
			t.wr_access = t.we_on + gpmc_round_ns_to_ticks(t_ds);
		}
	}

	/* Configure GPMC for synchronous read */
	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG1,
			  GPMC_CONFIG1_WRAPBURST_SUPP |
			  GPMC_CONFIG1_READMULTIPLE_SUPP |
			  GPMC_CONFIG1_READTYPE_SYNC |
			  (sync_write ? GPMC_CONFIG1_WRITEMULTIPLE_SUPP : 0) |
			  (sync_write ? GPMC_CONFIG1_WRITETYPE_SYNC : 0) |
			  GPMC_CONFIG1_CLKACTIVATIONTIME(fclk_offset) |
			  GPMC_CONFIG1_PAGE_LEN(2) |
			  (cpu_is_omap34xx() ? 0 :
				(GPMC_CONFIG1_WAIT_READ_MON |
				 GPMC_CONFIG1_WAIT_PIN_SEL(0))) |
			  GPMC_CONFIG1_DEVICESIZE_16 |
			  GPMC_CONFIG1_DEVICETYPE_NOR |
			  GPMC_CONFIG1_MUXADDDATA);

	err = gpmc_cs_set_timings(cs, &t);
	if (err)
		return err;

	set_onenand_cfg(onenand_base, latency, sync_write, hf);

	return 0;
}

int n800_onenand_setup(void __iomem *onenand_base, int freq)
{
	struct omap_onenand_platform_data *datap = &n800_onenand_data;
	struct device *dev = &n800_onenand_device.dev;

	/* Set sync timings in GPMC */
	if (omap2_onenand_set_sync_mode(datap->cs, onenand_base, freq) < 0) {
		dev_err(dev, "Unable to set synchronous mode\n");
		return -EINVAL;
	}

	return 0;
}

void __init n800_flash_init(void)
{
	const struct omap_partition_config *part;
	int i = 0;

	n800_onenand_data.gpio_irq = cpu_is_omap34xx() ? 65 : 26;

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
