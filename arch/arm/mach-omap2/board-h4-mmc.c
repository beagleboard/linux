/*
 * linux/arch/arm/mach-omap2/board-h4-mmc.c
 *
 * Copyright (C) 2007 Instituto Nokia de Tecnologia - INdT
 * Authors: David Cohen <david.cohen@indt.org.br>
 *          Carlos Eduardo Aguiar <carlos.aguiar@indt.org.br>
 *
 * This code is based on linux/arch/arm/mach-omap2/board-n800-mmc.c, which is:
 * Copyright (C) 2006 Nokia Corporation
 * Author: Juha Yrjola
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/i2c/menelaus.h>

#include <asm/mach-types.h>

#include <mach/mmc.h>

#ifdef CONFIG_MMC_OMAP

/* Bit mask for slots detection interrupts */
#define SD1_CD_ST	(1 << 0)
#define SD2_CD_ST	(1 << 1)

static int slot1_cover_open;
static int slot2_cover_open;
static struct device *mmc_device;

/*
 * VMMC --> slot 1
 * VDCDC3_APE, VMCS2_APE --> slot 2
 */

static int h4_mmc_switch_slot(struct device *dev, int slot)
{
	int r = 0;

#ifdef CONFIG_MMC_DEBUG
	dev_dbg(dev, "Choose slot %d\n", slot + 1);
#endif
	if (slot == 0) {
		r = menelaus_enable_slot(2, 0);
		r |= menelaus_enable_slot(1, 1);
	} else {
		r = menelaus_enable_slot(1, 0);
		r |= menelaus_enable_slot(2, 1);
	}

	return r ? -ENODEV : 0;
}

static int h4_mmc_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	int mV = 0;

#ifdef CONFIG_MMC_DEBUG
	dev_dbg(dev, "Set slot %d power: %s (vdd %d)\n", slot + 1,
		power_on ? "on" : "off", vdd);
#endif
	if (slot == 0) {
		if (!power_on)
			return menelaus_set_vmmc(3000);
		switch (1 << vdd) {
			case MMC_VDD_33_34:
			case MMC_VDD_32_33:
			case MMC_VDD_31_32:
				mV = 3100;
				break;
			case MMC_VDD_30_31:
				mV = 3000;
				break;
			case MMC_VDD_28_29:
				mV = 2800;
				break;
			case MMC_VDD_165_195:
				mV = 1850;
				break;
			default:
				BUG();
		}
		return menelaus_set_vmmc(mV);
	} else {
		if (!power_on)
			return menelaus_set_vdcdc(3, 3000);
		switch (1 << vdd) {
			case MMC_VDD_33_34:
			case MMC_VDD_32_33:
				mV = 3300;
				break;
			case MMC_VDD_30_31:
			case MMC_VDD_29_30:
				mV = 3000;
				break;
			case MMC_VDD_28_29:
			case MMC_VDD_27_28:
				mV = 2800;
				break;
			case MMC_VDD_24_25:
			case MMC_VDD_23_24:
				mV = 2400;
				break;
			case MMC_VDD_22_23:
			case MMC_VDD_21_22:
				mV = 2200;
				break;
			case MMC_VDD_20_21:
				mV = 2000;
				break;
			case MMC_VDD_165_195:
				mV = 1800;
				break;
			default:
				BUG();
		}
		return menelaus_set_vdcdc(3, mV);
	}
	return 0;
}

static int h4_mmc_set_bus_mode(struct device *dev, int slot, int bus_mode)
{
	int r = 0;

#ifdef CONFIG_MMC_DEBUG
	dev_dbg(dev, "Set slot %d bus mode %s\n", slot + 1,
		bus_mode == MMC_BUSMODE_OPENDRAIN ? "open-drain" : "push-pull");
#endif
	BUG_ON(slot != 0 && slot != 1);
	slot++;
	switch (bus_mode) {
	case MMC_BUSMODE_OPENDRAIN:
		r = menelaus_set_mmc_opendrain(slot, 1);
		break;
	case MMC_BUSMODE_PUSHPULL:
		r = menelaus_set_mmc_opendrain(slot, 0);
		break;
	default:
		BUG();
	}
	if (r != 0 && printk_ratelimit()) {
		dev_err(dev, "MMC: unable to set bus mode for slot %d\n",
			slot);
	}
	return r;
}

static int h4_mmc_slot1_cover_state(struct device *dev, int slot)
{
	BUG_ON(slot != 0);
	return slot1_cover_open;
}

static int h4_mmc_slot2_cover_state(struct device *dev, int slot)
{
	BUG_ON(slot != 1);
	return slot2_cover_open;
}

static void h4_mmc_slot_callback(void *data, u8 card_mask)
{
	int cover_open;

	cover_open = (card_mask & SD1_CD_ST) ? 0 : 1;
	if (cover_open != slot1_cover_open) {
		slot1_cover_open = cover_open;
		omap_mmc_notify_cover_event(mmc_device, 0, slot1_cover_open);
	}

	cover_open = (card_mask & SD2_CD_ST) ? 0 : 1;
	if (cover_open != slot2_cover_open) {
		slot2_cover_open = cover_open;
		omap_mmc_notify_cover_event(mmc_device, 1, slot2_cover_open);
	}
}

static int h4_mmc_late_init(struct device *dev)
{
	int r;

	mmc_device = dev;

	r = menelaus_set_mmc_slot(1, 0, 0, 1);
	if (r < 0)
		goto out;
	r = menelaus_set_mmc_slot(2, 0, 0, 1);
	if (r < 0)
		goto out;

	r = menelaus_get_slot_pin_states();
	if (r < 0)
		goto out;

	if (r & SD1_CD_ST)
		slot1_cover_open = 1;
	else
		slot1_cover_open = 0;

	/* Slot pin bits seem to be inversed until first swith change,
	 * but just for slot 2
	 */
	if ((r == 0xf) || (r == (0xf & ~SD2_CD_ST)))
		r = ~r;

	if (r & SD2_CD_ST)
		slot2_cover_open = 1;
	else
		slot2_cover_open = 0;

	r = menelaus_register_mmc_callback(h4_mmc_slot_callback, NULL);

out:
	return r;
}

static void h4_mmc_cleanup(struct device *dev)
{
	menelaus_unregister_mmc_callback();
}

static struct omap_mmc_platform_data mmc1_data = {
	.nr_slots		= 2,
	.switch_slot		= h4_mmc_switch_slot,
	.init			= h4_mmc_late_init,
	.cleanup		= h4_mmc_cleanup,
	.dma_mask		= 0xffffffff,
	.slots[0] = {
		.wires		= 4,
		.set_power	= h4_mmc_set_power,
		.set_bus_mode	= h4_mmc_set_bus_mode,
		.get_cover_state= h4_mmc_slot1_cover_state,
		.ocr_mask	= MMC_VDD_165_195 |
				  MMC_VDD_28_29 | MMC_VDD_30_31 |
				  MMC_VDD_32_33 | MMC_VDD_33_34,
		.name		= "slot1",
	},
	.slots[1] = {
		.wires		= 4,
		.set_power	= h4_mmc_set_power,
		.set_bus_mode	= h4_mmc_set_bus_mode,
		.get_cover_state= h4_mmc_slot2_cover_state,
		.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_20_21 |
				  MMC_VDD_21_22 | MMC_VDD_22_23 | MMC_VDD_23_24 |
				  MMC_VDD_24_25 | MMC_VDD_27_28 | MMC_VDD_28_29 |
				  MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_32_33 |
				  MMC_VDD_33_34,
		.name		= "slot2",
	},
};

static struct omap_mmc_platform_data *mmc_data[OMAP24XX_NR_MMC];

void __init h4_mmc_init(void)
{
	mmc_data[0] = &mmc1_data;
	omap2_init_mmc(mmc_data, OMAP24XX_NR_MMC);
}

#else

void __init h4_mmc_init(void)
{
}

#endif

