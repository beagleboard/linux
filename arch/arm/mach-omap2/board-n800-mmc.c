/*
 * linux/arch/arm/mach-omap2/board-n800-mmc.c
 *
 * Copyright (C) 2006 Nokia Corporation
 * Author: Juha Yrjola
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/arch/mmc.h>
#include <asm/arch/menelaus.h>
#include <asm/arch/gpio.h>

#ifdef CONFIG_MMC_OMAP

static const int slot_switch_gpio = 96;
static const int slot1_wp_gpio = 23;
static const int slot2_wp_gpio = 8;
static int slot1_cover_closed;
static int slot2_cover_closed;
static struct device *mmc_device;

/*
 * VMMC --> slot 1
 * VDCDC3_APE, VMCS2_APE --> slot 2
 * GPIO96 --> Menelaus GPIO2
 */

static int n800_mmc_switch_slot(struct device *dev, int slot)
{
#ifdef CONFIG_MMC_DEBUG
	dev_dbg(dev, "Choose slot %d\n", slot + 1);
#endif
	if (slot == 0)
		omap_set_gpio_dataout(slot_switch_gpio, 0);
	else
		omap_set_gpio_dataout(slot_switch_gpio, 1);
	return 0;
}

static int n800_mmc_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	int mV;

#ifdef CONFIG_MMC_DEBUG
	dev_dbg(dev, "Set slot %d power: %s (vdd %d)\n", slot + 1,
		power_on ? "on" : "off", vdd);
#endif
	if (slot == 0) {
		if (!power_on)
			return menelaus_set_vmmc(0);
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
		case MMC_VDD_18_19:
			mV = 1850;
			break;
		default:
			BUG();
		}
		return menelaus_set_vmmc(mV);
	} else {
		if (!power_on)
			return menelaus_set_vdcdc(3, 0);
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
		case MMC_VDD_19_20:
			mV = 2000;
			break;
		case MMC_VDD_18_19:
		case MMC_VDD_17_18:
			mV = 1800;
			break;
		case MMC_VDD_150_155:
		case MMC_VDD_145_150:
			mV = 1500;
			break;
		default:
			BUG();
		}
		return menelaus_set_vdcdc(3, mV);
	}
	return 0;
}

static int n800_mmc_set_bus_mode(struct device *dev, int slot, int bus_mode)
{
	int r;

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
	if (r != 0 && printk_ratelimit())
		dev_err(dev, "MMC: unable to set bus mode for slot %d\n",
			slot);
	return r;
}

#if 0
static int n800_mmc_get_ro(struct device *dev, int slot)
{
	int ro;

	slot++;
	if (slot == 1)
		ro = omap_get_gpio_datain(slot1_wp_gpio);
	else
		ro = omap_get_gpio_datain(slot2_wp_gpio);
#ifdef CONFIG_MMC_DEBUG
	dev_dbg(dev, "Get RO slot %d: %s\n",
		slot, ro ? "read-only" : "read-write");
#endif
	return ro;
}
#endif

static int n800_mmc_get_cover_state(struct device *dev, int slot)
{
	slot++;
	BUG_ON(slot != 1 && slot != 2);
	if (slot == 1)
		return slot1_cover_closed;
	else
		return slot2_cover_closed;
}

static void n800_mmc_callback(void *data, u8 card_mask)
{
	if (card_mask & (1 << 1))
		slot2_cover_closed = 0;
	else
		slot2_cover_closed = 1;
        omap_mmc_notify_cover_event(mmc_device, 1, slot2_cover_closed);
}

void n800_mmc_slot1_cover_handler(void *arg, int state)
{
	if (mmc_device == NULL)
		return;

	slot1_cover_closed = state;
	omap_mmc_notify_cover_event(mmc_device, 0, state);
}

static int n800_mmc_late_init(struct device *dev)
{
	int r;

	mmc_device = dev;

	r = menelaus_set_slot_sel(1);
	if (r < 0)
		return r;

	r = menelaus_set_mmc_slot(1, 1, 0, 1);
	if (r < 0)
		return r;
	r = menelaus_set_mmc_slot(2, 1, 0, 1);
	if (r < 0)
		return r;

	r = menelaus_get_slot_pin_states();
	if (r < 0)
		return r;

	if (r & (1 << 1))
		slot2_cover_closed = 1;
	else
		slot2_cover_closed = 0;

	r = menelaus_register_mmc_callback(n800_mmc_callback, NULL);

	return r;
}

static void n800_mmc_cleanup(struct device *dev)
{
	menelaus_unregister_mmc_callback();
}

static struct omap_mmc_platform_data n800_mmc_data = {
	.enabled		= 1,
	.nr_slots		= 2,
	.wire4			= 1,
	.switch_slot		= n800_mmc_switch_slot,
	.init			= n800_mmc_late_init,
	.cleanup		= n800_mmc_cleanup,
	.slots[0] = {
		.set_power	= n800_mmc_set_power,
		.set_bus_mode	= n800_mmc_set_bus_mode,
		.get_ro		= NULL,
		.get_cover_state= n800_mmc_get_cover_state,
		.ocr_mask	= MMC_VDD_18_19 | MMC_VDD_28_29 | MMC_VDD_30_31 |
				  MMC_VDD_32_33 | MMC_VDD_33_34,
		.name		= "internal",
	},
	.slots[1] = {
		.set_power	= n800_mmc_set_power,
		.set_bus_mode	= n800_mmc_set_bus_mode,
		.get_ro		= NULL,
		.get_cover_state= n800_mmc_get_cover_state,
		.ocr_mask	= MMC_VDD_150_155 | MMC_VDD_145_150 | MMC_VDD_17_18 |
				  MMC_VDD_18_19 | MMC_VDD_19_20 | MMC_VDD_20_21 |
				  MMC_VDD_21_22 | MMC_VDD_22_23 | MMC_VDD_23_24 |
				  MMC_VDD_24_25 | MMC_VDD_27_28 | MMC_VDD_28_29 |
				  MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_32_33 |
				  MMC_VDD_33_34,
		.name		= "external",
	},
};

void __init n800_mmc_init(void)
{
	omap_set_mmc_info(1, &n800_mmc_data);
	if (omap_request_gpio(slot_switch_gpio) < 0)
		BUG();
	omap_set_gpio_dataout(slot_switch_gpio, 0);
	omap_set_gpio_direction(slot_switch_gpio, 0);
	if (omap_request_gpio(slot1_wp_gpio) < 0)
		BUG();
	if (omap_request_gpio(slot2_wp_gpio) < 0)
		BUG();
	omap_set_gpio_direction(slot1_wp_gpio, 1);
	omap_set_gpio_direction(slot2_wp_gpio, 1);
}

#else

void __init n800_mmc_init(void)
{
}

void n800_mmc_slot1_cover_handler(void *arg, int state)
{
}

#endif
