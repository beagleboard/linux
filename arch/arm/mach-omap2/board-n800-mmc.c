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

#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c/menelaus.h>

#include <asm/mach-types.h>

#include <mach/mmc.h>

#if defined(CONFIG_MMC_OMAP) || defined(CONFIG_MMC_OMAP_MODULE)

static const int slot_switch_gpio = 96;

static const int n810_slot2_pw_vddf = 23;
static const int n810_slot2_pw_vdd = 9;

static int slot1_cover_open;
static int slot2_cover_open;
static struct device *mmc_device;

/*
 * VMMC   --> slot 1 (N800 & N810)
 * VDCDC3_APE, VMCS2_APE --> slot 2 on N800
 * GPIO96 --> Menelaus GPIO2
 * GPIO23 --> controls slot2 VSD    (N810 only)
 * GPIO9  --> controls slot2 VIO_SD (N810 only)
 */

static int n800_mmc_switch_slot(struct device *dev, int slot)
{
#ifdef CONFIG_MMC_DEBUG
	dev_dbg(dev, "Choose slot %d\n", slot + 1);
#endif
	gpio_set_value(slot_switch_gpio, slot);
	return 0;
}

static int n800_mmc_set_power_menelaus(struct device *dev, int slot,
					int power_on, int vdd)
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
		case MMC_VDD_165_195:
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

static void nokia_mmc_set_power_internal(struct device *dev,
					 int power_on)
{
	dev_dbg(dev, "Set internal slot power %s\n",
		power_on ? "on" : "off");

	if (power_on) {
		gpio_set_value(n810_slot2_pw_vddf, 1);
		udelay(30);
		gpio_set_value(n810_slot2_pw_vdd, 1);
		udelay(100);
	} else {
		gpio_set_value(n810_slot2_pw_vdd, 0);
		msleep(50);
		gpio_set_value(n810_slot2_pw_vddf, 0);
		msleep(50);
	}
}

static int n800_mmc_set_power(struct device *dev, int slot, int power_on,
			      int vdd)
{
	if (machine_is_nokia_n800() || slot == 0)
		return n800_mmc_set_power_menelaus(dev, slot, power_on, vdd);

	nokia_mmc_set_power_internal(dev, power_on);

	return 0;
}

static int n800_mmc_set_bus_mode(struct device *dev, int slot, int bus_mode)
{
	int r;

	dev_dbg(dev, "Set slot %d bus mode %s\n", slot + 1,
		bus_mode == MMC_BUSMODE_OPENDRAIN ? "open-drain" : "push-pull");
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

static int n800_mmc_get_cover_state(struct device *dev, int slot)
{
	slot++;
	BUG_ON(slot != 1 && slot != 2);
	if (slot == 1)
		return slot1_cover_open;
	else
		return slot2_cover_open;
}

static void n800_mmc_callback(void *data, u8 card_mask)
{
	int bit, *openp, index;

	if (machine_is_nokia_n800()) {
		bit = 1 << 1;
		openp = &slot2_cover_open;
		index = 1;
	} else {
		bit = 1;
		openp = &slot1_cover_open;
		index = 0;
	}

	if (card_mask & bit)
		*openp = 1;
	else
		*openp = 0;

	omap_mmc_notify_cover_event(mmc_device, index, *openp);
}

void n800_mmc_slot1_cover_handler(void *arg, int closed_state)
{
	if (mmc_device == NULL)
		return;

	slot1_cover_open = !closed_state;
	omap_mmc_notify_cover_event(mmc_device, 0, closed_state);
}

static int n800_mmc_late_init(struct device *dev)
{
	int r, bit, *openp;
	int vs2sel;

	mmc_device = dev;

	r = menelaus_set_slot_sel(1);
	if (r < 0)
		return r;

	if (machine_is_nokia_n800())
		vs2sel = 0;
	else
		vs2sel = 2;

	r = menelaus_set_mmc_slot(2, 0, vs2sel, 1);
	if (r < 0)
		return r;

	n800_mmc_set_power(dev, 0, MMC_POWER_ON, 16); /* MMC_VDD_28_29 */
	n800_mmc_set_power(dev, 1, MMC_POWER_ON, 16);

	r = menelaus_set_mmc_slot(1, 1, 0, 1);
	if (r < 0)
		return r;
	r = menelaus_set_mmc_slot(2, 1, vs2sel, 1);
	if (r < 0)
		return r;

	r = menelaus_get_slot_pin_states();
	if (r < 0)
		return r;

	if (machine_is_nokia_n800()) {
		bit = 1 << 1;
		openp = &slot2_cover_open;
	} else {
		bit = 1;
		openp = &slot1_cover_open;
		slot2_cover_open = 0;
	}

	/* All slot pin bits seem to be inversed until first swith change */
	if (r == 0xf || r == (0xf & ~bit))
		r = ~r;

	if (r & bit)
		*openp = 1;
	else
		*openp = 0;

	r = menelaus_register_mmc_callback(n800_mmc_callback, NULL);

	return r;
}

static void n800_mmc_shutdown(struct device *dev)
{
	int vs2sel;

	if (machine_is_nokia_n800())
		vs2sel = 0;
	else
		vs2sel = 2;

	menelaus_set_mmc_slot(1, 0, 0, 0);
	menelaus_set_mmc_slot(2, 0, vs2sel, 0);
}

static void n800_mmc_cleanup(struct device *dev)
{
	menelaus_unregister_mmc_callback();

	gpio_free(slot_switch_gpio);

	if (machine_is_nokia_n810()) {
		gpio_free(n810_slot2_pw_vddf);
		gpio_free(n810_slot2_pw_vdd);
	}
}

/*
 * MMC controller1 has two slots that are multiplexed via I2C.
 * MMC controller2 is not in use.
 */
static struct omap_mmc_platform_data mmc1_data = {
	.nr_slots		= 2,
	.switch_slot		= n800_mmc_switch_slot,
	.init			= n800_mmc_late_init,
	.cleanup		= n800_mmc_cleanup,
	.shutdown		= n800_mmc_shutdown,
	.max_freq               = 24000000,
	.dma_mask		= 0xffffffff,
	.slots[0] = {
		.wires		= 4,
		.set_power	= n800_mmc_set_power,
		.set_bus_mode	= n800_mmc_set_bus_mode,
		.get_cover_state= n800_mmc_get_cover_state,
		.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_30_31 |
				  MMC_VDD_32_33   | MMC_VDD_33_34,
		.name		= "internal",
	},
	.slots[1] = {
		.set_power	= n800_mmc_set_power,
		.set_bus_mode	= n800_mmc_set_bus_mode,
		.get_cover_state= n800_mmc_get_cover_state,
		.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_20_21 |
				  MMC_VDD_21_22 | MMC_VDD_22_23 | MMC_VDD_23_24 |
				  MMC_VDD_24_25 | MMC_VDD_27_28 | MMC_VDD_28_29 |
				  MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_32_33 |
				  MMC_VDD_33_34,
		.name		= "external",
	},
};

static struct omap_mmc_platform_data *mmc_data[OMAP24XX_NR_MMC];

void __init n800_mmc_init(void)

{
	if (machine_is_nokia_n810()) {
		mmc1_data.slots[0].name = "external";

		/*
		 * Some Samsung Movinand chips do not like open-ended
		 * multi-block reads and fall to braind-dead state
		 * while doing so. Reducing the number of blocks in
		 * the transfer or delays in clock disable do not help
		 */
		mmc1_data.slots[1].name = "internal";
		mmc1_data.slots[1].ban_openended = 1;
	}

	if (gpio_request(slot_switch_gpio, "MMC slot switch") < 0)
		BUG();
	gpio_direction_output(slot_switch_gpio, 0);

	if (machine_is_nokia_n810()) {
		if (gpio_request(n810_slot2_pw_vddf, "MMC slot 2 Vddf") < 0)
			BUG();
		gpio_direction_output(n810_slot2_pw_vddf, 0);

		if (gpio_request(n810_slot2_pw_vdd, "MMC slot 2 Vdd") < 0)
			BUG();
		gpio_direction_output(n810_slot2_pw_vdd, 0);
	}

	mmc_data[0] = &mmc1_data;
	omap2_init_mmc(mmc_data, OMAP24XX_NR_MMC);
}
#else

void __init n800_mmc_init(void)
{
}

void n800_mmc_slot1_cover_handler(void *arg, int state)
{
}

#endif
