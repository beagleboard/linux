/*
 * linux/arch/arm/mach-omap2/board-sdp-hsmmc.c
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Author: Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c/twl4030.h>

#include <mach/hardware.h>
#include <mach/control.h>
#include <mach/mmc.h>
#include <mach/board.h>

#if defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)

#define TWL_GPIO_IMR1A		0x1C
#define TWL_GPIO_ISR1A		0x19
#define LDO_CLR			0x00
#define VSEL_S2_CLR		0x40
#define GPIO_0_BIT_POS		(1 << 0)

#define VMMC1_DEV_GRP		0x27
#define VMMC1_DEV_GRP_P1	0x20
#define VMMC1_DEDICATED		0x2A
#define VMMC1_CLR		0x00
#define VMMC1_315V		0x03
#define VMMC1_300V		0x02
#define VMMC1_285V		0x01
#define VMMC1_185V		0x00

static u16 control_pbias_offset;

static struct hsmmc_controller {
	u16		control_devconf_offset;
	u32		devconf_loopback_clock;
	int		card_detect_gpio;
} hsmmc[] = {
	{
		.control_devconf_offset		= OMAP2_CONTROL_DEVCONF0,
		.devconf_loopback_clock		= OMAP2_MMCSDIO1ADPCLKISEL,
		.card_detect_gpio		= OMAP_MAX_GPIO_LINES,
	},
	{
		/* control_devconf_offset set dynamically */
		.devconf_loopback_clock		= OMAP2_MMCSDIO2ADPCLKISEL,
	},
};

static int hsmmc1_card_detect(int irq)
{
	return gpio_get_value_cansleep(hsmmc[0].card_detect_gpio);
}

/*
 * MMC Slot Initialization.
 */
static int hsmmc1_late_init(struct device *dev)
{
	int ret = 0;

	/*
	 * Configure TWL4030 GPIO parameters for MMC hotplug irq
	 */
	ret = gpio_request(hsmmc[0].card_detect_gpio, "mmc0_cd");
	if (ret)
		goto err;

	ret = twl4030_set_gpio_debounce(0, true);
	if (ret)
		goto err;

	return ret;

err:
	dev_err(dev, "Failed to configure TWL4030 GPIO IRQ\n");
	return ret;
}

static void hsmmc1_cleanup(struct device *dev)
{
	gpio_free(hsmmc[0].card_detect_gpio);
}

#ifdef CONFIG_PM

/*
 * To mask and unmask MMC Card Detect Interrupt
 * mask : 1
 * unmask : 0
 */
static int mask_cd_interrupt(int mask)
{
	u8 reg = 0, ret = 0;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &reg, TWL_GPIO_IMR1A);
	if (ret)
		goto err;

	reg = (mask == 1) ? (reg | GPIO_0_BIT_POS) : (reg & ~GPIO_0_BIT_POS);

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, reg, TWL_GPIO_IMR1A);
	if (ret)
		goto err;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &reg, TWL_GPIO_ISR1A);
	if (ret)
		goto err;

	reg = (mask == 1) ? (reg | GPIO_0_BIT_POS) : (reg & ~GPIO_0_BIT_POS);

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, reg, TWL_GPIO_ISR1A);
	if (ret)
		goto err;

err:
	return ret;
}

static int hsmmc1_suspend(struct device *dev, int slot)
{
	int ret = 0;

	disable_irq(TWL4030_GPIO_IRQ_NO(0));
	ret = mask_cd_interrupt(1);

	return ret;
}

static int hsmmc1_resume(struct device *dev, int slot)
{
	int ret = 0;

	enable_irq(TWL4030_GPIO_IRQ_NO(0));
	ret = mask_cd_interrupt(0);

	return ret;
}

#endif

static int hsmmc1_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	u32 reg;
	int ret = 0;
	u16 control_devconf_offset = hsmmc[0].control_devconf_offset;

	if (power_on) {
		u32 vdd_sel = 0;

		switch (1 << vdd) {
		case MMC_VDD_33_34:
		case MMC_VDD_32_33:
		case MMC_VDD_31_32:
		case MMC_VDD_30_31:
			vdd_sel = VMMC1_315V;
			break;
		case MMC_VDD_29_30:
			vdd_sel = VMMC1_300V;
			break;
		case MMC_VDD_165_195:
			vdd_sel = VMMC1_185V;
		}

		if (cpu_is_omap2430()) {
			reg = omap_ctrl_readl(OMAP243X_CONTROL_DEVCONF1);
			if (vdd_sel >= VMMC1_300V)
				reg |= OMAP243X_MMC1_ACTIVE_OVERWRITE;
			else
				reg &= ~OMAP243X_MMC1_ACTIVE_OVERWRITE;
			omap_ctrl_writel(reg, OMAP243X_CONTROL_DEVCONF1);
		}

		/* REVISIT: Loop back clock not needed for 2430? */
		if (!cpu_is_omap2430()) {
			reg = omap_ctrl_readl(control_devconf_offset);
			reg |= OMAP2_MMCSDIO1ADPCLKISEL;
			omap_ctrl_writel(reg, control_devconf_offset);
		}

		reg = omap_ctrl_readl(control_pbias_offset);
		reg |= OMAP2_PBIASSPEEDCTRL0;
		reg &= ~OMAP2_PBIASLITEPWRDNZ0;
		omap_ctrl_writel(reg, control_pbias_offset);

		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
						VMMC1_DEV_GRP_P1, VMMC1_DEV_GRP);
		if (ret)
			goto err;

		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
						vdd_sel, VMMC1_DEDICATED);
		if (ret)
			goto err;

		/* 100ms delay required for PBIAS configuration */
		msleep(100);

		reg = omap_ctrl_readl(control_pbias_offset);
		reg |= (OMAP2_PBIASLITEPWRDNZ0 | OMAP2_PBIASSPEEDCTRL0);
		if (vdd_sel == VMMC1_185V)
			reg &= ~OMAP2_PBIASLITEVMODE0;
		else
			reg |= OMAP2_PBIASLITEVMODE0;
		omap_ctrl_writel(reg, control_pbias_offset);

		return ret;

	} else {
		/* Power OFF */

		/* For MMC1, Toggle PBIAS before every power up sequence */
		reg = omap_ctrl_readl(control_pbias_offset);
		reg &= ~OMAP2_PBIASLITEPWRDNZ0;
		omap_ctrl_writel(reg, control_pbias_offset);

		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
						LDO_CLR, VMMC1_DEV_GRP);
		if (ret)
			goto err;

		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
						VSEL_S2_CLR, VMMC1_DEDICATED);
		if (ret)
			goto err;

		/* 100ms delay required for PBIAS configuration */
		msleep(100);

		reg = omap_ctrl_readl(control_pbias_offset);
		reg |= (OMAP2_PBIASSPEEDCTRL0 | OMAP2_PBIASLITEPWRDNZ0 |
			OMAP2_PBIASLITEVMODE0);
		omap_ctrl_writel(reg, control_pbias_offset);
	}

	return 0;

err:
	return 1;
}

static struct omap_mmc_platform_data mmc1_data = {
	.nr_slots			= 1,
	.init				= hsmmc1_late_init,
	.cleanup			= hsmmc1_cleanup,
#ifdef CONFIG_PM
	.suspend			= hsmmc1_suspend,
	.resume				= hsmmc1_resume,
#endif
	.dma_mask			= 0xffffffff,
	.slots[0] = {
		.wire4			= 1,
		.set_power		= hsmmc1_set_power,
		.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34 |
						MMC_VDD_165_195,
		.name			= "first slot",

		.card_detect_irq        = TWL4030_GPIO_IRQ_NO(0),
		.card_detect            = hsmmc1_card_detect,
	},
};

static struct omap_mmc_platform_data *hsmmc_data[OMAP34XX_NR_MMC];

void __init hsmmc_init(void)
{
	if (cpu_is_omap2430()) {
		control_pbias_offset = OMAP243X_CONTROL_PBIAS_LITE;
		hsmmc[1].control_devconf_offset = OMAP243X_CONTROL_DEVCONF1;
	} else {
		control_pbias_offset = OMAP343X_CONTROL_PBIAS_LITE;
		hsmmc[1].control_devconf_offset = OMAP343X_CONTROL_DEVCONF1;
	}

	hsmmc_data[0] = &mmc1_data;
	omap2_init_mmc(hsmmc_data, OMAP34XX_NR_MMC);
}

#else

void __init hsmmc_init(void)
{

}

#endif
