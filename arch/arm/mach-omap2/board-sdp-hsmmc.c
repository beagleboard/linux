/*
 * linux/arch/arm/mach-omap2/board-sdp-hsmmc.c
 *
 * Copyright (C) 2007 Texas Instruments
 * Author: Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/hardware.h>
#include <asm/arch/twl4030.h>
#include <asm/arch/mmc.h>
#include <asm/arch/board.h>
#include <asm/io.h>

#if defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)

#define VMMC1_DEV_GRP		0x27
#define P1_DEV_GRP		0x20
#define VMMC1_DEDICATED		0x2A
#define VSEL_3V			0x02
#define VSEL_18V		0x00
#define TWL_GPIO_PUPDCTR1	0x13
#define TWL_GPIO_IMR1A		0x1C
#define TWL_GPIO_ISR1A		0x19
#define LDO_CLR			0x00
#define VSEL_S2_CLR		0x40
#define GPIO_0_BIT_POS		1 << 0
#define MMC1_CD_IRQ		0
#define MMC2_CD_IRQ		1

static int sdp_mmc_card_detect(int irq)
{
	return twl4030_get_gpio_datain(irq - IH_TWL4030_GPIO_BASE);
}

/*
 * MMC Slot Initialization.
 */
static int sdp_mmc_late_init(struct device *dev)
{
	int ret = 0;

	/*
	 * Configure TWL4030 GPIO parameters for MMC hotplug irq
	 */
	ret = twl4030_request_gpio(MMC1_CD_IRQ);
	if (ret != 0)
		goto err;

	ret = twl4030_set_gpio_edge_ctrl(MMC1_CD_IRQ,
			TWL4030_GPIO_EDGE_RISING | TWL4030_GPIO_EDGE_FALLING);
	if (ret != 0)
		goto err;

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x02,
						TWL_GPIO_PUPDCTR1);
	if (ret != 0)
		goto err;

	ret = twl4030_set_gpio_debounce(MMC1_CD_IRQ, TWL4030_GPIO_IS_ENABLE);
	if (ret != 0)
		goto err;

	return ret;
err:
	dev_err(dev, "Failed to configure TWL4030 GPIO IRQ\n");

	return ret;
}

static void sdp_mmc_cleanup(struct device *dev)
{
	int ret = 0;

	ret = twl4030_free_gpio(MMC1_CD_IRQ);
	if (ret != 0)
		dev_err(dev, "Failed to configure TWL4030 GPIO IRQ\n");
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
	if (ret != 0)
		goto err;

	reg = (mask == 1) ? (reg | GPIO_0_BIT_POS) : (reg & ~GPIO_0_BIT_POS);

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, reg, TWL_GPIO_IMR1A);
	if (ret != 0)
		goto err;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &reg, TWL_GPIO_ISR1A);
	if (ret != 0)
		goto err;

	reg = (mask == 1) ? (reg | GPIO_0_BIT_POS) : (reg & ~GPIO_0_BIT_POS);

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, reg, TWL_GPIO_ISR1A);
	if (ret != 0)
		goto err;
err:
	return ret;
}

static int sdp_mmc_suspend(struct device *dev, int slot)
{
	int ret = 0;

	disable_irq(TWL4030_GPIO_IRQ_NO(MMC1_CD_IRQ));
	ret = mask_cd_interrupt(1);

	return ret;
}

static int sdp_mmc_resume(struct device *dev, int slot)
{
	int ret = 0;

	enable_irq(TWL4030_GPIO_IRQ_NO(MMC1_CD_IRQ));
	ret = mask_cd_interrupt(0);

	return ret;
}

#endif

static int sdp_mmc_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	u32 vdd_sel = 0, devconf = 0, reg = 0;
	int ret = 0;

	/* REVISIT: Using address directly till the control.h defines
	 * are settled.
	 */
#if defined(CONFIG_ARCH_OMAP2430)
	#define OMAP2_CONTROL_PBIAS 0x490024A0
#else
	#define OMAP2_CONTROL_PBIAS 0x48002520
#endif

	if (power_on == 1) {
		if (cpu_is_omap24xx())
			devconf = omap_readl(0x490022E8);
		else
			devconf = omap_readl(0x48002274);

		switch (1 << vdd) {
		case MMC_VDD_33_34:
		case MMC_VDD_32_33:
			vdd_sel = VSEL_3V;
			if (cpu_is_omap24xx())
				devconf = (devconf | (1 << 31));
			break;
		case MMC_VDD_165_195:
			vdd_sel = VSEL_18V;
			if (cpu_is_omap24xx())
				devconf = (devconf & ~(1 << 31));
		}

		if (cpu_is_omap24xx())
			omap_writel(devconf, 0x490022E8);
		else
			omap_writel(devconf | 1 << 24, 0x48002274);

		omap_writel(omap_readl(OMAP2_CONTROL_PBIAS) | 1 << 2,
			OMAP2_CONTROL_PBIAS);
		omap_writel(omap_readl(OMAP2_CONTROL_PBIAS) & ~(1 << 1),
			OMAP2_CONTROL_PBIAS);

		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
						P1_DEV_GRP, VMMC1_DEV_GRP);
		if (ret != 0)
			goto err;

		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
						vdd_sel, VMMC1_DEDICATED);
		if (ret != 0)
			goto err;

		msleep(100);
		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg = (vdd_sel == VSEL_18V) ? ((reg | 0x6) & ~0x1)
						: (reg | 0x7);
		omap_writel(reg, OMAP2_CONTROL_PBIAS);

		return ret;

	} else if (power_on == 0) {
		/* Power OFF */

		/* For MMC1, Toggle PBIAS before every power up sequence */
		omap_writel(omap_readl(OMAP2_CONTROL_PBIAS) & ~(1 << 1),
					OMAP2_CONTROL_PBIAS);
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
						LDO_CLR, VMMC1_DEV_GRP);
		if (ret != 0)
			goto err;

		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
						VSEL_S2_CLR, VMMC1_DEDICATED);
		if (ret != 0)
			goto err;

		/* 100ms delay required for PBIAS configuration */
		msleep(100);
		omap_writel(omap_readl(OMAP2_CONTROL_PBIAS) | 0x7,
			OMAP2_CONTROL_PBIAS);
	} else {
		ret = -1;
		goto err;
	}

	return 0;
err:
	return 1;
}

static struct omap_mmc_platform_data sdp_mmc_data = {
	.nr_slots			= 1,
	.switch_slot			= NULL,
	.init				= sdp_mmc_late_init,
	.cleanup			= sdp_mmc_cleanup,
#ifdef CONFIG_PM
	.suspend			= sdp_mmc_suspend,
	.resume				= sdp_mmc_resume,
#endif
	.slots[0] = {
		.set_power		= sdp_mmc_set_power,
		.set_bus_mode		= NULL,
		.get_ro			= NULL,
		.get_cover_state	= NULL,
		.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34 |
						MMC_VDD_165_195,
		.name			= "first slot",

		.card_detect_irq        = TWL4030_GPIO_IRQ_NO(MMC1_CD_IRQ),
		.card_detect            = sdp_mmc_card_detect,
	},
};

void __init sdp_mmc_init(void)
{
	omap_set_mmc_info(1, &sdp_mmc_data);
}

#else

void __init sdp_mmc_init(void)
{

}

#endif
