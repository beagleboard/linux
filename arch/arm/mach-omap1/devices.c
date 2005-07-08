/*
 * linux/arch/arm/mach-omap1/devices.c
 *
 * OMAP1 platform device setup/initialization
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach/map.h>

#include <asm/arch/tc.h>
#include <asm/arch/board.h>
#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>


#if	defined(CONFIG_MMC_OMAP) || defined(CONFIG_MMC_OMAP_MODULE)

#define	OMAP_MMC1_BASE		0xfffb7800
#define	OMAP_MMC2_BASE		0xfffb7c00	/* omap16xx only */

static void mmc_release(struct device *dev)
{
	/* Nothing to release */
}

static struct omap_mmc_conf mmc1_conf;

static u64 mmc1_dmamask = 0xffffffff;

static struct resource mmc1_resources[] = {
	{
		.start		= IO_ADDRESS(OMAP_MMC1_BASE),
		.end		= IO_ADDRESS(OMAP_MMC1_BASE) + 0x7f,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_MMC,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device mmc_omap_device1 = {
	.name		= "mmci-omap",
	.id		= 1,
	.dev = {
		.release	= mmc_release,
		.dma_mask	= &mmc1_dmamask,
		.platform_data	= &mmc1_conf,
	},
	.num_resources	= ARRAY_SIZE(mmc1_resources),
	.resource	= mmc1_resources,
};

#ifdef	CONFIG_ARCH_OMAP16XX

static struct omap_mmc_conf mmc2_conf;

static u64 mmc2_dmamask = 0xffffffff;

static struct resource mmc2_resources[] = {
	{
		.start		= IO_ADDRESS(OMAP_MMC2_BASE),
		.end		= IO_ADDRESS(OMAP_MMC2_BASE) + 0x7f,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_1610_MMC2,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device mmc_omap_device2 = {
	.name		= "mmci-omap",
	.id		= 2,
	.dev = {
		.release	= mmc_release,
		.dma_mask	= &mmc2_dmamask,
		.platform_data	= &mmc2_conf,
	},
	.num_resources	= ARRAY_SIZE(mmc2_resources),
	.resource	= mmc2_resources,
};
#endif

static void __init omap_init_mmc(void)
{
	const struct omap_mmc_config	*mmc_conf;
	const struct omap_mmc_conf	*mmc;

	/* NOTE:  assumes MMC was never (wrongly) enabled */
	mmc_conf = omap_get_config(OMAP_TAG_MMC, struct omap_mmc_config);
	if (!mmc_conf)
		return;

	/* block 1 is always available and has just one pinout option */
	mmc = &mmc_conf->mmc[0];
	if (mmc->enabled) {
		omap_cfg_reg(MMC_CMD);
		omap_cfg_reg(MMC_CLK);
		omap_cfg_reg(MMC_DAT0);
		if (cpu_is_omap1710()) {
	              omap_cfg_reg(M15_1710_MMC_CLKI);
	              omap_cfg_reg(P19_1710_MMC_CMDDIR);
	              omap_cfg_reg(P20_1710_MMC_DATDIR0);
	        }
		if (mmc->wire4) {
			omap_cfg_reg(MMC_DAT1);
			/* NOTE:  DAT2 can be on W10 (here) or M15 */
			if (!mmc->nomux)
				omap_cfg_reg(MMC_DAT2);
			omap_cfg_reg(MMC_DAT3);
		}
		mmc1_conf = *mmc;
		(void) platform_device_register(&mmc_omap_device1);
	}

#ifdef	CONFIG_ARCH_OMAP16XX
	/* block 2 is on newer chips, and has many pinout options */
	mmc = &mmc_conf->mmc[1];
	if (mmc->enabled) {
		if (!mmc->nomux) {
			omap_cfg_reg(Y8_1610_MMC2_CMD);
			omap_cfg_reg(Y10_1610_MMC2_CLK);
			omap_cfg_reg(R18_1610_MMC2_CLKIN);
			omap_cfg_reg(W8_1610_MMC2_DAT0);
			if (mmc->wire4) {
				omap_cfg_reg(V8_1610_MMC2_DAT1);
				omap_cfg_reg(W15_1610_MMC2_DAT2);
				omap_cfg_reg(R10_1610_MMC2_DAT3);
			}

			/* These are needed for the level shifter */
			omap_cfg_reg(V9_1610_MMC2_CMDDIR);
			omap_cfg_reg(V5_1610_MMC2_DATDIR0);
			omap_cfg_reg(W19_1610_MMC2_DATDIR1);
		}

		/* Feedback clock must be set on OMAP-1710 MMC2 */
		if (cpu_is_omap1710())
			omap_writel(omap_readl(MOD_CONF_CTRL_1) | (1 << 24),
				     MOD_CONF_CTRL_1);
		mmc2_conf = *mmc;
		(void) platform_device_register(&mmc_omap_device2);
	}
#endif
	return;
}
#else
static inline void omap_init_mmc(void) {}
#endif

/*
 * This gets called after board-specific INIT_MACHINE, and initializes most
 * on-chip peripherals accessible on this board (except for few like USB):
 *
 *  (a) Does any "standard config" pin muxing needed.  Board-specific
 *	code will have muxed GPIO pins and done "nonstandard" setup;
 *	that code could live in the boot loader.
 *  (b) Populating board-specific platform_data with the data drivers
 *	rely on to handle wiring variations.
 *  (c) Creating platform devices as meaningful on this board and
 *	with this kernel configuration.
 *
 * Claiming GPIOs, and setting their direction and initial values, is the
 * responsibility of the device drivers.  So is responding to probe().
 *
 * Board-specific knowlege like creating devices or pin setup is to be
 * kept out of drivers as much as possible.  In particular, pin setup
 * may be handled by the boot loader, and drivers should expect it will
 * normally have been done by the time they're probed.
 */
static int __init omap_init_devices(void)
{
	omap_init_mmc();
	return 0;
}
arch_initcall(omap_init_devices);

