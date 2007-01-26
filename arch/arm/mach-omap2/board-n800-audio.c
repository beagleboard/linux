/*
 * linux/arch/arm/mach-omap2/board-n800-audio.c
 *
 * Copyright (C) 2006 Nokia Corporation
 * Contact: Juha Yrjola
 *          Jarkko Nikula <jarkko.nikula@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/spi/tsc2301.h>

#include <asm/io.h>
#include <asm/arch/eac.h>

#include "../plat-omap/dsp/dsp_common.h"

#if defined(CONFIG_SPI_TSC2301_AUDIO) && defined(CONFIG_SND_OMAP24XX_EAC)
#define AUDIO_ENABLED

static struct clk *sys_clkout2;
static struct clk *func96m_clk;
static struct device *eac_device;
static struct device *tsc2301_device;

static int enable_audio;
static int audio_ok;
static spinlock_t audio_lock;

/*
 * Leaving EAC and sys_clkout2 pins multiplexed to those subsystems results
 * in about 2 mA extra current leak when audios are powered down. The
 * workaround is to multiplex them to protected mode (with pull-ups enabled)
 * whenever audio is not being used.
 */
static int eac_mux_disabled = 0;
static int clkout2_mux_disabled = 0;
static u32 saved_mux[2];

static void n800_enable_eac_mux(void)
{
	if (!eac_mux_disabled)
		return;
	__raw_writel(saved_mux[1], IO_ADDRESS(0x48000124));
	eac_mux_disabled = 0;
}

static void n800_disable_eac_mux(void)
{
	if (eac_mux_disabled) {
		WARN_ON(eac_mux_disabled);
		return;
	}
	saved_mux[1] = __raw_readl(IO_ADDRESS(0x48000124));
	__raw_writel(0x1f1f1f1f, IO_ADDRESS(0x48000124));
	eac_mux_disabled = 1;
}

static void n800_enable_clkout2_mux(void)
{
	if (!clkout2_mux_disabled)
		return;
	__raw_writel(saved_mux[0], IO_ADDRESS(0x480000e8));
	clkout2_mux_disabled = 0;
}

static void n800_disable_clkout2_mux(void)
{
	u32 l;

	if (clkout2_mux_disabled) {
		WARN_ON(clkout2_mux_disabled);
		return;
	}
	saved_mux[0] = __raw_readl(IO_ADDRESS(0x480000e8));
	l = saved_mux[0] & ~0xff;
	l |= 0x1f;
	__raw_writel(l, IO_ADDRESS(0x480000e8));
	clkout2_mux_disabled = 1;
}

static int n800_eac_enable_ext_clocks(struct device *dev)
{
	BUG_ON(tsc2301_device == NULL);
	n800_enable_eac_mux();
	tsc2301_mixer_enable_mclk(tsc2301_device);

	return 0;
}

static void n800_eac_disable_ext_clocks(struct device *dev)
{
	BUG_ON(tsc2301_device == NULL);
	tsc2301_mixer_disable_mclk(tsc2301_device);
	n800_disable_eac_mux();
}

static int n800_audio_set_power(void *pdata, int dac, int adc)
{
	BUG_ON(pdata != tsc2301_device);
	tsc2301_mixer_set_power(tsc2301_device, dac, adc);

	return 0;
}

static int n800_audio_register_controls(void *pdata, struct snd_card *card)
{
	BUG_ON(pdata != tsc2301_device);
	return tsc2301_mixer_register_controls(tsc2301_device, card);
}

static struct eac_codec n800_eac_codec = {
	.mclk_src = EAC_MCLK_EXT_2x12288000,
	.codec_mode = EAC_CODEC_I2S,
	.codec_conf.i2s.polarity_changed_mode = 0,
	.codec_conf.i2s.sync_delay_enable = 0,
	.default_rate = 48000,
	.set_power = n800_audio_set_power,
	.register_controls = n800_audio_register_controls,
	.short_name = "TSC2301",
};

static int n800_register_codec(void)
{
	int r, do_enable = 0;
	unsigned long flags;

	n800_eac_codec.private_data = tsc2301_device;
	r = eac_register_codec(eac_device, &n800_eac_codec);
	if (r < 0)
		return r;
	spin_lock_irqsave(&audio_lock, flags);
	audio_ok = 1;
	if (enable_audio)
		do_enable = 1;
	spin_unlock_irqrestore(&audio_lock, flags);
	if (do_enable)
		eac_set_mode(eac_device, 1, 1);
	return 0;
}

static void n800_unregister_codec(void)
{
	audio_ok = 0;
	eac_unregister_codec(eac_device);
	eac_set_mode(eac_device, 0, 0);
}

static int n800_eac_init(struct device *dev)
{
	int r;

	BUG_ON(eac_device != NULL);
	eac_device = dev;
	if (tsc2301_device != NULL) {
		r = n800_register_codec();
		if (r < 0)
			return r;
	}

	return 0;
}

static void n800_eac_cleanup(struct device *dev)
{
	eac_device = NULL;
	if (tsc2301_device != NULL)
		n800_unregister_codec();
}

static int n800_codec_get_clocks(struct device *dev)
{
	sys_clkout2 = clk_get(dev, "sys_clkout2");
	if (IS_ERR(sys_clkout2)) {
		dev_err(dev, "Could not get sys_clkout2\n");
		return -ENODEV;
	}
	/* configure 12 MHz output on SYS_CLKOUT2. Therefore we must use
	 * 96 MHz as its parent in order to get 12 MHz */
	func96m_clk = clk_get(dev, "func_96m_ck");
	if (IS_ERR(func96m_clk)) {
		dev_err(dev, "Could not get func 96M clock\n");
		clk_put(sys_clkout2);
		return -ENODEV;
	}

	clk_set_parent(sys_clkout2, func96m_clk);
	clk_set_rate(sys_clkout2, 12000000);

	return 0;
}

static void n800_codec_put_clocks(struct device *dev)
{
	clk_put(func96m_clk);
	clk_put(sys_clkout2);
}

static int n800_codec_enable_clock(struct device *dev)
{
	n800_enable_clkout2_mux();
	return clk_enable(sys_clkout2);
}

static void n800_codec_disable_clock(struct device *dev)
{
	clk_disable(sys_clkout2);
	n800_disable_clkout2_mux();
}

static int n800_codec_init(struct device *dev)
{
	int r;

	BUG_ON(tsc2301_device != NULL);
	tsc2301_device = dev;
	if ((r = n800_codec_get_clocks(dev)) < 0)
		return r;
	if (eac_device != NULL) {
		r = n800_register_codec();
		if (r < 0) {
			n800_codec_put_clocks(dev);
			return r;
		}
	}
	return 0;
}

static void n800_codec_cleanup(struct device *dev)
{
	tsc2301_device = NULL;
	if (eac_device != NULL)
		n800_unregister_codec();
	n800_codec_put_clocks(dev);
}

static struct eac_platform_data n800_eac_data = {
	.init = n800_eac_init,
	.cleanup = n800_eac_cleanup,
	.enable_ext_clocks = n800_eac_enable_ext_clocks,
	.disable_ext_clocks = n800_eac_disable_ext_clocks,
};

static const struct tsc2301_mixer_gpio n800_mixer_gpios[] = {
	{
		.name			= "Headset Amplifier",
		.gpio			= 1,
		.deactivate_on_pd	= 1,
	}, {
		.name			= "Speaker Amplifier",
		.gpio			= 2,
		.def_enable		= 1,
		.deactivate_on_pd	= 1,
	}, {
		.name			= "Headset Mic Select",
		.gpio			= 3,
	}
};

static struct platform_device retu_headset_device = {
	.name		= "retu-headset",
	.id		= -1,
	.dev		= {
		.release	= NULL,
	},
};

void __init n800_audio_init(struct tsc2301_platform_data *tc)
{
	spin_lock_init(&audio_lock);

	if (platform_device_register(&retu_headset_device) < 0)
		return;
	omap_init_eac(&n800_eac_data);

	tc->pll_pdc = 7;
	tc->pll_a = 7;
	tc->pll_n = 9;
	tc->pll_output = 1;
	tc->mclk_ratio = TSC2301_MCLK_256xFS;
	tc->i2s_sample_rate = TSC2301_I2S_SR_48000;
	tc->i2s_format = TSC2301_I2S_FORMAT0;
	tc->power_down_blocks = TSC2301_REG_PD_MISC_MOPD;
	tc->mixer_gpios = n800_mixer_gpios;
	tc->n_mixer_gpios = ARRAY_SIZE(n800_mixer_gpios);
	tc->codec_init = n800_codec_init;
	tc->codec_cleanup = n800_codec_cleanup;
	tc->enable_clock = n800_codec_enable_clock;
	tc->disable_clock = n800_codec_disable_clock;
}

#else

void __init n800_audio_init(void)
{
}

#endif

#ifdef CONFIG_OMAP_DSP

int n800_audio_enable(struct dsp_kfunc_device *kdev, int stage)
{
#ifdef AUDIO_ENABLED
	unsigned long flags;
	int do_enable = 0;

	spin_lock_irqsave(&audio_lock, flags);

	pr_debug("DSP power up request (audio codec %sinitialized)\n",
		 audio_ok ? "" : "not ");

	if (enable_audio)
		goto out;
	enable_audio = 1;
	if (audio_ok)
		do_enable = 1;
out:
	spin_unlock_irqrestore(&audio_lock, flags);
	if (do_enable)
		eac_set_mode(eac_device, 1, 1);
#endif
	return 0;
}

int n800_audio_disable(struct dsp_kfunc_device *kdev, int stage)
{
#ifdef AUDIO_ENABLED
	unsigned long flags;
	int do_disable = 0;

	spin_lock_irqsave(&audio_lock, flags);

	pr_debug("DSP power down request (audio codec %sinitialized)\n",
		audio_ok ? "" : "not ");

	if (!enable_audio)
		goto out;
	enable_audio = 0;
	if (audio_ok)
		do_disable = 1;
out:
	spin_unlock_irqrestore(&audio_lock, flags);
	if (do_disable)
		eac_set_mode(eac_device, 0, 0);
#endif
	return 0;
}

#endif /* CONFIG_OMAP_DSP */
