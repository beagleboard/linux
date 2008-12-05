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
#include <mach/eac.h>

#include <mach/dsp_common.h>

void __init n800_audio_init(struct tsc2301_platform_data *tc)
{
}

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
