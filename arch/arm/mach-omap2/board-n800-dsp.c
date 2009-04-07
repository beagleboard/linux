/*
 * linux/arch/arm/mach-omap2/board-n800-dsp.c
 *
 * Copyright (C) 2006 Nokia Corporation.
 *
 * Contact: Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <mach/clock.h>
#include <mach/board.h>
#include <mach/dsp_common.h>

#if	defined(CONFIG_OMAP_DSP)

/*
 * dsp peripheral device: AUDIO
 */
static struct dsp_kfunc_device n800_audio_device = {
	.name	 = "audio",
	.type	 = DSP_KFUNC_DEV_TYPE_AUDIO,
	.enable	 = n800_audio_enable,
	.disable = n800_audio_disable,
};

/*
 * dsp peripheral device: TIMER
 */
static int dsp_timer_probe(struct dsp_kfunc_device *kdev, int stage)
{
	char clockname[20];

	strcpy(clockname, kdev->name);
	strcat(clockname, "_fck");

	kdev->fck = clk_get(NULL, clockname);
	if (IS_ERR(kdev->fck)) {
		printk(KERN_ERR "couldn't acquire %s\n", clockname);
		return PTR_ERR(kdev->fck);
	}
	pr_debug("%s probed successfully\n", clockname);

	strcpy(clockname, kdev->name);
	strcat(clockname, "_ick");
	kdev->ick = clk_get(NULL, clockname);
	if (IS_ERR(kdev->ick)) {
		printk(KERN_ERR "couldn't acquire %s\n", clockname);
		goto fail;
	}
	pr_debug("%s probed successfully\n", clockname);

	return 0;
 fail:
	clk_put(kdev->fck);

	return PTR_ERR(kdev->ick);
}

static int dsp_timer_remove(struct dsp_kfunc_device *kdev, int stage)
{
	clk_put(kdev->ick);
	clk_put(kdev->fck);
	pr_debug("%s removed successfully\n", kdev->name);
	return 0;
}

static int dsp_timer_enable(struct dsp_kfunc_device *kdev, int stage)
{
	pr_debug("%s enabled(%d)\n", kdev->name, stage);

	spin_lock(&kdev->lock);

	if (kdev->enabled)
		goto out;
	kdev->enabled = 1;

	clk_enable(kdev->fck);
	clk_enable(kdev->ick);
 out:
	spin_unlock(&kdev->lock);

	return 0;
}

static int dsp_timer_disable(struct dsp_kfunc_device *kdev, int stage)
{
	pr_debug("%s disabled(%d)\n", kdev->name, stage);

	spin_lock(&kdev->lock);

	if (kdev->enabled == 0)
		goto out;
	kdev->enabled = 0;

	clk_disable(kdev->ick);
	clk_disable(kdev->fck);
 out:
	spin_unlock(&kdev->lock);

	return 0;
}

static struct dsp_kfunc_device n800_timer_device = {
	.name	 = "gpt5",
	.type	 = DSP_KFUNC_DEV_TYPE_COMMON,
	.probe	 = dsp_timer_probe,
	.remove	 = dsp_timer_remove,
	.enable	 = dsp_timer_enable,
	.disable = dsp_timer_disable,
};

static struct dsp_kfunc_device *n800_kfunc_dev[] = {
	&n800_audio_device,
	&n800_timer_device,
};

void __init n800_dsp_init(void)
{
	int i, ret;
	struct dsp_kfunc_device **p = n800_kfunc_dev;

	for (i = 0; i < ARRAY_SIZE(n800_kfunc_dev); i++) {
		ret = dsp_kfunc_device_register(p[i]);
		if (ret) {
			printk(KERN_ERR
			       "KFUNC device registration failed: %s\n",
			       p[i]->name);
		}
	}
}

#else
void __init n800_dsp_init(void) { }
#endif	/* CONFIG_OMAP_DSP */
