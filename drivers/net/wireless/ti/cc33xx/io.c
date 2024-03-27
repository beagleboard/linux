// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2008-2010 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>

#include "wlcore.h"
#include "debug.h"
#include "cc33xx_80211.h"
#include "io.h"
#include "tx.h"

bool cc33xx_set_block_size(struct cc33xx *wl)
{
	if (wl->if_ops->set_block_size) {
		wl->if_ops->set_block_size(wl->dev, CC33XX_BUS_BLOCK_SIZE);
		cc33xx_debug(DEBUG_CC33xx,
			"Set BLKsize to %d", CC33XX_BUS_BLOCK_SIZE);
		return true;
	}
	else
		cc33xx_debug(DEBUG_CC33xx, "Could not set BLKsize");
	return false;
}

void wlcore_disable_interrupts_nosync(struct cc33xx *wl)
{
	wl->if_ops->disable_irq(wl->dev);
}

void wlcore_irq(void *cookie);
void wlcore_enable_interrupts(struct cc33xx *wl)
{
	wl->if_ops->enable_irq(wl->dev);

	printk(KERN_DEBUG "IBI_WA: Read core status");
	wlcore_irq(wl);
	printk(KERN_DEBUG "IBI_WA: Core status processed");
}

void cc33xx_io_reset(struct cc33xx *wl)
{
	if (wl->if_ops->reset)
		wl->if_ops->reset(wl->dev);
}

void cc33xx_io_init(struct cc33xx *wl)
{
	if (wl->if_ops->init)
		wl->if_ops->init(wl->dev);
}
