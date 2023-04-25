// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of wl1271
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
#include "wl12xx_80211.h"
#include "io.h"
#include "tx.h"

bool wl1271_set_block_size(struct wl1271 *wl)
{
	if (wl->if_ops->set_block_size) {
		wl->if_ops->set_block_size(wl->dev, CC33XX_BUS_BLOCK_SIZE);
		cc33xx_debug(DEBUG_OSPREY, 
			"Set BLKsize to %d", CC33XX_BUS_BLOCK_SIZE);
		return true;
	}
	else
		cc33xx_debug(DEBUG_OSPREY, "Could not set BLKsize");
	return false;
}

void wlcore_disable_interrupts_nosync(struct wl1271 *wl)
{
	wl->if_ops->disable_irq(wl->dev);
}

void wlcore_enable_interrupts(struct wl1271 *wl)
{
	wl->if_ops->enable_irq(wl->dev);
}

void wl1271_io_reset(struct wl1271 *wl)
{
	if (wl->if_ops->reset)
		wl->if_ops->reset(wl->dev);
}

void wl1271_io_init(struct wl1271 *wl)
{
	if (wl->if_ops->init)
		wl->if_ops->init(wl->dev);
}
