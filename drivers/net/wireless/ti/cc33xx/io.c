// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2008-2010 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#include "wlcore.h"
#include "debug.h"
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
	else {
		cc33xx_debug(DEBUG_CC33xx, "Could not set BLKsize");
	}
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

/* Raw target IO, address is not translated */
static int __must_check wlcore_raw_write(struct cc33xx *wl, int addr,
					 void *buf, size_t len, bool fixed)
{
	int ret;

	if (test_bit(CC33XX_FLAG_IO_FAILED, &wl->flags) ||
	    WARN_ON((test_bit(CC33XX_FLAG_IN_ELP, &wl->flags) &&
		     addr != HW_ACCESS_ELP_CTRL_REG)))
		return -EIO;

	ret = wl->if_ops->write(wl->dev, addr, buf, len, fixed);
	if (ret && wl->state != WLCORE_STATE_OFF)
		set_bit(CC33XX_FLAG_IO_FAILED, &wl->flags);

	return ret;
}

int __must_check wlcore_raw_read(struct cc33xx *wl, int addr,
				 void *buf, size_t len, bool fixed)
{
	int ret;

	if (test_bit(CC33XX_FLAG_IO_FAILED, &wl->flags) ||
	    WARN_ON((test_bit(CC33XX_FLAG_IN_ELP, &wl->flags) &&
		     addr != HW_ACCESS_ELP_CTRL_REG)))
		return -EIO;

	ret = wl->if_ops->read(wl->dev, addr, buf, len, fixed);
	if (ret && wl->state != WLCORE_STATE_OFF)
		set_bit(CC33XX_FLAG_IO_FAILED, &wl->flags);

	return ret;
}

int __must_check wlcore_write(struct cc33xx *wl, int addr,
			      void *buf, size_t len, bool fixed)
{
	return wlcore_raw_write(wl, addr, buf, len, fixed);
}

void claim_core_status_lock(struct cc33xx *wl)
{
	/* When accessing core-status data (read or write) the transport lock
	 * should be held. */
	wl->if_ops->interface_claim(wl->dev);
}

void release_core_status_lock(struct cc33xx *wl)
{
	/* After accessing core-status data (read or write) the transport lock
	 * should be released. */
	wl->if_ops->interface_release(wl->dev);
}

void cc33xx_power_off(struct cc33xx *wl)
{
	int ret = 0;

	if (!test_bit(CC33XX_FLAG_GPIO_POWER, &wl->flags))
		return;

	if (wl->if_ops->power)
		ret = wl->if_ops->power(wl->dev, false);
	if (!ret)
		clear_bit(CC33XX_FLAG_GPIO_POWER, &wl->flags);
}

int cc33xx_power_on(struct cc33xx *wl)
{
	int ret = 0;

	if (wl->if_ops->power)
		ret = wl->if_ops->power(wl->dev, true);
	if (ret == 0)
		set_bit(CC33XX_FLAG_GPIO_POWER, &wl->flags);

	return ret;
}
