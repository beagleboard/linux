/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 1998-2009 Texas Instruments. All rights reserved.
 * Copyright (C) 2008-2010 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#ifndef __IO_H__
#define __IO_H__

#include <linux/irqreturn.h>

struct cc33xx;

void wlcore_disable_interrupts_nosync(struct cc33xx *wl);
void wlcore_enable_interrupts(struct cc33xx *wl);

void cc33xx_io_reset(struct cc33xx *wl);
void cc33xx_io_init(struct cc33xx *wl);
int wlcore_translate_addr(struct cc33xx *wl, int addr);

/* Raw target IO, address is not translated */
static inline int __must_check wlcore_raw_write(struct cc33xx *wl, int addr,
						void *buf, size_t len,
						bool fixed)
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

static inline int __must_check wlcore_raw_read(struct cc33xx *wl, int addr,
					       void *buf, size_t len,
					       bool fixed)
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


static inline int __must_check wlcore_raw_read32(struct cc33xx *wl, int addr,
						 u32 *val)
{
	int ret;

	ret = wlcore_raw_read(wl, addr, wl->buffer_32,
			      sizeof(*wl->buffer_32), false);
	if (ret < 0)
		return ret;

	if (val)
		*val = le32_to_cpu(*wl->buffer_32);

	return 0;
}

static inline int __must_check wlcore_raw_write32(struct cc33xx *wl, int addr,
						  u32 val)
{
	*wl->buffer_32 = cpu_to_le32(val);
	return wlcore_raw_write(wl, addr, wl->buffer_32,
				sizeof(*wl->buffer_32), false);
}

static inline int __must_check wlcore_read(struct cc33xx *wl, int addr,
					   void *buf, size_t len, bool fixed)
{
	return wlcore_raw_read(wl, addr, buf, len, fixed);
}

static inline int __must_check wlcore_write(struct cc33xx *wl, int addr,
					    void *buf, size_t len, bool fixed)
{
	return wlcore_raw_write(wl, addr, buf, len, fixed);
}

static inline void claim_core_status_lock(struct cc33xx *wl)
{
	// When accessing core-status data (read or write) the transport lock
	// should be held.
	wl->if_ops->interface_claim(wl->dev);
}

static inline void release_core_status_lock(struct cc33xx *wl)
{
	// After accessing core-status data (read or write) the transport lock
	// should be released.
	wl->if_ops->interface_release(wl->dev);
}

static inline void cc33xx_power_off(struct cc33xx *wl)
{
	int ret = 0;

	if (!test_bit(CC33XX_FLAG_GPIO_POWER, &wl->flags))
		return;

	if (wl->if_ops->power)
		ret = wl->if_ops->power(wl->dev, false);
	if (!ret)
		clear_bit(CC33XX_FLAG_GPIO_POWER, &wl->flags);
}

static inline int cc33xx_power_on(struct cc33xx *wl)
{
	int ret = 0;

	if (wl->if_ops->power)
		ret = wl->if_ops->power(wl->dev, true);
	if (ret == 0)
		set_bit(CC33XX_FLAG_GPIO_POWER, &wl->flags);

	return ret;
}

bool cc33xx_set_block_size(struct cc33xx *wl);

/* Functions from main.c */

int cc33xx_tx_dummy_packet(struct cc33xx *wl);

#endif
