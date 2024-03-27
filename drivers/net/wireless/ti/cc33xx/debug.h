/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2011 Texas Instruments. All rights reserved.
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <coelho@ti.com>
 */

#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <linux/bitops.h>
#include <linux/printk.h>

#define DRIVER_NAME "wlcore"
#define DRIVER_PREFIX DRIVER_NAME ": "

enum {
	DEBUG_NONE	= 0,
	DEBUG_IRQ	= BIT(0),
	DEBUG_SPI	= BIT(1),
	DEBUG_BOOT	= BIT(2),
	DEBUG_CORE_STATUS = BIT(3),
	DEBUG_TESTMODE	= BIT(4),
	DEBUG_EVENT	= BIT(5),
	DEBUG_TX	= BIT(6),
	DEBUG_RX	= BIT(7),
	DEBUG_SCAN	= BIT(8),
	DEBUG_CRYPT	= BIT(9),
	DEBUG_PSM	= BIT(10),
	DEBUG_MAC80211	= BIT(11),
	DEBUG_CMD	= BIT(12),
	DEBUG_ACX	= BIT(13),
	DEBUG_SDIO	= BIT(14),
	DEBUG_FILTERS   = BIT(15),
	DEBUG_ADHOC     = BIT(16),
	DEBUG_AP	= BIT(17),
	DEBUG_PROBE	= BIT(18),
	DEBUG_IO	= BIT(19),
	DEBUG_MASTER	= (DEBUG_ADHOC | DEBUG_AP),
	DEBUG_CC33xx = BIT(20),
	DEBUG_ALL	= ~0,
	DEBUG_NO_DATAPATH = (DEBUG_ALL & ~DEBUG_IRQ & ~DEBUG_TX & ~DEBUG_RX & ~DEBUG_CORE_STATUS),
};

extern u32 cc33xx_debug_level;

#define DEBUG_DUMP_LIMIT 1024

#define cc33xx_error(fmt, arg...) \
	pr_err(DRIVER_PREFIX "ERROR " fmt "\n", ##arg)

#define cc33xx_warning(fmt, arg...) \
	pr_warn(DRIVER_PREFIX "WARNING " fmt "\n", ##arg)

#define cc33xx_notice(fmt, arg...) \
	pr_info(DRIVER_PREFIX fmt "\n", ##arg)

#define cc33xx_info(fmt, arg...) \
	pr_info(DRIVER_PREFIX fmt "\n", ##arg)

/* define the debug macro differently if dynamic debug is supported */
#if defined(CONFIG_DYNAMIC_DEBUG)
#define cc33xx_debug(level, fmt, arg...) \
	do { \
		if (unlikely(level & cc33xx_debug_level)) \
			dynamic_pr_debug(DRIVER_PREFIX fmt "\n", ##arg); \
	} while (0)
#else
#define cc33xx_debug(level, fmt, arg...) \
	do { \
		if (unlikely(level & cc33xx_debug_level)) \
			printk(KERN_DEBUG pr_fmt(DRIVER_PREFIX fmt "\n"), \
			       ##arg); \
	} while (0)
#endif

#define cc33xx_dump(level, prefix, buf, len)				      \
	do {								      \
		if (level & cc33xx_debug_level)				      \
			print_hex_dump_debug(DRIVER_PREFIX prefix,	      \
					DUMP_PREFIX_OFFSET, 16, 1,	      \
					buf,				      \
					min_t(size_t, len, DEBUG_DUMP_LIMIT), \
					0);				      \
	} while (0)

#define cc33xx_dump_ascii(level, prefix, buf, len)			      \
	do {								      \
		if (level & cc33xx_debug_level)				      \
			print_hex_dump_debug(DRIVER_PREFIX prefix,	      \
					DUMP_PREFIX_OFFSET, 16, 1,	      \
					buf,				      \
					min_t(size_t, len, DEBUG_DUMP_LIMIT), \
					true);				      \
	} while (0)

#endif /* __DEBUG_H__ */
