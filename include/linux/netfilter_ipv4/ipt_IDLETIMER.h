/*
 * linux/include/linux/netfilter_ipv4/ipt_IDLETIMER.h
 *
 * Header file for IP tables timer target module.
 *
 * Copyright (C) 2004 Nokia Corporation
 * Written by Timo Teräs <ext-timo.teras@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _IPT_TIMER_H
#define _IPT_TIMER_H

struct ipt_idletimer_info {
	unsigned int timeout;
};

#endif
