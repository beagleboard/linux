/*
 * omap_ts.h - header file for OMAP touchscreen support
 * 
 * Copyright (c) 2002 MontaVista Software Inc.
 * Copyright (c) 2004 Texas Instruments, Inc.
 *
 * Assembled using driver code copyright the companies above.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef __OMAP_TS_H
#define __OMAP_TS_H

#ifdef DEBUG
#define DEBUG_TS(fmt...)   printk(fmt)
#else
#define DEBUG_TS(fmt...)   do { } while (0)
#endif

struct omap_ts_t;

struct ts_device {
        int  (*probe)   (struct omap_ts_t *);
        void (*read)    (u16 *);
        void (*enable)  (void);
        void (*disable) (void);
        void (*remove)  (void);
        int  (*penup)  (void);
};

struct omap_ts_t{
	struct input_dev * inputdevice;
	struct timer_list ts_timer;      // Timer for triggering acquisitions
	int touched;
	int irq;
	int irq_type;
	int irq_enabled;
	struct ts_device *dev;
	spinlock_t lock;
};

extern struct ts_device hx_ts;

#endif /* __OMAP_TS_H */
