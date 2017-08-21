/*
 * Copyright (C) 2009 Wolfgang Grandegger <wg@denx.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __RTCAN_MSCAN_H_
#define __RTCAN_MSCAN_H_

#define RTCAN_DEV_NAME    "rtcan%d"
#define RTCAN_DRV_NAME    "rtcan_mscan"

/* MSCAN type variants */
enum {
	MSCAN_TYPE_MPC5200,
	MSCAN_TYPE_MPC5121
};

extern int rtcan_mscan_register(struct rtcan_device *dev, int irq,
				       int mscan_clksrc);
extern int rtcan_mscan_unregister(struct rtcan_device *dev);

extern int rtcan_mscan_create_proc(struct rtcan_device* dev);
extern void rtcan_mscan_remove_proc(struct rtcan_device* dev);

#endif /* __RTCAN_MSCAN_H_ */
