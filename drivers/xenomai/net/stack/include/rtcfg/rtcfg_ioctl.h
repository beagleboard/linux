/***
 *
 *  include/rtcfg/rtcfg_ioctl.h
 *
 *  Real-Time Configuration Distribution Protocol
 *
 *  Copyright (C) 2003, 2004 Jan Kiszka <jan.kiszka@web.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __RTCFG_IOCTL_H_
#define __RTCFG_IOCTL_H_

extern struct rtnet_ioctls rtcfg_ioctls;

#define rtcfg_init_ioctls() rtnet_register_ioctls(&rtcfg_ioctls)
#define rtcfg_cleanup_ioctls() rtnet_unregister_ioctls(&rtcfg_ioctls)

#endif /* __RTCFG_IOCTL_H_ */
