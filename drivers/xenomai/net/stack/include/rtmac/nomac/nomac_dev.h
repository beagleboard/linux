/***
 *
 *  include/rtmac/nomac/nomac_dev.h
 *
 *  RTmac - real-time networking media access control subsystem
 *  Copyright (C) 2002       Marc Kleine-Budde <kleine-budde@gmx.de>,
 *                2003, 2004 Jan Kiszka <Jan.Kiszka@web.de>
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

#ifndef __NOMAC_DEV_H_
#define __NOMAC_DEV_H_

#include <rtmac/nomac/nomac.h>

int nomac_dev_init(struct rtnet_device *rtdev, struct nomac_priv *nomac);

static inline void nomac_dev_release(struct nomac_priv *nomac)
{
	rtdm_dev_unregister(&nomac->api_device);
}

#endif /* __NOMAC_DEV_H_ */
