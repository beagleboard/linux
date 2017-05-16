/***
 *
 *  include/rtmac/tdma/tdma_dev.h
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

#ifndef __TDMA_DEV_H_
#define __TDMA_DEV_H_

#include <rtmac/tdma/tdma.h>


int tdma_dev_init(struct rtnet_device *rtdev, struct tdma_priv *tdma);


static inline void tdma_dev_release(struct tdma_priv *tdma)
{
    rtdm_dev_unregister(&tdma->api_device);
}

#endif /* __TDMA_DEV_H_ */
