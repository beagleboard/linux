/***
 *
 *  include/rtmac/nomac/nomac.h
 *
 *  RTmac - real-time networking media access control subsystem
 *  Copyright (C) 2002      Marc Kleine-Budde <kleine-budde@gmx.de>,
 *                2003-2005 Jan Kiszka <Jan.Kiszka@web.de>
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

#ifndef __NOMAC_H_
#define __NOMAC_H_

#include <rtdm/driver.h>

#include <rtmac/rtmac_disc.h>


#define RTMAC_TYPE_NOMAC        0

#define NOMAC_MAGIC             0x004D0A0C


struct nomac_priv {
    unsigned int                magic;
    struct rtnet_device         *rtdev;
    char                        device_name[32];
    struct rtdm_driver          api_driver;
    struct rtdm_device          api_device;
    /* ... */

#ifdef CONFIG_XENO_OPT_VFILE
    struct list_head            list_entry;
#endif
};


extern struct rtmac_disc        nomac_disc;

#endif /* __NOMAC_H_ */
