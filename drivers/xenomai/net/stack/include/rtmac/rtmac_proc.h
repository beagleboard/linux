/***
 *
 *  include/rtmac/rtmac_proc.h
 *
 *  rtmac - real-time networking medium access control subsystem
 *  Copyright (C) 2002 Marc Kleine-Budde <kleine-budde@gmx.de>
 *                2004 Jan Kiszka <jan.kiszka@web.de>
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

#ifndef __RTMAC_PROC_H_
#define __RTMAC_PROC_H_


int rtmac_disc_proc_register(struct rtmac_disc *disc);
void rtmac_disc_proc_unregister(struct rtmac_disc *disc);

int rtmac_proc_register(void);
void rtmac_proc_release(void);


#endif /* __RTMAC_PROC_H_ */
