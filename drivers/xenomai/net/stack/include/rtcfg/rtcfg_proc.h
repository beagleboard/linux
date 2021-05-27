/***
 *
 *  include/rtcfg/rtcfg_proc.c
 *
 *  Real-Time Configuration Distribution Protocol
 *
 *  Copyright (C) 2004 Jan Kiszka <jan.kiszka@web.de>
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

#ifndef __RTCFG_PROC_H_
#define __RTCFG_PROC_H_

#include <rtnet_internal.h>

#ifdef CONFIG_XENO_OPT_VFILE

extern struct mutex nrt_proc_lock;


void rtcfg_update_conn_proc_entries(int ifindex);
void rtcfg_remove_conn_proc_entries(int ifindex);

int rtcfg_init_proc(void);
void rtcfg_cleanup_proc(void);


static inline void rtcfg_lockwr_proc(int ifindex)
{
    mutex_lock(&nrt_proc_lock);
    rtcfg_remove_conn_proc_entries(ifindex);
}

static inline void rtcfg_unlockwr_proc(int ifindex)
{
    rtcfg_update_conn_proc_entries(ifindex);
    mutex_unlock(&nrt_proc_lock);
}

#else

#define rtcfg_lockwr_proc(x)    do {} while (0)
#define rtcfg_unlockwr_proc(x)  do {} while (0)

#endif /* CONFIG_XENO_OPT_VFILE */

#endif /* __RTCFG_PROC_H_ */
