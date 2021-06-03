/***
 *
 *  include/ipv4/ip_sock.h
 *
 *  RTnet - real-time networking subsystem
 *  Copyright (C) 2003-2005 Jan Kiszka <jan.kiszka@web.de>
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

#ifndef __RTNET_IP_SOCK_H_
#define __RTNET_IP_SOCK_H_

#include <rtnet_socket.h>

extern int rt_ip_ioctl(struct rtdm_fd *fd, int request, void *arg);

#endif /* __RTNET_IP_SOCK_H_ */
