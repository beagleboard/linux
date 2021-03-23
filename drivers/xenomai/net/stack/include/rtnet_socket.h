/***
 *
 *  include/rtnet_socket.h
 *
 *  RTnet - real-time networking subsystem
 *  Copyright (C) 1999       Lineo, Inc
 *                1999, 2002 David A. Schleef <ds@schleef.org>
 *                2002       Ulrich Marx <marx@kammer.uni-hannover.de>
 *                2003-2005  Jan Kiszka <jan.kiszka@web.de>
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

#ifndef __RTNET_SOCKET_H_
#define __RTNET_SOCKET_H_

#include <asm/atomic.h>
#include <linux/list.h>

#include <rtdev.h>
#include <rtdm/net.h>
#include <rtdm/driver.h>
#include <stack_mgr.h>

struct rtsocket {
	unsigned short protocol;

	struct rtskb_pool skb_pool;
	unsigned int pool_size;
	struct mutex pool_nrt_lock;

	struct rtskb_queue incoming;

	rtdm_lock_t param_lock;

	unsigned int priority;
	nanosecs_rel_t timeout; /* receive timeout, 0 for infinite */

	rtdm_sem_t pending_sem;

	void (*callback_func)(struct rtdm_fd *, void *arg);
	void *callback_arg;

	unsigned long flags;

	union {
		/* IP specific */
		struct {
			u32 saddr; /* source ip-addr (bind) */
			u32 daddr; /* destination ip-addr */
			u16 sport; /* source port */
			u16 dport; /* destination port */

			int reg_index; /* index in port registry */
			u8 tos;
			u8 state;
		} inet;

		/* packet socket specific */
		struct {
			struct rtpacket_type packet_type;
			int ifindex;
		} packet;
	} prot;
};

static inline struct rtdm_fd *rt_socket_fd(struct rtsocket *sock)
{
	return rtdm_private_to_fd(sock);
}

void *rtnet_get_arg(struct rtdm_fd *fd, void *tmp, const void *src, size_t len);

int rtnet_put_arg(struct rtdm_fd *fd, void *dst, const void *src, size_t len);

#define rt_socket_reference(sock) rtdm_fd_lock(rt_socket_fd(sock))
#define rt_socket_dereference(sock) rtdm_fd_unlock(rt_socket_fd(sock))

int rt_socket_init(struct rtdm_fd *fd, unsigned short protocol);

void rt_socket_cleanup(struct rtdm_fd *fd);
int rt_socket_common_ioctl(struct rtdm_fd *fd, int request, void __user *arg);
int rt_socket_if_ioctl(struct rtdm_fd *fd, int request, void __user *arg);
int rt_socket_select_bind(struct rtdm_fd *fd, rtdm_selector_t *selector,
			  enum rtdm_selecttype type, unsigned fd_index);

int rt_bare_socket_init(struct rtdm_fd *fd, unsigned short protocol,
			unsigned int priority, unsigned int pool_size);

static inline void rt_bare_socket_cleanup(struct rtsocket *sock)
{
	rtskb_pool_release(&sock->skb_pool);
}

#endif /* __RTNET_SOCKET_H_ */
