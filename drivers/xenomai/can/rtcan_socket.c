/*
 * Copyright (C) 2005,2006 Sebastian Smolorz
 *                         <Sebastian.Smolorz@stud.uni-hannover.de>
 *
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 *
 * Based on stack/socket.c - sockets implementation for RTnet
 *
 * Copyright (C) 1999       Lineo, Inc
 *               1999, 2002 David A. Schleef <ds@schleef.org>
 *               2002       Ulrich Marx <marx@kammer.uni-hannover.de>
 *               2003-2005  Jan Kiszka <jan.kiszka@web.de>
 *
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

#include "rtcan_socket.h"
#include "rtcan_list.h"


LIST_HEAD(rtcan_socket_list);

void rtcan_socket_init(struct rtdm_fd *fd)
{
    struct rtcan_socket *sock = rtdm_fd_to_private(fd);
    rtdm_lockctx_t lock_ctx;


    rtdm_sem_init(&sock->recv_sem, 0);

    sock->recv_head = 0;
    sock->recv_tail = 0;
    atomic_set(&sock->ifindex, 0);
    sock->flistlen = RTCAN_SOCK_UNBOUND;
    sock->flist = NULL;
    sock->err_mask = 0;
    sock->rx_buf_full = 0;
    sock->flags = 0;
#ifdef CONFIG_XENO_DRIVERS_CAN_LOOPBACK
    sock->loopback = 1;
#endif

    sock->tx_timeout = RTDM_TIMEOUT_INFINITE;
    sock->rx_timeout = RTDM_TIMEOUT_INFINITE;

    INIT_LIST_HEAD(&sock->tx_wait_head);

    rtdm_lock_get_irqsave(&rtcan_recv_list_lock, lock_ctx);
    list_add(&sock->socket_list, &rtcan_socket_list);
    rtdm_lock_put_irqrestore(&rtcan_recv_list_lock, lock_ctx);
}


void rtcan_socket_cleanup(struct rtdm_fd *fd)
{
    struct rtcan_socket *sock = rtdm_fd_to_private(fd);
    struct tx_wait_queue *tx_waiting;
    rtdm_lockctx_t lock_ctx;
    int tx_list_empty;

    /* Wake up sleeping senders. This is re-entrant-safe. */
    do {
	cobalt_atomic_enter(lock_ctx);
	/* Is someone there? */
	if (list_empty(&sock->tx_wait_head))
		tx_list_empty = 1;
	else {
		tx_list_empty = 0;

		/* Get next entry pointing to a waiting task */
		tx_waiting = list_entry(sock->tx_wait_head.next,
					struct tx_wait_queue, tx_wait_list);

		/* Remove it from list */
		list_del_init(&tx_waiting->tx_wait_list);

		/* Wake task up (atomic section is left implicitly) */
		rtdm_task_unblock(tx_waiting->rt_task);
	}
	cobalt_atomic_leave(lock_ctx);
    } while (!tx_list_empty);

    rtdm_sem_destroy(&sock->recv_sem);

    rtdm_lock_get_irqsave(&rtcan_recv_list_lock, lock_ctx);
    if (sock->socket_list.next) {
	list_del(&sock->socket_list);
	sock->socket_list.next = NULL;
    }
    rtdm_lock_put_irqrestore(&rtcan_recv_list_lock, lock_ctx);
}
