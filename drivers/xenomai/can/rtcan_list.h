/*
 * List management for the RTDM RTCAN device driver
 *
 * Copyright (C) 2005,2006 Sebastian Smolorz
 *                         <Sebastian.Smolorz@stud.uni-hannover.de>
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

#ifndef __RTCAN_LIST_H_
#define __RTCAN_LIST_H_

#include "rtcan_socket.h"


/*
 * List element in a single linked list used for registering reception sockets.
 * Every single struct can_filter which was bound to a socket gets such a
 * list entry. There is no member for the CAN interface because there is one
 * reception list for every CAN controller. This is because when a CAN message
 * is received it is clear from which interface and therefore minimizes
 * searching time.
 */
struct rtcan_recv {
    can_filter_t            can_filter;     /* filter used for deciding if
					     *   a socket wants to get a CAN
					     *   message */
    unsigned int            match_count;    /* count accepted messages */
    struct rtcan_socket     *sock;          /* pointer to registered socket
					     */
    struct rtcan_recv       *next;          /* pointer to next list element
					     */
};


/*
 *  Element in a TX wait queue.
 *
 *  Every socket holds a TX wait queue where all RT tasks are queued when they
 *  are blocked while waiting to be able to transmit a message via this socket.
 *
 *  Every sender holds its own element.
 */
struct tx_wait_queue {
    struct list_head        tx_wait_list;   /* List pointers */
    rtdm_task_t             *rt_task;       /* Pointer to task handle */
};


/* Spinlock for all reception lists and also for some members in
 * struct rtcan_socket */
extern rtdm_lock_t rtcan_recv_list_lock;


#endif  /* __RTCAN_LIST_H_ */
