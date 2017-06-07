/*
 * Copyright (C) 2005, 2006 Sebastian Smolorz
 *                          <Sebastian.Smolorz@stud.uni-hannover.de>
 *
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; eitherer version 2 of the License, or
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

#include <linux/module.h>
#include <linux/delay.h>

#include <rtdm/driver.h>

#include <rtdm/can.h>
#include "rtcan_internal.h"
#include "rtcan_socket.h"
#include "rtcan_list.h"
#include "rtcan_dev.h"
#include "rtcan_raw.h"


#if 0
void rtcan_raw_print_filter(struct rtcan_device *dev)
{
    int i;
    struct rtcan_recv *r = dev->receivers;

    rtdm_printk("%s: recv_list=%p empty_list=%p free_entries=%d\n",
		dev->name, dev->recv_list, dev->empty_list, dev->free_entries);
    for (i = 0; i < RTCAN_MAX_RECEIVERS; i++, r++) {
	rtdm_printk("%2d %p sock=%p next=%p id=%x mask=%x\n",
		    i, r, r->sock, r->next,
		    r->can_filter.can_id, r->can_filter.can_mask);
    }
}
#else
#define rtcan_raw_print_filter(dev)
#endif


static inline void rtcan_raw_mount_filter(can_filter_t *recv_filter,
					  can_filter_t *filter)
{
    if (filter->can_id & CAN_INV_FILTER) {
	recv_filter->can_id = filter->can_id & ~CAN_INV_FILTER;
	recv_filter->can_mask = filter->can_mask | CAN_INV_FILTER;
    } else {
	recv_filter->can_id = filter->can_id;
	recv_filter->can_mask = filter->can_mask & ~CAN_INV_FILTER;
    }

    /* Apply mask for fast filter check */
    recv_filter->can_id &= recv_filter->can_mask;
}


int rtcan_raw_check_filter(struct rtcan_socket *sock, int ifindex,
			   struct rtcan_filter_list *flist)
{
    int old_ifindex = 0, old_flistlen_all = 0;
    int free_entries, i, begin, end;
    struct rtcan_device *dev;
    int flistlen;

    if (rtcan_flist_no_filter(flist))
	return 0;

    /* Check if filter list has been defined by user */
    flistlen = (flist) ? flist->flistlen : 1;

    /* Now we check if a reception list would overflow. This takes some
     * preparation, so let's go ... */

    /* Check current bind status */
    if (rtcan_sock_has_filter(sock)) {
	/* Socket is bound */
	i = atomic_read(&sock->ifindex);

	if (i == 0)
	    /* Socket was bound to ALL interfaces */
	    old_flistlen_all = sock->flistlen;
	else    /* Socket was bound to only one interface */
	    old_ifindex = i;
    }

    if (ifindex) {
	/* We bind the socket to only one interface. */
	begin = ifindex;
	end   = ifindex;
    } else {
	/* Socket must be bound to all interfaces. */
	begin = 1;
	end = RTCAN_MAX_DEVICES;
    }

    /* Check if there is space for the new binding */
    for (i = begin; i <= end; i++) {
	if ((dev = rtcan_dev_get_by_index(i)) == NULL)
	    continue;
	free_entries = dev->free_entries + old_flistlen_all;
	rtcan_dev_dereference(dev);
	if (i == old_ifindex)
	    free_entries += sock->flistlen;
	/* Compare free list space to new filter list length */
	if (free_entries < flistlen)
	    return -ENOSPC;
    }

    return 0;
}


int rtcan_raw_add_filter(struct rtcan_socket *sock, int ifindex)
{
    int i, j, begin, end;
    struct rtcan_recv *first, *last;
    struct rtcan_device *dev;
    /* Check if filter list has been defined by user */
    int flistlen;

    if (rtcan_flist_no_filter(sock->flist)) {
	return 0;
    }

    flistlen = (sock->flist) ? sock->flist->flistlen : 0;

    if (ifindex) {
	/* We bind the socket to only one interface. */
	begin = ifindex;
	end   = ifindex;
    } else {
	/* Socket must be bound to all interfaces. */
	begin = 1;
	end = RTCAN_MAX_DEVICES;
    }

    for (i = begin; i <= end; i++) {
	if ((dev = rtcan_dev_get_by_index(i)) == NULL)
	    continue;

	/* Take first entry of empty list */
	first = last = dev->empty_list;
	/* Check if filter list is empty */
	if (flistlen) {
	    /* Filter list is not empty */
	    /* Register first filter */
	    rtcan_raw_mount_filter(&last->can_filter,
				   &sock->flist->flist[0]);
	    last->match_count = 0;
	    last->sock = sock;
	    for (j = 1; j < flistlen; j++) {
		/* Register remaining filters */
		last = last->next;
		rtcan_raw_mount_filter(&last->can_filter,
				       &sock->flist->flist[j]);
		last->sock = sock;
		last->match_count = 0;
	    }
	    /* Decrease free entries counter by length of filter list */
	    dev->free_entries -= flistlen;

	} else {
	    /* Filter list is empty. Socket must be bound to all CAN IDs. */
	    /* Fill list entry members */
	    last->can_filter.can_id = last->can_filter.can_mask = 0;
	    last->sock = sock;
	    last->match_count = 0;
	    /* Decrease free entries counter by 1
	     * (one filter for all CAN frames) */
	    dev->free_entries--;
	}

	/* Set new empty list header */
	dev->empty_list = last->next;
	/* Add new partial recv list to the head of reception list */
	last->next = dev->recv_list;
	/* Adjust rececption list pointer */
	dev->recv_list = first;

	rtcan_raw_print_filter(dev);
	rtcan_dev_dereference(dev);
    }

    return (flistlen) ? flistlen : 1;
}


void rtcan_raw_remove_filter(struct rtcan_socket *sock)
{
    int i, j, begin, end;
    struct rtcan_recv *first, *next, *last;
    int ifindex = atomic_read(&sock->ifindex);
    struct rtcan_device *dev;

    if (!rtcan_sock_has_filter(sock)) /* nothing to do */
	return;

    if (ifindex) {
	/* Socket was bound to one interface only. */
	begin = ifindex;
	end   = ifindex;
    } else {
	/* Socket was bound to all interfaces */
	begin = 1;
	end = RTCAN_MAX_DEVICES;
    }

    for (i = begin; i <= end; i++) {

	if ((dev = rtcan_dev_get_by_index(i)) == NULL)
	    continue;

	/* Search for first list entry pointing to this socket */
	first = NULL;
	next = dev->recv_list;
	while (next->sock != sock) {
	    first = next;
	    next = first->next;
	}

	/* Now go to the end of the old filter list */
	last = next;
	for (j = 1; j < sock->flistlen; j++)
	    last = last->next;

	/* Detach found first list entry from reception list */
	if (first)
	    first->next = last->next;
	else
	    dev->recv_list = last->next;
	/* Add partial list to the head of empty list */
	last->next = dev->empty_list;
	/* Adjust empty list pointer */
	dev->empty_list = next;

	/* Increase free entries counter by length of old filter list */
	dev->free_entries += sock->flistlen;

	rtcan_raw_print_filter(dev);
	rtcan_dev_dereference(dev);
    }
}
