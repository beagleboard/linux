/*
 * Copyright (C) 2005,2006 Sebastian Smolorz
 *                         <Sebastian.Smolorz@stud.uni-hannover.de>
 *
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 *
 * Derived from RTnet project file include/stack/socket.h:
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

#ifndef __RTCAN_SOCKET_H_
#define __RTCAN_SOCKET_H_

#include <rtdm/driver.h>

#include <rtdm/can.h>



/* This MUST BE 2^N */
#define RTCAN_RXBUF_SIZE          CONFIG_XENO_DRIVERS_CAN_RXBUF_SIZE

/* Size of timestamp */
#define RTCAN_TIMESTAMP_SIZE      sizeof(nanosecs_abs_t)

/* Bit in the can_dlc member of struct ring_buffer_frame used to indicate
 * whether a frame has got a timestamp or not */
#define RTCAN_HAS_TIMESTAMP       0x80

/* Mask for clearing bit RTCAN_HAS_TIMESTAMP */
#define RTCAN_HAS_NO_TIMESTAMP    0x7F

#define RTCAN_SOCK_UNBOUND        -1
#define RTCAN_FLIST_NO_FILTER     (struct rtcan_filter_list *)-1
#define rtcan_flist_no_filter(f)  ((f) == RTCAN_FLIST_NO_FILTER)
#define rtcan_sock_has_filter(s)  ((s)->flistlen > 0)
#define rtcan_sock_is_bound(s)    ((s)->flistlen >= 0)

/*
 *  Internal frame representation within the ring buffer of a
 *  struct rtcan_socket.
 *
 *  The data array is of arbitrary size when the frame is actually
 *  stored in a socket's ring buffer. The timestamp member exists if the
 *  socket was set to take timestamps (then it follows direcly after the
 *  arbitrary-sized data array), otherwise it does not exist.
 */
struct rtcan_rb_frame {

    /* CAN ID representation equal to struct can_frame */
    uint32_t            can_id;

    /* Interface index from which the frame originates */
    unsigned char       can_ifindex;

    /* DLC (between 0 and 15) and mark if frame has got a timestamp. The
     * existence of a timestamp is indicated by the RTCAN_HAS_TIMESTAMP
     * bit. */
    unsigned char       can_dlc;

    /* Data bytes */
    uint8_t             data[8];

    /* High precision timestamp indicating when the frame was received.
     * Exists when RTCAN_HAS_TIMESTAMP bit in can_dlc is set. */
    nanosecs_abs_t      timestamp;

} __attribute__ ((packed));


/* Size of struct rtcan_rb_frame without any data bytes and timestamp */
#define EMPTY_RB_FRAME_SIZE \
    sizeof(struct rtcan_rb_frame) - 8 - RTCAN_TIMESTAMP_SIZE


/*
 *  Wrapper structure around a struct rtcan_rb_frame with actual size
 *  of the frame.
 *
 *  This isn't really a socket buffer but only a sort of. It is constructed
 *  within the interrupt routine when a CAN frame is read from
 *  the controller. Then it's passed to the reception handler where only
 *  rb_frame finds its way to the sockets' ring buffers.
 */
struct rtcan_skb {
    /* Actual size of following rb_frame (without timestamp) */
    size_t                rb_frame_size;
    /* Frame to be stored in the sockets' ring buffers (as is) */
    struct rtcan_rb_frame rb_frame;
};

struct rtcan_filter_list {
    int flistlen;
    struct can_filter flist[1];
};

/*
 * Internal CAN socket structure.
 *
 * Every socket has an internal ring buffer for incoming messages. A message
 * is not stored as a struct can_frame (in order to save buffer space)
 * but as struct rtcan_rb_frame of arbitrary length depending on the
 * actual payload.
 */
struct rtcan_socket {

    struct list_head    socket_list;

    unsigned long	flags;

    /* Transmission timeout in ns. Protected by rtcan_socket_lock
     * in all socket structures. */
    nanosecs_rel_t      tx_timeout;

    /* Reception timeout in ns. Protected by rtcan_socket_lock
     * in all socket structures. */
    nanosecs_rel_t      rx_timeout;


    /* Begin of first frame data in the ring buffer. Protected by
     * rtcan_socket_lock in all socket structures. */
    int                 recv_head;

    /* End of last frame data in the ring buffer. I.e. position of first
     * free byte in the ring buffer. Protected by
     * rtcan_socket_lock in all socket structures. */
    int                 recv_tail;

    /* Ring buffer for incoming CAN frames. Protected by
     * rtcan_socket_lock in all socket structures. */
    unsigned char       recv_buf[RTCAN_RXBUF_SIZE];

    /* Semaphore for receivers and incoming messages */
    rtdm_sem_t          recv_sem;


    /* All senders waiting to be able to send
     * via this socket are queued here */
    struct list_head    tx_wait_head;


    /* Interface index the socket is bound to. Protected by
     * rtcan_recv_list_lock in all socket structures. */
    atomic_t            ifindex;

    /* Length of filter list. I.e. how many entries does this socket occupy in
     * the reception list. 0 if unbound. Protected by
     * rtcan_recv_list_lock in all socket structures. */
    int                 flistlen;

    uint32_t            err_mask;

    uint32_t            rx_buf_full;

    struct rtcan_filter_list *flist;

#ifdef CONFIG_XENO_DRIVERS_CAN_LOOPBACK
    int loopback;
#endif
};



/*
 *  Get the RTDM context from a struct rtcan_socket
 *
 *  @param[in] sock Pointer to socket structure
 *
 *  @return Pointer to a file descriptor of type struct rtdm_fd this socket
 *          belongs to
 */
/* FIXME: to be replaced with container_of */
static inline struct rtdm_fd *rtcan_socket_to_fd(struct rtcan_socket *sock)
{
    return rtdm_private_to_fd(sock);
}

/* Spinlock protecting the ring buffers and the timeouts of all
 * rtcan_sockets */
extern rtdm_lock_t rtcan_socket_lock;
extern struct list_head rtcan_socket_list;

extern void rtcan_socket_init(struct rtdm_fd *fd);
extern void rtcan_socket_cleanup(struct rtdm_fd *fd);


#endif  /* __RTCAN_SOCKET_H_ */
