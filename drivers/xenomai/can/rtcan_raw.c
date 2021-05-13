/*
 * Copyright (C) 2005, 2006 Sebastian Smolorz
 *                          <Sebastian.Smolorz@stud.uni-hannover.de>
 *
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 *
 * Parts of this software are based on the following:
 *
 * - RTAI CAN device driver for SJA1000 controllers by Jan Kiszka
 *
 * - linux-can.patch, a CAN socket framework for Linux,
 *   Copyright (C) 2004, 2005, Robert Schwebel, Benedikt Spranger,
 *   Marc Kleine-Budde, Sascha Hauer, Pengutronix
 *
 * - RTnet (www.rtnet.org)
 *
 * - serial device driver and profile included in Xenomai (RTDM),
 *   Copyright (C) 2005 Jan Kiszka <jan.kiszka@web.de>.
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/stringify.h>

#include <rtdm/driver.h>

#include <rtdm/can.h>
#include "rtcan_version.h"
#include "rtcan_socket.h"
#include "rtcan_list.h"
#include "rtcan_dev.h"
#include "rtcan_raw.h"
#include "rtcan_internal.h"


/*
 * Set if socket wants to receive a high precision timestamp together with
 * CAN frames
 */
#define RTCAN_GET_TIMESTAMP         0


MODULE_AUTHOR("RT-Socket-CAN Development Team");
MODULE_DESCRIPTION("RTDM CAN raw socket device driver");
MODULE_VERSION(__stringify(RTCAN_MAJOR_VER)
	       __stringify(RTCAN_MINOR_VER)
	       __stringify(RTCAN_BUGFIX_VER));
MODULE_LICENSE("GPL");

void rtcan_tx_push(struct rtcan_device *dev, struct rtcan_socket *sock,
		   can_frame_t *frame);

static inline int rtcan_accept_msg(uint32_t can_id, can_filter_t *filter)
{
    if ((filter->can_mask & CAN_INV_FILTER))
	return ((can_id & filter->can_mask) != filter->can_id);
    else
	return ((can_id & filter->can_mask) == filter->can_id);
}


static void rtcan_rcv_deliver(struct rtcan_recv *recv_listener,
			      struct rtcan_skb *skb)
{
    int size_free;
    size_t cpy_size, first_part_size;
    struct rtcan_rb_frame *frame = &skb->rb_frame;
    struct rtdm_fd *fd = rtdm_private_to_fd(recv_listener->sock);
    struct rtcan_socket *sock;

    if (rtdm_fd_lock(fd) < 0)
	return;

    sock = recv_listener->sock;

    cpy_size = skb->rb_frame_size;
    /* Check if socket wants to receive a timestamp */
    if (test_bit(RTCAN_GET_TIMESTAMP, &sock->flags)) {
	cpy_size += RTCAN_TIMESTAMP_SIZE;
	frame->can_dlc |= RTCAN_HAS_TIMESTAMP;
    } else
	frame->can_dlc &= RTCAN_HAS_NO_TIMESTAMP;

    /* Calculate free size in the ring buffer */
    size_free = sock->recv_head - sock->recv_tail;
    if (size_free <= 0)
	size_free += RTCAN_RXBUF_SIZE;

    /* Test if ring buffer has enough space. */
    if (size_free > cpy_size) {
	/* Check if we must wrap around the end of buffer */
	if ((sock->recv_tail + cpy_size) > RTCAN_RXBUF_SIZE) {
	    /* Wrap around: Two memcpy operations */

	    first_part_size = RTCAN_RXBUF_SIZE - sock->recv_tail;

	    memcpy(&sock->recv_buf[sock->recv_tail], (void *)frame,
		   first_part_size);
	    memcpy(&sock->recv_buf[0], (void *)frame +
		   first_part_size, cpy_size - first_part_size);
	} else
	    memcpy(&sock->recv_buf[sock->recv_tail], (void *)frame,
		   cpy_size);

	/* Adjust tail */
	sock->recv_tail = (sock->recv_tail + cpy_size) &
	    (RTCAN_RXBUF_SIZE - 1);

	/*Notify the delivery of the message */
	rtdm_sem_up(&sock->recv_sem);

    } else {
	/* Overflow of socket's ring buffer! */
	sock->rx_buf_full++;
	RTCAN_RTDM_DBG("rtcan: socket buffer overflow, message discarded\n");
    }

    rtdm_fd_unlock(fd);
}


void rtcan_rcv(struct rtcan_device *dev, struct rtcan_skb *skb)
{
    nanosecs_abs_t timestamp = rtdm_clock_read();
    /* Entry in reception list, begin with head */
    struct rtcan_recv *recv_listener = dev->recv_list;
    struct rtcan_rb_frame *frame = &skb->rb_frame;

    /* Copy timestamp to skb */
    memcpy((void *)&skb->rb_frame + skb->rb_frame_size,
	   &timestamp, RTCAN_TIMESTAMP_SIZE);

    if ((frame->can_id & CAN_ERR_FLAG)) {
	dev->err_count++;
	while (recv_listener != NULL) {
	    if ((frame->can_id & recv_listener->sock->err_mask)) {
		recv_listener->match_count++;
		rtcan_rcv_deliver(recv_listener, skb);
	    }
	    recv_listener = recv_listener->next;
	}
    } else {
	dev->rx_count++;
	while (recv_listener != NULL) {
	    if (rtcan_accept_msg(frame->can_id, &recv_listener->can_filter)) {
		recv_listener->match_count++;
		rtcan_rcv_deliver(recv_listener, skb);
	    }
	    recv_listener = recv_listener->next;
	}
    }
}

#ifdef CONFIG_XENO_DRIVERS_CAN_LOOPBACK

void rtcan_tx_push(struct rtcan_device *dev, struct rtcan_socket *sock,
		   can_frame_t *frame)
{
    struct rtcan_rb_frame *rb_frame = &dev->tx_skb.rb_frame;

    RTCAN_ASSERT(dev->tx_socket == 0,
		 rtdm_printk("(%d) TX skb still in use", dev->ifindex););

    rb_frame->can_id = frame->can_id;
    rb_frame->can_dlc = frame->can_dlc;
    dev->tx_skb.rb_frame_size = EMPTY_RB_FRAME_SIZE;
    if (frame->can_dlc && !(frame->can_id & CAN_RTR_FLAG)) {
	memcpy(rb_frame->data, frame->data, frame->can_dlc);
	dev->tx_skb.rb_frame_size += frame->can_dlc;
    }
    rb_frame->can_ifindex = dev->ifindex;
    dev->tx_socket = sock;
}

void rtcan_loopback(struct rtcan_device *dev)
{
    nanosecs_abs_t timestamp = rtdm_clock_read();
    /* Entry in reception list, begin with head */
    struct rtcan_recv *recv_listener = dev->recv_list;
    struct rtcan_rb_frame *frame = &dev->tx_skb.rb_frame;

    memcpy((void *)&dev->tx_skb.rb_frame + dev->tx_skb.rb_frame_size,
	   &timestamp, RTCAN_TIMESTAMP_SIZE);

    while (recv_listener != NULL) {
	dev->rx_count++;
	if ((dev->tx_socket != recv_listener->sock) &&
	    rtcan_accept_msg(frame->can_id, &recv_listener->can_filter)) {
	    recv_listener->match_count++;
	    rtcan_rcv_deliver(recv_listener, &dev->tx_skb);
	}
	recv_listener = recv_listener->next;
    }
    dev->tx_socket = NULL;
}

EXPORT_SYMBOL_GPL(rtcan_loopback);

#endif /* CONFIG_XENO_DRIVERS_CAN_LOOPBACK */


int rtcan_raw_socket(struct rtdm_fd *fd, int protocol)
{
    /* Only protocol CAN_RAW is supported */
    if (protocol != CAN_RAW && protocol != 0)
	return -EPROTONOSUPPORT;

    rtcan_socket_init(fd);

    return 0;
}


static inline void rtcan_raw_unbind(struct rtcan_socket *sock)
{
    rtcan_raw_remove_filter(sock);
    if (!rtcan_flist_no_filter(sock->flist) && sock->flist)
	rtdm_free(sock->flist);
    sock->flist = NULL;
    sock->flistlen = RTCAN_SOCK_UNBOUND;
    atomic_set(&sock->ifindex, 0);
}


static void rtcan_raw_close(struct rtdm_fd *fd)
{
    struct rtcan_socket *sock = rtdm_fd_to_private(fd);
    rtdm_lockctx_t lock_ctx;

    /* Get lock for reception lists */
    rtdm_lock_get_irqsave(&rtcan_recv_list_lock, lock_ctx);

    /* Check if socket is bound */
    if (rtcan_sock_is_bound(sock))
	rtcan_raw_unbind(sock);

    rtdm_lock_put_irqrestore(&rtcan_recv_list_lock, lock_ctx);


    rtcan_socket_cleanup(fd);
}


int rtcan_raw_bind(struct rtdm_fd *fd,
		   struct sockaddr_can *scan)
{
    struct rtcan_socket *sock = rtdm_fd_to_private(fd);
    rtdm_lockctx_t lock_ctx;
    int ret = 0;

    /* Check address family and
       check if given length of filter list is plausible */
    if (scan->can_family != AF_CAN)
	return -EINVAL;
    /* Check range of ifindex, must be between 0 and RTCAN_MAX_DEVICES */
    if (scan->can_ifindex < 0 || scan->can_ifindex > RTCAN_MAX_DEVICES)
	return -ENODEV;

    /* Get lock for reception lists */
    rtdm_lock_get_irqsave(&rtcan_recv_list_lock, lock_ctx);

    if ((ret = rtcan_raw_check_filter(sock, scan->can_ifindex,
				      sock->flist)))
	goto out;
    rtcan_raw_remove_filter(sock);
    /* Add filter and mark socket as bound */
    sock->flistlen = rtcan_raw_add_filter(sock, scan->can_ifindex);

    /* Set new interface index the socket is now bound to */
    atomic_set(&sock->ifindex, scan->can_ifindex);

 out:
    rtdm_lock_put_irqrestore(&rtcan_recv_list_lock, lock_ctx);

    return ret;
}


static int rtcan_raw_setsockopt(struct rtdm_fd *fd,
				struct _rtdm_setsockopt_args *so)
{
    struct rtcan_socket *sock = rtdm_fd_to_private(fd);
    struct rtcan_filter_list *flist;
    int ifindex = atomic_read(&sock->ifindex);
    rtdm_lockctx_t lock_ctx;
    can_err_mask_t err_mask;
    int val, ret = 0;

    if (so->level != SOL_CAN_RAW)
	return -ENOPROTOOPT;

    switch (so->optname) {

    case CAN_RAW_FILTER:
	if (so->optlen == 0) {
	    flist = RTCAN_FLIST_NO_FILTER;
	} else {
	    int flistlen;
	    flistlen = so->optlen / sizeof(struct can_filter);
	    if (flistlen < 1 || flistlen > RTCAN_MAX_RECEIVERS ||
		so->optlen % sizeof(struct can_filter) != 0)
		return -EINVAL;

	    flist = (struct rtcan_filter_list *)rtdm_malloc(so->optlen + sizeof(int));
	    if (flist == NULL)
		return -ENOMEM;
	    if (rtdm_fd_is_user(fd)) {
		if (!rtdm_read_user_ok(fd, so->optval, so->optlen) ||
		    rtdm_copy_from_user(fd, flist->flist,
					so->optval, so->optlen)) {
		    rtdm_free(flist);
		    return -EFAULT;
		}
	    } else
		memcpy(flist->flist, so->optval, so->optlen);
	    flist->flistlen = flistlen;
	}

	/* Get lock for reception lists */
	rtdm_lock_get_irqsave(&rtcan_recv_list_lock, lock_ctx);

	/* Check if there is space for the filter list if already bound */
	if (rtcan_sock_is_bound(sock)) {
	    if (!rtcan_flist_no_filter(flist) &&
		(ret = rtcan_raw_check_filter(sock, ifindex, flist))) {
		rtdm_free(flist);
		goto out_filter;
	    }
	    rtcan_raw_remove_filter(sock);
	}

	/* Remove previous list and attach the new one */
	if (!rtcan_flist_no_filter(flist) && sock->flist)
	    rtdm_free(sock->flist);
	sock->flist = flist;

	if (rtcan_sock_is_bound(sock))
	    sock->flistlen = rtcan_raw_add_filter(sock, ifindex);

    out_filter:
	/* Release lock for reception lists */
	rtdm_lock_put_irqrestore(&rtcan_recv_list_lock, lock_ctx);
	break;

    case CAN_RAW_ERR_FILTER:

	if (so->optlen != sizeof(can_err_mask_t))
	    return -EINVAL;

	if (rtdm_fd_is_user(fd)) {
	    if (!rtdm_read_user_ok(fd, so->optval, so->optlen) ||
		rtdm_copy_from_user(fd, &err_mask, so->optval, so->optlen))
		return -EFAULT;
	} else
	    memcpy(&err_mask, so->optval, so->optlen);

	/* Get lock for reception lists */
	rtdm_lock_get_irqsave(&rtcan_recv_list_lock, lock_ctx);
	sock->err_mask = err_mask;
	rtdm_lock_put_irqrestore(&rtcan_recv_list_lock, lock_ctx);

	break;

    case CAN_RAW_LOOPBACK:

	if (so->optlen != sizeof(int))
	    return -EINVAL;

	if (rtdm_fd_is_user(fd)) {
	    if (!rtdm_read_user_ok(fd, so->optval, so->optlen) ||
		rtdm_copy_from_user(fd, &val, so->optval, so->optlen))
		return -EFAULT;
	} else
	    memcpy(&val, so->optval, so->optlen);

#ifdef CONFIG_XENO_DRIVERS_CAN_LOOPBACK
	sock->loopback = val;
#else
	if (val)
	    return -EOPNOTSUPP;
#endif
	break;

    default:
	ret = -ENOPROTOOPT;
    }

    return ret;
}


int rtcan_raw_ioctl(struct rtdm_fd *fd,
		    unsigned int request, void *arg)
{
    int ret = 0;

    switch (request) {
    case _RTIOC_BIND: {
	struct _rtdm_setsockaddr_args *setaddr, setaddr_buf;
	struct sockaddr_can *sockaddr, sockaddr_buf;

	if (rtdm_fd_is_user(fd)) {
	    /* Copy argument structure from userspace */
	    if (!rtdm_read_user_ok(fd, arg,
				   sizeof(struct _rtdm_setsockaddr_args)) ||
		rtdm_copy_from_user(fd, &setaddr_buf, arg,
				    sizeof(struct _rtdm_setsockaddr_args)))
		return -EFAULT;

	    setaddr = &setaddr_buf;

	    /* Check size */
	    if (setaddr->addrlen != sizeof(struct sockaddr_can))
		return -EINVAL;

	    /* Copy argument structure from userspace */
	    if (!rtdm_read_user_ok(fd, arg,
				   sizeof(struct sockaddr_can)) ||
		rtdm_copy_from_user(fd, &sockaddr_buf, setaddr->addr,
				    sizeof(struct sockaddr_can)))
		return -EFAULT;
	    sockaddr = &sockaddr_buf;
	} else {
	    setaddr = (struct _rtdm_setsockaddr_args *)arg;
	    sockaddr = (struct sockaddr_can *)setaddr->addr;
	}

	/* Now, all required data are in kernel space */
	ret = rtcan_raw_bind(fd, sockaddr);

	break;
    }

    case _RTIOC_SETSOCKOPT: {
	struct _rtdm_setsockopt_args *setopt;
	struct _rtdm_setsockopt_args setopt_buf;

	if (rtdm_fd_is_user(fd)) {
	    if (!rtdm_read_user_ok(fd, arg,
				   sizeof(struct _rtdm_setsockopt_args)) ||
		rtdm_copy_from_user(fd, &setopt_buf, arg,
				    sizeof(struct _rtdm_setsockopt_args)))
		return -EFAULT;

	    setopt = &setopt_buf;
	} else
	    setopt = (struct _rtdm_setsockopt_args *)arg;

	return rtcan_raw_setsockopt(fd, setopt);
    }

    case RTCAN_RTIOC_TAKE_TIMESTAMP: {
	struct rtcan_socket *sock = rtdm_fd_to_private(fd);
	long timestamp_switch = (long)arg;

	if (timestamp_switch == RTCAN_TAKE_TIMESTAMPS)
	    set_bit(RTCAN_GET_TIMESTAMP, &sock->flags);
	else
	    clear_bit(RTCAN_GET_TIMESTAMP, &sock->flags);
	break;
    }

    case RTCAN_RTIOC_RCV_TIMEOUT:
    case RTCAN_RTIOC_SND_TIMEOUT: {
	/* Do some work these requests have in common. */
	struct rtcan_socket *sock = rtdm_fd_to_private(fd);

	nanosecs_rel_t *timeout = (nanosecs_rel_t *)arg;
	nanosecs_rel_t timeo_buf;

	if (rtdm_fd_is_user(fd)) {
	    /* Copy 64 bit timeout value from userspace */
	    if (!rtdm_read_user_ok(fd, arg,
				   sizeof(nanosecs_rel_t)) ||
		rtdm_copy_from_user(fd, &timeo_buf,
				    arg, sizeof(nanosecs_rel_t)))
		return -EFAULT;

	    timeout = &timeo_buf;
	}

	/* Now the differences begin between the requests. */
	if (request == RTCAN_RTIOC_RCV_TIMEOUT)
	    sock->rx_timeout = *timeout;
	else
	    sock->tx_timeout = *timeout;

	break;
    }

    default:
	ret = rtcan_raw_ioctl_dev(fd, request, arg);
	break;
    }

    return ret;
}


#define MEMCPY_FROM_RING_BUF(to, len)					\
do {									\
	if (unlikely((recv_buf_index + len) > RTCAN_RXBUF_SIZE)) { 	\
		/* Wrap around end of buffer */				\
		first_part_size = RTCAN_RXBUF_SIZE - recv_buf_index; 	\
		memcpy(to, &recv_buf[recv_buf_index], first_part_size);	\
		memcpy((void *)to + first_part_size, recv_buf,		\
		       len - first_part_size);				\
	} else								\
		memcpy(to, &recv_buf[recv_buf_index], len);		\
	recv_buf_index = (recv_buf_index + len) & (RTCAN_RXBUF_SIZE - 1); \
} while (0)

ssize_t rtcan_raw_recvmsg(struct rtdm_fd *fd,
			  struct user_msghdr *msg, int flags)
{
    struct rtcan_socket *sock = rtdm_fd_to_private(fd);
    struct sockaddr_can scan;
    nanosecs_rel_t timeout;
    struct iovec *iov = (struct iovec *)msg->msg_iov;
    struct iovec iov_buf;
    can_frame_t frame;
    nanosecs_abs_t timestamp = 0;
    unsigned char ifindex;
    unsigned char can_dlc;
    unsigned char *recv_buf;
    int recv_buf_index;
    size_t first_part_size;
    size_t payload_size;
    rtdm_lockctx_t lock_ctx;
    int ret;

    /* Clear frame memory location */
    memset(&frame, 0, sizeof(can_frame_t));

    /* Check flags */
    if (flags & ~(MSG_DONTWAIT | MSG_PEEK))
	return -EINVAL;


    /* Check if msghdr entries are sane */

    if (msg->msg_name != NULL) {
	if (msg->msg_namelen < sizeof(struct sockaddr_can))
	    return -EINVAL;

	if (rtdm_fd_is_user(fd)) {
	    if (!rtdm_rw_user_ok(fd, msg->msg_name, msg->msg_namelen))
		return -EFAULT;
	}

    } else {
	if (msg->msg_namelen != 0)
	    return -EINVAL;
    }

    /* Check msg_iovlen, only one buffer allowed */
    if (msg->msg_iovlen != 1)
	return -EMSGSIZE;

    if (rtdm_fd_is_user(fd)) {
	/* Copy IO vector from userspace */
	if (!rtdm_rw_user_ok(fd, msg->msg_iov,
			     sizeof(struct iovec)) ||
	    rtdm_copy_from_user(fd, &iov_buf, msg->msg_iov,
				sizeof(struct iovec)))
	    return -EFAULT;

	iov = &iov_buf;
    }

    /* Check size of buffer */
    if (iov->iov_len < sizeof(can_frame_t))
	return -EMSGSIZE;

    /* Check buffer if in user space */
    if (rtdm_fd_is_user(fd)) {
	if (!rtdm_rw_user_ok(fd, iov->iov_base, iov->iov_len))
	    return -EFAULT;
    }

    if (msg->msg_control != NULL) {
	if (msg->msg_controllen < sizeof(nanosecs_abs_t))
	    return -EINVAL;

	if (rtdm_fd_is_user(fd)) {
	    if (!rtdm_rw_user_ok(fd, msg->msg_control,
				 msg->msg_controllen))
		return -EFAULT;
	}

    } else {
	if (msg->msg_controllen != 0)
	    return -EINVAL;
    }

    rtcan_raw_enable_bus_err(sock);

    /* Set RX timeout */
    timeout = (flags & MSG_DONTWAIT) ? RTDM_TIMEOUT_NONE : sock->rx_timeout;

    /* Fetch message (ok, try it ...) */
    ret = rtdm_sem_timeddown(&sock->recv_sem, timeout, NULL);

    /* Error code returned? */
    if (unlikely(ret)) {
	/* Which error code? */

	if (ret == -EIDRM)
	    /* Socket was closed */
	    return -EBADF;

	else if (ret == -EWOULDBLOCK)
	    /* We would block but don't want to */
	    return -EAGAIN;

	else
	    /* Return all other error codes unmodified. */
	    return ret;
    }


    /* OK, we've got mail. */

    rtdm_lock_get_irqsave(&rtcan_socket_lock, lock_ctx);


    /* Construct a struct can_frame with data from socket's ring buffer */
    recv_buf_index = sock->recv_head;
    recv_buf = sock->recv_buf;


    /* Begin with CAN ID */
    MEMCPY_FROM_RING_BUF(&frame.can_id, sizeof(uint32_t));


    /* Fetch interface index */
    ifindex = recv_buf[recv_buf_index];
    recv_buf_index = (recv_buf_index + 1) & (RTCAN_RXBUF_SIZE - 1);


    /* Fetch DLC (with indicator if a timestamp exists) */
    can_dlc = recv_buf[recv_buf_index];
    recv_buf_index = (recv_buf_index + 1) & (RTCAN_RXBUF_SIZE - 1);

    frame.can_dlc = can_dlc & RTCAN_HAS_NO_TIMESTAMP;
    payload_size = (frame.can_dlc > 8) ? 8 : frame.can_dlc;


    /* If frame is an RTR or one with no payload it's not necessary
     * to copy the data bytes. */
    if (!(frame.can_id & CAN_RTR_FLAG) && payload_size)
	/* Copy data bytes */
	MEMCPY_FROM_RING_BUF(frame.data, payload_size);

    /* Is a timestamp available and is the caller actually interested? */
    if (msg->msg_controllen && (can_dlc & RTCAN_HAS_TIMESTAMP))
	/* Copy timestamp */
	MEMCPY_FROM_RING_BUF(&timestamp, RTCAN_TIMESTAMP_SIZE);

    /* Message completely read from the socket's ring buffer. Now check if
     * caller is just peeking. */
    if (flags & MSG_PEEK)
	/* Next one, please! */
	rtdm_sem_up(&sock->recv_sem);
    else
	/* Adjust begin of first message in the ring buffer. */
	sock->recv_head = recv_buf_index;


    /* Release lock */
    rtdm_lock_put_irqrestore(&rtcan_socket_lock, lock_ctx);


    /* Create CAN socket address to give back */
    if (msg->msg_namelen) {
	scan.can_family = AF_CAN;
	scan.can_ifindex = ifindex;
    }


    /* Last duty: Copy all back to the caller's buffers. */

    if (rtdm_fd_is_user(fd)) {
	/* Copy to user space */

	/* Copy socket address */
	if (msg->msg_namelen) {
	    if (rtdm_copy_to_user(fd, msg->msg_name, &scan,
				  sizeof(struct sockaddr_can)))
		return -EFAULT;

	    msg->msg_namelen = sizeof(struct sockaddr_can);
	}

	/* Copy CAN frame */
	if (rtdm_copy_to_user(fd, iov->iov_base, &frame,
			      sizeof(can_frame_t)))
	    return -EFAULT;
	/* Adjust iovec in the common way */
	iov->iov_base += sizeof(can_frame_t);
	iov->iov_len -= sizeof(can_frame_t);
	/* ... and copy it, too. */
	if (rtdm_copy_to_user(fd, msg->msg_iov, iov,
			      sizeof(struct iovec)))
	    return -EFAULT;

	/* Copy timestamp if existent and wanted */
	if (msg->msg_controllen) {
	    if (can_dlc & RTCAN_HAS_TIMESTAMP) {
		if (rtdm_copy_to_user(fd, msg->msg_control,
				      &timestamp, RTCAN_TIMESTAMP_SIZE))
		    return -EFAULT;

		msg->msg_controllen = RTCAN_TIMESTAMP_SIZE;
	    } else
		msg->msg_controllen = 0;
	}

    } else {
	/* Kernel space */

	/* Copy socket address */
	if (msg->msg_namelen) {
	    memcpy(msg->msg_name, &scan, sizeof(struct sockaddr_can));
	    msg->msg_namelen = sizeof(struct sockaddr_can);
	}

	/* Copy CAN frame */
	memcpy(iov->iov_base, &frame, sizeof(can_frame_t));
	/* Adjust iovec in the common way */
	iov->iov_base += sizeof(can_frame_t);
	iov->iov_len -= sizeof(can_frame_t);

	/* Copy timestamp if existent and wanted */
	if (msg->msg_controllen) {
	    if (can_dlc & RTCAN_HAS_TIMESTAMP) {
		memcpy(msg->msg_control, &timestamp, RTCAN_TIMESTAMP_SIZE);
		msg->msg_controllen = RTCAN_TIMESTAMP_SIZE;
	    } else
		msg->msg_controllen = 0;
	}
    }


    return sizeof(can_frame_t);
}


ssize_t rtcan_raw_sendmsg(struct rtdm_fd *fd,
			  const struct user_msghdr *msg, int flags)
{
    struct rtcan_socket *sock = rtdm_fd_to_private(fd);
    struct sockaddr_can *scan = (struct sockaddr_can *)msg->msg_name;
    struct sockaddr_can scan_buf;
    struct iovec *iov = (struct iovec *)msg->msg_iov;
    struct iovec iov_buf;
    can_frame_t *frame;
    can_frame_t frame_buf;
    rtdm_lockctx_t lock_ctx;
    nanosecs_rel_t timeout = 0;
    struct tx_wait_queue tx_wait;
    struct rtcan_device *dev;
    int ifindex = 0;
    int ret  = 0;
    spl_t s;


    if (flags & MSG_OOB)   /* Mirror BSD error message compatibility */
	return -EOPNOTSUPP;

    /* Only MSG_DONTWAIT is a valid flag. */
    if (flags & ~MSG_DONTWAIT)
	return -EINVAL;

    /* Check msg_iovlen, only one buffer allowed */
    if (msg->msg_iovlen != 1)
	return -EMSGSIZE;

    if (scan == NULL) {
	/* No socket address. Will use bound interface for sending */

	if (msg->msg_namelen != 0)
	    return -EINVAL;


	/* We only want a consistent value here, a spin lock would be
	 * overkill. Nevertheless, the binding could change till we have
	 * the chance to send. Blame the user, though. */
	ifindex = atomic_read(&sock->ifindex);

	if (!ifindex)
	    /* Socket isn't bound or bound to all interfaces. Go out. */
	    return -ENXIO;
    } else {
	/* Socket address given */
	if (msg->msg_namelen < sizeof(struct sockaddr_can))
	    return -EINVAL;

	if (rtdm_fd_is_user(fd)) {
	    /* Copy socket address from userspace */
	    if (!rtdm_read_user_ok(fd, msg->msg_name,
				   sizeof(struct sockaddr_can)) ||
		rtdm_copy_from_user(fd, &scan_buf, msg->msg_name,
				    sizeof(struct sockaddr_can)))
		return -EFAULT;

	    scan = &scan_buf;
	}

	/* Check address family */
	if (scan->can_family != AF_CAN)
	    return -EINVAL;

	ifindex = scan->can_ifindex;
    }

    if (rtdm_fd_is_user(fd)) {
	/* Copy IO vector from userspace */
	if (!rtdm_rw_user_ok(fd, msg->msg_iov,
			     sizeof(struct iovec)) ||
	    rtdm_copy_from_user(fd, &iov_buf, msg->msg_iov,
				sizeof(struct iovec)))
	    return -EFAULT;

	iov = &iov_buf;
    }

    /* Check size of buffer */
    if (iov->iov_len != sizeof(can_frame_t))
	return -EMSGSIZE;

    frame = (can_frame_t *)iov->iov_base;

    if (rtdm_fd_is_user(fd)) {
	/* Copy CAN frame from userspace */
	if (!rtdm_read_user_ok(fd, iov->iov_base,
			       sizeof(can_frame_t)) ||
	    rtdm_copy_from_user(fd, &frame_buf, iov->iov_base,
				sizeof(can_frame_t)))
	    return -EFAULT;

	frame = &frame_buf;
    }

    /* Adjust iovec in the common way */
    iov->iov_base += sizeof(can_frame_t);
    iov->iov_len -= sizeof(can_frame_t);
    /* ... and copy it back to userspace if necessary */
    if (rtdm_fd_is_user(fd)) {
	if (rtdm_copy_to_user(fd, msg->msg_iov, iov,
			      sizeof(struct iovec)))
	    return -EFAULT;
    }

    /* At last, we've got the frame ... */

    /* Check if DLC between 0 and 15 */
    if (frame->can_dlc > 15)
	return -EINVAL;

    /* Check if it is a standard frame and the ID between 0 and 2031 */
    if (!(frame->can_id & CAN_EFF_FLAG)) {
	u32 id = frame->can_id & CAN_EFF_MASK;
	if (id > (CAN_SFF_MASK - 16))
	    return -EINVAL;
    }

    if ((dev = rtcan_dev_get_by_index(ifindex)) == NULL)
	return -ENXIO;

    timeout = (flags & MSG_DONTWAIT) ? RTDM_TIMEOUT_NONE : sock->tx_timeout;

    tx_wait.rt_task = rtdm_task_current();

    /* Register the task at the socket's TX wait queue and decrement
     * the TX semaphore. This must be atomic. Finally, the task must
     * be deregistered again (also atomic). */
    cobalt_atomic_enter(s);

    list_add(&tx_wait.tx_wait_list, &sock->tx_wait_head);

    /* Try to pass the guard in order to access the controller */
    ret = rtdm_sem_timeddown(&dev->tx_sem, timeout, NULL);

    /* Only dequeue task again if socket isn't being closed i.e. if
     * this task was not unblocked within the close() function. */
    if (likely(!list_empty(&tx_wait.tx_wait_list)))
	/* Dequeue this task from the TX wait queue */
	list_del_init(&tx_wait.tx_wait_list);
    else
	/* The socket was closed. */
	ret = -EBADF;

    cobalt_atomic_leave(s);

    /* Error code returned? */
    if (ret != 0) {
	/* Which error code? */
	switch (ret) {
	case -EIDRM:
	    /* Controller is stopped or bus-off */
	    ret = -ENETDOWN;
	    goto send_out1;

	case -EWOULDBLOCK:
	    /* We would block but don't want to */
	    ret = -EAGAIN;
	    goto send_out1;

	default:
	    /* Return all other error codes unmodified. */
	    goto send_out1;
	}
    }

    /* We got access */


    /* Push message onto stack for loopback when TX done */
    if (rtcan_loopback_enabled(sock))
	rtcan_tx_push(dev, sock, frame);

    rtdm_lock_get_irqsave(&dev->device_lock, lock_ctx);

    /* Controller should be operating */
    if (!CAN_STATE_OPERATING(dev->state)) {
	if (dev->state == CAN_STATE_SLEEPING) {
	    ret = -ECOMM;
	    rtdm_lock_put_irqrestore(&dev->device_lock, lock_ctx);
	    rtdm_sem_up(&dev->tx_sem);
	    goto send_out1;
	}
	ret = -ENETDOWN;
	goto send_out2;
    }

    dev->tx_count++;
    ret = dev->hard_start_xmit(dev, frame);

    /* Return number of bytes sent upon successful completion */
    if (ret == 0)
	ret = sizeof(can_frame_t);

 send_out2:
    rtdm_lock_put_irqrestore(&dev->device_lock, lock_ctx);
 send_out1:
    rtcan_dev_dereference(dev);
    return ret;
}


static struct rtdm_driver rtcan_driver = {
	.profile_info		= RTDM_PROFILE_INFO(rtcan,
						    RTDM_CLASS_CAN,
						    RTDM_SUBCLASS_GENERIC,
						    RTCAN_PROFILE_VER),
	.device_flags		= RTDM_PROTOCOL_DEVICE,
	.device_count		= 1,
	.context_size		= sizeof(struct rtcan_socket),
	.protocol_family	= PF_CAN,
	.socket_type		= SOCK_RAW,
	.ops = {
		.socket		= rtcan_raw_socket,
		.close		= rtcan_raw_close,
		.ioctl_nrt	= rtcan_raw_ioctl,
		.recvmsg_rt	= rtcan_raw_recvmsg,
		.sendmsg_rt	= rtcan_raw_sendmsg,
	},
};

static struct rtdm_device rtcan_device = {
	.driver = &rtcan_driver,
	.label = "rtcan",
};

int __init rtcan_raw_proto_register(void)
{
    return rtdm_dev_register(&rtcan_device);
}

void __exit rtcan_raw_proto_unregister(void)
{
    rtdm_dev_unregister(&rtcan_device);
}


EXPORT_SYMBOL_GPL(rtcan_rcv);
