/***
 *
 *  stack/socket.c - sockets implementation for rtnet
 *
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

#include <linux/moduleparam.h>
#include <linux/spinlock.h>
#include <linux/socket.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <asm/bitops.h>

#include <rtnet.h>
#include <rtnet_internal.h>
#include <rtnet_iovec.h>
#include <rtnet_socket.h>
#include <ipv4/protocol.h>


#define SKB_POOL_CLOSED     0

static unsigned int socket_rtskbs = DEFAULT_SOCKET_RTSKBS;
module_param(socket_rtskbs, uint, 0444);
MODULE_PARM_DESC(socket_rtskbs, "Default number of realtime socket buffers in socket pools");


/************************************************************************
 *  internal socket functions                                           *
 ************************************************************************/

int __rt_bare_socket_init(struct rtdm_fd *fd, unsigned short protocol,
			unsigned int priority, unsigned int pool_size,
			struct module *module)
{
    struct rtsocket *sock = rtdm_fd_to_private(fd);
    int err;

    err = try_module_get(module);
    if (!err)
	return -EAFNOSUPPORT;

    err = rtskb_pool_init(&sock->skb_pool, pool_size, NULL, fd);
    if (err < 0) {
	module_put(module);
	return err;
    }

    sock->protocol = protocol;
    sock->priority = priority;
    sock->owner = module;

    return err;
}
EXPORT_SYMBOL_GPL(__rt_bare_socket_init);

/***
 *  rt_socket_init - initialises a new socket structure
 */
int __rt_socket_init(struct rtdm_fd *fd, unsigned short protocol,
		struct module *module)
{
    struct rtsocket *sock = rtdm_fd_to_private(fd);
    unsigned int    pool_size;

    sock->flags = 0;
    sock->callback_func = NULL;

    rtskb_queue_init(&sock->incoming);

    sock->timeout = 0;

    rtdm_lock_init(&sock->param_lock);
    rtdm_sem_init(&sock->pending_sem, 0);

    pool_size = __rt_bare_socket_init(fd, protocol,
				    RTSKB_PRIO_VALUE(SOCK_DEF_PRIO,
						    RTSKB_DEF_RT_CHANNEL),
				    socket_rtskbs, module);
    sock->pool_size = pool_size;
    mutex_init(&sock->pool_nrt_lock);

    if (pool_size < socket_rtskbs) {
	/* fix statistics */
	if (pool_size == 0)
	    rtskb_pools--;

	rt_socket_cleanup(fd);
	return -ENOMEM;
    }

    return 0;
}
EXPORT_SYMBOL_GPL(__rt_socket_init);


/***
 *  rt_socket_cleanup - releases resources allocated for the socket
 */
void rt_socket_cleanup(struct rtdm_fd *fd)
{
    struct rtsocket *sock  = rtdm_fd_to_private(fd);


    rtdm_sem_destroy(&sock->pending_sem);

    mutex_lock(&sock->pool_nrt_lock);

    set_bit(SKB_POOL_CLOSED, &sock->flags);

    if (sock->pool_size > 0)
	rtskb_pool_release(&sock->skb_pool);

    mutex_unlock(&sock->pool_nrt_lock);

    module_put(sock->owner);
}
EXPORT_SYMBOL_GPL(rt_socket_cleanup);


/***
 *  rt_socket_common_ioctl
 */
int rt_socket_common_ioctl(struct rtdm_fd *fd, int request, void *arg)
{
    struct rtsocket         *sock = rtdm_fd_to_private(fd);
    int                     ret = 0;
    struct rtnet_callback   *callback = arg;
    unsigned int            rtskbs;
    rtdm_lockctx_t          context;


    switch (request) {
	case RTNET_RTIOC_XMITPARAMS:
	    sock->priority = *(unsigned int *)arg;
	    break;

	case RTNET_RTIOC_TIMEOUT:
	    sock->timeout = *(nanosecs_rel_t *)arg;
	    break;

	case RTNET_RTIOC_CALLBACK:
	    if (rtdm_fd_is_user(fd))
		return -EACCES;

	    rtdm_lock_get_irqsave(&sock->param_lock, context);

	    sock->callback_func = callback->func;
	    sock->callback_arg  = callback->arg;

	    rtdm_lock_put_irqrestore(&sock->param_lock, context);
	    break;

	case RTNET_RTIOC_EXTPOOL:
	    rtskbs = *(unsigned int *)arg;

	    if (rtdm_in_rt_context())
		return -ENOSYS;

	    mutex_lock(&sock->pool_nrt_lock);

	    if (test_bit(SKB_POOL_CLOSED, &sock->flags)) {
		mutex_unlock(&sock->pool_nrt_lock);
		return -EBADF;
	    }
	    ret = rtskb_pool_extend(&sock->skb_pool, rtskbs);
	    sock->pool_size += ret;

	    mutex_unlock(&sock->pool_nrt_lock);

	    if (ret == 0 && rtskbs > 0)
		ret = -ENOMEM;

	    break;

	case RTNET_RTIOC_SHRPOOL:
	    rtskbs = *(unsigned int *)arg;

	    if (rtdm_in_rt_context())
		return -ENOSYS;

	    mutex_lock(&sock->pool_nrt_lock);

	    ret = rtskb_pool_shrink(&sock->skb_pool, rtskbs);
	    sock->pool_size -= ret;

	    mutex_unlock(&sock->pool_nrt_lock);

	    if (ret == 0 && rtskbs > 0)
		ret = -EBUSY;

	    break;

	default:
	    ret = -EOPNOTSUPP;
	    break;
    }

    return ret;
}
EXPORT_SYMBOL_GPL(rt_socket_common_ioctl);



/***
 *  rt_socket_if_ioctl
 */
int rt_socket_if_ioctl(struct rtdm_fd *fd, int request, void *arg)
{
    struct rtnet_device *rtdev;
    struct ifreq        *ifr = arg;
    struct sockaddr_in  *sin;
    int                 ret = 0;


    if (request == SIOCGIFCONF) {
	struct ifconf       *ifc = arg;
	struct ifreq        *cur_ifr = ifc->ifc_req;
	int                 size = 0;
	int                 i;

	for (i = 1; i <= MAX_RT_DEVICES; i++) {
	    rtdev = rtdev_get_by_index(i);
	    if (rtdev != NULL) {
		if ((rtdev->flags & IFF_UP) == 0) {
		    rtdev_dereference(rtdev);
		    continue;
		}

		size += sizeof(struct ifreq);
		if (size > ifc->ifc_len) {
		    rtdev_dereference(rtdev);
		    size = ifc->ifc_len;
		    break;
		}

		strlcpy(cur_ifr->ifr_name, rtdev->name,
			IFNAMSIZ);
		sin = (struct sockaddr_in *)&cur_ifr->ifr_addr;
		sin->sin_family      = AF_INET;
		sin->sin_addr.s_addr = rtdev->local_ip;

		cur_ifr++;
		rtdev_dereference(rtdev);
	    }
	}

	ifc->ifc_len = size;
	return 0;
    }
    if (request == SIOCGIFNAME) {
        rtdev = rtdev_get_by_index(ifr->ifr_ifindex);
        if (rtdev == NULL)
            return -ENODEV;
        strlcpy(ifr->ifr_name, rtdev->name, IFNAMSIZ);
	ret = 0;
	goto out;
    }

    rtdev = rtdev_get_by_name(ifr->ifr_name);
    if (rtdev == NULL)
	return -ENODEV;

    switch (request) {
	case SIOCGIFINDEX:
	    ifr->ifr_ifindex = rtdev->ifindex;
	    break;

	case SIOCGIFFLAGS:
	    ifr->ifr_flags = rtdev->flags;
            if ((ifr->ifr_flags & IFF_UP)
		    && (rtdev->link_state
			    & (RTNET_LINK_STATE_PRESENT
				    | RTNET_LINK_STATE_NOCARRIER))
                    == RTNET_LINK_STATE_PRESENT)
                    ifr->ifr_flags |= IFF_RUNNING;
	    break;

	case SIOCGIFHWADDR:
	    memcpy(ifr->ifr_hwaddr.sa_data, rtdev->dev_addr, rtdev->addr_len);
	    ifr->ifr_hwaddr.sa_family = rtdev->type;
	    break;

	case SIOCGIFADDR:
	    sin = (struct sockaddr_in *)&ifr->ifr_addr;
	    sin->sin_family      = AF_INET;
	    sin->sin_addr.s_addr = rtdev->local_ip;
	    break;

	case SIOCETHTOOL:
	    if (rtdev->do_ioctl != NULL)
		ret = rtdev->do_ioctl(rtdev, request, arg);
	    else
		ret = -EOPNOTSUPP;
	    break;

	default:
	    ret = -EOPNOTSUPP;
	    break;
    }

  out:
    rtdev_dereference(rtdev);
    return ret;
}
EXPORT_SYMBOL_GPL(rt_socket_if_ioctl);


int rt_socket_select_bind(struct rtdm_fd *fd,
			  rtdm_selector_t *selector,
			  enum rtdm_selecttype type,
			  unsigned fd_index)
{
    struct rtsocket *sock = rtdm_fd_to_private(fd);

    switch (type) {
	case XNSELECT_READ:
	    return rtdm_sem_select(&sock->pending_sem, selector,
				XNSELECT_READ, fd_index);
	default:
	    return -EBADF;
    }

    return -EINVAL;
}
EXPORT_SYMBOL_GPL(rt_socket_select_bind);
