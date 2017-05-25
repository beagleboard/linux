/***
 *
 *  ipv4/udp.c - UDP implementation for RTnet
 *
 *  Copyright (C) 1999, 2000 Zentropic Computing, LLC
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
#include <linux/socket.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/tcp.h>
#include <net/checksum.h>
#include <linux/list.h>

#include <rtskb.h>
#include <rtnet_internal.h>
#include <rtnet_port.h>
#include <rtnet_iovec.h>
#include <rtnet_socket.h>
#include <ipv4/ip_fragment.h>
#include <ipv4/ip_output.h>
#include <ipv4/ip_sock.h>
#include <ipv4/protocol.h>
#include <ipv4/route.h>
#include <ipv4/udp.h>


/***
 *  This structure is used to register a UDP socket for reception. All
 +  structures are kept in the port_registry array to increase the cache
 *  locality during the critical port lookup in rt_udp_v4_lookup().
 */
struct udp_socket {
    u16             sport;      /* local port */
    u32             saddr;      /* local ip-addr */
    struct rtsocket *sock;
    struct hlist_node link;
};

/***
 *  Automatic port number assignment

 *  The automatic assignment of port numbers to unbound sockets is realised as
 *  a simple addition of two values:
 *   - the socket ID (lower 8 bits of file descriptor) which is set during
 *     initialisation and left unchanged afterwards
 *   - the start value auto_port_start which is a module parameter

 *  auto_port_mask, also a module parameter, is used to define the range of
 *  port numbers which are used for automatic assignment. Any number within
 *  this range will be rejected when passed to bind_rt().

 */
static unsigned int         auto_port_start = 1024;
static unsigned int         auto_port_mask  = ~(RT_UDP_SOCKETS-1);
static int                  free_ports      = RT_UDP_SOCKETS;
#define RT_PORT_BITMAP_WORDS \
    ((RT_UDP_SOCKETS + BITS_PER_LONG - 1) / BITS_PER_LONG)
static unsigned long        port_bitmap[RT_PORT_BITMAP_WORDS];
static struct udp_socket    port_registry[RT_UDP_SOCKETS];
static DEFINE_RTDM_LOCK(udp_socket_base_lock);

static struct hlist_head port_hash[RT_UDP_SOCKETS * 2];
#define port_hash_mask (RT_UDP_SOCKETS * 2 - 1)

MODULE_LICENSE("GPL");

module_param(auto_port_start, uint, 0444);
module_param(auto_port_mask, uint, 0444);
MODULE_PARM_DESC(auto_port_start, "Start of automatically assigned port range");
MODULE_PARM_DESC(auto_port_mask,
                 "Mask that defines port range for automatic assignment");

static inline struct udp_socket *port_hash_search(u32 saddr, u16 sport)
{
        unsigned bucket = sport & port_hash_mask;
        struct udp_socket *sock;

        hlist_for_each_entry(sock, &port_hash[bucket], link)
                if (sock->sport == sport &&
                    (saddr == INADDR_ANY
                     || sock->saddr == saddr
                     || sock->saddr == INADDR_ANY))
                        return sock;

        return NULL;
}

static inline int port_hash_insert(struct udp_socket *sock, u32 saddr, u16 sport)
{
        unsigned bucket;

        if (port_hash_search(saddr, sport))
                return -EADDRINUSE;

        bucket = sport & port_hash_mask;
        sock->saddr = saddr;
        sock->sport = sport;
        hlist_add_head(&sock->link, &port_hash[bucket]);
        return 0;
}

static inline void port_hash_del(struct udp_socket *sock)
{
        hlist_del(&sock->link);
}

/***
 *  rt_udp_v4_lookup
 */
static inline struct rtsocket *rt_udp_v4_lookup(u32 daddr, u16 dport)
{
    rtdm_lockctx_t  context;
    struct udp_socket *sock;

    rtdm_lock_get_irqsave(&udp_socket_base_lock, context);
    sock = port_hash_search(daddr, dport);
    if (sock && rt_socket_reference(sock->sock) == 0) {

            rtdm_lock_put_irqrestore(&udp_socket_base_lock, context);

            return sock->sock;
    }

    rtdm_lock_put_irqrestore(&udp_socket_base_lock, context);

    return NULL;
}



/***
 *  rt_udp_bind - bind socket to local address
 *  @s:     socket
 *  @addr:  local address
 */
int rt_udp_bind(struct rtsocket *sock, const struct sockaddr *addr,
                socklen_t addrlen)
{
    struct sockaddr_in  *usin = (struct sockaddr_in *)addr;
    rtdm_lockctx_t      context;
    int                 index;
    int                 err = 0;


    if ((addrlen < (int)sizeof(struct sockaddr_in)) ||
        ((usin->sin_port & auto_port_mask) == auto_port_start))
        return -EINVAL;

    rtdm_lock_get_irqsave(&udp_socket_base_lock, context);

    if ((index = sock->prot.inet.reg_index) < 0) {
        /* socket is being closed */
        err = -EBADF;
        goto unlock_out;
    }
    if (sock->prot.inet.state != TCP_CLOSE) {
        err = -EINVAL;
        goto unlock_out;
    }

    port_hash_del(&port_registry[index]);
    if (port_hash_insert(&port_registry[index],
                         usin->sin_addr.s_addr,
                         usin->sin_port ?: index + auto_port_start)) {
            port_hash_insert(&port_registry[index],
                             port_registry[index].saddr,
                             port_registry[index].sport);
            rtdm_lock_put_irqrestore(&udp_socket_base_lock, context);
            return -EADDRINUSE;
    }

    /* set the source-addr */
    sock->prot.inet.saddr = port_registry[index].saddr;

    /* set source port, if not set by user */
    sock->prot.inet.sport = port_registry[index].sport;

 unlock_out:
    rtdm_lock_put_irqrestore(&udp_socket_base_lock, context);

    return err;
}



/***
 *  rt_udp_connect
 */
int rt_udp_connect(struct rtsocket *sock, const struct sockaddr *serv_addr,
                   socklen_t addrlen)
{
    struct sockaddr_in  *usin = (struct sockaddr_in *) serv_addr;
    rtdm_lockctx_t      context;
    int                 index;


    if (usin->sin_family == AF_UNSPEC) {
        if ((index = sock->prot.inet.reg_index) < 0)
            /* socket is being closed */
            return -EBADF;

        rtdm_lock_get_irqsave(&udp_socket_base_lock, context);

        sock->prot.inet.saddr = INADDR_ANY;
        /* Note: The following line differs from standard stacks, and we also
                 don't remove the socket from the port list. Might get fixed in
                 the future... */
        sock->prot.inet.sport = index + auto_port_start;
        sock->prot.inet.daddr = INADDR_ANY;
        sock->prot.inet.dport = 0;
        sock->prot.inet.state = TCP_CLOSE;

        rtdm_lock_put_irqrestore(&udp_socket_base_lock, context);
    } else {
        if ((addrlen < (int)sizeof(struct sockaddr_in)) ||
            (usin->sin_family != AF_INET))
            return -EINVAL;

        rtdm_lock_get_irqsave(&udp_socket_base_lock, context);

        if (sock->prot.inet.state != TCP_CLOSE) {
            rtdm_lock_put_irqrestore(&udp_socket_base_lock, context);
            return -EINVAL;
        }

        sock->prot.inet.state = TCP_ESTABLISHED;
        sock->prot.inet.daddr = usin->sin_addr.s_addr;
        sock->prot.inet.dport = usin->sin_port;

        rtdm_lock_put_irqrestore(&udp_socket_base_lock, context);
    }

    return 0;
}



/***
 *  rt_udp_socket - create a new UDP-Socket
 *  @s: socket
 */
int rt_udp_socket(struct rtdm_fd *fd)
{
    struct rtsocket *sock = rtdm_fd_to_private(fd);
    int             ret;
    int             i;
    int             index;
    rtdm_lockctx_t  context;


    if ((ret = rt_socket_init(fd, IPPROTO_UDP)) != 0)
        return ret;

    sock->prot.inet.saddr = INADDR_ANY;
    sock->prot.inet.state = TCP_CLOSE;
    sock->prot.inet.tos   = 0;

    rtdm_lock_get_irqsave(&udp_socket_base_lock, context);

    /* enforce maximum number of UDP sockets */
    if (free_ports == 0) {
        rtdm_lock_put_irqrestore(&udp_socket_base_lock, context);
        rt_socket_cleanup(fd);
        return -EAGAIN;
    }
    free_ports--;

    /* find free auto-port in bitmap */
    for (i = 0; i < RT_PORT_BITMAP_WORDS; i++)
        if (port_bitmap[i] != (unsigned long)-1)
            break;
    index = ffz(port_bitmap[i]);
    set_bit(index, &port_bitmap[i]);
    index += i*32;
    sock->prot.inet.reg_index = index;
    sock->prot.inet.sport     = index + auto_port_start;

    /* register UDP socket */
    port_hash_insert(&port_registry[index], INADDR_ANY, sock->prot.inet.sport);
    port_registry[index].sock  = sock;

    rtdm_lock_put_irqrestore(&udp_socket_base_lock, context);

    return 0;
}



/***
 *  rt_udp_close
 */
void rt_udp_close(struct rtdm_fd *fd)
{
    struct rtsocket *sock = rtdm_fd_to_private(fd);
    struct rtskb    *del;
    int             port;
    rtdm_lockctx_t  context;


    rtdm_lock_get_irqsave(&udp_socket_base_lock, context);

    sock->prot.inet.state = TCP_CLOSE;

    if (sock->prot.inet.reg_index >= 0) {
        port = sock->prot.inet.reg_index;
        clear_bit(port % BITS_PER_LONG, &port_bitmap[port / BITS_PER_LONG]);
        port_hash_del(&port_registry[port]);

        free_ports++;

        sock->prot.inet.reg_index = -1;
    }

    rtdm_lock_put_irqrestore(&udp_socket_base_lock, context);

    /* cleanup already collected fragments */
    rt_ip_frag_invalidate_socket(sock);

    /* free packets in incoming queue */
    while ((del = rtskb_dequeue(&sock->incoming)) != NULL)
        kfree_rtskb(del);

    rt_socket_cleanup(fd);
}



int rt_udp_ioctl(struct rtdm_fd *fd, unsigned int request, void *arg)
{
    struct rtsocket *sock = rtdm_fd_to_private(fd);
    struct _rtdm_setsockaddr_args *setaddr = arg;


    /* fast path for common socket IOCTLs */
    if (_IOC_TYPE(request) == RTIOC_TYPE_NETWORK)
        return rt_socket_common_ioctl(fd, request, arg);

    switch (request) {
        case _RTIOC_BIND:
            return rt_udp_bind(sock, setaddr->addr, setaddr->addrlen);

        case _RTIOC_CONNECT:
            return rt_udp_connect(sock, setaddr->addr, setaddr->addrlen);

        default:
            return rt_ip_ioctl(fd, request, arg);
    }
}



/***
 *  rt_udp_recvmsg
 */
ssize_t rt_udp_recvmsg(struct rtdm_fd *fd, struct user_msghdr *msg, int msg_flags)
{
    struct rtsocket     *sock = rtdm_fd_to_private(fd);
    size_t              len   = rt_iovec_len(msg->msg_iov, msg->msg_iovlen);
    struct rtskb        *skb;
    struct rtskb        *first_skb;
    size_t              copied = 0;
    size_t              block_size;
    size_t              data_len;
    struct udphdr       *uh;
    struct sockaddr_in  *sin;
    nanosecs_rel_t      timeout = sock->timeout;
    int                 ret;


    /* non-blocking receive? */
    if (msg_flags & MSG_DONTWAIT)
        timeout = -1;

    ret = rtdm_sem_timeddown(&sock->pending_sem, timeout, NULL);
    if (unlikely(ret < 0))
        switch (ret) {
            case -EWOULDBLOCK:
            case -ETIMEDOUT:
            case -EINTR:
                return ret;

            default:
                return -EBADF;   /* socket has been closed */
        }

    skb = rtskb_dequeue_chain(&sock->incoming);
    RTNET_ASSERT(skb != NULL, return -EFAULT;);

    uh = skb->h.uh;
    data_len = ntohs(uh->len) - sizeof(struct udphdr);
    sin = msg->msg_name;

    /* copy the address */
    msg->msg_namelen = sizeof(*sin);
    if (sin) {
        sin->sin_family      = AF_INET;
        sin->sin_port        = uh->source;
        sin->sin_addr.s_addr = skb->nh.iph->saddr;
    }

    /* remove the UDP header */
    __rtskb_pull(skb, sizeof(struct udphdr));

    first_skb = skb;

    /* iterate over all IP fragments */
    do {
        rtskb_trim(skb, data_len);

        block_size = skb->len;
        copied += block_size;
        data_len -= block_size;

        /* The data must not be longer than the available buffer size */
        if (copied > len) {
            block_size -= copied - len;
            copied = len;
            msg->msg_flags |= MSG_TRUNC;

            /* copy the data */
            rt_memcpy_tokerneliovec(msg->msg_iov, skb->data, block_size);

            break;
        }

        /* copy the data */
        rt_memcpy_tokerneliovec(msg->msg_iov, skb->data, block_size);

        /* next fragment */
        skb = skb->next;
    } while (skb != NULL);

    /* did we copied all bytes? */
    if (data_len > 0)
        msg->msg_flags |= MSG_TRUNC;

    if ((msg_flags & MSG_PEEK) == 0)
        kfree_rtskb(first_skb);
    else {
        __rtskb_push(first_skb, sizeof(struct udphdr));
        rtskb_queue_head(&sock->incoming, first_skb);
        rtdm_sem_up(&sock->pending_sem);
    }

    return copied;
}



/***
 *  struct udpfakehdr
 */
struct udpfakehdr
{
    struct udphdr uh;
    u32 daddr;
    u32 saddr;
    struct iovec *iov;
    int iovlen;
    u32 wcheck;
};



/***
 *
 */
static int rt_udp_getfrag(const void *p, unsigned char *to,
                          unsigned int offset, unsigned int fraglen)
{
    struct udpfakehdr *ufh = (struct udpfakehdr *)p;
    int i;


    // We should optimize this function a bit (copy+csum...)!
    if (offset==0) {
        /* Checksum of the complete data part of the UDP message: */
        for (i = 0; i < ufh->iovlen; i++) {
            ufh->wcheck = csum_partial(ufh->iov[i].iov_base, ufh->iov[i].iov_len,
                                       ufh->wcheck);
        }

        rt_memcpy_fromkerneliovec(to + sizeof(struct udphdr), ufh->iov,
                                  fraglen - sizeof(struct udphdr));

        /* Checksum of the udp header: */
        ufh->wcheck = csum_partial((unsigned char *)ufh,
                                   sizeof(struct udphdr), ufh->wcheck);

        ufh->uh.check = csum_tcpudp_magic(ufh->saddr, ufh->daddr, ntohs(ufh->uh.len),
                                          IPPROTO_UDP, ufh->wcheck);

        if (ufh->uh.check == 0)
            ufh->uh.check = -1;

        memcpy(to, ufh, sizeof(struct udphdr));
        return 0;
    }

    rt_memcpy_fromkerneliovec(to, ufh->iov, fraglen);

    return 0;
}



/***
 *  rt_udp_sendmsg
 */
ssize_t rt_udp_sendmsg(struct rtdm_fd *fd, const struct user_msghdr *msg, int msg_flags)
{
    struct rtsocket     *sock = rtdm_fd_to_private(fd);
    size_t              len   = rt_iovec_len(msg->msg_iov, msg->msg_iovlen);
    int                 ulen  = len + sizeof(struct udphdr);
    struct sockaddr_in  *usin;
    struct udpfakehdr   ufh;
    struct dest_route   rt;
    u32                 saddr;
    u32                 daddr;
    u16                 dport;
    int                 err;
    rtdm_lockctx_t      context;


    if ((len < 0) || (len > 0xFFFF-sizeof(struct iphdr)-sizeof(struct udphdr)))
        return -EMSGSIZE;

    if (msg_flags & MSG_OOB)   /* Mirror BSD error message compatibility */
        return -EOPNOTSUPP;

    if (msg_flags & ~(MSG_DONTROUTE|MSG_DONTWAIT) )
        return -EINVAL;

    if ((msg->msg_name) && (msg->msg_namelen==sizeof(struct sockaddr_in))) {
        usin = (struct sockaddr_in*) msg->msg_name;

        if ((usin->sin_family != AF_INET) && (usin->sin_family != AF_UNSPEC))
            return -EINVAL;

        daddr = usin->sin_addr.s_addr;
        dport = usin->sin_port;

        rtdm_lock_get_irqsave(&udp_socket_base_lock, context);
    } else {
        rtdm_lock_get_irqsave(&udp_socket_base_lock, context);

        if (sock->prot.inet.state != TCP_ESTABLISHED) {
	    rtdm_lock_put_irqrestore(&udp_socket_base_lock, context);
            return -ENOTCONN;
	}

        daddr = sock->prot.inet.daddr;
        dport = sock->prot.inet.dport;
    }
    saddr         = sock->prot.inet.saddr;
    ufh.uh.source = sock->prot.inet.sport;

    rtdm_lock_put_irqrestore(&udp_socket_base_lock, context);

    if ((daddr | dport) == 0)
        return -EINVAL;

    /* get output route */
    err = rt_ip_route_output(&rt, daddr, saddr);
    if (err)
        return err;

    /* we found a route, remember the routing dest-addr could be the netmask */
    ufh.saddr     = saddr != INADDR_ANY ? saddr : rt.rtdev->local_ip;
    ufh.daddr     = daddr;
    ufh.uh.dest   = dport;
    ufh.uh.len    = htons(ulen);
    ufh.uh.check  = 0;
    ufh.iov       = msg->msg_iov;
    ufh.iovlen    = msg->msg_iovlen;
    ufh.wcheck    = 0;

    err = rt_ip_build_xmit(sock, rt_udp_getfrag, &ufh, ulen, &rt, msg_flags);

    rtdev_dereference(rt.rtdev);

    if (!err)
        return len;
    else
        return err;
}



/***
 *  rt_udp_check
 */
static inline unsigned short rt_udp_check(struct udphdr *uh, int len,
                                          unsigned long saddr,
                                          unsigned long daddr,
                                          unsigned long base)
{
    return(csum_tcpudp_magic(saddr, daddr, len, IPPROTO_UDP, base));
}



struct rtsocket *rt_udp_dest_socket(struct rtskb *skb)
{
    struct udphdr           *uh   = skb->h.uh;
    unsigned short          ulen  = ntohs(uh->len);
    u32                     saddr = skb->nh.iph->saddr;
    u32                     daddr = skb->nh.iph->daddr;
    struct rtnet_device*    rtdev = skb->rtdev;


    if (uh->check == 0)
        skb->ip_summed = CHECKSUM_UNNECESSARY;
/* ip_summed (yet) never equals CHECKSUM_PARTIAL
    else
        if (skb->ip_summed == CHECKSUM_PARTIAL) {
            skb->ip_summed = CHECKSUM_UNNECESSARY;

            if ( !rt_udp_check(uh, ulen, saddr, daddr, skb->csum) )
                return NULL;

            skb->ip_summed = CHECKSUM_NONE;
        }*/

    if (skb->ip_summed != CHECKSUM_UNNECESSARY)
        skb->csum = csum_tcpudp_nofold(saddr, daddr, ulen, IPPROTO_UDP, 0);

    /* patch broadcast daddr */
    if (daddr == rtdev->broadcast_ip)
        daddr = rtdev->local_ip;

    /* find the destination socket */
    skb->sk = rt_udp_v4_lookup(daddr, uh->dest);

    return skb->sk;
}

/***
 *  rt_udp_rcv
 */
void rt_udp_rcv (struct rtskb *skb)
{
    struct rtsocket *sock = skb->sk;
    void            (*callback_func)(struct rtdm_fd *, void *);
    void            *callback_arg;
    rtdm_lockctx_t  context;

    rtskb_queue_tail(&sock->incoming, skb);
    rtdm_sem_up(&sock->pending_sem);

    rtdm_lock_get_irqsave(&sock->param_lock, context);
    callback_func = sock->callback_func;
    callback_arg  = sock->callback_arg;
    rtdm_lock_put_irqrestore(&sock->param_lock, context);

    if (callback_func)
        callback_func(rt_socket_fd(sock), callback_arg);
}



/***
 *  rt_udp_rcv_err
 */
void rt_udp_rcv_err (struct rtskb *skb)
{
    rtdm_printk("RTnet: rt_udp_rcv err\n");
}



/***
 *  UDP-Initialisation
 */
static struct rtinet_protocol udp_protocol = {
    .protocol =     IPPROTO_UDP,
    .dest_socket =  &rt_udp_dest_socket,
    .rcv_handler =  &rt_udp_rcv,
    .err_handler =  &rt_udp_rcv_err,
    .init_socket =  &rt_udp_socket
};

static struct rtdm_driver udp_driver = {
    .profile_info =     RTDM_PROFILE_INFO(udp,
                                        RTDM_CLASS_NETWORK,
                                        RTDM_SUBCLASS_RTNET,
                                        RTNET_RTDM_VER),
    .device_flags =     RTDM_PROTOCOL_DEVICE,
    .device_count =	1,
    .context_size =     sizeof(struct rtsocket),

    .protocol_family =  PF_INET,
    .socket_type =      SOCK_DGRAM,

    /* default is UDP */
    .ops = {
        .socket =       rt_inet_socket,
        .close =        rt_udp_close,
        .ioctl_rt =     rt_udp_ioctl,
        .ioctl_nrt =    rt_udp_ioctl,
        .recvmsg_rt =   rt_udp_recvmsg,
        .sendmsg_rt =   rt_udp_sendmsg,
        .select =       rt_socket_select_bind,
    },
};

static struct rtdm_device udp_device = {
    .driver = &udp_driver,
    .label = "udp",
};

/***
 *  rt_udp_init
 */
static int __init rt_udp_init(void)
{
    int i;
    if ((auto_port_start < 0) || (auto_port_start >= 0x10000 - RT_UDP_SOCKETS))
        auto_port_start = 1024;
    auto_port_start = htons(auto_port_start & (auto_port_mask & 0xFFFF));
    auto_port_mask  = htons(auto_port_mask | 0xFFFF0000);

    rt_inet_add_protocol(&udp_protocol);

    for (i = 0; i < ARRAY_SIZE(port_hash); i++)
            INIT_HLIST_HEAD(&port_hash[i]);

    return rtdm_dev_register(&udp_device);
}



/***
 *  rt_udp_release
 */
static void __exit rt_udp_release(void)
{
    rtdm_dev_unregister(&udp_device);
    rt_inet_del_protocol(&udp_protocol);
}

module_init(rt_udp_init);
module_exit(rt_udp_release);
