/***
 *
 *  ipv4/tcp/tcp.c - TCP implementation for RTnet
 *
 *  Copyright (C) 2009 Vladimir Zapolskiy <vladimir.zapolskiy@siemens.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License, version 2, as
 *  published by the Free Software Foundation.
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
#include <linux/list.h>
#include <linux/skbuff.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <net/tcp_states.h>
#include <net/tcp.h>

#include <rtdm/driver.h>
#include <rtnet_rtpc.h>
#include <rtskb.h>
#include <rtdev.h>
#include <rtnet_port.h>
#include <ipv4/tcp.h>
#include <ipv4/ip_sock.h>
#include <ipv4/ip_output.h>
#include <ipv4/ip_fragment.h>
#include <ipv4/route.h>
#include <ipv4/af_inet.h>
#include "timerwheel.h"

static unsigned int close_timeout = 1000;
module_param(close_timeout, uint, 0664);
MODULE_PARM_DESC(close_timeout,
		 "max time (ms) to wait during close for FIN-ACK handshake to complete, default 1000");

#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_TCP_ERROR_INJECTION

static unsigned int error_rate;
module_param(error_rate, uint, 0664);
MODULE_PARM_DESC(error_rate, "simulate packet loss after every n packets");

static unsigned int multi_error = 1;
module_param(multi_error, uint, 0664);
MODULE_PARM_DESC(multi_error, "on simulated error, drop n packets in a row");

static unsigned int counter_start = 1234;
module_param(counter_start, uint, 0664);
MODULE_PARM_DESC(counter_start, "start value of per-socket packet counter "
				"(used for error injection)");

#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_TCP_ERROR_INJECTION */

struct tcp_sync {
	u32 seq;
	u32 ack_seq;

	/* Local window size sent to peer  */
	u16 window;
	/* Last received destination peer window size */
	u16 dst_window;
};

/*
  connection timeout
*/
/* 5 second */
static const nanosecs_rel_t rt_tcp_connection_timeout = 1000000000ull;

/* retransmission timerwheel timeout */
static const u64 rt_tcp_retransmit_timeout = 100000000ull;

/*
  keepalive constants
*/
/* 75 second */
static const u64 rt_tcp_keepalive_intvl = 75000000000ull;
/* 9 probes to send */
static const u8 rt_tcp_keepalive_probes = 9;
/* 2 hour */
static const u64 rt_tcp_keepalive_timeout = 7200000000000ull;

/*
  retransmission timeout
*/
/* 50 millisecond */
static const nanosecs_rel_t rt_tcp_retransmission_timeout = 50000000ull;
/*
  maximum allowed number of retransmissions
*/
static const unsigned int max_retransmits = 3;

struct tcp_keepalive {
	u8 enabled;
	u32 probes;
	rtdm_timer_t timer;
};

/***
 *  This structure is used to register a TCP socket for reception. All
 *  structures are kept in the port_registry array to increase the cache
 *  locality during the critical port lookup in rt_tcp_v4_lookup().
 */

/* if dport & daddr are zeroes, it means a listening socket */
/* otherwise this is a data structure, which describes a connection */

/* NB: sock->prot.inet.saddr & sock->prot.inet.sport values are not used */
struct tcp_socket {
	struct rtsocket sock; /* set up by rt_socket_init() implicitly */
	u16 sport; /* local port */
	u32 saddr; /* local ip-addr */
	u16 dport; /* destination port */
	u32 daddr; /* destination ip-addr */

	u8 tcp_state; /* tcp connection state */

	u8 is_binding; /* if set, tcp socket is in port binding progress */
	u8 is_bound; /* if set, tcp socket is already port bound */
	u8 is_valid; /* if set, read() and write() can process */
	u8 is_accepting; /* if set, accept() is in progress */
	u8 is_accepted; /* if set, accept() is already called */
	u8 is_closed; /* close() call for resource deallocation follows */

	rtdm_event_t send_evt; /* write request is permissible */
	rtdm_event_t conn_evt; /* connection event */

	struct dest_route rt;
	struct tcp_sync sync;
	struct tcp_keepalive keepalive;
	rtdm_lock_t socket_lock;

	struct hlist_node link;

	nanosecs_rel_t sk_sndtimeo;

	/* retransmission routine data */
	u32 nacked_first;
	unsigned int timer_state;
	struct rtskb_queue retransmit_queue;
	struct timerwheel_timer timer;

	struct completion fin_handshake;
	rtdm_nrtsig_t close_sig;

#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_TCP_ERROR_INJECTION
	unsigned int packet_counter;
	unsigned int error_rate;
	unsigned int multi_error;
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_TCP_ERROR_INJECTION */
};

struct rt_tcp_dispatched_packet_send_cmd {
	__be32 flags; /* packet flags value */
	struct tcp_socket *ts;
};

/***
 *  Automatic port number assignment

 *  The automatic assignment of port numbers to unbound sockets is realised as
 *  a simple addition of two values:
 *   - the socket ID (lower 8 bits of file descriptor) which is set during
 *     initialisation and left unchanged afterwards
 *   - the start value tcp_auto_port_start which is a module parameter

 *  tcp_auto_port_mask, also a module parameter, is used to define the range of
 *  port numbers which are used for automatic assignment. Any number within
 *  this range will be rejected when passed to bind_rt().

 */

MODULE_LICENSE("GPL");

static struct {
	struct rtdm_dev_context dummy;
	struct tcp_socket rst_socket;
} rst_socket_container;

#define rst_fd (&rst_socket_container.dummy.fd)
#define rst_socket (*(struct tcp_socket *)rtdm_fd_to_private(rst_fd))

static u32 tcp_auto_port_start = 1024;
static u32 tcp_auto_port_mask = ~(RT_TCP_SOCKETS - 1);
static u32 free_ports = RT_TCP_SOCKETS;
#define RT_PORT_BITMAP_WORDS                                                   \
	((RT_TCP_SOCKETS + BITS_PER_LONG - 1) / BITS_PER_LONG)
static unsigned long port_bitmap[RT_PORT_BITMAP_WORDS];

static struct tcp_socket *port_registry[RT_TCP_SOCKETS];
static DEFINE_RTDM_LOCK(tcp_socket_base_lock);

static struct hlist_head port_hash[RT_TCP_SOCKETS * 2];
#define port_hash_mask (RT_TCP_SOCKETS * 2 - 1)

module_param(tcp_auto_port_start, uint, 0444);
module_param(tcp_auto_port_mask, uint, 0444);
MODULE_PARM_DESC(tcp_auto_port_start, "Start of automatically assigned "
				      "port range for TCP");
MODULE_PARM_DESC(tcp_auto_port_mask, "Mask that defines port range for TCP "
				     "for automatic assignment");

static inline struct tcp_socket *port_hash_search(u32 saddr, u16 sport)
{
	u32 bucket = sport & port_hash_mask;
	struct tcp_socket *ts;

	hlist_for_each_entry (ts, &port_hash[bucket], link)
		if (ts->sport == sport &&
		    (saddr == INADDR_ANY || ts->saddr == saddr ||
		     ts->saddr == INADDR_ANY))
			return ts;

	return NULL;
}

static int port_hash_insert(struct tcp_socket *ts, u32 saddr, u16 sport)
{
	u32 bucket;

	if (port_hash_search(saddr, sport))
		return -EADDRINUSE;

	bucket = sport & port_hash_mask;
	ts->saddr = saddr;
	ts->sport = sport;
	ts->daddr = 0;
	ts->dport = 0;

	hlist_add_head(&ts->link, &port_hash[bucket]);

	return 0;
}

static inline void port_hash_del(struct tcp_socket *ts)
{
	hlist_del(&ts->link);
}

/***
 *  rt_tcp_v4_lookup
 */
static struct rtsocket *rt_tcp_v4_lookup(u32 daddr, u16 dport)
{
	rtdm_lockctx_t context;
	struct tcp_socket *ts;
	int ret;

	rtdm_lock_get_irqsave(&tcp_socket_base_lock, context);
	ts = port_hash_search(daddr, dport);

	if (ts != NULL) {
		ret = rt_socket_reference(&ts->sock);
		if (ret == 0 || (ret == -EIDRM && ts->is_closed)) {
			rtdm_lock_put_irqrestore(&tcp_socket_base_lock,
						 context);

			return &ts->sock;
		}
	}

	rtdm_lock_put_irqrestore(&tcp_socket_base_lock, context);

	return NULL;
}

/* test seq1 <= seq2 */
static inline int rt_tcp_before(__u32 seq1, __u32 seq2)
{
	return (__s32)(seq1 - seq2) <= 0;
}

/* test seq1 => seq2 */
static inline int rt_tcp_after(__u32 seq1, __u32 seq2)
{
	return (__s32)(seq2 - seq1) <= 0;
}

static inline u32 rt_tcp_compute_ack_seq(struct tcphdr *th, u32 len)
{
	u32 ack_seq = ntohl(th->seq) + len;

	if (unlikely(th->syn || th->fin))
		ack_seq++;

	return ack_seq;
}

static void rt_tcp_keepalive_start(struct tcp_socket *ts)
{
	if (ts->tcp_state == TCP_ESTABLISHED) {
		rtdm_timer_start(&ts->keepalive.timer, rt_tcp_keepalive_timeout,
				 0, RTDM_TIMERMODE_RELATIVE);
	}
}

static void rt_tcp_keepalive_stop(struct tcp_socket *ts)
{
	if (ts->tcp_state == TCP_ESTABLISHED) {
		rtdm_timer_stop(&ts->keepalive.timer);
	}
}

#ifdef YET_UNUSED
static void rt_tcp_keepalive_timer(rtdm_timer_t *timer);

static void rt_tcp_keepalive_enable(struct tcp_socket *ts)
{
	rtdm_lockctx_t context;
	struct tcp_keepalive *keepalive;

	rtdm_lock_get_irqsave(&ts->socket_lock, context);

	keepalive = &ts->keepalive;

	if (keepalive->enabled) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return;
	}

	keepalive->probes = rt_tcp_keepalive_probes;

	rtdm_timer_init(&keepalive->timer, rt_tcp_keepalive_timer,
			"RT TCP keepalive timer");

	rt_tcp_keepalive_start(ts);

	keepalive->enabled = 1;

	rtdm_lock_put_irqrestore(&ts->socket_lock, context);
}
#endif

static void rt_tcp_keepalive_disable(struct tcp_socket *ts)
{
	struct tcp_keepalive *keepalive;

	keepalive = &ts->keepalive;

	if (!keepalive->enabled) {
		return;
	}

	rt_tcp_keepalive_stop(ts);
	rtdm_timer_destroy(&keepalive->timer);

	keepalive->enabled = 0;
}

static void rt_tcp_keepalive_feed(struct tcp_socket *ts)
{
	rtdm_lockctx_t context;
	struct tcp_keepalive *keepalive;

	rtdm_lock_get_irqsave(&ts->socket_lock, context);

	keepalive = &ts->keepalive;

	if (ts->tcp_state == TCP_ESTABLISHED && ts->keepalive.enabled) {
		keepalive->probes = rt_tcp_keepalive_probes;

		/* Restart keepalive timer */
		rtdm_timer_stop(&keepalive->timer);
		rtdm_timer_start(&keepalive->timer, rt_tcp_keepalive_timeout, 0,
				 RTDM_TIMERMODE_RELATIVE);

		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
	} else {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
	}
}

static int rt_tcp_socket_invalidate(struct tcp_socket *ts, u8 to_state)
{
	int signal = ts->is_valid;

	ts->tcp_state = to_state;

	/*
      multiple invalidation could happen without fuss,
      see rt_tcp_close(), rt_tcp_rcv(), timeout expiration etc.
    */
	if (ts->is_valid) {
		ts->is_valid = 0;

		if (ts->keepalive.enabled) {
			rt_tcp_keepalive_stop(ts);
		}
	}

	return signal;
}

static void rt_tcp_socket_invalidate_signal(struct tcp_socket *ts)
{
	/* awake all readers and writers destroying events */
	rtdm_sem_destroy(&ts->sock.pending_sem);
	rtdm_event_destroy(&ts->send_evt);
}

static void rt_tcp_socket_validate(struct tcp_socket *ts)
{
	ts->tcp_state = TCP_ESTABLISHED;

	ts->is_valid = 1;

	if (ts->keepalive.enabled) {
		rt_tcp_keepalive_start(ts);
	}

	rtdm_event_init(&ts->send_evt, 0);
}

/***
 *  rt_tcp_retransmit_handler - timerwheel handler to process a retransmission
 *  @data: pointer to a rttcp socket structure
 */
static void rt_tcp_retransmit_handler(void *data)
{
	struct tcp_socket *ts = (struct tcp_socket *)data;
	struct rtskb *skb;
	rtdm_lockctx_t context;
	int signal;

	rtdm_lock_get_irqsave(&ts->socket_lock, context);

	if (unlikely(rtskb_queue_empty(&ts->retransmit_queue))) {
		/* handled, but retransmission queue is empty */
		rtdm_lock_get_irqsave(&ts->socket_lock, context);
		rtdm_printk("rttcp: bug in RT TCP retransmission routine\n");
		return;
	}

	if (ts->tcp_state == TCP_CLOSE) {
		/* socket is already closed */
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return;
	}

	if (ts->timer_state) {
		/* more tries */
		ts->timer_state--;
		timerwheel_add_timer(&ts->timer, rt_tcp_retransmission_timeout);

		/* warning, rtskb_clone is under lock */
		skb = rtskb_clone(ts->retransmit_queue.first,
				  &ts->sock.skb_pool);
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);

		/* BUG, window changes are not respected */
		if (unlikely(rtdev_xmit(skb)) != 0) {
			kfree_rtskb(skb);
			rtdm_printk(
				"rttcp: packet retransmission from timer failed\n");
		}
	} else {
		ts->timer_state = max_retransmits;

		/* report about connection lost */
		signal = rt_tcp_socket_invalidate(ts, TCP_CLOSE);
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);

		if (signal)
			rt_tcp_socket_invalidate_signal(ts);

		/* retransmission queue will be cleaned up in rt_tcp_socket_destruct */
		rtdm_printk("rttcp: connection is lost by NACK timeout\n");
	}
}

/***
 *  rt_tcp_retransmit_ack - remove skbs from retransmission queue on ACK
 *  @ts: rttcp socket
 *  @ack_seq: received ACK sequence value
 */
static void rt_tcp_retransmit_ack(struct tcp_socket *ts, u32 ack_seq)
{
	struct rtskb *skb;
	rtdm_lockctx_t context;

	rtdm_lock_get_irqsave(&ts->socket_lock, context);

	/*
      ACK, but retransmission queue is empty
      This could happen on repeated ACKs
    */
	if (rtskb_queue_empty(&ts->retransmit_queue)) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return;
	}

	/*
      Check ts->nacked_first value firstly to ensure that
      skb for retransmission is present in the queue, otherwise
      retransmission queue will be drained completely
    */
	if (!rt_tcp_before(ts->nacked_first, ack_seq)) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return;
	}

	if (timerwheel_remove_timer(&ts->timer) != 0) {
		/* already timed out */
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return;
	}

dequeue_loop:
	if (ts->tcp_state == TCP_CLOSE) {
		/* warn about queue safety in race with anyone,
	   who closes the socket */
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return;
	}

	if ((skb = __rtskb_dequeue(&ts->retransmit_queue)) == NULL) {
		ts->timer_state = max_retransmits;
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return;
	}

	if (rt_tcp_before(ts->nacked_first, ack_seq)) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		kfree_rtskb(skb);
		rtdm_lock_get_irqsave(&ts->socket_lock, context);
		goto dequeue_loop;
	}

	/* Put NACKed skb back to queue */
	/* BUG, need to respect half-acknowledged packets */
	ts->nacked_first = ntohl(skb->h.th->seq) + 1;

	__rtskb_queue_head(&ts->retransmit_queue, skb);

	/* Have more packages in retransmission queue, restart the timer */
	timerwheel_add_timer(&ts->timer, rt_tcp_retransmission_timeout);

	rtdm_lock_put_irqrestore(&ts->socket_lock, context);
}

/***
 *  rt_tcp_retransmit_send - enqueue a skb to retransmission queue (not locked)
 *  @ts: rttcp socket
 *  @skb: a copied skb for enqueueing
 */
static void rt_tcp_retransmit_send(struct tcp_socket *ts, struct rtskb *skb)
{
	if (rtskb_queue_empty(&ts->retransmit_queue)) {
		/* retransmission queue is empty */
		ts->nacked_first = ntohl(skb->h.th->seq) + 1;

		__rtskb_queue_tail(&ts->retransmit_queue, skb);

		timerwheel_add_timer(&ts->timer, rt_tcp_retransmission_timeout);
	} else {
		/* retransmission queue is not empty */
		__rtskb_queue_tail(&ts->retransmit_queue, skb);
	}
}

static int rt_ip_build_frame(struct rtskb *skb, struct rtsocket *sk,
			     struct dest_route *rt, struct iphdr *iph)
{
	int ret;
	struct rtnet_device *rtdev = rt->rtdev;

	RTNET_ASSERT(rtdev->hard_header, return -EBADF;);

	if (!rtdev_reference(rt->rtdev))
		return -EIDRM;

	iph->ihl = 5; /* 20 byte header only - no TCP options */

	skb->nh.iph = iph;

	iph->version = 4;
	iph->tos = sk->prot.inet.tos;
	iph->tot_len = htons(skb->len); /* length of IP header and IP payload */
	iph->id = htons(0x00); /* zero IP frame id */
	iph->frag_off = htons(IP_DF); /* and no more frames */
	iph->ttl = 255;
	iph->protocol = sk->protocol;
	iph->saddr = rtdev->local_ip;
	iph->daddr = rt->ip;
	iph->check = 0; /* required to compute correct checksum */
	iph->check = ip_fast_csum((u8 *)iph, 5 /*iph->ihl*/);

	ret = rtdev->hard_header(skb, rtdev, ETH_P_IP, rt->dev_addr,
				 rtdev->dev_addr, skb->len);
	rtdev_dereference(rt->rtdev);

	if (ret != rtdev->hard_header_len) {
		rtdm_printk("rttcp: rt_ip_build_frame: error on lower level\n");
		return -EINVAL;
	}

	return 0;
}

static void rt_tcp_build_header(struct tcp_socket *ts, struct rtskb *skb,
				__be32 flags, u8 is_keepalive)
{
	u32 wcheck;
	u8 tcphdrlen = 20;
	u8 iphdrlen = 20;
	struct tcphdr *th;

	th = skb->h.th;
	th->source = ts->sport;
	th->dest = ts->dport;

	th->seq = htonl(ts->sync.seq);

	if (unlikely(is_keepalive))
		th->seq--;

	tcp_flag_word(th) = flags;
	th->ack_seq = htonl(ts->sync.ack_seq);
	th->window = htons(ts->sync.window);

	th->doff = tcphdrlen >> 2; /* No options for now */
	th->res1 = 0;
	th->check = 0;
	th->urg_ptr = 0;

	/* compute checksum */
	wcheck = csum_partial(th, tcphdrlen, 0);

	if (skb->len - tcphdrlen - iphdrlen) {
		wcheck = csum_partial(skb->data + tcphdrlen + iphdrlen,
				      skb->len - tcphdrlen - iphdrlen, wcheck);
	}

	th->check =
		tcp_v4_check(skb->len - iphdrlen, ts->saddr, ts->daddr, wcheck);
}

static int rt_tcp_segment(struct dest_route *rt, struct tcp_socket *ts,
			  __be32 flags, u32 data_len, u8 *data_ptr,
			  u8 is_keepalive)
{
	struct tcphdr *th;
	struct rtsocket *sk = &ts->sock;
	struct rtnet_device *rtdev = rt->rtdev;
	struct rtskb *skb;
	struct iphdr *iph;
	struct rtskb *cloned_skb;
	rtdm_lockctx_t context;

	int ret;

	u32 hh_len = (rtdev->hard_header_len + 15) & ~15;
	u32 prio = (volatile unsigned int)sk->priority;
	u32 mtu = rtdev->get_mtu(rtdev, prio);

	u8 *data = NULL;

	if ((skb = alloc_rtskb(mtu + hh_len + 15, &sk->skb_pool)) == NULL) {
		rtdm_printk(
			"rttcp: no more elements in skb_pool for allocation\n");
		return -ENOBUFS;
	}

	/* rtskb_reserve(skb, hh_len + 20); */
	rtskb_reserve(skb, hh_len);

	iph = (struct iphdr *)rtskb_put(skb, 20); /* length of IP header */
	skb->nh.iph = iph;

	th = (struct tcphdr *)rtskb_put(skb, 20); /* length of TCP header */
	skb->h.th = th;

	if (data_len) { /* check for available place */
		data = (u8 *)rtskb_put(skb,
				       data_len); /* length of TCP payload */
		if (!memcpy(data, (void *)data_ptr, data_len)) {
			ret = -EFAULT;
			goto error;
		}
	}

	/* used local phy MTU value */
	if (data_len > mtu)
		data_len = mtu;

	skb->rtdev = rtdev;
	skb->priority = prio;

	/* do not validate socket connection on xmit
       this should be done at upper level */

	rtdm_lock_get_irqsave(&ts->socket_lock, context);
	rt_tcp_build_header(ts, skb, flags, is_keepalive);

	if ((ret = rt_ip_build_frame(skb, sk, rt, iph)) != 0) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		goto error;
	}

	/* add rtskb entry to the socket retransmission queue */
	if (ts->tcp_state != TCP_CLOSE &&
	    ((flags & (TCP_FLAG_SYN | TCP_FLAG_FIN)) || data_len)) {
		/* rtskb_clone below is called under lock, this is an admission,
	   because for now there is no rtskb copy by reference */
		cloned_skb = rtskb_clone(skb, &ts->sock.skb_pool);
		if (!cloned_skb) {
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			rtdm_printk("rttcp: cann't clone skb\n");
			ret = -ENOMEM;
			goto error;
		}

		rt_tcp_retransmit_send(ts, cloned_skb);
	}

	/* need to update sync here, because it is safe way in
       comparison with races on fast ACK response */
	if (flags & (TCP_FLAG_FIN | TCP_FLAG_SYN))
		ts->sync.seq++;

	ts->sync.seq += data_len;
	ts->sync.dst_window -= data_len;

	rtdm_lock_put_irqrestore(&ts->socket_lock, context);

	/* ignore return value from rtdev_xmit */
	/* the packet was enqueued and on error will be retransmitted later */
	/* on critical error after retransmission timeout the connection will
       be closed by connection lost */
	rtdev_xmit(skb);

	return data_len;

error:
	kfree_rtskb(skb);
	return ret;
}

static int rt_tcp_send(struct tcp_socket *ts, __be32 flags)
{
	struct dest_route rt;
	int ret;

	/*
     * We may not have a route yet during setup. But once it is set, it stays
     * until the socket died.
     */
	if (likely(ts->rt.rtdev)) {
		ret = rt_tcp_segment(&ts->rt, ts, flags, 0, NULL, 0);
	} else {
		ret = rt_ip_route_output(&rt, ts->daddr, ts->saddr);
		if (ret == 0) {
			ret = rt_tcp_segment(&rt, ts, flags, 0, NULL, 0);
			rtdev_dereference(rt.rtdev);
		}
	}
	if (ret < 0)
		rtdm_printk("rttcp: can't send a packet: err %d\n", -ret);
	return ret;
}

#ifdef YET_UNUSED
static void rt_tcp_keepalive_timer(rtdm_timer_t *timer)
{
	rtdm_lockctx_t context;
	struct tcp_keepalive *keepalive =
		container_of(timer, struct tcp_keepalive, timer);

	struct tcp_socket *ts =
		container_of(keepalive, struct tcp_socket, keepalive);
	int signal = 0;

	rtdm_lock_get_irqsave(&ts->socket_lock, context);

	if (keepalive->probes) {
		/* Send a probe */
		if (rt_tcp_segment(&ts->rt, ts, 0, 0, NULL, 1) < 0) {
			/* data receiving and sending is not possible anymore */
			signal = rt_tcp_socket_invalidate(ts, TCP_TIME_WAIT);
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		}

		keepalive->probes--;
		rtdm_timer_start_in_handler(&keepalive->timer,
					    rt_tcp_keepalive_intvl, 0,
					    RTDM_TIMERMODE_RELATIVE);
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
	} else {
		/* data receiving and sending is not possible anymore */

		signal = rt_tcp_socket_invalidate(ts, TCP_TIME_WAIT);
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
	}

	if (signal)
		rt_tcp_socket_invalidate_signal(ts);
}
#endif

static inline u32 rt_tcp_initial_seq(void)
{
	uint64_t clock_val = rtdm_clock_read_monotonic();
	return (u32)(clock_val ^ (clock_val >> 32));
}

/***
 *  rt_tcp_dest_socket
 */
static struct rtsocket *rt_tcp_dest_socket(struct rtskb *skb)
{
	struct tcphdr *th = skb->h.th;

	u32 saddr = skb->nh.iph->saddr;
	u32 daddr = skb->nh.iph->daddr;
	u32 sport = th->source;
	u32 dport = th->dest;

	u32 data_len;

	if (tcp_v4_check(skb->len, saddr, daddr,
			 csum_partial(skb->data, skb->len, 0))) {
		rtdm_printk("rttcp: invalid TCP packet checksum, dropped\n");
		return NULL; /* Invalid checksum, drop the packet */
	}

	/* find the destination socket */
	if ((skb->sk = rt_tcp_v4_lookup(daddr, dport)) == NULL) {
		/*
	  rtdm_printk("Not found addr:0x%08x, port: 0x%04x\n", daddr, dport);
	*/
		if (!th->rst) {
			/* No listening socket found, send RST|ACK */
			rst_socket.saddr = daddr;
			rst_socket.daddr = saddr;
			rst_socket.sport = dport;
			rst_socket.dport = sport;

			data_len = skb->len - (th->doff << 2);

			rst_socket.sync.seq = 0;
			rst_socket.sync.ack_seq =
				rt_tcp_compute_ack_seq(th, data_len);

			if (rt_ip_route_output(&rst_socket.rt, daddr, saddr) ==
			    0) {
				rt_socket_reference(&rst_socket.sock);
				rt_tcp_send(&rst_socket,
					    TCP_FLAG_ACK | TCP_FLAG_RST);
				rtdev_dereference(rst_socket.rt.rtdev);
			}
		}
	}

	return skb->sk;
}

static void rt_tcp_window_update(struct tcp_socket *ts, u16 window)
{
	rtdm_lockctx_t context;

	rtdm_lock_get_irqsave(&ts->socket_lock, context);

	if (ts->sync.dst_window) {
		ts->sync.dst_window = window;
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		if (!window) {
			/* clear send event status */
			rtdm_event_clear(&ts->send_evt);
		}
	} else {
		if (window) {
			ts->sync.dst_window = window;
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			/* set send event status */
			rtdm_event_signal(&ts->send_evt);
		} else {
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		}
	}
}

/***
 *  rt_tcp_rcv
 */
static void rt_tcp_rcv(struct rtskb *skb)
{
	rtdm_lockctx_t context;
	struct tcp_socket *ts;
	struct tcphdr *th = skb->h.th;
	unsigned int data_len = skb->len - (th->doff << 2);
	u32 seq = ntohl(th->seq);
	int signal;

	ts = container_of(skb->sk, struct tcp_socket, sock);

	rtdm_lock_get_irqsave(&ts->socket_lock, context);

#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_TCP_ERROR_INJECTION
	if (ts->error_rate > 0) {
		if ((ts->packet_counter++ % error_rate) < ts->multi_error) {
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			goto drop;
		}
	}
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_TCP_ERROR_INJECTION */

	/* Check for daddr/dport correspondence to values stored in
       selected socket from hash */
	if (ts->tcp_state != TCP_LISTEN && (ts->daddr != skb->nh.iph->saddr ||
					    ts->dport != skb->h.th->source)) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		goto drop;
	}

	/* Check if it is a keepalive probe */
	if (ts->sync.ack_seq == (seq + 1) && ts->tcp_state == TCP_ESTABLISHED) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		rt_tcp_send(ts, TCP_FLAG_ACK);
		goto feed;
	}

	if (ts->tcp_state == TCP_SYN_SENT) {
		ts->sync.ack_seq = rt_tcp_compute_ack_seq(th, data_len);

		if (th->syn && th->ack) {
			rt_tcp_socket_validate(ts);
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			rtdm_event_signal(&ts->conn_evt);
			/* Send ACK */
			rt_tcp_send(ts, TCP_FLAG_ACK);
			goto feed;
		}

		ts->tcp_state = TCP_CLOSE;
		ts->sync.seq = ntohl(th->ack_seq);
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);

		/* Send RST|ACK */
		rtdm_event_signal(&ts->conn_evt);
		rt_tcp_send(ts, TCP_FLAG_RST | TCP_FLAG_ACK);
		goto drop;
	}

	/* Check for SEQ correspondence to determine the connection relevance */

	/* OR-list of conditions to be satisfied:
     *
     * th->ack && rt_tcp_after(ts->nacked_first, ntohl(th->ack_seq))
     * th->ack && th->rst && ...
     * th->syn && (ts->tcp_state == TCP_LISTEN ||
		   ts->tcp_state == TCP_SYN_SENT)
     * rt_tcp_after(seq, ts->sync.ack_seq) &&
	   rt_tcp_before(seq, ts->sync.ack_seq + ts->sync.window)
     */

	if ((rt_tcp_after(seq, ts->sync.ack_seq) &&
	     rt_tcp_before(seq, ts->sync.ack_seq + ts->sync.window)) ||
	    th->rst ||
	    (th->syn &&
	     (ts->tcp_state == TCP_LISTEN || ts->tcp_state == TCP_SYN_SENT))) {
		/* everything is ok */
	} else if (rt_tcp_after(seq, ts->sync.ack_seq - data_len)) {
		/* retransmission of data we already acked */
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		rt_tcp_send(ts, TCP_FLAG_ACK);
		goto drop;
	} else {
		/* drop forward ack */
		if (th->ack &&
		    /* but reset ack from old connection */
		    ts->tcp_state == TCP_ESTABLISHED) {
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			rtdm_printk(
				"rttcp: dropped unappropriate ACK packet %u\n",
				ts->sync.ack_seq);
			goto drop;
		}

		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		rtdm_printk("rttcp: sequence number is not in window, "
			    "dropped (failed: %u <= %u <= %u)\n",
			    ts->sync.ack_seq, seq,
			    ts->sync.ack_seq + ts->sync.window);

		/* That's a forced RST for a lost connection */
		rst_socket.saddr = skb->nh.iph->daddr;
		rst_socket.daddr = skb->nh.iph->saddr;
		rst_socket.sport = th->dest;
		rst_socket.dport = th->source;

		rst_socket.sync.seq = ntohl(th->ack_seq);
		rst_socket.sync.ack_seq = rt_tcp_compute_ack_seq(th, data_len);

		if (rt_ip_route_output(&rst_socket.rt, rst_socket.daddr,
				       rst_socket.saddr) == 0) {
			rt_socket_reference(&rst_socket.sock);
			rt_tcp_send(&rst_socket, TCP_FLAG_RST | TCP_FLAG_ACK);
			rtdev_dereference(rst_socket.rt.rtdev);
		}
		goto drop;
	}

	if (th->rst) {
		if (ts->tcp_state == TCP_SYN_RECV) {
			ts->tcp_state = TCP_LISTEN;
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			goto drop;
		} else {
			/* Drop our half-open connection, peer obviously went away. */
			signal = rt_tcp_socket_invalidate(ts, TCP_CLOSE);
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);

			if (signal)
				rt_tcp_socket_invalidate_signal(ts);

			goto drop;
		}
	}

	ts->sync.ack_seq = rt_tcp_compute_ack_seq(th, data_len);

	if (th->fin) {
		if (ts->tcp_state == TCP_ESTABLISHED) {
			/* Send ACK */
			signal = rt_tcp_socket_invalidate(ts, TCP_CLOSE_WAIT);
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);

			if (signal)
				rt_tcp_socket_invalidate_signal(ts);

			rt_tcp_send(ts, TCP_FLAG_ACK);
			goto feed;
		} else if ((ts->tcp_state == TCP_FIN_WAIT1 && th->ack) ||
			   ts->tcp_state == TCP_FIN_WAIT2) {
			/* Send ACK */
			ts->tcp_state = TCP_TIME_WAIT;
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			rt_tcp_send(ts, TCP_FLAG_ACK);
			/* data receiving is not possible anymore */
			rtdm_sem_destroy(&ts->sock.pending_sem);
			rtdm_nrtsig_pend(&ts->close_sig);
			goto feed;
		} else if (ts->tcp_state == TCP_FIN_WAIT1) {
			/* Send ACK */
			ts->tcp_state = TCP_CLOSING;
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			rt_tcp_send(ts, TCP_FLAG_ACK);
			/* data receiving is not possible anymore */
			rtdm_sem_destroy(&ts->sock.pending_sem);
			goto feed;
		} else {
			/* just drop it */
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			goto drop;
		}
	}

	if (th->syn) {
		/* Need to differentiate LISTEN socket from ESTABLISHED one */
		/* Both of them have the same sport/saddr, but different dport/daddr */
		/* dport is unknown if it is the first connection of n */

		if (ts->tcp_state == TCP_LISTEN) {
			/* Need to store ts->seq while sending SYN earlier */
			/* The socket shall be in TCP_LISTEN state */

			/* safe to update ts->saddr here due to a single task for
	       rt_tcp_rcv() and rt_tcp_dest_socket() callers */
			ts->saddr = skb->nh.iph->daddr;

			ts->daddr = skb->nh.iph->saddr;
			ts->dport = th->source;
			ts->sync.seq = rt_tcp_initial_seq();
			ts->sync.window = 4096;
			ts->tcp_state = TCP_SYN_RECV;
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);

			/* Send SYN|ACK */
			rt_tcp_send(ts, TCP_FLAG_SYN | TCP_FLAG_ACK);
			goto drop;
		}

		/* Send RST|ACK */
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		rt_tcp_send(ts, TCP_FLAG_RST | TCP_FLAG_ACK);
		goto drop;
	}

	/* ACK received without SYN, FIN or RST flags */
	if (th->ack) {
		/* Check ack sequence */
		if (rt_tcp_before(ts->sync.seq + 1, ntohl(th->ack_seq))) {
			rtdm_printk("rttcp: unexpected ACK %u %u %u\n",
				    ts->sync.seq, ts->nacked_first,
				    ntohl(th->ack_seq));
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			goto drop;
		}

		if (ts->tcp_state == TCP_LAST_ACK) {
			/* close connection and free socket data */
			ts->tcp_state = TCP_CLOSE;
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			/* socket destruction will be done on close() */
			rtdm_nrtsig_pend(&ts->close_sig);
			goto drop;
		} else if (ts->tcp_state == TCP_FIN_WAIT1) {
			ts->tcp_state = TCP_FIN_WAIT2;
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			goto feed;
		} else if (ts->tcp_state == TCP_SYN_RECV) {
			rt_tcp_socket_validate(ts);
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			rtdm_event_signal(&ts->conn_evt);
			goto feed;
		} else if (ts->tcp_state == TCP_CLOSING) {
			ts->tcp_state = TCP_TIME_WAIT;
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			/* socket destruction will be done on close() */
			rtdm_nrtsig_pend(&ts->close_sig);
			goto feed;
		}
	}

	if (ts->tcp_state != TCP_ESTABLISHED) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		goto drop;
	}

	if (data_len == 0) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		goto feed;
	}

	/* Send ACK */
	ts->sync.window -= data_len;
	rtdm_lock_put_irqrestore(&ts->socket_lock, context);
	rt_tcp_send(ts, TCP_FLAG_ACK);

	rtskb_queue_tail(&skb->sk->incoming, skb);
	rtdm_sem_up(&ts->sock.pending_sem);

	/* inform retransmission subsystem about arrived ack */
	if (th->ack) {
		rt_tcp_retransmit_ack(ts, ntohl(th->ack_seq));
	}

	rt_tcp_keepalive_feed(ts);
	rt_tcp_window_update(ts, ntohs(th->window));

	return;

feed:
	/* inform retransmission subsystem about arrived ack */
	if (th->ack) {
		rt_tcp_retransmit_ack(ts, ntohl(th->ack_seq));
	}

	rt_tcp_keepalive_feed(ts);
	rt_tcp_window_update(ts, ntohs(th->window));

drop:
	kfree_rtskb(skb);
	return;
}

/***
 *  rt_tcp_rcv_err
 */
static void rt_tcp_rcv_err(struct rtskb *skb)
{
	rtdm_printk("rttcp: rt_tcp_rcv err\n");
}

static int rt_tcp_window_send(struct tcp_socket *ts, u32 data_len, u8 *data_ptr)
{
	u32 dst_window = ts->sync.dst_window;
	int ret;

	if (data_len > dst_window)
		data_len = dst_window;

	if ((ret = rt_tcp_segment(&ts->rt, ts, TCP_FLAG_ACK, data_len, data_ptr,
				  0)) < 0) {
		rtdm_printk("rttcp: cann't send a packet: err %d\n", -ret);
		return ret;
	}

	return ret;
}

static void rt_tcp_close_signal_handler(rtdm_nrtsig_t *nrtsig, void *arg)
{
	complete_all((struct completion *)arg);
}

static int rt_tcp_socket_create(struct tcp_socket *ts)
{
	rtdm_lockctx_t context;
	int i;
	int index;
	struct rtsocket *sock = &ts->sock;

	sock->prot.inet.saddr = INADDR_ANY;
	sock->prot.inet.state = TCP_CLOSE;
	sock->prot.inet.tos = 0;
	/*
      rtdm_printk("rttcp: rt_tcp_socket_create 0x%p\n", ts);
    */
	rtdm_lock_init(&ts->socket_lock);

	ts->rt.rtdev = NULL;

	ts->tcp_state = TCP_CLOSE;

	ts->is_accepting = 0;
	ts->is_accepted = 0;
	ts->is_binding = 0;
	ts->is_bound = 0;
	ts->is_valid = 0;
	ts->is_closed = 0;

	ts->sk_sndtimeo = RTDM_TIMEOUT_INFINITE;

	rtdm_event_init(&ts->conn_evt, 0);

	ts->keepalive.enabled = 0;

	ts->timer_state = max_retransmits;
	timerwheel_init_timer(&ts->timer, rt_tcp_retransmit_handler, ts);
	rtskb_queue_init(&ts->retransmit_queue);

	init_completion(&ts->fin_handshake);
	rtdm_nrtsig_init(&ts->close_sig, rt_tcp_close_signal_handler,
			 &ts->fin_handshake);

#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_TCP_ERROR_INJECTION
	ts->packet_counter = counter_start;
	ts->error_rate = error_rate;
	ts->multi_error = multi_error;
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_TCP_ERROR_INJECTION */

	rtdm_lock_get_irqsave(&tcp_socket_base_lock, context);

	/* enforce maximum number of TCP sockets */
	if (free_ports == 0) {
		rtdm_lock_put_irqrestore(&tcp_socket_base_lock, context);
		rtdm_nrtsig_destroy(&ts->close_sig);
		return -EAGAIN;
	}
	free_ports--;

	/* find free auto-port in bitmap */
	for (i = 0; i < RT_PORT_BITMAP_WORDS; i++)
		if (port_bitmap[i] != (unsigned long)-1)
			break;
	index = ffz(port_bitmap[i]);
	set_bit(index, &port_bitmap[i]);
	index += i * 32;
	sock->prot.inet.reg_index = index;
	sock->prot.inet.sport = index + tcp_auto_port_start;

	/* register TCP socket */
	port_registry[index] = ts;
	port_hash_insert(ts, INADDR_ANY, sock->prot.inet.sport);

	rtdm_lock_put_irqrestore(&tcp_socket_base_lock, context);

	return 0;
}

/***
 *  rt_tcp_socket - create a new TCP-Socket
 *  @s: socket
 */
static int rt_tcp_socket(struct rtdm_fd *fd)
{
	struct tcp_socket *ts = rtdm_fd_to_private(fd);
	int ret;

	if ((ret = rt_socket_init(fd, IPPROTO_TCP)) != 0)
		return ret;

	if ((ret = rt_tcp_socket_create(ts)) != 0)
		rt_socket_cleanup(fd);

	return ret;
}

static int rt_tcp_dispatched_packet_send(struct rt_proc_call *call)
{
	int ret;
	struct rt_tcp_dispatched_packet_send_cmd *cmd;

	cmd = rtpc_get_priv(call, struct rt_tcp_dispatched_packet_send_cmd);
	ret = rt_tcp_send(cmd->ts, cmd->flags);

	return ret;
}

/***
 *  rt_tcp_socket_destruct
 *  this function requires non realtime context
 */
static void rt_tcp_socket_destruct(struct tcp_socket *ts)
{
	rtdm_lockctx_t context;
	struct rtskb *skb;
	int index;
	int signal;
	struct rtsocket *sock = &ts->sock;

	/*
      rtdm_printk("rttcp: rt_tcp_socket_destruct 0x%p\n", ts);
    */

	rtdm_lock_get_irqsave(&tcp_socket_base_lock, context);
	if (sock->prot.inet.reg_index >= 0) {
		index = sock->prot.inet.reg_index;

		clear_bit(index % BITS_PER_LONG,
			  &port_bitmap[index / BITS_PER_LONG]);
		port_hash_del(port_registry[index]);
		free_ports++;
		sock->prot.inet.reg_index = -1;
	}
	rtdm_lock_put_irqrestore(&tcp_socket_base_lock, context);

	rtdm_lock_get_irqsave(&ts->socket_lock, context);

	signal = rt_tcp_socket_invalidate(ts, TCP_CLOSE);

	rt_tcp_keepalive_disable(ts);

	sock->prot.inet.state = TCP_CLOSE;

	/* dereference rtdev */
	if (ts->rt.rtdev != NULL) {
		rtdev_dereference(ts->rt.rtdev);
		ts->rt.rtdev = NULL;
	}

	rtdm_lock_put_irqrestore(&ts->socket_lock, context);

	if (signal)
		rt_tcp_socket_invalidate_signal(ts);

	rtdm_event_destroy(&ts->conn_evt);

	rtdm_nrtsig_destroy(&ts->close_sig);

	/* cleanup already collected fragments */
	rt_ip_frag_invalidate_socket(sock);

	/* free packets in incoming queue */
	while ((skb = rtskb_dequeue(&sock->incoming)) != NULL)
		kfree_rtskb(skb);

	/* ensure that the timer is no longer running */
	timerwheel_remove_timer_sync(&ts->timer);

	/* free packets in retransmission queue */
	while ((skb = __rtskb_dequeue(&ts->retransmit_queue)) != NULL)
		kfree_rtskb(skb);
}

/***
 *  rt_tcp_close
 */
static void rt_tcp_close(struct rtdm_fd *fd)
{
	struct tcp_socket *ts = rtdm_fd_to_private(fd);
	struct rt_tcp_dispatched_packet_send_cmd send_cmd;
	rtdm_lockctx_t context;
	int signal = 0;

	rtdm_lock_get_irqsave(&ts->socket_lock, context);

	ts->is_closed = 1;

	if (ts->tcp_state == TCP_ESTABLISHED || ts->tcp_state == TCP_SYN_RECV) {
		/* close() from ESTABLISHED */
		send_cmd.ts = ts;
		send_cmd.flags = TCP_FLAG_FIN | TCP_FLAG_ACK;
		signal = rt_tcp_socket_invalidate(ts, TCP_FIN_WAIT1);

		rtdm_lock_put_irqrestore(&ts->socket_lock, context);

		rtpc_dispatch_call(rt_tcp_dispatched_packet_send, 0, &send_cmd,
				   sizeof(send_cmd), NULL, NULL);
		/* result is ignored */

		/* Give the peer some time to reply to our FIN.
		   Since it is not relevant what exactly causes the wait
		   function to return its result is ignored. */
		wait_for_completion_interruptible_timeout(&ts->fin_handshake,
					      msecs_to_jiffies(close_timeout));
	} else if (ts->tcp_state == TCP_CLOSE_WAIT) {
		/* Send FIN in CLOSE_WAIT */
		send_cmd.ts = ts;
		send_cmd.flags = TCP_FLAG_FIN | TCP_FLAG_ACK;
		signal = rt_tcp_socket_invalidate(ts, TCP_LAST_ACK);

		rtdm_lock_put_irqrestore(&ts->socket_lock, context);

		rtpc_dispatch_call(rt_tcp_dispatched_packet_send, 0, &send_cmd,
				   sizeof(send_cmd), NULL, NULL);
		/* result is ignored */

		/* Give the peer some time to reply to our FIN.
		   Since it is not relevant what exactly causes the wait
		   function to return its result is ignored. */
		wait_for_completion_interruptible_timeout(&ts->fin_handshake,
					      msecs_to_jiffies(close_timeout));
	} else {
		/*
	  rt_tcp_socket_validate() has not been called at all,
	  hence socket state is TCP_SYN_SENT or TCP_LISTEN,
	  or socket is in one of close states,
	  hence rt_tcp_socket_invalidate() was called,
	  but close() is called at first time
	*/
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
	}

	if (signal)
		rt_tcp_socket_invalidate_signal(ts);

	rt_tcp_socket_destruct(ts);

	rt_socket_cleanup(fd);
}

/***
 *  rt_tcp_bind - bind socket to local address
 *  @s:     socket
 *  @addr:  local address
 */
static int rt_tcp_bind(struct rtdm_fd *fd, struct tcp_socket *ts,
		       const struct sockaddr __user *addr, socklen_t addrlen)
{
	struct sockaddr_in *usin, _usin;
	rtdm_lockctx_t context;
	int index;
	int bound = 0;
	int ret = 0;

	usin = rtnet_get_arg(fd, &_usin, addr, sizeof(_usin));
	if (IS_ERR(usin))
		return PTR_ERR(usin);

	if ((addrlen < (int)sizeof(struct sockaddr_in)) ||
	    ((usin->sin_port & tcp_auto_port_mask) == tcp_auto_port_start))
		return -EINVAL;

	rtdm_lock_get_irqsave(&ts->socket_lock, context);
	if (ts->tcp_state != TCP_CLOSE || ts->is_bound || ts->is_binding) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return -EINVAL;
	}

	ts->is_binding = 1;
	rtdm_lock_put_irqrestore(&ts->socket_lock, context);

	rtdm_lock_get_irqsave(&tcp_socket_base_lock, context);

	if ((index = ts->sock.prot.inet.reg_index) < 0) {
		/* socket is destroyed */
		ret = -EBADF;
		goto unlock_out;
	}

	port_hash_del(ts);
	if (port_hash_insert(ts, usin->sin_addr.s_addr,
			     usin->sin_port ?: index + tcp_auto_port_start)) {
		port_hash_insert(ts, ts->saddr, ts->sport);

		ret = -EADDRINUSE;
		goto unlock_out;
	}

	bound = 1;

unlock_out:
	rtdm_lock_put_irqrestore(&tcp_socket_base_lock, context);

	rtdm_lock_get_irqsave(&ts->socket_lock, context);
	ts->is_bound = bound;
	ts->is_binding = 0;
	rtdm_lock_put_irqrestore(&ts->socket_lock, context);

	return ret;
}

/***
 *  rt_tcp_connect
 */
static int rt_tcp_connect(struct rtdm_fd *fd, struct tcp_socket *ts,
			  const struct sockaddr __user *serv_addr,
			  socklen_t addrlen)
{
	struct sockaddr_in *usin, _usin;
	struct dest_route rt;
	rtdm_lockctx_t context;
	int ret;

	if (addrlen < (int)sizeof(struct sockaddr_in))
		return -EINVAL;

	usin = rtnet_get_arg(fd, &_usin, serv_addr, sizeof(_usin));
	if (IS_ERR(usin))
		return PTR_ERR(usin);

	if (usin->sin_family != AF_INET)
		return -EAFNOSUPPORT;

	ret = rt_ip_route_output(&rt, usin->sin_addr.s_addr, ts->saddr);
	if (ret < 0) {
		/* no route to host */
		return -ENETUNREACH;
	}

	rtdm_lock_get_irqsave(&ts->socket_lock, context);

	if (ts->is_closed) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		ret = -EBADF;
		goto err_deref;
	}

	if (ts->tcp_state != TCP_CLOSE || ts->is_binding) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		ret = -EINVAL;
		goto err_deref;
	}

	if (ts->rt.rtdev == NULL)
		memcpy(&ts->rt, &rt, sizeof(rt));
	else
		rtdev_dereference(rt.rtdev);

	ts->saddr = rt.rtdev->local_ip;

	ts->daddr = usin->sin_addr.s_addr;
	ts->dport = usin->sin_port;

	ts->sync.seq = rt_tcp_initial_seq();
	ts->sync.ack_seq = 0;
	ts->sync.window = 4096;
	ts->sync.dst_window = 0;

	ts->tcp_state = TCP_SYN_SENT;

	rtdm_lock_put_irqrestore(&ts->socket_lock, context);

	/* Complete three-way handshake */
	ret = rt_tcp_send(ts, TCP_FLAG_SYN);
	if (ret < 0) {
		rtdm_printk("rttcp: cann't send SYN\n");
		return ret;
	}

	ret = rtdm_event_timedwait(&ts->conn_evt, rt_tcp_connection_timeout,
				   NULL);
	if (unlikely(ret < 0))
		switch (ret) {
		case -EWOULDBLOCK:
		case -ETIMEDOUT:
		case -EINTR:
			return ret;

		default:
			return -EBADF;
		}

	if (ts->tcp_state == TCP_SYN_SENT) {
		/* received conn_evt, but connection is not established */
		return -ECONNREFUSED;
	}

	return ret;

err_deref:
	rtdev_dereference(rt.rtdev);

	return ret;
}

/***
 *  rt_tcp_listen
 */
static int rt_tcp_listen(struct tcp_socket *ts, unsigned long backlog)
{
	int ret;
	rtdm_lockctx_t context;

	/* Ignore backlog value, maximum number of queued connections is 1 */

	rtdm_lock_get_irqsave(&ts->socket_lock, context);
	if (ts->is_closed) {
		ret = -EBADF;
		goto unlock_out;
	}

	if (ts->tcp_state != TCP_CLOSE || ts->is_binding) {
		ret = -EINVAL;
		goto unlock_out;
	}

	ts->tcp_state = TCP_LISTEN;
	ret = 0;

unlock_out:
	rtdm_lock_put_irqrestore(&ts->socket_lock, context);

	return ret;
}

/***
 *  rt_tcp_accept
 */
static int rt_tcp_accept(struct rtdm_fd *fd, struct tcp_socket *ts,
			 struct sockaddr *addr, socklen_t __user *addrlen)
{
	/* Return sockaddr, but bind it with rt_socket_init, so it would be
       possible to read/write from it in future, return valid file descriptor */

	int ret;
	socklen_t *uaddrlen, _uaddrlen;
	struct sockaddr_in sin;
	nanosecs_rel_t timeout = ts->sock.timeout;
	rtdm_lockctx_t context;
	struct dest_route rt;

	uaddrlen = rtnet_get_arg(fd, &_uaddrlen, addrlen, sizeof(_uaddrlen));
	if (IS_ERR(uaddrlen))
		return PTR_ERR(uaddrlen);

	rtdm_lock_get_irqsave(&ts->socket_lock, context);
	if (ts->is_accepting || ts->is_accepted) {
		/* socket is already accepted or is accepting a connection right now */
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return -EALREADY;
	}

	if (ts->tcp_state != TCP_LISTEN ||
	    *uaddrlen < sizeof(struct sockaddr_in)) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return -EINVAL;
	}

	ts->is_accepting = 1;
	rtdm_lock_put_irqrestore(&ts->socket_lock, context);

	ret = rtdm_event_timedwait(&ts->conn_evt, timeout, NULL);

	if (unlikely(ret < 0))
		switch (ret) {
		case -ETIMEDOUT:
		case -EINTR:
			goto err;

		default:
			ret = -EBADF;
			goto err;
		}

	/* accept() reported about connection establishment */
	ret = rt_ip_route_output(&rt, ts->daddr, ts->saddr);
	if (ret < 0) {
		/* strange, no route to host, keep status quo */
		ret = -EPROTO;
		goto err;
	}

	if (addr) {
		sin.sin_family = AF_INET;
		sin.sin_port = ts->dport;
		sin.sin_addr.s_addr = ts->daddr;
		ret = rtnet_put_arg(fd, addr, &sin, sizeof(sin));
		if (ret) {
			rtdev_dereference(rt.rtdev);
			ret = -EFAULT;
			goto err;
		}
	}

	rtdm_lock_get_irqsave(&ts->socket_lock, context);

	if (ts->tcp_state != TCP_ESTABLISHED) {
		/* protocol error */
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		rtdev_dereference(rt.rtdev);
		ret = -EPROTO;
		goto err;
	}

	if (ts->rt.rtdev == NULL)
		memcpy(&ts->rt, &rt, sizeof(rt));
	else
		rtdev_dereference(rt.rtdev);

	ts->is_accepted = 1;
	rtdm_lock_put_irqrestore(&ts->socket_lock, context);

	ret = rtdm_fd_ufd(rt_socket_fd(&ts->sock));

err:
	/* it is not critical to leave this unlocked
       due to single entry nature of accept() */
	ts->is_accepting = 0;

	return ret;
}

/***
 *  rt_tcp_shutdown
 */
static int rt_tcp_shutdown(struct tcp_socket *ts, unsigned long how)
{
	return -EOPNOTSUPP;
}

/***
 *  rt_tcp_setsockopt
 */
static int rt_tcp_setsockopt(struct rtdm_fd *fd, struct tcp_socket *ts,
			     int level, int optname, const void *optval,
			     socklen_t optlen)
{
	/* uint64_t val; */
	struct timeval tv;
	rtdm_lockctx_t context;

	switch (optname) {
	case SO_KEEPALIVE:
		if (optlen < sizeof(unsigned int))
			return -EINVAL;

		/* commented out, because current implementation transmits
	       keepalive probes from interrupt context */
		/*
	    val = *(unsigned long*)optval;

	    if (val)
		rt_tcp_keepalive_enable(ts);
	    else
		rt_tcp_keepalive_disable(ts);
	    */
		return 0;

	case SO_SNDTIMEO_OLD:
		if (optlen < sizeof(tv))
			return -EINVAL;
		if (rtdm_copy_from_user(fd, &tv, optval, sizeof(tv)))
			return -EFAULT;
		if (tv.tv_usec < 0 || tv.tv_usec >= 1000000)
			return -EDOM;

		rtdm_lock_get_irqsave(&ts->socket_lock, context);

		if (tv.tv_sec < 0) {
			ts->sk_sndtimeo = RTDM_TIMEOUT_NONE;
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			return 0;
		}

		ts->sk_sndtimeo = RTDM_TIMEOUT_INFINITE;
		if (tv.tv_sec == 0 && tv.tv_usec == 0) {
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			return 0;
		}

		if (tv.tv_sec < (MAX_SCHEDULE_TIMEOUT / 1000000000ull - 1))
			ts->sk_sndtimeo =
				(tv.tv_sec * 1000000 + tv.tv_usec) * 1000;

		rtdm_lock_put_irqrestore(&ts->socket_lock, context);

		return 0;

	case SO_REUSEADDR:
		/* to implement */
		return -EOPNOTSUPP;
	}

	return -ENOPROTOOPT;
}

/***
 *  rt_tcp_getsockopt
 */
static int rt_tcp_getsockopt(struct rtdm_fd *fd, struct tcp_socket *ts,
			     int level, int optname, void *optval,
			     socklen_t *optlen)
{
	int ret = 0;

	if (*optlen < sizeof(unsigned int))
		return -EINVAL;

	switch (optname) {
	case SO_ERROR:
		ret = 0; /* used in nonblocking connect(), extend later */
		break;

	default:
		ret = -ENOPROTOOPT;
		break;
	}

	return ret;
}

/***
 *  rt_tcp_ioctl
 */
static int rt_tcp_ioctl(struct rtdm_fd *fd, unsigned int request,
			void __user *arg)
{
	struct tcp_socket *ts = rtdm_fd_to_private(fd);
	const struct _rtdm_setsockaddr_args *setaddr;
	struct _rtdm_setsockaddr_args _setaddr;
	const struct _rtdm_getsockaddr_args *getaddr;
	struct _rtdm_getsockaddr_args _getaddr;
	const struct _rtdm_getsockopt_args *getopt;
	struct _rtdm_getsockopt_args _getopt;
	const struct _rtdm_setsockopt_args *setopt;
	struct _rtdm_setsockopt_args _setopt;
	int in_rt;

	/* fast path for common socket IOCTLs */
	if (_IOC_TYPE(request) == RTIOC_TYPE_NETWORK)
		return rt_socket_common_ioctl(fd, request, arg);

	in_rt = rtdm_in_rt_context();

	switch (request) {
	case _RTIOC_BIND:
		setaddr = rtnet_get_arg(fd, &_setaddr, arg, sizeof(_setaddr));
		if (IS_ERR(setaddr))
			return PTR_ERR(setaddr);
		return rt_tcp_bind(fd, ts, setaddr->addr, setaddr->addrlen);
	case _RTIOC_CONNECT:
		if (!in_rt)
			return -ENOSYS;
		setaddr = rtnet_get_arg(fd, &_setaddr, arg, sizeof(_setaddr));
		if (IS_ERR(setaddr))
			return PTR_ERR(setaddr);
		return rt_tcp_connect(fd, ts, setaddr->addr, setaddr->addrlen);

	case _RTIOC_LISTEN:
		return rt_tcp_listen(ts, (unsigned long)arg);

	case _RTIOC_ACCEPT:
		if (!in_rt)
			return -ENOSYS;
		getaddr = rtnet_get_arg(fd, &_getaddr, arg, sizeof(_getaddr));
		if (IS_ERR(getaddr))
			return PTR_ERR(getaddr);
		return rt_tcp_accept(fd, ts, getaddr->addr, getaddr->addrlen);

	case _RTIOC_SHUTDOWN:
		return rt_tcp_shutdown(ts, (unsigned long)arg);

	case _RTIOC_SETSOCKOPT:
		setopt = rtnet_get_arg(fd, &_setopt, arg, sizeof(_setopt));
		if (IS_ERR(setopt))
			return PTR_ERR(setopt);

		if (setopt->level != SOL_SOCKET)
			break;

		return rt_tcp_setsockopt(fd, ts, setopt->level, setopt->optname,
					 setopt->optval, setopt->optlen);

	case _RTIOC_GETSOCKOPT:
		getopt = rtnet_get_arg(fd, &_getopt, arg, sizeof(_getopt));
		if (IS_ERR(getopt))
			return PTR_ERR(getopt);

		if (getopt->level != SOL_SOCKET)
			break;

		return rt_tcp_getsockopt(fd, ts, getopt->level, getopt->optname,
					 getopt->optval, getopt->optlen);
	default:
		break;
	}

	return rt_ip_ioctl(fd, request, arg);
}

/***
 *  rt_tcp_read
 */
static ssize_t rt_tcp_read(struct rtdm_fd *fd, void *buf, size_t nbyte)
{
	struct tcp_socket *ts = rtdm_fd_to_private(fd);
	struct rtsocket *sock = &ts->sock;

	struct rtskb *skb;
	struct rtskb *first_skb;
	nanosecs_rel_t timeout = sock->timeout;
	size_t data_len;
	size_t th_len;
	size_t copied = 0;
	size_t block_size;
	u8 *user_buf = buf;
	int ret;
	rtdm_lockctx_t context;

	rtdm_toseq_t timeout_seq;

	if (!rtdm_fd_is_user(fd)) {
		return -EFAULT;
	}

	rtdm_lock_get_irqsave(&ts->socket_lock, context);

	if (ts->is_closed) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return -EBADF;
	}

	if (!ts->is_valid) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return 0;
	}

	if (ts->tcp_state != TCP_ESTABLISHED &&
	    ts->tcp_state != TCP_FIN_WAIT2) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return -EINVAL;
	}
	rtdm_lock_put_irqrestore(&ts->socket_lock, context);

	rtdm_toseq_init(&timeout_seq, timeout);

	while (copied < nbyte) {
		ret = rtdm_sem_timeddown(&ts->sock.pending_sem, timeout,
					 &timeout_seq);

		if (unlikely(ret < 0))
			switch (ret) {
			case -EWOULDBLOCK:
			case -ETIMEDOUT:
			case -EINTR:
				return (copied ? copied : ret);

			case -EIDRM: /* event is destroyed */
				if (ts->is_closed)
					return -EBADF;

				return copied;

			default:
				if (ts->is_closed) {
					return -EBADF;
				}

				return 0;
			}

		skb = rtskb_dequeue_chain(&sock->incoming);
		RTNET_ASSERT(skb != NULL, return -EFAULT;);

		th_len = (skb->h.th->doff) << 2;

		data_len = skb->len - th_len;

		__rtskb_pull(skb, th_len);

		first_skb = skb;

		/* iterate over all IP fragments */
	iterate_fragments:
		block_size = skb->len;
		copied += block_size;
		data_len -= block_size;

		if (copied > nbyte) {
			block_size -= copied - nbyte;
			copied = nbyte;

			if (rtdm_copy_to_user(fd, user_buf, skb->data,
					      block_size)) {
				kfree_rtskb(first_skb); /* or store the data? */
				return -EFAULT;
			}
			rtdm_lock_get_irqsave(&ts->socket_lock, context);
			if (ts->sync.window) {
				ts->sync.window += block_size;
				rtdm_lock_put_irqrestore(&ts->socket_lock,
							 context);
			} else {
				ts->sync.window = block_size;
				rtdm_lock_put_irqrestore(&ts->socket_lock,
							 context);
				rt_tcp_send(ts,
					    TCP_FLAG_ACK); /* window update */
			}

			__rtskb_pull(skb, block_size);
			__rtskb_push(first_skb, sizeof(struct tcphdr));
			first_skb->h.th->doff = 5;
			rtskb_queue_head(&sock->incoming, first_skb);
			rtdm_sem_up(&ts->sock.pending_sem);

			return copied;
		}

		if (rtdm_copy_to_user(fd, user_buf, skb->data, block_size)) {
			kfree_rtskb(first_skb); /* or store the data? */
			return -EFAULT;
		}
		rtdm_lock_get_irqsave(&ts->socket_lock, context);
		if (ts->sync.window) {
			ts->sync.window += block_size;
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		} else {
			ts->sync.window = block_size;
			rtdm_lock_put_irqrestore(&ts->socket_lock, context);
			rt_tcp_send(ts, TCP_FLAG_ACK); /* window update */
		}

		if ((skb = skb->next) != NULL) {
			user_buf += data_len;
			goto iterate_fragments;
		}

		kfree_rtskb(first_skb);
	}

	return copied;
}

/***
 *  rt_tcp_write
 */
static ssize_t rt_tcp_write(struct rtdm_fd *fd, const void __user *user_buf,
			    size_t nbyte)
{
	struct tcp_socket *ts = rtdm_fd_to_private(fd);
	uint32_t sent_len = 0;
	rtdm_lockctx_t context;
	int ret = 0;
	nanosecs_rel_t sk_sndtimeo;
	void *buf;

	if (!rtdm_fd_is_user(fd)) {
		return -EFAULT;
	}

	rtdm_lock_get_irqsave(&ts->socket_lock, context);

	sk_sndtimeo = ts->sk_sndtimeo;

	if (!ts->is_valid) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return -EPIPE;
	}

	if ((ts->daddr | ts->dport) == 0 || ts->tcp_state != TCP_ESTABLISHED) {
		rtdm_lock_put_irqrestore(&ts->socket_lock, context);
		return -EINVAL;
	}

	rtdm_lock_put_irqrestore(&ts->socket_lock, context);

	buf = xnmalloc(nbyte);
	if (buf == NULL)
		return -ENOMEM;

	ret = rtdm_copy_from_user(fd, buf, user_buf, nbyte);
	if (ret) {
		xnfree(buf);
		return ret;
	}

	while (sent_len < nbyte) {
		ret = rtdm_event_timedwait(&ts->send_evt, sk_sndtimeo, NULL);

		if (unlikely(ret < 0))
			switch (ret) {
			case -EWOULDBLOCK:
			case -ETIMEDOUT:
			case -EINTR:
				xnfree(buf);
				return sent_len ?: ret;

			case -EIDRM: /* event is destroyed */
			default:
				if (ts->is_closed) {
					xnfree(buf);
					return -EBADF;
				}

				xnfree(buf);
				return sent_len ?: ret;
			}

		ret = rt_tcp_window_send(ts, nbyte - sent_len,
					 ((u8 *)buf) + sent_len);

		if (ret < 0) { /* check this branch correctness */
			rtdm_event_signal(&ts->send_evt);
			break;
		}

		sent_len += ret;
		if (ts->sync.dst_window)
			rtdm_event_signal(&ts->send_evt);
	}

	xnfree(buf);
	return (ret < 0 ? ret : sent_len);
}

/***
 *  rt_tcp_recvmsg
 */
static ssize_t rt_tcp_recvmsg(struct rtdm_fd *fd, struct user_msghdr *msg,
			      int msg_flags)
{
	struct iovec iov_fast[RTDM_IOV_FASTMAX], *iov;
	ssize_t ret;
	size_t len;
	void *buf;

	if (msg_flags)
		return -EOPNOTSUPP;

	/* loop over all vectors to be implemented */
	if (msg->msg_iovlen != 1)
		return -EOPNOTSUPP;

	ret = rtdm_get_iovec(fd, &iov, msg, iov_fast);
	if (ret)
		return ret;

	len = iov[0].iov_len;
	if (len > 0) {
		buf = iov[0].iov_base;
		ret = rt_tcp_read(fd, buf, len);
	}

	rtdm_drop_iovec(iov, iov_fast);

	return ret;
}

/***
 *  rt_tcp_sendmsg
 */
static ssize_t rt_tcp_sendmsg(struct rtdm_fd *fd, const struct user_msghdr *msg,
			      int msg_flags)
{
	struct iovec iov_fast[RTDM_IOV_FASTMAX], *iov;
	ssize_t ret;
	size_t len;

	if (msg_flags)
		return -EOPNOTSUPP;

	/* loop over all vectors to be implemented */
	if (msg->msg_iovlen != 1)
		return -EOPNOTSUPP;

	ret = rtdm_get_iovec(fd, &iov, msg, iov_fast);
	if (ret)
		return ret;

	len = iov[0].iov_len;
	if (len > 0)
		ret = rt_tcp_write(fd, iov[0].iov_base, len);

	rtdm_drop_iovec(iov, iov_fast);

	return ret;
}

/***
 *  rt_tcp_select
 */
static int rt_tcp_select(struct rtdm_fd *fd, rtdm_selector_t *selector,
			 enum rtdm_selecttype type, unsigned fd_index)
{
	struct tcp_socket *ts = rtdm_fd_to_private(fd);

	switch (type) {
	case XNSELECT_READ:
		return rtdm_sem_select(&ts->sock.pending_sem, selector,
				       XNSELECT_READ, fd_index);
	case XNSELECT_WRITE:
		return rtdm_event_select(&ts->send_evt, selector,
					 XNSELECT_WRITE, fd_index);
	default:
		return -EBADF;
	}

	return -EINVAL;
}

/***
 *  TCP-Initialisation
 */
static struct rtinet_protocol tcp_protocol = { .protocol = IPPROTO_TCP,
					       .dest_socket =
						       &rt_tcp_dest_socket,
					       .rcv_handler = &rt_tcp_rcv,
					       .err_handler = &rt_tcp_rcv_err,
					       .init_socket = &rt_tcp_socket };

static struct rtdm_driver tcp_driver = {
    .profile_info =     RTDM_PROFILE_INFO(tcp,
					RTDM_CLASS_NETWORK,
					RTDM_SUBCLASS_RTNET,
					RTNET_RTDM_VER),
    .device_flags =     RTDM_PROTOCOL_DEVICE,
    .device_count =	1,
    .context_size =     sizeof(struct tcp_socket),

    .protocol_family =  PF_INET,
    .socket_type =      SOCK_STREAM,

    .ops = {
	.socket     =   rt_inet_socket,
	.close      =   rt_tcp_close,
	.ioctl_rt   =   rt_tcp_ioctl,
	.ioctl_nrt  =   rt_tcp_ioctl,
	.read_rt    =   rt_tcp_read,
	.write_rt   =   rt_tcp_write,
	.recvmsg_rt =   rt_tcp_recvmsg,
	.sendmsg_rt =   rt_tcp_sendmsg,
	.select     =   rt_tcp_select,
    },
};

static struct rtdm_device tcp_device = {
	.driver = &tcp_driver,
	.label = "tcp",
};

#ifdef CONFIG_XENO_OPT_VFILE
/***
 *  rt_tcp_proc_read
 */
static inline char *rt_tcp_string_of_state(u8 state)
{
	switch (state) {
	case TCP_ESTABLISHED:
		return "ESTABLISHED";
	case TCP_SYN_SENT:
		return "SYN_SENT";
	case TCP_SYN_RECV:
		return "SYN_RECV";
	case TCP_FIN_WAIT1:
		return "FIN_WAIT1";
	case TCP_FIN_WAIT2:
		return "FIN_WAIT2";
	case TCP_TIME_WAIT:
		return "TIME_WAIT";
	case TCP_CLOSE:
		return "CLOSE";
	case TCP_CLOSE_WAIT:
		return "CLOSE_WAIT";
	case TCP_LAST_ACK:
		return "LASK_ACK";
	case TCP_LISTEN:
		return "LISTEN";
	case TCP_CLOSING:
		return "CLOSING";
	default:
		return "UNKNOWN";
	}
}

static int rtnet_ipv4_tcp_show(struct xnvfile_regular_iterator *it, void *data)
{
	rtdm_lockctx_t context;
	struct tcp_socket *ts;
	u32 saddr, daddr;
	u16 sport = 0, dport = 0; /* set to 0 to silence compiler */
	char sbuffer[24];
	char dbuffer[24];
	int state;
	int index;

	xnvfile_printf(it, "Hash    Local Address           "
			   "Foreign Address         State\n");

	for (index = 0; index < RT_TCP_SOCKETS; index++) {
		rtdm_lock_get_irqsave(&tcp_socket_base_lock, context);

		ts = port_registry[index];
		state = ts ? ts->tcp_state : TCP_CLOSE;

		if (ts && ts->tcp_state != TCP_CLOSE) {
			saddr = ts->saddr;
			sport = ts->sport;
			daddr = ts->daddr;
			dport = ts->dport;
		}

		rtdm_lock_put_irqrestore(&tcp_socket_base_lock, context);

		if (state != TCP_CLOSE) {
			snprintf(sbuffer, sizeof(sbuffer), "%u.%u.%u.%u:%u",
				 NIPQUAD(saddr), ntohs(sport));
			snprintf(dbuffer, sizeof(dbuffer), "%u.%u.%u.%u:%u",
				 NIPQUAD(daddr), ntohs(dport));

			xnvfile_printf(it, "%04X    %-23s %-23s %s\n",
				       sport & port_hash_mask, sbuffer, dbuffer,
				       rt_tcp_string_of_state(state));
		}
	}

	return 0;
}

static struct xnvfile_regular_ops rtnet_ipv4_tcp_vfile_ops = {
	.show = rtnet_ipv4_tcp_show,
};

static struct xnvfile_regular rtnet_ipv4_tcp_vfile = {
	.ops = &rtnet_ipv4_tcp_vfile_ops,
};

/***
 *  rt_tcp_proc_register
 */
static int __init rt_tcp_proc_register(void)
{
	return xnvfile_init_regular("tcp", &rtnet_ipv4_tcp_vfile,
				    &ipv4_proc_root);
}

/***
 *  rt_tcp_proc_unregister
 */

static void rt_tcp_proc_unregister(void)
{
	xnvfile_destroy_regular(&rtnet_ipv4_tcp_vfile);
}
#endif /* CONFIG_XENO_OPT_VFILE */

/***
 *  rt_tcp_init
 */
int __init rt_tcp_init(void)
{
	unsigned int skbs;
	int i;
	int ret;

	if ((tcp_auto_port_start < 0) ||
	    (tcp_auto_port_start >= 0x10000 - RT_TCP_SOCKETS))
		tcp_auto_port_start = 1024;
	tcp_auto_port_start =
		htons(tcp_auto_port_start & (tcp_auto_port_mask & 0xFFFF));
	tcp_auto_port_mask = htons(tcp_auto_port_mask | 0xFFFF0000);

	for (i = 0; i < ARRAY_SIZE(port_hash); i++)
		INIT_HLIST_HEAD(&port_hash[i]);

	/* Perform essential initialization of the RST|ACK socket */
	skbs = rt_bare_socket_init(rst_fd, IPPROTO_TCP, RT_TCP_RST_PRIO,
				   RT_TCP_RST_POOL_SIZE);
	if (skbs < RT_TCP_RST_POOL_SIZE)
		printk("rttcp: allocated only %d RST|ACK rtskbs\n", skbs);
	rst_socket.sock.prot.inet.tos = 0;
	rst_fd->refs = 1;
	rtdm_lock_init(&rst_socket.socket_lock);

	/*
     * 100 ms forwarding timer with 8.38 ms slots
     */
	ret = timerwheel_init(100000000ull, 23);
	if (ret < 0) {
		rtdm_printk("rttcp: cann't initialize timerwheel task: %d\n",
			    -ret);
		goto out_1;
	}

#ifdef CONFIG_XENO_OPT_VFILE
	if ((ret = rt_tcp_proc_register()) < 0) {
		rtdm_printk("rttcp: cann't initialize proc entry: %d\n", -ret);
		goto out_2;
	}
#endif /* CONFIG_XENO_OPT_VFILE */

	rt_inet_add_protocol(&tcp_protocol);

	ret = rtdm_dev_register(&tcp_device);
	if (ret < 0) {
		rtdm_printk("rttcp: cann't register RT TCP: %d\n", -ret);
		goto out_3;
	}

	return ret;

out_3:
	rt_inet_del_protocol(&tcp_protocol);
#ifdef CONFIG_XENO_OPT_VFILE
	rt_tcp_proc_unregister();
#endif /* CONFIG_XENO_OPT_VFILE */

out_2:
	timerwheel_cleanup();

out_1:
	rt_bare_socket_cleanup(&rst_socket.sock);

	return ret;
}

/***
 *  rt_tcp_release
 */
void __exit rt_tcp_release(void)
{
	rt_inet_del_protocol(&tcp_protocol);

#ifdef CONFIG_XENO_OPT_VFILE
	rt_tcp_proc_unregister();
#endif /* CONFIG_XENO_OPT_VFILE */

	timerwheel_cleanup();

	rt_bare_socket_cleanup(&rst_socket.sock);

	rtdm_dev_unregister(&tcp_device);
}

module_init(rt_tcp_init);
module_exit(rt_tcp_release);
