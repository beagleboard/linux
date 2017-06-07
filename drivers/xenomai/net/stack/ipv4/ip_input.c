/***
 *
 *  ipv4/ip_input.c - process incoming IP packets
 *
 *  Copyright (C) 2002      Ulrich Marx <marx@kammer.uni-hannover.de>
 *                2003-2005 Jan Kiszka <jan.kiszka@web.de>
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

#include <net/checksum.h>
#include <net/ip.h>

#include <rtskb.h>
#include <rtnet_socket.h>
#include <stack_mgr.h>
#include <ipv4/ip_fragment.h>
#include <ipv4/protocol.h>
#include <ipv4/route.h>

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_PROXY)
#include <ipv4/ip_input.h>

rt_ip_fallback_handler_t rt_ip_fallback_handler = NULL;
EXPORT_SYMBOL_GPL(rt_ip_fallback_handler);
#endif /* CONFIG_XENO_DRIVERS_NET_ADDON_PROXY */



/***
 *  rt_ip_local_deliver
 */
static inline void rt_ip_local_deliver(struct rtskb *skb)
{
    struct iphdr *iph       = skb->nh.iph;
    unsigned short protocol = iph->protocol;
    struct rtinet_protocol *ipprot;
    struct rtsocket *sock;
    int err;


    ipprot = rt_inet_protocols[rt_inet_hashkey(protocol)];

    /* Check if we are supporting the protocol */
    if ((ipprot != NULL) && (ipprot->protocol == protocol))
    {
        __rtskb_pull(skb, iph->ihl*4);

        /* Point into the IP datagram, just past the header. */
        skb->h.raw = skb->data;

        /* Reassemble IP fragments */
        if (iph->frag_off & htons(IP_MF|IP_OFFSET)) {
            skb = rt_ip_defrag(skb, ipprot);
            if (!skb)
                return;

	    sock = skb->sk;
        } else {
            /* Get the destination socket */
            if ((sock = ipprot->dest_socket(skb)) == NULL) {
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_PROXY)
                if (rt_ip_fallback_handler) {
                    __rtskb_push(skb, iph->ihl*4);
                    rt_ip_fallback_handler(skb);
                    return;
                }
#endif
                kfree_rtskb(skb);
                return;
            }

            /* Acquire the rtskb, to unlock the device skb pool */
            err = rtskb_acquire(skb, &sock->skb_pool);

            if (err) {
                kfree_rtskb(skb);
		rt_socket_dereference(sock);
                return;
            }
        }

        /* Deliver the packet to the next layer */
        ipprot->rcv_handler(skb);

	/* Packet is queued, socket can be released */
	rt_socket_dereference(sock);
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_PROXY)
    } else if (rt_ip_fallback_handler) {
        /* If a fallback handler for IP protocol has been installed,
         * call it. */
        rt_ip_fallback_handler(skb);
#endif /* CONFIG_XENO_DRIVERS_NET_ADDON_PROXY */
    } else {
        rtdm_printk("RTnet: no protocol found\n");
        kfree_rtskb(skb);
    }
}



/***
 *  rt_ip_rcv
 */
int rt_ip_rcv(struct rtskb *skb, struct rtpacket_type *pt)
{
    struct iphdr *iph;
    __u32 len;

    /* When the interface is in promisc. mode, drop all the crap
     * that it receives, do not try to analyse it.
     */
    if (skb->pkt_type == PACKET_OTHERHOST)
        goto drop;

    iph = skb->nh.iph;

    /*
     *  RFC1122: 3.1.2.2 MUST silently discard any IP frame that fails the checksum.
     *
     *  Is the datagram acceptable?
     *
     *  1.  Length at least the size of an ip header
     *  2.  Version of 4
     *  3.  Checksums correctly. [Speed optimisation for later, skip loopback checksums]
     *  4.  Doesn't have a bogus length
     */
    if (iph->ihl < 5 || iph->version != 4)
        goto drop;

    if ( ip_fast_csum((u8 *)iph, iph->ihl)!=0 )
        goto drop;

    len = ntohs(iph->tot_len);
    if ( (skb->len<len) || (len<((__u32)iph->ihl<<2)) )
        goto drop;

    rtskb_trim(skb, len);

#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_ROUTER
    if (rt_ip_route_forward(skb, iph->daddr))
        return 0;
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_ROUTER */

    rt_ip_local_deliver(skb);
    return 0;

  drop:
    kfree_rtskb(skb);
    return 0;
}
