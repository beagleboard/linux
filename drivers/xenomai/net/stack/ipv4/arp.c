/***
 *
 *  ipv4/arp.h - Adress Resolution Protocol for RTnet
 *
 *  Copyright (C) 2002 Ulrich Marx <marx@kammer.uni-hannover.de>
 *                2004 Jan Kiszka <jan.kiszka@web.de>
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

#include <rtdev.h>
#include <stack_mgr.h>
#include <ipv4/arp.h>

#ifdef CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP
#include <ipv4/ip_input.h>
#endif /* CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP */

/***
 *  arp_send:   Create and send an arp packet. If (dest_hw == NULL),
 *              we create a broadcast message.
 */
void rt_arp_send(int type, int ptype, u32 dest_ip, struct rtnet_device *rtdev,
		 u32 src_ip, unsigned char *dest_hw, unsigned char *src_hw,
		 unsigned char *target_hw)
{
    struct rtskb *skb;
    struct arphdr *arp;
    unsigned char *arp_ptr;

    if (rtdev->flags & IFF_NOARP)
	return;

    if (!(skb=alloc_rtskb(sizeof(struct arphdr) + 2*(rtdev->addr_len+4) +
			   rtdev->hard_header_len+15, &global_pool)))
	return;

    rtskb_reserve(skb, (rtdev->hard_header_len+15)&~15);

    skb->nh.raw = skb->data;
    arp = (struct arphdr *)rtskb_put(skb, sizeof(struct arphdr) +
				     2*(rtdev->addr_len+4));

    skb->rtdev = rtdev;
    skb->protocol = __constant_htons (ETH_P_ARP);
    skb->priority = RT_ARP_SKB_PRIO;
    if (src_hw == NULL)
	src_hw = rtdev->dev_addr;
    if (dest_hw == NULL)
	dest_hw = rtdev->broadcast;

    /*
     *  Fill the device header for the ARP frame
     */
    if (rtdev->hard_header &&
	(rtdev->hard_header(skb,rtdev,ptype,dest_hw,src_hw,skb->len) < 0))
	goto out;

    arp->ar_hrd = htons(rtdev->type);
    arp->ar_pro = __constant_htons(ETH_P_IP);
    arp->ar_hln = rtdev->addr_len;
    arp->ar_pln = 4;
    arp->ar_op = htons(type);

    arp_ptr=(unsigned char *)(arp+1);

    memcpy(arp_ptr, src_hw, rtdev->addr_len);
    arp_ptr+=rtdev->addr_len;

    memcpy(arp_ptr, &src_ip,4);
    arp_ptr+=4;

    if (target_hw != NULL)
	memcpy(arp_ptr, target_hw, rtdev->addr_len);
    else
	memset(arp_ptr, 0, rtdev->addr_len);
    arp_ptr+=rtdev->addr_len;

    memcpy(arp_ptr, &dest_ip, 4);


    /* send the frame */
    rtdev_xmit(skb);

    return;

  out:
    kfree_rtskb(skb);
}



/***
 *  arp_rcv:    Receive an arp request by the device layer.
 */
int rt_arp_rcv(struct rtskb *skb, struct rtpacket_type *pt)
{
    struct rtnet_device *rtdev = skb->rtdev;
    struct arphdr       *arp = skb->nh.arph;
    unsigned char       *arp_ptr= (unsigned char *)(arp+1);
    unsigned char       *sha;
    u32                 sip, tip;
    u16                 dev_type = rtdev->type;

    /*
     *  The hardware length of the packet should match the hardware length
     *  of the device.  Similarly, the hardware types should match.  The
     *  device should be ARP-able.  Also, if pln is not 4, then the lookup
     *  is not from an IP number.  We can't currently handle this, so toss
     *  it.
     */
    if ((arp->ar_hln != rtdev->addr_len) ||
	(rtdev->flags & IFF_NOARP) ||
	(skb->pkt_type == PACKET_OTHERHOST) ||
	(skb->pkt_type == PACKET_LOOPBACK) ||
	(arp->ar_pln != 4))
	goto out;

    switch (dev_type) {
	default:
	    if ((arp->ar_pro != __constant_htons(ETH_P_IP)) &&
		(htons(dev_type) != arp->ar_hrd))
		goto out;
	    break;
	case ARPHRD_ETHER:
	    /*
	     * ETHERNET devices will accept ARP hardware types of either
	     * 1 (Ethernet) or 6 (IEEE 802.2).
	     */
	    if ((arp->ar_hrd != __constant_htons(ARPHRD_ETHER)) &&
		(arp->ar_hrd != __constant_htons(ARPHRD_IEEE802))) {
		goto out;
	    }
	    if (arp->ar_pro != __constant_htons(ETH_P_IP)) {
		goto out;
	    }
	    break;
    }

    /* Understand only these message types */
    if ((arp->ar_op != __constant_htons(ARPOP_REPLY)) &&
	(arp->ar_op != __constant_htons(ARPOP_REQUEST)))
	goto out;

    /*
     *  Extract fields
     */
    sha=arp_ptr;
    arp_ptr += rtdev->addr_len;
    memcpy(&sip, arp_ptr, 4);

    arp_ptr += 4;
    arp_ptr += rtdev->addr_len;
    memcpy(&tip, arp_ptr, 4);

    /* process only requests/replies directed to us */
    if (tip == rtdev->local_ip) {
	rt_ip_route_add_host(sip, sha, rtdev);

#ifdef CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP
	if (!rt_ip_fallback_handler)
#endif /* CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP */
		if (arp->ar_op == __constant_htons(ARPOP_REQUEST)) {
			rt_arp_send(ARPOP_REPLY, ETH_P_ARP, sip, rtdev, tip, sha,
				rtdev->dev_addr, sha);
			goto out1;
		}
    }

out:
#ifdef CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP
    if (rt_ip_fallback_handler) {
	    rt_ip_fallback_handler(skb);
	    return 0;
    }
#endif /* CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP */
out1:
    kfree_rtskb(skb);
    return 0;
}



static struct rtpacket_type arp_packet_type = {
    type:       __constant_htons(ETH_P_ARP),
    handler:    &rt_arp_rcv
};



/***
 *  rt_arp_init
 */
void __init rt_arp_init(void)
{
    rtdev_add_pack(&arp_packet_type);
}



/***
 *  rt_arp_release
 */
void rt_arp_release(void)
{
    rtdev_remove_pack(&arp_packet_type);
}
