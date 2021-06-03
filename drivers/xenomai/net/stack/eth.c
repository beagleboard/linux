/***
 *
 *  stack/eth.c - Ethernet-specific functions
 *
 *  Copyright (C) 2002 Ulrich Marx <marx@kammer.uni-hannover.de>
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

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#include <rtdev.h>
#include <rtnet_internal.h>

/*
 *  Create the Ethernet MAC header for an arbitrary protocol layer
 *
 *  saddr=NULL  means use device source address
 *  daddr=NULL  means leave destination address (eg unresolved arp)
 */
int rt_eth_header(struct rtskb *skb, struct rtnet_device *rtdev,
		  unsigned short type, void *daddr, void *saddr, unsigned len)
{
	struct ethhdr *eth = (struct ethhdr *)rtskb_push(skb, ETH_HLEN);

	/*
     *  Set rtskb mac field
     */

	skb->mac.ethernet = eth;

	/*
     *  Set the protocol type. For a packet of type ETH_P_802_3 we put the length
     *  in here instead. It is up to the 802.2 layer to carry protocol information.
     */

	if (type != ETH_P_802_3)
		eth->h_proto = htons(type);
	else
		eth->h_proto = htons(len);

	/*
     *  Set the source hardware address.
     */

	if (saddr)
		memcpy(eth->h_source, saddr, rtdev->addr_len);
	else
		memcpy(eth->h_source, rtdev->dev_addr, rtdev->addr_len);

	if (rtdev->flags & (IFF_LOOPBACK | IFF_NOARP)) {
		memset(eth->h_dest, 0, rtdev->addr_len);
		return rtdev->hard_header_len;
	}

	if (daddr) {
		memcpy(eth->h_dest, daddr, rtdev->addr_len);
		return rtdev->hard_header_len;
	}

	return -rtdev->hard_header_len;
}

unsigned short rt_eth_type_trans(struct rtskb *skb, struct rtnet_device *rtdev)
{
	struct ethhdr *eth;
	unsigned char *rawp;

	rtcap_mark_incoming(skb);

	skb->mac.raw = skb->data;
	rtskb_pull(skb, rtdev->hard_header_len);
	eth = skb->mac.ethernet;

	if (*eth->h_dest & 1) {
		if (memcmp(eth->h_dest, rtdev->broadcast, ETH_ALEN) == 0)
			skb->pkt_type = PACKET_BROADCAST;
		else
			skb->pkt_type = PACKET_MULTICAST;
	}

	/*
     *  This ALLMULTI check should be redundant by 1.4
     *  so don't forget to remove it.
     *
     *  Seems, you forgot to remove it. All silly devices
     *  seems to set IFF_PROMISC.
     */

	else if (1 /*rtdev->flags&IFF_PROMISC*/) {
		if (memcmp(eth->h_dest, rtdev->dev_addr, ETH_ALEN))
			skb->pkt_type = PACKET_OTHERHOST;
	}

	if (ntohs(eth->h_proto) >= 1536)
		return eth->h_proto;

	rawp = skb->data;

	/*
     *  This is a magic hack to spot IPX packets. Older Novell breaks
     *  the protocol design and runs IPX over 802.3 without an 802.2 LLC
     *  layer. We look for FFFF which isn't a used 802.2 SSAP/DSAP. This
     *  won't work for fault tolerant netware but does for the rest.
     */
	if (*(unsigned short *)rawp == 0xFFFF)
		return htons(ETH_P_802_3);

	/*
     *  Real 802.2 LLC
     */
	return htons(ETH_P_802_2);
}

EXPORT_SYMBOL_GPL(rt_eth_header);
EXPORT_SYMBOL_GPL(rt_eth_type_trans);
