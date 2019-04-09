/* rtmac_vnic.c
 *
 * rtmac - real-time networking media access control subsystem
 * Copyright (C) 2002      Marc Kleine-Budde <kleine-budde@gmx.de>,
 *               2003-2005 Jan Kiszka <Jan.Kiszka@web.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>

#include <rtnet_internal.h>
#include <rtdev.h>
#include <rtnet_port.h> /* for netdev_priv() */
#include <rtmac/rtmac_disc.h>
#include <rtmac/rtmac_proto.h>
#include <rtmac/rtmac_vnic.h>


static unsigned int vnic_rtskbs = DEFAULT_VNIC_RTSKBS;
module_param(vnic_rtskbs, uint, 0444);
MODULE_PARM_DESC(vnic_rtskbs, "Number of realtime socket buffers per virtual NIC");

static rtdm_nrtsig_t        vnic_signal;
static struct rtskb_queue   rx_queue;



int rtmac_vnic_rx(struct rtskb *rtskb, u16 type)
{
    struct rtmac_priv *mac_priv = rtskb->rtdev->mac_priv;
    struct rtskb_pool *pool = &mac_priv->vnic_skb_pool;


    if (rtskb_acquire(rtskb, pool) != 0) {
	mac_priv->vnic_stats.rx_dropped++;
	kfree_rtskb(rtskb);
	return -1;
    }

    rtskb->protocol = type;

    rtskb_queue_tail(&rx_queue, rtskb);
    rtdm_nrtsig_pend(&vnic_signal);

    return 0;
}



static void rtmac_vnic_signal_handler(rtdm_nrtsig_t *nrtsig, void *arg)
{
    struct rtskb            *rtskb;
    struct sk_buff          *skb;
    unsigned                hdrlen;
    struct net_device_stats *stats;
    struct rtnet_device     *rtdev;


    while (1)
    {
	rtskb = rtskb_dequeue(&rx_queue);
	if (!rtskb)
	    break;

	rtdev  = rtskb->rtdev;
	hdrlen = rtdev->hard_header_len;

	skb = dev_alloc_skb(hdrlen + rtskb->len + 2);
	if (skb) {
	    /* the rtskb stamp is useless (different clock), get new one */
	    __net_timestamp(skb);

	    skb_reserve(skb, 2); /* Align IP on 16 byte boundaries */

	    /* copy Ethernet header */
	    memcpy(skb_put(skb, hdrlen),
		   rtskb->data - hdrlen - sizeof(struct rtmac_hdr), hdrlen);

	    /* patch the protocol field in the original Ethernet header */
	    ((struct ethhdr*)skb->data)->h_proto = rtskb->protocol;

	    /* copy data */
	    memcpy(skb_put(skb, rtskb->len), rtskb->data, rtskb->len);

	    skb->dev      = rtskb->rtdev->mac_priv->vnic;
	    skb->protocol = eth_type_trans(skb, skb->dev);

	    stats = &rtskb->rtdev->mac_priv->vnic_stats;

	    kfree_rtskb(rtskb);

	    stats->rx_packets++;
	    stats->rx_bytes += skb->len;

	    netif_rx(skb);
	}
	else {
	    printk("RTmac: VNIC fails to allocate linux skb\n");
	    kfree_rtskb(rtskb);
	}
    }
}



static int rtmac_vnic_copy_mac(struct net_device *dev)
{
    memcpy(dev->dev_addr,
	   (*(struct rtnet_device **)netdev_priv(dev))->dev_addr,
	   MAX_ADDR_LEN);

    return 0;
}



int rtmac_vnic_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct rtnet_device     *rtdev = *(struct rtnet_device **)netdev_priv(dev);
    struct net_device_stats *stats = &rtdev->mac_priv->vnic_stats;
    struct rtskb_pool       *pool = &rtdev->mac_priv->vnic_skb_pool;
    struct ethhdr           *ethernet = (struct ethhdr*)skb->data;
    struct rtskb            *rtskb;
    int                     res;
    int                     data_len;


    rtskb =
	alloc_rtskb((skb->len + sizeof(struct rtmac_hdr) + 15) & ~15, pool);
    if (!rtskb)
	return NETDEV_TX_BUSY;

    rtskb_reserve(rtskb, rtdev->hard_header_len + sizeof(struct rtmac_hdr));

    data_len = skb->len - dev->hard_header_len;
    memcpy(rtskb_put(rtskb, data_len), skb->data + dev->hard_header_len,
	   data_len);

    res = rtmac_add_header(rtdev, ethernet->h_dest, rtskb,
			   ntohs(ethernet->h_proto), RTMAC_FLAG_TUNNEL);
    if (res < 0) {
	stats->tx_dropped++;
	kfree_rtskb(rtskb);
	goto done;
    }

    RTNET_ASSERT(rtdev->mac_disc->nrt_packet_tx != NULL, kfree_rtskb(rtskb);
		 goto done;);

    res = rtdev->mac_disc->nrt_packet_tx(rtskb);
    if (res < 0) {
	stats->tx_dropped++;
	kfree_rtskb(rtskb);
    } else {
	stats->tx_packets++;
	stats->tx_bytes += skb->len;
    }

done:
    dev_kfree_skb(skb);
    return NETDEV_TX_OK;
}



static struct net_device_stats *rtmac_vnic_get_stats(struct net_device *dev)
{
    return &(*(struct rtnet_device **)netdev_priv(dev))->mac_priv->vnic_stats;
}



static int rtmac_vnic_change_mtu(struct net_device *dev, int new_mtu)
{
    if ((new_mtu < 68) ||
	((unsigned)new_mtu > 1500 - sizeof(struct rtmac_hdr)))
	return -EINVAL;
    dev->mtu = new_mtu;
    return 0;
}



void rtmac_vnic_set_max_mtu(struct rtnet_device *rtdev, unsigned int max_mtu)
{
    struct rtmac_priv   *mac_priv = rtdev->mac_priv;
    struct net_device   *vnic = mac_priv->vnic;
    unsigned int        prev_mtu  = mac_priv->vnic_max_mtu;


    mac_priv->vnic_max_mtu = max_mtu - sizeof(struct rtmac_hdr);

    /* set vnic mtu in case max_mtu is smaller than the current mtu or
       the current mtu was set to previous max_mtu */
    rtnl_lock();
    if ((vnic->mtu > mac_priv->vnic_max_mtu) || (prev_mtu == mac_priv->vnic_max_mtu)) {
	dev_set_mtu(vnic, mac_priv->vnic_max_mtu);
    }
    rtnl_unlock();
}


static struct net_device_ops vnic_netdev_ops = {
    .ndo_open       = rtmac_vnic_copy_mac,
    .ndo_get_stats  = rtmac_vnic_get_stats,
    .ndo_change_mtu = rtmac_vnic_change_mtu,
};

static void rtmac_vnic_setup(struct net_device *dev)
{
    ether_setup(dev);

    dev->netdev_ops      = &vnic_netdev_ops;
    dev->flags           &= ~IFF_MULTICAST;
}

int rtmac_vnic_add(struct rtnet_device *rtdev, vnic_xmit_handler vnic_xmit)
{
    int                 res;
    struct rtmac_priv   *mac_priv = rtdev->mac_priv;
    struct net_device   *vnic;
    char                buf[IFNAMSIZ];


    /* does the discipline request vnic support? */
    if (!vnic_xmit)
	return 0;

    mac_priv->vnic = NULL;
    mac_priv->vnic_max_mtu = rtdev->mtu - sizeof(struct rtmac_hdr);
    memset(&mac_priv->vnic_stats, 0, sizeof(mac_priv->vnic_stats));

    /* create the rtskb pool */
    if (rtskb_pool_init(&mac_priv->vnic_skb_pool,
			    vnic_rtskbs, NULL, NULL) < vnic_rtskbs) {
	res = -ENOMEM;
	goto error;
    }

    snprintf(buf, sizeof(buf), "vnic%d", rtdev->ifindex-1);

    vnic = alloc_netdev(sizeof(struct rtnet_device *), buf,
		    NET_NAME_UNKNOWN, rtmac_vnic_setup);
    if (!vnic) {
	res = -ENOMEM;
	goto error;
    }

    vnic_netdev_ops.ndo_start_xmit = vnic_xmit;
    vnic->mtu = mac_priv->vnic_max_mtu;
    *(struct rtnet_device **)netdev_priv(vnic) = rtdev;
    rtmac_vnic_copy_mac(vnic);

    res = register_netdev(vnic);
    if (res < 0)
	goto error;

    mac_priv->vnic = vnic;

    return 0;

 error:
    rtskb_pool_release(&mac_priv->vnic_skb_pool);
    return res;
}



int rtmac_vnic_unregister(struct rtnet_device *rtdev)
{
    struct rtmac_priv   *mac_priv = rtdev->mac_priv;

    if (mac_priv->vnic) {
	rtskb_pool_release(&mac_priv->vnic_skb_pool);
	unregister_netdev(mac_priv->vnic);
	free_netdev(mac_priv->vnic);
	mac_priv->vnic = NULL;
    }

    return 0;
}



#ifdef CONFIG_XENO_OPT_VFILE
int rtnet_rtmac_vnics_show(struct xnvfile_regular_iterator *it, void *d)
{
    struct rtnet_device *rtdev;
    int                 i;
    int                 err;

    xnvfile_printf(it, "RT-NIC name\tVNIC name\n");

    for (i = 1; i <= MAX_RT_DEVICES; i++) {
	rtdev = rtdev_get_by_index(i);
	if (rtdev == NULL)
	    continue;

	err = mutex_lock_interruptible(&rtdev->nrt_lock);
	if (err < 0) {
	    rtdev_dereference(rtdev);
	    return err;
	}

	if (rtdev->mac_priv != NULL) {
	    struct rtmac_priv *rtmac;

	    rtmac = (struct rtmac_priv *)rtdev->mac_priv;
	    xnvfile_printf(it, "%-15s %s\n", rtdev->name, rtmac->vnic->name);
	}

	mutex_unlock(&rtdev->nrt_lock);
	rtdev_dereference(rtdev);
    }

    return 0;
}
#endif /* CONFIG_XENO_OPT_VFILE */



int __init rtmac_vnic_module_init(void)
{
    rtskb_queue_init(&rx_queue);

    rtdm_nrtsig_init(&vnic_signal, rtmac_vnic_signal_handler, NULL);

    return 0;
}



void rtmac_vnic_module_cleanup(void)
{
    struct rtskb *rtskb;


    rtdm_nrtsig_destroy(&vnic_signal);

    while ((rtskb = rtskb_dequeue(&rx_queue)) != NULL) {
	kfree_rtskb(rtskb);
    }
}
