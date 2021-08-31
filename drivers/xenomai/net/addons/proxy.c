/* rtnetproxy.c: a Linux network driver that uses the RTnet driver to
 * transport IP data from/to Linux kernel mode.
 * This allows the usage of TCP/IP from linux space using via the RTNET
 * network adapter.
 *
 *
 * Usage:
 *
 * insmod rtnetproxy.o    (only after having rtnet up and running)
 *
 * ifconfig rtproxy up IP_ADDRESS netmask NETMASK
 *
 * Use it like any other network device from linux.
 *
 * Restrictions:
 * Only IPV4 based protocols are supported, UDP and ICMP can be send out
 * but not received - as these are handled directly by rtnet!
 *
 *
 *
 * Based on the linux net driver dummy.c by Nick Holloway
 *
 *
 * Changelog:
 *
 * 08-Nov-2002  Mathias Koehrer - Clear separation between rtai context and
 *                                standard linux driver context.
 *                                Data exchange via ringbuffers.
 *                                A RTAI thread is used for rtnet transmission.
 *
 * 05-Nov-2002  Mathias Koehrer - Initial version!
 *                                Development based on rtnet 0.2.6,
 *                                rtai-24.1.10, kernel 2.4.19
 *
 *
 * Mathias Koehrer - mathias_koehrer@yahoo.de
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/init.h>
#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <net/ip.h>

#include <linux/if_ether.h> /* For the statistics structure. */
#include <linux/if_arp.h> /* For ARPHRD_ETHER */

#include <rtdev.h>
#include <rtskb.h>
#include <rtdm/driver.h>
#include <ipv4/ip_input.h>
#include <ipv4/route.h>
#include <rtnet_port.h>

static struct net_device *dev_rtnetproxy;

/* **************************************************************************
 *  SKB pool management (JK):
 * ************************************************************************ */
#define DEFAULT_PROXY_RTSKBS 32

static unsigned int proxy_rtskbs = DEFAULT_PROXY_RTSKBS;
module_param(proxy_rtskbs, uint, 0444);
MODULE_PARM_DESC(proxy_rtskbs,
		 "Number of realtime socket buffers in proxy pool");

static struct rtskb_pool rtskb_pool;

static struct rtskb_queue tx_queue;
static struct rtskb_queue rx_queue;

/* handle for non-real-time signal */
static rtdm_nrtsig_t rtnetproxy_rx_signal;

/* Thread for transmission */
static rtdm_task_t rtnetproxy_tx_task;

static rtdm_event_t rtnetproxy_tx_event;

#ifdef CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP
static char *rtdev_attach = "rteth0";
module_param(rtdev_attach, charp, 0444);
MODULE_PARM_DESC(rtdev_attach, "Attach to the specified RTnet device");

struct rtnet_device *rtnetproxy_rtdev;
#endif

/* ************************************************************************
 * ************************************************************************
 *   T R A N S M I T
 * ************************************************************************
 * ************************************************************************ */

static void rtnetproxy_tx_loop(void *arg)
{
	struct rtnet_device *rtdev;
	struct rtskb *rtskb;

	while (!rtdm_task_should_stop()) {
		if (rtdm_event_wait(&rtnetproxy_tx_event) < 0)
			break;

		while ((rtskb = rtskb_dequeue(&tx_queue)) != NULL) {
			rtdev = rtskb->rtdev;
			rtdev_xmit_proxy(rtskb);
			rtdev_dereference(rtdev);
		}
	}
}

/* ************************************************************************
 *  hard_xmit
 *
 *  This function runs in linux kernel context and is executed whenever
 *  there is a frame to be sent out.
 * ************************************************************************ */
static int rtnetproxy_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct ethhdr *eth = (struct ethhdr *)skb->data;
	struct rtskb *rtskb;
	int len = skb->len;
#ifndef CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP
	struct dest_route rt;
	struct iphdr *iph;
	u32 saddr, daddr;
#endif

	switch (ntohs(eth->h_proto)) {
	case ETH_P_IP:
		if (len < sizeof(struct ethhdr) + sizeof(struct iphdr))
			goto drop1;
#ifdef CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP
	case ETH_P_ARP:
#endif
		break;
	default:
	drop1:
		dev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	rtskb = alloc_rtskb(len, &rtskb_pool);
	if (!rtskb)
		return NETDEV_TX_BUSY;

	memcpy(rtskb_put(rtskb, len), skb->data, len);

#ifdef CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP
	dev_kfree_skb(skb);

	rtskb->rtdev = rtnetproxy_rtdev;
	if (rtdev_reference(rtnetproxy_rtdev) == 0) {
		dev->stats.tx_dropped++;
		kfree_rtskb(rtskb);
		return NETDEV_TX_BUSY;
	}

#else /* !CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP */
	iph = (struct iphdr *)(skb->data + sizeof(struct ethhdr));
	saddr = iph->saddr;
	daddr = iph->daddr;

	dev_kfree_skb(skb);

	if (rt_ip_route_output(&rt, daddr, INADDR_ANY) < 0) {
	drop2:
		dev->stats.tx_dropped++;
		kfree_rtskb(rtskb);
		return NETDEV_TX_OK;
	}
	if (rt.rtdev->local_ip != saddr) {
		rtdev_dereference(rt.rtdev);
		goto drop2;
	}

	eth = (struct ethhdr *)rtskb->data;
	memcpy(eth->h_source, rt.rtdev->dev_addr, rt.rtdev->addr_len);
	memcpy(eth->h_dest, rt.dev_addr, rt.rtdev->addr_len);

	rtskb->rtdev = rt.rtdev;
#endif /* CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP */

	dev->stats.tx_packets++;
	dev->stats.tx_bytes += len;

	rtskb_queue_tail(&tx_queue, rtskb);
	rtdm_event_signal(&rtnetproxy_tx_event);

	return NETDEV_TX_OK;
}

/* ************************************************************************
 * ************************************************************************
 *   R E C E I V E
 * ************************************************************************
 * ************************************************************************ */

/* ************************************************************************
 * This function runs in real-time context.
 *
 * It is called from inside rtnet whenever a packet has been received that
 * has to be processed by rtnetproxy.
 * ************************************************************************ */
static void rtnetproxy_recv(struct rtskb *rtskb)
{
	/* Acquire rtskb (JK) */
	if (rtskb_acquire(rtskb, &rtskb_pool) != 0) {
		dev_rtnetproxy->stats.rx_dropped++;
		rtdm_printk("rtnetproxy_recv: No free rtskb in pool\n");
		kfree_rtskb(rtskb);
		return;
	}

	if (rtskb_queue_tail_check(&rx_queue, rtskb))
		rtdm_nrtsig_pend(&rtnetproxy_rx_signal);
}

/* ************************************************************************
 * This function runs in kernel mode.
 * It is activated from rtnetproxy_signal_handler whenever rtnet received a
 * frame to be processed by rtnetproxy.
 * ************************************************************************ */
static inline void rtnetproxy_kernel_recv(struct rtskb *rtskb)
{
	struct sk_buff *skb;
	struct net_device *dev = dev_rtnetproxy;

	int header_len = rtskb->rtdev->hard_header_len;
	int len = rtskb->len + header_len;

	/* Copy the realtime skb (rtskb) to the standard skb: */
	skb = dev_alloc_skb(len + 2);
	skb_reserve(skb, 2);

	memcpy(skb_put(skb, len), rtskb->data - header_len, len);

	/* Set some relevant entries in the skb: */
	skb->protocol = eth_type_trans(skb, dev);
	skb->dev = dev;
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	skb->pkt_type = PACKET_HOST; /* Extremely important! Why?!? */

	/* the rtskb stamp is useless (different clock), get new one */
	__net_timestamp(skb);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 11, 0)
	dev->last_rx = jiffies;
#endif
	dev->stats.rx_bytes += skb->len;
	dev->stats.rx_packets++;

	netif_rx(skb); /* pass it to the received stuff */
}

/* ************************************************************************
 * This function runs in kernel mode.
 * It is activated from rtnetproxy_recv whenever rtnet received a frame to
 * be processed by rtnetproxy.
 * ************************************************************************ */
static void rtnetproxy_signal_handler(rtdm_nrtsig_t *nrtsig, void *arg)
{
	struct rtskb *rtskb;

	while ((rtskb = rtskb_dequeue(&rx_queue)) != NULL) {
		rtnetproxy_kernel_recv(rtskb);
		kfree_rtskb(rtskb);
	}
}

/* ************************************************************************
 * ************************************************************************
 *   G E N E R A L
 * ************************************************************************
 * ************************************************************************ */

static void fake_multicast_support(struct net_device *dev)
{
}

#ifdef CONFIG_NET_FASTROUTE
static int rtnetproxy_accept_fastpath(struct net_device *dev,
				      struct dst_entry *dst)
{
	return -1;
}
#endif

static int rtnetproxy_open(struct net_device *dev)
{
	int err = try_module_get(THIS_MODULE);
	if (err == 0)
		return -EIDRM;

	return 0;
}

static int rtnetproxy_stop(struct net_device *dev)
{
	module_put(THIS_MODULE);
	return 0;
}

static const struct net_device_ops rtnetproxy_netdev_ops = {
	.ndo_open = rtnetproxy_open,
	.ndo_stop = rtnetproxy_stop,
	.ndo_start_xmit = rtnetproxy_xmit,
	.ndo_set_rx_mode = fake_multicast_support,
};

/* ************************************************************************
 *  device init
 * ************************************************************************ */
static void __init rtnetproxy_init(struct net_device *dev)
{
	/* Fill in device structure with ethernet-generic values. */
	ether_setup(dev);

	dev->tx_queue_len = 0;
#ifdef CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP
	memcpy(dev->dev_addr, rtnetproxy_rtdev->dev_addr, MAX_ADDR_LEN);
#else
	dev->flags |= IFF_NOARP;
#endif
	dev->flags &= ~IFF_MULTICAST;

	dev->netdev_ops = &rtnetproxy_netdev_ops;
}

/* ************************************************************************
 * ************************************************************************
 *   I N I T
 * ************************************************************************
 * ************************************************************************ */
static int __init rtnetproxy_init_module(void)
{
	int err;

#ifdef CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP
	if ((rtnetproxy_rtdev = rtdev_get_by_name(rtdev_attach)) == NULL) {
		printk("Couldn't attach to %s\n", rtdev_attach);
		return -EINVAL;
	}
	printk("RTproxy attached to %s\n", rtdev_attach);
#endif

	/* Initialize the proxy's rtskb pool (JK) */
	if (rtskb_module_pool_init(&rtskb_pool, proxy_rtskbs) < proxy_rtskbs) {
		err = -ENOMEM;
		goto err1;
	}

	dev_rtnetproxy =
		alloc_netdev(0, "rtproxy", NET_NAME_UNKNOWN, rtnetproxy_init);
	if (!dev_rtnetproxy) {
		err = -ENOMEM;
		goto err1;
	}

	rtdm_nrtsig_init(&rtnetproxy_rx_signal, rtnetproxy_signal_handler,
			 NULL);

	rtskb_queue_init(&tx_queue);
	rtskb_queue_init(&rx_queue);

	err = register_netdev(dev_rtnetproxy);
	if (err < 0)
		goto err3;

	/* Init the task for transmission */
	rtdm_event_init(&rtnetproxy_tx_event, 0);
	err = rtdm_task_init(&rtnetproxy_tx_task, "rtnetproxy",
			     rtnetproxy_tx_loop, 0, RTDM_TASK_LOWEST_PRIORITY,
			     0);
	if (err)
		goto err4;

	/* Register with RTnet */
	rt_ip_fallback_handler = rtnetproxy_recv;

	printk("rtnetproxy installed as \"%s\"\n", dev_rtnetproxy->name);

	return 0;

err4:
	unregister_netdev(dev_rtnetproxy);

err3:
	rtdm_nrtsig_destroy(&rtnetproxy_rx_signal);

	free_netdev(dev_rtnetproxy);

err1:
	rtskb_pool_release(&rtskb_pool);
#ifdef CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP
	rtdev_dereference(rtnetproxy_rtdev);
#endif
	return err;
}

static void __exit rtnetproxy_cleanup_module(void)
{
	struct rtskb *rtskb;

	/* Unregister the fallback at rtnet */
	rt_ip_fallback_handler = NULL;

	/* Unregister the net device: */
	unregister_netdev(dev_rtnetproxy);
	free_netdev(dev_rtnetproxy);

	rtdm_event_destroy(&rtnetproxy_tx_event);
	rtdm_task_destroy(&rtnetproxy_tx_task);

	/* free the non-real-time signal */
	rtdm_nrtsig_destroy(&rtnetproxy_rx_signal);

	while ((rtskb = rtskb_dequeue(&tx_queue)) != NULL) {
		rtdev_dereference(rtskb->rtdev);
		kfree_rtskb(rtskb);
	}

	while ((rtskb = rtskb_dequeue(&rx_queue)) != NULL) {
		kfree_rtskb(rtskb);
	}

	rtskb_pool_release(&rtskb_pool);

#ifdef CONFIG_XENO_DRIVERS_NET_ADDON_PROXY_ARP
	rtdev_dereference(rtnetproxy_rtdev);
#endif
}

module_init(rtnetproxy_init_module);
module_exit(rtnetproxy_cleanup_module);
MODULE_LICENSE("GPL");
