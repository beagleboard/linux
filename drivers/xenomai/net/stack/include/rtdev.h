/***
 *
 *  rtdev.h
 *
 *  RTnet - real-time networking subsystem
 *  Copyright (C) 1999       Lineo, Inc
 *                1999, 2002 David A. Schleef <ds@schleef.org>
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

#ifndef __RTDEV_H_
#define __RTDEV_H_

#define MAX_RT_DEVICES 8

#ifdef __KERNEL__

#include <asm/atomic.h>
#include <linux/netdevice.h>

#include <rtskb.h>
#include <rtnet_internal.h>

#define RTDEV_VERS_2_0 0x0200

#define PRIV_FLAG_UP 0
#define PRIV_FLAG_ADDING_ROUTE 1

#ifndef NETIF_F_LLTX
#define NETIF_F_LLTX 4096
#endif

#define RTDEV_TX_OK 0
#define RTDEV_TX_BUSY 1

enum rtnet_link_state {
	__RTNET_LINK_STATE_XOFF = 0,
	__RTNET_LINK_STATE_START,
	__RTNET_LINK_STATE_PRESENT,
	__RTNET_LINK_STATE_NOCARRIER,
};
#define RTNET_LINK_STATE_XOFF (1 << __RTNET_LINK_STATE_XOFF)
#define RTNET_LINK_STATE_START (1 << __RTNET_LINK_STATE_START)
#define RTNET_LINK_STATE_PRESENT (1 << __RTNET_LINK_STATE_PRESENT)
#define RTNET_LINK_STATE_NOCARRIER (1 << __RTNET_LINK_STATE_NOCARRIER)

/***
 *  rtnet_device
 */
struct rtnet_device {
	/* Many field are borrowed from struct net_device in
     * <linux/netdevice.h> - WY
     */
	unsigned int vers;

	char name[IFNAMSIZ];
	struct device *sysbind; /* device bound in sysfs (optional) */

	unsigned long rmem_end; /* shmem "recv" end     */
	unsigned long rmem_start; /* shmem "recv" start   */
	unsigned long mem_end; /* shared mem end       */
	unsigned long mem_start; /* shared mem start     */
	unsigned long base_addr; /* device I/O address   */
	unsigned int irq; /* device IRQ number    */

	/*
     *  Some hardware also needs these fields, but they are not
     *  part of the usual set specified in Space.c.
     */
	unsigned char if_port; /* Selectable AUI, TP,..*/
	unsigned char dma; /* DMA channel          */
	__u16 __padding;

	unsigned long link_state;
	int ifindex;
	atomic_t refcount;

	struct device *sysdev; /* node in driver model for sysfs */
	struct module *rt_owner; /* like classic owner, but      *
				     * forces correct macro usage   */

	unsigned int flags; /* interface flags (a la BSD)   */
	unsigned long priv_flags; /* internal flags               */
	unsigned short type; /* interface hardware type      */
	unsigned short hard_header_len; /* hardware hdr length  */
	unsigned int mtu; /* eth = 1536, tr = 4...        */
	void *priv; /* pointer to private data      */
	netdev_features_t features; /* [RT]NETIF_F_*                */

	/* Interface address info. */
	unsigned char broadcast[MAX_ADDR_LEN]; /* hw bcast add */
	unsigned char dev_addr[MAX_ADDR_LEN]; /* hw address   */
	unsigned char addr_len; /* hardware address length      */

	int promiscuity;
	int allmulti;

	__u32 local_ip; /* IP address in network order  */
	__u32 broadcast_ip; /* broadcast IP in network order */

	rtdm_event_t *stack_event;

	rtdm_mutex_t xmit_mutex; /* protects xmit routine        */
	rtdm_lock_t rtdev_lock; /* management lock              */
	struct mutex nrt_lock; /* non-real-time locking        */

	unsigned int add_rtskbs; /* additionally allocated global rtskbs */

	struct rtskb_pool dev_pool;

	/* RTmac related fields */
	struct rtmac_disc *mac_disc;
	struct rtmac_priv *mac_priv;
	int (*mac_detach)(struct rtnet_device *rtdev);

	/* Device operations */
	int (*open)(struct rtnet_device *rtdev);
	int (*stop)(struct rtnet_device *rtdev);
	int (*hard_header)(struct rtskb *, struct rtnet_device *,
			   unsigned short type, void *daddr, void *saddr,
			   unsigned int len);
	int (*rebuild_header)(struct rtskb *);
	int (*hard_start_xmit)(struct rtskb *skb, struct rtnet_device *dev);
	int (*hw_reset)(struct rtnet_device *rtdev);

	/* Transmission hook, managed by the stack core, RTcap, and RTmac
     *
     * If xmit_lock is used, start_xmit points either to rtdev_locked_xmit or
     * the RTmac discipline handler. If xmit_lock is not required, start_xmit
     * points to hard_start_xmit or the discipline handler.
     */
	int (*start_xmit)(struct rtskb *skb, struct rtnet_device *dev);

	/* MTU hook, managed by the stack core and RTmac */
	unsigned int (*get_mtu)(struct rtnet_device *rtdev,
				unsigned int priority);

	int (*do_ioctl)(struct rtnet_device *rtdev, struct ifreq *ifr, int cmd);
	struct net_device_stats *(*get_stats)(struct rtnet_device *rtdev);

	/* DMA pre-mapping hooks */
	dma_addr_t (*map_rtskb)(struct rtnet_device *rtdev, struct rtskb *skb);
	void (*unmap_rtskb)(struct rtnet_device *rtdev, struct rtskb *skb);
};

struct rtnet_core_cmd;

struct rtdev_event_hook {
	struct list_head entry;
	void (*register_device)(struct rtnet_device *rtdev);
	void (*unregister_device)(struct rtnet_device *rtdev);
	void (*ifup)(struct rtnet_device *rtdev, struct rtnet_core_cmd *up_cmd);
	void (*ifdown)(struct rtnet_device *rtdev);
};

extern struct list_head event_hook_list;
extern struct mutex rtnet_devices_nrt_lock;
extern struct rtnet_device *rtnet_devices[];

int __rt_init_etherdev(struct rtnet_device *rtdev, unsigned int dev_pool_size,
		       struct module *module);

#define rt_init_etherdev(__rtdev, __dev_pool_size)                             \
	__rt_init_etherdev(__rtdev, __dev_pool_size, THIS_MODULE)

struct rtnet_device *__rt_alloc_etherdev(unsigned sizeof_priv,
					 unsigned dev_pool_size,
					 struct module *module);
#define rt_alloc_etherdev(priv_size, rx_size)                                  \
	__rt_alloc_etherdev(priv_size, rx_size, THIS_MODULE)

void rtdev_destroy(struct rtnet_device *rtdev);

void rtdev_free(struct rtnet_device *rtdev);

int rt_register_rtnetdev(struct rtnet_device *rtdev);
int rt_unregister_rtnetdev(struct rtnet_device *rtdev);

void rtdev_add_event_hook(struct rtdev_event_hook *hook);
void rtdev_del_event_hook(struct rtdev_event_hook *hook);

void rtdev_alloc_name(struct rtnet_device *rtdev, const char *name_mask);

/**
 *  __rtdev_get_by_index - find a rtnet_device by its ifindex
 *  @ifindex: index of device
 *  @note: caller must hold rtnet_devices_nrt_lock
 */
static inline struct rtnet_device *__rtdev_get_by_index(int ifindex)
{
	return rtnet_devices[ifindex - 1];
}

struct rtnet_device *rtdev_get_by_name(const char *if_name);
struct rtnet_device *rtdev_get_by_index(int ifindex);
struct rtnet_device *rtdev_get_by_hwaddr(unsigned short type, char *ha);
struct rtnet_device *rtdev_get_loopback(void);

int rtdev_reference(struct rtnet_device *rtdev);

static inline void rtdev_dereference(struct rtnet_device *rtdev)
{
	smp_mb__before_atomic();
	if (rtdev->rt_owner && atomic_dec_and_test(&rtdev->refcount))
		module_put(rtdev->rt_owner);
}

int rtdev_xmit(struct rtskb *skb);

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_PROXY)
int rtdev_xmit_proxy(struct rtskb *skb);
#endif

unsigned int rt_hard_mtu(struct rtnet_device *rtdev, unsigned int priority);

int rtdev_open(struct rtnet_device *rtdev);
int rtdev_close(struct rtnet_device *rtdev);

int rtdev_up(struct rtnet_device *rtdev, struct rtnet_core_cmd *cmd);
int rtdev_down(struct rtnet_device *rtdev);

int rtdev_map_rtskb(struct rtskb *skb);
void rtdev_unmap_rtskb(struct rtskb *skb);

struct rtskb *rtnetdev_alloc_rtskb(struct rtnet_device *dev, unsigned int size);

#define rtnetdev_priv(dev) ((dev)->priv)

#define rtdev_emerg(__dev, format, args...)                                    \
	pr_emerg("%s: " format, (__dev)->name, ##args)
#define rtdev_alert(__dev, format, args...)                                    \
	pr_alert("%s: " format, (__dev)->name, ##args)
#define rtdev_crit(__dev, format, args...)                                     \
	pr_crit("%s: " format, (__dev)->name, ##args)
#define rtdev_err(__dev, format, args...)                                      \
	pr_err("%s: " format, (__dev)->name, ##args)
#define rtdev_warn(__dev, format, args...)                                     \
	pr_warn("%s: " format, (__dev)->name, ##args)
#define rtdev_notice(__dev, format, args...)                                   \
	pr_notice("%s: " format, (__dev)->name, ##args)
#define rtdev_info(__dev, format, args...)                                     \
	pr_info("%s: " format, (__dev)->name, ##args)
#define rtdev_dbg(__dev, format, args...)                                      \
	pr_debug("%s: " format, (__dev)->name, ##args)

#ifdef VERBOSE_DEBUG
#define rtdev_vdbg rtdev_dbg
#else
#define rtdev_vdbg(__dev, format, args...)                                     \
	({                                                                     \
		if (0)                                                         \
			pr_debug("%s: " format, (__dev)->name, ##args);        \
                                                                               \
		0;                                                             \
	})
#endif

#endif /* __KERNEL__ */

#endif /* __RTDEV_H_ */
