/***
 *
 *  stack/rtdev.c - NIC device driver layer
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

#include <linux/spinlock.h>
#include <linux/if.h>
#include <linux/if_arp.h> /* ARPHRD_ETHER */
#include <linux/netdevice.h>
#include <linux/moduleparam.h>

#include <rtnet_internal.h>
#include <rtskb.h>
#include <ethernet/eth.h>
#include <rtmac/rtmac_disc.h>
#include <rtnet_port.h>


static unsigned int device_rtskbs = DEFAULT_DEVICE_RTSKBS;
module_param(device_rtskbs, uint, 0444);
MODULE_PARM_DESC(device_rtskbs, "Number of additional global realtime socket "
		 "buffers per network adapter");

struct rtnet_device         *rtnet_devices[MAX_RT_DEVICES];
static struct rtnet_device  *loopback_device;
static DEFINE_RTDM_LOCK(rtnet_devices_rt_lock);

LIST_HEAD(event_hook_list);
LIST_HEAD(rtskb_list);
DEFINE_MUTEX(rtnet_devices_nrt_lock);

static int rtdev_locked_xmit(struct rtskb *skb, struct rtnet_device *rtdev);

int rtdev_reference(struct rtnet_device *rtdev)
{
    smp_mb__before_atomic();
    if (rtdev->rt_owner && __atomic_add_unless(&rtdev->refcount, 1, 0) == 0) {
	if (!try_module_get(rtdev->rt_owner))
	    return 0;
	if (atomic_inc_return(&rtdev->refcount) != 1)
	    module_put(rtdev->rt_owner);
    }
    return 1;
}
EXPORT_SYMBOL_GPL(rtdev_reference);

struct rtskb *rtnetdev_alloc_rtskb(struct rtnet_device *rtdev, unsigned int size)
{
    struct rtskb *rtskb = alloc_rtskb(size, &rtdev->dev_pool);
    if (rtskb)
	rtskb->rtdev = rtdev;
    return rtskb;
}
EXPORT_SYMBOL_GPL(rtnetdev_alloc_rtskb);

/***
 *  __rtdev_get_by_name - find a rtnet_device by its name
 *  @name: name to find
 *  @note: caller must hold rtnet_devices_nrt_lock
 */
static struct rtnet_device *__rtdev_get_by_name(const char *name)
{
    int                 i;
    struct rtnet_device *rtdev;


    for (i = 0; i < MAX_RT_DEVICES; i++) {
	rtdev = rtnet_devices[i];
	if ((rtdev != NULL) && (strncmp(rtdev->name, name, IFNAMSIZ) == 0))
	    return rtdev;
    }
    return NULL;
}


/***
 *  rtdev_get_by_name - find and lock a rtnet_device by its name
 *  @name: name to find
 */
struct rtnet_device *rtdev_get_by_name(const char *name)
{
    struct rtnet_device *rtdev;
    rtdm_lockctx_t      context;


    rtdm_lock_get_irqsave(&rtnet_devices_rt_lock, context);

    rtdev = __rtdev_get_by_name(name);
    if (rtdev != NULL && !rtdev_reference(rtdev))
	    rtdev = NULL;

    rtdm_lock_put_irqrestore(&rtnet_devices_rt_lock, context);

    return rtdev;
}



/***
 *  rtdev_get_by_index - find and lock a rtnet_device by its ifindex
 *  @ifindex: index of device
 */
struct rtnet_device *rtdev_get_by_index(int ifindex)
{
    struct rtnet_device *rtdev;
    rtdm_lockctx_t      context;


    if ((ifindex <= 0) || (ifindex > MAX_RT_DEVICES))
	return NULL;

    rtdm_lock_get_irqsave(&rtnet_devices_rt_lock, context);

    rtdev = __rtdev_get_by_index(ifindex);
    if (rtdev != NULL && !rtdev_reference(rtdev))
	    rtdev = NULL;

    rtdm_lock_put_irqrestore(&rtnet_devices_rt_lock, context);

    return rtdev;
}



/***
 *  __rtdev_get_by_hwaddr - find a rtnetdevice by its mac-address
 *  @type:          Type of the net_device (may be ARPHRD_ETHER)
 *  @hw_addr:       MAC-Address
 */
static inline struct rtnet_device *__rtdev_get_by_hwaddr(unsigned short type, char *hw_addr)
{
    int                 i;
    struct rtnet_device *rtdev;


    for (i = 0; i < MAX_RT_DEVICES; i++) {
	rtdev = rtnet_devices[i];
	if ((rtdev != NULL) && (rtdev->type == type) &&
	    (!memcmp(rtdev->dev_addr, hw_addr, rtdev->addr_len))) {
	    return rtdev;
	}
    }
    return NULL;
}



/***
 *  rtdev_get_by_hwaddr - find and lock a rtnetdevice by its mac-address
 *  @type:          Type of the net_device (may be ARPHRD_ETHER)
 *  @hw_addr:       MAC-Address
 */
struct rtnet_device *rtdev_get_by_hwaddr(unsigned short type, char *hw_addr)
{
    struct rtnet_device *rtdev;
    rtdm_lockctx_t      context;


    rtdm_lock_get_irqsave(&rtnet_devices_rt_lock, context);

    rtdev = __rtdev_get_by_hwaddr(type, hw_addr);
    if (rtdev != NULL && !rtdev_reference(rtdev))
	    rtdev = NULL;

    rtdm_lock_put_irqrestore(&rtnet_devices_rt_lock, context);

    return rtdev;
}



/***
 *  rtdev_get_by_hwaddr - find and lock the loopback device if available
 */
struct rtnet_device *rtdev_get_loopback(void)
{
    struct rtnet_device *rtdev;
    rtdm_lockctx_t      context;


    rtdm_lock_get_irqsave(&rtnet_devices_rt_lock, context);

    rtdev = loopback_device;
    if (rtdev != NULL && !rtdev_reference(rtdev))
	    rtdev = NULL;

    rtdm_lock_put_irqrestore(&rtnet_devices_rt_lock, context);

    return rtdev;
}



/***
 *  rtdev_alloc_name - allocate a name for the rtnet_device
 *  @rtdev:         the rtnet_device
 *  @name_mask:     a name mask (e.g. "rteth%d" for ethernet)
 *
 *  This function have to be called from the driver probe function.
 */
void rtdev_alloc_name(struct rtnet_device *rtdev, const char *mask)
{
    char                buf[IFNAMSIZ];
    int                 i;
    struct rtnet_device *tmp;


    for (i = 0; i < MAX_RT_DEVICES; i++) {
	snprintf(buf, IFNAMSIZ, mask, i);
	if ((tmp = rtdev_get_by_name(buf)) == NULL) {
	    strncpy(rtdev->name, buf, IFNAMSIZ);
	    break;
	}
	else
	    rtdev_dereference(tmp);
    }
}

static int rtdev_pool_trylock(void *cookie)
{
    return rtdev_reference(cookie);
}

static void rtdev_pool_unlock(void *cookie)
{
    rtdev_dereference(cookie);
}

static const struct rtskb_pool_lock_ops rtdev_ops = {
    .trylock = rtdev_pool_trylock,
    .unlock = rtdev_pool_unlock,
};

/***
 *  rtdev_alloc
 *  @int sizeof_priv:
 *
 *  allocate memory for a new rt-network-adapter
 */
struct rtnet_device *rtdev_alloc(unsigned sizeof_priv, unsigned dev_pool_size)
{
    struct rtnet_device *rtdev;
    unsigned            alloc_size;
    int                 ret;


    /* ensure 32-byte alignment of the private area */
    alloc_size = sizeof (*rtdev) + sizeof_priv + 31;

    rtdev = (struct rtnet_device *)kmalloc(alloc_size, GFP_KERNEL);
    if (rtdev == NULL) {
	printk(KERN_ERR "RTnet: cannot allocate rtnet device\n");
	return NULL;
    }

    memset(rtdev, 0, alloc_size);

    ret = rtskb_pool_init(&rtdev->dev_pool, dev_pool_size, &rtdev_ops, rtdev);
    if (ret < dev_pool_size) {
	printk(KERN_ERR "RTnet: cannot allocate rtnet device pool\n");
	rtskb_pool_release(&rtdev->dev_pool);
	kfree(rtdev);
	return NULL;
    }

    rtdm_mutex_init(&rtdev->xmit_mutex);
    rtdm_lock_init(&rtdev->rtdev_lock);
    mutex_init(&rtdev->nrt_lock);

    atomic_set(&rtdev->refcount, 0);

    /* scale global rtskb pool */
    rtdev->add_rtskbs = rtskb_pool_extend(&global_pool, device_rtskbs);

    if (sizeof_priv)
	rtdev->priv = (void *)(((long)(rtdev + 1) + 31) & ~31);

    return rtdev;
}



/***
 *  rtdev_free
 */
void rtdev_free (struct rtnet_device *rtdev)
{
    if (rtdev != NULL) {
	rtskb_pool_release(&rtdev->dev_pool);
	rtskb_pool_shrink(&global_pool, rtdev->add_rtskbs);
	rtdev->stack_event = NULL;
	rtdm_mutex_destroy(&rtdev->xmit_mutex);
	kfree(rtdev);
    }
}



/**
 * rtalloc_etherdev - Allocates and sets up an ethernet device
 * @sizeof_priv: size of additional driver-private structure to
 *               be allocated for this ethernet device
 * @dev_pool_size: size of the rx pool
 * @module: module creating the deivce
 *
 * Fill in the fields of the device structure with ethernet-generic
 * values. Basically does everything except registering the device.
 *
 * A 32-byte alignment is enforced for the private data area.
 */
struct rtnet_device *__rt_alloc_etherdev(unsigned sizeof_priv,
					unsigned dev_pool_size,
					struct module *module)
{
    struct rtnet_device *rtdev;

    rtdev = rtdev_alloc(sizeof_priv, dev_pool_size);
    if (!rtdev)
	return NULL;

    rtdev->hard_header     = rt_eth_header;
    rtdev->type            = ARPHRD_ETHER;
    rtdev->hard_header_len = ETH_HLEN;
    rtdev->mtu             = 1500; /* eth_mtu */
    rtdev->addr_len        = ETH_ALEN;
    rtdev->flags           = IFF_BROADCAST; /* TODO: IFF_MULTICAST; */
    rtdev->get_mtu         = rt_hard_mtu;
    rtdev->rt_owner	   = module;

    memset(rtdev->broadcast, 0xFF, ETH_ALEN);
    strcpy(rtdev->name, "rteth%d");

    return rtdev;
}



static inline int __rtdev_new_index(void)
{
    int i;


    for (i = 0; i < MAX_RT_DEVICES; i++)
	if (rtnet_devices[i] == NULL)
	     return i+1;

    return -ENOMEM;
}



static int rtskb_map(struct rtnet_device *rtdev, struct rtskb *skb)
{
    dma_addr_t addr;

    addr = rtdev->map_rtskb(rtdev, skb);

    if (WARN_ON(addr == RTSKB_UNMAPPED))
	return -ENOMEM;

    if (skb->buf_dma_addr != RTSKB_UNMAPPED &&
	addr != skb->buf_dma_addr) {
	printk("RTnet: device %s maps skb differently than others. "
	       "Different IOMMU domain?\nThis is not supported.\n",
	       rtdev->name);
	return -EACCES;
    }

    skb->buf_dma_addr = addr;

    return 0;
}



int rtdev_map_rtskb(struct rtskb *skb)
{
    struct rtnet_device *rtdev;
    int err = 0;
    int i;

    skb->buf_dma_addr = RTSKB_UNMAPPED;

    mutex_lock(&rtnet_devices_nrt_lock);

    for (i = 0; i < MAX_RT_DEVICES; i++) {
	rtdev = rtnet_devices[i];
	if (rtdev && rtdev->map_rtskb) {
	    err = rtskb_map(rtdev, skb);
	    if (err)
		break;
	}
    }

    if (!err)
	list_add(&skb->entry, &rtskb_list);

    mutex_unlock(&rtnet_devices_nrt_lock);

    return err;
}



static int rtdev_map_all_rtskbs(struct rtnet_device *rtdev)
{
    struct rtskb *skb;
    int err = 0;

    if (!rtdev->map_rtskb)
	return 0;

    list_for_each_entry(skb, &rtskb_list, entry) {
	err = rtskb_map(rtdev, skb);
	if (err)
	   break;
    }

    return err;
}



void rtdev_unmap_rtskb(struct rtskb *skb)
{
    struct rtnet_device *rtdev;
    int i;

    if (skb->buf_dma_addr == RTSKB_UNMAPPED)
	return;

    mutex_lock(&rtnet_devices_nrt_lock);

    list_del(&skb->entry);

    for (i = 0; i < MAX_RT_DEVICES; i++) {
	rtdev = rtnet_devices[i];
	if (rtdev && rtdev->unmap_rtskb) {
	    rtdev->unmap_rtskb(rtdev, skb);
	}
    }

    skb->buf_dma_addr = RTSKB_UNMAPPED;

    mutex_unlock(&rtnet_devices_nrt_lock);
}



static void rtdev_unmap_all_rtskbs(struct rtnet_device *rtdev)
{
    struct rtskb *skb;

    if (!rtdev->unmap_rtskb)
	return;

    list_for_each_entry(skb, &rtskb_list, entry) {
	rtdev->unmap_rtskb(rtdev, skb);
    }
}



/***
 * rt_register_rtnetdev: register a new rtnet_device (linux-like)
 * @rtdev:               the device
 */
int rt_register_rtnetdev(struct rtnet_device *rtdev)
{
    struct list_head        *entry;
    struct rtdev_event_hook *hook;
    rtdm_lockctx_t          context;
    int                     ifindex;
    int                     err;


    /* requires at least driver layer version 2.0 */
    if (rtdev->vers < RTDEV_VERS_2_0)
	return -EINVAL;

    if (rtdev->features & NETIF_F_LLTX)
	rtdev->start_xmit = rtdev->hard_start_xmit;
    else
	rtdev->start_xmit = rtdev_locked_xmit;

    mutex_lock(&rtnet_devices_nrt_lock);

    ifindex = __rtdev_new_index();
    if (ifindex < 0) {
	mutex_unlock(&rtnet_devices_nrt_lock);
	return ifindex;
    }
    rtdev->ifindex = ifindex;

    if (strchr(rtdev->name,'%') != NULL)
	rtdev_alloc_name(rtdev, rtdev->name);

    if (__rtdev_get_by_name(rtdev->name) != NULL) {
	mutex_unlock(&rtnet_devices_nrt_lock);
	return -EEXIST;
    }

    err = rtdev_map_all_rtskbs(rtdev);
    if (err) {
	mutex_unlock(&rtnet_devices_nrt_lock);
	return err;
    }

    rtdm_lock_get_irqsave(&rtnet_devices_rt_lock, context);

    if (rtdev->flags & IFF_LOOPBACK) {
	/* allow only one loopback device */
	if (loopback_device) {
	    rtdm_lock_put_irqrestore(&rtnet_devices_rt_lock, context);
	    mutex_unlock(&rtnet_devices_nrt_lock);
	    return -EEXIST;
	}
	loopback_device = rtdev;
    }
    rtnet_devices[rtdev->ifindex-1] = rtdev;

    rtdm_lock_put_irqrestore(&rtnet_devices_rt_lock, context);

    list_for_each(entry, &event_hook_list) {
	hook = list_entry(entry, struct rtdev_event_hook, entry);
	if (hook->register_device)
	    hook->register_device(rtdev);
    }

    mutex_unlock(&rtnet_devices_nrt_lock);

    /* Default state at registration is that the device is present. */
    set_bit(__RTNET_LINK_STATE_PRESENT, &rtdev->link_state);

    printk("RTnet: registered %s\n", rtdev->name);

    return 0;
}



/***
 * rt_unregister_rtnetdev: unregister a rtnet_device
 * @rtdev:                 the device
 */
int rt_unregister_rtnetdev(struct rtnet_device *rtdev)
{
    struct list_head        *entry;
    struct rtdev_event_hook *hook;
    rtdm_lockctx_t          context;


    RTNET_ASSERT(rtdev->ifindex != 0,
	printk("RTnet: device %s/%p was not registered\n", rtdev->name, rtdev);
	return -ENODEV;);

    mutex_lock(&rtnet_devices_nrt_lock);
    rtdm_lock_get_irqsave(&rtnet_devices_rt_lock, context);

    RTNET_ASSERT(atomic_read(&rtdev->refcount == 0), BUG());
    rtnet_devices[rtdev->ifindex-1] = NULL;
    if (rtdev->flags & IFF_LOOPBACK)
	loopback_device = NULL;

    rtdm_lock_put_irqrestore(&rtnet_devices_rt_lock, context);

    list_for_each(entry, &event_hook_list) {
	hook = list_entry(entry, struct rtdev_event_hook, entry);
	if (hook->unregister_device)
	    hook->unregister_device(rtdev);
    }

    rtdev_unmap_all_rtskbs(rtdev);

    mutex_unlock(&rtnet_devices_nrt_lock);

    clear_bit(__RTNET_LINK_STATE_PRESENT, &rtdev->link_state);

    RTNET_ASSERT(atomic_read(&rtdev->refcount) == 0,
	   printk("RTnet: rtdev reference counter < 0!\n"););

    printk("RTnet: unregistered %s\n", rtdev->name);

    return 0;
}



void rtdev_add_event_hook(struct rtdev_event_hook *hook)
{
    mutex_lock(&rtnet_devices_nrt_lock);
    list_add(&hook->entry, &event_hook_list);
    mutex_unlock(&rtnet_devices_nrt_lock);
}



void rtdev_del_event_hook(struct rtdev_event_hook *hook)
{
    mutex_lock(&rtnet_devices_nrt_lock);
    list_del(&hook->entry);
    mutex_unlock(&rtnet_devices_nrt_lock);
}



/***
 *  rtdev_open
 *
 *  Prepare an interface for use.
 */
int rtdev_open(struct rtnet_device *rtdev)
{
    int ret = 0;


    if (rtdev->flags & IFF_UP)              /* Is it already up?                */
	return 0;

    if (!rtdev_reference(rtdev))
	return -EIDRM;

    if (rtdev->open)                        /* Call device private open method  */
	ret = rtdev->open(rtdev);

    if ( !ret )  {
	rtdev->flags |= IFF_UP;
	set_bit(__RTNET_LINK_STATE_START, &rtdev->link_state);
    } else
	rtdev_dereference(rtdev);

    return ret;
}



/***
 *  rtdev_close
 */
int rtdev_close(struct rtnet_device *rtdev)
{
    int ret = 0;


    if ( !(rtdev->flags & IFF_UP) )
	return 0;

    if (rtdev->stop)
	ret = rtdev->stop(rtdev);

    rtdev->flags &= ~(IFF_UP|IFF_RUNNING);
    clear_bit(__RTNET_LINK_STATE_START, &rtdev->link_state);

    if (ret == 0)
	rtdev_dereference(rtdev);

    return ret;
}



static int rtdev_locked_xmit(struct rtskb *skb, struct rtnet_device *rtdev)
{
    int ret;


    rtdm_mutex_lock(&rtdev->xmit_mutex);
    ret = rtdev->hard_start_xmit(skb, rtdev);
    rtdm_mutex_unlock(&rtdev->xmit_mutex);

    return ret;
}



/***
 *  rtdev_xmit - send real-time packet
 */
int rtdev_xmit(struct rtskb *rtskb)
{
    struct rtnet_device *rtdev;
    int                 err;


    RTNET_ASSERT(rtskb != NULL, return -EINVAL;);

    rtdev = rtskb->rtdev;

    if (!rtnetif_carrier_ok(rtdev)) {
	err = -EAGAIN;
	kfree_rtskb(rtskb);
	return err;
    }

    if (rtskb_acquire(rtskb, &rtdev->dev_pool) != 0) {
	err = -ENOBUFS;
	kfree_rtskb(rtskb);
	return err;
    }

    RTNET_ASSERT(rtdev != NULL, return -EINVAL;);

    err = rtdev->start_xmit(rtskb, rtdev);
    if (err) {
	/* on error we must free the rtskb here */
	kfree_rtskb(rtskb);

	rtdm_printk("hard_start_xmit returned %d\n", err);
    }

    return err;
}



#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_PROXY)
/***
 *      rtdev_xmit_proxy - send rtproxy packet
 */
int rtdev_xmit_proxy(struct rtskb *rtskb)
{
    struct rtnet_device *rtdev;
    int                 err;


    RTNET_ASSERT(rtskb != NULL, return -EINVAL;);

    rtdev = rtskb->rtdev;

    RTNET_ASSERT(rtdev != NULL, return -EINVAL;);

    /* TODO: make these lines race-condition-safe */
    if (rtdev->mac_disc) {
	RTNET_ASSERT(rtdev->mac_disc->nrt_packet_tx != NULL, return -EINVAL;);

	err = rtdev->mac_disc->nrt_packet_tx(rtskb);
    } else {
	err = rtdev->start_xmit(rtskb, rtdev);
	if (err) {
	    /* on error we must free the rtskb here */
	    kfree_rtskb(rtskb);

	    rtdm_printk("hard_start_xmit returned %d\n", err);
	}
    }

    return err;
}
#endif /* CONFIG_XENO_DRIVERS_NET_ADDON_PROXY */



unsigned int rt_hard_mtu(struct rtnet_device *rtdev, unsigned int priority)
{
    return rtdev->mtu;
}


EXPORT_SYMBOL_GPL(__rt_alloc_etherdev);
EXPORT_SYMBOL_GPL(rtdev_free);

EXPORT_SYMBOL_GPL(rtdev_alloc_name);

EXPORT_SYMBOL_GPL(rt_register_rtnetdev);
EXPORT_SYMBOL_GPL(rt_unregister_rtnetdev);

EXPORT_SYMBOL_GPL(rtdev_add_event_hook);
EXPORT_SYMBOL_GPL(rtdev_del_event_hook);

EXPORT_SYMBOL_GPL(rtdev_get_by_name);
EXPORT_SYMBOL_GPL(rtdev_get_by_index);
EXPORT_SYMBOL_GPL(rtdev_get_by_hwaddr);
EXPORT_SYMBOL_GPL(rtdev_get_loopback);

EXPORT_SYMBOL_GPL(rtdev_xmit);

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_PROXY)
EXPORT_SYMBOL_GPL(rtdev_xmit_proxy);
#endif

EXPORT_SYMBOL_GPL(rt_hard_mtu);
