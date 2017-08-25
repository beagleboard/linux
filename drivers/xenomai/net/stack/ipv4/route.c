/***
 *
 *  ipv4/route.c - real-time routing
 *
 *  Copyright (C) 2004, 2005 Jan Kiszka <jan.kiszka@web.de>
 *
 *  Rewritten version of the original route by David Schleef and Ulrich Marx
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
#include <net/ip.h>

#include <rtnet_internal.h>
#include <rtnet_port.h>
#include <rtnet_chrdev.h>
#include <ipv4/af_inet.h>
#include <ipv4/route.h>


/* FIXME: should also become some tunable parameter */
#define ROUTER_FORWARD_PRIO \
    RTSKB_PRIO_VALUE(QUEUE_MAX_PRIO+(QUEUE_MIN_PRIO-QUEUE_MAX_PRIO+1)/2, \
		     RTSKB_DEF_RT_CHANNEL)


/* First-level routing: explicite host routes */
struct host_route {
    struct host_route       *next;
    struct dest_route       dest_host;
};

/* Second-level routing: routes to other networks */
struct net_route {
    struct net_route        *next;
    u32                     dest_net_ip;
    u32                     dest_net_mask;
    u32                     gw_ip;
};

#if (CONFIG_XENO_DRIVERS_NET_RTIPV4_HOST_ROUTES & (CONFIG_XENO_DRIVERS_NET_RTIPV4_HOST_ROUTES - 1))
# error CONFIG_XENO_DRIVERS_NET_RTIPV4_HOST_ROUTES must be power of 2
#endif
#if CONFIG_XENO_DRIVERS_NET_RTIPV4_HOST_ROUTES < 256
# define HOST_HASH_TBL_SIZE 64
#else
# define HOST_HASH_TBL_SIZE ((CONFIG_XENO_DRIVERS_NET_RTIPV4_HOST_ROUTES / 256) * 64)
#endif
#define HOST_HASH_KEY_MASK  (HOST_HASH_TBL_SIZE-1)

static struct host_route    host_routes[CONFIG_XENO_DRIVERS_NET_RTIPV4_HOST_ROUTES];
static struct host_route    *free_host_route;
static int                  allocated_host_routes;
static struct host_route    *host_hash_tbl[HOST_HASH_TBL_SIZE];
static DEFINE_RTDM_LOCK(host_table_lock);

#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING
#if (CONFIG_XENO_DRIVERS_NET_RTIPV4_NET_ROUTES & (CONFIG_XENO_DRIVERS_NET_RTIPV4_NET_ROUTES - 1))
# error CONFIG_XENO_DRIVERS_NET_RTIPV4_NET_ROUTES must be power of 2
#endif
#if CONFIG_XENO_DRIVERS_NET_RTIPV4_NET_ROUTES < 256
# define NET_HASH_TBL_SIZE  64
#else
# define NET_HASH_TBL_SIZE  ((CONFIG_XENO_DRIVERS_NET_RTIPV4_NET_ROUTES / 256) * 64)
#endif
#define NET_HASH_KEY_MASK   (NET_HASH_TBL_SIZE-1)
#define NET_HASH_KEY_SHIFT  8

static struct net_route     net_routes[CONFIG_XENO_DRIVERS_NET_RTIPV4_NET_ROUTES];
static struct net_route     *free_net_route;
static int                  allocated_net_routes;
static struct net_route     *net_hash_tbl[NET_HASH_TBL_SIZE + 1];
static unsigned int         net_hash_key_shift = NET_HASH_KEY_SHIFT;
static DEFINE_RTDM_LOCK(net_table_lock);

module_param(net_hash_key_shift, uint, 0444);
MODULE_PARM_DESC(net_hash_key_shift, "destination right shift for "
		 "network hash key (default: 8)");
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING */



/***
 *  proc filesystem section
 */
#ifdef CONFIG_XENO_OPT_VFILE
static int rtnet_ipv4_route_show(struct xnvfile_regular_iterator *it, void *d)
{
#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING
    u32 mask;
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING */

    xnvfile_printf(it, "Host routes allocated/total:\t%d/%d\n"
	    "Host hash table size:\t\t%d\n",
	    allocated_host_routes,
	    CONFIG_XENO_DRIVERS_NET_RTIPV4_HOST_ROUTES,
	    HOST_HASH_TBL_SIZE);

#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING
    mask = NET_HASH_KEY_MASK << net_hash_key_shift;
    xnvfile_printf(it, "Network routes allocated/total:\t%d/%d\n"
	    "Network hash table size:\t%d\n"
	    "Network hash key shift/mask:\t%d/%08X\n",
	    allocated_net_routes,
	    CONFIG_XENO_DRIVERS_NET_RTIPV4_NET_ROUTES, NET_HASH_TBL_SIZE,
	    net_hash_key_shift, mask);
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING */

#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_ROUTER
    xnvfile_printf(it, "IP Router:\t\t\tyes\n");
#else
    xnvfile_printf(it, "IP Router:\t\t\tno\n");
#endif

    return 0;
}

static int rtnet_ipv4_module_lock(struct xnvfile *vfile)
{
    bool res = try_module_get(THIS_MODULE);
    if (!res)
	return -EIDRM;

    return 0;
}

static void rtnet_ipv4_module_unlock(struct xnvfile *vfile)
{
    module_put(THIS_MODULE);
}

static struct xnvfile_lock_ops rtnet_ipv4_module_lock_ops = {
    .get = rtnet_ipv4_module_lock,
    .put = rtnet_ipv4_module_unlock,
};

static struct xnvfile_regular_ops rtnet_ipv4_route_vfile_ops = {
    .show = rtnet_ipv4_route_show,
};

static struct xnvfile_regular rtnet_ipv4_route_vfile = {
    .entry = {
	.lockops = &rtnet_ipv4_module_lock_ops,
    },
    .ops = &rtnet_ipv4_route_vfile_ops,
};

static rtdm_lockctx_t rtnet_ipv4_host_route_lock_ctx;

static int rtnet_ipv4_host_route_lock(struct xnvfile *vfile)
{
    rtdm_lock_get_irqsave(&host_table_lock, rtnet_ipv4_host_route_lock_ctx);
    return 0;
}

static void rtnet_ipv4_host_route_unlock(struct xnvfile *vfile)
{
    rtdm_lock_put_irqrestore(&host_table_lock, rtnet_ipv4_host_route_lock_ctx);
}

static struct xnvfile_lock_ops rtnet_ipv4_host_route_lock_ops = {
    .get = rtnet_ipv4_host_route_lock,
    .put = rtnet_ipv4_host_route_unlock,
};

struct rtnet_ipv4_host_route_priv {
    unsigned key;
    struct host_route *entry_ptr;
};

struct rtnet_ipv4_host_route_data {
    int key;
    char name[IFNAMSIZ];
    struct dest_route dest_host;
};

struct xnvfile_rev_tag host_route_tag;

static void *rtnet_ipv4_host_route_begin(struct xnvfile_snapshot_iterator *it)
{
    struct rtnet_ipv4_host_route_priv *priv = xnvfile_iterator_priv(it);
    struct rtnet_ipv4_host_route_data *data;
    unsigned routes;
    int err;

    routes = allocated_host_routes;
    if (!routes)
	return VFILE_SEQ_EMPTY;

    data = kmalloc(sizeof(*data) * routes, GFP_KERNEL);
    if (data == NULL)
	return NULL;

    err = rtnet_ipv4_module_lock(NULL);
    if (err < 0) {
	kfree(data);
	return VFILE_SEQ_EMPTY;
    }

    priv->key = -1;
    priv->entry_ptr = NULL;
    return data;
}

static void rtnet_ipv4_host_route_end(struct xnvfile_snapshot_iterator *it,
				    void *buf)
{
    rtnet_ipv4_module_unlock(NULL);
    kfree(buf);
}

static int rtnet_ipv4_host_route_next(struct xnvfile_snapshot_iterator *it,
				    void *data)
{
    struct rtnet_ipv4_host_route_priv *priv = xnvfile_iterator_priv(it);
    struct rtnet_ipv4_host_route_data *p = data;
    struct rtnet_device *rtdev;

    if (priv->entry_ptr == NULL) {
	if (++priv->key >= HOST_HASH_TBL_SIZE)
	    return 0;

	priv->entry_ptr = host_hash_tbl[priv->key];
	if (priv->entry_ptr == NULL)
	    return VFILE_SEQ_SKIP;
    }

    rtdev = priv->entry_ptr->dest_host.rtdev;

    if (!rtdev_reference(rtdev))
	return -EIDRM;

    memcpy(&p->name, rtdev->name, sizeof(p->name));

    rtdev_dereference(rtdev);

    p->key = priv->key;

    memcpy(&p->dest_host, &priv->entry_ptr->dest_host, sizeof(p->dest_host));

    priv->entry_ptr = priv->entry_ptr->next;

    return 1;
}

static int rtnet_ipv4_host_route_show(struct xnvfile_snapshot_iterator *it,
				    void *data)
{
    struct rtnet_ipv4_host_route_data *p = data;

    if (p == NULL) {
	xnvfile_printf(it, "Hash\tDestination\tHW Address\t\tDevice\n");
	return 0;
    }

    xnvfile_printf(it, "%02X\t%u.%u.%u.%-3u\t"
		"%02X:%02X:%02X:%02X:%02X:%02X\t%s\n",
		p->key, NIPQUAD(p->dest_host.ip),
		p->dest_host.dev_addr[0], p->dest_host.dev_addr[1],
		p->dest_host.dev_addr[2], p->dest_host.dev_addr[3],
		p->dest_host.dev_addr[4], p->dest_host.dev_addr[5],
		p->name);
    return 0;
}

static struct xnvfile_snapshot_ops rtnet_ipv4_host_route_vfile_ops = {
    .begin = rtnet_ipv4_host_route_begin,
    .end = rtnet_ipv4_host_route_end,
    .next = rtnet_ipv4_host_route_next,
    .show = rtnet_ipv4_host_route_show,
};

static struct xnvfile_snapshot rtnet_ipv4_host_route_vfile = {
    .entry = {
	.lockops = &rtnet_ipv4_host_route_lock_ops,
    },
    .privsz = sizeof(struct rtnet_ipv4_host_route_priv),
    .datasz = sizeof(struct rtnet_ipv4_host_route_data),
    .tag = &host_route_tag,
    .ops = &rtnet_ipv4_host_route_vfile_ops,
};

static struct xnvfile_link rtnet_ipv4_arp_vfile;

#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING
static rtdm_lockctx_t rtnet_ipv4_net_route_lock_ctx;

static int rtnet_ipv4_net_route_lock(struct xnvfile *vfile)
{
    rtdm_lock_get_irqsave(&net_table_lock, rtnet_ipv4_net_route_lock_ctx);
    return 0;
}

static void rtnet_ipv4_net_route_unlock(struct xnvfile *vfile)
{
    rtdm_lock_put_irqrestore(&net_table_lock, rtnet_ipv4_net_route_lock_ctx);
}

static struct xnvfile_lock_ops rtnet_ipv4_net_route_lock_ops = {
    .get = rtnet_ipv4_net_route_lock,
    .put = rtnet_ipv4_net_route_unlock,
};

struct rtnet_ipv4_net_route_priv {
    unsigned key;
    struct net_route *entry_ptr;
};

struct rtnet_ipv4_net_route_data {
    int key;
    u32 dest_net_ip;
    u32 dest_net_mask;
    u32 gw_ip;
};

struct xnvfile_rev_tag net_route_tag;

static void *rtnet_ipv4_net_route_begin(struct xnvfile_snapshot_iterator *it)
{
    struct rtnet_ipv4_net_route_priv *priv = xnvfile_iterator_priv(it);
    struct rtnet_ipv4_net_route_data *data;
    unsigned routes;
    int err;

    routes = allocated_net_routes;
    if (!routes)
	return VFILE_SEQ_EMPTY;

    data = kmalloc(sizeof(*data) * routes, GFP_KERNEL);
    if (data == NULL)
	return NULL;

    err = rtnet_ipv4_module_lock(NULL);
    if (err < 0) {
	kfree(data);
	return VFILE_SEQ_EMPTY;
    }

    priv->key = -1;
    priv->entry_ptr = NULL;
    return data;
}

static void rtnet_ipv4_net_route_end(struct xnvfile_snapshot_iterator *it,
				    void *buf)
{
    rtnet_ipv4_module_unlock(NULL);
    kfree(buf);
}

static int rtnet_ipv4_net_route_next(struct xnvfile_snapshot_iterator *it,
				    void *data)
{
    struct rtnet_ipv4_net_route_priv *priv = xnvfile_iterator_priv(it);
    struct rtnet_ipv4_net_route_data *p = data;

    if (priv->entry_ptr == NULL) {
	if (++priv->key >= NET_HASH_TBL_SIZE + 1)
	    return 0;

	priv->entry_ptr = net_hash_tbl[priv->key];
	if (priv->entry_ptr == NULL)
	    return VFILE_SEQ_SKIP;
    }

    p->key = priv->key;
    p->dest_net_ip = priv->entry_ptr->dest_net_ip;
    p->dest_net_mask = priv->entry_ptr->dest_net_mask;
    p->gw_ip = priv->entry_ptr->gw_ip;

    priv->entry_ptr = priv->entry_ptr->next;

    return 1;
}

static int rtnet_ipv4_net_route_show(struct xnvfile_snapshot_iterator *it,
				    void *data)
{
    struct rtnet_ipv4_net_route_data *p = data;

    if (p == NULL) {
	xnvfile_printf(it, "Hash\tDestination\tMask\t\t\tGateway\n");
	return 0;
    }

    if (p->key < NET_HASH_TBL_SIZE)
	xnvfile_printf(it, "%02X\t%u.%u.%u.%-3u\t%u.%u.%u.%-3u"
		    "\t\t%u.%u.%u.%-3u\n",
		    p->key, NIPQUAD(p->dest_net_ip),
		    NIPQUAD(p->dest_net_mask),
		    NIPQUAD(p->gw_ip));
    else
	xnvfile_printf(it, "*\t%u.%u.%u.%-3u\t%u.%u.%u.%-3u\t\t"
		    "%u.%u.%u.%-3u\n",
		    NIPQUAD(p->dest_net_ip),
		    NIPQUAD(p->dest_net_mask),
		    NIPQUAD(p->gw_ip));

    return 0;
}

static struct xnvfile_snapshot_ops rtnet_ipv4_net_route_vfile_ops = {
    .begin = rtnet_ipv4_net_route_begin,
    .end = rtnet_ipv4_net_route_end,
    .next = rtnet_ipv4_net_route_next,
    .show = rtnet_ipv4_net_route_show,
};

static struct xnvfile_snapshot rtnet_ipv4_net_route_vfile = {
    .entry = {
	.lockops = &rtnet_ipv4_net_route_lock_ops,
    },
    .privsz = sizeof(struct rtnet_ipv4_net_route_priv),
    .datasz = sizeof(struct rtnet_ipv4_net_route_data),
    .tag = &net_route_tag,
    .ops = &rtnet_ipv4_net_route_vfile_ops,
};
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING */



static int __init rt_route_proc_register(void)
{
    int err;

    err = xnvfile_init_regular("route",
			    &rtnet_ipv4_route_vfile, &ipv4_proc_root);
    if (err < 0)
	goto err1;

    err = xnvfile_init_snapshot("host_route",
				&rtnet_ipv4_host_route_vfile, &ipv4_proc_root);
    if (err < 0)
	goto err2;

    /* create "arp" as an alias for "host_route" */
    err = xnvfile_init_link("arp", "host_route",
			    &rtnet_ipv4_arp_vfile, &ipv4_proc_root);
    if (err < 0)
	goto err3;

#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING
    err = xnvfile_init_snapshot("net_route",
				&rtnet_ipv4_net_route_vfile, &ipv4_proc_root);
    if (err < 0)
	goto err4;
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING */

    return 0;

#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING
  err4:
    xnvfile_destroy_link(&rtnet_ipv4_arp_vfile);
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING */

  err3:
    xnvfile_destroy_snapshot(&rtnet_ipv4_host_route_vfile);

  err2:
    xnvfile_destroy_regular(&rtnet_ipv4_route_vfile);

  err1:
    printk("RTnet: unable to initialize /proc entries (route)\n");
    return err;
}



static void rt_route_proc_unregister(void)
{
#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING
    xnvfile_destroy_snapshot(&rtnet_ipv4_net_route_vfile);
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING */
    xnvfile_destroy_link(&rtnet_ipv4_arp_vfile);
    xnvfile_destroy_snapshot(&rtnet_ipv4_host_route_vfile);
    xnvfile_destroy_regular(&rtnet_ipv4_route_vfile);
}
#endif /* CONFIG_XENO_OPT_VFILE */



/***
 *  rt_alloc_host_route - allocates new host route
 */
static inline struct host_route *rt_alloc_host_route(void)
{
    rtdm_lockctx_t      context;
    struct host_route   *rt;


    rtdm_lock_get_irqsave(&host_table_lock, context);

    if ((rt = free_host_route) != NULL) {
	free_host_route = rt->next;
	allocated_host_routes++;
    }

    rtdm_lock_put_irqrestore(&host_table_lock, context);

    return rt;
}



/***
 *  rt_free_host_route - releases host route
 *
 *  Note: must be called with host_table_lock held
 */
static inline void rt_free_host_route(struct host_route *rt)
{
    rt->next        = free_host_route;
    free_host_route = rt;
    allocated_host_routes--;
}



/***
 *  rt_ip_route_add_host: add or update host route
 */
int rt_ip_route_add_host(u32 addr, unsigned char *dev_addr,
			 struct rtnet_device *rtdev)
{
    rtdm_lockctx_t      context;
    struct host_route   *new_route;
    struct host_route   *rt;
    unsigned int        key;
    int                 ret = 0;


    rtdm_lock_get_irqsave(&rtdev->rtdev_lock, context);

    if ((!test_bit(PRIV_FLAG_UP, &rtdev->priv_flags) ||
	test_and_set_bit(PRIV_FLAG_ADDING_ROUTE, &rtdev->priv_flags))) {
	rtdm_lock_put_irqrestore(&rtdev->rtdev_lock, context);
	return -EBUSY;
    }

    rtdm_lock_put_irqrestore(&rtdev->rtdev_lock, context);

    if ((new_route = rt_alloc_host_route()) != NULL) {
	new_route->dest_host.ip    = addr;
	new_route->dest_host.rtdev = rtdev;
	memcpy(new_route->dest_host.dev_addr, dev_addr, rtdev->addr_len);
    }

    key = ntohl(addr) & HOST_HASH_KEY_MASK;

    rtdm_lock_get_irqsave(&host_table_lock, context);

    xnvfile_touch_tag(&host_route_tag);

    rt = host_hash_tbl[key];
    while (rt != NULL) {
	if ((rt->dest_host.ip == addr) &&
	    (rt->dest_host.rtdev->local_ip == rtdev->local_ip)) {
	    rt->dest_host.rtdev = rtdev;
	    memcpy(rt->dest_host.dev_addr, dev_addr, rtdev->addr_len);

	    if (new_route)
		rt_free_host_route(new_route);

	    rtdm_lock_put_irqrestore(&host_table_lock, context);

	    goto out;
	}

	rt = rt->next;
    }

    if (new_route) {
	new_route->next    = host_hash_tbl[key];
	host_hash_tbl[key] = new_route;

	rtdm_lock_put_irqrestore(&host_table_lock, context);
    } else {
	rtdm_lock_put_irqrestore(&host_table_lock, context);

	/*ERRMSG*/rtdm_printk("RTnet: no more host routes available\n");
	ret = -ENOBUFS;
    }

  out:
    clear_bit(PRIV_FLAG_ADDING_ROUTE, &rtdev->priv_flags);

    return ret;
}



/***
 *  rt_ip_route_del_host - deletes specified host route
 */
int rt_ip_route_del_host(u32 addr, struct rtnet_device *rtdev)
{
    rtdm_lockctx_t      context;
    struct host_route   *rt;
    struct host_route   **last_ptr;
    unsigned int        key;


    key = ntohl(addr) & HOST_HASH_KEY_MASK;
    last_ptr = &host_hash_tbl[key];

    rtdm_lock_get_irqsave(&host_table_lock, context);

    rt = host_hash_tbl[key];
    while (rt != NULL) {
	if ((rt->dest_host.ip == addr) &&
	    (!rtdev || (rt->dest_host.rtdev->local_ip == rtdev->local_ip))) {
	    *last_ptr = rt->next;

	    rt_free_host_route(rt);

	    xnvfile_touch_tag(&host_route_tag);

	    rtdm_lock_put_irqrestore(&host_table_lock, context);

	    return 0;
	}

	last_ptr = &rt->next;
	rt = rt->next;
    }

    rtdm_lock_put_irqrestore(&host_table_lock, context);

    return -ENOENT;
}



/***
 *  rt_ip_route_del_all - deletes all routes associated with a specified device
 */
void rt_ip_route_del_all(struct rtnet_device *rtdev)
{
    rtdm_lockctx_t      context;
    struct host_route   *host_rt;
    struct host_route   **last_host_ptr;
    unsigned int        key;
    u32                 ip;


    for (key = 0; key < HOST_HASH_TBL_SIZE; key++) {
      host_start_over:
	last_host_ptr = &host_hash_tbl[key];

	rtdm_lock_get_irqsave(&host_table_lock, context);

	host_rt = host_hash_tbl[key];
	while (host_rt != NULL) {
	    if (host_rt->dest_host.rtdev == rtdev) {
		*last_host_ptr = host_rt->next;

		rt_free_host_route(host_rt);

		rtdm_lock_put_irqrestore(&host_table_lock, context);

		goto host_start_over;
	    }

	    last_host_ptr = &host_rt->next;
	    host_rt = host_rt->next;
	}

	rtdm_lock_put_irqrestore(&host_table_lock, context);
    }

    if ((ip = rtdev->local_ip) != 0)
	rt_ip_route_del_host(ip, rtdev);
}


/***
 *  rt_ip_route_get_host - check if specified host route is resolved
 */
int rt_ip_route_get_host(u32 addr, char *if_name, unsigned char *dev_addr,
			 struct rtnet_device *rtdev)
{
    rtdm_lockctx_t      context;
    struct host_route   *rt;
    unsigned int        key;


    key = ntohl(addr) & HOST_HASH_KEY_MASK;

    rtdm_lock_get_irqsave(&host_table_lock, context);

    rt = host_hash_tbl[key];
    while (rt != NULL) {
	if ((rt->dest_host.ip == addr) &&
	    (!rtdev || rt->dest_host.rtdev->local_ip == rtdev->local_ip)) {
	    memcpy(dev_addr, rt->dest_host.dev_addr,
		   rt->dest_host.rtdev->addr_len);
	    strncpy(if_name, rt->dest_host.rtdev->name, IFNAMSIZ);

	    rtdm_lock_put_irqrestore(&host_table_lock, context);
	    return 0;
	}

	rt = rt->next;
    }

    rtdm_lock_put_irqrestore(&host_table_lock, context);

    return -ENOENT;
}


#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING
/***
 *  rt_alloc_net_route - allocates new network route
 */
static inline struct net_route *rt_alloc_net_route(void)
{
    rtdm_lockctx_t      context;
    struct net_route    *rt;


    rtdm_lock_get_irqsave(&net_table_lock, context);

    if ((rt = free_net_route) != NULL) {
	free_net_route = rt->next;
	allocated_net_routes++;
    }

    rtdm_lock_put_irqrestore(&net_table_lock, context);

    return rt;
}



/***
 *  rt_free_net_route - releases network route
 *
 *  Note: must be called with net_table_lock held
 */
static inline void rt_free_net_route(struct net_route *rt)
{
    rt->next       = free_net_route;
    free_net_route = rt;
    allocated_host_routes--;
}



/***
 *  rt_ip_route_add_net: add or update network route
 */
int rt_ip_route_add_net(u32 addr, u32 mask, u32 gw_addr)
{
    rtdm_lockctx_t      context;
    struct net_route    *new_route;
    struct net_route    *rt;
    struct net_route    **last_ptr;
    unsigned int        key;
    u32                 shifted_mask;


    addr &= mask;

    if ((new_route = rt_alloc_net_route()) != NULL) {
	new_route->dest_net_ip   = addr;
	new_route->dest_net_mask = mask;
	new_route->gw_ip         = gw_addr;
    }

    shifted_mask = NET_HASH_KEY_MASK << net_hash_key_shift;
    if ((mask & shifted_mask) == shifted_mask)
	key = (ntohl(addr) >> net_hash_key_shift) & NET_HASH_KEY_MASK;
    else
	key = NET_HASH_TBL_SIZE;
    last_ptr = &net_hash_tbl[key];

    rtdm_lock_get_irqsave(&net_table_lock, context);

    xnvfile_touch_tag(&net_route_tag);

    rt = net_hash_tbl[key];
    while (rt != NULL) {
	if ((rt->dest_net_ip == addr) && (rt->dest_net_mask == mask)) {
	    rt->gw_ip = gw_addr;

	    if (new_route)
		rt_free_net_route(new_route);

	    rtdm_lock_put_irqrestore(&net_table_lock, context);

	    return 0;
	}

	last_ptr = &rt->next;
	rt = rt->next;
    }

    if (new_route) {
	new_route->next = *last_ptr;
	*last_ptr       = new_route;

	rtdm_lock_put_irqrestore(&net_table_lock, context);

	return 0;
    } else {
	rtdm_lock_put_irqrestore(&net_table_lock, context);

	/*ERRMSG*/rtdm_printk("RTnet: no more network routes available\n");
	return -ENOBUFS;
    }
}



/***
 *  rt_ip_route_del_net - deletes specified network route
 */
int rt_ip_route_del_net(u32 addr, u32 mask)
{
    rtdm_lockctx_t      context;
    struct net_route    *rt;
    struct net_route    **last_ptr;
    unsigned int        key;
    u32                 shifted_mask;


    addr &= mask;

    shifted_mask = NET_HASH_KEY_MASK << net_hash_key_shift;
    if ((mask & shifted_mask) == shifted_mask)
	key = (ntohl(addr) >> net_hash_key_shift) & NET_HASH_KEY_MASK;
    else
	key = NET_HASH_TBL_SIZE;
    last_ptr = &net_hash_tbl[key];

    rtdm_lock_get_irqsave(&net_table_lock, context);

    rt = net_hash_tbl[key];
    while (rt != NULL) {
	if ((rt->dest_net_ip == addr) && (rt->dest_net_mask == mask)) {
	    *last_ptr = rt->next;

	    rt_free_net_route(rt);

	    xnvfile_touch_tag(&net_route_tag);

	    rtdm_lock_put_irqrestore(&net_table_lock, context);

	    return 0;
	}

	last_ptr = &rt->next;
	rt = rt->next;
    }

    rtdm_lock_put_irqrestore(&net_table_lock, context);

    return -ENOENT;
}
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING */



/***
 *  rt_ip_route_output - looks up output route
 *
 *  Note: increments refcount on returned rtdev in rt_buf
 */
int rt_ip_route_output(struct dest_route *rt_buf, u32 daddr, u32 saddr)
{
    rtdm_lockctx_t      context;
    struct host_route   *host_rt;
    unsigned int        key;

#ifndef CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING
    #define DADDR       daddr
#else
    #define DADDR       real_daddr

    struct net_route    *net_rt;
    int                 lookup_gw  = 1;
    u32                 real_daddr = daddr;


  restart:
#endif /* !CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING */

    key = ntohl(daddr) & HOST_HASH_KEY_MASK;

    rtdm_lock_get_irqsave(&host_table_lock, context);

    host_rt = host_hash_tbl[key];
    if (likely(saddr == INADDR_ANY))
	while (host_rt != NULL) {
	    if (host_rt->dest_host.ip == daddr) {
	      host_route_found:
		if (!rtdev_reference(host_rt->dest_host.rtdev)) {
		    rtdm_lock_put_irqrestore(&host_table_lock, context);
		    goto next;
		}

		memcpy(rt_buf->dev_addr, &host_rt->dest_host.dev_addr,
		       sizeof(rt_buf->dev_addr));
		rt_buf->rtdev = host_rt->dest_host.rtdev;

		rtdm_lock_put_irqrestore(&host_table_lock, context);

		rt_buf->ip = DADDR;

		return 0;
	    }
	  next:
	    host_rt = host_rt->next;
	}
    else
	while (host_rt != NULL) {
	    if ((host_rt->dest_host.ip == daddr) &&
		(host_rt->dest_host.rtdev->local_ip == saddr))
		goto host_route_found;
	    host_rt = host_rt->next;
	}

    rtdm_lock_put_irqrestore(&host_table_lock, context);

#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING
    if (lookup_gw) {
	lookup_gw = 0;
	key = (ntohl(daddr) >> net_hash_key_shift) & NET_HASH_KEY_MASK;

	rtdm_lock_get_irqsave(&net_table_lock, context);

	net_rt = net_hash_tbl[key];
	while (net_rt != NULL) {
	    if (net_rt->dest_net_ip == (daddr & net_rt->dest_net_mask)) {
		daddr = net_rt->gw_ip;

		rtdm_lock_put_irqrestore(&net_table_lock, context);

		/* start over, now using the gateway ip as destination */
		goto restart;
	    }

	    net_rt = net_rt->next;
	}

	rtdm_lock_put_irqrestore(&net_table_lock, context);

	/* last try: no hash key */
	rtdm_lock_get_irqsave(&net_table_lock, context);

	net_rt = net_hash_tbl[NET_HASH_TBL_SIZE];
	while (net_rt != NULL) {
	    if (net_rt->dest_net_ip == (daddr & net_rt->dest_net_mask)) {
		daddr = net_rt->gw_ip;

		rtdm_lock_put_irqrestore(&net_table_lock, context);

		/* start over, now using the gateway ip as destination */
		goto restart;
	    }

	    net_rt = net_rt->next;
	}

	rtdm_lock_put_irqrestore(&net_table_lock, context);
    }
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING */

    /*ERRMSG*/rtdm_printk("RTnet: host %u.%u.%u.%u unreachable\n", NIPQUAD(daddr));
    return -EHOSTUNREACH;
}



#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_ROUTER
int rt_ip_route_forward(struct rtskb *rtskb, u32 daddr)
{
    struct rtnet_device *rtdev = rtskb->rtdev;
    struct dest_route   dest;


    if (likely((daddr == rtdev->local_ip) || (daddr == rtdev->broadcast_ip) ||
	(rtdev->flags & IFF_LOOPBACK)))
	return 0;

    if (rtskb_acquire(rtskb, &global_pool) != 0) {
	/*ERRMSG*/rtdm_printk("RTnet: router overloaded, dropping packet\n");
	goto error;
    }

    if (rt_ip_route_output(&dest, daddr, INADDR_ANY) < 0) {
	/*ERRMSG*/rtdm_printk("RTnet: unable to forward packet from %u.%u.%u.%u\n",
			      NIPQUAD(rtskb->nh.iph->saddr));
	goto error;
    }

    rtskb->rtdev    = dest.rtdev;
    rtskb->priority = ROUTER_FORWARD_PRIO;

    if ((dest.rtdev->hard_header) &&
	(dest.rtdev->hard_header(rtskb, dest.rtdev, ETH_P_IP, dest.dev_addr,
				 dest.rtdev->dev_addr, rtskb->len) < 0))
	goto error;

    rtdev_xmit(rtskb);

    return 1;

  error:
    kfree_rtskb(rtskb);
    return 1;
}
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_ROUTER */



/***
 *  rt_ip_routing_init: initialize
 */
int __init rt_ip_routing_init(void)
{
    int i;


    for (i = 0; i < CONFIG_XENO_DRIVERS_NET_RTIPV4_HOST_ROUTES-2; i++)
	host_routes[i].next = &host_routes[i+1];
    free_host_route = &host_routes[0];

#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING
    for (i = 0; i < CONFIG_XENO_DRIVERS_NET_RTIPV4_NET_ROUTES-2; i++)
	net_routes[i].next = &net_routes[i+1];
    free_net_route = &net_routes[0];
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING */

#ifdef CONFIG_XENO_OPT_VFILE
    return rt_route_proc_register();
#else /* !CONFIG_XENO_OPT_VFILE */
    return 0;
#endif /* CONFIG_XENO_OPT_VFILE */
}



/***
 *  rt_ip_routing_realease
 */
void rt_ip_routing_release(void)
{
#ifdef CONFIG_XENO_OPT_VFILE
    rt_route_proc_unregister();
#endif /* CONFIG_XENO_OPT_VFILE */
}


EXPORT_SYMBOL_GPL(rt_ip_route_add_host);
EXPORT_SYMBOL_GPL(rt_ip_route_del_host);
EXPORT_SYMBOL_GPL(rt_ip_route_del_all);
EXPORT_SYMBOL_GPL(rt_ip_route_output);
