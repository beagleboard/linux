/***
 *
 *  stack/rtnet_module.c - module framework, proc file system
 *
 *  Copyright (C) 2002      Ulrich Marx <marx@kammer.uni-hannover.de>
 *                2003-2006 Jan Kiszka <jan.kiszka@web.de>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <rtdev_mgr.h>
#include <rtnet_chrdev.h>
#include <rtnet_internal.h>
#include <rtnet_socket.h>
#include <rtnet_rtpc.h>
#include <stack_mgr.h>
#include <rtwlan.h>

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RTnet stack core");


struct rtnet_mgr STACK_manager;
struct rtnet_mgr RTDEV_manager;

EXPORT_SYMBOL_GPL(STACK_manager);
EXPORT_SYMBOL_GPL(RTDEV_manager);

const char rtnet_rtdm_provider_name[] =
    "(C) 1999-2008 RTnet Development Team, http://www.rtnet.org";

EXPORT_SYMBOL_GPL(rtnet_rtdm_provider_name);


#ifdef CONFIG_XENO_OPT_VFILE
/***
 *      proc filesystem section
 */
struct xnvfile_directory rtnet_proc_root;
EXPORT_SYMBOL_GPL(rtnet_proc_root);


static int rtnet_devices_nrt_lock_get(struct xnvfile *vfile)
{
	return mutex_lock_interruptible(&rtnet_devices_nrt_lock);
}

static void rtnet_devices_nrt_lock_put(struct xnvfile *vfile)
{
	mutex_unlock(&rtnet_devices_nrt_lock);
}

static struct xnvfile_lock_ops rtnet_devices_nrt_lock_ops = {
	.get = rtnet_devices_nrt_lock_get,
	.put = rtnet_devices_nrt_lock_put,
};

static void *rtnet_devices_begin(struct xnvfile_regular_iterator *it)
{
	if (it->pos == 0)
		return VFILE_SEQ_START;

	return (void *)2UL;
}

static void *rtnet_devices_next(struct xnvfile_regular_iterator *it)
{
	if (it->pos >= MAX_RT_DEVICES)
		return NULL;

	return (void *)2UL;
}

static int rtnet_devices_show(struct xnvfile_regular_iterator *it, void *data)
{
	struct rtnet_device *rtdev;

	if (data == NULL) {
	    xnvfile_printf(it, "Index\tName\t\tFlags\n");
		return 0;
	}

	rtdev = __rtdev_get_by_index(it->pos);
	if (rtdev == NULL)
		return VFILE_SEQ_SKIP;

	xnvfile_printf(it, "%d\t%-15s %s%s%s%s\n",
				rtdev->ifindex, rtdev->name,
				(rtdev->flags & IFF_UP) ? "UP" : "DOWN",
				(rtdev->flags & IFF_BROADCAST) ? " BROADCAST" : "",
				(rtdev->flags & IFF_LOOPBACK) ? " LOOPBACK" : "",
				(rtdev->flags & IFF_PROMISC) ? " PROMISC" : "");
	return 0;
}

static struct xnvfile_regular_ops rtnet_devices_vfile_ops = {
	.begin = rtnet_devices_begin,
	.next = rtnet_devices_next,
	.show = rtnet_devices_show,
};

static struct xnvfile_regular rtnet_devices_vfile = {
	.entry = { .lockops = &rtnet_devices_nrt_lock_ops, },
	.ops = &rtnet_devices_vfile_ops,
};

static int rtnet_rtskb_show(struct xnvfile_regular_iterator *it, void *data)
{
    unsigned int rtskb_len;

    rtskb_len = ALIGN_RTSKB_STRUCT_LEN + SKB_DATA_ALIGN(RTSKB_SIZE);

    xnvfile_printf(it, "Statistics\t\tCurrent\tMaximum\n"
		     "rtskb pools\t\t%d\t%d\n"
		     "rtskbs\t\t\t%d\t%d\n"
		     "rtskb memory need\t%d\t%d\n",
		     rtskb_pools, rtskb_pools_max,
		     rtskb_amount, rtskb_amount_max,
		     rtskb_amount * rtskb_len, rtskb_amount_max * rtskb_len);
	return 0;
}

static struct xnvfile_regular_ops rtnet_rtskb_vfile_ops = {
	.show = rtnet_rtskb_show,
};

static struct xnvfile_regular rtnet_rtskb_vfile = {
	.ops = &rtnet_rtskb_vfile_ops,
};

static int rtnet_version_show(struct xnvfile_regular_iterator *it, void *data)
{
    const char verstr[] =
	    "RTnet for Xenomai v" XENO_VERSION_STRING "\n"
		"RTcap:      "
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_RTCAP)
	    "yes\n"
#else
	    "no\n"
#endif
		"rtnetproxy: "
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_PROXY)
	    "yes\n"
#else
	    "no\n"
#endif
		"bug checks: "
#ifdef CONFIG_XENO_DRIVERS_NET_CHECKED
	    "yes\n"
#else
	    "no\n"
#endif
		;

	xnvfile_printf(it, "%s", verstr);

	return 0;
}

static struct xnvfile_regular_ops rtnet_version_vfile_ops = {
	.show = rtnet_version_show,
};

static struct xnvfile_regular rtnet_version_vfile = {
	.ops = &rtnet_version_vfile_ops,
};

static void *rtnet_stats_begin(struct xnvfile_regular_iterator *it)
{
	return (void *)1UL;
}

static void *rtnet_stats_next(struct xnvfile_regular_iterator *it)
{
	if (it->pos >= MAX_RT_DEVICES)
		return NULL;

	return (void *)1UL;
}

static int rtnet_stats_show(struct xnvfile_regular_iterator *it, void *data)
{
	struct net_device_stats *stats;
	struct rtnet_device *rtdev;

	if (it->pos == 0) {
		xnvfile_printf(it, "Inter-|   Receive                            "
					"                    |  Transmit\n");
		xnvfile_printf(it, " face |bytes    packets errs drop fifo frame "
					"compressed multicast|bytes    packets errs "
					"drop fifo colls carrier compressed\n");
		return 0;
	}

	rtdev = __rtdev_get_by_index(it->pos);
	if (rtdev == NULL)
		return VFILE_SEQ_SKIP;

	if (rtdev->get_stats == NULL) {
		xnvfile_printf(it, "%6s: No statistics available.\n", rtdev->name);
		return 0;
	}

	stats = rtdev->get_stats(rtdev);
	xnvfile_printf(it,
				"%6s:%8lu %7lu %4lu %4lu %4lu %5lu %10lu %9lu "
				"%8lu %7lu %4lu %4lu %4lu %5lu %7lu %10lu\n",
				rtdev->name, stats->rx_bytes, stats->rx_packets,
				stats->rx_errors,
				stats->rx_dropped + stats->rx_missed_errors,
				stats->rx_fifo_errors,
				stats->rx_length_errors + stats->rx_over_errors +
				stats->rx_crc_errors + stats->rx_frame_errors,
				stats->rx_compressed, stats->multicast,
				stats->tx_bytes, stats->tx_packets,
				stats->tx_errors, stats->tx_dropped,
				stats->tx_fifo_errors, stats->collisions,
				stats->tx_carrier_errors +
				stats->tx_aborted_errors +
				stats->tx_window_errors +
				stats->tx_heartbeat_errors,
				stats->tx_compressed);
	return 0;
}

static struct xnvfile_regular_ops rtnet_stats_vfile_ops = {
	.begin = rtnet_stats_begin,
	.next = rtnet_stats_next,
	.show = rtnet_stats_show,
};

static struct xnvfile_regular rtnet_stats_vfile = {
	.entry = { .lockops = &rtnet_devices_nrt_lock_ops, },
	.ops = &rtnet_stats_vfile_ops,
};

static int rtnet_proc_register(void)
{
	int err;

	err = xnvfile_init_dir("rtnet", &rtnet_proc_root, NULL);
	if (err < 0)
		goto error1;

	err = xnvfile_init_regular("devices", &rtnet_devices_vfile, &rtnet_proc_root);
	if (err < 0)
		goto error2;

	err = xnvfile_init_regular("rtskb", &rtnet_rtskb_vfile, &rtnet_proc_root);
	if (err < 0)
		goto error3;

	err = xnvfile_init_regular("version", &rtnet_version_vfile, &rtnet_proc_root);
	if (err < 0)
		goto error4;

	err = xnvfile_init_regular("stats", &rtnet_stats_vfile, &rtnet_proc_root);
	if (err < 0)
		goto error5;

    return 0;

  error5:
	xnvfile_destroy_regular(&rtnet_version_vfile);

  error4:
	xnvfile_destroy_regular(&rtnet_rtskb_vfile);

  error3:
	xnvfile_destroy_regular(&rtnet_devices_vfile);

  error2:
	xnvfile_destroy_dir(&rtnet_proc_root);

  error1:
    printk("RTnet: unable to initialize /proc entries\n");
    return err;
}



static void rtnet_proc_unregister(void)
{
	xnvfile_destroy_regular(&rtnet_stats_vfile);
	xnvfile_destroy_regular(&rtnet_version_vfile);
	xnvfile_destroy_regular(&rtnet_rtskb_vfile);
	xnvfile_destroy_regular(&rtnet_devices_vfile);
	xnvfile_destroy_dir(&rtnet_proc_root);
}
#endif  /* CONFIG_XENO_OPT_VFILE */



/**
 *  rtnet_init()
 */
int __init rtnet_init(void)
{
    int err = 0;


    printk("\n*** RTnet for Xenomai v" XENO_VERSION_STRING " ***\n\n");
    printk("RTnet: initialising real-time networking\n");

    if ((err = rtskb_pools_init()) != 0)
	goto err_out1;

#ifdef CONFIG_XENO_OPT_VFILE
    if ((err = rtnet_proc_register()) != 0)
	goto err_out2;
#endif

    /* initialize the Stack-Manager */
    if ((err = rt_stack_mgr_init(&STACK_manager)) != 0)
	goto err_out3;

    /* initialize the RTDEV-Manager */
    if ((err = rt_rtdev_mgr_init(&RTDEV_manager)) != 0)
	goto err_out4;

    rtnet_chrdev_init();

    if ((err = rtwlan_init()) != 0)
	goto err_out5;

    if ((err = rtpc_init()) != 0)
	goto err_out6;

    return 0;


err_out6:
    rtwlan_exit();

err_out5:
    rtnet_chrdev_release();
    rt_rtdev_mgr_delete(&RTDEV_manager);

err_out4:
    rt_stack_mgr_delete(&STACK_manager);

err_out3:
#ifdef CONFIG_XENO_OPT_VFILE
    rtnet_proc_unregister();

err_out2:
#endif
    rtskb_pools_release();

err_out1:
    return err;
}


/**
 *  rtnet_release()
 */
void __exit rtnet_release(void)
{
    rtpc_cleanup();

    rtwlan_exit();

    rtnet_chrdev_release();

    rt_stack_mgr_delete(&STACK_manager);
    rt_rtdev_mgr_delete(&RTDEV_manager);

    rtskb_pools_release();

#ifdef CONFIG_XENO_OPT_VFILE
    rtnet_proc_unregister();
#endif

    printk("RTnet: unloaded\n");
}


module_init(rtnet_init);
module_exit(rtnet_release);
