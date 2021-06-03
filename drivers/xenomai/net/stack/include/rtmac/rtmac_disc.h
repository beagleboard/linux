/***
 *
 *  include/rtmac/rtmac_disc.h
 *
 *  rtmac - real-time networking media access control subsystem
 *  Copyright (C) 2002 Marc Kleine-Budde <kleine-budde@gmx.de>,
 *                2003, 2004 Jan Kiszka <Jan.Kiszka@web.de>
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

#ifndef __RTMAC_DISC_H_
#define __RTMAC_DISC_H_

#include <linux/list.h>
#include <linux/netdevice.h>

#include <rtdev.h>
#include <rtnet_chrdev.h>

#define RTMAC_NO_VNIC NULL
#define RTMAC_DEFAULT_VNIC rtmac_vnic_xmit

typedef int (*vnic_xmit_handler)(struct sk_buff *skb, struct net_device *dev);

struct rtmac_priv {
	int (*orig_start_xmit)(struct rtskb *skb, struct rtnet_device *dev);
	struct net_device *vnic;
	struct net_device_stats vnic_stats;
	struct rtskb_pool vnic_skb_pool;
	unsigned int vnic_max_mtu;

	u8 disc_priv[0] __attribute__((aligned(16)));
};

struct rtmac_proc_entry {
	const char *name;
	int (*handler)(struct xnvfile_regular_iterator *it, void *data);
	struct xnvfile_regular vfile;
};

struct rtmac_disc {
	struct list_head list;

	const char *name;
	unsigned int priv_size; /* size of rtmac_priv.disc_priv */
	u16 disc_type;

	int (*packet_rx)(struct rtskb *skb);
	/* rt_packet_tx prototype must be compatible with hard_start_xmit */
	int (*rt_packet_tx)(struct rtskb *skb, struct rtnet_device *dev);
	int (*nrt_packet_tx)(struct rtskb *skb);

	unsigned int (*get_mtu)(struct rtnet_device *rtdev,
				unsigned int priority);

	vnic_xmit_handler vnic_xmit;

	int (*attach)(struct rtnet_device *rtdev, void *disc_priv);
	int (*detach)(struct rtnet_device *rtdev, void *disc_priv);

	struct rtnet_ioctls ioctls;

	struct rtmac_proc_entry *proc_entries;
	unsigned nr_proc_entries;

	struct module *owner;
};

int rtmac_disc_attach(struct rtnet_device *rtdev, struct rtmac_disc *disc);
int rtmac_disc_detach(struct rtnet_device *rtdev);

int __rtmac_disc_register(struct rtmac_disc *disc, struct module *module);
#define rtmac_disc_register(disc) __rtmac_disc_register(disc, THIS_MODULE)

void rtmac_disc_deregister(struct rtmac_disc *disc);

#ifdef CONFIG_XENO_OPT_VFILE
int rtnet_rtmac_disciplines_show(struct xnvfile_regular_iterator *it, void *d);
#endif /* CONFIG_XENO_OPT_VFILE */

#endif /* __RTMAC_DISC_H_ */
