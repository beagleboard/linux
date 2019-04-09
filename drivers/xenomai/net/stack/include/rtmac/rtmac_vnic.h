/* include/rtmac/rtmac_vnic.h
 *
 * rtmac - real-time networking media access control subsystem
 * Copyright (C) 2002 Marc Kleine-Budde <kleine-budde@gmx.de>,
 *               2003 Jan Kiszka <Jan.Kiszka@web.de>
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

#ifndef __RTMAC_VNIC_H_
#define __RTMAC_VNIC_H_

#ifdef __KERNEL__

#include <linux/init.h>
#include <linux/netdevice.h>

#include <rtmac/rtmac_disc.h>

#define DEFAULT_VNIC_RTSKBS     32


int rtmac_vnic_rx(struct rtskb *skb, u16 type);

int rtmac_vnic_xmit(struct sk_buff *skb, struct net_device *dev);

void rtmac_vnic_set_max_mtu(struct rtnet_device *rtdev, unsigned int max_mtu);

int rtmac_vnic_add(struct rtnet_device *rtdev, vnic_xmit_handler vnic_xmit);
int rtmac_vnic_unregister(struct rtnet_device *rtdev);

static inline void rtmac_vnic_cleanup(struct rtnet_device *rtdev)
{
    struct rtmac_priv   *mac_priv = rtdev->mac_priv;

    rtskb_pool_release(&mac_priv->vnic_skb_pool);
}

#ifdef CONFIG_XENO_OPT_VFILE
int rtnet_rtmac_vnics_show(struct xnvfile_regular_iterator *it, void *data);
#endif

int __init rtmac_vnic_module_init(void);
void rtmac_vnic_module_cleanup(void);


#endif /* __KERNEL__ */

#endif /* __RTMAC_VNIC_H_ */
