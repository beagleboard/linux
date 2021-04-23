/*
 * Copyright (C) 2016 Gilles Chanteperdrix <gch@xenomai.org>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <asm/xenomai/syscall.h>
#include <xenomai/posix/corectl.h>

static int rtnet_corectl_call(struct notifier_block *self, unsigned long arg,
			      void *cookie)
{
	struct cobalt_config_vector *vec = cookie;
	int ret = 0;

	if (arg != _CC_COBALT_GET_NET_CONFIG)
		return NOTIFY_DONE;

	if (vec->u_bufsz < sizeof(ret))
		return notifier_from_errno(-EINVAL);

	if (IS_ENABLED(CONFIG_XENO_DRIVERS_NET))
		ret |= _CC_COBALT_NET;
	if (IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ETH_P_ALL))
		ret |= _CC_COBALT_NET_ETH_P_ALL;
	if (IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4))
		ret |= _CC_COBALT_NET_IPV4;
	if (IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4_ICMP))
		ret |= _CC_COBALT_NET_ICMP;
	if (IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4_NETROUTING))
		ret |= _CC_COBALT_NET_NETROUTING;
	if (IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4_ROUTE))
		ret |= _CC_COBALT_NET_ROUTER;
	if (IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4_UDP))
		ret |= _CC_COBALT_NET_UDP;
	if (IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTPACKET))
		ret |= _CC_COBALT_NET_AF_PACKET;
	if (IS_ENABLED(CONFIG_XENO_DRIVERS_NET_TDMA))
		ret |= _CC_COBALT_NET_TDMA;
	if (IS_ENABLED(CONFIG_XENO_DRIVERS_NET_NOMAC))
		ret |= _CC_COBALT_NET_NOMAC;
	if (IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTCFG))
		ret |= _CC_COBALT_NET_CFG;
	if (IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_RTCAP))
		ret |= _CC_COBALT_NET_CAP;
	if (IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_PROXY))
		ret |= _CC_COBALT_NET_PROXY;

	ret = cobalt_copy_to_user(vec->u_buf, &ret, sizeof(ret));

	return ret ? notifier_from_errno(-EFAULT) : NOTIFY_STOP;
}

static struct notifier_block rtnet_corectl_notifier = {
	.notifier_call = rtnet_corectl_call,
};

void rtnet_corectl_register(void)
{
	cobalt_add_config_chain(&rtnet_corectl_notifier);
}

void rtnet_corectl_unregister(void)
{
	cobalt_remove_config_chain(&rtnet_corectl_notifier);
}
