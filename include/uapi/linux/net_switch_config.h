// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments Ethernet Switch Driver
 *
 * Copyright (C) 2014-2018 Texas Instruments, Inc
 *
 * Userspace API for Switch Configuration
 */

#ifndef __NET_CONFIG_SWITCH_H__
#define __NET_CONFIG_SWITCH_H__

enum {
	CONFIG_SWITCH_INVALID,
	CONFIG_SWITCH_ADD_MULTICAST,
	CONFIG_SWITCH_DEL_MULTICAST,
	CONFIG_SWITCH_ADD_VLAN,
	CONFIG_SWITCH_DEL_VLAN,
	CONFIG_SWITCH_SET_PORT_CONFIG,
	CONFIG_SWITCH_GET_PORT_CONFIG,
	CONFIG_SWITCH_ADD_UNKNOWN_VLAN_INFO,
	CONFIG_SWITCH_GET_PORT_STATE,
	CONFIG_SWITCH_SET_PORT_STATE,
	CONFIG_SWITCH_GET_PORT_VLAN_CONFIG,
	CONFIG_SWITCH_SET_PORT_VLAN_CONFIG,
	CONFIG_SWITCH_RATELIMIT,
};

enum {
	PORT_STATE_DISABLED = 0,
	PORT_STATE_BLOCKED,
	PORT_STATE_LEARN,
	PORT_STATE_FORWARD,
};

/*
 * Pass all unused parameters as zero is recomented.
 */
struct net_switch_config {
	unsigned int cmd;	/* API to be invoked by the kernel driver */

	unsigned int	port;
	unsigned int	vid;		/* VLAN identifier */
	unsigned char	unreg_multi;	/* unreg multicast Egress Ports */
	unsigned char	reg_multi;	/* register multicast Egress ports */
	unsigned char	untag_port;	/* Untag ports */
	unsigned char	addr[6];
	unsigned int	super;
	struct ethtool_cmd ecmd;
	unsigned char	unknown_vlan_member;
	unsigned char	unknown_vlan_untag;
	unsigned int	unknown_vlan_unreg_multi;
	unsigned int	unknown_vlan_reg_multi;
	unsigned int	port_state;
	unsigned int	prio;
	bool		vlan_cfi;
	unsigned int	bcast_rate_limit;
	unsigned int	mcast_rate_limit;
	bool		direction;

	unsigned int ret_type;   /* Return  Success/Failure */
};

#endif /* __NET_CONFIG_SWITCH_H__*/
