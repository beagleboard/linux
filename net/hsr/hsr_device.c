// SPDX-License-Identifier: GPL-2.0
/* Copyright 2011-2014 Autronica Fire and Security AS
 *
 * Author(s):
 *	2011-2014 Arvid Brodin, arvid.brodin@alten.se
 * This file contains device methods for creating, using and destroying
 * virtual HSR or PRP devices.
 */

#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <linux/pkt_sched.h>
#include "hsr_device.h"
#include "hsr_slave.h"
#include "hsr_framereg.h"
#include "hsr_main.h"
#include "hsr_forward.h"

static bool is_admin_up(struct net_device *dev)
{
	return dev && (dev->flags & IFF_UP);
}

static bool is_slave_up(struct net_device *dev)
{
	return dev && is_admin_up(dev) && netif_oper_up(dev);
}

static void __hsr_set_operstate(struct net_device *dev, int transition)
{
	write_lock_bh(&dev_base_lock);
	if (dev->operstate != transition) {
		dev->operstate = transition;
		write_unlock_bh(&dev_base_lock);
		netdev_state_change(dev);
	} else {
		write_unlock_bh(&dev_base_lock);
	}
}

static void hsr_set_operstate(struct hsr_port *master, bool has_carrier)
{
	if (!is_admin_up(master->dev)) {
		__hsr_set_operstate(master->dev, IF_OPER_DOWN);
		return;
	}

	if (has_carrier)
		__hsr_set_operstate(master->dev, IF_OPER_UP);
	else
		__hsr_set_operstate(master->dev, IF_OPER_LOWERLAYERDOWN);
}

static bool hsr_check_carrier(struct hsr_port *master)
{
	struct hsr_port *port;
	bool has_carrier;

	has_carrier = false;

	rcu_read_lock();
	hsr_for_each_port(master->hsr, port)
		if (port->type != HSR_PT_MASTER && is_slave_up(port->dev)) {
			has_carrier = true;
			break;
		}
	rcu_read_unlock();

	if (has_carrier)
		netif_carrier_on(master->dev);
	else
		netif_carrier_off(master->dev);

	return has_carrier;
}

static void hsr_check_announce(struct net_device *hsr_dev,
			       unsigned char old_operstate)
{
	struct hsr_priv *hsr;

	hsr = netdev_priv(hsr_dev);

	if (hsr_dev->operstate == IF_OPER_UP && old_operstate != IF_OPER_UP) {
		/* Went up */
		hsr->announce_count = 0;
		mod_timer(&hsr->announce_timer,
			  jiffies + msecs_to_jiffies(HSR_ANNOUNCE_INTERVAL));
	}

	if (hsr_dev->operstate != IF_OPER_UP && old_operstate == IF_OPER_UP)
		/* Went down */
		del_timer(&hsr->announce_timer);
}

void hsr_check_carrier_and_operstate(struct hsr_priv *hsr)
{
	struct hsr_port *master;
	unsigned char old_operstate;
	bool has_carrier;

	master = hsr_port_get_hsr(hsr, HSR_PT_MASTER);
	/* netif_stacked_transfer_operstate() cannot be used here since
	 * it doesn't set IF_OPER_LOWERLAYERDOWN (?)
	 */
	old_operstate = master->dev->operstate;
	has_carrier = hsr_check_carrier(master);
	hsr_set_operstate(master, has_carrier);
	hsr_check_announce(master->dev, old_operstate);
}

int hsr_get_max_mtu(struct hsr_priv *hsr)
{
	unsigned int mtu_max;
	struct hsr_port *port;

	mtu_max = ETH_DATA_LEN;
	rcu_read_lock();
	hsr_for_each_port(hsr, port)
		if (port->type != HSR_PT_MASTER)
			mtu_max = min(port->dev->mtu, mtu_max);
	rcu_read_unlock();

	if (mtu_max < HSR_HLEN)
		return 0;

	/* For offloaded keep the mtu same as ETH_DATA_LEN as
	 * h/w is expected to extend the frame to accommodate RCT
	 * or TAG
	 */
	if (!hsr->rx_offloaded)
		return mtu_max - HSR_HLEN;

	return mtu_max;
}

int hsr_lredev_attr_get(struct hsr_priv *hsr, struct lredev_attr *attr)
{
	struct hsr_port *port_a = hsr_port_get_hsr(hsr, HSR_PT_SLAVE_A);
	struct net_device *slave_a_dev;

	if (!port_a)
		return -EINVAL;

	slave_a_dev = port_a->dev;
	if (slave_a_dev && slave_a_dev->lredev_ops &&
	    slave_a_dev->lredev_ops->lredev_attr_get)
		return slave_a_dev->lredev_ops->lredev_attr_get(slave_a_dev,
								attr);
	return -EINVAL;
}

int hsr_lredev_attr_set(struct hsr_priv *hsr, struct lredev_attr *attr)
{
	struct hsr_port *port_a = hsr_port_get_hsr(hsr, HSR_PT_SLAVE_A);
	struct net_device *slave_a_dev;

	if (!port_a)
		return -EINVAL;

	slave_a_dev = port_a->dev;
	if (slave_a_dev && slave_a_dev->lredev_ops &&
	    slave_a_dev->lredev_ops->lredev_attr_set)
		return slave_a_dev->lredev_ops->lredev_attr_set(slave_a_dev,
								attr);
	return -EINVAL;
}

static int _hsr_lredev_get_node_table(struct hsr_priv *hsr,
				      struct lre_node_table_entry table[],
				      int size)
{
	struct hsr_node *node;
	int i = 0;

	rcu_read_lock();

	list_for_each_entry_rcu(node, &hsr->node_db, mac_list) {
		if (hsr_addr_is_self(hsr, node->macaddress_A))
			continue;
		/* SANs are not shown as part of Node Table */
		if (node->san_a || node->san_b)
			continue;
		memcpy(&table[i].mac_address[0],
		       &node->macaddress_A[0], ETH_ALEN);
		table[i].time_last_seen_a = node->time_in[HSR_PT_SLAVE_A];
		table[i].time_last_seen_b = node->time_in[HSR_PT_SLAVE_B];
		if (hsr->prot_version == PRP_V1)
			table[i].node_type = IEC62439_3_DANP;
		else if (hsr->prot_version <= HSR_V1)
			table[i].node_type = IEC62439_3_DANH;
		else
			continue;
		i++;
	}
	rcu_read_unlock();

	return i;
}

int hsr_lredev_get_node_table(struct hsr_priv *hsr,
			      struct lre_node_table_entry table[],
			      int size)
{
	struct hsr_port *port_a = hsr_port_get_hsr(hsr, HSR_PT_SLAVE_A);
	struct net_device *slave_a_dev;
	int ret = -EINVAL;

	if (!port_a)
		return ret;

	if (!hsr->rx_offloaded)
		return _hsr_lredev_get_node_table(hsr, table, size);

	slave_a_dev = port_a->dev;

	if (slave_a_dev && slave_a_dev->lredev_ops &&
	    slave_a_dev->lredev_ops->lredev_get_node_table)
		ret =
		slave_a_dev->lredev_ops->lredev_get_node_table(slave_a_dev,
							       table,
							       size);
	return ret;
}

int hsr_lredev_get_lre_stats(struct hsr_priv *hsr, struct lre_stats *stats)
{
	struct hsr_port *port_a = hsr_port_get_hsr(hsr, HSR_PT_SLAVE_A);
	struct net_device *slave_a_dev;
	int ret = -EINVAL;

	if (!port_a)
		return ret;

	slave_a_dev = port_a->dev;

	if (slave_a_dev && slave_a_dev->lredev_ops &&
	    slave_a_dev->lredev_ops->lredev_get_stats)
		ret =
		slave_a_dev->lredev_ops->lredev_get_stats(slave_a_dev, stats);
	return ret;
}
static int hsr_dev_change_mtu(struct net_device *dev, int new_mtu)
{
	struct hsr_priv *hsr;
	struct hsr_port *master;

	hsr = netdev_priv(dev);
	master = hsr_port_get_hsr(hsr, HSR_PT_MASTER);

	if (new_mtu > hsr_get_max_mtu(hsr)) {
		netdev_info(master->dev, "A HSR master's MTU cannot be greater than the smallest MTU of its slaves minus the HSR Tag length (%d octets).\n",
			    HSR_HLEN);
		return -EINVAL;
	}

	dev->mtu = new_mtu;

	return 0;
}

static int hsr_dev_open(struct net_device *dev)
{
	struct hsr_priv *hsr;
	struct hsr_port *port;
	char designation;

	hsr = netdev_priv(dev);
	designation = '\0';

	rcu_read_lock();
	hsr_for_each_port(hsr, port) {
		if (port->type == HSR_PT_MASTER)
			continue;
		switch (port->type) {
		case HSR_PT_SLAVE_A:
			designation = 'A';
			break;
		case HSR_PT_SLAVE_B:
			designation = 'B';
			break;
		default:
			designation = '?';
		}
		if (!is_slave_up(port->dev))
			netdev_warn(dev, "Slave %c (%s) is not up; please bring it up to get a fully working HSR network\n",
				    designation, port->dev->name);
	}
	rcu_read_unlock();

	if (designation == '\0')
		netdev_warn(dev, "No slave devices configured\n");

	return 0;
}

static int hsr_dev_close(struct net_device *dev)
{
	struct hsr_port *port;
	struct hsr_priv *hsr;

	hsr = netdev_priv(dev);
	hsr_for_each_port(hsr, port) {
		if (port->type == HSR_PT_MASTER)
			continue;
		switch (port->type) {
		case HSR_PT_SLAVE_A:
		case HSR_PT_SLAVE_B:
			dev_uc_unsync(port->dev, dev);
			dev_mc_unsync(port->dev, dev);
			break;
		default:
			break;
		}
	}

	return 0;
}

static netdev_features_t hsr_features_recompute(struct hsr_priv *hsr,
						netdev_features_t features)
{
	netdev_features_t mask;
	struct hsr_port *port;

	mask = features;

	/* Mask out all features that, if supported by one device, should be
	 * enabled for all devices (see NETIF_F_ONE_FOR_ALL).
	 *
	 * Anything that's off in mask will not be enabled - so only things
	 * that were in features originally, and also is in NETIF_F_ONE_FOR_ALL,
	 * may become enabled.
	 */
	features &= ~NETIF_F_ONE_FOR_ALL;
	hsr_for_each_port(hsr, port)
		features = netdev_increment_features(features,
						     port->dev->features,
						     mask);

	return features;
}

static netdev_features_t hsr_fix_features(struct net_device *dev,
					  netdev_features_t features)
{
	struct hsr_priv *hsr = netdev_priv(dev);

	return hsr_features_recompute(hsr, features);
}

static int hsr_dev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct hsr_priv *hsr = netdev_priv(dev);
	struct hsr_port *master;

	master = hsr_port_get_hsr(hsr, HSR_PT_MASTER);
	if (master) {
		skb->dev = master->dev;
		hsr_forward_skb(skb, master);
		INC_CNT_RX_C(hsr);
	} else {
		atomic_long_inc(&dev->tx_dropped);
		dev_kfree_skb_any(skb);
	}
	return NETDEV_TX_OK;
}

static const struct header_ops hsr_header_ops = {
	.create	 = eth_header,
	.parse	 = eth_header_parse,
};

static void send_hsr_supervision_frame(struct hsr_port *master,
				       u8 type, u8 hsr_ver)
{
	struct sk_buff *skb;
	int hlen, tlen;
	struct hsr_tag *hsr_tag = NULL;
	struct prp_rct *rct;
	struct hsr_sup_tag *hsr_stag;
	struct hsr_sup_payload *hsr_sp;
	unsigned long irqflags;
	u16 proto;
	u8 *tail;

	hlen = LL_RESERVED_SPACE(master->dev);
	tlen = master->dev->needed_tailroom;
	/* skb size is same for PRP/HSR frames, only difference
	 * being, for PRP it is a trailer and for HSR it is a
	 * header
	 */
	skb = dev_alloc_skb(sizeof(struct hsr_tag) +
			    sizeof(struct hsr_sup_tag) +
			    sizeof(struct hsr_sup_payload) + hlen + tlen);

	if (!skb)
		return;

	skb_reserve(skb, hlen);

	if (!hsr_ver)
		proto = ETH_P_PRP;
	else
		proto = (hsr_ver == HSR_V1) ? ETH_P_HSR : ETH_P_PRP;
	skb->dev = master->dev;
	skb->protocol = htons(proto);
	skb->priority = TC_PRIO_CONTROL;

	if (dev_hard_header(skb, skb->dev, proto,
			    master->hsr->sup_multicast_addr,
			    skb->dev->dev_addr, skb->len) <= 0)
		goto out;
	skb_reset_mac_header(skb);
	skb_reset_network_header(skb);
	skb_reset_transport_header(skb);

	if (hsr_ver == HSR_V1) {
		hsr_tag = skb_put(skb, sizeof(struct hsr_tag));
		hsr_tag->encap_proto = htons(ETH_P_PRP);
		set_hsr_tag_LSDU_size(hsr_tag, HSR_V1_SUP_LSDUSIZE);
	}

	hsr_stag = skb_put(skb, sizeof(struct hsr_sup_tag));
	set_hsr_stag_path(hsr_stag, (hsr_ver ? 0x0 : 0xf));
	set_hsr_stag_HSR_ver(hsr_stag, (hsr_ver ? 1 : 0));

	/* From HSRv1 on we have separate supervision sequence numbers. */
	spin_lock_irqsave(&master->hsr->seqnr_lock, irqflags);
	if (hsr_ver > 0) {
		hsr_stag->sequence_nr = htons(master->hsr->sup_sequence_nr);
		master->hsr->sup_sequence_nr++;
		if (hsr_ver == HSR_V1) {
			hsr_tag->sequence_nr = htons(master->hsr->sequence_nr);
			master->hsr->sequence_nr++;
		}
	} else {
		hsr_stag->sequence_nr = htons(master->hsr->sequence_nr);
		master->hsr->sequence_nr++;
	}
	spin_unlock_irqrestore(&master->hsr->seqnr_lock, irqflags);

	hsr_stag->HSR_TLV_type = type;
	/* TODO: Why 12 in HSRv0? */
	hsr_stag->HSR_TLV_length =
				hsr_ver ? sizeof(struct hsr_sup_payload) : 12;

	/* Payload: MacAddressA */
	hsr_sp = skb_put(skb, sizeof(struct hsr_sup_payload));
	ether_addr_copy(hsr_sp->macaddress_A, master->dev->dev_addr);

	if (skb_put_padto(skb, ETH_ZLEN + HSR_HLEN))
		return;

	spin_lock_irqsave(&master->hsr->seqnr_lock, irqflags);
	if (hsr_ver == PRP_V1) {
		tail = skb_tail_pointer(skb) - HSR_HLEN;
		rct = (struct prp_rct *)tail;
		rct->PRP_suffix = htons(ETH_P_PRP);
		set_prp_LSDU_size(rct, HSR_V1_SUP_LSDUSIZE);
		rct->sequence_nr = htons(master->hsr->sequence_nr);
		master->hsr->sequence_nr++;
	}
	spin_unlock_irqrestore(&master->hsr->seqnr_lock, irqflags);

	hsr_forward_skb(skb, master);
	INC_CNT_TX_SUP(master->hsr);
	return;

out:
	WARN_ONCE(1, "HSR: Could not send supervision frame\n");
	kfree_skb(skb);
}

/* Announce (supervision frame) timer function
 */
static void hsr_announce(struct timer_list *t)
{
	struct hsr_priv *hsr;
	struct hsr_port *master;
	unsigned long interval;

	hsr = from_timer(hsr, t, announce_timer);

	rcu_read_lock();
	master = hsr_port_get_hsr(hsr, HSR_PT_MASTER);

	if (hsr->announce_count < 3 && hsr->prot_version == 0) {
		send_hsr_supervision_frame(master, HSR_TLV_ANNOUNCE,
					   hsr->prot_version);
		hsr->announce_count++;

		interval = msecs_to_jiffies(HSR_ANNOUNCE_INTERVAL);
	} else {
		if (hsr->prot_version <= HSR_V1) {
			send_hsr_supervision_frame(master,
						   HSR_TLV_LIFE_CHECK,
						   hsr->prot_version);
		} else {/* PRP */
			enum iec62439_3_dd_modes dd_mode =
				(hsr->dd_mode == IEC62439_3_DD) ?
					PRP_TLV_LIFE_CHECK_DD :
					PRP_TLV_LIFE_CHECK_DA;

			send_hsr_supervision_frame(master, dd_mode,
						   hsr->prot_version);
		}

		interval = msecs_to_jiffies(HSR_LIFE_CHECK_INTERVAL);
	}

	if (is_admin_up(master->dev))
		mod_timer(&hsr->announce_timer, jiffies + interval);

	rcu_read_unlock();
}

/* This has to be called after all the readers are gone.
 * Otherwise we would have to check the return value of
 * hsr_port_get_hsr().
 */
static void hsr_dev_destroy(struct net_device *hsr_dev)
{
	struct hsr_priv *hsr;
	struct hsr_port *port;
	struct hsr_port *tmp;

	hsr = netdev_priv(hsr_dev);

	hsr_remove_procfs(hsr, hsr_dev);
	hsr_debugfs_term(hsr);

	list_for_each_entry_safe(port, tmp, &hsr->ports, port_list)
		hsr_del_port(port);

	del_timer_sync(&hsr->prune_timer);
	del_timer_sync(&hsr->announce_timer);

	hsr_del_self_node(hsr);
	hsr_del_nodes(&hsr->node_db);
}

static void hsr_ndo_set_rx_mode(struct net_device *dev)
{
	struct hsr_port *port;
	struct hsr_priv *hsr;

	hsr = netdev_priv(dev);

	rcu_read_lock();
	hsr_for_each_port(hsr, port) {
		if (port->type == HSR_PT_MASTER)
			continue;
		switch (port->type) {
		case HSR_PT_SLAVE_A:
		case HSR_PT_SLAVE_B:
			dev_mc_sync_multiple(port->dev, dev);
			dev_uc_sync_multiple(port->dev, dev);
			break;
		default:
			break;
		}
	}
	rcu_read_unlock();
}

static void hsr_change_rx_flags(struct net_device *dev, int change)
{
	struct hsr_port *port;
	struct hsr_priv *hsr;

	hsr = netdev_priv(dev);

	rcu_read_lock();
	hsr_for_each_port(hsr, port) {
		if (port->type == HSR_PT_MASTER)
			continue;
		switch (port->type) {
		case HSR_PT_SLAVE_A:
		case HSR_PT_SLAVE_B:
			if (change & IFF_ALLMULTI)
				dev_set_allmulti(port->dev,
						 dev->flags &
						 IFF_ALLMULTI ? 1 : -1);
			break;
		default:
			break;
		}
	}
	rcu_read_unlock();
}

static int hsr_ndo_vlan_rx_add_vid(struct net_device *dev,
				   __be16 proto, u16 vid)
{
	struct hsr_port *port;
	struct hsr_priv *hsr;
	int ret = 0;

	hsr = netdev_priv(dev);

	rcu_read_lock();
	hsr_for_each_port(hsr, port) {
		if (port->type == HSR_PT_MASTER)
			continue;

		ret = vlan_vid_add(port->dev, proto, vid);
		switch (port->type) {
		case HSR_PT_SLAVE_A:
			if (ret) {
				netdev_err(dev, "add vid failed for Slave-A\n");
				goto err;
			}
			break;

		case HSR_PT_SLAVE_B:
			if (ret) {
				/* clean up Slave-A */
				netdev_err(dev, "add vid failed for Slave-B\n");
				vlan_vid_del(port->dev, proto, vid);
				goto err;
			}
			break;
		default:
			break;
		};
	}
err:
	rcu_read_unlock();

	return ret;
}

static int hsr_ndo_vlan_rx_kill_vid(struct net_device *dev,
				    __be16 proto, u16 vid)
{
	struct hsr_port *port;
	struct hsr_priv *hsr;

	hsr = netdev_priv(dev);

	rcu_read_lock();
	hsr_for_each_port(hsr, port) {
		if (port->type == HSR_PT_MASTER)
			continue;
		switch (port->type) {
		case HSR_PT_SLAVE_A:
		case HSR_PT_SLAVE_B:
			vlan_vid_del(port->dev, proto, vid);
			break;
		default:
			break;
		};
	}
	rcu_read_unlock();

	return 0;
}

static const struct net_device_ops hsr_device_ops = {
	.ndo_change_mtu = hsr_dev_change_mtu,
	.ndo_open = hsr_dev_open,
	.ndo_stop = hsr_dev_close,
	.ndo_start_xmit = hsr_dev_xmit,
	.ndo_change_rx_flags = hsr_change_rx_flags,
	.ndo_fix_features = hsr_fix_features,
	.ndo_uninit = hsr_dev_destroy,
	.ndo_set_rx_mode = hsr_ndo_set_rx_mode,
	.ndo_vlan_rx_add_vid = hsr_ndo_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid = hsr_ndo_vlan_rx_kill_vid,
};

static struct device_type hsr_type = {
	.name = "hsr",
};

void hsr_dev_setup(struct net_device *dev)
{
	eth_hw_addr_random(dev);

	ether_setup(dev);
	dev->min_mtu = 0;
	dev->header_ops = &hsr_header_ops;
	dev->netdev_ops = &hsr_device_ops;
	SET_NETDEV_DEVTYPE(dev, &hsr_type);
	dev->priv_flags |= IFF_NO_QUEUE;

	dev->needs_free_netdev = true;

	dev->hw_features = NETIF_F_SG | NETIF_F_FRAGLIST | NETIF_F_HIGHDMA |
			   NETIF_F_GSO_MASK | NETIF_F_HW_CSUM |
			   NETIF_F_HW_VLAN_CTAG_TX |
			   NETIF_F_HW_VLAN_CTAG_FILTER;

	dev->features = dev->hw_features;

	/* Prevent recursive tx locking */
	dev->features |= NETIF_F_LLTX;
	/* Not sure about this. Taken from bridge code. netdev_features.h says
	 * it means "Does not change network namespaces".
	 */
	dev->features |= NETIF_F_NETNS_LOCAL;
}

/* Return true if dev is a HSR master; return false otherwise.
 */
inline bool is_hsr_master(struct net_device *dev)
{
	return (dev->netdev_ops->ndo_start_xmit == hsr_dev_xmit);
}

/* Default multicast address for HSR Supervision frames */
static const unsigned char def_multicast_addr[ETH_ALEN] __aligned(2) = {
	0x01, 0x15, 0x4e, 0x00, 0x01, 0x00
};

int hsr_dev_finalize(struct net_device *hsr_dev, struct net_device *slave[2],
		     unsigned char multicast_spec, u8 protocol_version)
{
	struct hsr_priv *hsr;
	struct hsr_port *port;
	struct hsr_port *tmp;
	int res;

	hsr = netdev_priv(hsr_dev);
	INIT_LIST_HEAD(&hsr->ports);
	INIT_LIST_HEAD(&hsr->node_db);
	INIT_LIST_HEAD(&hsr->self_node_db);
	spin_lock_init(&hsr->list_lock);

	ether_addr_copy(hsr_dev->dev_addr, slave[0]->dev_addr);

	/* Make sure we recognize frames from ourselves in hsr_rcv() */
	res = hsr_create_self_node(hsr, hsr_dev->dev_addr,
				   slave[1]->dev_addr);
	if (res < 0)
		return res;

	hsr->prot_version = protocol_version;
	if (hsr->prot_version == PRP_V1) {
		/* For PRP, lan_id has most significant 3 bits holding
		 * the net_id of PRP_LAN_ID and also duplicate discard
		 * mode set.
		 */
		hsr->net_id = PRP_LAN_ID << 1;
		hsr->dd_mode = IEC62439_3_DD;
	} else {
		hsr->hsr_mode = IEC62439_3_HSR_MODE_H;
	}

	spin_lock_init(&hsr->seqnr_lock);
	/* Overflow soon to find bugs easier: */
	hsr->sequence_nr = HSR_SEQNR_START;
	hsr->sup_sequence_nr = HSR_SUP_SEQNR_START;

	timer_setup(&hsr->announce_timer, hsr_announce, 0);
	if (!hsr->rx_offloaded)
		timer_setup(&hsr->prune_timer, hsr_prune_nodes, 0);

	ether_addr_copy(hsr->sup_multicast_addr, def_multicast_addr);
	hsr->sup_multicast_addr[ETH_ALEN - 1] = multicast_spec;

	/* FIXME: should I modify the value of these?
	 *
	 * - hsr_dev->flags - i.e.
	 *			IFF_MASTER/SLAVE?
	 * - hsr_dev->priv_flags - i.e.
	 *			IFF_EBRIDGE?
	 *			IFF_TX_SKB_SHARING?
	 *			IFF_HSR_MASTER/SLAVE?
	 */

	/* Make sure the 1st call to netif_carrier_on() gets through */
	netif_carrier_off(hsr_dev);

	res = hsr_add_port(hsr, hsr_dev, HSR_PT_MASTER);
	if (res)
		goto err_add_master;

	if (hsr->prot_version == PRP_V1) {
		if ((slave[0]->features & NETIF_F_HW_HSR_RX_OFFLOAD) ||
		    (slave[1]->features & NETIF_F_HW_HSR_RX_OFFLOAD)) {
			res = -EINVAL;
			goto err_add_master;
		}
	} else {
		if ((slave[0]->features & NETIF_F_HW_PRP_RX_OFFLOAD) ||
		    (slave[1]->features & NETIF_F_HW_PRP_RX_OFFLOAD)) {
			res = -EINVAL;
			goto err_add_master;
		}
	}

	/* HSR/PRP LRE Rx offload supported in lower device? */
	if (((slave[0]->features & NETIF_F_HW_HSR_RX_OFFLOAD) &&
	     (slave[1]->features & NETIF_F_HW_HSR_RX_OFFLOAD)) ||
	     ((slave[0]->features & NETIF_F_HW_PRP_RX_OFFLOAD) &&
	     (slave[1]->features & NETIF_F_HW_PRP_RX_OFFLOAD)))
		hsr->rx_offloaded = true;

	/* HSR LRE L2 forward offload supported in lower device for hsr? */
	if (hsr->prot_version < PRP_V1 &&
	    ((slave[0]->features & NETIF_F_HW_L2FW_DOFFLOAD) &&
	    (slave[1]->features & NETIF_F_HW_L2FW_DOFFLOAD)))
		hsr->l2_fwd_offloaded = true;

	if ((slave[0]->features & NETIF_F_HW_VLAN_CTAG_FILTER) &&
	    (slave[1]->features & NETIF_F_HW_VLAN_CTAG_FILTER))
		hsr_dev->features |= NETIF_F_HW_VLAN_CTAG_FILTER;

	res = register_netdevice(hsr_dev);
	if (res)
		goto err_unregister;

	res = hsr_add_port(hsr, slave[0], HSR_PT_SLAVE_A);
	if (res)
		goto err_add_slaves;

	res = hsr_add_port(hsr, slave[1], HSR_PT_SLAVE_B);
	if (res)
		goto err_add_slaves;

	hsr_debugfs_init(hsr, hsr_dev);

	/* For LRE rx offload, pruning is expected to happen
	 * at the hardware or firmware . So don't do this in software
	 */
	if (!hsr->rx_offloaded)
		mod_timer(&hsr->prune_timer,
			  jiffies + msecs_to_jiffies(PRUNE_PERIOD));
	/* for offloaded case, expect both slaves have the
	 * same MAC address configured. If not fail.
	 */
	if (hsr->rx_offloaded &&
	    !ether_addr_equal(slave[0]->dev_addr,
			      slave[1]->dev_addr)) {
		netdev_err(hsr_dev,
			   "Slave's MAC addr must be same. So change it\n");
		res = -EINVAL;
		goto err_add_slaves;
	}

	res = hsr_create_procfs(hsr, hsr_dev);
	if (res)
		goto err_add_slaves;

	return 0;

err_add_slaves:
	unregister_netdevice(hsr_dev);
err_unregister:
	list_for_each_entry_safe(port, tmp, &hsr->ports, port_list)
		hsr_del_port(port);
err_add_master:
	hsr_del_self_node(hsr);

	return res;
}
