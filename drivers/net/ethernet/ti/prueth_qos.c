// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com
 */

#include <linux/kernel.h>
#include <linux/pruss.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <net/pkt_cls.h>

#include "icss_mii_rt.h"
#include "icss_vlan_mcast_filter_mmap.h"
#include "prueth.h"

static void emac_nsp_enable(void __iomem *counter, u16 credit)
{
	writel((credit << PRUETH_NSP_CREDIT_SHIFT) | PRUETH_NSP_ENABLE,
	       counter);
}

void prueth_enable_nsp(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	void __iomem *dram = prueth->mem[emac->dram].va;

	if (emac->nsp_bc.cookie)
		emac_nsp_enable(dram + STORM_PREVENTION_OFFSET_BC,
				emac->nsp_bc.credit);
	if (emac->nsp_mc.cookie)
		emac_nsp_enable(dram + STORM_PREVENTION_OFFSET_MC,
				emac->nsp_mc.credit);
	if (emac->nsp_uc.cookie)
		emac_nsp_enable(dram + STORM_PREVENTION_OFFSET_UC,
				emac->nsp_uc.credit);
}

static int emac_flower_parse_policer(struct prueth_emac *emac,
				     struct netlink_ext_ack *extack,
				     struct flow_cls_offload *cls,
				     u64 rate_bytes_per_sec)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(cls);
	struct flow_dissector *dissector = rule->match.dissector;
	u8 null_mac[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	u8 bc_mac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	u8 mc_mac[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
	struct flow_match_eth_addrs match;
	struct nsp_counter *nsp = NULL;
	unsigned long flags;
	char *str;
	u32 pps;

	if (dissector->used_keys &
	    ~(BIT(FLOW_DISSECTOR_KEY_BASIC) |
	      BIT(FLOW_DISSECTOR_KEY_CONTROL) |
	      BIT(FLOW_DISSECTOR_KEY_ETH_ADDRS))) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Unsupported keys used");
		return -EOPNOTSUPP;
	}

	if (!flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_ETH_ADDRS)) {
		NL_SET_ERR_MSG_MOD(extack, "Not matching on eth address");
		return -EOPNOTSUPP;
	}

	flow_rule_match_eth_addrs(rule, &match);

	if (!ether_addr_equal_masked(match.key->src, null_mac,
				     match.mask->src)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Matching on source MAC not supported");
		return -EOPNOTSUPP;
	}

	if (ether_addr_equal(match.key->dst, bc_mac)) {
		if (!emac->nsp_bc.cookie ||
		    emac->nsp_bc.cookie == cls->cookie)
			nsp = &emac->nsp_bc;
		else
			NL_SET_ERR_MSG_MOD(extack, "BC Filter already set");
		str = "Broad";
	} else if (ether_addr_equal_masked(match.key->dst, mc_mac, mc_mac)) {
		if (!emac->nsp_mc.cookie ||
		    emac->nsp_mc.cookie == cls->cookie)
			nsp = &emac->nsp_mc;
		else
			NL_SET_ERR_MSG_MOD(extack, "MC Filter already set");
		str = "Multi";
	} else {
		if (!emac->nsp_uc.cookie ||
		    emac->nsp_uc.cookie == cls->cookie)
			nsp = &emac->nsp_uc;
		else
			NL_SET_ERR_MSG_MOD(extack, "UC Filter already set");
		str = "Uni";
	}

	if (!nsp)
		return -EOPNOTSUPP;

	/* Calculate number of packets per second for given bps
	 * assuming min ethernet packet size
	 */
	pps = div_u64(rate_bytes_per_sec, ETH_ZLEN);
	/* Convert that to packets per 100ms */
	pps /= MSEC_PER_SEC / PRUETH_NSP_TIMER_MS;

	spin_lock_irqsave(&emac->nsp_lock, flags);
	nsp->cookie = cls->cookie;
	nsp->credit = pps;
	emac->nsp_enabled = emac->nsp_bc.cookie | emac->nsp_mc.cookie |
			    emac->nsp_uc.cookie;
	spin_unlock_irqrestore(&emac->nsp_lock, flags);

	netdev_dbg(emac->ndev,
		   "%scast filter set to %d packets per %dms\n", str,
		   nsp->credit, PRUETH_NSP_TIMER_MS);

	prueth_start_timer(emac->prueth);

	return 0;
}

static int emac_configure_clsflower(struct prueth_emac *emac,
				    struct flow_cls_offload *cls)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(cls);
	struct netlink_ext_ack *extack = cls->common.extack;
	const struct flow_action_entry *act;
	int i;

	flow_action_for_each(i, act, &rule->action) {
		switch (act->id) {
		case FLOW_ACTION_POLICE:
			return emac_flower_parse_policer(emac, extack, cls,
							 act->police.rate_bytes_ps);
		default:
			NL_SET_ERR_MSG_MOD(extack,
					   "Action not supported");
			return -EOPNOTSUPP;
		}
	}
	return -EOPNOTSUPP;
}

static int emac_delete_clsflower(struct prueth_emac *emac,
				 struct flow_cls_offload *cls)
{
	struct prueth *prueth = emac->prueth;
	void __iomem *dram = prueth->mem[emac->dram].va;
	unsigned long flags;

	spin_lock_irqsave(&emac->nsp_lock, flags);

	if (cls->cookie == emac->nsp_bc.cookie) {
		emac->nsp_bc.cookie = 0;
		emac->nsp_bc.credit = 0;
		writel(0, dram + STORM_PREVENTION_OFFSET_BC);
	} else if (cls->cookie == emac->nsp_mc.cookie) {
		emac->nsp_mc.cookie = 0;
		emac->nsp_mc.credit = 0;
		writel(0, dram + STORM_PREVENTION_OFFSET_MC);
	} else if (cls->cookie == emac->nsp_uc.cookie) {
		emac->nsp_uc.cookie = 0;
		emac->nsp_uc.credit = 0;
		writel(0, dram + STORM_PREVENTION_OFFSET_UC);
	}

	emac->nsp_enabled = emac->nsp_bc.cookie | emac->nsp_mc.cookie |
			    emac->nsp_uc.cookie;

	spin_unlock_irqrestore(&emac->nsp_lock, flags);

	return 0;
}

static int emac_setup_tc_cls_flower(struct prueth_emac *emac,
				    struct flow_cls_offload *cls_flower)
{
	switch (cls_flower->command) {
	case FLOW_CLS_REPLACE:
		return emac_configure_clsflower(emac, cls_flower);
	case FLOW_CLS_DESTROY:
		return emac_delete_clsflower(emac, cls_flower);
	default:
		return -EOPNOTSUPP;
	}
}

static int emac_setup_tc_block_cb(enum tc_setup_type type, void *type_data,
				  void *cb_priv)
{
	struct prueth_emac *emac = cb_priv;

	if (!tc_cls_can_offload_and_chain0(emac->ndev, type_data))
		return -EOPNOTSUPP;

	switch (type) {
	case TC_SETUP_CLSFLOWER:
		return emac_setup_tc_cls_flower(emac, type_data);
	default:
		return -EOPNOTSUPP;
	}
}

static LIST_HEAD(emac_block_cb_list);

int emac_ndo_setup_tc(struct net_device *dev, enum tc_setup_type type,
		      void *type_data)
{
	struct prueth_emac *emac = netdev_priv(dev);

	if (type == TC_SETUP_BLOCK) {
		return flow_block_cb_setup_simple(type_data,
						  &emac_block_cb_list,
						  emac_setup_tc_block_cb,
						  emac, emac, true);
	}

	return -EOPNOTSUPP;
}
