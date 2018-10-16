// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments ICSSG Ethernet driver
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#include "icssg_prueth.h"
#include <linux/regmap.h>

static u32 stats_base[] = {	0x54c,	/* Slice 0 stats start */
				0xb18,	/* Slice 1 stats start */
};

struct miig_stats_regs {
	/* Rx */
	u32 rx_good_frames;
	u32 rx_broadcast_frames;
	u32 rx_multicast_frames;
	u32 rx_crc_error_frames;
	u32 rx_mii_error_frames;
	u32 rx_odd_nibble_frames;
	u32 rx_frame_max_size;
	u32 rx_max_size_error_frames;
	u32 rx_frame_min_size;
	u32 rx_min_size_error_frames;
	u32 rx_overrun_frames;
	u32 rx_class0_hits;
	u32 rx_class1_hits;
	u32 rx_class2_hits;
	u32 rx_class3_hits;
	u32 rx_class4_hits;
	u32 rx_class5_hits;
	u32 rx_class6_hits;
	u32 rx_class7_hits;
	u32 rx_class8_hits;
	u32 rx_class9_hits;
	u32 rx_class10_hits;
	u32 rx_class11_hits;
	u32 rx_class12_hits;
	u32 rx_class13_hits;
	u32 rx_class14_hits;
	u32 rx_class15_hits;
	u32 rx_smd_frags;
	u32 rx_bucket1_size;
	u32 rx_bucket2_size;
	u32 rx_bucket3_size;
	u32 rx_bucket4_size;
	u32 rx_64B_frames;
	u32 rx_bucket1_frames;
	u32 rx_bucket2_frames;
	u32 rx_bucket3_frames;
	u32 rx_bucket4_frames;
	u32 rx_bucket5_frames;
	u32 rx_total_bytes;
	u32 rx_tx_total_bytes;
	/* Tx */
	u32 tx_good_frames;
	u32 tx_broadcast_frames;
	u32 tx_multicast_frames;
	u32 tx_odd_nibble_frames;
	u32 tx_underflow_errors;
	u32 tx_frame_max_size;
	u32 tx_max_size_error_frames;
	u32 tx_frame_min_size;
	u32 tx_min_size_error_frames;
	u32 tx_bucket1_size;
	u32 tx_bucket2_size;
	u32 tx_bucket3_size;
	u32 tx_bucket4_size;
	u32 tx_64B_frames;
	u32 tx_bucket1_frames;
	u32 tx_bucket2_frames;
	u32 tx_bucket3_frames;
	u32 tx_bucket4_frames;
	u32 tx_bucket5_frames;
	u32 tx_total_bytes;
};

#define ICSSG_STATS(field)				\
{							\
	#field,						\
	offsetof(struct miig_stats_regs, field),	\
}

struct icssg_stats {
	char name[ETH_GSTRING_LEN];
	u32 offset;
};

static const struct icssg_stats icssg_ethtool_stats[] = {
	/* Rx */
	ICSSG_STATS(rx_good_frames),
	ICSSG_STATS(rx_broadcast_frames),
	ICSSG_STATS(rx_multicast_frames),
	ICSSG_STATS(rx_crc_error_frames),
	ICSSG_STATS(rx_mii_error_frames),
	ICSSG_STATS(rx_odd_nibble_frames),
	ICSSG_STATS(rx_frame_max_size),
	ICSSG_STATS(rx_max_size_error_frames),
	ICSSG_STATS(rx_frame_min_size),
	ICSSG_STATS(rx_min_size_error_frames),
	ICSSG_STATS(rx_overrun_frames),
	ICSSG_STATS(rx_class0_hits),
	ICSSG_STATS(rx_class1_hits),
	ICSSG_STATS(rx_class2_hits),
	ICSSG_STATS(rx_class3_hits),
	ICSSG_STATS(rx_class4_hits),
	ICSSG_STATS(rx_class5_hits),
	ICSSG_STATS(rx_class6_hits),
	ICSSG_STATS(rx_class7_hits),
	ICSSG_STATS(rx_class8_hits),
	ICSSG_STATS(rx_class9_hits),
	ICSSG_STATS(rx_class10_hits),
	ICSSG_STATS(rx_class11_hits),
	ICSSG_STATS(rx_class12_hits),
	ICSSG_STATS(rx_class13_hits),
	ICSSG_STATS(rx_class14_hits),
	ICSSG_STATS(rx_class15_hits),
	ICSSG_STATS(rx_smd_frags),
	ICSSG_STATS(rx_bucket1_size),
	ICSSG_STATS(rx_bucket2_size),
	ICSSG_STATS(rx_bucket3_size),
	ICSSG_STATS(rx_bucket4_size),
	ICSSG_STATS(rx_64B_frames),
	ICSSG_STATS(rx_bucket1_frames),
	ICSSG_STATS(rx_bucket2_frames),
	ICSSG_STATS(rx_bucket3_frames),
	ICSSG_STATS(rx_bucket4_frames),
	ICSSG_STATS(rx_bucket5_frames),
	ICSSG_STATS(rx_total_bytes),
	ICSSG_STATS(rx_tx_total_bytes),
	/* Tx */
	ICSSG_STATS(tx_good_frames),
	ICSSG_STATS(tx_broadcast_frames),
	ICSSG_STATS(tx_multicast_frames),
	ICSSG_STATS(tx_odd_nibble_frames),
	ICSSG_STATS(tx_underflow_errors),
	ICSSG_STATS(tx_frame_max_size),
	ICSSG_STATS(tx_max_size_error_frames),
	ICSSG_STATS(tx_frame_min_size),
	ICSSG_STATS(tx_min_size_error_frames),
	ICSSG_STATS(tx_bucket1_size),
	ICSSG_STATS(tx_bucket2_size),
	ICSSG_STATS(tx_bucket3_size),
	ICSSG_STATS(tx_bucket4_size),
	ICSSG_STATS(tx_64B_frames),
	ICSSG_STATS(tx_bucket1_frames),
	ICSSG_STATS(tx_bucket2_frames),
	ICSSG_STATS(tx_bucket3_frames),
	ICSSG_STATS(tx_bucket4_frames),
	ICSSG_STATS(tx_bucket5_frames),
	ICSSG_STATS(tx_total_bytes),
};

static void emac_get_drvinfo(struct net_device *ndev,
			     struct ethtool_drvinfo *info)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	strlcpy(info->driver, dev_driver_string(prueth->dev),
		sizeof(info->driver));
	/* TODO: info->fw_version */
	strlcpy(info->bus_info, dev_name(prueth->dev), sizeof(info->bus_info));
}

static u32 emac_get_msglevel(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	return emac->msg_enable;
}

static void emac_set_msglevel(struct net_device *ndev, u32 value)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	emac->msg_enable = value;
}

static int emac_get_link_ksettings(struct net_device *ndev,
				   struct ethtool_link_ksettings *ecmd)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (!emac->phydev)
		return -EOPNOTSUPP;

	phy_ethtool_ksettings_get(emac->phydev, ecmd);
	return 0;
}

static int emac_set_link_ksettings(struct net_device *ndev,
				   const struct ethtool_link_ksettings *ecmd)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (!emac->phydev || phy_is_pseudo_fixed_link(emac->phydev))
		return -EOPNOTSUPP;

	return phy_ethtool_ksettings_set(emac->phydev, ecmd);
}

static int emac_get_eee(struct net_device *ndev, struct ethtool_eee *edata)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (!emac->phydev || phy_is_pseudo_fixed_link(emac->phydev))
		return -EOPNOTSUPP;

	return phy_ethtool_get_eee(emac->phydev, edata);
}

static int emac_set_eee(struct net_device *ndev, struct ethtool_eee *edata)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (!emac->phydev || phy_is_pseudo_fixed_link(emac->phydev))
		return -EOPNOTSUPP;

	return phy_ethtool_set_eee(emac->phydev, edata);
}

static int emac_nway_reset(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (!emac->phydev || phy_is_pseudo_fixed_link(emac->phydev))
		return -EOPNOTSUPP;

	return genphy_restart_aneg(emac->phydev);
}

static int emac_get_sset_count(struct net_device *ndev, int stringset)
{
	switch (stringset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(icssg_ethtool_stats);
	default:
		return -EOPNOTSUPP;
	}
}

static void emac_get_strings(struct net_device *ndev, u32 stringset, u8 *data)
{
	u8 *p = data;
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ARRAY_SIZE(icssg_ethtool_stats); i++) {
			memcpy(p, icssg_ethtool_stats[i].name,
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	default:
		break;
	}
}

static void emac_get_ethtool_stats(struct net_device *ndev,
				   struct ethtool_stats *stats, u64 *data)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	int i;
	int slice = prueth_emac_slice(emac);
	u32 base = stats_base[slice];
	u32 val;

	for (i = 0; i < ARRAY_SIZE(icssg_ethtool_stats); i++) {
		regmap_read(prueth->miig_rt,
			    base + icssg_ethtool_stats[i].offset,
			    &val);
		data[i] = val;
	}
}

const struct ethtool_ops icssg_ethtool_ops = {
	.get_drvinfo = emac_get_drvinfo,
	.get_msglevel = emac_get_msglevel,
	.set_msglevel = emac_set_msglevel,
	.get_sset_count = emac_get_sset_count,
	.get_strings = emac_get_strings,
	.get_ethtool_stats = emac_get_ethtool_stats,

	.get_link_ksettings = emac_get_link_ksettings,
	.set_link_ksettings = emac_set_link_ksettings,
	.get_link = ethtool_op_get_link,
	.get_eee = emac_get_eee,
	.set_eee = emac_set_eee,
	.nway_reset = emac_nway_reset,
};
