// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments ICSSG Ethernet driver
 *
 * Copyright (C) 2018-2021 Texas Instruments Incorporated - https://www.ti.com/
 *
 */

#include "icssg_prueth.h"
#include <linux/regmap.h>

#define STATS_TIME_LIMIT_MS 25000000

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

/* Ethtool priv_flags for IET/Frame Preemption configuration.
 * TODO: This is a temporary solution until upstream interface
 * is available.
 */
static const char emac_ethtool_priv_flags[][ETH_GSTRING_LEN] = {
#define EMAC_PRIV_IET_FRAME_PREEMPTION	BIT(0)
	"iet-frame-preemption",
#define EMAC_PRIV_IET_MAC_VERIFY		BIT(1)
	"iet-mac-verify",
};

static int emac_get_sset_count(struct net_device *ndev, int stringset)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	switch (stringset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(icssg_ethtool_stats);
	case ETH_SS_PRIV_FLAGS:
		if (!prueth->is_sr1)
			return ARRAY_SIZE(emac_ethtool_priv_flags);
		return -EOPNOTSUPP;
	default:
		return -EOPNOTSUPP;
	}
}

static void emac_get_strings(struct net_device *ndev, u32 stringset, u8 *data)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
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
	case ETH_SS_PRIV_FLAGS:
		if (prueth->is_sr1)
			return;

		for (i = 0; i < ARRAY_SIZE(emac_ethtool_priv_flags); i++) {
			memcpy(p, emac_ethtool_priv_flags[i],
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	default:
		break;
	}
}

static void emac_update_hardware_stats(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	int slice = prueth_emac_slice(emac);
	u32 base = stats_base[slice];
	u32 val;
	int i;

	for (i = 0; i < ARRAY_SIZE(icssg_ethtool_stats); i++) {
		regmap_read(prueth->miig_rt,
			    base + icssg_ethtool_stats[i].offset,
			    &val);
		regmap_write(prueth->miig_rt,
			     base + icssg_ethtool_stats[i].offset,
			     val);

		emac->stats[i] += val;
	}
}

void emac_stats_work_handler(struct work_struct *work)
{
	struct prueth_emac *emac = container_of(work, struct prueth_emac,
						stats_work.work);
	emac_update_hardware_stats(emac);

	queue_delayed_work(system_long_wq, &emac->stats_work,
			   msecs_to_jiffies(STATS_TIME_LIMIT_MS / emac->speed));
}

void emac_ethtool_stats_init(struct prueth_emac *emac)
{
	if (!emac->stats) {
		struct device *dev = emac->prueth->dev;

		emac->stats = devm_kzalloc(dev, ARRAY_SIZE(icssg_ethtool_stats) *
					   sizeof(*emac->stats), GFP_KERNEL);
	}
}

static void emac_get_ethtool_stats(struct net_device *ndev,
				   struct ethtool_stats *stats, u64 *data)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	int i;

	emac_update_hardware_stats(emac);

	for (i = 0; i < ARRAY_SIZE(icssg_ethtool_stats); i++)
		data[i] = emac->stats[i];
}

static int emac_get_ts_info(struct net_device *ndev,
			    struct ethtool_ts_info *info)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	info->so_timestamping =
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;

	info->phc_index = icss_iep_get_ptp_clock_idx(emac->iep);
	info->tx_types = BIT(HWTSTAMP_TX_OFF) | BIT(HWTSTAMP_TX_ON);
	info->rx_filters = BIT(HWTSTAMP_FILTER_NONE) | BIT(HWTSTAMP_FILTER_ALL);

	return 0;
}

static void emac_get_channels(struct net_device *ndev,
			      struct ethtool_channels *ch)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	ch->max_rx = 1;
	/* SR1 use high priority channel for management messages */
	ch->max_tx = emac->is_sr1 ? PRUETH_MAX_TX_QUEUES - 1 :
				    PRUETH_MAX_TX_QUEUES;
	ch->rx_count = 1;
	ch->tx_count = emac->is_sr1 ? emac->tx_ch_num - 1 :
				      emac->tx_ch_num;
}

static int emac_set_channels(struct net_device *ndev,
			     struct ethtool_channels *ch)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	/* verify we have at least one channel in each direction */
	/* TODO: remove below check before sending to LKML */
	if (!ch->rx_count || !ch->tx_count)
		return -EINVAL;

	/* Check if interface is up. Can change the num queues when
	 * the interface is down.
	 */
	if (netif_running(emac->ndev))
		return -EBUSY;

	emac->tx_ch_num = ch->tx_count;
	/* highest channel number for management messaging on SR1 */
	if (emac->is_sr1)
		emac->tx_ch_num++;

	return 0;
}

/* TODO : This is temporary until a formal ethtool interface become available
 * in LKML to configure IET FPE.
 */
static u32 emac_get_ethtool_priv_flags(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth_qos_iet *iet = &emac->qos.iet;
	u32 priv_flags = 0;

	if (emac->is_sr1)
		return priv_flags;

	/* Port specific flags */
	if (iet->fpe_configured)
		priv_flags |= EMAC_PRIV_IET_FRAME_PREEMPTION;
	if (iet->mac_verify_configured)
		priv_flags |= EMAC_PRIV_IET_MAC_VERIFY;

	return priv_flags;
}

static int emac_set_ethtool_priv_flags(struct net_device *ndev, u32 flags)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth_qos_iet *iet = &emac->qos.iet;
	int iet_fpe, mac_verify;

	if (emac->is_sr1)
		return -EOPNOTSUPP;

	iet_fpe = !!(flags & EMAC_PRIV_IET_FRAME_PREEMPTION);
	mac_verify = !!(flags & EMAC_PRIV_IET_MAC_VERIFY);

	if (netif_running(ndev))
		return -EBUSY;

	if (emac->tx_ch_num < 2 && iet_fpe) {
		netdev_err(ndev, "IET fpe needs at least 2 h/w queues\n");
		return -EINVAL;
	}

	if (mac_verify && (!iet->fpe_configured && !iet_fpe)) {
		netdev_err(ndev, "Enable IET FPE for IET MAC verify\n");
		return -EINVAL;
	}

	iet->fpe_configured = iet_fpe;
	iet->mac_verify_configured = mac_verify;

	return 0;
}

const struct ethtool_ops icssg_ethtool_ops = {
	.get_drvinfo = emac_get_drvinfo,
	.get_msglevel = emac_get_msglevel,
	.set_msglevel = emac_set_msglevel,
	.get_sset_count = emac_get_sset_count,
	.get_strings = emac_get_strings,
	.get_ethtool_stats = emac_get_ethtool_stats,
	.get_ts_info = emac_get_ts_info,
	.get_priv_flags = emac_get_ethtool_priv_flags,
	.set_priv_flags = emac_set_ethtool_priv_flags,

	.get_channels = emac_get_channels,
	.set_channels = emac_set_channels,
	.get_link_ksettings = emac_get_link_ksettings,
	.set_link_ksettings = emac_set_link_ksettings,
	.get_link = ethtool_op_get_link,
	.get_eee = emac_get_eee,
	.set_eee = emac_set_eee,
	.nway_reset = emac_nway_reset,
};
