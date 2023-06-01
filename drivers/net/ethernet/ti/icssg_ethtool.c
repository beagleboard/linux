// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments ICSSG Ethernet driver
 *
 * Copyright (C) 2018-2022 Texas Instruments Incorporated - https://www.ti.com/
 *
 */

#include "icssg_prueth.h"
#include <linux/regmap.h>

#define STATS_TIME_LIMIT_1G_MS    25000    /* 25 seconds @ 1G */
static const u32 stats_base[] = {	0x54c,	/* Slice 0 stats start */
					0xb18,	/* Slice 1 stats start */
};

/* ICSSG MIIG_STATS registers */
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

/* ICSSG PA_STATS registers */
struct pa_stats_regs {
	u32 iet_bad_frag_slice0;
	u32 iet_bad_frag_slice1;
	u32 iet_asm_err_slice0;
	u32 iet_asm_err_slice1;
	u32 iet_tx_frag_slice0;
	u32 iet_tx_frag_slice1;
	u32 iet_asm_ok_slice0;
	u32 iet_asm_ok_slice1;
	u32 iet_rx_frag_slice0;
	u32 iet_rx_frag_slice1;
};

#define ICSSG_STATS(field)				\
{							\
	#field,						\
	offsetof(struct miig_stats_regs, field),	\
}

#define ICSSG_PA_STATS(field)			\
{						\
	#field,					\
	offsetof(struct pa_stats_regs, field),	\
}

/**
 * struct icssg_stats - ICSSG Stats structure
 * @name: Stats name
 * @offset: ICSSG stats register offset
 */
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

static const struct icssg_stats icssg_pa_stats[] = {
	/* PA STATS */
	ICSSG_PA_STATS(iet_bad_frag_slice0),
	ICSSG_PA_STATS(iet_bad_frag_slice1),
	ICSSG_PA_STATS(iet_asm_err_slice0),
	ICSSG_PA_STATS(iet_asm_err_slice1),
	ICSSG_PA_STATS(iet_tx_frag_slice0),
	ICSSG_PA_STATS(iet_tx_frag_slice1),
	ICSSG_PA_STATS(iet_asm_ok_slice0),
	ICSSG_PA_STATS(iet_asm_ok_slice1),
	ICSSG_PA_STATS(iet_rx_frag_slice0),
	ICSSG_PA_STATS(iet_rx_frag_slice1),
};

static void emac_get_drvinfo(struct net_device *ndev,
			     struct ethtool_drvinfo *info)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	strscpy(info->driver, dev_driver_string(prueth->dev),
		sizeof(info->driver));
	strscpy(info->bus_info, dev_name(prueth->dev), sizeof(info->bus_info));
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
	return phy_ethtool_get_link_ksettings(ndev, ecmd);
}

static int emac_set_link_ksettings(struct net_device *ndev,
				   const struct ethtool_link_ksettings *ecmd)
{
	return phy_ethtool_set_link_ksettings(ndev, ecmd);
}

static int emac_get_eee(struct net_device *ndev, struct ethtool_eee *edata)
{
	if (!ndev->phydev)
		return -EOPNOTSUPP;

	return phy_ethtool_get_eee(ndev->phydev, edata);
}

static int emac_set_eee(struct net_device *ndev, struct ethtool_eee *edata)
{
	if (!ndev->phydev)
		return -EOPNOTSUPP;

	return phy_ethtool_set_eee(ndev->phydev, edata);
}

static int emac_nway_reset(struct net_device *ndev)
{
	return phy_ethtool_nway_reset(ndev);
}

#define EMAC_PRIV_IET_FRAME_PREEMPTION  BIT(0)
#define EMAC_PRIV_IET_MAC_VERIFY        BIT(1)

static const char emac_ethtool_priv_flags[][ETH_GSTRING_LEN] = {
	"iet-frame-preemption",
	"iet-mac-verify",
};

static int emac_get_sset_count(struct net_device *ndev, int stringset)
{
	switch (stringset) {
	case ETH_SS_STATS:
		return ICSSG_NUM_STATS;
	case ETH_SS_PRIV_FLAGS:
		return ARRAY_SIZE(emac_ethtool_priv_flags);
	default:
		return -EOPNOTSUPP;
	}
}

static void emac_get_strings(struct net_device *ndev, u32 stringset, u8 *data)
{
	const struct icssg_stats *hw_stats;
	u32 i, num_stats;
	u8 *p = data;

	switch (stringset) {
	case ETH_SS_STATS:
		num_stats = ARRAY_SIZE(icssg_ethtool_stats);
		hw_stats = icssg_ethtool_stats;
		for (i = 0; i < num_stats; i++) {
			memcpy(p, hw_stats[i].name, ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}

		num_stats = ARRAY_SIZE(icssg_pa_stats);
		hw_stats = icssg_pa_stats;
		for (i = 0; i < num_stats; i++) {
			memcpy(p, hw_stats[i].name, ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	case ETH_SS_PRIV_FLAGS:
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
	int i, j;
	u32 val;

	for (i = 0; i < ARRAY_SIZE(icssg_ethtool_stats); i++) {
		regmap_read(prueth->miig_rt,
			    base + icssg_ethtool_stats[i].offset,
			    &val);
		regmap_write(prueth->miig_rt,
			     base + icssg_ethtool_stats[i].offset,
			     val);

		emac->stats[i] += val;
	}

	for (j = 0; j < ARRAY_SIZE(icssg_pa_stats); j++) {
		regmap_read(prueth->pa_stats,
			    ICSSG_IET_STATS_BASE + icssg_pa_stats[j].offset,
			    &val);
		emac->stats[i + j] += val;
	}
}

void emac_stats_work_handler(struct work_struct *work)
{
	struct prueth_emac *emac = container_of(work, struct prueth_emac,
						stats_work.work);
	emac_update_hardware_stats(emac);

	queue_delayed_work(system_long_wq, &emac->stats_work,
			   msecs_to_jiffies((STATS_TIME_LIMIT_1G_MS * 1000) / emac->speed));
}

static void emac_get_ethtool_stats(struct net_device *ndev,
				   struct ethtool_stats *stats, u64 *data)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	int i;

	emac_update_hardware_stats(emac);

	for (i = 0; i < ICSSG_NUM_STATS; i++)
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

static int emac_set_channels(struct net_device *ndev,
			     struct ethtool_channels *ch)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	/* Check if interface is up. Can change the num queues when
	 * the interface is down.
	 */
	if (netif_running(emac->ndev))
		return -EBUSY;

	emac->tx_ch_num = ch->tx_count;

	return 0;
}

static void emac_get_channels(struct net_device *ndev,
			      struct ethtool_channels *ch)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	ch->max_rx = 1;
	ch->max_tx = PRUETH_MAX_TX_QUEUES;
	ch->rx_count = 1;
	ch->tx_count = emac->tx_ch_num;
}

/* TODO : This is temporary until a formal ethtool interface become available
 * in LKML to configure IET FPE.
 */
static u32 emac_get_ethtool_priv_flags(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth_qos_iet *iet = &emac->qos.iet;
	u32 priv_flags = 0;

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

static int emac_get_coalesce(struct net_device *ndev, struct ethtool_coalesce *coal,
			     struct kernel_ethtool_coalesce *kernel_coal,
			     struct netlink_ext_ack *extack)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth_tx_chn *tx_chn;

	tx_chn = &emac->tx_chns[0];

	coal->rx_coalesce_usecs = emac->rx_pace_timeout_ns / 1000;
	coal->tx_coalesce_usecs = tx_chn->tx_pace_timeout_ns / 1000;

	return 0;
}

static int emac_get_per_queue_coalesce(struct net_device *ndev, u32 queue,
				       struct ethtool_coalesce *coal)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth_tx_chn *tx_chn;

	if (queue >= PRUETH_MAX_TX_QUEUES)
		return -EINVAL;

	tx_chn = &emac->tx_chns[queue];

	coal->tx_coalesce_usecs = tx_chn->tx_pace_timeout_ns / 1000;

	return 0;
}

static int emac_set_coalesce(struct net_device *ndev, struct ethtool_coalesce *coal,
			     struct kernel_ethtool_coalesce *kernel_coal,
			     struct netlink_ext_ack *extack)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	struct prueth_tx_chn *tx_chn;

	tx_chn = &emac->tx_chns[0];

	if (coal->rx_coalesce_usecs && coal->rx_coalesce_usecs < ICSSG_MIN_COALESCE_USECS) {
		dev_info(prueth->dev, "defaulting to min value of %dus for rx-usecs\n",
			 ICSSG_MIN_COALESCE_USECS);
		coal->rx_coalesce_usecs = ICSSG_MIN_COALESCE_USECS;
	}

	if (coal->tx_coalesce_usecs && coal->tx_coalesce_usecs < ICSSG_MIN_COALESCE_USECS) {
		dev_info(prueth->dev, "defaulting to min value of %dus for tx-usecs\n",
			 ICSSG_MIN_COALESCE_USECS);
		coal->tx_coalesce_usecs = ICSSG_MIN_COALESCE_USECS;
	}

	emac->rx_pace_timeout_ns = coal->rx_coalesce_usecs * 1000;
	tx_chn->tx_pace_timeout_ns = coal->tx_coalesce_usecs * 1000;

	return 0;
}

static int emac_set_per_queue_coalesce(struct net_device *ndev, u32 queue,
				       struct ethtool_coalesce *coal)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	struct prueth_tx_chn *tx_chn;

	if (queue >= PRUETH_MAX_TX_QUEUES)
		return -EINVAL;

	tx_chn = &emac->tx_chns[queue];

	if (coal->tx_coalesce_usecs && coal->tx_coalesce_usecs < ICSSG_MIN_COALESCE_USECS) {
		dev_info(prueth->dev, "defaulting to min value of %dus for tx-usecs for tx-%u\n",
			 ICSSG_MIN_COALESCE_USECS, queue);
		coal->tx_coalesce_usecs = ICSSG_MIN_COALESCE_USECS;
	}

	tx_chn->tx_pace_timeout_ns = coal->tx_coalesce_usecs * 1000;

	return 0;
}

const struct ethtool_ops icssg_ethtool_ops = {
	.get_drvinfo = emac_get_drvinfo,
	.get_msglevel = emac_get_msglevel,
	.set_msglevel = emac_set_msglevel,
	.get_sset_count = emac_get_sset_count,
	.get_ethtool_stats = emac_get_ethtool_stats,
	.get_strings = emac_get_strings,
	.get_ts_info = emac_get_ts_info,
	.get_priv_flags = emac_get_ethtool_priv_flags,
	.set_priv_flags = emac_set_ethtool_priv_flags,
	.supported_coalesce_params = ETHTOOL_COALESCE_RX_USECS | ETHTOOL_COALESCE_TX_USECS,
	.get_coalesce           = emac_get_coalesce,
	.set_coalesce           = emac_set_coalesce,
	.get_per_queue_coalesce = emac_get_per_queue_coalesce,
	.set_per_queue_coalesce = emac_set_per_queue_coalesce,
	.get_channels = emac_get_channels,
	.set_channels = emac_set_channels,
	.get_link_ksettings = emac_get_link_ksettings,
	.set_link_ksettings = emac_set_link_ksettings,
	.get_link = ethtool_op_get_link,
	.get_eee = emac_get_eee,
	.set_eee = emac_set_eee,
	.nway_reset = emac_nway_reset,
};
