/*
 * NetCP ethss header file
 *
 * Copyright (C) 2014 - 2016 Texas Instruments Incorporated
 * Authors:	Sandeep Nair <sandeep_n@ti.com>
 *		Sandeep Paulraj <s-paulraj@ti.com>
 *		Cyril Chemparathy <cyril@ti.com>
 *		Santosh Shilimkar <santosh.shilimkar@ti.com>
 *		Wingman Kwok <w-kwok2@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __NETCP_ETHSS_H__
#define __NETCP_ETHSS_H__

#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/if_vlan.h>
#include <linux/io.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/phy/phy.h>
#include <linux/spinlock.h>
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/ethtool.h>

#include "cpsw_ale.h"
#include "netcp.h"

#define MAX_NUM_SERDES				2
#define MAX_SLAVES				8

struct gbe_ss_regs_ofs {
	u16	id_ver;
	u16	control;
	u16	rgmii_status; /* 2U */
};

struct gbe_switch_regs_ofs {
	u16	id_ver;
	u16	control;
	u16	soft_reset;
	u16	emcontrol;
	u16	stat_port_en;
	u16	ptype;
	u16	flow_control;
};

struct gbe_port_regs_ofs {
	u16	port_vlan;
	u16	tx_pri_map;
	u16	rx_pri_map;
	u16	sa_lo;
	u16	sa_hi;
	u16	ts_ctl;
	u16	ts_seq_ltype;
	u16	ts_vlan;
	u16	ts_ctl_ltype2;
	u16	ts_ctl2;
	u16	rx_maxlen;	/* 2U, NU */
};

struct gbe_host_port_regs_ofs {
	u16	port_vlan;
	u16	tx_pri_map;
	u16	rx_maxlen;
};

struct gbe_emac_regs_ofs {
	u16	mac_control;
	u16	soft_reset;
	u16	rx_maxlen;
};

#define GBE_MAX_HW_STAT_MODS			9

struct gbe_priv {
	struct device			*dev;
	struct netcp_device		*netcp_device;
	struct timer_list		timer;
	u32				num_slaves;
	u32				ale_entries;
	u32				ale_ports;
	bool				enable_ale;
	u8				max_num_slaves;
	u8				max_num_ports; /* max_num_slaves + 1 */
	u8				num_stats_mods;
	struct netcp_tx_pipe		tx_pipe;

	int				host_port;
	u32				rx_packet_max;
	u32				ss_version;
	u32				stats_en_mask;

	struct regmap			*ss_regmap;
	struct regmap			*pcsr_regmap;
	void __iomem                    *ss_regs;
	void __iomem			*switch_regs;
	void __iomem			*host_port_regs;
	void __iomem			*ale_reg;
	void __iomem			*sgmii_port_regs;
	void __iomem			*sgmii_port34_regs;
	void __iomem			*hw_stats_regs[GBE_MAX_HW_STAT_MODS];

	struct gbe_ss_regs_ofs		ss_regs_ofs;
	struct gbe_switch_regs_ofs	switch_regs_ofs;
	struct gbe_host_port_regs_ofs	host_port_regs_ofs;

	struct cpsw_ale			*ale;
	unsigned int			tx_queue_id;
	const char			*dma_chan_name;

	struct list_head		gbe_intf_head;
	struct list_head		secondary_slaves;
	struct net_device		*dummy_ndev;

	u64				*hw_stats;
	u32				*hw_stats_prev;
	const struct netcp_ethtool_stat *et_stats;
	int				num_et_stats;
	/*  Lock for updating the hwstats */
	spinlock_t			hw_stats_lock;

	struct kobject			kobj;
	struct kobject			tx_pri_kobj;
	struct kobject			pvlan_kobj;
	struct kobject			port_ts_kobj[MAX_SLAVES];
	struct kobject			stats_kobj;
};

struct gbe_slave {
	struct gbe_priv			*gbe_dev;
	void __iomem			*port_regs;
	void __iomem			*emac_regs;
	struct gbe_port_regs_ofs	port_regs_ofs;
	struct gbe_emac_regs_ofs	emac_regs_ofs;
	int				slave_num; /* 0 based logical number */
	int				port_num;  /* actual port number */
	atomic_t			link_state;
	bool				open;
	struct phy_device		*phy;
	u32				link_interface;
	u32				mac_control;
	u8				phy_port_t;
					/* work trigger threshold
					 *   0: triger disabled
					 * > 1: trigger enabled
					 */
	u32				link_recover_thresh;
					/* 0:NOT, > 0:recovering */
	u32				link_recovering;
	struct delayed_work		link_recover_work;
	struct device_node		*phy_node;
	struct list_head		slave_list;
	struct phy			*serdes_phy;
};

struct gbe_intf {
	struct net_device	*ndev;
	struct device		*dev;
	struct gbe_priv		*gbe_dev;
	struct netcp_tx_pipe	tx_pipe;
	struct gbe_slave	*slave;
	struct list_head	gbe_intf_list;
	unsigned long		active_vlans[BITS_TO_LONGS(VLAN_N_VID)];
};

int gbe_create_sysfs_entries(struct gbe_priv *gbe_dev);
void gbe_remove_sysfs_entries(struct gbe_priv *gbe_dev);
void gbe_reset_mod_stats(struct gbe_priv *gbe_dev, int stats_mod);
void gbe_reset_mod_stats_ver14(struct gbe_priv *gbe_dev, int stats_mod);

#define for_each_intf(i, priv) \
	list_for_each_entry((i), &(priv)->gbe_intf_head, gbe_intf_list)

#define GBE_REG_ADDR(p, rb, rn) (p->rb + p->rb##_ofs.rn)
#define GBE_MAJOR_VERSION(reg)		(reg >> 8 & 0x7)
#define GBE_MINOR_VERSION(reg)		(reg & 0xff)
#define GBE_RTL_VERSION(reg)		((reg >> 11) & 0x1f)
#define GBE_IDENT(reg)			((reg >> 16) & 0xffff)
#define GBE_SS_ID_NU			0x4ee6
#define GBE_SS_ID_2U			0x4ee8

#define IS_SS_ID_MU(d) \
	((GBE_IDENT((d)->ss_version) == GBE_SS_ID_NU) || \
	 (GBE_IDENT((d)->ss_version) == GBE_SS_ID_2U))

#define IS_SS_ID_NU(d) \
	(GBE_IDENT((d)->ss_version) == GBE_SS_ID_NU)

#define IS_SS_ID_2U(d) \
	(GBE_IDENT((d)->ss_version) == GBE_SS_ID_2U)

#define GBE_STATSA_MODULE			0
#define GBE_STATSB_MODULE			1
#define GBE_STATSC_MODULE			2
#define GBE_STATSD_MODULE			3

#define GBENU_STATS0_MODULE			0
#define GBENU_STATS1_MODULE			1
#define GBENU_STATS2_MODULE			2
#define GBENU_STATS3_MODULE			3
#define GBENU_STATS4_MODULE			4
#define GBENU_STATS5_MODULE			5
#define GBENU_STATS6_MODULE			6
#define GBENU_STATS7_MODULE			7
#define GBENU_STATS8_MODULE			8

#define XGBE_STATS0_MODULE			0
#define XGBE_STATS1_MODULE			1
#define XGBE_STATS2_MODULE			2

#define XGBE_SS_VERSION_10		0x4ee42100
#define GBE_SS_VERSION_14		0x4ed21104

#endif /* __NETCP_ETHSS_H */
