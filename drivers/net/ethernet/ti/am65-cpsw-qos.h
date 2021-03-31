/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 */

#ifndef AM65_CPSW_QOS_H_
#define AM65_CPSW_QOS_H_

#include <linux/netdevice.h>
#include <net/pkt_sched.h>
#include <net/pkt_cls.h>

struct am65_cpsw_port;
struct am65_cpsw_common;

struct am65_cpsw_est {
	int buf;
	/* has to be the last one */
	struct tc_taprio_qopt_offload taprio;
};

struct am65_cpsw_iet {
	struct net_device *ndev;
	/* Set through priv flags */
	bool fpe_configured;
	bool mac_verify_configured;
	/* frame preemption enabled */
	bool fpe_enabled;
	/* configured mask */
	u32 fpe_mask_configured;
	/* current mask */
	u32 mask;
};

struct am65_cpsw_mqprio {
	struct tc_mqprio_qopt_offload mqprio_hw;
	u64 max_rate_total;
	u32 tx_prio_map;

	unsigned enable:1;
	unsigned shaper_en:1;
	unsigned shaper_susp:1;
	unsigned tc0_q:3;
};

struct am65_cpsw_cut_thru {
	unsigned int rx_pri_mask;
	unsigned int tx_pri_mask;
	bool enable;
};

struct am65_cpsw_qos {
	struct am65_cpsw_est *est_admin;
	struct am65_cpsw_est *est_oper;
	ktime_t link_down_time;
	int link_speed;
	int duplex;
	struct am65_cpsw_iet iet;
	struct am65_cpsw_mqprio mqprio;
	struct am65_cpsw_cut_thru cut_thru;
};

int am65_cpsw_qos_ndo_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			       void *type_data);
void am65_cpsw_qos_link_up(struct net_device *ndev, int link_speed, int duplex);
void am65_cpsw_qos_link_down(struct net_device *ndev);
void am65_cpsw_qos_iet_init(struct net_device *ndev);
void am65_cpsw_qos_iet_cleanup(struct net_device *ndev);
void am65_cpsw_qos_cut_thru_init(struct am65_cpsw_port *port);
void am65_cpsw_qos_cut_thru_cleanup(struct am65_cpsw_port *port);
int am65_cpsw_qos_ndo_tx_p0_set_maxrate(struct net_device *ndev,
					int queue, u32 rate_mbps);
void am65_cpsw_qos_tx_p0_rate_init(struct am65_cpsw_common *common);
void am65_cpsw_qos_mqprio_init(struct am65_cpsw_port *port);

#endif /* AM65_CPSW_QOS_H_ */
