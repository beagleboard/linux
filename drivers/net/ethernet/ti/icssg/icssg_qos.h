/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
 */

#ifndef __NET_TI_ICSSG_QOS_H
#define __NET_TI_ICSSG_QOS_H

#include <linux/atomic.h>
#include <linux/netdevice.h>
#include <net/pkt_sched.h>

/* Maximum number of gate command entries in each list. */
#define TAS_MAX_CMD_LISTS   (16)

/* Maximum number of transmit queues supported by implementation */
#define TAS_MAX_NUM_QUEUES  (8)

/* Minimum cycle time supported by implementation (in ns) */
#define TAS_MIN_CYCLE_TIME  (1000000)

/* Minimum cycle time supported by implementation (in ns) */
#define TAS_MAX_CYCLE_TIME  (4000000000)

/* Minimum TAS window duration supported by implementation (in ns) */
#define TAS_MIN_WINDOW_DURATION  (10000)

/**
 * enum tas_list_num - TAS list number
 * @TAS_LIST0: TAS list number is 0
 * @TAS_LIST1: TAS list number is 1
 */
enum tas_list_num {
	TAS_LIST0 = 0,
	TAS_LIST1 = 1
};

/**
 * enum tas_state - State of TAS in firmware
 * @TAS_STATE_DISABLE: TAS state machine is disabled.
 * @TAS_STATE_ENABLE: TAS state machine is enabled.
 * @TAS_STATE_RESET: TAS state machine is reset.
 */
enum tas_state {
	TAS_STATE_DISABLE = 0,
	TAS_STATE_ENABLE = 1,
	TAS_STATE_RESET = 2,
};

/**
 * struct tas_config_list - Config state machine variables
 * @config_change_time: New list is copied at this time
 * @config_change_error_counter: Incremented if admin->BaseTime < current time
 *				 and TAS_enabled is true
 * @config_pending: True if list update is pending
 * @config_change: Set to true when application trigger updating of admin list
 *		   to active list, cleared when configChangeTime is updated
 */
struct __packed tas_config_list {
	u64 config_change_time;
	u32 config_change_error_counter;
	u8 config_pending;
	u8 config_change;
};

/* Max SDU table. See IEEE Std 802.1Q-2018 12.29.1.1 */
struct tas_max_sdu_table {
	u16 max_sdu[TAS_MAX_NUM_QUEUES];
};

/**
 * struct tas_firmware_list - TAS List Structure based on firmware memory map
 * @gate_mask_list: Window gate mask list
 * @win_end_time_list: Window end time list
 * @gate_close_time_list: Array of gate close time for each queue in each window
 */
struct __packed tas_firmware_list {
	u8 gate_mask_list[TAS_MAX_CMD_LISTS];
	u32 win_end_time_list[TAS_MAX_CMD_LISTS];
	u32 gate_close_time_list[TAS_MAX_CMD_LISTS][TAS_MAX_NUM_QUEUES];
};

/**
 * struct tas_config - Main Time Aware Shaper Handle
 * @state: TAS state
 * @max_sdu_table: Max SDU table
 * @config_list: Config change variables
 * @active_list: Current operating list operating list
 * @fw_active_list: Active List pointer, used by firmware
 * @fw_shadow_list: Shadow List pointer, used by driver
 */
struct tas_config {
	enum tas_state state;
	struct tas_max_sdu_table max_sdu_table;
	struct tas_config_list __iomem *config_list;
	u8 __iomem *active_list;
	struct tas_firmware_list __iomem *fw_active_list;
	struct tas_firmware_list __iomem *fw_shadow_list;
};

struct prueth_qos_tas {
	struct tc_taprio_qopt_offload *taprio_admin;
	struct tas_config config;
};

struct prueth_qos_iet {
	struct work_struct fpe_config_task;
	struct completion fpe_config_compl;
	struct prueth_emac *emac;
	atomic_t cancel_fpe_config;
	/* Set through priv flags to enable IET frame preemption */
	bool fpe_configured;
	/* Set through priv flags to enable IET MAC Verify state machine
	 * in firmware
	 */
	bool mac_verify_configured;
	/* Min TX fragment size, set via ethtool */
	u32 tx_min_frag_size;
	/* wait time between verification attempts in ms (according to clause
	 * 30.14.1.6 aMACMergeVerifyTime), set via ethtool
	 */
	u32 verify_time_ms;
	/* Set if IET FPE is active */
	bool fpe_enabled;
};

struct prueth_qos {
	struct prueth_qos_tas tas;
	struct prueth_qos_iet iet;
};

void icssg_qos_init(struct net_device *ndev);
int icssg_qos_ndo_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			   void *type_data);
void icssg_qos_link_up(struct net_device *ndev);
void icssg_qos_link_down(struct net_device *ndev);
#endif /* __NET_TI_ICSSG_QOS_H */
