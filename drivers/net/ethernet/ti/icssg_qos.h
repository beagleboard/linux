/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 */

#ifndef __NET_TI_ICSSG_QOS_H
#define __NET_TI_ICSSG_QOS_H

#include <linux/atomic.h>
#include <linux/netdevice.h>
#include <net/pkt_sched.h>

/**
 * Maximum number of gate command entries in each list.
 */
#define TAS_MAX_CMD_LISTS   (16)

/**
 * Maximum number of transmit queues supported by implementation
 */
#define TAS_MAX_NUM_QUEUES  (8)

/**
 * Minimum cycle time supported by implementation (in ns)
 */
#define TAS_MIN_CYCLE_TIME  (1000000)

/**
 * Minimum TAS window duration supported by implementation (in ns)
 */
#define TAS_MIN_WINDOW_DURATION  (10000)

/**
 * List number 0 or 1. Also the value at memory location TAS_ACTIVE_LIST_INDEX
 */
enum tas_list_num {
	TAS_LIST0 = 0,
	TAS_LIST1 = 1
};

/**
 * state of TAS in f/w
 */
enum tas_state {
	/* PRU's are idle */
	TAS_STATE_DISABLE = 0,
	/* Enable TAS */
	TAS_STATE_ENABLE = 1,
	/* Firmware will reset the state machine */
	TAS_STATE_RESET = 2,
};

/**
 * Config state machine variables. See IEEE Std 802.1Q-2018 8.6.8.4
 */
struct tas_config_list {
	/* New list is copied at this time */
	u64 config_change_time;
	/* config change error counter, incremented if
	 * admin->BaseTime < current time and TAS_enabled is true
	 */
	u32 config_change_error_counter;
	/* True if list update is pending */
	u8 config_pending;
	/* Set to true when application trigger updating of admin list
	 * to active list, cleared when configChangeTime is updated
	 */
	u8 config_change;
};

/**
 * Max SDU table. See IEEE Std 802.1Q-2018 12.29.1.1
 */
struct tas_max_sdu_table {
	u16 max_sdu[TAS_MAX_NUM_QUEUES];
};

/**
 * TAS List Structure based on firmware memory map
 */
struct tas_firmware_list {
	/* window gate mask list */
	u8 gate_mask_list[TAS_MAX_CMD_LISTS];
	/* window end time list */
	u32 window_end_time_list[TAS_MAX_CMD_LISTS];
	/* Array of gate close time for each queue in each window */
	u32 gate_close_time_list[TAS_MAX_CMD_LISTS][TAS_MAX_NUM_QUEUES];
};

/**
 * Main Time Aware Shaper Handle
 */
struct tas_config {
	enum tas_state state;
	struct tas_max_sdu_table max_sdu_table;
	/* Config change variables */
	struct __iomem tas_config_list * config_list;
	/* Whether list 1 or list 2 is the operating list */
	u8 __iomem *active_list;
	/* active List pointer, used by firmware */
	struct __iomem tas_firmware_list * firmware_active_list;
	/* shadow List pointer, used by driver */
	struct __iomem tas_firmware_list * firmware_shadow_list;
};

struct prueth_qos_tas {
	struct tc_taprio_qopt_offload *taprio_admin;
	struct tc_taprio_qopt_offload *taprio_oper;
	struct tas_config config;
};

struct prueth_qos_iet {
	struct work_struct fpe_config_task;
	struct completion fpe_config_compl;
	struct prueth_emac *emac;
	atomic_t cancel_fpe_config;
	/* Set through priv flags to enable IET frame preemption */
	bool fpe_configured;
	/* Set if IET FPE is active */
	bool fpe_enabled;
	/* Set through priv flags to enable IET MAC Verify state machine
	 * in firmware
	 */
	bool mac_verify_configured;
};

struct prueth_qos {
	struct prueth_qos_iet iet;
	struct prueth_qos_tas tas;
};

void icssg_qos_init(struct net_device *ndev);
int icssg_qos_ndo_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			   void *type_data);
void icssg_qos_link_up(struct net_device *ndev);
void icssg_qos_link_down(struct net_device *ndev);
#endif /* __NET_TI_ICSSG_QOS_H */
