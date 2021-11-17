// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments ICSSG PRUETH QoS submodule
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <linux/printk.h>
#include "icssg_prueth.h"
#include "icssg_switch_map.h"

/* in msec */
#define ICSSG_IET_FPE_VERIFY_TIMEOUT_MS	1000

static void icssg_qos_tas_init(struct net_device *ndev);
static void icssg_prueth_iet_fpe_disable(struct prueth_qos_iet *iet);
static int icssg_prueth_iet_fpe_enable(struct prueth_emac *emac);
static void icssg_prueth_iet_fpe_disable(struct prueth_qos_iet *iet);
static void icssg_qos_enable_ietfpe(struct work_struct *work);

void icssg_qos_init(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth_qos_iet *iet = &emac->qos.iet;

	icssg_qos_tas_init(ndev);

	if (!iet->fpe_configured)
		return;

	/* Init work queue for IET MAC verify process */
	iet->emac = emac;
	INIT_WORK(&iet->fpe_config_task, icssg_qos_enable_ietfpe);
	init_completion(&iet->fpe_config_compl);

	/* As worker may be sleeping, check this flag to abort
	 * as soon as it comes of out of sleep and cancel the
	 * fpe config task.
	 */
	atomic_set(&iet->cancel_fpe_config, 0);
}

static void tas_update_fw_list_pointers(struct prueth_emac *emac)
{
	struct tas_config *tas = &emac->qos.tas.config;

	if ((readb(tas->active_list)) == TAS_LIST0) {
		tas->firmware_active_list = emac->dram.va + TAS_GATE_MASK_LIST0;
		tas->firmware_shadow_list = emac->dram.va + TAS_GATE_MASK_LIST1;
	} else {
		tas->firmware_active_list = emac->dram.va + TAS_GATE_MASK_LIST1;
		tas->firmware_shadow_list = emac->dram.va + TAS_GATE_MASK_LIST0;
	}
}

void icssg_qos_link_up(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth_qos_iet *iet = &emac->qos.iet;

	if (!iet->fpe_configured)
		return;

	icssg_prueth_iet_fpe_enable(emac);
}

void icssg_qos_link_down(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth_qos_iet *iet = &emac->qos.iet;

	if (iet->fpe_configured)
		icssg_prueth_iet_fpe_disable(iet);
}
static void tas_update_maxsdu_table(struct prueth_emac *emac)
{
	struct tas_config *tas = &emac->qos.tas.config;
	u16 *max_sdu_tbl_ptr;
	u8 gate_idx;

	/* update the maxsdu table */
	max_sdu_tbl_ptr = emac->dram.va + TAS_QUEUE_MAX_SDU_LIST;

	for (gate_idx = 0; gate_idx < TAS_MAX_NUM_QUEUES; gate_idx++)
		writew(tas->max_sdu_table.max_sdu[gate_idx], &max_sdu_tbl_ptr[gate_idx]);
}

static void tas_reset(struct prueth_emac *emac)
{
	struct tas_config *tas = &emac->qos.tas.config;
	int i;

	for (i = 0; i < TAS_MAX_NUM_QUEUES; i++)
		tas->max_sdu_table.max_sdu[i] = 2048;

	tas_update_maxsdu_table(emac);

	writeb(TAS_LIST0, tas->active_list);

	memset_io(tas->firmware_active_list, 0, sizeof(*tas->firmware_active_list));
	memset_io(tas->firmware_shadow_list, 0, sizeof(*tas->firmware_shadow_list));
}

static int tas_set_state(struct prueth_emac *emac, enum tas_state state)
{
	struct tas_config *tas = &emac->qos.tas.config;
	int ret;

	if (tas->state == state)
		return 0;

	switch (state) {
	case TAS_STATE_RESET:
		tas_reset(emac);
		ret = emac_set_port_state(emac, ICSSG_EMAC_PORT_TAS_RESET);
		tas->state = TAS_STATE_RESET;
		break;
	case TAS_STATE_ENABLE:
		ret = emac_set_port_state(emac, ICSSG_EMAC_PORT_TAS_ENABLE);
		tas->state = TAS_STATE_ENABLE;
		break;
	case TAS_STATE_DISABLE:
		ret = emac_set_port_state(emac, ICSSG_EMAC_PORT_TAS_DISABLE);
		tas->state = TAS_STATE_DISABLE;
		break;
	default:
		netdev_err(emac->ndev, "%s: unsupported state\n", __func__);
		ret = -EINVAL;
		break;
	}

	if (ret)
		netdev_err(emac->ndev, "TAS set state failed %d\n", ret);
	return ret;
}

static int tas_set_trigger_list_change(struct prueth_emac *emac)
{
	struct tc_taprio_qopt_offload *admin_list = emac->qos.tas.taprio_admin;
	struct tas_config *tas = &emac->qos.tas.config;
	struct ptp_system_timestamp sts;
	u32 change_cycle_count;
	u32 cycle_time;
	u64 base_time;
	u64 cur_time;

	cycle_time = admin_list->cycle_time - 4; /* -4ns to compensate for IEP wraparound time */
	base_time = admin_list->base_time;
	cur_time = prueth_iep_gettime(emac, &sts);

	if (base_time > cur_time)
		change_cycle_count = DIV_ROUND_UP_ULL(base_time - cur_time, cycle_time);
	else
		change_cycle_count = 1;

	writel(cycle_time, emac->dram.va + TAS_ADMIN_CYCLE_TIME);
	writel(change_cycle_count, emac->dram.va + TAS_CONFIG_CHANGE_CYCLE_COUNT);
	writeb(admin_list->num_entries, emac->dram.va + TAS_ADMIN_LIST_LENGTH);

	/* config_change cleared by f/w to ack reception of new shadow list */
	writeb(1, &tas->config_list->config_change);
	/* config_pending cleared by f/w when new shadow list is copied to active list */
	writeb(1, &tas->config_list->config_pending);

	return emac_set_port_state(emac, ICSSG_EMAC_PORT_TAS_TRIGGER);
}

static int tas_update_oper_list(struct prueth_emac *emac)
{
	struct tas_config *tas = &emac->qos.tas.config;
	struct tc_taprio_qopt_offload *admin_list = emac->qos.tas.taprio_admin;
	int ret;
	u8 win_idx, gate_idx, val;
	u32 tas_acc_gate_close_time = 0;

	tas_update_fw_list_pointers(emac);

	for (win_idx = 0; win_idx < admin_list->num_entries; win_idx++) {
		tas->firmware_shadow_list->gate_mask_list[win_idx] = admin_list->entries[win_idx].gate_mask;
		tas_acc_gate_close_time += admin_list->entries[win_idx].interval;

		/* extend last entry till end of cycle time */
		if (win_idx == admin_list->num_entries - 1)
			tas->firmware_shadow_list->window_end_time_list[win_idx] = admin_list->cycle_time;
		else
			tas->firmware_shadow_list->window_end_time_list[win_idx] = tas_acc_gate_close_time;
	}

	/* clear remaining entries */
	for (win_idx = admin_list->num_entries; win_idx < TAS_MAX_CMD_LISTS; win_idx++) {
		tas->firmware_shadow_list->gate_mask_list[win_idx] = 0;
		tas->firmware_shadow_list->window_end_time_list[win_idx] = 0;
	}

	/* update the Array of gate close time for each queue in each window */
	for (win_idx = 0 ; win_idx < admin_list->num_entries; win_idx++) {
		/* On Linux, only PRUETH_MAX_TX_QUEUES are supported per port */
		for (gate_idx = 0; gate_idx < PRUETH_MAX_TX_QUEUES; gate_idx++) {
			u32 gate_close_time = 0;

			if (tas->firmware_shadow_list->gate_mask_list[win_idx] & BIT(gate_idx))
				gate_close_time = tas->firmware_shadow_list->window_end_time_list[win_idx];

			tas->firmware_shadow_list->gate_close_time_list[win_idx][gate_idx] = gate_close_time;
		}
	}

	/* tell f/w to swap active & shadow list */
	ret = tas_set_trigger_list_change(emac);
	if (ret) {
		netdev_err(emac->ndev, "failed to swap f/w config list: %d\n", ret);
		return ret;
	}

	/* Wait for completion */
	ret = readb_poll_timeout(&tas->config_list->config_change, val, !val,
				 USEC_PER_MSEC, 10 * USEC_PER_MSEC);
	if (ret) {
		netdev_err(emac->ndev, "TAS list change completion time out\n");
		return ret;
	}

	tas_update_fw_list_pointers(emac);

	return 0;
}

static int emac_set_taprio(struct prueth_emac *emac)
{
	int ret;
	struct tc_taprio_qopt_offload *taprio = emac->qos.tas.taprio_admin;

	if (!taprio->enable)
		return tas_set_state(emac, TAS_STATE_DISABLE);

	ret = tas_update_oper_list(emac);
	if (ret)
		return ret;

	return tas_set_state(emac, TAS_STATE_ENABLE);
}

static void emac_cp_taprio(struct tc_taprio_qopt_offload *from,
			   struct tc_taprio_qopt_offload *to)
{
	int i;

	*to = *from;
	for (i = 0; i < from->num_entries; i++)
		to->entries[i] = from->entries[i];
}

static int emac_setup_taprio(struct net_device *ndev, struct tc_taprio_qopt_offload *taprio)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct tc_taprio_qopt_offload *est_new;
	int ret, win_idx;

	if (!netif_running(ndev)) {
		netdev_err(ndev, "interface is down, link speed unknown\n");
		return -ENETDOWN;
	}

	if (taprio->cycle_time_extension) {
		netdev_err(ndev, "Failed to set cycle time extension");
		return -EOPNOTSUPP;
	}

	if (taprio->num_entries == 0 ||
	    taprio->num_entries > TAS_MAX_CMD_LISTS) {
		netdev_err(ndev, "unsupported num_entries %ld in taprio config\n",
			   taprio->num_entries);
		return -EINVAL;
	}

	/* If any time_interval is 0 in between the list, then exit */
	for (win_idx = 0; win_idx < taprio->num_entries; win_idx++) {
		if (taprio->entries[win_idx].interval == 0) {
			netdev_err(ndev, "0 interval in taprio config not supported\n");
			return -EINVAL;
		}
	}

	if (emac->qos.tas.taprio_admin)
		devm_kfree(&ndev->dev, emac->qos.tas.taprio_admin);

	est_new = devm_kzalloc(&ndev->dev,
			       struct_size(est_new, entries, taprio->num_entries),
			       GFP_KERNEL);
	emac_cp_taprio(taprio, est_new);
	emac->qos.tas.taprio_admin = est_new;
	ret = emac_set_taprio(emac);
	if (ret)
		devm_kfree(&ndev->dev, est_new);

	return ret;
}

int icssg_qos_ndo_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			   void *type_data)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (emac->prueth->is_sr1)
		return -EOPNOTSUPP;

	switch (type) {
	case TC_SETUP_QDISC_TAPRIO:
		return emac_setup_taprio(ndev, type_data);
	default:
		return -EOPNOTSUPP;
	}
}

static void icssg_qos_tas_init(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct tas_config *tas = &emac->qos.tas.config;
	bool need_setup = false;

	if (emac->prueth->is_sr1)
		return;

	if (tas->state == TAS_STATE_ENABLE)
		need_setup = true;

	tas->config_list = emac->dram.va + TAS_CONFIG_CHANGE_TIME;
	tas->active_list = emac->dram.va + TAS_ACTIVE_LIST_INDEX;

	tas_update_fw_list_pointers(emac);

	tas_set_state(emac, TAS_STATE_RESET);

	if (need_setup)
		emac_set_taprio(emac);
}

static int icssg_config_ietfpe(struct prueth_qos_iet *iet, bool enable)
{
	void *config = iet->emac->dram.va + ICSSG_CONFIG_OFFSET;
	u8 val;
	int ret, i;

	/* If FPE is to be enabled, first configure MAC Verify state
	 * machine in firmware as firmware kicks the Verify process
	 * as soon as ICSSG_EMAC_PORT_PREMPT_TX_ENABLE command is
	 * received.
	 */
	if (enable && iet->mac_verify_configured) {
		writeb(1, config + PRE_EMPTION_ENABLE_VERIFY);
		/* should be a multiple of 64. TODO to configure
		 * through ethtool.
		 */
		writew(64, config + PRE_EMPTION_ADD_FRAG_SIZE_LOCAL);
		writel(ICSSG_IET_FPE_VERIFY_TIMEOUT_MS, config + PRE_EMPTION_VERIFY_TIME);
	}

	/* Send command to enable FPE Tx side. Rx is always enabled */
	ret = emac_set_port_state(iet->emac,
				  enable ? ICSSG_EMAC_PORT_PREMPT_TX_ENABLE :
					   ICSSG_EMAC_PORT_PREMPT_TX_DISABLE);
	if (ret) {
		netdev_err(iet->emac->ndev, "TX pre-empt %s command failed\n",
			   enable ? "enable" : "disable");
		writeb(0, config + PRE_EMPTION_ENABLE_VERIFY);
		return ret;
	}

	/* Update FPE Tx enable bit. Assume firmware use this bit
	 * and enable PRE_EMPTION_ACTIVE_TX if everything looks
	 * good at firmware
	 */
	writeb(enable ? 1 : 0, config + PRE_EMPTION_ENABLE_TX);

	if (enable && iet->mac_verify_configured) {
		ret = readb_poll_timeout(config + PRE_EMPTION_VERIFY_STATUS, val,
					 (val == ICSSG_IETFPE_STATE_SUCCEEDED),
					 USEC_PER_MSEC, 5 * USEC_PER_SEC);
		if (ret == -ETIMEDOUT) {
			netdev_err(iet->emac->ndev,
				   "timeout for MAC Verify: status %x\n",
				   val);
			return ret;
		}
	} else {
		/* Give f/w some time to update PRE_EMPTION_ACTIVE_TX state */
		usleep_range(100, 200);
	}

	if (enable) {
		val = readb(config + PRE_EMPTION_ACTIVE_TX);
		if (val != 1) {
			netdev_err(iet->emac->ndev,
				   "F/w fails to activate IET/FPE\n");
			writeb(0, config + PRE_EMPTION_ENABLE_TX);
			return -ENODEV;
		}
	} else {
		return ret;
	}

	/* Configure highest queue as express. Set Bit 4 for FPE,
	 * Reset for express
	 */

	/* first set all 8 queues as Pre-emptive */
	for (i = 0; i < PRUETH_MAX_TX_QUEUES * PRUETH_NUM_MACS; i++)
		writeb(BIT(4), config + EXPRESS_PRE_EMPTIVE_Q_MAP + i);

	/* set highest priority channel queue as express */
	writeb(0, config + EXPRESS_PRE_EMPTIVE_Q_MAP + iet->emac->tx_ch_num - 1);

	/* set up queue mask for FPE. 1 means express */
	writeb(BIT(iet->emac->tx_ch_num - 1), config + EXPRESS_PRE_EMPTIVE_Q_MASK);

	iet->fpe_enabled = true;

	return ret;
}

static void icssg_qos_enable_ietfpe(struct work_struct *work)
{
	struct prueth_qos_iet *iet =
		container_of(work, struct prueth_qos_iet, fpe_config_task);
	int ret;

	/* Set the required flag and send a command to ICSSG firmware to
	 * enable FPE and start MAC verify
	 */
	ret = icssg_config_ietfpe(iet, true);

	/* if verify configured, poll for the status and complete.
	 * Or just do completion
	 */
	if (!ret)
		netdev_err(iet->emac->ndev, "IET FPE configured successfully\n");
	else
		netdev_err(iet->emac->ndev, "IET FPE config error\n");
	complete(&iet->fpe_config_compl);
}

static void icssg_prueth_iet_fpe_disable(struct prueth_qos_iet *iet)
{
	int ret;

	atomic_set(&iet->cancel_fpe_config, 1);
	cancel_work_sync(&iet->fpe_config_task);
	ret = icssg_config_ietfpe(iet, false);
	if (!ret)
		netdev_err(iet->emac->ndev, "IET FPE disabled successfully\n");
	else
		netdev_err(iet->emac->ndev, "IET FPE disable failed\n");
}

static int icssg_prueth_iet_fpe_enable(struct prueth_emac *emac)
{
	struct prueth_qos_iet *iet = &emac->qos.iet;
	int ret;

	/* Schedule MAC Verify and enable IET FPE if configured */
	atomic_set(&iet->cancel_fpe_config, 0);
	reinit_completion(&iet->fpe_config_compl);
	schedule_work(&iet->fpe_config_task);
	/* By trial, found it takes about 1.5s. So
	 * wait for 10s
	 */
	ret = wait_for_completion_timeout(&iet->fpe_config_compl,
					  msecs_to_jiffies(10000));
	if (!ret) {
		netdev_err(emac->ndev,
			   "IET verify completion timeout\n");
		/* cancel verify in progress */
		atomic_set(&iet->cancel_fpe_config, 1);
		cancel_work_sync(&iet->fpe_config_task);
	}

	return ret;
}
