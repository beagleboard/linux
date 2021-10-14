// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments ICSSG PRUETH QoS submodule
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <linux/printk.h>
#include "icssg_prueth.h"
#include "icssg_switch_map.h"

static void icssg_qos_tas_init(struct net_device *ndev);

void icssg_qos_init(struct net_device *ndev)
{
	icssg_qos_tas_init(ndev);

	/* IET init goes here */
}

void icssg_qos_cleanup(struct net_device *ndev)
{
	/* IET cleanup goes here */
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
