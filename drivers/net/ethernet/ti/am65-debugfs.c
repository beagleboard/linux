// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments K3 AM65 Ethernet debugfs submodule
 * Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>

#include "am65-cpsw-nuss.h"

int am65_cpsw_nuss_register_debugfs(struct am65_cpsw_common *common)
{
	common->debugfs_root = debugfs_create_dir(dev_name(common->dev), NULL);
	if (IS_ERR(common->debugfs_root))
		return PTR_ERR(common->debugfs_root);

	return 0;
}

void am65_cpsw_nuss_unregister_debugfs(struct am65_cpsw_common *common)
{
	debugfs_remove_recursive(common->debugfs_root);
}

static int
cut_thru_tx_pri_mask_get(void *data, u64 *val)
{
	struct am65_cpsw_port *port = data;
	struct am65_cpsw_cut_thru *cut_thru;
	int ret = -EINVAL;

	read_lock(&dev_base_lock);
	cut_thru = &port->qos.cut_thru;
	if (port->ndev->reg_state == NETREG_REGISTERED) {
		*val =  cut_thru->tx_pri_mask;
		ret = 0;
	}
	read_unlock(&dev_base_lock);

	return ret;
}

static int
cut_thru_tx_pri_mask_set(void *data, u64 val)
{
	struct am65_cpsw_cut_thru *cut_thru;
	struct am65_cpsw_port *port = data;
	struct am65_cpsw_common *common;
	int ret = 0;

	if (val & ~GENMASK(7, 0))
		return -EINVAL;

	if (!rtnl_trylock())
		return restart_syscall();

	common = port->common;
	cut_thru = &port->qos.cut_thru;

	if (cut_thru->enable) {
		dev_err(common->dev, "Port%u: can't set cut-thru tx_pri_mask while cut-thru enabled\n",
			port->port_id);
		ret = -EINVAL;
		goto err;
	}
	cut_thru->tx_pri_mask = val;

err:
	rtnl_unlock();
	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_cut_thru_tx_pri_mask, cut_thru_tx_pri_mask_get,
			 cut_thru_tx_pri_mask_set, "%llx\n");

static int
cut_thru_rx_pri_mask_get(void *data, u64 *val)
{
	struct am65_cpsw_port *port = data;
	struct am65_cpsw_cut_thru *cut_thru;
	int ret = -EINVAL;

	read_lock(&dev_base_lock);
	cut_thru = &port->qos.cut_thru;
	if (port->ndev->reg_state == NETREG_REGISTERED) {
		*val =  cut_thru->rx_pri_mask;
		ret = 0;
	}
	read_unlock(&dev_base_lock);

	return ret;
}

static int
cut_thru_rx_pri_mask_set(void *data, u64 val)
{
	struct am65_cpsw_cut_thru *cut_thru;
	struct am65_cpsw_port *port = data;
	struct am65_cpsw_common *common;
	int ret = 0;

	if (val & ~GENMASK(7, 0))
		return -EINVAL;

	if (!rtnl_trylock())
		return restart_syscall();

	common = port->common;
	cut_thru = &port->qos.cut_thru;

	if (cut_thru->enable) {
		dev_err(common->dev, "Port%u: can't set cut-thru rx_pri_mask while cut-thru enabled\n",
			port->port_id);
		ret = -EINVAL;
		goto err;
	}
	cut_thru->rx_pri_mask = val;

err:
	rtnl_unlock();
	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_cut_thru_rx_pri_mask, cut_thru_rx_pri_mask_get,
			 cut_thru_rx_pri_mask_set, "%llx\n");

int am65_cpsw_nuss_register_port_debugfs(struct am65_cpsw_port *port)
{
	struct am65_cpsw_common *common = port->common;
	char dirn[32];

	scnprintf(dirn, sizeof(dirn), "Port%x", port->port_id);
	port->debugfs_port = debugfs_create_dir(dirn, common->debugfs_root);
	if (IS_ERR(port->debugfs_port))
		return PTR_ERR(port->debugfs_port);

	debugfs_create_bool("disabled", 0400,
			    port->debugfs_port, &port->disabled);
	if (port->disabled)
		return 0;

	if (common->pdata.quirks & AM64_CPSW_QUIRK_CUT_THRU) {
		debugfs_create_file("cut_thru_tx_pri_mask", 0600,
				    port->debugfs_port,
				    port, &fops_cut_thru_tx_pri_mask);
		debugfs_create_file("cut_thru_rx_pri_mask", 0600,
				    port->debugfs_port,
				    port, &fops_cut_thru_rx_pri_mask);
	}

	return 0;
}
