// SPDX-License-Identifier: GPL-2.0
/* CPSW switch-configuration using non-standard private ioctl SIOCSWITCHCONFIG
 * Grygorii Strashko <grygorii.strashko@ti.com>:
 *  moved code in separate file to minimize merge conflicts with LKML
 */

static int cpsw_set_port_state(struct cpsw_priv *priv, int port,
			       int port_state)
{
	switch (port_state) {
	case PORT_STATE_DISABLED:
		priv->port_state[port] = ALE_PORT_STATE_DISABLE;
		break;
	case PORT_STATE_BLOCKED:
		priv->port_state[port] = ALE_PORT_STATE_BLOCK;
		break;
	case PORT_STATE_LEARN:
		priv->port_state[port] = ALE_PORT_STATE_LEARN;
		break;
	case PORT_STATE_FORWARD:
		priv->port_state[port] = ALE_PORT_STATE_FORWARD;
		break;
	default:
		dev_err(priv->dev, "Switch config: Invalid port state\n");
		return -EINVAL;
	}
	return cpsw_ale_control_set(priv->cpsw->ale, port, ALE_PORT_STATE,
			priv->port_state[port]);
}

static int cpsw_switch_config_ioctl(struct net_device *ndev,
				    struct ifreq *ifrq, int cmd)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	struct cpsw_common *cpsw = priv->cpsw;
	struct net_switch_config config;
	int ret = -EINVAL;

	if (cpsw->data.dual_emac) {
		dev_err(priv->dev, "CPSW not in switch mode\n");
		return -EOPNOTSUPP;
	}

	/* Only SIOCSWITCHCONFIG is used as cmd argument and hence, there is no
	 * switch statement required.
	 * Function calls are based on switch_config.cmd
	 */

	if (copy_from_user(&config, ifrq->ifr_data, sizeof(config)))
		return -EFAULT;

	if (config.vid > 4095) {
		dev_err(priv->dev, "Invalid VLAN id Arguments for cmd %d\n",
			config.cmd);
		return ret;
	}

	switch (config.cmd) {
	case SWITCH_ADD_MULTICAST:
		if (config.port > 0 && config.port <= 7 &&
		    is_multicast_ether_addr(config.addr)) {
			ret = cpsw_ale_add_mcast(cpsw->ale, config.addr,
						 config.port, ALE_VLAN,
						 config.vid, 0);
		} else {
			dev_err(priv->dev, "Invalid Arguments for cmd %d\n",
				config.cmd);
		}
		break;
	case SWITCH_DEL_MULTICAST:
		if (is_multicast_ether_addr(config.addr)) {
			ret = cpsw_ale_del_mcast(cpsw->ale, config.addr,
						 0, ALE_VLAN, config.vid);
		} else {
			dev_err(priv->dev, "Invalid Arguments for cmd %d\n",
				config.cmd);
		}
		break;
	case SWITCH_ADD_VLAN:
		if (config.port > 0 && config.port <= 7) {
			ret = cpsw_ale_add_vlan(cpsw->ale, config.vid,
						config.port,
						config.untag_port,
						config.reg_multi,
						config.unreg_multi);
		} else {
			dev_err(priv->dev, "Invalid Arguments for cmd %d\n",
				config.cmd);
		}
		break;
	case SWITCH_DEL_VLAN:
		ret = cpsw_ale_del_vlan(cpsw->ale, config.vid, 0);
		break;
	case SWITCH_SET_PORT_CONFIG:
	{
		struct phy_device *phy = NULL;
		struct ethtool_link_ksettings cmd;

		if (config.port == 1 || config.port == 2)
			phy = cpsw->slaves[config.port - 1].phy;

		if (!phy) {
			dev_err(priv->dev, "Phy not Found\n");
			break;
		}

		convert_legacy_settings_to_link_ksettings(&cmd, &config.ecmd);
		cmd.base.phy_address = phy->mdio.addr;
		ret = phy_ethtool_ksettings_set(phy, &cmd);
		break;
	}
	case SWITCH_GET_PORT_CONFIG:
	{
		struct phy_device *phy = NULL;
		struct ethtool_link_ksettings cmd;

		if (config.port == 1 || config.port == 2)
			phy = cpsw->slaves[config.port - 1].phy;

		if (!phy) {
			dev_err(priv->dev, "Phy not Found\n");
			break;
		}

		cmd.base.phy_address = phy->mdio.addr;
		phy_ethtool_ksettings_get(phy, &cmd);
		convert_link_ksettings_to_legacy_settings(&config.ecmd, &cmd);

		ret = copy_to_user(ifrq->ifr_data, &config, sizeof(config));
		break;
	}
	case SWITCH_ADD_UNKNOWN_VLAN_INFO:
		if (config.unknown_vlan_member <= 7 &&
		    config.unknown_vlan_untag <= 7 &&
		    config.unknown_vlan_unreg_multi <= 7 &&
		    config.unknown_vlan_reg_multi <= 7) {
			cpsw_ale_control_set(cpsw->ale, 0,
					     ALE_PORT_UNTAGGED_EGRESS,
					     config.unknown_vlan_untag);
			cpsw_ale_control_set(cpsw->ale, 0,
					     ALE_PORT_UNKNOWN_REG_MCAST_FLOOD,
					     config.unknown_vlan_reg_multi);
			cpsw_ale_control_set(cpsw->ale, 0,
					     ALE_PORT_UNKNOWN_MCAST_FLOOD,
					     config.unknown_vlan_unreg_multi);
			cpsw_ale_control_set(cpsw->ale, 0,
					     ALE_PORT_UNKNOWN_VLAN_MEMBER,
					     config.unknown_vlan_member);
			ret = 0;
		} else {
			dev_err(priv->dev, "Invalid Unknown VLAN Arguments\n");
		}
		break;
	case SWITCH_GET_PORT_STATE:
		if (config.port == 1 || config.port == 2) {
			config.port_state = priv->port_state[config.port];
			ret = copy_to_user(ifrq->ifr_data, &config,
					   sizeof(config));
		} else {
			dev_err(priv->dev, "Invalid Port number\n");
		}
		break;
	case SWITCH_SET_PORT_STATE:
		if (config.port == 1 || config.port == 2) {
			ret = cpsw_set_port_state(priv, config.port,
						  config.port_state);
		} else {
			dev_err(priv->dev, "Invalid Port number\n");
		}
		break;
	case SWITCH_GET_PORT_VLAN_CONFIG:
	{
		u32 __iomem *port_vlan_reg;
		u32 port_vlan;

		switch (config.port) {
		case 0:
			port_vlan_reg = &cpsw->host_port_regs->port_vlan;
			port_vlan = readl(port_vlan_reg);
			ret = 0;

			break;
		case 1:
		case 2:
		{
			int slave = config.port - 1;
			int reg = CPSW2_PORT_VLAN;

			if (cpsw->version == CPSW_VERSION_1)
				reg = CPSW1_PORT_VLAN;

			port_vlan = slave_read(cpsw->slaves + slave, reg);
			ret = 0;

			break;
		}
		default:
			dev_err(priv->dev, "Invalid Port number\n");
			break;
		}

		if (!ret) {
			config.vid = port_vlan & 0xfff;
			config.vlan_cfi = port_vlan & BIT(12) ? true : false;
			config.prio = (port_vlan >> 13) & 0x7;
			ret = copy_to_user(ifrq->ifr_data, &config,
					   sizeof(config));
		}
		break;
	}
	case SWITCH_SET_PORT_VLAN_CONFIG:
	{
		void __iomem *port_vlan_reg;
		u32 port_vlan;

		port_vlan = config.vid;
		port_vlan |= config.vlan_cfi ? BIT(12) : 0;
		port_vlan |= (config.prio & 0x7) << 13;

		switch (config.port) {
		case 0:
			port_vlan_reg = &cpsw->host_port_regs->port_vlan;
			writel(port_vlan, port_vlan_reg);
			ret = 0;

			break;
		case 1:
		case 2:
		{
			int slave = config.port - 1;
			int reg = CPSW2_PORT_VLAN;

			if (cpsw->version == CPSW_VERSION_1)
				reg = CPSW1_PORT_VLAN;

			slave_write(cpsw->slaves + slave, port_vlan, reg);
			ret = 0;

			break;
		}
		default:
			dev_err(priv->dev, "Invalid Port number\n");
			break;
		}

		break;
	}
	case SWITCH_RATELIMIT:
	{
		if (config.port > 2) {
			dev_err(priv->dev, "Invalid Port number\n");
			break;
		}

		ret = cpsw_ale_rx_ratelimit_mc(cpsw->ale, config.port, config.mcast_rate_limit);
		if (ret)
			dev_err(priv->dev, "CPSW_ALE set MC ratelimit failed");

		ret = cpsw_ale_rx_ratelimit_bc(cpsw->ale, config.port, config.bcast_rate_limit);
		if (ret)
			dev_err(priv->dev, "CPSW_ALE set BC ratelimit failed");

		break;
	}

	default:
		ret = -EOPNOTSUPP;
	}

	return ret;
}
