/*
 * Driver for Microsemi VSC85xx PHYs
 *
 * Author: Nagaraju Lakkaraju
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Microsemi Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/mdio.h>
#include <linux/mscc.h>

#include "mscc_reg.h"

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/* RGMII Rx Clock delay value change with board lay-out */
static u8 rgmii_rx_clk_delay = RGMII_RX_CLK_DELAY_1_1_NS;

static int vsc85xx_phy_page_set(struct phy_device *phydev, u8 page)
{
	int rc = 0;

	rc = phy_write(phydev, MSCC_EXT_PAGE_ACCESS, page);
	return rc;
}

static int vsc85xx_default_config(struct phy_device *phydev)
{
	int rc = 0;
	u16 reg_val = 0;

	phydev->supported = (SUPPORTED_1000baseT_Full |
			     SUPPORTED_1000baseT_Half |
			     SUPPORTED_100baseT_Full  |
			     SUPPORTED_100baseT_Half  |
			     SUPPORTED_10baseT_Full   |
			     SUPPORTED_10baseT_Half   |
			     SUPPORTED_Autoneg        |
			     SUPPORTED_Pause          |
			     SUPPORTED_Asym_Pause     |
			     SUPPORTED_TP);

	phydev->speed = SPEED_1000;
	phydev->duplex = DUPLEX_FULL;
	phydev->pause = 0;
	phydev->asym_pause = 0;
	phydev->interface = PHY_INTERFACE_MODE_RGMII;
	phydev->mdix = ETH_TP_MDI_AUTO;

	mutex_lock(&phydev->lock);
	rc = vsc85xx_phy_page_set(phydev, MSCC_PHY_PAGE_EXTENDED_2);
	if (rc != 0) {
		rc = -EINVAL;
		goto out_unlock;
	}
	reg_val = phy_read(phydev, MSCC_PHY_RGMII_CNTL);
	reg_val &= ~(RGMII_RX_CLK_DELAY_MASK);
	reg_val |= (rgmii_rx_clk_delay << RGMII_RX_CLK_DELAY_POS);
	phy_write(phydev, MSCC_PHY_RGMII_CNTL, reg_val);
	rc = vsc85xx_phy_page_set(phydev, MSCC_PHY_PAGE_STANDARD);
	if (rc != 0)
		rc = -EINVAL;

out_unlock:
	mutex_unlock(&phydev->lock);

	return rc;
}

static int vsc85xx_soft_reset(struct phy_device *phydev)
{
	int rc = 0;
	u16 reg_val = 0;

	reg_val = phy_read(phydev, MII_BMCR);
	reg_val |= BMCR_RESET;
	phy_write(phydev, MII_BMCR, reg_val);

	return rc;
}

static int vsc85xx_edge_rate_cntl_set(struct phy_device *phydev,
				      u8     *rate)
{
	int rc = 0;
	u16 reg_val = 0;
	u8  edge_rate = *rate;

	mutex_lock(&phydev->lock);
	rc = vsc85xx_phy_page_set(phydev, MSCC_PHY_PAGE_EXTENDED_2);
	if (rc != 0) {
		rc = -EINVAL;
		goto out_unlock;
	}
	reg_val = phy_read(phydev, MSCC_PHY_WOL_MAC_CONTROL);
	reg_val &= ~(EDGE_RATE_CNTL_MASK);
	reg_val |= (edge_rate << EDGE_RATE_CNTL_POS);
	phy_write(phydev, MSCC_PHY_WOL_MAC_CONTROL, reg_val);
	rc = vsc85xx_phy_page_set(phydev, MSCC_PHY_PAGE_STANDARD);
	if (rc != 0)
		rc = -EINVAL;

out_unlock:
	mutex_unlock(&phydev->lock);

	return rc;
}

static int vsc85xx_edge_rate_cntl_get(struct phy_device *phydev,
				      u8     *rate)
{
	int rc = 0;
	u16 reg_val = 0;

	mutex_lock(&phydev->lock);
	rc = vsc85xx_phy_page_set(phydev, MSCC_PHY_PAGE_EXTENDED_2);
	if (rc != 0) {
		rc = -EINVAL;
		goto out_unlock;
	}
	reg_val = phy_read(phydev, MSCC_PHY_WOL_MAC_CONTROL);
	reg_val &= EDGE_RATE_CNTL_MASK;
	*rate = reg_val >> EDGE_RATE_CNTL_POS;
	rc = vsc85xx_phy_page_set(phydev, MSCC_PHY_PAGE_STANDARD);
	if (rc != 0)
		rc = -EINVAL;

out_unlock:
	mutex_unlock(&phydev->lock);

	return rc;
}

static int vsc85xx_mac_if_set(struct phy_device *phydev,
			      phy_interface_t   *interface)
{
	int rc = 0;
	u16 reg_val = 0;
	phy_interface_t mac_if = *interface;

	mutex_lock(&phydev->lock);

	reg_val = phy_read(phydev, MSCC_PHY_EXT_PHY_CNTL_1);
	switch (mac_if) {
	case PHY_INTERFACE_MODE_RGMII:
		reg_val &= ~(MAC_IF_SELECTION_MASK);
		reg_val |= (MAC_IF_SELECTION_RGMII << MAC_IF_SELECTION_POS);
		break;
	case PHY_INTERFACE_MODE_RMII:
		reg_val &= ~(MAC_IF_SELECTION_MASK);
		reg_val |= (MAC_IF_SELECTION_RMII << MAC_IF_SELECTION_POS);
		break;
	case PHY_INTERFACE_MODE_MII:
	case PHY_INTERFACE_MODE_GMII:
	default:
		reg_val &= ~(MAC_IF_SELECTION_MASK);
		reg_val |= (MAC_IF_SELECTION_GMII << MAC_IF_SELECTION_POS);
		break;
	}
	phy_write(phydev, MSCC_PHY_EXT_PHY_CNTL_1, reg_val);
	rc = vsc85xx_soft_reset(phydev);
	phydev->interface = mac_if;
	reg_val = phy_read(phydev, MSCC_PHY_EXT_PHY_CNTL_1);

	mutex_unlock(&phydev->lock);

	return rc;
}

static int vsc85xx_mac_if_get(struct phy_device *phydev,
			      phy_interface_t   *interface)
{
	int rc = 0;
	u16 reg_val = 0;

	mutex_lock(&phydev->lock);

	reg_val = phy_read(phydev, MSCC_PHY_EXT_PHY_CNTL_1);
	reg_val = ((reg_val & MAC_IF_SELECTION_MASK) >> MAC_IF_SELECTION_POS);
	if (reg_val == MAC_IF_SELECTION_RGMII)
		*interface = PHY_INTERFACE_MODE_RGMII;
	else if (reg_val == MAC_IF_SELECTION_RMII)
		*interface = PHY_INTERFACE_MODE_RMII;
	else
		*interface = PHY_INTERFACE_MODE_GMII;
	phydev->interface = *interface;

	mutex_unlock(&phydev->lock);

	return rc;
}

static int vsc85xx_phy_read_reg(struct phy_device *phydev,
				struct ethtool_phy_reg *data)
{
	int rc = 0;
	u16 reg_addr = data->reg;
	u8  page = data->pg;
	u16 reg_val;

	data->val = 0;
	reg_val = 0;
	mutex_lock(&phydev->lock);
	if (page != 0) {
		rc = vsc85xx_phy_page_set(phydev, page);
		reg_val = phy_read(phydev, reg_addr);
		rc = vsc85xx_phy_page_set(phydev, 0);
	} else {
		reg_val = phy_read(phydev, reg_addr);
	}
	data->val = reg_val;

	mutex_unlock(&phydev->lock);

	return rc;
}

static int vsc85xx_phy_write_reg(struct phy_device *phydev,
				 struct ethtool_phy_reg *data)
{
	int rc = 0;
	u16 reg_addr = data->reg;
	u16 reg_val = data->val;
	u8  page = data->pg;

	mutex_lock(&phydev->lock);
	if (page != 0) {
		rc = vsc85xx_phy_page_set(phydev, page);
		reg_val = phy_write(phydev, reg_addr, reg_val);
		rc = vsc85xx_phy_page_set(phydev, 0);
	} else {
		reg_val = phy_write(phydev, reg_addr, reg_val);
	}
	mutex_unlock(&phydev->lock);

	return rc;
}

static int vsc85xx_features_set(struct phy_device *phydev)
{
	int rc = 0;
	struct phy_features_t *features = (struct phy_features_t *)phydev->priv;
	u8 command = features->cmd;
	u8 rate = features->rate;
	phy_interface_t mac_if = features->mac_if;
	struct ethtool_phy_reg *wr = features->data;

	switch (command) {
	case PHY_EDGE_RATE_CONTROL:
		rc = vsc85xx_edge_rate_cntl_set(phydev, &rate);
		break;
	case PHY_MAC_IF:
		rc = vsc85xx_mac_if_set(phydev, &mac_if);
		break;
	case PHY_WRITE_REG:
		rc = vsc85xx_phy_write_reg(phydev, wr);
		break;
	default:
		break;
	}

	return rc;
}

static int vsc85xx_features_get(struct phy_device *phydev)
{
	int rc = 0;
	struct phy_features_t *features = (struct phy_features_t *)phydev->priv;
	u8 command = features->cmd;
	u8 rate = 0;
	phy_interface_t mac_if = features->mac_if;
	struct ethtool_phy_reg *rd = features->data;

	switch (command) {
	case PHY_EDGE_RATE_CONTROL:
		rc = vsc85xx_edge_rate_cntl_get(phydev, &rate);
		features->rate = rate;
		break;
	case PHY_MAC_IF:
		rc = vsc85xx_mac_if_get(phydev, &mac_if);
		break;
	case PHY_READ_REG:
		rc = vsc85xx_phy_read_reg(phydev, rd);
		break;
	default:
		break;
	}

	return rc;
}

static int vsc85xx_config_init(struct phy_device *phydev)
{
	int rc = 0;

	rc = vsc85xx_default_config(phydev);
	rc = genphy_config_init(phydev);

	return rc;
}

static int vsc85xx_config_aneg(struct phy_device *phydev)
{
	int rc = 0;

	rc = genphy_config_aneg(phydev);

	return rc;
}

static int vsc85xx_ack_interrupt(struct phy_device *phydev)
{
	int rc = 0;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		rc = phy_read(phydev, MII_VSC85XX_INT_STATUS);

	return (rc < 0) ? rc : 0;
}

static int vsc85xx_config_intr(struct phy_device *phydev)
{
	int rc;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		rc = phy_write(phydev, MII_VSC85XX_INT_MASK,
			       MII_VSC85XX_INT_MASK_MASK);
	else {
		rc = phy_read(phydev, MII_VSC85XX_INT_STATUS);
		if (rc < 0)
			return rc;
		rc = phy_write(phydev, MII_VSC85XX_INT_MASK, 0);
	}

	return rc;
}

/* Microsemi VSC85xx PHYs */
static struct phy_driver vsc85xx_driver[] = {
{
	.phy_id         = PHY_ID_VSC8531,
	.name           = "Microsemi VSC8531",
	.phy_id_mask    = 0xfffffff0,
	.features       = PHY_GBIT_FEATURES,
	.flags          = PHY_HAS_INTERRUPT,
	.soft_reset     = &genphy_soft_reset,
	.config_init    = &vsc85xx_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.aneg_done      = &genphy_aneg_done,
	.read_status    = &genphy_read_status,
	.ack_interrupt  = &vsc85xx_ack_interrupt,
	.config_intr    = &vsc85xx_config_intr,
	.suspend        = &genphy_suspend,
	.resume         = &genphy_resume,
	.phy_features_set = &vsc85xx_features_set,
	.phy_features_get = &vsc85xx_features_get,
},
{
	.phy_id         = PHY_ID_VSC8541,
	.name           = "Microsemi VSC8541 SyncE",
	.phy_id_mask    = 0xfffffff0,
	.features       = PHY_GBIT_FEATURES,
	.flags          = PHY_HAS_INTERRUPT,
	.soft_reset     = &genphy_soft_reset,
	.config_init    = &vsc85xx_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.aneg_done      = &genphy_aneg_done,
	.read_status    = &genphy_read_status,
	.ack_interrupt  = &vsc85xx_ack_interrupt,
	.config_intr    = &vsc85xx_config_intr,
	.suspend        = &genphy_suspend,
	.resume         = &genphy_resume,
	.phy_features_set = &vsc85xx_features_set,
	.phy_features_get = &vsc85xx_features_get,
}

};

module_phy_driver(vsc85xx_driver);

static struct mdio_device_id __maybe_unused vsc85xx_tbl[] = {
	{ PHY_ID_VSC8531, 0xfffffff0, },
	{ PHY_ID_VSC8541, 0xfffffff0, },
	{ }
};

MODULE_DEVICE_TABLE(mdio, vsc85xx_tbl);

MODULE_DESCRIPTION("Microsemi VSC85xx PHY driver");
MODULE_AUTHOR("Nagaraju Lakkaraju");
MODULE_LICENSE("Dual MIT/GPL");
