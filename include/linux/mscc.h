/*
 * Driver header file for Microsemi VSC85xx PHYs
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

#ifndef __LINUX_MSCC_H
#define __LINUX_MSCC_H

enum phy_features {
	PHY_EDGE_RATE_CONTROL = 0,
	PHY_MAC_IF            = 1,
	PHY_READ_REG          = 2,
	PHY_WRITE_REG         = 3,
	PHY_SUPPORTED_FEATURES_MAX
};

enum rgmii_rx_clock_delay {
	RGMII_RX_CLK_DELAY_0_2_NS = 0,
	RGMII_RX_CLK_DELAY_0_8_NS = 1,
	RGMII_RX_CLK_DELAY_1_1_NS = 2,
	RGMII_RX_CLK_DELAY_1_7_NS = 3,
	RGMII_RX_CLK_DELAY_2_0_NS = 4,
	RGMII_RX_CLK_DELAY_2_3_NS = 5,
	RGMII_RX_CLK_DELAY_2_6_NS = 6,
	RGMII_RX_CLK_DELAY_3_4_NS = 7
};

struct phy_features_t {
	enum phy_features cmd;           /* PHY Supported Features */
	bool op;                         /* Enable/Disable operation */
	u8   rate;                       /* Edge rate control */
	u8   mdix;                       /* MDIX control */
	u8   status;                     /* Status */
	phy_interface_t mac_if;          /* MAC interface config */
	struct ethtool_phy_reg *data;    /* Read/Write register operatioins */
};

#endif /*  __LINUX_MSCC_H */
