/*
 * PHY Register Header file for Microsemi VSC85xx PHYs
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

#ifndef __MSCC_REG_H
#define __MSCC_REG_H

/* Microsemi VSC85xx PHY registers */
/* IEEE 802. Std Registers */
#define MSCC_PHY_BYPASS_CONTROL        18
#define DISABLE_HP_AUTO_MDIX_MASK      0x0080
#define DISABLE_PAIR_SWAP_CORR_MASK    0x0020
#define DISABLE_POLARITY_CORR_MASK     0x0010

#define MSCC_PHY_EXT_PHY_CNTL_1        23
#define MAC_IF_SELECTION_MASK          0x1800
#define MAC_IF_SELECTION_GMII          0
#define MAC_IF_SELECTION_RMII          1
#define MAC_IF_SELECTION_RGMII         2
#define MAC_IF_SELECTION_POS           11
#define FAR_END_LOOPBACK_MODE_MASK     0x0008

#define MSCC_PHY_EXT_PHY_CNTL_2        24
#define CONNECTOR_LOOPBACK_MASK        0x0001
#define JUMBO_PACKET_MODE_MASK         0x0030
#define JUMBO_PACKET_MODE_POS          4

#define MII_VSC85XX_INT_MASK           25
#define MII_VSC85XX_INT_MASK_MDINT     0x8000
#define MII_VSC85XX_INT_MASK_SPEED     0x4000
#define MII_VSC85XX_INT_MASK_LINK      0x2000
#define MII_VSC85XX_INT_MASK_DUPLEX    0x1000
#define MII_VSC85XX_INT_MASK_MASK      0xa000

#define MII_VSC85XX_INT_STATUS         26
#define MSCC_PHY_DEV_AUX_CNTL          28
#define HP_AUTO_MDIX_X_OVER_IND_MASK   0x2000

#define MSCC_EXT_PAGE_ACCESS           31
#define MSCC_PHY_PAGE_STANDARD         0x0000 /* Standard registers */
#define MSCC_PHY_PAGE_EXTENDED         0x0001 /* Extended registers */
#define MSCC_PHY_PAGE_EXTENDED_2       0x0002 /* Extended registers - page 2 */
#define MSCC_PHY_PAGE_EXTENDED_3       0x0003 /* Extended registers - page 3 */
#define MSCC_PHY_PAGE_EXTENDED_4       0x0004 /* Extended registers - page 4 */
#define MSCC_PHY_PAGE_GPIO             0x0010 /* GPIO registers */

/* Extended Page 1 Registers */
#define MSCC_PHY_EXT_MODE_CNTL         19
#define FORCE_MDI_CROSSOVER_MASK       0x000C
#define FORCE_MDI_CROSSOVER_NORMAL     0
#define FORCE_MDI_CROSSOVER_MDI        2
#define FORCE_MDI_CROSSOVER_MDIX       3
#define FORCE_MDI_CROSSOVER_POS        2

#define MSCC_PHY_ACTIPHY_CNTL          20
#define LINK_SPD_DOWNSHIFT_EN          0x0010
#define LINK_SPD_DOWNSHIFT_CNTL_POS    2
#define LINK_SPD_DOWNSHIFT_CNTL_MASK   0x000C

#define MSCC_PHY_POE_MISC              23
#define INLINE_POE_DETECTION           0x0400
#define INLINE_POE_STATUS_MASK         0x0300
#define INLINE_POE_STATUS_POS          8

/* Extended Page 2 Registers */
#define MSCC_PHY_RGMII_CNTL            20
#define FLF2_ENABLE                    0x8000
#define RGMII_RX_CLK_DELAY_MASK        0x0070
#define RGMII_RX_CLK_DELAY_POS         4

#define MSCC_PHY_WOL_LOWER_MAC_ADDR    21
#define MSCC_PHY_WOL_MID_MAC_ADDR      22
#define MSCC_PHY_WOL_UPPER_MAC_ADDR    23
#define MSCC_PHY_WOL_LOWER_PASSWD      24
#define MSCC_PHY_WOL_MID_PASSWD        25
#define MSCC_PHY_WOL_UPPER_PASSWD      26

#define MSCC_PHY_WOL_MAC_CONTROL       27
#define EDGE_RATE_CNTL_POS             5
#define EDGE_RATE_CNTL_MASK            0x00E0
#define SECURE_ON_ENABLE               0x8000
#define SECURE_ON_PASSWD_LEN_4         0x4000

/* Microsemi PHY ID's */
#define PHY_ID_VSC8531                 0x00070570
#define PHY_ID_VSC8541                 0x00070770

#endif /* __MSCC_REG_H */
