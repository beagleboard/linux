/*
 * Keystone SerDes Phy driver
 *
 * This is the SerDes Phy driver for Keystone devices. This is
 * required to support PCIe RC functionality based on designware
 * PCIe hardware found on these devices.
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The contents of the array k2_100mhz_pcie_5gbps_serdes is covered by BSD
 * license.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#define reg_dump(addr, mask) \
		pr_debug("reg %p has value %x\n", (addr), \
			(readl((addr)) & (mask)))

/* mask bits point to bits being modified */
#define reg_rmw(addr, value, mask) \
		writel(((readl((addr)) & (~(mask))) | \
			((value) & (mask))), (addr))

#define FINSR(base, offset, msb, lsb, val) \
	reg_rmw((base) + (offset), ((val) << (lsb)), (u32)GENMASK((msb), (lsb)))

struct ks_serdes_config {
	u32 ofs;
	u8 msb;
	u8 lsb;
	u32 val;
};

struct ks_serdes_phy {
	struct device *dev;
	void __iomem *base;
};

/* serdes lane configuration blob */
static struct ks_serdes_config k2_100mhz_pcie_5gbps_serdes[] = {
	/* SerDes Clock and common configuration */
	{0x0000,	15,	 8,	0x08},
	{0x0060,	 7,	 0,	0x5c},
	{0x0060,	15,	 8,	0x1c},
	{0x0060,	23,	16,	0x04},
	{0x0064,	15,	 8,	0xc7},
	{0x0064,	23,	16,	0x43},
	{0x0064,	31,	24,	0x03},
	{0x006c,	 7,	 0,	0x12},
	{0x0068,	23,	16,	0x07},
	{0x0078,	15,	 8,	0xc0},

	/* Lane - A Phy configuration */
	{0x0200,	 7,	 0,	0x00},
	{0x0204,	 7,	 0,	0x80},
	{0x0204,	31,	24,	0x5e},
	{0x0208,	 7,	 0,	0x06},
	{0x0208,	23,	16,	0x01},
	{0x0210,	 7,	 0,	0x23},
	{0x0214,	 7,	 0,	0x60},
	{0x0214,	15,	 8,	0x30},
	{0x0214,	31,	24,	0x2e},
	{0x0218,	31,	24,	0x76},
	{0x022c,	 7,	 0,	0x02},
	{0x022c,	23,	16,	0x20},
	{0x02a0,	23,	16,	0xee},
	{0x02a0,	31,	24,	0xff},
	{0x02a4,	 7,	 0,	0x0f},
	{0x0204,	31,	24,	0x5e},
	{0x0208,	 7,	 0,	0x06},
	{0x0278,	15,	 8,	0x20},
	{0x0280,	 7,	 0,	0x28},
	{0x0280,	23,	16,	0x28},
	{0x0284,	 7,	 0,	0x85},
	{0x0284,	15,	 8,	0x03},
	{0x0284,	23,	16,	0x0f},
	{0x0284,	31,	24,	0x2d},
	{0x0250,	31,	24,	0xd0},
	{0x0284,	 7,	 0,	0x85},
	{0x0294,	31,	24,	0x20},

	/* Lane - B Phy configuration */
	{0x0400,	 7,	 0,	0x00},
	{0x0404,	 7,	 0,	0x80},
	{0x0404,	31,	24,	0x5e},
	{0x0408,	 7,	 0,	0x06},
	{0x0408,	23,	16,	0x01},
	{0x0410,	 7,	 0,	0x23},
	{0x0414,	 7,	 0,	0x60},
	{0x0414,	15,	 8,	0x30},
	{0x0414,	31,	24,	0x2e},
	{0x0418,	31,	24,	0x76},
	{0x042c,	 7,	 0,	0x02},
	{0x042c,	23,	16,	0x20},
	{0x04a0,	23,	16,	0xee},
	{0x04a0,	31,	24,	0xff},
	{0x04a4,	 7,	 0,	0x0f},
	{0x0404,	31,	24,	0x5e},
	{0x0408,	 7,	 0,	0x06},
	{0x0478,	15,	 8,	0x20},
	{0x0480,	 7,	 0,	0x28},
	{0x0480,	23,	16,	0x28},
	{0x0484,	 7,	 0,	0x85},
	{0x0484,	15,	 8,	0x03},
	{0x0484,	23,	16,	0x0f},
	{0x0484,	31,	24,	0x2d},
	{0x0450,	31,	24,	0xd0},
	{0x0494,	31,	24,	0x20},

	/* Lane - C Phy configuration */
	{0x0600,	 7,	 0,	0x00},
	{0x0604,	31,	24,	0x5e},
	{0x0608,	 7,	 0,	0x06},
	{0x0608,	23,	16,	0x01},
	{0x0610,	 7,	 0,	0x23},
	{0x0614,	 7,	 0,	0x60},
	{0x0614,	15,	 8,	0x30},
	{0x0614,	31,	24,	0x2e},
	{0x0618,	31,	24,	0x76},
	{0x062c,	 7,	 0,	0x02},
	{0x062c,	23,	16,	0x20},
	{0x06a0,	23,	16,	0xee},
	{0x06a0,	31,	24,	0xff},
	{0x06a4,	 7,	 0,	0x0f},
	{0x0604,	31,	24,	0x5e},
	{0x0608,	 7,	 0,	0x06},
	{0x0678,	15,	 8,	0x20},
	{0x0680,	 7,	 0,	0x28},
	{0x0680,	23,	16,	0x28},
	{0x0684,	 7,	 0,	0x85},
	{0x0684,	15,	 8,	0x03},
	{0x0684,	23,	16,	0x0f},
	{0x0684,	31,	24,	0x2d},
	{0x0650,	31,	24,	0xd0},
	{0x0694,	31,	24,	0x20},

	/* Lane - D Phy configuration */
	{0x0800,	 7,	 0,	0x00},
	{0x0804,	 7,	 0,	0x80},
	{0x0804,	31,	24,	0x5e},
	{0x0808,	 7,	 0,	0x06},
	{0x0810,	 7,	 0,	0x23},
	{0x0814,	 7,	 0,	0x60},
	{0x0814,	15,	 8,	0x30},
	{0x0814,	31,	24,	0x2e},
	{0x0818,	31,	24,	0x76},
	{0x082c,	 7,	 0,	0x02},
	{0x082c,	23,	16,	0x20},
	{0x08a0,	23,	16,	0xee},
	{0x08a0,	31,	24,	0xff},
	{0x08a4,	 7,	 0,	0x0f},
	{0x0804,	31,	24,	0x5e},
	{0x0808,	 7,	 0,	0x06},
	{0x0808,	23,	16,	0x01},
	{0x0878,	15,	 8,	0x20},
	{0x0880,	 7,	 0,	0x28},
	{0x0880,	23,	16,	0x28},
	{0x0884,	 7,	 0,	0x85},
	{0x0884,	15,	 8,	0x03},
	{0x0884,	23,	16,	0x0f},
	{0x0884,	31,	24,	0x2d},
	{0x0850,	31,	24,	0xd0},
	{0x0894,	31,	24,	0x20},

	/* Common Lane configurations */
	{0x0a00,	15,	 8,	0x01},
	{0x0a08,	 7,	 0,	0x08},
	{0x0a08,	15,	 8,	0x2c},
	{0x0a08,	23,	16,	0xe1},
	{0x0a0c,	 7,	 0,	0x81},
	{0x0a18,	23,	16,	0xe8},
	{0x0a30,	15,	 8,	0x2f},
	{0x0a30,	23,	16,	0x2f},
	{0x0a4c,	23,	16,	0x82},
	{0x0a4c,	31,	24,	0xac},
	{0x0a54,	31,	24,	0xc0},
	{0x0a58,	 7,	 0,	0x41},
	{0x0a58,	15,	 8,	0x14},
	{0x0a84,	 7,	 0,	0x01},
	{0x0a84,	15,	 8,	0x03},
	{0x0a8c,	23,	16,	0x03},
	{0x0a8c,	31,	24,	0x81},
	{0x0a90,	 7,	 0,	0x01},
	{0x0a90,	15,	 8,	0x60},
	{0x0a94,	31,	24,	0x01},
	{0x0aa0,	31,	24,	0x81},
	{0x0aa4,	 7,	 0,	0x35},
	{0x0aa4,	15,	 8,	0xa8},
	{0x0abc,	31,	24,	0xff},
	{0x0ac0,	 7,	 0,	0x8b},
	{0x0a44,	15,	 8,	0x3d},
	{0x0a44,	23,	16,	0x73},
	{0x0a44,	31,	24,	0x5f},
	{0x0a48,	15,	 8,	0xca},
	{0x0a48,	23,	16,	0xfd},
	{0x0a5c,	23,	16,	0x00},
	{0x0a5c,	31,	24,	0x00},
	{0x0a60,	 7,	 0,	0x00},
	{0x0a60,	15,	 8,	0x80},
	{0x0a60,	23,	16,	0x00},
	{0x0a60,	31,	24,	0x00},
	{0x0a64,	 7,	 0,	0x20},
	{0x0a64,	15,	 8,	0x12},
	{0x0a64,	23,	16,	0x58},
	{0x0a64,	31,	24,	0x0c},
	{0x0a68,	 7,	 0,	0x02},
	{0x0a68,	15,	 8,	0x06},
	{0x0a68,	23,	16,	0x3b},
	{0x0a68,	31,	24,	0xe1},
	{0x0a6c,	 7,	 0,	0xc1},
	{0x0a6c,	15,	 8,	0x4c},
	{0x0a6c,	23,	16,	0x07},
	{0x0a6c,	31,	24,	0xb8},
	{0x0a70,	 7,	 0,	0x89},
	{0x0a70,	15,	 8,	0xe9},
	{0x0a70,	23,	16,	0x02},
	{0x0a70,	31,	24,	0x3f},
	{0x0a74,	 7,	 0,	0x01},
	{0x0b14,	23,	16,	0x37},
	{0x0b10,	31,	24,	0x37},
	{0x0b14,	 7,	 0,	0x5d},
	{0x0000,	 7,	 0,	0x03},
	{0x0a00,	 7,	 0,	0x9f},
};

static int ks_serdes_phy_init(struct phy *phy)
{
	struct ks_serdes_config *p = &k2_100mhz_pcie_5gbps_serdes[0];
	struct ks_serdes_phy *ks_phy = phy_get_drvdata(phy);
	int i;

	for (i = 0; i < ARRAY_SIZE(k2_100mhz_pcie_5gbps_serdes); i++) {
		FINSR(ks_phy->base, p[i].ofs, p[i].msb, p[i].lsb, p[i].val);
		reg_dump((ks_phy->base + p[i].ofs),
			 (u32)GENMASK(p[i].msb, p[i].lsb));
	}

	/*
	 * Add 20 msec delay for PLL to lock. This is the value chosen based on
	 * experiments done on K2E SoC and not based on any hardware
	 * documentation. This may be adjusted to get this functional on other
	 * SoCs.
	 */
	msleep(20);

	return 0;
}

static struct phy_ops ks_serdes_phy_ops = {
	.init		= ks_serdes_phy_init,
	.owner		= THIS_MODULE,
};

static int ks_serdes_phy_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct ks_serdes_phy *ks_phy;
	struct phy *phy;
	struct resource *res;

	ks_phy = devm_kzalloc(dev, sizeof(*ks_phy), GFP_KERNEL);
	if (!ks_phy)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "reg_serdes");
	ks_phy->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ks_phy->base))
		return PTR_ERR(ks_phy->base);

	ks_phy->dev = dev;
	phy = devm_phy_create(dev, NULL, &ks_serdes_phy_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	phy_set_drvdata(phy, ks_phy);
	phy_provider = devm_of_phy_provider_register(ks_phy->dev,
						     of_phy_simple_xlate);

	if (IS_ERR(phy_provider))
		return PTR_ERR_OR_ZERO(phy_provider);

	return 0;
}

static const struct of_device_id ks_serdes_phy_of_match[] = {
	{ .compatible = "ti,keystone-serdes-phy" },
	{ },
};
MODULE_DEVICE_TABLE(of, ks_serdes_phy_of_match);

static struct platform_driver ks_serdes_phy_driver = {
	.probe	= ks_serdes_phy_probe,
	.driver = {
		.of_match_table	= ks_serdes_phy_of_match,
		.name  = "ti,keystone-serdes-phy",
		.owner = THIS_MODULE,
	}
};
module_platform_driver(ks_serdes_phy_driver);

MODULE_DESCRIPTION("TI Keystone SerDes PHY driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Murali Karicheri <m-karicheri2@ti.com>");
