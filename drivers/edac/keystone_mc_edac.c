/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Keystone DDR3 MC ECC error detection driver
 *
 * TODO: Need to investgiate how to hook this to the edac core driver.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/module.h>

/* DDR3 controller registers */
#define DDR3_EOI			0x0A0
#define DDR3_IRQ_STATUS_RAW_SYS		0x0A4
#define DDR3_IRQ_STATUS_SYS		0x0AC
#define DDR3_IRQ_ENABLE_SET_SYS		0x0B4
#define DDR3_IRQ_ENABLE_CLR_SYS		0x0BC
#define DDR3_ECC_CTRL			0x110
#define DDR3_ONE_BIT_ECC_ERR_CNT	0x130
#define TWO_BIT_ECC_ERR_ADDR_LOG	0x140

#define DDR3_1B_ECC_ERR			BIT(5)
#define DDR3_2B_ECC_ERR			BIT(4)
#define DDR3_WR_ECC_ERR			BIT(3)
#define DDR3_SYS_ERR			BIT(0)
/* Bit 31 enables ECC and 28 enables RMW */
#define ECC_ENABLED			(BIT(31) | BIT(28))

static void ks2_mc_ddr3_ecc_check(void __iomem *reg)
{
	u32 irq_status;

	irq_status = readl(reg + DDR3_IRQ_STATUS_SYS);
	if ((irq_status & DDR3_2B_ECC_ERR) ||
	    (irq_status & DDR3_WR_ECC_ERR)) {
		/*
		 * Do a kernel panic as this is double bit ECC error
		 * or ECC write that are fatal
		 */
		if (irq_status & DDR3_2B_ECC_ERR)
			panic("UC DDR3 ECC err, irq stats 0x%x, addr 0x%x..\n",
			      irq_status,
			      readl(reg + TWO_BIT_ECC_ERR_ADDR_LOG));
		else
			panic("UC DDR3 ECC err, irq stats 0x%x..\n",
			      irq_status);
	}
}

static irqreturn_t ks2_mc_ddr3_ecc_isr(int irq, void *reg_virt)
{
	void __iomem *reg = (void __iomem *)reg_virt;

	ks2_mc_ddr3_ecc_check(reg);

	/*
	 * Other errors should be handled by hardware
	 * So, nothing to do here. For now it never reaches here
	 * as panic will be triggerred for ECC errors
	 */
	return IRQ_HANDLED;
}

static const struct of_device_id ks2_mc_ddr3_ecc_of_match[] = {
	{.compatible = "ti,keystone-ddr3-mc-edac", },
	{},
};

static int ks2_mc_ddr3_ecc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int error_irq = 0, ret = -ENODEV;
	struct device *dev = &pdev->dev;
	struct resource res;
	void __iomem *reg;
	u32 val;

	if (of_address_to_resource(np, 0, &res) < 0) {
		dev_err(dev, "no edac resource address\n");
		return ret;
	}

	reg = devm_ioremap_resource(dev, &res);
	if (IS_ERR(reg)) {
		dev_err(dev,
			"DDR3 controller regs not defined\n");
		return PTR_ERR(reg);
	}

	/* Check if ECC is enabled. If not, just return */
	val = readl(reg + DDR3_ECC_CTRL);
	if (!(val & ECC_ENABLED)) {
		dev_info(&pdev->dev, "ECC is not enabled, disable edac\n");
		return ret;
	}

	/* disable and clear unused ECC interrupts */
	writel(DDR3_1B_ECC_ERR | DDR3_SYS_ERR, reg + DDR3_IRQ_ENABLE_CLR_SYS);
	writel(DDR3_1B_ECC_ERR | DDR3_SYS_ERR, reg + DDR3_IRQ_STATUS_SYS);

	/* check if we already have unrecoverable errors */
	ks2_mc_ddr3_ecc_check(reg);

	writel(DDR3_2B_ECC_ERR | DDR3_WR_ECC_ERR,
	       reg + DDR3_IRQ_ENABLE_CLR_SYS);

	/* add DDR3 ECC error handler */
	error_irq = platform_get_irq(pdev, 0);
	if (!error_irq) {
		dev_err(&pdev->dev,
			"DDR3 ECC irq number not defined\n");
		return ret;
	}

	ret = devm_request_irq(dev, error_irq, ks2_mc_ddr3_ecc_isr, 0,
			       "ddr3-ecc-err-irq", (void *)reg);
	if (ret) {
		dev_err(&pdev->dev,
			"request_irq fail for DDR3 ECC error irq\n");
		return ret;
	}

	writel(DDR3_2B_ECC_ERR | DDR3_WR_ECC_ERR,
	       reg + DDR3_IRQ_ENABLE_SET_SYS);

	return ret;
}

static struct platform_driver ks2_mc_ddr3_ecc_driver = {
	.probe = ks2_mc_ddr3_ecc_probe,
	.driver = {
		   .name = "ks2_mc_ddr3_ecc",
		   .of_match_table = ks2_mc_ddr3_ecc_of_match,
	},
};

static int __init ks2_mc_ddr3_ecc_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&ks2_mc_ddr3_ecc_driver);
	if (ret)
		pr_warn("keystone DDR3 DDR3 ecc_init failed\n");

	return ret;
}
subsys_initcall(ks2_mc_ddr3_ecc_init);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("EDAC Driver for Keystone DDR3 MC");
MODULE_LICENSE("GPL v2");
