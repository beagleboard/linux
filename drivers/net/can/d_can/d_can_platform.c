/*
 * Platform CAN bus driver for Bosch D_CAN controller
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Borrowed from C_CAN driver
 * Copyright (C) 2010 ST Microelectronics
 * - Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * Borrowed heavily from the C_CAN driver originally written by:
 * Copyright (C) 2007
 * - Sascha Hauer, Marc Kleine-Budde, Pengutronix <s.hauer@pengutronix.de>
 * - Simon Kallweit, intefo AG <simon.kallweit@intefo.ch>
 *
 * Bosch D_CAN controller is compliant to CAN protocol version 2.0 part A and B.
 * Bosch D_CAN user manual can be obtained from:
 * http://www.semiconductors.bosch.de/media/en/pdf/ipmodules_1/can/
 * d_can_users_manual_111.pdf
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * Your platform definitions should specify module ram offsets and interrupt
 * number to use as follows:
 *
 * static struct d_can_platform_data am33xx_evm_d_can_pdata = {
 *	.d_can_offset		= 0,
 *	.d_can_ram_offset	= 0x1000,
 *	.num_of_msg_objs	= 64,
 *	.dma_support		= true,
 *	.parity_check		= false,
 *	.fck_name		= "dcan0_fck",
 *	.ick_name		= "dcan0_ick",
 * };
 *
 * Please see include/linux/can/platform/d_can.h for description of
 * above fields.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/can/platform/d_can.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/can/dev.h>

#include "d_can.h"

static int __devinit d_can_plat_probe(struct platform_device *pdev)
{
	int ret = 0;
	void __iomem *addr;
	struct net_device *ndev;
	struct d_can_priv *priv;
	struct resource *mem;
	struct d_can_platform_data *pdata;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data\n");
		goto exit;
	}

	/* allocate the d_can device */
	ndev = alloc_d_can_dev(pdata->num_of_msg_objs);
	if (!ndev) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "alloc_d_can_dev failed\n");
		goto exit;
	}

	priv = netdev_priv(ndev);

	priv->fck = clk_get(&pdev->dev, pdata->fck_name);
	if (IS_ERR(priv->fck)) {
		dev_err(&pdev->dev, "%s is not found\n", pdata->fck_name);
		ret = -ENODEV;
		goto exit_free_ndev;
	}
	clk_enable(priv->fck);

	priv->ick = clk_get(&pdev->dev, pdata->ick_name);
	if (IS_ERR(priv->ick)) {
		dev_err(&pdev->dev, "%s is not found\n", pdata->ick_name);
		ret = -ENODEV;
		goto exit_free_fck;
	}
	clk_enable(priv->ick);

	/* get the platform data */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "No mem resource\n");
		goto exit_free_clks;
	}

	if (!request_mem_region(mem->start, resource_size(mem),
				D_CAN_DRV_NAME)) {
		dev_err(&pdev->dev, "resource unavailable\n");
		ret = -EBUSY;
		goto exit_free_clks;
	}

	addr = ioremap(mem->start, resource_size(mem));
	if (!addr) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto exit_release_mem;
	}

	/* IRQ specific to Error and status & can be used for Message Object */
	ndev->irq = platform_get_irq_byname(pdev, "int0");
	if (!ndev->irq) {
		dev_err(&pdev->dev, "No irq0 resource\n");
		goto exit_iounmap;
	}

	/* IRQ specific for Message Object */
	priv->irq_obj = platform_get_irq_byname(pdev, "int1");
	if (!priv->irq_obj) {
		dev_err(&pdev->dev, "No irq1 resource\n");
		goto exit_iounmap;
	}

	priv->base = addr;
	priv->can.clock.freq = clk_get_rate(priv->fck);

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	ret = register_d_can_dev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "registering %s failed (err=%d)\n",
				D_CAN_DRV_NAME, ret);
		goto exit_free_device;
	}

	dev_info(&pdev->dev, "%s device registered (irq=%d, irq_obj=%d)\n",
				D_CAN_DRV_NAME, ndev->irq, priv->irq_obj);

	return 0;

exit_free_device:
	platform_set_drvdata(pdev, NULL);
exit_iounmap:
	iounmap(addr);
exit_release_mem:
	release_mem_region(mem->start, resource_size(mem));
exit_free_clks:
	clk_disable(priv->ick);
	clk_put(priv->ick);
exit_free_fck:
	clk_disable(priv->fck);
	clk_put(priv->fck);
exit_free_ndev:
	free_d_can_dev(ndev);
exit:
	dev_err(&pdev->dev, "probe failed\n");

	return ret;
}

static int __devexit d_can_plat_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct d_can_priv *priv = netdev_priv(ndev);
	struct resource *mem;

	unregister_d_can_dev(ndev);
	platform_set_drvdata(pdev, NULL);

	free_d_can_dev(ndev);
	iounmap(priv->base);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));

	clk_disable(priv->ick);
	clk_disable(priv->fck);
	clk_put(priv->ick);
	clk_put(priv->fck);

	return 0;
}

static struct platform_driver d_can_plat_driver = {
	.driver = {
		.name	= D_CAN_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= d_can_plat_probe,
	.remove = __devexit_p(d_can_plat_remove),
};

static int __init d_can_plat_init(void)
{
	printk(KERN_INFO D_CAN_DRV_DESC "\n");
	return platform_driver_register(&d_can_plat_driver);
}
module_init(d_can_plat_init);

static void __exit d_can_plat_exit(void)
{
	printk(KERN_INFO D_CAN_DRV_DESC " unloaded\n");
	platform_driver_unregister(&d_can_plat_driver);
}
module_exit(d_can_plat_exit);

MODULE_AUTHOR("AnilKumar Ch <anilkumar@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(D_CAN_VERSION);
MODULE_DESCRIPTION(D_CAN_DRV_DESC);
