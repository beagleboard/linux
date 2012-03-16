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
 *	.num_of_msg_objs	= 64,
 *	.dma_support		= false,
 *	.ram_init		= d_can_hw_raminit,
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
#include <linux/pm_runtime.h>
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
	struct clk *fck;

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
	fck = clk_get(&pdev->dev, "fck");
	if (IS_ERR(fck)) {
		dev_err(&pdev->dev, "fck is not found\n");
		ret = -ENODEV;
		goto exit_free_ndev;
	}

	/* get the platform data */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "No mem resource\n");
		goto exit_clk_put;
	}

	if (!request_mem_region(mem->start, resource_size(mem),
				D_CAN_DRV_NAME)) {
		dev_err(&pdev->dev, "resource unavailable\n");
		ret = -EBUSY;
		goto exit_clk_put;
	}

	addr = ioremap(mem->start, resource_size(mem));
	if (!addr) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto exit_release_mem;
	}

	/* IRQ specific to Error and status & can be used for Message Object */
	ndev->irq = platform_get_irq_byname(pdev, "d_can_ms");
	if (!ndev->irq) {
		dev_err(&pdev->dev, "No irq0 resource\n");
		goto exit_iounmap;
	}

	/* IRQ specific for Message Object */
	priv->irq_obj = platform_get_irq_byname(pdev, "d_can_mo");
	if (!priv->irq_obj) {
		dev_err(&pdev->dev, "No irq1 resource\n");
		goto exit_iounmap;
	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	priv->pdev = pdev;
	priv->base = addr;
	priv->can.clock.freq = clk_get_rate(fck);
	priv->ram_init = pdata->ram_init;
	priv->opened = false;

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	ret = register_d_can_dev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "registering %s failed (err=%d)\n",
				D_CAN_DRV_NAME, ret);
		goto exit_free_device;
	}

	/* Initialize DCAN RAM */
	d_can_reset_ram(priv, pdev->id, 1);

	dev_info(&pdev->dev, "device registered (irq=%d, irq_obj=%d)\n",
						ndev->irq, priv->irq_obj);

	return 0;

exit_free_device:
	platform_set_drvdata(pdev, NULL);
	pm_runtime_disable(&pdev->dev);
exit_iounmap:
	iounmap(addr);
exit_release_mem:
	release_mem_region(mem->start, resource_size(mem));
exit_clk_put:
	clk_put(fck);
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

	/* De-initialize DCAN RAM */
	d_can_reset_ram(priv, pdev->id, 0);

	unregister_d_can_dev(ndev);
	platform_set_drvdata(pdev, NULL);

	free_d_can_dev(ndev);
	iounmap(priv->base);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int d_can_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret;
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct d_can_priv *priv = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netif_stop_queue(ndev);
		netif_device_detach(ndev);
	}

	ret = d_can_power_down(priv);
	if (ret) {
		dev_err(&pdev->dev, "Not entered power down mode\n");
		return ret;
	}

	priv->can.state = CAN_STATE_SLEEPING;

	/* De-initialize DCAN RAM */
	d_can_reset_ram(priv, pdev->id, 0);

	/* Disable the module */
	pm_runtime_put_sync(&pdev->dev);

	return 0;
}

static int d_can_resume(struct platform_device *pdev)
{
	int ret;

	struct net_device *ndev = platform_get_drvdata(pdev);
	struct d_can_priv *priv = netdev_priv(ndev);

	/* Enable the module */
	pm_runtime_get_sync(&pdev->dev);

	/* Initialize DCAN RAM */
	d_can_reset_ram(priv, pdev->id, 1);

	ret = d_can_power_up(priv);
	if (ret) {
		dev_err(&pdev->dev, "Not came out from power down mode\n");
		return ret;
	}

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	if (netif_running(ndev)) {
		netif_device_attach(ndev);
		netif_start_queue(ndev);
	}

	return 0;
}
#else
#define d_can_suspend NULL
#define d_can_resume NULL
#endif

static struct platform_driver d_can_plat_driver = {
	.driver = {
		.name	= D_CAN_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= d_can_plat_probe,
	.remove		= __devexit_p(d_can_plat_remove),
	.suspend	= d_can_suspend,
	.resume		= d_can_resume,
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
