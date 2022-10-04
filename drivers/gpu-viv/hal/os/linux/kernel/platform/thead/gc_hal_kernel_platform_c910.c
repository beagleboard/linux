/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2021 THead Corporation
*
****************************************************************************/

#include <linux/clk.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include "gc_hal_kernel_linux.h"
#include "gc_hal_kernel_platform.h"
#include "gc_hal_kernel_device.h"
#include "gc_hal_driver.h"

struct c910_priv {
	struct clk *pclk;
	struct clk *aclk;
	struct clk *cclk;
};

static gcsPLATFORM c910_platform;

static const struct of_device_id c910_gc620_dt_ids[] = {
	{ .compatible = "thead,c910-gc620", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, c910_gc620_dt_ids);

static gceSTATUS c910_adjustParam(IN gcsPLATFORM * Platform,
				  OUT gcsMODULE_PARAMETERS *Args)
{
	int irq;
	struct resource *res;
	struct platform_device *pdev = Platform->device;

	irq = platform_get_irq_byname(pdev, "irq_2d");
	if (irq < 0)
		return gcvSTATUS_NOT_FOUND;

	Args->irqs[gcvCORE_2D] = irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return gcvSTATUS_NOT_FOUND;

	Args->registerBases[gcvCORE_2D] = res->start;
	Args->registerSizes[gcvCORE_2D] = resource_size(res);

	return gcvSTATUS_OK;
}

static gceSTATUS c910_getPower(IN gcsPLATFORM * Platform)
{
	struct c910_priv *c910 = Platform->priv;
	struct platform_device *pdev = c910_platform.device;

	c910->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(c910->pclk))
		return gcvSTATUS_NOT_FOUND;

	c910->aclk = devm_clk_get(&pdev->dev, "aclk");
	if (IS_ERR(c910->aclk))
		return gcvSTATUS_NOT_FOUND;

	c910->cclk = devm_clk_get(&pdev->dev, "cclk");
	if (IS_ERR(c910->cclk))
		return gcvSTATUS_NOT_FOUND;

	return gcvSTATUS_OK;
}

static gceSTATUS c910_setClock(IN gcsPLATFORM * Platform,
			       IN gceCORE GPU,
			       IN gctBOOL Enable)
{
	struct c910_priv *c910 = Platform->priv;

	if (Enable == gcvTRUE) {
		clk_prepare_enable(c910->pclk);
		clk_prepare_enable(c910->aclk);
		clk_prepare_enable(c910->cclk);
	} else {
		clk_disable_unprepare(c910->cclk);
		clk_disable_unprepare(c910->aclk);
		clk_disable_unprepare(c910->pclk);
	}

	return gcvSTATUS_OK;
}

static gcsPLATFORM_OPERATIONS c910_platform_ops = {
	.adjustParam = c910_adjustParam,
	.getPower = c910_getPower,
	.setClock = c910_setClock,
};

static gcsPLATFORM c910_platform = {
	.name = __FILE__,
	.ops  = &c910_platform_ops,
};

static int extra_platform_driver(struct platform_driver *pdrv)
{
	pdrv->driver.of_match_table = c910_gc620_dt_ids;

	/* TODO: add pm and runtime_pm support */

	return 0;
}

int gckPLATFORM_Init(struct platform_driver *pdrv, gcsPLATFORM **platform)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct c910_priv *c910;

	np = of_find_compatible_node(NULL, NULL, "thead,c910-gc620");
	if (!np)
		return -ENODEV;
	of_node_put(np);

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		pr_err("failed to get device for node %s\n", np->name);
		return -ENODEV;
	}

	c910 = kzalloc(sizeof(*c910), GFP_KERNEL);
	if (!c910)
		return -ENOMEM;

	c910_platform.device = pdev;
	c910_platform.priv = c910;

	extra_platform_driver(pdrv);

	*platform = &c910_platform;

	return 0;
}

int gckPLATFORM_Terminate(gcsPLATFORM *platform)
{
	kfree(platform->priv);
	return 0;
}
