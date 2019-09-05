// SPDX-License-Identifier: GPL-2.0
/*
 * TI j721e Cadence MHDP DP wrapper
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Jyri Sarha <jsarha@ti.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/io.h>

#include "cdns-mhdp-j721e.h"

#define	REVISION			0x00
#define	DPTX_IPCFG			0x04
#define	ECC_MEM_CFG			0x08
#define	DPTX_DSC_CFG			0x0c
#define	DPTX_SRC_CFG			0x10
#define	DPTX_VIF_SECURE_MODE_CFG	0x14
#define	DPTX_VIF_CONN_STATUS		0x18
#define	PHY_CLK_STATUS			0x1c

#define DPTX_SRC_AIF_EN			BIT(16)
#define DPTX_SRC_VIF_3_IN30B		BIT(11)
#define DPTX_SRC_VIF_2_IN30B		BIT(10)
#define DPTX_SRC_VIF_1_IN30B		BIT(9)
#define DPTX_SRC_VIF_0_IN30B		BIT(8)
#define DPTX_SRC_VIF_3_SEL_DPI5		BIT(7)
#define DPTX_SRC_VIF_3_SEL_DPI3		0
#define DPTX_SRC_VIF_2_SEL_DPI4		BIT(6)
#define DPTX_SRC_VIF_2_SEL_DPI2		0
#define DPTX_SRC_VIF_1_SEL_DPI3		BIT(5)
#define DPTX_SRC_VIF_1_SEL_DPI1		0
#define DPTX_SRC_VIF_0_SEL_DPI2		BIT(4)
#define DPTX_SRC_VIF_0_SEL_DPI0		0
#define DPTX_SRC_VIF_3_EN		BIT(3)
#define DPTX_SRC_VIF_2_EN		BIT(2)
#define DPTX_SRC_VIF_1_EN		BIT(1)
#define DPTX_SRC_VIF_0_EN		BIT(0)

/* TODO turn DPTX_IPCFG fw_mem_clk_en at pm_runtime_suspend. */

int cdns_mhdp_j721e_init(struct cdns_mhdp_device *mhdp)
{
	struct platform_device *pdev = to_platform_device(mhdp->dev);
	struct resource *regs;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	mhdp->j721e_regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(mhdp->j721e_regs))
		return PTR_ERR(mhdp->j721e_regs);

	return 0;
}

void cdns_mhdp_j721e_fini(struct cdns_mhdp_device *mhdp)
{
}

void cdns_mhdp_j721e_enable(struct cdns_mhdp_device *mhdp)
{
	/*
	 * Eneble VIF_0 and select DPI2 as its input. DSS0 DPI0 is connected
	 * to eDP DPI2. This is the only supported SST configuration on
	 * J721E.
	 */
	writel(DPTX_SRC_VIF_0_EN | DPTX_SRC_VIF_0_SEL_DPI2,
	       mhdp->j721e_regs + DPTX_SRC_CFG);
}

void cdns_mhdp_j721e_disable(struct cdns_mhdp_device *mhdp)
{
	/* HACK: always disable everything and put everything to defaults  */
	writel(0, mhdp->j721e_regs + DPTX_DSC_CFG);
}
