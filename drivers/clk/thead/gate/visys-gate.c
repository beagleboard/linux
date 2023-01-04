/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2022 Alibaba Group Holding Limited.
 */

#include <dt-bindings/clock/light-fm-ap-clock.h>
#include <dt-bindings/clock/light-visys.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include "clk-gate.h"
#include "../clk.h"

static struct clk *gates[LIGHT_CLKGEN_VISYS_CLK_END];
static struct clk_onecell_data clk_gate_data;

static u32 share_cnt_isp0_hclk_en;
static u32 share_cnt_isp0_aclk_en;

static int light_visys_clk_probe(struct platform_device *pdev)
{
	struct regmap *visys_regmap;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;

	visys_regmap = syscon_regmap_lookup_by_phandle(np, "visys-regmap");
	if (IS_ERR(visys_regmap)) {
		dev_err(&pdev->dev, "cannot find regmap for vi system register\n");
		return PTR_ERR(visys_regmap);
	}

	/* we assume that the gate clock is a root clock  */
	gates[LIGHT_CLKGEN_DW200_ACLK] = thead_gate_clk_register("clkgen_dw200_aclk", NULL,
									visys_regmap, 0xa0, 27, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_AXI4_VISYS1_ACLK] = thead_gate_clk_register("clkgen_axi4_visys1_aclk", NULL,
									visys_regmap, 0xa0, 26, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_AXI4_VISYS2_ACLK] = thead_gate_clk_register("clkgen_axi4_visys2_aclk", NULL,
									visys_regmap, 0xa0, 25, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_AXI4_VISYS3_ACLK] = thead_gate_clk_register("clkgen_axi4_visys3_aclk", NULL,
									visys_regmap, 0xa0, 24, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_ISP_RY_ACLK] = thead_gate_clk_register("clkgen_isp_ry_aclk", NULL,
									visys_regmap, 0xa0, 22, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_ISP_VENC_SHAKE_ACLK] = thead_gate_clk_register("clkgen_isp_venc_shake_aclk", NULL,
									visys_regmap, 0xa0, 30, GATE_NOT_SHARED, NULL, dev);

	gates[LIGHT_CLKGEN_VIPRE_ACLK] = thead_gate_clk_register("clkgen_vipre_aclk", NULL,
									visys_regmap, 0xa0, 31, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_DW200_HCLK] = thead_gate_clk_register("clkgen_dw200_hclk", NULL,
									visys_regmap, 0xa0, 13, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_ISP_RY_HCLK] = thead_gate_clk_register("clkgen_isp_ry_hclk", NULL,
									visys_regmap, 0xa0, 12, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_MIPI_CSI0_PCLK] = thead_gate_clk_register("clkgen_mipi_csi0_pclk", NULL,
									visys_regmap, 0xa0, 18, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_MIPI_CSI1_PCLK] = thead_gate_clk_register("clkgen_mipi_csi1_pclk", NULL,
									visys_regmap, 0xa0, 17, GATE_NOT_SHARED, NULL, dev);

	gates[LIGHT_CLKGEN_MIPI_CSI2_PCLK] = thead_gate_clk_register("clkgen_mipi_csi2_pclk", NULL,
									visys_regmap, 0xa0, 16, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_VIPRE_PCLK] = thead_gate_clk_register("clkgen_vipre_pclk", NULL,
									visys_regmap, 0xa0, 15, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_ISP_VENC_SHAKE_PCLK] = thead_gate_clk_register("clkgen_isp_venc_shake_pclk", NULL,
									visys_regmap, 0xa0, 29, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_MIPI_CSI0_PIXCLK] = thead_gate_clk_register("clkgen_mipi_csi0_pixclk", NULL,
									visys_regmap, 0xa0, 11, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_MIPI_CSI1_PIXCLK] = thead_gate_clk_register("clkgen_mipi_csi1_pixclk", NULL,
									visys_regmap, 0xa0, 10, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_MIPI_CSI2_PIXCLK] = thead_gate_clk_register("clkgen_mipi_csi2_pixclk", NULL,
									visys_regmap, 0xa0, 9, GATE_NOT_SHARED, NULL, dev);

	gates[LIGHT_CLKGEN_VIPRE_PIXELCLK] = thead_gate_clk_register("clkgen_vipre_pixelclk", NULL,
									visys_regmap, 0xa4, 23, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_MIPI_CSI0_CFG_CLK] = thead_gate_clk_register("clkgen_mipi_csi0_cfg_clk", NULL,
									visys_regmap, 0xa0, 8, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_MIPI_CSI1_CFG_CLK] = thead_gate_clk_register("clkgen_mipi_csi1_cfg_clk", NULL,
									visys_regmap, 0xa0, 6, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_MIPI_CSI2_CFG_CLK] = thead_gate_clk_register("clkgen_mipi_csi2_cfg_clk", NULL,
									visys_regmap, 0xa0, 7, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_DW200_CLK_VSE] = thead_gate_clk_register("clkgen_dw200_clk_vse", NULL,
									visys_regmap, 0xa0, 5, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_DW200_CLK_DWE] = thead_gate_clk_register("clkgen_dw200_clk_dwe", NULL,
									visys_regmap, 0xa0, 4, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_ISP0_CLK] = thead_gate_clk_register("clkgen_isp_clk_0", NULL,
									visys_regmap, 0xa4, 31, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_ISP1_CLK] = thead_gate_clk_register("clkgen_isp_clk_1", NULL,
									visys_regmap, 0xa4, 30, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_ISP_RY_CCLK] = thead_gate_clk_register("clkgen_isp_ry_cclk", NULL,
									visys_regmap, 0xa0, 21, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_ISP1_PIXELCLK] = thead_gate_clk_register("clkgen_isp1_pixelclk", NULL,
									visys_regmap, 0xa4, 28, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_ISP0_PIXELCLK] = thead_gate_clk_register("clkgen_isp0_pixelclk", NULL,
									visys_regmap, 0xa4, 29, GATE_NOT_SHARED, NULL, dev);
	gates[LIGHT_CLKGEN_ISP1_HCLK] = thead_gate_clk_register("clkgen_isp1_hclk", NULL,
									visys_regmap, 0xa0, 1, GATE_SHARED, &share_cnt_isp0_hclk_en, dev);
	gates[LIGHT_CLKGEN_ISP0_HCLK] = thead_gate_clk_register("clkgen_isp0_hclk", NULL,
									visys_regmap, 0xa0, 1, GATE_SHARED, &share_cnt_isp0_hclk_en, dev);
	gates[LIGHT_CLKGEN_ISP1_ACLK] = thead_gate_clk_register("clkgen_isp1_aclk", NULL,
									visys_regmap, 0xa0, 3, GATE_SHARED, &share_cnt_isp0_aclk_en, dev);
	gates[LIGHT_CLKGEN_ISP0_ACLK] = thead_gate_clk_register("clkgen_isp0_aclk", NULL,
									visys_regmap, 0xa0, 3, GATE_SHARED, &share_cnt_isp0_aclk_en, dev);

	clk_gate_data.clks = gates;
	clk_gate_data.clk_num = ARRAY_SIZE(gates);

	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &clk_gate_data);
	if (ret < 0) {
		dev_err(dev, "failed to register gate clks for light visys\n");
		goto unregister_clks;
	}

	dev_info(dev, "succeed to register visys gate clock provider\n");

	return 0;

unregister_clks:
	thead_unregister_clocks(gates, ARRAY_SIZE(gates));
	return ret;
}

static const struct of_device_id visys_clk_gate_of_match[] = {
	{ .compatible = "thead,visys-gate-controller" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, visys_clk_gate_of_match);

static struct platform_driver light_visys_clk_driver = {
	.probe = light_visys_clk_probe,
	.driver = {
		.name = "visys-clk-gate-provider",
		.of_match_table = of_match_ptr(visys_clk_gate_of_match),
	},
};

module_platform_driver(light_visys_clk_driver);
MODULE_AUTHOR("wei.liu <lw312886@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead Light Fullmask visys clock gate provider");
MODULE_LICENSE("GPL v2");
