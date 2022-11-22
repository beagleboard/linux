/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2022 Alibaba Group Holding Limited.
 */

#include <dt-bindings/clock/light-dspsys.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include "clk-gate.h"
#include "../clk.h"

static struct clk *gates[LIGHT_CLKGEN_DSPSYS_CLK_END];
static struct clk_onecell_data clk_gate_data;

static int light_dspsys_clk_probe(struct platform_device *pdev)
{
	struct regmap *dspsys_regmap, *tee_dspsys_regmap;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;

	dspsys_regmap = syscon_regmap_lookup_by_phandle(np, "dspsys-regmap");
	if (IS_ERR(dspsys_regmap)) {
		dev_err(&pdev->dev, "cannot find regmap for dsp system register\n");
		return PTR_ERR(dspsys_regmap);
	}

	tee_dspsys_regmap = syscon_regmap_lookup_by_phandle(np, "tee-dspsys-regmap");
	if (IS_ERR(tee_dspsys_regmap)) {
		dev_warn(&pdev->dev, "cannot find regmap for tee dsp system register\n");
		tee_dspsys_regmap = NULL;
	}

	gates[CLKGEN_DSP0_PCLK] = thead_gate_clk_register("clkgen_dsp0_pclk", NULL, dspsys_regmap,
							  0x24, 0, GATE_NOT_SHARED, NULL, dev);
	gates[CLKGEN_DSP1_PCLK] = thead_gate_clk_register("clkgen_dsp1_pclk", NULL, dspsys_regmap,
							  0x24, 1, GATE_NOT_SHARED, NULL, dev);
	gates[CLKGEN_DSP1_CCLK] = thead_gate_clk_register("clkgen_dsp1_cclk", NULL, dspsys_regmap,
							  0x24, 2, GATE_NOT_SHARED, NULL, dev);
	gates[CLKGEN_DSP0_CCLK] = thead_gate_clk_register("clkgen_dsp0_cclk", NULL, dspsys_regmap,
							  0x24, 3, GATE_NOT_SHARED, NULL, dev);
	gates[CLKGEN_X2X_DSP2_ACLK_S] = thead_gate_clk_register("clkgen_x2x_dsp2_aclk_s", NULL, dspsys_regmap,
							  0x24, 4, GATE_NOT_SHARED, NULL, dev);
	gates[CLKGEN_X2X_DSP0_ACLK_S] = thead_gate_clk_register("clkgen_x2x_dsp0_aclk_s", NULL, dspsys_regmap,
							  0x24, 5, GATE_NOT_SHARED, NULL, dev);
	gates[CLKGEN_X2X_X4_DSPSLV_DSP1_ACLK_M] = thead_gate_clk_register("clkgen_x2x_x4_dspslv_dsp1_aclk_m",
							  NULL, dspsys_regmap, 0x24, 6, GATE_NOT_SHARED, NULL, dev);
	gates[CLKGEN_X2X_X4_DSPSLV_DSP0_ACLK_M] = thead_gate_clk_register("clkgen_x2x_x4_dspslv_dsp0_aclk_m",
							  NULL, dspsys_regmap, 0x24, 7, GATE_NOT_SHARED, NULL, dev);
	gates[CLKGEN_AXI4_DSPSYS_SLV_ACLK] = thead_gate_clk_register("clkgen_axi4_dspsys_slv_aclk", NULL, dspsys_regmap,
							  0x24, 20, GATE_NOT_SHARED, NULL, dev);
	gates[CLKGEN_AXI4_DSPSYS_SLV_PCLK] = thead_gate_clk_register("clkgen_axi4_dspsys_slv_pclk", NULL, dspsys_regmap,
							  0x24, 21, GATE_NOT_SHARED, NULL, dev);
	gates[CLKGEN_AXI4_DSPSYS_ACLK] = thead_gate_clk_register("clkgen_axi4_dspsys_aclk", NULL, dspsys_regmap,
							  0x24, 23, GATE_NOT_SHARED, NULL, dev);
	gates[CLKGEN_AXI4_DSPSYS_PCLK] = thead_gate_clk_register("clkgen_axi4_dspsys_pclk", NULL, dspsys_regmap,
							  0x24, 24, GATE_NOT_SHARED, NULL, dev);
	if (tee_dspsys_regmap) {
		gates[CLKGEN_IOPMP_DSP1_PCLK] = thead_gate_clk_register("clkgen_iopmp_dsp1_pclk", NULL, tee_dspsys_regmap,
							  0x24, 25, GATE_NOT_SHARED, NULL, dev);
		gates[CLKGEN_IOPMP_DSP0_PCLK] = thead_gate_clk_register("clkgen_iopmp_dsp0_pclk", NULL, tee_dspsys_regmap,
							  0x24, 26, GATE_NOT_SHARED, NULL, dev);
	}

	clk_gate_data.clks = gates;
	clk_gate_data.clk_num = ARRAY_SIZE(gates);

	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &clk_gate_data);
	if (ret < 0) {
		dev_err(dev, "failed to register gate clks for light dspsys\n");
		goto unregister_clks;
	}

	dev_info(dev, "succeed to register dspsys gate clock provider\n");

	return 0;

unregister_clks:
	thead_unregister_clocks(gates, ARRAY_SIZE(gates));
	return ret;
}

static const struct of_device_id dspsys_clk_gate_of_match[] = {
	{ .compatible = "thead,dspsys-gate-controller" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, dspsys_clk_gate_of_match);

static struct platform_driver light_dspsys_clk_driver = {
	.probe = light_dspsys_clk_probe,
	.driver = {
		.name = "dspsys-clk-gate-provider",
		.of_match_table = of_match_ptr(dspsys_clk_gate_of_match),
	},
};

module_platform_driver(light_dspsys_clk_driver);
MODULE_AUTHOR("wei.liu <lw312886@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead Light Fullmask dspsys clock gate provider");
MODULE_LICENSE("GPL v2");
