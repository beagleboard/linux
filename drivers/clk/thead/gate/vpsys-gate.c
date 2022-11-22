/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2022 Alibaba Group Holding Limited.
 */

#include <dt-bindings/clock/light-fm-ap-clock.h>
#include <dt-bindings/clock/light-vpsys.h>
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
#include "../clk.h"

static struct clk *gates[LIGHT_VPSYS_CLK_END];
static struct clk_onecell_data clk_gate_data;

static u32 share_cnt_g2d_clk_en;
static u32 share_cnt_fce_clk_en;

static int light_vpsys_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	void __iomem *gate_base;
	int ret;

	gate_base = devm_platform_ioremap_resource(pdev, 0);
	if (WARN_ON(IS_ERR(gate_base)))
		return PTR_ERR(gate_base);

	/* we assume that the gate clock is a root clock  */
	gates[LIGHT_VPSYS_G2D_PCLK] = thead_clk_light_gate_shared("clkgen_vpsys_g2d_pclk", NULL,
								   gate_base + 0x20, 3, &share_cnt_g2d_clk_en);
	gates[LIGHT_VPSYS_G2D_ACLK] = thead_clk_light_gate_shared("clkgen_vpsys_g2d_aclk", NULL,
								   gate_base + 0x20, 3, &share_cnt_g2d_clk_en);
	gates[LIGHT_VPSYS_G2D_CCLK] = thead_clk_light_gate_shared("clkgen_vpsys_g2d_cclk", NULL,
								   gate_base + 0x20, 3, &share_cnt_g2d_clk_en);


	gates[LIGHT_VPSYS_FCE_PCLK] = thead_clk_light_gate_shared("clkgen_vpsys_fce_pclk", NULL,
								   gate_base + 0x20, 2, &share_cnt_fce_clk_en);
	gates[LIGHT_VPSYS_FCE_ACLK] = thead_clk_light_gate_shared("clkgen_vpsys_fce_aclk", NULL,
								   gate_base + 0x20, 2, &share_cnt_fce_clk_en);

	gates[LIGHT_VPSYS_VDEC_ACLK] = thead_clk_light_gate("clkgen_vdec_aclk", NULL, gate_base + 0x20, 4);
	gates[LIGHT_VPSYS_VDEC_CCLK] = thead_clk_light_gate("clkgen_vdec_cclk", NULL, gate_base + 0x20, 5);
	gates[LIGHT_VPSYS_VDEC_PCLK] = thead_clk_light_gate("clkgen_vdec_pclk", NULL, gate_base + 0x20, 6);

	gates[LIGHT_VPSYS_VENC_CCLK] = thead_clk_light_gate("clkgen_venc_cclk", NULL, gate_base + 0x20, 8);
	gates[LIGHT_VPSYS_VENC_PCLK] = thead_clk_light_gate("clkgen_venc_pclk", NULL, gate_base + 0x20, 9);
	gates[LIGHT_VPSYS_VENC_ACLK] = thead_clk_light_gate("clkgen_venc_aclk", NULL, gate_base + 0x20, 7);

	clk_gate_data.clks = gates;
	clk_gate_data.clk_num = ARRAY_SIZE(gates);

	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &clk_gate_data);
	if (ret < 0) {
		dev_err(dev, "failed to register gate clks for light vpsys\n");
		goto unregister_clks;
	}

	dev_info(dev, "succeed to register vpsys gate clock provider\n");

	return 0;

unregister_clks:
	thead_unregister_clocks(gates, ARRAY_SIZE(gates));
	return ret;
}

static const struct of_device_id vpsys_clk_gate_of_match[] = {
	{ .compatible = "thead,vpsys-gate-controller" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, vpsys_clk_gate_of_match);

static struct platform_driver light_vpsys_clk_driver = {
	.probe = light_vpsys_clk_probe,
	.driver = {
		.name = "vpsys-clk-gate-provider",
		.of_match_table = of_match_ptr(vpsys_clk_gate_of_match),
	},
};

module_platform_driver(light_vpsys_clk_driver);
MODULE_AUTHOR("wei.liu <lw312886@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead Light Fullmask vpsys clock gate provider");
MODULE_LICENSE("GPL v2");
