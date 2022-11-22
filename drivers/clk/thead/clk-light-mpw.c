/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#include <dt-bindings/clock/light-mpw-clock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include "clk.h"

static struct clk *clks[LIGHT_CLK_END];
static struct clk_onecell_data clk_data;

static u32 share_cnt_cpu2cfg_x2x_clk_en;
static u32 share_cnt_cpu2peri_x2h_clk_en;
static u32 share_cnt_aon2cpu_a2x_clk_en;
static u32 share_cnt_dmac_clk_en;
static u32 share_cnt_x2h_cpusys_clk_en;
static u32 share_cnt_cpu2tee_x2h_clk_en;
static u32 share_cnt_cpu2aon_x2h_clk_en;
static u32 share_cnt_cpu2cfg_x2h_clk_en;
static u32 share_cnt_timer0_clk_en;
static u32 share_cnt_timer1_clk_en;
static u32 share_cnt_peri2ddr_x2x_clk_en;
static u32 share_cnt_usb3_drd_clk_en;
static u32 share_cnt_gmac_clk_en;
static u32 share_cnt_emmc0_clk_en;
static u32 share_cnt_emmc1_clk_en;
static u32 share_cnt_pwm_clk_en;
static u32 share_cnt_qspi0_clk_en;
static u32 share_cnt_qspi1_clk_en;
static u32 share_cnt_spi_clk_en;
static u32 share_cnt_gpio0_clk_en;
static u32 share_cnt_gpio1_clk_en;
static u32 share_cnt_gpio2_clk_en;
static u32 share_cnt_dmac_1_clk_en;
static u32 share_cnt_dmac_2_clk_en;
static u32 share_cnt_dmac_3_clk_en;

#ifdef THEAD_LIGHT_AON_CLK
static const char * const audio_pll_bypass_sels[] = {"audio_pll_foutpostdiv", "osc_24m", };
static const char * const sys_pll_bypass_sels[] = {"sys_pll_foutpostdiv", "osc_24m", };
#endif
#ifdef THEAD_LIGHT_DDR_CLK
static const char * const ddr_pll_bypass_sels[] = {"ddr_pll_foutpostdiv", "osc_24m", };
#endif
static const char * const cpu_pll0_bypass_sels[] = {"cpu_pll0_foutpostdiv", "osc_24m", };
static const char * const cpu_pll1_bypass_sels[] = {"cpu_pll1_foutpostdiv", "osc_24m", };
static const char * const gmac_pll_bypass_sels[] = {"gmac_pll_foutpostdiv", "osc_24m", };
static const char * const video_pll_bypass_sels[] = {"video_pll_foutpostdiv", "osc_24m", };

#ifdef THEAD_LIGHT_AON_CLK
static const char * const aonsys_clk_switch_0_sels[] = {"audio_pll_fout3", "osc_24m", };
static const char * const aonsys_clk_switch_1_sels[] = {"aonsys_clk_switch_0", "rc_24m", };
#endif

static const char * const c910_cclk_i0_sels[] = {"cpu_pll0_bypass", "osc_24m", };
static const char * const c910_cclk_sels[] = {"c910_cclk_i0", "cpu_pll1_foutpostdiv", };
static const char * const cpusys_ahb_hclk_sel[] = {"cpusys_ahb_hclk_div", "osc_24m"};
static const char * const cpusys_cfg_axi_aclk_sel[] = {"cpusys_cfg_axi_aclk_div", "osc_24m"};
static const char * const perisys_ahb_hclk_sel[] = {"perisys_ahb_hclk_div", "osc_24m"};
static const char * const clk_out_1_sel[] = {"clk_out_1_div", "osc_24m"};
static const char * const clk_out_2_sel[] = {"clk_out_2_div", "osc_24m"};
static const char * const clk_out_3_sel[] = {"clk_out_3_div", "osc_24m"};
static const char * const clk_out_4_sel[] = {"clk_out_4_div", "osc_24m"};

static const struct light_pll_rate_table light_audiopll_tbl[] = {
	LIGHT_PLL_RATE(2654208000U, 147456000U, 1, 110, 9932112,  6, 3),
	LIGHT_PLL_RATE(884736000U,  294912000U, 1, 36,  14495600, 3, 1),
};

static const struct light_pll_rate_table light_syspll_tbl[] = {
	LIGHT_PLL_RATE(2438553600U, 812851200U, 1, 101, 10173704, 3, 1),
	LIGHT_PLL_RATE(884736000U,  294912000U, 1, 36,  14495600, 3, 1),
};

static const struct light_pll_rate_table light_cpupll_tbl[] = {
	LIGHT_PLL_RATE(1800000000U, 1800000000U, 1, 75,  0, 1, 1),
	LIGHT_PLL_RATE(3000000000U, 1500000000U, 1, 125, 0, 2, 1),
	LIGHT_PLL_RATE(3000000000U, 1000000000U, 1, 125, 0, 3, 1),
	LIGHT_PLL_RATE(3000000000U, 125000000U, 1, 125,  0, 6, 4),
};

#ifdef THEAD_LIGHT_DDR_CLK
static const struct light_pll_rate_table light_ddrpll_tbl[] = {
	LIGHT_PLL_RATE(3192000000U, 798000000U, 1, 133, 0, 4, 1),
	LIGHT_PLL_RATE(3192000000U, 532000000U, 1, 133, 0, 6, 1),
	LIGHT_PLL_RATE(2112000000U, 1056000000U, 1, 88, 0, 2, 1),
};
#endif

#ifdef THEAD_LIGHT_AON_CLK
static struct light_pll_clk light_audio_pllvco = {
	.out_type = LIGHT_PLL_VCO,
	.clk_type = LIGHT_AUDIO_PLL,
	.rate_table = light_audiopll_tbl,
	.rate_count = ARRAY_SIZE(light_audiopll_tbl),
};

static struct light_pll_clk light_audio_plldiv = {
	.out_type = LIGHT_PLL_DIV,
	.clk_type = LIGHT_AUDIO_PLL,
	.rate_table = light_audiopll_tbl,
	.rate_count = ARRAY_SIZE(light_audiopll_tbl),
};

static struct light_pll_clk light_sys_pllvco = {
	.out_type = LIGHT_PLL_VCO,
	.clk_type = LIGHT_SYS_PLL,
	.rate_table = light_syspll_tbl,
	.rate_count = ARRAY_SIZE(light_syspll_tbl),
};

static struct light_pll_clk light_sys_plldiv = {
	.out_type = LIGHT_PLL_DIV,
	.clk_type = LIGHT_SYS_PLL,
	.rate_table = light_syspll_tbl,
	.rate_count = ARRAY_SIZE(light_syspll_tbl),
};
#endif

static struct light_pll_clk light_cpu_pll0vco = {
	.out_type = LIGHT_PLL_VCO,
	.clk_type = LIGHT_CPU_PLL0,
	.rate_table = light_cpupll_tbl,
	.rate_count = ARRAY_SIZE(light_cpupll_tbl),
};

static struct light_pll_clk light_cpu_pll0div = {
	.out_type = LIGHT_PLL_DIV,
	.clk_type = LIGHT_CPU_PLL0,
	.rate_table = light_cpupll_tbl,
	.rate_count = ARRAY_SIZE(light_cpupll_tbl),
};

static struct light_pll_clk light_cpu_pll1vco = {
	.out_type = LIGHT_PLL_VCO,
	.clk_type = LIGHT_CPU_PLL1,
	.rate_table = light_cpupll_tbl,
	.rate_count = ARRAY_SIZE(light_cpupll_tbl),
};

static struct light_pll_clk light_cpu_pll1div = {
	.out_type = LIGHT_PLL_DIV,
	.clk_type = LIGHT_CPU_PLL1,
	.rate_table = light_cpupll_tbl,
	.rate_count = ARRAY_SIZE(light_cpupll_tbl),
};

#ifdef THEAD_LIGHT_DDR_CLK
static struct light_pll_clk light_ddr_pllvco = {
	.out_type = LIGHT_PLL_VCO,
	.clk_type = LIGHT_DDR_PLL,
	.rate_table = light_ddrpll_tbl,
	.rate_count = ARRAY_SIZE(light_ddrpll_tbl),
};

static struct light_pll_clk light_ddr_plldiv = {
	.out_type = LIGHT_PLL_DIV,
	.clk_type = LIGHT_DDR_PLL,
	.rate_table = light_ddrpll_tbl,
	.rate_count = ARRAY_SIZE(light_ddrpll_tbl),
};
#endif

static int light_clocks_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	void __iomem *ap_base;
#ifdef THEAD_LIGHT_DDR_CLK
	void __iomem *ddr_base;
#endif
#ifdef THEAD_LIGHT_AON_CLK
	void __iomem *aon_base;
#endif
	int ret;

#ifdef THEAD_LIGHT_AON_CLK
	np = of_find_compatible_node(NULL, NULL, "thead,light-aon-clk");
	aon_base = of_iomap(np, 0);
	if (WARN_ON(!aon_base)) {
		ret = -ENOMEM;
		goto unregister_clks;
	}
	of_node_put(np);
#endif

	/* Clock source */
	clks[LIGHT_CLK_DUMMY] = thead_clk_fixed("dummy", 0);
	clks[LIGHT_CLK_32K] = of_clk_get_by_name(np, "osc_32k");
	clks[LIGHT_CLK_24M] = of_clk_get_by_name(np, "osc_24m");
	clks[LIGHT_RC_24M] = of_clk_get_by_name(np, "rc_24m");

	/* AP Fixed PLL */
	clks[LIGHT_VIDEO_PLL_FOUTVCO] = thead_clk_fixed("video_pll_foutvco", 2376000000);
	clks[LIGHT_VIDEO_PLL_FOUTPOSTDIV] = thead_clk_fixed("video_pll_foutpostdiv", 796000000);
	clks[LIGHT_GMAC_PLL_FOUTVCO] = thead_clk_fixed("gmac_pll_foutvco", 2000000000);
	clks[LIGHT_GMAC_PLL_FOUTPOSTDIV] = thead_clk_fixed("gmac_pll_foutpostdiv", 1000000000);

#ifdef THEAD_LIGHT_AON_CLK
	/* Aon PLL clocks */
	clks[LIGHT_AUDIO_PLL_FOUTVCO] = thead_light_pll("audio_pll_foutvco", "osc_24m", aon_base, &light_audio_pllvco);
	clks[LIGHT_AUDIO_PLL_FOUTPOSTDIV] = thead_light_pll("audio_pll_foutpostdiv", "osc_24m", aon_base, &light_audio_plldiv);
	clks[LIGHT_SYS_PLL_FOUTVCO] = thead_light_pll("sys_pll_foutvco", "osc_24m", aon_base, &light_sys_pllvco);
	clks[LIGHT_SYS_PLL_FOUTPOSTDIV] = thead_light_pll("sys_pll_foutpostdiv", "osc_24m", aon_base, &light_sys_plldiv);
#endif

#ifdef THEAD_LIGHT_DDR_CLK
	np = of_find_compatible_node(NULL, NULL, "thead,light-ddr-clk");
	ddr_base = of_iomap(np, 0);
	if (WARN_ON(!ddr_base)) {
		ret = -ENOMEM;
		goto unregister_clks;
	}
	of_node_put(np);

	/* DDR PLL */
	clks[LIGHT_DDR_PLL_FOUTVCO] = thead_light_pll("ddr_pll_foutvco", "osc_24m", ddr_base, &light_ddr_pllvco);
	clks[LIGHT_DDR_PLL_FOUTPOSTDIV] = thead_light_pll("ddr_pll_foutpostdiv", "osc_24m", ddr_base, &light_ddr_plldiv);
#endif

	np = dev->of_node;
	ap_base = devm_platform_ioremap_resource(pdev, 0);
	if (WARN_ON(IS_ERR(ap_base))) {
		ret = PTR_ERR(ap_base);
		goto unregister_clks;
	}

	/* AP PLL clocks */
	clks[LIGHT_CPU_PLL0_FOUTVCO] = thead_light_pll("cpu_pll0_foutvco", "osc_24m", ap_base, &light_cpu_pll0vco);
	clks[LIGHT_CPU_PLL0_FOUTPOSTDIV] = thead_light_pll("cpu_pll0_foutpostdiv", "osc_24m", ap_base, &light_cpu_pll0div);
	clks[LIGHT_CPU_PLL1_FOUTVCO] = thead_light_pll("cpu_pll1_foutvco", "osc_24m", ap_base, &light_cpu_pll1vco);
	clks[LIGHT_CPU_PLL1_FOUTPOSTDIV] = thead_light_pll("cpu_pll1_foutpostdiv", "osc_24m", ap_base, &light_cpu_pll1div);

	/* PLL bypass */
#ifdef THEAD_LIGHT_AON_CLK
	clks[LIGHT_AUDIO_PLL_BYPASS] = thead_light_clk_mux_flags("audio_pll_bypass", aon_base + 0x4, 31, 1, audio_pll_bypass_sels, ARRAY_SIZE(audio_pll_bypass_sels), CLK_SET_RATE_PARENT);
	clks[LIGHT_SYS_PLL_BYPASS] = thead_light_clk_mux_flags("sys_pll_bypass", aon_base + 0x14, 31, 1, sys_pll_bypass_sels, ARRAY_SIZE(sys_pll_bypass_sels), CLK_SET_RATE_PARENT);
#endif
#ifdef THEAD_LIGHT_DDR_CLK
	clks[LIGHT_DDR_PLL_BYPASS] = thead_light_clk_mux_flags("ddr_pll_bypass", ddr_base + 0xc, 31, 1, ddr_pll_bypass_sels, ARRAY_SIZE(ddr_pll_bypass_sels), CLK_SET_RATE_PARENT);
#endif
	clks[LIGHT_CPU_PLL0_BYPASS] = thead_light_clk_mux_flags("cpu_pll0_bypass", ap_base + 0x4, 31, 1, cpu_pll0_bypass_sels, ARRAY_SIZE(cpu_pll0_bypass_sels), CLK_SET_RATE_PARENT);
	clks[LIGHT_CPU_PLL1_BYPASS] = thead_light_clk_mux_flags("cpu_pll1_bypass", ap_base + 0x14, 31, 1, cpu_pll1_bypass_sels, ARRAY_SIZE(cpu_pll1_bypass_sels), CLK_SET_RATE_PARENT);
	clks[LIGHT_GMAC_PLL_BYPASS] = thead_light_clk_mux_flags("gmac_pll_bypass", ap_base + 0x24, 31, 1, gmac_pll_bypass_sels, ARRAY_SIZE(gmac_pll_bypass_sels), CLK_SET_RATE_PARENT);
	clks[LIGHT_VIDEO_PLL_BYPASS] = thead_light_clk_mux_flags("video_pll_bypass", ap_base + 0x34, 31, 1, video_pll_bypass_sels, ARRAY_SIZE(video_pll_bypass_sels), CLK_SET_RATE_PARENT);

	/* PLL FOUT */
#ifdef THEAD_LIGHT_AON_CLK
	clks[LIGHT_AUDIO_PLL_FOUT3] = thead_light_clk_fixed_factor("audio_pll_fout3", "audio_pll_bypass", 1, 6);
	clks[LIGHT_SYS_PLL_FOUT4] = thead_light_clk_fixed_factor("sys_pll_fout4", "sys_pll_bypass", 1, 8);
#endif
#ifdef THEAD_LIGHT_DDR_CLK
	clks[LIGHT_DDR_PLL_FOUT4] = thead_light_clk_fixed_factor("ddr_pll_fout4", "ddr_pll_bypass", 1, 8);
#endif
	clks[LIGHT_CPU_PLL0_FOUT4] = thead_light_clk_fixed_factor("cpu_pll0_fout4", "cpu_pll0_bypass", 1, 8);
	clks[LIGHT_CPU_PLL1_FOUT4] = thead_light_clk_fixed_factor("cpu_pll1_fout4", "cpu_pll1_bypass", 1, 8);
	clks[LIGHT_GMAC_PLL_FOUT1PH0] = thead_light_clk_fixed_factor("gmac_pll_fout1ph0", "gmac_pll_bypass", 1, 2);
	clks[LIGHT_GMAC_CORECLK] = thead_light_clk_fixed_factor("gmac_coreclk", "gmac_pll_fout1ph0", 1, 1);
	clks[LIGHT_GMAC_PLL_FOUT4] = thead_light_clk_fixed_factor("gmac_pll_fout4", "gmac_pll_bypass", 1, 8);
	clks[LIGHT_VIDEO_PLL_FOUT4] = thead_light_clk_fixed_factor("video_pll_fout4", "video_pll_bypass", 1, 8);
	clks[LIGHT_GMAC_PLL_FOUTVCO_DIV5] = thead_light_clk_fixed_factor("gmac_pll_foutvco_div5", "gmac_pll_foutvco", 1, 5);
	clks[LIGHT_OSC_CLK_DIV24] = thead_light_clk_fixed_factor("osc_clk_div24", "osc_24m", 1, 24);
	clks[LIGHT_CHIP_DBG_CCLK] = thead_light_clk_fixed_factor("chip_dbg_cclk", "osc_24m", 1, 1);
	clks[LIGHT_AXI_ACLK] = thead_light_clk_fixed_factor("cpusys_axi_aclk", "cpu_pll0_bypass", 1, 2);
	clks[LIGHT_X2H_HCLK] = thead_light_clk_fixed_factor("aonsys_x2h_hclk", "osc_24m", 1, 1);
	clks[LIGHT_EMMC_CLK_DIV] = thead_light_clk_fixed_factor("emmc_clk_div", "video_pll_bypass", 1, 4);
	clks[LIGHT_EMMC0_OSC_CLK] = thead_light_clk_fixed_factor("emmc0_osc_clk", "osc_24m", 1, 1);
	clks[LIGHT_EMMC1_OSC_CLK] = thead_light_clk_fixed_factor("emmc1_osc_clk", "osc_24m", 1, 1);
	clks[LIGHT_PWM_CCLK] = thead_light_clk_fixed_factor("pwm_cclk", "osc_24m", 1, 1);
	clks[LIGHT_USB3_PHY_REF_CLK] = thead_light_clk_fixed_factor("usb3_phy_ref_clk", "osc_24m", 1, 1);
	clks[LIGHT_SPI_CLK] = thead_light_clk_fixed_factor("spi_clk", "gmac_pll_foutvco_div5", 1, 1);
	clks[LIGHT_GPIO_DBCLK] = thead_light_clk_fixed_factor("gpio_dbclk", "osc_32k", 1, 1);

#ifdef THEAD_LIGHT_AON_CLK
	/* Aon sys mux tree */
	clks[LIGHT_AONSYS_CLK_SWITCH_0] = thead_light_clk_mux_flags("aonsys_clk_switch_0", aon_base + 0x100, 4, 1, aonsys_clk_switch_0_sels, ARRAY_SIZE(aonsys_clk_switch_0_sels), CLK_SET_RATE_PARENT);
	clks[LIGHT_AONSYS_CLK_SWITCH_1] = thead_light_clk_mux_flags("aonsys_clk_switch_1", aon_base + 0x100, 5, 1, aonsys_clk_switch_1_sels, ARRAY_SIZE(aonsys_clk_switch_1_sels), CLK_SET_RATE_PARENT);

	/* Aon sys div tree */
	clks[LIGHT_AONSYS_CLK] = thead_clk_light_divider("aonsys_clk", "aonsys_clk_switch_1", aon_base + 0x100, 0, 3, 3, MUX_TYPE_CDE, 0, 7);
	clks[LIGHT_SHARE_SRAM_CLK] = thead_clk_light_divider("share_sram_clk", "sys_pll_foutvco", aon_base + 0x104, 0, 4, 4, MUX_TYPE_DIV, 3, 12);

	/* Aon sys gate tree */
	clks[LIGHT_CLKGEN_RTC_PCLK] = thead_clk_light_gate("rtc_pclk_en", "aonsys_clk", aon_base + 0x120, 0);
	clks[LIGHT_CLKGEN_AOGPIO_PCLK] = thead_clk_light_gate("aogpio_pclk_en", "aonsys_clk", aon_base + 0x120, 1);
	clks[LIGHT_CLKGEN_AOI2C_PCLK] = thead_clk_light_gate("aoi2c_pclk_en", "aonsys_clk", aon_base + 0x120, 2);
	clks[LIGHT_CLKGEN_PVTC_PCLK] = thead_clk_light_gate("pvtc_pclk_en", "aonsys_clk", aon_base + 0x120, 3);
	clks[LIGHT_CLKGEN_SRAM_AXI_ACLK] = thead_clk_light_gate("share_sram_clk_en", "aonsys_clk", aon_base + 0x120, 4);
	clks[LIGHT_CLKGEN_AOPAD_PCLK] = thead_clk_light_gate("aopad_pclk_en", "aonsys_clk", aon_base + 0x120, 5);
	clks[LIGHT_CLKGEN_AOAPB_HCLK] = thead_clk_light_gate("aoapb_hclk_en", "aonsys_clk", aon_base + 0x120, 6);
	clks[LIGHT_CLKGEN_AOSRAM_HCLK] = thead_clk_light_gate("aosram_hclk_en", "aonsys_clk", aon_base + 0x120, 7);
	clks[LIGHT_CLKGEN_AOAHB_HCLK] = thead_clk_light_gate("aoahb_hclk_en", "aonsys_clk", aon_base + 0x120, 8);
	clks[LIGHT_CLKGEN_AOGPIO_DBCLK] = thead_clk_light_gate("aogpio_dbclk_en", "aonsys_clk", aon_base + 0x120, 9);
	clks[LIGHT_CLKGEN_AOTIMER_PCLK] = thead_clk_light_gate("aotimer_pclk_en", "aonsys_clk", aon_base + 0x120, 10);
	clks[LIGHT_CLKGEN_AOTIMER_CCLK] = thead_clk_light_gate("aotimer_cclk_en", "aonsys_clk", aon_base + 0x120, 11);
	clks[LIGHT_CLKGEN_CPU2RAM_X2X_ACLK_S] = thead_clk_light_gate("apsys_clk_en", "aonsys_clk", aon_base + 0x130, 0);
#endif

	/* AP sys mux tree */
	clks[LIGHT_C910_CCLK_I0] = thead_light_clk_mux_flags("c910_cclk_i0", ap_base + 0x100, 1, 1, c910_cclk_i0_sels, ARRAY_SIZE(c910_cclk_i0_sels), CLK_SET_RATE_PARENT);
	clks[LIGHT_C910_CCLK] = thead_light_clk_mux_flags("c910_cclk", ap_base + 0x100, 0, 1, c910_cclk_sels, ARRAY_SIZE(c910_cclk_sels), CLK_SET_RATE_PARENT);
	clks[LIGHT_CPUSYS_AHB_HCLK] = thead_light_clk_mux_flags("cpusys_ahb_hclk", ap_base + 0x120, 5, 1, cpusys_ahb_hclk_sel, ARRAY_SIZE(cpusys_ahb_hclk_sel), CLK_SET_RATE_PARENT);
	clks[LIGHT_CPUSYS_CFG_AXI_ACLK] = thead_light_clk_mux_flags("cpusys_cfg_axi_aclk", ap_base + 0x138, 5, 1, cpusys_cfg_axi_aclk_sel, ARRAY_SIZE(cpusys_cfg_axi_aclk_sel), CLK_SET_RATE_PARENT);
	clks[LIGHT_PERISYS_AHB_HCLK] = thead_light_clk_mux_flags("perisys_ahb_hclk", ap_base + 0x40, 5, 1, perisys_ahb_hclk_sel, ARRAY_SIZE(perisys_ahb_hclk_sel), CLK_SET_RATE_PARENT);
	clks[LIGHT_CLK_OUT_1] = thead_light_clk_mux_flags("clk_out_1", ap_base + 0x1b4, 4, 1, clk_out_1_sel, ARRAY_SIZE(clk_out_1_sel), CLK_SET_RATE_PARENT);
	clks[LIGHT_CLK_OUT_2] = thead_light_clk_mux_flags("clk_out_2", ap_base + 0x1b8, 4, 1, clk_out_2_sel, ARRAY_SIZE(clk_out_2_sel), CLK_SET_RATE_PARENT);
	clks[LIGHT_CLK_OUT_3] = thead_light_clk_mux_flags("clk_out_3", ap_base + 0x1bc, 4, 1, clk_out_3_sel, ARRAY_SIZE(clk_out_3_sel), CLK_SET_RATE_PARENT);
	clks[LIGHT_CLK_OUT_4] = thead_light_clk_mux_flags("clk_out_4", ap_base + 0x1c0, 4, 1, clk_out_4_sel, ARRAY_SIZE(clk_out_4_sel), CLK_SET_RATE_PARENT);

	/* AP sys div tree */
	clks[LIGHT_CPUSYS_AHB_HCLK_DIV] = thead_clk_light_divider("cpusys_ahb_hclk_div", "gmac_pll_fout1ph0", ap_base + 0x120, 0, 4, 4, MUX_TYPE_DIV, 2, 8);
	clks[LIGHT_APB3_CPUSYS_PCLK] = thead_clk_light_divider("apb3_cpusys_pclk", "cpusys_ahb_hclk", ap_base + 0x130, 0, 3, 3, MUX_TYPE_CDE, 1, 7);
	clks[LIGHT_CPUSYS_SUB_AXI_ACLK] = thead_clk_light_divider("cpusys_sub_axi_aclk", "gmac_pll_bypass", ap_base + 0x134, 0, 4, 4, MUX_TYPE_DIV, 2, 8);
	clks[LIGHT_CPUSYS_CFG_AXI_ACLK_DIV] = thead_clk_light_divider("cpusys_cfg_axi_aclk_div", "gmac_pll_bypass", ap_base + 0x138, 0, 4, 4, MUX_TYPE_DIV, 8, 15);
	clks[LIGHT_TEESYS_HCLK] = thead_clk_light_divider("teesys_hclk", "gmac_pll_fout1ph0", ap_base + 0x154, 0, 2, 2, MUX_TYPE_DIV, 2, 3);
	clks[LIGHT_DMAC_1_CLK] = thead_clk_light_divider("dmac_1_clk", "video_pll_bypass", ap_base + 0x158, 0, 2, 2, MUX_TYPE_CDE, 0, 7);
	clks[LIGHT_DMAC_2_CLK] = thead_clk_light_divider("dmac_2_clk", "video_pll_bypass", ap_base + 0x16c, 0, 2, 2, MUX_TYPE_CDE, 0, 7);
	clks[LIGHT_DMAC_3_CLK] = thead_clk_light_divider("dmac_3_clk", "gmac_pll_bypass", ap_base + 0x160, 0, 2, 2, MUX_TYPE_CDE, 0, 7);
	clks[LIGHT_AXI_PORT4_CLK] = thead_clk_light_divider("axi_port4_clk", "video_pll_bypass", ap_base + 0x164, 0, 2, 2, MUX_TYPE_CDE, 0, 7);
	clks[LIGHT_PERISYS_AHB_HCLK_DIV] = thead_clk_light_divider("perisys_ahb_hclk_div", "gmac_pll_fout1ph0", ap_base + 0x140, 0, 4, 4, MUX_TYPE_DIV, 2, 8);
	clks[LIGHT_PERISYS_APB_PCLK] = thead_clk_light_divider("perisys_apb_pclk", "perisys_ahb_hclk", ap_base + 0x150, 0, 3, 3, MUX_TYPE_CDE, 3, 7);
	clks[LIGHT_CLK_OUT_1_DIV] = thead_clk_light_divider("clk_out_1_div", "osc_24m", ap_base + 0x1b4, 0, 3, 3, MUX_TYPE_DIV, 2, 7);
	clks[LIGHT_CLK_OUT_2_DIV] = thead_clk_light_divider("clk_out_2_div", "osc_24m", ap_base + 0x1b8, 0, 3, 3, MUX_TYPE_DIV, 2, 7);
	clks[LIGHT_CLK_OUT_3_DIV] = thead_clk_light_divider("clk_out_3_div", "osc_24m", ap_base + 0x1bc, 0, 3, 3, MUX_TYPE_DIV, 2, 7);
	clks[LIGHT_CLK_OUT_4_DIV] = thead_clk_light_divider("clk_out_4_div", "osc_24m", ap_base + 0x1c0, 0, 3, 3, MUX_TYPE_DIV, 2, 7);

	/* AP sys gate tree */
	clks[LIGHT_CLKGEN_PERISYS_AXI_ACLK] = thead_clk_light_gate("clkgen_perisys_axi_aclk", "perisys_ahb_hclk", ap_base + 0x200, 31);
	clks[LIGHT_CLKGEN_PERISYS_AHB_HCLK] = thead_clk_light_gate("clkgen_perisys_ahb_hclk", "perisys_ahb_hclk", ap_base + 0x200, 30);
	clks[LIGHT_CLKGEN_PERISYS_APB1_HCLK] = thead_clk_light_gate("clkgen_perisys_apb1_hclk", "perisys_ahb_hclk", ap_base + 0x200, 29);
	clks[LIGHT_CLKGEN_PERISYS_APB2_HCLK] = thead_clk_light_gate("clkgen_perisys_apb2_hclk", "perisys_ahb_hclk", ap_base + 0x200, 28);
	clks[LIGHT_CLKGEN_USB3_DRD_PHY_REF_CLK] = thead_clk_light_gate("clkgen_usb3_drd_phy_ref_clk", "usb3_phy_ref_clk", ap_base + 0x200, 27);
	clks[LIGHT_CLKGEN_USB3_DRD_CTRL_REF_CLK] = thead_clk_light_gate("clkgen_usb3_drd_ctrl_ref_clk", "usb3_ctrl_ref_clk", ap_base + 0x200, 26);
	clks[LIGHT_CLKGEN_USB3_DRD_SPDCLK] = thead_clk_light_gate("clkgen_usb3_drd_spdclk", "osc_clk_div24", ap_base + 0x200, 25);
	clks[LIGHT_CLKGEN_EMMC1_X2X_ACLK] = thead_clk_light_gate("clkgen_emmc1_x2x_aclk", "perisys_ahb_hclk", ap_base + 0x200, 23);
	clks[LIGHT_CLKGEN_EMMC0_X2X_ACLK] = thead_clk_light_gate("clkgen_emmc0_x2x_aclk", "perisys_ahb_hclk", ap_base + 0x200, 22);
	clks[LIGHT_CLKGEN_UART0_PCLK] = thead_clk_light_gate("clkgen_uart0_pclk", "perisys_apb_pclk", ap_base + 0x200, 14);
	clks[LIGHT_CLKGEN_UART1_PCLK] = thead_clk_light_gate("clkgen_uart1_pclk", "perisys_apb_pclk", ap_base + 0x200, 13);
	clks[LIGHT_CLKGEN_UART2_PCLK] = thead_clk_light_gate("clkgen_uart2_pclk", "perisys_apb_pclk", ap_base + 0x200, 12);
	clks[LIGHT_CLKGEN_UART3_PCLK] = thead_clk_light_gate("clkgen_uart3_pclk", "perisys_apb_pclk", ap_base + 0x200, 11);
	clks[LIGHT_CLKGEN_UART4_PCLK] = thead_clk_light_gate("clkgen_uart4_pclk", "perisys_apb_pclk", ap_base + 0x200, 10);
	clks[LIGHT_CLKGEN_UART5_PCLK] = thead_clk_light_gate("clkgen_uart5_pclk", "perisys_apb_pclk", ap_base + 0x200, 9);
	clks[LIGHT_CLKGEN_I2C0_IC_CLK] = thead_clk_light_gate("clkgen_i2c0_ic_clk", "perisys_apb_pclk", ap_base + 0x200, 5);
	clks[LIGHT_CLKGEN_I2C1_IC_CLK] = thead_clk_light_gate("clkgen_i2c1_ic_clk", "perisys_apb_pclk", ap_base + 0x200, 4);
	clks[LIGHT_CLKGEN_I2C2_IC_CLK] = thead_clk_light_gate("clkgen_i2c2_ic_clk", "perisys_apb_pclk", ap_base + 0x200, 3);
	clks[LIGHT_CLKGEN_I2C3_IC_CLK] = thead_clk_light_gate("clkgen_i2c3_ic_clk", "perisys_apb_pclk", ap_base + 0x200, 2);
	clks[LIGHT_CLKGEN_I2C4_IC_CLK] = thead_clk_light_gate("clkgen_i2c4_ic_clk", "perisys_apb_pclk", ap_base + 0x200, 1);
	clks[LIGHT_CLKGEN_I2C5_IC_CLK] = thead_clk_light_gate("clkgen_i2c5_ic_clk", "perisys_apb_pclk", ap_base + 0x200, 0);

	clks[LIGHT_CLKGEN_AXI_DUMMY_SLV_4_ACLK] = thead_clk_light_gate("clkgen_axi_dummy_slv_4_aclk", "cpusys_cfg_axi_aclk", ap_base + 0x204, 28);
	clks[LIGHT_CLKGEN_AXI_DUMMY_SLV_3_ACLK] = thead_clk_light_gate("clkgen_axi_dummy_slv_3_aclk", "cpusys_cfg_axi_aclk", ap_base + 0x204, 27);
	clks[LIGHT_CLKGEN_AXI_DUMMY_SLV_2_ACLK] = thead_clk_light_gate("clkgen_axi_dummy_slv_2_aclk", "cpusys_cfg_axi_aclk", ap_base + 0x204, 26);
	clks[LIGHT_CLKGEN_AXI_DUMMY_SLV_1_ACLK] = thead_clk_light_gate("clkgen_axi_dummy_slv_1_aclk", "cpusys_cfg_axi_aclk", ap_base + 0x204, 25);
	clks[LIGHT_CLKGEN_APB_CPU2FG_HCLK] = thead_clk_light_gate("clkgen_apb_cpu2cfg_hclk", "cpusys_cfg_axi_aclk", ap_base + 0x204, 24);
	clks[LIGHT_CLKGEN_CPU2RAM_X2X_ACLK_M] = thead_clk_light_gate("clkgen_cpu2ram_x2x_aclk_m", "cpusys_axi_aclk", ap_base + 0x204, 21);
	clks[LIGHT_CLKGEN_AXI4_CPUSYS2_ACLK] = thead_clk_light_gate("clkgen_axi4_cpusys2_aclk", "cpusys_sub_axi_aclk", ap_base + 0x204, 20);
	clks[LIGHT_CLKGEN_X2X_CPUSYS_ACLK_M] = thead_clk_light_gate("clkgen_x2x_cpusys_aclk_m", "cpusys_sub_axi_aclk", ap_base + 0x204, 19);
	clks[LIGHT_CLKGEN_CHIP_DBG_ACLK] = thead_clk_light_gate("clkgen_chip_dbg_aclk", "cpusys_sub_axi_aclk", ap_base + 0x204, 18);
	clks[LIGHT_CLKGEN_AXI4_CFG_BUS_ACLK] = thead_clk_light_gate("clkgen_axi4_cfg_bus_aclk", "cpusys_cfg_axi_aclk", ap_base + 0x204, 17);
	clks[LIGHT_CLKGEN_AHB2_CPUSYS_HCLK] = thead_clk_light_gate("clkgen_ahb2_cpusys_hclk", "cpusys_ahb_hclk", ap_base + 0x204, 11);
	clks[LIGHT_CLKGEN_APB3_CPUSYS_HCLK] = thead_clk_light_gate("clkgen_apb3_cpusys_hclk", "cpusys_ahb_hclk", ap_base + 0x204, 10);
	clks[LIGHT_CLKGEN_C910_BROM_HCLK] = thead_clk_light_gate("clkgen_c910_brom_hclk", "cpusys_ahb_hclk", ap_base + 0x204, 9);
	clks[LIGHT_CLKGEN_MBOX0_PCLK] = thead_clk_light_gate("clkgen_mbox0_pclk", "apb3_cpusys_pclk", ap_base + 0x204, 7);
	clks[LIGHT_CLKGEN_MBOX1_PCLK] = thead_clk_light_gate("clkgen_mbox1_pclk", "apb3_cpusys_pclk", ap_base + 0x204, 6);
	clks[LIGHT_CLKGEN_MBOX2_PCLK] = thead_clk_light_gate("clkgen_mbox2_pclk", "apb3_cpusys_pclk", ap_base + 0x204, 5);
	clks[LIGHT_CLKGEN_MBOX3_PCLK] = thead_clk_light_gate("clkgen_mbox3_pclk", "apb3_cpusys_pclk", ap_base + 0x204, 4);
	clks[LIGHT_CLKGEN_WDT0_PCLK] = thead_clk_light_gate("clkgen_wdt0_pclk", "apb3_cpusys_pclk", ap_base + 0x204, 3);
	clks[LIGHT_CLKGEN_WDT1_PCLK] = thead_clk_light_gate("clkgen_wdt1_pclk", "apb3_cpusys_pclk", ap_base + 0x204, 2);

	clks[LIGHT_CLKGEN_TRNG_RB_HCLK] = thead_clk_light_gate("clkgen_trng_rb_hclk", "teesys_hclk", ap_base + 0x208, 19);
	clks[LIGHT_CLKGEN_ADC_PCLK] = thead_clk_light_gate("clkgen_adc_pclk", "perisys_apb_pclk", ap_base + 0x208, 18);
	clks[LIGHT_CLKGEN_AXI_ACLK_4] = thead_clk_light_gate("axi_aclk_4", "axi_port4_clk", ap_base + 0x208, 17);
	clks[LIGHT_CLKGEN_AXI_ACLK_3] = thead_clk_light_gate("axi_aclk_3", "dmac_3_clk", ap_base + 0x208, 16);
	clks[LIGHT_CLKGEN_AXI_ACLK_2] = thead_clk_light_gate("axi_aclk_2", "dmac_2_clk", ap_base + 0x208, 15);
	clks[LIGHT_CLKGEN_AXI_ACLK_1] = thead_clk_light_gate("axi_aclk_1", "dmac_1_clk", ap_base + 0x208, 14);
	clks[LIGHT_CLKGEN_AXI_ACLK_0] = thead_clk_light_gate("axi_aclk_0", "cpusys_axi_aclk", ap_base + 0x208, 13);
	clks[LIGHT_CLKGEN_SRAM_AXI_PCLK] = thead_clk_light_gate("clkgen_sram_axi_pclk", "cpusys_cfg_axi_aclk", ap_base + 0x208, 9);
	clks[LIGHT_CLKGEN_AHB2_TEESYS_HCLK] = thead_clk_light_gate("clkgen_ahb2_teesys_hclk", "teesys_hclk", ap_base + 0x208, 8);
	clks[LIGHT_CLKGEN_EFUSE_MPW_PCLK] = thead_clk_light_gate("clkgen_efuse_mpw_pclk", "perisys_apb_pclk", ap_base + 0x208, 7);
	clks[LIGHT_CLKGEN_CLK_OUT_4_CLK] = thead_clk_light_gate("clkgen_clk_out_4_clk", "clk_out_4", ap_base + 0x208, 6);
	clks[LIGHT_CLKGEN_CLK_OUT_3_CLK] = thead_clk_light_gate("clkgen_clk_out_3_clk", "clk_out_3", ap_base + 0x208, 5);
	clks[LIGHT_CLKGEN_CLK_OUT_2_CLK] = thead_clk_light_gate("clkgen_clk_out_2_clk", "clk_out_2", ap_base + 0x208, 4);
	clks[LIGHT_CLKGEN_CLK_OUT_1_CLK] = thead_clk_light_gate("clkgen_clk_out_1_clk", "clk_out_1", ap_base + 0x208, 3);
	clks[LIGHT_CLKGEN_DDR_APB_PCLK] = thead_clk_light_gate("clkgen_ddr_apb_pclk", "cpusys_cfg_axi_aclk", ap_base + 0x208, 2);
	clks[LIGHT_CLKGEN_PADCTRL_APSYS_PCLK] = thead_clk_light_gate("clkgen_padctrl_apsys_pclk", "cpusys_cfg_axi_aclk", ap_base + 0x208, 1);
	clks[LIGHT_CLKGEN_CHIP_DBG_CCLK] = thead_clk_light_gate("clkgen_chip_dbg_cclk", "chip_dbg_cclk", ap_base + 0x208, 0);

	/* register AP shared gate */
	clks[LIGHT_CLKGEN_CPU2CFG_X2X_ACLK_M] = thead_clk_light_gate_shared("clkgen_cpu2cfg_x2x_aclk_m", "cpusys_axi_aclk", ap_base + 0x204, 22, &share_cnt_cpu2cfg_x2x_clk_en);
	clks[LIGHT_CLKGEN_CPU2CFG_X2X_ACLK_S] = thead_clk_light_gate_shared("clkgen_cpu2cfg_x2x_aclk_s", "cpusys_cfg_axi_aclk", ap_base + 0x204, 22, &share_cnt_cpu2cfg_x2x_clk_en);
	clks[LIGHT_CLKGEN_CPU2PERI_X2H_MHCLK] = thead_clk_light_gate_shared("clkgen_cpu2peri_x2h_mhclk", "perisys_ahb_hclk", ap_base + 0x204, 12, &share_cnt_cpu2peri_x2h_clk_en);
	clks[LIGHT_CLKGEN_CPU2CFG_X2H_ACLK_S] = thead_clk_light_gate_shared("clkgen_cpu2peri_x2h_aclk", "cpusys_axi_aclk", ap_base + 0x204, 12, &share_cnt_cpu2peri_x2h_clk_en);
	clks[LIGHT_CLKGEN_AON2CPU_A2X_ACLK] = thead_clk_light_gate_shared("clkgen_aon2cpu_a2x_aclk", "cpusys_sub_axi_aclk", ap_base + 0x204, 23, &share_cnt_aon2cpu_a2x_clk_en);
	clks[LIGHT_CLKGEN_AON2CPU_A2X_HCLK] = thead_clk_light_gate_shared("clkgen_aon2cpu_a2x_hclk", "aonsys_x2h_hclk", ap_base + 0x204, 23, &share_cnt_aon2cpu_a2x_clk_en);
	clks[LIGHT_CLKGEN_DMAC_ACLK] = thead_clk_light_gate_shared("clkgen_dmac_aclk", "cpusys_sub_axi_aclk", ap_base + 0x204, 8, &share_cnt_dmac_clk_en);
	clks[LIGHT_CLKGEN_DMAC_HCLK] = thead_clk_light_gate_shared("clkgen_dmac_hclk", "cpusys_ahb_hclk", ap_base + 0x204, 8, &share_cnt_dmac_clk_en);
	clks[LIGHT_CLKGEN_X2H_CPUSYS_ACLK] = thead_clk_light_gate_shared("clkgen_x2h_cpusys_aclk", "cpusys_cfg_axi_aclk", ap_base + 0x204, 16, &share_cnt_x2h_cpusys_clk_en);
	clks[LIGHT_CLKGEN_X2H_CPUSYS_MHCLK] = thead_clk_light_gate_shared("clkgen_x2h_cpusys_mhclk", "cpusys_ahb_hclk", ap_base + 0x204, 16, &share_cnt_x2h_cpusys_clk_en);
	clks[LIGHT_CLKGEN_CPU2TEE_X2H_ACLK] = thead_clk_light_gate_shared("clkgen_cpu2tee_x2h_aclk", "cpusys_cfg_axi_aclk", ap_base + 0x204, 15, &share_cnt_cpu2tee_x2h_clk_en);
	clks[LIGHT_CLKGEN_CPU2TEE_X2H_MHCLK] = thead_clk_light_gate_shared("clkgen_cpu2tee_x2h_mhclk", "teesys_hclk", ap_base + 0x204, 15, &share_cnt_cpu2tee_x2h_clk_en);
	clks[LIGHT_CLKGEN_CPU2AON_X2H_ACLK] = thead_clk_light_gate_shared("clkgen_cpu2aon_x2h_aclk", "cpusys_cfg_axi_aclk", ap_base + 0x204, 14, &share_cnt_cpu2aon_x2h_clk_en);
	clks[LIGHT_CLKGEN_CPU2AON_X2H_MHCLK] = thead_clk_light_gate_shared("clkgen_cpu2aon_x2h_mhclk", "aonsys_x2h_hclk", ap_base + 0x204, 14, &share_cnt_cpu2aon_x2h_clk_en);
	clks[LIGHT_CLKGEN_CPU2CFG_X2H_ACLK] = thead_clk_light_gate_shared("clkgen_cpu2cfg_x2h_aclk", "cpusys_cfg_axi_aclk", ap_base + 0x204, 13, &share_cnt_cpu2cfg_x2h_clk_en);
	clks[LIGHT_CLKGEN_CPU2CFG_X2H_MHCLK] = thead_clk_light_gate_shared("clkgen_cpu2cfg_x2h_mhclk", "cpusys_cfg_axi_aclk", ap_base + 0x204, 13, &share_cnt_cpu2cfg_x2h_clk_en);
	clks[LIGHT_CLKGEN_TIMER0_CCLK] = thead_clk_light_gate_shared("clkgen_timer0_cclk", "apb3_cpusys_pclk", ap_base + 0x204, 1, &share_cnt_timer0_clk_en);
	clks[LIGHT_CLKGEN_TIMER0_PCLK] = thead_clk_light_gate_shared("clkgen_timer0_pclk", "apb3_cpusys_pclk", ap_base + 0x204, 1, &share_cnt_timer0_clk_en);
	clks[LIGHT_CLKGEN_TIMER1_CCLK] = thead_clk_light_gate_shared("clkgen_timer1_cclk", "apb3_cpusys_pclk", ap_base + 0x204, 0, &share_cnt_timer1_clk_en);
	clks[LIGHT_CLKGEN_TIMER1_PCLK] = thead_clk_light_gate_shared("clkgen_timer1_pclk", "apb3_cpusys_pclk", ap_base + 0x204, 0, &share_cnt_timer1_clk_en);
	clks[LIGHT_CLKGEN_PERI2DDR_X2X_ACLK_M] = thead_clk_light_gate_shared("clkgen_peri2ddr_x2x_aclk_m", "perisys_ahb_hclk", ap_base + 0x204, 29, &share_cnt_peri2ddr_x2x_clk_en);
	clks[LIGHT_CLKGEN_PERI2DDR_X2X_ACLK_S] = thead_clk_light_gate_shared("clkgen_peri2ddr_x2x_aclk_s", "axi_port4_clk", ap_base + 0x204, 29, &share_cnt_peri2ddr_x2x_clk_en);
	clks[LIGHT_CLKGEN_USB3_DRD_ACLK] = thead_clk_light_gate_shared("clkgen_usb3_drd_aclk", "perisys_ahb_hclk", ap_base + 0x200, 24, &share_cnt_usb3_drd_clk_en);
	clks[LIGHT_CLKGEN_USB3_DRD_PCLK] = thead_clk_light_gate_shared("clkgen_usb3_drd_pclk", "perisys_apb_pclk", ap_base + 0x200, 24, &share_cnt_usb3_drd_clk_en);
	clks[LIGHT_CLKGEN_GMAC_HCLK] = thead_clk_light_gate_shared("clkgen_gmac_hclk", "perisys_ahb_hclk", ap_base + 0x200, 19, &share_cnt_gmac_clk_en);
	clks[LIGHT_CLKGEN_GMAC_ACLK] = thead_clk_light_gate_shared("clkgen_gmac_aclk", "perisys_ahb_hclk", ap_base + 0x200, 19, &share_cnt_gmac_clk_en);
	clks[LIGHT_CLKGEN_GMAC_PCLK] = thead_clk_light_gate_shared("clkgen_gmac_pclk", "perisys_apb_pclk", ap_base + 0x200, 19, &share_cnt_gmac_clk_en);
	clks[LIGHT_CLKGEN_GMAC_CCLK] = thead_clk_light_gate_shared("clkgen_gmac_cclk", "gmac_coreclk", ap_base + 0x200, 19, &share_cnt_gmac_clk_en);
	clks[LIGHT_CLKGEN_EMMC0_HCLK] = thead_clk_light_gate_shared("clkgen_emmc0_hclk", "perisys_ahb_hclk", ap_base + 0x200, 21, &share_cnt_emmc0_clk_en);
	clks[LIGHT_CLKGEN_EMMC0_ACLK] = thead_clk_light_gate_shared("clkgen_emmc0_aclk", "perisys_ahb_hclk", ap_base + 0x200, 21, &share_cnt_emmc0_clk_en);
	clks[LIGHT_CLKGEN_EMMC0_REF_CLK] = thead_clk_light_gate_shared("clkgen_emmc0_ref_clk", "emmc_clk_div", ap_base + 0x200, 21, &share_cnt_emmc0_clk_en);
	clks[LIGHT_CLKGEN_EMMC0_OSC_CLK] = thead_clk_light_gate_shared("clkgen_emmc0_osc_clk", "emmc0_osc_clk", ap_base + 0x200, 21, &share_cnt_emmc0_clk_en);
	clks[LIGHT_CLKGEN_EMMC1_HCLK] = thead_clk_light_gate_shared("clkgen_emmc1_hclk", "perisys_ahb_hclk", ap_base + 0x200, 20, &share_cnt_emmc1_clk_en);
	clks[LIGHT_CLKGEN_EMMC1_ACLK] = thead_clk_light_gate_shared("clkgen_emmc1_aclk", "perisys_ahb_hclk", ap_base + 0x200, 20, &share_cnt_emmc1_clk_en);
	clks[LIGHT_CLKGEN_EMMC1_REF_CLK] = thead_clk_light_gate_shared("clkgen_emmc1_ref_clk", "emmc_clk_div", ap_base + 0x200, 20, &share_cnt_emmc1_clk_en);
	clks[LIGHT_CLKGEN_EMMC1_OSC_CLK] = thead_clk_light_gate_shared("clkgen_emmc1_osc_clk", "emmc1_osc_clk", ap_base + 0x200, 20, &share_cnt_emmc1_clk_en);
	clks[LIGHT_CLKGEN_PWM_PCLK] = thead_clk_light_gate_shared("clkgen_pwm_pclk", "perisys_ahb_hclk", ap_base + 0x200, 18, &share_cnt_pwm_clk_en);
	clks[LIGHT_CLKGEN_PWM_CCLK] = thead_clk_light_gate_shared("clkgen_pwm_cclk", "pwm_cclk", ap_base + 0x200, 18, &share_cnt_pwm_clk_en);
	clks[LIGHT_CLKGEN_QSPI0_PCLK] = thead_clk_light_gate_shared("clkgen_qspi0_pclk", "perisys_apb_pclk", ap_base + 0x200, 17, &share_cnt_qspi0_clk_en);
	clks[LIGHT_CLKGEN_QSPI0_SSI_CLK] = thead_clk_light_gate_shared("clkgen_qspi0_ssi_clk", "spi_clk", ap_base + 0x200, 17, &share_cnt_qspi0_clk_en);
	clks[LIGHT_CLKGEN_QSPI1_PCLK] = thead_clk_light_gate_shared("clkgen_qspi1_pclk", "perisys_apb_pclk", ap_base + 0x200, 16, &share_cnt_qspi1_clk_en);
	clks[LIGHT_CLKGEN_QSPI1_SSI_CLK] = thead_clk_light_gate_shared("clkgen_qspi1_ssi_clk", "spi_clk", ap_base + 0x200, 16, &share_cnt_qspi0_clk_en);
	clks[LIGHT_CLKGEN_SPI_PCLK] = thead_clk_light_gate_shared("clkgen_spi_pclk", "perisys_apb_pclk", ap_base + 0x200, 15, &share_cnt_spi_clk_en);
	clks[LIGHT_CLKGEN_SPI_SSI_CLK] = thead_clk_light_gate_shared("clkgen_spi_ssi_clk", "spi_clk", ap_base + 0x200, 15, &share_cnt_spi_clk_en);
	clks[LIGHT_CLKGEN_GPIO0_PCLK] = thead_clk_light_gate_shared("clkgen_gpio0_pclk", "perisys_apb_pclk", ap_base + 0x200, 8, &share_cnt_gpio0_clk_en);
	clks[LIGHT_CLKGEN_GPIO0_DBCLK] = thead_clk_light_gate_shared("clkgen_gpio0_dbclk", "gpio_dbclk", ap_base + 0x200, 8, &share_cnt_gpio0_clk_en);
	clks[LIGHT_CLKGEN_GPIO1_PCLK] = thead_clk_light_gate_shared("clkgen_gpio1_pclk", "perisys_apb_pclk", ap_base + 0x200, 7, &share_cnt_gpio1_clk_en);
	clks[LIGHT_CLKGEN_GPIO1_DBCLK] = thead_clk_light_gate_shared("clkgen_gpio1_dbclk", "gpio_dbclk", ap_base + 0x200, 7, &share_cnt_gpio1_clk_en);
	clks[LIGHT_CLKGEN_GPIO2_PCLK] = thead_clk_light_gate_shared("clkgen_gpio2_pclk", "perisys_apb_pclk", ap_base + 0x200, 6, &share_cnt_gpio2_clk_en);
	clks[LIGHT_CLKGEN_GPIO2_DBCLK] = thead_clk_light_gate_shared("clkgen_gpio2_dbclk", "gpio_dbclk", ap_base + 0x200, 6, &share_cnt_gpio2_clk_en);
	clks[LIGHT_CLKGEN_DMAC_1_ACLK] = thead_clk_light_gate_shared("clkgen_dmac_1_aclk", "dmac_1_clk", ap_base + 0x208, 12, &share_cnt_dmac_1_clk_en);
	clks[LIGHT_CLKGEN_DMAC_1_HCLK] = thead_clk_light_gate_shared("clkgen_dmac_1_hclk", "teesys_hclk", ap_base + 0x208, 12, &share_cnt_dmac_1_clk_en);
	clks[LIGHT_CLKGEN_DMAC_2_ACLK] = thead_clk_light_gate_shared("clkgen_dmac_2_aclk", "dmac_2_clk", ap_base + 0x208, 11, &share_cnt_dmac_2_clk_en);
	clks[LIGHT_CLKGEN_DMAC_2_HCLK] = thead_clk_light_gate_shared("clkgen_dmac_2_hclk", "teesys_hclk", ap_base + 0x208, 11, &share_cnt_dmac_2_clk_en);
	clks[LIGHT_CLKGEN_DMAC_3_ACLK] = thead_clk_light_gate_shared("clkgen_dmac_3_aclk", "dmac_3_clk", ap_base + 0x208, 10, &share_cnt_dmac_3_clk_en);
	clks[LIGHT_CLKGEN_DMAC_3_HCLK] = thead_clk_light_gate_shared("clkgen_dmac_3_hclk", "teesys_hclk", ap_base + 0x208, 10, &share_cnt_dmac_3_clk_en);

	clk_data.clks = clks;
	clk_data.clk_num = ARRAY_SIZE(clks);
	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);

	if (ret < 0) {
		dev_err(dev, "failed to register clks for light\n");
		goto unregister_clks;
	}

	/* HW defalut */
	clk_set_parent(clks[LIGHT_C910_CCLK], clks[LIGHT_CPU_PLL1_FOUTPOSTDIV]);

	return 0;

unregister_clks:
	thead_unregister_clocks(clks, ARRAY_SIZE(clks));
	return ret;
}

static const struct of_device_id light_clk_of_match[] = {
	{ .compatible = "thead,light-mpw-clk" },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, light_clk_of_match);

static struct platform_driver light_clk_driver = {
	.probe = light_clocks_probe,
	.driver = {
		.name = "light-mpw-clk",
		.of_match_table = of_match_ptr(light_clk_of_match),
	},
};

module_platform_driver(light_clk_driver);
MODULE_AUTHOR("fugang.duan <duanfugang.dfg@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead Light MPW clock driver");
MODULE_LICENSE("GPL v2");
