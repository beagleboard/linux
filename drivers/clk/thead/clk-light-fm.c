/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#include <dt-bindings/clock/light-fm-ap-clock.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include "clk.h"

static struct clk *clks[CLK_END];
static struct clk_onecell_data clk_data;

/* Light Fullmask */
static u32 share_cnt_x2h_cpusys_clk_en;
static u32 share_cnt_dmac_cpusys_clk_en;
static u32 share_cnt_timer0_clk_en;
static u32 share_cnt_timer1_clk_en;
static u32 share_cnt_axi4_cpusys2_clk_en;
static u32 share_cnt_bmu_c910_clk_en;
static u32 share_cnt_aon2cpu_a2x_clk_en;
static u32 share_cnt_chip_dbg_clk_en;
static u32 share_cnt_x2x_cpusys_clk_en;
static u32 share_cnt_cfg2tee_x2h_clk_en;
static u32 share_cnt_cpu2aon_x2h_clk_en;
static u32 share_cnt_cpu2vp_x2p_clk_en;
static u32 share_cnt_npu_core_clk_en;
static u32 share_cnt_cpu2peri_x2h_clk_en;
static u32 share_cnt_cpu2vi_x2h_clk_en;
static u32 share_cnt_vpsys_axi_aclk_en;
static u32 share_cnt_gmac1_clk_en;
static u32 share_cnt_gmac0_clk_en;
static u32 share_cnt_perisys_apb3_hclk_en;
static u32 share_cnt_qspi0_clk_en;
static u32 share_cnt_gmac_axi_clk_en;
static u32 share_cnt_gpio0_clk_en;
static u32 share_cnt_gpio1_clk_en;
static u32 share_cnt_pwm_clk_en;
static u32 share_cnt_spi_clk_en;
static u32 share_cnt_uart0_clk_en;
static u32 share_cnt_uart2_clk_en;
static u32 share_cnt_i2c2_clk_en;
static u32 share_cnt_peri_i2s_clk_en;
static u32 share_cnt_qspi1_clk_en;
static u32 share_cnt_uart1_clk_en;
static u32 share_cnt_uart3_clk_en;
static u32 share_cnt_uart4_clk_en;
static u32 share_cnt_uart5_clk_en;
static u32 share_cnt_i2c0_clk_en;
static u32 share_cnt_i2c1_clk_en;
static u32 share_cnt_i2c4_clk_en;
static u32 share_cnt_i2c5_clk_en;
static u32 share_cnt_gpio2_clk_en;
static u32 share_cnt_gpio3_clk_en;
static u32 share_cnt_vosys_axi_aclk_en;

/* Light Fullmask PLL Bypass */
static const char * const cpu_pll0_bypass_sels[] = {"cpu_pll0_foutpostdiv", "osc_24m", };
static const char * const cpu_pll1_bypass_sels[] = {"cpu_pll1_foutpostdiv", "osc_24m", };
static const char * const gmac_pll_bypass_sels[] = {"gmac_pll_foutpostdiv", "osc_24m", };
static const char * const video_pll_bypass_sels[] = {"video_pll_foutpostdiv", "osc_24m", };
static const char * const tee_pll_bypass_sels[] = {"tee_pll_foutpostdiv", "osc_24m"};
static const char * const dpu0_pll_bypass_sels[] = {"dpu0_pll_foutpostdiv", "osc_24m"};
static const char * const dpu1_pll_bypass_sels[] = {"dpu1_pll_foutpostdiv", "osc_24m"};

/* light fullmask mux */
static const char * const ahb2_cpusys_hclk_sels[] = {"ahb2_cpusys_hclk_out_div", "osc_24m"};
static const char * const c910_cclk_i0_sels[] = {"cpu_pll0_foutpostdiv", "osc_24m"};
static const char * const c910_cclk_sels[] = {"c910_cclk_i0", "cpu_pll1_foutpostdiv"};
static const char * const cfg_axi_aclk_sels[] = {"cfg_axi_aclk_out_div", "osc_24m"};
static const char * const teesys_hclk_sels[] = {"teesys_i1_hclk", "teesys_i0_hclk"};
static const char * const perisys_ahb_hclk_sels[] = {"perisys_ahb_hclk_out_div", "osc_24m"};
static const char * const clk_out_1_sels[] = {"osc_24m", "clk_out_1_out_div"};
static const char * const clk_out_2_sels[] = {"osc_24m", "clk_out_2_out_div"};
static const char * const clk_out_3_sels[] = {"osc_24m", "clk_out_3_out_div"};
static const char * const clk_out_4_sels[] = {"osc_24m", "clk_out_4_out_div"};
static const char * const peri_i2s_src_clk_sels[] = {"clkgen_peri_i2s_src_clk_0", "clkgen_peri_i2s_src_clk_1"};
static const char * const npu_cclk_sels[] = {"gmac_pll_foutpostdiv", "npu_cclk_out_div"};
static const char * const cfg_apb_pclk_sels[] = {"cfg_apb_pclk_out_div", "osc_24m"};
static const char * const uart_sclk_sels[] = {"clk_100m", "osc_24m"};

static const struct light_pll_rate_table light_cpupll_tbl[] = {
	LIGHT_PLL_RATE(2616000000U, 2616000000U, 1, 109, 0, 1, 1),
	LIGHT_PLL_RATE(2592000000U, 2592000000U, 1, 108, 0, 1, 1),
	LIGHT_PLL_RATE(2568000000U, 2568000000U, 1, 107, 0, 1, 1),
	LIGHT_PLL_RATE(2544000000U, 2544000000U, 1, 106, 0, 1, 1),
	LIGHT_PLL_RATE(2520000000U, 2520000000U, 1, 105, 0, 1, 1),
	LIGHT_PLL_RATE(2496000000U, 2496000000U, 1, 104, 0, 1, 1),
	LIGHT_PLL_RATE(2472000000U, 2472000000U, 1, 103, 0, 1, 1),
	LIGHT_PLL_RATE(2448000000U, 2448000000U, 1, 102, 0, 1, 1),
	LIGHT_PLL_RATE(2424000000U, 2424000000U, 1, 101, 0, 1, 1),
	LIGHT_PLL_RATE(2400000000U, 2400000000U, 1, 100, 0, 1, 1),
	LIGHT_PLL_RATE(2376000000U, 2376000000U, 1, 99,  0, 1, 1),
	LIGHT_PLL_RATE(2352000000U, 2352000000U, 1, 98,  0, 1, 1),
	LIGHT_PLL_RATE(2328000000U, 2328000000U, 1, 97,  0, 1, 1),
	LIGHT_PLL_RATE(2304000000U, 2304000000U, 1, 96,  0, 1, 1),
	LIGHT_PLL_RATE(2280000000U, 2280000000U, 1, 95,  0, 1, 1),
	LIGHT_PLL_RATE(2256000000U, 2256000000U, 1, 94,  0, 1, 1),
	LIGHT_PLL_RATE(2232000000U, 2232000000U, 1, 93,  0, 1, 1),
	LIGHT_PLL_RATE(2208000000U, 2208000000U, 1, 92,  0, 1, 1),
	LIGHT_PLL_RATE(2184000000U, 2184000000U, 1, 91,  0, 1, 1),
	LIGHT_PLL_RATE(2160000000U, 2160000000U, 1, 90,  0, 1, 1),
	LIGHT_PLL_RATE(2136000000U, 2136000000U, 1, 89,  0, 1, 1),
	LIGHT_PLL_RATE(2112000000U, 2112000000U, 1, 88,  0, 1, 1),
	LIGHT_PLL_RATE(2088000000U, 2088000000U, 1, 87,  0, 1, 1),
	LIGHT_PLL_RATE(2064000000U, 2064000000U, 1, 86,  0, 1, 1),
	LIGHT_PLL_RATE(2040000000U, 2040000000U, 1, 85,  0, 1, 1),
	LIGHT_PLL_RATE(2016000000U, 2016000000U, 1, 84,  0, 1, 1),
	LIGHT_PLL_RATE(1992000000U, 1992000000U, 1, 83,  0, 1, 1),
	LIGHT_PLL_RATE(1968000000U, 1968000000U, 1, 82,  0, 1, 1),
	LIGHT_PLL_RATE(1944000000U, 1944000000U, 1, 81,  0, 1, 1),
	LIGHT_PLL_RATE(1920000000U, 1920000000U, 1, 80,  0, 1, 1),
	LIGHT_PLL_RATE(1896000000U, 1896000000U, 1, 79,  0, 1, 1),
	LIGHT_PLL_RATE(1872000000U, 1872000000U, 1, 78,  0, 1, 1),
	LIGHT_PLL_RATE(1848000000U, 1848000000U, 1, 77,  0, 1, 1),
	LIGHT_PLL_RATE(1824000000U, 1824000000U, 1, 76,  0, 1, 1),
	LIGHT_PLL_RATE(1800000000U, 1800000000U, 1, 75,  0, 1, 1),
	LIGHT_PLL_RATE(1776000000U, 1776000000U, 1, 74,  0, 1, 1),
	LIGHT_PLL_RATE(1752000000U, 1752000000U, 1, 73,  0, 1, 1),
	LIGHT_PLL_RATE(1728000000U, 1728000000U, 1, 72,  0, 1, 1),
	LIGHT_PLL_RATE(1704000000U, 1704000000U, 1, 71,  0, 1, 1),
	LIGHT_PLL_RATE(1680000000U, 1680000000U, 1, 70,  0, 1, 1),
	LIGHT_PLL_RATE(1656000000U, 1656000000U, 1, 69,  0, 1, 1),
	LIGHT_PLL_RATE(1632000000U, 1632000000U, 1, 68,  0, 1, 1),
	LIGHT_PLL_RATE(1608000000U, 1608000000U, 1, 67,  0, 1, 1),
	LIGHT_PLL_RATE(1584000000U, 1584000000U, 1, 66,  0, 1, 1),
	LIGHT_PLL_RATE(1560000000U, 1560000000U, 1, 65,  0, 1, 1),
	LIGHT_PLL_RATE(1536000000U, 1536000000U, 1, 64,  0, 1, 1),
	LIGHT_PLL_RATE(1512000000U, 1512000000U, 1, 63,  0, 1, 1),
	LIGHT_PLL_RATE(3000000000U, 1500000000U, 1, 125, 0, 2, 1),
	LIGHT_PLL_RATE(2976000000U, 1488000000U, 1, 124, 0, 2, 1),
	LIGHT_PLL_RATE(2952000000U, 1476000000U, 1, 123, 0, 2, 1),
	LIGHT_PLL_RATE(2928000000U, 1464000000U, 1, 122, 0, 2, 1),
	LIGHT_PLL_RATE(2904000000U, 1452000000U, 1, 121, 0, 2, 1),
	LIGHT_PLL_RATE(2880000000U, 1440000000U, 1, 120, 0, 2, 1),
	LIGHT_PLL_RATE(2856000000U, 1428000000U, 1, 119, 0, 2, 1),
	LIGHT_PLL_RATE(2832000000U, 1416000000U, 1, 118, 0, 2, 1),
	LIGHT_PLL_RATE(2808000000U, 1404000000U, 1, 117, 0, 2, 1),
	LIGHT_PLL_RATE(2784000000U, 1392000000U, 1, 116, 0, 2, 1),
	LIGHT_PLL_RATE(2760000000U, 1380000000U, 1, 115, 0, 2, 1),
	LIGHT_PLL_RATE(2736000000U, 1368000000U, 1, 114, 0, 2, 1),
	LIGHT_PLL_RATE(2712000000U, 1356000000U, 1, 113, 0, 2, 1),
	LIGHT_PLL_RATE(2688000000U, 1344000000U, 1, 112, 0, 2, 1),
	LIGHT_PLL_RATE(2664000000U, 1332000000U, 1, 111, 0, 2, 1),
	LIGHT_PLL_RATE(2640000000U, 1320000000U, 1, 110, 0, 2, 1),
	LIGHT_PLL_RATE(2616000000U, 1308000000U, 1, 109, 0, 2, 1),
	LIGHT_PLL_RATE(2592000000U, 1296000000U, 1, 108, 0, 2, 1),
	LIGHT_PLL_RATE(2568000000U, 1284000000U, 1, 107, 0, 2, 1),
	LIGHT_PLL_RATE(2544000000U, 1272000000U, 1, 106, 0, 2, 1),
	LIGHT_PLL_RATE(2520000000U, 1260000000U, 1, 105, 0, 2, 1),
	LIGHT_PLL_RATE(2496000000U, 1248000000U, 1, 104, 0, 2, 1),
	LIGHT_PLL_RATE(2472000000U, 1236000000U, 1, 103, 0, 2, 1),
	LIGHT_PLL_RATE(2448000000U, 1224000000U, 1, 102, 0, 2, 1),
	LIGHT_PLL_RATE(2424000000U, 1212000000U, 1, 101, 0, 2, 1),
	LIGHT_PLL_RATE(2400000000U, 1200000000U, 1, 100, 0, 2, 1),
	LIGHT_PLL_RATE(2376000000U, 1188000000U, 1, 99,  0, 2, 1),
	LIGHT_PLL_RATE(2352000000U, 1176000000U, 1, 98,  0, 2, 1),
	LIGHT_PLL_RATE(2328000000U, 1164000000U, 1, 97,  0, 2, 1),
	LIGHT_PLL_RATE(2304000000U, 1152000000U, 1, 96,  0, 2, 1),
	LIGHT_PLL_RATE(2280000000U, 1140000000U, 1, 95,  0, 2, 1),
	LIGHT_PLL_RATE(2256000000U, 1128000000U, 1, 94,  0, 2, 1),
	LIGHT_PLL_RATE(2232000000U, 1116000000U, 1, 93,  0, 2, 1),
	LIGHT_PLL_RATE(2208000000U, 1104000000U, 1, 92,  0, 2, 1),
	LIGHT_PLL_RATE(2184000000U, 1092000000U, 1, 91,  0, 2, 1),
	LIGHT_PLL_RATE(2160000000U, 1080000000U, 1, 90,  0, 2, 1),
	LIGHT_PLL_RATE(2136000000U, 1068000000U, 1, 89,  0, 2, 1),
	LIGHT_PLL_RATE(2112000000U, 1056000000U, 1, 88,  0, 2, 1),
	LIGHT_PLL_RATE(2088000000U, 1044000000U, 1, 87,  0, 2, 1),
	LIGHT_PLL_RATE(2064000000U, 1032000000U, 1, 86,  0, 2, 1),
	LIGHT_PLL_RATE(2040000000U, 1020000000U, 1, 85,  0, 2, 1),
	LIGHT_PLL_RATE(2016000000U, 1008000000U, 1, 84,  0, 2, 1),
	LIGHT_PLL_RATE(3000000000U, 1000000000U, 1, 125, 0, 3, 1),
	LIGHT_PLL_RATE(2976000000U, 992000000U,  1, 124, 0, 3, 1),
	LIGHT_PLL_RATE(2952000000U, 984000000U,  1, 123, 0, 3, 1),
	LIGHT_PLL_RATE(2928000000U, 976000000U,  1, 122, 0, 3, 1),
	LIGHT_PLL_RATE(2904000000U, 968000000U,  1, 121, 0, 3, 1),
	LIGHT_PLL_RATE(2880000000U, 960000000U,  1, 120, 0, 3, 1),
	LIGHT_PLL_RATE(2856000000U, 952000000U,  1, 119, 0, 3, 1),
	LIGHT_PLL_RATE(2832000000U, 944000000U,  1, 118, 0, 3, 1),
	LIGHT_PLL_RATE(2808000000U, 936000000U,  1, 117, 0, 3, 1),
	LIGHT_PLL_RATE(2784000000U, 928000000U,  1, 116, 0, 3, 1),
	LIGHT_PLL_RATE(2760000000U, 920000000U,  1, 115, 0, 3, 1),
	LIGHT_PLL_RATE(2736000000U, 912000000U,  1, 114, 0, 3, 1),
	LIGHT_PLL_RATE(2712000000U, 904000000U,  1, 113, 0, 3, 1),
	LIGHT_PLL_RATE(1800000000U, 900000000U,  1, 75,  0, 2, 1),
	LIGHT_PLL_RATE(2688000000U, 896000000U,  1, 112, 0, 3, 1),
	LIGHT_PLL_RATE(2664000000U, 888000000U,  1, 111, 0, 3, 1),
	LIGHT_PLL_RATE(2640000000U, 880000000U,  1, 110, 0, 3, 1),
	LIGHT_PLL_RATE(2616000000U, 872000000U,  1, 109, 0, 3, 1),
	LIGHT_PLL_RATE(2592000000U, 864000000U,  1, 108, 0, 3, 1),
	LIGHT_PLL_RATE(2568000000U, 856000000U,  1, 107, 0, 3, 1),
	LIGHT_PLL_RATE(2544000000U, 848000000U,  1, 106, 0, 3, 1),
	LIGHT_PLL_RATE(2520000000U, 840000000U,  1, 105, 0, 3, 1),
	LIGHT_PLL_RATE(2496000000U, 832000000U,  1, 104, 0, 3, 1),
	LIGHT_PLL_RATE(2472000000U, 824000000U,  1, 103, 0, 3, 1),
	LIGHT_PLL_RATE(2448000000U, 816000000U,  1, 102, 0, 3, 1),
	LIGHT_PLL_RATE(2424000000U, 808000000U,  1, 101, 0, 3, 1),
	LIGHT_PLL_RATE(2400000000U, 800000000U,  1, 100, 0, 3, 1),
	LIGHT_PLL_RATE(2376000000U, 792000000U,  1, 99,  0, 3, 1),
	LIGHT_PLL_RATE(2352000000U, 784000000U,  1, 98,  0, 3, 1),
	LIGHT_PLL_RATE(2328000000U, 776000000U,  1, 97,  0, 3, 1),
	LIGHT_PLL_RATE(2304000000U, 768000000U,  1, 96,  0, 3, 1),
	LIGHT_PLL_RATE(2280000000U, 760000000U,  1, 95,  0, 3, 1),
	LIGHT_PLL_RATE(2256000000U, 752000000U,  1, 94,  0, 3, 1),
	LIGHT_PLL_RATE(2232000000U, 744000000U,  1, 93,  0, 3, 1),
	LIGHT_PLL_RATE(2208000000U, 736000000U,  1, 92,  0, 3, 1),
	LIGHT_PLL_RATE(2184000000U, 728000000U,  1, 91,  0, 3, 1),
	LIGHT_PLL_RATE(2160000000U, 720000000U,  1, 90,  0, 3, 1),
	LIGHT_PLL_RATE(2136000000U, 712000000U,  1, 89,  0, 3, 1),
	LIGHT_PLL_RATE(2808000000U, 702000000U,  1, 117, 0, 4, 1),
	LIGHT_PLL_RATE(2760000000U, 690000000U,  1, 115, 0, 4, 1),
	LIGHT_PLL_RATE(2712000000U, 678000000U,  1, 113, 0, 4, 1),
	LIGHT_PLL_RATE(2664000000U, 666000000U,  1, 111, 0, 4, 1),
	LIGHT_PLL_RATE(2616000000U, 654000000U,  1, 109, 0, 4, 1),
	LIGHT_PLL_RATE(2568000000U, 642000000U,  1, 107, 0, 4, 1),
	LIGHT_PLL_RATE(2520000000U, 630000000U,  1, 105, 0, 4, 1),
	LIGHT_PLL_RATE(2472000000U, 618000000U,  1, 103, 0, 4, 1),
	LIGHT_PLL_RATE(2424000000U, 606000000U,  1, 101, 0, 4, 1),
	LIGHT_PLL_RATE(3000000000U, 600000000U,  1, 125, 0, 5, 1),
	LIGHT_PLL_RATE(2952000000U, 590400000U,  1, 123, 0, 5, 1),
	LIGHT_PLL_RATE(2904000000U, 580800000U,  1, 121, 0, 5, 1),
	LIGHT_PLL_RATE(2856000000U, 571200000U,  1, 119, 0, 5, 1),
	LIGHT_PLL_RATE(2808000000U, 561600000U,  1, 117, 0, 5, 1),
	LIGHT_PLL_RATE(2760000000U, 552000000U,  1, 115, 0, 5, 1),
	LIGHT_PLL_RATE(2712000000U, 542400000U,  1, 113, 0, 5, 1),
	LIGHT_PLL_RATE(2664000000U, 532800000U,  1, 111, 0, 5, 1),
	LIGHT_PLL_RATE(2616000000U, 523200000U,  1, 109, 0, 5, 1),
	LIGHT_PLL_RATE(2568000000U, 513600000U,  1, 107, 0, 5, 1),
	LIGHT_PLL_RATE(2520000000U, 504000000U,  1, 105, 0, 5, 1),
	LIGHT_PLL_RATE(3000000000U, 500000000U,  1, 125, 0, 6, 1),
	LIGHT_PLL_RATE(2952000000U, 492000000U,  1, 123, 0, 6, 1),
	LIGHT_PLL_RATE(2904000000U, 484000000U,  1, 121, 0, 6, 1),
	LIGHT_PLL_RATE(2856000000U, 476000000U,  1, 119, 0, 6, 1),
	LIGHT_PLL_RATE(2808000000U, 468000000U,  1, 117, 0, 6, 1),
	LIGHT_PLL_RATE(2760000000U, 460000000U,  1, 115, 0, 6, 1),
	LIGHT_PLL_RATE(2712000000U, 452000000U,  1, 113, 0, 6, 1),
	LIGHT_PLL_RATE(2664000000U, 444000000U,  1, 111, 0, 6, 1),
	LIGHT_PLL_RATE(2616000000U, 436000000U,  1, 109, 0, 6, 1),
	LIGHT_PLL_RATE(2568000000U, 428000000U,  1, 107, 0, 6, 1),
	LIGHT_PLL_RATE(2520000000U, 420000000U,  1, 105, 0, 6, 1),
	LIGHT_PLL_RATE(2472000000U, 412000000U,  1, 103, 0, 6, 1),
	LIGHT_PLL_RATE(2400000000U, 400000000U,  1, 100, 0, 3, 2),
	LIGHT_PLL_RATE(2352000000U, 392000000U,  1, 98,  0, 3, 2),
	LIGHT_PLL_RATE(2304000000U, 384000000U,  1, 96,  0, 3, 2),
	LIGHT_PLL_RATE(2256000000U, 376000000U,  1, 94,  0, 3, 2),
	LIGHT_PLL_RATE(2208000000U, 368000000U,  1, 92,  0, 3, 2),
	LIGHT_PLL_RATE(2160000000U, 360000000U,  1, 90,  0, 3, 2),
	LIGHT_PLL_RATE(2112000000U, 352000000U,  1, 88,  0, 3, 2),
	LIGHT_PLL_RATE(2064000000U, 344000000U,  1, 86,  0, 3, 2),
	LIGHT_PLL_RATE(2016000000U, 336000000U,  1, 84,  0, 3, 2),
	LIGHT_PLL_RATE(1968000000U, 328000000U,  1, 82,  0, 3, 2),
	LIGHT_PLL_RATE(1920000000U, 320000000U,  1, 80,  0, 3, 2),
	LIGHT_PLL_RATE(1872000000U, 312000000U,  1, 78,  0, 3, 2),
	LIGHT_PLL_RATE(1824000000U, 304000000U,  1, 76,  0, 3, 2),
	LIGHT_PLL_RATE(3000000000U, 300000000U,  1, 125, 0, 5, 2),
	LIGHT_PLL_RATE(2880000000U, 288000000U,  1, 120, 0, 5, 2),
	LIGHT_PLL_RATE(2760000000U, 276000000U,  1, 115, 0, 5, 2),
	LIGHT_PLL_RATE(2640000000U, 264000000U,  1, 110, 0, 5, 2),
	LIGHT_PLL_RATE(2520000000U, 252000000U,  1, 105, 0, 5, 2),
	LIGHT_PLL_RATE(2400000000U, 240000000U,  1, 100, 0, 5, 2),
	LIGHT_PLL_RATE(2280000000U, 228000000U,  1, 95,  0, 5, 2),
	LIGHT_PLL_RATE(2160000000U, 216000000U,  1, 90,  0, 5, 2),
	LIGHT_PLL_RATE(2040000000U, 204000000U,  1, 85,  0, 5, 2),
	LIGHT_PLL_RATE(3000000000U, 200000000U,  1, 125, 0, 5, 3),
	LIGHT_PLL_RATE(2880000000U, 192000000U,  1, 120, 0, 5, 3),
	LIGHT_PLL_RATE(2760000000U, 184000000U,  1, 115, 0, 5, 3),
	LIGHT_PLL_RATE(2640000000U, 176000000U,  1, 110, 0, 5, 3),
	LIGHT_PLL_RATE(2520000000U, 168000000U,  1, 105, 0, 5, 3),
	LIGHT_PLL_RATE(2400000000U, 160000000U,  1, 100, 0, 5, 3),
	LIGHT_PLL_RATE(2280000000U, 152000000U,  1, 95,  0, 5, 3),
	LIGHT_PLL_RATE(2160000000U, 144000000U,  1, 90,  0, 5, 3),
	LIGHT_PLL_RATE(2040000000U, 136000000U,  1, 85,  0, 5, 3),
	LIGHT_PLL_RATE(1920000000U, 128000000U,  1, 80,  0, 5, 3),
	LIGHT_PLL_RATE(3000000000U, 125000000U,  1, 125, 0, 6, 4),
	LIGHT_PLL_RATE(2760000000U, 115000000U,  1, 115, 0, 6, 4),
	LIGHT_PLL_RATE(2520000000U, 105000000U,  1, 105, 0, 6, 4),
	LIGHT_PLL_RATE(2280000000U, 95000000U,   1, 95,  0, 6, 4),
	LIGHT_PLL_RATE(2040000000U, 85000000U,   1, 85,  0, 6, 4),
	LIGHT_PLL_RATE(1800000000U, 75000000U,   1, 75,  0, 6, 4),
	LIGHT_PLL_RATE(1560000000U, 65000000U,   1, 65,  0, 6, 4),
	LIGHT_PLL_RATE(1320000000U, 55000000U,   1, 55,  0, 6, 4),
};

static const struct light_pll_rate_table light_dpupll_tbl[] = {
	LIGHT_PLL_RATE(2376000000U, 1188000000U, 1, 99, 0, 2, 1),
	LIGHT_PLL_RATE(1980000000U, 990000000U, 2, 165, 0, 2, 1),
	LIGHT_PLL_RATE(2970000000U, 742500000U, 4, 495, 0, 4, 1),
	LIGHT_PLL_RATE(2304000000U, 1152000000U, 1, 96, 0, 2, 1),
	LIGHT_PLL_RATE(1512000000U, 504000000U, 1, 63, 0, 3, 1),
	LIGHT_PLL_RATE(1512000000U, 503500000U, 1, 63, 0, 3, 1),
	LIGHT_PLL_RATE(2898000000U, 483000000U, 4, 483, 0, 6, 1),
	LIGHT_PLL_RATE(2592000000U, 648000000U, 1, 108, 0, 4, 1),
	LIGHT_PLL_RATE(2772000000U, 924000000U, 2, 231, 0, 3, 1),
	LIGHT_PLL_RATE(2856000000U, 476000000U, 1, 119, 0, 6, 1),
	LIGHT_PLL_RATE(2130000000U, 355000000U, 4, 355, 0, 6, 1),
	LIGHT_PLL_RATE(3192000000U, 456000000U, 1, 133, 0, 7, 1),
	LIGHT_PLL_RATE(2730000000U, 390000000U, 4, 455, 0, 7, 1),
	LIGHT_PLL_RATE(1680000000U, 240000000U, 1, 70, 0, 7, 1),
	LIGHT_PLL_RATE(2832000000U, 708000000U, 1, 118, 0, 4, 1),
	LIGHT_PLL_RATE(1026000000U, 342000000U, 4, 171, 0, 3, 1),
	LIGHT_PLL_RATE(1260000000U, 630000000U, 4, 210, 0, 2, 1),
};

static struct light_pll_clk light_cpu_pll0div = {
	.out_type = LIGHT_PLL_DIV,
	.clk_type = LIGHT_CPU_PLL0,
	.rate_table = light_cpupll_tbl,
	.rate_count = ARRAY_SIZE(light_cpupll_tbl),
};

static struct light_pll_clk light_cpu_pll1div = {
	.out_type = LIGHT_PLL_DIV,
	.clk_type = LIGHT_CPU_PLL1,
	.rate_table = light_cpupll_tbl,
	.rate_count = ARRAY_SIZE(light_cpupll_tbl),
};

static struct light_pll_clk light_dpu0_plldiv = {
	.out_type = LIGHT_PLL_DIV,
	.clk_type = LIGHT_DPU0_PLL,
	.rate_table = light_dpupll_tbl,
	.rate_count = ARRAY_SIZE(light_dpupll_tbl),
};

static struct light_pll_clk light_dpu1_plldiv = {
	.out_type = LIGHT_PLL_DIV,
	.clk_type = LIGHT_DPU1_PLL,
	.rate_table = light_dpupll_tbl,
	.rate_count = ARRAY_SIZE(light_dpupll_tbl),
};

static int light_clocks_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	void __iomem *ap_base;
	int ret;
	const bool *teesys = of_device_get_match_data(dev);

	/* Clock source */
	clks[CLK_DUMMY] = thead_clk_fixed("dummy", 0);
	clks[OSC_32K] = of_clk_get_by_name(np, "osc_32k");
	clks[OSC_24M] = of_clk_get_by_name(np, "osc_24m");
	clks[RC_24M] = of_clk_get_by_name(np, "rc_24m");

	np = dev->of_node;
	ap_base = devm_platform_ioremap_resource(pdev, 0);
	if (WARN_ON(IS_ERR(ap_base))) {
		ret = PTR_ERR(ap_base);
		goto unregister_clks;
	}

	/* Light Fullmask AP PLL clocks */
	clks[CPU_PLL0_FOUTPOSTDIV] = thead_light_pll("cpu_pll0_foutpostdiv", "osc_24m", ap_base, &light_cpu_pll0div);
	clks[CPU_PLL1_FOUTPOSTDIV] = thead_light_pll("cpu_pll1_foutpostdiv", "osc_24m", ap_base, &light_cpu_pll1div);

	clks[DPU0_PLL_FOUTPOSTDIV] = thead_light_pll("dpu0_pll_foutpostdiv", "osc_24m", ap_base, &light_dpu0_plldiv);
	clks[DPU1_PLL_FOUTPOSTDIV] = thead_light_pll("dpu1_pll_foutpostdiv", "osc_24m", ap_base, &light_dpu1_plldiv);

	/* Light Fullmask AP Fixed PLL */
	clks[GMAC_PLL_FOUTPOSTDIV] = thead_clk_fixed("gmac_pll_foutpostdiv", 1000000000);
	clks[VIDEO_PLL_FOUTPOSTDIV] = thead_clk_fixed("video_pll_foutpostdiv", 792000000);
	clks[VIDEO_PLL_FOUTVCO] = thead_clk_fixed("video_pll_foutvco", 2376000000);
	clks[TEE_PLL_FOUTPOSTDIV] = thead_clk_fixed("tee_pll_foutpostdiv", 792000000);
	clks[CLKGEN_PERI_I2S_SRC_CLK_0] = thead_clk_fixed("clkgen_peri_i2s_src_clk_0", 294912000);	//from audio_pll_foutpostdiv
	clks[CLKGEN_PERI_I2S_SRC_CLK_1] = thead_clk_fixed("clkgen_peri_i2s_src_clk_1", 135475200);	//from sys_pll_foutpostdiv
	clks[CLKGEN_C910_BUS_CLK_NO_ICG] = thead_clk_fixed("clkgen_c910_bus_clk_no_icg", 750000000);
	clks[AONSYS_BUS_CLK] = thead_clk_fixed("aonsys_hclk", 101606400);	//from sys_pll, maybe change ?

	/* Light Fullmask AP MUX */
	clks[CPU_PLL0_BYPASS] = thead_light_clk_mux_flags("cpu_pll0_bypass", ap_base + 0x4, 30, 1, cpu_pll0_bypass_sels, ARRAY_SIZE(cpu_pll0_bypass_sels), CLK_SET_RATE_PARENT);
	clks[CPU_PLL1_BYPASS] = thead_light_clk_mux_flags("cpu_pll1_bypass", ap_base + 0x14, 30, 1, cpu_pll1_bypass_sels, ARRAY_SIZE(cpu_pll1_bypass_sels), CLK_SET_RATE_PARENT);
	clks[GMAC_PLL_BYPASS] = thead_light_clk_mux_flags("gmac_pll_bypass", ap_base + 0x24, 30, 1, gmac_pll_bypass_sels, ARRAY_SIZE(gmac_pll_bypass_sels), CLK_SET_RATE_PARENT);
	clks[VIDEO_PLL_BYPASS] = thead_light_clk_mux_flags("video_pll_bypass", ap_base + 0x34, 30, 1, video_pll_bypass_sels, ARRAY_SIZE(video_pll_bypass_sels), CLK_SET_RATE_PARENT);
	clks[TEE_PLL_BYPASS] = thead_light_clk_mux_flags("tee_pll_bypass", ap_base + 0x64, 30, 1, tee_pll_bypass_sels, ARRAY_SIZE(tee_pll_bypass_sels), CLK_SET_RATE_PARENT);
	clks[DPU0_PLL_BYPASS] = thead_light_clk_mux_flags("dpu0_pll_bypass", ap_base + 0x44, 30, 1, dpu0_pll_bypass_sels, ARRAY_SIZE(dpu0_pll_bypass_sels), CLK_SET_RATE_PARENT);
	clks[DPU1_PLL_BYPASS] = thead_light_clk_mux_flags("dpu1_pll_bypass", ap_base + 0x54, 30, 1, dpu1_pll_bypass_sels, ARRAY_SIZE(dpu1_pll_bypass_sels), CLK_SET_RATE_PARENT);

	clks[AHB2_CPUSYS_HCLK] = thead_light_clk_mux_flags("ahb2_cpusys_hclk", ap_base + 0x120, 5, 1, ahb2_cpusys_hclk_sels, ARRAY_SIZE(ahb2_cpusys_hclk_sels), CLK_SET_RATE_PARENT);
	clks[C910_CCLK_I0] = thead_light_clk_mux_flags("c910_cclk_i0", ap_base + 0x100, 1, 1, c910_cclk_i0_sels, ARRAY_SIZE(c910_cclk_i0_sels), CLK_SET_RATE_PARENT);
	clks[C910_CCLK] = thead_light_clk_mux_flags("c910_cclk", ap_base + 0x100, 0, 1, c910_cclk_sels, ARRAY_SIZE(c910_cclk_sels), CLK_SET_RATE_PARENT);
	clks[CFG_AXI_ACLK] = thead_light_clk_mux_flags("cfg_axi_aclk", ap_base + 0x138, 5, 1, cfg_axi_aclk_sels, ARRAY_SIZE(cfg_axi_aclk_sels), CLK_SET_RATE_PARENT);

	if (teesys)
		clks[TEESYS_HCLK] = thead_light_clk_mux_flags("teesys_hclk", ap_base + 0x1cc, 13, 1, teesys_hclk_sels, ARRAY_SIZE(teesys_hclk_sels), CLK_SET_RATE_PARENT); //just for teesys!!!

	clks[PERISYS_AHB_HCLK] = thead_light_clk_mux_flags("perisys_ahb_hclk", ap_base + 0x140, 5, 1, perisys_ahb_hclk_sels, ARRAY_SIZE(perisys_ahb_hclk_sels), CLK_SET_RATE_PARENT);
	clks[CLK_OUT_1] = thead_light_clk_mux_flags("clk_out_1", ap_base + 0x1b4, 4, 1, clk_out_1_sels, ARRAY_SIZE(clk_out_1_sels), CLK_SET_RATE_PARENT);
	clks[CLK_OUT_2] = thead_light_clk_mux_flags("clk_out_2", ap_base + 0x1b8, 4, 1, clk_out_2_sels, ARRAY_SIZE(clk_out_2_sels), CLK_SET_RATE_PARENT);
	clks[CLK_OUT_3] = thead_light_clk_mux_flags("clk_out_3", ap_base + 0x1bc, 4, 1, clk_out_3_sels, ARRAY_SIZE(clk_out_3_sels), CLK_SET_RATE_PARENT);
	clks[CLK_OUT_4] = thead_light_clk_mux_flags("clk_out_4", ap_base + 0x1c0, 4, 1, clk_out_4_sels, ARRAY_SIZE(clk_out_4_sels), CLK_SET_RATE_PARENT);
	clks[PERI_I2S_SRC_CLK] = thead_light_clk_mux_flags("peri_i2s_src_clk", ap_base + 0x1f0, 0, 1, peri_i2s_src_clk_sels, ARRAY_SIZE(peri_i2s_src_clk_sels), CLK_SET_RATE_PARENT);
	clks[NPU_CCLK] = thead_light_clk_mux_flags("npu_cclk", ap_base + 0x1c8, 6, 1, npu_cclk_sels, ARRAY_SIZE(npu_cclk_sels), CLK_SET_RATE_PARENT);
	clks[CFG_APB_PCLK] = thead_light_clk_mux_flags("cfg_apb_pclk", ap_base + 0x1c4, 7, 1, cfg_apb_pclk_sels, ARRAY_SIZE(cfg_apb_pclk_sels), CLK_SET_RATE_PARENT);
	clks[UART_SCLK] = thead_light_clk_mux_flags("uart_sclk", ap_base + 0x210, 0, 1, uart_sclk_sels, ARRAY_SIZE(uart_sclk_sels), CLK_SET_RATE_PARENT);

	/* Light Fullmask AP Divider */
	clks[AHB2_CPUSYS_HCLK_OUT_DIV] = thead_clk_light_divider("ahb2_cpusys_hclk_out_div", "gmac_pll_fout1ph0", ap_base + 0x120, 0, 3, 4, MUX_TYPE_DIV, 2, 7);
	clks[APB3_CPUSYS_PCLK] = thead_clk_light_divider("apb3_cpusys_pclk", "ahb2_cpusys_hclk", ap_base + 0x130, 0, 3, 3, MUX_TYPE_CDE, 1, 7);
	clks[AXI4_CPUSYS2_ACLK] = thead_clk_light_divider("axi4_cpusys2_aclk", "gmac_pll_foutpostdiv", ap_base + 0x134, 0, 3, 4, MUX_TYPE_DIV, 2, 7);
	clks[CFG_AXI_ACLK_OUT_DIV] = thead_clk_light_divider("cfg_axi_aclk_out_div", "video_pll_foutpostdiv", ap_base + 0x138, 0, 4, 4, MUX_TYPE_DIV, 2, 15);

	if (teesys) {
		clks[TEESYS_I0_HCLK] = thead_clk_light_divider("teesys_i0_hclk", "tee_pll_foutpostdiv", ap_base + 0x1cc, 0, 4, 4, MUX_TYPE_DIV, 2, 15); //just for teesys!!!
		clks[TEESYS_I1_HCLK] = thead_clk_light_divider("teesys_i1_hclk", "video_pll_foutpostdiv", ap_base + 0x1cc, 8, 4, 12, MUX_TYPE_DIV, 2, 15); //just for teesys!!!
	}

	clks[PERISYS_AHB_HCLK_OUT_DIV] = thead_clk_light_divider("perisys_ahb_hclk_out_div", "gmac_pll_fout1ph0", ap_base + 0x140, 0, 4, 4, MUX_TYPE_DIV, 2, 7);
	clks[PERISYS_APB_PCLK] = thead_clk_light_divider("perisys_apb_pclk", "perisys_ahb_hclk", ap_base + 0x150, 0, 3, 3, MUX_TYPE_CDE, 3, 7);
	clks[PERI2SYS_APB_PCLK] = thead_clk_light_divider("peri2sys_apb_pclk", "gmac_pll_fout4", ap_base + 0x150, 4, 3, 8, MUX_TYPE_DIV, 2, 7);
	clks[CLK_OUT_1_OUT_DIV] = thead_clk_light_divider("clk_out_1_out_div", "osc_24m", ap_base + 0x1b4, 0, 3, 3, MUX_TYPE_DIV, 2, 4);
	clks[CLK_OUT_2_OUT_DIV] = thead_clk_light_divider("clk_out_2_out_div", "osc_24m", ap_base + 0x1b8, 0, 3, 3, MUX_TYPE_DIV, 2, 4);
	clks[CLK_OUT_3_OUT_DIV] = thead_clk_light_divider("clk_out_3_out_div", "osc_24m", ap_base + 0x1bc, 0, 3, 3, MUX_TYPE_DIV, 2, 4);
	clks[CLK_OUT_4_OUT_DIV] = thead_clk_light_divider("clk_out_4_out_div", "osc_24m", ap_base + 0x1c0, 0, 3, 3, MUX_TYPE_DIV, 2, 4);
	clks[VOSYS_ACLK_M] = thead_clk_light_divider("vosys_aclk_m", "video_pll_foutvco", ap_base + 0x1dc, 0, 4, 4, MUX_TYPE_DIV, 3, 15);
	clks[NPU_CCLK_OUT_DIV] = thead_clk_light_divider("npu_cclk_out_div", "video_pll_foutvco", ap_base + 0x1c8, 0, 3, 3, MUX_TYPE_DIV, 3, 7);
	clks[CFG_APB_PCLK_OUT_DIV] = thead_clk_light_divider("cfg_apb_pclk_out_div", "gmac_pll_foutpostdiv", ap_base + 0x1c4, 0, 4, 4, MUX_TYPE_DIV, 4, 15);
	clks[VISYS_ACLK_M] = thead_clk_light_divider("visys_aclk_m", "video_pll_foutvco", ap_base + 0x1d0, 16, 4, 20, MUX_TYPE_DIV, 3, 15);
	clks[VISYS_AHB_HCLK] = thead_clk_light_divider("visys_ahb_hclk", "video_pll_foutvco", ap_base + 0x1d0, 0, 4, 4, MUX_TYPE_DIV, 6, 15);
	clks[VPSYS_APB_PCLK] = thead_clk_light_divider("vpsys_apb_pclk", "gmac_pll_fout1ph0", ap_base + 0x1e0, 0, 3, 4, MUX_TYPE_DIV, 2, 7);
	clks[VPSYS_AXI_ACLK] = thead_clk_light_divider("vpsys_axi_aclk", "video_pll_foutvco", ap_base + 0x1e0, 8, 4, 12, MUX_TYPE_DIV, 3, 15);
	clks[VENC_CCLK] = thead_clk_light_divider("venc_cclk", "gmac_pll_foutpostdiv", ap_base + 0x1e4, 0, 3, 4, MUX_TYPE_DIV, 2, 7);
	clks[DPU0_PLL_DIV_CLK] = thead_clk_light_divider("dpu0_pll_div_clk", "dpu0_pll_foutpostdiv", ap_base + 0x1e8, 0, 8, 8, MUX_TYPE_DIV, 2, 214);
	clks[DPU1_PLL_DIV_CLK] = thead_clk_light_divider("dpu1_pll_div_clk", "dpu1_pll_foutpostdiv", ap_base + 0x1ec, 0, 8, 8, MUX_TYPE_DIV, 2, 214);

	/* Light Fullmask PLL FOUT */
	clks[GMAC_PLL_FOUT1PH0] = thead_light_clk_fixed_factor("gmac_pll_fout1ph0", "gmac_pll_bypass", 1, 2);
	clks[GMAC_PLL_FOUT4] = thead_light_clk_fixed_factor("gmac_pll_fout4", "gmac_pll_bypass", 1, 8);
	clks[VIDEO_PLL_FOUT1PH0] = thead_light_clk_fixed_factor("video_pll_fout1ph0", "video_pll_bybass", 1, 2);
	clks[VIDEO_PLL_FOUT4] = thead_light_clk_fixed_factor("video_pll_fout4", "video_pll_bypass", 1, 8);
	clks[TEE_PLL_FOUT4] = thead_light_clk_fixed_factor("tee_pll_fout4", "tee_pll_bypass", 1, 8);
	clks[CPU_PLL0_FOUT4] = thead_light_clk_fixed_factor("cpu_pll0_fout4", "cpu_pll0_bypass", 1, 8);
	clks[CPU_PLL1_FOUT4] = thead_light_clk_fixed_factor("cpu_pll1_fout4", "cpu_pll1_bypass", 1, 8);
	clks[DPU0_PLL_FOUT4] = thead_light_clk_fixed_factor("dpu0_pll_fout4", "dpu0_pll_bypass", 1, 8);
	clks[DPU1_PLL_FOUT4] = thead_light_clk_fixed_factor("dpu1_pll_fout4", "dpu1_pll_bypass", 1, 8);

	/* Light Fullmask Fixed Factor */
	clks[C910_OSC_CLK] = thead_light_clk_fixed_factor("c910_osc_clk", "osc_24m", 1, 1);
	clks[QSPI_SSI_CLK] = thead_light_clk_fixed_factor("qspi_ssi_clk", "video_pll_foutpostdiv", 1, 1);		/* Note: no mux to select, use default value */
	clks[QSPI0_SSI_CLK] = thead_light_clk_fixed_factor("qspi0_ssi_clk", "qspi_ssi_clk", 1, 1);
	clks[QSPI1_SSI_CLK] = thead_light_clk_fixed_factor("qspi1_ssi_clk", "video_pll_fout1ph0", 1, 1);
	clks[SPI_SSI_CLK] = thead_light_clk_fixed_factor("spi_ssi_clk", "video_pll_fout1ph0", 1, 1);
	clks[EMMC_SDIO_REF_CLK] = thead_light_clk_fixed_factor("emmc_sdio_ref_clk", "video_pll_foutpostdiv", 1, 1);	/* Note: no mux to select, use default value */
	clks[PWM_CCLK] = thead_light_clk_fixed_factor("pwm_cclk", "osc_24m", 1, 1);
	clks[CHIP_DBG_CCLK] = thead_light_clk_fixed_factor("chip_dbg_cclk", "osc_24m", 1, 1);
	clks[GMAC_CCLK] = thead_light_clk_fixed_factor("gmac_cclk", "gmac_pll_fout1ph0", 1, 1);
	clks[GPIO0_DBCLK] = thead_light_clk_fixed_factor("gpio0_dbclk", "pad_rtc_clk", 1, 1);
	clks[GPIO1_DBCLK] = thead_light_clk_fixed_factor("gpio1_dbclk", "pad_rtc_clk", 1, 1);
	clks[GPIO2_DBCLK] = thead_light_clk_fixed_factor("gpio2_dbclk", "pad_rtc_clk", 1, 1);
	clks[GPIO3_DBCLK] = thead_light_clk_fixed_factor("gpio3_dbclk", "pad_rtc_clk", 1, 1);
	clks[CLK_100M] = thead_light_clk_fixed_factor("clk_100m", "gmac_pll_foutpostdiv", 1, 10);
	clks[I2C_IC_CLK] = thead_light_clk_fixed_factor("i2c_ic_clk", "clk_100m", 1, 2);
	clks[TIMER_CCLK] = thead_light_clk_fixed_factor("timer_cclk", "osc_24m", 1, 1);
	clks[AXI4_CPUSYS1_ACLK] = thead_light_clk_fixed_factor("axi4_cpusys1_aclk", "clkgen_c910_bus_clk_no_icg", 1, 1);
	clks[CPU_BUS_DFTCLK] = thead_light_clk_fixed_factor("cpu_bus_dftclk", "cpu_pll0_foutpostdiv", 1, 2);
	clks[CPU_PLL0_TEST_CLK] = thead_light_clk_fixed_factor("cpu_pll0_test_clk", "cpu_pll0_fout4", 1, 8);
	clks[CPU_PLL1_TEST_CLK] = thead_light_clk_fixed_factor("cpu_pll1_test_clk", "cpu_pll1_fout4", 1, 8);
	clks[DPU0_PLL_TEST_CLK] = thead_light_clk_fixed_factor("dpu0_pll_test_clk", "dpu0_pll_fout4", 1, 8);
	clks[DPU1_PLL_TEST_CLK] = thead_light_clk_fixed_factor("dpu1_pll_test_clk", "dpu1_pll_fout4", 1, 8);
	clks[GMAC_PLL_TEST_CLK] = thead_light_clk_fixed_factor("gmac_pll_test_clk", "gmac_pll_fout4", 1, 8);
	clks[VIDEO_PLL_TEST_CLK] = thead_light_clk_fixed_factor("video_pll_test_clk", "video_pll_fout4", 1, 8);
	clks[TEE_PLL_TEST_CLK] = thead_light_clk_fixed_factor("tee_pll_test_clk", "tee_pll_fout4", 1, 8);
	clks[AONSYS_BUS_CLK] = thead_light_clk_fixed_factor("aonsys_bus_clk", "aonsys_hclk", 1, 1);

	/* Light Fullmask Clock Gate */
	clks[CLKGEN_AHB2_CPUSYS_HCLK] = thead_clk_light_gate("clkgen_ahb2_cpusys_hclk", "ahb2_cpusys_hclk", ap_base + 0x120, 6);
	clks[CLKGEN_APB3_CPUSYS_HCLK] = thead_clk_light_gate("clkgen_apb3_cpusys_hclk", "ahb2_cpusys_hclk", ap_base + 0x130, 4);
	clks[CLKGEN_C910_BROM_HCLK] = thead_clk_light_gate("clkgen_c910_brom_hclk", "ahb2_cpusys_hclk", ap_base + 0x100, 4);
	clks[CLKGEN_SPINLOCK_HCLK] = thead_clk_light_gate("clkgen_spinlock_hclk", "ahb2_cpusys_hclk", ap_base + 0x208, 10);
	clks[CLKGEN_MBOX0_PCLK] = thead_clk_light_gate("clkgen_mbox0_pclk", "apb3_cpusys_pclk", ap_base + 0x208, 7);
	clks[CLKGEN_MBOX1_PCLK] = thead_clk_light_gate("clkgen_mbox1_pclk", "apb3_cpusys_pclk", ap_base + 0x208, 6);
	clks[CLKGEN_MBOX2_PCLK] = thead_clk_light_gate("clkgen_mbox2_pclk", "apb3_cpusys_pclk", ap_base + 0x208, 5);
	clks[CLKGEN_MBOX3_PCLK] = thead_clk_light_gate("clkgen_mbox3_pclk", "apb3_cpusys_pclk", ap_base + 0x208, 4);
	clks[CLKGEN_WDT0_PCLK] = thead_clk_light_gate("clkgen_wdt0_pclk", "apb3_cpusys_pclk", ap_base + 0x208, 3);
	clks[CLKGEN_WDT1_PCLK] = thead_clk_light_gate("clkgen_wdt1_pclk", "apb3_cpusys_pclk", ap_base + 0x208, 2);

	if (teesys)
		clks[CLKGEN_MISCSYS_TEE_CCLK] = thead_clk_light_gate("clkgen_miscsys_tee_cclk", "teesys_hclk", ap_base + 0x1cc, 25); //just for teesys!!!

	clks[CLKGEN_SRAM_AXI_ACLK_2] = thead_clk_light_gate("clkgen_sram_axi_aclk_2", "axi4_cpusys1_aclk", ap_base + 0x20c, 2);
	clks[CLKGEN_PERISYS_AHB_HCLK] = thead_clk_light_gate("clkgen_perisys_ahb_hclk", "perisys_ahb_hclk", ap_base + 0x140, 6);
	clks[CLKGEN_PERISYS_APB1_HCLK] = thead_clk_light_gate("clkgen_perisys_apb1_hclk", "perisys_ahb_hclk", ap_base + 0x150, 9);
	clks[CLKGEN_PERISYS_APB2_HCLK] = thead_clk_light_gate("clkgen_perisys_apb2_hclk", "perisys_ahb_hclk", ap_base + 0x150, 10);
	clks[CLKGEN_PERISYS_APB4_HCLK] = thead_clk_light_gate("clkgen_perisys_apb4_hclk", "perisys_ahb_hclk", ap_base + 0x150, 12);
	clks[CLKGEN_PADCTRL0_APSYS_PCLK] = thead_clk_light_gate("clkgen_padctrl0_apsys_pclk", "perisys_ahb_hclk", ap_base + 0x204, 22);
	clks[CLKGEN_DSMART_PCLK] = thead_clk_light_gate("clkgen_dsmart_pclk", "peri2sys_apb_pclk", ap_base + 0x204, 23);
	clks[CLKGEN_PADCTRL1_APSYS_PCLK] = thead_clk_light_gate("clkgen_padctrl1_apsys_pclk", "peri2sys_apb_pclk", ap_base + 0x204, 24);
	clks[CLKGEN_CLK_OUT_1_CLK] = thead_clk_light_gate("clkgen_clk_out_1_clk", "clk_out_1", ap_base + 0x1b4, 5);
	clks[CLKGEN_CLK_OUT_2_CLK] = thead_clk_light_gate("clkgen_clk_out_2_clk", "clk_out_2", ap_base + 0x1b8, 5);
	clks[CLKGEN_CLK_OUT_3_CLK] = thead_clk_light_gate("clkgen_clk_out_3_clk", "clk_out_3", ap_base + 0x1bc, 5);
	clks[CLKGEN_CLK_OUT_4_CLK] = thead_clk_light_gate("clkgen_clk_out_4_clk", "clk_out_4", ap_base + 0x1c0, 5);
	clks[CLKGEN_NPUSYS_AXI_ACLK] = thead_clk_light_gate("clkgen_npusys_axi_aclk", "npu_cclk", ap_base + 0x1c8, 5);
	clks[CLKGEN_SRAM_AXI_ACLK_0] = thead_clk_light_gate("clkgen_sram_axi_aclk_0", "npu_cclk", ap_base + 0x20c, 4);
	clks[CLKGEN_APB_CPU2CFG_HCLK] = thead_clk_light_gate("clkgen_apb_cpu2cfg_hclk", "cfg_apb_pclk", ap_base + 0x1c4, 5);
	clks[CLKGEN_SRAM_AXI_ACLK_1] = thead_clk_light_gate("clkgen_sram_axi_aclk_1", "visys_aclk_m", ap_base + 0x20c, 3);
	clks[CLKGEN_SRAM_AXI_ACLK_3] = thead_clk_light_gate("clkgen_sram_axi_aclk_3", "vpsys_axi_aclk", ap_base + 0x20c, 1);
	clks[CLKGEN_VPSYS_VENC_CCLK] = thead_clk_light_gate("clkgen_vpsys_venc_cclk", "venc_cclk", ap_base + 0x1e4, 5);
	clks[CLKGEN_EMMC_SDIO_REF_CLK] = thead_clk_light_gate("clkgen_emmc_sdio_ref_clk", "emmc_sdio_ref_clk", ap_base + 0x204, 30);

	clks[CLKGEN_X2H_CPUSYS_MHCLK] = thead_clk_light_gate_shared("clkgen_x2h_cpusys_mhclk", "ahb2_cpusys_hclk", ap_base + 0x120, 7, &share_cnt_x2h_cpusys_clk_en);
	clks[CLKGEN_X2H_CPUSYS_ACLK] = thead_clk_light_gate_shared("clkgen_x2h_cpusys_aclk", "cfg_axi_aclk", ap_base + 0x120, 7, &share_cnt_x2h_cpusys_clk_en);
	clks[CLKGEN_DMAC_CPUSYS_HCLK] = thead_clk_light_gate_shared("clkgen_dmac_cpusys_hclk", "ahb2_cpusys_hclk", ap_base + 0x208, 8, &share_cnt_dmac_cpusys_clk_en);
	clks[CLKGEN_IOPMP_DMAC_CPUSYS_PCLK] = thead_clk_light_gate_shared("clkgen_iopmp_dmac_cpusys_pclk", "apb3_cpusys_pclk", ap_base + 0x208, 8, &share_cnt_dmac_cpusys_clk_en);
	clks[CLKGEN_DMAC_CPUSYS_ACLK] = thead_clk_light_gate_shared("clkgen_dmac_cpusys_aclk", "axi4_cpusys2_aclk", ap_base + 0x208, 8, &share_cnt_dmac_cpusys_clk_en);
	clks[CLKGEN_TIMER0_PCLK] = thead_clk_light_gate_shared("clkgen_timer0_pclk", "apb3_cpusys_pclk", ap_base + 0x208, 1, &share_cnt_timer0_clk_en);
	clks[CLKGEN_TIMER0_CCLK] = thead_clk_light_gate_shared("clkgen_timer0_cclk", "timer_cclk", ap_base + 0x208, 1, &share_cnt_timer0_clk_en);
	clks[CLKGEN_TIMER1_PCLK] = thead_clk_light_gate_shared("clkgen_timer1_pclk", "apb3_cpusys_pclk", ap_base + 0x208, 0, &share_cnt_timer1_clk_en);
	clks[CLKGEN_TIMER1_CCLK] = thead_clk_light_gate_shared("clkgen_timer1_cclk", "timer_cclk", ap_base + 0x208, 0, &share_cnt_timer1_clk_en);
	clks[CLKGEN_AXI4_CPUSYS2_PCLK] = thead_clk_light_gate_shared("clkgen_axi4_cpusys2_pclk", "apb3_cpusys_pclk", ap_base + 0x134, 5, &share_cnt_axi4_cpusys2_clk_en);
	clks[CLKGEN_AXI4_CPUSYS2_ACLK] = thead_clk_light_gate_shared("clkgen_axi4_cpusys2_aclk", "axi4_cpusys2_aclk", ap_base + 0x134, 5, &share_cnt_axi4_cpusys2_clk_en);
	clks[CLKGEN_BMU_C910_PCLK] = thead_clk_light_gate_shared("clkgen_bmu_c910_pclk", "apb3_cpusys_pclk", ap_base + 0x100, 5, &share_cnt_bmu_c910_clk_en);
	clks[CLKGEN_BMU_C910_ACLK] = thead_clk_light_gate_shared("clkgen_bmu_c910_aclk", "axi4_cpusys1_aclk", ap_base + 0x100, 5, &share_cnt_bmu_c910_clk_en);
	clks[CLKGEN_IOPMP_AON_PCLK] = thead_clk_light_gate_shared("clkgen_iopmp_aon_pclk", "apb3_cpusys_pclk", ap_base + 0x134, 8, &share_cnt_aon2cpu_a2x_clk_en);
	clks[CLKGEN_AON2CPU_A2X_ACLK] = thead_clk_light_gate_shared("clkgen_aon2cpu_a2x_aclk", "axi4_cpusys2_aclk", ap_base + 0x134, 8, &share_cnt_aon2cpu_a2x_clk_en);
	clks[CLKGEN_AON2CPU_A2X_HCLK] = thead_clk_light_gate_shared("clkgen_aon2cpu_a2x_hclk", "aonsys_bus_clk", ap_base + 0x134, 8, &share_cnt_aon2cpu_a2x_clk_en);
	clks[CLKGEN_IOPMP_CHIP_DBG_PCLK] = thead_clk_light_gate_shared("clkgen_iopmp_chip_dbg_pclk", "apb3_cpusys_pclk", ap_base + 0x208, 9, &share_cnt_chip_dbg_clk_en);
	clks[CLKGEN_CHIP_DBG_ACLK] = thead_clk_light_gate_shared("clkgen_chip_dbg_aclk", "axi4_cpusys2_aclk", ap_base + 0x208, 9, &share_cnt_chip_dbg_clk_en);
	clks[CLKGEN_CHIP_DBG_CCLK] = thead_clk_light_gate_shared("clkgen_chip_dbg_cclk", "chip_dbg_cclk", ap_base + 0x208, 9, &share_cnt_chip_dbg_clk_en);
	clks[CLKGEN_X2X_CPUSYS_ACLK_M] = thead_clk_light_gate_shared("clkgen_x2x_cpusys_aclk_m", "axi4_cpusys2_aclk", ap_base + 0x134, 7, &share_cnt_x2x_cpusys_clk_en);
	clks[CLKGEN_X2X_CPUSYS_ACLK_S] = thead_clk_light_gate_shared("clkgen_x2x_cpusys_aclk_s", "axi4_cpusys1_aclk", ap_base + 0x134, 7, &share_cnt_x2x_cpusys_clk_en);
	clks[CLKGEN_CPU2PERI_X2H_ACLK] = thead_clk_light_gate_shared("clkgen_cpu2peri_x2h_aclk", "axi4_cpusys1_aclk", ap_base + 0x140, 9, &share_cnt_cpu2peri_x2h_clk_en);
	clks[CLKGEN_CPU2PERI_X2H_MHCLK] = thead_clk_light_gate_shared("clkgen_cpu2peri_x2h_mhclk", "perisys_ahb_hclk", ap_base + 0x140, 9, &share_cnt_cpu2peri_x2h_clk_en);
	clks[CLKGEN_CPU2VI_X2H_ACLK] = thead_clk_light_gate_shared("clkgen_cpu2vi_x2h_aclk", "axi4_cpusys1_aclk", ap_base + 0x1d0, 21, &share_cnt_cpu2vi_x2h_clk_en);
	clks[CLKGEN_CPU2VI_X2H_MHCLK] = thead_clk_light_gate_shared("clkgen_cpu2vi_x2h_mhclk", "visys_ahb_hclk", ap_base + 0x1d0, 21, &share_cnt_cpu2vi_x2h_clk_en);
	clks[CLKGEN_CFG2TEE_X2H_ACLK] = thead_clk_light_gate_shared("clkgen_cfg2tee_x2h_aclk", "cfg_axi_aclk", ap_base + 0x1cc, 24, &share_cnt_cfg2tee_x2h_clk_en); // just for teesys!!!
	clks[CLKGEN_CFG2TEE_X2H_MHCLK] = thead_clk_light_gate_shared("clkgen_cfg2tee_x2h_mhclk", "teesys_hclk", ap_base + 0x1cc, 24, &share_cnt_cfg2tee_x2h_clk_en); // just for teesys!!!
	clks[CLKGEN_CPU2AON_X2H_ACLK] = thead_clk_light_gate_shared("clkgen_cpu2aon_x2h_aclk", "cfg_axi_aclk", ap_base + 0x138, 8, &share_cnt_cpu2aon_x2h_clk_en);
	clks[CLKGEN_CPU2AON_X2H_MHCLK] = thead_clk_light_gate_shared("clkgen_cpu2aon_x2h_mhclk", "aonsys_bus_clk", ap_base + 0x138, 8, &share_cnt_cpu2aon_x2h_clk_en);
	clks[CLKGEN_CPU2VP_X2P_ACLK] = thead_clk_light_gate_shared("clkgen_cpu2vp_x2p_aclk", "cfg_axi_aclk", ap_base + 0x1e0, 13, &share_cnt_cpu2vp_x2p_clk_en);
	clks[CLKGEN_CPU2VP_X2P_PCLK] = thead_clk_light_gate_shared("clkgen_cpu2vp_x2p_pclk", "vpsys_apb_pclk", ap_base + 0x1e0, 13, &share_cnt_cpu2vp_x2p_clk_en);
	clks[CLKGEN_TOP_AXI4S_ACLK] = thead_clk_light_gate_shared("clkgen_top_axi4s_aclk", "cfg_axi_aclk", ap_base + 0x1c8, 4, &share_cnt_npu_core_clk_en);
	clks[CLKGEN_TOP_APB_SX_PCLK] = thead_clk_light_gate_shared("clkgen_top_apb_sx_pclk", "cfg_apb_pclk", ap_base + 0x1c8, 4, &share_cnt_npu_core_clk_en);
	clks[CLKGEN_MISC2VP_X2X_ACLK_M] = thead_clk_light_gate_shared("clkgen_misc2vp_x2x_aclk_m", "perisys_ahb_hclk", ap_base + 0x1e0, 15, &share_cnt_vpsys_axi_aclk_en);
	clks[CLKGEN_VPSYS_ACLK] = thead_clk_light_gate_shared("clkgen_vpsys_aclk", "vpsys_axi_aclk", ap_base + 0x1e0, 15, &share_cnt_vpsys_axi_aclk_en);
	clks[CLKGEN_GMAC1_HCLK] = thead_clk_light_gate_shared("clkgen_gmac1_hclk", "perisys_ahb_hclk", ap_base + 0x204, 26, &share_cnt_gmac1_clk_en);
	clks[CLKGEN_GMAC1_PCLK] = thead_clk_light_gate_shared("clkgen_gmac1_pclk", "perisys_ahb_hclk", ap_base + 0x204, 26, &share_cnt_gmac1_clk_en);
	clks[CLKGEN_GMAC1_CCLK] = thead_clk_light_gate_shared("clkgen_gmac1_cclk", "gmac_cclk", ap_base + 0x204, 26, &share_cnt_gmac1_clk_en);
	clks[CLKGEN_GMAC0_HCLK] = thead_clk_light_gate_shared("clkgen_gmac0_hclk", "perisys_ahb_hclk", ap_base + 0x204, 19, &share_cnt_gmac0_clk_en);
	clks[CLKGEN_GMAC0_PCLK] = thead_clk_light_gate_shared("clkgen_gmac0_pclk", "perisys_ahb_hclk", ap_base + 0x204, 19, &share_cnt_gmac0_clk_en);
	clks[CLKGEN_GMAC0_CCLK] = thead_clk_light_gate_shared("clkgen_gmac0_cclk", "gmac_cclk", ap_base + 0x204, 19, &share_cnt_gmac0_clk_en);
	clks[CLKGEN_PERI2PERI1_APB_HCLK] = thead_clk_light_gate_shared("clkgen_peri2peri1_apb_hclk", "perisys_ahb_hclk", ap_base + 0x150, 11, &share_cnt_perisys_apb3_hclk_en);
	clks[CLKGEN_PERI2PERI1_APB_PCLK] = thead_clk_light_gate_shared("clkgen_peri2peri1_apb_pclk", "peri2sys_apb_pclk", ap_base + 0x150, 11, &share_cnt_perisys_apb3_hclk_en);
	clks[CLKGEN_QSPI0_PCLK] = thead_clk_light_gate_shared("clkgen_qspi0_pclk", "perisys_ahb_hclk", ap_base + 0x204, 17, &share_cnt_qspi0_clk_en);
	clks[CLKGEN_QSPI0_SSI_CLK] = thead_clk_light_gate_shared("clkgen_qspi0_ssi_clk", "qspi0_ssi_clk", ap_base + 0x204, 17, &share_cnt_qspi0_clk_en);
	clks[CLKGEN_GMAC_AXI_ACLK] = thead_clk_light_gate_shared("clkgen_gmac_axi_aclk", "perisys_ahb_hclk", ap_base + 0x204, 21, &share_cnt_gmac_axi_clk_en);
	clks[CLKGEN_GMAC_AXI_PCLK] = thead_clk_light_gate_shared("clkgen_gmac_axi_pclk", "perisys_ahb_hclk", ap_base + 0x204, 21, &share_cnt_gmac_axi_clk_en);
	clks[CLKGEN_GPIO0_PCLK] = thead_clk_light_gate_shared("clkgen_gpio0_pclk", "perisys_ahb_hclk", ap_base + 0x204, 8, &share_cnt_gpio0_clk_en);
	clks[CLKGEN_GPIO0_DBCLK] = thead_clk_light_gate_shared("clkgen_gpio0_dbclk", "gpio0_dbclk", ap_base + 0x204, 8, &share_cnt_gpio0_clk_en);
	clks[CLKGEN_GPIO1_PCLK] = thead_clk_light_gate_shared("clkgen_gpio1_pclk", "perisys_ahb_hclk", ap_base + 0x204, 7, &share_cnt_gpio0_clk_en);
	clks[CLKGEN_GPIO1_DBCLK] = thead_clk_light_gate_shared("clkgen_gpio1_dbclk", "gpio1_dbclk", ap_base + 0x204, 7, &share_cnt_gpio1_clk_en);
	clks[CLKGEN_PWM_PCLK] = thead_clk_light_gate_shared("clkgen_pwm_pclk", "perisys_apb_pclk", ap_base + 0x204, 18, &share_cnt_pwm_clk_en);
	clks[CLKGEN_PWM_CCLK] = thead_clk_light_gate_shared("clkgen_pwm_cclk", "pwm_cclk", ap_base + 0x204, 18, &share_cnt_pwm_clk_en);
	clks[CLKGEN_SPI_PCLK] = thead_clk_light_gate_shared("clkgen_spi_pclk", "perisys_apb_pclk", ap_base + 0x204, 15, &share_cnt_spi_clk_en);
	clks[CLKGEN_SPI_SSI_CLK] = thead_clk_light_gate_shared("clkgen_spi_ssi_clk", "spi_ssi_clk", ap_base + 0x204, 15, &share_cnt_spi_clk_en);
	clks[CLKGEN_UART0_PCLK] = thead_clk_light_gate_shared("clkgen_uart0_pclk", "perisys_apb_pclk", ap_base + 0x204, 14, &share_cnt_uart0_clk_en);
	clks[CLKGEN_UART0_SCLK] = thead_clk_light_gate_shared("clkgen_uart0_sclk", "uart_sclk", ap_base + 0x204, 14, &share_cnt_uart0_clk_en);
	clks[CLKGEN_UART2_PCLK] = thead_clk_light_gate_shared("clkgen_uart2_pclk", "perisys_apb_pclk", ap_base + 0x204, 12, &share_cnt_uart2_clk_en);
	clks[CLKGEN_UART2_SCLK] = thead_clk_light_gate_shared("clkgen_uart2_sclk", "uart_sclk", ap_base + 0x204, 12, &share_cnt_uart2_clk_en);
	clks[CLKGEN_I2C2_PCLK] = thead_clk_light_gate_shared("clkgen_i2c2_pclk", "perisys_apb_pclk", ap_base + 0x204, 3, &share_cnt_i2c2_clk_en);
	clks[CLKGEN_I2C2_IC_CLK] = thead_clk_light_gate_shared("clkgen_i2c2_ic_clk", "i2c_ic_clk", ap_base + 0x204, 3, &share_cnt_i2c2_clk_en);
	clks[CLKGEN_I2C3_PCLK] = thead_clk_light_gate_shared("clkgen_i2c3_pclk", "perisys_apb_pclk", ap_base + 0x204, 2, &share_cnt_i2c2_clk_en);
	clks[CLKGEN_I2C3_IC_CLK] = thead_clk_light_gate_shared("clkgen_i2c3_ic_clk", "i2c_ic_clk", ap_base + 0x204, 2, &share_cnt_i2c2_clk_en);
	clks[CLKGEN_I2S_PCLK] = thead_clk_light_gate_shared("clkgen_i2s_pclk", "perisys_apb_pclk", ap_base + 0x1f0, 1, &share_cnt_peri_i2s_clk_en);
	clks[CLKGEN_I2S_SRC_CLK] = thead_clk_light_gate_shared("clkgen_i2s_src_clk", "peri_i2s_src_clk", ap_base + 0x1f0, 1, &share_cnt_peri_i2s_clk_en);
	clks[CLKGEN_QSPI1_PCLK] = thead_clk_light_gate_shared("clkgen_qspi1_pclk", "peri2sys_apb_pclk", ap_base + 0x204, 16, &share_cnt_qspi1_clk_en);
	clks[CLKGEN_QSPI1_SSI_CLK] = thead_clk_light_gate_shared("clkgen_qspi1_ssi_clk", "qspi1_ssi_clk", ap_base + 0x204, 16, &share_cnt_qspi1_clk_en);
	clks[CLKGEN_UART1_PCLK] = thead_clk_light_gate_shared("clkgen_uart1_pclk", "per2sys_apb_pclk", ap_base + 0x204, 13, &share_cnt_uart1_clk_en);
	clks[CLKGEN_UART1_SCLK] = thead_clk_light_gate_shared("clkgen_uart1_sclk", "uart_sclk", ap_base + 0x204, 13, &share_cnt_uart1_clk_en);
	clks[CLKGEN_UART3_PCLK] = thead_clk_light_gate_shared("clkgen_uart3_pclk", "peri2sys_apb_pclk", ap_base + 0x204, 11, &share_cnt_uart3_clk_en);
	clks[CLKGEN_UART3_SCLK] = thead_clk_light_gate_shared("clkgen_uart3_sclk", "uart_sclk", ap_base + 0x204, 11, &share_cnt_uart3_clk_en);
	clks[CLKGEN_UART4_PCLK] = thead_clk_light_gate_shared("clkgen_uart4_pclk", "peri2sys_apb_pclk", ap_base + 0x204, 10, &share_cnt_uart4_clk_en);
	clks[CLKGEN_UART4_SCLK] = thead_clk_light_gate_shared("clkgen_uart4_sclk", "uart_sclk", ap_base + 0x204, 10, &share_cnt_uart4_clk_en);
	clks[CLKGEN_UART5_PCLK] = thead_clk_light_gate_shared("clkgen_uart5_pclk", "peri2sys_apb_pclk", ap_base + 0x204, 9, &share_cnt_uart5_clk_en);
	clks[CLKGEN_UART5_SCLK] = thead_clk_light_gate_shared("clkgen_uart5_sclk", "uart_sclk", ap_base + 0x204, 9, &share_cnt_uart5_clk_en);
	clks[CLKGEN_I2C0_PCLK] = thead_clk_light_gate_shared("clkgen_i2c0_pclk", "peri2sys_apb_pclk", ap_base + 0x204, 5, &share_cnt_i2c0_clk_en);
	clks[CLKGEN_I2C0_IC_CLK] = thead_clk_light_gate_shared("clkgen_i2c0_ic_clk", "i2c_ic_clk", ap_base + 0x204, 5, &share_cnt_i2c0_clk_en);
	clks[CLKGEN_I2C1_PCLK] = thead_clk_light_gate_shared("clkgen_i2c1_pclk", "peri2sys_apb_pclk", ap_base + 0x204, 4, &share_cnt_i2c1_clk_en);
	clks[CLKGEN_I2C1_IC_CLK] = thead_clk_light_gate_shared("clkgen_i2c1_ic_clk", "i2c_ic_clk", ap_base + 0x204, 4, &share_cnt_i2c1_clk_en);
	clks[CLKGEN_I2C4_PCLK] = thead_clk_light_gate_shared("clkgen_i2c4_pclk", "peri2sys_apb_pclk", ap_base + 0x204, 1, &share_cnt_i2c4_clk_en);
	clks[CLKGEN_I2C4_IC_CLK] = thead_clk_light_gate_shared("clkgen_i2c4_ic_clk", "i2c_ic_clk", ap_base + 0x204, 1, &share_cnt_i2c4_clk_en);
	clks[CLKGEN_I2C5_PCLK] = thead_clk_light_gate_shared("clkgen_i2c5_pclk", "peri2sys_apb_pclk", ap_base + 0x204, 0, &share_cnt_i2c5_clk_en);
	clks[CLKGEN_I2C5_IC_CLK] = thead_clk_light_gate_shared("clkgen_i2c5_ic_clk", "i2c_ic_clk", ap_base + 0x204, 0, &share_cnt_i2c5_clk_en);
	clks[CLKGEN_GPIO2_PCLK] = thead_clk_light_gate_shared("clkgen_gpio2_pclk", "peri2sys_apb_pclk", ap_base + 0x204, 6, &share_cnt_gpio2_clk_en);
	clks[CLKGEN_GPIO2_DBCLK] = thead_clk_light_gate_shared("clkgen_gpio2_dbclk", "gpio2_dbclk", ap_base + 0x204, 6, &share_cnt_gpio2_clk_en);
	clks[CLKGEN_GPIO3_PCLK] = thead_clk_light_gate_shared("clkgen_gpio3_pclk", "peri2sys_apb_pclk", ap_base + 0x204, 6, &share_cnt_gpio2_clk_en); //!!! gpio3 pclk is controlled by gpio2_clk_en
	clks[CLKGEN_GPIO3_DBCLK] = thead_clk_light_gate_shared("clkgen_gpio3_dbclk", "gpio3_dbclk", ap_base + 0x204, 20, &share_cnt_gpio3_clk_en);
	clks[CLKGEN_VOSYS_AXI_ACLK] = thead_clk_light_gate_shared("clkgen_vosys_axi_aclk", "vosys_aclk_m", ap_base + 0x1dc, 5, &share_cnt_vosys_axi_aclk_en);
	clks[CLKGEN_VOSYS_X2X_ACLK_S] = thead_clk_light_gate_shared("clkgen_vosys_x2x_aclk_s", "npu_cclk", ap_base + 0x1dc, 5, &share_cnt_vosys_axi_aclk_en);

	clk_data.clks = clks;
	clk_data.clk_num = ARRAY_SIZE(clks);

	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
	if (ret < 0) {
		dev_err(dev, "failed to register clks for light\n");
		goto unregister_clks;
	}

#ifndef FPGA_EMU
	/* HW defalut */
	clk_set_parent(clks[C910_CCLK], clks[CPU_PLL1_FOUTPOSTDIV]);
#else
	clk_set_parent(clks[C910_CCLK_I0], clks[OSC_24M]);
	clk_set_parent(clks[C910_CCLK], clks[C910_CCLK_I0]);
#endif
	dev_info(dev, "succeed to register light fullmask clock driver\n");

	return 0;

unregister_clks:
	thead_unregister_clocks(clks, ARRAY_SIZE(clks));
	return ret;
}


const bool tee_sys_flag;

static const struct of_device_id light_clk_of_match[] = {
	{ .compatible = "thead,light-fm-ree-clk" },
	{ .compatible = "thead,light-fm-tee-clk", .data = &tee_sys_flag,},
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, light_clk_of_match);

static struct platform_driver light_clk_driver = {
	.probe = light_clocks_probe,
	.driver = {
		.name = "light-fm-clk",
		.of_match_table = of_match_ptr(light_clk_of_match),
	},
};

module_platform_driver(light_clk_driver);
MODULE_AUTHOR("wei.liu <lw312886@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead Light Fullmask clock driver");
MODULE_LICENSE("GPL v2");
