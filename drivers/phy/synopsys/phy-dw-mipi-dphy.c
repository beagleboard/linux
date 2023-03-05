// SPDX-License-Identifier: GPL-2.0+
/*
 * DesignWare MIPI DSI DPHY driver
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* ----------------------------- DPHY registers --------------------- */

#define DW_PHY_TMR_LPCLK_CFG				0x98
    #define PHY_TMR_LPCLK_CFG_PHY_CLKHS2LP_TIME		GENMASK(25, 16)
    #define PHY_TMR_LPCLK_CFG_PHY_CLKLP2HS_TIME		GENMASK( 9,  0)
#define DW_PHY_TMR_CFG					0x9c
    #define PHY_TMR_CFG_PHY_HS2LP_TIME			GENMASK(25, 16)
    #define PHY_TMR_CFG_PHY_LP2HS_TIME			GENMASK( 9,  0)
#define DW_PHY_RSTZ					0xa0
    #define PHY_RSTZ_PHY_FORCEPLL			BIT(3)
    #define PHY_RSTZ_PHY_ENABLECLK			BIT(2)
    #define PHY_RSTZ_PHY_RSTZ				BIT(1)
    #define PHY_RSTZ_PHY_SHUTDOWNZ			BIT(0)
#define DW_PHY_IF_CFG					0xa4
    #define PHY_IF_CFG_PHY_STOP_WAIT_TIME		GENMASK(15,  8)
    #define PHY_IF_CFG_N_LANES				GENMASK( 1,  0)
#define DW_PHY_ULPS_CTRL				0xa8
    #define PHY_ULPS_CTRL_PHY_TXEXITULPSLAN		BIT(3)
    #define PHY_ULPS_CTRL_PHY_TXREQULPSLAN		BIT(2)
    #define PHY_ULPS_CTRL_PHY_TXEXITULPSCLK		BIT(1)
    #define PHY_ULPS_CTRL_PHY_TXREQULPSCLK		BIT(0)
#define DW_PHY_TX_TRIGGERS				0xac
    #define PHY_TX_TRIGGERS_PHY_TX_TRIGGERS		GENMASK( 3,  0)
#define DW_PHY_STATUS					0xb0
    #define PHY_STATUS_PHY_ULPSACTIVENOT3LANE		BIT(12)
    #define PHY_STATUS_PHY_STOPSTATE3LANE		BIT(11)
    #define PHY_STATUS_PHY_ULPSACTIVENOT2LANE		BIT(10)
    #define PHY_STATUS_PHY_STOPSTATE2LANE		BIT(9)
    #define PHY_STATUS_PHY_ULPSACTIVENOT1LANE		BIT(8)
    #define PHY_STATUS_PHY_STOPSTATE1LANE		BIT(7)
    #define PHY_STATUS_PHY_RXULPSESC0LANE		BIT(6)
    #define PHY_STATUS_PHY_ULPSACTIVENOT0LANE		BIT(5)
    #define PHY_STATUS_PHY_STOPSTATE0LANE		BIT(4)
    #define PHY_STATUS_PHY_ULPSACTIVENOTCLK		BIT(3)
    #define PHY_STATUS_PHY_STOPSTATE_CLKLANE		BIT(2)
    #define PHY_STATUS_PHY_DIRECTION			BIT(1)
    #define PHY_STATUS_PHY_LOCK				BIT(0)
#define DW_PHY_TST_CTRL0				0xb4
    #define PHY_TST_CTRL0_TESTCLK			BIT(1)
    #define PHY_TST_CTRL0_TESTCLR			BIT(0)
#define DW_PHY_TST_CTRL1				0xb8
    #define PHY_TST_CTRL1_TESTEN			BIT(16)
        #define TESTEN_WRITEADDR			0x1
        #define TESTEN_WRITEDATA			0x0
    #define PHY_TST_CTRL1_TESTOUT			GENMASK(15,  8)
    #define PHY_TST_CTRL1_TESTIN			GENMASK( 7,  0)
#define DW_PHY_CAL					0xcc
    #define PHY_CAL_TXSKEWCALHS				BIT(0)
#define DW_PHY_TMR_RD_CFG				0xf4
    #define PHY_TMR_RD_CFG_MAX_RD_TIME			GENMASK(14,  0)

/* -------------------------- DPHY Test Code ------------------------ */

#define TC_PLL_DELAY_LP_TX_START_LP11			0x02
    #define TXDINDLYTIME_9_2				GENMASK( 7,  0)
#define TC_PLL_FSM_CNTRL				0x03
#define TC_PLL_PROP_CHARGE_PUMP_CTRL			0x0e
    #define PLL_PROP_CNTRL				GENMASK( 5,  0)
#define TC_PLL_INT_CHARGE_PUMP_CTRL			0x0f
    #define PLL_INT_CNTRL				GENMASK( 5,  0)
#define TC_PLL_VCO_CTRL					0x12
    #define PLL_VCO_CNTRL_OVR_EN			BIT(6)
    #define PLL_VCO_CNTRL_OVR				GENMASK( 5,  0)
#define TC_PLL_GMP_CTRL_DIGITAL_TEST			0x13
    #define PLL_TESTLOCK				BIT(7)
    #define PLL_GMP_CNTRL				GENMASK( 5,  4)
    #define TSTPLLDIG					GENMASK( 3,  1)
        #define TSTPLLDIG_LOCK				0x0
        #define TSTPLLDIG_REFCLK			0x1
        #define TSTPLLDIG_FBCLK				0x2
        #define TSTPLLDIG_INT_CLKIN			0x3
        #define TSTPLLDIG_LOCK_FROM_DETECTOR		0x4
        #define TSTPLLDIG_OUTPUT			0x5
        #define TSTPLLDIG_BUF_VCOCLK			0x6
        #define TSTPLLDIG_UPDATEPLL			0x7
#define TC_PLL_N_DIV_FSM_SIG				0x17
    #define PLL_N_OVR					GENMASK( 3,  0)
#define TC_PLL_M_DIV					0x18
    #define PLL_M_OVR_4_0				GENMASK( 4,  0)
    #define PLL_M_OVR_9_5				GENMASK( 4,  0)
#define TC_PLL_N_AND_M_DIV_CTRL				0x19
    #define PLL_M_OVR_EN				BIT(5)
    #define PLL_N_OVR_EN				BIT(4)
#define TC_PLL_CHARGE_PUMP_BIAS_CTRL			0x1c
    #define PLL_CPBIAS_CNTRL				GENMASK( 6,  0)
#define TC_PLL_ANALOG_PROGRAM_CONTROL			0x1f
    #define PLL_MPLL_PROG_5_0				GENMASK( 5,  0)
    #define PLL_MPLL_PROG_11_6				GENMASK( 5,  0)
    #define PLL_MPLL_PROG_16_12				GENMASK( 4,  0)
#define TC_HS_FREQ_RANGE_OF_OPERATION			0x44
    #define HSFREQRANGE_OVR_EN				BIT(7)
    #define HSFREQRANGE_OVR				GENMASK( 6,  0)
    #define HSFREQRANGE					HSFREQRANGE_OVR
#define TC_PG_LP_BIAS_LANE0				0x4a
    #define PRG_ON_LANE0				BIT(6)
#define TC_SR_FSM_OVR_CNTRL				0xa0
#define TC_SR_DDL_LOOP_CONF				0xa3

/* ---------------------------------- PLL ------------------------------- */

#define FCLKIN_FREQ_MIN					2000UL		/* in KHz */
#define FCLKIN_FREQ_MAX					64000UL
#define FOUT_FREQ_MIN					40000UL
#define FOUT_FREQ_MAX					1250000UL
#define CFGCLK_FREQ_MIN					17000UL
#define CFGCLK_FREQ_MAX					27000UL

#define INPUT_DIV_MIN					1
#define INPUT_DIV_MAX					16
#define LOOP_DIV_MIN					64
#define LOOP_DIV_MAX					625
#define FIN_DIV_N_FREQ_MIN				2
#define FIN_DIV_N_FREQ_MAX				4

/* --------------------------------- VOSYS ------------------------------ */

#define VOSYS_MIPIDSI0_SYSREG				0x74
#define VOSYS_MIPIDSI1_SYSREG				0x78
    #define MIPIDSI_HSFREQRANGE				GENMASK( 9,  3)
    #define MIPIDSI_CFGCLKFREQRANGE			GENMASK(15, 10)

struct dw_pll_range {
	u32 data_rate;
	u8  hsfreqrange;
	u8  vco_range;
	u16 clk_lp2hs;
	u16 clk_hs2lp;
	u16 data_lp2hs;
	u16 data_hs2lp;
};

struct dw_dphy_cfg {
	unsigned long hs_clk_rate;
	u8 hsfreqrange;
	u8 cfgclkfreqrange;
	u8 vco_range;
	u8 n_div;
	u8 m_div;
	u16 clk_lp2hs;
	u16 clk_hs2lp;
	u16 data_lp2hs;
	u16 data_hs2lp;
};

struct dw_dphy {
	struct phy *phy;
	struct device *dev;
	struct dw_dphy_cfg cfg;
	struct regmap *regmap;
	struct regmap *vosys_regmap;
	struct clk *refclk;		/* PLL reference clock  */
	struct clk *cfgclk;		/* DPHY configure clock */
	struct clk *pclk;		/* APB slave bus clock  */
	struct clk *prefclk;
	struct clk *pcfgclk;
};

#define dw_fill_range(r, f, v, cl, ch, dl, dh) {	\
	.data_rate	= r,				\
	.hsfreqrange	= f,				\
	.vco_range	= v,				\
	.clk_lp2hs	= cl,				\
	.clk_hs2lp	= ch,				\
	.data_lp2hs	= dl,				\
	.data_hs2lp	= dh				\
}

static const struct dw_pll_range pll_range_table[] = {
	dw_fill_range(80,  0x00, 0x3F, 21, 17, 15, 10),
	dw_fill_range(90,  0x10, 0x3F, 23, 17, 16, 10),
	dw_fill_range(100, 0x20, 0x3F, 22, 17, 16, 10),
	dw_fill_range(110, 0x30, 0x39, 25, 18, 17, 11),
	dw_fill_range(120, 0x01, 0x39, 26, 20, 18, 11),
	dw_fill_range(130, 0x11, 0x39, 27, 19, 19, 11),
	dw_fill_range(140, 0x21, 0x39, 27, 19, 19, 11),
	dw_fill_range(150, 0x31, 0x39, 28, 20, 20, 12),
	dw_fill_range(160, 0x02, 0x39, 30, 21, 22, 13),
	dw_fill_range(170, 0x12, 0x2F, 30, 21, 23, 13),
	dw_fill_range(180, 0x22, 0x2F, 31, 21, 23, 13),
	dw_fill_range(190, 0x32, 0x2F, 32, 22, 24, 13),
	dw_fill_range(205, 0x03, 0x2F, 35, 22, 25, 13),
	dw_fill_range(220, 0x13, 0x29, 37, 26, 27, 15),
	dw_fill_range(235, 0x23, 0x29, 38, 28, 27, 16),
	dw_fill_range(250, 0x33, 0x29, 41, 29, 30, 17),
	dw_fill_range(275, 0x04, 0x29, 43, 29, 32, 18),
	dw_fill_range(300, 0x14, 0x29, 45, 32, 35, 19),
	dw_fill_range(325, 0x25, 0x29, 48, 33, 36, 18),
	dw_fill_range(350, 0x35, 0x1F, 51, 35, 40, 20),
	dw_fill_range(400, 0x05, 0x1F, 59, 37, 44, 21),
	dw_fill_range(450, 0x16, 0x19, 65, 40, 49, 23),
	dw_fill_range(500, 0x26, 0x19, 71, 41, 54, 24),
	dw_fill_range(550, 0x37, 0x19, 77, 44, 57, 26),
	dw_fill_range(600, 0x07, 0x19, 82, 46, 64, 27),
	dw_fill_range(650, 0x18, 0x19, 87, 48, 67, 28),
	dw_fill_range(700, 0x28, 0x0F, 94, 52, 71, 29),
	dw_fill_range(750, 0x39, 0x0F, 99, 52, 75, 31),
	dw_fill_range(800, 0x09, 0x0F, 105, 55, 82, 32),
	dw_fill_range(850, 0x19, 0x0F, 110, 58, 85, 32),
	dw_fill_range(900, 0x29, 0x09, 115, 58, 88, 35),
	dw_fill_range(950, 0x3A, 0x09, 120, 62, 93, 36),
	dw_fill_range(1000, 0x0A, 0x09, 128, 63, 99, 38),
	dw_fill_range(1050, 0x1A, 0x09, 132, 65, 102, 38),
	dw_fill_range(1100, 0x2A, 0x09, 138, 67, 106, 39),
	dw_fill_range(1150, 0x3B, 0x09, 146, 69, 112, 42),
	dw_fill_range(1200, 0x0B, 0x09, 151, 71, 117, 43),
	dw_fill_range(1250, 0x1B, 0x09, 153, 74, 120, 45),
	dw_fill_range(1300, 0x2B, 0x09, 160, 73, 124, 46),
	dw_fill_range(1350, 0x3C, 0x03, 165, 76, 130, 47),
	dw_fill_range(1400, 0x0C, 0x03, 172, 78, 134, 49),
	dw_fill_range(1450, 0x1C, 0x03, 177, 80, 138, 49),
	dw_fill_range(1500, 0x2C, 0x03, 183, 81, 143, 52),
	dw_fill_range(1550, 0x3D, 0x03, 191, 84, 147, 52),
	dw_fill_range(1600, 0x0D, 0x03, 194, 85, 152, 52),
	dw_fill_range(1650, 0x1D, 0x03, 201, 86, 155, 53),
	dw_fill_range(1700, 0x2E, 0x03, 208, 88, 161, 53),
	dw_fill_range(1750, 0x3E, 0x03, 212, 89, 165, 53),
	dw_fill_range(1800, 0x0E, 0x03, 220, 90, 171, 54),
	dw_fill_range(1850, 0x1E, 0x03, 223, 92, 175, 54),
	dw_fill_range(1900, 0x2F, 0x03, 231, 91, 180, 55),
	dw_fill_range(1950, 0x3F, 0x03, 236, 95, 185, 56),
	dw_fill_range(2000, 0x0F, 0x03, 243, 97, 190, 56),
	dw_fill_range(2050, 0x40, 0x03, 248, 99, 194, 58),
	dw_fill_range(2100, 0x41, 0x03, 252, 100, 199, 59),
	dw_fill_range(2150, 0x42, 0x03, 259, 102, 204, 61),
	dw_fill_range(2200, 0x43, 0x03, 266, 105, 210, 62),
	dw_fill_range(2250, 0x44, 0x03, 269, 109, 213, 63),
	dw_fill_range(2300, 0x45, 0x01, 272, 109, 217, 65),
	dw_fill_range(2350, 0x46, 0x01, 281, 112, 225, 66),
	dw_fill_range(2400, 0x47, 0x01, 283, 115, 226, 66),
	dw_fill_range(2450, 0x48, 0x01, 282, 115, 226, 67),
	dw_fill_range(2500, 0x49, 0x01, 281, 118, 227, 67),
};

static int dw_dphy_config_rstz(struct dw_dphy *dphy, u8 val)
{
	int ret;
	u32 rstz = FIELD_PREP(PHY_RSTZ_PHY_RSTZ, !!val);

	ret = regmap_update_bits(dphy->regmap, DW_PHY_RSTZ,
				 PHY_RSTZ_PHY_RSTZ, rstz);
	if (ret)
		dev_err(dphy->dev, "config rstz failed : %d\n", ret);

	return  ret;
}

static int dw_dphy_config_shutdownz(struct dw_dphy *dphy, u8 val)
{
	int ret;
	u32 shutdownz = FIELD_PREP(PHY_RSTZ_PHY_SHUTDOWNZ, !!val);

	ret = regmap_update_bits(dphy->regmap, DW_PHY_RSTZ,
				 PHY_RSTZ_PHY_SHUTDOWNZ, shutdownz);
	if (ret)
		dev_err(dphy->dev, "config shutdownz failed: %d\n", ret);

	return ret;
}

static int dw_dphy_config_enableclk(struct dw_dphy *dphy, u8 val)
{
	int ret;
	u32 enableclk = FIELD_PREP(PHY_RSTZ_PHY_ENABLECLK, !!val);

	ret = regmap_update_bits(dphy->regmap, DW_PHY_RSTZ,
				 PHY_RSTZ_PHY_ENABLECLK, enableclk);
	if (ret)
		dev_err(dphy->dev, "config enableclk failed: %d\n", ret);

	return ret;
}

static int dw_dphy_config_testclr(struct dw_dphy *dphy, u8 val)
{
	int ret;
	u32 testclr = FIELD_PREP(PHY_TST_CTRL0_TESTCLR, !!val);

	ret = regmap_update_bits(dphy->regmap, DW_PHY_TST_CTRL0,
				 PHY_TST_CTRL0_TESTCLR, testclr);
	if (ret)
		dev_err(dphy->dev, "config testclr failed: %d\n", ret);

	return ret;
}

static int dw_dphy_config_testclk(struct dw_dphy *dphy, u8 val)
{
	int ret;
	u32 testclk = FIELD_PREP(PHY_TST_CTRL0_TESTCLK, !!val);

	ret = regmap_update_bits(dphy->regmap, DW_PHY_TST_CTRL0,
				 PHY_TST_CTRL0_TESTCLK, testclk);
	if (ret)
		dev_err(dphy->dev, "config testclk failed: %d\n", ret);

	return ret;
}

static int dw_dphy_config_testen(struct dw_dphy *dphy, u8 val)
{
	int ret;
	u32 testen = FIELD_PREP(PHY_TST_CTRL1_TESTEN, !!val);

	ret = regmap_update_bits(dphy->regmap, DW_PHY_TST_CTRL1,
				 PHY_TST_CTRL1_TESTEN, testen);
	if (ret)
		dev_err(dphy->dev, "config testen failed: %d\n", ret);

	return ret;
}

static int __maybe_unused dw_dphy_read_testdout(struct dw_dphy *dphy, u8 *val)
{
	int ret;
	u32 phy_tst_ctrl1;

	ret = regmap_read(dphy->regmap, DW_PHY_TST_CTRL1, &phy_tst_ctrl1);
	if (ret) {
		dev_err(dphy->dev, "read phy_tst_ctrl1 failed: %d\n", ret);
		return ret;
	}

	*val = FIELD_GET(PHY_TST_CTRL1_TESTOUT, phy_tst_ctrl1);

	return 0;
}

static int dw_dphy_config_testdin(struct dw_dphy *dphy, u8 val)
{
	int ret;
	u32 testdin = FIELD_PREP(PHY_TST_CTRL1_TESTIN, val);

	ret = regmap_update_bits(dphy->regmap, DW_PHY_TST_CTRL1,
				 PHY_TST_CTRL1_TESTIN, testdin);
	if (ret)
		dev_err(dphy->dev, "config testdin failed: %d\n", ret);

	return ret;
}

/* Program Test Code */
static int dw_dphy_phy_write(struct dw_dphy *dphy, u8 test_code,
			     u8 *test_data, u8 len)
{
	int i;

	/* set the desired test code */
	dw_dphy_config_testclk(dphy, 1);
	dw_dphy_config_testdin(dphy, test_code);
	dw_dphy_config_testen(dphy, TESTEN_WRITEADDR);
	dw_dphy_config_testclk(dphy, 0);

	/* enter the necessary test data */
	dw_dphy_config_testen(dphy, TESTEN_WRITEDATA);
	for (i = 0; i < len; i++) {
		pr_info("%s: test_code = %#x, len = %u, data[%d] = %#x\n",
			__func__, test_code, len, i, test_data[i]);
		dw_dphy_config_testdin(dphy, test_data[i]);
		dw_dphy_config_testclk(dphy, 1);
		dw_dphy_config_testclk(dphy, 0);
	}

	return 0;
}

static int dw_dphy_pll_config(struct dw_dphy *dphy, struct dw_dphy_cfg *cfg)
{
	u8 data[8];
	u32 data_rate = DIV_ROUND_UP_ULL(cfg->hs_clk_rate, 1000000);

	data[0] = FIELD_PREP(PLL_PROP_CNTRL, data_rate >= 1150 ? 0xE : 0x8);
	dw_dphy_phy_write(dphy, TC_PLL_PROP_CHARGE_PUMP_CTRL, data, 1);

	data[0] = FIELD_PREP(PLL_CPBIAS_CNTRL, 0x10);
	dw_dphy_phy_write(dphy, TC_PLL_CHARGE_PUMP_BIAS_CTRL, data, 1);

	return 0;
}

static int dw_dphy_init(struct phy *phy)
{
	struct dw_dphy *dphy = phy_get_drvdata(phy);

	pm_runtime_get_sync(dphy->dev);

	/* reset testclr */
	dw_dphy_config_testclr(dphy, 1);
	dw_dphy_config_testclr(dphy, 0);

	pm_runtime_put(dphy->dev);

	return 0;
}

static int dw_dphy_exit(struct phy *phy)
{
	return 0;
}

static int dw_dphy_power_on(struct phy *phy)
{
	struct dw_dphy *dphy = phy_get_drvdata(phy);

	pm_runtime_get_sync(dphy->dev);

	dw_dphy_config_enableclk(dphy, 1);

	dw_dphy_config_shutdownz(dphy, 1);

	dw_dphy_config_rstz(dphy, 1);

	return 0;
}

static int dw_dphy_power_off(struct phy *phy)
{
	struct dw_dphy *dphy = phy_get_drvdata(phy);

	dw_dphy_config_rstz(dphy, 0);

	dw_dphy_config_enableclk(dphy, 0);

	dw_dphy_config_shutdownz(dphy, 0);

	pm_runtime_put(dphy->dev);

	return 0;
}

static int dw_dphy_get_pll_cfg(struct dw_dphy *dphy,
			       struct phy_configure_opts_mipi_dphy *opts,
			       struct dw_dphy_cfg *cfg)
{
	int i;
	u32 n_min, n_max;
	u32 delta, best_delta = ~0U;
	u32 n, m, vco_div, best_n = 0, best_m = 0;
	unsigned long fin, fout;
	const struct dw_pll_range *range;

	fin = DIV_ROUND_UP_ULL(clk_get_rate(dphy->prefclk), 1000);
	if (fin < FCLKIN_FREQ_MIN || fin > FCLKIN_FREQ_MAX)
		return -EINVAL;

	fout = DIV_ROUND_UP_ULL(opts->hs_clk_rate, 1000) >> 1;
	if (fout < FOUT_FREQ_MIN || fout > FOUT_FREQ_MAX)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(pll_range_table); i++) {
		range = &pll_range_table[i];
		if (DIV_ROUND_UP_ULL(opts->hs_clk_rate, 1000000) <= range->data_rate)
			break;
	}

	/* update hs_clk_rate */
	opts->hs_clk_rate = range->data_rate * 1000000;

	vco_div = 1 << (range->vco_range >> 4);
	fout = fout * vco_div;
	pr_debug("%s: vco_div = %u\n", __func__, vco_div);

	n_min = DIV_ROUND_UP_ULL(fin, FIN_DIV_N_FREQ_MAX * 1000);
	n_max = DIV_ROUND_UP_ULL(fin, FIN_DIV_N_FREQ_MIN * 1000);

	for (n = n_min; n < n_max + 1; n++) {
		m = DIV_ROUND_UP_ULL(fout * n, fin);

		if (m < 64 || m > 625)
			continue;

		delta = abs(fout * n - fin * m);
		if (delta < best_delta) {
			best_delta = delta;
			best_n = n;
			best_m = m;
		}
	}

	WARN_ON(unlikely(best_delta == ~0U));

	cfg->hs_clk_rate = opts->hs_clk_rate;
	cfg->hsfreqrange = range->hsfreqrange;
	cfg->vco_range	 = range->vco_range;
	cfg->n_div	 = best_n;
	cfg->m_div	 = best_m;

	cfg->clk_lp2hs	 = range->clk_lp2hs;
	cfg->clk_hs2lp	 = range->clk_hs2lp;
	cfg->data_lp2hs	 = range->data_lp2hs;
	cfg->data_hs2lp	 = range->data_hs2lp;

	return 0;
}

static int dw_dphy_config_from_opts(struct dw_dphy *dphy,
				    struct phy_configure_opts_mipi_dphy *opts,
				    struct dw_dphy_cfg *cfg)
{
	int ret;

	ret = phy_mipi_dphy_config_validate(opts);
	if (ret)
		return ret;

	ret = dw_dphy_get_pll_cfg(dphy, opts, cfg);
	if (ret)
		return ret;

	return 0;
}

static int dw_dphy_validate(struct phy *phy, enum phy_mode mode,
			    int submode, union phy_configure_opts *opts)
{
	struct dw_dphy_cfg cfg = { 0 };
	struct dw_dphy *dphy = phy_get_drvdata(phy);

	if (mode != PHY_MODE_MIPI_DPHY)
		return -EINVAL;

	return dw_dphy_config_from_opts(dphy, &opts->mipi_dphy, &cfg);
}

static int dw_dphy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	int ret;
	u8 data[8];
	u32 phy_if_cfg, hsfreqrange, cfgclkfreqrange;
	u32 phy_tmr_lpclk_cfg, phy_tmr_cfg;
	unsigned long cfgclk_freq;
	struct dw_dphy_cfg cfg = { 0 };
	struct dw_dphy *dphy = phy_get_drvdata(phy);
	struct phy_configure_opts_mipi_dphy *dphy_opts = &opts->mipi_dphy;

	ret = dw_dphy_config_from_opts(dphy, dphy_opts, &cfg);
	if (ret)
		return ret;

	/* configure hsfreqrange and cfgclkfreqrange */
	hsfreqrange = FIELD_PREP(MIPIDSI_HSFREQRANGE, cfg.hsfreqrange);
	ret = regmap_update_bits(dphy->vosys_regmap, VOSYS_MIPIDSI0_SYSREG,
				 MIPIDSI_HSFREQRANGE, hsfreqrange);
	if (ret) {
		dev_err(dphy->dev, "config dsi0 hsfreqrange failed\n");
		return ret;
	}

	hsfreqrange = FIELD_PREP(MIPIDSI_HSFREQRANGE, cfg.hsfreqrange);
	ret = regmap_update_bits(dphy->vosys_regmap, VOSYS_MIPIDSI1_SYSREG,
				 MIPIDSI_HSFREQRANGE, hsfreqrange);
	if (ret) {
		dev_err(dphy->dev, "config dsi1 hsfreqrange failed\n");
		return ret;
	}

	/* TODO: config prg_on_lane0 */

	/* TODO: config cfgclkhsfreqrange:
	 *
	 * cfgclkfreqrange[5:0] = round[(Fcfg_clk(MHz)-17)*4]
	 *
	 */
	cfgclk_freq = DIV_ROUND_UP_ULL(clk_get_rate(dphy->pcfgclk), 1000);
	if (cfgclk_freq < CFGCLK_FREQ_MIN || cfgclk_freq > CFGCLK_FREQ_MAX)
		return -EINVAL;
	cfgclkfreqrange = FIELD_PREP(MIPIDSI_CFGCLKFREQRANGE,
				     (cfgclk_freq / 1000 - 17) * 4);
	ret = regmap_update_bits(dphy->vosys_regmap, VOSYS_MIPIDSI0_SYSREG,
				 MIPIDSI_CFGCLKFREQRANGE, cfgclkfreqrange);
	if (ret) {
		dev_err(dphy->dev, "config dsi0 cfgclkfreqrange failed\n");
		return ret;
	}

	ret = regmap_update_bits(dphy->vosys_regmap, VOSYS_MIPIDSI1_SYSREG,
				 MIPIDSI_CFGCLKFREQRANGE, cfgclkfreqrange);
	if (ret) {
		dev_err(dphy->dev, "config dsi1 cfgclkfreqrange failed\n");
		return ret;
	}


	/* TODO: disable slew rate calibration */
	data[0] = 0x0;
	dw_dphy_phy_write(dphy, TC_SR_DDL_LOOP_CONF, data, 1);
	data[0] = 0x1;
	dw_dphy_phy_write(dphy, TC_SR_FSM_OVR_CNTRL, data, 1);

	/* configure prg_on_lane0 */
	data[0] = FIELD_PREP(PRG_ON_LANE0, 1);
	dw_dphy_phy_write(dphy, TC_PG_LP_BIAS_LANE0, data, 1);

	dw_dphy_pll_config(dphy, &cfg);

	phy_if_cfg = FIELD_PREP(PHY_IF_CFG_N_LANES, dphy_opts->lanes - 1) |
		     FIELD_PREP(PHY_IF_CFG_PHY_STOP_WAIT_TIME, 0x2);
	regmap_write(dphy->regmap, DW_PHY_IF_CFG, phy_if_cfg);

	/* config clock & data lane timers */
	phy_tmr_lpclk_cfg = FIELD_PREP(PHY_TMR_LPCLK_CFG_PHY_CLKHS2LP_TIME, cfg.clk_hs2lp)	|
			    FIELD_PREP(PHY_TMR_LPCLK_CFG_PHY_CLKLP2HS_TIME, cfg.clk_lp2hs);
	regmap_write(dphy->regmap, DW_PHY_TMR_LPCLK_CFG, phy_tmr_lpclk_cfg);

	phy_tmr_cfg = FIELD_PREP(PHY_TMR_CFG_PHY_HS2LP_TIME, cfg.data_hs2lp)	|
		      FIELD_PREP(PHY_TMR_CFG_PHY_LP2HS_TIME, cfg.data_lp2hs);
	regmap_write(dphy->regmap, DW_PHY_TMR_CFG, phy_tmr_cfg);

	regmap_write(dphy->regmap, DW_PHY_TMR_RD_CFG, 10000);

	return 0;
}

static const struct phy_ops dw_dphy_phy_ops = {
	.init		= dw_dphy_init,
	.exit		= dw_dphy_exit,
	.power_on	= dw_dphy_power_on,
	.power_off	= dw_dphy_power_off, 
	.validate	= dw_dphy_validate,
	.configure	= dw_dphy_configure,
	.owner		= THIS_MODULE,
};

static const struct regmap_config dw_dphy_regmap_config = {
	.name = "dw-dphy",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x40,
};

static int dw_dphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct phy_provider *provider;
	struct dw_dphy *dphy;

	dphy = devm_kzalloc(dev, sizeof(*dphy), GFP_KERNEL);
	if (!dphy)
		return -ENOMEM;
	dphy->dev = dev;

	dphy->regmap = syscon_regmap_lookup_by_phandle(np, "regmap");
	if (IS_ERR(dphy->regmap))
		return PTR_ERR(dphy->regmap);

	dphy->vosys_regmap = syscon_regmap_lookup_by_phandle(np, "vosys-regmap");
	if (IS_ERR(dphy->vosys_regmap))
		return PTR_ERR(dphy->vosys_regmap);

	dphy->refclk = devm_clk_get(dev, "refclk");
	if (IS_ERR(dphy->refclk))
		return PTR_ERR(dphy->refclk);

	dphy->cfgclk = devm_clk_get(dev, "cfgclk");
	if (IS_ERR(dphy->cfgclk))
		return PTR_ERR(dphy->cfgclk);

	dphy->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(dphy->pclk))
		return PTR_ERR(dphy->pclk);

	dphy->prefclk = devm_clk_get(dev, "prefclk");
	if (IS_ERR(dphy->prefclk))
		return PTR_ERR(dphy->prefclk);

	dphy->pcfgclk = devm_clk_get(dev, "pcfgclk");
	if (IS_ERR(dphy->pcfgclk))
		return PTR_ERR(dphy->pcfgclk);

	platform_set_drvdata(pdev, dphy);

	pm_suspend_ignore_children(dev, true);
	pm_runtime_enable(dev);

	dphy->phy = devm_phy_create(dev, np, &dw_dphy_phy_ops);
	if (IS_ERR(dphy->phy))
		return PTR_ERR(dphy->phy);
	phy_set_drvdata(dphy->phy, dphy);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	
	return PTR_ERR_OR_ZERO(provider);
}

static int dw_dphy_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id dw_dphy_of_match[] = {
	{ .compatible = "thead,light-mipi-dphy", .data = NULL, },
};
MODULE_DEVICE_TABLE(of, dw_dphy_of_match);

#ifdef CONFIG_PM
static int dw_dphy_runtime_suspend(struct device *dev)
{
	struct dw_dphy *dphy = dev_get_drvdata(dev);

	clk_disable_unprepare(dphy->refclk);
	clk_disable_unprepare(dphy->cfgclk);
	clk_disable_unprepare(dphy->pclk);

	return 0;
}

static int dw_dphy_runtime_resume(struct device *dev)
{
	int ret;
	struct dw_dphy *dphy = dev_get_drvdata(dev);

	ret = clk_prepare_enable(dphy->pclk);
	if (ret) {
		dev_err(dev, "failed to prepare/enable pclk\n");
		return ret;
	}

	ret = clk_prepare_enable(dphy->cfgclk);
	if (ret) {
		dev_err(dev, "failed to prepare/enable cfgclk\n");
		return ret;
	}

	ret = clk_prepare_enable(dphy->refclk);
	if (ret) {
		dev_err(dev, "failed to prepare/enable refclk\n");
		return ret;
	}

	return 0;
}
#endif

static const struct dev_pm_ops dw_dphy_pm_ops = {
    SET_LATE_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				 pm_runtime_force_resume)
    SET_RUNTIME_PM_OPS(dw_dphy_runtime_suspend, dw_dphy_runtime_resume, NULL)
};

static struct platform_driver dw_dphy_driver = {
	.probe	= dw_dphy_probe,
	.remove	= dw_dphy_remove,
	.driver	= {
		.name = "dw-mipi-dphy",
		.of_match_table = dw_dphy_of_match,
		.pm = &dw_dphy_pm_ops,
	},
};
module_platform_driver(dw_dphy_driver);

MODULE_AUTHOR("You Xiao <youxiao.fc@linux.alibaba.com>");
MODULE_DESCRIPTION("Synopsys DesignWare MIPI DPHY driver");
MODULE_LICENSE("GPL");
