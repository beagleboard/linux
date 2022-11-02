// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright: 2017-2018 Cadence Design Systems, Inc.
 * Copyright (C) 2021 Texas Instruments Incorporated - https://www.ti.com/
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/sys_soc.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>

#define REG_WAKEUP_TIME_NS		800
#define DPHY_PLL_RATE_HZ		108000000
#define POLL_TIMEOUT_US			1000

/* DPHY registers */
#define DPHY_PMA_CMN(reg)		(reg)
#define DPHY_PMA_LCLK(reg)		(0x100 + (reg))
#define DPHY_PMA_LDATA(lane, reg)	(0x200 + ((lane) * 0x100) + (reg))
#define DPHY_PMA_RCLK(reg)		(0x600 + (reg))
#define DPHY_PMA_RDATA(lane, reg)	(0x700 + ((lane) * 0x100) + (reg))
#define DPHY_PCS(reg)			(0xb00 + (reg))
#define DPHY_ISO(reg)			(0xc00 + (reg))
#define DPHY_WRAP(reg)			(0x1000 + (reg))

#define DPHY_CMN_SSM			DPHY_PMA_CMN(0x20)
#define DPHY_CMN_SSM_EN			BIT(0)
#define DPHY_CMN_RX_BANDGAP_TIMER_MASK	GENMASK(8, 1)
#define DPHY_CMN_TX_MODE_EN		BIT(9)
#define DPHY_CMN_RX_MODE_EN		BIT(10)
#define DPHY_CMN_RX_BANDGAP_TIMER	0x14

#define DPHY_CMN_PWM			DPHY_PMA_CMN(0x40)
#define DPHY_CMN_PWM_DIV(x)		((x) << 20)
#define DPHY_CMN_PWM_LOW(x)		((x) << 10)
#define DPHY_CMN_PWM_HIGH(x)		(x)

#define DPHY_CMN_FBDIV			DPHY_PMA_CMN(0x4c)
#define DPHY_CMN_FBDIV_VAL(low, high)	(((high) << 11) | ((low) << 22))
#define DPHY_CMN_FBDIV_FROM_REG		(BIT(10) | BIT(21))

#define DPHY_CMN_OPIPDIV		DPHY_PMA_CMN(0x50)
#define DPHY_CMN_IPDIV_FROM_REG		BIT(0)
#define DPHY_CMN_IPDIV(x)		((x) << 1)
#define DPHY_CMN_OPDIV_FROM_REG		BIT(6)
#define DPHY_CMN_OPDIV(x)		((x) << 7)

#define DPHY_BAND_CFG			DPHY_PCS(0x0)
#define DPHY_BAND_CFG_LEFT_BAND		GENMASK(4, 0)
#define DPHY_BAND_CFG_RIGHT_BAND	GENMASK(9, 5)

#define DPHY_PSM_CFG			DPHY_PCS(0x4)
#define DPHY_PSM_CFG_FROM_REG		BIT(0)
#define DPHY_PSM_CLK_DIV(x)		((x) << 1)

#define DPHY_POWER_ISLAND_EN_DATA	DPHY_PCS(0x8)
#define DPHY_POWER_ISLAND_EN_DATA_VAL	0xaaaaaaaa
#define DPHY_POWER_ISLAND_EN_CLK	DPHY_PCS(0xc)
#define DPHY_POWER_ISLAND_EN_CLK_VAL	0xaa

#define DPHY_LANE			DPHY_WRAP(0x0)
#define DPHY_LANE_RESET_CMN_EN		BIT(23)

#define DPHY_ISO_CL_CTRL_L		DPHY_ISO(0x10)
#define DPHY_ISO_DL_CTRL_L0		DPHY_ISO(0x14)
#define DPHY_ISO_DL_CTRL_L1		DPHY_ISO(0x20)
#define DPHY_ISO_DL_CTRL_L2		DPHY_ISO(0x30)
#define DPHY_ISO_DL_CTRL_L3		DPHY_ISO(0x3c)
#define DPHY_ISO_LANE_READY_BIT		0
#define DPHY_ISO_LANE_READY_TIMEOUT_MS	100UL

#define DSI_HBP_FRAME_OVERHEAD		12
#define DSI_HSA_FRAME_OVERHEAD		14
#define DSI_HFP_FRAME_OVERHEAD		6
#define DSI_HSS_VSS_VSE_FRAME_OVERHEAD	4
#define DSI_BLANKING_FRAME_OVERHEAD	6
#define DSI_NULL_FRAME_OVERHEAD		6
#define DSI_EOT_PKT_SIZE		4

#define DPHY_LANES_MIN			1
#define DPHY_LANES_MAX			4

#define DPHY_TX_J721E_WIZ_PLL_CTRL	0xF04
#define DPHY_TX_J721E_WIZ_STATUS	0xF08
#define DPHY_TX_J721E_WIZ_RST_CTRL	0xF0C
#define DPHY_TX_J721E_WIZ_PSM_FREQ	0xF10

#define DPHY_TX_J721E_WIZ_IPDIV		GENMASK(4, 0)
#define DPHY_TX_J721E_WIZ_OPDIV		GENMASK(13, 8)
#define DPHY_TX_J721E_WIZ_FBDIV		GENMASK(25, 16)
#define DPHY_TX_J721E_WIZ_LANE_RSTB	BIT(31)
#define DPHY_TX_WIZ_PLL_LOCK		BIT(31)
#define DPHY_TX_WIZ_O_CMN_READY		BIT(31)

struct cdns_dphy_cfg {
	u8 pll_ipdiv;
	u8 pll_opdiv;
	u16 pll_fbdiv;
	unsigned int nlanes;
};

enum cdns_dphy_clk_lane_cfg {
	DPHY_CLK_CFG_LEFT_DRIVES_ALL = 0,
	DPHY_CLK_CFG_LEFT_DRIVES_RIGHT = 1,
	DPHY_CLK_CFG_LEFT_DRIVES_LEFT = 2,
	DPHY_CLK_CFG_RIGHT_DRIVES_ALL = 3,
};

struct cdns_dphy;
struct cdns_dphy_ops {
	int (*probe)(struct cdns_dphy *dphy);
	void (*remove)(struct cdns_dphy *dphy);
	int (*power_on)(struct cdns_dphy *dphy);
	int (*power_off)(struct cdns_dphy *dphy);
	int (*validate)(struct cdns_dphy *dphy, enum phy_mode mode, int submode,
			union phy_configure_opts *opts);
	int (*configure)(struct cdns_dphy *dphy, union phy_configure_opts *opts);
	void (*set_psm_div)(struct cdns_dphy *dphy, u8 div);
	void (*set_clk_lane_cfg)(struct cdns_dphy *dphy,
				 enum cdns_dphy_clk_lane_cfg cfg);
	void (*set_pll_cfg)(struct cdns_dphy *dphy,
			    const struct cdns_dphy_cfg *cfg);
	unsigned long (*get_wakeup_time_ns)(struct cdns_dphy *dphy);
};

struct cdns_dphy_soc_data {
	bool has_hw_cmn_rstb;
};

struct cdns_dphy {
	struct cdns_dphy_cfg cfg;
	void __iomem *regs;
	struct device *dev;
	struct clk *psm_clk;
	struct clk *pll_ref_clk;
	const struct cdns_dphy_ops *ops;
	struct phy *phy;
};

struct cdns_dphy_driver_data {
	const struct cdns_dphy_ops *tx;
	const struct cdns_dphy_ops *rx;
};

struct cdns_dphy_band {
	unsigned int min_rate;
	unsigned int max_rate;
};

/* Order of bands is important since the index is the band number. */
struct cdns_dphy_band rx_bands[] = {
	{80, 100}, {100, 120}, {120, 160}, {160, 200}, {200, 240},
	{240, 280}, {280, 320}, {320, 360}, {360, 400}, {400, 480},
	{480, 560}, {560, 640}, {640, 720}, {720, 800}, {800, 880},
	{880, 1040}, {1040, 1200}, {1200, 1350}, {1350, 1500}, {1500, 1750},
	{1750, 2000}, {2000, 2250}, {2250, 2500}
};

int num_rx_bands = ARRAY_SIZE(rx_bands);

struct cdns_dphy_band tx_bands[] = {
	{80, 100}, {100, 120}, {120, 160}, {160, 200}, {200, 240},
	{240, 320}, {320, 390}, {390, 450}, {450, 510}, {510, 560},
	{560, 640}, {640, 690}, {690, 770}, {770, 870}, {870, 950},
	{950, 1000}, {1000, 1200}, {1200, 1400}, {1400, 1600}, {1600, 1800},
	{1800, 2000}, {2000, 2200}, {2200, 2500}
};

int num_tx_bands = ARRAY_SIZE(tx_bands);

static int cdns_dsi_get_dphy_pll_cfg(struct cdns_dphy *dphy,
				     struct cdns_dphy_cfg *cfg,
				     struct phy_configure_opts_mipi_dphy *opts,
				     unsigned int *dsi_hfp_ext)
{
	unsigned long pll_ref_hz = clk_get_rate(dphy->pll_ref_clk);
	u64 dlane_bps;

	memset(cfg, 0, sizeof(*cfg));

	if (pll_ref_hz < 9600000 || pll_ref_hz >= 150000000)
		return -EINVAL;
	else if (pll_ref_hz < 19200000)
		cfg->pll_ipdiv = 1;
	else if (pll_ref_hz < 38400000)
		cfg->pll_ipdiv = 2;
	else if (pll_ref_hz < 76800000)
		cfg->pll_ipdiv = 4;
	else
		cfg->pll_ipdiv = 8;

	dlane_bps = opts->hs_clk_rate;

	if (dlane_bps > 2500000000UL || dlane_bps < 160000000UL)
		return -EINVAL;
	else if (dlane_bps >= 1250000000)
		cfg->pll_opdiv = 1;
	else if (dlane_bps >= 630000000)
		cfg->pll_opdiv = 2;
	else if (dlane_bps >= 320000000)
		cfg->pll_opdiv = 4;
	else if (dlane_bps >= 160000000)
		cfg->pll_opdiv = 8;

	cfg->pll_fbdiv = DIV_ROUND_UP_ULL(dlane_bps * 2 * cfg->pll_opdiv *
					  cfg->pll_ipdiv,
					  pll_ref_hz);

	return 0;
}

static int cdns_dphy_setup_psm(struct cdns_dphy *dphy)
{
	unsigned long psm_clk_hz = clk_get_rate(dphy->psm_clk);
	unsigned long psm_div;

	if (!psm_clk_hz || psm_clk_hz > 100000000)
		return -EINVAL;

	psm_div = DIV_ROUND_CLOSEST(psm_clk_hz, 1000000);
	if (dphy->ops->set_psm_div)
		dphy->ops->set_psm_div(dphy, psm_div);

	return 0;
}

static void cdns_dphy_set_clk_lane_cfg(struct cdns_dphy *dphy,
				       enum cdns_dphy_clk_lane_cfg cfg)
{
	if (dphy->ops->set_clk_lane_cfg)
		dphy->ops->set_clk_lane_cfg(dphy, cfg);
}

static void cdns_dphy_set_pll_cfg(struct cdns_dphy *dphy,
				  const struct cdns_dphy_cfg *cfg)
{
	if (dphy->ops->set_pll_cfg)
		dphy->ops->set_pll_cfg(dphy, cfg);
}

static unsigned long cdns_dphy_get_wakeup_time_ns(struct cdns_dphy *dphy)
{
	return dphy->ops->get_wakeup_time_ns(dphy);
}

static unsigned long cdns_dphy_ref_get_wakeup_time_ns(struct cdns_dphy *dphy)
{
	/* Default wakeup time is 800 ns (in a simulated environment). */
	return 800;
}

static void cdns_dphy_ref_set_pll_cfg(struct cdns_dphy *dphy,
				      const struct cdns_dphy_cfg *cfg)
{
	u32 fbdiv_low, fbdiv_high;

	fbdiv_low = (cfg->pll_fbdiv / 4) - 2;
	fbdiv_high = cfg->pll_fbdiv - fbdiv_low - 2;

	writel(DPHY_CMN_IPDIV_FROM_REG | DPHY_CMN_OPDIV_FROM_REG |
	       DPHY_CMN_IPDIV(cfg->pll_ipdiv) |
	       DPHY_CMN_OPDIV(cfg->pll_opdiv),
	       dphy->regs + DPHY_CMN_OPIPDIV);
	writel(DPHY_CMN_FBDIV_FROM_REG |
	       DPHY_CMN_FBDIV_VAL(fbdiv_low, fbdiv_high),
	       dphy->regs + DPHY_CMN_FBDIV);
	writel(DPHY_CMN_PWM_HIGH(6) | DPHY_CMN_PWM_LOW(0x101) |
	       DPHY_CMN_PWM_DIV(0x8),
	       dphy->regs + DPHY_CMN_PWM);
}

static void cdns_dphy_ref_set_psm_div(struct cdns_dphy *dphy, u8 div)
{
	writel(DPHY_PSM_CFG_FROM_REG | DPHY_PSM_CLK_DIV(div),
	       dphy->regs + DPHY_PSM_CFG);
}

static int cdns_dphy_tx_config_from_opts(struct phy *phy,
					 struct phy_configure_opts_mipi_dphy *opts,
					 struct cdns_dphy_cfg *cfg)
{
	struct cdns_dphy *dphy = phy_get_drvdata(phy);
	unsigned int dsi_hfp_ext = 0;
	int ret;

	ret = phy_mipi_dphy_config_validate(opts);
	if (ret)
		return ret;

	ret = cdns_dsi_get_dphy_pll_cfg(dphy, cfg,
					opts, &dsi_hfp_ext);
	if (ret)
		return ret;

	opts->wakeup = cdns_dphy_get_wakeup_time_ns(dphy) / 1000;

	return 0;
}

static int cdns_dphy_tx_get_band_ctrl(unsigned long hs_clk_rate)
{
	unsigned int rate;
	int i;

	rate = hs_clk_rate / 1000000UL;

	if (rate < tx_bands[0].min_rate || rate >= tx_bands[num_tx_bands - 1].max_rate)
		return -EOPNOTSUPP;

	for (i = 0; i < num_tx_bands; i++) {
		if (rate >= tx_bands[i].min_rate && rate < tx_bands[i].max_rate)
			return i;
	}

	/* Unreachable. */
	WARN(1, "Reached unreachable code.");
	return -EINVAL;
}

static int cdns_dphy_tx_configure(struct cdns_dphy *dphy,
				  union phy_configure_opts *opts)
{
	struct cdns_dphy_cfg cfg = { 0 };
	int ret, band_ctrl;
	unsigned int reg;

	ret = cdns_dphy_tx_config_from_opts(dphy->phy, &opts->mipi_dphy, &cfg);
	if (ret)
		return ret;

	/*
	 * Configure the internal PSM clk divider so that the DPHY has a
	 * 1MHz clk (or something close).
	 */
	ret = cdns_dphy_setup_psm(dphy);
	if (ret)
		return ret;

	/*
	 * Configure attach clk lanes to data lanes: the DPHY has 2 clk lanes
	 * and 8 data lanes, each clk lane can be attache different set of
	 * data lanes. The 2 groups are named 'left' and 'right', so here we
	 * just say that we want the 'left' clk lane to drive the 'left' data
	 * lanes.
	 */
	cdns_dphy_set_clk_lane_cfg(dphy, DPHY_CLK_CFG_LEFT_DRIVES_LEFT);

	/*
	 * Configure the DPHY PLL that will be used to generate the TX byte
	 * clk.
	 */
	cdns_dphy_set_pll_cfg(dphy, &cfg);

	band_ctrl = cdns_dphy_tx_get_band_ctrl(opts->mipi_dphy.hs_clk_rate);
	if (band_ctrl < 0)
		return band_ctrl;

	reg = FIELD_PREP(DPHY_BAND_CFG_LEFT_BAND, band_ctrl) |
	      FIELD_PREP(DPHY_BAND_CFG_RIGHT_BAND, band_ctrl);
	writel(reg, dphy->regs + DPHY_BAND_CFG);

	return 0;
}

static int cdns_dphy_tx_validate(struct cdns_dphy *dphy, enum phy_mode mode,
				 int submode, union phy_configure_opts *opts)
{
	struct cdns_dphy_cfg cfg = { 0 };

	if (submode != PHY_MIPI_DPHY_SUBMODE_TX)
		return -EINVAL;

	return cdns_dphy_tx_config_from_opts(dphy->phy, &opts->mipi_dphy, &cfg);
}

static int cdns_dphy_tx_power_on(struct cdns_dphy *dphy)
{
	if (!dphy->psm_clk || !dphy->pll_ref_clk)
		return -EINVAL;

	clk_prepare_enable(dphy->psm_clk);
	clk_prepare_enable(dphy->pll_ref_clk);

	/* Start TX state machine. */
	writel(DPHY_CMN_SSM_EN | DPHY_CMN_TX_MODE_EN,
	       dphy->regs + DPHY_CMN_SSM);

	return 0;
}

static int cdns_dphy_tx_power_off(struct cdns_dphy *dphy)
{
	clk_disable_unprepare(dphy->pll_ref_clk);
	clk_disable_unprepare(dphy->psm_clk);

	return 0;
}

static unsigned long cdns_dphy_j721e_get_wakeup_time_ns(struct cdns_dphy *dphy)
{
	return 1000000;
}

static void cdns_dphy_j721e_set_pll_cfg(struct cdns_dphy *dphy,
					const struct cdns_dphy_cfg *cfg)
{
	u32 status;

	writel(DPHY_CMN_PWM_HIGH(6) | DPHY_CMN_PWM_LOW(0x101) |
	       DPHY_CMN_PWM_DIV(0x8),
	       dphy->regs + DPHY_CMN_PWM);

	writel((FIELD_PREP(DPHY_TX_J721E_WIZ_IPDIV, cfg->pll_ipdiv) |
		FIELD_PREP(DPHY_TX_J721E_WIZ_OPDIV, cfg->pll_opdiv) |
		FIELD_PREP(DPHY_TX_J721E_WIZ_FBDIV, cfg->pll_fbdiv)),
		dphy->regs + DPHY_TX_J721E_WIZ_PLL_CTRL);

	writel(DPHY_TX_J721E_WIZ_LANE_RSTB,
	       dphy->regs + DPHY_TX_J721E_WIZ_RST_CTRL);

	readl_poll_timeout(dphy->regs + DPHY_TX_J721E_WIZ_PLL_CTRL, status,
			   (status & DPHY_TX_WIZ_PLL_LOCK), 0, POLL_TIMEOUT_US);

	readl_poll_timeout(dphy->regs + DPHY_TX_J721E_WIZ_STATUS, status,
			   (status & DPHY_TX_WIZ_O_CMN_READY), 0,
			   POLL_TIMEOUT_US);
}

static void cdns_dphy_j721e_set_psm_div(struct cdns_dphy *dphy, u8 div)
{
	writel(div, dphy->regs + DPHY_TX_J721E_WIZ_PSM_FREQ);
}

static const struct cdns_dphy_ops tx_ref_dphy_ops = {
	.power_on = cdns_dphy_tx_power_on,
	.power_off = cdns_dphy_tx_power_off,
	.validate = cdns_dphy_tx_validate,
	.configure = cdns_dphy_tx_configure,
	.get_wakeup_time_ns = cdns_dphy_ref_get_wakeup_time_ns,
	.set_pll_cfg = cdns_dphy_ref_set_pll_cfg,
	.set_psm_div = cdns_dphy_ref_set_psm_div,
};

static const struct cdns_dphy_ops tx_j721e_dphy_ops = {
	.power_on = cdns_dphy_tx_power_on,
	.power_off = cdns_dphy_tx_power_off,
	.validate = cdns_dphy_tx_validate,
	.configure = cdns_dphy_tx_configure,
	.get_wakeup_time_ns = cdns_dphy_j721e_get_wakeup_time_ns,
	.set_pll_cfg = cdns_dphy_j721e_set_pll_cfg,
	.set_psm_div = cdns_dphy_j721e_set_psm_div,
};

static int cdns_dphy_rx_power_on(struct cdns_dphy *dphy)
{
	/* Start RX state machine. */
	writel(DPHY_CMN_SSM_EN | DPHY_CMN_RX_MODE_EN |
	       FIELD_PREP(DPHY_CMN_RX_BANDGAP_TIMER_MASK,
			  DPHY_CMN_RX_BANDGAP_TIMER),
	       dphy->regs + DPHY_CMN_SSM);

	return 0;
}

static int cdns_dphy_rx_power_off(struct cdns_dphy *dphy)
{
	writel(0, dphy->regs + DPHY_CMN_SSM);

	return 0;
}

static int cdns_dphy_rx_get_band_ctrl(unsigned long hs_clk_rate)
{
	unsigned int rate;
	int i;

	rate = hs_clk_rate / 1000000UL;
	/* Since CSI-2 clock is DDR, the bit rate is twice the clock rate. */
	rate *= 2;

	if (rate < rx_bands[0].min_rate || rate >= rx_bands[num_rx_bands - 1].max_rate)
		return -EOPNOTSUPP;

	for (i = 0; i < num_rx_bands; i++) {
		if (rate >= rx_bands[i].min_rate && rate < rx_bands[i].max_rate)
			return i;
	}

	/* Unreachable. */
	WARN(1, "Reached unreachable code.");
	return -EINVAL;
}

static int cdns_dphy_rx_wait_for_bit(void __iomem *addr, unsigned int bit)
{
	u32 val;

	return readl_relaxed_poll_timeout(addr, val, val & BIT(bit), 10,
					  DPHY_ISO_LANE_READY_TIMEOUT_MS * 1000);
}

static int cdns_dphy_rx_wait_lane_ready(struct cdns_dphy *dphy, int lanes)
{
	void __iomem *reg = dphy->regs;
	u32 data_lane_ctrl[] = {DPHY_ISO_DL_CTRL_L0, DPHY_ISO_DL_CTRL_L1,
				DPHY_ISO_DL_CTRL_L2, DPHY_ISO_DL_CTRL_L3};
	int ret, i;

	/* Data lanes. Minimum one lane is mandatory. */
	if (lanes < DPHY_LANES_MIN || lanes > DPHY_LANES_MAX)
		return -EINVAL;

	/* Clock lane */
	ret = cdns_dphy_rx_wait_for_bit(reg + DPHY_ISO_CL_CTRL_L,
					DPHY_ISO_LANE_READY_BIT);
	if (ret)
		return ret;

	for (i = 0; i < lanes; i++) {
		ret = cdns_dphy_rx_wait_for_bit(reg + data_lane_ctrl[i],
						DPHY_ISO_LANE_READY_BIT);
		if (ret)
			return ret;
	}

	return 0;
}

static struct cdns_dphy_soc_data j721e_soc_data = {
	.has_hw_cmn_rstb = true,
};

static const struct soc_device_attribute cdns_dphy_socinfo[] = {
	{
		.family = "J721E",
		.revision = "SR1.0",
		.data = &j721e_soc_data,
	},
	{/* sentinel */}
};

static int cdns_dphy_rx_configure(struct cdns_dphy *dphy,
				  union phy_configure_opts *opts)
{
	const struct soc_device_attribute *soc;
	const struct cdns_dphy_soc_data *soc_data = NULL;
	unsigned int reg;
	int band_ctrl, ret;

	soc = soc_device_match(cdns_dphy_socinfo);
	if (soc && soc->data)
		soc_data = soc->data;
	if (!soc || (soc_data && !soc_data->has_hw_cmn_rstb)) {
		reg = DPHY_LANE_RESET_CMN_EN;
		writel(reg, dphy->regs + DPHY_LANE);
	}

	band_ctrl = cdns_dphy_rx_get_band_ctrl(opts->mipi_dphy.hs_clk_rate);
	if (band_ctrl < 0)
		return band_ctrl;

	reg = FIELD_PREP(DPHY_BAND_CFG_LEFT_BAND, band_ctrl) |
	      FIELD_PREP(DPHY_BAND_CFG_RIGHT_BAND, band_ctrl);
	writel(reg, dphy->regs + DPHY_BAND_CFG);

	/*
	 * Set the required power island phase 2 time. This is mandated by DPHY
	 * specs.
	 */
	reg = DPHY_POWER_ISLAND_EN_DATA_VAL;
	writel(reg, dphy->regs + DPHY_POWER_ISLAND_EN_DATA);
	reg = DPHY_POWER_ISLAND_EN_CLK_VAL;
	writel(reg, dphy->regs + DPHY_POWER_ISLAND_EN_CLK);

	ret = cdns_dphy_rx_wait_lane_ready(dphy, opts->mipi_dphy.lanes);
	if (ret) {
		dev_err(dphy->dev, "DPHY wait for lane ready timeout\n");
		return ret;
	}

	return 0;
}

static int cdns_dphy_rx_validate(struct cdns_dphy *dphy, enum phy_mode mode,
				 int submode, union phy_configure_opts *opts)
{
	int ret;

	if (submode != PHY_MIPI_DPHY_SUBMODE_RX)
		return -EINVAL;

	ret = cdns_dphy_rx_get_band_ctrl(opts->mipi_dphy.hs_clk_rate);
	if (ret < 0)
		return ret;

	return phy_mipi_dphy_config_validate(&opts->mipi_dphy);
}

static const struct cdns_dphy_ops rx_ref_dphy_ops = {
	.power_on = cdns_dphy_rx_power_on,
	.power_off = cdns_dphy_rx_power_off,
	.configure = cdns_dphy_rx_configure,
	.validate = cdns_dphy_rx_validate,
};

/*
 * This is the reference implementation of DPHY hooks. Specific integration of
 * this IP may have to re-implement some of them depending on how they decided
 * to wire things in the SoC.
 */
static const struct cdns_dphy_driver_data ref_dphy_ops = {
	.tx = &tx_ref_dphy_ops,
	.rx = &rx_ref_dphy_ops,
};

static const struct cdns_dphy_driver_data j721e_dphy_ops = {
	.tx = &tx_j721e_dphy_ops,
	.rx = &rx_ref_dphy_ops,
};

static int cdns_dphy_validate(struct phy *phy, enum phy_mode mode, int submode,
			      union phy_configure_opts *opts)
{
	struct cdns_dphy *dphy = phy_get_drvdata(phy);

	if (mode != PHY_MODE_MIPI_DPHY)
		return -EINVAL;

	if (dphy->ops->validate)
		return dphy->ops->validate(dphy, mode, submode, opts);

	return 0;
}

static int cdns_dphy_power_on(struct phy *phy)
{
	struct cdns_dphy *dphy = phy_get_drvdata(phy);

	if (dphy->ops->power_on)
		return dphy->ops->power_on(dphy);

	return 0;
}

static int cdns_dphy_power_off(struct phy *phy)
{
	struct cdns_dphy *dphy = phy_get_drvdata(phy);

	if (dphy->ops->power_off)
		return dphy->ops->power_off(dphy);

	return 0;
}

static int cdns_dphy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct cdns_dphy *dphy = phy_get_drvdata(phy);

	if (dphy->ops->configure)
		return dphy->ops->configure(dphy, opts);

	return 0;
}

static int cdns_dphy_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct cdns_dphy *dphy = phy_get_drvdata(phy);
	const struct cdns_dphy_driver_data *ddata;

	ddata = of_device_get_match_data(dphy->dev);
	if (!ddata)
		return -EINVAL;

	if (mode != PHY_MODE_MIPI_DPHY)
		return -EINVAL;

	if (submode == PHY_MIPI_DPHY_SUBMODE_TX) {
		if (!ddata->tx)
			return -EOPNOTSUPP;

		dphy->ops = ddata->tx;
	} else if (submode == PHY_MIPI_DPHY_SUBMODE_RX) {
		if (!ddata->rx)
			return -EOPNOTSUPP;

		dphy->ops = ddata->rx;
	} else {
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct phy_ops cdns_dphy_ops = {
	.configure	= cdns_dphy_configure,
	.validate	= cdns_dphy_validate,
	.power_on	= cdns_dphy_power_on,
	.power_off	= cdns_dphy_power_off,
	.set_mode	= cdns_dphy_set_mode,
};

static int cdns_dphy_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct cdns_dphy *dphy;
	const struct cdns_dphy_driver_data *ddata;
	int ret;

	dphy = devm_kzalloc(&pdev->dev, sizeof(*dphy), GFP_KERNEL);
	if (!dphy)
		return -ENOMEM;
	dev_set_drvdata(&pdev->dev, dphy);
	dphy->dev = &pdev->dev;

	ddata = of_device_get_match_data(&pdev->dev);
	if (!ddata)
		return -EINVAL;

	dphy->ops = ddata->tx;
	if (!dphy->ops)
		return -EINVAL;

	dphy->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dphy->regs))
		return PTR_ERR(dphy->regs);

	dphy->psm_clk = devm_clk_get_optional(dphy->dev, "psm");
	if (IS_ERR(dphy->psm_clk))
		return PTR_ERR(dphy->psm_clk);

	dphy->pll_ref_clk = devm_clk_get_optional(dphy->dev, "pll_ref");
	if (IS_ERR(dphy->pll_ref_clk))
		return PTR_ERR(dphy->pll_ref_clk);

	if (dphy->ops->probe) {
		ret = dphy->ops->probe(dphy);
		if (ret)
			return ret;
	}

	dphy->phy = devm_phy_create(&pdev->dev, NULL, &cdns_dphy_ops);
	if (IS_ERR(dphy->phy)) {
		dev_err(&pdev->dev, "failed to create PHY\n");
		if (dphy->ops->remove)
			dphy->ops->remove(dphy);
		return PTR_ERR(dphy->phy);
	}

	phy_set_drvdata(dphy->phy, dphy);
	phy_provider = devm_of_phy_provider_register(&pdev->dev,
						     of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static int cdns_dphy_remove(struct platform_device *pdev)
{
	struct cdns_dphy *dphy = dev_get_drvdata(&pdev->dev);

	if (dphy->ops->remove)
		dphy->ops->remove(dphy);

	return 0;
}

static const struct of_device_id cdns_dphy_of_match[] = {
	{ .compatible = "cdns,dphy", .data = &ref_dphy_ops },
	{ .compatible = "ti,j721e-dphy", .data = &j721e_dphy_ops },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, cdns_dphy_of_match);

static struct platform_driver cdns_dphy_platform_driver = {
	.probe		= cdns_dphy_probe,
	.remove		= cdns_dphy_remove,
	.driver		= {
		.name		= "cdns-mipi-dphy",
		.of_match_table	= cdns_dphy_of_match,
	},
};
module_platform_driver(cdns_dphy_platform_driver);

MODULE_AUTHOR("Maxime Ripard <maxime.ripard@bootlin.com>");
MODULE_AUTHOR("Pratyush Yadav <p.yadav@ti.com>");
MODULE_DESCRIPTION("Cadence MIPI D-PHY Driver");
MODULE_LICENSE("GPL");
