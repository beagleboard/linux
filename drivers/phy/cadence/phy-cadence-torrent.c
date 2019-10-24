// SPDX-License-Identifier: GPL-2.0-only
/*
 * Cadence Torrent SD0801 PHY driver.
 *
 * Copyright 2018 Cadence Design Systems, Inc.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#define REF_CLK_19_2MHz		19200000
#define REF_CLK_25MHz		25000000

#define DEFAULT_NUM_LANES	4
#define MAX_NUM_LANES		4
#define DEFAULT_MAX_BIT_RATE	8100 /* in Mbps */

#define POLL_TIMEOUT_US		5000
#define LANE_MASK		0x7

/*
 * register offsets from DPTX PHY register block base (i.e MHDP
 * register base + 0x30a00)
 */
#define PHY_AUX_CONFIG			0x00
#define PHY_AUX_CTRL			0x04
#define PHY_RESET			0x20
#define PMA_TX_ELEC_IDLE_MASK		0xF0U
#define PMA_TX_ELEC_IDLE_SHIFT		4
#define PHY_L00_RESET_N_MASK		0x01U
#define PHY_PMA_XCVR_PLLCLK_EN		0x24
#define PHY_PMA_XCVR_PLLCLK_EN_ACK	0x28
#define PHY_PMA_XCVR_POWER_STATE_REQ	0x2c
#define PHY_POWER_STATE_LN_0	0x0000
#define PHY_POWER_STATE_LN_1	0x0008
#define PHY_POWER_STATE_LN_2	0x0010
#define PHY_POWER_STATE_LN_3	0x0018
#define PMA_XCVR_POWER_STATE_REQ_LN_MASK	0x3FU
#define PHY_PMA_XCVR_POWER_STATE_ACK	0x30
#define PHY_PMA_CMN_READY		0x34
#define PHY_PMA_XCVR_TX_VMARGIN		0x38
#define PHY_PMA_XCVR_TX_DEEMPH		0x3c

/*
 * register offsets from SD0801 PHY register block base (i.e MHDP
 * register base + 0x500000)
 */
#define CMN_SSM_BANDGAP_TMR		(0x00084/2)
#define CMN_SSM_BIAS_TMR		(0x00088/2)
#define CMN_PLLSM0_PLLPRE_TMR		(0x000a8/2)
#define CMN_PLLSM0_PLLLOCK_TMR		(0x000b0/2)
#define CMN_PLLSM1_PLLPRE_TMR		(0x000c8/2)
#define CMN_PLLSM1_PLLLOCK_TMR		(0x000d0/2)
#define CMN_BGCAL_INIT_TMR		(0x00190/2)
#define CMN_BGCAL_ITER_TMR		(0x00194/2)
#define CMN_IBCAL_INIT_TMR		(0x001d0/2)
#define CMN_PLL0_VCOCAL_TCTRL		(0x00208/2)
#define CMN_PLL0_VCOCAL_INIT_TMR	(0x00210/2)
#define CMN_PLL0_VCOCAL_ITER_TMR	(0x00214/2)
#define CMN_PLL0_VCOCAL_REFTIM_START	(0x00218/2)
#define CMN_PLL0_VCOCAL_PLLCNT_START	(0x00220/2)
#define CMN_PLL0_INTDIV_M0		(0x00240/2)
#define CMN_PLL0_FRACDIVL_M0		(0x00244/2)
#define CMN_PLL0_FRACDIVH_M0		(0x00248/2)
#define CMN_PLL0_HIGH_THR_M0		(0x0024c/2)
#define CMN_PLL0_DSM_DIAG_M0		(0x00250/2)
#define CMN_PLL0_SS_CTRL1_M0		(0x00260/2)
#define CMN_PLL0_SS_CTRL2_M0            (0x00264/2)
#define CMN_PLL0_SS_CTRL3_M0            (0x00268/2)
#define CMN_PLL0_SS_CTRL4_M0            (0x0026C/2)
#define CMN_PLL0_LOCK_REFCNT_START      (0x00270/2)
#define CMN_PLL0_LOCK_PLLCNT_START	(0x00278/2)
#define CMN_PLL0_LOCK_PLLCNT_THR        (0x0027C/2)
#define CMN_PLL1_VCOCAL_TCTRL		(0x00308/2)
#define CMN_PLL1_VCOCAL_INIT_TMR	(0x00310/2)
#define CMN_PLL1_VCOCAL_ITER_TMR	(0x00314/2)
#define CMN_PLL1_VCOCAL_REFTIM_START	(0x00318/2)
#define CMN_PLL1_VCOCAL_PLLCNT_START	(0x00320/2)
#define CMN_PLL1_INTDIV_M0		(0x00340/2)
#define CMN_PLL1_FRACDIVL_M0		(0x00344/2)
#define CMN_PLL1_FRACDIVH_M0		(0x00348/2)
#define CMN_PLL1_HIGH_THR_M0		(0x0034c/2)
#define CMN_PLL1_DSM_DIAG_M0		(0x00350/2)
#define CMN_PLL1_SS_CTRL1_M0		(0x00360/2)
#define CMN_PLL1_SS_CTRL2_M0            (0x00364/2)
#define CMN_PLL1_SS_CTRL3_M0            (0x00368/2)
#define CMN_PLL1_SS_CTRL4_M0            (0x0036C/2)
#define CMN_PLL1_LOCK_REFCNT_START      (0x00370/2)
#define CMN_PLL1_LOCK_PLLCNT_START	(0x00378/2)
#define CMN_PLL1_LOCK_PLLCNT_THR        (0x0037C/2)
#define CMN_TXPUCAL_INIT_TMR		(0x00410/2)
#define CMN_TXPUCAL_ITER_TMR		(0x00414/2)
#define CMN_TXPDCAL_INIT_TMR		(0x00430/2)
#define CMN_TXPDCAL_ITER_TMR		(0x00434/2)
#define CMN_RXCAL_INIT_TMR		(0x00450/2)
#define CMN_RXCAL_ITER_TMR		(0x00454/2)
#define CMN_SD_CAL_INIT_TMR		(0x00490/2)
#define CMN_SD_CAL_ITER_TMR		(0x00494/2)
#define CMN_SD_CAL_REFTIM_START		(0x00498/2)
#define CMN_SD_CAL_PLLCNT_START		(0x004a0/2)
#define CMN_PDIAG_PLL0_CTRL_M0		(0x00680/2)
#define CMN_PDIAG_PLL0_CLK_SEL_M0	(0x00684/2)
#define CMN_PDIAG_PLL0_CP_PADJ_M0	(0x00690/2)
#define CMN_PDIAG_PLL0_CP_IADJ_M0	(0x00694/2)
#define CMN_PDIAG_PLL0_FILT_PADJ_M0	(0x00698/2)
#define CMN_PDIAG_PLL0_CP_PADJ_M1	(0x006d0/2)
#define CMN_PDIAG_PLL0_CP_IADJ_M1	(0x006d4/2)
#define CMN_PDIAG_PLL1_CTRL_M0		(0x00700/2)
#define CMN_PDIAG_PLL1_CLK_SEL_M0	(0x00704/2)
#define CMN_PDIAG_PLL1_CP_PADJ_M0	(0x00710/2)
#define CMN_PDIAG_PLL1_CP_IADJ_M0	(0x00714/2)
#define CMN_PDIAG_PLL1_FILT_PADJ_M0	(0x00718/2)
#define CMN_PDIAG_PLL1_CP_PADJ_M1	(0x00750/2)
#define CMN_PDIAG_PLL1_CP_IADJ_M1	(0x00754/2)

#define XCVR_DIAG_PLLDRC_CTRL(j)		(0x4000 + 0x01ca + (j) * 0x400)
#define XCVR_DIAG_HSCLK_SEL(j)			(0x4000 + 0x01cc + (j) * 0x400)
#define XCVR_DIAG_HSCLK_DIV(j)			(0x4000 + 0x01ce + (j) * 0x400)
#define XCVR_DIAG_BIDI_CTRL(j)			(0x4000 + 0x01d4 + (j) * 0x400)
#define TX_PSC_A0(j)				(0x4000 + 0x0200 + (j) * 0x400)
#define TX_PSC_A1(j)				(0x4000 + 0x0202 + (j) * 0x400)
#define TX_PSC_A2(j)				(0x4000 + 0x0204 + (j) * 0x400)
#define TX_PSC_A3(j)				(0x4000 + 0x0206 + (j) * 0x400)

#define TX_RCVDET_ST_TMR(j)			(0x4000 + 0x0246 + (j) * 0x400)

#define RX_PSC_A0(j)				(0x8000 + 0x0000 + (j) * 0x400)
#define RX_PSC_A1(j)				(0x8000 + 0x0002 + (j) * 0x400)
#define RX_PSC_A2(j)				(0x8000 + 0x0004 + (j) * 0x400)
#define RX_PSC_A3(j)				(0x8000 + 0x0006 + (j) * 0x400)

#define TX_TXCC_CTRL(j)				(0x4000 + 0x80 + (j) * 0x400)
#define TX_DIAG_ACYA(j)				(0x4000 + 0x3ce + (j) * 0x400)
#define DRV_DIAG_TX_DRV(j)			(0x4000 + 0x18c + (j) * 0x400)
#define TX_TXCC_MGNFS_MULT_000(j)		(0x4000 + 0xa0 + (j) * 0x400)
#define TX_TXCC_CPOST_MULT_00(j)		(0x4000 + 0x98 + (j) * 0x400)

#define PHY_PLL_CFG				(0xc000 + 0x001c) // XXX is this correct?

#define PHY_PMA_CMN_CTRL2			0xe002
#define PHY_PMA_PLL_RAW_CTRL			0xe006


#define TX_DIAG_ACYA_HBDC_MASK 0x0001U

#define RX_PSC_CAL(j)				(0x8000 + 0x000c + (j) * 0x400)

#define RX_REE_GCSM1_CTRL(j)			(0x8000 + 0x0210 + (j) * 0x400)
#define RX_REE_GCSM2_CTRL(j)			(0x8000 + 0x0220 + (j) * 0x400)
#define RX_REE_PERGCSM_CTRL(j)			(0x8000 + 0x0230 + (j) * 0x400)

struct cdns_torrent_phy {
	void __iomem *base;	/* DPTX registers base */
	void __iomem *sd_base;	/* SD0801 registers base */
	u32 num_lanes; /* Number of lanes to use */
	u32 max_bit_rate; /* Maximum link bit rate to use (in Mbps) */
	struct reset_control *phy_rst;
	struct device *dev;
	struct clk *clk;
	unsigned long ref_clk_rate;
};

enum phy_powerstate {
	POWERSTATE_A0 = 0,
	// Powerstate A1 is unused
	POWERSTATE_A2 = 2,
	POWERSTATE_A3 = 3,
};

static int cdns_torrent_dp_init(struct phy *phy);
static int cdns_torrent_dp_exit(struct phy *phy);
static int cdns_torrent_dp_run(struct cdns_torrent_phy *cdns_phy);
static int cdns_torrent_dp_wait_pma_cmn_ready(struct cdns_torrent_phy *cdns_phy);
static void cdns_torrent_dp_pma_cfg(struct cdns_torrent_phy *cdns_phy);
static void cdns_torrent_dp_pma_cmn_cfg_19_2mhz(struct cdns_torrent_phy *cdns_phy);
static void cdns_torrent_dp_pma_cmn_vco_cfg_19_2mhz(struct cdns_torrent_phy *cdns_phy, u32 rate, bool ssc);
static void cdns_torrent_dp_pma_cmn_cfg_25mhz(struct cdns_torrent_phy *cdns_phy);
static void cdns_torrent_dp_pma_cmn_vco_cfg_25mhz(struct cdns_torrent_phy *cdns_phy, u32 rate, bool ssc);
static void cdns_torrent_dp_pma_lane_cfg(struct cdns_torrent_phy *cdns_phy,
				     unsigned int lane);
static void cdns_torrent_dp_pma_cmn_rate(struct cdns_torrent_phy *cdns_phy,
				     u32 rate, u32 lanes);
static void cdns_dp_phy_write_field(struct cdns_torrent_phy *cdns_phy,
				    unsigned int offset,
				    unsigned char start_bit,
				    unsigned char num_bits,
				    unsigned int val);
static int cdns_torrent_dp_configure(struct phy *phy, union phy_configure_opts *opts);
static void cdns_torrent_dp_set_a0_pll(struct cdns_torrent_phy *cdns_phy, u32 num_lanes);
static int cdns_torrent_dp_set_power_state(struct cdns_torrent_phy *cdns_phy,
				       u32 num_lanes,
				       enum phy_powerstate powerstate);

static int cdns_torrent_phy_on(struct phy *gphy);
static int cdns_torrent_phy_off(struct phy *gphy);

static const struct phy_ops cdns_torrent_phy_ops = {
	.init		= cdns_torrent_dp_init,
	.exit		= cdns_torrent_dp_exit,
	.configure	= cdns_torrent_dp_configure,
	.power_on	= cdns_torrent_phy_on,
	.power_off	= cdns_torrent_phy_off,
	.owner		= THIS_MODULE,
};

//#define DEBUG_DP_WRITES
//#define DEBUG_PHY_WRITES

/* PHY mmr access functions */

#ifdef DEBUG_PHY_WRITES
#define cdns_dp_phy_write_phy(cdns_phy, offset, val)  \
({ \
	printk("PHY WR PHY %s (0x%08x) = 0x%08x [%s:%d]\n", \
		#offset, offset, val, __func__, __LINE__); \
	_cdns_dp_phy_write_phy(cdns_phy, offset, val); \
})

static void _cdns_dp_phy_write_phy(struct cdns_torrent_phy *cdns_phy, u32 offset, u16 val)
{
	writew(val, cdns_phy->sd_base + offset);
}
#else
static void cdns_dp_phy_write_phy(struct cdns_torrent_phy *cdns_phy, u32 offset, u16 val)
{
	writew(val, cdns_phy->sd_base + offset);
}
#endif

static u16 cdns_dp_phy_read_phy(struct cdns_torrent_phy *cdns_phy, u32 offset)
{
	return readw(cdns_phy->sd_base + offset);
}

#define cdns_phy_read_poll_timeout(offset, val, cond, delay_us, timeout_us) \
	readw_poll_timeout(cdns_phy->sd_base + (offset), val, cond, delay_us, timeout_us)

/* DPTX mmr access functions */

#ifdef DEBUG_DP_WRITES
#define cdns_dp_phy_write_dp(cdns_phy, offset, val)  \
({ \
	printk("PHY WR DP %s (0x%08x) = 0x%08x [%s:%d]\n", \
		#offset, offset, val, __func__, __LINE__); \
	_cdns_dp_phy_write_dp(cdns_phy, offset, val); \
})

static void _cdns_dp_phy_write_dp(struct cdns_torrent_phy *cdns_phy, u32 offset, u16 val)
{
	writel(val, cdns_phy->base + offset);
}
#else
static void cdns_dp_phy_write_dp(struct cdns_torrent_phy *cdns_phy, u32 offset, u16 val)
{
	writel(val, cdns_phy->base + offset);
}
#endif

static u32 cdns_dp_phy_read_dp(struct cdns_torrent_phy *cdns_phy, u32 offset)
{
	return readl(cdns_phy->base + offset);
}

#define cdns_phy_read_dp_poll_timeout(cdns_phy, offset, val, cond, delay_us, timeout_us) \
	readl_poll_timeout(cdns_phy->base + (offset), val, cond, delay_us, timeout_us)


/*
 * Structure used to store values of PHY registers for voltage-related
 * coefficients, for particular voltage swing and re-emphasis level. Values
 * are shared across all physical lanes.
 */
struct coefficients {
	/** Value of DRV_DIAG_TX_DRV register to use */
	uint16_t diag_tx_drv;
	/** Value of TX_TXCC_MGNFS_MULT_000 register to use */
	uint16_t mgnfs_mult;
	/** Value of TX_TXCC_CPOST_MULT_00 register to use */
	uint16_t cpost_mult;
};

/*
 * Array consists of values of voltage-related registers for sd0801 PHY. A value
 * of 0xFFFF is a placeholder for invalid combination, and will never be used.
 */
static const struct coefficients voltage_coeffs[4][4] = {
	// voltage swing 0, pre-emphasis 0->3
	{	{.diag_tx_drv = 0x0003, .mgnfs_mult = 0x002A, .cpost_mult = 0x0000},
		{.diag_tx_drv = 0x0003, .mgnfs_mult = 0x001F, .cpost_mult = 0x0014},
		{.diag_tx_drv = 0x0003, .mgnfs_mult = 0x0012, .cpost_mult = 0x0020},
		{.diag_tx_drv = 0x0003, .mgnfs_mult = 0x0000, .cpost_mult = 0x002A}
	},

	// voltage swing 1, pre-emphasis 0->3
	{	{.diag_tx_drv = 0x0003, .mgnfs_mult = 0x001F, .cpost_mult = 0x0000},
		{.diag_tx_drv = 0x0003, .mgnfs_mult = 0x0013, .cpost_mult = 0x0012},
		{.diag_tx_drv = 0x0003, .mgnfs_mult = 0x0000, .cpost_mult = 0x001F},
		{.diag_tx_drv = 0xFFFF, .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF}
	},

	// voltage swing 2, pre-emphasis 0->3
	{	{.diag_tx_drv = 0x0003, .mgnfs_mult = 0x0013, .cpost_mult = 0x0000},
		{.diag_tx_drv = 0x0003, .mgnfs_mult = 0x0000, .cpost_mult = 0x0013},
		{.diag_tx_drv = 0xFFFF, .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF},
		{.diag_tx_drv = 0xFFFF, .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF}
	},

	// voltage swing 3, pre-emphasis 0->3
	{	{.diag_tx_drv = 0x0003, .mgnfs_mult = 0x0000, .cpost_mult = 0x0000},
		{.diag_tx_drv = 0xFFFF, .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF},
		{.diag_tx_drv = 0xFFFF, .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF},
		{.diag_tx_drv = 0xFFFF, .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF}
	}
};

static int cdns_torrent_dp_init(struct phy *phy)
{
	unsigned char lane_bits;
	int r;

	struct cdns_torrent_phy *cdns_phy = phy_get_drvdata(phy);

	r = clk_prepare_enable(cdns_phy->clk);
	if (r) {
		dev_err(cdns_phy->dev, "Failed to prepare ref clock\n");
		return r;
	}

	cdns_phy->ref_clk_rate = clk_get_rate(cdns_phy->clk);
	if (!(cdns_phy->ref_clk_rate)) {
		dev_err(cdns_phy->dev, "Failed to get ref clock rate\n");
		clk_disable_unprepare(cdns_phy->clk);
		return -EINVAL;
	}

	switch (cdns_phy->ref_clk_rate) {
	case REF_CLK_19_2MHz:
	case REF_CLK_25MHz:
		/* Valid Ref Clock Rate */
		break;
	default:
		dev_err(cdns_phy->dev, "Unsupported Ref Clock Rate\n");
		return -EINVAL;
	}

	cdns_dp_phy_write_dp(cdns_phy, PHY_AUX_CTRL, 0x0003); /* enable AUX */

	/* PHY PMA registers configuration function */
	cdns_torrent_dp_pma_cfg(cdns_phy);

	/*
	 * Set lines power state to A0
	 * Set lines pll clk enable to 0
	 */
	cdns_torrent_dp_set_a0_pll(cdns_phy, cdns_phy->num_lanes);

	/*
	 * release phy_l0*_reset_n and pma_tx_elec_idle_ln_* based on
	 * used lanes
	 */
	lane_bits = (1 << cdns_phy->num_lanes) - 1;
	cdns_dp_phy_write_dp(cdns_phy, PHY_RESET, ((0xF & ~lane_bits) << 4) | (0xF & lane_bits));

	/* release pma_xcvr_pllclk_en_ln_*, only for the master lane */
	cdns_dp_phy_write_dp(cdns_phy, PHY_PMA_XCVR_PLLCLK_EN, 0x0001);

	/* PHY PMA registers configuration functions */
	/* Initialize PHY with max supported link rate, without SSC. */
	if (cdns_phy->ref_clk_rate ==  REF_CLK_19_2MHz)
		cdns_torrent_dp_pma_cmn_vco_cfg_19_2mhz(cdns_phy, cdns_phy->max_bit_rate, false);
	else if (cdns_phy->ref_clk_rate == REF_CLK_25MHz)
		cdns_torrent_dp_pma_cmn_vco_cfg_25mhz(cdns_phy, cdns_phy->max_bit_rate, false);
	cdns_torrent_dp_pma_cmn_rate(cdns_phy, cdns_phy->max_bit_rate, cdns_phy->num_lanes);

	/* take out of reset */
	cdns_dp_phy_write_field(cdns_phy, PHY_RESET, 8, 1, 1);
	cdns_torrent_phy_on(phy);
	r = cdns_torrent_dp_wait_pma_cmn_ready(cdns_phy);
	if (r)
		return r;

	r = cdns_torrent_dp_run(cdns_phy);

	return r;
}

static int cdns_torrent_dp_exit(struct phy *phy)
{
	struct cdns_torrent_phy *cdns_phy = phy_get_drvdata(phy);

	clk_disable_unprepare(cdns_phy->clk);
	return 0;
}

static int cdns_torrent_dp_wait_pma_cmn_ready(struct cdns_torrent_phy *cdns_phy)
{
	unsigned int reg;
	int ret;

	ret = cdns_phy_read_dp_poll_timeout(cdns_phy, PHY_PMA_CMN_READY, reg,
					    reg & 1, 0, POLL_TIMEOUT_US);
	if (ret == -ETIMEDOUT) {
		dev_err(cdns_phy->dev,
			"timeout waiting for PMA common ready\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static void cdns_torrent_dp_pma_cfg(struct cdns_torrent_phy *cdns_phy)
{
	unsigned int i;

	if (cdns_phy->ref_clk_rate ==  REF_CLK_19_2MHz)
		/* PMA common configuration 19.2MHz */
		cdns_torrent_dp_pma_cmn_cfg_19_2mhz(cdns_phy);
	else if (cdns_phy->ref_clk_rate == REF_CLK_25MHz)
		/* PMA common configuration 25MHz */
		cdns_torrent_dp_pma_cmn_cfg_25mhz(cdns_phy);

	/* PMA lane configuration to deal with multi-link operation */
	for (i = 0; i < cdns_phy->num_lanes; i++)
		cdns_torrent_dp_pma_lane_cfg(cdns_phy, i);
}

static void cdns_torrent_dp_pma_cmn_cfg_19_2mhz(struct cdns_torrent_phy *cdns_phy)
{
	/* refclock registers - assumes 19.2 MHz refclock */
	cdns_dp_phy_write_phy(cdns_phy, CMN_SSM_BIAS_TMR, 0x0014);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLLSM0_PLLPRE_TMR, 0x0027);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLLSM0_PLLLOCK_TMR, 0x00A1);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLLSM1_PLLPRE_TMR, 0x0027);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLLSM1_PLLLOCK_TMR, 0x00A1);
	cdns_dp_phy_write_phy(cdns_phy, CMN_BGCAL_INIT_TMR, 0x0060);
	cdns_dp_phy_write_phy(cdns_phy, CMN_BGCAL_ITER_TMR, 0x0060);
	cdns_dp_phy_write_phy(cdns_phy, CMN_IBCAL_INIT_TMR, 0x0014);
	cdns_dp_phy_write_phy(cdns_phy, CMN_TXPUCAL_INIT_TMR, 0x0018);
	cdns_dp_phy_write_phy(cdns_phy, CMN_TXPUCAL_ITER_TMR, 0x0005);
	cdns_dp_phy_write_phy(cdns_phy, CMN_TXPDCAL_INIT_TMR, 0x0018);
	cdns_dp_phy_write_phy(cdns_phy, CMN_TXPDCAL_ITER_TMR, 0x0005);
	cdns_dp_phy_write_phy(cdns_phy, CMN_RXCAL_INIT_TMR, 0x0240);
	cdns_dp_phy_write_phy(cdns_phy, CMN_RXCAL_ITER_TMR, 0x0005);
	cdns_dp_phy_write_phy(cdns_phy, CMN_SD_CAL_INIT_TMR, 0x0002);
	cdns_dp_phy_write_phy(cdns_phy, CMN_SD_CAL_ITER_TMR, 0x0002);
	cdns_dp_phy_write_phy(cdns_phy, CMN_SD_CAL_REFTIM_START, 0x000B);
	cdns_dp_phy_write_phy(cdns_phy, CMN_SD_CAL_PLLCNT_START, 0x0137);

	/* PLL registers */
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_CP_PADJ_M0, 0x0509);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_CP_IADJ_M0, 0x0F00);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_FILT_PADJ_M0, 0x0F08);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_DSM_DIAG_M0, 0x0004);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL1_CP_PADJ_M0, 0x0509);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL1_CP_IADJ_M0, 0x0F00);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL1_FILT_PADJ_M0, 0x0F08);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_DSM_DIAG_M0, 0x0004);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_INIT_TMR, 0x00C0);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_ITER_TMR, 0x0004);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_VCOCAL_INIT_TMR, 0x00C0);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_VCOCAL_ITER_TMR, 0x0004);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_REFTIM_START, 0x0260);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_TCTRL, 0x0003);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_VCOCAL_REFTIM_START, 0x0260);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_VCOCAL_TCTRL, 0x0003);
}

/*
 * Set registers responsible for enabling and configuring SSC, with second and
 * third register values provided by parameters.
 */
static void cdns_torrent_dp_enable_ssc_19_2mhz(struct cdns_torrent_phy *cdns_phy,
					   u32 ctrl2_val, u32 ctrl3_val)
{
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL1_M0, 0x0001);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL1_M0, ctrl2_val);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL1_M0, ctrl3_val);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL4_M0, 0x0003);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL1_M0, 0x0001);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL1_M0, ctrl2_val);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL1_M0, ctrl3_val);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL4_M0, 0x0003);
}

static void cdns_torrent_dp_pma_cmn_vco_cfg_19_2mhz(struct cdns_torrent_phy *cdns_phy,
						u32 rate, bool ssc)
{

	/* Assumes 19.2 MHz refclock */
	switch (rate) {
	/* Setting VCO for 10.8GHz */
	case 2700:
	case 5400:
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_INTDIV_M0, 0x0119);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVL_M0, 0x4000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_HIGH_THR_M0, 0x00BC);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_CTRL_M0, 0x0012);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_INTDIV_M0, 0x0119);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVL_M0, 0x4000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_HIGH_THR_M0, 0x00BC);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL1_CTRL_M0, 0x0012);
		if (ssc)
			cdns_torrent_dp_enable_ssc_19_2mhz(cdns_phy, 0x033A, 0x006A);
		break;
	/* Setting VCO for 9.72GHz */
	case 1620:
	case 2430:
	case 3240:
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_INTDIV_M0, 0x01FA);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVL_M0, 0x4000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_HIGH_THR_M0, 0x0152);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_INTDIV_M0, 0x01FA);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVL_M0, 0x4000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_HIGH_THR_M0, 0x0152);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL1_CTRL_M0, 0x0002);
		if (ssc)
			cdns_torrent_dp_enable_ssc_19_2mhz(cdns_phy, 0x05DD, 0x0069);
		break;
	/* Setting VCO for 8.64GHz */
	case 2160:
	case 4320:
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_INTDIV_M0, 0x01C2);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVL_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_HIGH_THR_M0, 0x012C);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_INTDIV_M0, 0x01C2);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVL_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_HIGH_THR_M0, 0x012C);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL1_CTRL_M0, 0x0002);
		if (ssc)
			cdns_torrent_dp_enable_ssc_19_2mhz(cdns_phy, 0x0536, 0x0069);
		break;
	/* Setting VCO for 8.1GHz */
	case 8100:
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_INTDIV_M0, 0x01A5);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVL_M0, 0xE000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_HIGH_THR_M0, 0x011A);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_INTDIV_M0, 0x01A5);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVL_M0, 0xE000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_HIGH_THR_M0, 0x011A);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL1_CTRL_M0, 0x0002);
		if (ssc)
			cdns_torrent_dp_enable_ssc_19_2mhz(cdns_phy, 0x04D7, 0x006A);
		break;
	}

	if (ssc) {
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_PLLCNT_START, 0x025E);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_LOCK_PLLCNT_THR, 0x0005);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_VCOCAL_PLLCNT_START, 0x025E);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_LOCK_PLLCNT_THR, 0x0005);
	} else {
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0260);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_VCOCAL_PLLCNT_START, 0x0260);
		/* Set reset register values to disable SSC */
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL1_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL2_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL3_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL4_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_LOCK_PLLCNT_THR, 0x0003);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL1_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL2_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL3_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL4_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_LOCK_PLLCNT_THR, 0x0003);
	}

	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_LOCK_REFCNT_START, 0x0099);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_LOCK_PLLCNT_START, 0x0099);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_LOCK_REFCNT_START, 0x0099);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_LOCK_PLLCNT_START, 0x0099);
}


static void cdns_torrent_dp_pma_cmn_cfg_25mhz(struct cdns_torrent_phy *cdns_phy)
{
	/* refclock registers - assumes 25 MHz refclock */
	cdns_dp_phy_write_phy(cdns_phy, CMN_SSM_BIAS_TMR, 0x0019);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLLSM0_PLLPRE_TMR, 0x0032);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLLSM0_PLLLOCK_TMR, 0x00D1);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLLSM1_PLLPRE_TMR, 0x0032);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLLSM1_PLLLOCK_TMR, 0x00D1);
	cdns_dp_phy_write_phy(cdns_phy, CMN_BGCAL_INIT_TMR, 0x007D);
	cdns_dp_phy_write_phy(cdns_phy, CMN_BGCAL_ITER_TMR, 0x007D);
	cdns_dp_phy_write_phy(cdns_phy, CMN_IBCAL_INIT_TMR, 0x0019);
	cdns_dp_phy_write_phy(cdns_phy, CMN_TXPUCAL_INIT_TMR, 0x001E);
	cdns_dp_phy_write_phy(cdns_phy, CMN_TXPUCAL_ITER_TMR, 0x0006);
	cdns_dp_phy_write_phy(cdns_phy, CMN_TXPDCAL_INIT_TMR, 0x001E);
	cdns_dp_phy_write_phy(cdns_phy, CMN_TXPDCAL_ITER_TMR, 0x0006);
	cdns_dp_phy_write_phy(cdns_phy, CMN_RXCAL_INIT_TMR, 0x02EE);
	cdns_dp_phy_write_phy(cdns_phy, CMN_RXCAL_ITER_TMR, 0x0006);
	cdns_dp_phy_write_phy(cdns_phy, CMN_SD_CAL_INIT_TMR, 0x0002);
	cdns_dp_phy_write_phy(cdns_phy, CMN_SD_CAL_ITER_TMR, 0x0002);
	cdns_dp_phy_write_phy(cdns_phy, CMN_SD_CAL_REFTIM_START, 0x000E);
	cdns_dp_phy_write_phy(cdns_phy, CMN_SD_CAL_PLLCNT_START, 0x012B);
	/* PLL registers */
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_CP_PADJ_M0, 0x0509);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_CP_IADJ_M0, 0x0F00);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_FILT_PADJ_M0, 0x0F08);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_DSM_DIAG_M0, 0x0004);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL1_CP_PADJ_M0, 0x0509);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL1_CP_IADJ_M0, 0x0F00);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL1_FILT_PADJ_M0, 0x0F08);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_DSM_DIAG_M0, 0x0004);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_INIT_TMR, 0x00FA);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_ITER_TMR, 0x0004);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_VCOCAL_INIT_TMR, 0x00FA);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_VCOCAL_ITER_TMR, 0x0004);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_REFTIM_START, 0x0317);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_TCTRL, 0x0003);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_VCOCAL_REFTIM_START, 0x0317);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_VCOCAL_TCTRL, 0x0003);
}

/*
 * Set registers responsible for enabling and configuring SSC, with second
 * register value provided by a parameter.
 */
static void cdns_torrent_dp_enable_ssc_25mhz(struct cdns_torrent_phy *cdns_phy, u32 ctrl2_val)
{
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL1_M0, 0x0001);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL1_M0, ctrl2_val);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL1_M0, 0x007F);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL4_M0, 0x0003);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL1_M0, 0x0001);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL1_M0, ctrl2_val);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL1_M0, 0x007F);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL4_M0, 0x0003);
}

static void cdns_torrent_dp_pma_cmn_vco_cfg_25mhz(struct cdns_torrent_phy *cdns_phy,
					      u32 rate, bool ssc)
{
	/* Assumes 25 MHz refclock */
	switch (rate) {
	/* Setting VCO for 10.8GHz */
	case 2700:
	case 5400:
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_INTDIV_M0, 0x01B0);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVL_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_HIGH_THR_M0, 0x0120);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_INTDIV_M0, 0x01B0);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVL_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_HIGH_THR_M0, 0x0120);
		if (ssc)
			cdns_torrent_dp_enable_ssc_25mhz(cdns_phy, 0x0423);
		break;
	/* Setting VCO for 9.72GHz */
	case 1620:
	case 2430:
	case 3240:
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_INTDIV_M0, 0x0184);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVL_M0, 0xCCCD);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_HIGH_THR_M0, 0x0104);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_INTDIV_M0, 0x0184);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVL_M0, 0xCCCD);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_HIGH_THR_M0, 0x0104);
		if (ssc)
			cdns_torrent_dp_enable_ssc_25mhz(cdns_phy, 0x03B9);
		break;
	/* Setting VCO for 8.64GHz */
	case 2160:
	case 4320:
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_INTDIV_M0, 0x0159);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVL_M0, 0x999A);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_HIGH_THR_M0, 0x00E7);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_INTDIV_M0, 0x0159);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVL_M0, 0x999A);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_HIGH_THR_M0, 0x00E7);
		if (ssc)
			cdns_torrent_dp_enable_ssc_25mhz(cdns_phy, 0x034F);
		break;
	/* Setting VCO for 8.1GHz */
	case 8100:
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_INTDIV_M0, 0x0144);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVL_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_HIGH_THR_M0, 0x00D8);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_INTDIV_M0, 0x0144);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVL_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_HIGH_THR_M0, 0x00D8);
		if (ssc)
			cdns_torrent_dp_enable_ssc_25mhz(cdns_phy, 0x031A);
		break;
	}

	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL1_CTRL_M0, 0x0002);

	if (ssc) {
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0315);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_LOCK_PLLCNT_THR, 0x0005);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_VCOCAL_PLLCNT_START, 0x0315);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_LOCK_PLLCNT_THR, 0x0005);
	} else {
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0317);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_VCOCAL_PLLCNT_START, 0x0317);
		/* Set reset register values to disable SSC */
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL1_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL2_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL3_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_SS_CTRL4_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_LOCK_PLLCNT_THR, 0x0003);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL1_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL2_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL3_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_SS_CTRL4_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_LOCK_PLLCNT_THR, 0x0003);
	}

	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_LOCK_REFCNT_START, 0x00C7);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_LOCK_PLLCNT_START, 0x00C7);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_LOCK_REFCNT_START, 0x00C7);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_LOCK_PLLCNT_START, 0x00C7);
}


static void cdns_torrent_dp_pma_cmn_rate(struct cdns_torrent_phy *cdns_phy,
				     u32 rate, u32 lanes)
{
	unsigned int clk_sel_val = 0;
	unsigned int hsclk_div_val = 0;
	unsigned int i;

	/* 16'h0000 for single DP link configuration */
	cdns_dp_phy_write_phy(cdns_phy, PHY_PLL_CFG, 0x0000);

	switch (rate) {
	case 1620:
		clk_sel_val = 0x0f01;
		hsclk_div_val = 2;
		break;
	case 2160:
	case 2430:
	case 2700:
		clk_sel_val = 0x0701;
		hsclk_div_val = 1;
		break;
	case 3240:
		clk_sel_val = 0x0b00;
		hsclk_div_val = 2;
		break;
	case 4320:
	case 5400:
		clk_sel_val = 0x0301;
		hsclk_div_val = 0;
		break;
	case 8100:
		clk_sel_val = 0x0200;
		hsclk_div_val = 0;
		break;
	}

	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_CLK_SEL_M0, clk_sel_val);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL1_CLK_SEL_M0, clk_sel_val);

	/* PMA lane configuration to deal with multi-link operation */
	for (i = 0; i < cdns_phy->num_lanes; i++)
		cdns_dp_phy_write_phy(cdns_phy, XCVR_DIAG_HSCLK_DIV(i),
				      hsclk_div_val);
}

static void cdns_torrent_dp_pma_lane_cfg(struct cdns_torrent_phy *cdns_phy,
				     unsigned int lane)
{
	/* Per lane, refclock-dependent receiver detection setting */
	if (cdns_phy->ref_clk_rate ==  REF_CLK_19_2MHz)
		cdns_dp_phy_write_phy(cdns_phy, TX_RCVDET_ST_TMR(lane), 0x0780);
	else if (cdns_phy->ref_clk_rate == REF_CLK_25MHz)
		cdns_dp_phy_write_phy(cdns_phy, TX_RCVDET_ST_TMR(lane), 0x09C4);

	/* Writing Tx/Rx Power State Controllers registers */
	cdns_dp_phy_write_phy(cdns_phy, TX_PSC_A0(lane), 0x00FB);
	cdns_dp_phy_write_phy(cdns_phy, TX_PSC_A2(lane), 0x04AA);
	cdns_dp_phy_write_phy(cdns_phy, TX_PSC_A3(lane), 0x04AA);
	cdns_dp_phy_write_phy(cdns_phy, RX_PSC_A0(lane), 0x0000);
	cdns_dp_phy_write_phy(cdns_phy, RX_PSC_A2(lane), 0x0000);
	cdns_dp_phy_write_phy(cdns_phy, RX_PSC_A3(lane), 0x0000);

	cdns_dp_phy_write_phy(cdns_phy, RX_PSC_CAL(lane), 0x0000);

	cdns_dp_phy_write_phy(cdns_phy, RX_REE_GCSM1_CTRL(lane), 0x0000);
	cdns_dp_phy_write_phy(cdns_phy, RX_REE_GCSM2_CTRL(lane), 0x0000);
	cdns_dp_phy_write_phy(cdns_phy, RX_REE_PERGCSM_CTRL(lane), 0x0000);

	cdns_dp_phy_write_phy(cdns_phy, XCVR_DIAG_BIDI_CTRL(lane), 0x000F);
	cdns_dp_phy_write_phy(cdns_phy, XCVR_DIAG_PLLDRC_CTRL(lane), 0x0001);
	cdns_dp_phy_write_phy(cdns_phy, XCVR_DIAG_HSCLK_SEL(lane), 0x0000);
}

static int cdns_torrent_dp_run(struct cdns_torrent_phy *cdns_phy)
{
	unsigned int read_val;
	int ret;

	/*
	 * waiting for ACK of pma_xcvr_pllclk_en_ln_*, only for the
	 * master lane
	 */
	ret = cdns_phy_read_dp_poll_timeout(cdns_phy, PHY_PMA_XCVR_PLLCLK_EN_ACK,
					    read_val, read_val & 1, 0, POLL_TIMEOUT_US);
	if (ret == -ETIMEDOUT) {
		dev_err(cdns_phy->dev,
			"timeout waiting for link PLL clock enable ack\n");
		return ret;
	}

	ndelay(100);

	ret = cdns_torrent_dp_set_power_state(cdns_phy, cdns_phy->num_lanes, POWERSTATE_A2);
	if (ret)
		return ret;

	ret = cdns_torrent_dp_set_power_state(cdns_phy, cdns_phy->num_lanes, POWERSTATE_A0);

	return ret;
}

static void cdns_dp_phy_write_field(struct cdns_torrent_phy *cdns_phy,
				    unsigned int offset,
				    unsigned char start_bit,
				    unsigned char num_bits,
				    unsigned int val)
{
	unsigned int read_val;

	read_val = cdns_dp_phy_read_dp(cdns_phy, offset);
	cdns_dp_phy_write_dp(cdns_phy, offset,
			     ((val << start_bit) | (read_val & ~(((1 << num_bits) - 1) <<
								 start_bit))));
}

static int cdns_torrent_phy_on(struct phy *phy)
{
	struct cdns_torrent_phy *cdns_phy = phy_get_drvdata(phy);

	/* Take the PHY lane group out of reset */
	return reset_control_deassert(cdns_phy->phy_rst);
}

static int cdns_torrent_phy_off(struct phy *phy)
{
	struct cdns_torrent_phy *cdns_phy = phy_get_drvdata(phy);

	return reset_control_assert(cdns_phy->phy_rst);
}

static int cdns_torrent_phy_probe(struct platform_device *pdev)
{
	struct resource *regs;
	struct cdns_torrent_phy *cdns_phy;
	struct device *dev = &pdev->dev;
	struct phy_provider *phy_provider;
	struct phy *phy;
	int err;

	cdns_phy = devm_kzalloc(dev, sizeof(*cdns_phy), GFP_KERNEL);
	if (!cdns_phy)
		return -ENOMEM;

	cdns_phy->dev = &pdev->dev;

	phy = devm_phy_create(dev, NULL, &cdns_torrent_phy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "failed to create Torrent PHY\n");
		return PTR_ERR(phy);
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	cdns_phy->base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(cdns_phy->base))
		return PTR_ERR(cdns_phy->base);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cdns_phy->sd_base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(cdns_phy->sd_base))
		return PTR_ERR(cdns_phy->sd_base);

	cdns_phy->phy_rst = devm_reset_control_array_get_exclusive(dev);

	err = device_property_read_u32(dev, "num_lanes",
				       &cdns_phy->num_lanes);
	if (err)
		cdns_phy->num_lanes = DEFAULT_NUM_LANES;

	switch (cdns_phy->num_lanes) {
	case 1:
	case 2:
	case 4:
		/* valid number of lanes */
		break;
	default:
		dev_err(dev, "unsupported number of lanes: %d\n",
			cdns_phy->num_lanes);
		return -EINVAL;
	}

	err = device_property_read_u32(dev, "max_bit_rate",
				       &cdns_phy->max_bit_rate);
	if (err)
		cdns_phy->max_bit_rate = DEFAULT_MAX_BIT_RATE;

	switch (cdns_phy->max_bit_rate) {
	case 1620:
	case 2160:
	case 2430:
	case 2700:
	case 3240:
	case 4320:
	case 5400:
	case 8100:
		/* valid bit rate */
		break;
	default:
		dev_err(dev, "unsupported max bit rate: %dMbps\n",
			cdns_phy->max_bit_rate);
		return -EINVAL;
	}

	cdns_phy->clk = devm_clk_get(dev, "refclk");
	if (IS_ERR(cdns_phy->clk)) {
		dev_err(dev, "phy ref clock not found\n");
		return PTR_ERR(cdns_phy->clk);
	}

	phy_set_drvdata(phy, cdns_phy);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	dev_info(dev, "%d lanes, max bit rate %d.%03d Gbps\n",
		 cdns_phy->num_lanes,
		 cdns_phy->max_bit_rate / 1000,
		 cdns_phy->max_bit_rate % 1000);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static int cdns_torrent_dp_set_power_state(struct cdns_torrent_phy *cdns_phy,
				       u32 num_lanes,
				       enum phy_powerstate powerstate)
{
	/* Register value for power state for a single byte. */
	u32 value_part;
	u32 value;
	u32 mask;
	u32 read_val;
	u32 ret;

	switch (powerstate) {
	case (POWERSTATE_A0):
		value_part = 0x01U;
		break;
	case (POWERSTATE_A2):
		value_part = 0x04U;
		break;
	default:
		/* Powerstate A3 */
		value_part = 0x08U;
		break;
	}

	/* Select values of registers and mask, depending on enabled lane count. */
	switch (num_lanes) {
	// lane 0
	case (1):
		value = value_part;
		mask = 0x0000003FU;
		break;
	// lanes 0-1
	case (2):
		value = (value_part
			 | (value_part << 8));
		mask = 0x00003F3FU;
		break;
	// lanes 0-3, all
	default:
		value = (value_part
			 | (value_part << 8)
			 | (value_part << 16)
			 | (value_part << 24));
		mask = 0x3F3F3F3FU;
		break;
	}

	/* Set power state A<n>. */
	cdns_dp_phy_write_dp(cdns_phy, PHY_PMA_XCVR_POWER_STATE_REQ, value);
	/* Wait, until PHY acknowledges power state completion. */
	ret = cdns_phy_read_dp_poll_timeout(cdns_phy, PHY_PMA_XCVR_POWER_STATE_ACK,
					    read_val, (read_val & mask) == value, 0,
					    POLL_TIMEOUT_US);
	cdns_dp_phy_write_dp(cdns_phy, PHY_PMA_XCVR_POWER_STATE_REQ, 0x00000000);
	ndelay(100);

	return ret;
}

/*
 * Enable or disable PLL for selected lanes.
 */
static int cdns_torrent_dp_set_pll_en(struct cdns_torrent_phy *cdns_phy,
				  struct phy_configure_opts_dp *dp,
				  bool enable)
{
	u32 read_val;
	u32 ret;

	/*
	 * Used to determine, which bits to check for or enable in
	 * PHY_PMA_XCVR_PLLCLK_EN register.
	 */
	u32 pll_reg_bits;
	/* used to enable or disable lanes. */
	u32 pll_reg_write_val;

	/* Select values of registers and mask, depending on enabled lane count. */
	switch (dp->lanes) {
	// lane 0
	case (1):
		pll_reg_bits = 0x00000001;
		break;
	// lanes 0-1
	case (2):
		pll_reg_bits = 0x00000003;
		break;
	// lanes 0-3, all
	default:
		pll_reg_bits = 0x0000000F;
		break;
	}

	if (enable)
		pll_reg_write_val = pll_reg_bits;
	else
		pll_reg_write_val = 0x00000000;

	cdns_dp_phy_write_dp(cdns_phy, PHY_PMA_XCVR_PLLCLK_EN, pll_reg_write_val);

	/* Wait for acknowledgment from PHY. */
	ret = cdns_phy_read_dp_poll_timeout(cdns_phy, PHY_PMA_XCVR_PLLCLK_EN_ACK,
					    read_val,
					    (read_val & pll_reg_bits) == pll_reg_write_val,
					    0, POLL_TIMEOUT_US);
	ndelay(100);
	return ret;
}


/*
 * Perform register operations related to setting link rate, once powerstate is
 * set and PLL disable request was processed.
 */
static int cdns_torrent_dp_configure_rate(struct cdns_torrent_phy *cdns_phy,
				      struct phy_configure_opts_dp *dp)
{
	u32 ret;
	u32 read_val;

	/* Disable the cmn_pll0_en before re-programming the new data rate. */
	cdns_dp_phy_write_phy(cdns_phy, PHY_PMA_PLL_RAW_CTRL, 0);

	/* Wait for PLL ready de-assertion. */
	/* For PLL0 - PHY_PMA_CMN_CTRL2[2] == 1 */
	ret = cdns_phy_read_poll_timeout(PHY_PMA_CMN_CTRL2,
					 read_val,
					 ((read_val >> 2) & 0x01) != 0,
					 0, POLL_TIMEOUT_US);
	if (ret)
		return ret;
	ndelay(200);

	/* DP Rate Change - VCO Output settings. */
	if (cdns_phy->ref_clk_rate ==  REF_CLK_19_2MHz) {
		/* PMA common configuration 19.2MHz */
		cdns_torrent_dp_pma_cmn_vco_cfg_19_2mhz(cdns_phy, dp->link_rate, dp->ssc);
		cdns_torrent_dp_pma_cmn_cfg_19_2mhz(cdns_phy);
	} else if (cdns_phy->ref_clk_rate == REF_CLK_25MHz) {
		/* PMA common configuration 25MHz */
		cdns_torrent_dp_pma_cmn_vco_cfg_25mhz(cdns_phy, dp->link_rate, dp->ssc);
		cdns_torrent_dp_pma_cmn_cfg_25mhz(cdns_phy);
	}
	cdns_torrent_dp_pma_cmn_rate(cdns_phy, dp->link_rate, dp->lanes);

	/* Enable the cmn_pll0_en. */
	cdns_dp_phy_write_phy(cdns_phy, PHY_PMA_PLL_RAW_CTRL, 0x3);

	/* Wait for PLL ready assertion. */
	/* For PLL0 - PHY_PMA_CMN_CTRL2[0] == 1 */
	ret = cdns_phy_read_poll_timeout(PHY_PMA_CMN_CTRL2,
					 read_val,
					 (read_val & 0x01) != 0,
					 0, POLL_TIMEOUT_US);
	return ret;
}

/*
 * Verify, that parameters to configure PHY with are correct.
 */
static int cdns_torrent_dp_verify_config(struct cdns_torrent_phy *cdns_phy,
				     struct phy_configure_opts_dp *dp)
{
	u8 i;

	/* If changing link rate was required, verify it's supported. */
	if (dp->set_rate) {
		switch (dp->link_rate) {
		case 1620:
		case 2160:
		case 2430:
		case 2700:
		case 3240:
		case 4320:
		case 5400:
		case 8100:
			/* valid bit rate */
			break;
		default:
			return -EINVAL;
		}
	}

	/* Verify lane count. */
	switch (dp->lanes) {
	case 1:
	case 2:
	case 4:
		/* valid lane count. */
		break;
	default:
		return -EINVAL;
	}

	/* Check against actual number of PHY's lanes. */
	if (dp->lanes > cdns_phy->num_lanes)
		return -EINVAL;

	/*
	 * If changing voltages is required, check swing and pre-emphasis levels,
	 * per-lane.
	 */
	if (dp->set_voltages) {
		/* Lane count verified previously. */
		for (i = 0; i < dp->lanes; i++) {
			if ((dp->voltage[i] > 3) || (dp->pre[i] > 3))
				return -EINVAL;

			/* Sum of voltage swing and pre-emphasis levels cannot exceed 3. */
			if (dp->voltage[i] + dp->pre[i] > 3)
				return -EINVAL;
		}
	}

	return 0;
}

/* Set power state A0 and PLL clock enable to 0 on enabled lanes. */
static void cdns_torrent_dp_set_a0_pll(struct cdns_torrent_phy *cdns_phy,
				   u32 num_lanes)
{
	u32 pwr_state = cdns_dp_phy_read_dp(cdns_phy, PHY_PMA_XCVR_POWER_STATE_REQ);
	u32 pll_clk_en = cdns_dp_phy_read_dp(cdns_phy, PHY_PMA_XCVR_PLLCLK_EN);

	/* Lane 0 is always enabled. */
	pwr_state &= ~(PMA_XCVR_POWER_STATE_REQ_LN_MASK << PHY_POWER_STATE_LN_0);
	pll_clk_en &= ~0x01U;

	if (num_lanes > 1) {
		/* lane 1 */
		pwr_state &= ~(PMA_XCVR_POWER_STATE_REQ_LN_MASK << PHY_POWER_STATE_LN_1);
		pll_clk_en &= ~(0x01U << 1);
	}

	if (num_lanes > 2) {
		/* lanes 2 and 3 */
		pwr_state &= ~(PMA_XCVR_POWER_STATE_REQ_LN_MASK << PHY_POWER_STATE_LN_2);
		pwr_state &= ~(PMA_XCVR_POWER_STATE_REQ_LN_MASK << PHY_POWER_STATE_LN_3);
		pll_clk_en &= ~(0x01U << 2);
		pll_clk_en &= ~(0x01U << 3);
	}

	cdns_dp_phy_write_dp(cdns_phy, PHY_PMA_XCVR_POWER_STATE_REQ, pwr_state);
	cdns_dp_phy_write_dp(cdns_phy, PHY_PMA_XCVR_PLLCLK_EN, pll_clk_en);
}

/* Configure lane count as required. */
static int cdns_torrent_dp_set_lanes(struct cdns_torrent_phy *cdns_phy,
				 struct phy_configure_opts_dp *dp)
{
	u32 value;
	u32 ret;
	u8 lane_mask = (1 << dp->lanes) - 1;

	value = cdns_dp_phy_read_dp(cdns_phy, PHY_RESET);
	/* clear pma_tx_elec_idle_ln_* bits. */
	value &= ~PMA_TX_ELEC_IDLE_MASK;
	/* Assert pma_tx_elec_idle_ln_* for disabled lanes. */
	value |= ((~lane_mask) << PMA_TX_ELEC_IDLE_SHIFT) & PMA_TX_ELEC_IDLE_MASK;
	cdns_dp_phy_write_dp(cdns_phy, PHY_RESET, value);

	/* reset the link by asserting phy_l00_reset_n low */
	cdns_dp_phy_write_dp(cdns_phy, PHY_RESET, value & (~PHY_L00_RESET_N_MASK));

	/*
	 * Assert lane reset on unused lanes and lane 0 so they remain in reset
	 * and powered down when re-enabling the link
	 */
	value = (value & 0x0000FFF0) | (0x0000000E & lane_mask);
	cdns_dp_phy_write_dp(cdns_phy, PHY_RESET, value);

	cdns_torrent_dp_set_a0_pll(cdns_phy, dp->lanes);

	/* release phy_l0*_reset_n based on used laneCount */
	value = (value & 0x0000FFF0) | (0x0000000F & lane_mask);
	cdns_dp_phy_write_dp(cdns_phy, PHY_RESET, value);

	/* Wait, until PHY gets ready after releasing PHY reset signal. */
	ret = cdns_torrent_dp_wait_pma_cmn_ready(cdns_phy);
	if (ret)
		return ret;

	ndelay(100);

	/* release pma_xcvr_pllclk_en_ln_*, only for the master lane */
	cdns_dp_phy_write_dp(cdns_phy, PHY_PMA_XCVR_PLLCLK_EN, 0x0001);

	ret = cdns_torrent_dp_run(cdns_phy);

	return ret;
}

/* Configure link rate as required. */
static int cdns_torrent_dp_set_rate(struct cdns_torrent_phy *cdns_phy,
				struct phy_configure_opts_dp *dp)
{
	u32 ret;

	ret = cdns_torrent_dp_set_power_state(cdns_phy, dp->lanes, POWERSTATE_A3);
	if (ret)
		return ret;
	ret = cdns_torrent_dp_set_pll_en(cdns_phy, dp, false);
	if (ret)
		return ret;
	ndelay(200);

	ret = cdns_torrent_dp_configure_rate(cdns_phy, dp);
	if (ret)
		return ret;
	ndelay(200);

	ret = cdns_torrent_dp_set_pll_en(cdns_phy, dp, true);
	if (ret)
		return ret;
	ret = cdns_torrent_dp_set_power_state(cdns_phy, dp->lanes, POWERSTATE_A2);
	if (ret)
		return ret;
	ret = cdns_torrent_dp_set_power_state(cdns_phy, dp->lanes, POWERSTATE_A0);
	if (ret)
		return ret;
	ndelay(900);

	return ret;
}

/* Configure voltage swing and pre-emphasis for all enabled lanes. */
static void cdns_torrent_dp_set_voltages(struct cdns_torrent_phy *cdns_phy,
				     struct phy_configure_opts_dp *dp)
{
	u8 lane;
	u16 phy_reg;

	for (lane = 0; lane < dp->lanes; lane++) {
		phy_reg = cdns_dp_phy_read_phy(cdns_phy, TX_DIAG_ACYA(lane));
		/*
		 * Write 1 to register bit TX_DIAG_ACYA[0] to freeze the
		 * current state of the analog TX driver.
		 */
		phy_reg |= TX_DIAG_ACYA_HBDC_MASK;
		cdns_dp_phy_write_phy(cdns_phy, TX_DIAG_ACYA(lane), phy_reg);

		cdns_dp_phy_write_phy(cdns_phy, TX_TXCC_CTRL(lane), 0x08A4);
		phy_reg = voltage_coeffs[dp->voltage[lane]][dp->pre[lane]].diag_tx_drv;
		cdns_dp_phy_write_phy(cdns_phy, DRV_DIAG_TX_DRV(lane), phy_reg);
		phy_reg = voltage_coeffs[dp->voltage[lane]][dp->pre[lane]].mgnfs_mult;
		cdns_dp_phy_write_phy(cdns_phy, TX_TXCC_MGNFS_MULT_000(lane), phy_reg);
		phy_reg = voltage_coeffs[dp->voltage[lane]][dp->pre[lane]].cpost_mult;
		cdns_dp_phy_write_phy(cdns_phy, TX_TXCC_CPOST_MULT_00(lane), phy_reg);

		phy_reg = cdns_dp_phy_read_phy(cdns_phy, TX_DIAG_ACYA(lane));
		/*
		 * Write 0 to register bit TX_DIAG_ACYA[0] to allow the state of
		 * analog TX driver to reflect the new programmed one.
		 */
		phy_reg &= ~TX_DIAG_ACYA_HBDC_MASK;
		cdns_dp_phy_write_phy(cdns_phy, TX_DIAG_ACYA(lane), phy_reg);
	}
};

static int cdns_torrent_dp_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct cdns_torrent_phy *cdns_phy = phy_get_drvdata(phy);
	int ret;

	dev_dbg(&phy->dev,
		"cdns_dp_phy_configure rate %d, lanes %d(%d), vs %d,%d,%d,%d, pe %d,%d,%d,%d, ssc %d\n",
		opts->dp.set_rate	? opts->dp.link_rate : -1,
		opts->dp.lanes, opts->dp.set_lanes,
		(opts->dp.set_voltages && opts->dp.lanes > 0)	? opts->dp.voltage[0]	: -1,
		(opts->dp.set_voltages && opts->dp.lanes > 1)	? opts->dp.voltage[1]	: -1,
		(opts->dp.set_voltages && opts->dp.lanes > 2)	? opts->dp.voltage[2]	: -1,
		(opts->dp.set_voltages && opts->dp.lanes > 3)	? opts->dp.voltage[3]	: -1,
		(opts->dp.set_voltages && opts->dp.lanes > 0)	? opts->dp.pre[0]	: -1,
		(opts->dp.set_voltages && opts->dp.lanes > 1)	? opts->dp.pre[1]	: -1,
		(opts->dp.set_voltages && opts->dp.lanes > 2)	? opts->dp.pre[2]	: -1,
		(opts->dp.set_voltages && opts->dp.lanes > 3)	? opts->dp.pre[3]	: -1,
		opts->dp.ssc);

	ret = cdns_torrent_dp_verify_config(cdns_phy, &opts->dp);
	if (ret) {
		dev_err(&phy->dev, "invalid params for phy configure\n");
		return ret;
	}

	if (opts->dp.set_lanes) {
		ret = cdns_torrent_dp_set_lanes(cdns_phy, &opts->dp);
		if (ret) {
			dev_err(&phy->dev, "cdns_torrent_dp_set_lanes failed\n");
			return ret;
		}
	}

	if (opts->dp.set_rate) {
		ret = cdns_torrent_dp_set_rate(cdns_phy, &opts->dp);
		if (ret) {
			dev_err(&phy->dev, "cdns_torrent_dp_set_rate failed\n");
			return ret;
		}
	}

	if (opts->dp.set_voltages)
		cdns_torrent_dp_set_voltages(cdns_phy, &opts->dp);

	return ret;
}

static const struct of_device_id cdns_torrent_phy_of_match[] = {
	{
		.compatible = "cdns,torrent-phy"
	},
	{}
};
MODULE_DEVICE_TABLE(of, cdns_torrent_phy_of_match);

static struct platform_driver cdns_torrent_phy_driver = {
	.probe	= cdns_torrent_phy_probe,
	.driver = {
		.name	= "cdns-torrent-phy",
		.of_match_table	= cdns_torrent_phy_of_match,
	}
};
module_platform_driver(cdns_torrent_phy_driver);

MODULE_AUTHOR("Cadence Design Systems, Inc.");
MODULE_DESCRIPTION("Cadence Torrent PHY driver");
MODULE_LICENSE("GPL v2");
