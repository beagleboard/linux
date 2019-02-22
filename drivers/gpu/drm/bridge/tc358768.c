// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 *  Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_mipi_dsi.h>
#include <video/mipi_display.h>
#include <video/videomode.h>

/* Global (16-bit addressable) */
#define TC358768_CHIPID			0x0000
#define TC358768_SYSCTL			0x0002
#define TC358768_CONFCTL		0x0004
#define TC358768_VSDLY			0x0006
#define TC358768_DATAFMT		0x0008
#define TC358768_GPIOEN			0x000E
#define TC358768_GPIODIR		0x0010
#define TC358768_GPIOIN			0x0012
#define TC358768_GPIOOUT		0x0014
#define TC358768_PLLCTL0		0x0016
#define TC358768_PLLCTL1		0x0018
#define TC358768_CMDBYTE		0x0022
#define TC358768_PP_MISC		0x0032
#define TC358768_DSITX_DT		0x0050
#define TC358768_FIFOSTATUS		0x00F8

/* Debug (16-bit addressable) */
#define TC358768_VBUFCTRL		0x00E0
#define TC358768_DBG_WIDTH		0x00E2
#define TC358768_DBG_VBLANK		0x00E4
#define TC358768_DBG_DATA		0x00E8

/* TX PHY (32-bit addressable) */
#define TC358768_CLW_DPHYCONTTX		0x0100
#define TC358768_D0W_DPHYCONTTX		0x0104
#define TC358768_D1W_DPHYCONTTX		0x0108
#define TC358768_D2W_DPHYCONTTX		0x010C
#define TC358768_D3W_DPHYCONTTX		0x0110
#define TC358768_CLW_CNTRL		0x0140
#define TC358768_D0W_CNTRL		0x0144
#define TC358768_D1W_CNTRL		0x0148
#define TC358768_D2W_CNTRL		0x014C
#define TC358768_D3W_CNTRL		0x0150

/* TX PPI (32-bit addressable) */
#define TC358768_STARTCNTRL		0x0204
#define TC358768_DSITXSTATUS		0x0208
#define TC358768_LINEINITCNT		0x0210
#define TC358768_LPTXTIMECNT		0x0214
#define TC358768_TCLK_HEADERCNT		0x0218
#define TC358768_TCLK_TRAILCNT		0x021C
#define TC358768_THS_HEADERCNT		0x0220
#define TC358768_TWAKEUP		0x0224
#define TC358768_TCLK_POSTCNT		0x0228
#define TC358768_THS_TRAILCNT		0x022C
#define TC358768_HSTXVREGCNT		0x0230
#define TC358768_HSTXVREGEN		0x0234
#define TC358768_TXOPTIONCNTRL		0x0238
#define TC358768_BTACNTRL1		0x023C

/* TX CTRL (32-bit addressable) */
#define TC358768_DSI_CONTROL		0x040C
#define TC358768_DSI_STATUS		0x0410
#define TC358768_DSI_INT		0x0414
#define TC358768_DSI_INT_ENA		0x0418
#define TC358768_DSICMD_RDFIFO		0x0430
#define TC358768_DSI_ACKERR		0x0434
#define TC358768_DSI_ACKERR_INTENA	0x0438
#define TC358768_DSI_ACKERR_HALT	0x043c
#define TC358768_DSI_RXERR		0x0440
#define TC358768_DSI_RXERR_INTENA	0x0444
#define TC358768_DSI_RXERR_HALT		0x0448
#define TC358768_DSI_ERR		0x044C
#define TC358768_DSI_ERR_INTENA		0x0450
#define TC358768_DSI_ERR_HALT		0x0454
#define TC358768_DSI_CONFW		0x0500
#define TC358768_DSI_LPCMD		0x0500
#define TC358768_DSI_RESET		0x0504
#define TC358768_DSI_INT_CLR		0x050C
#define TC358768_DSI_START		0x0518

/* DSITX CTRL (16-bit addressable) */
#define TC358768_DSICMD_TX		0x0600
#define TC358768_DSICMD_TYPE		0x0602
#define TC358768_DSICMD_WC		0x0604
#define TC358768_DSICMD_WD0		0x0610
#define TC358768_DSICMD_WD1		0x0612
#define TC358768_DSICMD_WD2		0x0614
#define TC358768_DSICMD_WD3		0x0616
#define TC358768_DSI_EVENT		0x0620
#define TC358768_DSI_VSW		0x0622
#define TC358768_DSI_VBPR		0x0624
#define TC358768_DSI_VACT		0x0626
#define TC358768_DSI_HSW		0x0628
#define TC358768_DSI_HBPR		0x062A
#define TC358768_DSI_HACT		0x062C

struct tc358768_dsi_output {
	struct mipi_dsi_device *dev;
	struct drm_panel *panel;
	struct drm_bridge *bridge;
};

struct tc358768_priv {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	int enabled;
	struct clk *refclk;

	struct mipi_dsi_host dsi_host;
	struct drm_bridge bridge;
	struct tc358768_dsi_output output;

	const struct drm_display_mode *mode;
	u32 pd_lines; /* number of Parallel Port Input Data Lines */
	u32 dsi_lanes; /* number of DSI Lanes */

	u32 fbd;
	u32 prd;
	u32 frs;

	u32 dsiclk; /* pll_clk / 2 */
};

static inline struct tc358768_priv *dsi_host_to_tc358768(struct mipi_dsi_host
							 *host)
{
	return container_of(host, struct tc358768_priv, dsi_host);
}

static inline struct tc358768_priv *bridge_to_tc358768(struct drm_bridge
						       *bridge)
{
	return container_of(bridge, struct tc358768_priv, bridge);
}

static int tc358768_write(struct tc358768_priv *priv, u32 reg, u32 val)
{
	size_t count = 2;

	/* 16-bit register? */
	if (reg < 0x100 || reg >= 0x600)
		count = 1;

	return regmap_bulk_write(priv->regmap, reg, &val, count);
}

static int tc358768_read(struct tc358768_priv *priv, u32 reg, u32 *val)
{
	size_t count = 2;

	/* 16-bit register? */
	if (reg < 0x100 || reg >= 0x600) {
		*val = 0;
		count = 1;
	}

	return regmap_bulk_read(priv->regmap, reg, val, count);
}

static int tc358768_update_bits(struct tc358768_priv *priv, u32 reg, u32 mask,
				u32 val)
{
	int ret;
	u32 tmp, orig;

	ret = tc358768_read(priv, reg, &orig);
	if (ret != 0)
		return ret;

	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig)
		ret = tc358768_write(priv, reg, tmp);

	return ret;
}

static void tc358768_sw_reset(struct tc358768_priv *priv)
{
	/* Assert Reset */
	tc358768_write(priv, TC358768_SYSCTL, 1);
	/* Release Reset, Exit Sleep */
	tc358768_write(priv, TC358768_SYSCTL, 0);
}

static void tc358768_hw_enable(struct tc358768_priv *priv, bool enable)
{
	if (!priv->reset_gpio)
		return;

	if (enable && priv->enabled++ > 0)
		return;

	if (!enable && --priv->enabled != 0)
		return;

	gpiod_set_value_cansleep(priv->reset_gpio, enable);
	regcache_cache_only(priv->regmap, !enable);
	if (enable) {
		/* wait for encoder clocks to stabilize */
		usleep_range(1000, 2000);

		regcache_sync(priv->regmap);
	}
}

static u32 tc358768_pll_to_pclk(struct tc358768_priv *priv, u32 pll_clk)
{
	return (u32)div_u64((u64)pll_clk * priv->dsi_lanes, priv->pd_lines);
}

static u32 tc358768_pclk_to_pll(struct tc358768_priv *priv, u32 pclk)
{
	return (u32)div_u64((u64)pclk * priv->pd_lines, priv->dsi_lanes);
}

static int tc358768_calc_pll(struct tc358768_priv *priv)
{
	const u32 frs_limits[] = {
		1000000000,
		500000000,
		250000000,
		125000000,
		62500000
	};
	unsigned long refclk;
	u32 prd, target_pll, i, max_pll, min_pll;
	u32 frs, best_diff, best_pll, best_prd, best_fbd;

	target_pll = tc358768_pclk_to_pll(priv, priv->mode->clock * 1000);

	/* pll_clk = RefClk * [(FBD + 1)/ (PRD + 1)] * [1 / (2^FRS)] */

	frs = UINT_MAX;

	for (i = 0; i < ARRAY_SIZE(frs_limits) - 1; i++) {
		if (target_pll < frs_limits[i] &&
		    target_pll >= frs_limits[i + 1]) {
			frs = i;
			max_pll = frs_limits[i];
			min_pll = frs_limits[i + 1];
			break;
		}
	}

	if (frs == UINT_MAX)
		return -EINVAL;

	refclk = clk_get_rate(priv->refclk);

	best_diff = UINT_MAX;
	best_pll = 0;
	best_prd = 0;
	best_fbd = 0;

	for (prd = 0; prd < 16; ++prd) {
		u32 divisor = (prd + 1) * (1 << frs);
		u32 fbd;

		for (fbd = 0; fbd < 512; ++fbd) {
			u32 pll, diff;

			pll = (u32)div_u64((u64)refclk * (fbd + 1), divisor);

			if (pll >= max_pll || pll < min_pll)
				continue;

			diff = max(pll, target_pll) - min(pll, target_pll);

			if (diff < best_diff) {
				best_diff = diff;
				best_pll = pll;
				best_prd = prd;
				best_fbd = fbd;
			}

			if (best_diff == 0)
				break;
		}

		if (best_diff == 0)
			break;
	}

	if (best_diff == UINT_MAX) {
		dev_err(priv->dev, "could not find suitable PLL setup\n");
		return -EINVAL;
	}

	priv->fbd = best_fbd;
	priv->prd = best_prd;
	priv->frs = frs;
	priv->dsiclk = best_pll / 2;

	return 0;
}

static int tc358768_dsi_host_attach(struct mipi_dsi_host *host,
				    struct mipi_dsi_device *dev)
{
	struct tc358768_priv *priv = dsi_host_to_tc358768(host);
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	struct device_node *ep;
	int ret;

	if (dev->lanes > 4) {
		dev_err(priv->dev, "unsupported number of data lanes(%u)\n",
			dev->lanes);
		return -EINVAL;
	}

	/*
	 * tc358768 supports both Video and Pulse mode, but the driver only
	 * implements Video (event) mode currently
	 */
	if (!(dev->mode_flags & MIPI_DSI_MODE_VIDEO)) {
		dev_err(priv->dev, "Only MIPI_DSI_MODE_VIDEO is supported\n");
		return -ENOTSUPP;
	}

	/*
	 * tc358768 supports RGB888, RGB666, RGB666_PACKED and RGB565, but only
	 * RGB888 is verified.
	 */
	if (dev->format != MIPI_DSI_FMT_RGB888) {
		dev_warn(priv->dev, "Only MIPI_DSI_FMT_RGB888 tested!\n");
		return -ENOTSUPP;
	}

	ret = drm_of_find_panel_or_bridge(host->dev->of_node, 1, 0, &panel,
					  &bridge);
	if (ret)
		return ret;

	if (panel) {
		bridge = drm_panel_bridge_add(panel, DRM_MODE_CONNECTOR_DSI);
		if (IS_ERR(bridge))
			return PTR_ERR(bridge);
	}

	priv->output.dev = dev;
	priv->output.bridge = bridge;
	priv->output.panel = panel;

	priv->dsi_lanes = dev->lanes;

	/* get input ep (port0/endpoint0) */
	ret = -EINVAL;
	ep = of_graph_get_endpoint_by_regs(host->dev->of_node, 0, 0);
	if (ep) {
		ret = of_property_read_u32(ep, "data-lines", &priv->pd_lines);

		of_node_put(ep);
	}

	if (ret)
		priv->pd_lines = mipi_dsi_pixel_format_to_bpp(dev->format);

	drm_bridge_add(&priv->bridge);

	return 0;
}

static int tc358768_dsi_host_detach(struct mipi_dsi_host *host,
				    struct mipi_dsi_device *dev)
{
	struct tc358768_priv *priv = dsi_host_to_tc358768(host);

	drm_bridge_remove(&priv->bridge);
	if (priv->output.panel)
		drm_panel_bridge_remove(priv->output.bridge);

	return 0;
}

static ssize_t tc358768_dsi_host_transfer(struct mipi_dsi_host *host,
					  const struct mipi_dsi_msg *msg)
{
	struct tc358768_priv *priv = dsi_host_to_tc358768(host);
	struct mipi_dsi_packet packet;
	int ret;

	if (msg->rx_len) {
		dev_warn(priv->dev, "MIPI rx is not supported\n");
		return -ENOTSUPP;
	}

	if (msg->tx_len > 8) {
		dev_warn(priv->dev, "Maximum 8 byte MIPI tx is supported\n");
		return -ENOTSUPP;
	}

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret)
		return ret;

	tc358768_hw_enable(priv, true);

	if (mipi_dsi_packet_format_is_short(msg->type)) {
		tc358768_write(priv, TC358768_DSICMD_TYPE,
			       (0x10 << 8) | (packet.header[0] & 0x3f));
		tc358768_write(priv, TC358768_DSICMD_WC, 0);
		tc358768_write(priv, TC358768_DSICMD_WD0,
			       (packet.header[2] << 8) | packet.header[1]);
	} else {
		int i, j;

		tc358768_write(priv, TC358768_DSICMD_TYPE,
			       (0x40 << 8) | (packet.header[0] & 0x3f));
		tc358768_write(priv, TC358768_DSICMD_WC, packet.payload_length);
		for (i = 0; i < packet.payload_length; i += 2) {
			u16 val = 0;

			for (j = 0; j < 2 && i + j < packet.payload_length; j++)
				val |= packet.payload[i + j] << (8 * j);

			tc358768_write(priv, TC358768_DSICMD_WD0 + i, val);
		}
	}

	/* start transfer */
	tc358768_write(priv, TC358768_DSICMD_TX, 1);

	tc358768_hw_enable(priv, false);

	return 0;
}

static const struct mipi_dsi_host_ops tc358768_dsi_host_ops = {
	.attach = tc358768_dsi_host_attach,
	.detach = tc358768_dsi_host_detach,
	.transfer = tc358768_dsi_host_transfer,
};

static int tc358768_bridge_attach(struct drm_bridge *bridge)
{
	struct tc358768_priv *priv = bridge_to_tc358768(bridge);

	if (!drm_core_check_feature(bridge->dev, DRIVER_ATOMIC)) {
		dev_err(priv->dev, "needs atomic updates support\n");
		return -ENOTSUPP;
	}

	return drm_bridge_attach(bridge->encoder, priv->output.bridge, bridge);
}

static enum drm_mode_status
tc358768_bridge_mode_valid(struct drm_bridge *bridge,
			   const struct drm_display_mode *mode)
{
	struct tc358768_priv *priv = bridge_to_tc358768(bridge);

	priv->mode = mode;

	if (tc358768_calc_pll(priv))
		return MODE_CLOCK_RANGE;

	return MODE_OK;
}

static void tc358768_bridge_disable(struct drm_bridge *bridge)
{
	struct tc358768_priv *priv = bridge_to_tc358768(bridge);

	/* set FrmStop */
	tc358768_update_bits(priv, TC358768_PP_MISC, BIT(15), BIT(15));

	/* wait at least for one frame */
	msleep(50);

	/* clear PP_en */
	tc358768_update_bits(priv, TC358768_CONFCTL, BIT(6), 0);

	/* set RstPtr */
	tc358768_update_bits(priv, TC358768_PP_MISC, BIT(14), BIT(14));

	tc358768_hw_enable(priv, false);
}

static void tc358768_setup_pll(struct tc358768_priv *priv)
{
	u32 fbd, prd, frs;

	fbd = priv->fbd;
	prd = priv->prd;
	frs = priv->frs;

	dev_dbg(priv->dev, "PLL: refclk %lu, fbd %u, prd %u, frs %u\n",
		clk_get_rate(priv->refclk), fbd, prd, frs);
	dev_dbg(priv->dev, "PLL: pll_clk: %u, DSIClk %u, DSIByteClk %u\n",
		priv->dsiclk * 2, priv->dsiclk, priv->dsiclk / 4);
	dev_dbg(priv->dev, "PLL: pclk %u (panel: %u)\n",
		tc358768_pll_to_pclk(priv, priv->dsiclk * 2),
		priv->mode->clock * 1000);

	/* PRD[15:12] FBD[8:0] */
	tc358768_write(priv, TC358768_PLLCTL0, (prd << 12) | fbd);

	/* FRS[11:10] LBWS[9:8] CKEN[4] RESETB[1] EN[0] */
	tc358768_write(priv, TC358768_PLLCTL1,
		       (frs << 10) | (0x2 << 8) | BIT(1) | BIT(0));

	/* wait for lock */
	usleep_range(1000, 2000);

	/* FRS[11:10] LBWS[9:8] CKEN[4] PLL_CKEN[4] RESETB[1] EN[0] */
	tc358768_write(priv, TC358768_PLLCTL1,
		       (frs << 10) | (0x2 << 8) | BIT(4) | BIT(1) | BIT(0));
}

#define TC358768_PRECISION	1000
static u32 tc358768_ns_to_cnt(u32 ns, u32 period_nsk)
{
	return (ns * TC358768_PRECISION + period_nsk) / period_nsk;
}

static u32 tc358768_to_ns(u32 nsk)
{
	return (nsk / TC358768_PRECISION);
}

static void tc358768_bridge_enable(struct drm_bridge *bridge)
{
	struct tc358768_priv *priv = bridge_to_tc358768(bridge);
	const struct drm_display_mode *mode = priv->mode;
	struct mipi_dsi_device *dsi_dev = priv->output.dev;
	u32 val, val2, lptxcnt, hact, data_type;
	u32 dsiclk = priv->dsiclk;
	u32 dsibclk = dsiclk / 4;
	u32 dsibclk_nsk, dsiclk_nsk, ui_nsk, phy_delay_nsk;
	int i;

	tc358768_hw_enable(priv, true);

	tc358768_sw_reset(priv);

	tc358768_setup_pll(priv);

	/* VSDly[9:0] */
	tc358768_write(priv, TC358768_VSDLY, 1);

	/* Data Format Control Register */
	val = BIT(2) | BIT(1) | BIT(0); /* rdswap_en | dsitx_en | txdt_en */
	switch (dsi_dev->format) {
	case MIPI_DSI_FMT_RGB888:
		val |= (0x3 << 4);
		hact = mode->hdisplay * 3;
		data_type = MIPI_DSI_PACKED_PIXEL_STREAM_24;
		break;
	case MIPI_DSI_FMT_RGB666:
		val |= (0x4 << 4);
		hact = mode->hdisplay * 3;
		data_type = MIPI_DSI_PACKED_PIXEL_STREAM_18;
		break;

	case MIPI_DSI_FMT_RGB666_PACKED:
		val |= (0x4 << 4) | BIT(3);
		hact = mode->hdisplay * 18 / 8;
		data_type = MIPI_DSI_PIXEL_STREAM_3BYTE_18;
		break;

	case MIPI_DSI_FMT_RGB565:
		val |= (0x5 << 4);
		hact = mode->hdisplay * 2;
		data_type = MIPI_DSI_PACKED_PIXEL_STREAM_16;
		break;
	default:
		dev_err(priv->dev, "Invalid data format (%u)\n",
			dsi_dev->format);
		return;
	}

	tc358768_write(priv, TC358768_DATAFMT, val);
	tc358768_write(priv, TC358768_DSITX_DT, data_type);

	/* Enable D-PHY (HiZ->LP11) */
	tc358768_write(priv, TC358768_CLW_CNTRL, 0x0000);
	/* Enable lanes */
	for (i = 0; i < dsi_dev->lanes; i++)
		tc358768_write(priv, TC358768_D0W_CNTRL + i * 4, 0x0000);

	/* DSI Timings */
	dsibclk_nsk = (u32)div_u64((u64)1000000000 * TC358768_PRECISION,
				  dsibclk);
	dsiclk_nsk = (u32)div_u64((u64)1000000000 * TC358768_PRECISION, dsiclk);
	ui_nsk = dsiclk_nsk / 2;
	phy_delay_nsk = dsibclk_nsk + 2 * dsiclk_nsk;
	dev_dbg(priv->dev, "dsiclk_nsk: %u\n", dsiclk_nsk);
	dev_dbg(priv->dev, "ui_nsk: %u\n", ui_nsk);
	dev_dbg(priv->dev, "dsibclk_nsk: %u\n", dsibclk_nsk);
	dev_dbg(priv->dev, "phy_delay_nsk: %u\n", phy_delay_nsk);

	/* LP11 > 100us for D-PHY Rx Init */
	val = tc358768_ns_to_cnt(100 * 1000, dsibclk_nsk) - 1;
	dev_dbg(priv->dev, "LINEINITCNT: 0x%x\n", val);
	tc358768_write(priv, TC358768_LINEINITCNT, val);

	/* LPTimeCnt > 50ns */
	val = tc358768_ns_to_cnt(50, dsibclk_nsk) - 1;
	lptxcnt = val;
	dev_dbg(priv->dev, "LPTXTIMECNT: 0x%x\n", val);
	tc358768_write(priv, TC358768_LPTXTIMECNT, val);

	/* 38ns < TCLK_PREPARE < 95ns */
	val = tc358768_ns_to_cnt(65, dsibclk_nsk) - 1;
	/* TCLK_PREPARE > 300ns */
	val2 = tc358768_ns_to_cnt(300 + tc358768_to_ns(3 * ui_nsk),
				  dsibclk_nsk);
	val |= (val2 - tc358768_to_ns(phy_delay_nsk - dsibclk_nsk)) << 8;
	dev_dbg(priv->dev, "TCLK_HEADERCNT: 0x%x\n", val);
	tc358768_write(priv, TC358768_TCLK_HEADERCNT, val);

	/* TCLK_TRAIL > 60ns + 3*UI */
	val = 60 + tc358768_to_ns(3 * ui_nsk);
	val = tc358768_ns_to_cnt(val, dsibclk_nsk) - 5;
	dev_dbg(priv->dev, "TCLK_TRAILCNT: 0x%x\n", val);
	tc358768_write(priv, TC358768_TCLK_TRAILCNT, val);

	/* 40ns + 4*UI < THS_PREPARE < 85ns + 6*UI */
	val = 50 + tc358768_to_ns(4 * ui_nsk);
	val = tc358768_ns_to_cnt(val, dsibclk_nsk) - 1;
	/* THS_ZERO > 145ns + 10*UI */
	val2 = tc358768_ns_to_cnt(145 - tc358768_to_ns(ui_nsk), dsibclk_nsk);
	val |= (val2 - tc358768_to_ns(phy_delay_nsk)) << 8;
	dev_dbg(priv->dev, "THS_HEADERCNT: 0x%x\n", val);
	tc358768_write(priv, TC358768_THS_HEADERCNT, val);

	/* TWAKEUP > 1ms in lptxcnt steps */
	val = tc358768_ns_to_cnt(1020000, dsibclk_nsk);
	val = val / (lptxcnt + 1) - 1;
	dev_dbg(priv->dev, "TWAKEUP: 0x%x\n", val);
	tc358768_write(priv, TC358768_TWAKEUP, val);

	/* TCLK_POSTCNT > 60ns + 52*UI */
	val = tc358768_ns_to_cnt(60 + tc358768_to_ns(52 * ui_nsk),
				 dsibclk_nsk) - 3;
	dev_dbg(priv->dev, "TCLK_POSTCNT: 0x%x\n", val);
	tc358768_write(priv, TC358768_TCLK_POSTCNT, val);

	/* 60ns + 4*UI < THS_PREPARE < 105ns + 12*UI */
	val = tc358768_ns_to_cnt(60 + tc358768_to_ns(15 * ui_nsk),
				 dsibclk_nsk) - 5;
	dev_dbg(priv->dev, "THS_TRAILCNT: 0x%x\n", val);
	tc358768_write(priv, TC358768_THS_TRAILCNT, val);

	val = BIT(0);
	for (i = 0; i < dsi_dev->lanes; i++)
		val |= BIT(i + 1);
	tc358768_write(priv, TC358768_HSTXVREGEN, val);

	if (!(dsi_dev->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS))
		tc358768_write(priv, TC358768_TXOPTIONCNTRL, 0x1);

	/* TXTAGOCNT[26:16] RXTASURECNT[10:0] */
	val = tc358768_to_ns((lptxcnt + 1) * dsibclk_nsk * 4);
	val = tc358768_ns_to_cnt(val, dsibclk_nsk) - 1;
	val2 = tc358768_ns_to_cnt(tc358768_to_ns((lptxcnt + 1) * dsibclk_nsk),
				  dsibclk_nsk) - 2;
	val |= val2 << 16;
	dev_dbg(priv->dev, "BTACNTRL1: 0x%x\n", val);
	tc358768_write(priv, TC358768_BTACNTRL1, val);

	/* START[0] */
	tc358768_write(priv, TC358768_STARTCNTRL, 1);

	/* Set event mode */
	tc358768_write(priv, TC358768_DSI_EVENT, 1);

	/* vsw (+ vbp) */
	tc358768_write(priv, TC358768_DSI_VSW,
		       mode->vtotal - mode->vsync_start);
	/* vbp (not used in event mode) */
	tc358768_write(priv, TC358768_DSI_VBPR, 0);
	/* vact */
	tc358768_write(priv, TC358768_DSI_VACT, mode->vdisplay);

	/* (hsw + hbp) * byteclk * ndl / pclk */
	val = (u32)div_u64((mode->htotal - mode->hsync_start) *
			   ((u64)priv->dsiclk / 4) * priv->dsi_lanes,
			   mode->clock * 1000);
	tc358768_write(priv, TC358768_DSI_HSW, val);
	/* hbp (not used in event mode) */
	tc358768_write(priv, TC358768_DSI_HBPR, 0);
	/* hact (bytes) */
	tc358768_write(priv, TC358768_DSI_HACT, hact);

	/* VSYNC polarity */
	if (!(mode->flags & DRM_MODE_FLAG_NVSYNC))
		tc358768_update_bits(priv, TC358768_CONFCTL, BIT(5), BIT(5));
	/* HSYNC polarity */
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		tc358768_update_bits(priv, TC358768_PP_MISC, BIT(0), BIT(0));

	/* Start DSI Tx */
	tc358768_write(priv, TC358768_DSI_START, 0x1);

	/* Configure DSI_Control register */
	val = (6 << 29) | (0x3 << 24); /* CLEAR, DSI_Control */
	val |= BIT(7) | BIT(5) | 0x3 << 1 | BIT(0);
	tc358768_write(priv, TC358768_DSI_CONFW, val);

	val = (5 << 29) | (0x3 << 24); /* SET, DSI_Control */
	if (!(dsi_dev->mode_flags & MIPI_DSI_MODE_LPM))
		val |= BIT(7);
	if (!(dsi_dev->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS))
		val |= BIT(5);
	val |= (dsi_dev->lanes - 1) << 1;
	if (dsi_dev->mode_flags & MIPI_DSI_MODE_EOT_PACKET)
		val |= BIT(0);
	tc358768_write(priv, TC358768_DSI_CONFW, val);

	val = (6 << 29) | (0x3 << 24); /* CLEAR, DSI_Control */
	val |= 0x8000; /* DSI mode */
	tc358768_write(priv, TC358768_DSI_CONFW, val);

	/* clear FrmStop and RstPtr */
	tc358768_update_bits(priv, TC358768_PP_MISC, 0x3 << 14, 0);

	/* set PP_en */
	tc358768_update_bits(priv, TC358768_CONFCTL, BIT(6), BIT(6));
}

static const struct drm_bridge_funcs tc358768_bridge_funcs = {
	.attach = tc358768_bridge_attach,
	.mode_valid = tc358768_bridge_mode_valid,
	.disable = tc358768_bridge_disable,
	.enable = tc358768_bridge_enable,
};

static const struct drm_bridge_timings default_tc358768_timings = {
	.input_bus_flags = DRM_BUS_FLAG_PIXDATA_SAMPLE_POSEDGE
		 | DRM_BUS_FLAG_SYNC_SAMPLE_NEGEDGE
		 | DRM_BUS_FLAG_DE_HIGH,
};

static bool tc358768_is_reserved_reg(unsigned int reg)
{
	switch (reg) {
	case 0x114 ... 0x13f:
	case 0x200:
	case 0x20c:
	case 0x400 ... 0x408:
	case 0x41c ... 0x42f:
		return true;
	default:
		return false;
	}
}

static bool tc358768_writeable_reg(struct device *dev, unsigned int reg)
{
	if (tc358768_is_reserved_reg(reg))
		return false;

	switch (reg) {
	case TC358768_CHIPID:
	case TC358768_FIFOSTATUS:
	case TC358768_DSITXSTATUS ... (TC358768_DSITXSTATUS + 2):
	case TC358768_DSI_CONTROL ... (TC358768_DSI_INT_ENA + 2):
	case TC358768_DSICMD_RDFIFO ... (TC358768_DSI_ERR_HALT + 2):
		return false;
	default:
		return true;
	}
}

static bool tc358768_readable_reg(struct device *dev, unsigned int reg)
{
	if (tc358768_is_reserved_reg(reg))
		return false;

	switch (reg) {
	case TC358768_STARTCNTRL:
	case TC358768_DSI_CONFW ... (TC358768_DSI_CONFW + 2):
	case TC358768_DSI_INT_CLR ... (TC358768_DSI_INT_CLR + 2):
	case TC358768_DSI_START ... (TC358768_DSI_START + 2):
	case TC358768_DBG_DATA:
		return false;
	default:
		return true;
	}
}

static bool tc358768_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TC358768_FIFOSTATUS:
	case TC358768_STARTCNTRL ... (TC358768_DSITXSTATUS + 2):
	case TC358768_DSI_CONTROL ... (TC358768_DSI_INT_ENA + 2):
	case TC358768_DSICMD_RDFIFO ... (TC358768_DSI_ERR_HALT + 2):
	case TC358768_DSICMD_TX:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config tc358768_regmap_config = {
	.name = "tc358768",
	.reg_bits = 16,
	.val_bits = 16,
	.max_register = TC358768_DSI_HACT,
	.cache_type = REGCACHE_RBTREE,
	.writeable_reg = tc358768_writeable_reg,
	.readable_reg = tc358768_readable_reg,
	.volatile_reg = tc358768_volatile_reg,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static const struct i2c_device_id tc358768_i2c_ids[] = {
	{ "tc358768", 0 },
	{ "tc358778", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tc358768_i2c_ids);

static const struct of_device_id tc358768_of_ids[] = {
	{ .compatible = "toshiba,tc358768", },
	{ .compatible = "toshiba,tc358778", },
	{ }
};
MODULE_DEVICE_TABLE(of, tc358768_of_ids);

static int tc358768_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct tc358768_priv *priv;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	int ret;

	if (!np)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);
	priv->dev = dev;

	priv->refclk = devm_clk_get(dev, "refclk");
	if (IS_ERR(priv->refclk))
		return PTR_ERR(priv->refclk);

	priv->reset_gpio  = devm_gpiod_get_optional(dev, "reset",
						    GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio))
		return PTR_ERR(priv->reset_gpio);

	priv->regmap = devm_regmap_init_i2c(client, &tc358768_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "Failed to init regmap\n");
		return PTR_ERR(priv->regmap);
	}

	if (priv->reset_gpio)
		regcache_cache_only(priv->regmap, true);

	priv->dsi_host.dev = dev;
	priv->dsi_host.ops = &tc358768_dsi_host_ops;

	priv->bridge.funcs = &tc358768_bridge_funcs;
	priv->bridge.timings = &default_tc358768_timings;
	priv->bridge.of_node = np;

	i2c_set_clientdata(client, priv);

	ret = mipi_dsi_host_register(&priv->dsi_host);
	if (ret)
		return ret;

	return 0;
}

static int tc358768_i2c_remove(struct i2c_client *client)
{
	struct tc358768_priv *priv = i2c_get_clientdata(client);

	mipi_dsi_host_unregister(&priv->dsi_host);

	return 0;
}

static struct i2c_driver tc358768_driver = {
	.driver = {
		.name = "tc358768",
		.of_match_table = tc358768_of_ids,
	},
	.id_table = tc358768_i2c_ids,
	.probe = tc358768_i2c_probe,
	.remove	= tc358768_i2c_remove,
};
module_i2c_driver(tc358768_driver);

MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@ti.com>");
MODULE_DESCRIPTION("TC358768AXBG/TC358778XBG DSI bridge");
MODULE_LICENSE("GPL v2");
