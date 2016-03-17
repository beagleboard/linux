/*
 * Toshiba TC358768AXBG/TC358778XBG DPI to DSI encoder
 *
 * Copyright (C) 2015 Texas Instruments
 * Author: Marcus Cooksey <mcooksey@ti.com>
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <video/omapdss.h>
#include <video/omap-panel-data.h>
#include <video/of_display_timing.h>

#define TC358768_NAME		"tc358768"

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
#define TC358768_DSI_STATUS		0x0410
#define TC358768_DSI_INT		0x0414
#define TC358768_DSICMD_RXFIFO		0x0430
#define TC358768_DSI_ACKERR		0x0434
#define TC358768_DSI_RXERR		0x0440
#define TC358768_DSI_ERR		0x044C
#define TC358768_DSI_CONFW		0x0500
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

struct tc358768_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;

	struct device *dev;

	struct omap_video_timings videomode;

	struct gpio_desc *reset_gpio;

	struct regmap *regmap;

	struct clk *refclk;

	u32 dpi_ndl;
	u32 dsi_ndl;

	unsigned fbd;
	unsigned prd;
	unsigned frs;

	u32 bitclk;
};

static const struct regmap_config tc358768_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static int tc358768_write(struct tc358768_drv_data *ddata,
	unsigned int reg, unsigned int val)
{
	int r;

	/* 16-bit register? */
	if (reg < 0x100 || reg >= 0x600) {
		dev_dbg(ddata->dev, "WR16\t%04x\t%08x\n", reg, val);
		return regmap_write(ddata->regmap, reg, val);
	}

	dev_dbg(ddata->dev, "WR32\t%04x\t%08x\n", reg, val);

	/* 32-bit register, write in two parts */
	r = regmap_write(ddata->regmap, reg, val);
	if (r)
		return r;

	return regmap_write(ddata->regmap, reg + 2, val >> 16);
}

static int tc358768_read(struct tc358768_drv_data *ddata,
	unsigned int reg, unsigned int *val)
{
	int r;
	unsigned v1, v2;

	/* 16-bit register? */
	if (reg < 0x100 || reg >= 0x600)
		return regmap_read(ddata->regmap, reg, val);

	/* 32-bit register, read in two parts */
	r = regmap_read(ddata->regmap, reg, &v1);
	if (r)
		return r;

	r = regmap_read(ddata->regmap, reg + 2, &v2);
	if (r)
		return r;

	*val = v1 | (v2 << 16);
	return 0;
}

static int tc358768_update_bits(struct tc358768_drv_data *ddata,
	unsigned int reg, unsigned int mask, unsigned int val)
{
	int ret;
	unsigned int tmp, orig;

	ret = tc358768_read(ddata, reg, &orig);
	if (ret != 0)
		return ret;

	tmp = orig & ~mask;
	tmp |= val & mask;

	dev_dbg(ddata->dev, "UPD \t%04x\t%08x -> %08x\n", reg, orig, tmp);

	if (tmp != orig)
		ret = tc358768_write(ddata, reg, tmp);

	return ret;
}

static void tc358768_sw_reset(struct tc358768_drv_data *ddata)
{
	/* Assert Reset */
	tc358768_write(ddata, TC358768_SYSCTL, 1);
	/* Release Reset, Exit Sleep */
	tc358768_write(ddata, TC358768_SYSCTL, 0);
}

static int tc358768_calc_pll(struct tc358768_drv_data *ddata)
{
	unsigned fbd, prd, frs;
	u32 pll;
	u32 target;
	bool found;
	unsigned long refclk;

	/* target byteclk */
	target = ddata->videomode.pixelclock * ddata->dpi_ndl / 8 / ddata->dsi_ndl;
	/* target pll clk */
	target = target * 4 * 2;

	/* pll_clk = RefClk * [(FBD + 1)/ (PRD + 1)] * [1 / (2^FRS)] */

	if (target >= 500000000)
		frs = 0;
	else if (target >= 250000000)
		frs = 1;
	else if (target >= 125000000)
		frs = 2;
	else if (target >= 62500000)
		frs = 3;
	else
		return -EINVAL;

	found = false;

	refclk = clk_get_rate(ddata->refclk);

	for (prd = 0; prd < 16; ++prd) {
		for (fbd = 0; fbd < 512; ++fbd) {
			pll = (u32)div_u64((u64)refclk * (fbd + 1),
				(prd + 1) * (1 << frs));

			if (pll == target) {
				ddata->fbd = fbd;
				ddata->prd = prd;
				ddata->frs = frs;
				ddata->bitclk = pll / 2;

				return 0;
			}
		}
	}

	dev_err(ddata->dev, "could not find suitable PLL setup\n");

	return -EINVAL;
}

static void tc358768_setup_pll(struct tc358768_drv_data *ddata)
{
	unsigned fbd, prd, frs;

	fbd = ddata->fbd;
	prd = ddata->prd;
	frs = ddata->frs;

	dev_dbg(ddata->dev, "PLL: refclk %lu, fbd %u, prd %u, frs %u\n",
		clk_get_rate(ddata->refclk), fbd, prd, frs);

	dev_dbg(ddata->dev, "PLL: %u MHz, BitClk %u MHz, ByteClk %u MHz\n",
		ddata->bitclk * 2, ddata->bitclk, ddata->bitclk / 4);

	/* PRD[15:12] FBD[8:0] */
	tc358768_write(ddata, TC358768_PLLCTL0, (prd << 12) | fbd);

	/* FRS[11:10] LBWS[9:8] CKEN[4] RESETB[1] EN[0] */
	tc358768_write(ddata, TC358768_PLLCTL1,
		(frs << 10) | (0x2 << 8) | (0 << 4) | (1 << 1) | (1 << 0));

	/* wait for lock */
	usleep_range(1000, 2000);

	/* FRS[11:10] LBWS[9:8] CKEN[4] RESETB[1] EN[0] */
	tc358768_write(ddata, TC358768_PLLCTL1,
		(frs << 10) | (0x2 << 8) | (1 << 4) | (1 << 1) | (1 << 0));
}

static void tc358768_power_on(struct tc358768_drv_data *ddata)
{
	const struct omap_video_timings *t = &ddata->videomode;

	tc358768_sw_reset(ddata);

	tc358768_setup_pll(ddata);

	/* VSDly[9:0] */
	tc358768_write(ddata, TC358768_VSDLY, 1);
	/* PDFormat[7:4] spmode_en[3] rdswap_en[2] dsitx_en[1] txdt_en[0] */
	tc358768_write(ddata, TC358768_DATAFMT, (0x3 << 4) | (1 << 2) | (1 << 1) | (1 << 0));
	/* dsitx_dt[7:0] 3e = Packed Pixel Stream, 24-bit RGB, 8-8-8 Format*/
	tc358768_write(ddata, TC358768_DSITX_DT, 0x003e);

	/* Enable D-PHY (HiZ->LP11) */
	tc358768_write(ddata, TC358768_CLW_CNTRL, 0x0000);
	tc358768_write(ddata, TC358768_D0W_CNTRL, 0x0000);
	tc358768_write(ddata, TC358768_D1W_CNTRL, 0x0000);
	tc358768_write(ddata, TC358768_D2W_CNTRL, 0x0000);
	tc358768_write(ddata, TC358768_D3W_CNTRL, 0x0000);

	/* DSI Timings */
	/* LP11 = 100 us for D-PHY Rx Init */
	tc358768_write(ddata, TC358768_LINEINITCNT,	0x00002c88);
	tc358768_write(ddata, TC358768_LPTXTIMECNT,	0x00000005);
	tc358768_write(ddata, TC358768_TCLK_HEADERCNT,	0x00001f06);
	tc358768_write(ddata, TC358768_TCLK_TRAILCNT,	0x00000003);
	tc358768_write(ddata, TC358768_THS_HEADERCNT,	0x00000606);
	tc358768_write(ddata, TC358768_TWAKEUP,		0x00004a88);
	tc358768_write(ddata, TC358768_TCLK_POSTCNT,	0x0000000b);
	tc358768_write(ddata, TC358768_THS_TRAILCNT,	0x00000004);
	tc358768_write(ddata, TC358768_HSTXVREGEN,	0x0000001f);

	/* CONTCLKMODE[0] */
	tc358768_write(ddata, TC358768_TXOPTIONCNTRL, 0x1);
	/* TXTAGOCNT[26:16] RXTASURECNT[10:0] */
	tc358768_write(ddata, TC358768_BTACNTRL1, (0x5 << 16) | (0x5));
	/* START[0] */
	tc358768_write(ddata, TC358768_STARTCNTRL, 0x1);

	/* DSI Tx Timing Control */

	/* Set event mode */
	tc358768_write(ddata, TC358768_DSI_EVENT, 1);

	/* vsw (+ vbp) */
	tc358768_write(ddata, TC358768_DSI_VSW, t->vsw + t->vbp);
	/* vbp (not used in event mode) */
	tc358768_write(ddata, TC358768_DSI_VBPR, 0);
	/* vact */
	tc358768_write(ddata, TC358768_DSI_VACT, t->y_res);

	/* (hsw + hbp) * byteclk * ndl / pclk */
	tc358768_write(ddata, TC358768_DSI_HSW,
		(u32)div_u64((t->hsw + t->hbp) * ((u64)ddata->bitclk / 4) * ddata->dsi_ndl, t->pixelclock));
	/* hbp (not used in event mode) */
	tc358768_write(ddata, TC358768_DSI_HBPR, 0);
	/* hact (bytes) */
	tc358768_write(ddata, TC358768_DSI_HACT, t->x_res * 3);

	/* Start DSI Tx */
	tc358768_write(ddata, TC358768_DSI_START, 0x1);

	/* SET, DSI_Control, 0xa7 */
	/* 0xa7 = HS | CONTCLK | 4-datalines | EoTDisable */
	tc358768_write(ddata, TC358768_DSI_CONFW, (5<<29) | (0x3 << 24) | 0xa7);
	/* CLEAR, DSI_Control, 0x8001 */
	/* 0x8001 = DSIMode */
	tc358768_write(ddata, TC358768_DSI_CONFW, (6<<29) | (0x3 << 24) | 0x8000);

	/* clear FrmStop and RstPtr */
	tc358768_update_bits(ddata, TC358768_PP_MISC, 0x3 << 14, 0);

	/* set PP_en */
	tc358768_update_bits(ddata, TC358768_CONFCTL, 1 << 6, 1 << 6);
}

static void tc358768_power_off(struct tc358768_drv_data *ddata)
{
	/* set FrmStop */
	tc358768_update_bits(ddata, TC358768_PP_MISC, 1 << 15, 1 << 15);

	/* wait at least for one frame */
	msleep(50);

	/* clear PP_en */
	tc358768_update_bits(ddata, TC358768_CONFCTL, 1 << 6, 0);

	/* set RstPtr */
	tc358768_update_bits(ddata, TC358768_PP_MISC, 1 << 14, 1 << 14);
}

#define to_panel_data(p) container_of(p, struct tc358768_drv_data, dssdev)

static int tc358768_connect(struct omap_dss_device *dssdev,
		struct omap_dss_device *dst)
{
	struct tc358768_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (omapdss_device_is_connected(dssdev))
		return 0;

	r = in->ops.dpi->connect(in, dssdev);
	if (r)
		return r;

	dst->src = dssdev;
	dssdev->dst = dst;

	return 0;
}

static void tc358768_disconnect(struct omap_dss_device *dssdev,
		struct omap_dss_device *dst)
{
	struct tc358768_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	WARN_ON(!omapdss_device_is_connected(dssdev));
	if (!omapdss_device_is_connected(dssdev))
		return;

	WARN_ON(dst != dssdev->dst);
	if (dst != dssdev->dst)
		return;

	dst->src = NULL;
	dssdev->dst = NULL;

	in->ops.dpi->disconnect(in, dssdev);
}

static int tc358768_enable(struct omap_dss_device *dssdev)
{
	struct tc358768_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (!omapdss_device_is_connected(dssdev))
		return -ENODEV;

	if (omapdss_device_is_enabled(dssdev))
		return 0;

	in->ops.dpi->set_timings(in, &ddata->videomode);

	r = tc358768_calc_pll(ddata);
	if (r)
		return r;

	r = in->ops.dpi->enable(in);
	if (r)
		return r;

	if (ddata->reset_gpio)
		gpiod_set_value_cansleep(ddata->reset_gpio, 1);

	/* wait for encoder clocks to stabilize */
	usleep_range(1000, 2000);

	tc358768_power_on(ddata);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void tc358768_disable(struct omap_dss_device *dssdev)
{
	struct tc358768_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_enabled(dssdev))
		return;

	tc358768_power_off(ddata);

	if (ddata->reset_gpio)
		gpiod_set_value_cansleep(ddata->reset_gpio, 0);

	in->ops.dpi->disable(in);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static void tc358768_set_timings(struct omap_dss_device *dssdev,
				 struct omap_video_timings *timings)
{
	struct tc358768_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	ddata->videomode = *timings;

	ddata->videomode.data_pclk_edge = OMAPDSS_DRIVE_SIG_FALLING_EDGE;
	ddata->videomode.sync_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE;

	dssdev->panel.timings = ddata->videomode;

	in->ops.dpi->set_timings(in, &ddata->videomode);
}

static void tc358768_get_timings(struct omap_dss_device *dssdev,
				 struct omap_video_timings *timings)
{
	struct tc358768_drv_data *ddata = to_panel_data(dssdev);

	*timings = ddata->videomode;
}

static int tc358768_check_timings(struct omap_dss_device *dssdev,
				  struct omap_video_timings *timings)
{
	struct tc358768_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	return in->ops.dpi->check_timings(in, timings);
}

static const struct omapdss_dpi_ops tc358768_dpi_ops = {
	.connect	= tc358768_connect,
	.disconnect	= tc358768_disconnect,

	.enable		= tc358768_enable,
	.disable	= tc358768_disable,

	.check_timings	= tc358768_check_timings,
	.set_timings	= tc358768_set_timings,
	.get_timings	= tc358768_get_timings,
};

static int tc358768_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct regmap *regmap;
	struct tc358768_drv_data *ddata;
	struct device *dev = &client->dev;
	struct omap_dss_device *dssdev;
	struct gpio_desc *gpio;
	struct device_node *np = dev->of_node;
	struct device_node *ep;
	int r;

	if (!np)
		return -ENODEV;

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (ddata == NULL)
		return -ENOMEM;

	dev_set_drvdata(dev, ddata);
	ddata->dev = dev;

	ddata->refclk = devm_clk_get(dev, "refclk");
	if (IS_ERR(ddata->refclk))
		return PTR_ERR(ddata->refclk);

	/* get input ep (port0/endpoint0) */
	ep = omapdss_of_get_endpoint(np, 0, 0);
	if (!ep)
		return -EINVAL;

	r = of_property_read_u32(ep, "data-lines", &ddata->dpi_ndl);
	if (r)
		return r;

	of_node_put(ep);

	/* get output ep (port1/endpoint0) */
	ep = omapdss_of_get_endpoint(np, 1, 0);
	if (!ep)
		return -EINVAL;

	r = of_property_count_u32_elems(ep, "lanes");
	if (r < 0)
		return r;
	ddata->dsi_ndl = r / 2 - 1;

	of_node_put(ep);

	regmap = devm_regmap_init_i2c(client, &tc358768_regmap_config);
	if (IS_ERR(regmap)) {
		r = PTR_ERR(regmap);
		dev_err(dev, "Failed to init regmap: %d\n", r);
		return r;
	}

	ddata->regmap = regmap;

	gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);

	ddata->reset_gpio = gpio;

	ddata->in = omapdss_of_find_source_for_first_ep(np);
	if (IS_ERR(ddata->in)) {
		dev_err(dev, "failed to find video source\n");
		return PTR_ERR(ddata->in);
	}

	dssdev = &ddata->dssdev;
	dssdev->ops.dpi = &tc358768_dpi_ops;
	dssdev->dev = dev;
	dssdev->type = OMAP_DISPLAY_TYPE_DPI;
	/*
	 * XXX: output is really DSI but for now we only support dummy DSI
	 * displays, and we can model that easily with "DPI" output.
	 */
	dssdev->output_type = OMAP_DISPLAY_TYPE_DPI;
	dssdev->owner = THIS_MODULE;
	dssdev->phy.dpi.data_lines = ddata->dpi_ndl;
	dssdev->port_num = 1;

	r = omapdss_register_output(dssdev);
	if (r) {
		dev_err(dev, "Failed to register tc358768\n");
		goto err_reg_display;
	}

	return 0;

err_reg_display:
	omap_dss_put_device(ddata->in);
	return r;
}

static int tc358768_i2c_remove(struct i2c_client *client)
{
	struct tc358768_drv_data *ddata = dev_get_drvdata(&client->dev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct omap_dss_device *in = ddata->in;

	omapdss_unregister_output(&ddata->dssdev);

	WARN_ON(omapdss_device_is_enabled(dssdev));
	if (omapdss_device_is_enabled(dssdev))
		tc358768_disable(dssdev);

	WARN_ON(omapdss_device_is_connected(dssdev));
	if (omapdss_device_is_connected(dssdev))
		tc358768_disconnect(dssdev, dssdev->dst);

	omap_dss_put_device(in);

	return 0;
}

static const struct i2c_device_id tc358768_id[] = {
	{ TC358768_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tc358768_id);

static const struct of_device_id tc358768_of_match[] = {
	{ .compatible = "omapdss,toshiba,tc358768", },
	{ }
};
MODULE_DEVICE_TABLE(of, tc358768_of_match);

static struct i2c_driver tc358768_i2c_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= TC358768_NAME,
		.of_match_table	= tc358768_of_match,
	},
	.id_table	= tc358768_id,
	.probe		= tc358768_i2c_probe,
	.remove		= tc358768_i2c_remove,
};

module_i2c_driver(tc358768_i2c_driver);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ti.com>");
MODULE_DESCRIPTION("TC358768AXBG/TC358778XBG DPI-to-DSI Encoder");
MODULE_LICENSE("GPL");
