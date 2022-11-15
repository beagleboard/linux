// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright: 2017 Cadence Design Systems, Inc.
 *
 * Author: Boris Brezillon <boris.brezillon@bootlin.com>
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_probe_helper.h>
#include <video/mipi_display.h>

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include "cdns-dsi-core.h"
#ifdef CONFIG_DRM_CDNS_DSI_J721E
#include "cdns-dsi-j721e.h"
#endif

static inline struct cdns_dsi *input_to_dsi(struct cdns_dsi_input *input)
{
	return container_of(input, struct cdns_dsi, input);
}

static inline struct cdns_dsi *to_cdns_dsi(struct mipi_dsi_host *host)
{
	return container_of(host, struct cdns_dsi, base);
}

static inline struct cdns_dsi_input *
bridge_to_cdns_dsi_input(struct drm_bridge *bridge)
{
	return container_of(bridge, struct cdns_dsi_input, bridge);
}

static unsigned int mode_to_dpi_hfp(const struct drm_display_mode *mode,
				    bool mode_valid_check)
{
	if (mode_valid_check)
		return mode->hsync_start - mode->hdisplay;

	return mode->crtc_hsync_start - mode->crtc_hdisplay;
}

static unsigned int dpi_to_dsi_timing(unsigned int dpi_timing,
				      unsigned int dpi_bpp,
				      unsigned int dsi_pkt_overhead)
{
	unsigned int dsi_timing = DIV_ROUND_UP(dpi_timing * dpi_bpp, 8);

	if (dsi_timing < dsi_pkt_overhead)
		dsi_timing = 0;
	else
		dsi_timing -= dsi_pkt_overhead;

	return dsi_timing;
}

static int cdns_dsi_mode2cfg(struct cdns_dsi *dsi,
			     const struct drm_display_mode *mode,
			     struct cdns_dsi_cfg *dsi_cfg,
			     bool mode_valid_check)
{
	struct cdns_dsi_output *output = &dsi->output;
	unsigned int tmp;
	bool sync_pulse = false;
	int bpp;

	memset(dsi_cfg, 0, sizeof(*dsi_cfg));

	if (output->dev->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		sync_pulse = true;

	bpp = mipi_dsi_pixel_format_to_bpp(output->dev->format);

	if (mode_valid_check)
		tmp = mode->htotal -
		      (sync_pulse ? mode->hsync_end : mode->hsync_start);
	else
		tmp = mode->crtc_htotal -
		      (sync_pulse ?
		       mode->crtc_hsync_end : mode->crtc_hsync_start);

	dsi_cfg->hbp = dpi_to_dsi_timing(tmp, bpp, DSI_HBP_FRAME_OVERHEAD);

	if (sync_pulse) {
		if (mode_valid_check)
			tmp = mode->hsync_end - mode->hsync_start;
		else
			tmp = mode->crtc_hsync_end - mode->crtc_hsync_start;

		dsi_cfg->hsa = dpi_to_dsi_timing(tmp, bpp,
						 DSI_HSA_FRAME_OVERHEAD);
	}

	dsi_cfg->hact = dpi_to_dsi_timing(mode_valid_check ?
					  mode->hdisplay : mode->crtc_hdisplay,
					  bpp, 0);
	dsi_cfg->hfp = dpi_to_dsi_timing(mode_to_dpi_hfp(mode, mode_valid_check),
					 bpp, DSI_HFP_FRAME_OVERHEAD);

	return 0;
}

static int cdns_dsi_adjust_phy_config(struct cdns_dsi *dsi,
			      struct cdns_dsi_cfg *dsi_cfg,
			      struct phy_configure_opts_mipi_dphy *phy_cfg,
			      const struct drm_display_mode *mode,
			      bool mode_valid_check)
{
	struct cdns_dsi_output *output = &dsi->output;
	unsigned long long dlane_bps;
	unsigned long adj_dsi_htotal;
	unsigned long dsi_htotal;
	unsigned long dpi_htotal;
	unsigned long dpi_hz;
	unsigned int dsi_hfp_ext;
	unsigned int lanes = output->dev->lanes;

	dsi_htotal = dsi_cfg->hbp + DSI_HBP_FRAME_OVERHEAD;
	if (output->dev->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		dsi_htotal += dsi_cfg->hsa + DSI_HSA_FRAME_OVERHEAD;

	dsi_htotal += dsi_cfg->hact;
	dsi_htotal += dsi_cfg->hfp + DSI_HFP_FRAME_OVERHEAD;

	/*
	 * Make sure DSI htotal is aligned on a lane boundary when calculating
	 * the expected data rate. This is done by extending HFP in case of
	 * misalignment.
	 */
	adj_dsi_htotal = dsi_htotal;
	if (dsi_htotal % lanes)
		adj_dsi_htotal += lanes - (dsi_htotal % lanes);

	dpi_hz = (mode_valid_check ? mode->clock : mode->crtc_clock) * 1000;
	dlane_bps = (unsigned long long)dpi_hz * adj_dsi_htotal;

	/* data rate in bytes/sec is not an integer, refuse the mode. */
	dpi_htotal = mode_valid_check ? mode->htotal : mode->crtc_htotal;
	if (do_div(dlane_bps, lanes * dpi_htotal))
		return -EINVAL;

	/* data rate was in bytes/sec, convert to bits/sec. */
	phy_cfg->hs_clk_rate = dlane_bps * 8;

	dsi_hfp_ext = adj_dsi_htotal - dsi_htotal;
	dsi_cfg->hfp += dsi_hfp_ext;
	dsi_cfg->htotal = dsi_htotal + dsi_hfp_ext;

	return 0;
}

static int cdns_dsi_check_conf(struct cdns_dsi *dsi,
			       const struct drm_display_mode *mode,
			       struct cdns_dsi_cfg *dsi_cfg,
			       bool mode_valid_check)
{
	struct cdns_dsi_output *output = &dsi->output;
	struct phy_configure_opts_mipi_dphy *phy_cfg = &output->phy_opts.mipi_dphy;
	unsigned long dsi_hss_hsa_hse_hbp;
	unsigned int nlanes = output->dev->lanes;
	int ret;

	ret = cdns_dsi_mode2cfg(dsi, mode, dsi_cfg, mode_valid_check);
	if (ret)
		return ret;

	phy_mipi_dphy_get_default_config(mode->crtc_clock * 1000,
					 mipi_dsi_pixel_format_to_bpp(output->dev->format),
					 nlanes, phy_cfg);

	ret = cdns_dsi_adjust_phy_config(dsi, dsi_cfg, phy_cfg, mode, mode_valid_check);
	if (ret)
		return ret;

	ret = phy_validate(dsi->dphy, PHY_MODE_MIPI_DPHY, 0, &output->phy_opts);
	if (ret)
		return ret;

	dsi_hss_hsa_hse_hbp = dsi_cfg->hbp + DSI_HBP_FRAME_OVERHEAD;
	if (output->dev->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		dsi_hss_hsa_hse_hbp += dsi_cfg->hsa + DSI_HSA_FRAME_OVERHEAD;

	/*
	 * Make sure DPI(HFP) > DSI(HSS+HSA+HSE+HBP) to guarantee that the FIFO
	 * is empty before we start a receiving a new line on the DPI
	 * interface.
	 */
	if ((u64)phy_cfg->hs_clk_rate *
	    mode_to_dpi_hfp(mode, mode_valid_check) * nlanes <
	    (u64)dsi_hss_hsa_hse_hbp *
	    (mode_valid_check ? mode->clock : mode->crtc_clock) * 1000)
		return -EINVAL;

	return 0;
}

static int cdns_dsi_bridge_attach(struct drm_bridge *bridge,
				  enum drm_bridge_attach_flags flags)
{
	struct cdns_dsi_input *input = bridge_to_cdns_dsi_input(bridge);
	struct cdns_dsi *dsi = input_to_dsi(input);
	struct cdns_dsi_output *output = &dsi->output;

	if (!drm_core_check_feature(bridge->dev, DRIVER_ATOMIC)) {
		dev_err(dsi->base.dev,
			"cdns-dsi driver is only compatible with DRM devices supporting atomic updates");
		return -ENOTSUPP;
	}

	return drm_bridge_attach(bridge->encoder, output->bridge, bridge,
				 flags);
}

static enum drm_mode_status
cdns_dsi_bridge_mode_valid(struct drm_bridge *bridge,
			   const struct drm_display_info *info,
			   const struct drm_display_mode *mode)
{
	struct cdns_dsi_input *input = bridge_to_cdns_dsi_input(bridge);
	struct cdns_dsi *dsi = input_to_dsi(input);
	struct cdns_dsi_output *output = &dsi->output;
	struct cdns_dsi_cfg dsi_cfg;
	int bpp, ret;

	/*
	 * VFP_DSI should be less than VFP_DPI and VFP_DSI should be at
	 * least 1.
	 */
	if (mode->vtotal - mode->vsync_end < 2)
		return MODE_V_ILLEGAL;

	/* VSA_DSI = VSA_DPI and must be at least 2. */
	if (mode->vsync_end - mode->vsync_start < 2)
		return MODE_V_ILLEGAL;

	/* HACT must be 32-bits aligned. */
	bpp = mipi_dsi_pixel_format_to_bpp(output->dev->format);
	if ((mode->hdisplay * bpp) % 32)
		return MODE_H_ILLEGAL;

	ret = cdns_dsi_check_conf(dsi, mode, &dsi_cfg, true);
	if (ret)
		return MODE_BAD;

	return MODE_OK;
}

static void cdns_dsi_bridge_disable(struct drm_bridge *bridge)
{
	struct cdns_dsi_input *input = bridge_to_cdns_dsi_input(bridge);
	struct cdns_dsi *dsi = input_to_dsi(input);
	u32 val;

	val = readl(dsi->regs + MCTL_MAIN_DATA_CTL);
	val &= ~(IF_VID_SELECT_MASK | IF_VID_MODE | VID_EN | HOST_EOT_GEN |
		 DISP_EOT_GEN);
	writel(val, dsi->regs + MCTL_MAIN_DATA_CTL);

	val = readl(dsi->regs + MCTL_MAIN_EN) & ~IF_EN(input->id);
	writel(val, dsi->regs + MCTL_MAIN_EN);

	if (dsi->platform_ops && dsi->platform_ops->disable)
		dsi->platform_ops->disable(dsi);

	pm_runtime_put(dsi->base.dev);
}

static void cdns_dsi_bridge_post_disable(struct drm_bridge *bridge)
{
	struct cdns_dsi_input *input = bridge_to_cdns_dsi_input(bridge);
	struct cdns_dsi *dsi = input_to_dsi(input);

	pm_runtime_put(dsi->base.dev);
}

static void cdns_dsi_hs_init(struct cdns_dsi *dsi)
{
	struct cdns_dsi_output *output = &dsi->output;
	u32 status;

	if (dsi->phy_initialized)
		return;
	/*
	 * Power all internal DPHY blocks down and maintain their reset line
	 * asserted before changing the DPHY config.
	 */
	writel(DPHY_CMN_PSO | DPHY_PLL_PSO | DPHY_ALL_D_PDN | DPHY_C_PDN |
	       DPHY_CMN_PDN | DPHY_PLL_PDN,
	       dsi->regs + MCTL_DPHY_CFG0);

	phy_init(dsi->dphy);
	phy_set_mode(dsi->dphy, PHY_MODE_MIPI_DPHY);
	phy_configure(dsi->dphy, &output->phy_opts);
	phy_power_on(dsi->dphy);

	/* Activate the PLL and wait until it's locked. */
	writel(PLL_LOCKED, dsi->regs + MCTL_MAIN_STS_CLR);
	writel(DPHY_CMN_PSO | DPHY_ALL_D_PDN | DPHY_C_PDN | DPHY_CMN_PDN,
	       dsi->regs + MCTL_DPHY_CFG0);
	WARN_ON_ONCE(readl_poll_timeout(dsi->regs + MCTL_MAIN_STS, status,
					status & PLL_LOCKED, 100, 100));
	/* De-assert data and clock reset lines. */
	writel(DPHY_CMN_PSO | DPHY_ALL_D_PDN | DPHY_C_PDN | DPHY_CMN_PDN |
	       DPHY_D_RSTB(output->dev->lanes) | DPHY_C_RSTB,
	       dsi->regs + MCTL_DPHY_CFG0);
	dsi->phy_initialized = true;
}

static void cdns_dsi_init_link(struct cdns_dsi *dsi)
{
	struct cdns_dsi_output *output = &dsi->output;
	unsigned long sysclk_period, ulpout;
	u32 val;
	int i;

	if (dsi->link_initialized)
		return;

	val = 0;
	for (i = 1; i < output->dev->lanes; i++)
		val |= DATA_LANE_EN(i);

	if (!(output->dev->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS))
		val |= CLK_CONTINUOUS;

	writel(val, dsi->regs + MCTL_MAIN_PHY_CTL);

	/* ULPOUT should be set to 1ms and is expressed in sysclk cycles. */
	sysclk_period = NSEC_PER_SEC / clk_get_rate(dsi->dsi_sys_clk);
	ulpout = DIV_ROUND_UP(NSEC_PER_MSEC, sysclk_period);
	writel(CLK_LANE_ULPOUT_TIME(ulpout) | DATA_LANE_ULPOUT_TIME(ulpout),
	       dsi->regs + MCTL_ULPOUT_TIME);

	writel(LINK_EN, dsi->regs + MCTL_MAIN_DATA_CTL);

	val = CLK_LANE_EN | PLL_START;
	for (i = 0; i < output->dev->lanes; i++)
		val |= DATA_LANE_START(i);

	writel(val, dsi->regs + MCTL_MAIN_EN);

	dsi->link_initialized = true;
}

static void cdns_dsi_bridge_enable(struct drm_bridge *bridge)
{
	struct cdns_dsi_input *input = bridge_to_cdns_dsi_input(bridge);
	struct cdns_dsi *dsi = input_to_dsi(input);
	struct cdns_dsi_output *output = &dsi->output;
	struct drm_display_mode *mode;
	struct phy_configure_opts_mipi_dphy *phy_cfg = &output->phy_opts.mipi_dphy;
	unsigned long tx_byte_period;
	struct cdns_dsi_cfg dsi_cfg;
	u32 tmp, reg_wakeup, div;
	int nlanes;

	if (WARN_ON(pm_runtime_get_sync(dsi->base.dev) < 0))
		return;

	if (dsi->platform_ops && dsi->platform_ops->enable)
		dsi->platform_ops->enable(dsi);

	mode = &bridge->encoder->crtc->state->adjusted_mode;
	nlanes = output->dev->lanes;

	WARN_ON_ONCE(cdns_dsi_check_conf(dsi, mode, &dsi_cfg, false));

	cdns_dsi_hs_init(dsi);
	cdns_dsi_init_link(dsi);

	writel(HBP_LEN(dsi_cfg.hbp) | HSA_LEN(dsi_cfg.hsa),
	       dsi->regs + VID_HSIZE1);
	writel(HFP_LEN(dsi_cfg.hfp) | HACT_LEN(dsi_cfg.hact),
	       dsi->regs + VID_HSIZE2);

	writel(VBP_LEN(mode->crtc_vtotal - mode->crtc_vsync_end - 1) |
	       VFP_LEN(mode->crtc_vsync_start - mode->crtc_vdisplay) |
	       VSA_LEN(mode->crtc_vsync_end - mode->crtc_vsync_start + 1),
	       dsi->regs + VID_VSIZE1);
	writel(mode->crtc_vdisplay, dsi->regs + VID_VSIZE2);

	tmp = dsi_cfg.htotal -
	      (dsi_cfg.hsa + DSI_BLANKING_FRAME_OVERHEAD +
	       DSI_HSA_FRAME_OVERHEAD);
	writel(BLK_LINE_PULSE_PKT_LEN(tmp), dsi->regs + VID_BLKSIZE2);
	if (output->dev->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		writel(MAX_LINE_LIMIT(tmp - DSI_NULL_FRAME_OVERHEAD),
		       dsi->regs + VID_VCA_SETTING2);

	tmp = dsi_cfg.htotal -
	      (DSI_HSS_VSS_VSE_FRAME_OVERHEAD + DSI_BLANKING_FRAME_OVERHEAD);
	writel(BLK_LINE_EVENT_PKT_LEN(tmp), dsi->regs + VID_BLKSIZE1);
	if (!(output->dev->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE))
		writel(MAX_LINE_LIMIT(tmp - DSI_NULL_FRAME_OVERHEAD),
		       dsi->regs + VID_VCA_SETTING2);

	tmp = DIV_ROUND_UP(dsi_cfg.htotal, nlanes) -
	      DIV_ROUND_UP(dsi_cfg.hsa, nlanes);

	if (!(output->dev->mode_flags & MIPI_DSI_MODE_EOT_PACKET))
		tmp -= DIV_ROUND_UP(DSI_EOT_PKT_SIZE, nlanes);

	tx_byte_period = DIV_ROUND_DOWN_ULL((u64)NSEC_PER_SEC * 8,
					    phy_cfg->hs_clk_rate);
	reg_wakeup = (phy_cfg->hs_prepare + phy_cfg->hs_zero) / tx_byte_period;
	writel(REG_WAKEUP_TIME(reg_wakeup) | REG_LINE_DURATION(tmp),
	       dsi->regs + VID_DPHY_TIME);

	/*
	 * HSTX and LPRX timeouts are both expressed in TX byte clk cycles and
	 * both should be set to at least the time it takes to transmit a
	 * frame.
	 */
	tmp = NSEC_PER_SEC / drm_mode_vrefresh(mode);
	tmp /= tx_byte_period;

	for (div = 0; div <= CLK_DIV_MAX; div++) {
		if (tmp <= HSTX_TIMEOUT_MAX)
			break;

		tmp >>= 1;
	}

	if (tmp > HSTX_TIMEOUT_MAX)
		tmp = HSTX_TIMEOUT_MAX;

	writel(CLK_DIV(div) | HSTX_TIMEOUT(tmp),
	       dsi->regs + MCTL_DPHY_TIMEOUT1);

	writel(LPRX_TIMEOUT(tmp), dsi->regs + MCTL_DPHY_TIMEOUT2);

	if (output->dev->mode_flags & MIPI_DSI_MODE_VIDEO) {
		switch (output->dev->format) {
		case MIPI_DSI_FMT_RGB888:
			tmp = VID_PIXEL_MODE_RGB888 |
			      VID_DATATYPE(MIPI_DSI_PACKED_PIXEL_STREAM_24);
			break;

		case MIPI_DSI_FMT_RGB666:
			tmp = VID_PIXEL_MODE_RGB666 |
			      VID_DATATYPE(MIPI_DSI_PIXEL_STREAM_3BYTE_18);
			break;

		case MIPI_DSI_FMT_RGB666_PACKED:
			tmp = VID_PIXEL_MODE_RGB666_PACKED |
			      VID_DATATYPE(MIPI_DSI_PACKED_PIXEL_STREAM_18);
			break;

		case MIPI_DSI_FMT_RGB565:
			tmp = VID_PIXEL_MODE_RGB565 |
			      VID_DATATYPE(MIPI_DSI_PACKED_PIXEL_STREAM_16);
			break;

		default:
			dev_err(dsi->base.dev, "Unsupported DSI format\n");
			return;
		}

		if (output->dev->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
			tmp |= SYNC_PULSE_ACTIVE | SYNC_PULSE_HORIZONTAL;

		tmp |= REG_BLKLINE_MODE(REG_BLK_MODE_BLANKING_PKT) |
		       REG_BLKEOL_MODE(REG_BLK_MODE_BLANKING_PKT) |
		       RECOVERY_MODE(RECOVERY_MODE_NEXT_HSYNC) |
		       VID_IGNORE_MISS_VSYNC;

		writel(tmp, dsi->regs + VID_MAIN_CTL);
	}

	tmp = readl(dsi->regs + MCTL_MAIN_DATA_CTL);
	tmp &= ~(IF_VID_SELECT_MASK | HOST_EOT_GEN | IF_VID_MODE);

	if (!(output->dev->mode_flags & MIPI_DSI_MODE_EOT_PACKET))
		tmp |= HOST_EOT_GEN;

	if (output->dev->mode_flags & MIPI_DSI_MODE_VIDEO)
		tmp |= IF_VID_MODE | IF_VID_SELECT(input->id) | VID_EN;

	writel(tmp, dsi->regs + MCTL_MAIN_DATA_CTL);

	tmp = readl(dsi->regs + MCTL_MAIN_EN) | IF_EN(input->id);
	writel(tmp, dsi->regs + MCTL_MAIN_EN);
}

static void cdns_dsi_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct cdns_dsi_input *input = bridge_to_cdns_dsi_input(bridge);
	struct cdns_dsi *dsi = input_to_dsi(input);

	if (WARN_ON(pm_runtime_get_sync(dsi->base.dev) < 0))
		return;

	cdns_dsi_init_link(dsi);
	cdns_dsi_hs_init(dsi);
}

static const struct drm_bridge_funcs cdns_dsi_bridge_funcs = {
	.attach = cdns_dsi_bridge_attach,
	.mode_valid = cdns_dsi_bridge_mode_valid,
	.disable = cdns_dsi_bridge_disable,
	.pre_enable = cdns_dsi_bridge_pre_enable,
	.enable = cdns_dsi_bridge_enable,
	.post_disable = cdns_dsi_bridge_post_disable,
};

static int cdns_dsi_attach(struct mipi_dsi_host *host,
			   struct mipi_dsi_device *dev)
{
	struct cdns_dsi *dsi = to_cdns_dsi(host);
	struct cdns_dsi_output *output = &dsi->output;
	struct cdns_dsi_input *input = &dsi->input;
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	struct device_node *np;
	int ret;

	/*
	 * We currently do not support connecting several DSI devices to the
	 * same host. In order to support that we'd need the DRM bridge
	 * framework to allow dynamic reconfiguration of the bridge chain.
	 */
	if (output->dev)
		return -EBUSY;

	/* We do not support burst mode yet. */
	if (dev->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
		return -ENOTSUPP;

	/*
	 * The host <-> device link might be described using an OF-graph
	 * representation, in this case we extract the device of_node from
	 * this representation, otherwise we use dsidev->dev.of_node which
	 * should have been filled by the core.
	 */
	np = of_graph_get_remote_node(dsi->base.dev->of_node, DSI_OUTPUT_PORT,
				      dev->channel);
	if (!np)
		np = of_node_get(dev->dev.of_node);

	panel = of_drm_find_panel(np);
	if (!IS_ERR(panel)) {
		bridge = drm_panel_bridge_add_typed(panel,
						    DRM_MODE_CONNECTOR_DSI);
	} else {
		bridge = of_drm_find_bridge(dev->dev.of_node);
		if (!bridge)
			bridge = ERR_PTR(-EINVAL);
	}

	of_node_put(np);

	if (IS_ERR(bridge)) {
		ret = PTR_ERR(bridge);
		dev_err(host->dev, "failed to add DSI device %s (err = %d)",
			dev->name, ret);
		return ret;
	}

	output->dev = dev;
	output->bridge = bridge;
	output->panel = panel;

	/*
	 * The DSI output has been properly configured, we can now safely
	 * register the input to the bridge framework so that it can take place
	 * in a display pipeline.
	 */
	drm_bridge_add(&input->bridge);

	return 0;
}

static int cdns_dsi_detach(struct mipi_dsi_host *host,
			   struct mipi_dsi_device *dev)
{
	struct cdns_dsi *dsi = to_cdns_dsi(host);
	struct cdns_dsi_output *output = &dsi->output;
	struct cdns_dsi_input *input = &dsi->input;

	drm_bridge_remove(&input->bridge);
	if (output->panel)
		drm_panel_bridge_remove(output->bridge);

	return 0;
}

static irqreturn_t cdns_dsi_interrupt(int irq, void *data)
{
	struct cdns_dsi *dsi = data;
	irqreturn_t ret = IRQ_NONE;
	u32 flag, ctl;

	flag = readl(dsi->regs + DIRECT_CMD_STS_FLAG);
	if (flag) {
		ctl = readl(dsi->regs + DIRECT_CMD_STS_CTL);
		ctl &= ~flag;
		writel(ctl, dsi->regs + DIRECT_CMD_STS_CTL);
		complete(&dsi->direct_cmd_comp);
		ret = IRQ_HANDLED;
	}

	return ret;
}

static ssize_t cdns_dsi_transfer(struct mipi_dsi_host *host,
				 const struct mipi_dsi_msg *msg)
{
	struct cdns_dsi *dsi = to_cdns_dsi(host);
	u32 cmd, sts, val, wait = WRITE_COMPLETED, ctl = 0;
	struct mipi_dsi_packet packet;
	int ret, i, tx_len, rx_len;

	ret = pm_runtime_resume_and_get(host->dev);
	if (ret < 0)
		return ret;

	cdns_dsi_init_link(dsi);

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret)
		goto out;

	tx_len = msg->tx_buf ? msg->tx_len : 0;
	rx_len = msg->rx_buf ? msg->rx_len : 0;

	/* For read operations, the maximum TX len is 2. */
	if (rx_len && tx_len > 2) {
		ret = -ENOTSUPP;
		goto out;
	}

	/* TX len is limited by the CMD FIFO depth. */
	if (tx_len > dsi->direct_cmd_fifo_depth) {
		ret = -ENOTSUPP;
		goto out;
	}

	/* RX len is limited by the RX FIFO depth. */
	if (rx_len > dsi->rx_fifo_depth) {
		ret = -ENOTSUPP;
		goto out;
	}

	cmd = CMD_SIZE(tx_len) | CMD_VCHAN_ID(msg->channel) |
	      CMD_DATATYPE(msg->type);

	if (msg->flags & MIPI_DSI_MSG_USE_LPM)
		cmd |= CMD_LP_EN;

	if (mipi_dsi_packet_format_is_long(msg->type))
		cmd |= CMD_LONG;

	if (rx_len) {
		cmd |= READ_CMD;
		wait = READ_COMPLETED_WITH_ERR | READ_COMPLETED;
		ctl = READ_EN | BTA_EN;
	} else if (msg->flags & MIPI_DSI_MSG_REQ_ACK) {
		cmd |= BTA_REQ;
		wait = ACK_WITH_ERR_RCVD | ACK_RCVD;
		ctl = BTA_EN;
	}

	writel(readl(dsi->regs + MCTL_MAIN_DATA_CTL) | ctl,
	       dsi->regs + MCTL_MAIN_DATA_CTL);

	writel(cmd, dsi->regs + DIRECT_CMD_MAIN_SETTINGS);

	for (i = 0; i < tx_len; i += 4) {
		const u8 *buf = msg->tx_buf;
		int j;

		val = 0;
		for (j = 0; j < 4 && j + i < tx_len; j++)
			val |= (u32)buf[i + j] << (8 * j);

		writel(val, dsi->regs + DIRECT_CMD_WRDATA);
	}

	/* Clear status flags before sending the command. */
	writel(wait, dsi->regs + DIRECT_CMD_STS_CLR);
	writel(wait, dsi->regs + DIRECT_CMD_STS_CTL);
	reinit_completion(&dsi->direct_cmd_comp);
	writel(0, dsi->regs + DIRECT_CMD_SEND);

	wait_for_completion_timeout(&dsi->direct_cmd_comp,
				    msecs_to_jiffies(1000));

	sts = readl(dsi->regs + DIRECT_CMD_STS);
	writel(wait, dsi->regs + DIRECT_CMD_STS_CLR);
	writel(0, dsi->regs + DIRECT_CMD_STS_CTL);

	writel(readl(dsi->regs + MCTL_MAIN_DATA_CTL) & ~ctl,
	       dsi->regs + MCTL_MAIN_DATA_CTL);

	/* We did not receive the events we were waiting for. */
	if (!(sts & wait)) {
		ret = -ETIMEDOUT;
		goto out;
	}

	/* 'READ' or 'WRITE with ACK' failed. */
	if (sts & (READ_COMPLETED_WITH_ERR | ACK_WITH_ERR_RCVD)) {
		ret = -EIO;
		goto out;
	}

	for (i = 0; i < rx_len; i += 4) {
		u8 *buf = msg->rx_buf;
		int j;

		val = readl(dsi->regs + DIRECT_CMD_RDDATA);
		for (j = 0; j < 4 && j + i < rx_len; j++)
			buf[i + j] = val >> (8 * j);
	}

out:
	pm_runtime_put(host->dev);
	return ret;
}

static const struct mipi_dsi_host_ops cdns_dsi_ops = {
	.attach = cdns_dsi_attach,
	.detach = cdns_dsi_detach,
	.transfer = cdns_dsi_transfer,
};

static int __maybe_unused cdns_dsi_resume(struct device *dev)
{
	struct cdns_dsi *dsi = dev_get_drvdata(dev);

	reset_control_deassert(dsi->dsi_p_rst);
	clk_prepare_enable(dsi->dsi_p_clk);
	clk_prepare_enable(dsi->dsi_sys_clk);

	return 0;
}

static int __maybe_unused cdns_dsi_suspend(struct device *dev)
{
	struct cdns_dsi *dsi = dev_get_drvdata(dev);

	clk_disable_unprepare(dsi->dsi_sys_clk);
	clk_disable_unprepare(dsi->dsi_p_clk);
	reset_control_assert(dsi->dsi_p_rst);
	dsi->link_initialized = false;
	dsi->phy_initialized = false;
	return 0;
}

static UNIVERSAL_DEV_PM_OPS(cdns_dsi_pm_ops, cdns_dsi_suspend, cdns_dsi_resume,
			    NULL);

static int cdns_dsi_drm_probe(struct platform_device *pdev)
{
	struct cdns_dsi *dsi;
	struct cdns_dsi_input *input;
	struct resource *res;
	int ret, irq;
	u32 val;

	dsi = devm_kzalloc(&pdev->dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	platform_set_drvdata(pdev, dsi);

	input = &dsi->input;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dsi->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dsi->regs))
		return PTR_ERR(dsi->regs);

	dsi->dsi_p_clk = devm_clk_get(&pdev->dev, "dsi_p_clk");
	if (IS_ERR(dsi->dsi_p_clk))
		return PTR_ERR(dsi->dsi_p_clk);

	dsi->dsi_p_rst = devm_reset_control_get_optional_exclusive(&pdev->dev,
								"dsi_p_rst");
	if (IS_ERR(dsi->dsi_p_rst))
		return PTR_ERR(dsi->dsi_p_rst);

	dsi->dsi_sys_clk = devm_clk_get(&pdev->dev, "dsi_sys_clk");
	if (IS_ERR(dsi->dsi_sys_clk))
		return PTR_ERR(dsi->dsi_sys_clk);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	dsi->dphy = devm_phy_get(&pdev->dev, "dphy");
	if (IS_ERR(dsi->dphy))
		return PTR_ERR(dsi->dphy);

	ret = clk_prepare_enable(dsi->dsi_p_clk);
	if (ret)
		return ret;

	val = readl(dsi->regs + ID_REG);
	if (REV_VENDOR_ID(val) != 0xcad) {
		dev_err(&pdev->dev, "invalid vendor id\n");
		ret = -EINVAL;
		goto err_disable_pclk;
	}

	dsi->platform_ops = of_device_get_match_data(&pdev->dev);

	val = readl(dsi->regs + IP_CONF);
	dsi->direct_cmd_fifo_depth = 1 << (DIRCMD_FIFO_DEPTH(val) + 2);
	dsi->rx_fifo_depth = RX_FIFO_DEPTH(val);
	init_completion(&dsi->direct_cmd_comp);

	writel(0, dsi->regs + MCTL_MAIN_DATA_CTL);
	writel(0, dsi->regs + MCTL_MAIN_EN);
	writel(0, dsi->regs + MCTL_MAIN_PHY_CTL);

	/*
	 * We only support the DPI input, so force input->id to
	 * CDNS_DPI_INPUT.
	 */
	input->id = CDNS_DPI_INPUT;
	input->bridge.funcs = &cdns_dsi_bridge_funcs;
	input->bridge.of_node = pdev->dev.of_node;

	/* Mask all interrupts before registering the IRQ handler. */
	writel(0, dsi->regs + MCTL_MAIN_STS_CTL);
	writel(0, dsi->regs + MCTL_DPHY_ERR_CTL1);
	writel(0, dsi->regs + CMD_MODE_STS_CTL);
	writel(0, dsi->regs + DIRECT_CMD_STS_CTL);
	writel(0, dsi->regs + DIRECT_CMD_RD_STS_CTL);
	writel(0, dsi->regs + VID_MODE_STS_CTL);
	writel(0, dsi->regs + TVG_STS_CTL);
	writel(0, dsi->regs + DPI_IRQ_EN);
	ret = devm_request_irq(&pdev->dev, irq, cdns_dsi_interrupt, 0,
			       dev_name(&pdev->dev), dsi);
	if (ret)
		goto err_disable_pclk;

	pm_runtime_enable(&pdev->dev);
	dsi->base.dev = &pdev->dev;
	dsi->base.ops = &cdns_dsi_ops;

	if (dsi->platform_ops && dsi->platform_ops->init) {
		ret = dsi->platform_ops->init(dsi);
		if (ret != 0) {
			dev_err(&pdev->dev, "platform initialization failed: %d\n",
				ret);
			goto err_disable_runtime_pm;
		}
	}

	ret = mipi_dsi_host_register(&dsi->base);
	if (ret)
		goto err_deinit_platform;

	clk_disable_unprepare(dsi->dsi_p_clk);

	return 0;

err_deinit_platform:
	if (dsi->platform_ops && dsi->platform_ops->exit)
		dsi->platform_ops->exit(dsi);

err_disable_runtime_pm:
	pm_runtime_disable(&pdev->dev);

err_disable_pclk:
	clk_disable_unprepare(dsi->dsi_p_clk);

	return ret;
}

static int cdns_dsi_drm_remove(struct platform_device *pdev)
{
	struct cdns_dsi *dsi = platform_get_drvdata(pdev);

	mipi_dsi_host_unregister(&dsi->base);

	if (dsi->platform_ops && dsi->platform_ops->exit)
		dsi->platform_ops->exit(dsi);

	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id cdns_dsi_of_match[] = {
	{ .compatible = "cdns,dsi" },
#ifdef CONFIG_DRM_CDNS_DSI_J721E
	{ .compatible = "ti,j721e-dsi",
	  .data = &dsi_ti_j721e_ops,
	},
#endif
	{ },
};
MODULE_DEVICE_TABLE(of, cdns_dsi_of_match);

static struct platform_driver cdns_dsi_platform_driver = {
	.probe  = cdns_dsi_drm_probe,
	.remove = cdns_dsi_drm_remove,
	.driver = {
		.name   = "cdns-dsi",
		.of_match_table = cdns_dsi_of_match,
		.pm = &cdns_dsi_pm_ops,
	},
};
module_platform_driver(cdns_dsi_platform_driver);

MODULE_AUTHOR("Boris Brezillon <boris.brezillon@bootlin.com>");
MODULE_DESCRIPTION("Cadence DSI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cdns-dsi");
