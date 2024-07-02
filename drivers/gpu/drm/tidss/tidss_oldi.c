// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 - Texas Instruments Incorporated
 *
 * Aradhya Bhatia <a-bhatia1@ti.com>
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/mfd/syscon.h>
#include <linux/media-bus-format.h>
#include <linux/regmap.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_panel.h>
#include <drm/drm_of.h>
#include <drm/drm_crtc.h>
#include <drm/drm_probe_helper.h>

#include "tidss_drv.h"
#include "tidss_oldi.h"

struct tidss_oldi {
	struct tidss_device	*tidss;
	struct device		*dev;

	struct drm_bridge	bridge;
	struct drm_bridge	*next_bridge;

	struct drm_panel *panel;

	enum tidss_oldi_link_type link_type;
	const struct oldi_bus_format *bus_format;
	u32 oldi_instance;
	u32 parent_vp;

	struct clk *s_clk;
	struct regmap *io_ctrl;
};

static const struct oldi_bus_format oldi_bus_formats[] = {
	{ MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,	18, SPWG_18,	MEDIA_BUS_FMT_RGB666_1X18 },
	{ MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,	24, SPWG_24,	MEDIA_BUS_FMT_RGB888_1X24 },
	{ MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA,	24, JEIDA_24,	MEDIA_BUS_FMT_RGB888_1X24 },
};

#define OLDI_IDLE_CLK_HZ	25000000 /*25 MHz */

static inline struct tidss_oldi *
drm_bridge_to_tidss_oldi(struct drm_bridge *bridge)
{
	return container_of(bridge, struct tidss_oldi, bridge);
}

static int tidss_oldi_bridge_attach(struct drm_bridge *bridge,
				    enum drm_bridge_attach_flags flags)
{
	struct tidss_oldi *oldi = drm_bridge_to_tidss_oldi(bridge);

	if (!oldi->next_bridge) {
		dev_err(oldi->dev,
			"%s: OLDI%d Failure attach next bridge\n",
			__func__, oldi->oldi_instance);
		return -ENODEV;
	}

	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR)) {
		dev_err(oldi->dev,
			"%s: OLDI%d DRM_BRIDGE_ATTACH_NO_CONNECTOR is mandatory.\n",
			__func__, oldi->oldi_instance);
		return -EINVAL;
	}

	return drm_bridge_attach(bridge->encoder, oldi->next_bridge,
				 bridge, flags);
}

static int
oldi_set_serial_clk(struct tidss_oldi *oldi, unsigned long rate)
{
	unsigned long new_rate;
	int ret;

	ret = clk_set_rate(oldi->s_clk, rate);
	if (ret) {
		dev_err(oldi->dev,
			"OLDI%d: failed to set serial clk rate to %lu Hz\n",
			 oldi->oldi_instance, rate);
		return ret;
	}

	new_rate = clk_get_rate(oldi->s_clk);

	if (dispc_pclk_diff(rate, new_rate) > 5)
		dev_warn(oldi->dev,
			 "OLDI%d Clock rate %lu differs over 5%% from requested %lu\n",
			 oldi->oldi_instance, new_rate, rate);

	dev_dbg(oldi->dev, "OLDI%d: new rate %lu Hz (requested %lu Hz)\n",
		oldi->oldi_instance, clk_get_rate(oldi->s_clk), rate);

	return 0;
}

static void tidss_oldi_tx_power(struct tidss_oldi *oldi, bool power)
{
	u32 val = 0, mask;

	if (WARN_ON(!oldi->io_ctrl))
		return;

	/*
	 * The power control bits are Active Low, and remain powered off by
	 * default. That is, the bits are set to 1. To power on the OLDI TXes,
	 * the bits must be cleared to 0. Since there are cases where not all
	 * OLDI TXes are being used, the power logic selectively powers them
	 * on.
	 * Setting the variable 'val' to particular bit masks, makes sure that
	 * the unrequired OLDI TXes remain powered off.
	 */

	if (power) {
		val = 0;
		switch (oldi->link_type) {
		case OLDI_MODE_SINGLE_LINK:
			if (oldi->oldi_instance == OLDI(0))
				mask = OLDI_PWRDN_TX(0) | OLDI_PWRDN_BG;
			else if (oldi->oldi_instance == OLDI(1))
				mask = OLDI_PWRDN_TX(1) | OLDI_PWRDN_BG;

			break;
		case OLDI_MODE_CLONE_SINGLE_LINK:
		case OLDI_MODE_DUAL_LINK:
			mask = OLDI_PWRDN_TX(0) | OLDI_PWRDN_TX(1) | OLDI_PWRDN_BG;
			break;
		default:
			if (oldi->oldi_instance == OLDI(0))
				mask = OLDI_PWRDN_TX(0);
			else if (oldi->oldi_instance == OLDI(1))
				mask = OLDI_PWRDN_TX(1);

			val = mask;
			break;
		}
	} else {
		switch (oldi->link_type) {
		case OLDI_MODE_CLONE_SINGLE_LINK:
		case OLDI_MODE_DUAL_LINK:
			mask = OLDI_PWRDN_TX(0) | OLDI_PWRDN_TX(1) | OLDI_PWRDN_BG;
			break;
		case OLDI_MODE_SINGLE_LINK:
		default:
			if (oldi->oldi_instance == OLDI(0))
				mask = OLDI_PWRDN_TX(0);
			else if (oldi->oldi_instance == OLDI(1))
				mask = OLDI_PWRDN_TX(1);

			break;
		}
		val = mask;
	}

	regmap_update_bits(oldi->io_ctrl, OLDI_PD_CTRL, mask, val);
}

static int tidss_oldi_config(struct tidss_oldi *oldi)
{
	const struct oldi_bus_format *bus_fmt = NULL;
	u32 oldi_reset_bit = BIT(5);
	int count = 0;
	u32 oldi_cfg = 0;

	bus_fmt = oldi->bus_format;

	/*
	 * MASTERSLAVE and SRC bits of OLDI Config are always set to 0.
	 */

	if (bus_fmt->data_width == 24)
		oldi_cfg |= OLDI_MSB;
	else if (bus_fmt->data_width != 18)
		dev_warn(oldi->dev,
			 "OLDI%d: DSS port width %d not supported\n",
			 oldi->oldi_instance, bus_fmt->data_width);

	oldi_cfg |= OLDI_DEPOL;

	oldi_cfg = (oldi_cfg & (~OLDI_MAP)) | (bus_fmt->oldi_mode_reg_val << 1);

	oldi_cfg |= OLDI_SOFTRST;

	oldi_cfg |= OLDI_ENABLE;

	switch (oldi->link_type) {
	case OLDI_MODE_SINGLE_LINK:
		/* All configuration is done for this mode.  */
		break;

	case OLDI_MODE_CLONE_SINGLE_LINK:
		oldi_cfg |= OLDI_CLONE_MODE;
		break;

	case OLDI_MODE_DUAL_LINK:
		/* data-mapping field also indicates dual-link mode */
		oldi_cfg |= BIT(3);
		oldi_cfg |= OLDI_DUALMODESYNC;
		break;

	default:
		dev_err(oldi->dev, "OLDI%d: Unsupported mode.\n",
			oldi->oldi_instance);
		return -EINVAL;
	}

	tidss_configure_oldi(oldi->tidss, oldi->parent_vp, oldi_cfg);
	while (!(oldi_reset_bit & tidss_get_status(oldi->tidss)) &&
	       count < 10000)
		count++;

	if (!(oldi_reset_bit & tidss_get_status(oldi->tidss)))
		dev_warn(oldi->dev, "%s: OLDI%d timeout waiting OLDI reset done\n",
			 __func__, oldi->oldi_instance);

	return 0;
}

static void tidss_oldi_atomic_early_enable(struct drm_bridge *bridge,
					   struct drm_bridge_state *old_bridge_state)
{
	struct tidss_oldi *oldi = drm_bridge_to_tidss_oldi(bridge);
	struct drm_atomic_state *state = old_bridge_state->base.state;
	struct drm_connector *connector;
	struct drm_connector_state *conn_state;
	struct drm_crtc_state *crtc_state;
	struct drm_display_mode *mode;

	if (oldi->tidss->shared_mode)
		return;

	connector = drm_atomic_get_new_connector_for_encoder(state,
							     bridge->encoder);
	if (WARN_ON(!connector))
		return;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (WARN_ON(!conn_state))
		return;

	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	if (WARN_ON(!crtc_state))
		return;

	mode = &crtc_state->adjusted_mode;

	/* Configure the OLDI params*/
	tidss_oldi_config(oldi);

	/* Enable the OLDI serial clock (7 times the pixel clock) */
	oldi_set_serial_clk(oldi, mode->clock * 7 * 1000);

	/* Enable OLDI IO power */
	tidss_oldi_tx_power(oldi, true);
}

static void tidss_oldi_atomic_late_disable(struct drm_bridge *bridge,
					   struct drm_bridge_state *old_bridge_state)
{
	struct tidss_oldi *oldi = drm_bridge_to_tidss_oldi(bridge);

	if (oldi->tidss->shared_mode)
		return;

	/* Disable OLDI IO power */
	tidss_oldi_tx_power(oldi, false);

	/* Disable OLDI clock by setting IDLE Frequency */
	oldi_set_serial_clk(oldi, OLDI_IDLE_CLK_HZ);

	/* Clear OLDI Config */
	tidss_configure_oldi(oldi->tidss, oldi->parent_vp, 0);
}

#define MAX_INPUT_SEL_FORMATS	1

static u32 *tidss_oldi_atomic_get_input_bus_fmts(struct drm_bridge *bridge,
						 struct drm_bridge_state *bridge_state,
						 struct drm_crtc_state *crtc_state,
						 struct drm_connector_state *conn_state,
						 u32 output_fmt,
						 unsigned int *num_input_fmts)
{
	struct tidss_oldi *oldi = drm_bridge_to_tidss_oldi(bridge);
	u32 *input_fmts;
	int i;

	*num_input_fmts = 0;

	for (i = 0; i < ARRAY_SIZE(oldi_bus_formats); i++)
		if (oldi_bus_formats[i].bus_fmt == output_fmt)
			break;

	if (i == ARRAY_SIZE(oldi_bus_formats))
		return NULL;

	input_fmts = kcalloc(MAX_INPUT_SEL_FORMATS, sizeof(*input_fmts),
			     GFP_KERNEL);
	if (!input_fmts)
		return NULL;

	*num_input_fmts = 1;
	input_fmts[0] = oldi_bus_formats[i].input_bus_fmt;
	oldi->bus_format = &oldi_bus_formats[i];

	return input_fmts;
}

static int tidss_oldi_atomic_check(struct drm_bridge *bridge,
				   struct drm_bridge_state *bridge_state,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	/*
	 * There might be flags negotiation supported in future but
	 * set the bus flags in atomic_check statically for now.
	 */

	/* Not sure what this is required for, at the moment */
	bridge_state->input_bus_cfg.flags = bridge->timings->input_bus_flags;

	return 0;
}

static const struct drm_bridge_funcs tidss_oldi_bridge_funcs = {
	.attach		= tidss_oldi_bridge_attach,
	.atomic_check	= tidss_oldi_atomic_check,
	.atomic_early_enable = tidss_oldi_atomic_early_enable,
	.atomic_late_disable = tidss_oldi_atomic_late_disable,
	.atomic_get_input_bus_fmts = tidss_oldi_atomic_get_input_bus_fmts,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
};

static int get_oldi_mode(struct device_node *oldi_tx)
{
	struct device_node *companion;
	struct device_node *port0, *port1;
	int pixel_order;

	/*
	 * Find if the OLDI is paired with another OLDI for combined OLDI
	 * operation (dual-lvds or clone).
	 */
	companion = of_parse_phandle(oldi_tx, "ti,companion-oldi", 0);
	if (!companion) {
		if (of_property_read_bool(oldi_tx, "ti,secondary-oldi"))
			return OLDI_MODE_SECONDARY;

		/*
		 * The OLDI TX does not have a companion, nor is it a
		 * secondary OLDI. It will operate independently.
		 */
		return OLDI_MODE_SINGLE_LINK;
	}

	/*
	 * We need to work out if the sink is expecting us to function in
	 * dual-link mode. We do this by looking at the DT port nodes we are
	 * connected to, if they are marked as expecting even pixels and
	 * odd pixels than we need to enable vertical stripe output.
	 */
	port0 = of_graph_get_port_by_id(oldi_tx, 1);
	port1 = of_graph_get_port_by_id(companion, 1);
	pixel_order = drm_of_lvds_get_dual_link_pixel_order(port0, port1);
	of_node_put(port0);
	of_node_put(port1);
	of_node_put(companion);

	switch (pixel_order) {
	case -EINVAL:
		/*
		 * The dual link properties were not found in at least
		 * one of the sink nodes. Since 2 OLDI ports are present
		 * in the DT, it can be safely assumed that the required
		 * configuration is Clone Mode.
		 */
		return OLDI_MODE_CLONE_SINGLE_LINK;

	case DRM_LVDS_DUAL_LINK_ODD_EVEN_PIXELS:
		return OLDI_MODE_DUAL_LINK;

	/* Unsupported OLDI Modes */
	case DRM_LVDS_DUAL_LINK_EVEN_ODD_PIXELS:
	default:
		return OLDI_MODE_UNSUPPORTED;
	}
}

static u32 get_parent_dss_vp(struct device_node *oldi_tx, u32 *parent_vp)
{
	struct device_node *ep, *dss_port;
	int ret = 0;

	ep = of_graph_get_endpoint_by_regs(oldi_tx, OLDI_INPUT_PORT, -1);
	if (ep) {
		dss_port = of_graph_get_remote_port(ep);
		if (!dss_port) {
			ret = -ENODEV;
			goto err_return_ep_port;
		}

		ret = of_property_read_u32(dss_port, "reg", parent_vp);

		of_node_put(dss_port);
err_return_ep_port:
		of_node_put(ep);
		return ret;
	}

	return -ENODEV;
}

static const struct drm_bridge_timings default_tidss_oldi_timings = {
	.input_bus_flags = DRM_BUS_FLAG_SYNC_SAMPLE_NEGEDGE
			 | DRM_BUS_FLAG_DE_HIGH,
};

void tidss_oldi_deinit(struct tidss_device *tidss)
{
	for (int i = 0; i < tidss->num_oldis; i++) {
		if (tidss->oldis[i]) {
			drm_bridge_remove(&tidss->oldis[i]->bridge);
			tidss->oldis[i] = NULL;
		}
	}
}

int tidss_oldi_init(struct tidss_device *tidss)
{
	struct tidss_oldi *oldi;
	struct device_node *child;
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	u32 parent_vp, oldi_instance;
	enum tidss_oldi_link_type link_type = OLDI_MODE_UNSUPPORTED;
	struct device_node *oldi_parent;
	int ret = 0;

	tidss->num_oldis = 0;

	oldi_parent = of_get_child_by_name(tidss->dev->of_node, "oldi-txes");
	if (!oldi_parent)
		/* Return gracefully */
		return 0;

	for_each_child_of_node(oldi_parent, child) {
		ret = get_parent_dss_vp(child, &parent_vp);
		if (ret) {
			if (ret == -ENODEV) {
				/*
				 * ENODEV means that this particular OLDI node
				 * is not connected with the DSS, which is not
				 * a harmful case. There could be another OLDI
				 * which may still be connected.
				 * Continue to search for that.
				 */
				ret = 0;
				continue;
			}
			goto err_put_node;
		}

		ret = of_property_read_u32(child, "reg", &oldi_instance);
		if (ret)
			goto err_put_node;

		/*
		 * Now that its confirmed that OLDI is connected with DSS, let's
		 * continue getting the OLDI sinks ahead and other OLDI
		 * properties.
		 */
		ret = drm_of_find_panel_or_bridge(child, OLDI_OURPUT_PORT, -1,
						  &panel, &bridge);
		if (ret) {
			/*
			 * Either there was no OLDI sink in the devicetree, or
			 * the OLDI sink has not been added yet. In any case,
			 * return.
			 * We don't want to have an OLDI node connected to DSS
			 * but not to any sink.
			 */
			dev_err_probe(tidss->dev, ret, "no panel/bridge for OLDI%d.\n",
				      oldi_instance);
			goto err_put_node;
		}

		if (panel) {
			bridge = devm_drm_panel_bridge_add(tidss->dev, panel);
			if (IS_ERR(bridge)) {
				ret = PTR_ERR(bridge);
				goto err_put_node;
			}
		}

		link_type = get_oldi_mode(child);
		if (link_type == OLDI_MODE_UNSUPPORTED) {
			ret = dev_err_probe(tidss->dev, -EINVAL,
					    "OLDI%d: Unsupported OLDI connection.\n",
					    oldi_instance);
			goto err_put_node;
		} else if (link_type == OLDI_MODE_SECONDARY) {
			/*
			 * This is the secondary OLDI node, which serves as a
			 * companinon to the primary OLDI, when it is configured
			 * for the dual-lvds mode. Since the primary OLDI will
			 * be a part of bridge chain, no need to put this one
			 * too. Continue onto the next OLDI node.
			 */
			continue;
		}

		oldi = devm_kzalloc(tidss->dev, sizeof(*oldi), GFP_KERNEL);
		if (!oldi) {
			ret = -ENOMEM;
			goto err_put_node;
		}

		oldi->parent_vp = parent_vp;
		oldi->link_type = link_type;
		oldi->oldi_instance = oldi_instance;
		oldi->dev = tidss->dev;
		oldi->next_bridge = bridge;
		oldi->panel = panel;

		oldi->io_ctrl = syscon_regmap_lookup_by_phandle(child,
								"ti,oldi-io-ctrl");
		if (IS_ERR(oldi->io_ctrl)) {
			ret = dev_err_probe(oldi->dev, PTR_ERR(oldi->io_ctrl),
					    "OLDI%d: syscon_regmap_lookup_by_phandle failed.\n",
					    oldi_instance);
			goto err_put_node;
		}

		oldi->s_clk = of_clk_get_by_name(child, "s_clk");
		if (IS_ERR(oldi->s_clk)) {
			ret = dev_err_probe(oldi->dev, PTR_ERR(oldi->s_clk),
					    "OLDI%d: Failed to get serial clock (s_clk).\n",
					    oldi_instance);
			goto err_put_node;
		}

		/* Register the bridge. */
		oldi->bridge.of_node = child;
		oldi->bridge.driver_private = oldi;
		oldi->bridge.funcs = &tidss_oldi_bridge_funcs;
		oldi->bridge.timings = &default_tidss_oldi_timings;

		tidss->oldis[tidss->num_oldis++] = oldi;
		oldi->tidss = tidss;

		drm_bridge_add(&oldi->bridge);
	}

err_put_node:
	of_node_put(child);
	of_node_put(oldi_parent);
	return ret;
}
