/*
 * Copyright (C) 2016 Texas Instruments Ltd
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "omapdss.h"
#include "dss6.h"

struct dpi_data {
	struct platform_device *pdev;

	struct mutex lock;

	struct omap_video_timings timings;
	int data_lines;

	struct omap_dss_device output;

	bool port_initialized;
};

static struct dpi_data *dpi6_get_data_from_dssdev(struct omap_dss_device *out)
{
	return container_of(out, struct dpi_data, output);
}

static void dpi6_config_lcd_manager(struct dpi_data *dpi)
{
	struct dss_lcd_mgr_config mgr_config = {
		.io_pad_mode = DSS_IO_PAD_MODE_BYPASS,
		.stallmode = false,
		.fifohandcheck = false,
		.video_port_width = dpi->data_lines,
		.lcden_sig_polarity = 0,
	};

	dss_mgr_set_lcd_config(dpi->output.dispc_channel, &mgr_config);
}

static int dpi6_connect(struct omap_dss_device *out,
			struct omap_dss_device *dst)
{
	struct dpi_data *dpi = dpi6_get_data_from_dssdev(out);
	enum omap_channel channel = out->dispc_channel;
	int r;

	r = dss_mgr_connect(channel, out);
	if (r)
		return r;

	r = omapdss_output_set_device(out, dst);
	if (r) {
		dev_err(&dpi->pdev->dev,
			"failed to connect output to new device: %s\n",
			dst->name);
		dss_mgr_disconnect(channel, out);
		return r;
	}

	return 0;
}

static void dpi6_disconnect(struct omap_dss_device *out,
			    struct omap_dss_device *dst)
{
	WARN_ON(dst != out->dst);

	if (dst != out->dst)
		return;

	omapdss_output_unset_device(out);

	dss_mgr_disconnect(out->dispc_channel, out);
}

static int dpi6_display_enable(struct omap_dss_device *out)
{
	struct dpi_data *dpi = dpi6_get_data_from_dssdev(out);
	enum omap_channel channel = out->dispc_channel;
	int r;

	mutex_lock(&dpi->lock);

	if (!out->dispc_channel_connected) {
		dev_err(&dpi->pdev->dev, "failed to enable display: no output channel set\n");
		r = -ENODEV;
		goto err_no_out_mgr;
	}

	r = dispc6_runtime_get();
	if (r)
		goto err_get_dispc;

	r = dispc6_vp_set_clk_rate(channel, dpi->timings.pixelclock);
	if (r)
		goto err_set_clk;

	r = dispc6_vp_enable_clk(channel);
	if (r)
		goto err_enable_clk;

	dpi6_config_lcd_manager(dpi);

	r = dss_mgr_enable(channel);
	if (r)
		goto err_mgr_enable;

	dss6_ungate_dpi_clk(&dpi->pdev->dev, out->port_num);

	mutex_unlock(&dpi->lock);

	return 0;

err_mgr_enable:
	dispc6_vp_disable_clk(channel);
err_enable_clk:
err_set_clk:
	dispc6_runtime_put();
err_get_dispc:
err_no_out_mgr:
	mutex_unlock(&dpi->lock);
	return r;
}

static void dpi6_display_disable(struct omap_dss_device *out)
{
	struct dpi_data *dpi = dpi6_get_data_from_dssdev(out);
	enum omap_channel channel = out->dispc_channel;

	mutex_lock(&dpi->lock);

	dss6_gate_dpi_clk(&dpi->pdev->dev, out->port_num);

	dss_mgr_disable(channel);

	dispc6_vp_disable_clk(channel);

	dispc6_runtime_put();

	mutex_unlock(&dpi->lock);
}

static int dpi6_check_timings(struct omap_dss_device *out,
			      struct omap_video_timings *timings)
{
	enum omap_channel channel = out->dispc_channel;

	if (!dispc6_mgr_timings_ok(channel, timings))
		return -EINVAL;

	return 0;
}

static void dpi6_set_timings(struct omap_dss_device *out,
			     struct omap_video_timings *timings)
{
	struct dpi_data *dpi = dpi6_get_data_from_dssdev(out);

	mutex_lock(&dpi->lock);

	dpi->timings = *timings;

	mutex_unlock(&dpi->lock);
}

static void dpi6_get_timings(struct omap_dss_device *out,
			     struct omap_video_timings *timings)
{
	struct dpi_data *dpi = dpi6_get_data_from_dssdev(out);

	mutex_lock(&dpi->lock);

	*timings = dpi->timings;

	mutex_unlock(&dpi->lock);
}

static void dpi6_set_data_lines(struct omap_dss_device *out, int data_lines)
{
	struct dpi_data *dpi = dpi6_get_data_from_dssdev(out);

	mutex_lock(&dpi->lock);

	dpi->data_lines = data_lines;

	mutex_unlock(&dpi->lock);
}

static const struct omapdss_dpi_ops dpi6_ops = {
	.connect = dpi6_connect,
	.disconnect = dpi6_disconnect,

	.enable = dpi6_display_enable,
	.disable = dpi6_display_disable,

	.check_timings = dpi6_check_timings,
	.set_timings = dpi6_set_timings,
	.get_timings = dpi6_get_timings,

	.set_data_lines = dpi6_set_data_lines,
};

static void dpi6_setup_output_port(struct platform_device *pdev,
				   struct device_node *port)
{
	struct dpi_data *dpi = port->data;
	struct omap_dss_device *out = &dpi->output;
	int r;
	u32 port_num;

	r = of_property_read_u32(port, "reg", &port_num);
	if (r)
		port_num = 0;

	switch (port_num) {
	case 0:
	default:
		out->name = "dpi.0";
		break;
	}

	out->dev = &pdev->dev;
	out->id = OMAP_DSS_OUTPUT_DPI;
	out->output_type = OMAP_DISPLAY_TYPE_DPI;
	out->dispc_channel = OMAP_DSS_CHANNEL_LCD;
	out->port_num = port_num;
	out->ops.dpi = &dpi6_ops;
	out->owner = THIS_MODULE;

	omapdss_register_output(out);
}

int dpi6_init_port(struct platform_device *pdev, struct device_node *port)
{
	struct dpi_data *dpi;
	struct device_node *ep;
	u32 datalines;
	int r;

	dpi = devm_kzalloc(&pdev->dev, sizeof(*dpi), GFP_KERNEL);
	if (!dpi)
		return -ENOMEM;

	ep = omapdss_of_get_next_endpoint(port, NULL);
	if (!ep)
		return 0;

	r = of_property_read_u32(ep, "data-lines", &datalines);
	if (r) {
		dev_err(&dpi->pdev->dev, "failed to parse datalines\n");
		goto err_datalines;
	}

	dpi->data_lines = datalines;

	of_node_put(ep);

	dpi->pdev = pdev;
	port->data = dpi;

	mutex_init(&dpi->lock);

	dpi6_setup_output_port(pdev, port);

	dpi->port_initialized = true;

	return 0;

err_datalines:
	of_node_put(ep);

	return r;
}

void dpi6_uninit_port(struct device_node *port)
{
	struct dpi_data *dpi = port->data;

	if (!dpi->port_initialized)
		return;

	omapdss_unregister_output(&dpi->output);
}
