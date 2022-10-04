// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 VeriSilicon Holdings Co., Ltd.
 */
#include <linux/version.h>
#include <linux/component.h>
#include <linux/of_device.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 5, 0)
#include <linux/module.h>

#include <drm/drm_bridge.h>
#else
#include <drm/drmP.h>
#endif

#include <drm/drm_crtc_helper.h>
#include <drm/drm_of.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include "vs_crtc.h"
#include "vs_simple_enc.h"

static const struct simple_encoder_priv hdmi_priv = {
    .encoder_type = DRM_MODE_ENCODER_TMDS
};

static const struct simple_encoder_priv dsi_priv = {
    .encoder_type = DRM_MODE_ENCODER_DSI
};

static const struct simple_encoder_priv dpi_priv = {
    .encoder_type = DRM_MODE_ENCODER_DPI
};

static const struct drm_encoder_funcs encoder_funcs = {
    .destroy = drm_encoder_cleanup
};

static inline struct simple_encoder *to_simple_encoder(struct drm_encoder *enc)
{
    return container_of(enc, struct simple_encoder, encoder);
}

static int encoder_parse_dt(struct device *dev)
{
    struct simple_encoder *simple = dev_get_drvdata(dev);
    int ret = 0;
    int cnt, i;
    u32 *vals;
    u32 *masks;

    simple->dss_regmap = syscon_regmap_lookup_by_phandle(dev->of_node,
                        "verisilicon,dss-syscon");

    if (IS_ERR(simple->dss_regmap)) {
        if (PTR_ERR(simple->dss_regmap) != -ENODEV) {
            dev_err(dev, "failed to get dss-syscon\n");
            ret = PTR_ERR(simple->dss_regmap);
            goto err;
        }
        simple->dss_regmap = NULL;
        goto err;
    }

    cnt = of_property_count_elems_of_size(dev->of_node,
                "verisilicon,mux-mask", 4);
    if (!cnt) {
        ret = cnt;
        goto err;
    }

    simple->dss_regdatas = devm_kzalloc(dev,
        sizeof(*simple->dss_regdatas) * cnt, GFP_KERNEL);

    masks = kcalloc(cnt, sizeof(*masks), GFP_KERNEL);
    if (!masks) {
        ret = -ENOMEM;
        goto err;
    }

    vals = kcalloc(cnt, sizeof(*vals), GFP_KERNEL);
    if (!vals) {
        ret = -ENOMEM;
        goto err_free_masks;
    }

    ret = of_property_read_u32_array(
            dev->of_node, "verisilicon,mux-mask", masks, cnt);
    if (ret)
        goto err_free_vals;

    ret = of_property_read_u32_array(
            dev->of_node, "verisilicon,mux-val", vals, cnt);
    if (ret)
        goto err_free_vals;

    for (i = 0; i < cnt; i++) {
        simple->dss_regdatas[i].mask = masks[i];
        simple->dss_regdatas[i].value = vals[i];
    }

err_free_vals:
    kfree(vals);
err_free_masks:
    kfree(masks);
err:
    return ret;
}

void encoder_atomic_enable(struct drm_encoder *encoder,
                        struct drm_atomic_state *state)
{
    struct simple_encoder *simple = to_simple_encoder(encoder);
    struct dss_data *data = simple->dss_regdatas;
    int crtc_id;

    if (!simple->dss_regmap)
        return;

    crtc_id = drm_of_encoder_active_endpoint_id(
                simple->dev->of_node, encoder);

    regmap_update_bits(simple->dss_regmap, 0, data[crtc_id].mask,
            data[crtc_id].value);
}

int encoder_atomic_check(struct drm_encoder *encoder,
            struct drm_crtc_state *crtc_state,
            struct drm_connector_state *conn_state)
{
    struct vs_crtc_state *vs_crtc_state = to_vs_crtc_state(crtc_state);
    struct drm_connector *connector = conn_state->connector;
    int ret = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 9, 0)
    struct drm_bridge *first_bridge = drm_bridge_chain_get_first_bridge(encoder);
    struct drm_bridge_state *bridge_state = ERR_PTR(-EINVAL);
#endif

    vs_crtc_state->encoder_type = encoder->encoder_type;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 9, 0)
    if (first_bridge && first_bridge->funcs->atomic_duplicate_state)
        bridge_state = drm_atomic_get_bridge_state(
                       crtc_state->state, first_bridge);

    if (IS_ERR(bridge_state)) {
        if (connector->display_info.num_bus_formats)
            vs_crtc_state->output_fmt = connector->display_info.bus_formats[0];
        else
            vs_crtc_state->output_fmt = MEDIA_BUS_FMT_FIXED;
    } else {
        vs_crtc_state->output_fmt = bridge_state->input_bus_cfg.format;
    }
#else
    if (connector->display_info.num_bus_formats)
        vs_crtc_state->output_fmt = connector->display_info.bus_formats[0];
    else
        vs_crtc_state->output_fmt = MEDIA_BUS_FMT_RGB888_1X24;
#endif

    switch (vs_crtc_state->output_fmt) {
    case MEDIA_BUS_FMT_FIXED:
    case MEDIA_BUS_FMT_RGB565_1X16:
    case MEDIA_BUS_FMT_RGB666_1X18:
    case MEDIA_BUS_FMT_RGB888_1X24:
    case MEDIA_BUS_FMT_RGB666_1X24_CPADHI:
    case MEDIA_BUS_FMT_RGB101010_1X30:
    case MEDIA_BUS_FMT_UYYVYY8_0_5X24:
    case MEDIA_BUS_FMT_UYVY8_1X16:
    case MEDIA_BUS_FMT_YUV8_1X24:
    case MEDIA_BUS_FMT_UYYVYY10_0_5X30:
    case MEDIA_BUS_FMT_UYVY10_1X20:
    case MEDIA_BUS_FMT_YUV10_1X30:
        ret = 0;
        break;
    default:
        ret = -EINVAL;
        break;
    }

    /* If MEDIA_BUS_FMT_FIXED, set it to default value */
    if (vs_crtc_state->output_fmt == MEDIA_BUS_FMT_FIXED)
        vs_crtc_state->output_fmt = MEDIA_BUS_FMT_RGB888_1X24;

    return ret;
}

static const struct drm_encoder_helper_funcs encoder_helper_funcs = {
    .atomic_enable = encoder_atomic_enable,
    .atomic_check = encoder_atomic_check,
};

static int encoder_bind(struct device *dev, struct device *master, void *data)
{
    struct drm_device *drm_dev = data;
    struct simple_encoder *simple = dev_get_drvdata(dev);
    struct drm_encoder *encoder;
    struct drm_bridge *bridge;
    struct drm_panel *panel;
    int ret;

    encoder = &simple->encoder;

    /* Encoder. */
    ret = drm_encoder_init(drm_dev, encoder, &encoder_funcs,
                   simple->priv->encoder_type, NULL);
    if (ret)
        return ret;

    drm_encoder_helper_add(encoder, &encoder_helper_funcs);

    encoder->possible_crtcs =
            drm_of_find_possible_crtcs(drm_dev, dev->of_node);

    /* output port is port1*/
    ret = drm_of_find_panel_or_bridge(dev->of_node, 1, -1, &panel, &bridge);
    if (ret)
        goto err;

    if (panel) {
	bridge = drm_panel_bridge_add(panel);
	if (IS_ERR(bridge))
	    return PTR_ERR(bridge);
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 7, 0)
    ret = drm_bridge_attach(encoder, bridge, NULL, 0);
#else
    ret = drm_bridge_attach(encoder, bridge, NULL);
#endif
    if (ret)
        goto err;

    return 0;
err:
    drm_encoder_cleanup(encoder);

    return ret;
}

static void encoder_unbind(struct device *dev, struct device *master,
                  void *data)
{
    struct simple_encoder *simple = dev_get_drvdata(dev);

    drm_encoder_cleanup(&simple->encoder);
}

static const struct component_ops encoder_component_ops = {
    .bind = encoder_bind,
    .unbind = encoder_unbind,
};

static const struct of_device_id simple_encoder_dt_match[] = {
    { .compatible = "verisilicon,hdmi-encoder", .data = &hdmi_priv},
    { .compatible = "verisilicon,dp-encoder", .data = &hdmi_priv},
    { .compatible = "verisilicon,dsi-encoder", .data = &dsi_priv},
    { .compatible = "verisilicon,dpi-encoder", .data = &dpi_priv},
    {},
};
MODULE_DEVICE_TABLE(of, simple_encoder_dt_match);

static int encoder_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct simple_encoder *simple;
    int ret;

    simple = devm_kzalloc(dev, sizeof(*simple), GFP_KERNEL);
    if (!simple)
        return -ENOMEM;

    simple->priv = of_device_get_match_data(dev);

    simple->dev = dev;

    dev_set_drvdata(dev, simple);

    ret = encoder_parse_dt(dev);
    if (ret)
        return ret;

    return component_add(dev, &encoder_component_ops);
}

static int encoder_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;

    component_del(dev, &encoder_component_ops);

    dev_set_drvdata(dev, NULL);

    return 0;
}

struct platform_driver simple_encoder_driver = {
    .probe = encoder_probe,
    .remove = encoder_remove,
    .driver = {
        .name = "vs-simple-encoder",
        .of_match_table = of_match_ptr(simple_encoder_dt_match),
    },
};

MODULE_DESCRIPTION("Simple Encoder Driver");
MODULE_LICENSE("GPL v2");
