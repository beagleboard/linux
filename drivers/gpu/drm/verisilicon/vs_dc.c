// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 VeriSilicon Holdings Co., Ltd.
 */

#include <linux/component.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/media-bus-format.h>
#include <linux/of_graph.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>

#include <drm/drm_framebuffer.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/vs_drm.h>

#include "vs_type.h"
#include "vs_dc_hw.h"
#include "vs_dc.h"
#include "vs_crtc.h"
#include "vs_drv.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 5, 0)
#include <drm/drm_fourcc.h>
#include <drm/drm_vblank.h>
#endif

static inline void update_format(u32 format, u64 mod, struct dc_hw_fb *fb)
{
    u8 f = FORMAT_A8R8G8B8;

    switch (format) {
    case DRM_FORMAT_XRGB4444:
    case DRM_FORMAT_RGBX4444:
    case DRM_FORMAT_XBGR4444:
    case DRM_FORMAT_BGRX4444:
        f = FORMAT_X4R4G4B4;
        break;
    case DRM_FORMAT_ARGB4444:
    case DRM_FORMAT_RGBA4444:
    case DRM_FORMAT_ABGR4444:
    case DRM_FORMAT_BGRA4444:
        f = FORMAT_A4R4G4B4;
        break;
    case DRM_FORMAT_XRGB1555:
    case DRM_FORMAT_RGBX5551:
    case DRM_FORMAT_XBGR1555:
    case DRM_FORMAT_BGRX5551:
        f = FORMAT_X1R5G5B5;
        break;
    case DRM_FORMAT_ARGB1555:
    case DRM_FORMAT_RGBA5551:
    case DRM_FORMAT_ABGR1555:
    case DRM_FORMAT_BGRA5551:
        f = FORMAT_A1R5G5B5;
        break;
    case DRM_FORMAT_RGB565:
    case DRM_FORMAT_BGR565:
        f = FORMAT_R5G6B5;
        break;
    case DRM_FORMAT_XRGB8888:
    case DRM_FORMAT_RGBX8888:
    case DRM_FORMAT_XBGR8888:
    case DRM_FORMAT_BGRX8888:
        f = FORMAT_X8R8G8B8;
        break;
    case DRM_FORMAT_ARGB8888:
    case DRM_FORMAT_RGBA8888:
    case DRM_FORMAT_ABGR8888:
    case DRM_FORMAT_BGRA8888:
        f = FORMAT_A8R8G8B8;
        break;
    case DRM_FORMAT_YUYV:
    case DRM_FORMAT_YVYU:
        f = FORMAT_YUY2;
        break;
    case DRM_FORMAT_UYVY:
    case DRM_FORMAT_VYUY:
        f = FORMAT_UYVY;
        break;
    case DRM_FORMAT_YUV420:
    case DRM_FORMAT_YVU420:
        f = FORMAT_YV12;
        break;
    case DRM_FORMAT_NV21:
        f = FORMAT_NV12;
        break;
    case DRM_FORMAT_NV16:
    case DRM_FORMAT_NV61:
        f = FORMAT_NV16;
        break;
    case DRM_FORMAT_P010:
        f = FORMAT_P010;
        break;
    case DRM_FORMAT_ARGB2101010:
    case DRM_FORMAT_RGBA1010102:
    case DRM_FORMAT_ABGR2101010:
    case DRM_FORMAT_BGRA1010102:
        f = FORMAT_A2R10G10B10;
        break;
    case DRM_FORMAT_NV12:
        if (fourcc_mod_vs_get_type(mod) ==
            DRM_FORMAT_MOD_VS_TYPE_CUSTOM_10BIT)
            f = FORMAT_NV12_10BIT;
        else
            f = FORMAT_NV12;
        break;
    case DRM_FORMAT_YUV444:
        if (fourcc_mod_vs_get_type(mod) ==
            DRM_FORMAT_MOD_VS_TYPE_CUSTOM_10BIT)
            f = FORMAT_YUV444_10BIT;
        else
            f = FORMAT_YUV444;
        break;
    default:
        break;
    }

    fb->format = f;
}

static inline void update_swizzle(u32 format, struct dc_hw_fb *fb)
{
    fb->swizzle = SWIZZLE_ARGB;
    fb->uv_swizzle = 0;

    switch (format) {
    case DRM_FORMAT_RGBX4444:
    case DRM_FORMAT_RGBA4444:
    case DRM_FORMAT_RGBX5551:
    case DRM_FORMAT_RGBA5551:
    case DRM_FORMAT_RGBX8888:
    case DRM_FORMAT_RGBA8888:
    case DRM_FORMAT_RGBA1010102:
        fb->swizzle = SWIZZLE_RGBA;
        break;
    case DRM_FORMAT_XBGR4444:
    case DRM_FORMAT_ABGR4444:
    case DRM_FORMAT_XBGR1555:
    case DRM_FORMAT_ABGR1555:
    case DRM_FORMAT_BGR565:
    case DRM_FORMAT_XBGR8888:
    case DRM_FORMAT_ABGR8888:
    case DRM_FORMAT_ABGR2101010:
        fb->swizzle = SWIZZLE_ABGR;
        break;
    case DRM_FORMAT_BGRX4444:
    case DRM_FORMAT_BGRA4444:
    case DRM_FORMAT_BGRX5551:
    case DRM_FORMAT_BGRA5551:
    case DRM_FORMAT_BGRX8888:
    case DRM_FORMAT_BGRA8888:
    case DRM_FORMAT_BGRA1010102:
        fb->swizzle = SWIZZLE_BGRA;
        break;
    case DRM_FORMAT_YVYU:
    case DRM_FORMAT_VYUY:
    case DRM_FORMAT_NV21:
    case DRM_FORMAT_NV61:
        fb->uv_swizzle = 1;
        break;
    default:
        break;
    }
}

static inline void update_watermark(struct drm_property_blob *watermark,
                                    struct dc_hw_fb *fb)
{
    struct drm_vs_watermark *data;
    fb->water_mark = 0;

    if (watermark) {
        data = watermark->data;
        fb->water_mark = data->watermark & 0xFFFFF;
    }
}

static inline u8 to_vs_rotation(unsigned int rotation)
{
    u8 rot;

    switch (rotation & DRM_MODE_REFLECT_MASK) {
    case DRM_MODE_REFLECT_X:
        rot = FLIP_X;
        return rot;
    case DRM_MODE_REFLECT_Y:
        rot = FLIP_Y;
        return rot;
    case DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y:
        rot = FLIP_XY;
        return rot;
    default:
        break;
    }

    switch (rotation & DRM_MODE_ROTATE_MASK) {
    case DRM_MODE_ROTATE_0:
        rot = ROT_0;
        break;
    case DRM_MODE_ROTATE_90:
        rot = ROT_90;
        break;
    case DRM_MODE_ROTATE_180:
        rot = ROT_180;
        break;
    case DRM_MODE_ROTATE_270:
        rot = ROT_270;
        break;
    default:
        rot = ROT_0;
        break;
    }

    return rot;
}

static inline u8 to_vs_yuv_color_space(u32 color_space)
{
    u8 cs;

    switch (color_space) {
    case DRM_COLOR_YCBCR_BT601:
        cs = COLOR_SPACE_601;
        break;
    case DRM_COLOR_YCBCR_BT709:
        cs = COLOR_SPACE_709;
        break;
    case DRM_COLOR_YCBCR_BT2020:
        cs = COLOR_SPACE_2020;
        break;
    default:
        cs = COLOR_SPACE_601;
        break;
    }

    return cs;
}

static inline u8 to_vs_tile_mode(u64 modifier)
{
    return (u8)(modifier & DRM_FORMAT_MOD_VS_NORM_MODE_MASK);
}

static inline u8 to_vs_display_id(struct vs_dc *dc, struct drm_crtc *crtc)
{
    u8 panel_num = dc->hw.info->panel_num;
    u32 index = drm_crtc_index(crtc);
    int i;

    for (i = 0; i < panel_num; i++) {
        if (index == dc->crtc[i]->base.index)
            return i;
    }

    return 0;
}

static void dc_deinit(struct device *dev)
{
    struct vs_dc *dc = dev_get_drvdata(dev);

    pm_runtime_get_sync(dev);

    dc_hw_enable_interrupt(&dc->hw, 0);
    dc_hw_deinit(&dc->hw);

    pm_runtime_put(dev);
}

static int dc_init(struct device *dev)
{
    struct vs_dc *dc = dev_get_drvdata(dev);
    int ret = 0, i;

    dc->first_frame = true;

    for (i = 0; i < DC_DISPLAY_NUM; i++)
	dc->pix_clk_rate[i] = clk_get_rate(dc->pixclk[i]) / 1000;

    pm_runtime_get_sync(dev);
    ret = dc_hw_init(&dc->hw);
    if (ret) {
        dev_err(dev, "failed to init DC HW\n");
	goto out;
    }

out:
    pm_runtime_put(dev);

    return ret;
}

static void vs_dc_dump_enable(struct device *dev, dma_addr_t addr,
                   unsigned int pitch)
{
    struct vs_dc *dc = dev_get_drvdata(dev);

    dc_hw_enable_dump(&dc->hw, addr, pitch);
}

static void vs_dc_dump_disable(struct device *dev)
{
    struct vs_dc *dc = dev_get_drvdata(dev);

    dc_hw_disable_dump(&dc->hw);
}

static void vs_dc_enable(struct device *dev, struct drm_crtc *crtc)
{
    struct vs_dc *dc = dev_get_drvdata(dev);
    struct vs_crtc_state *crtc_state = to_vs_crtc_state(crtc->state);
    struct drm_display_mode *mode = &crtc->state->adjusted_mode;
    struct dc_hw_display display;

    display.bus_format = crtc_state->output_fmt;
    display.h_active = mode->hdisplay;
    display.h_total = mode->htotal;
    display.h_sync_start = mode->hsync_start;
    display.h_sync_end = mode->hsync_end;
    if (mode->flags & DRM_MODE_FLAG_PHSYNC)
        display.h_sync_polarity = true;
    else
        display.h_sync_polarity = false;

    display.v_active = mode->vdisplay;
    display.v_total = mode->vtotal;
    display.v_sync_start = mode->vsync_start;
    display.v_sync_end = mode->vsync_end;
    if (mode->flags & DRM_MODE_FLAG_PVSYNC)
        display.v_sync_polarity = true;
    else
        display.v_sync_polarity = false;

    display.sync_mode = crtc_state->sync_mode;
    display.bg_color = crtc_state->bg_color;

    display.id = to_vs_display_id(dc, crtc);
    display.sync_enable = crtc_state->sync_enable;
    display.dither_enable = crtc_state->dither_enable;

    display.enable = true;

    if (dc->pix_clk_rate[display.id] != mode->clock) {
	clk_set_rate(dc->pixclk[display.id], mode->clock * 1000);
        dc->pix_clk_rate[display.id] = mode->clock;
    }

    if (crtc_state->encoder_type == DRM_MODE_ENCODER_DSI ||
	crtc_state->encoder_type == DRM_MODE_ENCODER_DPI)
        dc_hw_set_out(&dc->hw, OUT_DPI, display.id);
    else
        dc_hw_set_out(&dc->hw, OUT_DP, display.id);

#ifdef CONFIG_VERISILICON_MMU
    if (crtc_state->mmu_prefetch == VS_MMU_PREFETCH_ENABLE)
        dc_hw_enable_mmu_prefetch(&dc->hw, true);
    else
        dc_hw_enable_mmu_prefetch(&dc->hw, false);
#endif

    dc_hw_setup_display(&dc->hw, &display);
}

static void vs_dc_disable(struct device *dev, struct drm_crtc *crtc)
{
    struct vs_dc *dc = dev_get_drvdata(dev);
    struct dc_hw_display display;

    display.id = to_vs_display_id(dc, crtc);
    display.enable = false;

    dc_hw_setup_display(&dc->hw, &display);
}

static bool vs_dc_mode_fixup(struct device *dev,
                  struct drm_crtc *crtc,
                  const struct drm_display_mode *mode,
                  struct drm_display_mode *adjusted_mode)
{
    struct vs_dc *dc = dev_get_drvdata(dev);
    int id = to_vs_display_id(dc, crtc);
    long clk_rate;

    if (unlikely(id >= DC_DISPLAY_NUM)) {
	dev_err(dev, "invalid display id : %d\n", id);
	return false;
    }

    if (dc->pixclk[id]) {
        clk_rate = clk_round_rate(dc->pixclk[id],
                      adjusted_mode->clock * 1000);
        adjusted_mode->clock = DIV_ROUND_UP(clk_rate, 1000);
    }

    return true;
}

static void vs_dc_set_gamma(struct device *dev, struct drm_crtc *crtc,
                 struct drm_color_lut *lut, unsigned int size)
{
    struct vs_dc *dc = dev_get_drvdata(dev);
    u16 i, r, g, b;
    u8 bits, id;

    if (size != dc->hw.info->gamma_size) {
        dev_err(dev, "gamma size does not match!\n");
        return;
    }

    id = to_vs_display_id(dc, crtc);

    bits = dc->hw.info->gamma_bits;
    for (i = 0; i < size; i++) {
        r = drm_color_lut_extract(lut[i].red, bits);
        g = drm_color_lut_extract(lut[i].green, bits);
        b = drm_color_lut_extract(lut[i].blue, bits);
        dc_hw_update_gamma(&dc->hw, id, i, r, g, b);
    }
}

static void vs_dc_enable_gamma(struct device *dev, struct drm_crtc *crtc,
                 bool enable)
{
    struct vs_dc *dc = dev_get_drvdata(dev);
    u8 id;

    id = to_vs_display_id(dc, crtc);
    dc_hw_enable_gamma(&dc->hw, id, enable);
}

static void vs_dc_enable_vblank(struct device *dev, struct drm_crtc *crtc,
		bool enable)
{
    struct vs_dc *dc = dev_get_drvdata(dev);

    dc_hw_enable_irq(&dc->hw, crtc->index, enable);
}

static u32 calc_factor(u32 src, u32 dest)
{
    u32 factor = 1 << 16;

    if ((src > 1) && (dest > 1))
        factor = ((src - 1) << 16) / (dest - 1);

    return factor;
}

static void update_scale(struct drm_plane_state *state, struct dc_hw_roi *roi,
                         struct dc_hw_scale *scale)
{
    int dst_w = drm_rect_width(&state->dst);
    int dst_h = drm_rect_height(&state->dst);
    int src_w, src_h, temp;
    scale->enable = false;

    if (roi->enable) {
        src_w = roi->width;
        src_h = roi->height;
    } else {
        src_w = drm_rect_width(&state->src) >> 16;
        src_h = drm_rect_height(&state->src) >> 16;
    }

    if (drm_rotation_90_or_270(state->rotation)) {
        temp = src_w;
        src_w = src_h;
        src_h = temp;
    }

    if (src_w != dst_w) {
        scale->scale_factor_x = calc_factor(src_w, dst_w);
        scale->enable = true;
    } else {
        scale->scale_factor_x = 1 << 16;
    }
    if (src_h != dst_h) {
        scale->scale_factor_y = calc_factor(src_h, dst_h);
        scale->enable = true;
    } else {
        scale->scale_factor_y = 1 << 16;
    }
}

static void update_fb(struct vs_plane *plane, u8 display_id,
                      struct dc_hw_fb *fb)
{
    struct drm_plane_state *state = plane->base.state;
    struct vs_plane_state *plane_state = to_vs_plane_state(state);
    struct drm_framebuffer *drm_fb = state->fb;
    struct drm_rect *src = &state->src;

    fb->display_id = display_id;
    fb->y_address = plane->dma_addr[0];
    fb->y_stride = drm_fb->pitches[0];
    if (drm_fb->format->format == DRM_FORMAT_YVU420) {
        fb->u_address = plane->dma_addr[2];
        fb->v_address = plane->dma_addr[1];
        fb->u_stride = drm_fb->pitches[2];
        fb->v_stride = drm_fb->pitches[1];
    } else {
        fb->u_address = plane->dma_addr[1];
        fb->v_address = plane->dma_addr[2];
        fb->u_stride = drm_fb->pitches[1];
        fb->v_stride = drm_fb->pitches[2];
    }
    fb->width = drm_rect_width(src) >> 16;
    fb->height = drm_rect_height(src) >> 16;
    fb->tile_mode = to_vs_tile_mode(drm_fb->modifier);
    fb->rotation = to_vs_rotation(state->rotation);
    fb->yuv_color_space = to_vs_yuv_color_space(state->color_encoding);
    fb->zpos = state->zpos;
    fb->enable = state->visible;
    update_format(drm_fb->format->format, drm_fb->modifier, fb);
    update_swizzle(drm_fb->format->format, fb);
    update_watermark(plane_state->watermark, fb);

    plane_state->status.tile_mode = fb->tile_mode;
}

#ifdef CONFIG_VERISILICON_DEC
static u8 get_stream_base(u8 id)
{
    u8 stream_base = 0;

    switch (id) {
    case OVERLAY_PLANE_0:
        stream_base = 3;
        break;
    case OVERLAY_PLANE_1:
        stream_base = 6;
        break;
    case PRIMARY_PLANE_1:
        stream_base = 16;
        break;
    case OVERLAY_PLANE_2:
        stream_base = 19;
        break;
    case OVERLAY_PLANE_3:
        stream_base = 22;
        break;
    default:
        break;
    }

    return stream_base;
}

static void update_fbc(struct vs_dc *dc, struct vs_plane *plane, bool *enable)
{
    struct dc_dec_fb dec_fb;
    struct drm_plane_state *state = plane->base.state;
    struct drm_framebuffer *drm_fb = state->fb;
    struct vs_dc_plane *dc_plane = &dc->planes[plane->id];
    u8 i, stream_id;

    if (!dc->hw.info->cap_dec) {
        *enable = false;
        return;
    }

    stream_id = get_stream_base(dc_plane->id);
    memset(&dec_fb, 0, sizeof(struct dc_dec_fb));
    dec_fb.fb = drm_fb;

    if (fourcc_mod_vs_get_type(drm_fb->modifier) !=
                    DRM_FORMAT_MOD_VS_TYPE_COMPRESSED) {
        *enable = false;
    } else {
        *enable = true;

        for (i = 0; i < DEC_PLANE_MAX; i++) {
            dec_fb.addr[i] = plane->dma_addr[i];
            dec_fb.stride[i] = drm_fb->pitches[i];
        }
    }

    dc_dec_config(&dc->dec400l, &dec_fb, stream_id);
}

static void disable_fbc(struct vs_dc *dc, struct vs_plane *plane)
{
    struct vs_dc_plane *dc_plane = &dc->planes[plane->id];
    u8 stream_id;

    if (!dc->hw.info->cap_dec)
        return;

    stream_id = get_stream_base(dc_plane->id);
    dc_dec_config(&dc->dec400l, NULL, stream_id);
}
#endif

static void update_degamma(struct vs_dc *dc, struct vs_plane *plane,
               struct vs_plane_state *plane_state)
{
    dc_hw_update_degamma(&dc->hw, plane->id, plane_state->degamma);
    plane_state->degamma_changed = false;
}

static void update_roi(struct vs_dc *dc, u8 id,
                       struct vs_plane_state *plane_state,
                       struct dc_hw_roi *roi)
{
    struct drm_vs_roi *data;
    struct drm_rect *src = &plane_state->base.src;
    u16 src_w = drm_rect_width(src) >> 16;
    u16 src_h = drm_rect_height(src) >> 16;

    if (plane_state->roi) {
        data = plane_state->roi->data;

        if (data->enable) {
            roi->x = data->roi_x;
            roi->y = data->roi_y;
            roi->width = (data->roi_x + data->roi_w > src_w) ?
                         (src_w - data->roi_x) : data->roi_w;
            roi->height = (data->roi_y + data->roi_h > src_h) ?
                          (src_h - data->roi_y) : data->roi_h;
            roi->enable = true;
        } else {
            roi->enable = false;
        }

        dc_hw_update_roi(&dc->hw, id, roi);
    } else {
        roi->enable = false;
    }
}

static void update_color_mgmt(struct vs_dc *dc, u8 id,
                            struct dc_hw_fb *fb,
                            struct vs_plane_state *plane_state)
{
    struct drm_vs_color_mgmt *data;
    struct dc_hw_colorkey colorkey;

    if (plane_state->color_mgmt) {
        data = plane_state->color_mgmt->data;

        fb->clear_enable = data->clear_enable;
        fb->clear_value = data->clear_value;

        if (data->colorkey > data->colorkey_high)
            data->colorkey = data->colorkey_high;

        colorkey.colorkey = data->colorkey;
        colorkey.colorkey_high = data->colorkey_high;
        colorkey.transparency = (data->transparency) ?
                                DC_TRANSPARENCY_KEY : DC_TRANSPARENCY_OPAQUE;
        dc_hw_update_colorkey(&dc->hw, id, &colorkey);
    }
}

static void update_plane(struct vs_dc *dc, struct vs_plane *plane)
{
    struct dc_hw_fb fb = {0};
    struct dc_hw_scale scale;
    struct dc_hw_position pos;
    struct dc_hw_blend blend;
    struct dc_hw_roi roi;
    struct drm_plane_state *state = plane->base.state;
    struct vs_plane_state *plane_state = to_vs_plane_state(state);
    struct drm_rect *dest = &state->dst;
    bool dec_enable = false;
    u8 display_id = 0;

#ifdef CONFIG_VERISILICON_DEC
    update_fbc(dc, plane, &dec_enable);
#endif

    display_id = to_vs_display_id(dc, state->crtc);
    update_fb(plane, display_id, &fb);
    fb.dec_enable = dec_enable;


    update_roi(dc, plane->id, plane_state, &roi);

    update_scale(state, &roi, &scale);

    if (plane_state->degamma_changed)
        update_degamma(dc, plane, plane_state);

    pos.start_x = dest->x1;
    pos.start_y = dest->y1;
    pos.end_x = dest->x2;
    pos.end_y = dest->y2;

    blend.alpha = (u8)(state->alpha >> 8);
    blend.blend_mode = (u8)(state->pixel_blend_mode);

    update_color_mgmt(dc, plane->id, &fb, plane_state);

    dc_hw_update_plane(&dc->hw, plane->id, &fb, &scale, &pos, &blend);
}

static void update_qos(struct vs_dc *dc, struct vs_plane *plane)
{
    struct drm_plane_state *state = plane->base.state;
    struct vs_plane_state *plane_state = to_vs_plane_state(state);
    struct drm_vs_watermark *data;
    struct dc_hw_qos qos;

    if (plane_state->watermark){
        data = plane_state->watermark->data;

        if (data->qos_high) {
            if (data->qos_low > data->qos_high)
                data->qos_low = data->qos_high;

            qos.low_value = data->qos_low & 0x0F;
            qos.high_value = data->qos_high & 0x0F;
            dc_hw_update_qos(&dc->hw, &qos);
        }
    }
}

static void update_cursor_size(struct drm_plane_state *state, struct dc_hw_cursor *cursor)
{
    u8 size_type;

    switch (state->crtc_w) {
    case 32:
        size_type = CURSOR_SIZE_32X32;
        break;
    case 64:
        size_type = CURSOR_SIZE_64X64;
        break;
    default:
        size_type = CURSOR_SIZE_32X32;
        break;
    }

    cursor->size = size_type;
}

static void update_cursor_plane(struct vs_dc *dc, struct vs_plane *plane)
{
    struct drm_plane_state *state = plane->base.state;
    struct drm_framebuffer *drm_fb = state->fb;
    struct dc_hw_cursor cursor;

    cursor.address = plane->dma_addr[0];
    cursor.x = state->crtc_x;
    cursor.y = state->crtc_y;
    cursor.hot_x = drm_fb->hot_x;
    cursor.hot_y = drm_fb->hot_y;
    cursor.display_id = to_vs_display_id(dc, state->crtc);
    update_cursor_size(state, &cursor);
    cursor.enable = true;

    dc_hw_update_cursor(&dc->hw, cursor.display_id, &cursor);
}

static void vs_dc_update_plane(struct device *dev, struct vs_plane *plane)
{
    struct vs_dc *dc = dev_get_drvdata(dev);
    enum drm_plane_type type = plane->base.type;

    switch (type) {
    case DRM_PLANE_TYPE_PRIMARY:
    case DRM_PLANE_TYPE_OVERLAY:
        update_plane(dc, plane);
        update_qos(dc, plane);
        break;
    case DRM_PLANE_TYPE_CURSOR:
        update_cursor_plane(dc, plane);
        break;
    default:
        break;
    }
}

static void vs_dc_disable_plane(struct device *dev, struct vs_plane *plane,
                                struct drm_plane_state *old_state)
{
    struct vs_dc *dc = dev_get_drvdata(dev);
    enum drm_plane_type type = plane->base.type;
    struct dc_hw_fb fb = {0};
    struct dc_hw_cursor cursor = {0};

    switch (type) {
    case DRM_PLANE_TYPE_PRIMARY:
    case DRM_PLANE_TYPE_OVERLAY:
        fb.enable = false;
        dc_hw_update_plane(&dc->hw, plane->id, &fb, NULL, NULL, NULL);
#ifdef CONFIG_VERISILICON_DEC
        disable_fbc(dc, plane);
#endif
        break;
    case DRM_PLANE_TYPE_CURSOR:
        cursor.enable = false;
        cursor.display_id = to_vs_display_id(dc, old_state->crtc);
        dc_hw_update_cursor(&dc->hw, cursor.display_id, &cursor);
        break;
    default:
        break;
    }
}

static bool vs_dc_mod_supported(const struct vs_plane_info *plane_info,
                                u64 modifier)
{
    const u64 *mods;

    if (plane_info->modifiers == NULL)
        return false;

    for(mods = plane_info->modifiers; *mods != DRM_FORMAT_MOD_INVALID; mods++) {
        if (*mods == modifier)
            return true;
    }

    return false;
}

static int vs_dc_check_plane(struct device *dev, struct vs_plane *plane,
                  struct drm_plane_state *state)
{
    struct vs_dc *dc = dev_get_drvdata(dev);
    struct drm_framebuffer *fb = state->fb;
    const struct vs_plane_info *plane_info;
    struct drm_crtc *crtc = state->crtc;
    struct drm_crtc_state *crtc_state;

    plane_info = &dc->hw.info->planes[plane->id];
    if (plane_info == NULL)
        return -EINVAL;

    if (fb->width < plane_info->min_width ||
        fb->width > plane_info->max_width ||
        fb->height < plane_info->min_height ||
        fb->height > plane_info->max_height)
        dev_err_once(dev, "buffer size may not support on plane%d.\n",
                 plane->id);

    if ((plane->base.type != DRM_PLANE_TYPE_CURSOR) &&
        (!vs_dc_mod_supported(plane_info, fb->modifier))) {
        dev_err(dev, "unsupported modifier on plane%d.\n", plane->id);
        return -EINVAL;
    }

    crtc_state = drm_atomic_get_existing_crtc_state(state->state, crtc);
    if (IS_ERR(crtc_state))
        return -EINVAL;

    /* 90/270 degree rotation requires non-linear fb */
    if ((state->rotation == DRM_MODE_ROTATE_90 ||
	state->rotation == DRM_MODE_ROTATE_270) &&
	(fb->modifier & DRM_FORMAT_MOD_LINEAR) == DRM_FORMAT_MOD_LINEAR)
	return -EINVAL;

    return drm_atomic_helper_check_plane_state(state, crtc_state,
                          plane_info->min_scale,
                          plane_info->max_scale,
                          true, true);
}

static irqreturn_t dc_isr(int irq, void *data)
{
    struct vs_dc *dc = data;
    struct vs_dc_info *dc_info = dc->hw.info;
    u32 i, intr_vec;

    intr_vec = dc_hw_get_interrupt(&dc->hw);

    for (i = 0; i < dc_info->panel_num; i++) {
	if (intr_vec & BIT(i))
	    vs_crtc_handle_vblank(&dc->crtc[i]->base,
			dc_hw_check_underflow(&dc->hw));
    }

    if (intr_vec & ~0x3)
	pr_warn("%s: unhandled interrupt %#x\n", __func__, intr_vec);

    return IRQ_HANDLED;
}

static void vs_dc_commit(struct device *dev)
{
    struct vs_dc *dc = dev_get_drvdata(dev);

#ifdef CONFIG_VERISILICON_DEC
    if (dc->hw.info->cap_dec)
        dc_dec_commit(&dc->dec400l, &dc->hw);
#endif

    dc_hw_enable_shadow_register(&dc->hw, false);

    dc_hw_commit(&dc->hw);

    if (dc->first_frame)
        dc->first_frame = false;

    dc_hw_enable_shadow_register(&dc->hw, true);
}

static const struct vs_crtc_funcs dc_crtc_funcs = {
    .enable         = vs_dc_enable,
    .disable        = vs_dc_disable,
    .mode_fixup     = vs_dc_mode_fixup,
    .set_gamma      = vs_dc_set_gamma,
    .enable_gamma       = vs_dc_enable_gamma,
    .enable_vblank      = vs_dc_enable_vblank,
    .commit         = vs_dc_commit,
};

static const struct vs_plane_funcs dc_plane_funcs = {
    .update         = vs_dc_update_plane,
    .disable        = vs_dc_disable_plane,
    .check          = vs_dc_check_plane,
};

static const struct vs_dc_funcs dc_funcs = {
    .dump_enable        = vs_dc_dump_enable,
    .dump_disable       = vs_dc_dump_disable,
};

static int dc_bind(struct device *dev, struct device *master, void *data)
{
    struct drm_device *drm_dev = data;
#ifdef CONFIG_VERISILICON_MMU
    struct vs_drm_private *priv = drm_dev->dev_private;
#endif
    struct vs_dc *dc = dev_get_drvdata(dev);
    struct device_node *port;
    struct vs_crtc *crtc;
    struct drm_crtc *drm_crtc;
    struct vs_dc_info *dc_info;
    struct vs_plane *plane;
    struct drm_plane *drm_plane, *tmp;
    struct vs_plane_info *plane_info;
    int i, ret;
    u32 ctrc_mask = 0;

    if (!drm_dev || !dc) {
        dev_err(dev, "devices are not created.\n");
        return -ENODEV;
    }

    ret = dc_init(dev);
    if (ret < 0) {
        dev_err(dev, "Failed to initialize DC hardware.\n");
        return ret;
    }

#ifdef CONFIG_VERISILICON_MMU
    ret = dc_mmu_construct(priv->dma_dev, &priv->mmu);
    if (ret) {
        dev_err(dev, "failed to construct DC MMU\n");
        goto err_clean_dc;
    }

    ret = dc_hw_mmu_init(&dc->hw, priv->mmu);
    if (ret) {
        dev_err(dev, "failed to init DC MMU\n");
        goto err_clean_dc;
    }
#endif

    ret = vs_drm_iommu_attach_device(drm_dev, dev);
    if (ret < 0) {
        dev_err(dev, "Failed to attached iommu device.\n");
        goto err_clean_dc;
    }

    if (!of_graph_is_present(dev->of_node)) {
        dev_err(dev, "no port node found\n");
	ret = -ENODEV;
        goto err_detach_dev;
    }

    dc_info = dc->hw.info;

    for(i = 0; i < dc_info->panel_num; i++) {
	port = of_graph_get_port_by_id(dev->of_node, i);
	if (!port) {
	    dev_err(dev, "no port node found for panel %d\n", i);
	    ret = -ENODEV;
	    goto err_detach_dev;
	};

        crtc = vs_crtc_create(drm_dev, dc_info);
        if (!crtc) {
            dev_err(dev, "Failed to create CRTC.\n");
            ret = -ENOMEM;
            goto err_detach_dev;
        }

        crtc->base.port = port;
        crtc->dev = dev;
        crtc->funcs = &dc_crtc_funcs;
        dc->crtc[i] = crtc;
        ctrc_mask |= drm_crtc_mask(&crtc->base);
    }

    for (i = 0; i < dc_info->plane_num; i++) {
        plane_info = (struct vs_plane_info *)&dc_info->planes[i];

        if (!strcmp(plane_info->name, "Primary") ||
            !strcmp(plane_info->name, "Cursor"))
            plane = vs_plane_create(drm_dev, plane_info, dc_info->layer_num,
                                    drm_crtc_mask(&dc->crtc[0]->base));
        else if (!strcmp(plane_info->name, "Primary_1") ||
                 !strcmp(plane_info->name, "Cursor_1"))
            plane = vs_plane_create(drm_dev, plane_info, dc_info->layer_num,
                                    drm_crtc_mask(&dc->crtc[1]->base));
        else
            plane = vs_plane_create(drm_dev, plane_info,
                                    dc_info->layer_num, ctrc_mask);

        if (!plane)
            goto err_cleanup_planes;

        plane->id = i;
        dc->planes[i].id = plane_info->id;

        plane->funcs = &dc_plane_funcs;

        if (plane_info->type == DRM_PLANE_TYPE_PRIMARY) {
            if (!strcmp(plane_info->name, "Primary"))
                dc->crtc[0]->base.primary = &plane->base;
            else
                dc->crtc[1]->base.primary = &plane->base;
            drm_dev->mode_config.min_width = plane_info->min_width;
            drm_dev->mode_config.min_height =
                            plane_info->min_height;
            drm_dev->mode_config.max_width = plane_info->max_width;
            drm_dev->mode_config.max_height =
                            plane_info->max_height;
        }

        if (plane_info->type == DRM_PLANE_TYPE_CURSOR) {
            if (!strcmp(plane_info->name, "Cursor"))
                dc->crtc[0]->base.cursor = &plane->base;
            else
                dc->crtc[1]->base.cursor = &plane->base;
            drm_dev->mode_config.cursor_width =
                            plane_info->max_width;
            drm_dev->mode_config.cursor_height =
                            plane_info->max_height;
        }
    }

    dc->funcs = &dc_funcs;

    vs_drm_update_pitch_alignment(drm_dev, dc_info->pitch_alignment);
    return 0;

err_cleanup_planes:
    list_for_each_entry_safe(drm_plane, tmp,
                 &drm_dev->mode_config.plane_list, head)
        if (drm_plane->possible_crtcs & ctrc_mask)
            vs_plane_destory(drm_plane);

    drm_for_each_crtc(drm_crtc, drm_dev)
        vs_crtc_destroy(drm_crtc);
err_detach_dev:
    vs_drm_iommu_detach_device(drm_dev, dev);
err_clean_dc:
    dc_deinit(dev);
    return ret;
}

static void dc_unbind(struct device *dev, struct device *master, void *data)
{
    struct drm_device *drm_dev = data;

    dc_deinit(dev);

    vs_drm_iommu_detach_device(drm_dev, dev);
}

const struct component_ops dc_component_ops = {
    .bind = dc_bind,
    .unbind = dc_unbind,
};

static const struct of_device_id dc_driver_dt_match[] = {
    { .compatible = "verisilicon,dc8200", },
    {},
};
MODULE_DEVICE_TABLE(of, dc_driver_dt_match);

static void dc_get_display_pll(struct device *dev, struct vs_dc *dc)
{
	struct device_node *np;

	np = of_find_node_by_name(NULL, "dw-mipi-dsi0");
	if (!np)
		dev_err(dev, "Failed to get dsi0\n");
	else
		dc->dpu0pll_on = of_device_is_available(np);

	np = of_find_node_by_name(NULL, "dw-mipi-dsi1");
	if (!np)
		dev_err(dev, "Failed to get dsi1\n");
	else
		dc->dpu1pll_on = of_device_is_available(np);

	/* dsi1/hdmi share the same pll1, hdmi detect again if dsi1 not use */
	if (!dc->dpu1pll_on) {
		np = of_find_node_by_name(NULL, "dw-hdmi-tx");
		if (!np)
			dev_err(dev, "Failed to get hdmi\n");
		else
			dc->dpu1pll_on = of_device_is_available(np);
	}


	dev_info(dev, "dpu0pll_on:%d dpu1pll_on:%d\n", dc->dpu0pll_on,
			dc->dpu1pll_on);
}

static int dc_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct vs_dc *dc;
    int irq, ret, i;
    char pixclk[16];

    dc = devm_kzalloc(dev, sizeof(*dc), GFP_KERNEL);
    if (!dc)
        return -ENOMEM;

    dc->hw.hi_base = devm_platform_ioremap_resource(pdev, 0);
    if (IS_ERR(dc->hw.hi_base))
        return PTR_ERR(dc->hw.hi_base);

    dc->hw.reg_base = devm_platform_ioremap_resource(pdev, 1);
    if (IS_ERR(dc->hw.reg_base))
        return PTR_ERR(dc->hw.reg_base);

#ifdef CONFIG_VERISILICON_MMU
    dc->hw.mmu_base = devm_platform_ioremap_resource(pdev, 2);
    if (IS_ERR(dc->hw.mmu_base))
        return PTR_ERR(dc->hw.mmu_base);
#endif

    irq = platform_get_irq(pdev, 0);
    ret = devm_request_irq(dev, irq, dc_isr, 0, dev_name(dev), dc);
    if (ret < 0) {
        dev_err(dev, "Failed to install irq:%u.\n", irq);
        return ret;
    }

    dc->core_clk = devm_clk_get_optional(dev, "core_clk");
    if (IS_ERR(dc->core_clk)) {
        dev_err(dev, "failed to get core_clk source\n");
        return PTR_ERR(dc->core_clk);
    }

    for (i = 0; i < DC_DISPLAY_NUM; i++) {
	snprintf(pixclk, ARRAY_SIZE(pixclk), "%s%d", "pix_clk", i);
	dc->pix_clk[i] = devm_clk_get_optional(dev, pixclk);

	if (IS_ERR(dc->pix_clk[i])) {
	    dev_err(dev, "failed to get pix_clk %d source\n", i);
	    return PTR_ERR(dc->pix_clk[i]);
	}

	snprintf(pixclk, ARRAY_SIZE(pixclk), "%s%d", "pixclk", i);
	dc->pixclk[i] = devm_clk_get_optional(dev, pixclk);
	if (IS_ERR(dc->pixclk[i])) {
	    dev_err(dev, "failed to get pixclk %d source\n", i);
	    return PTR_ERR(dc->pixclk[i]);
	}
    }

    dc->axi_clk = devm_clk_get_optional(dev, "axi_clk");
    if (IS_ERR(dc->axi_clk)) {
        dev_err(dev, "failed to get axi_clk source\n");
        return PTR_ERR(dc->axi_clk);
    }

    dc->cfg_clk = devm_clk_get_optional(dev, "cfg_clk");
    if (IS_ERR(dc->cfg_clk)) {
        dev_err(dev, "failed to get cfg_clk source\n");
        return PTR_ERR(dc->cfg_clk);
    }

    dc->dpu0pll_clk = devm_clk_get_optional(dev, "dpu0_pll_foutpostdiv");
    if (IS_ERR(dc->dpu0pll_clk)) {
        dev_err(dev, "failed to get dpu0pll_clk source\n");
        return PTR_ERR(dc->dpu0pll_clk);
    }

    dc->dpu1pll_clk = devm_clk_get_optional(dev, "dpu1_pll_foutpostdiv");
    if (IS_ERR(dc->dpu1pll_clk)) {
        dev_err(dev, "failed to get dpu1pll_clk source\n");
        return PTR_ERR(dc->dpu1pll_clk);
    }

    dc_get_display_pll(dev, dc);

    dev_set_drvdata(dev, dc);
    pm_runtime_enable(dev);

    return component_add(dev, &dc_component_ops);
}

static int dc_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;

    component_del(dev, &dc_component_ops);

    pm_runtime_disable(dev);

    dev_set_drvdata(dev, NULL);

    return 0;
}

#ifdef CONFIG_PM
static int dc_runtime_suspend(struct device *dev)
{
	int i;
	struct vs_dc *dc = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	clk_disable_unprepare(dc->axi_clk);

	for (i = 0; i < DC_DISPLAY_NUM; i++)
		clk_disable_unprepare(dc->pix_clk[i]);

	clk_disable_unprepare(dc->core_clk);

	clk_disable_unprepare(dc->cfg_clk);

	if (dc->dpu0pll_on)
		clk_disable_unprepare(dc->dpu0pll_clk);
	if (dc->dpu1pll_on)
		clk_disable_unprepare(dc->dpu1pll_clk);

	return 0;
}

static int dc_runtime_resume(struct device *dev)
{
	int i, ret;
	struct vs_dc *dc = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	if (dc->dpu0pll_on)
		clk_prepare_enable(dc->dpu0pll_clk);
	if (dc->dpu1pll_on)
		clk_prepare_enable(dc->dpu1pll_clk);

	ret = clk_prepare_enable(dc->cfg_clk);
	if (ret < 0) {
		dev_err(dev, "failed to prepare/enable cfg_clk\n");
		return ret;
	}

	ret = clk_prepare_enable(dc->core_clk);
	if (ret < 0) {
		dev_err(dev, "failed to prepare/enable core_clk\n");
		return ret;
	}

	for (i = 0; i < DC_DISPLAY_NUM; i++) {
		ret = clk_prepare_enable(dc->pix_clk[i]);

		if (ret < 0) {
			dev_err(dev, "failed to prepare/enable pix_clk %d\n", i);
			return ret;
		}
	}

	ret = clk_prepare_enable(dc->axi_clk);
	if (ret < 0) {
		dev_err(dev, "failed to prepare/enable axi_clk\n");
		return ret;
	}

	return 0;
}
#endif

static const struct dev_pm_ops dc_pm_ops = {
    SET_LATE_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				 pm_runtime_force_resume)
    SET_RUNTIME_PM_OPS(dc_runtime_suspend, dc_runtime_resume, NULL)
};

struct platform_driver dc_platform_driver = {
    .probe = dc_probe,
    .remove = dc_remove,
    .driver = {
        .name = "vs-dc",
        .of_match_table = of_match_ptr(dc_driver_dt_match),
        .pm = &dc_pm_ops,
    },
};

MODULE_DESCRIPTION("VeriSilicon DC Driver");
MODULE_LICENSE("GPL v2");
