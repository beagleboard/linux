/* tilcdc_fbdev.c
 *
 * Copyright (c) 2014 B. Scott Michel
 * Author: B. Scott Michel (bscottm@ieee.org)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * NOTE: This code provides the minimal DRM scaffolding copied from the
 * drm_fb_cma_helper.c source needed to hook into struct fb_ops.fb_check_var
 * function pointer.
 */

#include "tilcdc_drv.h"
#include "tilcdc_fbdev.h"

/*
 * These structures were copied from the drm_fb_cma_helper.c source, they
 * are bitwise equivalent:
 *
 * tilcdc_fb_cma -> drm_fb_cma
 * tilcdc_drm_fbdev -> drm_fbdev_cma
 */
struct tilcdc_fb_cma {
	struct drm_framebuffer		fb;
	struct drm_gem_cma_object	*obj[4];
};

struct tilcdc_drm_fbdev {
	struct drm_fb_helper	drm_fb_helper;
	struct tilcdc_fb_cma	*fb;
};

static inline struct tilcdc_drm_fbdev *to_tilcdc_fbdev(struct drm_fb_helper *helper)
{
	return container_of(helper, struct tilcdc_drm_fbdev, drm_fb_helper);
}

static inline struct tilcdc_fb_cma *to_tilcdc_fb_cma(struct drm_framebuffer *fb)
{
	return container_of(fb, struct tilcdc_fb_cma, fb);
}

static int tilcdc_fbdev_check_var(struct fb_var_screeninfo *var,
				  struct fb_info *info)
{
	int ret, depth;
	__u32 offs;
	struct drm_fb_helper *helper = info->par;
	struct drm_device *dev = helper->dev;
	struct tilcdc_drm_private *priv = dev->dev_private;

	ret = drm_fb_helper_check_var(var, info);
	if (ret)
		return ret;

	/* Calculate current depth, prepare for RGBx->BGRx swap, if requested */
	switch (var->bits_per_pixel) {
	case 16:
		depth = (var->green.length == 6) ? 16 : 15;
		break;
	case 32:
		depth = (var->transp.length > 0) ? 32 : 24;
		break;
	default:
		depth = var->bits_per_pixel;
		break;
	}

	DBG("depth: %d", depth);

	if ((depth == 16 && priv->bgrx_16bpp_swap) ||
	    (depth == 24 && priv->bgrx_24bpp_swap)) {
		/* RGBx -> BGRx reversal */
		offs = var->red.offset;
		var->red.offset = var->blue.offset;
		var->blue.offset = offs;
	}

	return 0;
}

/* framebuffer operations adapted from drm_fb_cma_helper.c */
static struct fb_ops tilcdc_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
	.fb_check_var	= tilcdc_fbdev_check_var,
	.fb_set_par	= drm_fb_helper_set_par,
	.fb_blank	= drm_fb_helper_blank,
	.fb_pan_display	= drm_fb_helper_pan_display,
	.fb_setcmap	= drm_fb_helper_setcmap,
};

static int tilcdc_fb_cma_create_handle(struct drm_framebuffer *fb,
	struct drm_file *file_priv, unsigned int *handle)
{
	struct tilcdc_fb_cma *fb_cma = to_tilcdc_fb_cma(fb);

	return drm_gem_handle_create(file_priv,
			&fb_cma->obj[0]->base, handle);
}

static void tilcdc_fb_cma_destroy(struct drm_framebuffer *fb)
{
	struct tilcdc_fb_cma *fb_cma = to_tilcdc_fb_cma(fb);
	int i;

	for (i = 0; i < 4; i++) {
		if (fb_cma->obj[i])
			drm_gem_object_unreference_unlocked(&fb_cma->obj[i]->base);
	}

	drm_framebuffer_cleanup(fb);
	kfree(fb_cma);
}

static struct drm_framebuffer_funcs tilcdc_fb_cma_funcs = {
	.destroy	= tilcdc_fb_cma_destroy,
	.create_handle	= tilcdc_fb_cma_create_handle,
};

static struct drm_fb_helper_funcs tilcdc_fb_cma_helper_funcs = {
	.fb_probe = tilcdc_drm_fbdev_probe,
};

/* Copied from drm_fb_cma_helper.c */
static struct tilcdc_fb_cma *tilcdc_fb_cma_alloc(struct drm_device *dev,
	struct drm_mode_fb_cmd2 *mode_cmd, struct drm_gem_cma_object **obj,
	unsigned int num_planes)
{
	struct tilcdc_fb_cma *fb_cma;
	int ret;
	int i;

	DBG("allocating fb_cma");

	fb_cma = kzalloc(sizeof(*fb_cma), GFP_KERNEL);
	if (!fb_cma)
		return ERR_PTR(-ENOMEM);

	ret = drm_framebuffer_init(dev, &fb_cma->fb, &tilcdc_fb_cma_funcs);
	if (ret) {
		dev_err(dev->dev, "Failed to initalize framebuffer: %d\n", ret);
		kfree(fb_cma);
		return ERR_PTR(ret);
	}

	drm_helper_mode_fill_fb_struct(&fb_cma->fb, mode_cmd);

	for (i = 0; i < num_planes; i++)
		fb_cma->obj[i] = obj[i];

	return fb_cma;
}

static int tilcdc_drm_fbdev_create(struct drm_fb_helper *helper,
				   struct drm_fb_helper_surface_size *sizes)
{
	struct tilcdc_drm_fbdev *fbdev_cma = to_tilcdc_fbdev(helper);
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };
	struct drm_device *dev = helper->dev;
	struct drm_gem_cma_object *obj;
	struct drm_framebuffer *fb;
	unsigned int bytes_per_pixel;
	unsigned long offset;
	struct fb_info *fbi;
	size_t size;
	int ret;

	DRM_DEBUG_KMS("surface width(%d), height(%d) and bpp(%d)\n",
			sizes->surface_width, sizes->surface_height,
			sizes->surface_bpp);

	bytes_per_pixel = DIV_ROUND_UP(sizes->surface_bpp, 8);

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;
	mode_cmd.pitches[0] = sizes->surface_width * bytes_per_pixel;
	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
		sizes->surface_depth);

	size = mode_cmd.pitches[0] * mode_cmd.height;
	obj = drm_gem_cma_create(dev, size);
	if (IS_ERR(obj))
		return -ENOMEM;

	fbi = framebuffer_alloc(0, dev->dev);
	if (!fbi) {
		dev_err(dev->dev, "Failed to allocate framebuffer info.\n");
		ret = -ENOMEM;
		goto err_drm_gem_cma_free_object;
	}

	fbi->par = helper;
	fbi->flags = FBINFO_DEFAULT;
	fbi->fbops = &tilcdc_fb_ops;

	fbdev_cma->fb = tilcdc_fb_cma_alloc(dev, &mode_cmd, &obj, 1);
	if (IS_ERR(fbdev_cma->fb)) {
		dev_err(dev->dev, "Failed to allocate DRM framebuffer.\n");
		ret = PTR_ERR(fbdev_cma->fb);
		goto err_framebuffer_release;
	}

	fb = &fbdev_cma->fb->fb;
	helper->fb = fb;
	helper->fbdev = fbi;

	ret = fb_alloc_cmap(&fbi->cmap, 256, 0);
	if (ret) {
		dev_err(dev->dev, "Failed to allocate color map.\n");
		goto err_drm_fb_cma_destroy;
	}

	drm_fb_helper_fill_fix(fbi, fb->pitches[0], fb->depth);
	drm_fb_helper_fill_var(fbi, helper, fb->width, fb->height);

	offset = fbi->var.xoffset * bytes_per_pixel;
	offset += fbi->var.yoffset * fb->pitches[0];

	dev->mode_config.fb_base = (resource_size_t)obj->paddr;
	fbi->screen_base = obj->vaddr + offset;
	fbi->fix.smem_start = (unsigned long)(obj->paddr + offset);
	fbi->screen_size = size;
	fbi->fix.smem_len = size;

	return 0;

err_drm_fb_cma_destroy:
	tilcdc_fb_cma_destroy(fb);
err_framebuffer_release:
	framebuffer_release(fbi);
err_drm_gem_cma_free_object:
	drm_gem_cma_free_object(&obj->base);
	return ret;
}

int tilcdc_drm_fbdev_probe(struct drm_fb_helper *helper,
			   struct drm_fb_helper_surface_size *sizes)
{
	int ret = 0;

	if (!helper->fb) {
		ret = tilcdc_drm_fbdev_create(helper, sizes);
		if (ret < 0) {
			DRM_ERROR("failed to create fbdev.\n");
			return ret;
		}

		ret = 1;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(tilcdc_drm_fbdev_probe);

struct tilcdc_drm_fbdev *tilcdc_fbdev_cma_init(struct drm_device *dev,
	unsigned int preferred_bpp, unsigned int num_crtc,
	unsigned int max_conn_count)
{
	struct tilcdc_drm_fbdev *fbdev_cma;
	struct drm_fb_helper *helper;
	int ret;

	fbdev_cma = kzalloc(sizeof(*fbdev_cma), GFP_KERNEL);
	if (!fbdev_cma) {
		dev_err(dev->dev, "Failed to allocate drm fbdev.\n");
		return ERR_PTR(-ENOMEM);
	}

	fbdev_cma->drm_fb_helper.funcs = &tilcdc_fb_cma_helper_funcs;
	helper = &fbdev_cma->drm_fb_helper;

	ret = drm_fb_helper_init(dev, helper, num_crtc, max_conn_count);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to initialize drm fb helper.\n");
		goto err_free;
	}

	ret = drm_fb_helper_single_add_all_connectors(helper);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to add connectors.\n");
		goto err_drm_fb_helper_fini;

	}

	ret = drm_fb_helper_initial_config(helper, preferred_bpp);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to set inital hw configuration.\n");
		goto err_drm_fb_helper_fini;
	}

	return fbdev_cma;

err_drm_fb_helper_fini:
	drm_fb_helper_fini(helper);
err_free:
	kfree(fbdev_cma);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(tilcdc_fbdev_cma_init);
