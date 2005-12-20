/*
 * File: drivers/video/omap/omapfb_main.c
 *
 * Framebuffer driver for TI OMAP boards
 *
 * Copyright (C) 2004 Nokia Corporation
 * Author: Imre Deak <imre.deak@nokia.com>
 *
 * Acknowledgements:
 *   Alex McMains <aam@ridgerun.com>       - Original driver
 *   Juha Yrjola <juha.yrjola@nokia.com>   - Original driver and improvements
 *   Dirk Behme <dirk.behme@de.bosch.com>  - changes for 2.6 kernel API
 *   Texas Instruments                     - H3 support
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/mach-types.h>

#include <asm/arch/dma.h>
#include <asm/arch/irqs.h>
#include <asm/arch/mux.h>
#include <asm/arch/board.h>
#include <asm/arch/omapfb.h>

/* #define OMAPFB_DBG 1 */

#include "debug.h"

#define OMAPFB_DRIVER	"omapfb"
#define MODULE_NAME	"omapfb"

#define pr_err(fmt, args...) printk(KERN_ERR MODULE_NAME ": " fmt, ## args)

static unsigned int	def_accel;
static unsigned long	def_vram;
static unsigned long	def_vxres;
static unsigned long	def_vyres;
static unsigned int	def_rotate;
static unsigned int	def_mirror;

#ifdef CONFIG_FB_OMAP_MANUAL_UPDATE
static int		manual_update = 1;
#else
static int		manual_update;
#endif

static struct caps_table_struct {
        unsigned long flag;
        const char *name;
} omapfb_caps_table[] = {
	{ OMAPFB_CAPS_MANUAL_UPDATE, "manual update" },
	{ OMAPFB_CAPS_SET_BACKLIGHT, "backlight setting" },
};

/*
 * ---------------------------------------------------------------------------
 * LCD panel
 * ---------------------------------------------------------------------------
 */
extern struct lcd_panel h4_panel;
extern struct lcd_panel h3_panel;
extern struct lcd_panel h2_panel;
extern struct lcd_panel p2_panel;
extern struct lcd_panel osk_panel;
extern struct lcd_panel palmte_panel;
extern struct lcd_panel innovator1610_panel;
extern struct lcd_panel innovator1510_panel;
extern struct lcd_panel lph8923_panel;

static struct lcd_panel *panels[] = {
#ifdef CONFIG_MACH_OMAP_H2
	&h2_panel,
#endif
#ifdef CONFIG_MACH_OMAP_H3
	&h3_panel,
#endif
#ifdef CONFIG_MACH_OMAP_H4
	&h4_panel,
#endif
#ifdef CONFIG_MACH_OMAP_PERSEUS2
	&p2_panel,
#endif
#ifdef CONFIG_MACH_OMAP_OSK
	&osk_panel,
#endif
#ifdef CONFIG_MACH_OMAP_PALMTE
	&palmte_panel,
#endif

#ifdef CONFIG_MACH_OMAP_INNOVATOR

#ifdef CONFIG_ARCH_OMAP15XX
	&innovator1510_panel,
#endif
#ifdef CONFIG_ARCH_OMAP16XX
	&innovator1610_panel,
#endif

#endif
};

extern struct lcd_ctrl omap1_int_ctrl;
extern struct lcd_ctrl omap2_int_ctrl;
extern struct lcd_ctrl hwa742_ctrl;
extern struct lcd_ctrl blizzard_ctrl;

static struct lcd_ctrl *ctrls[] = {
#ifdef CONFIG_FB_OMAP_LCDC_INTERNAL
#ifdef CONFIG_ARCH_OMAP1
	&omap1_int_ctrl,
#else
	&omap2_int_ctrl,
#endif
#endif
};

#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
#ifdef CONFIG_ARCH_OMAP1
extern struct lcd_ctrl_extif sossi_extif;
#else
extern struct lcd_ctrl_extif rfbi_extif;
#endif
#endif

static void omapfb_rqueue_lock(struct omapfb_device *fbdev)
{
	down(&fbdev->rqueue_sema);
}

static void omapfb_rqueue_unlock(struct omapfb_device *fbdev)
{
	up(&fbdev->rqueue_sema);
}

/*
 * ---------------------------------------------------------------------------
 * LCD controller and LCD DMA
 * ---------------------------------------------------------------------------
 */
/* Lookup table to map elem size to elem type. */
static const int dma_elem_type[] = {
	0,
	OMAP_DMA_DATA_TYPE_S8,
	OMAP_DMA_DATA_TYPE_S16,
	0,
	OMAP_DMA_DATA_TYPE_S32,
};

/* Allocate resources needed for LCD controller and LCD DMA operations. Video
 * memory is allocated from system memory according to the virtual display
 * size, except if a bigger memory size is specified explicitly as a kernel
 * parameter.
 */
static int ctrl_init(struct omapfb_device *fbdev)
{
	int r;

	DBGENTER(1);

	r = fbdev->ctrl->init(fbdev, 0, def_vram);
	if (r < 0) {
		pr_err("controller initialization failed\n");
		goto exit;
	}

	fbdev->ctrl->get_vram_layout(&fbdev->vram_size, &fbdev->vram_virt_base,
				     &fbdev->vram_phys_base);
	memset((void *)fbdev->vram_virt_base, 0, fbdev->vram_size);

	DBGPRINT(1, "vram_phys %08x vram_virt %p vram_size=%lu\n",
		 fbdev->vram_phys_base, fbdev->vram_virt_base,
		 fbdev->vram_size);

	DBGLEAVE(1);
	return 0;
exit:
	DBGLEAVE(1);
	return r;
}

static void ctrl_cleanup(struct omapfb_device *fbdev)
{
	fbdev->ctrl->cleanup();
}

static int ctrl_change_mode(struct omapfb_device *fbdev)
{
	int r;
	unsigned long offset;
	struct fb_var_screeninfo *var = &fbdev->fb_info->var;

	DBGPRINT(1, "xoffset %d yoffset %d line_length %d bits_per_pixel %d\n",
		var->xoffset, var->yoffset, fbdev->fb_info->fix.line_length,
		var->bits_per_pixel);
	offset = var->yoffset * fbdev->fb_info->fix.line_length +
		 var->xoffset * var->bits_per_pixel / 8;
	r = fbdev->ctrl->setup_plane(OMAPFB_PLANE_GFX, OMAPFB_CHANNEL_OUT_LCD,
				 offset, var->xres_virtual, 0, 0, var->xres,
				 var->yres, fbdev->color_mode);
	DBGLEAVE(1);

	return r;
}

/*
 * ---------------------------------------------------------------------------
 * fbdev framework callbacks and the ioctl interface
 * ---------------------------------------------------------------------------
 */
/* Called each time the omapfb device is opened */
static int omapfb_open(struct fb_info *info, int user)
{
	DBGENTER(1);
	DBGLEAVE(1);
	return 0;
}

static void omapfb_sync(struct fb_info *info);

/* Called when the omapfb device is closed. We make sure that any pending
 * gfx DMA operations are ended, before we return. */
static int omapfb_release(struct fb_info *info, int user)
{
	DBGENTER(1);

	omapfb_sync(info);

	DBGLEAVE(1);
	return 0;
}

/* Store a single color palette entry into a pseudo palette or the hardware
 * palette if one is available. For now we support only 16bpp and thus store
 * the entry only to the pseudo palette.
 */
static int _setcolreg(struct fb_info *info, u_int regno, u_int red, u_int green,
			u_int blue, u_int transp, int update_hw_pal)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)info->par;
	int r = 0;

	switch (fbdev->color_mode) {
	case OMAPFB_COLOR_YUV422:
	case OMAPFB_COLOR_YUV420:
		r = -EINVAL;
		break;
	case OMAPFB_COLOR_CLUT_8BPP:
	case OMAPFB_COLOR_CLUT_4BPP:
	case OMAPFB_COLOR_CLUT_2BPP:
	case OMAPFB_COLOR_CLUT_1BPP:
		if (fbdev->ctrl->setcolreg)
			r = fbdev->ctrl->setcolreg(regno, red, green, blue,
							transp, update_hw_pal);
		/* Fallthrough */
	case OMAPFB_COLOR_RGB565:
		if (r != 0)
			break;

		if (regno < 0) {
			r = -EINVAL;
			break;
		}

		if (regno < 16) {
			u16 pal;
			pal = ((red >> 11) << 11) | ((green >> 10) << 5) |
				(blue >> 11);
			((u32 *)(info->pseudo_palette))[regno] = pal;
		}
		break;
	default:
		BUG();
	}
	return r;
}

static int omapfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			    u_int transp, struct fb_info *info)
{
	int r = 0;

	DBGENTER(2);

	_setcolreg(info, regno, red, green, blue, transp, 1);

	DBGLEAVE(2);

	return r;
}

static int omapfb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	int count, index, r;
	u16 *red, *green, *blue, *transp;
	u16 trans = 0xffff;

	red     = cmap->red;
	green   = cmap->green;
	blue    = cmap->blue;
	transp  = cmap->transp;
	index   = cmap->start;

	for (count = 0; count < cmap->len; count++) {
		if (transp)
			trans = *transp++;
		r = _setcolreg(info, index++, *red++, *green++, *blue++, trans,
				count == cmap->len - 1);
		if (r != 0)
			return r;
	}

	return 0;
}


static void omapfb_update_full_screen(struct omapfb_device *fbdev);

static int omapfb_blank(int blank, struct fb_info *fbi)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;
	int r = 0;

	DBGENTER(1);

	omapfb_rqueue_lock(fbdev);
	switch (blank) {
	case VESA_NO_BLANKING:
		if (fbdev->state == OMAPFB_SUSPENDED) {
			if (fbdev->ctrl->resume)
				fbdev->ctrl->resume();
			fbdev->panel->enable();
			fbdev->state = OMAPFB_ACTIVE;
			if (fbdev->ctrl->get_update_mode() ==
					OMAPFB_MANUAL_UPDATE)
				omapfb_update_full_screen(fbdev);
		}
		break;
	case VESA_POWERDOWN:
		if (fbdev->state == OMAPFB_ACTIVE) {
			fbdev->panel->disable();
			if (fbdev->ctrl->suspend)
				fbdev->ctrl->suspend();
			fbdev->state = OMAPFB_SUSPENDED;
		}
		break;
	default:
		r = -EINVAL;
	}
	omapfb_rqueue_unlock(fbdev);

	DBGLEAVE(1);
	return r;
}

static void omapfb_sync(struct fb_info *fbi)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;

	omapfb_rqueue_lock(fbdev);
	if (fbdev->ctrl->sync)
		fbdev->ctrl->sync();
	omapfb_rqueue_unlock(fbdev);
}

/* Set fb_info.fix fields and also updates fbdev.
 * When calling this fb_info.var must be set up already.
 */
static void set_fb_fix(struct omapfb_device *fbdev)
{
	struct fb_info		 *fbi = fbdev->fb_info;
	struct fb_fix_screeninfo *fix = &fbi->fix;
	struct fb_var_screeninfo *var = &fbi->var;

	strncpy(fix->id, OMAPFB_DRIVER, sizeof(fix->id));
	fix->type = FB_TYPE_PACKED_PIXELS;
	switch (var->bits_per_pixel) {
	case 16:
		fix->visual = FB_VISUAL_TRUECOLOR;
		break;
	case 1:
	case 2:
	case 4:
	case 8:
		fix->visual = FB_VISUAL_PSEUDOCOLOR;
		break;
	}
	fix->accel		= FB_ACCEL_OMAP1610;
	fix->line_length	= var->xres_virtual * var->bits_per_pixel / 8;
	fix->smem_len		= fbdev->vram_size;
	fix->smem_start		= fbdev->vram_phys_base;
}

/* Check the values in var against our capabilities and in case of out of
 * bound values try to adjust them.
 */
static int set_fb_var(struct omapfb_device *fbdev,
		      struct fb_var_screeninfo *var)
{
	int		bpp;
	unsigned long	max_frame_size;
	unsigned long	line_size;
	struct lcd_panel *panel = fbdev->panel;

	bpp = var->bits_per_pixel = panel->bpp;

	switch (bpp) {
	case 16:
		fbdev->color_mode = OMAPFB_COLOR_RGB565;
		break;
	case 8:
		fbdev->color_mode = OMAPFB_COLOR_CLUT_8BPP;
		break;
	default:
		/* FIXME: other BPPs not yet supported */
		return -EINVAL;
	}

	switch (var->rotate) {
	case 0:
	case 180:
		var->xres = fbdev->panel->x_res;
		var->yres = fbdev->panel->y_res;
		break;
	case 90:
	case 270:
		var->xres = fbdev->panel->y_res;
		var->yres = fbdev->panel->x_res;
		break;
	default:
		return -EINVAL;
	}
	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;
	max_frame_size = fbdev->vram_size;
	line_size = var->xres_virtual * bpp / 8;
	if (line_size * var->yres_virtual > max_frame_size) {
		/* Try to keep yres_virtual first */
		line_size = max_frame_size / var->yres_virtual;
		var->xres_virtual = line_size * 8 / bpp;
		if (var->xres_virtual < var->xres) {
			/* Still doesn't fit. Shrink yres_virtual too */
			var->xres_virtual = var->xres;
			line_size = var->xres * bpp / 8;
			var->yres_virtual = max_frame_size / line_size;
		}
	}
	if (var->xres + var->xoffset > var->xres_virtual)
		var->xoffset = var->xres_virtual - var->xres;
	if (var->yres + var->yoffset > var->yres_virtual)
		var->yoffset = var->yres_virtual - var->yres;
	line_size = var->xres * bpp / 8;

	var->red.offset	 = 11; var->red.length	 = 5; var->red.msb_right   = 0;
	var->green.offset= 5;  var->green.length = 6; var->green.msb_right = 0;
	var->blue.offset = 0;  var->blue.length  = 5; var->blue.msb_right  = 0;

	var->height		= -1;
	var->width		= -1;
	var->grayscale		= 0;
	var->nonstd		= 0;

	/* pixclock in ps, the rest in pixclock */
	var->pixclock		= 10000000 / (panel->pixel_clock / 100);
	var->left_margin	= panel->hfp;
	var->right_margin	= panel->hbp;
	var->upper_margin	= panel->vfp;
	var->lower_margin	= panel->vbp;
	var->hsync_len		= panel->hsw;
	var->vsync_len		= panel->vsw;

	/* TODO: get these from panel->config */
	var->vmode		= FB_VMODE_NONINTERLACED;
	var->sync		= 0;

	return 0;
}

static struct fb_var_screeninfo new_var;

/* Set rotation (0, 90, 180, 270 degree), and switch to the new mode. */
static void omapfb_rotate(struct fb_info *fbi, int rotate)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;

	DBGENTER(1);

	if (cpu_is_omap1510() && rotate != fbdev->fb_info->var.rotate) {
		memcpy(&new_var, &fbi->var, sizeof(new_var));
		new_var.rotate = rotate;
		if (set_fb_var(fbdev, &new_var) == 0 &&
		    memcmp(&new_var, &fbi->var, sizeof(new_var))) {
			memcpy(&fbi->var, &new_var, sizeof(new_var));
			ctrl_change_mode(fbdev);
		}
	}

	DBGLEAVE(1);
}

/* Set new x,y offsets in the virtual display for the visible area and switch
 * to the new mode.
 */
static int omapfb_pan_display(struct fb_var_screeninfo *var,
			       struct fb_info *fbi)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;
	int r = 0;

	DBGENTER(1);

	if (var->xoffset != fbi->var.xoffset ||
	    var->yoffset != fbi->var.yoffset) {
		memcpy(&new_var, &fbi->var, sizeof(new_var));
		new_var.xoffset = var->xoffset;
		new_var.yoffset = var->yoffset;
		if (set_fb_var(fbdev, &new_var))
			r = -EINVAL;
		else {
			memcpy(&fbi->var, &new_var, sizeof(new_var));
			ctrl_change_mode(fbdev);
		}
	}

	DBGLEAVE(1);
	return r;
}

/* Set mirror to vertical axis and switch to the new mode. */
static int omapfb_mirror(struct omapfb_device *fbdev, int mirror)
{
	int r = 0;

	DBGENTER(1);

	mirror = mirror ? 1 : 0;
	if (cpu_is_omap1510())
		r = -EINVAL;
	else if (mirror != fbdev->mirror) {
		fbdev->mirror = mirror;
		r = ctrl_change_mode(fbdev);
	}

	DBGLEAVE(1);
	return r;
}

/* Check values in var, try to adjust them in case of out of bound values if
 * possible, or return error.
 */
static int omapfb_check_var(struct fb_var_screeninfo *var, struct fb_info *fbi)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;
	int r;

	DBGENTER(1);

	r = set_fb_var(fbdev, var);

	DBGLEAVE(1);
	return r;
}

/* Switch to a new mode. The parameters for it has been check already by
 * omapfb_check_var.
 */
static int omapfb_set_par(struct fb_info *fbi)
{
	int r;
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;

	DBGENTER(1);

	set_fb_fix(fbdev);
	r = ctrl_change_mode(fbdev);

	DBGLEAVE(1);
	return r;
}

static int omapfb_update_win(struct omapfb_device *fbdev,
				struct omapfb_update_window *win)
{
	struct fb_var_screeninfo *var = &fbdev->fb_info->var;
	int ret;

	if (win->x >= var->xres || win->y >= var->yres)
		return -EINVAL;

	if (!fbdev->ctrl->update_window ||
	    fbdev->ctrl->get_update_mode() != OMAPFB_MANUAL_UPDATE)
		return -ENODEV;

	if (win->x + win->width >= var->xres)
		win->width = var->xres - win->x;
	if (win->y + win->height >= var->yres)
		win->height = var->yres - win->y;
	if (!win->width || !win->height)
		return 0;

	omapfb_rqueue_lock(fbdev);
	ret = fbdev->ctrl->update_window(win, NULL, 0);
	omapfb_rqueue_unlock(fbdev);

	return ret;
}

static void omapfb_update_full_screen(struct omapfb_device *fbdev)
{
	struct omapfb_update_window win;

	win.x = 0;
	win.y = 0;
	win.width = fbdev->panel->x_res;
	win.height = fbdev->panel->y_res;
	win.format = 0;

	omapfb_rqueue_lock(fbdev);
	fbdev->ctrl->update_window(&win, NULL, 0);
	omapfb_rqueue_unlock(fbdev);
}

static int omapfb_setup_plane(struct omapfb_device *fbdev,
			      struct omapfb_setup_plane *sp)
{
	int r;

	omapfb_rqueue_lock(fbdev);
	r = fbdev->ctrl->setup_plane(sp->plane, sp->channel_out, sp->offset,
				 sp->width, sp->pos_x, sp->pos_y, sp->width,
				 sp->height, sp->color_mode);
	omapfb_rqueue_unlock(fbdev);

	return r;
}

static int omapfb_enable_plane(struct omapfb_device *fbdev, int plane,
				int enable)
{
	int r;

	omapfb_rqueue_lock(fbdev);
	r = fbdev->ctrl->enable_plane(plane, enable);
	omapfb_rqueue_unlock(fbdev);

	return r;
}

static int omapfb_set_color_key(struct omapfb_device *fbdev,
				struct omapfb_color_key *ck)
{
	int r;

	if (!fbdev->ctrl->set_color_key)
		return -ENODEV;

	omapfb_rqueue_lock(fbdev);
	r = fbdev->ctrl->set_color_key(ck);
	omapfb_rqueue_unlock(fbdev);

	return r;
}

static int omapfb_set_update_mode(struct omapfb_device *fbdev,
				   enum omapfb_update_mode mode)
{
	int r;

	omapfb_rqueue_lock(fbdev);
	r = fbdev->ctrl->set_update_mode(mode);
	omapfb_rqueue_unlock(fbdev);

	return r;
}

static enum omapfb_update_mode omapfb_get_update_mode(struct omapfb_device *fbdev)
{
	int r;

	omapfb_rqueue_lock(fbdev);
	r = fbdev->ctrl->get_update_mode();
	omapfb_rqueue_unlock(fbdev);

	return r;
}

static unsigned long omapfb_get_caps(struct fb_info *fbi)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;
	unsigned long caps;

	caps = 0;
	caps |= fbdev->panel->get_caps();
	caps |= fbdev->ctrl->get_caps();
	return caps;
}

/* For lcd testing */
void omapfb_write_first_pixel(struct omapfb_device *fbdev, u16 pixval)
{
	omapfb_rqueue_lock(fbdev);
	*(u16 *)fbdev->vram_virt_base = pixval;
	if (fbdev->ctrl->get_update_mode() == OMAPFB_MANUAL_UPDATE) {
		struct omapfb_update_window win;

		win.x = 0;
		win.y = 0;
		win.width = 1;
		win.height = 1;
		win.format = 0;
		fbdev->ctrl->update_window(&win, NULL, 0);
	}
	omapfb_rqueue_unlock(fbdev);
}
EXPORT_SYMBOL(omapfb_write_first_pixel);

/* Ioctl interface. Part of the kernel mode frame buffer API is duplicated
 * here to be accessible by user mode code. In addition transparent copy
 * graphics transformations, frame flipping support is provided through this
 * interface.
 */
static int omapfb_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg,
			struct fb_info *fbi)
{
	struct omapfb_device	*fbdev = (struct omapfb_device *)fbi->par;
	struct fb_ops		*ops = fbi->fbops;
	union {
		struct omapfb_update_window	update_window;
		struct omapfb_setup_plane	setup_plane;
		struct omapfb_enable_plane	enable_plane;
		struct omapfb_color_key		color_key;
		enum omapfb_update_mode		update_mode;
		unsigned long		caps;
		unsigned int		mirror;
	} p;
	int r = 0;

	DBGENTER(2);

	BUG_ON(!ops);
	DBGPRINT(2, "cmd=%010x\n", cmd);
	switch (cmd)
	{
	case OMAPFB_MIRROR:
		if (get_user(p.mirror, (int __user *)arg))
			r = -EFAULT;
		else
			omapfb_mirror(fbdev, p.mirror);
		break;
	case OMAPFB_SYNC_GFX:
		omapfb_sync(fbi);
		break;
	case OMAPFB_VSYNC:
		break;
	case OMAPFB_SET_UPDATE_MODE:
		if (get_user(p.update_mode, (int __user *)arg))
			r = -EFAULT;
		else
			r = omapfb_set_update_mode(fbdev, p.update_mode);
		break;
	case OMAPFB_GET_UPDATE_MODE:
		p.update_mode = omapfb_get_update_mode(fbdev);
		if (put_user(p.update_mode,
					(enum omapfb_update_mode __user *)arg))
			r = -EFAULT;
		break;
	case OMAPFB_UPDATE_WINDOW:
		if (copy_from_user(&p.update_window, (void __user *)arg,
				   sizeof(p.update_window)))
			r = -EFAULT;
		else
			r = omapfb_update_win(fbdev, &p.update_window);
		break;
	case OMAPFB_SETUP_PLANE:
		if (copy_from_user(&p.setup_plane, (void __user *)arg,
				   sizeof(p.setup_plane)))
			r = -EFAULT;
		else
			r = omapfb_setup_plane(fbdev, &p.setup_plane);
		break;
	case OMAPFB_ENABLE_PLANE:
		if (copy_from_user(&p.enable_plane, (void __user *)arg,
				   sizeof(p.enable_plane)))
			r = -EFAULT;
		else
			r = omapfb_enable_plane(fbdev,
				p.enable_plane.plane, p.enable_plane.enable);
		break;
	case OMAPFB_SET_COLOR_KEY:
		if (copy_from_user(&p.color_key, (void __user *)arg,
				   sizeof(p.color_key)))
			r = -EFAULT;
		else
			r = omapfb_set_color_key(fbdev, &p.color_key);
		break;
	case OMAPFB_GET_CAPS:
		p.caps = omapfb_get_caps(fbi);
		if (put_user(p.caps, (unsigned long __user *)arg))
			r = -EFAULT;
		break;
	case OMAPFB_LCD_TEST:
		{
			int test_num;

			if (get_user(test_num, (int __user *)arg)) {
				r = -EFAULT;
				break;
			}
			if (!fbdev->panel->run_test) {
				r = -EINVAL;
				break;
			}
			r = fbdev->panel->run_test(test_num);
			break;
		}
	case OMAPFB_CTRL_TEST:
		{
			int test_num;

			if (get_user(test_num, (int __user *)arg)) {
				r = -EFAULT;
				break;
			}
			if (!fbdev->ctrl->run_test) {
				r = -EINVAL;
				break;
			}
			r = fbdev->ctrl->run_test(test_num);
			break;
		}
	default:
		r = -EINVAL;
	}

	DBGLEAVE(2);
	return r;
}

/* Callback table for the frame buffer framework. Some of these pointers
 * will be changed according to the current setting of fb_info->accel_flags.
 */
static struct fb_ops omapfb_ops = {
	.owner		= THIS_MODULE,
	.fb_open        = omapfb_open,
	.fb_release     = omapfb_release,
	.fb_setcolreg	= omapfb_setcolreg,
	.fb_setcmap	= omapfb_setcmap,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_blank       = omapfb_blank,
	.fb_ioctl	= omapfb_ioctl,
	.fb_check_var	= omapfb_check_var,
	.fb_set_par	= omapfb_set_par,
	.fb_rotate	= omapfb_rotate,
	.fb_pan_display = omapfb_pan_display,
};

/*
 * ---------------------------------------------------------------------------
 * Sysfs interface
 * ---------------------------------------------------------------------------
 */
/* omapfbX sysfs entries */
static ssize_t omapfb_show_caps_num(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)dev->driver_data;

	return snprintf(buf, PAGE_SIZE, "%#010lx\n",
		omapfb_get_caps(fbdev->fb_info));
}

static ssize_t omapfb_show_caps_text(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)dev->driver_data;
	int pos = 0;
	int i;
	unsigned long caps;

	caps = omapfb_get_caps(fbdev->fb_info);
	for (i = 0; i < ARRAY_SIZE(omapfb_caps_table) && pos < PAGE_SIZE; i++) {
		if (omapfb_caps_table[i].flag & caps) {
			pos += snprintf(&buf[pos], PAGE_SIZE - pos, "%s\n",
					omapfb_caps_table[i].name);
		}
	}
	return min((int)PAGE_SIZE, pos);
}

static DEVICE_ATTR(caps_num, 0444, omapfb_show_caps_num, NULL);
static DEVICE_ATTR(caps_text, 0444, omapfb_show_caps_text, NULL);

/* panel sysfs entries */
static ssize_t omapfb_show_panel_name(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)dev->driver_data;

	return snprintf(buf, PAGE_SIZE, "%s\n", fbdev->panel->name);
}

static ssize_t omapfb_show_bklight_level(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)dev->driver_data;
	int r;

	if (fbdev->panel->get_bklight_level) {
		r = snprintf(buf, PAGE_SIZE, "%d\n",
			     fbdev->panel->get_bklight_level());
	} else
		r = -ENODEV;
	return r;
}

static ssize_t omapfb_store_bklight_level(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)dev->driver_data;
	int r;

	if (fbdev->panel->set_bklight_level) {
		unsigned int level;

		if (sscanf(buf, "%10d", &level) == 1) {
			r = fbdev->panel->set_bklight_level(level);
		} else
			r = -EINVAL;
	} else
		r = -ENODEV;
	return r ? r : size;
}

static ssize_t omapfb_show_bklight_max(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)dev->driver_data;
	int r;

	if (fbdev->panel->get_bklight_level) {
		r = snprintf(buf, PAGE_SIZE, "%d\n",
			     fbdev->panel->get_bklight_max());
	} else
		r = -ENODEV;
	return r;
}

static struct device_attribute dev_attr_panel_name =
	__ATTR(name, 0444, omapfb_show_panel_name, NULL);
static DEVICE_ATTR(backlight_level, 0664,
		   omapfb_show_bklight_level, omapfb_store_bklight_level);
static DEVICE_ATTR(backlight_max, 0444, omapfb_show_bklight_max, NULL);

static struct attribute *panel_attrs[] = {
	&dev_attr_panel_name.attr,
	&dev_attr_backlight_level.attr,
	&dev_attr_backlight_max.attr,
	NULL,
};

static struct attribute_group panel_attr_grp = {
	.name  = "panel",
	.attrs = panel_attrs,
};

/* ctrl sysfs entries */
static ssize_t omapfb_show_ctrl_name(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)dev->driver_data;

	return snprintf(buf, PAGE_SIZE, "%s\n", fbdev->ctrl->name);
}

static struct device_attribute dev_attr_ctrl_name =
	__ATTR(name, 0444, omapfb_show_ctrl_name, NULL);

static struct attribute *ctrl_attrs[] = {
	&dev_attr_ctrl_name.attr,
	NULL,
};

static struct attribute_group ctrl_attr_grp = {
	.name  = "ctrl",
	.attrs = ctrl_attrs,
};

static int omapfb_register_sysfs(struct omapfb_device *fbdev)
{
	int r;

	if ((r = device_create_file(fbdev->dev, &dev_attr_caps_num)))
		goto fail0;

	if ((r = device_create_file(fbdev->dev, &dev_attr_caps_text)))
		goto fail1;

	if ((r = sysfs_create_group(&fbdev->dev->kobj, &panel_attr_grp)))
		goto fail2;

	if ((r = sysfs_create_group(&fbdev->dev->kobj, &ctrl_attr_grp)))
		goto fail3;

	return 0;
fail3:
	sysfs_remove_group(&fbdev->dev->kobj, &panel_attr_grp);
fail2:
	device_remove_file(fbdev->dev, &dev_attr_caps_text);
fail1:
	device_remove_file(fbdev->dev, &dev_attr_caps_num);
fail0:
	pr_err("unable to register sysfs interface\n");
	return r;
}

static void omapfb_unregister_sysfs(struct omapfb_device *fbdev)
{
	sysfs_remove_group(&fbdev->dev->kobj, &ctrl_attr_grp);
	sysfs_remove_group(&fbdev->dev->kobj, &panel_attr_grp);
	device_remove_file(fbdev->dev, &dev_attr_caps_num);
	device_remove_file(fbdev->dev, &dev_attr_caps_text);
}

/*
 * ---------------------------------------------------------------------------
 * LDM callbacks
 * ---------------------------------------------------------------------------
 */
/* Initialize system fb_info object and set the default video mode.
 * The frame buffer memory already allocated by lcddma_init
 */
static int fbinfo_init(struct omapfb_device *fbdev)
{
	struct fb_info			*info = fbdev->fb_info;
	struct fb_var_screeninfo	*var = &info->var;
	int				r = 0;

	DBGENTER(1);

	BUG_ON(!fbdev->vram_virt_base);

	info->fbops = &omapfb_ops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->screen_base = (char __iomem *)fbdev->vram_virt_base;

	info->pseudo_palette = fbdev->pseudo_palette;

	var->accel_flags  = def_accel ? FB_ACCELF_TEXT : 0;
	var->xres_virtual = def_vxres;
	var->yres_virtual = def_vyres;
	var->rotate	  = def_rotate;

	fbdev->mirror = def_mirror;

	set_fb_var(fbdev, var);
	set_fb_fix(fbdev);

	r = fb_alloc_cmap(&info->cmap, 16, 0);
	if (r != 0)
		pr_err("unable to allocate color map memory\n");

	DBGLEAVE(1);
	return r;
}

/* Release the fb_info object */
static void fbinfo_cleanup(struct omapfb_device *fbdev)
{
	DBGENTER(1);

	fb_dealloc_cmap(&fbdev->fb_info->cmap);

	DBGLEAVE(1);
}

/* Free driver resources. Can be called to rollback an aborted initialization
 * sequence.
 */
static void omapfb_free_resources(struct omapfb_device *fbdev, int state)
{
	switch (state) {
	case OMAPFB_ACTIVE:
		unregister_framebuffer(fbdev->fb_info);
	case 7:
		omapfb_unregister_sysfs(fbdev);
	case 6:
		fbdev->panel->disable();
	case 5:
		omapfb_set_update_mode(fbdev, OMAPFB_UPDATE_DISABLED);
	case 4:
		fbinfo_cleanup(fbdev);
	case 3:
		ctrl_cleanup(fbdev);
	case 2:
		fbdev->panel->cleanup();
	case 1:
		dev_set_drvdata(fbdev->dev, NULL);
		framebuffer_release(fbdev->fb_info);
	case 0:
		/* nothing to free */
		break;
	default:
		BUG();
	}
}

static int omapfb_find_panel(struct omapfb_device *fbdev)
{
	const struct omap_lcd_config *conf;
	char name[17];
	int i;

	conf = (struct omap_lcd_config *)fbdev->dev->platform_data;
	fbdev->panel = NULL;
	if (conf == NULL)
		return -1;

	strncpy(name, conf->panel_name, sizeof(name) - 1);
	name[sizeof(name) - 1] = 0;
	for (i = 0; i < ARRAY_SIZE(panels); i++) {
		if (strcmp(panels[i]->name, name) == 0) {
			fbdev->panel = panels[i];
			break;
		}
	}

	if (fbdev->panel == NULL)
		return -1;

	return 0;
}

static int omapfb_find_ctrl(struct omapfb_device *fbdev)
{
	struct omap_lcd_config *conf;
	char name[17];
	int i;

	conf = (struct omap_lcd_config *)fbdev->dev->platform_data;

	fbdev->ctrl = NULL;
	if (conf == NULL)
		return -1;

	strncpy(name, conf->ctrl_name, sizeof(name) - 1);
	name[sizeof(name) - 1] = '\0';

	if (strcmp(name, "internal") == 0) {
		fbdev->ctrl = fbdev->int_ctrl;
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(ctrls); i++) {
		if (strcmp(ctrls[i]->name, name) == 0) {
			fbdev->ctrl = ctrls[i];
			break;
		}
	}

	if (fbdev->ctrl == NULL)
		return -1;

	return 0;
}

static void check_required_callbacks(struct omapfb_device *fbdev)
{
#define _C(x) (fbdev->ctrl->x != NULL)
#define _P(x) (fbdev->panel->x != NULL)
	BUG_ON(fbdev->ctrl == NULL || fbdev->panel == NULL);
	BUG_ON(!(_C(init) && _C(cleanup) && _C(get_caps) &&
		 _C(set_update_mode) && _C(setup_plane) && _C(enable_plane) &&
		 _P(init) && _P(cleanup) && _P(enable) && _P(disable) &&
		 _P(get_caps)));
#undef _P
#undef _C
}

/* Called by LDM binding to probe and attach a new device.
 * Initialization sequence:
 *   1. allocate system fb_info structure
 *      select panel type according to machine type
 *   2. init LCD panel
 *   3. init LCD controller and LCD DMA
 *   4. init system fb_info structure
 *   5. init gfx DMA
 *   6. enable LCD panel
 *      start LCD frame transfer
 *   7. register system fb_info structure
 */
static int omapfb_probe(struct platform_device *pdev)
{
	struct omapfb_device	*fbdev = NULL;
	struct fb_info		*fbi;
	int			init_state;
	unsigned long		phz, hhz, vhz;
	struct lcd_panel	*panel;
	int			r = 0;

	DBGENTER(1);

	init_state = 0;

	if (pdev->num_resources != 0) {
		pr_err("probed for an unknown device\n");
		r = -ENODEV;
		goto cleanup;
	}

	fbi = framebuffer_alloc(sizeof(struct omapfb_device), &pdev->dev);
	if (fbi == NULL) {
		pr_err("unable to allocate memory for device info\n");
		r = -ENOMEM;
		goto cleanup;
	}
	init_state++;

	fbdev = (struct omapfb_device *)fbi->par;
	fbdev->fb_info = fbi;
	fbdev->dev = &pdev->dev;
	platform_set_drvdata(pdev, fbdev);

	init_MUTEX(&fbdev->rqueue_sema);

#ifdef CONFIG_ARCH_OMAP1
	fbdev->int_ctrl = &omap1_int_ctrl;
#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
	fbdev->ext_if = &sossi_extif;
#endif
#else	/* OMAP2 */
	fbdev->int_ctrl = &omap2_int_ctrl;
#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
	fbdev->ext_if = &rfbi_extif;
#endif
#endif
	if (omapfb_find_ctrl(fbdev) < 0) {
		pr_err("LCD controller not found, board not supported\n");
		r = -ENODEV;
		goto cleanup;
	}

	if (omapfb_find_panel(fbdev) < 0) {
		pr_err("LCD panel not found, board not supported\n");
		r = -ENODEV;
		goto cleanup;
	}

	check_required_callbacks(fbdev);


	pr_info(MODULE_NAME ": configured for panel %s\n", fbdev->panel->name);

	r = fbdev->panel->init(fbdev);
	if (r)
		goto cleanup;
	init_state++;

	r = ctrl_init(fbdev);
	if (r)
		goto cleanup;
	init_state++;

	r = fbinfo_init(fbdev);
	if (r)
		goto cleanup;
	init_state++;

#ifdef CONFIG_FB_OMAP_DMA_TUNE
	/* Set DMA priority for EMIFF access to highest */
	omap_set_dma_priority(OMAP_DMA_PORT_EMIFF, 15);
#endif

	r = ctrl_change_mode(fbdev);
	if (r) {
		pr_err("mode setting failed\n");
		goto cleanup;
	}

	omapfb_enable_plane(fbdev, 0, 1);

	omapfb_set_update_mode(fbdev, manual_update ?
				   OMAPFB_MANUAL_UPDATE : OMAPFB_AUTO_UPDATE);
	init_state++;

	r = fbdev->panel->enable();
	if (r)
		goto cleanup;
	init_state++;

	r = omapfb_register_sysfs(fbdev);
	if (r)
		goto cleanup;
	init_state++;

	r = register_framebuffer(fbdev->fb_info);
	if (r != 0) {
		pr_err("register_framebuffer failed\n");
		goto cleanup;
	}

	fbdev->state = OMAPFB_ACTIVE;

	panel = fbdev->panel;
	phz = panel->pixel_clock * 1000;
	hhz = phz * 10 / (panel->hfp + panel->x_res + panel->hbp + panel->hsw);
	vhz = hhz / (panel->vfp + panel->y_res + panel->vbp + panel->vsw);

	pr_info(MODULE_NAME ": initialized vram=%lu "
			"pixclock %lu kHz hfreq %lu.%lu kHz vfreq %lu.%lu Hz\n",
			fbdev->vram_size,
			phz / 1000, hhz / 10000, hhz % 10, vhz / 10, vhz % 10);

	DBGLEAVE(1);
	return 0;

cleanup:
	omapfb_free_resources(fbdev, init_state);

	DBGLEAVE(1);
	return r;
}

/* Called when the device is being detached from the driver */
static int omapfb_remove(struct platform_device *pdev)
{
	struct omapfb_device *fbdev = platform_get_drvdata(pdev);
	enum omapfb_state saved_state = fbdev->state;

	DBGENTER(1);
	/* FIXME: wait till completion of pending events */

	fbdev->state = OMAPFB_DISABLED;
	omapfb_free_resources(fbdev, saved_state);

	DBGLEAVE(1);
	return 0;
}

/* PM suspend */
static int omapfb_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct omapfb_device *fbdev = platform_get_drvdata(pdev);

	DBGENTER(1);

	omapfb_blank(VESA_POWERDOWN, fbdev->fb_info);

	DBGLEAVE(1);

	return 0;
}

/* PM resume */
static int omapfb_resume(struct platform_device *pdev)
{
	struct omapfb_device *fbdev = platform_get_drvdata(pdev);

	DBGENTER(1);

	omapfb_blank(VESA_NO_BLANKING, fbdev->fb_info);

	DBGLEAVE(1);
	return 0;
}

static struct platform_driver omapfb_driver = {
	.probe		= omapfb_probe,
	.remove		= omapfb_remove,
	.suspend	= omapfb_suspend,
	.resume		= omapfb_resume,
	.driver		= {
		.name	= OMAPFB_DRIVER,
		.owner	= THIS_MODULE,
	},
};

#ifndef MODULE

/* Process kernel command line parameters */
static int __init omapfb_setup(char *options)
{
	char *this_opt = NULL;
	int r = 0;

	DBGENTER(1);

	if (!options || !*options)
		goto exit;

	while (!r && (this_opt = strsep(&options, ",")) != NULL) {
		if (!strncmp(this_opt, "accel", 5))
			def_accel = 1;
		else if (!strncmp(this_opt, "vram:", 5)) {
			char *suffix;
			def_vram = (simple_strtoul(this_opt + 5, &suffix, 0));
			switch (suffix[0]) {
			case '\0':
				break;
			case 'm':
			case 'M':
				def_vram *= 1024;
				/* Fall through */
			case 'k':
			case 'K':
				def_vram *= 1024;
				break;
			default:
				pr_err("invalid vram suffix\n");
				r = -1;
			}
		}
		else if (!strncmp(this_opt, "vxres:", 6))
			def_vxres = simple_strtoul(this_opt + 6, NULL, 0);
		else if (!strncmp(this_opt, "vyres:", 6))
			def_vyres = simple_strtoul(this_opt + 6, NULL, 0);
		else if (!strncmp(this_opt, "rotate:", 7))
			def_rotate = (simple_strtoul(this_opt + 7, NULL, 0));
		else if (!strncmp(this_opt, "mirror:", 7))
			def_mirror = (simple_strtoul(this_opt + 7, NULL, 0));
		else if (!strncmp(this_opt, "manual_update", 13))
			manual_update = 1;
		else {
			pr_err("invalid option\n");
			r = -1;
		}
	}
exit:
	DBGLEAVE(1);
	return r;
}

#endif

/* Register both the driver and the device */
static int __init omapfb_init(void)
{
	int r = 0;

	DBGENTER(1);

#ifndef MODULE
	{
		char *option;

		if (fb_get_options("omapfb", &option)) {
			r = -ENODEV;
			goto exit;
		}
		omapfb_setup(option);
	}
#endif
	/* Register the driver with LDM */
	if (platform_driver_register(&omapfb_driver)) {
		pr_err("failed to register omapfb driver\n");
		r = -ENODEV;
		goto exit;
	}

exit:
	DBGLEAVE(1);
	return r;
}

static void __exit omapfb_cleanup(void)
{
	DBGENTER(1);

	platform_driver_unregister(&omapfb_driver);

	DBGLEAVE(1);
}

module_param_named(accel, def_accel, uint, 0664);
module_param_named(vram, def_vram, ulong, 0664);
module_param_named(vxres, def_vxres, long, 0664);
module_param_named(vyres, def_vyres, long, 0664);
module_param_named(rotate, def_rotate, uint, 0664);
module_param_named(mirror, def_mirror, uint, 0664);
module_param_named(manual_update, manual_update, bool, 0664);

module_init(omapfb_init);
module_exit(omapfb_cleanup);

MODULE_DESCRIPTION("TI OMAP framebuffer driver");
MODULE_AUTHOR("Imre Deak <imre.deak@nokia.com>");
MODULE_LICENSE("GPL");
