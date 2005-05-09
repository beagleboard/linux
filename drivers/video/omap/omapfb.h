/*
 * File: drivers/video/omap_new/omapfb.c
 *
 * Framebuffer driver for TI OMAP boards
 *
 * Copyright (C) 2004 Nokia Corporation
 * Author: Imre Deak <imre.deak@nokia.com>
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

#ifndef __OMAPFB_H
#define __OMAPFB_H

/* IOCTL commands. */

#define OMAP_IOW(num, dtype)	_IOW('O', num, dtype)
#define OMAP_IOR(num, dtype)	_IOR('O', num, dtype)
#define OMAP_IOWR(num, dtype)	_IOWR('O', num, dtype)
#define OMAP_IO(num)		_IO('O', num)

#define OMAPFB_FILLRECT		OMAP_IOW(0, struct fb_fillrect)
#define OMAPFB_COPYAREA		OMAP_IOW(1, struct fb_copyarea)
#define OMAPFB_IMAGEBLIT	OMAP_IOW(2, struct fb_image)

#define OMAPFB_TRANSPARENT_BLIT	OMAP_IOW(30, struct fb_image)
#define OMAPFB_MIRROR		OMAP_IOW(31, int)
#define OMAPFB_SCALE		OMAP_IOW(32, struct fb_scale)
#define OMAPFB_SELECT_VIS_FRAME	OMAP_IOW(33, int)
#define OMAPFB_SELECT_SRC_FRAME OMAP_IOW(34, int)
#define OMAPFB_SELECT_DST_FRAME	OMAP_IOW(35, int)
#define OMAPFB_GET_FRAME_OFFSET	OMAP_IOWR(36, struct fb_frame_offset)
#define OMAPFB_SYNC_GFX		OMAP_IO(37)
#define OMAPFB_VSYNC		OMAP_IO(38)
#define OMAPFB_LATE_ACTIVATE	OMAP_IO(39)
#define OMAPFB_SET_UPDATE_MODE	OMAP_IOW(40, enum fb_update_mode)
#define OMAPFB_UPDATE_WINDOW	OMAP_IOW(41, struct fb_update_window)
#define OMAPFB_GET_CAPS		OMAP_IOR(42, unsigned long)
#define OMAPFB_GET_UPDATE_MODE	OMAP_IOW(43, enum fb_update_mode)
#define OMAPFB_GET_GFX_STATUS	OMAP_IOR(44, unsigned long)

#define FBCAPS_GENERIC_MASK	0x00000fff
#define FBCAPS_LCDC_MASK	0x00fff000
#define FBCAPS_PANEL_MASK	0xff000000

#define FBCAPS_MANUAL_UPDATE	0x00001000
#define FBCAPS_SET_BACKLIGHT	0x01000000

enum omapfb_gfx_status {
	OMAPFB_GFX_STATUS_OK	= 0,
	OMAPFB_GFX_STATUS_CHANGED
};

#define OMAPFB_UPDATE_FAILED	0x01
#define OMAPFB_FILLRECT_FAILED	0x02
#define OMAPFB_COPYAREA_FAILED	0x04
#define OMAPFB_IMGBLIT_FAILED	0x08

struct fb_copyarea_ext {
	__u32 dx;
	__u32 dy;
	__u32 width;
	__u32 height;
	__u32 sx;
	__u32 sy;
	__u32 trans_color;
	__u32 rev_dir;
};

struct fb_scale {
	unsigned int xscale, yscale;
};

struct fb_frame_offset {
	unsigned int idx;
	unsigned long offset;
};

struct fb_update_window {
	unsigned int x, y;
	unsigned int width, height;
};

enum fb_update_mode {
	FB_UPDATE_DISABLED = 0,
	FB_AUTO_UPDATE,
	FB_MANUAL_UPDATE
};

#ifdef __KERNEL__

#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/fb.h>

#define OMAPFB_DEVICE "omapfb"
#define OMAPFB_DRIVER "omapfb"

#define PRNERR(fmt, args...)  printk(KERN_ERR OMAPFB_DRIVER ": " fmt, ## args)

#define GFX_FIFO_SIZE 2

#define LCD_PANEL_TFT 0x01

#define OMAP_LCDC_INV_VSYNC             0x01
#define OMAP_LCDC_INV_HSYNC             0x02
#define OMAP_LCDC_INV_PIX_CLOCK         0x04
#define OMAP_LCDC_INV_OUTPUT_EN         0x08
#define OMAP_LCDC_HSVS_RISING_EDGE      0x10
#define OMAP_LCDC_HSVS_OPPOSITE         0x20

struct lcdc_video_mode {
	u16	x_res, y_res;
	u32	pixel_clock;	/* In kHz */
	int	bpp;
	u8	hsw;		/* Horizontal synchronization pulse width */
	u8	hfp;		/* Horizontal front porch */
	u8	hbp;		/* Horizontal back porch */
	u8	vsw;		/* Vertical synchronization pulse width */
	u8	vfp;		/* Vertical front porch */
	u8	vbp;		/* Vertical back porch */
	u8	acb;		/* ac-bias pin frequency */
	u8	pcd;		/* Pixel clock divider (this will change) */
	u8  flags;
};

struct lcd_panel {
	const char *name;
	int  config;
	int  signals;
	struct lcdc_video_mode *video_mode;

	int  (*init)	(struct lcd_panel *panel);
	void (*cleanup)	(struct lcd_panel *panel);
	int  (*enable)	(struct lcd_panel *panel);
	void (*disable)	(struct lcd_panel *panel);
	unsigned long (*get_caps)(struct lcd_panel *panel);
	int	      (*set_bklight_level)(struct lcd_panel *panel,
					   unsigned int level);
	unsigned int  (*get_bklight_level)(struct lcd_panel *panel);
	unsigned int  (*get_bklight_max)  (struct lcd_panel *panel);
};

struct omapfb_device;

struct lcd_ctrl {
	const char	*name;
	void		*data;
	int		(*init)		  (struct omapfb_device *fbdev);
	void		(*cleanup)	  (struct omapfb_device *fbdev);
	void		(*get_mem_layout) (struct omapfb_device *fbdev,
					   unsigned long *size,
					   unsigned long *fb_org);
	unsigned long	(*get_caps)	  (struct omapfb_device *fbdev);
	int		(*set_update_mode)(struct omapfb_device *fbdev,
				 	   enum fb_update_mode mode);
	enum fb_update_mode (*get_update_mode)(struct omapfb_device *fbdev);
	int		(*update_window)  (struct omapfb_device *fbdev,
					   struct fb_update_window *win);
	void 		(*suspend)	  (struct omapfb_device *fbdev);
	void		(*resume)	  (struct omapfb_device *fbdev);
	void		(*change_mode)	  (struct omapfb_device *fbdev);
};

enum omapfb_state {
	OMAPFB_DISABLED	= 0,
	OMAPFB_SUSPENDED= 99,
	OMAPFB_ACTIVE	= 100
};

struct gfx_lchannel {
	int			lch_num;
	struct gfx_lchannel	*next, *prev;
};

struct gfx_dma {
	spinlock_t		spinlock;

	struct completion	sync_complete;		/* Signalled when the
							   fifo gets empty */
	volatile int		done;			/* Indicates the
							   end of a DMA chain
							   transfer */
	struct gfx_lchannel	fifo[GFX_FIFO_SIZE];
	struct gfx_lchannel	*f_head, *f_tail;	/* Process and insert
							   points on the
							   fifo */
	struct gfx_lchannel	*f_chain_end;		/* Points to the new
							   chain end */
	struct semaphore	f_free;			/* # of free lch-s */
	int			f_run;			/* # of active lch-s */
	int			f_wait;			/* # of lch-s
							   waiting */
	struct tasklet_struct	dequeue_tasklet;	/* Processes new DMA
							   chain  transfers on
							   the fifo */
};

#define OMAPFB_RQUEUE_SIZE 20

struct omapfb_fillrect_params
{
	struct fb_info		*fbi;
	struct fb_fillrect	rect; 
};

struct omapfb_copyarea_params
{
	struct fb_info		*fbi;
	struct fb_copyarea_ext	area;
};

struct omapfb_update_window_params
{
	struct fb_info		*fbi;
	struct fb_update_window	win;
};

struct omapfb_imageblit_params
{
	struct fb_info	*fbi;
	struct fb_image	image;
	int		flags;
};

union req_params
{
	/* All possible requests are to be listed here */
	struct omapfb_fillrect_params		fillrect;
	struct omapfb_copyarea_params		copyarea;
	struct omapfb_update_window_params	update_window;
	struct omapfb_imageblit_params		imageblit;
};

struct omapfb_request
{
	struct list_head	entry;
	int			(*function)(void *par);
	union req_params	par;
};

struct omapfb_rqueue
{
	spinlock_t		lock;
	struct list_head	free_list;
	struct list_head	pending_list;
	struct completion	rqueue_empty;
	struct semaphore	free_sema;
	struct omapfb_request	req_pool[OMAPFB_RQUEUE_SIZE];
	struct work_struct	work;
	unsigned long		status;
};

struct omapfb_device {
	int			state;
	int                     ext_lcdc;               /* Using external
                                                           LCD controller */
	void			*lcddma_base;		/* MPU virtual
							   address */
	dma_addr_t		lcddma_handle;		/* Bus physical
							   address */
	unsigned long		lcddma_mem_size;
	unsigned long		palette_org;		/* Palette offset into
							   lcddma_base/handle */
	unsigned long		frame0_org, frame1_org;	/* Frame offsets for
							   back and front
							   frame buffers into
							   lcddma_base/handle */
	unsigned long		vis_frame_org;		/* Offset of visible
							   frame buffer.
							   = frame0/1_org */
	unsigned long		src_frame_org;		/* Offset of source
							   frame for drawing
							   operations.
							   = frame0/1_org */
	unsigned long		dst_frame_org;		/* Offset of dest
							   frame for drawing
							   operations.
							   = frame0/1_org */
	unsigned long		view_org;		/* View offset into
							   lcddma_base/handle+
							   vis_frame_org.
							   Used for panning */
	unsigned long		palette_size;
	int			xscale, yscale, mirror;	/* transformations.
							   rotate is stored in
							   fb_info->var */

	u32			pseudo_palette[17];

	struct gfx_dma		gfx;			/* Accelerator */
	struct omapfb_rqueue	rqueue;
	struct lcd_panel	*panel;			/* LCD panel */
	struct lcd_ctrl         *ctrl;			/* LCD controller */

	struct fb_info		*fb_info;		/* Linux fbdev
							   framework data */
	struct device		*dev;
};

extern struct lcd_panel h3_panel;
extern struct lcd_panel h2_panel;
extern struct lcd_panel p2_panel;
extern struct lcd_panel osk_panel;
extern struct lcd_panel innovator1610_panel;
extern struct lcd_panel innovator1510_panel;

extern struct lcd_ctrl omapfb_lcdc_ctrl;

#endif /* __KERNEL__ */

#endif /* __OMAPFB_H */
