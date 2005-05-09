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
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/mach-types.h>

#include <asm/arch/dma.h>
#include <asm/arch/irqs.h>
#include <asm/arch/mux.h>
#include <asm/arch/board.h>

#include "omapfb.h"

// #define OMAPFB_DBG_FIFO 1
// #define OMAPFB_DBG 1

#include "debug.h"

#define COPY_MODE_REV_DIR	0x01
#define COPY_MODE_TRANSPARENT	0x02
#define COPY_MODE_IMAGE		0x04

#ifdef OMAPFB_DBG_FIFO
struct gfx_stat {
	unsigned long f_run[GFX_FIFO_SIZE];
} stat;
#endif

static unsigned int	def_accel;
static unsigned long	def_vram;
static long		def_vxres;
static long		def_vyres;
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
	{ FBCAPS_MANUAL_UPDATE, "manual update" },
	{ FBCAPS_SET_BACKLIGHT, "backlight setting" },
};

/*
 * ---------------------------------------------------------------------------
 * LCD panel
 * ---------------------------------------------------------------------------
 */
static struct lcd_panel *panels[] = {
#ifdef CONFIG_MACH_OMAP_H2
	&h2_panel,
#endif
#ifdef CONFIG_MACH_OMAP_H3
	&h3_panel,
#endif
#ifdef CONFIG_MACH_OMAP_PERSEUS2
	&p2_panel,
#endif
#ifdef CONFIG_MACH_OMAP_OSK
	&osk_panel,
#endif
#ifdef CONFIG_MACH_OMAP_INNOVATOR
#ifdef CONFIG_ARCH_OMAP1510
	&innovator1510_panel,
#endif
#ifdef CONFIG_ARCH_OMAP16XX
	&innovator1610_panel,
#endif
#endif
};

static struct lcd_ctrl *ctrls[] = {
#ifdef CONFIG_FB_OMAP_INTERNAL_LCDC
	&omapfb_lcdc_ctrl,
#endif
};

static struct omapfb_request *omapfb_rqueue_alloc_req(struct omapfb_rqueue *rq)
{
	struct omapfb_request *req;

	down(&rq->free_sema);
	spin_lock(&rq->lock);
	req = list_entry(rq->free_list.next, struct omapfb_request, entry);
	list_del(&req->entry);
	spin_unlock(&rq->lock);
	return req;
}

static void __omapfb_rqueue_free_req(struct omapfb_rqueue *rq,
				     struct omapfb_request *req)
{
	list_add(&req->entry, &rq->free_list);
	up(&rq->free_sema);
}

static void omapfb_rqueue_process(void *data)
{
	struct omapfb_rqueue *rq = data;

	spin_lock(&rq->lock);
	while (!list_empty(&rq->pending_list)) {
		struct omapfb_request *req;

		req = list_entry(rq->pending_list.next,
				   struct omapfb_request, entry);
		list_del(&req->entry);
		spin_unlock(&rq->lock);
		rq->status |= req->function(&req->par);
		spin_lock(&rq->lock);
		__omapfb_rqueue_free_req(rq, req);
	}
	complete(&rq->rqueue_empty);
	spin_unlock(&rq->lock);
}

static void omapfb_rqueue_schedule_req(struct omapfb_rqueue *rq,
				       struct omapfb_request *req)
{
	spin_lock(&rq->lock);
	list_add_tail(&req->entry, &rq->pending_list);
	spin_unlock(&rq->lock);
	schedule_work(&rq->work);
}

static void omapfb_rqueue_sync(struct omapfb_rqueue *rq)
{
	int wait = 0;

	spin_lock(&rq->lock);
	if (!list_empty(&rq->pending_list)) {
		wait = 1;
		init_completion(&rq->rqueue_empty);
	}
	spin_unlock(&rq->lock);
	if (wait)
		wait_for_completion(&rq->rqueue_empty);
}

static void omapfb_rqueue_reset(struct omapfb_rqueue *rq, unsigned long *status)
{
	omapfb_rqueue_sync(rq);
	spin_lock(&rq->lock);
	*status = rq->status;
	rq->status = 0;
	spin_unlock(&rq->lock);
}

static void omapfb_rqueue_init(struct omapfb_rqueue *rq)
{
	int i;

	spin_lock_init(&rq->lock);
	sema_init(&rq->free_sema, OMAPFB_RQUEUE_SIZE);
	init_completion(&rq->rqueue_empty);
	INIT_WORK(&rq->work, omapfb_rqueue_process, rq);
	INIT_LIST_HEAD(&rq->free_list);
	INIT_LIST_HEAD(&rq->pending_list);
	for (i = 0; i < OMAPFB_RQUEUE_SIZE; i++)
		list_add(&rq->req_pool[i].entry, &rq->free_list);
}

static void omapfb_rqueue_cleanup(struct omapfb_rqueue *rq)
{
}

/*
 * ---------------------------------------------------------------------------
 * gfx DMA
 * ---------------------------------------------------------------------------
 */
/* Get a new logical channel from the gfx fifo. */
static void inline gfxdma_get_lch(struct gfx_dma *gfx, int *lch)
{
	DBGENTER(3);

	down(&gfx->f_free);

	spin_lock_bh(&gfx->spinlock);

	*lch = gfx->f_tail->lch_num;
	gfx->f_tail = gfx->f_tail->next;

	spin_unlock_bh(&gfx->spinlock);

	DBGLEAVE(3);
}

/* Set basic transfer params for the logical channel */
static inline void gfxdma_set_lch_params(int lch, int data_type,
					 int enumber, int fnumber,
				       	 unsigned long src_start, int src_amode,
				         unsigned long dst_start, int dst_amode)
{
	omap_set_dma_transfer_params(lch, data_type, enumber, fnumber, 0);
	omap_set_dma_src_params(lch, OMAP_DMA_PORT_EMIFF,
				src_amode, src_start);
	omap_set_dma_dest_params(lch, OMAP_DMA_PORT_EMIFF,
				 dst_amode, dst_start);
}

/* Set element and frame indexes for the logical channel, to support
 * image transformations
 */
static inline void gfxdma_set_lch_index(int lch, int src_eidx, int src_fidx,
					int dst_eidx, int dst_fidx)
{
	omap_set_dma_src_index(lch, src_eidx, src_fidx);
	omap_set_dma_dest_index(lch, dst_eidx, dst_fidx);
}

/* Set color parameter for the logical channel, to support constant fill and
 * transparent copy operations
 */
static inline void gfxdma_set_lch_color(int lch, u32 color,
					enum omap_dma_color_mode mode)
{
	omap_set_dma_color_mode(lch, mode, color);
}


/* Start a new transfer consisting of a single DMA logical channel,
 * or a chain (f_run > 1). Can be called in interrupt context.
 * gfx->spinlock must be held.
 */
static void inline gfxdma_start_chain(struct gfx_dma *gfx)
{
	DBGENTER(3);

	gfx->f_run = gfx->f_wait;
#ifdef OMAPFB_DBG_FIFO
	stat.f_run[gfx->f_run - 1]++;
#endif
	gfx->f_wait = 0;
	omap_enable_dma_irq(gfx->f_chain_end->lch_num, OMAP_DMA_BLOCK_IRQ);
	/* Let it go */
	DBGPRINT(1, "start %d\n", gfx->f_head->lch_num);
	omap_start_dma(gfx->f_head->lch_num);
	gfx->f_chain_end = gfx->f_chain_end->next;

	DBGLEAVE(3);
}

/* Enqueue a logical channel, that has been set up. If no other transfers
 * are pending start this new one right away. */
static void inline gfxdma_enqueue(struct gfx_dma *gfx, int lch)
{
	DBGENTER(3);

	spin_lock_bh(&gfx->spinlock);
	DBGPRINT(3, "run:%d wait:%d\n", gfx->f_run, gfx->f_wait);
	if (gfx->f_wait) {
		DBGPRINT(1, "link %d, %d\n", gfx->f_chain_end->lch_num, lch);
		omap_dma_link_lch(gfx->f_chain_end->lch_num, lch);
		gfx->f_chain_end = gfx->f_chain_end->next;
	}
	omap_disable_dma_irq(lch, OMAP_DMA_BLOCK_IRQ);

	gfx->f_wait++;

	if (!gfx->f_run)
		gfxdma_start_chain(gfx);
	spin_unlock_bh(&gfx->spinlock);

	DBGLEAVE(3);
}

/* Called by DMA core when the last transfer ended, or there is an error
 * condition. We dispatch handling of the end of transfer case to a tasklet.
 * Called in interrupt context.
 */
static void gfxdma_handler(int lch, u16 ch_status, void *data)
{
	struct gfx_dma *gfx = (struct gfx_dma *)data;
	int done = 0;

	DBGENTER(3);

	DBGPRINT(4, "lch=%d status=%#010x\n", lch, ch_status);
	if (unlikely(ch_status & (OMAP_DMA_TOUT_IRQ | OMAP_DMA_DROP_IRQ))) {
		PRNERR("gfx DMA error. status=%#010x\n", ch_status);
		done = 1;
	} else if (likely(ch_status & OMAP_DMA_BLOCK_IRQ))
		done = 1;
	if (likely(done)) {
		gfx->done = 1;
		tasklet_schedule(&gfx->dequeue_tasklet);
	}

	DBGLEAVE(3);
}

/* Let the DMA core know that the last transfer has ended. If there are
 * pending transfers in the fifo start them now.
 * Called in interrupt context.
 */
static void gfxdma_dequeue_tasklet(unsigned long data)
{
	struct gfx_dma *gfx = (struct gfx_dma *)data;
	struct gfx_lchannel *f_chain;

	DBGENTER(3);

	/* start an already programmed transfer
	 */
	while (likely(gfx->done)) {
		gfx->done = 0;
		spin_lock(&gfx->spinlock);
		f_chain = gfx->f_head;
		omap_stop_dma(f_chain->lch_num);
		/* Would be better w/o a loop.. */
		while (gfx->f_run--) {
			if (gfx->f_run)
				omap_dma_unlink_lch(f_chain->lch_num,
						    f_chain->next->lch_num);
			f_chain = f_chain->next;
			up(&gfx->f_free);
		}
		gfx->f_run = 0;
		gfx->f_head = f_chain;
		if (likely(gfx->f_wait))
			gfxdma_start_chain(gfx);
		else
			complete(&gfx->sync_complete);
		spin_unlock(&gfx->spinlock);
	}

	DBGLEAVE(3);
}

/* Wait till any pending transfers end. */
static void gfxdma_sync(struct gfx_dma *gfx)
{
	int wait = 0;

	DBGENTER(1);

	for (;;) {
		spin_lock_bh(&gfx->spinlock);
		if (gfx->f_run + gfx->f_wait) {
			wait = 1;
			init_completion(&gfx->sync_complete);
		}
		spin_unlock_bh(&gfx->spinlock);
		if (wait) {
			wait_for_completion(&gfx->sync_complete);
			wait = 0;
		} else
			break;
	}

	DBGLEAVE(1);
}

/* Initialize the gfx DMA object.
 * Allocate DMA logical channels according to the fifo size.
 * Set the channel parameters that will be the same for all transfers.
 */
static int gfxdma_init(struct gfx_dma *gfx)
{
	int			r = 0;
	int			i;

	DBGENTER(1);

	for (i = 0; i < GFX_FIFO_SIZE; i++) {
		int next_idx;
		int lch_num;

		r = omap_request_dma(0, OMAPFB_DRIVER,
				     gfxdma_handler, gfx, &lch_num);
		if (r) {
			int j;

			PRNERR("unable to get GFX DMA %d\n", i);
			for (j = 0; j < i; j++)
				omap_free_dma(lch_num);
			r = -1;
			goto exit;
		}
		omap_set_dma_src_data_pack(lch_num, 1);
		omap_set_dma_src_burst_mode(lch_num, OMAP_DMA_DATA_BURST_4);
		omap_set_dma_dest_data_pack(lch_num, 1);
		omap_set_dma_dest_burst_mode(lch_num, OMAP_DMA_DATA_BURST_4);

		gfx->fifo[i].lch_num = lch_num;

		next_idx = i < GFX_FIFO_SIZE - 1 ? i + 1 : 0;
		gfx->fifo[next_idx].prev = &gfx->fifo[i];
		gfx->fifo[i].next = &gfx->fifo[next_idx];
	}
	gfx->f_head = gfx->f_tail = gfx->f_chain_end = &gfx->fifo[0];
	sema_init(&gfx->f_free, GFX_FIFO_SIZE);

	spin_lock_init(&gfx->spinlock);

	tasklet_init(&gfx->dequeue_tasklet, gfxdma_dequeue_tasklet,
		     (unsigned long)gfx);

	init_completion(&gfx->sync_complete);
exit:
	DBGLEAVE(1);
	return r;
}

/* Clean up the gfx DMA object */
static void gfxdma_cleanup(struct gfx_dma *gfx)
{
	int i;

	DBGENTER(1);

	for (i = 0; i < GFX_FIFO_SIZE; i++)
		omap_free_dma(gfx->fifo[i].lch_num);

	DBGLEAVE(1);
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
	unsigned long mem_size;
	int r;

	DBGENTER(1);

	r = fbdev->ctrl->init(fbdev);
	if (r < 0)
		goto exit;
	fbdev->ctrl->get_mem_layout(fbdev, &mem_size, &fbdev->frame0_org);

	if (def_vram) {
		if (mem_size > def_vram) {
			PRNERR("specified frame buffer memory too small\n");
			r = -ENOMEM;
			goto cleanup_ctrl;
		}
		mem_size = def_vram;
	}
	fbdev->lcddma_mem_size = PAGE_SIZE << get_order(mem_size);
	fbdev->lcddma_base = dma_alloc_writecombine(fbdev->dev,
						    fbdev->lcddma_mem_size,
						    &fbdev->lcddma_handle,
						    GFP_KERNEL);
	if (fbdev->lcddma_base == NULL) {
		PRNERR("unable to allocate fb DMA memory\n");
		r = -ENOMEM;
		goto cleanup_ctrl;
	}

	memset(fbdev->lcddma_base, 0, fbdev->lcddma_mem_size);

	DBGPRINT(1, "lcddma_base=%#10x lcddma_handle=%#10x lcddma_mem_size=%d"
		 "palette_size=%d frame0_org=%d palette_org=%d\n",
		 fbdev->lcddma_base, fbdev->lcddma_handle,
		 fbdev->lcddma_mem_size,
		 fbdev->palette_size,
		 fbdev->frame0_org, fbdev->palette_org);

	DBGLEAVE(1);
	return 0;
cleanup_ctrl:
        fbdev->ctrl->cleanup(fbdev);
exit:
	DBGLEAVE(1);
	return r;
}

static void ctrl_cleanup(struct omapfb_device *fbdev)
{
	fbdev->ctrl->cleanup(fbdev);
	dma_free_writecombine(fbdev->dev, fbdev->lcddma_mem_size,
			      fbdev->lcddma_base, fbdev->lcddma_handle);
}

static void ctrl_change_mode(struct omapfb_device *fbdev)
{
	fbdev->ctrl->change_mode(fbdev);
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

#ifdef OMAPFB_DBG_FIFO
	memset(&stat, 0, sizeof(stat));
#endif

	DBGLEAVE(1);
	return 0;
}

/* Called when the omapfb device is closed. We make sure that any pending
 * gfx DMA operations are ended, before we return. */
static int omapfb_release(struct fb_info *info, int user)
{
	struct omapfb_device *dev = (struct omapfb_device *)info->par;
	int sync = 0;

	DBGENTER(1);

	spin_lock_bh(&dev->gfx.spinlock);
	if (dev->gfx.f_run)
		sync = 1;
	spin_unlock_bh(&dev->gfx.spinlock);
	if (sync) {
		gfxdma_sync(&dev->gfx);
	}
#ifdef OMAPFB_DBG_FIFO
	{
		int i;
		for (i = 0; i < GFX_FIFO_SIZE; i++)
			printk(KERN_INFO "f_run[%d]=%lu\n", i + 1,
					stat.f_run[i]);
	}
#endif

	DBGLEAVE(1);
	return 0;
}

/* Store a single color palette entry into a pseudo palette or the hardware
 * palette if one is available. For now we support only 16bpp and thus store
 * the entry only to the pseudo palette.
 */
static int omapfb_setcolreg(u_int regno, u_int red, u_int green,
			    u_int blue, u_int transp,
			    struct fb_info *info)
{
	u16 pal;
	int r = 0;

	DBGENTER(2);

	if (regno >= 16) {
		r = -1;
		goto exit;
	}
	pal = ((red >> 11) << 11) | ((green >> 10) << 5) | (blue >> 11);
	((u32 *)(info->pseudo_palette))[regno] = pal;

exit:
	DBGLEAVE(2);
	return r;
}

static int omapfb_suspend(struct device *dev, pm_message_t mesg, u32 level);
static int omapfb_resume(struct device *dev, u32 level);

static int omapfb_blank(int blank, struct fb_info *info)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)info->par;
	int r = 0;

	DBGENTER(1);
	switch (blank) {
	case VESA_NO_BLANKING:
		omapfb_resume(fbdev->dev, 0);
		break;
	case VESA_POWERDOWN:
		omapfb_suspend(fbdev->dev, 0, 0);
		break;
	default:
		r = -EINVAL;
	}

	DBGLEAVE(1);
	return r;
}

/* Setup a constant fill DMA transfer. Destination must be elem size aligned. */
static inline void fill_block(struct omapfb_device *fbdev,
			      unsigned long dst, unsigned long enumber,
			      unsigned long height, int esize, u32 color)
{
	unsigned long	fidx;
	int		lch;

	DBGPRINT(2, "dst:%#010x enumber:%d height:%d esize:%d color:%#010x\n",
		 dst, enumber, height, esize, color);
	fidx = fbdev->fb_info->fix.line_length - enumber * esize + 1;
	gfxdma_get_lch(&fbdev->gfx, &lch);
	gfxdma_set_lch_params(lch, dma_elem_type[esize], enumber, height,
			      0, OMAP_DMA_AMODE_CONSTANT,
			      dst, OMAP_DMA_AMODE_DOUBLE_IDX);
	gfxdma_set_lch_index(lch, 0, 0, 1, fidx);
	gfxdma_set_lch_color(lch, color, OMAP_DMA_CONSTANT_FILL);
	gfxdma_enqueue(&fbdev->gfx, lch);

	DUMP_DMA_REGS(lch);
}

/* Fill the specified rectangle with a solid color.
 * ROP_XOR and bpp<8 can't be handled by the DMA hardware.
 * When frame flipping is in effect use the destination frame.
 * We'll make our best to use the largest possible elem size, doing the fill
 * in more parts if alignment requires us to do so.
 */
static int omapfb_fillrect(void *data)
{
	struct omapfb_fillrect_params *par = data;
	struct omapfb_device *fbdev = (struct omapfb_device *)par->fbi->par;

	int		dx = par->rect.dx, dy = par->rect.dy;
	int		vxres = par->fbi->var.xres_virtual;
	int		vyres = par->fbi->var.yres_virtual;
	int		width = par->rect.width, height = par->rect.height;
	unsigned long	dst;
	u32		color;
	int		bpp;
	int		enumber, esize;
	int		r = 0;

	DBGENTER(2);
	bpp = par->fbi->var.bits_per_pixel;
	/* bpp < 8 is tbd.
	 * We can't do ROP_XOR with DMA
	 * If IRQs are disabled we can't use DMA
	 */
	if (bpp < 8 || par->rect.rop == ROP_XOR || irqs_disabled()) {
		r = OMAPFB_FILLRECT_FAILED;
		goto exit;
	}
	/* Clipping */
	if (!width || !height || dx > vxres || dy > vyres)
		goto exit;
	if (dx + width > vxres)
		width = vxres - dx;
	if (dy + height > vyres)
		height = vyres - dy;

	if (bpp == 12)
		bpp = 16;
	width = width * bpp / 8;

	dst = fbdev->lcddma_handle + fbdev->dst_frame_org;
	dst += dx * bpp / 8 + dy * par->fbi->fix.line_length;

	color = par->rect.color;
	switch (bpp) {
	case 8:
		color |= color << 8;
		/* Fall through */
	case 16:
		color |= color << 16;
	}

	if ((dst & 3) || width < 4) {
		if (!(dst & 1) && width > 1) {
			esize = 2;
			enumber = 1;
			width -= 2;
		} else {
			esize = 1;
			enumber = 4 - (esize & 3);
			if (enumber > width)
				enumber = width;
			width -= enumber;
		}
		fill_block(fbdev, dst, enumber, height, esize, color);
		dst = (dst + 3) & ~3;
	}
	if (width) {
		enumber = width / 4;
		fill_block(fbdev, dst, enumber, height, 4, color);
		dst += enumber * 4;
		width -= enumber * 4;
	}
	if (width) {
		if (width == 2) {
			esize = 2;
			enumber = 1;
		} else {
			esize = 1;
			enumber = width;
		}
		fill_block(fbdev, dst, enumber, height, esize, color);
	}

exit:
	DBGLEAVE(2);
	return r;
}

static int omapfb_schedule_fillrect(struct fb_info *fbi,
				     const struct fb_fillrect *rect)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;
	struct omapfb_request *req;

	if ((req = omapfb_rqueue_alloc_req(&fbdev->rqueue)) == NULL)
		return -ENOMEM;
	req->function = omapfb_fillrect;
	req->par.fillrect.fbi = fbi;
	req->par.fillrect.rect = *rect;
	omapfb_rqueue_schedule_req(&fbdev->rqueue, req);
	return fbdev->rqueue.status ? OMAPFB_GFX_STATUS_CHANGED : 0;
}

/* Setup a gfx DMA transfer to a rectangular area.
 * A color parameter can be specified for transparent copy.
 * Transfer direction can be setup to use either incremental or decremental
 * addresses.
 * Source and destination must be elem size aligned.
 */
static inline void transfer_block(struct omapfb_device *fbdev,
				  unsigned long src, unsigned long dst,
				  unsigned long img_width,
				  unsigned long enumber, unsigned long height,
				  int esize, u32 trans_color, int flags)
{
	s16	eidx;
	s16	s_fidx, d_fidx;
	int	lch;

	eidx = 1;
	s_fidx = img_width - enumber * esize + 1;
	d_fidx = fbdev->fb_info->fix.line_length - enumber * esize + 1;
	if (flags & COPY_MODE_REV_DIR) {
		eidx = -2 * esize + 1;
		s_fidx = -s_fidx + eidx + 1;
		d_fidx = -d_fidx + eidx + 1;
	}

	DBGPRINT(2, "src:%#010x dst:%#010x enumber:%d height:%d "
		 "esize:%d eidx:%d s_fidx:%d d_fidx bg_color:%#010x flags:%d\n",
		 src, dst, enumber, height, esize, eidx, s_fidx, d_fidx,
		 bg_color, flags);

	gfxdma_get_lch(&fbdev->gfx, &lch);
	gfxdma_set_lch_params(lch, dma_elem_type[esize], enumber, height,
			      src, OMAP_DMA_AMODE_DOUBLE_IDX,
			      dst, OMAP_DMA_AMODE_DOUBLE_IDX);
	gfxdma_set_lch_index(lch, eidx, s_fidx, eidx, d_fidx);
	if (flags & COPY_MODE_TRANSPARENT)
		gfxdma_set_lch_color(lch, trans_color,
				     OMAP_DMA_TRANSPARENT_COPY);
	else
		gfxdma_set_lch_color(lch, 0, OMAP_DMA_COLOR_DIS);
	gfxdma_enqueue(&fbdev->gfx, lch);

	DUMP_DMA_REGS(lch);
}

/* Copy a rectangular area or an image to another rectangular area.
 * A color parameter can be specified for transparent copy.
 * Transfer direction can be setup to use either incremental or decremental
 * addresses.
 * Currently both source and destination area must be entirely contained in
 * frame buffer memory.
 * The largest possible transfer elem size will be determined according to
 * source and destination address alignment, dividing the transfer into more
 * parts if necessary.
 */
static inline void copy_data(struct omapfb_device *fbdev,
			     unsigned long src, unsigned long dst,
			     unsigned long width, unsigned long height,
			     u32 trans_color, int flags)
{
	struct fb_info	 *fbi = fbdev->fb_info;
	int		 esize, stripe_esize;
	int		 step, rest, enumber;
	unsigned long	 img_width;
	static const int esize_arr[] = {4, 1, 2, 1};
	int		 rev;

	/* Check alignment constraints */
	esize = esize_arr[(src ^ dst) & 3];

	rev = flags & COPY_MODE_REV_DIR;
	if (rev) {
		rest = src & (esize - 1);
		if (rest > width)
			rest = width;
		src -= rest ? rest : esize;
		dst -= rest ? rest : esize;
	} else {
		rest = esize - (src & (esize - 1));
		if (rest > width)
			rest = width;
	}
	if (width < esize)
		rest = width;

	img_width = flags & COPY_MODE_IMAGE ? width : fbi->fix.line_length;

	DBGPRINT(2, "\nrev=%d src=%#010lx dst=%#010lx \n"
		 "esize=%d width=%d rest=%d\n",
		 rev, src, dst, esize, width, rest);
	if (rest) {
		/* Transfer this unaligned stripe, so that afterwards
		 * we have both src and dst 16bit or 32bit aligned.
		 */
		if (rest == 2) {
			/* Area body is 32bit aligned */
			stripe_esize = 2;
			enumber = 1;
			step = rev ? -esize : 2;
			width -= 2;
		} else {
			stripe_esize = 1;
			enumber = rest;
			step = rev ? -esize : rest;
		}
		transfer_block(fbdev, src, dst, img_width, enumber, height,
			       stripe_esize, trans_color, flags);
		src += step;
		dst += step;
	}
	if (width) {
		/* Transfer area body */
		enumber = (width & ~(esize - 1)) / esize;
		transfer_block(fbdev, src, dst, img_width, enumber, height,
			       esize, trans_color, flags);
		step = enumber * esize;
		width -= step;
		if (rev)
			step = -step + esize - width;
		src += step;
		dst += step;
	}
	if (width) {
		/* Transfer the remaining unaligned stripe */
		if (width == 2) {
			stripe_esize = 2;
			enumber = 1;
		} else {
			stripe_esize = 1;
			enumber = width;
		}
		transfer_block(fbdev, src, dst, img_width, enumber, height,
			       stripe_esize, trans_color, flags);
	}

	DBGLEAVE(2);
}

/* Copy a rectangular area in the frame buffer to another rectangular area.
 * Calculate the source and destination addresses.
 * Transfer direction will be determined taking care of possible area
 * overlapping.
 * Currently both source and destination area must be entirely contained in
 * frame buffer memory, in case of frame flipping source and destination frame
 * respectively.
 */
static int omapfb_copyarea(void *data)
{
	struct omapfb_copyarea_params *par = data;
	struct omapfb_device *fbdev = (struct omapfb_device *)par->fbi->par;

	int		width = par->area.width, height = par->area.height;
	int		sx = par->area.sx, sy = par->area.sy;
	int		dx = par->area.dx, dy = par->area.dy;
	unsigned long	dst, dst_ofs, src, src_ofs;
	unsigned long	end_ofs;
	int		bpp = par->fbi->var.bits_per_pixel;
	int		flags;
	int		r = 0;

	DBGENTER(2);

	if (!width || !height)
		goto exit;

	/* Bpp < 8 is tbd. If IRQs are disabled we can't use DMA */
	if (bpp < 8 || irqs_disabled()) {
		r = OMAPFB_COPYAREA_FAILED;
		goto exit;
	}

	src = fbdev->lcddma_handle;
	dst = src;
	src_ofs = fbdev->src_frame_org + sx * bpp / 8 +
		  sy * par->fbi->fix.line_length;
	dst_ofs = fbdev->dst_frame_org + dx * bpp / 8 +
		  dy * par->fbi->fix.line_length;
	end_ofs = (height - 1) * par->fbi->fix.line_length + width * bpp / 8;
	src += src_ofs;
	dst += dst_ofs;

	DBGPRINT(2, "src:%#010lx dst:%#010lx end_ofs:%#010lx\n",
		    src, dst, end_ofs);

	/* Currently we support only transfers where both source and destination
	 * area is contained entirely in fbmem. This is because of DMA memory
	 * constraints.
	 */
	if (src_ofs + end_ofs > fbdev->lcddma_mem_size ||
	    dst_ofs + end_ofs > fbdev->lcddma_mem_size) {
		r = OMAPFB_COPYAREA_FAILED;
		goto exit;
	}

	flags = 0;
	if (par->area.rev_dir) {
		flags = COPY_MODE_REV_DIR;
		src += end_ofs;
		dst += end_ofs;
	}
	if (par->area.trans_color != -1)
		flags |= COPY_MODE_TRANSPARENT;

	width = width * bpp / 8;
	copy_data(fbdev, src, dst, width, height, par->area.trans_color, flags);
exit:
	DBGLEAVE(2);
	return r;
}

static int omapfb_schedule_copyarea(struct fb_info *fbi,
				     const struct fb_copyarea_ext *area)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;
	struct omapfb_request *req;

	if ((req = omapfb_rqueue_alloc_req(&fbdev->rqueue)) == NULL)
		return -ENOMEM;
	req->function = omapfb_copyarea;
	req->par.copyarea.fbi = fbi;
	req->par.copyarea.area = *area;
	omapfb_rqueue_schedule_req(&fbdev->rqueue, req);
	return fbdev->rqueue.status ? OMAPFB_GFX_STATUS_CHANGED : 0;
}

/* Copy an image to a rectangular area in the frame buffer.
 * A color parameter can be specified for transparent copy.
 * Calculate the source and destination addresses.
 * Transfer direction will be determined taking care of possible area
 * overlapping.
 * Currently both source and destination area must be entirely contained in
 * frame buffer memory, in case of frame flipping source and destination frame
 * respectively.
 */
static int do_imageblit(void *data)
{
	struct omapfb_imageblit_params *par = data;
	struct omapfb_device *fbdev = (struct omapfb_device *)par->fbi->par;

	int		width = par->image.width, height = par->image.height;
	int		dx = par->image.dx, dy = par->image.dy;
	const char	*img_data = par->image.data;
	unsigned long	dst, dst_ofs;
	unsigned long	dst_end_ofs;
	int		bpp = par->fbi->var.bits_per_pixel;
	u32		bg_color;
	int		r = 0;

	DBGENTER(2);

	if (!width || !height)
		goto exit;

	/* bpp conversion is not supported, let the default function handle it.
	 * Note that image->depth is either 1 for monochrome image, or equals
	 * bpp of the current video mode, so we can't rely on it.
	 * If IRQs are disabled we can't use DMA.
	 */
	if (bpp < 8 || par->image.depth != bpp || irqs_disabled()) {
		r = OMAPFB_IMGBLIT_FAILED;
		goto exit;
	}

	dst = fbdev->lcddma_handle;
	dst_ofs = fbdev->dst_frame_org +
		  dx * bpp / 8 + dy * par->fbi->fix.line_length;
	dst_end_ofs = (height - 1) * par->fbi->fix.line_length +
		  width * bpp / 8;
	dst += dst_ofs;

	DBGPRINT(2, "data:%#010lx dst:%#010lx dst_end_ofs:%#010lx\n",
		    img_data, dst, dst_end_ofs);

	 /* Check that both source and destination is DMA -able */
	if (dst_ofs + dst_end_ofs > fbdev->lcddma_mem_size) {
		r = OMAPFB_IMGBLIT_FAILED;
		goto exit;
	}

	if (((unsigned long)img_data < (unsigned long)fbdev->lcddma_base) ||
	    ((unsigned long)img_data + width * bpp / 8 * height >
	     (unsigned long)fbdev->lcddma_base + fbdev->lcddma_mem_size)) {
		r = OMAPFB_IMGBLIT_FAILED;
		goto exit;
	}

	bg_color = par->image.bg_color;
	if (par->flags & COPY_MODE_TRANSPARENT) {
		switch (bpp) {
		case 8:
			bg_color |= bg_color << 8;
			/* Fall through */
		case 16:
			bg_color |= bg_color << 16;
		}
	}

	width = width * bpp / 8;
	copy_data(fbdev, (unsigned long)img_data, dst, width, height,
		  bg_color, par->flags | COPY_MODE_IMAGE);
exit:
	DBGLEAVE(2);
	return r;
}

static int omapfb_schedule_imageblit(struct fb_info *fbi,
				      const struct fb_image *image, int flags)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;
	struct omapfb_request *req;

	if ((req = omapfb_rqueue_alloc_req(&fbdev->rqueue)) == NULL)
		return -ENOMEM;
	req->function = do_imageblit;
	req->par.imageblit.fbi = fbi;
	req->par.imageblit.image = *image;
	req->par.imageblit.flags = flags;
	omapfb_rqueue_schedule_req(&fbdev->rqueue, req);
	return fbdev->rqueue.status ? OMAPFB_GFX_STATUS_CHANGED : 0;
}

/* Set fb_info.fix fields and also updates fbdev.
 * When calling this fb_info.var must be set up already.
 */
static void set_fb_fix(struct omapfb_device *fbdev)
{
	struct fb_info		 *fbi = fbdev->fb_info;
	struct fb_fix_screeninfo *fix = &fbi->fix;
	struct fb_var_screeninfo *var = &fbi->var;
	int frame_size;

	strncpy(fix->id, OMAPFB_DRIVER, sizeof(fix->id));
	fix->type		= FB_TYPE_PACKED_PIXELS;
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
	fix->smem_len		= fbdev->lcddma_mem_size - fbdev->frame0_org;
	fix->smem_start 	= fbdev->lcddma_handle + fbdev->frame0_org;

	/* Set the second frame buffer offset for flipping if there is
	 * room for it. */
	frame_size = fix->line_length * var->yres;
	fbdev->frame1_org = fbdev->frame0_org + frame_size;
	if (fbdev->frame1_org + frame_size > fbdev->lcddma_mem_size)
		fbdev->frame1_org = 0;
	fbdev->vis_frame_org = fbdev->src_frame_org = fbdev->dst_frame_org =
		fbdev->frame0_org;

	fbdev->view_org = var->yoffset * fix->line_length +
			  var->xoffset * var->bits_per_pixel / 8;
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

	bpp = var->bits_per_pixel = fbdev->panel->video_mode->bpp;
	if (bpp != 16)
		/* Not yet supported */
		return -1;
	switch (var->rotate) {
	case 0:
	case 180:
		var->xres = fbdev->panel->video_mode->x_res;
		var->yres = fbdev->panel->video_mode->y_res;
		break;
	case 90:
	case 270:
		var->xres = fbdev->panel->video_mode->y_res;
		var->yres = fbdev->panel->video_mode->x_res;
		break;
	default:
		return -1;
	}
	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;
	max_frame_size = fbdev->lcddma_mem_size - fbdev->frame0_org;
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

	/* TODO: video timing params, sync */
	var->pixclock		= -1;
	var->left_margin	= -1;
	var->right_margin	= -1;
	var->upper_margin	= -1;
	var->lower_margin	= -1;
	var->hsync_len		= -1;
	var->vsync_len		= -1;

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
			set_fb_fix(fbdev);
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
			set_fb_fix(fbdev);
			ctrl_change_mode(fbdev);
		}
	}

	DBGLEAVE(1);
	return r;
}

/* Set mirror to vertical axis and switch to the new mode. */
static int omapfb_mirror(struct fb_info *fbi, int mirror)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;
	int r = 0;

	DBGENTER(1);

	mirror = mirror ? 1 : 0;
	if (cpu_is_omap1510())
		r = -EINVAL;
	else if (mirror != fbdev->mirror) {
		fbdev->mirror = mirror;
		ctrl_change_mode(fbdev);
	}

	DBGLEAVE(1);
	return r;
}

/* Set x,y scale and switch to the new mode */
static int omapfb_scale(struct fb_info *fbi,
		      unsigned int xscale, unsigned int yscale)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;
	int r = 0;

	DBGENTER(1);

	if (cpu_is_omap1510())
		r = -EINVAL;
	else if (xscale != fbdev->xscale || yscale != fbdev->yscale) {
		if (fbi->var.xres * xscale > fbi->var.xres_virtual ||
		    fbi->var.yres * yscale > fbi->var.yres_virtual)
			r = -EINVAL;
		else {
			fbdev->xscale = xscale;
			fbdev->yscale = yscale;
			ctrl_change_mode(fbdev);
		}
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
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;

	DBGENTER(1);

	set_fb_fix(fbdev);
	ctrl_change_mode(fbdev);

	DBGLEAVE(1);
	return 0;
}

/* Frame flipping support. Assign the primary or the secondary frame to the
 * visible frame, as well as the source and destination frames for graphics
 * operations like rectangle fill and area copy. Flipping is only possible
 * if we have enough video memory for the secondary frame.
 */
static int omapfb_select_vis_frame(struct fb_info *fbi, unsigned int vis_idx)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;

	if (vis_idx > 1 || (vis_idx == 1 && !fbdev->frame1_org))
		return -EINVAL;
	fbdev->vis_frame_org = vis_idx ? fbdev->frame1_org : fbdev->frame0_org;
	ctrl_change_mode(fbdev);
	return 0;
}

static int omapfb_select_src_frame(struct fb_info *fbi, unsigned int src_idx)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;

	if (src_idx > 1 || (src_idx == 1 && !fbdev->frame1_org))
		return -EINVAL;
	fbdev->src_frame_org = src_idx ? fbdev->frame1_org : fbdev->frame0_org;
	return 0;
}

static int omapfb_select_dst_frame(struct fb_info *fbi, unsigned int dst_idx)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;

	if (dst_idx > 1 || (dst_idx == 1 && !fbdev->frame1_org))
		return -EINVAL;
	fbdev->dst_frame_org = dst_idx ? fbdev->frame1_org : fbdev->frame0_org;
	DBGPRINT(1, "dst_frame_org=%#010x\n", fbdev->dst_frame_org);
	return 0;
}

/* Get the address of the primary and secondary frames */
static int omapfb_get_frame_offset(struct fb_info *fbi,
				   struct fb_frame_offset *fb_offset)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;

	if (fb_offset->idx > 1)
		return -EINVAL;
	if (fb_offset->idx == 1 && !fbdev->frame1_org)
		return -EINVAL;
	fb_offset->offset = fb_offset->idx ? fbdev->frame1_org :
		fbdev->frame0_org;
	return 0;
}

static int omapfb_update_window(void *data)
{
	struct omapfb_update_window_params *par = data;
	struct omapfb_device *fbdev = (struct omapfb_device *)par->fbi->par;

	gfxdma_sync(&fbdev->gfx);
	if (fbdev->ctrl->update_window(fbdev, &par->win))
		return OMAPFB_UPDATE_FAILED;
	else
		return 0;
}

static int omapfb_schedule_update_window(struct fb_info *fbi,
					 struct fb_update_window *win)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;
	struct omapfb_request *req;

	if (!fbdev->ctrl->update_window ||
	    win->x >= fbi->var.xres || win->y >= fbi->var.yres)
		return -EINVAL;
	if (win->x + win->width >= fbi->var.xres)
		win->width = fbi->var.xres - win->x;
	if (win->y + win->height >= fbi->var.yres)
		win->height = fbi->var.yres - win->y;
	if (!win->width || !win->height)
		return 0;
	if ((req = omapfb_rqueue_alloc_req(&fbdev->rqueue)) == NULL)
		return -ENOMEM;
	req->function = omapfb_update_window;
	req->par.update_window.fbi = fbi;
	req->par.update_window.win = *win;
	omapfb_rqueue_schedule_req(&fbdev->rqueue, req);
	return fbdev->rqueue.status ? OMAPFB_GFX_STATUS_CHANGED : 0;
}

static int omapfb_schedule_full_update(struct fb_info *fbi)
{
	struct fb_update_window win;

	win.x = 0;
	win.y = 0;
	win.width = fbi->var.xres;
	win.height = fbi->var.yres;
	return omapfb_schedule_update_window(fbi, &win);
}

static unsigned long omapfb_get_caps(struct fb_info *fbi)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)fbi->par;
	unsigned long caps;

	caps = 0;
	caps |= fbdev->panel->get_caps(fbdev->panel);
	caps |= fbdev->ctrl->get_caps(fbdev);
	return caps;
}

static int omapfb_set_update_mode(struct omapfb_device *fbdev,
				  enum fb_update_mode new_mode);

static enum fb_update_mode omapfb_get_update_mode(struct omapfb_device *fbdev);

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
		struct fb_fillrect	rect;
		struct fb_copyarea_ext	area;
		struct fb_image		image;
		struct fb_scale		scale;
		struct fb_frame_offset	frame_offset;
		struct fb_update_window	update_window;
		unsigned int		frame_idx;
		unsigned int		mirror;
		enum fb_update_mode	update_mode;
		unsigned long		caps;
		unsigned long		rqueue_status;
	} p;
	int			r = 0;

	DBGENTER(2);

	BUG_ON(!ops);
	DBGPRINT(2, "cmd=%010x\n", cmd);
	switch (cmd)
	{
	case OMAPFB_FILLRECT:
		if (copy_from_user(&p.rect, (void __user *)arg, sizeof(p.rect)))
			r = -EFAULT;
		else
			r = omapfb_schedule_fillrect(fbi, &p.rect);
		break;
	case OMAPFB_COPYAREA:
		if (copy_from_user(&p.area, (void __user *)arg, sizeof(p.area)))
			r = -EFAULT;
		else
			r = omapfb_schedule_copyarea(fbi, &p.area);
		break;
	case OMAPFB_IMAGEBLIT:
		if (copy_from_user(&p.image, (void __user *)arg,
						sizeof(p.image)))
			r = -EFAULT;
		else
			r = omapfb_schedule_imageblit(fbi, &p.image, 0);
		break;
	case OMAPFB_TRANSPARENT_BLIT:
		if (copy_from_user(&p.image, (void __user *)arg,
						sizeof(p.image)))
			r = -EFAULT;
		else
			r = omapfb_schedule_imageblit(fbi, &p.image,
						      COPY_MODE_TRANSPARENT);
		break;
	case OMAPFB_MIRROR:
		if (get_user(p.mirror, (int __user *)arg))
			r = -EFAULT;
		else
			omapfb_mirror(fbi, p.mirror);
		break;
	case OMAPFB_SCALE:
		if (copy_from_user(&p.scale, (void __user *)arg,
						sizeof(p.scale)))
			r = -EFAULT;
		else
			r = omapfb_scale(fbi, p.scale.xscale, p.scale.yscale);
		break;
	case OMAPFB_SELECT_VIS_FRAME:
		if (get_user(p.frame_idx, (int __user *)arg))
			r = -EFAULT;
		else
			r = omapfb_select_vis_frame(fbi, p.frame_idx);
		break;
	case OMAPFB_SELECT_SRC_FRAME:
		if (get_user(p.frame_idx, (int __user *)arg))
			r = - EFAULT;
		else
			r = omapfb_select_src_frame(fbi, p.frame_idx);
		break;
	case OMAPFB_SELECT_DST_FRAME:
		if (get_user(p.frame_idx, (int __user *)arg))
			r = -EFAULT;
		else
			r = omapfb_select_dst_frame(fbi, p.frame_idx);
		break;
	case OMAPFB_GET_FRAME_OFFSET:
		if (copy_from_user(&p.frame_offset, (void __user *)arg,
				   sizeof(p.frame_offset)))
			r = -EFAULT;
		else {
			r = omapfb_get_frame_offset(fbi, &p.frame_offset);
			if (copy_to_user((void __user *)arg, &p.frame_offset,
					 sizeof(p.frame_offset)))
				r = -EFAULT;
		}
		break;
	case OMAPFB_SYNC_GFX:
		omapfb_rqueue_sync(&fbdev->rqueue);
		break;
	case OMAPFB_VSYNC:
		break;
	case OMAPFB_LATE_ACTIVATE:
		printk(KERN_WARNING OMAPFB_DRIVER
			": LATE_ACTIVATE obsoleted by SET_UPDATE_MODE.\n");
//		r = -EINVAL;
		break;
	case OMAPFB_SET_UPDATE_MODE:
		if (get_user(p.update_mode, (int __user *)arg))
			r = -EFAULT;
		else
			r = omapfb_set_update_mode(fbdev, p.update_mode);
		break;
	case OMAPFB_GET_UPDATE_MODE:
		p.update_mode = omapfb_get_update_mode(fbdev);
		if (put_user(p.update_mode, (enum fb_update_mode __user *)arg))
			r = -EFAULT;
		break;
	case OMAPFB_UPDATE_WINDOW:
		if (copy_from_user(&p.update_window, (void __user *)arg,
				   sizeof(p.update_window)))
			r = -EFAULT;
		else
			r = omapfb_schedule_update_window(fbi, &p.update_window);
		break;
	case OMAPFB_GET_CAPS:
		p.caps = omapfb_get_caps(fbi);
		if (put_user(p.caps, (unsigned long __user *)arg))
			r = -EFAULT;
		break;
	case OMAPFB_GET_GFX_STATUS:
		omapfb_rqueue_reset(&fbdev->rqueue, &p.rqueue_status);
		if (put_user(p.rqueue_status, (unsigned long *)arg))
			r = -EFAULT;
		break;
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
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_cursor	= soft_cursor,
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
static ssize_t omapfb_show_caps_num(struct device *dev, char *buf)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)dev->driver_data;

	return snprintf(buf, PAGE_SIZE, "%#010lx\n",
		omapfb_get_caps(fbdev->fb_info));
}

static ssize_t omapfb_show_caps_text(struct device *dev, char *buf)
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
static ssize_t omapfb_show_panel_name(struct device *dev, char *buf)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)dev->driver_data;

	return snprintf(buf, PAGE_SIZE, "%s\n", fbdev->panel->name);
}

static ssize_t omapfb_show_bklight_level(struct device *dev, char *buf)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)dev->driver_data;
	int r;

	if (fbdev->panel->get_bklight_level) {
		r = snprintf(buf, PAGE_SIZE, "%d\n",
			     fbdev->panel->get_bklight_level(fbdev->panel));
	} else
		r = -ENODEV;
	return r;
}

static ssize_t omapfb_store_bklight_level(struct device *dev, const char *buf,
					  size_t size)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)dev->driver_data;
	int r;

	if (fbdev->panel->set_bklight_level) {
		unsigned int level;

		if (sscanf(buf, "%10d", &level) == 1) {
			r = fbdev->panel->set_bklight_level(fbdev->panel,
							    level);
		} else
			r = -EINVAL;
	} else
		r = -ENODEV;
	return r ? r : size;
}

static ssize_t omapfb_show_bklight_max(struct device *dev, char *buf)
{
	struct omapfb_device *fbdev = (struct omapfb_device *)dev->driver_data;
	int r;

	if (fbdev->panel->get_bklight_level) {
		r = snprintf(buf, PAGE_SIZE, "%d\n",
			     fbdev->panel->get_bklight_max(fbdev->panel));
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
static ssize_t omapfb_show_ctrl_name(struct device *dev, char *buf)
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
	PRNERR("unable to register sysfs interface\n");
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

	BUG_ON(!fbdev->lcddma_base);
	info->fbops = &omapfb_ops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->screen_base = (char __iomem *) fbdev->lcddma_base
				+ fbdev->frame0_org;
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
		PRNERR("unable to allocate color map memory\n");

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
	case 8:
		omapfb_unregister_sysfs(fbdev);
	case 7:
		omapfb_set_update_mode(fbdev, FB_UPDATE_DISABLED);
	case 6:
		fbdev->panel->disable(fbdev->panel);
	case 5:
		gfxdma_cleanup(&fbdev->gfx);
	case 4:
		fbinfo_cleanup(fbdev);
	case 3:
		ctrl_cleanup(fbdev);
	case 2:
		fbdev->panel->cleanup(fbdev->panel);
	case 1:
		omapfb_rqueue_cleanup(&fbdev->rqueue);
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
	const struct omap_lcd_config *cfg;
	char name[17];
	int i;

	fbdev->panel = NULL;
	cfg = omap_get_config(OMAP_TAG_LCD, struct omap_lcd_config);
	if (cfg == NULL) {
		const char *def_name = NULL;

		if (machine_is_omap_h2())
			def_name = "h2";
		if (machine_is_omap_h3())
			def_name = "h3";
		if (machine_is_omap_perseus2())
			def_name = "p2";
		if (machine_is_omap_osk())
			def_name = "osk";
		if (machine_is_omap_innovator() && cpu_is_omap1610())
			def_name = "inn1610";
		if (machine_is_omap_innovator() && cpu_is_omap1510())
			def_name = "inn1510";
		if (def_name == NULL)
			return -1;
		strncpy(name, def_name, sizeof(name) - 1);
	} else
		strncpy(name, cfg->panel_name, sizeof(name) - 1);
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
	const struct omap_lcd_config *cfg;
	char name[17];
	int i;

	fbdev->ctrl = NULL;
	cfg = omap_get_config(OMAP_TAG_LCD, struct omap_lcd_config);
	if (cfg == NULL) {
		strcpy(name, "internal");
	} else
		strncpy(name, cfg->ctrl_name, sizeof(name) - 1);
	name[sizeof(name) - 1] = 0;
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
#define _C(x) (fbdev->ctrl->x)
#define _P(x) (fbdev->panel->x)
	BUG_ON(!(_C(init) && _C(cleanup) && _C(get_caps) &&
		 _C(get_mem_layout) && _C(set_update_mode) && _C(change_mode) &&
		 _P(init) && _P(cleanup) && _P(enable) && _P(disable) &&
		 _P(get_caps)));
#undef _P
#undef _C
}

static int omapfb_set_update_mode(struct omapfb_device *fbdev,
				  enum fb_update_mode mode)
{
	return fbdev->ctrl->set_update_mode(fbdev, mode);
}

static enum fb_update_mode omapfb_get_update_mode(struct omapfb_device *fbdev)
{
	return fbdev->ctrl->get_update_mode(fbdev);
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
static int omapfb_probe(struct device *dev)
{
	struct platform_device	*pdev;
	struct omapfb_device	*fbdev = NULL;
	struct fb_info		*fbi;
	int			init_state;
	int 			r = 0;

	DBGENTER(1);

	init_state = 0;

	pdev = to_platform_device(dev);
	if (pdev->num_resources != 0) {
		PRNERR("probed for an unknown device\n");
		r = -ENODEV;
		goto cleanup;
	}

	fbi = framebuffer_alloc(sizeof(struct omapfb_device), dev);
	if (!fbi) {
		PRNERR("unable to allocate memory for device info\n");
		r = -ENOMEM;
		goto cleanup;
	}

	fbdev = (struct omapfb_device *)fbi->par;
	fbdev->fb_info = fbi;
	fbdev->dev = dev;
	dev_set_drvdata(dev, fbdev);

	init_state++;
	if (omapfb_find_ctrl(fbdev) < 0) {
		PRNERR("LCD controller not found, board not supported\n");
		r = -ENODEV;
		goto cleanup;
	}		
	if (omapfb_find_panel(fbdev) < 0) {
		PRNERR("LCD panel not found, board not supported\n");
		r = -ENODEV;
		goto cleanup;
	}

	check_required_callbacks(fbdev);
	
	printk(KERN_INFO OMAPFB_DRIVER ": configured for panel %s\n",
	       fbdev->panel->name);

	omapfb_rqueue_init(&fbdev->rqueue);

	r = fbdev->panel->init(fbdev->panel);
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

	r = gfxdma_init(&fbdev->gfx);
	if (r)
		goto cleanup;
	init_state++;

#ifdef CONFIG_FB_OMAP_DMA_TUNE
	/* Set DMA priority for EMIFF access to highest */
	omap_set_dma_priority(OMAP_DMA_PORT_EMIFF, 15);
#endif

	r = fbdev->panel->enable(fbdev->panel);
	if (r)
		goto cleanup;
	init_state++;

	r = omapfb_set_update_mode(fbdev, manual_update ?
				   FB_MANUAL_UPDATE : FB_AUTO_UPDATE);
	if (r)
		goto cleanup;
	init_state++;

	r = omapfb_register_sysfs(fbdev);
	if (r)
		goto cleanup;
	init_state++;

	r = register_framebuffer(fbdev->fb_info);
	if (r != 0) {
		PRNERR("register_framebuffer failed\n");
		goto cleanup;
	}

	fbdev->state = OMAPFB_ACTIVE;

	printk(KERN_INFO "OMAP framebuffer initialized vram=%lu\n",
			 fbdev->lcddma_mem_size);

	DBGLEAVE(1);
	return 0;

cleanup:
	omapfb_free_resources(fbdev, init_state);

	DBGLEAVE(1);
	return r;
}

/* Called when the device is being detached from the driver */
static int omapfb_remove(struct device *dev)
{
	struct omapfb_device *fbdev = dev_get_drvdata(dev);
	enum omapfb_state saved_state = fbdev->state;

	DBGENTER(1);
	/* FIXME: wait till completion of pending events */

	fbdev->state = OMAPFB_DISABLED;
	omapfb_free_resources(fbdev, saved_state);

	DBGLEAVE(1);
	return 0;
}

/* PM suspend */
static int omapfb_suspend(struct device *dev, pm_message_t mesg, u32 level)
{
	struct omapfb_device *fbdev = dev_get_drvdata(dev);

	DBGENTER(1);

	if (fbdev->state == OMAPFB_ACTIVE) {
		if (fbdev->ctrl->suspend)
			fbdev->ctrl->suspend(fbdev);
		fbdev->panel->disable(fbdev->panel);
		fbdev->state = OMAPFB_SUSPENDED;
	}

	DBGLEAVE(1);
	return 0;
}

/* PM resume */
static int omapfb_resume(struct device *dev, u32 level)
{
	struct omapfb_device *fbdev = dev_get_drvdata(dev);

	DBGENTER(1);

	if (fbdev->state == OMAPFB_SUSPENDED) {
		fbdev->panel->enable(fbdev->panel);
		if (fbdev->ctrl->resume)
			fbdev->ctrl->resume(fbdev);
		fbdev->state = OMAPFB_ACTIVE;
		if (manual_update)
			omapfb_schedule_full_update(fbdev->fb_info);
	}

	DBGLEAVE(1);
	return 0;
}

static void omapfb_release_dev(struct device *dev)
{
	DBGENTER(1);
	DBGLEAVE(1);
}

static u64 omapfb_dmamask = ~(u32)0;

static struct platform_device omapfb_device = {
	.name		= OMAPFB_DEVICE,
	.id		= -1,
	.dev = {
		.release  = omapfb_release_dev,
		.dma_mask = &omapfb_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources = 0,
};

static struct device_driver omapfb_driver = {
	.name   	= OMAPFB_DRIVER,
	.bus		= &platform_bus_type,
	.probe          = omapfb_probe,
	.remove         = omapfb_remove,
	.suspend	= omapfb_suspend,
	.resume		= omapfb_resume
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
			case 'm':
			case 'M':
				def_vram *= 1024 * 1024;
				break;
			case 'k':
			case 'K':
				def_vram *= 1024;
				break;
			default:
				PRNERR("invalid vram suffix\n");
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
			PRNERR("invalid option\n");
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
	/* Register the device with LDM */
	if (platform_device_register(&omapfb_device)) {
		PRNERR("failed to register omapfb device\n");
		r = -ENODEV;
		goto exit;
	}
	/* Register the driver with LDM */
	if (driver_register(&omapfb_driver)) {
		PRNERR("failed to register omapfb driver\n");
		platform_device_unregister(&omapfb_device);
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

	driver_unregister(&omapfb_driver);
	platform_device_unregister(&omapfb_device);

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
