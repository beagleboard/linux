/*
 * sdma2edma.c
 *
 * SDMA to EDMA3 Wrapper.
 *
 * NOTE: Since we are invoking EDMA API, comments for all APIs in this file
 * are EDMA specific.
 *
 * Copyright (C) 2010-2011 Texas Instruments.
 * Author: Mansoor Ahamed <mansoor.ahamed@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <asm/system.h>
#include <mach/hardware.h>
#include <plat/dma.h>
#include <plat/tc.h>

/* some edma specific hacks which might change */
#include <mach/edma.h>

/**
 * omap_request_dma - allocate DMA channel and paired parameter RAM
 * @dev_id: specific channel to allocate; negative for "any unmapped channel"
 * @callback: optional; to be issued on DMA completion or errors
 * @data: passed to callback
 * @dma_ch_out: allocated channel number returned in this variable
 *
 * This allocates a DMA channel and its associated parameter RAM slot.
 * The parameter RAM is initialized to hold a dummy transfer.
 *
 * Normal use is to pass a specific channel number as @channel, to make
 * use of hardware events mapped to that channel.  When the channel will
 * be used only for software triggering or event chaining, channels not
 * mapped to hardware events (or mapped to unused events) are preferable.
 *
 * DMA transfers start from a channel using edma_start(), or by
 * chaining.  When the transfer described in that channel's parameter RAM
 * slot completes, that slot's data may be reloaded through a link.
 *
 * DMA errors are only reported to the @callback associated with the
 * channel driving that transfer, but transfer completion callbacks can
 * be sent to another channel under control of the TCC field in
 * the option word of the transfer's parameter RAM set.  Drivers must not
 * use DMA transfer completion callbacks for channels they did not allocate.
 * (The same applies to TCC codes used in transfer chaining.)
 *
 * TODO: -
 * . In the edma call, last param i.e TC hard coded to EVENTQ_2
 * . The callback's ch_status which should be used in McSPI driver
 *   to stop/clean EDMA is currently ignored in some driver (eg. McSPI)
 */
int omap_request_dma(int dev_id, const char *dev_name,
		void (*callback)(int lch, u16 ch_status, void *data),
		void *data, int *dma_ch_out)
{
	struct edmacc_param p_ram;
	typedef void (*EDMA_CALLBACK)(unsigned, u16, void*);
	EDMA_CALLBACK edma_callback = (EDMA_CALLBACK)(callback);

	*dma_ch_out = edma_alloc_channel(dev_id, edma_callback, data, EVENTQ_2);
	if (*dma_ch_out < 0)
		return -1;

	/* enable interrupts */
	edma_read_slot(*dma_ch_out, &p_ram);
	p_ram.opt |= TCINTEN | EDMA_TCC(EDMA_CHAN_SLOT(*dma_ch_out));
	edma_write_slot(*dma_ch_out, &p_ram);

	return 0;
}
EXPORT_SYMBOL(omap_request_dma);

/**
 * omap_free_dma - deallocate DMA channel
 * @lch: dma channel returned from edma_alloc_channel()
 *
 * This deallocates the DMA channel and associated parameter RAM slot
 * allocated by omap_request_dma().
 *
 * Callers are responsible for ensuring the channel is inactive, and
 * will not be reactivated by linking, chaining, or software calls to
 * omap_start_dma().
 */
void omap_free_dma(int lch)
{
	edma_free_channel((unsigned)lch);
}
EXPORT_SYMBOL(omap_free_dma);

/**
 * omap_start_dma - start dma on a channel
 * @lch: channel being activated
 *
 * Channels with event associations will be triggered by their hardware
 * events, and channels without such associations will be triggered by
 * software.  (At this writing there is no interface for using software
 * triggers except with channels that don't support hardware triggers.)
 *
 */
void omap_start_dma(int lch)
{
	edma_start((unsigned)lch);
}
EXPORT_SYMBOL(omap_start_dma);

/**
 * omap_stop_dma - stops dma on the channel passed
 * @lch: channel being deactivated
 *
 * When @lch is a channel, any active transfer is paused and
 * all pending hardware events are cleared.  The current transfer
 * may not be resumed, and the channel's Parameter RAM should be
 * reinitialized before being reused.
 */
void omap_stop_dma(int lch)
{
	edma_stop((unsigned)lch);
}
EXPORT_SYMBOL(omap_stop_dma);

/**
 * omap_cleanup_dma - Bring back DMA to initial state
 * @lch: channel being cleaned up
 *
 * It cleans ParamEntry and bring back EDMA to initial state if media has
 * been removed before EDMA has finished.It is usedful for removable media.
 *
 *
 * FIXME this should not be needed ... edma_stop() should suffice.
 *
 */
void omap_cleanup_dma(int lch)
{
	edma_clean_channel((unsigned)lch);
}
EXPORT_SYMBOL(omap_cleanup_dma);

/**
 * omap_set_dma_transfer_params - configure DMA transfer parameters
 * @lch: parameter RAM slot being configured
 * @data_type: how many bytes per array (at least one)
 * @elem_count: how many arrays per frame (at least one)
 * @frame_count: how many frames per block (at least one)
 * @sync_mode: ASYNC or ABSYNC
 * @dma_trigger: device id (not used)
 * @src_or_dst_synch: not used
 *
 * See the EDMA3 documentation to understand how to configure and link
 * transfers using the fields in PaRAM slots.  If you are not doing it
 * all at once with edma_write_slot(), you will use this routine
 * plus two calls each for source and destination, setting the initial
 * address and saying how to index that address.
 *
 * An example of an A-Synchronized transfer is a serial link using a
 * single word shift register.  In that case, @acnt would be equal to
 * that word size; the serial controller issues a DMA synchronization
 * event to transfer each word, and memory access by the DMA transfer
 * controller will be word-at-a-time.
 *
 * An example of an AB-Synchronized transfer is a device using a FIFO.
 * In that case, @acnt equals the FIFO width and @bcnt equals its depth.
 * The controller with the FIFO issues DMA synchronization events when
 * the FIFO threshold is reached, and the DMA transfer controller will
 * transfer one frame to (or from) the FIFO.  It will probably use
 * efficient burst modes to access memory.
 *
 * . dma_trigger and channel number, this is ignored for EDMA
 * . Setting bcnt_rld same as bcnt
 * TODO
 * . what is src_or_dst_synch?
 */
void omap_set_dma_transfer_params(int lch, int data_type, int elem_count,
			int frame_count, int sync_mode,
			int dma_trigger, int src_or_dst_synch)
{
	int d_type[3] = {1, 2, 4};
	if ((enum sync_dimension)sync_mode > ABSYNC) {
		printk(KERN_ERR "SDMA2EDMA: Line:%d : Param \'sync_mode\' out"
				" of range\n", __LINE__);
		return;
	}

	/* translate data_type */
	data_type = d_type[data_type];

	edma_set_transfer_params(lch, (u16)data_type, (u16)elem_count,
					(u16)frame_count, (u16)elem_count,
					(enum sync_dimension)sync_mode);
}
EXPORT_SYMBOL(omap_set_dma_transfer_params);

/**
 * omap_set_dma_dest_params - Set initial DMA destination addr in param RAM slot
 * @lch: parameter RAM slot being configured
 * @dest_port: not used
 * @dest_amode: INCR, except in very rare cases
 * @dest_start: physical address of destination (memory, controller FIFO, etc)
 * @dst_ei: byte offset between destination arrays in a frame
 * @dst_fi: byte offset between destination frames in a block
 *
 * Note that the destination address is modified during the DMA transfer
 * according to edma_set_dest_index().
 *
 * TODO
 * . Not sure about dst_ei and dst_fi
 * . fifo_width for edma is not available in sdma API hence setting it to
 *   W8BIT
 * . dest_port is ignored
 */
void omap_set_dma_dest_params(int lch, int dest_port, int dest_amode,
			unsigned long dest_start, int dst_ei, int dst_fi)
{
	if ((enum address_mode)dest_amode > FIFO) {
		printk(KERN_ERR "SDMA2EDMA: Line:%d : Param \'dest_amode\' out"
				" of range\n", __LINE__);
		return;
	}

	edma_set_dest((unsigned)lch, (dma_addr_t)dest_start,
					!dest_amode, W32BIT);
	edma_set_dest_index((unsigned)(lch), (s16)dst_ei, (s16)dst_fi);
}
EXPORT_SYMBOL(omap_set_dma_dest_params);

/**
 * omap_set_dma_src_params - Set initial DMA source addr in param RAM slot
 * @lch: parameter RAM slot being configured
 * @src_port: not used
 * @src_amode: INCR, except in very rare cases
 * @src_start: physical address of destination (memory, controller FIFO, etc)
 * @src_ei: byte offset between destination arrays in a frame
 * @src_fi: byte offset between destination frames in a block
 *
 * Note that the source address is modified during the DMA transfer
 * according to edma_set_src_index().
 *
 * TODO
 * . Not sure about src_ei and src_fi
 * . fifo_width for edma is not available in sdma API hence setting it to
 *   W8BIT
 * . src_port is ignored
 */
void omap_set_dma_src_params(int lch, int src_port, int src_amode,
			unsigned long src_start, int src_ei, int src_fi)
{
	if ((enum address_mode)src_amode > FIFO) {
		printk(KERN_ERR "SDMA2EDMA: Line:%d : Param \'src_amode\' out "
				"of range\n", __LINE__);
		return;
	}

	edma_set_src((unsigned)lch, (dma_addr_t)src_start,
					!src_amode, W32BIT);
	edma_set_src_index((unsigned)(lch), (s16)src_ei, (s16)src_fi);
}
EXPORT_SYMBOL(omap_set_dma_src_params);

void omap_set_dma_src_burst_mode(int lch, enum omap_dma_burst_mode burst_mode)
{
	printk(KERN_WARNING "omap_set_dma_src_burst_mode: un-supported SDMA"
				" wrapper\n");
}
EXPORT_SYMBOL(omap_set_dma_src_burst_mode);

void omap_set_dma_dest_burst_mode(int lch, enum omap_dma_burst_mode burst_mode)
{
	printk(KERN_WARNING "omap_set_dma_dest_burst_mode: un-supported SDMA"
				" wrapper\n");
}
EXPORT_SYMBOL(omap_set_dma_dest_burst_mode);

dma_addr_t omap_get_dma_dst_pos(int lch)
{
	printk(KERN_WARNING "omap_get_dma_dst_pos: un-supported in SDMA"
				" wrapper\n");
	return 0;
}
EXPORT_SYMBOL(omap_get_dma_dst_pos);

int omap_get_dma_active_status(int lch)
{
	printk(KERN_WARNING "omap_get_dma_active_status: un-supported in SDMA"
				" wrapper\n");
	return 0;
}
EXPORT_SYMBOL(omap_get_dma_active_status);

void omap_dma_global_context_save(void)
{
	printk(KERN_WARNING "omap_dma_global_context_save: un-supported in SDMA"
				" wrapper\n");
}
EXPORT_SYMBOL(omap_dma_global_context_save);

void omap_dma_global_context_restore(void)
{
	printk(KERN_WARNING "omap_dma_global_context_restore: un-supported in"
				"SDMA wrapper\n");
}
EXPORT_SYMBOL(omap_dma_global_context_restore);

int omap_dma_running(void)
{
	printk(KERN_WARNING "omap_dma_running: un-supported in SDMA wrapper\n");

	return 0;
}
EXPORT_SYMBOL(omap_dma_running);

void omap_set_dma_color_mode(int lch, enum omap_dma_color_mode mode, u32 color)
{
	printk(KERN_WARNING "omap_set_dma_color_mode: un-supported in"
				"SDMA wrapper\n");
}
EXPORT_SYMBOL(omap_set_dma_color_mode);

void omap_set_dma_dest_data_pack(int lch, int enable)
{
	printk(KERN_WARNING "omap_set_dma_dest_data_pack: un-supported in"
				"SDMA wrapper\n");
}
EXPORT_SYMBOL(omap_set_dma_dest_data_pack);

void omap_set_dma_src_data_pack(int lch, int enable)
{
	printk(KERN_WARNING "omap_set_dma_src_data_pack: un-supported in"
				"SDMA wrapper\n");
}
EXPORT_SYMBOL(omap_set_dma_src_data_pack);

void omap_set_dma_write_mode(int lch, enum omap_dma_write_mode mode)
{
	printk(KERN_WARNING "omap_set_dma_write_mode: un-supported in"
				"SDMA wrapper\n");
}
EXPORT_SYMBOL(omap_set_dma_write_mode);
