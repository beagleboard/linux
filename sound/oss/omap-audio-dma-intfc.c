/*
 * linux/sound/oss/omap-audio-dma-intfc.c
 *
 * Common audio DMA handling for the OMAP processors
 *
 * Copyright (C) 2004 Texas Instruments, Inc.
 *
 * Copyright (C) 2000, 2001 Nicolas Pitre <nico@cam.org>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * 2004-06-07	Sriram Kannan	- Created new file from omap_audio_dma_intfc.c. This file
 * 				  will contain only the DMA interface and buffer handling of OMAP
 * 				  audio driver.
 *
 * 2004-06-22	Sriram Kannan	- removed legacy code (auto-init). Self-linking of DMA logical channel.
 *
 * 2004-08-12   Nishanth Menon  - Modified to integrate Audio requirements on 1610,1710 platforms
 *
 * 2004-11-01   Nishanth Menon  - 16xx platform code base modified to support multi channel chaining.
 *
 * 2004-12-15   Nishanth Menon  - Improved 16xx platform channel logic introduced - tasklets, queue handling updated
 *
 * 2005-12-10   Dirk Behme      - Added L/R Channel Interchange fix as proposed by Ajaya Babu
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/sysrq.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/completion.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/semaphore.h>

#include <asm/arch/dma.h>
#include "omap-audio-dma-intfc.h"

#include <asm/arch/mcbsp.h>

#include "omap-audio.h"

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define DPRINTK(ARGS...)  printk(KERN_INFO "<%s>: ",__FUNCTION__);printk(ARGS)
#define FN_IN printk(KERN_INFO "[%s]: start\n", __FUNCTION__)
#define FN_OUT(n) printk(KERN_INFO "[%s]: end(%u)\n",__FUNCTION__, n)
#else

#define DPRINTK( x... )
#define FN_IN
#define FN_OUT(x)
#endif

#define ERR(ARGS...) printk(KERN_ERR "{%s}-ERROR: ", __FUNCTION__);printk(ARGS);

#define AUDIO_NAME		"omap-audio"
#define AUDIO_NBFRAGS_DEFAULT	8
#define AUDIO_FRAGSIZE_DEFAULT	8192

#define AUDIO_ACTIVE(state)	((state)->rd_ref || (state)->wr_ref)

#define SPIN_ADDR		(dma_addr_t)0
#define SPIN_SIZE		2048

/* Channel Queue Handling macros
 * tail always points to the current free entry
 * Head always points to the current entry being used
 * end is either head or tail
 */

#define AUDIO_QUEUE_INIT(s) s->dma_q_head = s->dma_q_tail = s->dma_q_count = 0;
#define AUDIO_QUEUE_FULL(s) (nr_linked_channels == s->dma_q_count)
#define AUDIO_QUEUE_LAST(s) (1 == s->dma_q_count)
#define AUDIO_QUEUE_EMPTY(s) (0 == s->dma_q_count)
#define __AUDIO_INCREMENT_QUEUE(end) ((end)=((end)+1) % nr_linked_channels)
#define AUDIO_INCREMENT_HEAD(s) __AUDIO_INCREMENT_QUEUE(s->dma_q_head); s->dma_q_count--;
#define AUDIO_INCREMENT_TAIL(s) __AUDIO_INCREMENT_QUEUE(s->dma_q_tail); s->dma_q_count++;

/* DMA buffer fragmentation sizes */
#define MAX_DMA_SIZE		 0x1000000
#define CUT_DMA_SIZE		 0x1000
/* TODO: To be moved to more appropriate location */
#define DCSR_ERROR           0x3
#define DCSR_SYNC_SET        (1 << 6)

#define DCCR_FS              (1 << 5)
#define DCCR_PRIO            (1 << 6)
#define DCCR_EN              (1 << 7)
#define DCCR_AI              (1 << 8)
#define DCCR_REPEAT          (1 << 9)
/* if 0 the channel works in 3.1 compatible mode*/
#define DCCR_N31COMP         (1 << 10)
#define DCCR_EP              (1 << 11)
#define DCCR_SRC_AMODE_BIT   12
#define DCCR_SRC_AMODE_MASK  (0x3<<12)
#define DCCR_DST_AMODE_BIT   14
#define DCCR_DST_AMODE_MASK  (0x3<<14)
#define AMODE_CONST          0x0
#define AMODE_POST_INC       0x1
#define AMODE_SINGLE_INDEX   0x2
#define AMODE_DOUBLE_INDEX   0x3

/**************************** DATA STRUCTURES *****************************************/

static spinlock_t dma_list_lock = SPIN_LOCK_UNLOCKED;

struct audio_isr_work_item {
	int current_lch;
	u16 ch_status;
	audio_stream_t *s;
};

static char work_item_running = 0;
static char nr_linked_channels = 1;
static struct audio_isr_work_item work1, work2;


/*********************************** MODULE SPECIFIC FUNCTIONS PROTOTYPES *************/

static void audio_dsr_handler(unsigned long);
static DECLARE_TASKLET(audio_isr_work1, audio_dsr_handler,
		(unsigned long)&work1);
static DECLARE_TASKLET(audio_isr_work2, audio_dsr_handler,
		(unsigned long)&work2);

static void sound_dma_irq_handler(int lch, u16 ch_status, void *data);
static void audio_dma_callback(int lch, u16 ch_status, void *data);
static int omap_start_sound_dma(audio_stream_t * s, dma_addr_t dma_ptr,
				u_int size);
static int audio_set_dma_params_play(int channel, dma_addr_t dma_ptr,
				     u_int dma_size);
static int audio_set_dma_params_capture(int channel, dma_addr_t dma_ptr,
					u_int dma_size);
static int audio_start_dma_chain(audio_stream_t * s);

/*********************************** GLOBAL FUNCTIONS DEFINTIONS ***********************/

/***************************************************************************************
 *
 * Buffer creation/destruction
 *
 **************************************************************************************/
int audio_setup_buf(audio_stream_t * s)
{
	int frag;
	int dmasize = 0;
	char *dmabuf = NULL;
	dma_addr_t dmaphys = 0;
	FN_IN;
	if (s->buffers) {
		FN_OUT(1);
		return -EBUSY;
	}
	s->buffers = kmalloc(sizeof(audio_buf_t) * s->nbfrags, GFP_KERNEL);
	if (!s->buffers)
		goto err;
	memset(s->buffers, 0, sizeof(audio_buf_t) * s->nbfrags);
	for (frag = 0; frag < s->nbfrags; frag++) {
		audio_buf_t *b = &s->buffers[frag];
		/*
		 * Let's allocate non-cached memory for DMA buffers.
		 * We try to allocate all memory at once.
		 * If this fails (a common reason is memory fragmentation),
		 * then we allocate more smaller buffers.
		 */
		if (!dmasize) {
			dmasize = (s->nbfrags - frag) * s->fragsize;
			do {
				dmabuf =
				    dma_alloc_coherent(NULL, dmasize, &dmaphys,
						       0);
				if (!dmabuf)
					dmasize -= s->fragsize;
			}
			while (!dmabuf && dmasize);
			if (!dmabuf)
				goto err;
			b->master = dmasize;
			memzero(dmabuf, dmasize);
		}
		b->data = dmabuf;
		b->dma_addr = dmaphys;
		dmabuf += s->fragsize;
		dmaphys += s->fragsize;
		dmasize -= s->fragsize;
	}
	s->usr_head = s->dma_head = s->dma_tail = 0;
	AUDIO_QUEUE_INIT(s);
	s->started = 0;
	s->bytecount = 0;
	s->fragcount = 0;
	init_completion(&s->wfc);
	s->wfc.done = s->nbfrags;
	FN_OUT(0);
	return 0;
      err:
	audio_discard_buf(s);
	FN_OUT(1);
	return -ENOMEM;
}

void audio_discard_buf(audio_stream_t * s)
{
	FN_IN;
	/* ensure DMA isn't using those buffers */
	audio_reset(s);
	if (s->buffers) {
		int frag;
		for (frag = 0; frag < s->nbfrags; frag++) {
			if (!s->buffers[frag].master)
				continue;
			dma_free_coherent(NULL,
					  s->buffers[frag].master,
					  s->buffers[frag].data,
					  s->buffers[frag].dma_addr);
		}
		kfree(s->buffers);
		s->buffers = NULL;
	}
	FN_OUT(0);
}

/***************************************************************************************
 *
 * DMA channel requests
 *
 **************************************************************************************/
static void omap_sound_dma_link_lch(void *data)
{
	audio_stream_t *s = (audio_stream_t *) data;
	int *chan = s->lch;
	int i;

	FN_IN;
	if (s->linked) {
		FN_OUT(1);
		return;
	}
	for (i = 0; i < nr_linked_channels; i++) {
		int cur_chan = chan[i];
		int nex_chan =
		    ((nr_linked_channels - 1 ==
		      i) ? chan[0] : chan[i + 1]);
		omap_dma_link_lch(cur_chan, nex_chan);
	}
	s->linked = 1;
	FN_OUT(0);
}

int
omap_request_sound_dma(int device_id, const char *device_name, void *data,
		       int **channels)
{
	int i, err = 0;
	int *chan = NULL;
	FN_IN;
	if (unlikely((NULL == channels) || (NULL == device_name))) {
		BUG();
		return -EPERM;
	}
	/* Try allocate memory for the num channels */
	*channels =
	    (int *)kmalloc(sizeof(int) * nr_linked_channels,
			   GFP_KERNEL);
	chan = *channels;
	if (NULL == chan) {
		ERR("No Memory for channel allocs!\n");
		FN_OUT(-ENOMEM);
		return -ENOMEM;
	}
	spin_lock(&dma_list_lock);
	for (i = 0; i < nr_linked_channels; i++) {
		err =
		    omap_request_dma(device_id, device_name,
				     sound_dma_irq_handler, data, &chan[i]);
		/* Handle Failure condition here */
		if (err < 0) {
			int j;
			for (j = 0; j < i; j++) {
				omap_free_dma(chan[j]);
			}
			spin_unlock(&dma_list_lock);
			kfree(chan);
			*channels = NULL;
			ERR("Error in requesting channel %d=0x%x\n", i, err);
			FN_OUT(err);
			return err;
		}
	}

	/* Chain the channels together */
	if (!cpu_is_omap1510())
		omap_sound_dma_link_lch(data);

	spin_unlock(&dma_list_lock);
	FN_OUT(0);
	return 0;
}

/***************************************************************************************
 *
 * DMA channel requests Freeing
 *
 **************************************************************************************/
static void omap_sound_dma_unlink_lch(void *data)
{
	audio_stream_t *s = (audio_stream_t *) data;
	int *chan = s->lch;
	int i;

	FN_IN;
	if (!s->linked) {
		FN_OUT(1);
		return;
	}
	for (i = 0; i < nr_linked_channels; i++) {
		int cur_chan = chan[i];
		int nex_chan =
		    ((nr_linked_channels - 1 ==
		      i) ? chan[0] : chan[i + 1]);
		omap_dma_unlink_lch(cur_chan, nex_chan);
	}
	s->linked = 0;
	FN_OUT(0);
}

int omap_free_sound_dma(void *data, int **channels)
{
	int i;
	int *chan = NULL;
	FN_IN;
	if (unlikely(NULL == channels)) {
		BUG();
		return -EPERM;
	}
	if (unlikely(NULL == *channels)) {
		BUG();
		return -EPERM;
	}
	chan = (*channels);

	if (!cpu_is_omap1510())
		omap_sound_dma_unlink_lch(data);
	for (i = 0; i < nr_linked_channels; i++) {
		int cur_chan = chan[i];
		omap_stop_dma(cur_chan);
		omap_free_dma(cur_chan);
	}
	kfree(*channels);
	*channels = NULL;
	FN_OUT(0);
	return 0;
}

/***************************************************************************************
 *
 * Process DMA requests - This will end up starting the transfer. Proper fragments of
 * Transfers will be initiated.
 *
 **************************************************************************************/
int audio_process_dma(audio_stream_t * s)
{
	int ret = 0;
	unsigned long flags;
	FN_IN;

	/* Dont let the ISR over ride touching the in_use flag */
	local_irq_save(flags);
	if (1 == s->in_use) {
		local_irq_restore(flags);
		ERR("Called again while In Use\n");
		return 0;
	}
	s->in_use = 1;
	local_irq_restore(flags);

	if (s->stopped)
		goto spin;

	if (s->dma_spinref > 0 && s->pending_frags) {
		s->dma_spinref = 0;
		DMA_CLEAR(s);
	}
	while (s->pending_frags) {
		audio_buf_t *b = &s->buffers[s->dma_head];
		u_int dma_size = s->fragsize - b->offset;
		if (dma_size > MAX_DMA_SIZE)
			dma_size = CUT_DMA_SIZE;
		ret =
		    omap_start_sound_dma(s, b->dma_addr + b->offset, dma_size);
		if (ret) {
			goto process_out;
		}
		b->dma_ref++;
		b->offset += dma_size;
		if (b->offset >= s->fragsize) {
			s->pending_frags--;
			if (++s->dma_head >= s->nbfrags)
				s->dma_head = 0;
		}
	}
      spin:
	if (s->spin_idle) {
		int spincnt = 0;
		ERR("we are spinning\n");
		while (omap_start_sound_dma(s, SPIN_ADDR, SPIN_SIZE) == 0)
			spincnt++;
		/*
		 * Note: if there is still a data buffer being
		 * processed then the ref count is negative.  This
		 * allows for the DMA termination to be accounted in
		 * the proper order.  Of course dma_spinref can't be
		 * greater than 0 if dma_ref is not 0 since we kill
		 * the spinning above as soon as there is real data to process.
		 */
		if (s->buffers && s->buffers[s->dma_tail].dma_ref)
			spincnt = -spincnt;
		s->dma_spinref += spincnt;
	}

      process_out:
	s->in_use = 0;

	FN_OUT(ret);
	return ret;
}

/***************************************************************************************
 *
 * Prime Rx - Since the recieve buffer has no time limit as to when it would arrive,
 *            we need to prime it
 *            
 **************************************************************************************/
void audio_prime_rx(audio_state_t * state)
{
	audio_stream_t *is = state->input_stream;

	FN_IN;
	if (state->need_tx_for_rx) {
		/*
		 * With some codecs like the Philips UDA1341 we must ensure
		 * there is an output stream at any time while recording since
		 * this is how the UDA1341 gets its clock from the SA1100.
		 * So while there is no playback data to send, the output DMA
		 * will spin with all zeroes.  We use the cache flush special
		 * area for that.
		 */
		state->output_stream->spin_idle = 1;
		audio_process_dma(state->output_stream);
	}
	is->pending_frags = is->nbfrags;
	init_completion(&is->wfc);
	is->wfc.done = 0;

	is->active = 1;
	audio_process_dma(is);

	FN_OUT(0);
	return;
}

/***************************************************************************************
 *
 * set the fragment size
 *
 **************************************************************************************/
int audio_set_fragments(audio_stream_t * s, int val)
{
	FN_IN;
	if (s->active)
		return -EBUSY;
	if (s->buffers)
		audio_discard_buf(s);
	s->nbfrags = (val >> 16) & 0x7FFF;
	val &= 0xFFFF;
	if (val < 4)
		val = 4;
	if (val > 15)
		val = 15;
	s->fragsize = 1 << val;
	if (s->nbfrags < 2)
		s->nbfrags = 2;
	if (s->nbfrags * s->fragsize > 128 * 1024)
		s->nbfrags = 128 * 1024 / s->fragsize;
	FN_OUT(0);
	if (audio_setup_buf(s))
		return -ENOMEM;
	return val | (s->nbfrags << 16);

}

/***************************************************************************************
 *
 * Sync up the buffers before we shutdown, else under-run errors will happen
 *
 **************************************************************************************/
int audio_sync(struct file *file)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *s = state->output_stream;
	audio_buf_t *b;
	u_int shiftval = 0;
	unsigned long flags;

	DECLARE_WAITQUEUE(wait, current);

	FN_IN;

	if (!(file->f_mode & FMODE_WRITE) || !s->buffers || s->mapped) {
		FN_OUT(1);
		return 0;
	}

	/*
	 * Send current buffer if it contains data.  Be sure to send
	 * a full sample count.
	 */
	b = &s->buffers[s->usr_head];
	if (b->offset &= ~3) {
		/* Wait for a buffer to become free */
		if (wait_for_completion_interruptible(&s->wfc))
			return 0;
		/*
		 * HACK ALERT !
		 * To avoid increased complexity in the rest of the code
		 * where full fragment sizes are assumed, we cheat a little
		 * with the start pointer here and don't forget to restore
		 * it later.
		 */
		
		/* As this is a last frag we need only one dma channel
		 * to complete. So it's need to unlink dma channels
		 * to avoid empty dma work.
		 */
		if (!cpu_is_omap1510() && AUDIO_QUEUE_EMPTY(s))
			omap_sound_dma_unlink_lch(s);

		shiftval = s->fragsize - b->offset;
		b->offset = shiftval;
		b->dma_addr -= shiftval;
		b->data -= shiftval;
		local_irq_save(flags);
		s->bytecount -= shiftval;
		if (++s->usr_head >= s->nbfrags)
			s->usr_head = 0;

		s->pending_frags++;
		audio_process_dma(s);
		local_irq_restore(flags);
	}

	/* Let's wait for all buffers to complete */
	set_current_state(TASK_INTERRUPTIBLE);
	add_wait_queue(&s->wq, &wait);
	while ((s->pending_frags || (s->wfc.done < s->nbfrags))
	       && !signal_pending(current)) {
		schedule();
		set_current_state(TASK_INTERRUPTIBLE);
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&s->wq, &wait);

	/* undo the pointer hack above */
	if (shiftval) {
		local_irq_save(flags);
		b->dma_addr += shiftval;
		b->data += shiftval;
		/* ensure sane DMA code behavior if not yet processed */
		if (b->offset != 0)
			b->offset = s->fragsize;
		local_irq_restore(flags);
	}

	FN_OUT(0);
	return 0;
}

/***************************************************************************************
 *
 * Stop all the DMA channels of the stream
 *
 **************************************************************************************/
void audio_stop_dma(audio_stream_t * s)
{
	int *chan = s->lch;
	int i;
	FN_IN;
	if (unlikely(NULL == chan)) {
		BUG();
		return;
	}
	for (i = 0; i < nr_linked_channels; i++) {
		int cur_chan = chan[i];
		omap_stop_dma(cur_chan);
	}
	s->started = 0;
	FN_OUT(0);
	return;
}

/***************************************************************************************
 *
 * Get the dma posn
 *
 **************************************************************************************/
u_int audio_get_dma_pos(audio_stream_t * s)
{
	audio_buf_t *b = &s->buffers[s->dma_tail];
	u_int offset;

	FN_IN;
	if (b->dma_ref) {
		offset = omap_get_dma_src_pos(s->lch[s->dma_q_head]) - b->dma_addr;
		if (offset >= s->fragsize)
			offset = s->fragsize - 4;
	} else if (s->pending_frags) {
		offset = b->offset;
	} else {
		offset = 0;
	}
	FN_OUT(offset);
	return offset;
}

/***************************************************************************************
 *
 * Reset the audio buffers
 *
 **************************************************************************************/
void audio_reset(audio_stream_t * s)
{
	FN_IN;
	if (s->buffers) {
		audio_stop_dma(s);
		s->buffers[s->dma_head].offset = 0;
		s->buffers[s->usr_head].offset = 0;
		s->usr_head = s->dma_head;
		s->pending_frags = 0;
		init_completion(&s->wfc);
		s->wfc.done = s->nbfrags;
	}
	s->active = 0;
	s->stopped = 0;
	s->started = 0;
	FN_OUT(0);
	return;
}

/***************************************************************************************
 *
 * Clear any pending transfers
 *
 **************************************************************************************/
void omap_clear_sound_dma(audio_stream_t * s)
{
	FN_IN;
	omap_clear_dma(s->lch[s->dma_q_head]);
	FN_OUT(0);
	return;
}

/***************************************************************************************
 *
 * DMA related functions
 *
 **************************************************************************************/
static int audio_set_dma_params_play(int channel, dma_addr_t dma_ptr,
				     u_int dma_size)
{
	int dt = 0x1;		/* data type 16 */
	int cen = 32;		/* Stereo */
	int cfn = dma_size / (2 * cen);
	unsigned long dest_start;
	int dest_port = 0;
	int sync_dev = 0;

	FN_IN;

	if (cpu_is_omap1510() || cpu_is_omap16xx()) {
		dest_start = AUDIO_MCBSP_DATAWRITE;
		dest_port = OMAP_DMA_PORT_MPUI;
	}
	if (cpu_is_omap24xx()) {
		dest_start = AUDIO_MCBSP_DATAWRITE;
		sync_dev = AUDIO_DMA_TX;
	}

	omap_set_dma_dest_params(channel, dest_port, OMAP_DMA_AMODE_CONSTANT, dest_start, 0, 0);
	omap_set_dma_src_params(channel, 0, OMAP_DMA_AMODE_POST_INC, dma_ptr, 0, 0);
	omap_set_dma_transfer_params(channel, dt, cen, cfn, OMAP_DMA_SYNC_ELEMENT, sync_dev, 0);

	FN_OUT(0);
	return 0;
}

static int audio_set_dma_params_capture(int channel, dma_addr_t dma_ptr,
					u_int dma_size)
{
	int dt = 0x1;		/* data type 16 */
	int cen = 16;		/* mono */
	int cfn = dma_size / (2 * cen);
	unsigned long src_start;
	int src_port = 0;
	int sync_dev = 0;
	int src_sync = 0;

	FN_IN;

	if (cpu_is_omap1510() || cpu_is_omap16xx()) {
		src_start = AUDIO_MCBSP_DATAREAD;
		src_port = OMAP_DMA_PORT_MPUI;
	}
	if (cpu_is_omap24xx()) {
		src_start = AUDIO_MCBSP_DATAREAD;
		sync_dev = AUDIO_DMA_RX;
		src_sync = 1;
	}

	omap_set_dma_src_params(channel, src_port, OMAP_DMA_AMODE_CONSTANT, src_start, 0, 0);
	omap_set_dma_dest_params(channel, 0, OMAP_DMA_AMODE_POST_INC, dma_ptr, 0, 0);
	omap_set_dma_transfer_params(channel, dt, cen, cfn, OMAP_DMA_SYNC_ELEMENT, sync_dev, src_sync);

	FN_OUT(0);
	return 0;
}

static int audio_start_dma_chain(audio_stream_t * s)
{
	int channel = s->lch[s->dma_q_head];
	FN_IN;
	if (!s->started) {
		s->hw_stop();		/* stops McBSP Interface */
		omap_start_dma(channel);
		s->started = 1;
		s->hw_start();		/* start McBSP interface */
	}
	/* else the dma itself will progress forward with out our help */
	FN_OUT(0);
	return 0;
}

/* Start DMA -
 * Do the initial set of work to initialize all the channels as required.
 * We shall then initate a transfer
 */
static int omap_start_sound_dma(audio_stream_t * s, dma_addr_t dma_ptr,
				u_int dma_size)
{
	int ret = -EPERM;

	FN_IN;
	if (unlikely(dma_size > MAX_DMA_SIZE)) {
		ERR("DmaSoundDma: Start: overflowed %d-%d\n", dma_size,
		    MAX_DMA_SIZE);
		return -EOVERFLOW;
	}

	if (AUDIO_QUEUE_FULL(s)) {
		ret = -2;
		goto sound_out;
	}
	
	if (s->input_or_output == FMODE_WRITE)
		/*playback */
	{
		ret =
		    audio_set_dma_params_play(s->lch[s->dma_q_tail], dma_ptr,
					      dma_size);
	} else {
		ret =
		    audio_set_dma_params_capture(s->lch[s->dma_q_tail], dma_ptr,
						 dma_size);
	}
	if (ret != 0) {
		ret = -2;	/* indicate queue full */
		goto sound_out;
	}
	AUDIO_INCREMENT_TAIL(s);
	ret = audio_start_dma_chain(s);
	if (ret) {
		ERR("dma start failed");
	}
      sound_out:
	FN_OUT(ret);
	return ret;

}

/***************************************************************************************
 *
 * ISR related functions
 *
 **************************************************************************************/
/* The work item handler */
static void audio_dsr_handler(unsigned long inData)
{
	void *data = (void *)inData;
	struct audio_isr_work_item *work = data;
	audio_stream_t *s = (work->s);
	int sound_curr_lch = work->current_lch;
	u16 ch_status = work->ch_status;

	FN_IN;
	DPRINTK("lch=%d,status=0x%x, data=%p as=%p\n", sound_curr_lch,
		ch_status, data, s);
	if (AUDIO_QUEUE_EMPTY(s)) {
		ERR("Interrupt(%d)  for empty queue(h=%d, T=%d)???\n",
		    sound_curr_lch, s->dma_q_head, s->dma_q_tail);
		ERR("nbfrag=%d,pendfrags=%d,USR-H=%d, QH-%d QT-%d\n",
		    s->nbfrags, s->pending_frags, s->usr_head, s->dma_head,
		    s->dma_tail);
		FN_OUT(-1);
		return;
	}

	AUDIO_INCREMENT_HEAD(s);	/* Empty the queue */

	/* Try to fill again */
	audio_dma_callback(sound_curr_lch, ch_status, s);
	FN_OUT(0);

}

/* Macro to trace the IRQ calls - checks for multi-channel irqs */
//#define IRQ_TRACE
#ifdef IRQ_TRACE
#define MAX_UP 10
static char xyz[MAX_UP] = { 0 };
static int h = 0;
#endif

/* ISRs have to be short and smart.. So we transfer every heavy duty stuff to the 
 * work item
 */
static void sound_dma_irq_handler(int sound_curr_lch, u16 ch_status, void *data)
{
	int dma_status = ch_status;
	audio_stream_t *s = (audio_stream_t *) data;
	FN_IN;
#ifdef IRQ_TRACE
	xyz[h++] = '0' + sound_curr_lch;
	if (h == MAX_UP - 1) {
		printk("%s-", xyz);
		h = 0;
	}
#endif
	DPRINTK("lch=%d,status=0x%x, dma_status=%d, data=%p\n", sound_curr_lch,
		ch_status, dma_status, data);

	if (dma_status & (DCSR_ERROR)) {
		if (cpu_is_omap1510() || cpu_is_omap16xx())
			OMAP_DMA_CCR_REG(sound_curr_lch) &= ~DCCR_EN;
		ERR("DCSR_ERROR!\n");
		FN_OUT(-1);
		return;
	}

	if (AUDIO_QUEUE_LAST(s))
		audio_stop_dma(s);

	/* Start the work item  - we ping pong the work items */
	if (!work_item_running) {
		work1.current_lch = sound_curr_lch;
		work1.ch_status = ch_status;
		work1.s = s;
		/* schedule tasklet 1 */
		tasklet_schedule(&audio_isr_work1);
		work_item_running = 1;
	} else {
		work2.current_lch = sound_curr_lch;
		work2.ch_status = ch_status;
		work2.s = s;
		/* schedule tasklet 2 */
		tasklet_schedule(&audio_isr_work2);
		work_item_running = 0;
	}
	FN_OUT(0);
	return;
}

/* The call back that handles buffer stuff */
static void audio_dma_callback(int lch, u16 ch_status, void *data)
{
	audio_stream_t *s = data;
	audio_buf_t *b = &s->buffers[s->dma_tail];
	FN_IN;

	if (s->dma_spinref > 0) {
		s->dma_spinref--;
	} else if (!s->buffers) {
		printk(KERN_CRIT
		       "omap_audio: received DMA IRQ for non existent buffers!\n");
		return;
	} else if (b->dma_ref && --b->dma_ref == 0 && b->offset >= s->fragsize) {
		/* This fragment is done */
		b->offset = 0;
		s->bytecount += s->fragsize;
		s->fragcount++;
		s->dma_spinref = -s->dma_spinref;

		if (++s->dma_tail >= s->nbfrags)
			s->dma_tail = 0;

		if (!s->mapped)
			complete(&s->wfc);
		else
			s->pending_frags++;

		wake_up(&s->wq);
	}

	audio_process_dma(s);
	
	FN_OUT(0);
	return;
}

/*********************************************************************************
 *
 * audio_get_dma_callback(): return the dma interface call back function
 *
 *********************************************************************************/
dma_callback_t audio_get_dma_callback(void)
{
	FN_IN;
	FN_OUT(0);
	return audio_dma_callback;
}

static int __init audio_dma_init(void)
{
	if (!cpu_is_omap1510())
		nr_linked_channels = 2;

	return 0;
}

static void __exit audio_dma_exit(void)
{
	/* Nothing */
}

module_init(audio_dma_init);
module_exit(audio_dma_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Common DMA handling for Audio driver on OMAP processors");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(omap_clear_sound_dma);
EXPORT_SYMBOL(omap_request_sound_dma);
EXPORT_SYMBOL(omap_free_sound_dma);

EXPORT_SYMBOL(audio_get_dma_callback);
EXPORT_SYMBOL(audio_setup_buf);
EXPORT_SYMBOL(audio_process_dma);
EXPORT_SYMBOL(audio_prime_rx);
EXPORT_SYMBOL(audio_set_fragments);
EXPORT_SYMBOL(audio_sync);
EXPORT_SYMBOL(audio_stop_dma);
EXPORT_SYMBOL(audio_get_dma_pos);
EXPORT_SYMBOL(audio_reset);
EXPORT_SYMBOL(audio_discard_buf);
