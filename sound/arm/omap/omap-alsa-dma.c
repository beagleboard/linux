/*
 * sound/arm/omap/omap-alsa-dma.c
 *
 * Common audio DMA handling for the OMAP processors
 *
 * Copyright (C) 2006 Mika Laitio <lamikr@cc.jyu.fi>
 *
 * Copyright (C) 2005 Instituto Nokia de Tecnologia - INdT - Manaus Brazil
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
 * 2005-07-19	INdT Kernel Team - Alsa port. Creation of new file omap-alsa-dma.c based in
 * 				   omap-audio-dma-intfc.c oss file. Support for aic23 codec.
 * 				   Removal of buffer handling (Alsa does that), modifications
 *	in dma handling and port to alsa structures.
 *
 * 2005-12-18   Dirk Behme      - Added L/R Channel Interchange fix as proposed by Ajaya Babu
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

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/semaphore.h>

#include <asm/arch/dma.h>
#include "omap-alsa-dma.h"

#include <asm/arch/mcbsp.h>

#include <asm/arch/omap-alsa.h>

#undef DEBUG

#define ERR(ARGS...) printk(KERN_ERR "{%s}-ERROR: ", __FUNCTION__);printk(ARGS);

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
#define MAX_DMA_SIZE		 0x1000000 /* todo: sync with alsa */
//#define CUT_DMA_SIZE		 0x1000
/* TODO: To be moved to more appropriate location */
#define DCSR_ERROR           0x3
#define DCSR_END_BLOCK       (1 << 5)
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

static char nr_linked_channels = 1;

/*********************************** MODULE SPECIFIC FUNCTIONS ***********************/

static void sound_dma_irq_handler(int lch, u16 ch_status, void *data);
static int audio_set_dma_params_play(int channel, dma_addr_t dma_ptr,
				     u_int dma_size);
static int audio_set_dma_params_capture(int channel, dma_addr_t dma_ptr,
					u_int dma_size);
static int audio_start_dma_chain(struct audio_stream * s);

/***************************************************************************************
 *
 * DMA channel requests
 *
 **************************************************************************************/
static void omap_sound_dma_link_lch(void *data)
{

	struct audio_stream *s = (struct audio_stream *) data;
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

int omap_request_alsa_sound_dma(int device_id, const char *device_name,
			   void *data, int **channels)
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
	    (int *) kmalloc(sizeof(int) * nr_linked_channels, GFP_KERNEL);
	chan = *channels;
	if (NULL == chan) {
		ERR("No Memory for channel allocs!\n");
		FN_OUT(-ENOMEM);
		return -ENOMEM;
	}
	spin_lock(&dma_list_lock);
	for (i = 0; i < nr_linked_channels; i++) {
		err = omap_request_dma(device_id, 
				device_name,
				sound_dma_irq_handler, 
				data,
				&chan[i]);

		/* Handle Failure condition here */
		if (err < 0) {
			int j;
			for (j = 0; j < i; j++) {
				omap_free_dma(chan[j]);
			}
			spin_unlock(&dma_list_lock);
			kfree(chan);
			*channels = NULL;
			ERR("Error in requesting channel %d=0x%x\n", i,
			    err);
			FN_OUT(err);
			return err;
		}
	}

	/* Chain the channels together */
	if (!cpu_is_omap15xx())
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
	struct audio_stream *s = (struct audio_stream *)data;
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

int omap_free_alsa_sound_dma(void *data, int **channels)
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

	if (!cpu_is_omap15xx())
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
 * Stop all the DMA channels of the stream
 *
 **************************************************************************************/
void omap_stop_alsa_sound_dma(struct audio_stream *s)
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
 * Clear any pending transfers
 *
 **************************************************************************************/
void omap_clear_alsa_sound_dma(struct audio_stream * s)
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
	
	FN_IN;
	omap_set_dma_dest_params(channel, 0x05, 0x00,
				 (OMAP1510_MCBSP1_BASE + 0x06),
				 0, 0);
	omap_set_dma_src_params(channel, 0x00, 0x01, dma_ptr,
				0, 0);
	omap_set_dma_transfer_params(channel, dt, cen, cfn, 0x00, 0, 0);
	FN_OUT(0);
	return 0;
}

static int audio_set_dma_params_capture(int channel, dma_addr_t dma_ptr,
					u_int dma_size)
{
	int dt = 0x1;		/* data type 16 */
	int cen = 32;		/* stereo */
	int cfn = dma_size / (2 * cen);
	
	FN_IN;
	omap_set_dma_src_params(channel, 0x05, 0x00,
				(OMAP1510_MCBSP1_BASE + 0x02),
				0, 0);
	omap_set_dma_dest_params(channel, 0x00, 0x01, dma_ptr, 0, 0);
	omap_set_dma_transfer_params(channel, dt, cen, cfn, 0x00, 0, 0);
	FN_OUT(0);
	return 0;
}

static int audio_start_dma_chain(struct audio_stream *s)
{
	int channel = s->lch[s->dma_q_head];
	FN_IN;
	if (!s->started) {
	 	s->hw_stop();	   /* stops McBSP Interface */
		omap_start_dma(channel);
		s->started = 1;
		s->hw_start();	   /* start McBSP interface */
	} else if (cpu_is_omap310())
		omap_start_dma(channel);
	/* else the dma itself will progress forward with out our help */
	FN_OUT(0);
	return 0;
}

/* Start DMA -
 * Do the initial set of work to initialize all the channels as required.
 * We shall then initate a transfer
 */
int omap_start_alsa_sound_dma(struct audio_stream *s, 
			dma_addr_t dma_ptr, 
			u_int dma_size)
{
	int ret = -EPERM;

	FN_IN;

	if (unlikely(dma_size > MAX_DMA_SIZE)) {
		ERR("DmaSoundDma: Start: overflowed %d-%d\n", dma_size,
		    MAX_DMA_SIZE);
		return -EOVERFLOW;
	}
	//if (AUDIO_QUEUE_FULL(s)) {
	//      ret = -2;
	//      goto sound_out;
	//}

	if (s->stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
		/*playback */
		ret =
		    audio_set_dma_params_play(s->lch[s->dma_q_tail],
					      dma_ptr, dma_size);
	} else {
		ret =
		    audio_set_dma_params_capture(s->lch[s->dma_q_tail],
						 dma_ptr, dma_size);
	}
	if (ret != 0) {
		ret = -3;	/* indicate queue full */
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

/* 
 * ISRs have to be short and smart.. 
 * Here we call alsa handling, after some error checking
 */
static void sound_dma_irq_handler(int sound_curr_lch, u16 ch_status,
				  void *data)
{
	int dma_status = ch_status;
	struct audio_stream *s = (struct audio_stream *) data;
	FN_IN;

	/*
	 * some register checkings
	 */ 
	DPRINTK("lch=%d,status=0x%x, dma_status=%d, data=%p\n",
		sound_curr_lch, ch_status, dma_status, data);

	if (dma_status & (DCSR_ERROR)) {
		OMAP_DMA_CCR_REG(sound_curr_lch) &= ~DCCR_EN;
		ERR("DCSR_ERROR!\n");
		FN_OUT(-1);
		return;
	}

	if (ch_status & DCSR_END_BLOCK) 
		callback_omap_alsa_sound_dma(s);
	FN_OUT(0);
	return;
}

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Common DMA handling for Audio driver on OMAP processors");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(omap_start_alsa_sound_dma);
EXPORT_SYMBOL(omap_clear_alsa_sound_dma);
EXPORT_SYMBOL(omap_request_alsa_sound_dma);
EXPORT_SYMBOL(omap_free_alsa_sound_dma);
EXPORT_SYMBOL(omap_stop_alsa_sound_dma);
