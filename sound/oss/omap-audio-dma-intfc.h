/*  
 * linux/sound/oss/omap-audio-dma-intfc.h
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
 * 2004/08/12  Nishanth Menon - Modified to integrate Audio requirements on 1610,1710 platforms
 */

#ifndef __OMAP_AUDIO_DMA_INTFC_H
#define __OMAP_AUDIO_DMA_INTFC_H

/************************** INCLUDES *************************************/

/* Requires omap-audio.h */
#include "omap-audio.h"

/************************** GLOBAL MACROS *************************************/

/* Provide the Macro interfaces common across platforms */
#define DMA_REQUEST(e,s, cb)	{e=omap_request_sound_dma(s->dma_dev, s->id, s, &s->lch);}
#define DMA_FREE(s)		omap_free_sound_dma(s, &s->lch)
#define DMA_CLEAR(s)		omap_clear_sound_dma(s)

/************************** GLOBAL DATA STRUCTURES *********************************/

typedef void (*dma_callback_t) (int lch, u16 ch_status, void *data);

/************************** GLOBAL FUNCTIONS ***************************************/

dma_callback_t audio_get_dma_callback(void);
int audio_setup_buf(audio_stream_t * s);
int audio_process_dma(audio_stream_t * s);
void audio_prime_rx(audio_state_t * state);
int audio_set_fragments(audio_stream_t * s, int val);
int audio_sync(struct file *file);
void audio_stop_dma(audio_stream_t * s);
u_int audio_get_dma_pos(audio_stream_t * s);
void audio_reset(audio_stream_t * s);
void audio_discard_buf(audio_stream_t * s);

/**************** ARCH SPECIFIC FUNCIONS *******************************************/

void omap_clear_sound_dma(audio_stream_t * s);

int omap_request_sound_dma(int device_id, const char *device_name, void *data,
			   int **channels);
int omap_free_sound_dma(void *data, int **channels);

#endif				/* #ifndef __OMAP_AUDIO_DMA_INTFC_H */
