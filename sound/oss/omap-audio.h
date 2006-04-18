/*
 * linux/sound/oss/omap-audio.h
 *
 * Common audio handling for the OMAP processors
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
 *  History
 *  -------
 *  2004/08/12 Nishanth Menon - Modified to integrate Audio requirements on 1610,1710 platforms
 *
 *  2004/04/04 Nishanth menon - Added hooks for power management
 *
 *  2005/12/10 Dirk Behme     - Added L/R Channel Interchange fix as proposed by Ajaya Babu
 */

#ifndef __OMAP_AUDIO_H
#define __OMAP_AUDIO_H

/* Requires dma.h */
#include <asm/arch/dma.h>

/*
 * Buffer Management
 */
typedef struct {
	int offset;		/* current offset */
	char *data;		/* points to actual buffer */
	dma_addr_t dma_addr;	/* physical buffer address */
	int dma_ref;		/* DMA refcount */
	int master;		/* owner for buffer allocation, contain size when true */
} audio_buf_t;

/*
 * Structure describing the data stream related information
 */
typedef struct {
	char *id;		/* identification string */
	audio_buf_t *buffers;	/* pointer to audio buffer structures */
	u_int usr_head;		/* user fragment index */
	u_int dma_head;		/* DMA fragment index to go */
	u_int dma_tail;		/* DMA fragment index to complete */
	u_int fragsize;		/* fragment i.e. buffer size */
	u_int nbfrags;		/* nbr of fragments i.e. buffers */
	u_int pending_frags;	/* Fragments sent to DMA */
	int dma_dev;		/* device identifier for DMA */

#ifdef OMAP_DMA_CHAINING_SUPPORT
	lch_chain *dma_chain;
	dma_regs_t *dma_regs;	/* points to our DMA registers */
#else
	char started;		/* to store if the chain was started or not */
	int dma_q_head;		/* DMA Channel Q Head */
	int dma_q_tail;		/* DMA Channel Q Tail */
	char dma_q_count;	/* DMA Channel Q Count */
	char in_use;		/*  Is this is use? */
	int *lch;		/*  Chain of channels this stream is linked to */
#endif
	int input_or_output;	/* Direction of this data stream */
	int bytecount;		/* nbr of processed bytes */
	int fragcount;		/* nbr of fragment transitions */
	struct completion wfc;  /* wait for "nbfrags" fragment completion */
	wait_queue_head_t wq;	/* for poll */
	int dma_spinref;	/* DMA is spinning */
	unsigned mapped:1;	/* mmap()'ed buffers */
	unsigned active:1;	/* actually in progress */
	unsigned stopped:1;	/* might be active but stopped */
	unsigned spin_idle:1;	/* have DMA spin on zeros when idle */
	unsigned linked:1;	/* dma channels linked */
	int (*hw_start)(void);  /* interface to start HW interface, e.g. McBSP */
	int (*hw_stop)(void);   /* interface to stop HW interface, e.g. McBSP */
} audio_stream_t;

/*
 * State structure for one instance
 */
typedef struct {
	struct module *owner;   /* Codec module ID */
	audio_stream_t *output_stream;
	audio_stream_t *input_stream;
	unsigned rd_ref:1;	/* open reference for recording */
	unsigned wr_ref:1;	/* open reference for playback */
	unsigned need_tx_for_rx:1; /* if data must be sent while receiving */
	void *data;
	void (*hw_init) (void *);
	void (*hw_shutdown) (void *);
	int (*client_ioctl) (struct inode *, struct file *, uint, ulong);
	int (*hw_probe) (void);
	void (*hw_remove) (void);
	void (*hw_cleanup) (void);
	int (*hw_suspend) (void);
	int (*hw_resume) (void);
	struct pm_dev *pm_dev;
	struct mutex mutex;	/* to protect against races in attach() */
} audio_state_t;

#ifdef AUDIO_PM
void audio_ldm_suspend(void *data);

void audio_ldm_resume(void *data);

#endif

/* Register a Codec using this function */
extern int audio_register_codec(audio_state_t * codec_state);
/* Un-Register a Codec using this function */
extern int audio_unregister_codec(audio_state_t * codec_state);
/* Function to provide fops of omap audio driver */
extern struct file_operations *audio_get_fops(void);
/* Function to initialize the device info for audio driver */
extern int audio_dev_init(void);
/* Function to un-initialize the device info for audio driver */
void audio_dev_uninit(void);

#endif				/* End of #ifndef __OMAP_AUDIO_H */
