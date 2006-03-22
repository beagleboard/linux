/*
 * arch/arc/mach-omap1/omap-alsa-tsc2101.h
 * 
 * Alsa Driver for TSC2101 codec for OMAP platform boards.
 *
 * Based on former omap-aic23.h and tsc2101 OSS drivers.
 * Copyright (C) 2004 Texas Instruments, Inc.
 * 	Written by Nishanth Menon and Sriram Kannan
 *
 * Copyright (C) 2006 Instituto Nokia de Tecnologia - INdT - Manaus Brazil
 *          Alsa modularization by Daniel Petrini (d.pensator@gmail.com)
 * 
 * Copyright (C) 2006 Mika Laitio <lamikr@cc.jyu.fi>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#ifndef OMAP_ALSA_TSC2101_H_
#define OMAP_ALSA_TSC2101_H_

#include <linux/types.h>

/* Define to set the tsc as the master w.r.t McBSP */
#define TSC_MASTER

#define NUMBER_SAMPLE_RATES_SUPPORTED	16

/*
 * AUDIO related MACROS
 */
#ifndef DEFAULT_BITPERSAMPLE
#define DEFAULT_BITPERSAMPLE		16
#endif

#define DEFAULT_SAMPLE_RATE		44100
#define CODEC_CLOCK 			12000000
#define AUDIO_MCBSP        		OMAP_MCBSP1

#define PAGE2_AUDIO_CODEC_REGISTERS	(2)

struct tsc2101_samplerate_reg_info {
	u16 sample_rate;
	u8 divisor;
	u8 fs_44kHz;	/* if 0 48 khz, if 1 44.1 khz fsref */
};

/*
 * Defines codec specific functions pointers that can be used from the 
 * common omap-alse base driver for all omap codecs. (tsc2101 and aic23)
 */
inline void tsc2101_configure(void);
void tsc2101_set_samplerate(long rate);
void tsc2101_clock_setup(void);
int tsc2101_clock_on(void);
int tsc2101_clock_off(void);
int tsc2101_get_default_samplerate(void);

#endif /*OMAP_ALSA_TSC2101_H_*/
