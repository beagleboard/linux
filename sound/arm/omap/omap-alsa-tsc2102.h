/*
 * sound/arm/omap/omap-alsa-tsc2102.h
 * 
 * Alsa codec driver for TSC2102 chip for OMAP platforms.
 *
 * Copyright (c) 2006 Andrzej Zaborowski  <balrog@zabor.org>
 * Code based on the TSC2101 ALSA driver.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#ifndef OMAP_ALSA_TSC2102_H_
#define OMAP_ALSA_TSC2102_H_

/* Define to set the tsc as the master w.r.t McBSP */
#define TSC_MASTER

/*
 * Audio related macros
 */
#ifndef DEFAULT_BITPERSAMPLE
#define DEFAULT_BITPERSAMPLE		16
#endif

#define DEFAULT_SAMPLE_RATE		44100
#define CODEC_CLOCK			12000000
#define AUDIO_MCBSP			OMAP_MCBSP1

/*
 * ALSA mixer related macros
 */
#define OUTPUT_VOLUME_MIN		0x7f	/* 1111111 = -63.5 dB */
#define OUTPUT_VOLUME_MAX		0x00	/* 0000000 */
#define OUTPUT_VOLUME_RANGE		(OUTPUT_VOLUME_MIN - OUTPUT_VOLUME_MAX)

#define DEFAULT_OUTPUT_VOLUME		90	/* Default output volume */

#endif	/* OMAP_ALSA_TSC2102_H_ */
