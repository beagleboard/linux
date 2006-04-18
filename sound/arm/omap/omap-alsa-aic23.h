/*
 * sound/arm/omap-alsa-aic23.h
 * 
 * Alsa Driver for AIC23 codec on OSK5912 platform board
 *
 * Copyright (C) 2005 Instituto Nokia de Tecnologia - INdT - Manaus Brazil
 * Written by Daniel Petrini, David Cohen, Anderson Briglia
 *            {daniel.petrini, david.cohen, anderson.briglia}@indt.org.br
 *
 * Copyright (C) 2006 Mika Laitio <lamikr@cc.jyu.fi>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#ifndef __OMAP_ALSA_AIC23_H
#define __OMAP_ALSA_AIC23_H

#include <sound/driver.h>
#include <asm/arch/dma.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <asm/arch/mcbsp.h>

/* Define to set the AIC23 as the master w.r.t McBSP */
#define AIC23_MASTER

#define NUMBER_SAMPLE_RATES_SUPPORTED	10

/*
 * AUDIO related MACROS
 */
#ifndef DEFAULT_BITPERSAMPLE
#define DEFAULT_BITPERSAMPLE		16
#endif

#define DEFAULT_SAMPLE_RATE		44100
#define CODEC_CLOCK			12000000
#define AUDIO_MCBSP			OMAP_MCBSP1

#define DEFAULT_OUTPUT_VOLUME		0x60
#define DEFAULT_INPUT_VOLUME		0x00	/* 0 ==> mute line in */

#define OUTPUT_VOLUME_MIN		LHV_MIN
#define OUTPUT_VOLUME_MAX		LHV_MAX
#define OUTPUT_VOLUME_RANGE		(OUTPUT_VOLUME_MAX - OUTPUT_VOLUME_MIN)
#define OUTPUT_VOLUME_MASK		OUTPUT_VOLUME_MAX

#define INPUT_VOLUME_MIN		LIV_MIN
#define INPUT_VOLUME_MAX		LIV_MAX
#define INPUT_VOLUME_RANGE		(INPUT_VOLUME_MAX - INPUT_VOLUME_MIN)
#define INPUT_VOLUME_MASK		INPUT_VOLUME_MAX

#define SIDETONE_MASK			0x1c0
#define SIDETONE_0			0x100
#define SIDETONE_6			0x000
#define SIDETONE_9			0x040
#define SIDETONE_12			0x080
#define SIDETONE_18			0x0c0

#define DEFAULT_ANALOG_AUDIO_CONTROL  DAC_SELECTED | STE_ENABLED | BYPASS_ON | INSEL_MIC | MICB_20DB

struct aic23_samplerate_reg_info {
	u32 sample_rate;
	u8 control;		/* SR3, SR2, SR1, SR0 and BOSR */
	u8 divider;		/* if 0 CLKIN = MCLK, if 1 CLKIN = MCLK/2 */
};

/*
 * Defines codec specific functions pointers that can be used from the 
 * common omap-alse base driver for all omap codecs. (tsc2101 and aic23)
 */
void define_codec_functions(struct omap_alsa_codec_config *codec_config);
inline void aic23_configure(void);
void aic23_set_samplerate(long rate);
void aic23_clock_setup(void);
int aic23_clock_on(void);
int aic23_clock_off(void);
int aic23_get_default_samplerate(void);

#endif
