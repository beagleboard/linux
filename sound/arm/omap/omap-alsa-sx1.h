/*
 * arch/arc/mach-omap1/omap-alsa-sx1.h
 *
 * based on omap-alsa-tsc2101.h
 *
 * Alsa Driver for Siemens SX1.
 * Copyright (C) 2006 Vladimir Ananiev (vovan888 at gmail com)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#ifndef OMAP_ALSA_SX1_H_
#define OMAP_ALSA_SX1_H_

#include <linux/types.h>

#define NUMBER_SAMPLE_RATES_SUPPORTED	9

/*
 * AUDIO related MACROS
 */
#ifndef DEFAULT_BITPERSAMPLE
#define DEFAULT_BITPERSAMPLE		16
#endif

#define DEFAULT_SAMPLE_RATE		44100
/* fw15: 18356000 */
#define CODEC_CLOCK 			18359000
/* McBSP for playing music */
#define AUDIO_MCBSP			OMAP_MCBSP1
/* McBSP for record/play audio from phone and mic */
#define AUDIO_MCBSP_PCM			OMAP_MCBSP2
/* gpio pin for enable/disable clock */
#define OSC_EN				2

/* Send IPC message to sound server */
extern int cn_sx1snd_send(unsigned int cmd, unsigned int arg1, unsigned int arg2);
/* cmd for IPC_GROUP_DAC */
#define DAC_VOLUME_UPDATE		0
#define DAC_SETAUDIODEVICE		1
#define DAC_OPEN_RING			2
#define DAC_OPEN_DEFAULT		3
#define DAC_CLOSE			4
#define DAC_FMRADIO_OPEN		5
#define DAC_FMRADIO_CLOSE		6
#define DAC_PLAYTONE			7
/* cmd for IPC_GROUP_PCM */
#define PCM_PLAY			(0+8)
#define PCM_RECORD			(1+8)
#define PCM_CLOSE			(2+8)

/* for DAC_SETAUDIODEVICE */
#define SX1_DEVICE_SPEAKER		0
#define SX1_DEVICE_HEADPHONE		4
#define SX1_DEVICE_PHONE		3
/* frequencies for MdaDacOpenDefaultL,	MdaDacOpenRingL */
#define FRQ_8000	0
#define FRQ_11025		1
#define FRQ_12000		2
#define FRQ_16000		3
#define FRQ_22050		4
#define FRQ_24000		5
#define FRQ_32000		6
#define FRQ_44100		7
#define FRQ_48000		8

#endif
