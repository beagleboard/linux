/*
 * sound/arm/omap/omap-alsa-sx1-mixer.h
 *
 * Alsa codec Driver for Siemens SX1 board.
 * based on omap-alsa-tsc2101-mixer.c
 *
 *  Copyright (C) 2006 Vladimir Ananiev (vovan888 at gmail com)
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef OMAPALSATSC2101MIXER_H_
#define OMAPALSATSC2101MIXER_H_

#include "omap-alsa-dma.h"

#define PLAYBACK_TARGET_COUNT		0x03
#define PLAYBACK_TARGET_LOUDSPEAKER	0x00
#define PLAYBACK_TARGET_HEADPHONE	0x01
#define PLAYBACK_TARGET_CELLPHONE	0x02

/* following are used for register 03h Mixer PGA control bits D7-D5 for selecting record source */
#define REC_SRC_TARGET_COUNT		0x08
#define REC_SRC_SINGLE_ENDED_MICIN_HED	0x00	/* oss code referred to MIXER_LINE */
#define REC_SRC_SINGLE_ENDED_MICIN_HND	0x01	/* oss code referred to MIXER_MIC */
#define REC_SRC_SINGLE_ENDED_AUX1	0x02
#define REC_SRC_SINGLE_ENDED_AUX2	0x03
#define REC_SRC_MICIN_HED_AND_AUX1	0x04
#define REC_SRC_MICIN_HED_AND_AUX2	0x05
#define REC_SRC_MICIN_HND_AND_AUX1	0x06
#define REC_SRC_MICIN_HND_AND_AUX2	0x07

#define DEFAULT_OUTPUT_VOLUME		5	/* default output volume to dac dgc */
#define DEFAULT_INPUT_VOLUME		2	/* default record volume */

#endif
