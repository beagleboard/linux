/*
 * sound/arm/omap/omap-alsa-tsc2101-mixer.c
 * 
 * Alsa Driver for TSC2101 codec for OMAP platform boards.
 *
 * Copyright (C) 2005 Mika Laitio <lamikr@cc.jyu.fi> and 
 * 		     Everett Coleman II <gcc80x86@fuzzyneural.net>
 *
 * Based on the ideas in omap-aic23.c and sa11xx-uda1341.c
 * Copyright (C) 2005 Instituto Nokia de Tecnologia - INdT - Manaus Brazil
 * Copyright (C) 2002 Tomas Kasparek <tomas.kasparek@seznam.cz>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * History:
 *
 * 2006-03-01   Mika Laitio - Mixer for the tsc2101 driver used in omap boards.
 * 		Can switch between headset and loudspeaker playback, 
 * 		mute and unmute dgc, set dgc volume. Record source switch,
 * 		keyclick, buzzer and headset volume and handset volume control 
 * 		are still missing.
 */

#ifndef OMAPALSATSC2101MIXER_H_
#define OMAPALSATSC2101MIXER_H_

#include <asm/hardware/tsc2101.h>
#include <../drivers/ssi/omap-tsc2101.h>
#include "omap-alsa-dma.h"

/* tsc2101 DAC gain control volume specific  */
#define OUTPUT_VOLUME_MIN		0x7F	// 1111111 = -63.5 DB
#define OUTPUT_VOLUME_MAX		0x32	// 110010
#define OUTPUT_VOLUME_RANGE		(OUTPUT_VOLUME_MIN - OUTPUT_VOLUME_MAX)

/* use input vol of 75 for 0dB gain */
#define INPUT_VOLUME_MIN 		0x0
#define INPUT_VOLUME_MAX		0x7D
#define INPUT_VOLUME_RANGE		(INPUT_VOLUME_MAX - INPUT_VOLUME_MIN)

#define PLAYBACK_TARGET_COUNT		0x03
#define PLAYBACK_TARGET_LOUDSPEAKER	0x00
#define PLAYBACK_TARGET_HEADPHONE	0x01
#define PLAYBACK_TARGET_CELLPHONE	0x02

/* following are used for register 03h Mixer PGA control bits D7-D5 for selecting record source */
#define REC_SRC_TARGET_COUNT		0x08
#define REC_SRC_SINGLE_ENDED_MICIN_HED	0x00	// oss code referred to MIXER_LINE
#define REC_SRC_SINGLE_ENDED_MICIN_HND	0x01	// oss code referred to MIXER_MIC
#define REC_SRC_SINGLE_ENDED_AUX1	0x02
#define REC_SRC_SINGLE_ENDED_AUX2	0x03
#define REC_SRC_MICIN_HED_AND_AUX1	0x04
#define REC_SRC_MICIN_HED_AND_AUX2	0x05
#define REC_SRC_MICIN_HND_AND_AUX1	0x06
#define REC_SRC_MICIN_HND_AND_AUX2	0x07

#define DEFAULT_OUTPUT_VOLUME		90	// default output volume to dac dgc
#define DEFAULT_INPUT_VOLUME		20	// default record volume

#define TSC2101_AUDIO_CODEC_REGISTERS_PAGE2     (2)

#endif /*OMAPALSATSC2101MIXER_H_*/
