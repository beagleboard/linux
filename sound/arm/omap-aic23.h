/*
 * sound/arm/omap-aic23.h
 * 
 * Alsa Driver for AIC23 codec on OSK5912 platform board
 *
 * Copyright (C) 2005 Instituto Nokia de Tecnologia - INdT - Manaus Brazil
 * Written by Daniel Petrini, David Cohen, Anderson Briglia
 *            {daniel.petrini, david.cohen, anderson.briglia}@indt.org.br
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
 *  History
 *  -------
 *
 *  2005/07/25 INdT-10LE Kernel Team - 	Alsa driver for omap osk,
 *  					original version based in sa1100 driver
 *  					and omap oss driver.
 *
 *  2005-12-18   Dirk Behme      - Added L/R Channel Interchange fix as proposed by Ajaya Babu
 */

#ifndef __OMAP_AIC23_H
#define __OMAP_AIC23_H

#include <sound/driver.h>
#include <asm/arch/dma.h>
#include <sound/core.h>
#include <sound/pcm.h>

#define DEFAULT_OUTPUT_VOLUME         0x60
#define DEFAULT_INPUT_VOLUME          0x00	/* 0 ==> mute line in */

#define OUTPUT_VOLUME_MIN             LHV_MIN
#define OUTPUT_VOLUME_MAX             LHV_MAX
#define OUTPUT_VOLUME_RANGE           (OUTPUT_VOLUME_MAX - OUTPUT_VOLUME_MIN)
#define OUTPUT_VOLUME_MASK            OUTPUT_VOLUME_MAX

#define INPUT_VOLUME_MIN 	      LIV_MIN
#define INPUT_VOLUME_MAX 	      LIV_MAX
#define INPUT_VOLUME_RANGE 	      (INPUT_VOLUME_MAX - INPUT_VOLUME_MIN)
#define INPUT_VOLUME_MASK 	      INPUT_VOLUME_MAX

#define SIDETONE_MASK                 0x1c0
#define SIDETONE_0                    0x100
#define SIDETONE_6                    0x000
#define SIDETONE_9                    0x040
#define SIDETONE_12                   0x080
#define SIDETONE_18                   0x0c0

#define DEFAULT_ANALOG_AUDIO_CONTROL  DAC_SELECTED | STE_ENABLED | BYPASS_ON | INSEL_MIC | MICB_20DB

/*
 * Buffer management for alsa and dma
 */
struct audio_stream {
	char *id;		/* identification string */
	int stream_id;		/* numeric identification */
	int dma_dev;		/* dma number of that device */
	int *lch;		/* Chain of channels this stream is linked to */
	char started;		/* to store if the chain was started or not */
	int dma_q_head;		/* DMA Channel Q Head */
	int dma_q_tail;		/* DMA Channel Q Tail */
	char dma_q_count;	/* DMA Channel Q Count */
	int active:1;		/* we are using this stream for transfer now */
	int period;		/* current transfer period */
	int periods;		/* current count of periods registerd in the DMA engine */
	spinlock_t dma_lock;	/* for locking in DMA operations */
	snd_pcm_substream_t *stream;	/* the pcm stream */
	unsigned linked:1;	/* dma channels linked */
	int offset;		/* store start position of the last period in the alsa buffer */
	int (*hw_start)(void);  /* interface to start HW interface, e.g. McBSP */
	int (*hw_stop)(void);   /* interface to stop HW interface, e.g. McBSP */
};

/*
 * Alsa card structure for aic23
 */
struct snd_card_omap_aic23 {
	snd_card_t *card;
	snd_pcm_t *pcm;
	long samplerate;
	struct audio_stream s[2];	/* playback & capture */
};

/*********** Function Prototypes *************************/

void audio_dma_callback(void *);
int snd_omap_mixer(struct snd_card_omap_aic23 *);
void snd_omap_init_mixer(void);
/* Clock functions */
int omap_aic23_clock_on(void);
int omap_aic23_clock_off(void);

#ifdef CONFIG_PM
void snd_omap_suspend_mixer(void);
void snd_omap_resume_mixer(void);
#endif

/* Codec AIC23 */
#if defined(CONFIG_SENSORS_TLV320AIC23) || defined (CONFIG_SENSORS_TLV320AIC23_MODULE)

extern int tlv320aic23_write_value(u8 reg, u16 value);

/* TLV320AIC23 is a write only device */
static __inline__ void audio_aic23_write(u8 address, u16 data)
{
	tlv320aic23_write_value(address, data);
}

#endif /* CONFIG_SENSORS_TLV320AIC23 */

#endif
