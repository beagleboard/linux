/*
 * sound/arm/omap-alsa.c
 * 
 * Alsa Driver for OMAP
 *
 * Copyright (C) 2005 Instituto Nokia de Tecnologia - INdT - Manaus Brazil
 * Written by Daniel Petrini, David Cohen, Anderson Briglia
 *            {daniel.petrini, david.cohen, anderson.briglia}@indt.org.br
 *
 * Copyright (C) 2006 Mika Laitio <lamikr@cc.jyu.fi>
 *
 * Based on sa11xx-uda1341.c, 
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
 * 2005-07-29   INdT Kernel Team - Alsa driver for omap osk. Creation of new 
 *                                 file omap-aic23.c
 * 
 * 2005-12-18   Dirk Behme       - Added L/R Channel Interchange fix as proposed 
 *                                 by Ajaya Babu
 *
 */

#include <linux/platform_device.h>
#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#include <sound/driver.h>
#include <sound/core.h>

#include <asm/arch/omap-alsa.h>
#include "omap-alsa-dma.h"

MODULE_AUTHOR("Mika Laitio, Daniel Petrini, David Cohen, Anderson Briglia - INdT");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OMAP driver for ALSA");
MODULE_ALIAS("omap_alsa_mcbsp.1");

static char *id	= NULL;	
static struct snd_card_omap_codec 	*alsa_codec		= NULL;
static struct omap_alsa_codec_config	*alsa_codec_config	= NULL;

/*
 * HW interface start and stop helper functions
 */
static int audio_ifc_start(void)
{
	omap_mcbsp_start(AUDIO_MCBSP);
	return 0;
}

static int audio_ifc_stop(void)
{
	omap_mcbsp_stop(AUDIO_MCBSP);
	return 0;
}

static void omap_alsa_audio_init(struct snd_card_omap_codec *omap_alsa)
{
	/* Setup DMA stuff */
	omap_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].id = "Alsa omap out";
	omap_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].stream_id =
	    SNDRV_PCM_STREAM_PLAYBACK;
	omap_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].dma_dev =
	    OMAP_DMA_MCBSP1_TX;
	omap_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].hw_start =
	    audio_ifc_start;
	omap_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].hw_stop =
	    audio_ifc_stop;

	omap_alsa->s[SNDRV_PCM_STREAM_CAPTURE].id = "Alsa omap in";
	omap_alsa->s[SNDRV_PCM_STREAM_CAPTURE].stream_id =
	    SNDRV_PCM_STREAM_CAPTURE;
	omap_alsa->s[SNDRV_PCM_STREAM_CAPTURE].dma_dev =
	    OMAP_DMA_MCBSP1_RX;
	omap_alsa->s[SNDRV_PCM_STREAM_CAPTURE].hw_start =
	    audio_ifc_start;
	omap_alsa->s[SNDRV_PCM_STREAM_CAPTURE].hw_stop =
	    audio_ifc_stop;
}

/* 
 * DMA functions 
 * Depends on omap-alsa-dma.c functions and (omap) dma.c
 * 
 */
static int audio_dma_request(struct audio_stream *s,
			     void (*callback) (void *))
{
	int err;
	ADEBUG();

	err = omap_request_alsa_sound_dma(s->dma_dev, s->id, s, &s->lch);
	if (err < 0)
		printk(KERN_ERR "Unable to grab audio dma 0x%x\n", s->dma_dev);
	return err;
}

static int audio_dma_free(struct audio_stream *s)
{
	int err = 0;
	ADEBUG();

	err = omap_free_alsa_sound_dma(s, &s->lch);
	if (err < 0)
		printk(KERN_ERR "Unable to free audio dma channels!\n");
	return err;
}

/*
 *  This function should calculate the current position of the dma in the
 *  buffer. It will help alsa middle layer to continue update the buffer.
 *  Its correctness is crucial for good functioning.
 */
static u_int audio_get_dma_pos(struct audio_stream *s)
{
	snd_pcm_substream_t *substream = s->stream;
	snd_pcm_runtime_t *runtime = substream->runtime;
	unsigned int offset;
	unsigned long flags;
	dma_addr_t count;
	ADEBUG();

	/* this must be called w/ interrupts locked as requested in dma.c */
	spin_lock_irqsave(&s->dma_lock, flags);

	/* For the current period let's see where we are */
	count = omap_get_dma_src_addr_counter(s->lch[s->dma_q_head]);

	spin_unlock_irqrestore(&s->dma_lock, flags);

	/* Now, the position related to the end of that period */
	offset = bytes_to_frames(runtime, s->offset) - bytes_to_frames(runtime, count);

	if (offset >= runtime->buffer_size)
		offset = 0;

	return offset;
}

/*
 * this stops the dma and clears the dma ptrs
 */
static void audio_stop_dma(struct audio_stream *s)
{
	unsigned long flags;
	ADEBUG();

	spin_lock_irqsave(&s->dma_lock, flags);
	s->active = 0;
	s->period = 0;
	s->periods = 0;

	/* this stops the dma channel and clears the buffer ptrs */
	omap_stop_alsa_sound_dma(s);

	omap_clear_alsa_sound_dma(s);

	spin_unlock_irqrestore(&s->dma_lock, flags);
}

/*
 *  Main dma routine, requests dma according where you are in main alsa buffer
 */
static void audio_process_dma(struct audio_stream *s)
{
	snd_pcm_substream_t *substream = s->stream;
	snd_pcm_runtime_t *runtime;
	unsigned int dma_size;
	unsigned int offset;
	int ret;
	
	ADEBUG();
	runtime = substream->runtime;
	if (s->active) {
		dma_size = frames_to_bytes(runtime, runtime->period_size);
		offset = dma_size * s->period;
		snd_assert(dma_size <= DMA_BUF_SIZE,);
		/*
		 * On omap1510 based devices, we need to call the stop_dma
		 * before calling the start_dma or we will not receive the
		 * irq from DMA after the first transfered/played buffer.
		 * (invocation of callback_omap_alsa_sound_dma() method).
		 */
		if (cpu_is_omap1510()) {
			omap_stop_alsa_sound_dma(s);
		}
		ret = omap_start_alsa_sound_dma(s,
				(dma_addr_t)runtime->dma_area + offset,
				dma_size);
		if (ret) {
			printk(KERN_ERR
			       "audio_process_dma: cannot queue DMA buffer (%i)\n",
			       ret);
			return;
		}

		s->period++;
		s->period %= runtime->periods;
		s->periods++;
		s->offset = offset;
	}
}

/* 
 *  This is called when dma IRQ occurs at the end of each transmited block
 */
void callback_omap_alsa_sound_dma(void *data)
{
	struct audio_stream *s = data;
	
	ADEBUG();
	/* 
	 * If we are getting a callback for an active stream then we inform
	 * the PCM middle layer we've finished a period
	 */
	if (s->active)
		snd_pcm_period_elapsed(s->stream);

	spin_lock(&s->dma_lock);
	if (s->periods > 0) 
		s->periods--;
	
	audio_process_dma(s);
	spin_unlock(&s->dma_lock);
}

/* 
 * Alsa section
 * PCM settings and callbacks
 */
static int snd_omap_alsa_trigger(snd_pcm_substream_t * substream, int cmd)
{
	struct snd_card_omap_codec *chip =
	    snd_pcm_substream_chip(substream);
	int stream_id = substream->pstr->stream;
	struct audio_stream *s = &chip->s[stream_id];
	int err = 0;
	
	ADEBUG();
	/* note local interrupts are already disabled in the midlevel code */
	spin_lock(&s->dma_lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* requested stream startup */
		s->active = 1;
		audio_process_dma(s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		/* requested stream shutdown */
		audio_stop_dma(s);
		break;
	default:
		err = -EINVAL;
		break;
	}
	spin_unlock(&s->dma_lock);
	
	return err;
}

static int snd_omap_alsa_prepare(snd_pcm_substream_t * substream)
{
	struct snd_card_omap_codec *chip = snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;
	struct audio_stream *s = &chip->s[substream->pstr->stream];
	
	ADEBUG();
	/* set requested samplerate */
	alsa_codec_config->codec_set_samplerate(runtime->rate);
	chip->samplerate = runtime->rate;

	s->period = 0;
	s->periods = 0;

	return 0;
}

static snd_pcm_uframes_t snd_omap_alsa_pointer(snd_pcm_substream_t *substream)
{
	struct snd_card_omap_codec *chip = snd_pcm_substream_chip(substream);

	ADEBUG();	
	return audio_get_dma_pos(&chip->s[substream->pstr->stream]);
}

static int snd_card_omap_alsa_open(snd_pcm_substream_t * substream)
{
	struct snd_card_omap_codec *chip =
	    snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;
	int stream_id = substream->pstr->stream;
	int err;
	
	ADEBUG();
	chip->s[stream_id].stream = substream;
	alsa_codec_config->codec_clock_on();
	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) 
		runtime->hw = *(alsa_codec_config->snd_omap_alsa_playback);
	else 
		runtime->hw = *(alsa_codec_config->snd_omap_alsa_capture);
	
	if ((err = snd_pcm_hw_constraint_integer(runtime,
					   SNDRV_PCM_HW_PARAM_PERIODS)) < 0) 
		return err;
	
	if ((err = snd_pcm_hw_constraint_list(runtime,
					0,
					SNDRV_PCM_HW_PARAM_RATE,
					alsa_codec_config->hw_constraints_rates)) < 0) 
		return err;
	
	return 0;
}

static int snd_card_omap_alsa_close(snd_pcm_substream_t * substream)
{
	struct snd_card_omap_codec *chip = snd_pcm_substream_chip(substream);
	
	ADEBUG();
	alsa_codec_config->codec_clock_off();
	chip->s[substream->pstr->stream].stream = NULL;
	
	return 0;
}

/* HW params & free */
static int snd_omap_alsa_hw_params(snd_pcm_substream_t * substream,
				    snd_pcm_hw_params_t * hw_params)
{
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int snd_omap_alsa_hw_free(snd_pcm_substream_t * substream)
{
	return snd_pcm_lib_free_pages(substream);
}

/* pcm operations */
static snd_pcm_ops_t snd_card_omap_alsa_playback_ops = {
	.open =		snd_card_omap_alsa_open,
	.close =	snd_card_omap_alsa_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	snd_omap_alsa_hw_params,
	.hw_free =	snd_omap_alsa_hw_free,
	.prepare =	snd_omap_alsa_prepare,
	.trigger =	snd_omap_alsa_trigger,
	.pointer =	snd_omap_alsa_pointer,
};

static snd_pcm_ops_t snd_card_omap_alsa_capture_ops = {
	.open =		snd_card_omap_alsa_open,
	.close =	snd_card_omap_alsa_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	snd_omap_alsa_hw_params,
	.hw_free =	snd_omap_alsa_hw_free,
	.prepare =	snd_omap_alsa_prepare,
	.trigger =	snd_omap_alsa_trigger,
	.pointer =	snd_omap_alsa_pointer,
};

/*
 *  Alsa init and exit section
 *  
 *  Inits pcm alsa structures, allocate the alsa buffer, suspend, resume
 */
static int __init snd_card_omap_alsa_pcm(struct snd_card_omap_codec *omap_alsa, 
					int device)
{
	snd_pcm_t *pcm;
	int err;
	
	ADEBUG();
	if ((err = snd_pcm_new(omap_alsa->card, "OMAP PCM", device, 1, 1, &pcm)) < 0)
		return err;

	/* sets up initial buffer with continuous allocation */
	snd_pcm_lib_preallocate_pages_for_all(pcm,
					      SNDRV_DMA_TYPE_CONTINUOUS,
					      snd_dma_continuous_data
					      (GFP_KERNEL),
					      128 * 1024, 128 * 1024);

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_card_omap_alsa_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
			&snd_card_omap_alsa_capture_ops);
	pcm->private_data = omap_alsa;
	pcm->info_flags = 0;
	strcpy(pcm->name, "omap alsa pcm");

	omap_alsa_audio_init(omap_alsa);

	/* setup DMA controller */
	audio_dma_request(&omap_alsa->s[SNDRV_PCM_STREAM_PLAYBACK],
			  callback_omap_alsa_sound_dma);
	audio_dma_request(&omap_alsa->s[SNDRV_PCM_STREAM_CAPTURE],
			  callback_omap_alsa_sound_dma);

	omap_alsa->pcm = pcm;

	return 0;
}


#ifdef CONFIG_PM
/*
 * Driver suspend/resume - calls alsa functions. Some hints from aaci.c
 */
int snd_omap_alsa_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_card_omap_codec *chip;
	snd_card_t *card = platform_get_drvdata(pdev);
	
	if (card->power_state != SNDRV_CTL_POWER_D3hot) {
		chip = card->private_data;
		if (chip->card->power_state != SNDRV_CTL_POWER_D3hot) {
			snd_power_change_state(chip->card, SNDRV_CTL_POWER_D3hot);
			snd_pcm_suspend_all(chip->pcm);
			/* Mutes and turn clock off */
			alsa_codec_config->codec_clock_off();
			snd_omap_suspend_mixer();
		}
	}
	return 0;
}

int snd_omap_alsa_resume(struct platform_device *pdev)
{
	struct snd_card_omap_codec *chip;
	snd_card_t *card = platform_get_drvdata(pdev);

	if (card->power_state != SNDRV_CTL_POWER_D0) {				
		chip = card->private_data;
		if (chip->card->power_state != SNDRV_CTL_POWER_D0) {
			snd_power_change_state(chip->card, SNDRV_CTL_POWER_D0);
			alsa_codec_config->codec_clock_on();
			snd_omap_resume_mixer();
		}
	}
	return 0;
}

#endif	/* CONFIG_PM */

void snd_omap_alsa_free(snd_card_t * card)
{
	struct snd_card_omap_codec *chip = card->private_data;
	ADEBUG();
	
	/*
	 * Turn off codec after it is done.
	 * Can't do it immediately, since it may still have
	 * buffered data.
	 */
	schedule_timeout_interruptible(2);

	omap_mcbsp_stop(AUDIO_MCBSP);
	omap_mcbsp_free(AUDIO_MCBSP);

	audio_dma_free(&chip->s[SNDRV_PCM_STREAM_PLAYBACK]);
	audio_dma_free(&chip->s[SNDRV_PCM_STREAM_CAPTURE]);
}

/* module init & exit */

/* 
 * Inits alsa soudcard structure.
 * Called by the probe method in codec after function pointers has been set.
 */
int snd_omap_alsa_post_probe(struct platform_device *pdev, struct omap_alsa_codec_config *config)
{
	int err = 0;
	int def_rate;
	snd_card_t *card;
	
	ADEBUG();
	alsa_codec_config	= config;

	alsa_codec_config->codec_clock_setup();
	alsa_codec_config->codec_clock_on(); 

	omap_mcbsp_request(AUDIO_MCBSP);
	omap_mcbsp_stop(AUDIO_MCBSP);
	omap_mcbsp_config(AUDIO_MCBSP, alsa_codec_config->mcbsp_regs_alsa);
	omap_mcbsp_start(AUDIO_MCBSP);
	
	if (alsa_codec_config && alsa_codec_config->codec_configure_dev)
		alsa_codec_config->codec_configure_dev();

	alsa_codec_config->codec_clock_off();

	/* register the soundcard */
	card = snd_card_new(-1, id, THIS_MODULE, sizeof(alsa_codec));
	if (card == NULL)
		goto nodev1;

	alsa_codec = kcalloc(1, sizeof(*alsa_codec), GFP_KERNEL);
	if (alsa_codec == NULL)
		goto nodev2;

	card->private_data = (void *)alsa_codec;
	card->private_free = snd_omap_alsa_free;

	alsa_codec->card	= card;
	def_rate		= alsa_codec_config->get_default_samplerate(); 
	alsa_codec->samplerate	= def_rate;

	spin_lock_init(&alsa_codec->s[0].dma_lock);
	spin_lock_init(&alsa_codec->s[1].dma_lock);

	/* mixer */
	if ((err = snd_omap_mixer(alsa_codec)) < 0)
		goto nodev3;

	/* PCM */
	if ((err = snd_card_omap_alsa_pcm(alsa_codec, 0)) < 0)
		goto nodev3;

	strcpy(card->driver, "OMAP_ALSA");
	strcpy(card->shortname, alsa_codec_config->name);
	sprintf(card->longname, alsa_codec_config->name);

	snd_omap_init_mixer();
	snd_card_set_dev(card, &pdev->dev);
	
	if ((err = snd_card_register(card)) == 0) {
		printk(KERN_INFO "audio support initialized\n");
		platform_set_drvdata(pdev, card);
		return 0;
	}
	
nodev3:
	kfree(alsa_codec);	
nodev2:	
	snd_card_free(card);
nodev1:
	omap_mcbsp_stop(AUDIO_MCBSP);
	omap_mcbsp_free(AUDIO_MCBSP);

	return err;
}

int snd_omap_alsa_remove(struct platform_device *pdev)
{
	snd_card_t *card = platform_get_drvdata(pdev);
	struct snd_card_omap_codec *chip = card->private_data;
	
	snd_card_free(card);

	alsa_codec = NULL;
	card->private_data = NULL;
	kfree(chip);
	
	platform_set_drvdata(pdev, NULL);
	
	return 0;
}
