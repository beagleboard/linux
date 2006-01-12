/*
 * sound/arm/omap-aic23.c
 * 
 * Alsa Driver for AIC23 codec on OSK5912 platform board
 *
 * Copyright (C) 2005 Instituto Nokia de Tecnologia - INdT - Manaus Brazil
 * Written by Daniel Petrini, David Cohen, Anderson Briglia
 *            {daniel.petrini, david.cohen, anderson.briglia}@indt.org.br
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
 */

#include <linux/config.h>
#include <sound/driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/slab.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/arch/dma.h>
#include <asm/arch/aic23.h>
#include <asm/hardware/clock.h>
#include <asm/arch/mcbsp.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/memalloc.h>

#include "omap-alsa-dma.h"
#include "omap-aic23.h"

#undef DEBUG

#ifdef DEBUG
#define ADEBUG() printk("XXX Alsa debug f:%s, l:%d\n", __FUNCTION__, __LINE__)
#else
#define ADEBUG()		/* nop */
#endif

/* Define to set the AIC23 as the master w.r.t McBSP */
#define AIC23_MASTER

/*
 * AUDIO related MACROS
 */
#define DEFAULT_BITPERSAMPLE          16
#define AUDIO_RATE_DEFAULT	      44100
#define AUDIO_MCBSP                   OMAP_MCBSP1
#define NUMBER_SAMPLE_RATES_SUPPORTED 10


MODULE_AUTHOR("Daniel Petrini, David Cohen, Anderson Briglia - INdT");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OMAP AIC23 driver for ALSA");
MODULE_SUPPORTED_DEVICE("{{AIC23,OMAP AIC23}}");
MODULE_ALIAS("omap_mcbsp.1");

static char *id = NULL;	
MODULE_PARM_DESC(id, "OMAP OSK ALSA Driver for AIC23 chip.");

static struct snd_card_omap_aic23 *omap_aic23 = NULL;

static struct clk *aic23_mclk = 0;

struct sample_rate_rate_reg_info {
	u8 control;		/* SR3, SR2, SR1, SR0 and BOSR */
	u8 divider;		/* if 0 CLKIN = MCLK, if 1 CLKIN = MCLK/2 */
};

/*
 * DAC USB-mode sampling rates (MCLK = 12 MHz)
 * The rates and rate_reg_into MUST be in the same order
 */
static unsigned int rates[] = {
	4000, 8000, 16000, 22050,
	24000, 32000, 44100,
	48000, 88200, 96000,
};
static const struct sample_rate_rate_reg_info
 rate_reg_info[NUMBER_SAMPLE_RATES_SUPPORTED] = {
	{0x06, 1},		/*  4000 */
	{0x06, 0},		/*  8000 */
	{0x0C, 1},		/* 16000 */
	{0x11, 1},              /* 22050 */
	{0x00, 1},		/* 24000 */
	{0x0C, 0},		/* 32000 */
	{0x11, 0},		/* 44100 */
	{0x00, 0},		/* 48000 */
	{0x1F, 0},		/* 88200 */
	{0x0E, 0},		/* 96000 */
};

/*
 *  mcbsp configuration structure
 */
static struct omap_mcbsp_reg_cfg initial_config_mcbsp = {
	.spcr2 = FREE | FRST | GRST | XRST | XINTM(3),
	.spcr1 = RINTM(3) | RRST,
	.rcr2 = RPHASE | RFRLEN2(OMAP_MCBSP_WORD_8) |
	    RWDLEN2(OMAP_MCBSP_WORD_16) | RDATDLY(0),
	.rcr1 = RFRLEN1(OMAP_MCBSP_WORD_8) | RWDLEN1(OMAP_MCBSP_WORD_16),
	.xcr2 = XPHASE | XFRLEN2(OMAP_MCBSP_WORD_8) |
	    XWDLEN2(OMAP_MCBSP_WORD_16) | XDATDLY(0) | XFIG,
	.xcr1 = XFRLEN1(OMAP_MCBSP_WORD_8) | XWDLEN1(OMAP_MCBSP_WORD_16),
	.srgr1 = FWID(DEFAULT_BITPERSAMPLE - 1),
	.srgr2 = GSYNC | CLKSP | FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1),
#ifndef AIC23_MASTER
	/* configure McBSP to be the I2S master */
	.pcr0 = FSXM | FSRM | CLKXM | CLKRM | CLKXP | CLKRP,
#else
	/* configure McBSP to be the I2S slave */
	.pcr0 = CLKXP | CLKRP,
#endif				/* AIC23_MASTER */
};

static snd_pcm_hw_constraint_list_t hw_constraints_rates = {
	.count = ARRAY_SIZE(rates),
	.list = rates,
	.mask = 0,
};


/*
 * Codec/mcbsp init and configuration section
 * codec dependent code.
 */

/*
 * Sample rate changing
 */
static void omap_aic23_set_samplerate(struct snd_card_omap_aic23
				      *omap_aic23, long rate)
{
	u8 count = 0;
	u16 data = 0;

	/* Fix the rate if it has a wrong value */
	if (rate >= 96000)
		rate = 96000;
	else if (rate >= 88200)
		rate = 88200;
	else if (rate >= 48000)
		rate = 48000;
	else if (rate >= 44100)
		rate = 44100;
	else if (rate >= 32000)
		rate = 32000;
	else if (rate >= 24000)
		rate = 24000;
	else if (rate >= 22050)
		rate = 22050;
	else if (rate >= 16000)
		rate = 16000;
	else if (rate >= 8000)
		rate = 8000;
	else
		rate = 4000;

	/* Search for the right sample rate */
	/* Verify what happens if the rate is not supported
	 * now it goes to 96Khz */
	while ((rates[count] != rate) &&
	       (count < (NUMBER_SAMPLE_RATES_SUPPORTED - 1))) {
		count++;
	}

	data = (rate_reg_info[count].divider << CLKIN_SHIFT) |
	    (rate_reg_info[count].control << BOSR_SHIFT) | USB_CLK_ON;

	audio_aic23_write(SAMPLE_RATE_CONTROL_ADDR, data);

	omap_aic23->samplerate = rate;
}

static inline void aic23_configure(void)
{
	/* Reset codec */
	audio_aic23_write(RESET_CONTROL_ADDR, 0);

	/* Initialize the AIC23 internal state */

	/* Analog audio path control, DAC selected, delete INSEL_MIC for line in */
	audio_aic23_write(ANALOG_AUDIO_CONTROL_ADDR, DEFAULT_ANALOG_AUDIO_CONTROL);

	/* Digital audio path control, de-emphasis control 44.1kHz */
	audio_aic23_write(DIGITAL_AUDIO_CONTROL_ADDR, DEEMP_44K);

	/* Digital audio interface, master/slave mode, I2S, 16 bit */
#ifdef AIC23_MASTER
	audio_aic23_write(DIGITAL_AUDIO_FORMAT_ADDR,
			  MS_MASTER | IWL_16 | FOR_DSP);
#else
	audio_aic23_write(DIGITAL_AUDIO_FORMAT_ADDR, IWL_16 | FOR_DSP);
#endif

	/* Enable digital interface */
	audio_aic23_write(DIGITAL_INTERFACE_ACT_ADDR, ACT_ON);

}

static void omap_aic23_audio_init(struct snd_card_omap_aic23 *omap_aic23)
{
	/* Setup DMA stuff */
	omap_aic23->s[SNDRV_PCM_STREAM_PLAYBACK].id = "Alsa AIC23 out";
	omap_aic23->s[SNDRV_PCM_STREAM_PLAYBACK].stream_id =
	    SNDRV_PCM_STREAM_PLAYBACK;
	omap_aic23->s[SNDRV_PCM_STREAM_PLAYBACK].dma_dev =
	    OMAP_DMA_MCBSP1_TX;

	omap_aic23->s[SNDRV_PCM_STREAM_CAPTURE].id = "Alsa AIC23 in";
	omap_aic23->s[SNDRV_PCM_STREAM_CAPTURE].stream_id =
	    SNDRV_PCM_STREAM_CAPTURE;
	omap_aic23->s[SNDRV_PCM_STREAM_CAPTURE].dma_dev =
	    OMAP_DMA_MCBSP1_RX;

	/* configuring the McBSP */
	omap_mcbsp_request(AUDIO_MCBSP);

	/* if configured, then stop mcbsp */
	omap_mcbsp_stop(AUDIO_MCBSP);

	omap_mcbsp_config(AUDIO_MCBSP, &initial_config_mcbsp);
	omap_mcbsp_start(AUDIO_MCBSP);
	aic23_configure();
}

/* 
 * DMA functions 
 * Depends on omap-aic23-dma.c functions and (omap) dma.c
 * 
 */
#define DMA_BUF_SIZE	1024 * 8

static int audio_dma_request(struct audio_stream *s,
			     void (*callback) (void *))
{
	int err;

	err = omap_request_sound_dma(s->dma_dev, s->id, s, &s->lch);
	if (err < 0)
		printk(KERN_ERR "unable to grab audio dma 0x%x\n",
		       s->dma_dev);
	return err;
}

static int audio_dma_free(struct audio_stream *s)
{
	int err = 0;

	err = omap_free_sound_dma(s, &s->lch);
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

	if (offset >= runtime->buffer_size || offset < 0)
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
	omap_audio_stop_dma(s);

	omap_clear_sound_dma(s);

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

	runtime = substream->runtime;
	if (s->active) {
		dma_size = frames_to_bytes(runtime, runtime->period_size);
		offset = dma_size * s->period;
		snd_assert(dma_size <= DMA_BUF_SIZE,);
		ret =
		    omap_start_sound_dma(s,
					 (dma_addr_t) runtime->dma_area +
					 offset, dma_size);
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
void audio_dma_callback(void *data)
{
	struct audio_stream *s = data;

	/* 
	 * If we are getting a callback for an active stream then we inform
	 * the PCM middle layer we've finished a period
	 */
	if (s->active)
		snd_pcm_period_elapsed(s->stream);

	spin_lock(&s->dma_lock);
	if (s->periods > 0) {
		s->periods--;
	}
	audio_process_dma(s);
	spin_unlock(&s->dma_lock);
}


/* 
 * Alsa section
 * PCM settings and callbacks
 */

static int snd_omap_aic23_trigger(snd_pcm_substream_t * substream, int cmd)
{
	struct snd_card_omap_aic23 *chip =
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

static int snd_omap_aic23_prepare(snd_pcm_substream_t * substream)
{
	struct snd_card_omap_aic23 *chip =
	    snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;
	struct audio_stream *s = &chip->s[substream->pstr->stream];

	/* set requested samplerate */
	omap_aic23_set_samplerate(chip, runtime->rate);

	s->period = 0;
	s->periods = 0;

	return 0;
}

static snd_pcm_uframes_t snd_omap_aic23_pointer(snd_pcm_substream_t *
						substream)
{
	struct snd_card_omap_aic23 *chip =
	    snd_pcm_substream_chip(substream);
	
	return audio_get_dma_pos(&chip->s[substream->pstr->stream]);
}

/* Hardware capabilities */

static snd_pcm_hardware_t snd_omap_aic23_capture = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
		  SNDRV_PCM_RATE_KNOT),
	.rate_min = 8000,
	.rate_max = 96000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = 8 * 1024,
	.periods_min = 16,
	.periods_max = 255,
	.fifo_size = 0,
};

static snd_pcm_hardware_t snd_omap_aic23_playback = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),	
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
		  SNDRV_PCM_RATE_KNOT),
	.rate_min = 8000,
	.rate_max = 96000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = 8 * 1024,
	.periods_min = 16,
	.periods_max = 255,
	.fifo_size = 0,
};

static int snd_card_omap_aic23_open(snd_pcm_substream_t * substream)
{
	struct snd_card_omap_aic23 *chip =
	    snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;
	int stream_id = substream->pstr->stream;
	int err;
	ADEBUG();

	chip->s[stream_id].stream = substream;
	
	omap_aic23_clock_on();
	
	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK)
		runtime->hw = snd_omap_aic23_playback;
	else
		runtime->hw = snd_omap_aic23_capture;
	if ((err =
	     snd_pcm_hw_constraint_integer(runtime,
					   SNDRV_PCM_HW_PARAM_PERIODS)) <
	    0)
		return err;
	if ((err =
	     snd_pcm_hw_constraint_list(runtime, 0,
					SNDRV_PCM_HW_PARAM_RATE,
					&hw_constraints_rates)) < 0)
		return err;

	return 0;
}

static int snd_card_omap_aic23_close(snd_pcm_substream_t * substream)
{
	struct snd_card_omap_aic23 *chip =
	    snd_pcm_substream_chip(substream);
	ADEBUG();
	
	omap_aic23_clock_off();
	chip->s[substream->pstr->stream].stream = NULL;
	
	return 0;
}

/* HW params & free */

static int snd_omap_aic23_hw_params(snd_pcm_substream_t * substream,
				    snd_pcm_hw_params_t * hw_params)
{
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int snd_omap_aic23_hw_free(snd_pcm_substream_t * substream)
{
	return snd_pcm_lib_free_pages(substream);
}

/* pcm operations */

static snd_pcm_ops_t snd_card_omap_aic23_playback_ops = {
	.open =		snd_card_omap_aic23_open,
	.close =	snd_card_omap_aic23_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	snd_omap_aic23_hw_params,
	.hw_free =	snd_omap_aic23_hw_free,
	.prepare =	snd_omap_aic23_prepare,
	.trigger =	snd_omap_aic23_trigger,
	.pointer =	snd_omap_aic23_pointer,
};

static snd_pcm_ops_t snd_card_omap_aic23_capture_ops = {
	.open =		snd_card_omap_aic23_open,
	.close =	snd_card_omap_aic23_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	snd_omap_aic23_hw_params,
	.hw_free =	snd_omap_aic23_hw_free,
	.prepare =	snd_omap_aic23_prepare,
	.trigger =	snd_omap_aic23_trigger,
	.pointer =	snd_omap_aic23_pointer,
};

/*
 *  Alsa init and exit section
 *  
 *  Inits pcm alsa structures, allocate the alsa buffer, suspend, resume
 */
static int __init snd_card_omap_aic23_pcm(struct snd_card_omap_aic23
					  *omap_aic23, int device)
{
	snd_pcm_t *pcm;
	int err;
	ADEBUG();

	if ((err =
	     snd_pcm_new(omap_aic23->card, "AIC23 PCM", device, 1, 1,
			 &pcm)) < 0)
		return err;

	/* sets up initial buffer with continuous allocation */
	snd_pcm_lib_preallocate_pages_for_all(pcm,
					      SNDRV_DMA_TYPE_CONTINUOUS,
					      snd_dma_continuous_data
					      (GFP_KERNEL),
					      128 * 1024, 128 * 1024);

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_card_omap_aic23_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
			&snd_card_omap_aic23_capture_ops);
	pcm->private_data = omap_aic23;
	pcm->info_flags = 0;
	strcpy(pcm->name, "omap aic23 pcm");

	omap_aic23_audio_init(omap_aic23);

	/* setup DMA controller */
	audio_dma_request(&omap_aic23->s[SNDRV_PCM_STREAM_PLAYBACK],
			  audio_dma_callback);
	audio_dma_request(&omap_aic23->s[SNDRV_PCM_STREAM_CAPTURE],
			  audio_dma_callback);

	omap_aic23->pcm = pcm;

	return 0;
}


#ifdef CONFIG_PM

static int snd_omap_aic23_suspend(snd_card_t * card, pm_message_t state)
{
	struct snd_card_omap_aic23 *chip = card->pm_private_data;
	ADEBUG();

	if (chip->card->power_state != SNDRV_CTL_POWER_D3hot) {
		snd_power_change_state(chip->card, SNDRV_CTL_POWER_D3hot);
		snd_pcm_suspend_all(chip->pcm);
		/* Mutes and turn clock off */
		omap_aic23_clock_off();
		snd_omap_suspend_mixer();
	}

	return 0;
}

/*
 *  Prepare hardware for resume
 */
static int snd_omap_aic23_resume(snd_card_t * card)
{
	struct snd_card_omap_aic23 *chip = card->pm_private_data;
	ADEBUG();
	
	if (chip->card->power_state != SNDRV_CTL_POWER_D0) {
		snd_power_change_state(chip->card, SNDRV_CTL_POWER_D0);
		omap_aic23_clock_on();
		snd_omap_resume_mixer();
	}

	return 0;
}

/*
 * Driver suspend/resume - calls alsa functions. Some hints from aaci.c
 */
static int omap_aic23_suspend(struct platform_device *pdev, pm_message_t state)
{
	snd_card_t *card = platform_get_drvdata(pdev);
	
	if (card->power_state != SNDRV_CTL_POWER_D3hot) {
		snd_omap_aic23_suspend(card, PMSG_SUSPEND);
	}
	return 0;
}

static int omap_aic23_resume(struct platform_device *pdev)
{
	snd_card_t *card = platform_get_drvdata(pdev);

	if (card->power_state != SNDRV_CTL_POWER_D0) {
		snd_omap_aic23_resume(card);
	}
	return 0;
}

#else
#define snd_omap_aic23_suspend	NULL
#define snd_omap_aic23_resume	NULL
#define omap_aic23_suspend	NULL
#define omap_aic23_resume	NULL

#endif	/* CONFIG_PM */

/* 
 */
void snd_omap_aic23_free(snd_card_t * card)
{
	struct snd_card_omap_aic23 *chip = card->private_data;
	ADEBUG();
	
	/*
	 * Turn off codec after it is done.
	 * Can't do it immediately, since it may still have
	 * buffered data.
	 */
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(2);

	omap_mcbsp_stop(AUDIO_MCBSP);
	omap_mcbsp_free(AUDIO_MCBSP);

	audio_aic23_write(RESET_CONTROL_ADDR, 0);
	audio_aic23_write(POWER_DOWN_CONTROL_ADDR, 0xff);

	audio_dma_free(&chip->s[SNDRV_PCM_STREAM_PLAYBACK]);
	audio_dma_free(&chip->s[SNDRV_PCM_STREAM_CAPTURE]);
}

/*
 *  Omap MCBSP clock configuration
 *  
 *  Here we have some functions that allows clock to be enabled and
 *   disabled only when needed. Besides doing clock configuration 
 *   it allows turn on/turn off audio when necessary. 
 */
#define CODEC_CLOCK                   12000000
#define AUDIO_RATE_DEFAULT	      44100

/*
 * Do clock framework mclk search
 */
static __init void omap_aic23_clock_setup(void)
{
	aic23_mclk = clk_get(0, "mclk");
}

/*
 * Do some sanity check, set clock rate, starts it and
 *  turn codec audio on 
 */
int omap_aic23_clock_on(void)
{
	if (clk_get_usecount(aic23_mclk) > 0) {
		/* MCLK is already in use */
		printk(KERN_WARNING
		       "MCLK in use at %d Hz. We change it to %d Hz\n",
		       (uint) clk_get_rate(aic23_mclk),
		       CODEC_CLOCK);
	}
	
	if (clk_set_rate(aic23_mclk, CODEC_CLOCK)) {
		printk(KERN_ERR
		       "Cannot set MCLK for AIC23 CODEC\n");
		return -ECANCELED;
	}

	clk_enable(aic23_mclk);

	printk(KERN_DEBUG
		"MCLK = %d [%d], usecount = %d\n",
	       (uint) clk_get_rate(aic23_mclk), CODEC_CLOCK,
	       clk_get_usecount(aic23_mclk));

	/* Now turn the audio on */
	audio_aic23_write(POWER_DOWN_CONTROL_ADDR, 
			  ~DEVICE_POWER_OFF & ~OUT_OFF & ~DAC_OFF &
			  ~ADC_OFF & ~MIC_OFF & ~LINE_OFF);
	
	return 0;
}
/*
 * Do some sanity check, turn clock off and then turn
 *  codec audio off
 */
int omap_aic23_clock_off(void)
{
	if  (clk_get_usecount(aic23_mclk) > 0) { 
		if (clk_get_rate(aic23_mclk) != CODEC_CLOCK) {
			printk(KERN_WARNING
			       "MCLK for audio should be %d Hz. But is %d Hz\n",
			       (uint) clk_get_rate(aic23_mclk),
			       CODEC_CLOCK);
		}

		clk_disable(aic23_mclk);
	}
	
	audio_aic23_write(POWER_DOWN_CONTROL_ADDR,
			  DEVICE_POWER_OFF | OUT_OFF | DAC_OFF |
			  ADC_OFF | MIC_OFF | LINE_OFF);	
	return 0;
}

/* module init & exit */

/* 
 *  Inits alsa soudcard structure
 */
static int __init snd_omap_aic23_probe(struct platform_device *pdev)
{
	int err = 0;
	snd_card_t *card;
	ADEBUG();
	
	/* gets clock from clock framework */
	omap_aic23_clock_setup();

	/* register the soundcard */
	card = snd_card_new(-1, id, THIS_MODULE, sizeof(omap_aic23));
	if (card == NULL)
		return -ENOMEM;

	omap_aic23 = kcalloc(1, sizeof(*omap_aic23), GFP_KERNEL);
	if (omap_aic23 == NULL)
		return -ENOMEM;

	card->private_data = (void *) omap_aic23;
	card->private_free = snd_omap_aic23_free;

	omap_aic23->card = card;
	omap_aic23->samplerate = AUDIO_RATE_DEFAULT;

	spin_lock_init(&omap_aic23->s[0].dma_lock);
	spin_lock_init(&omap_aic23->s[1].dma_lock);

	/* mixer */
	if ((err = snd_omap_mixer(omap_aic23)) < 0) 
		goto nodev;

	/* PCM */
	if ((err = snd_card_omap_aic23_pcm(omap_aic23, 0)) < 0)
		goto nodev;

	snd_card_set_pm_callback(card, snd_omap_aic23_suspend,
				 snd_omap_aic23_resume, omap_aic23);

	strcpy(card->driver, "AIC23");
	strcpy(card->shortname, "OSK AIC23");
	sprintf(card->longname, "OMAP OSK with AIC23");

	snd_omap_init_mixer();
	
	if ((err = snd_card_register(card)) == 0) {
		printk(KERN_INFO "OSK audio support initialized\n");
		platform_set_drvdata(pdev, card);
		return 0;
	}
	
nodev:
	snd_omap_aic23_free(card);
	
	return err;
}

static int snd_omap_aic23_remove(struct platform_device *pdev)
{
	snd_card_t *card = platform_get_drvdata(pdev);
	struct snd_card_omap_aic23 *chip = card->private_data;
	
	snd_card_free(card);

	omap_aic23 = NULL;
	card->private_data = NULL;
	kfree(chip);
	
	platform_set_drvdata(pdev, NULL);
	
	return 0;
	
}

static struct platform_driver omap_alsa_driver = {
	.probe =	snd_omap_aic23_probe,
	.remove =	snd_omap_aic23_remove,
	.suspend =	omap_aic23_suspend, 
	.resume =	omap_aic23_resume, 
	.driver = {
		.name =	"omap_mcbsp",
	},
};

static int __init omap_aic23_init(void)
{
	int err;
	ADEBUG();

	err = platform_driver_register(&omap_alsa_driver);

	return err;
}

static void __exit omap_aic23_exit(void)
{
	ADEBUG();
	
	platform_driver_unregister(&omap_alsa_driver);
}

module_init(omap_aic23_init);
module_exit(omap_aic23_exit);
