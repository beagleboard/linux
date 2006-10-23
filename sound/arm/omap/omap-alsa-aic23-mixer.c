/*
 * sound/arm/omap/omap-alsa-aic23-mixer.c
 * 
 * Alsa Driver Mixer for generic codecs for omap boards
 *
 * Copyright (C) 2005 Instituto Nokia de Tecnologia - INdT - Manaus Brazil
 * Written by David Cohen, Daniel Petrini
 *            {david.cohen, daniel.petrini}@indt.org.br
 *
 * Based on es1688_lib.c, 
 * Copyright (c) by Jaroslav Kysela <perex@suse.cz>
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
 * 2005-08-02   INdT Kernel Team - Alsa mixer driver for omap osk. Creation of new 
 *                                 file omap-alsa-mixer.c. Initial version
 *                                 with aic23 codec for osk5912
 */

#include <sound/driver.h>
#include <asm/arch/aic23.h>

#include <asm/arch/omap-alsa.h>
#include "omap-alsa-aic23.h"
#include <sound/initval.h>
#include <sound/control.h>

MODULE_AUTHOR("David Cohen, Daniel Petrini - INdT");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OMAP Alsa mixer driver for ALSA");

/*
 * Codec dependent region
 */

/* Codec AIC23 */
#if defined(CONFIG_SENSORS_TLV320AIC23) || defined (CONFIG_SENSORS_TLV320AIC23_MODULE)

extern void audio_aic23_write(u8, u16);

#define MIXER_NAME		     "Mixer AIC23"
#define SND_OMAP_WRITE(reg, val)     audio_aic23_write(reg, val)

#endif

/* Callback Functions */
#define OMAP_BOOL(xname, xindex, reg, reg_index, mask, invert) \
{ \
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_omap_info_bool, \
	.get = snd_omap_get_bool, \
	.put = snd_omap_put_bool, \
	.private_value = reg | (reg_index << 8) | (invert << 10) | (mask << 12) \
}

#define OMAP_MUX(xname, reg, reg_index, mask) \
{ \
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.info = snd_omap_info_mux, \
	.get = snd_omap_get_mux, \
	.put = snd_omap_put_mux, \
	.private_value = reg | (reg_index << 8) | (mask << 10) \
}

#define OMAP_SINGLE(xname, xindex, reg, reg_index, reg_val, mask) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_omap_info_single, \
	.get = snd_omap_get_single, \
	.put = snd_omap_put_single, \
	.private_value = reg | (reg_val << 8) | (reg_index << 16) | (mask << 18) \
}

#define OMAP_DOUBLE(xname, xindex, left_reg, right_reg, reg_index, mask) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_omap_info_double, \
	.get = snd_omap_get_double, \
	.put = snd_omap_put_double, \
	.private_value = left_reg | (right_reg << 8) | (reg_index << 16) | (mask << 18) \
}

/* Local Registers */
enum snd_device_index {
	PCM_INDEX = 0,
	LINE_INDEX,
	AAC_INDEX, /* Analog Audio Control: reg = l_reg */
};

struct {
	u16 l_reg;
	u16 r_reg;
	u8 sw;
} omap_regs[3];

#ifdef CONFIG_PM
struct {
	u16 l_reg;
	u16 r_reg;
	u8 sw;
} omap_pm_regs[3];
#endif

u16 snd_sidetone[6] = {
	SIDETONE_18,
	SIDETONE_12,
	SIDETONE_9,
	SIDETONE_6,
	SIDETONE_0,
	0
};

/* Begin Bool Functions */

static int snd_omap_info_bool(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t * uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	
	return 0;
}

static int snd_omap_get_bool(snd_kcontrol_t * kcontrol, snd_ctl_elem_value_t * ucontrol)
{
	int mic_index = (kcontrol->private_value >> 8) & 0x03;
	u16 mask = (kcontrol->private_value >> 12) & 0xff;
	int invert = (kcontrol->private_value >> 10) & 0x03;
	
	if (invert)
		ucontrol->value.integer.value[0] = (omap_regs[mic_index].l_reg & mask) ? 0 : 1;
	else
		ucontrol->value.integer.value[0] = (omap_regs[mic_index].l_reg & mask) ? 1 : 0;
	
	return 0;
}

static int snd_omap_put_bool(snd_kcontrol_t * kcontrol, snd_ctl_elem_value_t * ucontrol)
{
	int mic_index = (kcontrol->private_value >> 8) & 0x03;
	u16 mask = (kcontrol->private_value >> 12) & 0xff;
	u16 reg = kcontrol->private_value & 0xff;
	int invert = (kcontrol->private_value >> 10) & 0x03;
	
	int changed = 1;

	if (ucontrol->value.integer.value[0]) /* XOR */
		if (invert)
			omap_regs[mic_index].l_reg &= ~mask;
		else
			omap_regs[mic_index].l_reg |= mask;
	else
		if (invert)
			omap_regs[mic_index].l_reg |= mask;
		else
			omap_regs[mic_index].l_reg &= ~mask;
		
	SND_OMAP_WRITE(reg, omap_regs[mic_index].l_reg);
	
	return changed;
}

/* End Bool Functions */

/* Begin Mux Functions */

static int snd_omap_info_mux(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t * uinfo)
{
	/* Mic = 0
	 * Line = 1 */
	static char *texts[2] =	{ "Mic", "Line"	};

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;
	
	if (uinfo->value.enumerated.item > 1)
		uinfo->value.enumerated.item = 1;
	
	strcpy(uinfo->value.enumerated.name, texts[uinfo->value.enumerated.item]);
	
	return 0;
}

static int snd_omap_get_mux(snd_kcontrol_t * kcontrol, snd_ctl_elem_value_t * ucontrol)
{
	u16 mask = (kcontrol->private_value >> 10) & 0xff;
	int mux_index = (kcontrol->private_value >> 8) & 0x03;

	ucontrol->value.enumerated.item[0] = (omap_regs[mux_index].l_reg & mask) ? 0 /* Mic */ : 1 /* Line */;
	
	return 0;
}

static int snd_omap_put_mux(snd_kcontrol_t * kcontrol, snd_ctl_elem_value_t * ucontrol)
{
	u16 reg = kcontrol->private_value & 0xff;
	u16 mask = (kcontrol->private_value >> 10) & 0xff;
	int mux_index = (kcontrol->private_value >> 8) & 0x03;
	
	int changed = 1;

	if (!ucontrol->value.integer.value[0])
		omap_regs[mux_index].l_reg |= mask; /* AIC23: Mic */
	else
		omap_regs[mux_index].l_reg &= ~mask; /* AIC23: Line */
	
	SND_OMAP_WRITE(reg, omap_regs[mux_index].l_reg);
	
	return changed;
}

/* End Mux Functions */

/* Begin Single Functions */

static int snd_omap_info_single(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t * uinfo)
{
	int mask = (kcontrol->private_value >> 18) & 0xff;
	int reg_val = (kcontrol->private_value >> 8) & 0xff;
	
	uinfo->type = mask ? SNDRV_CTL_ELEM_TYPE_INTEGER : SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = reg_val-1;
	
	return 0;
}

static int snd_omap_get_single(snd_kcontrol_t * kcontrol, snd_ctl_elem_value_t * ucontrol)
{
	u16 reg_val = (kcontrol->private_value >> 8) & 0xff;

	ucontrol->value.integer.value[0] = snd_sidetone[reg_val];
	
	return 0;
}

static int snd_omap_put_single(snd_kcontrol_t * kcontrol, snd_ctl_elem_value_t * ucontrol)
{
	u16 reg_index = (kcontrol->private_value >> 16) & 0x03;
	u16 mask = (kcontrol->private_value >> 18) & 0x1ff;
	u16 reg = kcontrol->private_value & 0xff;
	u16 reg_val = (kcontrol->private_value >> 8) & 0xff;

	int changed = 0;

	/* Volume */
	if ((omap_regs[reg_index].l_reg != (ucontrol->value.integer.value[0] & mask)))
	{
		changed = 1;
	
		omap_regs[reg_index].l_reg &= ~mask;
		omap_regs[reg_index].l_reg |= snd_sidetone[ucontrol->value.integer.value[0]];

		snd_sidetone[reg_val] = ucontrol->value.integer.value[0];
		SND_OMAP_WRITE(reg, omap_regs[reg_index].l_reg);
	}
	else
		changed = 0;
	
	return changed;
}

/* End Single Functions */

/* Begin Double Functions */

static int snd_omap_info_double(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t * uinfo)
{
	/* mask == 0 : Switch
	 * mask != 0 : Volume */
	int mask = (kcontrol->private_value >> 18) & 0xff;

	uinfo->type = mask ? SNDRV_CTL_ELEM_TYPE_INTEGER : SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = mask ? 2 : 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = mask ? mask : 1;
	
	return 0;
}

static int snd_omap_get_double(snd_kcontrol_t * kcontrol, snd_ctl_elem_value_t * ucontrol)
{
	/* mask == 0 : Switch
	 * mask != 0 : Volume */
	int mask = (kcontrol->private_value >> 18) & 0xff;
	int vol_index = (kcontrol->private_value >> 16) & 0x03;
	
	if (!mask)
		/* Switch */
		ucontrol->value.integer.value[0] = omap_regs[vol_index].sw;
	else
	{
		/* Volume */
		ucontrol->value.integer.value[0] = omap_regs[vol_index].l_reg;
		ucontrol->value.integer.value[1] = omap_regs[vol_index].r_reg;
	}

	return 0;
}

static int snd_omap_put_double(snd_kcontrol_t * kcontrol, snd_ctl_elem_value_t * ucontrol)
{
	/* mask == 0 : Switch
	 * mask != 0 : Volume */
	int vol_index = (kcontrol->private_value >> 16) & 0x03;
	int mask = (kcontrol->private_value >> 18) & 0xff;
	int left_reg = kcontrol->private_value & 0xff;
	int right_reg = (kcontrol->private_value >> 8) & 0xff;

	int changed = 0;

	if (!mask)
	{
		/* Switch */
		if (!ucontrol->value.integer.value[0])
		{
			SND_OMAP_WRITE(left_reg, 0x00);
			SND_OMAP_WRITE(right_reg, 0x00);
		}
		else
		{
			SND_OMAP_WRITE(left_reg, omap_regs[vol_index].l_reg);
			SND_OMAP_WRITE(right_reg, omap_regs[vol_index].r_reg);
		}
		changed = 1;
		omap_regs[vol_index].sw = ucontrol->value.integer.value[0]; 
	}
	else
	{
		/* Volume */
		if ((omap_regs[vol_index].l_reg != (ucontrol->value.integer.value[0] & mask)) ||
		    (omap_regs[vol_index].r_reg != (ucontrol->value.integer.value[1] & mask)))
		{
			changed = 1;
		
			omap_regs[vol_index].l_reg &= ~mask;
			omap_regs[vol_index].r_reg &= ~mask;
			omap_regs[vol_index].l_reg |= (ucontrol->value.integer.value[0] & mask);
			omap_regs[vol_index].r_reg |= (ucontrol->value.integer.value[1] & mask);
			if (omap_regs[vol_index].sw)
			{
				/* write to registers only if sw is actived */
				SND_OMAP_WRITE(left_reg, omap_regs[vol_index].l_reg);
				SND_OMAP_WRITE(right_reg, omap_regs[vol_index].r_reg);
			}
		}
		else
			changed = 0;
	}
	
	return changed;
}

/* End Double Functions */

static snd_kcontrol_new_t snd_omap_controls[] = {
	OMAP_DOUBLE("PCM Playback Switch", 0, LEFT_CHANNEL_VOLUME_ADDR, RIGHT_CHANNEL_VOLUME_ADDR,
		     PCM_INDEX, 0x00),
	OMAP_DOUBLE("PCM Playback Volume", 0, LEFT_CHANNEL_VOLUME_ADDR, RIGHT_CHANNEL_VOLUME_ADDR,
		     PCM_INDEX, OUTPUT_VOLUME_MASK),
	OMAP_BOOL("Line Playback Switch", 0, ANALOG_AUDIO_CONTROL_ADDR, AAC_INDEX, BYPASS_ON, 0),
	OMAP_DOUBLE("Line Capture Switch", 0, LEFT_LINE_VOLUME_ADDR, RIGHT_LINE_VOLUME_ADDR,
		     LINE_INDEX, 0x00),
	OMAP_DOUBLE("Line Capture Volume", 0, LEFT_LINE_VOLUME_ADDR, RIGHT_LINE_VOLUME_ADDR,
		     LINE_INDEX, INPUT_VOLUME_MASK),	
	OMAP_BOOL("Mic Playback Switch", 0, ANALOG_AUDIO_CONTROL_ADDR, AAC_INDEX, STE_ENABLED, 0),	
	OMAP_SINGLE("Mic Playback Volume", 0, ANALOG_AUDIO_CONTROL_ADDR, AAC_INDEX, 5, SIDETONE_MASK),
	OMAP_BOOL("Mic Capture Switch", 0, ANALOG_AUDIO_CONTROL_ADDR, AAC_INDEX, MICM_MUTED, 1),
	OMAP_BOOL("Mic Booster Playback Switch", 0, ANALOG_AUDIO_CONTROL_ADDR, AAC_INDEX, MICB_20DB, 0),
	OMAP_MUX("Capture Source", ANALOG_AUDIO_CONTROL_ADDR, AAC_INDEX, INSEL_MIC),
};

#ifdef CONFIG_PM

void snd_omap_suspend_mixer(void)
{
	/* Saves current values to wake-up correctly */
	omap_pm_regs[LINE_INDEX].l_reg = omap_regs[LINE_INDEX].l_reg;
	omap_pm_regs[LINE_INDEX].r_reg = omap_regs[LINE_INDEX].l_reg;
	omap_pm_regs[LINE_INDEX].sw = omap_regs[LINE_INDEX].sw;
	
	omap_pm_regs[AAC_INDEX].l_reg = omap_regs[AAC_INDEX].l_reg;
	
	omap_pm_regs[PCM_INDEX].l_reg = omap_regs[PCM_INDEX].l_reg;
	omap_pm_regs[PCM_INDEX].r_reg = omap_regs[PCM_INDEX].r_reg;
	omap_pm_regs[PCM_INDEX].sw = omap_regs[PCM_INDEX].sw;
}

void snd_omap_resume_mixer(void)
{
	/* Line's saved values */
	omap_regs[LINE_INDEX].l_reg = omap_pm_regs[LINE_INDEX].l_reg;
	omap_regs[LINE_INDEX].r_reg = omap_pm_regs[LINE_INDEX].l_reg;
	omap_regs[LINE_INDEX].sw = omap_pm_regs[LINE_INDEX].sw;
	SND_OMAP_WRITE(LEFT_LINE_VOLUME_ADDR, omap_pm_regs[LINE_INDEX].l_reg);
	SND_OMAP_WRITE(RIGHT_LINE_VOLUME_ADDR, omap_pm_regs[LINE_INDEX].l_reg);
	
	/* Analog Audio Control's saved values */
	omap_regs[AAC_INDEX].l_reg = omap_pm_regs[AAC_INDEX].l_reg;
	SND_OMAP_WRITE(ANALOG_AUDIO_CONTROL_ADDR, omap_regs[AAC_INDEX].l_reg);
	
	/* Headphone's saved values */
	omap_regs[PCM_INDEX].l_reg = omap_pm_regs[PCM_INDEX].l_reg;
	omap_regs[PCM_INDEX].r_reg = omap_pm_regs[PCM_INDEX].r_reg;
	omap_regs[PCM_INDEX].sw = omap_pm_regs[PCM_INDEX].sw;
	SND_OMAP_WRITE(LEFT_CHANNEL_VOLUME_ADDR, omap_pm_regs[PCM_INDEX].l_reg);
	SND_OMAP_WRITE(RIGHT_CHANNEL_VOLUME_ADDR, omap_pm_regs[PCM_INDEX].r_reg);
}
#endif

void snd_omap_init_mixer(void)
{
	u16 vol_reg;

	/* Line's default values */
	omap_regs[LINE_INDEX].l_reg = DEFAULT_INPUT_VOLUME & INPUT_VOLUME_MASK;
	omap_regs[LINE_INDEX].r_reg = DEFAULT_INPUT_VOLUME & INPUT_VOLUME_MASK;
	omap_regs[LINE_INDEX].sw = 0;
	SND_OMAP_WRITE(LEFT_LINE_VOLUME_ADDR, DEFAULT_INPUT_VOLUME & INPUT_VOLUME_MASK);
	SND_OMAP_WRITE(RIGHT_LINE_VOLUME_ADDR, DEFAULT_INPUT_VOLUME & INPUT_VOLUME_MASK);
	
	/* Analog Audio Control's default values */
	omap_regs[AAC_INDEX].l_reg = DEFAULT_ANALOG_AUDIO_CONTROL;
	
	/* Headphone's default values */
	vol_reg = LZC_ON;
	vol_reg &= ~OUTPUT_VOLUME_MASK;
	vol_reg |= DEFAULT_OUTPUT_VOLUME;
	omap_regs[PCM_INDEX].l_reg = DEFAULT_OUTPUT_VOLUME;
	omap_regs[PCM_INDEX].r_reg = DEFAULT_OUTPUT_VOLUME;
	omap_regs[PCM_INDEX].sw = 1;
	SND_OMAP_WRITE(LEFT_CHANNEL_VOLUME_ADDR, vol_reg);
	SND_OMAP_WRITE(RIGHT_CHANNEL_VOLUME_ADDR, vol_reg);
}

int snd_omap_mixer(struct snd_card_omap_codec *chip)
{
	snd_card_t *card;
	unsigned int idx;
	int err;

	snd_assert(chip != NULL && chip->card != NULL, return -EINVAL);

	card = chip->card;

	strcpy(card->mixername, MIXER_NAME);

	/* Registering alsa mixer controls */
	for (idx = 0; idx < ARRAY_SIZE(snd_omap_controls); idx++) 
		if ((err = snd_ctl_add(card, snd_ctl_new1(&snd_omap_controls[idx], chip))) < 0)
			return err;

	return 0;
}
