/*
 * sound/arm/omap/omap-alsa-tsc2102-mixer.c
 *
 * Alsa mixer driver for TSC2102 chip for OMAP platforms.
 *
 * Copyright (c) 2006 Andrzej Zaborowski  <balrog@zabor.org>
 * Code based on the TSC2101 ALSA driver.
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
 */

#include <linux/types.h>
#include <linux/spi/tsc2102.h>

#include <asm/arch/omap-alsa.h>

#include <sound/driver.h>
#include <sound/initval.h>
#include <sound/control.h>

#include "omap-alsa-tsc2102.h"
#include "omap-alsa-dma.h"

static int vol[2], mute[2], filter[2];

/*
 * Converts the Alsa mixer volume (0 - 100) to actual Digital
 * Gain Control (DGC) value that can be written or read from the
 * TSC2102 registers.
 *
 * Note that the number "OUTPUT_VOLUME_MAX" is smaller than
 * OUTPUT_VOLUME_MIN because DGC works as a volume decreaser.  (The
 * higher the value sent to DAC, the more the volume of controlled
 * channel is decreased)
 */
static void set_dac_gain_stereo(int left_ch, int right_ch)
{
	int lch, rch;

	if (left_ch > 100)
		vol[0] = 100;
	else if (left_ch < 0)
		vol[0] = 0;
	else
		vol[0] = left_ch;
	lch = OUTPUT_VOLUME_MIN - vol[0] *
		(OUTPUT_VOLUME_MIN - OUTPUT_VOLUME_MAX) / 100;

	if (right_ch > 100)
		vol[1] = 100;
	else if (right_ch < 0)
		vol[1] = 0;
	else
		vol[1] = right_ch;
	rch = OUTPUT_VOLUME_MIN - vol[1] *
		(OUTPUT_VOLUME_MIN - OUTPUT_VOLUME_MAX) / 100;

	tsc2102_set_volume(lch, rch);
}

void init_playback_targets(void)
{
	set_dac_gain_stereo(DEFAULT_OUTPUT_VOLUME, DEFAULT_OUTPUT_VOLUME);

	/* Unmute */
	tsc2102_set_mute(0, 0);

	mute[0] = mute[1] = 0;
	filter[0] = filter[1] = 0;
}

/*
 * Initializes TSC 2102 and playback target.
 */
void snd_omap_init_mixer(void)
{
	FN_IN;

	init_playback_targets();

	FN_OUT(0);
}

static int __pcm_playback_volume_info(
		snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 2;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 100;
	return 0;
}

static int __pcm_playback_volume_get(
		snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
	ucontrol->value.integer.value[0] = vol[0];	/* L */
	ucontrol->value.integer.value[1] = vol[1];	/* R */

	return 0;
}

static int __pcm_playback_volume_put(
		snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
	set_dac_gain_stereo(
			ucontrol->value.integer.value[0],	/* L */
			ucontrol->value.integer.value[1]);	/* R */
	return 1;
}

static int __pcm_playback_switch_info(
		snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 2;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;
	return 0;
}

static int __pcm_playback_switch_get(
		snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
	ucontrol->value.integer.value[0] = !mute[0];		/* L */
	ucontrol->value.integer.value[1] = !mute[1];		/* R */

	return 0;
}

static int __pcm_playback_switch_put(
		snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	mute[0] = (ucontrol->value.integer.value[0] == 0);	/* L */
	mute[1] = (ucontrol->value.integer.value[1] == 0);	/* R */

	tsc2102_set_mute(mute[0], mute[1]);
	return 1;
}

static int __pcm_playback_deemphasis_info(
		snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;
	return 0;
}

static int __pcm_playback_deemphasis_get(
		snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
	ucontrol->value.integer.value[0] = filter[0];
	return 0;
}

static int __pcm_playback_deemphasis_put(
		snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	filter[0] = (ucontrol->value.integer.value[0] > 0);

	tsc2102_set_deemphasis(filter[0]);
	return 1;
}

static int __pcm_playback_bassboost_info(
		snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;
	return 0;
}

static int __pcm_playback_bassboost_get(
		snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
	ucontrol->value.integer.value[0] = filter[1];
	return 0;
}

static int __pcm_playback_bassboost_put(
		snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	filter[1] = (ucontrol->value.integer.value[0] > 0);

	tsc2102_set_bassboost(filter[1]);
	return 1;
}

static snd_kcontrol_new_t tsc2102_control[] __devinitdata = {
	{
		.name	= "Master Playback Volume",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= __pcm_playback_volume_info,
		.get	= __pcm_playback_volume_get,
		.put	= __pcm_playback_volume_put,
	},
	{
		.name	= "Master Playback Switch",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= __pcm_playback_switch_info,
		.get	= __pcm_playback_switch_get,
		.put	= __pcm_playback_switch_put,
	},
	{
		.name	= "De-emphasis Filter Switch",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= __pcm_playback_deemphasis_info,
		.get	= __pcm_playback_deemphasis_get,
		.put	= __pcm_playback_deemphasis_put,
	},
	{
		.name	= "Bass-boost Filter Switch",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= __pcm_playback_bassboost_info,
		.get	= __pcm_playback_bassboost_get,
		.put	= __pcm_playback_bassboost_put,
	},
};

#ifdef CONFIG_PM
void snd_omap_suspend_mixer(void)
{
	/* Nothing to do */
}

void snd_omap_resume_mixer(void)
{
	/* The chip was reset, restore the last used values */
	set_dac_gain_stereo(vol[0], vol[1]);

	tsc2102_set_mute(mute[0], mute[1]);
	tsc2102_set_deemphasis(filter[0]);
	tsc2102_set_bassboost(filter[1]);
}
#endif

int snd_omap_mixer(struct snd_card_omap_codec *tsc2102)
{
	int i, err;

	if (!tsc2102)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(tsc2102_control); i ++) {
		err = snd_ctl_add(tsc2102->card,
				snd_ctl_new1(&tsc2102_control[i],
				tsc2102->card));

		if (err < 0)
			return err;
	}
	return 0;
}
