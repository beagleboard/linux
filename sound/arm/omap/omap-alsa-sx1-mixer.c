/*
 * sound/arm/omap/omap-alsa-sx1-mixer.c
 *
 * Alsa codec Driver for Siemens SX1 board.
 * based on omap-alsa-tsc2101-mixer.c
 *
 *  Copyright (C) 2006 Vladimir Ananiev (vovan888 at gmail com)
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

#include "omap-alsa-sx1.h"
#include "omap-alsa-sx1-mixer.h"

#include <linux/types.h>
#include <sound/initval.h>
#include <sound/control.h>

static int current_playback_target	= PLAYBACK_TARGET_LOUDSPEAKER;
static int current_rec_src 		= REC_SRC_SINGLE_ENDED_MICIN_HED;
static int current_volume;	/* current volume, we cant read it */
static int current_fm_volume;	/* current FM radio volume, we cant read it */

/*
 * Select SX1 recording source.
 */
static void set_record_source(int val)
{
	/* TODO Recording is done on McBSP2 and Mic only */
	current_rec_src	= val;
}

static int set_mixer_volume(int mixer_vol)
{
	int ret, i;
	if ((mixer_vol < 0) || (mixer_vol > 9)) {
		printk(KERN_ERR "Trying a bad mixer volume (%d)!\n", mixer_vol);
		return -EPERM;
	}
	ret = (current_volume != mixer_vol);
	current_volume = mixer_vol; /* set current volume, we cant read it */

	i = cn_sx1snd_send(DAC_VOLUME_UPDATE, mixer_vol, 0);
	if (i)
		return i;
	return ret;
}

static void set_loudspeaker_to_playback_target(void)
{
	/* TODO */
	cn_sx1snd_send(DAC_SETAUDIODEVICE, SX1_DEVICE_SPEAKER, 0);

	current_playback_target	= PLAYBACK_TARGET_LOUDSPEAKER;
}

static void set_headphone_to_playback_target(void)
{
	/* TODO */
	cn_sx1snd_send(DAC_SETAUDIODEVICE, SX1_DEVICE_HEADPHONE, 0);

	current_playback_target	= PLAYBACK_TARGET_HEADPHONE;
}

static void set_telephone_to_playback_target(void)
{
	/* TODO */
	cn_sx1snd_send(DAC_SETAUDIODEVICE, SX1_DEVICE_PHONE, 0);

	current_playback_target	= PLAYBACK_TARGET_CELLPHONE;
}

static void set_telephone_to_record_source(void)
{
	cn_sx1snd_send(DAC_SETAUDIODEVICE, SX1_DEVICE_PHONE, 0);
}

static void init_playback_targets(void)
{
	set_loudspeaker_to_playback_target();
	set_mixer_volume(DEFAULT_OUTPUT_VOLUME);
}

/*
 * Initializes SX1 record source (to mic) and playback target (to loudspeaker)
 */
void snd_omap_init_mixer(void)
{
	/* Select headset to record source */
	set_record_source(REC_SRC_SINGLE_ENDED_MICIN_HED);
	/* Init loudspeaker as a default playback target*/
	init_playback_targets();
}

/* ---------------------------------------------------------------------- */
static int pcm_playback_target_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	static char *texts[PLAYBACK_TARGET_COUNT] = {
		"Loudspeaker", "Headphone", "Cellphone"
	};

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = PLAYBACK_TARGET_COUNT;
	if (uinfo->value.enumerated.item > PLAYBACK_TARGET_COUNT - 1) {
		uinfo->value.enumerated.item = PLAYBACK_TARGET_COUNT - 1;
	}
	strcpy(uinfo->value.enumerated.name,
			texts[uinfo->value.enumerated.item]);
	return 0;
}

static int pcm_playback_target_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = current_playback_target;
	return 0;
}

static int pcm_playback_target_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int ret_val = 0;
	int cur_val = ucontrol->value.integer.value[0];

	if ((cur_val >= 0) &&
		(cur_val < PLAYBACK_TARGET_COUNT) &&
		(cur_val != current_playback_target)) {
		if (cur_val == PLAYBACK_TARGET_LOUDSPEAKER) {
			set_record_source(REC_SRC_SINGLE_ENDED_MICIN_HED);
			set_loudspeaker_to_playback_target();
 		} else if (cur_val == PLAYBACK_TARGET_HEADPHONE) {
			set_record_source(REC_SRC_SINGLE_ENDED_MICIN_HND);
 			set_headphone_to_playback_target();
 		} else if (cur_val == PLAYBACK_TARGET_CELLPHONE) {
			set_telephone_to_record_source();
			set_telephone_to_playback_target();
		}
		ret_val	= 1;
	}
	return ret_val;
}

/*-----------------------------------------------------------*/
static int pcm_playback_volume_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 9;
	return 0;
}

static int pcm_playback_volume_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = current_volume;
	return 0;
}

static int pcm_playback_volume_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	return set_mixer_volume(ucontrol->value.integer.value[0]);
}

static int pcm_playback_switch_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;
	return 0;
}

static int pcm_playback_switch_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 1;
	return 0;
}

static int pcm_playback_switch_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

/* ----------------------------------------------------------- */

static int headset_playback_volume_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 9;
	return 0;
}

static int headset_playback_volume_get(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]	= current_volume;
	return 0;
}

static int headset_playback_volume_put(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	return set_mixer_volume(ucontrol->value.integer.value[0]);
}

static int headset_playback_switch_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type 			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;
	return 0;
}

static int headset_playback_switch_get(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 1;
	return 0;
}

static int headset_playback_switch_put(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	/* mute/unmute headset */
#if 0
	return adc_pga_unmute_control(ucontrol->value.integer.value[0],
				TSC2101_HEADSET_GAIN_CTRL,
				15);
#endif
	return 0;
}
/* ----------------------------------------------------------- */
static int fmradio_playback_volume_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 9;
	return 0;
}

static int fmradio_playback_volume_get(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = current_fm_volume;
	return 0;
}

static int fmradio_playback_volume_put(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	int ret = current_fm_volume != ucontrol->value.integer.value[0];
	int i;
	current_fm_volume = ucontrol->value.integer.value[0];
	i = cn_sx1snd_send(DAC_FMRADIO_OPEN, current_fm_volume, 0);
	if (i)
		return i;
	return ret;
}

static int fmradio_playback_switch_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type 			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;
	return 0;
}

static int fmradio_playback_switch_get(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 1;
	return 0;
}

static int fmradio_playback_switch_put(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	/* mute/unmute FM radio */
	if (ucontrol->value.integer.value[0])
		cn_sx1snd_send(DAC_FMRADIO_OPEN, current_fm_volume, 0);
	else
		cn_sx1snd_send(DAC_FMRADIO_CLOSE, 0, 0);

	return 0;
}
/* ----------------------------------------------------------- */
static int cellphone_input_switch_info(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_info *uinfo)
{
	uinfo->type 			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;
	return 0;
}

static int cellphone_input_switch_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 1;
	return 0;
}

static int cellphone_input_switch_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
#if 0
	return adc_pga_unmute_control(ucontrol->value.integer.value[0],
				TSC2101_BUZZER_GAIN_CTRL, 15);
#endif
	return 0;
}
/* ----------------------------------------------------------- */

static int buzzer_input_switch_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type 			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;
	return 0;
}

static int buzzer_input_switch_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 1;
	return 0;
}

static int buzzer_input_switch_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
#if 0
	return adc_pga_unmute_control(ucontrol->value.integer.value[0],
				TSC2101_BUZZER_GAIN_CTRL, 6);
#endif
	return 0;
}
/*-----------------------------------------------------------*/

static struct snd_kcontrol_new egold_control[] __devinitdata = {
	{
		.name	= "Playback Playback Route",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= pcm_playback_target_info,
		.get	= pcm_playback_target_get,
		.put	= pcm_playback_target_put,
	}, {
		.name	= "Master Playback Volume",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= pcm_playback_volume_info,
		.get	= pcm_playback_volume_get,
		.put	= pcm_playback_volume_put,
	}, {
		.name	= "Master Playback Switch",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= pcm_playback_switch_info,
		.get	= pcm_playback_switch_get,
		.put	= pcm_playback_switch_put,
	}, {
		.name	= "Headset Playback Volume",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 1,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= headset_playback_volume_info,
		.get	= headset_playback_volume_get,
		.put	= headset_playback_volume_put,
	}, {
		.name	= "Headset Playback Switch",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 1,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= headset_playback_switch_info,
		.get	= headset_playback_switch_get,
		.put	= headset_playback_switch_put,
	}, {
		.name	= "FM Playback Volume",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 2,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= fmradio_playback_volume_info,
		.get	= fmradio_playback_volume_get,
		.put	= fmradio_playback_volume_put,
	}, {
		.name	= "FM Playback Switch",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 2,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= fmradio_playback_switch_info,
		.get	= fmradio_playback_switch_get,
		.put	= fmradio_playback_switch_put,
	}, {
		.name	= "Cellphone Input Switch",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= cellphone_input_switch_info,
		.get	= cellphone_input_switch_get,
		.put	= cellphone_input_switch_put,
	}, {
		.name	= "Buzzer Input Switch",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= buzzer_input_switch_info,
		.get	= buzzer_input_switch_get,
		.put	= buzzer_input_switch_put,
	}
};

#ifdef CONFIG_PM
void snd_omap_suspend_mixer(void)
{
}

void snd_omap_resume_mixer(void)
{
	snd_omap_init_mixer();
}
#endif

int snd_omap_mixer(struct snd_card_omap_codec *egold)
{
	int i = 0;
	int err = 0;

	if (!egold)
		return -EINVAL;

	for (i=0; i < ARRAY_SIZE(egold_control); i++) {
		err = snd_ctl_add(egold->card,
				snd_ctl_new1(&egold_control[i], egold->card));
		if (err < 0)
			return err;
	}
	return 0;
}
