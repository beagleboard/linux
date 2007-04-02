/*
 * ALSA Mixer implementation for TSC2301
 *
 * Copyright (C) 2006 Nokia Corporation.
 *
 * Contact: Jarkko Nikula <jarkko.nikula@nokia.com>
 *          Juha Yrjola
 *
 * Some notes about TSC2301:
 * - PLL will stop when DAC and ADC's are powered down.
 * - Touchscreen will stop working when audio part is powered up and if audio
 *   MCLK is stopped. Problem is avoided if audio is powered down before
 *   stopping MCLK.
 * - Audio DAC or audio outputs will activate only after 100 msec from the
 *   chip power-up. Reason seems to be VCM since there is no this delay if the
 *   chip and VCM (bit AVPD on PD/MISC) were not powered down. The chip will
 *   consume about 1 mA if all other audio blocks are powered down except the
 *   chip itself and VCM. Full power down consumes only about few uA.
 * - Power-down transition could happen earliest about 100 msec after the chip
 *   power-up. Otherwise power-down will fail if there is no that 100 msec
 *   on time before it. It's not obvious why is that since chip reports
 *   power-up to be completed and also PLL output on GPIO_0 is active in few
 *   milliseconds.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2301.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/control.h>

/* shadow register indexes */
enum {
	/* audio control and volume registers */
	AUDCNTL_INDEX,
	ADCVOL_INDEX,
	DACVOL_INDEX,
	BPVOL_INDEX,
	/* keyclick control register (not needed here) */
	/* audio power control register */
	PD_MISC_INDEX,
	/* TSC2301 GPIO control register */
	GPIO_INDEX,

	SHADOW_REG_COUNT,
};

/* structure for driver private data */
struct tsc2301_mixer {
	struct tsc2301 *tsc;
	struct mutex mutex;

	/* shadow registers holding TSC2301 audio registers. Used to hold
	 * their states during the sleep and also to reduce communication with
	 * the chip since get callback functions could get register values
	 * directly from these shadow registers without needing to read them
	 * from the chip */
	u16 shadow_regs[SHADOW_REG_COUNT];

	/* audio controller driver usage of the ADC and DAC */
	unsigned adc_enabled:1, dac_enabled:1;
	unsigned pll_output:1;
	unsigned mclk_enabled;

	/* latest audio power-up timestamp */
	unsigned long pu_jiffies;

	/* these are used when upper layer(s) are going to power-down TSC2301
	 * before 100 msec is passed from power-up */
	struct delayed_work delayed_power_down;
	unsigned delayed_pd_active:1;

	int (* platform_init)(struct device *);
	void (* platform_cleanup)(struct device *);

	struct tsc2301_mixer_gpio *mixer_gpios;
	int n_mixer_gpios;
};

#define TSC2301_DAC_DELAY		msecs_to_jiffies(100)
#define TSC2301_MIN_PU_PERIOD		msecs_to_jiffies(100)

#define TSC2301_REG_TO_PVAL(reg)	\
	(TSC2301_REG_TO_PAGE(reg) << 6 | TSC2301_REG_TO_ADDR(reg))
#define  TSC2301_PVAL_TO_REG(v)		\
	(TSC2301_REG((((v) >> 6) & 3),((v) & 0x1f)))

#define TSC2301_VOLUME_MASK		0x7f
#define TSC2301_MIN_ADCVOL		6
#define TSC2301_MIN_DACVOL		0
#define TSC2301_MIN_BPVOL		31
#define TSC2301_MUTE_LEFT_SHIFT		15
#define TSC2301_VOL_LEFT_SHIFT		8
#define TSC2301_MUTE_RIGHT_SHIFT	7
#define TSC2301_VOL_RIGHT_SHIFT		0

#define TSC2301_INM_MASK		3
#define TSC2301_INML_SHIFT		12
#define TSC2301_INMR_SHIFT		10

#define TSC2301_MICG_MASK		3
#define TSC2301_MICG_MIN		1 /* values 0 & 1 both mean 0 dB */
#define TSC2301_MICG_SHIFT		8

#define TSC2301_REG_AUDCNTL_MCLK(v)	(((v) & 3) << 6)
#define TSC2301_REG_AUDCNTL_I2SFS(v)	(((v) & 0xf) << 2)
#define TSC2301_REG_AUDCNTL_I2SFM(v)	(((v) & 3) << 0)

#define TSC2301_SINGLE(xname, xindex, reg, shadow_index, shift, mask, min) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_tsc2301_info_single, \
	.get = snd_tsc2301_get_single, \
	.put = snd_tsc2301_put_single, \
	.private_value = TSC2301_REG_TO_PVAL(reg) | \
		(shadow_index << 8) | (shift << 16) | (mask << 24) | \
		(min << 28) \
}
#define TSC2301_SINGLE_MINVAL(v)	(((v) >> 28) & 15)
#define TSC2301_SINGLE_SHIFT(v)		(((v) >> 16) & 15)
#define TSC2301_SINGLE_MASK(v)		(((v) >> 24) & 15)

#define TSC2301_DOUBLE(xname, xindex, reg, shadow_index, ls, rs, mask, min) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_tsc2301_info_double, \
	.get = snd_tsc2301_get_double, \
	.put = snd_tsc2301_put_double, \
	.private_value = TSC2301_REG_TO_PVAL(reg) | \
		(shadow_index << 8) | (min << 11) | \
		(ls << 16) | (rs << 20) | (mask << 24) \
}
#define TSC2301_DOUBLE_MINVAL(v)	(((v) >> 11) & 0x1f)
#define TSC2301_DOUBLE_LEFT_SHIFT(v)	(((v) >> 16) & 15)
#define TSC2301_DOUBLE_RIGHT_SHIFT(v)	(((v) >> 20) & 15)
#define TSC2301_DOUBLE_MASK(v)		(((v) >> 24) & TSC2301_VOLUME_MASK)

#define TSC2301_MUX(xname, xindex, reg, shadow_index, ls, rs, mask) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_tsc2301_info_mux, \
	.get = snd_tsc2301_get_mux, \
	.put = snd_tsc2301_put_mux, \
	.private_value = TSC2301_REG_TO_PVAL(reg) | \
		(shadow_index << 8) | (ls << 16) | (rs << 20) | (mask << 24) \
}
#define TSC2301_MUX_LEFT_SHIFT(v)	(((v) >> 16) & 15)
#define TSC2301_MUX_RIGHT_SHIFT(v)	(((v) >> 20) & 15)
#define TSC2301_MUX_MASK(v)		(((v) >> 24) & TSC2301_VOLUME_MASK)

#define TSC2301_BOOL(xname, xindex, reg, shadow_index, shift, invert, state) \
{ \
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_tsc2301_info_bool, \
	.get = snd_tsc2301_get_bool, \
	.put = snd_tsc2301_put_bool, \
	.private_value = TSC2301_REG_TO_PVAL(reg) | \
		(shadow_index << 8) | (shift << 16) | \
		(invert << 24) | (state << 25) \
}
#define TSC2301_BOOL_SHIFT(v)		(((v) >> 16) & 7)
#define TSC2301_BOOL_INVERT(v)		(((v) >> 24) & 1)
#define TSC2301_BOOL_STATE(v)		(((v) >> 25) & 1)

#define TSC2301_SHADOW_INDEX(v)		(((v) >> 8) & 7)

/*
 * Power-down handler for additional GPIO mixer controls. GPIO state of GPIO
 * controls whose power-down flag is enabled are set to their false/deactivate
 * state
 *
 * Must be called tsc->mixer->mutex locked
 */
static void tsc2301_gpio_power_down(struct tsc2301 *tsc)
{
	struct tsc2301_mixer *mix = tsc->mixer;
	u16 temp;
	int i;

	temp = mix->shadow_regs[GPIO_INDEX];
	for (i = 0; i < mix->n_mixer_gpios; i++) {
		const struct tsc2301_mixer_gpio *mg;

		mg = mix->mixer_gpios + i;
		if (mg->deactivate_on_pd) {
			int gpio = mg->gpio;

			temp &= ~(1 << gpio);
			temp |= mg->inverted << gpio;
		}
	}
	tsc2301_write_reg(tsc, TSC2301_REG_GPIO, temp);
}

/*
 * Powers down/up audio blocks which are muted or become unused.
 * shadow_index >= 0, changes power state of single audio block
 * shadow_index < 0, changes power state of all blocks
 *
 * Must be called tsc->mixer->mutex locked
 */
#define TSC2301_MUTE_MASK \
	((1 << TSC2301_MUTE_LEFT_SHIFT) | (1 << TSC2301_MUTE_RIGHT_SHIFT))
static void tsc2301_power_ctrl(struct tsc2301 *tsc, int shadow_index,
			       int poll_pdsts)
{
	struct tsc2301_mixer *mix = tsc->mixer;
	u16 pd_ctrl, pd_ctrl_old, w;
	unsigned long timeout;
	int power_up = 0;

	if (mix->delayed_pd_active) {
		mix->delayed_pd_active = 0;
		mix->mclk_enabled--;
		cancel_delayed_work(&mix->delayed_power_down);
	}

	pd_ctrl = pd_ctrl_old = mix->shadow_regs[PD_MISC_INDEX];
	/* power control helper based on used space mixer selections. See
	 * actual power control decisions below */
	if (shadow_index < 0 || shadow_index == ADCVOL_INDEX) {
		/* ADC left and right power down control */
		if (mix->shadow_regs[ADCVOL_INDEX] &
		    (1 << TSC2301_MUTE_LEFT_SHIFT))
			/* left ADC muted. Power down the left ADC */
			pd_ctrl |= TSC2301_REG_PD_MISC_ADPDL;
		else
			pd_ctrl &= ~TSC2301_REG_PD_MISC_ADPDL;
		if (mix->shadow_regs[ADCVOL_INDEX] &
		    (1 << TSC2301_MUTE_LEFT_SHIFT))
			/* right ADC muted. Power down the right ADC */
			pd_ctrl |= TSC2301_REG_PD_MISC_ADPDR;
		else
			pd_ctrl &= ~TSC2301_REG_PD_MISC_ADPDR;
	}
	if (shadow_index < 0 || shadow_index == DACVOL_INDEX) {
		/* DAC power down control */
		if ((mix->shadow_regs[DACVOL_INDEX] &
		     TSC2301_MUTE_MASK) == TSC2301_MUTE_MASK)
			/* both DACs muted. Power down the DAC */
			pd_ctrl |= TSC2301_REG_PD_MISC_DAPD;
		else
			pd_ctrl &= ~TSC2301_REG_PD_MISC_DAPD;
	}
	if (shadow_index < 0 || shadow_index == BPVOL_INDEX) {
		/* line bypass power down control */
		if ((mix->shadow_regs[BPVOL_INDEX] &
		     TSC2301_MUTE_MASK) == TSC2301_MUTE_MASK)
			/* both line bypasses muted. Power down the bypass
			 * path */
			pd_ctrl |= TSC2301_REG_PD_MISC_ABPD;
		else
			pd_ctrl &= ~TSC2301_REG_PD_MISC_ABPD;
	}
	if (shadow_index < 0 || shadow_index == AUDCNTL_INDEX) {
		/* mic bias power down control */
		if ((mix->shadow_regs[AUDCNTL_INDEX] &
		     (3 << TSC2301_INML_SHIFT)) &&
		    (mix->shadow_regs[AUDCNTL_INDEX] &
		     (3 << TSC2301_INMR_SHIFT)))
			/* both ADC channels use other than mic input. Power
			 * down the mic bias output */
			pd_ctrl |= TSC2301_REG_PD_MISC_MIBPD;
		else
			pd_ctrl &= ~TSC2301_REG_PD_MISC_MIBPD;
	}

	/* power control decisions based on codec usage and user space mixer
	 * selections detected above */
	pd_ctrl &= ~TSC2301_REG_PD_MISC_APD; /* audio not powered down */
	if (mix->mclk_enabled) {
		if (!mix->adc_enabled) {
			/* ADC not used, power down both ADC's and mic bias
			 * output independently of user space mixer
			 * selections */
			pd_ctrl |= TSC2301_REG_PD_MISC_ADPDL;
			pd_ctrl |= TSC2301_REG_PD_MISC_ADPDR;
			pd_ctrl |= TSC2301_REG_PD_MISC_MIBPD;
		}
		if (!mix->dac_enabled) {
			/* DAC not used, power down DAC independently of user
			 * space mixer selections */
			pd_ctrl |= TSC2301_REG_PD_MISC_DAPD;
		}

		if (mix->pll_output) {
			/* GPIO_0 is configured as PLL output so audio
			 * controller is expecting clock from TSC2301. Either
			 * ADC or DAC must be active in order to keep PLL on */
			if ((pd_ctrl & TSC2301_REG_PD_MISC_ADPDL) &&
			    (pd_ctrl & TSC2301_REG_PD_MISC_ADPDR) &&
			    (pd_ctrl & TSC2301_REG_PD_MISC_DAPD)) {
				/* neither ADC or DAC used. Force ADC on in
				 * order to keep PLL active */
				pd_ctrl &= ~(TSC2301_REG_PD_MISC_ADPDL |
					     TSC2301_REG_PD_MISC_ADPDR);
			}
		}
	} else {
		/* audio input clock is not enabled so power down DAC and ADC
		 * in order to shutdown PLL and to keep touchscreen and keypad
		 * parts working. Touchscreen and keypad use audio clock when
		 * PLL is on and internal clock otherwise */
		pd_ctrl |= TSC2301_REG_PD_MISC_DAPD |
			   TSC2301_REG_PD_MISC_ADPDL |
			   TSC2301_REG_PD_MISC_ADPDR;
	}

	if ((pd_ctrl & TSC2301_REG_PD_MISC_ADPDL) &&
	    (pd_ctrl & TSC2301_REG_PD_MISC_ADPDR) &&
	    (pd_ctrl & TSC2301_REG_PD_MISC_DAPD) &&
	    (pd_ctrl & TSC2301_REG_PD_MISC_ABPD)) {
		/* all ADC, DAC and line bypass path unused. Power down the
		 * whole audio part of the TSC2301 */
		pd_ctrl |= TSC2301_REG_PD_MISC_APD;
	}

	if (pd_ctrl == pd_ctrl_old)
		return;

	/* power down control changed. Update into TSC2301 */
	if ((pd_ctrl ^ pd_ctrl_old) & TSC2301_REG_PD_MISC_APD) {
		/* whole audio power state changed. Update GPIO states */
		if (pd_ctrl & TSC2301_REG_PD_MISC_APD) {
			/* power down GPIO controls before powering down
			 * the codec */
			tsc2301_gpio_power_down(tsc);
			/* we must really ensure that codec has been on no less
			 * than 100 msec before doing power-down */
			timeout = mix->pu_jiffies + TSC2301_MIN_PU_PERIOD -
				  jiffies;
			if (timeout <= TSC2301_MIN_PU_PERIOD) {
				mix->delayed_pd_active = 1;
				mix->mclk_enabled++;
				schedule_delayed_work(&mix->delayed_power_down,
						      timeout + 1);
				return;
			}
		} else
			/* restore GPIOs after codec is powered up */
			power_up = 1;
	}
	mix->shadow_regs[PD_MISC_INDEX] = pd_ctrl;
	tsc2301_write_reg(tsc, TSC2301_REG_PD_MISC, pd_ctrl);
	if (power_up)
		mix->pu_jiffies = jiffies;
	if (!poll_pdsts) {
		if (power_up)
			tsc2301_write_reg(tsc, TSC2301_REG_GPIO,
					  mix->shadow_regs[GPIO_INDEX]);
		return;
	}

	/* wait until power-up/-down is completed */
	timeout = jiffies + msecs_to_jiffies(100);
	w = 0;
	do {
		if (time_after(jiffies, timeout)) {
			/* Print a warning only if the I2S clock is not
			 * present / out of sync. This can happen during
			 * init time, when that clock will be turned on
			 * by another driver like in the OMAP EAC with
			 * external clock case.
			 */
			if (w & TSC2301_REG_PD_MISC_OTSYN) {
				dev_warn(&tsc->spi->dev,
				   "I2S clock not in sync or off.\n");
			} else {
				dev_err(&tsc->spi->dev,
				   "power-up/-down timed out "
				   "(0x%04x, 0x%04x -> 0x%04x)\n",
				   w, pd_ctrl_old, pd_ctrl);
			}
			goto out;
		}
		w = tsc2301_read_reg(tsc, TSC2301_REG_PD_MISC);
	} while (!(w & TSC2301_REG_PD_MISC_PDSTS));

out:
	if (((pd_ctrl ^ pd_ctrl_old) & TSC2301_REG_PD_MISC_DAPD) &&
	    !(pd_ctrl & TSC2301_REG_PD_MISC_DAPD)) {
		/* DAC powered up now. Ensure that DAC and audio outputs are
		 * activated. They are up 100 msec after the chip power-up
		 * command */
		timeout = mix->pu_jiffies + TSC2301_DAC_DELAY - jiffies;
		if (timeout <= TSC2301_DAC_DELAY)
			schedule_timeout_interruptible(timeout);
		/* FIXME: This is lazy. We restore GPIOs only after activating
		 * the DAC. It would be better to do some kind of delayed GPIO
		 * restore. That ensures that we restore them also if only ADC
		 * path is activated. But this is required only if there is
		 * some input amplifier, bias control, etc. and their power
		 * state is under TSC GPIO control */
		tsc2301_write_reg(tsc, TSC2301_REG_GPIO,
				  mix->shadow_regs[GPIO_INDEX]);
	}
}

static int snd_tsc2301_info_single(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	int mask = TSC2301_SINGLE_MASK(kcontrol->private_value);
	int minval = TSC2301_SINGLE_MINVAL(kcontrol->private_value);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = minval;
	uinfo->value.integer.max = mask;

	return 0;
}

static int snd_tsc2301_get_single(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct tsc2301 *tsc = kcontrol->private_data;
	unsigned long priv = kcontrol->private_value;
	int mask = TSC2301_SINGLE_MASK(priv);
	int shift = TSC2301_SINGLE_SHIFT(priv);
	int shadow_index = TSC2301_SHADOW_INDEX(priv);
	u16 shadow_reg;

	shadow_reg = tsc->mixer->shadow_regs[shadow_index];

	ucontrol->value.integer.value[0] = (shadow_reg >> shift) & mask;

	return 0;
}

static int snd_tsc2301_put_single(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct tsc2301 *tsc = kcontrol->private_data;
	unsigned long priv = kcontrol->private_value;
	int mask = TSC2301_SINGLE_MASK(priv);
	int shadow_index = TSC2301_SHADOW_INDEX(priv);
	u16 shadow_reg, shadow_reg_old;
	int shift = TSC2301_SINGLE_SHIFT(priv);
	int reg = TSC2301_PVAL_TO_REG(priv);
	int changed;

	mutex_lock(&tsc->mixer->mutex);
	shadow_reg = shadow_reg_old = tsc->mixer->shadow_regs[shadow_index];

	/* zero bits to be modified */
	shadow_reg &= ~(mask << shift);
	/* modify with new value */
	shadow_reg |= ((ucontrol->value.integer.value[0] & mask) << shift);

	changed = (shadow_reg != shadow_reg_old);
	tsc->mixer->shadow_regs[shadow_index] = shadow_reg;

	/* update into TSC2301 if necessary */
	if (changed)
		tsc2301_write_reg(tsc, reg, shadow_reg);
	mutex_unlock(&tsc->mixer->mutex);

	return changed;
}

static int snd_tsc2301_info_double(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	/* mask == 1 : Switch
	 * mask > 1 : Max volume */
	int mask = TSC2301_DOUBLE_MASK(kcontrol->private_value);
	int minval = TSC2301_DOUBLE_MINVAL(kcontrol->private_value);

	uinfo->type = mask == 1 ? SNDRV_CTL_ELEM_TYPE_BOOLEAN :
		SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = minval;
	uinfo->value.integer.max = mask;

	return 0;
}

static int snd_tsc2301_get_double(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct tsc2301 *tsc = kcontrol->private_data;
	unsigned long priv = kcontrol->private_value;
	/* mask == 1 : Switch
	 * mask > 1 : Volume */
	int mask = TSC2301_DOUBLE_MASK(priv);
	int ls = TSC2301_DOUBLE_LEFT_SHIFT(priv);
	int rs = TSC2301_DOUBLE_RIGHT_SHIFT(priv);
	int shadow_index = TSC2301_SHADOW_INDEX(priv);
	u16 shadow_reg;

	shadow_reg = tsc->mixer->shadow_regs[shadow_index];

	/* invert mute bits for the switches */
	if (mask == 1)
		shadow_reg = ~shadow_reg;

	ucontrol->value.integer.value[0] = (shadow_reg >> ls) & mask;
	ucontrol->value.integer.value[1] = (shadow_reg >> rs) & mask;

	return 0;
}

static int snd_tsc2301_put_double(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct tsc2301 *tsc = kcontrol->private_data;
	unsigned long priv = kcontrol->private_value;
	/* mask == 1 : Switch
	 * mask > 1 : Volume */
	int mask = TSC2301_DOUBLE_MASK(priv);
	int shadow_index = TSC2301_SHADOW_INDEX(priv);
	u16 shadow_reg, shadow_reg_old;
	int ls = TSC2301_DOUBLE_LEFT_SHIFT(priv);
	int rs = TSC2301_DOUBLE_RIGHT_SHIFT(priv);
	int reg = TSC2301_PVAL_TO_REG(priv);
	int changed;

	mutex_lock(&tsc->mixer->mutex);
	shadow_reg = shadow_reg_old = tsc->mixer->shadow_regs[shadow_index];

	/* zero bits to be modified */
	shadow_reg &= ~((mask << ls) | (mask << rs));
	/* modify with new value */
	if (mask == 1) {
		/* switch. Invert switch values for the mute bits */
		shadow_reg |=
			((~ucontrol->value.integer.value[0] & mask) << ls) |
			((~ucontrol->value.integer.value[1] & mask) << rs);
	} else {
		/* volume */
		shadow_reg |=
			(ucontrol->value.integer.value[0] << ls) |
			(ucontrol->value.integer.value[1] << rs);
	}

	changed = (shadow_reg != shadow_reg_old);
	tsc->mixer->shadow_regs[shadow_index] = shadow_reg;

	/* update into TSC2301 if necessary */
	if (changed)
		tsc2301_write_reg(tsc, reg, shadow_reg);

	if (mask == 1)
		/* check is need to power down/up audio blocks in case of
		 * muted state change */
		tsc2301_power_ctrl(tsc, shadow_index, 0);
	mutex_unlock(&tsc->mixer->mutex);

	return changed;
}

static int snd_tsc2301_info_mux(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	static char *texts[4] = {"Mic", "Line", "Line swapped", "Line mono"};

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 2;
	uinfo->value.enumerated.items = 4;
	if (uinfo->value.enumerated.item > 3)
		uinfo->value.enumerated.item = 3;
	strcpy(uinfo->value.enumerated.name,
	       texts[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_tsc2301_get_mux(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct tsc2301 *tsc = kcontrol->private_data;
	unsigned long priv = kcontrol->private_value;
	int mask = TSC2301_MUX_MASK(priv);
	int ls = TSC2301_MUX_LEFT_SHIFT(priv);
	int rs = TSC2301_MUX_RIGHT_SHIFT(priv);
	int shadow_index = TSC2301_SHADOW_INDEX(priv);
	u16 shadow_reg;

	shadow_reg = tsc->mixer->shadow_regs[shadow_index];
	ucontrol->value.enumerated.item[0] = (shadow_reg >> ls) & mask;
	ucontrol->value.enumerated.item[1] = (shadow_reg >> rs) & mask;

	return 0;
}

static int snd_tsc2301_put_mux(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct tsc2301 *tsc = kcontrol->private_data;
	unsigned long priv = kcontrol->private_value;
	int mask = TSC2301_MUX_MASK(priv);
	int shadow_index = TSC2301_SHADOW_INDEX(priv);
	u16 shadow_reg, shadow_reg_old;
	int ls = TSC2301_MUX_LEFT_SHIFT(priv);
	int rs = TSC2301_MUX_RIGHT_SHIFT(priv);
	int reg = TSC2301_PVAL_TO_REG(priv);
	int changed;

	mutex_lock(&tsc->mixer->mutex);
	shadow_reg = shadow_reg_old = tsc->mixer->shadow_regs[shadow_index];

	/* zero bits to be modified */
	shadow_reg &= ~((mask << ls) | (mask << rs));
	/* modify with new value */
	shadow_reg |= (ucontrol->value.enumerated.item[0] << ls);
	shadow_reg |= (ucontrol->value.enumerated.item[1] << rs);

	changed = (shadow_reg != shadow_reg_old);

	/* update into TSC2301 if necessary */
	if (changed) {
		tsc->mixer->shadow_regs[shadow_index] = shadow_reg;
		tsc2301_write_reg(tsc, reg, shadow_reg);
	}

	/* check is need to power up/down audio blocks in case of ADC input
	 * change */
	tsc2301_power_ctrl(tsc, shadow_index, 0);
	mutex_unlock(&tsc->mixer->mutex);

	return changed;
}

static int snd_tsc2301_info_bool(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;

	return 0;
}

static int snd_tsc2301_get_bool(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct tsc2301 *tsc = kcontrol->private_data;
	unsigned long priv = kcontrol->private_value;
	int shadow_index = TSC2301_SHADOW_INDEX(priv);
	int shift = TSC2301_BOOL_SHIFT(priv);
	int invert = TSC2301_BOOL_INVERT(priv);
	u16 shadow_reg;

	shadow_reg = tsc->mixer->shadow_regs[shadow_index];
	ucontrol->value.integer.value[0] =
		invert ^ ((shadow_reg >> shift) & 1);

	return 0;
}

static int snd_tsc2301_put_bool(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct tsc2301 *tsc = kcontrol->private_data;
	unsigned long priv = kcontrol->private_value;
	int shadow_index = TSC2301_SHADOW_INDEX(priv);
	int shift = TSC2301_BOOL_SHIFT(priv);
	int invert = TSC2301_BOOL_INVERT(priv);
	int reg = TSC2301_PVAL_TO_REG(priv);
	u16 shadow_reg, shadow_reg_old;
	int changed;

	mutex_lock(&tsc->mixer->mutex);
	shadow_reg = shadow_reg_old = tsc->mixer->shadow_regs[shadow_index];

	/* zero bit to be modified */
	shadow_reg &= ~(1 << shift);
	/* modify with new value */
	shadow_reg |=
		(invert ^ (ucontrol->value.integer.value[0] & 1)) << shift;

	changed = (shadow_reg != shadow_reg_old);

	/* update into TSC2301 if necessary */
	if (changed) {
		tsc->mixer->shadow_regs[shadow_index] = shadow_reg;
		if ((shadow_index == GPIO_INDEX) &&
		    (tsc->mixer->shadow_regs[PD_MISC_INDEX] &
		     TSC2301_REG_PD_MISC_APD)) {
			/* changing GPIO control and audio is powered down.
			 * Update GPIO states according to their power-down
			 * flag */
			tsc2301_gpio_power_down(tsc);
		} else
			tsc2301_write_reg(tsc, reg, shadow_reg);
	}
	mutex_unlock(&tsc->mixer->mutex);

	return changed;
}

/* TSC2301 internal mixer controls */
static struct snd_kcontrol_new snd_tsc2301_controls[] = {
	/* stereo ADC input switches and volumes */
	TSC2301_DOUBLE("Capture Switch", 0,
		TSC2301_REG_ADCVOL, ADCVOL_INDEX,
		TSC2301_MUTE_LEFT_SHIFT, TSC2301_MUTE_RIGHT_SHIFT,
		1, 0),
	TSC2301_DOUBLE("Capture Volume", 0,
		TSC2301_REG_ADCVOL, ADCVOL_INDEX,
		TSC2301_VOL_LEFT_SHIFT, TSC2301_VOL_RIGHT_SHIFT,
		TSC2301_VOLUME_MASK, TSC2301_MIN_ADCVOL),

	/* stereo DAC output switches and volumes */
	TSC2301_DOUBLE("PCM Playback Switch", 0,
		TSC2301_REG_DACVOL, DACVOL_INDEX,
		TSC2301_MUTE_LEFT_SHIFT, TSC2301_MUTE_RIGHT_SHIFT,
		1, 0),
	TSC2301_DOUBLE("PCM Playback Volume", 0,
		TSC2301_REG_DACVOL, DACVOL_INDEX,
		TSC2301_VOL_LEFT_SHIFT, TSC2301_VOL_RIGHT_SHIFT,
		TSC2301_VOLUME_MASK, TSC2301_MIN_DACVOL),

	/* stereo line input bypass switches and volumes */
	TSC2301_DOUBLE("Line Playback Switch", 0,
		TSC2301_REG_BPVOL, BPVOL_INDEX,
		TSC2301_MUTE_LEFT_SHIFT, TSC2301_MUTE_RIGHT_SHIFT,
		1, 0),
	TSC2301_DOUBLE("Line Playback Volume", 0,
		TSC2301_REG_BPVOL, BPVOL_INDEX,
		TSC2301_VOL_LEFT_SHIFT, TSC2301_VOL_RIGHT_SHIFT,
		TSC2301_VOLUME_MASK, TSC2301_MIN_BPVOL),

	/* mono microphone input gain */
	TSC2301_SINGLE("Mic Boost", 0,
		TSC2301_REG_AUDCNTL, AUDCNTL_INDEX,
		TSC2301_MICG_SHIFT,
		TSC2301_MICG_MASK, TSC2301_MICG_MIN),

	/* ADC input sources. Both channels could be selected separately */
	TSC2301_MUX("Capture Source", 0,
		TSC2301_REG_AUDCNTL, AUDCNTL_INDEX,
		TSC2301_INML_SHIFT, TSC2301_INMR_SHIFT,
		TSC2301_INM_MASK),
};

/* must be called tsc->mixer->mutex locked */
static void tsc2301_flush_shadow_regs(struct tsc2301 *tsc)
{
	int i, page, addr;
	u16 temp;

	page = TSC2301_REG_TO_PAGE(TSC2301_REG_AUDCNTL);
	addr = TSC2301_REG_TO_ADDR(TSC2301_REG_AUDCNTL);

	for (i = 0; i < 4; i++) {
		temp = tsc->mixer->shadow_regs[i];
		tsc2301_write_reg(tsc, TSC2301_REG(page, addr + i), temp);
	}
	temp = tsc->mixer->shadow_regs[GPIO_INDEX];
	tsc2301_write_reg(tsc, TSC2301_REG_GPIO, temp);

	/* Update power state of all audio blocks depending are they
	 * muted or unused. */
	tsc2301_power_ctrl(tsc, -1, 0);
}

#ifdef CONFIG_PM
int tsc2301_mixer_suspend(struct tsc2301 *tsc)
{
	/* power down entire audio section inside TSC2301 in case the
	 * chip is still powered during the system sleep. However this driver
	 * doesn't require that chip is powered because registers are restored
	 * in function tsc2301_mixer_resume */
	mutex_lock(&tsc->mixer->mutex);
	tsc2301_gpio_power_down(tsc);
	tsc->mixer->shadow_regs[PD_MISC_INDEX] |= TSC2301_REG_PD_MISC_APD;
	tsc2301_write_reg(tsc, TSC2301_REG_PD_MISC,
			  tsc->mixer->shadow_regs[PD_MISC_INDEX]);
	mutex_unlock(&tsc->mixer->mutex);
	return 0;
}

void tsc2301_mixer_resume(struct tsc2301 *tsc)
{
	/* power up the TSC2301 audio section and restore registers */
	mutex_lock(&tsc->mixer->mutex);
	tsc->mixer->shadow_regs[PD_MISC_INDEX] &= ~TSC2301_REG_PD_MISC_APD;
	tsc2301_flush_shadow_regs(tsc);
	mutex_unlock(&tsc->mixer->mutex);
}
#endif

void tsc2301_mixer_enable_mclk(struct device *dev)
{
	struct tsc2301 *tsc = dev_get_drvdata(dev);
	struct tsc2301_mixer *mix = tsc->mixer;

	mutex_lock(&mix->mutex);
	if (!mix->mclk_enabled++ && tsc->enable_clock != NULL) {
		tsc->enable_clock(dev);
	}
	tsc2301_power_ctrl(tsc, -1, 1);
	mutex_unlock(&mix->mutex);
}

void tsc2301_mixer_disable_mclk(struct device *dev)
{
	struct tsc2301 *tsc = dev_get_drvdata(dev);
	struct tsc2301_mixer *mix = tsc->mixer;

	mutex_lock(&mix->mutex);
	mix->mclk_enabled--;
	tsc2301_power_ctrl(tsc, -1, 1);
	if (!mix->mclk_enabled && tsc->disable_clock != NULL) {
		tsc->disable_clock(dev);
	}
	mutex_unlock(&mix->mutex);
}

static void tsc2301_mixer_delayed_power_down(struct work_struct *work)
{
	struct tsc2301_mixer *mix = container_of(work, struct tsc2301_mixer,
						 delayed_power_down.work);
	struct tsc2301 *tsc = mix->tsc;

	mutex_lock(&mix->mutex);
	if (!mix->delayed_pd_active) {
		mutex_unlock(&mix->mutex);
		return;
	}
	mix->delayed_pd_active = 0;
	mutex_unlock(&mix->mutex);
	tsc2301_mixer_disable_mclk(&tsc->spi->dev);
}

/*
 * Allows audio controller driver to notify its usage of ADC and DAC
 */
void tsc2301_mixer_set_power(struct device *dev, int dac, int adc)
{
	struct tsc2301 *tsc = dev_get_drvdata(dev);

	mutex_lock(&tsc->mixer->mutex);
	tsc->mixer->adc_enabled = adc;
	tsc->mixer->dac_enabled = dac;

	/* update power state of all audio blocks */
	tsc2301_power_ctrl(tsc, -1, 1);
	mutex_unlock(&tsc->mixer->mutex);
}

/*
 * Registers TSC2301 ALSA Mixer controls for the given sound card
 */
int tsc2301_mixer_register_controls(struct device *dev, struct snd_card *card)
{
	struct tsc2301 *tsc = dev_get_drvdata(dev);
	struct tsc2301_mixer *mix = tsc->mixer;
	int i, err;

	/* Register ALSA mixer controls */
	for (i = 0; i < ARRAY_SIZE(snd_tsc2301_controls); i++) {
		err = snd_ctl_add(card,
				  snd_ctl_new1(&snd_tsc2301_controls[i], tsc));
		if (err < 0)
			return err;
	}

	if (!mix->n_mixer_gpios)
		return 0;

	/* Register additional GPIO controls if defined */
	for (i = 0; i < mix->n_mixer_gpios; i++) {
		const struct tsc2301_mixer_gpio *mg = mix->mixer_gpios + i;
		struct snd_kcontrol *ctrlp;
		struct snd_kcontrol_new ctrl =
			TSC2301_BOOL((char *)mg->name, 0,
				     TSC2301_REG_GPIO, GPIO_INDEX,
				     mg->gpio, mg->inverted, mg->def_enable);

		ctrlp = snd_ctl_new1(&ctrl, tsc);
		err = snd_ctl_add(card, ctrlp);
		if (err < 0)
			return err;
	}

	return 0;
}

int tsc2301_mixer_init(struct tsc2301 *tsc,
		       struct tsc2301_platform_data *pdata)
{
	struct tsc2301_mixer *mix;
	int err = 0;
	u16 w;

	mix = kzalloc(sizeof(*mix), GFP_KERNEL);
	if (mix == NULL)
		return -ENOMEM;
	tsc->mixer = mix;

	mix->tsc = tsc;
	mutex_init(&mix->mutex);
	mix->platform_init = pdata->codec_init;
	mix->platform_cleanup = pdata->codec_cleanup;
	mix->pll_output = pdata->pll_output;

	INIT_DELAYED_WORK(&mix->delayed_power_down,
			  tsc2301_mixer_delayed_power_down);

	/* initialize shadow register default values */
	w = 0xc000;
	w |= (pdata->mclk_ratio << 6) | (pdata->i2s_sample_rate << 2);
	w |= pdata->i2s_format;
	mix->shadow_regs[AUDCNTL_INDEX] = w;
	mix->shadow_regs[ADCVOL_INDEX] = 0xd7d7;
	mix->shadow_regs[DACVOL_INDEX] = 0xffff;
	mix->shadow_regs[BPVOL_INDEX] = 0xe7e7;
	mix->shadow_regs[PD_MISC_INDEX] = pdata->power_down_blocks;

	/* if extra mixer controls configured, then configure associated
	 * GPIOs as output and drive their default state */
	if (pdata->n_mixer_gpios) {
		int i;

		w = 0;
		for (i = 0; i < pdata->n_mixer_gpios; i++) {
			const struct tsc2301_mixer_gpio *mg;
			int gpio;

			mg = pdata->mixer_gpios + i;
			gpio = mg->gpio;
			w |= (1 << gpio) << 8;
			w |= (mg->inverted ^ mg->def_enable) << gpio;
		}
		mix->shadow_regs[GPIO_INDEX] = w;

		mix->mixer_gpios = kmalloc(sizeof(*pdata->mixer_gpios) *
					   pdata->n_mixer_gpios,
					   GFP_KERNEL);
		if (mix->mixer_gpios == NULL) {
			err = -ENOMEM;
			goto err1;
		}
		memcpy(mix->mixer_gpios, pdata->mixer_gpios,
		       sizeof(*pdata->mixer_gpios) * pdata->n_mixer_gpios);
		mix->n_mixer_gpios = pdata->n_mixer_gpios;
	}

	/* PLL control */
	tsc2301_write_pll(tsc, pdata->pll_n, pdata->pll_a, pdata->pll_pdc,
			  0, mix->pll_output ? 0 : 1);

	tsc2301_flush_shadow_regs(tsc);

	if (mix->platform_init != NULL) {
		err = mix->platform_init(&tsc->spi->dev);
		if (err < 0)
			goto err2;
	}

	return 0;
err2:
	if (mix->mixer_gpios != NULL)
		kfree(mix->mixer_gpios);
err1:
	kfree(mix);
	return err;
}

void tsc2301_mixer_exit(struct tsc2301 *tsc)
{
	struct tsc2301_mixer *mixer = tsc->mixer;

	if (mixer->platform_cleanup != NULL)
		mixer->platform_cleanup(&tsc->spi->dev);

	if (mixer->mixer_gpios != NULL)
		kfree(mixer->mixer_gpios);
}

MODULE_AUTHOR("Jarkko Nikula <jarkko.nikula@nokia.com>");
MODULE_LICENSE("GPL");
