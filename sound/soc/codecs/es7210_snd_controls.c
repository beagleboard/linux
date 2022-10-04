/*
 * ALSA SoC ES7210 adc driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.^M
 *
 * Notes:
 *  ES7210 is a 4-ch ADC of Everest
 *  This file is used for snd_soc_controls for es7210, such as 
 *  PGA gain control, mute, volume and suspend
 */

static const DECLARE_TLV_DB_SCALE(mic_boost_tlv, 0, 300, 0);
static const DECLARE_TLV_DB_SCALE(direct_gain_tlv, -9550, 50, 0);
#if ES7210_CHANNELS_MAX > 0
static int es7210_micboost1_setting_set(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x43,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[0]);
	return 0;
}

static int es7210_micboost1_setting_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x43, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_micboost2_setting_set(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x44,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[0]);
	return 0;
}

static int es7210_micboost2_setting_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x44, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_micboost3_setting_set(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x45,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[0]);
	return 0;
}

static int es7210_micboost3_setting_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x45, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_micboost4_setting_set(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x46,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[0]);
	return 0;
}

static int es7210_micboost4_setting_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x46, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_adc1_mute_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC12_MUTE_REG15,
			   0x01,
			   ucontrol->value.integer.value[0] & 0x01,
			   i2c_clt1[0]);
	return 0;
}

static int es7210_adc1_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC12_MUTE_REG15, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}

static int es7210_adc2_mute_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC12_MUTE_REG15,
			   0x02,
			   (ucontrol->value.integer.value[0] & 0x01) << 1,
			   i2c_clt1[0]);
	return 0;
}

static int es7210_adc2_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC12_MUTE_REG15, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = (val & 0x02) >> 1;
	return 0;
}

static int es7210_adc3_mute_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC34_MUTE_REG14,
			   0x01,
			   ucontrol->value.integer.value[0] & 0x01,
			   i2c_clt1[0]);
	return 0;
}

static int es7210_adc3_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC34_MUTE_REG14, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}

static int es7210_adc4_mute_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC34_MUTE_REG14,
			   0x02,
			   (ucontrol->value.integer.value[0] & 0x01) << 1,
			   i2c_clt1[0]);
	return 0;
}

static int es7210_adc4_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC34_MUTE_REG14, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = (val & 0x02) >> 1;
	return 0;
}

static int es7210_adc12_suspend_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {	//suspend
		es7210_update_bits(0x4b, 0xff, 0xff, i2c_clt1[0]);
		es7210_update_bits(0x01, 0x2a, 0x2a, i2c_clt1[0]);
	} else {
		es7210_update_bits(0x4b, 0xff, 0x00, i2c_clt1[0]);
		es7210_read(0x01, &val, i2c_clt1[0]);
		val &= 0x34;
		es7210_write(0x01, val, i2c_clt1[0]);
	}
	return 0;
}

static int es7210_adc12_suspend_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x4b, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}

static int es7210_adc34_suspend_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {	//suspend
		es7210_update_bits(0x4c, 0xff, 0xff, i2c_clt1[0]);
		es7210_update_bits(0x01, 0x34, 0x34, i2c_clt1[0]);
	} else {
		es7210_update_bits(0x4c, 0xff, 0x00, i2c_clt1[0]);
		es7210_read(0x01, &val, i2c_clt1[0]);
		val &= 0x2a;
		es7210_write(0x01, val, i2c_clt1[0]);
	}
	return 0;
}

static int es7210_adc34_suspend_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x4c, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}
#endif

#if ES7210_CHANNELS_MAX > 4
static int es7210_micboost5_setting_set(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x43,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[1]);
	return 0;
}

static int es7210_micboost5_setting_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x43, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_micboost6_setting_set(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x44,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[1]);
	return 0;
}

static int es7210_micboost6_setting_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x44, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_micboost7_setting_set(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x45,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[1]);
	return 0;
}

static int es7210_micboost7_setting_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x45, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_micboost8_setting_set(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x46,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[1]);
	return 0;
}

static int es7210_micboost8_setting_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x46, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_adc5_mute_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC12_MUTE_REG15,
			   0x01,
			   ucontrol->value.integer.value[0] & 0x01,
			   i2c_clt1[1]);
	return 0;
}

static int es7210_adc5_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC12_MUTE_REG15, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}

static int es7210_adc6_mute_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC12_MUTE_REG15,
			   0x02,
			   (ucontrol->value.integer.value[0] & 0x01) << 1,
			   i2c_clt1[1]);
	return 0;
}

static int es7210_adc6_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC12_MUTE_REG15, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = (val & 0x02) >> 1;
	return 0;
}

static int es7210_adc7_mute_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC34_MUTE_REG14,
			   0x01,
			   ucontrol->value.integer.value[0] & 0x01,
			   i2c_clt1[1]);
	return 0;
}

static int es7210_adc7_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC34_MUTE_REG14, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}

static int es7210_adc8_mute_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC34_MUTE_REG14,
			   0x02,
			   (ucontrol->value.integer.value[0] & 0x01) << 1,
			   i2c_clt1[1]);
	return 0;
}

static int es7210_adc8_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC34_MUTE_REG14, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = (val & 0x02) >> 1;
	return 0;
}

static int es7210_adc56_suspend_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {	//suspend
		es7210_update_bits(0x4b, 0xff, 0xff, i2c_clt1[1]);
		es7210_update_bits(0x01, 0x2a, 0x2a, i2c_clt1[1]);
	} else {
		es7210_update_bits(0x4b, 0xff, 0x00, i2c_clt1[1]);
		es7210_read(0x01, &val, i2c_clt1[1]);
		val &= 0x34;
		es7210_write(0x01, val, i2c_clt1[1]);
	}
	return 0;
}

static int es7210_adc56_suspend_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x4b, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}

static int es7210_adc78_suspend_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {	//suspend
		es7210_update_bits(0x4c, 0xff, 0xff, i2c_clt1[1]);
		es7210_update_bits(0x01, 0x34, 0x34, i2c_clt1[1]);
	} else {
		es7210_update_bits(0x4c, 0xff, 0x00, i2c_clt1[1]);
		es7210_read(0x01, &val, i2c_clt1[1]);
		val &= 0x2a;
		es7210_write(0x01, val, i2c_clt1[1]);
	}
	return 0;
}

static int es7210_adc78_suspend_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x4c, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}
#endif
#if ES7210_CHANNELS_MAX > 8
static int es7210_micboost9_setting_set(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x43,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[2]);
	return 0;
}

static int es7210_micboost9_setting_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x43, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_micboost10_setting_set(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x44,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[2]);
	return 0;
}

static int es7210_micboost10_setting_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x44, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_micboost11_setting_set(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x45,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[2]);
	return 0;
}

static int es7210_micboost11_setting_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x45, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_micboost12_setting_set(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x46,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[2]);
	return 0;
}

static int es7210_micboost12_setting_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x46, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_adc9_mute_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC12_MUTE_REG15, 0x01,
			   ucontrol->value.integer.value[0] & 0x01,
			   i2c_clt1[2]);
	return 0;
}

static int es7210_adc9_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC12_MUTE_REG15, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}

static int es7210_adc10_mute_set(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC12_MUTE_REG15,
			   0x02,
			   (ucontrol->value.integer.value[0] & 0x01) << 1,
			   i2c_clt1[2]);
	return 0;
}

static int es7210_adc10_mute_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC12_MUTE_REG15, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = (val & 0x02) >> 1;
	return 0;
}

static int es7210_adc11_mute_set(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC34_MUTE_REG14,
			   0x01,
			   ucontrol->value.integer.value[0] & 0x01,
			   i2c_clt1[2]);
	return 0;
}

static int es7210_adc11_mute_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC34_MUTE_REG14, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}

static int es7210_adc12_mute_set(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC34_MUTE_REG14,
			   0x02,
			   (ucontrol->value.integer.value[0] & 0x01) << 1,
			   i2c_clt1[2]);
	return 0;
}

static int es7210_adc12_mute_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC34_MUTE_REG14, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = (val & 0x02) >> 1;
	return 0;
}

static int es7210_adc910_suspend_set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {	//suspend
		es7210_update_bits(0x4b, 0xff, 0xff, i2c_clt1[2]);
		es7210_update_bits(0x01, 0x2a, 0x2a, i2c_clt1[2]);
	} else {
		es7210_update_bits(0x4b, 0xff, 0x00, i2c_clt1[2]);
		es7210_read(0x01, &val, i2c_clt1[2]);
		val &= 0x34;
		es7210_write(0x01, val, i2c_clt1[2]);
	}
	return 0;
}

static int es7210_adc910_suspend_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x4b, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}

static int es7210_adc1112_suspend_set(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {	//suspend
		es7210_update_bits(0x4c, 0xff, 0xff, i2c_clt1[2]);
		es7210_update_bits(0x01, 0x34, 0x34, i2c_clt1[2]);
	} else {
		es7210_update_bits(0x4c, 0xff, 0x00, i2c_clt1[2]);
		es7210_read(0x01, &val, i2c_clt1[2]);
		val &= 0x2a;
		es7210_write(0x01, val, i2c_clt1[2]);
	}
	return 0;
}

static int es7210_adc1112_suspend_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x4c, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}
#endif
#if ES7210_CHANNELS_MAX > 12
static int es7210_micboost13_setting_set(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x43,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[3]);
	return 0;
}

static int es7210_micboost13_setting_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x43, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_micboost14_setting_set(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x44,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[3]);
	return 0;
}

static int es7210_micboost14_setting_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x44, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_micboost15_setting_set(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x45,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[3]);
	return 0;
}

static int es7210_micboost15_setting_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x45, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_micboost16_setting_set(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x46,
			   0x0F,
			   ucontrol->value.integer.value[0] & 0x0f, i2c_clt1[3]);
	return 0;
}

static int es7210_micboost16_setting_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x46, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int es7210_adc13_mute_set(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC12_MUTE_REG15, 0x01,
			   ucontrol->value.integer.value[0] & 0x01,
			   i2c_clt1[3]);
	return 0;
}

static int es7210_adc13_mute_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC12_MUTE_REG15, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}

static int es7210_adc14_mute_set(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC12_MUTE_REG15,
			   0x02,
			   (ucontrol->value.integer.value[0] & 0x01) << 1,
			   i2c_clt1[3]);
	return 0;
}

static int es7210_adc14_mute_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC12_MUTE_REG15, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = (val & 0x02) >> 1;
	return 0;
}

static int es7210_adc15_mute_set(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC34_MUTE_REG14,
			   0x01,
			   ucontrol->value.integer.value[0] & 0x01,
			   i2c_clt1[3]);
	return 0;
}

static int es7210_adc15_mute_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC34_MUTE_REG14, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}

static int es7210_adc16_mute_set(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(ES7210_ADC34_MUTE_REG14,
			   0x02,
			   (ucontrol->value.integer.value[0] & 0x01) << 1,
			   i2c_clt1[3]);
	return 0;
}

static int es7210_adc16_mute_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(ES7210_ADC34_MUTE_REG14, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = (val & 0x02) >> 1;
	return 0;
}

static int es7210_adc1314_suspend_set(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {	//suspend
		es7210_update_bits(0x4b, 0xff, 0xff, i2c_clt1[3]);
		es7210_update_bits(0x01, 0x2a, 0x2a, i2c_clt1[3]);
	} else {
		es7210_update_bits(0x4b, 0xff, 0x00, i2c_clt1[3]);
		es7210_read(0x01, &val, i2c_clt1[3]);
		val &= 0x34;
		es7210_write(0x01, val, i2c_clt1[3]);
	}
	return 0;
}

static int es7210_adc1314_suspend_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x4b, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}

static int es7210_adc1516_suspend_set(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {	//suspend
		es7210_update_bits(0x4c, 0xff, 0xff, i2c_clt1[3]);
		es7210_update_bits(0x01, 0x34, 0x34, i2c_clt1[3]);
	} else {
		es7210_update_bits(0x4c, 0xff, 0x00, i2c_clt1[3]);
		es7210_read(0x01, &val, i2c_clt1[3]);
		val &= 0x2a;
		es7210_write(0x01, val, i2c_clt1[3]);
	}
	return 0;
}

static int es7210_adc1516_suspend_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x4c, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = val & 0x01;
	return 0;
}
#endif

#if ES7210_CHANNELS_MAX > 0
static int es7210_direct_gain_1_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1e,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[0]);
	return 0;
}

static int es7210_direct_gain_1_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1e, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_2_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1D,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[0]);
	return 0;
}

static int es7210_direct_gain_2_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1D, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_3_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1C,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[0]);
	return 0;
}

static int es7210_direct_gain_3_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1C, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_4_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1B,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[0]);
	return 0;
}

static int es7210_direct_gain_4_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1B, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_12_sameon_set(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value
					    *ucontrol)
{
	es7210_update_bits(0x1a,
			   0x01,
			   ucontrol->value.integer.value[0], i2c_clt1[0]);
	return 0;
}

static int es7210_direct_gain_12_sameon_get(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value
					    *ucontrol)
{
	u8 val;
	es7210_read(0x1a, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_34_sameon_set(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value
					    *ucontrol)
{
	es7210_update_bits(0x1a,
			   0x02,
			   ucontrol->value.integer.value[0], i2c_clt1[0]);
	return 0;
}

static int es7210_direct_gain_34_sameon_get(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value
					    *ucontrol)
{
	u8 val;
	es7210_read(0x1a, &val, i2c_clt1[0]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}
#endif

#if ES7210_CHANNELS_MAX > 4
static int es7210_direct_gain_5_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1e,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[1]);
	return 0;
}

static int es7210_direct_gain_5_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1e, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_6_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1D,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[1]);
	return 0;
}

static int es7210_direct_gain_6_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1D, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_7_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1C,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[1]);
	return 0;
}

static int es7210_direct_gain_7_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1C, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_8_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1B,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[1]);
	return 0;
}

static int es7210_direct_gain_8_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1B, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_56_sameon_set(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value
					    *ucontrol)
{
	es7210_update_bits(0x1a, 0
			   x01, ucontrol->value.integer.value[0], i2c_clt1[1]);
	return 0;
}

static int es7210_direct_gain_56_sameon_get(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value
					    *ucontrol)
{
	u8 val;
	es7210_read(0x1a, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_78_sameon_set(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value
					    *ucontrol)
{
	es7210_update_bits(0x1a,
			   0x02,
			   ucontrol->value.integer.value[0], i2c_clt1[1]);
	return 0;
}

static int es7210_direct_gain_78_sameon_get(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value
					    *ucontrol)
{
	u8 val;
	es7210_read(0x1a, &val, i2c_clt1[1]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}
#endif

#if ES7210_CHANNELS_MAX > 8
static int es7210_direct_gain_9_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1e,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[2]);
	return 0;
}

static int es7210_direct_gain_9_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1e, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_10_set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1D,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[2]);
	return 0;
}

static int es7210_direct_gain_10_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1D, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_11_set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1C,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[2]);
	return 0;
}

static int es7210_direct_gain_11_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1C, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_12_set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1B,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[2]);
	return 0;
}

static int es7210_direct_gain_12_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1B, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_9_10_sameon_set(struct snd_kcontrol *kcontrol,
					      struct snd_ctl_elem_value
					      *ucontrol)
{
	es7210_update_bits(0x1a,
			   0x01,
			   ucontrol->value.integer.value[0], i2c_clt1[2]);
	return 0;
}

static int es7210_direct_gain_9_10_sameon_get(struct snd_kcontrol *kcontrol,
					      struct snd_ctl_elem_value
					      *ucontrol)
{
	u8 val;
	es7210_read(0x1a, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_11_12_sameon_set(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value
					       *ucontrol)
{
	es7210_update_bits(0x1a,
			   0x02,
			   ucontrol->value.integer.value[0], i2c_clt1[2]);
	return 0;
}

static int es7210_direct_gain_11_12_sameon_get(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value
					       *ucontrol)
{
	u8 val;
	es7210_read(0x1a, &val, i2c_clt1[2]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}
#endif

#if ES7210_CHANNELS_MAX > 12
static int es7210_direct_gain_13_set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1e,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[3]);
	return 0;
}

static int es7210_direct_gain_13_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1e, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_14_set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1D,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[3]);
	return 0;
}

static int es7210_direct_gain_14_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1D, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_15_set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1C,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[3]);
	return 0;
}

static int es7210_direct_gain_15_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1C, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_16_set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	es7210_update_bits(0x1B,
			   0xFF,
			   ucontrol->value.integer.value[0], i2c_clt1[3]);
	return 0;
}

static int es7210_direct_gain_16_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7210_read(0x1B, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_13_14_sameon_set(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value
					       *ucontrol)
{
	es7210_update_bits(0x1a,
			   0x01,
			   ucontrol->value.integer.value[0], i2c_clt1[3]);
	return 0;
}

static int es7210_direct_gain_13_14_sameon_get(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value
					       *ucontrol)
{
	u8 val;
	es7210_read(0x1a, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int es7210_direct_gain_15_16_sameon_set(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value
					       *ucontrol)
{
	es7210_update_bits(0x1a,
			   0x02,
			   ucontrol->value.integer.value[0], i2c_clt1[3]);
	return 0;
}

static int es7210_direct_gain_15_16_sameon_get(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value
					       *ucontrol)
{
	u8 val;
	es7210_read(0x1a, &val, i2c_clt1[3]);
	ucontrol->value.integer.value[0] = val;
	return 0;
}
#endif

static const struct snd_kcontrol_new es7210_snd_controls[] = {
#if ES7210_CHANNELS_MAX > 0
	SOC_SINGLE_EXT_TLV("PGA1_setting", 0x43, 0, 0x0F, 0,
			   es7210_micboost1_setting_get,
			   es7210_micboost1_setting_set,
			   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA2_setting", 0x44, 0, 0x0F, 0,
			   es7210_micboost2_setting_get,
			   es7210_micboost2_setting_set,
			   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA3_setting", 0x45, 0, 0x0F, 0,
			   es7210_micboost3_setting_get,
			   es7210_micboost3_setting_set,
			   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA4_setting", 0x46, 0, 0x0F, 0,
			   es7210_micboost4_setting_get,
			   es7210_micboost4_setting_set,
			   mic_boost_tlv),
	SOC_SINGLE_EXT("ADC1_MUTE", ES7210_ADC12_MUTE_REG15,
		       0, 1, 0,
		       es7210_adc1_mute_get,
		       es7210_adc1_mute_set),
	SOC_SINGLE_EXT("ADC2_MUTE", ES7210_ADC12_MUTE_REG15,
		       1, 1, 0,
		       es7210_adc2_mute_get,
		       es7210_adc2_mute_set),
	SOC_SINGLE_EXT("ADC3_MUTE", ES7210_ADC34_MUTE_REG14,
		       0, 1, 0,
		       es7210_adc3_mute_get,
		       es7210_adc3_mute_set),
	SOC_SINGLE_EXT("ADC4_MUTE", ES7210_ADC34_MUTE_REG14,
		       1, 1, 0,
		       es7210_adc4_mute_get,
		       es7210_adc4_mute_set),

	SOC_SINGLE_EXT_TLV("ADC1_DIRECT_GAIN",
			   ES7210_ALC1_MAX_GAIN_REG1E,
			   0, 0xff, 0,
			   es7210_direct_gain_1_get,
			   es7210_direct_gain_1_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT_TLV("ADC2_DIRECT_GAIN",
			   ES7210_ALC2_MAX_GAIN_REG1D,
			   0, 0xff, 0,
			   es7210_direct_gain_2_get,
			   es7210_direct_gain_2_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT_TLV("ADC3_DIRECT_GAIN",
			   ES7210_ALC3_MAX_GAIN_REG1C,
			   0, 0xff, 0,
			   es7210_direct_gain_3_get,
			   es7210_direct_gain_3_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT_TLV("ADC4_DIRECT_GAIN",
			   ES7210_ALC4_MAX_GAIN_REG1B,
			   0, 0xff, 0,
			   es7210_direct_gain_4_get,
			   es7210_direct_gain_4_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT("DIRECT_1_2_GAIN_SAME_ON",
		       ES7210_ALC_COM_CFG2_REG1A,
		       0, 1, 0,
		       es7210_direct_gain_12_sameon_get,
		       es7210_direct_gain_12_sameon_set),
	SOC_SINGLE_EXT("DIRECT_3_4_GAIN_SAME_ON",
		       ES7210_ALC_COM_CFG2_REG1A, 1, 1, 0,
		       es7210_direct_gain_34_sameon_get,
		       es7210_direct_gain_34_sameon_set),
	SOC_SINGLE_EXT("ADC12_SUSPEND",
		       0x4b, 0, 1, 0,
		       es7210_adc12_suspend_get,
		       es7210_adc12_suspend_set),
	SOC_SINGLE_EXT("ADC34_SUSPEND",
		       0x4c, 0, 1, 0,
		       es7210_adc34_suspend_get,
		       es7210_adc34_suspend_set),
#endif

#if ES7210_CHANNELS_MAX > 4
	SOC_SINGLE_EXT_TLV("PGA5_setting", 0x43, 0, 0x0F, 0,
			   es7210_micboost5_setting_get,
			   es7210_micboost5_setting_set,
			   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA6_setting", 0x44, 0, 0x0F, 0,
			   es7210_micboost6_setting_get,
			   es7210_micboost6_setting_set,
			   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA7_setting", 0x45, 0, 0x0F, 0,
			   es7210_micboost7_setting_get,
			   es7210_micboost7_setting_set,
			   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA8_setting", 0x46, 0, 0x0F, 0,
			   es7210_micboost8_setting_get,
			   es7210_micboost8_setting_set,
			   mic_boost_tlv),

	SOC_SINGLE_EXT("ADC5_MUTE", ES7210_ADC12_MUTE_REG15,
		       0, 1, 0,
		       es7210_adc5_mute_get,
		       es7210_adc5_mute_set),
	SOC_SINGLE_EXT("ADC6_MUTE", ES7210_ADC12_MUTE_REG15,
		       1, 1, 0,
		       es7210_adc6_mute_get,
		       es7210_adc6_mute_set),
	SOC_SINGLE_EXT("ADC7_MUTE", ES7210_ADC34_MUTE_REG14,
		       0, 1, 0,
		       es7210_adc7_mute_get,
		       es7210_adc7_mute_set),
	SOC_SINGLE_EXT("ADC8_MUTE", ES7210_ADC34_MUTE_REG14,
		       1, 1, 0,
		       es7210_adc8_mute_get,
		       es7210_adc8_mute_set),

	SOC_SINGLE_EXT_TLV("ADC5_DIRECT_GAIN",
			   ES7210_ALC1_MAX_GAIN_REG1E,
			   0, 0xff, 0,
			   es7210_direct_gain_5_get,
			   es7210_direct_gain_5_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT_TLV("ADC6_DIRECT_GAIN",
			   ES7210_ALC2_MAX_GAIN_REG1D,
			   0, 0xff, 0,
			   es7210_direct_gain_6_get,
			   es7210_direct_gain_6_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT_TLV("ADC7_DIRECT_GAIN",
			   ES7210_ALC3_MAX_GAIN_REG1C,
			   0, 0xff, 0,
			   es7210_direct_gain_7_get,
			   es7210_direct_gain_7_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT_TLV("ADC8_DIRECT_GAIN",
			   ES7210_ALC4_MAX_GAIN_REG1B,
			   0, 0xff, 0,
			   es7210_direct_gain_8_get,
			   es7210_direct_gain_8_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT("DIRECT_5_6_GAIN_SAME_ON",
		       ES7210_ALC_COM_CFG2_REG1A,
		       0, 1, 0,
		       es7210_direct_gain_56_sameon_get,
		       es7210_direct_gain_56_sameon_set),
	SOC_SINGLE_EXT("DIRECT_7_8_GAIN_SAME_ON",
		       ES7210_ALC_COM_CFG2_REG1A,
		       1, 1, 0,
		       es7210_direct_gain_78_sameon_get,
		       es7210_direct_gain_78_sameon_set),
	SOC_SINGLE_EXT("ADC56_SUSPEND",
		       0x4b, 0, 1, 0,
		       es7210_adc56_suspend_get,
		       es7210_adc56_suspend_set),
	SOC_SINGLE_EXT("ADC78_SUSPEND",
		       0x4c, 0, 1, 0,
		       es7210_adc78_suspend_get,
		       es7210_adc78_suspend_set),
#endif
#if ES7210_CHANNELS_MAX > 8
	SOC_SINGLE_EXT_TLV("PGA9_setting", 0x43, 0, 0x0F, 0,
			   es7210_micboost9_setting_get,
			   es7210_micboost9_setting_set,
			   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA10_setting", 0x44, 0, 0x0F, 0,
			   es7210_micboost10_setting_get,
			   es7210_micboost10_setting_set,
			   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA11_setting", 0x45, 0, 0x0F, 0,
			   es7210_micboost11_setting_get,
			   es7210_micboost11_setting_set,
			   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA12_setting", 0x46, 0, 0x0F, 0,
			   es7210_micboost12_setting_get,
			   es7210_micboost12_setting_set,
			   mic_boost_tlv),

	SOC_SINGLE_EXT("ADC9_MUTE", ES7210_ADC12_MUTE_REG15,
		       0, 1, 0,
		       es7210_adc9_mute_get,
		       es7210_adc9_mute_set),
	SOC_SINGLE_EXT("ADC10_MUTE", ES7210_ADC12_MUTE_REG15,
		       1, 1, 0,
		       es7210_adc10_mute_get,
		       es7210_adc10_mute_set),
	SOC_SINGLE_EXT("ADC11_MUTE", ES7210_ADC34_MUTE_REG14,
		       0, 1, 0,
		       es7210_adc11_mute_get,
		       es7210_adc11_mute_set),
	SOC_SINGLE_EXT("ADC12_MUTE", ES7210_ADC34_MUTE_REG14,
		       1, 1, 0,
		       es7210_adc12_mute_get,
		       es7210_adc12_mute_set),

	SOC_SINGLE_EXT_TLV("ADC9_DIRECT_GAIN",
			   ES7210_ALC1_MAX_GAIN_REG1E,
			   0, 0xff, 0,
			   es7210_direct_gain_9_get,
			   es7210_direct_gain_9_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT_TLV("ADC10_DIRECT_GAIN",
			   ES7210_ALC2_MAX_GAIN_REG1D,
			   0, 0xff, 0,
			   es7210_direct_gain_10_get,
			   es7210_direct_gain_10_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT_TLV("ADC11_DIRECT_GAIN",
			   ES7210_ALC3_MAX_GAIN_REG1C,
			   0, 0xff, 0,
			   es7210_direct_gain_11_get,
			   es7210_direct_gain_11_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT_TLV("ADC12_DIRECT_GAIN",
			   ES7210_ALC4_MAX_GAIN_REG1B,
			   0, 0xff, 0,
			   es7210_direct_gain_12_get,
			   es7210_direct_gain_12_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT("DIRECT_9_10_GAIN_SAME_ON",
		       ES7210_ALC_COM_CFG2_REG1A,
		       0, 1, 0,
		       es7210_direct_gain_9_10_sameon_get,
		       es7210_direct_gain_9_10_sameon_set),
	SOC_SINGLE_EXT("DIRECT_11_12_GAIN_SAME_ON",
		       ES7210_ALC_COM_CFG2_REG1A,
		       1, 1, 0,
		       es7210_direct_gain_11_12_sameon_get,
		       es7210_direct_gain_11_12_sameon_set),
	SOC_SINGLE_EXT("ADC910_SUSPEND",
		       0x4b, 0, 1, 0,
		       es7210_adc910_suspend_get,
		       es7210_adc910_suspend_set),
	SOC_SINGLE_EXT("ADC1112_SUSPEND",
		       0x4c, 0, 1, 0,
		       es7210_adc1112_suspend_get,
		       es7210_adc1112_suspend_set),

#endif
#if ES7210_CHANNELS_MAX > 12
	SOC_SINGLE_EXT_TLV("PGA13_setting", 0x43, 0, 0x0F, 0,
			   es7210_micboost13_setting_get,
			   es7210_micboost13_setting_set,
			   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA14_setting", 0x44, 0, 0x0F, 0,
			   es7210_micboost14_setting_get,
			   es7210_micboost14_setting_set,
			   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA15_setting", 0x45, 0, 0x0F, 0,
			   es7210_micboost15_setting_get,
			   es7210_micboost15_setting_set,
			   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA16_setting", 0x46, 0, 0x0F, 0,
			   es7210_micboost16_setting_get,
			   es7210_micboost16_setting_set,
			   mic_boost_tlv),

	SOC_SINGLE_EXT("ADC13_MUTE", ES7210_ADC12_MUTE_REG15,
		       0, 1, 0,
		       es7210_adc13_mute_get,
		       es7210_adc13_mute_set),
	SOC_SINGLE_EXT("ADC14_MUTE", ES7210_ADC12_MUTE_REG15,
		       1, 1, 0,
		       es7210_adc14_mute_get,
		       es7210_adc14_mute_set),
	SOC_SINGLE_EXT("ADC15_MUTE", ES7210_ADC34_MUTE_REG14,
		       0, 1, 0,
		       es7210_adc15_mute_get,
		       es7210_adc15_mute_set),
	SOC_SINGLE_EXT("ADC16_MUTE", ES7210_ADC34_MUTE_REG14,
		       1, 1, 0,
		       es7210_adc16_mute_get,
		       es7210_adc16_mute_set),

	SOC_SINGLE_EXT_TLV("ADC13_DIRECT_GAIN",
			   ES7210_ALC1_MAX_GAIN_REG1E,
			   0, 0xff, 0,
			   es7210_direct_gain_13_get,
			   es7210_direct_gain_13_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT_TLV("ADC14_DIRECT_GAIN",
			   ES7210_ALC2_MAX_GAIN_REG1D,
			   0, 0xff, 0,
			   es7210_direct_gain_14_get,
			   es7210_direct_gain_14_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT_TLV("ADC15_DIRECT_GAIN",
			   ES7210_ALC3_MAX_GAIN_REG1C,
			   0, 0xff, 0,
			   es7210_direct_gain_15_get,
			   es7210_direct_gain_15_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT_TLV("ADC16_DIRECT_GAIN",
			   ES7210_ALC4_MAX_GAIN_REG1B,
			   0, 0xff, 0,
			   es7210_direct_gain_16_get,
			   es7210_direct_gain_16_set,
			   direct_gain_tlv),
	SOC_SINGLE_EXT("DIRECT_13_14_GAIN_SAME_ON",
		       ES7210_ALC_COM_CFG2_REG1A,
		       0, 1, 0,
		       es7210_direct_gain_13_14_sameon_get,
		       es7210_direct_gain_13_14_sameon_set),
	SOC_SINGLE_EXT("DIRECT_15_16_GAIN_SAME_ON",
		       ES7210_ALC_COM_CFG2_REG1A,
		       1, 1, 0,
		       es7210_direct_gain_15_16_sameon_get,
		       es7210_direct_gain_15_16_sameon_set),
	SOC_SINGLE_EXT("ADC1314_SUSPEND",
		       0x4b, 0, 1, 0,
		       es7210_adc1314_suspend_get,
		       es7210_adc1314_suspend_set),
	SOC_SINGLE_EXT("ADC1516_SUSPEND",
		       0x4c, 0, 1, 0,
		       es7210_adc1516_suspend_get,
		       es7210_adc1516_suspend_set),
#endif

};
