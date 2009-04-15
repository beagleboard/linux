/*
 * include/linux/spi/tsc210x.h
 *
 * TI TSC2101/2102 control register definitions.
 *
 * Copyright (c) 2005-2007 Andrzej Zaborowski  <balrog@zabor.org>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __LINUX_SPI_TSC210X_H
#define __LINUX_SPI_TSC210X_H

struct apm_power_info;

struct tsc210x_config {
	int use_internal;	/* Use internal reference voltage */
	u32 monitor;		/* What inputs are wired on this board */
	int temp_at25c[2];	/* Thermometer calibration data */
	void (*apm_report)(struct apm_power_info *info, int battery[]);
				/* Report status to APM based on battery[] */
	void *alsa_config;	/* .platform_data for the ALSA device */
	const char *mclk;	/* Optional: mclk name */
	const char *bclk;	/* Optional: bclk name */
};

#define TSC_BAT1	(1 << 0)
#define TSC_BAT2	(1 << 1)
#define TSC_AUX1	(1 << 2)
#define TSC_AUX2	(1 << 3)
#define TSC_TEMP	(1 << 4)

#define TSC_AUX		TSC_AUX1
#define TSC_VBAT	TSC_BAT1

struct tsc210x_dev;

/* Drivers for tsc210x components like touchscreen, sensor, and audio
 * are packaged as platform drivers which can issue synchronous register
 * acceses, and may also register a callback to process their particular
 * type of data when that data is automatically sampled.  The platform
 * device is a child of the TSC spi device.
 */

extern int tsc210x_read_sync(struct tsc210x_dev *dev, int page, u8 address);
extern int tsc210x_reads_sync(struct tsc210x_dev *dev, int page,
		u8 startaddress, u16 *data, int numregs);
extern int tsc210x_write_sync(struct tsc210x_dev *dev, int page,
		u8 address, u16 data);

typedef void (*tsc210x_touch_t)(void *context, int touching);
typedef void (*tsc210x_coords_t)(void *context, int x, int y, int z1, int z2);
typedef void (*tsc210x_ports_t)(void *context, int bat[], int aux[]);
typedef void (*tsc210x_temp_t)(void *context, int temp);

extern int tsc210x_touch_cb(struct device *dev,
		tsc210x_touch_t handler, void *context);
extern int tsc210x_coords_cb(struct device *dev,
		tsc210x_coords_t handler, void *context);
extern int tsc210x_ports_cb(struct device *dev,
		tsc210x_ports_t handler, void *context);
extern int tsc210x_temp1_cb(struct device *dev,
		tsc210x_temp_t handler, void *context);
extern int tsc210x_temp2_cb(struct device *dev,
		tsc210x_temp_t handler, void *context);

#if defined(CONFIG_SOUND) || defined(CONFIG_SOUND_MODULE)
extern void tsc210x_set_dac_volume(struct device *dev, u8 left, u8 right);
extern void tsc210x_set_dac_mute(struct device *dev, int left, int right);
extern void tsc210x_get_dac_mute(struct device *dev, int *left, int *right);
extern void tsc210x_dac_power(struct device *dev, int on);
extern int tsc210x_set_rate(struct device *dev, int rate);
extern void tsc210x_set_i2s_master(struct device *dev, int state);
extern void tsc210x_set_deemphasis(struct device *dev, int enable);
extern void tsc2102_set_bassboost(struct device *dev, int enable);
#endif

/*
 * Emit a short keyclick typically in order to give feedback to
 * user on specific events.
 *
 * amplitude must be between 0 (lowest) and 2 (highest).
 * freq must be between 0 (corresponds to 62.5 Hz) and 7 (8 kHz).
 * length should be between 2 and 32 periods.
 *
 * This function sleeps but for a period unrelated to the length of
 * the sound, i.e. returning doesn't indicate that the sound has
 * finished.
 */
extern void tsc210x_keyclick(struct tsc210x_dev *dev,
		int amplitude, int freq, int length);

/* Page 0, Touch Screen & Keypad Data registers */
#define TSC210X_TS_X			0, 0x00
#define TSC210X_TS_Y			0, 0x01
#define TSC210X_TS_Z1			0, 0x02
#define TSC210X_TS_Z2			0, 0x03
#define TSC210X_TS_BAT1			0, 0x05
#define TSC2102_TS_BAT2			0, 0x06
#define TSC210X_TS_AUX1			0, 0x07
#define TSC2101_TS_AUX2			0, 0x08
#define TSC210X_TS_TEMP1		0, 0x09
#define TSC210X_TS_TEMP2		0, 0x0a

/* Page 1, Touch Screen & Keypad Control registers */
#define TSC210X_TS_ADC_CTRL		1, 0x00
#define TSC210X_TS_STATUS_CTRL		1, 0x01
#define TSC2101_TS_BUFFER_CTRL		1, 0x02
#define TSC210X_TS_REF_CTRL		1, 0x03
#define TSC210X_TS_RESET_CTRL		1, 0x04
#define TSC210X_TS_CONFIG_CTRL		1, 0x05
#define TSC2101_TS_TEMPMAX_CTRL		1, 0x06
#define TSC2101_TS_TEMPMIN_CTRL		1, 0x07
#define TSC2101_TS_AUX1MAX_CTRL		1, 0x08
#define TSC2101_TS_AUX1MIN_CTRL		1, 0x09
#define TSC2101_TS_AUX2MAX_CTRL		1, 0x0a
#define TSC2101_TS_AUX2MIN_CTRL		1, 0x0b
#define TSC2101_TS_MCONFIG_CTRL		1, 0x0c
#define TSC2101_TS_DELAY_CTRL		1, 0x0d

/* Page 2, Audio Control registers */
#define TSC210X_AUDIO1_CTRL		2, 0x00
#define TSC2101_HEADSET_GAIN_CTRL	2, 0x01
#define TSC210X_DAC_GAIN_CTRL		2, 0x02
#define TSC2101_MIXER_GAIN_CTRL		2, 0x03
#define TSC210X_AUDIO2_CTRL		2, 0x04
#define TSC210X_POWER_CTRL		2, 0x05
#define TSC210X_AUDIO3_CTRL		2, 0x06
#define TSC210X_LCH_BASS_BOOST_N0	2, 0x07
#define TSC210X_LCH_BASS_BOOST_N1	2, 0x08
#define TSC210X_LCH_BASS_BOOST_N2	2, 0x09
#define TSC210X_LCH_BASS_BOOST_N3	2, 0x0a
#define TSC210X_LCH_BASS_BOOST_N4	2, 0x0b
#define TSC210X_LCH_BASS_BOOST_N5	2, 0x0c
#define TSC210X_LCH_BASS_BOOST_D1	2, 0x0d
#define TSC210X_LCH_BASS_BOOST_D2	2, 0x0e
#define TSC210X_LCH_BASS_BOOST_D4	2, 0x0f
#define TSC210X_LCH_BASS_BOOST_D5	2, 0x10
#define TSC210X_RCH_BASS_BOOST_N0	2, 0x11
#define TSC210X_RCH_BASS_BOOST_N1	2, 0x12
#define TSC210X_RCH_BASS_BOOST_N2	2, 0x13
#define TSC210X_RCH_BASS_BOOST_N3	2, 0x14
#define TSC210X_RCH_BASS_BOOST_N4	2, 0x15
#define TSC210X_RCH_BASS_BOOST_N5	2, 0x16
#define TSC210X_RCH_BASS_BOOST_D1	2, 0x17
#define TSC210X_RCH_BASS_BOOST_D2	2, 0x18
#define TSC210X_RCH_BASS_BOOST_D4	2, 0x19
#define TSC210X_RCH_BASS_BOOST_D5	2, 0x1a
#define TSC210X_PLL1_CTRL		2, 0x1b
#define TSC210X_PLL2_CTRL		2, 0x1c
#define TSC210X_AUDIO4_CTRL		2, 0x1d
#define TSC2101_HANDSET_GAIN_CTRL	2, 0x1e
#define TSC2101_CELL_GAIN_CTRL		2, 0x1f
#define TSC2101_AUIDO5_CTRL		2, 0x20
#define TSC2101_AUDIO6_CTRL		2, 0x21
#define TSC2101_AUDIO7_CTRL		2, 0x22
#define TSC2101_GPIO_CTRL		2, 0x23
#define TSC2101_IN_AGC_CTRL		2, 0x24
#define TSC2101_POWER_STATUS		2, 0x25
#define TSC2101_MIX_AGC_CTRL		2, 0x26
#define TSC2101_CELL_AGC_CTRL		2, 0x27

/* Field masks for Audio Control 1 */
#define AC1_WLEN(ARG)			(((ARG) & 0x03) << 10)
#define AC1_DATFM(ARG)			(((ARG) & 0x03) << 8)
#define AC1_DACFS(ARG)			((ARG) & 0x3f)

/* Field masks for TSC2102_DAC_GAIN_CTRL */
#define DGC_DALMU			(1 << 15)
#define DGC_DALVL(ARG)			(((ARG) & 0x7f) << 8)
#define DGC_DARMU			(1 << 7)
#define DGC_DARVL(ARG)			(((ARG) & 0x7f))

/* Field formats for TSC210X_AUDIO2_CTRL */
#define AC2_KCLEN			(1 << 15)
#define AC2_KCLAC(ARG)			(((ARG) & 0x07) << 12)
#define AC2_KCLFRQ(ARG)			(((ARG) & 0x07) << 8)
#define AC2_KCLLN(ARG)			(((ARG) & 0x0f) << 4)
#define AC2_DLGAF			(1 << 3)
#define AC2_DRGAF			(1 << 2)
#define AC2_DASTC			(1 << 1)

/* Field masks for TSC210X_DAC_POWER_CTRL */
#define CPC_PWDNC			(1 << 15)
#define CPC_DAODRC			(1 << 12)
#define CPC_DAPWDN			(1 << 10)
#define CPC_VGPWDN			(1 << 8)
#define CPC_DAPWDF			(1 << 6)
#define CPC_BASSBC			(1 << 1)
#define CPC_DEEMPF			(0x01)

/* Field masks for TSC210X_AUDIO3_CTRL */
#define AC3_DMSVOL(ARG)			(((ARG) & 0x03) << 14)
#define AC3_REFFS			(1 << 13)
#define AC3_DAXFM			(1 << 12)
#define AC3_SLVMS			(1 << 11)
#define AC3_DALOVF			(1 << 7)
#define AC3_DAROVF			(1 << 6)
#define AC3_REVID(ARG)			(((ARG) & 0x07))

/* Field masks for TSC210X_PLL1_CTRL */
#define PLL1_PLLEN			(1 << 15)
#define PLL1_Q_VAL(ARG)			(((ARG) & 0x0f) << 11)
#define PLL1_P_VAL(ARG)			(((ARG) & 0x07) << 8)
#define PLL1_I_VAL(ARG)			(((ARG) & 0x3f) << 2)

/* Field masks for TSC210X_PLL2_CTRL */
#define PLL2_D_VAL(ARG)			(((ARG) & 0x3fff) << 2)

/* Field masks for TSC210X_AUDIO4_CTRL */
#define AC4_DASTPD			(1 << 14)

struct tsc210x_rate_info_s {
	u16 sample_rate;
	u8 divisor;
	u8 fs_44k;	/* 44.1 kHz Fsref if 1, 48 kHz if 0 */
};

#endif	/* __LINUX_SPI_TSC210X_H */
