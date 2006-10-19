/*
 * include/linux/spi/tsc2102.h
 *
 * TI TSC2102 Touchscreen, Audio and Battery control register definitions 
 *
 * Copyright (c) 2005 Andrzej Zaborowski  <balrog@zabor.org>
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

#ifndef __LINUX_SPI_TSC2102_H
#define __LINUX_SPI_TSC2102_H

struct apm_power_info;
struct tsc2102_config {
	int use_internal;	/* Use internal reference voltage */
	uint32_t monitor;	/* What inputs are relevant */
	int temp_at25c[2];	/* Thermometer calibration data */
	void (*apm_report)(struct apm_power_info *info, int *battery);
				/* Report status to APM based on battery[] */
	void *alsa_config;	/* .platform_data for the ALSA device */
};

#define TSC_BAT1	(1 << 0)
#define TSC_BAT2	(1 << 1)
#define TSC_AUX		(1 << 2)
#define TSC_TEMP	(1 << 4)

extern u16 tsc2102_read_sync(int page, u8 address);
extern void tsc2102_reads_sync(int page, u8 startaddress, u16 *data,
		int numregs);
extern void tsc2102_write_sync(int page, u8 address, u16 data);

typedef void (*tsc2102_touch_t)(int touching);
typedef void (*tsc2102_coords_t)(int x, int y, int z1, int z2);
typedef void (*tsc2102_ports_t)(int bat1, int bat2, int aux);
typedef void (*tsc2102_temp_t)(int temp);
extern int tsc2102_touch_cb(tsc2102_touch_t handler);
extern int tsc2102_coords_cb(tsc2102_coords_t handler);
extern int tsc2102_ports_cb(tsc2102_ports_t handler);
extern int tsc2102_temp1_cb(tsc2102_temp_t handler);
extern int tsc2102_temp2_cb(tsc2102_temp_t handler);

#ifdef CONFIG_SOUND
extern void tsc2102_set_volume(uint8_t left_ch, uint8_t right_ch);
extern void tsc2102_set_mute(int left_ch, int right_ch);
extern void tsc2102_get_mute(int *left_ch, int *right_ch);
extern void tsc2102_dac_power(int state);
extern int tsc2102_set_rate(int rate);
extern void tsc2102_set_i2s_master(int state);
extern void tsc2102_set_deemphasis(int enable);
extern void tsc2102_set_bassboost(int enable);
#endif

extern void tsc2102_keyclick(int amplitude, int freq, int length);

#define TSC2102_REG(pg, addr)		pg, addr

/* Page 0, Touch Screen & Keypad Data registers */
#define TSC2102_TS_X			TSC2102_REG(0, 0x00)
#define TSC2102_TS_Y			TSC2102_REG(0, 0x01)
#define TSC2102_TS_Z1			TSC2102_REG(0, 0x02)
#define TSC2102_TS_Z2			TSC2102_REG(0, 0x03)
#define TSC2102_TS_BAT1			TSC2102_REG(0, 0x05)
#define TSC2102_TS_BAT2			TSC2102_REG(0, 0x06)
#define TSC2102_TS_AUX			TSC2102_REG(0, 0x07)
#define TSC2102_TS_TEMP1		TSC2102_REG(0, 0x09)
#define TSC2102_TS_TEMP2		TSC2102_REG(0, 0x0a)

/* Page 1, Touch Screen & Keypad Control registers */
#define TSC2102_TS_ADC_CTRL		TSC2102_REG(1, 0x00)
#define TSC2102_TS_STATUS_CTRL		TSC2102_REG(1, 0x01)
#define TSC2102_TS_REF_CTRL		TSC2102_REG(1, 0x03)
#define TSC2102_TS_RESET_CTRL		TSC2102_REG(1, 0x04)
#define TSC2102_TS_CONFIG_CTRL		TSC2102_REG(1, 0x05)

/* Page 2, Audio Control registers */
#define TSC2102_AUDIO1_CTRL		TSC2102_REG(2, 0x00)
#define TSC2102_DAC_GAIN_CTRL		TSC2102_REG(2, 0x02)
#define TSC2102_AUDIO2_CTRL		TSC2102_REG(2, 0x04)
#define TSC2102_DAC_POWER_CTRL		TSC2102_REG(2, 0x05)
#define TSC2102_AUDIO3_CTRL		TSC2102_REG(2, 0x06)
#define TSC2102_LCH_BASS_BOOST_N0	TSC2102_REG(2, 0x07)
#define TSC2102_LCH_BASS_BOOST_N1	TSC2102_REG(2, 0x08)
#define TSC2102_LCH_BASS_BOOST_N2	TSC2102_REG(2, 0x09)
#define TSC2102_LCH_BASS_BOOST_N3	TSC2102_REG(2, 0x0a)
#define TSC2102_LCH_BASS_BOOST_N4	TSC2102_REG(2, 0x0b)
#define TSC2102_LCH_BASS_BOOST_N5	TSC2102_REG(2, 0x0c)
#define TSC2102_LCH_BASS_BOOST_D1	TSC2102_REG(2, 0x0d)
#define TSC2102_LCH_BASS_BOOST_D2	TSC2102_REG(2, 0x0e)
#define TSC2102_LCH_BASS_BOOST_D4	TSC2102_REG(2, 0x0f)
#define TSC2102_LCH_BASS_BOOST_D5	TSC2102_REG(2, 0x10)
#define TSC2102_RCH_BASS_BOOST_N0	TSC2102_REG(2, 0x11)
#define TSC2102_RCH_BASS_BOOST_N1	TSC2102_REG(2, 0x12)
#define TSC2102_RCH_BASS_BOOST_N2	TSC2102_REG(2, 0x13)
#define TSC2102_RCH_BASS_BOOST_N3	TSC2102_REG(2, 0x14)
#define TSC2102_RCH_BASS_BOOST_N4	TSC2102_REG(2, 0x15)
#define TSC2102_RCH_BASS_BOOST_N5	TSC2102_REG(2, 0x16)
#define TSC2102_RCH_BASS_BOOST_D1	TSC2102_REG(2, 0x17)
#define TSC2102_RCH_BASS_BOOST_D2	TSC2102_REG(2, 0x18)
#define TSC2102_RCH_BASS_BOOST_D4	TSC2102_REG(2, 0x19)
#define TSC2102_RCH_BASS_BOOST_D5	TSC2102_REG(2, 0x1a)
#define TSC2102_PLL1_CTRL		TSC2102_REG(2, 0x1b)
#define TSC2102_PLL2_CTRL		TSC2102_REG(2, 0x1c)
#define TSC2102_AUDIO4_CTRL		TSC2102_REG(2, 0x1d)

/* Field masks for Audio Control 1 */
#define AC1_WLEN(ARG)			(((ARG) & 0x03) << 10)
#define AC1_DATFM(ARG)			(((ARG) & 0x03) << 8)
#define AC1_DACFS(ARG)			((ARG) & 0x3f)

/* Field masks for TSC2102_DAC_GAIN_CTRL */
#define DGC_DALMU			(1 << 15)
#define DGC_DALVL(ARG)			(((ARG) & 0x7f) << 8)
#define DGC_DARMU			(1 << 7)
#define DGC_DARVL(ARG)			(((ARG) & 0x7f))

/* Field formats for TSC2102_AUDIO2_CTRL */
#define AC2_KCLEN			(1 << 15)
#define AC2_KCLAC(ARG)			(((ARG) & 0x07) << 12)
#define AC2_KCLFRQ(ARG)			(((ARG) & 0x07) << 8)
#define AC2_KCLLN(ARG)			(((ARG) & 0x0f) << 4)
#define AC2_DLGAF			(1 << 3)
#define AC2_DRGAF			(1 << 2)
#define AC2_DASTC			(1 << 1)

/* Field masks for TSC2102_DAC_POWER_CTRL */
#define CPC_PWDNC			(1 << 15)
#define CPC_DAODRC			(1 << 12)
#define CPC_DAPWDN			(1 << 10)
#define CPC_VGPWDN			(1 << 8)
#define CPC_DAPWDF			(1 << 6)
#define CPC_BASSBC			(1 << 1)
#define CPC_DEEMPF			(0x01)

/* Field masks for TSC2101_AUDIO_CTRL_3 */
#define AC3_DMSVOL(ARG)			(((ARG) & 0x03) << 14)
#define AC3_REFFS			(1 << 13)
#define AC3_DAXFM			(1 << 12)
#define AC3_SLVMS			(1 << 11)
#define AC3_DALOVF			(1 << 7)
#define AC3_DAROVF			(1 << 6)
#define AC3_REVID(ARG)			(((ARG) & 0x07))

/* Field masks for TSC2102_PLL1_CTRL */
#define PLL1_PLLEN			(1 << 15)
#define PLL1_Q_VAL(ARG)			(((ARG) & 0x0f) << 11)
#define PLL1_P_VAL(ARG)			(((ARG) & 0x07) << 8)
#define PLL1_I_VAL(ARG)			(((ARG) & 0x3f) << 2)

/* Field masks for TSC2102_PLL2_CTRL */
#define PLL2_D_VAL(ARG)			(((ARG) & 0x3fff) << 2)

/* Field masks for TSC2101_AUDIO_CTRL_4 */
#define AC4_DASTPD			(1 << 14)

struct tsc2102_rate_info_s {
	u16 sample_rate;
	u8 divisor;
	u8 fs_44k;	/* 44.1 kHz Fsref if 1, 48 kHz if 0 */
};

#endif	/* __LINUX_SPI_TSC2102_H */
