/*
 * ASoC codec driver for sta321mp
 *
 * sound/soc/codecs/sta321mp.c -- ALSA SoC sta321mp codec driver
 *
 * Copyright (C) 2014 Robin Scheibler <fakufaku@gmail.com>
 *
 * Based on sound/soc/codecs/sta529.c by Rajeev Kumar
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _ASOC_STA321MP_H
#define _ASOC_STA321MP_H

/* STA321MP Register offsets */
#define	 STA321MP_CONFA		0x00
#define	 STA321MP_CONFC		0x02
#define	 STA321MP_CONFD		0x03
#define	 STA321MP_CONFE		0x04
#define	 STA321MP_CONFF		0x05
#define	 STA321MP_CONFG		0x06
#define	 STA321MP_CONFH		0x07
#define	 STA321MP_CONFI		0x08

#define	 STA321MP_MMUTE		0x09
#define	 STA321MP_MVOL		0x0A
#define	 STA321MP_C1VOL		0x0B
#define	 STA321MP_C2VOL		0x0C
#define	 STA321MP_C3VOL		0x0D
#define	 STA321MP_C4VOL		0x0E
#define	 STA321MP_C5VOL		0x0F
#define	 STA321MP_C6VOL		0x10
#define	 STA321MP_C7VOL		0x11
#define	 STA321MP_C8VOL		0x12

#define  STA321MP_C1VTMB	0x13
#define  STA321MP_C2VTMB	0x14
#define  STA321MP_C3VTMB	0x15
#define  STA321MP_C4VTMB	0x16
#define  STA321MP_C5VTMB	0x17
#define  STA321MP_C6VTMB	0x18
#define  STA321MP_C7VTMB	0x19
#define  STA321MP_C8VTMB	0x1A

#define  STA321MP_C12IM		0x1B
#define  STA321MP_C34IM		0x1C
#define  STA321MP_C56IM		0x1D
#define  STA321MP_C78IM		0x1E

#define  STA321MP_BQIP		0x28
#define  STA321MP_MXIP		0x29
#define  STA321MP_EQBP		0x2A
#define  STA321MP_TONEBP	0x2B
#define  STA321MP_TONE		0x2C

#define  STA321MP_C12OT		0x33
#define  STA321MP_C34OT		0x34
#define  STA321MP_C56OT		0x35
#define  STA321MP_C78OT		0x36

#define  STA321MP_C12OM		0x37
#define  STA321MP_C34OM		0x38
#define  STA321MP_C56OM		0x39
#define  STA321MP_C78OM		0x3A

#define  STA321MP_MRBIST	0x5C
#define  STA321MP_RCTR1		0x5D
#define  STA321MP_PDMCT		0x5E
#define  STA321MP_RCTR2		0x5F
#define  STA321MP_RCTR3		0x60
#define  STA321MP_RCTR4		0x61
#define  STA321MP_RCTR5		0x62
#define  STA321MP_RCTR6		0x63
#define  STA321MP_RCTR7		0x64
#define  STA321MP_RCTR8		0x65
#define  STA321MP_RCTR9		0x66
#define  STA321MP_RCTR10	0x67
#define  STA321MP_RCTR11	0x68
#define  STA321MP_RCTR12	0x69
#define  STA321MP_RCTR13	0x6A

#define  STA321MP_DPT		0x80
#define  STA321MP_CFR129	0x81
#define  STA321MP_TSDLY1	0x82
#define  STA322MP_TSDLY1	0x83



#define STA321MP_MAX_REGISTER	0x83

#define I2S_S24_LE  0x00
#define I2S_S24_3LE 0x0E
#define I2S_S16_LE  0x07
#define I2S_MSB_1ST 0x00
#define I2S_LSB_1ST 0x10
#define I2S_DIV_4   0x20
#define I2S_DIV_1   0x00

#define PDM_I_EN		0x9B
#define FS_XTI_256		0x20
#define CH78_BIN		0xC0
#define RM_SOFT_VOL		0x7A
#define BRG_PWR_UP		0x80
#define MST_VOL_0DB		0x00
#define MIC_MODE		0x01
#define I2S_OUT			0x09

#define AUDIO_MUTE_MSK		0x01
#define DATA_FORMAT_MSK		0x0F

#define I2S_DATA_FORMAT		0x00
#define CODEC_MUTE_VAL    0x01

#endif /* _ASOC_STA321MP_H */
