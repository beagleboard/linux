/*
 * linux/sound/oss/omap-audio-tsc2101.c
 *
 * Glue driver for TSC2101 for OMAP processors
 *
 * Copyright (C) 2004 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *  -------
 *  2004-08-12 Nishanth Menon - Modified to integrate Audio requirements on 1610,1710 platforms.
 *  2004-09-14 Sriram Kannan - Added /proc support for asynchronous starting/stopping the codec
 *		(without affecting the normal driver flow).
 *  2004-11-04 Nishanth Menon - Support for power management
 *  2004-11-07 Nishanth Menon - Support for Common TSC access b/w Touchscreen and audio drivers
 */

/***************************** INCLUDES ************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/mutex.h>

#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/arch/dma.h>
#include <asm/io.h>
#include <asm/hardware.h>

#include <asm/arch/mux.h>
#include <asm/arch/io.h>
#include <asm/mach-types.h>

#include "omap-audio.h"
#include "omap-audio-dma-intfc.h"
#include <asm/arch/mcbsp.h>
#ifdef CONFIG_ARCH_OMAP16XX
#include <../drivers/ssi/omap-uwire.h>
#include <asm/arch/dsp_common.h>
#elif defined(CONFIG_ARCH_OMAP24XX)
#else
#error "Unsupported configuration"
#endif

#include <asm/hardware/tsc2101.h>
#include <../drivers/ssi/omap-tsc2101.h>

/***************************** MACROS ************************************/

#define PROC_SUPPORT

#ifdef PROC_SUPPORT
#include <linux/proc_fs.h>
#define PROC_START_FILE "driver/tsc2101-audio-start"
#define PROC_STOP_FILE  "driver/tsc2101-audio-stop"
#endif

#define CODEC_NAME		 "TSC2101"

#ifdef CONFIG_ARCH_OMAP16XX
#define PLATFORM_NAME "OMAP16XX"
#elif defined(CONFIG_ARCH_OMAP24XX)
#define PLATFORM_NAME "OMAP2"
#endif

/* Define to set the tsc as the master w.r.t McBSP */
#define TSC_MASTER

/*
 * AUDIO related MACROS
 */
#define DEFAULT_BITPERSAMPLE          16
#define AUDIO_RATE_DEFAULT	          44100
#define PAGE2_AUDIO_CODEC_REGISTERS   (2)
#define LEAVE_CS 			          0x80

/* Select the McBSP For Audio */
/* 16XX is MCBSP1 and 24XX is MCBSP2*/
/* see include/asm-arm/arch-omap/mcbsp.h */
#ifndef AUDIO_MCBSP
#error "UnSupported Configuration"
#endif

#define REC_MASK 			          (SOUND_MASK_LINE | SOUND_MASK_MIC)
#define DEV_MASK 			          (REC_MASK | SOUND_MASK_VOLUME)

#define SET_VOLUME 			          1
#define SET_LINE   			          2
#define SET_MIC	      3
#define SET_RECSRC	      4

#define DEFAULT_VOLUME                93
#define DEFAULT_INPUT_VOLUME	  20	/* An minimal volume */

/* Tsc Audio Specific */
#define NUMBER_SAMPLE_RATES_SUPPORTED 16
#define OUTPUT_VOLUME_MIN 0x7F
#define OUTPUT_VOLUME_MAX 0x32
#define OUTPUT_VOLUME_RANGE           (OUTPUT_VOLUME_MIN - OUTPUT_VOLUME_MAX)
#define OUTPUT_VOLUME_MASK            OUTPUT_VOLUME_MIN
#define DEFAULT_VOLUME_LEVEL          OUTPUT_VOLUME_MAX

/* use input vol of 75 for 0dB gain */
#define INPUT_VOLUME_MIN 		      0x0
#define INPUT_VOLUME_MAX	 0x7D
#define INPUT_VOLUME_RANGE 		      (INPUT_VOLUME_MAX - INPUT_VOLUME_MIN)
#define INPUT_VOLUME_MASK 		      INPUT_VOLUME_MAX

/*********** Debug Macros ********/
/* To Generate a rather shrill tone -test the entire path */
//#define TONE_GEN
/* To Generate a tone for each keyclick - test the tsc,spi paths*/
//#define TEST_KEYCLICK
/* To dump the tsc registers for debug */
//#define TSC_DUMP_REGISTERS

#ifdef DPRINTK
#undef DPRINTK
#endif
#undef DEBUG

//#define DEBUG
#ifdef DEBUG
#define DPRINTK(ARGS...)  printk(KERN_INFO "<%s>: ",__FUNCTION__);printk(ARGS)
#define FN_IN printk(KERN_INFO "[%s]: start\n", __FUNCTION__)
#define FN_OUT(n) printk(KERN_INFO "[%s]: end(%u)\n",__FUNCTION__, n)
#else
#define DPRINTK( x... )
#define FN_IN
#define FN_OUT(n)
#endif

/***************************** Data Structures **********************************/

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

static audio_stream_t output_stream = {
	.id			= "TSC2101 out",
	.dma_dev		= AUDIO_DMA_TX,
	.input_or_output	= FMODE_WRITE,
	.hw_start		= audio_ifc_start,
	.hw_stop		= audio_ifc_stop,
};

static audio_stream_t input_stream = {
	.id			= "TSC2101 in",
	.dma_dev		= AUDIO_DMA_RX,
	.input_or_output	= FMODE_READ,
	.hw_start		= audio_ifc_start,
	.hw_stop		= audio_ifc_stop,
};

static int audio_dev_id, mixer_dev_id;

typedef struct {
	u8	volume;
	u8	line;
	u8	mic;
	int	recsrc;
	int	mod_cnt;
} tsc2101_local_info;

static tsc2101_local_info tsc2101_local = {
	volume:		DEFAULT_VOLUME,
	line:		DEFAULT_INPUT_VOLUME,
	mic:		DEFAULT_INPUT_VOLUME,
	recsrc:		SOUND_MASK_LINE,
	mod_cnt:	0
};

struct sample_rate_reg_info {
	u16 sample_rate;
	u8  divisor;
	u8  fs_44kHz;		/* if 0 48 khz, if 1 44.1 khz fsref */
};

/* To Store the default sample rate */
static long audio_samplerate = AUDIO_RATE_DEFAULT;

static const struct sample_rate_reg_info
 reg_info[NUMBER_SAMPLE_RATES_SUPPORTED] = {
	/* Div 1 */
	{48000, 0, 0},
	{44100, 0, 1},
	/* Div 1.5 */
	{32000, 1, 0},
	{29400, 1, 1},
	/* Div 2 */
	{24000, 2, 0},
	{22050, 2, 1},
	/* Div 3 */
	{16000, 3, 0},
	{14700, 3, 1},
	/* Div 4 */
	{12000, 4, 0},
	{11025, 4, 1},
	/* Div 5 */
	{9600, 5, 0},
	{8820, 5, 1},
	/* Div 5.5 */
	{8727, 6, 0},
	{8018, 6, 1},
	/* Div 6 */
	{8000, 7, 0},
	{7350, 7, 1},
};

static struct omap_mcbsp_reg_cfg initial_config = {
	.spcr2 = FREE | FRST | GRST | XRST | XINTM(3),
	.spcr1 = RINTM(3) | RRST,
	.rcr2  = RPHASE | RFRLEN2(OMAP_MCBSP_WORD_8) |
	         RWDLEN2(OMAP_MCBSP_WORD_16) | RDATDLY(1),
	.rcr1  = RFRLEN1(OMAP_MCBSP_WORD_8) | RWDLEN1(OMAP_MCBSP_WORD_16),
	.xcr2  = XPHASE | XFRLEN2(OMAP_MCBSP_WORD_8) |
	         XWDLEN2(OMAP_MCBSP_WORD_16) | XDATDLY(1) | XFIG,
	.xcr1  = XFRLEN1(OMAP_MCBSP_WORD_8) | XWDLEN1(OMAP_MCBSP_WORD_16),
	.srgr1 = FWID(15),
	.srgr2 = GSYNC | CLKSP | FSGM | FPER(31),

	/* platform specific initialization */
#ifdef CONFIG_MACH_OMAP_H2
	.pcr0  = CLKXM | CLKRM | FSXP | FSRP | CLKXP | CLKRP,
#elif defined(CONFIG_MACH_OMAP_H3) || defined(CONFIG_MACH_OMAP_H4) || defined(CONFIG_MACH_OMAP_APOLLON)

#ifndef TSC_MASTER
	.pcr0  = FSXM | FSRM | CLKXM | CLKRM | CLKXP | CLKRP,
#else
	.pcr0  = CLKRM | SCLKME | FSXP | FSRP | CLKXP | CLKRP,
#endif				/* tsc Master defs */

#endif				/* platform specific inits */
};

/***************************** MODULES SPECIFIC FUNCTION PROTOTYPES ********************/

static void omap_tsc2101_initialize(void *dummy);

static void omap_tsc2101_shutdown(void *dummy);

static int  omap_tsc2101_ioctl(struct inode *inode, struct file *file,
			       uint cmd, ulong arg);

static int  omap_tsc2101_probe(void);

static void omap_tsc2101_remove(void);

static int  omap_tsc2101_suspend(void);

static int  omap_tsc2101_resume(void);

static void tsc2101_configure(void);

static int  mixer_open(struct inode *inode, struct file *file);

static int  mixer_release(struct inode *inode, struct file *file);

static int  mixer_ioctl(struct inode *inode, struct file *file, uint cmd,
		        ulong arg);

#ifdef TEST_KEYCLICK
void tsc2101_testkeyclick(void);
#endif

#ifdef TONE_GEN
void toneGen(void);
#endif

#ifdef TSC_DUMP_REGISTERS
static void tsc2101_dumpRegisters(void);
#endif

#ifdef PROC_SUPPORT
static int codec_start(char *buf, char **start, off_t offset, int count,
		       int *eof, void *data);

static int codec_stop(char *buf, char **start, off_t offset, int count,
		      int *eof, void *data);

static void tsc2101_start(void);
#endif

/******************** DATA STRUCTURES USING FUNCTION POINTERS **************************/

/* File Op structure for mixer */
static struct file_operations omap_mixer_fops = {
	.open           = mixer_open,
	.release        = mixer_release,
	.ioctl          = mixer_ioctl,
	.owner          = THIS_MODULE
};

/* To store characteristic info regarding the codec for the audio driver */
static audio_state_t tsc2101_state = {
	.output_stream  = &output_stream,
	.input_stream   = &input_stream,
/*	.need_tx_for_rx = 1, //Once the Full Duplex works  */
	.need_tx_for_rx = 0,
	.hw_init        = omap_tsc2101_initialize,
	.hw_shutdown    = omap_tsc2101_shutdown,
	.client_ioctl   = omap_tsc2101_ioctl,
	.hw_probe       = omap_tsc2101_probe,
	.hw_remove      = omap_tsc2101_remove,
	.hw_suspend     = omap_tsc2101_suspend,
	.hw_resume      = omap_tsc2101_resume,
};

/* This will be defined in the Audio.h */
static struct file_operations *omap_audio_fops;

/***************************** MODULES SPECIFIC FUNCTIONs *******************************/

/*********************************************************************************
 *
 * Simplified write for tsc Audio
 *
 *********************************************************************************/
static __inline__ void audio_tsc2101_write(u8 address, u16 data)
{
	omap_tsc2101_write(PAGE2_AUDIO_CODEC_REGISTERS, address, data);
}

/*********************************************************************************
 *
 * Simplified read for tsc  Audio
 *
 *********************************************************************************/
static __inline__ u16 audio_tsc2101_read(u8 address)
{
	return (omap_tsc2101_read(PAGE2_AUDIO_CODEC_REGISTERS, address));
}

/*********************************************************************************
 *
 * tsc2101_update()
 * Volume Adj etc
 *
 ********************************************************************************/
static int tsc2101_update(int flag, int val)
{
	u16 volume;
	u16 data;

	FN_IN;
	switch (flag) {
	case SET_VOLUME:
		if (val < 0 || val > 100) {
			printk(KERN_ERR "Trying a bad volume value(%d)!\n", val);
			return -EPERM;
		}
		/* Convert 0 -> 100 volume to 0x7F(min) -> y(max) volume range */
		volume =
		    ((val * OUTPUT_VOLUME_RANGE) / 100) + OUTPUT_VOLUME_MAX;
		/* invert the value for getting the proper range 0 min and 100 max */
		volume = OUTPUT_VOLUME_MIN - volume;
		data = audio_tsc2101_read(TSC2101_DAC_GAIN_CTRL);
		data &=
		    ~(DGC_DALVL(OUTPUT_VOLUME_MIN) |
		      DGC_DARVL(OUTPUT_VOLUME_MIN));
		data |= DGC_DALVL(volume) | DGC_DARVL(volume);
		audio_tsc2101_write(TSC2101_DAC_GAIN_CTRL, data);
		data = audio_tsc2101_read(TSC2101_DAC_GAIN_CTRL);

		break;

	case SET_LINE:
		if (val < 0 || val > 100) {
			printk(KERN_ERR "Trying a bad volume value(%d)!\n", val);
			return -EPERM;
		}
		/* Convert 0 -> 100 volume to 0x0(min) -> 0x7D(max) volume range */
		/* NOTE: 0 is minimum volume and not mute */
		volume = ((val * INPUT_VOLUME_RANGE) / 100) + INPUT_VOLUME_MIN;
		/* Handset Input not muted, AGC for Handset In off */
		audio_tsc2101_write(TSC2101_HEADSET_GAIN_CTRL,
	HGC_ADPGA_HED(volume));
		break;

	case SET_MIC:
		if (val < 0 || val > 100) {
			printk(KERN_ERR "Trying a bad volume value(%d)!\n", val);
			return -EPERM;
		}
		/* Convert 0 -> 100 volume to 0x0(min) -> 0x7D(max) volume range */
		/* NOTE: 0 is minimum volume and not mute */
		volume = ((val * INPUT_VOLUME_RANGE) / 100) + INPUT_VOLUME_MIN;
		/* Handset Input not muted, AGC for Handset In off */
		audio_tsc2101_write(TSC2101_HANDSET_GAIN_CTRL,
	HNGC_ADPGA_HND(volume));
		break;

	case SET_RECSRC:
		/*
		 * If more than one recording device selected,
		 * disable the device that is currently in use.
		 */
		if (hweight32(val) > 1)
			val &= ~tsc2101_local.recsrc;

		data = audio_tsc2101_read(TSC2101_MIXER_PGA_CTRL);
		data &= ~MPC_MICSEL(7); /* clear all MICSEL bits */

		if (val == SOUND_MASK_MIC) {
			data |=  MPC_MICSEL(1);
			audio_tsc2101_write(TSC2101_MIXER_PGA_CTRL, data);
		}
		else if (val == SOUND_MASK_LINE) {
			data |=  MPC_MICSEL(0);
			audio_tsc2101_write(TSC2101_MIXER_PGA_CTRL, data);
		}
		else {
			printk(KERN_WARNING "omap1610-tsc2101: Wrong RECSRC"
	 " value specified\n");
			return -EINVAL;
		}
		tsc2101_local.recsrc = val;
		break;
	default:
		printk(KERN_WARNING "omap1610-tsc2101: Wrong tsc2101_update "
	"flag specified\n");
		break;
	}

	FN_OUT(0);
	return 0;
}

/*********************************************************************************
 *
 * mixer_open()
 *
 ********************************************************************************/
static int mixer_open(struct inode *inode, struct file *file)
{
	/* Any mixer specific initialization */

	/* Initalize the tsc2101 */
	omap_tsc2101_enable();

	return 0;
}

/*********************************************************************************
 *
 * mixer_release()
 *
 ********************************************************************************/
static int mixer_release(struct inode *inode, struct file *file)
{
	/* Any mixer specific Un-initialization */
	omap_tsc2101_disable();

	return 0;
}

/*********************************************************************************
 *
 * mixer_ioctl()
 *
 ********************************************************************************/
static int
mixer_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
	int val;
	int gain;
	int ret = 0;
	int nr = _IOC_NR(cmd);

	/*
	 * We only accept mixer (type 'M') ioctls.
	 */
	FN_IN;
	if (_IOC_TYPE(cmd) != 'M')
		return -EINVAL;

	DPRINTK(" 0x%08x\n", cmd);

	if (cmd == SOUND_MIXER_INFO) {
		struct mixer_info mi;

		strncpy(mi.id, "TSC2101", sizeof(mi.id));
		strncpy(mi.name, "TI TSC2101", sizeof(mi.name));
		mi.modify_counter = tsc2101_local.mod_cnt;
		FN_OUT(1);
		return copy_to_user((void __user *)arg, &mi, sizeof(mi));
	}

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		ret = get_user(val, (int __user *)arg);
		if (ret)
			goto out;

		/* Ignore separate left/right channel for now,
		 * even the codec does support it.
		 */
		gain = val & 255;

		switch (nr) {
		case SOUND_MIXER_VOLUME:
			tsc2101_local.volume = val;
			tsc2101_local.mod_cnt++;
			ret = tsc2101_update(SET_VOLUME, gain);
			break;

		case SOUND_MIXER_LINE:
			tsc2101_local.line = val;
			tsc2101_local.mod_cnt++;
			ret = tsc2101_update(SET_LINE, gain);
			break;

		case SOUND_MIXER_MIC:
			tsc2101_local.mic = val;
			tsc2101_local.mod_cnt++;
			ret = tsc2101_update(SET_MIC, gain);
			break;

		case SOUND_MIXER_RECSRC:
			if ((val & SOUND_MASK_LINE) ||
			    (val & SOUND_MASK_MIC)) {
				if (tsc2101_local.recsrc != val) {
					tsc2101_local.mod_cnt++;
					tsc2101_update(SET_RECSRC, val);
				}
			}
			else {
				ret = -EINVAL;
			}
			break;

		default:
			ret = -EINVAL;
		}
	}

	if (ret == 0 && _IOC_DIR(cmd) & _IOC_READ) {
		ret = 0;

		switch (nr) {
		case SOUND_MIXER_VOLUME:
			val = tsc2101_local.volume;
			val = (tsc2101_local.volume << 8) |
	  tsc2101_local.volume;
			break;
		case SOUND_MIXER_LINE:
			val = (tsc2101_local.line << 8) |
	  tsc2101_local.line;
			break;
		case SOUND_MIXER_MIC:
			val = (tsc2101_local.mic << 8) |
	  tsc2101_local.mic;
			break;
		case SOUND_MIXER_RECSRC:
			val = tsc2101_local.recsrc;
			break;
		case SOUND_MIXER_RECMASK:
			val = REC_MASK;
			break;
		case SOUND_MIXER_DEVMASK:
			val = DEV_MASK;
			break;
		case SOUND_MIXER_CAPS:
			val = 0;
			break;
		case SOUND_MIXER_STEREODEVS:
			val = SOUND_MASK_VOLUME;
			break;
		default:
			val = 0;
			printk(KERN_WARNING "omap1610-tsc2101: unknown mixer "
	 "read ioctl flag specified\n");
			ret = -EINVAL;
			break;
		}

		if (ret == 0)
			ret = put_user(val, (int __user *)arg);
	}
      out:
	FN_OUT(0);
	return ret;

}

/*********************************************************************************
 *
 * omap_set_samplerate()
 *
 ********************************************************************************/
static int omap_set_samplerate(long sample_rate)
{
	u8 count = 0;
	u16 data = 0;
	int clkgdv = 0;
	/* wait for any frame to complete */
	udelay(125);

	/* Search for the right sample rate */
	while ((reg_info[count].sample_rate != sample_rate) &&
	       (count < NUMBER_SAMPLE_RATES_SUPPORTED)) {
		count++;
	}
	if (count == NUMBER_SAMPLE_RATES_SUPPORTED) {
		printk(KERN_ERR "Invalid Sample Rate %d requested\n",
		       (int)sample_rate);
		return -EPERM;
	}

	/* Set AC1 */
	data = audio_tsc2101_read(TSC2101_AUDIO_CTRL_1);
	/*Clear prev settings */
	data &= ~(AC1_DACFS(0x07) | AC1_ADCFS(0x07));
	data |=
	    AC1_DACFS(reg_info[count].divisor) | AC1_ADCFS(reg_info[count].
							   divisor);
	audio_tsc2101_write(TSC2101_AUDIO_CTRL_1, data);

	/* Set the AC3 */
	data = audio_tsc2101_read(TSC2101_AUDIO_CTRL_3);
	/*Clear prev settings */
	data &= ~(AC3_REFFS | AC3_SLVMS);
	data |= (reg_info[count].fs_44kHz) ? AC3_REFFS : 0;
#ifdef TSC_MASTER
	data |= AC3_SLVMS;
#endif				/* #ifdef TSC_MASTER */
	audio_tsc2101_write(TSC2101_AUDIO_CTRL_3, data);

	/* program the PLLs */
	if (reg_info[count].fs_44kHz) {
		/* 44.1 khz - 12 MHz Mclk */
		audio_tsc2101_write(TSC2101_PLL_PROG_1, PLL1_PLLSEL | PLL1_PVAL(1) | PLL1_I_VAL(7));	/* PVAL 1; I_VAL 7 */
		audio_tsc2101_write(TSC2101_PLL_PROG_2, PLL2_D_VAL(0x1490));	/* D_VAL 5264 */
	} else {
		/* 48 khz - 12 Mhz Mclk */
		audio_tsc2101_write(TSC2101_PLL_PROG_1, PLL1_PLLSEL | PLL1_PVAL(1) | PLL1_I_VAL(8));	/* PVAL 1; I_VAL 8 */
		audio_tsc2101_write(TSC2101_PLL_PROG_2, PLL2_D_VAL(0x780));	/* D_VAL 1920 */
	}

	audio_samplerate = sample_rate;

	/* Set the sample rate */
#ifndef TSC_MASTER
	clkgdv =
	    DEFAULT_MCBSP_CLOCK / (sample_rate *
				   (DEFAULT_BITPERSAMPLE * 2 - 1));
	if (clkgdv)
		initial_config.srgr1 =
		    (FWID(DEFAULT_BITPERSAMPLE - 1) | CLKGDV(clkgdv));
	else
		return (1);

	/* Stereo Mode */
	initial_config.srgr2 =
	    (CLKSM | FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1));
#else
	initial_config.srgr1 =
	    (FWID(DEFAULT_BITPERSAMPLE - 1) | CLKGDV(clkgdv));
	initial_config.srgr2 =
	    ((GSYNC | CLKSP | FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1)));

#endif				/* end of #ifdef TSC_MASTER */
	omap_mcbsp_config(AUDIO_MCBSP, &initial_config);

	return 0;
}

/*********************************************************************************
 *
 * omap_tsc2101_initialize() [hw_init() ]
 *
 ********************************************************************************/
static void omap_tsc2101_initialize(void *dummy)
{

	DPRINTK("omap_tsc2101_initialize entry\n");

	/* initialize with default sample rate */
	audio_samplerate = AUDIO_RATE_DEFAULT;

	omap_mcbsp_request(AUDIO_MCBSP);

	/* if configured, then stop mcbsp */
	omap_mcbsp_stop(AUDIO_MCBSP);

	omap_tsc2101_enable();

	omap_mcbsp_config(AUDIO_MCBSP, &initial_config);
	omap_mcbsp_start(AUDIO_MCBSP);
	tsc2101_configure();

#ifdef TEST_KEYCLICK
	tsc2101_testkeyclick();
#endif

#ifdef TONE_GEN
	toneGen();
#endif

	DPRINTK("omap_tsc2101_initialize exit\n");
}

/*********************************************************************************
 *
 * omap_tsc2101_shutdown() [hw_shutdown() ]
 *
 ********************************************************************************/
static void omap_tsc2101_shutdown(void *dummy)
{
	/*
	   Turn off codec after it is done.
	   Can't do it immediately, since it may still have
	   buffered data.

	   Wait 20ms (arbitrary value) and then turn it off.
	 */

	FN_IN;
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(2);

	omap_mcbsp_stop(AUDIO_MCBSP);
	omap_mcbsp_free(AUDIO_MCBSP);

	audio_tsc2101_write(TSC2101_CODEC_POWER_CTRL,
			    ~(CPC_SP1PWDN | CPC_SP2PWDN | CPC_BASSBC));

	omap_tsc2101_disable();

	FN_OUT(0);
}

/*********************************************************************************
 *
 * tsc2101_configure
 *
 ********************************************************************************/
static void tsc2101_configure(void)
{
	FN_IN;

	audio_tsc2101_write(TSC2101_CODEC_POWER_CTRL, 0x0000);

	/*Mute Analog Sidetone */
	/*Select MIC_INHED input for headset */
	/*Cell Phone In not connected */
	audio_tsc2101_write(TSC2101_MIXER_PGA_CTRL,
			    MPC_ASTMU | MPC_ASTG(0x40) | MPC_MICADC);

	/* Set record source */
	tsc2101_update(SET_RECSRC, tsc2101_local.recsrc);

	/* ADC, DAC, Analog Sidetone, cellphone, buzzer softstepping enabled */
	/* 1dB AGC hysteresis */
	/* MICes bias 2V */
	audio_tsc2101_write(TSC2101_AUDIO_CTRL_4, AC4_MB_HED(0));

	/* Set codec output volume */
	audio_tsc2101_write(TSC2101_DAC_GAIN_CTRL, 0x0000);

	/* DAC left and right routed to SPK2 */
	/* SPK1/2 unmuted */
	audio_tsc2101_write(TSC2101_AUDIO_CTRL_5,
			    AC5_DAC2SPK1(3) | AC5_AST2SPK1 | AC5_KCL2SPK1 |
			    AC5_DAC2SPK2(3) | AC5_AST2SPK2 | AC5_KCL2SPK2 |
			    AC5_HDSCPTC);

	/* OUT8P/N muted, CPOUT muted */

	audio_tsc2101_write(TSC2101_AUDIO_CTRL_6,
			    AC6_MUTLSPK | AC6_MUTSPK2 | AC6_LDSCPTC |
			    AC6_VGNDSCPTC);

	/* Headset/Hook switch detect disabled */
	audio_tsc2101_write(TSC2101_AUDIO_CTRL_7, 0x0000);

	/* Left line input volume control */
	tsc2101_update(SET_LINE, tsc2101_local.line);

	/* mic input volume control */
	tsc2101_update(SET_MIC, tsc2101_local.mic);

	/* Left/Right headphone channel volume control */
	/* Zero-cross detect on */
	tsc2101_update(SET_VOLUME, tsc2101_local.volume);

	/* clock configuration */
	omap_set_samplerate(audio_samplerate);

#ifdef TSC_DUMP_REGISTERS
	tsc2101_dumpRegisters();
#endif

	FN_OUT(0);
}

#ifdef PROC_SUPPORT
static void tsc2101_start(void)
{
	FN_IN;

	audio_tsc2101_write(TSC2101_CODEC_POWER_CTRL, 0x0000);

	/*Mute Analog Sidetone */
	/*Select MIC_INHED input for headset */
	/*Cell Phone In not connected */
	audio_tsc2101_write(TSC2101_MIXER_PGA_CTRL,
			    MPC_ASTMU | MPC_ASTG(0x40) | MPC_MICADC);

	/* Set record source */
	tsc2101_update(SET_RECSRC, tsc2101_local.recsrc);

	/* ADC, DAC, Analog Sidetone, cellphone, buzzer softstepping enabled */
	/* 1dB AGC hysteresis */
	/* MICes bias 2V */
	audio_tsc2101_write(TSC2101_AUDIO_CTRL_4, AC4_MB_HED(0));

	/* Set codec output volume */
	audio_tsc2101_write(TSC2101_DAC_GAIN_CTRL, 0x0000);

	/* DAC left and right routed to SPK2 */
	/* SPK1/2 unmuted */
	audio_tsc2101_write(TSC2101_AUDIO_CTRL_5,
			    AC5_DAC2SPK1(3) | AC5_AST2SPK1 | AC5_KCL2SPK1 |
			    AC5_DAC2SPK2(3) | AC5_AST2SPK2 | AC5_KCL2SPK2 |
			    AC5_HDSCPTC);

	/* OUT8P/N muted, CPOUT muted */

	audio_tsc2101_write(TSC2101_AUDIO_CTRL_6,
			    AC6_MUTLSPK | AC6_MUTSPK2 | AC6_LDSCPTC |
			    AC6_VGNDSCPTC);

	/* Headset/Hook switch detect disabled */
	audio_tsc2101_write(TSC2101_AUDIO_CTRL_7, 0x0000);

	/* Left line input volume control */
	tsc2101_update(SET_LINE, tsc2101_local.line);

	/* mic input volume control */
	tsc2101_update(SET_MIC, tsc2101_local.mic);

	/* Left/Right headphone channel volume control */
	/* Zero-cross detect on */
	tsc2101_update(SET_VOLUME, tsc2101_local.volume);

	FN_OUT(0);

}
#endif

/******************************************************************************************
 *
 * All generic ioctl's are handled by audio_ioctl() [File: omap-audio.c]. This
 * routine handles some platform specific ioctl's
 *
 ******************************************************************************************/
static int
omap_tsc2101_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
	long val;
	int ret = 0;

	DPRINTK(" 0x%08x\n", cmd);

	/*
	 * These are platform dependent ioctls which are not handled by the
	 * generic omap-audio module.
	 */
	switch (cmd) {
	case SNDCTL_DSP_STEREO:
		ret = get_user(val, (int __user *)arg);
		if (ret)
			return ret;
		/* the AIC23 is stereo only */
		ret = (val == 0) ? -EINVAL : 1;
		FN_OUT(1);
		return put_user(ret, (int __user *)arg);

	case SNDCTL_DSP_CHANNELS:
	case SOUND_PCM_READ_CHANNELS:
		/* the AIC23 is stereo only */
		FN_OUT(2);
		return put_user(2, (long __user *)arg);

	case SNDCTL_DSP_SPEED:
		ret = get_user(val, (long __user *)arg);
		if (ret)
			break;
		ret = omap_set_samplerate(val);
		if (ret)
			break;
		/* fall through */

	case SOUND_PCM_READ_RATE:
		FN_OUT(3);
		return put_user(audio_samplerate, (long __user *)arg);

	case SOUND_PCM_READ_BITS:
	case SNDCTL_DSP_SETFMT:
	case SNDCTL_DSP_GETFMTS:
		/* we can do 16-bit only */
		FN_OUT(4);
		return put_user(AFMT_S16_LE, (long __user *)arg);

	default:
		/* Maybe this is meant for the mixer (As per OSS Docs) */
		FN_OUT(5);
		return mixer_ioctl(inode, file, cmd, arg);
	}

	FN_OUT(0);
	return ret;
}

/*********************************************************************************
 *
 * module_probe for TSC2101
 *
 ********************************************************************************/
static int omap_tsc2101_probe(void)
{
	FN_IN;

	/* Get the fops from audio oss driver */
	if (!(omap_audio_fops = audio_get_fops())) {
		printk(KERN_ERR "Unable to Get the FOPs of Audio OSS driver\n");
		audio_unregister_codec(&tsc2101_state);
		return -EPERM;
	}

	/* register devices */
	audio_dev_id = register_sound_dsp(omap_audio_fops, -1);
	mixer_dev_id = register_sound_mixer(&omap_mixer_fops, -1);

#ifdef PROC_SUPPORT
	create_proc_read_entry(PROC_START_FILE, 0 /* default mode */ ,
			       NULL /* parent dir */ ,
			       codec_start, NULL /* client data */ );

	create_proc_read_entry(PROC_STOP_FILE, 0 /* default mode */ ,
			       NULL /* parent dir */ ,
			       codec_stop, NULL /* client data */ );
#endif

	/* Announcement Time */
	printk(KERN_INFO PLATFORM_NAME " " CODEC_NAME
	       " Audio support initialized\n");

	FN_OUT(0);
	return 0;
}

/*********************************************************************************
 *
 * Module Remove for TSC2101
 *
 ********************************************************************************/
static void omap_tsc2101_remove(void)
{
	FN_IN;
	/* Un-Register the codec with the audio driver */
	unregister_sound_dsp(audio_dev_id);
	unregister_sound_mixer(mixer_dev_id);

#ifdef PROC_SUPPORT
	remove_proc_entry(PROC_START_FILE, NULL);
	remove_proc_entry(PROC_STOP_FILE, NULL);
#endif
	FN_OUT(0);

}

/*********************************************************************************
 *
 * Module Suspend for TSC2101
 *
 ********************************************************************************/
static int omap_tsc2101_suspend(void)
{

	FN_OUT(0);
	return 0;
}

/*********************************************************************************
 *
 * Module Resume for TSC2101
 *
 ********************************************************************************/
static int omap_tsc2101_resume(void)
{

	FN_OUT(0);
	return 0;
}

/*********************************************************************************
 *
 * module_init for TSC2101
 *
 ********************************************************************************/
static int __init audio_tsc2101_init(void)
{

	int err = 0;
	FN_IN;

	if (machine_is_omap_osk() || machine_is_omap_innovator())
		return -ENODEV;

	mutex_init(&tsc2101_state.mutex);

	/* register the codec with the audio driver */
	if ((err = audio_register_codec(&tsc2101_state))) {
		printk(KERN_ERR
		       "Failed to register TSC driver with Audio OSS Driver\n");
	}
	FN_OUT(err);
	return err;
}

/*********************************************************************************
 *
 * module_exit for TSC2101
 *
 ********************************************************************************/
static void __exit audio_tsc2101_exit(void)
{

	FN_IN;
	(void)audio_unregister_codec(&tsc2101_state);
	FN_OUT(0);
	return;
}

/**************************** DEBUG FUNCTIONS ***********************************/

/*********************************************************************************
 * TEST_KEYCLICK:
 * This is a test to generate various keyclick sound on tsc.
 * verifies if the tsc and the spi interfaces are operational.
 *
 ********************************************************************************/
#ifdef TEST_KEYCLICK
void tsc2101_testkeyclick(void)
{
	u8 freq = 0;
	u16 old_reg_val, reg_val;
	u32 uDummyVal = 0;
	u32 uTryVal = 0;

	old_reg_val = audio_tsc2101_read(TSC2101_AUDIO_CTRL_2);

	/* Keyclick active, max amplitude and longest key click len(32 period) */
	printk(KERN_INFO " TESTING KEYCLICK\n Listen carefully NOW....\n");
	printk(KERN_INFO " OLD REG VAL=0x%x\n", old_reg_val);
	/* try all frequencies */
	for (; freq < 8; freq++) {
		/* Keyclick active, max amplitude and longest key click len(32 period) */
		reg_val = old_reg_val | AC2_KCLAC(0x7) | AC2_KCLLN(0xF);
		uDummyVal = 0;
		uTryVal = 0;
		printk(KERN_INFO "\n\nTrying frequency %d reg val= 0x%x\n",
		       freq, reg_val | AC2_KCLFRQ(freq) | AC2_KCLEN);
		audio_tsc2101_write(TSC2101_AUDIO_CTRL_2,
				    reg_val | AC2_KCLFRQ(freq) | AC2_KCLEN);
		printk("DONE. Wait 10 ms ...\n");
		/* wait till the kclk bit is auto cleared! time out also to be considered. */
		while (audio_tsc2101_read(TSC2101_AUDIO_CTRL_2) & AC2_KCLEN) {
			udelay(3);
			uTryVal++;
			if (uTryVal > 2000) {
				printk(KERN_ERR
				       "KEYCLICK TIMED OUT! freq val=%d, POSSIBLE ERROR!\n",
				       freq);
				printk(KERN_INFO
				       "uTryVal == %d: Read back new reg val= 0x%x\n",
				       uTryVal,
				       audio_tsc2101_read
				       (TSC2101_AUDIO_CTRL_2));
				/* clear */
				audio_tsc2101_write(TSC2101_AUDIO_CTRL_2, 0x00);
				break;
			}
		}
	}
	/* put the old value back */
	audio_tsc2101_write(TSC2101_AUDIO_CTRL_2, old_reg_val);
	printk(KERN_INFO " KEYCLICK TEST COMPLETE\n");

}				/* End of tsc2101_testkeyclick */

#endif				/* TEST_KEYCLICK */

/*********************************************************************************
 * TONEGEN:
 * This is a test to generate a rather unpleasant sound..
 * verifies if the mcbsp is active (requires MCBSP_DIRECT_RW to be active on McBSP)
 *
 ********************************************************************************/
#ifdef TONE_GEN
/* Generates a shrill tone */
u16 tone[] = {
	0x0ce4, 0x0ce4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000,
	0x0CE4, 0x0CE4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000,
	0x0CE4, 0x0CE4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000
};

void toneGen(void)
{
	int count = 0;
	int ret = 0;
	printk(KERN_INFO "TONE GEN TEST :");

	for (count = 0; count < 5000; count++) {
		int bytes;
		for (bytes = 0; bytes < sizeof(tone) / 2; bytes++) {
			ret = omap_mcbsp_pollwrite(AUDIO_MCBSP, tone[bytes]);
			if (ret == -1) {
				/* retry */
				bytes--;
			} else if (ret == -2) {
				printk(KERN_INFO "ERROR:bytes=%d\n", bytes);
				return;
			}
		}
	}
	printk(KERN_INFO "SUCCESS\n");
}

#endif				/* End of TONE_GEN */

/*********************************************************************************
 *
 * TSC_DUMP_REGISTERS:
 * This will dump the entire register set of Page 2 tsc2101. 
 * Useful for major goof ups
 *
 ********************************************************************************/
#ifdef TSC_DUMP_REGISTERS
static void tsc2101_dumpRegisters(void)
{
	int i = 0;
	u16 data = 0;
	printk("TSC 2101 Register dump for Page 2 \n");
	for (i = 0; i < 0x27; i++) {
		data = audio_tsc2101_read(i);
		printk(KERN_INFO "Register[%x]=0x%04x\n", i, data);

	}
}
#endif				/* End of #ifdef TSC_DUMP_REGISTERS */

#ifdef PROC_SUPPORT
static int codec_start(char *buf, char **start, off_t offset, int count,
		       int *eof, void *data)
{
	omap_tsc2101_enable();
	tsc2101_start();
	printk("Codec initialization done.\n");
	return 0;
}
static int codec_stop(char *buf, char **start, off_t offset, int count,
		      int *eof, void *data)
{

	omap_tsc2101_disable();
	audio_tsc2101_write(TSC2101_CODEC_POWER_CTRL,
			    ~(CPC_SP1PWDN | CPC_SP2PWDN | CPC_BASSBC));
	printk("Codec shutdown.\n");
	return 0;
}
#endif

/*********************************************************************************
 *
 * Other misc management, registration etc
 *
 ********************************************************************************/
module_init(audio_tsc2101_init);
module_exit(audio_tsc2101_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION
    ("Glue audio driver for the TI OMAP1610/OMAP1710 TSC2101 codec.");
MODULE_LICENSE("GPL");
