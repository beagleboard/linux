/*
 * sound/arm/omap/omap-alsa-tsc2101-mixer.c
 * 
 * Alsa Driver for TSC2101 codec for OMAP platform boards.
 *
 * Copyright (C) 2005 Mika Laitio <lamikr@cc.jyu.fi> and 
 * 		     Everett Coleman II <gcc80x86@fuzzyneural.net>
 *
 * Board initialization code is based on the code in TSC2101 OSS driver.
 * Copyright (C) 2004 Texas Instruments, Inc.
 * 	Written by Nishanth Menon and Sriram Kannan
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
 * 2006-03-01   Mika Laitio - Mixer for the tsc2101 driver used in omap boards.
 * 		Can switch between headset and loudspeaker playback, 
 * 		mute and unmute dgc, set dgc volume. Record source switch,
 * 		keyclick, buzzer and headset volume and handset volume control 
 * 		are still missing.
 * 		
 */
 
#include "omap-alsa-tsc2101.h"
#include "omap-alsa-tsc2101-mixer.h"

#include <linux/types.h>
#include <sound/initval.h>
#include <sound/control.h>

//#define M_DPRINTK(ARGS...)  printk(KERN_INFO "<%s>: ",__FUNCTION__);printk(ARGS)
#define M_DPRINTK(ARGS...)  		/* nop */

#define CHECK_BIT(INDX, ARG) (((ARG) & TSC2101_BIT(INDX)) >> INDX)
#define IS_UNMUTED(INDX, ARG) (((CHECK_BIT(INDX, ARG)) == 0))

#define DGC_DALVL_EXTRACT(ARG) ((ARG & 0x7f00) >> 8)
#define DGC_DARVL_EXTRACT(ARG) ((ARG & 0x007f))

#define HGC_ADPGA_HED_EXTRACT(ARG) ((ARG & 0x7f00) >> 8)
#define HNGC_ADPGA_HND_EXTRACT(ARG) ((ARG & 0x7f00) >> 8)
#define BGC_ADPGA_BGC_EXTRACT(ARG) ((ARG & 0x7f00) >> 8)

static int current_playback_target	= PLAYBACK_TARGET_LOUDSPEAKER;
static int current_rec_src 		= REC_SRC_SINGLE_ENDED_MICIN_HED;

/* 
 * Simplified write for the tsc2101 audio registers.
 */
inline void omap_tsc2101_audio_write(u8 address, u16 data)
{
	omap_tsc2101_write(PAGE2_AUDIO_CODEC_REGISTERS, address, data);
}

/* 
 * Simplified read for the tsc2101 audio registers.
 */
inline u16 omap_tsc2101_audio_read(u8 address)
{
	return (omap_tsc2101_read(PAGE2_AUDIO_CODEC_REGISTERS, address));
}

/*
 * For selecting tsc2101 recourd source.
 */
static void set_record_source(int val)
{
	u16	data;
	
	/* Mute Analog Sidetone
	 * Analog sidetone gain db?
	 * Input selected by MICSEL connected to ADC
	 */
	data	= MPC_ASTMU | MPC_ASTG(0x45);
	data	&= ~MPC_MICSEL(7); /* clear all MICSEL bits */
	data	|= MPC_MICSEL(val);
	data	|= MPC_MICADC;
	omap_tsc2101_audio_write(TSC2101_MIXER_PGA_CTRL, data);
	
	current_rec_src	= val;
}

/*
 * Converts the Alsa mixer volume (0 - 100) to real 
 * Digital Gain Control (DGC) value that can be written
 * or read from the TSC2101 registry.
 * 
 * Note that the number "OUTPUT_VOLUME_MAX" is smaller than OUTPUT_VOLUME_MIN
 * because DGC works as a volume decreaser. (The more bigger value is put
 * to DGC, the more the volume of controlled channel is decreased)
 * 
 * In addition the TCS2101 chip would allow the maximum volume reduction be 63.5 DB
 * but according to some tests user can not hear anything with this chip
 * when the volume is set to be less than 25 db.
 * Therefore this function will return a value that means 38.5 db (63.5 db - 25 db) 
 * reduction in the channel volume, when mixer is set to 0.
 * For mixer value 100, this will return a value that means 0 db volume reduction.
 * ([mute_left_bit]0000000[mute_right_bit]0000000)
*/
int get_mixer_volume_as_dac_gain_control_volume(int vol)
{
	u16 retVal;

	/* Convert 0 -> 100 volume to 0x7F(min) -> y(max) volume range */
	retVal	= ((vol * OUTPUT_VOLUME_RANGE) / 100) + OUTPUT_VOLUME_MAX;
	/* invert the value for getting the proper range 0 min and 100 max */
	retVal	= OUTPUT_VOLUME_MIN - retVal;
	
	return retVal;
}

/*
 * Converts the Alsa mixer volume (0 - 100) to TSC2101 
 * Digital Gain Control (DGC) volume. Alsa mixer volume 0
 * is converted to value meaning the volume reduction of -38.5 db
 * and Alsa mixer volume 100 is converted to value meaning the
 * reduction of 0 db.
 */
int set_mixer_volume_as_dac_gain_control_volume(int mixerVolL, int mixerVolR) 
{
	u16 val;
	int retVal;
	int volL;
	int volR;
	
	if ((mixerVolL < 0) || 
	    (mixerVolL > 100) ||
	    (mixerVolR < 0) ||
	    (mixerVolR > 100)) {
		printk(KERN_ERR "Trying a bad mixer volume as dac gain control volume value, left (%d), right (%d)!\n", mixerVolL, mixerVolR);
		return -EPERM;
	}
	M_DPRINTK("mixer volume left = %d, right = %d\n", mixerVolL, mixerVolR);	
	volL	= get_mixer_volume_as_dac_gain_control_volume(mixerVolL);
	volR	= get_mixer_volume_as_dac_gain_control_volume(mixerVolR);
	
	val	= omap_tsc2101_audio_read(TSC2101_DAC_GAIN_CTRL);
	/* keep the old mute bit settings */
	val	&= ~(DGC_DALVL(OUTPUT_VOLUME_MIN) | DGC_DARVL(OUTPUT_VOLUME_MIN));
	val	|= DGC_DALVL(volL) | DGC_DARVL(volR);
	retVal	= 2;
	if (retVal) {
		omap_tsc2101_audio_write(TSC2101_DAC_GAIN_CTRL, val);
	}
	M_DPRINTK("to registry: left = %d, right = %d, total = %d\n", DGC_DALVL_EXTRACT(val), DGC_DARVL_EXTRACT(val), val);
	return retVal;
}

/**
 * If unmuteLeft/unmuteRight == 0  --> mute
 * If unmuteLeft/unmuteRight == 1 --> unmute
 */
int dac_gain_control_unmute(int unmuteLeft, int unmuteRight)
{
	u16 val;
	int count;

	count	= 0;
	val	= omap_tsc2101_audio_read(TSC2101_DAC_GAIN_CTRL);
	/* in alsa mixer 1 --> on, 0 == off. In tsc2101 registry 1 --> off, 0 --> on
	 * so if values are same, it's time to change the registry value.
	 */
	if (unmuteLeft != IS_UNMUTED(15, val)) {
		if (unmuteLeft == 0) {
			/* mute --> turn bit on */
			val	= val | DGC_DALMU;
		}
		else {
			/* unmute --> turn bit off */
			val	= val & ~DGC_DALMU;
		}
		count++;
	} /* L */
	if (unmuteRight != IS_UNMUTED(7, val)) {
		if (unmuteRight == 0) {
			/* mute --> turn bit on */
			val	= val | DGC_DARMU;
		}
		else {
			/* unmute --> turn bit off */
			val	= val & ~DGC_DARMU;
		}		
		count++;
	} /* R */
	if (count) {
		omap_tsc2101_audio_write(TSC2101_DAC_GAIN_CTRL, val);
		M_DPRINTK("changed value, is_unmuted left = %d, right = %d\n", 
			IS_UNMUTED(15, val),
			IS_UNMUTED(7, val));
	}
	return count;	
}

/**
 * unmute: 0 --> mute, 1 --> unmute
 * page2RegIndx: Registry index in tsc2101 page2.
 * muteBitIndx: Index number for the bit in registry that indicates whether muted or unmuted.
 */
int adc_pga_unmute_control(int unmute, int page2regIndx, int muteBitIndx)
{
	int count;
	u16 val;
	
	count	= 0;
	val 	= omap_tsc2101_audio_read(page2regIndx);
	/* in alsa mixer 1 --> on, 0 == off. In tsc2101 registry 1 --> off, 0 --> on
	 * so if the values are same, it's time to change the registry value...
	 */
	if (unmute != IS_UNMUTED(muteBitIndx, val)) {
		if (unmute == 0) {
			/* mute --> turn bit on */
			val	= val | TSC2101_BIT(muteBitIndx);
		}
		else {
			/* unmute --> turn bit off */
			val	= val & ~TSC2101_BIT(muteBitIndx);
		}
		M_DPRINTK("changed value, is_unmuted = %d\n", IS_UNMUTED(muteBitIndx, val));
		count++;
	}
	if (count) {
		omap_tsc2101_audio_write(page2regIndx, val);
	}
	return count;
}

/*
 * Converts the DGC registry value read from the TSC2101 registry to 
 * Alsa mixer volume format (0 - 100).
 */
int get_dac_gain_control_volume_as_mixer_volume(u16 vol) 
{
	u16 retVal;	

	retVal	= OUTPUT_VOLUME_MIN - vol;
	retVal	= ((retVal - OUTPUT_VOLUME_MAX) * 100) / OUTPUT_VOLUME_RANGE;
	/* fix scaling error */
	if ((retVal > 0) && (retVal < 100)) {
		retVal++;
	}
	return retVal;
}

/*
 * Converts the headset gain control volume (0 - 63.5 db)
 * to Alsa mixer volume (0 - 100)
 */
int get_headset_gain_control_volume_as_mixer_volume(u16 registerVal) 
{
	u16 retVal;
	
	retVal	= ((registerVal * 100) / INPUT_VOLUME_RANGE);
	return retVal;
}

/*
 * Converts the handset gain control volume (0 - 63.5 db)
 * to Alsa mixer volume (0 - 100)
 */
int get_handset_gain_control_volume_as_mixer_volume(u16 registerVal) 
{
	return get_headset_gain_control_volume_as_mixer_volume(registerVal);
}

/*
 * Converts the Alsa mixer volume (0 - 100) to 
 * headset gain control volume (0 - 63.5 db)
 */
int get_mixer_volume_as_headset_gain_control_volume(u16 mixerVal) 
{
	u16 retVal;
	
	retVal	= ((mixerVal * INPUT_VOLUME_RANGE) / 100) + INPUT_VOLUME_MIN;	
	return retVal;
}

/*
 * Writes Alsa mixer volume (0 - 100) to TSC2101 headset volume registry in
 * a TSC2101 format. (0 - 63.5 db)
 * In TSC2101 OSS driver this functionality was controlled with "SET_LINE" parameter.
 */
int set_mixer_volume_as_headset_gain_control_volume(int mixerVol) 
{
	int volume;
	int retVal;
	u16 val;

	if (mixerVol < 0 || mixerVol > 100) {
		M_DPRINTK("Trying a bad headset mixer volume value(%d)!\n", mixerVol);
		return -EPERM;
	}
	M_DPRINTK("mixer volume = %d\n", mixerVol);
	/* Convert 0 -> 100 volume to 0x0(min) -> 0x7D(max) volume range */
	/* NOTE: 0 is minimum volume and not mute */
	volume	= get_mixer_volume_as_headset_gain_control_volume(mixerVol);	
	val	= omap_tsc2101_audio_read(TSC2101_HEADSET_GAIN_CTRL);
	/* preserve the old mute settings */
	val	&= ~(HGC_ADPGA_HED(INPUT_VOLUME_MAX));
	val	|= HGC_ADPGA_HED(volume);
	omap_tsc2101_audio_write(TSC2101_HEADSET_GAIN_CTRL, val);	
	retVal	= 1;
	
	M_DPRINTK("to registry = %d\n", val);	
	return retVal;
}

/*
 * Writes Alsa mixer volume (0 - 100) to TSC2101 handset volume registry in
 * a TSC2101 format. (0 - 63.5 db)
 * In TSC2101 OSS driver this functionality was controlled with "SET_MIC" parameter.
 */
int set_mixer_volume_as_handset_gain_control_volume(int mixerVol) 
{
	int volume;
	int retVal;
	u16 val;	

	if (mixerVol < 0 || mixerVol > 100) {
		M_DPRINTK("Trying a bad mic mixer volume value(%d)!\n", mixerVol);
		return -EPERM;
	}
	M_DPRINTK("mixer volume = %d\n", mixerVol);
	/* Convert 0 -> 100 volume to 0x0(min) -> 0x7D(max) volume range
	 * NOTE: 0 is minimum volume and not mute 
	 */
	volume	= get_mixer_volume_as_headset_gain_control_volume(mixerVol);
	val	= omap_tsc2101_audio_read(TSC2101_HANDSET_GAIN_CTRL);
	/* preserve the old mute settigns */
	val	&= ~(HNGC_ADPGA_HND(INPUT_VOLUME_MAX));
	val	|= HNGC_ADPGA_HND(volume);
	omap_tsc2101_audio_write(TSC2101_HANDSET_GAIN_CTRL, val);
	retVal	= 1;
	
	M_DPRINTK("to registry = %d\n", val);	
	return retVal;
}

void set_loudspeaker_to_playback_target(void)
{
	/* power down SPK1, SPK2 and loudspeaker */
	omap_tsc2101_audio_write(TSC2101_CODEC_POWER_CTRL,
			CPC_SP1PWDN | CPC_SP2PWDN | CPC_LDAPWDF);	
	/* ADC, DAC, Analog Sidetone, cellphone, buzzer softstepping enabled
	 * 1dB AGC hysteresis
	 * MICes bias 2V
	 */
	omap_tsc2101_audio_write(TSC2101_AUDIO_CTRL_4, AC4_MB_HED(0));

	/* DAC left and right routed to SPK1/SPK2
	 * SPK1/SPK2 unmuted
	 * Keyclicks routed to SPK1/SPK2 */
	omap_tsc2101_audio_write(TSC2101_AUDIO_CTRL_5, 
			AC5_DIFFIN |
			AC5_DAC2SPK1(3) | AC5_AST2SPK1 | AC5_KCL2SPK1 |
			AC5_DAC2SPK2(3) | AC5_AST2SPK2 | AC5_KCL2SPK2);
	
	/* routing selected to SPK1 goes also to OUT8P/OUT8N. (loudspeaker)
	 * analog sidetone routed to loudspeaker
	 * buzzer pga routed to loudspeaker
	 * keyclick routing to loudspeaker
	 * cellphone input routed to loudspeaker
	 * mic selection (control register 04h/page2) routed to cell phone output (CP_OUT)
	 * routing selected for SPK1 goes also to cellphone output (CP_OUT)
	 * OUT8P/OUT8N (loudspeakers) unmuted (0 = unmuted)
	 * Cellphone output is not muted (0 = unmuted)
	 * Enable loudspeaker short protection control (0 = enable protection)
	 * VGND short protection control (0 = enable protection)
	 */
	omap_tsc2101_audio_write(TSC2101_AUDIO_CTRL_6,
			AC6_SPL2LSK | AC6_AST2LSK | AC6_BUZ2LSK | AC6_KCL2LSK |
			AC6_CPI2LSK | AC6_MIC2CPO | AC6_SPL2CPO);
	current_playback_target	= PLAYBACK_TARGET_LOUDSPEAKER;
}

void set_headphone_to_playback_target(void)
{
	/* power down SPK1, SPK2 and loudspeaker */
	omap_tsc2101_audio_write(TSC2101_CODEC_POWER_CTRL,
			CPC_SP1PWDN | CPC_SP2PWDN | CPC_LDAPWDF);
	/* ADC, DAC, Analog Sidetone, cellphone, buzzer softstepping enabled */
	/* 1dB AGC hysteresis */
	/* MICes bias 2V */
	omap_tsc2101_audio_write(TSC2101_AUDIO_CTRL_4, AC4_MB_HED(0));
				
	/* DAC left and right routed to SPK1/SPK2
	 * SPK1/SPK2 unmuted
	 * Keyclicks routed to SPK1/SPK2 */
	omap_tsc2101_audio_write(TSC2101_AUDIO_CTRL_5,
			AC5_DAC2SPK1(3) | AC5_AST2SPK1 | AC5_KCL2SPK1 |
			AC5_DAC2SPK2(3) | AC5_AST2SPK2 | AC5_KCL2SPK2 |
			AC5_HDSCPTC);
			
	/* OUT8P/OUT8N muted, CPOUT muted */
	omap_tsc2101_audio_write(TSC2101_AUDIO_CTRL_6,
			AC6_MUTLSPK | AC6_MUTSPK2 | AC6_LDSCPTC |
			AC6_VGNDSCPTC);
	current_playback_target	= PLAYBACK_TARGET_HEADPHONE;
}

void set_telephone_to_playback_target(void)
{
	/* 
	 * 0110 1101 0101 1100
	 * power down MICBIAS_HED, Analog sidetone, SPK2, DAC, 
	 * Driver virtual ground, loudspeaker. Values D2-d5 are flags.
	 */	 
	omap_tsc2101_audio_write(TSC2101_CODEC_POWER_CTRL,
			CPC_MBIAS_HED | CPC_ASTPWD | CPC_SP2PWDN | CPC_DAPWDN |
			CPC_VGPWDN | CPC_LSPWDN);
			
	/* 
	 * 0010 1010 0100 0000
	 * ADC, DAC, Analog Sidetone, cellphone, buzzer softstepping enabled
	 * 1dB AGC hysteresis
	 * MICes bias 2V
	 */
	omap_tsc2101_audio_write(TSC2101_AUDIO_CTRL_4,
			AC4_MB_HND | AC4_MB_HED(0) | AC4_AGCHYS(1) | 
			AC4_BISTPD | AC4_ASSTPD | AC4_DASTPD);
	printk("set_telephone_to_playback_target(), TSC2101_AUDIO_CTRL_4 = %d\n", omap_tsc2101_audio_read(TSC2101_AUDIO_CTRL_4));
			
	/* 
	 * 1110 0010 0000 0010
	 * DAC left and right routed to SPK1/SPK2
	 * SPK1/SPK2 unmuted
	 * keyclicks routed to SPK1/SPK2
	 */	 
	omap_tsc2101_audio_write(TSC2101_AUDIO_CTRL_5,
			AC5_DIFFIN | AC5_DAC2SPK1(3) | 
		  	AC5_CPI2SPK1 | AC5_MUTSPK2);
	
	omap_tsc2101_audio_write(TSC2101_AUDIO_CTRL_6,
			AC6_MIC2CPO | AC6_MUTLSPK | 
			AC6_LDSCPTC | AC6_VGNDSCPTC | AC6_CAPINTF);
	current_playback_target	= PLAYBACK_TARGET_CELLPHONE;
}

/*
 * 1100 0101 1101 0000
 * 
 * #define MPC_ASTMU           TSC2101_BIT(15)
 * #define MPC_ASTG(ARG)       (((ARG) & 0x7F) << 8)
 * #define MPC_MICSEL(ARG)     (((ARG) & 0x07) << 5)
 * #define MPC_MICADC          TSC2101_BIT(4)
 * #define MPC_CPADC           TSC2101_BIT(3)
 * #define MPC_ASTGF           (0x01)
 */
static void set_telephone_to_record_source(void)
{
	u16	val;
	
	/* 
	 * D0       = 0: 
	 * 		--> AGC is off for handset input.
	 *		--> ADC PGA is controlled by the ADMUT_HDN + ADPGA_HND
	 *          (D15, D14-D8)
	 * D4 - D1  = 0000 
	 * 		--> AGC time constant for handset input, 
	 * 		attack time = 8 mc, decay time = 100 ms
	 * D7 - D5  = 000
	 * 		--> AGC Target gain for handset input = -5.5 db
	 * D14 - D8 = 011 1100
	 * 		--> ADC handset PGA settings = 60 = 30 db
	 * D15 		= 0
	 * 		--> Handset input ON (unmuted)
	 */
	val	= 0x3c00;	// 0011 1100 0000 0000 = 60 = 30
	omap_tsc2101_audio_write(TSC2101_HANDSET_GAIN_CTRL, val);
	
	/*
	 * D0		= 0
	 * 		--> AGC is off for headset/Aux input
	 * 		--> ADC headset/Aux PGA is contoller by ADMUT_HED + ADPGA_HED
	 *          (D15, D14-D8)
	 * D4 - D1	= 0000 
	 * 		--> Agc constant for headset/Aux input,
	 *      	attack time = 8 mc, decay time = 100 ms      
	 * D7 - D5	= 000
	 * 		--> AGC target gain for headset input = -5.5 db
	 * D14 - D8 = 000 0000
	 * 		--> Adc headset/AUX pga settings = 0 db
	 * D15		= 1
	 * 		--> Headset/AUX input muted
	 * 
	 * Mute headset aux input
	 */
	val	= 0x8000;	// 1000 0000 0000 0000
	omap_tsc2101_audio_write(TSC2101_HEADSET_GAIN_CTRL, val);
	set_record_source(REC_SRC_MICIN_HND_AND_AUX1);

	// hacks start
	/* D0		= flag, Headset/Aux or handset PGA flag
	 * 		--> & with 1 (= 1 -->gain applied == pga register settings)
	 * D1		= 0, DAC channel PGA soft stepping control
	 * 		--> 0.5 db change every WCLK
	 * D2		= flag, DAC right channel PGA flag
	 * 		--> & with 1
	 * D3		= flag, DAC left channel PGA flag
	 * 		-- > & with 1
	 * D7 - D4	= 0001, keyclick length
	 * 		--> 4 periods key clicks
	 * D10 - D8 = 100, keyclick frequenzy
	 * 		--> 1 kHz, 
	 * D11		= 0, Headset/Aux or handset soft stepping control
	 * 		--> 0,5 db change every WCLK or ADWS
	 * D14 -D12 = 100, Keyclick applitude control
	 * 		--> Medium amplitude
	 * D15		= 0, keyclick disabled
	 */
	val	= omap_tsc2101_audio_read(TSC2101_AUDIO_CTRL_2);
	val	= val & 0x441d;
	val	= val | 0x4410;	// D14, D10, D4 bits == 1
	omap_tsc2101_audio_write(TSC2101_AUDIO_CTRL_2, val);

	/*
	 * D0		= 0	(reserved, write always 0)
	 * D1		= flag,
	 * 			--> & with 1
	 * D2 - D5	= 0000 (reserved, write always 0000)
	 * D6		= 1
	 * 			--> MICBIAS_HND = 2.0 v
	 * D8 - D7	= 00
	 * 			--> MICBIAS_HED = 3.3 v
	 * D10 - D9	= 01, 
	 * 			--> Mic AGC hysteric selection = 2 db
	 * D11		= 1, 
	 * 			--> Disable buzzer PGA soft stepping
	 * D12		= 0,
	 * 			--> Enable CELL phone PGA soft stepping control
	 * D13		= 1
	 * 			--> Disable analog sidetone soft stepping control
	 * D14		= 0
	 * 			--> Enable DAC PGA soft stepping control
	 * D15		= 0,
	 * 			--> Enable headset/Aux or Handset soft stepping control
	 */
	val	= omap_tsc2101_audio_read(TSC2101_AUDIO_CTRL_4);
	val	= val & 0x2a42;	// 0010 1010 0100 0010
	val	= val | 0x2a40;	// bits D13, D11, D9, D6 == 1
	omap_tsc2101_audio_write(TSC2101_AUDIO_CTRL_4, val);
	printk("set_telephone_to_record_source(), TSC2101_AUDIO_CTRL_4 = %d\n", omap_tsc2101_audio_read(TSC2101_AUDIO_CTRL_4));
	/*
	 * D0		= 0
	 * 		--> reserved, write always = 0
	 * D1		= flag, read only
	 * 		--> & with 1
	 * D5 - D2	= 1111, Buzzer input PGA settings
	 * 		--> 0 db
	 * D6		= 1,
	 * 		--> power down buzzer input pga
	 * D7		= flag, read only
	 * 		--> & with 1
	 * D14 - D8	= 101 1101
	 * 		--> 12 DB
	 * D15		= 0
	 * 		--> power up cell phone input PGA
	 */
	val	= omap_tsc2101_audio_read(TSC2101_BUZZER_GAIN_CTRL);
	val	= val & 0x5dfe;
	val	= val | 0x5dfe;	// bits, D14, D12, D11, D10, D8, D6, D5,D4,D3,D2
	omap_tsc2101_audio_write(TSC2101_BUZZER_GAIN_CTRL, val);
	
	/* D6 - D0	= 000 1001
	 * 		--> -4.5 db for DAC right channel volume control
	 * D7		= 1
	 * 		-->  DAC right channel muted
	 * D14 - D8 = 000 1001
	 * 		--> -4.5 db for DAC left channel volume control
	 * D15 		= 1
	 * 		--> DAC left channel muted
	 */
	//val	= omap_tsc2101_audio_read(TSC2101_DAC_GAIN_CTRL);
	val	= 0x8989;
	omap_tsc2101_audio_write(TSC2101_DAC_GAIN_CTRL, val);	
	
	/*  0000 0000 0100 0000
	 * 
	 * D1 - D0	= 0
	 * 		--> GPIO 1 pin output is three stated
	 * D2		= 0
	 * 		--> Disaple GPIO2 for CLKOUT mode
	 * D3		= 0
	 * 		--> Disable GPUI1 for interrupt detection
	 * D4		= 0
	 * 		--> Disable GPIO2 for headset detection interrupt
	 * D5		= reserved, always 0
	 * D7 - D6	= 01
	 * 		--> 8 ms clitch detection
	 * D8		= reserved, write only 0
	 * D10 -D9	= 00
	 * 		--> 16 ms de bouncing programmatitily 
	 *          for glitch detection during headset detection
	 * D11		= flag for button press
	 * D12		= flag for headset detection
	 * D14-D13	= 00
	 * 		--> type of headset detected = 00 == no stereo headset deected
	 * D15		= 0
	 * 		--> Disable headset detection
	 * 
	 * */
	val	= 0x40;
	omap_tsc2101_audio_write(TSC2101_AUDIO_CTRL_7, val);	
}

/*
 * Checks whether the headset is detected.
 * If headset is detected, the type is returned. Type can be
 * 	0x01	= stereo headset detected
 * 	0x02	= cellurar headset detected
 * 	0x03	= stereo + cellurar headset detected
 * If headset is not detected 0 is returned.
 */
u16 get_headset_detected(void)
{
	u16	curDetected;
	u16	curType;
	u16	curVal;
	
	curType	= 0;	/* not detected */
	curVal	= omap_tsc2101_audio_read(TSC2101_AUDIO_CTRL_7);
	curDetected	= curVal & AC7_HDDETFL;
	if (curDetected) {
		printk("headset detected, checking type from %d \n", curVal);
		curType	= ((curVal & 0x6000) >> 13);
		printk("headset type detected = %d \n", curType);
	}
	else {
		printk("headset not detected\n");
	}
	return curType;
}

void init_playback_targets(void)
{
	u16	val;

	set_loudspeaker_to_playback_target();
	/* Left line input volume control
	 * = SET_LINE in the OSS driver
	 */
	set_mixer_volume_as_headset_gain_control_volume(DEFAULT_INPUT_VOLUME);

	/* Set headset to be controllable by handset mixer
	 * AGC enable for handset input
	 * Handset input not muted
	 */
	val	= omap_tsc2101_audio_read(TSC2101_HANDSET_GAIN_CTRL);
	val	= val | HNGC_AGCEN_HND;	
	val	= val & ~HNGC_ADMUT_HND;
	omap_tsc2101_audio_write(TSC2101_HANDSET_GAIN_CTRL, val);	
			
	/* mic input volume control
	 * SET_MIC in the OSS driver 
	 */
	set_mixer_volume_as_handset_gain_control_volume(DEFAULT_INPUT_VOLUME);

	/* Left/Right headphone channel volume control
	 * Zero-cross detect on
	 */
	set_mixer_volume_as_dac_gain_control_volume(DEFAULT_OUTPUT_VOLUME, DEFAULT_OUTPUT_VOLUME);	
	/* unmute */
	dac_gain_control_unmute(1, 1);
}

/*
 * Initializes tsc2101 recourd source (to line) and playback target (to loudspeaker)
 */
void snd_omap_init_mixer(void)
{	
	FN_IN;
	
	/* Headset/Hook switch detect enabled */
	omap_tsc2101_audio_write(TSC2101_AUDIO_CTRL_7, AC7_DETECT);

	/* Select headset to record source (MIC_INHED)*/
	set_record_source(REC_SRC_SINGLE_ENDED_MICIN_HED);
	/* Init loudspeaker as a default playback target*/
	init_playback_targets();

	FN_OUT(0);
}

static int __pcm_playback_target_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo) 
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

static int __pcm_playback_target_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	ucontrol->value.integer.value[0] = current_playback_target;
	return 0;
}

static int __pcm_playback_target_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	int	retVal;
	int	curVal;
	
	retVal	= 0;
	curVal	= ucontrol->value.integer.value[0];
	if ((curVal >= 0) &&
	    (curVal < PLAYBACK_TARGET_COUNT) &&
	    (curVal != current_playback_target)) {		
		if (curVal == PLAYBACK_TARGET_LOUDSPEAKER) {
			set_record_source(REC_SRC_SINGLE_ENDED_MICIN_HED);
			set_loudspeaker_to_playback_target();
		}
		else if (curVal == PLAYBACK_TARGET_HEADPHONE) {
			set_record_source(REC_SRC_SINGLE_ENDED_MICIN_HND);
			set_headphone_to_playback_target();
		}
		else if (curVal == PLAYBACK_TARGET_CELLPHONE) {
			set_telephone_to_record_source();
			set_telephone_to_playback_target();
		}
		retVal	= 1;
	}
	return retVal;
}	

static int __pcm_playback_volume_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo) 
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 2;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 100;
	return 0;
}

/*
 * Alsa mixer interface function for getting the volume read from the DGC in a 
 * 0 -100 alsa mixer format.
 */
static int __pcm_playback_volume_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	u16 volL;
	u16 volR;	
	u16 val;
	
	val	= omap_tsc2101_audio_read(TSC2101_DAC_GAIN_CTRL);
	M_DPRINTK("registry value = %d!\n", val);
	volL	= DGC_DALVL_EXTRACT(val);
	volR	= DGC_DARVL_EXTRACT(val);
	/* make sure that other bits are not on */
	volL	= volL & ~DGC_DALMU;
	volR	= volR & ~DGC_DARMU;

	volL	= get_dac_gain_control_volume_as_mixer_volume(volL);
	volR	= get_dac_gain_control_volume_as_mixer_volume(volR);
	
	ucontrol->value.integer.value[0]	= volL; /* L */
	ucontrol->value.integer.value[1]	= volR; /* R */
	
	M_DPRINTK("mixer volume left = %ld, right = %ld\n", ucontrol->value.integer.value[0], ucontrol->value.integer.value[1]);
	return 0;
}

static int __pcm_playback_volume_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	return set_mixer_volume_as_dac_gain_control_volume(ucontrol->value.integer.value[0], 
							ucontrol->value.integer.value[1]);
}

static int __pcm_playback_switch_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo) 
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 2;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;
	return 0;
}

/* 
 * When DGC_DALMU (bit 15) is 1, the left channel is muted.
 * When DGC_DALMU is 0, left channel is not muted.
 * Same logic apply also for the right channel.
 */
static int __pcm_playback_switch_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	u16 val	= omap_tsc2101_audio_read(TSC2101_DAC_GAIN_CTRL);
	
	ucontrol->value.integer.value[0]	= IS_UNMUTED(15, val);	// left
	ucontrol->value.integer.value[1]	= IS_UNMUTED(7, val);	// right
	return 0;
}

static int __pcm_playback_switch_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	return dac_gain_control_unmute(ucontrol->value.integer.value[0], 
					ucontrol->value.integer.value[1]);
}

static int __headset_playback_volume_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo) 
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 100;
	return 0;
}

static int __headset_playback_volume_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	u16 val;
	u16 vol;
	
	val	= omap_tsc2101_audio_read(TSC2101_HEADSET_GAIN_CTRL);
	M_DPRINTK("registry value = %d\n", val);
	vol	= HGC_ADPGA_HED_EXTRACT(val);
	vol	= vol & ~HGC_ADMUT_HED;

	vol	= get_headset_gain_control_volume_as_mixer_volume(vol);
	ucontrol->value.integer.value[0]	= vol;
	
	M_DPRINTK("mixer volume returned = %ld\n", ucontrol->value.integer.value[0]);
	return 0;
}

static int __headset_playback_volume_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	return set_mixer_volume_as_headset_gain_control_volume(ucontrol->value.integer.value[0]);	
}

static int __headset_playback_switch_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo) 
{
	uinfo->type 			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;
	return 0;
}

/* When HGC_ADMUT_HED (bit 15) is 1, the headset is muted.
 * When HGC_ADMUT_HED is 0, headset is not muted.
 */
static int __headset_playback_switch_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	u16 val = omap_tsc2101_audio_read(TSC2101_HEADSET_GAIN_CTRL);
	ucontrol->value.integer.value[0]	= IS_UNMUTED(15, val);
	return 0;
}

static int __headset_playback_switch_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	// mute/unmute headset
	return adc_pga_unmute_control(ucontrol->value.integer.value[0],
				TSC2101_HEADSET_GAIN_CTRL,
				15);
}

static int __handset_playback_volume_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo) 
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 100;
	return 0;
}

static int __handset_playback_volume_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	u16 val;
	u16 vol;
	
	val	= omap_tsc2101_audio_read(TSC2101_HANDSET_GAIN_CTRL);
	M_DPRINTK("registry value = %d\n", val);
	vol	= HNGC_ADPGA_HND_EXTRACT(val);
	vol	= vol & ~HNGC_ADMUT_HND;
	vol	= get_handset_gain_control_volume_as_mixer_volume(vol);
	ucontrol->value.integer.value[0]	= vol;
	
	M_DPRINTK("mixer volume returned = %ld\n", ucontrol->value.integer.value[0]);
	return 0;
}

static int __handset_playback_volume_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	return set_mixer_volume_as_handset_gain_control_volume(ucontrol->value.integer.value[0]);	
}

static int __handset_playback_switch_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo) 
{
	uinfo->type 			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;
	return 0;
}

/* When HNGC_ADMUT_HND (bit 15) is 1, the handset is muted.
 * When HNGC_ADMUT_HND is 0, handset is not muted.
 */
static int __handset_playback_switch_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	u16 val = omap_tsc2101_audio_read(TSC2101_HANDSET_GAIN_CTRL);
	ucontrol->value.integer.value[0]	= IS_UNMUTED(15, val);
	return 0;
}

static int __handset_playback_switch_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	// handset mute/unmute
	return adc_pga_unmute_control(ucontrol->value.integer.value[0],
				TSC2101_HANDSET_GAIN_CTRL,
				15);
}

static int __cellphone_input_switch_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo) 
{
	uinfo->type 			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;
	return 0;
}

/* When BGC_MUT_CP (bit 15) = 1, power down cellphone input pga.
 * When BGC_MUT_CP = 0, power up cellphone input pga.
 */
static int __cellphone_input_switch_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	u16 val = omap_tsc2101_audio_read(TSC2101_BUZZER_GAIN_CTRL);
	ucontrol->value.integer.value[0]	= IS_UNMUTED(15, val);
	return 0;
}

static int __cellphone_input_switch_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	return adc_pga_unmute_control(ucontrol->value.integer.value[0],
				TSC2101_BUZZER_GAIN_CTRL,
				15);	
}

static int __buzzer_input_switch_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo) 
{
	uinfo->type 			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;
	return 0;
}

/* When BGC_MUT_BU (bit 6) = 1, power down cellphone input pga.
 * When BGC_MUT_BU = 0, power up cellphone input pga.
 */
static int __buzzer_input_switch_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	u16 val = omap_tsc2101_audio_read(TSC2101_BUZZER_GAIN_CTRL);
	ucontrol->value.integer.value[0]	= IS_UNMUTED(6, val);
	return 0;
}

static int __buzzer_input_switch_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol) 
{
	return adc_pga_unmute_control(ucontrol->value.integer.value[0],
				TSC2101_BUZZER_GAIN_CTRL,
				6);	
}

static snd_kcontrol_new_t tsc2101_control[] __devinitdata = {
	{
		.name  = "Target Playback Route",
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index = 0,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info  = __pcm_playback_target_info,
		.get   = __pcm_playback_target_get,
		.put   = __pcm_playback_target_put,
	}, {
		.name  = "Master Playback Volume",
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index = 0,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info  = __pcm_playback_volume_info,
		.get   = __pcm_playback_volume_get,
		.put   = __pcm_playback_volume_put,
	}, {
		.name  = "Master Playback Switch",
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index = 0,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info  = __pcm_playback_switch_info,
		.get   = __pcm_playback_switch_get,
		.put   = __pcm_playback_switch_put,
	}, {
		.name  = "Headset Playback Volume",
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index = 0,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info  = __headset_playback_volume_info,
		.get   = __headset_playback_volume_get,
		.put   = __headset_playback_volume_put,
	}, {
		.name  = "Headset Playback Switch",
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index = 0,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info  = __headset_playback_switch_info,
		.get   = __headset_playback_switch_get,
		.put   = __headset_playback_switch_put,
	}, {
		.name  = "Handset Playback Volume",
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index = 0,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info  = __handset_playback_volume_info,
		.get   = __handset_playback_volume_get,
		.put   = __handset_playback_volume_put,
	}, {
		.name  = "Handset Playback Switch",
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index = 0,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info  = __handset_playback_switch_info,
		.get   = __handset_playback_switch_get,
		.put   = __handset_playback_switch_put,
	}, {
		.name  = "Cellphone Input Switch",
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index = 0,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info  = __cellphone_input_switch_info,
		.get   = __cellphone_input_switch_get,
		.put   = __cellphone_input_switch_put,
	}, {
		.name  = "Buzzer Input Switch",
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index = 0,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info  = __buzzer_input_switch_info,
		.get   = __buzzer_input_switch_get,
		.put   = __buzzer_input_switch_put,
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

int snd_omap_mixer(struct snd_card_omap_codec *tsc2101) 
{
	int i=0;
	int err=0;

	if (!tsc2101) {
		return -EINVAL;
	}
	for (i=0; i < ARRAY_SIZE(tsc2101_control); i++) {
		if ((err = snd_ctl_add(tsc2101->card, 
				snd_ctl_new1(&tsc2101_control[i], 
				tsc2101->card))) < 0) {
			return err;
		}
	}
	return 0;
}
