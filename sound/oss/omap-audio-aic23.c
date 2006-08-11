/*
 * linux/sound/oss/omap-audio-aic23.c
 *
 * Glue audio driver for TI TLV320AIC23 codec
 *
 * Copyright (c) 2000 Nicolas Pitre <nico@cam.org>
 * Copyright (C) 2001, Steve Johnson <stevej@ridgerun.com>
 * Copyright (C) 2004 Texas Instruments, Inc.
 * Copyright (C) 2005 Dirk Behme <dirk.behme@de.bosch.com>
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/clk.h>
#include <linux/mutex.h>

#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <asm/arch/mcbsp.h>
#include <asm/arch/fpga.h>
#include <asm/arch/aic23.h>
#include <asm/arch/clock.h>

#include "omap-audio.h"
#include "omap-audio-dma-intfc.h"

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#define PROC_START_FILE "driver/aic23-audio-start"
#define PROC_STOP_FILE  "driver/aic23-audio-stop"
#endif

//#define DEBUG

#ifdef DEBUG
#define DPRINTK(ARGS...)  printk("<%s>: ",__FUNCTION__);printk(ARGS)
#else
#define DPRINTK( x... )
#endif

#define CODEC_NAME		 "AIC23"

#if CONFIG_MACH_OMAP_OSK
#define PLATFORM_NAME            "OMAP OSK"
#elif CONFIG_MACH_OMAP_INNOVATOR
#define PLATFORM_NAME            "OMAP INNOVATOR"
#else
#error "Unsupported plattform"
#endif

/* Define to set the AIC23 as the master w.r.t McBSP */
#define AIC23_MASTER

#define CODEC_CLOCK                   12000000

/*
 * AUDIO related MACROS
 */
#define DEFAULT_BITPERSAMPLE          16
#define AUDIO_RATE_DEFAULT	      44100

/* Select the McBSP For Audio */
#define AUDIO_MCBSP                   OMAP_MCBSP1

#define REC_MASK 		      (SOUND_MASK_LINE | SOUND_MASK_MIC)
#define DEV_MASK 		      (REC_MASK | SOUND_MASK_VOLUME)

#define SET_VOLUME 		      1
#define SET_LINE   		      2

#define DEFAULT_OUTPUT_VOLUME         93
#define DEFAULT_INPUT_VOLUME          0	/* 0 ==> mute line in */

#define OUTPUT_VOLUME_MIN             LHV_MIN
#define OUTPUT_VOLUME_MAX             LHV_MAX
#define OUTPUT_VOLUME_RANGE           (OUTPUT_VOLUME_MAX - OUTPUT_VOLUME_MIN)
#define OUTPUT_VOLUME_MASK            OUTPUT_VOLUME_MAX

#define INPUT_VOLUME_MIN 	      LIV_MIN
#define INPUT_VOLUME_MAX 	      LIV_MAX
#define INPUT_VOLUME_RANGE 	      (INPUT_VOLUME_MAX - INPUT_VOLUME_MIN)
#define INPUT_VOLUME_MASK 	      INPUT_VOLUME_MAX

#define NUMBER_SAMPLE_RATES_SUPPORTED 9

/*
 * HW interface start and stop helper functions
 */
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
        .id              = "AIC23 out",
        .dma_dev         = OMAP_DMA_MCBSP1_TX,
	.input_or_output = FMODE_WRITE,
	.hw_start	= audio_ifc_start,
	.hw_stop	 = audio_ifc_stop
};

static audio_stream_t input_stream = {
        .id              = "AIC23 in",
        .dma_dev         = OMAP_DMA_MCBSP1_RX,
	.input_or_output = FMODE_READ,
	.hw_start	= audio_ifc_start,
	.hw_stop	 = audio_ifc_stop
};

static struct clk *aic23_mclk = 0;

static int audio_dev_id, mixer_dev_id;

static struct aic23_local_info {
        u8  volume;
        u16 volume_reg;
        u8  line;
        u8  mic;
        u16 input_volume_reg;
        int mod_cnt;
} aic23_local;

struct sample_rate_reg_info {
        u32 sample_rate;
        u8  control;            /* SR3, SR2, SR1, SR0 and BOSR */
        u8  divider;		/* if 0 CLKIN = MCLK, if 1 CLKIN = MCLK/2 */
};

/* To Store the default sample rate */
static long audio_samplerate = AUDIO_RATE_DEFAULT;

/* DAC USB-mode sampling rates (MCLK = 12 MHz) */
static const struct sample_rate_reg_info
reg_info[NUMBER_SAMPLE_RATES_SUPPORTED] = {
        {96000, 0x0E, 0},
        {88200, 0x1F, 0},
        {48000, 0x00, 0},
        {44100, 0x11, 0},
        {32000, 0x0C, 0},
        {24000, 0x00, 1},
        {16000, 0x0C, 1},
        { 8000, 0x06, 0},
        { 4000, 0x06, 1},
};

static struct omap_mcbsp_reg_cfg initial_config = {
        .spcr2 = FREE | FRST | GRST | XRST | XINTM(3),
        .spcr1 = RINTM(3) | RRST,
        .rcr2  = RPHASE | RFRLEN2(OMAP_MCBSP_WORD_8) |
	RWDLEN2(OMAP_MCBSP_WORD_16) | RDATDLY(0),
        .rcr1  = RFRLEN1(OMAP_MCBSP_WORD_8) | RWDLEN1(OMAP_MCBSP_WORD_16),
        .xcr2  = XPHASE | XFRLEN2(OMAP_MCBSP_WORD_8) |
        XWDLEN2(OMAP_MCBSP_WORD_16) | XDATDLY(0) | XFIG,
        .xcr1  = XFRLEN1(OMAP_MCBSP_WORD_8) | XWDLEN1(OMAP_MCBSP_WORD_16),
        .srgr1 = FWID(DEFAULT_BITPERSAMPLE - 1),
        .srgr2 = GSYNC | CLKSP | FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1),
#ifndef AIC23_MASTER
        /* configure McBSP to be the I2S master */
        .pcr0  = FSXM | FSRM | CLKXM | CLKRM | CLKXP | CLKRP,
#else
        /* configure McBSP to be the I2S slave */
        .pcr0  = CLKXP | CLKRP,
#endif /* AIC23_MASTER */
};

static void omap_aic23_initialize(void *dummy);
static void omap_aic23_shutdown(void *dummy);
static int  omap_aic23_ioctl(struct inode *inode, struct file *file,
                             uint cmd, ulong arg);
static int  omap_aic23_probe(void);
#ifdef MODULE
static void omap_aic23_remove(void);
#endif
static int  omap_aic23_suspend(void);
static int  omap_aic23_resume(void);
static inline void aic23_configure(void);
static int  mixer_open(struct inode *inode, struct file *file);
static int  mixer_release(struct inode *inode, struct file *file);
static int  mixer_ioctl(struct inode *inode, struct file *file, uint cmd,
                        ulong arg);

#ifdef CONFIG_PROC_FS
static int codec_start(char *buf, char **start, off_t offset, int count,
                       int *eof, void *data);
static int codec_stop(char *buf, char **start, off_t offset, int count,
                      int *eof, void *data);
#endif


/* File Op structure for mixer */
static struct file_operations omap_mixer_fops = {
        .open           = mixer_open,
        .release        = mixer_release,
        .ioctl          = mixer_ioctl,
        .owner          = THIS_MODULE
};

/* To store characteristic info regarding the codec for the audio driver */
static audio_state_t aic23_state = {
        .output_stream  = &output_stream,
        .input_stream   = &input_stream,
/*	.need_tx_for_rx = 1, //Once the Full Duplex works  */
        .need_tx_for_rx = 0,
        .hw_init        = omap_aic23_initialize,
        .hw_shutdown    = omap_aic23_shutdown,
        .client_ioctl   = omap_aic23_ioctl,
        .hw_probe       = omap_aic23_probe,
        .hw_remove      =  __exit_p(omap_aic23_remove),
        .hw_suspend     = omap_aic23_suspend,
        .hw_resume      = omap_aic23_resume,
};

/* This will be defined in the audio.h */
static struct file_operations *omap_audio_fops;

extern int aic23_write_value(u8 reg, u16 value);

/* TLV320AIC23 is a write only device */
static __inline__ void audio_aic23_write(u8 address, u16 data)
{
        aic23_write_value(address, data);
}

static int aic23_update(int flag, int val)
{
        u16 volume;

        /* Ignore separate left/right channel for now,
           even the codec does support it. */
        val &= 0xff;

        if (val < 0 || val > 100) {
                printk(KERN_ERR "Trying a bad volume value(%d)!\n",val);
                return -EPERM;
        }

        switch (flag) {
        case SET_VOLUME:
                // Convert 0 -> 100 volume to 0x00 (LHV_MIN) -> 0x7f (LHV_MAX) 
                // volume range
                volume = ((val * OUTPUT_VOLUME_RANGE) / 100) + OUTPUT_VOLUME_MIN;
                
                // R/LHV[6:0] 1111111 (+6dB) to 0000000 (-73dB) in 1db steps,
                // default 1111001 (0dB)
                aic23_local.volume_reg &= ~OUTPUT_VOLUME_MASK;
                aic23_local.volume_reg |= volume;
                audio_aic23_write(LEFT_CHANNEL_VOLUME_ADDR, aic23_local.volume_reg);
                audio_aic23_write(RIGHT_CHANNEL_VOLUME_ADDR, aic23_local.volume_reg);
                break;

        case SET_LINE:
                // Convert 0 -> 100 volume to 0x0 (LIV_MIN) -> 0x1f (LIV_MAX) 
                // volume range
                volume = ((val * INPUT_VOLUME_RANGE) / 100) + INPUT_VOLUME_MIN;

                // R/LIV[4:0] 11111 (+12dB) to 00000 (-34.5dB) in 1.5dB steps,
                // default 10111 (0dB)
                aic23_local.input_volume_reg &= ~INPUT_VOLUME_MASK;
                aic23_local.input_volume_reg |= volume;
                audio_aic23_write(LEFT_LINE_VOLUME_ADDR, aic23_local.input_volume_reg);
                audio_aic23_write(RIGHT_LINE_VOLUME_ADDR, aic23_local.input_volume_reg);
                break;
        }
        return 0;
}

static int mixer_open(struct inode *inode, struct file *file)
{
        /* Any mixer specific initialization */

        return 0;
}

static int mixer_release(struct inode *inode, struct file *file)
{
        /* Any mixer specific Un-initialization */

        return 0;
}

static int
mixer_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
        int val;
        int ret = 0;
        int nr = _IOC_NR(cmd);

        /*
         * We only accept mixer (type 'M') ioctls.
         */
        if (_IOC_TYPE(cmd) != 'M')
                return -EINVAL;

        DPRINTK(" 0x%08x\n", cmd);

        if (cmd == SOUND_MIXER_INFO) {
                struct mixer_info mi;

                strncpy(mi.id, "AIC23", sizeof(mi.id));
                strncpy(mi.name, "TI AIC23", sizeof(mi.name));
                mi.modify_counter = aic23_local.mod_cnt;
                return copy_to_user((void *)arg, &mi, sizeof(mi));
        }

        if (_IOC_DIR(cmd) & _IOC_WRITE) {
                ret = get_user(val, (int *)arg);
                if (ret)
                        goto out;

        
                switch (nr) {
                case SOUND_MIXER_VOLUME:
                        aic23_local.volume = val;
                        aic23_local.mod_cnt++;
                        ret = aic23_update(SET_VOLUME, val);
                        break;

                case SOUND_MIXER_LINE:
                        aic23_local.line = val;
                        aic23_local.mod_cnt++;
                        ret = aic23_update(SET_LINE, val);
                        break;

                case SOUND_MIXER_MIC:
                        aic23_local.mic = val;
                        aic23_local.mod_cnt++;
                        ret = aic23_update(SET_LINE, val);
                        break;

                case SOUND_MIXER_RECSRC:
                        break;

                default:
                        ret = -EINVAL;
                }
        }

        if (ret == 0 && _IOC_DIR(cmd) & _IOC_READ) {
                ret = 0;

                switch (nr) {
                case SOUND_MIXER_VOLUME:
                        val = aic23_local.volume;
                        break;
                case SOUND_MIXER_LINE:
                        val = aic23_local.line;
                        break;
                case SOUND_MIXER_MIC:
                        val = aic23_local.mic;
                        break;
                case SOUND_MIXER_RECSRC:
                        val = REC_MASK;
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
                        val = 0;
                        break;
                default:
                        val = 0;
                        ret = -EINVAL;
                        break;
                }

                if (ret == 0)
                        ret = put_user(val, (int *)arg);
        }
out:
        return ret;

}

int omap_set_samplerate(long sample_rate)
{
        u8 count = 0;
        u16 data = 0;
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

        if (machine_is_omap_innovator()) {
                /* set the CODEC clock input source to 12.000MHz */
                fpga_write(fpga_read(OMAP1510_FPGA_POWER) & ~0x01, 
                           OMAP1510_FPGA_POWER);
        }

        data = (reg_info[count].divider << CLKIN_SHIFT) | 
                (reg_info[count].control << BOSR_SHIFT) | USB_CLK_ON;

        audio_aic23_write(SAMPLE_RATE_CONTROL_ADDR, data);

        audio_samplerate = sample_rate;

#ifndef AIC23_MASTER
        {
                int clkgdv = 0;
                /* 
                   Set Sample Rate at McBSP

                   Formula : 
                   Codec System Clock = CODEC_CLOCK, or half if clock_divider = 1;
                   clkgdv = ((Codec System Clock / (SampleRate * BitsPerSample * 2)) - 1);

                   FWID = BitsPerSample - 1;
                   FPER = (BitsPerSample * 2) - 1;
                */  
                if (reg_info[count].divider)
                        clkgdv = CODEC_CLOCK / 2;
                else 
                        clkgdv = CODEC_CLOCK;

                clkgdv = (clkgdv / (sample_rate * DEFAULT_BITPERSAMPLE * 2)) - 1;

                initial_config.srgr1 = (FWID(DEFAULT_BITPERSAMPLE - 1) | CLKGDV(clkgdv));

                initial_config.srgr2 =
                        (CLKSM | FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1));

                omap_mcbsp_config(AUDIO_MCBSP, &initial_config);
        }
#endif /* AIC23_MASTER */

        return 0;
}

static void omap_aic23_initialize(void *dummy)
{
        DPRINTK("entry\n");

        /* initialize with default sample rate */
        audio_samplerate = AUDIO_RATE_DEFAULT;

        omap_mcbsp_request(AUDIO_MCBSP);

        /* if configured, then stop mcbsp */
        omap_mcbsp_stop(AUDIO_MCBSP);

        omap_mcbsp_config(AUDIO_MCBSP, &initial_config);
        omap_mcbsp_start(AUDIO_MCBSP);
        aic23_configure();

        DPRINTK("exit\n");
}

static void omap_aic23_shutdown(void *dummy)
{
        /*
          Turn off codec after it is done.
          Can't do it immediately, since it may still have
          buffered data.

          Wait 20ms (arbitrary value) and then turn it off.
        */

        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(2);

        omap_mcbsp_stop(AUDIO_MCBSP);
        omap_mcbsp_free(AUDIO_MCBSP);

        audio_aic23_write(RESET_CONTROL_ADDR, 0);
        audio_aic23_write(POWER_DOWN_CONTROL_ADDR, 0xff);
}

static inline void aic23_configure()
{
        /* Reset codec */
        audio_aic23_write(RESET_CONTROL_ADDR, 0);

        /* Initialize the AIC23 internal state */

        /* Left/Right line input volume control */
        aic23_local.line = DEFAULT_INPUT_VOLUME;
        aic23_local.mic = DEFAULT_INPUT_VOLUME;
        aic23_update(SET_LINE, DEFAULT_INPUT_VOLUME);

        /* Left/Right headphone channel volume control */
        /* Zero-cross detect on */
        aic23_local.volume_reg = LZC_ON;
        aic23_update(SET_VOLUME, aic23_local.volume);

        /* Analog audio path control, DAC selected, delete INSEL_MIC for line in */
        audio_aic23_write(ANALOG_AUDIO_CONTROL_ADDR, DAC_SELECTED | INSEL_MIC);

        /* Digital audio path control, de-emphasis control 44.1kHz */
        audio_aic23_write(DIGITAL_AUDIO_CONTROL_ADDR, DEEMP_44K);

        /* Power control, everything is on */
        audio_aic23_write(POWER_DOWN_CONTROL_ADDR, 0);

        /* Digital audio interface, master/slave mode, I2S, 16 bit */
#ifdef AIC23_MASTER
        audio_aic23_write(DIGITAL_AUDIO_FORMAT_ADDR, MS_MASTER | IWL_16 | FOR_DSP);
#else
        audio_aic23_write(DIGITAL_AUDIO_FORMAT_ADDR, IWL_16 | FOR_DSP);
#endif /* AIC23_MASTER */

        /* Enable digital interface */
        audio_aic23_write(DIGITAL_INTERFACE_ACT_ADDR, ACT_ON);

        /* clock configuration */
        omap_set_samplerate(audio_samplerate);
}

static int
omap_aic23_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
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
                ret = get_user(val, (int *)arg);
                if (ret)
                        return ret;
                /* the AIC23 is stereo only */
                ret = (val == 0) ? -EINVAL : 1;
                return put_user(ret, (int *)arg);

        case SNDCTL_DSP_CHANNELS:
        case SOUND_PCM_READ_CHANNELS:
                /* the AIC23 is stereo only */
                return put_user(2, (long *)arg);

        case SNDCTL_DSP_SPEED:
                ret = get_user(val, (long *)arg);
                if (ret)
                        break;
                ret = omap_set_samplerate(val);
                if (ret)
                        break;
                /* fall through */

        case SOUND_PCM_READ_RATE:
                return put_user(audio_samplerate, (long *)arg);

        case SOUND_PCM_READ_BITS:
        case SNDCTL_DSP_SETFMT:
        case SNDCTL_DSP_GETFMTS:
                /* we can do 16-bit only */
                return put_user(AFMT_S16_LE, (long *)arg);

        default:
                /* Maybe this is meant for the mixer (As per OSS Docs) */
                return mixer_ioctl(inode, file, cmd, arg);
        }

        return ret;
}

static int omap_aic23_probe(void)
{
        /* Get the fops from audio oss driver */
        if (!(omap_audio_fops = audio_get_fops())) {
                printk(KERN_ERR "Unable to get the file operations for AIC23 OSS driver\n");
                audio_unregister_codec(&aic23_state);
                return -EPERM;
        }

        aic23_local.volume = DEFAULT_OUTPUT_VOLUME;

        /* register devices */
        audio_dev_id = register_sound_dsp(omap_audio_fops, -1);
        mixer_dev_id = register_sound_mixer(&omap_mixer_fops, -1);

#ifdef CONFIG_PROC_FS
        create_proc_read_entry(PROC_START_FILE, 0 /* default mode */ ,
                               NULL /* parent dir */ ,
                               codec_start, NULL /* client data */ );

        create_proc_read_entry(PROC_STOP_FILE, 0 /* default mode */ ,
                               NULL /* parent dir */ ,
                               codec_stop, NULL /* client data */ );
#endif

        /* Announcement Time */
        printk(KERN_INFO PLATFORM_NAME " " CODEC_NAME
               " audio support initialized\n");
        return 0;
}

#ifdef MODULE
static void __exit omap_aic23_remove(void)
{
        /* Un-Register the codec with the audio driver */
        unregister_sound_dsp(audio_dev_id);
        unregister_sound_mixer(mixer_dev_id);

#ifdef CONFIG_PROC_FS
        remove_proc_entry(PROC_START_FILE, NULL);
        remove_proc_entry(PROC_STOP_FILE, NULL);
#endif
}
#endif /* MODULE */

static int omap_aic23_suspend(void)
{
        /* Empty for the moment */
        return 0;
}

static int omap_aic23_resume(void)
{
        /* Empty for the moment */
        return 0;
}

static int __init audio_aic23_init(void)
{

        int err = 0;

	if (machine_is_omap_h2() || machine_is_omap_h3())
		return -ENODEV;

	mutex_init(&aic23_state.mutex);

        if (machine_is_omap_osk()) {
                /* Set MCLK to be clock input for AIC23 */
                aic23_mclk = clk_get(0, "mclk");
            
                if(clk_get_rate( aic23_mclk) != CODEC_CLOCK){
                        /* MCLK ist not at CODEC_CLOCK */
                        if( clk_get_usecount(aic23_mclk) > 0 ){
                                /* MCLK is already in use */
                                printk(KERN_WARNING "MCLK in use at %d Hz. We change it to %d Hz\n",
                                       (uint)clk_get_rate( aic23_mclk), CODEC_CLOCK);
                        }
                        if( clk_set_rate( aic23_mclk, CODEC_CLOCK ) ){
                                printk(KERN_ERR "Cannot set MCLK for AIC23 CODEC\n");
                                return -ECANCELED;
			}
		}

                clk_enable( aic23_mclk );

                DPRINTK("MCLK = %d [%d], usecount = %d\n",(uint)clk_get_rate( aic23_mclk ), 
                        CODEC_CLOCK, clk_get_usecount( aic23_mclk));
        }

        if (machine_is_omap_innovator()) {
                u8 fpga;
                /*
                  Turn on chip select for CODEC (shared with touchscreen).  
                  Don't turn it back off, in case touch screen needs it.
                */                           
                fpga = fpga_read(OMAP1510_FPGA_TOUCHSCREEN);
                fpga |= 0x4;
                fpga_write(fpga, OMAP1510_FPGA_TOUCHSCREEN);
        }

        /* register the codec with the audio driver */
        if ((err = audio_register_codec(&aic23_state))) {
                printk(KERN_ERR
                       "Failed to register AIC23 driver with Audio OSS Driver\n");
        }

        return err;
}

static void __exit audio_aic23_exit(void)
{
        (void)audio_unregister_codec(&aic23_state);
        return;
}

#ifdef CONFIG_PROC_FS
static int codec_start(char *buf, char **start, off_t offset, int count,
                       int *eof, void *data)
{
        void *foo = NULL;

        omap_aic23_initialize(foo);

        printk("AIC23 codec initialization done.\n");
        return 0;
}
static int codec_stop(char *buf, char **start, off_t offset, int count,
                      int *eof, void *data)
{
        void *foo = NULL;

        omap_aic23_shutdown(foo);

        printk("AIC23 codec shutdown.\n");
        return 0;
}
#endif /* CONFIG_PROC_FS */

module_init(audio_aic23_init);
module_exit(audio_aic23_exit);

MODULE_AUTHOR("Dirk Behme <dirk.behme@de.bosch.com>");
MODULE_DESCRIPTION("Glue audio driver for the TI AIC23 codec.");
MODULE_LICENSE("GPL");
