#ifndef _LINUX_SPI_TSC2301_H
#define _LINUX_SPI_TSC2301_H

#include <linux/types.h>
#include <linux/timer.h>

struct tsc2301_platform_data {
	/*
	 * Keypad
	 */
	s16	reset_gpio;
	s16	keyb_int;
	s16	keymap[16];	/* Set a key to a negative value if not used */
	unsigned kp_rep:1;	/* Enable keypad repeating */
	char    *keyb_name;     /* Keyboard device name */

	/*
	 * Touchscreen
	 */
	s16	dav_int;
	u16	ts_x_plate_ohm;
	u32	ts_stab_time;	/* voltage settling time */
	u8	ts_hw_avg;	/* HW assiseted averaging. Can be
				   0, 4, 8, 16 samples per reading */
	u32	ts_max_pressure;/* Samples with bigger pressure value will
				   be ignored, since the corresponding X, Y
				   values are unreliable */
	u32	ts_touch_pressure;	/* Pressure limit until we report a
					   touch event. After that we switch
					   to ts_max_pressure. */
	u32	ts_pressure_fudge;
	u32	ts_x_max;
	u32	ts_x_fudge;
	u32	ts_y_max;
	u32	ts_y_fudge;

	/*
	 * Audio
	 */
	unsigned	pll_pdc:4;
	unsigned	pll_a:4;
	unsigned	pll_n:4;
	unsigned	pll_output:1; /* Output PLL on GPIO_0 */

	unsigned	mclk_ratio:2;
	unsigned	i2s_sample_rate:4;
	unsigned	i2s_format:2;
	/* Mask for audio blocks to be powered down */
	u16		power_down_blocks;

	/* Called after codec has been initialized, can be NULL */
	int (* codec_init)(struct device *tsc2301_dev);
	/* Called when codec is being removed, can be NULL */
	void (* codec_cleanup)(struct device *tsc2301_dev);
	int	(*enable_clock)(struct device *dev);
	void	(*disable_clock)(struct device *dev);

	const struct tsc2301_mixer_gpio {
		const char	*name;
		unsigned	gpio:4;
		unsigned	inverted:1;
		unsigned	def_enable:1; /* enable by default */
		unsigned	deactivate_on_pd:1; /* power-down flag */
	} *mixer_gpios;
	int	n_mixer_gpios;
};

struct tsc2301_kp;
struct tsc2301_ts;
struct tsc2301_mixer;

struct tsc2301 {
	struct spi_device	*spi;

	s16			reset_gpio;
	u16			config2_shadow;

        struct tsc2301_kp	*kp;
	struct tsc2301_ts	*ts;
	struct tsc2301_mixer	*mixer;

	int			(*enable_clock)(struct device *dev);
	void			(*disable_clock)(struct device *dev);
};


#define TSC2301_HZ	33000000

#define TSC2301_REG(page, addr)  (((page) << 11) | ((addr) << 5))
#define TSC2301_REG_TO_PAGE(reg) (((reg) >> 11) & 0x03)
#define TSC2301_REG_TO_ADDR(reg) (((reg) >> 5)  & 0x1f)

#define TSC2301_REG_X		TSC2301_REG(0, 0)
#define TSC2301_REG_Y		TSC2301_REG(0, 1)
#define TSC2301_REG_Z1		TSC2301_REG(0, 2)
#define TSC2301_REG_Z2		TSC2301_REG(0, 3)
#define TSC2301_REG_KPDATA	TSC2301_REG(0, 4)
#define TSC2301_REG_ADC		TSC2301_REG(1, 0)
#define TSC2301_REG_KEY		TSC2301_REG(1, 1)
#define TSC2301_REG_DAC		TSC2301_REG(1, 2)
#define TSC2301_REG_REF		TSC2301_REG(1, 3)
#define TSC2301_REG_RESET	TSC2301_REG(1, 4)
#define TSC2301_REG_CONFIG	TSC2301_REG(1, 5)
#define TSC2301_REG_CONFIG2	TSC2301_REG(1, 6)
#define TSC2301_REG_KPMASK	TSC2301_REG(1, 16)
#define TSC2301_REG_AUDCNTL	TSC2301_REG(2, 0)
#define TSC2301_REG_ADCVOL	TSC2301_REG(2, 1)
#define TSC2301_REG_DACVOL	TSC2301_REG(2, 2)
#define TSC2301_REG_BPVOL	TSC2301_REG(2, 3)
#define TSC2301_REG_KEYCTL	TSC2301_REG(2, 4)
#define TSC2301_REG_PD_MISC	TSC2301_REG(2, 5)
#define TSC2301_REG_GPIO	TSC2301_REG(2, 6)
#define TSC2301_REG_ADCLKCFG	TSC2301_REG(2, 27)

#define TSC2301_REG_PD_MISC_APD		(1 << 15)
#define TSC2301_REG_PD_MISC_AVPD	(1 << 14)
#define TSC2301_REG_PD_MISC_ABPD	(1 << 13)
#define TSC2301_REG_PD_MISC_HAPD	(1 << 12)
#define TSC2301_REG_PD_MISC_MOPD	(1 << 11)
#define TSC2301_REG_PD_MISC_DAPD	(1 << 10)
#define TSC2301_REG_PD_MISC_ADPDL	(1 << 9)
#define TSC2301_REG_PD_MISC_ADPDR	(1 << 8)
#define TSC2301_REG_PD_MISC_PDSTS	(1 << 7)
#define TSC2301_REG_PD_MISC_MIBPD	(1 << 6)
#define TSC2301_REG_PD_MISC_OTSYN	(1 << 2)

/* I2S sample rate */
#define TSC2301_I2S_SR_48000	0x00
#define TSC2301_I2S_SR_44100	0x01
#define TSC2301_I2S_SR_32000	0x02
#define TSC2301_I2S_SR_24000	0x03
#define TSC2301_I2S_SR_22050	0x04
#define TSC2301_I2S_SR_16000	0x05
#define TSC2301_I2S_SR_12000	0x06
#define TSC2301_I2S_SR_11050	0x07
#define TSC2301_I2S_SR_8000	0x08

/* 16-bit, MSB-first. DAC Right-Justified, ADC Left-Justified */
#define TSC2301_I2S_FORMAT0	0x00
/* 20-bit, MSB-first. DAC Right-Justified, ADC Left-Justified */
#define TSC2301_I2S_FORMAT1	0x01
/* 20-bit, MSB-first. DAC Left-Justified, ADC Left-Justified */
#define TSC2301_I2S_FORMAT2	0x02
/* 20-bit, MSB-first */
#define TSC2301_I2S_FORMAT3	0x03

/* Master Clock Ratio */
#define TSC2301_MCLK_256xFS	0x00 /* default */
#define TSC2301_MCLK_384xFS	0x01
#define TSC2301_MCLK_512xFS	0x02


extern u16 tsc2301_read_reg(struct tsc2301 *tsc, int reg);
extern void tsc2301_write_reg(struct tsc2301 *tsc, int reg, u16 val);
extern void tsc2301_write_kbc(struct tsc2301 *tsc, int val);
extern void tsc2301_write_pll(struct tsc2301 *tsc, int pll_n, int pll_a,
			      int pll_pdc, int pct_e, int pll_o);
extern void tsc2301_read_buf(struct tsc2301 *tsc, int reg, u16 *buf, int len);

#define TSC2301_DECL_MOD(module)					\
extern int  tsc2301_##module##_init(struct tsc2301 *tsc,		\
			   struct tsc2301_platform_data *pdata);	\
extern void tsc2301_##module##_exit(struct tsc2301 *tsc);		\
extern int  tsc2301_##module##_suspend(struct tsc2301 *tsc);		\
extern void tsc2301_##module##_resume(struct tsc2301 *tsc);

#define TSC2301_DECL_EMPTY_MOD(module)					\
static inline int tsc2301_##module##_init(struct tsc2301 *tsc,		\
			   struct tsc2301_platform_data *pdata)		\
{									\
	return 0;							\
}									\
static inline void tsc2301_##module##_exit(struct tsc2301 *tsc) {}	\
static inline int  tsc2301_##module##_suspend(struct tsc2301 *tsc)	\
{									\
	return 0;							\
}									\
static inline void tsc2301_##module##_resume(struct tsc2301 *tsc) {}

#ifdef CONFIG_KEYBOARD_TSC2301
TSC2301_DECL_MOD(kp)
void tsc2301_kp_restart(struct tsc2301 *tsc);
#else
TSC2301_DECL_EMPTY_MOD(kp)
static inline void tsc2301_kp_restart(struct tsc2301 *tsc) {}
#endif

#ifdef CONFIG_TOUCHSCREEN_TSC2301
TSC2301_DECL_MOD(ts)
#else
TSC2301_DECL_EMPTY_MOD(ts)
#endif

extern void tsc2301_mixer_enable_mclk(struct device *tsc_dev);
extern void tsc2301_mixer_disable_mclk(struct device *tsc_dev);

#endif
