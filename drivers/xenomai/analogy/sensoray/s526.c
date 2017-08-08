/*
 * Analogy driver for Sensoray Model 526 board
 *
 * Copyright (C) 2009 Simon Boulay <simon.boulay@gmail.com>
 *
 * Derived from comedi:
 * Copyright (C) 2000 David A. Schleef <ds@schleef.org>
 *               2006 Everett Wang <everett.wang@everteq.com>
 *               2009 Ian Abbott <abbotti@mev.co.uk>
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * This code is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
/*
 * Original code comes from comedi linux-next staging driver (2009.12.20)
 * Board documentation: http://www.sensoray.com/products/526data.htm
 * Everything should work as in comedi:
 *   - Encoder works
 *   - Analog input works
 *   - Analog output works
 *   - PWM output works
 *   - Commands are not supported yet.
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <asm/byteorder.h>
#include <rtdm/analogy/device.h>

/* Board description */
#define S526_GPCT_CHANS	4
#define S526_GPCT_BITS	24
#define S526_AI_CHANS	10	/* 8 regular differential inputs
				 * channel 8 is "reference 0" (+10V)
				 * channel 9 is "reference 1" (0V) */
#define S526_AI_BITS	16
#define S526_AI_TIMEOUT 100
#define S526_AO_CHANS	4
#define S526_AO_BITS	16
#define S526_DIO_CHANS	8
#define S526_DIO_BITS	1

/* Ports */
#define S526_IOSIZE		0x40  /* 64 bytes */
#define S526_DEFAULT_ADDRESS	0x2C0 /* Manufacturing default */

/* Registers */
#define REG_TCR 0x00
#define REG_WDC 0x02
#define REG_DAC 0x04
#define REG_ADC 0x06
#define REG_ADD 0x08
#define REG_DIO 0x0A
#define REG_IER 0x0C
#define REG_ISR 0x0E
#define REG_MSC 0x10
#define REG_C0L 0x12
#define REG_C0H 0x14
#define REG_C0M 0x16
#define REG_C0C 0x18
#define REG_C1L 0x1A
#define REG_C1H 0x1C
#define REG_C1M 0x1E
#define REG_C1C 0x20
#define REG_C2L 0x22
#define REG_C2H 0x24
#define REG_C2M 0x26
#define REG_C2C 0x28
#define REG_C3L 0x2A
#define REG_C3H 0x2C
#define REG_C3M 0x2E
#define REG_C3C 0x30
#define REG_EED 0x32
#define REG_EEC 0x34

#define ISR_ADC_DONE 0x4

struct counter_mode_register_t {
#if defined (__LITTLE_ENDIAN_BITFIELD)
	unsigned short coutSource:1;
	unsigned short coutPolarity:1;
	unsigned short autoLoadResetRcap:3;
	unsigned short hwCtEnableSource:2;
	unsigned short ctEnableCtrl:2;
	unsigned short clockSource:2;
	unsigned short countDir:1;
	unsigned short countDirCtrl:1;
	unsigned short outputRegLatchCtrl:1;
	unsigned short preloadRegSel:1;
	unsigned short reserved:1;
#elif defined(__BIG_ENDIAN_BITFIELD)
	unsigned short reserved:1;
	unsigned short preloadRegSel:1;
	unsigned short outputRegLatchCtrl:1;
	unsigned short countDirCtrl:1;
	unsigned short countDir:1;
	unsigned short clockSource:2;
	unsigned short ctEnableCtrl:2;
	unsigned short hwCtEnableSource:2;
	unsigned short autoLoadResetRcap:3;
	unsigned short coutPolarity:1;
	unsigned short coutSource:1;
#else
#error Unknown bit field order
#endif
};

union cmReg {
	struct counter_mode_register_t reg;
	unsigned short value;
};

/* Application Classes for GPCT Subdevices */
enum S526_GPCT_APP_CLASS {
	CountingAndTimeMeasurement,
	SinglePulseGeneration,
	PulseTrainGeneration,
	PositionMeasurement,
	Miscellaneous
};

/* GPCT subdevices configuration */
#define MAX_GPCT_CONFIG_DATA 6
struct s526GPCTConfig {
	enum S526_GPCT_APP_CLASS app;
	int data[MAX_GPCT_CONFIG_DATA];
};

typedef struct s526_priv {
	unsigned long io_base;
} s526_priv_t;

struct s526_subd_gpct_priv {
	struct s526GPCTConfig config[4];
};

struct s526_subd_ai_priv {
	uint16_t config;
};

struct s526_subd_ao_priv {
	uint16_t readback[2];
};

struct s526_subd_dio_priv {
	int io_bits;
	unsigned int state;
};

#define devpriv ((s526_priv_t*)(dev->priv))

#define ADDR_REG(reg) (devpriv->io_base + (reg))
#define ADDR_CHAN_REG(reg, chan) (devpriv->io_base + (reg) + (chan) * 8)


static int s526_gpct_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	struct s526_subd_gpct_priv *subdpriv =
	    (struct s526_subd_gpct_priv *)subd->priv;
	unsigned int *data = (unsigned int *)insn->data;
	int subdev_channel = CR_CHAN(insn->chan_desc);
	int i;
	short value;
	union cmReg cmReg;

	a4l_dbg(1, drv_dbg, dev,
		"s526_gpct_insn_config: Configuring Channel %d\n",
		subdev_channel);

	for (i = 0; i < MAX_GPCT_CONFIG_DATA; i++) {
		subdpriv->config[subdev_channel].data[i] = data[i];
		a4l_dbg(1, drv_dbg, dev, "data[%d]=%x\n", i, data[i]);
	}

	switch (data[0]) {
	case A4L_INSN_CONFIG_GPCT_QUADRATURE_ENCODER:
		/*
		 * data[0]: Application Type
		 * data[1]: Counter Mode Register Value
		 * data[2]: Pre-load Register Value
		 * data[3]: Conter Control Register
		 */
		a4l_dbg(1, drv_dbg, dev, "s526_gpct_insn_config: Configuring Encoder\n");
		subdpriv->config[subdev_channel].app = PositionMeasurement;

		/* Set Counter Mode Register */
		cmReg.value = data[1] & 0xFFFF;

		a4l_dbg(1, drv_dbg, dev, "Counter Mode register=%x\n", cmReg.value);
		outw(cmReg.value, ADDR_CHAN_REG(REG_C0M, subdev_channel));

		/* Reset the counter if it is software preload */
		if (cmReg.reg.autoLoadResetRcap == 0) {
			outw(0x8000, ADDR_CHAN_REG(REG_C0C, subdev_channel)); /* Reset the counter */
			/* outw(0x4000, ADDR_CHAN_REG(REG_C0C, subdev_channel));	/\* Load the counter from PR0 *\/ */
		}
		break;

	case A4L_INSN_CONFIG_GPCT_SINGLE_PULSE_GENERATOR:
		/*
		 * data[0]: Application Type
		 * data[1]: Counter Mode Register Value
		 * data[2]: Pre-load Register 0 Value
		 * data[3]: Pre-load Register 1 Value
		 * data[4]: Conter Control Register
		 */
		a4l_dbg(1, drv_dbg, dev, "s526_gpct_insn_config: Configuring SPG\n");
		subdpriv->config[subdev_channel].app = SinglePulseGeneration;

		/* Set Counter Mode Register */
		cmReg.value = (short)(data[1] & 0xFFFF);
		cmReg.reg.preloadRegSel = 0; /* PR0 */
		outw(cmReg.value, ADDR_CHAN_REG(REG_C0M, subdev_channel));

		/* Load the pre-load register 0 high word */
		value = (short)((data[2] >> 16) & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0H, subdev_channel));

		/* Load the pre-load register 0 low word */
		value = (short)(data[2] & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0L, subdev_channel));

		/* Set Counter Mode Register */
		cmReg.value = (short)(data[1] & 0xFFFF);
		cmReg.reg.preloadRegSel = 1; /* PR1 */
		outw(cmReg.value, ADDR_CHAN_REG(REG_C0M, subdev_channel));

		/* Load the pre-load register 1 high word */
		value = (short)((data[3] >> 16) & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0H, subdev_channel));

		/* Load the pre-load register 1 low word */
		value = (short)(data[3] & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0L, subdev_channel));

		/* Write the Counter Control Register */
		if (data[4] != 0) {
			value = (short)(data[4] & 0xFFFF);
			outw(value, ADDR_CHAN_REG(REG_C0C, subdev_channel));
		}
		break;

	case A4L_INSN_CONFIG_GPCT_PULSE_TRAIN_GENERATOR:
		/*
		 * data[0]: Application Type
		 * data[1]: Counter Mode Register Value
		 * data[2]: Pre-load Register 0 Value
		 * data[3]: Pre-load Register 1 Value
		 * data[4]: Conter Control Register
		 */
		a4l_dbg(1, drv_dbg, dev, "s526_gpct_insn_config: Configuring PTG\n");
		subdpriv->config[subdev_channel].app = PulseTrainGeneration;

		/* Set Counter Mode Register */
		cmReg.value = (short)(data[1] & 0xFFFF);
		cmReg.reg.preloadRegSel = 0; /* PR0 */
		outw(cmReg.value, ADDR_CHAN_REG(REG_C0M, subdev_channel));

		/* Load the pre-load register 0 high word */
		value = (short)((data[2] >> 16) & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0H, subdev_channel));

		/* Load the pre-load register 0 low word */
		value = (short)(data[2] & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0L, subdev_channel));

		/* Set Counter Mode Register */
		cmReg.value = (short)(data[1] & 0xFFFF);
		cmReg.reg.preloadRegSel = 1; /* PR1 */
		outw(cmReg.value, ADDR_CHAN_REG(REG_C0M, subdev_channel));

		/* Load the pre-load register 1 high word */
		value = (short)((data[3] >> 16) & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0H, subdev_channel));

		/* Load the pre-load register 1 low word */
		value = (short)(data[3] & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0L, subdev_channel));

		/* Write the Counter Control Register */
		if (data[4] != 0) {
			value = (short)(data[4] & 0xFFFF);
			outw(value, ADDR_CHAN_REG(REG_C0C, subdev_channel));
		}
		break;

	default:
		a4l_err(dev, "s526_gpct_insn_config: unsupported GPCT_insn_config\n");
		return -EINVAL;
		break;
	}

	return 0;
}

static int s526_gpct_rinsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint32_t *data = (uint32_t *)insn->data;
	int counter_channel = CR_CHAN(insn->chan_desc);
	unsigned short datalow;
	unsigned short datahigh;
	int i;

	if (insn->data_size <= 0) {
		a4l_err(dev, "s526_gpct_rinsn: data size should be > 0\n");
		return -EINVAL;
	}

	for (i = 0; i < insn->data_size / sizeof(uint32_t); i++) {
		datalow = inw(ADDR_CHAN_REG(REG_C0L, counter_channel));
		datahigh = inw(ADDR_CHAN_REG(REG_C0H, counter_channel));
		data[i] = (int)(datahigh & 0x00FF);
		data[i] = (data[i] << 16) | (datalow & 0xFFFF);
		a4l_dbg(1, drv_dbg, dev,
			"s526_gpct_rinsn GPCT[%d]: %x(0x%04x, 0x%04x)\n",
			counter_channel, data[i], datahigh, datalow);
	}

	return 0;
}

static int s526_gpct_winsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	struct s526_subd_gpct_priv *subdpriv =
	    (struct s526_subd_gpct_priv *)subd->priv;
	uint32_t *data = (uint32_t *)insn->data;
	int subdev_channel = CR_CHAN(insn->chan_desc);
	short value;
	union cmReg cmReg;

	a4l_dbg(1, drv_dbg, dev,
		"s526_gpct_winsn: GPCT_INSN_WRITE on channel %d\n",
		subdev_channel);

	cmReg.value = inw(ADDR_CHAN_REG(REG_C0M, subdev_channel));
	a4l_dbg(1, drv_dbg, dev,
		"s526_gpct_winsn: Counter Mode Register: %x\n", cmReg.value);

	/* Check what Application of Counter this channel is configured for */
	switch (subdpriv->config[subdev_channel].app) {
	case PositionMeasurement:
		a4l_dbg(1, drv_dbg, dev, "s526_gpct_winsn: INSN_WRITE: PM\n");
		outw(0xFFFF & ((*data) >> 16), ADDR_CHAN_REG(REG_C0H,
							     subdev_channel));
		outw(0xFFFF & (*data),
		     ADDR_CHAN_REG(REG_C0L, subdev_channel));
		break;

	case SinglePulseGeneration:
		a4l_dbg(1, drv_dbg, dev, "s526_gpct_winsn: INSN_WRITE: SPG\n");
		outw(0xFFFF & ((*data) >> 16), ADDR_CHAN_REG(REG_C0H,
							     subdev_channel));
		outw(0xFFFF & (*data),
		     ADDR_CHAN_REG(REG_C0L, subdev_channel));
		break;

	case PulseTrainGeneration:
		/*
		 * data[0] contains the PULSE_WIDTH
		 * data[1] contains the PULSE_PERIOD
		 * @pre PULSE_PERIOD > PULSE_WIDTH > 0
		 * The above periods must be expressed as a multiple of the
		 * pulse frequency on the selected source
		 */
		a4l_dbg(1, drv_dbg, dev, "s526_gpct_winsn: INSN_WRITE: PTG\n");
		if ((data[1] > data[0]) && (data[0] > 0)) {
			(subdpriv->config[subdev_channel]).data[0] = data[0];
			(subdpriv->config[subdev_channel]).data[1] = data[1];
		} else {
			a4l_err(dev,
				"s526_gpct_winsn: INSN_WRITE: PTG: Problem with Pulse params -> %du %du\n",
				data[0], data[1]);
			return -EINVAL;
		}

		value = (short)((*data >> 16) & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0H, subdev_channel));
		value = (short)(*data & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0L, subdev_channel));
		break;
	default:		/* Impossible */
		a4l_err(dev,
			"s526_gpct_winsn: INSN_WRITE: Functionality %d not implemented yet\n",
			 subdpriv->config[subdev_channel].app);
		return -EINVAL;
	}

	return 0;
}

static int s526_ai_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	struct s526_subd_ai_priv *subdpriv =
	    (struct s526_subd_ai_priv *)subd->priv;
	unsigned int *data = (unsigned int *)insn->data;

	if (insn->data_size < sizeof(unsigned int))
		return -EINVAL;

	/* data[0] : channels was set in relevant bits.
	 * data[1] : delay
	 */
	/* COMMENT: abbotti 2008-07-24: I don't know why you'd want to
	 * enable channels here.  The channel should be enabled in the
	 * INSN_READ handler. */

	/* Enable ADC interrupt */
	outw(ISR_ADC_DONE, ADDR_REG(REG_IER));
	a4l_dbg(1, drv_dbg, dev,
		"s526_ai_insn_config: ADC current value: 0x%04x\n",
		inw(ADDR_REG(REG_ADC)));

	subdpriv->config = (data[0] & 0x3FF) << 5;
	if (data[1] > 0)
		subdpriv->config |= 0x8000; /* set the delay */

	subdpriv->config |= 0x0001; /* ADC start bit. */

	return 0;
}

static int s526_ai_rinsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	struct s526_subd_ai_priv *subdpriv =
	    (struct s526_subd_ai_priv *)subd->priv;
	uint16_t *data = (uint16_t *)insn->data;
	int n, i;
	int chan = CR_CHAN(insn->chan_desc);
	uint16_t value;
	uint16_t d;
	uint16_t status;

	/* Set configured delay, enable channel for this channel only,
	 * select "ADC read" channel, set "ADC start" bit. */
	value = (subdpriv->config & 0x8000) |
	    ((1 << 5) << chan) | (chan << 1) | 0x0001;

	/* convert n samples */
	for (n = 0; n < insn->data_size / sizeof(uint16_t); n++) {
		/* trigger conversion */
		outw(value, ADDR_REG(REG_ADC));
		a4l_dbg(1, drv_dbg, dev, "s526_ai_rinsn: Wrote 0x%04x to ADC\n",
			value);

		/* wait for conversion to end */
		for (i = 0; i < S526_AI_TIMEOUT; i++) {
			status = inw(ADDR_REG(REG_ISR));
			if (status & ISR_ADC_DONE) {
				outw(ISR_ADC_DONE, ADDR_REG(REG_ISR));
				break;
			}
		}
		if (i == S526_AI_TIMEOUT) {
			a4l_warn(dev, "s526_ai_rinsn: ADC(0x%04x) timeout\n",
				 inw(ADDR_REG(REG_ISR)));
			return -ETIMEDOUT;
		}

		/* read data */
		d = inw(ADDR_REG(REG_ADD));
		a4l_dbg(1, drv_dbg, dev, "s526_ai_rinsn: AI[%d]=0x%04x\n",
			n, (uint16_t)(d & 0xFFFF));

		/* munge data */
		data[n] = d ^ 0x8000;
	}

	return 0;
}

static int s526_ao_winsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	struct s526_subd_ao_priv *subdpriv =
	    (struct s526_subd_ao_priv *)subd->priv;
	uint16_t *data = (uint16_t *)insn->data;
	int i;
	int chan = CR_CHAN(insn->chan_desc);
	uint16_t val;

	val = chan << 1;
	outw(val, ADDR_REG(REG_DAC));

	for (i = 0; i < insn->data_size / sizeof(uint16_t); i++) {
		outw(data[i], ADDR_REG(REG_ADD)); /* write the data to preload register */
		subdpriv->readback[chan] = data[i];
		outw(val + 1, ADDR_REG(REG_DAC)); /* starts the D/A conversion. */
	}

	return 0;
}

static int s526_ao_rinsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct s526_subd_ao_priv *subdpriv =
		(struct s526_subd_ao_priv *)subd->priv;
	uint16_t *data = (uint16_t *)insn->data;
	int i;
	int chan = CR_CHAN(insn->chan_desc);

	for (i = 0; i < insn->data_size / sizeof(uint16_t); i++)
		data[i] = subdpriv->readback[chan];

	return 0;
}

static int s526_dio_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	struct s526_subd_dio_priv *subdpriv =
	    (struct s526_subd_dio_priv *)subd->priv;
	unsigned int *data = (unsigned int *)insn->data;
	int chan = CR_CHAN(insn->chan_desc);
	int group, mask;

	group = chan >> 2;
	mask = 0xF << (group << 2);

	switch (data[0]) {
	case A4L_INSN_CONFIG_DIO_OUTPUT:
		subdpriv->state |= 1 << (group + 10); /* bit 10/11 set the
						       * group 1/2's mode */
		subdpriv->io_bits |= mask;
		break;
	case A4L_INSN_CONFIG_DIO_INPUT:
		subdpriv->state &= ~(1 << (group + 10)); /* 1 is output, 0 is
							  * input. */
		subdpriv->io_bits &= ~mask;
		break;
	case A4L_INSN_CONFIG_DIO_QUERY:
		data[1] =
		    (subdpriv->io_bits & mask) ? A4L_OUTPUT : A4L_INPUT;
		return 0;
	default:
		return -EINVAL;
	}

	outw(subdpriv->state, ADDR_REG(REG_DIO));

	return 0;
}

static int s526_dio_insn_bits(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	struct s526_subd_dio_priv *subdpriv =
		(struct s526_subd_dio_priv *)subd->priv;
	uint8_t *data = (uint8_t *)insn->data;

	if (insn->data_size != 2 * sizeof(uint8_t))
		return -EINVAL;

	if (data[0]) {
		subdpriv->state &= ~(data[0]);
		subdpriv->state |= data[0] & data[1];

		outw(subdpriv->state, ADDR_REG(REG_DIO));
	}

	data[1] = inw(ADDR_REG(REG_DIO)) & 0xFF; /* low 8 bits are the data */

	return 0;
}

/* --- Channels descriptor --- */

static struct a4l_channels_desc s526_chan_desc_gpct = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = S526_GPCT_CHANS,
	.chans = {
		{A4L_CHAN_AREF_GROUND, S526_GPCT_BITS},
	},
};

static struct a4l_channels_desc s526_chan_desc_ai = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = S526_AI_CHANS,
	.chans = {
		{A4L_CHAN_AREF_GROUND, S526_AI_BITS},
	},
};

static struct a4l_channels_desc s526_chan_desc_ao = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = S526_AO_CHANS,
	.chans = {
		{A4L_CHAN_AREF_GROUND, S526_AO_BITS},
	},
};

static struct a4l_channels_desc s526_chan_desc_dio = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = S526_DIO_CHANS,
	.chans = {
		{A4L_CHAN_AREF_GROUND, S526_DIO_BITS},
	},
};

/* --- Subdevice initialization functions --- */

/* General purpose counter/timer (gpct) */
static void setup_subd_gpct(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_COUNTER;
	subd->chan_desc = &s526_chan_desc_gpct;
	subd->insn_read = s526_gpct_rinsn;
	subd->insn_config = s526_gpct_insn_config;
	subd->insn_write = s526_gpct_winsn;
}

/* Analog input subdevice */
static void setup_subd_ai(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_AI;
	subd->chan_desc = &s526_chan_desc_ai;
	subd->rng_desc = &a4l_range_bipolar10;
	subd->insn_read = s526_ai_rinsn;
	subd->insn_config = s526_ai_insn_config;
}

/* Analog output subdevice */
static void setup_subd_ao(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_AO;
	subd->chan_desc = &s526_chan_desc_ao;
	subd->rng_desc = &a4l_range_bipolar10;
	subd->insn_write = s526_ao_winsn;
	subd->insn_read = s526_ao_rinsn;
}

/* Digital i/o subdevice */
static void setup_subd_dio(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_DIO;
	subd->chan_desc = &s526_chan_desc_dio;
	subd->rng_desc = &range_digital;
	subd->insn_bits = s526_dio_insn_bits;
	subd->insn_config = s526_dio_insn_config;
}

struct setup_subd {
	void (*setup_func) (struct a4l_subdevice *);
	int sizeof_priv;
};

static struct setup_subd setup_subds[4] = {
	{
		.setup_func = setup_subd_gpct,
		.sizeof_priv = sizeof(struct s526_subd_gpct_priv),
	},
	{
		.setup_func = setup_subd_ai,
		.sizeof_priv = sizeof(struct s526_subd_ai_priv),
	},
	{
		.setup_func = setup_subd_ao,
		.sizeof_priv = sizeof(struct s526_subd_ao_priv),
	},
	{
		.setup_func = setup_subd_dio,
		.sizeof_priv = sizeof(struct s526_subd_dio_priv),
	},
};

static int dev_s526_attach(struct a4l_device *dev, a4l_lnkdesc_t *arg)
{
	int io_base;
	int i;
	int err = 0;

	if (arg->opts == NULL || arg->opts_size < sizeof(unsigned long)) {
		a4l_warn(dev,
			 "dev_s526_attach: no attach options specified; "
			 "using defaults: addr=0x%x\n",
			 S526_DEFAULT_ADDRESS);
		io_base = S526_DEFAULT_ADDRESS;
	} else {
		io_base = ((unsigned long *)arg->opts)[0];
	}

	if (!request_region(io_base, S526_IOSIZE, "s526")) {
		a4l_err(dev, "dev_s526_attach: I/O port conflict\n");
		return -EIO;
	}

	/* Allocate the subdevice structures. */
	for (i = 0; i < 4; i++) {
		struct a4l_subdevice *subd = a4l_alloc_subd(setup_subds[i].sizeof_priv,
						  setup_subds[i].setup_func);

		if (subd == NULL)
			return -ENOMEM;

		err = a4l_add_subd(dev, subd);
		if (err != i)
			return err;
	}

	devpriv->io_base = io_base;

	a4l_info(dev, " attached (address = 0x%x)\n", io_base);

	return 0;
}

static int dev_s526_detach(struct a4l_device *dev)
{
	int err = 0;

	if (devpriv->io_base != 0)
		release_region(devpriv->io_base, S526_IOSIZE);

	return err;
}

static struct a4l_driver drv_s526 = {
	.owner = THIS_MODULE,
	.board_name = "analogy_s526",
	.driver_name = "s526",
	.attach = dev_s526_attach,
	.detach = dev_s526_detach,
	.privdata_size = sizeof(s526_priv_t),
};

static int __init drv_s526_init(void)
{
	return a4l_register_drv(&drv_s526);
}

static void __exit drv_s526_cleanup(void)
{
	a4l_unregister_drv(&drv_s526);
}

MODULE_DESCRIPTION("Analogy driver for Sensoray Model 526 board.");
MODULE_LICENSE("GPL");

module_init(drv_s526_init);
module_exit(drv_s526_cleanup);
