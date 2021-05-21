/*
 * Hardware driver for NI Mite PCI interface chip
 * Copyright (C) 1999 David A. Schleef <ds@schleef.org>
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
#ifndef __ANALOGY_NI_MIO_H__
#define __ANALOGY_NI_MIO_H__

/* Debug stuff */

#ifdef CONFIG_DEBUG_MIO
#define MDPRINTK(fmt, args...) rtdm_printk(format, ##args)
#else /* !CONFIG_DEBUG_MIO */
#define MDPRINTK(fmt, args...)
#endif /* CONFIG_DEBUG_MIO */

/* Subdevice related defines */

#define AIMODE_NONE		0
#define AIMODE_HALF_FULL	1
#define AIMODE_SCAN		2
#define AIMODE_SAMPLE		3

#define NI_AI_SUBDEV		0
#define NI_AO_SUBDEV		1
#define NI_DIO_SUBDEV		2
#define NI_8255_DIO_SUBDEV	3
#define NI_UNUSED_SUBDEV	4
#define NI_CALIBRATION_SUBDEV	5
#define NI_EEPROM_SUBDEV	6
#define NI_PFI_DIO_SUBDEV	7
#define NI_CS5529_CALIBRATION_SUBDEV 8
#define NI_SERIAL_SUBDEV	9
#define NI_RTSI_SUBDEV		10
#define NI_GPCT0_SUBDEV		11
#define NI_GPCT1_SUBDEV		12
#define NI_FREQ_OUT_SUBDEV	13
#define NI_NUM_SUBDEVICES	14

#define NI_GPCT_SUBDEV(x)	((x == 1) ? NI_GPCT1_SUBDEV : NI_GPCT0_SUBDEV)

#define TIMEBASE_1_NS		50
#define TIMEBASE_2_NS		10000

#define SERIAL_DISABLED		0
#define SERIAL_600NS		600
#define SERIAL_1_2US		1200
#define SERIAL_10US		10000

/* PFI digital filtering options for ni m-series for use with
   INSN_CONFIG_FILTER. */
#define NI_PFI_FILTER_OFF	0x0
#define NI_PFI_FILTER_125ns	0x1
#define NI_PFI_FILTER_6425ns	0x2
#define NI_PFI_FILTER_2550us	0x3

/* Signals which can be routed to an NI PFI pin on an m-series board
   with INSN_CONFIG_SET_ROUTING. These numbers are also returned by
   INSN_CONFIG_GET_ROUTING on pre-m-series boards, even though their
   routing cannot be changed. The numbers assigned are not arbitrary,
   they correspond to the bits required to program the board. */
#define NI_PFI_OUTPUT_PFI_DEFAULT	0
#define NI_PFI_OUTPUT_AI_START1		1
#define NI_PFI_OUTPUT_AI_START2		2
#define NI_PFI_OUTPUT_AI_CONVERT	3
#define NI_PFI_OUTPUT_G_SRC1		4
#define NI_PFI_OUTPUT_G_GATE1		5
#define NI_PFI_OUTPUT_AO_UPDATE_N	6
#define NI_PFI_OUTPUT_AO_START1		7
#define NI_PFI_OUTPUT_AI_START_PULSE	8
#define NI_PFI_OUTPUT_G_SRC0		9
#define NI_PFI_OUTPUT_G_GATE0		10
#define NI_PFI_OUTPUT_EXT_STROBE	11
#define NI_PFI_OUTPUT_AI_EXT_MUX_CLK	12
#define NI_PFI_OUTPUT_GOUT0		13
#define NI_PFI_OUTPUT_GOUT1		14
#define NI_PFI_OUTPUT_FREQ_OUT		15
#define NI_PFI_OUTPUT_PFI_DO		16
#define NI_PFI_OUTPUT_I_ATRIG		17
#define NI_PFI_OUTPUT_RTSI0		18
#define NI_PFI_OUTPUT_PXI_STAR_TRIGGER_IN 26
#define NI_PFI_OUTPUT_SCXI_TRIG1	27
#define NI_PFI_OUTPUT_DIO_CHANGE_DETECT_RTSI 28
#define NI_PFI_OUTPUT_CDI_SAMPLE	29
#define NI_PFI_OUTPUT_CDO_UPDATE	30

static inline unsigned int NI_PFI_OUTPUT_RTSI(unsigned rtsi_channel) {
	return NI_PFI_OUTPUT_RTSI0 + rtsi_channel;
}

/* Ranges declarations */

extern struct a4l_rngdesc a4l_range_ni_E_ai;
extern struct a4l_rngdesc a4l_range_ni_E_ai_limited;
extern struct a4l_rngdesc a4l_range_ni_E_ai_limited14;
extern struct a4l_rngdesc a4l_range_ni_E_ai_bipolar4;
extern struct a4l_rngdesc a4l_range_ni_E_ai_611x;
extern struct a4l_rngdesc range_ni_E_ai_622x;
extern struct a4l_rngdesc range_ni_E_ai_628x;
extern struct a4l_rngdesc a4l_range_ni_S_ai_6143;
extern struct a4l_rngdesc a4l_range_ni_E_ao_ext;

/* Misc functions declarations */

int a4l_ni_E_interrupt(unsigned int irq, void *d);
int a4l_ni_E_init(struct a4l_device *dev);


#endif /* !__ANALOGY_NI_MIO_H__ */
