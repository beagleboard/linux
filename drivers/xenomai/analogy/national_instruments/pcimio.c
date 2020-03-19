/*
 * Hardware driver for NI PCI-MIO E series cards
 *
 * Copyright (C) 1997-8 David A. Schleef <ds@schleef.org>
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
 *
 * Description: National Instruments PCI-MIO-E series and M series
 * (all boards)
 *
 * Author: ds, John Hallen, Frank Mori Hess, Rolf Mueller, Herbert Peremans,
 * Herman Bruyninckx, Terry Barnaby
 * Status: works
 * Devices: [National Instruments] PCI-MIO-16XE-50 (ni_pcimio),
 * PCI-MIO-16XE-10, PXI-6030E, PCI-MIO-16E-1, PCI-MIO-16E-4, PCI-6014,
 * PCI-6040E,PXI-6040E, PCI-6030E, PCI-6031E, PCI-6032E, PCI-6033E,
 * PCI-6071E, PCI-6023E, PCI-6024E, PCI-6025E, PXI-6025E, PCI-6034E,
 * PCI-6035E, PCI-6052E, PCI-6110, PCI-6111, PCI-6220, PCI-6221,
 * PCI-6224, PCI-6225, PCI-6229, PCI-6250, PCI-6251, PCIe-6251,
 * PCI-6254, PCI-6259, PCIe-6259, PCI-6280, PCI-6281, PXI-6281,
 * PCI-6284, PCI-6289, PCI-6711, PXI-6711, PCI-6713, PXI-6713,
 * PXI-6071E, PCI-6070E, PXI-6070E, PXI-6052E, PCI-6036E, PCI-6731,
 * PCI-6733, PXI-6733, PCI-6143, PXI-6143
 *
 * These boards are almost identical to the AT-MIO E series, except that
 * they use the PCI bus instead of ISA (i.e., AT).  See the notes for
 * the ni_atmio.o driver for additional information about these boards.
 *
 * By default, the driver uses DMA to transfer analog input data to
 * memory.  When DMA is enabled, not all triggering features are
 * supported.
 *
 * Note that the PCI-6143 is a simultaneous sampling device with 8
 * convertors. With this board all of the convertors perform one
 * simultaneous sample during a scan interval. The period for a scan
 * is used for the convert time in an Analgoy cmd. The convert trigger
 * source is normally set to TRIG_NOW by default.
 *
 * The RTSI trigger bus is supported on these cards on subdevice
 * 10. See the Analogy library documentation for details.
 *
 * References:
 * 341079b.pdf  PCI E Series Register-Level Programmer Manual
 * 340934b.pdf  DAQ-STC reference manual
 * 322080b.pdf  6711/6713/6715 User Manual
 * 320945c.pdf  PCI E Series User Manual
 * 322138a.pdf  PCI-6052E and DAQPad-6052E User Manual
 *
 * ISSUES:
 * - When DMA is enabled, XXX_EV_CONVERT does not work correctly.
 * - Calibration is not fully implemented
 * - SCXI is probably broken for m-series boards
 * - Digital I/O may not work on 673x.
 * - Information (number of channels, bits, etc.) for some devices may
 *   be incorrect.  Please check this and submit a bug if there are
 *   problems for your device.
 * - Need to deal with external reference for DAC, and other DAC
 *   properties in board properties
 * - Deal with at-mio-16de-10 revision D to N changes, etc.
 * - Need to add other CALDAC type
 * - Need to slow down DAC loading.  I don't trust NI's claim that two
 *   writes to the PCI bus slows IO enough.  I would prefer to use
 *   a4l_udelay().  Timing specs: (clock)
 *     AD8522   30ns
 *     DAC8043  120ns
 *     DAC8800  60ns
 *     MB88341   ?
 *
 */

#include <linux/module.h>
#include <rtdm/analogy/device.h>

#include "../intel/8255.h"
#include "ni_stc.h"
#include "ni_mio.h"
#include "mite.h"

#define PCIMIO_IRQ_POLARITY 1

/* The following two tables must be in the same order */
static struct pci_device_id ni_pci_table[] __maybe_unused = {
	{ PCI_VENDOR_ID_NATINST, 0x0162, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1170, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1180, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1190, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x11b0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x11c0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x11d0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1270, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1330, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1340, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1350, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x14e0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x14f0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1580, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x15b0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1880, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1870, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x18b0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x18c0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2410, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2420, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2430, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2890, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x28c0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2a60, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2a70, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2a80, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2ab0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2b80, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2b90, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2c80, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2ca0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70aa, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70ab, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70ac, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70af, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70b0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70b4, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70b6, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70b7, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70b8, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70bc, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70bd, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70bf, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70c0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70f2, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x710d, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x716c, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x717f, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x71bc, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x717d, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, ni_pci_table);

/* These are not all the possible ao ranges for 628x boards.
 They can do OFFSET +- REFERENCE where OFFSET can be
 0V, 5V, APFI<0,1>, or AO<0...3> and RANGE can
 be 10V, 5V, 2V, 1V, APFI<0,1>, AO<0...3>.  That's
 63 different possibilities.  An AO channel
 can not act as it's own OFFSET or REFERENCE.
*/

#if 0
static struct a4l_rngtab rng_ni_M_628x_ao = { 8, {
	RANGE(-10, 10),
	RANGE(-5, 5),
	RANGE(-2, 2),
	RANGE(-1, 1),
	RANGE(-5, 15),
	RANGE(0, 10),
	RANGE(3, 7),
	RANGE(4, 6),
	RANGE_ext(-1, 1)
}};
static struct a4l_rngdesc range_ni_M_628x_ao =
	RNG_GLOBAL(rng_ni_M_628x_ao);
#endif

static struct a4l_rngtab rng_ni_M_625x_ao = { 3, {
	RANGE(-10, 10),
	RANGE(-5, 5),
	RANGE_ext(-1, 1)
}};
static struct a4l_rngdesc range_ni_M_625x_ao =
	RNG_GLOBAL(rng_ni_M_625x_ao);

static struct a4l_rngtab rng_ni_M_622x_ao = { 1, {
	RANGE(-10, 10),
}};
static struct a4l_rngdesc range_ni_M_622x_ao =
	RNG_GLOBAL(rng_ni_M_622x_ao);

static ni_board ni_boards[]={
	{       device_id:      0x0162, // NI also says 0x1620.  typo?
		name:           "pci-mio-16xe-50",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  2048,
		alwaysdither:   1,
		gainlkup:       ai_gain_8,
		ai_speed:	50000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  0,
		.ao_range_table = &a4l_range_bipolar10,
		ao_unipolar:    0,
		ao_speed:	50000,
		.num_p0_dio_channels = 8,
		caldac:         {dac8800,dac8043},
		has_8255:       0,
	},
	{       device_id:      0x1170,
		name:           "pci-mio-16xe-10", // aka pci-6030E
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  2048,
		.ao_range_table = &a4l_range_ni_E_ao_ext,
		ao_unipolar:    1,
		ao_speed:	10000,
		.num_p0_dio_channels = 8,
		caldac:         {dac8800,dac8043,ad8522},
		has_8255:       0,
	},
	{	device_id:      0x28c0,
		name:           "pci-6014",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_4,
		ai_speed:       5000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  0,
		.ao_range_table = &a4l_range_bipolar10,
		ao_unipolar:    0,
		ao_speed:	100000,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804_debug},
		has_8255:       0,
	},
	{       device_id:      0x11d0,
		name:           "pxi-6030e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  2048,
		.ao_range_table = &a4l_range_ni_E_ao_ext,
		ao_unipolar:    1,
		ao_speed:	10000,
		.num_p0_dio_channels = 8,
		caldac:         {dac8800,dac8043,ad8522},
		has_8255:       0,
	},

	{       device_id:      0x1180,
		name:           "pci-mio-16e-1",	/* aka pci-6070e */
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_16,
		ai_speed:	800,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  2048,
		.ao_range_table = &a4l_range_ni_E_ao_ext,
		ao_unipolar:    1,
		ao_speed:	1000,
		.num_p0_dio_channels = 8,
		caldac:         {mb88341},
		has_8255:       0,
	},
	{       device_id:      0x1190,
		name:           "pci-mio-16e-4", /* aka pci-6040e */
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_16,
		/* Note: there have been reported problems with full speed
		 * on this board */
		ai_speed:	2000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  512,
		.ao_range_table = &a4l_range_ni_E_ao_ext,
		ao_unipolar:    1,
		ao_speed:	1000,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804_debug}, // doc says mb88341
		has_8255:       0,
	},
	{       device_id:      0x11c0,
		name:           "pxi-6040e",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_16,
		ai_speed:	2000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  512,
		.ao_range_table = &a4l_range_ni_E_ao_ext,
		ao_unipolar:    1,
		ao_speed:	1000,
		.num_p0_dio_channels = 8,
		caldac:         {mb88341},
		has_8255:       0,
	},

	{       device_id:      0x1330,
		name:           "pci-6031e",
		n_adchan:       64,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  2048,
		.ao_range_table = &a4l_range_ni_E_ao_ext,
		ao_unipolar:    1,
		ao_speed:	10000,
		.num_p0_dio_channels = 8,
		caldac:         {dac8800,dac8043,ad8522},
		has_8255:       0,
	},
	{       device_id:      0x1270,
		name:           "pci-6032e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  0,
		ao_unipolar:    0,
		.num_p0_dio_channels = 8,
		caldac:         {dac8800,dac8043,ad8522},
		has_8255:       0,
	},
	{       device_id:      0x1340,
		name:           "pci-6033e",
		n_adchan:       64,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  0,
		ao_unipolar:    0,
		.num_p0_dio_channels = 8,
		caldac:         {dac8800,dac8043,ad8522},
		has_8255:       0,
	},
	{       device_id:      0x1350,
		name:           "pci-6071e",
		n_adchan:       64,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_16,
		ai_speed:	800,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  2048,
		.ao_range_table = &a4l_range_ni_E_ao_ext,
		ao_unipolar:    1,
		ao_speed:	1000,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804_debug},
		has_8255:       0,
	},
	{       device_id:      0x2a60,
		name:           "pci-6023e",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_4,
		ai_speed:	5000,
		n_aochan:       0,
		aobits:         0,
		ao_unipolar:    0,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804_debug}, /* manual is wrong */
		has_8255:	0,
	},
	{       device_id:      0x2a70,
		name:           "pci-6024e",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_4,
		ai_speed:	5000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  0,
		.ao_range_table = &a4l_range_bipolar10,
		ao_unipolar:    0,
		ao_speed:	100000,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804_debug}, /* manual is wrong */
		has_8255:	0,
	},
	{       device_id:      0x2a80,
		name:           "pci-6025e",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_4,
		ai_speed:	5000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  0,
		.ao_range_table = &a4l_range_bipolar10,
		ao_unipolar:    0,
		ao_speed:	100000,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804_debug}, /* manual is wrong */
		has_8255:	1,
	},
	{       device_id:      0x2ab0,
		name:           "pxi-6025e",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_4,
		ai_speed:	5000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  0,
		.ao_range_table = &a4l_range_ni_E_ao_ext,
		ao_unipolar:    1,
		ao_speed:	100000,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804_debug}, /* manual is wrong */
		has_8255:	1,
	},

	{       device_id:      0x2ca0,
		name:           "pci-6034e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_4,
		ai_speed:	5000,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  0,
		ao_unipolar:    0,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804_debug},
		has_8255:	0,
	},
	{       device_id:      0x2c80,
		name:           "pci-6035e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_4,
		ai_speed:	5000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  0,
		.ao_range_table = &a4l_range_bipolar10,
		ao_unipolar:    0,
		ao_speed:	100000,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804_debug},
		has_8255:	0,
	},
	{       device_id:      0x18b0,
		name:           "pci-6052e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_16,
		ai_speed:	3000,
		n_aochan:       2,
		aobits:         16,
		ao_unipolar:    1,
		ao_fifo_depth:  2048,
		.ao_range_table = &a4l_range_ni_E_ao_ext,
		ao_speed:	3000,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804_debug,ad8804_debug,ad8522}, /* manual is wrong */
	},
	{       device_id:      0x14e0,
		name:           "pci-6110",
		n_adchan:       4,
		adbits:         12,
		ai_fifo_depth:  8192,
		alwaysdither:   0,
		gainlkup:       ai_gain_611x,
		ai_speed:	200,
		n_aochan:       2,
		aobits:         16,
		reg_type:	ni_reg_611x,
		.ao_range_table = &a4l_range_bipolar10,
		ao_unipolar:    0,
		ao_fifo_depth:  2048,
		ao_speed:	250,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804,ad8804},
	},
	{       device_id:      0x14f0,
		name:           "pci-6111",
		n_adchan:       2,
		adbits:         12,
		ai_fifo_depth:  8192,
		alwaysdither:   0,
		gainlkup:       ai_gain_611x,
		ai_speed:	200,
		n_aochan:       2,
		aobits:         16,
		reg_type:	ni_reg_611x,
		.ao_range_table = &a4l_range_bipolar10,
		ao_unipolar:    0,
		ao_fifo_depth:  2048,
		ao_speed:	250,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804,ad8804},
	},
#if 0 /* Need device IDs */
	/* The 6115 boards probably need their own driver */
	{       device_id:      0x2ed0,
		name:           "pci-6115",
		n_adchan:       4,
		adbits:         12,
		ai_fifo_depth:  8192,
		alwaysdither:   0,
		gainlkup:       ai_gain_611x,
		ai_speed:	100,
		n_aochan:       2,
		aobits:         16,
		ao_671x:	1,
		ao_unipolar:    0,
		ao_fifo_depth:  2048,
		ao_speed:	250,
		.num_p0_dio_channels = 8,
		reg_611x:	1,
		caldac:         {ad8804_debug,ad8804_debug,ad8804_debug},/* XXX */
	},
#endif
#if 0 /* Need device IDs */
	{       device_id:      0x0000,
		name:           "pxi-6115",
		n_adchan:       4,
		adbits:         12,
		ai_fifo_depth:  8192,
		alwaysdither:   0,
		gainlkup:       ai_gain_611x,
		ai_speed:	100,
		n_aochan:       2,
		aobits:         16,
		ao_671x:	1,
		ao_unipolar:    0,
		ao_fifo_depth:  2048,
		ao_speed:	250,
		reg_611x:	1,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804_debug,ad8804_debug,ad8804_debug},/* XXX */
	},
#endif
	{       device_id:      0x1880,
		name:           "pci-6711",
		n_adchan:       0, /* no analog input */
		n_aochan:	4,
		aobits:         12,
		ao_unipolar:    0,
		ao_fifo_depth:  16384, /* data sheet says 8192, but fifo really holds 16384 samples */
		.ao_range_table = &a4l_range_bipolar10,
		ao_speed:	1000,
		.num_p0_dio_channels = 8,
		reg_type:	ni_reg_6711,
		caldac:         {ad8804_debug},
	},
	{       device_id:      0x2b90,
		name:           "pxi-6711",
		n_adchan:       0, /* no analog input */
		n_aochan:	4,
		aobits:         12,
		ao_unipolar:    0,
		ao_fifo_depth:  16384,
		.ao_range_table = &a4l_range_bipolar10,
		ao_speed:	1000,
		.num_p0_dio_channels = 8,
		reg_type:	ni_reg_6711,
		caldac:         {ad8804_debug},
	},
	{       device_id:      0x1870,
		name:           "pci-6713",
		n_adchan:       0, /* no analog input */
		n_aochan:	8,
		aobits:         12,
		ao_unipolar:    0,
		ao_fifo_depth:  16384,
		.ao_range_table = &a4l_range_bipolar10,
		ao_speed:	1000,
		.num_p0_dio_channels = 8,
		reg_type:	ni_reg_6713,
		caldac:         {ad8804_debug,ad8804_debug},
	},
	{       device_id:      0x2b80,
		name:           "pxi-6713",
		n_adchan:       0, /* no analog input */
		n_aochan:	8,
		aobits:         12,
		ao_unipolar:    0,
		ao_fifo_depth:  16384,
		.ao_range_table = &a4l_range_bipolar10,
		ao_speed:	1000,
		.num_p0_dio_channels = 8,
		reg_type:	ni_reg_6713,
		caldac:         {ad8804_debug,ad8804_debug},
	},
	{	device_id:	0x2430,
		name:           "pci-6731",
		n_adchan:       0, /* no analog input */
		n_aochan:	4,
		aobits:         16,
		ao_unipolar:    0,
		ao_fifo_depth:  8192,
		.ao_range_table = &a4l_range_bipolar10,
		ao_speed:	1000,
		.num_p0_dio_channels = 8,
		reg_type:	ni_reg_6711,
		caldac:         {ad8804_debug},
	},
#if 0	/* Need device IDs */
	{       device_id:      0x0,
		name:           "pxi-6731",
		n_adchan:       0, /* no analog input */
		n_aochan:	4,
		aobits:         16,
		ao_unipolar:    0,
		ao_fifo_depth:  8192,
		.ao_range_table = &a4l_range_bipolar10,
		.num_p0_dio_channels = 8,
		reg_type:	ni_reg_6711,
		caldac:         {ad8804_debug},
	},
#endif
	{       device_id:      0x2410,
		name:           "pci-6733",
		n_adchan:       0, /* no analog input */
		n_aochan:	8,
		aobits:         16,
		ao_unipolar:    0,
		ao_fifo_depth:  16384,
		.ao_range_table = &a4l_range_bipolar10,
		ao_speed:	1000,
		.num_p0_dio_channels = 8,
		reg_type:	ni_reg_6713,
		caldac:         {ad8804_debug,ad8804_debug},
	},
	{       device_id:      0x2420,
		name:           "pxi-6733",
		n_adchan:       0, /* no analog input */
		n_aochan:	8,
		aobits:         16,
		ao_unipolar:    0,
		ao_fifo_depth:  16384,
		.ao_range_table = &a4l_range_bipolar10,
		ao_speed:	1000,
		.num_p0_dio_channels = 8,
		reg_type:	ni_reg_6713,
		caldac:         {ad8804_debug,ad8804_debug},
	},
	{	device_id:      0x15b0,
		name:           "pxi-6071e",
		n_adchan:       64,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_16,
		ai_speed:       800,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  2048,
		.ao_range_table = &a4l_range_ni_E_ao_ext,
		ao_unipolar:    1,
		ao_speed:	1000,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804_debug},
		has_8255:       0,
	},
	{	device_id:      0x11b0,
		name:           "pxi-6070e",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_16,
		ai_speed:       800,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  2048,
		.ao_range_table = &a4l_range_ni_E_ao_ext,
		ao_unipolar:    1,
		ao_speed:	1000,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804_debug},
		has_8255:       0,
	},
	{	device_id:      0x18c0,
		name:           "pxi-6052e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_16,
		ai_speed:	3000,
		n_aochan:       2,
		aobits:         16,
		ao_unipolar:    1,
		ao_fifo_depth:  2048,
		.ao_range_table = &a4l_range_ni_E_ao_ext,
		ao_speed:	3000,
		.num_p0_dio_channels = 8,
		caldac:         {mb88341,mb88341,ad8522},
	},
	{	device_id:      0x1580,
		name:           "pxi-6031e",
		n_adchan:       64,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  2048,
		.ao_range_table = &a4l_range_ni_E_ao_ext,
		ao_unipolar:    1,
		ao_speed:	10000,
		.num_p0_dio_channels = 8,
		caldac:         {dac8800,dac8043,ad8522},
	},
	{	device_id:      0x2890,
		name:           "pci-6036e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_4,
		ai_speed:	5000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  0,
		.ao_range_table = &a4l_range_bipolar10,
		ao_unipolar:    0,
		ao_speed:	100000,
		.num_p0_dio_channels = 8,
		caldac:         {ad8804_debug},
		has_8255:	0,
	},
	{	device_id:      0x70b0,
		name:           "pci-6220",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,	//FIXME: guess
		gainlkup:       ai_gain_622x,
		ai_speed:	4000,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  0,
		.num_p0_dio_channels = 8,
		reg_type:	ni_reg_622x,
		ao_unipolar:    0,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x70af,
		name:           "pci-6221",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  4095,
		gainlkup:       ai_gain_622x,
		ai_speed:	4000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  8191,
		.ao_range_table = &a4l_range_bipolar10,
		reg_type:	ni_reg_622x,
		ao_unipolar:    0,
		ao_speed:	1200,
		.num_p0_dio_channels = 8,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x71bc,
		name:           "pci-6221_37pin",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  4095,
		gainlkup:       ai_gain_622x,
		ai_speed:	4000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  8191,
		.ao_range_table = &a4l_range_bipolar10,
		reg_type:	ni_reg_622x,
		ao_unipolar:    0,
		ao_speed:	1200,
		.num_p0_dio_channels = 8,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x70f2,
		name:           "pci-6224",
		n_adchan:       32,
		adbits:         16,
		ai_fifo_depth:  4095,
		gainlkup:       ai_gain_622x,
		ai_speed:	4000,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  0,
		reg_type:	ni_reg_622x,
		ao_unipolar:    0,
		.num_p0_dio_channels = 32,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x716c,
		name:           "pci-6225",
		n_adchan:       80,
		adbits:         16,
		ai_fifo_depth:  4095,
		gainlkup:       ai_gain_622x,
		ai_speed:	4000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  8191,
		.ao_range_table = &range_ni_M_622x_ao,
		reg_type:	ni_reg_622x,
		ao_unipolar:    0,
		ao_speed:	1200,
		.num_p0_dio_channels = 32,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x70aa,
		name:           "pci-6229",
		n_adchan:       32,
		adbits:         16,
		ai_fifo_depth:  4095,
		gainlkup:       ai_gain_622x,
		ai_speed:	4000,
		n_aochan:       4,
		aobits:         16,
		ao_fifo_depth:  8191,
		.ao_range_table = &range_ni_M_622x_ao,
		reg_type:	ni_reg_622x,
		ao_unipolar:    0,
		ao_speed:	1200,
		.num_p0_dio_channels = 32,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x70b4,
		name:           "pci-6250",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  4095,
		.gainlkup = ai_gain_628x,
		ai_speed:	800,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  0,
		reg_type:	ni_reg_625x,
		ao_unipolar:    0,
		.num_p0_dio_channels = 8,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x70b8,
		name:           "pci-6251",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  4095,
		.gainlkup = ai_gain_628x,
		ai_speed:	800,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  8191,
		.ao_range_table = &range_ni_M_625x_ao,
		reg_type:	ni_reg_625x,
		ao_unipolar:    0,
		ao_speed:	357,
		.num_p0_dio_channels = 8,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x717d,
		name:           "pcie-6251",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  4095,
		.gainlkup = ai_gain_628x,
		ai_speed:	800,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  8191,
		.ao_range_table = &range_ni_M_625x_ao,
		reg_type:	ni_reg_625x,
		ao_unipolar:    0,
		ao_speed:	357,
		.num_p0_dio_channels = 8,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x70b7,
		name:           "pci-6254",
		n_adchan:       32,
		adbits:         16,
		ai_fifo_depth:  4095,
		.gainlkup = ai_gain_628x,
		ai_speed:	800,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  0,
		reg_type:	ni_reg_625x,
		ao_unipolar:    0,
		.num_p0_dio_channels = 32,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x70ab,
		name:           "pci-6259",
		n_adchan:       32,
		adbits:         16,
		ai_fifo_depth:  4095,
		.gainlkup = ai_gain_628x,
		ai_speed:	800,
		n_aochan:       4,
		aobits:         16,
		ao_fifo_depth:  8191,
		.ao_range_table = &range_ni_M_625x_ao,
		reg_type:	ni_reg_625x,
		ao_unipolar:    0,
		ao_speed:	357,
		.num_p0_dio_channels = 32,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x717f,
		name:           "pcie-6259",
		n_adchan:       32,
		adbits:         16,
		ai_fifo_depth:  4095,
		.gainlkup = ai_gain_628x,
		ai_speed:	800,
		n_aochan:       4,
		aobits:         16,
		ao_fifo_depth:  8191,
		.ao_range_table = &range_ni_M_625x_ao,
		reg_type:	ni_reg_625x,
		ao_unipolar:    0,
		ao_speed:	357,
		.num_p0_dio_channels = 32,
		.caldac = {caldac_none},
		has_8255:	0,
	},
#if 0 /* TODO: fix data size */
	{	device_id:      0x70b6,
		name:           "pci-6280",
		n_adchan:       16,
		adbits:         18,
		ai_fifo_depth:  2047,
		.gainlkup = ai_gain_628x,
		ai_speed:	1600,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  8191,
		reg_type:	ni_reg_628x,
		ao_unipolar:    0,
		.num_p0_dio_channels = 8,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x70bd,
		name:           "pci-6281",
		n_adchan:       16,
		adbits:         18,
		ai_fifo_depth:  2047,
		.gainlkup = ai_gain_628x,
		ai_speed:	1600,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  8191,
		.ao_range_table = &range_ni_M_628x_ao,
		reg_type:	ni_reg_628x,
		ao_unipolar:    1,
		ao_speed:	357,
		.num_p0_dio_channels = 8,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x70bf,
		name:           "pxi-6281",
		n_adchan:       16,
		adbits:         18,
		ai_fifo_depth:  2047,
		.gainlkup = ai_gain_628x,
		ai_speed:	1600,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  8191,
		.ao_range_table = &range_ni_M_628x_ao,
		reg_type:	ni_reg_628x,
		ao_unipolar:    1,
		ao_speed:	357,
		.num_p0_dio_channels = 8,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x70bc,
		name:           "pci-6284",
		n_adchan:       32,
		adbits:         18,
		ai_fifo_depth:  2047,
		.gainlkup = ai_gain_628x,
		ai_speed:	1600,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  0,
		reg_type:	ni_reg_628x,
		ao_unipolar:    0,
		.num_p0_dio_channels = 32,
		.caldac = {caldac_none},
		has_8255:	0,
	},
	{	device_id:      0x70ac,
		name:           "pci-6289",
		n_adchan:       32,
		adbits:         18,
		ai_fifo_depth:  2047,
		.gainlkup = ai_gain_628x,
		ai_speed:	1600,
		n_aochan:       4,
		aobits:         16,
		ao_fifo_depth:  8191,
		.ao_range_table = &range_ni_M_628x_ao,
		reg_type:	ni_reg_628x,
		ao_unipolar:    1,
		ao_speed:	357,
		.num_p0_dio_channels = 32,
		.caldac = {caldac_none},
		has_8255:	0,
	},
#endif /* TODO: fix data size */
	{	device_id:      0x70C0,
		name:           "pci-6143",
		n_adchan:       8,
		adbits:         16,
		ai_fifo_depth:  1024,
		alwaysdither:   0,
		gainlkup:       ai_gain_6143,
		ai_speed:	4000,
		n_aochan:       0,
		aobits:         0,
		reg_type:	ni_reg_6143,
		ao_unipolar:    0,
		ao_fifo_depth:  0,
		.num_p0_dio_channels = 8,
		.caldac = {ad8804_debug,ad8804_debug},
	},
	{	device_id:      0x710D,
		name:           "pxi-6143",
		n_adchan:       8,
		adbits:         16,
		ai_fifo_depth:  1024,
		alwaysdither:   0,
		gainlkup:       ai_gain_6143,
		ai_speed:	4000,
		n_aochan:       0,
		aobits:         0,
		reg_type:	ni_reg_6143,
		ao_unipolar:    0,
		ao_fifo_depth:  0,
		.num_p0_dio_channels = 8,
		.caldac = {ad8804_debug,ad8804_debug},
	},
};
#define n_pcimio_boards ((sizeof(ni_boards)/sizeof(ni_boards[0])))

/* How we access STC registers */

/* We automatically take advantage of STC registers that can be
 * read/written directly in the I/O space of the board.  Most
 * PCIMIO devices map the low 8 STC registers to iobase+addr*2.
 * The 611x devices map the write registers to iobase+addr*2, and
 * the read registers to iobase+(addr-1)*2. */
/* However, the 611x boards still aren't working, so I'm disabling
 * non-windowed STC access temporarily */

static void e_series_win_out(struct a4l_device *dev, uint16_t data, int reg)
{
	unsigned long flags;

	rtdm_lock_get_irqsave(&devpriv->window_lock, flags);
	ni_writew(reg, Window_Address);
	ni_writew(data, Window_Data);
	rtdm_lock_put_irqrestore(&devpriv->window_lock, flags);
}

static uint16_t e_series_win_in(struct a4l_device *dev, int reg)
{
	unsigned long flags;
	uint16_t ret;

	rtdm_lock_get_irqsave(&devpriv->window_lock, flags);
	ni_writew(reg, Window_Address);
	ret = ni_readw(Window_Data);
	rtdm_lock_put_irqrestore(&devpriv->window_lock,flags);

	return ret;
}

static void m_series_stc_writew(struct a4l_device *dev, uint16_t data, int reg)
{
	unsigned offset;
	switch(reg)
	{
	case ADC_FIFO_Clear:
		offset = M_Offset_AI_FIFO_Clear;
		break;
	case AI_Command_1_Register:
		offset = M_Offset_AI_Command_1;
		break;
	case AI_Command_2_Register:
		offset = M_Offset_AI_Command_2;
		break;
	case AI_Mode_1_Register:
		offset = M_Offset_AI_Mode_1;
		break;
	case AI_Mode_2_Register:
		offset = M_Offset_AI_Mode_2;
		break;
	case AI_Mode_3_Register:
		offset = M_Offset_AI_Mode_3;
		break;
	case AI_Output_Control_Register:
		offset = M_Offset_AI_Output_Control;
		break;
	case AI_Personal_Register:
		offset = M_Offset_AI_Personal;
		break;
	case AI_SI2_Load_A_Register:
		/* This is actually a 32 bit register on m series boards */
		ni_writel(data, M_Offset_AI_SI2_Load_A);
		return;
		break;
	case AI_SI2_Load_B_Register:
		/* This is actually a 32 bit register on m series boards */
		ni_writel(data, M_Offset_AI_SI2_Load_B);
		return;
		break;
	case AI_START_STOP_Select_Register:
		offset = M_Offset_AI_START_STOP_Select;
		break;
	case AI_Trigger_Select_Register:
		offset = M_Offset_AI_Trigger_Select;
		break;
	case Analog_Trigger_Etc_Register:
		offset = M_Offset_Analog_Trigger_Etc;
		break;
	case AO_Command_1_Register:
		offset = M_Offset_AO_Command_1;
		break;
	case AO_Command_2_Register:
		offset = M_Offset_AO_Command_2;
		break;
	case AO_Mode_1_Register:
		offset = M_Offset_AO_Mode_1;
		break;
	case AO_Mode_2_Register:
		offset = M_Offset_AO_Mode_2;
		break;
	case AO_Mode_3_Register:
		offset = M_Offset_AO_Mode_3;
		break;
	case AO_Output_Control_Register:
		offset = M_Offset_AO_Output_Control;
		break;
	case AO_Personal_Register:
		offset = M_Offset_AO_Personal;
		break;
	case AO_Start_Select_Register:
		offset = M_Offset_AO_Start_Select;
		break;
	case AO_Trigger_Select_Register:
		offset = M_Offset_AO_Trigger_Select;
		break;
	case Clock_and_FOUT_Register:
		offset = M_Offset_Clock_and_FOUT;
		break;
	case Configuration_Memory_Clear:
		offset = M_Offset_Configuration_Memory_Clear;
		break;
	case DAC_FIFO_Clear:
		offset = M_Offset_AO_FIFO_Clear;
		break;
	case DIO_Control_Register:
		rtdm_printk("%s: FIXME: register 0x%x does not map cleanly on to m-series boards.\n", __FUNCTION__, reg);
		return;
		break;
	case G_Autoincrement_Register(0):
		offset = M_Offset_G0_Autoincrement;
		break;
	case G_Autoincrement_Register(1):
		offset = M_Offset_G1_Autoincrement;
		break;
	case G_Command_Register(0):
		offset = M_Offset_G0_Command;
		break;
	case G_Command_Register(1):
		offset = M_Offset_G1_Command;
		break;
	case G_Input_Select_Register(0):
		offset = M_Offset_G0_Input_Select;
		break;
	case G_Input_Select_Register(1):
		offset = M_Offset_G1_Input_Select;
		break;
	case G_Mode_Register(0):
		offset = M_Offset_G0_Mode;
		break;
	case G_Mode_Register(1):
		offset = M_Offset_G1_Mode;
		break;
	case Interrupt_A_Ack_Register:
		offset = M_Offset_Interrupt_A_Ack;
		break;
	case Interrupt_A_Enable_Register:
		offset = M_Offset_Interrupt_A_Enable;
		break;
	case Interrupt_B_Ack_Register:
		offset = M_Offset_Interrupt_B_Ack;
		break;
	case Interrupt_B_Enable_Register:
		offset = M_Offset_Interrupt_B_Enable;
		break;
	case Interrupt_Control_Register:
		offset = M_Offset_Interrupt_Control;
		break;
	case IO_Bidirection_Pin_Register:
		offset = M_Offset_IO_Bidirection_Pin;
		break;
	case Joint_Reset_Register:
		offset = M_Offset_Joint_Reset;
		break;
	case RTSI_Trig_A_Output_Register:
		offset = M_Offset_RTSI_Trig_A_Output;
		break;
	case RTSI_Trig_B_Output_Register:
		offset = M_Offset_RTSI_Trig_B_Output;
		break;
	case RTSI_Trig_Direction_Register:
		offset = M_Offset_RTSI_Trig_Direction;
		break;
		/* FIXME: DIO_Output_Register (16 bit reg) is replaced
		by M_Offset_Static_Digital_Output (32 bit) and
		M_Offset_SCXI_Serial_Data_Out (8 bit) */
	default:
		rtdm_printk("%s: bug! unhandled register=0x%x in switch.\n",
			    __FUNCTION__, reg);
		BUG();
		return;
	}
	ni_writew(data, offset);
}

static uint16_t m_series_stc_readw(struct a4l_device *dev, int reg)
{
	unsigned offset;
	switch(reg)
	{
	case AI_Status_1_Register:
		offset = M_Offset_AI_Status_1;
		break;
	case AO_Status_1_Register:
		offset = M_Offset_AO_Status_1;
		break;
	case AO_Status_2_Register:
		offset = M_Offset_AO_Status_2;
		break;
	case DIO_Serial_Input_Register:
		return ni_readb(M_Offset_SCXI_Serial_Data_In);
		break;
	case Joint_Status_1_Register:
		offset = M_Offset_Joint_Status_1;
		break;
	case Joint_Status_2_Register:
		offset = M_Offset_Joint_Status_2;
		break;
	case G_Status_Register:
		offset = M_Offset_G01_Status;
		break;
	default:
		rtdm_printk("%s: bug! "
			    "unhandled register=0x%x in switch.\n",
			    __FUNCTION__, reg);
		BUG();
		return 0;
		break;
	}
	return ni_readw(offset);
}

static void m_series_stc_writel(struct a4l_device *dev, uint32_t data, int reg)
{
	unsigned offset;

	switch(reg)
	{
	case AI_SC_Load_A_Registers:
		offset = M_Offset_AI_SC_Load_A;
		break;
	case AI_SI_Load_A_Registers:
		offset = M_Offset_AI_SI_Load_A;
		break;
	case AO_BC_Load_A_Register:
		offset = M_Offset_AO_BC_Load_A;
		break;
	case AO_UC_Load_A_Register:
		offset = M_Offset_AO_UC_Load_A;
		break;
	case AO_UI_Load_A_Register:
		offset = M_Offset_AO_UI_Load_A;
		break;
	case G_Load_A_Register(0):
		offset = M_Offset_G0_Load_A;
		break;
	case G_Load_A_Register(1):
		offset = M_Offset_G1_Load_A;
		break;
	case G_Load_B_Register(0):
		offset = M_Offset_G0_Load_B;
		break;
	case G_Load_B_Register(1):
		offset = M_Offset_G1_Load_B;
		break;
	default:
		rtdm_printk("%s: bug! unhandled register=0x%x in switch.\n",
			    __FUNCTION__, reg);
		BUG();
		return;
	}
	ni_writel(data, offset);
}

static uint32_t m_series_stc_readl(struct a4l_device *dev, int reg)
{
	unsigned offset;
	switch(reg)
	{
	case G_HW_Save_Register(0):
		offset = M_Offset_G0_HW_Save;
		break;
	case G_HW_Save_Register(1):
		offset = M_Offset_G1_HW_Save;
		break;
	case G_Save_Register(0):
		offset = M_Offset_G0_Save;
		break;
	case G_Save_Register(1):
		offset = M_Offset_G1_Save;
		break;
	default:
		rtdm_printk("%s: bug! unhandled register=0x%x in switch.\n",
			    __FUNCTION__, reg);
		BUG();
		return 0;
	}
	return ni_readl(offset);
}

static void win_out2(struct a4l_device *dev, uint32_t data, int reg)
{
	devpriv->stc_writew(dev, data >> 16, reg);
	devpriv->stc_writew(dev, data & 0xffff, reg + 1);
}

static uint32_t win_in2(struct a4l_device *dev, int reg)
{
	uint32_t bits;
	bits = devpriv->stc_readw(dev, reg) << 16;
	bits |= devpriv->stc_readw(dev, reg + 1);
	return bits;
}

static void m_series_init_eeprom_buffer(struct a4l_device *dev)
{
	static const int Start_Cal_EEPROM = 0x400;
	static const unsigned window_size = 10;
	unsigned old_iodwbsr_bits;
	unsigned old_iodwbsr1_bits;
	unsigned old_iodwcr1_bits;
	int i;

	old_iodwbsr_bits = readl(devpriv->mite->mite_io_addr + MITE_IODWBSR);
	old_iodwbsr1_bits = readl(devpriv->mite->mite_io_addr + MITE_IODWBSR_1);
	old_iodwcr1_bits = readl(devpriv->mite->mite_io_addr + MITE_IODWCR_1);
	writel(0x0, devpriv->mite->mite_io_addr + MITE_IODWBSR);
	writel(((0x80 | window_size) | devpriv->mite->daq_phys_addr),
	       devpriv->mite->mite_io_addr + MITE_IODWBSR_1);
	writel(0x0, devpriv->mite->mite_io_addr + MITE_IODWCR_1);
	writel(0xf, devpriv->mite->mite_io_addr + 0x30);

	for(i = 0; i < M_SERIES_EEPROM_SIZE; ++i)
	{
		devpriv->eeprom_buffer[i] = ni_readb(Start_Cal_EEPROM + i);
	}

	writel(old_iodwbsr1_bits, devpriv->mite->mite_io_addr + MITE_IODWBSR_1);
	writel(old_iodwbsr_bits, devpriv->mite->mite_io_addr + MITE_IODWBSR);
	writel(old_iodwcr1_bits, devpriv->mite->mite_io_addr + MITE_IODWCR_1);
	writel(0x0, devpriv->mite->mite_io_addr + 0x30);
}

static void init_6143(struct a4l_device *dev)
{
	/* Disable interrupts */
	devpriv->stc_writew(dev, 0, Interrupt_Control_Register);

	/* Initialise 6143 AI specific bits */

	/* Set G0,G1 DMA mode to E series version */
	ni_writeb(0x00, Magic_6143);
	/* Set EOCMode, ADCMode and pipelinedelay */
	ni_writeb(0x80, PipelineDelay_6143);
	/* Set EOC Delay */
	ni_writeb(0x00, EOC_Set_6143);

	/* Set the FIFO half full level */
	ni_writel(boardtype.ai_fifo_depth / 2, AIFIFO_Flag_6143);

	/* Strobe Relay disable bit */
	devpriv->ai_calib_source_enabled = 0;
	ni_writew(devpriv->ai_calib_source | Calibration_Channel_6143_RelayOff,
		  Calibration_Channel_6143);
	ni_writew(devpriv->ai_calib_source, Calibration_Channel_6143);
}

static int pcimio_attach(struct a4l_device *dev, a4l_lnkdesc_t *arg)
{
	int ret, bus, slot, i, irq;
	struct mite_struct *mite = NULL;
	struct ni_board_struct *board = NULL;

	if(arg->opts == NULL || arg->opts_size == 0)
		bus = slot = 0;
	else {
		bus = arg->opts_size >= sizeof(unsigned long) ?
			((unsigned long *)arg->opts)[0] : 0;
		slot = arg->opts_size >= sizeof(unsigned long) * 2 ?
			((unsigned long *)arg->opts)[1] : 0;
	}

	for(i = 0; i < n_pcimio_boards && mite == NULL; i++) {
		mite = a4l_mite_find_device(bus, slot, ni_boards[i].device_id);
		board = &ni_boards[i];
	}

	if(mite == 0)
		return -ENOENT;

	devpriv->irq_polarity = PCIMIO_IRQ_POLARITY;
	devpriv->irq_pin = 0;

	devpriv->mite = mite;
	devpriv->board_ptr = board;

	devpriv->ai_mite_ring = mite_alloc_ring(mite);
	devpriv->ao_mite_ring = mite_alloc_ring(mite);
	devpriv->cdo_mite_ring = mite_alloc_ring(mite);
	devpriv->gpct_mite_ring[0] = mite_alloc_ring(mite);
	devpriv->gpct_mite_ring[1] = mite_alloc_ring(mite);

	if(devpriv->ai_mite_ring == NULL ||
	   devpriv->ao_mite_ring == NULL ||
	   devpriv->cdo_mite_ring == NULL ||
	   devpriv->gpct_mite_ring[0] == NULL ||
	   devpriv->gpct_mite_ring[1] == NULL)
		return -ENOMEM;

	a4l_info(dev, "found %s board\n", boardtype.name);

	if(boardtype.reg_type & ni_reg_m_series_mask)
	{
		devpriv->stc_writew = &m_series_stc_writew;
		devpriv->stc_readw = &m_series_stc_readw;
		devpriv->stc_writel = &m_series_stc_writel;
		devpriv->stc_readl = &m_series_stc_readl;
	}else
	{
		devpriv->stc_writew = &e_series_win_out;
		devpriv->stc_readw = &e_series_win_in;
		devpriv->stc_writel = &win_out2;
		devpriv->stc_readl = &win_in2;
	}

	ret = a4l_mite_setup(devpriv->mite, 0);
	if(ret < 0)
	{
		a4l_err(dev, "pcmio_attach: error setting up mite\n");
		return ret;
	}

	if(boardtype.reg_type & ni_reg_m_series_mask)
		m_series_init_eeprom_buffer(dev);
	if(boardtype.reg_type == ni_reg_6143)
		init_6143(dev);

	irq = mite_irq(devpriv->mite);

	if(irq == 0){
		a4l_warn(dev, "pcimio_attach: unknown irq (bad)\n\n");
	}else{
		a4l_info(dev, "found irq %u\n", irq);
		ret = a4l_request_irq(dev,
				      irq,
				      a4l_ni_E_interrupt, RTDM_IRQTYPE_SHARED, dev);
		if(ret < 0)
			a4l_err(dev, "pcimio_attach: irq not available\n");
	}

	ret = a4l_ni_E_init(dev);
	if(ret < 0)
		return ret;

	dev->driver->driver_name = devpriv->board_ptr->name;

	return ret;
}

static int pcimio_detach(struct a4l_device *dev)
{
	if(a4l_get_irq(dev)!=A4L_IRQ_UNUSED){
		a4l_free_irq(dev,a4l_get_irq(dev));
	}

	if(dev->priv != NULL && devpriv->mite != NULL)
	{
		mite_free_ring(devpriv->ai_mite_ring);
		mite_free_ring(devpriv->ao_mite_ring);
		mite_free_ring(devpriv->gpct_mite_ring[0]);
		mite_free_ring(devpriv->gpct_mite_ring[1]);
		a4l_mite_unsetup(devpriv->mite);
	}

	dev->driver->driver_name = NULL;

	return 0;
}

static struct a4l_driver pcimio_drv = {
	.owner = THIS_MODULE,
	.board_name = "analogy_ni_pcimio",
	.driver_name = NULL,
	.attach = pcimio_attach,
	.detach = pcimio_detach,
	.privdata_size = sizeof(ni_private),
};

static int __init pcimio_init(void)
{
	return a4l_register_drv(&pcimio_drv);
}

static void __exit pcimio_cleanup(void)
{
	a4l_unregister_drv(&pcimio_drv);
}

MODULE_DESCRIPTION("Analogy driver for NI PCI-MIO series cards");
MODULE_LICENSE("GPL");

module_init(pcimio_init);
module_exit(pcimio_cleanup);
