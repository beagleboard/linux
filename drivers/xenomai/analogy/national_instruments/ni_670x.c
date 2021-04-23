/*
    comedi/drivers/ni_670x.c
    Hardware driver for NI 670x devices

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2001 David A. Schleef <ds@schleef.org>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

*/
/*
Driver: ni_670x
Description: National Instruments 670x
Author: Bart Joris <bjoris@advalvas.be>
Updated: Wed, 11 Dec 2002 18:25:35 -0800
Devices: [National Instruments] PCI-6703 (ni_670x), PCI-6704
Status: unknown

Commands are not supported.
*/

/*
	Bart Joris <bjoris@advalvas.be> Last updated on 20/08/2001

	Manuals:

	322110a.pdf	PCI/PXI-6704 User Manual
	322110b.pdf	PCI/PXI-6703/6704 User Manual
*/

/*
 * Integration with Xenomai/Analogy layer based on the
 * comedi driver. Adaptation made by
 *   Julien Delange <julien.delange@esa.int>
 */

#include <linux/interrupt.h>
#include <linux/slab.h>
#include <rtdm/analogy/device.h>

#include "../intel/8255.h"
#include "ni_mio.h"
#include "mite.h"

#define PCIMIO_IRQ_POLARITY 1

#define  AO_VALUE_OFFSET         0x00
#define  AO_CHAN_OFFSET          0x0c
#define  AO_STATUS_OFFSET        0x10
#define  AO_CONTROL_OFFSET       0x10
#define  DIO_PORT0_DIR_OFFSET    0x20
#define  DIO_PORT0_DATA_OFFSET   0x24
#define  DIO_PORT1_DIR_OFFSET    0x28
#define  DIO_PORT1_DATA_OFFSET   0x2c
#define  MISC_STATUS_OFFSET      0x14
#define  MISC_CONTROL_OFFSET     0x14

/* Board description*/

struct ni_670x_board {
	unsigned short device_id;
	const char *name;
	unsigned short ao_chans;
	unsigned short ao_bits;
};

#define thisboard ((struct ni_670x_board *)dev->board_ptr)

struct ni_670x_private {
	struct mite_struct *mite;
	int boardtype;
	int dio;
	unsigned int ao_readback[32];

	/*
	 * Added when porting to xenomai
	 */
	int irq_polarity;
	int irq_pin;
	int irq;
	struct ni_670x_board *board_ptr;
	/*
	 * END OF ADDED when porting to xenomai
	 */
};

struct ni_670x_subd_priv {
	int io_bits;
	unsigned int state;
	uint16_t readback[2];
	uint16_t config;
	void* counter;
};

static int ni_670x_ao_winsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn);
static int ni_670x_ao_rinsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn);
static int ni_670x_dio_insn_bits(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn);
static int ni_670x_dio_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn);

static struct a4l_channels_desc ni_670x_desc_dio = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = 8,
	.chans = {
		{A4L_CHAN_AREF_GROUND, 1},
	},
};

static struct a4l_channels_desc ni_670x_desc_ao = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = 0, /* initialized later according to the board found */
	.chans = {
		{A4L_CHAN_AREF_GROUND, 16},
	},
};


static struct a4l_rngtab range_0_20mA = { 1, {RANGE_mA(0, 20)} };
static struct a4l_rngtab rng_bipolar10 = { 1, {RANGE_V(-10, 10) }};

struct a4l_rngtab *range_table_list[32] = {
	&rng_bipolar10, &rng_bipolar10, &rng_bipolar10, &rng_bipolar10,
	&rng_bipolar10, &rng_bipolar10, &rng_bipolar10, &rng_bipolar10,
	&rng_bipolar10, &rng_bipolar10, &rng_bipolar10, &rng_bipolar10,
	&rng_bipolar10, &rng_bipolar10, &rng_bipolar10, &rng_bipolar10,
	&range_0_20mA, &range_0_20mA, &range_0_20mA, &range_0_20mA,
	&range_0_20mA, &range_0_20mA, &range_0_20mA, &range_0_20mA,
	&range_0_20mA, &range_0_20mA, &range_0_20mA, &range_0_20mA,
	&range_0_20mA, &range_0_20mA, &range_0_20mA, &range_0_20mA};

static A4L_RNGDESC(32) ni670x_ao_desc;

static void setup_subd_ao(struct a4l_subdevice *subd)
{
	int i;
	int nchans;

	nchans = ((struct ni_670x_private*)(subd->dev->priv))->board_ptr->ao_chans;
	subd->flags                = A4L_SUBD_AO;
	subd->chan_desc            = &ni_670x_desc_ao;
	subd->chan_desc->length    = nchans;
	if (nchans == 32) {

		subd->rng_desc = (struct a4l_rngdesc*) &ni670x_ao_desc;
		subd->rng_desc->mode = A4L_RNG_PERCHAN_RNGDESC;
		for (i = 0 ; i < 16 ; i++) {
			subd->rng_desc->rngtabs[i] =&rng_bipolar10;
			subd->rng_desc->rngtabs[16+i] =&range_0_20mA;
		}
	} else
		subd->rng_desc = &a4l_range_bipolar10;

	subd->insn_write = &ni_670x_ao_winsn;
	subd->insn_read = &ni_670x_ao_rinsn;
}

static void setup_subd_dio(struct a4l_subdevice *s)
{
	/* Digital i/o subdevice */
	s->flags = A4L_SUBD_DIO;
	s->chan_desc = &ni_670x_desc_dio;
	s->rng_desc = &range_digital;
	s->insn_bits = ni_670x_dio_insn_bits;
	s->insn_config = ni_670x_dio_insn_config;
}

struct setup_subd {
	void (*setup_func) (struct a4l_subdevice *);
	int sizeof_priv;
};

static struct setup_subd setup_subds[2] = {
	{
		.setup_func = setup_subd_ao,
		.sizeof_priv = sizeof(struct ni_670x_subd_priv),
	},
	{
		.setup_func = setup_subd_dio,
		.sizeof_priv = sizeof(struct ni_670x_subd_priv),
	},
};

static const struct ni_670x_board ni_670x_boards[] = {
	{
		.device_id = 0x2c90,
		.name = "PCI-6703",
		.ao_chans = 16,
		.ao_bits = 16,
	},
	{
		.device_id = 0x1920,
		.name = "PXI-6704",
		.ao_chans = 32,
		.ao_bits = 16,
	},
	{
		.device_id = 0x1290,
		.name = "PCI-6704",
		.ao_chans = 32,
		.ao_bits = 16,
	 },
};

#define n_ni_670x_boards ((sizeof(ni_670x_boards)/sizeof(ni_670x_boards[0])))

static const struct pci_device_id ni_670x_pci_table[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_NI, 0x2c90)},
	{PCI_DEVICE(PCI_VENDOR_ID_NI, 0x1920)},
	{0}
};

MODULE_DEVICE_TABLE(pci, ni_670x_pci_table);

#define devpriv ((struct ni_670x_private *)dev->priv)

static inline struct ni_670x_private *private(struct a4l_device *dev)
{
	return (struct ni_670x_private*) dev->priv;
}


static int ni_670x_attach (struct a4l_device *dev, a4l_lnkdesc_t *arg);
static int ni_670x_detach(struct a4l_device *dev);

static struct a4l_driver ni_670x_drv = {
	.owner = THIS_MODULE,
	.board_name = "analogy_ni_670x",
	.driver_name = "ni_670x",
	.attach = ni_670x_attach,
	.detach = ni_670x_detach,
	.privdata_size = sizeof(struct ni_670x_private),
};

static int __init driver_ni_670x_init_module(void)
{
	return a4l_register_drv (&ni_670x_drv);
}

static void __exit driver_ni_670x_cleanup_module(void)
{
	a4l_unregister_drv (&ni_670x_drv);
}

module_init(driver_ni_670x_init_module);
module_exit(driver_ni_670x_cleanup_module);

static int ni_670x_attach (struct a4l_device *dev, a4l_lnkdesc_t *arg)
{
	int ret, bus, slot, i, irq;
	struct mite_struct *mite;
	struct ni_670x_board* board = NULL;
	int err;

	if(arg->opts == NULL || arg->opts_size == 0)
		bus = slot = 0;
	else {
		bus = arg->opts_size >= sizeof(unsigned long) ?
			((unsigned long *)arg->opts)[0] : 0;
		slot = arg->opts_size >= sizeof(unsigned long) * 2 ?
			((unsigned long *)arg->opts)[1] : 0;
	}

	a4l_info(dev, "ni670x attach procedure started(bus=%d/slot=%d)...\n",
		 bus, slot);

	mite = NULL;

	for(i = 0; i <  n_ni_670x_boards && mite == NULL; i++) {
		mite = a4l_mite_find_device(bus,
					    slot, ni_670x_boards[i].device_id);
		board = (struct ni_670x_board*) &ni_670x_boards[i];
	}

	if(mite == NULL) {
		a4l_err(dev, "%s: cannot find the MITE device\n", __FUNCTION__);
		return -ENOENT;
	}

	a4l_info(dev, "Found device %d %s\n", i, ni_670x_boards[i].name);

	devpriv->irq_polarity = PCIMIO_IRQ_POLARITY;
	devpriv->irq_pin = 0;

	devpriv->mite = mite;
	devpriv->board_ptr = board;

	ret = a4l_mite_setup(devpriv->mite, 0);
	if (ret < 0) {
		a4l_err(dev, "%s: error setting up mite\n", __FUNCTION__);
		return ret;
	}

	irq = mite_irq(devpriv->mite);
	devpriv->irq = irq;

	a4l_info(dev, "found %s board\n", board->name);

	for (i = 0; i < 2; i++) {
		struct a4l_subdevice *subd =
			a4l_alloc_subd(setup_subds[i].sizeof_priv, NULL);

		if (subd == NULL) {
			a4l_err(dev,
				"%s: cannot allocate subdevice\n",
				__FUNCTION__);
			return -ENOMEM;
		}

		err = a4l_add_subd(dev, subd);
		if (err != i) {
			a4l_err(dev,
				"%s: cannot add subdevice\n",
				__FUNCTION__);
			return err;
		}

		setup_subds[i].setup_func (subd);
	}

	/* Config of misc registers */
	writel(0x10, devpriv->mite->daq_io_addr + MISC_CONTROL_OFFSET);
	/* Config of ao registers */
	writel(0x00, devpriv->mite->daq_io_addr + AO_CONTROL_OFFSET);

	a4l_info(dev, "ni670x attached\n");

	return 0;
}

static int ni_670x_detach(struct a4l_device *dev)
{
	a4l_info(dev, "ni670x detach procedure started...\n");

	if(dev->priv != NULL && devpriv->mite != NULL)
		a4l_mite_unsetup(devpriv->mite);

	a4l_info(dev, "ni670x detach procedure succeeded...\n");

	return 0;
}


static int ni_670x_dio_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	unsigned int *data = (unsigned int *)insn->data;
	int chan = CR_CHAN(insn->chan_desc);
	struct ni_670x_subd_priv *subdpriv =
		(struct ni_670x_subd_priv *)subd->priv;

	switch (data[0]) {
	case A4L_INSN_CONFIG_DIO_OUTPUT:
		subdpriv->io_bits |= 1 << chan;
		break;
	case A4L_INSN_CONFIG_DIO_INPUT:
		subdpriv->io_bits &= ~(1 << chan);
		break;
	case A4L_INSN_CONFIG_DIO_QUERY:
		data[1] = (subdpriv->io_bits & (1 << chan)) ?
			A4L_OUTPUT : A4L_INPUT;
		return 0;
		break;
	default:
		return -EINVAL;
		break;
	}

	writel(subdpriv->io_bits,
	       devpriv->mite->daq_io_addr + DIO_PORT0_DIR_OFFSET);

	return 0;
}

static int ni_670x_ao_winsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	int i;
	unsigned int tmp;
	unsigned int* dtmp;
	int chan;
	dtmp = (unsigned int*)insn->data;
	chan = CR_CHAN(insn->chan_desc);

	/* Channel number mapping :

	   NI 6703/ NI 6704     | NI 6704 Only
	   ----------------------------------------------------
	   vch(0)       :       0       | ich(16)       :       1
	   vch(1)       :       2       | ich(17)       :       3
	   .    :       .       |   .                   .
	   .    :       .       |   .                   .
	   .    :       .       |   .                   .
	   vch(15)      :       30      | ich(31)       :       31 */

	for (i = 0; i < insn->data_size / sizeof(unsigned int); i++) {

		tmp = dtmp[i];

		/* First write in channel register which channel to use */
		writel(((chan & 15) << 1) | ((chan & 16) >> 4),
		       private (subd->dev)->mite->daq_io_addr + AO_CHAN_OFFSET);

		/* write channel value */
		writel(dtmp[i],
		       private(subd->dev)->mite->daq_io_addr + AO_VALUE_OFFSET);
		private(subd->dev)->ao_readback[chan] = tmp;
	}

   return 0;
}

static int ni_670x_ao_rinsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	int i;
	unsigned int* dtmp;
	int chan = CR_CHAN(insn->chan_desc);

	dtmp = (unsigned int*)insn->data;

	for (i = 0; i < insn->data_size / sizeof(unsigned int); i++)
		dtmp[i] = private(subd->dev)->ao_readback[chan];

	return 0;
}


static int ni_670x_dio_insn_bits(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	return -ENOSYS;
}

MODULE_DESCRIPTION("Analogy driver for NI670x series cards");
MODULE_LICENSE("GPL");
