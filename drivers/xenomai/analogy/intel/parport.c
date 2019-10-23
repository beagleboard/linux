/*
 * Analogy driver for standard parallel port
 * Copyright (C) 1998,2001 David A. Schleef <ds@schleef.org>
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
   A cheap and easy way to get a few more digital I/O lines.  Steal
   additional parallel ports from old computers or your neighbors'
   computers.

   Attach options list:
   0: I/O port base for the parallel port.
   1: IRQ

   Parallel Port Lines:

   pin     subdev  chan    aka
   ---     ------  ----    ---
   1       2       0       strobe
   2       0       0       data 0
   3       0       1       data 1
   4       0       2       data 2
   5       0       3       data 3
   6       0       4       data 4
   7       0       5       data 5
   8       0       6       data 6
   9       0       7       data 7
   10      1       3       acknowledge
   11      1       4       busy
   12      1       2       output
   13      1       1       printer selected
   14      2       1       auto LF
   15      1       0       error
   16      2       2       init
   17      2       3       select printer
   18-25   ground

   Notes:

   Subdevices 0 is digital I/O, subdevice 1 is digital input, and
   subdevice 2 is digital output.  Unlike other Analogy devices,
   subdevice 0 defaults to output.

   Pins 13 and 14 are inverted once by Analogy and once by the
   hardware, thus cancelling the effect.

   Pin 1 is a strobe, thus acts like one.  There's no way in software
   to change this, at least on a standard parallel port.

   Subdevice 3 pretends to be a digital input subdevice, but it always
   returns 0 when read.  However, if you run a command with
   scan_begin_src=TRIG_EXT, it uses pin 10 as a external triggering
   pin, which can be used to wake up tasks.

   see http://www.beyondlogic.org/ for information.
   or http://www.linux-magazin.de/ausgabe/1999/10/IO/io.html
*/

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/io.h>		/* For inb/outb */
#include <rtdm/analogy/device.h>

#define PARPORT_SIZE 3

#define PARPORT_A 0
#define PARPORT_B 1
#define PARPORT_C 2

#define DEFAULT_ADDRESS 0x378
#define DEFAULT_IRQ 7

typedef struct parport_subd_priv {
	unsigned long io_bits;
} parport_spriv_t;

typedef struct parport_priv {
	unsigned long io_base;
	unsigned int a_data;
	unsigned int c_data;
	int enable_irq;
} parport_priv_t;

#define devpriv ((parport_priv_t *)(dev->priv))

static int parport_insn_a(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint8_t *data = (uint8_t *)insn->data;

	if (data[0]) {
		devpriv->a_data &= ~data[0];
		devpriv->a_data |= (data[0] & data[1]);

		outb(devpriv->a_data, devpriv->io_base + PARPORT_A);
	}

	data[1] = inb(devpriv->io_base + PARPORT_A);

	return 0;
}

static int parport_insn_config_a(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	parport_spriv_t *spriv = (parport_spriv_t *)subd->priv;
	unsigned int *data = (unsigned int *)insn->data;

	/* No need to check the channel descriptor; the input / output
	   setting is global for all channels */

	switch (data[0]) {

	case A4L_INSN_CONFIG_DIO_OUTPUT:
		spriv->io_bits = 0xff;
		devpriv->c_data &= ~(1 << 5);
		break;

	case A4L_INSN_CONFIG_DIO_INPUT:
		spriv->io_bits = 0;
		devpriv->c_data |= (1 << 5);
		break;

	case A4L_INSN_CONFIG_DIO_QUERY:
		data[1] = (spriv->io_bits == 0xff) ?
			A4L_OUTPUT: A4L_INPUT;
		break;

	default:
		return -EINVAL;
	}

	outb(devpriv->c_data, devpriv->io_base + PARPORT_C);

	return 0;
}

static int parport_insn_b(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint8_t *data = (uint8_t *)insn->data;

	if (data[0]) {
		/* should writes be ignored? */
	}

	data[1] = (inb(devpriv->io_base + PARPORT_B) >> 3);

	return 0;
}

static int parport_insn_c(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint8_t *data = (uint8_t *)insn->data;

	data[0] &= 0x0f;
	if (data[0]) {
		devpriv->c_data &= ~data[0];
		devpriv->c_data |= (data[0] & data[1]);

		outb(devpriv->c_data, devpriv->io_base + PARPORT_C);
	}

	data[1] = devpriv->c_data & 0xf;

	return 2;
}

static int parport_intr_insn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	uint8_t *data = (uint8_t *)insn->data;

	if (insn->data_size < sizeof(uint8_t))
		return -EINVAL;

	data[1] = 0;
	return 0;
}

static struct a4l_cmd_desc parport_intr_cmd_mask = {
	.idx_subd = 0,
	.start_src = TRIG_NOW,
	.scan_begin_src = TRIG_EXT,
	.convert_src = TRIG_FOLLOW,
	.scan_end_src = TRIG_COUNT,
	.stop_src = TRIG_NONE,
};

static int parport_intr_cmdtest(struct a4l_subdevice *subd, struct a4l_cmd_desc * cmd)
{

	if (cmd->start_arg != 0) {
		return -EINVAL;
	}
	if (cmd->scan_begin_arg != 0) {
		return -EINVAL;
	}
	if (cmd->convert_arg != 0) {
		return -EINVAL;
	}
	if (cmd->scan_end_arg != 1) {
		return -EINVAL;
	}
	if (cmd->stop_arg != 0) {
		return -EINVAL;
	}

	return 0;
}

static int parport_intr_cmd(struct a4l_subdevice *subd, struct a4l_cmd_desc *cmd)
{
	struct a4l_device *dev = subd->dev;

	devpriv->c_data |= 0x10;
	outb(devpriv->c_data, devpriv->io_base + PARPORT_C);

	devpriv->enable_irq = 1;

	return 0;
}

static void parport_intr_cancel(struct a4l_subdevice *subd)
{
	struct a4l_device *dev = subd->dev;

	a4l_info(dev, "cancel in progress\n");

	devpriv->c_data &= ~0x10;
	outb(devpriv->c_data, devpriv->io_base + PARPORT_C);

	devpriv->enable_irq = 0;
}

static int parport_interrupt(unsigned int irq, void *d)
{
	struct a4l_device *dev = d;
	struct a4l_subdevice *subd = a4l_get_subd(dev, 3);

	if (!devpriv->enable_irq) {
		a4l_err(dev, "parport_interrupt: bogus irq, ignored\n");
		return IRQ_NONE;
	}

	a4l_buf_put(subd, 0, sizeof(unsigned int));
	a4l_buf_evt(subd, 0);

	return 0;
}


/* --- Channels descriptor --- */

static struct a4l_channels_desc parport_chan_desc_a = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = 8,
	.chans = {
		{A4L_CHAN_AREF_GROUND, 1},
	},
};

static struct a4l_channels_desc parport_chan_desc_b = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = 5,
	.chans = {
		{A4L_CHAN_AREF_GROUND, 1},
	},
};

static struct a4l_channels_desc parport_chan_desc_c = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = 4,
	.chans = {
		{A4L_CHAN_AREF_GROUND, 1},
	},
};

static struct a4l_channels_desc parport_chan_desc_intr = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = 1,
	.chans = {
		{A4L_CHAN_AREF_GROUND, 1},
	},
};

/* --- Subdevice initialization functions --- */

static void setup_subd_a(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_DIO;
	subd->chan_desc = &parport_chan_desc_a;
	subd->rng_desc = &range_digital;
	subd->insn_bits = parport_insn_a;
	subd->insn_config = parport_insn_config_a;
}

static void setup_subd_b(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_DI;
	subd->chan_desc = &parport_chan_desc_b;
	subd->rng_desc = &range_digital;
	subd->insn_bits = parport_insn_b;
}

static void setup_subd_c(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_DO;
	subd->chan_desc = &parport_chan_desc_c;
	subd->rng_desc = &range_digital;
	subd->insn_bits = parport_insn_c;
}

static void setup_subd_intr(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_DI;
	subd->chan_desc = &parport_chan_desc_intr;
	subd->rng_desc = &range_digital;
	subd->insn_bits = parport_intr_insn;
	subd->cmd_mask = &parport_intr_cmd_mask;
	subd->do_cmdtest = parport_intr_cmdtest;
	subd->do_cmd = parport_intr_cmd;
	subd->cancel = parport_intr_cancel;
}

static void (*setup_subds[3])(struct a4l_subdevice *) = {
	setup_subd_a,
	setup_subd_b,
	setup_subd_c
};

static int dev_parport_attach(struct a4l_device *dev, a4l_lnkdesc_t *arg)
{
	int i, err = 0, irq = A4L_IRQ_UNUSED;
	unsigned long io_base;

	if(arg->opts == NULL || arg->opts_size < sizeof(unsigned long)) {

		a4l_warn(dev,
			 "dev_parport_attach: no attach options specified, "
			 "taking default options (addr=0x%x, irq=%d)\n",
			 DEFAULT_ADDRESS, DEFAULT_IRQ);

		io_base = DEFAULT_ADDRESS;
		irq = DEFAULT_IRQ;
	} else {

		io_base = ((unsigned long *)arg->opts)[0];

		if (arg->opts_size >= 2 * sizeof(unsigned long))
			irq = (int) ((unsigned long *)arg->opts)[1];
	}

	if (!request_region(io_base, PARPORT_SIZE, "analogy_parport")) {
		a4l_err(dev, "dev_parport_attach: I/O port conflict");
		return -EIO;
	}

	a4l_info(dev, "address = 0x%lx\n", io_base);

	for (i = 0; i < 3; i++) {

		struct a4l_subdevice *subd = a4l_alloc_subd(sizeof(parport_spriv_t),
						  setup_subds[i]);
		if (subd == NULL)
			return -ENOMEM;

		err = a4l_add_subd(dev, subd);
		if (err != i)
			return err;
	}

	if (irq != A4L_IRQ_UNUSED) {

		struct a4l_subdevice *subd;

		a4l_info(dev, "irq = %d\n", irq);

		err = a4l_request_irq(dev, irq, parport_interrupt, 0, dev);
		if (err < 0) {
			a4l_err(dev, "dev_parport_attach: irq not available\n");
			return err;
		}

		subd = a4l_alloc_subd(0, setup_subd_intr);
		if (subd == NULL)
			return -ENOMEM;

		err = a4l_add_subd(dev, subd);
		if (err < 0)
			return err;
	}

	devpriv->io_base = io_base;

	devpriv->a_data = 0;
	outb(devpriv->a_data, devpriv->io_base + PARPORT_A);

	devpriv->c_data = 0;
	outb(devpriv->c_data, devpriv->io_base + PARPORT_C);

	return 0;
}

static int dev_parport_detach(struct a4l_device *dev)
{
	int err = 0;

	if (devpriv->io_base != 0)
		release_region(devpriv->io_base, PARPORT_SIZE);

	if (a4l_get_irq(dev) != A4L_IRQ_UNUSED) {
		a4l_free_irq(dev, a4l_get_irq(dev));
	}


	return err;
}

static struct a4l_driver drv_parport = {
	.owner = THIS_MODULE,
	.board_name = "analogy_parport",
	.driver_name = "parport",
	.attach = dev_parport_attach,
	.detach = dev_parport_detach,
	.privdata_size = sizeof(parport_priv_t),
};

static int __init drv_parport_init(void)
{
	return a4l_register_drv(&drv_parport);
}

static void __exit drv_parport_cleanup(void)
{
	a4l_unregister_drv(&drv_parport);
}

MODULE_DESCRIPTION("Analogy driver for standard parallel port");
MODULE_LICENSE("GPL");

module_init(drv_parport_init);
module_exit(drv_parport_cleanup);
