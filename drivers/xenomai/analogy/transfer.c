/*
 * Analogy for Linux, transfer related features
 *
 * Copyright (C) 1997-2000 David A. Schleef <ds@schleef.org>
 * Copyright (C) 2008 Alexis Berlemont <alexis.berlemont@free.fr>
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/module.h>
#include <linux/fs.h>
#include <asm/errno.h>
#include <rtdm/analogy/device.h>

#include "proc.h"

/* --- Initialization / cleanup / cancel functions --- */

int a4l_precleanup_transfer(struct a4l_device_context * cxt)
{
	struct a4l_device *dev;
	struct a4l_transfer *tsf;
	int i, err = 0;

	dev = a4l_get_dev(cxt);
	tsf = &dev->transfer;

	if (tsf == NULL) {
		__a4l_err("a4l_precleanup_transfer: "
			  "incoherent status, transfer block not reachable\n");
		return -ENODEV;
	}

	for (i = 0; i < tsf->nb_subd; i++) {
		unsigned long *status = &tsf->subds[i]->status;

		__a4l_dbg(1, core_dbg, "subd[%d]->status=0x%08lx\n", i, *status);

		if (test_and_set_bit(A4L_SUBD_BUSY, status)) {
			__a4l_err("a4l_precleanup_transfer: "
				  "device busy, acquisition occuring\n");
			err = -EBUSY;
			goto out_error;
		} else
			set_bit(A4L_SUBD_CLEAN, status);
	}

	return 0;

out_error:
	for (i = 0; i < tsf->nb_subd; i++) {
		unsigned long *status = &tsf->subds[i]->status;

		if (test_bit(A4L_SUBD_CLEAN, status)){
			clear_bit(A4L_SUBD_BUSY, status);
			clear_bit(A4L_SUBD_CLEAN, status);
		}
	}

	return err;
}

int a4l_cleanup_transfer(struct a4l_device_context * cxt)
{
	struct a4l_device *dev;
	struct a4l_transfer *tsf;

	dev = a4l_get_dev(cxt);
	tsf = &dev->transfer;

	/* Releases the pointers tab, if need be */
	if (tsf->subds != NULL) {
		rtdm_free(tsf->subds);
	}

	memset(tsf, 0, sizeof(struct a4l_transfer));

	return 0;
}

void a4l_presetup_transfer(struct a4l_device_context *cxt)
{
	struct a4l_device *dev = NULL;
	struct a4l_transfer *tsf;

	dev = a4l_get_dev(cxt);
	tsf = &dev->transfer;

	/* Clear the structure */
	memset(tsf, 0, sizeof(struct a4l_transfer));

	tsf->default_bufsize = A4L_BUF_DEFSIZE;

	/* 0 is also considered as a valid IRQ, then
	   the IRQ number must be initialized with another value */
	tsf->irq_desc.irq = A4L_IRQ_UNUSED;
}

int a4l_setup_transfer(struct a4l_device_context * cxt)
{
	struct a4l_device *dev = NULL;
	struct a4l_transfer *tsf;
	struct list_head *this;
	int i = 0, ret = 0;

	dev = a4l_get_dev(cxt);
	tsf = &dev->transfer;

	/* Recovers the subdevices count
	   (as they are registered in a linked list */
	list_for_each(this, &dev->subdvsq) {
		tsf->nb_subd++;
	}

	__a4l_dbg(1, core_dbg, "nb_subd=%d\n", tsf->nb_subd);

	/* Allocates a suitable tab for the subdevices */
	tsf->subds = rtdm_malloc(tsf->nb_subd * sizeof(struct a4l_subdevice *));
	if (tsf->subds == NULL) {
		__a4l_err("a4l_setup_transfer: call1(alloc) failed \n");
		ret = -ENOMEM;
		goto out_setup_tsf;
	}

	/* Recovers the subdevices pointers */
	list_for_each(this, &dev->subdvsq) {
		tsf->subds[i++] = list_entry(this, struct a4l_subdevice, list);
	}

out_setup_tsf:

	if (ret != 0)
		a4l_cleanup_transfer(cxt);

	return ret;
}

/* --- IRQ handling section --- */

int a4l_request_irq(struct a4l_device * dev,
		    unsigned int irq,
		    a4l_irq_hdlr_t handler,
		    unsigned long flags, void *cookie)
{
	int ret;

	if (dev->transfer.irq_desc.irq != A4L_IRQ_UNUSED)
		return -EBUSY;

	ret = __a4l_request_irq(&dev->transfer.irq_desc, irq, handler, flags,
		cookie);
	if (ret != 0) {
		__a4l_err("a4l_request_irq: IRQ registration failed\n");
		dev->transfer.irq_desc.irq = A4L_IRQ_UNUSED;
	}

	return ret;
}

int a4l_free_irq(struct a4l_device * dev, unsigned int irq)
{

	int ret = 0;

	if (dev->transfer.irq_desc.irq != irq)
		return -EINVAL;

	/* There is less need to use a spinlock
	   than for a4l_request_irq() */
	ret = __a4l_free_irq(&dev->transfer.irq_desc);

	if (ret == 0)
		dev->transfer.irq_desc.irq = A4L_IRQ_UNUSED;

	return ret;
}

unsigned int a4l_get_irq(struct a4l_device * dev)
{
	return dev->transfer.irq_desc.irq;
}

/* --- Proc section --- */

#ifdef CONFIG_PROC_FS

int a4l_rdproc_transfer(struct seq_file *seq, void *v)
{
	struct a4l_transfer *transfer = (struct a4l_transfer *) seq->private;
	int i;

	if (v != SEQ_START_TOKEN)
		return -EINVAL;

	seq_printf(seq, "--  Subdevices --\n\n");
	seq_printf(seq, "| idx | type\n");

	/* Gives the subdevice type's name */
	for (i = 0; i < transfer->nb_subd; i++) {
		char *type;
		switch (transfer->subds[i]->flags & A4L_SUBD_TYPES) {
		case A4L_SUBD_UNUSED:
			type = "Unused subdevice";
			break;
		case A4L_SUBD_AI:
			type = "Analog input subdevice";
			break;
		case A4L_SUBD_AO:
			type = "Analog output subdevice";
			break;
		case A4L_SUBD_DI:
			type = "Digital input subdevice";
			break;
		case A4L_SUBD_DO:
			type = "Digital output subdevice";
			break;
		case A4L_SUBD_DIO:
			type = "Digital input/output subdevice";
			break;
		case A4L_SUBD_COUNTER:
			type = "Counter subdevice";
			break;
		case A4L_SUBD_TIMER:
			type = "Timer subdevice";
			break;
		case A4L_SUBD_MEMORY:
			type = "Memory subdevice";
			break;
		case A4L_SUBD_CALIB:
			type = "Calibration subdevice";
			break;
		case A4L_SUBD_PROC:
			type = "Processor subdevice";
			break;
		case A4L_SUBD_SERIAL:
			type = "Serial subdevice";
			break;
		default:
			type = "Unknown subdevice";
		}

		seq_printf(seq, "|  %02d | %s\n", i, type);
	}

	return 0;
}

#endif /* CONFIG_PROC_FS */
