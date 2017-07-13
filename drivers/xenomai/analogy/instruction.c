/*
 * Analogy for Linux, instruction related features
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
#include <linux/version.h>
#include <linux/ioport.h>
#include <linux/mman.h>
#include <asm/div64.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <rtdm/analogy/device.h>

int a4l_do_insn_gettime(struct a4l_kernel_instruction * dsc)
{
	nanosecs_abs_t ns;
	uint32_t ns2;

	unsigned int *data = (unsigned int *)dsc->data;

	/* Basic checkings */
	if (dsc->data_size != 2 * sizeof(unsigned int)) {
		__a4l_err("a4l_do_insn_gettime: data size should be 2\n");
		return -EINVAL;
	}

	/* Get a timestamp */
	ns = a4l_get_time();

	/* Perform the conversion */
	ns2 = do_div(ns, 1000000000);
	data[0] = (unsigned int) ns;
	data[1] = (unsigned int) ns2 / 1000;

	return 0;
}

int a4l_do_insn_wait(struct a4l_kernel_instruction * dsc)
{
	unsigned int us;
	unsigned int *data = (unsigned int *)dsc->data;

	/* Basic checkings */
	if (dsc->data_size != sizeof(unsigned int)) {
		__a4l_err("a4l_do_insn_wait: data size should be 1\n");
		return -EINVAL;
	}

	if (data[0] > A4L_INSN_WAIT_MAX) {
		__a4l_err("a4l_do_insn_wait: wait duration is out of range\n");
		return -EINVAL;
	}

	/* As we use (a4l_)udelay, we have to convert the delay into
	   microseconds */
	us = data[0] / 1000;

	/* At least, the delay is rounded up to 1 microsecond */
	if (us == 0)
		us = 1;

	/* Performs the busy waiting */
	a4l_udelay(us);

	return 0;
}

int a4l_do_insn_trig(struct a4l_device_context * cxt, struct a4l_kernel_instruction * dsc)
{
	struct a4l_subdevice *subd;
	struct a4l_device *dev = a4l_get_dev(cxt);
	unsigned int trignum;
	unsigned int *data = (unsigned int*)dsc->data;

	/* Basic checkings */
	if (dsc->data_size > 1) {
		__a4l_err("a4l_do_insn_trig: data size should not be > 1\n");
		return -EINVAL;
	}

	trignum = (dsc->data_size == sizeof(unsigned int)) ? data[0] : 0;

	if (dsc->idx_subd >= dev->transfer.nb_subd) {
		__a4l_err("a4l_do_insn_trig: "
			  "subdevice index is out of range\n");
		return -EINVAL;
	}

	subd = dev->transfer.subds[dsc->idx_subd];

	/* Checks that the concerned subdevice is trigger-compliant */
	if ((subd->flags & A4L_SUBD_CMD) == 0 || subd->trigger == NULL) {
		__a4l_err("a4l_do_insn_trig: subdevice does not support "
			  "triggering or asynchronous acquisition\n");
		return -EINVAL;
	}

	/* Performs the trigger */
	return subd->trigger(subd, trignum);
}

int a4l_fill_insndsc(struct a4l_device_context * cxt, struct a4l_kernel_instruction * dsc, void *arg)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	int ret = 0;
	void *tmp_data = NULL;

	ret = rtdm_safe_copy_from_user(fd,
				       dsc, arg, sizeof(a4l_insn_t));
	if (ret != 0)
		goto out_insndsc;

	if (dsc->data_size != 0 && dsc->data == NULL) {
		__a4l_err("a4l_fill_insndsc: no data pointer specified\n");
		ret = -EINVAL;
		goto out_insndsc;
	}

	if (dsc->data_size != 0 && dsc->data != NULL) {
		tmp_data = rtdm_malloc(dsc->data_size);
		if (tmp_data == NULL) {
			ret = -ENOMEM;
			goto out_insndsc;
		}

		if ((dsc->type & A4L_INSN_MASK_WRITE) != 0) {
			ret = rtdm_safe_copy_from_user(fd,
						       tmp_data, dsc->data,
						       dsc->data_size);
			if (ret < 0)
				goto out_insndsc;
		}
	}

	dsc->__udata = dsc->data;
	dsc->data = tmp_data;

out_insndsc:

	if (ret != 0 && tmp_data != NULL)
		rtdm_free(tmp_data);

	return ret;
}

int a4l_free_insndsc(struct a4l_device_context * cxt, struct a4l_kernel_instruction * dsc)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	int ret = 0;

	if ((dsc->type & A4L_INSN_MASK_READ) != 0)
		ret = rtdm_safe_copy_to_user(fd,
					     dsc->__udata,
					     dsc->data, dsc->data_size);

	if (dsc->data != NULL)
		rtdm_free(dsc->data);

	return ret;
}

int a4l_do_special_insn(struct a4l_device_context * cxt, struct a4l_kernel_instruction * dsc)
{
	int ret = 0;

	switch (dsc->type) {
	case A4L_INSN_GTOD:
		ret = a4l_do_insn_gettime(dsc);
		break;
	case A4L_INSN_WAIT:
		ret = a4l_do_insn_wait(dsc);
		break;
	case A4L_INSN_INTTRIG:
		ret = a4l_do_insn_trig(cxt, dsc);
		break;
	default:
		__a4l_err("a4l_do_special_insn: "
			  "incoherent instruction code\n");
		return -EINVAL;
	}

	if (ret < 0)
		__a4l_err("a4l_do_special_insn: "
			  "execution of the instruction failed (err=%d)\n",
			  ret);

	return ret;
}

int a4l_do_insn(struct a4l_device_context * cxt, struct a4l_kernel_instruction * dsc)
{
	int ret = 0;
	struct a4l_subdevice *subd;
	struct a4l_device *dev = a4l_get_dev(cxt);
	int (*hdlr) (struct a4l_subdevice *, struct a4l_kernel_instruction *) = NULL;

	/* Checks the subdevice index */
	if (dsc->idx_subd >= dev->transfer.nb_subd) {
		__a4l_err("a4l_do_insn: "
			  "subdevice index out of range (idx=%d)\n",
			  dsc->idx_subd);
		return -EINVAL;
	}

	/* Recovers pointers on the proper subdevice */
	subd = dev->transfer.subds[dsc->idx_subd];

	/* Checks the subdevice's characteristics */
	if ((subd->flags & A4L_SUBD_TYPES) == A4L_SUBD_UNUSED) {
		__a4l_err("a4l_do_insn: wrong subdevice selected\n");
		return -EINVAL;
	}

	/* Checks the channel descriptor */
	if ((subd->flags & A4L_SUBD_TYPES) != A4L_SUBD_CALIB) {
		ret = a4l_check_chanlist(dev->transfer.subds[dsc->idx_subd],
					 1, &dsc->chan_desc);
		if (ret < 0)
			return ret;
	}

	/* Choose the proper handler, we can check the pointer because
	   the subdevice was memset to 0 at allocation time */
	switch (dsc->type) {
	case A4L_INSN_READ:
		hdlr = subd->insn_read;
		break;
	case A4L_INSN_WRITE:
		hdlr = subd->insn_write;
		break;
	case A4L_INSN_BITS:
		hdlr = subd->insn_bits;
		break;
	case A4L_INSN_CONFIG:
		hdlr = subd->insn_config;
		break;
	default:
		ret = -EINVAL;
	}

	/* We check the instruction type */
	if (ret < 0)
		return ret;

	/* We check whether a handler is available */
	if (hdlr == NULL)
		return -ENOSYS;

	/* Prevents the subdevice from being used during
	   the following operations */
	if (test_and_set_bit(A4L_SUBD_BUSY_NR, &subd->status)) {
		ret = -EBUSY;
		goto out_do_insn;
	}

	/* Let's the driver-specific code perform the instruction */
	ret = hdlr(subd, dsc);

	if (ret < 0)
		__a4l_err("a4l_do_insn: "
			  "execution of the instruction failed (err=%d)\n",
			  ret);

out_do_insn:

	/* Releases the subdevice from its reserved state */
	clear_bit(A4L_SUBD_BUSY_NR, &subd->status);

	return ret;
}

int a4l_ioctl_insn(struct a4l_device_context * cxt, void *arg)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	int ret = 0;
	struct a4l_kernel_instruction insn;
	struct a4l_device *dev = a4l_get_dev(cxt);

	if (!rtdm_in_rt_context() && rtdm_rt_capable(fd))
		return -ENOSYS;

	/* Basic checking */
	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_ioctl_insn: unattached device\n");
		return -EINVAL;
	}

	/* Recovers the instruction descriptor */
	ret = a4l_fill_insndsc(cxt, &insn, arg);
	if (ret != 0)
		goto err_ioctl_insn;

	/* Performs the instruction */
	if ((insn.type & A4L_INSN_MASK_SPECIAL) != 0)
		ret = a4l_do_special_insn(cxt, &insn);
	else
		ret = a4l_do_insn(cxt, &insn);

	if (ret < 0)
		goto err_ioctl_insn;

	/* Frees the used memory and sends back some
	   data, if need be */
	ret = a4l_free_insndsc(cxt, &insn);

	return ret;

err_ioctl_insn:
	a4l_free_insndsc(cxt, &insn);
	return ret;
}

int a4l_fill_ilstdsc(struct a4l_device_context * cxt, struct a4l_kernel_instruction_list * dsc, void *arg)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	int i, ret = 0;

	dsc->insns = NULL;

	/* Recovers the structure from user space */
	ret = rtdm_safe_copy_from_user(fd,
				       dsc, arg, sizeof(a4l_insnlst_t));
	if (ret < 0)
		return ret;

	/* Some basic checking */
	if (dsc->count == 0) {
		__a4l_err("a4l_fill_ilstdsc: instruction list's count is 0\n");
		return -EINVAL;
	}

	/* Keeps the user pointer in an opaque field */
	dsc->__uinsns = (a4l_insn_t *)dsc->insns;

	dsc->insns = rtdm_malloc(dsc->count * sizeof(struct a4l_kernel_instruction));
	if (dsc->insns == NULL)
		return -ENOMEM;

	/* Recovers the instructions, one by one. This part is not
	   optimized */
	for (i = 0; i < dsc->count && ret == 0; i++)
		ret = a4l_fill_insndsc(cxt,
				       &(dsc->insns[i]),
				       &(dsc->__uinsns[i]));

	/* In case of error, frees the allocated memory */
	if (ret < 0 && dsc->insns != NULL)
		rtdm_free(dsc->insns);

	return ret;
}

int a4l_free_ilstdsc(struct a4l_device_context * cxt, struct a4l_kernel_instruction_list * dsc)
{
	int i, ret = 0;

	if (dsc->insns != NULL) {

		for (i = 0; i < dsc->count && ret == 0; i++)
			ret = a4l_free_insndsc(cxt, &(dsc->insns[i]));

		while (i < dsc->count) {
			a4l_free_insndsc(cxt, &(dsc->insns[i]));
			i++;
		}

		rtdm_free(dsc->insns);
	}

	return ret;
}

/* This function is not optimized in terms of memory footprint and
   CPU charge; however, the whole analogy instruction system was not
   designed for performance issues */
int a4l_ioctl_insnlist(struct a4l_device_context * cxt, void *arg)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	int i, ret = 0;
	struct a4l_kernel_instruction_list ilst;
	struct a4l_device *dev = a4l_get_dev(cxt);

	if (!rtdm_in_rt_context() && rtdm_rt_capable(fd))
		return -ENOSYS;

	/* Basic checking */
	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_ioctl_insnlist: unattached device\n");
		return -EINVAL;
	}

	if ((ret = a4l_fill_ilstdsc(cxt, &ilst, arg)) < 0)
		return ret;

	/* Performs the instructions */
	for (i = 0; i < ilst.count && ret == 0; i++) {
		if ((ilst.insns[i].type & A4L_INSN_MASK_SPECIAL) != 0)
			ret = a4l_do_special_insn(cxt, &ilst.insns[i]);
		else
			ret = a4l_do_insn(cxt, &ilst.insns[i]);
	}

	if (ret < 0)
		goto err_ioctl_ilst;

	return a4l_free_ilstdsc(cxt, &ilst);

err_ioctl_ilst:
	a4l_free_ilstdsc(cxt, &ilst);
	return ret;
}
