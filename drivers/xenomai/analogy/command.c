/*
 * Analogy for Linux, command related features
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
#include <linux/ioport.h>
#include <linux/mman.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <rtdm/analogy/device.h>

/* --- Command descriptor management functions --- */
int a4l_fill_cmddesc(struct a4l_device_context *cxt, struct a4l_cmd_desc *desc,
		     unsigned int **chan_descs, void *arg)
{
	unsigned int *tmpchans = NULL;
	int ret = 0;

	ret = rtdm_safe_copy_from_user(rtdm_private_to_fd(cxt),
				       desc, arg, sizeof(struct a4l_cmd_desc));
	if (ret != 0)
		goto out_cmddesc;


	if (desc->nb_chan == 0) {
		ret = -EINVAL;
		goto out_cmddesc;
	}

	tmpchans = rtdm_malloc(desc->nb_chan * sizeof(unsigned int));
	if (tmpchans == NULL) {
		ret = -ENOMEM;
		goto out_cmddesc;
	}

	ret = rtdm_safe_copy_from_user(rtdm_private_to_fd(cxt),
				       tmpchans,
				       desc->chan_descs,
				       desc->nb_chan * sizeof(unsigned int));
	if (ret != 0) {
		__a4l_err("%s invalid arguments \n", __FUNCTION__);
		goto out_cmddesc;
	}

	*chan_descs = desc->chan_descs;
	desc->chan_descs = tmpchans;

	__a4l_dbg(1, core_dbg, "desc dump: \n");
	__a4l_dbg(1, core_dbg, "\t->idx_subd=%u\n", desc->idx_subd);
	__a4l_dbg(1, core_dbg, "\t->flags=%lu\n", desc->flags);
	__a4l_dbg(1, core_dbg, "\t->nb_chan=%u\n", desc->nb_chan);
	__a4l_dbg(1, core_dbg, "\t->chan_descs=0x%x\n", *desc->chan_descs);
	__a4l_dbg(1, core_dbg, "\t->data_len=%u\n", desc->data_len);
	__a4l_dbg(1, core_dbg, "\t->pdata=0x%p\n", desc->data);

	out_cmddesc:

	if (ret != 0) {
		__a4l_err("a4l_fill_cmddesc: %d \n", ret);
		if (tmpchans != NULL)
			rtdm_free(tmpchans);
		desc->chan_descs = NULL;
	}

	return ret;
}

void a4l_free_cmddesc(struct a4l_cmd_desc * desc)
{
	if (desc->chan_descs != NULL)
		rtdm_free(desc->chan_descs);
}

int a4l_check_cmddesc(struct a4l_device_context * cxt, struct a4l_cmd_desc * desc)
{
	struct a4l_device *dev = a4l_get_dev(cxt);
	struct a4l_subdevice *subd;

	if (desc->idx_subd >= dev->transfer.nb_subd) {
		__a4l_err("a4l_check_cmddesc: "
			  "subdevice index out of range (idx=%u)\n",
			  desc->idx_subd);
		return -EINVAL;
	}

	subd = dev->transfer.subds[desc->idx_subd];

	if ((subd->flags & A4L_SUBD_TYPES) == A4L_SUBD_UNUSED) {
		__a4l_err("a4l_check_cmddesc: "
			  "subdevice type incoherent\n");
		return -EIO;
	}

	if (!(subd->flags & A4L_SUBD_CMD)) {
		__a4l_err("a4l_check_cmddesc: operation not supported, "
			  "synchronous only subdevice\n");
		return -EIO;
	}

	if (test_bit(A4L_SUBD_BUSY, &subd->status)) {
		__a4l_err("a4l_check_cmddesc: subdevice busy\n");
		return -EBUSY;
	}

	return a4l_check_chanlist(dev->transfer.subds[desc->idx_subd],
				  desc->nb_chan, desc->chan_descs);
}

/* --- Command checking functions --- */

int a4l_check_generic_cmdcnt(struct a4l_cmd_desc * desc)
{
	unsigned int tmp1, tmp2;

	/* Makes sure trigger sources are trivially valid */
	tmp1 =
	desc->start_src & ~(TRIG_NOW | TRIG_INT | TRIG_EXT | TRIG_FOLLOW);
	tmp2 = desc->start_src & (TRIG_NOW | TRIG_INT | TRIG_EXT | TRIG_FOLLOW);
	if (tmp1 != 0 || tmp2 == 0) {
		__a4l_err("a4l_check_cmddesc: start_src, weird trigger\n");
		return -EINVAL;
	}

	tmp1 = desc->scan_begin_src & ~(TRIG_TIMER | TRIG_EXT | TRIG_FOLLOW);
	tmp2 = desc->scan_begin_src & (TRIG_TIMER | TRIG_EXT | TRIG_FOLLOW);
	if (tmp1 != 0 || tmp2 == 0) {
		__a4l_err("a4l_check_cmddesc: scan_begin_src, , weird trigger\n");
		return -EINVAL;
	}

	tmp1 = desc->convert_src & ~(TRIG_TIMER | TRIG_EXT | TRIG_NOW);
	tmp2 = desc->convert_src & (TRIG_TIMER | TRIG_EXT | TRIG_NOW);
	if (tmp1 != 0 || tmp2 == 0) {
		__a4l_err("a4l_check_cmddesc: convert_src, weird trigger\n");
		return -EINVAL;
	}

	tmp1 = desc->scan_end_src & ~(TRIG_COUNT);
	if (tmp1 != 0) {
		__a4l_err("a4l_check_cmddesc: scan_end_src, weird trigger\n");
		return -EINVAL;
	}

	tmp1 = desc->stop_src & ~(TRIG_COUNT | TRIG_NONE);
	tmp2 = desc->stop_src & (TRIG_COUNT | TRIG_NONE);
	if (tmp1 != 0 || tmp2 == 0) {
		__a4l_err("a4l_check_cmddesc: stop_src, weird trigger\n");
		return -EINVAL;
	}

	/* Makes sure trigger sources are unique */
	if (desc->start_src != TRIG_NOW &&
	    desc->start_src != TRIG_INT &&
	    desc->start_src != TRIG_EXT && desc->start_src != TRIG_FOLLOW) {
		__a4l_err("a4l_check_cmddesc: start_src, "
			  "only one trigger should be set\n");
		return -EINVAL;
	}

	if (desc->scan_begin_src != TRIG_TIMER &&
	    desc->scan_begin_src != TRIG_EXT &&
	    desc->scan_begin_src != TRIG_FOLLOW) {
		__a4l_err("a4l_check_cmddesc: scan_begin_src, "
			  "only one trigger should be set\n");
		return -EINVAL;
	}

	if (desc->convert_src != TRIG_TIMER &&
	    desc->convert_src != TRIG_EXT && desc->convert_src != TRIG_NOW) {
		__a4l_err("a4l_check_cmddesc: convert_src, "
			  "only one trigger should be set\n");
		return -EINVAL;
	}

	if (desc->stop_src != TRIG_COUNT && desc->stop_src != TRIG_NONE) {
		__a4l_err("a4l_check_cmddesc: stop_src, "
			  "only one trigger should be set\n");
		return -EINVAL;
	}

	/* Makes sure arguments are trivially compatible */
	tmp1 = desc->start_src & (TRIG_NOW | TRIG_FOLLOW | TRIG_INT);
	tmp2 = desc->start_arg;
	if (tmp1 != 0 && tmp2 != 0) {
		__a4l_err("a4l_check_cmddesc: no start_arg expected\n");
		return -EINVAL;
	}

	tmp1 = desc->scan_begin_src & TRIG_FOLLOW;
	tmp2 = desc->scan_begin_arg;
	if (tmp1 != 0 && tmp2 != 0) {
		__a4l_err("a4l_check_cmddesc: no scan_begin_arg expected\n");
		return -EINVAL;
	}

	tmp1 = desc->convert_src & TRIG_NOW;
	tmp2 = desc->convert_arg;
	if (tmp1 != 0 && tmp2 != 0) {
		__a4l_err("a4l_check_cmddesc: no convert_arg expected\n");
		return -EINVAL;
	}

	tmp1 = desc->stop_src & TRIG_NONE;
	tmp2 = desc->stop_arg;
	if (tmp1 != 0 && tmp2 != 0) {
		__a4l_err("a4l_check_cmddesc: no stop_arg expected\n");
		return -EINVAL;
	}

	return 0;
}

int a4l_check_specific_cmdcnt(struct a4l_device_context * cxt, struct a4l_cmd_desc * desc)
{
	unsigned int tmp1, tmp2;
	struct a4l_device *dev = a4l_get_dev(cxt);
	struct a4l_cmd_desc *cmd_mask = dev->transfer.subds[desc->idx_subd]->cmd_mask;

	if (cmd_mask == NULL)
		return 0;

	if (cmd_mask->start_src != 0) {
		tmp1 = desc->start_src & ~(cmd_mask->start_src);
		tmp2 = desc->start_src & (cmd_mask->start_src);
		if (tmp1 != 0 || tmp2 == 0) {
			__a4l_err("a4l_check_cmddesc: start_src, "
				  "trigger unsupported\n");
			return -EINVAL;
		}
	}

	if (cmd_mask->scan_begin_src != 0) {
		tmp1 = desc->scan_begin_src & ~(cmd_mask->scan_begin_src);
		tmp2 = desc->scan_begin_src & (cmd_mask->scan_begin_src);
		if (tmp1 != 0 || tmp2 == 0) {
			__a4l_err("a4l_check_cmddesc: scan_begin_src, "
				  "trigger unsupported\n");
			return -EINVAL;
		}
	}

	if (cmd_mask->convert_src != 0) {
		tmp1 = desc->convert_src & ~(cmd_mask->convert_src);
		tmp2 = desc->convert_src & (cmd_mask->convert_src);
		if (tmp1 != 0 || tmp2 == 0) {
			__a4l_err("a4l_check_cmddesc: convert_src, "
				  "trigger unsupported\n");
			return -EINVAL;
		}
	}

	if (cmd_mask->scan_end_src != 0) {
		tmp1 = desc->scan_end_src & ~(cmd_mask->scan_end_src);
		if (tmp1 != 0) {
			__a4l_err("a4l_check_cmddesc: scan_end_src, "
				  "trigger unsupported\n");
			return -EINVAL;
		}
	}

	if (cmd_mask->stop_src != 0) {
		tmp1 = desc->stop_src & ~(cmd_mask->stop_src);
		tmp2 = desc->stop_src & (cmd_mask->stop_src);
		if (tmp1 != 0 || tmp2 == 0) {
			__a4l_err("a4l_check_cmddesc: stop_src, "
				  "trigger unsupported\n");
			return -EINVAL;
		}
	}

	return 0;
}

/* --- IOCTL / FOPS function --- */

int a4l_ioctl_cmd(struct a4l_device_context * ctx, void *arg)
{
	int ret = 0, simul_flag = 0;
	struct a4l_cmd_desc *cmd_desc = NULL;
	struct a4l_device *dev = a4l_get_dev(ctx);
	unsigned int *chan_descs, *tmp;
	struct a4l_subdevice *subd;

	/* The command launching cannot be done in real-time because
	   of some possible buffer allocations in the drivers */
	if (rtdm_in_rt_context())
		return -ENOSYS;

	/* Basically check the device */
	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_ioctl_cmd: cannot command "
			  "an unattached device\n");
		return -EINVAL;
	}

	/* Allocates the command */
	cmd_desc = (struct a4l_cmd_desc *) rtdm_malloc(sizeof(struct a4l_cmd_desc));
	if (cmd_desc == NULL)
		return -ENOMEM;
	memset(cmd_desc, 0, sizeof(struct a4l_cmd_desc));

	/* Gets the command */
	ret = a4l_fill_cmddesc(ctx, cmd_desc, &chan_descs, arg);
	if (ret != 0)
		goto out_ioctl_cmd;

	/* Checks the command */
	ret = a4l_check_cmddesc(ctx, cmd_desc);
	if (ret != 0)
		goto out_ioctl_cmd;

	ret = a4l_check_generic_cmdcnt(cmd_desc);
	if (ret != 0)
		goto out_ioctl_cmd;

	ret = a4l_check_specific_cmdcnt(ctx, cmd_desc);
	if (ret != 0)
		goto out_ioctl_cmd;

	__a4l_dbg(1, core_dbg,"1st cmd checks passed\n");
	subd = dev->transfer.subds[cmd_desc->idx_subd];

	/* Tests the command with the cmdtest function */
	if (cmd_desc->flags & A4L_CMD_SIMUL) {
		simul_flag = 1;

		if (!subd->do_cmdtest) {
			__a4l_err("a4l_ioctl_cmd: driver's cmd_test NULL\n");
			ret = -EINVAL;
			goto out_ioctl_cmd;
		}

		ret = subd->do_cmdtest(subd, cmd_desc);
		if (ret != 0) {
			__a4l_err("a4l_ioctl_cmd: driver's cmd_test failed\n");
			goto out_ioctl_cmd;
		}
		__a4l_dbg(1, core_dbg, "driver's cmd checks passed\n");
		goto out_ioctl_cmd;
	}


	/* Gets the transfer system ready */
	ret = a4l_setup_buffer(ctx, cmd_desc);
	if (ret < 0)
		goto out_ioctl_cmd;

	/* Eventually launches the command */
	ret = subd->do_cmd(subd, cmd_desc);

	if (ret != 0) {
		a4l_cancel_buffer(ctx);
		goto out_ioctl_cmd;
	}

	out_ioctl_cmd:

	if (simul_flag) {
		/* copy the kernel based descriptor */
		tmp = cmd_desc->chan_descs;
		/* return the user based descriptor */
		cmd_desc->chan_descs = chan_descs;
		rtdm_safe_copy_to_user(rtdm_private_to_fd(ctx), arg, cmd_desc,
				       sizeof(struct a4l_cmd_desc));
		/* make sure we release the memory associated to the kernel */
		cmd_desc->chan_descs = tmp;

	}

	if (ret != 0 || simul_flag == 1) {
		a4l_free_cmddesc(cmd_desc);
		rtdm_free(cmd_desc);
	}

	return ret;
}
