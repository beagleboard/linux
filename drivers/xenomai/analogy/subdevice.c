/*
 * Analogy for Linux, subdevice, channel and range related features
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

/* --- Common ranges declarations --- */

struct a4l_rngtab rng_bipolar10 = { 1, {
		RANGE_V(-10, 10),
	}};
struct a4l_rngdesc a4l_range_bipolar10 = RNG_GLOBAL(rng_bipolar10);

struct a4l_rngtab rng_bipolar5 = { 1, {
		RANGE_V(-5, 5),
	}};
struct a4l_rngdesc a4l_range_bipolar5 = RNG_GLOBAL(rng_bipolar5);

struct a4l_rngtab rng_unipolar10 = { 1, {
		RANGE_V(0, 10),
	}};
struct a4l_rngdesc a4l_range_unipolar10 = RNG_GLOBAL(rng_unipolar10);

struct a4l_rngtab rng_unipolar5 = { 1, {
		RANGE_V(0, 5),
	}};
struct a4l_rngdesc a4l_range_unipolar5 = RNG_GLOBAL(rng_unipolar5);

struct a4l_rngtab rng_unknown = { 1, {
		RANGE(0, 1),
	}};
struct a4l_rngdesc a4l_range_unknown = RNG_GLOBAL(rng_unknown);

struct a4l_rngtab rng_fake = { 0, {
		RANGE(0, 0),
	}};
struct a4l_rngdesc a4l_range_fake = RNG_GLOBAL(rng_fake);

/* --- Basic channel / range management functions --- */

struct a4l_channel *a4l_get_chfeat(struct a4l_subdevice *sb, int idx)
{
	int i = (sb->chan_desc->mode != A4L_CHAN_GLOBAL_CHANDESC) ? idx : 0;
	return &(sb->chan_desc->chans[i]);
}

struct a4l_range *a4l_get_rngfeat(struct a4l_subdevice *sb, int chidx, int rngidx)
{
	int i = (sb->rng_desc->mode != A4L_RNG_GLOBAL_RNGDESC) ? chidx : 0;
	return &(sb->rng_desc->rngtabs[i]->rngs[rngidx]);
}

int a4l_check_chanlist(struct a4l_subdevice *subd,
		       unsigned char nb_chan, unsigned int *chans)
{
	int i, j;

	if (nb_chan > subd->chan_desc->length)
		return -EINVAL;

	for (i = 0; i < nb_chan; i++) {
		j = (subd->chan_desc->mode != A4L_CHAN_GLOBAL_CHANDESC) ? i : 0;

		if (CR_CHAN(chans[i]) >= subd->chan_desc->length) {
			__a4l_err("a4l_check_chanlist: "
				  "chan idx out_of range (%u>=%lu)\n",
				  CR_CHAN(chans[i]), subd->chan_desc->length);
			return -EINVAL;
		}
		if (CR_AREF(chans[i]) != 0 &&
		    (CR_AREF(chans[i]) & subd->chan_desc->chans[j].flags) == 0)
		{
			__a4l_err("a4l_check_chanlist: "
				  "bad channel type\n");
			return -EINVAL;
		}
	}

	if (subd->rng_desc == NULL)
		return 0;

	for (i = 0; i < nb_chan; i++) {
		j = (subd->rng_desc->mode != A4L_RNG_GLOBAL_RNGDESC) ? i : 0;

		if (CR_RNG(chans[i]) > subd->rng_desc->rngtabs[j]->length) {
			__a4l_err("a4l_check_chanlist: "
				  "rng idx out_of range (%u>=%u)\n",
				  CR_RNG(chans[i]),
				  subd->rng_desc->rngtabs[j]->length);
			return -EINVAL;
		}
	}

	return 0;
}

/* --- Upper layer functions --- */

struct a4l_subdevice * a4l_alloc_subd(int sizeof_priv,
			    void (*setup)(struct a4l_subdevice *))
{
	struct a4l_subdevice *subd;

	subd = rtdm_malloc(sizeof(struct a4l_subdevice) + sizeof_priv);

	if(subd != NULL) {
		memset(subd, 0 , sizeof(struct a4l_subdevice) + sizeof_priv);
		if(setup != NULL)
			setup(subd);
	}

	return subd;
}

int a4l_add_subd(struct a4l_device * dev, struct a4l_subdevice * subd)
{
	struct list_head *this;
	int i = 0;

	/* Basic checking */
	if (dev == NULL || subd == NULL)
		return -EINVAL;

	list_add_tail(&subd->list, &dev->subdvsq);

	subd->dev = dev;

	list_for_each(this, &dev->subdvsq) {
		i++;
	}

	subd->idx = --i;

	return i;
}

struct a4l_subdevice *a4l_get_subd(struct a4l_device *dev, int idx)
{
	int i = 0;
	struct a4l_subdevice *subd = NULL;
	struct list_head *this;

	/* This function is not optimized as we do not go through the
	   transfer structure */

	list_for_each(this, &dev->subdvsq) {
		if(idx == i++)
			subd = list_entry(this, struct a4l_subdevice, list);
	}

	return subd;
}

/* --- IOCTL / FOPS functions --- */

int a4l_ioctl_subdinfo(struct a4l_device_context * cxt, void *arg)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	struct a4l_device *dev = a4l_get_dev(cxt);
	int i, ret = 0;
	a4l_sbinfo_t *subd_info;

	/* Basic checking */
	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_ioctl_subdinfo: unattached device\n");
		return -EINVAL;
	}

	subd_info = rtdm_malloc(dev->transfer.nb_subd *
				sizeof(a4l_sbinfo_t));
	if (subd_info == NULL)
		return -ENOMEM;

	for (i = 0; i < dev->transfer.nb_subd; i++) {
		subd_info[i].flags = dev->transfer.subds[i]->flags;
		subd_info[i].status = dev->transfer.subds[i]->status;
		subd_info[i].nb_chan =
			(dev->transfer.subds[i]->chan_desc != NULL) ?
			dev->transfer.subds[i]->chan_desc->length : 0;
	}

	if (rtdm_safe_copy_to_user(fd,
				   arg,
				   subd_info, dev->transfer.nb_subd *
				   sizeof(a4l_sbinfo_t)) != 0)
		ret = -EFAULT;

	rtdm_free(subd_info);

	return ret;

}

int a4l_ioctl_nbchaninfo(struct a4l_device_context * cxt, void *arg)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	struct a4l_device *dev = a4l_get_dev(cxt);
	a4l_chinfo_arg_t inarg;

	/* Basic checking */
	if (!dev->flags & A4L_DEV_ATTACHED_NR) {
		__a4l_err("a4l_ioctl_nbchaninfo: unattached device\n");
		return -EINVAL;
	}

	if (rtdm_safe_copy_from_user(fd,
				     &inarg, arg,
				     sizeof(a4l_chinfo_arg_t)) != 0)
		return -EFAULT;

	if (inarg.idx_subd >= dev->transfer.nb_subd) {
		__a4l_err("a4l_ioctl_nbchaninfo: subdevice index "
			  "out of range\n");
		return -EINVAL;
	}

	if(dev->transfer.subds[inarg.idx_subd]->chan_desc == NULL)
		inarg.info = (void *)0;
	else
		inarg.info = (void *)(unsigned long)
			dev->transfer.subds[inarg.idx_subd]->chan_desc->length;

	if (rtdm_safe_copy_to_user(fd,
				   arg,
				   &inarg, sizeof(a4l_chinfo_arg_t)) != 0)
		return -EFAULT;

	return 0;
}

int a4l_ioctl_chaninfo(struct a4l_device_context * cxt, void *arg)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	int i, ret = 0;
	struct a4l_device *dev = a4l_get_dev(cxt);
	a4l_chinfo_t *chan_info;
	a4l_chinfo_arg_t inarg;
	struct a4l_channels_desc *chan_desc;
	struct a4l_rngdesc *rng_desc;

	/* Basic checking */
	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_ioctl_chaninfo: unattached device\n");
		return -EINVAL;
	}

	if (rtdm_safe_copy_from_user(fd,
				     &inarg, arg,
				     sizeof(a4l_chinfo_arg_t)) != 0)
		return -EFAULT;

	if (inarg.idx_subd >= dev->transfer.nb_subd) {
		__a4l_err("a4l_ioctl_chaninfo: bad subdevice index\n");
		return -EINVAL;
	}

	chan_desc = dev->transfer.subds[inarg.idx_subd]->chan_desc;
	rng_desc = dev->transfer.subds[inarg.idx_subd]->rng_desc;

	if (chan_desc == NULL) {
		__a4l_err("a4l_ioctl_chaninfo: no channel descriptor "
			  "for subdevice %d\n", inarg.idx_subd);
		return -EINVAL;
	}

	if(rng_desc == NULL)
		rng_desc = &a4l_range_fake;

	chan_info = rtdm_malloc(chan_desc->length * sizeof(a4l_chinfo_t));
	if (chan_info == NULL)
		return -ENOMEM;

	/* If the channel descriptor is global, the fields are filled
	   with the same instance of channel descriptor */
	for (i = 0; i < chan_desc->length; i++) {
		int j =
			(chan_desc->mode != A4L_CHAN_GLOBAL_CHANDESC) ? i : 0;
		int k = (rng_desc->mode != A4L_RNG_GLOBAL_RNGDESC) ? i : 0;

		chan_info[i].chan_flags = chan_desc->chans[j].flags;
		chan_info[i].nb_bits = chan_desc->chans[j].nb_bits;
		chan_info[i].nb_rng = rng_desc->rngtabs[k]->length;

		if (chan_desc->mode == A4L_CHAN_GLOBAL_CHANDESC)
			chan_info[i].chan_flags |= A4L_CHAN_GLOBAL;
	}

	if (rtdm_safe_copy_to_user(fd,
				   inarg.info,
				   chan_info,
				   chan_desc->length *
				   sizeof(a4l_chinfo_t)) != 0)
		return -EFAULT;

	rtdm_free(chan_info);

	return ret;
}

int a4l_ioctl_nbrnginfo(struct a4l_device_context * cxt, void *arg)
{
	int i;
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	struct a4l_device *dev = a4l_get_dev(cxt);
	a4l_rnginfo_arg_t inarg;
	struct a4l_rngdesc *rng_desc;

	/* Basic checking */
	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_ioctl_nbrnginfo: unattached device\n");
		return -EINVAL;
	}

	if (rtdm_safe_copy_from_user(fd,
				     &inarg,
				     arg, sizeof(a4l_rnginfo_arg_t)) != 0)
		return -EFAULT;

	if (inarg.idx_subd >= dev->transfer.nb_subd) {
		__a4l_err("a4l_ioctl_nbrnginfo: bad subdevice index\n");
		return -EINVAL;
	}

	if (dev->transfer.subds[inarg.idx_subd]->chan_desc == NULL) {
		__a4l_err("a4l_ioctl_nbrnginfo: no channel descriptor "
			  "for subdevice %d\n", inarg.idx_subd);
		return -EINVAL;
	}

	if (inarg.idx_chan >=
	    dev->transfer.subds[inarg.idx_subd]->chan_desc->length) {
		__a4l_err("a4l_ioctl_nbrnginfo: bad channel index\n");
		return -EINVAL;
	}

	rng_desc = dev->transfer.subds[inarg.idx_subd]->rng_desc;
	if (rng_desc != NULL) {
		i = (rng_desc->mode != A4L_RNG_GLOBAL_RNGDESC) ?
			inarg.idx_chan : 0;
		inarg.info = (void *)(unsigned long)
			rng_desc->rngtabs[i]->length;
	} else
		inarg.info = (void *)0;


	if (rtdm_safe_copy_to_user(fd,
				   arg,
				   &inarg, sizeof(a4l_rnginfo_arg_t)) != 0)
		return -EFAULT;

	return 0;
}

int a4l_ioctl_rnginfo(struct a4l_device_context * cxt, void *arg)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	int i, ret = 0;
	unsigned int tmp;
	struct a4l_device *dev = a4l_get_dev(cxt);
	struct a4l_rngdesc *rng_desc;
	a4l_rnginfo_t *rng_info;
	a4l_rnginfo_arg_t inarg;

	/* Basic checking */
	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_ioctl_rnginfo: unattached device\n");
		return -EINVAL;
	}

	if (rtdm_safe_copy_from_user(fd,
				     &inarg,
				     arg, sizeof(a4l_rnginfo_arg_t)) != 0)
		return -EFAULT;

	if (inarg.idx_subd >= dev->transfer.nb_subd) {
		__a4l_err("a4l_ioctl_rnginfo: bad subdevice index\n");
		return -EINVAL;
	}

	if (dev->transfer.subds[inarg.idx_subd]->chan_desc == NULL) {
		__a4l_err("a4l_ioctl_rnginfo: no channel descriptor "
			  "for subdevice %d\n", inarg.idx_subd);
		return -EINVAL;
	}

	if (inarg.idx_chan >=
	    dev->transfer.subds[inarg.idx_subd]->chan_desc->length) {
		__a4l_err("a4l_ioctl_rnginfo: bad channel index\n");
		return -EINVAL;
	}

	rng_desc = dev->transfer.subds[inarg.idx_subd]->rng_desc;
	if (rng_desc == NULL) {
		__a4l_err("a4l_ioctl_rnginfo: no range descriptor "
			  "for channel %d\n", inarg.idx_chan);
		return -EINVAL;
	}

	/* If the range descriptor is global,
	   we take the first instance */
	tmp = (rng_desc->mode != A4L_RNG_GLOBAL_RNGDESC) ?
		inarg.idx_chan : 0;

	rng_info = rtdm_malloc(rng_desc->rngtabs[tmp]->length *
			       sizeof(a4l_rnginfo_t));
	if (rng_info == NULL)
		return -ENOMEM;

	for (i = 0; i < rng_desc->rngtabs[tmp]->length; i++) {
		rng_info[i].min = rng_desc->rngtabs[tmp]->rngs[i].min;
		rng_info[i].max = rng_desc->rngtabs[tmp]->rngs[i].max;
		rng_info[i].flags = rng_desc->rngtabs[tmp]->rngs[i].flags;

		if (rng_desc->mode == A4L_RNG_GLOBAL_RNGDESC)
			rng_info[i].flags |= A4L_RNG_GLOBAL;
	}

	if (rtdm_safe_copy_to_user(fd,
				   inarg.info,
				   rng_info,
				   rng_desc->rngtabs[tmp]->length *
				   sizeof(a4l_rnginfo_t)) != 0)
		return -EFAULT;

	rtdm_free(rng_info);

	return ret;
}
