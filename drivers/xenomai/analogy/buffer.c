/*
 * Analogy for Linux, buffer related features
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
#include <linux/mman.h>
#include <linux/vmalloc.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <rtdm/analogy/device.h>

/* --- Initialization functions (init, alloc, free) --- */

/* The buffer charactistic is very close to the Comedi one: it is
   allocated with vmalloc() and all physical addresses of the pages which
   compose the virtual buffer are hold in a table */

void a4l_free_buffer(struct a4l_buffer * buf_desc)
{
	__a4l_dbg(1, core_dbg, "buf=%p buf->buf=%p\n", buf_desc, buf_desc->buf);

	if (buf_desc->pg_list != NULL) {
		rtdm_free(buf_desc->pg_list);
		buf_desc->pg_list = NULL;
	}

	if (buf_desc->buf != NULL) {
		char *vaddr, *vabase = buf_desc->buf;
		for (vaddr = vabase; vaddr < vabase + buf_desc->size;
		     vaddr += PAGE_SIZE)
			ClearPageReserved(vmalloc_to_page(vaddr));
		vfree(buf_desc->buf);
		buf_desc->buf = NULL;
	}
}

int a4l_alloc_buffer(struct a4l_buffer *buf_desc, int buf_size)
{
	int ret = 0;
	char *vaddr, *vabase;

	buf_desc->size = buf_size;
	buf_desc->size = PAGE_ALIGN(buf_desc->size);

	buf_desc->buf = vmalloc_32(buf_desc->size);
	if (buf_desc->buf == NULL) {
		ret = -ENOMEM;
		goto out_virt_contig_alloc;
	}

	vabase = buf_desc->buf;

	for (vaddr = vabase; vaddr < vabase + buf_desc->size;
	     vaddr += PAGE_SIZE)
		SetPageReserved(vmalloc_to_page(vaddr));

	buf_desc->pg_list = rtdm_malloc(((buf_desc->size) >> PAGE_SHIFT) *
					sizeof(unsigned long));
	if (buf_desc->pg_list == NULL) {
		ret = -ENOMEM;
		goto out_virt_contig_alloc;
	}

	for (vaddr = vabase; vaddr < vabase + buf_desc->size;
	     vaddr += PAGE_SIZE)
		buf_desc->pg_list[(vaddr - vabase) >> PAGE_SHIFT] =
			(unsigned long) page_to_phys(vmalloc_to_page(vaddr));

	__a4l_dbg(1, core_dbg, "buf=%p buf->buf=%p\n", buf_desc, buf_desc->buf);

out_virt_contig_alloc:
	if (ret != 0)
		a4l_free_buffer(buf_desc);

	return ret;
}

static void a4l_reinit_buffer(struct a4l_buffer *buf_desc)
{
	/* No command to process yet */
	buf_desc->cur_cmd = NULL;

	/* No more (or not yet) linked with a subdevice */
	buf_desc->subd = NULL;

	/* Initializes counts and flags */
	buf_desc->end_count = 0;
	buf_desc->prd_count = 0;
	buf_desc->cns_count = 0;
	buf_desc->tmp_count = 0;
	buf_desc->mng_count = 0;

	/* Flush pending events */
	buf_desc->flags = 0;
	a4l_flush_sync(&buf_desc->sync);
}

void a4l_init_buffer(struct a4l_buffer *buf_desc)
{
	memset(buf_desc, 0, sizeof(struct a4l_buffer));
	a4l_init_sync(&buf_desc->sync);
	a4l_reinit_buffer(buf_desc);
}

void a4l_cleanup_buffer(struct a4l_buffer *buf_desc)
{
	a4l_cleanup_sync(&buf_desc->sync);
}

int a4l_setup_buffer(struct a4l_device_context *cxt, struct a4l_cmd_desc *cmd)
{
	struct a4l_buffer *buf_desc = cxt->buffer;
	int i;

	/* Retrieve the related subdevice */
	buf_desc->subd = a4l_get_subd(cxt->dev, cmd->idx_subd);
	if (buf_desc->subd == NULL) {
		__a4l_err("a4l_setup_buffer: subdevice index "
			  "out of range (%d)\n", cmd->idx_subd);
		return -EINVAL;
	}

	if (test_and_set_bit(A4L_SUBD_BUSY_NR, &buf_desc->subd->status)) {
		__a4l_err("a4l_setup_buffer: subdevice %d already busy\n",
			  cmd->idx_subd);
		return -EBUSY;
	}

	/* Checks if the transfer system has to work in bulk mode */
	if (cmd->flags & A4L_CMD_BULK)
		set_bit(A4L_BUF_BULK_NR, &buf_desc->flags);

	/* Sets the working command */
	buf_desc->cur_cmd = cmd;

	/* Link the subdevice with the context's buffer */
	buf_desc->subd->buf = buf_desc;

	/* Computes the count to reach, if need be */
	if (cmd->stop_src == TRIG_COUNT) {
		for (i = 0; i < cmd->nb_chan; i++) {
			struct a4l_channel *chft;
			chft = a4l_get_chfeat(buf_desc->subd,
					      CR_CHAN(cmd->chan_descs[i]));
			buf_desc->end_count += chft->nb_bits / 8;
		}
		buf_desc->end_count *= cmd->stop_arg;
	}

	__a4l_dbg(1, core_dbg, "end_count=%lu\n", buf_desc->end_count);

	return 0;
}

void a4l_cancel_buffer(struct a4l_device_context *cxt)
{
	struct a4l_buffer *buf_desc = cxt->buffer;
	struct a4l_subdevice *subd = buf_desc->subd;

	if (!subd || !test_bit(A4L_SUBD_BUSY_NR, &subd->status))
		return;

	/* If a "cancel" function is registered, call it
	   (Note: this function is called before having checked
	   if a command is under progress; we consider that
	   the "cancel" function can be used as as to (re)initialize
	   some component) */
	if (subd->cancel != NULL)
		subd->cancel(subd);

	if (buf_desc->cur_cmd != NULL) {
		a4l_free_cmddesc(buf_desc->cur_cmd);
		rtdm_free(buf_desc->cur_cmd);
		buf_desc->cur_cmd = NULL;
	}

	a4l_reinit_buffer(buf_desc);

	clear_bit(A4L_SUBD_BUSY_NR, &subd->status);
	subd->buf = NULL;
}

/* --- Munge related function --- */

int a4l_get_chan(struct a4l_subdevice *subd)
{
	int i, j, tmp_count, tmp_size = 0;
	struct a4l_cmd_desc *cmd;

	cmd = a4l_get_cmd(subd);
	if (!cmd)
		return -EINVAL;

	/* There is no need to check the channel idx,
	   it has already been controlled in command_test */

	/* We assume channels can have different sizes;
	   so, we have to compute the global size of the channels
	   in this command... */
	for (i = 0; i < cmd->nb_chan; i++) {
		j = (subd->chan_desc->mode != A4L_CHAN_GLOBAL_CHANDESC) ?
			CR_CHAN(cmd->chan_descs[i]) : 0;
		tmp_size += subd->chan_desc->chans[j].nb_bits;
	}

	/* Translation bits -> bytes */
	tmp_size /= 8;

	tmp_count = subd->buf->mng_count % tmp_size;

	/* Translation bytes -> bits */
	tmp_count *= 8;

	/* ...and find the channel the last munged sample
	   was related with */
	for (i = 0; tmp_count > 0 && i < cmd->nb_chan; i++) {
		j = (subd->chan_desc->mode != A4L_CHAN_GLOBAL_CHANDESC) ?
			CR_CHAN(cmd->chan_descs[i]) : 0;
		tmp_count -= subd->chan_desc->chans[j].nb_bits;
	}

	if (tmp_count == 0)
		return i;
	else
		return -EINVAL;
}

/* --- Transfer / copy functions --- */

/* The following functions are explained in the Doxygen section
   "Buffer management services" in driver_facilities.c */

int a4l_buf_prepare_absput(struct a4l_subdevice *subd, unsigned long count)
{
	struct a4l_buffer *buf = subd->buf;

	if (!buf || !test_bit(A4L_SUBD_BUSY_NR, &subd->status))
		return -ENOENT;

	if (!a4l_subd_is_input(subd))
		return -EINVAL;

	return __pre_abs_put(buf, count);
}


int a4l_buf_commit_absput(struct a4l_subdevice *subd, unsigned long count)
{
	struct a4l_buffer *buf = subd->buf;

	if (!buf || !test_bit(A4L_SUBD_BUSY_NR, &subd->status))
		return -ENOENT;

	if (!a4l_subd_is_input(subd))
		return -EINVAL;

	return __abs_put(buf, count);
}

int a4l_buf_prepare_put(struct a4l_subdevice *subd, unsigned long count)
{
	struct a4l_buffer *buf = subd->buf;

	if (!buf || !test_bit(A4L_SUBD_BUSY_NR, &subd->status))
		return -ENOENT;

	if (!a4l_subd_is_input(subd))
		return -EINVAL;

	return __pre_put(buf, count);
}

int a4l_buf_commit_put(struct a4l_subdevice *subd, unsigned long count)
{
	struct a4l_buffer *buf = subd->buf;

	if (!buf || !test_bit(A4L_SUBD_BUSY_NR, &subd->status))
		return -ENOENT;

	if (!a4l_subd_is_input(subd))
		return -EINVAL;

	return __put(buf, count);
}

int a4l_buf_put(struct a4l_subdevice *subd, void *bufdata, unsigned long count)
{
	struct a4l_buffer *buf = subd->buf;
	int err;

	if (!buf || !test_bit(A4L_SUBD_BUSY_NR, &subd->status))
		return -ENOENT;

	if (!a4l_subd_is_input(subd))
		return -EINVAL;

	if (__count_to_put(buf) < count)
		return -EAGAIN;

	err = __produce(NULL, buf, bufdata, count);
	if (err < 0)
		return err;

	err = __put(buf, count);

	return err;
}

int a4l_buf_prepare_absget(struct a4l_subdevice *subd, unsigned long count)
{
	struct a4l_buffer *buf = subd->buf;

	if (!buf || !test_bit(A4L_SUBD_BUSY_NR, &subd->status))
		return -ENOENT;

	if (!a4l_subd_is_output(subd))
		return -EINVAL;

	return __pre_abs_get(buf, count);
}

int a4l_buf_commit_absget(struct a4l_subdevice *subd, unsigned long count)
{
	struct a4l_buffer *buf = subd->buf;

	if (!buf || !test_bit(A4L_SUBD_BUSY_NR, &subd->status))
		return -ENOENT;

	if (!a4l_subd_is_output(subd))
		return -EINVAL;

	return __abs_get(buf, count);
}

int a4l_buf_prepare_get(struct a4l_subdevice *subd, unsigned long count)
{
	struct a4l_buffer *buf = subd->buf;

	if (!buf || !test_bit(A4L_SUBD_BUSY_NR, &subd->status))
		return -ENOENT;

	if (!a4l_subd_is_output(subd))
		return -EINVAL;

	return __pre_get(buf, count);
}

int a4l_buf_commit_get(struct a4l_subdevice *subd, unsigned long count)
{
	struct a4l_buffer *buf = subd->buf;

	/* Basic checkings */

	if (!buf || !test_bit(A4L_SUBD_BUSY_NR, &subd->status))
		return -ENOENT;

	if (!a4l_subd_is_output(subd))
		return -EINVAL;

	return __get(buf, count);
}

int a4l_buf_get(struct a4l_subdevice *subd, void *bufdata, unsigned long count)
{
	struct a4l_buffer *buf = subd->buf;
	int err;

	/* Basic checkings */

	if (!buf || !test_bit(A4L_SUBD_BUSY_NR, &subd->status))
		return -ENOENT;

	if (!a4l_subd_is_output(subd))
		return -EINVAL;

	if (__count_to_get(buf) < count)
		return -EAGAIN;

	/* Update the counter */
	err = __consume(NULL, buf, bufdata, count);
	if (err < 0)
		return err;

	/* Perform the transfer */
	err = __get(buf, count);

	return err;
}

int a4l_buf_evt(struct a4l_subdevice *subd, unsigned long evts)
{
	struct a4l_buffer *buf = subd->buf;
	int tmp;
	unsigned long wake = 0, count = ULONG_MAX;

	/* Warning: here, there may be a condition race : the cancel
	   function is called by the user side and a4l_buf_evt and all
	   the a4l_buf_... functions are called by the kernel
	   side. Nonetheless, the driver should be in charge of such
	   race conditions, not the framework */

	/* Basic checking */
	if (!buf || !test_bit(A4L_SUBD_BUSY_NR, &subd->status))
		return -ENOENT;

	/* Here we save the data count available for the user side */
	if (evts == 0) {
		count = a4l_subd_is_input(subd) ?
			__count_to_get(buf) : __count_to_put(buf);
		wake = __count_to_end(buf) < buf->wake_count ?
			__count_to_end(buf) : buf->wake_count;
	} else {
		/* Even if it is a little more complex, atomic
		   operations are used so as to prevent any kind of
		   corner case */
		while ((tmp = ffs(evts) - 1) != -1) {
			set_bit(tmp, &buf->flags);
			clear_bit(tmp, &evts);
		}
	}

	if (count >= wake)
		/* Notify the user-space side */
		a4l_signal_sync(&buf->sync);

	return 0;
}

unsigned long a4l_buf_count(struct a4l_subdevice *subd)
{
	struct a4l_buffer *buf = subd->buf;
	unsigned long ret = 0;

	/* Basic checking */
	if (!buf || !test_bit(A4L_SUBD_BUSY_NR, &subd->status))
		return -ENOENT;

	if (a4l_subd_is_input(subd))
		ret = __count_to_put(buf);
	else if (a4l_subd_is_output(subd))
		ret = __count_to_get(buf);

	return ret;
}

/* --- Mmap functions --- */

void a4l_map(struct vm_area_struct *area)
{
	unsigned long *status = (unsigned long *)area->vm_private_data;
	set_bit(A4L_BUF_MAP_NR, status);
}

void a4l_unmap(struct vm_area_struct *area)
{
	unsigned long *status = (unsigned long *)area->vm_private_data;
	clear_bit(A4L_BUF_MAP_NR, status);
}

static struct vm_operations_struct a4l_vm_ops = {
	.open = a4l_map,
	.close = a4l_unmap,
};

int a4l_ioctl_mmap(struct a4l_device_context *cxt, void *arg)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	a4l_mmap_t map_cfg;
	struct a4l_device *dev;
	struct a4l_buffer *buf;
	int ret;

	/* The mmap operation cannot be performed in a
	   real-time context */
	if (rtdm_in_rt_context()) {
		return -ENOSYS;
	}

	dev = a4l_get_dev(cxt);
	buf = cxt->buffer;

	/* Basic checkings */

	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_ioctl_mmap: cannot mmap on "
			  "an unattached device\n");
		return -EINVAL;
	}

	if (test_bit(A4L_BUF_MAP_NR, &buf->flags)) {
		__a4l_err("a4l_ioctl_mmap: buffer already mapped\n");
		return -EBUSY;
	}

	if (rtdm_safe_copy_from_user(fd,
				     &map_cfg, arg, sizeof(a4l_mmap_t)) != 0)
		return -EFAULT;

	/* Check the size to be mapped */
	if ((map_cfg.size & ~(PAGE_MASK)) != 0 || map_cfg.size > buf->size)
		return -EFAULT;

	/* All the magic is here */
	ret = rtdm_mmap_to_user(fd,
				buf->buf,
				map_cfg.size,
				PROT_READ | PROT_WRITE,
				&map_cfg.ptr, &a4l_vm_ops, &buf->flags);

	if (ret < 0) {
		__a4l_err("a4l_ioctl_mmap: internal error, "
			  "rtdm_mmap_to_user failed (err=%d)\n", ret);
		return ret;
	}

	return rtdm_safe_copy_to_user(fd,
				      arg, &map_cfg, sizeof(a4l_mmap_t));
}

/* --- IOCTL / FOPS functions --- */

int a4l_ioctl_cancel(struct a4l_device_context * cxt, void *arg)
{
	unsigned int idx_subd = (unsigned long)arg;
	struct a4l_device *dev = a4l_get_dev(cxt);
	struct a4l_subdevice *subd;

	/* Basically check the device */
	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_ioctl_cancel: operation not supported on "
			  "an unattached device\n");
		return -EINVAL;
	}

	if (cxt->buffer->subd == NULL) {
		__a4l_err("a4l_ioctl_cancel: "
			  "no acquisition to cancel on this context\n");
		return -EINVAL;
	}

	if (idx_subd >= dev->transfer.nb_subd) {
		__a4l_err("a4l_ioctl_cancel: bad subdevice index\n");
		return -EINVAL;
	}

	subd = dev->transfer.subds[idx_subd];

	if (subd != cxt->buffer->subd) {
		__a4l_err("a4l_ioctl_cancel: "
			  "current context works on another subdevice "
			  "(%d!=%d)\n", cxt->buffer->subd->idx, subd->idx);
		return -EINVAL;
	}

	a4l_cancel_buffer(cxt);
	return 0;
}

/* The ioctl BUFCFG is only useful for changing the size of the
   asynchronous buffer.
   (BUFCFG = free of the current buffer + allocation of a new one) */

int a4l_ioctl_bufcfg(struct a4l_device_context * cxt, void *arg)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	struct a4l_device *dev = a4l_get_dev(cxt);
	struct a4l_buffer *buf = cxt->buffer;
	struct a4l_subdevice *subd = buf->subd;
	a4l_bufcfg_t buf_cfg;

	/* As Linux API is used to allocate a virtual buffer,
	   the calling process must not be in primary mode */
	if (rtdm_in_rt_context()) {
		return -ENOSYS;
	}

	/* Basic checking */
	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_ioctl_bufcfg: unattached device\n");
		return -EINVAL;
	}

	if (rtdm_safe_copy_from_user(fd,
				     &buf_cfg,
				     arg, sizeof(a4l_bufcfg_t)) != 0)
		return -EFAULT;

	if (buf_cfg.buf_size > A4L_BUF_MAXSIZE) {
		__a4l_err("a4l_ioctl_bufcfg: buffer size too big (<=16MB)\n");
		return -EINVAL;
	}

	if (buf_cfg.idx_subd == A4L_BUF_DEFMAGIC) {
		cxt->dev->transfer.default_bufsize = buf_cfg.buf_size;
		return 0;
	}

	if (subd && test_bit(A4L_SUBD_BUSY_NR, &subd->status)) {
		__a4l_err("a4l_ioctl_bufcfg: acquisition in progress\n");
		return -EBUSY;
	}

	if (test_bit(A4L_BUF_MAP, &buf->flags)) {
		__a4l_err("a4l_ioctl_bufcfg: please unmap before "
			  "configuring buffer\n");
		return -EPERM;
	}

	/* Free the buffer... */
	a4l_free_buffer(buf);

	/* ...to reallocate it */
	return a4l_alloc_buffer(buf, buf_cfg.buf_size);
}

/* The ioctl BUFCFG2 allows the user space process to define the
   minimal amount of data which should trigger a wake-up. If the ABI
   could be broken, this facility would be handled by the original
   BUFCFG ioctl. At the next major release, this ioctl will vanish. */

int a4l_ioctl_bufcfg2(struct a4l_device_context * cxt, void *arg)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	struct a4l_device *dev = a4l_get_dev(cxt);
	struct a4l_buffer *buf = cxt->buffer;
	a4l_bufcfg2_t buf_cfg;

	/* Basic checking */
	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_ioctl_bufcfg2: unattached device\n");
		return -EINVAL;
	}

	if (rtdm_safe_copy_from_user(fd,
				     &buf_cfg,
				     arg, sizeof(a4l_bufcfg2_t)) != 0)
		return -EFAULT;

	if (buf_cfg.wake_count > buf->size) {
		__a4l_err("a4l_ioctl_bufcfg2: "
			  "wake-up threshold too big (> buffer size: %lu)\n",
			  buf->size);
		return -EINVAL;
	}

	buf->wake_count = buf_cfg.wake_count;

	return 0;
}

/* The BUFINFO ioctl provides two basic roles:
   - tell the user app the size of the asynchronous buffer
   - display the read/write counters (how many bytes to read/write) */

int a4l_ioctl_bufinfo(struct a4l_device_context * cxt, void *arg)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	struct a4l_device *dev = a4l_get_dev(cxt);
	struct a4l_buffer *buf = cxt->buffer;
	struct a4l_subdevice *subd = buf->subd;
	a4l_bufinfo_t info;

	unsigned long tmp_cnt;
	int ret;

	if (!rtdm_in_rt_context() && rtdm_rt_capable(fd))
		return -ENOSYS;

	/* Basic checking */
	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_ioctl_bufinfo: unattached device\n");
		return -EINVAL;
	}

	if (rtdm_safe_copy_from_user(fd,
				     &info, arg, sizeof(a4l_bufinfo_t)) != 0)
		return -EFAULT;


	/* If a transfer is not occuring, simply return buffer
	   informations, otherwise make the transfer progress */
	if (!subd || !test_bit(A4L_SUBD_BUSY_NR, &subd->status)) {
		info.rw_count = 0;
		goto a4l_ioctl_bufinfo_out;
	}

	ret = __handle_event(buf);

	if (a4l_subd_is_input(subd)) {

		/* Updates consume count if rw_count is not null */
		if (info.rw_count != 0)
			buf->cns_count += info.rw_count;

		/* Retrieves the data amount to read */
		tmp_cnt = info.rw_count = __count_to_get(buf);

		__a4l_dbg(1, core_dbg, "count to read=%lu\n", tmp_cnt);

		if ((ret < 0 && ret != -ENOENT) ||
		    (ret == -ENOENT && tmp_cnt == 0)) {
			a4l_cancel_buffer(cxt);
			return ret;
		}
	} else if (a4l_subd_is_output(subd)) {

		if (ret < 0) {
			a4l_cancel_buffer(cxt);
			if (info.rw_count != 0)
				return ret;
		}

		/* If rw_count is not null,
		   there is something to write / munge  */
		if (info.rw_count != 0 && info.rw_count <= __count_to_put(buf)) {

			/* Updates the production pointer */
			buf->prd_count += info.rw_count;

			/* Sets the munge count */
			tmp_cnt = info.rw_count;
		} else
			tmp_cnt = 0;

		/* Retrieves the data amount which is writable */
		info.rw_count = __count_to_put(buf);

		__a4l_dbg(1, core_dbg, " count to write=%lu\n", info.rw_count);

	} else {
		__a4l_err("a4l_ioctl_bufinfo: inappropriate subdevice\n");
		return -EINVAL;
	}

	/* Performs the munge if need be */
	if (subd->munge != NULL) {

		/* Call the munge callback */
		__munge(subd, subd->munge, buf, tmp_cnt);

		/* Updates munge count */
		buf->mng_count += tmp_cnt;
	}

a4l_ioctl_bufinfo_out:

	/* Sets the buffer size */
	info.buf_size = buf->size;

	/* Sends the structure back to user space */
	if (rtdm_safe_copy_to_user(fd,
				   arg, &info, sizeof(a4l_bufinfo_t)) != 0)
		return -EFAULT;

	return 0;
}

/* The ioctl BUFINFO2 tells the user application the minimal amount of
data which should trigger a wake-up. If the ABI could be broken, this
facility would be handled by the original BUFINFO ioctl. At the next
major release, this ioctl will vanish. */

int a4l_ioctl_bufinfo2(struct a4l_device_context * cxt, void *arg)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	struct a4l_device *dev = a4l_get_dev(cxt);
	struct a4l_buffer *buf = cxt->buffer;
	a4l_bufcfg2_t buf_cfg;

	/* Basic checking */
	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_ioctl_bufcfg2: unattached device\n");
		return -EINVAL;
	}

	buf_cfg.wake_count = buf->wake_count;

	if (rtdm_safe_copy_to_user(fd,
				   arg, &buf_cfg, sizeof(a4l_bufcfg2_t)) != 0)
		return -EFAULT;

	return 0;
}

/* The function a4l_read_buffer can be considered as the kernel entry
   point of the RTDM syscall read. This syscall is supposed to be used
   only during asynchronous acquisitions */
ssize_t a4l_read_buffer(struct a4l_device_context * cxt, void *bufdata, size_t nbytes)
{
	struct a4l_device *dev = a4l_get_dev(cxt);
	struct a4l_buffer *buf = cxt->buffer;
	struct a4l_subdevice *subd = buf->subd;
	ssize_t count = 0;

	/* Basic checkings */

	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_read: unattached device\n");
		return -EINVAL;
	}

	if (!subd || !test_bit(A4L_SUBD_BUSY_NR, &subd->status)) {
		__a4l_err("a4l_read: idle subdevice on this context\n");
		return -ENOENT;
	}

	if (!a4l_subd_is_input(subd)) {
		__a4l_err("a4l_read: operation requires an input subdevice \n");
		return -EINVAL;
	}

	while (count < nbytes) {

		unsigned long tmp_cnt;

		/* Check the events */
		int ret = __handle_event(buf);

		__dump_buffer_counters(buf);

		/* Compute the data amount to copy */
		tmp_cnt = __count_to_get(buf);

		/* Check tmp_cnt count is not higher than
		   the global count to read */
		if (tmp_cnt > nbytes - count)
			tmp_cnt = nbytes - count;

		/* We check whether there is an error */
		if (ret < 0 && ret != -ENOENT) {
			__a4l_err("a4l_read: failed to handle event %d \n", ret);
			a4l_cancel_buffer(cxt);
			count = ret;
			goto out_a4l_read;
		}

		/* We check whether the acquisition is over */
		if (ret == -ENOENT && tmp_cnt == 0) {
			__a4l_info("a4l_read: acquisition done - all data "
				   "requested by the client was delivered \n");
			a4l_cancel_buffer(cxt);
			count = 0;
			goto out_a4l_read;
		}

		if (tmp_cnt > 0) {

			/* Performs the munge if need be */
			if (subd->munge != NULL) {
				__munge(subd, subd->munge, buf, tmp_cnt);

				/* Updates munge count */
				buf->mng_count += tmp_cnt;
			}

			/* Performs the copy */
			ret = __consume(cxt, buf, bufdata + count, tmp_cnt);

			if (ret < 0) {
				count = ret;
				goto out_a4l_read;
			}

			/* Updates consume count */
			buf->cns_count += tmp_cnt;
			a4l_dbg(1, core_dbg, dev, "buf->cns_cnt=%ld \n", buf->cns_count);

			/* Updates the return value */
			count += tmp_cnt;

			/* If the driver does not work in bulk mode,
			   we must leave this function */
			if (!test_bit(A4L_BUF_BULK, &buf->flags))
				goto out_a4l_read;
		}
		else {
			/* If the acquisition is not over, we must not
			   leave the function without having read a least byte */
			ret = a4l_wait_sync(&(buf->sync), rtdm_in_rt_context());
			if (ret < 0) {
				if (ret == -ERESTARTSYS)
					ret = -EINTR;
				count = ret;
				goto out_a4l_read;
			}
		}
	}

out_a4l_read:

	return count;
}

/* The function a4l_write_buffer can be considered as the kernel entry
   point of the RTDM syscall write. This syscall is supposed to be
   used only during asynchronous acquisitions */
ssize_t a4l_write_buffer(struct a4l_device_context *cxt, const void *bufdata, size_t nbytes)
{
	struct a4l_device *dev = a4l_get_dev(cxt);
	struct a4l_buffer *buf = cxt->buffer;
	struct a4l_subdevice *subd = buf->subd;
	ssize_t count = 0;

	/* Basic checkings */

	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_write: unattached device\n");
		return -EINVAL;
	}

	if (!subd || !test_bit(A4L_SUBD_BUSY_NR, &subd->status)) {
		__a4l_err("a4l_write: idle subdevice on this context\n");
		return -ENOENT;
	}

	if (!a4l_subd_is_output(subd)) {
		__a4l_err("a4l_write: operation requires an output subdevice \n");
		return -EINVAL;
	}

	while (count < nbytes) {

		unsigned long tmp_cnt;

		/* Check the events */
		int ret = __handle_event(buf);

		__dump_buffer_counters(buf);

		/* Compute the data amount to copy */
		tmp_cnt = __count_to_put(buf);

		/* Check tmp_cnt count is not higher than
		   the global count to write */
		if (tmp_cnt > nbytes - count)
			tmp_cnt = nbytes - count;

		if (ret < 0) {
			count = (ret == -ENOENT) ? -EINVAL : ret;
			__a4l_err("a4l_write: failed to handle event %d \n", ret);
			a4l_cancel_buffer(cxt);
			goto out_a4l_write;
		}

		if (tmp_cnt > 0) {


			/* Performs the copy */
			ret = __produce(cxt,
					buf, (void *)bufdata + count, tmp_cnt);
			if (ret < 0) {
				count = ret;
				goto out_a4l_write;
			}

			/* Performs the munge if need be */
			if (subd->munge != NULL) {
				__munge(subd, subd->munge, buf, tmp_cnt);

				/* Updates munge count */
				buf->mng_count += tmp_cnt;
			}

			/* Updates produce count */
			buf->prd_count += tmp_cnt;
			a4l_dbg(1, core_dbg, dev , "buf->prd_cnt=%ld \n", buf->prd_count);

			/* Updates the return value */
			count += tmp_cnt;

			/* If the driver does not work in bulk mode,
			   we must leave this function */
			if (!test_bit(A4L_BUF_BULK, &buf->flags))
				goto out_a4l_write;
		} else {
			/* The buffer is full, we have to wait for a slot to free */
			ret = a4l_wait_sync(&(buf->sync), rtdm_in_rt_context());
			if (ret < 0) {
				__a4l_err("a4l_write: failed to wait for free slot (%d)\n", ret);
				if (ret == -ERESTARTSYS)
					ret = -EINTR;
				count = ret;
				goto out_a4l_write;
			}
		}
	}

out_a4l_write:

	return count;
}

int a4l_select(struct a4l_device_context *cxt,
	       rtdm_selector_t *selector,
	       enum rtdm_selecttype type, unsigned fd_index)
{
	struct a4l_device *dev = a4l_get_dev(cxt);
	struct a4l_buffer *buf = cxt->buffer;
	struct a4l_subdevice *subd = buf->subd;

	/* Basic checkings */

	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_select: unattached device\n");
		return -EINVAL;
	}

	if (!subd || !test_bit(A4L_SUBD_BUSY, &subd->status)) {
		__a4l_err("a4l_select: idle subdevice on this context\n");
		return -ENOENT;
	}

	/* Check the RTDM select type
	   (RTDM_SELECTTYPE_EXCEPT is not supported) */

	if(type != RTDM_SELECTTYPE_READ &&
	   type != RTDM_SELECTTYPE_WRITE) {
		__a4l_err("a4l_select: wrong select argument\n");
		return -EINVAL;
	}

	if (type == RTDM_SELECTTYPE_READ && !a4l_subd_is_input(subd)) {
		__a4l_err("a4l_select: current context "
			  "does not work with an input subdevice\n");
		return -EINVAL;
	}

	if (type == RTDM_SELECTTYPE_WRITE && !a4l_subd_is_output(subd)) {
		__a4l_err("a4l_select: current context "
			  "does not work with an input subdevice\n");
		return -EINVAL;
	}

	/* Performs a bind on the Analogy synchronization element */
	return a4l_select_sync(&(buf->sync), selector, type, fd_index);
}

int a4l_ioctl_poll(struct a4l_device_context * cxt, void *arg)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	int ret = 0;
	unsigned long tmp_cnt = 0;
	struct a4l_device *dev = a4l_get_dev(cxt);
	struct a4l_buffer *buf = cxt->buffer;
	struct a4l_subdevice *subd = buf->subd;
	a4l_poll_t poll;

	if (!rtdm_in_rt_context() && rtdm_rt_capable(fd))
		return -ENOSYS;

	/* Basic checking */

	if (!test_bit(A4L_DEV_ATTACHED_NR, &dev->flags)) {
		__a4l_err("a4l_poll: unattached device\n");
		return -EINVAL;
	}

	if (!subd || !test_bit(A4L_SUBD_BUSY_NR, &subd->status)) {
		__a4l_err("a4l_poll: idle subdevice on this context\n");
		return -ENOENT;
	}

	if (rtdm_safe_copy_from_user(fd,
				     &poll, arg, sizeof(a4l_poll_t)) != 0)
		return -EFAULT;

	/* Checks the buffer events */
	a4l_flush_sync(&buf->sync);
	ret = __handle_event(buf);

	/* Retrieves the data amount to compute
	   according to the subdevice type */
	if (a4l_subd_is_input(subd)) {

		tmp_cnt = __count_to_get(buf);

		/* Check if some error occured */
		if (ret < 0 && ret != -ENOENT) {
			a4l_cancel_buffer(cxt);
			return ret;
		}

		/* Check whether the acquisition is over */
		if (ret == -ENOENT && tmp_cnt == 0) {
			a4l_cancel_buffer(cxt);
			return 0;
		}
	} else {

		/* If some error was detected, cancel the transfer */
		if (ret < 0) {
			a4l_cancel_buffer(cxt);
			return ret;
		}

		tmp_cnt = __count_to_put(buf);
	}

	if (poll.arg == A4L_NONBLOCK || tmp_cnt != 0)
		goto out_poll;

	if (poll.arg == A4L_INFINITE)
		ret = a4l_wait_sync(&(buf->sync), rtdm_in_rt_context());
	else {
		unsigned long long ns = ((unsigned long long)poll.arg) *
			((unsigned long long)NSEC_PER_MSEC);
		ret = a4l_timedwait_sync(&(buf->sync), rtdm_in_rt_context(), ns);
	}

	if (ret == 0) {
		/* Retrieves the count once more */
		if (a4l_subd_is_input(dev->transfer.subds[poll.idx_subd]))
			tmp_cnt = __count_to_get(buf);
		else
			tmp_cnt = __count_to_put(buf);
	}
	else
		return ret;

out_poll:

	poll.arg = tmp_cnt;

	ret = rtdm_safe_copy_to_user(fd,
				     arg, &poll, sizeof(a4l_poll_t));

	return ret;
}
