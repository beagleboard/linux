/*
 * Analogy for Linux, user interface (open, read, write, ioctl, proc)
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
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <rtdm/driver.h>
#include <rtdm/analogy/device.h>

MODULE_AUTHOR("Alexis Berlemont");
MODULE_DESCRIPTION("Analogy core driver");
MODULE_LICENSE("GPL");

int (* const a4l_ioctl_functions[]) (struct a4l_device_context *, void *) = {
	[_IOC_NR(A4L_DEVCFG)] = a4l_ioctl_devcfg,
	[_IOC_NR(A4L_DEVINFO)] = a4l_ioctl_devinfo,
	[_IOC_NR(A4L_SUBDINFO)] = a4l_ioctl_subdinfo,
	[_IOC_NR(A4L_CHANINFO)] = a4l_ioctl_chaninfo,
	[_IOC_NR(A4L_RNGINFO)] = a4l_ioctl_rnginfo,
	[_IOC_NR(A4L_CMD)] = a4l_ioctl_cmd,
	[_IOC_NR(A4L_CANCEL)] = a4l_ioctl_cancel,
	[_IOC_NR(A4L_INSNLIST)] = a4l_ioctl_insnlist,
	[_IOC_NR(A4L_INSN)] = a4l_ioctl_insn,
	[_IOC_NR(A4L_BUFCFG)] = a4l_ioctl_bufcfg,
	[_IOC_NR(A4L_BUFINFO)] = a4l_ioctl_bufinfo,
	[_IOC_NR(A4L_POLL)] = a4l_ioctl_poll,
	[_IOC_NR(A4L_MMAP)] = a4l_ioctl_mmap,
	[_IOC_NR(A4L_NBCHANINFO)] = a4l_ioctl_nbchaninfo,
	[_IOC_NR(A4L_NBRNGINFO)] = a4l_ioctl_nbrnginfo,
	[_IOC_NR(A4L_BUFCFG2)] = a4l_ioctl_bufcfg2,
	[_IOC_NR(A4L_BUFINFO2)] = a4l_ioctl_bufinfo2
};

#ifdef CONFIG_PROC_FS
struct proc_dir_entry *a4l_proc_root;

static int a4l_proc_devs_open(struct inode *inode, struct file *file)
{
	return single_open(file, a4l_rdproc_devs, NULL);
}

static const struct file_operations a4l_proc_devs_ops = {
	.open		= a4l_proc_devs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int a4l_proc_drvs_open(struct inode *inode, struct file *file)
{
	return single_open(file, a4l_rdproc_drvs, NULL);
}

static const struct file_operations a4l_proc_drvs_ops = {
	.open		= a4l_proc_drvs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int a4l_init_proc(void)
{
	int ret = 0;
	struct proc_dir_entry *entry;

	/* Creates the global directory */
	a4l_proc_root = proc_mkdir("analogy", NULL);
	if (a4l_proc_root == NULL) {
		__a4l_err("a4l_proc_init: "
			  "failed to create /proc/analogy\n");
		return -ENOMEM;
	}

	/* Creates the devices related file */
	entry = proc_create("devices", 0444, a4l_proc_root,
			    &a4l_proc_devs_ops);
	if (entry == NULL) {
		__a4l_err("a4l_proc_init: "
			  "failed to create /proc/analogy/devices\n");
		ret = -ENOMEM;
		goto err_proc_init;
	}

	/* Creates the drivers related file */
	entry = proc_create("drivers", 0444, a4l_proc_root,
			    &a4l_proc_drvs_ops);
	if (entry == NULL) {
		__a4l_err("a4l_proc_init: "
			  "failed to create /proc/analogy/drivers\n");
		ret = -ENOMEM;
		goto err_proc_init;
	}

	return 0;

err_proc_init:
	remove_proc_entry("devices", a4l_proc_root);
	remove_proc_entry("analogy", NULL);
	return ret;
}

void a4l_cleanup_proc(void)
{
	remove_proc_entry("drivers", a4l_proc_root);
	remove_proc_entry("devices", a4l_proc_root);
	remove_proc_entry("analogy", NULL);
}

#else /* !CONFIG_PROC_FS */

#define a4l_init_proc() 0
#define a4l_cleanup_proc()

#endif /* CONFIG_PROC_FS */

int a4l_open(struct rtdm_fd *fd, int flags)
{
	struct a4l_device_context *cxt = (struct a4l_device_context *)rtdm_fd_to_private(fd);

	/* Get a pointer on the selected device (thanks to minor index) */
	a4l_set_dev(cxt);

	/* Initialize the buffer structure */
	cxt->buffer = rtdm_malloc(sizeof(struct a4l_buffer));

	a4l_init_buffer(cxt->buffer);
	/* Allocate the asynchronous buffer
	   NOTE: it should be interesting to allocate the buffer only
	   on demand especially if the system is short of memory */
	if (cxt->dev->transfer.default_bufsize)
		a4l_alloc_buffer(cxt->buffer,
				 cxt->dev->transfer.default_bufsize);

	__a4l_dbg(1, core_dbg, "cxt=%p cxt->buf=%p, cxt->buf->buf=%p\n",
		cxt, cxt->buffer, cxt->buffer->buf);

	return 0;
}

void a4l_close(struct rtdm_fd *fd)
{
	struct a4l_device_context *cxt = (struct a4l_device_context *)rtdm_fd_to_private(fd);

	/* Cancel the maybe occuring asynchronous transfer */
	a4l_cancel_buffer(cxt);

	/* Free the buffer which was linked with this context and... */
	a4l_free_buffer(cxt->buffer);

	/* ...free the other buffer resources (sync) and... */
	a4l_cleanup_buffer(cxt->buffer);

	/* ...free the structure */
	rtdm_free(cxt->buffer);
}

ssize_t a4l_read(struct rtdm_fd *fd, void *buf, size_t nbytes)
{
	struct a4l_device_context *cxt = (struct a4l_device_context *)rtdm_fd_to_private(fd);

	/* Jump into the RT domain if possible */
	if (!rtdm_in_rt_context() && rtdm_rt_capable(fd))
		return -ENOSYS;

	if (nbytes == 0)
		return 0;

	return a4l_read_buffer(cxt, buf, nbytes);
}

ssize_t a4l_write(struct rtdm_fd *fd, const void *buf, size_t nbytes)
{
	struct a4l_device_context *cxt = (struct a4l_device_context *)rtdm_fd_to_private(fd);

	/* Jump into the RT domain if possible */
	if (!rtdm_in_rt_context() && rtdm_rt_capable(fd))
		return -ENOSYS;

	if (nbytes == 0)
		return 0;

	return a4l_write_buffer(cxt, buf, nbytes);
}

int a4l_ioctl(struct rtdm_fd *fd, unsigned int request, void *arg)
{
	struct a4l_device_context *cxt = (struct a4l_device_context *)rtdm_fd_to_private(fd);

	return a4l_ioctl_functions[_IOC_NR(request)] (cxt, arg);
}

int a4l_rt_select(struct rtdm_fd *fd,
		  rtdm_selector_t *selector,
		  enum rtdm_selecttype type, unsigned fd_index)
{
	struct a4l_device_context *cxt = (struct a4l_device_context *)rtdm_fd_to_private(fd);

	return a4l_select(cxt, selector, type, fd_index);
}

static struct rtdm_driver analogy_driver = {
	.profile_info =		RTDM_PROFILE_INFO(analogy,
						  RTDM_CLASS_EXPERIMENTAL,
						  RTDM_SUBCLASS_ANALOGY,
						  0),
	.device_flags =		RTDM_NAMED_DEVICE,
	.device_count =		A4L_NB_DEVICES,
	.context_size =		sizeof(struct a4l_device_context),
	.ops = {
		.open =		a4l_open,
		.close =	a4l_close,
		.ioctl_rt =	a4l_ioctl,
		.read_rt =	a4l_read,
		.write_rt =	a4l_write,
		.ioctl_nrt =	a4l_ioctl,
		.read_nrt =	a4l_read,
		.write_nrt =	a4l_write,
		.select =	a4l_rt_select,
	},
};

static struct rtdm_device rtdm_devs[A4L_NB_DEVICES] = {
	[0 ... A4L_NB_DEVICES - 1] = {
		.driver = &analogy_driver,
		.label = "analogy%d",
	}
};

int a4l_register(void)
{
	int i, ret;

	for (i = 0; i < A4L_NB_DEVICES; i++) {
		ret = rtdm_dev_register(rtdm_devs + i);
		if (ret)
			goto fail;
	}

	return 0;
fail:
	while (i-- > 0)
		rtdm_dev_unregister(rtdm_devs + i);

	return ret;
}

void a4l_unregister(void)
{
	int i;
	for (i = 0; i < A4L_NB_DEVICES; i++)
		rtdm_dev_unregister(&(rtdm_devs[i]));
}

static int __init a4l_init(void)
{
	int ret;

	if (!realtime_core_enabled())
		return 0;

	/* Initializes the devices */
	a4l_init_devs();

	/* Initializes Analogy time management */
	a4l_init_time();

	/* Registers RTDM / fops interface */
	ret = a4l_register();
	if (ret != 0) {
		a4l_unregister();
		goto out_a4l_init;
	}

	/* Initializes Analogy proc layer */
	ret = a4l_init_proc();

out_a4l_init:
	return ret;
}

static void __exit a4l_cleanup(void)
{
	if (!realtime_core_enabled())
		return;

	/* Removes Analogy proc files */
	a4l_cleanup_proc();

	/* Unregisters RTDM / fops interface */
	a4l_unregister();
}

module_init(a4l_init);
module_exit(a4l_cleanup);
