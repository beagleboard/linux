/**
 * @note Copyright (C) 2016 Philippe Gerum <rpm@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include "spi-master.h"

static inline
struct device *to_kdev(struct rtdm_spi_remote_slave *slave)
{
	return rtdm_dev_to_kdev(&slave->dev);
}

static inline struct rtdm_spi_remote_slave *fd_to_slave(struct rtdm_fd *fd)
{
	struct rtdm_device *dev = rtdm_fd_device(fd);

	return container_of(dev, struct rtdm_spi_remote_slave, dev);
}

static int update_slave_config(struct rtdm_spi_remote_slave *slave,
			       struct rtdm_spi_config *config)
{
	struct rtdm_spi_config old_config;
	struct rtdm_spi_master *master = slave->master;
	int ret;

	rtdm_mutex_lock(&master->bus_lock);

	old_config = slave->config;
	slave->config = *config;
	ret = slave->master->ops->configure(slave);
	if (ret) {
		slave->config = old_config;
		rtdm_mutex_unlock(&master->bus_lock);
		return ret;
	}

	rtdm_mutex_unlock(&master->bus_lock);
	
	dev_info(to_kdev(slave),
		 "configured mode %d, %s%s%s%s%u bits/w, %u Hz max\n",
		 (int) (slave->config.mode & (SPI_CPOL | SPI_CPHA)),
		 (slave->config.mode & SPI_CS_HIGH) ? "cs_high, " : "",
		 (slave->config.mode & SPI_LSB_FIRST) ? "lsb, " : "",
		 (slave->config.mode & SPI_3WIRE) ? "3wire, " : "",
		 (slave->config.mode & SPI_LOOP) ? "loopback, " : "",
		 slave->config.bits_per_word,
		 slave->config.speed_hz);
	
	return 0;
}

static int spi_master_open(struct rtdm_fd *fd, int oflags)
{
	struct rtdm_spi_remote_slave *slave = fd_to_slave(fd);
	struct rtdm_spi_master *master = slave->master;

	if (master->ops->open)
		return master->ops->open(slave);
		
	return 0;
}

static void spi_master_close(struct rtdm_fd *fd)
{
	struct rtdm_spi_remote_slave *slave = fd_to_slave(fd);
	struct rtdm_spi_master *master = slave->master;
	rtdm_lockctx_t c;

	rtdm_lock_get_irqsave(&master->lock, c);

	if (master->cs == slave)
		master->cs = NULL;

	rtdm_lock_put_irqrestore(&master->lock, c);

	if (master->ops->close)
		master->ops->close(slave);
}

static int do_chip_select(struct rtdm_spi_remote_slave *slave)
{				/* master->bus_lock held */
	struct rtdm_spi_master *master = slave->master;
	rtdm_lockctx_t c;
	int state;

	if (slave->config.speed_hz == 0)
		return -EINVAL; /* Setup is missing. */

	/* Serialize with spi_master_close() */
	rtdm_lock_get_irqsave(&master->lock, c);
	
	if (master->cs != slave) {
		if (gpio_is_valid(slave->cs_gpio)) {
			state = !!(slave->config.mode & SPI_CS_HIGH);
			gpiod_set_raw_value(slave->cs_gpiod, state);
		} else
			master->ops->chip_select(slave, true);
		master->cs = slave;
	}

	rtdm_lock_put_irqrestore(&master->lock, c);

	return 0;
}

static void do_chip_deselect(struct rtdm_spi_remote_slave *slave)
{				/* master->bus_lock held */
	struct rtdm_spi_master *master = slave->master;
	rtdm_lockctx_t c;
	int state;

	rtdm_lock_get_irqsave(&master->lock, c);

	if (gpio_is_valid(slave->cs_gpio)) {
		state = !(slave->config.mode & SPI_CS_HIGH);
		gpiod_set_raw_value(slave->cs_gpiod, state);
	} else
		master->ops->chip_select(slave, false);

	master->cs = NULL;

	rtdm_lock_put_irqrestore(&master->lock, c);
}

static int spi_master_ioctl_rt(struct rtdm_fd *fd,
			       unsigned int request, void *arg)
{
	struct rtdm_spi_remote_slave *slave = fd_to_slave(fd);
	struct rtdm_spi_master *master = slave->master;
	struct rtdm_spi_config config;
	int ret, len;

	switch (request) {
	case SPI_RTIOC_SET_CONFIG:
		ret = rtdm_safe_copy_from_user(fd, &config,
					       arg, sizeof(config));
		if (ret == 0)
			ret = update_slave_config(slave, &config);
		break;
	case SPI_RTIOC_GET_CONFIG:
		rtdm_mutex_lock(&master->bus_lock);
		config = slave->config;
		rtdm_mutex_unlock(&master->bus_lock);
		ret = rtdm_safe_copy_to_user(fd, arg,
					     &config, sizeof(config));
		break;
	case SPI_RTIOC_TRANSFER:
		ret = -EINVAL;
		if (master->ops->transfer_iobufs) {
			rtdm_mutex_lock(&master->bus_lock);
			ret = do_chip_select(slave);
			if (ret == 0) {
				ret = master->ops->transfer_iobufs(slave);
				do_chip_deselect(slave);
			}
			rtdm_mutex_unlock(&master->bus_lock);
		}
		break;
	case SPI_RTIOC_TRANSFER_N:
		ret = -EINVAL;
		if (master->ops->transfer_iobufs_n) {
			len = (int)arg;
			rtdm_mutex_lock(&master->bus_lock);
			ret = do_chip_select(slave);
			if (ret == 0) {
				ret = master->ops->transfer_iobufs_n(slave, len);
				do_chip_deselect(slave);
			}
			rtdm_mutex_unlock(&master->bus_lock);
		}
		break;
	default:
		ret = -ENOSYS;
	}

	return ret;
}

static int spi_master_ioctl_nrt(struct rtdm_fd *fd,
				unsigned int request, void *arg)
{
	struct rtdm_spi_remote_slave *slave = fd_to_slave(fd);
	struct rtdm_spi_master *master = slave->master;
	struct rtdm_spi_iobufs iobufs;
	int ret;

	switch (request) {
	case SPI_RTIOC_SET_IOBUFS:
		ret = rtdm_safe_copy_from_user(fd, &iobufs,
					       arg, sizeof(iobufs));
		if (ret)
			break;
		/*
		 * No transfer can happen without I/O buffers being
		 * set, and I/O buffers cannot be reset, therefore we
		 * need no serialization with the transfer code here.
		 */
		mutex_lock(&slave->ctl_lock);
		ret = master->ops->set_iobufs(slave, &iobufs);
		mutex_unlock(&slave->ctl_lock);
		if (ret == 0)
			ret = rtdm_safe_copy_to_user(fd, arg,
					     &iobufs, sizeof(iobufs));
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t spi_master_read_rt(struct rtdm_fd *fd,
				  void __user *u_buf, size_t len)
{
	struct rtdm_spi_remote_slave *slave = fd_to_slave(fd);
	struct rtdm_spi_master *master = slave->master;
	void *rx;
	int ret;

	if (len == 0)
		return 0;

	rx = xnmalloc(len);
	if (rx == NULL)
		return -ENOMEM;

	rtdm_mutex_lock(&master->bus_lock);
	ret = do_chip_select(slave);
	if (ret == 0) {
		ret = master->ops->read(slave, rx, len);
		do_chip_deselect(slave);
	}
	rtdm_mutex_unlock(&master->bus_lock);
	if (ret > 0)
		ret = rtdm_safe_copy_to_user(fd, u_buf, rx, ret);
	
	xnfree(rx);
	
	return ret;
}

static ssize_t spi_master_write_rt(struct rtdm_fd *fd,
				   const void __user *u_buf, size_t len)
{
	struct rtdm_spi_remote_slave *slave = fd_to_slave(fd);
	struct rtdm_spi_master *master = slave->master;
	void *tx;
	int ret;

	if (len == 0)
		return 0;

	tx = xnmalloc(len);
	if (tx == NULL)
		return -ENOMEM;

	ret = rtdm_safe_copy_from_user(fd, tx, u_buf, len);
	if (ret == 0) {
		rtdm_mutex_lock(&master->bus_lock);
		ret = do_chip_select(slave);
		if (ret == 0) {
			ret = master->ops->write(slave, tx, len);
			do_chip_deselect(slave);
		}
		rtdm_mutex_unlock(&master->bus_lock);
	}
	
	xnfree(tx);

	return ret;
}

static void iobufs_vmopen(struct vm_area_struct *vma)
{
	struct rtdm_spi_remote_slave *slave = vma->vm_private_data;

	atomic_inc(&slave->mmap_refs);
	dev_dbg(slave_to_kdev(slave), "mapping added\n");
}

static void iobufs_vmclose(struct vm_area_struct *vma)
{
	struct rtdm_spi_remote_slave *slave = vma->vm_private_data;

	if (atomic_dec_and_test(&slave->mmap_refs)) {
		slave->master->ops->mmap_release(slave);
		dev_dbg(slave_to_kdev(slave), "mapping released\n");
	}
}

static struct vm_operations_struct iobufs_vmops = {
	.open = iobufs_vmopen,
	.close = iobufs_vmclose,
};

static int spi_master_mmap(struct rtdm_fd *fd, struct vm_area_struct *vma)
{
	struct rtdm_spi_remote_slave *slave = fd_to_slave(fd);
	int ret;

	if (slave->master->ops->mmap_iobufs == NULL)
		return -EINVAL;

	ret = slave->master->ops->mmap_iobufs(slave, vma);
	if (ret)
		return ret;

	dev_dbg(slave_to_kdev(slave), "mapping created\n");
	atomic_inc(&slave->mmap_refs);

	if (slave->master->ops->mmap_release) {
		vma->vm_ops = &iobufs_vmops;
		vma->vm_private_data = slave;
	}

	return 0;
}

static char *spi_slave_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "rtdm/%s/%s",
			 dev->class->name,
			 dev_name(dev));
}

struct rtdm_spi_master *
__rtdm_spi_alloc_master(struct device *dev, size_t size, int off)
{
	struct rtdm_spi_master *master;
	struct spi_master *kmaster;

	kmaster = spi_alloc_master(dev, size);
	if (kmaster == NULL)
		return NULL;
	
	master = (void *)(kmaster + 1) + off;
	master->kmaster = kmaster;
	spi_master_set_devdata(kmaster, master);

	return master;
}
EXPORT_SYMBOL_GPL(__rtdm_spi_alloc_master);

int __rtdm_spi_setup_driver(struct rtdm_spi_master *master)
{
	master->classname = kstrdup(
		dev_name(&master->kmaster->dev), GFP_KERNEL);
	master->devclass = class_create(THIS_MODULE,
		master->classname);
	if (IS_ERR(master->devclass)) {
		kfree(master->classname);
		printk(XENO_ERR "cannot create sysfs class\n");
		return PTR_ERR(master->devclass);
	}

	master->devclass->devnode = spi_slave_devnode;
	master->cs = NULL;

	master->driver.profile_info = (struct rtdm_profile_info)
		RTDM_PROFILE_INFO(rtdm_spi_master,
				  RTDM_CLASS_SPI,
				  master->subclass,
				  0);
	master->driver.device_flags = RTDM_NAMED_DEVICE;
	master->driver.base_minor = 0;
	master->driver.device_count = 256;
	master->driver.context_size = 0;
	master->driver.ops = (struct rtdm_fd_ops){
		.open		=	spi_master_open,
		.close		=	spi_master_close,
		.read_rt	=	spi_master_read_rt,
		.write_rt	=	spi_master_write_rt,
		.ioctl_rt	=	spi_master_ioctl_rt,
		.ioctl_nrt	=	spi_master_ioctl_nrt,
		.mmap		=	spi_master_mmap,
	};
	
	rtdm_drv_set_sysclass(&master->driver, master->devclass);

	INIT_LIST_HEAD(&master->slaves);
	rtdm_lock_init(&master->lock);
	rtdm_mutex_init(&master->bus_lock);

	return 0;
}

static int spi_transfer_one_unimp(struct spi_master *master,
				  struct spi_device *spi,
				  struct spi_transfer *tfr)
{
	return -ENODEV;
}

int rtdm_spi_add_master(struct rtdm_spi_master *master)
{
	struct spi_master *kmaster = master->kmaster;

	/*
	 * Prevent the transfer handler to be called from the regular
	 * SPI stack, just in case.
	 */
	kmaster->transfer_one = spi_transfer_one_unimp;
	master->devclass = NULL;

	/*
	 * Add the core SPI driver, devices on the bus will be
	 * enumerated, handed to spi_device_probe().
	 */
	return spi_register_master(kmaster);
}
EXPORT_SYMBOL_GPL(rtdm_spi_add_master);

void rtdm_spi_remove_master(struct rtdm_spi_master *master)
{
	struct class *class = master->devclass;
	char *classname = master->classname;
	
	rtdm_mutex_destroy(&master->bus_lock);
	spi_unregister_master(master->kmaster);
	rtdm_drv_set_sysclass(&master->driver, NULL);
	class_destroy(class);
	kfree(classname);
}
EXPORT_SYMBOL_GPL(rtdm_spi_remove_master);

MODULE_LICENSE("GPL");
