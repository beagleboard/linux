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
#ifndef _RTDM_SPI_MASTER_H
#define _RTDM_SPI_MASTER_H

#include <rtdm/driver.h>
#include <rtdm/uapi/spi.h>
#include "spi-device.h"

struct class;
struct device_node;
struct rtdm_spi_master;
struct spi_master;

struct rtdm_spi_master_ops {
	int (*open)(struct rtdm_spi_remote_slave *slave);
	void (*close)(struct rtdm_spi_remote_slave *slave);
	int (*configure)(struct rtdm_spi_remote_slave *slave);
	void (*chip_select)(struct rtdm_spi_remote_slave *slave,
			    bool active);
	int (*set_iobufs)(struct rtdm_spi_remote_slave *slave,
			  struct rtdm_spi_iobufs *p);
	int (*mmap_iobufs)(struct rtdm_spi_remote_slave *slave,
			   struct vm_area_struct *vma);
	void (*mmap_release)(struct rtdm_spi_remote_slave *slave);
	int (*transfer_iobufs)(struct rtdm_spi_remote_slave *slave);
	int (*transfer_iobufs_n)(struct rtdm_spi_remote_slave *slave, int len);
	ssize_t (*write)(struct rtdm_spi_remote_slave *slave,
			 const void *tx, size_t len);
	ssize_t (*read)(struct rtdm_spi_remote_slave *slave,
			 void *rx, size_t len);
	struct rtdm_spi_remote_slave *(*attach_slave)
		(struct rtdm_spi_master *master,
			struct spi_device *spi);
	void (*detach_slave)(struct rtdm_spi_remote_slave *slave);
};

struct rtdm_spi_master {
	int subclass;
	const struct rtdm_spi_master_ops *ops;
	struct spi_master *kmaster;
	struct {	/* Internal */
		struct rtdm_driver driver;
		struct class *devclass;
		char *classname;
		struct list_head slaves;
		struct list_head next;
		rtdm_lock_t lock;
		rtdm_mutex_t bus_lock;
		struct rtdm_spi_remote_slave *cs;
	};
};

#define rtdm_spi_alloc_master(__dev, __type, __mptr)			\
	__rtdm_spi_alloc_master(__dev, sizeof(__type),			\
				offsetof(__type, __mptr))		\

struct rtdm_spi_master *
__rtdm_spi_alloc_master(struct device *dev, size_t size, int off);

int __rtdm_spi_setup_driver(struct rtdm_spi_master *master);

int rtdm_spi_add_master(struct rtdm_spi_master *master);

void rtdm_spi_remove_master(struct rtdm_spi_master *master);

#endif /* !_RTDM_SPI_MASTER_H */
