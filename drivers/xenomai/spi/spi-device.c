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
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include "spi-master.h"

int rtdm_spi_add_remote_slave(struct rtdm_spi_remote_slave *slave,
			      struct rtdm_spi_master *master,
			      struct spi_device *spi)
{
	struct spi_master *kmaster = master->kmaster;
	struct rtdm_device *dev;
	rtdm_lockctx_t c;
	int ret;

	memset(slave, 0, sizeof(*slave));
	slave->chip_select = spi->chip_select;
	slave->config.bits_per_word = spi->bits_per_word;
	slave->config.speed_hz = spi->max_speed_hz;
	slave->config.mode = spi->mode;
	slave->master = master;
	
	dev = &slave->dev;
	dev->driver = &master->driver;
	dev->label = kasprintf(GFP_KERNEL, "%s/slave%d.%%d",
			       dev_name(&kmaster->dev),
			       kmaster->bus_num);
	if (dev->label == NULL)
		return -ENOMEM;

	if (gpio_is_valid(spi->cs_gpio))
		slave->cs_gpio = spi->cs_gpio;
	else {
		slave->cs_gpio = -ENOENT;
		if (kmaster->cs_gpios)
			slave->cs_gpio = kmaster->cs_gpios[spi->chip_select];
	}

	if (gpio_is_valid(slave->cs_gpio)) {
		ret = gpio_request(slave->cs_gpio, dev->label);
		if (ret)
			goto fail;
		slave->cs_gpiod = gpio_to_desc(slave->cs_gpio);
		if (slave->cs_gpiod == NULL)
			goto fail;
	}
	
	mutex_init(&slave->ctl_lock);

	dev->device_data = master;
	ret = rtdm_dev_register(dev);
	if (ret)
		goto fail;

	rtdm_lock_get_irqsave(&master->lock, c);
	list_add_tail(&slave->next, &master->slaves);
	rtdm_lock_put_irqrestore(&master->lock, c);

	return 0;
fail:
	kfree(dev->label);

	return ret;
}
EXPORT_SYMBOL_GPL(rtdm_spi_add_remote_slave);

void rtdm_spi_remove_remote_slave(struct rtdm_spi_remote_slave *slave)
{
	struct rtdm_spi_master *master = slave->master;
	struct rtdm_device *dev;
	rtdm_lockctx_t c;
	
	if (gpio_is_valid(slave->cs_gpio))
		gpio_free(slave->cs_gpio);

	mutex_destroy(&slave->ctl_lock);
	rtdm_lock_get_irqsave(&master->lock, c);
	list_del(&slave->next);
	rtdm_lock_put_irqrestore(&master->lock, c);
	dev = &slave->dev;
	rtdm_dev_unregister(dev);
	kfree(dev->label);
}
EXPORT_SYMBOL_GPL(rtdm_spi_remove_remote_slave);

static int spi_device_probe(struct spi_device *spi)
{
	struct rtdm_spi_remote_slave *slave;
	struct rtdm_spi_master *master;
	int ret;

	/*
	 * Chicken and egg issue: we want the RTDM device class name
	 * to duplicate the SPI master name, but that information is
	 * only available after spi_register_master() has returned. We
	 * solve this by initializing the RTDM driver descriptor on
	 * the fly when the first SPI device on the bus is advertised
	 * on behalf of spi_register_master().
	 *
	 * NOTE: the driver core guarantees serialization.
	 */
	master = spi_master_get_devdata(spi->master);
	if (master->devclass == NULL) {
		ret = __rtdm_spi_setup_driver(master);
		if (ret)
			return ret;
	}

	slave = master->ops->attach_slave(master, spi);
	if (IS_ERR(slave))
		return PTR_ERR(slave);

	spi_set_drvdata(spi, slave);

	return 0;
}

static int spi_device_remove(struct spi_device *spi)
{
	struct rtdm_spi_remote_slave *slave = spi_get_drvdata(spi);

	slave->master->ops->detach_slave(slave);

	return 0;
}

static const struct of_device_id spi_device_match[] = {
	{
		.compatible = "rtdm-spidev",
	},
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, spi_device_match);

static struct spi_driver spi_device_driver = {
	.driver = {
		.name =	"rtdm_spi_device",
		.owner = THIS_MODULE,
		.of_match_table = spi_device_match,
	},
	.probe	= spi_device_probe,
	.remove	= spi_device_remove,
};

static int __init spi_device_init(void)
{
	int ret;

	ret = spi_register_driver(&spi_device_driver);

	return ret;
}
module_init(spi_device_init);

static void __exit spi_device_exit(void)
{
	spi_unregister_driver(&spi_device_driver);

}
module_exit(spi_device_exit);
