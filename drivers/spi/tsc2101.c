/*
 * linux/drivers/spi/tsc2101.c
 *
 * TSC2101 codec interface driver for the OMAP platform
 *
 * Copyright (C) 2004 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * 2004/11/07   Nishanth Menon - Modified for common hooks for Audio and Touchscreen
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2101.h>

struct tsc2101_device {
	struct mutex		mutex;
	int			mclk_enabled;
	struct clock		*mclk_ck;
	struct spi_message	message;
	struct spi_transfer	transfer[2];
	u16			command;
	void			(*enable_mclk)(struct spi_device *spi);
	void			(*disable_mclk)(struct spi_device *spi);
};

int tsc2101_enable_mclk(struct spi_device *spi)
{
	struct tsc2101_device *tsc2101;

	tsc2101 = spi_get_drvdata(spi);

	mutex_lock(&tsc2101->mutex);

	if (spi->dev.power.power_state.event != PM_EVENT_ON) {
		mutex_unlock(&tsc2101->mutex);
		return -ENODEV;
	}

	if (tsc2101->mclk_enabled++ == 0) {
		if (tsc2101->enable_mclk != NULL)
			tsc2101->enable_mclk(spi);
	}

	mutex_unlock(&tsc2101->mutex);
	return 0;
}
EXPORT_SYMBOL(tsc2101_enable_mclk);

void tsc2101_disable_mclk(struct spi_device *spi)
{
	struct tsc2101_device *tsc2101;

	tsc2101 = spi_get_drvdata(spi);

	mutex_lock(&tsc2101->mutex);

	if (--tsc2101->mclk_enabled == 0) {
		if (tsc2101->disable_mclk != NULL &&
		    spi->dev.power.power_state.event == PM_EVENT_ON)
			tsc2101->disable_mclk(spi);
	}

	mutex_lock(&tsc2101->mutex);
}
EXPORT_SYMBOL(tsc2101_disable_mclk);

int tsc2101_write_sync(struct spi_device *spi, int page, u8 address, u16 data)
{
	struct tsc2101_device *tsc2101;
	struct spi_message *m;
	struct spi_transfer *t;
	int ret;

	tsc2101 = spi_get_drvdata(spi);

	mutex_lock(&tsc2101->mutex);
	if (spi->dev.power.power_state.event != PM_EVENT_ON) {
		mutex_unlock(&tsc2101->mutex);
		return -ENODEV;
	}

	m = &tsc2101->message;
	spi_message_init(m);
	t = &tsc2101->transfer[0];
	memset(t, 0, sizeof(tsc2101->transfer));

	/* Address */
	tsc2101->command = (page << 11) | (address << 5);
	t->tx_buf = &tsc2101->command;
	t->len = 2;
	spi_message_add_tail(t, m);

	/* Data */
	t++;
	t->tx_buf = &data;
	t->len = 2;
	spi_message_add_tail(t, m);

	ret = spi_sync(spi, m);
	if (!ret)
		ret = tsc2101->message.status;
	mutex_unlock(&tsc2101->mutex);

	return ret;
}
EXPORT_SYMBOL(tsc2101_write_sync);

int tsc2101_reads_sync(struct spi_device *spi,
		       int page, u8 startaddress, u16 *data, int numregs)
{
	struct tsc2101_device *tsc2101;
	struct spi_message *m;
	struct spi_transfer *t;
	int ret;

	tsc2101 = spi_get_drvdata(spi);

	mutex_lock(&tsc2101->mutex);
	if (spi->dev.power.power_state.event != PM_EVENT_ON) {
		mutex_unlock(&tsc2101->mutex);
		return -ENODEV;
	}

	m = &tsc2101->message;
	spi_message_init(m);
	t = &tsc2101->transfer[0];
	memset(t, 0, sizeof(tsc2101->transfer));

	/* Address */
	tsc2101->command = 0x8000 | (page << 11) | (startaddress << 5);
	t->tx_buf = &tsc2101->command;
	t->len = 2;
	spi_message_add_tail(t, m);

	/* Data */
	t++;
	t->rx_buf = data;
	t->len = numregs << 1;
	spi_message_add_tail(t, m);

	ret = spi_sync(spi, m);
	if (!ret)
		ret = tsc2101->message.status;

	mutex_unlock(&tsc2101->mutex);

	return ret;
}
EXPORT_SYMBOL(tsc2101_reads_sync);

int tsc2101_read_sync(struct spi_device *spi, int page, u8 address)
{
	int err;
	u16 val;

	err = tsc2101_reads_sync(spi, page, address, &val, 1);
	if (err)
		return err;
	return val;
}
EXPORT_SYMBOL(tsc2101_read_sync);

static int tsc2101_suspend(struct spi_device *spi, pm_message_t state)
{
	struct tsc2101_device *tsc2101;

	tsc2101 = spi_get_drvdata(spi);

	if (tsc2101 == NULL)
		return 0;

	mutex_lock(&tsc2101->mutex);

	spi->dev.power.power_state = state;
	if (tsc2101->mclk_enabled && tsc2101->disable_mclk != NULL)
		tsc2101->disable_mclk(spi);

	mutex_unlock(&tsc2101->mutex);

	return 0;
}

static int tsc2101_resume(struct spi_device *spi)
{
	struct tsc2101_device *tsc2101;

	tsc2101 = spi_get_drvdata(spi);

	if (tsc2101 == NULL)
		return 0;

	mutex_lock(&tsc2101->mutex);

	spi->dev.power.power_state = PMSG_ON;
	if (tsc2101->mclk_enabled && tsc2101->enable_mclk != NULL)
		tsc2101->enable_mclk(spi);

	mutex_unlock(&tsc2101->mutex);

	return 0;
}

static int tsc2101_probe(struct spi_device *spi)
{
	struct tsc2101_platform_data *pdata;
	struct tsc2101_device *tsc2101;
	u16 w;
	int r;

	pdata = spi->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&spi->dev, "no platform data\n");
		return -ENODEV;
	}

	tsc2101 = kzalloc(sizeof(*tsc2101), GFP_KERNEL);
	if (tsc2101 == NULL) {
		dev_err(&spi->dev, "out of mem\n");
		return -ENOMEM;
	}

	spi_set_drvdata(spi, tsc2101);
	tsc2101->enable_mclk = pdata->enable_mclk;
	tsc2101->disable_mclk = pdata->disable_mclk;

	mutex_init(&tsc2101->mutex);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 16;
	if ((r = spi_setup(spi)) < 0) {
		dev_err(&spi->dev, "SPI setup failed\n");
		goto err;
	}

	w = tsc2101_read_sync(spi, 1, 0);
	if (!(w & (1 << 14))) {
		dev_err(&spi->dev, "invalid ADC register value %04x\n", w);
		goto err;
	}

	if (pdata->init != NULL) {
		if ((r = pdata->init(spi)) < 0) {
			dev_err(&spi->dev, "platform init failed\n");
			goto err;
		}
	}

	dev_info(&spi->dev, "initialized\n");

	return 0;
err:
	kfree(tsc2101);
	return r;
}

static int tsc2101_remove(struct spi_device *spi)
{
	struct tsc2101_platform_data *pdata;
	struct tsc2101_device *tsc2101;

	pdata = spi->dev.platform_data;
	tsc2101 = spi_get_drvdata(spi);

	/* We assume that this can't race with the rest of the driver. */
	if (tsc2101->mclk_enabled && tsc2101->disable_mclk != NULL)
		tsc2101->disable_mclk(spi);

	if (pdata->cleanup != NULL)
		pdata->cleanup(spi);

	spi_set_drvdata(spi, NULL);
	kfree(tsc2101);

	return 0;
}

static struct spi_driver tsc2101_driver = {
	.probe		= tsc2101_probe,
	.remove		= tsc2101_remove,
	.suspend	= tsc2101_suspend,
	.resume		= tsc2101_resume,
	.driver		= {
		.name	= "tsc2101",
		.owner	= THIS_MODULE,
	},
};

static int tsc2101_init(void)
{
	return spi_register_driver(&tsc2101_driver);
}

static void tsc2101_exit(void)
{
	spi_unregister_driver(&tsc2101_driver);
}

module_init(tsc2101_init);
module_exit(tsc2101_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION
    ("Glue audio driver for the TI OMAP1610/OMAP1710 TSC2101 codec.");
MODULE_LICENSE("GPL");
