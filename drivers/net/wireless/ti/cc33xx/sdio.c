// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2009-2010 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#include <linux/mmc/sdio_func.h>
#include <linux/mmc/host.h>
#include <linux/gpio.h>
#include <linux/pm_runtime.h>
#include <linux/of_irq.h>

#include "wlcore.h"
#include "io.h"


#ifndef SDIO_VENDOR_ID_TI
#define SDIO_VENDOR_ID_TI		0x0097
#endif

#define SDIO_DEVICE_ID_CC33XX_NO_EFUSE 	0x4076
#define SDIO_DEVICE_ID_TI_CC33XX	0x4077


static bool dump = false;

struct cc33xx_sdio_glue {
	struct device *dev;
	struct platform_device *core;
};

static const struct sdio_device_id cc33xx_devices[] = {
	{ SDIO_DEVICE(SDIO_VENDOR_ID_TI, SDIO_DEVICE_ID_TI_CC33XX) },
	{ SDIO_DEVICE(SDIO_VENDOR_ID_TI, SDIO_DEVICE_ID_CC33XX_NO_EFUSE) },
	{}
};
MODULE_DEVICE_TABLE(sdio, cc33xx_devices);

static void cc33xx_sdio_claim(struct device *child)
{
	struct cc33xx_sdio_glue *glue = dev_get_drvdata(child->parent);
	struct sdio_func *func = dev_to_sdio_func(glue->dev);

	sdio_claim_host(func);
}

static void cc33xx_sdio_release(struct device *child)
{
	struct cc33xx_sdio_glue *glue = dev_get_drvdata(child->parent);
	struct sdio_func *func = dev_to_sdio_func(glue->dev);

	sdio_release_host(func);
}

static void cc33xx_sdio_set_block_size(struct device *child,
				       unsigned int blksz)
{
	struct cc33xx_sdio_glue *glue = dev_get_drvdata(child->parent);
	struct sdio_func *func = dev_to_sdio_func(glue->dev);

	sdio_claim_host(func);
	sdio_set_block_size(func, blksz);
	sdio_release_host(func);
}

static int __must_check cc33xx_sdio_raw_read(struct device *child, int addr,
					     void *buf, size_t len, bool fixed)
{
	int ret;
	struct cc33xx_sdio_glue *glue = dev_get_drvdata(child->parent);
	struct sdio_func *func = dev_to_sdio_func(glue->dev);

	sdio_claim_host(func);

	if (unlikely(addr == HW_ACCESS_ELP_CTRL_REG)) {
		((u8 *)buf)[0] = sdio_f0_readb(func, addr, &ret);
		dev_dbg(child->parent, "sdio read 52 addr 0x%x, byte 0x%02x\n",
			addr, ((u8 *)buf)[0]);
	} else {
		if (fixed)
			ret = sdio_readsb(func, buf, addr, len);
		else
			ret = sdio_memcpy_fromio(func, buf, addr, len);

		dev_dbg(child->parent, "sdio read 53 addr 0x%x, %zu bytes\n",
			addr, len);
	}

	sdio_release_host(func);

	if (WARN_ON(ret))
		dev_err(child->parent, "sdio read failed (%d)\n", ret);

	if (unlikely(dump)) {
		printk(KERN_DEBUG "wlcore_sdio: READ from 0x%04x\n", addr);
		print_hex_dump(KERN_DEBUG, "wlcore_sdio: READ ",
			       DUMP_PREFIX_OFFSET, 16, 1, buf, len, false);
	}

	return ret;
}

static int __must_check cc33xx_sdio_raw_write(struct device *child, int addr,
					      void *buf, size_t len, bool fixed)
{
	int ret;
	struct cc33xx_sdio_glue *glue = dev_get_drvdata(child->parent);
	struct sdio_func *func = dev_to_sdio_func(glue->dev);

	sdio_claim_host(func);

	if (unlikely(dump)) {
		printk(KERN_DEBUG "wlcore_sdio: "
		       "WRITE to 0x%04x length 0x%x (first 64 Bytes):\n",
		       addr, len);
		print_hex_dump(KERN_DEBUG, "wlcore_sdio: WRITE ",
			       DUMP_PREFIX_OFFSET,16, 1, buf,
			       min(len, (size_t)64), false);
	}

	if (unlikely(addr == HW_ACCESS_ELP_CTRL_REG)) {
		sdio_f0_writeb(func, ((u8 *)buf)[0], addr, &ret);
		dev_dbg(child->parent, "sdio write 52 addr 0x%x, byte 0x%02x\n",
			addr, ((u8 *)buf)[0]);
	} else {
		dev_dbg(child->parent, "sdio write 53 addr 0x%x, %zu bytes\n",
			addr, len);

		if (fixed)
			ret = sdio_writesb(func, addr, buf, len);
		else
			ret = sdio_memcpy_toio(func, addr, buf, len);
	}

	sdio_release_host(func);

	if (WARN_ON(ret))
		dev_err(child->parent, "sdio write failed (%d)\n", ret);

	return ret;
}

static int cc33xx_sdio_power_on(struct cc33xx_sdio_glue *glue)
{
	int ret;
	struct sdio_func *func = dev_to_sdio_func(glue->dev);
	struct mmc_card *card = func->card;

	ret = pm_runtime_get_sync(&card->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(&card->dev);
		dev_err(glue->dev, "%s: failed to get_sync(%d)\n",
			__func__, ret);

		return ret;
	}

	sdio_claim_host(func);
	sdio_enable_func(func);
	sdio_release_host(func);

	return 0;
}

static int cc33xx_sdio_power_off(struct cc33xx_sdio_glue *glue)
{
	struct sdio_func *func = dev_to_sdio_func(glue->dev);
	struct mmc_card *card = func->card;

	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);

	/* Let runtime PM know the card is powered off */
	pm_runtime_put(&card->dev);
	return 0;
}

static int cc33xx_sdio_set_power(struct device *child, bool enable)
{
	struct cc33xx_sdio_glue *glue = dev_get_drvdata(child->parent);

	if (enable)
		return cc33xx_sdio_power_on(glue);
	else
		return cc33xx_sdio_power_off(glue);
}

/**
 *	inband_irq_handler - Called from the MMC subsystem when the
 *	function's IRQ is signaled.
 *	@func: an SDIO function of the card
 *
 *	Note that the host is already claimed when handler is invoked.
 */
static void inband_irq_handler(struct sdio_func *func)
{
	struct cc33xx_sdio_glue *glue = sdio_get_drvdata(func);
	struct platform_device *pdev = glue->core;
	struct wlcore_platdev_data *pdev_data = dev_get_platdata(&pdev->dev);

	dev_dbg(glue->dev, "Inband SDIO IRQ");

	BUG_ON(!pdev_data->irq_handler);
	pdev_data->irq_handler(pdev);
}

static void cc33xx_enable_async_interrupt(struct sdio_func *func)
{
	uint8_t regVal;
	const int CCCR_REG_16_ADDR = 0x16;
	const int ENABLE_ASYNC_IRQ_BIT = BIT(1);

	regVal = sdio_f0_readb(func, CCCR_REG_16_ADDR, NULL);
	regVal |= ENABLE_ASYNC_IRQ_BIT;
	sdio_f0_writeb(func, regVal, CCCR_REG_16_ADDR, NULL);
}

static void cc33xx_sdio_enable_irq(struct device *child)
{
	struct cc33xx_sdio_glue *glue = dev_get_drvdata(child->parent);
	struct sdio_func *func = dev_to_sdio_func(glue->dev);

	sdio_claim_host(func);
	cc33xx_enable_async_interrupt(func);
	sdio_claim_irq(func, inband_irq_handler);
	sdio_release_host(func);
}

static void cc33xx_sdio_disable_irq(struct device *child)
{
	struct cc33xx_sdio_glue *glue = dev_get_drvdata(child->parent);
	struct sdio_func *func = dev_to_sdio_func(glue->dev);

	sdio_claim_host(func);
	sdio_release_irq(func);
	sdio_release_host(func);
}

static void cc33xx_enable_line_irq(struct device *child)
{
	struct cc33xx_sdio_glue *glue = dev_get_drvdata(child->parent);
	struct platform_device *pdev = glue->core;
	struct wlcore_platdev_data *pdev_data = dev_get_platdata(&pdev->dev);

	enable_irq(pdev_data->gpio_irq_num);
}

static void cc33xx_disable_line_irq(struct device *child)
{
	struct cc33xx_sdio_glue *glue = dev_get_drvdata(child->parent);
	struct platform_device *pdev = glue->core;
	struct wlcore_platdev_data *pdev_data = dev_get_platdata(&pdev->dev);

	disable_irq_nosync(pdev_data->gpio_irq_num);
}

static void cc33xx_set_irq_handler(struct device *child, void* handler)
{
	struct cc33xx_sdio_glue *glue = dev_get_drvdata(child->parent);
	struct platform_device *pdev = glue->core;
	struct wlcore_platdev_data *pdev_data = dev_get_platdata(&pdev->dev);

	pdev_data->irq_handler = handler;
}

static struct cc33xx_if_operations sdio_ops_gpio_irq = {
	.interface_claim	= cc33xx_sdio_claim,
	.interface_release 	= cc33xx_sdio_release,
	.read			= cc33xx_sdio_raw_read,
	.write			= cc33xx_sdio_raw_write,
	.power			= cc33xx_sdio_set_power,
	.set_block_size 	= cc33xx_sdio_set_block_size,
	.set_irq_handler	= cc33xx_set_irq_handler,
	.disable_irq		= cc33xx_disable_line_irq,
	.enable_irq		= cc33xx_enable_line_irq,
};

static struct cc33xx_if_operations sdio_ops_inband_irq = {
	.interface_claim	= cc33xx_sdio_claim,
	.interface_release 	= cc33xx_sdio_release,
	.read			= cc33xx_sdio_raw_read,
	.write			= cc33xx_sdio_raw_write,
	.power			= cc33xx_sdio_set_power,
	.set_block_size 	= cc33xx_sdio_set_block_size,
	.set_irq_handler	= cc33xx_set_irq_handler,
	.disable_irq		= cc33xx_sdio_disable_irq,
	.enable_irq		= cc33xx_sdio_enable_irq,
};

#ifdef CONFIG_OF
static const struct cc33xx_family_data cc33xx_data = {
	.name = "cc33xx",
	.cfg_name = "ti-connectivity/cc33xx-conf.bin",
	.nvs_name = "ti-connectivity/cc33xx-nvs.bin",
};

static const struct of_device_id wlcore_sdio_of_match_table[] = {
	{ .compatible = "ti,cc33xx", .data = &cc33xx_data },
	{ }
};

static int wlcore_probe_of(struct device *dev, int *irq, int *wakeirq,
			   struct wlcore_platdev_data *pdev_data)
{
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_id;

	of_id = of_match_node(wlcore_sdio_of_match_table, np);
	if (!of_id)
		return -ENODEV;

	pdev_data->family = of_id->data;

	*irq = irq_of_parse_and_map(np, 0);

	*wakeirq = irq_of_parse_and_map(np, 1);

	return 0;
}
#else
static int wlcore_probe_of(struct device *dev, int *irq, int *wakeirq,
			   struct wlcore_platdev_data *pdev_data)
{
	return -ENODATA;
}
#endif /* CONFIG_OF */

static irqreturn_t gpio_irq_hard_handler(int irq, void *cookie)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t gpio_irq_thread_handler(int irq, void *cookie)
{
	struct sdio_func *func = cookie;
	struct cc33xx_sdio_glue *glue = sdio_get_drvdata(func);
	struct platform_device *pdev = glue->core;
	struct wlcore_platdev_data *pdev_data = dev_get_platdata(&pdev->dev);

	BUG_ON(!pdev_data->irq_handler);

	pdev_data->irq_handler(pdev);

	return IRQ_HANDLED;
}

static int sdio_cc33xx_probe(struct sdio_func *func,
				  const struct sdio_device_id *id)
{
	struct wlcore_platdev_data *pdev_data;
	struct cc33xx_sdio_glue *glue;
	struct resource res[1];
	mmc_pm_flag_t mmcflags;
	int ret = -ENOMEM;
	int gpio_irq, wakeirq, irq_flags;
	const char *chip_family;

	/* We are only able to handle the wlan function */
	if (func->num != 0x02)
		return -ENODEV;

	pdev_data = devm_kzalloc(&func->dev, sizeof(*pdev_data), GFP_KERNEL);
	if (!pdev_data)
		return -ENOMEM;

	glue = devm_kzalloc(&func->dev, sizeof(*glue), GFP_KERNEL);
	if (!glue)
		return -ENOMEM;

	glue->dev = &func->dev;

	/* Grab access to FN0 for ELP reg. */
	func->card->quirks |= MMC_QUIRK_LENIENT_FN0;

	/* Use block mode for transferring over one block size of data */
	func->card->quirks |= MMC_QUIRK_BLKSZ_FOR_BYTE_MODE;

	ret = wlcore_probe_of(&func->dev, &gpio_irq, &wakeirq, pdev_data);
	if (ret)
		goto out;

	/* if sdio can keep power while host is suspended, enable wow */
	mmcflags = sdio_get_host_pm_caps(func);
	dev_dbg(glue->dev, "sdio PM caps = 0x%x\n", mmcflags);

	sdio_set_drvdata(func, glue);

	/* Tell PM core that we don't need the card to be powered now */
	pm_runtime_put_noidle(&func->dev);

	chip_family = "cc33xx";

	glue->core = platform_device_alloc(chip_family, PLATFORM_DEVID_AUTO);
	if (!glue->core) {
		dev_err(glue->dev, "can't allocate platform_device");
		ret = -ENOMEM;
		goto out;
	}

	glue->core->dev.parent = &func->dev;

	if (gpio_irq)
	{
		dev_info(glue->dev, "Using GPIO as IRQ\n");

		irq_flags = irqd_get_trigger_type(irq_get_irq_data(gpio_irq));

		irq_set_status_flags(gpio_irq, IRQ_NOAUTOEN);

		if (irq_flags & (IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW))
			irq_flags |= IRQF_ONESHOT;

		ret = request_threaded_irq(gpio_irq, gpio_irq_hard_handler,
					   gpio_irq_thread_handler,
					   irq_flags, glue->core->name, func);
		if (ret) {
			dev_err(glue->dev, "can't register GPIO IRQ handler\n");
			goto out_dev_put;
		}

		pdev_data->gpio_irq_num = gpio_irq;

		if ((mmcflags & MMC_PM_KEEP_POWER) &&
		    (enable_irq_wake(gpio_irq)==0))
			pdev_data->pwr_in_suspend = true;

		pdev_data->if_ops = &sdio_ops_gpio_irq;
	}
	else
	{
		dev_info(glue->dev, "Using SDIO in-band IRQ\n");

		pdev_data->if_ops = &sdio_ops_inband_irq;
	}

	if (wakeirq > 0) {
		res[0].start = wakeirq;
		res[0].flags = IORESOURCE_IRQ |
			irqd_get_trigger_type(irq_get_irq_data(wakeirq));
		res[0].name = "wakeirq";

		ret = platform_device_add_resources(glue->core, res, 1);
		if (ret) {
			dev_err(glue->dev, "can't add resources\n");
			goto out_dev_put;
		}
	}

	ret = platform_device_add_data(glue->core, pdev_data,
				       sizeof(*pdev_data));
	if (ret) {
		dev_err(glue->dev, "can't add platform data\n");
		goto out_dev_put;
	}

	ret = platform_device_add(glue->core);
	if (ret) {
		dev_err(glue->dev, "can't add platform device\n");
		goto out_dev_put;
	}
	return 0;

out_dev_put:
	platform_device_put(glue->core);

out:
	return ret;
}

static void sdio_cc33xx_remove(struct sdio_func *func)
{
	struct cc33xx_sdio_glue *glue = sdio_get_drvdata(func);
	struct platform_device *pdev = glue->core;
	struct wlcore_platdev_data *pdev_data = dev_get_platdata(&pdev->dev);

	/* Undo decrement done above in sdio_cc33xx_probe */
	pm_runtime_get_noresume(&func->dev);

	platform_device_unregister(glue->core);

	if (pdev_data->gpio_irq_num){
		free_irq(pdev_data->gpio_irq_num, func);
		if (pdev_data->pwr_in_suspend)
			disable_irq_wake(pdev_data->gpio_irq_num);
	}
	else{
		sdio_claim_host(func);
		sdio_release_irq(func);
		sdio_release_host(func);
	}
}

#ifdef CONFIG_PM
static int cc33xx_suspend(struct device *dev)
{
	/* Tell MMC/SDIO core it's OK to power down the card
	 * (if it isn't already), but not to remove it completely */
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct cc33xx_sdio_glue *glue = sdio_get_drvdata(func);
	struct cc33xx *wl = platform_get_drvdata(glue->core);
	mmc_pm_flag_t sdio_flags;
	int ret = 0;

	if (!wl) {
		dev_err(dev, "no wilink module was probed\n");
		goto out;
	}

	dev_dbg(dev, "cc33xx suspend. keep_device_power: %d\n",
		wl->keep_device_power);

	if (wl->keep_device_power) {
		sdio_flags = sdio_get_host_pm_caps(func);

		if (!(sdio_flags & MMC_PM_KEEP_POWER)) {
			dev_err(dev, "can't keep power while host "
				     "is suspended\n");
			ret = -EINVAL;
			goto out;
		}

		/* keep power while host suspended */
		ret = sdio_set_host_pm_flags(func, MMC_PM_KEEP_POWER);
		if (ret) {
			dev_err(dev, "error while trying to keep power\n");
			goto out;
		}
	}
out:
	return ret;
}

static int cc33xx_resume(struct device *dev)
{
	dev_dbg(dev, "cc33xx resume\n");

	return 0;
}

static const struct dev_pm_ops cc33xx_sdio_pm_ops = {
	.suspend	= cc33xx_suspend,
	.resume		= cc33xx_resume,
};

static struct sdio_driver cc33xx_sdio_driver = {
	.name		= "cc33xx_sdio",
	.id_table	= cc33xx_devices,
	.probe		= sdio_cc33xx_probe,
	.remove		= sdio_cc33xx_remove,
	.drv = {
		.pm = &cc33xx_sdio_pm_ops,
	},
};
#else
static struct sdio_driver cc33xx_sdio_driver = {
	.name		= "cc33xx_sdio",
	.id_table	= cc33xx_devices,
	.probe		= sdio_cc33xx_probe,
	.remove		= sdio_cc33xx_remove,
};
#endif /* CONFIG_PM */

static int __init sdio_cc33xx_init(void)
{
	return sdio_register_driver(&cc33xx_sdio_driver);
}

static void __exit sdio_cc33xx_exit(void)
{
	sdio_unregister_driver(&cc33xx_sdio_driver);
}

module_init(sdio_cc33xx_init);
module_exit(sdio_cc33xx_exit);

module_param(dump, bool, 0600);
MODULE_PARM_DESC(dump, "Enable sdio read/write dumps.");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luciano Coelho <coelho@ti.com>");
MODULE_AUTHOR("Juuso Oikarinen <juuso.oikarinen@nokia.com>");
