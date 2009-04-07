/* drivers/leds/leds-omap.c
 *
 * (C) 2006 Samsung Electronics
 * Kyungmin Park <kyungmin.park@samsung.com>
 *
 * OMAP - LEDs GPIO driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <mach/led.h>

/* our context */

static void omap_set_led_gpio(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	struct omap_led_config *led_dev;

	led_dev = container_of(led_cdev, struct omap_led_config, cdev);
	gpio_set_value(led_dev->gpio, value);
}

static int omap_led_probe(struct platform_device *dev)
{
	struct omap_led_platform_data *pdata = dev->dev.platform_data;
	struct omap_led_config *leds = pdata->leds;
	int i, ret = 0;

	for (i = 0; ret >= 0 && i < pdata->nr_leds; i++) {
		ret = gpio_request(leds[i].gpio, leds[i].cdev.name);
		if (ret < 0)
			break;
		gpio_direction_output(leds[i].gpio, 0);
		if (!leds[i].cdev.brightness_set)
			leds[i].cdev.brightness_set = omap_set_led_gpio;

		ret = led_classdev_register(&dev->dev, &leds[i].cdev);
	}

	if (ret < 0 && i > 1) {
		for (i = i - 2; i >= 0; i--) {
			led_classdev_unregister(&leds[i].cdev);
			gpio_free(leds[i].gpio);
		}
	}

	return ret;
}

static int omap_led_remove(struct platform_device *dev)
{
	struct omap_led_platform_data *pdata = dev->dev.platform_data;
	struct omap_led_config *leds = pdata->leds;
	int i;

	for (i = 0; i < pdata->nr_leds; i++) {
		led_classdev_unregister(&leds[i].cdev);
		gpio_free(leds[i].gpio);
	}

	return 0;
}

#ifdef CONFIG_PM
static int omap_led_suspend(struct platform_device *dev, pm_message_t state)
{
	struct omap_led_platform_data *pdata = dev->dev.platform_data;
	struct omap_led_config *leds = pdata->leds;
	int i;

	for (i = 0; i < pdata->nr_leds; i++)
		led_classdev_suspend(&leds[i].cdev);

	return 0;
}

static int omap_led_resume(struct platform_device *dev)
{
	struct omap_led_platform_data *pdata = dev->dev.platform_data;
	struct omap_led_config *leds = pdata->leds;
	int i;

	for (i = 0; i < pdata->nr_leds; i++)
		led_classdev_resume(&leds[i].cdev);

	return 0;
}
#else
#define omap_led_suspend	NULL
#define omap_led_resume		NULL
#endif

static struct platform_driver omap_led_driver = {
	.probe		= omap_led_probe,
	.remove		= omap_led_remove,
	.suspend	= omap_led_suspend,
	.resume		= omap_led_resume,
	.driver		= {
		.name		= "omap-led",
		.owner		= THIS_MODULE,
	},
};

static int __init omap_led_init(void)
{
	return platform_driver_register(&omap_led_driver);
}

static void __exit omap_led_exit(void)
{
 	platform_driver_unregister(&omap_led_driver);
}

module_init(omap_led_init);
module_exit(omap_led_exit);

MODULE_AUTHOR("Kyungmin Park<kyungmin.park@samsung.com>");
MODULE_DESCRIPTION("OMAP LED driver");
MODULE_LICENSE("GPL");
