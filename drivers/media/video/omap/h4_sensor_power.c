/*
 * drivers/media/video/omap/h4_sensor_power.c
 *
 * H4 sensor powerup/down functions.
 *
 * Author: Andy Lowe (source@mvista.com)
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2004 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/types.h>

#include <asm/arch/gpioexpander.h>

int h4_sensor_powerup(void);
int h4_sensor_powerdown(void);

int
h4_sensor_powerup(void)
{
	unsigned char expa;
	int err;

	/* read current state of GPIO EXPA outputs */
	if ((err = read_gpio_expa(&expa, 0x20))) {
		printk(KERN_ERR "Error reading GPIO EXPA\n");
		return err;
	}
	/* Set GPIO EXPA P3 (CAMERA_MODULE_EN) to power-up sensor */
	if ((err = write_gpio_expa(expa | 0x08, 0x20))) {
		printk(KERN_ERR "Error writing to GPIO EXPA\n");
		return err;
	}

	/* read current state of GPIO EXPA outputs */
	if ((err = read_gpio_expa(&expa, 0x22))) {
		printk(KERN_ERR "Error reading GPIO EXPA\n");
		return err;
	}
	/* Clear GPIO EXPA P7 (CAM_RST) */
	if ((err = write_gpio_expa(expa & ~0x80, 0x22))) {
		printk(KERN_ERR "Error writing to GPIO EXPA\n");
		return err;
	}

	return 0;
}

int
h4_sensor_powerdown(void)
{
	unsigned char expa;
	int err;

	/* read current state of GPIO EXPA outputs */
	if ((err = read_gpio_expa(&expa, 0x20))) {
		printk(KERN_ERR "Error reading GPIO EXPA\n");
		return err;
	}
	/* Clear GPIO EXPA P3 (CAMERA_MODULE_EN) to power-down sensor */
	if ((err = write_gpio_expa(expa & ~0x08, 0x20))) {
		printk(KERN_ERR "Error writing to GPIO EXPA\n");
		return err;
	}

	return 0;
}

EXPORT_SYMBOL(h4_sensor_powerup);
EXPORT_SYMBOL(h4_sensor_powerdown);
