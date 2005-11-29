/*
 * drivers/media/video/omap/h3_sensor_power.c
 *
 * H3 sensor powerup/down functions.
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

int h3_sensor_powerup(void);
int h3_sensor_powerdown(void);

int
h3_sensor_powerup(void)
{
	unsigned char expa;
	int err;

	/* read the current state of GPIO EXPA output */
	if (( err = read_gpio_expa(&expa, 0x27))) {
		printk(KERN_ERR "Error reading GPIO EXPA \n");
		return err;
	}
	/* set GPIO EXPA P7 CAMERA_MOD_EN to power-up sensor */
	if ((err = write_gpio_expa(expa | 0x80, 0x27))) {
		printk(KERN_ERR "Error writing to GPIO EXPA \n");
		return err;
	}
	return 0;
}

int
h3_sensor_powerdown(void)
{
	unsigned char expa;
	int err;

	/* read the current state of GPIO EXPA output */
	if (( err = read_gpio_expa(&expa, 0x27))) {
		printk(KERN_ERR "Error reading GPIO EXPA \n");
		return err;
	}
	/* clear GPIO EXPA P7 CAMERA_MOD_EN to power-up sensor */
	if ((err = write_gpio_expa(expa & ~0x80, 0x27))) {
		printk(KERN_ERR "Error writing to GPIO EXPA \n");
		return err;
	}
	return 0;
}

EXPORT_SYMBOL(h3_sensor_powerup);
EXPORT_SYMBOL(h3_sensor_powerdown);
