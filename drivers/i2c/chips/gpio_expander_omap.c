/*
 * drivers/i2c/chips/gpio_expander_omap.c
 *
 * Copyright (C) 2004 Texas Instruments Inc
 * Author:
 *
 * gpio expander is used to configure IrDA, camera and audio devices on omap 1710 processor.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/errno.h>

int read_gpio_expa(u8 * val, int addr);
int write_gpio_expa(u8 val, int addr);

int write_gpio_expa(u8 val, int addr)
{
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[1];

	adap = i2c_get_adapter(0);
	if (!adap)
		return -ENODEV;
	msg->addr = addr;	/* I2C address of GPIO EXPA */
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;
	data[0] = val;
	err = i2c_transfer(adap, msg, 1);
	if (err >= 0)
		return 0;
	return err;
}

/* Read from I/O EXPANDER on the H3 board.
 * The IO expanders need an independent I2C client driver.
 */

int read_gpio_expa(u8 * val, int addr)
{
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[1];

	adap = i2c_get_adapter(0);
	if (!adap)
		return -ENODEV;
	msg->addr = addr;	/* I2C address of GPIO EXPA */
	msg->flags = I2C_M_RD;
	msg->len = 2;
	msg->buf = data;
	err = i2c_transfer(adap, msg, 1);
	*val = data[0];

	if (err >= 0)
		return 0;
	return err;
}

EXPORT_SYMBOL(read_gpio_expa);
EXPORT_SYMBOL(write_gpio_expa);

