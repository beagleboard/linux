/*
 * i2c/eeprom.h
 *
 * In-kernel interface for accessing eeprom memory.
 *
 * Copyright (C) 2012 Texas Instruments Inc.
 *                    Pantelis Antoniou <panto@antoniou-consulting.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef I2C_EEPROM_H
#define I2C_EEPROM_H

#include <linux/types.h>
#include <linux/memory.h>
#include <linux/err.h>
#include <linux/i2c.h>

/*
 * The method called in the client is
 *
 * int command(struct i2c_client *client, unsigned int cmd, void *arg);
 *
 * A single command is supported, which returns a pointer to the memory
 * accessor already available, but which was only accessible via platform
 * callbacks. We can't use platform callbacks anymore for device tree
 * platforms, hence this interface.
 *
 */

/* interface commands */
#define I2C_EEPROM_GET_MEMORY_ACCESSOR	1

static inline struct memory_accessor *
i2c_eeprom_get_memory_accessor(struct i2c_client *client)
{
	int ret;
	struct memory_accessor *macc;

	/* verify that the i2c client's driver has a command method */
	if (!client || !client->driver || !client->driver->command)
		return ERR_PTR(-ENOTSUPP);

	macc = NULL;
	ret = client->driver->command(client, I2C_EEPROM_GET_MEMORY_ACCESSOR,
			&macc);
	if (ret != 0)
		return ERR_PTR(ret);

	if (macc == NULL)
		return ERR_PTR(-ENOTSUPP);

	return macc;
}

#endif
