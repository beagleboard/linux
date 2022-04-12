/* SPDX-License-Identifier: GPL-2.0 */
/*
 * drivers/i2c/i2c-atr.h -- I2C Address Translator
 *
 * Copyright (c) 2019 Luca Ceresoli <luca@lucaceresoli.net>
 *
 * Based on i2c-mux.h
 */

#ifndef _LINUX_I2C_ATR_H
#define _LINUX_I2C_ATR_H

#ifdef __KERNEL__

#include <linux/i2c.h>
#include <linux/mutex.h>

struct i2c_atr;

/**
 * struct i2c_atr_ops - Callbacks from ATR to the device driver.
 * @select:        Ask the driver to select a child bus (optional)
 * @deselect:      Ask the driver to deselect a child bus (optional)
 * @attach_client: Notify the driver of a new device connected on a child
 *                 bus. The driver must choose an I2C alias, configure the
 *                 hardware to use it and return it in `alias_id`.
 * @detach_client: Notify the driver of a device getting disconnected. The
 *                 driver must configure the hardware to stop using the
 *                 alias.
 *
 * All these functions return 0 on success, a negative error code otherwise.
 */
struct i2c_atr_ops {
	int  (*select)(struct i2c_atr *atr, u32 chan_id);
	int  (*deselect)(struct i2c_atr *atr, u32 chan_id);
	int  (*attach_client)(struct i2c_atr *atr, u32 chan_id,
			      const struct i2c_board_info *info,
			      const struct i2c_client *client,
			      u16 *alias_id);
	void (*detach_client)(struct i2c_atr *atr, u32 chan_id,
			      const struct i2c_client *client);
};

/*
 * Helper to add I2C ATR features to a device driver.
 */
struct i2c_atr {
	/* private: internal use only */

	struct i2c_adapter *parent;
	struct device *dev;
	const struct i2c_atr_ops *ops;

	void *priv;

	struct i2c_algorithm algo;
	struct mutex lock;
	int max_adapters;

	struct i2c_adapter *adapter[0];
};

struct i2c_atr *i2c_atr_new(struct i2c_adapter *parent, struct device *dev,
			    const struct i2c_atr_ops *ops, int max_adapters);
void i2c_atr_delete(struct i2c_atr *atr);

static inline void i2c_atr_set_clientdata(struct i2c_atr *atr, void *data)
{
	atr->priv = data;
}

static inline void *i2c_atr_get_clientdata(struct i2c_atr *atr)
{
	return atr->priv;
}

int i2c_atr_add_adapter(struct i2c_atr *atr, u32 chan_id);
void i2c_atr_del_adapter(struct i2c_atr *atr, u32 chan_id);

#endif /* __KERNEL__ */

#endif /* _LINUX_I2C_ATR_H */
