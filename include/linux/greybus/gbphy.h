/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Greybus Bridged-Phy Bus driver
 *
 * Copyright 2016 Google Inc.
 */

#ifndef __GBPHY_H
#define __GBPHY_H

#include <linux/i2c.h>
#include <linux/gpio/driver.h>

struct gbphy_host {
	struct gb_bundle *bundle;
	struct list_head devices;
};


struct gb_i2c_device {
	struct gb_connection	*connection;
	struct gbphy_device	*gbphy_dev;
	u32			functionality;
	struct i2c_adapter	adapter;
};

struct gb_spilib {
	struct gb_connection	*connection;
	struct device		*parent;
	struct spi_transfer	*first_xfer;
	struct spi_transfer	*last_xfer;
	struct spilib_ops	*ops;
	u32			rx_xfer_offset;
	u32			tx_xfer_offset;
	u32			last_xfer_size;
	unsigned int		op_timeout;
	u16			mode;
	u16			flags;
	u32			bits_per_word_mask;
	u8			num_chipselect;
	u32			min_speed_hz;
	u32			max_speed_hz;
};
struct gb_gpio_line {
	/* The following has to be an array of line_max entries */
	/* --> make them just a flags field */
	u8			active:    1,
				direction: 1,	/* 0 = output, 1 = input */
				value:     1;	/* 0 = low, 1 = high */
	u16			debounce_usec;

	u8			irq_type;
	bool			irq_type_pending;
	bool			masked;
	bool			masked_pending;
};

struct gb_gpio_controller {
	struct gbphy_device	*gbphy_dev;
	struct gb_connection	*connection;
	u8			line_max;	/* max line number */
	struct gb_gpio_line	*lines;
	struct gpio_chip	chip;
	struct irq_chip		irqc;
	struct mutex		irq_lock;
};

struct gbphy_device {
	u32 id;
	struct greybus_descriptor_cport *cport_desc;
	struct gb_bundle *bundle;
	struct list_head list;
	struct device dev;
};
#define to_gbphy_dev(d) container_of(d, struct gbphy_device, dev)

static inline void *gb_gbphy_get_data(struct gbphy_device *gdev)
{
	return dev_get_drvdata(&gdev->dev);
}

static inline void gb_gbphy_set_data(struct gbphy_device *gdev, void *data)
{
	dev_set_drvdata(&gdev->dev, data);
}

struct gbphy_device_id {
	__u8 protocol_id;
};

#define GBPHY_PROTOCOL(p)		\
	.protocol_id	= (p),

struct gbphy_driver {
	const char *name;
	int (*probe)(struct gbphy_device *,
		     const struct gbphy_device_id *id);
	void (*remove)(struct gbphy_device *);
	const struct gbphy_device_id *id_table;

	struct device_driver driver;
};
#define to_gbphy_driver(d) container_of(d, struct gbphy_driver, driver)

int gb_gbphy_register_driver(struct gbphy_driver *driver,
			     struct module *owner, const char *mod_name);
void gb_gbphy_deregister_driver(struct gbphy_driver *driver);

#define gb_gbphy_register(driver) \
	gb_gbphy_register_driver(driver, THIS_MODULE, KBUILD_MODNAME)
#define gb_gbphy_deregister(driver) \
	gb_gbphy_deregister_driver(driver)

/**
 * module_gbphy_driver() - Helper macro for registering a gbphy driver
 * @__gbphy_driver: gbphy_driver structure
 *
 * Helper macro for gbphy drivers to set up proper module init / exit
 * functions.  Replaces module_init() and module_exit() and keeps people from
 * printing pointless things to the kernel log when their driver is loaded.
 */
#define module_gbphy_driver(__gbphy_driver)	\
	module_driver(__gbphy_driver, gb_gbphy_register, gb_gbphy_deregister)

#ifdef CONFIG_PM
static inline int gbphy_runtime_get_sync(struct gbphy_device *gbphy_dev)
{
	struct device *dev = &gbphy_dev->dev;
	int ret;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed: %d\n", ret);
		pm_runtime_put_noidle(dev);
		return ret;
	}

	return 0;
}

static inline void gbphy_runtime_put_autosuspend(struct gbphy_device *gbphy_dev)
{
	struct device *dev = &gbphy_dev->dev;

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
}

static inline void gbphy_runtime_get_noresume(struct gbphy_device *gbphy_dev)
{
	pm_runtime_get_noresume(&gbphy_dev->dev);
}

static inline void gbphy_runtime_put_noidle(struct gbphy_device *gbphy_dev)
{
	pm_runtime_put_noidle(&gbphy_dev->dev);
}
#else
static inline int gbphy_runtime_get_sync(struct gbphy_device *gbphy_dev) { return 0; }
static inline void gbphy_runtime_put_autosuspend(struct gbphy_device *gbphy_dev) {}
static inline void gbphy_runtime_get_noresume(struct gbphy_device *gbphy_dev) {}
static inline void gbphy_runtime_put_noidle(struct gbphy_device *gbphy_dev) {}
#endif

#endif /* __GBPHY_H */

