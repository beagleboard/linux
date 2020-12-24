// SPDX-License-Identifier: GPL-2.0
/*
 * mikroBUS driver for instantiating add-on
 * board devices with an identifier EEPROM
 *
 * Copyright 2020 Vaishnav M A, BeagleBoard.org Foundation.
 */

#define pr_fmt(fmt) "mikrobus:%s: " fmt, __func__

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/jump_label.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/mutex.h>
#include <linux/w1.h>
#include <linux/w1-gpio.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/gpio/machine.h>
#include <linux/nvmem-consumer.h>
#include <linux/nvmem-provider.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/serdev.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/clk-provider.h>
#include <linux/greybus/greybus_manifest.h>

#include "mikrobus_core.h"
#include "mikrobus_manifest.h"

static DEFINE_MUTEX(core_lock);
static DEFINE_IDR(mikrobus_port_idr);
static struct class_compat *mikrobus_port_compat_class;
int	__mikrobus_first_dynamic_bus_num;
static bool is_registered;
static int mikrobus_port_id_eeprom_probe(struct mikrobus_port *port);

const char *MIKROBUS_PINCTRL_STR[] = {"pwm", "uart", "i2c", "spi"};

struct bus_type mikrobus_bus_type = {
	.name = "mikrobus",
};
EXPORT_SYMBOL_GPL(mikrobus_bus_type);

int mikrobus_port_scan_eeprom(struct mikrobus_port *port)
{
	struct addon_board_info *board;
	int manifest_size;
	char header[12];
	int retval;
	char *buf;

	if(port->skip_scan)
		return -EINVAL;

	retval = nvmem_device_read(port->eeprom, 0, 12, header);
	if (retval != 12) {
		dev_err(&port->dev, "failed to fetch manifest header %d\n",
			retval);
		return -EINVAL;
	}
	manifest_size = mikrobus_manifest_header_validate(header, 12);
	if (manifest_size < 0) {
		dev_err(&port->dev, "invalid manifest size %d\n",
			manifest_size);
		return -EINVAL;
	}
	buf = kzalloc(manifest_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	retval = nvmem_device_read(port->eeprom, 0, manifest_size, buf);
	if (retval != manifest_size) {
		dev_err(&port->dev, "failed to fetch manifest %d\n", retval);
		retval = -EINVAL;
		goto err_free_buf;
	}
	board = kzalloc(sizeof(*board), GFP_KERNEL);
	if (!board) {
		retval = -ENOMEM;
		goto err_free_buf;
	}
	w1_reset_bus(port->w1_master);
	w1_write_8(port->w1_master, MIKROBUS_EEPROM_EXIT_ID_CMD);
	set_bit(W1_ABORT_SEARCH, &port->w1_master->flags);
	INIT_LIST_HEAD(&board->manifest_descs);
	INIT_LIST_HEAD(&board->devices);
	retval = mikrobus_manifest_parse(board, buf, manifest_size);
	if (!retval) {
		dev_err(&port->dev, "failed to parse manifest, size %d\n",
			manifest_size);
		retval = -EINVAL;
		goto err_free_board;
	}
	retval = mikrobus_board_register(port, board);
	if (retval) {
		dev_err(&port->dev, "failed to register board %s\n",
			board->name);
		goto err_free_board;
	}
	kfree(buf);
	return 0;
err_free_board:
	kfree(board);
err_free_buf:
	kfree(buf);
	return retval;
}
EXPORT_SYMBOL_GPL(mikrobus_port_scan_eeprom);

static ssize_t name_show(struct device *dev, struct device_attribute *attr,
						 char *buf)
{
	return sprintf(buf, "%s\n", to_mikrobus_port(dev)->name);
}
static DEVICE_ATTR_RO(name);

static ssize_t new_device_store(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct mikrobus_port *port = to_mikrobus_port(dev);
	struct addon_board_info *board;
	int retval;

	if (port->board) {
		dev_err(dev, "already has board registered\n");
		return -EBUSY;
	}

	board = kzalloc(sizeof(*board), GFP_KERNEL);
	if (!board)
		return -ENOMEM;
	INIT_LIST_HEAD(&board->manifest_descs);
	INIT_LIST_HEAD(&board->devices);
	retval = mikrobus_manifest_parse(board, (void *)buf, count);
	if (!retval) {
		dev_err(dev, "failed to parse manifest\n");
		retval = -EINVAL;
		goto err_free_board;
	}
	retval = mikrobus_board_register(port, board);
	if (retval) {
		dev_err(dev, "failed to register board %s\n", board->name);
		retval = -EINVAL;
		goto err_free_board;
	}
	return count;
err_free_board:
	kfree(board);
	return retval;
}
static DEVICE_ATTR_WO(new_device);

static ssize_t rescan_store(struct device *dev, struct device_attribute *attr,
							const char *buf, size_t count)
{
	struct mikrobus_port *port = to_mikrobus_port(dev);
	unsigned long id;
	int retval;
	int i;

	if (kstrtoul(buf, 0, &id)) {
		dev_err(dev, "cannot parse trigger\n");
		return -EINVAL;
	}
	if (port->board) {
		dev_err(dev, "already has board registered\n");
		return -EBUSY;
	}

	if (!port->eeprom){
		return mikrobus_port_id_eeprom_probe(port);
	}
	/* Enter ID Mode */
	sprintf(port->pinctrl_selected[MIKROBUS_PINCTRL_SPI], "%s_%s",
			MIKROBUS_PINCTRL_STR[MIKROBUS_PINCTRL_SPI], MIKROBUS_PINCTRL_STATE_GPIO);

	retval = mikrobus_port_pinctrl_select(port);
	/* set MOSI LOW, SCK HIGH */
	gpiod_direction_output(port->gpios->desc[MIKROBUS_PIN_MOSI], 0);
	gpiod_direction_output(port->gpios->desc[MIKROBUS_PIN_SCK], 1);
	msleep(100);
	for( i = 0; i < 4; i++){
		gpiod_set_value(port->gpios->desc[MIKROBUS_PIN_MOSI] , 1);
		udelay(1000);
		gpiod_set_value(port->gpios->desc[MIKROBUS_PIN_MOSI] , 0);
		udelay(1000);
	}
	msleep(100); /* temporary delay to fix ROM ID copy */

	port->skip_scan = 0;
	retval = mikrobus_port_scan_eeprom(port);
	if (retval) {
		dev_err(dev, "board register from manifest failed\n");
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR_WO(rescan);

static ssize_t idmode_store(struct device *dev, struct device_attribute *attr,
							const char *buf, size_t count)
{
	struct mikrobus_port *port = to_mikrobus_port(dev);
	unsigned long id;
	int retval;
	int i;

	if (kstrtoul(buf, 0, &id)) {
		dev_err(dev, "cannot parse trigger\n");
		return -EINVAL;
	}
	if (port->board) {
		dev_err(dev, "already has board registered\n");
		return -EBUSY;
	}

	if (!port->eeprom){
		return mikrobus_port_id_eeprom_probe(port);
	}
	/* Enter ID Mode */
	sprintf(port->pinctrl_selected[MIKROBUS_PINCTRL_SPI], "%s_%s",
			MIKROBUS_PINCTRL_STR[MIKROBUS_PINCTRL_SPI], MIKROBUS_PINCTRL_STATE_GPIO);

	retval = mikrobus_port_pinctrl_select(port);
	/* set MOSI LOW, SCK HIGH */
	gpiod_direction_output(port->gpios->desc[MIKROBUS_PIN_MOSI], 0);
	gpiod_direction_output(port->gpios->desc[MIKROBUS_PIN_SCK], 1);
	msleep(100);
	for( i = 0; i < 4; i++){
		gpiod_set_value(port->gpios->desc[MIKROBUS_PIN_MOSI] , 1);
		udelay(1000);
		gpiod_set_value(port->gpios->desc[MIKROBUS_PIN_MOSI] , 0);
		udelay(1000);
	}
	msleep(100); /* temporary delay to fix ROM ID copy */
	port->skip_scan = 1;
	return count;
}
static DEVICE_ATTR_WO(idmode);

static ssize_t delete_device_store(struct device *dev, struct device_attribute *attr,
							const char *buf, size_t count)
{
	struct mikrobus_port *port = to_mikrobus_port(dev);
	unsigned long id;

	if (kstrtoul(buf, 0, &id)) {
		dev_err(dev, "cannot parse board id");
		return -EINVAL;
	}
	if (!port->board) {
		dev_err(dev, "does not have registered boards");
		return -ENODEV;
	}
	mikrobus_board_unregister(port, port->board);
	return count;
}
static DEVICE_ATTR_IGNORE_LOCKDEP(delete_device, 0200, NULL, delete_device_store);

static struct attribute *mikrobus_port_attrs[] = {
	&dev_attr_new_device.attr, &dev_attr_rescan.attr, &dev_attr_idmode.attr,
	&dev_attr_delete_device.attr, &dev_attr_name.attr, NULL};
ATTRIBUTE_GROUPS(mikrobus_port);

static void mikrobus_port_release(struct device *dev)
{
	struct mikrobus_port *port = to_mikrobus_port(dev);

	mutex_lock(&core_lock);
	idr_remove(&mikrobus_port_idr, port->id);
	mutex_unlock(&core_lock);
	kfree(port);
}

struct device_type mikrobus_port_type = {
	.groups = mikrobus_port_groups,
	.release = mikrobus_port_release,
};
EXPORT_SYMBOL_GPL(mikrobus_port_type);

static int mikrobus_w1_master_match(struct device *dev, const void *data)
{
	struct mikrobus_port *port;

	if(dev->type != &mikrobus_port_type)
		return 0;	

	port = to_mikrobus_port(dev);

	return port->w1_master == data;
}

struct mikrobus_port *mikrobus_find_port_by_w1_master(struct w1_master *master)
{
	struct device *dev;

	dev = bus_find_device(&mikrobus_bus_type, NULL, master, mikrobus_w1_master_match);
	if (!dev)
		return NULL;

	return (dev->type == &mikrobus_port_type) ? to_mikrobus_port(dev) : NULL;
}
EXPORT_SYMBOL(mikrobus_find_port_by_w1_master);

int mikrobus_port_pinctrl_select(struct mikrobus_port *port)
{
	struct pinctrl_state *state;
	int retval;
	int i;

	for (i = 0; i < MIKROBUS_NUM_PINCTRL_STATE; i++) {
		state = pinctrl_lookup_state(port->pinctrl,
						port->pinctrl_selected[i]);
		if (!IS_ERR(state)) {
			retval = pinctrl_select_state(port->pinctrl, state);
			pr_info("setting pinctrl %s\n",
					port->pinctrl_selected[i]);
			if (retval != 0) {
				dev_err(&port->dev, "failed to select state %s\n",
						port->pinctrl_selected[i]);
				return retval;
			}
		} else {
			dev_err(&port->dev, "failed to find state %s\n",
						port->pinctrl_selected[i]);
			return PTR_ERR(state);
		}
	}

	return retval;
}
EXPORT_SYMBOL_GPL(mikrobus_port_pinctrl_select);

static int mikrobus_port_pinctrl_setup(struct mikrobus_port *port, struct addon_board_info *board)
{
	int retval;
	int i;

	for (i = 0; i < MIKROBUS_NUM_PINCTRL_STATE; i++) {
		switch (i) {
		case MIKROBUS_PINCTRL_PWM:
			if (board->pin_state[MIKROBUS_PIN_PWM] == MIKROBUS_STATE_PWM)
				sprintf(port->pinctrl_selected[i], "%s_%s",
					MIKROBUS_PINCTRL_STR[i], PINCTRL_STATE_DEFAULT);
			else
				sprintf(port->pinctrl_selected[i], "%s_%s",
					MIKROBUS_PINCTRL_STR[i], MIKROBUS_PINCTRL_STATE_GPIO);
			break;
		case MIKROBUS_PINCTRL_UART:
			if (board->pin_state[MIKROBUS_PIN_RX] == MIKROBUS_STATE_UART)
				sprintf(port->pinctrl_selected[i], "%s_%s",
					MIKROBUS_PINCTRL_STR[i], PINCTRL_STATE_DEFAULT);
			else
				sprintf(port->pinctrl_selected[i], "%s_%s",
					MIKROBUS_PINCTRL_STR[i], MIKROBUS_PINCTRL_STATE_GPIO);
			break;
		case MIKROBUS_PINCTRL_I2C:
			if (board->pin_state[MIKROBUS_PIN_SCL] == MIKROBUS_STATE_I2C)
				sprintf(port->pinctrl_selected[i], "%s_%s",
					MIKROBUS_PINCTRL_STR[i], PINCTRL_STATE_DEFAULT);
			else
				sprintf(port->pinctrl_selected[i], "%s_%s",
					MIKROBUS_PINCTRL_STR[i], MIKROBUS_PINCTRL_STATE_GPIO);
			break;
		case MIKROBUS_PINCTRL_SPI:
			if (board->pin_state[MIKROBUS_PIN_MOSI] == MIKROBUS_STATE_SPI)
				sprintf(port->pinctrl_selected[i], "%s_%s",
					MIKROBUS_PINCTRL_STR[i], PINCTRL_STATE_DEFAULT);
			else
				sprintf(port->pinctrl_selected[i], "%s_%s",
					MIKROBUS_PINCTRL_STR[i], MIKROBUS_PINCTRL_STATE_GPIO);
			break;
		}
	}

	retval = mikrobus_port_pinctrl_select(port);
	if (retval)
		dev_err(&port->dev, "failed to select pinctrl states [%d]", retval);
	return retval;
}

static int mikrobus_irq_get(struct mikrobus_port *port, int irqno,
							int irq_type)
{
	int irq;

	if (irqno > port->gpios->ndescs - 1) {
		dev_err(&port->dev, "GPIO %d does not exist", irqno);
		return -ENODEV;
	}

	irq = gpiod_to_irq(port->gpios->desc[irqno]);
	if (irq < 0) {
		dev_err(&port->dev, "could not get irq %d", irqno);
		return -EINVAL;
	}
	irq_set_irq_type(irq, irq_type);
	return irq;
}

static int mikrobus_gpio_setup(struct gpio_desc *gpio, int gpio_state)
{
	int retval;

	switch (gpio_state) {
	case MIKROBUS_STATE_INPUT:
		retval = gpiod_direction_input(gpio);
		break;
	case MIKROBUS_STATE_OUTPUT_HIGH:
		retval = gpiod_direction_output(gpio, 1);
		break;
	case MIKROBUS_STATE_OUTPUT_LOW:
		retval = gpiod_direction_output(gpio, 0);
		break;
	case MIKROBUS_STATE_PWM:
	case MIKROBUS_STATE_SPI:
	case MIKROBUS_STATE_I2C:
	default:
		return 0;
	}
	return retval;
}

static char *mikrobus_gpio_chip_name_get(struct mikrobus_port *port, int gpio)
{
	char *name;
	struct gpio_chip *gpiochip;

	if (gpio > port->gpios->ndescs - 1)
		return NULL;

	gpiochip = gpiod_to_chip(port->gpios->desc[gpio]);
	name = kmemdup(gpiochip->label, MIKROBUS_NAME_SIZE, GFP_KERNEL);
	return name;
}

static int mikrobus_gpio_hwnum_get(struct mikrobus_port *port, int gpio)
{
	int hwnum;
	struct gpio_chip *gpiochip;

	if (gpio > port->gpios->ndescs - 1)
		return -ENODEV;

	gpiochip = gpiod_to_chip(port->gpios->desc[gpio]);
	hwnum = desc_to_gpio(port->gpios->desc[gpio]) - gpiochip->base;
	return hwnum;
}

static void mikrobus_board_device_release_all(struct addon_board_info *info)
{
	struct board_device_info *dev;
	struct board_device_info *next;

	list_for_each_entry_safe(dev, next, &info->devices, links) {
		list_del(&dev->links);
		kfree(dev);
	}
}

static int mikrobus_device_register(struct mikrobus_port *port,
					struct board_device_info *dev, char *board_name)
{
	struct i2c_board_info *i2c;
	struct spi_device *spi;
	struct serdev_device *serdev;
	struct platform_device *pdev;
	struct gpiod_lookup_table *lookup;
	struct regulator_consumer_supply regulator;
	char devname[MIKROBUS_NAME_SIZE];
	int i;
	u64 *val;

	dev_info(&port->dev, "registering device : %s", dev->drv_name);

	if (dev->gpio_lookup) {
		lookup = dev->gpio_lookup;
		if (dev->protocol == GREYBUS_PROTOCOL_SPI) {
			snprintf(devname, sizeof(devname), "%s.%u",
				dev_name(&port->spi_mstr->dev),
				port->chip_select[dev->reg]);
			lookup->dev_id = kmemdup(devname, MIKROBUS_NAME_SIZE, GFP_KERNEL);
		} else if (dev->protocol == GREYBUS_PROTOCOL_RAW) {
			snprintf(devname, sizeof(devname), "%s.%u",
				 dev->drv_name, dev->reg);
			lookup->dev_id = kmemdup(devname, MIKROBUS_NAME_SIZE, GFP_KERNEL);
		} else
			lookup->dev_id = dev->drv_name;
		dev_info(&port->dev, " adding lookup table : %s\n",
			lookup->dev_id);
		for (i = 0; i < dev->num_gpio_resources; i++) {
			lookup->table[i].key =
			mikrobus_gpio_chip_name_get(port,
						lookup->table[i].chip_hwnum);
			lookup->table[i].chip_hwnum =
			mikrobus_gpio_hwnum_get(port,
						lookup->table[i].chip_hwnum);
		}
		gpiod_add_lookup_table(lookup);
	}
	if (dev->regulators) {
		if (dev->protocol == GREYBUS_PROTOCOL_SPI) {
			snprintf(devname, sizeof(devname), "%s.%u",
				dev_name(&port->spi_mstr->dev),
				port->chip_select[dev->reg]);
			regulator.dev_name  = kmemdup(devname, MIKROBUS_NAME_SIZE, GFP_KERNEL);
		} else if (dev->protocol == GREYBUS_PROTOCOL_RAW) {
			snprintf(devname, sizeof(devname), "%s.%u",
				 dev->drv_name, dev->reg);
			regulator.dev_name  = kmemdup(devname, MIKROBUS_NAME_SIZE, GFP_KERNEL);
		} else
			regulator.dev_name  = dev->drv_name;

		for (i = 0; i < dev->num_regulators; i++) {
			val = dev->regulators[i].value.u64_data;
			regulator.supply = kmemdup(dev->regulators[i].name, MIKROBUS_NAME_SIZE, GFP_KERNEL);
			dev_info(&port->dev, " adding fixed regulator %llu uv, %s for %s\n",
				*val, regulator.supply, regulator.dev_name);
			regulator_register_always_on(0, dev->regulators[i].name, &regulator,
				     1, *val);
		}
	}
	switch (dev->protocol) {
	case GREYBUS_PROTOCOL_SPI:
		spi = spi_alloc_device(port->spi_mstr);
		if (!spi)
			return -ENOMEM;
		strncpy(spi->modalias, dev->drv_name, sizeof(spi->modalias) - 1);
		if (dev->irq)
			spi->irq = mikrobus_irq_get(port, dev->irq, dev->irq_type);
		if (dev->properties)
			device_add_properties(&spi->dev, dev->properties);
		spi->chip_select = port->chip_select[dev->reg];
		spi->max_speed_hz = dev->max_speed_hz;
		spi->mode = dev->mode;
		if (dev->clocks) {
			for (i = 0; i < dev->num_clocks; i++) {
				val = dev->clocks[i].value.u64_data;
				dev_info(&port->dev, " adding fixed clock %s, %llu hz\n",
					dev->clocks[i].name, *val);
				//failing: under debug
				clk_register_fixed_rate(&spi->dev, dev->clocks[i].name, devname, 0, *val);
			}
		}
		dev->dev_client = (void *) spi_add_device(spi);
		break;
	case GREYBUS_PROTOCOL_I2C:
		i2c = kzalloc(sizeof(*i2c), GFP_KERNEL);
		if (!i2c)
			return -ENOMEM;
		strncpy(i2c->type, dev->drv_name, sizeof(i2c->type) - 1);
		if (dev->irq)
			i2c->irq = mikrobus_irq_get(port, dev->irq, dev->irq_type);
		if (dev->properties)
			i2c->properties = dev->properties;
		i2c->addr = dev->reg;
		dev->dev_client = (void *) i2c_new_client_device(port->i2c_adap, i2c);
		break;
	case GREYBUS_PROTOCOL_RAW:
		pdev = platform_device_alloc(dev->drv_name, 0);
		if (!pdev)
			return -ENOMEM;
		if (dev->properties)
			platform_device_add_properties(pdev, dev->properties);
		dev->dev_client = pdev;
		platform_device_add(dev->dev_client);
		break;
	case GREYBUS_PROTOCOL_UART:
		serdev = serdev_device_alloc(port->ser_ctrl);
		if (!serdev)
			return -ENOMEM;
		strncpy(serdev->modalias, dev->drv_name, sizeof(serdev->modalias) - 1);
		if (dev->properties)
			device_add_properties(&serdev->dev, dev->properties);
		dev->dev_client = serdev;
		serdev_device_add(serdev);
		break;
	break;
	default:
	return -EINVAL;
	}
	return 0;
}

static void mikrobus_device_unregister(struct mikrobus_port *port,
					struct board_device_info *dev, char *board_name)
{
	dev_info(&port->dev, "removing device %s\n", dev->drv_name);
	if (dev->gpio_lookup) {
		gpiod_remove_lookup_table(dev->gpio_lookup);
		kfree(dev->gpio_lookup);
	}
	kfree(dev->properties);
	switch (dev->protocol) {
	case GREYBUS_PROTOCOL_SPI:
		spi_unregister_device((struct spi_device *)dev->dev_client);
		break;
	case GREYBUS_PROTOCOL_I2C:
		i2c_unregister_device((struct i2c_client *)dev->dev_client);
		break;
	case GREYBUS_PROTOCOL_RAW:
		platform_device_unregister((struct platform_device  *)dev->dev_client);
		break;
	case GREYBUS_PROTOCOL_UART:
		serdev_device_remove((struct serdev_device  *)dev->dev_client);
		break;
	}
}

int mikrobus_board_register(struct mikrobus_port *port,	struct addon_board_info *board)
{
	struct board_device_info *devinfo;
	struct board_device_info *next;
	int retval;
	int i;

	if (WARN_ON(list_empty(&board->devices)))
		return false;
	if (port->pinctrl) {
		retval = mikrobus_port_pinctrl_setup(port, board);
		if (retval)
			dev_err(&port->dev, "failed to setup pinctrl state [%d]", retval);

	}
	if (port->gpios) {
		for (i = 0; i < port->gpios->ndescs; i++) {
			retval = mikrobus_gpio_setup(port->gpios->desc[i], board->pin_state[i]);
			if (retval)
				dev_err(&port->dev, "failed to setup gpio %d, state %d",
									i, board->pin_state[i]);
		}
	}
	list_for_each_entry_safe(devinfo, next, &board->devices, links)
		mikrobus_device_register(port, devinfo, board->name);
	port->board = board;
	return 0;
}
EXPORT_SYMBOL_GPL(mikrobus_board_register);

void mikrobus_board_unregister(struct mikrobus_port *port, struct addon_board_info *board)
{
	struct board_device_info *devinfo;
	struct board_device_info *next;

	if (WARN_ON(list_empty(&board->devices)))
		return;
	port->board = NULL;
	list_for_each_entry_safe(devinfo, next, &board->devices, links)
		mikrobus_device_unregister(port, devinfo, board->name);
	mikrobus_board_device_release_all(board);
	kfree(board);
	port->board = NULL;
}
EXPORT_SYMBOL_GPL(mikrobus_board_unregister);

static struct w1_gpio_platform_data mikrobus_id_eeprom_w1_pdata = {
      .pullup_gpiod  = NULL,
};

static struct platform_device mikrobus_id_eeprom_w1_device = {
      .name                   = "w1-gpio",
	  .dev.platform_data      = &mikrobus_id_eeprom_w1_pdata,
};

static int mikrobus_port_id_eeprom_probe(struct mikrobus_port *port)
{
	struct w1_bus_master *bm;
	struct gpiod_lookup_table *lookup;
	char devname[MIKROBUS_NAME_SIZE];
	int retval;
	int i;

	sprintf(port->pinctrl_selected[MIKROBUS_PINCTRL_SPI], "%s_%s",
			MIKROBUS_PINCTRL_STR[MIKROBUS_PINCTRL_SPI], MIKROBUS_PINCTRL_STATE_GPIO);

	retval = mikrobus_port_pinctrl_select(port);
	/* set MOSI LOW, SCK HIGH */
	gpiod_direction_output(port->gpios->desc[MIKROBUS_PIN_MOSI], 0);
	gpiod_direction_output(port->gpios->desc[MIKROBUS_PIN_SCK], 1);
	msleep(100);
	for( i = 0; i < 4; i++){
		gpiod_set_value(port->gpios->desc[MIKROBUS_PIN_MOSI] , 1);
		udelay(1000);
		gpiod_set_value(port->gpios->desc[MIKROBUS_PIN_MOSI] , 0);
		udelay(1000);
	}
	msleep(100); /* temporary delay to fix ROM ID copy */

	lookup = kzalloc(struct_size(lookup, table, 1),
					GFP_KERNEL);
	if (!lookup)
			return -ENOMEM;
	snprintf(devname, sizeof(devname), "%s.%u",
				mikrobus_id_eeprom_w1_device.name,
				port->id);
	mikrobus_id_eeprom_w1_device.id = port->id;
	lookup->dev_id = kmemdup(devname, MIKROBUS_NAME_SIZE, GFP_KERNEL);
	lookup->table[0].key = mikrobus_gpio_chip_name_get(port,
						MIKROBUS_PIN_CS);
	lookup->table[0].flags = GPIO_ACTIVE_HIGH|GPIO_OPEN_DRAIN;
	lookup->table[0].chip_hwnum = mikrobus_gpio_hwnum_get(port,
						MIKROBUS_PIN_CS);
	gpiod_add_lookup_table(lookup);
	platform_device_register(&mikrobus_id_eeprom_w1_device);
	port->w1_gpio = &mikrobus_id_eeprom_w1_device;
	bm = (struct w1_bus_master *) platform_get_drvdata(&mikrobus_id_eeprom_w1_device);
	if(bm) {
		port->w1_master = w1_find_master_device(bm);
		if(!port->w1_master){
			dev_err(&port->dev, "failed to find W1 GPIO master, port [%s]\n",
									port->name);
			gpiod_remove_lookup_table(lookup);
			kfree(lookup);
			return -ENODEV;
		}
		mutex_lock(&port->w1_master->mutex);
		port->w1_master->max_slave_count = 1;
		clear_bit(W1_WARN_MAX_COUNT, &port->w1_master->flags);
		mutex_unlock(&port->w1_master->mutex);
	}
	return 0;
}

int mikrobus_port_register(struct mikrobus_port *port)
{
	struct device *dev = &port->dev;
	int retval;
	int id;

	if (WARN_ON(!is_registered))
		return -EAGAIN;

	if (dev->of_node) {
		id = of_alias_get_id(dev->of_node, "mikrobus");
		if (id >= 0) {
			port->id = id;
			mutex_lock(&core_lock);
			id = idr_alloc(&mikrobus_port_idr, port, port->id, port->id + 1,
										GFP_KERNEL);
			mutex_unlock(&core_lock);
			if (WARN(id < 0, "couldn't get idr"))
				return id == -ENOSPC ? -EBUSY : id;
		}
	} else {
		mutex_lock(&core_lock);
		id = idr_alloc(&mikrobus_port_idr, port, __mikrobus_first_dynamic_bus_num, 0,
											GFP_KERNEL);
		mutex_unlock(&core_lock);
		if (id < 0)
			return id;
		port->id = id;
	}
	port->dev.bus = &mikrobus_bus_type;
	port->dev.type = &mikrobus_port_type;
	strncpy(port->name, "mikrobus-port", sizeof(port->name) - 1);
	dev_set_name(&port->dev, "mikrobus-%d", port->id);
	pr_info("registering port mikrobus-%d ", port->id);
	retval = device_register(&port->dev);
	if (retval) {
		pr_err("port '%d': can't register device (%d)", port->id, retval);
		put_device(&port->dev);
		return retval;
	}
	retval = class_compat_create_link(mikrobus_port_compat_class, &port->dev,
								port->dev.parent);
	if (retval)
		dev_warn(&port->dev, "failed to create compatibility class link\n");
	if (!port->w1_master) {
		dev_info(&port->dev, "mikrobus port %d eeprom empty probing default eeprom\n",
											port->id);
		retval = mikrobus_port_id_eeprom_probe(port);
	}
	return retval;
}
EXPORT_SYMBOL_GPL(mikrobus_port_register);

void mikrobus_port_delete(struct mikrobus_port *port)
{
	struct mikrobus_port *found;

	mutex_lock(&core_lock);
	found = idr_find(&mikrobus_port_idr, port->id);
	mutex_unlock(&core_lock);
	if (found != port) {
		pr_err("port [%s] not registered", port->name);
		return;
	}
	if (port->board) {
		dev_err(&port->dev, "attempting to delete port with registered boards, port [%s]\n",
									port->name);
		return;
	}

	if (port->eeprom) {
		nvmem_device_put(port->eeprom);
		platform_device_unregister(port->w1_gpio);
	}

	class_compat_remove_link(mikrobus_port_compat_class, &port->dev,
							port->dev.parent);
	device_unregister(&port->dev);
	mutex_lock(&core_lock);
	idr_remove(&mikrobus_port_idr, port->id);
	mutex_unlock(&core_lock);
	memset(&port->dev, 0, sizeof(port->dev));
}
EXPORT_SYMBOL_GPL(mikrobus_port_delete);

static int mikrobus_port_probe_pinctrl_setup(struct mikrobus_port *port)
{
	struct pinctrl_state *state;
	struct device *dev = port->dev.parent;
	int retval, i;

	state = pinctrl_lookup_state(port->pinctrl, PINCTRL_STATE_DEFAULT);
	if (!IS_ERR(state)) {
		retval = pinctrl_select_state(port->pinctrl, state);
		if (retval != 0) {
			dev_err(dev, "Failed to select state %s\n",
				PINCTRL_STATE_DEFAULT);
			return retval;
		}
	} else {
		dev_err(dev, "failed to find state %s\n",
			PINCTRL_STATE_DEFAULT);
		return PTR_ERR(state);
	}

	for (i = 0; i < MIKROBUS_NUM_PINCTRL_STATE; i++) {
		port->pinctrl_selected[i] =
				kmalloc(MIKROBUS_PINCTRL_NAME_SIZE, GFP_KERNEL);
		sprintf(port->pinctrl_selected[i], "%s_%s",
			MIKROBUS_PINCTRL_STR[i], PINCTRL_STATE_DEFAULT);
	}

	retval = mikrobus_port_pinctrl_select(port);
	if (retval)
		dev_err(dev, "failed to select pinctrl states [%d]", retval);
	return retval;
}

static int mikrobus_port_probe(struct platform_device *pdev)
{
	struct mikrobus_port *port;
	struct device *dev = &pdev->dev;
	struct device_node *i2c_adap_np;
	struct device_node *uart_np;
	int retval;
	u32 val;

	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	i2c_adap_np = of_parse_phandle(dev->of_node, "i2c-adapter", 0);
	if (!i2c_adap_np) {
		dev_err(dev, "cannot parse i2c-adapter\n");
		retval = -ENODEV;
		goto err_port;
	}
	port->i2c_adap = of_find_i2c_adapter_by_node(i2c_adap_np);
	of_node_put(i2c_adap_np);
	retval = device_property_read_u32(dev, "spi-master", &val);
	if (retval) {
		dev_err(dev, "failed to get spi-master [%d]\n", retval);
		goto err_port;
	}
	port->spi_mstr = spi_busnum_to_master(val);
	retval = device_property_read_u32_array(dev, "spi-cs",
						port->chip_select, 2);
	if (retval) {
		dev_err(dev, "failed to get spi-cs [%d]\n", retval);
		goto err_port;
	}
	uart_np = of_parse_phandle(dev->of_node, "uart", 0);
	if (!uart_np) {
		dev_err(dev, "cannot parse uart\n");
		retval = -ENODEV;
		goto err_port;
	}
	port->ser_ctrl = of_find_serdev_controller_by_node(uart_np);
	of_node_put(uart_np);
	//port->pwm = devm_pwm_get(dev, NULL);
	//if (IS_ERR(port->pwm)) {
	//	retval = PTR_ERR(port->pwm);
	//	if (retval != -EPROBE_DEFER)
	//		dev_err(dev, "failed to request PWM device [%d]\n",
	//			retval);
	//	goto err_port;
	//}
	port->gpios = gpiod_get_array(dev, "mikrobus", GPIOD_OUT_LOW);
	if (IS_ERR(port->gpios)) {
		retval = PTR_ERR(port->gpios);
		dev_err(dev, "failed to get gpio array [%d]\n", retval);
		goto err_port;
	}
	port->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(port->pinctrl)) {
		retval = PTR_ERR(port->pinctrl);
		dev_err(dev, "failed to get pinctrl [%d]\n", retval);
		goto err_port;
	}
	port->dev.parent = dev;
	port->dev.of_node = pdev->dev.of_node;

	retval = mikrobus_port_probe_pinctrl_setup(port);
	if (retval) {
		dev_err(dev, "failed to setup pinctrl [%d]\n", retval);
		goto err_port;
	}

	retval = mikrobus_port_register(port);
	if (retval) {
		pr_err("port : can't register port [%d]\n", retval);
		goto err_port;
	}
	platform_set_drvdata(pdev, port);
	return 0;
err_port:
	kfree(port);
	return retval;
}

static int mikrobus_port_remove(struct platform_device *pdev)
{
	struct mikrobus_port	*port = platform_get_drvdata(pdev);

	mikrobus_port_delete(port);
	return 0;
}

static const struct of_device_id mikrobus_port_of_match[] = {
	{.compatible = "linux,mikrobus"},
	{},
};
MODULE_DEVICE_TABLE(of, mikrobus_port_of_match);

static struct platform_driver mikrobus_port_driver = {
	.probe = mikrobus_port_probe,
	.remove = mikrobus_port_remove,
	.driver = {
		.name = "mikrobus",
		.of_match_table = of_match_ptr(mikrobus_port_of_match),
	},
};

static int __init mikrobus_init(void)
{
	int retval;

	retval = bus_register(&mikrobus_bus_type);
	if (retval) {
		pr_err("bus_register failed (%d)\n", retval);
		return retval;
	}
	mikrobus_port_compat_class = class_compat_register("mikrobus-port");
	if (!mikrobus_port_compat_class) {
		pr_err("class_compat register failed (%d)\n", retval);
		retval = -ENOMEM;
		goto class_err;
	}
	retval = of_alias_get_highest_id("mikrobus");
	if (retval >= __mikrobus_first_dynamic_bus_num)
		__mikrobus_first_dynamic_bus_num = retval + 1;

	is_registered = true;
	retval = platform_driver_register(&mikrobus_port_driver);
	if (retval)
		pr_err("driver register failed [%d]\n", retval);
	return retval;

class_err:
	bus_unregister(&mikrobus_bus_type);
	idr_destroy(&mikrobus_port_idr);
	is_registered = false;
	return retval;
}
subsys_initcall(mikrobus_init);

static void __exit mikrobus_exit(void)
{
	platform_driver_unregister(&mikrobus_port_driver);
	bus_unregister(&mikrobus_bus_type);
	class_compat_unregister(mikrobus_port_compat_class);
	idr_destroy(&mikrobus_port_idr);
}
module_exit(mikrobus_exit);

MODULE_AUTHOR("Vaishnav M A <vaishnav@beagleboard.org>");
MODULE_DESCRIPTION("mikroBUS main module");
MODULE_LICENSE("GPL v2");
