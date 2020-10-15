/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mikroBUS Driver for instantiating add-on
 * board devices with an identifier EEPROM
 *
 * Copyright 2020 Vaishnav M A, BeagleBoard.org Foundation.
 */

#ifndef __MIKROBUS_H
#define __MIKROBUS_H

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/spi/spi.h>
#include <linux/serdev.h>
#include <linux/property.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <linux/nvmem-consumer.h>
#include <linux/nvmem-provider.h>

#define MIKROBUS_VERSION_MAJOR 0x00
#define MIKROBUS_VERSION_MINOR 0x03

#define MIKROBUS_NAME_SIZE		40
#define MIKROBUS_PINCTRL_NAME_SIZE	20

#define MIKROBUS_NUM_PINCTRL_STATE	4
#define MIKROBUS_NUM_CS			2

#define MIKROBUS_PINCTRL_PWM		0
#define MIKROBUS_PINCTRL_UART		1
#define MIKROBUS_PINCTRL_I2C		2
#define MIKROBUS_PINCTRL_SPI		3

#define MIKROBUS_PINCTRL_STATE_GPIO	"gpio"

#define MIKROBUS_EEPROM_EXIT_ID_CMD 0xD2

extern struct bus_type mikrobus_bus_type;
extern struct device_type mikrobus_port_type;
extern const char *MIKROBUS_PINCTRL_STR[];

enum mikrobus_property_type {
	MIKROBUS_PROPERTY_TYPE_MIKROBUS = 0x00,
	MIKROBUS_PROPERTY_TYPE_PROPERTY	= 0x01,
	MIKROBUS_PROPERTY_TYPE_GPIO	= 0x02,
	MIKROBUS_PROPERTY_TYPE_U8	= 0x03,
	MIKROBUS_PROPERTY_TYPE_U16	= 0x04,
	MIKROBUS_PROPERTY_TYPE_U32	= 0x05,
	MIKROBUS_PROPERTY_TYPE_U64	= 0x06,
	MIKROBUS_PROPERTY_TYPE_REGULATOR	= 0x07,
	MIKROBUS_PROPERTY_TYPE_CLOCK	= 0x08,
};

enum mikrobus_pin {
	MIKROBUS_PIN_PWM	= 0x00,
	MIKROBUS_PIN_INT	= 0x01,
	MIKROBUS_PIN_RX		= 0x02,
	MIKROBUS_PIN_TX		= 0x03,
	MIKROBUS_PIN_SCL	= 0x04,
	MIKROBUS_PIN_SDA	= 0x05,
	MIKROBUS_PIN_MOSI	= 0x06,
	MIKROBUS_PIN_MISO	= 0x07,
	MIKROBUS_PIN_SCK	= 0x08,
	MIKROBUS_PIN_CS		= 0x09,
	MIKROBUS_PIN_RST	= 0x0A,
	MIKROBUS_PIN_AN		= 0x0B,
	MIKROBUS_PORT_PIN_COUNT = 0x0C,
};

enum mikrobus_pin_state {
	MIKROBUS_STATE_INPUT		= 0x01,
	MIKROBUS_STATE_OUTPUT_HIGH	= 0x02,
	MIKROBUS_STATE_OUTPUT_LOW	= 0x03,
	MIKROBUS_STATE_PWM		= 0x04,
	MIKROBUS_STATE_SPI		= 0x05,
	MIKROBUS_STATE_I2C		= 0x06,
	MIKROBUS_STATE_UART		= 0x07,
};

/*
 * board_device_info describes a single device on a mikrobus add-on
 * board, an add-on board can present one or more device to the host
 *
 * @gpio_lookup: used to provide the GPIO lookup table for
 * passing the named GPIOs to device drivers.
 * @properties: used to provide the property_entry to pass named
 * properties to device drivers, applicable only when driver uses
 * device_property_read_* calls to fetch the properties.
 * @num_gpio_resources: number of named gpio resources for the device,
 * used mainly for gpiod_lookup_table memory allocation.
 * @num_properties: number of custom properties for the device,
 * used mainly for property_entry memory allocation.
 * @protocol: used to know the type of the device and it should
 * contain one of the values defined under 'enum greybus_class_type'
 * under linux/greybus/greybus_manifest.h
 * @reg: I2C address for the device, for devices on the SPI bus
 * this field is the chip select address relative to the mikrobus
 * port:0->device chip select connected to CS pin on mikroBUS port
 *	1->device chip select connected to RST Pin on mikroBUS port
 * @mode: SPI mode
 * @max_speed_hz: SPI max speed(Hz)
 * @drv_name: device_id to match with the driver
 * @irq_type: type of IRQ trigger , match with defines in linux/interrupt.h
 * @irq: irq number relative to the mikrobus port should contain one of the
 * values defined under 'enum mikrobus_pin'
 * @id: device id starting from 1
 */
struct board_device_info {
	struct gpiod_lookup_table *gpio_lookup;
	struct property_entry *properties;
	struct property_entry *regulators;
	struct property_entry *clocks;
	struct list_head links;
	unsigned short num_gpio_resources;
	unsigned short num_properties;
	unsigned short num_regulators;
	unsigned short num_clocks;
	unsigned short protocol;
	unsigned short reg;
	unsigned int mode;
	void *dev_client;
	u32 max_speed_hz;
	char *drv_name;
	int irq_type;
	int irq;
	int id;
};

/*
 * addon_board_info describes a mikrobus add-on device the add-on
 * board, an add-on board can present one or more device to the host
 *
 * @manifest_descs: list of manifest descriptors
 * @devices: list of devices on the board
 * @pin_state: the state of each pin on the mikrobus port required
 * for the add-on board should contain one of the values defined under
 * 'enum mikrobus_pin_state' restrictions are as per mikrobus standard
 * specifications.
 * @name: add-on board name
 */
struct addon_board_info {
	struct list_head manifest_descs;
	struct list_head devices;
	u8 pin_state[MIKROBUS_PORT_PIN_COUNT];
	char *name;
};

/*
 * mikrobus_port describes the peripherals mapped to a
 * mikrobus port.
 *
 * @eeprom_client: i2c_client corresponding to the eeprom
 * on the add-on board.
 * @board: pointer to the attached add-on board.
 * @i2c_adap: I2C adapter attached to the mikrobus port.
 * @spi_mstr: SPI master attached to the mikrobus port.
 * @eeprom: nvmem_device for the eeprom on the add-on board.
 * @pwm: pwm_device attached to the mikrobus port PWM pin.
 * @pinctrl_selected: current pinctrl_selected state.
 * @chip_select: chip select number mapped to the SPI
 * CS pin on the mikrobus port and the RST pin on the mikrobus
 * port
 * @id: port id starting from 1
 */
struct mikrobus_port {
	struct addon_board_info *board;
	struct nvmem_device *eeprom;
	struct i2c_adapter *i2c_adap;
	struct spi_master *spi_mstr;
	struct w1_master *w1_master;
	struct platform_device *w1_gpio;
	struct serdev_controller *ser_ctrl;
	struct gpio_descs *gpios;
	struct pwm_device *pwm;
	struct pinctrl *pinctrl;
	struct module *owner;
	struct device dev;
	char name[MIKROBUS_NAME_SIZE];
	char *pinctrl_selected[MIKROBUS_NUM_PINCTRL_STATE];
	unsigned int chip_select[MIKROBUS_NUM_CS];
	int id;
};
#define to_mikrobus_port(d) container_of(d, struct mikrobus_port, dev)

void mikrobus_board_unregister(struct mikrobus_port *port,
				struct addon_board_info *board);
int mikrobus_board_register(struct mikrobus_port *port,
				struct addon_board_info *board);
int mikrobus_port_register(struct mikrobus_port *port);
int mikrobus_port_pinctrl_select(struct mikrobus_port *port);
void mikrobus_port_delete(struct mikrobus_port *port);
int mikrobus_port_scan_eeprom(struct mikrobus_port *port);
struct mikrobus_port *mikrobus_find_port_by_w1_master(struct w1_master *master);
#endif /* __MIKROBUS_H */
