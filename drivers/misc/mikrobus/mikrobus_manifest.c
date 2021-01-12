// SPDX-License-Identifier: GPL-2.0
/*
 * mikroBUS manifest parsing, an
 * extension to Greybus Manifest Parsing
 * under drivers/greybus/manifest.c
 *
 * Copyright 2014-2015 Google Inc.
 * Copyright 2014-2015 Linaro Ltd.
 */

#define pr_fmt(fmt) "mikrobus_manifest:%s: " fmt, __func__

#include <linux/bits.h>
#include <linux/types.h>
#include <linux/property.h>
#include <linux/greybus/greybus_manifest.h>

#include "mikrobus_manifest.h"

struct manifest_desc {
	struct list_head links;
	size_t size;
	void *data;
	enum greybus_descriptor_type type;
};

static void manifest_descriptor_release_all(struct addon_board_info *board)
{
	struct manifest_desc *descriptor;
	struct manifest_desc *next;

	list_for_each_entry_safe(descriptor, next, &board->manifest_descs, links) {
		list_del(&descriptor->links);
		kfree(descriptor);
	}
}

static int board_descriptor_add(struct addon_board_info *board, struct greybus_descriptor *desc,
				size_t size)
{
	struct greybus_descriptor_header *desc_header = &desc->header;
	struct manifest_desc *descriptor;
	size_t desc_size;
	size_t expected_size;

	if (size < sizeof(*desc_header)) {
		pr_err("short descriptor (%zu < %zu)", size, sizeof(*desc_header));
		return -EINVAL;
	}
	desc_size = le16_to_cpu(desc_header->size);
	if (desc_size > size) {
		pr_err("incorrect descriptor size (%zu != %zu)", size, desc_size);
		return -EINVAL;
	}
	expected_size = sizeof(*desc_header);
	switch (desc_header->type) {
	case GREYBUS_TYPE_STRING:
		expected_size += sizeof(struct greybus_descriptor_string);
		expected_size += desc->string.length;
		expected_size = ALIGN(expected_size, 4);
		break;
	case GREYBUS_TYPE_PROPERTY:
		expected_size += sizeof(struct greybus_descriptor_property);
		expected_size += desc->property.length;
		expected_size = ALIGN(expected_size, 4);
		break;
	case GREYBUS_TYPE_DEVICE:
		expected_size += sizeof(struct greybus_descriptor_device);
		break;
	case GREYBUS_TYPE_MIKROBUS:
		expected_size += sizeof(struct greybus_descriptor_mikrobus);
		break;
	case GREYBUS_TYPE_INTERFACE:
		expected_size += sizeof(struct greybus_descriptor_interface);
		break;
	case GREYBUS_TYPE_CPORT:
		expected_size += sizeof(struct greybus_descriptor_cport);
		break;
	case GREYBUS_TYPE_BUNDLE:
		expected_size += sizeof(struct greybus_descriptor_bundle);
		break;
	case GREYBUS_TYPE_INVALID:
	default:
		pr_err("invalid descriptor type %d", desc_header->type);
		return -EINVAL;
	}

	descriptor = kzalloc(sizeof(*descriptor), GFP_KERNEL);
	if (!descriptor)
		return -ENOMEM;
	descriptor->size = desc_size;
	descriptor->data = (char *)desc + sizeof(*desc_header);
	descriptor->type = desc_header->type;
	list_add_tail(&descriptor->links, &board->manifest_descs);
	return desc_size;
}

static char *mikrobus_string_get(struct addon_board_info *board, u8 string_id)
{
	struct greybus_descriptor_string *desc_string;
	struct manifest_desc *descriptor;
	bool found = false;
	char *string;

	if (!string_id)
		return NULL;

	list_for_each_entry(descriptor, &board->manifest_descs, links) {
		if (descriptor->type != GREYBUS_TYPE_STRING)
			continue;
		desc_string = descriptor->data;
		if (desc_string->id == string_id) {
			found = true;
			break;
		}
	}
	if (!found)
		return ERR_PTR(-ENOENT);
	string = kmemdup(&desc_string->string, desc_string->length + 1, GFP_KERNEL);
	if (!string)
		return ERR_PTR(-ENOMEM);
	string[desc_string->length] = '\0';
	return string;
}

static void mikrobus_state_get(struct addon_board_info *board)
{
	struct greybus_descriptor_mikrobus *mikrobus;
	struct greybus_descriptor_interface *interface;
	struct manifest_desc *descriptor;
	bool found = false;
	int i;

	list_for_each_entry(descriptor, &board->manifest_descs, links) {
		if (descriptor->type == GREYBUS_TYPE_MIKROBUS) {
			mikrobus = descriptor->data;
			found = true;
			break;
		}
	}
	if (!found) {
		pr_err("mikrobus descriptor not found");
		return;
	}
	for (i = 0; i < MIKROBUS_PORT_PIN_COUNT; i++)
		board->pin_state[i] =  mikrobus->pin_state[i];

	found = false;
	list_for_each_entry(descriptor, &board->manifest_descs, links) {
		if (descriptor->type == GREYBUS_TYPE_INTERFACE) {
			interface = descriptor->data;
			found = true;
			break;
		}
	}
	if (!found) {
		pr_err("interface descriptor not found");
		return;
	}
	board->name = mikrobus_string_get(board, interface->product_stringid);
}

static struct property_entry *
mikrobus_property_entry_get(struct addon_board_info *board, u8 *prop_link,
			    int num_properties)
{
	struct greybus_descriptor_property *desc_property;
	struct manifest_desc *descriptor;
	struct property_entry *properties;
	bool found = false;
	char *prop_name;
	u64 *val_u64;
	u32 *val_u32;
	u16 *val_u16;
	u8 *val_u8;
	int i;

	properties = kcalloc(num_properties, sizeof(*properties), GFP_KERNEL);
	if (!properties)
		return ERR_PTR(-ENOMEM);
	for (i = 0; i < num_properties; i++) {
		list_for_each_entry(descriptor, &board->manifest_descs, links) {
			if (descriptor->type != GREYBUS_TYPE_PROPERTY)
				continue;
			desc_property = descriptor->data;
			if (desc_property->id == prop_link[i]) {
				found = true;
				break;
			}
		}
		if (!found) {
			kfree(properties);
			return ERR_PTR(-ENOENT);
		}
		prop_name = mikrobus_string_get(board, desc_property->propname_stringid);
		if (!prop_name) {
			kfree(properties);
			return ERR_PTR(-ENOENT);
		}
		switch (desc_property->type) {
		case MIKROBUS_PROPERTY_TYPE_U8:
			val_u8 = kmemdup(&desc_property->value,
					(desc_property->length) * sizeof(u8),
					GFP_KERNEL);
			if (desc_property->length == 1)
				properties[i] = PROPERTY_ENTRY_U8(prop_name, *val_u8);
			else
				properties[i] = PROPERTY_ENTRY_U8_ARRAY_LEN(prop_name,
					(void *)desc_property->value, desc_property->length);
			break;
		case MIKROBUS_PROPERTY_TYPE_U16:
			val_u16 = kmemdup(&desc_property->value,
					(desc_property->length) * sizeof(u16), GFP_KERNEL);
			if (desc_property->length == 1)
				properties[i] = PROPERTY_ENTRY_U16(prop_name, *val_u16);
			else
				properties[i] = PROPERTY_ENTRY_U16_ARRAY_LEN(prop_name,
					(void *)desc_property->value, desc_property->length);
			break;
		case MIKROBUS_PROPERTY_TYPE_U32:
			val_u32 = kmemdup(&desc_property->value,
						(desc_property->length) * sizeof(u32), GFP_KERNEL);
			if (desc_property->length == 1)
				properties[i] = PROPERTY_ENTRY_U32(prop_name, *val_u32);
			else
				properties[i] = PROPERTY_ENTRY_U32_ARRAY_LEN(prop_name,
					(void *)desc_property->value, desc_property->length);
			break;
		case MIKROBUS_PROPERTY_TYPE_U64:
			val_u64 = kmemdup(&desc_property->value,
					(desc_property->length) * sizeof(u64), GFP_KERNEL);
			if (desc_property->length == 1)
				properties[i] = PROPERTY_ENTRY_U64(prop_name, *val_u64);
			else
				properties[i] = PROPERTY_ENTRY_U64_ARRAY_LEN(prop_name,
					(void *)desc_property->value, desc_property->length);
			break;
		default:
			kfree(properties);
			return ERR_PTR(-EINVAL);
		}
	}
	return properties;
}

static u8 *mikrobus_property_link_get(struct addon_board_info *board, u8 prop_id,
					struct board_device_info *board_dev,  u8 prop_type)
{
	struct greybus_descriptor_property *desc_property;
	struct manifest_desc *descriptor;
	bool found = false;
	u8 *val_u8;

	if (!prop_id)
		return NULL;
	list_for_each_entry(descriptor, &board->manifest_descs, links) {
		if (descriptor->type != GREYBUS_TYPE_PROPERTY)
			continue;
		desc_property = descriptor->data;
		if (desc_property->id == prop_id && desc_property->type == prop_type) {
			found = true;
			break;
		}
	}
	if (!found)
		return ERR_PTR(-ENOENT);
	val_u8 = kmemdup(&desc_property->value, desc_property->length, GFP_KERNEL);
	if (prop_type == MIKROBUS_PROPERTY_TYPE_GPIO)
		board_dev->num_gpio_resources = desc_property->length;
	else if (prop_type == MIKROBUS_PROPERTY_TYPE_PROPERTY)
		board_dev->num_properties = desc_property->length;
	else if (prop_type == MIKROBUS_PROPERTY_TYPE_REGULATOR)
		board_dev->num_regulators = desc_property->length;
	else if (prop_type == MIKROBUS_PROPERTY_TYPE_CLOCK)
		board_dev->num_clocks = desc_property->length;
	return val_u8;
}

static int mikrobus_manifest_attach_device(struct addon_board_info *board,
						struct greybus_descriptor_device *dev_desc)
{
	struct board_device_info *board_dev;
	struct gpiod_lookup_table *lookup;
	struct greybus_descriptor_property *desc_property;
	struct manifest_desc *descriptor;
	u8 *gpio_desc_link;
	u8 *prop_link;
	u8 *reg_link;
	u8 *clock_link;
	u8 *gpioval;
	int retval;
	int i;

	board_dev = kzalloc(sizeof(*board_dev), GFP_KERNEL);
	if (!board_dev)
		return -ENOMEM;
	board_dev->id = dev_desc->id;
	board_dev->drv_name = mikrobus_string_get(board, dev_desc->driver_stringid);
	if (!board_dev->drv_name) {
		retval = -ENOENT;
		goto err_free_board_dev;
	}
	board_dev->protocol = dev_desc->protocol;
	board_dev->reg = dev_desc->reg;
	board_dev->irq = dev_desc->irq;
	board_dev->irq_type = dev_desc->irq_type;
	board_dev->max_speed_hz = le32_to_cpu(dev_desc->max_speed_hz);
	board_dev->mode = dev_desc->mode;
	pr_info("parsed device %d, driver=%s, protocol=%d, reg=%x", board_dev->id, board_dev->drv_name, board_dev->protocol, board_dev->reg);

	if (dev_desc->prop_link > 0) {
		prop_link = mikrobus_property_link_get(board, dev_desc->prop_link,
				board_dev, MIKROBUS_PROPERTY_TYPE_PROPERTY);
		if (!prop_link) {
			retval = -ENOENT;
			goto err_free_board_dev;
		}
		pr_info("device %d, number of properties=%d", board_dev->id,
								board_dev->num_properties);
		board_dev->properties = mikrobus_property_entry_get(board, prop_link,
									board_dev->num_properties);
	}

	if (dev_desc->gpio_link > 0) {
		gpio_desc_link = mikrobus_property_link_get(board, dev_desc->gpio_link, board_dev,
							MIKROBUS_PROPERTY_TYPE_GPIO);
		if (!gpio_desc_link) {
			retval = -ENOENT;
			goto err_free_board_dev;
		}
		pr_info("device %d, number of gpio resource=%d", board_dev->id,
							board_dev->num_gpio_resources);
		lookup = kzalloc(struct_size(lookup, table, board_dev->num_gpio_resources),
					GFP_KERNEL);
		if (!lookup) {
			retval = -ENOMEM;
			goto err_free_board_dev;
		}
		for (i = 0; i < board_dev->num_gpio_resources; i++) {
			list_for_each_entry(descriptor, &board->manifest_descs, links) {
				if (descriptor->type != GREYBUS_TYPE_PROPERTY)
					continue;
				desc_property = descriptor->data;
				if (desc_property->id == gpio_desc_link[i]) {
					gpioval = desc_property->value;
					lookup->table[i].chip_hwnum = gpioval[0];
					lookup->table[i].flags = gpioval[1];
					lookup->table[i].con_id =
					mikrobus_string_get(board,
							desc_property->propname_stringid);
					break;
				}
			}
		}
		board_dev->gpio_lookup = lookup;
	}

	if (dev_desc->reg_link > 0) {
		reg_link = mikrobus_property_link_get(board, dev_desc->reg_link,
				board_dev, MIKROBUS_PROPERTY_TYPE_REGULATOR);
		if (!reg_link) {
			retval = -ENOENT;
			goto err_free_board_dev;
		}
		pr_info("device %d, number of regulators=%d", board_dev->id,
								board_dev->num_regulators);
		board_dev->regulators = mikrobus_property_entry_get(board, reg_link,
									board_dev->num_regulators);
	}

	if (dev_desc->clock_link > 0) {
		clock_link = mikrobus_property_link_get(board, dev_desc->clock_link,
				board_dev, MIKROBUS_PROPERTY_TYPE_CLOCK);
		if (!clock_link) {
			retval = -ENOENT;
			goto err_free_board_dev;
		}
		pr_info("device %d, number of clocks=%d", board_dev->id,
								board_dev->num_clocks);
		board_dev->clocks = mikrobus_property_entry_get(board, clock_link,
									board_dev->num_clocks);
	}
	list_add_tail(&board_dev->links, &board->devices);
	return 0;
err_free_board_dev:
	kfree(board_dev);
	return retval;
}

static int mikrobus_manifest_parse_devices(struct addon_board_info *board)
{
	struct greybus_descriptor_device *desc_device;
	struct manifest_desc *desc, *next;
	int retval;
	int devcount = 0;

	list_for_each_entry_safe(desc, next, &board->manifest_descs, links) {
		if (desc->type != GREYBUS_TYPE_DEVICE)
			continue;
		desc_device = desc->data;
		retval = mikrobus_manifest_attach_device(board, desc_device);
		devcount++;
	}
	return devcount;
}

int mikrobus_manifest_parse(struct addon_board_info *board, void *data,
							 size_t size)
{
	struct greybus_manifest_header *header;
	struct greybus_manifest *manifest;
	struct greybus_descriptor *desc;
	u16 manifest_size;
	int dev_count;
	int desc_size;

	if (size < sizeof(*header)) {
		pr_err("short manifest (%zu < %zu)", size, sizeof(*header));
		return -EINVAL;
	}

	manifest = data;
	header = &manifest->header;
	manifest_size = le16_to_cpu(header->size);

	if (manifest_size != size) {
		pr_err("invalid manifest size(%zu < %u)", size, manifest_size);
		return -EINVAL;
	}

	if (header->version_major > MIKROBUS_VERSION_MAJOR) {
		pr_err("manifest version too new (%u.%u > %u.%u)",
		header->version_major, header->version_minor,
		MIKROBUS_VERSION_MAJOR, MIKROBUS_VERSION_MINOR);
		return -EINVAL;
	}

	desc = manifest->descriptors;
	size -= sizeof(*header);
	while (size) {
		desc_size = board_descriptor_add(board, desc, size);
		if (desc_size < 0) {
			pr_err("invalid manifest descriptor, size: %u", desc_size);
			return -EINVAL;
		}
		desc = (void *)desc + desc_size;
		size -= desc_size;
	}
	mikrobus_state_get(board);
	dev_count = mikrobus_manifest_parse_devices(board);
	pr_info(" %s manifest parsed with %d devices", board->name, dev_count);
	manifest_descriptor_release_all(board);
	return true;
}

size_t mikrobus_manifest_header_validate(void *data, size_t size)
{
	struct greybus_manifest_header *header;
	u16 manifest_size;

	if (size < sizeof(*header)) {
		pr_err("short manifest (%zu < %zu)", size, sizeof(*header));
		return -EINVAL;
	}
	header = data;
	manifest_size = le16_to_cpu(header->size);
	if (header->version_major > MIKROBUS_VERSION_MAJOR) {
		pr_err("manifest version too new (%u.%u > %u.%u)",
		header->version_major, header->version_minor,
		MIKROBUS_VERSION_MAJOR, MIKROBUS_VERSION_MINOR);
		return -EINVAL;
	}
	return manifest_size;
}

