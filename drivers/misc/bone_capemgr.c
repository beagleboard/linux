/*
 * TI Beaglebone cape manager
 *
 * Copyright (C) 2012 Texas Instruments Inc.
 * Copyright (C) 2012-2015 Konsulko Group.
 * Author: Pantelis Antoniou <pantelis.antoniou@konsulko.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_fdt.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/pinctrl/consumer.h>
#include <linux/firmware.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/memory.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/nvmem-consumer.h>

/* disabled capes */
static char *disable_partno;
module_param(disable_partno, charp, 0444);
MODULE_PARM_DESC(disable_partno,
		"Comma delimited list of PART-NUMBER[:REV] of disabled capes");

/* enable capes */
static char *enable_partno;
module_param(enable_partno, charp, 0444);
MODULE_PARM_DESC(enable_partno,
		"Comma delimited list of PART-NUMBER[:REV] of enabled capes");

/* delay to scan on boot until rootfs appears */
static int boot_scan_period = 1000;
module_param(boot_scan_period, int, 0444);
MODULE_PARM_DESC(boot_scan_period,
		"boot scan period until rootfs firmware is available");

static int uboot_capemgr_enabled = 0;
module_param(uboot_capemgr_enabled, int, 0444);
MODULE_PARM_DESC(uboot_capemgr_enabled,
		 "U-Boot Cape Manager is enabled (0=Kernel Cape Manager [default], 1=Disable Kernel Cape Manager)");

struct capemgr_info;

struct slot_ee_attribute {
	struct device_attribute devattr;
	unsigned int field;
	struct bone_cape_slot *slot;	/* this is filled when instantiated */
};
#define to_slot_ee_attribute(x) \
	container_of((x), struct slot_ee_attribute, devattr)

struct bbrd_ee_attribute {
	struct device_attribute devattr;
	unsigned int field;
};
#define to_bbrd_ee_attribute(x) \
	container_of((x), struct bbrd_ee_attribute, devattr)

struct bone_cape_slot {
	struct list_head	node;
	struct capemgr_info	*info;
	int			slotno;
	struct nvmem_cell	*nvmem_cell;

	char			text_id[256];
	char			signature[256];
	/* quick access */
	char			board_name[32+1];
	char			version[4+1];
	char			manufacturer[16+1];
	char			part_number[16+1];

	/* attribute group */
	char			*ee_attr_name;
	int			ee_attrs_count;
	struct slot_ee_attribute *ee_attrs;
	struct attribute	**ee_attrs_tab;
	struct attribute_group	attrgroup;

	/* state flags */
	unsigned int		probed : 1;
	unsigned int		probe_failed : 1;
	unsigned int		override : 1;
	unsigned int		loading : 1;
	unsigned int		loaded : 1;
	unsigned int		retry_loading : 1;
	unsigned int		disabled : 1;

	char			*dtbo;
	const struct firmware	*fw;
	struct device_node	*overlay;
	int			overlay_id;

	/* loader thread */
	struct task_struct	*loader_thread;

	/* load priority */
	int priority;
};

struct bone_baseboard {

	/* from the matched boardmap node */
	char			*compatible_name;

	/* filled in by reading the eeprom */
	char			signature[256];
	char			text_id[64+1];

	/* quick access */
	char			board_name[8+1];
	char			revision[4+1];
	char			serial_number[12+1];

	/* access to the eeprom */
	struct nvmem_cell	*nvmem_cell;
};

struct capemgr_info {
	struct platform_device	*pdev;

	atomic_t next_slot_nr;
	struct list_head	slot_list;
	struct mutex		slots_list_mutex;

	/* baseboard EEPROM data */
	struct bone_baseboard	baseboard;

	/* wait queue for keeping the priorities straight */
	wait_queue_head_t	load_wq;
};

static int bone_slot_fill_override(struct bone_cape_slot *slot,
		const char *part_number, const char *version);
static struct bone_cape_slot *capemgr_add_slot(
		struct capemgr_info *info, const char *slot_name,
		const char *part_number, const char *version, int prio);
static int capemgr_remove_slot_no_lock(struct bone_cape_slot *slot);
static int capemgr_remove_slot(struct bone_cape_slot *slot);
static int capemgr_load_slot(struct bone_cape_slot *slot);
static int capemgr_unload_slot(struct bone_cape_slot *slot);

/* baseboard EEPROM field definition */
#define BBRD_EE_FIELD_HEADER		0
#define BBRD_EE_FIELD_BOARD_NAME	1
#define BBRD_EE_FIELD_REVISION		2
#define BBRD_EE_FIELD_SERIAL_NUMBER	3
#define BBRD_EE_FIELD_CONFIG_OPTION	4
#define BBRD_EE_FILED_RSVD1		5
#define BBRD_EE_FILED_RSVD2		6
#define BBRD_EE_FILED_RSVD3		7

/* cape EEPROM field definitions */
#define CAPE_EE_FIELD_HEADER		0
#define CAPE_EE_FIELD_EEPROM_REV	1
#define CAPE_EE_FIELD_BOARD_NAME	2
#define CAPE_EE_FIELD_VERSION		3
#define CAPE_EE_FIELD_MANUFACTURER	4
#define CAPE_EE_FIELD_PART_NUMBER	5
#define CAPE_EE_FIELD_NUMBER_OF_PINS	6
#define CAPE_EE_FIELD_SERIAL_NUMBER	7
#define CAPE_EE_FIELD_PIN_USAGE		8
#define CAPE_EE_FIELD_VDD_3V3EXP	9
#define CAPE_EE_FIELD_VDD_5V		10
#define CAPE_EE_FIELD_SYS_5V		11
#define CAPE_EE_FIELD_DC_SUPPLIED	12
#define CAPE_EE_FIELD_FIELDS_NR		13

#define EE_FIELD_MAKE_HEADER(p)	\
	({ \
		const u8 *_p = (p); \
		(((u32)_p[0] << 24) | ((u32)_p[1] << 16) | \
		 ((u32)_p[2] <<  8) |  (u32)_p[3]); \
	})

#define EE_FIELD_HEADER_VALID	0xaa5533ee

struct ee_field {
	const char	*name;
	int		start;
	int		size;
	unsigned int	ascii : 1;
	unsigned int	strip_trailing_dots : 1;
	const char	*override;
};

/* baseboard EEPROM definitions */
static const struct ee_field bbrd_sig_fields[] = {
	[BBRD_EE_FIELD_HEADER] = {
		.name		= "header",
		.start		= 0,
		.size		= 4,
		.ascii		= 0,
		.override	= "\xaa\x55\x33\xee",	/* AA 55 33 EE */
	},
	[BBRD_EE_FIELD_BOARD_NAME] = {
		.name		= "board-name",
		.start		= 4,
		.size		= 8,
		.ascii		= 1,
		.strip_trailing_dots = 1,
		.override	= "Board Name",
	},
	[BBRD_EE_FIELD_REVISION] = {
		.name		= "revision",
		.start		= 12,
		.size		= 4,
		.ascii		= 1,
		.override	= "00A0",
	},
	[BBRD_EE_FIELD_SERIAL_NUMBER] = {
		.name		= "serial-number",
		.start		= 16,
		.size		= 12,
		.ascii		= 1,
		.override	= "0000000000",
	},
	[BBRD_EE_FIELD_CONFIG_OPTION] = {
		.name		= "config-option",
		.start		= 28,
		.size		= 32,
	},
};

/* cape EEPROM definitions */
static const struct ee_field cape_sig_fields[] = {
	[CAPE_EE_FIELD_HEADER] = {
		.name		= "header",
		.start		= 0,
		.size		= 4,
		.ascii		= 0,
		.override	= "\xaa\x55\x33\xee",	/* AA 55 33 EE */
	},
	[CAPE_EE_FIELD_EEPROM_REV] = {
		.name		= "eeprom-format-revision",
		.start		= 4,
		.size		= 2,
		.ascii		= 1,
		.override	= "A0",
	},
	[CAPE_EE_FIELD_BOARD_NAME] = {
		.name		= "board-name",
		.start		= 6,
		.size		= 32,
		.ascii		= 1,
		.strip_trailing_dots = 1,
		.override	= "Override Board Name",
	},
	[CAPE_EE_FIELD_VERSION] = {
		.name		= "version",
		.start		= 38,
		.size		= 4,
		.ascii		= 1,
		.override	= "00A0",
	},
	[CAPE_EE_FIELD_MANUFACTURER] = {
		.name		= "manufacturer",
		.start		= 42,
		.size		= 16,
		.ascii		= 1,
		.strip_trailing_dots = 1,
		.override	= "Override Manuf",
	},
	[CAPE_EE_FIELD_PART_NUMBER] = {
		.name		= "part-number",
		.start		= 58,
		.size		= 16,
		.ascii		= 1,
		.strip_trailing_dots = 1,
		.override	= "Override Part#",
	},
	[CAPE_EE_FIELD_NUMBER_OF_PINS] = {
		.name		= "number-of-pins",
		.start		= 74,
		.size		= 2,
		.ascii		= 0,
		.override	= NULL,
	},
	[CAPE_EE_FIELD_SERIAL_NUMBER] = {
		.name		= "serial-number",
		.start		= 76,
		.size		= 12,
		.ascii		= 1,
		.override	= "0000000000",
	},
	[CAPE_EE_FIELD_PIN_USAGE] = {
		.name		= "pin-usage",
		.start		= 88,
		.size		= 140,
		.ascii		= 0,
		.override	= NULL,
	},
	[CAPE_EE_FIELD_VDD_3V3EXP] = {
		.name		= "vdd-3v3exp",
		.start		= 228,
		.size		= 2,
		.ascii		= 0,
		.override	= NULL,
	},
	[CAPE_EE_FIELD_VDD_5V] = {
		.name		= "vdd-5v",
		.start		= 230,
		.size		= 2,
		.ascii		= 0,
		.override	= NULL,
	},
	[CAPE_EE_FIELD_SYS_5V] = {
		.name		= "sys-5v",
		.start		= 232,
		.size		= 2,
		.ascii		= 0,
		.override	= NULL,
	},
	[CAPE_EE_FIELD_DC_SUPPLIED] = {
		.name		= "dc-supplied",
		.start		= 234,
		.size		= 2,
		.ascii		= 0,
		.override	= NULL,
	},
};

static char *ee_field_get(const struct ee_field *sig_field,
		const void *data, int field, char *buf, int bufsz)
{
	int len;

	/* enough space? */
	if (bufsz < sig_field->size + sig_field->ascii)
		return NULL;

	memcpy(buf, (char *)data + sig_field->start, sig_field->size);

	/* terminate ascii field */
	if (sig_field->ascii)
		buf[sig_field->size] = '\0';

	if (sig_field->strip_trailing_dots) {
		len = strlen(buf);
		while (len > 1 && buf[len - 1] == '.')
			buf[--len] = '\0';
	}

	return buf;
}

char *bbrd_ee_field_get(const void *data,
		int field, char *buf, int bufsz)
{
	if ((unsigned int)field >= ARRAY_SIZE(bbrd_sig_fields))
		return NULL;

	return ee_field_get(&bbrd_sig_fields[field], data, field, buf, bufsz);
}

char *cape_ee_field_get(const void *data,
		int field, char *buf, int bufsz)
{
	if ((unsigned int)field >= ARRAY_SIZE(cape_sig_fields))
		return NULL;

	return ee_field_get(&cape_sig_fields[field], data, field, buf, bufsz);
}

#ifdef CONFIG_OF
static const struct of_device_id capemgr_of_match[] = {
	{
		.compatible = "ti,bone-capemgr",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, capemgr_of_match);

#endif

static int bone_baseboard_scan(struct bone_baseboard *bbrd)
{
	struct capemgr_info *info = container_of(bbrd,
			struct capemgr_info, baseboard);
	const u8 *p;
	int ret;
	size_t len;

	p = nvmem_cell_read(bbrd->nvmem_cell, &len);
	if (IS_ERR(p)) {
		ret = PTR_ERR(p);
		dev_err(&info->pdev->dev,
			"Cannot read cell (ret=%d)\n", ret);
		return ret;
	}
	if (len < sizeof(bbrd->signature)) {
		dev_info(&info->pdev->dev,
			"Short read %d (should be >= %d bytes)\n",
			len, sizeof(bbrd->signature));
		return -EINVAL;
	}
	memcpy(bbrd->signature, p, sizeof(bbrd->signature));

	p = bbrd->signature;
	if (EE_FIELD_MAKE_HEADER(p) != EE_FIELD_HEADER_VALID) {
		dev_err(&info->pdev->dev, "Invalid board signature '%08x'\n",
			EE_FIELD_MAKE_HEADER(p));
		return -ENODEV;
	}

	bbrd_ee_field_get(bbrd->signature,
			BBRD_EE_FIELD_BOARD_NAME,
			bbrd->board_name, sizeof(bbrd->board_name));
	bbrd_ee_field_get(bbrd->signature,
			BBRD_EE_FIELD_REVISION,
			bbrd->revision, sizeof(bbrd->revision));
	bbrd_ee_field_get(bbrd->signature,
			BBRD_EE_FIELD_SERIAL_NUMBER,
			bbrd->serial_number, sizeof(bbrd->serial_number));

	/* board_name,version,manufacturer,part_number */
	snprintf(bbrd->text_id, sizeof(bbrd->text_id) - 1,
			"%s,%s,%s", bbrd->board_name, bbrd->revision,
			bbrd->serial_number);

	/* terminate always */
	bbrd->text_id[sizeof(bbrd->text_id) - 1] = '\0';

	return 0;
}

static int bone_slot_scan(struct bone_cape_slot *slot)
{
	struct capemgr_info *info = slot->info;
	const u8 *p;
	int r;
	ssize_t len;

	/* need to read EEPROM? */
	if (slot->probed)
		goto slot_fail_check;

	if (uboot_capemgr_enabled)
		goto slot_fail_check;

	slot->probed = 1;

	if (!slot->override) {

		p = nvmem_cell_read(slot->nvmem_cell, &len);
		if (IS_ERR(p)) {
			r = PTR_ERR(p);
			slot->probe_failed = 1;

			/* timeout is normal when no cape is present */
			if (r != -ETIMEDOUT)
				dev_err(&info->pdev->dev,
					"Cannot read cell (ret=%d)\n", r);
			return r;
		}
		if (len < sizeof(slot->signature)) {
			dev_info(&info->pdev->dev,
				"Short read %d (should be >= %d bytes)\n",
				len, sizeof(slot->signature));
			return -EINVAL;
		}
		memcpy(slot->signature, p, sizeof(slot->signature));

	} else
		dev_info(&info->pdev->dev,
			"Using override eeprom data at slot %d\n",
			slot->slotno);

	p = slot->signature;
	if (EE_FIELD_MAKE_HEADER(p) != EE_FIELD_HEADER_VALID) {
		dev_err(&info->pdev->dev,
			"Invalid signature '%08x' at slot %d\n",
			EE_FIELD_MAKE_HEADER(p), slot->slotno);
		slot->probe_failed = 1;
		return -ENODEV;
	}

	cape_ee_field_get(slot->signature,
			CAPE_EE_FIELD_BOARD_NAME,
			slot->board_name, sizeof(slot->board_name));
	cape_ee_field_get(slot->signature,
			CAPE_EE_FIELD_VERSION,
			slot->version, sizeof(slot->version));
	cape_ee_field_get(slot->signature,
			CAPE_EE_FIELD_MANUFACTURER,
			slot->manufacturer, sizeof(slot->manufacturer));
	cape_ee_field_get(slot->signature,
			CAPE_EE_FIELD_PART_NUMBER,
			slot->part_number, sizeof(slot->part_number));

	/* board_name,version,manufacturer,part_number */
	snprintf(slot->text_id, sizeof(slot->text_id) - 1,
			"%s,%s,%s,%s", slot->board_name, slot->version,
			slot->manufacturer, slot->part_number);

	/* terminate always */
	slot->text_id[sizeof(slot->text_id) - 1] = '\0';

slot_fail_check:
	/* slot has failed and we don't support hotpluging */
	if (slot->probe_failed)
		return -ENODEV;

	return 0;
}

/* return 0 if not matched,, 1 if matched */
static int bone_match_cape(const char *match,
		const char *part_number, const char *version)
{
	char *tmp_part_number, *tmp_version;
	char *buf, *s, *e, *sn;
	int found;

	if (match == NULL || part_number == NULL)
		return 0;

	/* copy the argument to work on it */
	buf = kstrdup(match, GFP_KERNEL);

	/* no memory, too bad... */
	if (buf == NULL)
		return 0;

	found = 0;
	s = buf;
	e = s + strlen(s);
	while (s < e) {
		/* find comma separator */
		sn = strchr(s, ',');
		if (sn != NULL)
			*sn++ = '\0';
		else
			sn = e;
		tmp_part_number = s;
		tmp_version = strchr(tmp_part_number, ':');
		if (tmp_version != NULL)
			*tmp_version++ = '\0';
		s = sn;

		/* the part names must match */
		if (strcmp(tmp_part_number, part_number) != 0)
			continue;

		/* if there's no version, match any */
		if (version == NULL || tmp_version == NULL ||
			strcmp(version, tmp_version) == 0) {
			found = 1;
			break;
		}
	}

	kfree(buf);

	return found;
}

/* helper method */
static int of_multi_prop_cmp(const struct property *prop, const char *value)
{
	const char *cp;
	int cplen, vlen, l;

	/* check if it's directly compatible */
	cp = prop->value;
	cplen = prop->length;
	vlen = strlen(value);

	while (cplen > 0) {
		/* compatible? */
		if (of_compat_cmp(cp, value, vlen) == 0)
			return 0;
		l = strlen(cp) + 1;
		cp += l;
		cplen -= l;
	}
	return -1;
}

static ssize_t slot_ee_attr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct slot_ee_attribute *ee_attr = to_slot_ee_attribute(attr);
	struct bone_cape_slot *slot = ee_attr->slot;
	const struct ee_field *sig_field;
	int i, len;
	char *p, *s;
	u16 val;

	/* add newline for ascii fields */
	sig_field = &cape_sig_fields[ee_attr->field];

	len = sig_field->size + sig_field->ascii;
	p = kmalloc(len, GFP_KERNEL);
	if (p == NULL)
		return -ENOMEM;

	s = cape_ee_field_get(slot->signature, ee_attr->field, p, len);
	if (s == NULL)
		return -EINVAL;

	/* add newline for ascii fields and return */
	if (sig_field->ascii) {
		len = sprintf(buf, "%s\n", s);
		goto out;
	}

	/* case by case handling */
	switch (ee_attr->field) {
	case CAPE_EE_FIELD_HEADER:
		len = sprintf(buf, "%02x %02x %02x %02x\n",
				s[0], s[1], s[2], s[3]);
		break;

		/* 2 bytes */
	case CAPE_EE_FIELD_NUMBER_OF_PINS:
	case CAPE_EE_FIELD_VDD_3V3EXP:
	case CAPE_EE_FIELD_VDD_5V:
	case CAPE_EE_FIELD_SYS_5V:
	case CAPE_EE_FIELD_DC_SUPPLIED:
		/* the bone is LE */
		val = s[0] & (s[1] << 8);
		len = sprintf(buf, "%u\n", (unsigned int)val & 0xffff);
		break;

	case CAPE_EE_FIELD_PIN_USAGE:

		len = 0;
		for (i = 0; i < sig_field->size / 2; i++) {
			/* the bone is LE */
			val = s[0] & (s[1] << 8);
			sprintf(buf, "%04x\n", val);
			buf += 5;
			len += 5;
			s += 2;
		}

		break;

	default:
		*buf = '\0';
		len = 0;
		break;
	}

out:
	kfree(p);

	return len;
}

#define SLOT_EE_ATTR(_name, _field) \
	{ \
		.devattr = __ATTR(_name, S_IRUGO, slot_ee_attr_show, NULL), \
		.field = CAPE_EE_FIELD_##_field, \
		.slot = NULL, \
	}

static const struct slot_ee_attribute slot_ee_attrs[] = {
	SLOT_EE_ATTR(header, HEADER),
	SLOT_EE_ATTR(eeprom-format-revision, EEPROM_REV),
	SLOT_EE_ATTR(board-name, BOARD_NAME),
	SLOT_EE_ATTR(version, VERSION),
	SLOT_EE_ATTR(manufacturer, MANUFACTURER),
	SLOT_EE_ATTR(part-number, PART_NUMBER),
	SLOT_EE_ATTR(number-of-pins, NUMBER_OF_PINS),
	SLOT_EE_ATTR(serial-number, SERIAL_NUMBER),
	SLOT_EE_ATTR(pin-usage, PIN_USAGE),
	SLOT_EE_ATTR(vdd-3v3exp, VDD_3V3EXP),
	SLOT_EE_ATTR(vdd-5v, VDD_5V),
	SLOT_EE_ATTR(sys-5v, SYS_5V),
	SLOT_EE_ATTR(dc-supplied, DC_SUPPLIED),
};

static int bone_cape_slot_sysfs_register(struct bone_cape_slot *slot)
{
	struct capemgr_info *info = slot->info;
	struct device *dev = &info->pdev->dev;
	struct slot_ee_attribute *ee_attr;
	struct attribute_group *attrgroup;
	int i, err, sz;

	slot->ee_attr_name = kasprintf(GFP_KERNEL, "slot-%d", slot->slotno);
	if (slot->ee_attr_name == NULL) {
		dev_err(dev, "slot #%d: Failed to allocate ee_attr_name\n",
				slot->slotno);
		err = -ENOMEM;
		goto err_fail_no_ee_attr_name;
	}

	slot->ee_attrs_count = ARRAY_SIZE(slot_ee_attrs);

	sz = slot->ee_attrs_count * sizeof(*slot->ee_attrs);
	slot->ee_attrs = kmalloc(sz, GFP_KERNEL);
	if (slot->ee_attrs == NULL) {
		dev_err(dev, "slot #%d: Failed to allocate ee_attrs\n",
				slot->slotno);
		err = -ENOMEM;
		goto err_fail_no_ee_attrs;
	}

	attrgroup = &slot->attrgroup;
	memset(attrgroup, 0, sizeof(*attrgroup));
	attrgroup->name = slot->ee_attr_name;

	sz = sizeof(*slot->ee_attrs_tab) * (slot->ee_attrs_count + 1);
	attrgroup->attrs = kmalloc(sz, GFP_KERNEL);
	if (attrgroup->attrs == NULL) {
		dev_err(dev, "slot #%d: Failed to allocate ee_attrs_tab\n",
				slot->slotno);
		err = -ENOMEM;
		goto err_fail_no_ee_attrs_tab;
	}
	/* copy everything over */
	memcpy(slot->ee_attrs, slot_ee_attrs, sizeof(slot_ee_attrs));

	/* bind this attr to the slot */
	for (i = 0; i < slot->ee_attrs_count; i++) {
		ee_attr = &slot->ee_attrs[i];
		ee_attr->slot = slot;
		attrgroup->attrs[i] = &ee_attr->devattr.attr;
	}
	attrgroup->attrs[i] = NULL;

	/* make lockdep happy */
	for (i = 0; i < slot->ee_attrs_count; i++) {
		ee_attr = &slot->ee_attrs[i];
		sysfs_attr_init(&ee_attr->devattr.attr);
	}

	err = sysfs_create_group(&dev->kobj, attrgroup);
	if (err != 0) {
		dev_err(dev, "slot #%d: Failed to allocate ee_attrs_tab\n",
				slot->slotno);
		err = -ENOMEM;
		goto err_fail_no_ee_attrs_group;
	}

	return 0;

err_fail_no_ee_attrs_group:
	kfree(slot->ee_attrs_tab);
err_fail_no_ee_attrs_tab:
	kfree(slot->ee_attrs);
err_fail_no_ee_attrs:
	kfree(slot->ee_attr_name);
err_fail_no_ee_attr_name:
	return err;
}

static void bone_cape_slot_sysfs_unregister(struct bone_cape_slot *slot)
{
	struct capemgr_info *info = slot->info;
	struct device *dev = &info->pdev->dev;

	sysfs_remove_group(&dev->kobj, &slot->attrgroup);
	kfree(slot->ee_attrs_tab);
	kfree(slot->ee_attrs);
	kfree(slot->ee_attr_name);
}

static ssize_t bbrd_ee_attr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bbrd_ee_attribute *ee_attr = to_bbrd_ee_attribute(attr);
	struct platform_device *pdev = to_platform_device(dev);
	struct capemgr_info *info = platform_get_drvdata(pdev);
	struct bone_baseboard *bbrd = &info->baseboard;
	const struct ee_field *sig_field;
	u16 val;
	int i, len;
	char *p, *s;

	/* add newline for ascii fields */
	sig_field = &bbrd_sig_fields[ee_attr->field];

	len = sig_field->size + sig_field->ascii;
	p = kmalloc(len, GFP_KERNEL);
	if (p == NULL)
		return -ENOMEM;

	s = bbrd_ee_field_get(bbrd->signature, ee_attr->field, p, len);
	if (s == NULL)
		return -EINVAL;

	/* add newline for ascii fields and return */
	if (sig_field->ascii) {
		len = sprintf(buf, "%s\n", s);
		goto out;
	}

	/* case by case handling */
	switch (ee_attr->field) {
	case BBRD_EE_FIELD_HEADER:
		len = sprintf(buf, "%02x %02x %02x %02x\n",
				s[0], s[1], s[2], s[3]);
		break;

	case BBRD_EE_FIELD_CONFIG_OPTION:
		len = 0;
		for (i = 0; i < sig_field->size / 2; i++) {
			/* the bone is LE */
			val = s[0] & (s[1] << 8);
			sprintf(buf, "%04x\n", val);
			buf += 5;
			len += 5;
			s += 2;
		}
		break;

	default:
		*buf = '\0';
		len = 0;
		break;
	}

out:
	kfree(p);

	return len;
}

#define BBRD_EE_ATTR(_name, _field) \
	{ \
		.devattr = __ATTR(_name, 0440, bbrd_ee_attr_show, NULL), \
		.field = BBRD_EE_FIELD_##_field, \
	}

static struct bbrd_ee_attribute bbrd_ee_attrs[] = {
	BBRD_EE_ATTR(header, HEADER),
	BBRD_EE_ATTR(board-name, BOARD_NAME),
	BBRD_EE_ATTR(revision, REVISION),
	BBRD_EE_ATTR(serial-number, SERIAL_NUMBER),
	BBRD_EE_ATTR(config-option, CONFIG_OPTION),
};

static struct attribute *bbrd_attrs_flat[] = {
	&bbrd_ee_attrs[BBRD_EE_FIELD_HEADER].devattr.attr,
	&bbrd_ee_attrs[BBRD_EE_FIELD_BOARD_NAME].devattr.attr,
	&bbrd_ee_attrs[BBRD_EE_FIELD_REVISION].devattr.attr,
	&bbrd_ee_attrs[BBRD_EE_FIELD_SERIAL_NUMBER].devattr.attr,
	&bbrd_ee_attrs[BBRD_EE_FIELD_CONFIG_OPTION].devattr.attr,
	NULL,
};

static const struct attribute_group bbrd_attr_group = {
	.name	= "baseboard",
	.attrs	= bbrd_attrs_flat,
};

static ssize_t slots_show(struct device *dev, struct device_attribute *attr,
		char *buf);
static ssize_t slots_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count);

static DEVICE_ATTR(slots, 0644, slots_show, slots_store);

static struct attribute *root_attrs_flat[] = {
	&dev_attr_slots.attr,
	NULL,
};

static const struct attribute_group root_attr_group = {
	.attrs = root_attrs_flat,
};

static const struct attribute_group *attr_groups[] = {
	&root_attr_group,
	&bbrd_attr_group,
	NULL,
};

static ssize_t slots_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct capemgr_info *info = platform_get_drvdata(pdev);
	struct bone_cape_slot *slot;
	ssize_t len, sz;

	mutex_lock(&info->slots_list_mutex);
	sz = 0;
	list_for_each_entry(slot, &info->slot_list, node) {

		len = sprintf(buf, "%2d: %c%c%c%c%c%c %3d %s\n",
				slot->slotno,
				slot->probed       ? 'P' : '-',
				slot->probe_failed ? 'F' : '-',
				slot->override     ? 'O' : '-',
				slot->loading	   ? 'l' : '-',
				slot->loaded	   ? 'L' : '-',
				slot->disabled     ? 'D' : '-',
				slot->overlay_id, slot->text_id);

		buf += len;
		sz += len;
	}
	mutex_unlock(&info->slots_list_mutex);

	return sz;
}

static ssize_t slots_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct capemgr_info *info = platform_get_drvdata(pdev);
	struct bone_cape_slot *slot;
	struct device_node *pnode, *node;
	char *s, *part_number, *version;
	int ret;
	int slotno;

	/* check for remove slot */
	if (strlen(buf) > 0 && buf[0] == '-') {
		ret = kstrtoint(buf + 1, 10, &slotno);
		if (ret != 0)
			return ret;

		/* now load each (take lock to be sure */
		mutex_lock(&info->slots_list_mutex);
		list_for_each_entry(slot, &info->slot_list, node) {
			if (slotno == slot->slotno)
				goto found;
		}

		mutex_unlock(&info->slots_list_mutex);
		return -ENODEV;
found:
		/* the hardware slots just get unloaded */
		if (!slot->override) {
			ret = capemgr_unload_slot(slot);
			if (ret == 0)
				dev_info(&pdev->dev,
					"Unloaded slot #%d\n", slotno);
			else
				dev_err(&pdev->dev,
					"Failed to unload slot #%d\n", slotno);
		} else {
			ret = capemgr_remove_slot_no_lock(slot);
			if (ret == 0)
				dev_info(&pdev->dev,
					"Removed slot #%d\n", slotno);
			else
				dev_err(&pdev->dev,
					"Failed to remove slot #%d\n", slotno);
		}
		mutex_unlock(&info->slots_list_mutex);

		return ret == 0 ? strlen(buf) : ret;
	}

	part_number = kstrdup(buf, GFP_KERNEL);
	if (part_number == NULL)
		return -ENOMEM;

	/* remove trailing spaces dots and newlines */
	s = part_number + strlen(part_number);
	while (s > part_number &&
			(isspace(s[-1]) || s[-1] == '\n' || s[-1] == '.'))
		*--s = '\0';

	version = strchr(part_number, ':');
	if (version != NULL)
		*version++ = '\0';

	dev_info(&pdev->dev, "part_number '%s', version '%s'\n",
			part_number, version ? version : "N/A");

	pnode = pdev->dev.of_node;
	node = NULL;
	slot = NULL;
	ret = 0;

	/* no specific slot found, try immediate */
	slot = capemgr_add_slot(info, NULL, part_number, version, 0);

	if (IS_ERR_OR_NULL(slot)) {
		dev_err(&pdev->dev, "Failed to add slot #%d\n",
			atomic_read(&info->next_slot_nr) - 1);
		ret = slot ? PTR_ERR(slot) : -ENODEV;
		slot = NULL;
		goto err_fail;
	}

	kfree(part_number);

	ret = capemgr_load_slot(slot);
	if (ret != 0)
		capemgr_remove_slot(slot);

	return ret == 0 ? strlen(buf) : ret;
err_fail:
	of_node_put(node);
	kfree(part_number);
	return ret;
}

/* verify the overlay */
static int capemgr_verify_overlay(struct bone_cape_slot *slot)
{
	struct capemgr_info *info = slot->info;
	struct device *dev = &info->pdev->dev;
	struct bone_baseboard *bbrd = &info->baseboard;
	struct device_node *node = slot->overlay;
	struct property *prop;
	struct bone_cape_slot *slotn;
	int err, counta, countb, i, j;
	const char *ra, *rb;

	/* validate */
	if (node == NULL) {
		dev_err(dev, "slot #%d: No overlay for '%s'\n",
				slot->slotno, slot->part_number);
		return -EINVAL;
	}

	/* check if the slot is compatible with the board */
	prop = of_find_property(node, "compatible", NULL);

	/* no compatible property? */
	if (prop == NULL) {
		dev_err(dev, "slot #%d: No compatible property for '%s'\n",
				slot->slotno, slot->part_number);
		return -EINVAL;
	}

	/* verify that the cape is baseboard compatible */
	if (of_multi_prop_cmp(prop, bbrd->compatible_name) != 0) {
		dev_err(dev, "slot #%d: Incompatible with baseboard for '%s'\n",
				slot->slotno, slot->part_number);
		return -EINVAL;
	}

	/* count the strings */
	counta = of_property_count_strings(node, "exclusive-use");
	/* no valid property, or no resources; no matter, it's OK */
	if (counta <= 0)
		return 0;

	/* and now check if there's a resource conflict */
	err = 0;
	mutex_lock(&info->slots_list_mutex);
	for (i = 0; i < counta; i++) {

		ra = NULL;
		err = of_property_read_string_index(node, "exclusive-use",
				i, &ra);
		if (err != 0) {
			dev_err(dev, "slot #%d: Could not read string #%d\n",
					slot->slotno, i);
			break;
		}

		list_for_each_entry(slotn, &info->slot_list, node) {

			/* don't check against self */
			if (slot == slotn)
				continue;

			/* only check against loaded or loading slots */
			if (!slotn->loaded && !slotn->loading)
				continue;

			countb = of_property_count_strings(slotn->overlay,
					"exclusive-use");
			/* no valid property, or resources; it's OK */
			if (countb <= 0)
				continue;


			for (j = 0; j < countb; j++) {

				/* count the resources */
				rb = NULL;
				err = of_property_read_string_index(
					slotn->overlay, "exclusive-use",
						j, &rb);
				if (err != 0) {
					/* error, but we don't care */
					err = 0;
					break;
				}

				/* ignore case; just in case ;) */
				if (strcasecmp(ra, rb) == 0) {

					/* resource conflict */
					err = -EEXIST;
					dev_err(dev,
						"slot #%d: %s conflict %s (#%d:%s)\n",
						slot->slotno,
						slot->part_number, ra,
						slotn->slotno,
						slotn->part_number);
					goto out;
				}
			}
		}
	}
out:
	mutex_unlock(&info->slots_list_mutex);

	return err;
}

static int capemgr_load_slot(struct bone_cape_slot *slot)
{
	struct capemgr_info *info = slot->info;
	struct device *dev = &info->pdev->dev;
	const char *dtbo;
	int err;

	if (slot->probe_failed) {
		dev_err(dev, "slot #%d: probe failed for '%s'\n",
			slot->slotno, slot->part_number);
		return -ENODEV;
	}

	if (slot->loaded) {
		dev_err(dev, "slot #%d: already loaded for '%s'\n",
			slot->slotno, slot->part_number);
		return -EAGAIN;
	}

	/* make sure we don't leak this on repeated calls */
	kfree(slot->dtbo);
	slot->dtbo = NULL;

	dev_dbg(dev, "slot #%d: Requesting part number/version based '%s-%s.dtbo\n",
			slot->slotno, slot->part_number, slot->version);

	/* request the part number + .dtbo*/
	slot->dtbo = kasprintf(GFP_KERNEL, "%s-%s.dtbo",
			slot->part_number, slot->version);
	if (slot->dtbo == NULL) {
		dev_err(dev, "slot #%d: Failed to get dtbo '%s'\n",
				slot->slotno, dtbo);
		return -ENOMEM;
	}

	dev_dbg(dev, "slot #%d: Requesting firmware '%s' for board-name '%s', version '%s'%s\n",
			slot->slotno,
			slot->dtbo, slot->board_name, slot->version,
			system_state == SYSTEM_BOOTING ? " - booting" : "");

	err = request_firmware_direct(&slot->fw, slot->dtbo, dev);
	if (err != 0) {
		dev_dbg(dev, "failed to load firmware '%s'\n", slot->dtbo);
		goto err_fail_no_fw;
	}

	dev_dbg(dev, "slot #%d: dtbo '%s' loaded; converting to live tree\n",
			slot->slotno, slot->dtbo);

	of_fdt_unflatten_tree((unsigned long *)slot->fw->data, NULL,
			&slot->overlay);
	if (slot->overlay == NULL) {
		dev_err(dev, "slot #%d: Failed to unflatten\n",
				slot->slotno);
		err = -EINVAL;
		goto err_fail;
	}

	/* mark it as detached */
	of_node_set_flag(slot->overlay, OF_DETACHED);

	/* perform resolution */
	err = of_resolve_phandles(slot->overlay);
	if (err != 0) {
		dev_err(dev, "slot #%d: Failed to resolve tree\n",
				slot->slotno);
		goto err_fail;
	}

	err = capemgr_verify_overlay(slot);
	if (err != 0) {
		dev_err(dev, "slot #%d: Failed verification\n",
				slot->slotno);
		goto err_fail;
	}

	err = of_overlay_create(slot->overlay);
	if (err < 0) {
		dev_err(dev, "slot #%d: Failed to create overlay\n",
				slot->slotno);
		goto err_fail;
	}
	slot->overlay_id = err;

	slot->loading = 0;
	slot->loaded = 1;

	dev_info(dev, "slot #%d: dtbo '%s' loaded; overlay id #%d\n",
			slot->slotno, slot->dtbo, slot->overlay_id);

	return 0;

err_fail:

	/* TODO: free the overlay, we can't right now cause
	 * the unflatten method does not track it */
	slot->overlay = NULL;

	release_firmware(slot->fw);
	slot->fw = NULL;

err_fail_no_fw:
	slot->loading = 0;
	return err;
}

static int capemgr_unload_slot(struct bone_cape_slot *slot)
{
	if (!slot->loaded || slot->overlay_id == -1)
		return -EINVAL;

	of_overlay_destroy(slot->overlay_id);
	slot->overlay_id = -1;

	slot->loaded = 0;

	return 0;

}

/* slots_list_mutex must be taken */
static int capemgr_remove_slot_no_lock(struct bone_cape_slot *slot)
{
	struct capemgr_info *info = slot->info;
	struct device *dev = &info->pdev->dev;
	int ret;

	if (slot == NULL)
		return 0;

	if (slot->loaded && slot->overlay_id >= 0) {
		/* unload just in case */
		ret = capemgr_unload_slot(slot);
		if (ret != 0) {
			dev_err(dev, "Unable to unload slot #%d\n",
				slot->slotno);
			return ret;
		}
	}

	/* if probed OK, remove the sysfs nodes */
	if (slot->probed && !slot->probe_failed)
		bone_cape_slot_sysfs_unregister(slot);

	/* remove it from the list */
	list_del(&slot->node);

	if (slot->nvmem_cell)
		nvmem_cell_put(slot->nvmem_cell);
	devm_kfree(dev, slot);
	return 0;
}

static int capemgr_remove_slot(struct bone_cape_slot *slot)
{
	struct capemgr_info *info = slot->info;
	int ret;

	mutex_lock(&info->slots_list_mutex);
	ret = capemgr_remove_slot_no_lock(slot);
	mutex_unlock(&info->slots_list_mutex);

	return ret;
}

static int bone_slot_fill_override(struct bone_cape_slot *slot,
		const char *part_number, const char *version)
{
	const struct ee_field *sig_field;
	int i, len, has_part_number;
	char *p;

	slot->probe_failed = 0;
	slot->probed = 0;

	/* zero out signature */
	memset(slot->signature, 0,
			sizeof(slot->signature));

	/* first, fill in all with override defaults */
	for (i = 0; i < ARRAY_SIZE(cape_sig_fields); i++) {

		sig_field = &cape_sig_fields[i];

		/* point to the entry */
		p = slot->signature + sig_field->start;

		if (sig_field->override)
			memcpy(p, sig_field->override,
					sig_field->size);
		else
			memset(p, 0, sig_field->size);
	}

	/* if a part_number is supplied use it */
	len = part_number ? strlen(part_number) : 0;
	if (len > 0) {
		sig_field = &cape_sig_fields[CAPE_EE_FIELD_PART_NUMBER];

		/* point to the entry */
		p = slot->signature + sig_field->start;

		/* copy and zero out any remainder */
		if (len > sig_field->size)
			len = sig_field->size;
		memcpy(p, part_number, len);
		if (len < sig_field->size)
			memset(p + len, 0, sig_field->size - len);

		has_part_number = 1;
	}

	/* if a version is supplied use it */
	len = version ? strlen(version) : 0;
	if (len > 0) {
		sig_field = &cape_sig_fields[CAPE_EE_FIELD_VERSION];

		/* point to the entry */
		p = slot->signature + sig_field->start;

		/* copy and zero out any remainder */
		if (len > sig_field->size)
			len = sig_field->size;
		memcpy(p, version, len);
		if (len < sig_field->size)
			memset(p + len, 0, sig_field->size - len);
	}

	/* we must have a part number */
	if (!has_part_number)
		return -EINVAL;

	slot->override = 1;

	return 0;
}

static struct bone_cape_slot *
capemgr_add_slot(struct capemgr_info *info, const char *slot_name,
		const char *part_number, const char *version, int prio)
{
	struct bone_cape_slot *slot;
	struct device *dev = &info->pdev->dev;
	int slotno;
	int ret;

	slotno = atomic_inc_return(&info->next_slot_nr) - 1;

	slot = devm_kzalloc(dev, sizeof(*slot), GFP_KERNEL);
	if (slot == NULL)
		return ERR_PTR(-ENOMEM);

	slot->info = info;
	slot->slotno = slotno;
	slot->priority = prio;
	slot->overlay_id = -1;

	if (slot_name) {
		slot->nvmem_cell = nvmem_cell_get(dev, slot_name);
		if (IS_ERR(slot->nvmem_cell)) {
			ret = PTR_ERR(slot->nvmem_cell);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get slot eeprom cell\n");
			slot->nvmem_cell = NULL;
			goto err_out;
		}
	} else {
		dev_err(dev, "slot #%d: override\n", slotno);

		/* fill in everything with defaults first */
		ret = bone_slot_fill_override(slot, part_number, version);
		if (ret != 0) {
			dev_err(dev, "slot #%d: override failed\n", slotno);
			goto err_out;
		}
	}

	ret = bone_slot_scan(slot);
	if (ret != 0) {

		if (!slot->probe_failed) {
			dev_err(dev, "slot #%d: scan failed\n",
					slotno);
			goto err_out;
		}

		dev_err(dev, "slot #%d: No cape found\n", slotno);
		/* but all is fine */
	} else {
		if (uboot_capemgr_enabled == 0) {
			dev_err(dev, "slot #%d: '%s'\n",
					slotno, slot->text_id);

			ret = bone_cape_slot_sysfs_register(slot);
			if (ret != 0) {
				dev_err(dev, "slot #%d: sysfs register failed\n",
						slotno);
				goto err_out;
			}
		} else {
			dev_err(dev, "slot #%d: auto loading handled by U-Boot\n", slotno);
		}
	}

	/* add to the slot list */
	mutex_lock(&info->slots_list_mutex);
	list_add_tail(&slot->node, &info->slot_list);
	mutex_unlock(&info->slots_list_mutex);

	return slot;

err_out:
	if (slot->nvmem_cell)
		nvmem_cell_put(slot->nvmem_cell);
	devm_kfree(dev, slot);
	return ERR_PTR(ret);
}

/* return 1 if it makes sense to retry loading */
static int retry_loading_condition(struct bone_cape_slot *slot)
{
	struct capemgr_info *info = slot->info;
	struct device *dev = &info->pdev->dev;
	struct bone_cape_slot *slotn;
	int ret;

	dev_dbg(dev, "loader: retry_loading slot-%d %s:%s (prio %d)\n",
			slot->slotno, slot->part_number, slot->version,
			slot->priority);

	mutex_lock(&info->slots_list_mutex);
	ret = 0;
	list_for_each_entry(slotn, &info->slot_list, node) {
		/* if same slot or not loading skip */
		if (!slotn->loading || slotn->retry_loading)
			continue;
		/* at least one cape is still loading (without retrying) */
		ret = 1;
	}
	mutex_unlock(&info->slots_list_mutex);
	return ret;
}

/* return 1 if this slot is clear to try to load now */
static int clear_to_load_condition(struct bone_cape_slot *slot)
{
	struct capemgr_info *info = slot->info;
	int my_prio = slot->priority;
	struct device *dev = &info->pdev->dev;
	int ret;

	dev_dbg(dev, "loader: check slot-%d %s:%s (prio %d)\n", slot->slotno,
			slot->part_number, slot->version, slot->priority);

	mutex_lock(&info->slots_list_mutex);
	ret = 1;
	list_for_each_entry(slot, &info->slot_list, node) {
		/* if any slot is loading with lowest priority */
		if (!slot->loading)
			continue;
		if (slot->priority < my_prio) {
			ret = 0;
			break;
		}
	}
	mutex_unlock(&info->slots_list_mutex);
	return ret;
}

static int capemgr_loader(void *data)
{
	struct bone_cape_slot *slot = data;
	struct capemgr_info *info = slot->info;
	struct device *dev = &info->pdev->dev;
	int ret, done, other_loading, booting;

	done = 0;

	slot->retry_loading = 0;

	dev_dbg(dev, "loader: before slot-%d %s:%s (prio %d)\n", slot->slotno,
			slot->part_number, slot->version, slot->priority);

	/*
	 * We have a basic priority based arbitration system
	 * Slots have priorities, so the lower priority ones
	 * should start loading first. So each time we end up
	 * here.
	 */
	ret = wait_event_interruptible(info->load_wq,
			clear_to_load_condition(slot));
	if (ret < 0) {
		dev_warn(dev, "loader, Signal pending\n");
		return ret;
	}

	dev_dbg(dev, "loader: after slot-%d %s:%s (prio %d)\n", slot->slotno,
			slot->part_number, slot->version, slot->priority);

	/* using the return value */
	ret = capemgr_load_slot(slot);

	/* wake up all just in case */
	wake_up_interruptible_all(&info->load_wq);

	if (ret == 0)
		goto done;

	dev_dbg(dev, "loader: retrying slot-%d %s:%s (prio %d)\n", slot->slotno,
			slot->part_number, slot->version, slot->priority);

	/* first attempt has failed; now try each time there's any change */
	slot->retry_loading = 1;

	for (;;) {
		booting = (system_state == SYSTEM_BOOTING);
		other_loading = retry_loading_condition(slot);
		if (!booting && !other_loading)
			break;

		/* simple wait for someone to kick us */
		if (other_loading) {
			DEFINE_WAIT(__wait);

			prepare_to_wait(&info->load_wq, &__wait,
					TASK_INTERRUPTIBLE);
			finish_wait(&info->load_wq, &__wait);
		} else {
			/* always delay when booting */
			msleep(boot_scan_period);
		}

		if (signal_pending(current)) {
			dev_warn(dev, "loader, Signal pending\n");
			ret = -ERESTARTSYS;
			goto done;
		}

		/* using the return value */
		ret = capemgr_load_slot(slot);
		if (ret == 0)
			goto done;

		/* wake up all just in case */
		wake_up_interruptible_all(&info->load_wq);
	}

done:
	slot->loading = 0;
	slot->retry_loading = 0;

	if (ret == 0) {
		dev_dbg(dev, "loader: done slot-%d %s:%s (prio %d)\n",
			slot->slotno, slot->part_number, slot->version,
			slot->priority);
	} else {
		dev_err(dev, "loader: failed to load slot-%d %s:%s (prio %d)\n",
			slot->slotno, slot->part_number, slot->version,
			slot->priority);

		/* if it's a override slot remove it */
		if (slot->override)
			capemgr_remove_slot(slot);
	}

	return ret;
}

static int
capemgr_probe(struct platform_device *pdev)
{
	struct capemgr_info *info;
	struct bone_baseboard *bbrd;
	struct bone_cape_slot *slot;
	struct device_node *pnode = pdev->dev.of_node;
	struct device_node *baseboardmaps_node;
	struct device_node *node;
	const char *part_number;
	const char *version;
	const char *board_name;
	const char *compatible_name;
	char slot_name[16];
	u32 slots_nr;
	int i, ret, len, prio;
	long val;
	char *wbuf, *s, *p, *e;

	if (uboot_capemgr_enabled)
		return 0;

	/* we don't use platform_data at all; we require OF */
	if (pnode == NULL)
		return -ENOTSUPP;

	info = devm_kzalloc(&pdev->dev,
			sizeof(struct capemgr_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->pdev = pdev;
	platform_set_drvdata(pdev, info);

	atomic_set(&info->next_slot_nr, 0);
	INIT_LIST_HEAD(&info->slot_list);
	mutex_init(&info->slots_list_mutex);

	init_waitqueue_head(&info->load_wq);

	baseboardmaps_node = NULL;

	/* find the baseboard */
	bbrd = &info->baseboard;

	baseboardmaps_node = of_get_child_by_name(pnode, "baseboardmaps");
	if (baseboardmaps_node == NULL) {
		dev_err(&pdev->dev, "Failed to get baseboardmaps node");
		ret = -ENODEV;
		goto err_exit;
	}

	bbrd->nvmem_cell = nvmem_cell_get(&pdev->dev, "baseboard");
	if (IS_ERR(bbrd->nvmem_cell)) {
		ret = PTR_ERR(bbrd->nvmem_cell);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to get baseboard eeprom cell\n");
		bbrd->nvmem_cell = NULL;
		goto err_exit;
	}

	ret = bone_baseboard_scan(bbrd);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to scan baseboard eeprom\n");
		goto err_exit;
	}

	dev_info(&pdev->dev, "Baseboard: '%s'\n", bbrd->text_id);

	board_name = NULL;
	compatible_name = NULL;
	for_each_child_of_node(baseboardmaps_node, node) {
		/* there must be board-name */
		if (of_property_read_string(node, "board-name",
					&board_name) != 0 ||
		    of_property_read_string(node, "compatible-name",
					&compatible_name) != 0)
			continue;

		if (strcmp(bbrd->board_name, board_name) == 0)
			break;
	}
	of_node_put(baseboardmaps_node);
	baseboardmaps_node = NULL;

	if (node == NULL) {
		dev_err(&pdev->dev, "Failed to find compatible map for %s\n",
				bbrd->board_name);
		ret = -ENODEV;
		goto err_exit;
	}
	bbrd->compatible_name = kstrdup(compatible_name, GFP_KERNEL);
	if (bbrd->compatible_name == NULL) {
		ret = -ENOMEM;
		goto err_exit;
	}
	of_node_put(node);

	/* get slot number */
	ret = of_property_read_u32(pnode, "#slots", &slots_nr);
	if (ret != 0)
		slots_nr = 0;

	dev_info(&pdev->dev, "compatible-baseboard=%s - #slots=%d\n",
			bbrd->compatible_name, slots_nr);

	for (i = 0; i < slots_nr; i++) {
		snprintf(slot_name, sizeof(slot_name), "slot%d", i);
		slot = capemgr_add_slot(info, slot_name, NULL, NULL, 0);
		if (IS_ERR(slot)) {
			dev_err(&pdev->dev, "Failed to add slot #%d\n",
				atomic_read(&info->next_slot_nr));
			ret = PTR_ERR(slot);
			goto err_exit;
		}
	}

	/* iterate over enable_partno (if there) */
	if (enable_partno && strlen(enable_partno) > 0) {

		/* allocate a temporary buffer */
		wbuf = devm_kzalloc(&pdev->dev, PAGE_SIZE, GFP_KERNEL);
		if (wbuf == NULL) {
			ret = -ENOMEM;
			goto err_exit;
		}

		/* add any enable_partno capes */
		s = enable_partno;
		while (*s) {
			/* form is PART[:REV[:PRIO]],PART.. */
			p = strchr(s, ',');
			if (p == NULL)
				e = s + strlen(s);
			else
				e = p;

			/* copy to temp buffer */
			len = e - s;
			if (len >= PAGE_SIZE - 1)
				len = PAGE_SIZE - 1;
			memcpy(wbuf, s, len);
			wbuf[len] = '\0';

			/* move to the next */
			s = *e ? e + 1 : e;

			part_number = wbuf;

			/* default version is NULL & prio is 0 */
			version = NULL;
			prio = 0;

			/* now split the rev & prio part */
			p = strchr(wbuf, ':');
			if (p != NULL) {
				*p++ = '\0';
				if (*p != ':')
					version = p;
				p = strchr(p, ':');
				if (p != NULL) {
					*p++ = '\0';
					ret = kstrtol(p, 10, &val);
					if (ret == 0)
						prio = val;
				}
			}

			dev_info(&pdev->dev,
				"enabled_partno PARTNO '%s' VER '%s' PR '%d'\n",
					part_number,
					version ? version : "N/A", prio);

			/* only immediate slots are allowed here */
			slot = capemgr_add_slot(info, NULL,
					part_number, version, prio);

			/* we continue even in case of an error */
			if (IS_ERR_OR_NULL(slot)) {
				dev_warn(&pdev->dev, "Failed to add slot #%d\n",
					atomic_read(&info->next_slot_nr) - 1);
			}
		}

		devm_kfree(&pdev->dev, wbuf);
	}

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to pm_runtime_get_sync()\n");
		goto err_exit;
	}

	pm_runtime_put(&pdev->dev);

	/* it is safe to create the attribute groups */
	ret = sysfs_create_groups(&pdev->dev.kobj, attr_groups);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to create sysfs attributes\n");
		goto err_exit;
	}
	/* automatically cleared by driver core now */
	pdev->dev.groups = attr_groups;

	/* now load each (take lock to be sure */
	mutex_lock(&info->slots_list_mutex);

	list_for_each_entry(slot, &info->slot_list, node) {

		/* if matches the disabled ones skip */
		if (bone_match_cape(disable_partno, slot->part_number,
					slot->version)) {
			dev_info(&pdev->dev,
				"Skipping loading of disabled cape with part# %s\n",
				slot->part_number);
			slot->disabled = 1;
			continue;
		}

		if (!slot->probe_failed && !slot->loaded)
			slot->loading = 1;
	}

	/* now start the loader thread(s) (all at once) */
	list_for_each_entry(slot, &info->slot_list, node) {

		if (!slot->loading)
			continue;

		slot->loader_thread = kthread_run(capemgr_loader,
				slot, "capemgr-loader-%d",
				slot->slotno);
		if (IS_ERR(slot->loader_thread)) {
			dev_warn(&pdev->dev, "slot #%d: Failed to start loader\n",
					slot->slotno);
			slot->loader_thread = NULL;
		}
	}
	mutex_unlock(&info->slots_list_mutex);

	dev_info(&pdev->dev, "initialized OK.\n");

	return 0;

err_exit:
	if (bbrd->nvmem_cell)
		nvmem_cell_put(bbrd->nvmem_cell);
	of_node_put(baseboardmaps_node);
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, info);

	return ret;
}

static int capemgr_remove(struct platform_device *pdev)
{
	struct capemgr_info *info = platform_get_drvdata(pdev);
	struct bone_baseboard *bbrd = &info->baseboard;
	struct bone_cape_slot *slot, *slotn;
	int ret;

	mutex_lock(&info->slots_list_mutex);
	list_for_each_entry_safe(slot, slotn, &info->slot_list, node)
		capemgr_remove_slot_no_lock(slot);
	mutex_unlock(&info->slots_list_mutex);

	platform_set_drvdata(pdev, NULL);

	ret = pm_runtime_get_sync(&pdev->dev);
	if (IS_ERR_VALUE(ret))
		return ret;

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	if (bbrd->nvmem_cell)
		nvmem_cell_put(bbrd->nvmem_cell);
	devm_kfree(&pdev->dev, info);

	return 0;
}

static struct platform_driver capemgr_driver = {
	.probe		= capemgr_probe,
	.remove		= capemgr_remove,
	.driver		= {
		.name	= "bone_capemgr",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(capemgr_of_match),
	},
};

module_platform_driver(capemgr_driver);

MODULE_AUTHOR("Pantelis Antoniou");
MODULE_DESCRIPTION("Beaglebone cape manager");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bone_capemgr");
