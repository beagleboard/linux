/*
 * TI Beaglebone cape controller
 *
 * Copyright (C) 2012 Pantelis Antoniou <panto@antoniou-consulting.com>
 * Copyright (C) 2012 Texas Instruments Inc.
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_i2c.h>
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
#include <linux/i2c.h>
#include <linux/i2c/eeprom.h>
#include <linux/kthread.h>
#include <linux/wait.h>

/* extra command line overrides */
static char *extra_override = NULL;
module_param(extra_override, charp, 0444);
MODULE_PARM_DESC(extra_override,
		"Comma delimited list of PART-NUMBER[:REV] overrides");

/* disabled capes */
static char *disable_partno = NULL;
module_param(disable_partno, charp, 0444);
MODULE_PARM_DESC(disable_partno,
		"Comma delimited list of PART-NUMBER[:REV] of disabled capes");

struct bone_capemgr_info;

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
	struct bone_capemgr_info *info;
	int			slotno;
	u32			eeprom_handle;
	int			eeprom_addr;
	struct i2c_client	*client;
	struct memory_accessor	*macc;
	unsigned int		probed : 1;
	unsigned int		probe_failed : 1;
	unsigned int		override : 1;
	char			text_id[256];
	char			signature[256];
	/* quick access */
	char			board_name[32+1];
	char 			version[4+1];
	char 			manufacturer[16+1];
	char 			part_number[16+1];

	/* attribute group */
	char			*ee_attr_name;
	int			ee_attrs_count;
	struct slot_ee_attribute *ee_attrs;
	struct attribute	**ee_attrs_tab;
	struct attribute_group	attrgroup;

	unsigned int		loading : 1;
	unsigned int		loaded : 1;
	char			*dtbo;
	const struct firmware	*fw;
	struct device_node	*overlay;
	int			ovinfo_cnt;
	struct of_overlay_info	*ovinfo;

	/* loader thread */
	struct task_struct	*loader_thread;

	/* load priority */
	int priority;
};

struct bone_capemap {
	struct list_head node;
	char *part_number;
	struct device_node *map_node;
};

struct bone_baseboard {

	/* from the matched boardmap node */
	char			*compatible_name;

	/* filled in by reading the eeprom */
	char			signature[256];
	char			text_id[64+1];

	/* quick access */
	char			board_name[8+1];
	char 			revision[4+1];
	char 			serial_number[12+1];

	/* access to the eeprom */
	u32			eeprom_handle;
	int			eeprom_addr;
	struct i2c_client	*client;
	struct memory_accessor	*macc;
	unsigned int		probed : 1;
	unsigned int		probe_failed : 1;
	unsigned int		override : 1;
};

struct bone_capemgr_info {
	struct platform_device	*pdev;

	atomic_t next_slot_nr;
	struct list_head	slot_list;
	struct mutex		slots_list_mutex;

	int capemaps_nr;
	struct list_head	capemap_list;
	struct mutex		capemap_mutex;

	/* baseboard EEPROM data */
	struct bone_baseboard	baseboard;

	/* wait queue for keeping the priorities straight */
	wait_queue_head_t	load_wq;
};

static int bone_slot_fill_override(struct bone_cape_slot *slot,
		struct device_node *node,
		const char *part_number, const char *version);
static struct bone_cape_slot *bone_capemgr_add_slot(
		struct bone_capemgr_info *info, struct device_node *node,
		const char *part_number, const char *version);
static int bone_capemgr_remove_slot_no_lock(struct bone_cape_slot *slot);
static int bone_capemgr_remove_slot(struct bone_cape_slot *slot);
static int bone_capemgr_load(struct bone_cape_slot *slot);
static int bone_capemgr_unload(struct bone_cape_slot *slot);

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
		( (u32)_p[2] <<  8) |  (u32)_p[3]      ); \
	})

#define EE_FIELD_HEADER_VALID	0xaa5533ee

struct ee_field {
	const char 	*name;
	int 		start;
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
		buf[sig_field->size] = '\0';;

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
static const struct of_device_id bone_capemgr_of_match[] = {
	{
		.compatible = "ti,bone-capemgr",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, bone_capemgr_of_match);

#endif

static int bone_baseboard_scan(struct bone_baseboard *bbrd)
{
	struct bone_capemgr_info *info = container_of(bbrd,
			struct bone_capemgr_info, baseboard);
	struct memory_accessor *macc = bbrd->macc;
	const u8 *p;
	int i, r;

	/* need to read EEPROM? */
	if (bbrd->probed)
		goto bbrd_fail_check;

	bbrd->probed = 1;

	if (!bbrd->override) {

		if (macc == NULL || macc->read == NULL) {
			dev_err(&info->pdev->dev,
				"bone: No memory accessor for baseboard\n");
			return -ENODEV;
		}

		for (i = 0; i < 10; i++) {

			/* perform read */
			r = macc->read(macc, bbrd->signature,
					0, sizeof(bbrd->signature));

			if (r == sizeof(bbrd->signature))
				break;

			dev_info(&info->pdev->dev,
				"bone: scan failed (%d time)\n", i + 1);

			msleep(500);
		}

		if (i >= 10) {
			bbrd->probe_failed = 1;
			return r >= 0 ? -EINVAL : r;
		}

	} else
		dev_info(&info->pdev->dev,
			"bone: Using override eeprom data for baseboard\n");

	p = bbrd->signature;
	if (EE_FIELD_MAKE_HEADER(p) != EE_FIELD_HEADER_VALID) {
		dev_err(&info->pdev->dev, "bone: Invalid signature "
			"'%08x' at baseboard\n",
			EE_FIELD_MAKE_HEADER(p));
		bbrd->probe_failed = 1;
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

bbrd_fail_check:
	/* bbrd has failed and we don't support hotpluging */
	if (bbrd->probe_failed)
		return -ENODEV;

	return 0;
}

static int bone_slot_scan(struct bone_cape_slot *slot)
{
	struct bone_capemgr_info *info = slot->info;
	struct memory_accessor *macc = slot->macc;
	const u8 *p;
	int r;

	/* need to read EEPROM? */
	if (slot->probed)
		goto slot_fail_check;

	slot->probed = 1;

	if (!slot->override) {

		if (macc == NULL || macc->read == NULL) {
			dev_err(&info->pdev->dev,
				"bone: No memory accessor for slot %d\n",
				slot->slotno);
			return -ENODEV;
		}

		/* perform read */
		r = macc->read(macc, slot->signature,
				0, sizeof(slot->signature));

		if (r != sizeof(slot->signature)) {
			slot->probe_failed = 1;
			return r >= 0 ? -EINVAL : r;
		}
	} else
		dev_info(&info->pdev->dev,
			"bone: Using override eeprom data at slot %d\n",
			slot->slotno);

	p = slot->signature;
	if (EE_FIELD_MAKE_HEADER(p) != EE_FIELD_HEADER_VALID) {
		dev_err(&info->pdev->dev, "bone: Invalid signature "
			"'%08x' at slot %d\n",
			EE_FIELD_MAKE_HEADER(p),
			slot->slotno);
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

/* check an override slot node if it's compatible */
static int bone_is_compatible_override(struct device_node *node,
		const char *compatible_name)
{
	struct property *prop;
	const char *part_number;
	const char *version;

	/* check if the slot is compatible with the board */
	prop = of_find_property(node, "compatible", NULL);

	/* no prop, it's something that's compatible with everything */
	if (prop == NULL)
		return 1;

	/* check if it's directly compatible with the baseboard */
	if (of_multi_prop_cmp(prop, compatible_name) == 0)
		return 1;

	/* forced override? */
	if (of_multi_prop_cmp(prop, "force") == 0)
		return 1;

	/* final try, check if it's specified in the kernel command line */
	if (extra_override == NULL)
		return 0;

	/* the compatible name should have kernel-command-line in it */
	if (of_multi_prop_cmp(prop, "kernel-command-line") != 0)
		return 0;

	/* we must have at least the part-name */
	if (of_property_read_string(node, "part-number",
				&part_number) != 0)
		return 0;

	/* read the version (if it exists) */
	if (of_property_read_string(node, "version", &version) != 0)
		version = NULL;

	/* match on the extra override */
	return bone_match_cape(extra_override, part_number, version);
}

static int bone_is_compatible_runtime_override(struct device_node *node,
		const char *req_part_number, const char *req_version)
{
	struct property *prop;
	const char *part_number;
	const char *version;

	/* only check overrides */
	if (!of_property_read_bool(node, "ti,cape-override"))
		return 0;

	/* check if the slot is compatible with the board */
	prop = of_find_property(node, "compatible", NULL);

	/* no prop, it's something that's compatible with everything */
	if (prop == NULL)
		return 1;

	/* the compatible name should have runtime in it */
	if (of_multi_prop_cmp(prop, "runtime") != 0)
		return 0;

	/* we must have at least the part-name */
	if (of_property_read_string(node, "part-number",
				&part_number) != 0)
		return 0;

	/* read the version (if it exists) */
	if (of_property_read_string(node, "version", &version) != 0)
		version = NULL;

	/* the part names must match */
	if (strcmp(req_part_number, part_number) != 0)
		return 0;

	/* if any version is null, any version matches */
	if (version == NULL || req_version == NULL)
		return 1;

	/* finally versions must match */
	return strcmp(req_version, version) == 0;
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
		.devattr = __ATTR(_name, 0440, slot_ee_attr_show, NULL), \
		.field = CAPE_EE_FIELD_##_field , \
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
	struct bone_capemgr_info *info = slot->info;
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
	struct bone_capemgr_info *info = slot->info;
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
	struct bone_capemgr_info *info = platform_get_drvdata(pdev);
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
		.field = BBRD_EE_FIELD_##_field , \
	}

static struct bbrd_ee_attribute bbrd_ee_attrs[] = {
	BBRD_EE_ATTR(header, HEADER),
	BBRD_EE_ATTR(board-name, BOARD_NAME),
	BBRD_EE_ATTR(revision, REVISION),
	BBRD_EE_ATTR(serial-number, SERIAL_NUMBER),
	BBRD_EE_ATTR(config-option, CONFIG_OPTION),
};

static struct attribute *bbrd_attrs_flat[] = {
	&bbrd_ee_attrs[BBRD_EE_FIELD_HEADER	 	].devattr.attr,
	&bbrd_ee_attrs[BBRD_EE_FIELD_BOARD_NAME	 	].devattr.attr,
	&bbrd_ee_attrs[BBRD_EE_FIELD_REVISION		].devattr.attr,
	&bbrd_ee_attrs[BBRD_EE_FIELD_SERIAL_NUMBER	].devattr.attr,
	&bbrd_ee_attrs[BBRD_EE_FIELD_CONFIG_OPTION	].devattr.attr,
	NULL,
};

static const struct attribute_group bbrd_attr_group = {
	.name	= "baseboard",
	.attrs	= bbrd_attrs_flat,
};

static ssize_t slots_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_capemgr_info *info = platform_get_drvdata(pdev);
	struct bone_cape_slot *slot;
	ssize_t len, sz;

	mutex_lock(&info->slots_list_mutex);
	sz = 0;
	list_for_each_entry(slot, &info->slot_list, node) {

		len = sprintf(buf, "%2d: %02x:%c%c%c%c%c %s\n",
				slot->slotno,
				(int)slot->client ?
					slot->client->addr & 0x7f : 0xff,
				slot->probed       ? 'P' : '-',
				slot->probe_failed ? 'F' : '-',
				slot->override     ? 'O' : '-',
				slot->loading	   ? 'l' : '-',
				slot->loaded	   ? 'L' : '-',
				slot->text_id);

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
	struct bone_capemgr_info *info = platform_get_drvdata(pdev);
	struct bone_cape_slot *slot;
	struct device_node *pnode, *node, *slots_node;
	char *s, *part_number, *version;
	int ret;
	int slotno;

	/* check for remove slot */
	if (strlen(buf) > 0 && buf[0] == '-') {
		slotno = simple_strtoul(buf + 1, NULL, 10);

		/* now load each (take lock to be sure */
		mutex_lock(&info->slots_list_mutex);
		list_for_each_entry(slot, &info->slot_list, node) {
			if (slotno == slot->slotno)
				goto found;
		}

		mutex_unlock(&info->slots_list_mutex);
		return -ENODEV;
found:
		ret = bone_capemgr_remove_slot_no_lock(slot);
		mutex_unlock(&info->slots_list_mutex);

		if (ret == 0)
			dev_info(&pdev->dev, "Removed slot #%d\n", slotno);
		else
			dev_err(&pdev->dev, "Failed to remove slot #%d\n",
					slotno);

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

	/* iterate over any slots */
	slots_node = of_get_child_by_name(pnode, "slots");
	if (slots_node != NULL) {
		for_each_child_of_node(slots_node, node) {

			/* check if the override is compatible */
			if (!bone_is_compatible_runtime_override(node,
						part_number, version))
				continue;

			/* if matches the disabled ones skip */
			if (bone_match_cape(disable_partno,
						part_number, NULL)) {
				dev_info(&pdev->dev,
					"Skipping disabled cape with "
						"part# %s\n", part_number);
				continue;
			}

			slot = bone_capemgr_add_slot(info, node,
					part_number, version);
			if (IS_ERR(slot)) {
				dev_err(&pdev->dev, "Failed to add slot #%d\n",
					atomic_read(&info->next_slot_nr) - 1);
				ret = PTR_ERR(slot);
				slot = NULL;
				goto err_fail;
			}
			break;
		}
		of_node_put(node);
		of_node_put(slots_node);
	}
	slots_node = NULL;

	/* no specific slot found, try immediate */
	if (!slot)
		slot = bone_capemgr_add_slot(info, NULL,
				part_number, version);

	if (IS_ERR_OR_NULL(slot)) {
		dev_err(&pdev->dev, "Failed to add slot #%d\n",
			atomic_read(&info->next_slot_nr) - 1);
		ret = slot ? PTR_ERR(slot) : -ENODEV;
		slot = NULL;
		goto err_fail;
	}

	kfree(part_number);

	ret = bone_capemgr_load(slot);

	if (ret != 0)
		bone_capemgr_remove_slot(slot);

	return ret == 0 ? strlen(buf) : ret;
err_fail:
	of_node_put(node);
	of_node_put(slots_node);
	kfree(part_number);
	return ret;
}

static DEVICE_ATTR(slots, 0644, slots_show, slots_store);

static int bone_capemgr_info_sysfs_register(struct bone_capemgr_info *info)
{
	struct device *dev = &info->pdev->dev;
	int ret;

	ret = device_create_file(dev, &dev_attr_slots);
	if (ret != 0)
		goto err_fail_no_slots;

	ret = sysfs_create_group(&dev->kobj, &bbrd_attr_group);
	if (ret != 0)
		goto err_fail_no_bbrd_grp;

	return 0;
err_fail_no_bbrd_grp:
	device_remove_file(dev, &dev_attr_slots);
err_fail_no_slots:
	return ret;
}

static void bone_capemgr_info_sysfs_unregister(struct bone_capemgr_info *info)
{
	struct device *dev = &info->pdev->dev;

	sysfs_remove_group(&dev->kobj, &bbrd_attr_group);
	device_remove_file(dev, &dev_attr_slots);
}

/* verify the overlay */
static int bone_capemgr_verify_overlay(struct bone_cape_slot *slot)
{
	struct bone_capemgr_info *info = slot->info;
	struct device *dev = &info->pdev->dev;
	struct bone_baseboard *bbrd = &info->baseboard;
	struct device_node *node = slot->overlay;
	struct property *prop;
	struct bone_cape_slot *slotn;
	int err, counta, countb, i, j;
	const char *ra, *rb;

	/* validate */
	if (node == NULL) {
		dev_err(dev, "slot #%d: No overlay "
				"for '%s'\n",
				slot->slotno, slot->part_number);
		return -EINVAL;
	}

	/* check if the slot is compatible with the board */
	prop = of_find_property(node, "compatible", NULL);

	/* no compatible property? */
	if (prop == NULL) {
		dev_err(dev, "slot #%d: No compatible property "
				"for '%s'\n",
				slot->slotno, slot->part_number);
		return -EINVAL;
	}

	/* verify that the cape is baseboard compatible */
	if (of_multi_prop_cmp(prop, bbrd->compatible_name) != 0) {
		dev_err(dev, "slot #%d: Incompatible with baseboard "
				"for '%s'\n",
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
					dev_err(dev, "slot #%d: %s conflict "
						"%s (#%d:%s)\n", slot->slotno,
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

static int bone_capemgr_load(struct bone_cape_slot *slot)
{
	struct bone_capemgr_info *info = slot->info;
	struct device *dev = &info->pdev->dev;
	struct device_node *node;
	struct property *prop;
	const char *dtbo;
	int found, err;
	struct bone_capemap *capemap;

	if (slot->probe_failed)
		return -ENODEV;

	if (slot->loaded)
		return -EAGAIN;

	mutex_lock(&info->capemap_mutex);
	found = 0;
	list_for_each_entry(capemap, &info->capemap_list, node) {
		if (strcmp(capemap->part_number, slot->part_number) == 0) {
			found = 1;
			break;
		}
	}

	/* found? */
	if (found) {
		if (capemap->map_node == NULL) {
			mutex_unlock(&info->capemap_mutex);
			/* need to match programatically; not supported yet */
			dev_err(dev, "slot #%d: Failed to find capemap "
					"for '%s'\n",
					slot->slotno, slot->part_number);
			return -ENODEV;
		}

		/* locate first match */
		dtbo = NULL;
		for_each_child_of_node(capemap->map_node, node) {

			/* dtbo must exist */
			if (of_property_read_string(node, "dtbo", &dtbo) != 0)
				continue;

			/* get version property (if any) */
			prop = of_find_property(node, "version", NULL);

			/* if no version node exists, we match */
			if (prop == NULL)
				break;

			if (of_multi_prop_cmp(prop, slot->version) == 0)
				break;
		}

		if (node == NULL) {
			/* can't find dtbo version node? try the default */
			if (of_property_read_string(capemap->map_node,
						"dtbo", &dtbo) != 0) {
				mutex_unlock(&info->capemap_mutex);
				dev_err(dev, "slot #%d: Failed to find dtbo "
						"for '%s'\n",
						slot->slotno,
						slot->part_number);
				return -ENODEV;
			}
		}

		slot->dtbo = kstrdup(dtbo, GFP_KERNEL);
		of_node_put(node);	/* handles NULL */
	} else {
		dev_info(dev, "slot #%d: Requesting part number/version based "
				"'%s-%s.dtbo\n",
				slot->slotno,
				slot->part_number, slot->version);

		/* no specific capemap node; request the part number + .dtbo*/
		slot->dtbo = kasprintf(GFP_KERNEL, "%s-%s.dtbo",
				slot->part_number, slot->version);
	}

	if (slot->dtbo == NULL) {
		mutex_unlock(&info->capemap_mutex);
		dev_err(dev, "slot #%d: Failed to get dtbo '%s'\n",
				slot->slotno, dtbo);
		return -ENOMEM;
	}

	dev_info(dev, "slot #%d: Requesting firmware '%s' for board-name '%s'"
			", version '%s'\n",
			slot->slotno,
			slot->dtbo, slot->board_name, slot->version);

	err = request_firmware(&slot->fw, slot->dtbo, dev);
	if (err != 0) {
		dev_err(dev, "failed to load firmware '%s'\n", slot->dtbo);
		mutex_unlock(&info->capemap_mutex);
		goto err_fail_no_fw;
	}

	dev_info(dev, "slot #%d: dtbo '%s' loaded; converting to live tree\n",
			slot->slotno, slot->dtbo);

	mutex_unlock(&info->capemap_mutex);

	of_fdt_unflatten_tree((void *)slot->fw->data, &slot->overlay);
	if (slot->overlay == NULL) {
		dev_err(dev, "slot #%d: Failed to unflatten\n",
				slot->slotno);
		err = -EINVAL;
		goto err_fail;
	}

	/* mark it as detached */
	of_node_set_flag(slot->overlay, OF_DETACHED);

	/* perform resolution */
	err = of_resolve(slot->overlay);
	if (err != 0) {
		dev_err(dev, "slot #%d: Failed to resolve tree\n",
				slot->slotno);
		goto err_fail;
	}

	err = bone_capemgr_verify_overlay(slot);
	if (err != 0) {
		dev_err(dev, "slot #%d: Failed verification\n",
				slot->slotno);
		goto err_fail;
	}

	/* now build an overlay info array */
	err = of_build_overlay_info(slot->overlay,
			&slot->ovinfo_cnt, &slot->ovinfo);
	if (err != 0) {
		dev_err(dev, "slot #%d: Failed to build overlay info\n",
				slot->slotno);
		goto err_fail;
	}

	dev_info(dev, "slot #%d: #%d overlays\n",
			slot->slotno, slot->ovinfo_cnt);

	err = of_overlay(slot->ovinfo_cnt, slot->ovinfo);
	if (err != 0) {
		if (err != 0) {
			dev_err(dev, "slot #%d: Failed to overlay\n",
					slot->slotno);
			goto err_fail_overlay;
		}
	}

	dev_info(dev, "slot #%d: Applied #%d overlays.\n",
			slot->slotno, slot->ovinfo_cnt);

	slot->loading = 0;
	slot->loaded = 1;

	return 0;

err_fail_overlay:

	of_free_overlay_info(slot->ovinfo_cnt, slot->ovinfo);
	slot->ovinfo_cnt = 0;
	slot->ovinfo = NULL;

err_fail:

	/* we can't free the overlay, because the unflatten method is a mess */
	/* __of_free_tree(slot->overlay); */
	slot->overlay = NULL;

	release_firmware(slot->fw);
	slot->fw = NULL;

err_fail_no_fw:
	slot->loading = 0;
	return err;
}

static int bone_capemgr_unload(struct bone_cape_slot *slot)
{
	if (!slot->loaded || slot->ovinfo == NULL)
		return -EINVAL;

	of_overlay_revert(slot->ovinfo_cnt, slot->ovinfo);

	slot->ovinfo_cnt = 0;
	kfree(slot->ovinfo);

	slot->loaded = 0;

	return 0;

}

/* slots_list_mutex must be taken */
static int bone_capemgr_remove_slot_no_lock(struct bone_cape_slot *slot)
{
	struct bone_capemgr_info *info = slot->info;
	struct device *dev = &info->pdev->dev;
	int ret;

	if (slot == NULL)
		return 0;

	if (slot->loaded && slot->ovinfo) {
		/* unload just in case */
		ret = bone_capemgr_unload(slot);
		if (ret != 0) {
			dev_err(dev, "Unable to unload slot #%d\n", slot->slotno);
			return ret;
		}
	}

	/* if probed OK, remove the sysfs nodes */
	if (slot->probed && !slot->probe_failed)
		bone_cape_slot_sysfs_unregister(slot);

	/* remove it from the list */
	list_del(&slot->node);

	devm_kfree(dev, slot);

	return 0;
}

static int bone_capemgr_remove_slot(struct bone_cape_slot *slot)
{
	struct bone_capemgr_info *info = slot->info;
	int ret;

	mutex_lock(&info->slots_list_mutex);
	ret = bone_capemgr_remove_slot_no_lock(slot);
	mutex_unlock(&info->slots_list_mutex);

	return ret;
}

static int bone_slot_fill_override(struct bone_cape_slot *slot,
		struct device_node *node,
		const char *part_number, const char *version)
{
	const struct ee_field *sig_field;
	struct property *prop;
	int i, len, has_part_number;
	u32 val;
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

	/* and now, fill any override data from the node */
	has_part_number = 0;
	if (node != NULL) {
		for (i = 0; i < ARRAY_SIZE(cape_sig_fields); i++) {

			sig_field = &cape_sig_fields[i];

			/* find property with the same name (if any) */
			prop = of_find_property(node, sig_field->name, NULL);
			if (prop == NULL)
				continue;

			/* point to the entry */
			p = slot->signature + sig_field->start;

			/* copy and zero out any remainder */
			len = prop->length;
			if (prop->length > sig_field->size)
				len = sig_field->size;
			memcpy(p, prop->value, len);
			if (len < sig_field->size)
				memset(p + len, 0, sig_field->size - len);

			/* remember if we got a part number which is required */
			if (i == CAPE_EE_FIELD_PART_NUMBER && len > 0)
				has_part_number = 1;
		}

		if (of_property_read_u32(node, "priority", &val) != 0)
			val = 0;
		slot->priority = val;
	}

	/* if a part_number is supplied use it */
	if (part_number && (len = strlen(part_number)) > 0) {
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
	if (version && (len = strlen(version)) > 0) {
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
bone_capemgr_add_slot(struct bone_capemgr_info *info, struct device_node *node,
		const char *part_number, const char *version)
{
	struct device_node *eeprom_node;
	struct bone_cape_slot *slot;
	struct device *dev = &info->pdev->dev;
	int slotno;
	int ret;

	eeprom_node = NULL;

	slotno = atomic_inc_return(&info->next_slot_nr) - 1;

	slot = devm_kzalloc(dev, sizeof(*slot), GFP_KERNEL);
	if (slot == NULL) {
		ret = -ENOMEM;
		goto err_no_mem;
	}
	slot->info = info;
	slot->slotno = slotno;

	if (node && !of_property_read_bool(node, "ti,cape-override")) {
		ret = of_property_read_u32(node, "eeprom",
				&slot->eeprom_handle);
		if (ret != 0) {
			dev_err(dev, "slot #%d: failed to locate eeprom\n",
					slotno);
			goto err_no_eeprom;
		}
		eeprom_node = of_find_node_by_phandle(slot->eeprom_handle);
		if (eeprom_node == NULL) {
			dev_err(dev, "slot #%d: failed to find eeprom node\n",
					slotno);
			ret = -ENODEV;
			goto err_no_eeprom_node;
		}
		slot->client = of_find_i2c_device_by_node(eeprom_node);
		if (slot->client == NULL) {
			dev_err(dev, "slot #%d: failed to find i2c client\n",
					slotno);
			ret = -ENODEV;
			goto err_no_eeprom_client;
		}
		/* release ref to the node & get ref of the i2c client */
		of_node_put(eeprom_node);
		eeprom_node = NULL;
		i2c_use_client(slot->client);

		/* grab the memory accessor of the eeprom */
		slot->macc = i2c_eeprom_get_memory_accessor(slot->client);
		if (IS_ERR_OR_NULL(slot->macc)) {
			dev_err(dev, "slot #%d: failed to get "
					"memory accessor\n", slotno);
			ret = slot->macc == NULL ? -ENODEV :
				PTR_ERR(slot->macc);
			slot->macc = NULL;
			goto err_no_eeprom_macc;
		}

	} else {
		if (node)
			dev_info(dev, "slot #%d: specific override\n", slotno);
		else
			dev_info(dev, "slot #%d: generic override\n", slotno);

		/* fill in everything with defaults first */
		ret = bone_slot_fill_override(slot, node, part_number, version);
		if (ret != 0) {
			dev_err(dev, "slot #%d: override failed\n",
					slotno);
			goto err_no_eeprom;
		}
	}

	ret = bone_slot_scan(slot);
	if (ret != 0) {

		if (!slot->probe_failed) {
			dev_err(dev, "slot #%d: scan failed\n",
					slotno);
			goto err_bad_scan;
		}

		dev_err(dev, "slot #%d: No cape found\n",
				slotno);
		/* but all is fine */
	} else {

		dev_info(dev, "slot #%d: '%s'\n",
				slotno, slot->text_id);

		ret = bone_cape_slot_sysfs_register(slot);
		if (ret != 0) {
			dev_err(dev, "slot #%d: sysfs register failed\n",
					slotno);
			goto err_no_sysfs;
		}

	}

	/* add to the slot list */
	mutex_lock(&info->slots_list_mutex);
	list_add_tail(&slot->node, &info->slot_list);
	mutex_unlock(&info->slots_list_mutex);

	return slot;

err_no_sysfs:
err_bad_scan:
err_no_eeprom_macc:
	i2c_release_client(slot->client);
err_no_eeprom_client:
	of_node_put(eeprom_node);	/* handles NULL */
err_no_eeprom_node:
	/* nothing */
err_no_eeprom:
	devm_kfree(dev, slot);

err_no_mem:
	return ERR_PTR(ret);
}

static int lowest_loading_cape(struct bone_cape_slot *slot)
{
	struct bone_capemgr_info *info = slot->info;
	int my_prio = slot->priority;
	struct device *dev = &info->pdev->dev;
	int ret;

	dev_info(dev, "loader: check slot-%d %s:%s (prio %d)\n", slot->slotno,
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

static int bone_capemgr_loader(void *data)
{
	struct bone_cape_slot *slot = data;
	struct bone_capemgr_info *info = slot->info;
	struct device *dev = &info->pdev->dev;
	int ret;

	dev_info(dev, "loader: before slot-%d %s:%s (prio %d)\n", slot->slotno,
			slot->part_number, slot->version, slot->priority);

	/*
	 * We have a basic priority based arbitration system
	 * Slots have priorities, so the lower priority ones
	 * should start loading first. So each time we end up
	 * here.
	 */
	ret = wait_event_interruptible(info->load_wq,
			lowest_loading_cape(slot));
	if (ret < 0) {
		dev_info(dev, "loader, Signal pending\n");
		return ret;
	}

	dev_info(dev, "loader: after slot-%d %s:%s (prio %d)\n", slot->slotno,
			slot->part_number, slot->version, slot->priority);

	ret = bone_capemgr_load(slot);

	slot->loading = 0;

	if (ret == 0)
		dev_info(dev, "loader: done slot-%d %s:%s (prio %d)\n",
			slot->slotno, slot->part_number, slot->version,
			slot->priority);
	else
		dev_err(dev, "loader: failed to load slot-%d %s:%s (prio %d)\n",
			slot->slotno, slot->part_number, slot->version,
			slot->priority);

	/* we're done, wake up all */
	wake_up_interruptible_all(&info->load_wq);

	/* not loaded? remove */
	if (ret != 0)
		bone_capemgr_remove_slot(slot);

	return ret;
}

static int
bone_capemgr_probe(struct platform_device *pdev)
{
	struct bone_capemgr_info *info;
	struct bone_baseboard *bbrd;
	struct bone_cape_slot *slot;
	struct device_node *pnode = pdev->dev.of_node;
	struct device_node *baseboardmaps_node;
	struct device_node *slots_node, *capemaps_node, *node;
	struct device_node *eeprom_node;
	const char *part_number;
	const char *board_name;
	const char *compatible_name;
	struct bone_capemap *capemap;
	int ret, len;

	/* we don't use platform_data at all; we require OF */
	if (pnode == NULL)
		return -ENOTSUPP;

	info = devm_kzalloc(&pdev->dev,
			sizeof(struct bone_capemgr_info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "Failed to allocate device structure\n");
		return -ENOMEM;
	}

	info->pdev = pdev;
	platform_set_drvdata(pdev, info);

	atomic_set(&info->next_slot_nr, 0);
	INIT_LIST_HEAD(&info->slot_list);
	mutex_init(&info->slots_list_mutex);

	INIT_LIST_HEAD(&info->capemap_list);
	mutex_init(&info->capemap_mutex);

	init_waitqueue_head(&info->load_wq);

	baseboardmaps_node = NULL;
	capemaps_node = NULL;

	/* find the baseboard */
	bbrd = &info->baseboard;

	baseboardmaps_node = of_get_child_by_name(pnode, "baseboardmaps");
	if (baseboardmaps_node == NULL) {
		dev_err(&pdev->dev, "Failed to get baseboardmaps node");
		ret = -ENODEV;
		goto err_exit;
	}

	/* get eeprom of the baseboard */
	ret = of_property_read_u32(pnode, "eeprom",
			&bbrd->eeprom_handle);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to locate baseboard eeprom\n");
		goto err_exit;
	}
	eeprom_node = of_find_node_by_phandle(bbrd->eeprom_handle);
	if (eeprom_node == NULL) {
		dev_err(&pdev->dev, "Failed to find baseboard eeprom node\n");
		ret = -ENODEV;
		goto err_exit;
	}
	bbrd->client = of_find_i2c_device_by_node(eeprom_node);
	of_node_put(eeprom_node);
	eeprom_node = NULL;
	if (bbrd->client == NULL) {
		dev_err(&pdev->dev, "Failed to find baseboard i2c client\n");
		ret = -ENODEV;
		goto err_exit;
	}

	/* release ref to the node & get ref of the i2c client */
	i2c_use_client(bbrd->client);

	/* grab the memory accessor of the eeprom */
	bbrd->macc = i2c_eeprom_get_memory_accessor(bbrd->client);
	if (IS_ERR_OR_NULL(bbrd->macc)) {
		dev_err(&pdev->dev, "Failed to get "
				"baseboard memory accessor\n");
		ret = bbrd->macc == NULL ? -ENODEV :
			PTR_ERR(bbrd->macc);
		bbrd->macc = NULL;
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
		dev_err(&pdev->dev, "Failed to allocate compatible name\n");
		ret = -ENOMEM;
		goto err_exit;
	}
	of_node_put(node);

	dev_info(&pdev->dev, "compatible-baseboard=%s\n",
			bbrd->compatible_name);

	/* iterate over any capemaps */
	capemaps_node = of_get_child_by_name(pnode, "capemaps");
	if (capemaps_node != NULL) {

		for_each_child_of_node(capemaps_node, node) {

			/* there must be part-number */
			if (of_property_read_string(node, "part-number",
						&part_number) != 0)
				continue;

			/* if matches the disabled ones skip */
			if (bone_match_cape(disable_partno,
						part_number, NULL)) {
				dev_info(&pdev->dev,
					"Skipping disabled cape with "
						"part# %s\n", part_number);
				continue;
			}

			len = sizeof(*capemap) + strlen(part_number) + 1;
			capemap = devm_kzalloc(&pdev->dev, len, GFP_KERNEL);
			if (capemap == NULL) {
				dev_err(&pdev->dev, "Failed to allocate "
						"capemap\n");
				ret = -ENOMEM;
				goto err_exit;
			}
			capemap->part_number = (char *)(capemap + 1);
			capemap->map_node = of_node_get(node);
			strcpy(capemap->part_number, part_number);

			/* add to the slot list */
			mutex_lock(&info->capemap_mutex);
			list_add_tail(&capemap->node, &info->capemap_list);
			info->capemaps_nr++;
			mutex_unlock(&info->capemap_mutex);
		}
		of_node_put(capemaps_node);
		capemaps_node = NULL;
	}

	/* iterate over any slots */
	slots_node = of_get_child_by_name(pnode, "slots");
	if (slots_node != NULL) {
		for_each_child_of_node(slots_node, node) {

			/* check if the override is compatible */
			if (!bone_is_compatible_override(node,
						bbrd->compatible_name))
				continue;

			slot = bone_capemgr_add_slot(info, node,
					NULL, NULL);
			if (IS_ERR(slot)) {
				dev_err(&pdev->dev, "Failed to add slot #%d\n",
					atomic_read(&info->next_slot_nr));
				ret = PTR_ERR(slot);
				goto err_exit;
			}
			/* note that slot may be NULL (means it was disabled) */
		}
		of_node_put(slots_node);
	}
	slots_node = NULL;

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to pm_runtime_get_sync()\n");
		goto err_exit;
	}

	pm_runtime_put(&pdev->dev);

	bone_capemgr_info_sysfs_register(info);

	/* now load each (take lock to be sure */
	mutex_lock(&info->slots_list_mutex);
	list_for_each_entry(slot, &info->slot_list, node) {

		/* if matches the disabled ones skip */
		if (bone_match_cape(disable_partno,
					slot->part_number, NULL)) {
			dev_info(&pdev->dev,
				"Skipping loading of disabled cape with "
					"part# %s\n", slot->part_number);
			continue;
		}

		if (!slot->probe_failed && !slot->loaded)
			slot->loading = 1;
	}

	/* now start the loader thread(s) (all at once) */
	list_for_each_entry(slot, &info->slot_list, node) {

		if (!slot->loading)
			continue;

		slot->loader_thread = kthread_run(bone_capemgr_loader,
				slot, "capemgr-loader-%d",
				slot->slotno);
		if (IS_ERR(slot->loader_thread)) {
			dev_warn(&pdev->dev, "slot #%d: Failed to "
					"start loader\n", slot->slotno);
			slot->loader_thread = NULL;
		}
	}
	mutex_unlock(&info->slots_list_mutex);

	dev_info(&pdev->dev, "initialized OK.\n");

	return 0;

err_exit:
	of_node_put(baseboardmaps_node);
	of_node_put(capemaps_node);
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, info);

	return ret;
}

static int bone_capemgr_remove(struct platform_device *pdev)
{
	struct bone_capemgr_info *info = platform_get_drvdata(pdev);
	struct bone_cape_slot *slot, *slotn;
	int ret;

	mutex_lock(&info->slots_list_mutex);
	list_for_each_entry_safe(slot, slotn, &info->slot_list, node)
		bone_capemgr_remove_slot_no_lock(slot);
	mutex_unlock(&info->slots_list_mutex);

	bone_capemgr_info_sysfs_unregister(info);

	platform_set_drvdata(pdev, NULL);

	ret = pm_runtime_get_sync(&pdev->dev);
	if (IS_ERR_VALUE(ret))
		return ret;

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	devm_kfree(&pdev->dev, info);

	return 0;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME
static int bone_capemgr_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_capemgr_info *_dev = platform_get_drvdata(pdev);

	(void)_dev;
	return 0;
}

static int bone_capemgr_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_capemgr_info *_dev = platform_get_drvdata(pdev);

	(void)_dev;
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */

static struct dev_pm_ops bone_capemgr_pm_ops = {
	SET_RUNTIME_PM_OPS(bone_capemgr_runtime_suspend,
			   bone_capemgr_runtime_resume, NULL)
};
#define BONE_CAPEMGR_PM_OPS (&bone_capemgr_pm_ops)
#else
#define BONE_CAPEMGR_PM_OPS NULL
#endif /* CONFIG_PM */

static struct platform_driver bone_capemgr_driver = {
	.probe		= bone_capemgr_probe,
	.remove		= bone_capemgr_remove,
	.driver		= {
		.name	= "bone-capemgr",
		.owner	= THIS_MODULE,
		.pm	= BONE_CAPEMGR_PM_OPS,
		.of_match_table = of_match_ptr(bone_capemgr_of_match),
	},
};

module_platform_driver(bone_capemgr_driver);

MODULE_AUTHOR("Pantelis Antoniou");
MODULE_DESCRIPTION("Beaglebone cape manager");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bone_capemgr");
