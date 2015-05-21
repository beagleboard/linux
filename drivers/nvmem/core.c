/*
 * nvmem framework core.
 *
 * Copyright (C) 2015 Srinivas Kandagatla <srinivas.kandagatla@linaro.org>
 * Copyright (C) 2013 Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/nvmem-provider.h>
#include <linux/nvmem-consumer.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

struct nvmem_device {
	const char		*name;
	struct regmap		*regmap;
	struct module		*owner;
	struct device		dev;
	int			stride;
	int			word_size;
	int			ncells;
	int			id;
	int			users;
	size_t			size;
	bool			read_only;
};

struct nvmem_cell {
	const char *name;
	int			offset;
	int			bytes;
	int			bit_offset;
	int			nbits;
	struct nvmem_device	*nvmem;
	struct list_head	node;
};

static DEFINE_MUTEX(nvmem_mutex);
static DEFINE_IDA(nvmem_ida);

static LIST_HEAD(nvmem_cells);
static DEFINE_MUTEX(nvmem_cells_mutex);

#define to_nvmem_device(d) container_of(d, struct nvmem_device, dev)

static ssize_t bin_attr_nvmem_read(struct file *filp, struct kobject *kobj,
				    struct bin_attribute *attr,
				    char *buf, loff_t pos, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct nvmem_device *nvmem = to_nvmem_device(dev);
	int rc;

	/* Stop the user from reading */
	if (pos > nvmem->size)
		return 0;

	if (pos + count > nvmem->size)
		count = nvmem->size - pos;

	count = count/nvmem->word_size * nvmem->word_size;

	rc = regmap_raw_read(nvmem->regmap, pos, buf, count);

	if (IS_ERR_VALUE(rc))
		return rc;

	return count;
}

static ssize_t bin_attr_nvmem_write(struct file *filp, struct kobject *kobj,
				     struct bin_attribute *attr,
				     char *buf, loff_t pos, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct nvmem_device *nvmem = to_nvmem_device(dev);
	int rc;

	/* Stop the user from writing */
	if (pos > nvmem->size)
		return 0;

	if (pos + count > nvmem->size)
		count = nvmem->size - pos;

	count = count/nvmem->word_size * nvmem->word_size;

	rc = regmap_raw_write(nvmem->regmap, pos, buf, count);

	if (IS_ERR_VALUE(rc))
		return rc;

	return count;
}

/* default read/write permissions */
static struct bin_attribute bin_attr_nvmem = {
	.attr	= {
		.name	= "nvmem",
		.mode	= S_IWUSR | S_IRUGO,
	},
	.read	= bin_attr_nvmem_read,
	.write	= bin_attr_nvmem_write,
};

static struct bin_attribute *nvmem_bin_attributes[] = {
	&bin_attr_nvmem,
	NULL,
};

static const struct attribute_group nvmem_bin_group = {
	.bin_attrs	= nvmem_bin_attributes,
};

static const struct attribute_group *nvmem_dev_groups[] = {
	&nvmem_bin_group,
	NULL,
};

/* read only permission */
static struct bin_attribute bin_attr_ro_nvmem = {
	.attr	= {
		.name	= "nvmem",
		.mode	= S_IRUGO,
	},
	.read	= bin_attr_nvmem_read,
};

static struct bin_attribute *nvmem_bin_ro_attributes[] = {
	&bin_attr_ro_nvmem,
	NULL,
};
static const struct attribute_group nvmem_bin_ro_group = {
	.bin_attrs	= nvmem_bin_ro_attributes,
};

static void nvmem_release(struct device *dev)
{
	struct nvmem_device *nvmem = to_nvmem_device(dev);

	ida_simple_remove(&nvmem_ida, nvmem->id);
	kfree(nvmem);
}

static struct class nvmem_class = {
	.name		= "nvmem",
	.dev_groups	= nvmem_dev_groups,
	.dev_release	= nvmem_release,
};

static int of_nvmem_match(struct device *dev, const void *nvmem_np)
{
	return dev->of_node == nvmem_np;
}

static struct nvmem_device *of_nvmem_find(struct device_node *nvmem_np)
{
	struct device *d;

	if (!nvmem_np)
		return NULL;

	d = class_find_device(&nvmem_class, NULL, nvmem_np, of_nvmem_match);

	return d ? to_nvmem_device(d) : NULL;
}

static struct nvmem_cell *nvmem_find_cell(const char *cell_id)
{
	struct nvmem_cell *p;

	list_for_each_entry(p, &nvmem_cells, node) {
		if (p && !strcmp(p->name, cell_id))
			return p;
	}

	return NULL;
}

static void nvmem_cell_drop(struct nvmem_cell *cell)
{
	mutex_lock(&nvmem_cells_mutex);
	list_del(&cell->node);
	mutex_unlock(&nvmem_cells_mutex);
	kfree(cell);
}

static void nvmem_device_remove_all_cells(struct nvmem_device *nvmem)
{
	struct nvmem_cell *cell = NULL;
	struct list_head *p, *n;

	list_for_each_safe(p, n, &nvmem_cells) {
		cell = list_entry(p, struct nvmem_cell, node);
		if (cell->nvmem == nvmem)
			nvmem_cell_drop(cell);
	}
}

static void nvmem_cell_add(struct nvmem_cell *cell)
{
	mutex_lock(&nvmem_cells_mutex);
	list_add_tail(&cell->node, &nvmem_cells);
	mutex_unlock(&nvmem_cells_mutex);
}

static int nvmem_cell_info_to_nvmem_cell(struct nvmem_device *nvmem,
				   struct nvmem_cell_info *info,
				   struct nvmem_cell *cell)
{
	cell->nvmem = nvmem;
	cell->offset = info->offset;
	cell->bytes = info->bytes;
	cell->name = info->name;

	cell->bit_offset = info->bit_offset;
	cell->nbits = info->nbits;

	if (cell->nbits)
		cell->bytes = DIV_ROUND_UP(cell->nbits + cell->bit_offset,
					   BITS_PER_BYTE);

	if (!IS_ALIGNED(cell->offset, nvmem->stride)) {
		dev_err(&nvmem->dev,
			"cell %s unaligned to nvmem stride %d\n",
			cell->name, nvmem->stride);
		return -EINVAL;
	}

	return 0;
}

static int nvmem_add_cells(struct nvmem_device *nvmem,
			   struct nvmem_config *cfg)
{
	struct nvmem_cell **cells;
	struct nvmem_cell_info *info = cfg->cells;
	int i, rval;

	cells = kzalloc(sizeof(*cells) * cfg->ncells, GFP_KERNEL);
	if (!cells)
		return -ENOMEM;

	for (i = 0; i < cfg->ncells; i++) {
		cells[i] = kzalloc(sizeof(struct nvmem_cell), GFP_KERNEL);
		if (!cells[i]) {
			rval = -ENOMEM;
			goto err;
		}

		rval = nvmem_cell_info_to_nvmem_cell(nvmem, &info[i], cells[i]);
		if (IS_ERR_VALUE(rval)) {
			kfree(cells[i]);
			goto err;
		}

		nvmem_cell_add(cells[i]);
	}

	nvmem->ncells = cfg->ncells;
	/* remove tmp array */
	kfree(cells);

	return 0;
err:
	while (--i)
		nvmem_cell_drop(cells[i]);

	return rval;
}

/**
 * nvmem_register(): Register a nvmem device for given nvmem.
 * Also creates an binary entry in /sys/class/nvmem/dev-name/nvmem
 *
 * @nvmem: nvmem device that needs to be created
 *
 * The return value will be an ERR_PTR() on error or a valid pointer
 * to nvmem_device.
 */

struct nvmem_device *nvmem_register(struct nvmem_config *config)
{
	struct nvmem_device *nvmem;
	struct regmap *rm;
	int rval;

	if (!config->dev)
		return ERR_PTR(-EINVAL);

	rm = dev_get_regmap(config->dev, NULL);
	if (!rm) {
		dev_err(config->dev, "Regmap not found\n");
		return ERR_PTR(-EINVAL);
	}

	nvmem = kzalloc(sizeof(*nvmem), GFP_KERNEL);
	if (!nvmem)
		return ERR_PTR(-ENOMEM);

	nvmem->id = ida_simple_get(&nvmem_ida, 0, 0, GFP_KERNEL);
	if (nvmem->id < 0) {
		kfree(nvmem);
		return ERR_PTR(nvmem->id);
	}

	nvmem->regmap = rm;
	nvmem->owner = config->owner;
	nvmem->stride = regmap_get_reg_stride(rm);
	nvmem->word_size = regmap_get_val_bytes(rm);
	nvmem->size = regmap_get_max_register(rm) + nvmem->stride;
	nvmem->dev.class = &nvmem_class;
	nvmem->dev.parent = config->dev;
	nvmem->dev.of_node = config->dev->of_node;
	dev_set_name(&nvmem->dev, "%s%d",
		     config->name ? : "nvmem", config->id);

	nvmem->read_only = of_property_read_bool(nvmem->dev.of_node,
						 "read-only");

	device_initialize(&nvmem->dev);

	dev_dbg(&nvmem->dev, "Registering nvmem device %s\n",
		dev_name(&nvmem->dev));

	rval = device_add(&nvmem->dev);
	if (rval) {
		ida_simple_remove(&nvmem_ida, nvmem->id);
		kfree(nvmem);
		return ERR_PTR(rval);
	}

	/* update sysfs attributes */
	if (nvmem->read_only)
		sysfs_update_group(&nvmem->dev.kobj, &nvmem_bin_ro_group);

	if (config->cells)
		nvmem_add_cells(nvmem, config);

	return nvmem;
}
EXPORT_SYMBOL_GPL(nvmem_register);

/**
 * nvmem_unregister(): Unregister previously registered nvmem device
 *
 * @nvmem: Pointer to previously registered nvmem device.
 *
 * The return value will be an non zero on error or a zero on success.
 */
int nvmem_unregister(struct nvmem_device *nvmem)
{
	mutex_lock(&nvmem_mutex);
	if (nvmem->users) {
		mutex_unlock(&nvmem_mutex);
		return -EBUSY;
	}
	mutex_unlock(&nvmem_mutex);

	nvmem_device_remove_all_cells(nvmem);
	device_del(&nvmem->dev);

	return 0;
}
EXPORT_SYMBOL_GPL(nvmem_unregister);

static struct nvmem_device *__nvmem_device_get(struct device_node *np,
					       struct nvmem_cell **cellp,
					       const char *cell_id)
{
	struct nvmem_device *nvmem = NULL;

	mutex_lock(&nvmem_mutex);

	if (np) {
		nvmem = of_nvmem_find(np);
		if (!nvmem) {
			mutex_unlock(&nvmem_mutex);
			return ERR_PTR(-EPROBE_DEFER);
		}
	} else {
		struct nvmem_cell *cell = nvmem_find_cell(cell_id);

		if (cell) {
			nvmem = cell->nvmem;
			*cellp = cell;
		}

		if (!nvmem) {
			mutex_unlock(&nvmem_mutex);
			return ERR_PTR(-ENOENT);
		}
	}

	nvmem->users++;
	mutex_unlock(&nvmem_mutex);

	if (!try_module_get(nvmem->owner)) {
		dev_err(&nvmem->dev,
			"could not increase module refcount for cell %s\n",
			nvmem->name);

		mutex_lock(&nvmem_mutex);
		nvmem->users--;
		mutex_unlock(&nvmem_mutex);

		return ERR_PTR(-EINVAL);
	}

	return nvmem;
}

static int __nvmem_device_put(struct nvmem_device *nvmem)
{
	module_put(nvmem->owner);
	mutex_lock(&nvmem_mutex);
	nvmem->users--;
	mutex_unlock(&nvmem_mutex);

	return 0;
}

static int nvmem_match(struct device *dev, const void *data)
{
	return !strcmp(dev_name(dev), (const char *)data);
}

static struct nvmem_device *nvmem_find(const char *name)
{
	struct device *d;

	d = class_find_device(&nvmem_class, NULL, (void *)name, nvmem_match);

	return d ? to_nvmem_device(d) : NULL;
}

struct nvmem_device *nvmem_device_get(struct device *dev, const char *dev_name)
{
	struct device_node *nvmem_np, *np = dev->of_node;
	struct nvmem_device *nvmem;
	int index;

	if (np) { /* try dt first */
		index = of_property_match_string(np, "nvmem-names", dev_name);

		nvmem_np = of_parse_phandle(np, "nvmem", index);
		if (!nvmem_np)
			return ERR_PTR(-EINVAL);

		nvmem = __nvmem_device_get(nvmem_np, NULL, NULL);

		if (!IS_ERR(nvmem) || PTR_ERR(nvmem) == -EPROBE_DEFER)
			return nvmem;

	}

	return nvmem_find(dev_name);

}
EXPORT_SYMBOL_GPL(nvmem_device_get);

void nvmem_device_put(struct nvmem_device *nvmem)
{
	__nvmem_device_put(nvmem);
}
EXPORT_SYMBOL_GPL(nvmem_device_put);

static struct nvmem_cell *nvmem_cell_get_from_list(const char *cell_id)
{
	struct nvmem_cell *cell = NULL;
	struct nvmem_device *nvmem;

	nvmem = __nvmem_device_get(NULL, &cell, cell_id);
	if (IS_ERR(nvmem))
		return (struct nvmem_cell *)nvmem;


	return cell;

}

static struct nvmem_cell *of_nvmem_cell_get(struct device_node *np,
					    const char *name)
{
	struct device_node *cell_np, *nvmem_np;
	struct nvmem_cell *cell;
	struct nvmem_device *nvmem;
	const __be32 *addr;
	int rval, len, index;

	index = of_property_match_string(np, "nvmem-cell-names", name);

	cell_np = of_parse_phandle(np, "nvmem-cell", index);
	if (!cell_np)
		return ERR_PTR(-EINVAL);

	nvmem_np = of_get_next_parent(cell_np);
	if (!nvmem_np)
		return ERR_PTR(-EINVAL);

	nvmem = __nvmem_device_get(nvmem_np, NULL, NULL);
	if (IS_ERR(nvmem))
		return (struct nvmem_cell *)nvmem;

	addr = of_get_property(cell_np, "reg", &len);
	if (!addr || (len < 2 * sizeof(int))) {
		dev_err(&nvmem->dev, "of_i2c: invalid reg on %s\n",
			cell_np->full_name);
		rval  = -EINVAL;
		goto err_mem;
	}

	cell = kzalloc(sizeof(*cell), GFP_KERNEL);
	if (!cell) {
		rval = -ENOMEM;
		goto err_mem;
	}

	cell->nvmem = nvmem;
	cell->offset = be32_to_cpup(addr++);
	cell->bytes = be32_to_cpup(addr);
	cell->name = cell_np->name;

	of_property_read_u32(cell_np, "bit-offset", &cell->bit_offset);
	of_property_read_u32(cell_np, "nbits", &cell->nbits);

	if (cell->nbits)
		cell->bytes = DIV_ROUND_UP(cell->nbits + cell->bit_offset,
					   BITS_PER_BYTE);

	if (!IS_ALIGNED(cell->offset, nvmem->stride)) {
			dev_err(&nvmem->dev,
				"cell %s unaligned to nvmem stride %d\n",
				cell->name, nvmem->stride);
		rval  = -EINVAL;
		goto err_sanity;
	}

	nvmem_cell_add(cell);

	return cell;

err_sanity:
	kfree(cell);

err_mem:
	__nvmem_device_put(nvmem);

	return ERR_PTR(rval);

}

/**
 * nvmem_cell_get(): Get nvmem cell of device form a given index
 *
 * @dev node: Device tree node that uses the nvmem cell
 * @index: nvmem index in nvmems property.
 *
 * The return value will be an ERR_PTR() on error or a valid pointer
 * to a struct nvmem_cell.  The nvmem_cell will be freed by the
 * nvmem_cell_put().
 */
struct nvmem_cell *nvmem_cell_get(struct device *dev, const char *cell_id)
{
	struct nvmem_cell *cell;

	if (dev->of_node) { /* try dt first */
		cell = of_nvmem_cell_get(dev->of_node, cell_id);
		if (!IS_ERR(cell) || PTR_ERR(cell) == -EPROBE_DEFER)
			return cell;
	}

	return nvmem_cell_get_from_list(cell_id);

}
EXPORT_SYMBOL_GPL(nvmem_cell_get);

/**
 * nvmem_cell_put(): Release previously allocated nvmem cell.
 *
 * @cell: Previously allocated nvmem cell by nvmem_cell_get()
 * or nvmem_cell_get().
 */
void nvmem_cell_put(struct nvmem_cell *cell)
{
	struct nvmem_device *nvmem = cell->nvmem;

	__nvmem_device_put(nvmem);
	nvmem_cell_drop(cell);
}
EXPORT_SYMBOL_GPL(nvmem_cell_put);

static inline void nvmem_shift_read_buffer_in_place(struct nvmem_cell *cell,
						    void *buf)
{
	u8 *p, *b;
	int i, bit_offset = cell->bit_offset;

	p = b = buf;
	if (bit_offset) {
		/* First shift */
		*b++ >>= bit_offset;

		/* setup rest of the bytes if any */
		for (i = 1; i < cell->bytes; i++) {
			/* Get bits from next byte and shift them towards msb */
			*p |= *b << (BITS_PER_BYTE - bit_offset);

			p = b;
			*b++ >>= bit_offset;
		}

		/* result fits in less bytes */
		if (cell->bytes != DIV_ROUND_UP(cell->nbits, BITS_PER_BYTE))
			*p-- = 0;
	}
	/* clear msb bits if any leftover in the last byte */
	*p &= GENMASK((cell->nbits%BITS_PER_BYTE) - 1, 0);
}

static int __nvmem_cell_read(struct nvmem_device *nvmem,
		      struct nvmem_cell *cell,
		      void *buf, ssize_t *len)
{
	int rc;

	rc = regmap_raw_read(nvmem->regmap, cell->offset, buf, cell->bytes);

	if (IS_ERR_VALUE(rc))
		return rc;

	/* shift bits in-place */
	if (cell->bit_offset || cell->bit_offset)
		nvmem_shift_read_buffer_in_place(cell, buf);

	*len = cell->bytes;

	return *len;
}
/**
 * nvmem_cell_read(): Read a given nvmem cell
 *
 * @cell: nvmem cell to be read.
 * @len: pointer to length of cell which will be populated on successful read.
 *
 * The return value will be an ERR_PTR() on error or a valid pointer
 * to a char * bufffer.  The buffer should be freed by the consumer with a
 * kfree().
 */
void *nvmem_cell_read(struct nvmem_cell *cell, ssize_t *len)
{
	struct nvmem_device *nvmem = cell->nvmem;
	u8 *buf;
	int rc;

	if (!nvmem || !nvmem->regmap)
		return ERR_PTR(-EINVAL);

	buf = kzalloc(cell->bytes, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	rc = __nvmem_cell_read(nvmem, cell, buf, len);
	if (IS_ERR_VALUE(rc)) {
		kfree(buf);
		return ERR_PTR(rc);
	}

	return buf;
}
EXPORT_SYMBOL_GPL(nvmem_cell_read);

static inline void *nvmem_cell_prepare_write_buffer(struct nvmem_cell *cell,
						    u8 *_buf, int len)
{
	struct nvmem_device *nvmem = cell->nvmem;
	int i, rc, nbits, bit_offset = cell->bit_offset;
	u8 v, *p, *buf, *b, pbyte, pbits;

	nbits = cell->nbits;
	buf = kzalloc(cell->bytes, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	memcpy(buf, _buf, len);
	p = b = buf;

	if (bit_offset) {
		pbyte = *b;
		*b <<= bit_offset;

		/* setup the first byte with lsb bits from nvmem */
		rc = regmap_raw_read(nvmem->regmap, cell->offset, &v, 1);
		*b++ |= GENMASK(bit_offset - 1, 0) & v;

		/* setup rest of the byte if any */
		for (i = 1; i < cell->bytes; i++) {
			/* Get last byte bits and shift them towards lsb */
			pbits = pbyte >> (BITS_PER_BYTE - 1 - bit_offset);
			pbyte = *b;
			p = b;
			*b <<= bit_offset;
			*b++ |= pbits;
		}
	}

	/* if it's not end on byte boundary */
	if ((nbits + bit_offset) % BITS_PER_BYTE) {
		/* setup the last byte with msb bits from nvmem */
		rc = regmap_raw_read(nvmem->regmap,
				    cell->offset + cell->bytes - 1, &v, 1);
		*p |= GENMASK(7, (nbits + bit_offset) % BITS_PER_BYTE) & v;

	}

	return buf;
}

/**
 * nvmem_cell_write(): Write to a given nvmem cell
 *
 * @cell: nvmem cell to be written.
 * @buf: Buffer to be written.
 * @len: length of buffer to be written to nvmem cell.
 *
 * The return value will be an length of bytes written or non zero on failure.
 */
int nvmem_cell_write(struct nvmem_cell *cell, void *buf, ssize_t len)
{
	struct nvmem_device *nvmem = cell->nvmem;
	int rc;
	void *wbuf = buf;

	if (!nvmem || !nvmem->regmap || nvmem->read_only ||
	    (cell->bit_offset == 0 && len != cell->bytes))
		return -EINVAL;

	if (cell->bit_offset || cell->nbits) {
		wbuf = nvmem_cell_prepare_write_buffer(cell, buf, len);
		if (IS_ERR(wbuf))
			return PTR_ERR(wbuf);
	}

	rc = regmap_raw_write(nvmem->regmap, cell->offset, wbuf, cell->bytes);

	/* free the tmp buffer */
	if (cell->bit_offset)
		kfree(wbuf);

	if (IS_ERR_VALUE(rc))
		return rc;

	return len;
}
EXPORT_SYMBOL_GPL(nvmem_cell_write);

int nvmem_device_cell_read(struct nvmem_device *nvmem,
			   struct nvmem_cell_info *info, void *buf)
{
	struct nvmem_cell cell;
	int rc, len;

	if (!nvmem || !nvmem->regmap)
		return -EINVAL;

	rc = nvmem_cell_info_to_nvmem_cell(nvmem, info, &cell);
	if (IS_ERR_VALUE(rc))
		return rc;

	rc = __nvmem_cell_read(nvmem, &cell, buf, &len);
	if (IS_ERR_VALUE(rc))
		return rc;

	return len;
}
EXPORT_SYMBOL_GPL(nvmem_device_cell_read);

int nvmem_device_cell_write(struct nvmem_device *nvmem,
			    struct nvmem_cell_info *info, void *buf)
{
	struct nvmem_cell cell;
	int rc;

	if (!nvmem || !nvmem->regmap)
		return -EINVAL;

	rc = nvmem_cell_info_to_nvmem_cell(nvmem, info, &cell);
	if (IS_ERR_VALUE(rc))
		return rc;

	return nvmem_cell_write(&cell, buf, cell.bytes);
}
EXPORT_SYMBOL_GPL(nvmem_device_cell_write);

int nvmem_device_read(struct nvmem_device *nvmem,
		      unsigned int offset,
		      size_t bytes, void *buf)
{
	int rc;

	if (!nvmem || !nvmem->regmap)
		return -EINVAL;

	rc = regmap_raw_read(nvmem->regmap, offset, buf, bytes);

	if (IS_ERR_VALUE(rc))
		return rc;

	return bytes;
}
EXPORT_SYMBOL_GPL(nvmem_device_read);

int nvmem_device_write(struct nvmem_device *nvmem,
		       unsigned int offset,
		       size_t bytes, void *buf)
{
	int rc;

	if (!nvmem || !nvmem->regmap)
		return -EINVAL;

	rc = regmap_raw_write(nvmem->regmap, offset, buf, bytes);

	if (IS_ERR_VALUE(rc))
		return rc;


	return bytes;
}
EXPORT_SYMBOL_GPL(nvmem_device_write);

static int nvmem_init(void)
{
	return class_register(&nvmem_class);
}

static void nvmem_exit(void)
{
	class_unregister(&nvmem_class);
}

subsys_initcall(nvmem_init);
module_exit(nvmem_exit);

MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com");
MODULE_DESCRIPTION("nvmem Driver Core");
MODULE_LICENSE("GPL v2");
