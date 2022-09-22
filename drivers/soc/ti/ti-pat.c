// SPDX-License-Identifier: GPL-2.0
/*
 * TI PAT mapped DMA-BUF memory re-exporter
 *
 * Copyright (C) 2018-2019 Texas Instruments Incorporated - https://www.ti.com/
 *	Andrew Davis <afd@ti.com>
 */

#include <linux/dma-buf.h>
#include <linux/fs.h>
#include <linux/genalloc.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

#include <linux/ti-pat.h>

/* TI PAT MMRS registers */
#define TI_PAT_MMRS_PID		0x0 /* Revision Register */
#define TI_PAT_MMRS_CONFIG	0x4 /* Config Register */
#define TI_PAT_MMRS_CONTROL	0x10 /* Control Register */

/* TI PAT CONTROL register field values */
#define TI_PAT_CONTROL_ARB_MODE_UF	0x0 /* Updates first */
#define TI_PAT_CONTROL_ARB_MODE_RR	0x2 /* Round-robin */

#define TI_PAT_CONTROL_PAGE_SIZE_4KB	0x0
#define TI_PAT_CONTROL_PAGE_SIZE_16KB	0x1
#define TI_PAT_CONTROL_PAGE_SIZE_64KB	0x2
#define TI_PAT_CONTROL_PAGE_SIZE_1MB	0x3

/* TI PAT TABLE registers */
#define TI_PAT_TABLE_ADDRL	0x0 /* Low address Register */
#define TI_PAT_TABLE_ADDRH	0x4 /* High address and enable Register */

static unsigned int ti_pat_page_sizes[] = {
	[TI_PAT_CONTROL_PAGE_SIZE_4KB]  = 4 * 1024,
	[TI_PAT_CONTROL_PAGE_SIZE_16KB] = 16 * 1024,
	[TI_PAT_CONTROL_PAGE_SIZE_64KB] = 64 * 1024,
	[TI_PAT_CONTROL_PAGE_SIZE_1MB]  = 1024 * 1024,
};

enum ti_pat_fields {
	/* Revision */
	F_PID_MAJOR,
	F_PID_MINOR,

	/* Controls */
	F_CONTROL_ARB_MODE,
	F_CONTROL_PAGE_SIZE,
	F_CONTROL_REPLACE_OID_EN,
	F_CONTROL_EN,

	/* sentinel */
	F_MMRS_FIELDS,

	/* Table */
	F_TABLE_ADDRL,
	F_TABLE_ADDRH,
	F_TABLE_ENABLE,

	/* sentinel */
	F_MAX_FIELDS
};

static struct reg_field ti_pat_reg_fields[] = {
	/* Revision */
	[F_PID_MAJOR]			= REG_FIELD(TI_PAT_MMRS_PID, 8, 10),
	[F_PID_MINOR]			= REG_FIELD(TI_PAT_MMRS_PID, 0, 5),
	/* Controls */
	[F_CONTROL_ARB_MODE]		= REG_FIELD(TI_PAT_MMRS_CONTROL, 6, 7),
	[F_CONTROL_PAGE_SIZE]		= REG_FIELD(TI_PAT_MMRS_CONTROL, 4, 5),
	[F_CONTROL_REPLACE_OID_EN]	= REG_FIELD(TI_PAT_MMRS_CONTROL, 1, 1),
	[F_CONTROL_EN]			= REG_FIELD(TI_PAT_MMRS_CONTROL, 0, 0),
	/* Table */
	[F_TABLE_ADDRL]			= REG_FIELD(TI_PAT_TABLE_ADDRL, 0, 31),
	[F_TABLE_ADDRH]			= REG_FIELD(TI_PAT_TABLE_ADDRH, 0, 3),
	[F_TABLE_ENABLE]		= REG_FIELD(TI_PAT_TABLE_ADDRH, 31, 31),
};

/**
 * struct ti_pat_data - PAT device instance data
 * @dev: PAT device structure
 * @mdev: misc device
 * @mmrs_fields: Register fields for both MMRS and TABLE
 * @page_count: Total number of pages in this PAT
 * @page_size: Size of region mapped by each page in bytes
 * @window_base: Base address of WINDOW region
 * @pool: Pool for managing translation space
 */
struct ti_pat_data {
	struct device *dev;
	struct miscdevice mdev;
	struct regmap_field *fields[F_MAX_FIELDS];
	unsigned int page_count;
	unsigned int page_size;
	phys_addr_t window_base;
	struct gen_pool *pool;
};

struct ti_pat_dma_buf_attachment {
	struct device *dev;
	struct sg_table *table;
	struct ti_pat_buffer *buffer;
	struct list_head list;
};

/**
 * struct ti_pat_buffer - Single buffer instance data
 * @pat: PAT instance for whom this buffer belongs
 * @i_dma_buf: Imported DMA-BUF buffer
 * @size: Total allocated size of this buffer
 * @offset: Allocated offset into the PAT window
 * @e_dma_buf: Exported DMA-BUF buffer
 * @attachment: Our attachment to the imported buffer
 * @sgt: DMA map of our imported buffer
 * @attachments: Attachments to this buffer
 * @map_count: Reference count of mappings to this buffer
 * @lock: Protect the attach list and map count
 */
struct ti_pat_buffer {
	struct ti_pat_data *pat;
	struct dma_buf *i_dma_buf;
	size_t size;
	unsigned long offset;
	struct dma_buf *e_dma_buf;

	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;

	struct list_head attachments;
	int map_count;

	struct mutex lock;
};

static const struct regmap_config ti_pat_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

static int ti_pat_dma_buf_attach(struct dma_buf *dmabuf,
				 struct dma_buf_attachment *attachment)
{
	struct ti_pat_dma_buf_attachment *a;
	struct ti_pat_buffer *buffer = dmabuf->priv;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	a->dev = attachment->dev;
	a->buffer = buffer;
	INIT_LIST_HEAD(&a->list);

	a->table = kzalloc(sizeof(*a->table), GFP_KERNEL);
	if (!a->table) {
		kfree(a);
		return -ENOMEM;
	}

	if (sg_alloc_table(a->table, 1, GFP_KERNEL)) {
		kfree(a->table);
		kfree(a);
		return -ENOMEM;
	}

	sg_set_page(a->table->sgl, pfn_to_page(PFN_DOWN(buffer->offset)), buffer->size, 0);

	attachment->priv = a;

	mutex_lock(&buffer->lock);
	/* First time attachment we attach to parent */
	if (list_empty(&buffer->attachments)) {
		buffer->attachment = dma_buf_attach(buffer->i_dma_buf, buffer->pat->dev);
		if (IS_ERR(buffer->attachment)) {
			dev_err(buffer->pat->dev, "Unable to attach to parent DMA-BUF\n");
			mutex_unlock(&buffer->lock);
			sg_free_table(a->table);
			kfree(a->table);
			kfree(a);
			return PTR_ERR(buffer->attachment);
		}
	}
	list_add(&a->list, &buffer->attachments);
	mutex_unlock(&buffer->lock);

	return 0;
}

static void ti_pat_dma_buf_detach(struct dma_buf *dmabuf,
				  struct dma_buf_attachment *attachment)
{
	struct ti_pat_dma_buf_attachment *a = attachment->priv;
	struct ti_pat_buffer *buffer = dmabuf->priv;

	mutex_lock(&buffer->lock);
	list_del(&a->list);
	/* Last attachment we detach from parent */
	if (list_empty(&buffer->attachments)) {
		dma_buf_detach(buffer->i_dma_buf, buffer->attachment);
		buffer->attachment = NULL;
	}
	mutex_unlock(&buffer->lock);

	sg_free_table(a->table);
	kfree(a->table);
	kfree(a);
}

static unsigned int ti_pat_table_index_from_page(size_t page)
{
	/* Every 256 pages start on a 4k boundary */
	unsigned int boundery = page / 256;
	unsigned int offset = page % 256;
	/* Each page occupies 8 bytes in the table */
	return (boundery * (4096 / 8)) + offset;
}

static void ti_pat_set_page(struct ti_pat_data *pat, size_t page_id, dma_addr_t dma_address)
{
	unsigned int index = ti_pat_table_index_from_page(page_id);

	/*
	 * Addresses will always be at least 4K aligned, so both high and low
	 * addresses are shifted by an additional 12 bits before being written
	 * to the PAT.
	 */
	u32 base_l = dma_address >> 12;
	u32 base_h = dma_address >> 44;

	dev_dbg(pat->dev, "Enabling PAT index: %zu pointing to %pad\n", page_id, &dma_address);

	regmap_fields_write(pat->fields[F_TABLE_ADDRL], index, base_l);
	regmap_fields_write(pat->fields[F_TABLE_ADDRH], index, base_h);
	regmap_fields_write(pat->fields[F_TABLE_ENABLE], index, 1);
}

static void ti_pat_unset_page(struct ti_pat_data *pat, size_t page_id)
{
	unsigned int index = ti_pat_table_index_from_page(page_id);

	dev_dbg(pat->dev, "Disabling PAT index: %zu\n", page_id);

	regmap_fields_write(pat->fields[F_TABLE_ENABLE], index, 0);
}

static struct sg_table *ti_pat_dma_buf_map(struct dma_buf_attachment *attachment,
					   enum dma_data_direction direction)
{
	struct ti_pat_dma_buf_attachment *a = attachment->priv;
	struct ti_pat_buffer *buffer = a->buffer;
	struct ti_pat_data *pat = buffer->pat;
	struct sg_table *table = a->table;
	struct scatterlist *s;
	unsigned int i, s_len;
	size_t page_id;
	int ret;

	mutex_lock(&buffer->lock);
	/* First time mapping we map to parent */
	if (!buffer->map_count) {
		buffer->sgt = dma_buf_map_attachment(buffer->attachment, DMA_BIDIRECTIONAL);
		if (IS_ERR(buffer->sgt)) {
			dev_err(pat->dev, "Unable to map parent DMA-BUF\n");
			return buffer->sgt;
		}

		/* And program PAT area for this set of pages */
		page_id = (size_t)(buffer->offset - pat->window_base) / pat->page_size;
		for_each_sg(buffer->sgt->sgl, s, buffer->sgt->nents, i) {
			if (s->offset) {
				dev_err(pat->dev, "Cannot use offset buffers\n");
				ret = -EINVAL;
				goto unmap;
			}

			if (s->length % pat->page_size) {
				dev_err(pat->dev, "Cannot use buffers not a multiple of page size\n");
				ret = -EINVAL;
				goto unmap;
			}

			for (s_len = 0; s_len < s->length; s_len += pat->page_size)
				ti_pat_set_page(pat, page_id++, s->dma_address + s_len);
		}
	}
	buffer->map_count++;
	mutex_unlock(&buffer->lock);

	/* Map the attached device's table to get DMA addresses */
	if (!dma_map_sg_attrs(attachment->dev, table->sgl, table->nents, direction, DMA_ATTR_SKIP_CPU_SYNC))
		return ERR_PTR(-ENOMEM);

	return table;

unmap:
	dma_buf_unmap_attachment(buffer->attachment, buffer->sgt, DMA_BIDIRECTIONAL);
	mutex_unlock(&buffer->lock);
	return ERR_PTR(ret);
}

static void ti_pat_dma_buf_unmap(struct dma_buf_attachment *attachment,
				 struct sg_table *table,
				 enum dma_data_direction direction)
{
	struct ti_pat_dma_buf_attachment *a = attachment->priv;
	struct ti_pat_buffer *buffer = a->buffer;
	struct ti_pat_data *pat = buffer->pat;

	/* Unmap the attached device's table */
	dma_unmap_sg_attrs(attachment->dev, table->sgl, table->nents, direction, DMA_ATTR_SKIP_CPU_SYNC);

	mutex_lock(&buffer->lock);
	buffer->map_count--;
	/* Last mapping we unmap from parent */
	if (!buffer->map_count) {
		/* Disable PAT pages for this area */
		size_t page_start = (size_t)(buffer->offset - pat->window_base) / pat->page_size;
		size_t page_end = page_start + (buffer->size / pat->page_size);

		for (; page_start < page_end; page_start++)
			ti_pat_unset_page(pat, page_start);

		dma_buf_unmap_attachment(buffer->attachment, buffer->sgt, DMA_BIDIRECTIONAL);
		buffer->sgt = NULL;
	}
	mutex_unlock(&buffer->lock);
}

static void ti_pat_dma_buf_release(struct dma_buf *dmabuf)
{
	struct ti_pat_buffer *buffer = dmabuf->priv;

	if (buffer->attachment && buffer->sgt)
		dma_buf_unmap_attachment(buffer->attachment, buffer->sgt, DMA_BIDIRECTIONAL);
	if (buffer->i_dma_buf && !IS_ERR_OR_NULL(buffer->attachment))
		dma_buf_detach(buffer->i_dma_buf, buffer->attachment);
	if (buffer->i_dma_buf)
		dma_buf_put(buffer->i_dma_buf);

	if (buffer->offset)
		gen_pool_free(buffer->pat->pool, buffer->offset, buffer->size);

	kfree(buffer);
}

static int ti_pat_dma_buf_begin_cpu_access(struct dma_buf *dmabuf, enum dma_data_direction direction)
{
	struct ti_pat_buffer *buffer = dmabuf->priv;

	return dma_buf_begin_cpu_access(buffer->i_dma_buf, direction);
}

static int ti_pat_dma_buf_end_cpu_access(struct dma_buf *dmabuf, enum dma_data_direction direction)
{
	struct ti_pat_buffer *buffer = dmabuf->priv;

	return dma_buf_end_cpu_access(buffer->i_dma_buf, direction);
}

static int ti_pat_dma_buf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct ti_pat_buffer *buffer = dmabuf->priv;

	return dma_buf_mmap(buffer->i_dma_buf, vma, vma->vm_pgoff);
}

static void *ti_pat_dma_buf_vmap(struct dma_buf *dmabuf)
{
	struct ti_pat_buffer *buffer = dmabuf->priv;

	return dma_buf_vmap(buffer->i_dma_buf);
}

static void ti_pat_dma_buf_vunmap(struct dma_buf *dmabuf, void *vaddr)
{
	struct ti_pat_buffer *buffer = dmabuf->priv;

	return dma_buf_vunmap(buffer->i_dma_buf, vaddr);
}

static const struct dma_buf_ops dma_buf_ops = {
	.attach = ti_pat_dma_buf_attach,
	.detach = ti_pat_dma_buf_detach,

	.map_dma_buf = ti_pat_dma_buf_map,
	.unmap_dma_buf = ti_pat_dma_buf_unmap,

	.release = ti_pat_dma_buf_release,

	.begin_cpu_access = ti_pat_dma_buf_begin_cpu_access,
	.end_cpu_access = ti_pat_dma_buf_end_cpu_access,
	.mmap = ti_pat_dma_buf_mmap,
	.vmap = ti_pat_dma_buf_vmap,
	.vunmap = ti_pat_dma_buf_vunmap,
};

int ti_pat_export(struct ti_pat_data *pat,
		  struct dma_buf *i_dma_buf,
		  struct dma_buf **e_dma_buf)
{
	struct ti_pat_buffer *buffer;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	int ret;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	buffer->pat = pat;
	buffer->i_dma_buf = i_dma_buf;
	buffer->size = buffer->i_dma_buf->size;
	mutex_init(&buffer->lock);
	INIT_LIST_HEAD(&buffer->attachments);
	buffer->map_count = 0;

	/* Reserve PAT space */
	buffer->offset = gen_pool_alloc(buffer->pat->pool, buffer->size);
	if (!buffer->offset) {
		ret = -ENOMEM;
		goto free_buffer;
	}

	exp_info.ops = &dma_buf_ops;
	exp_info.size = buffer->size;
	exp_info.flags = O_RDWR;
	exp_info.priv = buffer;

	*e_dma_buf = dma_buf_export(&exp_info);
	if (IS_ERR(*e_dma_buf)) {
		ret = PTR_ERR(*e_dma_buf);
		goto free_pool;
	}

	return 0;

free_pool:
	gen_pool_free(buffer->pat->pool, buffer->offset, buffer->size);
free_buffer:
	kfree(buffer);
	return ret;
}
EXPORT_SYMBOL_GPL(ti_pat_export);

static long ti_pat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ti_pat_data *pat = container_of(file->private_data, struct ti_pat_data, mdev);

	switch (cmd) {
	case TI_PAT_IOC_EXPORT:
	{
		struct ti_pat_export_data export;
		struct dma_buf *i_dma_buf;
		struct dma_buf *e_dma_buf;
		int ret;

		if (_IOC_SIZE(cmd) > sizeof(export))
			return -EINVAL;

		if (copy_from_user(&export, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;

		i_dma_buf = dma_buf_get(export.fd);
		if (IS_ERR(i_dma_buf))
			return PTR_ERR(i_dma_buf);

		ret = ti_pat_export(pat, i_dma_buf, &e_dma_buf);
		if (ret) {
			dma_buf_put(i_dma_buf);
			return ret;
		}

		export.fd = dma_buf_fd(e_dma_buf, O_CLOEXEC);
		if (export.fd < 0) {
			dma_buf_put(e_dma_buf);
			dma_buf_put(i_dma_buf);
			return export.fd;
		}

		if (copy_to_user((void __user *)arg, &export, _IOC_SIZE(cmd)))
			return -EFAULT;

		break;
	}
	default:
		return -ENOTTY;
	}

	return 0;
}

static const struct file_operations ti_pat_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = ti_pat_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= ti_pat_ioctl,
#endif
};

static int ti_pat_probe(struct platform_device *pdev)
{
	struct ti_pat_data *pat;
	struct resource *res;
	void __iomem *base;
	struct regmap *mmrs_map;
	struct regmap *table_map;
	unsigned int revision_major;
	unsigned int revision_minor;
	resource_size_t size;
	size_t page_size;
	int i, ret;

	pat = devm_kzalloc(&pdev->dev, sizeof(*pat), GFP_KERNEL);
	if (!pat)
		return -ENOMEM;
	platform_set_drvdata(pdev, pat);
	pat->dev = &pdev->dev;

	/* Set DMA mask to 64 bits */
	ret = dma_set_mask_and_coherent(pat->dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(pat->dev, "Unable to set coherent mask to 64");
		return ret;
	}

	/* MMRS */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mmrs");
	if (!res) {
		dev_err(pat->dev, "Unable to find MMRS IO resource\n");
		return -ENOENT;
	}
	base = devm_ioremap_resource(pat->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	mmrs_map = devm_regmap_init_mmio(pat->dev, base, &ti_pat_regmap_config);
	if (IS_ERR(mmrs_map)) {
		dev_err(pat->dev, "Unable to allocate MMRS register map\n");
		return PTR_ERR(mmrs_map);
	}

	for (i = 0; i < F_MMRS_FIELDS; i++) {
		pat->fields[i] = devm_regmap_field_alloc(pat->dev, mmrs_map, ti_pat_reg_fields[i]);
		if (IS_ERR(pat->fields[i])) {
			dev_err(pat->dev, "Unable to allocate Regmap fields\n");
			return PTR_ERR(pat->fields[i]);
		}
	}

	ret = regmap_read(mmrs_map, TI_PAT_MMRS_CONFIG, &pat->page_count);
	if (ret) {
		dev_err(pat->dev, "Unable to read device page count\n");
		return ret;
	}

	ret = regmap_field_read(pat->fields[F_PID_MAJOR], &revision_major);
	if (ret) {
		dev_err(pat->dev, "Unable to read device major revision\n");
		return ret;
	}

	ret = regmap_field_read(pat->fields[F_PID_MINOR], &revision_minor);
	if (ret) {
		dev_err(pat->dev, "Unable to read device minor revision\n");
		return ret;
	}

	dev_info(pat->dev, "Found PAT Rev %d.%d with %d pages\n", revision_major, revision_minor, pat->page_count);

	/* TABLE */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "table");
	if (!res) {
		dev_err(pat->dev, "Unable to find TABLE IO resource\n");
		return -ENOENT;
	}
	base = devm_ioremap_resource(pat->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	/* 256 pages per 4KB of table space */
	size = resource_size(res);
	if (size != (pat->page_count << 4))
		dev_warn(pat->dev, "TABLE region size (%pa) does not match reported page count\n", &size);

	table_map = devm_regmap_init_mmio(pat->dev, base, &ti_pat_regmap_config);
	if (IS_ERR(table_map)) {
		dev_err(pat->dev, "Unable to allocate TABLE register map\n");
		return PTR_ERR(table_map);
	}

	for (i = F_MMRS_FIELDS + 1; i < F_MAX_FIELDS; i++) {
		ti_pat_reg_fields[i].id_size = ti_pat_table_index_from_page(pat->page_count);
		ti_pat_reg_fields[i].id_offset = 8; /* 8 bytes per entry */
		pat->fields[i] = devm_regmap_field_alloc(pat->dev, table_map, ti_pat_reg_fields[i]);
		if (IS_ERR(pat->fields[i])) {
			dev_err(pat->dev, "Unable to allocate Regmap fields\n");
			return PTR_ERR(pat->fields[i]);
		}
	}

	/* WINDOW */
	ret = device_property_read_u64(pat->dev, "ti,pat-window-base", &pat->window_base);
	if (ret) {
		dev_err(pat->dev, "Unable to find ti,pat-window-base\n");
		return -ENOENT;
	}

	ret = device_property_read_u64(pat->dev, "ti,pat-window-size", &size);
	if (ret) {
		dev_err(pat->dev, "Unable to find ti,pat-window-size\n");
		return -ENOENT;
	}

	pat->page_size = PAGE_SIZE;
	for (page_size = 0; page_size < ARRAY_SIZE(ti_pat_page_sizes); page_size++)
		if (ti_pat_page_sizes[page_size] == pat->page_size)
			break;
	if (page_size == ARRAY_SIZE(ti_pat_page_sizes)) {
		dev_err(pat->dev, "Unsupported PAGE_SIZE (%d)\n", pat->page_size);
		return -EINVAL;
	}
	regmap_field_write(pat->fields[F_CONTROL_PAGE_SIZE], page_size);

	/* Enable this PAT module */
	regmap_field_write(pat->fields[F_CONTROL_EN], 1);

	pat->pool = gen_pool_create(PAGE_SHIFT, -1);
	if (!pat->pool)
		return -ENOMEM;
	gen_pool_add(pat->pool, pat->window_base, size, -1);

	pat->mdev.minor = MISC_DYNAMIC_MINOR;
	pat->mdev.name = pdev->name;
	pat->mdev.fops = &ti_pat_fops;
	pat->mdev.parent = NULL;
	ret = misc_register(&pat->mdev);
	if (ret) {
		dev_err(pat->dev, "Unable to register misc device\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id ti_pat_of_match[] = {
	{ .compatible = "ti,j721e-pat", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ti_pat_of_match);

static struct platform_driver ti_pat_driver = {
	.probe = ti_pat_probe,
	.driver = {
		.name = "ti-pat",
		.of_match_table = ti_pat_of_match,
	},
};
module_platform_driver(ti_pat_driver);

MODULE_AUTHOR("Andrew Davis <afd@ti.com>");
MODULE_DESCRIPTION("TI PAT mapped DMA-BUF memory exporter");
MODULE_LICENSE("GPL v2");
