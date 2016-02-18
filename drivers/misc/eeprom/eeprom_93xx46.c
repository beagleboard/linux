/*
 * Driver for 93xx46 EEPROMs
 *
 * (C) 2011 DENX Software Engineering, Anatolij Gustschin <agust@denx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/nvmem-provider.h>
#include <linux/regmap.h>
#include <linux/eeprom_93xx46.h>

#define OP_START	0x4
#define OP_WRITE	(OP_START | 0x1)
#define OP_READ		(OP_START | 0x2)
#define ADDR_EWDS	0x00
#define ADDR_ERAL	0x20
#define ADDR_EWEN	0x30

struct eeprom_93xx46_dev {
	struct spi_device *spi;
	struct eeprom_93xx46_platform_data *pdata;
	struct mutex lock;
	struct regmap_config regmap_config;
	struct nvmem_config nvmem_config;
	struct nvmem_device *nvmem;
	int addrlen;
	int size;
};

static ssize_t
eeprom_93xx46_read(struct eeprom_93xx46_dev *edev, char *buf,
		   unsigned off, size_t count)
{
	struct spi_message m;
	struct spi_transfer t[2];
	int bits, ret;
	u16 cmd_addr;

	if (unlikely(off >= edev->size))
		return 0;
	if ((off + count) > edev->size)
		count = edev->size - off;
	if (unlikely(!count))
		return count;

	cmd_addr = OP_READ << edev->addrlen;

	if (edev->addrlen == 7) {
		cmd_addr |= off & 0x7f;
		bits = 10;
	} else {
		cmd_addr |= off & 0x3f;
		bits = 9;
	}

	dev_dbg(&edev->spi->dev, "read cmd 0x%x, %d Hz\n",
		cmd_addr, edev->spi->max_speed_hz);

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	t[0].tx_buf = (char *)&cmd_addr;
	t[0].len = 2;
	t[0].bits_per_word = bits;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = count;
	t[1].bits_per_word = 8;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&edev->lock);

	if (edev->pdata->prepare)
		edev->pdata->prepare(edev);

	ret = spi_sync(edev->spi, &m);
	/* have to wait at least Tcsl ns */
	ndelay(250);
	if (ret) {
		dev_err(&edev->spi->dev, "read %zu bytes at %d: err. %d\n",
			count, (int)off, ret);
	}

	if (edev->pdata->finish)
		edev->pdata->finish(edev);

	mutex_unlock(&edev->lock);
	return ret ? : count;
}

static int eeprom_93xx46_ew(struct eeprom_93xx46_dev *edev, int is_on)
{
	struct spi_message m;
	struct spi_transfer t;
	int bits, ret;
	u16 cmd_addr;

	cmd_addr = OP_START << edev->addrlen;
	if (edev->addrlen == 7) {
		cmd_addr |= (is_on ? ADDR_EWEN : ADDR_EWDS) << 1;
		bits = 10;
	} else {
		cmd_addr |= (is_on ? ADDR_EWEN : ADDR_EWDS);
		bits = 9;
	}

	dev_dbg(&edev->spi->dev, "ew cmd 0x%04x\n", cmd_addr);

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));

	t.tx_buf = &cmd_addr;
	t.len = 2;
	t.bits_per_word = bits;
	spi_message_add_tail(&t, &m);

	mutex_lock(&edev->lock);

	if (edev->pdata->prepare)
		edev->pdata->prepare(edev);

	ret = spi_sync(edev->spi, &m);
	/* have to wait at least Tcsl ns */
	ndelay(250);
	if (ret)
		dev_err(&edev->spi->dev, "erase/write %sable error %d\n",
			is_on ? "en" : "dis", ret);

	if (edev->pdata->finish)
		edev->pdata->finish(edev);

	mutex_unlock(&edev->lock);
	return ret;
}

static ssize_t
eeprom_93xx46_write_word(struct eeprom_93xx46_dev *edev,
			 const char *buf, unsigned off)
{
	struct spi_message m;
	struct spi_transfer t[2];
	int bits, data_len, ret;
	u16 cmd_addr;

	cmd_addr = OP_WRITE << edev->addrlen;

	if (edev->addrlen == 7) {
		cmd_addr |= off & 0x7f;
		bits = 10;
		data_len = 1;
	} else {
		cmd_addr |= off & 0x3f;
		bits = 9;
		data_len = 2;
	}

	dev_dbg(&edev->spi->dev, "write cmd 0x%x\n", cmd_addr);

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	t[0].tx_buf = (char *)&cmd_addr;
	t[0].len = 2;
	t[0].bits_per_word = bits;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	t[1].len = data_len;
	t[1].bits_per_word = 8;
	spi_message_add_tail(&t[1], &m);

	ret = spi_sync(edev->spi, &m);
	/* have to wait program cycle time Twc ms */
	mdelay(6);
	return ret;
}

static ssize_t
eeprom_93xx46_write(struct eeprom_93xx46_dev *edev, const char *buf,
		    loff_t off, size_t count)
{
	int i, ret, step = 1;

	if (unlikely(off >= edev->size))
		return -EFBIG;
	if ((off + count) > edev->size)
		count = edev->size - off;
	if (unlikely(!count))
		return count;

	/* only write even number of bytes on 16-bit devices */
	if (edev->addrlen == 6) {
		step = 2;
		count &= ~1;
	}

	/* erase/write enable */
	ret = eeprom_93xx46_ew(edev, 1);
	if (ret)
		return ret;

	mutex_lock(&edev->lock);

	if (edev->pdata->prepare)
		edev->pdata->prepare(edev);

	for (i = 0; i < count; i += step) {
		ret = eeprom_93xx46_write_word(edev, &buf[i], off + i);
		if (ret) {
			dev_err(&edev->spi->dev, "write failed at %d: %d\n",
				(int)off + i, ret);
			break;
		}
	}

	if (edev->pdata->finish)
		edev->pdata->finish(edev);

	mutex_unlock(&edev->lock);

	/* erase/write disable */
	eeprom_93xx46_ew(edev, 0);
	return ret ? : count;
}

/*
 * Provide a regmap interface, which is registered with the NVMEM
 * framework
*/
static int eeprom_93xx46_regmap_read(void *context, const void *reg,
				     size_t reg_size, void *val,
				     size_t val_size)
{
	struct eeprom_93xx46_dev *eeprom_93xx46 = context;
	off_t offset = *(u32 *)reg;
	int err;

	err = eeprom_93xx46_read(eeprom_93xx46, val, offset, val_size);
	if (err)
		return err;
	return 0;
}

static int eeprom_93xx46_regmap_write(void *context, const void *data,
				      size_t count)
{
	struct eeprom_93xx46_dev *eeprom_93xx46 = context;
	const char *buf;
	u32 offset;
	size_t len;
	int err;

	memcpy(&offset, data, sizeof(offset));
	buf = (const char *)data + sizeof(offset);
	len = count - sizeof(offset);

	err = eeprom_93xx46_write(eeprom_93xx46, buf, offset, len);
	if (err)
		return err;
	return 0;
}

static const struct regmap_bus eeprom_93xx46_regmap_bus = {
	.read = eeprom_93xx46_regmap_read,
	.write = eeprom_93xx46_regmap_write,
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
};

static int eeprom_93xx46_eral(struct eeprom_93xx46_dev *edev)
{
	struct eeprom_93xx46_platform_data *pd = edev->pdata;
	struct spi_message m;
	struct spi_transfer t;
	int bits, ret;
	u16 cmd_addr;

	cmd_addr = OP_START << edev->addrlen;
	if (edev->addrlen == 7) {
		cmd_addr |= ADDR_ERAL << 1;
		bits = 10;
	} else {
		cmd_addr |= ADDR_ERAL;
		bits = 9;
	}

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));

	t.tx_buf = &cmd_addr;
	t.len = 2;
	t.bits_per_word = bits;
	spi_message_add_tail(&t, &m);

	mutex_lock(&edev->lock);

	if (edev->pdata->prepare)
		edev->pdata->prepare(edev);

	ret = spi_sync(edev->spi, &m);
	if (ret)
		dev_err(&edev->spi->dev, "erase error %d\n", ret);
	/* have to wait erase cycle time Tec ms */
	mdelay(6);

	if (pd->finish)
		pd->finish(edev);

	mutex_unlock(&edev->lock);
	return ret;
}

static ssize_t eeprom_93xx46_store_erase(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct eeprom_93xx46_dev *edev = dev_get_drvdata(dev);
	int erase = 0, ret;

	sscanf(buf, "%d", &erase);
	if (erase) {
		ret = eeprom_93xx46_ew(edev, 1);
		if (ret)
			return ret;
		ret = eeprom_93xx46_eral(edev);
		if (ret)
			return ret;
		ret = eeprom_93xx46_ew(edev, 0);
		if (ret)
			return ret;
	}
	return count;
}
static DEVICE_ATTR(erase, S_IWUSR, NULL, eeprom_93xx46_store_erase);

static int eeprom_93xx46_probe(struct spi_device *spi)
{
	struct eeprom_93xx46_platform_data *pd;
	struct eeprom_93xx46_dev *edev;
	struct regmap *regmap;
	int err;

	pd = spi->dev.platform_data;
	if (!pd) {
		dev_err(&spi->dev, "missing platform data\n");
		return -ENODEV;
	}

	edev = kzalloc(sizeof(*edev), GFP_KERNEL);
	if (!edev)
		return -ENOMEM;

	if (pd->flags & EE_ADDR8)
		edev->addrlen = 7;
	else if (pd->flags & EE_ADDR16)
		edev->addrlen = 6;
	else {
		dev_err(&spi->dev, "unspecified address type\n");
		err = -EINVAL;
		goto fail;
	}

	mutex_init(&edev->lock);

	edev->spi = spi_dev_get(spi);
	edev->pdata = pd;

	edev->size = 128;

	edev->regmap_config.reg_bits = 32;
	edev->regmap_config.val_bits = 8;
	edev->regmap_config.reg_stride = 1;
	edev->regmap_config.max_register = edev->size - 1;

	regmap = devm_regmap_init(&spi->dev, &eeprom_93xx46_regmap_bus, edev,
				  &edev->regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "regmap init failed\n");
		err = PTR_ERR(regmap);
		goto fail;
	}

	edev->nvmem_config.name = dev_name(&spi->dev);
	edev->nvmem_config.dev = &spi->dev;
	edev->nvmem_config.read_only = pd->flags & EE_READONLY;
	edev->nvmem_config.root_only = true;
	edev->nvmem_config.owner = THIS_MODULE;
	edev->nvmem_config.compat = true;
	edev->nvmem_config.base_dev = &spi->dev;

	edev->nvmem = nvmem_register(&edev->nvmem_config);
	if (IS_ERR(edev->nvmem)) {
		err = PTR_ERR(edev->nvmem);
		goto fail;
	}

	dev_info(&spi->dev, "%d-bit eeprom %s\n",
		(pd->flags & EE_ADDR8) ? 8 : 16,
		(pd->flags & EE_READONLY) ? "(readonly)" : "");

	if (!(pd->flags & EE_READONLY)) {
		if (device_create_file(&spi->dev, &dev_attr_erase))
			dev_err(&spi->dev, "can't create erase interface\n");
	}

	spi_set_drvdata(spi, edev);
	return 0;
fail:
	kfree(edev);
	return err;
}

static int eeprom_93xx46_remove(struct spi_device *spi)
{
	struct eeprom_93xx46_dev *edev = spi_get_drvdata(spi);

	nvmem_unregister(edev->nvmem);

	if (!(edev->pdata->flags & EE_READONLY))
		device_remove_file(&spi->dev, &dev_attr_erase);

	kfree(edev);
	return 0;
}

static struct spi_driver eeprom_93xx46_driver = {
	.driver = {
		.name	= "93xx46",
	},
	.probe		= eeprom_93xx46_probe,
	.remove		= eeprom_93xx46_remove,
};

module_spi_driver(eeprom_93xx46_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for 93xx46 EEPROMs");
MODULE_AUTHOR("Anatolij Gustschin <agust@denx.de>");
MODULE_ALIAS("spi:93xx46");
