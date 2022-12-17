// SPDX-License-Identifier: GPL-2.0-only
/*
 * mikrobus_id.c - w1 mikroBUS ID family EEPROM driver
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/crc16.h>
#include <linux/w1.h>
#include <linux/nvmem-provider.h>

#include <linux/mikrobus.h>

#define W1_EEPROM_MIKROBUS_ID				0xCC
#define W1_MIKROBUS_ID_EEPROM_SIZE	 		0x0200
#define W1_MIKROBUS_ID_EEPROM_PAGE_SIZE		32
#define W1_MIKROBUS_ID_READ_EEPROM 			0x69
#define W1_MIKROBUS_ID_WRITE_EEPROM 		0x96
#define W1_MIKROBUS_ID_RELEASE_EEPROM 		0xAA
#define W1_MIKROBUS_ID_EEPROM_READ_RETRIES	10

#define W1_MIKROBUS_EEPROM_MANIFEST_START_PAGE	1

static ssize_t mikrobus_manifest_store(struct device *device,
			    				 struct device_attribute *attr,
			    				 const char *buf, size_t count)
{
	u8 status = 0;
	u16 pos = 0, crc, crc_read;
	int cnt;

	u8  write_request[] = { W1_MIKROBUS_ID_WRITE_EEPROM,
	 						W1_MIKROBUS_EEPROM_MANIFEST_START_PAGE};
	u8  release_command = W1_MIKROBUS_ID_RELEASE_EEPROM;

	struct w1_slave *sl = dev_to_w1_slave(device);

	if (count > W1_MIKROBUS_ID_EEPROM_SIZE)
		return -ENOMEM;

	mutex_lock(&sl->master->bus_mutex);

	pr_info("mikrobus_id: writing manifest size = %lu bytes", count);
	while (pos < count)
	{		
		if (w1_reset_select_slave(sl))
			break;
		
		w1_write_block(sl->master, write_request, sizeof(write_request));
		crc = crc16(0, write_request, sizeof(write_request)) ^ 0xFFFF;
		w1_read_block(sl->master, (u8*)&crc_read, sizeof(crc_read));

		if (crc != crc_read)
			break;

		for (cnt = 0; cnt < W1_MIKROBUS_ID_EEPROM_PAGE_SIZE; cnt++)
		{
			w1_write_8(sl->master, (u8)buf[cnt]);
		}
		crc = crc16(0, buf, W1_MIKROBUS_ID_EEPROM_PAGE_SIZE) ^ 0xFFFF;
		msleep(1);
		w1_read_block(sl->master, (u8*)&crc_read, sizeof(crc_read));

		if (crc != crc_read)
			break;

		w1_write_8(sl->master, release_command);

		msleep(10);

		status = w1_read_8(sl->master);
		w1_read_block(sl->master, (u8*)&crc_read, sizeof(crc_read));
		crc = crc16(0, (u8*)&release_command, sizeof(release_command)) ^ 0xFFFF;

		if (status != W1_MIKROBUS_ID_RELEASE_EEPROM)
			break;

		if (crc != crc_read)
			break;	
		
		buf += W1_MIKROBUS_ID_EEPROM_PAGE_SIZE;
		pos += W1_MIKROBUS_ID_EEPROM_PAGE_SIZE;
		write_request[1]++;
	}
	pr_info("mikrobus_id: manifest written bytes: %d", pos);
	mutex_unlock(&sl->master->bus_mutex);
	
	return count > pos ? count : pos;
}
static DEVICE_ATTR_WO(mikrobus_manifest);

static struct attribute *w1_mikrobus_attrs[] = {
	&dev_attr_mikrobus_manifest.attr,
	NULL
};

ATTRIBUTE_GROUPS(w1_mikrobus);

static int w1_mikrobus_id_readpage(struct w1_slave *sl, int pageaddr, char *buf)
{
	u8 crc_rdbuf[2];

	if (w1_reset_select_slave(sl))
				return -1;
	w1_write_8(sl->master, W1_MIKROBUS_ID_READ_EEPROM);
	w1_write_8(sl->master, pageaddr);
	w1_read_block(sl->master, crc_rdbuf, 2);
	w1_write_8(sl->master, W1_MIKROBUS_ID_RELEASE_EEPROM);
	msleep(10);
	w1_read_block(sl->master, crc_rdbuf, 1);
	w1_read_block(sl->master, buf, W1_MIKROBUS_ID_EEPROM_PAGE_SIZE);
	w1_read_block(sl->master, crc_rdbuf, 2);
	return 0;
}

static int w1_mikrobus_id_readbuf(struct w1_slave *sl, int count, int off, char *buf)
{
	u8 pageaddr = off/W1_MIKROBUS_ID_EEPROM_PAGE_SIZE;
	int iter, index, ret;
	int	len = count - (count % W1_MIKROBUS_ID_EEPROM_PAGE_SIZE);
	u8 temp_rdbuf[W1_MIKROBUS_ID_EEPROM_PAGE_SIZE];

	while(len > 0) {			
			ret = w1_mikrobus_id_readpage(sl, pageaddr, buf + (W1_MIKROBUS_ID_EEPROM_PAGE_SIZE*pageaddr - off));
			pageaddr += 1;
			len -= W1_MIKROBUS_ID_EEPROM_PAGE_SIZE;
	}

	if(count % W1_MIKROBUS_ID_EEPROM_PAGE_SIZE){
			ret = w1_mikrobus_id_readpage(sl, pageaddr, temp_rdbuf);
			for(iter = W1_MIKROBUS_ID_EEPROM_PAGE_SIZE*pageaddr - off, index=0; iter < count; iter++, index++)
				buf[iter] = temp_rdbuf[index];
	}
	return ret;
}

static int w1_mikrobus_id_readblock(struct w1_slave *sl, int off, int count, char *buf)
{
	u8 *cmp;
	int tries = W1_MIKROBUS_ID_EEPROM_READ_RETRIES;

	do {	
		w1_mikrobus_id_readbuf(sl, count, off, buf);
		cmp = kzalloc(count, GFP_KERNEL);
		if (!cmp)
			return -ENOMEM;		
		w1_mikrobus_id_readbuf(sl, count, off, cmp);
		if (!memcmp(cmp, buf, count)){
			kfree(cmp);
			return 0;
		}
	} while (--tries);

	kfree(cmp);
	return -EINVAL;
}

static int w1_mikrobus_id_nvmem_read(void *priv, unsigned int off, void *buf, size_t count)
{
	struct w1_slave *sl = priv;
	int ret;

	mutex_lock(&sl->master->bus_mutex);
	ret = w1_mikrobus_id_readblock(sl, off, count, buf);
	mutex_unlock(&sl->master->bus_mutex);
	
	return ret;
}

static int w1_mikrobus_id_add_slave(struct w1_slave *sl)
{
	struct nvmem_device *nvmem;
	struct mikrobus_port *port;
	struct nvmem_config nvmem_cfg = {
		.dev = &sl->dev,
		.reg_read = w1_mikrobus_id_nvmem_read,
		.type = NVMEM_TYPE_EEPROM,
		.read_only = true,
		.word_size = 1,
		.stride = 1,
		.size = W1_MIKROBUS_ID_EEPROM_SIZE,
		.priv = sl,
	};

	port = mikrobus_find_port_by_w1_master(sl->master);
	if(!port)
		return -ENODEV;

	nvmem_cfg.name = port->name;
	nvmem = devm_nvmem_register(&sl->dev, &nvmem_cfg);
	port->eeprom = nvmem;
	mikrobus_port_scan_eeprom(port);

	return PTR_ERR_OR_ZERO(nvmem);
}

static struct w1_family_ops w1_family_mikrobus_id_fops = {
	.add_slave		= w1_mikrobus_id_add_slave,
	.groups 	= w1_mikrobus_groups
};

static struct w1_family w1_family_mikrobus_id = {
	.fid = W1_EEPROM_MIKROBUS_ID,
	.fops = &w1_family_mikrobus_id_fops,
};

static int __init w1_mikrobusid_init(void)
{
	int err;

	err = w1_register_family(&w1_family_mikrobus_id);

	return err;
}

static void __exit w1_mikrobusid_exit(void)
{
	w1_unregister_family(&w1_family_mikrobus_id);
}

module_init(w1_mikrobusid_init);
module_exit(w1_mikrobusid_exit);

MODULE_AUTHOR("Vaishnav M A <vaishnav@beagleboard.org>");
MODULE_DESCRIPTION("w1 family CC driver for mikroBUS ID EEPROM");
MODULE_LICENSE("GPL");
MODULE_ALIAS("w1-family-" __stringify(W1_EEPROM_MIKROBUS_ID));
