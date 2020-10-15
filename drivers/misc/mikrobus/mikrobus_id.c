// SPDX-License-Identifier: GPL-2.0-only
/*
 * mikrobus_id.c - w1 mikroBUS ID family EEPROM driver
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>

#include <linux/w1.h>
#include <linux/nvmem-provider.h>

#include "mikrobus_core.h"

#define W1_EEPROM_MIKROBUS_ID	0xAC

#define W1_MIKROBUS_ID_EEPROM_SIZE	1536
#define W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE	32
#define W1_MIKROBUS_ID_EEPROM_VERIFY_SCRATCH_SIZE	35
#define W1_MIKROBUS_ID_READ_EEPROM	0xF0
#define W1_MIKROBUS_ID_EEPROM_READ_RETRIES	10
#define W1_MIKROBUS_ID_EEPROM_WRITE_RETRIES	5
#define W1_MIKROBUS_ID_EEPROM_WRITE_SCRATCH	0x0F
#define W1_MIKROBUS_ID_EEPROM_READ_SCRATCH	0xAA
#define W1_MIKROBUS_ID_EEPROM_COPY_SCRATCH	0x55
#define W1_MIKROBUS_ID_EEPROM_COPY_SCRATCH_ES	0x40
#define W1_MIKROBUS_ID_EEPROM_TPROG_MS		20

static int w1_mikrobus_id_readblock(struct w1_slave *sl, int off, int count, char *buf)
{
	u8 wrbuf[3];
	u8 *cmp;
	int tries = W1_MIKROBUS_ID_EEPROM_READ_RETRIES;
	
	do {
		wrbuf[0] = W1_MIKROBUS_ID_READ_EEPROM;
		wrbuf[1] = count >> 8;
		wrbuf[2] = count & 0xFF;

		if (w1_reset_select_slave(sl))
				return -1;
		w1_write_block(sl->master, wrbuf, 3);
		w1_read_block(sl->master, buf, count);

		if (w1_reset_select_slave(sl))
				return -1;
		cmp = kzalloc(count, GFP_KERNEL);
		if (!cmp)
			return -ENOMEM;
		w1_write_block(sl->master, wrbuf, 3);
		w1_read_block(sl->master, cmp, count);
		if (!memcmp(cmp, buf, count)){
			kfree(cmp);
			return 0;
		}
	} while (--tries);

	dev_err(&sl->dev, "proof reading failed %d times\n",
			W1_MIKROBUS_ID_EEPROM_READ_RETRIES);
	kfree(cmp);
	return -EIO;
}

static int w1_mikrobus_id_movescratch(struct w1_slave *sl, int addr, char *buf)
{
	u8 wrbuf[4];
	u8 scratchpad_verify[W1_MIKROBUS_ID_EEPROM_VERIFY_SCRATCH_SIZE];
	u8 write_scratchpad_crc[2];
	int verify_status;
	int tries;

	wrbuf[0] = W1_MIKROBUS_ID_EEPROM_WRITE_SCRATCH;
	wrbuf[1] = addr >> 8;
	wrbuf[2] = addr & 0xFF;

	tries = W1_MIKROBUS_ID_EEPROM_WRITE_RETRIES;
	do {
		if (w1_reset_select_slave(sl))
			return -1;
		w1_write_block(sl->master, wrbuf, 3);
		w1_write_block(sl->master, buf, W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE);
		udelay(100); /* delay for CRC calculation at slave */
		w1_read_block(sl->master, write_scratchpad_crc, 2);
		if (w1_reset_select_slave(sl))
			return -1;
		w1_write_8(sl->master, W1_MIKROBUS_ID_EEPROM_READ_SCRATCH);
		w1_read_block(sl->master, scratchpad_verify, W1_MIKROBUS_ID_EEPROM_VERIFY_SCRATCH_SIZE);
		verify_status = memcmp(buf, scratchpad_verify + 3, W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE);
	} while(verify_status && --tries);

	if(!tries && verify_status){
		dev_err(&sl->dev, "verify scratchpad failed %d times\n",
			W1_MIKROBUS_ID_EEPROM_WRITE_RETRIES);
		return -EIO;
	}
		
	wrbuf[0] = W1_MIKROBUS_ID_EEPROM_COPY_SCRATCH;
	wrbuf[1] = addr >> 8;
	wrbuf[2] = addr & 0xFF;
	wrbuf[3] = W1_MIKROBUS_ID_EEPROM_COPY_SCRATCH_ES;
	if (w1_reset_select_slave(sl))
			return -1;
	w1_write_block(sl->master, wrbuf, 4);
	msleep(W1_MIKROBUS_ID_EEPROM_TPROG_MS);
	return 0;
}

static int w1_mikrobus_id_writeblock(struct w1_slave *sl, int off, int count, char *buf)
{
	u16 wraddr = 0;
	u16 len = count - (count % W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE);
	u8 scratchpad_write[W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE];

	while(len > 0) {
		w1_mikrobus_id_movescratch(sl, wraddr, buf + wraddr);
		wraddr += W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE;
		len -= W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE;
	}

	if(count % W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE){
		memcpy(scratchpad_write, buf + wraddr, count % W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE);
		w1_mikrobus_id_movescratch(sl, wraddr, scratchpad_write);
	}

	return 0;
}

static int w1_mikrobus_id_nvmem_read(void *priv, unsigned int off, void *buf, size_t count)
{
	struct w1_slave *sl = priv;
	int ret;

	/* mikroBUS ID EEPROM does not support reading from offsets */
	if (off)
		return -EINVAL;

	if (count > W1_MIKROBUS_ID_EEPROM_SIZE)
		return -EINVAL;

	mutex_lock(&sl->master->bus_mutex);
	ret = w1_mikrobus_id_readblock(sl, off, count, buf);
	mutex_unlock(&sl->master->bus_mutex);
	
	return ret;
}

static int w1_mikrobus_id_nvmem_write(void *priv, unsigned int off, void *buf, size_t count)
{
	struct w1_slave *sl = priv;
	int ret;

	if ((off + count) > W1_MIKROBUS_ID_EEPROM_SIZE)
		return -EINVAL;

	mutex_lock(&sl->master->bus_mutex);
	ret = w1_mikrobus_id_writeblock(sl, off, count, buf);
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
		.reg_write = w1_mikrobus_id_nvmem_write,
		.type = NVMEM_TYPE_EEPROM,
		.read_only = false,
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

static const struct w1_family_ops w1_family_mikrobus_id_fops = {
	.add_slave		= w1_mikrobus_id_add_slave,
};

static struct w1_family w1_family_mikrobus_id = {
	.fid = W1_EEPROM_MIKROBUS_ID,
	.fops = &w1_family_mikrobus_id_fops,
};
module_w1_family(w1_family_mikrobus_id);

MODULE_AUTHOR("Vaishnav M A <vaishnav@beagleboard.org>");
MODULE_DESCRIPTION("w1 family ac driver for mikroBUS ID EEPROM");
MODULE_LICENSE("GPL");
MODULE_ALIAS("w1-family-" __stringify(W1_EEPROM_MIKROBUS_ID));