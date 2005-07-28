/*
 *   Texas Instrumens TLV320AIC23 audio codec's i2c interface.
 *   
 *   Copyright (c) by Kai Svahn <kai.svahn@nokia.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/arch/aic23.h>

#define TLV320AIC23_VERSION	"0.1"
#define TLV320AIC23_DATE	"12-Aug-2004"

/* I2C Addresses to scan */
static unsigned short normal_i2c[] = { TLV320AIC23ID1, TLV320AIC23ID2, I2C_CLIENT_END };

/* This makes all addr_data:s */
I2C_CLIENT_INSMOD;

static struct i2c_driver tlv320aic23_driver; 
static struct i2c_client *new_client;
//static struct i2c_client *client;

static int _tlv320aic23_write_value(struct i2c_client *client, u8 reg, u16 value)
{
	u8 val, wreg;
	
	/* TLV320AIC23 has 7 bit address and 9 bits of data
	 * so we need to switch one data bit into reg and rest
	 * of data into val
	 */
	
	wreg = (reg << 1);
	val = (0x01 & (value >> 8));
	wreg = (wreg | val);
	val = (0x00ff & value);
	
	return i2c_smbus_write_byte_data(client, wreg, val);
}

int tlv320aic23_write_value(u8 reg, u16 value)
{
	static struct i2c_client *client;
	client = new_client;
	_tlv320aic23_write_value(client, reg, value);
	
	return 0;
}

static int tlv320aic23_detect_client(struct i2c_adapter *adapter, int address, 
				     int kind)
{
	int err = 0;
      	const char *client_name = "TLV320AIC23 Audio Codec";
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA | 
				     I2C_FUNC_SMBUS_WRITE_BYTE)) {
		printk(KERN_WARNING "%s functinality check failed\n", client_name);
		return err;
	}
	
	if (!(new_client = kmalloc(sizeof(struct i2c_client),
				   GFP_KERNEL))) {
		err = -ENOMEM;
		printk(KERN_WARNING "Couldn't allocate memory for %s\n", client_name);
		return err;
	}
	
	memset(new_client, 0x00, sizeof(struct i2c_client));
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &tlv320aic23_driver;
	new_client->flags = 0;
	strlcpy(new_client->name, client_name, I2C_NAME_SIZE);
       	
	if ((err = i2c_attach_client(new_client))) {
	       	printk(KERN_WARNING "Couldn't attach %s\n", client_name);
		kfree(new_client);
		return err;
	}
	
	return 0;
}
	
static int tlv320aic23_detach_client(struct i2c_client *client)
{
	int err;
	
	if ((err = i2c_detach_client(client))) {
		printk("tlv320aic23.o: Client deregistration failed, client not detached.\n");
		return err;
	}
	
	kfree(client);
	return 0;
}

static int tlv320aic23_attach_adapter(struct i2c_adapter *adapter)
{
	int res;
	
	res = i2c_probe(adapter, &addr_data, &tlv320aic23_detect_client);
	return res;
}

/*-----------------------------------------------------------------------*/

static struct i2c_driver tlv320aic23_driver = {
	.owner          = THIS_MODULE, 
        .name           = "OMAP+TLV320AIC23 codec",
        .id		= I2C_DRIVERID_EXP0,           /* Experimental ID */
        .flags		= I2C_DF_NOTIFY,
        .attach_adapter	= tlv320aic23_attach_adapter,
        .detach_client	= tlv320aic23_detach_client,
};

/*
 *  INIT part
 */

static int __init tlv320aic23_init(void)
{
	int res;
	struct i2c_client *client = client;
	
	if ((res = i2c_add_driver(&tlv320aic23_driver))) {
		printk("tlv320aic23 i2c: Driver registration failed, module not inserted.\n");
		return res;
	}
	
	printk("TLV320AIC23 I2C version %s (%s)\n", TLV320AIC23_VERSION, 
	       TLV320AIC23_DATE);
	
	return 0;
}

static void __exit tlv320aic23_exit(void)
{
	int res;

	if ((res = i2c_del_driver(&tlv320aic23_driver))) 
		printk("tlv320aic23 i2c: Driver remove failed, module not removed.\n");
}

MODULE_AUTHOR("Kai Svahn <kai.svahn@nokia.com>");
MODULE_DESCRIPTION("I2C interface for TLV320AIC23 codec.");
MODULE_LICENSE("GPL");

module_init(tlv320aic23_init)
module_exit(tlv320aic23_exit)

EXPORT_SYMBOL(tlv320aic23_write_value);
