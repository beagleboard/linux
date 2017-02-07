/*
 * Microchip Serial Touchscreen Driver
 *
 * Copyright (c) 2010 Microchip Technology, Inc.
 * 
 * http://www.microchip.com/mtouch
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/*
 * This driver can handle serial Microchip controllers using the AR1XXX 5-byte protocol 
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include "serio-mchp.h"
#include <linux/init.h>
#include <linux/ctype.h>

#define DRIVER_DESC	"Microchip AR1XXX Serial Touchscreen Driver"

MODULE_AUTHOR("Steve Grahovac <steve.grahovac@microchip.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/*
 * Definitions & global arrays.
 */

#define MCHIP_MAX_LENGTH	5

/*
 * Per-touchscreen controller data.
 */

struct mchip {
	struct input_dev *dev;
	struct serio *serio;
	int id;
	int index;
	unsigned char data[MCHIP_MAX_LENGTH];
	char phys[32];
};

static void mchpar1xxx_process_data(struct mchip *mchip, unsigned char data)
{
	struct input_dev *dev = mchip->dev;

	mchip->data[mchip->index] = data;

	/****************************************************
	
	Data format, 5 bytes: SYNC, DATA1, DATA2, DATA3, DATA4
	
	SYNC [7:0]: 1,0,0,0,0,TOUCHSTATUS[0:0]
	DATA1[7:0]: 0,X-LOW[6:0]
	DATA2[7:0]: 0,X-HIGH[4:0]
	DATA3[7:0]: 0,Y-LOW[6:0]
	DATA4[7:0]: 0,Y-HIGH[4:0]
	
	TOUCHSTATUS: 0 = Touch up, 1 = Touch down
	
	****************************************************/		
	
	switch (mchip->index++) {
		case 0:
			if (!(0x80 & data))
			{
/*			    printk(KERN_ERR "mchip: Sync bit not set\n"); */
			    mchip->index = 0;			  
			}
			break;

		case (MCHIP_MAX_LENGTH - 1):
			/* verify byte is valid for current index */
			if (0x80 & data)
			{
				/* byte not valid */			  
				mchip->data[0]=data;
				mchip->index = 1;
				break;
			}			  

			mchip->index = 0;

/*			printk(KERN_ERR "mchip: %d %d %d\n",((mchip->data[2] & 0x1f) << 7) | (mchip->data[1] & 0x7f), 				((mchip->data[4] & 0x1f) << 7) | (mchip->data[3] & 0x7f), 0 != (mchip->data[0] & 7)); */			
			
			input_report_abs(dev, ABS_X, ((mchip->data[2] & 0x1f) << 7) | (mchip->data[1] & 0x7f));
			input_report_abs(dev, ABS_Y, ((mchip->data[4] & 0x1f) << 7) | (mchip->data[3] & 0x7f));
			input_report_key(dev, BTN_TOUCH, 0 != (mchip->data[0] & 1) );
			input_sync(dev);			
			break;
		default:
			/* verify byte is valid for current index */
			if (0x80 & data)
			{
				/* byte not valid */
				mchip->data[0]=data;
				mchip->index = 1;
			}			  
			break;
			
	}
}


static irqreturn_t mchpar1xxx_interrupt(struct serio *serio,
		unsigned char data, unsigned int flags)
{
    struct mchip *mchip = serio_get_drvdata(serio);

    mchpar1xxx_process_data(mchip, data);

    return IRQ_HANDLED;
}

static int mchpar1xxx_setup(struct mchip *mchip)
{
	struct input_dev *dev = mchip->dev;

	input_set_abs_params(dev, ABS_X, 0, 4095, 0, 0);
	input_set_abs_params(dev, ABS_Y, 0, 4095, 0, 0);
	return 0;
}

/*
 * mchpar1xxx_disconnect() is the opposite of mchpar1xxx_connect()
 */

static void mchpar1xxx_disconnect(struct serio *serio)
{
	struct mchip *mchip = serio_get_drvdata(serio);

	input_get_device(mchip->dev);
	input_unregister_device(mchip->dev);
	serio_close(serio);
	serio_set_drvdata(serio, NULL);
	input_put_device(mchip->dev);
	kfree(mchip);
}

/*
 * mchpar1xxx_connect() is the routine that is called when someone adds a
 * new serio device that supports AR1XXX protocol and registers it as
 * an input device.
 */

static int mchpar1xxx_connect(struct serio *serio, struct serio_driver *drv)
{
	struct mchip *mchip;
	struct input_dev *input_dev;
	int err;

	mchip = kzalloc(sizeof(struct mchip), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!mchip || !input_dev) {
		err = -ENOMEM;
		goto fail1;
	}

	mchip->serio = serio;
	mchip->id = serio->id.id;
	mchip->dev = input_dev;
	snprintf(mchip->phys, sizeof(mchip->phys), "%s/input0", serio->phys);

	input_dev->name = "Microchip AR1XXX Serial TouchScreen";
	input_dev->phys = mchip->phys;
	input_dev->id.bustype = BUS_RS232;
	input_dev->id.vendor = SERIO_MCHPAR1XXX;
	input_dev->id.product = mchip->id;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &serio->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	serio_set_drvdata(serio, mchip);

	err = serio_open(serio, drv);
	if (err)
		goto fail2;

	mchpar1xxx_setup(mchip);

	err = input_register_device(mchip->dev);
	if (err)
		goto fail3;

	return 0;

 fail3: serio_close(serio);
 fail2:	serio_set_drvdata(serio, NULL);
 fail1:	input_free_device(input_dev);
	kfree(mchip);
	return err;
}

/*
 * The serio driver structure.
 */

static struct serio_device_id mchpar1xxx_serio_ids[] = {
	{
		.type	= SERIO_RS232,
		.proto	= SERIO_MCHPAR1XXX,
		.id	= SERIO_ANY,
		.extra	= SERIO_ANY,
	},
	{ 0 }
};

MODULE_DEVICE_TABLE(serio, mchpar1xxx_serio_ids);

static struct serio_driver mchpar1xxx_drv = {
	.driver		= {
		.name	= "mchip",
	},
	.description	= DRIVER_DESC,
	.id_table	= mchpar1xxx_serio_ids,
	.interrupt	= mchpar1xxx_interrupt,
	.connect	= mchpar1xxx_connect,
	.disconnect	= mchpar1xxx_disconnect,
};

/*
 * The functions for inserting/removing us as a module.
 */

static int __init mchpar1xxx_init(void)
{
	return serio_register_driver(&mchpar1xxx_drv);
}

static void __exit mchpar1xxx_exit(void)
{
	serio_unregister_driver(&mchpar1xxx_drv);
}

module_init(mchpar1xxx_init);
module_exit(mchpar1xxx_exit);
