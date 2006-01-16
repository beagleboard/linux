/*
 * drivers/i2c/chips/menelaus.c
 *
 * Copyright (C) 2004 Texas Instruments, Inc.
 *
 * Some parts based tps65010.c:
 * Copyright (C) 2004 Texas Instruments and
 * Copyright (C) 2004-2005 David Brownell
 *
 * Some parts based on tlv320aic24.c:
 * Copyright (C) by Kai Svahn <kai.svahn@nokia.com>
 *
 * Changes for interrupt handling and clean-up by
 * Tony Lindgren <tony@atomide.com> and Imre Deak <imre.deak@nokia.com>
 * Copyright (C) 2005 Nokia Corporation
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sched.h>

#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/menelaus.h>

#define DRIVER_NAME			"menelaus"

#define pr_err(fmt, arg...)	printk(KERN_ERR DRIVER_NAME ": ", ## arg);

#define MENELAUS_I2C_ADDRESS		0x72

#define MENELAUS_REV			0x01
#define MENELAUS_VCORE_CTRL1		0x02
#define MENELAUS_VCORE_CTRL2		0x03
#define MENELAUS_VCORE_CTRL3		0x04
#define MENELAUS_VCORE_CTRL4		0x05
#define MENELAUS_VCORE_CTRL5		0x06
#define MENELAUS_DCDC_CTRL1		0x07
#define MENELAUS_DCDC_CTRL2		0x08
#define MENELAUS_DCDC_CTRL3		0x09
#define MENELAUS_LDO_CTRL1		0x0A
#define MENELAUS_LDO_CTRL2		0x0B
#define MENELAUS_LDO_CTRL3		0x0C
#define MENELAUS_LDO_CTRL4		0x0D
#define MENELAUS_LDO_CTRL5		0x0E
#define MENELAUS_LDO_CTRL6		0x0F
#define MENELAUS_LDO_CTRL7		0x10
#define MENELAUS_LDO_CTRL8		0x11
#define MENELAUS_SLEEP_CTRL1		0x12
#define MENELAUS_SLEEP_CTRL2		0x13
#define MENELAUS_DEVICE_OFF		0x14
#define MENELAUS_OSC_CTRL		0x15
#define MENELAUS_DETECT_CTRL		0x16
#define MENELAUS_INT_MASK1		0x17
#define MENELAUS_INT_MASK2		0x18
#define MENELAUS_INT_STATUS1		0x19
#define MENELAUS_INT_STATUS2		0x1A
#define MENELAUS_INT_ACK1		0x1B
#define MENELAUS_INT_ACK2		0x1C
#define MENELAUS_GPIO_CTRL		0x1D
#define MENELAUS_GPIO_IN		0x1E
#define MENELAUS_GPIO_OUT		0x1F
#define MENELAUS_BBSMS			0x20
#define MENELAUS_RTC_CTRL		0x21
#define MENELAUS_RTC_UPDATE		0x22
#define MENELAUS_RTC_SEC		0x23
#define MENELAUS_RTC_MIN		0x24
#define MENELAUS_RTC_HR			0x25
#define MENELAUS_RTC_DAY		0x26
#define MENELAUS_RTC_MON		0x27
#define MENELAUS_RTC_YR			0x28
#define MENELAUS_RTC_WKDAY		0x29
#define MENELAUS_RTC_AL_SEC		0x2A
#define MENELAUS_RTC_AL_MIN		0x2B
#define MENELAUS_RTC_AL_HR		0x2C
#define MENELAUS_RTC_AL_DAY		0x2D
#define MENELAUS_RTC_AL_MON		0x2E
#define MENELAUS_RTC_AL_YR		0x2F
#define MENELAUS_RTC_COMP_MSB		0x30
#define MENELAUS_RTC_COMP_LSB		0x31
#define MENELAUS_S1_PULL_EN		0x32
#define MENELAUS_S1_PULL_DIR		0x33
#define MENELAUS_S2_PULL_EN		0x34
#define MENELAUS_S2_PULL_DIR		0x35
#define MENELAUS_MCT_CTRL1		0x36
#define MENELAUS_MCT_CTRL2		0x37
#define MENELAUS_MCT_CTRL3		0x38
#define MENELAUS_MCT_PIN_ST		0x39
#define MENELAUS_DEBOUNCE1		0x3A

#define IH_MENELAUS_IRQS		12
#define MENELAUS_MMC_S1CD_IRQ		0	/* MMC slot 1 card change */
#define MENELAUS_MMC_S2CD_IRQ		1	/* MMC slot 2 card change */
#define MENELAUS_MMC_S1D1_IRQ		2	/* MMC DAT1 low in slot 1 */
#define MENELAUS_MMC_S2D1_IRQ		3	/* MMC DAT1 low in slot 2 */
#define MENELAUS_LOWBAT_IRQ		4	/* Low battery */
#define MENELAUS_HOTDIE_IRQ		5	/* Hot die detect */
#define MENELAUS_UVLO_IRQ		6	/* UVLO detect */
#define MENELAUS_TSHUT_IRQ		7	/* Thermal shutdown */
#define MENELAUS_RTCTMR_IRQ		8	/* RTC timer */
#define MENELAUS_RTCALM_IRQ		9	/* RTC alarm */
#define MENELAUS_RTCERR_IRQ		10	/* RTC error */
#define MENELAUS_PSHBTN_IRQ		11	/* Push button */
#define MENELAUS_RESERVED12_IRQ		12	/* Reserved */
#define MENELAUS_RESERVED13_IRQ		13	/* Reserved */
#define MENELAUS_RESERVED14_IRQ		14	/* Reserved */
#define MENELAUS_RESERVED15_IRQ		15	/* Reserved */

static void menelaus_work(void * _menelaus);

/* Initialized by menelaus_init */
static unsigned short normal_i2c[] = { MENELAUS_I2C_ADDRESS, I2C_CLIENT_END };

I2C_CLIENT_INSMOD;

struct menelaus_chip {
	unsigned long		initialized;
	struct semaphore	lock;
	struct i2c_client	client;
	struct work_struct	work;
	int			irq;
	void			*handlers[16];
	void			(*mmc_callback)(unsigned long data, u8 mask);
	unsigned long		mmc_callback_data;
};

static struct menelaus_chip menelaus;

static void menelaus_write(u8 value, u8 reg)
{
	if (i2c_smbus_write_byte_data(&menelaus.client, reg, value) < 0)
		pr_err("write error");
}

static u8 menelaus_read(u8 reg)
{
	int val = i2c_smbus_read_byte_data(&menelaus.client, reg);

	if (val < 0) {
		pr_err("read error");
		return 0;
	}

	return val;
}

static void menelaus_enable_irq(int irq)
{
	if (irq > 7)
		menelaus_write(menelaus_read(MENELAUS_INT_MASK2) &
			       ~(1 << (irq - 8)), MENELAUS_INT_MASK2);
	else
		menelaus_write(menelaus_read(MENELAUS_INT_MASK1) &
			       ~(1 << irq), MENELAUS_INT_MASK1);
}

static void menelaus_disable_irq(int irq)
{
	if (irq > 7)
		menelaus_write(menelaus_read(MENELAUS_INT_MASK2)
			       | (1 << (irq - 8)), MENELAUS_INT_MASK2);
	else
		menelaus_write(menelaus_read(MENELAUS_INT_MASK1)
			       | (1 << irq), MENELAUS_INT_MASK1);
}

static void menelaus_ack_irq(int irq)
{
	if (irq > 7)
		menelaus_write(1 << (irq - 8), MENELAUS_INT_ACK2);
	else
		menelaus_write(1 << irq, MENELAUS_INT_ACK1);	
}

/* Adds a handler for an interrupt. Does not run in interrupt context */
static int menelaus_add_irq_work(int irq, void * handler)
{
	down(&menelaus.lock);
	menelaus.handlers[irq] = handler;
	menelaus_enable_irq(irq);
	up(&menelaus.lock);

	return 0;
}

/* Removes handler for an interrupt */
static void menelaus_remove_irq_work(int irq)
{
	down(&menelaus.lock);
	menelaus_disable_irq(irq);
	menelaus.handlers[irq] = NULL;
	up(&menelaus.lock);
}

/*-----------------------------------------------------------------------*/

/*
 * Toggles the MMC slots between open-drain and push-pull mode.
 * We always set both slots the same way.
 */
void menelaus_mmc_opendrain(int enable)
{
	unsigned char reg = menelaus_read(MENELAUS_MCT_CTRL1);

	if (enable)
		reg |= (0x3 << 2);
	else
		reg &= ~(0x3 << 2);

	menelaus_write(reg, MENELAUS_MCT_CTRL1);
}
EXPORT_SYMBOL(menelaus_mmc_opendrain);

/*
 * Gets scheduled when a card detect interrupt happens. Note that in some cases
 * this line is wired to card cover switch rather than the card detect switch
 * in each slot. In this case the cards are not seen by menelaus.
 * FIXME: Add handling for D1 too
 */
static int menelaus_mmc_cd_work(struct menelaus_chip *menelaus)
{
	unsigned char reg;
	unsigned char card_mask = 0;

	reg = menelaus_read(MENELAUS_MCT_PIN_ST);

	if (!(reg & 0x1))
		card_mask |= (1 << 0);

	if (!(reg & 0x2))
		card_mask |= (1 << 1);

	if (menelaus->mmc_callback)
		menelaus->mmc_callback(menelaus->mmc_callback_data, card_mask);

	return 0;
}

/* Initializes MMC slots */
void menelaus_mmc_register(void (*callback)(unsigned long data, u8 card_mask), unsigned long data)
{
	int reg;

	/* DCDC3 to 3V */
	reg = menelaus_read(MENELAUS_DCDC_CTRL1);
	reg |= 0x6 << 4;
	menelaus_write(reg, MENELAUS_DCDC_CTRL1);

	reg = menelaus_read(MENELAUS_DCDC_CTRL3);
	reg |= 0x6;
	menelaus_write(reg, MENELAUS_DCDC_CTRL3);

	/* Enable both slots, do not set auto shutdown */
	reg = menelaus_read(MENELAUS_MCT_CTRL3);
	reg |= 0x3;
	menelaus_write(reg, MENELAUS_MCT_CTRL3);

	/* Enable card detect for both slots, slot 2 powered from DCDC3 */
	reg = menelaus_read(MENELAUS_MCT_CTRL2);
	reg |= 0xf0;
	menelaus_write(reg, MENELAUS_MCT_CTRL2);

	/* Set both slots in open-drain mode, card detect normally closed */
	reg = menelaus_read(MENELAUS_MCT_CTRL1);
	reg |= 0xfc;
	menelaus_write(reg, MENELAUS_MCT_CTRL1);

	/* Set MMC voltage */
	reg = menelaus_read(MENELAUS_LDO_CTRL7);
	reg |= 0x03;
	menelaus_write(reg, MENELAUS_LDO_CTRL7);

	menelaus.mmc_callback_data = data;
	menelaus.mmc_callback = callback;

	menelaus_add_irq_work(MENELAUS_MMC_S1CD_IRQ, menelaus_mmc_cd_work);
	menelaus_add_irq_work(MENELAUS_MMC_S2CD_IRQ, menelaus_mmc_cd_work);
	menelaus_add_irq_work(MENELAUS_MMC_S1D1_IRQ, menelaus_mmc_cd_work);
	menelaus_add_irq_work(MENELAUS_MMC_S2D1_IRQ, menelaus_mmc_cd_work);
}
EXPORT_SYMBOL(menelaus_mmc_register);

void menelaus_mmc_remove(void)
{
	menelaus_remove_irq_work(MENELAUS_MMC_S1CD_IRQ);
	menelaus_remove_irq_work(MENELAUS_MMC_S2CD_IRQ);
	menelaus_remove_irq_work(MENELAUS_MMC_S1D1_IRQ);
	menelaus_remove_irq_work(MENELAUS_MMC_S2D1_IRQ);

	menelaus.mmc_callback = NULL;
	menelaus.mmc_callback_data = 0;

	/* FIXME: Shutdown MMC components of Menelaus */
}
EXPORT_SYMBOL(menelaus_mmc_remove);

/*-----------------------------------------------------------------------*/

/* Handles Menelaus interrupts. Does not run in interrupt context */
static void menelaus_work(void * _menelaus)
{
	struct menelaus_chip *menelaus = _menelaus;
	int (*handler)(struct menelaus_chip *menelaus);
 
	while (1) {
		int i;
		unsigned char isr;

		isr = menelaus_read(MENELAUS_INT_STATUS1) |
		      (menelaus_read(MENELAUS_INT_STATUS2) << 8);

		if (!isr)
			break;

		for (i = 0; i < IH_MENELAUS_IRQS; i++) {
			if (isr & (1 << i)) {
				down(&menelaus->lock);
				menelaus_disable_irq(i);
				menelaus_ack_irq(i);
				if (menelaus->handlers[i]) {
					handler = menelaus->handlers[i];
					handler(menelaus);
				}
				menelaus_enable_irq(i);
				up(&menelaus->lock);
			}
		}
	}
	enable_irq(menelaus->irq);
}

/*
 * We cannot use I2C in interrupt context, so we just schedule work.
 */
static irqreturn_t menelaus_irq(int irq, void *_menelaus, struct pt_regs *regs)
{
	struct menelaus_chip *menelaus = _menelaus;

	disable_irq_nosync(irq);
	(void)schedule_work(&menelaus->work);

	return IRQ_HANDLED;
}

static struct i2c_driver menelaus_i2c_driver;

static int menelaus_probe(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client	*c;
	int			rev = 0;
	int			err = 0, i;

	if (test_and_set_bit(0, &menelaus.initialized))
		return -EBUSY;

	c = &menelaus.client;
	strncpy(c->name, DRIVER_NAME, sizeof(c->name));
	c->addr		= address;
	c->adapter	= adapter;
	c->driver	= &menelaus_i2c_driver;
	c->flags	= 0;

	if ((err = i2c_attach_client(c)) < 0) {
		pr_err("couldn't attach\n");
		goto fail1;
	}

	/* If a true probe check the device */
	if (kind < 0 && (rev = menelaus_read(MENELAUS_REV)) < 0) {
		pr_err("device not found");
		err = -ENODEV;
		goto fail2;
	}

	/* Most likely Menelaus interrupt is at SYS_NIRQ */
	omap_cfg_reg(W19_24XX_SYS_NIRQ);
	menelaus.irq = INT_24XX_SYS_NIRQ;

	/* Disable all menelaus interrupts */
	for (i = 0; i < 16; i++) {
		menelaus_ack_irq(i);
		menelaus_disable_irq(i);
	}

	err = request_irq(menelaus.irq, menelaus_irq, SA_INTERRUPT,
			  DRIVER_NAME, &menelaus);
	if (err)
		printk(KERN_ERR "Could not get Menelaus IRQ\n");

	init_MUTEX(&menelaus.lock);
	INIT_WORK(&menelaus.work, menelaus_work, &menelaus);

	if (kind < 0)
		pr_info("Menelaus rev %d.%d\n", rev >> 4, rev & 0x0f);

	return 0;

fail2:
	i2c_detach_client(c);
fail1:
	clear_bit(0, &menelaus.initialized);
	return err;
}

static int menelaus_remove(struct i2c_client *client)
{
	int err;

	free_irq(menelaus.irq, &menelaus);

	if ((err = i2c_detach_client(client))) {
		pr_err("client deregistration failed\n");
		return err;
	}

	clear_bit(0, &menelaus.initialized);

	return 0;
}

/*-----------------------------------------------------------------------*/

static int menelaus_scan_bus(struct i2c_adapter *bus)
{
	if (!i2c_check_functionality(bus, I2C_FUNC_SMBUS_BYTE_DATA |
					  I2C_FUNC_SMBUS_WRITE_BYTE)) {
		pr_err("invalid i2c bus functionality\n");
		return -EINVAL;
	}

	return i2c_probe(bus, &addr_data, menelaus_probe);
}

static struct i2c_driver menelaus_i2c_driver = {
	.driver {
		.name		= DRIVER_NAME,
	},
	.id		= I2C_DRIVERID_MISC, /*FIXME:accroding to i2c-ids.h */
	.class		= I2C_CLASS_HWMON,
	.attach_adapter	= menelaus_scan_bus,
	.detach_client	= menelaus_remove,
};

static int __init menelaus_init(void)
{
	int res;

	if ((res = i2c_add_driver(&menelaus_i2c_driver)) < 0) {
		pr_err("driver registration failed\n");
		return res;
	}

	return 0;
}

static void __exit menelaus_exit(void)
{
	if (i2c_del_driver(&menelaus_i2c_driver) < 0)
		pr_err("driver remove failed\n");

	/* FIXME: Shutdown menelaus parts that can be shut down */
}

MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_DESCRIPTION("I2C interface for Menelaus.");
MODULE_LICENSE("GPL");

module_init(menelaus_init);
module_exit(menelaus_exit);
