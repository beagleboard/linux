/*
 * twl4030_core.c - driver for TWL4030 PM and audio CODEC device
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * Modifications to defer interrupt handling to a kernel thread:
 * Copyright (C) 2006 MontaVista Software, Inc.
 *
 * Based on tlv320aic23.c:
 * Copyright (c) by Kai Svahn <kai.svahn@nokia.com>
 *
 * Code cleanup and modifications to IRQ handler.
 * by syed khasim <x0khasim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/random.h>
#include <linux/syscalls.h>
#include <linux/kthread.h>

#include <linux/i2c.h>
#include <linux/i2c/twl4030.h>
#include <linux/i2c/twl4030-gpio.h>
#include <linux/i2c/twl4030-madc.h>
#include <linux/i2c/twl4030-pwrirq.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/irq.h>

#include <asm/mach/irq.h>

#include <mach/gpio.h>
#include <mach/mux.h>

#define DRIVER_NAME			"twl4030"

/* Macro Definitions */
#define TWL_CLIENT_STRING		"TWL4030-ID"
#define TWL_CLIENT_USED			1
#define TWL_CLIENT_FREE			0

/* IRQ Flags */
#define FREE				0
#define USED				1

/* Primary Interrupt Handler on TWL4030 Registers */

/* Register Definitions */

#define REG_PIH_ISR_P1			(0x1)
#define REG_PIH_ISR_P2			(0x2)
#define REG_PIH_SIR			(0x3)

/* Triton Core internal information (BEGIN) */

/* Last - for index max*/
#define TWL4030_MODULE_LAST		TWL4030_MODULE_SECURED_REG

/* Slave address */
#define TWL4030_NUM_SLAVES		0x04
#define TWL4030_SLAVENUM_NUM0		0x00
#define TWL4030_SLAVENUM_NUM1		0x01
#define TWL4030_SLAVENUM_NUM2		0x02
#define TWL4030_SLAVENUM_NUM3		0x03
#define TWL4030_SLAVEID_ID0		0x48
#define TWL4030_SLAVEID_ID1		0x49
#define TWL4030_SLAVEID_ID2		0x4A
#define TWL4030_SLAVEID_ID3		0x4B

/* Base Address defns */
/* USB ID */
#define TWL4030_BASEADD_USB		0x0000
/* AUD ID */
#define TWL4030_BASEADD_AUDIO_VOICE	0x0000
#define TWL4030_BASEADD_GPIO		0x0098

#define TWL4030_BASEADD_INTBR		0x0085
#define TWL4030_BASEADD_PIH		0x0080
#define TWL4030_BASEADD_TEST		0x004C
/* AUX ID */
#define TWL4030_BASEADD_INTERRUPTS	0x00B9
#define TWL4030_BASEADD_LED		0x00EE
#define TWL4030_BASEADD_MADC		0x0000
#define TWL4030_BASEADD_MAIN_CHARGE	0x0074
#define TWL4030_BASEADD_PRECHARGE	0x00AA
#define TWL4030_BASEADD_PWM0		0x00F8
#define TWL4030_BASEADD_PWM1		0x00FB
#define TWL4030_BASEADD_PWMA		0x00EF
#define TWL4030_BASEADD_PWMB		0x00F1
#define TWL4030_BASEADD_KEYPAD		0x00D2
/* POWER ID */
#define TWL4030_BASEADD_BACKUP		0x0014
#define TWL4030_BASEADD_INT		0x002E
#define TWL4030_BASEADD_PM_MASTER	0x0036
#define TWL4030_BASEADD_PM_RECEIVER	0x005B
#define TWL4030_BASEADD_RTC		0x001C
#define TWL4030_BASEADD_SECURED_REG	0x0000

/* TWL4030 BCI registers */
#define TWL4030_INTERRUPTS_BCIIMR1A	0x2
#define TWL4030_INTERRUPTS_BCIIMR2A	0x3
#define TWL4030_INTERRUPTS_BCIIMR1B	0x6
#define TWL4030_INTERRUPTS_BCIIMR2B	0x7
#define TWL4030_INTERRUPTS_BCIISR1A	0x0
#define TWL4030_INTERRUPTS_BCIISR2A	0x1
#define TWL4030_INTERRUPTS_BCIISR1B	0x4
#define TWL4030_INTERRUPTS_BCIISR2B	0x5

/* TWL4030 keypad registers */
#define TWL4030_KEYPAD_KEYP_IMR1	0x12
#define TWL4030_KEYPAD_KEYP_IMR2	0x14
#define TWL4030_KEYPAD_KEYP_ISR1	0x11
#define TWL4030_KEYPAD_KEYP_ISR2	0x13


/* Triton Core internal information (END) */

/* Few power values */
#define R_CFG_BOOT			0x05
#define R_PROTECT_KEY			0x0E

/* access control */
#define KEY_UNLOCK1			0xce
#define KEY_UNLOCK2			0xec
#define KEY_LOCK			0x00

#define HFCLK_FREQ_19p2_MHZ		(1 << 0)
#define HFCLK_FREQ_26_MHZ		(2 << 0)
#define HFCLK_FREQ_38p4_MHZ		(3 << 0)
#define HIGH_PERF_SQ			(1 << 3)

/* on I2C-1 for 2430SDP */
#define CONFIG_I2C_TWL4030_ID		1

/* SIH_CTRL registers that aren't defined elsewhere */
#define TWL4030_INTERRUPTS_BCISIHCTRL	0x0d
#define TWL4030_MADC_MADC_SIH_CTRL	0x67
#define TWL4030_KEYPAD_KEYP_SIH_CTRL	0x17

#define TWL4030_SIH_CTRL_COR_MASK	(1 << 2)

/**
 * struct twl4030_mod_iregs - TWL module IMR/ISR regs to mask/clear at init
 * @mod_no: TWL4030 module number (e.g., TWL4030_MODULE_GPIO)
 * @sih_ctrl: address of module SIH_CTRL register
 * @reg_cnt: number of IMR/ISR regs
 * @imrs: pointer to array of TWL module interrupt mask register indices
 * @isrs: pointer to array of TWL module interrupt status register indices
 *
 * Ties together TWL4030 modules and lists of IMR/ISR registers to mask/clear
 * during twl_init_irq().
 */
struct twl4030_mod_iregs {
	const u8 mod_no;
	const u8 sih_ctrl;
	const u8 reg_cnt;
	const u8 *imrs;
	const u8 *isrs;
};

/* TWL4030 INT module interrupt mask registers */
static const u8 __initconst twl4030_int_imr_regs[] = {
	TWL4030_INT_PWR_IMR1,
	TWL4030_INT_PWR_IMR2,
};

/* TWL4030 INT module interrupt status registers */
static const u8 __initconst twl4030_int_isr_regs[] = {
	TWL4030_INT_PWR_ISR1,
	TWL4030_INT_PWR_ISR2,
};

/* TWL4030 INTERRUPTS module interrupt mask registers */
static const u8 __initconst twl4030_interrupts_imr_regs[] = {
	TWL4030_INTERRUPTS_BCIIMR1A,
	TWL4030_INTERRUPTS_BCIIMR1B,
	TWL4030_INTERRUPTS_BCIIMR2A,
	TWL4030_INTERRUPTS_BCIIMR2B,
};

/* TWL4030 INTERRUPTS module interrupt status registers */
static const u8 __initconst twl4030_interrupts_isr_regs[] = {
	TWL4030_INTERRUPTS_BCIISR1A,
	TWL4030_INTERRUPTS_BCIISR1B,
	TWL4030_INTERRUPTS_BCIISR2A,
	TWL4030_INTERRUPTS_BCIISR2B,
};

/* TWL4030 MADC module interrupt mask registers */
static const u8 __initconst twl4030_madc_imr_regs[] = {
	TWL4030_MADC_IMR1,
	TWL4030_MADC_IMR2,
};

/* TWL4030 MADC module interrupt status registers */
static const u8 __initconst twl4030_madc_isr_regs[] = {
	TWL4030_MADC_ISR1,
	TWL4030_MADC_ISR2,
};

/* TWL4030 keypad module interrupt mask registers */
static const u8 __initconst twl4030_keypad_imr_regs[] = {
	TWL4030_KEYPAD_KEYP_IMR1,
	TWL4030_KEYPAD_KEYP_IMR2,
};

/* TWL4030 keypad module interrupt status registers */
static const u8 __initconst twl4030_keypad_isr_regs[] = {
	TWL4030_KEYPAD_KEYP_ISR1,
	TWL4030_KEYPAD_KEYP_ISR2,
};

/* TWL4030 GPIO module interrupt mask registers */
static const u8 __initconst twl4030_gpio_imr_regs[] = {
	REG_GPIO_IMR1A,
	REG_GPIO_IMR1B,
	REG_GPIO_IMR2A,
	REG_GPIO_IMR2B,
	REG_GPIO_IMR3A,
	REG_GPIO_IMR3B,
};

/* TWL4030 GPIO module interrupt status registers */
static const u8 __initconst twl4030_gpio_isr_regs[] = {
	REG_GPIO_ISR1A,
	REG_GPIO_ISR1B,
	REG_GPIO_ISR2A,
	REG_GPIO_ISR2B,
	REG_GPIO_ISR3A,
	REG_GPIO_ISR3B,
};

/* TWL4030 modules that have IMR/ISR registers that must be masked/cleared */
static const struct twl4030_mod_iregs __initconst twl4030_mod_regs[] = {
	{
		.mod_no	  = TWL4030_MODULE_INT,
		.sih_ctrl = TWL4030_INT_PWR_SIH_CTRL,
		.reg_cnt  = ARRAY_SIZE(twl4030_int_imr_regs),
		.imrs	  = twl4030_int_imr_regs,
		.isrs	  = twl4030_int_isr_regs,
	},
	{
		.mod_no	  = TWL4030_MODULE_INTERRUPTS,
		.sih_ctrl = TWL4030_INTERRUPTS_BCISIHCTRL,
		.reg_cnt  = ARRAY_SIZE(twl4030_interrupts_imr_regs),
		.imrs	  = twl4030_interrupts_imr_regs,
		.isrs	  = twl4030_interrupts_isr_regs,
	},
	{
		.mod_no	  = TWL4030_MODULE_MADC,
		.sih_ctrl = TWL4030_MADC_MADC_SIH_CTRL,
		.reg_cnt  = ARRAY_SIZE(twl4030_madc_imr_regs),
		.imrs	  = twl4030_madc_imr_regs,
		.isrs	  = twl4030_madc_isr_regs,
	},
	{
		.mod_no	  = TWL4030_MODULE_KEYPAD,
		.sih_ctrl = TWL4030_KEYPAD_KEYP_SIH_CTRL,
		.reg_cnt  = ARRAY_SIZE(twl4030_keypad_imr_regs),
		.imrs	  = twl4030_keypad_imr_regs,
		.isrs	  = twl4030_keypad_isr_regs,
	},
	{
		.mod_no	  = TWL4030_MODULE_GPIO,
		.sih_ctrl = REG_GPIO_SIH_CTRL,
		.reg_cnt  = ARRAY_SIZE(twl4030_gpio_imr_regs),
		.imrs	  = twl4030_gpio_imr_regs,
		.isrs	  = twl4030_gpio_isr_regs,
	},
};


/* Helper functions */
static int
twl4030_detect_client(struct i2c_adapter *adapter, unsigned char sid);
static int twl4030_attach_adapter(struct i2c_adapter *adapter);
static int twl4030_detach_client(struct i2c_client *client);
static void do_twl4030_irq(unsigned int irq, irq_desc_t *desc);

static void twl_init_irq(void);

/* Data Structures */
/* To have info on T2 IRQ substem activated or not */
static unsigned char twl_irq_used = FREE;
static struct completion irq_event;

/* Structure to define on TWL4030 Slave ID */
struct twl4030_client {
	struct i2c_client client;
	const char client_name[sizeof(TWL_CLIENT_STRING) + 1];
	const unsigned char address;
	const char adapter_index;
	unsigned char inuse;

	/* max numb of i2c_msg required is for read =2 */
	struct i2c_msg xfer_msg[2];

	/* To lock access to xfer_msg */
	struct mutex xfer_lock;
};

/* Module Mapping */
struct twl4030mapping {
	unsigned char sid;	/* Slave ID */
	unsigned char base;	/* base address */
};

/* mapping the module id to slave id and base address */
static struct twl4030mapping twl4030_map[TWL4030_MODULE_LAST + 1] = {
	{ TWL4030_SLAVENUM_NUM0, TWL4030_BASEADD_USB },
	{ TWL4030_SLAVENUM_NUM1, TWL4030_BASEADD_AUDIO_VOICE },
	{ TWL4030_SLAVENUM_NUM1, TWL4030_BASEADD_GPIO },
	{ TWL4030_SLAVENUM_NUM1, TWL4030_BASEADD_INTBR },
	{ TWL4030_SLAVENUM_NUM1, TWL4030_BASEADD_PIH },
	{ TWL4030_SLAVENUM_NUM1, TWL4030_BASEADD_TEST },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_KEYPAD },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_MADC },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_INTERRUPTS },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_LED },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_MAIN_CHARGE },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_PRECHARGE },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_PWM0 },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_PWM1 },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_PWMA },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_PWMB },
	{ TWL4030_SLAVENUM_NUM3, TWL4030_BASEADD_BACKUP },
	{ TWL4030_SLAVENUM_NUM3, TWL4030_BASEADD_INT },
	{ TWL4030_SLAVENUM_NUM3, TWL4030_BASEADD_PM_MASTER },
	{ TWL4030_SLAVENUM_NUM3, TWL4030_BASEADD_PM_RECEIVER },
	{ TWL4030_SLAVENUM_NUM3, TWL4030_BASEADD_RTC },
	{ TWL4030_SLAVENUM_NUM3, TWL4030_BASEADD_SECURED_REG },
};

static struct twl4030_client twl4030_modules[TWL4030_NUM_SLAVES] = {
	{
		.address	= TWL4030_SLAVEID_ID0,
		.client_name	= TWL_CLIENT_STRING "0",
		.adapter_index	= CONFIG_I2C_TWL4030_ID,
	},
	{
		.address	= TWL4030_SLAVEID_ID1,
		.client_name	= TWL_CLIENT_STRING "1",
		.adapter_index	= CONFIG_I2C_TWL4030_ID,
	},
	{
		.address	= TWL4030_SLAVEID_ID2,
		.client_name	= TWL_CLIENT_STRING "2",
		.adapter_index	= CONFIG_I2C_TWL4030_ID,
	},
	{
		.address	= TWL4030_SLAVEID_ID3,
		.client_name	= TWL_CLIENT_STRING "3",
		.adapter_index	= CONFIG_I2C_TWL4030_ID,
	},
};

/* One Client Driver , 4 Clients */
static struct i2c_driver twl4030_driver = {
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.attach_adapter	= twl4030_attach_adapter,
	.detach_client	= twl4030_detach_client,
};

/*
 * TWL4030 doesn't have PIH mask, hence dummy function for mask
 * and unmask.
 */

static void twl4030_i2c_ackirq(unsigned int irq) {}
static void twl4030_i2c_disableint(unsigned int irq) {}
static void twl4030_i2c_enableint(unsigned int irq) {}

/* information for processing in the Work Item */
static struct irq_chip twl4030_irq_chip = {
	.name	= "twl4030",
	.ack	= twl4030_i2c_ackirq,
	.mask	= twl4030_i2c_disableint,
	.unmask	= twl4030_i2c_enableint,
};

/* Global Functions */

/**
 * twl4030_i2c_write - Writes a n bit register in TWL4030
 * @mod_no: module number
 * @value: an array of num_bytes+1 containing data to write
 * @reg: register address (just offset will do)
 * @num_bytes: number of bytes to transfer
 *
 * IMPORTANT: for 'value' parameter: Allocate value num_bytes+1 and
 * valid data starts at Offset 1.
 *
 * Returns the result of operation - 0 is success
 */
int twl4030_i2c_write(u8 mod_no, u8 *value, u8 reg, u8 num_bytes)
{
	int ret;
	int sid;
	struct twl4030_client *twl;
	struct i2c_msg *msg;

	if (unlikely(mod_no > TWL4030_MODULE_LAST)) {
		pr_err("Invalid module Number\n");
		return -EPERM;
	}
	sid = twl4030_map[mod_no].sid;
	twl = &twl4030_modules[sid];

	if (unlikely(twl->inuse != TWL_CLIENT_USED)) {
		pr_err("I2C Client[%d] is not initialized[%d]\n",
		       sid, __LINE__);
		return -EPERM;
	}
	mutex_lock(&twl->xfer_lock);
	/*
	 * [MSG1]: fill the register address data
	 * fill the data Tx buffer
	 */
	msg = &twl->xfer_msg[0];
	msg->addr = twl->address;
	msg->len = num_bytes + 1;
	msg->flags = 0;
	msg->buf = value;
	/* over write the first byte of buffer with the register address */
	*value = twl4030_map[mod_no].base + reg;
	ret = i2c_transfer(twl->client.adapter, twl->xfer_msg, 1);
	mutex_unlock(&twl->xfer_lock);

	/* i2cTransfer returns num messages.translate it pls.. */
	if (ret >= 0)
		ret = 0;
	return ret;
}
EXPORT_SYMBOL(twl4030_i2c_write);

/**
 * twl4030_i2c_read - Reads a n bit register in TWL4030
 * @mod_no: module number
 * @value: an array of num_bytes containing data to be read
 * @reg: register address (just offset will do)
 * @num_bytes: number of bytes to transfer
 *
 * Returns result of operation - num_bytes is success else failure.
 */
int twl4030_i2c_read(u8 mod_no, u8 *value, u8 reg, u8 num_bytes)
{
	int ret;
	u8 val;
	int sid;
	struct twl4030_client *twl;
	struct i2c_msg *msg;

	if (unlikely(mod_no > TWL4030_MODULE_LAST)) {
		pr_err("Invalid module Number\n");
		return -EPERM;
	}
	sid = twl4030_map[mod_no].sid;
	twl = &twl4030_modules[sid];

	if (unlikely(twl->inuse != TWL_CLIENT_USED)) {
		pr_err("I2C Client[%d] is not initialized[%d]\n", sid,
		       __LINE__);
		return -EPERM;
	}
	mutex_lock(&twl->xfer_lock);
	/* [MSG1] fill the register address data */
	msg = &twl->xfer_msg[0];
	msg->addr = twl->address;
	msg->len = 1;
	msg->flags = 0;	/* Read the register value */
	val = twl4030_map[mod_no].base + reg;
	msg->buf = &val;
	/* [MSG2] fill the data rx buffer */
	msg = &twl->xfer_msg[1];
	msg->addr = twl->address;
	msg->flags = I2C_M_RD;	/* Read the register value */
	msg->len = num_bytes;	/* only n bytes */
	msg->buf = value;
	ret = i2c_transfer(twl->client.adapter, twl->xfer_msg, 2);
	mutex_unlock(&twl->xfer_lock);

	/* i2cTransfer returns num messages.translate it pls.. */
	if (ret >= 0)
		ret = 0;
	return ret;
}
EXPORT_SYMBOL(twl4030_i2c_read);

/**
 * twl4030_i2c_write_u8 - Writes a 8 bit register in TWL4030
 * @mod_no: module number
 * @value: the value to be written 8 bit
 * @reg: register address (just offset will do)
 *
 * Returns result of operation - 0 is success
 */
int twl4030_i2c_write_u8(u8 mod_no, u8 value, u8 reg)
{

	/* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
	u8 temp_buffer[2] = { 0 };
	/* offset 1 contains the data */
	temp_buffer[1] = value;
	return twl4030_i2c_write(mod_no, temp_buffer, reg, 1);
}
EXPORT_SYMBOL(twl4030_i2c_write_u8);

/**
 * twl4030_i2c_read_u8 - Reads a 8 bit register from TWL4030
 * @mod_no: module number
 * @value: the value read 8 bit
 * @reg: register address (just offset will do)
 *
 * Returns result of operation - 0 is success
 */
int twl4030_i2c_read_u8(u8 mod_no, u8 *value, u8 reg)
{
	return twl4030_i2c_read(mod_no, value, reg, 1);
}
EXPORT_SYMBOL(twl4030_i2c_read_u8);

/* Helper Functions */

/*
 * do_twl4030_module_irq() is the desc->handle method for each of the twl4030
 * module interrupts.  It executes in kernel thread context.
 * On entry, cpu interrupts are disabled.
 */
static void do_twl4030_module_irq(unsigned int irq, irq_desc_t *desc)
{
	struct irqaction *action;
	const unsigned int cpu = smp_processor_id();

	/*
	 * Earlier this was desc->triggered = 1;
	 */
	desc->status |= IRQ_LEVEL;

	/*
	 * The desc->handle method would normally call the desc->chip->ack
	 * method here, but we won't bother since our ack method is NULL.
	 */

	if (!desc->depth) {
		kstat_cpu(cpu).irqs[irq]++;

		action = desc->action;
		if (action) {
			int ret;
			int status = 0;
			int retval = 0;

			local_irq_enable();

			do {
				/* Call the ISR with cpu interrupts enabled */
				ret = action->handler(irq, action->dev_id);
				if (ret == IRQ_HANDLED)
					status |= action->flags;
				retval |= ret;
				action = action->next;
			} while (action);

			if (status & IRQF_SAMPLE_RANDOM)
				add_interrupt_randomness(irq);

			local_irq_disable();

			if (retval != IRQ_HANDLED)
				printk(KERN_ERR "ISR for TWL4030 module"
					" irq %d can't handle interrupt\n",
					irq);

			/*
			 * Here is where we should call the unmask method, but
			 * again we won't bother since it is NULL.
			 */
		} else
			printk(KERN_CRIT "TWL4030 module irq %d has no ISR"
					" but can't be masked!\n", irq);
	} else
		printk(KERN_CRIT "TWL4030 module irq %d is disabled but can't"
				" be masked!\n", irq);
}

/*
 * twl4030_irq_thread() runs as a kernel thread.  It queries the twl4030
 * interrupt controller to see which modules are generating interrupt requests
 * and then calls the desc->handle method for each module requesting service.
 */
static int twl4030_irq_thread(void *data)
{
	int irq = (int)data;
	irq_desc_t *desc = irq_desc + irq;
	static unsigned i2c_errors;
	const static unsigned max_i2c_errors = 100;

	daemonize("twl4030-irq");
	current->flags |= PF_NOFREEZE;

	while (!kthread_should_stop()) {
		int ret;
		int module_irq;
		u8 pih_isr;

		wait_for_completion_interruptible(&irq_event);

		ret = twl4030_i2c_read_u8(TWL4030_MODULE_PIH, &pih_isr,
					  REG_PIH_ISR_P1);
		if (ret) {
			printk(KERN_WARNING "I2C error %d while reading TWL4030"
					" PIH ISR register.\n", ret);
			if (++i2c_errors >= max_i2c_errors) {
				printk(KERN_ERR "Maximum I2C error count"
						" exceeded.  Terminating %s.\n",
						__func__);
				break;
			}
			continue;
		}

		for (module_irq = TWL4030_IRQ_BASE; 0 != pih_isr;
			 pih_isr >>= 1, module_irq++) {
			if (pih_isr & 0x1) {
				irq_desc_t *d = irq_desc + module_irq;

				local_irq_disable();

				d->handle_irq(module_irq, d);

				local_irq_enable();
			}
		}

		desc->chip->unmask(irq);
	}

	return 0;
}

/*
 * do_twl4030_irq() is the desc->handle method for the twl4030 interrupt.
 * This is a chained interrupt, so there is no desc->action method for it.
 * Now we need to query the interrupt controller in the twl4030 to determine
 * which module is generating the interrupt request.  However, we can't do i2c
 * transactions in interrupt context, so we must defer that work to a kernel
 * thread.  All we do here is acknowledge and mask the interrupt and wakeup
 * the kernel thread.
 */
static void do_twl4030_irq(unsigned int irq, irq_desc_t *desc)
{
	const unsigned int cpu = smp_processor_id();

	/*
	 * Earlier this was desc->triggered = 1;
	 */
	desc->status |= IRQ_LEVEL;

	/*
	 * Acknowledge, clear _AND_ disable the interrupt.
	 */
	desc->chip->ack(irq);

	if (!desc->depth) {
		kstat_cpu(cpu).irqs[irq]++;

		complete(&irq_event);
	}
}

/* attach a client to the adapter */
static int __init twl4030_detect_client(struct i2c_adapter *adapter,
					unsigned char sid)
{
	int err = 0;
	struct twl4030_client *twl;

	if (unlikely(sid >= TWL4030_NUM_SLAVES)) {
		pr_err("sid[%d] > MOD_LAST[%d]\n", sid, TWL4030_NUM_SLAVES);
		return -EPERM;
	}

	/* Check basic functionality */
	err = i2c_check_functionality(adapter,
			I2C_FUNC_SMBUS_WORD_DATA
			| I2C_FUNC_SMBUS_WRITE_BYTE);
	if (!err) {
		pr_err("SlaveID=%d functionality check failed\n", sid);
		return err;
	}
	twl = &twl4030_modules[sid];
	if (unlikely(twl->inuse)) {
		pr_err("Client %s is already in use\n", twl->client_name);
		return -EPERM;
	}

	memset(&twl->client, 0, sizeof(struct i2c_client));

	twl->client.addr	= twl->address;
	twl->client.adapter	= adapter;
	twl->client.driver	= &twl4030_driver;

	memcpy(&twl->client.name, twl->client_name,
			sizeof(TWL_CLIENT_STRING) + 1);

	pr_info("TWL4030: TRY attach Slave %s on Adapter %s [%x]\n",
				twl->client_name, adapter->name, err);

	err = i2c_attach_client(&twl->client);
	if (err) {
		pr_err("Couldn't attach Slave %s on Adapter"
		       "%s [%x]\n", twl->client_name, adapter->name, err);
	} else {
		twl->inuse = TWL_CLIENT_USED;
		mutex_init(&twl->xfer_lock);
	}

	return err;
}

/* adapter callback */
static int __init twl4030_attach_adapter(struct i2c_adapter *adapter)
{
	int i;
	int ret = 0;
	static int twl_i2c_adapter = 1;

	for (i = 0; i < TWL4030_NUM_SLAVES; i++) {
		/* Check if I need to hook on to this adapter or not */
		if (twl4030_modules[i].adapter_index == twl_i2c_adapter) {
			ret = twl4030_detect_client(adapter, i);
			if (ret)
				goto free_client;
		}
	}
	twl_i2c_adapter++;

	/*
	 * Check if the PIH module is initialized, if yes, then init
	 * the T2 Interrupt subsystem
	 */
	if ((twl4030_modules[twl4030_map[TWL4030_MODULE_PIH].sid].inuse ==
		TWL_CLIENT_USED) && (twl_irq_used != USED)) {
		twl_init_irq();
		twl_irq_used = USED;
	}
	return 0;

free_client:
	pr_err("TWL_CLIENT(Idx=%d] registration failed[0x%x]\n", i, ret);

	/* ignore current slave..it never got registered */
	i--;
	while (i >= 0) {
		/* now remove all those from the current adapter... */
		if (twl4030_modules[i].adapter_index == twl_i2c_adapter)
			(void)twl4030_detach_client(&twl4030_modules[i].client);
		i--;
	}
	return ret;
}

/* adapter's callback */
static int twl4030_detach_client(struct i2c_client *client)
{
	int err;
	err = i2c_detach_client(client);
	if (err) {
		pr_err("Client detach failed\n");
		return err;
	}
	return 0;
}

static struct task_struct * __init start_twl4030_irq_thread(int irq)
{
	struct task_struct *thread;

	init_completion(&irq_event);
	thread = kthread_run(twl4030_irq_thread, (void *)irq,
			     "twl4030 irq %d", irq);
	if (!thread)
		pr_err("%s: could not create twl4030 irq %d thread!\n",
		       __func__, irq);

	return thread;
}

/*
 * These three functions should be part of Voltage frame work
 * added here to complete the functionality for now.
 */
static int __init protect_pm_master(void)
{
	int e = 0;

	e = twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, KEY_LOCK,
			R_PROTECT_KEY);
	return e;
}

static int __init unprotect_pm_master(void)
{
	int e = 0;

	e |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, KEY_UNLOCK1,
			R_PROTECT_KEY);
	e |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, KEY_UNLOCK2,
			R_PROTECT_KEY);
	return e;
}

static int __init power_companion_init(void)
{
	struct clk *osc;
	u32 rate;
	u8 ctrl = HFCLK_FREQ_26_MHZ;
	int e = 0;

	if (cpu_is_omap2430())
		osc = clk_get(NULL, "osc_ck");
	else
		osc = clk_get(NULL, "osc_sys_ck");
	if (IS_ERR(osc)) {
		printk(KERN_WARNING "Skipping twl3040 internal clock init and "
				"using bootloader value (unknown osc rate)\n");
		return 0;
	}

	rate = clk_get_rate(osc);
	clk_put(osc);

	switch (rate) {
	case 19200000 : ctrl = HFCLK_FREQ_19p2_MHZ; break;
	case 26000000 : ctrl = HFCLK_FREQ_26_MHZ; break;
	case 38400000 : ctrl = HFCLK_FREQ_38p4_MHZ; break;
	}

	ctrl |= HIGH_PERF_SQ;
	e |= unprotect_pm_master();
	/* effect->MADC+USB ck en */
	e |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, ctrl, R_CFG_BOOT);
	e |= protect_pm_master();

	return e;
}

/**
 * twl4030_i2c_clear_isr - clear TWL4030 SIH ISR regs via read + write
 * @mod_no: TWL4030 module number
 * @reg: register index to clear
 * @cor: value of the <module>_SIH_CTRL.COR bit (1 or 0)
 *
 * Either reads (cor == 1) or writes (cor == 0) to a TWL4030 interrupt
 * status register to ensure that any prior interrupts are cleared.
 * Returns the status from the I2C read operation.
 */
static int __init twl4030_i2c_clear_isr(u8 mod_no, u8 reg, u8 cor)
{
	u8 tmp;

	return (cor) ? twl4030_i2c_read_u8(mod_no, &tmp, reg) :
		twl4030_i2c_write_u8(mod_no, 0xff, reg);
}

/**
 * twl4030_read_cor_bit - are TWL module ISRs cleared by reads or writes?
 * @mod_no: TWL4030 module number
 * @reg: register index to clear
 *
 * Returns 1 if the TWL4030 SIH interrupt status registers (ISRs) for
 * the specified TWL module are cleared by reads, or 0 if cleared by
 * writes.
 */
static int twl4030_read_cor_bit(u8 mod_no, u8 reg)
{
	u8 tmp = 0;

	WARN_ON(twl4030_i2c_read_u8(mod_no, &tmp, reg) < 0);

	tmp &= TWL4030_SIH_CTRL_COR_MASK;
	tmp >>= __ffs(TWL4030_SIH_CTRL_COR_MASK);

	return tmp;
}

/**
 * twl4030_mask_clear_intrs - mask and clear all TWL4030 interrupts
 * @t: pointer to twl4030_mod_iregs array
 * @t_sz: ARRAY_SIZE(t) (starting at 1)
 *
 * Mask all TWL4030 interrupt mask registers (IMRs) and clear all
 * interrupt status registers (ISRs).  No return value, but will WARN if
 * any I2C operations fail.
 */
static void __init twl4030_mask_clear_intrs(const struct twl4030_mod_iregs *t,
					    const u8 t_sz)
{
	int i, j;

	/*
	 * N.B. - further efficiency is possible here.  Eight I2C
	 * operations on BCI and GPIO modules are avoidable if I2C
	 * burst read/write transactions were implemented.  Would
	 * probably save about 1ms of boot time and a small amount of
	 * power.
	 */
	for (i = 0; i < t_sz; i++) {
		const struct twl4030_mod_iregs tmr = t[i];
		int cor;

		/* Are ISRs cleared by reads or writes? */
		cor = twl4030_read_cor_bit(tmr.mod_no, tmr.sih_ctrl);
		WARN_ON(cor < 0);

		for (j = 0; j < tmr.reg_cnt; j++) {

			/* Mask interrupts at the TWL4030 */
			WARN_ON(twl4030_i2c_write_u8(tmr.mod_no, 0xff,
						     tmr.imrs[j]) < 0);

			/* Clear TWL4030 ISRs */
			WARN_ON(twl4030_i2c_clear_isr(tmr.mod_no,
						      tmr.isrs[j], cor) < 0);
		}
	}

	return;
}


static void twl_init_irq(void)
{
	int	i;
	int	res = 0;
	char	*msg = "Unable to register interrupt subsystem";
	unsigned int irq_num;

	/*
	 * Mask and clear all TWL4030 interrupts since initially we do
	 * not have any TWL4030 module interrupt handlers present
	 */
	twl4030_mask_clear_intrs(twl4030_mod_regs,
				 ARRAY_SIZE(twl4030_mod_regs));

	/* install an irq handler for each of the PIH modules */
	for (i = TWL4030_IRQ_BASE; i < TWL4030_IRQ_END; i++) {
		set_irq_chip(i, &twl4030_irq_chip);
		set_irq_handler(i, do_twl4030_module_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	irq_num = (cpu_is_omap2430()) ? INT_24XX_SYS_NIRQ : INT_34XX_SYS_NIRQ;

	/* install an irq handler to demultiplex the TWL4030 interrupt */
	set_irq_data(irq_num, start_twl4030_irq_thread(irq_num));
	set_irq_type(irq_num, IRQ_TYPE_EDGE_FALLING);
	set_irq_chained_handler(irq_num, do_twl4030_irq);

	res = power_companion_init();
	if (res < 0)
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
}

static int __init twl4030_init(void)
{
	return i2c_add_driver(&twl4030_driver);
}

static void __exit twl4030_exit(void)
{
	i2c_del_driver(&twl4030_driver);
	twl_irq_used = FREE;
}

subsys_initcall(twl4030_init);
module_exit(twl4030_exit);

MODULE_ALIAS("i2c:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_DESCRIPTION("I2C Core interface for TWL4030");
MODULE_LICENSE("GPL");
