/*
 * drivers/input/keyboard/omap-twl4030keypad.c
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * Code re-written for 2430SDP by:
 * Syed Mohammed Khasim <x0khasim@ti.com>
 *
 * Initial Code:
 * Manjunatha G K <manjugk@ti.com>
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
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <asm/irq.h>
#include <asm/arch/keypad.h>
#include <asm/arch/twl4030.h>
#include "twl4030-keypad.h"

#define OMAP_TWL4030KP_LOG_LEVEL	0

#define KEY(col, row, val)	(((col) << 28) | ((row) << 24) | (val))
#define NUM_ROWS		5
#define NUM_COLS		6

#define ROW_MASK		((1<<NUM_ROWS)-1)

#define SCAN_RATE		HZ/20
#define KEYNUM_MASK		0xFF000000
#define ROWCOL_MASK		0x00FFFFFF

static char *switch_name[NUM_ROWS][NUM_COLS] = {
	{"S2_L", "S2_D", "S2_S", "S3", "S4", "S23"},
	{"S2_R", "S2_U", "S5", "S6", "S7", "S24"},
	{"S8", "S9", "S10", "S11", "S12", "S25"},
	{"S13", "S14", "S15", "S16", "S17", "S26"},
	{"S18", "S19", "S20", "S21", "S22", "S27"},
};

/* Global variables */
static int *keymap;
static unsigned char kp_state[NUM_ROWS];
static struct device * dbg_dev;

/* Function Templates */
static struct input_dev *omap_twl4030kp;
struct timer_list kptimer;
static void omap_kp_timer(unsigned long);
static void twl4030_kp_scan(void);
struct work_struct timer_work;
static void twl4030_timer_work(struct work_struct *unused);

static int twl4030_kpread_u8(u32 module, u8 * data, u32 reg)
{
	int ret;

	ret = twl4030_i2c_read_u8(module, data, reg);
	if (ret < 0) {
		dev_warn(dbg_dev, "Couldn't read TWL4030 register %X - ret %d[%x]\n",
			 reg, ret, ret);
		return ret;
	}
	return ret;
}

static int twl4030_kpwrite_u8(u32 module, u8 data, u32 reg)
{
	int ret;

	ret = twl4030_i2c_write_u8(module, data, reg);
	if (ret < 0) {
		dev_warn(dbg_dev, "Could not write TWL4030 register %X - ret %d[%x]\n",
			 reg, ret, ret);
		return ret;
	}
	return ret;
}

static inline int omap_kp_find_key(int col, int row)
{
	int i, key;

	key = KEY(col, row, 0);
	for (i = 0; keymap[i] != 0; i++)
		if ((keymap[i] & KEYNUM_MASK) == key)
			return keymap[i] & ROWCOL_MASK;

	return -EINVAL;
}

static void twl4030_kp_scan(void)
{
	unsigned char new_state[NUM_ROWS], changed, key_down = 0;
	u8 col, row, spurious = 0;
	u8 code_reg = REG_FULL_CODE_7_0;
	int ret;

	/* check for any changes */
	ret =
		twl4030_i2c_read(TWL4030_MODULE_KEYPAD, new_state, code_reg,
				 NUM_ROWS);
	if (ret < 0)
		dev_warn(dbg_dev, "Could not read TWL4030 register %X - ret %d[%x]\n",
			 code_reg, ret, ret);

	/* check for changes and print those */
	for (row = 0; row < NUM_ROWS; row++) {
		changed = new_state[row] ^ kp_state[row];
		key_down |= new_state[row];

		if (changed == 0)
			continue;

		for (col = 0; col < NUM_COLS; col++) {
			int key;

			if (!(changed & (1 << col)))
				continue;

			dev_dbg(dbg_dev, "key %s %s\n", switch_name[row][col],
				(new_state[row] & (1 << col)) ?
				"press" : "release");

			key = omap_kp_find_key(col, row);
			if (key < 0) {
				dev_warn(dbg_dev, "omap-kp: Spurious key event %d-%d\n",
					 col, row);
				/* We scan again after a couple of seconds */
				spurious = 1;
				continue;
			}
			input_report_key(omap_twl4030kp, key,
					 new_state[row] & (1 << col));
		}
	}
	if (key_down) {
		/*
		 * some key is pressed - keep irq disabled and use timer
		 * to poll for key release
		 */
		if (spurious)
			mod_timer(&kptimer, jiffies + SCAN_RATE * 2);
		else
			mod_timer(&kptimer, jiffies + SCAN_RATE);
	}
	memcpy(kp_state, new_state, sizeof(kp_state));
}

static void twl4030_timer_work(struct work_struct *unused)
{
	twl4030_kp_scan();
}

void omap_kp_timer(unsigned long data)
{
	schedule_work(&timer_work);
}

/*
 * Keypad interrupt handler
 */
static irqreturn_t do_kp_irq(int irq, void *dev_id)
{
	u8 reg;
	int ret;

	/* Mask keypad interrupts */
	reg = KEYP_IMR1_MASK;
	ret = twl4030_kpwrite_u8(TWL4030_MODULE_KEYPAD, reg, REG_KEYP_IMR1);

	/*
	 * Scan keypad for any changes
	 * in keypad matrix.
	 */
	twl4030_kp_scan();

	/* Clear TWL4030 PIH interrupt */
	ret = twl4030_kpread_u8(TWL4030_MODULE_KEYPAD, &reg, REG_KEYP_ISR1);

	/* Enable interrupts */
	reg = KEYP_IMR1_UNMASK;
	ret = twl4030_kpwrite_u8(TWL4030_MODULE_KEYPAD, reg, REG_KEYP_IMR1);

	return IRQ_HANDLED;
}

/*
 * Registers keypad device with input sub system
 * and configures TWL4030 keypad registers
 *
 */
static int __init omap_kp_probe(struct platform_device *pdev)
{
	u8 reg, i;
	u8 code_reg = REG_FULL_CODE_7_0;
	int ret = 0;
	struct omap_kp_platform_data *pdata =  pdev->dev.platform_data;

	/* Get the debug Device */
	dbg_dev = &(pdev->dev);

	if (!pdata->rows || !pdata->cols || !pdata->keymap) {
		dev_err(dbg_dev, "No rows, cols or keymap from pdata\n");
		return -EINVAL;
	}

	omap_twl4030kp = input_allocate_device();
	if (omap_twl4030kp == NULL)
		return -ENOMEM;

	keymap = pdata->keymap;

	/* setup input device */
	set_bit(EV_KEY, omap_twl4030kp->evbit);

	/* Enable auto repeat feature of Linux input subsystem */
	set_bit(EV_REP, omap_twl4030kp->evbit);

	for (i = 0; keymap[i] != 0; i++)
		set_bit(keymap[i] & 0x00ffffff, omap_twl4030kp->keybit);

	omap_twl4030kp->name		= "omap_twl4030keypad";
	omap_twl4030kp->phys		= "omap_twl4030keypad/input0";
	omap_twl4030kp->dev.parent	= &pdev->dev;

	omap_twl4030kp->id.bustype	= BUS_HOST;
	omap_twl4030kp->id.vendor	= 0x0001;
	omap_twl4030kp->id.product	= 0x0001;
	omap_twl4030kp->id.version	= 0x0003;

	omap_twl4030kp->keycode		= keymap;
	omap_twl4030kp->keycodesize	= sizeof(unsigned int);
	omap_twl4030kp->keycodemax	= pdata->keymapsize;

	ret = input_register_device(omap_twl4030kp);
	if (ret < 0) {
		dev_err(dbg_dev, "Unable to register twl4030 keypad device\n");
		goto err2;
	}

	setup_timer(&kptimer,omap_kp_timer,(unsigned long) omap_twl4030kp);

	/*
	 * Since keypad driver uses I2C for reading
	 * twl4030 keypad registers, tasklets cannot
	 * be used.
	 */
	INIT_WORK(&timer_work, twl4030_timer_work);

	reg = KEYP_CTRL_REG_MASK_NOAUTORPT;
	ret = twl4030_kpwrite_u8(TWL4030_MODULE_KEYPAD, reg,
						REG_KEYP_CTRL_REG);
	if (ret < 0)
		goto err3;

	/* Set all events to Falling Edge detection */
	reg = KEYP_EDR_MASK;
	ret = twl4030_kpwrite_u8(TWL4030_MODULE_KEYPAD, reg, REG_KEYP_EDR);
	if (ret < 0)
		goto err3;

	/* Set Pre Scalar Field PTV to 4 */
	reg = BIT_LK_PTV_REG_PTV_MASK & (BIT_PTV_REG_PTV4 << BIT_LK_PTV_REG_PTV);

	ret = twl4030_kpwrite_u8(TWL4030_MODULE_KEYPAD, reg, REG_LK_PTV_REG);
	if (ret < 0)
		goto err3;

	/*
	 * Set key debounce time to 10 ms using equation
	 * Tint = Tclk * (LOAD_TIM+1) * 2^(PTV+1)
	 * Where Tclk = 31.25 us ( since kbd_if_clk is 32KHz)
	 * PTV = 4 for all the operations.
	 */
	ret = twl4030_kpwrite_u8(TWL4030_MODULE_KEYPAD, 0x3f,
						REG_KEY_DEB_REG);
	if (ret < 0)
		goto err3;

	/* Set SIH Ctrl register */
	reg = KEYP_SIH_CTRL_MASK;
	ret = twl4030_kpwrite_u8(TWL4030_MODULE_KEYPAD, reg,
						REG_KEYP_SIH_CTRL);
	if (ret < 0)
		goto err3;

	/*
	 * This ISR will always execute in kernel thread context because of
	 * the need to access the TWL4030 over the I2C bus.
	 */
	ret = request_irq(TWL4030_MODIRQ_KEYPAD, do_kp_irq,
		IRQF_DISABLED, "TWL4030 Keypad", omap_twl4030kp);
	if (ret < 0) {
		dev_info(dbg_dev, "request_irq failed for irq no=%d\n",
			TWL4030_MODIRQ_KEYPAD);
		goto err3;
	} else {
		/* Enable keypad module interrupts now. */
		reg = KEYP_IMR1_UNMASK;
		ret = twl4030_kpwrite_u8(TWL4030_MODULE_KEYPAD, reg,
						REG_KEYP_IMR1);
		if (ret < 0) {
			/* mask all events - dont care abt result */
			(void)twl4030_kpwrite_u8(TWL4030_MODULE_KEYPAD, 0xff,
						 REG_KEYP_IMR1);
			goto err4;
		}
	}

	/* Read initial state of keypad matrix. */
	ret = twl4030_i2c_read(TWL4030_MODULE_KEYPAD, kp_state, code_reg,
		NUM_ROWS);
	if (ret < 0) {
		dev_warn(dbg_dev, "Could not read TWL4030 register %X - ret %d[%x]\n",
			 reg, ret, ret);
		goto err4;
	}
	return (ret);
err4:
	free_irq(TWL4030_MODIRQ_KEYPAD, NULL);
err3:
	input_unregister_device(omap_twl4030kp);
err2:
	input_free_device(omap_twl4030kp);
	return -ENODEV;
}

static int omap_kp_remove(struct platform_device *pdev)
{
	free_irq(TWL4030_MODIRQ_KEYPAD, NULL);
	del_timer_sync(&kptimer);

	input_unregister_device(omap_twl4030kp);
	return 0;
}


static struct platform_driver omap_kp_driver = {
	.probe		= omap_kp_probe,
	.remove		= omap_kp_remove,
	.driver		= {
		.name	= "omap_twl4030keypad",
	},
};

/*
 * OMAP TWL4030 Keypad init
 */
static int __devinit omap_kp_init(void)
{
	return platform_driver_register(&omap_kp_driver);
}

static void __exit omap_kp_exit(void)
{
	platform_driver_unregister(&omap_kp_driver);
}

module_init(omap_kp_init);
module_exit(omap_kp_exit);
MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP TWL4030 Keypad Driver");
MODULE_LICENSE("GPL");
