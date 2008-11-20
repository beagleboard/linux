/*
 * drivers/input/keyboard/omap-twl4030keypad.c
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Copyright (C) 2008 Nokia Corporation
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
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c/twl4030.h>
#include <linux/irq.h>
#include <mach/keypad.h>

#include "twl4030-keypad.h"

#define PTV_PRESCALER		4

#define MAX_ROWS		8 /* TWL4030 hardlimit */

/* Global variables */

struct omap_keypad {
	int		*keymap;
	unsigned int	keymapsize;
	u16		kp_state[MAX_ROWS];
	int		n_rows;
	int		n_cols;
	int		irq;

	struct device	*dbg_dev;
	struct input_dev *omap_twl4030kp;

	/* sync read/write */
	struct mutex	mutex;
};

static int twl4030_kpread(struct omap_keypad *kp,
		u32 module, u8 *data, u32 reg, u8 num_bytes)
{
	int ret;

	ret = twl4030_i2c_read(module, data, reg, num_bytes);
	if (ret < 0) {
		dev_warn(kp->dbg_dev,
			"Couldn't read TWL4030: %X - ret %d[%x]\n",
			 reg, ret, ret);
		return ret;
	}
	return ret;
}

static int twl4030_kpwrite_u8(struct omap_keypad *kp,
		u32 module, u8 data, u32 reg)
{
	int ret;

	ret = twl4030_i2c_write_u8(module, data, reg);
	if (ret < 0) {
		dev_warn(kp->dbg_dev,
			"Could not write TWL4030: %X - ret %d[%x]\n",
			 reg, ret, ret);
		return ret;
	}
	return ret;
}

static int omap_kp_find_key(struct omap_keypad *kp, int col, int row)
{
	int i, rc;

	rc = KEY(col, row, 0);
	for (i = 0; i < kp->keymapsize; i++)
		if ((kp->keymap[i] & ROWCOL_MASK) == rc)
			return kp->keymap[i] & (KEYNUM_MASK | KEY_PERSISTENT);

	return -EINVAL;
}

static inline u16 omap_kp_col_xlate(struct omap_keypad *kp, u8 col)
{
	/* If all bits in a row are active for all coloumns then
	 * we have that row line connected to gnd. Mark this
	 * key on as if it was on matrix position n_cols (ie
	 * one higher than the size of the matrix).
	 */
	if (col == 0xFF)
		return 1 << kp->n_cols;
	else
		return col & ((1 << kp->n_cols) - 1);
}

static int omap_kp_read_kp_matrix_state(struct omap_keypad *kp, u16 *state)
{
	u8 new_state[MAX_ROWS];
	int row;
	int ret = twl4030_kpread(kp, TWL4030_MODULE_KEYPAD,
				 new_state, KEYP_FULL_CODE_7_0, kp->n_rows);
	if (ret >= 0) {
		for (row = 0; row < kp->n_rows; row++)
			state[row] = omap_kp_col_xlate(kp, new_state[row]);
	}
	return ret;
}

static int omap_kp_is_in_ghost_state(struct omap_keypad *kp, u16 *key_state)
{
	int i;
	u16 check = 0;

	for (i = 0; i < kp->n_rows; i++) {
		u16 col = key_state[i];

		if ((col & check) && hweight16(col) > 1)
			return 1;
		check |= col;
	}

	return 0;
}

static void twl4030_kp_scan(struct omap_keypad *kp, int release_all)
{
	u16 new_state[MAX_ROWS];
	int col, row;

	if (release_all)
		memset(new_state, 0, sizeof(new_state));
	else {
		/* check for any changes */
		int ret = omap_kp_read_kp_matrix_state(kp, new_state);
		if (ret < 0)	/* panic ... */
			return;

		if (omap_kp_is_in_ghost_state(kp, new_state))
			return;
	}

	mutex_lock(&kp->mutex);

	/* check for changes and print those */
	for (row = 0; row < kp->n_rows; row++) {
		int changed = new_state[row] ^ kp->kp_state[row];

		if (!changed)
			continue;

		for (col = 0; col < kp->n_cols; col++) {
			int key;

			if (!(changed & (1 << col)))
				continue;

			dev_dbg(kp->dbg_dev, "key [%d:%d] %s\n", row, col,
				(new_state[row] & (1 << col)) ?
				"press" : "release");

			key = omap_kp_find_key(kp, col, row);
			if (key < 0)
				dev_warn(kp->dbg_dev,
					"Spurious key event %d-%d\n",
					 col, row);
			else if (key & KEY_PERSISTENT)
				continue;
			else
				input_report_key(kp->omap_twl4030kp, key,
						 new_state[row] & (1 << col));
		}
		kp->kp_state[row] = new_state[row];
	}

	mutex_unlock(&kp->mutex);
}

/*
 * Keypad interrupt handler
 */
static irqreturn_t do_kp_irq(int irq, void *_kp)
{
	struct omap_keypad *kp = _kp;
	u8 reg;
	int ret;

#ifdef CONFIG_LOCKDEP
	/* WORKAROUND for lockdep forcing IRQF_DISABLED on us, which
	 * we don't want and can't tolerate.  Although it might be
	 * friendlier not to borrow this thread context...
	 */
	local_irq_enable();
#endif

	/* Read & Clear TWL4030 pending interrupt */
	ret = twl4030_kpread(kp, TWL4030_MODULE_KEYPAD, &reg, KEYP_ISR1, 1);

	/* Release all keys if I2C has gone bad or
	 * the KEYP has gone to idle state */
	if ((ret >= 0) && (reg & KEYP_IMR1_KP))
		twl4030_kp_scan(kp, 0);
	else
		twl4030_kp_scan(kp, 1);

	return IRQ_HANDLED;
}

/*
 * Registers keypad device with input sub system
 * and configures TWL4030 keypad registers
 */
static int __init omap_kp_probe(struct platform_device *pdev)
{
	u8 reg;
	int i;
	int ret = 0;
	struct omap_keypad *kp;
	struct twl4030_keypad_data *pdata = pdev->dev.platform_data;

	kp = kzalloc(sizeof(*kp), GFP_KERNEL);
	if (!kp)
		return -ENOMEM;

	if (!pdata->rows || !pdata->cols || !pdata->keymap) {
		dev_err(&pdev->dev, "No rows, cols or keymap from pdata\n");
		kfree(kp);
		return -EINVAL;
	}

	dev_set_drvdata(&pdev->dev, kp);

	/* Get the debug Device */
	kp->dbg_dev = &pdev->dev;

	kp->omap_twl4030kp = input_allocate_device();
	if (!kp->omap_twl4030kp) {
		kfree(kp);
		return -ENOMEM;
	}

	mutex_init(&kp->mutex);

	kp->keymap = pdata->keymap;
	kp->keymapsize = pdata->keymapsize;
	kp->n_rows = pdata->rows;
	kp->n_cols = pdata->cols;
	kp->irq = platform_get_irq(pdev, 0);

	/* setup input device */
	set_bit(EV_KEY, kp->omap_twl4030kp->evbit);

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		set_bit(EV_REP, kp->omap_twl4030kp->evbit);

	for (i = 0; i < kp->keymapsize; i++)
		set_bit(kp->keymap[i] & KEYNUM_MASK,
				kp->omap_twl4030kp->keybit);

	kp->omap_twl4030kp->name	= "omap_twl4030keypad";
	kp->omap_twl4030kp->phys	= "omap_twl4030keypad/input0";
	kp->omap_twl4030kp->dev.parent	= &pdev->dev;

	kp->omap_twl4030kp->id.bustype	= BUS_HOST;
	kp->omap_twl4030kp->id.vendor	= 0x0001;
	kp->omap_twl4030kp->id.product	= 0x0001;
	kp->omap_twl4030kp->id.version	= 0x0003;

	kp->omap_twl4030kp->keycode	= kp->keymap;
	kp->omap_twl4030kp->keycodesize	= sizeof(unsigned int);
	kp->omap_twl4030kp->keycodemax	= kp->keymapsize;

	ret = input_register_device(kp->omap_twl4030kp);
	if (ret < 0) {
		dev_err(kp->dbg_dev,
			"Unable to register twl4030 keypad device\n");
		goto err2;
	}

	/* Disable auto-repeat */
	reg = KEYP_CTRL_NOAUTORPT;
	ret = twl4030_kpwrite_u8(kp, TWL4030_MODULE_KEYPAD, reg, KEYP_CTRL);
	if (ret < 0)
		goto err3;

	/* Enable TO rising and KP rising and falling edge detection */
	reg = KEYP_EDR_KP_BOTH | KEYP_EDR_TO_RISING;
	ret = twl4030_kpwrite_u8(kp, TWL4030_MODULE_KEYPAD, reg, KEYP_EDR);
	if (ret < 0)
		goto err3;

	/* Set PTV prescaler Field */
	reg = (PTV_PRESCALER << KEYP_LK_PTV_PTV_SHIFT);
	ret = twl4030_kpwrite_u8(kp, TWL4030_MODULE_KEYPAD, reg, KEYP_LK_PTV);
	if (ret < 0)
		goto err3;

	/* Set key debounce time to 20 ms */
	i = KEYP_PERIOD_US(20000, PTV_PRESCALER);
	ret = twl4030_kpwrite_u8(kp, TWL4030_MODULE_KEYPAD, i, KEYP_DEB);
	if (ret < 0)
		goto err3;

	/* Set timeout period to 100 ms */
	i = KEYP_PERIOD_US(200000, PTV_PRESCALER);
	ret = twl4030_kpwrite_u8(kp, TWL4030_MODULE_KEYPAD,
				 (i & 0xFF), KEYP_TIMEOUT_L);
	if (ret < 0)
		goto err3;

	ret = twl4030_kpwrite_u8(kp, TWL4030_MODULE_KEYPAD,
				 (i >> 8), KEYP_TIMEOUT_H);
	if (ret < 0)
		goto err3;

	/* Enable Clear-on-Read */
	reg = KEYP_SIH_CTRL_COR | KEYP_SIH_CTRL_PEND_DIS;
	ret = twl4030_kpwrite_u8(kp, TWL4030_MODULE_KEYPAD,
				 reg, KEYP_SIH_CTRL);
	if (ret < 0)
		goto err3;

	/*
	 * This ISR will always execute in kernel thread context because of
	 * the need to access the TWL4030 over the I2C bus.
	 */
	ret = request_irq(kp->irq, do_kp_irq, 0, pdev->name, kp);
	if (ret < 0) {
		dev_info(kp->dbg_dev, "request_irq failed for irq no=%d\n",
			kp->irq);
		goto err3;
	} else {
		/* Enable KP and TO interrupts now. */
		reg = ~(KEYP_IMR1_KP | KEYP_IMR1_TO);
		ret = twl4030_kpwrite_u8(kp, TWL4030_MODULE_KEYPAD,
					 reg, KEYP_IMR1);
		if (ret < 0)
			goto err5;
	}

	ret = omap_kp_read_kp_matrix_state(kp, kp->kp_state);
	if (ret < 0)
		goto err4;

	return ret;
err5:
	/* mask all events - we don't care about the result */
	(void) twl4030_kpwrite_u8(kp, TWL4030_MODULE_KEYPAD, 0xff, KEYP_IMR1);
err4:
	free_irq(kp->irq, NULL);
err3:
	input_unregister_device(kp->omap_twl4030kp);
err2:
	input_free_device(kp->omap_twl4030kp);

	return -ENODEV;
}

static int omap_kp_remove(struct platform_device *pdev)
{
	struct omap_keypad *kp = dev_get_drvdata(&pdev->dev);

	free_irq(kp->irq, kp);
	input_unregister_device(kp->omap_twl4030kp);
	kfree(kp);

	return 0;
}


static struct platform_driver omap_kp_driver = {
	.probe		= omap_kp_probe,
	.remove		= __devexit_p(omap_kp_remove),
	.driver		= {
		.name	= "twl4030_keypad",
		.owner	= THIS_MODULE,
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
MODULE_ALIAS("platform:twl4030_keypad");
MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP TWL4030 Keypad Driver");
MODULE_LICENSE("GPL");
