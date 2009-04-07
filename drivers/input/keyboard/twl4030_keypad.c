/*
 * twl4030_keypad.c - driver for 8x8 keypad controller in twl4030 chips
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl4030.h>


/*
 * The TWL4030 family chips include a keypad controller that supports
 * up to an 8x8 switch matrix.  The controller can issue system wakeup
 * events, since it uses only the always-on 32KiHz oscillator, and has
 * an internal state machine that decodes pressed keys, including
 * multi-key combinations.
 *
 * This driver lets boards define what keycodes they wish to report for
 * which scancodes, as part of the "struct twl4030_keypad_data" used in
 * the probe() routine.
 *
 * See the TPS65950 documentation; that's the general availability
 * version of the TWL5030 second generation part.
 */
#define MAX_ROWS		8	/* TWL4030 hard limit */

struct twl4030_keypad {
	unsigned	*keymap;
	unsigned int	keymapsize;
	u16		kp_state[MAX_ROWS];
	unsigned	n_rows;
	unsigned	n_cols;
	unsigned	irq;

	struct device	*dbg_dev;
	struct input_dev *input;
};

#define ROWCOL_MASK	KEY(0xf, 0xf, 0)
#define KEYNUM_MASK	~PERSISTENT_KEY(0xf, 0xf)

/*----------------------------------------------------------------------*/

/* arbitrary prescaler value 0..7 */
#define PTV_PRESCALER			4

/* Register Offsets */
#define KEYP_CTRL			0x00
#define KEYP_DEB			0x01
#define KEYP_LONG_KEY			0x02
#define KEYP_LK_PTV			0x03
#define KEYP_TIMEOUT_L			0x04
#define KEYP_TIMEOUT_H			0x05
#define KEYP_KBC			0x06
#define KEYP_KBR			0x07
#define KEYP_SMS			0x08
#define KEYP_FULL_CODE_7_0		0x09	/* row 0 column status */
#define KEYP_FULL_CODE_15_8		0x0a	/* ... row 1 ... */
#define KEYP_FULL_CODE_23_16		0x0b
#define KEYP_FULL_CODE_31_24		0x0c
#define KEYP_FULL_CODE_39_32		0x0d
#define KEYP_FULL_CODE_47_40		0x0e
#define KEYP_FULL_CODE_55_48		0x0f
#define KEYP_FULL_CODE_63_56		0x10
#define KEYP_ISR1			0x11
#define KEYP_IMR1			0x12
#define KEYP_ISR2			0x13
#define KEYP_IMR2			0x14
#define KEYP_SIR			0x15
#define KEYP_EDR			0x16	/* edge triggers */
#define KEYP_SIH_CTRL			0x17

/* KEYP_CTRL_REG Fields */
#define KEYP_CTRL_SOFT_NRST		BIT(0)
#define KEYP_CTRL_SOFTMODEN		BIT(1)
#define KEYP_CTRL_LK_EN			BIT(2)
#define KEYP_CTRL_TOE_EN		BIT(3)
#define KEYP_CTRL_TOLE_EN		BIT(4)
#define KEYP_CTRL_RP_EN			BIT(5)
#define KEYP_CTRL_KBD_ON		BIT(6)

/* KEYP_DEB, KEYP_LONG_KEY, KEYP_TIMEOUT_x*/
#define KEYP_PERIOD_US(t, prescale)	((t) / (31 << (prescale + 1)) - 1)

/* KEYP_LK_PTV_REG Fields */
#define KEYP_LK_PTV_PTV_SHIFT		5

/* KEYP_{IMR,ISR,SIR} Fields */
#define KEYP_IMR1_MIS			BIT(3)
#define KEYP_IMR1_TO			BIT(2)
#define KEYP_IMR1_LK			BIT(1)
#define KEYP_IMR1_KP			BIT(0)

/* KEYP_EDR Fields */
#define KEYP_EDR_KP_FALLING		0x01
#define KEYP_EDR_KP_RISING		0x02
#define KEYP_EDR_KP_BOTH		0x03
#define KEYP_EDR_LK_FALLING		0x04
#define KEYP_EDR_LK_RISING		0x08
#define KEYP_EDR_TO_FALLING		0x10
#define KEYP_EDR_TO_RISING		0x20
#define KEYP_EDR_MIS_FALLING		0x40
#define KEYP_EDR_MIS_RISING		0x80


/*----------------------------------------------------------------------*/

static int twl4030_kpread(struct twl4030_keypad *kp,
		u8 *data, u32 reg, u8 num_bytes)
{
	int ret;

	ret = twl4030_i2c_read(TWL4030_MODULE_KEYPAD, data, reg, num_bytes);
	if (ret < 0) {
		dev_warn(kp->dbg_dev,
			"Couldn't read TWL4030: %X - ret %d[%x]\n",
			 reg, ret, ret);
		return ret;
	}
	return ret;
}

static int twl4030_kpwrite_u8(struct twl4030_keypad *kp, u8 data, u32 reg)
{
	int ret;

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_KEYPAD, data, reg);
	if (ret < 0) {
		dev_warn(kp->dbg_dev,
			"Could not write TWL4030: %X - ret %d[%x]\n",
			 reg, ret, ret);
		return ret;
	}
	return ret;
}

static int twl4030_find_key(struct twl4030_keypad *kp, int col, int row)
{
	int i, rc;

	rc = KEY(col, row, 0);
	for (i = 0; i < kp->keymapsize; i++)
		if ((kp->keymap[i] & ROWCOL_MASK) == rc)
			return kp->keymap[i] & (KEYNUM_MASK | KEY_PERSISTENT);

	return -EINVAL;
}

static inline u16 twl4030_col_xlate(struct twl4030_keypad *kp, u8 col)
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

static int twl4030_read_kp_matrix_state(struct twl4030_keypad *kp, u16 *state)
{
	u8 new_state[MAX_ROWS];
	int row;
	int ret = twl4030_kpread(kp,
				 new_state, KEYP_FULL_CODE_7_0, kp->n_rows);
	if (ret >= 0) {
		for (row = 0; row < kp->n_rows; row++)
			state[row] = twl4030_col_xlate(kp, new_state[row]);
	}
	return ret;
}

static int twl4030_is_in_ghost_state(struct twl4030_keypad *kp, u16 *key_state)
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

static void twl4030_kp_scan(struct twl4030_keypad *kp, int release_all)
{
	u16 new_state[MAX_ROWS];
	int col, row;

	if (release_all)
		memset(new_state, 0, sizeof(new_state));
	else {
		/* check for any changes */
		int ret = twl4030_read_kp_matrix_state(kp, new_state);

		if (ret < 0)	/* panic ... */
			return;
		if (twl4030_is_in_ghost_state(kp, new_state))
			return;
	}

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

			key = twl4030_find_key(kp, col, row);
			if (key < 0)
				dev_warn(kp->dbg_dev,
					"Spurious key event %d-%d\n",
					 col, row);
			else if (key & KEY_PERSISTENT)
				continue;
			else
				input_report_key(kp->input, key,
						 new_state[row] & (1 << col));
		}
		kp->kp_state[row] = new_state[row];
	}
	input_sync(kp->input);
}

/*
 * Keypad interrupt handler
 */
static irqreturn_t do_kp_irq(int irq, void *_kp)
{
	struct twl4030_keypad *kp = _kp;
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
	ret = twl4030_kpread(kp, &reg, KEYP_ISR1, 1);

	/* Release all keys if I2C has gone bad or
	 * the KEYP has gone to idle state */
	if ((ret >= 0) && (reg & KEYP_IMR1_KP))
		twl4030_kp_scan(kp, 0);
	else
		twl4030_kp_scan(kp, 1);

	return IRQ_HANDLED;
}

/*
 * Registers keypad device with input subsystem
 * and configures TWL4030 keypad registers
 */
static int __devinit twl4030_kp_probe(struct platform_device *pdev)
{
	u8 reg;
	int i;
	int ret = 0;
	struct twl4030_keypad *kp;
	struct twl4030_keypad_data *pdata = pdev->dev.platform_data;

	if (!pdata || !pdata->rows || !pdata->cols || !pdata->keymap
			|| pdata->rows > 8 || pdata->cols > 8) {
		dev_err(&pdev->dev, "Invalid platform_data\n");
		return -EINVAL;
	}

	kp = kzalloc(sizeof(*kp), GFP_KERNEL);
	if (!kp)
		return -ENOMEM;

	platform_set_drvdata(pdev, kp);

	/* Get the debug Device */
	kp->dbg_dev = &pdev->dev;

	kp->input = input_allocate_device();
	if (!kp->input) {
		kfree(kp);
		return -ENOMEM;
	}

	kp->keymap = pdata->keymap;
	kp->keymapsize = pdata->keymapsize;
	kp->n_rows = pdata->rows;
	kp->n_cols = pdata->cols;
	kp->irq = platform_get_irq(pdev, 0);

	/* setup input device */
	__set_bit(EV_KEY, kp->input->evbit);

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, kp->input->evbit);

	for (i = 0; i < kp->keymapsize; i++)
		__set_bit(kp->keymap[i] & KEYNUM_MASK,
				kp->input->keybit);

	kp->input->name		= "TWL4030 Keypad";
	kp->input->phys		= "twl4030_keypad/input0";
	kp->input->dev.parent	= &pdev->dev;

	kp->input->id.bustype	= BUS_HOST;
	kp->input->id.vendor	= 0x0001;
	kp->input->id.product	= 0x0001;
	kp->input->id.version	= 0x0003;

	kp->input->keycode	= kp->keymap;
	kp->input->keycodesize	= sizeof(unsigned int);
	kp->input->keycodemax	= kp->keymapsize;

	ret = input_register_device(kp->input);
	if (ret < 0) {
		dev_err(kp->dbg_dev,
			"Unable to register twl4030 keypad device\n");
		goto err2;
	}

	/* Enable controller, with hardware decoding but not autorepeat */
	reg = KEYP_CTRL_SOFT_NRST | KEYP_CTRL_SOFTMODEN
		| KEYP_CTRL_TOE_EN | KEYP_CTRL_KBD_ON;
	ret = twl4030_kpwrite_u8(kp, reg, KEYP_CTRL);
	if (ret < 0)
		goto err3;

	/* NOTE:  we could use sih_setup() here to package keypad
	 * event sources as four different IRQs ... but we don't.
	 */

	/* Enable TO rising and KP rising and falling edge detection */
	reg = KEYP_EDR_KP_BOTH | KEYP_EDR_TO_RISING;
	ret = twl4030_kpwrite_u8(kp, reg, KEYP_EDR);
	if (ret < 0)
		goto err3;

	/* Set PTV prescaler Field */
	reg = (PTV_PRESCALER << KEYP_LK_PTV_PTV_SHIFT);
	ret = twl4030_kpwrite_u8(kp, reg, KEYP_LK_PTV);
	if (ret < 0)
		goto err3;

	/* Set key debounce time to 20 ms */
	i = KEYP_PERIOD_US(20000, PTV_PRESCALER);
	ret = twl4030_kpwrite_u8(kp, i, KEYP_DEB);
	if (ret < 0)
		goto err3;

	/* Set timeout period to 100 ms */
	i = KEYP_PERIOD_US(200000, PTV_PRESCALER);
	ret = twl4030_kpwrite_u8(kp, (i & 0xFF), KEYP_TIMEOUT_L);
	if (ret < 0)
		goto err3;
	ret = twl4030_kpwrite_u8(kp, (i >> 8), KEYP_TIMEOUT_H);
	if (ret < 0)
		goto err3;

	/* Enable Clear-on-Read; disable remembering events that fire
	 * after the IRQ but before our handler acks (reads) them,
	 */
	reg = TWL4030_SIH_CTRL_COR_MASK | TWL4030_SIH_CTRL_PENDDIS_MASK;
	ret = twl4030_kpwrite_u8(kp, reg, KEYP_SIH_CTRL);
	if (ret < 0)
		goto err3;

	/* initialize key state; irqs update it from here on */
	ret = twl4030_read_kp_matrix_state(kp, kp->kp_state);
	if (ret < 0)
		goto err3;

	/*
	 * This ISR will always execute in kernel thread context because of
	 * the need to access the TWL4030 over the I2C bus.
	 *
	 * NOTE:  we assume this host is wired to TWL4040 INT1, not INT2 ...
	 */
	ret = request_irq(kp->irq, do_kp_irq, 0, pdev->name, kp);
	if (ret < 0) {
		dev_info(kp->dbg_dev, "request_irq failed for irq no=%d\n",
			kp->irq);
		goto err3;
	} else {
		/* Enable KP and TO interrupts now. */
		reg = (u8) ~(KEYP_IMR1_KP | KEYP_IMR1_TO);
		ret = twl4030_kpwrite_u8(kp, reg, KEYP_IMR1);
		if (ret < 0)
			goto err5;
	}

	return ret;
err5:
	/* mask all events - we don't care about the result */
	(void) twl4030_kpwrite_u8(kp, 0xff, KEYP_IMR1);
	free_irq(kp->irq, NULL);
err3:
	input_unregister_device(kp->input);
	kp->input = NULL;
err2:
	input_free_device(kp->input);
	kfree(kp);
	return -ENODEV;
}

static int __devexit twl4030_kp_remove(struct platform_device *pdev)
{
	struct twl4030_keypad *kp = platform_get_drvdata(pdev);

	free_irq(kp->irq, kp);
	input_unregister_device(kp->input);
	kfree(kp);

	return 0;
}

/*
 * NOTE: twl4030 are multi-function devices connected via I2C.
 * So this device is a child of an I2C parent, thus it needs to
 * support unplug/replug (which most platform devices don't).
 */

MODULE_ALIAS("platform:twl4030_keypad");

static struct platform_driver twl4030_kp_driver = {
	.probe		= twl4030_kp_probe,
	.remove		= __devexit_p(twl4030_kp_remove),
	.driver		= {
		.name	= "twl4030_keypad",
		.owner	= THIS_MODULE,
	},
};

static int __init twl4030_kp_init(void)
{
	return platform_driver_register(&twl4030_kp_driver);
}
module_init(twl4030_kp_init);

static void __exit twl4030_kp_exit(void)
{
	platform_driver_unregister(&twl4030_kp_driver);
}
module_exit(twl4030_kp_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("TWL4030 Keypad Driver");
MODULE_LICENSE("GPL");
