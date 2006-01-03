/*
 * linux/drivers/char/omap-keypad.c
 *
 * OMAP Keypad Driver
 *
 * Copyright (C) 2003 Nokia Corporation
 * Written by Timo Teräs <ext-timo.teras@nokia.com>
 *
 * Added support for H2 & H3 Keypad
 * Copyright (C) 2004 Texas Instruments
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
#include <asm/arch/irqs.h>
#include <asm/arch/gpio.h>
#include <asm/arch/hardware.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/mach-types.h>
#include <asm/arch/mux.h>

#undef NEW_BOARD_LEARNING_MODE

static void omap_kp_tasklet(unsigned long);
static void omap_kp_timer(unsigned long);

static unsigned char keypad_state[8];
static unsigned int keypad_irq = INT_KEYBOARD;

struct omap_kp {
	struct input_dev *input;
	struct timer_list timer;
};

DECLARE_TASKLET_DISABLED(kp_tasklet, omap_kp_tasklet, 0);

#define KEY(col, row, val) (((col) << 28) | ((row) << 24) | (val))

static int h2_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_3),
	KEY(0, 3, KEY_F10),
	KEY(0, 4, KEY_F5),
	KEY(0, 5, KEY_9),
	KEY(1, 0, KEY_DOWN),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_2),
	KEY(1, 3, KEY_F9),
	KEY(1, 4, KEY_F7),
	KEY(1, 5, KEY_0),
	KEY(2, 0, KEY_ENTER),
	KEY(2, 1, KEY_6),
	KEY(2, 2, KEY_1),
	KEY(2, 3, KEY_F2),
	KEY(2, 4, KEY_F6),
	KEY(2, 5, KEY_HOME),
	KEY(3, 0, KEY_8),
	KEY(3, 1, KEY_5),
	KEY(3, 2, KEY_F12),
	KEY(3, 3, KEY_F3),
	KEY(3, 4, KEY_F8),
	KEY(3, 5, KEY_END),
	KEY(4, 0, KEY_7),
	KEY(4, 1, KEY_4),
	KEY(4, 2, KEY_F11),
	KEY(4, 3, KEY_F1),
	KEY(4, 4, KEY_F4),
	KEY(4, 5, KEY_ESC),
	KEY(5, 0, KEY_F13),
	KEY(5, 1, KEY_F14),
	KEY(5, 2, KEY_F15),
	KEY(5, 3, KEY_F16),
	KEY(5, 4, KEY_SLEEP),
	0
};

static int test_keymap[] = {
	KEY(0, 0, KEY_F4),
	KEY(1, 0, KEY_LEFT),
	KEY(2, 0, KEY_F1),
	KEY(0, 1, KEY_DOWN),
	KEY(1, 1, KEY_ENTER),
	KEY(2, 1, KEY_UP),
	KEY(0, 2, KEY_F3),
	KEY(1, 2, KEY_RIGHT),
	KEY(2, 2, KEY_F2),
	0
};

static int innovator_keymap[] = {
	KEY(0, 0, KEY_F1),
	KEY(0, 3, KEY_DOWN),
	KEY(1, 1, KEY_F2),
	KEY(1, 2, KEY_RIGHT),
	KEY(2, 0, KEY_F3),
	KEY(2, 1, KEY_F4),
	KEY(2, 2, KEY_UP),
	KEY(3, 2, KEY_ENTER),
	KEY(3, 3, KEY_LEFT),
	0
};

static int osk_keymap[] = {
	KEY(0, 0, KEY_F1),
	KEY(0, 3, KEY_UP),
	KEY(1, 1, KEY_LEFTCTRL),
	KEY(1, 2, KEY_LEFT),
	KEY(2, 0, KEY_SPACE),
	KEY(2, 1, KEY_ESC),
	KEY(2, 2, KEY_DOWN),
	KEY(3, 2, KEY_ENTER),
	KEY(3, 3, KEY_RIGHT),
	0 
}; 

static int p2_keymap[] = {
	KEY(0,0,KEY_UP),
	KEY(0,1,KEY_RIGHT),
	KEY(0,2,KEY_LEFT),
	KEY(0,3,KEY_DOWN),
	KEY(0,4,KEY_CENTER),
	KEY(0,5,KEY_0_5),
	KEY(1,0,KEY_SOFT2),
	KEY(1,1,KEY_SEND),
	KEY(1,2,KEY_END),
	KEY(1,3,KEY_VOLUMEDOWN),
	KEY(1,4,KEY_VOLUMEUP),
	KEY(1,5,KEY_RECORD),
	KEY(2,0,KEY_SOFT1),
	KEY(2,1,KEY_3),
	KEY(2,2,KEY_6),
	KEY(2,3,KEY_9),
	KEY(2,4,KEY_SHARP),
	KEY(2,5,KEY_2_5),
	KEY(3,0,KEY_BACK),
	KEY(3,1,KEY_2),
	KEY(3,2,KEY_5),
	KEY(3,3,KEY_8),
	KEY(3,4,KEY_0),
	KEY(3,5,KEY_HEADSETHOOK),
	KEY(4,0,KEY_HOME),
	KEY(4,1,KEY_1),
	KEY(4,2,KEY_4),
	KEY(4,3,KEY_7),
	KEY(4,4,KEY_STAR),
	KEY(4,5,KEY_POWER),
	0
};

static int *keymap;

static irqreturn_t omap_kp_interrupt(int irq, void *dev_id,
				     struct pt_regs *regs)
{
	/* disable keyboard interrupt and schedule for handling */
	omap_writew(1, OMAP_MPUIO_BASE + OMAP_MPUIO_KBD_MASKIT);
	tasklet_schedule(&kp_tasklet);

	return IRQ_HANDLED;
}

static void omap_kp_timer(unsigned long data)
{
	tasklet_schedule(&kp_tasklet);
}

static void omap_kp_scan_keypad(unsigned char *state)
{
	int col = 0;

	/* read the keypad status */
	omap_writew(0xff, OMAP_MPUIO_BASE + OMAP_MPUIO_KBC);
	for (col = 0; col < 8; col++) {
		omap_writew(~(1 << col) & 0xff, OMAP_MPUIO_BASE + OMAP_MPUIO_KBC);

		if (machine_is_omap_osk() || machine_is_omap_h2() || machine_is_omap_h3()) {
			udelay(9);
		} else {
			udelay(4);
		}

		state[col] = ~omap_readw(OMAP_MPUIO_BASE + OMAP_MPUIO_KBR_LATCH) & 0xff;
	}
	omap_writew(0x00, OMAP_MPUIO_BASE + OMAP_MPUIO_KBC);
	udelay(2);
}

static inline int omap_kp_find_key(int col, int row)
{
	int i, key;

	key = KEY(col, row, 0);
	for (i = 0; keymap[i] != 0; i++)
		if ((keymap[i] & 0xff000000) == key)
			return keymap[i] & 0x00ffffff;
	return -1;
}

static void omap_kp_tasklet(unsigned long data)
{
	struct omap_kp *omap_kp_data = (struct omap_kp *) data;
	unsigned char new_state[8], changed, key_down = 0;
	int col, row;
	int spurious = 0;

	/* check for any changes */
	omap_kp_scan_keypad(new_state);

	/* check for changes and print those */
	for (col = 0; col < 8; col++) {
		changed = new_state[col] ^ keypad_state[col];
		key_down |= new_state[col];
		if (changed == 0)
			continue;

		for (row = 0; row < 8; row++) {
			int key;
			if (!(changed & (1 << row)))
				continue;
#ifdef NEW_BOARD_LEARNING_MODE
			printk(KERN_INFO "omap-keypad: key %d-%d %s\n", col, row, (new_state[col] & (1 << row)) ? "pressed" : "released");
#else
			key = omap_kp_find_key(col, row);
			if (key < 0) {
				printk(KERN_WARNING "omap-keypad: Spurious key event %d-%d\n",
				       col, row);
				/* We scan again after a couple of seconds */
				spurious = 1;
				continue;
			}

			input_report_key(omap_kp_data->input, key,
					 new_state[col] & (1 << row));
#endif
		}
	}
	memcpy(keypad_state, new_state, sizeof(keypad_state));

	if (key_down) {
                int delay = HZ / 20;
		/* some key is pressed - keep irq disabled and use timer
		 * to poll the keypad */
		if (spurious)
			delay = 2 * HZ;
		mod_timer(&omap_kp_data->timer, jiffies + delay);
	} else {
		/* enable interrupts */
		omap_writew(0, OMAP_MPUIO_BASE + OMAP_MPUIO_KBD_MASKIT);
	}
}

#ifdef CONFIG_PM
static int omap_kp_suspend(struct platform_device *dev, pm_message_t state)
{
	/* Nothing yet */

	return 0;
}

static int omap_kp_resume(struct platform_device *dev)
{
	/* Nothing yet */

	return 0;
}
#else
#define omap_kp_suspend	NULL
#define omap_kp_resume	NULL
#endif

static int __init omap_kp_probe(struct platform_device *pdev)
{
	struct omap_kp *omap_kp;
	struct input_dev *input_dev;
	int i;

	omap_kp = kzalloc(sizeof(struct omap_kp), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!omap_kp || !input_dev) {
		kfree(omap_kp);
		input_free_device(input_dev);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, omap_kp);

	omap_kp->input = input_dev;

	/* Disable the interrupt for the MPUIO keyboard */
	omap_writew(1, OMAP_MPUIO_BASE + OMAP_MPUIO_KBD_MASKIT);

	if (machine_is_omap_h2() || machine_is_omap_h3()) {
		keymap = h2_keymap;
		set_bit(EV_REP, input_dev->evbit);
	} else if (machine_is_omap_innovator()) {
		keymap = innovator_keymap;
	} else if (machine_is_omap_osk()) {
		keymap = osk_keymap;
	} else if (machine_is_omap_perseus2()) {
		keymap = p2_keymap;
		keypad_irq = INT_730_MPUIO_KEYPAD;
	} else {
		keymap = test_keymap;
	}

	init_timer(&omap_kp->timer);
	omap_kp->timer.function = omap_kp_timer;
	omap_kp->timer.data = (unsigned long) omap_kp;

	/* get the irq and init timer*/
	tasklet_enable(&kp_tasklet);
	kp_tasklet.data = (unsigned long) omap_kp;
	if (request_irq(keypad_irq, omap_kp_interrupt, 0,
			"omap-keypad", 0) < 0)
		return -EINVAL;

	/* setup input device */
	set_bit(EV_KEY, input_dev->evbit);
	for (i = 0; keymap[i] != 0; i++)
		set_bit(keymap[i] & 0x00ffffff, input_dev->keybit);
	input_dev->name = "omap-keypad";
	input_dev->cdev.dev = &pdev->dev;
	input_dev->private = omap_kp;
	input_register_device(omap_kp->input);

	if (machine_is_omap_h2() || machine_is_omap_h3() ||
	    machine_is_omap_perseus2()) {
		omap_writew(0xff, OMAP_MPUIO_BASE + OMAP_MPUIO_GPIO_DEBOUNCING);
	}
	/* scan current status and enable interrupt */
	omap_kp_scan_keypad(keypad_state);
	omap_writew(0, OMAP_MPUIO_BASE + OMAP_MPUIO_KBD_MASKIT);

	return 0;
}

static int omap_kp_remove(struct platform_device *pdev)
{
	struct omap_kp *omap_kp = platform_get_drvdata(pdev);

	/* disable keypad interrupt handling */
	tasklet_disable(&kp_tasklet);
	omap_writew(1, OMAP_MPUIO_BASE + OMAP_MPUIO_KBD_MASKIT);

	free_irq(keypad_irq, 0);
	del_timer_sync(&omap_kp->timer);

	/* unregister everything */
	input_unregister_device(omap_kp->input);

	kfree(omap_kp);

	return 0;
}

static struct platform_driver omap_kp_driver = {
	.probe		= omap_kp_probe,
	.remove		= omap_kp_remove,
	.suspend	= omap_kp_suspend,
	.resume		= omap_kp_resume,
	.driver		= {
		.name	= "omap-keypad",
	},
};

static int __devinit omap_kp_init(void)
{
	printk(KERN_INFO "OMAP Keypad Driver\n");
	return platform_driver_register(&omap_kp_driver);
}

static void __exit omap_kp_exit(void)
{
	platform_driver_unregister(&omap_kp_driver);
}

module_init(omap_kp_init);
module_exit(omap_kp_exit);

MODULE_AUTHOR("Timo Teräs");
MODULE_DESCRIPTION("OMAP Keypad Driver");
MODULE_LICENSE("GPL");
