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

static struct input_dev omap_kp_dev;
static unsigned char keypad_state[8];

static struct timer_list kp_timer;
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
			udelay(2);
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

			input_report_key(&omap_kp_dev, key,
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
		mod_timer(&kp_timer, jiffies + delay);
	} else {
		/* enable interrupts */
		omap_writew(0, OMAP_MPUIO_BASE + OMAP_MPUIO_KBD_MASKIT);
	}
}

static int __init omap_kp_init(void)
{
	int i;

	printk(KERN_INFO "OMAP Keypad Driver\n");

	/* Disable the interrupt for the MPUIO keyboard */
	omap_writew(1, OMAP_MPUIO_BASE + OMAP_MPUIO_KBD_MASKIT);

	if (machine_is_omap_h2() || machine_is_omap_h3()) {
		keymap = h2_keymap;
		set_bit(EV_REP, omap_kp_dev.evbit);
	} else if (machine_is_omap_innovator()) {
		keymap = innovator_keymap;
	} else if (machine_is_omap_osk()) {
		keymap = osk_keymap;
	} else {
		keymap = test_keymap;
	}

	init_timer(&kp_timer);
	kp_timer.function = omap_kp_timer;

	/* get the irq and init timer*/
	tasklet_enable(&kp_tasklet);
	if (request_irq(INT_KEYBOARD, omap_kp_interrupt, 0,
			"omap-keypad", 0) < 0)
		return -EINVAL;

	/* setup input device */
	set_bit(EV_KEY, omap_kp_dev.evbit);
	for (i = 0; keymap[i] != 0; i++)
		set_bit(keymap[i] & 0x00ffffff, omap_kp_dev.keybit);
	omap_kp_dev.name = "omap-keypad";
	input_register_device(&omap_kp_dev);

	if (machine_is_omap_h2() || machine_is_omap_h3()) {
		omap_cfg_reg(F18_1610_KBC0);
		omap_cfg_reg(D20_1610_KBC1);
		omap_cfg_reg(D19_1610_KBC2);
		omap_cfg_reg(E18_1610_KBC3);
		omap_cfg_reg(C21_1610_KBC4);

		omap_cfg_reg(G18_1610_KBR0);
		omap_cfg_reg(F19_1610_KBR1);
		omap_cfg_reg(H14_1610_KBR2);
		omap_cfg_reg(E20_1610_KBR3);
		omap_cfg_reg(E19_1610_KBR4);
		omap_cfg_reg(N19_1610_KBR5);

		omap_writew(0xff, OMAP_MPUIO_BASE + OMAP_MPUIO_GPIO_DEBOUNCING);
	}

	/* scan current status and enable interrupt */
	omap_kp_scan_keypad(keypad_state);
	omap_writew(0, OMAP_MPUIO_BASE + OMAP_MPUIO_KBD_MASKIT);

	return 0;
}

static void __exit omap_kp_exit(void)
{
	/* disable keypad interrupt handling */
	tasklet_disable(&kp_tasklet);
	omap_writew(1, OMAP_MPUIO_BASE + OMAP_MPUIO_KBD_MASKIT);

	free_irq(INT_KEYBOARD, 0);
	del_timer_sync(&kp_timer);

	/* unregister everything */
	input_unregister_device(&omap_kp_dev);
}

module_init(omap_kp_init);
module_exit(omap_kp_exit);

MODULE_AUTHOR("Timo Teräs");
MODULE_DESCRIPTION("OMAP Keypad Driver");
MODULE_LICENSE("GPL");
