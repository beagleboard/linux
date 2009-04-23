/*
 * lm8323.h - Configuration for LM8323 keypad driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License only).
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

#ifndef __LINUX_LM8323_H
#define __LINUX_LM8323_H

#include <linux/types.h>

/*
 * Largest keycode that the chip can send, plus one,
 * so keys can be mapped directly at the index of the
 * LM8323 keycode instead of subtracting one.
 */
#define LM8323_KEYMAP_SIZE (0x7f + 1)

struct lm8323_platform_data {
	int debounce_time; /* Time to watch for key bouncing, in ms. */
	int active_time; /* Idle time until sleep, in ms. */

	int size_x;
	int size_y;
	int repeat:1;
	const s16 *keymap;

	char *pwm1_name; /* Device name for PWM1. */
	char *pwm2_name; /* Device name for PWM2. */
	char *pwm3_name; /* Device name for PWM3. */

	char *name; /* Device name. */
};

#endif /* __LINUX_LM8323_H */
