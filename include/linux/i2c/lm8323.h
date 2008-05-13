/*
 * include/lm8323.h
 *
 * Configuration for LM8323 keypad driver.
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
	int repeat : 1;
	const s16 *keymap;

	char *pwm1_name; /* Device name for PWM1. */
	char *pwm2_name; /* Device name for PWM2. */
	char *pwm3_name; /* Device name for PWM3. */

	char *name; /* Device name. */
};

void __init lm8323_set_platform_data(struct lm8323_platform_data *pdata);

#endif /* __LINUX_LM8323_H */
