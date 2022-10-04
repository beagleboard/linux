// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Light hardware spinlock driver
 *
 * Copyright (C) 2020-2025 Alibaba Group Holding Limited
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/hwspinlock.h>

static void hwspinlock_test(struct hwspinlock *hwlock, int lock_no)
{
	int i;
	int ret;

	for (i = 0; i < 10; i++) {
		ret = hwspin_trylock(hwlock);
		if (ret) {
			pr_err("%d:%d Lock failed\n", lock_no, i);
			return;
		}

		ret = hwspin_trylock(hwlock);
		if (!ret) {
			pr_err("%d:%d Recursive lock succeeded unexpectedly! Test failed\n",
						lock_no, i);
			return;
		}

		hwspin_unlock(hwlock);
		ret = hwspin_trylock(hwlock);
		if (ret) {
			pr_err("%d:%d Unlock failed\n", lock_no, i);
			hwspin_unlock(hwlock);
			return;
		}

		hwspin_unlock(hwlock);
	}
}

#define HW_SPINLOCK_NUMBER  64
static int light_hwspinlock_test_init(void)
{
	int i;
	struct hwspinlock *hwlock = NULL;

	for (i = 0; i < HW_SPINLOCK_NUMBER; i++) {
		hwlock = hwspin_lock_request_specific(i);
		if (!hwlock) {
			pr_err("request lock %d failed\n", i);
			return -EIO;
		}

		hwspinlock_test(hwlock, i);
		hwspin_lock_free(hwlock);
	}

	return 0;
}

static void light_hwspinlock_test_exit(void)
{
	pr_info("Test finished\n");
}

module_init(light_hwspinlock_test_init);
module_exit(light_hwspinlock_test_exit);

MODULE_LICENSE("Dual BSD/GPL");
