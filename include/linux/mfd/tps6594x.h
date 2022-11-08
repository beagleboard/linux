/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Core driver interface for TI TPS6594x PMIC family
 *
 * Copyright (C) 2022 Texas Instruments Incorporated - https://www.ti.com/
 */

#ifndef __MFD_TPS6594X_H
#define __MFD_TPS6594X_H

#include <linux/bits.h>

/* TPS6594x chip ID list */
#define TPS6594X			0x00

/* All register addresses */
#define TPS6594X_REG_DEV_REV			0x01
#define TPS6594X_INT_STARTUP			0x65
#define TPS6594X_INT_MISC			0x66
#define TPS6594X_CONFIG_1			0x7d
#define TPS6594X_FSM_I2C_TRIGGERS		0x85
#define TPS6594X_FSM_NSLEEP_TRIGGERS		0x86

#define TPS6594X_GPIO1_CONF			0x31
#define TPS6594X_GPIO_OUT_1			0x3d
#define TPS6594X_GPIO_OUT_2			0x3e
#define TPS6594X_GPIO_IN_1			0x3f
#define TPS6594X_GPIO_IN_2			0x40

#define TPS6594X_RTC_SECONDS			0xb5
#define TPS6594X_RTC_MINUTES			0xb6
#define TPS6594X_RTC_HOURS			0xb7
#define TPS6594X_RTC_DAYS			0xb8
#define TPS6594X_RTC_MONTHS			0xb9
#define TPS6594X_RTC_YEARS			0xba
#define TPS6594X_RTC_WEEKS			0xbb
#define TPS6594X_ALARM_SECONDS			0xbc
#define TPS6594X_ALARM_MINUTES			0xbd
#define TPS6594X_ALARM_HOURS			0xbe
#define TPS6594X_ALARM_DAYS			0xbf
#define TPS6594X_ALARM_MONTHS			0xc0
#define TPS6594X_ALARM_YEARS			0xc1
#define TPS6594X_RTC_CTRL_1			0xc2
#define TPS6594X_RTC_CTRL_2			0xc3
#define TPS6594X_RTC_STATUS			0xc4
#define TPS6594X_RTC_INTERRUPTS			0xc5
#define TPS6594X_REG_MAX			0xd0

/* Register field definitions */
#define TPS6594X_DEV_REV_DEV_ID			0xff

#define TPS6594X_INT_STARTUP_NPWRON_START_INT	BIT(0)
#define TPS6594X_INT_STARTUP_ENABLE_INT		BIT(1)
#define TPS6594X_INT_STARTUP_RTC_INT		BIT(2)
#define TPS6594X_INT_STARTUP_FSD_INT		BIT(4)
#define TPS6594X_INT_STARTUP_SOFT_REBOOT_INT	BIT(5)

#define TPS6594X_INT_MISC_BIST_PASS_INT		BIT(0)
#define TPS6594X_INT_MISC_EXT_CLK_INT		BIT(1)
#define TPS6594X_INT_MISC_TWARN_INT		BIT(3)

#define TPS6594X_CONFIG_NSLEEP1_MASK		BIT(6)
#define TPS6594X_CONFIG_NSLEEP2_MASK		BIT(7)

#define TPS6594X_FSM_I2C_TRIGGERS_I2C0		BIT(0)

#define TPS6594X_FSM_NSLEEP_NSLEEP1B		BIT(0)
#define TPS6594X_FSM_NSLEEP_NSLEEP2B		BIT(1)

#define TPS6594X_RTC_CTRL_REG_GET_TIME		BIT(6)
#define TPS6594X_RTC_CTRL_REG_STOP_RTC		BIT(0)
#define TPS6594X_RTC_INTERRUPTS_REG_IT_ALARM	BIT(3)

#define TPS6594X_RTC_STATUS_RUN			BIT(1)

/**
 * struct tps6594x - state holder for the tps6594x driver
 * @dev: struct device pointer for MFD device
 * @rev: revision of the tps6594x
 * @lock: lock guarding the data structure
 * @regmap: register map of the tps6594x PMIC
 *
 * Device data may be used to access the TPS6594X chip
 */
struct tps6594x {
	struct device *dev;
	u8 rev;
	struct regmap *regmap;
};
#endif /* __MFD_TPS6594X_H */
