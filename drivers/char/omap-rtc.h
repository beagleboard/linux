/*
 *  TI OMAP Real Time Clock header file
 *
 *  Copyright (C) 2003 TI
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 */

#define OMAP_RTC_BASE			0xfffb4800
#define OMAP_RTC_SIZE			128

#define OMAP_RTC_VIRT_BASE		IO_ADDRESS(OMAP_RTC_BASE)

/*
 * Real-Time Clock
 */

#define OMAP_RTC_SECONDS_REG            (OMAP_RTC_BASE + 0x00)
#define OMAP_RTC_MINUTES_REG            (OMAP_RTC_BASE + 0x04)
#define OMAP_RTC_HOURS_REG              (OMAP_RTC_BASE + 0x08)
#define OMAP_RTC_DAYS_REG               (OMAP_RTC_BASE + 0x0C)
#define OMAP_RTC_MONTHS_REG             (OMAP_RTC_BASE + 0x10)
#define OMAP_RTC_YEARS_REG              (OMAP_RTC_BASE + 0x14)
#define OMAP_RTC_WEEKS_REG              (OMAP_RTC_BASE + 0x18)
#define OMAP_RTC_ALARM_SECONDS_REG      (OMAP_RTC_BASE + 0x20)
#define OMAP_RTC_ALARM_MINUTES_REG      (OMAP_RTC_BASE + 0x24)
#define OMAP_RTC_ALARM_HOURS_REG        (OMAP_RTC_BASE + 0x28)
#define OMAP_RTC_ALARM_DAYS_REG         (OMAP_RTC_BASE + 0x2c)
#define OMAP_RTC_ALARM_MONTHS_REG       (OMAP_RTC_BASE + 0x30)
#define OMAP_RTC_ALARM_YEARS_REG        (OMAP_RTC_BASE + 0x34)
#define OMAP_RTC_CTRL_REG               (OMAP_RTC_BASE + 0x40)
#define OMAP_RTC_STATUS_REG             (OMAP_RTC_BASE + 0x44)
#define OMAP_RTC_INTERRUPTS_REG         (OMAP_RTC_BASE + 0x48)
#define OMAP_RTC_COMP_LSB_REG           (OMAP_RTC_BASE + 0x4c)
#define OMAP_RTC_COMP_MSB_REG           (OMAP_RTC_BASE + 0x50)

/* RTC Control Register bit fields: */

#define OMAP_RTC_CTRL_STOP              (1<<0)

/* RTC Status Register bit fields: */

#define OMAP_RTC_STATUS_POWER_UP        (1<<7)
#define OMAP_RTC_STATUS_ALARM           (1<<6)
#define OMAP_RTC_STATUS_1D_EVENT        (1<<5)
#define OMAP_RTC_STATUS_1H_EVENT        (1<<4)
#define OMAP_RTC_STATUS_1M_EVENT        (1<<3)
#define OMAP_RTC_STATUS_1S_EVENT        (1<<2)
#define OMAP_RTC_STATUS_RUN             (1<<1)
#define OMAP_RTC_STATUS_BUSY            (1<<0)

/* RTC Interrupt Register bit fields: */

#define OMAP_RTC_INTERRUPTS_IT_ALARM    (1<<3)
#define OMAP_RTC_INTERRUPTS_IT_TIMER    (1<<2)
