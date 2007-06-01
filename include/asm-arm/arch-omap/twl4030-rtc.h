/*
 * include/asm-arm/arch-omap/twl4030-rtc.h
 *
 * Copyright (C) 2006 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __TWL4030_RTC_H__
#define __TWL4030_RTC_H__

#define REG_SECONDS_REG                          (0x0)
#define REG_MINUTES_REG                          (0x1)
#define REG_HOURS_REG                            (0x2)
#define REG_DAYS_REG                             (0x3)
#define REG_MONTHS_REG                           (0x4)
#define REG_YEARS_REG                            (0x5)
#define REG_WEEKS_REG                            (0x6)
#define REG_ALARM_SECONDS_REG                    (0x7)
#define REG_ALARM_MINUTES_REG                    (0x8)
#define REG_ALARM_HOURS_REG                      (0x9)
#define REG_ALARM_DAYS_REG                       (0xA)
#define REG_ALARM_MONTHS_REG                     (0xB)
#define REG_ALARM_YEARS_REG                      (0xC)
#define REG_RTC_CTRL_REG                         (0xD)
#define REG_RTC_STATUS_REG                       (0xE)
#define REG_RTC_INTERRUPTS_REG                   (0xF)
#define REG_RTC_COMP_LSB_REG                     (0x10)
#define REG_RTC_COMP_MSB_REG                     (0x11)

/* REVISIT: these TWL4030 power registers are only used
 * by rtc-twl4030 driver, move to an appropriate header
 * if other drivers need the registers
 */
/* Power registers */
#define REG_PWR_ISR1		0x00
#define REG_PWR_IMR1		0x01
#define REG_PWR_EDR1		0x05

#define PWR_RTC_IT_UNMASK	 ~(0x08)
#define PWR_RTC_INT_CLR          0x08

/**** BitField Definitions */
/* SECONDS_REG Fields */
#define BIT_SECONDS_REG_SEC0                     (0x000)
#define BIT_SECONDS_REG_SEC0_M                   (0x0000000F)
#define BIT_SECONDS_REG_SEC1                     (0x004)
#define BIT_SECONDS_REG_SEC1_M                   (0x00000070)
/* MINUTES_REG Fields */
#define BIT_MINUTES_REG_MIN0                     (0x000)
#define BIT_MINUTES_REG_MIN0_M                   (0x0000000F)
#define BIT_MINUTES_REG_MIN1                     (0x004)
#define BIT_MINUTES_REG_MIN1_M                   (0x00000070)
/* HOURS_REG Fields */
#define BIT_HOURS_REG_HOUR0                      (0x000)
#define BIT_HOURS_REG_HOUR0_M                    (0x0000000F)
#define BIT_HOURS_REG_HOUR1                      (0x004)
#define BIT_HOURS_REG_HOUR1_M                    (0x00000030)
#define BIT_HOURS_REG_PM_NAM                     (0x007)
#define BIT_HOURS_REG_PM_NAM_M                   (0x00000080)
/* DAYS_REG Fields */
#define BIT_DAYS_REG_DAY0                        (0x000)
#define BIT_DAYS_REG_DAY0_M                      (0x0000000F)
#define BIT_DAYS_REG_DAY1                        (0x004)
#define BIT_DAYS_REG_DAY1_M                      (0x00000030)
/* MONTHS_REG Fields */
#define BIT_MONTHS_REG_MONTH0                    (0x000)
#define BIT_MONTHS_REG_MONTH0_M                  (0x0000000F)
#define BIT_MONTHS_REG_MONTH1                    (0x004)
#define BIT_MONTHS_REG_MONTH1_M                  (0x00000010)
/* YEARS_REG Fields */
#define BIT_YEARS_REG_YEAR0                      (0x000)
#define BIT_YEARS_REG_YEAR0_M                    (0x0000000F)
#define BIT_YEARS_REG_YEAR1                      (0x004)
#define BIT_YEARS_REG_YEAR1_M                    (0x000000F0)
/* WEEKS_REG Fields */
#define BIT_WEEKS_REG_WEEK                       (0x000)
#define BIT_WEEKS_REG_WEEK_M                     (0x00000007)
/* ALARM_SECONDS_REG Fields */
#define BIT_ALARM_SECONDS_REG_ALARM_SEC0         (0x000)
#define BIT_ALARM_SECONDS_REG_ALARM_SEC0_M       (0x0000000F)
#define BIT_ALARM_SECONDS_REG_ALARM_SEC1         (0x004)
#define BIT_ALARM_SECONDS_REG_ALARM_SEC1_M       (0x00000070)
/* ALARM_MINUTES_REG Fields */
#define BIT_ALARM_MINUTES_REG_ALARM_MIN0         (0x000)
#define BIT_ALARM_MINUTES_REG_ALARM_MIN0_M       (0x0000000F)
#define BIT_ALARM_MINUTES_REG_ALARM_MIN1         (0x004)
#define BIT_ALARM_MINUTES_REG_ALARM_MIN1_M       (0x00000070)
/* ALARM_HOURS_REG Fields */
#define BIT_ALARM_HOURS_REG_ALARM_HOUR0          (0x000)
#define BIT_ALARM_HOURS_REG_ALARM_HOUR0_M        (0x0000000F)
#define BIT_ALARM_HOURS_REG_ALARM_HOUR1          (0x004)
#define BIT_ALARM_HOURS_REG_ALARM_HOUR1_M        (0x00000030)
#define BIT_ALARM_HOURS_REG_ALARM_PM_NAM         (0x007)
#define BIT_ALARM_HOURS_REG_ALARM_PM_NAM_M       (0x00000080)
/* ALARM_DAYS_REG Fields */
#define BIT_ALARM_DAYS_REG_ALARM_DAY0            (0x000)
#define BIT_ALARM_DAYS_REG_ALARM_DAY0_M          (0x0000000F)
#define BIT_ALARM_DAYS_REG_ALARM_DAY1            (0x004)
#define BIT_ALARM_DAYS_REG_ALARM_DAY1_M          (0x00000030)
/* ALARM_MONTHS_REG Fields */
#define BIT_ALARM_MONTHS_REG_ALARM_MONTH0        (0x000)
#define BIT_ALARM_MONTHS_REG_ALARM_MONTH0_M      (0x0000000F)
#define BIT_ALARM_MONTHS_REG_ALARM_MONTH1        (0x004)
#define BIT_ALARM_MONTHS_REG_ALARM_MONTH1_M      (0x00000010)
/* ALARM_YEARS_REG Fields */
#define BIT_ALARM_YEARS_REG_ALARM_YEAR0          (0x000)
#define BIT_ALARM_YEARS_REG_ALARM_YEAR0_M        (0x0000000F)
#define BIT_ALARM_YEARS_REG_ALARM_YEAR1          (0x004)
#define BIT_ALARM_YEARS_REG_ALARM_YEAR1_M        (0x000000F0)
/* RTC_CTRL_REG Fields */
#define BIT_RTC_CTRL_REG_STOP_RTC                (0x000)
#define BIT_RTC_CTRL_REG_STOP_RTC_M              (0x00000001)
#define BIT_RTC_CTRL_REG_ROUND_30S               (0x001)
#define BIT_RTC_CTRL_REG_ROUND_30S_M             (0x00000002)
#define BIT_RTC_CTRL_REG_AUTO_COMP               (0x002)
#define BIT_RTC_CTRL_REG_AUTO_COMP_M             (0x00000004)
#define BIT_RTC_CTRL_REG_MODE_12_24              (0x003)
#define BIT_RTC_CTRL_REG_MODE_12_24_M            (0x00000008)
#define BIT_RTC_CTRL_REG_TEST_MODE               (0x004)
#define BIT_RTC_CTRL_REG_TEST_MODE_M             (0x00000010)
#define BIT_RTC_CTRL_REG_SET_32_COUNTER          (0x005)
#define BIT_RTC_CTRL_REG_SET_32_COUNTER_M        (0x00000020)
#define BIT_RTC_CTRL_REG_GET_TIME                (0x006)
#define BIT_RTC_CTRL_REG_GET_TIME_M              (0x00000040)
/* RTC_STATUS_REG Fields */
#define BIT_RTC_STATUS_REG_RUN                   (0x001)
#define BIT_RTC_STATUS_REG_RUN_M                 (0x00000002)
#define BIT_RTC_STATUS_REG_1S_EVENT              (0x002)
#define BIT_RTC_STATUS_REG_1S_EVENT_M            (0x00000004)
#define BIT_RTC_STATUS_REG_1M_EVENT              (0x003)
#define BIT_RTC_STATUS_REG_1M_EVENT_M            (0x00000008)
#define BIT_RTC_STATUS_REG_1H_EVENT              (0x004)
#define BIT_RTC_STATUS_REG_1H_EVENT_M            (0x00000010)
#define BIT_RTC_STATUS_REG_1D_EVENT              (0x005)
#define BIT_RTC_STATUS_REG_1D_EVENT_M            (0x00000020)
#define BIT_RTC_STATUS_REG_ALARM                 (0x006)
#define BIT_RTC_STATUS_REG_ALARM_M               (0x00000040)
#define BIT_RTC_STATUS_REG_POWER_UP              (0x007)
#define BIT_RTC_STATUS_REG_POWER_UP_M            (0x00000080)

/* RTC_INTERRUPTS_REG Fields */
#define BIT_RTC_INTERRUPTS_REG_EVERY             (0x000)
#define BIT_RTC_INTERRUPTS_REG_EVERY_M           (0x00000003)
#define BIT_RTC_INTERRUPTS_REG_IT_TIMER          (0x002)
#define BIT_RTC_INTERRUPTS_REG_IT_TIMER_M        (0x00000004)
#define BIT_RTC_INTERRUPTS_REG_IT_ALARM          (0x003)
#define BIT_RTC_INTERRUPTS_REG_IT_ALARM_M        (0x00000008)
/* RTC_COMP_LSB_REG Fields */
#define BIT_RTC_COMP_LSB_REG_RTC_COMP_LSB        (0x000)
#define BIT_RTC_COMP_LSB_REG_RTC_COMP_LSB_M      (0x000000FF)
/* RTC_COMP_MSB_REG Fields */
#define BIT_RTC_COMP_MSB_REG_RTC_COMP_MSB        (0x000)
#define BIT_RTC_COMP_MSB_REG_RTC_COMP_MSB_M      (0x000000FF)

/* ALARM_DAYS_REG Fields */
#define BIT_ALARM_DAYS_REG_ALARM_DAY1            (0x004)
#define BIT_ALARM_DAYS_REG_ALARM_DAY1_M          (0x00000030)
/* ALARM_MONTHS_REG Fields */
#define BIT_ALARM_MONTHS_REG_ALARM_MONTH0        (0x000)
#define BIT_ALARM_MONTHS_REG_ALARM_MONTH0_M      (0x0000000F)
#define BIT_ALARM_MONTHS_REG_ALARM_MONTH1        (0x004)
#define BIT_ALARM_MONTHS_REG_ALARM_MONTH1_M      (0x00000010)
/* ALARM_YEARS_REG Fields */
#define BIT_ALARM_YEARS_REG_ALARM_YEAR0          (0x000)
#define BIT_ALARM_YEARS_REG_ALARM_YEAR0_M        (0x0000000F)
#define BIT_ALARM_YEARS_REG_ALARM_YEAR1          (0x004)
#define BIT_ALARM_YEARS_REG_ALARM_YEAR1_M        (0x000000F0)
/* RTC_CTRL_REG Fields */
#define BIT_RTC_CTRL_REG_STOP_RTC                (0x000)
#define BIT_RTC_CTRL_REG_STOP_RTC_M              (0x00000001)
#define BIT_RTC_CTRL_REG_ROUND_30S               (0x001)
#define BIT_RTC_CTRL_REG_ROUND_30S_M             (0x00000002)
#define BIT_RTC_CTRL_REG_AUTO_COMP               (0x002)
#define BIT_RTC_CTRL_REG_AUTO_COMP_M             (0x00000004)
#define BIT_RTC_CTRL_REG_MODE_12_24              (0x003)
#define BIT_RTC_CTRL_REG_MODE_12_24_M            (0x00000008)
#define BIT_RTC_CTRL_REG_TEST_MODE               (0x004)
#define BIT_RTC_CTRL_REG_TEST_MODE_M             (0x00000010)
#define BIT_RTC_CTRL_REG_SET_32_COUNTER          (0x005)
#define BIT_RTC_CTRL_REG_SET_32_COUNTER_M        (0x00000020)
#define BIT_RTC_CTRL_REG_GET_TIME                (0x006)
#define BIT_RTC_CTRL_REG_GET_TIME_M              (0x00000040)
/* RTC_STATUS_REG Fields */
#define BIT_RTC_STATUS_REG_RUN                   (0x001)
#define BIT_RTC_STATUS_REG_RUN_M                 (0x00000002)
#define BIT_RTC_STATUS_REG_1S_EVENT              (0x002)
#define BIT_RTC_STATUS_REG_1S_EVENT_M            (0x00000004)
#define BIT_RTC_STATUS_REG_1M_EVENT              (0x003)
#define BIT_RTC_STATUS_REG_1M_EVENT_M            (0x00000008)
#define BIT_RTC_STATUS_REG_1H_EVENT              (0x004)
#define BIT_RTC_STATUS_REG_1H_EVENT_M            (0x00000010)
#define BIT_RTC_STATUS_REG_1D_EVENT              (0x005)
#define BIT_RTC_STATUS_REG_1D_EVENT_M            (0x00000020)
#define BIT_RTC_STATUS_REG_ALARM                 (0x006)
#define BIT_RTC_STATUS_REG_ALARM_M               (0x00000040)
#define BIT_RTC_STATUS_REG_POWER_UP              (0x007)
#define BIT_RTC_STATUS_REG_POWER_UP_M            (0x00000080)
/* RTC_INTERRUPTS_REG Fields */
#define BIT_RTC_INTERRUPTS_REG_EVERY             (0x000)
#define BIT_RTC_INTERRUPTS_REG_EVERY_M           (0x00000003)
#define BIT_RTC_INTERRUPTS_REG_IT_TIMER          (0x002)
#define BIT_RTC_INTERRUPTS_REG_IT_TIMER_M        (0x00000004)
#define BIT_RTC_INTERRUPTS_REG_IT_ALARM          (0x003)
#define BIT_RTC_INTERRUPTS_REG_IT_ALARM_M        (0x00000008)
/* RTC_COMP_LSB_REG Fields */
#define BIT_RTC_COMP_LSB_REG_RTC_COMP_LSB        (0x000)
#define BIT_RTC_COMP_LSB_REG_RTC_COMP_LSB_M      (0x000000FF)
/* RTC_COMP_MSB_REG Fields */
#define BIT_RTC_COMP_MSB_REG_RTC_COMP_MSB        (0x000)
#define BIT_RTC_COMP_MSB_REG_RTC_COMP_MSB_M      (0x000000FF)


struct twl4030rtc_platform_data {
        int (*init)(void);
        void (*exit)(void);
};

#endif				/* End of __TWL4030_RTC_H__ */
