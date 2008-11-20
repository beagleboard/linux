/*
 * linux/drivers/power/twl4030_bci_battery.c
 *
 * OMAP2430/3430 BCI battery driver for Linux
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 * Author: Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl4030.h>
#include <linux/power_supply.h>
#include <linux/i2c/twl4030-madc.h>

#define T2_BATTERY_VOLT		0x04
#define T2_BATTERY_TEMP		0x06
#define T2_BATTERY_CUR		0x08

/* charger constants */
#define NO_PW_CONN		0
#define AC_PW_CONN		0x01
#define USB_PW_CONN		0x02

/* TWL4030_MODULE_USB */
#define REG_POWER_CTRL		0x0AC
#define OTG_EN			0x020
#define REG_PHY_CLK_CTRL	0x0FE
#define REG_PHY_CLK_CTRL_STS	0x0FF
#define PHY_DPLL_CLK		0x01

#define REG_BCICTL1		0x023
#define REG_BCICTL2		0x024
#define CGAIN			0x020
#define ITHEN			0x010
#define ITHSENS			0x007

/* Boot BCI flag bits */
#define BCIAUTOWEN		0x020
#define CONFIG_DONE		0x010
#define BCIAUTOUSB		0x002
#define BCIAUTOAC		0x001
#define BCIMSTAT_MASK		0x03F

/* Boot BCI register */
#define REG_BOOT_BCI		0x007
#define REG_CTRL1		0x00
#define REG_SW1SELECT_MSB	0x07
#define SW1_CH9_SEL		0x02
#define REG_CTRL_SW1		0x012
#define SW1_TRIGGER		0x020
#define EOC_SW1			0x002
#define REG_GPCH9		0x049
#define REG_STS_HW_CONDITIONS	0x0F
#define STS_VBUS		0x080
#define STS_CHG			0x02
#define REG_BCIMSTATEC		0x02
#define REG_BCIMFSTS4		0x010
#define REG_BCIMFSTS2		0x00E
#define REG_BCIMFSTS3		0x00F
#define REG_BCIMFSTS1		0x001
#define USBFASTMCHG		0x004
#define BATSTSPCHG		0x004
#define BATSTSMCHG		0x040
#define VBATOV4			0x020
#define VBATOV3			0x010
#define VBATOV2			0x008
#define VBATOV1			0x004
#define REG_BB_CFG		0x012
#define BBCHEN			0x010

/* Power supply charge interrupt */
#define REG_PWR_ISR1		0x00
#define REG_PWR_IMR1		0x01
#define REG_PWR_EDR1		0x05
#define REG_PWR_SIH_CTRL	0x007

#define USB_PRES		0x004
#define CHG_PRES		0x002

#define USB_PRES_RISING		0x020
#define USB_PRES_FALLING	0x010
#define CHG_PRES_RISING		0x008
#define CHG_PRES_FALLING	0x004
#define AC_STATEC		0x20
#define COR			0x004

/* interrupt status registers */
#define REG_BCIISR1A		0x0
#define REG_BCIISR2A		0x01

/* Interrupt flags bits BCIISR1 */
#define BATSTS_ISR1		0x080
#define VBATLVL_ISR1		0x001

/* Interrupt mask registers for int1*/
#define REG_BCIIMR1A		0x002
#define REG_BCIIMR2A		0x003

 /* Interrupt masks for BCIIMR1 */
#define BATSTS_IMR1		0x080
#define VBATLVL_IMR1		0x001

/* Interrupt edge detection register */
#define REG_BCIEDR1		0x00A
#define REG_BCIEDR2		0x00B
#define REG_BCIEDR3		0x00C

/* BCIEDR2 */
#define	BATSTS_EDRRISIN		0x080
#define BATSTS_EDRFALLING	0x040

/* BCIEDR3 */
#define	VBATLVL_EDRRISIN	0x02

/* Step size and prescaler ratio */
#define TEMP_STEP_SIZE		147
#define TEMP_PSR_R		100

#define VOLT_STEP_SIZE		588
#define VOLT_PSR_R		100

#define CURR_STEP_SIZE		147
#define CURR_PSR_R1		44
#define CURR_PSR_R2		80

#define BK_VOLT_STEP_SIZE	441
#define BK_VOLT_PSR_R		100

#define ENABLE		1
#define DISABLE		1

/* Ptr to thermistor table */
int *therm_tbl;

struct twl4030_bci_device_info {
	struct device		*dev;

	unsigned long		update_time;
	int			voltage_uV;
	int			bk_voltage_uV;
	int			current_uA;
	int			temp_C;
	int			charge_rsoc;
	int			charge_status;

	struct power_supply	bat;
	struct power_supply	bk_bat;
	struct delayed_work	twl4030_bci_monitor_work;
	struct delayed_work	twl4030_bk_bci_monitor_work;
};

static int usb_charger_flag;
static int LVL_1, LVL_2, LVL_3, LVL_4;

static int read_bci_val(u8 reg_1);
static inline int clear_n_set(u8 mod_no, u8 clear, u8 set, u8 reg);
static int twl4030charger_presence(void);

/*
 * Report and clear the charger presence event.
 */
static inline int twl4030charger_presence_evt(void)
{
	int ret;
	u8 chg_sts, set = 0, clear = 0;

	/* read charger power supply status */
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &chg_sts,
		REG_STS_HW_CONDITIONS);
	if (ret)
		return IRQ_NONE;

	if (chg_sts & STS_CHG) { /* If the AC charger have been connected */
		/* configuring falling edge detection for CHG_PRES */
		set = CHG_PRES_FALLING;
		clear = CHG_PRES_RISING;
	} else { /* If the AC charger have been disconnected */
		/* configuring rising edge detection for CHG_PRES */
		set = CHG_PRES_RISING;
		clear = CHG_PRES_FALLING;
	}

	/* Update the interrupt edge detection register */
	clear_n_set(TWL4030_MODULE_INT, clear, set, REG_PWR_EDR1);

	return 0;
}

/*
 * Interrupt service routine
 *
 * Attends to TWL 4030 power module interruptions events, specifically
 * USB_PRES (USB charger presence) CHG_PRES (AC charger presence) events
 *
 */
static irqreturn_t twl4030charger_interrupt(int irq, void *_di)
{
	struct twl4030_bci_device_info *di = _di;

#ifdef CONFIG_LOCKDEP
	/* WORKAROUND for lockdep forcing IRQF_DISABLED on us, which
	 * we don't want and can't tolerate.  Although it might be
	 * friendlier not to borrow this thread context...
	 */
	local_irq_enable();
#endif

	twl4030charger_presence_evt();
	power_supply_changed(&di->bat);

	return IRQ_HANDLED;
}

/*
 * This function handles the twl4030 battery presence interrupt
 */
static int twl4030battery_presence_evt(void)
{
	int ret;
	u8 batstsmchg, batstspchg;

	/* check for the battery presence in main charge*/
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE,
			&batstsmchg, REG_BCIMFSTS3);
	if (ret)
		return ret;

	/* check for the battery presence in precharge */
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_PRECHARGE,
			&batstspchg, REG_BCIMFSTS1);
	if (ret)
		return ret;

	/*
	 * REVISIT: Physically inserting/removing the batt
	 * does not seem to generate an int on 3430ES2 SDP.
	 */
	if ((batstspchg & BATSTSPCHG) || (batstsmchg & BATSTSMCHG)) {
		/* In case of the battery insertion event */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, BATSTS_EDRRISIN,
			BATSTS_EDRFALLING, REG_BCIEDR2);
		if (ret)
			return ret;
	} else {
		/* In case of the battery removal event */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, BATSTS_EDRFALLING,
			BATSTS_EDRRISIN, REG_BCIEDR2);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * This function handles the twl4030 battery voltage level interrupt.
 */
static int twl4030battery_level_evt(void)
{
	int ret;
	u8 mfst;

	/* checking for threshold event */
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE,
			&mfst, REG_BCIMFSTS2);
	if (ret)
		return ret;

	/* REVISIT could use a bitmap */
	if (mfst & VBATOV4) {
		LVL_4 = 1;
		LVL_3 = 0;
		LVL_2 = 0;
		LVL_1 = 0;
	} else if (mfst & VBATOV3) {
		LVL_4 = 0;
		LVL_3 = 1;
		LVL_2 = 0;
		LVL_1 = 0;
	} else if (mfst & VBATOV2) {
		LVL_4 = 0;
		LVL_3 = 0;
		LVL_2 = 1;
		LVL_1 = 0;
	} else {
		LVL_4 = 0;
		LVL_3 = 0;
		LVL_2 = 0;
		LVL_1 = 1;
	}

	return 0;
}

/*
 * Interrupt service routine
 *
 * Attends to BCI interruptions events,
 * specifically BATSTS (battery connection and removal)
 * VBATOV (main battery voltage threshold) events
 *
 */
static irqreturn_t twl4030battery_interrupt(int irq, void *_di)
{
	u8 isr1a_val, isr2a_val, clear_2a, clear_1a;
	int ret;

#ifdef CONFIG_LOCKDEP
	/* WORKAROUND for lockdep forcing IRQF_DISABLED on us, which
	 * we don't want and can't tolerate.  Although it might be
	 * friendlier not to borrow this thread context...
	 */
	local_irq_enable();
#endif

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_INTERRUPTS, &isr1a_val,
				REG_BCIISR1A);
	if (ret)
		return IRQ_NONE;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_INTERRUPTS, &isr2a_val,
				REG_BCIISR2A);
	if (ret)
		return IRQ_NONE;

	clear_2a = (isr2a_val & VBATLVL_ISR1) ? (VBATLVL_ISR1) : 0;
	clear_1a = (isr1a_val & BATSTS_ISR1) ? (BATSTS_ISR1) : 0;

	/* cleaning BCI interrupt status flags */
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_INTERRUPTS,
			clear_1a , REG_BCIISR1A);
	if (ret)
		return IRQ_NONE;

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_INTERRUPTS,
			clear_2a , REG_BCIISR2A);
	if (ret)
		return IRQ_NONE;

	/* battery connetion or removal event */
	if (isr1a_val & BATSTS_ISR1)
		twl4030battery_presence_evt();
	/* battery voltage threshold event*/
	else if (isr2a_val & VBATLVL_ISR1)
		twl4030battery_level_evt();
	else
		return IRQ_NONE;

	return IRQ_HANDLED;
}

/*
 * Enable/Disable hardware battery level event notifications.
 */
static int twl4030battery_hw_level_en(int enable)
{
	int ret;

	if (enable) {
		/* unmask VBATOV interrupt for INT1 */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, VBATLVL_IMR1,
			0, REG_BCIIMR2A);
		if (ret)
			return ret;

		/* configuring interrupt edge detection for VBATOv */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, 0,
			VBATLVL_EDRRISIN, REG_BCIEDR3);
		if (ret)
			return ret;
	} else {
		/* mask VBATOV interrupt for INT1 */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, 0,
			VBATLVL_IMR1, REG_BCIIMR2A);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Enable/disable hardware battery presence event notifications.
 */
static int twl4030battery_hw_presence_en(int enable)
{
	int ret;

	if (enable) {
		/* unmask BATSTS interrupt for INT1 */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, BATSTS_IMR1,
			0, REG_BCIIMR1A);
		if (ret)
			return ret;

		/* configuring interrupt edge for BATSTS */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, 0,
			BATSTS_EDRRISIN | BATSTS_EDRFALLING, REG_BCIEDR2);
		if (ret)
			return ret;
	} else {
		/* mask BATSTS interrupt for INT1 */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, 0,
			BATSTS_IMR1, REG_BCIIMR1A);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Enable/Disable AC Charge funtionality.
 */
static int twl4030charger_ac_en(int enable)
{
	int ret;

	if (enable) {
		/* forcing the field BCIAUTOAC (BOOT_BCI[0) to 1 */
		ret = clear_n_set(TWL4030_MODULE_PM_MASTER, 0,
			(CONFIG_DONE | BCIAUTOWEN | BCIAUTOAC),
			REG_BOOT_BCI);
		if (ret)
			return ret;
	} else {
		/* forcing the field BCIAUTOAC (BOOT_BCI[0) to 0*/
		ret = clear_n_set(TWL4030_MODULE_PM_MASTER, BCIAUTOAC,
			(CONFIG_DONE | BCIAUTOWEN),
			REG_BOOT_BCI);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Enable/Disable USB Charge funtionality.
 */
int twl4030charger_usb_en(int enable)
{
	u8 value;
	int ret;
	unsigned long timeout;

	if (enable) {
		/* Check for USB charger conneted */
		ret = twl4030charger_presence();
		if (ret < 0)
			return ret;

		if (!(ret & USB_PW_CONN))
			return -ENXIO;

		/* forcing the field BCIAUTOUSB (BOOT_BCI[1]) to 1 */
		ret = clear_n_set(TWL4030_MODULE_PM_MASTER, 0,
			(CONFIG_DONE | BCIAUTOWEN | BCIAUTOUSB),
			REG_BOOT_BCI);
		if (ret)
			return ret;

		ret = clear_n_set(TWL4030_MODULE_USB, 0, PHY_DPLL_CLK,
			REG_PHY_CLK_CTRL);
		if (ret)
			return ret;

		value = 0;
		timeout = jiffies + msecs_to_jiffies(50);

		while ((!(value & PHY_DPLL_CLK)) &&
			time_before(jiffies, timeout)) {
			udelay(10);
			ret = twl4030_i2c_read_u8(TWL4030_MODULE_USB, &value,
				REG_PHY_CLK_CTRL_STS);
			if (ret)
				return ret;
		}

		/* OTG_EN (POWER_CTRL[5]) to 1 */
		ret = clear_n_set(TWL4030_MODULE_USB, 0, OTG_EN,
			REG_POWER_CTRL);
		if (ret)
			return ret;

		mdelay(50);

		/* forcing USBFASTMCHG(BCIMFSTS4[2]) to 1 */
		ret = clear_n_set(TWL4030_MODULE_MAIN_CHARGE, 0,
			USBFASTMCHG, REG_BCIMFSTS4);
		if (ret)
			return ret;
	} else {
		twl4030charger_presence();
		ret = clear_n_set(TWL4030_MODULE_PM_MASTER, BCIAUTOUSB,
			(CONFIG_DONE | BCIAUTOWEN), REG_BOOT_BCI);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Return battery temperature
 * Or < 0 on failure.
 */
static int twl4030battery_temperature(void)
{
	u8 val;
	int temp, curr, volt, res, ret;

	/* Getting and calculating the thermistor voltage */
	ret = read_bci_val(T2_BATTERY_TEMP);
	if (ret < 0)
		return ret;

	volt = (ret * TEMP_STEP_SIZE) / TEMP_PSR_R;

	/* Getting and calculating the supply current in micro ampers */
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, &val,
		 REG_BCICTL2);
	if (ret)
		return 0;

	curr = ((val & ITHSENS) + 1) * 10;

	/* Getting and calculating the thermistor resistance in ohms*/
	res = volt * 1000 / curr;

	/*calculating temperature*/
	for (temp = 58; temp >= 0; temp--) {
		int actual = therm_tbl[temp];
		if ((actual - res) >= 0)
			break;
	}

	/* Negative temperature */
	if (temp < 3) {
		if (temp == 2)
			temp = -1;
		else if (temp == 1)
			temp = -2;
		else
			temp = -3;
	}

	return temp + 1;
}

/*
 * Return battery voltage
 * Or < 0 on failure.
 */
static int twl4030battery_voltage(void)
{
	int volt = read_bci_val(T2_BATTERY_VOLT);

	return (volt * VOLT_STEP_SIZE) / VOLT_PSR_R;
}

/*
 * Return the battery current
 * Or < 0 on failure.
 */
static int twl4030battery_current(void)
{
	int ret, curr = read_bci_val(T2_BATTERY_CUR);
	u8 val;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, &val,
		REG_BCICTL1);
	if (ret)
		return ret;

	if (val & CGAIN) /* slope of 0.44 mV/mA */
		return (curr * CURR_STEP_SIZE) / CURR_PSR_R1;
	else /* slope of 0.88 mV/mA */
		return (curr * CURR_STEP_SIZE) / CURR_PSR_R2;
}

/*
 * Return the battery backup voltage
 * Or < 0 on failure.
 */
static int twl4030backupbatt_voltage(void)
{
	struct twl4030_madc_request req;
	int temp;

	req.channels = (1 << 9);
	req.do_avg = 0;
	req.method = TWL4030_MADC_SW1;
	req.active = 0;
	req.func_cb = NULL;
	twl4030_madc_conversion(&req);
	temp = (u16)req.rbuf[9];

	return  (temp * BK_VOLT_STEP_SIZE) / BK_VOLT_PSR_R;
}

/*
 * Returns an integer value, that means,
 * NO_PW_CONN  no power supply is connected
 * AC_PW_CONN  if the AC power supply is connected
 * USB_PW_CONN  if the USB power supply is connected
 * AC_PW_CONN + USB_PW_CONN if USB and AC power supplies are both connected
 *
 * Or < 0 on failure.
 */
static int twl4030charger_presence(void)
{
	int ret;
	u8 hwsts;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &hwsts,
		REG_STS_HW_CONDITIONS);
	if (ret) {
		pr_err("twl4030_bci: error reading STS_HW_CONDITIONS\n");
		return ret;
	}

	ret = (hwsts & STS_CHG) ? AC_PW_CONN : NO_PW_CONN;
	ret += (hwsts & STS_VBUS) ? USB_PW_CONN : NO_PW_CONN;

	if (ret & USB_PW_CONN)
		usb_charger_flag = 1;
	else
		usb_charger_flag = 0;

	return ret;

}

/*
 * Returns the main charge FSM status
 * Or < 0 on failure.
 */
static int twl4030bci_status(void)
{
	int ret;
	u8 status;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE,
		&status, REG_BCIMSTATEC);
	if (ret) {
		pr_err("twl4030_bci: error reading BCIMSTATEC\n");
		return ret;
	}

	return (int) (status & BCIMSTAT_MASK);
}

static int read_bci_val(u8 reg)
{
	int ret, temp;
	u8 val;

	/* reading MSB */
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, &val,
		reg + 1);
	if (ret)
		return ret;

	temp = ((int)(val & 0x03)) << 8;

	/* reading LSB */
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, &val,
		reg);
	if (ret)
		return ret;

	return temp | val;
}

/*
 * Settup the twl4030 BCI module to enable backup
 * battery charging.
 */
static int twl4030backupbatt_voltage_setup(void)
{
	int ret;

	/* Starting backup batery charge */
	ret = clear_n_set(TWL4030_MODULE_PM_RECEIVER, 0, BBCHEN,
		REG_BB_CFG);
	if (ret)
		return ret;

	return 0;
}

/*
 * Settup the twl4030 BCI module to measure battery
 * temperature
 */
static int twl4030battery_temp_setup(void)
{
	int ret;

	/* Enabling thermistor current */
	ret = clear_n_set(TWL4030_MODULE_MAIN_CHARGE, 0, ITHEN,
		REG_BCICTL1);
	if (ret)
		return ret;

	return 0;
}

/*
 * Sets and clears bits on an given register on a given module
 */
static inline int clear_n_set(u8 mod_no, u8 clear, u8 set, u8 reg)
{
	int ret;
	u8 val = 0;

	/* Gets the initial register value */
	ret = twl4030_i2c_read_u8(mod_no, &val, reg);
	if (ret)
		return ret;

	/* Clearing all those bits to clear */
	val &= ~(clear);

	/* Setting all those bits to set */
	val |= set;

	/* Update the register */
	ret = twl4030_i2c_write_u8(mod_no, val, reg);
	if (ret)
		return ret;

	return 0;
}

static enum power_supply_property twl4030_bci_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static enum power_supply_property twl4030_bk_bci_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static void
twl4030_bk_bci_battery_read_status(struct twl4030_bci_device_info *di)
{
	di->bk_voltage_uV = twl4030backupbatt_voltage();
}

static void twl4030_bk_bci_battery_work(struct work_struct *work)
{
	struct twl4030_bci_device_info *di = container_of(work,
		struct twl4030_bci_device_info,
		twl4030_bk_bci_monitor_work.work);

	twl4030_bk_bci_battery_read_status(di);
	schedule_delayed_work(&di->twl4030_bk_bci_monitor_work, 500);
}

static void twl4030_bci_battery_read_status(struct twl4030_bci_device_info *di)
{
	di->temp_C = twl4030battery_temperature();
	di->voltage_uV = twl4030battery_voltage();
	di->current_uA = twl4030battery_current();
}

static void
twl4030_bci_battery_update_status(struct twl4030_bci_device_info *di)
{
	twl4030_bci_battery_read_status(di);
	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;

	if (power_supply_am_i_supplied(&di->bat))
		di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	else
		di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
}

static void twl4030_bci_battery_work(struct work_struct *work)
{
	struct twl4030_bci_device_info *di = container_of(work,
		struct twl4030_bci_device_info, twl4030_bci_monitor_work.work);

	twl4030_bci_battery_update_status(di);
	schedule_delayed_work(&di->twl4030_bci_monitor_work, 100);
}


#define to_twl4030_bci_device_info(x) container_of((x), \
			struct twl4030_bci_device_info, bat);

static void twl4030_bci_battery_external_power_changed(struct power_supply *psy)
{
	struct twl4030_bci_device_info *di = to_twl4030_bci_device_info(psy);

	cancel_delayed_work(&di->twl4030_bci_monitor_work);
	schedule_delayed_work(&di->twl4030_bci_monitor_work, 0);
}

#define to_twl4030_bk_bci_device_info(x) container_of((x), \
		struct twl4030_bci_device_info, bk_bat);

static int twl4030_bk_bci_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct twl4030_bci_device_info *di = to_twl4030_bk_bci_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->bk_voltage_uV;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int twl4030_bci_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct twl4030_bci_device_info *di;
	int status = 0;

	di = to_twl4030_bci_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->charge_status;
		return 0;
	default:
		break;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage_uV;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->temp_C;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		status = twl4030bci_status();
		if ((status & AC_STATEC) == AC_STATEC)
			val->intval = POWER_SUPPLY_TYPE_MAINS;
		else if (usb_charger_flag)
			val->intval = POWER_SUPPLY_TYPE_USB;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/*
		 * need to get the correct percentage value per the
		 * battery characteristics. Approx values for now.
		 */
		if (di->voltage_uV < 2894 || LVL_1) {
			val->intval = 5;
			LVL_1 = 0;
		} else if ((di->voltage_uV < 3451 && di->voltage_uV > 2894)
			|| LVL_2) {
			val->intval = 20;
			LVL_2 = 0;
		} else if ((di->voltage_uV < 3902 && di->voltage_uV > 3451)
			|| LVL_3) {
			val->intval = 50;
			LVL_3 = 0;
		} else if ((di->voltage_uV < 3949 && di->voltage_uV > 3902)
			|| LVL_4) {
			val->intval = 75;
			LVL_4 = 0;
		} else if (di->voltage_uV > 3949)
			val->intval = 90;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static char *twl4030_bci_supplied_to[] = {
	"twl4030_bci_battery",
};

static int __init twl4030_bci_battery_probe(struct platform_device *pdev)
{
	struct twl4030_bci_platform_data *pdata = pdev->dev.platform_data;
	struct twl4030_bci_device_info *di;
	int irq;
	int ret;

	therm_tbl = pdata->battery_tmp_tbl;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->dev = &pdev->dev;
	di->bat.name = "twl4030_bci_battery";
	di->bat.supplied_to = twl4030_bci_supplied_to;
	di->bat.num_supplicants = ARRAY_SIZE(twl4030_bci_supplied_to);
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = twl4030_bci_battery_props;
	di->bat.num_properties = ARRAY_SIZE(twl4030_bci_battery_props);
	di->bat.get_property = twl4030_bci_battery_get_property;
	di->bat.external_power_changed =
			twl4030_bci_battery_external_power_changed;

	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;

	di->bk_bat.name = "twl4030_bci_bk_battery";
	di->bk_bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bk_bat.properties = twl4030_bk_bci_battery_props;
	di->bk_bat.num_properties = ARRAY_SIZE(twl4030_bk_bci_battery_props);
	di->bk_bat.get_property = twl4030_bk_bci_battery_get_property;
	di->bk_bat.external_power_changed = NULL;

	twl4030charger_ac_en(ENABLE);
	twl4030charger_usb_en(ENABLE);
	twl4030battery_hw_level_en(ENABLE);
	twl4030battery_hw_presence_en(ENABLE);

	platform_set_drvdata(pdev, di);

	/* settings for temperature sensing */
	ret = twl4030battery_temp_setup();
	if (ret)
		goto temp_setup_fail;

	/* enabling GPCH09 for read back battery voltage */
	ret = twl4030backupbatt_voltage_setup();
	if (ret)
		goto voltage_setup_fail;

	/* REVISIT do we need to request both IRQs ?? */

	/* request BCI interruption */
	irq = platform_get_irq(pdev, 1);
	ret = request_irq(irq, twl4030battery_interrupt,
		0, pdev->name, NULL);
	if (ret) {
		dev_dbg(&pdev->dev, "could not request irq %d, status %d\n",
			irq, ret);
		goto batt_irq_fail;
	}

	/* request Power interruption */
	irq = platform_get_irq(pdev, 0);
	ret = request_irq(irq, twl4030charger_interrupt,
		0, pdev->name, di);

	if (ret) {
		dev_dbg(&pdev->dev, "could not request irq %d, status %d\n",
			irq, ret);
		goto chg_irq_fail;
	}

	ret = power_supply_register(&pdev->dev, &di->bat);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register main battery\n");
		goto batt_failed;
	}

	INIT_DELAYED_WORK_DEFERRABLE(&di->twl4030_bci_monitor_work,
				twl4030_bci_battery_work);
	schedule_delayed_work(&di->twl4030_bci_monitor_work, 0);

	ret = power_supply_register(&pdev->dev, &di->bk_bat);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register backup battery\n");
		goto bk_batt_failed;
	}

	INIT_DELAYED_WORK_DEFERRABLE(&di->twl4030_bk_bci_monitor_work,
				twl4030_bk_bci_battery_work);
	schedule_delayed_work(&di->twl4030_bk_bci_monitor_work, 500);

	return 0;

bk_batt_failed:
	power_supply_unregister(&di->bat);
batt_failed:
	free_irq(irq, di);
chg_irq_fail:
	irq = platform_get_irq(pdev, 1);
	free_irq(irq, NULL);
batt_irq_fail:
voltage_setup_fail:
temp_setup_fail:
	twl4030charger_ac_en(DISABLE);
	twl4030charger_usb_en(DISABLE);
	twl4030battery_hw_level_en(DISABLE);
	twl4030battery_hw_presence_en(DISABLE);
	kfree(di);

	return ret;
}

static int __exit twl4030_bci_battery_remove(struct platform_device *pdev)
{
	struct twl4030_bci_device_info *di = platform_get_drvdata(pdev);
	int irq;

	twl4030charger_ac_en(DISABLE);
	twl4030charger_usb_en(DISABLE);
	twl4030battery_hw_level_en(DISABLE);
	twl4030battery_hw_presence_en(DISABLE);

	irq = platform_get_irq(pdev, 0);
	free_irq(irq, di);

	irq = platform_get_irq(pdev, 1);
	free_irq(irq, NULL);

	flush_scheduled_work();
	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->bk_bat);
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return 0;
}

#ifdef CONFIG_PM
static int twl4030_bci_battery_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct twl4030_bci_device_info *di = platform_get_drvdata(pdev);

	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	cancel_delayed_work(&di->twl4030_bci_monitor_work);
	cancel_delayed_work(&di->twl4030_bk_bci_monitor_work);
	return 0;
}

static int twl4030_bci_battery_resume(struct platform_device *pdev)
{
	struct twl4030_bci_device_info *di = platform_get_drvdata(pdev);

	schedule_delayed_work(&di->twl4030_bci_monitor_work, 0);
	schedule_delayed_work(&di->twl4030_bk_bci_monitor_work, 50);
	return 0;
}
#else
#define twl4030_bci_battery_suspend	NULL
#define twl4030_bci_battery_resume	NULL
#endif /* CONFIG_PM */

static struct platform_driver twl4030_bci_battery_driver = {
	.probe		= twl4030_bci_battery_probe,
	.remove		= __exit_p(twl4030_bci_battery_remove),
	.suspend	= twl4030_bci_battery_suspend,
	.resume		= twl4030_bci_battery_resume,
	.driver		= {
		.name	= "twl4030_bci",
	},
};

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:twl4030_bci");
MODULE_AUTHOR("Texas Instruments Inc");

static int __init twl4030_battery_init(void)
{
	return platform_driver_register(&twl4030_bci_battery_driver);
}
module_init(twl4030_battery_init);

static void __exit twl4030_battery_exit(void)
{
	platform_driver_unregister(&twl4030_bci_battery_driver);
}
module_exit(twl4030_battery_exit);

