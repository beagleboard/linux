/*
 * drivers/i2c/chips/menelaus.c
 *
 * Copyright (C) 2004 Texas Instruments, Inc.
 *
 * Some parts based tps65010.c:
 * Copyright (C) 2004 Texas Instruments and
 * Copyright (C) 2004-2005 David Brownell
 *
 * Some parts based on tlv320aic24.c:
 * Copyright (C) by Kai Svahn <kai.svahn@nokia.com>
 *
 * Changes for interrupt handling and clean-up by
 * Tony Lindgren <tony@atomide.com> and Imre Deak <imre.deak@nokia.com>
 * Cleanup and generalized support for voltage setting by
 * Juha Yrjola
 * Added support for controlling VCORE and regulator sleep states,
 * Amit Kucheria <amit.kucheria@nokia.com>
 * Copyright (C) 2005, 2006 Nokia Corporation
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
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/menelaus.h>

#define DEBUG

#define DRIVER_NAME			"menelaus"

#define pr_err(fmt, arg...)	printk(KERN_ERR DRIVER_NAME ": ", ## arg);

#define MENELAUS_I2C_ADDRESS		0x72

#define MENELAUS_REV			0x01
#define MENELAUS_VCORE_CTRL1		0x02
#define MENELAUS_VCORE_CTRL2		0x03
#define MENELAUS_VCORE_CTRL3		0x04
#define MENELAUS_VCORE_CTRL4		0x05
#define MENELAUS_VCORE_CTRL5		0x06
#define MENELAUS_DCDC_CTRL1		0x07
#define MENELAUS_DCDC_CTRL2		0x08
#define MENELAUS_DCDC_CTRL3		0x09
#define MENELAUS_LDO_CTRL1		0x0A
#define MENELAUS_LDO_CTRL2		0x0B
#define MENELAUS_LDO_CTRL3		0x0C
#define MENELAUS_LDO_CTRL4		0x0D
#define MENELAUS_LDO_CTRL5		0x0E
#define MENELAUS_LDO_CTRL6		0x0F
#define MENELAUS_LDO_CTRL7		0x10
#define MENELAUS_LDO_CTRL8		0x11
#define MENELAUS_SLEEP_CTRL1		0x12
#define MENELAUS_SLEEP_CTRL2		0x13
#define MENELAUS_DEVICE_OFF		0x14
#define MENELAUS_OSC_CTRL		0x15
#define MENELAUS_DETECT_CTRL		0x16
#define MENELAUS_INT_MASK1		0x17
#define MENELAUS_INT_MASK2		0x18
#define MENELAUS_INT_STATUS1		0x19
#define MENELAUS_INT_STATUS2		0x1A
#define MENELAUS_INT_ACK1		0x1B
#define MENELAUS_INT_ACK2		0x1C
#define MENELAUS_GPIO_CTRL		0x1D
#define MENELAUS_GPIO_IN		0x1E
#define MENELAUS_GPIO_OUT		0x1F
#define MENELAUS_BBSMS			0x20
#define MENELAUS_RTC_CTRL		0x21
#define MENELAUS_RTC_UPDATE		0x22
#define MENELAUS_RTC_SEC		0x23
#define MENELAUS_RTC_MIN		0x24
#define MENELAUS_RTC_HR			0x25
#define MENELAUS_RTC_DAY		0x26
#define MENELAUS_RTC_MON		0x27
#define MENELAUS_RTC_YR			0x28
#define MENELAUS_RTC_WKDAY		0x29
#define MENELAUS_RTC_AL_SEC		0x2A
#define MENELAUS_RTC_AL_MIN		0x2B
#define MENELAUS_RTC_AL_HR		0x2C
#define MENELAUS_RTC_AL_DAY		0x2D
#define MENELAUS_RTC_AL_MON		0x2E
#define MENELAUS_RTC_AL_YR		0x2F
#define MENELAUS_RTC_COMP_MSB		0x30
#define MENELAUS_RTC_COMP_LSB		0x31
#define MENELAUS_S1_PULL_EN		0x32
#define MENELAUS_S1_PULL_DIR		0x33
#define MENELAUS_S2_PULL_EN		0x34
#define MENELAUS_S2_PULL_DIR		0x35
#define MENELAUS_MCT_CTRL1		0x36
#define MENELAUS_MCT_CTRL2		0x37
#define MENELAUS_MCT_CTRL3		0x38
#define MENELAUS_MCT_PIN_ST		0x39
#define MENELAUS_DEBOUNCE1		0x3A

#define IH_MENELAUS_IRQS		12
#define MENELAUS_MMC_S1CD_IRQ		0	/* MMC slot 1 card change */
#define MENELAUS_MMC_S2CD_IRQ		1	/* MMC slot 2 card change */
#define MENELAUS_MMC_S1D1_IRQ		2	/* MMC DAT1 low in slot 1 */
#define MENELAUS_MMC_S2D1_IRQ		3	/* MMC DAT1 low in slot 2 */
#define MENELAUS_LOWBAT_IRQ		4	/* Low battery */
#define MENELAUS_HOTDIE_IRQ		5	/* Hot die detect */
#define MENELAUS_UVLO_IRQ		6	/* UVLO detect */
#define MENELAUS_TSHUT_IRQ		7	/* Thermal shutdown */
#define MENELAUS_RTCTMR_IRQ		8	/* RTC timer */
#define MENELAUS_RTCALM_IRQ		9	/* RTC alarm */
#define MENELAUS_RTCERR_IRQ		10	/* RTC error */
#define MENELAUS_PSHBTN_IRQ		11	/* Push button */
#define MENELAUS_RESERVED12_IRQ		12	/* Reserved */
#define MENELAUS_RESERVED13_IRQ		13	/* Reserved */
#define MENELAUS_RESERVED14_IRQ		14	/* Reserved */
#define MENELAUS_RESERVED15_IRQ		15	/* Reserved */

static void menelaus_work(struct work_struct *_menelaus);

/* Initialized by menelaus_init */
static unsigned short normal_i2c[] = { MENELAUS_I2C_ADDRESS, I2C_CLIENT_END };

I2C_CLIENT_INSMOD;

struct menelaus_chip {
	unsigned long		initialized;
	struct mutex		lock;
	struct i2c_client	client;
	struct work_struct	work;
	int			irq;
	unsigned		vcore_hw_mode:1;
	void			*handlers[16];
	void			(*mmc_callback)(void *data, u8 mask);
	void			*mmc_callback_data;
};

static struct menelaus_chip menelaus;
static struct menelaus_platform_data *menelaus_pdata;

static int menelaus_write_reg(int reg, u8 value)
{
	int val = i2c_smbus_write_byte_data(&menelaus.client, reg, value);

	if (val < 0) {
		pr_err("write error");
		return val;
	}

	return 0;
}

static int menelaus_read_reg(int reg)
{
	int val = i2c_smbus_read_byte_data(&menelaus.client, reg);

	if (val < 0)
		pr_err("read error");

	return val;
}

static int menelaus_enable_irq(int irq)
{
	if (irq > 7)
		return menelaus_write_reg(MENELAUS_INT_MASK2,
					  menelaus_read_reg(MENELAUS_INT_MASK2)
					  & ~(1 << (irq - 8)));
	else
		return menelaus_write_reg(MENELAUS_INT_MASK1,
					  menelaus_read_reg(MENELAUS_INT_MASK1)
					  & ~(1 << irq));
}

static int menelaus_disable_irq(int irq)
{
	if (irq > 7)
		return menelaus_write_reg(menelaus_read_reg(MENELAUS_INT_MASK2)
					  | (1 << (irq - 8)),
					  MENELAUS_INT_MASK2);
	else
		return menelaus_write_reg(MENELAUS_INT_MASK1,
					  menelaus_read_reg(MENELAUS_INT_MASK1)
					  | (1 << irq));
}

static int menelaus_ack_irq(int irq)
{
	if (irq > 7)
		return menelaus_write_reg(MENELAUS_INT_ACK2, 1 << (irq - 8));
	else
		return menelaus_write_reg(MENELAUS_INT_ACK1, 1 << irq);
}

/* Adds a handler for an interrupt. Does not run in interrupt context */
static int menelaus_add_irq_work(int irq, void * handler)
{
	int ret = 0;

	mutex_lock(&menelaus.lock);
	menelaus.handlers[irq] = handler;
	ret = menelaus_enable_irq(irq);
	mutex_unlock(&menelaus.lock);

	return ret;
}

/* Removes handler for an interrupt */
static int menelaus_remove_irq_work(int irq)
{
	int ret = 0;

	mutex_lock(&menelaus.lock);
	ret = menelaus_disable_irq(irq);
	menelaus.handlers[irq] = NULL;
	mutex_unlock(&menelaus.lock);

	return ret;
}

/*
 * Gets scheduled when a card detect interrupt happens. Note that in some cases
 * this line is wired to card cover switch rather than the card detect switch
 * in each slot. In this case the cards are not seen by menelaus.
 * FIXME: Add handling for D1 too
 */
static int menelaus_mmc_cd_work(struct menelaus_chip * menelaus_hw)
{
	int reg;
	unsigned char card_mask = 0;

	reg = menelaus_read_reg(MENELAUS_MCT_PIN_ST);
	if (reg < 0)
		return reg;

	if (!(reg & 0x1))
		card_mask |= (1 << 0);

	if (!(reg & 0x2))
		card_mask |= (1 << 1);

	if (menelaus_hw->mmc_callback)
		menelaus_hw->mmc_callback(menelaus_hw->mmc_callback_data,
					  card_mask);

	return 0;
}

/*
 * Toggles the MMC slots between open-drain and push-pull mode.
 */
int menelaus_set_mmc_opendrain(int slot, int enable)
{
	int ret, val;

	if (slot != 1 && slot != 2)
		return -EINVAL;
	mutex_lock(&menelaus.lock);
	ret = menelaus_read_reg(MENELAUS_MCT_CTRL1);
	if (ret < 0) {
		mutex_unlock(&menelaus.lock);
		return ret;
	}
	val = ret;
	if (slot == 1) {
		if (enable)
			val |= 1 << 2;
		else
			val &= ~(1 << 2);
	} else {
		if (enable)
			val |= 1 << 3;
		else
			val &= ~(1 << 3);
	}
	ret = menelaus_write_reg(MENELAUS_MCT_CTRL1, val);
	mutex_unlock(&menelaus.lock);

	return ret;
}
EXPORT_SYMBOL(menelaus_set_mmc_opendrain);

int menelaus_set_slot_sel(int enable)
{
	int ret;

	mutex_lock(&menelaus.lock);
	ret = menelaus_read_reg(MENELAUS_GPIO_CTRL);
	if (ret < 0)
		goto out;
	ret |= 0x02;
	if (enable)
		ret |= 1 << 5;
	else
		ret &= ~(1 << 5);
	ret = menelaus_write_reg(MENELAUS_GPIO_CTRL, ret);
out:
	mutex_unlock(&menelaus.lock);
	return ret;
}
EXPORT_SYMBOL(menelaus_set_slot_sel);

int menelaus_set_mmc_slot(int slot, int enable, int power, int cd_en)
{
	int ret, val;

	if (slot != 1 && slot != 2)
		return -EINVAL;
	if (power >= 3)
		return -EINVAL;

	mutex_lock(&menelaus.lock);

	ret = menelaus_read_reg(MENELAUS_MCT_CTRL2);
	if (ret < 0)
		goto out;
	val = ret;
	if (slot == 1) {
		if (cd_en)
			val |= (1 << 4) | (1 << 6);
		else
			val &= ~((1 << 4) | (1 << 6));
	} else {
		if (cd_en)
			val |= (1 << 5) | (1 << 7);
		else
			val &= ~((1 << 5) | (1 << 7));
	}
	ret = menelaus_write_reg(MENELAUS_MCT_CTRL2, val);
	if (ret < 0)
		goto out;

	ret = menelaus_read_reg(MENELAUS_MCT_CTRL3);
	if (ret < 0)
		goto out;
	val = ret;
	if (slot == 1) {
		if (enable)
			val |= 1 << 0;
		else
			val &= ~(1 << 0);
	} else {
		int b;

		if (enable)
			ret |= 1 << 1;
		else
			ret &= ~(1 << 1);
		b = menelaus_read_reg(MENELAUS_MCT_CTRL2);
		b &= ~0x03;
		b |= power;
		ret = menelaus_write_reg(MENELAUS_MCT_CTRL2, b);
		if (ret < 0)
			goto out;
	}
	/* Disable autonomous shutdown */
	val &= ~(0x03 << 2);
	ret = menelaus_write_reg(MENELAUS_MCT_CTRL3, val);
out:
	mutex_unlock(&menelaus.lock);
	return ret;
}
EXPORT_SYMBOL(menelaus_set_mmc_slot);

#include <linux/delay.h>

int menelaus_register_mmc_callback(void (*callback)(void *data, u8 card_mask),
				   void *data)
{
	int ret = 0;

	menelaus.mmc_callback_data = data;
	menelaus.mmc_callback = callback;
	ret = menelaus_add_irq_work(MENELAUS_MMC_S1CD_IRQ,
				    menelaus_mmc_cd_work);
	if (ret < 0)
		return ret;
	ret = menelaus_add_irq_work(MENELAUS_MMC_S2CD_IRQ,
				    menelaus_mmc_cd_work);
	if (ret < 0)
		return ret;
	ret = menelaus_add_irq_work(MENELAUS_MMC_S1D1_IRQ,
				    menelaus_mmc_cd_work);
	if (ret < 0)
		return ret;
	ret = menelaus_add_irq_work(MENELAUS_MMC_S2D1_IRQ,
				    menelaus_mmc_cd_work);

	return ret;
}
EXPORT_SYMBOL(menelaus_register_mmc_callback);

void menelaus_unregister_mmc_callback(void)
{
	menelaus_remove_irq_work(MENELAUS_MMC_S1CD_IRQ);
	menelaus_remove_irq_work(MENELAUS_MMC_S2CD_IRQ);
	menelaus_remove_irq_work(MENELAUS_MMC_S1D1_IRQ);
	menelaus_remove_irq_work(MENELAUS_MMC_S2D1_IRQ);

	menelaus.mmc_callback = NULL;
	menelaus.mmc_callback_data = 0;
}
EXPORT_SYMBOL(menelaus_unregister_mmc_callback);

struct menelaus_vtg {
	const char *name;
	u8 vtg_reg;
	u8 vtg_shift;
	u8 vtg_bits;
	u8 mode_reg;
};

struct menelaus_vtg_value {
	u16 vtg;
	u16 val;
};

static int menelaus_set_voltage(const struct menelaus_vtg *vtg, int mV,
				int vtg_val, int mode)
{
	int val, ret;

	mutex_lock(&menelaus.lock);
	if (vtg == 0)
		goto set_voltage;

	ret = menelaus_read_reg(vtg->vtg_reg);
	if (ret < 0)
		goto out;
	val = ret & ~(((1 << vtg->vtg_bits) - 1) << vtg->vtg_shift);
	val |= vtg_val << vtg->vtg_shift;
#ifdef DEBUG
	printk("menelaus: Setting voltage '%s' to %d mV (reg 0x%02x, val 0x%02x)\n",
	       vtg->name, mV, vtg->vtg_reg, val);
#endif
	ret = menelaus_write_reg(vtg->vtg_reg, val);
	if (ret < 0)
		goto out;
set_voltage:
	ret = menelaus_write_reg(vtg->mode_reg, mode);
out:
	mutex_unlock(&menelaus.lock);
	if (ret == 0) {
		/* Wait for voltage to stabilize */
		msleep(1);
	}
	return ret;
}

static int menelaus_get_vtg_value(int vtg, const struct menelaus_vtg_value *tbl,
				  int n)
{
	int i;

	for (i = 0; i < n; i++, tbl++)
		if (tbl->vtg == vtg)
			return tbl->val;
	return -EINVAL;
}

/* Vcore can be programmed in two ways:
 * SW-controlled: Required voltage is programmed into VCORE_CTRL1
 * HW-controlled: Required range (roof-floor) is programmed into VCORE_CTRL3
 * and VCORE_CTRL4

 * Call correct 'set' function accordingly
 */

static const struct menelaus_vtg_value vcore_values[] = {
	{ 1000, 0 },
	{ 1025, 1 },
	{ 1050, 2 },
	{ 1075, 3 },
	{ 1100, 4 },
	{ 1125, 5 },
	{ 1150, 6 },
	{ 1175, 7 },
	{ 1200, 8 },
	{ 1225, 9 },
	{ 1250, 10 },
	{ 1275, 11 },
	{ 1300, 12 },
	{ 1325, 13 },
	{ 1350, 14 },
	{ 1375, 15 },
	{ 1400, 16 },
	{ 1425, 17 },
	{ 1450, 18 },
};

int menelaus_set_vcore_sw(unsigned int mV)
{
	int val, ret;

	val = menelaus_get_vtg_value(mV, vcore_values, ARRAY_SIZE(vcore_values));
	if (val < 0)
		return -EINVAL;
#ifdef DEBUG
	printk("menelaus: Setting VCORE to %d mV (val 0x%02x)\n", mV, val);
#endif

	/* Set SW mode and the voltage in one go. */
	mutex_lock(&menelaus.lock);
	ret = menelaus_write_reg(MENELAUS_VCORE_CTRL1, val);
	if (ret == 0)
		menelaus.vcore_hw_mode = 0;
	mutex_unlock(&menelaus.lock);
	msleep(1);

	return ret;
}

int menelaus_set_vcore_hw(unsigned int roof_mV, unsigned int floor_mV)
{
	int fval, rval, val, ret;

	rval = menelaus_get_vtg_value(roof_mV, vcore_values, ARRAY_SIZE(vcore_values));
	if (rval < 0)
		return -EINVAL;
	fval = menelaus_get_vtg_value(floor_mV, vcore_values, ARRAY_SIZE(vcore_values));
	if (fval < 0)
		return -EINVAL;

#ifdef DEBUG
	printk("menelaus: Setting VCORE FLOOR to %d mV and ROOF to %d mV\n",
	       floor_mV, roof_mV);
#endif

	mutex_lock(&menelaus.lock);
	ret = menelaus_write_reg(MENELAUS_VCORE_CTRL3, fval);
	if (ret < 0)
		goto out;
	ret = menelaus_write_reg(MENELAUS_VCORE_CTRL4, rval);
	if (ret < 0)
		goto out;
	if (!menelaus.vcore_hw_mode) {
		val = menelaus_read_reg(MENELAUS_VCORE_CTRL1);
		val |= ((1 << 7) | (1 << 5)); /* HW mode, turn OFF byte comparator */
		ret = menelaus_write_reg(MENELAUS_VCORE_CTRL1, val);
		menelaus.vcore_hw_mode = 1;
	}
	msleep(1);
out:
	mutex_unlock(&menelaus.lock);
	return ret;
}

static const struct menelaus_vtg vmem_vtg = {
	.name = "VMEM",
	.vtg_reg = MENELAUS_LDO_CTRL1,
	.vtg_shift = 0,
	.vtg_bits = 2,
	.mode_reg = MENELAUS_LDO_CTRL3,
};

static const struct menelaus_vtg_value vmem_values[] = {
	{ 1500, 0 },
	{ 1800, 1 },
	{ 1900, 2 },
	{ 2500, 3 },
};

int menelaus_set_vmem(unsigned int mV)
{
	int val;

	if (mV == 0)
		return menelaus_set_voltage(&vmem_vtg, 0, 0, 0);

	val = menelaus_get_vtg_value(mV, vmem_values, ARRAY_SIZE(vmem_values));
	if (val < 0)
		return -EINVAL;
	return menelaus_set_voltage(&vmem_vtg, mV, val, 0x02);
}
EXPORT_SYMBOL(menelaus_set_vmem);

static const struct menelaus_vtg vio_vtg = {
	.name = "VIO",
	.vtg_reg = MENELAUS_LDO_CTRL1,
	.vtg_shift = 2,
	.vtg_bits = 2,
	.mode_reg = MENELAUS_LDO_CTRL4,
};

static const struct menelaus_vtg_value vio_values[] = {
	{ 1500, 0 },
	{ 1800, 1 },
	{ 2500, 2 },
	{ 2800, 3 },
};

int menelaus_set_vio(unsigned int mV)
{
	int val;

	if (mV == 0)
		return menelaus_set_voltage(&vio_vtg, 0, 0, 0);

	val = menelaus_get_vtg_value(mV, vio_values, ARRAY_SIZE(vio_values));
	if (val < 0)
		return -EINVAL;
	return menelaus_set_voltage(&vio_vtg, mV, val, 0x02);
}
EXPORT_SYMBOL(menelaus_set_vio);

static const struct menelaus_vtg_value vdcdc_values[] = {
	{ 1500, 0 },
	{ 1800, 1 },
	{ 2000, 2 },
	{ 2200, 3 },
	{ 2400, 4 },
	{ 2800, 5 },
	{ 3000, 6 },
	{ 3300, 7 },
};

static const struct menelaus_vtg vdcdc2_vtg = {
	.name = "VDCDC2",
	.vtg_reg = MENELAUS_DCDC_CTRL1,
	.vtg_shift = 0,
	.vtg_bits = 3,
	.mode_reg = MENELAUS_DCDC_CTRL2,
};

static const struct menelaus_vtg vdcdc3_vtg = {
	.name = "VDCDC3",
	.vtg_reg = MENELAUS_DCDC_CTRL1,
	.vtg_shift = 3,
	.vtg_bits = 3,
	.mode_reg = MENELAUS_DCDC_CTRL3,
};

int menelaus_set_vdcdc(int dcdc, unsigned int mV)
{
	const struct menelaus_vtg *vtg;
	int val;

	if (dcdc != 2 && dcdc != 3)
		return -EINVAL;
	if (dcdc == 2)
		vtg = &vdcdc2_vtg;
	else
		vtg = &vdcdc3_vtg;

	if (mV == 0)
		return menelaus_set_voltage(vtg, 0, 0, 0);

	val = menelaus_get_vtg_value(mV, vdcdc_values, ARRAY_SIZE(vdcdc_values));
	if (val < 0)
		return -EINVAL;
	return menelaus_set_voltage(vtg, mV, val, 0x03);
}

static const struct menelaus_vtg_value vmmc_values[] = {
	{ 1850, 0 },
	{ 2800, 1 },
	{ 3000, 2 },
	{ 3100, 3 },
};

static const struct menelaus_vtg vmmc_vtg = {
	.name = "VMMC",
	.vtg_reg = MENELAUS_LDO_CTRL1,
	.vtg_shift = 6,
	.vtg_bits = 2,
	.mode_reg = MENELAUS_LDO_CTRL7,
};

int menelaus_set_vmmc(unsigned int mV)
{
	int val;

	if (mV == 0)
		return menelaus_set_voltage(&vmmc_vtg, 0, 0, 0);

	val = menelaus_get_vtg_value(mV, vmmc_values, ARRAY_SIZE(vmmc_values));
	if (val < 0)
		return -EINVAL;
	return menelaus_set_voltage(&vmmc_vtg, mV, val, 0x02);
}
EXPORT_SYMBOL(menelaus_set_vmmc);


static const struct menelaus_vtg_value vaux_values[] = {
	{ 1500, 0 },
	{ 1800, 1 },
	{ 2500, 2 },
	{ 2800, 3 },
};

static const struct menelaus_vtg vaux_vtg = {
	.name = "VAUX",
	.vtg_reg = MENELAUS_LDO_CTRL1,
	.vtg_shift = 4,
	.vtg_bits = 2,
	.mode_reg = MENELAUS_LDO_CTRL6,
};

int menelaus_set_vaux(unsigned int mV)
{
	int val;

	if (mV == 0)
		return menelaus_set_voltage(&vaux_vtg, 0, 0, 0);

	val = menelaus_get_vtg_value(mV, vaux_values, ARRAY_SIZE(vaux_values));
	if (val < 0)
		return -EINVAL;
	return menelaus_set_voltage(&vaux_vtg, mV, val, 0x02);
}
EXPORT_SYMBOL(menelaus_set_vaux);

int menelaus_get_slot_pin_states(void)
{
	return menelaus_read_reg(MENELAUS_MCT_PIN_ST);
}
EXPORT_SYMBOL(menelaus_get_slot_pin_states);

int menelaus_set_regulator_sleep(int enable, u32 val)
{
	int t, ret;

        mutex_lock(&menelaus.lock);
	ret = menelaus_write_reg(MENELAUS_SLEEP_CTRL2, val);
	if (ret < 0)
		goto out;
#ifdef DEBUG
	printk("menelaus: regulator sleep configuration: %02x\n", val);
#endif
	ret = menelaus_read_reg(MENELAUS_GPIO_CTRL);
	if (ret < 0)
		goto out;
	t = ((1 << 6) | 0x04);
	if (enable)
		ret |= t;
	else
		ret &= ~t;
	ret = menelaus_write_reg(MENELAUS_GPIO_CTRL, ret);
out:
	mutex_unlock(&menelaus.lock);
	return ret;
}

/*-----------------------------------------------------------------------*/

/* Handles Menelaus interrupts. Does not run in interrupt context */
static void menelaus_work(struct work_struct *_menelaus)
{
	struct menelaus_chip *menelaus =
			container_of(_menelaus, struct menelaus_chip, work);
	int (*handler)(struct menelaus_chip *menelaus);

	while (1) {
		int i;
		unsigned char isr;

		isr = menelaus_read_reg(MENELAUS_INT_STATUS1) |
		      (menelaus_read_reg(MENELAUS_INT_STATUS2) << 8);

		if (!isr)
			break;

		for (i = 0; i < IH_MENELAUS_IRQS; i++) {
			if (isr & (1 << i)) {
				mutex_lock(&menelaus->lock);
				menelaus_disable_irq(i);
				menelaus_ack_irq(i);
				if (menelaus->handlers[i]) {
					handler = menelaus->handlers[i];
					handler(menelaus);
				}
				menelaus_enable_irq(i);
				mutex_unlock(&menelaus->lock);
			}
		}
	}
	enable_irq(menelaus->irq);
}

/*
 * We cannot use I2C in interrupt context, so we just schedule work.
 */
static irqreturn_t menelaus_irq(int irq, void *_menelaus)
{
	struct menelaus_chip *menelaus = _menelaus;

	disable_irq_nosync(irq);
	(void)schedule_work(&menelaus->work);

	return IRQ_HANDLED;
}

static struct i2c_driver menelaus_i2c_driver;

static int menelaus_probe(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client	*c;
	int			rev = 0, val;
	int			err = 0;

	if (test_and_set_bit(0, &menelaus.initialized))
		return -EBUSY;

	c = &menelaus.client;
	strncpy(c->name, DRIVER_NAME, sizeof(c->name));
	c->addr		= address;
	c->adapter	= adapter;
	c->driver	= &menelaus_i2c_driver;
	c->flags	= 0;

	if ((err = i2c_attach_client(c)) < 0) {
		pr_err("couldn't attach\n");
		goto fail1;
	}

	/* If a true probe check the device */
	if (kind < 0 && (rev = menelaus_read_reg(MENELAUS_REV)) < 0) {
		pr_err("device not found");
		err = -ENODEV;
		goto fail2;
	}

	/* Most likely Menelaus interrupt is at SYS_NIRQ */
	omap_cfg_reg(W19_24XX_SYS_NIRQ);
	menelaus.irq = INT_24XX_SYS_NIRQ;

	/* Ack and disable all Menelaus interrupts */
	menelaus_write_reg(MENELAUS_INT_ACK1, 0xff);
	menelaus_write_reg(MENELAUS_INT_ACK2, 0xff);
	menelaus_write_reg(MENELAUS_INT_MASK1, 0xff);
	menelaus_write_reg(MENELAUS_INT_MASK2, 0xff);

	/* Set output buffer strengths */
	menelaus_write_reg(MENELAUS_MCT_CTRL1, 0x73);

	err = request_irq(menelaus.irq, menelaus_irq, IRQF_DISABLED,
			  DRIVER_NAME, &menelaus);
	if (err) {
		printk(KERN_ERR "Could not get Menelaus IRQ\n");
		goto fail2;
	}

	mutex_init(&menelaus.lock);
	INIT_WORK(&menelaus.work, menelaus_work);

	if (kind < 0)
		pr_info("Menelaus rev %d.%d\n", rev >> 4, rev & 0x0f);

	val = menelaus_read_reg(MENELAUS_VCORE_CTRL1);
	if (val < 0)
		goto fail3;
	if (val & (1 << 7))
		menelaus.vcore_hw_mode = 1;
	else
		menelaus.vcore_hw_mode = 0;

	if (menelaus_pdata != NULL && menelaus_pdata->late_init != NULL) {
		err = menelaus_pdata->late_init(&c->dev);
		if (err < 0)
			goto fail3;
	}

	return 0;
fail3:
	free_irq(menelaus.irq, &menelaus);
	flush_scheduled_work();
fail2:
	i2c_detach_client(c);
fail1:
	clear_bit(0, &menelaus.initialized);
	return err;
}

static int menelaus_remove(struct i2c_client *client)
{
	int err;

	free_irq(menelaus.irq, &menelaus);

	if ((err = i2c_detach_client(client))) {
		pr_err("client deregistration failed\n");
		return err;
	}

	clear_bit(0, &menelaus.initialized);

	return 0;
}

/*-----------------------------------------------------------------------*/

static int menelaus_scan_bus(struct i2c_adapter *bus)
{
	if (!i2c_check_functionality(bus, I2C_FUNC_SMBUS_BYTE_DATA |
					  I2C_FUNC_SMBUS_WRITE_BYTE)) {
		pr_err("invalid i2c bus functionality\n");
		return -EINVAL;
	}

	return i2c_probe(bus, &addr_data, menelaus_probe);
}

static struct i2c_driver menelaus_i2c_driver = {
	.driver = {
		.name		= DRIVER_NAME,
	},
	.id		= I2C_DRIVERID_MISC, /*FIXME:accroding to i2c-ids.h */
	.class		= I2C_CLASS_HWMON,
	.attach_adapter	= menelaus_scan_bus,
	.detach_client	= menelaus_remove,
};

static int __init menelaus_init(void)
{
	int res;

	if ((res = i2c_add_driver(&menelaus_i2c_driver)) < 0) {
		pr_err("driver registration failed\n");
		return res;
	}

	return 0;
}

static void __exit menelaus_exit(void)
{
	if (i2c_del_driver(&menelaus_i2c_driver) < 0)
		pr_err("driver remove failed\n");

	/* FIXME: Shutdown menelaus parts that can be shut down */
}

void __init menelaus_set_platform_data(struct menelaus_platform_data *pdata)
{
	menelaus_pdata = pdata;
}

MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_DESCRIPTION("I2C interface for Menelaus.");
MODULE_LICENSE("GPL");

module_init(menelaus_init);
module_exit(menelaus_exit);
