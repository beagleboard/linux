/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
 *
 * File Name		: lsm330_acc.c
 * Authors		: MSH - Motion Mems BU - Application Team
 *			: Matteo Dameno (matteo.dameno@st.com)
 *			: Denis Ciocca (denis.ciocca@st.com)
 *			: Author is willing to be considered the contact
 *			: and update point for the driver.
* Version		: V.1.0.2
* Date			: 2012/Oct/15
 * Description		: LSM330 accelerometer driver
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************
Version History.
	V 1.0.0		First Release
	V 1.0.2		I2C address bugfix
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

#include <linux/input/lsm330.h>
/*#include "lsm330.h"*/


//#define	DEBUG

#define G_MAX			23920640	/* ug */
#define	I2C_RETRY_DELAY		5		/* Waiting for signals [ms] */
#define	I2C_RETRIES		5		/* Number of retries */
#define	I2C_AUTO_INCREMENT	0x80		/* Autoincrement i2c address */

#define SENSITIVITY_2G		60		/* ug/LSB	*/
#define SENSITIVITY_4G		120		/* ug/LSB	*/
#define SENSITIVITY_6G		180		/* ug/LSB	*/
#define SENSITIVITY_8G		240		/* ug/LSB	*/
#define SENSITIVITY_16G		730		/* ug/LSB	*/

#define	LSM330_ACC_FS_MASK	(0x38)

/* Output Data Rates ODR */
#define	LSM330_ODR_MASK		0XF0
#define LSM330_PM_OFF		0x00		/* OFF */
#define	LSM330_ODR3_125		0x10		/*    3.125 Hz */
#define	LSM330_ODR6_25		0x20		/*    6.25  Hz */
#define	LSM330_ODR12_5		0x30		/*   12.5   Hz */
#define	LSM330_ODR25		0x40		/*   25     Hz */
#define	LSM330_ODR50		0x50		/*   50     Hz */
#define	LSM330_ODR100		0x60		/*  100     Hz */
#define	LSM330_ODR400		0x70		/*  400     Hz */
#define	LSM330_ODR800		0x80		/*  800     Hz */
#define	LSM330_ODR1600		0x90		/* 1600     Hz */

/* Registers configuration Mask and settings */
/* CTRLREG1 */
#define LSM330_INTEN_MASK		0x01
#define LSM330_INTEN_OFF		0x00
#define LSM330_INTEN_ON			0x01

/* CTRLREG2 */
#define LSM330_HIST1_MASK		0xE0
#define LSM330_SM1INT_PIN_MASK		0x08
#define LSM330_SM1INT_PINB		0x08
#define LSM330_SM1INT_PINA		0x00
#define LSM330_SM1_EN_MASK		0x01
#define LSM330_SM1_EN_ON		0x01
#define LSM330_SM1_EN_OFF		0x00
/* */

/* CTRLREG3 */
#define LSM330_HIST2_MASK		0xE0
#define LSM330_SM2INT_PIN_MASK		0x08
#define LSM330_SM2INT_PINB		0x08
#define LSM330_SM2INT_PINA		0x00
#define LSM330_SM2_EN_MASK		0x01
#define LSM330_SM2_EN_ON		0x01
#define LSM330_SM2_EN_OFF		0x00
/* */

/* CTRLREG4 */
#define LSM330_INT_ACT_MASK		(0x01 << 6)
#define LSM330_INT_ACT_H		(0x01 << 6)
#define LSM330_INT_ACT_L		0x00

#define LSM330_INT2_EN_MASK		(0x01 << 4)
#define LSM330_INT2_EN_ON		(0x01 << 4)
#define LSM330_INT2_EN_OFF		0x00

#define LSM330_INT1_EN_MASK		(0x01 << 3)
#define LSM330_INT1_EN_ON		(0x01 << 3)
#define LSM330_INT1_EN_OFF		0x00
/* */

#define	OUT_AXISDATA_REG		LSM330_OUTX_L
#define WHOAMI_LSM330_ACC		0x40	/* Expected content for WAI */

/*	CONTROL REGISTERS	*/
#define	LSM330_WHO_AM_I			0x0F	/* WhoAmI register Address */

#define	LSM330_OUTX_L			0x28	/* Output X LSByte */
#define	LSM330_OUTX_H			0x29	/* Output X MSByte */
#define	LSM330_OUTY_L			0x2A	/* Output Y LSByte */
#define	LSM330_OUTY_H			0x2B	/* Output Y MSByte */
#define	LSM330_OUTZ_L			0x2C	/* Output Z LSByte */
#define	LSM330_OUTZ_H			0x2D	/* Output Z MSByte */
#define	LSM330_LC_L			0x16	/* LSByte Long Counter Status */
#define	LSM330_LC_H			0x17	/* MSByte Long Counter Status */

#define	LSM330_STATUS_REG		0x27	/* Status */

#define	LSM330_CTRL_REG1		0x21	/* control reg 1 */
#define	LSM330_CTRL_REG2		0x22	/* control reg 2 */
#define	LSM330_CTRL_REG3		0x23	/* control reg 3 */
#define	LSM330_CTRL_REG4		0x20	/* control reg 4 */
#define	LSM330_CTRL_REG5		0x24	/* control reg 3 */
#define	LSM330_CTRL_REG6		0x25	/* control reg 4 */

#define	LSM330_OFF_X			0x10	/* Offset X Corr */
#define	LSM330_OFF_Y			0x11	/* Offset Y Corr */
#define	LSM330_OFF_Z			0x12	/* Offset Z Corr */

#define	LSM330_CS_X			0x13	/* Const Shift X */
#define	LSM330_CS_Y			0x14	/* Const Shift Y */
#define	LSM330_CS_Z			0x15	/* Const Shift Z */

#define	LSM330_VFC_1			0x1B	/* Vect Filter Coeff 1 */
#define	LSM330_VFC_2			0x1C	/* Vect Filter Coeff 2 */
#define	LSM330_VFC_3			0x1D	/* Vect Filter Coeff 3 */
#define	LSM330_VFC_4			0x1E	/* Vect Filter Coeff 4 */

/*	end CONTROL REGISTRES	*/


/* RESUME STATE INDICES */
#define	LSM330_RES_LC_L				0
#define	LSM330_RES_LC_H				1

#define	LSM330_RES_CTRL_REG1			2
#define	LSM330_RES_CTRL_REG2			3
#define	LSM330_RES_CTRL_REG3			4
#define	LSM330_RES_CTRL_REG4			5
#define	LSM330_RES_CTRL_REG5			6

#define	LSM330_RES_TIM4_1			20
#define	LSM330_RES_TIM3_1			21
#define	LSM330_RES_TIM2_1_L			22
#define	LSM330_RES_TIM2_1_H			23
#define	LSM330_RES_TIM1_1_L			24
#define	LSM330_RES_TIM1_1_H			25

#define	LSM330_RES_THRS2_1			26
#define	LSM330_RES_THRS1_1			27
#define	LSM330_RES_SA_1				28
#define	LSM330_RES_MA_1				29
#define	LSM330_RES_SETT_1			30

#define	LSM330_RES_TIM4_2			31
#define	LSM330_RES_TIM3_2			32
#define	LSM330_RES_TIM2_2_L			33
#define	LSM330_RES_TIM2_2_H			34
#define	LSM330_RES_TIM1_2_L			35
#define	LSM330_RES_TIM1_2_H			36

#define	LSM330_RES_THRS2_2			37
#define	LSM330_RES_THRS1_2			38
#define	LSM330_RES_DES_2			39
#define	LSM330_RES_SA_2				40
#define	LSM330_RES_MA_2				41
#define	LSM330_RES_SETT_2			42

#define	LSM330_RESUME_ENTRIES			43



#define	LSM330_STATE_PR_SIZE			16
/* end RESUME STATE INDICES */

/* STATE PROGRAMS ENABLE CONTROLS */
#define	LSM330_SM1_DIS_SM2_DIS			0x00
#define	LSM330_SM1_DIS_SM2_EN			0x01
#define	LSM330_SM1_EN_SM2_DIS			0x02
#define	LSM330_SM1_EN_SM2_EN			0x03

/* INTERRUPTS ENABLE CONTROLS */
#define	LSM330_INT1_DIS_INT2_DIS		0x00
#define	LSM330_INT1_DIS_INT2_EN			0x01
#define	LSM330_INT1_EN_INT2_DIS			0x02
#define	LSM330_INT1_EN_INT2_EN			0x03

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lsm330_acc_odr_table[] = {
		{    1, LSM330_ODR1600 },
		{    3, LSM330_ODR400  },
		{   10, LSM330_ODR100  },
		{   20, LSM330_ODR50   },
		{   40, LSM330_ODR25   },
		{   80, LSM330_ODR12_5 },
		{  160, LSM330_ODR6_25 },
		{  320, LSM330_ODR3_125},
};

static const struct lsm330_acc_platform_data default_lsm330_acc_pdata = {
	.fs_range = LSM330_ACC_G_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 100,
	.min_interval = LSM330_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LSM330_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = LSM330_ACC_DEFAULT_INT2_GPIO,
};

struct lsm330_acc_data {
	struct i2c_client *client;
	struct lsm330_acc_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;

	u16 sensitivity;
	u8 stateprogs_enable_setting;
	u8 interrupts_enable_setting;

	u8 resume_state[LSM330_RESUME_ENTRIES];
	u8 resume_stmach_program1[LSM330_STATE_PR_SIZE];
	u8 resume_stmach_program2[LSM330_STATE_PR_SIZE];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

#ifdef DEBUG
	u8 reg_addr;
#endif
};


/* sets default init values to be written in registers at probe stage */
static void lsm330_acc_set_init_register_values(struct lsm330_acc_data *acc)
{
	acc->resume_state[LSM330_RES_LC_L] = 0x00;
	acc->resume_state[LSM330_RES_LC_H] = 0x00;

	acc->resume_state[LSM330_RES_CTRL_REG1] = LSM330_INT_ACT_H;
	acc->resume_state[LSM330_RES_CTRL_REG2] = 0x00;
	acc->resume_state[LSM330_RES_CTRL_REG3] = 0x00;
	acc->resume_state[LSM330_RES_CTRL_REG4] = 0x8f;
	acc->resume_state[LSM330_RES_CTRL_REG5] = 0x00;
}

static int lsm330_acc_i2c_read(struct lsm330_acc_data *acc,
				u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lsm330_acc_i2c_write(struct lsm330_acc_data *acc, u8 * buf,
								int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lsm330_acc_i2c_update(struct lsm330_acc_data *acc,
				u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 rdbuf[1] = { reg_address };
	u8 wrbuf[2] = { reg_address , 0x00 };

	u8 init_val;
	u8 updated_val;
	err = lsm330_acc_i2c_read(acc, rdbuf, 1);
	if (!(err < 0)) {
		init_val = rdbuf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		wrbuf[1] = updated_val;
		err = lsm330_acc_i2c_write(acc, wrbuf, 1);
	}
	return err;
}

static int lsm330_acc_hw_init(struct lsm330_acc_data *acc)
{
	int err = -1;
	u8 buf[17];

	pr_info("%s: hw init start\n", LSM330_ACC_DEV_NAME);

	buf[0] = LSM330_WHO_AM_I;
	err = lsm330_acc_i2c_read(acc, buf, 1);
	if (err < 0) {
	dev_warn(&acc->client->dev, "Error reading WHO_AM_I: is device "
		"available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;

	if (buf[0] != WHOAMI_LSM330_ACC) {
	dev_err(&acc->client->dev,
		"device unknown. Expected: 0x%x,"
		" Replies: 0x%x\n", WHOAMI_LSM330_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}


	buf[0] = (I2C_AUTO_INCREMENT | LSM330_LC_L);
	buf[1] = acc->resume_state[LSM330_RES_LC_L];
	buf[2] = acc->resume_state[LSM330_RES_LC_H];
	err = lsm330_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG2);
	buf[1] = acc->resume_state[LSM330_RES_CTRL_REG2];
	buf[2] = acc->resume_state[LSM330_RES_CTRL_REG3];
	err = lsm330_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG4);
	buf[1] = acc->resume_state[LSM330_RES_CTRL_REG4];
	buf[2] = acc->resume_state[LSM330_RES_CTRL_REG1];
	err = lsm330_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;

	acc->hw_initialized = 1;
	pr_info("%s: hw init done\n", LSM330_ACC_DEV_NAME);
	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void lsm330_acc_device_power_off(struct lsm330_acc_data *acc)
{
	int err;

	err = lsm330_acc_i2c_update(acc, LSM330_CTRL_REG4,
					LSM330_ODR_MASK, LSM330_PM_OFF);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);

	if (acc->pdata->power_off) {
		if(acc->pdata->gpio_int1)
			disable_irq_nosync(acc->irq1);
		if(acc->pdata->gpio_int2)
			disable_irq_nosync(acc->irq2);
		acc->pdata->power_off();
		acc->hw_initialized = 0;
	}
	if (acc->hw_initialized) {
		if(acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
		acc->hw_initialized = 0;
	}
}

static int lsm330_acc_device_power_on(struct lsm330_acc_data *acc)
{
	int err = -1;

	if (acc->pdata->power_on) {
		err = acc->pdata->power_on();
		if (err < 0) {
			dev_err(&acc->client->dev,
					"power_on failed: %d\n", err);
			return err;
		}
		if(acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}

	if (!acc->hw_initialized) {
		err = lsm330_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			lsm330_acc_device_power_off(acc);
			return err;
		}
	}

	if (acc->hw_initialized) {
		if(acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}
	return 0;
}

static irqreturn_t lsm330_acc_isr1(int irq, void *dev)
{
	struct lsm330_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);
	pr_info("%s: isr1 queued\n", LSM330_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static irqreturn_t lsm330_acc_isr2(int irq, void *dev)
{
	struct lsm330_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);
	pr_info("%s: isr2 queued\n", LSM330_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void lsm330_acc_irq1_work_func(struct work_struct *work)
{

	struct lsm330_acc_data *acc;
	acc = container_of(work, struct lsm330_acc_data, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm330_acc_get_int1_source(acc); */
	;
	/*  */
	pr_info("%s: IRQ1 triggered\n", LSM330_ACC_DEV_NAME);
exit:
	enable_irq(acc->irq1);
}

static void lsm330_acc_irq2_work_func(struct work_struct *work)
{

	struct lsm330_acc_data *acc;
	acc = container_of(work, struct lsm330_acc_data, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm330_acc_get_tap_source(acc); */
	;
	/*  */

	pr_info("%s: IRQ2 triggered\n", LSM330_ACC_DEV_NAME);
exit:
	enable_irq(acc->irq2);
}

static int lsm330_acc_register_masked_update(struct lsm330_acc_data *acc,
		u8 reg_address, u8 mask, u8 new_bit_values, int resume_index)
{
	u8 config[2] = {0};
	u8 init_val, updated_val;
	int err;
	int step = 0;

	config[0] = reg_address;
	err = lsm330_acc_i2c_read(acc, config, 1);
	if (err < 0)
		goto error;
	init_val = config[0];
	init_val = acc->resume_state[resume_index];
	step = 1;
	updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
	config[0] = reg_address;
	config[1] = updated_val;
	err = lsm330_acc_i2c_write(acc, config, 1);
	if (err < 0)
		goto error;
	acc->resume_state[resume_index] = updated_val;

	return err;
	error:
		dev_err(&acc->client->dev,
			"register 0x%x update failed at step %d, error: %d\n",
				config[0], step, err);
	return err;
}

static int lsm330_acc_update_fs_range(struct lsm330_acc_data *acc,
								u8 new_fs_range)
{
	int err=-1;
	u16 sensitivity;

	switch (new_fs_range) {
	case LSM330_ACC_G_2G:
		sensitivity = SENSITIVITY_2G;
		break;
	case LSM330_ACC_G_4G:
		sensitivity = SENSITIVITY_4G;
		break;
	case LSM330_ACC_G_6G:
		sensitivity = SENSITIVITY_6G;
		break;
	case LSM330_ACC_G_8G:
		sensitivity = SENSITIVITY_8G;
		break;
	case LSM330_ACC_G_16G:
		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(&acc->client->dev, "invalid g range requested: %u\n",
				new_fs_range);
		return -EINVAL;
	}

	if (atomic_read(&acc->enabled)) {
		/* Updates configuration register 1,
		* which contains g range setting */
		err = lsm330_acc_register_masked_update(acc, LSM330_CTRL_REG5,
			LSM330_ACC_FS_MASK, new_fs_range, LSM330_RES_CTRL_REG5);
		if(err < 0) {
			dev_err(&acc->client->dev, "update g range failed\n");
			return err;
		}
		else
			acc->sensitivity = sensitivity;
	}

	if(err < 0)
		dev_err(&acc->client->dev, "update g range not executed "
						"because the device is off\n");
	return err;
}


static int lsm330_acc_update_odr(struct lsm330_acc_data *acc,
							int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 new_odr;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lsm330_acc_odr_table) - 1; i >= 0; i--) {
		if (lsm330_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	new_odr = lsm330_acc_odr_table[i].mask;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enabled)) {
		err = lsm330_acc_register_masked_update(acc,
			LSM330_CTRL_REG4, LSM330_ODR_MASK, new_odr,
							LSM330_RES_CTRL_REG4);
	}

	if(err < 0)
		dev_err(&acc->client->dev, "update odr failed\n");
	return err;
}


#ifdef DEBUG
static int lsm330_acc_register_write(struct lsm330_acc_data *acc, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = lsm330_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			return err;
	return err;
}

static int lsm330_acc_register_read(struct lsm330_acc_data *acc, u8 *buf,
		u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = lsm330_acc_i2c_read(acc, buf, 1);
	return err;
}

static int lsm330_acc_register_update(struct lsm330_acc_data *acc, u8 *buf,
		u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = lsm330_acc_register_read(acc, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = lsm330_acc_register_write(acc, buf, reg_address,
				updated_val);
	}
	return err;
}
#endif


static int lsm330_acc_get_acceleration_data(struct lsm330_acc_data *acc,
		int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s32 hw_d[3] = { 0 };

	acc_data[0] = (I2C_AUTO_INCREMENT | OUT_AXISDATA_REG);
	err = lsm330_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = ((s16) ((acc_data[1] << 8) | acc_data[0]));
	hw_d[1] = ((s16) ((acc_data[3] << 8) | acc_data[2]));
	hw_d[2] = ((s16) ((acc_data[5] << 8) | acc_data[4]));

	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;


	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));

#ifdef DEBUG
//	pr_info("%s read x=%d, y=%d, z=%d\n",
//			LSM330_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
#endif
	return err;
}

static void lsm330_acc_report_values(struct lsm330_acc_data *acc,
					int *xyz)
{
	input_report_abs(acc->input_dev, ABS_X, xyz[0]);
	input_report_abs(acc->input_dev, ABS_Y, xyz[1]);
	input_report_abs(acc->input_dev, ABS_Z, xyz[2]);
	input_sync(acc->input_dev);
}

static int lsm330_acc_enable(struct lsm330_acc_data *acc)
{
	int err;

	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = lsm330_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		schedule_delayed_work(&acc->input_work,
			msecs_to_jiffies(acc->pdata->poll_interval));
	}

	return 0;
}

static int lsm330_acc_disable(struct lsm330_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work_sync(&acc->input_work);
		lsm330_acc_device_power_off(acc);
	}

	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
					struct device_attribute *attr,
								char *buf)
{
	int val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	mutex_lock(&acc->lock);
	err = lsm330_acc_update_odr(acc, interval_ms);
	if(err >= 0)
	{
		acc->pdata->poll_interval = interval_ms;
	}
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int range = 2;
	mutex_lock(&acc->lock);
	val = acc->pdata->fs_range ;
	switch(val) {
	case LSM330_ACC_G_2G:
		range = 2;
		break;
	case LSM330_ACC_G_4G:
		range = 4;
		break;
	case LSM330_ACC_G_6G:
		range = 6;
		break;
	case LSM330_ACC_G_8G:
		range = 8;
		break;
	case LSM330_ACC_G_16G:
		range = 16;
		break;
	}
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch(val) {
		case 2:
			range = LSM330_ACC_G_2G;
			break;
		case 4:
			range = LSM330_ACC_G_4G;
			break;
		case 6:
			range = LSM330_ACC_G_6G;
			break;
		case 8:
			range = LSM330_ACC_G_8G;
			break;
		case 16:
			range = LSM330_ACC_G_16G;
			break;
		default:
			return -1;
	}

	mutex_lock(&acc->lock);
	err = lsm330_acc_update_fs_range(acc, range);
	if(err >= 0)
	{
		acc->pdata->fs_range = range;
	}
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm330_acc_enable(acc);
	else
		lsm330_acc_disable(acc);

	return size;
}


static int lsm330_acc_interrupt_enable_control(struct lsm330_acc_data *acc,
								u8 settings)
{
	u8 val1;
	u8 val2 = LSM330_INTEN_ON;
	u8 mask1 = (LSM330_INT1_EN_MASK | LSM330_INT2_EN_MASK);
	int err = -1;
	settings = settings & 0x03;

	switch ( settings ) {
	case LSM330_INT1_DIS_INT2_DIS:
		val1 = (LSM330_INT1_EN_OFF | LSM330_INT2_EN_OFF);
		val2 = LSM330_INTEN_OFF;
		break;
	case LSM330_INT1_DIS_INT2_EN:
		val1 = (LSM330_INT1_EN_OFF | LSM330_INT2_EN_ON);
		break;
	case LSM330_INT1_EN_INT2_DIS:
		val1 = (LSM330_INT1_EN_ON | LSM330_INT2_EN_OFF);
		break;
	case LSM330_INT1_EN_INT2_EN:
		val1 = ( LSM330_INT1_EN_ON | LSM330_INT2_EN_ON);
		break;
	default :
		pr_err("invalid interrupt setting : 0x%02x\n",settings);
		return err;
	}
	err = lsm330_acc_register_masked_update(acc,
		LSM330_CTRL_REG3, mask1, val1, LSM330_RES_CTRL_REG3);
	if (err < 0 )
		return err;

	err = lsm330_acc_register_masked_update(acc,
		LSM330_CTRL_REG1, LSM330_INTEN_MASK, val2,
							LSM330_RES_CTRL_REG1);
	if (err < 0 )
			return err;
	acc->interrupts_enable_setting = settings;
#ifdef DEBUG
	pr_err("interrupt setting : 0x%02x\n",acc->interrupts_enable_setting);
#endif
	return err;
}

static ssize_t attr_get_interr_enable(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	u8 val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->interrupts_enable_setting;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_set_interr_enable(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	int err = -1;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	long val=0;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;


	if ( val < 0x00 || val > LSM330_INT1_EN_INT2_EN){
#ifdef DEBUG
		pr_err("invalid interrupt setting, val: %d\n",val);
#endif
		return -EINVAL;
	}

	mutex_lock(&acc->lock);
	err = lsm330_acc_interrupt_enable_control(acc, val);
	mutex_unlock(&acc->lock);
	if (err < 0)
		return err;
	return size;
}

#ifdef DEBUG
/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	x[0] = acc->reg_addr;
	mutex_unlock(&acc->lock);
	x[1] = val;
	rc = lsm330_acc_i2c_write(acc, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&acc->lock);
	data = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = lsm330_acc_i2c_read(acc, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);
	return size;
}
#endif

static struct device_attribute attributes[] = {

	__ATTR(pollrate_ms, 0666, attr_get_polling_rate,
							attr_set_polling_rate),
	__ATTR(range, 0666, attr_get_range, attr_set_range),
	__ATTR(enable_device, 0666, attr_get_enable, attr_set_enable),
	__ATTR(enable_interrupt_output, 0666, attr_get_interr_enable,
							attr_set_interr_enable),
#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void lsm330_acc_input_work_func(struct work_struct *work)
{
	struct lsm330_acc_data *acc;

	int xyz[3] = { 0 };
	int err;

	acc = container_of(work, struct lsm330_acc_data, input_work);

	mutex_lock(&acc->lock);
	err = lsm330_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		lsm330_acc_report_values(acc, xyz);

	schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
			acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
}

int lsm330_acc_input_open(struct input_dev *input)
{
	struct lsm330_acc_data *acc = input_get_drvdata(input);

	return lsm330_acc_enable(acc);
}

void lsm330_acc_input_close(struct input_dev *dev)
{
	struct lsm330_acc_data *acc = input_get_drvdata(dev);

	lsm330_acc_disable(acc);
}

static int lsm330_acc_validate_pdata(struct lsm330_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
		acc->pdata->axis_map_y > 2 ||
		 acc->pdata->axis_map_z > 2) {
		dev_err(&acc->client->dev, "invalid axis_map value "
			"x:%u y:%u z%u\n", acc->pdata->axis_map_x,
				acc->pdata->axis_map_y, acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1
			|| acc->pdata->negate_z > 1) {
		dev_err(&acc->client->dev, "invalid negate value "
			"x:%u y:%u z:%u\n", acc->pdata->negate_x,
				acc->pdata->negate_y, acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm330_acc_input_init(struct lsm330_acc_data *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->input_work, lsm330_acc_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocation failed\n");
		goto err0;
	}

	acc->input_dev->open = lsm330_acc_input_open;
	acc->input_dev->close = lsm330_acc_input_close;
	acc->input_dev->name = LSM330_ACC_DEV_NAME;

	acc->input_dev->id.bustype = BUS_I2C;
	acc->input_dev->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, 0, 0);
	/*	next is used for interruptA sources data if the case */
//input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
//input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);


	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,
				"unable to register input device %s\n",
				acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void lsm330_acc_input_cleanup(struct lsm330_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static int lsm330_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct lsm330_acc_data *acc;

	int err = -1;

	pr_info("%s: probe start.\n", LSM330_ACC_DEV_NAME);

	/*if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}*/

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}


	acc = kzalloc(sizeof(struct lsm330_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->client = client;
	i2c_set_clientdata(client, acc);

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto err_mutexunlock;
	}

	if(client->dev.platform_data == NULL) {
		pr_info("using default platform_data for accelerometer\n");
		memcpy(acc->pdata, &default_lsm330_acc_pdata,
							sizeof(*acc->pdata));
	} else {
		memcpy(acc->pdata, client->dev.platform_data,
							sizeof(*acc->pdata));
	}

	err = lsm330_acc_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}


	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	if(acc->pdata->gpio_int1 >= 0){
		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
		pr_info("%s: %s has set irq1 to irq: %d "
							"mapped on gpio:%d\n",
			LSM330_ACC_DEV_NAME, __func__, acc->irq1,
							acc->pdata->gpio_int1);
	}

	if(acc->pdata->gpio_int2 >= 0){
		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);
		pr_info("%s: %s has set irq2 to irq: %d "
							"mapped on gpio:%d\n",
			LSM330_ACC_DEV_NAME, __func__, acc->irq2,
							acc->pdata->gpio_int2);
	}

	/* resume state init config */
	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));
	lsm330_acc_set_init_register_values(acc);

	err = lsm330_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&acc->enabled, 1);

	err = lsm330_acc_interrupt_enable_control(acc,
						acc->interrupts_enable_setting);
	if (err < 0) {
		dev_err(&client->dev, "interrupt settings failed\n");
		goto  err_power_off;
	}

	err = lsm330_acc_update_fs_range(acc, acc->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lsm330_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lsm330_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}


	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
		   "device LSM330_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	lsm330_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	if(acc->pdata->gpio_int1 >= 0){
		INIT_WORK(&acc->irq1_work, lsm330_acc_irq1_work_func);
		acc->irq1_work_queue =
			create_singlethread_workqueue("lsm330_acc_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(acc->irq1, lsm330_acc_isr1,
				IRQF_TRIGGER_RISING, "lsm330_acc_irq1", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq1);
	}

	if(acc->pdata->gpio_int2 >= 0){
		INIT_WORK(&acc->irq2_work, lsm330_acc_irq2_work_func);
		acc->irq2_work_queue =
			create_singlethread_workqueue("lsm330_acc_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(acc->irq2, lsm330_acc_isr2,
				IRQF_TRIGGER_RISING, "lsm330_acc_irq2", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(acc->irq2);
	}



	mutex_unlock(&acc->lock);

	dev_info(&client->dev, "%s: probed\n", LSM330_ACC_DEV_NAME);

	return 0;

err_destoyworkqueue2:
	if(acc->pdata->gpio_int2 >= 0)
		destroy_workqueue(acc->irq2_work_queue);
err_free_irq1:
	free_irq(acc->irq1, acc);
err_destoyworkqueue1:
	if(acc->pdata->gpio_int1 >= 0)
		destroy_workqueue(acc->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
err_input_cleanup:
	lsm330_acc_input_cleanup(acc);
err_power_off:
	lsm330_acc_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
//err_freedata:
	kfree(acc);
exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", LSM330_ACC_DEV_NAME);
	return err;
}

static int lsm330_acc_remove(struct i2c_client *client)
{
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	if(acc->pdata->gpio_int1 >= 0){
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
		destroy_workqueue(acc->irq1_work_queue);
	}

	if(acc->pdata->gpio_int2 >= 0){
		free_irq(acc->irq2, acc);
		gpio_free(acc->pdata->gpio_int2);
		destroy_workqueue(acc->irq2_work_queue);
	}

	if (atomic_cmpxchg(&acc->enabled, 1, 0))
			cancel_delayed_work_sync(&acc->input_work);

	lsm330_acc_device_power_off(acc);
	lsm330_acc_input_cleanup(acc);
	remove_sysfs_interfaces(&client->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();

	kfree(acc->pdata);
	kfree(acc);

	return 0;
}

#ifdef CONFIG_PM
static int lsm330_acc_resume(struct i2c_client *client)
{
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	if (acc->on_before_suspend)
		return lsm330_acc_enable(acc);
	return 0;
}

static int lsm330_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	acc->on_before_suspend = atomic_read(&acc->enabled);
	return lsm330_acc_disable(acc);
}
#else
#define lsm330_acc_suspend	NULL
#define lsm330_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lsm330_acc_id[]
		= { { LSM330_ACC_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, lsm330_acc_id);

static struct i2c_driver lsm330_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LSM330_ACC_DEV_NAME,
		  },
	.probe = lsm330_acc_probe,
	.remove = lsm330_acc_remove,
	.suspend = lsm330_acc_suspend,
	.resume = lsm330_acc_resume,
	.id_table = lsm330_acc_id,
};

static int __init lsm330_acc_init(void)
{
	pr_info("%s accelerometer driver: init\n", LSM330_ACC_DEV_NAME);
	return i2c_add_driver(&lsm330_acc_driver);
}

static void __exit lsm330_acc_exit(void)
{
#ifdef DEBUG
	pr_info("%s accelerometer driver exit\n", LSM330_ACC_DEV_NAME);
#endif /* DEBUG */
	i2c_del_driver(&lsm330_acc_driver);
	return;
}

module_init(lsm330_acc_init);
module_exit(lsm330_acc_exit);

MODULE_DESCRIPTION("lsm330 accelerometer driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");
