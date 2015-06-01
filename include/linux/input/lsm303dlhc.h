
/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name          : lsm303dlhc_sysfs.h
* Authors            : MH - C&I BU - Application Team
*		     : Matteo Dameno (matteo.dameno@st.com)
*		     : Carmine Iascone (carmine.iascone@st.com)
* Version            : V.1.0.13
* Date               : 2012/Jun/30
*
********************************************************************************
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
*******************************************************************************/
/*******************************************************************************
Version History.

Revision 1-0-6 10/11/2010
	ioclt not supported
	sysfs support
Revision 1-0-7 26/11/2010
	moved to input/misc
	manages use/non-use of interrupts on accelerometer side
 Revision 1.0.8 2010/Apr/01
  corrects a bug in interrupt pin management in 1.0.7 acc side
 Revision 1.0.8 2010/Apr/01
  corrects a bug in interrupt pin management in 1.0.7 acc side
 Revision 1.0.9: 2011/May/23
	SLEEP_MODE correction; update_odr correction; get/set_polling_rate corr.
 Revision 1.0.10: 2011/Aug/16
  introduces default_platform_data, i2c_read and i2c_write function rewritten,
  manages smbus beside i2c
 Revision 1.0.11: 2012/Jan/09
  moved under input/misc
 Revision 1.0.12: 2012/Feb/29
  moved use_smbus inside status struct; modified:-update_fs_range;-set_range
  input format; allows gpio_intX to be passed as parameter at insmod time;
 Revision 1.0.12.1: 2012/May/29
  renamed field g_range to fs_range in lsm303dlhc_acc_platform_data
  replaced defines SA0L and SA0H with LSM303DLHC_SAD0x
 Revision 1.0.13: 2012/Jun/30
  mag: corrects saturation code management;
  mag: changes sysfs range input commands to decimal mGauss;
*******************************************************************************/

#ifndef	__LSM303DLHC_H__
#define	__LSM303DLHC_H__


#define	LSM303DLHC_ACC_DEV_NAME		"lsm303dlhc_acc"
#define LSM303DLHC_MAG_DEV_NAME		"lsm303dlhc_mag"

/************************************************/
/* 	Accelerometer section defines	 	*/
/************************************************/

#define	LSM303DLHC_ACC_MIN_POLL_PERIOD_MS	1


/************************************************/
/* 	Magnetometer section defines	 	*/
/************************************************/

#define LSM303DLHC_MAG_MIN_POLL_PERIOD_MS	5

/* Magnetometer Sensor Full Scale */
#define LSM303DLHC_H_1_3G		(0x20)
#define LSM303DLHC_H_1_9G		(0x40)
#define LSM303DLHC_H_2_5G		(0x60)
#define LSM303DLHC_H_4_0G		(0x80)
#define LSM303DLHC_H_4_7G		(0xA0)
#define LSM303DLHC_H_5_6G		(0xC0)
#define LSM303DLHC_H_8_1G		(0xE0)


#ifdef	__KERNEL__

#define LSM303DLHC_SAD0L		(0x00)
#define LSM303DLHC_SAD0H		(0x01)

/* magnetometer section */
#define LSM303DLHC_MAG_I2C_SAD		(0x1E)

/* Magnetic Sensor Operating Mode */
#define LSM303DLHC_MAG_NORMAL_MODE	(0x00)
#define LSM303DLHC_MAG_POS_BIAS		(0x01)
#define LSM303DLHC_MAG_NEG_BIAS		(0x02)
#define LSM303DLHC_MAG_CC_MODE		(0x00)
#define LSM303DLHC_MAG_SC_MODE		(0x01)
#define LSM303DLHC_MAG_SLEEP_MODE	(0x03)


/* acc section */
#define LSM303DLHC_ACC_I2C_SADROOT		(0x0C)
/* I2C address if acc SA0 pin to GND */
#define LSM303DLHC_ACC_I2C_SAD_L		((LSM303DLHC_ACC_I2C_SADROOT<<1)| \
							LSM303DLHC_SAD0L)
/* I2C address if acc SA0 pin to Vdd */
#define LSM303DLHC_ACC_I2C_SAD_H		((LSM303DLHC_ACC_I2C_SADROOT<<1)| \
							LSM303DLHC_SAD0H)

/* to set gpios numb connected to gyro interrupt pins,
 * the unused ones havew to be set to -EINVAL
 */
#define LSM303DLHC_ACC_DEFAULT_INT1_GPIO	(-EINVAL)
#define LSM303DLHC_ACC_DEFAULT_INT2_GPIO	(-EINVAL)

/* Accelerometer Sensor Full Scale */
#define	LSM303DLHC_ACC_FS_MASK		(0x30)
#define LSM303DLHC_ACC_G_2G		(0x00)
#define LSM303DLHC_ACC_G_4G		(0x10)
#define LSM303DLHC_ACC_G_8G		(0x20)
#define LSM303DLHC_ACC_G_16G		(0x30)



struct lsm303dlhc_acc_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	/* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
	int gpio_int2;
};

struct lsm303dlhc_mag_platform_data {


	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif	/* __KERNEL__ */

#endif	/* __LSM303DLHC_H__ */



