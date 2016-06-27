/*
* Copyright (C) 2012 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/iio/iio.h>
#include <linux/acpi.h>
#include <linux/completion.h>

#include "inv_mpu_iio.h"

/*
 * this is the gyro scale translated from dynamic range plus/minus
 * {250, 500, 1000, 2000} to rad/s
 */
static const int gyro_scale_6050[] = {133090, 266181, 532362, 1064724};

/*
 * this is the accel scale translated from dynamic range plus/minus
 * {2, 4, 8, 16} to m/s^2
 */
static const int accel_scale[] = {598, 1196, 2392, 4785};

static const struct inv_mpu6050_reg_map reg_set_6500 = {
	.sample_rate_div	= INV_MPU6050_REG_SAMPLE_RATE_DIV,
	.lpf                    = INV_MPU6050_REG_CONFIG,
	.user_ctrl              = INV_MPU6050_REG_USER_CTRL,
	.fifo_en                = INV_MPU6050_REG_FIFO_EN,
	.gyro_config            = INV_MPU6050_REG_GYRO_CONFIG,
	.accl_config            = INV_MPU6050_REG_ACCEL_CONFIG,
	.fifo_count_h           = INV_MPU6050_REG_FIFO_COUNT_H,
	.fifo_r_w               = INV_MPU6050_REG_FIFO_R_W,
	.raw_gyro               = INV_MPU6050_REG_RAW_GYRO,
	.raw_accl               = INV_MPU6050_REG_RAW_ACCEL,
	.temperature            = INV_MPU6050_REG_TEMPERATURE,
	.int_enable             = INV_MPU6050_REG_INT_ENABLE,
	.pwr_mgmt_1             = INV_MPU6050_REG_PWR_MGMT_1,
	.pwr_mgmt_2             = INV_MPU6050_REG_PWR_MGMT_2,
	.int_pin_cfg		= INV_MPU6050_REG_INT_PIN_CFG,
	.accl_offset		= INV_MPU6500_REG_ACCEL_OFFSET,
	.gyro_offset		= INV_MPU6050_REG_GYRO_OFFSET,
	.mst_status		= INV_MPU6050_REG_I2C_MST_STATUS,
};

static const struct inv_mpu6050_reg_map reg_set_6050 = {
	.sample_rate_div	= INV_MPU6050_REG_SAMPLE_RATE_DIV,
	.lpf                    = INV_MPU6050_REG_CONFIG,
	.user_ctrl              = INV_MPU6050_REG_USER_CTRL,
	.fifo_en                = INV_MPU6050_REG_FIFO_EN,
	.gyro_config            = INV_MPU6050_REG_GYRO_CONFIG,
	.accl_config            = INV_MPU6050_REG_ACCEL_CONFIG,
	.fifo_count_h           = INV_MPU6050_REG_FIFO_COUNT_H,
	.fifo_r_w               = INV_MPU6050_REG_FIFO_R_W,
	.raw_gyro               = INV_MPU6050_REG_RAW_GYRO,
	.raw_accl               = INV_MPU6050_REG_RAW_ACCEL,
	.temperature            = INV_MPU6050_REG_TEMPERATURE,
	.int_enable             = INV_MPU6050_REG_INT_ENABLE,
	.pwr_mgmt_1             = INV_MPU6050_REG_PWR_MGMT_1,
	.pwr_mgmt_2             = INV_MPU6050_REG_PWR_MGMT_2,
	.int_pin_cfg		= INV_MPU6050_REG_INT_PIN_CFG,
	.accl_offset		= INV_MPU6050_REG_ACCEL_OFFSET,
	.gyro_offset		= INV_MPU6050_REG_GYRO_OFFSET,
	.mst_status		= INV_MPU6050_REG_I2C_MST_STATUS,
};

static const struct inv_mpu6050_chip_config chip_config_6050 = {
	.fsr = INV_MPU6050_FSR_2000DPS,
	.lpf = INV_MPU6050_FILTER_20HZ,
	.fifo_rate = INV_MPU6050_INIT_FIFO_RATE,
	.gyro_fifo_enable = false,
	.accl_fifo_enable = false,
	.accl_fs = INV_MPU6050_FS_02G,
};

/* Indexed by enum inv_devices */
static const struct inv_mpu6050_hw hw_info[] = {
	{
		.whoami = INV_MPU6050_WHOAMI_VALUE,
		.name = "MPU6050",
		.reg = &reg_set_6050,
		.config = &chip_config_6050,
	},
	{
		.whoami = INV_MPU6500_WHOAMI_VALUE,
		.name = "MPU6500",
		.reg = &reg_set_6500,
		.config = &chip_config_6050,
	},
	{
		.whoami = INV_MPU6000_WHOAMI_VALUE,
		.name = "MPU6000",
		.reg = &reg_set_6050,
		.config = &chip_config_6050,
	},
	{
		.whoami = INV_MPU9150_WHOAMI_VALUE,
		.name = "MPU9150",
		.reg = &reg_set_6050,
		.config = &chip_config_6050,
	},
};

static bool inv_mpu6050_volatile_reg(struct device *dev, unsigned int reg)
{
	if (reg >= INV_MPU6050_REG_RAW_ACCEL && reg < INV_MPU6050_REG_RAW_ACCEL + 6)
		return true;
	if (reg >= INV_MPU6050_REG_RAW_GYRO && reg < INV_MPU6050_REG_RAW_GYRO + 6)
		return true;
	if (reg < INV_MPU6050_REG_EXT_SENS_DATA_00 + INV_MPU6050_CNT_EXT_SENS_DATA &&
			reg >= INV_MPU6050_REG_EXT_SENS_DATA_00)
		return true;
	switch (reg) {
	case INV_MPU6050_REG_TEMPERATURE:
	case INV_MPU6050_REG_TEMPERATURE + 1:
	case INV_MPU6050_REG_USER_CTRL:
	case INV_MPU6050_REG_PWR_MGMT_1:
	case INV_MPU6050_REG_FIFO_COUNT_H:
	case INV_MPU6050_REG_FIFO_COUNT_H + 1:
	case INV_MPU6050_REG_FIFO_R_W:
	case INV_MPU6050_REG_I2C_MST_STATUS:
	case INV_MPU6050_REG_INT_STATUS:
	case INV_MPU6050_REG_I2C_SLV4_CTRL:
	case INV_MPU6050_REG_I2C_SLV4_DI:
		return true;
	default:
		return false;
	}
}

static bool inv_mpu6050_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case INV_MPU6050_REG_FIFO_R_W:
	case INV_MPU6050_REG_I2C_MST_STATUS:
	case INV_MPU6050_REG_INT_STATUS:
		return true;
	default:
		return false;
	}
}

/*
 * Common regmap config for inv_mpu devices
 *
 * The current volatile/precious registers are common among supported devices.
 * When that changes the volatile/precious callbacks should go through the
 * inv_mpu6050_reg_map structs.
 */
const struct regmap_config inv_mpu_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = inv_mpu6050_volatile_reg,
	.precious_reg = inv_mpu6050_precious_reg,
};
EXPORT_SYMBOL_GPL(inv_mpu_regmap_config);

int inv_mpu6050_switch_engine(struct inv_mpu6050_state *st, bool en, u32 mask)
{
	unsigned int d, mgmt_1;
	int result;
	/*
	 * switch clock needs to be careful. Only when gyro is on, can
	 * clock source be switched to gyro. Otherwise, it must be set to
	 * internal clock
	 */
	if (mask == INV_MPU6050_BIT_PWR_GYRO_STBY) {
		result = regmap_read(st->map, st->reg->pwr_mgmt_1, &mgmt_1);
		if (result)
			return result;

		mgmt_1 &= ~INV_MPU6050_BIT_CLK_MASK;
	}

	if ((mask == INV_MPU6050_BIT_PWR_GYRO_STBY) && (!en)) {
		/*
		 * turning off gyro requires switch to internal clock first.
		 * Then turn off gyro engine
		 */
		mgmt_1 |= INV_CLK_INTERNAL;
		result = regmap_write(st->map, st->reg->pwr_mgmt_1, mgmt_1);
		if (result)
			return result;
	}

	result = regmap_read(st->map, st->reg->pwr_mgmt_2, &d);
	if (result)
		return result;
	if (en)
		d &= ~mask;
	else
		d |= mask;
	result = regmap_write(st->map, st->reg->pwr_mgmt_2, d);
	if (result)
		return result;

	if (en) {
		/* Wait for output stabilize */
		msleep(INV_MPU6050_TEMP_UP_TIME);
		if (mask == INV_MPU6050_BIT_PWR_GYRO_STBY) {
			/* switch internal clock to PLL */
			mgmt_1 |= INV_CLK_PLL;
			result = regmap_write(st->map,
					      st->reg->pwr_mgmt_1, mgmt_1);
			if (result)
				return result;
		}
	}

	return 0;
}

int inv_mpu6050_set_power_itg(struct inv_mpu6050_state *st, bool power_on)
{
	int result = 0;

	if (power_on) {
		/* Already under indio-dev->mlock mutex */
		if (!st->powerup_count)
			result = regmap_write(st->map, st->reg->pwr_mgmt_1, 0);
		if (!result)
			st->powerup_count++;
	} else {
		st->powerup_count--;
		if (!st->powerup_count)
			result = regmap_write(st->map, st->reg->pwr_mgmt_1,
					      INV_MPU6050_BIT_SLEEP);
	}

	if (result)
		return result;

	if (power_on)
		usleep_range(INV_MPU6050_REG_UP_TIME_MIN,
			     INV_MPU6050_REG_UP_TIME_MAX);

	return 0;
}
EXPORT_SYMBOL_GPL(inv_mpu6050_set_power_itg);

/**
 *  inv_mpu6050_init_config() - Initialize hardware, disable FIFO.
 *
 *  Initial configuration:
 *  FSR: ± 2000DPS
 *  DLPF: 20Hz
 *  FIFO rate: 50Hz
 *  Clock source: Gyro PLL
 */
static int inv_mpu6050_init_config(struct iio_dev *indio_dev)
{
	int result;
	u8 d;
	struct inv_mpu6050_state *st = iio_priv(indio_dev);

	result = inv_mpu6050_set_power_itg(st, true);
	if (result)
		return result;
	d = (INV_MPU6050_FSR_2000DPS << INV_MPU6050_GYRO_CONFIG_FSR_SHIFT);
	result = regmap_write(st->map, st->reg->gyro_config, d);
	if (result)
		return result;

	d = INV_MPU6050_FILTER_20HZ;
	result = regmap_write(st->map, st->reg->lpf, d);
	if (result)
		return result;

	d = INV_MPU6050_ONE_K_HZ / INV_MPU6050_INIT_FIFO_RATE - 1;
	result = regmap_write(st->map, st->reg->sample_rate_div, d);
	if (result)
		return result;

	d = (INV_MPU6050_FS_02G << INV_MPU6050_ACCL_CONFIG_FSR_SHIFT);
	result = regmap_write(st->map, st->reg->accl_config, d);
	if (result)
		return result;

	memcpy(&st->chip_config, hw_info[st->chip_type].config,
	       sizeof(struct inv_mpu6050_chip_config));
	result = inv_mpu6050_set_power_itg(st, false);

	return result;
}

static int inv_mpu6050_sensor_set(struct inv_mpu6050_state  *st, int reg,
				int axis, int val)
{
	int ind, result;
	__be16 d = cpu_to_be16(val);

	ind = (axis - IIO_MOD_X) * 2;
	result = regmap_bulk_write(st->map, reg + ind, (u8 *)&d, 2);
	if (result)
		return -EINVAL;

	return 0;
}

static int inv_mpu6050_sensor_show(struct inv_mpu6050_state  *st, int reg,
				   int axis, int *val)
{
	int ind, result;
	__be16 d;

	ind = (axis - IIO_MOD_X) * 2;
	result = regmap_bulk_read(st->map, reg + ind, (u8 *)&d, 2);
	if (result)
		return -EINVAL;
	*val = (short)be16_to_cpup(&d);

	return IIO_VAL_INT;
}

static int
inv_mpu6050_read_raw(struct iio_dev *indio_dev,
		     struct iio_chan_spec const *chan,
		     int *val, int *val2, long mask)
{
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	int ret = 0;
	int chan_index;

	if (chan > indio_dev->channels + indio_dev->num_channels ||
			chan < indio_dev->channels)
		return -EINVAL;
	chan_index = chan - indio_dev->channels;
	if (chan_index >= INV_MPU6050_NUM_INT_CHAN) {
		struct inv_mpu_ext_chan_state *ext_chan_state =
				&st->ext_chan[chan_index - INV_MPU6050_NUM_INT_CHAN];
		struct inv_mpu_ext_sens_state *ext_sens_state =
				&st->ext_sens[ext_chan_state->ext_sens_index];
		struct iio_dev *orig_dev = ext_sens_state->orig_dev;
		const struct iio_chan_spec *orig_chan =
				&orig_dev->channels[ext_chan_state->orig_chan_index];
		return orig_dev->info->read_raw(orig_dev, orig_chan, val, val2, mask);
	}

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	{
		int result;

		ret = IIO_VAL_INT;
		result = 0;
		mutex_lock(&indio_dev->mlock);
		if (!st->chip_config.enable) {
			result = inv_mpu6050_set_power_itg(st, true);
			if (result)
				goto error_read_raw;
		}
		/* when enable is on, power is already on */
		switch (chan->type) {
		case IIO_ANGL_VEL:
			if (!st->chip_config.gyro_fifo_enable ||
			    !st->chip_config.enable) {
				result = inv_mpu6050_switch_engine(st, true,
						INV_MPU6050_BIT_PWR_GYRO_STBY);
				if (result)
					goto error_read_raw;
			}
			ret = inv_mpu6050_sensor_show(st, st->reg->raw_gyro,
						      chan->channel2, val);
			if (!st->chip_config.gyro_fifo_enable ||
			    !st->chip_config.enable) {
				result = inv_mpu6050_switch_engine(st, false,
						INV_MPU6050_BIT_PWR_GYRO_STBY);
				if (result)
					goto error_read_raw;
			}
			break;
		case IIO_ACCEL:
			if (!st->chip_config.accl_fifo_enable ||
			    !st->chip_config.enable) {
				result = inv_mpu6050_switch_engine(st, true,
						INV_MPU6050_BIT_PWR_ACCL_STBY);
				if (result)
					goto error_read_raw;
			}
			ret = inv_mpu6050_sensor_show(st, st->reg->raw_accl,
						      chan->channel2, val);
			if (!st->chip_config.accl_fifo_enable ||
			    !st->chip_config.enable) {
				result = inv_mpu6050_switch_engine(st, false,
						INV_MPU6050_BIT_PWR_ACCL_STBY);
				if (result)
					goto error_read_raw;
			}
			break;
		case IIO_TEMP:
			/* wait for stablization */
			msleep(INV_MPU6050_SENSOR_UP_TIME);
			ret = inv_mpu6050_sensor_show(st, st->reg->temperature,
						IIO_MOD_X, val);
			break;
		default:
			ret = -EINVAL;
			break;
		}
error_read_raw:
		if (!st->chip_config.enable)
			result |= inv_mpu6050_set_power_itg(st, false);
		mutex_unlock(&indio_dev->mlock);
		if (result)
			return result;

		return ret;
	}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			*val  = 0;
			*val2 = gyro_scale_6050[st->chip_config.fsr];

			return IIO_VAL_INT_PLUS_NANO;
		case IIO_ACCEL:
			*val = 0;
			*val2 = accel_scale[st->chip_config.accl_fs];

			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_TEMP:
			*val = 0;
			*val2 = INV_MPU6050_TEMP_SCALE;

			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_TEMP:
			*val = INV_MPU6050_TEMP_OFFSET;

			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_CALIBBIAS:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			ret = inv_mpu6050_sensor_show(st, st->reg->gyro_offset,
						chan->channel2, val);
			return IIO_VAL_INT;
		case IIO_ACCEL:
			ret = inv_mpu6050_sensor_show(st, st->reg->accl_offset,
						chan->channel2, val);
			return IIO_VAL_INT;

		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int inv_mpu6050_write_gyro_scale(struct inv_mpu6050_state *st, int val)
{
	int result, i;
	u8 d;

	for (i = 0; i < ARRAY_SIZE(gyro_scale_6050); ++i) {
		if (gyro_scale_6050[i] == val) {
			d = (i << INV_MPU6050_GYRO_CONFIG_FSR_SHIFT);
			result = regmap_write(st->map, st->reg->gyro_config, d);
			if (result)
				return result;

			st->chip_config.fsr = i;
			return 0;
		}
	}

	return -EINVAL;
}

static int inv_write_raw_get_fmt(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			return IIO_VAL_INT_PLUS_NANO;
		default:
			return IIO_VAL_INT_PLUS_MICRO;
		}
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}

static int inv_mpu6050_write_accel_scale(struct inv_mpu6050_state *st, int val)
{
	int result, i;
	u8 d;

	for (i = 0; i < ARRAY_SIZE(accel_scale); ++i) {
		if (accel_scale[i] == val) {
			d = (i << INV_MPU6050_ACCL_CONFIG_FSR_SHIFT);
			result = regmap_write(st->map, st->reg->accl_config, d);
			if (result)
				return result;

			st->chip_config.accl_fs = i;
			return 0;
		}
	}

	return -EINVAL;
}

static int inv_mpu6050_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int val, int val2, long mask)
{
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	int chan_index;
	int result;

	if (chan > indio_dev->channels + indio_dev->num_channels ||
			chan < indio_dev->channels)
		return -EINVAL;
	chan_index = chan - indio_dev->channels;
	if (chan_index >= INV_MPU6050_NUM_INT_CHAN) {
		struct inv_mpu_ext_chan_state *ext_chan_state =
				&st->ext_chan[chan_index - INV_MPU6050_NUM_INT_CHAN];
		struct inv_mpu_ext_sens_state *ext_sens_state =
				&st->ext_sens[ext_chan_state->ext_sens_index];
		struct iio_dev *orig_dev = ext_sens_state->orig_dev;
		const struct iio_chan_spec *orig_chan =
				&orig_dev->channels[ext_chan_state->orig_chan_index];
		return orig_dev->info->write_raw(orig_dev, orig_chan, val, val2, mask);
	}

	mutex_lock(&indio_dev->mlock);
	/*
	 * we should only update scale when the chip is disabled, i.e.
	 * not running
	 */
	if (st->chip_config.enable) {
		result = -EBUSY;
		goto error_write_raw;
	}
	result = inv_mpu6050_set_power_itg(st, true);
	if (result)
		goto error_write_raw;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			result = inv_mpu6050_write_gyro_scale(st, val2);
			break;
		case IIO_ACCEL:
			result = inv_mpu6050_write_accel_scale(st, val2);
			break;
		default:
			result = -EINVAL;
			break;
		}
		break;
	case IIO_CHAN_INFO_CALIBBIAS:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			result = inv_mpu6050_sensor_set(st,
							st->reg->gyro_offset,
							chan->channel2, val);
			break;
		case IIO_ACCEL:
			result = inv_mpu6050_sensor_set(st,
							st->reg->accl_offset,
							chan->channel2, val);
			break;
		default:
			result = -EINVAL;
		}
	default:
		result = -EINVAL;
		break;
	}

error_write_raw:
	result |= inv_mpu6050_set_power_itg(st, false);
	mutex_unlock(&indio_dev->mlock);

	return result;
}

/**
 *  inv_mpu6050_set_lpf() - set low pass filer based on fifo rate.
 *
 *                  Based on the Nyquist principle, the sampling rate must
 *                  exceed twice of the bandwidth of the signal, or there
 *                  would be alising. This function basically search for the
 *                  correct low pass parameters based on the fifo rate, e.g,
 *                  sampling frequency.
 */
static int inv_mpu6050_set_lpf(struct inv_mpu6050_state *st, int rate)
{
	const int hz[] = {188, 98, 42, 20, 10, 5};
	const int d[] = {INV_MPU6050_FILTER_188HZ, INV_MPU6050_FILTER_98HZ,
			INV_MPU6050_FILTER_42HZ, INV_MPU6050_FILTER_20HZ,
			INV_MPU6050_FILTER_10HZ, INV_MPU6050_FILTER_5HZ};
	int i, h, result;
	u8 data;

	h = (rate >> 1);
	i = 0;
	while ((h < hz[i]) && (i < ARRAY_SIZE(d) - 1))
		i++;
	data = d[i];
	result = regmap_write(st->map, st->reg->lpf, data);
	if (result)
		return result;
	st->chip_config.lpf = data;

	return 0;
}

/**
 * inv_mpu6050_fifo_rate_store() - Set fifo rate.
 */
static ssize_t
inv_mpu6050_fifo_rate_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	s32 fifo_rate;
	u8 d;
	int result;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct inv_mpu6050_state *st = iio_priv(indio_dev);

	if (kstrtoint(buf, 10, &fifo_rate))
		return -EINVAL;
	if (fifo_rate < INV_MPU6050_MIN_FIFO_RATE ||
	    fifo_rate > INV_MPU6050_MAX_FIFO_RATE)
		return -EINVAL;
	if (fifo_rate == st->chip_config.fifo_rate)
		return count;

	mutex_lock(&indio_dev->mlock);
	if (st->chip_config.enable) {
		result = -EBUSY;
		goto fifo_rate_fail;
	}
	result = inv_mpu6050_set_power_itg(st, true);
	if (result)
		goto fifo_rate_fail;

	d = INV_MPU6050_ONE_K_HZ / fifo_rate - 1;
	result = regmap_write(st->map, st->reg->sample_rate_div, d);
	if (result)
		goto fifo_rate_fail;
	st->chip_config.fifo_rate = fifo_rate;

	result = inv_mpu6050_set_lpf(st, fifo_rate);
	if (result)
		goto fifo_rate_fail;

fifo_rate_fail:
	result |= inv_mpu6050_set_power_itg(st, false);
	mutex_unlock(&indio_dev->mlock);
	if (result)
		return result;

	return count;
}

/**
 * inv_fifo_rate_show() - Get the current sampling rate.
 */
static ssize_t
inv_fifo_rate_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	struct inv_mpu6050_state *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->chip_config.fifo_rate);
}

/**
 * inv_attr_show() - calling this function will show current
 *                    parameters.
 *
 * Deprecated in favor of IIO mounting matrix API.
 *
 * See inv_get_mount_matrix()
 */
static ssize_t inv_attr_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct inv_mpu6050_state *st = iio_priv(dev_to_iio_dev(dev));
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	s8 *m;

	switch (this_attr->address) {
	/*
	 * In MPU6050, the two matrix are the same because gyro and accel
	 * are integrated in one chip
	 */
	case ATTR_GYRO_MATRIX:
	case ATTR_ACCL_MATRIX:
		m = st->plat_data.orientation;

		return sprintf(buf, "%d, %d, %d; %d, %d, %d; %d, %d, %d\n",
			m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
	default:
		return -EINVAL;
	}
}

/**
 * inv_mpu6050_validate_trigger() - validate_trigger callback for invensense
 *                                  MPU6050 device.
 * @indio_dev: The IIO device
 * @trig: The new trigger
 *
 * Returns: 0 if the 'trig' matches the trigger registered by the MPU6050
 * device, -EINVAL otherwise.
 */
static int inv_mpu6050_validate_trigger(struct iio_dev *indio_dev,
					struct iio_trigger *trig)
{
	struct inv_mpu6050_state *st = iio_priv(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

static const struct iio_mount_matrix *
inv_get_mount_matrix(const struct iio_dev *indio_dev,
		     const struct iio_chan_spec *chan)
{
	return &((struct inv_mpu6050_state *)iio_priv(indio_dev))->orientation;
}

static const struct iio_chan_spec_ext_info inv_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_TYPE, inv_get_mount_matrix),
	{ },
};

#define INV_MPU6050_CHAN(_type, _channel2, _index)                    \
	{                                                             \
		.type = _type,                                        \
		.modified = 1,                                        \
		.channel2 = _channel2,                                \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	      \
				      BIT(IIO_CHAN_INFO_CALIBBIAS),   \
		.scan_index = _index,                                 \
		.scan_type = {                                        \
				.sign = 's',                          \
				.realbits = 16,                       \
				.storagebits = 16,                    \
				.shift = 0,                           \
				.endianness = IIO_BE,                 \
			     },                                       \
		.ext_info = inv_ext_info,                             \
	}

static const struct iio_chan_spec inv_mpu_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(INV_MPU6050_SCAN_TIMESTAMP),
	/*
	 * Note that temperature should only be via polled reading only,
	 * not the final scan elements output.
	 */
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
				| BIT(IIO_CHAN_INFO_OFFSET)
				| BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = -1,
	},
	INV_MPU6050_CHAN(IIO_ANGL_VEL, IIO_MOD_X, INV_MPU6050_SCAN_GYRO_X),
	INV_MPU6050_CHAN(IIO_ANGL_VEL, IIO_MOD_Y, INV_MPU6050_SCAN_GYRO_Y),
	INV_MPU6050_CHAN(IIO_ANGL_VEL, IIO_MOD_Z, INV_MPU6050_SCAN_GYRO_Z),

	INV_MPU6050_CHAN(IIO_ACCEL, IIO_MOD_X, INV_MPU6050_SCAN_ACCL_X),
	INV_MPU6050_CHAN(IIO_ACCEL, IIO_MOD_Y, INV_MPU6050_SCAN_ACCL_Y),
	INV_MPU6050_CHAN(IIO_ACCEL, IIO_MOD_Z, INV_MPU6050_SCAN_ACCL_Z),
};

/* constant IIO attribute */
static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("10 20 50 100 200 500");
static IIO_CONST_ATTR(in_anglvel_scale_available,
					  "0.000133090 0.000266181 0.000532362 0.001064724");
static IIO_CONST_ATTR(in_accel_scale_available,
					  "0.000598 0.001196 0.002392 0.004785");
static IIO_DEV_ATTR_SAMP_FREQ(S_IRUGO | S_IWUSR, inv_fifo_rate_show,
	inv_mpu6050_fifo_rate_store);

/* Deprecated: kept for userspace backward compatibility. */
static IIO_DEVICE_ATTR(in_gyro_matrix, S_IRUGO, inv_attr_show, NULL,
	ATTR_GYRO_MATRIX);
static IIO_DEVICE_ATTR(in_accel_matrix, S_IRUGO, inv_attr_show, NULL,
	ATTR_ACCL_MATRIX);

static struct attribute *inv_attributes[] = {
	&iio_dev_attr_in_gyro_matrix.dev_attr.attr,  /* deprecated */
	&iio_dev_attr_in_accel_matrix.dev_attr.attr, /* deprecated */
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_accel_scale_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group inv_attribute_group = {
	.attrs = inv_attributes
};

static const struct iio_info mpu_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &inv_mpu6050_read_raw,
	.write_raw = &inv_mpu6050_write_raw,
	.write_raw_get_fmt = &inv_write_raw_get_fmt,
	.attrs = &inv_attribute_group,
	.validate_trigger = inv_mpu6050_validate_trigger,
};

extern struct device_type iio_device_type;

static int iio_device_from_i2c_client_match(struct device *dev, void *data)
{
	return dev->type == &iio_device_type;
}

static struct iio_dev* iio_device_from_i2c_client(struct i2c_client* i2c)
{
	struct device *child;

	child = device_find_child(&i2c->dev, NULL, iio_device_from_i2c_client_match);

	return (child ? dev_to_iio_dev(child) : NULL);
}

static int inv_mpu_slave_enable(struct inv_mpu6050_state *st, int index, bool state)
{
	return regmap_update_bits(st->map, INV_MPU6050_REG_I2C_SLV_CTRL(index),
				  INV_MPU6050_BIT_I2C_SLV_EN,
				  state ? INV_MPU6050_BIT_I2C_SLV_EN : 0);
}

/* Enable slaves based on scan mask */
int inv_mpu_slave_enable_mask(struct inv_mpu6050_state *st,
			      const unsigned long mask)
{
	int i, result;

	for (i = 0; i < INV_MPU6050_MAX_EXT_SENSORS; ++i) {
		long slave_mask = st->ext_sens[i].scan_mask;
		result = inv_mpu_slave_enable(st, i, slave_mask & mask);
		if (result)
			return result;
	}

	return 0;
}

static int inv_mpu_parse_one_ext_sens(struct device *dev,
				      struct device_node *np,
				      struct inv_mpu_ext_sens_spec *spec)
{
	int result;
	u32 addr, reg, len;

	result = of_property_read_u32(np, "invensense,addr", &addr);
	if (result)
		return result;
	result = of_property_read_u32(np, "invensense,reg", &reg);
	if (result)
		return result;
	result = of_property_read_u32(np, "invensense,len", &len);
	if (result)
		return result;

	spec->addr = addr;
	spec->reg = reg;
	spec->len = len;

	result = of_property_count_u32_elems(np, "invensense,external-channels");
	if (result < 0)
		return result;
	spec->num_ext_channels = result;
	spec->ext_channels = devm_kmalloc(dev, spec->num_ext_channels * sizeof(*spec->ext_channels), GFP_KERNEL);
	if (!spec->ext_channels)
		return -ENOMEM;
	result = of_property_read_u32_array(np, "invensense,external-channels",
					    spec->ext_channels,
					    spec->num_ext_channels);
	if (result)
		return result;

	return 0;
}

static int inv_mpu_parse_ext_sens(struct device *dev,
				  struct device_node *node,
				  struct inv_mpu_ext_sens_spec *specs)
{
	struct device_node *child;
	int result;
	u32 reg;

	for_each_available_child_of_node(node, child) {
		result = of_property_read_u32(child, "reg", &reg);
		if (result)
			return result;
		if (reg > INV_MPU6050_MAX_EXT_SENSORS) {
			dev_err(dev, "External sensor index %u out of range, max %d\n",
				reg, INV_MPU6050_MAX_EXT_SENSORS);
			return -EINVAL;
		}
		result = inv_mpu_parse_one_ext_sens(dev, child, &specs[reg]);
		if (result)
			return result;
	}

	return 0;
}

static int inv_mpu_get_ext_sens_spec(struct iio_dev *indio_dev)
{
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(st->map);
	struct device_node *node;
	int result;

	node = of_get_child_by_name(dev->of_node, "invensense,external-sensors");
	if (node) {
		result = inv_mpu_parse_ext_sens(dev, node, st->ext_sens_spec);
		if (result)
			dev_err(dev, "Failed to parsing external-sensors devicetree data\n");
		return result;
	}

	return 0;
}

/* Struct used while enumerating devices and matching them */
struct inv_mpu_handle_ext_sensor_fnarg
{
	struct iio_dev *indio_dev;

	/* Current scan index */
	int scan_index;
	/* Current channel index */
	int chan_index;
	/* Non-const pointer to channels */
	struct iio_chan_spec *channels;
};

/*
 * Write initial configuration of a slave at probe time.
 *
 * This is mostly fixed except for enabling/disabling individual slaves.
 */
static int inv_mpu_config_external_read(struct inv_mpu6050_state *st, int index,
					const struct inv_mpu_ext_sens_spec *spec)
{
	int result;

	result = regmap_write(st->map, INV_MPU6050_REG_I2C_SLV_ADDR(index),
			      INV_MPU6050_BIT_I2C_SLV_RW | spec->addr);
	if (result)
		return result;
	result = regmap_write(st->map, INV_MPU6050_REG_I2C_SLV_REG(index), spec->reg);
	if (result)
		return result;
	result = regmap_write(st->map, INV_MPU6050_REG_I2C_SLV_CTRL(index),
			      spec->len);
	if (result)
		return result;

	return result;
}

/* Handle one device */
static int inv_mpu_handle_slave_device(
		struct inv_mpu_handle_ext_sensor_fnarg *fnarg,
		int slave_index,
		struct iio_dev *orig_dev)
{
	int i, j;
	int data_offset;
	struct iio_dev *indio_dev = fnarg->indio_dev;
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	struct device *mydev = regmap_get_device(st->map);
	struct inv_mpu_ext_sens_spec *ext_sens_spec =
			&st->ext_sens_spec[slave_index];
	struct inv_mpu_ext_sens_state *ext_sens_state =
			&st->ext_sens[slave_index];

	dev_info(mydev, "slave %d is device %s\n",
			slave_index, orig_dev->name ?: dev_name(&orig_dev->dev));
	ext_sens_state->orig_dev = orig_dev;
	ext_sens_state->scan_mask = 0;
	data_offset = 0;

	/* handle channels: */
	for (i = 0; i < ext_sens_spec->num_ext_channels; ++i) {
		int orig_chan_index = -1;
		const struct iio_chan_spec *orig_chan_spec;
		struct iio_chan_spec *chan_spec;
		struct inv_mpu_ext_chan_state *chan_state;

		for (j = 0; j < orig_dev->num_channels; ++j)
			if (orig_dev->channels[j].scan_index == ext_sens_spec->ext_channels[i]) {
				orig_chan_index = j;
				break;
			}

		if (orig_chan_index < 0) {
			dev_err(mydev, "Could not find slave channel with scan_index %d\n",
					ext_sens_spec->ext_channels[i]);
		}

		orig_chan_spec = &orig_dev->channels[orig_chan_index];
		chan_spec = &fnarg->channels[INV_MPU6050_NUM_INT_CHAN + fnarg->chan_index];
		chan_state = &st->ext_chan[fnarg->chan_index];

		chan_state->ext_sens_index = slave_index;
		chan_state->orig_chan_index = orig_chan_index;
		chan_state->data_offset = data_offset;
		memcpy(chan_spec, orig_chan_spec, sizeof(struct iio_chan_spec));
		chan_spec->scan_index = fnarg->scan_index;
		ext_sens_state->scan_mask |= (1 << chan_spec->scan_index);

		fnarg->scan_index++;
		fnarg->chan_index++;
		data_offset += chan_spec->scan_type.storagebits / 8;
		dev_info(mydev, "Reading external channel #%d scan_index %d data_offset %d"
				" from original device %s chan #%d scan_index %d\n",
				fnarg->chan_index, chan_spec->scan_index, chan_state->data_offset,
				orig_dev->name ?: dev_name(&orig_dev->dev), orig_chan_index, orig_chan_spec->scan_index);
	}
	if (ext_sens_spec->len != data_offset) {
		dev_err(mydev, "slave %d length mismatch between "
				"i2c slave read length (%d) and "
				"sum of channel sizes (%d)\n",
				slave_index, ext_sens_spec->len, data_offset);
		return -EINVAL;
	}

	return inv_mpu_config_external_read(st, slave_index, ext_sens_spec);
}

/* device_for_each_child enum function */
static int inv_mpu_handle_ext_sensor_fn(struct device *dev, void *data)
{
	struct inv_mpu_handle_ext_sensor_fnarg *fnarg = data;
	struct iio_dev *indio_dev = fnarg->indio_dev;
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	struct i2c_client *client;
	struct iio_dev *orig_dev;
	int i, result;

	client = i2c_verify_client(dev);
	if (!client)
		return 0;
	orig_dev = iio_device_from_i2c_client(client);
	if (!orig_dev)
		return 0;

	for (i = 0; i < INV_MPU6050_MAX_EXT_SENSORS; ++i) {
		if (st->ext_sens_spec[i].addr != client->addr)
			continue;
		if (st->ext_sens[i].orig_dev) {
			dev_warn(&indio_dev->dev, "already found slave with at addr %d\n", client->addr);
			continue;
		}

		result = inv_mpu_handle_slave_device(fnarg, i, orig_dev);
		if (result)
			return result;
	}
	return 0;
}

static int inv_mpu6050_handle_ext_sensors(struct iio_dev *indio_dev)
{
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(st->map);
	struct inv_mpu_handle_ext_sensor_fnarg fnarg = {
		.indio_dev = indio_dev,
		.chan_index = 0,
		.scan_index = INV_MPU6050_SCAN_TIMESTAMP,
	};
	int i, result;
	int num_ext_chan = 0, sum_data_len = 0;
	int num_scan_elements;

	inv_mpu_get_ext_sens_spec(indio_dev);
	for (i = 0; i < INV_MPU6050_MAX_EXT_SENSORS; ++i) {
		num_ext_chan += st->ext_sens_spec[i].num_ext_channels;
		sum_data_len += st->ext_sens_spec[i].len;
	}
	if (sum_data_len > INV_MPU6050_CNT_EXT_SENS_DATA) {
		dev_err(dev, "Too many bytes from external sensors:"
			      " requested=%d max=%d\n",
			      sum_data_len, INV_MPU6050_CNT_EXT_SENS_DATA);
		return -EINVAL;
	}

	/* Allocate scan_offsets/scan_lengths */
	num_scan_elements = INV_MPU6050_NUM_INT_SCAN_ELEMENTS + num_ext_chan;
	st->scan_offsets = devm_kmalloc(dev, num_scan_elements * sizeof(int), GFP_KERNEL);
	if (!st->scan_offsets)
		return -ENOMEM;
	st->scan_lengths = devm_kmalloc(dev, num_scan_elements * sizeof(int), GFP_KERNEL);
	if (!st->scan_lengths)
		return -ENOMEM;

	/* Zero length means nothing to do, just publish internal channels */
	if (!sum_data_len) {
		indio_dev->channels = inv_mpu_channels;
		indio_dev->num_channels = INV_MPU6050_NUM_INT_CHAN;
		BUILD_BUG_ON(ARRAY_SIZE(inv_mpu_channels) != INV_MPU6050_NUM_INT_CHAN);
		return 0;
	}

	indio_dev->num_channels = INV_MPU6050_NUM_INT_CHAN + num_ext_chan;
	indio_dev->channels = fnarg.channels = devm_kmalloc(dev,
			indio_dev->num_channels * sizeof(struct iio_chan_spec),
			GFP_KERNEL);
	if (!fnarg.channels)
		return -ENOMEM;
	memcpy(fnarg.channels, inv_mpu_channels, sizeof(inv_mpu_channels));
	memset(fnarg.channels + INV_MPU6050_NUM_INT_CHAN, 0,
	       num_ext_chan * sizeof(struct iio_chan_spec));

	st->ext_chan = devm_kzalloc(dev, num_ext_chan * sizeof(*st->ext_chan), GFP_KERNEL);
	if (!st->ext_chan)
		return -ENOMEM;

	result = inv_mpu6050_set_power_itg(st, true);
	if (result < 0)
		return result;

	result = device_for_each_child(&st->aux_master_adapter.dev, &fnarg,
				       inv_mpu_handle_ext_sensor_fn);
	if (result)
		goto out_disable;
	/* Timestamp channel has index 0 and last scan_index */
	fnarg.channels[0].scan_index = fnarg.scan_index;

	if (fnarg.chan_index != num_ext_chan) {
		dev_err(&indio_dev->dev, "Failed to match all external channels!\n");
		result = -EINVAL;
		goto out_disable;
	}

	result = inv_mpu6050_set_power_itg(st, false);
	return result;

out_disable:
	inv_mpu6050_set_power_itg(st, false);
	return result;
}

/**
 *  inv_check_and_setup_chip() - check and setup chip.
 */
static int inv_check_and_setup_chip(struct inv_mpu6050_state *st)
{
	int result;
	unsigned int regval;

	st->hw  = &hw_info[st->chip_type];
	st->reg = hw_info[st->chip_type].reg;

	/* reset to make sure previous state are not there */
	result = regmap_write(st->map, st->reg->pwr_mgmt_1,
			      INV_MPU6050_BIT_H_RESET);
	if (result)
		return result;
	msleep(INV_MPU6050_POWER_UP_TIME);

	/* check chip self-identification */
	result = regmap_read(st->map, INV_MPU6050_REG_WHOAMI, &regval);
	if (result)
		return result;
	if (regval != st->hw->whoami) {
		dev_warn(regmap_get_device(st->map),
				"whoami mismatch got %#02x expected %#02hhx for %s\n",
				regval, st->hw->whoami, st->hw->name);
	}

	/*
	 * toggle power state. After reset, the sleep bit could be on
	 * or off depending on the OTP settings. Toggling power would
	 * make it in a definite state as well as making the hardware
	 * state align with the software state
	 */
	result = inv_mpu6050_set_power_itg(st, false);
	if (result)
		return result;
	result = inv_mpu6050_set_power_itg(st, true);
	if (result)
		return result;

	result = inv_mpu6050_switch_engine(st, false,
					   INV_MPU6050_BIT_PWR_ACCL_STBY);
	if (result)
		return result;
	result = inv_mpu6050_switch_engine(st, false,
					   INV_MPU6050_BIT_PWR_GYRO_STBY);
	if (result)
		return result;

	return 0;
}

static irqreturn_t inv_mpu_irq_handler(int irq, void *private)
{
	struct inv_mpu6050_state *st = (struct inv_mpu6050_state *)private;

	iio_trigger_poll(st->trig);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t inv_mpu_irq_thread_handler(int irq, void *private)
{
	struct inv_mpu6050_state *st = (struct inv_mpu6050_state *)private;
	int ret, val;

	ret = regmap_read(st->map, st->reg->mst_status, &val);
	if (ret < 0)
		return ret;

#ifdef CONFIG_I2C
	if (val & INV_MPU6050_BIT_I2C_SLV4_DONE) {
		st->slv4_done_status = val;
		complete(&st->slv4_done);
	}
#endif

	return IRQ_HANDLED;
}

#ifdef CONFIG_I2C
static u32 inv_mpu_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_BYTE;
}

static int inv_mpu_i2c_smbus_xfer_real(struct i2c_adapter *adap, u16 addr,
				       unsigned short flags, char read_write,
				       u8 command, int size,
				       union i2c_smbus_data *data)
{
	struct inv_mpu6050_state *st = i2c_get_adapdata(adap);
	unsigned long time_left;
	unsigned int val;
	u8 ctrl, status;
	int result;

	if (read_write == I2C_SMBUS_WRITE)
		addr |= INV_MPU6050_BIT_I2C_SLV4_W;
	else
		addr |= INV_MPU6050_BIT_I2C_SLV4_R;

	/*
	 * Consecutive operations with the same address are very common.
	 * Read the old addr from the regmap cache and try to avoid extra
	 * transfers.
	 */
	result = regmap_read(st->map, INV_MPU6050_REG_I2C_SLV4_ADDR, &val);
	if (result < 0)
		return result;
	if (addr != val) {
		result = regmap_write(st->map, INV_MPU6050_REG_I2C_SLV4_ADDR, addr);
		if (result < 0)
			return result;
	}

	if (size == I2C_SMBUS_BYTE_DATA) {
		result = regmap_write(st->map, INV_MPU6050_REG_I2C_SLV4_REG, command);
		if (result < 0)
			return result;
	}

	if (read_write == I2C_SMBUS_WRITE) {
		result = regmap_write(st->map, INV_MPU6050_REG_I2C_SLV4_DO, data->byte);
		if (result < 0)
			return result;
	}

	ctrl = INV_MPU6050_BIT_SLV4_EN | INV_MPU6050_BIT_SLV4_INT_EN;
	if (size == I2C_SMBUS_BYTE)
		ctrl |= INV_MPU6050_BIT_SLV4_REG_DIS;
	result = regmap_write(st->map, INV_MPU6050_REG_I2C_SLV4_CTRL, ctrl);
	if (result < 0)
		return result;

	/* Wait for completion for both reads and writes */
	time_left = wait_for_completion_timeout(&st->slv4_done, HZ);
	if (!time_left)
		return -ETIMEDOUT;

	/* Check status for transfer errors */
	status = st->slv4_done_status;
	if (status & INV_MPU6050_BIT_I2C_SLV4_NACK)
		return -EIO;
	if (status & INV_MPU6050_BIT_I2C_LOST_ARB)
		return -EIO;

	if (read_write == I2C_SMBUS_READ) {
		result = regmap_read(st->map, INV_MPU6050_REG_I2C_SLV4_DI, &val);
		if (result < 0)
			return result;
		data->byte = val;
	}

	return 0;
}

static int inv_mpu_i2c_smbus_xfer(struct i2c_adapter *adap, u16 addr,
				  unsigned short flags, char read_write,
				  u8 command, int size,
				  union i2c_smbus_data *data)
{
	struct inv_mpu6050_state *st = i2c_get_adapdata(adap);
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	int result, power_result;

	/*
	 * The i2c adapter we implement is extremely limited.
	 * Check for callers that don't check functionality bits.
	 *
	 * If we don't check we might succesfully return incorrect data.
	 */
	if (size != I2C_SMBUS_BYTE_DATA && size != I2C_SMBUS_BYTE) {
		dev_err(&adap->dev, "unsupported xfer rw=%d size=%d\n",
			read_write, size);
		return -EINVAL;
	}

	mutex_lock(&indio_dev->mlock);
	power_result = inv_mpu6050_set_power_itg(st, true);
	mutex_unlock(&indio_dev->mlock);
	if (power_result < 0)
		return power_result;

	result = inv_mpu_i2c_smbus_xfer_real(adap, addr, flags, read_write, command, size, data);

	mutex_lock(&indio_dev->mlock);
	power_result = inv_mpu6050_set_power_itg(st, false);
	mutex_unlock(&indio_dev->mlock);
	if (power_result < 0)
		return power_result;

	return result;
}

static const struct i2c_algorithm inv_mpu_i2c_algo = {
	.smbus_xfer	=	inv_mpu_i2c_smbus_xfer,
	.functionality	=	inv_mpu_i2c_functionality,
};

static struct device_node *inv_mpu_get_aux_i2c_ofnode(struct device_node *parent)
{
	struct device_node *child;
	int result;
	u32 reg;

	if (!parent)
		return NULL;
	for_each_child_of_node(parent, child) {
		result = of_property_read_u32(child, "reg", &reg);
		if (result)
			continue;
		if (reg == 1 && !strcmp(child->name, "i2c"))
			return child;
	}

	return NULL;
}

static int inv_mpu_probe_aux_master(struct iio_dev* indio_dev)
{
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(st->map);
	int result;

	/* First check i2c-aux-master property */
	st->i2c_aux_master_mode = of_property_read_bool(dev->of_node,
							"invensense,i2c-aux-master");
	if (!st->i2c_aux_master_mode)
		return 0;
	dev_info(dev, "Configuring aux i2c in master mode\n");

	result = inv_mpu6050_set_power_itg(st, true);
	if (result < 0)
		return result;

	/* enable master */
	st->chip_config.user_ctrl |= INV_MPU6050_BIT_I2C_MST_EN;
	result = regmap_write(st->map, st->reg->user_ctrl, st->chip_config.user_ctrl);
	if (result < 0)
		return result;

	result = regmap_update_bits(st->map, st->reg->int_enable,
				 INV_MPU6050_BIT_MST_INT_EN,
				 INV_MPU6050_BIT_MST_INT_EN);
	if (result < 0)
		return result;

	result = inv_mpu6050_set_power_itg(st, false);
	if (result < 0)
		return result;

	init_completion(&st->slv4_done);
	st->aux_master_adapter.owner = THIS_MODULE;
	st->aux_master_adapter.algo = &inv_mpu_i2c_algo;
	st->aux_master_adapter.dev.parent = dev;
	snprintf(st->aux_master_adapter.name, sizeof(st->aux_master_adapter.name),
			"aux-master-%s", indio_dev->name);
	st->aux_master_adapter.dev.of_node = inv_mpu_get_aux_i2c_ofnode(dev->of_node);
	i2c_set_adapdata(&st->aux_master_adapter, st);
	/* This will also probe aux devices so transfers must work now */
	result = i2c_add_adapter(&st->aux_master_adapter);
	if (result < 0) {
		dev_err(dev, "i2x aux master register fail %d\n", result);
		return result;
	}

	return 0;
}
#endif

int inv_mpu_core_probe(struct regmap *regmap, int irq, const char *name,
		int (*inv_mpu_bus_setup)(struct iio_dev *), int chip_type)
{
	struct inv_mpu6050_state *st;
	struct iio_dev *indio_dev;
	struct inv_mpu6050_platform_data *pdata;
	struct device *dev = regmap_get_device(regmap);
	int result;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	BUILD_BUG_ON(ARRAY_SIZE(hw_info) != INV_NUM_PARTS);
	if (chip_type < 0 || chip_type >= INV_NUM_PARTS) {
		dev_err(dev, "Bad invensense chip_type=%d name=%s\n",
				chip_type, name);
		return -ENODEV;
	}
	st = iio_priv(indio_dev);
	st->chip_type = chip_type;
	st->powerup_count = 0;
	st->irq = irq;
	st->map = regmap;

	pdata = dev_get_platdata(dev);
	if (!pdata) {
		result = of_iio_read_mount_matrix(dev, "mount-matrix",
						  &st->orientation);
		if (result) {
			dev_err(dev, "Failed to retrieve mounting matrix %d\n",
				result);
			return result;
		}
	} else {
		st->plat_data = *pdata;
	}

	/* power is turned on inside check chip type*/
	result = inv_check_and_setup_chip(st);
	if (result)
		return result;

	if (inv_mpu_bus_setup)
		inv_mpu_bus_setup(indio_dev);

	result = inv_mpu6050_init_config(indio_dev);
	if (result) {
		dev_err(dev, "Could not initialize device.\n");
		return result;
	}

	dev_set_drvdata(dev, indio_dev);
	indio_dev->dev.parent = dev;
	/* name will be NULL when enumerated via ACPI */
	if (name)
		indio_dev->name = name;
	else
		indio_dev->name = dev_name(dev);

	indio_dev->info = &mpu_info;
	indio_dev->modes = INDIO_BUFFER_TRIGGERED;

	result = iio_triggered_buffer_setup(indio_dev,
					    inv_mpu6050_irq_handler,
					    inv_mpu6050_read_fifo,
					    NULL);
	if (result) {
		dev_err(dev, "configure buffer fail %d\n", result);
		return result;
	}
	result = inv_mpu6050_probe_trigger(indio_dev);
	if (result) {
		dev_err(dev, "trigger probe fail %d\n", result);
		goto out_unreg_ring;
	}

	/* Request interrupt for trigger and i2c master adapter */
	result = devm_request_threaded_irq(&indio_dev->dev, st->irq,
					   &inv_mpu_irq_handler,
					   &inv_mpu_irq_thread_handler,
					   IRQF_TRIGGER_RISING, "inv_mpu",
					   st);
	if (result) {
		dev_err(dev, "request irq fail %d\n", result);
		goto out_remove_trigger;
	}

#ifdef CONFIG_I2C
	result = inv_mpu_probe_aux_master(indio_dev);
	if (result < 0) {
		dev_err(dev, "i2c aux master probe fail %d\n", result);
		goto out_remove_trigger;
	}
#endif

	result = inv_mpu6050_handle_ext_sensors(indio_dev);
	if (result < 0) {
		dev_err(dev, "failed to configure channels %d\n", result);
		goto out_remove_trigger;
	}

	INIT_KFIFO(st->timestamps);
	spin_lock_init(&st->time_stamp_lock);
	result = iio_device_register(indio_dev);
	if (result) {
		dev_err(dev, "IIO register fail %d\n", result);
		goto out_del_adapter;
	}

	return 0;

out_del_adapter:
#ifdef CONFIG_I2C
	i2c_del_adapter(&st->aux_master_adapter);
#endif
out_remove_trigger:
	inv_mpu6050_remove_trigger(st);
out_unreg_ring:
	iio_triggered_buffer_cleanup(indio_dev);
	return result;
}
EXPORT_SYMBOL_GPL(inv_mpu_core_probe);

int inv_mpu_core_remove(struct device  *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu6050_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	inv_mpu6050_remove_trigger(iio_priv(indio_dev));
	iio_triggered_buffer_cleanup(indio_dev);
#ifdef CONFIG_I2C
	i2c_del_adapter(&st->aux_master_adapter);
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(inv_mpu_core_remove);

#ifdef CONFIG_PM_SLEEP

static int inv_mpu_resume(struct device *dev)
{
	return inv_mpu6050_set_power_itg(iio_priv(dev_get_drvdata(dev)), true);
}

static int inv_mpu_suspend(struct device *dev)
{
	return inv_mpu6050_set_power_itg(iio_priv(dev_get_drvdata(dev)), false);
}
#endif /* CONFIG_PM_SLEEP */

SIMPLE_DEV_PM_OPS(inv_mpu_pmops, inv_mpu_suspend, inv_mpu_resume);
EXPORT_SYMBOL_GPL(inv_mpu_pmops);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense device MPU6050 driver");
MODULE_LICENSE("GPL");
