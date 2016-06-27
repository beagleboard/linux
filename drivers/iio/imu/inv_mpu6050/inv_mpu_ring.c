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
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/spi/spi.h>
#include "inv_mpu_iio.h"

static void inv_mpu_get_scan_offsets(
		struct iio_dev *indio_dev,
		const unsigned long *mask,
		const unsigned int masklen,
		unsigned int *scan_offsets)
{
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	unsigned int pos = 0;
	int i, j;

	if (*mask & INV_MPU6050_SCAN_MASK_ACCEL) {
		scan_offsets[INV_MPU6050_SCAN_ACCL_X] = pos + 0;
		scan_offsets[INV_MPU6050_SCAN_ACCL_Y] = pos + 2;
		scan_offsets[INV_MPU6050_SCAN_ACCL_Z] = pos + 4;
		pos += 6;
	}
	if (*mask & INV_MPU6050_SCAN_MASK_GYRO) {
		scan_offsets[INV_MPU6050_SCAN_GYRO_X] = pos + 0;
		scan_offsets[INV_MPU6050_SCAN_GYRO_Y] = pos + 2;
		scan_offsets[INV_MPU6050_SCAN_GYRO_Z] = pos + 4;
		pos += 6;
	}
	/* HW lays out channels in slave order */
	for (i = 0; i < INV_MPU6050_MAX_EXT_SENSORS; ++i) {
		struct inv_mpu_ext_sens_spec *ext_sens_spec;
		struct inv_mpu_ext_sens_state *ext_sens_state;
		ext_sens_spec = &st->ext_sens_spec[i];
		ext_sens_state = &st->ext_sens[i];

		if (!(ext_sens_state->scan_mask & *mask))
			continue;
		for (j = 0; j + INV_MPU6050_NUM_INT_CHAN < indio_dev->num_channels; ++j) {
			const struct iio_chan_spec *chan;
			if (st->ext_chan[j].ext_sens_index != i)
				continue;
			chan = &indio_dev->channels[j + INV_MPU6050_NUM_INT_CHAN];
			scan_offsets[chan->scan_index] = pos + st->ext_chan[j].data_offset;
		}
		pos += ext_sens_spec->len;
	}
}

/* This is slowish but relatively common */
static const struct iio_chan_spec *
iio_chan_by_scan_index(struct iio_dev *indio_dev, int index)
{
	int i;

	for (i = 0; i < indio_dev->num_channels; ++i)
		if (indio_dev->channels[i].scan_index == index)
			return &indio_dev->channels[i];

	return NULL;
}

static int iio_check_scan_offsets_aligned(
		struct iio_dev *indio_dev,
		const unsigned long *mask,
		unsigned int masklen,
		unsigned int *scan_offsets)
{
	int scan_index;
	unsigned int pos = 0;
	const struct iio_chan_spec *chan;

	for_each_set_bit(scan_index, mask, masklen) {
		chan = iio_chan_by_scan_index(indio_dev, scan_index);
		if (scan_offsets[scan_index] != pos)
			return false;
		pos = ALIGN(pos + chan->scan_type.storagebits / 8,
			    chan->scan_type.storagebits / 8);
	}

	return true;
}

static void iio_get_scan_lengths(struct iio_dev *indio_dev, unsigned int *scan_length)
{
	int i;
	const struct iio_chan_spec *chan;

	for (i = 0; i < indio_dev->num_channels; ++i) {
		chan = &indio_dev->channels[i];
		if (chan->scan_index < 0)
			continue;
		scan_length[chan->scan_index] = chan->scan_type.storagebits / 8;
	}
}

static void iio_realign_sample(
		struct iio_dev *indio_dev,
		const u8 *ibuf, u8 *obuf,
		const unsigned int *scan_offset,
		const unsigned int *scan_lengths)
{
	unsigned int pos = 0;
	int i;

	for_each_set_bit(i, indio_dev->active_scan_mask, indio_dev->masklength) {
		memcpy(&obuf[pos], &ibuf[scan_offset[i]], scan_lengths[i]);
		pos = ALIGN(pos + scan_lengths[i], scan_lengths[i]);
	}
}

static void inv_clear_kfifo(struct inv_mpu6050_state *st)
{
	unsigned long flags;

	/* take the spin lock sem to avoid interrupt kick in */
	spin_lock_irqsave(&st->time_stamp_lock, flags);
	kfifo_reset(&st->timestamps);
	spin_unlock_irqrestore(&st->time_stamp_lock, flags);
}

int inv_reset_fifo(struct iio_dev *indio_dev)
{
	int result;
	u8 d;
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	const unsigned long *mask = indio_dev->active_scan_mask;
	unsigned int masklen = indio_dev->masklength;

	/* disable interrupt */
	result = regmap_update_bits(st->map, st->reg->int_enable,
				    INV_MPU6050_BIT_DATA_RDY_EN, 0);
	if (result) {
		dev_err(regmap_get_device(st->map), "int_enable failed %d\n",
			result);
		return result;
	}
	/* disable the sensor output to FIFO */
	result = regmap_write(st->map, st->reg->fifo_en, 0);
	if (result)
		goto reset_fifo_fail;
	result = regmap_update_bits(st->map, INV_MPU6050_REG_MST_CTRL,
				    INV_MPU6050_BIT_SLV3_FIFO_EN, 0);
	if (result)
		goto reset_fifo_fail;
	/* disable fifo reading */
	st->chip_config.user_ctrl &= ~INV_MPU6050_BIT_FIFO_EN;
	result = regmap_write(st->map, st->reg->user_ctrl,
			      st->chip_config.user_ctrl);
	if (result)
		goto reset_fifo_fail;

	/* reset FIFO*/
	result = regmap_write(st->map, st->reg->user_ctrl,
			      st->chip_config.user_ctrl | INV_MPU6050_BIT_FIFO_RST);
	if (result)
		goto reset_fifo_fail;

	/* clear timestamps fifo */
	inv_clear_kfifo(st);

	/* enable interrupt */
	result = regmap_update_bits(st->map, st->reg->int_enable,
				    INV_MPU6050_BIT_DATA_RDY_EN,
				    INV_MPU6050_BIT_DATA_RDY_EN);
	if (result)
		return result;

	/* enable FIFO reading and I2C master interface*/
	st->chip_config.user_ctrl |= INV_MPU6050_BIT_FIFO_EN;
	result = regmap_write(st->map, st->reg->user_ctrl,
			      st->chip_config.user_ctrl);
	if (result)
		goto reset_fifo_fail;
	/* enable sensor output to FIFO */
	d = 0;
	if (st->chip_config.gyro_fifo_enable)
		d |= INV_MPU6050_BITS_GYRO_OUT;
	if (st->chip_config.accl_fifo_enable)
		d |= INV_MPU6050_BIT_ACCEL_OUT;
	if (*indio_dev->active_scan_mask & st->ext_sens[0].scan_mask)
		d |= INV_MPU6050_BIT_SLV0_FIFO_EN;
	if (*indio_dev->active_scan_mask & st->ext_sens[1].scan_mask)
		d |= INV_MPU6050_BIT_SLV1_FIFO_EN;
	if (*indio_dev->active_scan_mask & st->ext_sens[2].scan_mask)
		d |= INV_MPU6050_BIT_SLV2_FIFO_EN;
	result = regmap_write(st->map, st->reg->fifo_en, d);
	if (result)
		goto reset_fifo_fail;
	if (*indio_dev->active_scan_mask & st->ext_sens[3].scan_mask) {
		result = regmap_update_bits(st->map, INV_MPU6050_REG_MST_CTRL,
					    INV_MPU6050_BIT_SLV3_FIFO_EN,
					    INV_MPU6050_BIT_SLV3_FIFO_EN);
		if (result)
			goto reset_fifo_fail;
	}

	/* check realign required */
	inv_mpu_get_scan_offsets(indio_dev, mask, masklen, st->scan_offsets);
	st->realign_required = !iio_check_scan_offsets_aligned(
			indio_dev, mask, masklen, st->scan_offsets);
	if (st->realign_required)
		iio_get_scan_lengths(indio_dev, st->scan_lengths);

	return 0;

reset_fifo_fail:
	dev_err(regmap_get_device(st->map), "reset fifo failed %d\n", result);
	result = regmap_update_bits(st->map, st->reg->int_enable,
				    INV_MPU6050_BIT_DATA_RDY_EN,
				    INV_MPU6050_BIT_DATA_RDY_EN);

	return result;
}

/**
 * inv_mpu6050_irq_handler() - Cache a timestamp at each data ready interrupt.
 */
irqreturn_t inv_mpu6050_irq_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	s64 timestamp;

	timestamp = iio_get_time_ns();
	kfifo_in_spinlocked(&st->timestamps, &timestamp, 1,
			    &st->time_stamp_lock);

	return IRQ_WAKE_THREAD;
}

/**
 * inv_mpu6050_read_fifo() - Transfer data from hardware FIFO to KFIFO.
 */
irqreturn_t inv_mpu6050_read_fifo(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	size_t bytes_per_datum = 0;
	int result;
	int i;
	u8 data[INV_MPU6050_OUTPUT_DATA_SIZE];
	u16 fifo_count;
	s64 timestamp;

	struct device *regmap_dev = regmap_get_device(st->map);
	struct i2c_client *i2c;
	struct spi_device *spi = NULL;

	i2c = i2c_verify_client(regmap_dev);
	spi = i2c ? NULL: to_spi_device(regmap_dev);

	mutex_lock(&indio_dev->mlock);

	/* Sample length */
	bytes_per_datum = 0;
	if (st->chip_config.accl_fifo_enable)
		bytes_per_datum += INV_MPU6050_BYTES_PER_3AXIS_SENSOR;
	if (st->chip_config.gyro_fifo_enable)
		bytes_per_datum += INV_MPU6050_BYTES_PER_3AXIS_SENSOR;
	for (i = 0; i < INV_MPU6050_MAX_EXT_SENSORS; ++i)
		if (st->ext_sens[i].scan_mask & *indio_dev->active_scan_mask)
			bytes_per_datum += st->ext_sens_spec[i].len;
	if (!bytes_per_datum)
		return 0;

	/*
	 * read fifo_count register to know how many bytes inside FIFO
	 * right now
	 */
	result = regmap_bulk_read(st->map, st->reg->fifo_count_h, data,
				  INV_MPU6050_FIFO_COUNT_BYTE);
	if (result)
		goto end_session;
	fifo_count = be16_to_cpup((__be16 *)(&data[0]));
	if (fifo_count < bytes_per_datum)
		goto end_session;
	/* fifo count can't be odd number, if it is odd, reset fifo*/
	if (fifo_count & 1)
		goto flush_fifo;
	if (fifo_count >  INV_MPU6050_FIFO_THRESHOLD)
		goto flush_fifo;
	/* Timestamp mismatch. */
	if (kfifo_len(&st->timestamps) >
	    fifo_count / bytes_per_datum + INV_MPU6050_TIME_STAMP_TOR)
		goto flush_fifo;
	while (fifo_count >= bytes_per_datum) {
		/*
		 * We need to do a large burst read from a single register.
		 *
		 * regmap_read_bulk assumes that multiple registers are
		 * involved but in our case st->reg->fifo_r_w + 1 is something
		 * completely unrelated.
		 */
		if (spi) {
			u8 cmd = st->reg->fifo_r_w | 0x80;
			result = spi_write_then_read(spi,
					&cmd, 1,
					data, bytes_per_datum);
			if (result)
				goto flush_fifo;
		} else {
			result = i2c_smbus_read_i2c_block_data(i2c,
					st->reg->fifo_r_w,
					bytes_per_datum, data);
			if (result != bytes_per_datum)
				goto flush_fifo;
		}

		result = kfifo_out(&st->timestamps, &timestamp, 1);
		/* when there is no timestamp, put timestamp as 0 */
		if (result == 0)
			timestamp = 0;

		if (st->realign_required) {
			u8 adata[INV_MPU6050_OUTPUT_DATA_SIZE];
			iio_realign_sample(indio_dev, data, adata,
					    st->scan_offsets, st->scan_lengths);
			result = iio_push_to_buffers_with_timestamp(
					indio_dev, adata, timestamp);
		} else {
			result = iio_push_to_buffers_with_timestamp(
					indio_dev, data, timestamp);
		}
		if (result)
			goto flush_fifo;
		fifo_count -= bytes_per_datum;
	}

end_session:
	mutex_unlock(&indio_dev->mlock);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;

flush_fifo:
	/* Flush HW and SW FIFOs. */
	inv_reset_fifo(indio_dev);
	mutex_unlock(&indio_dev->mlock);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}
