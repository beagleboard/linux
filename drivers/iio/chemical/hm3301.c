// SPDX-License-Identifier: GPL-2.0
/*
 * Sensirion HM3301 particulate matter sensor driver
 *
 * Copyright (c) Tomasz Duszynski <zuobaozhu@gmail.com>
 *
 * I2C slave address: 0x40
 */

#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/kernel.h>
#include <linux/module.h>


/* minimum and maximum self cleaning periods in seconds */
#define HM3301_AUTO_CLEANING_PERIOD_MIN 0
#define HM3301_AUTO_CLEANING_PERIOD_MAX 604800

/* HM3301 commands */
#define HM3301_START_MEAS 0x0010
#define HM3301_STOP_MEAS 0x0104
#define HM3301_READ_DATA_READY_FLAG 0x0202
#define HM3301_READ_DATA 0x0300
#define HM3301_READ_SERIAL 0xd033
#define HM3301_START_FAN_CLEANING 0x5607
#define HM3301_AUTO_CLEANING_PERIOD 0x8004
/* not a sensor command per se, used only to distinguish write from read */
#define HM3301_READ_AUTO_CLEANING_PERIOD 0x8005

enum {
	PM1,
	PM2P5,
	PM10,
};

enum {
	RESET,
	MEASURING,
};

struct hm3301_state {
	struct i2c_client *client;
	/*
	 * Guards against concurrent access to sensor registers.
	 * Must be held whenever sequence of commands is to be executed.
	 */
	struct mutex lock;
	int state;
};


static int hm3301_write_then_read(struct hm3301_state *state, u8 *txbuf,
				 int txsize, u8 *rxbuf, int rxsize)
{
	int ret;  
	/*
	 * Sensor does not support repeated start so instead of
	 * sending two i2c messages in a row we just send one by one.
	 */
	ret = i2c_master_send(state->client, txbuf, txsize);
	if (ret != txsize)
		return ret < 0 ? ret : -EIO;
      
        msleep(10);
	if (!rxbuf)
		return 0;

	ret = i2c_master_recv(state->client, rxbuf, rxsize);
	if (ret != rxsize)
		return ret < 0 ? ret : -EIO;

	return 0;
}

static int hm3301_do_cmd(struct hm3301_state *state, u16 cmd, u8 *data, int size)
{
	/*
	 * Internally sensor stores measurements in a following manner:
	 *
	 * PM1: upper two bytes, crc8, lower two bytes, crc8
	 * PM2P5: upper two bytes, crc8, lower two bytes, crc8
	 * PM10: upper two bytes, crc8, lower two bytes, crc8
	 *
	 * What follows next are number concentration measurements and
	 * typical particle size measurement which we omit.
	 */
	u8 txbuf[2];
	u8 rxbuf[49] = {8};
	int i, ret = 0;

	switch (cmd) {
	case HM3301_START_MEAS:
	     txbuf[0] = 0;
             txbuf[1] = 0x88;
	     ret = hm3301_write_then_read(state, txbuf, 2, NULL, 0);
	        break;
	case HM3301_STOP_MEAS:
	case HM3301_START_FAN_CLEANING:
		break;
	case HM3301_READ_AUTO_CLEANING_PERIOD:
		/* fall through */
	case HM3301_READ_DATA_READY_FLAG:
	case HM3301_READ_DATA:
	case HM3301_READ_SERIAL:
		/* every two data bytes are checksummed */
                txbuf[0] = 0x40;
                txbuf[1] = 0;
		ret = hm3301_write_then_read(state, txbuf, 2, rxbuf, 29);
	/* validate received data and strip off crc bytes */
        	for (i = 0; i < 29; i ++) {
		  *data++ = rxbuf[i];
	        }
		break;
	case HM3301_AUTO_CLEANING_PERIOD:
		break;
	}

	if (ret)
		return ret;


	return 0;
}



static int hm3301_do_meas(struct hm3301_state *state, s32 *data, int size)
{
	int i, ret;
	u8 tmp[32];

	if (state->state == RESET) {
		ret = hm3301_do_cmd(state, HM3301_START_MEAS, NULL, 0);
		if (ret)
			return ret;

		state->state = MEASURING;
	}
   
#if 0
	while (tries--) {
		ret = hm3301_do_cmd(state, HM3301_READ_DATA_READY_FLAG, tmp, 2);
		if (ret)
			return -EIO;

		/* new measurements ready to be read */
		if (tmp[1] == 1)
			break;

		msleep_interruptible(300);
	}

	if (tries == -1)
		return -ETIMEDOUT;
#endif
	ret = hm3301_do_cmd(state, HM3301_READ_DATA, tmp, sizeof(int) * size);
	if (ret)
		return ret;

        int a;
        for (i = 0; i < size; i++) {
	    for (a = 2; a< (size+2); a++)
		{
	            u16 value = 0;
		    value = (u16) tmp[a * 2] << 8 | tmp[a * 2 +1];
                    data[i] = value * 100; 
		}
	}
	return 0;
}

static irqreturn_t hm3301_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct hm3301_state *state = iio_priv(indio_dev);
	int ret;
	struct {
		s32 data[3]; /* PM1, PM2P5, PM10 */
		s64 ts;
	} scan;

	mutex_lock(&state->lock);
	ret = hm3301_do_meas(state, scan.data, ARRAY_SIZE(scan.data));
	mutex_unlock(&state->lock);
	if (ret)
		goto err;

	iio_push_to_buffers_with_timestamp(indio_dev, &scan,
					   iio_get_time_ns(indio_dev));
err:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int hm3301_read_raw(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan,
			  int *val, int *val2, long mask)
{
	struct hm3301_state *state = iio_priv(indio_dev);
	int data[4], ret = -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		switch (chan->type) {
		case IIO_MASSCONCENTRATION:
			mutex_lock(&state->lock);
			/* read up to the number of bytes actually needed */
			switch (chan->channel2) {
			case IIO_MOD_PM1:
				ret = hm3301_do_meas(state, data, 1);
				break;
			case IIO_MOD_PM2P5:
				ret = hm3301_do_meas(state, data, 2);
				break;
			case IIO_MOD_PM10:
				ret = hm3301_do_meas(state, data, 3);
				break;
			}
			mutex_unlock(&state->lock);
			if (ret)
				return ret;

			*val = data[chan->address] / 100;
			*val2 = (data[chan->address] % 100) * 10000;

			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_MASSCONCENTRATION:
			switch (chan->channel2) {
			case IIO_MOD_PM1:
			case IIO_MOD_PM2P5:
			case IIO_MOD_PM10:
				*val = 0;
				*val2 = 10000;

				return IIO_VAL_INT_PLUS_MICRO;
			default:
				return -EINVAL;
			}
		default:
			return -EINVAL;
		}
	}

	return -EINVAL;
}



static ssize_t start_cleaning_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct hm3301_state *state = iio_priv(indio_dev);
	int val, ret;

	if (kstrtoint(buf, 0, &val) || val != 1)
		return -EINVAL;

	mutex_lock(&state->lock);
	ret = hm3301_do_cmd(state, HM3301_START_FAN_CLEANING, NULL, 0);
	mutex_unlock(&state->lock);
	if (ret)
		return ret;

	return len;
}

static ssize_t cleaning_period_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct hm3301_state *state = iio_priv(indio_dev);
	u8 tmp[4];
	int ret;

	mutex_lock(&state->lock);
	ret = hm3301_do_cmd(state, HM3301_READ_AUTO_CLEANING_PERIOD, tmp, 4);
	mutex_unlock(&state->lock);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", get_unaligned_be32(tmp));
}

static ssize_t cleaning_period_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct hm3301_state *state = iio_priv(indio_dev);
	int val, ret;
	u8 tmp[4];

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	if ((val < HM3301_AUTO_CLEANING_PERIOD_MIN) ||
	    (val > HM3301_AUTO_CLEANING_PERIOD_MAX))
		return -EINVAL;

	put_unaligned_be32(val, tmp);

	mutex_lock(&state->lock);
	ret = hm3301_do_cmd(state, HM3301_AUTO_CLEANING_PERIOD, tmp, 0);
	if (ret) {
		mutex_unlock(&state->lock);
		return ret;
	}

	msleep(20);

	/*
	 * sensor requires reset in order to return up to date self cleaning
	 * period
	 */
//	ret = hm3301_do_cmd_reset(state);
//	if (ret)
//		dev_warn(dev,
//			 "period changed but reads will return the old value\n");

	mutex_unlock(&state->lock);

	return len;
}

static ssize_t cleaning_period_available_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	return snprintf(buf, PAGE_SIZE, "[%d %d %d]\n",
			HM3301_AUTO_CLEANING_PERIOD_MIN, 1,
			HM3301_AUTO_CLEANING_PERIOD_MAX);
}

static IIO_DEVICE_ATTR_WO(start_cleaning, 0);
static IIO_DEVICE_ATTR_RW(cleaning_period, 0);
static IIO_DEVICE_ATTR_RO(cleaning_period_available, 0);

static struct attribute *hm3301_attrs[] = {
	&iio_dev_attr_start_cleaning.dev_attr.attr,
	&iio_dev_attr_cleaning_period.dev_attr.attr,
	&iio_dev_attr_cleaning_period_available.dev_attr.attr,
	NULL
};

static const struct attribute_group hm3301_attr_group = {
	.attrs = hm3301_attrs,
};

static const struct iio_info hm3301_info = {
	.attrs = &hm3301_attr_group,
	.read_raw = hm3301_read_raw,
};

#define HM3301_CHAN(_index, _mod) { \
	.type = IIO_MASSCONCENTRATION, \
	.modified = 1, \
	.channel2 = IIO_MOD_ ## _mod, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.address = _mod, \
	.scan_index = _index, \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 19, \
		.storagebits = 32, \
		.endianness = IIO_CPU, \
	}, \
}

static const struct iio_chan_spec hm3301_channels[] = {
	HM3301_CHAN(0, PM1),
	HM3301_CHAN(1, PM2P5),
	HM3301_CHAN(2, PM10),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static void hm3301_stop_meas(void *data)
{
	struct hm3301_state *state = data;
	hm3301_do_cmd(state, HM3301_STOP_MEAS, NULL, 0);
}

static const unsigned long hm3301_scan_masks[] = { 0x0f, 0x00 };

static int hm3301_probe(struct i2c_client *client)
{
	struct iio_dev *indio_dev;
	struct hm3301_state *state;
	u8 buf[32];
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*state));
	if (!indio_dev)
		return -ENOMEM;

	state = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	state->client = client;
	state->state = RESET;
	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &hm3301_info;
	indio_dev->name = client->name;
	indio_dev->channels = hm3301_channels;
	indio_dev->num_channels = ARRAY_SIZE(hm3301_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->available_scan_masks = hm3301_scan_masks;

	mutex_init(&state->lock);
	ret = devm_add_action_or_reset(&client->dev, hm3301_stop_meas, state);
	if (ret)
		return ret;
	ret = devm_iio_triggered_buffer_setup(&client->dev, indio_dev, NULL,
					      hm3301_trigger_handler, NULL);
	if (ret)
		return ret;

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct i2c_device_id hm3301_id[] = {
	{ "hm3301" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hm3301_id);

static const struct of_device_id hm3301_of_match[] = {
	{ .compatible = "seeed,hm3301" },
	{ }
};
MODULE_DEVICE_TABLE(of, hm3301_of_match);

static struct i2c_driver hm3301_driver = {
	.driver = {
		.name = "hm3301",
		.of_match_table = hm3301_of_match,
	},
	.id_table = hm3301_id,
	.probe_new = hm3301_probe,
};
module_i2c_driver(hm3301_driver);

MODULE_AUTHOR("Baozhu Zuo <zuobaozhu@gmail.com>");
MODULE_DESCRIPTION("HM3301 laser dust  detection   sensor driver");
MODULE_LICENSE("GPL v2");

