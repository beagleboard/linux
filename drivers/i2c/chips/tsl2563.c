/*
 * drivers/i2c/chips/tsl2563.c
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Written by Timo O. Karjalainen <timo.o.karjalainen@nokia.com>
 * Contact: Mathias Nyman <mathias.nyman@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/hwmon.h>
#include <linux/err.h>
#include <mach/board.h>

#define DRIVER_NAME  "tsl2563"

/* Use this many bits for fraction part. */
#define ADC_FRAC_BITS		(14)

/* Given number of 1/10000's in ADC_FRAC_BITS precision. */
#define FRAC10K(f)		(((f) * (1L << (ADC_FRAC_BITS))) / (10000))

/* Bits used for fraction in calibration coefficients.*/
#define CALIB_FRAC_BITS		(10)
/* 0.5 in CALIB_FRAC_BITS precision */
#define CALIB_FRAC_HALF		(1 << (CALIB_FRAC_BITS - 1))
/* Make a fraction from a number n that was multiplied with b. */
#define CALIB_FRAC(n, b)	(((n) << CALIB_FRAC_BITS) / (b))
/* Decimal 10^(digits in sysfs presentation) */
#define CALIB_BASE_SYSFS	(1000)

#define TSL2563_CMD		(0x80)
#define TSL2563_CLEARINT	(0x40)

#define TSL2563_REG_CTRL	(0x00)
#define TSL2563_REG_TIMING	(0x01)
#define TSL2563_REG_LOWLOW	(0x02) /* data0 low threshold, 2 bytes */
#define TSL2563_REG_LOWHIGH	(0x03)
#define TSL2563_REG_HIGHLOW	(0x04) /* data0 high threshold, 2 bytes */
#define TSL2563_REG_HIGHHIGH	(0x05)
#define TSL2563_REG_INT		(0x06)
#define TSL2563_REG_ID		(0x0a)
#define TSL2563_REG_DATA0LOW	(0x0c) /* broadband sensor value, 2 bytes */
#define TSL2563_REG_DATA0HIGH	(0x0d)
#define TSL2563_REG_DATA1LOW	(0x0e) /* infrared sensor value, 2 bytes */
#define TSL2563_REG_DATA1HIGH	(0x0f)

#define TSL2563_CMD_POWER_ON	(0x03)
#define TSL2563_CMD_POWER_OFF	(0x00)
#define TSL2563_CTRL_POWER_MASK	(0x03)

#define TSL2563_TIMING_13MS	(0x00)
#define TSL2563_TIMING_100MS	(0x01)
#define TSL2563_TIMING_400MS	(0x02)
#define TSL2563_TIMING_MASK	(0x03)
#define TSL2563_TIMING_GAIN16	(0x10)
#define TSL2563_TIMING_GAIN1	(0x00)

#define TSL2563_INT_DISBLED	(0x00)
#define TSL2563_INT_LEVEL	(0x10)
#define TSL2563_INT_PERSIST(n)	((n) & 0x0F)

struct tsl2563_gainlevel_coeff {
	u8 gaintime;
	u16 min;
	u16 max;
};

static struct tsl2563_gainlevel_coeff tsl2563_gainlevel_table[] = {
	{
		.gaintime	= TSL2563_TIMING_400MS | TSL2563_TIMING_GAIN16,
		.min		= 0,
		.max		= 65534,
	}, {
		.gaintime	= TSL2563_TIMING_400MS | TSL2563_TIMING_GAIN1,
		.min		= 2048,
		.max		= 65534,
	}, {
		.gaintime	= TSL2563_TIMING_100MS | TSL2563_TIMING_GAIN1,
		.min		= 4095,
		.max		= 37177,
	}, {
		.gaintime	= TSL2563_TIMING_13MS | TSL2563_TIMING_GAIN1,
		.min		= 3000,
		.max		= 65535,
	},
};

struct tsl2563_chip {
	struct mutex		lock;
	struct i2c_client	*client;
	struct device		*hwmon_dev;

	/* Remember state for suspend and resume functions */
	pm_message_t		state;

	struct tsl2563_gainlevel_coeff *gainlevel;

	/* Thresholds are in lux */
	u16			low_thres;
	u16			high_thres;
	u8			intr;

	/* Calibration coefficients */
	u32			calib0;
	u32			calib1;

	/* Cache current values, to be returned while suspended */
	u32			data0;
	u32			data1;
};

static int tsl2563_write(struct i2c_client *client, u8 reg, u8 value)
{
	int ret;
	u8 buf[2];

	buf[0] = TSL2563_CMD | reg;
	buf[1] = value;

	ret = i2c_master_send(client, buf, sizeof(buf));
	return (ret == sizeof(buf)) ? 0 : ret;
}

static int tsl2563_read(struct i2c_client *client, u8 reg, void *buf, int len)
{
	int ret;
	u8 cmd = TSL2563_CMD | reg;

	ret = i2c_master_send(client, &cmd, sizeof(cmd));
	if (ret != sizeof(cmd))
		return ret;

	return i2c_master_recv(client, buf, len);
}

static int tsl2563_set_power(struct tsl2563_chip *chip, int on)
{
	struct i2c_client *client = chip->client;
	u8 cmd;

	cmd = on ? TSL2563_CMD_POWER_ON : TSL2563_CMD_POWER_OFF;
	return tsl2563_write(client, TSL2563_REG_CTRL, cmd);
}

/*
 * Return value is 0 for off, 1 for on, or a negative error
 * code if reading failed.
 */
static int tsl2563_get_power(struct tsl2563_chip *chip)
{
	struct i2c_client *client = chip->client;
	int ret;
	u8 val;

	ret = tsl2563_read(client, TSL2563_REG_CTRL, &val, sizeof(val));
	if (ret != sizeof(val))
		return ret;

	return (val & TSL2563_CTRL_POWER_MASK) == TSL2563_CMD_POWER_ON;
}

static int tsl2563_configure(struct tsl2563_chip *chip)
{
	struct i2c_client *client = chip->client;
	int ret;

	ret = tsl2563_write(client, TSL2563_REG_TIMING,
			chip->gainlevel->gaintime);
	if (ret)
		goto out;

	ret = tsl2563_write(client, TSL2563_REG_INT, chip->intr);

out:
	return ret;
}

static int tsl2563_detect(struct tsl2563_chip *chip)
{
	int ret;

	ret = tsl2563_set_power(chip, 1);
	if (ret)
		return ret;

	ret = tsl2563_get_power(chip);
	if (ret < 0)
		return ret;

	return ret ? 0 : -ENODEV;
}

static int tsl2563_read_id(struct tsl2563_chip *chip, u8 *id)
{
	struct i2c_client *client = chip->client;
	int ret;

	ret = tsl2563_read(client, TSL2563_REG_ID, id, sizeof(*id));
	if (ret != sizeof(*id))
		return ret;

	return 0;
}

/*
 * "Normalized" ADC value is one obtained with 400ms of integration time and
 * 16x gain. This function returns the number of bits of shift needed to
 * convert between normalized values and HW values obtained using given
 * timing and gain settings.
 */
static int adc_shiftbits(u8 timing)
{
	int shift = 0;

	switch (timing & TSL2563_TIMING_MASK) {
	case TSL2563_TIMING_13MS:
		shift += 5;
		break;
	case TSL2563_TIMING_100MS:
		shift += 2;
		break;
	case TSL2563_TIMING_400MS:
		/* no-op */
		break;
	}

	if (!(timing & TSL2563_TIMING_GAIN16))
		shift += 4;

	return shift;
}

/* Convert a HW ADC value to normalized scale. */
static u32 normalize_adc(u16 adc, u8 timing)
{
	return adc << adc_shiftbits(timing);
}

static void tsl2563_wait_adc(struct tsl2563_chip *chip)
{
	unsigned int delay;

	switch (chip->gainlevel->gaintime & TSL2563_TIMING_MASK) {
	case TSL2563_TIMING_13MS:
		delay = 14;
		break;
	case TSL2563_TIMING_100MS:
		delay = 101;
		break;
	default:
		delay = 402;
	}
	/*
	 * TODO: Make sure that we wait at least required delay but why we
	 * have to extend it one tick more?
	 */
	schedule_timeout_interruptible(msecs_to_jiffies(delay) + 2);
}

static int tsl2563_adjust_gainlevel(struct tsl2563_chip *chip, u16 adc)
{
	struct i2c_client *client = chip->client;

	if (adc > chip->gainlevel->max || adc < chip->gainlevel->min) {

		(adc > chip->gainlevel->max) ?
			chip->gainlevel++ : chip->gainlevel--;

		tsl2563_write(client, TSL2563_REG_TIMING,
			      chip->gainlevel->gaintime);

		tsl2563_wait_adc(chip);
		tsl2563_wait_adc(chip);

		return 1;
	} else
		return 0;
}

static int tsl2563_get_adc(struct tsl2563_chip *chip)
{
	struct i2c_client *client = chip->client;
	u8 buf0[2], buf1[2];
	u16 adc0, adc1;
	int retry = 1;
	int ret = 0;

	if (chip->state.event != PM_EVENT_ON)
		goto out;

	while (retry) {
		ret = tsl2563_read(client,
				   TSL2563_REG_DATA0LOW | TSL2563_CLEARINT,
				   buf0, sizeof(buf0));
		if (ret != sizeof(buf0))
			goto out;

		ret = tsl2563_read(client, TSL2563_REG_DATA1LOW,
				   buf1, sizeof(buf1));
		if (ret != sizeof(buf1))
			goto out;

		adc0 = (buf0[1] << 8) + buf0[0];
		adc1 = (buf1[1] << 8) + buf1[0];

		retry = tsl2563_adjust_gainlevel(chip, adc0);
	}

	chip->data0 = normalize_adc(adc0, chip->gainlevel->gaintime);
	chip->data1 = normalize_adc(adc1, chip->gainlevel->gaintime);

	ret = 0;
out:
	return ret;
}

static inline int calib_to_sysfs(u32 calib)
{
	return (int) (((calib * CALIB_BASE_SYSFS) +
		       CALIB_FRAC_HALF) >> CALIB_FRAC_BITS);
}

static inline u32 calib_from_sysfs(int value)
{
	return (((u32) value) << CALIB_FRAC_BITS) / CALIB_BASE_SYSFS;
}

/*
 * Conversions between lux and ADC values.
 *
 * The basic formula is lux = c0 * adc0 - c1 * adc1, where c0 and c1 are
 * appropriate constants. Different constants are needed for different
 * kinds of light, determined by the ratio adc1/adc0 (basically the ratio
 * of the intensities in infrared and visible wavelengths). lux_table below
 * lists the upper threshold of the adc1/adc0 ratio and the corresponding
 * constants.
 */

struct tsl2563_lux_coeff {
	unsigned long ch_ratio;
	unsigned long ch0_coeff;
	unsigned long ch1_coeff;
};

static const struct tsl2563_lux_coeff lux_table[] = {
	{
		.ch_ratio	= FRAC10K(1300),
		.ch0_coeff	= FRAC10K(315),
		.ch1_coeff	= FRAC10K(262),
	}, {
		.ch_ratio	= FRAC10K(2600),
		.ch0_coeff	= FRAC10K(337),
		.ch1_coeff	= FRAC10K(430),
	}, {
		.ch_ratio	= FRAC10K(3900),
		.ch0_coeff	= FRAC10K(363),
		.ch1_coeff	= FRAC10K(529),
	}, {
		.ch_ratio	= FRAC10K(5200),
		.ch0_coeff	= FRAC10K(392),
		.ch1_coeff	= FRAC10K(605),
	}, {
		.ch_ratio	= FRAC10K(6500),
		.ch0_coeff	= FRAC10K(229),
		.ch1_coeff	= FRAC10K(291),
	}, {
		.ch_ratio	= FRAC10K(8000),
		.ch0_coeff	= FRAC10K(157),
		.ch1_coeff	= FRAC10K(180),
	}, {
		.ch_ratio	= FRAC10K(13000),
		.ch0_coeff	= FRAC10K(34),
		.ch1_coeff	= FRAC10K(26),
	}, {
		.ch_ratio	= ULONG_MAX,
		.ch0_coeff	= 0,
		.ch1_coeff	= 0,
	},
};

/*
 * Convert normalized, scaled ADC values to lux.
 */
static unsigned int adc_to_lux(u32 adc0, u32 adc1)
{
	const struct tsl2563_lux_coeff *lp = lux_table;
	unsigned long ratio, lux, ch0 = adc0, ch1 = adc1;

	ratio = ch0 ? ((ch1 << ADC_FRAC_BITS) / ch0) : ULONG_MAX;

	while (lp->ch_ratio < ratio)
		lp++;

	lux = ch0 * lp->ch0_coeff - ch1 * lp->ch1_coeff;

	return (unsigned int) (lux >> ADC_FRAC_BITS);
}

/*--------------------------------------------------------------*/
/*                      Sysfs interface                         */
/*--------------------------------------------------------------*/

static ssize_t tsl2563_adc0_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct tsl2563_chip *chip = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&chip->lock);

	ret = tsl2563_get_adc(chip);
	if (ret)
		return ret;

	ret = snprintf(buf, PAGE_SIZE, "%d\n", chip->data0);
	mutex_unlock(&chip->lock);

	return ret;
}

static ssize_t tsl2563_adc1_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct tsl2563_chip *chip = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&chip->lock);

	ret = tsl2563_get_adc(chip);
	if (ret)
		return ret;

	ret = snprintf(buf, PAGE_SIZE, "%d\n", chip->data1);
	mutex_unlock(&chip->lock);

	return ret;
}

/* Apply calibration coefficient to ADC count. */
static u32 calib_adc(u32 adc, u32 calib)
{
	unsigned long scaled = adc;

	scaled *= calib;
	scaled >>= CALIB_FRAC_BITS;

	return (u32) scaled;
}

static ssize_t tsl2563_lux_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct tsl2563_chip *chip = dev_get_drvdata(dev);
	u32 calib0, calib1;
	int ret;

	mutex_lock(&chip->lock);

	ret = tsl2563_get_adc(chip);
	if (ret)
		goto out;

	calib0 = calib_adc(chip->data0, chip->calib0);
	calib1 = calib_adc(chip->data1, chip->calib1);

	ret = snprintf(buf, PAGE_SIZE, "%d\n", adc_to_lux(calib0, calib1));

out:
	mutex_unlock(&chip->lock);
	return ret;
}

static ssize_t format_calib(char *buf, int len, u32 calib)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", calib_to_sysfs(calib));
}

static ssize_t tsl2563_calib0_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct tsl2563_chip *chip = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&chip->lock);
	ret = format_calib(buf, PAGE_SIZE, chip->calib0);
	mutex_unlock(&chip->lock);
	return ret;
}

static ssize_t tsl2563_calib1_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct tsl2563_chip *chip = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&chip->lock);
	ret = format_calib(buf, PAGE_SIZE, chip->calib1);
	mutex_unlock(&chip->lock);
	return ret;
}

static int do_calib_store(struct device *dev, const char *buf, size_t len,
			  int ch)
{
	struct tsl2563_chip *chip = dev_get_drvdata(dev);
	int value;
	u32 calib;

	if (1 != sscanf(buf, "%d", &value))
		return -EINVAL;

	calib = calib_from_sysfs(value);

	if (ch)
		chip->calib1 = calib;
	else
		chip->calib0 = calib;

	return len;
}

static ssize_t tsl2563_calib0_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	return do_calib_store(dev, buf, len, 0);
}

static ssize_t tsl2563_calib1_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	return do_calib_store(dev, buf, len, 1);
}

static DEVICE_ATTR(adc0, S_IRUGO, tsl2563_adc0_show, NULL);
static DEVICE_ATTR(adc1, S_IRUGO, tsl2563_adc1_show, NULL);
static DEVICE_ATTR(lux, S_IRUGO, tsl2563_lux_show, NULL);
static DEVICE_ATTR(calib0, S_IRUGO | S_IWUSR,
		   tsl2563_calib0_show, tsl2563_calib0_store);
static DEVICE_ATTR(calib1, S_IRUGO | S_IWUSR,
		   tsl2563_calib1_show, tsl2563_calib1_store);

static struct attribute *tsl2563_attributes[] = {
	&dev_attr_adc0.attr,
	&dev_attr_adc1.attr,
	&dev_attr_lux.attr,
	&dev_attr_calib0.attr,
	&dev_attr_calib1.attr,
	NULL
};

static const struct attribute_group tsl2563_group = {
	.attrs = tsl2563_attributes,
};

static int tsl2563_register_sysfs(struct i2c_client *client)
{
	struct device *dev = &client->dev;

	return sysfs_create_group(&dev->kobj, &tsl2563_group);
}

static void tsl2563_unregister_sysfs(struct i2c_client *client)
{
	struct device *dev = &client->dev;

	sysfs_remove_group(&dev->kobj, &tsl2563_group);
}

/*--------------------------------------------------------------*/
/*                      Probe, Attach, Remove                   */
/*--------------------------------------------------------------*/
static struct i2c_driver tsl2563_i2c_driver;

static int tsl2563_probe(struct i2c_client *client,
		const struct i2c_device_id *device_id)
{
	struct tsl2563_chip *chip;
	int err = 0;
	u8 id;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	i2c_set_clientdata(client, chip);
	chip->client = client;

	err = tsl2563_detect(chip);
	if (err) {
		dev_err(&client->dev, "device not found, error %d \n", -err);
		goto fail1;
	}

	err = tsl2563_read_id(chip, &id);
	if (err)
		goto fail1;

	mutex_init(&chip->lock);

	/* Default values used until userspace says otherwise */
	chip->low_thres = 0x0;
	chip->high_thres = 0xffff;
	chip->gainlevel = tsl2563_gainlevel_table;
	chip->intr = TSL2563_INT_PERSIST(4);
	chip->calib0 = calib_from_sysfs(CALIB_BASE_SYSFS);
	chip->calib1 = calib_from_sysfs(CALIB_BASE_SYSFS);

	dev_info(&client->dev, "model %d, rev. %d\n", id >> 4, id & 0x0f);

	err = tsl2563_configure(chip);
	if (err)
		goto fail1;

	chip->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(chip->hwmon_dev))
		goto fail1;

	err = tsl2563_register_sysfs(client);
	if (err) {
		dev_err(&client->dev, "sysfs registration failed, %d\n", err);
		goto fail2;
	}

	return 0;
fail2:
	hwmon_device_unregister(chip->hwmon_dev);
fail1:
	kfree(chip);
	return err;
}

static int tsl2563_remove(struct i2c_client *client)
{
	struct tsl2563_chip *chip = i2c_get_clientdata(client);

	tsl2563_unregister_sysfs(client);
	hwmon_device_unregister(chip->hwmon_dev);

	kfree(chip);
	return 0;
}

static int tsl2563_suspend(struct i2c_client *client, pm_message_t state)
{
	struct tsl2563_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->lock);

	ret = tsl2563_set_power(chip, 0);
	if (ret)
		goto out;

	chip->state = state;

out:
	mutex_unlock(&chip->lock);
	return ret;
}

static int tsl2563_resume(struct i2c_client *client)
{
	struct tsl2563_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->lock);

	ret = tsl2563_set_power(chip, 1);
	if (ret)
		goto out;

	ret = tsl2563_configure(chip);
	if (ret)
		goto out;

	chip->state.event = PM_EVENT_ON;

out:
	mutex_unlock(&chip->lock);
	return ret;
}

static const struct i2c_device_id tsl2563_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, tsl2563_id);

static struct i2c_driver tsl2563_i2c_driver = {
	.driver = {
		.name	 = DRIVER_NAME,
	},
	.suspend	= tsl2563_suspend,
	.resume		= tsl2563_resume,
	.probe		= tsl2563_probe,
	.remove		= __devexit_p(tsl2563_remove),
	.id_table	= tsl2563_id,
};

static int __init tsl2563_init(void)
{
	return i2c_add_driver(&tsl2563_i2c_driver);
}

static void __exit tsl2563_exit(void)
{
	i2c_del_driver(&tsl2563_i2c_driver);
}

MODULE_AUTHOR("Nokia Corporation");
MODULE_DESCRIPTION("tsl2563 light sensor driver");
MODULE_LICENSE("GPL");

module_init(tsl2563_init);
module_exit(tsl2563_exit);
