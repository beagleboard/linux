/* tmp468.c
 * Driver for the TI TMP468 temperature sensor
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>

/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x48, 0x49, 0x4a, 0x4b,
	I2C_CLIENT_END
};

enum chips {
	tmp464,
	tmp468,
};

/*
 * The TMP468 registers
 */

#define TMP468_CONFIG_REG			0x30
#define TMP468_TEMP_HYST_REG			0x38
#define TMP468_LOCK_REG				0xC4
#define TMP468_MANUFACTURER_ID_REG		0xFE
#define TMP468_DEVICE_ID_REG			0xFF

static const u8 TMP468_TEMP[4][9] = {
	{ 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 }, /* temp */
	{ 0x39, 0x42, 0x4a, 0x52, 0x5a, 0x62, 0x6a, 0x72, 0x7a }, /* high limit */
	{ 0x3a, 0x43, 0x4b, 0x53, 0x5b, 0x63, 0x6b, 0x73, 0x7b }, /* crit limit */
	{ 0xff, 0x40, 0x48, 0x50, 0x58, 0x60, 0x68, 0x70, 0x78 }, /* offset */
};

/* [0] = high, [1] = crit, [2] = fault */
static const u8 TMP468_STATUS[] = {
	0x21, 0x22, 0x23
};

/*
 * Constants: flags, masks, offsets, ids
 */

#define TMP468_CONFIG_RESERVE			BIT(0)
#define TMP468_CONFIG_SHUTDOWN 			BIT(5)
#define TMP468_CONFIG_ALL_CHANNELS 		GENMASK(15, 7)
#define TMP468_CONFIG_CONV_RATE_MASK 		GENMASK(4, 2)
#define TMP468_CONFIG_CONV_RATE_OFFSET 		2
#define TMP468_UNLOCK				0xEB19
#define TMP468_HYST_MASK			GENMASK(14, 7)

#define TMP468_MAX_TEMP				255000

/* Status bit for sensors */
static const u16 TMP468_STATUS_MASKS[] = {
	BIT(7), BIT(8), BIT(9), BIT(10), BIT(11),
	BIT(12), BIT(13), BIT(14), BIT(15)
};

/* Manufacturer and Device ID */
#define TMP468_MANUFACTURER_ID			0x5449
#define TMP464_DEVICE_ID			0x0464
#define TMP468_DEVICE_ID			0x0468

/*
 * Driver data (common to all clients)
 */

static const struct i2c_device_id tmp468_id[] = {
	{ "tmp464", tmp464 },
	{ "tmp468", tmp468 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tmp468_id);

static const struct of_device_id tmp468_of_match[] = {
	{
		.compatible = "ti,tmp464",
		.data = (void *)tmp464,
	},
	{
		.compatible = "ti,tmp468",
		.data = (void *)tmp468,
	},
	{},
};
MODULE_DEVICE_TABLE(of, tmp468_of_match);

/*
 * Client data
 */

struct tmp468_data {
	struct i2c_client *client;
	const struct attribute_group *groups[2];
	struct mutex update_lock;
	char valid; /* zero until following fields are valid */
	unsigned long last_updated; /* in jiffies */
	enum chips kind;

	unsigned int update_interval; /* in milliseconds */

	/* register values */
	u16 status[3];
	u16 config;
	u16 temp[4][9];
	u16 temp_hyst;
};

/*
 * Sysfs code
 */

static int tmp468_register_to_temp(u16 reg)
{
	int intVal = reg >> 7;
	int decVal = (reg >> 3) & 0xf;

	decVal = DIV_ROUND_CLOSEST(decVal * 135, 2);
	intVal = (intVal & ~(1 << 8)) - (intVal & (1 << 8));
	intVal *= 1000;

	return intVal + decVal;
}

static u16 tmp468_temp_to_register(long temp)
{
	temp = clamp_val(temp, 0, TMP468_MAX_TEMP);

	return DIV_ROUND_CLOSEST(temp, 1000) << 7;
}

static int tmp468_update_device_reg16(struct i2c_client *client,
				      struct tmp468_data *data)
{
	int i;
	int j;
	int val;
	int num_regs = 4;
	int num_sensors = 9;

	for (i = 0; i < num_regs; i++) {		/* temp / high / ... */
		for (j = 0; j < num_sensors; j++) {	/* local / r1 / r2 */
			u8 regaddr;
			regaddr = TMP468_TEMP[i][j];
			if (regaddr == 0xff)
				continue;
			val = i2c_smbus_read_word_swapped(client, regaddr);
			if (val < 0)
				return val;

			data->temp[i][j] = val;
		}
	}
	return 0;
}

static struct tmp468_data *tmp468_update_device(struct device *dev)
{
	struct tmp468_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	struct tmp468_data *ret = data;
	int i;
	int val;
	unsigned long next_update;

	mutex_lock(&data->update_lock);

	next_update = data->last_updated +
		      msecs_to_jiffies(data->update_interval);
	if (time_after(jiffies, next_update) || !data->valid) {
		for (i = 0; i < ARRAY_SIZE(data->status); i++) {
			val = i2c_smbus_read_word_swapped(client, TMP468_STATUS[i]);
			if (val < 0) {
				ret = ERR_PTR(val);
				goto abort;
			}
			data->status[i] = val;
		}
		val = i2c_smbus_read_word_swapped(client, TMP468_CONFIG_REG);
		if (val < 0) {
			ret = ERR_PTR(val);
			goto abort;
		}
		data->config = val;
		val = tmp468_update_device_reg16(client, data);
		if (val < 0) {
			ret = ERR_PTR(val);
			goto abort;
		}
		val = i2c_smbus_read_word_swapped(client, TMP468_TEMP_HYST_REG);
		if (val < 0) {
			ret = ERR_PTR(val);
			goto abort;
		}
		data->temp_hyst = val;

		data->last_updated = jiffies;
		data->valid = 1;
	}

abort:
	mutex_unlock(&data->update_lock);
	return ret;
}

static ssize_t show_temp(struct device *dev,
			 struct device_attribute *devattr, char *buf)
{
	int nr = to_sensor_dev_attr_2(devattr)->nr;
	int index = to_sensor_dev_attr_2(devattr)->index;
	struct tmp468_data *data = tmp468_update_device(dev);

	if (IS_ERR(data))
		return PTR_ERR(data);

	return sprintf(buf, "%d\n", tmp468_register_to_temp(data->temp[nr][index]));
}

static ssize_t show_temp_hyst(struct device *dev,
			      struct device_attribute *devattr, char *buf)
{
	int nr = to_sensor_dev_attr_2(devattr)->nr;
	int index = to_sensor_dev_attr_2(devattr)->index;
	struct tmp468_data *data = tmp468_update_device(dev);
	int temp;

	if (IS_ERR(data))
		return PTR_ERR(data);

	mutex_lock(&data->update_lock);
	temp = tmp468_register_to_temp(data->temp[nr][index]);
	temp -= tmp468_register_to_temp(data->temp_hyst);
	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", temp);
}

static ssize_t show_status(struct device *dev,
			   struct device_attribute *devattr, char *buf)
{
	int nr = to_sensor_dev_attr_2(devattr)->nr;
	int mask = TMP468_STATUS_MASKS[to_sensor_dev_attr_2(devattr)->index];
	struct tmp468_data *data = tmp468_update_device(dev);

	if (IS_ERR(data))
		return PTR_ERR(data);

	return sprintf(buf, "%d\n", !!(data->status[nr] & mask));
}

static ssize_t store_temp(struct device *dev, struct device_attribute *devattr,
			  const char *buf, size_t count)
{
	int nr = to_sensor_dev_attr_2(devattr)->nr;
	int index = to_sensor_dev_attr_2(devattr)->index;
	struct tmp468_data *data = dev_get_drvdata(dev);
	long val;
	int err;
	u16 reg;
	u8 regaddr;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	reg = tmp468_temp_to_register(val);

	mutex_lock(&data->update_lock);
	regaddr = TMP468_TEMP[nr][index];

	err = i2c_smbus_write_word_swapped(data->client, regaddr, reg);
	if (!err)
		data->temp[nr][index] = reg;
	mutex_unlock(&data->update_lock);

	return err ? err : count;
}

static ssize_t store_temp_hyst(struct device *dev,
			       struct device_attribute *devattr,
			       const char *buf, size_t count)
{
	int nr = to_sensor_dev_attr_2(devattr)->nr;
	int index = to_sensor_dev_attr_2(devattr)->index;
	struct tmp468_data *data = tmp468_update_device(dev);
	int temp;
	long val;
	int err;
	u16 reg;

	if (IS_ERR(data))
		return PTR_ERR(data);

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	val = clamp_val(val, 0, TMP468_MAX_TEMP);

	mutex_lock(&data->update_lock);
	temp = tmp468_register_to_temp(data->temp[nr][index]);
	val = clamp_val(val, temp - TMP468_MAX_TEMP, temp);

	reg = tmp468_temp_to_register(temp - val);
	reg &= TMP468_HYST_MASK;

	err = i2c_smbus_write_word_swapped(data->client, TMP468_TEMP_HYST_REG, reg);

	if (!err)
		data->temp_hyst = reg;

	mutex_unlock(&data->update_lock);

	return err ? err : count;
}

#define DECLARE_SENSOR_ATTRS(x)							\
static SENSOR_DEVICE_ATTR_2(temp##x##_input, S_IRUGO, show_temp, NULL, 0, x-1);	\
static SENSOR_DEVICE_ATTR_2(temp##x##_max, S_IWUSR | S_IRUGO, show_temp,	\
			    store_temp, 1, x-1);				\
static SENSOR_DEVICE_ATTR_2(temp##x##_crit, S_IWUSR | S_IRUGO, show_temp,	\
			    store_temp, 2, x-1);				\
static SENSOR_DEVICE_ATTR_2(temp##x##_max_alarm, S_IRUGO, show_status, NULL,	\
			    0, x-1);						\
static SENSOR_DEVICE_ATTR_2(temp##x##_crit_alarm, S_IRUGO, show_status, NULL,	\
			    1, x-1);						\
static SENSOR_DEVICE_ATTR_2(temp##x##_max_hyst, S_IWUSR | S_IRUGO,		\
			    show_temp_hyst, store_temp_hyst, 1, x-1);		\
static SENSOR_DEVICE_ATTR_2(temp##x##_crit_hyst, S_IWUSR | S_IRUGO,		\
			    show_temp_hyst, store_temp_hyst, 2, x-1)

#define DECLARE_REMOTE_SENSOR_ATTRS(x) 						\
DECLARE_SENSOR_ATTRS(x);							\
static SENSOR_DEVICE_ATTR_2(temp##x##_offset, S_IWUSR | S_IRUGO, show_temp,	\
			    store_temp, 3, x-1);				\
static SENSOR_DEVICE_ATTR_2(temp##x##_fault, S_IRUGO, show_status, NULL,	\
			    2, x-1)

DECLARE_SENSOR_ATTRS(1);
DECLARE_REMOTE_SENSOR_ATTRS(2);
DECLARE_REMOTE_SENSOR_ATTRS(3);
DECLARE_REMOTE_SENSOR_ATTRS(4);
DECLARE_REMOTE_SENSOR_ATTRS(5);
DECLARE_REMOTE_SENSOR_ATTRS(6);
DECLARE_REMOTE_SENSOR_ATTRS(7);
DECLARE_REMOTE_SENSOR_ATTRS(8);
DECLARE_REMOTE_SENSOR_ATTRS(9);

#define SENSOR_ATTRS(x)						\
	&sensor_dev_attr_temp##x##_input.dev_attr.attr,		\
	&sensor_dev_attr_temp##x##_max.dev_attr.attr,		\
	&sensor_dev_attr_temp##x##_crit.dev_attr.attr,		\
	&sensor_dev_attr_temp##x##_max_alarm.dev_attr.attr,	\
	&sensor_dev_attr_temp##x##_crit_alarm.dev_attr.attr,	\
	&sensor_dev_attr_temp##x##_max_hyst.dev_attr.attr,	\
	&sensor_dev_attr_temp##x##_crit_hyst.dev_attr.attr

#define REMOTE_SENSOR_ATTRS(x) 					\
	SENSOR_ATTRS(x),					\
	&sensor_dev_attr_temp##x##_offset.dev_attr.attr,	\
	&sensor_dev_attr_temp##x##_fault.dev_attr.attr

static struct attribute *tmp468_attributes[] = {
	SENSOR_ATTRS(1),
	REMOTE_SENSOR_ATTRS(2),
	REMOTE_SENSOR_ATTRS(3),
	REMOTE_SENSOR_ATTRS(4),
	REMOTE_SENSOR_ATTRS(5),
	REMOTE_SENSOR_ATTRS(6),
	REMOTE_SENSOR_ATTRS(7),
	REMOTE_SENSOR_ATTRS(8),
	REMOTE_SENSOR_ATTRS(9),

	NULL
};

static const struct attribute_group tmp468_group = {
	.attrs = tmp468_attributes,
};


/*
 * Driver init code
 */

static int tmp468_init_client(struct tmp468_data *data,
			      struct i2c_client *client)
{
	int config;
	int config_orig;
	int ret = 0;

	/* Read config */
	config = i2c_smbus_read_word_swapped(client, TMP468_CONFIG_REG);
	if (config < 0)
		return config;

	/* Unlock write */
	ret = i2c_smbus_write_word_swapped(client, TMP468_LOCK_REG, TMP468_UNLOCK);
	if (ret)
		return ret;

	config_orig = config;

	/* Disable shutdown */
	config &= ~TMP468_CONFIG_SHUTDOWN;
	/* Enable all channals */
	config |= TMP468_CONFIG_ALL_CHANNELS;
	/* Set conv rate to 2 hz */
	config &= ~TMP468_CONFIG_CONV_RATE_MASK;
	config |= 0x101 << TMP468_CONFIG_CONV_RATE_OFFSET;

	if (config != config_orig)
		ret = i2c_smbus_write_word_swapped(client, TMP468_CONFIG_REG,
						   config);
	if (!ret)
		data->update_interval = 500;
	return ret;
}

static int tmp468_detect(struct i2c_client *client,
			 struct i2c_board_info *info)
{
	enum chips kind;
	struct i2c_adapter *adapter = client->adapter;
	u16 reg;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	/* Detect and identify the chip */
	reg = i2c_smbus_read_word_swapped(client, TMP468_MANUFACTURER_ID_REG);
	if (reg != TMP468_MANUFACTURER_ID)
		return -ENODEV;

	reg = i2c_smbus_read_word_swapped(client, TMP468_DEVICE_ID_REG);

	switch (reg) {
	case TMP468_DEVICE_ID:
		kind = tmp468;
		break;
	case TMP464_DEVICE_ID:
		kind = tmp464;
		break;
	default:
		return -ENODEV;
	}

	strlcpy(info->type, tmp468_id[kind].name, I2C_NAME_SIZE);

	return 0;
}

static int tmp468_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	static const char * const names[] = {
		"TMP468"
	};
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct tmp468_data *data;
	int groups = 0;
	int status;

	data = devm_kzalloc(dev, sizeof(struct tmp468_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	mutex_init(&data->update_lock);
	data->kind = id->driver_data;

	/* Initialize the TMP468 chip */
	status = tmp468_init_client(data, client);
	if (status < 0)
		return status;

	/* Register sysfs hooks */
	data->groups[groups++] = &tmp468_group;

	hwmon_dev = devm_hwmon_device_register_with_groups(dev, client->name,
							   data, data->groups);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	dev_info(dev, "Detected TI %s chip\n", names[data->kind]);

	return 0;
}

static struct i2c_driver tmp468_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "tmp468",
		.of_match_table = of_match_ptr(tmp468_of_match),
	},
	.probe		= tmp468_probe,
	.id_table	= tmp468_id,
	.detect		= tmp468_detect,
	.address_list	= normal_i2c,
};

module_i2c_driver(tmp468_driver);

MODULE_AUTHOR("Arista Networks");
MODULE_DESCRIPTION("TI TMP468 temperature sensor driver");
MODULE_LICENSE("GPL");
