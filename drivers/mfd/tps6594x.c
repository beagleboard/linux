// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for tps6594x PMIC chips
 *
 * Copyright (C) 2022 Texas Instruments Incorporated - https://www.ti.com/
 * Author: Keerthy <j-keerthy@ti.com>
 */

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mfd/core.h>
#include <linux/mfd/tps6594x.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>

static const struct regmap_config tps6594x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = TPS6594X_REG_MAX,
};

static const struct mfd_cell tps6594x_cells[] = {
	{ .name = "tps6594x-gpio" },
	{ .name = "tps6594x-rtc" },
};

static struct tps6594x *tps;

static void tps6594x_power_off(void)
{
	regmap_write(tps->regmap, TPS6594X_FSM_NSLEEP_TRIGGERS,
		TPS6594X_FSM_NSLEEP_NSLEEP1B | TPS6594X_FSM_NSLEEP_NSLEEP2B);

	regmap_write(tps->regmap, TPS6594X_INT_STARTUP,
		TPS6594X_INT_STARTUP_NPWRON_START_INT |
		TPS6594X_INT_STARTUP_ENABLE_INT | TPS6594X_INT_STARTUP_RTC_INT |
		TPS6594X_INT_STARTUP_SOFT_REBOOT_INT);

	regmap_write(tps->regmap, TPS6594X_INT_MISC,
		TPS6594X_INT_MISC_BIST_PASS_INT |
		TPS6594X_INT_MISC_EXT_CLK_INT | TPS6594X_INT_MISC_TWARN_INT);

	regmap_write(tps->regmap, TPS6594X_CONFIG_1,
		TPS6594X_CONFIG_NSLEEP1_MASK | TPS6594X_CONFIG_NSLEEP2_MASK);

	regmap_write(tps->regmap, TPS6594X_FSM_I2C_TRIGGERS,
		TPS6594X_FSM_I2C_TRIGGERS_I2C0);
}

static int tps6594x_probe(struct i2c_client *client)
{
	struct tps6594x *ddata;
	struct device_node *node = client->dev.of_node;
	unsigned int otpid;
	int ret;

	ddata = devm_kzalloc(&client->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->dev = &client->dev;

	ddata->regmap = devm_regmap_init_i2c(client, &tps6594x_regmap_config);
	if (IS_ERR(ddata->regmap)) {
		ret = PTR_ERR(ddata->regmap);
		dev_err(ddata->dev,
			"Failed to initialize register map: %d\n", ret);
		return ret;
	}

	ret = regmap_read(ddata->regmap, TPS6594X_REG_DEV_REV, &otpid);
	if (ret) {
		dev_err(ddata->dev, "Failed to read OTP ID\n");
		return ret;
	}

	ddata->rev = otpid;
	i2c_set_clientdata(client, ddata);

	ret = mfd_add_devices(ddata->dev, PLATFORM_DEVID_AUTO, tps6594x_cells,
			      ARRAY_SIZE(tps6594x_cells), NULL, 0, NULL);
	if (ret) {
		dev_err(ddata->dev, "Failed to register cells\n");
		return ret;
	}

	tps = ddata;

	if (of_property_read_bool(node, "ti,system-power-controller"))
		pm_power_off = tps6594x_power_off;

	return 0;
}

static const struct of_device_id of_tps6594x_match_table[] = {
	{ .compatible = "ti,tps6594x", },
	{}
};
MODULE_DEVICE_TABLE(of, of_tps6594x_match_table);

static const struct i2c_device_id tps6594x_id_table[] = {
	{ "tps6594x", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, tps6594x_id_table);

static struct i2c_driver tps6594x_driver = {
	.driver	= {
		.name	= "tps6594x",
		.of_match_table = of_tps6594x_match_table,
	},
	.probe_new	= tps6594x_probe,
	.id_table	= tps6594x_id_table,
};
module_i2c_driver(tps6594x_driver);

MODULE_AUTHOR("J Keerthy <j-keerthy@ti.com>");
MODULE_DESCRIPTION("TPS6594X PMIC device driver");
MODULE_LICENSE("GPL");
