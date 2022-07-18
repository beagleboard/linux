// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for TPS65219 Integrated power management chipsets
 *
 * Copyright (C) 2022 BayLibre Incorporated - https://www.baylibre.com/
 */

/* This implementation derived from tps65218 authored by
 * "J Keerthy <j-keerthy@ti.com>"
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>

#include <linux/mfd/core.h>
#include <linux/mfd/tps65219.h>

static struct i2c_client *tps65219_i2c_client;

/**
 * tps65219_warm_reset: issue warm reset to SOC.
 *
 * @tps: Device to write to.
 */
static int tps65219_warm_reset(struct tps65219 *tps)
{
	dev_dbg(tps->dev, "warm reset");
	return regmap_update_bits(tps->regmap, TPS65219_REG_MFP_CTRL,
				  TPS65219_MFP_WARM_RESET_I2C_CTRL_MASK,
				  TPS65219_MFP_WARM_RESET_I2C_CTRL_MASK);
}

/**
 * tps65219_cold_reset: issue cold reset to SOC.
 *
 * @tps: Device to write to.
 */
static int tps65219_cold_reset(struct tps65219 *tps)
{
	dev_dbg(tps->dev, "cold reset");
	return regmap_update_bits(tps->regmap, TPS65219_REG_MFP_CTRL,
				  TPS65219_MFP_COLD_RESET_I2C_CTRL_MASK,
				  TPS65219_MFP_COLD_RESET_I2C_CTRL_MASK);
}

/**
 * tps65219_soft_shutdown: issue cold reset to SOC.
 *
 * @tps: Device to write to.
 */
static int tps65219_soft_shutdown(struct tps65219 *tps)
{
	dev_dbg(tps->dev, "software shutdown");
	return regmap_update_bits(tps->regmap, TPS65219_REG_MFP_CTRL,
				  TPS65219_MFP_I2C_OFF_REQ_MASK,
				  TPS65219_MFP_I2C_OFF_REQ_MASK);
}

/**
 * pmic_rst_restart: trig tps65219 reset to SOC.
 *
 * Trigged via notifier
 */
static int pmic_rst_restart(struct notifier_block *this,
			    unsigned long reboot_mode, void *cmd)
{
	struct tps65219 *tps;

	tps = container_of(this, struct tps65219, nb);
	if (!tps) {
		pr_err("%s: pointer to tps65219 is invalid\n", __func__);
		return -ENODEV;
	}
	if (reboot_mode == REBOOT_WARM)
		tps65219_warm_reset(tps);
	else
		tps65219_cold_reset(tps);
	return NOTIFY_DONE;
}

static struct notifier_block pmic_rst_restart_nb = {
	.notifier_call = pmic_rst_restart,
	.priority = 200,
};

/**
 * pmic_do_poweroff: trig tps65219 regulators power OFF sequence.
 */
static void pmic_do_poweroff(void)
{
	struct tps65219 *tps;

	tps = dev_get_drvdata(&tps65219_i2c_client->dev);
	tps65219_soft_shutdown(tps);
}

static const struct mfd_cell tps65219_cells[] = {
	{ .name = "tps65219-regulator", },
};

static const struct regmap_config tps65219_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = TPS65219_REG_FACTORY_CONFIG_2,
};

static const struct of_device_id of_tps65219_match_table[] = {
	{ .compatible = "ti,tps65219", },
	{}
};
MODULE_DEVICE_TABLE(of, of_tps65219_match_table);

static int tps65219_probe(struct i2c_client *client,
			  const struct i2c_device_id *ids)
{
	struct tps65219 *tps;
	int ret;
	unsigned int chipid;
	bool sys_pwr;

	tps = devm_kzalloc(&client->dev, sizeof(*tps), GFP_KERNEL);
	if (!tps)
		return -ENOMEM;

	i2c_set_clientdata(client, tps);
	tps->dev = &client->dev;
	tps->regmap = devm_regmap_init_i2c(client, &tps65219_regmap_config);
	if (IS_ERR(tps->regmap)) {
		ret = PTR_ERR(tps->regmap);
		dev_err(tps->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	ret = regmap_read(tps->regmap, TPS65219_REG_TI_DEV_ID, &chipid);
	if (ret) {
		dev_err(tps->dev, "Failed to read device ID: %d\n", ret);
		return ret;
	}

	ret = devm_mfd_add_devices(tps->dev, PLATFORM_DEVID_AUTO, tps65219_cells,
				   ARRAY_SIZE(tps65219_cells), NULL, 0,
				   NULL);
	if (ret) {
		dev_err(tps->dev, "mfd_add_devices failed: %d\n", ret);
		return ret;
	}

	tps->nb = pmic_rst_restart_nb;
	ret = register_restart_handler(&tps->nb);
	if (ret) {
		dev_err(tps->dev, "%s: cannot register restart handler, %d\n",
			__func__, ret);
		return -ENODEV;
	}

	sys_pwr = of_property_read_bool(tps->dev->of_node,
					"system-power-controller");

	if (sys_pwr) {
		if (pm_power_off)
			dev_warn(tps->dev, "Setup as system-power-controller but pm_power_off function already registered, overwriting\n");
		tps65219_i2c_client = client;
		pm_power_off = &pmic_do_poweroff;
	}
	return ret;
}

static int tps65219_remove(struct i2c_client *client)
{
	struct tps65219 *tps = i2c_get_clientdata(client);

	if (tps65219_i2c_client == client) {
		pm_power_off = NULL;
		tps65219_i2c_client = NULL;
	}

	return unregister_restart_handler(&tps->nb);
}

static const struct i2c_device_id tps65219_id_table[] = {
	{ "tps65219", TPS65219 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, tps65219_id_table);

static struct i2c_driver tps65219_driver = {
	.driver		= {
		.name	= "tps65219",
		.of_match_table = of_tps65219_match_table,
	},
	.probe		= tps65219_probe,
	.id_table       = tps65219_id_table,
	.remove		= tps65219_remove,
};

module_i2c_driver(tps65219_driver);

MODULE_AUTHOR("Jerome Neanne <jneanne@baylibre.com>");
MODULE_DESCRIPTION("TPS65219 chip family multi-function driver");
MODULE_LICENSE("GPL");
