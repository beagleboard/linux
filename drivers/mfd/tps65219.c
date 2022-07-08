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
	int ret;

	dev_dbg(tps->dev, "warm reset");
	ret =  regmap_update_bits(tps->regmap, TPS65219_REG_MFP_CTRL,
				  TPS65219_MFP_WARM_RESET_I2C_CTRL_MASK,
				  TPS65219_MFP_WARM_RESET_I2C_CTRL_MASK);
	return ret;
}

/**
 * tps65219_cold_reset: issue cold reset to SOC.
 *
 * @tps: Device to write to.
 */
static int tps65219_cold_reset(struct tps65219 *tps)
{
	int ret;

	dev_dbg(tps->dev, "cold reset");
	ret =  regmap_update_bits(tps->regmap, TPS65219_REG_MFP_CTRL,
				  TPS65219_MFP_COLD_RESET_I2C_CTRL_MASK,
				  TPS65219_MFP_COLD_RESET_I2C_CTRL_MASK);
	return ret;
}

/**
 * tps65219_soft_shutdown: issue cold reset to SOC.
 *
 * @tps: Device to write to.
 */
static int tps65219_soft_shutdown(struct tps65219 *tps)
{
	int ret;

	dev_dbg(tps->dev, "software shutdown");
	ret =  regmap_update_bits(tps->regmap, TPS65219_REG_MFP_CTRL,
				  TPS65219_MFP_I2C_OFF_REQ_MASK,
				  TPS65219_MFP_I2C_OFF_REQ_MASK);
	return ret;
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

static const struct regmap_range tps65219_volatile_ranges[] = {
	regmap_reg_range(TPS65219_REG_TI_DEV_ID, TPS65219_REG_FACTORY_CONFIG_2),
};

static const struct regmap_access_table tps65219_volatile_table = {
	.yes_ranges = tps65219_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(tps65219_volatile_ranges),
};

static const struct regmap_config tps65219_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.volatile_table = &tps65219_volatile_table,
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
	if (ret < 0) {
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

	/* If a pm_power_off function has already been added, leave it alone */
	if (pm_power_off) {
		dev_info(tps->dev,
			 "%s: pm_power_off function already registered\n",
			 __func__);
	} else {
		tps65219_i2c_client = client;
		pm_power_off = &pmic_do_poweroff;
	}
	return ret;
}

static int tps65219_remove(struct i2c_client *client)
{
	struct tps65219 *tps = i2c_get_clientdata(client);

	if (pm_power_off ==  &pmic_do_poweroff)
		pm_power_off = NULL;

	unregister_restart_handler(&tps->nb);

	return 0;
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
