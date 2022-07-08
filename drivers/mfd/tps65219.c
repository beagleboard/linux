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
	{
		.name = "tps65219-pwrbutton",
		.of_compatible = "ti,tps65219-pwrbutton"
	},
};

static const struct regmap_range tps65219_volatile_ranges[] = {
	regmap_reg_range(TPS65219_REG_TI_DEV_ID,
			 TPS65219_REG_FACTORY_CONFIG_2),
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

/*
 * Mapping of main IRQ register bits to sub-IRQ register offsets so that we can
 * access corect sub-IRQ registers based on bits that are set in main IRQ
 * register.
 */
/* Timeout Residual Voltage Shutdown */
static unsigned int bit0_offsets[] = {TO_RV_POS};
static unsigned int bit1_offsets[] = {RV_POS};	/* Residual Voltage */
static unsigned int bit2_offsets[] = {SYS_POS};	/* System */
static unsigned int bit3_offsets[] = {BUCK_1_2_POS};	/* Buck 1-2 */
static unsigned int bit4_offsets[] = {BUCK_3_POS};	/* Buck 3 */
static unsigned int bit5_offsets[] = {LDO_1_2_POS};	/* LDO 1-2 */
static unsigned int bit6_offsets[] = {LDO_3_4_POS};	/* LDO 3-4 */
static unsigned int bit7_offsets[] = {PB_POS};	/* Power Button */

static struct regmap_irq_sub_irq_map tps65219_sub_irq_offsets[] = {
	REGMAP_IRQ_MAIN_REG_OFFSET(bit0_offsets),
	REGMAP_IRQ_MAIN_REG_OFFSET(bit1_offsets),
	REGMAP_IRQ_MAIN_REG_OFFSET(bit2_offsets),
	REGMAP_IRQ_MAIN_REG_OFFSET(bit3_offsets),
	REGMAP_IRQ_MAIN_REG_OFFSET(bit4_offsets),
	REGMAP_IRQ_MAIN_REG_OFFSET(bit5_offsets),
	REGMAP_IRQ_MAIN_REG_OFFSET(bit6_offsets),
	REGMAP_IRQ_MAIN_REG_OFFSET(bit7_offsets),
};

static struct regmap_irq tps65219_irqs[] = {
	REGMAP_IRQ_REG(TPS65219_INT_LDO3_SCG, LDO_3_4_POS,
		       TPS65219_INT_LDO3_SCG_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO3_OC,
		       LDO_3_4_POS, TPS65219_INT_LDO3_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO3_UV, LDO_3_4_POS,
		       TPS65219_INT_LDO3_UV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO4_SCG, LDO_3_4_POS,
		       TPS65219_INT_LDO4_SCG_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO4_OC, LDO_3_4_POS,
		       TPS65219_INT_LDO4_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO4_UV, LDO_3_4_POS,
		       TPS65219_INT_LDO4_UV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO1_SCG,
		       LDO_1_2_POS, TPS65219_INT_LDO1_SCG_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO1_OC, LDO_1_2_POS,
		       TPS65219_INT_LDO1_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO1_UV, LDO_1_2_POS,
		       TPS65219_INT_LDO1_UV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO2_SCG, LDO_1_2_POS,
		       TPS65219_INT_LDO2_SCG_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO2_OC, LDO_1_2_POS,
		       TPS65219_INT_LDO2_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO2_UV, LDO_1_2_POS,
		       TPS65219_INT_LDO2_UV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK3_SCG, BUCK_3_POS,
		       TPS65219_INT_BUCK3_SCG_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK3_OC, BUCK_3_POS,
		       TPS65219_INT_BUCK3_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK3_NEG_OC, BUCK_3_POS,
		       TPS65219_INT_BUCK3_NEG_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK3_UV, BUCK_3_POS,
		       TPS65219_INT_BUCK3_UV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK2_SCG, BUCK_1_2_POS,
		       TPS65219_INT_BUCK2_SCG_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK2_OC, BUCK_1_2_POS,
		       TPS65219_INT_BUCK2_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK2_NEG_OC, BUCK_1_2_POS,
		       TPS65219_INT_BUCK2_NEG_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK2_UV, BUCK_1_2_POS,
		       TPS65219_INT_BUCK2_UV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK1_SCG, BUCK_1_2_POS,
		       TPS65219_INT_BUCK1_SCG_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK1_OC, BUCK_1_2_POS,
		       TPS65219_INT_BUCK1_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK1_NEG_OC, BUCK_1_2_POS,
		       TPS65219_INT_BUCK1_NEG_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK1_UV, BUCK_1_2_POS,
		       TPS65219_INT_BUCK1_UV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_3_WARM,
		       SYS_POS, TPS65219_INT_SENSOR_3_WARM_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_2_WARM, SYS_POS,
		       TPS65219_INT_SENSOR_2_WARM_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_1_WARM, SYS_POS,
		       TPS65219_INT_SENSOR_1_WARM_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_0_WARM, SYS_POS,
		       TPS65219_INT_SENSOR_0_WARM_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_3_HOT, SYS_POS,
		       TPS65219_INT_SENSOR_3_HOT_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_2_HOT, SYS_POS,
		       TPS65219_INT_SENSOR_2_HOT_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_1_HOT, SYS_POS,
		       TPS65219_INT_SENSOR_1_HOT_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_0_HOT, SYS_POS,
		       TPS65219_INT_SENSOR_0_HOT_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK1_RV, RV_POS,
		       TPS65219_INT_BUCK1_RV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK2_RV, RV_POS,
		       TPS65219_INT_BUCK2_RV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK3_RV, RV_POS,
		       TPS65219_INT_BUCK3_RV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO1_RV, RV_POS,
		       TPS65219_INT_LDO1_RV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO2_RV, RV_POS,
		       TPS65219_INT_LDO2_RV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO3_RV, RV_POS,
		       TPS65219_INT_LDO3_RV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO4_RV, RV_POS,
		       TPS65219_INT_LDO4_RV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK1_RV_SD,
		       TO_RV_POS, TPS65219_INT_BUCK1_RV_SD_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK2_RV_SD,
		       TO_RV_POS, TPS65219_INT_BUCK2_RV_SD_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK3_RV_SD, TO_RV_POS,
		       TPS65219_INT_BUCK3_RV_SD_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO1_RV_SD, TO_RV_POS,
		       TPS65219_INT_LDO1_RV_SD_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO2_RV_SD, TO_RV_POS,
		       TPS65219_INT_LDO2_RV_SD_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO3_RV_SD,
		       TO_RV_POS, TPS65219_INT_LDO3_RV_SD_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO4_RV_SD, TO_RV_POS,
		       TPS65219_INT_LDO4_RV_SD_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_TIMEOUT, TO_RV_POS,
		       TPS65219_INT_TIMEOUT_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_PB_FALLING_EDGE_DETECT,
		       PB_POS, TPS65219_INT_PB_FALLING_EDGE_DET_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_PB_RISING_EDGE_DETECT, PB_POS,
		       TPS65219_INT_PB_RISING_EDGE_DET_MASK),
};

static struct regmap_irq_chip tps65219_irq_chip = {
	.name = "tps65219_irq",
	.main_status = TPS65219_REG_INT_SOURCE,
	.irqs = &tps65219_irqs[0],
	.num_irqs = ARRAY_SIZE(tps65219_irqs),
	.status_base = TPS65219_REG_INT_LDO_3_4,
	.ack_base = TPS65219_REG_INT_LDO_3_4,
	.clear_ack = 1,
	.num_regs = 8,
	.num_main_regs = 1,
	.sub_reg_offsets = &tps65219_sub_irq_offsets[0],
	.num_main_status_bits = 8,
	.irq_reg_stride = 1,
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
	tps->irq = client->irq;
	tps->regmap = devm_regmap_init_i2c(client, &tps65219_regmap_config);
	if (IS_ERR(tps->regmap)) {
		ret = PTR_ERR(tps->regmap);
		dev_err(tps->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	ret = devm_regmap_add_irq_chip(&client->dev, tps->regmap, tps->irq,
				       IRQF_ONESHOT, 0, &tps65219_irq_chip,
				       &tps->irq_data);
	if (ret < 0)
		return ret;

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
