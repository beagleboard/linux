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

static const struct resource tps65219_pwrbutton_resources[] = {
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_PB_FALLING_EDGE_DETECT, "falling"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_PB_RISING_EDGE_DETECT, "rising"),
};

static const struct resource tps65219_regulator_resources[] = {
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO3_SCG, "LDO3_SCG"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO3_OC, "LDO3_OC"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO3_UV, "LDO3_UV"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO4_SCG, "LDO4_SCG"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO4_OC, "LDO4_OC"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO4_UV, "LDO4_UV"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO1_SCG, "LDO1_SCG"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO1_OC, "LDO1_OC"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO1_UV, "LDO1_UV"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO2_SCG, "LDO2_SCG"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO2_OC, "LDO2_OC"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO2_UV, "LDO2_UV"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK3_SCG, "BUCK3_SCG"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK3_OC, "BUCK3_OC"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK3_NEG_OC, "BUCK3_NEG_OC"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK3_UV, "BUCK3_UV"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK1_SCG, "BUCK1_SCG"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK1_OC, "BUCK1_OC"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK1_NEG_OC, "BUCK1_NEG_OC"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK1_UV, "BUCK1_UV"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK2_SCG, "BUCK2_SCG"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK2_OC, "BUCK2_OC"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK2_NEG_OC, "BUCK2_NEG_OC"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK2_UV, "BUCK2_UV"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK1_RV, "BUCK1_RV"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK2_RV, "BUCK2_RV"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK3_RV, "BUCK3_RV"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO1_RV, "LDO1_RV"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO2_RV, "LDO2_RV"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO3_RV, "LDO3_RV"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO4_RV, "LDO4_RV"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK1_RV_SD, "BUCK1_RV_SD"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK2_RV_SD, "BUCK2_RV_SD"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_BUCK3_RV_SD, "BUCK3_RV_SD"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO1_RV_SD, "LDO1_RV_SD"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO2_RV_SD, "LDO2_RV_SD"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO3_RV_SD, "LDO3_RV_SD"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_LDO4_RV_SD, "LDO4_RV_SD"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_TIMEOUT, "TIMEOUT"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_SENSOR_3_WARM, "SENSOR_3_WARM"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_SENSOR_2_WARM, "SENSOR_2_WARM"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_SENSOR_1_WARM, "SENSOR_1_WARM"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_SENSOR_0_WARM, "SENSOR_0_WARM"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_SENSOR_3_HOT, "SENSOR_3_HOT"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_SENSOR_2_HOT, "SENSOR_2_HOT"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_SENSOR_1_HOT, "SENSOR_1_HOT"),
	DEFINE_RES_IRQ_NAMED(TPS65219_INT_SENSOR_0_HOT, "SENSOR_0_HOT"),
};

#define TPS65219_MAX_CELLS 2

static const struct mfd_cell tps65219_regulator_cell = {
	.name = "tps65219-regulator",
	.resources = tps65219_regulator_resources,
	.num_resources = ARRAY_SIZE(tps65219_regulator_resources),
};

static const struct mfd_cell tps65219_pwrbutton_cell = {
	.name = "tps65219-pwrbutton",
	.resources = tps65219_pwrbutton_resources,
	.num_resources = ARRAY_SIZE(tps65219_pwrbutton_resources),
};

static const struct regmap_config tps65219_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = TPS65219_REG_FACTORY_CONFIG_2,
};

/*
 * Mapping of main IRQ register bits to sub-IRQ register offsets so that we can
 * access corect sub-IRQ registers based on bits that are set in main IRQ
 * register.
 */
/* Timeout Residual Voltage Shutdown */
static unsigned int bit0_offsets[] = {TPS65219_TO_RV_POS};
static unsigned int bit1_offsets[] = {TPS65219_RV_POS};	/* Residual Voltage */
static unsigned int bit2_offsets[] = {TPS65219_SYS_POS};	/* System */
static unsigned int bit3_offsets[] = {TPS65219_BUCK_1_2_POS};	/* Buck 1-2 */
static unsigned int bit4_offsets[] = {TPS65219_BUCK_3_POS};	/* Buck 3 */
static unsigned int bit5_offsets[] = {TPS65219_LDO_1_2_POS};	/* LDO 1-2 */
static unsigned int bit6_offsets[] = {TPS65219_LDO_3_4_POS};	/* LDO 3-4 */
static unsigned int bit7_offsets[] = {TPS65219_PB_POS};	/* Power Button */

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
	REGMAP_IRQ_REG(TPS65219_INT_LDO3_SCG, TPS65219_LDO_3_4_POS,
		       TPS65219_INT_LDO3_SCG_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO3_OC,
		       TPS65219_LDO_3_4_POS, TPS65219_INT_LDO3_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO3_UV, TPS65219_LDO_3_4_POS,
		       TPS65219_INT_LDO3_UV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO4_SCG, TPS65219_LDO_3_4_POS,
		       TPS65219_INT_LDO4_SCG_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO4_OC, TPS65219_LDO_3_4_POS,
		       TPS65219_INT_LDO4_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO4_UV, TPS65219_LDO_3_4_POS,
		       TPS65219_INT_LDO4_UV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO1_SCG,
		       TPS65219_LDO_1_2_POS, TPS65219_INT_LDO1_SCG_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO1_OC, TPS65219_LDO_1_2_POS,
		       TPS65219_INT_LDO1_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO1_UV, TPS65219_LDO_1_2_POS,
		       TPS65219_INT_LDO1_UV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO2_SCG, TPS65219_LDO_1_2_POS,
		       TPS65219_INT_LDO2_SCG_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO2_OC, TPS65219_LDO_1_2_POS,
		       TPS65219_INT_LDO2_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO2_UV, TPS65219_LDO_1_2_POS,
		       TPS65219_INT_LDO2_UV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK3_SCG, TPS65219_BUCK_3_POS,
		       TPS65219_INT_BUCK3_SCG_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK3_OC, TPS65219_BUCK_3_POS,
		       TPS65219_INT_BUCK3_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK3_NEG_OC, TPS65219_BUCK_3_POS,
		       TPS65219_INT_BUCK3_NEG_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK3_UV, TPS65219_BUCK_3_POS,
		       TPS65219_INT_BUCK3_UV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK2_SCG, TPS65219_BUCK_1_2_POS,
		       TPS65219_INT_BUCK2_SCG_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK2_OC, TPS65219_BUCK_1_2_POS,
		       TPS65219_INT_BUCK2_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK2_NEG_OC, TPS65219_BUCK_1_2_POS,
		       TPS65219_INT_BUCK2_NEG_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK2_UV, TPS65219_BUCK_1_2_POS,
		       TPS65219_INT_BUCK2_UV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK1_SCG, TPS65219_BUCK_1_2_POS,
		       TPS65219_INT_BUCK1_SCG_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK1_OC, TPS65219_BUCK_1_2_POS,
		       TPS65219_INT_BUCK1_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK1_NEG_OC, TPS65219_BUCK_1_2_POS,
		       TPS65219_INT_BUCK1_NEG_OC_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK1_UV, TPS65219_BUCK_1_2_POS,
		       TPS65219_INT_BUCK1_UV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_3_WARM,
		       TPS65219_SYS_POS, TPS65219_INT_SENSOR_3_WARM_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_2_WARM, TPS65219_SYS_POS,
		       TPS65219_INT_SENSOR_2_WARM_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_1_WARM, TPS65219_SYS_POS,
		       TPS65219_INT_SENSOR_1_WARM_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_0_WARM, TPS65219_SYS_POS,
		       TPS65219_INT_SENSOR_0_WARM_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_3_HOT, TPS65219_SYS_POS,
		       TPS65219_INT_SENSOR_3_HOT_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_2_HOT, TPS65219_SYS_POS,
		       TPS65219_INT_SENSOR_2_HOT_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_1_HOT, TPS65219_SYS_POS,
		       TPS65219_INT_SENSOR_1_HOT_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_SENSOR_0_HOT, TPS65219_SYS_POS,
		       TPS65219_INT_SENSOR_0_HOT_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK1_RV, TPS65219_RV_POS,
		       TPS65219_INT_BUCK1_RV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK2_RV, TPS65219_RV_POS,
		       TPS65219_INT_BUCK2_RV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK3_RV, TPS65219_RV_POS,
		       TPS65219_INT_BUCK3_RV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO1_RV, TPS65219_RV_POS,
		       TPS65219_INT_LDO1_RV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO2_RV, TPS65219_RV_POS,
		       TPS65219_INT_LDO2_RV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO3_RV, TPS65219_RV_POS,
		       TPS65219_INT_LDO3_RV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO4_RV, TPS65219_RV_POS,
		       TPS65219_INT_LDO4_RV_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK1_RV_SD,
		       TPS65219_TO_RV_POS, TPS65219_INT_BUCK1_RV_SD_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK2_RV_SD,
		       TPS65219_TO_RV_POS, TPS65219_INT_BUCK2_RV_SD_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_BUCK3_RV_SD, TPS65219_TO_RV_POS,
		       TPS65219_INT_BUCK3_RV_SD_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO1_RV_SD, TPS65219_TO_RV_POS,
		       TPS65219_INT_LDO1_RV_SD_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO2_RV_SD, TPS65219_TO_RV_POS,
		       TPS65219_INT_LDO2_RV_SD_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO3_RV_SD,
		       TPS65219_TO_RV_POS, TPS65219_INT_LDO3_RV_SD_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_LDO4_RV_SD, TPS65219_TO_RV_POS,
		       TPS65219_INT_LDO4_RV_SD_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_TIMEOUT, TPS65219_TO_RV_POS,
		       TPS65219_INT_TIMEOUT_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_PB_FALLING_EDGE_DETECT,
		       TPS65219_PB_POS, TPS65219_INT_PB_FALLING_EDGE_DET_MASK),
	REGMAP_IRQ_REG(TPS65219_INT_PB_RISING_EDGE_DETECT, TPS65219_PB_POS,
		       TPS65219_INT_PB_RISING_EDGE_DET_MASK),
};

static struct regmap_irq_chip tps65219_irq_chip = {
	.name = "tps65219_irq",
	.main_status = TPS65219_REG_INT_SOURCE,
	.num_main_regs = 1,
	.num_main_status_bits = 8,
	.irqs = tps65219_irqs,
	.num_irqs = ARRAY_SIZE(tps65219_irqs),
	.status_base = TPS65219_REG_INT_LDO_3_4,
	.ack_base = TPS65219_REG_INT_LDO_3_4,
	.clear_ack = 1,
	.num_regs = 8,
	.sub_reg_offsets = &tps65219_sub_irq_offsets[0],
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
	bool pwr_button;
	bool sys_pwr;
	struct mfd_cell cells[TPS65219_MAX_CELLS];
	int nr_cells = 0;

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
	if (ret)
		return ret;

	ret = regmap_read(tps->regmap, TPS65219_REG_TI_DEV_ID, &chipid);
	if (ret) {
		dev_err(tps->dev, "Failed to read device ID: %d\n", ret);
		return ret;
	}

	memcpy(&cells[nr_cells++], &tps65219_regulator_cell,
	       sizeof(tps65219_regulator_cell));
	pwr_button = of_property_read_bool(tps->dev->of_node, "power-button");
	if (pwr_button)
		memcpy(&cells[nr_cells++], &tps65219_pwrbutton_cell,
		       sizeof(tps65219_pwrbutton_cell));

	ret = devm_mfd_add_devices(tps->dev, PLATFORM_DEVID_AUTO, cells,
				   nr_cells, NULL, 0,
				   regmap_irq_get_domain(tps->irq_data));
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
