// SPDX-License-Identifier: GPL-2.0
/*
 * tps65219-regulator.c
 *
 * Regulator driver for TPS65219 PMIC
 *
 * Copyright (C) 2022 BayLibre Incorporated - https://www.baylibre.com/
 */

/* This implementation derived from tps65218 authored by "J Keerthy <j-keerthy@ti.com>" */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/tps65219.h>

struct tps65219_regulator_irq_type {
	const char *irq_name;
	const char *regulator_name;
	const char *event_name;
};

struct tps65219_regulator_irq_type tps65219_regulator_irq_types[] = {
	{ "LDO3_SCG", "LDO3", "short circuit to ground" },
	{ "LDO3_OC", "LDO3", "overcurrent" },
	{ "LDO3_UV", "LDO3", "undervoltage" },
	{ "LDO4_SCG", "LDO4", "short circuit to ground" },
	{ "LDO4_OC", "LDO4", "overcurrent" },
	{ "LDO4_UV", "LDO4", "undervoltage" },
	{ "LDO1_SCG", "LDO1", "short circuit to ground" },
	{ "LDO1_OC", "LDO1", "overcurrent" },
	{ "LDO1_UV", "LDO1", "undervoltage" },
	{ "LDO2_SCG", "LDO2", "short circuit to ground" },
	{ "LDO2_OC", "LDO2", "overcurrent" },
	{ "LDO2_UV", "LDO2", "undervoltage" },
	{ "BUCK3_SCG", "BUCK3", "short circuit to ground" },
	{ "BUCK3_OC", "BUCK3", "overcurrent" },
	{ "BUCK3_NEG_OC", "BUCK3", "negative overcurrent" },
	{ "BUCK3_UV", "BUCK3", "undervoltage" },
	{ "BUCK1_SCG", "BUCK1", "short circuit to ground" },
	{ "BUCK1_OC", "BUCK1", "overcurrent" },
	{ "BUCK1_NEG_OC", "BUCK1", "negative overcurrent" },
	{ "BUCK1_UV", "BUCK1", "undervoltage" },
	{ "BUCK2_SCG", "BUCK2", "short circuit to ground" },
	{ "BUCK2_OC", "BUCK2", "overcurrent" },
	{ "BUCK2_NEG_OC", "BUCK2", "negative overcurrent" },
	{ "BUCK2_UV", "BUCK2", "undervoltage" },
	{ "BUCK1_RV", "BUCK1", "residual voltage" },
	{ "BUCK2_RV", "BUCK2", "residual voltage" },
	{ "BUCK3_RV", "BUCK3", "residual voltage" },
	{ "LDO1_RV", "LDO1", "residual voltage" },
	{ "LDO2_RV", "LDO2", "residual voltage" },
	{ "LDO3_RV", "LDO3", "residual voltage" },
	{ "LDO4_RV", "LDO4", "residual voltage" },
	{ "BUCK1_RV_SD", "BUCK1", "residual voltage on shutdown" },
	{ "BUCK2_RV_SD", "BUCK2", "residual voltage on shutdown" },
	{ "BUCK3_RV_SD", "BUCK3", "residual voltage on shutdown" },
	{ "LDO1_RV_SD", "LDO1", "residual voltage on shutdown" },
	{ "LDO2_RV_SD", "LDO2", "residual voltage on shutdown" },
	{ "LDO3_RV_SD", "LDO3", "residual voltage on shutdown" },
	{ "LDO4_RV_SD", "LDO4", "residual voltage on shutdown" },
	{ "SENSOR_3_WARM", "SENSOR3", "warm temperature" },
	{ "SENSOR_2_WARM", "SENSOR2", "warm temperature" },
	{ "SENSOR_1_WARM", "SENSOR1", "warm temperature" },
	{ "SENSOR_0_WARM", "SENSOR0", "warm temperature" },
	{ "SENSOR_3_HOT", "SENSOR3", "hot temperature" },
	{ "SENSOR_2_HOT", "SENSOR2", "hot temperature" },
	{ "SENSOR_1_HOT", "SENSOR1", "hot temperature" },
	{ "SENSOR_0_HOT", "SENSOR0", "hot temperature" },
	{ "TIMEOUT", "", "" },
};

struct tps65219_regulator_irq_data {
	struct device *dev;
	struct tps65219_regulator_irq_type *type;
};

#define TPS65219_REGULATOR(_name, _of, _id, _type, _ops, _n, _vr, _vm, _er, \
			   _em, _cr, _cm, _lr, _nlr, _delay, _fuv, \
			   _ct, _ncl, _bpm) \
	{								\
		.name			= _name,			\
		.of_match		= _of,				\
		.regulators_node	= of_match_ptr("regulators"),	\
		.supply_name		= _of,				\
		.id			= _id,				\
		.ops			= &(_ops),			\
		.n_voltages		= _n,				\
		.type			= _type,			\
		.owner			= THIS_MODULE,			\
		.vsel_reg		= _vr,				\
		.vsel_mask		= _vm,				\
		.csel_reg		= _cr,				\
		.csel_mask		= _cm,				\
		.curr_table		= _ct,				\
		.n_current_limits	= _ncl,				\
		.enable_reg		= _er,				\
		.enable_mask		= _em,				\
		.volt_table		= NULL,				\
		.linear_ranges		= _lr,				\
		.n_linear_ranges	= _nlr,				\
		.ramp_delay		= _delay,			\
		.fixed_uV		= _fuv,				\
		.bypass_reg		= _vr,				\
		.bypass_mask		= _bpm,				\
		.bypass_val_on		= 1,				\
	}								\

static const struct linear_range bucks_ranges[] = {
	REGULATOR_LINEAR_RANGE(600000, 0x0, 0x1f, 25000),
	REGULATOR_LINEAR_RANGE(1400000, 0x20, 0x33, 100000),
	REGULATOR_LINEAR_RANGE(3400000, 0x34, 0x3f, 0),
};

static const struct linear_range ldos_1_2_ranges[] = {
	REGULATOR_LINEAR_RANGE(600000, 0x0, 0x37, 50000),
	REGULATOR_LINEAR_RANGE(3400000, 0x38, 0x3f, 0),
};

static const struct linear_range ldos_3_4_ranges[] = {
	REGULATOR_LINEAR_RANGE(1200000, 0x0, 0xC, 0),
	REGULATOR_LINEAR_RANGE(1250000, 0xD, 0x35, 50000),
	REGULATOR_LINEAR_RANGE(3300000, 0x36, 0x3F, 0),
};

static int tps65219_pmic_set_voltage_sel(struct regulator_dev *dev,
					 unsigned int selector)
{
	int ret;
	struct tps65219 *tps = rdev_get_drvdata(dev);

	/* Set the voltage based on vsel value */
	ret = regmap_update_bits(tps->regmap, dev->desc->vsel_reg,
				 dev->desc->vsel_mask, selector);
	if (ret) {
		dev_dbg(tps->dev, "%s failed for regulator %s: %d ",
			__func__, dev->desc->name, ret);
	}
	return ret;
}

static int tps65219_pmic_enable(struct regulator_dev *dev)
{
	struct tps65219 *tps = rdev_get_drvdata(dev);

	return regmap_set_bits(tps->regmap, dev->desc->enable_reg,
			      dev->desc->enable_mask);
}

static int tps65219_pmic_disable(struct regulator_dev *dev)
{
	struct tps65219 *tps = rdev_get_drvdata(dev);

	return regmap_clear_bits(tps->regmap, dev->desc->enable_reg,
				 dev->desc->enable_mask);
}

static int tps65219_set_mode(struct regulator_dev *dev, unsigned int mode)
{
	struct tps65219 *tps = rdev_get_drvdata(dev);

	switch (mode) {
	case REGULATOR_MODE_NORMAL:
		return regmap_set_bits(tps->regmap, TPS65219_REG_STBY_1_CONFIG,
				       dev->desc->enable_mask);

	case REGULATOR_MODE_STANDBY:
		return regmap_clear_bits(tps->regmap,
					 TPS65219_REG_STBY_1_CONFIG,
					 dev->desc->enable_mask);
	}

	return -EINVAL;
}

static unsigned int tps65219_get_mode(struct regulator_dev *dev)
{
	struct tps65219 *tps = rdev_get_drvdata(dev);
	unsigned int rid = rdev_get_id(dev);
	int ret, value = 0;

	ret = regmap_read(tps->regmap, TPS65219_REG_STBY_1_CONFIG, &value);
	if (ret) {
		dev_dbg(tps->dev, "%s failed for regulator %s: %d ",
			__func__, dev->desc->name, ret);
		return ret;
	}
	value = (value & BIT(rid)) >> rid;
	if (value)
		return REGULATOR_MODE_STANDBY;
	else
		return REGULATOR_MODE_NORMAL;
}

/*
 * generic regulator_set_bypass_regmap does not fully match requirements
 * TPS65219 Requires explicitly that regulator is disabled before switch
 */
static int tps65219_set_bypass(struct regulator_dev *dev, bool enable)
{
	struct tps65219 *tps = rdev_get_drvdata(dev);
	unsigned int rid = rdev_get_id(dev);
	int ret = 0;

	if (dev->desc->ops->enable) {
		dev_err(tps->dev,
			"%s LDO%d enabled, must be shut down to set bypass ",
			__func__, rid);
		return -EBUSY;
	}
	ret =  regulator_set_bypass_regmap(dev, enable);
	return ret;
}

/* Operations permitted on BUCK1/2/3 */
static const struct regulator_ops tps65219_bucks_ops = {
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= tps65219_pmic_enable,
	.disable		= tps65219_pmic_disable,
	.set_mode		= tps65219_set_mode,
	.get_mode		= tps65219_get_mode,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= tps65219_pmic_set_voltage_sel,
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,

};

/* Operations permitted on LDO1/2 */
static const struct regulator_ops tps65219_ldos_1_2_ops = {
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= tps65219_pmic_enable,
	.disable		= tps65219_pmic_disable,
	.set_mode		= tps65219_set_mode,
	.get_mode		= tps65219_get_mode,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= tps65219_pmic_set_voltage_sel,
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.set_bypass		= tps65219_set_bypass,
	.get_bypass		= regulator_get_bypass_regmap,
};

/* Operations permitted on LDO3/4 */
static const struct regulator_ops tps65219_ldos_3_4_ops = {
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= tps65219_pmic_enable,
	.disable		= tps65219_pmic_disable,
	.set_mode		= tps65219_set_mode,
	.get_mode		= tps65219_get_mode,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= tps65219_pmic_set_voltage_sel,
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
};

static const struct regulator_desc regulators[] = {
	TPS65219_REGULATOR("BUCK1", "buck1", TPS65219_BUCK_1,
			   REGULATOR_VOLTAGE, tps65219_bucks_ops, 64,
			   TPS65219_REG_BUCK1_VOUT,
			   TPS65219_BUCKS_LDOS_VOUT_VSET_MASK,
			   TPS65219_REG_ENABLE_CTRL,
			   TPS65219_ENABLE_BUCK1_EN_MASK, 0, 0, bucks_ranges,
			   3, 4000, 0, NULL, 0, 0),
	TPS65219_REGULATOR("BUCK2", "buck2", TPS65219_BUCK_2,
			   REGULATOR_VOLTAGE, tps65219_bucks_ops, 64,
			   TPS65219_REG_BUCK2_VOUT,
			   TPS65219_BUCKS_LDOS_VOUT_VSET_MASK,
			   TPS65219_REG_ENABLE_CTRL,
			   TPS65219_ENABLE_BUCK2_EN_MASK, 0, 0, bucks_ranges,
			   3, 4000, 0, NULL, 0, 0),
	TPS65219_REGULATOR("BUCK3", "buck3", TPS65219_BUCK_3,
			   REGULATOR_VOLTAGE, tps65219_bucks_ops, 64,
			   TPS65219_REG_BUCK3_VOUT,
			   TPS65219_BUCKS_LDOS_VOUT_VSET_MASK,
			   TPS65219_REG_ENABLE_CTRL,
			   TPS65219_ENABLE_BUCK3_EN_MASK, 0, 0, bucks_ranges,
			   3, 0, 0, NULL, 0, 0),
	TPS65219_REGULATOR("LDO1", "ldo1", TPS65219_LDO_1,
			   REGULATOR_VOLTAGE, tps65219_ldos_1_2_ops, 64,
			   TPS65219_REG_LDO1_VOUT,
			   TPS65219_BUCKS_LDOS_VOUT_VSET_MASK,
			   TPS65219_REG_ENABLE_CTRL,
			   TPS65219_ENABLE_LDO1_EN_MASK, 0, 0, ldos_1_2_ranges,
			   2, 0, 0, NULL, 0, TPS65219_LDOS_BYP_CONFIG_MASK),
	TPS65219_REGULATOR("LDO2", "ldo2", TPS65219_LDO_2,
			   REGULATOR_VOLTAGE, tps65219_ldos_1_2_ops, 64,
			   TPS65219_REG_LDO2_VOUT,
			   TPS65219_BUCKS_LDOS_VOUT_VSET_MASK,
			   TPS65219_REG_ENABLE_CTRL,
			   TPS65219_ENABLE_LDO2_EN_MASK, 0, 0, ldos_1_2_ranges,
			   2, 0, 0, NULL, 0, TPS65219_LDOS_BYP_CONFIG_MASK),
	TPS65219_REGULATOR("LDO3", "ldo3", TPS65219_LDO_3,
			   REGULATOR_VOLTAGE, tps65219_ldos_3_4_ops, 64,
			   TPS65219_REG_LDO3_VOUT,
			   TPS65219_BUCKS_LDOS_VOUT_VSET_MASK,
			   TPS65219_REG_ENABLE_CTRL,
			   TPS65219_ENABLE_LDO3_EN_MASK, 0, 0, ldos_3_4_ranges,
			   3, 0, 0, NULL, 0, 0),
	TPS65219_REGULATOR("LDO4", "ldo4", TPS65219_LDO_4,
			   REGULATOR_VOLTAGE, tps65219_ldos_3_4_ops, 64,
			   TPS65219_REG_LDO4_VOUT,
			   TPS65219_BUCKS_LDOS_VOUT_VSET_MASK,
			   TPS65219_REG_ENABLE_CTRL,
			   TPS65219_ENABLE_LDO4_EN_MASK, 0, 0, ldos_3_4_ranges,
			   3, 0, 0, NULL, 0, 0),
};

static irqreturn_t tps65219_regulator_irq_handler(int irq, void *data)
{
	struct tps65219_regulator_irq_data *irq_data = data;

	if (irq_data->type->event_name[0] == '\0') {
		/* This is the timeout interrupt */
		dev_err(irq_data->dev, "System was put in shutdown during an active or standby transition.\n");
		return IRQ_HANDLED;
	}

	dev_err(irq_data->dev, "Registered %s for %s\n",
		irq_data->type->event_name, irq_data->type->regulator_name);
	return IRQ_HANDLED;
}

static int tps65219_regulator_probe(struct platform_device *pdev)
{
	struct tps65219 *tps = dev_get_drvdata(pdev->dev.parent);
	struct regulator_dev *rdev;
	struct regulator_config config = { };
	int i;
	int error;
	int irq;
	struct tps65219_regulator_irq_data *irq_data;
	struct tps65219_regulator_irq_type *irq_type;

	config.dev = tps->dev;
	config.driver_data = tps;
	config.regmap = tps->regmap;

	for (i = 0; i < ARRAY_SIZE(regulators); i++) {
		dev_dbg(tps->dev, "%s regul i= %d START", __func__, i);
		rdev = devm_regulator_register(&pdev->dev, &regulators[i],
					       &config);
		if (IS_ERR(rdev)) {
			dev_err(tps->dev, "failed to register %s regulator\n",
				pdev->name);
			return PTR_ERR(rdev);
		}

		dev_dbg(tps->dev, "%s regul i= %d COMPLETED", __func__, i);
	}

	irq_data = devm_kmalloc(tps->dev,
				ARRAY_SIZE(tps65219_regulator_irq_types) *
				sizeof(struct tps65219_regulator_irq_data),
				GFP_KERNEL);
	if (!irq_data)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(tps65219_regulator_irq_types); ++i) {
		irq_type = &tps65219_regulator_irq_types[i];

		irq = platform_get_irq_byname(pdev, irq_type->irq_name);
		if (irq < 0) {
			dev_err(tps->dev, "Failed to get IRQ %s: %d\n",
				irq_type->irq_name, irq);
			return -EINVAL;
		}
		irq_data[i].dev = tps->dev;
		irq_data[i].type = irq_type;

		error = devm_request_threaded_irq(tps->dev, irq, NULL,
						  tps65219_regulator_irq_handler,
						  IRQF_ONESHOT,
						  irq_type->irq_name,
						  &irq_data[i]);
		if (error) {
			dev_err(tps->dev, "failed to request %s IRQ %d: %d\n",
				irq_type->irq_name, irq, error);
			return error;
		}
	}

	return 0;
}

static const struct platform_device_id tps65219_regulator_id_table[] = {
	{ "tps65219-regulator", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, tps65219_regulator_id_table);

static struct platform_driver tps65219_regulator_driver = {
	.driver = {
		.name = "tps65219-pmic",
	},
	.probe = tps65219_regulator_probe,
	.id_table = tps65219_regulator_id_table,
};

module_platform_driver(tps65219_regulator_driver);

MODULE_AUTHOR("Jerome Neanne <j-neanne@baylibre.com>");
MODULE_DESCRIPTION("TPS65219 voltage regulator driver");
MODULE_ALIAS("platform:tps65219-pmic");
MODULE_LICENSE("GPL");
