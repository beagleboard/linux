// SPDX-License-Identifier: GPL-2.0
/*
 * simple driver for PWM (Pulse Width Modulator) controller
 *
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#define MAX_PWM_NUM	6

#define LIGHT_PWM_CHN_BASE(n)		((n) * 0x20)
#define LIGHT_PWM_CTRL(n)		(LIGHT_PWM_CHN_BASE(n) + 0x00)
#define LIGHT_PWM_RPT(n)		(LIGHT_PWM_CHN_BASE(n) + 0x04)
#define LIGHT_PWM_PER(n)		(LIGHT_PWM_CHN_BASE(n) + 0x08)
#define LIGHT_PWM_FP(n)			(LIGHT_PWM_CHN_BASE(n) + 0x0c)
#define LIGHT_PWM_STATUS(n)		(LIGHT_PWM_CHN_BASE(n) + 0x10)

/* bit definition PWM_CTRL */
#define PWM_START				BIT(0)
#define PWM_SOFT_RST				BIT(1)
#define PWM_CFG_UPDATE				BIT(2)
#define PWM_INT_EN				BIT(3)
#define PWM_ONE_SHOT_MODE			BIT(4)
#define PWM_CONTINUOUS_MODE			BIT(5)
#define PWM_EVT_RISING_TRIG_UNDER_ONE_SHOT	BIT(6)
#define PWM_EVT_FALLING_TRIG_UNDER_ONE_SHOT	BIT(7)
#define PWM_FPOUT				BIT(8)
#define PWM_INFACTOUT				BIT(9)




struct pwm_light_chip {
	struct clk *pwm_pclk;
	struct clk *pwm_cclk;
	void __iomem *mmio_base;
	struct pwm_chip chip;
};

#define to_pwm_light_chip(chip)		container_of(chip, struct pwm_light_chip, chip)

static int pwm_light_clk_prepare_enable(struct pwm_chip *chip)
{
	struct pwm_light_chip *plc = to_pwm_light_chip(chip);
	int ret;

	ret = clk_prepare_enable(plc->pwm_pclk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(plc->pwm_cclk);
	if (ret) {
		clk_disable_unprepare(plc->pwm_pclk);
		return ret;
	}

	return 0;
}

static void pwm_light_clk_disable_unprepare(struct pwm_chip *chip)
{
	struct pwm_light_chip *plc = to_pwm_light_chip(chip);

	clk_disable_unprepare(plc->pwm_pclk);
	clk_disable_unprepare(plc->pwm_cclk);
}

static int pwm_light_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pwm_light_chip *plc = to_pwm_light_chip(chip);
	u32 value;
	int ret;

	ret = pm_runtime_get_sync(chip->dev);
	if (ret < 0) {
		dev_err(chip->dev, "failed to clock on the pwm device(%d)\n", ret);
		pm_runtime_put_noidle(chip->dev);
		return ret;
	}

	value = readl(plc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));
	value |= PWM_START;
	writel(value, plc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));

	return 0;
}

static void pwm_light_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pwm_light_chip *plc = to_pwm_light_chip(chip);
	u32 value;
	int ret;

	ret = pm_runtime_get_sync(chip->dev);
	if (ret < 0) {
		dev_err(chip->dev, "failed to clock on the pwm device(%d)\n", ret);
		pm_runtime_put_noidle(chip->dev);
		return;
	}

	value = readl(plc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));
	value &= ~PWM_START;
	writel(value, plc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));

	pm_runtime_put_sync(chip->dev);
}

static int pwm_light_config(struct pwm_chip *chip, struct pwm_device *pwm, int duty_ns, int period_ns)
{
	struct pwm_light_chip *plc = to_pwm_light_chip(chip);
	unsigned long rate = clk_get_rate(plc->pwm_cclk);
	unsigned long duty_cycle, period_cycle;
	u32 pwm_cfg = PWM_INFACTOUT | PWM_FPOUT | PWM_CONTINUOUS_MODE | PWM_INT_EN;
	int ret;

	if (duty_ns > period_ns) {
		dev_err(chip->dev, "invalid pwm configure\n");
		return -EINVAL;
	}

	ret = pm_runtime_get_sync(chip->dev);
	if (ret < 0) {
		dev_err(chip->dev, "failed to clock on the pwm device(%d)\n", ret);
		pm_runtime_put_noidle(chip->dev);
		return ret;
	}

	writel(pwm_cfg, plc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));	//0x328

	period_cycle = period_ns * rate;

	pr_debug("[%s,%d]pwm cclock = %ldHZ, period_ns = %d, period_cycle = %ld, pwm id = %d\n", __func__, __LINE__,
			rate, period_ns, period_cycle, pwm->hwpwm);

	do_div(period_cycle, NSEC_PER_SEC);

	pr_debug("[%s,%d]pwm cclock = %ldHZ, period_ns = %d, period_cycle = %ld, pwm_id = %d\n", __func__, __LINE__,
			rate, period_ns, period_cycle, pwm->hwpwm);

	writel(period_cycle, plc->mmio_base + LIGHT_PWM_PER(pwm->hwpwm));

	duty_cycle = duty_ns * rate;
	do_div(duty_cycle, NSEC_PER_SEC);

	pr_debug("[%s, %d]duty_cycle = %ld\n", __func__, __LINE__, duty_cycle);

	writel(duty_cycle, plc->mmio_base + LIGHT_PWM_FP(pwm->hwpwm));


	pwm_cfg = readl(plc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));
	writel(pwm_cfg | PWM_CFG_UPDATE, plc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));	//0x32c

	pm_runtime_put_sync(chip->dev);

	return 0;
}

static int pwm_light_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm, enum pwm_polarity polarity)
{
	struct pwm_light_chip *plc = to_pwm_light_chip(chip);
	u32 value = readl(plc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));
	int ret;

	ret = pm_runtime_get_sync(chip->dev);
	if (ret < 0) {
		dev_err(chip->dev, "failed to clock on the pwm device(%d)\n", ret);
		pm_runtime_put_noidle(chip->dev);
		return ret;
	}


	if (polarity == PWM_POLARITY_INVERSED)
		/* Duty cycle defines LOW period of PWM */
		value |= PWM_FPOUT;
	else
		/* Duty cycle defines HIGH period of PWM */
		value &= ~PWM_FPOUT;

	writel(value, plc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));

	pm_runtime_put_sync(chip->dev);

	return 0;
}

static const struct pwm_ops pwm_light_ops = {
	.enable = pwm_light_enable,
	.disable = pwm_light_disable,
	.config = pwm_light_config,
	.set_polarity = pwm_light_set_polarity,
	.owner = THIS_MODULE,
};

static int __maybe_unused light_pwm_runtime_suspend(struct device *dev)
{
	struct pwm_light_chip *plc = dev_get_drvdata(dev);

	pwm_light_clk_disable_unprepare(&plc->chip);

	return 0;
}

static int __maybe_unused light_pwm_runtime_resume(struct device *dev)
{
	struct pwm_light_chip *plc = dev_get_drvdata(dev);
	int ret;

	ret = pwm_light_clk_prepare_enable(&plc->chip);
	if (ret) {
		dev_err(dev, "failed to enable pwm clock(%d)\n", ret);
		return ret;
	}

	return 0;
}

static int pwm_light_probe(struct platform_device *pdev)
{
	struct pwm_light_chip *plc;
	struct resource *res;
	int ret;

	plc = devm_kzalloc(&pdev->dev, sizeof(*plc), GFP_KERNEL);
	if (!plc)
		return -ENOMEM;

	platform_set_drvdata(pdev, plc);

	/* optional clock, default open */
	plc->pwm_pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(plc->pwm_pclk)) {
		if (PTR_ERR(plc->pwm_pclk) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to get pwm pclk");
		return PTR_ERR(plc->pwm_pclk);
	}

	plc->pwm_cclk = devm_clk_get(&pdev->dev, "cclk");
	if (IS_ERR(plc->pwm_cclk)) {
		if (PTR_ERR(plc->pwm_cclk) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to get pwm cclk");
		return PTR_ERR(plc->pwm_cclk);
	}

#ifndef CONFIG_PM
	ret = pwm_light_clk_prepare_enable(&plc->chip);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable pwm clock(%d)\n", ret);
		return ret;
	}
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	plc->mmio_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(plc->mmio_base))
		return PTR_ERR(plc->mmio_base);

	plc->chip.ops = &pwm_light_ops;
	plc->chip.dev = &pdev->dev;
	plc->chip.npwm = MAX_PWM_NUM;

	ret = pwmchip_add(&plc->chip);
	if (ret)
		return ret;

	pm_runtime_enable(&pdev->dev);

	dev_info(&pdev->dev, "succeed to add a pwm chip\n");

	return 0;
}

static int pwm_light_remove(struct platform_device *pdev)
{
	struct pwm_light_chip *plc = platform_get_drvdata(pdev);

	pwm_light_clk_disable_unprepare(&plc->chip);

	pm_runtime_disable(&pdev->dev);

	return pwmchip_remove(&plc->chip);
}

static const struct of_device_id pwm_light_dt_ids[] = {
	{.compatible = "thead,pwm-light",},
	{/* sentinel */}
};

static const struct dev_pm_ops pwm_runtime_pm_ops = {
	SET_RUNTIME_PM_OPS(light_pwm_runtime_suspend, light_pwm_runtime_resume, NULL)
};

static struct platform_driver pwm_light_driver = {
	.driver = {
		.name = "pwm-light",
		.of_match_table = pwm_light_dt_ids,
		.pm = &pwm_runtime_pm_ops,
	},
	.probe = pwm_light_probe,
	.remove = pwm_light_remove,
};
module_platform_driver(pwm_light_driver);

MODULE_AUTHOR("wei.liu <lw312886@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead light pwm driver");
MODULE_LICENSE("GPL v2");
