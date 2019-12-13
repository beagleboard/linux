// SPDX-License-Identifier: GPL-2.0
/*
 * PWM driver for PRU PWM controller
 *
 * Copyright (C) 2019 by Texas Instruments Incorporated - http://www.ti.com/
 * Author: Bin Liu <b-liu@ti.com>
 */

#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pruss.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>

#define PP_FW_MAGIC_NUMBER	0x4d575047	/* "GPWM" */
#define PP_NUM_CHIPS		2
#define PPC_NUM_PWMS		12
/*
 * PWM duty cycle and period thresholds in ns. PWM output is undefined
 * if its duty cycle or period is out of the range. Valid settings:
 *     period: 40us ~ 2sec
 *     duty cycle: 400ns ~ (period - 400ns)
 *
 * These thresholds are determined by the timing required in PRU to clear
 * the IEP CMP events which triggers toggling the PWM pins.
 */
#define PP_MIN_DUTY_NS		400
#define PP_MIN_PERIOD_NS	40000
#define PP_MAX_PERIOD_NS	2000000000	/* 2 sec */

/* global registers */
#define PP_FW_MAGIC		0x00
#define PP_FW_VERSION		0x08
#define PP_CTRL			0x14
#define PP_STAT			0x18

/* PP_CTRL bits */
#define PP_CTRL_IEP_EN		BIT(0)

#define PP_CHIP0_OFFSET		0x1c
#define PP_CHIP1_OFFSET		0x68

/* pwm chip register offsets */
#define PPC_PWM_CFG		0x00
#define PPC_PWM_EN		0x08
#define PPC_PWM_PERIOD		0x0c	/* holds half of period_ns value */
#define PPC_PWM_DC0		0x10
#define PPC_PWM_DC(x)		(PPC_PWM_DC0 + ((x) << 2))

/* PP_PWM_RECFG bits */
#define PPC_PWM_CFG_DC0		BIT(2)
#define PPC_PWM_CFG_DC_MASK	GENMASK(13, 2)

#define PPC_PWM_CFG_COMMIT	1

enum {
	PWMEN_UPDATE,
	PRD_UPDATE,

	MAX_REGFIELDS
};

#define to_pru_pwmchip(c)	container_of((c), struct pru_pwmchip, chip)

struct pru_pwmchip {
	struct pwm_chip chip;
	int period_owner;
	spinlock_t period_lock;    /* lock to serialize pwm period access */
	struct regmap *map;
	struct regmap_field *pwmen_update;
	struct regmap_field *period_update;
};

struct pru_pwm {
	struct device *dev;
	struct rproc *pru;
	struct pruss *pruss;
	int pru_id;
	struct pruss_mem_region mem;
	struct regmap *map;
	struct regmap_field *fw_inited;
};

static const struct regmap_config ppc_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = PPC_PWM_DC(PPC_NUM_PWMS - 1),
};

static const struct reg_field ppc_regfields[MAX_REGFIELDS] = {
	[PWMEN_UPDATE] = REG_FIELD(PPC_PWM_CFG, 0, 0),
	[PRD_UPDATE]   = REG_FIELD(PPC_PWM_CFG, 1, 1),
};

static struct regmap_config pp_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = PP_STAT,
};

static const struct reg_field pp_regfield = REG_FIELD(PP_STAT, 3, 3);

static int prupwm_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			     int duty_ns, int period_ns)
{
	struct pru_pwmchip *ppc = to_pru_pwmchip(chip);
	int duty;
	int idx;
	int ret = 0;

	spin_lock(&ppc->period_lock);
	if (period_ns < PP_MIN_PERIOD_NS || period_ns > PP_MAX_PERIOD_NS) {
		ret = -EINVAL;
		goto out;
	}

	/* check whether period has been set by another pwm */
	if (period_ns != pwm_get_period(pwm) && ppc->period_owner != -1 &&
	    ppc->period_owner != pwm->hwpwm) {
		ret = -EACCES;
		goto out;
	}

	if ((duty_ns && duty_ns < PP_MIN_DUTY_NS) ||
	    duty_ns > period_ns - PP_MIN_DUTY_NS) {
		ret = -EINVAL;
		goto out;
	}

	if (period_ns != pwm_get_period(pwm)) {
		for (idx = 0; idx < chip->npwm; idx++) {
			/* skip current pwm device */
			if (idx == pwm->hwpwm)
				continue;
			/*
			 * the period is global to all pwms, so it cannot be
			 * less than the duty cycle of any pwm.
			 */
			duty = pwm_get_duty_cycle(&chip->pwms[idx]);
			if (period_ns >= duty)
				continue;

			dev_err(chip->dev, "Error: new period (%d) is less than pwm%d duty cycle (%d)\n",
				period_ns, idx, duty);
			ret = -EINVAL;
			goto out;
		}
		ppc->period_owner = pwm->hwpwm;
		kobject_uevent(&chip->dev->kobj, KOBJ_CHANGE);

		/* update the new period in pwm->state for all pwms */
		for (idx = 0; idx < chip->npwm; idx++)
			pwm_set_period(&chip->pwms[idx], period_ns);

		/* the pwm period register holds half of the period value */
		regmap_write(ppc->map, PPC_PWM_PERIOD, period_ns >> 1);
		regmap_field_write(ppc->period_update, PPC_PWM_CFG_COMMIT);
	}

	regmap_write(ppc->map, PPC_PWM_DC(pwm->hwpwm), duty_ns);
	regmap_update_bits(ppc->map, PPC_PWM_CFG, PPC_PWM_CFG_DC0 << pwm->hwpwm,
			   PPC_PWM_CFG_DC0 << pwm->hwpwm);

out:
	spin_unlock(&ppc->period_lock);
	return ret;
}

static int prupwm_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pru_pwmchip *ppc = to_pru_pwmchip(chip);

	regmap_update_bits(ppc->map, PPC_PWM_EN, BIT(pwm->hwpwm),
			   BIT(pwm->hwpwm));
	regmap_field_write(ppc->pwmen_update, PPC_PWM_CFG_COMMIT);

	return 0;
}

static void prupwm_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pru_pwmchip *ppc = to_pru_pwmchip(chip);

	regmap_update_bits(ppc->map, PPC_PWM_EN, BIT(pwm->hwpwm), 0);
	regmap_field_write(ppc->pwmen_update, PPC_PWM_CFG_COMMIT);
}

/* default period register value might not be 0, update it in pwm->state */
static void prupwm_pwm_get_init_state(struct pwm_chip *chip,
				      struct pwm_device *pwm,
				      struct pwm_state *state)
{
	struct pru_pwmchip *ppc = to_pru_pwmchip(chip);
	int period;

	regmap_read(ppc->map, PPC_PWM_PERIOD, &period);
	pwm_set_period(pwm, period << 1);
}

static void prupwm_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pru_pwmchip *ppc = to_pru_pwmchip(chip);

	/* set pwm duty cycle register to 0 */
	regmap_write(ppc->map, PPC_PWM_DC(pwm->hwpwm), 0);
	regmap_update_bits(ppc->map, PPC_PWM_CFG, PPC_PWM_CFG_DC0 << pwm->hwpwm,
			   PPC_PWM_CFG_DC0 << pwm->hwpwm);

	pwm_set_duty_cycle(pwm, 0);

	spin_lock(&ppc->period_lock);
	if (ppc->period_owner == pwm->hwpwm) {
		ppc->period_owner = -1;
		kobject_uevent(&chip->dev->kobj, KOBJ_CHANGE);
	}
	spin_unlock(&ppc->period_lock);
}

static const struct pwm_ops prupwm_pwm_ops = {
	.config = prupwm_pwm_config,
	.enable = prupwm_pwm_enable,
	.disable = prupwm_pwm_disable,
	.get_state = prupwm_pwm_get_init_state,
	.free = prupwm_pwm_free,
	.owner = THIS_MODULE,
};

static const struct of_device_id pru_pwmchip_of_match[] = {
	{ .compatible = "ti,pru-pwmchip", },
	{},
};
MODULE_DEVICE_TABLE(of, pru_pwmchip_of_match);

#ifdef CONFIG_PWM_SYSFS
static int prupwm_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct pwm_chip *chip = dev_get_drvdata(dev);
	struct pru_pwmchip *ppc = to_pru_pwmchip(chip);
	int ret;

	ret = add_uevent_var(env, "PERIOD_OWNER=%d", ppc->period_owner);
	return ret;
}
#endif

static int pru_pwmchip_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *parent = to_platform_device(dev->parent);
	struct pru_pwm *pp = platform_get_drvdata(parent);
	struct pru_pwmchip *ppc;
	void __iomem *mbase;
	int idx, ret;

	ppc = devm_kzalloc(pp->dev, sizeof(*ppc), GFP_KERNEL);
	if (!ppc)
		return -ENOMEM;

	ret = of_property_read_u32(dev->of_node, "reg", &idx);
	if (ret || idx < 0 || idx >= PP_NUM_CHIPS)
		return -EINVAL;

	platform_set_drvdata(pdev, ppc);
	spin_lock_init(&ppc->period_lock);
	ppc->period_owner = -1;

	mbase = pp->mem.va + (idx ? PP_CHIP1_OFFSET : PP_CHIP0_OFFSET);
	ppc->map = devm_regmap_init_mmio(dev, mbase, &ppc_regmap_config);
	if (IS_ERR(ppc->map)) {
		dev_err(dev, "failed to init regmap (%d)\n", ret);
		return PTR_ERR(ppc->map);
	}

	ppc->pwmen_update =
		devm_regmap_field_alloc(dev, ppc->map,
					ppc_regfields[PWMEN_UPDATE]);
	if (IS_ERR(ppc->pwmen_update))
		return PTR_ERR(ppc->pwmen_update);

	ppc->period_update =
		devm_regmap_field_alloc(dev, ppc->map,
					ppc_regfields[PRD_UPDATE]);
	if (IS_ERR(ppc->period_update))
		return PTR_ERR(ppc->period_update);

	ppc->chip.dev = dev;
	ppc->chip.ops = &prupwm_pwm_ops;
	ppc->chip.base = -1;
	ppc->chip.npwm = PPC_NUM_PWMS;

	/* set initial duty cycle register of all pwms to 0 */
	for (idx = 0; idx < PPC_NUM_PWMS; idx++)
		regmap_write(ppc->map, PPC_PWM_DC(idx), 0);

	/* commit the duty cycle config changes */
	regmap_write(ppc->map, PPC_PWM_CFG, PPC_PWM_CFG_DC_MASK);

	ret = pwmchip_add(&ppc->chip);
	if (ret)
		dev_err(dev, "pwmchip_add() failed: %d\n", ret);

#ifdef CONFIG_PWM_SYSFS
	if (ppc->chip.sysfs_dev)
		ppc->chip.sysfs_dev->class->dev_uevent = prupwm_uevent;
#endif

	return ret;
}

static int pru_pwmchip_remove(struct platform_device *pdev)
{
	struct pru_pwmchip *ppc = platform_get_drvdata(pdev);

	return pwmchip_remove(&ppc->chip);
}

static struct platform_driver pru_pwmchip_driver = {
	.driver = {
		.name = "pru_pwmchip",
		.of_match_table = pru_pwmchip_of_match,
	},
	.probe = pru_pwmchip_probe,
	.remove = pru_pwmchip_remove,
};

static int prupwm_init_prufw(struct device_node *np, struct pru_pwm *pp)
{
	struct device *dev = pp->dev;
	struct device_node *child;
	enum pruss_mem mem_id;
	u32 reg, id;
	int ret = 0;

	pp->pru = pru_rproc_get(np, 0);
	if (IS_ERR(pp->pru)) {
		ret = PTR_ERR(pp->pru);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get pru (%d)\n", ret);
		return ret;
	}

	pp->pruss = pruss_get(pp->pru);
	if (IS_ERR(pp->pruss)) {
		ret = PTR_ERR(pp->pruss);
		dev_err(dev, "failed to get pruss handle (%d)\n", ret);
		goto put_pru;
	}

	pp->pru_id = pru_rproc_get_id(pp->pru);
	if (pp->pru_id < 0) {
		dev_err(dev, "failed to get pru id (%d)\n", pp->pru_id);
		ret = -EINVAL;
		goto put_pruss;
	}

	if (pp->pru_id > 1) {
		dev_err(dev, "invalid pru id (%d)\n", pp->pru_id);
		ret = -EINVAL;
		goto put_pruss;
	}

	mem_id = pp->pru_id ? PRUSS_MEM_DRAM1 : PRUSS_MEM_DRAM0;
	ret = pruss_request_mem_region(pp->pruss, mem_id, &pp->mem);
	if (ret) {
		dev_err(dev, "failed to get pruss mem region (%d)\n", ret);
		goto put_pruss;
	}

	pp->map = devm_regmap_init_mmio(dev, pp->mem.va, &pp_regmap_config);
	if (IS_ERR(pp->map)) {
		ret = PTR_ERR(pp->map);
		dev_err(dev, "failed to init register map (%d)\n", ret);
		goto put_mem;
	}

	pp->fw_inited = devm_regmap_field_alloc(dev, pp->map, pp_regfield);
	if (IS_ERR(pp->fw_inited)) {
		ret = PTR_ERR(pp->fw_inited);
		goto put_mem;
	}

	/* clear the mem region before firmware runs by rproc_boot() */
	memset_io(pp->mem.va, 0, pp->mem.size);

	ret = rproc_boot(pp->pru);
	if (ret) {
		dev_err(dev, "failed to boot pru (%d)\n", ret);
		goto put_mem;
	}

	regmap_read(pp->map, PP_FW_MAGIC, &reg);
	if (reg != PP_FW_MAGIC_NUMBER) {
		dev_err(dev, "invalid firmware magic number\n");
		ret = -EINVAL;
		goto put_rproc;
	}

	regmap_read(pp->map, PP_FW_VERSION, &reg);
	if (reg > 0x01000000) {
		dev_err(dev, "unsupported firmware version(0x%x)\n", reg);
		ret = -EINVAL;
		goto put_rproc;
	}

	reg = 0;
	for_each_available_child_of_node(np, child) {
		ret = of_property_read_u32(child, "reg", &id);

		if (ret || id >= PP_NUM_CHIPS) {
			dev_err(dev, "invalid pwmchip id %d (%d)\n", id, ret);
			ret = -EINVAL;
			goto put_rproc;
		}

		reg |= 1 << (id + 1);
	}

	if (reg)
		reg |= PP_CTRL_IEP_EN;
	regmap_write(pp->map, PP_CTRL, reg);

	/* check for firmware init completion, timeout in 100us */
	ret = regmap_field_read_poll_timeout(pp->fw_inited, reg, reg, 0, 100);
	if (ret == -ETIMEDOUT)
		dev_err(dev, "failed to initialize firmware\n");
	else if (!ret)
		return 0;

put_rproc:
	rproc_shutdown(pp->pru);
put_mem:
	pruss_release_mem_region(pp->pruss, &pp->mem);
put_pruss:
	pruss_put(pp->pruss);
put_pru:
	pru_rproc_put(pp->pru);

	return ret;
}

static int prupwm_exit_pruss(struct pru_pwm *pp)
{
	int ret;

	rproc_shutdown(pp->pru);
	ret = pruss_release_mem_region(pp->pruss, &pp->mem);
	if (ret)
		return ret;

	pruss_put(pp->pruss);
	pru_rproc_put(pp->pru);

	return 0;
}

static const struct of_device_id prupwm_of_match[] = {
	{ .compatible = "ti,pru-pwm", },
	{},
};
MODULE_DEVICE_TABLE(of, prupwm_of_match);

static int prupwm_probe(struct platform_device *pdev)
{
	struct pru_pwm *pp;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;

	if (!np)
		return -ENODEV;	/* non-DT not supported */

	pp = devm_kzalloc(dev, sizeof(*pp), GFP_KERNEL);
	if (!pp)
		return -ENOMEM;

	pp->dev = dev;
	ret = prupwm_init_prufw(np, pp);
	if (ret < 0)
		return -ENODEV;

	platform_set_drvdata(pdev, pp);

	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret) {
		dev_err(dev, "failed to create pwmchip\n");
		prupwm_exit_pruss(pp);
		return ret;
	}

	return 0;
}

static int prupwm_remove(struct platform_device *pdev)
{
	struct pru_pwm *pp = platform_get_drvdata(pdev);

	of_platform_depopulate(&pdev->dev);
	return prupwm_exit_pruss(pp);
}

static struct platform_driver prupwm_driver = {
	.driver = {
		.name = "prupwm",
		.of_match_table = prupwm_of_match,
	},
	.probe = prupwm_probe,
	.remove = prupwm_remove,
};

static int __init prupwm_init(void)
{
	int ret;

	ret = platform_driver_register(&pru_pwmchip_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&prupwm_driver);
	if (ret)
		platform_driver_unregister(&pru_pwmchip_driver);

	return ret;
}
module_init(prupwm_init);

static void __exit prupwm_exit(void)
{
	platform_driver_unregister(&prupwm_driver);
	platform_driver_unregister(&pru_pwmchip_driver);
}
module_exit(prupwm_exit);

MODULE_AUTHOR("Bin Liu <b-liu@ti.com>");
MODULE_DESCRIPTION("PRU PWM Driver");
MODULE_LICENSE("GPL v2");
