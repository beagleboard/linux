/*
 * eCAP driver for PWM output generation
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm/pwm.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>

#include <plat/clock.h>
#include <plat/config_pwm.h>

#define TIMER_CTR_REG			0x0
#define CAPTURE_1_REG			0x08
#define CAPTURE_2_REG			0x0c
#define CAPTURE_3_REG			0x10
#define CAPTURE_4_REG			0x14
#define CAPTURE_CTRL2_REG		0x2A

#define ECTRL2_SYNCOSEL_MASK		(0x03 << 6)

#define ECTRL2_MDSL_ECAP		BIT(9)
#define ECTRL2_CTRSTP_FREERUN		BIT(4)
#define ECTRL2_PLSL_LOW			BIT(10)
#define ECTRL2_SYNC_EN			BIT(5)

struct ecap_regs {
	unsigned	tsctr;
	unsigned	cap1;
	unsigned	cap2;
	unsigned	cap3;
	unsigned	cap4;
	unsigned short	ecctl2;
	unsigned short	clkconfig;
};

struct ecap_pwm {
	struct pwm_device pwm;
	struct pwm_device_ops ops;
	spinlock_t	lock;
	struct clk	*clk;
	void __iomem	*mmio_base;
	u8 version;
	void __iomem *config_mem_base;
	struct device *dev;
	struct ecap_regs ctx;
};

static inline struct ecap_pwm *to_ecap_pwm(const struct pwm_device *p)
{
	return pwm_get_drvdata(p);
}

static int ecap_pwm_stop(struct pwm_device *p)
{
	unsigned long flags, v;
	struct ecap_pwm *ep = to_ecap_pwm(p);

	/* Trying to stop a non-running PWM, not allowed */
	if (!pwm_is_running(p))
		return -EPERM;

	spin_lock_irqsave(&ep->lock, flags);
	v = readw(ep->mmio_base + CAPTURE_CTRL2_REG);
	v &= ~ECTRL2_CTRSTP_FREERUN;
	writew(v, ep->mmio_base + CAPTURE_CTRL2_REG);
	spin_unlock_irqrestore(&ep->lock, flags);

	/* For PWM clock should be disabled on stop */
	pm_runtime_put_sync(ep->dev);
	clear_bit(FLAG_RUNNING, &p->flags);

	return 0;
}

static int ecap_pwm_start(struct pwm_device *p)
{
	int ret = 0;
	unsigned long flags, v;
	struct ecap_pwm *ep = to_ecap_pwm(p);

	/* Trying to start a running PWM, not allowed */
	if (pwm_is_running(p))
		return -EPERM;

	/* For PWM clock should be enabled on start */
	pm_runtime_get_sync(ep->dev);

	spin_lock_irqsave(&ep->lock, flags);
	v = readw(ep->mmio_base + CAPTURE_CTRL2_REG);
	v |= ECTRL2_CTRSTP_FREERUN;
	writew(v, ep->mmio_base + CAPTURE_CTRL2_REG);
	spin_unlock_irqrestore(&ep->lock, flags);
	set_bit(FLAG_RUNNING, &p->flags);

	return ret;
}

static int ecap_pwm_set_polarity(struct pwm_device *p, char pol)
{
	unsigned long flags, v;
	struct ecap_pwm *ep = to_ecap_pwm(p);

	pm_runtime_get_sync(ep->dev);

	spin_lock_irqsave(&ep->lock, flags);
	v = readw(ep->mmio_base + CAPTURE_CTRL2_REG);
	v &= ~ECTRL2_PLSL_LOW;
	v |= (!pol << 10);
	writew(v, ep->mmio_base + CAPTURE_CTRL2_REG);
	spin_unlock_irqrestore(&ep->lock, flags);

	pm_runtime_put_sync(ep->dev);
	return 0;
}

static int ecap_pwm_config_period(struct pwm_device *p)
{
	unsigned long flags, v;
	struct ecap_pwm *ep = to_ecap_pwm(p);

	 pm_runtime_get_sync(ep->dev);

	spin_lock_irqsave(&ep->lock, flags);
	writel((p->period_ticks) - 1, ep->mmio_base + CAPTURE_3_REG);
	v = readw(ep->mmio_base + CAPTURE_CTRL2_REG);
	v |= (ECTRL2_MDSL_ECAP | ECTRL2_SYNCOSEL_MASK);
	writew(v, ep->mmio_base + CAPTURE_CTRL2_REG);
	spin_unlock_irqrestore(&ep->lock, flags);

	pm_runtime_put_sync(ep->dev);
	return 0;
}

static int ecap_pwm_config_duty(struct pwm_device *p)
{
	unsigned long flags, v;
	struct ecap_pwm *ep = to_ecap_pwm(p);

	pm_runtime_get_sync(ep->dev);

	spin_lock_irqsave(&ep->lock, flags);
	v = readw(ep->mmio_base + CAPTURE_CTRL2_REG);
	v |= (ECTRL2_MDSL_ECAP | ECTRL2_SYNCOSEL_MASK);
	writew(v, ep->mmio_base + CAPTURE_CTRL2_REG);

	if (p->duty_ticks > 0) {
		writel(p->duty_ticks, ep->mmio_base + CAPTURE_4_REG);
	} else {
		writel(p->duty_ticks, ep->mmio_base + CAPTURE_2_REG);
		writel(0, ep->mmio_base + TIMER_CTR_REG);
	}
	spin_unlock_irqrestore(&ep->lock, flags);

	pm_runtime_put_sync(ep->dev);
	return 0;
}

static int ecap_pwm_config(struct pwm_device *p,
				struct pwm_config *c)
{
	int ret = 0;
	switch (c->config_mask) {

	case BIT(PWM_CONFIG_DUTY_TICKS):
		p->duty_ticks = c->duty_ticks;
		ret = ecap_pwm_config_duty(p);
		break;

	case BIT(PWM_CONFIG_PERIOD_TICKS):
		p->period_ticks = c->period_ticks;
		ret = ecap_pwm_config_period(p);
		break;

	case BIT(PWM_CONFIG_POLARITY):
		ret = ecap_pwm_set_polarity(p, c->polarity);
		break;

	case BIT(PWM_CONFIG_START):
		ret = ecap_pwm_start(p);
		break;

	case BIT(PWM_CONFIG_STOP):
		ret = ecap_pwm_stop(p);
		break;
	}

	return ret;
}

static int ecap_pwm_request(struct pwm_device *p)
{
	struct ecap_pwm *ep = to_ecap_pwm(p);

	p->tick_hz = clk_get_rate(ep->clk);
	return 0;
}

static int ecap_frequency_transition_cb(struct pwm_device *p)
{
	struct ecap_pwm *ep = to_ecap_pwm(p);
	unsigned long duty_ns, rate;

	rate = clk_get_rate(ep->clk);
	if (rate == p->tick_hz)
		return 0;
	p->tick_hz = rate;

	duty_ns = p->duty_ns;
	if (pwm_is_running(p)) {
		pwm_stop(p);
		pwm_set_duty_ns(p, 0);
		pwm_set_period_ns(p, p->period_ns);
		pwm_set_duty_ns(p, duty_ns);
		pwm_start(p);
	} else {
		pwm_set_duty_ns(p, 0);
		pwm_set_period_ns(p, p->period_ns);
		pwm_set_duty_ns(p, duty_ns);
	}
		return 0;
}

static int ecap_probe(struct platform_device *pdev)
{
	struct ecap_pwm *ep = NULL;
	struct resource *r;
	int ret = 0;
	int val;
	char con_id[PWM_CON_ID_STRING_LENGTH] = "epwmss";
	struct pwmss_platform_data *pdata = (&pdev->dev)->platform_data;

	ep = kzalloc(sizeof(*ep), GFP_KERNEL);

	if (!ep) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	ep->version = pdata->version;

	if (ep->version == PWM_VERSION_1) {
		sprintf(con_id, "%s%d_%s", con_id, pdev->id, "fck");
		ep->clk = clk_get(&pdev->dev, con_id);
	} else
		ep->clk = clk_get(&pdev->dev, "ecap");

	pm_runtime_enable(&pdev->dev);
	ep->dev = &pdev->dev;
	if (IS_ERR(ep->clk)) {
		ret = PTR_ERR(ep->clk);
		goto err_clk_get;
	}

	if (ep->version == PWM_VERSION_1) {
		r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

		if (!r) {
			dev_err(&pdev->dev, "no memory resource defined\n");
			ret = -ENOMEM;
			goto err_get_resource;
		}

		ep->config_mem_base = ioremap(r->start, resource_size(r));

		if (!ep->config_mem_base) {

			dev_err(&pdev->dev, "failed to ioremap() registers\n");
			ret = -ENOMEM;
			goto err_get_resource;
		}

		pm_runtime_get_sync(ep->dev);
		val = readw(ep->config_mem_base + PWMSS_CLKCONFIG);
		val |= BIT(ECAP_CLK_EN);
		writew(val, ep->config_mem_base + PWMSS_CLKCONFIG);
		pm_runtime_put_sync(ep->dev);
	}

	spin_lock_init(&ep->lock);
	ep->ops.config = ecap_pwm_config;
	ep->ops.request = ecap_pwm_request;
	ep->ops.freq_transition_notifier_cb = ecap_frequency_transition_cb;

	if (ep->version == PWM_VERSION_1)
		r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	else
		r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!r) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_request_mem;
	}

	r = request_mem_region(r->start, resource_size(r), pdev->name);
	if (!r) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_request_mem;
	}

	ep->mmio_base = ioremap(r->start, resource_size(r));
	if (!ep->mmio_base) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_ioremap;
	}

	ep->pwm.ops = &ep->ops;
	pwm_set_drvdata(&ep->pwm, ep);
	ret =  pwm_register(&ep->pwm, &pdev->dev, -1);
	platform_set_drvdata(pdev, ep);
	return 0;

err_ioremap:
	release_mem_region(r->start, resource_size(r));
err_request_mem:
	if (ep->version == PWM_VERSION_1) {
		iounmap(ep->config_mem_base);
		ep->config_mem_base = NULL;
	}
err_get_resource:
	clk_put(ep->clk);
	pm_runtime_disable(&pdev->dev);
err_clk_get:
	kfree(ep);
	return ret;
}

#ifdef CONFIG_PM

void ecap_save_reg(struct ecap_pwm *ep)
{
	pm_runtime_get_sync(ep->dev);

	ep->ctx.ecctl2 = readw(ep->mmio_base + CAPTURE_CTRL2_REG);
	ep->ctx.tsctr = readl(ep->mmio_base + TIMER_CTR_REG);
	ep->ctx.cap1 = readl(ep->mmio_base + CAPTURE_1_REG);
	ep->ctx.cap2 = readl(ep->mmio_base + CAPTURE_2_REG);
	ep->ctx.cap4 = readl(ep->mmio_base + CAPTURE_4_REG);
	ep->ctx.cap3 = readl(ep->mmio_base + CAPTURE_3_REG);

	ep->ctx.clkconfig = readw(ep->config_mem_base + PWMSS_CLKCONFIG);

	pm_runtime_put_sync(ep->dev);
}

void ecap_restore_reg(struct ecap_pwm *ep)
{
	writew(ep->ctx.clkconfig, ep->config_mem_base + PWMSS_CLKCONFIG);

	writel(ep->ctx.cap3, ep->mmio_base + CAPTURE_3_REG);
	writel(ep->ctx.cap4, ep->mmio_base + CAPTURE_4_REG);
	writel(ep->ctx.cap2, ep->mmio_base + CAPTURE_2_REG);
	writel(ep->ctx.cap1, ep->mmio_base + CAPTURE_1_REG);
	writel(ep->ctx.tsctr, ep->mmio_base + TIMER_CTR_REG);
	writew(ep->ctx.ecctl2, ep->mmio_base + CAPTURE_CTRL2_REG);
}

static int ecap_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ecap_pwm *ep = platform_get_drvdata(pdev);

	ecap_save_reg(ep);
	pm_runtime_put_sync(ep->dev);

	return 0;
}

static int ecap_resume(struct platform_device *pdev)
{
	struct ecap_pwm *ep = platform_get_drvdata(pdev);

	pm_runtime_get_sync(ep->dev);

	ecap_restore_reg(ep);

	return 0;
}

#else
#define ecap_suspend NULL
#define ecap_resume NULL
#endif

static int __devexit ecap_remove(struct platform_device *pdev)
{
	struct ecap_pwm *ep = platform_get_drvdata(pdev);
	struct resource *r;
	struct pwmss_platform_data *pdata;
	int val;

	if (ep->version == PWM_VERSION_1) {
		pdata = (&pdev->dev)->platform_data;
		val = readw(ep->config_mem_base + PWMSS_CLKCONFIG);
		val &= ~BIT(ECAP_CLK_EN);
		writew(val, ep->config_mem_base + PWMSS_CLKCONFIG);
		iounmap(ep->config_mem_base);
		ep->config_mem_base = NULL;
	}

	pwm_unregister(&ep->pwm);
	iounmap(ep->mmio_base);

	if (ep->version == PWM_VERSION_1)
		r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	else
		r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, resource_size(r));
	platform_set_drvdata(pdev, NULL);
	clk_put(ep->clk);
	pm_runtime_disable(&pdev->dev);
	kfree(ep);

	return 0;
}

static struct platform_driver ecap_driver = {
	.driver	= {
		.name	= "ecap",
		.owner	= THIS_MODULE,
	},
	.probe		= ecap_probe,
	.remove		= __devexit_p(ecap_remove),
	.suspend	= ecap_suspend,
	.resume		= ecap_resume,
};

static int __init ecap_init(void)
{
	return platform_driver_register(&ecap_driver);
}

static void __exit ecap_exit(void)
{
	platform_driver_unregister(&ecap_driver);
}

module_init(ecap_init);
module_exit(ecap_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Driver for Davinci eCAP peripheral");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ecap");
