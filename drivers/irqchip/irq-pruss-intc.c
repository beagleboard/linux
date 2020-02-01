// SPDX-License-Identifier: GPL-2.0-only
/*
 * PRU-ICSS INTC IRQChip driver for various TI SoCs
 *
 * Copyright (C) 2016-2020 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 *	Suman Anna <s-anna@ti.com>
 */

#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

/*
 * Number of host interrupts reaching the main MPU sub-system. Note that this
 * is not the same as the total number of host interrupts supported by the PRUSS
 * INTC instance
 */
#define MAX_NUM_HOST_IRQS	8

/* minimum starting host interrupt number for MPU */
#define MIN_PRU_HOST_INT	2

/* maximum number of system events */
#define MAX_PRU_SYS_EVENTS	64

/* PRU_ICSS_INTC registers */
#define PRU_INTC_REVID		0x0000
#define PRU_INTC_CR		0x0004
#define PRU_INTC_GER		0x0010
#define PRU_INTC_GNLR		0x001c
#define PRU_INTC_SISR		0x0020
#define PRU_INTC_SICR		0x0024
#define PRU_INTC_EISR		0x0028
#define PRU_INTC_EICR		0x002c
#define PRU_INTC_HIEISR		0x0034
#define PRU_INTC_HIDISR		0x0038
#define PRU_INTC_GPIR		0x0080
#define PRU_INTC_SRSR0		0x0200
#define PRU_INTC_SRSR1		0x0204
#define PRU_INTC_SECR0		0x0280
#define PRU_INTC_SECR1		0x0284
#define PRU_INTC_ESR0		0x0300
#define PRU_INTC_ESR1		0x0304
#define PRU_INTC_ECR0		0x0380
#define PRU_INTC_ECR1		0x0384
#define PRU_INTC_CMR(x)		(0x0400 + (x) * 4)
#define PRU_INTC_HMR(x)		(0x0800 + (x) * 4)
#define PRU_INTC_HIPIR(x)	(0x0900 + (x) * 4)
#define PRU_INTC_SIPR0		0x0d00
#define PRU_INTC_SIPR1		0x0d04
#define PRU_INTC_SITR0		0x0d80
#define PRU_INTC_SITR1		0x0d84
#define PRU_INTC_HINLR(x)	(0x1100 + (x) * 4)
#define PRU_INTC_HIER		0x1500

/* HIPIR register bit-fields */
#define INTC_HIPIR_NONE_HINT	0x80000000

/**
 * struct pruss_intc - PRUSS interrupt controller structure
 * @irqs: kernel irq numbers corresponding to PRUSS host interrupts
 * @base: base virtual address of INTC register space
 * @irqchip: irq chip for this interrupt controller
 * @domain: irq domain for this interrupt controller
 * @lock: mutex to serialize access to INTC
 */
struct pruss_intc {
	unsigned int irqs[MAX_NUM_HOST_IRQS];
	void __iomem *base;
	struct irq_chip *irqchip;
	struct irq_domain *domain;
	struct mutex lock; /* PRUSS INTC lock */
};

static inline u32 pruss_intc_read_reg(struct pruss_intc *intc, unsigned int reg)
{
	return readl_relaxed(intc->base + reg);
}

static inline void pruss_intc_write_reg(struct pruss_intc *intc,
					unsigned int reg, u32 val)
{
	writel_relaxed(val, intc->base + reg);
}

static int pruss_intc_check_write(struct pruss_intc *intc, unsigned int reg,
				  unsigned int sysevent)
{
	if (!intc)
		return -EINVAL;

	if (sysevent >= MAX_PRU_SYS_EVENTS)
		return -EINVAL;

	pruss_intc_write_reg(intc, reg, sysevent);

	return 0;
}

static void pruss_intc_init(struct pruss_intc *intc)
{
	int i;

	/* configure polarity to active high for all system interrupts */
	pruss_intc_write_reg(intc, PRU_INTC_SIPR0, 0xffffffff);
	pruss_intc_write_reg(intc, PRU_INTC_SIPR1, 0xffffffff);

	/* configure type to pulse interrupt for all system interrupts */
	pruss_intc_write_reg(intc, PRU_INTC_SITR0, 0);
	pruss_intc_write_reg(intc, PRU_INTC_SITR1, 0);

	/* clear all 16 interrupt channel map registers */
	for (i = 0; i < 16; i++)
		pruss_intc_write_reg(intc, PRU_INTC_CMR(i), 0);

	/* clear all 3 host interrupt map registers */
	for (i = 0; i < 3; i++)
		pruss_intc_write_reg(intc, PRU_INTC_HMR(i), 0);
}

static void pruss_intc_irq_ack(struct irq_data *data)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);
	unsigned int hwirq = data->hwirq;

	pruss_intc_check_write(intc, PRU_INTC_SICR, hwirq);
}

static void pruss_intc_irq_mask(struct irq_data *data)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);
	unsigned int hwirq = data->hwirq;

	pruss_intc_check_write(intc, PRU_INTC_EICR, hwirq);
}

static void pruss_intc_irq_unmask(struct irq_data *data)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);
	unsigned int hwirq = data->hwirq;

	pruss_intc_check_write(intc, PRU_INTC_EISR, hwirq);
}

static int pruss_intc_irq_reqres(struct irq_data *data)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	return 0;
}

static void pruss_intc_irq_relres(struct irq_data *data)
{
	module_put(THIS_MODULE);
}

static int pruss_intc_irq_domain_map(struct irq_domain *d, unsigned int virq,
				     irq_hw_number_t hw)
{
	struct pruss_intc *intc = d->host_data;

	irq_set_chip_data(virq, intc);
	irq_set_chip_and_handler(virq, intc->irqchip, handle_level_irq);

	return 0;
}

static void pruss_intc_irq_domain_unmap(struct irq_domain *d, unsigned int virq)
{
	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
}

static const struct irq_domain_ops pruss_intc_irq_domain_ops = {
	.xlate	= irq_domain_xlate_onecell,
	.map	= pruss_intc_irq_domain_map,
	.unmap	= pruss_intc_irq_domain_unmap,
};

static void pruss_intc_irq_handler(struct irq_desc *desc)
{
	unsigned int irq = irq_desc_get_irq(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct pruss_intc *intc = irq_get_handler_data(irq);
	u32 hipir;
	unsigned int virq;
	int i, hwirq;

	chained_irq_enter(chip, desc);

	/* find our host irq number */
	for (i = 0; i < MAX_NUM_HOST_IRQS; i++)
		if (intc->irqs[i] == irq)
			break;
	if (i == MAX_NUM_HOST_IRQS)
		goto err;

	i += MIN_PRU_HOST_INT;

	/* get highest priority pending PRUSS system event */
	hipir = pruss_intc_read_reg(intc, PRU_INTC_HIPIR(i));
	while (!(hipir & INTC_HIPIR_NONE_HINT)) {
		hwirq = hipir & GENMASK(9, 0);
		virq = irq_linear_revmap(intc->domain, hwirq);

		/*
		 * NOTE: manually ACK any system events that do not have a
		 * handler mapped yet
		 */
		if (unlikely(!virq))
			pruss_intc_check_write(intc, PRU_INTC_SICR, hwirq);
		else
			generic_handle_irq(virq);

		/* get next system event */
		hipir = pruss_intc_read_reg(intc, PRU_INTC_HIPIR(i));
	}
err:
	chained_irq_exit(chip, desc);
}

static int pruss_intc_probe(struct platform_device *pdev)
{
	static const char * const irq_names[] = {
		"host_intr0", "host_intr1", "host_intr2", "host_intr3",
		"host_intr4", "host_intr5", "host_intr6", "host_intr7", };
	struct device *dev = &pdev->dev;
	struct pruss_intc *intc;
	struct irq_chip *irqchip;
	int i, irq;

	intc = devm_kzalloc(dev, sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return -ENOMEM;
	platform_set_drvdata(pdev, intc);

	intc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(intc->base)) {
		dev_err(dev, "failed to parse and map intc memory resource\n");
		return PTR_ERR(intc->base);
	}

	mutex_init(&intc->lock);

	pruss_intc_init(intc);

	irqchip = devm_kzalloc(dev, sizeof(*irqchip), GFP_KERNEL);
	if (!irqchip)
		return -ENOMEM;

	irqchip->irq_ack = pruss_intc_irq_ack;
	irqchip->irq_mask = pruss_intc_irq_mask;
	irqchip->irq_unmask = pruss_intc_irq_unmask;
	irqchip->irq_request_resources = pruss_intc_irq_reqres;
	irqchip->irq_release_resources = pruss_intc_irq_relres;
	irqchip->name = dev_name(dev);
	intc->irqchip = irqchip;

	/* always 64 events */
	intc->domain = irq_domain_add_linear(dev->of_node, MAX_PRU_SYS_EVENTS,
					     &pruss_intc_irq_domain_ops, intc);
	if (!intc->domain)
		return -ENOMEM;

	for (i = 0; i < MAX_NUM_HOST_IRQS; i++) {
		irq = platform_get_irq_byname(pdev, irq_names[i]);
		if (irq < 0) {
			dev_err(dev, "platform_get_irq_byname failed for %s : %d\n",
				irq_names[i], irq);
			goto fail_irq;
		}

		intc->irqs[i] = irq;
		irq_set_handler_data(irq, intc);
		irq_set_chained_handler(irq, pruss_intc_irq_handler);
	}

	return 0;

fail_irq:
	while (--i >= 0) {
		if (intc->irqs[i])
			irq_set_chained_handler_and_data(intc->irqs[i], NULL,
							 NULL);
	}
	irq_domain_remove(intc->domain);
	return irq;
}

static int pruss_intc_remove(struct platform_device *pdev)
{
	struct pruss_intc *intc = platform_get_drvdata(pdev);
	unsigned int hwirq;
	int i;

	for (i = 0; i < MAX_NUM_HOST_IRQS; i++) {
		if (intc->irqs[i])
			irq_set_chained_handler_and_data(intc->irqs[i], NULL,
							 NULL);
	}

	for (hwirq = 0; hwirq < MAX_PRU_SYS_EVENTS; hwirq++)
		irq_dispose_mapping(irq_find_mapping(intc->domain, hwirq));
	irq_domain_remove(intc->domain);

	return 0;
}

static const struct of_device_id pruss_intc_of_match[] = {
	{ .compatible = "ti,pruss-intc", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pruss_intc_of_match);

static struct platform_driver pruss_intc_driver = {
	.driver = {
		.name = "pruss-intc",
		.of_match_table = pruss_intc_of_match,
		.suppress_bind_attrs = true,
	},
	.probe  = pruss_intc_probe,
	.remove = pruss_intc_remove,
};
module_platform_driver(pruss_intc_driver);

MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_DESCRIPTION("TI PRU-ICSS INTC Driver");
MODULE_LICENSE("GPL v2");
