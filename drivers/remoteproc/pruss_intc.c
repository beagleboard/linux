/*
 * PRU-ICSS INTC IRQ domain driver for various TI SoCs
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 *	Suman Anna <s-anna@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pruss.h>

#include "pruss.h"

#define MAX_HOST_NUM_IRQS	8

static const char * const irq_names[] = {
	"host2", "host3", "host4", "host5", "host6", "host7", "host8", "host9",
};

/**
 * struct pruss_intc_match_data - match data to handle SoC variations
 * @no_host7_intr: flag denoting the absence of host7 interrupt into MPU
 */
struct pruss_intc_match_data {
	bool no_host7_intr;
};

/**
 * struct pruss_intc: PRUSS interrupt controller structure
 * @pruss: back-reference to parent PRUSS structure
 * @irqs: kernel irq numbers corresponding to PRUSS host interrupts
 * @irqchip: irq chip for this interrupt controller
 * @domain: irq domain for this interrupt controller
 */
struct pruss_intc {
	struct pruss *pruss;
	unsigned int irqs[MAX_HOST_NUM_IRQS];
	struct irq_chip *irqchip;
	struct irq_domain *domain;
};

static inline u32 pruss_intc_read_reg(struct pruss *pruss, unsigned int reg)
{
	return readl_relaxed(pruss->mem_regions[PRUSS_MEM_INTC].va + reg);
}

static inline void pruss_intc_write_reg(struct pruss *pruss, unsigned int reg,
					u32 val)
{
	writel_relaxed(val, pruss->mem_regions[PRUSS_MEM_INTC].va + reg);
}

static int pruss_intc_check_write(struct pruss *pruss, unsigned int reg,
				  unsigned int sysevent)
{
	if (!pruss)
		return -EINVAL;

	if (sysevent >= MAX_PRU_SYS_EVENTS)
		return -EINVAL;

	pruss_intc_write_reg(pruss, reg, sysevent);

	return 0;
}

static void pruss_intc_irq_ack(struct irq_data *data)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);
	unsigned int hwirq = data->hwirq;

	pruss_intc_check_write(intc->pruss, PRU_INTC_SICR, hwirq);
}

static void pruss_intc_irq_mask(struct irq_data *data)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);
	unsigned int hwirq = data->hwirq;

	pruss_intc_check_write(intc->pruss, PRU_INTC_EICR, hwirq);
}

static void pruss_intc_irq_unmask(struct irq_data *data)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);
	unsigned int hwirq = data->hwirq;

	pruss_intc_check_write(intc->pruss, PRU_INTC_EISR, hwirq);
}

static int pruss_intc_irq_retrigger(struct irq_data *data)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);
	unsigned int hwirq = data->hwirq;

	return pruss_intc_check_write(intc->pruss, PRU_INTC_SISR, hwirq);
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
	for (i = 0; i < MAX_HOST_NUM_IRQS; i++)
		if (intc->irqs[i] == irq)
			break;
	if (i == MAX_HOST_NUM_IRQS)
		goto err;

	i += MIN_PRU_HOST_INT;

	/* get highest priority pending PRUSS system event */
	hipir = pruss_intc_read_reg(intc->pruss, PRU_INTC_HIPIR(i));
	while (!(hipir & BIT(31))) {
		hwirq = hipir & GENMASK(9, 0);
		virq = irq_linear_revmap(intc->domain, hwirq);

		/*
		 * XXX: manually ACK any system events that do not have a
		 * handler mapped yet
		 */
		if (unlikely(!virq))
			pruss_intc_check_write(intc->pruss, PRU_INTC_SICR,
					       hwirq);
		else
			generic_handle_irq(virq);

		/* get next system event */
		hipir = pruss_intc_read_reg(intc->pruss, PRU_INTC_HIPIR(i));
	}
err:
	chained_irq_exit(chip, desc);
}

static const struct of_device_id pruss_intc_of_match[];

static int pruss_intc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *ppdev = to_platform_device(dev->parent);
	struct pruss_intc *intc;
	struct irq_chip *irqchip;
	int i, irq;
	const struct pruss_intc_match_data *data;
	bool skip_host7;

	data = of_match_device(pruss_intc_of_match, dev)->data;
	skip_host7 = data ? data->no_host7_intr : false;

	intc = devm_kzalloc(dev, sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return -ENOMEM;
	platform_set_drvdata(pdev, intc);

	intc->pruss = platform_get_drvdata(ppdev);

	irqchip = devm_kzalloc(dev, sizeof(*irqchip), GFP_KERNEL);
	if (!irqchip)
		return -ENOMEM;

	irqchip->irq_ack = pruss_intc_irq_ack;
	irqchip->irq_mask = pruss_intc_irq_mask;
	irqchip->irq_unmask = pruss_intc_irq_unmask;
	irqchip->irq_retrigger = pruss_intc_irq_retrigger;
	irqchip->name = dev_name(dev);
	intc->irqchip = irqchip;

	/* always 64 events */
	intc->domain = irq_domain_add_linear(dev->of_node, MAX_PRU_SYS_EVENTS,
					     &pruss_intc_irq_domain_ops, intc);
	if (!intc->domain)
		return -ENOMEM;

	for (i = 0; i < MAX_HOST_NUM_IRQS; i++) {
		irq = platform_get_irq_byname(pdev, irq_names[i]);
		if (irq < 0) {
			if (!strcmp(irq_names[i], "host7") && !!skip_host7)
				continue;

			dev_err(dev, "platform_ger_irq_byname failed for %s : %d\n",
				irq_names[i], irq);
			goto fail_irq;
		}

		intc->irqs[i] = irq;
		irq_set_handler_data(irq, intc);
		irq_set_chained_handler(irq, pruss_intc_irq_handler);
	}

	return 0;

fail_irq:
	irq_domain_remove(intc->domain);
	return irq;
}

static int pruss_intc_remove(struct platform_device *pdev)
{
	struct pruss_intc *intc = platform_get_drvdata(pdev);
	unsigned int hwirq;

	if (intc->domain) {
		for (hwirq = 0; hwirq < MAX_PRU_SYS_EVENTS; hwirq++)
			irq_dispose_mapping(irq_find_mapping(intc->domain,
							     hwirq));
		irq_domain_remove(intc->domain);
	}

	return 0;
}

static struct pruss_intc_match_data am437x_pruss_intc_data = {
	.no_host7_intr = true,
};

static const struct of_device_id pruss_intc_of_match[] = {
	{
		.compatible = "ti,am3352-pruss-intc",
		.data = NULL,
	},
	{
		.compatible = "ti,am4372-pruss-intc",
		.data = &am437x_pruss_intc_data,
	},
	{
		.compatible = "ti,am5728-pruss-intc",
		.data = NULL,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pruss_intc_of_match);

static struct platform_driver pruss_intc_driver = {
	.driver = {
		.name = "pruss-intc",
		.of_match_table = pruss_intc_of_match,
	},
	.probe  = pruss_intc_probe,
	.remove = pruss_intc_remove,
};
module_platform_driver(pruss_intc_driver);

MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_DESCRIPTION("PRU-ICSS INTC Driver");
MODULE_LICENSE("GPL v2");
