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
 * @mem: kernel-mapping data for the INTC register space
 * @irqchip: irq chip for this interrupt controller
 * @domain: irq domain for this interrupt controller
 * @config_map: stored INTC configuration mapping data
 * @lock: mutex to serialize access to INTC
 */
struct pruss_intc {
	struct pruss *pruss;
	unsigned int irqs[MAX_HOST_NUM_IRQS];
	struct pruss_mem_region mem;
	struct irq_chip *irqchip;
	struct irq_domain *domain;
	struct pruss_intc_config config_map;
	struct mutex lock; /* PRUSS INTC lock */
};

static inline u32 pruss_intc_read_reg(struct pruss_intc *intc, unsigned int reg)
{
	return readl_relaxed(intc->mem.va + reg);
}

static inline void pruss_intc_write_reg(struct pruss_intc *intc,
					unsigned int reg, u32 val)
{
	writel_relaxed(val, intc->mem.va + reg);
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

static struct pruss_intc *to_pruss_intc(struct pruss *pruss)
{
	struct device_node *parent = pruss->dev->of_node;
	struct device_node *np;
	struct platform_device *pdev;
	struct pruss_intc *intc = NULL;

	np = of_get_child_by_name(parent, "intc");
	if (!np) {
		dev_err(pruss->dev, "pruss does not have an intc node\n");
		return NULL;
	}

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		dev_err(pruss->dev, "no associated platform device\n");
		goto out;
	}

	intc = platform_get_drvdata(pdev);
out:
	of_node_put(np);
	return intc;
}

/**
 * pruss_intc_configure() - configure the PRUSS INTC
 * @pruss: the pruss instance
 * @intc_config: PRU core-specific INTC configuration
 *
 * Configures the PRUSS INTC with the provided configuration from
 * a PRU core. Any existing event to channel mappings or channel to
 * host interrupt mappings are checked to make sure there are no
 * conflicting configuration between both the PRU cores. The function
 * is intended to be only used by the PRU remoteproc driver.
 *
 * Returns 0 on success, or a suitable error code otherwise
 */
int pruss_intc_configure(struct pruss *pruss,
			 struct pruss_intc_config *intc_config)
{
	struct device *dev = pruss->dev;
	struct pruss_intc *intc = to_pruss_intc(pruss);
	int i, idx, ch, host, ret;
	u64 sysevt_mask = 0;
	u32 ch_mask = 0;
	u32 host_mask = 0;
	u32 val;

	if (!intc)
		return -EINVAL;

	mutex_lock(&intc->lock);

	/*
	 * configure channel map registers - each register holds map info
	 * for 4 events, with each event occupying the lower nibble in
	 * a register byte address in little-endian fashion
	 */
	for (i = 0; i < ARRAY_SIZE(intc_config->sysev_to_ch); i++) {
		ch = intc_config->sysev_to_ch[i];
		if (ch < 0)
			continue;

		/* check if sysevent already assigned */
		if (intc->config_map.sysev_to_ch[i] != -1) {
			dev_err(dev, "event %d (req. channel %d) already assigned to channel %d\n",
				i, ch, intc->config_map.sysev_to_ch[i]);
			ret = -EEXIST;
			goto unlock;
		}

		intc->config_map.sysev_to_ch[i] = ch;

		idx = i / 4;
		val = pruss_intc_read_reg(intc, PRU_INTC_CMR(idx));
		val |= ch << ((i & 3) * 8);
		pruss_intc_write_reg(intc, PRU_INTC_CMR(idx), val);
		sysevt_mask |= BIT_ULL(i);
		ch_mask |= BIT(ch);

		dev_dbg(dev, "SYSEV%d -> CH%d (CMR%d 0x%08x)\n", i, ch, idx,
			pruss_intc_read_reg(intc, PRU_INTC_CMR(idx)));
	}

	/*
	 * set host map registers - each register holds map info for
	 * 4 channels, with each channel occupying the lower nibble in
	 * a register byte address in little-endian fashion
	 */
	for (i = 0; i < ARRAY_SIZE(intc_config->ch_to_host); i++) {
		host = intc_config->ch_to_host[i];
		if (host < 0)
			continue;

		/* check if channel already assigned */
		if (intc->config_map.ch_to_host[i] != -1) {
			dev_err(dev, "channel %d (req. intr_no %d) already assigned to intr_no %d\n",
				i, host, intc->config_map.ch_to_host[i]);
			ret = -EEXIST;
			goto unlock;
		}

		/* check if host intr is already in use by other PRU */
		if (pruss->host_mask & (1U << host)) {
			dev_err(dev, "%s: host intr %d already in use\n",
				__func__, host);
			ret = -EEXIST;
			goto unlock;
		}

		intc->config_map.ch_to_host[i] = host;

		idx = i / 4;

		val = pruss_intc_read_reg(intc, PRU_INTC_HMR(idx));
		val |= host << ((i & 3) * 8);
		pruss_intc_write_reg(intc, PRU_INTC_HMR(idx), val);

		ch_mask |= BIT(i);
		host_mask |= BIT(host);

		dev_dbg(dev, "CH%d -> HOST%d (HMR%d 0x%08x)\n", i, host, idx,
			pruss_intc_read_reg(intc, PRU_INTC_HMR(idx)));
	}

	dev_info(dev, "configured system_events = 0x%016llx intr_channels = 0x%08x host_intr = 0x%08x\n",
		 sysevt_mask, ch_mask, host_mask);

	/* enable system events, writing 0 has no-effect */
	pruss_intc_write_reg(intc, PRU_INTC_ESR0, lower_32_bits(sysevt_mask));
	pruss_intc_write_reg(intc, PRU_INTC_SECR0, lower_32_bits(sysevt_mask));
	pruss_intc_write_reg(intc, PRU_INTC_ESR1, upper_32_bits(sysevt_mask));
	pruss_intc_write_reg(intc, PRU_INTC_SECR1, upper_32_bits(sysevt_mask));

	/* enable host interrupts */
	for (i = 0; i < MAX_PRU_HOST_INT; i++) {
		if (host_mask & BIT(i))
			pruss_intc_write_reg(intc, PRU_INTC_HIEISR, i);
	}

	/* global interrupt enable */
	pruss_intc_write_reg(intc, PRU_INTC_GER, 1);

	pruss->host_mask |= host_mask;

	mutex_unlock(&intc->lock);
	return 0;

unlock:
	mutex_unlock(&intc->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(pruss_intc_configure);

/**
 * pruss_intc_unconfigure() - unconfigure the PRUSS INTC
 * @pruss: the pruss instance
 * @intc_config: PRU core specific INTC configuration
 *
 * Undo whatever was done in pruss_intc_configure() for a PRU core.
 * It should be sufficient to just mark the resources free in the
 * global map and disable the host interrupts and sysevents.
 */
int pruss_intc_unconfigure(struct pruss *pruss,
			   struct pruss_intc_config *intc_config)
{
	struct device *dev = pruss->dev;
	struct pruss_intc *intc = to_pruss_intc(pruss);
	int i, ch, host;
	u64 sysevt_mask = 0;
	u32 host_mask = 0;

	if (!intc)
		return -EINVAL;

	mutex_lock(&intc->lock);

	for (i = 0; i < ARRAY_SIZE(intc_config->sysev_to_ch); i++) {
		ch = intc_config->sysev_to_ch[i];
		if (ch < 0)
			continue;

		/* mark sysevent free in global map */
		intc->config_map.sysev_to_ch[i] = -1;
		sysevt_mask |= BIT_ULL(i);
	}

	for (i = 0; i < ARRAY_SIZE(intc_config->ch_to_host); i++) {
		host = intc_config->ch_to_host[i];
		if (host < 0)
			continue;

		/* mark channel free in global map */
		intc->config_map.ch_to_host[i] = -1;
		host_mask |= BIT(host);
	}

	dev_info(dev, "unconfigured system_events = 0x%016llx host_intr = 0x%08x\n",
		 sysevt_mask, host_mask);

	/* disable system events, writing 0 has no-effect */
	pruss_intc_write_reg(intc, PRU_INTC_ECR0, lower_32_bits(sysevt_mask));
	pruss_intc_write_reg(intc, PRU_INTC_ECR1, upper_32_bits(sysevt_mask));
	/* clear any pending status */
	pruss_intc_write_reg(intc, PRU_INTC_SECR0, lower_32_bits(sysevt_mask));
	pruss_intc_write_reg(intc, PRU_INTC_SECR1, upper_32_bits(sysevt_mask));

	/* disable host interrupts */
	for (i = 0; i < MAX_PRU_HOST_INT; i++) {
		if (host_mask & BIT(i))
			pruss_intc_write_reg(intc, PRU_INTC_HIDISR, i);
	}

	pruss->host_mask &= ~host_mask;
	mutex_unlock(&intc->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(pruss_intc_unconfigure);

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

static int pruss_intc_irq_retrigger(struct irq_data *data)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);
	unsigned int hwirq = data->hwirq;

	return pruss_intc_check_write(intc, PRU_INTC_SISR, hwirq);
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

/**
 * pruss_intc_trigger() - trigger a PRU system event
 * @irq: linux IRQ number associated with a PRU system event
 *
 * Trigger an interrupt by signalling a specific PRU system event.
 * This can be used by PRUSS client users to raise/send an event to
 * a PRU or any other core that is listening on the host interrupt
 * mapped to that specific PRU system event. The @irq variable is the
 * Linux IRQ number associated with a specific PRU system event that
 * a client user/application uses. The interrupt mappings for this is
 * provided by the PRUSS INTC irqchip instance.
 *
 * Returns 0 on success, or an error value upon failure.
 */
int pruss_intc_trigger(unsigned int irq)
{
	struct irq_desc *desc;

	if (irq <= 0)
		return -EINVAL;

	desc = irq_to_desc(irq);
	if (!desc)
		return -EINVAL;

	pruss_intc_irq_retrigger(&desc->irq_data);

	return 0;
}
EXPORT_SYMBOL_GPL(pruss_intc_trigger);

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
	hipir = pruss_intc_read_reg(intc, PRU_INTC_HIPIR(i));
	while (!(hipir & BIT(31))) {
		hwirq = hipir & GENMASK(9, 0);
		virq = irq_linear_revmap(intc->domain, hwirq);

		/*
		 * XXX: manually ACK any system events that do not have a
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

static const struct of_device_id pruss_intc_of_match[];

static int pruss_intc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *ppdev = to_platform_device(dev->parent);
	struct pruss_intc *intc;
	struct resource *res;
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

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "intc");
	intc->mem.va = devm_ioremap_resource(dev, res);
	if (IS_ERR(intc->mem.va)) {
		dev_err(dev, "failed to parse and map intc memory resource\n");
		return PTR_ERR(intc->mem.va);
	}
	intc->mem.pa = res->start;
	intc->mem.size = resource_size(res);

	dev_dbg(dev, "intc memory: pa %pa size 0x%x va %p\n", &intc->mem.pa,
		intc->mem.size, intc->mem.va);

	mutex_init(&intc->lock);

	for (i = 0; i < ARRAY_SIZE(intc->config_map.sysev_to_ch); i++)
		intc->config_map.sysev_to_ch[i] = -1;

	for (i = 0; i < ARRAY_SIZE(intc->config_map.ch_to_host); i++)
		intc->config_map.ch_to_host[i] = -1;

	intc->pruss = platform_get_drvdata(ppdev);
	pruss_intc_init(intc);

	irqchip = devm_kzalloc(dev, sizeof(*irqchip), GFP_KERNEL);
	if (!irqchip)
		return -ENOMEM;

	irqchip->irq_ack = pruss_intc_irq_ack;
	irqchip->irq_mask = pruss_intc_irq_mask;
	irqchip->irq_unmask = pruss_intc_irq_unmask;
	irqchip->irq_retrigger = pruss_intc_irq_retrigger;
	irqchip->irq_request_resources = pruss_intc_irq_reqres;
	irqchip->irq_release_resources = pruss_intc_irq_relres;
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

static struct pruss_intc_match_data k2g_pruss_intc_data = {
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
	{
		.compatible = "ti,k2g-pruss-intc",
		.data = &k2g_pruss_intc_data,
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
