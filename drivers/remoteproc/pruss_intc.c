// SPDX-License-Identifier: GPL-2.0
/*
 * PRU-ICSS INTC IRQChip driver for various TI SoCs
 *
 * Copyright (C) 2016-2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 *	Suman Anna <s-anna@ti.com>
 */

#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pruss.h>

#include "pruss.h"

/*
 * Number of host interrupts reaching the main MPU sub-system. Note that this
 * is not the same as the total number of host interrupts supported by the PRUSS
 * INTC instance
 */
#define MAX_HOST_NUM_IRQS	8

static const char * const irq_names[] = {
	"host2", "host3", "host4", "host5", "host6", "host7", "host8", "host9",
};

/**
 * struct pruss_intc_match_data - match data to handle SoC variations
 * @num_system_events: number of input system events handled by the PRUSS INTC
 * @num_host_intrs: number of host interrupts supported by the PRUSS INTC
 * @no_host7_intr: flag denoting the absence of host7 interrupt into MPU
 */
struct pruss_intc_match_data {
	u8 num_system_events;
	u8 num_host_intrs;
	bool no_host7_intr;
};

/**
 * struct pruss_intc - PRUSS interrupt controller structure
 * @pruss: back-reference to parent PRUSS structure
 * @irqs: kernel irq numbers corresponding to PRUSS host interrupts
 * @mem: kernel-mapping data for the INTC register space
 * @irqchip: irq chip for this interrupt controller
 * @domain: irq domain for this interrupt controller
 * @data: cached PRUSS INTC IP configuration data
 * @config_map: stored INTC configuration mapping data
 * @lock: mutex to serialize access to INTC
 */
struct pruss_intc {
	struct pruss *pruss;
	unsigned int irqs[MAX_HOST_NUM_IRQS];
	struct pruss_mem_region mem;
	struct irq_chip *irqchip;
	struct irq_domain *domain;
	const struct pruss_intc_match_data *data;
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

	if (sysevent >= intc->data->num_system_events)
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
	int i, idx, ch, host;
	u32 num_events = intc->data->num_system_events;
	u32 num_intrs = intc->data->num_host_intrs;
	u32 num_regs = DIV_ROUND_UP(num_events, 32);
	u32 *sysevt_mask = NULL;
	u32 ch_mask = 0;
	u32 host_mask = 0;
	int ret = 0;
	u32 val;

	if (!intc)
		return -EINVAL;

	sysevt_mask = kcalloc(num_regs, sizeof(*sysevt_mask), GFP_KERNEL);
	if (!sysevt_mask)
		return -ENOMEM;

	mutex_lock(&intc->lock);

	/*
	 * configure channel map registers - each register holds map info
	 * for 4 events, with each event occupying the lower nibble in
	 * a register byte address in little-endian fashion
	 */
	for (i = 0; i < num_events; i++) {
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
		sysevt_mask[i / 32] |= BIT(i % 32);
		ch_mask |= BIT(ch);

		dev_dbg(dev, "SYSEV%d -> CH%d (CMR%d 0x%08x)\n", i, ch, idx,
			pruss_intc_read_reg(intc, PRU_INTC_CMR(idx)));
	}

	/*
	 * set host map registers - each register holds map info for
	 * 4 channels, with each channel occupying the lower nibble in
	 * a register byte address in little-endian fashion
	 */
	for (i = 0; i < num_intrs; i++) {
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

	if (num_events == MAX_PRU_SYS_EVENTS) {
		dev_info(dev, "configured system_events[63-0] = 0x%08x.%08x",
			 sysevt_mask[1], sysevt_mask[0]);
	} else if (num_events == MAX_PRU_SYS_EVENTS_K3) {
		dev_info(dev, "configured system_events[159-0] = 0x%08x.%08x.%08x.%08x.%08x",
			 sysevt_mask[4], sysevt_mask[3],  sysevt_mask[2],
			 sysevt_mask[1],  sysevt_mask[0]);
	}
	dev_info(dev, "configured intr_channels = 0x%08x host_intr = 0x%08x\n",
		 ch_mask, host_mask);

	/* enable system events, writing 0 has no-effect */
	for (i = 0; i < num_regs; i++) {
		pruss_intc_write_reg(intc, PRU_INTC_ESR(i), sysevt_mask[i]);
		pruss_intc_write_reg(intc, PRU_INTC_SECR(i), sysevt_mask[i]);
	}

	/* enable host interrupts */
	for (i = 0; i < num_intrs; i++) {
		if (host_mask & BIT(i))
			pruss_intc_write_reg(intc, PRU_INTC_HIEISR, i);
	}

	/* global interrupt enable */
	pruss_intc_write_reg(intc, PRU_INTC_GER, 1);

	pruss->host_mask |= host_mask;

unlock:
	mutex_unlock(&intc->lock);
	kfree(sysevt_mask);
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
	u32 num_events = intc->data->num_system_events;
	u32 num_intrs = intc->data->num_host_intrs;
	u32 num_regs = DIV_ROUND_UP(num_events, 32);
	u32 *sysevt_mask = NULL;
	u32 host_mask = 0;

	if (!intc)
		return -EINVAL;

	sysevt_mask = kcalloc(num_regs, sizeof(*sysevt_mask), GFP_KERNEL);
	if (!sysevt_mask)
		return -ENOMEM;

	mutex_lock(&intc->lock);

	for (i = 0; i < num_events; i++) {
		ch = intc_config->sysev_to_ch[i];
		if (ch < 0)
			continue;

		/* mark sysevent free in global map */
		intc->config_map.sysev_to_ch[i] = -1;
		sysevt_mask[i / 32] |= BIT(i % 32);
	}

	for (i = 0; i < num_intrs; i++) {
		host = intc_config->ch_to_host[i];
		if (host < 0)
			continue;

		/* mark channel free in global map */
		intc->config_map.ch_to_host[i] = -1;
		host_mask |= BIT(host);
	}

	if (num_events == MAX_PRU_SYS_EVENTS) {
		dev_info(dev, "unconfigured system_events[63-0] = 0x%08x.%08x",
			 sysevt_mask[1], sysevt_mask[0]);
	} else if (num_events == MAX_PRU_SYS_EVENTS_K3) {
		dev_info(dev, "unconfigured system_events[159-0] = 0x%08x.%08x.%08x.%08x.%08x",
			 sysevt_mask[4], sysevt_mask[3],  sysevt_mask[2],
			 sysevt_mask[1],  sysevt_mask[0]);
	}
	dev_info(dev, "unconfigured host_intr = 0x%08x\n", host_mask);

	for (i = 0; i < num_regs; i++) {
		/* disable system events, writing 0 has no-effect */
		pruss_intc_write_reg(intc, PRU_INTC_ECR(i), sysevt_mask[i]);
		/* clear any pending status */
		pruss_intc_write_reg(intc, PRU_INTC_SECR(i), sysevt_mask[i]);
	}

	/* disable host interrupts */
	for (i = 0; i < num_intrs; i++) {
		if (host_mask & BIT(i))
			pruss_intc_write_reg(intc, PRU_INTC_HIDISR, i);
	}

	pruss->host_mask &= ~host_mask;
	mutex_unlock(&intc->lock);
	kfree(sysevt_mask);

	return 0;
}
EXPORT_SYMBOL_GPL(pruss_intc_unconfigure);

static void pruss_intc_init(struct pruss_intc *intc)
{
	int i;
	int num_chnl_map_regs = DIV_ROUND_UP(intc->data->num_system_events, 4);
	int num_host_intr_regs = DIV_ROUND_UP(intc->data->num_host_intrs, 4);
	int num_event_type_regs =
			DIV_ROUND_UP(intc->data->num_system_events, 32);

	/*
	 * configure polarity (SIPR register) to active high and
	 * type (SITR register) to pulse interrupt for all system events
	 */
	for (i = 0; i < num_event_type_regs; i++) {
		pruss_intc_write_reg(intc, PRU_INTC_SIPR(i), 0xffffffff);
		pruss_intc_write_reg(intc, PRU_INTC_SITR(i), 0);
	}

	/* clear all interrupt channel map registers, 4 events per register */
	for (i = 0; i < num_chnl_map_regs; i++)
		pruss_intc_write_reg(intc, PRU_INTC_CMR(i), 0);

	/* clear all host interrupt map registers, 4 channels per register */
	for (i = 0; i < num_host_intr_regs; i++)
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
	u8 max_system_events;

	data = of_device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	skip_host7 = data->no_host7_intr;
	max_system_events = data->num_system_events;

	intc = devm_kzalloc(dev, sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return -ENOMEM;
	intc->data = data;
	platform_set_drvdata(pdev, intc);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "intc");
	intc->mem.va = devm_ioremap_resource(dev, res);
	if (IS_ERR(intc->mem.va)) {
		dev_err(dev, "failed to parse and map intc memory resource\n");
		return PTR_ERR(intc->mem.va);
	}
	intc->mem.pa = res->start;
	intc->mem.size = resource_size(res);

	dev_dbg(dev, "intc memory: pa %pa size 0x%zx va %p\n", &intc->mem.pa,
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

	intc->domain = irq_domain_add_linear(dev->of_node, max_system_events,
					     &pruss_intc_irq_domain_ops, intc);
	if (!intc->domain)
		return -ENOMEM;

	for (i = 0; i < MAX_HOST_NUM_IRQS; i++) {
		irq = platform_get_irq_byname(ppdev, irq_names[i]);
		if (irq < 0) {
			if (!strcmp(irq_names[i], "host7") && !!skip_host7)
				continue;

			dev_err(dev->parent, "platform_get_irq_byname failed for %s : %d\n",
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
	u8 max_system_events = intc->data->num_system_events;
	unsigned int hwirq;

	if (intc->domain) {
		for (hwirq = 0; hwirq < max_system_events; hwirq++)
			irq_dispose_mapping(irq_find_mapping(intc->domain,
							     hwirq));
		irq_domain_remove(intc->domain);
	}

	return 0;
}

static const struct pruss_intc_match_data am335x_am57xx_pruss_intc_data = {
	.num_system_events = 64,
	.num_host_intrs = 10,
	.no_host7_intr = false,
};

static const struct pruss_intc_match_data am437x_k2g_pruss_intc_data = {
	.num_system_events = 64,
	.num_host_intrs = 10,
	.no_host7_intr = true,
};

static const struct pruss_intc_match_data am6x_icssg_intc_data = {
	.num_system_events = 160,
	.num_host_intrs = 20,
	.no_host7_intr = false,
};

static const struct of_device_id pruss_intc_of_match[] = {
	{
		.compatible = "ti,am3356-pruss-intc",
		.data = &am335x_am57xx_pruss_intc_data,
	},
	{
		.compatible = "ti,am4376-pruss-intc",
		.data = &am437x_k2g_pruss_intc_data,
	},
	{
		.compatible = "ti,am5728-pruss-intc",
		.data = &am335x_am57xx_pruss_intc_data,
	},
	{
		.compatible = "ti,k2g-pruss-intc",
		.data = &am437x_k2g_pruss_intc_data,
	},
	{
		.compatible = "ti,am654-icssg-intc",
		.data = &am6x_icssg_intc_data,
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
MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_DESCRIPTION("PRU-ICSS INTC Driver");
MODULE_LICENSE("GPL v2");
