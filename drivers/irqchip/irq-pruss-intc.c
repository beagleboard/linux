// SPDX-License-Identifier: GPL-2.0
/*
 * PRU-ICSS INTC IRQChip driver for various TI SoCs
 *
 * Copyright (C) 2016-2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 *	Suman Anna <s-anna@ti.com>
 */

#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pruss_driver.h>

/*
 * Number of host interrupts reaching the main MPU sub-system. Note that this
 * is not the same as the total number of host interrupts supported by the PRUSS
 * INTC instance
 */
#define MAX_HOST_NUM_IRQS	8

/* PRU_ICSS_INTC registers */
#define PRU_INTC_REVID		0x0000
#define PRU_INTC_CR		0x0004
#define PRU_INTC_GER		0x0010
#define PRU_INTC_GNLR		0x001C
#define PRU_INTC_SISR		0x0020
#define PRU_INTC_SICR		0x0024
#define PRU_INTC_EISR		0x0028
#define PRU_INTC_EICR		0x002C
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
#define PRU_INTC_SIPR0		0x0D00
#define PRU_INTC_SIPR1		0x0D04
#define PRU_INTC_SITR0		0x0D80
#define PRU_INTC_SITR1		0x0D84
#define PRU_INTC_HINLR(x)	(0x1100 + (x) * 4)
#define PRU_INTC_HIER		0x1500

/* HIPIR register bit-fields */
#define INTC_HIPIR_NONE_HINT	0x80000000

static const char * const irq_names[] = {
	"host2", "host3", "host4", "host5", "host6", "host7", "host8", "host9",
};

/**
 * struct pruss_intc - PRUSS interrupt controller structure
 * @pruss: back-reference to parent PRUSS structure
 * @irqs: kernel irq numbers corresponding to PRUSS host interrupts
 * @base: base virtual address of INTC register space
 * @irqchip: irq chip for this interrupt controller
 * @domain: irq domain for this interrupt controller
 * @config_map: stored INTC configuration mapping data
 * @lock: mutex to serialize access to INTC
 * @host_mask: indicate which HOST IRQs are enabled
 */
struct pruss_intc {
	struct pruss *pruss;
	unsigned int irqs[MAX_HOST_NUM_IRQS];
	void __iomem *base;
	struct irq_chip *irqchip;
	struct irq_domain *domain;
	struct pruss_intc_config config_map;
	struct mutex lock; /* PRUSS INTC lock */
	u32 host_mask;
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

static struct pruss_intc *to_pruss_intc(struct pruss *pruss)
{
	struct device_node *parent = pruss->dev->of_node;
	struct device_node *np;
	struct platform_device *pdev;
	struct pruss_intc *intc = NULL;

	np = of_get_child_by_name(parent, "interrupt-controller");
	if (!np) {
		dev_err(pruss->dev, "pruss does not have an interrupt-controller node\n");
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
 * is intended to be used only by the PRU remoteproc driver.
 *
 * Returns 0 on success, or a suitable error code otherwise
 */
int pruss_intc_configure(struct pruss *pruss,
			 struct pruss_intc_config *intc_config)
{
	struct device *dev = pruss->dev;
	struct pruss_intc *intc = to_pruss_intc(pruss);
	int i, idx, ret;
	s8 ch, host;
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
		if (intc->host_mask & (1U << host)) {
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

	intc->host_mask |= host_mask;

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
	int i;
	s8 ch, host;
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

	intc->host_mask &= ~host_mask;
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

static int pruss_intc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *ppdev = to_platform_device(dev->parent);
	struct pruss_intc *intc;
	struct resource *res;
	struct irq_chip *irqchip;
	int i, irq;

	intc = devm_kzalloc(dev, sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return -ENOMEM;
	platform_set_drvdata(pdev, intc);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	intc->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(intc->base)) {
		dev_err(dev, "failed to parse and map intc memory resource\n");
		return PTR_ERR(intc->base);
	}

	dev_dbg(dev, "intc memory: pa %pa size 0x%zx va %pK\n", &res->start,
		(size_t)resource_size(res), intc->base);

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
		irq = platform_get_irq_byname(ppdev, irq_names[i]);
		if (irq < 0) {
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
	unsigned int hwirq;

	if (intc->domain) {
		for (hwirq = 0; hwirq < MAX_PRU_SYS_EVENTS; hwirq++)
			irq_dispose_mapping(irq_find_mapping(intc->domain,
							     hwirq));
		irq_domain_remove(intc->domain);
	}

	return 0;
}

static const struct of_device_id pruss_intc_of_match[] = {
	{ .compatible = "ti,am3356-pruss-intc", },
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
