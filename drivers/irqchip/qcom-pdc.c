// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/soc/qcom/irq.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <linux/qcom_scm.h>

#define PDC_MAX_IRQS		168
#define PDC_MAX_GPIO_IRQS	256

#define CLEAR_INTR(reg, intr)	(reg & ~(1 << intr))
#define ENABLE_INTR(reg, intr)	(reg | (1 << intr))

#define IRQ_ENABLE_BANK		0x10
#define IRQ_i_CFG		0x110

#define PDC_NO_PARENT_IRQ	~0UL

struct pdc_pin_region {
	u32 pin_base;
	u32 parent_base;
	u32 cnt;
};

struct spi_cfg_regs {
	union {
		u64 start;
		void __iomem *base;
	};
	resource_size_t size;
	bool scm_io;
};

static DEFINE_RAW_SPINLOCK(pdc_lock);
static void __iomem *pdc_base;
static struct pdc_pin_region *pdc_region;
static int pdc_region_cnt;
static struct spi_cfg_regs *spi_cfg;

static void pdc_reg_write(int reg, u32 i, u32 val)
{
	writel_relaxed(val, pdc_base + reg + i * sizeof(u32));
}

static u32 pdc_reg_read(int reg, u32 i)
{
	return readl_relaxed(pdc_base + reg + i * sizeof(u32));
}

static void pdc_enable_intr(struct irq_data *d, bool on)
{
	int pin_out = d->hwirq;
	unsigned long flags;
	u32 index, mask;
	u32 enable;

	index = pin_out / 32;
	mask = pin_out % 32;

	raw_spin_lock_irqsave(&pdc_lock, flags);
	enable = pdc_reg_read(IRQ_ENABLE_BANK, index);
	enable = on ? ENABLE_INTR(enable, mask) : CLEAR_INTR(enable, mask);
	pdc_reg_write(IRQ_ENABLE_BANK, index, enable);
	raw_spin_unlock_irqrestore(&pdc_lock, flags);
}

static void qcom_pdc_gic_disable(struct irq_data *d)
{
	pdc_enable_intr(d, false);
	irq_chip_disable_parent(d);
}

static void qcom_pdc_gic_enable(struct irq_data *d)
{
	pdc_enable_intr(d, true);
	irq_chip_enable_parent(d);
}

static u32 __spi_pin_read(unsigned int pin)
{
	void __iomem *cfg_reg = spi_cfg->base + pin * 4;
	u64 scm_cfg_reg = spi_cfg->start + pin * 4;

	if (spi_cfg->scm_io) {
		unsigned int val;

		qcom_scm_io_readl(scm_cfg_reg, &val);
		return val;
	} else {
		return readl(cfg_reg);
	}
}

static void __spi_pin_write(unsigned int pin, unsigned int val)
{
	void __iomem *cfg_reg = spi_cfg->base + pin * 4;
	u64 scm_cfg_reg = spi_cfg->start + pin * 4;

	if (spi_cfg->scm_io)
		qcom_scm_io_writel(scm_cfg_reg, val);
	else
		writel(val, cfg_reg);
}

static int spi_configure_type(irq_hw_number_t hwirq, unsigned int type)
{
	int spi = hwirq - 32;
	u32 pin = spi / 32;
	u32 mask = BIT(spi % 32);
	u32 val;
	unsigned long flags;

	if (!spi_cfg)
		return 0;

	if (pin * 4 > spi_cfg->size)
		return -EFAULT;

	raw_spin_lock_irqsave(&pdc_lock, flags);
	val = __spi_pin_read(pin);
	val &= ~mask;
	if (type & IRQ_TYPE_LEVEL_MASK)
		val |= mask;
	__spi_pin_write(pin, val);
	raw_spin_unlock_irqrestore(&pdc_lock, flags);

	return 0;
}

/*
 * GIC does not handle falling edge or active low. To allow falling edge and
 * active low interrupts to be handled at GIC, PDC has an inverter that inverts
 * falling edge into a rising edge and active low into an active high.
 * For the inverter to work, the polarity bit in the IRQ_CONFIG register has to
 * set as per the table below.
 * Level sensitive active low    LOW
 * Rising edge sensitive         NOT USED
 * Falling edge sensitive        LOW
 * Dual Edge sensitive           NOT USED
 * Level sensitive active High   HIGH
 * Falling Edge sensitive        NOT USED
 * Rising edge sensitive         HIGH
 * Dual Edge sensitive           HIGH
 */
enum pdc_irq_config_bits {
	PDC_LEVEL_LOW		= 0b000,
	PDC_EDGE_FALLING	= 0b010,
	PDC_LEVEL_HIGH		= 0b100,
	PDC_EDGE_RISING		= 0b110,
	PDC_EDGE_DUAL		= 0b111,
};

/**
 * qcom_pdc_gic_set_type: Configure PDC for the interrupt
 *
 * @d: the interrupt data
 * @type: the interrupt type
 *
 * If @type is edge triggered, forward that as Rising edge as PDC
 * takes care of converting falling edge to rising edge signal
 * If @type is level, then forward that as level high as PDC
 * takes care of converting falling edge to rising edge signal
 */
static int qcom_pdc_gic_set_type(struct irq_data *d, unsigned int type)
{
	int parent_hwirq = d->parent_data->hwirq;
	enum pdc_irq_config_bits pdc_type;
	enum pdc_irq_config_bits old_pdc_type;
	int ret;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		pdc_type = PDC_EDGE_RISING;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		pdc_type = PDC_EDGE_FALLING;
		type = IRQ_TYPE_EDGE_RISING;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		pdc_type = PDC_EDGE_DUAL;
		type = IRQ_TYPE_EDGE_RISING;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		pdc_type = PDC_LEVEL_HIGH;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		pdc_type = PDC_LEVEL_LOW;
		type = IRQ_TYPE_LEVEL_HIGH;
		break;
	default:
		WARN_ON(1);
		return -EINVAL;
	}

	old_pdc_type = pdc_reg_read(IRQ_i_CFG, d->hwirq);
	pdc_reg_write(IRQ_i_CFG, d->hwirq, pdc_type);

	/* Additionally, configure (only) the GPIO in the f/w */
	ret = spi_configure_type(parent_hwirq, type);
	if (ret)
		return ret;

	ret = irq_chip_set_type_parent(d, type);
	if (ret)
		return ret;

	/*
	 * When we change types the PDC can give a phantom interrupt.
	 * Clear it.  Specifically the phantom shows up when reconfiguring
	 * polarity of interrupt without changing the state of the signal
	 * but let's be consistent and clear it always.
	 *
	 * Doing this works because we have IRQCHIP_SET_TYPE_MASKED so the
	 * interrupt will be cleared before the rest of the system sees it.
	 */
	if (old_pdc_type != pdc_type)
		irq_chip_set_parent_state(d, IRQCHIP_STATE_PENDING, false);

	return 0;
}

static struct irq_chip qcom_pdc_gic_chip = {
	.name			= "PDC",
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_disable		= qcom_pdc_gic_disable,
	.irq_enable		= qcom_pdc_gic_enable,
	.irq_get_irqchip_state	= irq_chip_get_parent_state,
	.irq_set_irqchip_state	= irq_chip_set_parent_state,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_type		= qcom_pdc_gic_set_type,
	.flags			= IRQCHIP_MASK_ON_SUSPEND |
				  IRQCHIP_SET_TYPE_MASKED |
				  IRQCHIP_SKIP_SET_WAKE |
				  IRQCHIP_ENABLE_WAKEUP_ON_SUSPEND,
	.irq_set_vcpu_affinity	= irq_chip_set_vcpu_affinity_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
};

static irq_hw_number_t get_parent_hwirq(int pin)
{
	int i;
	struct pdc_pin_region *region;

	for (i = 0; i < pdc_region_cnt; i++) {
		region = &pdc_region[i];
		if (pin >= region->pin_base &&
		    pin < region->pin_base + region->cnt)
			return (region->parent_base + pin - region->pin_base);
	}

	return PDC_NO_PARENT_IRQ;
}

static int qcom_pdc_translate(struct irq_domain *d, struct irq_fwspec *fwspec,
			      unsigned long *hwirq, unsigned int *type)
{
	if (is_of_node(fwspec->fwnode)) {
		if (fwspec->param_count != 2)
			return -EINVAL;

		*hwirq = fwspec->param[0];
		*type = fwspec->param[1] & IRQ_TYPE_SENSE_MASK;
		return 0;
	}

	return -EINVAL;
}

static int qcom_pdc_alloc(struct irq_domain *domain, unsigned int virq,
			  unsigned int nr_irqs, void *data)
{
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec parent_fwspec;
	irq_hw_number_t hwirq, parent_hwirq;
	unsigned int type;
	int ret;

	ret = qcom_pdc_translate(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	ret  = irq_domain_set_hwirq_and_chip(domain, virq, hwirq,
					     &qcom_pdc_gic_chip, NULL);
	if (ret)
		return ret;

	parent_hwirq = get_parent_hwirq(hwirq);
	if (parent_hwirq == PDC_NO_PARENT_IRQ)
		return irq_domain_disconnect_hierarchy(domain->parent, virq);

	if (type & IRQ_TYPE_EDGE_BOTH)
		type = IRQ_TYPE_EDGE_RISING;

	if (type & IRQ_TYPE_LEVEL_MASK)
		type = IRQ_TYPE_LEVEL_HIGH;

	parent_fwspec.fwnode      = domain->parent->fwnode;
	parent_fwspec.param_count = 3;
	parent_fwspec.param[0]    = 0;
	parent_fwspec.param[1]    = parent_hwirq;
	parent_fwspec.param[2]    = type;

	return irq_domain_alloc_irqs_parent(domain, virq, nr_irqs,
					    &parent_fwspec);
}

static const struct irq_domain_ops qcom_pdc_ops = {
	.translate	= qcom_pdc_translate,
	.alloc		= qcom_pdc_alloc,
	.free		= irq_domain_free_irqs_common,
};

static int qcom_pdc_gpio_alloc(struct irq_domain *domain, unsigned int virq,
			       unsigned int nr_irqs, void *data)
{
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec parent_fwspec;
	irq_hw_number_t hwirq, parent_hwirq;
	unsigned int type;
	int ret;

	ret = qcom_pdc_translate(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	if (hwirq == GPIO_NO_WAKE_IRQ)
		return irq_domain_disconnect_hierarchy(domain, virq);

	ret = irq_domain_set_hwirq_and_chip(domain, virq, hwirq,
					    &qcom_pdc_gic_chip, NULL);
	if (ret)
		return ret;

	parent_hwirq = get_parent_hwirq(hwirq);
	if (parent_hwirq == PDC_NO_PARENT_IRQ)
		return irq_domain_disconnect_hierarchy(domain->parent, virq);

	if (type & IRQ_TYPE_EDGE_BOTH)
		type = IRQ_TYPE_EDGE_RISING;

	if (type & IRQ_TYPE_LEVEL_MASK)
		type = IRQ_TYPE_LEVEL_HIGH;

	parent_fwspec.fwnode      = domain->parent->fwnode;
	parent_fwspec.param_count = 3;
	parent_fwspec.param[0]    = 0;
	parent_fwspec.param[1]    = parent_hwirq;
	parent_fwspec.param[2]    = type;

	return irq_domain_alloc_irqs_parent(domain, virq, nr_irqs,
					    &parent_fwspec);
}

static int qcom_pdc_gpio_domain_select(struct irq_domain *d,
				       struct irq_fwspec *fwspec,
				       enum irq_domain_bus_token bus_token)
{
	return bus_token == DOMAIN_BUS_WAKEUP;
}

static const struct irq_domain_ops qcom_pdc_gpio_ops = {
	.select		= qcom_pdc_gpio_domain_select,
	.alloc		= qcom_pdc_gpio_alloc,
	.free		= irq_domain_free_irqs_common,
};

static int pdc_setup_pin_mapping(struct device_node *np)
{
	int ret, n, i;
	u32 irq_index, reg_index, val;

	n = of_property_count_elems_of_size(np, "qcom,pdc-ranges", sizeof(u32));
	if (n <= 0 || n % 3)
		return -EINVAL;

	pdc_region_cnt = n / 3;
	pdc_region = kcalloc(pdc_region_cnt, sizeof(*pdc_region), GFP_KERNEL);
	if (!pdc_region) {
		pdc_region_cnt = 0;
		return -ENOMEM;
	}

	for (n = 0; n < pdc_region_cnt; n++) {
		ret = of_property_read_u32_index(np, "qcom,pdc-ranges",
						 n * 3 + 0,
						 &pdc_region[n].pin_base);
		if (ret)
			return ret;
		ret = of_property_read_u32_index(np, "qcom,pdc-ranges",
						 n * 3 + 1,
						 &pdc_region[n].parent_base);
		if (ret)
			return ret;
		ret = of_property_read_u32_index(np, "qcom,pdc-ranges",
						 n * 3 + 2,
						 &pdc_region[n].cnt);
		if (ret)
			return ret;

		for (i = 0; i < pdc_region[n].cnt; i++) {
			reg_index = (i + pdc_region[n].pin_base) >> 5;
			irq_index = (i + pdc_region[n].pin_base) & 0x1f;
			val = pdc_reg_read(IRQ_ENABLE_BANK, reg_index);
			val &= ~BIT(irq_index);
			pdc_reg_write(IRQ_ENABLE_BANK, reg_index, val);
		}
	}

	return 0;
}

static int qcom_pdc_init(struct device_node *node, struct device_node *parent)
{
	struct irq_domain *parent_domain, *pdc_domain, *pdc_gpio_domain;
	struct resource res;
	int ret;

	pdc_base = of_iomap(node, 0);
	if (!pdc_base) {
		pr_err("%pOF: unable to map PDC registers\n", node);
		return -ENXIO;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("%pOF: unable to find PDC's parent domain\n", node);
		ret = -ENXIO;
		goto fail;
	}

	ret = pdc_setup_pin_mapping(node);
	if (ret) {
		pr_err("%pOF: failed to init PDC pin-hwirq mapping\n", node);
		goto fail;
	}

	pdc_domain = irq_domain_create_hierarchy(parent_domain, 0, PDC_MAX_IRQS,
						 of_fwnode_handle(node),
						 &qcom_pdc_ops, NULL);
	if (!pdc_domain) {
		pr_err("%pOF: GIC domain add failed\n", node);
		ret = -ENOMEM;
		goto fail;
	}

	ret = of_address_to_resource(node, 1, &res);
	if (!ret) {
		spi_cfg = kcalloc(1, sizeof(*spi_cfg), GFP_KERNEL);
		if (!spi_cfg) {
			ret = -ENOMEM;
			goto remove;
		}
		spi_cfg->scm_io = of_find_property(node,
						   "qcom,scm-spi-cfg", NULL);
		spi_cfg->size = resource_size(&res);
		if (spi_cfg->scm_io) {
			spi_cfg->start = res.start;
		} else {
			spi_cfg->base = ioremap(res.start, spi_cfg->size);
			if (!spi_cfg->base) {
				ret = -ENOMEM;
				goto remove;
			}
		}
	}

	pdc_gpio_domain = irq_domain_create_hierarchy(parent_domain,
					IRQ_DOMAIN_FLAG_QCOM_PDC_WAKEUP,
					PDC_MAX_GPIO_IRQS,
					of_fwnode_handle(node),
					&qcom_pdc_gpio_ops, NULL);
	if (!pdc_gpio_domain) {
		pr_err("%pOF: PDC domain add failed for GPIO domain\n", node);
		ret = -ENOMEM;
		goto remove;
	}

	irq_domain_update_bus_token(pdc_gpio_domain, DOMAIN_BUS_WAKEUP);

	return 0;

remove:
	irq_domain_remove(pdc_domain);
	kfree(spi_cfg);
fail:
	kfree(pdc_region);
	iounmap(pdc_base);
	return ret;
}

static int qcom_pdc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *parent = of_irq_find_parent(np);

	return qcom_pdc_init(np, parent);
}

static const struct of_device_id qcom_pdc_match_table[] = {
	{ .compatible = "qcom,pdc" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_pdc_match_table);

static struct platform_driver qcom_pdc_driver = {
	.probe = qcom_pdc_probe,
	.driver = {
		.name = "qcom-pdc",
		.of_match_table = qcom_pdc_match_table,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(qcom_pdc_driver);
MODULE_DESCRIPTION("Qualcomm Technologies, Inc. Power Domain Controller");
MODULE_LICENSE("GPL v2");
