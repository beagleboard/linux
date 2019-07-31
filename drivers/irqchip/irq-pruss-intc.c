// SPDX-License-Identifier: GPL-2.0-only
/*
 * PRU-ICSS INTC IRQChip driver for various TI SoCs
 *
 * Copyright (C) 2016-2020 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 *	Suman Anna <s-anna@ti.com>
 */

#include <linux/bitmap.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqchip/irq-pruss-intc.h>
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
#define PRU_INTC_SRSR(x)	(0x0200 + (x) * 4)
#define PRU_INTC_SECR(x)	(0x0280 + (x) * 4)
#define PRU_INTC_ESR(x)		(0x0300 + (x) * 4)
#define PRU_INTC_ECR(x)		(0x0380 + (x) * 4)
#define PRU_INTC_CMR(x)		(0x0400 + (x) * 4)
#define PRU_INTC_HMR(x)		(0x0800 + (x) * 4)
#define PRU_INTC_HIPIR(x)	(0x0900 + (x) * 4)
#define PRU_INTC_SIPR(x)	(0x0d00 + (x) * 4)
#define PRU_INTC_SITR(x)	(0x0d80 + (x) * 4)
#define PRU_INTC_HINLR(x)	(0x1100 + (x) * 4)
#define PRU_INTC_HIER		0x1500

/* CMR register bit-field macros */
#define CMR_EVT_MAP_MASK	0xf
#define CMR_EVT_MAP_BITS	8
#define CMR_EVT_PER_REG		4

/* HMR register bit-field macros */
#define HMR_CH_MAP_MASK		0xf
#define HMR_CH_MAP_BITS		8
#define HMR_CH_PER_REG		4

/* HIPIR register bit-fields */
#define INTC_HIPIR_NONE_HINT	0x80000000

/**
 * struct pruss_intc_match_data - match data to handle SoC variations
 * @num_system_events: number of input system events handled by the PRUSS INTC
 * @num_host_intrs: number of host interrupts supported by the PRUSS INTC
 */
struct pruss_intc_match_data {
	u8 num_system_events;
	u8 num_host_intrs;
};

/**
 * struct pruss_intc - PRUSS interrupt controller structure
 * @irqs: kernel irq numbers corresponding to PRUSS host interrupts
 * @base: base virtual address of INTC register space
 * @irqchip: irq chip for this interrupt controller
 * @domain: irq domain for this interrupt controller
 * @data: cached PRUSS INTC IP configuration data
 * @config_map: stored INTC configuration mapping data
 * @lock: mutex to serialize access to INTC
 * @host_mask: indicate which HOST IRQs are enabled
 * @shared_intr: bit-map denoting if the MPU host interrupt is shared
 * @invalid_intr: bit-map denoting if host interrupt is not connected to MPU
 */
struct pruss_intc {
	unsigned int irqs[MAX_NUM_HOST_IRQS];
	void __iomem *base;
	struct irq_chip *irqchip;
	struct irq_domain *domain;
	const struct pruss_intc_match_data *data;
	struct pruss_intc_config config_map;
	struct mutex lock; /* PRUSS INTC lock */
	u32 host_mask;
	u16 shared_intr;
	u16 invalid_intr;
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

	if (sysevent >= intc->data->num_system_events)
		return -EINVAL;

	pruss_intc_write_reg(intc, reg, sysevent);

	return 0;
}

static void pruss_intc_update_cmr(struct pruss_intc *intc, int evt, s8 ch)
{
	u32 idx, val;

	idx = evt / CMR_EVT_PER_REG;
	val = pruss_intc_read_reg(intc, PRU_INTC_CMR(idx));
	val &= ~(CMR_EVT_MAP_MASK <<
		 ((evt % CMR_EVT_PER_REG) * CMR_EVT_MAP_BITS));
	val |= ch << ((evt % CMR_EVT_PER_REG) * CMR_EVT_MAP_BITS);
	pruss_intc_write_reg(intc, PRU_INTC_CMR(idx), val);
}

static void pruss_intc_update_hmr(struct pruss_intc *intc, int ch, s8 host)
{
	u32 idx, val;

	idx = ch / HMR_CH_PER_REG;
	val = pruss_intc_read_reg(intc, PRU_INTC_HMR(idx));
	val &= ~(HMR_CH_MAP_MASK <<
		 ((ch % HMR_CH_PER_REG) * HMR_CH_MAP_BITS));
	val |= host << ((ch % HMR_CH_PER_REG) * HMR_CH_MAP_BITS);
	pruss_intc_write_reg(intc, PRU_INTC_HMR(idx), val);
}

static struct pruss_intc *to_pruss_intc(struct device *pru_dev)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct device *pruss_dev = pru_dev->parent;
	struct pruss_intc *intc = ERR_PTR(-ENODEV);

	np = of_get_child_by_name(pruss_dev->of_node, "interrupt-controller");
	if (!np) {
		dev_err(pruss_dev, "pruss does not have an interrupt-controller node\n");
		return intc;
	}

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		dev_err(pruss_dev, "no associated platform device\n");
		goto out;
	}

	intc = platform_get_drvdata(pdev);
	if (!intc) {
		dev_err(pruss_dev, "pruss intc device probe failed?\n");
		intc = ERR_PTR(-EINVAL);
	}

out:
	of_node_put(np);
	return intc;
}

/**
 * pruss_intc_configure() - configure the PRUSS INTC
 * @dev: pru device pointer
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
int pruss_intc_configure(struct device *dev,
			 struct pruss_intc_config *intc_config)
{
	struct pruss_intc *intc;
	int i, idx;
	s8 ch, host;
	u32 num_events, num_intrs, num_regs;
	unsigned long *sysevt_bitmap;
	u32 *sysevts;
	u32 ch_mask = 0;
	u32 host_mask = 0;
	int ret = 0;

	intc = to_pruss_intc(dev);
	if (IS_ERR(intc))
		return PTR_ERR(intc);

	num_events = intc->data->num_system_events;
	num_intrs = intc->data->num_host_intrs;
	num_regs = DIV_ROUND_UP(num_events, 32);

	sysevt_bitmap = bitmap_zalloc(num_events, GFP_KERNEL);
	if (!sysevt_bitmap)
		return -ENOMEM;
	sysevts = (u32 *)sysevt_bitmap;

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
		if (intc->config_map.sysev_to_ch[i] != PRU_INTC_FREE) {
			dev_err(dev, "event %d (req. channel %d) already assigned to channel %d\n",
				i, ch, intc->config_map.sysev_to_ch[i]);
			ret = -EEXIST;
			goto fail_evt;
		}

		intc->config_map.sysev_to_ch[i] = ch;
		pruss_intc_update_cmr(intc, i, ch);
		bitmap_set(sysevt_bitmap, i, 1);
		ch_mask |= BIT(ch);
		idx = i / CMR_EVT_PER_REG;

		dev_dbg(dev, "SYSEVT%d -> CH%d (CMR%d 0x%08x)\n", i, ch, idx,
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
		if (intc->config_map.ch_to_host[i] != PRU_INTC_FREE) {
			dev_err(dev, "channel %d (req. intr_no %d) already assigned to intr_no %d\n",
				i, host, intc->config_map.ch_to_host[i]);
			ret = -EEXIST;
			goto fail_ch;
		}

		/* check if host intr is already in use by other PRU */
		if (intc->host_mask & (1U << host)) {
			dev_err(dev, "%s: host intr %d already in use\n",
				__func__, host);
			ret = -EEXIST;
			goto fail_ch;
		}

		intc->config_map.ch_to_host[i] = host;
		pruss_intc_update_hmr(intc, i, host);
		ch_mask |= BIT(i);
		host_mask |= BIT(host);
		idx = i / HMR_CH_PER_REG;

		dev_dbg(dev, "CH%d -> HOST%d (HMR%d 0x%08x)\n", i, host, idx,
			pruss_intc_read_reg(intc, PRU_INTC_HMR(idx)));
	}

	dev_info(dev, "configured system_events[%d-0] = %*pb\n",
		 num_events - 1, num_events, sysevt_bitmap);
	dev_info(dev, "configured intr_channels = 0x%08x host_intr = 0x%08x\n",
		 ch_mask, host_mask);

	/* enable system events, writing 0 has no-effect */
	for (i = 0; i < num_regs; i++) {
		pruss_intc_write_reg(intc, PRU_INTC_ESR(i), sysevts[i]);
		pruss_intc_write_reg(intc, PRU_INTC_SECR(i), sysevts[i]);
	}

	/* enable host interrupts */
	for (i = 0; i < num_intrs; i++) {
		if (host_mask & BIT(i))
			pruss_intc_write_reg(intc, PRU_INTC_HIEISR, i);
	}

	/* global interrupt enable */
	pruss_intc_write_reg(intc, PRU_INTC_GER, 1);

	intc->host_mask |= host_mask;
	goto out;

fail_ch:
	while (--i >= 0) {
		if (intc_config->ch_to_host[i] >= 0) {
			intc->config_map.ch_to_host[i] = PRU_INTC_FREE;
			pruss_intc_update_hmr(intc, i, 0);
		}
	}
	i = num_events;
fail_evt:
	while (--i >= 0) {
		if (intc_config->sysev_to_ch[i] >= 0) {
			intc->config_map.sysev_to_ch[i] = PRU_INTC_FREE;
			pruss_intc_update_cmr(intc, i, 0);
		}
	}
out:
	mutex_unlock(&intc->lock);
	bitmap_free(sysevt_bitmap);
	return ret;
}
EXPORT_SYMBOL_GPL(pruss_intc_configure);

/**
 * pruss_intc_unconfigure() - unconfigure the PRUSS INTC
 * @dev: pru device pointer
 * @intc_config: PRU core specific INTC configuration
 *
 * Undo whatever was done in pruss_intc_configure() for a PRU core.
 * It should be sufficient to just mark the resources free in the
 * global map and disable the host interrupts and sysevents.
 */
int pruss_intc_unconfigure(struct device *dev,
			   struct pruss_intc_config *intc_config)
{
	struct pruss_intc *intc;
	int i;
	s8 ch, host;
	u32 num_events, num_intrs, num_regs;
	unsigned long *sysevt_bitmap;
	u32 *sysevts;
	u32 host_mask = 0;

	intc = to_pruss_intc(dev);
	if (IS_ERR(intc))
		return PTR_ERR(intc);

	num_events = intc->data->num_system_events;
	num_intrs = intc->data->num_host_intrs;
	num_regs = DIV_ROUND_UP(num_events, 32);

	sysevt_bitmap = bitmap_zalloc(num_events, GFP_KERNEL);
	if (!sysevt_bitmap)
		return -ENOMEM;
	sysevts = (u32 *)sysevt_bitmap;

	mutex_lock(&intc->lock);

	for (i = 0; i < num_events; i++) {
		ch = intc_config->sysev_to_ch[i];
		if (ch < 0)
			continue;

		/* mark sysevent free in global map */
		intc->config_map.sysev_to_ch[i] = PRU_INTC_FREE;
		bitmap_set(sysevt_bitmap, i, 1);
		/* clear the map using reset value 0 */
		pruss_intc_update_cmr(intc, i, 0);
	}

	for (i = 0; i < num_intrs; i++) {
		host = intc_config->ch_to_host[i];
		if (host < 0)
			continue;

		/* mark channel free in global map */
		intc->config_map.ch_to_host[i] = PRU_INTC_FREE;
		host_mask |= BIT(host);
		/* clear the map using reset value 0 */
		pruss_intc_update_hmr(intc, i, 0);
	}

	dev_info(dev, "unconfigured system_events[%d-0] = %*pb\n",
		 num_events - 1, num_events, sysevt_bitmap);
	dev_info(dev, "unconfigured host_intr = 0x%08x\n", host_mask);

	for (i = 0; i < num_regs; i++) {
		/* disable system events, writing 0 has no-effect */
		pruss_intc_write_reg(intc, PRU_INTC_ECR(i), sysevts[i]);
		/* clear any pending status */
		pruss_intc_write_reg(intc, PRU_INTC_SECR(i), sysevts[i]);
	}

	/* disable host interrupts */
	for (i = 0; i < num_intrs; i++) {
		if (host_mask & BIT(i))
			pruss_intc_write_reg(intc, PRU_INTC_HIDISR, i);
	}

	intc->host_mask &= ~host_mask;
	mutex_unlock(&intc->lock);
	bitmap_free(sysevt_bitmap);

	return 0;
}
EXPORT_SYMBOL_GPL(pruss_intc_unconfigure);

static void pruss_intc_init(struct pruss_intc *intc)
{
	int i;
	int num_chnl_map_regs = DIV_ROUND_UP(intc->data->num_system_events,
					     CMR_EVT_PER_REG);
	int num_host_intr_regs = DIV_ROUND_UP(intc->data->num_host_intrs,
					      HMR_CH_PER_REG);
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

static int pruss_intc_irq_get_irqchip_state(struct irq_data *data,
					    enum irqchip_irq_state which,
					    bool *state)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);
	u32 reg, mask, srsr;

	if (which != IRQCHIP_STATE_PENDING)
		return -EINVAL;

	reg = PRU_INTC_SRSR(data->hwirq / 32);
	mask = BIT(data->hwirq % 32);

	srsr = pruss_intc_read_reg(intc, reg);

	*state = !!(srsr & mask);

	return 0;
}

static int pruss_intc_irq_set_irqchip_state(struct irq_data *data,
					    enum irqchip_irq_state which,
					    bool state)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);

	if (which != IRQCHIP_STATE_PENDING)
		return -EINVAL;

	if (state)
		return pruss_intc_check_write(intc, PRU_INTC_SISR, data->hwirq);

	return pruss_intc_check_write(intc, PRU_INTC_SICR, data->hwirq);
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
	int i, irq, count;
	const struct pruss_intc_match_data *data;
	u8 temp_intr[MAX_NUM_HOST_IRQS] = { 0 };
	u8 max_system_events;

	data = of_device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	max_system_events = data->num_system_events;

	intc = devm_kzalloc(dev, sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return -ENOMEM;
	intc->data = data;
	platform_set_drvdata(pdev, intc);

	intc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(intc->base)) {
		dev_err(dev, "failed to parse and map intc memory resource\n");
		return PTR_ERR(intc->base);
	}

	count = of_property_read_variable_u8_array(dev->of_node,
						   "ti,irqs-reserved",
						   temp_intr, 0,
						   MAX_NUM_HOST_IRQS);
	if (count < 0 && count != -EINVAL)
		return count;
	count = (count == -EINVAL ? 0 : count);
	for (i = 0; i < count; i++) {
		if (temp_intr[i] < MAX_NUM_HOST_IRQS) {
			intc->invalid_intr |= BIT(temp_intr[i]);
		} else {
			dev_warn(dev, "ignoring invalid reserved irq %d\n",
				 temp_intr[i]);
		}
		temp_intr[i] = 0;
	}

	count = of_property_read_variable_u8_array(dev->of_node,
						   "ti,irqs-shared",
						   temp_intr, 0,
						   MAX_NUM_HOST_IRQS);
	if (count < 0 && count != -EINVAL)
		return count;
	count = (count == -EINVAL ? 0 : count);
	for (i = 0; i < count; i++) {
		if (temp_intr[i] < MAX_NUM_HOST_IRQS) {
			intc->shared_intr |= BIT(temp_intr[i]);
		} else {
			dev_warn(dev, "ignoring invalid shared irq %d\n",
				 temp_intr[i]);
		}
	}

	mutex_init(&intc->lock);

	for (i = 0; i < ARRAY_SIZE(intc->config_map.sysev_to_ch); i++)
		intc->config_map.sysev_to_ch[i] = PRU_INTC_FREE;

	for (i = 0; i < ARRAY_SIZE(intc->config_map.ch_to_host); i++)
		intc->config_map.ch_to_host[i] = PRU_INTC_FREE;

	pruss_intc_init(intc);

	irqchip = devm_kzalloc(dev, sizeof(*irqchip), GFP_KERNEL);
	if (!irqchip)
		return -ENOMEM;

	irqchip->irq_ack = pruss_intc_irq_ack;
	irqchip->irq_mask = pruss_intc_irq_mask;
	irqchip->irq_unmask = pruss_intc_irq_unmask;
	irqchip->irq_request_resources = pruss_intc_irq_reqres;
	irqchip->irq_release_resources = pruss_intc_irq_relres;
	irqchip->irq_get_irqchip_state = pruss_intc_irq_get_irqchip_state;
	irqchip->irq_set_irqchip_state = pruss_intc_irq_set_irqchip_state;
	irqchip->name = dev_name(dev);
	intc->irqchip = irqchip;

	intc->domain = irq_domain_add_linear(dev->of_node, max_system_events,
					     &pruss_intc_irq_domain_ops, intc);
	if (!intc->domain)
		return -ENOMEM;

	for (i = 0; i < MAX_NUM_HOST_IRQS; i++) {
		if (intc->invalid_intr & BIT(i))
			continue;

		irq = platform_get_irq_byname(pdev, irq_names[i]);
		if (irq < 0) {
			if (intc->shared_intr & BIT(i))
				continue;

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
	u8 max_system_events = intc->data->num_system_events;
	unsigned int hwirq;
	int i;

	for (i = 0; i < MAX_NUM_HOST_IRQS; i++) {
		if (intc->irqs[i])
			irq_set_chained_handler_and_data(intc->irqs[i], NULL,
							 NULL);
	}

	for (hwirq = 0; hwirq < max_system_events; hwirq++)
		irq_dispose_mapping(irq_find_mapping(intc->domain, hwirq));
	irq_domain_remove(intc->domain);

	return 0;
}

static const struct pruss_intc_match_data pruss_intc_data = {
	.num_system_events = 64,
	.num_host_intrs = 10,
};

static const struct pruss_intc_match_data icssg_intc_data = {
	.num_system_events = 160,
	.num_host_intrs = 20,
};

static const struct of_device_id pruss_intc_of_match[] = {
	{
		.compatible = "ti,pruss-intc",
		.data = &pruss_intc_data,
	},
	{
		.compatible = "ti,icssg-intc",
		.data = &icssg_intc_data,
	},
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
