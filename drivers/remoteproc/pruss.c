/*
 * PRU-ICSS platform driver for various TI SoCs
 *
 * Copyright (C) 2014-2016 Texas Instruments Incorporated - http://www.ti.com/
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

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>
#include <linux/virtio.h>
#include <linux/pruss.h>

#include <linux/platform_data/remoteproc-pruss.h>

#include "remoteproc_internal.h"
#include "pruss.h"

/**
 * struct pruss_private_data - PRUSS driver private data
 * @num_irqs: number of interrupts to MPU
 * @host_events: bit mask of PRU host interrupts that are routed to MPU
 * @aux_data: auxiliary data used for creating the child nodes
 * @has_reset: flag to indicate the presence of global module reset
 */
struct pruss_private_data {
	int num_irqs;
	int host_events;
	struct of_dev_auxdata *aux_data;
	bool has_reset;
};

/**
 * struct pruss_match_private_data - match private data to handle multiple instances
 * @device_name: device name of the PRUSS instance
 * @priv_data: PRUSS driver private data for this PRUSS instance
 */
struct pruss_match_private_data {
	const char *device_name;
	struct pruss_private_data *priv_data;
};

static DEFINE_MUTEX(pruss_list_mutex);
static LIST_HEAD(pruss_list);

/**
 * pruss_get() - get the pruss for the given device
 * @dev: device interested in the pruss
 *
 * Finds the pruss device referenced by the "pruss" property in the
 * requesting (client) device's device node.
 *
 * This function increments the pruss device's refcount, so always
 * use pruss_put() to decrement it back once pruss isn't needed anymore.
 *
 * Returns the pruss handle on success, and NULL on failure.
 */
struct pruss *pruss_get(struct device *dev)
{
	struct pruss *pruss = NULL, *p;
	struct device_node *np;

	if (!dev)
		return NULL;

	np = of_parse_phandle(dev->of_node, "pruss", 0);
	if (!np)
		return NULL;

	mutex_lock(&pruss_list_mutex);
	list_for_each_entry(p, &pruss_list, node) {
		if (p->dev->of_node == np) {
			pruss = p;
			get_device(pruss->dev);
			break;
		}
	}

	mutex_unlock(&pruss_list_mutex);
	of_node_put(np);

	return pruss;
}
EXPORT_SYMBOL_GPL(pruss_get);

/**
 * pruss_put() - decrement pruss device's usecount
 * @pruss: pruss handle
 *
 * Complimentary function for pruss_get(). Needs to be called
 * after the PRUSS is used, and only if the pruss_get() succeeds.
 */
void pruss_put(struct pruss *pruss)
{
	if (!pruss)
		return;

	put_device(pruss->dev);
}
EXPORT_SYMBOL_GPL(pruss_put);

/*
 * get the rproc phandle corresponding to a pru_id.
 * Caller must call rproc_put() when done with rproc.
 */
static struct rproc *__pruss_rproc_get(struct pruss *pruss,
				       enum pruss_pru_id pru_id)
{
	struct device_node *rproc_np;
	struct platform_device *pdev;
	struct rproc *rproc;

	/* get rproc corresponding to pru_id */
	switch (pru_id) {
	case PRUSS_PRU0:
		rproc_np = of_get_child_by_name(pruss->dev->of_node, "pru0");
		break;
	case PRUSS_PRU1:
		rproc_np = of_get_child_by_name(pruss->dev->of_node, "pru1");
		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	if (!rproc_np)
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(rproc_np);
	of_node_put(rproc_np);

	if (!pdev)
		/* probably PRU not yet probed */
		return ERR_PTR(-EPROBE_DEFER);

	rproc = platform_get_drvdata(pdev);
	get_device(&rproc->dev);

	return rproc;
}

/**
 * pruss_rproc_get() - Get the rproc instance corresponding to pru_id
 * @pruss: the pruss instance
 * @pru_id: the PRU id of which we need the rproc instance
 *
 * Allows only one user to own the rproc resource at a time.
 * Caller must call pruss_put_rproc() when done with using the rproc.
 *
 * Returns rproc handle on success. ERR_PTR on failure e.g.
 * -EBUSY if PRU is already reserved by someone else
 * -ENODEV if not yet available.
 * -EINVAL if invalid parameters.
 */
struct rproc *pruss_rproc_get(struct pruss *pruss,
			      enum pruss_pru_id pru_id)
{
	struct rproc *rproc;
	int ret;

	if (!pruss)
		return ERR_PTR(-EINVAL);

	rproc = __pruss_rproc_get(pruss, pru_id);
	if (IS_ERR(rproc))
		return rproc;

	mutex_lock(&pruss->lock);

	if (pruss->pru_in_use[pru_id]) {
		ret = -EBUSY;
		goto unlock;
	}

	pruss->pru_in_use[pru_id] = rproc;

	mutex_unlock(&pruss->lock);

	return rproc;

unlock:
	mutex_unlock(&pruss->lock);
	rproc_put(rproc);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(pruss_rproc_get);

/* find out PRU ID from the rproc instance */
static enum pruss_pru_id pruss_rproc_to_pru_id(struct pruss *pruss,
					       struct rproc *rproc)
{
	enum pruss_pru_id pru_id;

	for (pru_id = PRUSS_PRU0; pru_id < PRUSS_NUM_PRUS; pru_id++)
		if (pruss->pru_in_use[pru_id] == rproc)
			return pru_id;

	return -1;
}

/**
 * pruss_rproc_put() - release the PRU rproc resource
 * @pruss: the pruss instance
 * @rproc: the rproc resource to release
 *
 * Releases the rproc resource and makes it available to other
 * users.
 */
void pruss_rproc_put(struct pruss *pruss, struct rproc *rproc)
{
	int pru_id;

	if (!pruss || !rproc)
		return;

	mutex_lock(&pruss->lock);

	pru_id = pruss_rproc_to_pru_id(pruss, rproc);
	if (pru_id < 0) {
		mutex_unlock(&pruss->lock);
		return;
	}

	if (!pruss->pru_in_use[pru_id]) {
		mutex_unlock(&pruss->lock);
		return;
	}
	pruss->pru_in_use[pru_id] = NULL;

	mutex_unlock(&pruss->lock);

	rproc_put(rproc);
}
EXPORT_SYMBOL(pruss_rproc_put);

/**
 * pruss_rproc_boot() - boot the specified PRU with specified firmware
 * @rproc: the rproc instance of the PRU
 * @fw_name: path of the firmware blob
 *
 * Boot the specified PRU with the firmware blob in the specified path.
 * Shouldn't be called from IRQ context.
 *
 * Returns 0 on success. Non-zero on failure e.g.
 * -EBUSY if PRU is already reserved by someone else
 * -ENODEV if not yet available.
 * -EINVAL if invalid parameters or boot failed.
 */
int pruss_rproc_boot(struct pruss *pruss, struct rproc *rproc,
		     const char *fw_name)
{
	int ret, pru_id;

	if (!pruss || !rproc || !fw_name)
		return -EINVAL;

	mutex_lock(&pruss->lock);

	pru_id = pruss_rproc_to_pru_id(pruss, rproc);
	if (pru_id < 0) {
		ret = -EINVAL;
		goto unlock;
	}

	/* is PRU already running */
	if (pruss->pru_running[pru_id]) {
		ret = -EBUSY;
		goto unlock;
	}

	rproc->firmware = fw_name;
	ret = rproc_boot(rproc);
	if (ret) {
		dev_err(pruss->dev, "rproc_boot failed: %d\n", ret);
		ret = -EINVAL;
		goto unlock;
	}
	pruss->pru_running[pru_id] = true;

unlock:
	mutex_unlock(&pruss->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(pruss_rproc_boot);

/**
 * pruss_rproc_halt() - halt the specified PRU
 * @pruss: the pruss instance
 * @rproc: the rproc instance of the PRU to halt
 *
 * Halt the specified PRU if running. The rproc structure is used
 * here instead of PRU id to avoid unrelated users to halt a PRU
 * that was not booted by them.
 */
void pruss_rproc_halt(struct pruss *pruss, struct rproc *rproc)
{
	int pru_id;

	if (!pruss || !rproc)
		return;

	mutex_lock(&pruss->lock);

	/* Find which pru_id the rproc is */
	pru_id = pruss_rproc_to_pru_id(pruss, rproc);
	if (pru_id < 0)
		goto out;

	if (!pruss->pru_running[pru_id])
		goto out;

	/* halt the processor */
	rproc_shutdown(rproc);
	pruss->pru_running[pru_id] = false;

out:
	mutex_unlock(&pruss->lock);
}
EXPORT_SYMBOL_GPL(pruss_rproc_halt);

/**
 * pruss_request_mem_region() - request a memory resource
 * @pruss: the pruss instance
 * @mem_id: the memory resource id
 * @region: pointer to memory region structure to be filled in
 *
 * This function allows a client driver to requests a memory resource,
 * and if successful, will let the client driver own the particular
 * memory region until released using the pruss_release_mem_region()
 * API.
 *
 * Returns the memory region if requested resource is available, an
 * error otherwise
 */
int pruss_request_mem_region(struct pruss *pruss, enum pruss_mem mem_id,
			     struct pruss_mem_region *region)
{
	if (!pruss || !region)
		return -EINVAL;

	if (mem_id >= PRUSS_MEM_MAX)
		return -EINVAL;

	mutex_lock(&pruss->lock);

	if (pruss->mem_in_use[mem_id]) {
		mutex_unlock(&pruss->lock);
		return -EBUSY;
	}

	*region = pruss->mem_regions[mem_id];
	pruss->mem_in_use[mem_id] = region;

	mutex_unlock(&pruss->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(pruss_request_mem_region);

/**
 * pruss_release_mem_region() - release a memory resource
 * @pruss: the pruss instance
 * @region: the memory region to release
 *
 * This function is the complimentary function to
 * pruss_request_mem_region(), and allows the client drivers to
 * release back a memory resource.
 *
 * Returns 0 on success, an error code otherwise
 */
int pruss_release_mem_region(struct pruss *pruss,
			     struct pruss_mem_region *region)
{
	int id;

	if (!pruss || !region)
		return -EINVAL;

	mutex_lock(&pruss->lock);

	/* find out the memory region being released */
	for (id = 0; id < PRUSS_MEM_MAX; id++) {
		if (pruss->mem_in_use[id] == region)
			break;
	}

	if (id == PRUSS_MEM_MAX) {
		mutex_unlock(&pruss->lock);
		return -EINVAL;
	}

	pruss->mem_in_use[id] = NULL;

	mutex_unlock(&pruss->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(pruss_release_mem_region);

static inline u32 pruss_intc_read_reg(struct pruss *pruss, unsigned int reg)
{
	return readl_relaxed(pruss->mem_regions[PRUSS_MEM_INTC].va + reg);
}

static inline void pruss_intc_write_reg(struct pruss *pruss, unsigned int reg,
					u32 val)
{
	writel_relaxed(val, pruss->mem_regions[PRUSS_MEM_INTC].va + reg);
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
	int i, idx, ch, host, ret;
	u64 sysevt_mask = 0;
	u32 ch_mask = 0;
	u32 host_mask = 0;
	u32 val;

	mutex_lock(&pruss->intc_lock);

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
		if (pruss->intc_config.sysev_to_ch[i] != -1) {
			dev_err(dev, "event %d (req. channel %d) already assigned to channel %d\n",
				i, ch, pruss->intc_config.sysev_to_ch[i]);
			ret = -EEXIST;
			goto unlock;
		}

		pruss->intc_config.sysev_to_ch[i] = ch;

		idx = i / 4;
		val = pruss_intc_read_reg(pruss, PRU_INTC_CMR(idx));
		val |= ch << ((i & 3) * 8);
		pruss_intc_write_reg(pruss, PRU_INTC_CMR(idx), val);

		sysevt_mask |= 1LLU << i;
		ch_mask |= 1U << ch;

		dev_dbg(dev, "SYSEV%d -> CH%d (CMR%d 0x%08x)\n", i, ch, idx,
			pruss_intc_read_reg(pruss, PRU_INTC_CMR(idx)));
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
		if (pruss->intc_config.ch_to_host[i] != -1) {
			dev_err(dev, "channel %d (req. intr_no %d) already assigned to intr_no %d\n",
				i, host, pruss->intc_config.ch_to_host[i]);
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

		pruss->intc_config.ch_to_host[i] = host;

		idx = i / 4;

		val = pruss_intc_read_reg(pruss, PRU_INTC_HMR(idx));
		val |= host << ((i & 3) * 8);
		pruss_intc_write_reg(pruss, PRU_INTC_HMR(idx), val);

		ch_mask |= 1U << i;
		host_mask |= 1U << host;

		dev_dbg(dev, "CH%d -> HOST%d (HMR%d 0x%08x)\n", i, host, idx,
			pruss_intc_read_reg(pruss, PRU_INTC_HMR(idx)));
	}

	dev_info(dev, "configured system_events = 0x%016llx intr_channels = 0x%08x host_intr = 0x%08x\n",
		 sysevt_mask, ch_mask, host_mask);

	/* enable system events, writing 0 has no-effect */
	pruss_intc_write_reg(pruss, PRU_INTC_ESR0, lower_32_bits(sysevt_mask));
	pruss_intc_write_reg(pruss, PRU_INTC_SECR0, lower_32_bits(sysevt_mask));
	pruss_intc_write_reg(pruss, PRU_INTC_ESR1, upper_32_bits(sysevt_mask));
	pruss_intc_write_reg(pruss, PRU_INTC_SECR1, upper_32_bits(sysevt_mask));

	/* enable host interrupts */
	for (i = 0; i < MAX_PRU_HOST_INT; i++) {
		if ((host_mask & (1 << i)))
			pruss_intc_write_reg(pruss, PRU_INTC_HIEISR, i);
	}

	/* global interrupt enable */
	pruss_intc_write_reg(pruss, PRU_INTC_GER, 1);

	pruss->host_mask |= host_mask;

	mutex_unlock(&pruss->intc_lock);
	return 0;

unlock:
	mutex_unlock(&pruss->intc_lock);
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
	int i, ch, host;
	u64 sysevt_mask = 0;
	u32 host_mask = 0;

	mutex_lock(&pruss->intc_lock);

	for (i = 0; i < ARRAY_SIZE(intc_config->sysev_to_ch); i++) {
		ch = intc_config->sysev_to_ch[i];
		if (ch < 0)
			continue;

		/* mark sysevent free in global map */
		pruss->intc_config.sysev_to_ch[i] = -1;
		sysevt_mask |= 1LLU << i;
	}

	for (i = 0; i < ARRAY_SIZE(intc_config->ch_to_host); i++) {
		host = intc_config->ch_to_host[i];
		if (host < 0)
			continue;

		/* mark channel free in global map */
		pruss->intc_config.ch_to_host[i] = -1;
		host_mask |= 1U << host;
	}

	dev_info(dev, "unconfigured system_events = 0x%016llx host_intr = 0x%08x\n",
		 sysevt_mask, host_mask);

	/* disable system events, writing 0 has no-effect */
	pruss_intc_write_reg(pruss, PRU_INTC_ECR0, lower_32_bits(sysevt_mask));
	pruss_intc_write_reg(pruss, PRU_INTC_ECR1, upper_32_bits(sysevt_mask));
	/* clear any pending status */
	pruss_intc_write_reg(pruss, PRU_INTC_SECR0, lower_32_bits(sysevt_mask));
	pruss_intc_write_reg(pruss, PRU_INTC_SECR1, upper_32_bits(sysevt_mask));

	/* disable host interrupts */
	for (i = 0; i < MAX_PRU_HOST_INT; i++) {
		if ((host_mask & (1 << i)))
			pruss_intc_write_reg(pruss, PRU_INTC_HIDISR, i);
	}

	pruss->host_mask &= ~host_mask;
	mutex_unlock(&pruss->intc_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(pruss_intc_unconfigure);

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

/**
 * pruss_intc_sysevent_irqdisable() - disable the INTC sysevent IRQ
 * @pruss: the pruss instance
 * @sysevent: sysevent number
 */
int pruss_intc_sysevent_irqdisable(struct pruss *pruss,
				   unsigned short sysevent)
{
	return pruss_intc_check_write(pruss, PRU_INTC_EICR, sysevent);
}
EXPORT_SYMBOL(pruss_intc_sysevent_irqdisable);

/**
 * pruss_intc_sysevent_irqenable() - enable the INTC sysevent IRQ
 * @pruss: the pruss instance
 * @sysevent: sysevent number
 */
int pruss_intc_sysevent_irqenable(struct pruss *pruss, unsigned short sysevent)
{
	return pruss_intc_check_write(pruss, PRU_INTC_EISR, sysevent);
}
EXPORT_SYMBOL(pruss_intc_sysevent_irqenable);

/**
 * pruss_intc_sysevent_check() - check the system event interrupt
 * @pruss: the pruss instance
 * @sysevent: sysevent number
 *
 * Returns true if sysevent is pending, false otherwise
 */
bool pruss_intc_sysevent_check(struct pruss *pruss, unsigned short sysevent)
{
	u64 reg;

	if (!pruss)
		return false;

	if (sysevent >= MAX_PRU_SYS_EVENTS)
		return false;

	if (sysevent < 32) {
		reg = pruss_intc_read_reg(pruss, PRU_INTC_SRSR0);
	} else {
		reg = pruss_intc_read_reg(pruss, PRU_INTC_SRSR1);
		sysevent -= 32;
	}

	return reg & BIT(sysevent);
}
EXPORT_SYMBOL_GPL(pruss_intc_sysevent_check);

/**
 * pruss_intc_sysevent_clear() - clear the system event interrupt
 * @pruss: the pruss instance
 * @sysevent: sysevent number
 *
 * Returns 0 on success or -EVINVAL on invalid pruss/sysevent.
 */
int pruss_intc_sysevent_clear(struct pruss *pruss, unsigned short sysevent)
{
	return pruss_intc_check_write(pruss, PRU_INTC_SICR, sysevent);
}
EXPORT_SYMBOL_GPL(pruss_intc_sysevent_clear);

/**
 * pruss_host_to_mpu_irq() - Return MPU_IRQ of the given HOST_IRQ
 * @pruss: the pruss instance
 * @host_irq: the host irq number
 *
 * The HOST_IRQ is independent of the SoC but the MPU_IRQ is
 * specific to the SoC. HOST_IRQ 0 and 1 go to the PRU and not the
 * MPU so they don't map to any MPU_IRQ and the function will return
 * error. Not all of the remaining HOST_IRQs reach the MPU on all
 * SoCs.
 *
 * Returns the MPU_IRQ number for the given HOST_IRQ, or an error
 * value upon failure
 */
int pruss_host_to_mpu_irq(struct pruss *pruss, unsigned int host_irq)
{
	int host_events;
	int irq = 0;

	if (!pruss)
		return -EINVAL;

	if (host_irq < MIN_PRU_HOST_INT || host_irq >= MAX_PRU_HOST_INT)
		return -EINVAL;

	/* check whether the interrupt can reach MPU */
	host_events = pruss->data->host_events;
	if (!(host_events & BIT(host_irq)))
		return -EINVAL;

	/* no need to process the first two host interrupts connected to PRU */
	host_events >>= MIN_PRU_HOST_INT;
	host_irq -= MIN_PRU_HOST_INT;
	while (host_irq--) {
		irq += (host_events & BIT(0));
		host_events >>= 1;
	}

	return pruss->irqs[irq];
}
EXPORT_SYMBOL_GPL(pruss_host_to_mpu_irq);

static void pruss_intc_init(struct pruss *pruss)
{
	int i;

	/* configure polarity to active high for all system interrupts */
	pruss_intc_write_reg(pruss, PRU_INTC_SIPR0, 0xffffffff);
	pruss_intc_write_reg(pruss, PRU_INTC_SIPR1, 0xffffffff);

	/* configure type to pulse interrupt for all system interrupts */
	pruss_intc_write_reg(pruss, PRU_INTC_SITR0, 0);
	pruss_intc_write_reg(pruss, PRU_INTC_SITR1, 0);

	/* clear all interrupt channel map registers */
	for (i = PRU_INTC_CMR(0); i <= PRU_INTC_CMR(15); i += 4)
		pruss_intc_write_reg(pruss, i, 0);

	/* clear all host interrupt map registers */
	for (i = PRU_INTC_HMR(0); i <= PRU_INTC_HMR(2); i += 4)
		pruss_intc_write_reg(pruss, i, 0);
}

static inline u32 pruss_read_reg(struct pruss *pruss, enum pruss_mem region,
				 unsigned int reg)
{
	return readl_relaxed(pruss->mem_regions[region].va + reg);
}

static inline void pruss_write_reg(struct pruss *pruss, enum pruss_mem region,
				   unsigned int reg, u32 val)
{
	writel_relaxed(val, pruss->mem_regions[region].va + reg);
}

static inline void pruss_set_reg(struct pruss *pruss, enum pruss_mem region,
				 unsigned int reg, u32 mask, u32 set)
{
	u32 val;

	val = pruss_read_reg(pruss, region, reg);
	val &= ~mask;
	val |= (set & mask);
	pruss_write_reg(pruss, region, reg, val);
}

/* firmware must be idle when calling this function */
static void pruss_disable_module(struct pruss *pruss)
{
	/* configure Smart Standby */
	pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG,
		      PRUSS_SYSCFG_STANDBY_MODE_MASK,
		      PRUSS_SYSCFG_STANDBY_MODE_SMART);

	/* initiate MStandby */
	pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG,
		      PRUSS_SYSCFG_STANDBY_INIT,
		      PRUSS_SYSCFG_STANDBY_INIT);

	/* tell PRCM to initiate IDLE request */
	pm_runtime_put_sync(pruss->dev);
}

/*
 * This function programs the PRUSS_SYSCFG.STANDBY_INIT bit to achieve dual
 * functionalities - one is to deassert the MStandby signal to the device
 * PRCM, and the other is to enable OCP master ports to allow accesses
 * outside of the PRU-ICSS. The function has to wait for the PRCM to
 * acknowledge through the monitoring of the PRUSS_SYSCFG.SUB_MWAIT bit.
 */
static int pruss_enable_ocp_master_ports(struct pruss *pruss)
{
	int i;
	bool ready;

	pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG,
		      PRUSS_SYSCFG_STANDBY_INIT, 0);

	/* wait till we are ready for transactions - delay is arbitrary */
	for (i = 0; i < 10; i++) {
		ready = !(pruss_read_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG)
			  & PRUSS_SYSCFG_SUB_MWAIT_READY);
		if (ready)
			break;
		udelay(10);
	}

	if (!ready) {
		dev_err(pruss->dev, "timeout waiting for SUB_MWAIT_READY\n");
		return -1;
	}

	return 0;
}

static int pruss_enable_module(struct pruss *pruss)
{
	int ret;

	/* tell PRCM to de-assert IDLE request */
	ret = pm_runtime_get_sync(pruss->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(pruss->dev);
		return ret;
	}

	/* configure for smart idle & smart standby */
	pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG,
		      PRUSS_SYSCFG_IDLE_MODE_MASK,
		      PRUSS_SYSCFG_IDLE_MODE_SMART);
	pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG,
		      PRUSS_SYSCFG_STANDBY_MODE_MASK,
		      PRUSS_SYSCFG_STANDBY_MODE_SMART);

	/* enable OCP master ports/disable MStandby */
	ret = pruss_enable_ocp_master_ports(pruss);
	if (ret)
		pruss_disable_module(pruss);

	return ret;
}

/**
 * pruss_cfg_gpimode() - set the GPI mode of the PRU
 * @pruss: the pruss instance handle
 * @rproc: the rproc instance handle of the PRU
 * @mode: GPI mode to set
 *
 * Sets the GPI mode for a given PRU by programming the
 * corresponding PRUSS_CFG_GPCFGx register
 *
 * Returns 0 on success, or an error code otherwise
 */
int pruss_cfg_gpimode(struct pruss *pruss, struct rproc *rproc,
		      enum pruss_gpi_mode mode)
{
	u32 reg;
	int pru_id;

	pru_id = pruss_rproc_to_pru_id(pruss, rproc);
	if (pru_id < 0 || pru_id >= PRUSS_NUM_PRUS) {
		dev_err(pruss->dev, "%s: PRU id not found, %d\n",
			__func__, pru_id);
		return -EINVAL;
	}

	reg = PRUSS_CFG_GPCFG0 + (0x4 * pru_id);

	mutex_lock(&pruss->cfg_lock);
	pruss_set_reg(pruss, PRUSS_MEM_CFG, reg,
		      PRUSS_GPCFG_PRU_GPI_MODE_MASK,
		      mode << PRUSS_GPCFG_PRU_GPI_MODE_SHIFT);
	mutex_unlock(&pruss->cfg_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(pruss_cfg_gpimode);

/**
 * pruss_cfg_miirt_enable() - Enable/disable MII RT Events
 * @pruss: the pruss instance
 * @enable: enable/disable
 *
 * Enable/disable the MII RT Events for the PRUSS.
 */
void pruss_cfg_miirt_enable(struct pruss *pruss, bool enable)
{
	u32 set = enable ? PRUSS_MII_RT_EVENT_EN : 0;

	mutex_lock(&pruss->cfg_lock);
	pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_MII_RT,
		      PRUSS_MII_RT_EVENT_EN, set);
	mutex_unlock(&pruss->cfg_lock);
}
EXPORT_SYMBOL_GPL(pruss_cfg_miirt_enable);

/**
 * pruss_cfg_xfr_enable() - Enable/disable XIN XOUT shift functionality
 * @pruss: the pruss instance
 * @enable: enable/disable
 */
void pruss_cfg_xfr_enable(struct pruss *pruss, bool enable)
{
	u32 set = enable ? PRUSS_SPP_XFER_SHIFT_EN : 0;

	mutex_lock(&pruss->cfg_lock);
	pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SPP,
		      PRUSS_SPP_XFER_SHIFT_EN, set);
	mutex_unlock(&pruss->cfg_lock);
}
EXPORT_SYMBOL_GPL(pruss_cfg_xfr_enable);

#ifdef CONFIG_PM_SLEEP
static int pruss_suspend(struct device *dev)
{
	struct pruss *pruss = dev_get_drvdata(dev);

	/* initiate MStandby, undo the MStandby config in probe */
	pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG,
		      PRUSS_SYSCFG_STANDBY_INIT,
		      PRUSS_SYSCFG_STANDBY_INIT);

	return 0;
}

static int pruss_resume(struct device *dev)
{
	struct pruss *pruss = dev_get_drvdata(dev);
	int ret;

	/* re-enable OCP master ports/disable MStandby */
	ret = pruss_enable_ocp_master_ports(pruss);
	if (ret)
		dev_err(dev, "%s failed\n", __func__);

	return ret;
}
#endif /* CONFIG_PM_SLEEP */

static const struct of_device_id pruss_of_match[];

static const
struct pruss_private_data *pruss_get_private_data(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct pruss_match_private_data *data;
	const struct of_device_id *match;

	match = of_match_device(pruss_of_match, &pdev->dev);
	if (!match)
		return ERR_PTR(-ENODEV);

	if (of_device_is_compatible(np, "ti,am3352-pruss") ||
	    of_device_is_compatible(np, "ti,am4372-pruss"))
		return match->data;

	data = match->data;
	for (; data && data->device_name; data++) {
		if (!strcmp(dev_name(&pdev->dev), data->device_name))
			return data->priv_data;
	}

	return NULL;
}

static int pruss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	int ret;
	struct pruss *pruss;
	struct resource *res;
	int err, i, num_irqs;
	struct pruss_platform_data *pdata = dev_get_platdata(dev);
	const struct pruss_private_data *data;
	const char *mem_names[PRUSS_MEM_MAX] = { "dram0", "dram1", "shrdram2",
						 "intc", "cfg", "iep",
						 "mii_rt" };

	if (!node) {
		dev_err(dev, "Non-DT platform device not supported\n");
		return -ENODEV;
	}

	data = pruss_get_private_data(pdev);
	if (IS_ERR_OR_NULL(data)) {
		dev_err(dev, "missing private data\n");
		return -ENODEV;
	}

	if (data->has_reset && (!pdata || !pdata->deassert_reset ||
				!pdata->assert_reset || !pdata->reset_name)) {
		dev_err(dev, "platform data (reset configuration information) missing\n");
		return -ENODEV;
	}

	err = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(dev, "dma_set_coherent_mask: %d\n", err);
		return err;
	}

	pruss = devm_kzalloc(dev, sizeof(*pruss), GFP_KERNEL);
	if (!pruss)
		return -ENOMEM;

	pruss->dev = dev;
	pruss->data = data;
	mutex_init(&pruss->lock);
	mutex_init(&pruss->intc_lock);
	mutex_init(&pruss->cfg_lock);

	num_irqs = data->num_irqs;
	pruss->irqs = devm_kzalloc(dev, sizeof(*pruss->irqs) * num_irqs,
				   GFP_KERNEL);
	if (!pruss->irqs)
		return -ENOMEM;

	for (i = 0; i < num_irqs; i++)
		pruss->irqs[i] = -1;

	for (i = 0; i < ARRAY_SIZE(pruss->intc_config.sysev_to_ch); i++)
		pruss->intc_config.sysev_to_ch[i] = -1;

	for (i = 0; i < ARRAY_SIZE(pruss->intc_config.ch_to_host); i++)
		pruss->intc_config.ch_to_host[i] = -1;

	for (i = 0; i < num_irqs; i++) {
		pruss->irqs[i] = platform_get_irq(pdev, i);
		if (pruss->irqs[i] < 0) {
			dev_err(dev, "failed to get irq #%d ret = %d\n",
				i, pruss->irqs[i]);
			return pruss->irqs[i];
		}
	}
	dev_dbg(dev, "%d PRU interrupts parsed\n", num_irqs);

	for (i = 0; i < ARRAY_SIZE(mem_names); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   mem_names[i]);
		pruss->mem_regions[i].va = devm_ioremap_resource(dev, res);
		if (IS_ERR(pruss->mem_regions[i].va)) {
			dev_err(dev, "failed to parse and map memory resource %d %s\n",
				i, mem_names[i]);
			return PTR_ERR(pruss->mem_regions[i].va);
		}
		pruss->mem_regions[i].pa = res->start;
		pruss->mem_regions[i].size = resource_size(res);

		dev_dbg(dev, "memory %8s: pa %pa size 0x%x va %p\n",
			mem_names[i], &pruss->mem_regions[i].pa,
			pruss->mem_regions[i].size, pruss->mem_regions[i].va);
	}

	if (data->has_reset) {
		err = pdata->deassert_reset(pdev, pdata->reset_name);
		if (err) {
			dev_err(dev, "deassert_reset failed: %d\n", err);
			goto err_fail;
		}
	}

	pm_runtime_enable(dev);
	err = pruss_enable_module(pruss);
	if (err < 0) {
		dev_err(dev, "couldn't enable pruss\n");
		goto err_rpm_fail;
	}

	pruss_intc_init(pruss);

	platform_set_drvdata(pdev, pruss);

	mutex_lock(&pruss_list_mutex);
	list_add_tail(&pruss->node, &pruss_list);
	mutex_unlock(&pruss_list_mutex);

	dev_info(&pdev->dev, "creating platform devices for PRU cores\n");
	ret = of_platform_populate(node, NULL, data->aux_data, &pdev->dev);
	if (err) {
		dev_err(dev, "of_platform_populate failed\n");
		goto err_of_fail;
	}

	return 0;

err_of_fail:
	mutex_lock(&pruss_list_mutex);
	list_del(&pruss->node);
	mutex_unlock(&pruss_list_mutex);

	pruss_disable_module(pruss);
err_rpm_fail:
	pm_runtime_disable(dev);
	if (data->has_reset)
		pdata->assert_reset(pdev, pdata->reset_name);
err_fail:
	return err;
}

static int pruss_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pruss_platform_data *pdata = dev_get_platdata(dev);
	struct pruss *pruss = platform_get_drvdata(pdev);

	dev_info(dev, "remove platform devices for PRU cores\n");
	of_platform_depopulate(dev);

	mutex_lock(&pruss_list_mutex);
	list_del(&pruss->node);
	mutex_unlock(&pruss_list_mutex);

	pruss_disable_module(pruss);
	pm_runtime_disable(dev);
	if (pruss->data->has_reset)
		pdata->assert_reset(pdev, pdata->reset_name);

	return 0;
}

/*
 * auxdata lookup table for giving specific device names to PRU platform
 * devices. The device names are used in the driver to find private data
 * specific to a PRU-core such as an id and a firmware name etc, especially
 * needed when there are multiple PRUSS instances present on a SoC.
 * XXX: The auxdata in general is not a recommended usage, and this should
 *      eventually be eliminated. The current usage allows us to define the
 *      PRU device names with an identifier like xxxxxxxx.pru0 instead of a
 *      generic xxxxxxxx.pru.
 */
static struct of_dev_auxdata am335x_pruss_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,am3352-pru-rproc", 0x4a334000, "4a334000.pru0",
		       NULL),
	OF_DEV_AUXDATA("ti,am3352-pru-rproc", 0x4a338000, "4a338000.pru1",
		       NULL),
	{ /* sentinel */ },
};

static struct of_dev_auxdata am437x_pruss1_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,am4372-pru-rproc", 0x54434000, "54434000.pru0",
		       NULL),
	OF_DEV_AUXDATA("ti,am4372-pru-rproc", 0x54438000, "54438000.pru1",
		       NULL),
	{ /* sentinel */ },
};

static struct of_dev_auxdata am57xx_pruss1_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,am5728-pru-rproc", 0x4b234000, "4b234000.pru0",
		       NULL),
	OF_DEV_AUXDATA("ti,am5728-pru-rproc", 0x4b238000, "4b238000.pru1",
		       NULL),
	{ /* sentinel */ },
};

static struct of_dev_auxdata am57xx_pruss2_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,am5728-pru-rproc", 0x4b2b4000, "4b2b4000.pru0",
		       NULL),
	OF_DEV_AUXDATA("ti,am5728-pru-rproc", 0x4b2b8000, "4b2b8000.pru1",
		       NULL),
	{ /* sentinel */ },
};

/*
 * There is a one-to-one relation between PRU Host interrupts
 * and the PRU Host events. The interrupts are expected to be
 * in the increasing order of PRU Host events.
 */
static struct pruss_private_data am335x_priv_data = {
	.num_irqs = 8,
	.host_events = (BIT(2) | BIT(3) | BIT(4) | BIT(5) |
			BIT(6) | BIT(7) | BIT(8) | BIT(9)),
	.aux_data = am335x_pruss_rproc_auxdata_lookup,
	.has_reset = true,
};

static struct pruss_private_data am437x_priv_data = {
	.num_irqs = 7,
	.host_events = (BIT(2) | BIT(3) | BIT(4) | BIT(5) |
			BIT(6) | BIT(8) | BIT(9)),
	.aux_data = am437x_pruss1_rproc_auxdata_lookup,
	.has_reset = true,
};

static struct pruss_private_data am57xx_pruss1_priv_data = {
	.num_irqs = 8,
	.host_events = (BIT(2) | BIT(3) | BIT(4) | BIT(5) |
			BIT(6) | BIT(7) | BIT(8) | BIT(9)),
	.aux_data = am57xx_pruss1_rproc_auxdata_lookup,
};

static struct pruss_private_data am57xx_pruss2_priv_data = {
	.num_irqs = 8,
	.host_events = (BIT(2) | BIT(3) | BIT(4) | BIT(5) |
			BIT(6) | BIT(7) | BIT(8) | BIT(9)),
	.aux_data = am57xx_pruss2_rproc_auxdata_lookup,
};

static struct pruss_match_private_data am57xx_match_data[] = {
	{
		.device_name	= "4b200000.pruss",
		.priv_data	= &am57xx_pruss1_priv_data,
	},
	{
		.device_name	= "4b280000.pruss",
		.priv_data	= &am57xx_pruss2_priv_data,
	},
	{
		/* sentinel */
	},
};

static const struct of_device_id pruss_of_match[] = {
	{ .compatible = "ti,am3352-pruss", .data = &am335x_priv_data, },
	{ .compatible = "ti,am4372-pruss", .data = &am437x_priv_data, },
	{ .compatible = "ti,am5728-pruss", .data = &am57xx_match_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pruss_of_match);

static const struct dev_pm_ops pruss_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pruss_suspend, pruss_resume)
};

static struct platform_driver pruss_driver = {
	.driver = {
		.name = "pruss-rproc",
		.pm = &pruss_pm_ops,
		.of_match_table = of_match_ptr(pruss_of_match),
	},
	.probe  = pruss_probe,
	.remove = pruss_remove,
};
module_platform_driver(pruss_driver);

MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_DESCRIPTION("PRU-ICSS Subsystem Driver");
MODULE_LICENSE("GPL v2");
