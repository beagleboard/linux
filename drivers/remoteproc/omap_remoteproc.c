// SPDX-License-Identifier: GPL-2.0
/*
 * OMAP Remote Processor driver
 *
 * Copyright (C) 2011-2019 Texas Instruments Incorporated - http://www.ti.com/
 * Copyright (C) 2011 Google, Inc.
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Brian Swetland <swetland@google.com>
 * Fernando Guzman Lugo <fernando.lugo@ti.com>
 * Mark Grosen <mgrosen@ti.com>
 * Suman Anna <s-anna@ti.com>
 * Hari Kanigeri <h-kanigeri2@ti.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/remoteproc.h>
#include <linux/mailbox_client.h>
#include <linux/omap-mailbox.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <linux/platform_data/remoteproc-omap.h>

#include "omap_remoteproc.h"
#include "remoteproc_internal.h"

#define OMAP_RPROC_DSP_LOCAL_MEM_OFFSET		(0x00800000)
#define OMAP_RPROC_IPU_L2RAM_DEV_ADDR		(0x20000000)

/**
 * struct omap_rproc_boot_data - boot data structure for the DSP omap rprocs
 * @syscon: regmap handle for the system control configuration module
 * @boot_reg: boot register offset within the @syscon regmap
 * @boot_reg_shift: bit-field shift required for the boot address value in
 *		    @boot_reg
 */
struct omap_rproc_boot_data {
	struct regmap *syscon;
	unsigned int boot_reg;
	unsigned int boot_reg_shift;
};

/**
 * struct omap_rproc_mem - internal memory structure
 * @cpu_addr: MPU virtual address of the memory region
 * @bus_addr: bus address used to access the memory region
 * @dev_addr: device address of the memory region from DSP view
 * @size: size of the memory region
 */
struct omap_rproc_mem {
	void __iomem *cpu_addr;
	phys_addr_t bus_addr;
	u32 dev_addr;
	size_t size;
};

/**
 * struct omap_rproc - omap remote processor state
 * @mbox: mailbox channel handle
 * @client: mailbox client to request the mailbox channel
 * @boot_data: boot data structure for setting processor boot address
 * @mem: internal memory regions data
 * @num_mems: number of internal memory regions
 * @rproc: rproc handle
 */
struct omap_rproc {
	struct mbox_chan *mbox;
	struct mbox_client client;
	struct omap_rproc_boot_data *boot_data;
	struct omap_rproc_mem *mem;
	int num_mems;
	struct rproc *rproc;
};

/**
 * struct omap_rproc_dev_data - device data for the omap remote processor
 * @device_name: device name of the remote processor
 * @fw_name: firmware name to use
 */
struct omap_rproc_dev_data {
	const char *device_name;
	const char *fw_name;
};

/**
 * omap_rproc_mbox_callback() - inbound mailbox message handler
 * @client: mailbox client pointer used for requesting the mailbox channel
 * @data: mailbox payload
 *
 * This handler is invoked by omap's mailbox driver whenever a mailbox
 * message is received. Usually, the mailbox payload simply contains
 * the index of the virtqueue that is kicked by the remote processor,
 * and we let remoteproc core handle it.
 *
 * In addition to virtqueue indices, we also have some out-of-band values
 * that indicates different events. Those values are deliberately very
 * big so they don't coincide with virtqueue indices.
 */
static void omap_rproc_mbox_callback(struct mbox_client *client, void *data)
{
	struct omap_rproc *oproc = container_of(client, struct omap_rproc,
						client);
	struct device *dev = oproc->rproc->dev.parent;
	const char *name = oproc->rproc->name;
	u32 msg = (u32)data;

	dev_dbg(dev, "mbox msg: 0x%x\n", msg);

	switch (msg) {
	case RP_MBOX_CRASH:
		/* just log this for now. later, we'll also do recovery */
		dev_err(dev, "omap rproc %s crashed\n", name);
		break;
	case RP_MBOX_ECHO_REPLY:
		dev_info(dev, "received echo reply from %s\n", name);
		break;
	default:
		if (msg >= RP_MBOX_READY && msg < RP_MBOX_END_MSG)
			return;
		if (msg > oproc->rproc->max_notifyid) {
			dev_dbg(dev, "dropping unknown message 0x%x", msg);
			return;
		}
		/* msg contains the index of the triggered vring */
		if (rproc_vq_interrupt(oproc->rproc, msg) == IRQ_NONE)
			dev_dbg(dev, "no message was found in vqid %d\n", msg);
	}
}

/* kick a virtqueue */
static void omap_rproc_kick(struct rproc *rproc, int vqid)
{
	struct omap_rproc *oproc = rproc->priv;
	struct device *dev = rproc->dev.parent;
	int ret;

	/* send the index of the triggered virtqueue in the mailbox payload */
	ret = mbox_send_message(oproc->mbox, (void *)vqid);
	if (ret < 0)
		dev_err(dev, "failed to send mailbox message, status = %d\n",
			ret);
}

/**
 * omap_rproc_write_dsp_boot_addr - set boot address for a DSP remote processor
 * @rproc: handle of a remote processor
 *
 * Set boot address for a supported DSP remote processor.
 */
static int omap_rproc_write_dsp_boot_addr(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct omap_rproc *oproc = rproc->priv;
	struct omap_rproc_boot_data *bdata = oproc->boot_data;
	u32 offset = bdata->boot_reg;
	unsigned int value = rproc->bootaddr;
	unsigned int mask = ~(SZ_1K - 1);

	if (value & (SZ_1K - 1)) {
		dev_err(dev, "invalid boot address 0x%x, must be aligned on a 1KB boundary\n",
			value);
		return -EINVAL;
	}

	value >>= bdata->boot_reg_shift;
	mask >>= bdata->boot_reg_shift;

	regmap_update_bits(bdata->syscon, offset, mask, value);

	return 0;
}

/*
 * Power up the remote processor.
 *
 * This function will be invoked only after the firmware for this rproc
 * was loaded, parsed successfully, and all of its resource requirements
 * were met.
 */
static int omap_rproc_start(struct rproc *rproc)
{
	struct omap_rproc *oproc = rproc->priv;
	struct device *dev = rproc->dev.parent;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;
	int ret;
	struct mbox_client *client = &oproc->client;

	if (oproc->boot_data) {
		ret = omap_rproc_write_dsp_boot_addr(rproc);
		if (ret)
			return ret;
	}

	client->dev = dev;
	client->tx_done = NULL;
	client->rx_callback = omap_rproc_mbox_callback;
	client->tx_block = false;
	client->knows_txdone = false;

	oproc->mbox = mbox_request_channel(client, 0);
	if (IS_ERR(oproc->mbox)) {
		ret = -EBUSY;
		dev_err(dev, "mbox_request_channel failed: %ld\n",
			PTR_ERR(oproc->mbox));
		return ret;
	}

	/*
	 * Ping the remote processor. this is only for sanity-sake;
	 * there is no functional effect whatsoever.
	 *
	 * Note that the reply will _not_ arrive immediately: this message
	 * will wait in the mailbox fifo until the remote processor is booted.
	 */
	ret = mbox_send_message(oproc->mbox, (void *)RP_MBOX_ECHO_REQUEST);
	if (ret < 0) {
		dev_err(dev, "mbox_send_message failed: %d\n", ret);
		goto put_mbox;
	}

	ret = pdata->device_enable(pdev);
	if (ret) {
		dev_err(dev, "omap_device_enable failed: %d\n", ret);
		goto put_mbox;
	}

	return 0;

put_mbox:
	mbox_free_channel(oproc->mbox);
	return ret;
}

/* power off the remote processor */
static int omap_rproc_stop(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;
	struct omap_rproc *oproc = rproc->priv;
	int ret;

	ret = pdata->device_shutdown(pdev);
	if (ret)
		return ret;

	mbox_free_channel(oproc->mbox);

	return 0;
}

/*
 * Internal Memory translation helper
 *
 * Custom function implementing the rproc .da_to_va ops to provide address
 * translation (device address to kernel virtual address) for internal RAMs
 * present in a DSP or IPU device). The translated addresses can be used
 * either by the remoteproc core for loading, or by any rpmsg bus drivers.
 */
static void *omap_rproc_da_to_va(struct rproc *rproc, u64 da, int len,
				 u32 flags)
{
	struct omap_rproc *oproc = rproc->priv;
	void *va = NULL;
	int i;
	u32 offset;

	if (len <= 0)
		return NULL;

	if (!oproc->num_mems)
		return NULL;

	for (i = 0; i < oproc->num_mems; i++) {
		if (da >= oproc->mem[i].dev_addr && da + len <=
		    oproc->mem[i].dev_addr +  oproc->mem[i].size) {
			offset = da -  oproc->mem[i].dev_addr;
			/* __force to make sparse happy with type conversion */
			va = (__force void *)(oproc->mem[i].cpu_addr + offset);
			break;
		}
	}

	return va;
}

static const struct rproc_ops omap_rproc_ops = {
	.start		= omap_rproc_start,
	.stop		= omap_rproc_stop,
	.kick		= omap_rproc_kick,
	.da_to_va	= omap_rproc_da_to_va,
};

static const struct omap_rproc_dev_data omap4_dsp_dev_data = {
	.device_name	= "dsp",
	.fw_name	= "omap4-dsp-fw.xe64T",
};

static const struct omap_rproc_dev_data omap4_ipu_dev_data = {
	.device_name	= "ipu",
	.fw_name	= "omap4-ipu-fw.xem3",
};

static const struct omap_rproc_dev_data omap5_dsp_dev_data = {
	.device_name	= "dsp",
	.fw_name	= "omap5-dsp-fw.xe64T",
};

static const struct omap_rproc_dev_data omap5_ipu_dev_data = {
	.device_name	= "ipu",
	.fw_name	= "omap5-ipu-fw.xem4",
};

static const struct omap_rproc_dev_data dra7_rproc_dev_data[] = {
	{
		.device_name	= "40800000.dsp",
		.fw_name	= "dra7-dsp1-fw.xe66",
	},
	{
		.device_name	= "41000000.dsp",
		.fw_name	= "dra7-dsp2-fw.xe66",
	},
	{
		.device_name	= "55020000.ipu",
		.fw_name	= "dra7-ipu2-fw.xem4",
	},
	{
		.device_name	= "58820000.ipu",
		.fw_name	= "dra7-ipu1-fw.xem4",
	},
	{
		/* sentinel */
	},
};

static const struct of_device_id omap_rproc_of_match[] = {
	{
		.compatible     = "ti,omap4-dsp",
		.data           = &omap4_dsp_dev_data,
	},
	{
		.compatible     = "ti,omap4-ipu",
		.data           = &omap4_ipu_dev_data,
	},
	{
		.compatible     = "ti,omap5-dsp",
		.data           = &omap5_dsp_dev_data,
	},
	{
		.compatible     = "ti,omap5-ipu",
		.data           = &omap5_ipu_dev_data,
	},
	{
		.compatible     = "ti,dra7-dsp",
		.data           = dra7_rproc_dev_data,
	},
	{
		.compatible     = "ti,dra7-ipu",
		.data           = dra7_rproc_dev_data,
	},
	{
		/* end */
	},
};
MODULE_DEVICE_TABLE(of, omap_rproc_of_match);

static const char *omap_rproc_get_firmware(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct omap_rproc_dev_data *data;

	data = of_device_get_match_data(&pdev->dev);
	if (!data)
		return ERR_PTR(-ENODEV);

	if (!of_device_is_compatible(np, "ti,dra7-dsp") &&
	    !of_device_is_compatible(np, "ti,dra7-ipu"))
		return data->fw_name;

	for (; data && data->device_name; data++) {
		if (!strcmp(dev_name(&pdev->dev), data->device_name))
			return data->fw_name;
	}

	return ERR_PTR(-ENOENT);
}

static int omap_rproc_get_boot_data(struct platform_device *pdev,
				    struct rproc *rproc)
{
	struct device_node *np = pdev->dev.of_node;
	struct omap_rproc *oproc = rproc->priv;
	int ret;

	if (!of_device_is_compatible(np, "ti,omap4-dsp") &&
	    !of_device_is_compatible(np, "ti,omap5-dsp") &&
	    !of_device_is_compatible(np, "ti,dra7-dsp"))
		return 0;

	oproc->boot_data = devm_kzalloc(&pdev->dev, sizeof(*oproc->boot_data),
					GFP_KERNEL);
	if (!oproc->boot_data)
		return -ENOMEM;

	if (!of_property_read_bool(np, "syscon-bootreg")) {
		dev_err(&pdev->dev, "syscon-bootreg property is missing\n");
		return -EINVAL;
	}

	oproc->boot_data->syscon =
			syscon_regmap_lookup_by_phandle(np, "syscon-bootreg");
	if (IS_ERR(oproc->boot_data->syscon)) {
		ret = PTR_ERR(oproc->boot_data->syscon);
		return ret;
	}

	if (of_property_read_u32_index(np, "syscon-bootreg", 1,
				       &oproc->boot_data->boot_reg)) {
		dev_err(&pdev->dev, "couldn't get the boot register\n");
		return -EINVAL;
	}

	if (of_device_is_compatible(np, "ti,dra7-dsp"))
		oproc->boot_data->boot_reg_shift = 10;

	return 0;
}

static int omap_rproc_of_get_internal_memories(struct platform_device *pdev,
					       struct rproc *rproc)
{
	static const char * const ipu_mem_names[] = {"l2ram"};
	static const char * const dra7_dsp_mem_names[] = {"l2ram", "l1pram",
								"l1dram"};
	struct device_node *np = pdev->dev.of_node;
	struct omap_rproc *oproc = rproc->priv;
	struct device *dev = &pdev->dev;
	const char * const *mem_names;
	struct resource *res;
	int num_mems;
	const __be32 *addrp;
	u32 l4_offset = 0;
	u64 size;
	int i;

	/* OMAP4 and OMAP5 DSPs do not have support for flat SRAM */
	if (of_device_is_compatible(np, "ti,omap4-dsp") ||
	    of_device_is_compatible(np, "ti,omap5-dsp"))
		return 0;

	/* DRA7 DSPs have two additional SRAMs at L1 level */
	if (of_device_is_compatible(np, "ti,dra7-dsp")) {
		mem_names = dra7_dsp_mem_names;
		num_mems = ARRAY_SIZE(dra7_dsp_mem_names);
	} else {
		mem_names = ipu_mem_names;
		num_mems = ARRAY_SIZE(ipu_mem_names);
	}

	oproc->mem = devm_kcalloc(dev, num_mems, sizeof(*oproc->mem),
				  GFP_KERNEL);
	if (!oproc->mem)
		return -ENOMEM;

	for (i = 0; i < num_mems; i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   mem_names[i]);
		oproc->mem[i].cpu_addr = devm_ioremap_resource(dev, res);
		if (IS_ERR(oproc->mem[i].cpu_addr)) {
			dev_err(dev, "failed to parse and map %s memory\n",
				mem_names[i]);
			return PTR_ERR(oproc->mem[i].cpu_addr);
		}
		oproc->mem[i].bus_addr = res->start;

		/*
		 * The DSPs have the internal memories starting at a fixed
		 * offset of 0x800000 from address 0, and this corresponds to
		 * L2RAM. The L3 address view has the L2RAM bus address as the
		 * starting address for the IP, so the L2RAM memory region needs
		 * to be processed first, and the device addresses for each
		 * memory region can be computed using the relative offset
		 * from this base address.
		 */
		if (of_device_is_compatible(np, "ti,dra7-dsp") &&
		    !strcmp(mem_names[i], "l2ram")) {
			addrp = of_get_address(dev->of_node, i, &size, NULL);
			l4_offset = be32_to_cpu(*addrp);
		}
		oproc->mem[i].dev_addr =
			of_device_is_compatible(np, "ti,dra7-dsp") ?
				res->start - l4_offset +
				OMAP_RPROC_DSP_LOCAL_MEM_OFFSET :
				OMAP_RPROC_IPU_L2RAM_DEV_ADDR;
		oproc->mem[i].size = resource_size(res);

		dev_dbg(dev, "memory %8s: bus addr %pa size 0x%x va %p da 0x%x\n",
			mem_names[i], &oproc->mem[i].bus_addr,
			oproc->mem[i].size, oproc->mem[i].cpu_addr,
			oproc->mem[i].dev_addr);
	}
	oproc->num_mems = num_mems;

	return 0;
}

static int omap_rproc_probe(struct platform_device *pdev)
{
	struct omap_rproc_pdata *pdata = dev_get_platdata(&pdev->dev);
	struct device_node *np = pdev->dev.of_node;
	struct omap_rproc *oproc;
	struct rproc *rproc;
	const char *firmware;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "only DT-based devices are supported\n");
		return -ENODEV;
	}

	if (!pdata || !pdata->device_enable || !pdata->device_shutdown) {
		dev_err(&pdev->dev, "platform data is either missing or incomplete\n");
		return -ENODEV;
	}

	firmware = omap_rproc_get_firmware(pdev);
	if (IS_ERR(firmware))
		return PTR_ERR(firmware);

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "dma_set_coherent_mask: %d\n", ret);
		return ret;
	}

	rproc = rproc_alloc(&pdev->dev, dev_name(&pdev->dev), &omap_rproc_ops,
			    firmware, sizeof(*oproc));
	if (!rproc)
		return -ENOMEM;

	oproc = rproc->priv;
	oproc->rproc = rproc;
	/* All existing OMAP IPU and DSP processors have an MMU */
	rproc->has_iommu = true;

	ret = omap_rproc_of_get_internal_memories(pdev, rproc);
	if (ret)
		goto free_rproc;

	ret = omap_rproc_get_boot_data(pdev, rproc);
	if (ret)
		goto free_rproc;

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "device does not have specific CMA pool\n");
		goto free_rproc;
	}

	platform_set_drvdata(pdev, rproc);

	ret = rproc_add(rproc);
	if (ret)
		goto release_mem;

	return 0;

release_mem:
	of_reserved_mem_device_release(&pdev->dev);
free_rproc:
	rproc_free(rproc);
	return ret;
}

static int omap_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);
	rproc_free(rproc);
	of_reserved_mem_device_release(&pdev->dev);

	return 0;
}

static struct platform_driver omap_rproc_driver = {
	.probe = omap_rproc_probe,
	.remove = omap_rproc_remove,
	.driver = {
		.name = "omap-rproc",
		.of_match_table = omap_rproc_of_match,
	},
};

module_platform_driver(omap_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("OMAP Remote Processor control driver");
