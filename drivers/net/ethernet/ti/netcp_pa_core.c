/*
 * Keystone NetCP PA (Packet Accelerator) Core Driver functions
 *
 * Copyright (C) 2012-2015 Texas Instruments Incorporated
 * Author: Murali Karicheri <m-karicheri2@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/if_vlan.h>
#include <linux/io.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/rtnetlink.h>
#include <linux/soc/ti/knav_qmss.h>
#include <linux/soc/ti/knav_dma.h>
#include <linux/soc/ti/knav_helpers.h>

#include "netcp.h"
#include "netcp_pa_core.h"

#define mcast_filter_attr_to_pa(attr)	\
	container_of(attr, struct pa_core_device, mcast_filter_attr)

static struct pa_lut_entry *pa_core_lut_alloc(struct pa_core_device *core_dev,
					      enum pa_lut_type type,
					      bool backwards)
{
	struct pa_lut_entry *lut_table, *entry;
	u32 lut_size;
	int i;

	if (type == PA_LUT_MAC) {
		lut_table = core_dev->lut;
		lut_size = core_dev->lut_size;
	} else {
		lut_table = core_dev->ip_lut;
		lut_size = core_dev->ip_lut_size;
	}

	if (!backwards) {
		for (i = 0; i < lut_size; i++) {
			entry = lut_table + i;
			if (!entry->valid || entry->in_use)
				continue;
			entry->in_use = true;
			return entry;
		}
	} else {
		for (i = lut_size - 1; i >= 0; i--) {
			entry = lut_table + i;
			if (!entry->valid || entry->in_use)
				continue;
			entry->in_use = true;
			return entry;
		}
	}

	return NULL;
}

static int pa_core_parse_lut_range(struct pa_core_device *core_dev,
				   struct device_node *node, bool mac)
{
	int ret = -EINVAL, len = 0, start, end, i, j, table_size, num_ranges;
	struct device *dev = core_dev->dev;
	char *lut_range = mac ? "mac-lut-ranges" : "ip-lut-ranges";
	struct pa_lut_entry *lut;
	u32 *prange;

	if (!of_get_property(node, lut_range, &len)) {
		dev_info(dev, "No %s array in dt bindings for PA\n",
			 lut_range);
		/* ip-lut-range is optional */
		return 0;
	}

	prange = kzalloc(len, GFP_KERNEL);
	if (!prange)
		return -ENOMEM;

	len = len / sizeof(u32);
	if ((len % 2) != 0) {
		dev_err(dev, "invalid address map in dt binding\n");
		ret = -EINVAL;
		goto free_prange;
	}

	num_ranges = len / 2;
	if (of_property_read_u32_array(node, lut_range, prange, len)) {
		dev_err(dev, "No range-map array for %s in dt bindings\n",
			lut_range);
		ret = -EINVAL;
		goto free_prange;
	}

	table_size = prange[2 * num_ranges - 1] + 1;
	len = table_size * sizeof(struct pa_lut_entry);
	lut  = kzalloc(len, GFP_KERNEL);
	if (!lut) {
		ret = -ENOMEM;
		goto free_prange;
	}

	for (i = 0; i < num_ranges; i++) {
		start = prange[i * 2];
		end   = prange[i * 2 + 1];
		for (j = start; j <= end; j++) {
			lut[j].valid = true;
			lut[j].index = j;
			dev_dbg(dev, "setting entry %d to valid\n", j);
		}
	}
	if (mac) {
		core_dev->lut_size = table_size;
		core_dev->lut = lut;
	} else {
		core_dev->ip_lut_size = table_size;
		core_dev->ip_lut = lut;
	}

	ret = 0;
free_prange:
	kfree(prange);
	return ret;
}

static int pa_core_get_dt_bindings(struct pa_core_device *core_dev,
				   struct device_node *node)
{
	struct device_node *child, *clusters;
	struct device *dev = core_dev->dev;
	struct pa_cluster_config *cfg;
	int ret = 0;
	u32 tmp[2];

	/* Get cluster tx queue and channel */
	clusters = of_get_child_by_name(node, "clusters");
	if (clusters) {
		for_each_child_of_node(clusters, child) {
			int cluster_id;

			ret = of_property_read_u32(child, "reg", &cluster_id);
			if (ret) {
				dev_info(core_dev->dev,
					 "No reg for cluster id\n");
				goto release_of_node;
			}
			if (cluster_id >= core_dev->hw->num_clusters) {
				dev_err(dev, "invalid cluster id %d\n",
					cluster_id);
				ret = -EINVAL;
				goto release_of_node;
			}
			cfg = &core_dev->cluster_config[cluster_id];
			ret = of_property_read_u32(child, "tx-queue",
						   &cfg->tx_queue_id);
			if (ret < 0) {
				dev_err(dev, "No \"tx-queue\" for cluster %d\n",
					cluster_id);
				goto release_of_node;
			}
			ret = of_property_read_string(child, "tx-channel",
						      &cfg->tx_chan_name);
			if (ret < 0) {
				dev_err(dev,
					"No \"tx-channel\" for cluster %d\n",
					cluster_id);
				goto release_of_node;
			}
		}
	}
	of_node_put(clusters);

	ret = of_property_read_u32_array(node, "tx-cmd-pool", tmp, 2);
	if (ret < 0) {
		dev_err(dev, "missing \"tx-cmd-pool\" parameter\n");
		return ret;
	}
	core_dev->tx_cmd_pool_size = tmp[0];
	core_dev->tx_cmd_pool_region_id = tmp[1];

	ret = of_property_read_string(node, "rx-cmd-rsp-chan",
				      &core_dev->rx_cmd_rsp_chan_name);

	if (ret < 0) {
		dev_err(dev, "missing \"rx-cmd-rsp-chan\" parameter, err %d\n",
			ret);
		return ret;
	}

	ret = of_property_read_u32_array(node, "rx-cmd-rsp-pool", tmp, 2);
	if (ret < 0) {
		dev_err(dev, "missing \"rx-cmd-rsp-pool\" parameter\n");
		return ret;
	}
	core_dev->rx_cmd_rsp_pool_size = tmp[0];
	core_dev->rx_cmd_rsp_pool_region_id = tmp[1];

	ret = of_property_read_u32_array(node, "rx-cmd-rsp-queue-depth",
					 core_dev->rx_cmd_rsp_queue_depths,
					 KNAV_DMA_FDQ_PER_CHAN);
	if (ret < 0) {
		dev_err(dev, "missing \"rx-cmd-rsp-queue-depth\" parameter\n");
		return ret;
	}

	ret = of_property_read_u32_array(node, "rx-cmd-rsp-buffer-size",
					 core_dev->rx_cmd_rsp_buffer_sizes,
					 KNAV_DMA_FDQ_PER_CHAN);
	if (ret) {
		dev_err(dev, "missing \"rx-cmd-rsp-buffer-size\" parameter\n");
		return ret;
	}

	if (of_device_is_compatible(node, "ti,netcp-pa2")) {
		core_dev->disable_hw_tstamp = true;
	} else {
		core_dev->disable_hw_tstamp = false;
		if (of_property_read_bool(node, "disable-hw-timestamp")) {
			core_dev->disable_hw_tstamp = true;
			dev_warn(dev, "No PA timestamping\n");
		}
	}

	ret = pa_core_parse_lut_range(core_dev, node, true);
	if (ret)
		return ret;

	/* NOTE: DTS configuration MUST ensure that the completion queue &
	 * Rx flow for each interface is sequential.
	 */
	ret = of_property_read_u32_array(node, "rx-route", tmp, 2);
	if (ret) {
		dev_err(dev, "Couldn't get rx-route from dt bindings\n");
		return ret;
	}
	core_dev->rx_queue_base = tmp[0];
	core_dev->rx_flow_base = tmp[1];

	/* Get IP lut ranges */
	ret = pa_core_parse_lut_range(core_dev, node, false);

	return ret;

release_of_node:
	of_node_put(child);
	of_node_put(clusters);
	return ret;
}

static int
pa_core_load_n_start_pdsps(struct pa_core_device *core_dev,
			   const char *firmwares[][PA_MAX_FIRMWARES])
{
	int i, j, ret = 0;
	bool found;

	for (i = 0; i < core_dev->hw->num_pdsps; ++i) {
		found = false;
		/* search through the firmware list for each PDSP */
		for (j = 0; j < PA_MAX_FIRMWARES && firmwares[i][j]; j++) {
			ret =
			request_firmware_direct(&core_dev->fw[i],
						firmwares[i][j], core_dev->dev);
			if (ret == 0) {
				/* found a firmware */
				found = true;
				break;
			}
		}
		if (found) {
			/* Download the firmware to the PDSP */
			dev_dbg(core_dev->dev,
				"Download firmware %s to PDSP %d\n",
				firmwares[i][j], i);
			core_dev->hw->set_firmware(core_dev, i,
				  (const unsigned int *)core_dev->fw[i]->data,
				   core_dev->fw[i]->size);

			release_firmware(core_dev->fw[i]);
		} else {
			dev_err(core_dev->dev,
				"firmware not found for pdsp %d\n", i);
			for (j = i - 1; j >= 0; j--)
				release_firmware(core_dev->fw[j]);
			return -ENODEV;
		}
	}
	return ret;
}

static int pa_core_setup_desc_pools(struct pa_core_device *core_dev)
{
	int ret = 0;

	core_dev->tx_cmd_pool =
		knav_pool_create("pa-cluster-tx-cmd-pool",
				 core_dev->tx_cmd_pool_size,
				 core_dev->tx_cmd_pool_region_id);

	if (IS_ERR_OR_NULL(core_dev->tx_cmd_pool)) {
		dev_err(core_dev->dev,
			"Couldn't create tx cmd pool for PA clusters\n");
		ret = PTR_ERR(core_dev->tx_cmd_pool);
		core_dev->tx_cmd_pool = NULL;
		return ret;
	}

	core_dev->rx_cmd_rsp_pool =
	knav_pool_create("pa-pdsp-rx-cmd-rsp-pool",
			 core_dev->rx_cmd_rsp_pool_size,
			 core_dev->rx_cmd_rsp_pool_region_id);

	if (IS_ERR_OR_NULL(core_dev->rx_cmd_rsp_pool)) {
		dev_err(core_dev->dev,
			"Couldn't create rx cmd rsp pool for PA PDSPs\n");
		ret = PTR_ERR(core_dev->rx_cmd_rsp_pool);
		core_dev->rx_cmd_rsp_pool = NULL;
	}

	return ret;
}

static int pa_core_init_egress_tx_pipe(struct pa_core_device *core_dev,
				       struct netcp_device *netcp_device)
{
	int cluster = core_dev->hw->egress_cluster_id;
	struct pa_cluster_config *cfg = &core_dev->cluster_config[cluster];
	int ret = 0;

	/* initialize txpipe for tx egress cluster for pa device */
	netcp_txpipe_init(&core_dev->tx_pipe, netcp_device,
			  cfg->tx_chan_name,
			  cfg->tx_queue_id);

	ret = netcp_txpipe_open(&core_dev->tx_pipe);
	if (ret) {
		dev_err(core_dev->dev, "Error creating tx pipe to egress cluster\n");
		return ret;
	}

	return ret;
}

static void pa_tx_dma_callback(void *data)
{
	struct pa_cluster_config *cfg = (struct pa_cluster_config *)data;

	knav_queue_disable_notify(cfg->tx_cmd_cmpl_queue);
	tasklet_schedule(&cfg->tx_task);
}

static void  pa_allocate_rx_descs(struct pa_core_device *core_dev, int fdq)
{
	struct knav_dma_desc *hwdesc;
	unsigned int buf_len, dma_sz;
	u32 desc_info, pkt_info;
	dma_addr_t dma;
	void *bufptr;
	u32 pad[2];

	/* Allocate descriptor */
	hwdesc = knav_pool_desc_get(core_dev->rx_cmd_rsp_pool);
	if (IS_ERR_OR_NULL(hwdesc)) {
		dev_dbg(core_dev->dev, "out of pa rx cmd rsp pool desc\n");
		return;
	}

	/* Allocate a primary receive queue entry */
	buf_len = core_dev->rx_cmd_rsp_buffer_sizes[fdq];
	bufptr = kmalloc(buf_len, GFP_ATOMIC | GFP_DMA | __GFP_COLD);
	pad[0] = (u32)bufptr;
	pad[1] = buf_len;

	if (unlikely(!bufptr)) {
		dev_warn_ratelimited(core_dev->dev,
				     "Primary RX buffer alloc failed\n");
		goto fail;
	}

	dma = dma_map_single(core_dev->dev, bufptr, buf_len, DMA_TO_DEVICE);
	desc_info =  KNAV_DMA_DESC_PS_INFO_IN_DESC;
	desc_info |= buf_len & KNAV_DMA_DESC_PKT_LEN_MASK;
	pkt_info =  KNAV_DMA_DESC_HAS_EPIB;
	pkt_info |= KNAV_DMA_NUM_PS_WORDS << KNAV_DMA_DESC_PSLEN_SHIFT;
	pkt_info |= (core_dev->rx_cmd_rsp_queue_id & KNAV_DMA_DESC_RETQ_MASK) <<
		    KNAV_DMA_DESC_RETQ_SHIFT;
	knav_dma_set_org_pkt_info(dma, buf_len, hwdesc);
	knav_dma_set_pad_info(pad[0], pad[1], hwdesc);
	knav_dma_set_desc_info(desc_info, pkt_info, hwdesc);

	/* Push to FDQs */
	knav_pool_desc_map(core_dev->rx_cmd_rsp_pool, hwdesc,
			   sizeof(*hwdesc), &dma, &dma_sz);
	knav_queue_push(core_dev->rx_fdq[fdq], dma, sizeof(*hwdesc), 0);
	return;

fail:
	knav_pool_desc_put(core_dev->rx_cmd_rsp_pool, hwdesc);
}

/* Refill Rx FDQ with descriptors */
static void pa_rxfdq_refill(struct pa_core_device *core_dev)
{
	u32 fdq_deficit[KNAV_DMA_FDQ_PER_CHAN] = {0};
	int i;

	/* Calculate the FDQ deficit and refill */
	for (i = 0; i < KNAV_DMA_FDQ_PER_CHAN && core_dev->rx_fdq[i]; i++) {
		fdq_deficit[i] = core_dev->rx_cmd_rsp_queue_depths[i] -
				 knav_queue_get_count(core_dev->rx_fdq[i]);

		while (fdq_deficit[i]--)
			pa_allocate_rx_descs(core_dev, i);
	}
}

static void pa_tx_compl_work_handler(unsigned long data)
{
	struct pa_cluster_config *cfg = (struct pa_cluster_config *)data;
	struct pa_core_device *core_dev = cfg->core_dev;
	struct knav_dma_desc *hwdesc;
	dma_addr_t dma_desc, dma_buf;
	unsigned int buf_len, dma_sz;
	struct pa_packet *p_info;
	u32 tmp;

	for (; ;) {
		dma_buf = knav_queue_pop(cfg->tx_cmd_cmpl_queue, &dma_sz);
		if (!dma_buf)
			break;

		hwdesc = knav_pool_desc_unmap(core_dev->tx_cmd_pool,
					      dma_buf, dma_sz);
		if (!hwdesc) {
			dev_err(core_dev->dev, "failed to unmap Tx desc\n");
			continue;
		}

		knav_dma_get_pkt_info(&dma_buf, &buf_len, &dma_desc, hwdesc);
		if (dma_buf && buf_len)
			dma_unmap_single(core_dev->dev, dma_buf, buf_len,
					 DMA_TO_DEVICE);
		else
			dev_warn(core_dev->dev,
				 "bad Tx desc buf(%p), len(%d)\n",
				 (void *)dma_buf, buf_len);

		knav_dma_get_pad_info((u32 *)&p_info, &tmp, hwdesc);
		if (!p_info) {
			dev_err(core_dev->dev,
				"p_info ptr corrupted in desc\n");
			continue;
		}
		kfree(p_info);
		knav_pool_desc_put(core_dev->tx_cmd_pool, hwdesc);
	}
	knav_queue_enable_notify(cfg->tx_cmd_cmpl_queue);
}

static int pa_core_init_ingress_tx_resources(struct pa_core_device *core_dev,
					     bool l2_cluster)
{
	struct knav_queue_notify_config notify_cfg;
	struct device *dev = core_dev->dev;
	struct knav_dma_cfg config;
	struct pa_cluster_config *cfg;
	int ret = 0, cluster_id;
	char name[32];

	cluster_id = (l2_cluster) ? core_dev->hw->ingress_l2_cluster_id :
				    core_dev->hw->ingress_l3_cluster_id;

	/* Open the PA Command transmit channel */
	memset(&config, 0, sizeof(config));
	config.direction = DMA_MEM_TO_DEV;
	config.u.tx.filt_einfo = false;
	config.u.tx.filt_pswords = false;
	config.u.tx.priority = DMA_PRIO_MED_L;

	cfg = &core_dev->cluster_config[cluster_id];
	cfg->tx_chan = knav_dma_open_channel(dev, cfg->tx_chan_name,
					     &config);
	if (IS_ERR_OR_NULL(cfg->tx_chan)) {
		dev_err(dev, "Couldnt get PATX cmd channel\n");
		ret = PTR_ERR(cfg->tx_chan);
		cfg->tx_chan = NULL;
		return ret;
	}

	snprintf(name, sizeof(name), "pa-cluster%d-cmd-queue",
		 cluster_id);
	cfg->tx_queue = knav_queue_open(name, cfg->tx_queue_id, 0);
	if (IS_ERR_OR_NULL(cfg->tx_queue)) {
		ret = PTR_ERR(cfg->tx_queue);
		cfg->tx_queue = NULL;
		return ret;
	}

	snprintf(name, sizeof(name), "pa-cluster%d-tx-compl-queue",
		 cluster_id);
	cfg->tx_cmd_cmpl_queue = knav_queue_open(name, KNAV_QUEUE_QPEND, 0);
	if (IS_ERR_OR_NULL(cfg->tx_cmd_cmpl_queue)) {
		dev_err(dev,
			"Error in open pa-cluster-tx-compl-queue\n");
		ret = PTR_ERR(cfg->tx_cmd_cmpl_queue);
		cfg->tx_cmd_cmpl_queue = NULL;
		return ret;
	}

	cfg->tx_cmd_cmpl_queue_id =
			knav_queue_get_id(cfg->tx_cmd_cmpl_queue);

	notify_cfg.fn = pa_tx_dma_callback;
	notify_cfg.fn_arg = cfg;
	knav_queue_disable_notify(cfg->tx_cmd_cmpl_queue);
	ret = knav_queue_device_control(cfg->tx_cmd_cmpl_queue,
					KNAV_QUEUE_SET_NOTIFIER,
					(unsigned long)&notify_cfg);
	if (ret) {
		dev_err(dev,
			"Error in set notifier for tx_cmd_cmpl_queue\n");
		return ret;
	}
	tasklet_init(&cfg->tx_task, pa_tx_compl_work_handler,
		     (unsigned long)cfg);
	cfg->core_dev = core_dev;
	knav_queue_enable_notify(cfg->tx_cmd_cmpl_queue);

	return ret;
}

static void pa_rx_chan_notify(void *arg)
{
	struct pa_core_device *core_dev = arg;

	knav_queue_disable_notify(core_dev->rx_cmd_rsp_queue);
	tasklet_schedule(&core_dev->rx_task);
}

static void pa_free_rx_desc_chain(struct pa_core_device *core_dev,
				  struct knav_dma_desc *desc)
{
	dma_addr_t dma_desc, dma_buf;
	struct knav_dma_desc *ndesc;
	unsigned int buf_len, org_buf_len, dma_sz = sizeof(*ndesc);
	void *org_buf_ptr;

	knav_dma_get_words(&dma_desc, 1, &desc->next_desc);
	while (dma_desc) {
		ndesc = knav_pool_desc_unmap(core_dev->rx_cmd_rsp_pool,
					     dma_desc, dma_sz);
		if (!ndesc) {
			dev_err(core_dev->dev, "failed to unmap Rx desc\n");
			break;
		}
		knav_dma_get_pkt_info(&dma_buf, &buf_len, &dma_desc, ndesc);
		knav_dma_get_pad_info((u32 *)&org_buf_ptr, &org_buf_len, ndesc);
		dma_unmap_single(core_dev->dev, dma_buf, org_buf_len,
				 DMA_TO_DEVICE);
		kfree(org_buf_ptr);
		knav_pool_desc_put(core_dev->rx_cmd_rsp_pool, ndesc);
		knav_dma_get_words(&dma_desc, 1, &ndesc->next_desc);
	}
}

static void pa_rx_compl_work_handler(unsigned long data)
{
	struct pa_core_device *core_dev = (struct pa_core_device *)data;
	unsigned int buf_len, dma_sz, org_buf_len;
	struct knav_dma_desc *hwdesc;
	dma_addr_t dma_desc, dma_buf;
	struct pa_packet p_info;
	void *org_buf_ptr;

	for (; ;) {
		dma_desc = knav_queue_pop(core_dev->rx_cmd_rsp_queue, &dma_sz);
		if (!dma_desc)
			break;

		hwdesc = knav_pool_desc_unmap(core_dev->rx_cmd_rsp_pool,
					      dma_desc, dma_sz);
		if (!hwdesc) {
			dev_err(core_dev->dev, "failed to unmap rx desc\n");
			continue;
		}
		/* Release chained descriptors. only data in the first
		 * descriptor buffer needs to be processed based on current
		 * response types supported (status of commands and timestamp.
		 * This will change if more response types are supported in
		 * future.
		 */
		pa_free_rx_desc_chain(core_dev, hwdesc);
		knav_dma_get_pad_info((u32 *)&org_buf_ptr, &org_buf_len,
				      hwdesc);
		knav_dma_get_pkt_info(&dma_buf, &buf_len, &dma_desc, hwdesc);
		dma_unmap_single(core_dev->dev, dma_buf, org_buf_len,
				 DMA_TO_DEVICE);
		p_info.epib = &hwdesc->epib[0];
		p_info.psdata = &hwdesc->psdata[0];
		p_info.data =  org_buf_ptr;
		p_info.core_dev = core_dev;
		p_info.hwdesc = hwdesc;
		core_dev->hw->rx_packet_handler(&p_info);

		kfree(org_buf_ptr);
		knav_pool_desc_put(core_dev->rx_cmd_rsp_pool, hwdesc);
	}
	/* refill the rx response queue fdq */
	pa_rxfdq_refill(core_dev);
	knav_queue_enable_notify(core_dev->rx_cmd_rsp_queue);
	dev_dbg(core_dev->dev, "pa_rx_compl_work_handler - end\n");
}

static int pa_core_init_common_rx_resources(struct pa_core_device *core_dev)
{
	struct knav_queue_notify_config notify_cfg;
	struct device *dev = core_dev->dev;
	struct knav_dma_cfg config;
	u32 last_fdq = 0;
	int i, ret = 0;
	char name[32];

	memset(&config, 0, sizeof(config));
	/* open Rx FDQs */
	for (i = 0; i < KNAV_DMA_FDQ_PER_CHAN &&
	     core_dev->rx_cmd_rsp_queue_depths[i] &&
	     core_dev->rx_cmd_rsp_buffer_sizes[i]; ++i) {
		snprintf(name, sizeof(name), "pa-pdsp-rx-fdq-%d", i);
		core_dev->rx_fdq[i] = knav_queue_open(name, KNAV_QUEUE_GP, 0);
		if (IS_ERR_OR_NULL(core_dev->rx_fdq[i])) {
			ret = PTR_ERR(core_dev->rx_fdq[i]);
			core_dev->rx_fdq[i] = NULL;
			dev_err(dev, "Couldn't open fdq %s\n", name);
			return ret;
		}
	}

	for (i = 0; i < KNAV_DMA_FDQ_PER_CHAN; ++i) {
		if (core_dev->rx_fdq[i])
			last_fdq = knav_queue_get_id(core_dev->rx_fdq[i]);
		config.u.rx.fdq[i] = last_fdq;
	}

	/* Open the PA common response channel */
	core_dev->rx_cmd_rsp_queue = knav_queue_open("pa-pdsp-rx-queue",
						   KNAV_QUEUE_QPEND, 0);
	if (IS_ERR_OR_NULL(core_dev->rx_cmd_rsp_queue)) {
		ret = PTR_ERR(core_dev->rx_cmd_rsp_queue);
		core_dev->rx_cmd_rsp_queue = NULL;
		dev_err(dev, "Couldn't open pa-pdsp-rx-queue\n");
		return ret;
	}

	core_dev->rx_cmd_rsp_queue_id =
			knav_queue_get_id(core_dev->rx_cmd_rsp_queue);
	config.u.rx.dst_q = core_dev->rx_cmd_rsp_queue_id;
	config.direction = DMA_DEV_TO_MEM;
	config.u.rx.einfo_present = true;
	config.u.rx.psinfo_present = true;
	config.u.rx.err_mode = DMA_DROP;
	config.u.rx.desc_type = DMA_DESC_HOST;
	config.u.rx.psinfo_at_sop = false;
	config.u.rx.thresh = DMA_THRESH_NONE;

	core_dev->rx_cmd_rsp_chan =
		knav_dma_open_channel(dev,
				      core_dev->rx_cmd_rsp_chan_name,
				      &config);

	if (IS_ERR_OR_NULL(core_dev->rx_cmd_rsp_chan)) {
		dev_err(dev,
			"Couldnt get PATX LUT-1 cmd channel\n");
		core_dev->rx_cmd_rsp_chan = NULL;
		ret = -ENODEV;
		return ret;
	}

	notify_cfg.fn = pa_rx_chan_notify;
	notify_cfg.fn_arg = core_dev;
	knav_queue_disable_notify(core_dev->rx_cmd_rsp_queue);
	ret = knav_queue_device_control(core_dev->rx_cmd_rsp_queue,
					KNAV_QUEUE_SET_NOTIFIER,
					(unsigned long)&notify_cfg);
	if (ret) {
		dev_err(dev,
			"Couldn't set notifier for rx_cmd_rsp_queue\n");
		return ret;
	}

	tasklet_init(&core_dev->rx_task, pa_rx_compl_work_handler,
		     (unsigned long)core_dev);

	core_dev->cmd_flow_num = knav_dma_get_flow(core_dev->rx_cmd_rsp_chan);
	core_dev->cmd_queue_num = core_dev->rx_cmd_rsp_queue_id;
	dev_dbg(dev, "command receive flow %d, queue %d\n",
		core_dev->cmd_flow_num, core_dev->cmd_queue_num);
	pa_rxfdq_refill(core_dev);
	knav_queue_enable_notify(core_dev->rx_cmd_rsp_queue);

	return ret;
}

static void pa_empty_queue(struct pa_core_device *core_dev, void *queue,
			   bool tx)
{
	struct knav_dma_desc *desc;
	unsigned int dma_sz;
	dma_addr_t dma;
	void *pool = (tx ? core_dev->tx_cmd_pool : core_dev->rx_cmd_rsp_pool);

	for (; ;) {
		dma = knav_queue_pop(queue, &dma_sz);
		if (!dma)
			break;

		desc = knav_pool_desc_unmap(pool, dma, dma_sz);
		if (unlikely(!desc)) {
			dev_err(core_dev->dev, "%s: failed to unmap desc\n",
				__func__);
			continue;
		}
		knav_pool_desc_put(pool, desc);
	}
}

static void pa_core_cleanup_common_rx_resources(struct pa_core_device *core_dev)
{
	int i;

	if (core_dev->rx_cmd_rsp_chan)
		knav_dma_close_channel(core_dev->rx_cmd_rsp_chan);
	if (core_dev->rx_cmd_rsp_queue) {
		pa_rx_compl_work_handler((unsigned long)core_dev);
		knav_queue_disable_notify(core_dev->rx_cmd_rsp_queue);
		knav_queue_close(core_dev->rx_cmd_rsp_queue);
		tasklet_kill(&core_dev->rx_task);
	}

	for (i = 0; i < KNAV_DMA_FDQ_PER_CHAN &&
	     !IS_ERR_OR_NULL(core_dev->rx_fdq[i]); i++) {
		pa_empty_queue(core_dev, core_dev->rx_fdq[i], false);
		knav_queue_close(core_dev->rx_fdq[i]);
	}

	knav_pool_destroy(core_dev->rx_cmd_rsp_pool);
}

static void
pa_core_cleanup_ingress_tx_resources(struct pa_core_device *core_dev,
				     bool l2_cluster)
{
	struct pa_cluster_config *cfg;
	int cluster_id;

	cluster_id = (l2_cluster) ? core_dev->hw->ingress_l2_cluster_id :
				    core_dev->hw->ingress_l3_cluster_id;

	cfg = &core_dev->cluster_config[cluster_id];
	if (cfg->tx_chan)
		knav_dma_close_channel(cfg->tx_chan);
	if (cfg->tx_queue)
		knav_queue_close(cfg->tx_queue);
	if (cfg->tx_cmd_cmpl_queue) {
		/* if there are any completion packets, handle it */
		pa_tx_compl_work_handler((unsigned long)cfg);
		knav_queue_disable_notify(cfg->tx_cmd_cmpl_queue);
		/* Recycle descriptors from completion queue */
		pa_empty_queue(core_dev, cfg->tx_cmd_cmpl_queue, true);
		knav_queue_close(cfg->tx_cmd_cmpl_queue);
		tasklet_kill(&cfg->tx_task);
	}

	/* clean up tx cmd pool last, when l3_cluster resources are cleaned */
	if (core_dev->tx_cmd_pool && !l2_cluster)
		knav_pool_destroy(core_dev->tx_cmd_pool);
}

int pa_core_remove(struct netcp_device *netcp_device, void *inst_priv)
{
	struct pa_core_device *core_dev = inst_priv;

	netcp_txpipe_close(&core_dev->tx_pipe);
	/* clean up ingress L2 cluster tx resources */
	pa_core_cleanup_ingress_tx_resources(core_dev, true);
	/* clean up ingress L3 cluster tx resources */
	pa_core_cleanup_ingress_tx_resources(core_dev, false);
	/* clean up common rx resources */
	pa_core_cleanup_common_rx_resources(core_dev);
	device_remove_file(core_dev->dev,
			   &core_dev->mcast_filter_attr);
	kfree(core_dev->cluster_config);
	kfree(core_dev->fw);
	kfree(core_dev->lut);
	kfree(core_dev->ip_lut);
	kfree(core_dev->version);
	core_dev->hw->unmap_resources(core_dev);
	kfree(core_dev);

	return 0;
}
EXPORT_SYMBOL_GPL(pa_core_remove);

static int pa_add_ip_proto(struct pa_core_device *core_dev, u8 proto)
{
	struct pa_hw *hw = core_dev->hw;
	struct pa_lut_entry *entry;
	int ret;

	/* RX checksum is not supported on NetCP 1.5 for now */
	if (!hw->add_ip_proto)
		return 0;

	entry = pa_core_lut_alloc(core_dev, PA_LUT_IP, 1);
	if (!entry)
		return -1;

	entry->u.ip_proto = proto;

	ret = hw->add_ip_proto(core_dev, entry->index, proto, PACKET_PARSE);
	if (ret)
		dev_err(core_dev->dev, "failed to add IP proto(%d) rule\n",
			proto);
	return ret;
}

static int pa_del_ip_proto(struct pa_core_device *core_dev, u8 proto)
{
	struct pa_hw *hw = core_dev->hw;
	struct pa_lut_entry *entry;
	int idx, ret = 0;

	for (idx = 0; idx < core_dev->ip_lut_size; idx++) {
		entry = core_dev->ip_lut + idx;
		if (!entry->valid || !entry->in_use ||
		    entry->u.ip_proto != proto)
			continue;
		ret = hw->add_ip_proto(core_dev, entry->index, 0, PACKET_DROP);
		if (ret)
			dev_err(core_dev->dev,
				"failed to del IP proto(%d) rule\n", proto);
		entry->in_use = false;
		entry->u.ip_proto = 0;
	}
	return ret;
}

static inline int extract_l4_proto(struct netcp_packet *p_info)
{
	struct sk_buff *skb = p_info->skb;
	int l4_proto = 0;
	__be16 l3_proto;

	l3_proto = skb->protocol;
	if (l3_proto == htons(ETH_P_8021Q)) {
		/* Can't use vlan_eth_hdr() here, skb->mac_header isn't valid */
		struct vlan_ethhdr *vhdr = (struct vlan_ethhdr *)skb->data;

		l3_proto = vhdr->h_vlan_encapsulated_proto;
	}

	switch (l3_proto) {
	case htons(ETH_P_IP):
		l4_proto = ip_hdr(skb)->protocol;
		break;
	case htons(ETH_P_IPV6):
		l4_proto = ipv6_hdr(skb)->nexthdr;
		break;
	default:
		if (unlikely(net_ratelimit())) {
			dev_warn(p_info->netcp->dev,
				 "partial checksum but L3 proto = 0x%04hx!\n",
				 ntohs(l3_proto));
		}
	}

	return l4_proto;
}

static int pa_core_tx_hook(int order, void *data, struct netcp_packet *p_info)
{
	struct pa_intf *pa_intf = data;
	struct netcp_intf *netcp_intf = pa_intf->netcp_intf;
	struct pa_core_device *core_dev = pa_intf->core_dev;
	struct pa_hw *hw = core_dev->hw;
	struct sk_buff *skb = p_info->skb;
	int size, total = 0;

	/* HACK!!! If the upper 24 bits of the skb mark match with match
	 * filter pattern, then drop the packet if the port bit is not
	 * set in the lower 8 bits. This is a hack to prevent multiple
	 * copies of multicast packets in the tx path to switch when
	 * interface is part of a bridge and switch is enabled through sysfs.
	 * In this usecase switch will replicate multicast packets to each
	 * of its slave ports. So avoid duplication before the switch
	 * using this hack. It is expected to be handled in future by
	 * enhancing the driver to support switch device.
	 */
	if (netcp_intf->bridged &&
	    core_dev->enable_mcast_filter && skb->mark &&
	    ((skb->mark & core_dev->mask) == core_dev->match) &&
	    ((skb->mark & BIT(pa_intf->eth_port - 1)) == 0)) {
		return NETCP_TX_DROP;
	}

	/* Generate the next route command */
	size = hw->fmtcmd_next_route(p_info, pa_intf->eth_port);
	if (unlikely(size < 0))
		return size;

	total += size;
	/* If TX Timestamp required, request it */
	if (unlikely(pa_intf->tx_timestamp_enable &&
		     !core_dev->disable_hw_tstamp &&
		     (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
		     !(skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS)))
		total += hw->do_tx_timestamp(core_dev, p_info);

	/* If checksum offload required, request it */
	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		int l4_proto;

		l4_proto = extract_l4_proto(p_info);
		switch (l4_proto) {
		case IPPROTO_TCP:
		case IPPROTO_UDP:
			size = hw->fmtcmd_tx_csum(p_info);
			break;
		default:
			if (unlikely(net_ratelimit())) {
				dev_warn(p_info->netcp->dev,
					 "partial checksum but L4 proto = %d!\n",
					 l4_proto);
			}
			size = 0;
			break;
		}

		if (unlikely(size < 0))
			return size;
		total += size;
	}

	/* The next hook may require the command stack to be 8-byte aligned */
	size = netcp_align_psdata(p_info, 8);
	if (unlikely(size < 0))
		return size;

	if (size > 0) {
		size = hw->fmtcmd_align(p_info, size);
		if (unlikely(size < 0))
			return size;
		total += size;
	}

	/* psdata_len is in words */
	if (total)
		p_info->psdata_len = total >> 2;
	p_info->tx_pipe = &pa_intf->tx_pipe;
	return 0;
}

static int pa_core_rx_hook(int order, void *data, struct netcp_packet *p_info)
{
	struct pa_intf *pa_intf = data;
	struct pa_core_device *core_dev = pa_intf->core_dev;
	struct pa_hw *hw = core_dev->hw;

	dev_dbg(core_dev->dev, "pa_rx_hook, pa_dev->disable_hw_tstamp %d\n",
		core_dev->disable_hw_tstamp);
	/* Timestamping on Rx packets */
	if (!unlikely(core_dev->disable_hw_tstamp))
		hw->rx_timestamp_hook(pa_intf, p_info);

	/* Checksum offload on Rx packets */
	if (core_dev->netif_features & NETIF_F_RXCSUM)
		hw->rx_checksum_hook(p_info);

	return 0;
}

int pa_core_close(void *intf_priv,
		  struct net_device *ndev)
{
	struct pa_intf *pa_intf = (struct pa_intf *)intf_priv;
	struct pa_core_device *core_dev = pa_intf->core_dev;
	struct netcp_intf *netcp_priv = netdev_priv(ndev);
	struct pa_hw *hw = core_dev->hw;

	netcp_unregister_txhook(netcp_priv, PA_TXHOOK_ORDER,
				pa_core_tx_hook, pa_intf);

	if ((core_dev->netif_features & NETIF_F_RXCSUM) ||
	    (!core_dev->disable_hw_tstamp))
		netcp_unregister_rxhook(netcp_priv, PA_RXHOOK_ORDER,
					pa_core_rx_hook, pa_intf);

	/* De-Configure the streaming switch */
	mutex_lock(&hw->module_lock);
	hw->set_streaming_switch(core_dev,
				 pa_intf->eth_port,
				 pa_intf->saved_ss_state);

	if (!--core_dev->inuse_if_count) {
		/* Do pa disable related stuff only if this is the last
		 * interface to go down
		 */
		if (core_dev->netif_features & NETIF_F_RXCSUM) {
			pa_del_ip_proto(core_dev, IPPROTO_TCP);
			pa_del_ip_proto(core_dev, IPPROTO_UDP);
		}
		if (hw->cleanup)
			hw->cleanup(core_dev);
	}
	mutex_unlock(&hw->module_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(pa_core_close);

int pa_core_open(void *intf_priv,
		 struct net_device *ndev)
{
	struct pa_intf *pa_intf = (struct pa_intf *)intf_priv;
	struct netcp_intf *netcp_priv = netdev_priv(ndev);
	struct pa_core_device *core_dev = pa_intf->core_dev;
	struct pa_hw *hw = core_dev->hw;
	int i, ret;

	mutex_lock(&hw->module_lock);
	/* The first time an open is being called */
	dev_dbg(core_dev->dev, "pa_open() called for port: %d\n",
		pa_intf->eth_port);

	pa_intf->data_flow_num = knav_dma_get_flow(netcp_priv->rx_channel);
	pa_intf->data_queue_num = netcp_priv->rx_queue_id;

	dev_dbg(core_dev->dev, "configuring data receive flow %d, queue %d\n",
		pa_intf->data_flow_num, pa_intf->data_queue_num);

	if (++core_dev->inuse_if_count == 1) {
		if (hw->config_exception_route) {
			ret = hw->config_exception_route(core_dev);
			if (ret < 0)
				goto fail;
		}

		if (core_dev->netif_features & NETIF_F_RXCSUM) {
			/* make lut entries invalid */
			for (i = 0; i < core_dev->lut_size; i++) {
				if (!core_dev->lut[i].valid)
					continue;
				hw->add_mac_rule(pa_intf, i, NULL, NULL,
						 PACKET_DROP, 0,
						 PA_INVALID_PORT);
			}

			/* make IP LUT entries invalid */
			for (i = 0; i < core_dev->ip_lut_size; i++) {
				if (!core_dev->ip_lut[i].valid)
					continue;
				hw->add_ip_proto(core_dev, i, 0, PACKET_DROP);
			}

			/* if Rx checksum is enabled, add IP LUT entries for
			 * Rx checksumming
			 */
			ret = pa_add_ip_proto(core_dev, IPPROTO_TCP);
			if (ret)
				goto fail;
			ret = pa_add_ip_proto(core_dev, IPPROTO_UDP);
			if (ret)
				goto fail;
		}
	}

	pa_intf->saved_ss_state =
		hw->get_streaming_switch(core_dev, pa_intf->eth_port);
	dev_dbg(core_dev->dev, "saved_ss_state for port %d is %d\n",
		pa_intf->eth_port, pa_intf->saved_ss_state);

	/* Configure the streaming switch */
	hw->set_streaming_switch(core_dev, pa_intf->eth_port,
				 hw->streaming_pdsp);
	mutex_unlock(&hw->module_lock);

	netcp_register_txhook(netcp_priv, PA_TXHOOK_ORDER,
			      pa_core_tx_hook, pa_intf);

	if ((!core_dev->disable_hw_tstamp) ||
	    (core_dev->netif_features & NETIF_F_RXCSUM))
		netcp_register_rxhook(netcp_priv, PA_RXHOOK_ORDER,
				      pa_core_rx_hook, pa_intf);
	return 0;

fail:
	mutex_unlock(&hw->module_lock);
	pa_core_close(pa_intf, ndev);
	return ret;
}
EXPORT_SYMBOL_GPL(pa_core_open);

static ssize_t mcast_filter_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct pa_core_device *pa_core_dev = mcast_filter_attr_to_pa(attr);
	int len = 0;

	if (!pa_core_dev->match && !pa_core_dev->mask)
		return len;

	len = snprintf(buf, SZ_4K, "0x%x, 0x%x\n", pa_core_dev->match,
		       pa_core_dev->mask);
	return len;
}

static ssize_t mcast_filter_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct pa_core_device *pa_core_dev = mcast_filter_attr_to_pa(attr);
	int len;

	len = sscanf(buf, "%8x %8x", &pa_core_dev->match, &pa_core_dev->mask);
	if (len != 2)
		return -EINVAL;

	if (!pa_core_dev->mask && !pa_core_dev->match) {
		pa_core_dev->enable_mcast_filter = false;
		return count;
	}
	pa_core_dev->enable_mcast_filter = true;

	return count;
}

DEVICE_ATTR(mcast_filter, S_IRUGO | S_IWUSR,
	    mcast_filter_show, mcast_filter_store);

static void *pa_core_init(struct netcp_device *netcp_device,
			  struct device *dev,
			  struct device_node *node,
			  int size, int *error,
			  struct pa_hw *hw,
			  const char *firmwares[][PA_MAX_FIRMWARES])
{
	struct pa_core_device *core_dev;
	unsigned long pa_rate;
	u64 max_sec;

	core_dev = kzalloc(size, GFP_KERNEL);
	if (!core_dev) {
		*error = -ENOMEM;
		return NULL;
	}

	core_dev->netcp_device = netcp_device;
	core_dev->dev = dev;
	core_dev->hw = hw;

	core_dev->mcast_filter_attr = dev_attr_mcast_filter;
	sysfs_attr_init(&core_dev->mcast_filter_attr.attr);

	*error = device_create_file(core_dev->dev,
				    &core_dev->mcast_filter_attr);
	if (*error) {
		kfree(core_dev);
		return NULL;
	}

	core_dev->netif_features = (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM);
	if (hw->features & PA_RX_CHECKSUM)
		core_dev->netif_features |= NETIF_F_RXCSUM;

	core_dev->cluster_config =
		kzalloc(sizeof(struct pa_cluster_config) *
			hw->num_clusters, GFP_KERNEL);
	core_dev->fw =
		kzalloc(sizeof(struct firmware *) * hw->num_pdsps, GFP_KERNEL);

	core_dev->version = kzalloc(PA_SIZE_VERSION * hw->num_pdsps,
				    GFP_KERNEL);
	if (!core_dev->cluster_config || !core_dev->fw ||
	    !core_dev->version) {
		*error = -ENOMEM;
		goto cleanup;
	}

	*error = pa_core_get_dt_bindings(core_dev, node);
	if (*error < 0)
		goto cleanup;

	*error = hw->map_resources(core_dev, node);
	if (*error < 0)
		goto cleanup;

	if (hw->features & PA_TIMESTAMP) {
		core_dev->clk = clk_get(core_dev->dev, "pa_clk");
		if (IS_ERR(core_dev->clk)) {
			*error = PTR_ERR(core_dev->clk);
			goto cleanup;
		}

		pa_rate = clk_get_rate(core_dev->clk) / 2;
		clk_put(core_dev->clk);
		/* calculate the multiplier/shift to
		 * convert PA counter ticks to ns.
		 */
		max_sec = ((1ULL << 48) - 1) + (pa_rate - 1);
		do_div(max_sec, pa_rate);

		clocks_calc_mult_shift(&core_dev->timestamp_info.mult,
				       &core_dev->timestamp_info.shift, pa_rate,
				       NSEC_PER_SEC, max_sec);

		core_dev->timestamp_info.system_offset = 0;
		dev_info(core_dev->dev, "pa_clk_rate(%lu HZ),mult(%u),shift(%u)\n",
			 pa_rate, core_dev->timestamp_info.mult,
			 core_dev->timestamp_info.shift);
	}

	hw->pre_init(core_dev);
	*error = pa_core_load_n_start_pdsps(core_dev, firmwares);
	if (*error)
		goto cleanup;

	*error = hw->post_init(core_dev);
	if (*error)
		goto cleanup;

	kfree(core_dev->version);
	core_dev->version = NULL;
	*error = pa_core_setup_desc_pools(core_dev);
	if (*error)
		goto cleanup;

	*error = pa_core_init_egress_tx_pipe(core_dev, netcp_device);
	if (*error)
		goto cleanup;

	/* initialize tx resources for L2 ingress cluster */
	*error = pa_core_init_ingress_tx_resources(core_dev, true);
	if (*error)
		goto cleanup;

	/* initialize tx resources for L3 ingress cluster */
	*error = pa_core_init_ingress_tx_resources(core_dev, false);
	if (*error)
		goto cleanup;

	/* initialize common rx resources */
	*error = pa_core_init_common_rx_resources(core_dev);
	if (*error)
		goto cleanup;

	spin_lock_init(&core_dev->lock);
	mutex_init(&hw->module_lock);

	return core_dev;

cleanup:
	pa_core_remove(NULL, core_dev);
	return NULL;
}

static void __iomem *pa_core_map_resource(struct device *dev,
					  struct device_node *node, int index)
{
	struct resource res;
	void __iomem *reg;
	int ret = 0;

	ret = of_address_to_resource(node, index, &res);
	if (ret) {
		dev_err(dev,
			"Can't translate of pa node(%s) address at index %d\n",
			node->name, index);
		return NULL;
	}

	reg = ioremap_nocache(res.start, resource_size(&res));
	if (!reg)
		dev_err(dev, "Failed to map register res %pR\n", &res);
	return reg;
}

int pa_core_attach(void *inst_priv, struct net_device *ndev,
		   struct device_node *node, void **intf_priv)
{
	struct pa_core_device *core_dev = inst_priv;
	struct netcp_intf *netcp_priv = netdev_priv(ndev);
	struct pa_intf *pa_intf;
	int ret;

	pa_intf = kzalloc(sizeof(*pa_intf), GFP_KERNEL);
	if (!pa_intf)
		return -ENOMEM;

	pa_intf->net_device = ndev;
	pa_intf->core_dev = core_dev;
	pa_intf->tx_pipe = core_dev->tx_pipe;
	pa_intf->netcp_intf = netcp_priv;

	if (of_property_read_u32(node, "slave-port", &pa_intf->eth_port)) {
		dev_err(core_dev->dev, "missing slave-port parameter\n");
		kfree(pa_intf);
		return -EINVAL;
	}
	/* eth_port is slave-port + 1 */
	pa_intf->eth_port++;

	/* override the rx channel for the interface */
	ret = of_property_read_string(node, "rx-channel",
				      &pa_intf->rx_chan_name);
	if (ret < 0)
		pa_intf->rx_chan_name = NULL;

	if (pa_intf->rx_chan_name)
		dev_info(core_dev->dev, "rx_chan_name %s for port %d\n",
			 pa_intf->rx_chan_name,
			 pa_intf->eth_port);

	if (pa_intf->rx_chan_name) {
		pa_intf->saved_rx_channel = netcp_priv->dma_chan_name;
		netcp_priv->dma_chan_name = pa_intf->rx_chan_name;
	}

	*intf_priv = pa_intf;

	rtnl_lock();
	ndev->features		|= core_dev->netif_features;
	ndev->hw_features	|= core_dev->netif_features;
	ndev->wanted_features	|= core_dev->netif_features;
	netdev_update_features(ndev);
	rtnl_unlock();

	return 0;
}
EXPORT_SYMBOL_GPL(pa_core_attach);

int pa_core_release(void *intf_priv)
{
	struct pa_intf *pa_intf = (struct pa_intf *)intf_priv;
	struct pa_core_device *core_dev = pa_intf->core_dev;
	struct net_device *ndev = pa_intf->net_device;
	struct pa_hw *hw = core_dev->hw;

	mutex_lock(&hw->module_lock);
	if (pa_intf->saved_rx_channel)
		pa_intf->netcp_intf->dma_chan_name = pa_intf->saved_rx_channel;
	mutex_unlock(&hw->module_lock);

	if (!--core_dev->inuse_if_count) {
		rtnl_lock();
		ndev->features		&= ~core_dev->netif_features;
		ndev->hw_features	&= ~core_dev->netif_features;
		ndev->wanted_features	&= ~core_dev->netif_features;
		netdev_update_features(ndev);
		rtnl_unlock();
	}
	kfree(pa_intf);

	return 0;
}
EXPORT_SYMBOL_GPL(pa_core_release);

static struct pa_packet *pa_alloc_packet(struct pa_core_device *core_dev,
					 unsigned cmd_size,
					 unsigned int cluster_id)
{
	struct pa_cluster_config *cfg = &core_dev->cluster_config[cluster_id];
	struct pa_packet *p_info;
	unsigned int buflen;

	buflen =  sizeof(*p_info) + cmd_size;

	dev_dbg(core_dev->dev, "buflen = %d(%x), cmd_size %d\n",
		buflen, buflen, cmd_size);
	p_info = kzalloc(buflen, GFP_ATOMIC | GFP_DMA | __GFP_COLD);
	if (!p_info)
		return NULL;

	p_info->tx_queue = cfg->tx_queue;
	/* Allocate descriptor */
	p_info->hwdesc = knav_pool_desc_get(core_dev->tx_cmd_pool);
	if (IS_ERR_OR_NULL(p_info->hwdesc)) {
		kfree(p_info);
		dev_dbg(core_dev->dev, "out of tx cmd pool desc\n");
		return NULL;
	}

	p_info->core_dev = core_dev;
	p_info->data = p_info + 1;
	p_info->data_sz = cmd_size;
	p_info->epib = p_info->hwdesc->epib;
	p_info->psdata = p_info->hwdesc->psdata;
	/* by default set the psdata len to 1 word since it just
	 * have the cmd type
	 */
	p_info->psdata_len = sizeof(u32) / 4;

	return p_info;
}

static void pa_core_free_packet(struct pa_core_device *core_dev,
				struct pa_packet *p_info)
{
	knav_pool_desc_unmap(core_dev->tx_cmd_pool,
			     p_info->dma_desc, p_info->dma_desc_sz);
	knav_pool_desc_put(core_dev->tx_cmd_pool, p_info->hwdesc);
	kfree(p_info);
}

static int pa_core_submit_packet(struct pa_packet *p_info,
				 unsigned int cluster_id)
{
	struct pa_core_device *core_dev = p_info->core_dev;
	struct pa_cluster_config *cfg = &core_dev->cluster_config[cluster_id];
	u32 tmp = 0;

	tmp |= (p_info->psdata_len & KNAV_DMA_DESC_PSLEN_MASK) <<
		KNAV_DMA_DESC_PSLEN_SHIFT;

	tmp |= KNAV_DMA_DESC_HAS_EPIB |
	       ((cfg->tx_cmd_cmpl_queue_id & KNAV_DMA_DESC_RETQ_MASK) <<
	       KNAV_DMA_DESC_RETQ_SHIFT);

	knav_dma_set_words(&tmp, 1, &p_info->hwdesc->packet_info);
	/* save p_info in pad to do completion */
	knav_dma_set_words((u32 *)&p_info, 1, &p_info->hwdesc->pad[0]);

	p_info->dma_data = dma_map_single(core_dev->dev,
					  p_info->data, p_info->data_sz,
					  DMA_TO_DEVICE);
	if (dma_mapping_error(core_dev->dev, p_info->dma_data)) {
		dev_err(core_dev->dev, "failed to map buffer\n");
		return -ENOMEM;
	}
	knav_dma_set_pkt_info(p_info->dma_data, p_info->data_sz, 0,
			      p_info->hwdesc);
	knav_dma_set_words(&p_info->data_sz, 1, &p_info->hwdesc->desc_info);

	knav_pool_desc_map(core_dev->tx_cmd_pool, p_info->hwdesc,
			   sizeof(*p_info->hwdesc),
			   &p_info->dma_desc, &p_info->dma_desc_sz);

	knav_queue_push(p_info->tx_queue, p_info->dma_desc,
			p_info->dma_desc_sz, 0);
	return 0;
}

static void pa_core_load_firmware(void __iomem *dest,
				  const unsigned int *src, u32 wc)
{
	int i;

	for (i = 0; i < wc; i++)
		writel_relaxed(be32_to_cpu(*(src + i)), dest + (i * 4));
}

#define PA_CORE_MAX_LUT_ENTRY		3
static inline int pa_lut_entry_count(enum netcp_addr_type type)
{
	return (type == ADDR_DEV || type == ADDR_UCAST || type == ADDR_ANY) ?
		PA_CORE_MAX_LUT_ENTRY : 1;
}

int pa_core_add_addr(void *intf_priv, struct netcp_addr *naddr)
{
	struct pa_intf *pa_intf = (struct pa_intf *)intf_priv;
	struct pa_core_device *core_dev = pa_intf->core_dev;
	struct pa_hw *hw = core_dev->hw;
	int count = pa_lut_entry_count(naddr->type);
	struct pa_lut_entry *entries[PA_CORE_MAX_LUT_ENTRY];
	unsigned long flag;
	int idx, error, port = pa_intf->eth_port;
	const u8 *addr;

	dev_dbg(core_dev->dev, "pa_add_addr, port %d\n", pa_intf->eth_port);
	if (!core_dev->inuse_if_count)
		return -ENXIO;

	spin_lock_irqsave(&core_dev->lock, flag);
	for (idx = 0; idx < count; idx++) {
		entries[idx] = pa_core_lut_alloc(core_dev, PA_LUT_MAC,
						 naddr->type == ADDR_ANY);
		if (!entries[idx]) {
			error = -ENOMEM;
			goto fail_alloc;
		}
		entries[idx]->u.naddr = naddr;
	}

	addr = (naddr->type == ADDR_ANY) ? NULL : naddr->addr;
	idx = 0;

	if (naddr->type == ADDR_ANY) {
		error = hw->add_mac_rule(pa_intf, entries[idx++]->index,
					 NULL, addr, PACKET_HST, 0, port);
		if (error)
			goto fail_rule;
	}

	if (count > 1) {
		error = hw->add_mac_rule(pa_intf, entries[idx++]->index,
					 NULL, addr, PACKET_PARSE,
					 0x0800, port);
		if (error)
			goto fail_rule;

		error = hw->add_mac_rule(pa_intf, entries[idx++]->index,
					 NULL, addr, PACKET_PARSE,
					 0x86dd, port);
		if (error)
			goto fail_rule;
	}

	if (naddr->type != ADDR_ANY)
		error = hw->add_mac_rule(pa_intf, entries[idx++]->index,
					 NULL, addr, PACKET_HST, 0,
					 port);
fail_rule:
	spin_unlock_irqrestore(&core_dev->lock, flag);

	return error;

fail_alloc:
	spin_unlock_irqrestore(&core_dev->lock, flag);
	for (idx--; idx >= 0; idx--)
		entries[idx]->in_use = false;
	return error;
}
EXPORT_SYMBOL_GPL(pa_core_add_addr);

int pa_core_del_addr(void *intf_priv, struct netcp_addr *naddr)
{
	struct pa_intf *pa_intf = intf_priv;
	struct pa_core_device *core_dev = pa_intf->core_dev;
	struct pa_hw *hw = core_dev->hw;
	struct pa_lut_entry *entry;
	unsigned long flag;
	int idx;

	if (!core_dev->inuse_if_count)
		return -ENXIO;

	spin_lock_irqsave(&core_dev->lock, flag);
	for (idx = 0; idx < core_dev->lut_size; idx++) {
		entry = core_dev->lut + idx;
		if (!entry->valid || !entry->in_use || entry->u.naddr != naddr)
			continue;
		hw->add_mac_rule(pa_intf, entry->index, NULL, NULL,
				 PACKET_DROP, 0, PA_INVALID_PORT);
		entry->in_use = false;
		entry->u.naddr = NULL;
	}
	spin_unlock_irqrestore(&core_dev->lock, flag);

	return 0;
}
EXPORT_SYMBOL_GPL(pa_core_del_addr);

struct pa_core_ops netcp_pa_core_ops = {
	.init = pa_core_init,
	.map_resource = pa_core_map_resource,
	.alloc_packet = pa_alloc_packet,
	.free_packet = pa_core_free_packet,
	.submit_packet = pa_core_submit_packet,
	.alloc_lut = pa_core_lut_alloc,
	.load_firmware = pa_core_load_firmware,
};
EXPORT_SYMBOL_GPL(netcp_pa_core_ops);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI NETCP Packet Accelerator Core driver for Keystone SOCs");
MODULE_AUTHOR("Murali Karicheri <m-karicheri2@ti.com");
