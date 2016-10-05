/*
 * Keystone NetCP PA (Packet Accelerator) Core Driver includes
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
#ifndef NETCP_CORE_PA_H
#define NETCP_CORE_PA_H

#define PA_TXHOOK_ORDER				10
#define PA_RXHOOK_ORDER				10
#define PA_MAX_NUM_INTERFACES			8

/* must match with firmware header */
#define PA_SIZE_VERSION				16
#define PA_MAX_FIRMWARES			10

#define PACKET_DROP				 0
#define PACKET_PARSE				 1
#define PACKET_HST				 2
#define PA_INVALID_PORT				0xff

enum pa_lut_type {
	PA_LUT_MAC,
	PA_LUT_IP
};

struct pa_timestamp_info {
	u32	mult;
	u32	shift;
	u64	system_offset;
};

struct pa_lut_entry {
	int			index;
	bool			valid, in_use;
	union {
		struct netcp_addr	*naddr;
		u8			ip_proto;
	} u;
};

struct pa_intf {
	struct pa_core_device	*core_dev;
	struct net_device	*net_device;
	bool			tx_timestamp_enable;
	bool			rx_timestamp_enable;
	struct netcp_tx_pipe	tx_pipe;
	unsigned		data_flow_num;
	unsigned		data_queue_num;
	unsigned		eth_port;
	const char		*rx_chan_name;
	u32			saved_ss_state;
	const char		*saved_rx_channel;
	struct netcp_intf	*netcp_intf;
};

struct pa_packet {
	void				*data;
	dma_addr_t			dma_data;
	unsigned int			data_sz;
	u32				*epib;
	u32				*psdata;
	unsigned int			psdata_len;
	struct knav_dma_desc		*hwdesc;
	/* dma address and size of the descriptor */
	dma_addr_t			dma_desc;
	unsigned			dma_desc_sz;
	void				*tx_queue;
	struct pa_core_device		*core_dev;
};

struct pa_cluster_config {
	unsigned int		tx_queue_id;
	void			*tx_queue;
	void			*tx_chan;
	const char		*tx_chan_name;
	void			*tx_cmd_cmpl_queue;
	unsigned		tx_cmd_cmpl_queue_id;
	struct pa_core_device	*core_dev;
	struct tasklet_struct	tx_task;
};

struct pa_hw {
	/* hw functions implemented by PA hw driver module */
	int (*fmtcmd_next_route)(struct netcp_packet *p_info, int eth_port);
	int (*do_tx_timestamp)(struct pa_core_device *core_dev,
			       struct netcp_packet *p_info);
	int (*fmtcmd_tx_csum)(struct netcp_packet *p_info);
	int (*fmtcmd_align)(struct netcp_packet *p_info, const unsigned size);
	void (*rx_checksum_hook)(struct netcp_packet *p_info);
	void (*rx_timestamp_hook)(struct pa_intf *pa_intf,
				  struct netcp_packet *p_info);
	int (*map_resources)(struct pa_core_device *core_dev,
			     struct device_node *node);
	void (*unmap_resources)(struct pa_core_device *core_dev);
	int (*set_firmware)(struct pa_core_device *core_dev,
			    int pdsp, const unsigned int *buffer, int len);
	void	(*pre_init)(struct pa_core_device *core_dev);
	int	(*post_init)(struct pa_core_device *core_dev);
	void	(*rx_packet_handler)(void *param);
	int	(*add_mac_rule)(struct pa_intf *pa_intf, int index,
				const u8 *smac, const u8 *dmac,
				int rule, unsigned etype, int port);
	int	(*config_exception_route)(struct pa_core_device *core_dev);
	int	(*add_ip_proto)(struct pa_core_device *core_dev, int index,
				u8 proto, int rule);
	u32	(*set_streaming_switch)(struct pa_core_device *core_dev,
					int port, u32 new_value);
	u32	(*get_streaming_switch)(struct pa_core_device *core_dev,
					int port);
	void	(*cleanup)(struct pa_core_device *core_dev);

	/* hw features */
#define PA_RX_CHECKSUM		BIT(0)
#define PA_TIMESTAMP		BIT(1)
	u32 features;
	int num_clusters;
	int num_pdsps;
	int ingress_l2_cluster_id;
	int ingress_l3_cluster_id;
	int egress_cluster_id;
	int streaming_pdsp;
	/* To protect against concurrent accesses */
	struct mutex module_lock;
};

struct pa_core_device {
	bool disable_hw_tstamp;
	/* mask to be used to extract valid mark bit for checking */
	u32 mask;
	/* match pattern to use for comparison to mark value  */
	u32 match;
	/* bool to enable/disable special filtering based on above
	 * mask, match values
	 */
	bool enable_mcast_filter;
	struct pa_timestamp_info timestamp_info;
	struct netcp_tx_pipe tx_pipe;
	struct netcp_device *netcp_device;
	struct device *dev;
	struct clk *clk;
	struct device_attribute mcast_filter_attr;

	/* rx side */
	void *rx_cmd_rsp_chan;
	const char *rx_cmd_rsp_chan_name;
	unsigned int rx_cmd_rsp_queue_id;
	void *rx_cmd_rsp_queue;
	void *rx_test_queue;
	u32 rx_cmd_rsp_queue_depths[KNAV_DMA_FDQ_PER_CHAN];
	u32 rx_cmd_rsp_buffer_sizes[KNAV_DMA_FDQ_PER_CHAN];
	void *rx_fdq[KNAV_DMA_FDQ_PER_CHAN];
	void *rx_cmd_rsp_pool;
	u32 rx_cmd_rsp_pool_size;
	u32 rx_cmd_rsp_pool_region_id;
	unsigned rx_flow_base;
	unsigned rx_queue_base;
	struct tasklet_struct rx_task;

	/* common tx side */
	void *tx_cmd_pool;
	u32 tx_cmd_pool_size;
	u32 tx_cmd_pool_region_id;
	u32 tx_cmd_queue_depth;
	unsigned cmd_flow_num;
	unsigned cmd_queue_num;

	/* to protect against concurrent updates of addresses */
	spinlock_t lock;
	u32 inuse_if_count;
	u32 lut_inuse_count;
	struct pa_lut_entry *lut;
	u32 lut_size;
	struct pa_lut_entry	*ip_lut;
	u32 ip_lut_size;
	struct pa_cluster_config *cluster_config;
	char *version;
	const struct firmware **fw;
	struct pa_hw *hw;
	netdev_features_t netif_features;
};

int pa_core_release(void *intf_priv);
int pa_core_add_addr(void *intf_priv, struct netcp_addr *naddr);
int pa_core_del_addr(void *intf_priv, struct netcp_addr *naddr);
int pa_core_open(void *intf_priv, struct net_device *ndev);
int pa_core_close(void *intf_priv, struct net_device *ndev);
int pa_core_attach(void *inst_priv, struct net_device *ndev,
		   struct device_node *node, void **intf_priv);
int pa_core_remove(struct netcp_device *netcp_device, void *inst_priv);

struct pa_core_ops {
	void *(*init)(struct netcp_device *netcp_device,
		      struct device *dev, struct device_node *node,
		      int size, int *error,
		      struct pa_hw *hw,
		      const char *firmwares[][PA_MAX_FIRMWARES]);
	void __iomem *(*map_resource)(struct device *dev,
				      struct device_node *node, int index);
	struct pa_packet *(*alloc_packet)(struct pa_core_device *core_dev,
					  unsigned cmd_size,
					  unsigned int cluster_id);
	void (*free_packet)(struct pa_core_device *pa_dev,
			    struct pa_packet *p_info);
	int (*submit_packet)(struct pa_packet *p_info,
			     unsigned int cluster_id);
	struct pa_lut_entry *(*alloc_lut)(struct pa_core_device *core_dev,
					  enum pa_lut_type type,
					  bool backwards);
	void (*load_firmware)(void __iomem *dest, const unsigned int *src,
			      u32 wc);
};

extern struct pa_core_ops netcp_pa_core_ops;

#endif
