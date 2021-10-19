/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Subhajit Paul <subhajit_paul@ti.com>
 */

#ifndef __RPMSG_REMOTEDEV_H__
#define __RPMSG_REMOTEDEV_H__

#include <linux/etherdevice.h>

struct rpmsg_remotedev;

/* Defines for demo device */
#define REMOTEDEV_DEMO_MSG_LEN		(128)

/* Defines for display device */
#define RPMSG_REMOTEDEV_DISPLAY_MAX_PLANES		(3)
#define RPMSG_REMOTEDEV_DISPLAY_MAX_DISPS		(8)
#define RPMSG_REMOTEDEV_DISPLAY_MAX_PIPES		(8)
#define RPMSG_REMOTEDEV_DISPLAY_MAX_FORMATS		(32)
#define RPMSG_REMOTEDEV_DISPLAY_MAX_ZORDERS		(8)

/* Struct definitions for display device */
struct rpmsg_remotedev_display_buffer {
	uint32_t width;
	uint32_t height;
	uint32_t format;
	uint32_t num_planes;
	dma_addr_t planes[RPMSG_REMOTEDEV_DISPLAY_MAX_PLANES];
	uint32_t pitches[RPMSG_REMOTEDEV_DISPLAY_MAX_PLANES];
	struct rpmsg_remotedev *rdev;
	void *priv;
};

struct rpmsg_remotedev_display_pipe_update {
	uint32_t pipe_id;
	bool enabled;
	uint32_t dst_w;
	uint32_t dst_h;
	uint32_t dst_x;
	uint32_t dst_y;
	struct rpmsg_remotedev_display_buffer *buffer;
};

struct rpmsg_remotedev_display_commit {
	uint32_t disp_id;
	uint32_t num_pipe_updates;
	struct rpmsg_remotedev_display_pipe_update pipes[RPMSG_REMOTEDEV_DISPLAY_MAX_PIPES];
	struct rpmsg_remotedev *rdev;
	void *priv;
};

struct rpmsg_remotedev_display_pipe {
	uint32_t pipe_id;
	bool can_scale;
	bool can_mod_win;
	uint32_t fixed_win_x;
	uint32_t fixed_win_y;
	uint32_t fixed_win_w;
	uint32_t fixed_win_h;
	uint32_t initial_zorder;
	uint32_t num_formats;
	uint32_t formats[RPMSG_REMOTEDEV_DISPLAY_MAX_FORMATS];
	uint32_t num_allowed_zorders;
	uint32_t allowed_zorders[RPMSG_REMOTEDEV_DISPLAY_MAX_ZORDERS];
};

struct rpmsg_remotedev_display_disp {
	uint32_t disp_id;
	uint32_t width;
	uint32_t height;
	uint32_t refresh;
	uint32_t num_pipes;
	struct rpmsg_remotedev_display_pipe pipes[RPMSG_REMOTEDEV_DISPLAY_MAX_PIPES];
};

struct rpmsg_remotedev_display_resinfo {
	uint32_t num_disps;
	struct rpmsg_remotedev_display_disp disps[RPMSG_REMOTEDEV_DISPLAY_MAX_DISPS];
};

/* callbacks for demo device */
struct rpmsg_remotedev_demo_cb {
	void (*s2c_message)(void *data, ssize_t len, void *cb_data);
};

/* callbacks for display device */
struct rpmsg_remotedev_display_cb {
	void (*commit_done)(struct rpmsg_remotedev_display_commit *commit, void *cb_data);
	void (*buffer_done)(struct rpmsg_remotedev_display_buffer *buffer, void *cb_data);
};

/* requests for demo device */
struct rpmsg_remotedev_demo_ops {
	int (*get_data)(struct rpmsg_remotedev *rdev, void *data, ssize_t len);
	int (*ping)(struct rpmsg_remotedev *rdev, void *ping_data, ssize_t ping_len);
	int (*c2s_message)(struct rpmsg_remotedev *rdev, void *c2s_msg_data, ssize_t len);
};

/* requests for display device */
struct rpmsg_remotedev_display_ops {
	bool (*ready)(struct rpmsg_remotedev *rdev);
	int (*get_res_info)(struct rpmsg_remotedev *rdev, struct rpmsg_remotedev_display_resinfo *res);
	int (*commit)(struct rpmsg_remotedev *rdev, struct rpmsg_remotedev_display_commit *commit);
};

#define RPMSG_RDEV_ETHSWITCH_CPSW_PRIORITY_NUM   (8)

/* features bits */
#define RPMSG_KDRV_ETHSWITCH_FEATURE_TXCSUM BIT(0)
#define RPMSG_KDRV_ETHSWITCH_FEATURE_DUMP_STATS BIT(1)
#define RPMSG_KDRV_ETHSWITCH_FEATURE_MAC_ONLY BIT(2)
#define RPMSG_KDRV_ETHSWITCH_FEATURE_MC_FILTER BIT(3)

struct rpmsg_rdev_eth_switch_attach_info {
	/* MTU of rx packet */
	u32 rx_mtu;
	/* MTU of tx packet per priority */
	u32 tx_mtu[RPMSG_RDEV_ETHSWITCH_CPSW_PRIORITY_NUM];
	/* Supported Features mask */
	u32 features;
	u32 mac_only_port;
};

struct rpmsg_rdev_eth_switch_attach_ext_info {
	/* MTU of rx packet */
	u32 rx_mtu;
	/* MTU of tx packet per priority */
	u32 tx_mtu[RPMSG_RDEV_ETHSWITCH_CPSW_PRIORITY_NUM];
	/* Supported Features mask */
	u32 features;
	u32 flow_idx;
	u32 tx_cpsw_psil_dst_id;
	u8 mac_addr[ETH_ALEN];
	u32 mac_only_port;
};

struct rpmsg_rdev_eth_switch_tx_info {
	/* Tx PSIL Peer destination thread id */
	u32 tx_cpsw_psil_dst_id;
};

struct rpmsg_rdev_eth_switch_rx_info {
	/* Allocated flow's index */
	u32 flow_idx;
};

struct rpmsg_remotedev_eth_switch_ops {
	void (*get_fw_ver)(struct rpmsg_remotedev *rdev,
			   char *buf, size_t size);
	int (*attach)(struct rpmsg_remotedev *rdev,
		      struct rpmsg_rdev_eth_switch_attach_info *attach_info);
	int (*attach_ext)(struct rpmsg_remotedev *rdev,
			  struct rpmsg_rdev_eth_switch_attach_ext_info *attach_ext_info);
	int (*detach)(struct rpmsg_remotedev *rdev);
	int (*get_tx_info)(struct rpmsg_remotedev *rdev,
			   struct rpmsg_rdev_eth_switch_tx_info *info);
	int (*get_rx_info)(struct rpmsg_remotedev *rdev,
			   struct rpmsg_rdev_eth_switch_rx_info *info);
	int (*get_mac)(struct rpmsg_remotedev *rdev, void *mac_addr);
	int (*register_mac)(struct rpmsg_remotedev *rdev,
			    void *mac_addr, u32 flow_idx_offset);
	int (*unregister_mac)(struct rpmsg_remotedev *rdev,
			      void *mac_addr, u32 flow_idx_offset);
	int (*register_ipv4)(struct rpmsg_remotedev *rdev,
			     void *mac_addr, __be32 ipv4);
	int (*unregister_ipv4)(struct rpmsg_remotedev *rdev, __be32 ipv4);
	int (*ping)(struct rpmsg_remotedev *rdev, const u8 *data, int size);
	int (*read_reg)(struct rpmsg_remotedev *rdev, u32 reg_addr, u32 *val);
	int (*dbg_dump_stats)(struct rpmsg_remotedev *rdev);
	int (*set_promisc_mode)(struct rpmsg_remotedev *rdev, u32 enable);
	int (*filter_add_mc)(struct rpmsg_remotedev *rdev,
			     const void *mac_addr, u16 vlan_id, u32 flow_idx);
	int (*filter_del_mc)(struct rpmsg_remotedev *rdev,
			     const void *mac_addr, u16 vlan_id, u32 flow_idx);
};

enum rpmsg_remotedev_type {
	RPMSG_REMOTEDEV_DEMO_DEVICE,
	RPMSG_REMOTEDEV_DISPLAY_DEVICE,
	RPMSG_REMOTEDEV_ETH_SWITCH_DEVICE,
};

struct rpmsg_remotedev {
	enum rpmsg_remotedev_type type;
	union {
		struct {
			struct rpmsg_remotedev_demo_ops *ops;
			struct rpmsg_remotedev_demo_cb *cb_ops;
		} demo;
		struct {
			struct rpmsg_remotedev_display_ops *ops;
			struct rpmsg_remotedev_display_cb *cb_ops;
		} display;

		struct {
			struct rpmsg_remotedev_eth_switch_ops *ops;
		} eth_switch;
	} device;
	void *cb_data;
};

#if IS_REACHABLE(CONFIG_RPMSG_KDRV)
extern struct rpmsg_remotedev *rpmsg_remotedev_get_named_device(const char *device_name);
extern void rpmsg_remotedev_put_device(struct rpmsg_remotedev *rdev);
#else
static inline struct rpmsg_remotedev * __maybe_unused rpmsg_remotedev_get_named_device(const char *device_name)
{
	return NULL;
}

static inline void __maybe_unused rpmsg_remotedev_put_device(struct rpmsg_remotedev *rdev)
{
}
#endif

#endif
