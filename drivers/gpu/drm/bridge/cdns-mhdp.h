/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cadence MHDP DP MST bridge driver.
 *
 * Copyright: 2018 Cadence Design Systems, Inc.
 *
 * Author: Quentin Schulz <quentin.schulz@free-electrons.com>
 */


#ifndef CDNS_MHDP_H
#define CDNS_MHDP_H

#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_dp_helper.h>

#define CDNS_APB_CFG				0x00000
#define CDNS_APB_CTRL				(CDNS_APB_CFG + 0x00)
#define CDNS_CPU_STALL				BIT(3)

#define CDNS_MAILBOX_FULL			(CDNS_APB_CFG + 0x08)
#define CDNS_MAILBOX_EMPTY			(CDNS_APB_CFG + 0x0c)
#define CDNS_MAILBOX_TX_DATA			(CDNS_APB_CFG + 0x10)
#define CDNS_MAILBOX_RX_DATA			(CDNS_APB_CFG + 0x14)
#define CDNS_KEEP_ALIVE				(CDNS_APB_CFG + 0x18)
#define CDNS_KEEP_ALIVE_MASK			GENMASK(7, 0)

#define CDNS_MB_INT_MASK			(CDNS_APB_CFG + 0x34)
#define CDNS_MB_INT_STATUS			(CDNS_APB_CFG + 0x38)

#define CDNS_SW_CLK_L				(CDNS_APB_CFG + 0x3c)
#define CDNS_SW_CLK_H				(CDNS_APB_CFG + 0x40)
#define CDNS_SW_EVENT0				(CDNS_APB_CFG + 0x44)
#define CDNS_DPTX_HPD				BIT(0)

#define CDNS_SW_EVENT1				(CDNS_APB_CFG + 0x48)
#define CDNS_SW_EVENT2				(CDNS_APB_CFG + 0x4c)
#define CDNS_SW_EVENT3				(CDNS_APB_CFG + 0x50)

#define CDNS_APB_INT_MASK			(CDNS_APB_CFG + 0x6C)
#define CDNS_APB_INT_MASK_MAILBOX_INT		BIT(0)
#define CDNS_APB_INT_MASK_SW_EVENT_INT		BIT(1)
#define CDNS_APB_INT_STATUS			(CDNS_APB_CFG + 0x70)

#define CDNS_DPTX_CAR				(CDNS_APB_CFG + 0x904)
#define CDNS_VIF_CLK_EN				BIT(0)
#define CDNS_VIF_CLK_RSTN			BIT(1)

#define CDNS_SOURCE_VIDEO_IF(s)			(0x00b00 + (s * 0x20))
#define CDNS_BND_HSYNC2VSYNC(s)			(CDNS_SOURCE_VIDEO_IF(s) + \
						 0x00)
#define CDNS_IP_DTCT_WIN			GENMASK(11, 0)
#define CDNS_IP_DET_INTERLACE_FORMAT		BIT(12)
#define CDNS_IP_BYPASS_V_INTERFACE		BIT(13)

#define CDNS_HSYNC2VSYNC_POL_CTRL(s)		(CDNS_SOURCE_VIDEO_IF(s) + \
						 0x10)
#define CDNS_H2V_HSYNC_POL_ACTIVE_LOW		BIT(1)
#define CDNS_H2V_VSYNC_POL_ACTIVE_LOW		BIT(2)

#define CDNS_DPTX_PHY_CONFIG			0x02000
#define CDNS_PHY_TRAINING_EN			BIT(0)
#define CDNS_PHY_TRAINING_TYPE(x)		(((x) & GENMASK(3, 0)) << 1)
#define CDNS_PHY_SCRAMBLER_BYPASS		BIT(5)
#define CDNS_PHY_ENCODER_BYPASS			BIT(6)
#define CDNS_PHY_SKEW_BYPASS			BIT(7)
#define CDNS_PHY_TRAINING_AUTO			BIT(8)
#define CDNS_PHY_LANE0_SKEW(x)			(((x) & GENMASK(2, 0)) << 9)
#define CDNS_PHY_LANE1_SKEW(x)			(((x) & GENMASK(2, 0)) << 12)
#define CDNS_PHY_LANE2_SKEW(x)			(((x) & GENMASK(2, 0)) << 15)
#define CDNS_PHY_LANE3_SKEW(x)			(((x) & GENMASK(2, 0)) << 18)
#define CDNS_PHY_COMMON_CONFIG			(CDNS_PHY_LANE1_SKEW(1) | \
						CDNS_PHY_LANE2_SKEW(2) |  \
						CDNS_PHY_LANE3_SKEW(3))
#define CDNS_PHY_10BIT_EN			BIT(21)

#define CDNS_DPTX_FRAMER			0x02200
#define CDNS_DP_FRAMER_GLOBAL_CONFIG		(CDNS_DPTX_FRAMER + 0x00)
#define CDNS_DP_NUM_LANES(x)			(x - 1)
#define CDNS_DP_MST_EN				BIT(2)
#define CDNS_DP_FRAMER_EN			BIT(3)
#define CDNS_DP_RATE_GOVERNOR_EN		BIT(4)
#define CDNS_DP_NO_VIDEO_MODE			BIT(5)
#define CDNS_DP_DISABLE_PHY_RST			BIT(6)
#define CDNS_DP_WR_FAILING_EDGE_VSYNC		BIT(7)

#define CDNS_DP_FRAMER_TU			(CDNS_DPTX_FRAMER + 0x08)
#define CDNS_DP_FRAMER_TU_SIZE(x)		(((x) & GENMASK(6, 0)) << 8)
#define CDNS_DP_FRAMER_TU_VS(x)			((x) & GENMASK(5, 0))
#define CDNS_DP_FRAMER_TU_CNT_RST_EN		BIT(15)

#define CDNS_DPTX_STREAM(s)			(0x03000 + s * 0x80)
#define CDNS_DP_MSA_HORIZONTAL_0(s)		(CDNS_DPTX_STREAM(s) + 0x00)
#define CDNS_DP_MSAH0_H_TOTAL(x)		(x)
#define CDNS_DP_MSAH0_HSYNC_START(x)		((x) << 16)

#define CDNS_DP_MSA_HORIZONTAL_1(s)		(CDNS_DPTX_STREAM(s) + 0x04)
#define CDNS_DP_MSAH1_HSYNC_WIDTH(x)		(x)
#define CDNS_DP_MSAH1_HSYNC_POL_LOW		BIT(15)
#define CDNS_DP_MSAH1_HDISP_WIDTH(x)		((x) << 16)

#define CDNS_DP_MSA_VERTICAL_0(s)		(CDNS_DPTX_STREAM(s) + 0x08)
#define CDNS_DP_MSAV0_V_TOTAL(x)		(x)
#define CDNS_DP_MSAV0_VSYNC_START(x)		((x) << 16)

#define CDNS_DP_MSA_VERTICAL_1(s)		(CDNS_DPTX_STREAM(s) + 0x0c)
#define CDNS_DP_MSAV1_VSYNC_WIDTH(x)		(x)
#define CDNS_DP_MSAV1_VSYNC_POL_LOW		BIT(15)
#define CDNS_DP_MSAV1_VDISP_WIDTH(x)		((x) << 16)

#define CDNS_DP_MSA_MISC(s)			(CDNS_DPTX_STREAM(s) + 0x10)
#define CDNS_DP_STREAM_CONFIGs(s)		(CDNS_DPTX_STREAM(s) + 0x14)
#define CDNS_DP_STREAM_CONFIG_2(s)		(CDNS_DPTX_STREAM(s) + 0x2c)
#define CDNS_DP_SC2_TU_VS_DIFF(x)		((x) << 8)

#define CDNS_DP_HORIZONTAL(s)			(CDNS_DPTX_STREAM(s) + 0x30)
#define CDNS_DP_H_HSYNC_WIDTH(x)		(x)
#define CDNS_DP_H_H_TOTAL(x)			((x) << 16)

#define CDNS_DP_VERTICAL_0(s)			(CDNS_DPTX_STREAM(s) + 0x34)
#define CDNS_DP_V0_VHEIGHT(x)			(x)
#define CDNS_DP_V0_VSTART(x)			((x) << 16)

#define CDNS_DP_VERTICAL_1(s)			(CDNS_DPTX_STREAM(s) + 0x38)
#define CDNS_DP_V1_VTOTAL(x)			(x)
#define CDNS_DP_V1_VTOTAL_EVEN			BIT(16)

#define CDNS_DP_FRAMER_PXL_REPR(s)		(CDNS_DPTX_STREAM(s) + 0x4c)
#define CDNS_DP_FRAMER_6_BPC			BIT(0)
#define CDNS_DP_FRAMER_8_BPC			BIT(1)
#define CDNS_DP_FRAMER_10_BPC			BIT(2)
#define CDNS_DP_FRAMER_12_BPC			BIT(3)
#define CDNS_DP_FRAMER_16_BPC			BIT(4)
#define CDNS_DP_FRAMER_PXL_FORMAT		0x8
#define CDNS_DP_FRAMER_RGB			BIT(0)
#define CDNS_DP_FRAMER_YCBCR444			BIT(1)
#define CDNS_DP_FRAMER_YCBCR422			BIT(2)
#define CDNS_DP_FRAMER_YCBCR420			BIT(3)
#define CDNS_DP_FRAMER_Y_ONLY			BIT(4)

#define CDNS_DP_FRAMER_SP(s)			(CDNS_DPTX_STREAM(s) + 0x50)
#define CDNS_DP_FRAMER_VSYNC_POL_LOW		BIT(0)
#define CDNS_DP_FRAMER_HSYNC_POL_LOW		BIT(1)
#define CDNS_DP_FRAMER_INTERLACE		BIT(2)

#define CDNS_DP_LINE_THRESH(s)			(CDNS_DPTX_STREAM(s) + 0x64)
#define CDNS_DP_ACTIVE_LINE_THRESH(x)		(x)

#define CDNS_DP_VB_ID(s)			(CDNS_DPTX_STREAM(s) + 0x68)
#define CDNS_DP_VB_ID_INTERLACED		BIT(2)
#define CDNS_DP_VB_ID_COMPRESSED		BIT(6)

#define CDNS_DP_FRONT_BACK_PORCH(s)		(CDNS_DPTX_STREAM(s) + 0x78)
#define CDNS_DP_BACK_PORCH(x)			(x)
#define CDNS_DP_FRONT_PORCH(x)			((x) << 16)

#define CDNS_DP_BYTE_COUNT(s)			(CDNS_DPTX_STREAM(s) + 0x7c)
#define CDNS_DP_BYTE_COUNT_BYTES_IN_CHUNK_SHIFT	16

#define CDNS_DP_MST_STREAM_CONFIG(s)		(CDNS_DPTX_STREAM(s) + 0x14)
#define CDNS_DP_MST_STRM_CFG_STREAM_EN		BIT(0)
#define CDNS_DP_MST_STRM_CFG_NO_VIDEO		BIT(1)

#define CDNS_DP_MST_SLOT_ALLOCATE(s)		(CDNS_DPTX_STREAM(s) + 0x44)
#define CDNS_DP_S_ALLOC_START_SLOT(x)		(x)
#define CDNS_DP_S_ALLOC_END_SLOT(x)		((x) << 8)

#define CDNS_DP_RATE_GOVERNING(s)		(CDNS_DPTX_STREAM(s) + 0x48)
#define CDNS_DP_RG_TARG_AV_SLOTS_Y(x)		(x)
#define CDNS_DP_RG_TARG_AV_SLOTS_X(x)		(x << 4)
#define CDNS_DP_RG_ENABLE			BIT(10)

#define CDNS_DP_MTPH_CONTROL			0x2264
#define CDNS_DP_MTPH_ECF_EN			BIT(0)
#define CDNS_DP_MTPH_ACT_EN			BIT(1)
#define CDNS_DP_MTPH_LVP_EN			BIT(2)

#define CDNS_DP_MTPH_STATUS			0x226C
#define CDNS_DP_MTPH_ACT_STATUS			BIT(0)


#define CDNS_DPTX_GLOBAL			0x02300
#define CDNS_DP_LANE_EN				(CDNS_DPTX_GLOBAL + 0x00)
#define CDNS_DP_LANE_EN_LANES(x)		GENMASK(x - 1, 0)
#define CDNS_DP_ENHNCD				(CDNS_DPTX_GLOBAL + 0x04)

struct cdns_mhdp_host {
	unsigned int link_rate;
	u8 lanes_cnt;
	u8 volt_swing;
	u8 pre_emphasis;
	u8 pattern_supp;
	u8 lane_mapping;
	u8 fast_link : 1;
	u8 enhanced : 1;
	u8 scrambler : 1;
	u8 ssc : 1;
};

struct cdns_mhdp_sink {
	unsigned int link_rate;
	u8 lanes_cnt;
	u8 pattern_supp;
	u8 fast_link : 1;
	u8 enhanced : 1;
	u8 ssc : 1;
};

struct cdns_mhdp_display_fmt {
	u32 color_format;
	u32 bpc;
	u8 y_only : 1;
};

/*
 * These enums present MHDP hw initialization state
 * Legal state transitions are:
 * MHDP_HW_INACTIVE <-> MHDP_HW_LOADING -> MHDP_HW_READY
 *        |                                     |
 *        '----------> MHDP_HW_STOPPED <--------'
 */
enum mhdp_hw_state { MHDP_HW_INACTIVE = 0, /* HW not initialized */
		     MHDP_HW_LOADING,	   /* HW initialization in progress */
		     MHDP_HW_READY,	   /* HW ready, FW active*/
		     MHDP_HW_STOPPED };	   /* Driver removal FW to be stopped */

struct cdns_mhdp_device;

struct mhdp_platform_ops {
	int (*init)(struct cdns_mhdp_device *mhdp);
	void (*exit)(struct cdns_mhdp_device *mhdp);
	void (*enable)(struct cdns_mhdp_device *mhdp);
	void (*disable)(struct cdns_mhdp_device *mhdp);
};

struct cdns_mhdp_device {
	void __iomem *regs;
	void __iomem *j721e_regs;

	struct device *dev;
	struct clk *clk;
	struct phy *phy;

	const struct mhdp_platform_ops *ops;

	/* This is to protect mailbox communications with the firmware */
	struct mutex mbox_mutex;

	struct drm_connector connector;
	struct drm_bridge bridge;

	struct drm_dp_link link;
	struct drm_dp_aux aux;

	struct cdns_mhdp_host host;
	struct cdns_mhdp_sink sink;
	struct cdns_mhdp_display_fmt display_fmt;
	s8 stream_id;

	u8 link_up : 1;
	u8 plugged : 1;

	/*
	 * "start_lock" protects the access to bridge_attached and
	 * hw_state data members that control the delayed firmware
	 * loading and attaching the bridge. They are accessed from
	 * both the DRM core and mhdp_fw_cb(). In most cases just
	 * protecting the data members is enough, but the irq mask
	 * setting needs to be protected when enabling the FW.
	 */
	spinlock_t start_lock;
	u8 bridge_attached : 1;
	enum mhdp_hw_state hw_state;
};

#define connector_to_mhdp(x) container_of(x, struct cdns_mhdp_device, connector)
#define bridge_to_mhdp(x) container_of(x, struct cdns_mhdp_device, bridge)

#define CDNS_MHDP_MAX_STREAMS   4

u32 cdns_mhdp_get_bpp(struct cdns_mhdp_display_fmt *fmt);
void cdns_mhdp_configure_video(struct drm_bridge *bridge);
void cdns_mhdp_enable(struct drm_bridge *bridge);

#endif
