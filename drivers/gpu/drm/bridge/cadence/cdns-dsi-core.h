/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright: 2017 Cadence Design Systems, Inc.
 *
 * Author: Boris Brezillon <boris.brezillon@bootlin.com>
 */

#ifndef CDNS_DSI_H
#define CDNS_DSI_H

#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>

#define IP_CONF				0x0
#define SP_HS_FIFO_DEPTH(x)		(((x) & GENMASK(30, 26)) >> 26)
#define SP_LP_FIFO_DEPTH(x)		(((x) & GENMASK(25, 21)) >> 21)
#define VRS_FIFO_DEPTH(x)		(((x) & GENMASK(20, 16)) >> 16)
#define DIRCMD_FIFO_DEPTH(x)		(((x) & GENMASK(15, 13)) >> 13)
#define SDI_IFACE_32			BIT(12)
#define INTERNAL_DATAPATH_32		(0 << 10)
#define INTERNAL_DATAPATH_16		(1 << 10)
#define INTERNAL_DATAPATH_8		(3 << 10)
#define INTERNAL_DATAPATH_SIZE		((x) & GENMASK(11, 10))
#define NUM_IFACE(x)			((((x) & GENMASK(9, 8)) >> 8) + 1)
#define MAX_LANE_NB(x)			(((x) & GENMASK(7, 6)) >> 6)
#define RX_FIFO_DEPTH(x)		((x) & GENMASK(5, 0))

#define MCTL_MAIN_DATA_CTL		0x4
#define TE_MIPI_POLLING_EN		BIT(25)
#define TE_HW_POLLING_EN		BIT(24)
#define DISP_EOT_GEN			BIT(18)
#define HOST_EOT_GEN			BIT(17)
#define DISP_GEN_CHECKSUM		BIT(16)
#define DISP_GEN_ECC			BIT(15)
#define BTA_EN				BIT(14)
#define READ_EN				BIT(13)
#define REG_TE_EN			BIT(12)
#define IF_TE_EN(x)			BIT(8 + (x))
#define TVG_SEL				BIT(6)
#define VID_EN				BIT(5)
#define IF_VID_SELECT(x)		((x) << 2)
#define IF_VID_SELECT_MASK		GENMASK(3, 2)
#define IF_VID_MODE			BIT(1)
#define LINK_EN				BIT(0)

#define MCTL_MAIN_PHY_CTL		0x8
#define HS_INVERT_DAT(x)		BIT(19 + ((x) * 2))
#define SWAP_PINS_DAT(x)		BIT(18 + ((x) * 2))
#define HS_INVERT_CLK			BIT(17)
#define SWAP_PINS_CLK			BIT(16)
#define HS_SKEWCAL_EN			BIT(15)
#define WAIT_BURST_TIME(x)		((x) << 10)
#define DATA_ULPM_EN(x)			BIT(6 + (x))
#define CLK_ULPM_EN			BIT(5)
#define CLK_CONTINUOUS			BIT(4)
#define DATA_LANE_EN(x)			BIT((x) - 1)

#define MCTL_MAIN_EN			0xc
#define DATA_FORCE_STOP			BIT(17)
#define CLK_FORCE_STOP			BIT(16)
#define IF_EN(x)			BIT(13 + (x))
#define DATA_LANE_ULPM_REQ(l)		BIT(9 + (l))
#define CLK_LANE_ULPM_REQ		BIT(8)
#define DATA_LANE_START(x)		BIT(4 + (x))
#define CLK_LANE_EN			BIT(3)
#define PLL_START			BIT(0)

#define MCTL_DPHY_CFG0			0x10
#define DPHY_C_RSTB			BIT(20)
#define DPHY_D_RSTB(x)			GENMASK(15 + (x), 16)
#define DPHY_PLL_PDN			BIT(10)
#define DPHY_CMN_PDN			BIT(9)
#define DPHY_C_PDN			BIT(8)
#define DPHY_D_PDN(x)			GENMASK(3 + (x), 4)
#define DPHY_ALL_D_PDN			GENMASK(7, 4)
#define DPHY_PLL_PSO			BIT(1)
#define DPHY_CMN_PSO			BIT(0)

#define MCTL_DPHY_TIMEOUT1		0x14
#define HSTX_TIMEOUT(x)			((x) << 4)
#define HSTX_TIMEOUT_MAX		GENMASK(17, 0)
#define CLK_DIV(x)			(x)
#define CLK_DIV_MAX			GENMASK(3, 0)

#define MCTL_DPHY_TIMEOUT2		0x18
#define LPRX_TIMEOUT(x)			(x)

#define MCTL_ULPOUT_TIME		0x1c
#define DATA_LANE_ULPOUT_TIME(x)	((x) << 9)
#define CLK_LANE_ULPOUT_TIME(x)		(x)

#define MCTL_3DVIDEO_CTL		0x20
#define VID_VSYNC_3D_EN			BIT(7)
#define VID_VSYNC_3D_LR			BIT(5)
#define VID_VSYNC_3D_SECOND_EN		BIT(4)
#define VID_VSYNC_3DFORMAT_LINE		(0 << 2)
#define VID_VSYNC_3DFORMAT_FRAME	(1 << 2)
#define VID_VSYNC_3DFORMAT_PIXEL	(2 << 2)
#define VID_VSYNC_3DMODE_OFF		0
#define VID_VSYNC_3DMODE_PORTRAIT	1
#define VID_VSYNC_3DMODE_LANDSCAPE	2

#define MCTL_MAIN_STS			0x24
#define MCTL_MAIN_STS_CTL		0x130
#define MCTL_MAIN_STS_CLR		0x150
#define MCTL_MAIN_STS_FLAG		0x170
#define HS_SKEWCAL_DONE			BIT(11)
#define IF_UNTERM_PKT_ERR(x)		BIT(8 + (x))
#define LPRX_TIMEOUT_ERR		BIT(7)
#define HSTX_TIMEOUT_ERR		BIT(6)
#define DATA_LANE_RDY(l)		BIT(2 + (l))
#define CLK_LANE_RDY			BIT(1)
#define PLL_LOCKED			BIT(0)

#define MCTL_DPHY_ERR			0x28
#define MCTL_DPHY_ERR_CTL1		0x148
#define MCTL_DPHY_ERR_CLR		0x168
#define MCTL_DPHY_ERR_FLAG		0x188
#define ERR_CONT_LP(x, l)		BIT(18 + ((x) * 4) + (l))
#define ERR_CONTROL(l)			BIT(14 + (l))
#define ERR_SYNESC(l)			BIT(10 + (l))
#define ERR_ESC(l)			BIT(6 + (l))

#define MCTL_DPHY_ERR_CTL2		0x14c
#define ERR_CONT_LP_EDGE(x, l)		BIT(12 + ((x) * 4) + (l))
#define ERR_CONTROL_EDGE(l)		BIT(8 + (l))
#define ERR_SYN_ESC_EDGE(l)		BIT(4 + (l))
#define ERR_ESC_EDGE(l)			BIT(0 + (l))

#define MCTL_LANE_STS			0x2c
#define PPI_C_TX_READY_HS		BIT(18)
#define DPHY_PLL_LOCK			BIT(17)
#define PPI_D_RX_ULPS_ESC(x)		(((x) & GENMASK(15, 12)) >> 12)
#define LANE_STATE_START		0
#define LANE_STATE_IDLE			1
#define LANE_STATE_WRITE		2
#define LANE_STATE_ULPM			3
#define LANE_STATE_READ			4
#define DATA_LANE_STATE(l, val)		\
	(((val) >> (2 + 2 * (l) + ((l) ? 1 : 0))) & GENMASK((l) ? 1 : 2, 0))
#define CLK_LANE_STATE_HS		2
#define CLK_LANE_STATE(val)		((val) & GENMASK(1, 0))

#define DSC_MODE_CTL			0x30
#define DSC_MODE_EN			BIT(0)

#define DSC_CMD_SEND			0x34
#define DSC_SEND_PPS			BIT(0)
#define DSC_EXECUTE_QUEUE		BIT(1)

#define DSC_PPS_WRDAT			0x38

#define DSC_MODE_STS			0x3c
#define DSC_PPS_DONE			BIT(1)
#define DSC_EXEC_DONE			BIT(2)

#define CMD_MODE_CTL			0x70
#define IF_LP_EN(x)			BIT(9 + (x))
#define IF_VCHAN_ID(x, c)		((c) << ((x) * 2))

#define CMD_MODE_CTL2			0x74
#define TE_TIMEOUT(x)			((x) << 11)
#define FILL_VALUE(x)			((x) << 3)
#define ARB_IF_WITH_HIGHEST_PRIORITY(x)	((x) << 1)
#define ARB_ROUND_ROBIN_MODE		BIT(0)

#define CMD_MODE_STS			0x78
#define CMD_MODE_STS_CTL		0x134
#define CMD_MODE_STS_CLR		0x154
#define CMD_MODE_STS_FLAG		0x174
#define ERR_IF_UNDERRUN(x)		BIT(4 + (x))
#define ERR_UNWANTED_READ		BIT(3)
#define ERR_TE_MISS			BIT(2)
#define ERR_NO_TE			BIT(1)
#define CSM_RUNNING			BIT(0)

#define DIRECT_CMD_SEND			0x80

#define DIRECT_CMD_MAIN_SETTINGS	0x84
#define TRIGGER_VAL(x)			((x) << 25)
#define CMD_LP_EN			BIT(24)
#define CMD_SIZE(x)			((x) << 16)
#define CMD_VCHAN_ID(x)			((x) << 14)
#define CMD_DATATYPE(x)			((x) << 8)
#define CMD_LONG			BIT(3)
#define WRITE_CMD			0
#define READ_CMD			1
#define TE_REQ				4
#define TRIGGER_REQ			5
#define BTA_REQ				6

#define DIRECT_CMD_STS			0x88
#define DIRECT_CMD_STS_CTL		0x138
#define DIRECT_CMD_STS_CLR		0x158
#define DIRECT_CMD_STS_FLAG		0x178
#define RCVD_ACK_VAL(val)		((val) >> 16)
#define RCVD_TRIGGER_VAL(val)		(((val) & GENMASK(14, 11)) >> 11)
#define READ_COMPLETED_WITH_ERR		BIT(10)
#define BTA_FINISHED			BIT(9)
#define BTA_COMPLETED			BIT(8)
#define TE_RCVD				BIT(7)
#define TRIGGER_RCVD			BIT(6)
#define ACK_WITH_ERR_RCVD		BIT(5)
#define ACK_RCVD			BIT(4)
#define READ_COMPLETED			BIT(3)
#define TRIGGER_COMPLETED		BIT(2)
#define WRITE_COMPLETED			BIT(1)
#define SENDING_CMD			BIT(0)

#define DIRECT_CMD_STOP_READ		0x8c

#define DIRECT_CMD_WRDATA		0x90

#define DIRECT_CMD_FIFO_RST		0x94

#define DIRECT_CMD_RDDATA		0xa0

#define DIRECT_CMD_RD_PROPS		0xa4
#define RD_DCS				BIT(18)
#define RD_VCHAN_ID(val)		(((val) >> 16) & GENMASK(1, 0))
#define RD_SIZE(val)			((val) & GENMASK(15, 0))

#define DIRECT_CMD_RD_STS		0xa8
#define DIRECT_CMD_RD_STS_CTL		0x13c
#define DIRECT_CMD_RD_STS_CLR		0x15c
#define DIRECT_CMD_RD_STS_FLAG		0x17c
#define ERR_EOT_WITH_ERR		BIT(8)
#define ERR_MISSING_EOT			BIT(7)
#define ERR_WRONG_LENGTH		BIT(6)
#define ERR_OVERSIZE			BIT(5)
#define ERR_RECEIVE			BIT(4)
#define ERR_UNDECODABLE			BIT(3)
#define ERR_CHECKSUM			BIT(2)
#define ERR_UNCORRECTABLE		BIT(1)
#define ERR_FIXED			BIT(0)

#define VID_MAIN_CTL			0xb0
#define VID_IGNORE_MISS_VSYNC		BIT(31)
#define VID_FIELD_SW			BIT(28)
#define VID_INTERLACED_EN		BIT(27)
#define RECOVERY_MODE(x)		((x) << 25)
#define RECOVERY_MODE_NEXT_HSYNC	0
#define RECOVERY_MODE_NEXT_STOP_POINT	2
#define RECOVERY_MODE_NEXT_VSYNC	3
#define REG_BLKEOL_MODE(x)		((x) << 23)
#define REG_BLKLINE_MODE(x)		((x) << 21)
#define REG_BLK_MODE_NULL_PKT		0
#define REG_BLK_MODE_BLANKING_PKT	1
#define REG_BLK_MODE_LP			2
#define SYNC_PULSE_HORIZONTAL		BIT(20)
#define SYNC_PULSE_ACTIVE		BIT(19)
#define BURST_MODE			BIT(18)
#define VID_PIXEL_MODE_MASK		GENMASK(17, 14)
#define VID_PIXEL_MODE_RGB565		(0 << 14)
#define VID_PIXEL_MODE_RGB666_PACKED	(1 << 14)
#define VID_PIXEL_MODE_RGB666		(2 << 14)
#define VID_PIXEL_MODE_RGB888		(3 << 14)
#define VID_PIXEL_MODE_RGB101010	(4 << 14)
#define VID_PIXEL_MODE_RGB121212	(5 << 14)
#define VID_PIXEL_MODE_YUV420		(8 << 14)
#define VID_PIXEL_MODE_YUV422_PACKED	(9 << 14)
#define VID_PIXEL_MODE_YUV422		(10 << 14)
#define VID_PIXEL_MODE_YUV422_24B	(11 << 14)
#define VID_PIXEL_MODE_DSC_COMP		(12 << 14)
#define VID_DATATYPE(x)			((x) << 8)
#define VID_VIRTCHAN_ID(iface, x)	((x) << (4 + (iface) * 2))
#define STOP_MODE(x)			((x) << 2)
#define START_MODE(x)			(x)

#define VID_VSIZE1			0xb4
#define VFP_LEN(x)			((x) << 12)
#define VBP_LEN(x)			((x) << 6)
#define VSA_LEN(x)			(x)

#define VID_VSIZE2			0xb8
#define VACT_LEN(x)			(x)

#define VID_HSIZE1			0xc0
#define HBP_LEN(x)			((x) << 16)
#define HSA_LEN(x)			(x)

#define VID_HSIZE2			0xc4
#define HFP_LEN(x)			((x) << 16)
#define HACT_LEN(x)			(x)

#define VID_BLKSIZE1			0xcc
#define BLK_EOL_PKT_LEN(x)		((x) << 15)
#define BLK_LINE_EVENT_PKT_LEN(x)	(x)

#define VID_BLKSIZE2			0xd0
#define BLK_LINE_PULSE_PKT_LEN(x)	(x)

#define VID_PKT_TIME			0xd8
#define BLK_EOL_DURATION(x)		(x)

#define VID_DPHY_TIME			0xdc
#define REG_WAKEUP_TIME(x)		((x) << 17)
#define REG_LINE_DURATION(x)		(x)

#define VID_ERR_COLOR1			0xe0
#define COL_GREEN(x)			((x) << 12)
#define COL_RED(x)			(x)

#define VID_ERR_COLOR2			0xe4
#define PAD_VAL(x)			((x) << 12)
#define COL_BLUE(x)			(x)

#define VID_VPOS			0xe8
#define LINE_VAL(val)			(((val) & GENMASK(14, 2)) >> 2)
#define LINE_POS(val)			((val) & GENMASK(1, 0))

#define VID_HPOS			0xec
#define HORIZ_VAL(val)			(((val) & GENMASK(17, 3)) >> 3)
#define HORIZ_POS(val)			((val) & GENMASK(2, 0))

#define VID_MODE_STS			0xf0
#define VID_MODE_STS_CTL		0x140
#define VID_MODE_STS_CLR		0x160
#define VID_MODE_STS_FLAG		0x180
#define VSG_RECOVERY			BIT(10)
#define ERR_VRS_WRONG_LEN		BIT(9)
#define ERR_LONG_READ			BIT(8)
#define ERR_LINE_WRITE			BIT(7)
#define ERR_BURST_WRITE			BIT(6)
#define ERR_SMALL_HEIGHT		BIT(5)
#define ERR_SMALL_LEN			BIT(4)
#define ERR_MISSING_VSYNC		BIT(3)
#define ERR_MISSING_HSYNC		BIT(2)
#define ERR_MISSING_DATA		BIT(1)
#define VSG_RUNNING			BIT(0)

#define VID_VCA_SETTING1		0xf4
#define BURST_LP			BIT(16)
#define MAX_BURST_LIMIT(x)		(x)

#define VID_VCA_SETTING2		0xf8
#define MAX_LINE_LIMIT(x)		((x) << 16)
#define EXACT_BURST_LIMIT(x)		(x)

#define TVG_CTL				0xfc
#define TVG_STRIPE_SIZE(x)		((x) << 5)
#define TVG_MODE_MASK			GENMASK(4, 3)
#define TVG_MODE_SINGLE_COLOR		(0 << 3)
#define TVG_MODE_VSTRIPES		(2 << 3)
#define TVG_MODE_HSTRIPES		(3 << 3)
#define TVG_STOPMODE_MASK		GENMASK(2, 1)
#define TVG_STOPMODE_EOF		(0 << 1)
#define TVG_STOPMODE_EOL		(1 << 1)
#define TVG_STOPMODE_NOW		(2 << 1)
#define TVG_RUN				BIT(0)

#define TVG_IMG_SIZE			0x100
#define TVG_NBLINES(x)			((x) << 16)
#define TVG_LINE_SIZE(x)		(x)

#define TVG_COLOR1			0x104
#define TVG_COL1_GREEN(x)		((x) << 12)
#define TVG_COL1_RED(x)			(x)

#define TVG_COLOR1_BIS			0x108
#define TVG_COL1_BLUE(x)		(x)

#define TVG_COLOR2			0x10c
#define TVG_COL2_GREEN(x)		((x) << 12)
#define TVG_COL2_RED(x)			(x)

#define TVG_COLOR2_BIS			0x110
#define TVG_COL2_BLUE(x)		(x)

#define TVG_STS				0x114
#define TVG_STS_CTL			0x144
#define TVG_STS_CLR			0x164
#define TVG_STS_FLAG			0x184
#define TVG_STS_RUNNING			BIT(0)

#define STS_CTL_EDGE(e)			((e) << 16)

#define DPHY_LANES_MAP			0x198
#define DAT_REMAP_CFG(b, l)		((l) << ((b) * 8))

#define DPI_IRQ_EN			0x1a0
#define DPI_IRQ_CLR			0x1a4
#define DPI_IRQ_STS			0x1a8
#define PIXEL_BUF_OVERFLOW		BIT(0)

#define DPI_CFG				0x1ac
#define DPI_CFG_FIFO_DEPTH(x)		((x) >> 16)
#define DPI_CFG_FIFO_LEVEL(x)		((x) & GENMASK(15, 0))

#define TEST_GENERIC			0x1f0
#define TEST_STATUS(x)			((x) >> 16)
#define TEST_CTRL(x)			(x)

#define ID_REG				0x1fc
#define REV_VENDOR_ID(x)		(((x) & GENMASK(31, 20)) >> 20)
#define REV_PRODUCT_ID(x)		(((x) & GENMASK(19, 12)) >> 12)
#define REV_HW(x)			(((x) & GENMASK(11, 8)) >> 8)
#define REV_MAJOR(x)			(((x) & GENMASK(7, 4)) >> 4)
#define REV_MINOR(x)			((x) & GENMASK(3, 0))

#define DSI_OUTPUT_PORT			0
#define DSI_INPUT_PORT(inputid)		(1 + (inputid))

#define DSI_HBP_FRAME_OVERHEAD		12
#define DSI_HSA_FRAME_OVERHEAD		14
#define DSI_HFP_FRAME_OVERHEAD		6
#define DSI_HSS_VSS_VSE_FRAME_OVERHEAD	4
#define DSI_BLANKING_FRAME_OVERHEAD	6
#define DSI_NULL_FRAME_OVERHEAD		6
#define DSI_EOT_PKT_SIZE		4

struct cdns_dsi_output {
	struct mipi_dsi_device *dev;
	struct drm_panel *panel;
	struct drm_bridge *bridge;
	union phy_configure_opts phy_opts;
};

enum cdns_dsi_input_id {
	CDNS_SDI_INPUT,
	CDNS_DPI_INPUT,
	CDNS_DSC_INPUT,
};

struct cdns_dsi_cfg {
	unsigned int hfp;
	unsigned int hsa;
	unsigned int hbp;
	unsigned int hact;
	unsigned int htotal;
};

struct cdns_dsi_input {
	enum cdns_dsi_input_id id;
	struct drm_bridge bridge;
};

struct cdns_dsi;

struct dsi_platform_ops {
	int (*init)(struct cdns_dsi *dsi);
	void (*exit)(struct cdns_dsi *dsi);
	void (*enable)(struct cdns_dsi *dsi);
	void (*disable)(struct cdns_dsi *dsi);
};

struct cdns_dsi {
	struct mipi_dsi_host base;
	void __iomem *regs;
#ifdef CONFIG_DRM_CDNS_DSI_J721E
	void __iomem *j721e_regs;
#endif
	const struct dsi_platform_ops *platform_ops;
	struct cdns_dsi_input input;
	struct cdns_dsi_output output;
	unsigned int direct_cmd_fifo_depth;
	unsigned int rx_fifo_depth;
	struct completion direct_cmd_comp;
	struct clk *dsi_p_clk;
	struct reset_control *dsi_p_rst;
	struct clk *dsi_sys_clk;
	bool link_initialized;
	bool phy_initialized;
	struct phy *dphy;
};

#endif /* !CDNS_DSI_H */
