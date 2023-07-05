/* SPDX-License-Identifier: GPL-2.0 */
/*
 * pci-j721e - PCIe controller driver for TI's J721E SoCs
 *
 * Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com
 */

#ifndef _PCIE_J721E_H
#define _PCIE_J721E_H

#define ENABLE_REG_SYS_0	0x100
#define STATUS_REG_SYS_0	0x500
#define STATUS_CLR_REG_SYS_0	0x700
#define ENABLE_REG_SYS_1	0x104
#define STATUS_REG_SYS_1	0x504
#define ENABLE_REG_SYS_2	0x108
#define CLEAR_REG_SYS_2         0x308
#define STATUS_REG_SYS_2	0x508
#define STATUS_CLR_REG_SYS_2	0x708
#define LINK_DOWN		BIT(1)
#define J7200_LINK_DOWN		BIT(10)

#define J721E_PCIE_USER_CMD_STATUS	0x4
#define LINK_TRAINING_ENABLE		BIT(0)

#define J721E_PCIE_USER_LINKSTATUS	0x14
#define LINK_STATUS			GENMASK(1, 0)

#define EOI_REG			0x10
#define INTx_EN(num)		(1 << (num))
#define SYS1_INTx_EN(num)	(1 << (22 + (num)))
#define USER_EOI_REG		0xC8

enum eoi_reg {
	EOI_DOWNSTREAM_INTERRUPT,
	EOI_FLR_INTERRUPT,
	EOI_LEGACY_INTERRUPT,
	EOI_POWER_STATE_INTERRUPT,
};

enum link_status {
	NO_RECEIVERS_DETECTED,
	LINK_TRAINING_IN_PROGRESS,
	LINK_UP_DL_IN_PROGRESS,
	LINK_UP_DL_COMPLETED,
};

#define J721E_MODE_RC			BIT(7)
#define LANE_COUNT(n)			((n) << 8)

#define GENERATION_SEL_MASK		GENMASK(1, 0)

struct j721e_pcie {
	struct cdns_pcie	*cdns_pcie;
	struct clk		*refclk;
	u32			mode;
	u32			num_lanes;
	u32			max_lanes;
	void __iomem		*user_cfg_base;
	void __iomem		*intd_cfg_base;
	u32			linkdown_irq_regfield;
	struct gpio_desc        *gpiod;
	struct irq_domain	*legacy_irq_domain;
	unsigned int		is_intc_v1:1;
};

enum j721e_pcie_mode {
	PCI_MODE_RC,
	PCI_MODE_EP,
};

struct j721e_pcie_data {
	enum j721e_pcie_mode	mode;
	unsigned int		is_intc_v1:1;
	unsigned int		quirk_retrain_flag:1;
	unsigned int		quirk_detect_quiet_flag:1;
	unsigned int		quirk_disable_flr:1;
	u32			linkdown_irq_regfield;
	unsigned int		byte_access_allowed:1;
	unsigned int		max_lanes;
};

static inline u32 j721e_pcie_user_readl(struct j721e_pcie *pcie, u32 offset)
{
	return readl(pcie->user_cfg_base + offset);
}

static inline void j721e_pcie_user_writel(struct j721e_pcie *pcie, u32 offset,
					  u32 value)
{
	writel(value, pcie->user_cfg_base + offset);
}

static inline u32 j721e_pcie_intd_readl(struct j721e_pcie *pcie, u32 offset)
{
	return readl(pcie->intd_cfg_base + offset);
}

static inline void j721e_pcie_intd_writel(struct j721e_pcie *pcie, u32 offset,
					  u32 value)
{
	writel(value, pcie->intd_cfg_base + offset);
}

extern const struct cdns_pcie_ops j721e_pcie_ops;
int j721e_pcie_common_init(struct j721e_pcie *pcie);
void j721e_pcie_remove_link_irq(struct j721e_pcie *pcie);
void j721e_disable_common_init(struct device *dev);

#endif /* _PCIE_J721E_H */
