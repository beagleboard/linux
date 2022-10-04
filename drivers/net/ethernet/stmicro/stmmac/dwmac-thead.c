// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>

#include "stmmac_platform.h"

/* clock registers */
#define GMAC_CLK_CFG0	0x00
#define GMAC_CLK_CFG1	0x04
#define GMAC_CLK_CFG2	0x08
#define GMAC_CLK_CFG3	0x0C
#define GMAC_CLK_CFG4	0x10
#define GMAC_CLK_CFG5	0x14
#define GMAC_CLK_CFG6	0x18

/* phy interface */
#define DWMAC_PHYIF_MII_GMII	0
#define DWMAC_PHYIF_RGMII	1
#define DWMAC_PHYIF_RMII	4
/* register bit fields, bit[3]: reserved, bit[2:0]: phy interface */
#define DWMAC_PHYIF_MASK	0x7
#define DWMAC_PHYIF_BIT_WIDTH	4

/* TXCLK direction, 1:input, 0:output */
#define TXCLK_DIR_OUTPUT	0
#define TXCLK_DIR_INPUT		1

#define GMAC_CLK_PLLOUT_250M	250000000
#define GMAC_GMII_RGMII_RATE	125000000
#define GMAC_MII_RATE		25000000
/* clock divider for speed */
#define GMAC_CLKDIV_125M	(GMAC_CLK_PLLOUT_250M / GMAC_GMII_RGMII_RATE)
#define GMAC_CLKDIV_25M		(GMAC_CLK_PLLOUT_250M / GMAC_MII_RATE)

struct thead_dwmac_priv_data {
	int id;
	void __iomem *phy_if_reg;
	void __iomem *txclk_dir_reg;
	void __iomem *gmac_clk_reg;
	phy_interface_t interface;
	struct clk *gmac_pll_clk;
	unsigned int gmac_pll_clk_freq;
};

/* set GMAC PHY interface, 0:MII/GMII, 1:RGMII, 4:RMII */
static void thead_dwmac_set_phy_if(struct platform_device *pdev,
				   void __iomem *phy_if_reg, int interface,
				   int devid)
{
	struct device *dev = &pdev->dev;
	unsigned int phyif = PHY_INTERFACE_MODE_MII;
	volatile uint32_t reg;

	if (phy_if_reg == NULL)
		return;

	switch (interface)
	{
	case PHY_INTERFACE_MODE_MII:
	case PHY_INTERFACE_MODE_GMII:
		phyif = DWMAC_PHYIF_MII_GMII;
		break;
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_TXID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_ID:
		phyif = DWMAC_PHYIF_RGMII;
		break;
	case PHY_INTERFACE_MODE_RMII:
		phyif = DWMAC_PHYIF_RMII;
		break;
	default:
		dev_err(dev, "phy interface %d not supported\n", interface);
		return;
	};

	reg = readl(phy_if_reg);
	reg &= ~(DWMAC_PHYIF_MASK << (DWMAC_PHYIF_BIT_WIDTH * devid));
	reg |= (phyif & DWMAC_PHYIF_MASK) << (DWMAC_PHYIF_BIT_WIDTH * devid);
	writel(reg, phy_if_reg);
}

/*
 * set GMAC TXCLK direction
 *     MII        : TXCLK is input
 *     GMII/RGMII : TXCLK is output
 */
static void thead_dwmac_set_txclk_dir(struct platform_device *pdev,
				void __iomem *txclk_dir_reg, int interface)
{
	struct device *dev = &pdev->dev;
	unsigned int txclk_dir = TXCLK_DIR_INPUT;

	if (txclk_dir_reg == NULL)
		return;

	switch (interface)
	{
	case PHY_INTERFACE_MODE_MII:
	case PHY_INTERFACE_MODE_RMII:
		txclk_dir = TXCLK_DIR_INPUT;
		break;
	case PHY_INTERFACE_MODE_GMII:
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_TXID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_ID:
		txclk_dir = TXCLK_DIR_OUTPUT;
		break;
	default:
		dev_err(dev, "phy interface %d not supported\n", interface);
		return;
	};

	writel(txclk_dir, txclk_dir_reg);
}

static void thead_dwmac_set_clk_source(struct platform_device *pdev,
				void __iomem *gmac_clk_reg, int interface)
{
	struct device *dev = &pdev->dev;
	volatile uint32_t reg;

	if (gmac_clk_reg == NULL)
		return;

	reg = readl(gmac_clk_reg + GMAC_CLK_CFG0);

	/* RX clock source */
	reg |= BIT(7);  /* gmac_rx_clk_sel: extern pin */

	/* TX clock source */
	if (interface == PHY_INTERFACE_MODE_MII) {
		reg |= BIT(1);  /* gmac_tx_clk_sel: extern pin */
		reg &= ~BIT(2); /* gmac_tx_clk_gbit_sel: u_tx_clk_mux */
	} else if (interface == PHY_INTERFACE_MODE_GMII) {
		reg &= ~BIT(5); /* gmac_tx_clk_out_sel: GMAC PLL */
		reg |= BIT(2);  /* gmac_tx_clk_gbit_sel: GMAC PLL */
	} else if (interface == PHY_INTERFACE_MODE_RGMII
		|| interface == PHY_INTERFACE_MODE_RGMII_ID
		|| interface == PHY_INTERFACE_MODE_RGMII_RXID
		|| interface == PHY_INTERFACE_MODE_RGMII_TXID) {
		reg &= ~BIT(5); /* gmac_tx_clk_out_sel: GMAC PLL */
		reg |= BIT(2);  /* gmac_tx_clk_gbit_sel: GMAC PLL */
	} else {
		dev_err(dev, "phy interface %d not supported\n", interface);
		return;
	}

	writel(reg, gmac_clk_reg + GMAC_CLK_CFG0);
}


/* set clock source */
static void thead_dwmac_set_clock_delay(struct platform_device *pdev,
				void __iomem *gmac_clk_reg, int interface)
{
	unsigned int delay;

	if (gmac_clk_reg == NULL)
		return;

	if (of_property_read_u32(pdev->dev.of_node, "rx-clk-delay",
				 &delay) == 0) {
		/* RX clk delay */
		writel(delay, gmac_clk_reg + GMAC_CLK_CFG1);
		pr_info("RX clk delay: 0x%X\n", delay);
	}

	if (of_property_read_u32(pdev->dev.of_node, "tx-clk-delay",
				 &delay) == 0) {
		/* TX clk delay */
		writel(delay, gmac_clk_reg + GMAC_CLK_CFG2);
		pr_info("TX clk delay: 0x%X\n", delay);
	}
}

/* set gmac pll divider (u_pll_clk_div) to get 250MHz clock */
static void thead_dwmac_set_pll_250M(void __iomem *gmac_clk_reg, int interface,
				unsigned int src_freq)
{
	volatile unsigned int reg;
	unsigned int div = 1;

	if (gmac_clk_reg == NULL)
		return;

	if (interface == PHY_INTERFACE_MODE_MII) {
		/* For MII, no internal PLL is used */
		return;
	} else if (interface == PHY_INTERFACE_MODE_GMII
		|| interface == PHY_INTERFACE_MODE_RGMII
		|| interface == PHY_INTERFACE_MODE_RGMII_ID
		|| interface == PHY_INTERFACE_MODE_RGMII_RXID
		|| interface == PHY_INTERFACE_MODE_RGMII_TXID) {

		/* check clock */
		if ((src_freq == 0) || (src_freq % GMAC_CLK_PLLOUT_250M != 0)) {
			pr_err("error! invalid gmac pll freq %d\n", src_freq);
			return;
		}
		div = src_freq / GMAC_CLK_PLLOUT_250M;

		/* disable pll_clk_div */
		reg = readl(gmac_clk_reg + GMAC_CLK_CFG3);
		reg &= ~BIT(31);
		writel(reg, gmac_clk_reg + GMAC_CLK_CFG3);

		/* modify divider */
		writel(div, gmac_clk_reg + GMAC_CLK_CFG3);

		/* enable pll_clk_div */
		reg = readl(gmac_clk_reg + GMAC_CLK_CFG3);
		reg |= BIT(31);
		writel(reg, gmac_clk_reg + GMAC_CLK_CFG3);
	} else {
		pr_err("phy interface %d not supported\n", interface);
		return;
	}
}

/* set gmac speed */
static void thead_dwmac_set_speed(void __iomem *gmac_clk_reg, int interface,
				unsigned int speed)
{
	volatile unsigned int reg;

	if (gmac_clk_reg == NULL)
		return;

	if (interface == PHY_INTERFACE_MODE_MII) {
		/* For MII, no internal PLL is used */
		return;
	} else if (interface == PHY_INTERFACE_MODE_GMII
		|| interface == PHY_INTERFACE_MODE_RGMII
		|| interface == PHY_INTERFACE_MODE_RGMII_ID
		|| interface == PHY_INTERFACE_MODE_RGMII_RXID
		|| interface == PHY_INTERFACE_MODE_RGMII_TXID) {

		/* disable gtx_clk_div */
		reg = readl(gmac_clk_reg + GMAC_CLK_CFG4);
		reg &= ~BIT(31);
		writel(reg, gmac_clk_reg + GMAC_CLK_CFG4);

		/*
		 * modify divider
		 */
		/* gtx_clk_div */
		if (speed == SPEED_1000) {
			writel(GMAC_CLKDIV_125M, gmac_clk_reg + GMAC_CLK_CFG4);
		} else if (speed == SPEED_100) {
			writel(GMAC_CLKDIV_25M, gmac_clk_reg + GMAC_CLK_CFG4);
		} else {
			writel(GMAC_CLKDIV_25M / 10, gmac_clk_reg + GMAC_CLK_CFG4);
		}

		/* enable gtx_clk_div */
		reg = readl(gmac_clk_reg + GMAC_CLK_CFG4);
		reg |= BIT(31);
		writel(reg, gmac_clk_reg + GMAC_CLK_CFG4);
	} else {
		pr_err("phy interface %d not supported\n", interface);
		return;
	}
}

/* enable gmac clock */
static void thead_dwmac_enable_clock(struct platform_device *pdev,
				void __iomem *gmac_clk_reg, int interface)
{
	struct device *dev = &pdev->dev;
	volatile unsigned int reg;

	if (gmac_clk_reg == NULL)
		return;

	reg = readl(gmac_clk_reg + GMAC_CLK_CFG0);

	/* enable gmac_hclk */
	reg |= BIT(14);

	if (interface == PHY_INTERFACE_MODE_MII) {
		reg |= BIT(8);  /* enable gmac_rx_clk */
		reg |= BIT(3);  /* enable gmac_tx_clk */
	} else if (interface == PHY_INTERFACE_MODE_GMII) {
		reg |= BIT(8);  /* enable gmac_rx_clk */
		reg |= BIT(3);  /* enable gmac_tx_clk */
		reg |= BIT(6);  /* enable gmac_tx_clk_out */
	} else if (interface == PHY_INTERFACE_MODE_RGMII
		|| interface == PHY_INTERFACE_MODE_RGMII_ID
		|| interface == PHY_INTERFACE_MODE_RGMII_RXID
		|| interface == PHY_INTERFACE_MODE_RGMII_TXID) {
		reg |= BIT(8);  /* enable gmac_rx_clk */
		reg |= BIT(3);  /* enable gmac_tx_clk */
		reg |= BIT(6);  /* enable gmac_tx_clk_out */
		reg |= BIT(9);  /* enable gmac_rx_clk_n */
		reg |= BIT(4);  /* enable gmac_tx_clk_n */
	} else {
		dev_err(dev, "phy interface %d not supported\n", interface);
		return;
	}

	writel(reg, gmac_clk_reg + GMAC_CLK_CFG0);
}

#if 0
/* disable gmac clock */
static void thead_dwmac_disable_clock(struct platform_device *pdev,
				void __iomem *gmac_clk_reg, int interface)
{
	struct device *dev = &pdev->dev;
	volatile unsigned int reg;

	if (gmac_clk_reg == NULL)
		return;

	reg = readl(gmac_clk_reg + GMAC_CLK_CFG0);

	/* disable gmac_hclk */
	reg &= ~BIT(14);

	if (interface == PHY_INTERFACE_MODE_MII) {
		reg &= ~BIT(8); /* disable gmac_rx_clk */
		reg &= ~BIT(3); /* disable gmac_tx_clk */
	} else if (interface == PHY_INTERFACE_MODE_GMII) {
		reg &= ~BIT(8); /* disable gmac_rx_clk */
		reg &= ~BIT(3); /* disable gmac_tx_clk */
		reg &= ~BIT(6); /* disable gmac_tx_clk_out */
	} else if (interface == PHY_INTERFACE_MODE_RGMII
		|| interface == PHY_INTERFACE_MODE_RGMII_ID
		|| interface == PHY_INTERFACE_MODE_RGMII_RXID
		|| interface == PHY_INTERFACE_MODE_RGMII_TXID) {
		reg &= ~BIT(8); /* disable gmac_rx_clk */
		reg &= ~BIT(3); /* disable gmac_tx_clk */
		reg &= ~BIT(6); /* disable gmac_tx_clk_out */
		reg &= ~BIT(9); /* disable gmac_rx_clk_n */
		reg &= ~BIT(4); /* disable gmac_tx_clk_n */
	} else {
		dev_err(dev, "phy interface %d not supported\n", interface);
		return;
	}

	writel(reg, gmac_clk_reg + GMAC_CLK_CFG0);
}
#endif

static int thead_dwmac_init(struct platform_device *pdev, void *bsp_priv)
{
	struct thead_dwmac_priv_data *thead_plat_dat = bsp_priv;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	void __iomem *ptr;
	struct clk *clktmp;
	int ret;

	thead_plat_dat->id = of_alias_get_id(np, "ethernet");
	if (thead_plat_dat->id < 0) {
		thead_plat_dat->id = 0;
	}
	dev_info(dev, "id: %d\n", thead_plat_dat->id);

	if (of_get_phy_mode(dev->of_node, &(thead_plat_dat->interface))) {
		dev_err(dev, "of_get_phy_mode error\n");
		return -1;
	}

	dev_info(dev, "phy interface: %d\n", thead_plat_dat->interface);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy_if_reg");
	if ((res != NULL) && (resource_type(res) == IORESOURCE_MEM)) {
		ptr = devm_ioremap(dev, res->start, resource_size(res));
		if (!ptr) {
			dev_err(dev, "phy interface register not exist, skipped it\n");
		} else {
			thead_plat_dat->phy_if_reg = ptr;
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "txclk_dir_reg");
	ptr = devm_ioremap_resource(dev, res);
	if (IS_ERR(ptr)) {
		dev_err(dev, "txclk_dir register not exist, skipped it\n");
	} else {
		thead_plat_dat->txclk_dir_reg = ptr;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "clk_mgr_reg");
	ptr = devm_ioremap_resource(dev, res);
	if (IS_ERR(ptr)) {
		dev_err(dev, "gmac_clk register not exist, skipped it\n");
	} else {
		thead_plat_dat->gmac_clk_reg = ptr;
	}

	/* get gmac pll clk */
	clktmp = devm_clk_get(dev, "gmac_pll_clk");
	if (IS_ERR(clktmp)) {
		dev_err(dev, "gmac_pll_clk not exist, skipped it\n");
	} else {
		thead_plat_dat->gmac_pll_clk = clktmp;

		ret = clk_prepare_enable(thead_plat_dat->gmac_pll_clk);
		if (ret) {
			dev_err(dev, "Failed to enable clk 'gmac_pll_clk'\n");
			return -1;
		}

		thead_plat_dat->gmac_pll_clk_freq =
				clk_get_rate(thead_plat_dat->gmac_pll_clk);
	}

	thead_dwmac_set_phy_if(pdev, thead_plat_dat->phy_if_reg,
				thead_plat_dat->interface, thead_plat_dat->id);

	thead_dwmac_set_txclk_dir(pdev, thead_plat_dat->txclk_dir_reg,
				thead_plat_dat->interface);

	thead_dwmac_set_clk_source(pdev, thead_plat_dat->gmac_clk_reg,
				thead_plat_dat->interface);
	thead_dwmac_set_clock_delay(pdev, thead_plat_dat->gmac_clk_reg,
				thead_plat_dat->interface);

	thead_dwmac_set_pll_250M(thead_plat_dat->gmac_clk_reg,
				thead_plat_dat->interface,
				thead_plat_dat->gmac_pll_clk_freq);

	/* default speed is 1Gbps */
	thead_dwmac_set_speed(thead_plat_dat->gmac_clk_reg,
				thead_plat_dat->interface, SPEED_1000);

	thead_dwmac_enable_clock(pdev, thead_plat_dat->gmac_clk_reg,
				thead_plat_dat->interface);
	return 0;
}

static void thead_dwmac_fix_speed(void *bsp_priv, unsigned int speed)
{
	struct thead_dwmac_priv_data *thead_plat_dat = bsp_priv;

	thead_dwmac_set_speed(thead_plat_dat->gmac_clk_reg,
				thead_plat_dat->interface, speed);
}

/**
 * dwmac1000_validate_mcast_bins - validates the number of Multicast filter bins
 * @mcast_bins: Multicast filtering bins
 * Description:
 * this function validates the number of Multicast filtering bins specified
 * by the configuration through the device tree. The Synopsys GMAC supports
 * 64 bins, 128 bins, or 256 bins. "bins" refer to the division of CRC
 * number space. 64 bins correspond to 6 bits of the CRC, 128 corresponds
 * to 7 bits, and 256 refers to 8 bits of the CRC. Any other setting is
 * invalid and will cause the filtering algorithm to use Multicast
 * promiscuous mode.
 */
static int dwmac1000_validate_mcast_bins(int mcast_bins)
{
	int x = mcast_bins;

	switch (x) {
	case HASH_TABLE_SIZE:
	case 128:
	case 256:
		break;
	default:
		x = 0;
		pr_info("Hash table entries set to unexpected value %d",
			mcast_bins);
		break;
	}
	return x;
}

/**
 * dwmac1000_validate_ucast_entries - validate the Unicast address entries
 * @ucast_entries: number of Unicast address entries
 * Description:
 * This function validates the number of Unicast address entries supported
 * by a particular Synopsys 10/100/1000 controller. The Synopsys controller
 * supports 1..32, 64, or 128 Unicast filter entries for it's Unicast filter
 * logic. This function validates a valid, supported configuration is
 * selected, and defaults to 1 Unicast address if an unsupported
 * configuration is selected.
 */
static int dwmac1000_validate_ucast_entries(int ucast_entries)
{
	int x = ucast_entries;

	switch (x) {
	case 1 ... 32:
	case 64:
	case 128:
		break;
	default:
		x = 1;
		pr_info("Unicast table entries set to unexpected value %d\n",
			ucast_entries);
		break;
	}
	return x;
}

static int thead_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct thead_dwmac_priv_data *thead_plat_dat;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	thead_plat_dat = devm_kzalloc(dev, sizeof(*thead_plat_dat), GFP_KERNEL);
	if (thead_plat_dat == NULL) {
		dev_err(&pdev->dev, "allocate memory failed\n");
		return -ENOMEM;
	}

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	if (pdev->dev.of_node) {
		plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
		if (IS_ERR(plat_dat)) {
			dev_err(&pdev->dev, "dt configuration failed\n");
			return PTR_ERR(plat_dat);
		}
	} else {
		plat_dat = dev_get_platdata(&pdev->dev);
		if (!plat_dat) {
			dev_err(&pdev->dev, "no platform data provided\n");
			return  -EINVAL;
		}

		/* Set default value for multicast hash bins */
		plat_dat->multicast_filter_bins = HASH_TABLE_SIZE;

		/* Set default value for unicast filter entries */
		plat_dat->unicast_filter_entries = 1;
	}

	/* Custom initialisation (if needed) */
	if (plat_dat->init) {
		ret = plat_dat->init(pdev, plat_dat->bsp_priv);
		if (ret)
			goto err_remove_config_dt;
	}

	/* populate bsp private data */
	plat_dat->bsp_priv = thead_plat_dat;
	plat_dat->fix_mac_speed = thead_dwmac_fix_speed;
	of_property_read_u32(np, "max-frame-size", &plat_dat->maxmtu);
	of_property_read_u32(np, "snps,multicast-filter-bins",
			     &plat_dat->multicast_filter_bins);
	of_property_read_u32(np, "snps,perfect-filter-entries",
			     &plat_dat->unicast_filter_entries);
	plat_dat->unicast_filter_entries = dwmac1000_validate_ucast_entries(
				       plat_dat->unicast_filter_entries);
	plat_dat->multicast_filter_bins = dwmac1000_validate_mcast_bins(
				      plat_dat->multicast_filter_bins);
	plat_dat->has_gmac = 1;
	plat_dat->pmt = 1;

	ret = thead_dwmac_init(pdev, plat_dat->bsp_priv);
	if (ret)
		goto err_exit;

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_exit;

	return 0;

err_exit:
	if (plat_dat->exit)
		plat_dat->exit(pdev, plat_dat->bsp_priv);
err_remove_config_dt:
	if (pdev->dev.of_node)
		stmmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static const struct of_device_id thead_dwmac_match[] = {
	{ .compatible = "thead,dwmac"},
	{ }
};
MODULE_DEVICE_TABLE(of, thead_dwmac_match);

static struct platform_driver thead_dwmac_driver = {
	.probe  = thead_dwmac_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name           = "thead_dwmac_eth",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = of_match_ptr(thead_dwmac_match),
	},
};
module_platform_driver(thead_dwmac_driver);

MODULE_DESCRIPTION("T-HEAD dwmac driver");
MODULE_LICENSE("GPL v2");
