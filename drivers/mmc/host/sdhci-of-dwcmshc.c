// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Synopsys DesignWare Cores Mobile Storage Host Controller
 *
 * Copyright (C) 2018 Synaptics Incorporated
 *
 * Author: Jisheng Zhang <jszhang@kernel.org>
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/sizes.h>
#include <linux/delay.h>

#include "sdhci-pltfm.h"
#include "sdhci-of-dwcmshc.h"

#define SDHCI_DWCMSHC_ARG2_STUFF	GENMASK(31, 16)

/* DWCMSHC specific Mode Select value */
#define DWCMSHC_CTRL_HS400		0x7

#define BOUNDARY_OK(addr, len) \
	((addr | (SZ_128M - 1)) == ((addr + len - 1) | (SZ_128M - 1)))

struct dwcmshc_priv {
	struct clk	*bus_clk;
	void __iomem *soc_base;
	bool is_emmc_card;
	bool pull_up_en;
	bool io_fixed_1v8;
	bool wprtn_ignore;
};

#define HS400_DELAY_LINE 24

static uint32_t delay_line = 50;

static void sdhci_phy_1_8v_init_no_pull(struct sdhci_host *host)
{
	uint32_t val;
	sdhci_writel(host, 1, DWC_MSHC_PTR_PHY_R);
	sdhci_writeb(host, 1 << 4, PHY_SDCLKDL_CNFG_R);
	sdhci_writeb(host, 0x40, PHY_SDCLKDL_DC_R);

	val = sdhci_readb(host, PHY_SDCLKDL_CNFG_R);
	val &= ~(1 << 4);
	sdhci_writeb(host, val, PHY_SDCLKDL_CNFG_R);


	val = sdhci_readw(host, PHY_CMDPAD_CNFG_R);
	sdhci_writew(host, val | 1, PHY_CMDPAD_CNFG_R);

	val = sdhci_readw(host, PHY_DATAPAD_CNFG_R);
	sdhci_writew(host, val | 1, PHY_DATAPAD_CNFG_R);

	val = sdhci_readw(host, PHY_RSTNPAD_CNFG_R);
	sdhci_writew(host, val | 1, PHY_RSTNPAD_CNFG_R);

	val = sdhci_readw(host, PHY_STBPAD_CNFG_R);
	sdhci_writew(host, val | 1, PHY_STBPAD_CNFG_R);

	val = sdhci_readb(host, PHY_DLL_CTRL_R);
	sdhci_writeb(host, val | 1, PHY_DLL_CTRL_R);
}

static void sdhci_phy_3_3v_init_no_pull(struct sdhci_host *host)
{
	uint32_t val;
	sdhci_writel(host, 1, DWC_MSHC_PTR_PHY_R);
	sdhci_writeb(host, 1 << 4, PHY_SDCLKDL_CNFG_R);
	sdhci_writeb(host, 0x40, PHY_SDCLKDL_DC_R);

	val = sdhci_readb(host, PHY_SDCLKDL_CNFG_R);
	val &= ~(1 << 4);
	sdhci_writeb(host, val, PHY_SDCLKDL_CNFG_R);

	val = sdhci_readw(host, PHY_CMDPAD_CNFG_R);
	sdhci_writew(host, val | 2, PHY_CMDPAD_CNFG_R);

	val = sdhci_readw(host, PHY_DATAPAD_CNFG_R);
	sdhci_writew(host, val | 2, PHY_DATAPAD_CNFG_R);

	val = sdhci_readw(host, PHY_RSTNPAD_CNFG_R);
	sdhci_writew(host, val | 2, PHY_RSTNPAD_CNFG_R);

	val = sdhci_readw(host, PHY_STBPAD_CNFG_R);
	sdhci_writew(host, val | 2, PHY_STBPAD_CNFG_R);

	val = sdhci_readb(host, PHY_DLL_CTRL_R);
	sdhci_writeb(host, val | 1, PHY_DLL_CTRL_R);
}

static void snps_phy_1_8v_init(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct dwcmshc_priv *priv;
	u32 val;

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);
	if (priv->pull_up_en == 0) {
		sdhci_phy_1_8v_init_no_pull(host);
		return;
	}

	//set driving force
	sdhci_writel(host, (1 << PHY_RSTN) | (0xc << PAD_SP) | (0xc << PAD_SN), PHY_CNFG_R);

	//disable delay lane
	sdhci_writeb(host, 1 << UPDATE_DC, PHY_SDCLKDL_CNFG_R);
	//set delay lane
	sdhci_writeb(host, delay_line, PHY_SDCLKDL_DC_R);
	sdhci_writeb(host, 0xa, PHY_DLL_CNFG2_R);
	//enable delay lane
	val = sdhci_readb(host, PHY_SDCLKDL_CNFG_R);
	val &= ~(1 << UPDATE_DC);
	sdhci_writeb(host, val, PHY_SDCLKDL_CNFG_R);

	val = (1 << RXSEL) | (1 << WEAKPULL_EN) | (3 << TXSLEW_CTRL_P) | (3 << TXSLEW_CTRL_N);
	sdhci_writew(host, val, PHY_CMDPAD_CNFG_R);
	sdhci_writew(host, val, PHY_DATAPAD_CNFG_R);
	sdhci_writew(host, val, PHY_RSTNPAD_CNFG_R);

	val = (3 << TXSLEW_CTRL_P) | (3 << TXSLEW_CTRL_N);
	sdhci_writew(host, val, PHY_CLKPAD_CNFG_R);

	val = (1 << RXSEL) | (2 << WEAKPULL_EN) | (3 << TXSLEW_CTRL_P) | (3 << TXSLEW_CTRL_N);
	sdhci_writew(host, val, PHY_STBPAD_CNFG_R);

	/* enable data strobe mode */
	sdhci_writeb(host, 3 << SLV_INPSEL, PHY_DLLDL_CNFG_R);
	sdhci_writeb(host, (1 << DLL_EN),  PHY_DLL_CTRL_R);
}

static void snps_phy_3_3v_init(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct dwcmshc_priv *priv;
	u32 val;

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);
	if (priv->pull_up_en == 0) {
		sdhci_phy_3_3v_init_no_pull(host);
		return;
	}

	//set driving force
	sdhci_writel(host, (1 << PHY_RSTN) | (0xc << PAD_SP) | (0xc << PAD_SN), PHY_CNFG_R);

	//disable delay lane
	sdhci_writeb(host, 1 << UPDATE_DC, PHY_SDCLKDL_CNFG_R);
	//set delay lane
	sdhci_writeb(host, delay_line, PHY_SDCLKDL_DC_R);
	sdhci_writeb(host, 0xa, PHY_DLL_CNFG2_R);
	//enable delay lane
	val = sdhci_readb(host, PHY_SDCLKDL_CNFG_R);
	val &= ~(1 << UPDATE_DC);
	sdhci_writeb(host, val, PHY_SDCLKDL_CNFG_R);

	val = (2 << RXSEL) | (1 << WEAKPULL_EN) | (3 << TXSLEW_CTRL_P) | (3 << TXSLEW_CTRL_N);
	sdhci_writew(host, val, PHY_CMDPAD_CNFG_R);
	sdhci_writew(host, val, PHY_DATAPAD_CNFG_R);
	sdhci_writew(host, val, PHY_RSTNPAD_CNFG_R);

	val = (3 << TXSLEW_CTRL_P) | (3 << TXSLEW_CTRL_N);
	sdhci_writew(host, val, PHY_CLKPAD_CNFG_R);

	val = (2 << RXSEL) | (2 << WEAKPULL_EN) | (3 << TXSLEW_CTRL_P) | (3 << TXSLEW_CTRL_N);
	sdhci_writew(host, val, PHY_STBPAD_CNFG_R);
}

static int __sdhci_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	#define DW_SDHCI_TUNING_LOOP_COUNT 128
	int i;
	/*
	 * Issue opcode repeatedly till Execute Tuning is set to 0 or the number
	 * of loops reaches tuning loop count.
	 */
	for (i = 0; i < DW_SDHCI_TUNING_LOOP_COUNT; i++) {
		u16 ctrl;

		sdhci_send_tuning(host, opcode);

		if (!host->tuning_done) {
			pr_debug("%s: Tuning timeout, falling back to fixed sampling clock\n",
				 mmc_hostname(host->mmc));
			sdhci_abort_tuning(host, opcode);
			return -ETIMEDOUT;
		}

		/* Spec does not require a delay between tuning cycles */
		if (host->tuning_delay > 0)
			mdelay(host->tuning_delay);

		ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
		if (!(ctrl & SDHCI_CTRL_EXEC_TUNING)) {
			if (ctrl & SDHCI_CTRL_TUNED_CLK)
				return 0; /* Success! */
			break;
		}
	}

	pr_info("%s: Tuning failed, falling back to fixed sampling clock\n",
	mmc_hostname(host->mmc));
	printk("%s: Tuning failed, falling back to fixed sampling clock\n",
	mmc_hostname(host->mmc));
	sdhci_reset_tuning(host);
	return -EAGAIN;
}

static int snps_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	u32 val = 0;

	if (host->flags & SDHCI_HS400_TUNING) {
		return 0;
	}

	sdhci_writeb(host, 3 << INPSEL_CNFG, PHY_ATDL_CNFG_R);

	val = sdhci_readl(host, AT_CTRL_R);

	val &= ~((1 << CI_SEL) | (1 << RPT_TUNE_ERR)\
	    | (1 << SW_TUNE_EN) |(0xf << WIN_EDGE_SEL));
	val |= (1 << AT_EN) | (1 << SWIN_TH_EN) | (1 << TUNE_CLK_STOP_EN)\
	    | (1 << PRE_CHANGE_DLY) | (3 << POST_CHANGE_DLY) | (9 << SWIN_TH_VAL);

	sdhci_writel(host, val, AT_CTRL_R);
	val = sdhci_readl(host, AT_CTRL_R);
	if(!(val & (1 << AT_EN))) {
		pr_err("*****Auto Tuning is NOT Enable!!!\n");
		return -1;
	}

	val &= ~(1 << AT_EN);
	sdhci_writel(host, val, AT_CTRL_R);

	sdhci_start_tuning(host);

	host->tuning_err = __sdhci_execute_tuning(host, opcode);
	if (host->tuning_err) {
	val &= ~(1 << AT_EN);
	sdhci_writel(host, val, AT_CTRL_R);
	return -1;
	}

	sdhci_end_tuning(host);

	return 0;
}

static void snps_sdhci_reset(struct sdhci_host *host, u8 mask)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct dwcmshc_priv *priv;
	u8 emmc_ctl;
	//u32 soc_reg;

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);

	/*soc reset, fix host reset error*/
	//soc_reg = readl( priv->soc_base);
	//soc_reg &= ~1;
	//writel(soc_reg, priv->soc_base);
	//soc_reg |= 1;
	//writel(soc_reg, priv->soc_base);

	/*host reset*/
	sdhci_reset(host, mask);
	/*fix host reset error*/
	mdelay(100);

	emmc_ctl = sdhci_readw(host, EMMC_CTRL_R);
	if (priv->is_emmc_card) {
		snps_phy_1_8v_init(host);
		emmc_ctl |= (1 << CARD_IS_EMMC);
	} else {
		snps_phy_3_3v_init(host);
		emmc_ctl &=~(1 << CARD_IS_EMMC);
	}
	sdhci_writeb(host, emmc_ctl, EMMC_CTRL_R);
	sdhci_writeb(host, 0x25, PHY_DLL_CNFG1_R);
}
/*
 * If DMA addr spans 128MB boundary, we split the DMA transfer into two
 * so that each DMA transfer doesn't exceed the boundary.
 */
static void dwcmshc_adma_write_desc(struct sdhci_host *host, void **desc,
				    dma_addr_t addr, int len, unsigned int cmd)
{
	int tmplen, offset;

	if (likely(!len || BOUNDARY_OK(addr, len))) {
		sdhci_adma_write_desc(host, desc, addr, len, cmd);
		return;
	}

	offset = addr & (SZ_128M - 1);
	tmplen = SZ_128M - offset;
	sdhci_adma_write_desc(host, desc, addr, tmplen, cmd);

	addr += tmplen;
	len -= tmplen;
	sdhci_adma_write_desc(host, desc, addr, len, cmd);
}

static void dwcmshc_check_auto_cmd23(struct mmc_host *mmc,
				     struct mmc_request *mrq)
{
	struct sdhci_host *host = mmc_priv(mmc);

	/*
	 * No matter V4 is enabled or not, ARGUMENT2 register is 32-bit
	 * block count register which doesn't support stuff bits of
	 * CMD23 argument on dwcmsch host controller.
	 */
	if (mrq->sbc && (mrq->sbc->arg & SDHCI_DWCMSHC_ARG2_STUFF))
		host->flags &= ~SDHCI_AUTO_CMD23;
	else
		host->flags |= SDHCI_AUTO_CMD23;
}

static void dwcmshc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	dwcmshc_check_auto_cmd23(mmc, mrq);

	sdhci_request(mmc, mrq);
}

static void dwcmshc_set_uhs_signaling(struct sdhci_host *host,
				      unsigned int timing)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);
	u16 ctrl_2;

	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	/* Select Bus Speed Mode for host */
	ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
	if ((timing == MMC_TIMING_MMC_HS200) ||
	    (timing == MMC_TIMING_UHS_SDR104))
		ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
	else if (timing == MMC_TIMING_UHS_SDR12)
		ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
	else if ((timing == MMC_TIMING_UHS_SDR25) ||
		 (timing == MMC_TIMING_MMC_HS))
		ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
	else if (timing == MMC_TIMING_UHS_SDR50)
		ctrl_2 |= SDHCI_CTRL_UHS_SDR50;
	else if ((timing == MMC_TIMING_UHS_DDR50) ||
		 (timing == MMC_TIMING_MMC_DDR52))
		ctrl_2 |= SDHCI_CTRL_UHS_DDR50;
	else if (timing == MMC_TIMING_MMC_HS400)
		ctrl_2 |= DWCMSHC_CTRL_HS400;

	if (priv->io_fixed_1v8)
		ctrl_2 |= SDHCI_CTRL_VDD_180;

	sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);

	if (timing == MMC_TIMING_MMC_HS400) {
		// //disable delay lane
		// sdhci_writeb(host, 1 << UPDATE_DC, PHY_SDCLKDL_CNFG_R);
		// //set delay lane
		// sdhci_writeb(host, delay_line, PHY_SDCLKDL_DC_R);
		// //enable delay lane
		// reg = sdhci_readb(host, PHY_SDCLKDL_CNFG_R);
		// reg &= ~(1 << UPDATE_DC);
		// sdhci_writeb(host, reg, PHY_SDCLKDL_CNFG_R);

		//disable auto tuning
		u32 reg = sdhci_readl(host, AT_CTRL_R);
		reg &= ~1;
		sdhci_writel(host, reg, AT_CTRL_R);

		delay_line = HS400_DELAY_LINE;
	} else {
		sdhci_writeb(host, 0, PHY_DLLDL_CNFG_R);
	}
}

static unsigned int dwcmshc_pltfm_get_ro(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);
	int is_readonly;

	if (priv->wprtn_ignore)
		return 0;
	is_readonly = !(sdhci_readl(host, SDHCI_PRESENT_STATE)
			& SDHCI_WRITE_PROTECT);

	return is_readonly;
}

static const struct sdhci_ops sdhci_dwcmshc_ops = {
	.set_clock		= sdhci_set_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.set_uhs_signaling	= dwcmshc_set_uhs_signaling,
	.get_max_clock		= sdhci_pltfm_clk_get_max_clock,
	.get_ro			= dwcmshc_pltfm_get_ro,
	.reset			= snps_sdhci_reset,
	.adma_write_desc	= dwcmshc_adma_write_desc,
	.voltage_switch		= snps_phy_1_8v_init,
	.platform_execute_tuning = &snps_execute_tuning,

};

static const struct sdhci_pltfm_data sdhci_dwcmshc_pdata = {
	.ops = &sdhci_dwcmshc_ops,
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
};

static int dwcmshc_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	struct dwcmshc_priv *priv;
	int err;
	u32 extra;

	host = sdhci_pltfm_init(pdev, &sdhci_dwcmshc_pdata,
				sizeof(struct dwcmshc_priv));
	if (IS_ERR(host))
		return PTR_ERR(host);

	/*
	 * extra adma table cnt for cross 128M boundary handling.
	 */
	extra = DIV_ROUND_UP_ULL(dma_get_required_mask(&pdev->dev), SZ_128M);
	if (extra > SDHCI_MAX_SEGS)
		extra = SDHCI_MAX_SEGS;
	host->adma_table_cnt += extra;
	host->v4_mode = true;

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);

	/*used fix sdhci reset error*/
	priv->soc_base = devm_platform_ioremap_resource(pdev, 1);

	if (device_property_present(&pdev->dev, "is_emmc")) {
		priv->is_emmc_card = 1;
	} else {
		priv->is_emmc_card = 0;
	}

	if (device_property_present(&pdev->dev, "pull_up")) {
		priv->pull_up_en = 1;
	} else {
		priv->pull_up_en = 0;
	}

	if (device_property_present(&pdev->dev, "io_fixed_1v8"))
		priv->io_fixed_1v8 = true;
	else
		priv->io_fixed_1v8 = false;

	if (device_property_present(&pdev->dev, "wprtn_ignore"))
		priv->wprtn_ignore = true;
	else
		priv->wprtn_ignore = false;

	pltfm_host->clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(pltfm_host->clk)) {
		err = PTR_ERR(pltfm_host->clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", err);
		goto free_pltfm;
	}
	err = clk_prepare_enable(pltfm_host->clk);
	if (err)
		goto free_pltfm;

	priv->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (!IS_ERR(priv->bus_clk))
		clk_prepare_enable(priv->bus_clk);

	err = mmc_of_parse(host->mmc);
	if (err)
		goto err_clk;

	sdhci_get_of_property(pdev);

	host->mmc_host_ops.request = dwcmshc_request;

	err = sdhci_add_host(host);
	if (err)
		goto err_clk;

	return 0;

err_clk:
	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);
free_pltfm:
	sdhci_pltfm_free(pdev);
	return err;
}

static int dwcmshc_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);

	sdhci_remove_host(host, 0);

	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);

	sdhci_pltfm_free(pdev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dwcmshc_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);
	int ret;

	ret = sdhci_suspend_host(host);
	if (ret)
		return ret;

	clk_disable_unprepare(pltfm_host->clk);
	if (!IS_ERR(priv->bus_clk))
		clk_disable_unprepare(priv->bus_clk);

	return ret;
}

static int dwcmshc_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);
	int ret;

	ret = clk_prepare_enable(pltfm_host->clk);
	if (ret)
		return ret;

	if (!IS_ERR(priv->bus_clk)) {
		ret = clk_prepare_enable(priv->bus_clk);
		if (ret)
			return ret;
	}

	return sdhci_resume_host(host);
}
#endif

static SIMPLE_DEV_PM_OPS(dwcmshc_pmops, dwcmshc_suspend, dwcmshc_resume);

static const struct of_device_id sdhci_dwcmshc_dt_ids[] = {
	{ .compatible = "snps,dwcmshc-sdhci" },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_dwcmshc_dt_ids);

static struct platform_driver sdhci_dwcmshc_driver = {
	.driver	= {
		.name	= "sdhci-dwcmshc",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = sdhci_dwcmshc_dt_ids,
		.pm = &dwcmshc_pmops,
	},
	.probe	= dwcmshc_probe,
	.remove	= dwcmshc_remove,
};
module_platform_driver(sdhci_dwcmshc_driver);

MODULE_DESCRIPTION("SDHCI platform driver for Synopsys DWC MSHC");
MODULE_AUTHOR("Jisheng Zhang <jszhang@kernel.org>");
MODULE_LICENSE("GPL v2");
