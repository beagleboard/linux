/*
 * drivers/mmc/host/omap_hsmmc.c
 *
 * Driver for OMAP2430/3430 MMC controller.
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * Authors:
 *	Syed Mohammed Khasim	<x0khasim@ti.com>
 *	Madhusudhan		<madhu.cr@ti.com>
 *	Mohit Jalori		<mjalori@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/dmaengine.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeirq.h>
#include <linux/platform_data/hsmmc-omap.h>
#include <linux/mmc/sd.h>

/* OMAP HSMMC Host Controller Registers */
#define OMAP_HSMMC_HL_REV	0x0000
#define OMAP_HSMMC_HL_HWINFO	0x0004
#define OMAP_HSMMC_HL_SYSCONFIG	0x0010
#define OMAP_HSMMC_SYSSTATUS	0x0014
#define OMAP_HSMMC_CON		0x002C
#define OMAP_HSMMC_DLL		0x0034
#define OMAP_HSMMC_SDMASA	0x0100
#define OMAP_HSMMC_BLK		0x0104
#define OMAP_HSMMC_ARG		0x0108
#define OMAP_HSMMC_CMD		0x010C
#define OMAP_HSMMC_RSP10	0x0110
#define OMAP_HSMMC_RSP32	0x0114
#define OMAP_HSMMC_RSP54	0x0118
#define OMAP_HSMMC_RSP76	0x011C
#define OMAP_HSMMC_DATA		0x0120
#define OMAP_HSMMC_PSTATE	0x0124
#define OMAP_HSMMC_HCTL		0x0128
#define OMAP_HSMMC_SYSCTL	0x012C
#define OMAP_HSMMC_STAT		0x0130
#define OMAP_HSMMC_IE		0x0134
#define OMAP_HSMMC_ISE		0x0138
#define OMAP_HSMMC_AC12		0x013C
#define OMAP_HSMMC_CAPA		0x0140
#define OMAP_HSMMC_CAPA2	0x0144
#define OMAP_HSMMC_ADMAES	0x0154
#define OMAP_HSMMC_ADMASAL	0x0158

#define MADMA_EN		(1 << 0)
#define VS18			(1 << 26)
#define VS30			(1 << 25)
#define HSS			(1 << 21)
#define SDVS18			(0x5 << 9)
#define SDVS30			(0x6 << 9)
#define SDVS33			(0x7 << 9)
#define SDVS_MASK		0x00000E00
#define SDVSCLR			0xFFFFF1FF
#define SDVSDET			0x00000400
#define DMA_SELECT		(2 << 3)
#define AUTOIDLE		0x1
#define SDBP			(1 << 8)
#define DTO			0xe
#define ICE			0x1
#define ICS			0x2
#define CEN			(1 << 2)
#define CLKD_MAX		0x3FF		/* max clock divisor: 1023 */
#define CLKD_MASK		0x0000FFC0
#define CLKD_SHIFT		6
#define DTO_MASK		0x000F0000
#define DTO_SHIFT		16
#define INIT_STREAM		(1 << 1)
#define ACEN_ACMD23		(2 << 2)
#define DP_SELECT		(1 << 21)
#define DDIR			(1 << 4)
#define DMAE			0x1
#define MSBS			(1 << 5)
#define BCE			(1 << 1)
#define FOUR_BIT		(1 << 1)
#define HSPE			(1 << 2)
#define IWE			(1 << 24)
#define DMA_MASTER		(1 << 20)
#define DDR			(1 << 19)
#define CLKEXTFREE		(1 << 16)
#define CTPL			(1 << 11)
#define DW8			(1 << 5)
#define OD			0x1
#define STAT_CLEAR		0xFFFFFFFF
#define INIT_STREAM_CMD		0x00000000
#define DUAL_VOLT_OCR_BIT	7
#define SRC			(1 << 25)
#define SRD			(1 << 26)
#define SOFTRESET		(1 << 1)

/* PSTATE */
#define DLEV_DAT(x)		(1 << (20 + (x)))

/* AC12 */
#define AC12_V1V8_SIGEN		(1 << 19)
#define AC12_SCLK_SEL		(1 << 23)
#define AC12_UHSMC_MASK		(7 << 16)
#define AC12_UHSMC_SDR12	(0 << 16)
#define AC12_UHSMC_SDR25	(1 << 16)
#define AC12_UHSMC_SDR50	(2 << 16)
#define AC12_UHSMC_SDR104	(3 << 16)
#define AC12_UHSMC_DDR50	(4 << 16)
#define AC12_UHSMC_RES		(0x7 << 16)

/* DLL */
#define DLL_SWT			(1 << 20)
#define DLL_FORCE_SR_C_SHIFT	13
#define DLL_FORCE_SR_C_MASK	0x7f
#define DLL_FORCE_VALUE		(1 << 12)
#define DLL_CALIB		(1 << 1)

#define MAX_PHASE_DELAY		0x7c

/* CAPA2 */
#define CAPA2_TSDR50		(1 << 13)

/* Interrupt masks for IE and ISE register */
#define CC_EN			(1 << 0)
#define TC_EN			(1 << 1)
#define BWR_EN			(1 << 4)
#define BRR_EN			(1 << 5)
#define CIRQ_EN			(1 << 8)
#define ERR_EN			(1 << 15)
#define CTO_EN			(1 << 16)
#define CCRC_EN			(1 << 17)
#define CEB_EN			(1 << 18)
#define CIE_EN			(1 << 19)
#define DTO_EN			(1 << 20)
#define DCRC_EN			(1 << 21)
#define DEB_EN			(1 << 22)
#define ACE_EN			(1 << 24)
#define ADMAE_EN		(1 << 24)
#define CERR_EN			(1 << 28)
#define BADA_EN			(1 << 29)

#define INT_EN_MASK (BADA_EN | CERR_EN | ADMAE_EN | ACE_EN | DEB_EN | DCRC_EN |\
		DTO_EN | CIE_EN | CEB_EN | CCRC_EN | CTO_EN | \
		BRR_EN | BWR_EN | TC_EN | CC_EN)

#define CNI	(1 << 7)
#define ACIE	(1 << 4)
#define ACEB	(1 << 3)
#define ACCE	(1 << 2)
#define ACTO	(1 << 1)
#define ACNE	(1 << 0)

#define MMC_AUTOSUSPEND_DELAY	100
#define MMC_TIMEOUT_MS		20		/* 20 mSec */
#define MMC_TIMEOUT_US		20000		/* 20000 micro Sec */
#define OMAP_MMC_MIN_CLOCK	400000
#define OMAP_MMC_MAX_CLOCK	52000000
#define DRIVER_NAME		"omap_hsmmc"

#define VDD_1V8			1800000		/* 180000 uV */
#define VDD_3V0			3000000		/* 300000 uV */
#define VDD_165_195		(ffs(MMC_VDD_165_195) - 1)
#define VDD_30_31		(ffs(MMC_VDD_30_31) - 1)

#define CON_CLKEXTFREE		(1 << 16)
#define CON_PADEN		(1 << 15)
#define PSTATE_CLEV		(1 << 24)
#define PSTATE_DLEV		(0xF << 20)
#define PSTATE_DLEV_DAT0	(0x1 << 20)

#define MMC_BLOCK_TRANSFER_TIME_NS(blksz, bus_width, freq)		\
				   ((unsigned long long)		\
				   (2 * (((blksz) * NSEC_PER_SEC *	\
				   (8 / (bus_width))) / (freq))))

/*
 * One controller can have multiple slots, like on some omap boards using
 * omap.c controller driver. Luckily this is not currently done on any known
 * omap_hsmmc.c device.
 */
#define mmc_pdata(host)		host->pdata

/*
 * MMC Host controller read/write API's
 */
#define OMAP_HSMMC_READ(base, reg)	\
	__raw_readl((base) + OMAP_HSMMC_##reg)

#define OMAP_HSMMC_WRITE(base, reg, val) \
	__raw_writel((val), (base) + OMAP_HSMMC_##reg)

struct omap_hsmmc_adma_desc {
	u8 attr;
	u8 reserved;
	u16 len;
	u32 addr;
} __packed;

#define ADMA_MAX_LEN			65532

/* Decriptor table defines */
#define ADMA_DESC_ATTR_VALID		BIT(0)
#define ADMA_DESC_ATTR_END		BIT(1)
#define ADMA_DESC_ATTR_INT		BIT(2)
#define ADMA_DESC_ATTR_ACT1		BIT(4)
#define ADMA_DESC_ATTR_ACT2		BIT(5)

#define ADMA_DESC_TRANSFER_DATA		ADMA_DESC_ATTR_ACT2
#define ADMA_DESC_LINK_DESC	(ADMA_DESC_ATTR_ACT1 | ADMA_DESC_ATTR_ACT2)

/* ADMA error status */
#define AES_MASK		0x3
#define ST_FDS			0x0
#define ST_STOP			0x1
#define ST_TFR			0x3

struct omap_hsmmc_next {
	unsigned int	dma_len;
	s32		cookie;
};

struct omap_hsmmc_host {
	struct	device		*dev;
	struct	mmc_host	*mmc;
	struct	mmc_request	*mrq;
	struct	mmc_command	*cmd;
	u32			last_cmd;
	struct	mmc_data	*data;
	struct	clk		*fclk;
	struct	clk		*dbclk;
	struct	regulator	*pbias;
	bool			pbias_enabled;
	void	__iomem		*base;
	int			vqmmc_enabled;
	resource_size_t		mapbase;
	spinlock_t		irq_lock; /* Prevent races with irq handler */
	unsigned int		dma_len;
	unsigned int		dma_sg_idx;
	unsigned char		bus_mode;
	unsigned char		power_mode;
	unsigned char		timing;
	int			suspended;
	u32			con;
	u32			hctl;
	u32			sysctl;
	u32			capa;
	int			irq;
	int			wake_irq;
	int			dma_ch;
	int			use_adma;
	struct dma_chan		*tx_chan;
	struct dma_chan		*rx_chan;
	int			response_busy;
	int			context_loss;
	int			protect_card;
	int			reqs_blocked;
	int			req_in_progress;
	unsigned long		clk_rate;
	unsigned int		flags;
#define AUTO_CMD23		(1 << 0)        /* Auto CMD23 support */
#define HSMMC_SDIO_IRQ_ENABLED	(1 << 1)        /* SDIO irq enabled */
#define CLKEXTFREE_ENABLED	(1 << 2)        /* CLKEXTFREE enabled */
	struct omap_hsmmc_next	next_data;
	struct	omap_hsmmc_platform_data	*pdata;

	struct timer_list	timer;
	unsigned long long	data_timeout;

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pinctrl_state;
	struct pinctrl_state	*default_pinctrl_state;
	struct pinctrl_state	*sdr104_pinctrl_state;
	struct pinctrl_state	*hs200_1_8v_pinctrl_state;
	struct pinctrl_state	*ddr50_pinctrl_state;
	struct pinctrl_state	*sdr50_pinctrl_state;
	struct pinctrl_state	*sdr25_pinctrl_state;
	struct pinctrl_state	*sdr12_pinctrl_state;
	struct pinctrl_state	*hs_pinctrl_state;
	struct pinctrl_state	*ddr_1_8v_pinctrl_state;

	struct omap_hsmmc_adma_desc *adma_desc_table;
	dma_addr_t              adma_desc_table_addr;

	/* return MMC cover switch state, can be NULL if not supported.
	 *
	 * possible return values:
	 *   0 - closed
	 *   1 - open
	 */
	int (*get_cover_state)(struct device *dev);

	int (*card_detect)(struct device *dev);
};

struct omap_mmc_of_data {
	u32 reg_offset;
	u8 controller_flags;
};

static void omap_hsmmc_start_dma_transfer(struct omap_hsmmc_host *host);
static void omap_hsmmc_conf_bus_power(struct omap_hsmmc_host *host, int iov);
static void omap_hsmmc_disable_tuning(struct omap_hsmmc_host *host);

static int omap_hsmmc_card_detect(struct device *dev)
{
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);

	return mmc_gpio_get_cd(host->mmc);
}

static int omap_hsmmc_get_cover_state(struct device *dev)
{
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);

	return mmc_gpio_get_cd(host->mmc);
}

static int omap_hsmmc_enable_supply(struct mmc_host *mmc, int iov)
{
	int ret;
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	struct mmc_ios *ios = &mmc->ios;
	int uvoltage;

	if (mmc->supply.vmmc) {
		ret = mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);
		if (ret)
			return ret;
	}

	/* Enable interface voltage rail, if needed */
	if (mmc->supply.vqmmc) {
		if (host->vqmmc_enabled) {
			ret = regulator_disable(mmc->supply.vqmmc);
			if (ret) {
				dev_err(mmc_dev(mmc),
					"vmmc_aux reg disable failed\n");
				goto err_vqmmc;
			}
			host->vqmmc_enabled = 0;
		}

		uvoltage = (iov == VDD_165_195) ? VDD_1V8 : VDD_3V0;
		ret = regulator_set_voltage(mmc->supply.vqmmc, uvoltage,
					    uvoltage);
		if (ret) {
			dev_err(mmc_dev(mmc), "vmmc_aux set voltage failed\n");
			goto err_vqmmc;
		}

		ret = regulator_enable(mmc->supply.vqmmc);
		if (ret) {
			dev_err(mmc_dev(mmc), "vmmc_aux reg enable failed\n");
			goto err_vqmmc;
		}
		host->vqmmc_enabled = 1;
	}

	return 0;

err_vqmmc:
	if (mmc->supply.vmmc)
		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);

	return ret;
}

static int omap_hsmmc_disable_supply(struct mmc_host *mmc)
{
	int ret;
	int status;
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	if (mmc->supply.vqmmc && host->vqmmc_enabled) {
		ret = regulator_disable(mmc->supply.vqmmc);
		if (ret) {
			dev_err(mmc_dev(mmc), "vmmc_aux reg disable failed\n");
			return ret;
		}
		host->vqmmc_enabled = 0;
	}

	if (mmc->supply.vmmc) {
		ret = mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);
		if (ret)
			goto err_set_ocr;
	}

	return 0;

err_set_ocr:
	if (mmc->supply.vqmmc) {
		status = regulator_enable(mmc->supply.vqmmc);
		if (status)
			dev_err(mmc_dev(mmc), "vmmc_aux re-enable failed\n");
	}

	return ret;
}

static int omap_hsmmc_set_pbias(struct omap_hsmmc_host *host, bool power_on,
				int iov)
{
	int ret;
	int uvoltage;

	if (!host->pbias)
		return 0;

	if (power_on) {
		uvoltage = (iov <= VDD_165_195) ? VDD_1V8 : VDD_3V0;
		ret = regulator_set_voltage(host->pbias, uvoltage, uvoltage);
		if (ret) {
			dev_err(host->dev, "pbias set voltage failed\n");
			return ret;
		}

		if (host->pbias_enabled == 0) {
			ret = regulator_enable(host->pbias);
			if (ret) {
				dev_err(host->dev, "pbias reg enable fail\n");
				return ret;
			}
			host->pbias_enabled = 1;
		}
	} else {
		if (host->pbias_enabled == 1) {
			ret = regulator_disable(host->pbias);
			if (ret) {
				dev_err(host->dev, "pbias reg disable fail\n");
				return ret;
			}
			host->pbias_enabled = 0;
		}
	}

	return 0;
}

static int omap_hsmmc_set_power(struct device *dev, int power_on, int iov)
{
	struct omap_hsmmc_host *host =
		platform_get_drvdata(to_platform_device(dev));
	struct mmc_host *mmc = host->mmc;
	int ret = 0;

	if (mmc_pdata(host)->set_power)
		return mmc_pdata(host)->set_power(dev, power_on, iov);

	if (mmc_pdata(host)->before_set_reg)
		mmc_pdata(host)->before_set_reg(dev, power_on, iov);

	ret = omap_hsmmc_set_pbias(host, false, 0);
	if (ret)
		return ret;

	/*
	 * Assume Vcc regulator is used only to power the card ... OMAP
	 * VDDS is used to power the pins, optionally with a transceiver to
	 * support cards using voltages other than VDDS (1.8V nominal).  When a
	 * transceiver is used, DAT3..7 are muxed as transceiver control pins.
	 *
	 * In some cases this regulator won't support enable/disable;
	 * e.g. it's a fixed rail for a WLAN chip.
	 *
	 * In other cases vcc_aux switches interface power.  Example, for
	 * eMMC cards it represents VccQ.  Sometimes transceivers or SDIO
	 * chips/cards need an interface voltage rail too.
	 */
	if (power_on) {
		ret = omap_hsmmc_enable_supply(mmc, iov);
		if (ret)
			return ret;

		ret = omap_hsmmc_set_pbias(host, true, iov);
		if (ret)
			goto err_set_voltage;
	} else {
		ret = omap_hsmmc_disable_supply(mmc);
		if (ret)
			return ret;
	}

	if (mmc_pdata(host)->after_set_reg)
		mmc_pdata(host)->after_set_reg(dev, power_on, iov);

	return 0;

err_set_voltage:
	omap_hsmmc_disable_supply(mmc);

	return ret;
}

static int omap_hsmmc_disable_boot_regulator(struct regulator *reg)
{
	int ret;

	if (!reg)
		return 0;

	if (regulator_is_enabled(reg)) {
		ret = regulator_enable(reg);
		if (ret)
			return ret;

		ret = regulator_disable(reg);
		if (ret)
			return ret;
	}

	return 0;
}

static int omap_hsmmc_disable_boot_regulators(struct omap_hsmmc_host *host)
{
	struct mmc_host *mmc = host->mmc;
	int ret;

	/*
	 * disable regulators enabled during boot and get the usecount
	 * right so that regulators can be enabled/disabled by checking
	 * the return value of regulator_is_enabled
	 */
	ret = omap_hsmmc_disable_boot_regulator(mmc->supply.vmmc);
	if (ret) {
		dev_err(host->dev, "fail to disable boot enabled vmmc reg\n");
		return ret;
	}

	ret = omap_hsmmc_disable_boot_regulator(mmc->supply.vqmmc);
	if (ret) {
		dev_err(host->dev,
			"fail to disable boot enabled vmmc_aux reg\n");
		return ret;
	}

	ret = omap_hsmmc_disable_boot_regulator(host->pbias);
	if (ret) {
		dev_err(host->dev,
			"failed to disable boot enabled pbias reg\n");
		return ret;
	}

	return 0;
}

static int omap_hsmmc_reg_get(struct omap_hsmmc_host *host)
{
	int ocr_value = 0;
	int ret;
	struct mmc_host *mmc = host->mmc;

	if (mmc_pdata(host)->set_power)
		return 0;

	mmc->supply.vmmc = devm_regulator_get_optional(host->dev, "vmmc");
	if (IS_ERR(mmc->supply.vmmc)) {
		ret = PTR_ERR(mmc->supply.vmmc);
		if ((ret != -ENODEV) && host->dev->of_node)
			return ret;
		dev_dbg(host->dev, "unable to get vmmc regulator %ld\n",
			PTR_ERR(mmc->supply.vmmc));
		mmc->supply.vmmc = NULL;
	} else {
		ocr_value = mmc_regulator_get_ocrmask(mmc->supply.vmmc);
		if (ocr_value > 0)
			mmc_pdata(host)->ocr_mask = ocr_value;
	}

	/* Allow an aux regulator */
	mmc->supply.vqmmc = devm_regulator_get_optional(host->dev, "vmmc_aux");
	if (IS_ERR(mmc->supply.vqmmc)) {
		ret = PTR_ERR(mmc->supply.vqmmc);
		if ((ret != -ENODEV) && host->dev->of_node)
			return ret;
		dev_dbg(host->dev, "unable to get vmmc_aux regulator %ld\n",
			PTR_ERR(mmc->supply.vqmmc));
		mmc->supply.vqmmc = NULL;
	}

	host->pbias = devm_regulator_get_optional(host->dev, "pbias");
	if (IS_ERR(host->pbias)) {
		ret = PTR_ERR(host->pbias);
		if ((ret != -ENODEV) && host->dev->of_node) {
			dev_err(host->dev,
			"SD card detect fail? enable CONFIG_REGULATOR_PBIAS\n");
			return ret;
		}
		dev_dbg(host->dev, "unable to get pbias regulator %ld\n",
			PTR_ERR(host->pbias));
		host->pbias = NULL;
	}

	/* For eMMC do not power off when not in sleep state */
	if (mmc_pdata(host)->no_regulator_off_init)
		return 0;

	ret = omap_hsmmc_disable_boot_regulators(host);
	if (ret)
		return ret;

	return 0;
}

static irqreturn_t omap_hsmmc_cover_irq(int irq, void *dev_id);

static int omap_hsmmc_gpio_init(struct mmc_host *mmc,
				struct omap_hsmmc_host *host,
				struct omap_hsmmc_platform_data *pdata)
{
	int ret;

	if (gpio_is_valid(pdata->gpio_cod)) {
		ret = mmc_gpio_request_cd(mmc, pdata->gpio_cod, 0);
		if (ret)
			return ret;

		host->get_cover_state = omap_hsmmc_get_cover_state;
		mmc_gpio_set_cd_isr(mmc, omap_hsmmc_cover_irq);
	} else if (gpio_is_valid(pdata->gpio_cd)) {
		ret = mmc_gpio_request_cd(mmc, pdata->gpio_cd, 0);
		if (ret)
			return ret;

		host->card_detect = omap_hsmmc_card_detect;
	}

	if (gpio_is_valid(pdata->gpio_wp)) {
		ret = mmc_gpio_request_ro(mmc, pdata->gpio_wp);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Start clock to the card
 */
static void omap_hsmmc_start_clock(struct omap_hsmmc_host *host)
{
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) | CEN);
}

/*
 * Stop clock to the card
 */
static void omap_hsmmc_stop_clock(struct omap_hsmmc_host *host)
{
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) & ~CEN);
	if ((OMAP_HSMMC_READ(host->base, SYSCTL) & CEN) != 0x0)
		dev_dbg(mmc_dev(host->mmc), "MMC Clock is not stopped\n");
}

static void omap_hsmmc_enable_irq(struct omap_hsmmc_host *host,
				  struct mmc_command *cmd)
{
	u32 irq_mask = INT_EN_MASK;
	unsigned long flags;
	bool is_tuning;

	is_tuning = cmd && ((cmd->opcode == MMC_SEND_TUNING_BLOCK) ||
		    (cmd->opcode == MMC_SEND_TUNING_BLOCK_HS200));

	if (is_tuning)
		/*
		 * OMAP5/DRA74X/DRA72x Errata i802:
		 * DCRC error interrupts (MMCHS_STAT[21] DCRC=0x1) can occur
		 * during the tuning procedure. So disable it during the
		 * tuning procedure.
		 */
		irq_mask &= ~DCRC_EN;

	/* BRR and BWR need not be enabled for DMA */
	irq_mask &= ~(BRR_EN | BWR_EN);

	/* Disable timeout for erases or when using software timeout */
	if (cmd && (cmd->opcode == MMC_ERASE || host->data_timeout))
		irq_mask &= ~DTO_EN;

	if (host->flags & CLKEXTFREE_ENABLED)
		irq_mask |= CIRQ_EN;

	spin_lock_irqsave(&host->irq_lock, flags);
	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
	OMAP_HSMMC_WRITE(host->base, ISE, irq_mask);

	/* latch pending CIRQ, but don't signal MMC core */
	if (host->flags & HSMMC_SDIO_IRQ_ENABLED)
		irq_mask |= CIRQ_EN;
	OMAP_HSMMC_WRITE(host->base, IE, irq_mask);
	spin_unlock_irqrestore(&host->irq_lock, flags);
}

static void omap_hsmmc_disable_irq(struct omap_hsmmc_host *host)
{
	u32 irq_mask = 0;
	unsigned long flags;

	spin_lock_irqsave(&host->irq_lock, flags);
	/* no transfer running but need to keep cirq if enabled */
	if (host->flags & HSMMC_SDIO_IRQ_ENABLED)
		irq_mask |= CIRQ_EN;
	OMAP_HSMMC_WRITE(host->base, ISE, irq_mask);
	OMAP_HSMMC_WRITE(host->base, IE, irq_mask);
	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
	spin_unlock_irqrestore(&host->irq_lock, flags);
}

/* Calculate divisor for the given clock frequency */
static u16 calc_divisor(struct omap_hsmmc_host *host, struct mmc_ios *ios)
{
	u16 dsor = 0;

	if (ios->clock) {
		dsor = DIV_ROUND_UP(clk_get_rate(host->fclk), ios->clock);
		if (dsor > CLKD_MAX)
			dsor = CLKD_MAX;
	}

	return dsor;
}

static void omap_hsmmc_set_clock(struct omap_hsmmc_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;
	unsigned long regval;
	unsigned long timeout;
	unsigned long clkdiv;

	dev_vdbg(mmc_dev(host->mmc), "Set clock to %uHz\n", ios->clock);

	omap_hsmmc_stop_clock(host);

	regval = OMAP_HSMMC_READ(host->base, SYSCTL);
	regval = regval & ~(CLKD_MASK | DTO_MASK);
	clkdiv = calc_divisor(host, ios);
	regval = regval | (clkdiv << 6) | (DTO << 16);
	OMAP_HSMMC_WRITE(host->base, SYSCTL, regval);
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) | ICE);

	/* Wait till the ICS bit is set */
	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & ICS) != ICS
		&& time_before(jiffies, timeout))
		cpu_relax();

	/*
	 * Enable High-Speed Support
	 * Pre-Requisites
	 *	- Controller should support High-Speed-Enable Bit
	 *	- Controller should not be using DDR Mode
	 *	- Controller should advertise that it supports High Speed
	 *	  in capabilities register
	 *	- MMC/SD clock coming out of controller > 25MHz
	 */
	if ((mmc_pdata(host)->features & HSMMC_HAS_HSPE_SUPPORT) &&
	    (ios->timing != MMC_TIMING_MMC_DDR52) &&
	    (ios->timing != MMC_TIMING_UHS_DDR50) &&
	    ((OMAP_HSMMC_READ(host->base, CAPA) & HSS) == HSS)) {
		regval = OMAP_HSMMC_READ(host->base, HCTL);
		if (clkdiv && (clk_get_rate(host->fclk)/clkdiv) > 25000000)
			regval |= HSPE;
		else
			regval &= ~HSPE;

		OMAP_HSMMC_WRITE(host->base, HCTL, regval);
	}

	omap_hsmmc_start_clock(host);
}

static void omap_hsmmc_set_bus_width(struct omap_hsmmc_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;
	u32 con;

	con = OMAP_HSMMC_READ(host->base, CON);
	if (ios->timing == MMC_TIMING_MMC_DDR52 ||
	    ios->timing == MMC_TIMING_UHS_DDR50)
		con |= DDR;	/* configure in DDR mode */
	else
		con &= ~DDR;
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_8:
		OMAP_HSMMC_WRITE(host->base, CON, con | DW8);
		break;
	case MMC_BUS_WIDTH_4:
		OMAP_HSMMC_WRITE(host->base, CON, con & ~DW8);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | FOUR_BIT);
		break;
	case MMC_BUS_WIDTH_1:
		OMAP_HSMMC_WRITE(host->base, CON, con & ~DW8);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) & ~FOUR_BIT);
		break;
	}
}

static void omap_hsmmc_set_bus_mode(struct omap_hsmmc_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;
	u32 con;

	con = OMAP_HSMMC_READ(host->base, CON);
	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
		OMAP_HSMMC_WRITE(host->base, CON, con | OD);
	else
		OMAP_HSMMC_WRITE(host->base, CON, con & ~OD);
}

#ifdef CONFIG_PM

/*
 * Restore the MMC host context, if it was lost as result of a
 * power state change.
 */
static int omap_hsmmc_context_restore(struct omap_hsmmc_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;
	u32 hctl, capa;
	unsigned long timeout;

	if (host->con == OMAP_HSMMC_READ(host->base, CON) &&
	    host->hctl == OMAP_HSMMC_READ(host->base, HCTL) &&
	    host->sysctl == OMAP_HSMMC_READ(host->base, SYSCTL) &&
	    host->capa == OMAP_HSMMC_READ(host->base, CAPA))
		return 0;

	host->context_loss++;

	if (host->pdata->controller_flags & OMAP_HSMMC_SUPPORTS_DUAL_VOLT) {
		if (host->power_mode != MMC_POWER_OFF &&
		    (1 << ios->vdd) <= MMC_VDD_23_24)
			hctl = SDVS18;
		else
			hctl = SDVS30;
		capa = VS30 | VS18;
	} else {
		hctl = SDVS18;
		capa = VS18;
	}

	if (host->mmc->caps & MMC_CAP_SDIO_IRQ)
		hctl |= IWE;

	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | hctl);

	OMAP_HSMMC_WRITE(host->base, CAPA,
			OMAP_HSMMC_READ(host->base, CAPA) | capa);

	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | SDBP);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((OMAP_HSMMC_READ(host->base, HCTL) & SDBP) != SDBP
		&& time_before(jiffies, timeout))
		;

	OMAP_HSMMC_WRITE(host->base, ISE, 0);
	OMAP_HSMMC_WRITE(host->base, IE, 0);
	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);

	/* Do not initialize card-specific things if the power is off */
	if (host->power_mode == MMC_POWER_OFF)
		goto out;

	omap_hsmmc_set_bus_width(host);

	omap_hsmmc_set_clock(host);

	omap_hsmmc_set_bus_mode(host);

out:
	dev_dbg(mmc_dev(host->mmc), "context is restored: restore count %d\n",
		host->context_loss);
	return 0;
}

/*
 * Save the MMC host context (store the number of power state changes so far).
 */
static void omap_hsmmc_context_save(struct omap_hsmmc_host *host)
{
	host->con =  OMAP_HSMMC_READ(host->base, CON);
	host->hctl = OMAP_HSMMC_READ(host->base, HCTL);
	host->sysctl =  OMAP_HSMMC_READ(host->base, SYSCTL);
	host->capa = OMAP_HSMMC_READ(host->base, CAPA);
}

#else

static int omap_hsmmc_context_restore(struct omap_hsmmc_host *host)
{
	return 0;
}

static void omap_hsmmc_context_save(struct omap_hsmmc_host *host)
{
}

#endif

/*
 * Send init stream sequence to card
 * before sending IDLE command
 */
static void send_init_stream(struct omap_hsmmc_host *host)
{
	int reg = 0;
	unsigned long timeout;

	if (host->protect_card)
		return;

	disable_irq(host->irq);

	OMAP_HSMMC_WRITE(host->base, IE, INT_EN_MASK);
	OMAP_HSMMC_WRITE(host->base, CON,
		OMAP_HSMMC_READ(host->base, CON) | INIT_STREAM);
	OMAP_HSMMC_WRITE(host->base, CMD, INIT_STREAM_CMD);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((reg != CC_EN) && time_before(jiffies, timeout))
		reg = OMAP_HSMMC_READ(host->base, STAT) & CC_EN;

	OMAP_HSMMC_WRITE(host->base, CON,
		OMAP_HSMMC_READ(host->base, CON) & ~INIT_STREAM);

	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
	OMAP_HSMMC_READ(host->base, STAT);

	enable_irq(host->irq);
}

static inline
int omap_hsmmc_cover_is_closed(struct omap_hsmmc_host *host)
{
	int r = 1;

	if (host->get_cover_state)
		r = host->get_cover_state(host->dev);
	return r;
}

static ssize_t
omap_hsmmc_show_cover_switch(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mmc_host *mmc = container_of(dev, struct mmc_host, class_dev);
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	return sprintf(buf, "%s\n",
			omap_hsmmc_cover_is_closed(host) ? "closed" : "open");
}

static DEVICE_ATTR(cover_switch, S_IRUGO, omap_hsmmc_show_cover_switch, NULL);

static ssize_t
omap_hsmmc_show_slot_name(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct mmc_host *mmc = container_of(dev, struct mmc_host, class_dev);
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	return sprintf(buf, "%s\n", mmc_pdata(host)->name);
}

static DEVICE_ATTR(slot_name, S_IRUGO, omap_hsmmc_show_slot_name, NULL);

/*
 * Configure the response type and send the cmd.
 */
static void
omap_hsmmc_start_command(struct omap_hsmmc_host *host, struct mmc_command *cmd,
	struct mmc_data *data)
{
	int cmdreg = 0, resptype = 0, cmdtype = 0;

	dev_vdbg(mmc_dev(host->mmc), "%s: CMD%d, argument 0x%08x\n",
		mmc_hostname(host->mmc), cmd->opcode, cmd->arg);
	host->cmd = cmd;

	omap_hsmmc_enable_irq(host, cmd);

	host->response_busy = 0;
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			resptype = 1;
		else if (cmd->flags & MMC_RSP_BUSY) {
			resptype = 3;
			host->response_busy = 1;
		} else
			resptype = 2;
	}

	/*
	 * Unlike OMAP1 controller, the cmdtype does not seem to be based on
	 * ac, bc, adtc, bcr. Only commands ending an open ended transfer need
	 * a val of 0x3, rest 0x0.
	 */
	if (cmd == host->mrq->stop)
		cmdtype = 0x3;

	cmdreg = (cmd->opcode << 24) | (resptype << 16) | (cmdtype << 22);

	if ((host->flags & AUTO_CMD23) && mmc_op_multi(cmd->opcode) &&
	    host->mrq->sbc) {
		cmdreg |= ACEN_ACMD23;
		OMAP_HSMMC_WRITE(host->base, SDMASA, host->mrq->sbc->arg);
	}
	if (data) {
		cmdreg |= DP_SELECT | MSBS | BCE;
		if (data->flags & MMC_DATA_READ)
			cmdreg |= DDIR;
		else
			cmdreg &= ~(DDIR);
	}

	/* Tuning command is special. Data Present Select should be set */
	if ((cmd->opcode == MMC_SEND_TUNING_BLOCK) ||
	    (cmd->opcode == MMC_SEND_TUNING_BLOCK_HS200))
		cmdreg |= DP_SELECT | DDIR;

	cmdreg |= DMAE;

	host->req_in_progress = 1;
	host->last_cmd = cmd->opcode;

	OMAP_HSMMC_WRITE(host->base, ARG, cmd->arg);
	OMAP_HSMMC_WRITE(host->base, CMD, cmdreg);
}

static int
omap_hsmmc_get_dma_dir(struct omap_hsmmc_host *host, struct mmc_data *data)
{
	if (data->flags & MMC_DATA_WRITE)
		return DMA_TO_DEVICE;
	else
		return DMA_FROM_DEVICE;
}

static struct dma_chan *omap_hsmmc_get_dma_chan(struct omap_hsmmc_host *host,
	struct mmc_data *data)
{
	return data->flags & MMC_DATA_WRITE ? host->tx_chan : host->rx_chan;
}

static void omap_hsmmc_request_done(struct omap_hsmmc_host *host, struct mmc_request *mrq)
{
	int dma_ch;
	unsigned long flags;

	spin_lock_irqsave(&host->irq_lock, flags);
	host->req_in_progress = 0;
	dma_ch = host->dma_ch;
	spin_unlock_irqrestore(&host->irq_lock, flags);

	omap_hsmmc_disable_irq(host);
	/* Do not complete the request if DMA is still in progress */
	if (mrq->data && dma_ch != -1)
		return;
	host->mrq = NULL;
	mmc_request_done(host->mmc, mrq);
	pm_runtime_mark_last_busy(host->dev);
	pm_runtime_put_autosuspend(host->dev);
}

/*
 * Notify the transfer complete to MMC core
 */
static void
omap_hsmmc_xfer_done(struct omap_hsmmc_host *host, struct mmc_data *data)
{
	if (!data) {
		struct mmc_request *mrq = host->mrq;

		/* TC before CC from CMD6 - don't know why, but it happens */
		if (host->cmd && host->cmd->opcode == 6 &&
		    host->response_busy) {
			host->response_busy = 0;
			return;
		}

		omap_hsmmc_request_done(host, mrq);
		return;
	}

	host->data = NULL;

	if (!data->error)
		data->bytes_xfered += data->blocks * (data->blksz);
	else
		data->bytes_xfered = 0;

	if (data->stop && (data->error || !host->mrq->sbc))
		omap_hsmmc_start_command(host, data->stop, NULL);
	else
		omap_hsmmc_request_done(host, data->mrq);
}

/*
 * Notify the core about command completion
 */
static void
omap_hsmmc_cmd_done(struct omap_hsmmc_host *host, struct mmc_command *cmd)
{
	if (host->mrq->sbc && (host->cmd == host->mrq->sbc) &&
	    !host->mrq->sbc->error && !(host->flags & AUTO_CMD23)) {
		host->cmd = NULL;
		omap_hsmmc_start_dma_transfer(host);
		omap_hsmmc_start_command(host, host->mrq->cmd,
						host->mrq->data);
		return;
	}

	host->cmd = NULL;

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			/* response type 2 */
			cmd->resp[3] = OMAP_HSMMC_READ(host->base, RSP10);
			cmd->resp[2] = OMAP_HSMMC_READ(host->base, RSP32);
			cmd->resp[1] = OMAP_HSMMC_READ(host->base, RSP54);
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP76);
		} else {
			/* response types 1, 1b, 3, 4, 5, 6 */
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP10);
		}
	}
	if ((host->data == NULL && !host->response_busy) || cmd->error)
		omap_hsmmc_request_done(host, host->mrq);
}

/*
 * DMA clean up for command errors
 */
static void omap_hsmmc_dma_cleanup(struct omap_hsmmc_host *host, int errno)
{
	int dma_ch;
	unsigned long flags;

	host->data->error = errno;

	spin_lock_irqsave(&host->irq_lock, flags);
	dma_ch = host->dma_ch;
	host->dma_ch = -1;
	spin_unlock_irqrestore(&host->irq_lock, flags);

	if (host->use_adma) {
		dma_unmap_sg(host->dev, host->data->sg, host->data->sg_len,
			     omap_hsmmc_get_dma_dir(host, host->data));
		host->data->host_cookie = 0;
	} else if (dma_ch != -1) {
		struct dma_chan *chan = omap_hsmmc_get_dma_chan(host,
								host->data);
		dmaengine_terminate_all(chan);
		dma_unmap_sg(chan->device->dev,
			     host->data->sg, host->data->sg_len,
		omap_hsmmc_get_dma_dir(host, host->data));

		host->data->host_cookie = 0;
	}
	host->data = NULL;
}

/*
 * Readable error output
 */
#ifdef CONFIG_MMC_DEBUG
static void omap_hsmmc_dbg_report_irq(struct omap_hsmmc_host *host, u32 status)
{
	/* --- means reserved bit without definition at documentation */
	static const char *omap_hsmmc_status_bits[] = {
		"CC"  , "TC"  , "BGE", "---", "BWR" , "BRR" , "---" , "---" ,
		"CIRQ",	"OBI" , "---", "---", "---" , "---" , "---" , "ERRI",
		"CTO" , "CCRC", "CEB", "CIE", "DTO" , "DCRC", "DEB" , "---" ,
		"ACE" , "---" , "---", "---", "CERR", "BADA", "---" , "---"
	};
	char res[256];
	char *buf = res;
	int len, i;

	len = sprintf(buf, "MMC IRQ 0x%x :", status);
	buf += len;

	for (i = 0; i < ARRAY_SIZE(omap_hsmmc_status_bits); i++)
		if (status & (1 << i)) {
			len = sprintf(buf, " %s", omap_hsmmc_status_bits[i]);
			buf += len;
		}

	dev_vdbg(mmc_dev(host->mmc), "%s\n", res);
}
#else
static inline void omap_hsmmc_dbg_report_irq(struct omap_hsmmc_host *host,
					     u32 status)
{
}
#endif  /* CONFIG_MMC_DEBUG */

/*
 * MMC controller internal state machines reset
 *
 * Used to reset command or data internal state machines, using respectively
 *  SRC or SRD bit of SYSCTL register
 * Can be called from interrupt context
 */
static inline void omap_hsmmc_reset_controller_fsm(struct omap_hsmmc_host *host,
						   unsigned long bit)
{
	unsigned long i = 0;
	unsigned long limit = MMC_TIMEOUT_US;

	OMAP_HSMMC_WRITE(host->base, SYSCTL,
			 OMAP_HSMMC_READ(host->base, SYSCTL) | bit);

	/*
	 * OMAP4 ES2 and greater has an updated reset logic.
	 * Monitor a 0->1 transition first
	 */
	if (mmc_pdata(host)->features & HSMMC_HAS_UPDATED_RESET) {
		while ((!(OMAP_HSMMC_READ(host->base, SYSCTL) & bit))
					&& (i++ < limit))
			udelay(1);
	}
	i = 0;

	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & bit) &&
		(i++ < limit))
		udelay(1);

	if (OMAP_HSMMC_READ(host->base, SYSCTL) & bit)
		dev_err(mmc_dev(host->mmc),
			"Timeout waiting on controller reset in %s\n",
			__func__);
}

static void hsmmc_command_incomplete(struct omap_hsmmc_host *host,
					int err, int end_cmd)
{
	if (end_cmd) {
		omap_hsmmc_reset_controller_fsm(host, SRC);
		if (host->cmd)
			host->cmd->error = err;
	}

	if (host->data) {
		omap_hsmmc_reset_controller_fsm(host, SRD);
		omap_hsmmc_dma_cleanup(host, err);
	} else if (host->mrq && host->mrq->cmd)
		host->mrq->cmd->error = err;
}

static void omap_hsmmc_adma_err(struct omap_hsmmc_host *host)
{
	u32 admaes, admasal;

	admaes = OMAP_HSMMC_READ(host->base, ADMAES);
	admasal = OMAP_HSMMC_READ(host->base, ADMASAL);

	switch (admaes & AES_MASK) {
	case ST_FDS:
		dev_err(mmc_dev(host->mmc),
			"ADMA err: ST_FDS, erroneous desc at 0x%08x\n",
			admasal);
		break;
	case ST_STOP:
		dev_err(mmc_dev(host->mmc),
			"ADMA err: ST_STOP, desc at 0x%08x follows the erroneous one\n",
			admasal);
		break;
	case ST_TFR:
		dev_err(mmc_dev(host->mmc),
			"ADMA err: ST_TFR, desc at 0x%08x follows the erroneous one\n",
			admasal);
		break;
	default:
		dev_warn(mmc_dev(host->mmc), "Unexpected ADMA error state\n");
		break;
	}
}

static void omap_hsmmc_do_irq(struct omap_hsmmc_host *host, int status)
{
	struct mmc_data *data;
	int end_cmd = 0, end_trans = 0;
	int error = 0;

	data = host->data;
	dev_vdbg(mmc_dev(host->mmc), "IRQ Status is %x\n", status);

	if (status & ERR_EN) {
		omap_hsmmc_dbg_report_irq(host, status);

		if (status & (CTO_EN | CCRC_EN))
			end_cmd = 1;
		if (host->data || host->response_busy) {
			end_trans = !end_cmd;
			host->response_busy = 0;
		}
		if (status & (CTO_EN | DTO_EN))
			hsmmc_command_incomplete(host, -ETIMEDOUT, end_cmd);
		else if (status & (CCRC_EN | DCRC_EN | DEB_EN | CEB_EN |
				   BADA_EN))
			hsmmc_command_incomplete(host, -EILSEQ, end_cmd);

		if (status & ACE_EN) {
			u32 ac12;
			ac12 = OMAP_HSMMC_READ(host->base, AC12);
			if (!(ac12 & ACNE) && host->mrq->sbc) {
				end_cmd = 1;
				if (ac12 & ACTO)
					error =  -ETIMEDOUT;
				else if (ac12 & (ACCE | ACEB | ACIE))
					error = -EILSEQ;
				host->mrq->sbc->error = error;
				hsmmc_command_incomplete(host, error, end_cmd);
			}
			dev_dbg(mmc_dev(host->mmc), "AC12 err: 0x%x\n", ac12);
		}

		if (status & ADMAE_EN) {
			omap_hsmmc_adma_err(host);
			end_trans = 1;
			data->error = -EIO;
		}
	}

	OMAP_HSMMC_WRITE(host->base, STAT, status);
	if (end_cmd || ((status & CC_EN) && host->cmd)) {
		omap_hsmmc_cmd_done(host, host->cmd);
		if (host->data_timeout) {
			unsigned long timeout;

			timeout = jiffies +
				  nsecs_to_jiffies(host->data_timeout);
			mod_timer(&host->timer, timeout);
		}
	}
	if ((end_trans || (status & TC_EN)) && host->mrq)
		omap_hsmmc_xfer_done(host, data);
}

/*
 * MMC controller IRQ handler
 */
static irqreturn_t omap_hsmmc_irq(int irq, void *dev_id)
{
	struct omap_hsmmc_host *host = dev_id;
	int status;

	status = OMAP_HSMMC_READ(host->base, STAT);

	while (status & (INT_EN_MASK | CIRQ_EN)) {
		/*
		 * During a successful bulk data transfer command-completion
		 * interrupt and transfer-completion interrupt will be
		 * generated, but software-timeout timer should be deleted
		 * only on non-cc interrupts (transfer complete or error)
		 */
		if (host->data_timeout && (status & (~CC_EN))) {
			del_timer(&host->timer);
			host->data_timeout = 0;
		}

		if (host->req_in_progress)
			omap_hsmmc_do_irq(host, status);

		if (status & CIRQ_EN)
			mmc_signal_sdio_irq(host->mmc);

		/* Flush posted write */
		status = OMAP_HSMMC_READ(host->base, STAT);
	}

	return IRQ_HANDLED;
}

static void omap_hsmmc_soft_timeout(unsigned long data)
{
	struct omap_hsmmc_host *host = (struct omap_hsmmc_host *)data;
	bool end_trans;

	omap_hsmmc_disable_irq(host);
	if (host->data || host->response_busy) {
		host->response_busy = 0;
		end_trans = 1;
	}

	hsmmc_command_incomplete(host, -ETIMEDOUT, 0);
	if (end_trans && host->mrq)
		omap_hsmmc_xfer_done(host, host->data);
	else if (host->cmd)
		omap_hsmmc_cmd_done(host, host->cmd);
	host->data_timeout = 0;
}

static void set_sd_bus_power(struct omap_hsmmc_host *host)
{
	unsigned long i;

	OMAP_HSMMC_WRITE(host->base, HCTL,
			 OMAP_HSMMC_READ(host->base, HCTL) | SDBP);
	for (i = 0; i < loops_per_jiffy; i++) {
		if (OMAP_HSMMC_READ(host->base, HCTL) & SDBP)
			break;
		cpu_relax();
	}
}

/* Protect the card while the cover is open */
static void omap_hsmmc_protect_card(struct omap_hsmmc_host *host)
{
	if (!host->get_cover_state)
		return;

	host->reqs_blocked = 0;
	if (host->get_cover_state(host->dev)) {
		if (host->protect_card) {
			dev_info(host->dev, "%s: cover is closed, "
					 "card is now accessible\n",
					 mmc_hostname(host->mmc));
			host->protect_card = 0;
		}
	} else {
		if (!host->protect_card) {
			dev_info(host->dev, "%s: cover is open, "
					 "card is now inaccessible\n",
					 mmc_hostname(host->mmc));
			host->protect_card = 1;
		}
	}
}

/*
 * irq handler when (cell-phone) cover is mounted/removed
 */
static irqreturn_t omap_hsmmc_cover_irq(int irq, void *dev_id)
{
	struct omap_hsmmc_host *host = dev_id;

	sysfs_notify(&host->mmc->class_dev.kobj, NULL, "cover_switch");

	omap_hsmmc_protect_card(host);
	mmc_detect_change(host->mmc, (HZ * 200) / 1000);
	return IRQ_HANDLED;
}

static void omap_hsmmc_dma_callback(void *param)
{
	struct omap_hsmmc_host *host = param;
	struct dma_chan *chan;
	struct mmc_data *data;
	int req_in_progress;

	spin_lock_irq(&host->irq_lock);
	if (host->dma_ch < 0) {
		spin_unlock_irq(&host->irq_lock);
		return;
	}

	data = host->mrq->data;
	chan = omap_hsmmc_get_dma_chan(host, data);
	if (!data->host_cookie)
		dma_unmap_sg(chan->device->dev,
			     data->sg, data->sg_len,
			     omap_hsmmc_get_dma_dir(host, data));

	req_in_progress = host->req_in_progress;
	host->dma_ch = -1;
	spin_unlock_irq(&host->irq_lock);

	/* If DMA has finished after TC, complete the request */
	if (!req_in_progress) {
		struct mmc_request *mrq = host->mrq;

		host->mrq = NULL;
		mmc_request_done(host->mmc, mrq);
		pm_runtime_mark_last_busy(host->dev);
		pm_runtime_put_autosuspend(host->dev);
	}
}

static int omap_hsmmc_pre_dma_transfer(struct omap_hsmmc_host *host,
				       struct mmc_data *data,
				       struct omap_hsmmc_next *next,
				       struct dma_chan *chan)
{
	int dma_len;
	struct device *dev;

	if (!next && data->host_cookie &&
	    data->host_cookie != host->next_data.cookie) {
		dev_warn(host->dev, "[%s] invalid cookie: data->host_cookie %d"
		       " host->next_data.cookie %d\n",
		       __func__, data->host_cookie, host->next_data.cookie);
		data->host_cookie = 0;
	}

	if (chan)
		dev = chan->device->dev;
	else
		dev = mmc_dev(host->mmc);

	/* Check if next job is already prepared */
	if (next || data->host_cookie != host->next_data.cookie) {
		dma_len = dma_map_sg(dev, data->sg, data->sg_len,
				     omap_hsmmc_get_dma_dir(host, data));

	} else {
		dma_len = host->next_data.dma_len;
		host->next_data.dma_len = 0;
	}


	if (dma_len == 0)
		return -EINVAL;

	if (next) {
		next->dma_len = dma_len;
		data->host_cookie = ++next->cookie < 0 ? 1 : next->cookie;
	} else
		host->dma_len = dma_len;

	return 0;
}

/*
 * Routine to configure and start DMA for the MMC card
 */
static int omap_hsmmc_setup_dma_transfer(struct omap_hsmmc_host *host,
					struct mmc_request *req)
{
	struct dma_slave_config cfg;
	struct dma_async_tx_descriptor *tx;
	int ret = 0, i;
	struct mmc_data *data = req->data;
	struct dma_chan *chan;

	/* Sanity check: all the SG entries must be aligned by block size. */
	for (i = 0; i < data->sg_len; i++) {
		struct scatterlist *sgl;

		sgl = data->sg + i;
		if (sgl->length % data->blksz)
			return -EINVAL;
	}
	if ((data->blksz % 4) != 0)
		/* REVISIT: The MMC buffer increments only when MSB is written.
		 * Return error for blksz which is non multiple of four.
		 */
		return -EINVAL;

	BUG_ON(host->dma_ch != -1);

	chan = omap_hsmmc_get_dma_chan(host, data);

	cfg.src_addr = host->mapbase + OMAP_HSMMC_DATA;
	cfg.dst_addr = host->mapbase + OMAP_HSMMC_DATA;
	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.src_maxburst = data->blksz / 4;
	cfg.dst_maxburst = data->blksz / 4;

	ret = dmaengine_slave_config(chan, &cfg);
	if (ret)
		return ret;

	ret = omap_hsmmc_pre_dma_transfer(host, data, NULL, chan);
	if (ret)
		return ret;

	tx = dmaengine_prep_slave_sg(chan, data->sg, data->sg_len,
		data->flags & MMC_DATA_WRITE ? DMA_MEM_TO_DEV : DMA_DEV_TO_MEM,
		DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx) {
		dev_err(mmc_dev(host->mmc), "prep_slave_sg() failed\n");
		/* FIXME: cleanup */
		return -1;
	}

	tx->callback = omap_hsmmc_dma_callback;
	tx->callback_param = host;

	/* Does not fail */
	dmaengine_submit(tx);

	host->dma_ch = 1;

	return 0;
}

static void set_data_timeout(struct omap_hsmmc_host *host,
			     unsigned int timeout_ns,
			     unsigned int timeout_clks)
{
	unsigned int timeout, cycle_ns;
	uint32_t reg, clkd, dto = 0;
	struct mmc_ios *ios = &host->mmc->ios;
	struct mmc_data *data = host->mrq->data;

	reg = OMAP_HSMMC_READ(host->base, SYSCTL);
	clkd = (reg & CLKD_MASK) >> CLKD_SHIFT;
	if (clkd == 0)
		clkd = 1;

	cycle_ns = 1000000000 / (host->clk_rate / clkd);
	timeout = timeout_ns / cycle_ns;
	timeout += timeout_clks;
	if (timeout) {
		while ((timeout & 0x80000000) == 0) {
			dto += 1;
			timeout <<= 1;
		}
		dto = 31 - dto;
		timeout <<= 1;
		if (timeout && dto)
			dto += 1;
		if (dto >= 13)
			dto -= 13;
		else
			dto = 0;
		if (dto > 14) {
			/*
			 * DRA7 Errata No i834: When using high speed HS200 and
			 * SDR104 cards, the functional clock for MMC module
			 * will be 192MHz. At this frequency, the maximum
			 * obtainable timeout (DTO =0xE) in hardware is
			 * (1/192MHz)*2^27 = 700ms. Commands taking longer than
			 * 700ms will be affected by this small window frame
			 * and will be timing out frequently even without a
			 * genuine timeout from the card. Workaround for
			 * this errata is use a software timer instead of
			 * hardware timer to provide the timeout requested
			 * by the upper layer.
			 *
			 * The timeout from the upper layer denotes the delay
			 * between the end bit of the read command and the
			 * start bit of the data block and in the case of
			 * multiple-read operation, they also define the
			 * typical delay between the end bit of a data
			 * block and the start bit of next data block.
			 *
			 * Calculate the total timeout value for the entire
			 * transfer to complete from the timeout value given
			 * by the upper layer.
			 */
			host->data_timeout = (data->blocks * (timeout_ns +
					      MMC_BLOCK_TRANSFER_TIME_NS(
					      data->blksz, ios->bus_width,
					      ios->clock)));
		}
	}

	reg &= ~DTO_MASK;
	reg |= dto << DTO_SHIFT;
	OMAP_HSMMC_WRITE(host->base, SYSCTL, reg);
}

static void omap_hsmmc_start_dma_transfer(struct omap_hsmmc_host *host)
{
	struct mmc_request *req = host->mrq;
	struct dma_chan *chan;

	if (!req->data)
		return;
	OMAP_HSMMC_WRITE(host->base, BLK, (req->data->blksz)
				| (req->data->blocks << 16));
	set_data_timeout(host, req->data->timeout_ns,
				req->data->timeout_clks);

	if (host->use_adma) {
		OMAP_HSMMC_WRITE(host->base, ADMASAL,
				 (u32)host->adma_desc_table_addr);
	} else {
		chan = omap_hsmmc_get_dma_chan(host, req->data);
		dma_async_issue_pending(chan);
	}
}

static int omap_hsmmc_write_adma_desc(struct omap_hsmmc_host *host, void *desc,
				      dma_addr_t addr, u16 len, u8 attr)
{
	struct omap_hsmmc_adma_desc *dma_desc = desc;

	dma_desc->len = len;
	dma_desc->addr = (u32)addr;
	dma_desc->reserved = 0;
	dma_desc->attr = attr;

	return 0;
}

static int omap_hsmmc_setup_adma_transfer(struct omap_hsmmc_host *host,
					  struct mmc_request *req)
{
	struct mmc_data *data = req->data;
	struct scatterlist *sg;
	int i;
	int len;
	int ret;
	dma_addr_t addr;
	struct omap_hsmmc_adma_desc *dma_desc;

	ret = omap_hsmmc_pre_dma_transfer(host, data, NULL, NULL);
	if (ret)
		return ret;

	dma_desc = host->adma_desc_table;
	for_each_sg(data->sg, sg, host->dma_len, i) {
		addr = sg_dma_address(sg);
		len = sg_dma_len(sg);
		WARN_ON(len > ADMA_MAX_LEN);
		omap_hsmmc_write_adma_desc(host, dma_desc, addr, len,
					   ADMA_DESC_ATTR_VALID |
					   ADMA_DESC_TRANSFER_DATA);
		dma_desc++;
	}
	omap_hsmmc_write_adma_desc(host, dma_desc, 0, 0, ADMA_DESC_ATTR_END);

	return 0;
}

/*
 * Configure block length for MMC/SD cards and initiate the transfer.
 */
static int
omap_hsmmc_prepare_data(struct omap_hsmmc_host *host, struct mmc_request *req)
{
	int ret;
	unsigned int timeout;

	host->data = req->data;

	if (req->data == NULL) {
		OMAP_HSMMC_WRITE(host->base, BLK, 0);
		if (req->cmd->flags & MMC_RSP_BUSY) {
			timeout = req->cmd->busy_timeout * NSEC_PER_MSEC;

			/*
			 * Set an arbitrary 100ms data timeout for commands with
			 * busy signal and no indication of busy_timeout.
			 */
			if (!timeout)
				timeout = 100000000U;

			set_data_timeout(host, timeout, 0);
		}
		return 0;
	}

	if (host->use_adma) {
		ret = omap_hsmmc_setup_adma_transfer(host, req);
		if (ret != 0) {
			dev_err(mmc_dev(host->mmc), "MMC adma setup failed\n");
			return ret;
		}
	} else {
		ret = omap_hsmmc_setup_dma_transfer(host, req);
		if (ret != 0) {
			dev_err(mmc_dev(host->mmc), "MMC start dma failure\n");
			return ret;
		}
	}
	return 0;
}

static void omap_hsmmc_post_req(struct mmc_host *mmc, struct mmc_request *mrq,
				int err)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;
	struct device *dev;
	struct dma_chan *c;

	if (data->host_cookie) {
		if (host->use_adma) {
			dev = mmc_dev(mmc);
		} else {
			c = omap_hsmmc_get_dma_chan(host, mrq->data);
			dev = c->device->dev;
		}

		dma_unmap_sg(dev, data->sg, data->sg_len,
			     omap_hsmmc_get_dma_dir(host, data));
		data->host_cookie = 0;
	}
}

static void omap_hsmmc_pre_req(struct mmc_host *mmc, struct mmc_request *mrq,
			       bool is_first_req)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	struct dma_chan *c = NULL;

	if (mrq->data->host_cookie) {
		mrq->data->host_cookie = 0;
		return ;
	}

	if (!host->use_adma)
		c = omap_hsmmc_get_dma_chan(host, mrq->data);

	if (omap_hsmmc_pre_dma_transfer(host, mrq->data,
					&host->next_data, c))
		mrq->data->host_cookie = 0;
}

/*
 * Request function. for read/write operation
 */
static void omap_hsmmc_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	int err;

	BUG_ON(host->req_in_progress);
	BUG_ON(host->dma_ch != -1);
	pm_runtime_get_sync(host->dev);
	if (host->protect_card) {
		if (host->reqs_blocked < 3) {
			/*
			 * Ensure the controller is left in a consistent
			 * state by resetting the command and data state
			 * machines.
			 */
			omap_hsmmc_reset_controller_fsm(host, SRD);
			omap_hsmmc_reset_controller_fsm(host, SRC);
			host->reqs_blocked += 1;
		}
		req->cmd->error = -EBADF;
		if (req->data)
			req->data->error = -EBADF;
		req->cmd->retries = 0;
		mmc_request_done(mmc, req);
		pm_runtime_mark_last_busy(host->dev);
		pm_runtime_put_autosuspend(host->dev);
		return;
	} else if (host->reqs_blocked)
		host->reqs_blocked = 0;
	WARN_ON(host->mrq != NULL);
	host->mrq = req;
	host->clk_rate = clk_get_rate(host->fclk);
	err = omap_hsmmc_prepare_data(host, req);
	if (err) {
		req->cmd->error = err;
		if (req->data)
			req->data->error = err;
		host->mrq = NULL;
		mmc_request_done(mmc, req);
		pm_runtime_mark_last_busy(host->dev);
		pm_runtime_put_autosuspend(host->dev);
		return;
	}
	if (req->sbc && !(host->flags & AUTO_CMD23)) {
		omap_hsmmc_start_command(host, req->sbc, NULL);
		return;
	}

	omap_hsmmc_start_dma_transfer(host);
	omap_hsmmc_start_command(host, req->cmd, req->data);
}

static void omap_hsmmc_set_timing(struct omap_hsmmc_host *host)
{
	u32 val;
	int ret;
	struct pinctrl_state *pinctrl_state;
	struct mmc_ios *ios = &host->mmc->ios;

	omap_hsmmc_stop_clock(host);

	val = OMAP_HSMMC_READ(host->base, AC12);
	val &= ~AC12_UHSMC_MASK;
	switch (ios->timing) {
	case MMC_TIMING_UHS_SDR104:
		val |= AC12_UHSMC_SDR104;
		pinctrl_state = host->sdr104_pinctrl_state;
		break;
	case MMC_TIMING_MMC_HS200:
		val |= AC12_UHSMC_SDR104;
		pinctrl_state = host->hs200_1_8v_pinctrl_state;
		break;
	case MMC_TIMING_UHS_DDR50:
		val |= AC12_UHSMC_DDR50;
		pinctrl_state = host->ddr50_pinctrl_state;
		break;
	case MMC_TIMING_UHS_SDR50:
		val |= AC12_UHSMC_SDR50;
		pinctrl_state = host->sdr50_pinctrl_state;
		break;
	case MMC_TIMING_UHS_SDR25:
		val |= AC12_UHSMC_SDR25;
		pinctrl_state = host->sdr25_pinctrl_state;
		break;
	case MMC_TIMING_UHS_SDR12:
		val |= AC12_UHSMC_SDR12;
		pinctrl_state = host->sdr12_pinctrl_state;
		break;
	case MMC_TIMING_SD_HS:
	case MMC_TIMING_MMC_HS:
		val |= AC12_UHSMC_RES;
		pinctrl_state = host->hs_pinctrl_state;
		break;
	case MMC_TIMING_MMC_DDR52:
		val |= AC12_UHSMC_RES;
		pinctrl_state = host->ddr_1_8v_pinctrl_state;
		break;
	default:
		val |= AC12_UHSMC_RES;
		pinctrl_state = host->default_pinctrl_state;
		break;
	}
	OMAP_HSMMC_WRITE(host->base, AC12, val);

	if (host->pdata->controller_flags & OMAP_HSMMC_REQUIRE_IODELAY) {
		ret = pinctrl_select_state(host->pinctrl, pinctrl_state);
		if (ret) {
			dev_err(mmc_dev(host->mmc),
				"failed to select pinctrl state\n");
			return;
		}
		host->pinctrl_state = pinctrl_state;
	}

	omap_hsmmc_start_clock(host);
}

/* Routine to configure clock values. Exposed API to core */
static void omap_hsmmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	int do_send_init_stream = 0;

	pm_runtime_get_sync(host->dev);

	if (ios->power_mode != host->power_mode) {
		switch (ios->power_mode) {
		case MMC_POWER_OFF:
			omap_hsmmc_disable_tuning(host);
			omap_hsmmc_set_power(host->dev, 0, 0);
			break;
		case MMC_POWER_UP:
			omap_hsmmc_set_power(host->dev, 1, ios->vdd);
			break;
		case MMC_POWER_ON:
			omap_hsmmc_conf_bus_power(host, ios->signal_voltage);
			do_send_init_stream = 1;
			break;
		}
		host->power_mode = ios->power_mode;
	}

	/* FIXME: set registers based only on changes to ios */

	omap_hsmmc_set_bus_width(host);

	omap_hsmmc_set_clock(host);

	if (ios->timing != host->timing) {
		omap_hsmmc_set_timing(host);
		host->timing = ios->timing;
	}

	if (do_send_init_stream)
		send_init_stream(host);

	omap_hsmmc_set_bus_mode(host);

	pm_runtime_put_autosuspend(host->dev);
}

static int omap_hsmmc_get_cd(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	if (!host->card_detect)
		return -ENOSYS;
	return host->card_detect(host->dev);
}

static void omap_hsmmc_init_card(struct mmc_host *mmc, struct mmc_card *card)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	if (mmc_pdata(host)->init_card)
		mmc_pdata(host)->init_card(card);
}

static void omap_hsmmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	u32 irq_mask, con;
	unsigned long flags;

	spin_lock_irqsave(&host->irq_lock, flags);

	con = OMAP_HSMMC_READ(host->base, CON);
	irq_mask = OMAP_HSMMC_READ(host->base, ISE);
	if (enable) {
		host->flags |= HSMMC_SDIO_IRQ_ENABLED;
		irq_mask |= CIRQ_EN;
		con |= CTPL | CLKEXTFREE;
	} else {
		host->flags &= ~HSMMC_SDIO_IRQ_ENABLED;
		irq_mask &= ~CIRQ_EN;
		con &= ~(CTPL | CLKEXTFREE);
	}
	OMAP_HSMMC_WRITE(host->base, CON, con);
	OMAP_HSMMC_WRITE(host->base, IE, irq_mask);

	/*
	 * if enable, piggy back detection on current request
	 * but always disable immediately
	 */
	if (!host->req_in_progress || !enable)
		OMAP_HSMMC_WRITE(host->base, ISE, irq_mask);

	/* flush posted write */
	OMAP_HSMMC_READ(host->base, IE);

	spin_unlock_irqrestore(&host->irq_lock, flags);
}

static int omap_hsmmc_configure_wake_irq(struct omap_hsmmc_host *host)
{
	int ret;

	/*
	 * For omaps with wake-up path, wakeirq will be irq from pinctrl and
	 * for other omaps, wakeirq will be from GPIO (dat line remuxed to
	 * gpio). wakeirq is needed to detect sdio irq in runtime suspend state
	 * with functional clock disabled.
	 */
	if (!host->dev->of_node || !host->wake_irq)
		return -ENODEV;

	ret = dev_pm_set_dedicated_wake_irq(host->dev, host->wake_irq);
	if (ret) {
		dev_err(mmc_dev(host->mmc), "Unable to request wake IRQ\n");
		goto err;
	}

	/*
	 * Some omaps don't have wake-up path from deeper idle states
	 * and need to remux SDIO DAT1 to GPIO for wake-up from idle.
	 */
	if (host->pdata->controller_flags & OMAP_HSMMC_SWAKEUP_MISSING) {
		struct pinctrl *p = devm_pinctrl_get(host->dev);
		if (!p) {
			ret = -ENODEV;
			goto err_free_irq;
		}
		if (IS_ERR(pinctrl_lookup_state(p, PINCTRL_STATE_DEFAULT))) {
			dev_info(host->dev, "missing default pinctrl state\n");
			devm_pinctrl_put(p);
			ret = -EINVAL;
			goto err_free_irq;
		}

		if (IS_ERR(pinctrl_lookup_state(p, PINCTRL_STATE_IDLE))) {
			dev_info(host->dev, "missing idle pinctrl state\n");
			devm_pinctrl_put(p);
			ret = -EINVAL;
			goto err_free_irq;
		}
		devm_pinctrl_put(p);
	}

	OMAP_HSMMC_WRITE(host->base, HCTL,
			 OMAP_HSMMC_READ(host->base, HCTL) | IWE);
	return 0;

err_free_irq:
	dev_pm_clear_wake_irq(host->dev);
err:
	dev_warn(host->dev, "no SDIO IRQ support, falling back to polling\n");
	host->wake_irq = 0;
	return ret;
}

static void omap_hsmmc_set_capabilities(struct omap_hsmmc_host *host)
{
	u32 val;

	val = OMAP_HSMMC_READ(host->base, CAPA);

	/* Only MMC1 supports 3.0V */
	if (host->pdata->controller_flags & OMAP_HSMMC_SUPPORTS_DUAL_VOLT)
		val |= (VS30 | VS18);
	else
		val |= VS18;

	OMAP_HSMMC_WRITE(host->base, CAPA, val);
}

static void omap_hsmmc_conf_bus_power(struct omap_hsmmc_host *host, int iov)
{
	u32 hctl, value;

	value = OMAP_HSMMC_READ(host->base, HCTL) & ~SDVS_MASK;
	hctl = (iov == MMC_SIGNAL_VOLTAGE_180) ? SDVS18 : SDVS30;
	OMAP_HSMMC_WRITE(host->base, HCTL, value | hctl);

	/* Set SD bus power bit */
	set_sd_bus_power(host);
}

static int omap_hsmmc_multi_io_quirk(struct mmc_card *card,
				     unsigned int direction, int blk_size)
{
	/* This controller can't do multiblock reads due to hw bugs */
	if (direction == MMC_DATA_READ)
		return 1;

	return blk_size;
}

static int omap_hsmmc_start_signal_voltage_switch(struct mmc_host *mmc,
						  struct mmc_ios *ios)
{
	struct omap_hsmmc_host *host;
	u32 val = 0;
	int ret = 0;

	host  = mmc_priv(mmc);

	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		val = OMAP_HSMMC_READ(host->base, CAPA);
		if (!(val & VS30))
			return -EOPNOTSUPP;

		omap_hsmmc_conf_bus_power(host, ios->signal_voltage);

		val = OMAP_HSMMC_READ(host->base, AC12);
		val &= ~AC12_V1V8_SIGEN;
		OMAP_HSMMC_WRITE(host->base, AC12, val);

		ret = omap_hsmmc_set_power(host->dev, 1, VDD_30_31);
		if (ret) {
			dev_err(mmc_dev(host->mmc), "failed to switch to 3v\n");
			return ret;
		}

		dev_dbg(mmc_dev(host->mmc), " i/o voltage switch to 3V\n");
	} else if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		val = OMAP_HSMMC_READ(host->base, CAPA);
		if (!(val & VS18))
			return -EOPNOTSUPP;

		omap_hsmmc_conf_bus_power(host, ios->signal_voltage);

		val = OMAP_HSMMC_READ(host->base, AC12);
		val |= AC12_V1V8_SIGEN;
		OMAP_HSMMC_WRITE(host->base, AC12, val);

		ret = omap_hsmmc_set_power(host->dev, 1, VDD_165_195);
		if (ret < 0) {
			dev_err(mmc_dev(host->mmc), "failed to switch 1.8v\n");
			return ret;
		}
	} else {
		return -EOPNOTSUPP;
	}

	return 0;
}

static int omap_hsmmc_card_busy_low(struct omap_hsmmc_host *host)
{
	int i;
	u32 val;

	val = OMAP_HSMMC_READ(host->base, CON);
	val &= ~CON_CLKEXTFREE;
	val |= CON_PADEN;
	OMAP_HSMMC_WRITE(host->base, CON, val);

	/* By observation, card busy status reflects in 100 - 200us */
	for (i = 0; i < 5; i++) {
		val = OMAP_HSMMC_READ(host->base, PSTATE);
		if (!(val & (PSTATE_CLEV | PSTATE_DLEV)))
			return true;

		usleep_range(100, 200);
	}

	dev_dbg(mmc_dev(host->mmc), "card busy\n");

	return false;
}

static int omap_hsmmc_card_busy_high(struct omap_hsmmc_host *host)
{
	int i;
	u32 val;
	int ret = true;

	val = OMAP_HSMMC_READ(host->base, CON);
	val |= CLKEXTFREE;
	OMAP_HSMMC_WRITE(host->base, CON, val);

	host->flags |= CLKEXTFREE_ENABLED;
	disable_irq(host->irq);
	omap_hsmmc_enable_irq(host, NULL);

	/* By observation, card busy status reflects in 100 - 200us */
	for (i = 0; i < 5; i++) {
		val = OMAP_HSMMC_READ(host->base, PSTATE);
		if ((val & PSTATE_CLEV) && (val & PSTATE_DLEV)) {
			val = OMAP_HSMMC_READ(host->base, CON);
			val &= ~(CON_CLKEXTFREE | CON_PADEN);
			OMAP_HSMMC_WRITE(host->base, CON, val);
			ret = false;
			goto disable_irq;
		}

		usleep_range(100, 200);
	}

	dev_err(mmc_dev(host->mmc), "card busy\n");

disable_irq:
	omap_hsmmc_disable_irq(host);
	enable_irq(host->irq);
	host->flags &= ~CLKEXTFREE_ENABLED;

	return ret;
}

static int omap_hsmmc_card_busy(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host;
	u32 val;
	int ret;

	host  = mmc_priv(mmc);

	if (host->last_cmd != SD_SWITCH_VOLTAGE) {
		val = OMAP_HSMMC_READ(host->base, PSTATE);
		if (val & PSTATE_DLEV_DAT0)
			return true;
		return false;
	}

	val = OMAP_HSMMC_READ(host->base, AC12);
	if (val & AC12_V1V8_SIGEN)
		ret = omap_hsmmc_card_busy_high(host);
	else
		ret = omap_hsmmc_card_busy_low(host);

	return ret;
}

static inline void omap_hsmmc_set_dll(struct omap_hsmmc_host *host, int count)
{
	int i;
	u32 dll;

	dll = OMAP_HSMMC_READ(host->base, DLL);
	dll |= DLL_FORCE_VALUE;
	dll &= ~(DLL_FORCE_SR_C_MASK << DLL_FORCE_SR_C_SHIFT);
	dll |= (count << DLL_FORCE_SR_C_SHIFT);
	OMAP_HSMMC_WRITE(host->base, DLL, dll);

	dll |= DLL_CALIB;
	OMAP_HSMMC_WRITE(host->base, DLL, dll);
	for (i = 0; i < 1000; i++) {
		if (OMAP_HSMMC_READ(host->base, DLL) & DLL_CALIB)
			break;
	}
	dll &= ~DLL_CALIB;
	OMAP_HSMMC_WRITE(host->base, DLL, dll);
}

static void omap_hsmmc_disable_tuning(struct omap_hsmmc_host *host)
{
	int val;

	val = OMAP_HSMMC_READ(host->base, AC12);
	val &= ~(AC12_SCLK_SEL);
	OMAP_HSMMC_WRITE(host->base, AC12, val);

	val = OMAP_HSMMC_READ(host->base, DLL);
	val &= ~(DLL_FORCE_VALUE | DLL_SWT);
	OMAP_HSMMC_WRITE(host->base, DLL, val);
}

static int omap_hsmmc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	u32 val;
	u8 cur_match, prev_match = 0;
	int ret;
	u32 phase_delay = 0;
	u32 start_window = 0, max_window = 0;
	u32 length = 0, max_len = 0;
	struct mmc_ios *ios = &mmc->ios;
	struct omap_hsmmc_host *host;

	/* clock tuning is not needed for upto 52MHz */
	if (ios->clock <= OMAP_MMC_MAX_CLOCK)
		return 0;

	host = mmc_priv(mmc);

	val = OMAP_HSMMC_READ(host->base, CAPA2);
	if (ios->timing == MMC_TIMING_UHS_SDR50 && !(val & CAPA2_TSDR50))
		return 0;

	val = OMAP_HSMMC_READ(host->base, DLL);
	val |= DLL_SWT;
	OMAP_HSMMC_WRITE(host->base, DLL, val);

	while (phase_delay <= MAX_PHASE_DELAY) {
		omap_hsmmc_set_dll(host, phase_delay);

		cur_match = !mmc_send_tuning(mmc, opcode, NULL);

		if (cur_match) {
			if (prev_match) {
				length++;
			} else {
				start_window = phase_delay;
				length = 1;
			}
		}

		if (length > max_len) {
			max_window = start_window;
			max_len = length;
		}

		prev_match = cur_match;
		phase_delay += 4;
	}

	if (!max_len) {
		dev_err(mmc_dev(host->mmc), "Unable to find match\n");
		ret = -EIO;
		goto tuning_error;
	}

	val = OMAP_HSMMC_READ(host->base, AC12);
	if (!(val & AC12_SCLK_SEL)) {
		ret = -EIO;
		goto tuning_error;
	}

	phase_delay = max_window + 4 * (max_len >> 1);
	omap_hsmmc_set_dll(host, phase_delay);

	omap_hsmmc_reset_controller_fsm(host, SRD);
	omap_hsmmc_reset_controller_fsm(host, SRC);

	return 0;

tuning_error:
	dev_err(mmc_dev(host->mmc),
		"Tuning failed. Using fixed sampling clock\n");

	omap_hsmmc_disable_tuning(host);
	omap_hsmmc_reset_controller_fsm(host, SRD);
	omap_hsmmc_reset_controller_fsm(host, SRC);

	return ret;
}

static struct mmc_host_ops omap_hsmmc_ops = {
	.post_req = omap_hsmmc_post_req,
	.pre_req = omap_hsmmc_pre_req,
	.request = omap_hsmmc_request,
	.set_ios = omap_hsmmc_set_ios,
	.get_cd = omap_hsmmc_get_cd,
	.get_ro = mmc_gpio_get_ro,
	.init_card = omap_hsmmc_init_card,
	.enable_sdio_irq = omap_hsmmc_enable_sdio_irq,
	.start_signal_voltage_switch = omap_hsmmc_start_signal_voltage_switch,
	.card_busy = omap_hsmmc_card_busy,
	.execute_tuning = omap_hsmmc_execute_tuning,
};

#ifdef CONFIG_DEBUG_FS

static int omap_hsmmc_regs_show(struct seq_file *s, void *data)
{
	struct mmc_host *mmc = s->private;
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	seq_printf(s, "mmc%d:\n", mmc->index);
	seq_printf(s, "sdio irq mode\t%s\n",
		   (mmc->caps & MMC_CAP_SDIO_IRQ) ? "interrupt" : "polling");

	if (mmc->caps & MMC_CAP_SDIO_IRQ) {
		seq_printf(s, "sdio irq \t%s\n",
			   (host->flags & HSMMC_SDIO_IRQ_ENABLED) ?  "enabled"
			   : "disabled");
	}
	seq_printf(s, "ctx_loss:\t%d\n", host->context_loss);

	pm_runtime_get_sync(host->dev);
	seq_puts(s, "\nregs:\n");
	seq_printf(s, "CON:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, CON));
	seq_printf(s, "PSTATE:\t\t0x%08x\n",
		   OMAP_HSMMC_READ(host->base, PSTATE));
	seq_printf(s, "HCTL:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, HCTL));
	seq_printf(s, "SYSCTL:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, SYSCTL));
	seq_printf(s, "IE:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, IE));
	seq_printf(s, "ISE:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, ISE));
	seq_printf(s, "CAPA:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, CAPA));

	pm_runtime_mark_last_busy(host->dev);
	pm_runtime_put_autosuspend(host->dev);

	return 0;
}

static int omap_hsmmc_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, omap_hsmmc_regs_show, inode->i_private);
}

static const struct file_operations mmc_regs_fops = {
	.open           = omap_hsmmc_regs_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static void omap_hsmmc_debugfs(struct mmc_host *mmc)
{
	if (mmc->debugfs_root)
		debugfs_create_file("regs", S_IRUSR, mmc->debugfs_root,
			mmc, &mmc_regs_fops);
}

#else

static void omap_hsmmc_debugfs(struct mmc_host *mmc)
{
}

#endif

#ifdef CONFIG_OF
static const struct omap_mmc_of_data omap3_pre_es3_mmc_of_data = {
	/* See 35xx errata 2.1.1.128 in SPRZ278F */
	.controller_flags = OMAP_HSMMC_BROKEN_MULTIBLOCK_READ,
};

static const struct omap_mmc_of_data omap4_mmc_of_data = {
	.reg_offset = 0x100,
	.controller_flags = OMAP_HSMMC_HAS_HWPARAM,
};
static const struct omap_mmc_of_data am33xx_mmc_of_data = {
	.reg_offset = 0x100,
	.controller_flags = OMAP_HSMMC_SWAKEUP_MISSING,
};

static const struct omap_mmc_of_data dra7_mmc_of_data = {
	.reg_offset = 0x100,
	.controller_flags = OMAP_HSMMC_SWAKEUP_MISSING |
			    OMAP_HSMMC_REQUIRE_IODELAY |
			    OMAP_HSMMC_HAS_HWPARAM,
};

static const struct of_device_id omap_mmc_of_match[] = {
	{
		.compatible = "ti,omap2-hsmmc",
	},
	{
		.compatible = "ti,omap3-pre-es3-hsmmc",
		.data = &omap3_pre_es3_mmc_of_data,
	},
	{
		.compatible = "ti,omap3-hsmmc",
	},
	{
		.compatible = "ti,omap4-hsmmc",
		.data = &omap4_mmc_of_data,
	},
	{
		.compatible = "ti,am33xx-hsmmc",
		.data = &am33xx_mmc_of_data,
	},
	{
		.compatible = "ti,dra7-hsmmc",
		.data = &dra7_mmc_of_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, omap_mmc_of_match);

static struct omap_hsmmc_platform_data *of_get_hsmmc_pdata(struct device *dev)
{
	struct omap_hsmmc_platform_data *pdata = dev->platform_data;
	struct device_node *np = dev->of_node;

	/* no pdata quirks! */
	if (!pdata) {
		pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return ERR_PTR(-ENOMEM); /* out of memory */
	}

	if (of_find_property(np, "ti,dual-volt", NULL))
		pdata->controller_flags |= OMAP_HSMMC_SUPPORTS_DUAL_VOLT;

	pdata->gpio_cd = -EINVAL;
	pdata->gpio_cod = -EINVAL;
	pdata->gpio_wp = -EINVAL;

	if (of_find_property(np, "ti,non-removable", NULL)) {
		pdata->nonremovable = true;
		pdata->no_regulator_off_init = true;
	}

	if (of_find_property(np, "ti,needs-special-reset", NULL))
		pdata->features |= HSMMC_HAS_UPDATED_RESET;

	if (of_find_property(np, "ti,needs-special-hs-handling", NULL))
		pdata->features |= HSMMC_HAS_HSPE_SUPPORT;

	return pdata;
}
#else
static inline struct omap_hsmmc_platform_data
			*of_get_hsmmc_pdata(struct device *dev)
{
	return ERR_PTR(-EINVAL);
}
#endif

#define OMAP_HSMMC_SETUP_PINCTRL(capvar, capmask, mode)			\
	do {								\
		struct pinctrl_state *s = ERR_PTR(-ENODEV);		\
		char str[20];						\
		char *version = host->pdata->version;			\
									\
		if (!(mmc->capvar & (capmask)))				\
			break;						\
									\
		if (host->pdata->version) {				\
			sprintf(str, "%s-%s", #mode, version);		\
			s = pinctrl_lookup_state(host->pinctrl, str);	\
		}							\
									\
		if (IS_ERR(s)) {					\
			sprintf(str, "%s", #mode);			\
			s = pinctrl_lookup_state(host->pinctrl, str);	\
		}							\
									\
		if (IS_ERR(s)) {					\
			dev_err(host->dev, "no pinctrl state for %s "	\
				"mode\n", #mode);			\
			mmc->capvar &= ~(capmask);			\
		} else {						\
			host->mode##_pinctrl_state = s;			\
		}							\
									\
	} while (0)

static int omap_hsmmc_get_iodelay_pinctrl_state(struct omap_hsmmc_host *host)
{
	struct mmc_host *mmc = host->mmc;

	if (!(host->pdata->controller_flags & OMAP_HSMMC_REQUIRE_IODELAY))
		return 0;

	host->pinctrl = devm_pinctrl_get(host->dev);
	if (IS_ERR(host->pinctrl)) {
		dev_err(host->dev, "Cannot get pinctrl\n");
		return PTR_ERR(host->pinctrl);
	}

	host->default_pinctrl_state = pinctrl_lookup_state(host->pinctrl,
							   "default");
	if (IS_ERR(host->default_pinctrl_state)) {
		dev_err(host->dev,
			"no pinctrl state for default mode\n");
		return PTR_ERR(host->default_pinctrl_state);
	}

	OMAP_HSMMC_SETUP_PINCTRL(caps,	MMC_CAP_UHS_SDR104,	sdr104);
	OMAP_HSMMC_SETUP_PINCTRL(caps,	MMC_CAP_UHS_DDR50,	ddr50);
	OMAP_HSMMC_SETUP_PINCTRL(caps,	MMC_CAP_UHS_SDR50,	sdr50);
	OMAP_HSMMC_SETUP_PINCTRL(caps,	MMC_CAP_UHS_SDR25,	sdr25);
	OMAP_HSMMC_SETUP_PINCTRL(caps,	MMC_CAP_UHS_SDR12,	sdr12);
	OMAP_HSMMC_SETUP_PINCTRL(caps,	MMC_CAP_1_8V_DDR,	ddr_1_8v);
	OMAP_HSMMC_SETUP_PINCTRL(caps,
				 MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
				 hs);
	OMAP_HSMMC_SETUP_PINCTRL(caps2,	MMC_CAP2_HS200_1_8V_SDR,
				 hs200_1_8v);

	host->pinctrl_state = host->default_pinctrl_state;

	return 0;
}

static int omap_hsmmc_adma_init(struct omap_hsmmc_host *host)
{
	struct mmc_host *mmc = host->mmc;
	u32 val;

	host->adma_desc_table = dma_alloc_coherent(host->dev, mmc->max_segs + 1,
						   &host->adma_desc_table_addr,
						   GFP_KERNEL);
	if (!host->adma_desc_table) {
		dev_err(host->dev, "failed to allocate adma desc table\n");
		return -ENOMEM;
	}

	val = OMAP_HSMMC_READ(host->base, HCTL);
	val |= DMA_SELECT;
	OMAP_HSMMC_WRITE(host->base, HCTL, val);

	val = OMAP_HSMMC_READ(host->base, CON);
	val |= DMA_MASTER;
	OMAP_HSMMC_WRITE(host->base, CON, val);

	return 0;
}

static void omap_hsmmc_adma_exit(struct omap_hsmmc_host *host)
{
	struct mmc_host *mmc = host->mmc;

	dma_free_coherent(host->dev, mmc->max_segs + 1,
			  host->adma_desc_table, host->adma_desc_table_addr);
}

static int omap_hsmmc_dma_init(struct omap_hsmmc_host *host)
{
	host->rx_chan = dma_request_chan(host->dev, "rx");
	if (IS_ERR(host->rx_chan)) {
		dev_err(mmc_dev(host->mmc), "RX DMA channel request failed\n");
		return PTR_ERR(host->rx_chan);
	}

	host->tx_chan = dma_request_chan(host->dev, "tx");
	if (IS_ERR(host->tx_chan)) {
		dev_err(mmc_dev(host->mmc), "TX DMA channel request failed\n");
		return PTR_ERR(host->tx_chan);
	}

	return 0;
}

static void omap_hsmmc_dma_exit(struct omap_hsmmc_host *host)
{
	if (!IS_ERR_OR_NULL(host->tx_chan))
		dma_release_channel(host->tx_chan);
	if (!IS_ERR_OR_NULL(host->rx_chan))
		dma_release_channel(host->rx_chan);
}

static int omap_hsmmc_probe(struct platform_device *pdev)
{
	struct omap_hsmmc_platform_data *pdata = pdev->dev.platform_data;
	struct mmc_host *mmc;
	struct omap_hsmmc_host *host = NULL;
	struct resource *res;
	int ret, irq;
	u32 val;
	const struct of_device_id *match;
	const struct omap_mmc_of_data *data;
	void __iomem *base;

	match = of_match_device(of_match_ptr(omap_mmc_of_match), &pdev->dev);
	if (match) {
		pdata = of_get_hsmmc_pdata(&pdev->dev);

		if (IS_ERR(pdata))
			return PTR_ERR(pdata);

		if (match->data) {
			data = match->data;
			pdata->reg_offset = data->reg_offset;
			pdata->controller_flags |= data->controller_flags;
		}
	}

	if (pdata == NULL) {
		dev_err(&pdev->dev, "Platform Data is missing\n");
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (res == NULL || irq < 0)
		return -ENXIO;

	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	mmc = mmc_alloc_host(sizeof(struct omap_hsmmc_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto err;
	}

	ret = mmc_of_parse(mmc);
	if (ret)
		goto err1;

	host		= mmc_priv(mmc);
	host->mmc	= mmc;
	host->pdata	= pdata;
	host->dev	= &pdev->dev;
	host->dma_ch	= -1;
	host->irq	= irq;
	host->mapbase	= res->start + pdata->reg_offset;
	host->base	= base + pdata->reg_offset;
	host->power_mode = MMC_POWER_OFF;
	host->timing	= 0;
	host->next_data.cookie = 1;
	host->pbias_enabled = 0;
	host->vqmmc_enabled = 0;
	host->use_adma	= false;

	ret = omap_hsmmc_gpio_init(mmc, host, pdata);
	if (ret)
		goto err_gpio;

	platform_set_drvdata(pdev, host);

	if (pdev->dev.of_node)
		host->wake_irq = irq_of_parse_and_map(pdev->dev.of_node, 1);

	mmc->ops	= &omap_hsmmc_ops;

	mmc->f_min = OMAP_MMC_MIN_CLOCK;

	if (pdata->max_freq > 0)
		mmc->f_max = pdata->max_freq;
	else if (mmc->f_max == 0)
		mmc->f_max = OMAP_MMC_MAX_CLOCK;

	spin_lock_init(&host->irq_lock);
	setup_timer(&host->timer, omap_hsmmc_soft_timeout,
		    (unsigned long)host);

	host->fclk = devm_clk_get(&pdev->dev, "fck");
	if (IS_ERR(host->fclk)) {
		ret = PTR_ERR(host->fclk);
		goto err1;
	}

	ret = clk_set_rate(host->fclk, mmc->f_max);
	if (ret) {
		dev_err(&pdev->dev, "failed to set clock to %d\n", mmc->f_max);
		goto err1;
	}

	if (host->pdata->controller_flags & OMAP_HSMMC_BROKEN_MULTIBLOCK_READ) {
		dev_info(&pdev->dev, "multiblock reads disabled due to 35xx erratum 2.1.1.128; MMC read performance may suffer\n");
		omap_hsmmc_ops.multi_io_quirk = omap_hsmmc_multi_io_quirk;
	}

	device_init_wakeup(&pdev->dev, true);
	pm_runtime_enable(host->dev);
	pm_runtime_get_sync(host->dev);
	pm_runtime_set_autosuspend_delay(host->dev, MMC_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(host->dev);

	omap_hsmmc_context_save(host);

	host->dbclk = devm_clk_get(&pdev->dev, "mmchsdb_fck");
	/*
	 * MMC can still work without debounce clock.
	 */
	if (IS_ERR(host->dbclk)) {
		host->dbclk = NULL;
	} else if (clk_prepare_enable(host->dbclk) != 0) {
		dev_warn(mmc_dev(host->mmc), "Failed to enable debounce clk\n");
		host->dbclk = NULL;
	}

	if (host->pdata->controller_flags & OMAP_HSMMC_HAS_HWPARAM) {
		val = OMAP_HSMMC_READ(base, HL_HWINFO);
		if (val & MADMA_EN)
			host->use_adma = true;
	}

	/* Since we do only SG emulation, we can have as many segs
	 * as we want. */
	mmc->max_segs = 1024;

	mmc->max_blk_size = 512;       /* Block Length at max can be 1024 */
	mmc->max_blk_count = 0xFFFF;    /* No. of Blocks is 16 bits */
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	if (host->use_adma)
		mmc->max_seg_size = ADMA_MAX_LEN;
	else
		mmc->max_seg_size = mmc->max_req_size;

	mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
		     MMC_CAP_WAIT_WHILE_BUSY | MMC_CAP_ERASE;

	mmc->caps |= mmc_pdata(host)->caps;
	if (mmc->caps & MMC_CAP_8_BIT_DATA)
		mmc->caps |= MMC_CAP_4_BIT_DATA;

	if (mmc_pdata(host)->nonremovable)
		mmc->caps |= MMC_CAP_NONREMOVABLE;

	mmc->pm_caps |= mmc_pdata(host)->pm_caps;

	omap_hsmmc_set_capabilities(host);

	ret = omap_hsmmc_get_iodelay_pinctrl_state(host);
	if (ret)
		goto err_pinctrl;

	if (host->use_adma)
		ret = omap_hsmmc_adma_init(host);
	else
		ret = omap_hsmmc_dma_init(host);
	if (ret)
		goto err_irq;

	/* Request IRQ for MMC operations */
	ret = devm_request_irq(&pdev->dev, host->irq, omap_hsmmc_irq, 0,
			mmc_hostname(mmc), host);
	if (ret) {
		dev_err(mmc_dev(host->mmc), "Unable to grab HSMMC IRQ\n");
		goto err_irq;
	}

	ret = omap_hsmmc_reg_get(host);
	if (ret)
		goto err_irq;

	mmc->ocr_avail = mmc_pdata(host)->ocr_mask;

	omap_hsmmc_disable_irq(host);

	/*
	 * For now, only support SDIO interrupt if we have a separate
	 * wake-up interrupt configured from device tree. This is because
	 * the wake-up interrupt is needed for idle state and some
	 * platforms need special quirks. And we don't want to add new
	 * legacy mux platform init code callbacks any longer as we
	 * are moving to DT based booting anyways.
	 */
	ret = omap_hsmmc_configure_wake_irq(host);
	if (!ret)
		mmc->caps |= MMC_CAP_SDIO_IRQ;

	omap_hsmmc_protect_card(host);

	mmc_add_host(mmc);

	if (mmc_pdata(host)->name != NULL) {
		ret = device_create_file(&mmc->class_dev, &dev_attr_slot_name);
		if (ret < 0)
			goto err_slot_name;
	}
	if (host->get_cover_state) {
		ret = device_create_file(&mmc->class_dev,
					 &dev_attr_cover_switch);
		if (ret < 0)
			goto err_slot_name;
	}

	omap_hsmmc_debugfs(mmc);
	pm_runtime_mark_last_busy(host->dev);
	pm_runtime_put_autosuspend(host->dev);

	return 0;

err_slot_name:
	mmc_remove_host(mmc);
err_irq:
	if (host->use_adma)
		omap_hsmmc_adma_exit(host);
	else
		omap_hsmmc_dma_exit(host);
err_pinctrl:
	if (host->dbclk)
		clk_disable_unprepare(host->dbclk);
	pm_runtime_put_sync(host->dev);
	pm_runtime_disable(host->dev);
	device_init_wakeup(&pdev->dev, false);
err1:
err_gpio:
	mmc_free_host(mmc);
err:
	return ret;
}

static int omap_hsmmc_remove(struct platform_device *pdev)
{
	struct omap_hsmmc_host *host = platform_get_drvdata(pdev);

	pm_runtime_get_sync(host->dev);
	mmc_remove_host(host->mmc);

	if (host->use_adma)
		omap_hsmmc_adma_exit(host);
	else
		omap_hsmmc_dma_exit(host);

	del_timer_sync(&host->timer);

	pm_runtime_put_sync(host->dev);
	pm_runtime_disable(host->dev);
	device_init_wakeup(&pdev->dev, false);
	if (host->dbclk)
		clk_disable_unprepare(host->dbclk);

	mmc_free_host(host->mmc);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int omap_hsmmc_suspend(struct device *dev)
{
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);

	if (!host)
		return 0;

	pm_runtime_get_sync(host->dev);

	if (!(host->mmc->pm_flags & MMC_PM_KEEP_POWER)) {
		OMAP_HSMMC_WRITE(host->base, ISE, 0);
		OMAP_HSMMC_WRITE(host->base, IE, 0);
		OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
		OMAP_HSMMC_WRITE(host->base, HCTL,
				OMAP_HSMMC_READ(host->base, HCTL) & ~SDBP);
	}

	if (host->dbclk)
		clk_disable_unprepare(host->dbclk);

	del_timer_sync(&host->timer);

	pm_runtime_put_sync(host->dev);
	return 0;
}

/* Routine to resume the MMC device */
static int omap_hsmmc_resume(struct device *dev)
{
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);
	struct mmc_ios *ios;

	if (!host)
		return 0;

	pm_runtime_get_sync(host->dev);

	if (host->dbclk)
		clk_prepare_enable(host->dbclk);

	ios = &host->mmc->ios;
	if (!(host->mmc->pm_flags & MMC_PM_KEEP_POWER))
		omap_hsmmc_conf_bus_power(host, ios->signal_voltage);

	omap_hsmmc_protect_card(host);
	pm_runtime_mark_last_busy(host->dev);
	pm_runtime_put_autosuspend(host->dev);
	return 0;
}
#endif

static int omap_hsmmc_runtime_suspend(struct device *dev)
{
	struct omap_hsmmc_host *host;
	unsigned long flags;
	int ret = 0;

	host = platform_get_drvdata(to_platform_device(dev));
	omap_hsmmc_context_save(host);
	dev_dbg(dev, "disabled\n");

	spin_lock_irqsave(&host->irq_lock, flags);
	if ((host->mmc->caps & MMC_CAP_SDIO_IRQ) &&
	    (host->flags & HSMMC_SDIO_IRQ_ENABLED)) {
		/* disable sdio irq handling to prevent race */
		OMAP_HSMMC_WRITE(host->base, ISE, 0);
		OMAP_HSMMC_WRITE(host->base, IE, 0);

		if (!(OMAP_HSMMC_READ(host->base, PSTATE) & DLEV_DAT(1))) {
			/*
			 * dat1 line low, pending sdio irq
			 * race condition: possible irq handler running on
			 * multi-core, abort
			 */
			dev_dbg(dev, "pending sdio irq, abort suspend\n");
			OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
			OMAP_HSMMC_WRITE(host->base, ISE, CIRQ_EN);
			OMAP_HSMMC_WRITE(host->base, IE, CIRQ_EN);
			pm_runtime_mark_last_busy(dev);
			ret = -EBUSY;
			goto abort;
		}

		pinctrl_pm_select_idle_state(dev);
	} else {
		pinctrl_pm_select_idle_state(dev);
	}

abort:
	spin_unlock_irqrestore(&host->irq_lock, flags);
	return ret;
}

static int omap_hsmmc_runtime_resume(struct device *dev)
{
	struct omap_hsmmc_host *host;
	unsigned long flags;
	int ret;

	host = platform_get_drvdata(to_platform_device(dev));
	omap_hsmmc_context_restore(host);
	dev_dbg(dev, "enabled\n");

	spin_lock_irqsave(&host->irq_lock, flags);
	if ((host->mmc->caps & MMC_CAP_SDIO_IRQ) &&
	    (host->flags & HSMMC_SDIO_IRQ_ENABLED)) {

		pinctrl_pm_select_default_state(host->dev);

		/* irq lost, if pinmux incorrect */
		OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
		OMAP_HSMMC_WRITE(host->base, ISE, CIRQ_EN);
		OMAP_HSMMC_WRITE(host->base, IE, CIRQ_EN);
	} else {
		if (host->pinctrl) {
			ret = pinctrl_select_state(host->pinctrl,
						   host->pinctrl_state);
			if (ret)
				dev_err(mmc_dev(host->mmc),
					"failed to activate pinctrl state\n");
		}
	}
	spin_unlock_irqrestore(&host->irq_lock, flags);
	return 0;
}

static struct dev_pm_ops omap_hsmmc_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(omap_hsmmc_suspend, omap_hsmmc_resume)
	.runtime_suspend = omap_hsmmc_runtime_suspend,
	.runtime_resume = omap_hsmmc_runtime_resume,
};

static struct platform_driver omap_hsmmc_driver = {
	.probe		= omap_hsmmc_probe,
	.remove		= omap_hsmmc_remove,
	.driver		= {
		.name = DRIVER_NAME,
		.pm = &omap_hsmmc_dev_pm_ops,
		.of_match_table = of_match_ptr(omap_mmc_of_match),
	},
};

module_platform_driver(omap_hsmmc_driver);
MODULE_DESCRIPTION("OMAP High Speed Multimedia Card driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
