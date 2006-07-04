/*
 * OMAP2 McSPI controller driver
 *
 * Copyright (C) 2005, 2006 Nokia Corporation
 * Author: Samuel Ortiz <samuel.ortiz@nokia.com> and
 *         Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <linux/spi/spi.h>

#include <asm/io.h>
#include <asm/arch/dma.h>
#include <asm/arch/mcspi.h>

#define OMAP2_MCSPI_MAX_FREQ		48000000

#define OMAP2_MCSPI_REVISION		0x00
#define OMAP2_MCSPI_SYSCONFIG		0x10
#define OMAP2_MCSPI_SYSSTATUS		0x14
#define OMAP2_MCSPI_IRQSTATUS		0x18
#define OMAP2_MCSPI_IRQENABLE		0x1c
#define OMAP2_MCSPI_WAKEUPENABLE	0x20
#define OMAP2_MCSPI_SYST		0x24
#define OMAP2_MCSPI_MODULCTRL		0x28
#define OMAP2_MCSPI_CHCONF0		0x2c
#define OMAP2_MCSPI_CHSTAT0		0x30
#define OMAP2_MCSPI_CHCTRL0		0x34
#define OMAP2_MCSPI_TX0			0x38
#define OMAP2_MCSPI_RX0			0x3c

#define OMAP2_MCSPI_SYSCONFIG_SOFTRESET	(1 << 1)

#define OMAP2_MCSPI_SYSSTATUS_RESETDONE	(1 << 0)

#define OMAP2_MCSPI_MODULCTRL_SINGLE	(1 << 0)
#define OMAP2_MCSPI_MODULCTRL_MS	(1 << 2)
#define OMAP2_MCSPI_MODULCTRL_STEST	(1 << 3)

#define OMAP2_MCSPI_CHCONF_PHA		(1 << 0)
#define OMAP2_MCSPI_CHCONF_POL		(1 << 1)
#define OMAP2_MCSPI_CHCONF_CLKD_MASK	(0x0f << 2)
#define OMAP2_MCSPI_CHCONF_EPOL		(1 << 6)
#define OMAP2_MCSPI_CHCONF_WL_MASK	(0x1f << 7)
#define OMAP2_MCSPI_CHCONF_TRM_RX_ONLY	(0x01 << 12)
#define OMAP2_MCSPI_CHCONF_TRM_TX_ONLY	(0x02 << 12)
#define OMAP2_MCSPI_CHCONF_TRM_MASK	(0x03 << 12)
#define OMAP2_MCSPI_CHCONF_DMAW		(1 << 14)
#define OMAP2_MCSPI_CHCONF_DMAR		(1 << 15)
#define OMAP2_MCSPI_CHCONF_DPE0		(1 << 16)
#define OMAP2_MCSPI_CHCONF_DPE1		(1 << 17)
#define OMAP2_MCSPI_CHCONF_IS		(1 << 18)
#define OMAP2_MCSPI_CHCONF_TURBO	(1 << 19)
#define OMAP2_MCSPI_CHCONF_FORCE	(1 << 20)


#define OMAP2_MCSPI_CHSTAT_RXS		(1 << 0)
#define OMAP2_MCSPI_CHSTAT_TXS		(1 << 1)
#define OMAP2_MCSPI_CHSTAT_EOT		(1 << 2)

#define OMAP2_MCSPI_CHCTRL_EN		(1 << 0)

/* We have 2 DMA channels per CS, one for RX and one for TX */
struct omap2_mcspi_dma {
	int dma_tx_channel;
	int dma_rx_channel;

	int dma_tx_sync_dev;
	int dma_rx_sync_dev;

	struct completion dma_tx_completion;
	struct completion dma_rx_completion;
};

struct omap2_mcspi {
	struct work_struct	work;
	spinlock_t		lock;
	struct list_head	msg_queue;
	struct spi_master	*master;
	struct clk		*ick;
	struct clk		*fck;
	/* Virtual base address of the controller */
	unsigned long		base;
	/* SPI1 has 4 channels, while SPI2 has 2 */
	struct omap2_mcspi_dma	*dma_channels;
};

struct omap2_mcspi_cs {
	u8 transmit_mode;
	int word_len;
};

static struct workqueue_struct * omap2_mcspi_wq;

#define MOD_REG_BIT(val, mask, set) do { \
	if (set) \
		val |= mask; \
	else \
		val &= ~mask; \
} while(0)

static inline void mcspi_write_reg(struct spi_master *master,
				   int idx, u32 val)
{
	struct omap2_mcspi * mcspi = class_get_devdata(&master->cdev);

	__raw_writel(val, mcspi->base + idx);
}

static inline u32 mcspi_read_reg(struct spi_master *master,
				 int idx)
{
	struct omap2_mcspi * mcspi = class_get_devdata(&master->cdev);

	return __raw_readl(mcspi->base + idx);
}

static inline void mcspi_write_cs_reg(const struct spi_device *spi,
				      int idx, u32 val)
{
	struct omap2_mcspi * mcspi = class_get_devdata(&spi->master->cdev);

	__raw_writel(val, mcspi->base + spi->chip_select * 0x14 + idx);
}

static inline u32 mcspi_read_cs_reg(const struct spi_device *spi,
				    int idx)
{
	struct omap2_mcspi * mcspi = class_get_devdata(&spi->master->cdev);

	return __raw_readl(mcspi->base + spi->chip_select * 0x14 + idx);
}

static void omap2_mcspi_set_dma_req(const struct spi_device *spi,
				    int is_read, int enable)
{
	u32 l, rw;

	l = mcspi_read_cs_reg(spi, OMAP2_MCSPI_CHCONF0);

	if (is_read) /* 1 is read, 0 write */
		rw = OMAP2_MCSPI_CHCONF_DMAR;
	else
		rw = OMAP2_MCSPI_CHCONF_DMAW;

	MOD_REG_BIT(l, rw, enable);
	mcspi_write_cs_reg(spi, OMAP2_MCSPI_CHCONF0, l);
}

static void omap2_mcspi_set_enable(const struct spi_device *spi, int enable)
{
	u32 l;

	l = mcspi_read_cs_reg(spi, OMAP2_MCSPI_CHCTRL0);
	MOD_REG_BIT(l, OMAP2_MCSPI_CHCTRL_EN, enable);
	mcspi_write_cs_reg(spi, OMAP2_MCSPI_CHCTRL0, l);
}

static void omap2_mcspi_force_cs(struct spi_device *spi, int cs_active)
{
	u32 l;

	l = mcspi_read_cs_reg(spi, OMAP2_MCSPI_CHCONF0);
	MOD_REG_BIT(l, OMAP2_MCSPI_CHCONF_FORCE, cs_active);
	mcspi_write_cs_reg(spi, OMAP2_MCSPI_CHCONF0, l);
}

static void omap2_mcspi_set_master_mode(struct spi_device *spi, int single_channel)
{
	u32 l;

	/* Need reset when switching from slave mode */
	l = mcspi_read_reg(spi->master, OMAP2_MCSPI_MODULCTRL);
	MOD_REG_BIT(l, OMAP2_MCSPI_MODULCTRL_STEST, 0);
	MOD_REG_BIT(l, OMAP2_MCSPI_MODULCTRL_MS, 0);
	MOD_REG_BIT(l, OMAP2_MCSPI_MODULCTRL_SINGLE, single_channel);
	mcspi_write_reg(spi->master, OMAP2_MCSPI_MODULCTRL, l);
}

static void omap2_mcspi_txrx_dma(struct spi_device *spi,
				 struct spi_transfer *xfer)
{
	struct omap2_mcspi      * mcspi;
	struct omap2_mcspi_cs   * cs = spi->controller_state;
	struct omap2_mcspi_dma  * mcspi_dma;
	unsigned int		count, c;
	unsigned long		base, tx_reg, rx_reg;
	int			word_len, data_type, element_count;
	u8			* rx;
	const u8		* tx;
	u32			l;

	mcspi = class_get_devdata(&spi->master->cdev);
	mcspi_dma = &mcspi->dma_channels[spi->chip_select];

	count = xfer->len;
	c = count;
	word_len = cs->word_len;

	l = mcspi_read_cs_reg(spi, OMAP2_MCSPI_CHCONF0);
	l &= ~OMAP2_MCSPI_CHCONF_TRM_MASK;
	if (xfer->tx_buf == NULL)
		l |= OMAP2_MCSPI_CHCONF_TRM_RX_ONLY;
	else if (xfer->rx_buf == NULL)
		l |= OMAP2_MCSPI_CHCONF_TRM_TX_ONLY;
	mcspi_write_cs_reg(spi, OMAP2_MCSPI_CHCONF0, l);

	omap2_mcspi_set_enable(spi, 1);

	base = io_v2p(mcspi->base) + spi->chip_select * 0x14;
	tx_reg = base + OMAP2_MCSPI_TX0;
	rx_reg = base + OMAP2_MCSPI_RX0;
	rx = xfer->rx_buf;
	tx = xfer->tx_buf;

	if (word_len <= 8) {
		data_type = OMAP_DMA_DATA_TYPE_S8;
		element_count = count;
	} else if (word_len <= 16) {
		data_type = OMAP_DMA_DATA_TYPE_S16;
		element_count = count >> 1;
	} else if (word_len <= 32) {
		data_type = OMAP_DMA_DATA_TYPE_S32;
		element_count = count >> 2;
	} else
		return;

	/* RX_ONLY mode needs dummy data in TX reg */
	if (tx == NULL)
		__raw_writel(0, mcspi->base +
			     spi->chip_select * 0x14 + OMAP2_MCSPI_TX0);

	if (tx != NULL) {
		xfer->tx_dma = dma_map_single(&spi->dev, (void *) tx, count,
					      DMA_TO_DEVICE);
		if (dma_mapping_error(xfer->tx_dma)) {
			printk(KERN_ERR "%s(): Couldn't DMA map a %d bytes TX buffer\n",
			       __FUNCTION__, count);
			return;
		}

		omap_set_dma_transfer_params(mcspi_dma->dma_tx_channel,
					     data_type, element_count, 1,
					     OMAP_DMA_SYNC_ELEMENT,
					     mcspi_dma->dma_tx_sync_dev, 0);

		omap_set_dma_dest_params(mcspi_dma->dma_tx_channel, 0,
					 OMAP_DMA_AMODE_CONSTANT,
					 tx_reg, 0, 0);

		omap_set_dma_src_params(mcspi_dma->dma_tx_channel, 0,
					OMAP_DMA_AMODE_POST_INC,
					xfer->tx_dma, 0, 0);
	}

	if (rx != NULL) {
		xfer->rx_dma = dma_map_single(&spi->dev, rx, count,
					      DMA_FROM_DEVICE);
		if (dma_mapping_error(xfer->rx_dma)) {
			printk(KERN_ERR "%s(): Couldn't DMA map a %d bytes RX buffer\n",
			       __FUNCTION__, count);
			if (tx != NULL)
				dma_unmap_single(NULL, xfer->tx_dma,
						 count, DMA_TO_DEVICE);
			return;
		}

		omap_set_dma_transfer_params(mcspi_dma->dma_rx_channel,
					     data_type, element_count, 1,
					     OMAP_DMA_SYNC_ELEMENT,
					     mcspi_dma->dma_rx_sync_dev, 1);

		omap_set_dma_src_params(mcspi_dma->dma_rx_channel, 0,
					OMAP_DMA_AMODE_CONSTANT,
					rx_reg, 0, 0);

		omap_set_dma_dest_params(mcspi_dma->dma_rx_channel, 0,
					 OMAP_DMA_AMODE_POST_INC,
					 xfer->rx_dma, 0, 0);
	}

	if (tx != NULL) {
		omap_start_dma(mcspi_dma->dma_tx_channel);
		omap2_mcspi_set_dma_req(spi, 0, 1);
	}

	if (rx != NULL) {
		omap_start_dma(mcspi_dma->dma_rx_channel);
		omap2_mcspi_set_dma_req(spi, 1, 1);
	}

	if (tx != NULL) {
		wait_for_completion(&mcspi_dma->dma_tx_completion);
		dma_unmap_single(NULL, xfer->tx_dma, count, DMA_TO_DEVICE);
	}

	if (rx != NULL) {
		wait_for_completion(&mcspi_dma->dma_rx_completion);
		dma_unmap_single(NULL, xfer->rx_dma, count, DMA_FROM_DEVICE);
	}

	omap2_mcspi_set_enable(spi, 0);
}

static void omap2_mcspi_txrx_pio(struct spi_device *spi, struct spi_transfer *xfer)
{
	struct omap2_mcspi * mcspi;
	struct omap2_mcspi_cs *cs = spi->controller_state;
	unsigned int		count, c;
	u32                     l;
	unsigned long		base, tx_reg, rx_reg, chstat_reg;
	int			word_len;

	mcspi = class_get_devdata(&spi->master->cdev);
	count = xfer->len;
	c = count;
	word_len = cs->word_len;

	l = mcspi_read_cs_reg(spi, OMAP2_MCSPI_CHCONF0);
	l &= ~OMAP2_MCSPI_CHCONF_TRM_MASK;
	if (xfer->tx_buf == NULL)
		l |= OMAP2_MCSPI_CHCONF_TRM_RX_ONLY;
	else if (xfer->rx_buf == NULL)
		l |= OMAP2_MCSPI_CHCONF_TRM_TX_ONLY;
	mcspi_write_cs_reg(spi, OMAP2_MCSPI_CHCONF0, l);

	omap2_mcspi_set_enable(spi, 1);

	/* We store the pre-calculated register addresses on stack to speed
	 * up the transfer loop. */
	base = mcspi->base + spi->chip_select * 0x14;
	tx_reg		= base + OMAP2_MCSPI_TX0;
	rx_reg		= base + OMAP2_MCSPI_RX0;
	chstat_reg	= base + OMAP2_MCSPI_CHSTAT0;

	/* RX_ONLY mode needs dummy data in TX reg */
	if (xfer->tx_buf == NULL)
		__raw_writel(0, tx_reg);

	if (word_len <= 8) {
		u8		*rx;
		const u8	*tx;

		rx = xfer->rx_buf;
		tx = xfer->tx_buf;

		while (c--) {
			if (tx != NULL) {
				while (!(__raw_readl(chstat_reg) & OMAP2_MCSPI_CHSTAT_TXS));
#ifdef VERBOSE
				dev_dbg(&spi->dev, "write-%d %02x\n",
						word_len, *tx);
#endif
				__raw_writel(*tx, tx_reg);
			}
			if (rx != NULL) {
				while (!(__raw_readl(chstat_reg) & OMAP2_MCSPI_CHSTAT_RXS));
				if (c == 0 && tx == NULL)
					omap2_mcspi_set_enable(spi, 0);
				*rx++ = __raw_readl(rx_reg);
#ifdef VERBOSE
				dev_dbg(&spi->dev, "read-%d %02x\n",
						word_len, *(rx - 1));
#endif
			}
		}
	} else if (word_len <= 16) {
		u16		*rx;
		const u16	*tx;

		rx = xfer->rx_buf;
		tx = xfer->tx_buf;
		c >>= 1;
		while (c--) {
			if (tx != NULL) {
				while (!(__raw_readl(chstat_reg) & OMAP2_MCSPI_CHSTAT_TXS));
#ifdef VERBOSE
				dev_dbg(&spi->dev, "write-%d %04x\n",
						word_len, *tx);
#endif
				__raw_writel(*tx++, tx_reg);
			}
			if (rx != NULL) {
				while (!(__raw_readl(chstat_reg) & OMAP2_MCSPI_CHSTAT_RXS));
				if (c == 0 && tx == NULL)
					omap2_mcspi_set_enable(spi, 0);
				*rx++ = __raw_readl(rx_reg);
#ifdef VERBOSE
				dev_dbg(&spi->dev, "read-%d %04x\n",
						word_len, *(rx - 1));
#endif
			}
		}
	} else if (word_len <= 32) {
		u32		*rx;
		const u32	*tx;

		rx = xfer->rx_buf;
		tx = xfer->tx_buf;
		c >>= 2;
		while (c--) {
			if (tx != NULL) {
				while (!(__raw_readl(chstat_reg) & OMAP2_MCSPI_CHSTAT_TXS));
#ifdef VERBOSE
				dev_dbg(&spi->dev, "write-%d %04x\n",
						word_len, *tx);
#endif
				__raw_writel(*tx++, tx_reg);
			}
			if (rx != NULL) {
				while (!(__raw_readl(chstat_reg) & OMAP2_MCSPI_CHSTAT_RXS));
				if (c == 0 && tx == NULL)
					omap2_mcspi_set_enable(spi, 0);
				*rx++ = __raw_readl(rx_reg);
#ifdef VERBOSE
				dev_dbg(&spi->dev, "read-%d %04x\n",
						word_len, *(rx - 1));
#endif
			}
		}
	}

	if (xfer->tx_buf != NULL) {
		while (!(__raw_readl(chstat_reg) & OMAP2_MCSPI_CHSTAT_TXS));
		while (!(__raw_readl(chstat_reg) & OMAP2_MCSPI_CHSTAT_EOT));
		omap2_mcspi_set_enable(spi, 0);
	}
}

static int omap2_mcspi_setup_transfer(struct spi_device *spi,
				      struct spi_transfer *t)
{
	struct omap2_mcspi_cs *cs = spi->controller_state;
	struct omap2_mcspi_device_config *conf;
	u32 l = 0, div = 0;
	u8 word_len = spi->bits_per_word;

	if (t != NULL && t->bits_per_word)
		word_len = t->bits_per_word;
	if (!word_len)
		word_len = 8;

	if (spi->bits_per_word > 32)
		return -EINVAL;
	cs->word_len = word_len;

	conf = (struct omap2_mcspi_device_config *) spi->controller_data;

	if (conf->single_channel == 1)
		omap2_mcspi_set_master_mode(spi, 1);
	else
		omap2_mcspi_set_master_mode(spi, 0);

	if (spi->max_speed_hz) {
		while (div <= 15 && (OMAP2_MCSPI_MAX_FREQ / (1 << div)) > spi->max_speed_hz)
			div++;
	} else
		div = 15;

	if (spi->chip_select > 3 ||
	    word_len < 4 || word_len > 32 ||
	    div > 15) {
		dev_err(&spi->dev, "Invalid McSPI channel setting\n");
		return -EINVAL;
	}

	l = mcspi_read_cs_reg(spi, OMAP2_MCSPI_CHCONF0);
	l &= ~OMAP2_MCSPI_CHCONF_IS;
	l &= ~OMAP2_MCSPI_CHCONF_DPE1;
	l |= OMAP2_MCSPI_CHCONF_DPE0;
	l &= ~OMAP2_MCSPI_CHCONF_WL_MASK;
	l |= (word_len - 1) << 7;
	if (!(spi->mode & SPI_CS_HIGH))
		l |= OMAP2_MCSPI_CHCONF_EPOL;
	else
		l &= ~OMAP2_MCSPI_CHCONF_EPOL;
	l &= ~OMAP2_MCSPI_CHCONF_CLKD_MASK;
	l |= div << 2;
	if (spi->mode & SPI_CPOL)
		l |= OMAP2_MCSPI_CHCONF_POL;
	else
		l &= ~OMAP2_MCSPI_CHCONF_POL;
	if (spi->mode & SPI_CPHA)
		l &= ~OMAP2_MCSPI_CHCONF_PHA;
	else
		l |= OMAP2_MCSPI_CHCONF_PHA;
	mcspi_write_cs_reg(spi, OMAP2_MCSPI_CHCONF0, l);

	dev_dbg(&spi->dev, "setup: speed %d, sample %s edge, clk %s inverted\n",
			OMAP2_MCSPI_MAX_FREQ / (1 << div),
			(spi->mode & SPI_CPHA) ? "odd" : "even",
			(spi->mode & SPI_CPOL) ? "" : "not");

	return 0;
}

static void omap2_mcspi_dma_rx_callback(int lch, u16 ch_status, void *data)
{
	struct spi_device * spi = (struct spi_device *)data;
	struct omap2_mcspi * mcspi;
	struct omap2_mcspi_dma * mcspi_dma;

	mcspi = class_get_devdata(&spi->master->cdev);
	mcspi_dma = &(mcspi->dma_channels[spi->chip_select]);

	complete(&mcspi_dma->dma_rx_completion);

	/* We must disable the DMA RX request */
	omap2_mcspi_set_dma_req(spi, 1, 0);
}

static void omap2_mcspi_dma_tx_callback(int lch, u16 ch_status, void *data)
{
	struct spi_device * spi = (struct spi_device *)data;
	struct omap2_mcspi * mcspi;
	struct omap2_mcspi_dma * mcspi_dma;

	mcspi = class_get_devdata(&spi->master->cdev);
	mcspi_dma = &(mcspi->dma_channels[spi->chip_select]);

	complete(&mcspi_dma->dma_tx_completion);

	/* We must disable the DMA TX request */
	omap2_mcspi_set_dma_req(spi, 0, 0);
}

static int omap2_mcspi_request_dma(struct spi_device *spi)
{
	int rx_dev_id, tx_dev_id;
	struct spi_master *master = spi->master;
	struct omap2_mcspi * mcspi;
	struct omap2_mcspi_dma * mcspi_dma;

	mcspi = class_get_devdata(&master->cdev);
	mcspi_dma = &(mcspi->dma_channels[spi->chip_select]);

	if (master->bus_num == 1) {
		switch (spi->chip_select) {
		case 0:
			rx_dev_id = OMAP24XX_DMA_SPI1_RX0;
			tx_dev_id = OMAP24XX_DMA_SPI1_TX0;
			break;
		case 1:
			rx_dev_id = OMAP24XX_DMA_SPI1_RX1;
			tx_dev_id = OMAP24XX_DMA_SPI1_TX1;
			break;
		case 2:
			rx_dev_id = OMAP24XX_DMA_SPI1_RX2;
			tx_dev_id = OMAP24XX_DMA_SPI1_TX2;
			break;
		case 3:
			rx_dev_id = OMAP24XX_DMA_SPI1_RX3;
			tx_dev_id = OMAP24XX_DMA_SPI1_TX3;
			break;
		default:
			return -EINVAL;
		}
	} else if (master->bus_num == 2) {
		/* McSPI 2 has 1 chipselect */
		switch (spi->chip_select) {
		case 0:
			rx_dev_id = OMAP24XX_DMA_SPI2_RX0;
			tx_dev_id = OMAP24XX_DMA_SPI2_TX0;
			break;
		case 1:
			rx_dev_id = OMAP24XX_DMA_SPI2_RX1;
			tx_dev_id = OMAP24XX_DMA_SPI2_TX1;
			break;
		default:
			return -EINVAL;
		}
	} else
		return -EINVAL;


	if (omap_request_dma(rx_dev_id, "McSPI RX",
			     omap2_mcspi_dma_rx_callback, spi,
			     &mcspi_dma->dma_rx_channel)) {
		printk(KERN_ERR "Unable to request DMA channel for McSPI RX\n");
		return -EAGAIN;
	}
	
	if (omap_request_dma(tx_dev_id, "McSPI TX",
			     omap2_mcspi_dma_tx_callback, spi,
			     &mcspi_dma->dma_tx_channel)) {
		omap_free_dma(mcspi_dma->dma_rx_channel);
		mcspi_dma->dma_rx_channel = -1;
		printk(KERN_ERR "Unable to request DMA channel for McSPI TX\n");
		return -EAGAIN;
	}
	
	mcspi_dma->dma_rx_sync_dev = rx_dev_id;
	mcspi_dma->dma_tx_sync_dev = tx_dev_id;

	init_completion(&mcspi_dma->dma_rx_completion);
	init_completion(&mcspi_dma->dma_tx_completion);
	
	return 0;
}

static int omap2_mcspi_setup(struct spi_device *spi)
{
	int ret;
	struct omap2_mcspi * mcspi;
	struct omap2_mcspi_dma * mcspi_dma;
	struct omap2_mcspi_cs *cs = spi->controller_state;

	mcspi = class_get_devdata(&spi->master->cdev);
	mcspi_dma = &mcspi->dma_channels[spi->chip_select];

	if (!cs) {
		cs = kzalloc(sizeof *cs, SLAB_KERNEL);
		if (!cs)
			return -ENOMEM;
		spi->controller_state = cs;
	}
	
	if (mcspi_dma->dma_rx_channel == -1 ||
	    mcspi_dma->dma_tx_channel == -1) {
		ret = omap2_mcspi_request_dma(spi);
		if (ret < 0)
			return ret;
	}

	return omap2_mcspi_setup_transfer(spi, NULL);
}

static void omap2_mcspi_cleanup(const struct spi_device *spi)
{
	struct omap2_mcspi * mcspi;
	struct omap2_mcspi_dma * mcspi_dma;

	mcspi = class_get_devdata(&spi->master->cdev);
	mcspi_dma = &mcspi->dma_channels[spi->chip_select];

	if (spi->controller_state != NULL)
		kfree(spi->controller_state);

	if (mcspi_dma->dma_rx_channel != -1 &&
	    mcspi_dma->dma_tx_channel != -1) {
		omap_free_dma(mcspi_dma->dma_tx_channel);
		omap_free_dma(mcspi_dma->dma_rx_channel);
	}
}


static void omap2_mcspi_work(void * arg)
{
	struct omap2_mcspi	*mcspi = (struct omap2_mcspi *) arg;
	unsigned long		flags;

	spin_lock_irqsave(&mcspi->lock, flags);
	while (!list_empty(&mcspi->msg_queue)) {
		struct spi_message		*m;
		struct spi_device		*spi;
		struct spi_transfer		*t = NULL;
		int				cs_active = 0;
		struct omap2_mcspi_device_config *conf;
		struct omap2_mcspi_cs		*cs;
		int				par_override = 0;
		int status = 0;

		m = container_of(mcspi->msg_queue.next, struct spi_message,
				 queue);

		list_del_init(&m->queue);
		spin_unlock_irqrestore(&mcspi->lock, flags);

		spi = m->spi;
		conf = (struct omap2_mcspi_device_config *) spi->controller_data;
		cs = (struct omap2_mcspi_cs *) spi->controller_state;

		list_for_each_entry(t, &m->transfers, transfer_list) {
			if (t->tx_buf == NULL && t->rx_buf == NULL && t->len) {
				status = -EINVAL;
				break;
			}
			if (par_override || t->speed_hz || t->bits_per_word) {
				par_override = 1;
				status = omap2_mcspi_setup_transfer(spi, t);
				if (status < 0)
					break;
				if (!t->speed_hz && !t->bits_per_word)
					par_override = 0;
			}

			if (!cs_active) {
				omap2_mcspi_force_cs(spi, 1);
				cs_active = 1;
			}

			if (m->is_dma_mapped &&
			    (t->tx_dma != 0 || t->rx_dma != 0))
				omap2_mcspi_txrx_dma(spi, t);
			else
				omap2_mcspi_txrx_pio(spi, t);

			if (t->cs_change) {
				/* In the last transfer entry the flag means
				 * _leave_ CS on */
				if (t->transfer_list.next != &m->transfers)
					omap2_mcspi_force_cs(spi, 0);
				cs_active = 0;
			}
		}

		/* Restore defaults they are overriden */
		if (par_override) {
			par_override = 0;
			status = omap2_mcspi_setup_transfer(spi, NULL);
		}

		if (cs_active)
			omap2_mcspi_force_cs(spi, 0);

		m->status = status;
		m->complete(m->context);

		spin_lock_irqsave(&mcspi->lock, flags);
	}
	spin_unlock_irqrestore(&mcspi->lock, flags);
}

static int omap2_mcspi_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct omap2_mcspi	*mcspi;
	unsigned long		flags;

	m->actual_length = 0;
	m->status = 0;

	mcspi = class_get_devdata(&spi->master->cdev);

	spin_lock_irqsave(&mcspi->lock, flags);
	list_add_tail(&m->queue, &mcspi->msg_queue);
	spin_unlock_irqrestore(&mcspi->lock, flags);

	queue_work(omap2_mcspi_wq, &mcspi->work);

	return 0;
}

static int __devinit omap2_mcspi_reset(struct spi_master *master)
{
#if 0
	mcspi_write_reg(master, OMAP2_MCSPI_SYSCONFIG,
			OMAP2_MCSPI_SYSCONFIG_SOFTRESET);
	while (!(mcspi_read_reg(master, OMAP2_MCSPI_SYSSTATUS) & OMAP2_MCSPI_SYSSTATUS_RESETDONE));
#else
	return 0;
#endif
}

static int __devinit omap2_mcspi_probe(struct platform_device *pdev)
{
	struct spi_master		*master;
	struct omap2_mcspi_platform_config *pdata = pdev->dev.platform_data;
	struct omap2_mcspi		*mcspi;
	struct resource                 *r;
	int				status = 0, i;
		
	if (!pdata)
		return -EINVAL;

	master = spi_alloc_master(&pdev->dev, sizeof *mcspi);
	if (master == NULL) {
		dev_err(&pdev->dev, "master allocation failed\n");
		return -ENOMEM;
	}

	if (pdev->id != -1)
		master->bus_num = pdev->id;

	master->setup = omap2_mcspi_setup;
	master->transfer = omap2_mcspi_transfer;
	master->cleanup = omap2_mcspi_cleanup;
	master->num_chipselect = pdata->num_cs;

	if (class_device_get(&master->cdev) == NULL) {
		dev_err(&pdev->dev, "no master->cdev");
		status = -ENOMEM;
		goto err0;
	}

	dev_set_drvdata(&pdev->dev, master);

	mcspi = class_get_devdata(&master->cdev);
	mcspi->master = master;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		status = -ENODEV;
		goto err1;
	}
	
	mcspi->base = io_p2v(r->start);

	INIT_WORK(&mcspi->work, omap2_mcspi_work, mcspi);

	spin_lock_init(&mcspi->lock);
	INIT_LIST_HEAD(&mcspi->msg_queue);

	mcspi->ick = clk_get(&pdev->dev, "mcspi_ick");
	if (IS_ERR(mcspi->ick)) {
		dev_err(&pdev->dev, "can't get mcspi_ick\n");
		status = PTR_ERR(mcspi->ick);
		goto err1;
	}
	clk_enable(mcspi->ick);
	mcspi->fck = clk_get(&pdev->dev, "mcspi_fck");
	if (IS_ERR(mcspi->fck)) {
		dev_err(&pdev->dev, "can't get mcspi_fck\n");
		status = PTR_ERR(mcspi->fck);
		goto err2;
	}
	clk_enable(mcspi->fck);

	mcspi->dma_channels = 
		(struct omap2_mcspi_dma *)kzalloc(master->num_chipselect *
						  sizeof(struct omap2_mcspi_dma),
						  GFP_KERNEL);
	
	if (mcspi->dma_channels == NULL)
		goto err3;

	for (i = 0; i < master->num_chipselect; i++) {
		mcspi->dma_channels[i].dma_rx_channel = -1;
		mcspi->dma_channels[i].dma_tx_channel = -1;
	}

	if (omap2_mcspi_reset(master) < 0)
                goto err4;

	status = spi_register_master(master);
	if (status < 0)
		goto err4;

	return status;

err4:
	kfree(mcspi->dma_channels);
err3:
	clk_disable(mcspi->fck);
	clk_put(mcspi->fck);
err2:
	clk_disable(mcspi->ick);
	clk_put(mcspi->ick);
err1:
	class_device_put(&master->cdev);
err0:
	return status;
}

static int __devexit omap2_mcspi_remove(struct platform_device *pdev)
{
	struct spi_master		*master;
	struct omap2_mcspi		*mcspi;

	master = dev_get_drvdata(&pdev->dev);

	spi_unregister_master(master);
	mcspi = class_get_devdata(&master->cdev);
	clk_disable(mcspi->fck);
	clk_put(mcspi->fck);
	clk_disable(mcspi->ick);
	clk_put(mcspi->ick);
	class_device_put(&master->cdev);
	kfree(mcspi->dma_channels);

	return 0;
}

struct platform_driver omap2_mcspi_driver = {
	.driver = {
		.name =		"omap2_mcspi",
		.owner =	THIS_MODULE,
	},
	.probe =	omap2_mcspi_probe,
	.remove =	__devexit_p(omap2_mcspi_remove),
};
EXPORT_SYMBOL_GPL(omap2_mcspi_driver);


static int __init omap2_mcspi_init(void)
{
	printk(KERN_INFO "OMAP24xx McSPI driver initializing\n");
	omap2_mcspi_wq = create_workqueue("OMAP McSPI");
	if (omap2_mcspi_wq == NULL)
		return -1;
	return platform_driver_register(&omap2_mcspi_driver);
}
subsys_initcall(omap2_mcspi_init);

static void __exit omap2_mcspi_exit(void)
{
	platform_driver_unregister(&omap2_mcspi_driver);
}
module_exit(omap2_mcspi_exit);

MODULE_LICENSE("GPL");
