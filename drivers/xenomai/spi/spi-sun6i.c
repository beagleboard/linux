/**
 * I/O handling lifted from drivers/spi/spi-sun6i.c:
 * Copyright (C) 2012 - 2014 Allwinner Tech
 * Pan Nan <pannan@allwinnertech.com>
 * Copyright (C) 2014 Maxime Ripard
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * RTDM integration by:
 * Copyright (C) 2017 Philippe Gerum <rpm@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/reset.h>
#include "spi-master.h"

#define RTDM_SUBCLASS_SUN6I  2

#define SUN6I_GBL_CTL_REG		0x04
#define SUN6I_GBL_CTL_BUS_ENABLE	BIT(0)
#define SUN6I_GBL_CTL_MASTER		BIT(1)
#define SUN6I_GBL_CTL_TP		BIT(7)
#define SUN6I_GBL_CTL_RST		BIT(31)

#define SUN6I_TFR_CTL_REG		0x08
#define SUN6I_TFR_CTL_CPHA		BIT(0)
#define SUN6I_TFR_CTL_CPOL		BIT(1)
#define SUN6I_TFR_CTL_SPOL		BIT(2)
#define SUN6I_TFR_CTL_CS_MASK		0x30
#define SUN6I_TFR_CTL_CS(cs)		(((cs) << 4) & SUN6I_TFR_CTL_CS_MASK)
#define SUN6I_TFR_CTL_CS_MANUAL		BIT(6)
#define SUN6I_TFR_CTL_CS_LEVEL		BIT(7)
#define SUN6I_TFR_CTL_DHB		BIT(8)
#define SUN6I_TFR_CTL_FBS		BIT(12)
#define SUN6I_TFR_CTL_XCH		BIT(31)

#define SUN6I_INT_CTL_REG		0x10
#define SUN6I_INT_CTL_RX_RDY		BIT(0)
#define SUN6I_INT_CTL_TX_RDY		BIT(4)
#define SUN6I_INT_CTL_RX_OVF		BIT(8)
#define SUN6I_INT_CTL_TC		BIT(12)

#define SUN6I_INT_STA_REG		0x14

#define SUN6I_FIFO_CTL_REG		0x18
#define SUN6I_FIFO_CTL_RX_RDY_TRIG_LEVEL_MASK	0xff
#define SUN6I_FIFO_CTL_RX_RDY_TRIG_LEVEL_BITS	0
#define SUN6I_FIFO_CTL_RX_RST			BIT(15)
#define SUN6I_FIFO_CTL_TX_RDY_TRIG_LEVEL_MASK	0xff
#define SUN6I_FIFO_CTL_TX_RDY_TRIG_LEVEL_BITS	16
#define SUN6I_FIFO_CTL_TX_RST			BIT(31)

#define SUN6I_FIFO_STA_REG		0x1c
#define SUN6I_FIFO_STA_RX_CNT(reg)	(((reg) >> 0) & 0xff)
#define SUN6I_FIFO_STA_TX_CNT(reg)	(((reg) >> 16) & 0xff)

#define SUN6I_CLK_CTL_REG		0x24
#define SUN6I_CLK_CTL_CDR2_MASK		0xff
#define SUN6I_CLK_CTL_CDR2(div)		(((div) & SUN6I_CLK_CTL_CDR2_MASK) << 0)
#define SUN6I_CLK_CTL_CDR1_MASK		0xf
#define SUN6I_CLK_CTL_CDR1(div)		(((div) & SUN6I_CLK_CTL_CDR1_MASK) << 8)
#define SUN6I_CLK_CTL_DRS		BIT(12)

#define SUN6I_MAX_XFER_SIZE		0xffffff

#define SUN6I_BURST_CNT_REG		0x30
#define SUN6I_BURST_CNT(cnt)		((cnt) & SUN6I_MAX_XFER_SIZE)

#define SUN6I_XMIT_CNT_REG		0x34
#define SUN6I_XMIT_CNT(cnt)		((cnt) & SUN6I_MAX_XFER_SIZE)

#define SUN6I_BURST_CTL_CNT_REG		0x38
#define SUN6I_BURST_CTL_CNT_STC(cnt)	((cnt) & SUN6I_MAX_XFER_SIZE)

#define SUN6I_TXDATA_REG		0x200
#define SUN6I_RXDATA_REG		0x300

#define SUN6I_SPI_MODE_BITS	(SPI_CPOL | SPI_CPHA | SPI_CS_HIGH	\
				 | SPI_LSB_FIRST)

	struct spi_setup_data {
		int fifo_depth;
	};

static struct spi_setup_data sun6i_data = {
	.fifo_depth = 128,
};

static struct spi_setup_data sun8i_data = {
	.fifo_depth = 64,
};

struct spi_master_sun6i {
	struct rtdm_spi_master master;
	void __iomem *regs;
	struct reset_control *rstc;
	struct clk *hclk;
	struct clk *mclk;
	unsigned long clk_hz;
	rtdm_irq_t irqh;
	const u8 *tx_buf;
	u8 *rx_buf;
	int tx_len;
	int rx_len;
	rtdm_event_t transfer_done;
	const struct spi_setup_data *setup;
};

struct spi_slave_sun6i {
	struct rtdm_spi_remote_slave slave;
	void *io_virt;
	dma_addr_t io_dma;
	size_t io_len;
};

static inline struct spi_slave_sun6i *
to_slave_sun6i(struct rtdm_spi_remote_slave *slave)
{
	return container_of(slave, struct spi_slave_sun6i, slave);
}

static inline struct spi_master_sun6i *
to_master_sun6i(struct rtdm_spi_remote_slave *slave)
{
	return container_of(slave->master, struct spi_master_sun6i, master);
}

static inline struct device *
master_to_kdev(struct rtdm_spi_master *master)
{
	return &master->kmaster->dev;
}

static inline u32 sun6i_rd(struct spi_master_sun6i *spim,
			   unsigned int reg)
{
	return readl(spim->regs + reg);
}

static inline void sun6i_wr(struct spi_master_sun6i *spim,
			    unsigned int reg, u32 val)
{
	writel(val, spim->regs + reg);
}

static void sun6i_rd_fifo(struct spi_master_sun6i *spim)
{
	u32 reg;
	int len;
	u8 byte;

	reg = sun6i_rd(spim, SUN6I_FIFO_STA_REG);
	len = min((int)SUN6I_FIFO_STA_RX_CNT(reg), spim->rx_len);

	while (len-- > 0) {
		byte = sun6i_rd(spim, SUN6I_RXDATA_REG);
		if (spim->rx_buf)
			*spim->rx_buf++ = byte;
		spim->rx_len--;
	}
}

static void sun6i_wr_fifo(struct spi_master_sun6i *spim)
{
	u32 reg;
	int len;
	u8 byte;

	reg = sun6i_rd(spim, SUN6I_FIFO_STA_REG);
	len = min(spim->setup->fifo_depth - (int)SUN6I_FIFO_STA_TX_CNT(reg),
		  spim->tx_len);
	
	while (len-- > 0) {
		byte = spim->tx_buf ? *spim->tx_buf++ : 0;
		sun6i_wr(spim, SUN6I_TXDATA_REG, byte);
		spim->tx_len--;
	}
}

static int sun6i_spi_interrupt(rtdm_irq_t *irqh)
{
	struct spi_master_sun6i *spim;
	u32 status;

	spim = rtdm_irq_get_arg(irqh, struct spi_master_sun6i);

	sun6i_rd_fifo(spim);
	sun6i_wr_fifo(spim);
	
	status = sun6i_rd(spim, SUN6I_INT_STA_REG);
	if ((status & SUN6I_INT_CTL_TC)) {
		sun6i_wr(spim, SUN6I_INT_STA_REG, SUN6I_INT_CTL_TC);
		sun6i_wr(spim, SUN6I_INT_CTL_REG, 0);
		rtdm_event_signal(&spim->transfer_done);
	} else if (status & SUN6I_INT_CTL_TX_RDY)
		sun6i_wr(spim, SUN6I_INT_STA_REG, SUN6I_INT_CTL_TX_RDY);

	return RTDM_IRQ_HANDLED;
}

static int sun6i_configure(struct rtdm_spi_remote_slave *slave)
{
	struct spi_master_sun6i *spim = to_master_sun6i(slave);
	struct rtdm_spi_config *config = &slave->config;
	u32 reg, div;
	
	/* Set clock polarity and phase. */

	reg = sun6i_rd(spim, SUN6I_TFR_CTL_REG);
	reg &= ~(SUN6I_TFR_CTL_CPOL | SUN6I_TFR_CTL_CPHA |
		 SUN6I_TFR_CTL_FBS | SUN6I_TFR_CTL_SPOL);

	/* Manual CS via ->chip_select(). */
	reg |= SUN6I_TFR_CTL_CS_MANUAL;

	if (config->mode & SPI_CPOL)
		reg |= SUN6I_TFR_CTL_CPOL;

	if (config->mode & SPI_CPHA)
		reg |= SUN6I_TFR_CTL_CPHA;

	if (config->mode & SPI_LSB_FIRST)
		reg |= SUN6I_TFR_CTL_FBS;

	if (!(config->mode & SPI_CS_HIGH))
		reg |= SUN6I_TFR_CTL_SPOL;

	sun6i_wr(spim, SUN6I_TFR_CTL_REG, reg);
	
	/* Setup clock divider. */

	div = spim->clk_hz / (2 * config->speed_hz);
	if (div <= SUN6I_CLK_CTL_CDR2_MASK + 1) {
		if (div > 0)
			div--;
		reg = SUN6I_CLK_CTL_CDR2(div) | SUN6I_CLK_CTL_DRS;
	} else {
		div = ilog2(spim->clk_hz) - ilog2(config->speed_hz);
		reg = SUN6I_CLK_CTL_CDR1(div);
	}

	sun6i_wr(spim, SUN6I_CLK_CTL_REG, reg);

	return 0;
}

static void sun6i_chip_select(struct rtdm_spi_remote_slave *slave,
			      bool active)
{
	struct spi_master_sun6i *spim = to_master_sun6i(slave);
	u32 reg;

	/*
	 * We have no cs_gpios, so this handler will be called for
	 * each transfer.
	 */
	reg = sun6i_rd(spim, SUN6I_TFR_CTL_REG);
	reg &= ~(SUN6I_TFR_CTL_CS_MASK | SUN6I_TFR_CTL_CS_LEVEL);
	reg |= SUN6I_TFR_CTL_CS(slave->chip_select);

	if (active)
		reg |= SUN6I_TFR_CTL_CS_LEVEL;

	sun6i_wr(spim, SUN6I_TFR_CTL_REG, reg);
}

static int do_transfer_irq(struct rtdm_spi_remote_slave *slave)
{
	struct spi_master_sun6i *spim = to_master_sun6i(slave);
	u32 tx_len = 0, reg;
	int ret;

	/* Reset FIFO. */
	sun6i_wr(spim, SUN6I_FIFO_CTL_REG,
		 SUN6I_FIFO_CTL_RX_RST | SUN6I_FIFO_CTL_TX_RST);

	/* Set FIFO interrupt trigger level to 3/4 of the fifo depth. */
	reg = spim->setup->fifo_depth / 4 * 3;
	sun6i_wr(spim, SUN6I_FIFO_CTL_REG,
		 (reg << SUN6I_FIFO_CTL_RX_RDY_TRIG_LEVEL_BITS) |
		 (reg << SUN6I_FIFO_CTL_TX_RDY_TRIG_LEVEL_BITS));

	reg = sun6i_rd(spim, SUN6I_TFR_CTL_REG);
	reg &= ~SUN6I_TFR_CTL_DHB;
	/* Discard unused SPI bursts if TX only. */
	if (spim->rx_buf == NULL)
		reg |= SUN6I_TFR_CTL_DHB;
	sun6i_wr(spim, SUN6I_TFR_CTL_REG, reg);

	if (spim->tx_buf)
		tx_len = spim->tx_len;

	/* Setup the counters. */
	sun6i_wr(spim, SUN6I_BURST_CNT_REG, SUN6I_BURST_CNT(spim->tx_len));
	sun6i_wr(spim, SUN6I_XMIT_CNT_REG, SUN6I_XMIT_CNT(tx_len));
	sun6i_wr(spim, SUN6I_BURST_CTL_CNT_REG,
		 SUN6I_BURST_CTL_CNT_STC(tx_len));

	/* Fill the TX FIFO */
	sun6i_wr_fifo(spim);

	/* Enable interrupts. */
	reg = sun6i_rd(spim, SUN6I_INT_CTL_REG);
	reg |= SUN6I_INT_CTL_TC | SUN6I_INT_CTL_TX_RDY;
	sun6i_wr(spim, SUN6I_INT_CTL_REG, reg);

	/* Start the transfer. */
	reg = sun6i_rd(spim, SUN6I_TFR_CTL_REG);
	sun6i_wr(spim, SUN6I_TFR_CTL_REG, reg | SUN6I_TFR_CTL_XCH);
	
	ret = rtdm_event_wait(&spim->transfer_done);
	if (ret) {
		sun6i_wr(spim, SUN6I_INT_CTL_REG, 0);
		return ret;
	}

	return 0;
}

static int sun6i_transfer_iobufs(struct rtdm_spi_remote_slave *slave)
{
	struct spi_master_sun6i *spim = to_master_sun6i(slave);
	struct spi_slave_sun6i *sun6i = to_slave_sun6i(slave);

	if (sun6i->io_len == 0)
		return -EINVAL;	/* No I/O buffers set. */
	
	spim->tx_len = sun6i->io_len / 2;
	spim->rx_len = spim->tx_len;
	spim->tx_buf = sun6i->io_virt + spim->rx_len;
	spim->rx_buf = sun6i->io_virt;

	return do_transfer_irq(slave);
}

static int sun6i_transfer_iobufs_n(struct rtdm_spi_remote_slave *slave,
				   int len)
{
	struct spi_master_sun6i *spim = to_master_sun6i(slave);
	struct spi_slave_sun6i *sun6i = to_slave_sun6i(slave);

	if ((sun6i->io_len == 0) ||
		(len <= 0) || (len > (sun6i->io_len / 2)))
		return -EINVAL;

	spim->tx_len = len;
	spim->rx_len = len;
	spim->tx_buf = sun6i->io_virt + sun6i->io_len / 2;
	spim->rx_buf = sun6i->io_virt;

	return do_transfer_irq(slave);
}

static ssize_t sun6i_read(struct rtdm_spi_remote_slave *slave,
			  void *rx, size_t len)
{
	struct spi_master_sun6i *spim = to_master_sun6i(slave);

	spim->tx_len = len;
	spim->rx_len = len;
	spim->tx_buf = NULL;
	spim->rx_buf = rx;

	return do_transfer_irq(slave) ?: len;
}

static ssize_t sun6i_write(struct rtdm_spi_remote_slave *slave,
			   const void *tx, size_t len)
{
	struct spi_master_sun6i *spim = to_master_sun6i(slave);

	spim->tx_len = len;
	spim->rx_len = len;
	spim->tx_buf = tx;
	spim->rx_buf = NULL;

	return do_transfer_irq(slave) ?: len;
}

static int set_iobufs(struct spi_slave_sun6i *sun6i, size_t len)
{
	dma_addr_t dma;
	void *p;

	if (len == 0)
		return -EINVAL;
	
	len = L1_CACHE_ALIGN(len) * 2;
	if (len == sun6i->io_len)
		return 0;

	if (sun6i->io_len)
		return -EINVAL;	/* I/O buffers may not be resized. */

	p = dma_alloc_coherent(NULL, len, &dma, GFP_KERNEL);
	if (p == NULL)
		return -ENOMEM;

	sun6i->io_dma = dma;
	sun6i->io_virt = p;
	smp_mb();
	sun6i->io_len = len;
	
	return 0;
}

static int sun6i_set_iobufs(struct rtdm_spi_remote_slave *slave,
			    struct rtdm_spi_iobufs *p)
{
	struct spi_slave_sun6i *sun6i = to_slave_sun6i(slave);
	int ret;

	ret = set_iobufs(sun6i, p->io_len);
	if (ret)
		return ret;

	p->i_offset = 0;
	p->o_offset = sun6i->io_len / 2;
	p->map_len = sun6i->io_len;
	
	return 0;
}

static int sun6i_mmap_iobufs(struct rtdm_spi_remote_slave *slave,
			     struct vm_area_struct *vma)
{
	struct spi_slave_sun6i *sun6i = to_slave_sun6i(slave);

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	return rtdm_mmap_kmem(vma, sun6i->io_virt);
}

static void sun6i_mmap_release(struct rtdm_spi_remote_slave *slave)
{
	struct spi_slave_sun6i *sun6i = to_slave_sun6i(slave);

	dma_free_coherent(NULL, sun6i->io_len,
			  sun6i->io_virt, sun6i->io_dma);
	sun6i->io_len = 0;
}

static struct rtdm_spi_remote_slave *
sun6i_attach_slave(struct rtdm_spi_master *master, struct spi_device *spi)
{
	struct spi_slave_sun6i *sun6i;
	int ret;

	sun6i = kzalloc(sizeof(*sun6i), GFP_KERNEL);
	if (sun6i == NULL)
		return ERR_PTR(-ENOMEM);

	ret = rtdm_spi_add_remote_slave(&sun6i->slave, master, spi);
	if (ret) {
		dev_err(&spi->dev,
			"%s: failed to attach slave\n", __func__);
		kfree(sun6i);
		return ERR_PTR(ret);
	}

	return &sun6i->slave;
}

static void sun6i_detach_slave(struct rtdm_spi_remote_slave *slave)
{
	struct spi_slave_sun6i *sun6i = to_slave_sun6i(slave);

	rtdm_spi_remove_remote_slave(slave);
	kfree(sun6i);
}

static struct rtdm_spi_master_ops sun6i_master_ops = {
	.configure = sun6i_configure,
	.chip_select = sun6i_chip_select,
	.set_iobufs = sun6i_set_iobufs,
	.mmap_iobufs = sun6i_mmap_iobufs,
	.mmap_release = sun6i_mmap_release,
	.transfer_iobufs = sun6i_transfer_iobufs,
	.transfer_iobufs_n = sun6i_transfer_iobufs_n,
	.write = sun6i_write,
	.read = sun6i_read,
	.attach_slave = sun6i_attach_slave,
	.detach_slave = sun6i_detach_slave,
};

static int sun6i_spi_probe(struct platform_device *pdev)
{
	struct rtdm_spi_master *master;
	struct spi_master_sun6i *spim;
	struct spi_master *kmaster;
	struct resource *r;
	int ret, irq;
	u32 clk_rate;

	dev_dbg(&pdev->dev, "%s: entered\n", __func__);

	master = rtdm_spi_alloc_master(&pdev->dev,
				       struct spi_master_sun6i, master);
	if (master == NULL)
		return -ENOMEM;

	master->subclass = RTDM_SUBCLASS_SUN6I;
	master->ops = &sun6i_master_ops;
	platform_set_drvdata(pdev, master);

	kmaster = master->kmaster;
	kmaster->max_speed_hz = 100 * 1000 * 1000;
	kmaster->min_speed_hz = 3 * 1000;
	kmaster->mode_bits = SUN6I_SPI_MODE_BITS;
	kmaster->bits_per_word_mask = SPI_BPW_MASK(8);
	kmaster->num_chipselect = 4;
	kmaster->dev.of_node = pdev->dev.of_node;

	spim = container_of(master, struct spi_master_sun6i, master);
	spim->setup = of_device_get_match_data(&pdev->dev);

	rtdm_event_init(&spim->transfer_done, 0);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	spim->regs = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(spim->regs)) {
		dev_err(&pdev->dev, "%s: cannot map I/O memory\n", __func__);
		ret = PTR_ERR(spim->regs);
		goto fail;
	}
	
	spim->hclk = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(spim->hclk)) {
		dev_err(&pdev->dev, "Unable to acquire AHB clock\n");
		ret = PTR_ERR(spim->hclk);
		goto fail;
	}

	spim->mclk = devm_clk_get(&pdev->dev, "mod");
	if (IS_ERR(spim->mclk)) {
		dev_err(&pdev->dev, "Unable to acquire MOD clock\n");
		ret = PTR_ERR(spim->mclk);
		goto fail;
	}

	spim->rstc = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(spim->rstc)) {
		dev_err(&pdev->dev, "Couldn't get reset controller\n");
		ret = PTR_ERR(spim->rstc);
		goto fail;
	}

	/*
	 * Ensure that we have a parent clock fast enough to handle
	 * the fastest transfers properly.
	 */
	clk_rate = clk_get_rate(spim->mclk);
	if (clk_rate < 2 * kmaster->max_speed_hz)
		clk_set_rate(spim->mclk, 2 * kmaster->max_speed_hz);

	spim->clk_hz = clk_get_rate(spim->mclk);

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (irq <= 0) {
		ret = irq ?: -ENODEV;
		goto fail;
	}

	clk_prepare_enable(spim->hclk);
	clk_prepare_enable(spim->mclk);

	ret = reset_control_deassert(spim->rstc);
	if (ret)
		goto fail_unclk;

	/* Enable SPI module, in master mode with smart burst. */

	sun6i_wr(spim, SUN6I_GBL_CTL_REG,
		 SUN6I_GBL_CTL_BUS_ENABLE | SUN6I_GBL_CTL_MASTER |
		 SUN6I_GBL_CTL_TP);

	/* Disable and clear all interrupts. */
	sun6i_wr(spim, SUN6I_INT_CTL_REG, 0);
	sun6i_wr(spim, SUN6I_INT_STA_REG, ~0);
	
	ret = rtdm_irq_request(&spim->irqh, irq,
			       sun6i_spi_interrupt, 0,
			       dev_name(&pdev->dev), spim);
	if (ret) {
		dev_err(&pdev->dev, "%s: cannot request IRQ%d\n",
			__func__, irq);
		goto fail_unclk;
	}

	ret = rtdm_spi_add_master(&spim->master);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to add master\n",
			__func__);
		goto fail_register;
	}

	return 0;

fail_register:
	rtdm_irq_free(&spim->irqh);
fail_unclk:
	clk_disable_unprepare(spim->mclk);
	clk_disable_unprepare(spim->hclk);
fail:
	spi_master_put(kmaster);

	return ret;
}

static int sun6i_spi_remove(struct platform_device *pdev)
{
	struct rtdm_spi_master *master = platform_get_drvdata(pdev);
	struct spi_master_sun6i *spim;

	dev_dbg(&pdev->dev, "%s: entered\n", __func__);

	spim = container_of(master, struct spi_master_sun6i, master);

	rtdm_irq_free(&spim->irqh);

	clk_disable_unprepare(spim->mclk);
	clk_disable_unprepare(spim->hclk);

	rtdm_spi_remove_master(master);

	return 0;
}

static const struct of_device_id sun6i_spi_match[] = {
	{
		.compatible = "allwinner,sun6i-a31-spi",
		.data = &sun6i_data,
	},
	{
		.compatible = "allwinner,sun8i-h3-spi",
		.data = &sun8i_data,
	},
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, sun6i_spi_match);

static struct platform_driver sun6i_spi_driver = {
	.driver		= {
		.name		= "spi-sun6i",
		.of_match_table	= sun6i_spi_match,
	},
	.probe		= sun6i_spi_probe,
	.remove		= sun6i_spi_remove,
};
module_platform_driver(sun6i_spi_driver);

MODULE_LICENSE("GPL");
