/**
 * I/O handling lifted from drivers/spi/spi-bcm2835.c:
 * Copyright (C) 2012 Chris Boot
 * Copyright (C) 2013 Stephen Warren
 * Copyright (C) 2015 Martin Sperl
 *
 * RTDM integration by:
 * Copyright (C) 2016 Philippe Gerum <rpm@xenomai.org>
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
#include <linux/of_gpio.h>
#include "spi-master.h"

#define RTDM_SUBCLASS_BCM2835  1

/* SPI register offsets */
#define BCM2835_SPI_CS			0x00
#define BCM2835_SPI_FIFO		0x04
#define BCM2835_SPI_CLK			0x08
#define BCM2835_SPI_DLEN		0x0c
#define BCM2835_SPI_LTOH		0x10
#define BCM2835_SPI_DC			0x14

/* Bitfields in CS */
#define BCM2835_SPI_CS_LEN_LONG		0x02000000
#define BCM2835_SPI_CS_DMA_LEN		0x01000000
#define BCM2835_SPI_CS_CSPOL2		0x00800000
#define BCM2835_SPI_CS_CSPOL1		0x00400000
#define BCM2835_SPI_CS_CSPOL0		0x00200000
#define BCM2835_SPI_CS_RXF		0x00100000
#define BCM2835_SPI_CS_RXR		0x00080000
#define BCM2835_SPI_CS_TXD		0x00040000
#define BCM2835_SPI_CS_RXD		0x00020000
#define BCM2835_SPI_CS_DONE		0x00010000
#define BCM2835_SPI_CS_LEN		0x00002000
#define BCM2835_SPI_CS_REN		0x00001000
#define BCM2835_SPI_CS_ADCS		0x00000800
#define BCM2835_SPI_CS_INTR		0x00000400
#define BCM2835_SPI_CS_INTD		0x00000200
#define BCM2835_SPI_CS_DMAEN		0x00000100
#define BCM2835_SPI_CS_TA		0x00000080
#define BCM2835_SPI_CS_CSPOL		0x00000040
#define BCM2835_SPI_CS_CLEAR_RX		0x00000020
#define BCM2835_SPI_CS_CLEAR_TX		0x00000010
#define BCM2835_SPI_CS_CPOL		0x00000008
#define BCM2835_SPI_CS_CPHA		0x00000004
#define BCM2835_SPI_CS_CS_10		0x00000002
#define BCM2835_SPI_CS_CS_01		0x00000001

#define BCM2835_SPI_POLLING_LIMIT_US	30
#define BCM2835_SPI_POLLING_JIFFIES	2
#define BCM2835_SPI_DMA_MIN_LENGTH	96
#define BCM2835_SPI_MODE_BITS	(SPI_CPOL | SPI_CPHA | SPI_CS_HIGH \
				| SPI_NO_CS | SPI_3WIRE)

struct spi_master_bcm2835 {
	struct rtdm_spi_master master;
	void __iomem *regs;
	struct clk *clk;
	unsigned long clk_hz;
	rtdm_irq_t irqh;
	const u8 *tx_buf;
	u8 *rx_buf;
	int tx_len;
	int rx_len;
	rtdm_event_t transfer_done;
};

struct spi_slave_bcm2835 {
	struct rtdm_spi_remote_slave slave;
	void *io_virt;
	dma_addr_t io_dma;
	size_t io_len;
};

static inline struct spi_slave_bcm2835 *
to_slave_bcm2835(struct rtdm_spi_remote_slave *slave)
{
	return container_of(slave, struct spi_slave_bcm2835, slave);
}

static inline struct spi_master_bcm2835 *
to_master_bcm2835(struct rtdm_spi_remote_slave *slave)
{
	return container_of(slave->master, struct spi_master_bcm2835, master);
}

static inline struct device *
master_to_kdev(struct rtdm_spi_master *master)
{
	return &master->kmaster->dev;
}

static inline u32 bcm2835_rd(struct spi_master_bcm2835 *spim,
			     unsigned int reg)
{
	return readl(spim->regs + reg);
}

static inline void bcm2835_wr(struct spi_master_bcm2835 *spim,
			      unsigned int reg, u32 val)
{
	writel(val, spim->regs + reg);
}

static inline void bcm2835_rd_fifo(struct spi_master_bcm2835 *spim)
{
	u8 byte;

	while (spim->rx_len > 0 &&
	       (bcm2835_rd(spim, BCM2835_SPI_CS) & BCM2835_SPI_CS_RXD)) {
		byte = bcm2835_rd(spim, BCM2835_SPI_FIFO);
		if (spim->rx_buf)
			*spim->rx_buf++ = byte;
		spim->rx_len--;
	}
}

static inline void bcm2835_wr_fifo(struct spi_master_bcm2835 *spim)
{
	u8 byte;

	while (spim->tx_len > 0 &&
	       (bcm2835_rd(spim, BCM2835_SPI_CS) & BCM2835_SPI_CS_TXD)) {
		byte = spim->tx_buf ? *spim->tx_buf++ : 0;
		bcm2835_wr(spim, BCM2835_SPI_FIFO, byte);
		spim->tx_len--;
	}
}

static void bcm2835_reset_hw(struct spi_master_bcm2835 *spim)
{
	u32 cs = bcm2835_rd(spim, BCM2835_SPI_CS);

	cs &= ~(BCM2835_SPI_CS_INTR |
		BCM2835_SPI_CS_INTD |
		BCM2835_SPI_CS_DMAEN |
		BCM2835_SPI_CS_TA);
	cs |= BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX;

	/* Reset the SPI block. */
	bcm2835_wr(spim, BCM2835_SPI_CS, cs);
	bcm2835_wr(spim, BCM2835_SPI_DLEN, 0);
}

static int bcm2835_spi_interrupt(rtdm_irq_t *irqh)
{
	struct spi_master_bcm2835 *spim;

	spim = rtdm_irq_get_arg(irqh, struct spi_master_bcm2835);

	bcm2835_rd_fifo(spim);
	bcm2835_wr_fifo(spim);

	if (bcm2835_rd(spim, BCM2835_SPI_CS) & BCM2835_SPI_CS_DONE) {
		bcm2835_reset_hw(spim);
		rtdm_event_signal(&spim->transfer_done);
	}

	return RTDM_IRQ_HANDLED;
}

static int bcm2835_configure(struct rtdm_spi_remote_slave *slave)
{
	struct spi_master_bcm2835 *spim = to_master_bcm2835(slave);
	struct rtdm_spi_config *config = &slave->config;
	unsigned long spi_hz, cdiv;
	u32 cs;

	/* Set clock polarity and phase. */

	cs = bcm2835_rd(spim, BCM2835_SPI_CS);

	cs &= ~(BCM2835_SPI_CS_CPOL | BCM2835_SPI_CS_CPHA);
	if (config->mode & SPI_CPOL)
		cs |= BCM2835_SPI_CS_CPOL;
	if (config->mode & SPI_CPHA)
		cs |= BCM2835_SPI_CS_CPHA;

	bcm2835_wr(spim, BCM2835_SPI_CS, cs);
	
	/* Set clock frequency. */

	spi_hz = config->speed_hz;

	/*
	 * Fastest clock rate is of the APB clock, which is close to
	 * clk_hz / 2.
	 */
	if (spi_hz >= spim->clk_hz / 2)
		cdiv = 2;
	else if (spi_hz) {
		cdiv = DIV_ROUND_UP(spim->clk_hz, spi_hz); /* Multiple of 2. */
		cdiv += (cdiv % 2);
		if (cdiv >= 65536)
			cdiv = 0;
	} else
		cdiv = 0;

	bcm2835_wr(spim, BCM2835_SPI_CLK, cdiv);
	
	return 0;
}

static void bcm2835_chip_select(struct rtdm_spi_remote_slave *slave,
				bool active)
{
	struct spi_master_bcm2835 *spim = to_master_bcm2835(slave);
	struct rtdm_spi_config *config = &slave->config;
	u32 cs;

	cs = bcm2835_rd(spim, BCM2835_SPI_CS);

	if (config->mode & SPI_CS_HIGH) {
		cs |= BCM2835_SPI_CS_CSPOL;
		cs |= BCM2835_SPI_CS_CSPOL0 << slave->chip_select;
	} else {
		cs &= ~BCM2835_SPI_CS_CSPOL;
		cs &= ~(BCM2835_SPI_CS_CSPOL0 << slave->chip_select);
	}

	/* "active" is the logical state, not the impedance level. */

	if (active) {
		if (config->mode & SPI_NO_CS)
			cs |= BCM2835_SPI_CS_CS_10 | BCM2835_SPI_CS_CS_01;
		else {
			cs &= ~(BCM2835_SPI_CS_CS_10 | BCM2835_SPI_CS_CS_01);
			cs |= slave->chip_select;
		}
	} else {
		/* Put HW-CS into deselected state. */
		cs &= ~BCM2835_SPI_CS_CSPOL;
		/* Use the "undefined" chip-select as precaution. */
		cs |= BCM2835_SPI_CS_CS_10 | BCM2835_SPI_CS_CS_01;
	}

	bcm2835_wr(spim, BCM2835_SPI_CS, cs);
}

static int do_transfer_irq(struct rtdm_spi_remote_slave *slave)
{
	struct spi_master_bcm2835 *spim = to_master_bcm2835(slave);
	int ret;
	u32 cs;
	
	cs = bcm2835_rd(spim, BCM2835_SPI_CS);

	cs &= ~BCM2835_SPI_CS_REN;
	if ((slave->config.mode & SPI_3WIRE) && spim->rx_buf)
		cs |= BCM2835_SPI_CS_REN;

	cs |= BCM2835_SPI_CS_TA;

	/*
	 * Fill in fifo if we have gpio-cs note that there have been
	 * rare events where the native-CS flapped for <1us which may
	 * change the behaviour with gpio-cs this does not happen, so
	 * it is implemented only for this case.
	 */
	if (gpio_is_valid(slave->cs_gpio)) {
		/* Set dummy CS, ->chip_select() was not called. */
		cs |= BCM2835_SPI_CS_CS_10 | BCM2835_SPI_CS_CS_01;
		/* Enable SPI block, before filling FIFO. */
		bcm2835_wr(spim, BCM2835_SPI_CS, cs);
		bcm2835_wr_fifo(spim);
	}

	/* Enable interrupts last, wait for transfer completion. */
	cs |= BCM2835_SPI_CS_INTR | BCM2835_SPI_CS_INTD;
	bcm2835_wr(spim, BCM2835_SPI_CS, cs);

	ret = rtdm_event_wait(&spim->transfer_done);
	if (ret) {
		bcm2835_reset_hw(spim);
		return ret;
	}

	return 0;
}

static int bcm2835_transfer_iobufs(struct rtdm_spi_remote_slave *slave)
{
	struct spi_master_bcm2835 *spim = to_master_bcm2835(slave);
	struct spi_slave_bcm2835 *bcm = to_slave_bcm2835(slave);

	if (bcm->io_len == 0)
		return -EINVAL;	/* No I/O buffers set. */
	
	spim->tx_len = bcm->io_len / 2;
	spim->rx_len = spim->tx_len;
	spim->tx_buf = bcm->io_virt + spim->rx_len;
	spim->rx_buf = bcm->io_virt;

	return do_transfer_irq(slave);
}

static int bcm2835_transfer_iobufs_n(struct rtdm_spi_remote_slave *slave,
				     int len)
{
	struct spi_master_bcm2835 *spim = to_master_bcm2835(slave);
	struct spi_slave_bcm2835 *bcm = to_slave_bcm2835(slave);

	if ((bcm->io_len == 0) ||
		(len <= 0) || (len > (bcm->io_len / 2)))
		return -EINVAL;

	spim->tx_len = len;
	spim->rx_len = len;
	spim->tx_buf = bcm->io_virt + bcm->io_len / 2;
	spim->rx_buf = bcm->io_virt;

	return do_transfer_irq(slave);
}

static ssize_t bcm2835_read(struct rtdm_spi_remote_slave *slave,
			    void *rx, size_t len)
{
	struct spi_master_bcm2835 *spim = to_master_bcm2835(slave);

	spim->tx_len = len;
	spim->rx_len = len;
	spim->tx_buf = NULL;
	spim->rx_buf = rx;

	return do_transfer_irq(slave) ?: len;
}

static ssize_t bcm2835_write(struct rtdm_spi_remote_slave *slave,
			     const void *tx, size_t len)
{
	struct spi_master_bcm2835 *spim = to_master_bcm2835(slave);

	spim->tx_len = len;
	spim->rx_len = len;
	spim->tx_buf = tx;
	spim->rx_buf = NULL;

	return do_transfer_irq(slave) ?: len;
}

static int set_iobufs(struct spi_slave_bcm2835 *bcm, size_t len)
{
	dma_addr_t dma;
	void *p;

	if (len == 0)
		return -EINVAL;
	
	len = L1_CACHE_ALIGN(len) * 2;
	if (len == bcm->io_len)
		return 0;

	if (bcm->io_len)
		return -EINVAL;	/* I/O buffers may not be resized. */

	/*
	 * Since we need the I/O buffers to be set for starting a
	 * transfer, there is no need for serializing this routine and
	 * transfer_iobufs(), provided io_len is set last.
	 *
	 * NOTE: We don't need coherent memory until we actually get
	 * DMA transfers working, this code is a bit ahead of
	 * schedule.
	 *
	 * Revisit: this assumes DMA mask is 4Gb.
	 */
	p = dma_alloc_coherent(NULL, len, &dma, GFP_KERNEL);
	if (p == NULL)
		return -ENOMEM;

	bcm->io_dma = dma;
	bcm->io_virt = p;
	smp_mb();
	/*
	 * May race with transfer_iobufs(), must be assigned after all
	 * the rest is set up, enforcing a membar.
	 */
	bcm->io_len = len;
	
	return 0;
}

static int bcm2835_set_iobufs(struct rtdm_spi_remote_slave *slave,
			      struct rtdm_spi_iobufs *p)
{
	struct spi_slave_bcm2835 *bcm = to_slave_bcm2835(slave);
	int ret;

	ret = set_iobufs(bcm, p->io_len);
	if (ret)
		return ret;

	p->i_offset = 0;
	p->o_offset = bcm->io_len / 2;
	p->map_len = bcm->io_len;
	
	return 0;
}

static int bcm2835_mmap_iobufs(struct rtdm_spi_remote_slave *slave,
			       struct vm_area_struct *vma)
{
	struct spi_slave_bcm2835 *bcm = to_slave_bcm2835(slave);

	/*
	 * dma_alloc_coherent() delivers non-cached memory, make sure
	 * to return consistent mapping attributes. Typically, mixing
	 * memory attributes across address spaces referring to the
	 * same physical area is architecturally wrong on ARM.
	 */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	return rtdm_mmap_kmem(vma, bcm->io_virt);
}

static void bcm2835_mmap_release(struct rtdm_spi_remote_slave *slave)
{
	struct spi_slave_bcm2835 *bcm = to_slave_bcm2835(slave);

	dma_free_coherent(NULL, bcm->io_len,
			  bcm->io_virt, bcm->io_dma);
	bcm->io_len = 0;
}

static int gpio_match_name(struct gpio_chip *chip, void *data)
{
	return !strcmp(chip->label, data);
}

static int find_cs_gpio(struct spi_device *spi)
{
	struct spi_master *kmaster = spi->master;
	u32 pingroup_index, pin, pin_index;
	struct device_node *pins;
	struct gpio_chip *chip;
	int ret;

	if (gpio_is_valid(spi->cs_gpio)) {
		dev_info(&spi->dev, "using GPIO%i for CS%d\n",
			 spi->cs_gpio, spi->chip_select);
		return 0;
	}

	/* Translate native CS to GPIO. */

	for (pingroup_index = 0;
	     (pins = of_parse_phandle(kmaster->dev.of_node,
		     "pinctrl-0", pingroup_index)) != 0; pingroup_index++) {
		for (pin_index = 0;
		     of_property_read_u32_index(pins, "brcm,pins",
				pin_index, &pin) == 0; pin_index++) {
			if ((spi->chip_select == 0 &&
			     (pin == 8 || pin == 36 || pin == 46)) ||
			    (spi->chip_select == 1 &&
			     (pin == 7 || pin == 35))) {
				spi->cs_gpio = pin;
				break;
			}
		}
		of_node_put(pins);
	}

	/* If that failed, assume GPIOs 7-11 are used */
	if (!gpio_is_valid(spi->cs_gpio) ) {
		chip = gpiochip_find("pinctrl-bcm2835", gpio_match_name);
		if (chip == NULL)
			return 0;

		spi->cs_gpio = chip->base + 8 - spi->chip_select;
	}

	dev_info(&spi->dev,
		 "setting up native-CS%i as GPIO %i\n",
		 spi->chip_select, spi->cs_gpio);

	ret = gpio_direction_output(spi->cs_gpio,
			    (spi->mode & SPI_CS_HIGH) ? 0 : 1);
	if (ret) {
		dev_err(&spi->dev,
			"could not set CS%i gpio %i as output: %i",
			spi->chip_select, spi->cs_gpio, ret);
		return ret;
	}

	/*
	 * Force value on GPIO in case the pin controller does not
	 * handle that properly when switching to output mode.
	 */
	gpio_set_value(spi->cs_gpio, (spi->mode & SPI_CS_HIGH) ? 0 : 1);

	return 0;
}

static struct rtdm_spi_remote_slave *
bcm2835_attach_slave(struct rtdm_spi_master *master, struct spi_device *spi)
{
	struct spi_slave_bcm2835 *bcm;
	int ret;

	if (spi->chip_select > 1) {
		/*
		 * Error in the case of native CS requested with CS >
		 * 1 officially there is a CS2, but it is not
		 * documented which GPIO is connected with that...
		 */
		dev_err(&spi->dev,
			"%s: only two native chip-selects are supported\n",
			__func__);
		return ERR_PTR(-EINVAL);
	}

	ret = find_cs_gpio(spi);
	if (ret)
		return ERR_PTR(ret);
	
	bcm = kzalloc(sizeof(*bcm), GFP_KERNEL);
	if (bcm == NULL)
		return ERR_PTR(-ENOMEM);

	ret = rtdm_spi_add_remote_slave(&bcm->slave, master, spi);
	if (ret) {
		dev_err(&spi->dev,
			"%s: failed to attach slave\n", __func__);
		kfree(bcm);
		return ERR_PTR(ret);
	}

	return &bcm->slave;
}

static void bcm2835_detach_slave(struct rtdm_spi_remote_slave *slave)
{
	struct spi_slave_bcm2835 *bcm = to_slave_bcm2835(slave);

	rtdm_spi_remove_remote_slave(slave);
	kfree(bcm);
}

static struct rtdm_spi_master_ops bcm2835_master_ops = {
	.configure = bcm2835_configure,
	.chip_select = bcm2835_chip_select,
	.set_iobufs = bcm2835_set_iobufs,
	.mmap_iobufs = bcm2835_mmap_iobufs,
	.mmap_release = bcm2835_mmap_release,
	.transfer_iobufs = bcm2835_transfer_iobufs,
	.transfer_iobufs_n = bcm2835_transfer_iobufs_n,
	.write = bcm2835_write,
	.read = bcm2835_read,
	.attach_slave = bcm2835_attach_slave,
	.detach_slave = bcm2835_detach_slave,
};

static int bcm2835_spi_probe(struct platform_device *pdev)
{
	struct spi_master_bcm2835 *spim;
	struct rtdm_spi_master *master;
	struct spi_master *kmaster;
	struct resource *r;
	int ret, irq;

	dev_dbg(&pdev->dev, "%s: entered\n", __func__);

	master = rtdm_spi_alloc_master(&pdev->dev,
		   struct spi_master_bcm2835, master);
	if (master == NULL)
		return -ENOMEM;

	master->subclass = RTDM_SUBCLASS_BCM2835;
	master->ops = &bcm2835_master_ops;
	platform_set_drvdata(pdev, master);

	kmaster = master->kmaster;
	kmaster->mode_bits = BCM2835_SPI_MODE_BITS;
	kmaster->bits_per_word_mask = SPI_BPW_MASK(8);
	kmaster->num_chipselect = 2;
	kmaster->dev.of_node = pdev->dev.of_node;

	spim = container_of(master, struct spi_master_bcm2835, master);
	rtdm_event_init(&spim->transfer_done, 0);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	spim->regs = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(spim->regs)) {
		dev_err(&pdev->dev, "%s: cannot map I/O memory\n", __func__);
		ret = PTR_ERR(spim->regs);
		goto fail;
	}
	
	spim->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(spim->clk)) {
		ret = PTR_ERR(spim->clk);
		goto fail;
	}

	spim->clk_hz = clk_get_rate(spim->clk);

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (irq <= 0) {
		ret = irq ?: -ENODEV;
		goto fail;
	}

	clk_prepare_enable(spim->clk);

	/* Initialise the hardware with the default polarities */
	bcm2835_wr(spim, BCM2835_SPI_CS,
		   BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX);

	ret = rtdm_irq_request(&spim->irqh, irq,
			       bcm2835_spi_interrupt, 0,
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
		goto fail_unclk;
	}

	return 0;

fail_unclk:
	clk_disable_unprepare(spim->clk);
fail:
	spi_master_put(kmaster);

	return ret;
}

static int bcm2835_spi_remove(struct platform_device *pdev)
{
	struct rtdm_spi_master *master = platform_get_drvdata(pdev);
	struct spi_master_bcm2835 *spim;

	dev_dbg(&pdev->dev, "%s: entered\n", __func__);

	spim = container_of(master, struct spi_master_bcm2835, master);

	/* Clear FIFOs, and disable the HW block */
	bcm2835_wr(spim, BCM2835_SPI_CS,
		   BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX);

	rtdm_irq_free(&spim->irqh);

	clk_disable_unprepare(spim->clk);

	rtdm_spi_remove_master(master);

	return 0;
}

static const struct of_device_id bcm2835_spi_match[] = {
	{
		.compatible = "brcm,bcm2835-spi",
	},
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, bcm2835_spi_match);

static struct platform_driver bcm2835_spi_driver = {
	.driver		= {
		.name		= "spi-bcm2835",
		.of_match_table	= bcm2835_spi_match,
	},
	.probe		= bcm2835_spi_probe,
	.remove		= bcm2835_spi_remove,
};
module_platform_driver(bcm2835_spi_driver);

MODULE_LICENSE("GPL");
