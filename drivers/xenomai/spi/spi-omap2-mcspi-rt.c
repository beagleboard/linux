/**
 * I/O handling lifted from drivers/spi/spi-omap2-mcspi.c:
 * Copyright (C) 2019 Laurentiu-Cristian Duca
 *  <laurentiu [dot] duca [at] gmail [dot] com>
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
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/gcd.h>
#include "spi-master.h"

#define RTDM_SUBCLASS_OMAP2_MCSPI  3

#define OMAP4_MCSPI_REG_OFFSET 0x100
#define OMAP2_MCSPI_SPI_MODE_BITS	(SPI_CPOL | SPI_CPHA | SPI_CS_HIGH)

#define OMAP2_MCSPI_MAX_FREQ		48000000
#define OMAP2_MCSPI_DRIVER_MAX_FREQ	40000000
#define OMAP2_MCSPI_MAX_DIVIDER		4096
#define OMAP2_MCSPI_MAX_FIFODEPTH	64
#define OMAP2_MCSPI_MAX_FIFOWCNT	0xFFFF
#define SPI_AUTOSUSPEND_TIMEOUT		2000
#define PM_NEGATIVE_DELAY			-2000

#define OMAP2_MCSPI_REVISION		0x00
#define OMAP2_MCSPI_SYSCONFIG		0x10
#define OMAP2_MCSPI_SYSSTATUS		0x14
#define OMAP2_MCSPI_IRQSTATUS		0x18
#define OMAP2_MCSPI_IRQENABLE		0x1c
#define OMAP2_MCSPI_WAKEUPENABLE	0x20
#define OMAP2_MCSPI_SYST		0x24
#define OMAP2_MCSPI_MODULCTRL		0x28
#define OMAP2_MCSPI_XFERLEVEL		0x7c

/* per-channel (chip select) banks, 0x14 bytes each, first is: */
#define OMAP2_MCSPI_CHANNELBANK_SIZE	0x14
#define OMAP2_MCSPI_CHCONF0		0x2c
#define OMAP2_MCSPI_CHSTAT0		0x30
#define OMAP2_MCSPI_CHCTRL0		0x34
#define OMAP2_MCSPI_TX0			0x38
#define OMAP2_MCSPI_RX0			0x3c

/* per-register bitmasks: */
#define OMAP2_MCSPI_IRQSTATUS_EOW		BIT(17)
#define OMAP2_MCSPI_IRQSTATUS_RX1_FULL  BIT(6)
#define OMAP2_MCSPI_IRQSTATUS_TX1_EMPTY	BIT(4)
#define OMAP2_MCSPI_IRQSTATUS_RX0_FULL  BIT(2)
#define OMAP2_MCSPI_IRQSTATUS_TX0_EMPTY	BIT(0)

#define OMAP2_MCSPI_IRQENABLE_EOW		BIT(17)
#define OMAP2_MCSPI_IRQENABLE_RX1_FULL  BIT(6)
#define OMAP2_MCSPI_IRQENABLE_TX1_EMPTY	BIT(4)
#define OMAP2_MCSPI_IRQENABLE_RX0_FULL  BIT(2)
#define OMAP2_MCSPI_IRQENABLE_TX0_EMPTY	BIT(0)

#define OMAP2_MCSPI_MODULCTRL_SINGLE	BIT(0)
#define OMAP2_MCSPI_MODULCTRL_MS	BIT(2)
#define OMAP2_MCSPI_MODULCTRL_STEST	BIT(3)

#define OMAP2_MCSPI_CHCONF_PHA		BIT(0)
#define OMAP2_MCSPI_CHCONF_POL		BIT(1)
#define OMAP2_MCSPI_CHCONF_CLKD_MASK	(0x0f << 2)
#define OMAP2_MCSPI_CHCONF_EPOL		BIT(6)
#define OMAP2_MCSPI_CHCONF_WL_MASK	(0x1f << 7)
#define OMAP2_MCSPI_CHCONF_TRM_RX_ONLY	BIT(12)
#define OMAP2_MCSPI_CHCONF_TRM_TX_ONLY	BIT(13)
#define OMAP2_MCSPI_CHCONF_TRM_MASK	(0x03 << 12)
#define OMAP2_MCSPI_CHCONF_DMAW		BIT(14)
#define OMAP2_MCSPI_CHCONF_DMAR		BIT(15)
#define OMAP2_MCSPI_CHCONF_DPE0		BIT(16)
#define OMAP2_MCSPI_CHCONF_DPE1		BIT(17)
#define OMAP2_MCSPI_CHCONF_IS		BIT(18)
#define OMAP2_MCSPI_CHCONF_TURBO	BIT(19)
#define OMAP2_MCSPI_CHCONF_FORCE	BIT(20)
#define OMAP2_MCSPI_CHCONF_FFET		BIT(27)
#define OMAP2_MCSPI_CHCONF_FFER		BIT(28)
#define OMAP2_MCSPI_CHCONF_CLKG		BIT(29)

#define OMAP2_MCSPI_CHSTAT_RXS		BIT(0)
#define OMAP2_MCSPI_CHSTAT_TXS		BIT(1)
#define OMAP2_MCSPI_CHSTAT_EOT		BIT(2)
#define OMAP2_MCSPI_CHSTAT_TXFFE	BIT(3)

#define OMAP2_MCSPI_CHCTRL_EN		BIT(0)
#define OMAP2_MCSPI_CHCTRL_EXTCLK_MASK	(0xff << 8)

#define OMAP2_MCSPI_WAKEUPENABLE_WKEN	BIT(0)

#define OMAP2_MCSPI_SYSCONFIG_CLOCKACTIVITY_MASK	(0x3 << 8)
#define OMAP2_MCSPI_SYSCONFIG_SIDLEMODE_MASK		(0x3 << 3)
#define OMAP2_MCSPI_SYSCONFIG_SOFTRESET				BIT(1)
#define OMAP2_MCSPI_SYSCONFIG_AUTOIDLE				BIT(0)

#define OMAP2_MCSPI_SYSSTATUS_RESETDONE BIT(0)

/* current version supports max 2 CS per module */
#define OMAP2_MCSPI_CS_N	2

#define MCSPI_PINDIR_D0_IN_D1_OUT	0
#define MCSPI_PINDIR_D0_OUT_D1_IN	1

struct omap2_mcspi_platform_config {
	unsigned short	num_cs;
	unsigned int regs_offset;
	unsigned int pin_dir:1;
};

struct omap2_mcspi_cs {
	/* CS channel */
	void __iomem		*regs;
	unsigned long		phys;
	u8 chosen;
};

struct spi_master_omap2_mcspi {
	struct rtdm_spi_master master;
	void __iomem *regs;
	unsigned long phys;
	rtdm_irq_t irqh;
	const u8 *tx_buf;
	u8 *rx_buf;
	int tx_len;
	int rx_len;
	int fifo_depth;
	rtdm_event_t transfer_done;
	rtdm_lock_t lock;
	unsigned int pin_dir:1;
	struct omap2_mcspi_cs cs[OMAP2_MCSPI_CS_N];
	/* logging */
	int n_rx_full;
	int n_tx_empty;
	int n_interrupts;
};

struct spi_slave_omap2_mcspi {
	struct rtdm_spi_remote_slave slave;
	void *io_virt;
	dma_addr_t io_dma;
	size_t io_len;
};

static inline struct spi_slave_omap2_mcspi *
to_slave_omap2_mcspi(struct rtdm_spi_remote_slave *slave)
{
	return container_of(slave, struct spi_slave_omap2_mcspi, slave);
}

static inline struct spi_master_omap2_mcspi *
to_master_omap2_mcspi(struct rtdm_spi_remote_slave *slave)
{
	return container_of(slave->master,
			struct spi_master_omap2_mcspi, master);
}

static inline struct device *
master_to_kdev(struct rtdm_spi_master *master)
{
	return &master->kmaster->dev;
}

static inline u32 mcspi_rd_reg(struct spi_master_omap2_mcspi *spim,
			     unsigned int reg)
{
	return readl(spim->regs + reg);
}

static inline void mcspi_wr_reg(struct spi_master_omap2_mcspi *spim,
			      unsigned int reg, u32 val)
{
	writel(val, spim->regs + reg);
}

static inline u32
mcspi_rd_cs_reg(struct spi_master_omap2_mcspi *spim,
				int cs_id, unsigned int reg)
{
	return readl(spim->cs[cs_id].regs + reg);
}

static inline void
mcspi_wr_cs_reg(struct spi_master_omap2_mcspi *spim, int cs_id,
				unsigned int reg, u32 val)
{
	writel(val, spim->cs[cs_id].regs + reg);
}

static void omap2_mcspi_init_hw(struct spi_master_omap2_mcspi *spim)
{
	u32 l;

	l = mcspi_rd_reg(spim, OMAP2_MCSPI_SYSCONFIG);
	/* CLOCKACTIVITY = 3h: OCP and Functional clocks are maintained */
	l |= OMAP2_MCSPI_SYSCONFIG_CLOCKACTIVITY_MASK;
	/* SIDLEMODE = 1h: ignore idle requests */
	l &= ~OMAP2_MCSPI_SYSCONFIG_SIDLEMODE_MASK;
	l |= 0x1 << 3;
	/* AUTOIDLE=0: OCP clock is free-running */
	l &= ~OMAP2_MCSPI_SYSCONFIG_AUTOIDLE;
	mcspi_wr_reg(spim, OMAP2_MCSPI_SYSCONFIG, l);

	/* Initialise the hardware with the default polarities (only omap2) */
	mcspi_wr_reg(spim, OMAP2_MCSPI_WAKEUPENABLE,
				 OMAP2_MCSPI_WAKEUPENABLE_WKEN);

	/* Setup single-channel master mode */
	l = mcspi_rd_reg(spim, OMAP2_MCSPI_MODULCTRL);
	/* MS=0 => spi master */
	l &= ~(OMAP2_MCSPI_MODULCTRL_STEST | OMAP2_MCSPI_MODULCTRL_MS);
	l |= OMAP2_MCSPI_MODULCTRL_SINGLE;
	mcspi_wr_reg(spim, OMAP2_MCSPI_MODULCTRL, l);
}

static void omap2_mcspi_reset_hw(struct spi_master_omap2_mcspi *spim)
{
	u32 l;

	l = mcspi_rd_reg(spim, OMAP2_MCSPI_SYSCONFIG);
	l |= OMAP2_MCSPI_SYSCONFIG_SOFTRESET;
	mcspi_wr_reg(spim, OMAP2_MCSPI_SYSCONFIG, l);
	/* wait until reset is done */
	do {
		l = mcspi_rd_reg(spim, OMAP2_MCSPI_SYSSTATUS);
		cpu_relax();
	} while (!(l & OMAP2_MCSPI_SYSSTATUS_RESETDONE));
}

static void
omap2_mcspi_chip_select(struct rtdm_spi_remote_slave *slave, bool active)
{
	struct spi_master_omap2_mcspi *spim = to_master_omap2_mcspi(slave);
	u32 l;

	/* FORCE: manual SPIEN assertion to keep SPIEN active */
	l = mcspi_rd_cs_reg(spim, slave->chip_select, OMAP2_MCSPI_CHCONF0);
	/* "active" is the logical state, not the impedance level. */
	if (active)
		l |= OMAP2_MCSPI_CHCONF_FORCE;
	else
		l &= ~OMAP2_MCSPI_CHCONF_FORCE;
	mcspi_wr_cs_reg(spim, slave->chip_select, OMAP2_MCSPI_CHCONF0, l);
	/* Flash post-writes */
	l = mcspi_rd_cs_reg(spim, slave->chip_select, OMAP2_MCSPI_CHCONF0);
}

static u32 omap2_mcspi_calc_divisor(u32 speed_hz)
{
	u32 div;

	for (div = 0; div < 15; div++)
		if (speed_hz >= (OMAP2_MCSPI_MAX_FREQ >> div))
			return div;

	return 15;
}

/* channel 0 enable/disable */
static void
omap2_mcspi_channel_enable(struct rtdm_spi_remote_slave *slave, int enable)
{
	struct spi_master_omap2_mcspi *spim = to_master_omap2_mcspi(slave);
	u32 l;

	l = mcspi_rd_cs_reg(spim, slave->chip_select, OMAP2_MCSPI_CHCTRL0);
	if (enable)
		l |= OMAP2_MCSPI_CHCTRL_EN;
	else
		l &= ~OMAP2_MCSPI_CHCTRL_EN;
	mcspi_wr_cs_reg(spim, slave->chip_select, OMAP2_MCSPI_CHCTRL0, l);
	/* Flash post-writes */
	l = mcspi_rd_cs_reg(spim, slave->chip_select, OMAP2_MCSPI_CHCTRL0);
}

/* called only when no transfer is active to this device */
static int omap2_mcspi_configure(struct rtdm_spi_remote_slave *slave)
{
	struct spi_master_omap2_mcspi *spim = to_master_omap2_mcspi(slave);
	struct rtdm_spi_config *config = &slave->config;
	u32 l = 0, clkd = 0, div = 1, extclk = 0, clkg = 0, word_len;
	u32 speed_hz = OMAP2_MCSPI_MAX_FREQ;
	u32 chctrl0;

	/* The configuration parameters can be loaded in MCSPI_CH(i)CONF
	 * only when the channel is disabled
	 */
	omap2_mcspi_channel_enable(slave, 0);

	l = mcspi_rd_cs_reg(spim, slave->chip_select, OMAP2_MCSPI_CHCONF0);

	/* Set clock frequency. */
	speed_hz = (u32) config->speed_hz;
	if (speed_hz > OMAP2_MCSPI_DRIVER_MAX_FREQ) {
		dev_warn(slave_to_kdev(slave),
			"maximum clock frequency is %d",
			OMAP2_MCSPI_DRIVER_MAX_FREQ);
	}
	speed_hz = min_t(u32, speed_hz, OMAP2_MCSPI_DRIVER_MAX_FREQ);
	if (speed_hz < (OMAP2_MCSPI_MAX_FREQ / OMAP2_MCSPI_MAX_DIVIDER)) {
		clkd = omap2_mcspi_calc_divisor(speed_hz);
		speed_hz = OMAP2_MCSPI_MAX_FREQ >> clkd;
		clkg = 0;
	} else {
		div = (OMAP2_MCSPI_MAX_FREQ + speed_hz - 1) / speed_hz;
		speed_hz = OMAP2_MCSPI_MAX_FREQ / div;
		clkd = (div - 1) & 0xf;
		extclk = (div - 1) >> 4;
		clkg = OMAP2_MCSPI_CHCONF_CLKG;
	}
	/* set clock divisor */
	l &= ~OMAP2_MCSPI_CHCONF_CLKD_MASK;
	l |= clkd << 2;
	/* set clock granularity */
	l &= ~OMAP2_MCSPI_CHCONF_CLKG;
	l |= clkg;
	if (clkg) {
		chctrl0 = mcspi_rd_cs_reg(spim,
			slave->chip_select, OMAP2_MCSPI_CHCTRL0);
		chctrl0 &= ~OMAP2_MCSPI_CHCTRL_EXTCLK_MASK;
		chctrl0 |= extclk << 8;
		mcspi_wr_cs_reg(spim,
			slave->chip_select, OMAP2_MCSPI_CHCTRL0, chctrl0);
	}

	if (spim->pin_dir == MCSPI_PINDIR_D0_IN_D1_OUT) {
		l &= ~OMAP2_MCSPI_CHCONF_IS;
		l &= ~OMAP2_MCSPI_CHCONF_DPE1;
		l |= OMAP2_MCSPI_CHCONF_DPE0;
	} else {
		l |= OMAP2_MCSPI_CHCONF_IS;
		l |= OMAP2_MCSPI_CHCONF_DPE1;
		l &= ~OMAP2_MCSPI_CHCONF_DPE0;
	}

	/* wordlength */
	word_len = config->bits_per_word;
	/* TODO: allow word_len != 8 */
	if (word_len != 8) {
		dev_err(slave_to_kdev(slave), "word_len(%d) != 8.\n",
				word_len);
		return -EIO;
	}
	l &= ~OMAP2_MCSPI_CHCONF_WL_MASK;
	l |= (word_len - 1) << 7;

	/* set chipselect polarity; manage with FORCE */
	if (!(config->mode & SPI_CS_HIGH))
		/* CS active-low */
		l |= OMAP2_MCSPI_CHCONF_EPOL;
	else
		l &= ~OMAP2_MCSPI_CHCONF_EPOL;

	/* set SPI mode 0..3 */
	if (config->mode & SPI_CPOL)
		l |= OMAP2_MCSPI_CHCONF_POL;
	else
		l &= ~OMAP2_MCSPI_CHCONF_POL;
	if (config->mode & SPI_CPHA)
		l |= OMAP2_MCSPI_CHCONF_PHA;
	else
		l &= ~OMAP2_MCSPI_CHCONF_PHA;

	mcspi_wr_cs_reg(spim, slave->chip_select, OMAP2_MCSPI_CHCONF0, l);
	l = mcspi_rd_cs_reg(spim, slave->chip_select, OMAP2_MCSPI_CHCONF0);

	omap2_mcspi_chip_select(slave, 0);

	return 0;
}

static void mcspi_rd_fifo(struct spi_master_omap2_mcspi *spim, int cs_id)
{
	u8 byte;
	int i;

	/* Receiver register must be read to remove source of interrupt */
	for (i = 0; i < spim->fifo_depth; i++) {
		byte = mcspi_rd_cs_reg(spim, cs_id, OMAP2_MCSPI_RX0);
		if (spim->rx_buf && (spim->rx_len > 0))
			*spim->rx_buf++ = byte;
		spim->rx_len--;
	}
}

static void mcspi_wr_fifo(struct spi_master_omap2_mcspi *spim, int cs_id)
{
	u8 byte;
	int i;

	/* load transmitter register to remove the source of the interrupt */
	for (i = 0; i < spim->fifo_depth; i++) {
		if (spim->tx_len <= 0)
			byte = 0;
		else
			byte = spim->tx_buf ? *spim->tx_buf++ : 0;
		mcspi_wr_cs_reg(spim, cs_id, OMAP2_MCSPI_TX0, byte);
		spim->tx_len--;
	}
}

static void mcspi_wr_fifo_bh(struct spi_master_omap2_mcspi *spim, int cs_id)
{
	u8 byte;
	int i;
	rtdm_lockctx_t c;

	rtdm_lock_get_irqsave(&spim->lock, c);

	for (i = 0; i < spim->fifo_depth; i++) {
		if (spim->tx_len <= 0)
			byte = 0;
		else
			byte = spim->tx_buf ? *spim->tx_buf++ : 0;
		mcspi_wr_cs_reg(spim, cs_id, OMAP2_MCSPI_TX0, byte);
		spim->tx_len--;
	}

	rtdm_lock_put_irqrestore(&spim->lock, c);
}

static int omap2_mcspi_interrupt(rtdm_irq_t *irqh)
{
	struct spi_master_omap2_mcspi *spim;
	u32 l;
	int i, cs_id = 0;

	spim = rtdm_irq_get_arg(irqh, struct spi_master_omap2_mcspi);
	rtdm_lock_get(&spim->lock);

	for (i = 0; i < OMAP2_MCSPI_CS_N; i++)
		if (spim->cs[i].chosen) {
			cs_id = i;
			break;
		}

	spim->n_interrupts++;
	l = mcspi_rd_reg(spim, OMAP2_MCSPI_IRQSTATUS);

	if ((l & OMAP2_MCSPI_IRQSTATUS_RX0_FULL) ||
	   (l & OMAP2_MCSPI_IRQSTATUS_RX1_FULL)) {
		mcspi_rd_fifo(spim, cs_id);
		spim->n_rx_full++;
	}
	if ((l & OMAP2_MCSPI_IRQSTATUS_TX0_EMPTY) ||
		(l & OMAP2_MCSPI_IRQSTATUS_TX1_EMPTY)) {
		if (spim->tx_len > 0)
			mcspi_wr_fifo(spim, cs_id);
		spim->n_tx_empty++;
	}

	/* write 1 to OMAP2_MCSPI_IRQSTATUS field to reset it */
	mcspi_wr_reg(spim, OMAP2_MCSPI_IRQSTATUS, l);

	if ((spim->tx_len <= 0) && (spim->rx_len <= 0)) {
		/* disable interrupts */
		mcspi_wr_reg(spim, OMAP2_MCSPI_IRQENABLE, 0);

		rtdm_event_signal(&spim->transfer_done);
	}

	rtdm_lock_put(&spim->lock);

	return RTDM_IRQ_HANDLED;
}

static int omap2_mcspi_disable_fifo(struct rtdm_spi_remote_slave *slave,
							int cs_id)
{
	struct spi_master_omap2_mcspi *spim = to_master_omap2_mcspi(slave);
	u32 chconf;

	chconf = mcspi_rd_cs_reg(spim, cs_id, OMAP2_MCSPI_CHCONF0);
	chconf &= ~(OMAP2_MCSPI_CHCONF_FFER | OMAP2_MCSPI_CHCONF_FFET);
	mcspi_wr_cs_reg(spim, cs_id, OMAP2_MCSPI_CHCONF0, chconf);
	return 0;
}

static int omap2_mcspi_set_fifo(struct rtdm_spi_remote_slave *slave)
{
	struct spi_master_omap2_mcspi *spim = to_master_omap2_mcspi(slave);
	unsigned int wcnt;
	int max_fifo_depth, fifo_depth, bytes_per_word;
	u32 chconf, xferlevel;

	chconf = mcspi_rd_cs_reg(spim, slave->chip_select, OMAP2_MCSPI_CHCONF0);
	bytes_per_word = 1;

	max_fifo_depth = OMAP2_MCSPI_MAX_FIFODEPTH / 2;
	if (spim->tx_len < max_fifo_depth) {
		fifo_depth = spim->tx_len;
		wcnt = spim->tx_len / bytes_per_word;
	} else {
		fifo_depth = max_fifo_depth;
		wcnt = max_fifo_depth * (spim->tx_len / max_fifo_depth)
			/ bytes_per_word;
	}
	if (wcnt > OMAP2_MCSPI_MAX_FIFOWCNT) {
		dev_err(slave_to_kdev(slave),
			"%s: wcnt=%d: too many bytes in a transfer.\n",
			__func__, wcnt);
		return -EINVAL;
	}

	chconf |= OMAP2_MCSPI_CHCONF_FFER;
	chconf |= OMAP2_MCSPI_CHCONF_FFET;

	mcspi_wr_cs_reg(spim, slave->chip_select, OMAP2_MCSPI_CHCONF0, chconf);
	spim->fifo_depth = fifo_depth;

	xferlevel = wcnt << 16;
	xferlevel |= (fifo_depth - 1) << 8;
	xferlevel |= fifo_depth - 1;
	mcspi_wr_reg(spim, OMAP2_MCSPI_XFERLEVEL, xferlevel);

	return 0;
}


static int do_transfer_irq_bh(struct rtdm_spi_remote_slave *slave)
{
	struct spi_master_omap2_mcspi *spim = to_master_omap2_mcspi(slave);
	u32 chconf, l;
	int ret;
	int i;

	/* configure to send and receive */
	chconf = mcspi_rd_cs_reg(spim, slave->chip_select, OMAP2_MCSPI_CHCONF0);
	chconf &= ~OMAP2_MCSPI_CHCONF_TRM_MASK;
	chconf &= ~OMAP2_MCSPI_CHCONF_TURBO;
	mcspi_wr_cs_reg(spim, slave->chip_select, OMAP2_MCSPI_CHCONF0, chconf);

	/* fifo can be enabled on a single channel */
	if (slave->chip_select == 0) {
		if (spim->cs[1].chosen)
			omap2_mcspi_disable_fifo(slave, 1);
	} else {
		if (spim->cs[0].chosen)
			omap2_mcspi_disable_fifo(slave, 0);
	}
	ret = omap2_mcspi_set_fifo(slave);
	if (ret)
		return ret;

	omap2_mcspi_channel_enable(slave, 1);

	/* Set slave->chip_select as chosen */
	for (i = 0; i < OMAP2_MCSPI_CS_N; i++)
		if (i == slave->chip_select)
			spim->cs[i].chosen = 1;
		else
			spim->cs[i].chosen = 0;

	/* The interrupt status bit should always be reset
	 * after the channel is enabled
	 * and before the event is enabled as an interrupt source.
	 */
	/* write 1 to OMAP2_MCSPI_IRQSTATUS field to reset it */
	l = mcspi_rd_reg(spim, OMAP2_MCSPI_IRQSTATUS);
	mcspi_wr_reg(spim, OMAP2_MCSPI_IRQSTATUS, l);

	spim->n_interrupts = 0;
	spim->n_rx_full = 0;
	spim->n_tx_empty = 0;

	/* Enable interrupts last. */
	/* support only two channels */
	if (slave->chip_select == 0)
		l = OMAP2_MCSPI_IRQENABLE_TX0_EMPTY |
			OMAP2_MCSPI_IRQENABLE_RX0_FULL;
	else
		l = OMAP2_MCSPI_IRQENABLE_TX1_EMPTY |
			OMAP2_MCSPI_IRQENABLE_RX1_FULL;
	mcspi_wr_reg(spim, OMAP2_MCSPI_IRQENABLE, l);

	/* TX_EMPTY will be raised only after data is transfered */
	mcspi_wr_fifo_bh(spim, slave->chip_select);

	/* wait for transfer completion */
	ret = rtdm_event_wait(&spim->transfer_done);
	omap2_mcspi_channel_enable(slave, 0);
	if (ret)
		return ret;

	/* spim->tx_len and spim->rx_len should be 0 */
	if (spim->tx_len || spim->rx_len)
		return -EIO;
	return 0;
}

static int do_transfer_irq(struct rtdm_spi_remote_slave *slave)
{
	struct spi_master_omap2_mcspi *spim = to_master_omap2_mcspi(slave);
	int len, first_size, last_size, ret;

	len = spim->tx_len;

	if (len < (OMAP2_MCSPI_MAX_FIFODEPTH / 2))
		goto label_last;

	first_size = (OMAP2_MCSPI_MAX_FIFODEPTH / 2) *
		(len / (OMAP2_MCSPI_MAX_FIFODEPTH / 2));
	spim->tx_len = first_size;
	spim->rx_len = first_size;
	ret = do_transfer_irq_bh(slave);
	if (ret)
		return ret;

label_last:
	last_size = len % (OMAP2_MCSPI_MAX_FIFODEPTH / 2);
	if (last_size == 0)
		return ret;
	spim->tx_len = last_size;
	spim->rx_len = last_size;
	ret = do_transfer_irq_bh(slave);
	return ret;
}

static int omap2_mcspi_transfer_iobufs(struct rtdm_spi_remote_slave *slave)
{
	struct spi_master_omap2_mcspi *spim = to_master_omap2_mcspi(slave);
	struct spi_slave_omap2_mcspi *mapped_data = to_slave_omap2_mcspi(slave);
	int ret;

	if (mapped_data->io_len == 0)
		return -EINVAL;	/* No I/O buffers set. */

	spim->tx_len = mapped_data->io_len / 2;
	spim->rx_len = spim->tx_len;
	spim->tx_buf = mapped_data->io_virt + spim->rx_len;
	spim->rx_buf = mapped_data->io_virt;

	ret = do_transfer_irq(slave);

	return ret ? : 0;
}

static int omap2_mcspi_transfer_iobufs_n(struct rtdm_spi_remote_slave *slave,
								 int len)
{
	struct spi_master_omap2_mcspi *spim = to_master_omap2_mcspi(slave);
	struct spi_slave_omap2_mcspi *mapped_data = to_slave_omap2_mcspi(slave);
	int ret;

	if ((mapped_data->io_len == 0) ||
		(len <= 0) || (len > (mapped_data->io_len / 2)))
		return -EINVAL;

	spim->tx_len = len;
	spim->rx_len = len;
	spim->tx_buf = mapped_data->io_virt + mapped_data->io_len / 2;
	spim->rx_buf = mapped_data->io_virt;

	ret = do_transfer_irq(slave);


	return ret ? : 0;
}

static ssize_t omap2_mcspi_read(struct rtdm_spi_remote_slave *slave,
			    void *rx, size_t len)
{
	struct spi_master_omap2_mcspi *spim = to_master_omap2_mcspi(slave);
	int ret;

	spim->tx_len = len;
	spim->rx_len = len;
	spim->tx_buf = NULL;
	spim->rx_buf = rx;

	ret = do_transfer_irq(slave);

	return  ret ? : len;
}

static ssize_t omap2_mcspi_write(struct rtdm_spi_remote_slave *slave,
			     const void *tx, size_t len)
{
	struct spi_master_omap2_mcspi *spim = to_master_omap2_mcspi(slave);
	int ret;

	spim->tx_len = len;
	spim->rx_len = len;
	spim->tx_buf = tx;
	spim->rx_buf = NULL;

	ret = do_transfer_irq(slave);

	return  ret ? : len;
}

static int set_iobufs(struct spi_slave_omap2_mcspi *mapped_data, size_t len)
{
	dma_addr_t dma;
	void *p;

	if (len == 0)
		return -EINVAL;

	len = L1_CACHE_ALIGN(len) * 2;
	if (len == mapped_data->io_len)
		return 0;

	if (mapped_data->io_len)
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

	mapped_data->io_dma = dma;
	mapped_data->io_virt = p;
	/*
	 * May race with transfer_iobufs(), must be assigned after all
	 * the rest is set up, enforcing a membar.
	 */
	smp_mb();
	mapped_data->io_len = len;

	return 0;
}

static int omap2_mcspi_set_iobufs(struct rtdm_spi_remote_slave *slave,
			      struct rtdm_spi_iobufs *p)
{
	struct spi_slave_omap2_mcspi *mapped_data = to_slave_omap2_mcspi(slave);
	int ret;

	ret = set_iobufs(mapped_data, p->io_len);
	if (ret)
		return ret;

	p->i_offset = 0;
	p->o_offset = mapped_data->io_len / 2;
	p->map_len = mapped_data->io_len;

	return 0;
}

static int omap2_mcspi_mmap_iobufs(struct rtdm_spi_remote_slave *slave,
			       struct vm_area_struct *vma)
{
	struct spi_slave_omap2_mcspi *mapped_data = to_slave_omap2_mcspi(slave);

	/*
	 * dma_alloc_coherent() delivers non-cached memory, make sure
	 * to return consistent mapping attributes. Typically, mixing
	 * memory attributes across address spaces referring to the
	 * same physical area is architecturally wrong on ARM.
	 */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);


	return rtdm_mmap_kmem(vma, mapped_data->io_virt);
}

static void omap2_mcspi_mmap_release(struct rtdm_spi_remote_slave *slave)
{
	struct spi_slave_omap2_mcspi *mapped_data = to_slave_omap2_mcspi(slave);

	dma_free_coherent(NULL, mapped_data->io_len,
			  mapped_data->io_virt, mapped_data->io_dma);
	mapped_data->io_len = 0;
}

static struct rtdm_spi_remote_slave *
omap2_mcspi_attach_slave(struct rtdm_spi_master *master, struct spi_device *spi)
{
	struct spi_master_omap2_mcspi *spim;
	struct spi_slave_omap2_mcspi *mapped_data;
	int ret;

	if ((spi->chip_select >= OMAP2_MCSPI_CS_N) || (OMAP2_MCSPI_CS_N > 2)) {
		/* Error in the case of native CS requested with CS > 1 */
		dev_err(&spi->dev, "%s: only two native CS per spi module are supported\n",
			__func__);
		return ERR_PTR(-EINVAL);
	}

	mapped_data = kzalloc(sizeof(*mapped_data), GFP_KERNEL);
	if (mapped_data == NULL)
		return ERR_PTR(-ENOMEM);

	ret = rtdm_spi_add_remote_slave(&mapped_data->slave, master, spi);
	if (ret) {
		dev_err(&spi->dev, "%s: failed to attach slave\n", __func__);
		kfree(mapped_data);
		return ERR_PTR(ret);
	}

	spim = container_of(master, struct spi_master_omap2_mcspi, master);
	spim->cs[spi->chip_select].chosen = 0;
	spim->cs[spi->chip_select].regs = spim->regs +
		spi->chip_select * OMAP2_MCSPI_CHANNELBANK_SIZE;
	spim->cs[spi->chip_select].phys = spim->phys +
		spi->chip_select * OMAP2_MCSPI_CHANNELBANK_SIZE;

	return &mapped_data->slave;
}

static void omap2_mcspi_detach_slave(struct rtdm_spi_remote_slave *slave)
{
	struct spi_slave_omap2_mcspi *mapped_data = to_slave_omap2_mcspi(slave);

	rtdm_spi_remove_remote_slave(slave);

	kfree(mapped_data);
}

static struct rtdm_spi_master_ops omap2_mcspi_master_ops = {
	.configure = omap2_mcspi_configure,
	.chip_select = omap2_mcspi_chip_select,
	.set_iobufs = omap2_mcspi_set_iobufs,
	.mmap_iobufs = omap2_mcspi_mmap_iobufs,
	.mmap_release = omap2_mcspi_mmap_release,
	.transfer_iobufs = omap2_mcspi_transfer_iobufs,
	.transfer_iobufs_n = omap2_mcspi_transfer_iobufs_n,
	.write = omap2_mcspi_write,
	.read = omap2_mcspi_read,
	.attach_slave = omap2_mcspi_attach_slave,
	.detach_slave = omap2_mcspi_detach_slave,
};

static struct omap2_mcspi_platform_config omap2_pdata = {
	.regs_offset = 0,
};

static struct omap2_mcspi_platform_config omap4_pdata = {
	.regs_offset = OMAP4_MCSPI_REG_OFFSET,
};

static const struct of_device_id omap_mcspi_of_match[] = {
	{
		.compatible = "ti,omap2-mcspi",
		.data = &omap2_pdata,
	},
	{
		/* beaglebone black */
		.compatible = "ti,omap4-mcspi",
		.data = &omap4_pdata,
	},
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, omap_mcspi_of_match);

static int omap2_mcspi_probe(struct platform_device *pdev)
{
	struct spi_master_omap2_mcspi *spim;
	struct rtdm_spi_master *master;
	struct spi_master *kmaster;
	struct resource *r;
	int ret, irq;
	u32 regs_offset = 0;
	const struct omap2_mcspi_platform_config *pdata;
	const struct of_device_id *match;
	u32 num_cs = 1;
	unsigned int pin_dir = MCSPI_PINDIR_D0_IN_D1_OUT;

	match = of_match_device(omap_mcspi_of_match, &pdev->dev);
	if (match) {
		pdata = match->data;
		regs_offset = pdata->regs_offset;
	} else {
		dev_err(&pdev->dev, "%s: cannot find a match with device tree\n"
				"of '%s' or '%s'",
				__func__,
				omap_mcspi_of_match[0].compatible,
				omap_mcspi_of_match[1].compatible);
		return -ENOENT;
	}

	master = rtdm_spi_alloc_master(&pdev->dev,
		   struct spi_master_omap2_mcspi, master);
	if (master == NULL)
		return -ENOMEM;

	master->subclass = RTDM_SUBCLASS_OMAP2_MCSPI;
	master->ops = &omap2_mcspi_master_ops;
	platform_set_drvdata(pdev, master);

	kmaster = master->kmaster;
	/* flags understood by this controller driver */
	kmaster->mode_bits = OMAP2_MCSPI_SPI_MODE_BITS;
	/* TODO: SPI_BPW_RANGE_MASK(4, 32); */
	kmaster->bits_per_word_mask = SPI_BPW_MASK(8);
	of_property_read_u32(pdev->dev.of_node, "ti,spi-num-cs", &num_cs);
	kmaster->num_chipselect = num_cs;
	if (of_get_property(pdev->dev.of_node,
		"ti,pindir-d0-out-d1-in", NULL)) {
		pin_dir = MCSPI_PINDIR_D0_OUT_D1_IN;
	}

	kmaster->max_speed_hz = OMAP2_MCSPI_MAX_FREQ;
	kmaster->min_speed_hz = OMAP2_MCSPI_MAX_FREQ >> 15;
	kmaster->dev.of_node = pdev->dev.of_node;

	spim = container_of(master, struct spi_master_omap2_mcspi, master);
	rtdm_event_init(&spim->transfer_done, 0);
	rtdm_lock_init(&spim->lock);

	spim->pin_dir = pin_dir;
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	spim->regs = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(spim->regs)) {
		dev_err(&pdev->dev, "%s: cannot map I/O memory\n", __func__);
		ret = PTR_ERR(spim->regs);
		goto fail;
	}
	spim->phys = r->start + regs_offset;
	spim->regs += regs_offset;

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (irq <= 0) {
		ret = irq ?: -ENODEV;
		dev_err(&pdev->dev, "%s: irq_of_parse_and_map: %d\n",
				__func__, irq);
		goto fail;
	}

	ret = rtdm_irq_request(&spim->irqh, irq,
			       omap2_mcspi_interrupt, 0,
			       dev_name(&pdev->dev), spim);
	if (ret) {
		dev_err(&pdev->dev, "%s: cannot request IRQ%d\n",
				__func__, irq);
		goto fail_unclk;
	}

	ret = rtdm_spi_add_master(&spim->master);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to add master\n", __func__);
		goto fail_unclk;
	}

	pm_runtime_use_autosuspend(&pdev->dev);
	/* if delay is negative and the use_autosuspend flag is set
	 * then runtime suspends are prevented.
	 */
	pm_runtime_set_autosuspend_delay(&pdev->dev, PM_NEGATIVE_DELAY);
	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: pm_runtime_get_sync error %d\n",
				__func__, ret);
		return ret;
	}

	omap2_mcspi_reset_hw(spim);
	omap2_mcspi_init_hw(spim);

	dev_info(&pdev->dev, "success\n");
	return 0;

fail_unclk:
fail:
	spi_master_put(kmaster);

	return ret;
}

static int omap2_mcspi_remove(struct platform_device *pdev)
{
	struct rtdm_spi_master *master = platform_get_drvdata(pdev);
	struct spi_master_omap2_mcspi *spim;

	spim = container_of(master, struct spi_master_omap2_mcspi, master);

	omap2_mcspi_reset_hw(spim);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	rtdm_irq_free(&spim->irqh);

	rtdm_spi_remove_master(master);

	return 0;
}

static struct platform_driver omap2_mcspi_spi_driver = {
	.driver		= {
		.name		= "omap2_mcspi_rt",
		.of_match_table	= omap_mcspi_of_match,
	},
	.probe		= omap2_mcspi_probe,
	.remove		= omap2_mcspi_remove,
};
module_platform_driver(omap2_mcspi_spi_driver);

MODULE_LICENSE("GPL");
