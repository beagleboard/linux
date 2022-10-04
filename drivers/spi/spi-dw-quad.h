// SPDX-License-Identifier: GPL-2.0-only
/*
 * Designware ehance-spi core controller driver
 *
 * Copyright (c) 2021, alibaba-inc Corporation.
 *
 * base on design-ware spi-core driver(spi-dw.h)
 */
#ifndef DW_QSPI_HEADER_H
#define DW_QSPI_HEADER_H

#include <linux/io.h>
#include <linux/scatterlist.h>
#include <linux/gpio/consumer.h>

//#define DW_QSPI_DRV_DEBUG
#ifdef  DW_QSPI_DRV_DEBUG
#define dw_drv_log(fmt,arg...) printk(fmt,##arg)
#else
#define dw_drv_log(fmt,arg...)
#endif

/* regs define for snps,dw-apb-ssi v4.02a */

/* Register offsets */
#define DW_SPI_CTRL0			0x00
#define DW_SPI_CTRL1			0x04
#define DW_SPI_SSIENR			0x08
#define DW_SPI_MWCR			    0x0c
#define DW_SPI_SER			    0x10
#define DW_SPI_BAUDR			0x14
#define DW_SPI_TXFLTR			0x18
#define DW_SPI_RXFLTR			0x1c
#define DW_SPI_TXFLR			0x20
#define DW_SPI_RXFLR			0x24
#define DW_SPI_SR		        0x28
#define DW_SPI_IMR			    0x2c
#define DW_SPI_ISR			    0x30
#define DW_SPI_RISR			    0x34
#define DW_SPI_TXOICR			0x38
#define DW_SPI_RXOICR			0x3c
#define DW_SPI_RXUICR			0x40
#define DW_SPI_MSTICR			0x44
#define DW_SPI_ICR			    0x48
#define DW_SPI_DMACR			0x4c
#define DW_SPI_DMATDLR			0x50
#define DW_SPI_DMARDLR			0x54
#define DW_SPI_IDR			    0x58
#define DW_SPI_VERSION			0x5c
#define DW_SPI_DR			    0x60
#define DW_SPI_RX_SMP_DLY       0xf0
#define DW_SPI_SPI_CTRLR0		0xf4
#define DW_SPI_TXD_DRV_EDGE     0xf8

/* Bit fields in CTRLR0 */
#define SPI_DFS_OFFSET			0       /* if SSI_MAX_XFER_SIZE is configured to 16 */

#define SPI_FRF_OFFSET			4
#define SPI_FRF_SPI			    0x0
#define SPI_FRF_SSP			    0x1
#define SPI_FRF_MICROWIRE		0x2
#define SPI_FRF_RESV			0x3

#define SPI_MODE_OFFSET			6
#define SPI_SCPH_OFFSET			6
#define SPI_SCOL_OFFSET			7

#define SPI_TMOD_OFFSET			8
#define SPI_TMOD_MASK			(0x3 << SPI_TMOD_OFFSET)
#define	SPI_TMOD_TR	            0x0		/* xmit & recv */
#define SPI_TMOD_TO			    0x1		/* xmit only */
#define SPI_TMOD_RO			    0x2		/* recv only */
#define SPI_TMOD_EPROMREAD		0x3		/* eeprom read mode */

#define SPI_SLVOE_OFFSET		10
#define SPI_SRL_OFFSET			11
#define SPI_CFS_OFFSET			12

#define SPI_DFS32_OFFSET        16      /* if SSI_MAX_XFER_SIZE is configured to 32 */
#define SPI_DFS32_MASK          (0x1f << SPI_DFS32_OFFSET)

#define SPI_SPI_FRF_OFFSET      21
#define SPI_SPI_FRF_MASK        (0x3 << SPI_SPI_FRF_OFFSET)
#define SPI_SPI_FRF_STD         0x0
#define SPI_SPI_FRF_DUAL        0x1
#define SPI_SPI_FRF_QUAD        0x2
#define SPI_SPI_FRF_OCTA        0x3

#define SPI_SSTE_OFFSET         24
#define SPI_SSTE_MASK           (1 << SPI_SSTE_OFFSET )

/* Bit fields in SR, 7 bits */
#define SR_MASK				    0x7f	   /* cover 7 bits */
#define SR_BUSY				    (1 << 0)   /*  set when serial transfer is in progress */
#define SR_TF_NOT_FULL			(1 << 1)   /*  tx fifo not full */
#define SR_TF_EMPT			    (1 << 2)   /*  tx fifo empty */
#define SR_RF_NOT_EMPT			(1 << 3)   /*  rx fifo not empty */
#define SR_RF_FULL			    (1 << 4)   /*  rx fifo full */
#define SR_TX_ERR			    (1 << 5)   /*  tx err(tx start but fifo is empty) only set when configured as slave device */
#define SR_DCOL				    (1 << 6)   /*  data colllision error */

/* Bit fields in ISR, IMR, RISR, 7 bits */
#define SPI_INT_TXEI			(1 << 0)   /* tx fifo empty */
#define SPI_INT_TXOI			(1 << 1)   /* tx fifo overflow */
#define SPI_INT_RXUI			(1 << 2)   /* rx fifo underfolow */
#define SPI_INT_RXOI			(1 << 3)   /* rx fifo overflow */
#define SPI_INT_RXFI			(1 << 4)   /* rx fifo full  */
#define SPI_INT_MSTI			(1 << 5)   /* multi-master contention interrupt, set only when configured as slave device */

/* Bit fields in DMACR */
#define SPI_DMA_RDMAE			(1 << 0)
#define SPI_DMA_TDMAE			(1 << 1)

/* Bit fields in SPI_CTRLR0 */
#define SPI_CTRLR0_TRNAS_OFFET       0x0
#define SPI_CTRLR0_TRANS_ISTD_ASTD   (0)       /* both instr and address are sent in stdandard mode */
#define SPI_CTRLR0_TRANS_ISTD_ASPF   (1)       /* instr sent in standard mode , address sent in the mode specified by CTRLR0.SPI_FRF */
#define SPI_CTRLR0_TRANS_ISPF_ASPF   (2)       /* both instr and address sent  in the mode specified by CTRLR0.SPI_FRF */

#define SPI_CTRLR0_ADDR_L_OFFSET     0x2
#define SPI_CTRLR0_ADDR_L_24         (6)
#define SPI_CTRLR0_ADDR_L_16         (4)

#define SPI_CTRLR0_INST_L_OFFSET     0x8
#define SPI_CTRLR0_INST_L_0          (0)
#define SPI_CTRLR0_INST_L_4          (1)
#define SPI_CTRLR0_INST_L_8          (2)
#define SPI_CTRLR0_INST_L_16         (3)

#define SPI_CTRLR0_WAIT_CYCLES_OFFSET (11)     /* count in io clks */

#define SPI_CTRLR0_DDR_EN_OFFSET     (16)      /* enable dual-ata rate transfers in dual/quad/octal frame formats of spi */
#define SPI_CTRLR0_DDR_EN_MASK       (1 << SPI_CTRLR0_DDR_EN_OFFSET)

#define SPI_CTRLR0_INST_DDR_EN_OFFSET (17)
#define SPI_CTRLR0_INST_DDR_EN_MASK   (1 << SPI_CTRLR0_INST_DDR_EN_OFFSET) 

#define SPI_CTRLR0_RXDS_EN_OFFSET    (18)
#define SPI_CTRLR0_RXDS_EN_MASK      (1 << SPI_CTRLR0_RXDS_EN_OFFSET)


/* TX RX interrupt level threshold, max can be 256 */
#define SPI_INT_THRESHOLD		32

enum dw_ssi_type {
	SSI_MOTO_SPI = 0,
	SSI_TI_SSP,
	SSI_NS_MICROWIRE,
};

struct dw_spi;
struct dw_spi_dma_ops {
	int (*dma_init)(struct dw_spi *dws);
	void (*dma_exit)(struct dw_spi *dws);
	int (*dma_setup)(struct dw_spi *dws, struct spi_transfer *xfer);
	bool (*can_dma)(struct spi_controller *master, struct spi_device *spi,
			struct spi_transfer *xfer);
	int (*dma_transfer)(struct dw_spi *dws, struct spi_transfer *xfer);
	void (*dma_stop)(struct dw_spi *dws);
};

struct xfer_pre_t{
#define       DW_MAX_CMD_BUF_LEN     32
    u8        xfer_pre[DW_MAX_CMD_BUF_LEN];
    u32       xfer_pre_len;
};

struct dw_spi {
	struct spi_controller	*master;
	enum dw_ssi_type	type;

	void __iomem		*regs;
	unsigned long		paddr;
	int			irq;
	u32			fifo_len;	/* depth of the FIFO buffer */
	u32			max_freq;	/* max bus freq supported */

	int			cs_override;
	u32			reg_io_width;	/* DR I/O width in bytes */
	u16			bus_num;
	u16			num_cs;		/* supported slave numbers */
	u16                     rx_sample_delay;/* timing value for rx sample delay */
	struct gpio_desc       *slave_cs;       /* gpio cs handle */
	void (*set_cs)(struct spi_device *spi, bool enable);
	/* used by spi_controller_mem_ops interface */
	struct xfer_pre_t xfer_data_pre;
	/* Current message transfer state info */
	size_t			len;
	void			*tx;
	void			*tx_end;
	spinlock_t		buf_lock;
	void			*rx;
	void			*rx_end;
	int			dma_mapped;
	u8			n_bytes;	/* current is a 1/2/4 bytes op */
	u32			dma_width;
	irqreturn_t		(*transfer_handler)(struct dw_spi *dws);
	u32			current_freq;	/* spi-io frequency in hz */

	/* DMA info */
	int			dma_inited;
	struct dma_chan		*txchan;
	struct dma_chan		*rxchan;
	unsigned long		dma_chan_busy;
	dma_addr_t		dma_addr; /* phy address of the Data register */
	const struct dw_spi_dma_ops *dma_ops;
	void			*dma_tx;
	void			*dma_rx;

	/* Bus interface info */
	void			*priv;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs;
#endif
};

static inline u32 dw_readl(struct dw_spi *dws, u32 offset)
{
	return __raw_readl(dws->regs + offset);
}

static inline u16 dw_readw(struct dw_spi *dws, u32 offset)
{
	return __raw_readw(dws->regs + offset);
}

static inline void dw_writel(struct dw_spi *dws, u32 offset, u32 val)
{
	__raw_writel(val, dws->regs + offset);
}

static inline void dw_writew(struct dw_spi *dws, u32 offset, u16 val)
{
	__raw_writew(val, dws->regs + offset);
}

static inline u32 dw_read_io_reg(struct dw_spi *dws, u32 offset)
{
	switch (dws->reg_io_width) {
	case 2:
		return dw_readw(dws, offset);
	case 4:
	default:
		return dw_readl(dws, offset);
	}
}

static inline void dw_write_io_reg(struct dw_spi *dws, u32 offset, u32 val)
{
	switch (dws->reg_io_width) {
	case 2:
		dw_writew(dws, offset, val);
		break;
	case 4:
	default:
		dw_writel(dws, offset, val);
		break;
	}
}

static inline void spi_enable_slave(struct dw_spi *dws, u32 slave_idx)
{
	u32 val;
	val = dw_readl(dws,DW_SPI_SER);
	val |= 1 << slave_idx;
	dw_writel(dws, DW_SPI_SER, val);
}
static inline void spi_disable_slave(struct dw_spi *dws, u32 slave_idx)
{
	u32 val;
	val = dw_readl(dws,DW_SPI_SER);
	val &= ~(1 << slave_idx);
	dw_writel(dws, DW_SPI_SER, val);
}
static inline void spi_enable_chip(struct dw_spi *dws, int enable)
{
	dw_writel(dws, DW_SPI_SSIENR, (enable ? 1 : 0));
}

static inline void spi_set_clk(struct dw_spi *dws, u16 div)
{
	dw_writel(dws, DW_SPI_BAUDR, div);
}

/* Disable IRQ bits */
static inline void spi_mask_intr(struct dw_spi *dws, u32 mask)
{
	u32 new_mask;

	new_mask = dw_readl(dws, DW_SPI_IMR) & ~mask;
	dw_writel(dws, DW_SPI_IMR, new_mask);
}

/* Enable IRQ bits */
static inline void spi_umask_intr(struct dw_spi *dws, u32 mask)
{
	u32 new_mask;

	new_mask = dw_readl(dws, DW_SPI_IMR) | mask;
	dw_writel(dws, DW_SPI_IMR, new_mask);
}

/*
 * This does disable the SPI controller, interrupts, and re-enable the
 * controller back. Transmit and receive FIFO buffers are cleared when the
 * device is disabled.
 */
static inline void spi_reset_chip(struct dw_spi *dws)
{
	spi_enable_chip(dws, 0);
	spi_mask_intr(dws, 0xff);
	spi_enable_chip(dws, 1);
}

static inline void spi_shutdown_chip(struct dw_spi *dws)
{
	spi_enable_chip(dws, 0);
	spi_set_clk(dws, 0);
}

/*
 * Each SPI slave device to work with dw_api controller should
 * has such a structure claiming its working mode (poll or PIO/DMA),
 * which can be save in the "controller_data" member of the
 * struct spi_device.
 */
struct dw_spi_chip {
	u8 poll_mode;	                /* 1 for controller polling mode */
	u8 type;	                    /* SPI/SSP/MicroWire */
	void (*cs_control)(u32 command);
};

extern void dw_qspi_set_cs(struct spi_device *spi,struct dw_spi *dws, bool enable);
extern int  dw_qspi_add_host(struct device *dev, struct dw_spi *dws);
extern void dw_qspi_remove_host(struct dw_spi *dws);
extern int  dw_qspi_suspend_host(struct dw_spi *dws);
extern int  dw_qspi_resume_host(struct dw_spi *dws);

#endif /* DW_QSPI_HEADER_H */
