// SPDX-License-Identifier: GPL-2.0-only
/*
 * Designware ehance-spi  core controller driver
 *
 * Copyright (c) 2021, alibaba-inc Corporation.
 *
 * base on design-ware spi-core driver(spi-dw-xxx.c)
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/highmem.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/iopoll.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/sizes.h>
#include <linux/spi/spi-mem.h>
#include "spi-dw-quad.h"
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

/* Slave spi_dev related */
struct chip_data {
	u8 tmode;		/* TR/TO/RO/EEPROM */
	u8 type;		/* SPI/SSP/MicroWire */

	u8 poll_mode;		/* 1 means use poll mode */

	u16 clk_div;		/* baud rate divider */
	u32 speed_hz;		/* baud rate of spi io clk */
	void (*cs_control)(u32 command);
};

#ifdef CONFIG_DEBUG_FS
#define SPI_REGS_BUFSIZE	1024
static ssize_t dw_qspi_show_regs(struct file *file, char __user *user_buf,
		size_t count, loff_t *ppos)
{
	struct dw_spi *dws = file->private_data;
	char *buf;
	u32 len = 0;
	ssize_t ret;

	buf = kzalloc(SPI_REGS_BUFSIZE, GFP_KERNEL);
	if (!buf)
		return 0;

	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"%s registers:\n", dev_name(&dws->master->dev));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"=================================\n");
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"CTRL0: \t\t0x%08x\n", dw_readl(dws, DW_SPI_CTRL0));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"CTRL1: \t\t0x%08x\n", dw_readl(dws, DW_SPI_CTRL1));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"SSIENR: \t0x%08x\n", dw_readl(dws, DW_SPI_SSIENR));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"SER: \t\t0x%08x\n", dw_readl(dws, DW_SPI_SER));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"BAUDR: \t\t0x%08x\n", dw_readl(dws, DW_SPI_BAUDR));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"TXFTLR: \t0x%08x\n", dw_readl(dws, DW_SPI_TXFLTR));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"RXFTLR: \t0x%08x\n", dw_readl(dws, DW_SPI_RXFLTR));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"TXFLR: \t\t0x%08x\n", dw_readl(dws, DW_SPI_TXFLR));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"RXFLR: \t\t0x%08x\n", dw_readl(dws, DW_SPI_RXFLR));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"SR: \t\t0x%08x\n", dw_readl(dws, DW_SPI_SR));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"IMR: \t\t0x%08x\n", dw_readl(dws, DW_SPI_IMR));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"ISR: \t\t0x%08x\n", dw_readl(dws, DW_SPI_ISR));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"DMACR: \t\t0x%08x\n", dw_readl(dws, DW_SPI_DMACR));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"DMATDLR: \t0x%08x\n", dw_readl(dws, DW_SPI_DMATDLR));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"DMARDLR: \t0x%08x\n", dw_readl(dws, DW_SPI_DMARDLR));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"RX_SAMPLE_DELAY: \t0x%08x\n", dw_readl(dws, DW_SPI_RX_SMP_DLY));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"SPI_CTRL0: \t0x%08x\n", dw_readl(dws, DW_SPI_SPI_CTRLR0));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"TXD_DRIVE_EDGE: \t0x%08x\n", dw_readl(dws, DW_SPI_TXD_DRV_EDGE));
	len += scnprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"=================================\n");

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret;
}

static const struct file_operations dw_qspi_regs_ops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.read		= dw_qspi_show_regs,
	.llseek		= default_llseek,
};

static int dw_qspi_debugfs_init(struct dw_spi *dws)
{
	char name[32];

	snprintf(name, 32, "dw_qspi%d", dws->master->bus_num);
	dws->debugfs = debugfs_create_dir(name, NULL);
	if (!dws->debugfs)
		return -ENOMEM;

	debugfs_create_file("registers", S_IFREG | S_IRUGO,
		dws->debugfs, (void *)dws, &dw_qspi_regs_ops);
	return 0;
}

static void dw_qspi_debugfs_remove(struct dw_spi *dws)
{
	debugfs_remove_recursive(dws->debugfs);
}

#else
static inline int dw_qspi_debugfs_init(struct dw_spi *dws)
{
	return 0;
}

static inline void dw_qspi_debugfs_remove(struct dw_spi *dws)
{
}
#endif /* CONFIG_DEBUG_FS */

void dw_qspi_set_cs(struct spi_device *spi,struct dw_spi *dws, bool enable)
{
	bool enable1 = !enable;

	if (dws->slave_cs) {
		/*
		 * Honour the SPI_NO_CS flag and invert the enable line, as
		 * active low is default for SPI. Execution paths that handle
		 * polarity inversion in gpiolib (such as device tree) will
		 * enforce active high using the SPI_CS_HIGH resulting in a
		 * double inversion through the code above.
		 */
		if (!(spi->mode & SPI_NO_CS)) {
			if (dws->slave_cs)
				gpiod_set_value_cansleep(dws->slave_cs, enable);
			else
				gpiod_set_value_cansleep(dws->slave_cs, enable1);
		}
		/* Some SPI masters need both GPIO CS & slave_select */
		if ((spi->controller->flags & SPI_MASTER_GPIO_SS) &&
		    spi->controller->set_cs)
			spi->controller->set_cs(spi, !enable);
	} else if (spi->controller->set_cs) {
		spi->controller->set_cs(spi, !enable);
	}
}

static void __maybe_unused set_cs(struct spi_device *spi, bool enable)
{
	struct dw_spi *dws = spi_controller_get_devdata(spi->controller);
	struct chip_data *chip = spi_get_ctldata(spi);

	/* Chip select logic is inverted from spi_set_cs() */
	if (chip && chip->cs_control)
		chip->cs_control(!enable);

	if (!enable)
		dw_writel(dws, DW_SPI_SER, BIT(spi->chip_select));
	else if (dws->cs_override)
		dw_writel(dws, DW_SPI_SER, 0);
}

/* Return the max entries we can fill into tx fifo */
static inline u32 tx_max(struct dw_spi *dws)
{
	u32 tx_left, tx_room, rxtx_gap;

	tx_left = (dws->tx_end - dws->tx) / dws->n_bytes;
	tx_room = dws->fifo_len - dw_readl(dws, DW_SPI_TXFLR);

	/*
	 * Another concern is about the tx/rx mismatch, we
	 * though to use (dws->fifo_len - rxflr - txflr) as
	 * one maximum value for tx, but it doesn't cover the
	 * data which is out of tx/rx fifo and inside the
	 * shift registers. So a control from sw point of
	 * view is taken.
	 */
	rxtx_gap =  ((dws->rx_end - dws->rx) - (dws->tx_end - dws->tx))
			/ dws->n_bytes;

	return min3(tx_left, tx_room, (u32) (dws->fifo_len - rxtx_gap));
}

/* Return the max entries we should read out of rx fifo */
static inline u32 rx_max(struct dw_spi *dws)
{
	u32 rx_left = (dws->rx_end - dws->rx) / dws->n_bytes;

	return min_t(u32, rx_left, dw_readl(dws, DW_SPI_RXFLR));
}

static void dw_writer(struct dw_spi *dws)
{
	u32 max;
	u32 txw = 0;

	spin_lock(&dws->buf_lock);
	max = tx_max(dws);
	while (max--) {
		/* Set the tx word if the transfer's original "tx" is not null */
		if (dws->tx_end - dws->len) {
			if (dws->n_bytes == 1)
				txw = *(u8 *)(dws->tx);
			else if(dws->n_bytes == 2)
				txw = *(u16 *)(dws->tx);
			else
				txw = *(u32 *)(dws->tx);
		}
		dw_write_io_reg(dws, DW_SPI_DR, txw);
		dws->tx += dws->n_bytes;
	}
	spin_unlock(&dws->buf_lock);
}

static void dw_reader(struct dw_spi *dws)
{
	u32 max;
	u32 rxw;

	spin_lock(&dws->buf_lock);
	max = rx_max(dws);
	while (max--) {
		rxw = dw_read_io_reg(dws, DW_SPI_DR);
		/* Care rx only if the transfer's original "rx" is not null */
		if (dws->rx_end - dws->len) {
			if (dws->n_bytes == 1)
				*(u8 *)(dws->rx) = rxw;
			else if(dws->n_bytes ==2)
				*(u16 *)(dws->rx) = rxw;
			else
				*(u32 *)(dws->rx) = rxw;
		}
		dws->rx += dws->n_bytes;
	}
	spin_unlock(&dws->buf_lock);
}

static irqreturn_t dw_qspi_irq(int irq, void *dev_id)
{
	struct spi_controller *master = dev_id;
	struct dw_spi *dws = spi_controller_get_devdata(master);
	u16 irq_status = dw_readl(dws, DW_SPI_ISR) & 0x3f;

	if (!irq_status)
		return IRQ_NONE;

	if (!master->cur_msg) {
		spi_mask_intr(dws, SPI_INT_TXEI);
		return IRQ_HANDLED;
	}

	return dws->transfer_handler(dws);
}

static void dw_qspi_handle_err(struct spi_controller *master,
		struct spi_message *msg)
{
	struct dw_spi *dws = spi_controller_get_devdata(master);

	if (dws->dma_mapped)
		dws->dma_ops->dma_stop(dws);

	spi_reset_chip(dws);
}

/* This may be called twice for each spi dev */
static int dw_qspi_setup(struct spi_device *spi)
{
	struct dw_spi_chip *chip_info = NULL;
	struct chip_data *chip;

	/* Only alloc on first setup */
	chip = spi_get_ctldata(spi);
	if (!chip) {
		chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;
		spi_set_ctldata(spi, chip);
	}

	/*
	 * Protocol drivers may change the chip settings, so...
	 * if chip_info exists, use it
	 */
	chip_info = spi->controller_data;

	/* chip_info doesn't always exist */
	if (chip_info) {
		if (chip_info->cs_control)
			chip->cs_control = chip_info->cs_control;

		chip->poll_mode = chip_info->poll_mode;
		chip->type = chip_info->type;
	}

	/* use both transmit and receive mode by default */
	chip->tmode = SPI_TMOD_TR;

	return 0;
}

static void dw_qspi_cleanup(struct spi_device *spi)
{
	struct chip_data *chip = spi_get_ctldata(spi);

	kfree(chip);
	spi_set_ctldata(spi, NULL);
}

/* Restart the controller, disable all interrupts, clean rx fifo */
static void qspi_hw_init(struct device *dev, struct dw_spi *dws)
{
	spi_reset_chip(dws);

	/*
	 * Try to detect the FIFO depth if not set by interface driver,
	 * the depth could be from 2 to 256 from HW spec
	 */
	if (!dws->fifo_len) {
		u32 fifo;

		for (fifo = 1; fifo < 256; fifo++) {
			dw_writel(dws, DW_SPI_TXFLTR, fifo);
			if (fifo != dw_readl(dws, DW_SPI_TXFLTR))
				break;
		}
		dw_writel(dws, DW_SPI_TXFLTR, 0);
		dw_writel(dws,DW_SPI_SER,0x3);
		dws->fifo_len = (fifo == 1) ? 0 : fifo;
		dev_dbg(dev, "Detected FIFO size: %u uint32 \n", dws->fifo_len);
	}
	spi_enable_chip(dws,0);
	dw_writel(dws, DW_SPI_RX_SMP_DLY, dws->rx_sample_delay);
	spi_enable_chip(dws,1);
}

static int dw_qspi_adjust_op_size(struct spi_mem *mem, struct spi_mem_op *op)
{
	struct dw_spi *dws = spi_controller_get_devdata(mem->spi->master);
	/* dw spi's rx and tx have the same fifo depth */

	if (op->data.dir == SPI_MEM_DATA_OUT && op->data.nbytes >= ((dws->fifo_len -2)<<2) ) {
		op->data.nbytes = ((dws->fifo_len -2)<<2);
	}

	if (op->data.dir == SPI_MEM_DATA_IN && op->data.nbytes >= (dws->fifo_len<<2) ) {
		op->data.nbytes = (dws->fifo_len<<2);
	}
	return 0;

}

static int dw_qspi_check_buswidth(struct dw_spi *dws, u8 width)
{
	switch (width) {
	case 1:
	case 2:
	case 4:
		return 0;
	}

	return -ENOTSUPP;
}
static bool dw_qspi_supports_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct dw_spi *dws = spi_controller_get_devdata(mem->spi->master);
	int ret=0;
	u32 temp_len;
	/* check buswidth */

	if(op->cmd.buswidth!=1 || \
	  (op->addr.nbytes && op->addr.buswidth !=1) || \
	  ( op->dummy.nbytes && op->dummy.buswidth !=1))
	  {
		  return false;
	  }
	if (op->data.nbytes)
		ret |= dw_qspi_check_buswidth(dws, op->data.buswidth);

	if (ret){
		return false;
	}
    /* check addr bits length */
	temp_len = op->addr.nbytes << 3;
	if(op->data.nbytes && op->data.buswidth > 4 && temp_len > 60 ){
		return false;
	}

	return true;
}

static int dw_qspi_readl_poll_time_out(struct dw_spi *dws,u32 delay_us, u32 timeout_us)
{
	u32 val;
	void __iomem *reg = dws->regs + DW_SPI_SR;
	return readl_poll_timeout(reg,val,!(val & SR_BUSY),delay_us,timeout_us);
}

static void dw_qspi_build_xfer_pre_portion(struct dw_spi *dws,const struct spi_mem_op *op)
{
    u32 i = 0,j=0;

    /* operation code */
    dws->xfer_data_pre.xfer_pre[i++]= op->cmd.opcode;

    /* addr */
    if(op->addr.nbytes){
        for(j=0; j < op->addr.nbytes; j++)
        {
            dws->xfer_data_pre.xfer_pre[i++] = (op->addr.val >> (8 *(op->addr.nbytes -j -1))) & 0xFF;
        }
    }

    /* dummy */
    if(op->dummy.nbytes){
        memset(&dws->xfer_data_pre.xfer_pre[i],0xFF,op->dummy.nbytes);
        i += op->dummy.nbytes;
    }

    dws->xfer_data_pre.xfer_pre_len = i;
	return;
}

static bool dw_qspi_can_xfer_32bits_frame(const struct spi_mem_op *op)
{
	bool ret = false;

	if(op->data.buswidth > 1 && op->data.nbytes && !(op->data.nbytes & 0x3) && !(op->data.nbytes & 0x3)){
		if(op->data.dir == SPI_MEM_DATA_OUT && !((unsigned long)(op->data.buf.out) & 0x03) )
		{
			ret = true;
		}
		else if(op->data.dir == SPI_MEM_DATA_IN && !((unsigned long)(op->data.buf.in) & 0x03) ){
			ret = true;
		}
	}

	return ret;
}

static int dw_qspi_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct dw_spi *dws = spi_controller_get_devdata(mem->spi->master);
	struct spi_device *spi = mem->spi;
	struct chip_data *chip = spi_get_ctldata(mem->spi);
	int    ret=0;
	u32    cr0=0,spi_cr0=0;
	u32    rx_len;
	u32 addr_bits_len, dummy_bits_len;
	/* wait for the controller being ready */
	ret = dw_qspi_readl_poll_time_out(dws,10,1000*1000);
	if(ret){
		dev_err(&dws->master->dev, "time out during waiting for spi-core idle\n");
		return ret;
	}

	/*disable spi core */
	spi_enable_chip(dws,0);
	if(op->data.dir == SPI_MEM_DATA_OUT) {
		spi_disable_slave(dws,0);
	}else {
		spi_enable_slave(dws,0);
	}

	/* config clk rate, div = 8 */
    //printk("dws->max_freq = %d spi->max_speed_hz %d \n",dws->max_freq,spi->max_speed_hz);
	chip->clk_div = (DIV_ROUND_UP(dws->max_freq,spi->max_speed_hz)+1) & 0xfffe;
	chip->speed_hz = dws->current_freq = spi->max_speed_hz;
	spi_set_clk(dws,chip->clk_div);

	/* build pre-xfer data portion*/
	dw_qspi_build_xfer_pre_portion(dws,op);

	/*
	 * default cr0 register setting
	 * SSI_TYPE-----> SPI_FRF_SPI
	 * CPOL&CPHA----> SPI_CPOL=SPI_CPHA = 0
	 * FRAME_SIZE---> 8bits
	 * SPI_TMOD_OFFSET SPI_TMOD_TO
	 */

	cr0 = (chip->type << SPI_FRF_OFFSET) | \
	      (((spi->mode & SPI_CPOL) ? 1 : 0) << SPI_SCOL_OFFSET) | \
		  (((spi->mode & SPI_CPHA) ? 1 : 0) << SPI_SCPH_OFFSET) | \
	      (chip->tmode << SPI_TMOD_OFFSET) | \
		  ((8-1) << SPI_DFS32_OFFSET) ;

	chip->tmode = SPI_TMOD_TO;
	if(op->data.dir == SPI_MEM_NO_DATA || op->data.dir == SPI_MEM_DATA_OUT){
		chip->tmode = SPI_TMOD_TO;
	}
	else if(op->data.dir == SPI_MEM_DATA_IN && op->data.buswidth > 1){
		chip->tmode = SPI_TMOD_RO;
		    rx_len = op->data.nbytes;
	    if(dw_qspi_can_xfer_32bits_frame(op)){
			rx_len >>=2;
		}
		dw_writel(dws,DW_SPI_CTRL1,rx_len -1);
	}
	cr0 &=~SPI_TMOD_MASK;
	cr0 |=(chip->tmode << SPI_TMOD_OFFSET);

	/* init config spi_cr0 if use non-standard spi mode */
	if(op->data.buswidth > 1){
		/*
		 * trans_type = both instruction and address are sent in standard mode
		 * instruction bits length is 8bits
		 *
		 */
		spi_cr0 = (SPI_CTRLR0_TRANS_ISTD_ASTD << SPI_CTRLR0_TRNAS_OFFET) | \
		          (SPI_CTRLR0_INST_L_8 << SPI_CTRLR0_INST_L_OFFSET);

		addr_bits_len = (op->addr.nbytes << 3)>>2;
		dummy_bits_len = op->dummy.nbytes << 3;
		spi_cr0 |= (addr_bits_len << SPI_CTRLR0_ADDR_L_OFFSET);
		spi_cr0 |= (dummy_bits_len << SPI_CTRLR0_WAIT_CYCLES_OFFSET);
		dw_writel(dws,DW_SPI_SPI_CTRLR0,spi_cr0);

		/* update SPI_SPI_FRF */
		cr0 &=~ SPI_SPI_FRF_MASK;
		switch(op->data.buswidth){
			case 2:
				cr0 |= (SPI_SPI_FRF_DUAL << SPI_SPI_FRF_OFFSET);
			break;
			case 4:
				cr0 |= (SPI_SPI_FRF_QUAD << SPI_SPI_FRF_OFFSET);
			default:
			break;
		}
	}

	/*check whether can xfer through 32bit-frame size, then update cr0 */
	if(dw_qspi_can_xfer_32bits_frame(op)){
		cr0 &=~SPI_DFS32_MASK;
		cr0 |= ((32-1) << SPI_DFS32_OFFSET );
	}
	dw_writel(dws,DW_SPI_CTRL0,cr0);
	/*  for poll mode just disable all interrupts */
	spi_mask_intr(dws,0xff);

	/* dma mode  */
	#if 0
	if (dws->dma_mapped) {
		ret = dws->dma_setup(dws, &dws->xfer_data_pre,op);
		if (ret < 0) {
			spi_enable_chip(dws, 1);
			return ret;
		}
    }
	#endif

	/* transfer data_pre portion(cmd+addr+dummy) */
	dw_qspi_set_cs(mem->spi,dws,true);
	spi_enable_chip(dws,1);

	/* pre portion is sent with spi-standard mode and data-frame size is 8bit frame */
	dws->n_bytes = 1;
	if(op->data.nbytes && op->data.buswidth > 1)
	{
		/*!!!note: in quad mode, dw-ssi can't be interrupt during sending pre portion and data portion
		 * otherwise, the timing won't be expected
		 */
		dw_writel(dws,DW_SPI_DR,op->cmd.opcode);
		if(op->addr.nbytes){
		dw_writel(dws,DW_SPI_DR,op->addr.val);
		}
	}
	else
	{
		dws->tx = (void *)dws->xfer_data_pre.xfer_pre;
		dws->tx_end = dws->tx  + dws->xfer_data_pre.xfer_pre_len;
		dws->len = dws->xfer_data_pre.xfer_pre_len;
		do{
			dw_writer(dws);
		}while(dws->tx_end > dws->tx);
	}

	/* transfer data portion if needs */
	if(op->data.dir == SPI_MEM_DATA_OUT){
		dws->tx =(void*) op->data.buf.out;
		dws->tx_end = dws->tx + op->data.nbytes;
		dws->len = op->data.nbytes;
		if(dw_qspi_can_xfer_32bits_frame(op)){
			dws->n_bytes = 4;
		}
		do{
			dw_writer(dws);
		}while(dws->tx_end > dws->tx);
		if(op->data.nbytes && op->data.buswidth > 1){
			//local_irq_restore(flag);
		}
		spi_enable_slave(dws,0);
	}
	else if (op->data.dir == SPI_MEM_DATA_IN)
	{
		/* if data portion use standard mode,pre transfer mode is tx, need set rx mode  */
		if(op->data.buswidth == 1){
			while(dw_readl(dws,DW_SPI_SR) & SR_BUSY){
				cpu_relax();
			}
			spi_enable_chip(dws,0);
			cr0 = dw_readl(dws,DW_SPI_CTRL0);
			cr0 &=~SPI_TMOD_MASK;
			cr0 |= (SPI_TMOD_RO << SPI_TMOD_OFFSET);
			dw_writel(dws,DW_SPI_CTRL0,cr0);
			dw_writel(dws,DW_SPI_CTRL1,op->data.nbytes -1);
			spi_enable_chip(dws,1);
			dw_write_io_reg(dws,DW_SPI_DR,0);
			dws->rx = op->data.buf.in;
			dws->rx_end = dws->rx + op->data.nbytes;
			dws->len = op->data.nbytes;
			do{
				dw_reader(dws);
				//cpu_relax();
			}while(dws->rx_end > dws->rx);
		}
		/* non-standard mode */
		else {

			dws->rx = op->data.buf.in;
			dws->rx_end = dws->rx + op->data.nbytes;
			dws->len = op->data.nbytes;
	        if(dw_qspi_can_xfer_32bits_frame(op)) {
			   dws->n_bytes = 4;
            }
			do {
				dw_reader(dws);
			}while(dws->rx_end > dws->rx);
			if(op->data.nbytes && op->data.buswidth > 1) {
				//local_irq_restore(flag);
			}
		}

	}

	while(dw_readl(dws,DW_SPI_SR) & SR_BUSY) {
		cpu_relax();
	}

	if(dw_readl(dws,DW_SPI_RISR) &(SPI_INT_RXOI|SPI_INT_RXUI) ) {
		printk("###rx err %02x \n",dw_readl(dws,DW_SPI_RISR));
	}
	dw_qspi_set_cs(mem->spi,dws,false);
	return 0;
}

static const struct spi_controller_mem_ops dw_qspi_mem_ops = {
	.adjust_op_size = dw_qspi_adjust_op_size,
	.supports_op =    dw_qspi_supports_op,
	.exec_op =        dw_qspi_exec_op,
	.get_name =       NULL,/*dw_qspi_get_name,*/
};

int dw_qspi_add_host(struct device *dev, struct dw_spi *dws)
{
	struct spi_controller *master;
	int ret;

	BUG_ON(dws == NULL);

	/*  allocate master */
	master = spi_alloc_master(dev, 0);
	if (!master)
		return -ENOMEM;

	dws->master = master;
	dws->type = SSI_MOTO_SPI;
	dws->dma_inited = 0;
	dws->reg_io_width = 4;
	dws->dma_addr = (dma_addr_t)(dws->paddr + DW_SPI_DR);
	spin_lock_init(&dws->buf_lock);

	/* save device data */
	spi_controller_set_devdata(master, dws);

	/*  register interrupt call-back */
	ret = request_irq(dws->irq, dw_qspi_irq, IRQF_SHARED, dev_name(dev),
			  master);
	if (ret < 0) {
		dev_err(dev, "can not get IRQ\n");
		goto err_free_master;
	}

	master->mode_bits = SPI_CPOL |\
			    SPI_CPHA |\
	                    SPI_RX_DUAL |\
			    SPI_RX_QUAD | \
		            SPI_TX_DUAL| \
			    SPI_TX_QUAD;
	master->bits_per_word_mask =  SPI_BPW_RANGE_MASK(4, 32);
	master->bus_num = dws->bus_num;
	master->num_chipselect = dws->num_cs;
	master->setup = dw_qspi_setup;
	master->cleanup = dw_qspi_cleanup;
	master->handle_err = dw_qspi_handle_err;
	master->max_speed_hz = dws->max_freq;
	master->dev.of_node = dev->of_node;
	master->dev.fwnode = dev->fwnode;

	if (dws->set_cs)
		master->set_cs = dws->set_cs;

	/* Basic HW init */
	qspi_hw_init(dev, dws);

	if (dws->dma_ops && dws->dma_ops->dma_init) {
		ret = dws->dma_ops->dma_init(dws);
		if (ret) {
			dev_warn(dev, "DMA init failed\n");
			dws->dma_inited = 0;
		} else {
			master->can_dma = dws->dma_ops->can_dma;
		}
	}

	/* set qspi-mem ops table */
	master->mem_ops = &dw_qspi_mem_ops;
	/* regist master controler */
	ret = devm_spi_register_controller(dev, master);
	if (ret) {
		dev_err(&master->dev, "problem registering spi master\n");
		goto err_dma_exit;
	}

	/* init debugfs feature */
	dw_qspi_debugfs_init(dws);
	return 0;

err_dma_exit:
	if (dws->dma_ops && dws->dma_ops->dma_exit)
		dws->dma_ops->dma_exit(dws);
	spi_enable_chip(dws, 0);
	free_irq(dws->irq, master);
err_free_master:
	spi_controller_put(master);
	return ret;
}
EXPORT_SYMBOL_GPL(dw_qspi_add_host);

void dw_qspi_remove_host(struct dw_spi *dws)
{
	dw_qspi_debugfs_remove(dws);

	if (dws->dma_ops && dws->dma_ops->dma_exit)
		dws->dma_ops->dma_exit(dws);

	spi_shutdown_chip(dws);

	free_irq(dws->irq, dws->master);
}
EXPORT_SYMBOL_GPL(dw_qspi_remove_host);

int dw_qspi_suspend_host(struct dw_spi *dws)
{
	int ret;

	ret = spi_controller_suspend(dws->master);
	if (ret)
		return ret;

	spi_shutdown_chip(dws);
	return 0;
}
EXPORT_SYMBOL_GPL(dw_qspi_suspend_host);

int dw_qspi_resume_host(struct dw_spi *dws)
{
	qspi_hw_init(&dws->master->dev, dws);
	return spi_controller_resume(dws->master);
}
EXPORT_SYMBOL_GPL(dw_qspi_resume_host);

MODULE_AUTHOR("linghui zeng<linghui.zlh@linux.alibaba.com>");
MODULE_DESCRIPTION("Driver for DesignWare ehance-spi controller core");
MODULE_LICENSE("GPL v2");
