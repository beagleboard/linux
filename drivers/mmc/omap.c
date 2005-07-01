/*
 *  linux/drivers/media/mmc/omap.c
 *
 *  Copyright (C) 2004 Nokia Corporation
 *  Written by Tuukka Tikkanen and Juha Yrjölä <juha.yrjola@nokia.com>
 *  Pin multiplexing and Innovator support by Tony Lindgren <tony@atomide.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/mmc/host.h>
#include <linux/mmc/protocol.h>
#include <linux/mmc/card.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/scatterlist.h>
#include <asm/mach-types.h>

#include <asm/arch/board.h>
#include <asm/arch/gpio.h>
#include <asm/arch/dma.h>
#include <asm/arch/mux.h>
#include <asm/arch/fpga.h>
#include <asm/arch/tps65010.h>

#include <asm/hardware/clock.h>

#include "omap.h"

#define DRIVER_NAME "mmci-omap"

#ifdef CONFIG_MMC_DEBUG
#define DBG(x...)	pr_debug(x)
//#define DBG(x...)	printk(x)
#else
#define DBG(x...)	do { } while (0)
#endif

/* Specifies how often in millisecs to poll for card status changes
 * when the cover switch is open */
#define OMAP_MMC_SWITCH_POLL_DELAY     500

static int mmc_omap_enable_poll = 1;

struct mmc_omap_host {
	int                     initialized;
	int			suspended;
	struct mmc_request *	mrq;
	struct mmc_command *	cmd;
	struct mmc_data *	data;
	struct mmc_host *	mmc;
	struct device *		dev;
	unsigned char		id; /* 16xx chips have 2 MMC blocks */
	struct clk *		clk;
	void __iomem		*base;
	int irq;
	unsigned char		bus_mode;
	unsigned int		dma_len;
#define OMAP_MMC_DATADIR_NONE	0
#define OMAP_MMC_DATADIR_READ	1
#define OMAP_MMC_DATADIR_WRITE	2
	unsigned char		datadir;
	u16 * buffer;
	u32 bytesleft;
	short			power_pin;
	short			wp_pin;
	short			use_dma, dma_ch;
	struct completion       dma_completion;

	int			switch_pin;
	struct work_struct	switch_work;
	struct timer_list	switch_timer;
	int			switch_last_state;
};

static inline int
mmc_omap_cover_is_open(struct mmc_omap_host *host)
{
	if (host->switch_pin < 0)
		return 0;
        return omap_get_gpio_datain(host->switch_pin);
}

static ssize_t
mmc_omap_show_cover_switch(struct device *dev, char *buf)
{
	struct mmc_omap_host *host = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", mmc_omap_cover_is_open(host) ? "open" : "closed");
}

static DEVICE_ATTR(cover_switch, S_IRUGO, mmc_omap_show_cover_switch, NULL);

static ssize_t
mmc_omap_show_enable_poll(struct device *dev, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mmc_omap_enable_poll);
}

static ssize_t
mmc_omap_store_enable_poll(struct device *dev, const char *buf, size_t size)
{
	int enable_poll;

	if (sscanf(buf, "%10d", &enable_poll) != 1)
		return -EINVAL;

	if (enable_poll != mmc_omap_enable_poll) {
		struct mmc_omap_host *host = dev_get_drvdata(dev);

		mmc_omap_enable_poll = enable_poll;
		if (enable_poll && host->switch_pin >= 0)
			schedule_work(&host->switch_work);
	}
	return size;
}

static DEVICE_ATTR(enable_poll, 0664,
		   mmc_omap_show_enable_poll, mmc_omap_store_enable_poll);

static void
mmc_omap_start_command(struct mmc_omap_host *host, struct mmc_command *cmd)
{
	u32 cmdreg;
	u32 resptype;
	u32 cmdtype;

	DBG("MMC%d: CMD%d, argument 0x%08x", host->id, cmd->opcode, cmd->arg);
	if (cmd->flags & MMC_RSP_SHORT)
		DBG(", 32-bit response");
	if (cmd->flags & MMC_RSP_LONG)
		DBG(", 128-bit response");
	if (cmd->flags & MMC_RSP_CRC)
		DBG(", CRC");
	if (cmd->flags & MMC_RSP_BUSY)
		DBG(", busy notification");
	DBG("\n");

	host->cmd = cmd;

	resptype = 0;
	cmdtype = 0;

	/* Protocol layer does not provide response type,
	 * but our hardware needs to know exact type, not just size!
	 */
	switch (cmd->flags & MMC_RSP_MASK) {
	case MMC_RSP_NONE:
		/* resp 0 */
		break;
	case MMC_RSP_SHORT:
		/* resp 1, resp 1b */
		/* OR resp 3!! (assume this if bus is set opendrain) */
		if (host->bus_mode == MMC_BUSMODE_OPENDRAIN)
			resptype = 3;
		else
			resptype = 1;
		break;
	case MMC_RSP_LONG:
		/* resp 2 */
		resptype = 2;
		break;
	}

	/* Protocol layer does not provide command type, but our hardware
	 * needs it!
	 * any data transfer means adtc type (but that information is not
	 * in command structure, so we flagged it into host struct.)
	 * However, telling bc, bcr and ac apart based on response is
	 * not foolproof:
	 * CMD0  = bc  = resp0  CMD15 = ac  = resp0
	 * CMD2  = bcr = resp2  CMD10 = ac  = resp2
	 *
	 * Resolve to best guess with some exception testing:
	 * resp0 -> bc, except CMD15 = ac
	 * rest are ac, except if opendrain
	 */
	if (host->datadir) {
		cmdtype = OMAP_MMC_CMDTYPE_ADTC;
	} else if (resptype == 0 && cmd->opcode != 15) {
		cmdtype = OMAP_MMC_CMDTYPE_BC;
	} else if (host->bus_mode == MMC_BUSMODE_OPENDRAIN) {
		cmdtype = OMAP_MMC_CMDTYPE_BCR;
	} else {
		cmdtype = OMAP_MMC_CMDTYPE_AC;
	}

	cmdreg = cmd->opcode | (resptype << 8) | (cmdtype << 12);

	if (host->bus_mode == MMC_BUSMODE_OPENDRAIN)
		cmdreg |= 1 << 6;

	if (cmd->flags & MMC_RSP_BUSY)
		cmdreg |= 1 << 11;

	if (host->datadir == OMAP_MMC_DATADIR_READ)
		cmdreg |= 1 << 15;

	clk_use(host->clk);

	OMAP_MMC_WRITE(host->base, CTO, 200);
	OMAP_MMC_WRITE(host->base, ARGL, cmd->arg & 0xffff);
	OMAP_MMC_WRITE(host->base, ARGH, cmd->arg >> 16);
	OMAP_MMC_WRITE(host->base, IE,
		       OMAP_MMC_STAT_A_EMPTY    | OMAP_MMC_STAT_A_FULL    |
		       OMAP_MMC_STAT_CMD_CRC    | OMAP_MMC_STAT_CMD_TOUT  |
		       OMAP_MMC_STAT_DATA_CRC   | OMAP_MMC_STAT_DATA_TOUT |
		       OMAP_MMC_STAT_END_OF_CMD | OMAP_MMC_STAT_CARD_ERR  |
		       OMAP_MMC_STAT_END_OF_DATA);
	OMAP_MMC_WRITE(host->base, CMD, cmdreg);
}

static void
mmc_omap_xfer_done(struct mmc_omap_host *host, struct mmc_data *data)
{
	enum dma_data_direction dma_data_dir;

	host->data = NULL;
	host->datadir = OMAP_MMC_DATADIR_NONE;

	if (data->error == MMC_ERR_NONE)
		data->bytes_xfered += data->blocks * (1<<data->blksz_bits);
	else {
		int dma_ch = host->dma_ch;

		/* We got an error, let's free the DMA channel if it's
		 * still allocated. */
		if (dma_ch != -1) {
			host->dma_ch = -1;
			omap_free_dma(dma_ch);
		}
	}

	if (host->datadir == OMAP_MMC_DATADIR_WRITE)
		dma_data_dir = DMA_TO_DEVICE;
	else
		dma_data_dir = DMA_FROM_DEVICE;

	dma_unmap_sg(mmc_dev(host->mmc), data->sg, host->dma_len,
		     dma_data_dir);

	host->dma_len = 0;

	clk_unuse(host->clk);

	if (!data->stop) {
		host->mrq = NULL;
		mmc_request_done(host->mmc, data->mrq);
		return;
	}

	mmc_omap_start_command(host, data->stop);
}


static void
mmc_omap_cmd_done(struct mmc_omap_host *host, struct mmc_command *cmd)
{
	host->cmd = NULL;

	switch (cmd->flags & MMC_RSP_MASK) {
	case MMC_RSP_NONE:
		/* resp 0 */
		break;
	case MMC_RSP_SHORT:
		/* response types 1, 1b, 3, 4, 5, 6 */
		cmd->resp[0] =
			OMAP_MMC_READ(host->base, RSP6) |
			(OMAP_MMC_READ(host->base, RSP7) << 16);
		DBG("MMC%d: Response %08x\n", host->id, cmd->resp[0]);
		break;
	case MMC_RSP_LONG:
		/* response type 2 */
		cmd->resp[3] =
			OMAP_MMC_READ(host->base, RSP0) |
			(OMAP_MMC_READ(host->base, RSP1) << 16);
		cmd->resp[2] =
			OMAP_MMC_READ(host->base, RSP2) |
			(OMAP_MMC_READ(host->base, RSP3) << 16);
		cmd->resp[1] =
			OMAP_MMC_READ(host->base, RSP4) |
			(OMAP_MMC_READ(host->base, RSP5) << 16);
		cmd->resp[0] =
			OMAP_MMC_READ(host->base, RSP6) |
			(OMAP_MMC_READ(host->base, RSP7) << 16);
		DBG("MMC%d: Response %08x %08x %08x %08x\n", host->id,
		    cmd->resp[0], cmd->resp[1],
		    cmd->resp[2], cmd->resp[3]);
		break;
	}

	if (host->data == NULL || cmd->error != MMC_ERR_NONE) {
		DBG("MMC%d: End request, err %x\n", host->id, cmd->error);
		host->mrq = NULL;
		clk_unuse(host->clk);
		mmc_request_done(host->mmc, cmd->mrq);
	}
}

static irqreturn_t mmc_omap_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	struct mmc_omap_host * host = (struct mmc_omap_host *)dev_id;
	u16 status;
	int end_command;
	int end_transfer;
	int ii;

	if (host->cmd == NULL && host->data == NULL) {
		status = OMAP_MMC_READ(host->base, STAT);
		printk(KERN_INFO "MMC%d: Spurious interrupt 0x%04x\n", host->id, status);
		if (status != 0) {
			OMAP_MMC_WRITE(host->base, STAT, status);
			OMAP_MMC_WRITE(host->base, IE, 0);
		}
		return IRQ_HANDLED;
	}

	end_command = 0;
	end_transfer = 0;

	while ((status = OMAP_MMC_READ(host->base, STAT)) != 0) {
		OMAP_MMC_WRITE(host->base, STAT, status); // Reset status bits
		DBG("\tMMC IRQ %04x (cmd %d)\n", status,
		    host->cmd != NULL ? host->cmd->opcode : -1);

		if ((status & OMAP_MMC_STAT_A_FULL) ||
		    ((status & OMAP_MMC_STAT_END_OF_DATA) &&
		     (host->bytesleft > 0))) {
			// Buffer almost full
			ii = host->bytesleft / 2;
			if (ii > 32)
				ii = 32;
			host->bytesleft -= ii * 2;
			while (ii-- > 0)
				*host->buffer++ = OMAP_MMC_READ(host->base, DATA);
		}

		if (status & OMAP_MMC_STAT_A_EMPTY) {
			// Buffer almost empty
			ii = host->bytesleft / 2;
			if (ii > 32)
				ii = 32;
			host->bytesleft -= ii * 2;
			while (ii-- > 0)
				OMAP_MMC_WRITE(host->base, DATA, *host->buffer++);
		}

		if (status & OMAP_MMC_STAT_END_OF_DATA) {
			// Block sent/received
			end_transfer = 1;
		}

		if (status & OMAP_MMC_STAT_DATA_TOUT) {
			// Data timeout
			printk(KERN_DEBUG "MMC%d: Data timeout\n", host->id);
			if (host->data) {
				host->data->error |= MMC_ERR_TIMEOUT;
				end_transfer = 1;
			}
		}

		if (status & OMAP_MMC_STAT_DATA_CRC) {
			// Data CRC error
			if (host->data) {
				host->data->error |= MMC_ERR_BADCRC;
				printk(KERN_DEBUG "MMC%d: Data CRC error, bytes left %d\n",
				       host->id, host->bytesleft);
				end_transfer = 1;
			} else {
				printk(KERN_DEBUG "MMC%d: Data CRC error\n",
				       host->id);
			}
		}

		if (status & OMAP_MMC_STAT_CMD_TOUT) {
			/* Timeouts are routine with some commands */
			if (host->cmd) {
				if (host->cmd->opcode != MMC_ALL_SEND_CID &&
				    host->cmd->opcode != MMC_SEND_OP_COND &&
				    host->cmd->opcode != MMC_APP_CMD &&
				    !mmc_omap_cover_is_open(host))
					printk(KERN_ERR "MMC%d: Command timeout, CMD%d\n",
					       host->id, host->cmd->opcode);
				host->cmd->error |= MMC_ERR_TIMEOUT;
				end_command = 1;
			}
		}

		if (status & OMAP_MMC_STAT_CMD_CRC) {
			// Command CRC error
			printk(KERN_ERR "MMC%d: Command CRC error\n", host->id);
			if (host->cmd) {
				host->cmd->error |= MMC_ERR_BADCRC;
				end_command = 1;
			}
		}

		if (status & OMAP_MMC_STAT_OCR_BUSY) {
			/* OCR Busy ... happens a lot */
			if (host->cmd && host->cmd->opcode != MMC_SEND_OP_COND
				&& host->cmd->opcode != MMC_SET_RELATIVE_ADDR) {
				DBG("MMC%d: OCR busy error, CMD%d\n",
				       host->id, host->cmd->opcode);
			}
		}

		if (status & OMAP_MMC_STAT_CARD_ERR) {
			// Card status error
			printk(KERN_DEBUG "MMC%d: Card status error (CMD%d)\n",
			       host->id, host->cmd->opcode);
			if (host->cmd) {
				host->cmd->error |= MMC_ERR_FAILED;
				end_command = 1;
			}
			if (host->data) {
				host->data->error |= MMC_ERR_FAILED;
				end_transfer = 1;
			}
		}

		/*
		 * NOTE: On 1610 the END_OF_CMD may come too early when
		 *       starting a write 
		 */
		if ((status & OMAP_MMC_STAT_END_OF_CMD) &&
		    (!(status & OMAP_MMC_STAT_A_EMPTY))) {
			// End of command phase
			end_command = 1;
		}
	}

	if (end_command) {
		mmc_omap_cmd_done(host, host->cmd);
	}
	if (end_transfer) {
		mmc_omap_xfer_done(host, host->data);
	}

	return IRQ_HANDLED;
}

static irqreturn_t mmc_omap_switch_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	struct mmc_omap_host *host = (struct mmc_omap_host *) dev_id;

	DBG("MMC%d cover is now %s\n", host->id,
	    omap_get_gpio_datain(host->switch_pin) ? "open" : "closed");
	schedule_work(&host->switch_work);

	return IRQ_HANDLED;
}

static void mmc_omap_switch_timer(unsigned long arg)
{
	struct mmc_omap_host *host = (struct mmc_omap_host *) arg;

	schedule_work(&host->switch_work);
}

static void mmc_omap_switch_handler(void *data)
{
	struct mmc_omap_host *host = (struct mmc_omap_host *) data;
	struct mmc_card *card;
	static int complained = 0;
	int cards = 0, cover_open;

	if (host->switch_pin == -1)
		return;
	cover_open = mmc_omap_cover_is_open(host);
	if (cover_open != host->switch_last_state) {
		kobject_uevent(&host->dev->kobj, KOBJ_CHANGE, &dev_attr_cover_switch.attr);
		host->switch_last_state = cover_open;
	}
	DBG("MMC cover switch handler started\n");
	mmc_detect_change(host->mmc);
	list_for_each_entry(card, &host->mmc->cards, node) {
		if (mmc_card_present(card))
			cards++;
	}
	DBG("MMC%d: %d card(s) present\n", host->id, cards);
	if (mmc_omap_cover_is_open(host)) {
		if (!complained) {
			printk(KERN_INFO "MMC%d: cover is open\n", host->id);
			complained = 1;
		}
		if (mmc_omap_enable_poll)
			mod_timer(&host->switch_timer, jiffies +
				msecs_to_jiffies(OMAP_MMC_SWITCH_POLL_DELAY));
	} else {
		complained = 0;
	}
}

static void mmc_omap_dma_cb(int lch, u16 ch_status, void *data)
{
	struct mmc_omap_host *host = (struct mmc_omap_host *) data;
	int dma_ch;

	/* FIXME: We ignore the possible errors for now. */
	if (host->dma_ch < 0) {
		printk(KERN_ERR "MMC%d: DMA callback while DMA not enabled?\n",
		       host->id);
		return;
	}
	dma_ch = host->dma_ch;
	host->dma_ch = -1;

	omap_free_dma(dma_ch);
	complete(&host->dma_completion);
}

static int
mmc_omap_start_dma_transfer(struct mmc_omap_host *host, struct mmc_request *req)
{
	int sync_dev, dma_ch, r;
	const char *dev_name;
	struct mmc_data *data = req->data;
	u16 buf, frame;
	u32 words;

	/* If for some reason the DMA transfer is still active,
	 * we wait for it to complete. This shouldn't normally happen. */
	if (host->dma_ch != -1)
		wait_for_completion(&host->dma_completion);

	init_completion(&host->dma_completion);

	if (!(data->flags & MMC_DATA_WRITE)) {
		if (host->id == 1) {
			sync_dev = OMAP_DMA_MMC_RX;
			dev_name = "MMC1 read";
		} else {
			sync_dev = OMAP_DMA_MMC2_RX;
			dev_name = "MMC2 read";
		}
	} else {
		if (host->id == 1) {
			sync_dev = OMAP_DMA_MMC_TX;
			dev_name = "MMC1 write";
		} else {
			sync_dev = OMAP_DMA_MMC2_TX;
			dev_name = "MMC2 write";
		}
	}
	r = omap_request_dma(sync_dev, dev_name, mmc_omap_dma_cb,
			host, &dma_ch);
	if (r != 0) {
		printk("MMC%d: omap_request_dma() failed with %d\n",
				host->id, r);
		return r;
	}

	/* FIFO is 32x2 bytes; use 32 word frames when possible */
	frame = 1 << (data->blksz_bits - 1);
	words = data->blocks * frame;
	if (frame > 32)
		frame = 32;

	if (!(data->flags & MMC_DATA_WRITE)) {
		buf = 0x800f | ((frame - 1) << 8);
		omap_set_dma_src_params(dma_ch, OMAP_DMA_PORT_TIPB,
				OMAP_DMA_AMODE_CONSTANT,
				virt_to_phys((void __force *)host->base)
						+ OMAP_MMC_REG_DATA);
		omap_set_dma_dest_params(dma_ch, OMAP_DMA_PORT_EMIFF,
				OMAP_DMA_AMODE_POST_INC,
				data->sg->dma_address);
		omap_set_dma_dest_data_pack(dma_ch, 1);
		omap_set_dma_dest_burst_mode(dma_ch, OMAP_DMA_DATA_BURST_4);
	} else {
		buf = 0x0f80 | ((frame - 1) << 0);
		omap_set_dma_dest_params(dma_ch, OMAP_DMA_PORT_TIPB,
				OMAP_DMA_AMODE_CONSTANT,
				virt_to_phys((void __force *)host->base)
						+ OMAP_MMC_REG_DATA);
		omap_set_dma_src_params(dma_ch, OMAP_DMA_PORT_EMIFF,
				OMAP_DMA_AMODE_POST_INC,
				data->sg->dma_address);
		omap_set_dma_src_data_pack(dma_ch, 1);
		omap_set_dma_src_burst_mode(dma_ch, OMAP_DMA_DATA_BURST_4);
	}

	/* blocksize is usually 512 bytes; but not for some SD reads */
	OMAP_MMC_WRITE(host->base, BUF, buf);
	omap_set_dma_transfer_params(dma_ch, OMAP_DMA_DATA_TYPE_S16,
		frame, words / frame, OMAP_DMA_SYNC_FRAME);

	host->dma_ch = dma_ch;
	omap_start_dma(dma_ch);

        return 0;
}

static inline void
set_cmd_timeout(struct mmc_omap_host *host, struct mmc_request *req)
{
	u16 reg;

	reg = OMAP_MMC_READ(host->base, SDIO);
	reg &= ~(1 << 5);
	OMAP_MMC_WRITE(host->base, SDIO, reg);
	/* Set maximum timeout */
	OMAP_MMC_WRITE(host->base, CTO, 0xff);
}

static inline void set_data_timeout(struct mmc_omap_host *host, struct mmc_request *req)
{
	int timeout;
	u16 reg;
	
	/* Convert ns to clock cycles by assuming 20MHz frequency
	 * 1 cycle at 20MHz = 500 ns
	 */
	timeout = req->data->timeout_clks + req->data->timeout_ns / 500;

	/* Some cards require more time to do at least the first read operation */
	timeout = timeout << 4;

	/* Check if we need to use timeout multiplier register */
	reg = OMAP_MMC_READ(host->base, SDIO);
	if (timeout > 0xffff) {
		reg |= (1 << 5);
		timeout /= 1024;
	} else
		reg &= ~(1 << 5);
	OMAP_MMC_WRITE(host->base, SDIO, reg);
	OMAP_MMC_WRITE(host->base, DTO, timeout);
}

static void mmc_omap_prepare_data(struct mmc_omap_host *host, struct mmc_request *req)
{
	struct mmc_data *data = req->data;
	enum dma_data_direction dma_data_dir;

	host->data = data;
	if (data == NULL) {
		host->datadir = OMAP_MMC_DATADIR_NONE;
		OMAP_MMC_WRITE(host->base, BLEN, 0);
		OMAP_MMC_WRITE(host->base, NBLK, 0);
		OMAP_MMC_WRITE(host->base, BUF, 0);
		set_cmd_timeout(host, req);
		return;
	}

	DBG("MMC%d: Data xfer (%s %s), DTO %d cycles + %d ns, %d blocks of %d bytes\n",
	    host->id, (data->flags & MMC_DATA_STREAM) ? "stream" : "block",
	    (data->flags & MMC_DATA_WRITE) ? "write" : "read",
	    data->timeout_clks, data->timeout_ns, data->blocks,
	    1 << data->blksz_bits);

	OMAP_MMC_WRITE(host->base, NBLK, data->blocks - 1);
	OMAP_MMC_WRITE(host->base, BLEN, (1 << data->blksz_bits) - 1);
	set_data_timeout(host, req);

	if (data->flags & MMC_DATA_WRITE) {
		host->datadir = OMAP_MMC_DATADIR_WRITE;
		dma_data_dir = DMA_TO_DEVICE;
	} else {
		host->datadir = OMAP_MMC_DATADIR_READ;
		dma_data_dir = DMA_FROM_DEVICE;
	}
  
  	host->dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
				   dma_data_dir);

	/* No SG-DMA */
	if (unlikely(host->dma_len > 1))
		BUG();

	if (host->use_dma) {
		if (mmc_omap_start_dma_transfer(host, req) == 0) {
			host->buffer = NULL;
			host->bytesleft = 0;
		}
	} else {
		/* Revert to CPU copy */
		OMAP_MMC_WRITE(host->base, BUF, 0x1f1f);
		host->buffer = page_address(data->sg->page) + data->sg->offset;
		host->bytesleft = data->blocks * (1 << data->blksz_bits);
		host->dma_ch = -1;
	}
}

static inline int is_broken_card(struct mmc_card *card)
{
	int i;
	struct mmc_cid *c = &card->cid;
	static const struct broken_card_cid {
		unsigned int manfid;
		char prod_name[8];
		unsigned char hwrev;
		unsigned char fwrev;
	} broken_cards[] = {
		{ 0x00150000, "\x30\x30\x30\x30\x30\x30\x15\x00", 0x06, 0x03 },
	};

	for (i = 0; i < sizeof(broken_cards)/sizeof(broken_cards[0]); i++) {
		const struct broken_card_cid *b = broken_cards + i;

		if (b->manfid != c->manfid)
			continue;
		if (memcmp(b->prod_name, c->prod_name, sizeof(b->prod_name)) != 0)
			continue;
		if (b->hwrev != c->hwrev || b->fwrev != c->fwrev)
			continue;
		return 1;
	}
	return 0;
}

static void mmc_omap_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct mmc_omap_host *host = mmc_priv(mmc);

	WARN_ON(host->mrq != NULL);

	host->mrq = req;

	/* Some cards (vendor left unnamed to protect the guilty) seem to
	 * require this delay after power-up. Otherwise we'll get mysterious
	 * data timeouts. */
	if (req->cmd->opcode == MMC_SEND_CSD) {
		struct mmc_card *card;
		int broken_present = 0;

		list_for_each_entry(card, &mmc->cards, node) {
			if (is_broken_card(card)) {
				broken_present = 1;
				break;
			}
		}
		if (broken_present) {
			static int complained = 0;

			if (!complained) {
				printk(KERN_WARNING "MMC%d: Broken card workaround enabled\n",
				       host->id);
				complained = 1;
			}
			if (in_interrupt()) {
				/* This is nasty */
				 printk(KERN_ERR "Sleeping in IRQ handler, FIXME please!\n");
				 dump_stack();
				 mdelay(100);
			} else {
				set_current_state(TASK_UNINTERRUPTIBLE);
				schedule_timeout(100 * HZ / 1000);
			}
		}
	}

	mmc_omap_prepare_data(host, req);
	mmc_omap_start_command(host, req->cmd);
}

static void innovator_fpga_socket_power(int on)
{
#if defined(CONFIG_MACH_OMAP_INNOVATOR) && defined(CONFIG_ARCH_OMAP1510)

	if (on) {
		fpga_write(fpga_read(OMAP1510_FPGA_POWER) | (1 << 3),
		     OMAP1510_FPGA_POWER);
	} else {
		fpga_write(fpga_read(OMAP1510_FPGA_POWER) & ~(1 << 3),
		     OMAP1510_FPGA_POWER);
	}
#endif
}

/*
 * Turn the socket power on/off. Innovator uses FPGA, most boards
 * probably use GPIO.
 */
static void mmc_omap_power(struct mmc_omap_host *host, int on)
{
	if (on) {
		if (machine_is_omap_innovator())
			innovator_fpga_socket_power(1);
		else if (machine_is_omap_h2())
			tps65010_set_gpio_out_value(GPIO3, HIGH);
		else if (machine_is_omap_h3())
			/* GPIO 4 of TPS65010 sends SD_EN signal */
			tps65010_set_gpio_out_value(GPIO4, HIGH);
		else
			if (host->power_pin >= 0)
				omap_set_gpio_dataout(host->power_pin, 1);
	} else {
		if (machine_is_omap_innovator())
			innovator_fpga_socket_power(0);
		else if (machine_is_omap_h2())
			tps65010_set_gpio_out_value(GPIO3, LOW);
		else if (machine_is_omap_h3())
			tps65010_set_gpio_out_value(GPIO4, LOW);
		else
			if (host->power_pin >= 0)
				omap_set_gpio_dataout(host->power_pin, 0);
	}
}

static void mmc_omap_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct mmc_omap_host *host = mmc_priv(mmc);
	int dsor;
	int realclock, i;

	DBG("MMC%d: set_ios: clock %dHz busmode %d powermode %d Vdd %d.%02d\n",
	    host->id, ios->clock, ios->bus_mode, ios->power_mode,
	    ios->vdd / 100, ios->vdd % 100);

	if (ios->power_mode == MMC_POWER_UP && ios->clock < 400000) {
		/* Fix for broken stack */
		realclock = 400000;
	} else {
		realclock = ios->clock;
	}

	if (ios->clock == 0) {
		dsor = 0;
	} else {
		dsor = 48000000 / realclock;
		if (dsor < 1)
			dsor = 1;

		if (48000000 / dsor > realclock)
			dsor++;

		if (dsor > 250)
			dsor = 250;
	}

	/* REVISIT:  if (ios->bus_width == MMC_BUS_WIDTH_4) dsor |= 1 << 15; */

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		mmc_omap_power(host, 0);
		break;
	case MMC_POWER_UP:
	case MMC_POWER_ON:
		mmc_omap_power(host, 1);
		dsor |= 1<<11;
		break;
	}

	host->bus_mode = ios->bus_mode;
	clk_use(host->clk);
	/* On insanely high arm_per frequencies something sometimes
	 * goes somehow out of sync, and the POW bit is not being set,
	 * which results in the while loop below getting stuck.
	 * Writing to the CON register twice seems to do the trick. */
	for (i = 0; i < 2; i++)
		OMAP_MMC_WRITE(host->base, CON, dsor);
	if (ios->power_mode == MMC_POWER_UP) {
		/* Send clock cycles, poll completion */
		OMAP_MMC_WRITE(host->base, IE, 0);
		OMAP_MMC_WRITE(host->base, STAT, 0xffff);
		OMAP_MMC_WRITE(host->base, CMD, 1<<7);
		while (0 == (OMAP_MMC_READ(host->base, STAT) & 1));
		OMAP_MMC_WRITE(host->base, STAT, 1);
	}
	clk_unuse(host->clk);
}

static struct mmc_host_ops mmc_omap_ops = {
	.request	= mmc_omap_request,
	.set_ios	= mmc_omap_set_ios,
};

static int __init mmc_omap_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_mmc_conf *minfo = dev->platform_data;
	struct mmc_host *mmc;
	struct mmc_omap_host *host = NULL;
	int ret = 0;

	if (pdev->resource[0].flags != IORESOURCE_MEM
	    || pdev->resource[1].flags != IORESOURCE_IRQ) {
		printk(KERN_ERR "mmc_omap_probe: invalid resource type\n");
		return -ENODEV;
	}

	if (!request_mem_region(pdev->resource[0].start,
				pdev->resource[0].end - pdev->resource[0].start + 1, 
				pdev->name)) {
		dev_dbg(&pdev->dev, "request_mem_region failed\n");
		return -EBUSY;
	}

	mmc = mmc_alloc_host(sizeof(struct mmc_omap_host), dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;

	host->id = pdev->id;

	host->clk = clk_get(dev, (host->id == 1) ? "mmc1_ck" : "mmc2_ck");
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		goto out;
	}

	/* REVISIT:  SD-only support, when core merged
	 *  - if (minfo->wire4) mmc->caps |= MMC_CAP_4_BIT_DATA;
	 *  - mmc_omap_ops.get_ro uses wp_pin to sense slider
	 * Also, use minfo->cover to decide how to manage
	 * the card detect sensing.
	 */
	host->power_pin = minfo->power_pin;
	host->switch_pin = minfo->switch_pin;
	host->wp_pin = minfo->wp_pin;

	host->use_dma = 1;
	host->dma_ch = -1;

	host->irq = pdev->resource[1].start;
	host->base = (void __iomem *)pdev->resource[0].start;

	mmc->ops = &mmc_omap_ops;
	mmc->f_min = 400000;
	mmc->f_max = 24000000;
	mmc->ocr_avail = MMC_VDD_33_34;

	/* No SG-DMA */
	mmc->max_phys_segs = 1;
	mmc->max_seg_size = PAGE_SIZE;

	if (host->power_pin >= 0) {
		if ((ret = omap_request_gpio(host->power_pin)) != 0) {
			printk(KERN_ERR "MMC%d: Unable to get GPIO pin for MMC power\n",
			       host->id);
			goto out;
		}
		omap_set_gpio_direction(host->power_pin, 0);
	}

	ret = request_irq(host->irq, mmc_omap_irq, 0, DRIVER_NAME, host);
	if (ret)
		goto out;

	host->dev = dev;
	dev_set_drvdata(dev, host);

	mmc_add_host(mmc);

	if (host->switch_pin >= 0) {
		INIT_WORK(&host->switch_work, mmc_omap_switch_handler, host);
		init_timer(&host->switch_timer);
		host->switch_timer.function = mmc_omap_switch_timer;
		host->switch_timer.data = (unsigned long) host;
		if (omap_request_gpio(host->switch_pin) != 0) {
			printk(KERN_WARNING "MMC%d: Unable to get GPIO pin for MMC cover switch\n",
			       host->id);
			host->switch_pin = -1;
			goto no_switch;
		}

		omap_set_gpio_direction(host->switch_pin, 1);
		set_irq_type(OMAP_GPIO_IRQ(host->switch_pin), IRQT_RISING);
		ret = request_irq(OMAP_GPIO_IRQ(host->switch_pin),
				  mmc_omap_switch_irq, 0, DRIVER_NAME, host);
		if (ret) {
			printk(KERN_WARNING "MMC%d: Unable to get IRQ for MMC cover switch\n",
			       host->id);
			omap_free_gpio(host->switch_pin);
			host->switch_pin = -1;
			goto no_switch;
		}
		ret = device_create_file(dev, &dev_attr_cover_switch);
		if (ret == 0) {
			ret = device_create_file(dev, &dev_attr_enable_poll);
			if (ret != 0)
				device_remove_file(dev, &dev_attr_cover_switch);
		}
		if (ret) {
			printk(KERN_WARNING "MMC%d: Unable to create sysfs attributes\n", 
			       host->id);
			free_irq(OMAP_GPIO_IRQ(host->switch_pin), host);
			omap_free_gpio(host->switch_pin);
			host->switch_pin = -1;
			goto no_switch;
		}
		if (mmc_omap_enable_poll && mmc_omap_cover_is_open(host))
			schedule_work(&host->switch_work);
	}
no_switch:
	return 0;
out:
	/* FIXME: Free other resources too. */
	if (host) {
		if (host->clk && !IS_ERR(host->clk))
			clk_put(host->clk);
		mmc_free_host(host->mmc);
	}
	return ret;
}

static int __exit mmc_omap_remove(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mmc_omap_host *host = dev_get_drvdata(dev);

	dev_set_drvdata(dev, NULL);

	if (host) {
		mmc_remove_host(host->mmc);
		free_irq(host->irq, host);
		mmc_omap_power(host, 0);

		if (host->power_pin >= 0)
			omap_free_gpio(host->power_pin);
		if (host->switch_pin >= 0) {
			device_remove_file(dev, &dev_attr_enable_poll);
			device_remove_file(dev, &dev_attr_cover_switch);
			free_irq(OMAP_GPIO_IRQ(host->switch_pin), host);
			omap_free_gpio(host->switch_pin);
			host->switch_pin = -1;
			del_timer_sync(&host->switch_timer);
			flush_scheduled_work();
		}
		if (host->clk && !IS_ERR(host->clk))
			clk_put(host->clk);
		mmc_free_host(host->mmc);
	}

	release_mem_region(pdev->resource[0].start, 
			   pdev->resource[0].end - pdev->resource[0].start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int mmc_omap_suspend(struct device *dev, pm_message_t mesg, u32 level)
{
	int ret = 0;
	struct mmc_omap_host *host = dev_get_drvdata(dev);

	if (host && host->suspended)
		return 0;

	if (!irqs_disabled())
		return -EAGAIN;

	if (host) {
		ret = mmc_suspend_host(host->mmc, mesg);
		if (ret == 0)
			host->suspended = 1;
	}
	return ret;
}

static int mmc_omap_resume(struct device *dev, u32 level)
{
	int ret = 0;
	struct mmc_omap_host *host = dev_get_drvdata(dev);

	if (host && !host->suspended)
		return 0;

	if (host) {
		ret = mmc_resume_host(host->mmc);
		if (ret == 0)
			host->suspended = 0;
	}

	return ret;
}
#else
#define mmc_omap_suspend	NULL
#define mmc_omap_resume		NULL
#endif

static struct device_driver mmc_omap_driver = {
	.name		= "mmci-omap",
	.bus		= &platform_bus_type,
	.probe		= mmc_omap_probe,
	.remove		= __exit_p(mmc_omap_remove),
	.suspend	= mmc_omap_suspend,
	.resume		= mmc_omap_resume,
};

static int __init mmc_omap_init(void)
{
	return driver_register(&mmc_omap_driver);
}

static void __exit mmc_omap_exit(void)
{
	driver_unregister(&mmc_omap_driver);
}

module_init(mmc_omap_init);
module_exit(mmc_omap_exit);

MODULE_DESCRIPTION("OMAP Multimedia Card driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juha Yrjölä");
