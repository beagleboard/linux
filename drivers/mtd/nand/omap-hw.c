/*
 *  drivers/mtd/nand/omap-hw.c
 *
 *  This is the MTD driver for OMAP1710 internal HW NAND controller.
 *
 *  Copyright (C) 2004-2006 Nokia Corporation
 *
 *  Author: Jarkko Lavinen <jarkko.lavinen@nokia.com> and
 *          Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; see the file COPYING. If not, write to the Free Software
 * Foundation, 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

#include <asm/io.h>

#include <mach/board.h>
#include <mach/dma.h>

#define NAND_BASE		0xfffbcc00
#define NND_REVISION		0x00
#define NND_ACCESS		0x04
#define NND_ADDR_SRC		0x08
#define NND_CTRL		0x10
#define NND_MASK		0x14
#define NND_STATUS		0x18
#define NND_READY		0x1c
#define NND_COMMAND		0x20
#define NND_COMMAND_SEC		0x24
#define NND_ECC_SELECT		0x28
#define NND_ECC_START		0x2c
#define NND_ECC_9		0x4c
#define NND_RESET		0x50
#define NND_FIFO		0x54
#define NND_FIFOCTRL		0x58
#define NND_PSC_CLK		0x5c
#define NND_SYSTEST		0x60
#define NND_SYSCFG		0x64
#define NND_SYSSTATUS		0x68
#define NND_FIFOTEST1		0x6c
#define NND_FIFOTEST2		0x70
#define NND_FIFOTEST3		0x74
#define NND_FIFOTEST4		0x78
#define NND_PSC1_CLK		0x8c
#define NND_PSC2_CLK		0x90


#define NND_CMD_READ1_LOWER	0x00
#define NND_CMD_WRITE1_LOWER	0x00
#define NND_CMD_READ1_UPPER	0x01
#define NND_CMD_WRITE1_UPPER	0x01
#define NND_CMD_PROGRAM_END	0x10
#define NND_CMD_READ2_SPARE	0x50
#define NND_CMD_WRITE2_SPARE	0x50
#define NND_CMD_ERASE		0x60
#define NND_CMD_STATUS		0x70
#define NND_CMD_PROGRAM		0x80
#define NND_CMD_READ_ID		0x90
#define NND_CMD_ERASE_END	0xD0
#define NND_CMD_RESET		0xFF


#define NAND_Ecc_P1e		(1 << 0)
#define NAND_Ecc_P2e		(1 << 1)
#define NAND_Ecc_P4e		(1 << 2)
#define NAND_Ecc_P8e		(1 << 3)
#define NAND_Ecc_P16e		(1 << 4)
#define NAND_Ecc_P32e		(1 << 5)
#define NAND_Ecc_P64e		(1 << 6)
#define NAND_Ecc_P128e		(1 << 7)
#define NAND_Ecc_P256e		(1 << 8)
#define NAND_Ecc_P512e		(1 << 9)
#define NAND_Ecc_P1024e		(1 << 10)
#define NAND_Ecc_P2048e		(1 << 11)

#define NAND_Ecc_P1o		(1 << 16)
#define NAND_Ecc_P2o		(1 << 17)
#define NAND_Ecc_P4o		(1 << 18)
#define NAND_Ecc_P8o		(1 << 19)
#define NAND_Ecc_P16o		(1 << 20)
#define NAND_Ecc_P32o		(1 << 21)
#define NAND_Ecc_P64o		(1 << 22)
#define NAND_Ecc_P128o		(1 << 23)
#define NAND_Ecc_P256o		(1 << 24)
#define NAND_Ecc_P512o		(1 << 25)
#define NAND_Ecc_P1024o		(1 << 26)
#define NAND_Ecc_P2048o		(1 << 27)

#define TF(value)	(value ? 1 : 0)

#define P2048e(a)	(TF(a & NAND_Ecc_P2048e)	<< 0 )
#define P2048o(a)	(TF(a & NAND_Ecc_P2048o)	<< 1 )
#define P1e(a)		(TF(a & NAND_Ecc_P1e)		<< 2 )
#define P1o(a)		(TF(a & NAND_Ecc_P1o)		<< 3 )
#define P2e(a)		(TF(a & NAND_Ecc_P2e)		<< 4 )
#define P2o(a)		(TF(a & NAND_Ecc_P2o)		<< 5 )
#define P4e(a)		(TF(a & NAND_Ecc_P4e)		<< 6 )
#define P4o(a)		(TF(a & NAND_Ecc_P4o)		<< 7 )

#define P8e(a)		(TF(a & NAND_Ecc_P8e)		<< 0 )
#define P8o(a)		(TF(a & NAND_Ecc_P8o)		<< 1 )
#define P16e(a)		(TF(a & NAND_Ecc_P16e)		<< 2 )
#define P16o(a)		(TF(a & NAND_Ecc_P16o)		<< 3 )
#define P32e(a)		(TF(a & NAND_Ecc_P32e)		<< 4 )
#define P32o(a)		(TF(a & NAND_Ecc_P32o)		<< 5 )
#define P64e(a)		(TF(a & NAND_Ecc_P64e)		<< 6 )
#define P64o(a)		(TF(a & NAND_Ecc_P64o)		<< 7 )

#define P128e(a)	(TF(a & NAND_Ecc_P128e)		<< 0 )
#define P128o(a)	(TF(a & NAND_Ecc_P128o)		<< 1 )
#define P256e(a)	(TF(a & NAND_Ecc_P256e)		<< 2 )
#define P256o(a)	(TF(a & NAND_Ecc_P256o)		<< 3 )
#define P512e(a)	(TF(a & NAND_Ecc_P512e)		<< 4 )
#define P512o(a)	(TF(a & NAND_Ecc_P512o)		<< 5 )
#define P1024e(a)	(TF(a & NAND_Ecc_P1024e)	<< 6 )
#define P1024o(a)	(TF(a & NAND_Ecc_P1024o)	<< 7 )

#define P8e_s(a)	(TF(a & NAND_Ecc_P8e)		<< 0 )
#define P8o_s(a)	(TF(a & NAND_Ecc_P8o)		<< 1 )
#define P16e_s(a)	(TF(a & NAND_Ecc_P16e)		<< 2 )
#define P16o_s(a)	(TF(a & NAND_Ecc_P16o)		<< 3 )
#define P1e_s(a)	(TF(a & NAND_Ecc_P1e)		<< 4 )
#define P1o_s(a)	(TF(a & NAND_Ecc_P1o)		<< 5 )
#define P2e_s(a)	(TF(a & NAND_Ecc_P2e)		<< 6 )
#define P2o_s(a)	(TF(a & NAND_Ecc_P2o)		<< 7 )

#define P4e_s(a)	(TF(a & NAND_Ecc_P4e)		<< 0 )
#define P4o_s(a)	(TF(a & NAND_Ecc_P4o)		<< 1 )

extern struct nand_oobinfo jffs2_oobinfo;

/*
 * MTD structure for OMAP board
 */
static struct mtd_info *omap_mtd;
static struct clk *omap_nand_clk;
static int omap_nand_dma_ch;
static struct completion omap_nand_dma_comp;
static unsigned long omap_nand_base = OMAP1_IO_ADDRESS(NAND_BASE);

static inline u32 nand_read_reg(int idx)
{
	return __raw_readl(omap_nand_base + idx);
}

static inline void nand_write_reg(int idx, u32 val)
{
	__raw_writel(val, omap_nand_base + idx);
}

static inline u8 nand_read_reg8(int idx)
{
	return __raw_readb(omap_nand_base + idx);
}

static inline void nand_write_reg8(int idx, u8 val)
{
	__raw_writeb(val, omap_nand_base + idx);
}

static void omap_nand_select_chip(struct mtd_info *mtd, int chip)
{
	u32 l;

	switch(chip) {
	case -1:
		l = nand_read_reg(NND_CTRL);
		l |= (1 << 8) | (1 << 10) | (1 << 12) | (1 << 14);
		nand_write_reg(NND_CTRL, l);
		break;
	case 0:
		/* Also CS1, CS2, CS4 would be available */
		l = nand_read_reg(NND_CTRL);
		l &= ~(1 << 8);
		nand_write_reg(NND_CTRL, l);
		break;
	default:
		BUG();
	}
}

static void nand_dma_cb(int lch, u16 ch_status, void *data)
{
	complete((struct completion *) data);
}

static void omap_nand_dma_transfer(struct mtd_info *mtd, void *addr,
                                         unsigned int u32_count, int is_write)
{
	const int block_size = 16;
	unsigned int block_count, len;
	int dma_ch;
	unsigned long fifo_reg, timeout, jiffies_before, jiffies_spent;
	static unsigned long max_jiffies = 0;

	dma_ch = omap_nand_dma_ch;
	block_count = u32_count * 4 / block_size;
	nand_write_reg(NND_STATUS, 0x0f);
	nand_write_reg(NND_FIFOCTRL, (block_size << 24) | block_count);
	fifo_reg = NAND_BASE + NND_FIFO;
	if (is_write) {
		omap_set_dma_dest_params(dma_ch, OMAP_DMA_PORT_TIPB,
					 OMAP_DMA_AMODE_CONSTANT, fifo_reg,
					 0, 0);
		omap_set_dma_src_params(dma_ch, OMAP_DMA_PORT_EMIFF,
					OMAP_DMA_AMODE_POST_INC,
					virt_to_phys(addr),
					0, 0);
//		omap_set_dma_src_burst_mode(dma_ch, OMAP_DMA_DATA_BURST_4);
		/* Set POSTWRITE bit */
		nand_write_reg(NND_CTRL, nand_read_reg(NND_CTRL) | (1 << 16));
	} else {
		omap_set_dma_src_params(dma_ch, OMAP_DMA_PORT_TIPB,
					OMAP_DMA_AMODE_CONSTANT, fifo_reg,
					0, 0);
		omap_set_dma_dest_params(dma_ch, OMAP_DMA_PORT_EMIFF,
					 OMAP_DMA_AMODE_POST_INC,
					 virt_to_phys(addr),
					 0, 0);
//		omap_set_dma_dest_burst_mode(dma_ch, OMAP_DMA_DATA_BURST_8);
		/* Set PREFETCH bit */
		nand_write_reg(NND_CTRL, nand_read_reg(NND_CTRL) | (1 << 17));
	}
	omap_set_dma_transfer_params(dma_ch, OMAP_DMA_DATA_TYPE_S32, block_size / 4,
				     block_count, OMAP_DMA_SYNC_FRAME,
				     0, 0);
	init_completion(&omap_nand_dma_comp);

	len = u32_count << 2;
	dma_cache_maint(addr, len, DMA_TO_DEVICE);
	omap_start_dma(dma_ch);
	jiffies_before = jiffies;
	timeout = wait_for_completion_timeout(&omap_nand_dma_comp,
					      msecs_to_jiffies(1000));
	jiffies_spent = (unsigned long)((long)jiffies - (long)jiffies_before);
	if (jiffies_spent > max_jiffies)
		max_jiffies = jiffies_spent;

	if (timeout == 0) {
		printk(KERN_WARNING "omap-hw-nand: DMA timeout after %u ms, max. seen latency %u ms\n",
		       jiffies_to_msecs(jiffies_spent),
		       jiffies_to_msecs(max_jiffies));
	}
	if (!is_write)
		dma_cache_maint(addr, len, DMA_FROM_DEVICE);

	nand_write_reg(NND_CTRL, nand_read_reg(NND_CTRL) & ~((1 << 16) | (1 << 17)));
}

static void fifo_read(u32 *out, unsigned int len)
{
	const int block_size = 16;
	unsigned long status_reg, fifo_reg;
	int c;

	status_reg = omap_nand_base + NND_STATUS;
	fifo_reg = omap_nand_base + NND_FIFO;
	len = len * 4 / block_size;
	nand_write_reg(NND_FIFOCTRL, (block_size << 24) | len);
	nand_write_reg(NND_STATUS, 0x0f);
	nand_write_reg(NND_CTRL, nand_read_reg(NND_CTRL) | (1 << 17));
	c = block_size / 4;
	while (len--) {
		int i;

		while ((__raw_readl(status_reg) & (1 << 2)) == 0);
		__raw_writel(0x0f, status_reg);
		for (i = 0; i < c; i++) {
			u32 l = __raw_readl(fifo_reg);
			*out++ = l;
		}
	}
	nand_write_reg(NND_CTRL, nand_read_reg(NND_CTRL) & ~(1 << 17));
	nand_write_reg(NND_STATUS, 0x0f);
}

static void omap_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	unsigned long access_reg;

	if (likely(((unsigned long) buf & 3) == 0 && (len & 3) == 0)) {
		int u32_count = len >> 2;
		u32 *dest = (u32 *) buf;
		/* If the transfer is big enough and the length divisible by
		 * 16, we try to use DMA transfer, or FIFO copy in case of
		 * DMA failure (e.g. all channels busy) */
		if (u32_count > 64 && (u32_count & 3) == 0) {
			if (omap_nand_dma_ch >= 0) {
				omap_nand_dma_transfer(mtd, buf, u32_count, 0);
				return;
			}
			/* In case of an error, fallback to FIFO copy */
			fifo_read((u32 *) buf, u32_count);
			return;
		}
		access_reg = omap_nand_base + NND_ACCESS;
		/* Small buffers we just read directly */
		while (u32_count--)
			*dest++ = __raw_readl(access_reg);
	} else {
		/* If we're not word-aligned, we use byte copy */
		access_reg = omap_nand_base + NND_ACCESS;
		while (len--)
			*buf++ = __raw_readb(access_reg);
	}
}

static void omap_nand_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	if (likely(((unsigned long) buf & 3) == 0 && (len & 3) == 0)) {
		const u32 *src = (const u32 *) buf;

		len >>= 2;
#if 0
		/* If the transfer is big enough and length divisible by 16,
		 * we try to use DMA transfer. */
		if (len > 256 / 4 && (len & 3) == 0) {
			if (omap_nand_dma_transfer(mtd, (void *) buf, len, 1) == 0)
				return;
			/* In case of an error, fallback to CPU copy */
		}
#endif
		while (len--)
			nand_write_reg(NND_ACCESS, *src++);
	} else {
		while (len--)
			nand_write_reg8(NND_ACCESS, *buf++);
	}
}

static int omap_nand_verify_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	if (likely(((unsigned long) buf & 3) == 0 && (len & 3) == 0)) {
		const u32 *dest = (const u32 *) buf;
		len >>= 2;
		while (len--)
			if (*dest++ != nand_read_reg(NND_ACCESS))
				return -EFAULT;
	} else {
		while (len--)
			if (*buf++ != nand_read_reg8(NND_ACCESS))
				return -EFAULT;
	}
	return 0;
}

static u_char omap_nand_read_byte(struct mtd_info *mtd)
{
	return nand_read_reg8(NND_ACCESS);
}

static int omap_nand_dev_ready(struct mtd_info *mtd)
{
	u32 l;

	l = nand_read_reg(NND_READY);
	return l & 0x01;
}

static int nand_write_command(u8 cmd, u32 addr, int addr_valid)
{
	if (addr_valid) {
		nand_write_reg(NND_ADDR_SRC, addr);
		nand_write_reg8(NND_COMMAND, cmd);
	} else {
		nand_write_reg(NND_ADDR_SRC, 0);
		nand_write_reg8(NND_COMMAND_SEC, cmd);
	}
	while (!omap_nand_dev_ready(NULL));
	return 0;
}

/*
 * Send command to NAND device
 */
static void omap_nand_command(struct mtd_info *mtd, unsigned command, int column, int page_addr)
{
	struct nand_chip *this = mtd->priv;

	/*
	 * Write out the command to the device.
	 */
	if (command == NAND_CMD_SEQIN) {
		int readcmd;

		if (column >= mtd->writesize) {
			/* OOB area */
			column -= mtd->writesize;
			readcmd = NAND_CMD_READOOB;
		} else if (column < 256) {
			/* First 256 bytes --> READ0 */
			readcmd = NAND_CMD_READ0;
		} else {
			column -= 256;
			readcmd = NAND_CMD_READ1;
		}
		nand_write_command(readcmd, 0, 0);
	}
	switch (command) {
	case NAND_CMD_RESET:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_STATUS:
	case NAND_CMD_ERASE2:
		nand_write_command(command, 0, 0);
		break;
	case NAND_CMD_ERASE1:
		nand_write_command(command, ((page_addr & 0xFFFFFF00) << 1) | (page_addr & 0XFF), 1);
		break;
	default:
		nand_write_command(command, (page_addr << this->page_shift) | column, 1);
	}
}

static void omap_nand_command_lp(struct mtd_info *mtd, unsigned command, int column, int page_addr)
{
	struct nand_chip *this = mtd->priv;

	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}
	switch (command) {
	case NAND_CMD_RESET:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_STATUS:
	case NAND_CMD_ERASE2:		
		nand_write_command(command, 0, 0);
		break;
	case NAND_CMD_ERASE1:
		nand_write_command(command, page_addr << this->page_shift >> 11, 1);
		break;
	default:
		nand_write_command(command, (page_addr << 16) | column, 1);
	}
	if (command == NAND_CMD_READ0)
		nand_write_command(NAND_CMD_READSTART, 0, 0);
}

/*
 * Generate non-inverted ECC bytes.
 *
 * Using noninverted ECC can be considered ugly since writing a blank
 * page ie. padding will clear the ECC bytes. This is no problem as long
 * nobody is trying to write data on the seemingly unused page.
 *
 * Reading an erased page will produce an ECC mismatch between
 * generated and read ECC bytes that has to be dealt with separately.
 */
static int omap_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	u32 l;
	int reg;
	int n;
	struct nand_chip *this = mtd->priv;

	/* Ex NAND_ECC_HW12_2048 */
	if ((this->ecc.mode == NAND_ECC_HW) && (this->ecc.size  == 2048))
		n = 4;
	else
		n = 1;
	reg = NND_ECC_START;
	while (n--) {
		l = nand_read_reg(reg);
		*ecc_code++ = l;          // P128e, ..., P1e
		*ecc_code++ = l >> 16;    // P128o, ..., P1o
		// P2048o, P1024o, P512o, P256o, P2048e, P1024e, P512e, P256e
		*ecc_code++ = ((l >> 8) & 0x0f) | ((l >> 20) & 0xf0);
		reg += 4;
	}
	return 0;
}

/*
 * This function will generate true ECC value, which can be used
 * when correcting data read from NAND flash memory core
 */
static void gen_true_ecc(u8 *ecc_buf)
{
	u32 tmp = ecc_buf[0] | (ecc_buf[1] << 16) | ((ecc_buf[2] & 0xF0) << 20) | ((ecc_buf[2] & 0x0F) << 8);

	ecc_buf[0] = ~(P64o(tmp) | P64e(tmp) | P32o(tmp) | P32e(tmp) | P16o(tmp) | P16e(tmp) | P8o(tmp) | P8e(tmp) );
	ecc_buf[1] = ~(P1024o(tmp) | P1024e(tmp) | P512o(tmp) | P512e(tmp) | P256o(tmp) | P256e(tmp) | P128o(tmp) | P128e(tmp));
	ecc_buf[2] = ~( P4o(tmp) | P4e(tmp) | P2o(tmp) | P2e(tmp) | P1o(tmp) | P1e(tmp) | P2048o(tmp) | P2048e(tmp));
}

/*
 * This function compares two ECC's and indicates if there is an error.
 * If the error can be corrected it will be corrected to the buffer
 */
static int omap_nand_compare_ecc(u8 *ecc_data1,   /* read from NAND memory */
				 u8 *ecc_data2,   /* read from register */
				 u8 *page_data)
{
	uint   i;
	u8     tmp0_bit[8], tmp1_bit[8], tmp2_bit[8];
	u8     comp0_bit[8], comp1_bit[8], comp2_bit[8];
	u8     ecc_bit[24];
	u8     ecc_sum = 0;
	u8     find_bit = 0;
	uint   find_byte = 0;
	int    isEccFF;

	isEccFF = ((*(u32 *)ecc_data1 & 0xFFFFFF) == 0xFFFFFF);

	gen_true_ecc(ecc_data1);
	gen_true_ecc(ecc_data2);

	for (i = 0; i <= 2; i++) {
		*(ecc_data1 + i) = ~(*(ecc_data1 + i));
		*(ecc_data2 + i) = ~(*(ecc_data2 + i));
	}

	for (i = 0; i < 8; i++) {
		tmp0_bit[i]      = *ecc_data1 % 2;
		*ecc_data1       = *ecc_data1 / 2;
	}

	for (i = 0; i < 8; i++) {
		tmp1_bit[i]      = *(ecc_data1 + 1) % 2;
		*(ecc_data1 + 1) = *(ecc_data1 + 1) / 2;
	}

	for (i = 0; i < 8; i++) {
		tmp2_bit[i]      = *(ecc_data1 + 2) % 2;
		*(ecc_data1 + 2) = *(ecc_data1 + 2) / 2;
	}

	for (i = 0; i < 8; i++) {
		comp0_bit[i]     = *ecc_data2 % 2;
		*ecc_data2       = *ecc_data2 / 2;
	}

	for (i = 0; i < 8; i++) {
		comp1_bit[i]     = *(ecc_data2 + 1) % 2;
		*(ecc_data2 + 1) = *(ecc_data2 + 1) / 2;
	}

	for (i = 0; i < 8; i++) {
		comp2_bit[i]     = *(ecc_data2 + 2) % 2;
		*(ecc_data2 + 2) = *(ecc_data2 + 2) / 2;
	}

	for (i = 0; i< 6; i++ )
		ecc_bit[i] = tmp2_bit[i + 2] ^ comp2_bit[i + 2];

	for (i = 0; i < 8; i++)
		ecc_bit[i + 6] = tmp0_bit[i] ^ comp0_bit[i];

	for (i = 0; i < 8; i++)
		ecc_bit[i + 14] = tmp1_bit[i] ^ comp1_bit[i];

	ecc_bit[22] = tmp2_bit[0] ^ comp2_bit[0];
	ecc_bit[23] = tmp2_bit[1] ^ comp2_bit[1];

	for (i = 0; i < 24; i++)
		ecc_sum += ecc_bit[i];

	switch (ecc_sum) {
	case 0:
		/* Not reached because this function is not called if
		   ECC values are equal */
		return 0;

	case 1:
		/* Uncorrectable error */
		DEBUG (MTD_DEBUG_LEVEL0, "ECC UNCORRECTED_ERROR 1\n");
		return -1;

	case 12:
		/* Correctable error */
		find_byte = (ecc_bit[23] << 8) + 
			    (ecc_bit[21] << 7) + 
			    (ecc_bit[19] << 6) +
			    (ecc_bit[17] << 5) +
			    (ecc_bit[15] << 4) +
			    (ecc_bit[13] << 3) +
			    (ecc_bit[11] << 2) +
			    (ecc_bit[9]  << 1) +
			    ecc_bit[7];

		find_bit = (ecc_bit[5] << 2) + (ecc_bit[3] << 1) + ecc_bit[1];

		DEBUG (MTD_DEBUG_LEVEL0, "Correcting single bit ECC error at offset: %d, bit: %d\n", find_byte, find_bit);

		page_data[find_byte] ^= (1 << find_bit);

		return 0;
	default:
		if (isEccFF) {
			if (ecc_data2[0] == 0 && ecc_data2[1] == 0 && ecc_data2[2] == 0)
				return 0;
		} 
		DEBUG (MTD_DEBUG_LEVEL0, "UNCORRECTED_ERROR default\n");
		return -1;
	}
}

static int omap_nand_correct_data(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	struct nand_chip *this;
	int block_count = 0, i, r;

	this = mtd->priv;
	/* Ex NAND_ECC_HW12_2048 */
	if ((this->ecc.mode == NAND_ECC_HW) && (this->ecc.size  == 2048))
		block_count = 4;
	else
		block_count = 1;
	for (i = 0; i < block_count; i++) {
		if (memcmp(read_ecc, calc_ecc, 3) != 0) {
			r = omap_nand_compare_ecc(read_ecc, calc_ecc, dat);
			if (r < 0)
				return r;
		}
		read_ecc += 3;
		calc_ecc += 3;
		dat += 512;
	}
	return 0;
}

static void omap_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	nand_write_reg(NND_RESET, 0x01);
}

#ifdef CONFIG_MTD_CMDLINE_PARTS

extern int mtdpart_setup(char *);

static int __init add_dynamic_parts(struct mtd_info *mtd)
{
	static const char *part_parsers[] = { "cmdlinepart", NULL };
	struct mtd_partition *parts;
	const struct omap_flash_part_str_config *cfg;
	char *part_str = NULL;
	size_t part_str_len;
	int c;

	cfg = omap_get_var_config(OMAP_TAG_FLASH_PART_STR, &part_str_len);
	if (cfg != NULL) {
		part_str = kmalloc(part_str_len + 1, GFP_KERNEL);
		if (part_str == NULL)
			return -ENOMEM;
		memcpy(part_str, cfg->part_table, part_str_len);
		part_str[part_str_len] = '\0';
		mtdpart_setup(part_str);
	}
	c = parse_mtd_partitions(omap_mtd, part_parsers, &parts, 0);
	if (part_str != NULL) {
		mtdpart_setup(NULL);
		kfree(part_str);
	}
	if (c <= 0)
		return -1;

	add_mtd_partitions(mtd, parts, c);

	return 0;
}

#else

static inline int add_dynamic_parts(struct mtd_info *mtd)
{
	return -1;
}

#endif

static inline int calc_psc(int ns, int cycle_ps)
{
	return (ns * 1000 + (cycle_ps - 1)) / cycle_ps;
}

static void set_psc_regs(int psc_ns, int psc1_ns, int psc2_ns)
{
	int psc[3], i;
	unsigned long rate, ps;

	rate = clk_get_rate(omap_nand_clk);
	ps = 1000000000 / (rate / 1000);
	psc[0] = calc_psc(psc_ns, ps);
	psc[1] = calc_psc(psc1_ns, ps);
	psc[2] = calc_psc(psc2_ns, ps);
	for (i = 0; i < 3; i++) {
		if (psc[i] < 2)
			psc[i] = 2;
		else if (psc[i] > 256)
			psc[i] = 256;
	}
	nand_write_reg(NND_PSC_CLK, psc[0] - 1);
	nand_write_reg(NND_PSC1_CLK, psc[1] - 1);
	nand_write_reg(NND_PSC2_CLK, psc[2] - 1);
	printk(KERN_INFO "omap-hw-nand: using PSC values %d, %d, %d\n", psc[0], psc[1], psc[2]);
}

/*
 * Main initialization routine
 */
static int __init omap_nand_init(void)
{
	struct nand_chip *this;
	int err = 0;
	u32 l;

	omap_nand_clk = clk_get(NULL, "armper_ck");
	BUG_ON(omap_nand_clk == NULL);
	clk_enable(omap_nand_clk);

	l = nand_read_reg(NND_REVISION);	
	printk(KERN_INFO "omap-hw-nand: OMAP NAND Controller rev. %d.%d\n", l>>4, l & 0xf);

	/* Reset the NAND Controller */
	nand_write_reg(NND_SYSCFG, 0x02);
	while ((nand_read_reg(NND_SYSSTATUS) & 0x01) == 0);

	/* No Prefetch, no postwrite, write prot & enable pairs disabled,
	   addres counter set to send 4 byte addresses to flash,
	   A8 is set not to be sent to flash (erase addre needs formatting),
	   choose little endian, enable 512 byte ECC logic,	   
	 */
	nand_write_reg(NND_CTRL, 0xFF01);

	/* Allocate memory for MTD device structure and private data */
	omap_mtd = kmalloc(sizeof(struct mtd_info) + sizeof(struct nand_chip), GFP_KERNEL);
	if (!omap_mtd) {
		printk(KERN_WARNING "omap-hw-nand: Unable to allocate OMAP NAND MTD device structure.\n");
		err = -ENOMEM;
		goto free_clock;
	}
#if 1
	err = omap_request_dma(OMAP_DMA_NAND, "NAND", nand_dma_cb,
			       &omap_nand_dma_comp, &omap_nand_dma_ch);
	if (err < 0) {
		printk(KERN_WARNING "omap-hw-nand: Unable to reserve DMA channel\n");
		omap_nand_dma_ch = -1;
	}
#else
	omap_nand_dma_ch = -1;
#endif
	/* Get pointer to private data */
	this = (struct nand_chip *) (&omap_mtd[1]);

	/* Initialize structures */
	memset((char *) omap_mtd, 0, sizeof(struct mtd_info));
	memset((char *) this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	omap_mtd->priv = this;
	omap_mtd->name = "omap-nand";

	this->options = NAND_SKIP_BBTSCAN;

	/* Used from chip select and nand_command() */
	this->read_byte = omap_nand_read_byte;

	this->select_chip   = omap_nand_select_chip;
	this->dev_ready     = omap_nand_dev_ready;
	this->chip_delay    = 0;
	this->ecc.mode      = NAND_ECC_HW;
	this->ecc.bytes     = 3;
	this->ecc.size      = 512;
	this->cmdfunc       = omap_nand_command;
	this->write_buf     = omap_nand_write_buf;
	this->read_buf      = omap_nand_read_buf;
	this->verify_buf    = omap_nand_verify_buf;
	this->ecc.calculate = omap_nand_calculate_ecc;
	this->ecc.correct   = omap_nand_correct_data;
	this->ecc.hwctl     = omap_nand_enable_hwecc;

	nand_write_reg(NND_SYSCFG, 0x1); /* Enable auto idle */
	nand_write_reg(NND_PSC_CLK, 10);
	/* Scan to find existance of the device */
	if (nand_scan(omap_mtd, 1)) {
		err = -ENXIO;
		goto out_mtd;
	}

	set_psc_regs(25, 15, 35);
	if (this->page_shift == 11) {
		this->cmdfunc = omap_nand_command_lp;
		l = nand_read_reg(NND_CTRL);
		l |= 1 << 4; /* Set the A8 bit in CTRL reg */
		nand_write_reg(NND_CTRL, l);
		this->ecc.mode = NAND_ECC_HW;
		this->ecc.steps = 1;
		this->ecc.size = 2048;
		this->ecc.bytes = 12;
		nand_write_reg(NND_ECC_SELECT, 6);
	}

	/* We have to do bbt scanning ourselves */
	if (this->scan_bbt (omap_mtd)) {
		err = -ENXIO;
		goto out_mtd;
	}

	err = add_dynamic_parts(omap_mtd);
	if (err < 0) {
		printk(KERN_ERR "omap-hw-nand: no partitions defined\n");
		err = -ENODEV;
		nand_release(omap_mtd);
		goto out_mtd;
	}
	/* init completed */
	return 0;
out_mtd:
	if (omap_nand_dma_ch >= 0)
		omap_free_dma(omap_nand_dma_ch);
	kfree(omap_mtd);
free_clock:
	clk_put(omap_nand_clk);
	return err;
}

module_init(omap_nand_init);

/*
 * Clean up routine
 */
static void __exit omap_nand_cleanup (void)
{
	clk_disable(omap_nand_clk);
	clk_put(omap_nand_clk);
	nand_release(omap_mtd);
	kfree(omap_mtd);
}

module_exit(omap_nand_cleanup);

