// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017 exceet electronics GmbH
 *
 * Authors:
 *	Frieder Schrempf <frieder.schrempf@exceet.de>
 *	Boris Brezillon <boris.brezillon@bootlin.com>
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mtd/spinand.h>

#define SPINAND_MFR_WINBOND		0xEF

#define WINBOND_CFG_BUF_READ		BIT(3)

static SPINAND_OP_VARIANTS(read_cache_variants,
		SPINAND_PAGE_READ_FROM_CACHE_QUADIO_OP(0, 2, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_X4_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_DUALIO_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_X2_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_OP(true, 0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_OP(false, 0, 1, NULL, 0));

static SPINAND_OP_VARIANTS(write_cache_variants,
		SPINAND_PROG_LOAD_X4(true, 0, NULL, 0),
		SPINAND_PROG_LOAD(true, 0, NULL, 0));

static SPINAND_OP_VARIANTS(update_cache_variants,
		SPINAND_PROG_LOAD_X4(false, 0, NULL, 0),
		SPINAND_PROG_LOAD(false, 0, NULL, 0));

static int w25m02gv_ooblayout_ecc(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *region)
{
	if (section > 3)
		return -ERANGE;

	region->offset = (16 * section) + 8;
	region->length = 8;

	return 0;
}

static int w25m02gv_ooblayout_free(struct mtd_info *mtd, int section,
				   struct mtd_oob_region *region)
{
	if (section > 3)
		return -ERANGE;

	region->offset = (16 * section) + 2;
	region->length = 6;

	return 0;
}

static const struct mtd_ooblayout_ops w25m02gv_ooblayout = {
	.ecc = w25m02gv_ooblayout_ecc,
	.free = w25m02gv_ooblayout_free,
};

static int w25m02gv_select_target(struct spinand_device *spinand,
				  unsigned int target)
{
	struct spi_mem_op op = SPI_MEM_OP(SPI_MEM_OP_CMD(0xc2, 1),
					  SPI_MEM_OP_NO_ADDR,
					  SPI_MEM_OP_NO_DUMMY,
					  SPI_MEM_OP_DATA_OUT(1,
							spinand->scratchbuf,
							1));

	*spinand->scratchbuf = target;
	return spi_mem_exec_op(spinand->spimem, &op);
}

static const struct spinand_info winbond_spinand_table[] = {
	SPINAND_INFO("W25M02GV",
		     SPINAND_ID(SPINAND_READID_METHOD_OPCODE_DUMMY, 0xab),
		     NAND_MEMORG(1, 2048, 64, 64, 1024, 20, 1, 1, 2),
		     NAND_ECCREQ(1, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
					      &write_cache_variants,
					      &update_cache_variants),
		     0,
		     SPINAND_ECCINFO(&w25m02gv_ooblayout, NULL),
		     SPINAND_SELECT_TARGET(w25m02gv_select_target)),
	SPINAND_INFO("W25N01GV",
		     SPINAND_ID(SPINAND_READID_METHOD_OPCODE_DUMMY, 0xaa),
		     NAND_MEMORG(1, 2048, 64, 64, 1024, 20, 1, 1, 1),
		     NAND_ECCREQ(1, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
					      &write_cache_variants,
					      &update_cache_variants),
		     0,
		     SPINAND_ECCINFO(&w25m02gv_ooblayout, NULL)),
};

static int winbond_spinand_init(struct spinand_device *spinand)
{
	struct nand_device *nand = spinand_to_nand(spinand);
	unsigned int i;

	/*
	 * Make sure all dies are in buffer read mode and not continuous read
	 * mode.
	 */
	for (i = 0; i < nand->memorg.ntargets; i++) {
		spinand_select_target(spinand, i);
		spinand_upd_cfg(spinand, WINBOND_CFG_BUF_READ,
				WINBOND_CFG_BUF_READ);
	}

	return 0;
}

/**
 * winbond_write_vcr_op() - write values onto the volatile configuration
 *			    registers (VCR)
 * @spinand: the spinand device
 * @reg: the address of the particular reg in the VCR to be written on
 * @val: the value to be written on the reg in the VCR
 *
 * Volatile configuration registers are a separate set of configuration
 * registers, i.e. they differ from the status registers SR-1/2/3. A different
 * SPI instruction is required to write to these registers. Any changes
 * to the Volatile Configuration Register get transferred directly to
 * the Internal Configuration Register and instantly reflect on the
 * device operation.
 */
static int winbond_write_vcr_op(struct spinand_device *spinand, u8 reg, u8 val)
{
	int ret;
	struct spi_mem_op op =
		SPI_MEM_OP(SPI_MEM_OP_CMD(0x81, 1),
			   SPI_MEM_OP_ADDR(3, reg, 1),
			   SPI_MEM_OP_NO_DUMMY,
			   SPI_MEM_OP_DATA_OUT(1, spinand->scratchbuf, 1));

	*spinand->scratchbuf = val;

	ret = spinand_write_enable_op(spinand);
	if (ret)
		return ret;

	ret = spi_mem_exec_op(spinand->spimem, &op);
	if (ret)
		return ret;

	/*
	 * Write VCR operation doesn't set the busy bit in SR, so can't perform
	 * a status poll. Minimum time of 50ns is needed to complete the write.
	 * So, give thrice the minimum required delay.
	 */
	ndelay(150);
	return 0;
}

static const struct spinand_manufacturer_ops winbond_spinand_manuf_ops = {
	.init = winbond_spinand_init,
};

const struct spinand_manufacturer winbond_spinand_manufacturer = {
	.id = SPINAND_MFR_WINBOND,
	.name = "Winbond",
	.chips = winbond_spinand_table,
	.nchips = ARRAY_SIZE(winbond_spinand_table),
	.ops = &winbond_spinand_manuf_ops,
};
