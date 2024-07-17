// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017 exceet electronics GmbH
 *
 * Authors:
 *	Frieder Schrempf <frieder.schrempf@exceet.de>
 *	Boris Brezillon <boris.brezillon@bootlin.com>
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mtd/spinand.h>
#include <linux/delay.h>

#define SPINAND_MFR_WINBOND		0xEF

#define WINBOND_CFG_BUF_READ		BIT(3)

#define W25N01JW_STATUS_REG4			0xd0
#define W25N01JW_STATUS_REG4_HS			BIT(2)
#define W25N01JW_STATUS_REG4_HS_MASK		0xFB

#define W35N01JW_VCR_IO_MODE_ADDR		0x00
#define W35N01JW_VCR_IO_MODE_ADDR_DEFAULT	0xFF
#define W35N01JW_VCR_IO_MODE_ADDR_OCTAL_SDR	0xDF

#define W35N01JW_VCR_DUMMY_CLOCK_ADDR		0x01
#define W35N01JW_VCR_DUMMY_CLOCK_ADDR_DEFAULT	0xFF
#define W35N01JW_VCR_DUMMY_CLOCK_ADDR_20	0x14

static SPINAND_OP_VARIANTS(read_cache_variants,
		SPINAND_PAGE_READ_FROM_CACHE_QUADIO_OP(0, 4, NULL, 0),
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

static SPINAND_OP_VARIANTS(octalio_read_cache_variants,
		SPINAND_PAGE_READ_FROM_CACHE_OCTALIO_OP(0, 20, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_OP(true, 0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_OP(false, 0, 1, NULL, 0));

static SPINAND_OP_VARIANTS(x8_write_cache_variants,
		SPINAND_PROG_LOAD_OCTALIO(true, 0, NULL, 0),
		SPINAND_PROG_LOAD(true, 0, NULL, 0));

static SPINAND_OP_VARIANTS(x8_update_cache_variants,
		SPINAND_PROG_LOAD_OCTALIO(false, 0, NULL, 0),
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

static int w25n01jw_ooblayout_ecc(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *region)
{
	if (section > 3)
		return -ERANGE;

	region->offset = (16 * section) + 12;
	region->length = 4;

	return 0;
}

static int w25n01jw_ooblayout_free(struct mtd_info *mtd, int section,
				   struct mtd_oob_region *region)
{
	if (section > 3)
		return -ERANGE;

	region->offset = (16 * section) + 2;
	region->length = 10;

	return 0;
}

static int w35n01jw_ooblayout_ecc(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *region)
{
	if (section > 7)
		return -ERANGE;

	region->offset = (16 * section) + 12;
	region->length = 4;

	return 0;
}

static int w35n01jw_ooblayout_free(struct mtd_info *mtd, int section,
				   struct mtd_oob_region *region)
{
	if (section > 7)
		return -ERANGE;

	region->offset = (16 * section) + 2;
	region->length = 10;

	return 0;
}

static const struct mtd_ooblayout_ops w25m02gv_ooblayout = {
	.ecc = w25m02gv_ooblayout_ecc,
	.free = w25m02gv_ooblayout_free,
};

static const struct mtd_ooblayout_ops w25n01jw_ooblayout = {
	.ecc = w25n01jw_ooblayout_ecc,
	.free = w25n01jw_ooblayout_free,
};

static const struct mtd_ooblayout_ops w35n01jw_ooblayout = {
	.ecc = w35n01jw_ooblayout_ecc,
	.free = w35n01jw_ooblayout_free,
};

static int w35n01jw_write_vcr_op(struct spinand_device *spinand, u8 reg, u8 val)
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

static int w25n01jw_late_init(struct spinand_device *spinand)
{
	u8 *sr4_read;
	u8 sr4_write;
	int ret;

	sr4_read = kmalloc(sizeof(u8), GFP_KERNEL);
	if (!sr4_read)
		return -ENOMEM;

	ret = spinand_read_reg_op(spinand, W25N01JW_STATUS_REG4, sr4_read);
	if (ret)
		return ret;

	*sr4_read &= W25N01JW_STATUS_REG4_HS_MASK;
	*sr4_read |= W25N01JW_STATUS_REG4_HS;

	sr4_write = *sr4_read;

	ret = spinand_write_reg_op(spinand, W25N01JW_STATUS_REG4, sr4_write);
	if (ret)
		return ret;

	return 0;
}

static int w35n01jw_late_init(struct spinand_device *spinand)
{
	int ret;

	ret = w35n01jw_write_vcr_op(spinand, W35N01JW_VCR_IO_MODE_ADDR,
				    W35N01JW_VCR_IO_MODE_ADDR_OCTAL_SDR);
	if (ret)
		return ret;

	ret = w35n01jw_write_vcr_op(spinand, W35N01JW_VCR_DUMMY_CLOCK_ADDR,
				    W35N01JW_VCR_DUMMY_CLOCK_ADDR_20);
	if (ret)
		return ret;

	return 0;
}

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

static int w25n02kv_ooblayout_ecc(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *region)
{
	if (section > 3)
		return -ERANGE;

	region->offset = 64 + (16 * section);
	region->length = 13;

	return 0;
}

static int w25n02kv_ooblayout_free(struct mtd_info *mtd, int section,
				   struct mtd_oob_region *region)
{
	if (section > 3)
		return -ERANGE;

	region->offset = (16 * section) + 2;
	region->length = 14;

	return 0;
}

static const struct mtd_ooblayout_ops w25n02kv_ooblayout = {
	.ecc = w25n02kv_ooblayout_ecc,
	.free = w25n02kv_ooblayout_free,
};

static int w25n02kv_ecc_get_status(struct spinand_device *spinand,
				   u8 status)
{
	struct nand_device *nand = spinand_to_nand(spinand);
	u8 mbf = 0;
	struct spi_mem_op op = SPINAND_GET_FEATURE_OP(0x30, spinand->scratchbuf);

	switch (status & STATUS_ECC_MASK) {
	case STATUS_ECC_NO_BITFLIPS:
		return 0;

	case STATUS_ECC_UNCOR_ERROR:
		return -EBADMSG;

	case STATUS_ECC_HAS_BITFLIPS:
		/*
		 * Let's try to retrieve the real maximum number of bitflips
		 * in order to avoid forcing the wear-leveling layer to move
		 * data around if it's not necessary.
		 */
		if (spi_mem_exec_op(spinand->spimem, &op))
			return nanddev_get_ecc_conf(nand)->strength;

		mbf = *(spinand->scratchbuf) >> 4;

		if (WARN_ON(mbf > nanddev_get_ecc_conf(nand)->strength || !mbf))
			return nanddev_get_ecc_conf(nand)->strength;

		return mbf;

	default:
		break;
	}

	return -EINVAL;
}

static const struct spinand_info winbond_spinand_table[] = {
	SPINAND_INFO("W25M02GV",
		     SPINAND_ID(SPINAND_READID_METHOD_OPCODE_DUMMY, 0xab, 0x21),
		     NAND_MEMORG(1, 2048, 64, 64, 1024, 20, 1, 1, 2),
		     NAND_ECCREQ(1, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
					      &write_cache_variants,
					      &update_cache_variants),
		     0,
		     SPINAND_ECCINFO(&w25m02gv_ooblayout, NULL),
		     SPINAND_SELECT_TARGET(w25m02gv_select_target)),
	SPINAND_INFO("W25N01GV",
		     SPINAND_ID(SPINAND_READID_METHOD_OPCODE_DUMMY, 0xaa, 0x21),
		     NAND_MEMORG(1, 2048, 64, 64, 1024, 20, 1, 1, 1),
		     NAND_ECCREQ(1, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
					      &write_cache_variants,
					      &update_cache_variants),
		     0,
		     SPINAND_ECCINFO(&w25m02gv_ooblayout, NULL)),
	SPINAND_INFO("W25N02KV",
		     SPINAND_ID(SPINAND_READID_METHOD_OPCODE_DUMMY, 0xaa, 0x22),
		     NAND_MEMORG(1, 2048, 128, 64, 2048, 40, 1, 1, 1),
		     NAND_ECCREQ(8, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
					      &write_cache_variants,
					      &update_cache_variants),
		     0,
		     SPINAND_ECCINFO(&w25n02kv_ooblayout, w25n02kv_ecc_get_status)),
	SPINAND_INFO("W25N01JW",
		     SPINAND_ID(SPINAND_READID_METHOD_OPCODE_DUMMY, 0xbc, 0x21),
		     NAND_MEMORG(1, 2048, 64, 64, 1024, 20, 1, 1, 1),
		     NAND_ECCREQ(1, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
					      &write_cache_variants,
					      &update_cache_variants),
		     SPINAND_HAS_QE_BIT,
		     SPINAND_ECCINFO(&w25n01jw_ooblayout, NULL),
		     SPINAND_LATE_INIT(w25n01jw_late_init)),
	SPINAND_INFO("W35N01JW",
		     SPINAND_ID(SPINAND_READID_METHOD_OPCODE_DUMMY, 0xdc, 0x21),
		     NAND_MEMORG(1, 4096, 128, 64, 512, 20, 1, 1, 1),
		     NAND_ECCREQ(1, 512),
		     SPINAND_INFO_OP_VARIANTS(&octalio_read_cache_variants,
					      &x8_write_cache_variants,
					      &x8_update_cache_variants),
		     SPINAND_HAS_CR_FEAT_BIT,
		     SPINAND_ECCINFO(&w35n01jw_ooblayout, NULL),
		     SPINAND_LATE_INIT(w35n01jw_late_init)),
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
