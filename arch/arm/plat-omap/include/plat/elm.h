/*
 * BCH Error Location Module for TI81xx
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef OMAP_ELM_H
#define OMAP_ELM_H

enum omap_bch_ecc {
	OMAP_BCH4_ECC = 0,
	OMAP_BCH8_ECC,
	OMAP_BCH16_ECC,
};

#define BCH8_ECC_BYTES			(512)
#define BCH8_ECC_OOB_BYTES		(13)
#define BCH_MAX_ECC_BYTES_PER_SECTOR	(28)
#define BCH8_ECC_MAX	((BCH8_ECC_BYTES + BCH8_ECC_OOB_BYTES) * 8)

int omap_elm_decode_bch_error(int bch_type, char *ecc_calc,
		unsigned int *err_loc);
void omap_configure_elm(struct mtd_info *mtdi, int bch_type);
#endif /* OMAP_ELM_H */
