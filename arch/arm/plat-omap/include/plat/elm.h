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

#ifndef TI81_ELM_H
#define TI81_ELM_H

#define ELM_SYSCONFIG                0x010
#define ELM_SYSSTATUS                0x014
#define ELM_IRQSTATUS                0x018
#define ELM_IRQENABLE                0x01c
#define ELM_LOCATION_CONFIG          0x020
#define ELM_PAGE_CTRL                0x080
#define ELM_SYNDROME_FRAGMENT_0      0x400
#define ELM_SYNDROME_FRAGMENT_1      0x404
#define ELM_SYNDROME_FRAGMENT_2      0x408
#define ELM_SYNDROME_FRAGMENT_3      0x40c
#define ELM_SYNDROME_FRAGMENT_4      0x410
#define ELM_SYNDROME_FRAGMENT_5      0x414
#define ELM_SYNDROME_FRAGMENT_6      0x418
#define ELM_LOCATION_STATUS          0x800
#define ELM_ERROR_LOCATION_0         0x880
#define ELM_ERROR_LOCATION_1         0x884
#define ELM_ERROR_LOCATION_2         0x888
#define ELM_ERROR_LOCATION_3         0x88c
#define ELM_ERROR_LOCATION_4         0x890
#define ELM_ERROR_LOCATION_5         0x894
#define ELM_ERROR_LOCATION_6         0x898
#define ELM_ERROR_LOCATION_7         0x89c
#define ELM_ERROR_LOCATION_8         0x8a0
#define ELM_ERROR_LOCATION_9         0x8a4
#define ELM_ERROR_LOCATION_10        0x8a8
#define ELM_ERROR_LOCATION_11        0x8ac
#define ELM_ERROR_LOCATION_12        0x8b0
#define ELM_ERROR_LOCATION_13        0x8b4
#define ELM_ERROR_LOCATION_14        0x8b8
#define ELM_ERROR_LOCATION_15        0x8bc

/* ELM System Configuration Register */
#define ELM_SYSCONFIG_SOFTRESET      (1 << 1)
#define ELM_SYSCONFIG_SIDLE_MASK     (3 << 3)
#define ELM_SYSCONFIG_SMART_IDLE     (2 << 3)

/* ELM System Status Register */
#define ELM_SYSSTATUS_RESETDONE      (1 << 0)

/* ELM Interrupt Status Register */
#define INTR_STATUS_PAGE_VALID       (1 << 8)
#define INTR_STATUS_LOC_VALID_7      (1 << 7)
#define INTR_STATUS_LOC_VALID_6      (1 << 6)
#define INTR_STATUS_LOC_VALID_5      (1 << 5)
#define INTR_STATUS_LOC_VALID_4      (1 << 4)
#define INTR_STATUS_LOC_VALID_3      (1 << 3)
#define INTR_STATUS_LOC_VALID_2      (1 << 2)
#define INTR_STATUS_LOC_VALID_1      (1 << 1)
#define INTR_STATUS_LOC_VALID_0      (1 << 0)

/* ELM Interrupt Enable Register */
#define INTR_EN_PAGE_MASK            (1 << 8)
#define INTR_EN_LOCATION_MASK_7      (1 << 7)
#define INTR_EN_LOCATION_MASK_6      (1 << 6)
#define INTR_EN_LOCATION_MASK_5      (1 << 5)
#define INTR_EN_LOCATION_MASK_4      (1 << 4)
#define INTR_EN_LOCATION_MASK_3      (1 << 3)
#define INTR_EN_LOCATION_MASK_2      (1 << 2)
#define INTR_EN_LOCATION_MASK_1      (1 << 1)
#define INTR_EN_LOCATION_MASK_0      (1 << 0)

/* ELM Location Configuration Register */
#define ECC_SIZE_MASK                (0x7ff << 16)
#define ECC_BCH_LEVEL_MASK           (0x3 << 0)
#define ECC_BCH4_LEVEL               (0x0 << 0)
#define ECC_BCH8_LEVEL               (0x1 << 0)
#define ECC_BCH16_LEVEL              (0x2 << 0)

/* ELM Page Definition Register */
#define PAGE_MODE_SECTOR_7           (1 << 7)
#define PAGE_MODE_SECTOR_6           (1 << 6)
#define PAGE_MODE_SECTOR_5           (1 << 5)
#define PAGE_MODE_SECTOR_4           (1 << 4)
#define PAGE_MODE_SECTOR_3           (1 << 3)
#define PAGE_MODE_SECTOR_2           (1 << 2)
#define PAGE_MODE_SECTOR_1           (1 << 1)
#define PAGE_MODE_SECTOR_0           (1 << 0)

/* ELM syndrome */
#define ELM_SYNDROME_VALID           (1 << 16)

/* ELM_LOCATION_STATUS Register */
#define ECC_CORRECTABLE_MASK         (1 << 8)
#define ECC_NB_ERRORS_MASK           (0x1f << 0)

/*  ELM_ERROR_LOCATION_0-15 Registers */
#define ECC_ERROR_LOCATION_MASK      (0x1fff << 0)

extern int elm_decode_bch_error(int bch_type, char *ecc_calc,
		unsigned int *err_loc);
#endif /* TI81_ELM_H */
