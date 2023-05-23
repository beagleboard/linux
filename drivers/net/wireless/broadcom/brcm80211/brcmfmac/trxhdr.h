/* SPDX-License-Identifier: ISC */
/* Copyright (c) 2020 Cypress Semiconductor Corporation */

#ifndef BRCMFMAC_TRXHDR_H
#define BRCMFMAC_TRXHDR_H

/* Bootloader makes special use of trx header "offsets" array */
enum {
	TRX_OFFSET_SIGN_INFO_IDX		= 0,
	TRX_OFFSET_DATA_FOR_SIGN1_IDX		= 1,
	TRX_OFFSET_DATA_FOR_SIGN2_IDX		= 2,
	TRX_OFFSET_ROOT_MODULUS_IDX		= 3,
	TRX_OFFSET_ROOT_EXPONENT_IDX		= 67,
	TRX_OFFSET_CONT_MODULUS_IDX		= 68,
	TRX_OFFSET_CONT_EXPONENT_IDX		= 132,
	TRX_OFFSET_HASH_FW_IDX			= 133,
	TRX_OFFSET_FW_LEN_IDX			= 149,
	TRX_OFFSET_TR_RST_IDX			= 150,
	TRX_OFFSET_FW_VER_FOR_ANTIROOLBACK_IDX	= 151,
	TRX_OFFSET_IV_IDX			= 152,
	TRX_OFFSET_NONCE_IDX			= 160,
	TRX_OFFSET_SIGN_INFO2_IDX		= 168,
	TRX_OFFSET_MAX_IDX
};

#define TRX_MAGIC	0x30524448		/* "HDR0" */
#define TRX_VERSION	4			/* Version 4 */
#define TRX_MAX_OFFSET	TRX_OFFSET_MAX_IDX	/* Max number of file offsets */

struct trx_header_le {
	__le32 magic;		/* "HDR0" */
	__le32 len;		/* Length of file including header */
	__le32 crc32;		/* CRC from flag_version to end of file */
	__le32 flag_version;	/* 0:15 flags, 16:31 version */
	__le32 offsets[TRX_MAX_OFFSET];	/* Offsets of partitions */
};

#endif /* BRCMFMAC_TRXHDR_H */
