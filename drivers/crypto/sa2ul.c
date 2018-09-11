// SPDX-License-Identifier: GPL-2.0
/*
 * AM6 SA2UL crypto accelerator driver
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors:	Keerthy
 *              Vitaly Andrianov
 */
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/dmapool.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/dmaengine.h>
#include <linux/cryptohash.h>

#include <crypto/authenc.h>
#include <crypto/des.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/skcipher.h>
#include <crypto/internal/hash.h>
#include <crypto/scatterwalk.h>
#include <crypto/sha.h>

#include "sa2ul.h"

/* Byte offset for key in encryption security context */
#define SC_ENC_KEY_OFFSET (1 + 27 + 4)
/* Byte offset for Aux-1 in encryption security context */
#define SC_ENC_AUX1_OFFSET (1 + 27 + 4 + 32)

#define SA_CMDL_UPD_ENC         0x0001
#define SA_CMDL_UPD_AUTH        0x0002
#define SA_CMDL_UPD_ENC_IV      0x0004
#define SA_CMDL_UPD_AUTH_IV     0x0008
#define SA_CMDL_UPD_AUX_KEY     0x0010

#define SA_AUTH_SUBKEY_LEN	16
#define SA_CMDL_PAYLOAD_LENGTH_MASK	0xFFFF
#define SA_CMDL_SOP_BYPASS_LEN_MASK	0xFF000000

#define MODE_CONTROL_BYTES	27
#define SA_HASH_PROCESSING	0
#define SA_CRYPTO_PROCESSING	0
#define SA_UPLOAD_HASH_TO_TLR	BIT(6)

#define SA_SW0_FLAGS_MASK	0xF0000
#define SA_SW0_CMDL_INFO_MASK	0x1F00000
#define SA_SW0_CMDL_PRESENT	BIT(4)
#define SA_SW0_ENG_ID_MASK	0x3E000000
#define SA_SW0_DEST_INFO_PRESENT	BIT(30)
#define SA_SW2_EGRESS_LENGTH		0xFF000000

#define SHA256_DIGEST_WORDS    8
/* Make 32-bit word from 4 bytes */
#define SA_MK_U32(b0, b1, b2, b3) (((b0) << 24) | ((b1) << 16) | \
				   ((b2) << 8) | (b3))

/* size of SCCTL structure in bytes */
#define SA_SCCTL_SZ 16

/* Max Authentication tag size */
#define SA_MAX_AUTH_TAG_SZ 64

static struct device *sa_k3_dev;

/**
 * struct sa_cmdl_cfg - Command label configuration descriptor
 * @enc1st: If the iteration needs encryption before authentication
 * @aalg: authentication algorithm ID
 * @enc_eng_id: Encryption Engine ID supported by the SA hardware
 * @auth_eng_id: authentication Engine ID
 * @iv_size: Initialization Vector size
 * @akey: Authentication key
 * @akey_len: Authentication key length
 */
struct sa_cmdl_cfg {
	int enc1st;
	int aalg;
	u8 enc_eng_id;
	u8 auth_eng_id;
	u8 iv_size;
	const u8 *akey;
	u16 akey_len;
	u16 auth_subkey_len;
};

/**
 * struct algo_data - Crypto algorithm specific data
 * @enc_eng: Encryption engine info structure
 * @auth_eng: Authentication engine info structure
 * @auth_ctrl: Authentication control word
 * @hash_size: Size of Digest
 * @ealg_id: Encryption Algorithm ID
 * @aalg_id: Authentication algorithm ID
 * @mci_enc: Mode Control Instruction for Encryption algorithm
 * @mci_dec: Mode Control Instruction for Decryption
 * @inv_key: Whether the encryption algorithm demands key inversion
 * @keyed_mac: Whether the Authentication algorithm has Key
 * @prep_iopad: Function pointer to generate intermediate ipad/opad
 */
struct algo_data {
	struct sa_eng_info enc_eng;
	struct sa_eng_info auth_eng;
	u8 auth_ctrl;
	u8 hash_size;
	u8 ealg_id;
	u8 aalg_id;
	u8 *mci_enc;
	u8 *mci_dec;
	bool inv_key;
	bool keyed_mac;
	void (*prep_iopad)(const u8 *key, u16 key_sz, u32 *ipad, u32 *opad);
};

/**
 * struct sa_alg_tmpl: A generic template encompassing crypto/aead algorithms
 * @alg: A union of aead/crypto algorithm type.
 * @registered: Flag indicating if the crypto algorithm is already registered
 */
struct sa_alg_tmpl {
	u32 type;		/* CRYPTO_ALG_TYPE from <linux/crypto.h> */
	union {
		struct crypto_alg crypto;
		struct aead_alg aead;
	} alg;
	int registered;
};

/**
 * struct sa_rx_data: RX Packet miscellaneous data place holder
 * @req: crypto request data pointer
 * @tx_in: dma_async_tx_descriptor pointer for rx channel
 * @enc: Flag indicating either encryption or decryption
 */
struct sa_rx_data {
	void *req;
	struct dma_async_tx_descriptor *tx_in;
	u8 enc;
};

/*
 * Mode Control Instructions for various Key lengths 128, 192, 256
 * For ECB (Electronic Code Book) mode for encryption
 */
static u8 mci_ecb_enc_array[3][27] = {
	{	0x21, 0x00, 0x00, 0x80, 0x8a, 0x04, 0xb7, 0x90, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
	{	0x21, 0x00, 0x00, 0x84, 0x8a, 0x04, 0xb7, 0x90, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
	{	0x21, 0x00, 0x00, 0x88, 0x8a, 0x04, 0xb7, 0x90, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
};

/*
 * Mode Control Instructions for various Key lengths 128, 192, 256
 * For ECB (Electronic Code Book) mode for decryption
 */
static u8 mci_ecb_dec_array[3][27] = {
	{	0x31, 0x00, 0x00, 0x80, 0x8a, 0x04, 0xb7, 0x90, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
	{	0x31, 0x00, 0x00, 0x84, 0x8a, 0x04, 0xb7, 0x90, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
	{	0x31, 0x00, 0x00, 0x88, 0x8a, 0x04, 0xb7, 0x90, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
};

/*
 * Mode Control Instructions for various Key lengths 128, 192, 256
 * For CBC (Cipher Block Chaining) mode for encryption
 */
static u8 mci_cbc_enc_array[3][MODE_CONTROL_BYTES] = {
	{	0x21, 0x00, 0x00, 0x18, 0x88, 0x0a, 0xaa, 0x4b, 0x7e, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
	{	0x21, 0x00, 0x00, 0x18, 0x88, 0x4a, 0xaa, 0x4b, 0x7e, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
	{	0x21, 0x00, 0x00, 0x18, 0x88, 0x8a, 0xaa, 0x4b, 0x7e, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
};

/*
 * Mode Control Instructions for various Key lengths 128, 192, 256
 * For CBC (Cipher Block Chaining) mode for decryption
 */
static u8 mci_cbc_dec_array[3][MODE_CONTROL_BYTES] = {
	{	0x31, 0x00, 0x00, 0x80, 0x8a, 0xca, 0x98, 0xf4, 0x40, 0xc0,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
	{	0x31, 0x00, 0x00, 0x84, 0x8a, 0xca, 0x98, 0xf4, 0x40, 0xc0,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
	{	0x31, 0x00, 0x00, 0x88, 0x8a, 0xca, 0x98, 0xf4, 0x40, 0xc0,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
};

/*
 * Mode Control Instructions for DES algorithm
 * For CBC (Cipher Block Chaining) mode and ECB mode
 * encryption and for decryption respectively
 */
static u8 mci_cbc_3des_enc_array[MODE_CONTROL_BYTES] = {
	0x20, 0x00, 0x00, 0x18, 0x88, 0x52, 0xaa, 0x4b, 0x7e, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00,
};

static u8 mci_cbc_3des_dec_array[MODE_CONTROL_BYTES] = {
	0x30, 0x00, 0x00, 0x85, 0x0a, 0xca, 0x98, 0xf4, 0x40, 0xc0, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00,
};

static u8 mci_ecb_3des_enc_array[MODE_CONTROL_BYTES] = {
	0x20, 0x00, 0x00, 0x85, 0x0a, 0x04, 0xb7, 0x90, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00,
};

static u8 mci_ecb_3des_dec_array[MODE_CONTROL_BYTES] = {
	0x30, 0x00, 0x00, 0x85, 0x0a, 0x04, 0xb7, 0x90, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00,
};

/*
 * Perform 16 byte or 128 bit swizzling
 * The SA2UL Expects the security context to
 * be in little Endian and the bus width is 128 bits or 16 bytes
 * Hence swap 16 bytes at a time from higher to lower address
 */
static void sa_swiz_128(u8 *in, u16 len)
{
	u8 data[16];
	int i, j;

	for (i = 0; i < len; i += 16) {
		memcpy(data, &in[i], 16);
		for (j = 0; j < 16; j++)
			in[i + j] = data[15 - j];
	}
}

/* Prepare the ipad and opad from key as per SHA algorithm step 1*/
static void prepare_kiopad(u8 *k_ipad, u8 *k_opad, const u8 *key, u16 key_sz)
{
	int i;

	for (i = 0; i < key_sz; i++) {
		k_ipad[i] = key[i] ^ 0x36;
		k_opad[i] = key[i] ^ 0x5c;
	}

	/* Instead of XOR with 0 */
	for (; i < SHA_MESSAGE_BYTES; i++) {
		k_ipad[i] = 0x36;
		k_opad[i] = 0x5c;
	}
}

void sha256_init(u32 *buf)
{
	buf[0] = SHA256_H0;
	buf[1] = SHA256_H1;
	buf[2] = SHA256_H2;
	buf[3] = SHA256_H3;
	buf[4] = SHA256_H4;
	buf[5] = SHA256_H5;
	buf[6] = SHA256_H6;
	buf[7] = SHA256_H7;
}

static void sa_hmac_sha256_get_pad(const u8 *key, u16 key_sz, u32 *ipad,
				   u32 *opad)
{
	u8 k_ipad[SHA_MESSAGE_BYTES];
	u8 k_opad[SHA_MESSAGE_BYTES];
	int i;

	prepare_kiopad(k_ipad, k_opad, key, key_sz);

	/* SHA-256 on k_ipad */
	sha256_init(ipad);
	sha256_transform(ipad, k_ipad);

	for (i = 0; i < SHA256_DIGEST_WORDS; i++)
		ipad[i] = cpu_to_be32(ipad[i]);

	/* SHA-256 on k_opad */
	sha256_init(opad);
	sha256_transform(opad, k_opad);

	for (i = 0; i < SHA256_DIGEST_WORDS; i++)
		opad[i] = cpu_to_be32(opad[i]);
}

/* Generate HMAC-SHA1 intermediate Hash */
static
void sa_hmac_sha1_get_pad(const u8 *key, u16 key_sz, u32 *ipad, u32 *opad)
{
	u32 ws[SHA_WORKSPACE_WORDS];
	u8 k_ipad[SHA_MESSAGE_BYTES];
	u8 k_opad[SHA_MESSAGE_BYTES];
	int i;

	prepare_kiopad(k_ipad, k_opad, key, key_sz);

	/* SHA-1 on k_ipad */
	sha_init(ipad);
	sha_transform(ipad, k_ipad, ws);

	for (i = 0; i < SHA_DIGEST_WORDS; i++)
		ipad[i] = cpu_to_be32(ipad[i]);

	/* SHA-1 on k_opad */
	sha_init(opad);
	sha_transform(opad, k_opad, ws);

	for (i = 0; i < SHA_DIGEST_WORDS; i++)
		opad[i] = cpu_to_be32(opad[i]);
}

/* Derive the inverse key used in AES-CBC decryption operation */
static inline int sa_aes_inv_key(u8 *inv_key, const u8 *key, u16 key_sz)
{
	struct crypto_aes_ctx ctx;
	int key_pos;

	if (crypto_aes_expand_key(&ctx, key, key_sz)) {
		pr_err("%s: bad key len(%d)\n", __func__, key_sz);
		return -EINVAL;
	}

	/* Based crypto_aes_expand_key logic */
	switch (key_sz) {
	case AES_KEYSIZE_128:
	case AES_KEYSIZE_192:
		key_pos = key_sz + 24;
		break;

	case AES_KEYSIZE_256:
		key_pos = key_sz + 24 - 4;
		break;

	default:
		pr_err("%s: bad key len(%d)\n", __func__, key_sz);
		return -EINVAL;
	}

	memcpy(inv_key, &ctx.key_enc[key_pos], key_sz);
	return 0;
}

/* Set Security context for the encryption engine */
static int sa_set_sc_enc(struct algo_data *ad, const u8 *key, u16 key_sz,
			 u16 aad_len, u8 enc, u8 *sc_buf)
{
	const u8 *mci = NULL;

	/* Set Encryption mode selector to crypto processing */
	sc_buf[0] = SA_CRYPTO_PROCESSING;

	if (enc)
		mci = ad->mci_enc;
	else
		mci = ad->mci_dec;
	/* Set the mode control instructions in security context */
	if (mci)
		memcpy(&sc_buf[1], mci, MODE_CONTROL_BYTES);

	/* For AES-CBC decryption get the inverse key */
	if (ad->inv_key && !enc) {
		if (sa_aes_inv_key(&sc_buf[SC_ENC_KEY_OFFSET], key, key_sz))
			return -EINVAL;
	/* For all other cases: key is used */
	} else {
		memcpy(&sc_buf[SC_ENC_KEY_OFFSET], key, key_sz);
	}

	return 0;
}

/* Set Security context for the authentication engine */
static void sa_set_sc_auth(struct algo_data *ad, const u8 *key, u16 key_sz,
			   u8 *sc_buf)
{
	u32 ipad[64], opad[64];

	/* Set Authentication mode selector to hash processing */
	sc_buf[0] = SA_HASH_PROCESSING;
	/* Auth SW ctrl word: bit[6]=1 (upload computed hash to TLR section) */
	sc_buf[1] = SA_UPLOAD_HASH_TO_TLR;
	sc_buf[1] |= ad->auth_ctrl;

	/* Copy the keys or ipad/opad */
	if (ad->keyed_mac) {
		ad->prep_iopad(key, key_sz, ipad, opad);
		/* Copy ipad to AuthKey */
		memcpy(&sc_buf[32], ipad, ad->hash_size);
		/* Copy opad to Aux-1 */
		memcpy(&sc_buf[64], opad, ad->hash_size);
	}
}

static inline void sa_copy_iv(u32 *out, const u8 *iv, bool size16)
{
	int j;

	for (j = 0; j < ((size16) ? 4 : 2); j++) {
		*out = cpu_to_be32(*((u32 *)iv));
		iv += 4;
		out++;
	}
}

/* Format general command label */
static int sa_format_cmdl_gen(struct sa_cmdl_cfg *cfg, u8 *cmdl,
			      struct sa_cmdl_upd_info *upd_info)
{
	u8 enc_offset = 0, auth_offset = 0, total = 0;
	u8 enc_next_eng = SA_ENG_ID_OUTPORT2;
	u8 auth_next_eng = SA_ENG_ID_OUTPORT2;
	u32 *word_ptr = (u32 *)cmdl;
	int i;

	/* Clear the command label */
	memzero_explicit(cmdl, (SA_MAX_CMDL_WORDS * sizeof(u32)));

	/* Iniialize the command update structure */
	memzero_explicit(upd_info, sizeof(*upd_info));

	if (cfg->enc1st) {
		if (cfg->enc_eng_id != SA_ENG_ID_NONE)
			auth_offset = SA_CMDL_HEADER_SIZE_BYTES;

		if (cfg->iv_size)
			auth_offset += cfg->iv_size;

		if (cfg->auth_eng_id != SA_ENG_ID_NONE)
			enc_next_eng = cfg->auth_eng_id;
		else
			enc_next_eng = SA_ENG_ID_OUTPORT2;
	} else {
		if (cfg->auth_eng_id != SA_ENG_ID_NONE)
			enc_offset = SA_CMDL_HEADER_SIZE_BYTES;

		if (cfg->auth_subkey_len)
			enc_offset += cfg->auth_subkey_len;

		if (cfg->enc_eng_id != SA_ENG_ID_NONE)
			auth_next_eng = cfg->enc_eng_id;
		else
			auth_next_eng = SA_ENG_ID_OUTPORT2;
	}

	if (cfg->enc_eng_id != SA_ENG_ID_NONE) {
		upd_info->flags |= SA_CMDL_UPD_ENC;
		upd_info->enc_size.index = enc_offset >> 2;
		upd_info->enc_offset.index = upd_info->enc_size.index + 1;
		/* Encryption command label */
		cmdl[enc_offset + SA_CMDL_OFFSET_NESC] = enc_next_eng;

		/* Encryption modes requiring IV */
		if (cfg->iv_size) {
			upd_info->flags |= SA_CMDL_UPD_ENC_IV;
			upd_info->enc_iv.index =
				(enc_offset + SA_CMDL_HEADER_SIZE_BYTES) >> 2;
			upd_info->enc_iv.size = cfg->iv_size;

			cmdl[enc_offset + SA_CMDL_OFFSET_LABEL_LEN] =
				SA_CMDL_HEADER_SIZE_BYTES + cfg->iv_size;

			cmdl[enc_offset + SA_CMDL_OFFSET_OPTION_CTRL1] =
				(SA_CTX_ENC_AUX2_OFFSET | (cfg->iv_size >> 3));
			enc_offset += SA_CMDL_HEADER_SIZE_BYTES + cfg->iv_size;
		} else {
			cmdl[enc_offset + SA_CMDL_OFFSET_LABEL_LEN] =
						SA_CMDL_HEADER_SIZE_BYTES;
			enc_offset += SA_CMDL_HEADER_SIZE_BYTES;
		}
	}

	if (cfg->auth_eng_id != SA_ENG_ID_NONE) {
		upd_info->flags |= SA_CMDL_UPD_AUTH;
		upd_info->auth_size.index = auth_offset >> 2;
		upd_info->auth_offset.index = upd_info->auth_size.index + 1;
		cmdl[auth_offset + SA_CMDL_OFFSET_NESC] = auth_next_eng;

		/* Algorithm with subkeys */
		if (cfg->aalg == SA_AALG_ID_AES_XCBC ||
		    cfg->aalg == SA_AALG_ID_CMAC) {
			upd_info->flags |= SA_CMDL_UPD_AUX_KEY;
			upd_info->aux_key_info.index =
				(auth_offset + SA_CMDL_HEADER_SIZE_BYTES) >> 2;
			cmdl[auth_offset + SA_CMDL_OFFSET_LABEL_LEN] =
						SA_CMDL_HEADER_SIZE_BYTES +
							cfg->auth_subkey_len;
			cmdl[auth_offset + SA_CMDL_OFFSET_OPTION_CTRL1] =
						(SA_CTX_ENC_AUX1_OFFSET |
						(cfg->auth_subkey_len  >> 3));

			auth_offset += SA_CMDL_HEADER_SIZE_BYTES +
							cfg->auth_subkey_len;
		} else {
			cmdl[auth_offset + SA_CMDL_OFFSET_LABEL_LEN] =
						SA_CMDL_HEADER_SIZE_BYTES;
			auth_offset += SA_CMDL_HEADER_SIZE_BYTES;
		}
	}

	if (cfg->enc1st)
		total = auth_offset;
	else
		total = enc_offset;

	total = roundup(total, 8);

	for (i = 0; i < total / 4; i++)
		word_ptr[i] = be32_to_cpu(word_ptr[i]);

	return total;
}

/* Update Command label */
static inline void
sa_update_cmdl(struct device *dev, u8 enc_offset, u16 enc_size, u8 *enc_iv,
	       u8 auth_offset, u16 auth_size, u8 *auth_iv, u8 aad_size,
	       u8 *aad, struct sa_cmdl_upd_info *upd_info, u32 *cmdl)
{
	int i = 0, j;

	if (upd_info->submode != SA_MODE_GEN) {
		dev_err(dev, "unsupported mode(%d)\n", upd_info->submode);
		return;
	}

	if (likely(upd_info->flags & SA_CMDL_UPD_ENC)) {
		cmdl[upd_info->enc_size.index] &= ~SA_CMDL_PAYLOAD_LENGTH_MASK;
		cmdl[upd_info->enc_size.index] |= enc_size;
		cmdl[upd_info->enc_offset.index] &=
						~SA_CMDL_SOP_BYPASS_LEN_MASK;
		cmdl[upd_info->enc_offset.index] |=
			((u32)enc_offset << __ffs(SA_CMDL_SOP_BYPASS_LEN_MASK));

		if (likely(upd_info->flags & SA_CMDL_UPD_ENC_IV)) {
			u32 *data = &cmdl[upd_info->enc_iv.index];

			for (j = 0; i < upd_info->enc_iv.size; i += 4, j++) {
				data[j] = cpu_to_be32(*((u32 *)enc_iv));
				enc_iv += 4;
			}
		}
	}

	if (likely(upd_info->flags & SA_CMDL_UPD_AUTH)) {
		cmdl[upd_info->auth_size.index] &= ~SA_CMDL_PAYLOAD_LENGTH_MASK;
		cmdl[upd_info->auth_size.index] |= auth_size;
		cmdl[upd_info->auth_offset.index] &=
						~SA_CMDL_SOP_BYPASS_LEN_MASK;
		cmdl[upd_info->auth_offset.index] |= ((u32)auth_offset <<
					__ffs(SA_CMDL_SOP_BYPASS_LEN_MASK));
		if (upd_info->flags & SA_CMDL_UPD_AUTH_IV) {
			sa_copy_iv(&cmdl[upd_info->auth_iv.index], auth_iv,
				   (upd_info->auth_iv.size > 8));
		}

		if (upd_info->flags & SA_CMDL_UPD_AUX_KEY) {
			int offset = (auth_size & 0xF) ? 4 : 0;

			memcpy(&cmdl[upd_info->aux_key_info.index],
			       &upd_info->aux_key[offset], 16);
		}
	}
}

/* Format SWINFO words to be sent to SA */
static
void sa_set_swinfo(u8 eng_id, u16 sc_id, dma_addr_t sc_phys,
		   u8 cmdl_present, u8 cmdl_offset, u8 flags,
		   u8 hash_size, u32 *swinfo)
{
	swinfo[0] = sc_id;
	swinfo[0] |= (flags << __ffs(SA_SW0_FLAGS_MASK));
	if (likely(cmdl_present))
		swinfo[0] |= ((cmdl_offset | SA_SW0_CMDL_PRESENT) <<
						__ffs(SA_SW0_CMDL_INFO_MASK));
	swinfo[0] |= (eng_id << __ffs(SA_SW0_ENG_ID_MASK));

	swinfo[0] |= SA_SW0_DEST_INFO_PRESENT;
	swinfo[1] = (u32)(sc_phys & 0xFFFFFFFFULL);
	swinfo[2] = (u32)((sc_phys & 0xFFFFFFFF00000000ULL) >> 32);
	swinfo[2] |= (hash_size << __ffs(SA_SW2_EGRESS_LENGTH));
}

/* Dump the security context */
static void sa_dump_sc(u8 *buf, dma_addr_t dma_addr)
{
#ifdef DEBUG
	dev_info(sa_k3_dev, "Security context dump:: 0x%pad\n", &dma_addr);
	print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET,
		       16, 1, buf, SA_CTX_MAX_SZ, false);
#endif
}

static
int sa_init_sc(struct sa_ctx_info *ctx, const u8 *enc_key,
	       u16 enc_key_sz, const u8 *auth_key, u16 auth_key_sz,
	       struct algo_data *ad, u8 enc, u32 *swinfo, bool auth_req)
{
	int use_enc = 0;
	int enc_sc_offset, auth_sc_offset;
	u8 *sc_buf = ctx->sc;
	u16 sc_id = ctx->sc_id;
	u16 aad_len = 0;	/* Currently not supporting AEAD algo */
	u8 first_engine;

	memzero_explicit(sc_buf, SA_CTX_MAX_SZ);

	if (ad->auth_eng.eng_id <= SA_ENG_ID_EM2 || !auth_req)
		use_enc = 1;

	/* Determine the order of encryption & Authentication contexts */
	if (enc || !use_enc) {
		if (auth_req) {
			enc_sc_offset = SA_CTX_PHP_PE_CTX_SZ;
			auth_sc_offset = enc_sc_offset + ad->enc_eng.sc_size;
		} else {
			enc_sc_offset = SA_CTX_PHP_PE_CTX_SZ;
		}
	} else {
		auth_sc_offset = SA_CTX_PHP_PE_CTX_SZ;
		enc_sc_offset = auth_sc_offset + ad->auth_eng.sc_size;
	}

	/* SCCTL Owner info: 0=host, 1=CP_ACE */
	sc_buf[SA_CTX_SCCTL_OWNER_OFFSET] = 0;
	/* SCCTL F/E control */
	if (auth_req)
		sc_buf[1] = SA_SCCTL_FE_AUTH_ENC;
	else
		sc_buf[1] = SA_SCCTL_FE_ENC;
	memcpy(&sc_buf[2], &sc_id, 2);
	sc_buf[4] = 0x0;
	sc_buf[5] = 0x0;
	sc_buf[6] = 0x0;
	sc_buf[7] = 0x0;

	/* Initialize the rest of PHP context */
	memzero_explicit(sc_buf + SA_SCCTL_SZ, SA_CTX_PHP_PE_CTX_SZ -
			 SA_SCCTL_SZ);

	/* Prepare context for encryption engine */
	if (ad->enc_eng.sc_size) {
		if (sa_set_sc_enc(ad, enc_key, enc_key_sz, aad_len,
				  enc, &sc_buf[enc_sc_offset]))
			return -EINVAL;
	}

	/* Prepare context for authentication engine */
	if (ad->auth_eng.sc_size) {
		if (use_enc) {
			if (sa_set_sc_enc(ad, auth_key, auth_key_sz,
					  aad_len, 0, &sc_buf[auth_sc_offset]))
				return -EINVAL;
		} else {
			sa_set_sc_auth(ad, auth_key, auth_key_sz,
				       &sc_buf[auth_sc_offset]);
		}
	}

	/* Set the ownership of context to CP_ACE */
	sc_buf[SA_CTX_SCCTL_OWNER_OFFSET] = 0x80;

	/* swizzle the security context */
	sa_swiz_128(sc_buf, SA_CTX_MAX_SZ);
	/* Setup SWINFO */
	if (!auth_req)
		first_engine = ad->enc_eng.eng_id;
	else
		first_engine = enc ? ad->enc_eng.eng_id : ad->auth_eng.eng_id;

	if (auth_req) {
		if (!ad->hash_size)
			return -EINVAL;
		/* Round up the tag size to multiple of 4 */
		ad->hash_size = roundup(ad->hash_size, 8);
	}

	sa_set_swinfo(first_engine, ctx->sc_id, ctx->sc_phys, 1, 0,
		      SA_SW_INFO_FLAG_EVICT, ad->hash_size, swinfo);

	sa_dump_sc(sc_buf, ctx->sc_phys);

	return 0;
}

/* Free the per direction context memory */
static void sa_free_ctx_info(struct sa_ctx_info *ctx,
			     struct sa_crypto_data *data)
{
	unsigned long bn;

	bn = ctx->sc_id - data->sc_id_start;
	spin_lock(&data->scid_lock);
	__clear_bit(bn, data->ctx_bm);
	data->sc_id--;
	spin_unlock(&data->scid_lock);

	if (ctx->sc) {
		dma_pool_free(data->sc_pool, ctx->sc, ctx->sc_phys);
		ctx->sc = NULL;
	}
}

static int sa_init_ctx_info(struct sa_ctx_info *ctx,
			    struct sa_crypto_data *data)
{
	unsigned long bn;
	int err;

	spin_lock(&data->scid_lock);
	bn = find_first_zero_bit(data->ctx_bm, SA_MAX_NUM_CTX);
	__set_bit(bn, data->ctx_bm);
	data->sc_id++;
	spin_unlock(&data->scid_lock);

	ctx->sc_id = (u16)(data->sc_id_start + bn);

	ctx->sc = dma_pool_alloc(data->sc_pool, GFP_KERNEL, &ctx->sc_phys);
	if (!ctx->sc) {
		dev_err(&data->pdev->dev, "Failed to allocate SC memory\n");
		err = -ENOMEM;
		goto scid_rollback;
	}

	return 0;

scid_rollback:
	spin_lock(&data->scid_lock);
	__clear_bit(bn, data->ctx_bm);
	data->sc_id--;
	spin_unlock(&data->scid_lock);

	return err;
}

static void sa_aes_cra_exit(struct crypto_tfm *tfm)
{
	struct crypto_alg *alg = tfm->__crt_alg;
	struct sa_tfm_ctx *ctx = crypto_tfm_ctx(tfm);
	struct sa_crypto_data *data = dev_get_drvdata(sa_k3_dev);

	dev_dbg(sa_k3_dev, "%s(0x%p) sc-ids(0x%x(0x%pad), 0x%x(0x%pad))\n",
		__func__, tfm, ctx->enc.sc_id, &ctx->enc.sc_phys,
		ctx->dec.sc_id, &ctx->dec.sc_phys);

	if ((alg->cra_flags & CRYPTO_ALG_TYPE_ABLKCIPHER)
	    == CRYPTO_ALG_TYPE_ABLKCIPHER) {
		sa_free_ctx_info(&ctx->enc, data);
		sa_free_ctx_info(&ctx->dec, data);
	}
}

static int sa_aes_cra_init(struct crypto_tfm *tfm)
{
	struct sa_tfm_ctx *ctx = crypto_tfm_ctx(tfm);
	struct crypto_alg *alg = tfm->__crt_alg;
	struct sa_crypto_data *data = dev_get_drvdata(sa_k3_dev);
	int ret;

	if ((alg->cra_flags & CRYPTO_ALG_TYPE_MASK) ==
	    CRYPTO_ALG_TYPE_ABLKCIPHER) {
		memzero_explicit(ctx, sizeof(*ctx));
		ctx->dev_data = data;

		ret = sa_init_ctx_info(&ctx->enc, data);
		if (ret)
			return ret;
		ret = sa_init_ctx_info(&ctx->dec, data);
		if (ret) {
			sa_free_ctx_info(&ctx->enc, data);
			return ret;
		}
	}

	dev_dbg(sa_k3_dev, "%s(0x%p) sc-ids(0x%x(0x%pad), 0x%x(0x%pad))\n",
		__func__, tfm, ctx->enc.sc_id, &ctx->enc.sc_phys,
		ctx->dec.sc_id, &ctx->dec.sc_phys);
	return 0;
}

static int sa_aes_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
			 unsigned int keylen, struct algo_data *ad)
{
	struct sa_tfm_ctx *ctx = crypto_ablkcipher_ctx(tfm);

	const char *cra_name;
	int cmdl_len;
	struct sa_cmdl_cfg cfg;

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 &&
	    keylen != AES_KEYSIZE_256)
		return -EINVAL;

	cra_name = crypto_tfm_alg_name(&tfm->base);

	memzero_explicit(&cfg, sizeof(cfg));
	cfg.enc1st = 1;
	cfg.enc_eng_id = ad->enc_eng.eng_id;
	cfg.iv_size = crypto_ablkcipher_ivsize(tfm);
	cfg.auth_eng_id = SA_ENG_ID_NONE;
	cfg.auth_subkey_len = 0;

	/* Setup Encryption Security Context & Command label template */
	if (sa_init_sc(&ctx->enc, key, keylen,
		       NULL, 0, ad, 1, &ctx->enc.epib[1], false))
		goto badkey;

	cmdl_len = sa_format_cmdl_gen(&cfg,
				      (u8 *)ctx->enc.cmdl,
				      &ctx->enc.cmdl_upd_info);
	if (cmdl_len <= 0 || (cmdl_len > SA_MAX_CMDL_WORDS * sizeof(u32)))
		goto badkey;

	ctx->enc.cmdl_size = cmdl_len;

	/* Setup Decryption Security Context & Command label template */
	if (sa_init_sc(&ctx->dec, key, keylen,
		       NULL, 0, ad, 0, &ctx->dec.epib[1], false))
		goto badkey;

	cfg.enc1st = 0;
	cfg.enc_eng_id = ad->enc_eng.eng_id;
	cfg.auth_eng_id = SA_ENG_ID_NONE;
	cfg.auth_subkey_len = 0;
	cmdl_len = sa_format_cmdl_gen(&cfg, (u8 *)ctx->dec.cmdl,
				      &ctx->dec.cmdl_upd_info);

	if (cmdl_len <= 0 || (cmdl_len > SA_MAX_CMDL_WORDS * sizeof(u32)))
		goto badkey;

	ctx->dec.cmdl_size = cmdl_len;

	kfree(ad);

	return 0;

badkey:
	dev_err(sa_k3_dev, "%s: badkey\n", __func__);
	return -EINVAL;
}

static int sa_aes_cbc_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
			     unsigned int keylen)
{
	struct algo_data *ad = kzalloc(sizeof(*ad), GFP_KERNEL);
	/* Convert the key size (16/24/32) to the key size index (0/1/2) */
	int key_idx = (keylen >> 3) - 2;

	ad->enc_eng.eng_id = SA_ENG_ID_EM1;
	ad->enc_eng.sc_size = SA_CTX_ENC_TYPE1_SZ;
	ad->auth_eng.eng_id = SA_ENG_ID_NONE;
	ad->auth_eng.sc_size = 0;
	ad->mci_enc = mci_cbc_enc_array[key_idx];
	ad->mci_dec = mci_cbc_dec_array[key_idx];
	ad->inv_key = true;
	ad->ealg_id = SA_EALG_ID_AES_CBC;
	ad->aalg_id = SA_AALG_ID_NONE;

	return sa_aes_setkey(tfm, key, keylen, ad);
}

static int sa_aes_ecb_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
			     unsigned int keylen)
{
	struct algo_data *ad = kzalloc(sizeof(*ad), GFP_KERNEL);
	/* Convert the key size (16/24/32) to the key size index (0/1/2) */
	int key_idx = (keylen >> 3) - 2;

	ad->enc_eng.eng_id = SA_ENG_ID_EM1;
	ad->enc_eng.sc_size = SA_CTX_ENC_TYPE1_SZ;
	ad->auth_eng.eng_id = SA_ENG_ID_NONE;
	ad->auth_eng.sc_size = 0;
	ad->mci_enc = mci_ecb_enc_array[key_idx];
	ad->mci_dec = mci_ecb_dec_array[key_idx];
	ad->inv_key = true;
	ad->ealg_id = SA_EALG_ID_AES_ECB;
	ad->aalg_id = SA_AALG_ID_NONE;

	return sa_aes_setkey(tfm, key, keylen, ad);
}

static int sa_3des_cbc_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
			      unsigned int keylen)
{
	struct algo_data *ad = kzalloc(sizeof(*ad), GFP_KERNEL);

	ad->enc_eng.eng_id = SA_ENG_ID_EM1;
	ad->enc_eng.sc_size = SA_CTX_ENC_TYPE1_SZ;
	ad->auth_eng.eng_id = SA_ENG_ID_NONE;
	ad->auth_eng.sc_size = 0;
	ad->mci_enc = mci_cbc_3des_enc_array;
	ad->mci_dec = mci_cbc_3des_dec_array;
	ad->ealg_id = SA_EALG_ID_3DES_CBC;
	ad->aalg_id = SA_AALG_ID_NONE;

	return sa_aes_setkey(tfm, key, keylen, ad);
}

static int sa_3des_ecb_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
			      unsigned int keylen)
{
	struct algo_data *ad = kzalloc(sizeof(*ad), GFP_KERNEL);

	ad->enc_eng.eng_id = SA_ENG_ID_EM1;
	ad->enc_eng.sc_size = SA_CTX_ENC_TYPE1_SZ;
	ad->auth_eng.eng_id = SA_ENG_ID_NONE;
	ad->auth_eng.sc_size = 0;
	ad->mci_enc = mci_ecb_3des_enc_array;
	ad->mci_dec = mci_ecb_3des_dec_array;
	ad->aalg_id = SA_AALG_ID_NONE;

	return sa_aes_setkey(tfm, key, keylen, ad);
}

static void sa_aes_dma_in_callback(void *data)
{
	struct sa_rx_data *rxd = (struct sa_rx_data *)data;
	struct ablkcipher_request *req = (struct ablkcipher_request *)rxd->req;

	int sglen = sg_nents_for_len(req->dst, req->nbytes);

	kfree(rxd);

	dma_unmap_sg(sa_k3_dev, req->src, sglen, DMA_TO_DEVICE);
	if (req->src != req->dst)
		dma_unmap_sg(sa_k3_dev, req->dst, sglen, DMA_FROM_DEVICE);

	ablkcipher_request_complete(req, 0);
}

static void sa_aead_dma_in_callback(void *data)
{
	struct sa_rx_data *rxd = (struct sa_rx_data *)data;
	struct aead_request *req = (struct aead_request *)rxd->req;
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	u32 *mdptr;
	unsigned int start = req->assoclen + req->cryptlen;
	unsigned int authsize = crypto_aead_authsize(tfm);
	u8 auth_tag[SA_MAX_AUTH_TAG_SZ];
	int i, sglen, err = 0;
	size_t pl, ml;

	mdptr = (u32 *)dmaengine_desc_get_metadata_ptr(rxd->tx_in, &pl, &ml);
	for (i = 0; i < (authsize / 4); i++)
		mdptr[i + 4] = htonl(mdptr[i + 4]);

	if (rxd->enc) {
		scatterwalk_map_and_copy((void *)&mdptr[4], req->dst,
					 start, crypto_aead_authsize(tfm), 1);
	} else {
		start -= authsize;
		scatterwalk_map_and_copy(auth_tag, req->src,
					 start, crypto_aead_authsize(tfm), 0);

		err = memcmp((void *)&mdptr[4],
			     auth_tag, authsize) ? -EBADMSG : 0;
	}

	kfree(rxd);

	sglen = sg_nents_for_len(req->dst, req->cryptlen + authsize);
	dma_unmap_sg(sa_k3_dev, req->dst, sglen, DMA_FROM_DEVICE);

	sglen =  sg_nents_for_len(req->src, req->assoclen + req->cryptlen);
	dma_unmap_sg(sa_k3_dev, req->src, sglen, DMA_TO_DEVICE);

	aead_request_complete(req, err);
}

static void
sa_prepare_tx_desc(u32 *mdptr, u32 pslen, u32 *psdata, u32 epiblen, u32 *epib)
{
	u32 *out, *in;
	int i;

	for (out = mdptr, in = epib, i = 0; i < epiblen / sizeof(u32); i++)
		*out++ = *in++;

	mdptr[4] = (0xFFFF << 16);
	for (out = &mdptr[5], in = psdata, i = 0;
	     i < pslen / sizeof(u32); i++)
		*out++ = *in++;
}

static int sa_aes_run(struct ablkcipher_request *req, u8 *iv, int enc)
{
	struct sa_tfm_ctx *ctx =
	    crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	struct sa_ctx_info *sa_ctx = enc ? &ctx->enc : &ctx->dec;
	struct sa_crypto_data *pdata = dev_get_drvdata(sa_k3_dev);
	struct sa_dma_req_ctx req_ctx;
	struct dma_async_tx_descriptor *tx_in, *tx_out;
	struct sa_rx_data *rxd;
	u8 enc_offset;
	int sg_nents, dst_nents;
	int psdata_offset;
	u8 auth_offset = 0;
	u8 *auth_iv = NULL;
	u8 *aad = NULL;
	u8 aad_len = 0;
	u16 enc_len;
	u16 auth_len = 0;
	u32 req_type;
	u32 *mdptr;
	size_t pl, ml;

	struct dma_chan *dma_rx;
	gfp_t flags;

	flags = req->base.flags & CRYPTO_TFM_REQ_MAY_SLEEP ?
		GFP_KERNEL : GFP_ATOMIC;

	enc_offset = 0x0;
	enc_len = req->nbytes;

	/* Allocate descriptor & submit packet */
	sg_nents = sg_nents_for_len(req->src, enc_len);
	dst_nents = sg_nents_for_len(req->dst, enc_len);

	memcpy(req_ctx.cmdl, sa_ctx->cmdl, sa_ctx->cmdl_size);

	/* Update Command Label */
	sa_update_cmdl(sa_k3_dev, enc_offset, enc_len,
		       iv, auth_offset, auth_len,
		       auth_iv, aad_len, aad,
		       &sa_ctx->cmdl_upd_info, req_ctx.cmdl);

	/*
	 * Last 2 words in PSDATA will have the crypto alg type &
	 * crypto request pointer
	 */
	req_type = CRYPTO_ALG_TYPE_ABLKCIPHER;
	if (enc)
		req_type |= (SA_REQ_SUBTYPE_ENC << SA_REQ_SUBTYPE_SHIFT);
	else
		req_type |= (SA_REQ_SUBTYPE_DEC << SA_REQ_SUBTYPE_SHIFT);

	psdata_offset = sa_ctx->cmdl_size / sizeof(u32);
	req_ctx.cmdl[psdata_offset++] = req_type;

	/* map the packet */
	req_ctx.src = req->src;
	req_ctx.src_nents = dma_map_sg(sa_k3_dev, req_ctx.src,
				       sg_nents, DMA_TO_DEVICE);
	if (req->src != req->dst)
		dst_nents = dma_map_sg(sa_k3_dev, req->dst,
				       sg_nents, DMA_FROM_DEVICE);
	else
		dst_nents = req_ctx.src_nents;

	if (unlikely(req_ctx.src_nents != sg_nents)) {
		dev_warn_ratelimited(sa_k3_dev, "failed to map tx pkt\n");
		return -EIO;
	}

	req_ctx.dev_data = pdata;
	req_ctx.pkt = true;

	dma_sync_sg_for_device(pdata->dev, req->src, req_ctx.src_nents,
			       DMA_TO_DEVICE);

	if (enc_len >= 256)
		dma_rx = pdata->dma_rx2;
	else
		dma_rx = pdata->dma_rx1;

	tx_in = dmaengine_prep_slave_sg(dma_rx, req->dst, dst_nents,
					DMA_DEV_TO_MEM,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx_in) {
		dev_err(pdata->dev, "IN prep_slave_sg() failed\n");
		return -EINVAL;
	}

	rxd = kzalloc(sizeof(*rxd), GFP_KERNEL);
	rxd->req = (void *)req;

	/* IN */
	tx_in->callback = sa_aes_dma_in_callback;
	tx_in->callback_param = rxd;

	tx_out = dmaengine_prep_slave_sg(pdata->dma_tx, req->src,
					 req_ctx.src_nents, DMA_MEM_TO_DEV,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx_out) {
		dev_err(pdata->dev, "OUT prep_slave_sg() failed\n");
		return -EINVAL;
	}

	mdptr = (u32 *)dmaengine_desc_get_metadata_ptr(tx_out, &pl, &ml);

	sa_prepare_tx_desc(mdptr, (sa_ctx->cmdl_size + (SA_PSDATA_CTX_WORDS *
			   sizeof(u32))), req_ctx.cmdl,
			   sizeof(sa_ctx->epib), sa_ctx->epib);

	ml = sa_ctx->cmdl_size + (SA_PSDATA_CTX_WORDS * sizeof(u32));
	dmaengine_desc_set_metadata_len(tx_out, 44);

	dmaengine_submit(tx_out);
	dmaengine_submit(tx_in);

	dma_async_issue_pending(dma_rx);
	dma_async_issue_pending(pdata->dma_tx);

	return -EINPROGRESS;
}

static int sa_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	return sa_aes_run(req, req->info, 1);
}

static int sa_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	return sa_aes_run(req, req->info, 0);
}

static int sa_init_tfm(struct crypto_tfm *tfm)
{
	struct crypto_alg *alg = tfm->__crt_alg;
	struct sa_tfm_ctx *ctx = crypto_tfm_ctx(tfm);
	struct sa_crypto_data *data = dev_get_drvdata(sa_k3_dev);
	int ret;

	if ((alg->cra_flags & CRYPTO_ALG_TYPE_MASK) == CRYPTO_ALG_TYPE_AEAD) {
		memset(ctx, 0, sizeof(*ctx));
		ctx->dev_data = data;

		ret = sa_init_ctx_info(&ctx->enc, data);
		if (ret)
			return ret;
		ret = sa_init_ctx_info(&ctx->dec, data);
		if (ret) {
			sa_free_ctx_info(&ctx->enc, data);
			return ret;
		}
	}

	dev_dbg(sa_k3_dev, "%s(0x%p) sc-ids(0x%x(0x%pad), 0x%x(0x%pad))\n",
		__func__, tfm, ctx->enc.sc_id, &ctx->enc.sc_phys,
		ctx->dec.sc_id, &ctx->dec.sc_phys);
	return 0;
}

/* Algorithm init */
static int sa_cra_init_aead(struct crypto_aead *tfm)
{
	return sa_init_tfm(crypto_aead_tfm(tfm));
}

/* Algorithm context teardown */
static void sa_exit_tfm(struct crypto_tfm *tfm)
{
	struct crypto_alg *alg = tfm->__crt_alg;
	struct sa_tfm_ctx *ctx = crypto_tfm_ctx(tfm);
	struct sa_crypto_data *data = dev_get_drvdata(sa_k3_dev);

	dev_dbg(sa_k3_dev, "%s(0x%p) sc-ids(0x%x(0x%pad), 0x%x(0x%pad))\n",
		__func__, tfm, ctx->enc.sc_id, &ctx->enc.sc_phys,
		ctx->dec.sc_id, &ctx->dec.sc_phys);

	if ((alg->cra_flags & CRYPTO_ALG_TYPE_MASK)
		== CRYPTO_ALG_TYPE_AEAD) {
		sa_free_ctx_info(&ctx->enc, data);
		sa_free_ctx_info(&ctx->dec, data);
	}
}

static void sa_exit_tfm_aead(struct crypto_aead *tfm)
{
	return sa_exit_tfm(crypto_aead_tfm(tfm));
}

/* AEAD algorithm configuration interface function */
static int sa_aead_setkey(struct crypto_aead *authenc,
			  const u8 *key, unsigned int keylen,
			  struct algo_data *ad)
{
	struct sa_tfm_ctx *ctx = crypto_aead_ctx(authenc);
	struct crypto_authenc_keys keys;

	const char *cra_name;
	int cmdl_len;
	struct sa_cmdl_cfg cfg;

	if (crypto_authenc_extractkeys(&keys, key, keylen) != 0)
		goto badkey;

	cra_name = crypto_tfm_alg_name(crypto_aead_tfm(authenc));

	memset(&cfg, 0, sizeof(cfg));
	cfg.enc1st = 1;
	cfg.aalg = ad->aalg_id;
	cfg.enc_eng_id = ad->enc_eng.eng_id;
	cfg.auth_eng_id = ad->auth_eng.eng_id;
	cfg.iv_size = crypto_aead_ivsize(authenc);
	cfg.akey = keys.authkey;
	cfg.akey_len = keys.authkeylen;

	/* Setup Encryption Security Context & Command label template */
	if (sa_init_sc(&ctx->enc, keys.enckey, keys.enckeylen,
		       keys.authkey, keys.authkeylen,
		       ad, 1, &ctx->enc.epib[1], true))
		goto badkey;

	cmdl_len = sa_format_cmdl_gen(&cfg,
				      (u8 *)ctx->enc.cmdl,
				      &ctx->enc.cmdl_upd_info);
	if (cmdl_len <= 0 || (cmdl_len > SA_MAX_CMDL_WORDS * sizeof(u32)))
		goto badkey;

	ctx->enc.cmdl_size = cmdl_len;

	/* Setup Decryption Security Context & Command label template */
	if (sa_init_sc(&ctx->dec, keys.enckey, keys.enckeylen,
		       keys.authkey, keys.authkeylen,
		       ad, 0, &ctx->dec.epib[1], true))
		goto badkey;

	cfg.enc1st = 0;
	cfg.enc_eng_id = ad->enc_eng.eng_id;
	cmdl_len = sa_format_cmdl_gen(&cfg, (u8 *)ctx->dec.cmdl,
				      &ctx->dec.cmdl_upd_info);

	if (cmdl_len <= 0 || (cmdl_len > SA_MAX_CMDL_WORDS * sizeof(u32)))
		goto badkey;

	ctx->dec.cmdl_size = cmdl_len;

	kfree(ad);

	return 0;

badkey:
	dev_err(sa_k3_dev, "%s: badkey\n", __func__);
	return -EINVAL;
}

static int sa_aead_cbc_sha1_setkey(struct crypto_aead *authenc,
				   const u8 *key, unsigned int keylen)
{
	struct algo_data *ad = kzalloc(sizeof(*ad), GFP_KERNEL);
	struct crypto_authenc_keys keys;
	int ret = 0, key_idx;

	ret = crypto_authenc_extractkeys(&keys, key, keylen);
	if (ret)
		return ret;

	/* Convert the key size (16/24/32) to the key size index (0/1/2) */
	key_idx = (keys.enckeylen >> 3) - 2;
	ad->enc_eng.eng_id = SA_ENG_ID_EM1;
	ad->enc_eng.sc_size = SA_CTX_ENC_TYPE1_SZ;
	ad->auth_eng.eng_id = SA_ENG_ID_AM1;
	ad->auth_eng.sc_size = SA_CTX_AUTH_TYPE2_SZ;
	ad->mci_enc = mci_cbc_enc_array[key_idx];
	ad->mci_dec = mci_cbc_dec_array[key_idx];
	ad->inv_key = true;
	ad->keyed_mac = true;
	ad->ealg_id = SA_EALG_ID_AES_CBC;
	ad->aalg_id = SA_AALG_ID_HMAC_SHA1;
	ad->hash_size = SHA1_DIGEST_SIZE;
	ad->auth_ctrl = 0x2;
	ad->prep_iopad = sa_hmac_sha1_get_pad;

	return sa_aead_setkey(authenc, key, keylen, ad);
}

static int sa_aead_cbc_sha256_setkey(struct crypto_aead *authenc,
				     const u8 *key, unsigned int keylen)
{
	struct algo_data *ad = kzalloc(sizeof(*ad), GFP_KERNEL);
	struct crypto_authenc_keys keys;
	int ret = 0, key_idx;

	ret = crypto_authenc_extractkeys(&keys, key, keylen);
	if (ret)
		return ret;

	/* Convert the key size (16/24/32) to the key size index (0/1/2) */
	key_idx = (keys.enckeylen >> 3) - 2;

	ad->enc_eng.eng_id = SA_ENG_ID_EM1;
	ad->enc_eng.sc_size = SA_CTX_ENC_TYPE1_SZ;
	ad->auth_eng.eng_id = SA_ENG_ID_AM1;
	ad->auth_eng.sc_size = SA_CTX_AUTH_TYPE2_SZ;
	ad->mci_enc = mci_cbc_enc_array[key_idx];
	ad->mci_dec = mci_cbc_dec_array[key_idx];
	ad->inv_key = true;
	ad->keyed_mac = true;
	ad->ealg_id = SA_EALG_ID_AES_CBC;
	ad->aalg_id = SA_AALG_ID_HMAC_SHA2_256;
	ad->hash_size = SHA256_DIGEST_SIZE;
	ad->auth_ctrl = 0x4;
	ad->prep_iopad = sa_hmac_sha256_get_pad;

	return sa_aead_setkey(authenc, key, keylen, ad);
}

static int sa_aead_run(struct aead_request *req, u8 *iv, int enc)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct sa_tfm_ctx *ctx = crypto_aead_ctx(tfm);
	struct sa_ctx_info *sa_ctx = enc ? &ctx->enc : &ctx->dec;
	struct sa_rx_data *rxd;
	struct dma_async_tx_descriptor *tx_in, *tx_out;
	struct sa_crypto_data *pdata = dev_get_drvdata(sa_k3_dev);
	struct sa_dma_req_ctx req_ctx;
	u8 enc_offset;
	int sg_nents, dst_nents;
	int psdata_offset;
	u8 auth_offset = 0;
	u8 *auth_iv = NULL;
	u8 *aad = NULL;
	u8 aad_len = 0;
	u16 enc_len;
	u16 auth_len = 0;
	u32 *mdptr;
	u32 req_type;
	struct dma_chan *dma_rx;
	gfp_t flags;
	size_t pl, ml;

	flags = req->base.flags & CRYPTO_TFM_REQ_MAY_SLEEP ?
			GFP_KERNEL : GFP_ATOMIC;

	if (enc) {
		iv = (u8 *)(req->iv);
		enc_offset = req->assoclen;
		enc_len = req->cryptlen;
		auth_len = req->assoclen + req->cryptlen;
	} else {
		enc_offset = req->assoclen;
		enc_len = req->cryptlen - crypto_aead_authsize(tfm);
		auth_len = req->assoclen + req->cryptlen -
			crypto_aead_authsize(tfm);
	}

	/* Allocate descriptor & submit packet */
	sg_nents = sg_nents_for_len(req->src, enc_len + req->assoclen);
	dst_nents = sg_nents_for_len(req->dst, enc_len +
				     crypto_aead_authsize(tfm));

	memcpy(req_ctx.cmdl, sa_ctx->cmdl, sa_ctx->cmdl_size);
	/* Update Command Label */
	sa_update_cmdl(sa_k3_dev, enc_offset, enc_len,
		       iv, auth_offset, auth_len,
		       auth_iv, aad_len, aad,
		       &sa_ctx->cmdl_upd_info, req_ctx.cmdl);

	/*
	 * Last 2 words in PSDATA will have the crypto alg type &
	 * crypto request pointer
	 */
	req_type = CRYPTO_ALG_TYPE_AEAD;
	if (enc)
		req_type |= (SA_REQ_SUBTYPE_ENC << SA_REQ_SUBTYPE_SHIFT);
	else
		req_type |= (SA_REQ_SUBTYPE_DEC << SA_REQ_SUBTYPE_SHIFT);

	psdata_offset = sa_ctx->cmdl_size / sizeof(u32);

	/* map the packet */
	req_ctx.src = req->src;
	req_ctx.src_nents = dma_map_sg(sa_k3_dev, req_ctx.src,
				       sg_nents, DMA_TO_DEVICE);
	dst_nents = dma_map_sg(sa_k3_dev, req->dst,
			       dst_nents, DMA_FROM_DEVICE);

	if (unlikely(req_ctx.src_nents != sg_nents)) {
		dev_warn_ratelimited(sa_k3_dev, "failed to map tx pkt\n");
		return -EIO;
	}

	req_ctx.dev_data = pdata;
	req_ctx.pkt = true;

	dma_sync_sg_for_device(pdata->dev, req->src, req_ctx.src_nents,
			       DMA_TO_DEVICE);

	if (enc_len >= 256)
		dma_rx = pdata->dma_rx2;
	else
		dma_rx = pdata->dma_rx1;

	tx_in = dmaengine_prep_slave_sg(dma_rx, req->dst, dst_nents,
					DMA_DEV_TO_MEM,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx_in) {
		dev_err(pdata->dev, "IN prep_slave_sg() failed\n");
		return -EINVAL;
	}

	rxd = kzalloc(sizeof(*rxd), GFP_KERNEL);
	rxd->req = (void *)req;
	rxd->enc = enc;
	rxd->tx_in = tx_in;

	/* IN */
	tx_in->callback = sa_aead_dma_in_callback;
	tx_in->callback_param = rxd;

	tx_out = dmaengine_prep_slave_sg(pdata->dma_tx, req->src,
					 req_ctx.src_nents, DMA_MEM_TO_DEV,
					 DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx_out) {
		dev_err(pdata->dev, "OUT prep_slave_sg() failed\n");
		return -EINVAL;
	}

	mdptr = (u32 *)dmaengine_desc_get_metadata_ptr(tx_out, &pl, &ml);

	sa_prepare_tx_desc(mdptr, (sa_ctx->cmdl_size + (SA_PSDATA_CTX_WORDS *
			   sizeof(u32))), req_ctx.cmdl,
			   sizeof(sa_ctx->epib), sa_ctx->epib);

	ml = sa_ctx->cmdl_size + (SA_PSDATA_CTX_WORDS * sizeof(u32));
	dmaengine_desc_set_metadata_len(tx_out, 52);

	dmaengine_submit(tx_out);
	dmaengine_submit(tx_in);

	dma_async_issue_pending(dma_rx);
	dma_async_issue_pending(pdata->dma_tx);
	return -EINPROGRESS;
}

/* AEAD algorithm encrypt interface function */
static int sa_aead_encrypt(struct aead_request *req)
{
	return sa_aead_run(req, req->iv, 1);
}

/* AEAD algorithm decrypt interface function */
static int sa_aead_decrypt(struct aead_request *req)
{
	return sa_aead_run(req, req->iv, 0);
}

static int sa_sham_cra_init_alg(struct crypto_tfm *tfm, const char *alg_base)
{
	struct sa_tfm_ctx *ctx = crypto_tfm_ctx(tfm);
	struct crypto_alg *alg = tfm->__crt_alg;
	struct sa_crypto_data *data = dev_get_drvdata(sa_k3_dev);
	int ret;

	if ((alg->cra_flags & CRYPTO_ALG_TYPE_MASK) ==
	    CRYPTO_ALG_TYPE_AHASH) {
		memset(ctx, 0, sizeof(*ctx));
		ctx->dev_data = data;
		ret = sa_init_ctx_info(&ctx->enc, data);
		if (ret)
			return ret;
	}

	if (alg_base) {
		ctx->shash = crypto_alloc_shash(alg_base, 0,
						CRYPTO_ALG_NEED_FALLBACK);
		if (IS_ERR(ctx->shash)) {
			pr_err("base driver %s couldn't be loaded\n", alg_base);
			return PTR_ERR(ctx->shash);
		}
	}

	dev_dbg(sa_k3_dev, "%s(0x%p) sc-ids(0x%x(0x%pad), 0x%x(0x%pad))\n",
		__func__, tfm, ctx->enc.sc_id, &ctx->enc.sc_phys,
		ctx->dec.sc_id, &ctx->dec.sc_phys);

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct sa_dma_req_ctx) +
				 SHA512_BLOCK_SIZE);

	return 0;
}

static void sa_sham_dma_in_callback(void *data)
{
	struct sa_rx_data *rxd = (struct sa_rx_data *)data;
	struct ahash_request *req = (struct ahash_request *)rxd->req;
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	unsigned int authsize = crypto_ahash_digestsize(tfm);
	int i;
	size_t ml, pl;
	u32 *mdptr, *result;

	mdptr = (u32 *)dmaengine_desc_get_metadata_ptr(rxd->tx_in, &pl, &ml);
	result = (u32 *)req->result;

	kfree(rxd);

	for (i = 0; i < (authsize / 4); i++)
		result[i] = htonl(mdptr[i + 4]);

	ahash_request_complete(req, 0);
}

static int sa_sham_digest(struct ahash_request *req)
{
	struct sa_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct sa_ctx_info *sa_ctx = &ctx->enc;
	struct dma_async_tx_descriptor *tx_in, *tx_out;
	struct sa_crypto_data *pdata = dev_get_drvdata(sa_k3_dev);
	struct sa_dma_req_ctx req_ctx;
	struct sa_rx_data *rxd;
	u8 enc_offset;
	int sg_nents;
	int psdata_offset;
	u8 auth_offset = 0;
	u8 *auth_iv = NULL;
	u8 *aad = NULL;
	u8 aad_len = 0;
	u16 enc_len;
	u16 auth_len = 0;
	u32 req_type;
	u32 *mdptr;
	struct dma_chan *dma_rx;
	gfp_t flags;
	size_t pl, ml;

	flags = req->base.flags & CRYPTO_TFM_REQ_MAY_SLEEP ?
			GFP_KERNEL : GFP_ATOMIC;
	enc_len = 0;
	auth_len = req->nbytes;
	enc_offset = 0;

	/* Allocate descriptor & submit packet */
	sg_nents = sg_nents_for_len(req->src, req->nbytes);

	memcpy(req_ctx.cmdl, sa_ctx->cmdl, sa_ctx->cmdl_size);
	/* Update Command Label */
	sa_update_cmdl(sa_k3_dev, enc_offset, enc_len,
		       NULL, auth_offset, auth_len,
		       auth_iv, aad_len, aad,
		       &sa_ctx->cmdl_upd_info, req_ctx.cmdl);

	/*
	 * Last 2 words in PSDATA will have the crypto alg type &
	 * crypto request pointer
	 */
	req_type = CRYPTO_ALG_TYPE_AHASH;

	psdata_offset = sa_ctx->cmdl_size / sizeof(u32);
	req_ctx.cmdl[psdata_offset++] = req_type;

	/* map the packet */
	req_ctx.src = req->src;
	req_ctx.src_nents = dma_map_sg(sa_k3_dev, req_ctx.src,
				       sg_nents, DMA_TO_DEVICE);

	if (unlikely(req_ctx.src_nents != sg_nents)) {
		dev_warn_ratelimited(sa_k3_dev, "failed to map tx pkt\n");
		return -EIO;
	}

	req_ctx.dev_data = pdata;
	req_ctx.pkt = true;

	dma_sync_sg_for_device(pdata->dev, req->src, req_ctx.src_nents,
			       DMA_TO_DEVICE);

	if (enc_len > 256)
		dma_rx = pdata->dma_rx2;
	else
		dma_rx = pdata->dma_rx1;

	tx_in = dmaengine_prep_slave_sg(dma_rx, req->src, req_ctx.src_nents,
					DMA_DEV_TO_MEM,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx_in) {
		dev_err(pdata->dev, "IN prep_slave_sg() failed\n");
		return -EINVAL;
	}

	rxd = kzalloc(sizeof(*rxd), GFP_KERNEL);
	rxd->req = (void *)req;
	rxd->tx_in = tx_in;
	tx_in->callback = sa_sham_dma_in_callback;
	tx_in->callback_param = rxd;

	tx_out = dmaengine_prep_slave_sg(pdata->dma_tx, req->src,
					 req_ctx.src_nents, DMA_MEM_TO_DEV,
					 DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx_out) {
		dev_err(pdata->dev, "OUT prep_slave_sg() failed\n");
		return -EINVAL;
	}

	mdptr = (u32 *)dmaengine_desc_get_metadata_ptr(tx_out, &pl, &ml);
	sa_prepare_tx_desc(mdptr, (sa_ctx->cmdl_size + (SA_PSDATA_CTX_WORDS *
			   sizeof(u32))), req_ctx.cmdl,
			   sizeof(sa_ctx->epib), sa_ctx->epib);

	dmaengine_desc_set_metadata_len(tx_out, 28);

	dmaengine_submit(tx_out);
	dmaengine_submit(tx_in);

	dma_async_issue_pending(dma_rx);
	dma_async_issue_pending(pdata->dma_tx);

	return -EINPROGRESS;
}

static int sa_sham_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);

	dev_dbg(sa_k3_dev, "init: digest size: %d\n",
		crypto_ahash_digestsize(tfm));

	return 0;
}

static int sa_sham_shash_digest(struct crypto_shash *tfm, u32 flags,
				const u8 *data, unsigned int len, u8 *out)
{
	SHASH_DESC_ON_STACK(shash, tfm);

	shash->tfm = tfm;
	shash->flags = flags & CRYPTO_TFM_REQ_MAY_SLEEP;

	return crypto_shash_digest(shash, data, len, out);
}

static int sa_sham_setkey(struct crypto_ahash *tfm, const u8 *key,
			  unsigned int keylen, struct  algo_data *ad)
{
	struct sa_tfm_ctx *ctx = crypto_ahash_ctx(tfm);
	int bs = crypto_shash_blocksize(ctx->shash);
	int ds = crypto_shash_digestsize(ctx->shash);
	int cmdl_len;
	struct sa_cmdl_cfg cfg;
	int err;

	if (keylen > bs) {
		err = sa_sham_shash_digest(ctx->shash,
					   crypto_shash_get_flags(ctx->shash),
					   key, keylen, ctx->authkey);
		if (err)
			return err;
		keylen = ds;
	} else {
		memcpy(ctx->authkey, key, keylen);
	}

	memset(ctx->authkey + keylen, 0, bs - keylen);
	memset(&cfg, 0, sizeof(cfg));
	cfg.enc1st = 0;
	cfg.aalg = ad->aalg_id;
	cfg.enc_eng_id = ad->enc_eng.eng_id;
	cfg.auth_eng_id = ad->auth_eng.eng_id;
	cfg.iv_size = 0;
	cfg.akey = ctx->authkey;
	cfg.akey_len = keylen;

	/* Setup Encryption Security Context & Command label template */
	if (sa_init_sc(&ctx->enc, NULL, 0, ctx->authkey, keylen, ad, 0,
		       &ctx->enc.epib[1], true))
		goto badkey;

	cmdl_len = sa_format_cmdl_gen(&cfg,
				      (u8 *)ctx->enc.cmdl,
				      &ctx->enc.cmdl_upd_info);
	if (cmdl_len <= 0 || (cmdl_len > SA_MAX_CMDL_WORDS * sizeof(u32)))
		goto badkey;

	ctx->enc.cmdl_size = cmdl_len;

	kfree(ad);

	return 0;
badkey:
	dev_err(sa_k3_dev, "%s: badkey\n", __func__);
	return -EINVAL;
}

static int sa_sham_sha1_setkey(struct crypto_ahash *tfm, const u8 *key,
			       unsigned int keylen)
{
	struct algo_data *ad = kzalloc(sizeof(*ad), GFP_KERNEL);

	ad->enc_eng.eng_id = SA_ENG_ID_NONE;
	ad->enc_eng.sc_size = SA_CTX_ENC_TYPE1_SZ;
	ad->auth_eng.eng_id = SA_ENG_ID_AM1;
	ad->auth_eng.sc_size = SA_CTX_AUTH_TYPE2_SZ;
	ad->mci_enc = NULL;
	ad->mci_dec = NULL;
	ad->inv_key = false;
	ad->keyed_mac = true;
	ad->ealg_id = SA_EALG_ID_NONE;
	ad->aalg_id = SA_AALG_ID_HMAC_SHA1;
	ad->hash_size = SHA1_DIGEST_SIZE;
	ad->auth_ctrl = 0x2;
	ad->prep_iopad = sa_hmac_sha1_get_pad;

	return sa_sham_setkey(tfm, key, keylen, ad);
}

static int sa_sham_sha256_setkey(struct crypto_ahash *tfm, const u8 *key,
				 unsigned int keylen)
{
	struct algo_data *ad = kzalloc(sizeof(*ad), GFP_KERNEL);

	ad->enc_eng.eng_id = SA_ENG_ID_NONE;
	ad->enc_eng.sc_size = SA_CTX_ENC_TYPE1_SZ;
	ad->auth_eng.eng_id = SA_ENG_ID_AM1;
	ad->auth_eng.sc_size = SA_CTX_AUTH_TYPE2_SZ;
	ad->mci_enc = NULL;
	ad->mci_dec = NULL;
	ad->inv_key = false;
	ad->keyed_mac = true;
	ad->ealg_id = SA_EALG_ID_NONE;
	ad->aalg_id = SA_AALG_ID_HMAC_SHA2_256;
	ad->hash_size = SHA256_DIGEST_SIZE;
	ad->auth_ctrl = 0x4;
	ad->prep_iopad = sa_hmac_sha256_get_pad;

	return sa_sham_setkey(tfm, key, keylen, ad);
}

static int sa_sham_cra_sha1_init(struct crypto_tfm *tfm)
{
	return sa_sham_cra_init_alg(tfm, "sha1");
}

static int sa_sham_cra_sha256_init(struct crypto_tfm *tfm)
{
	return sa_sham_cra_init_alg(tfm, "sha256");
}

static void sa_sham_cra_exit(struct crypto_tfm *tfm)
{
	struct crypto_alg *alg = tfm->__crt_alg;
	struct sa_tfm_ctx *ctx = crypto_tfm_ctx(tfm);
	struct sa_crypto_data *data = dev_get_drvdata(sa_k3_dev);

	dev_dbg(sa_k3_dev, "%s(0x%p) sc-ids(0x%x(0x%pad), 0x%x(0x%pad))\n",
		__func__, tfm, ctx->enc.sc_id, &ctx->enc.sc_phys,
		ctx->dec.sc_id, &ctx->dec.sc_phys);

	if ((alg->cra_flags & CRYPTO_ALG_TYPE_AHASH)
	    == CRYPTO_ALG_TYPE_AHASH) {
		sa_free_ctx_info(&ctx->enc, data);
	}
}

static int sa_sham_update(struct ahash_request *req)
{
	return 0;
}

static int sa_sham_final(struct ahash_request *req)
{
	return sa_sham_digest(req);
}

static int sa_sham_finup(struct ahash_request *req)
{
	return 0;
}

static struct sa_alg_tmpl sa_algs[] = {
	{.type = CRYPTO_ALG_TYPE_ABLKCIPHER,
	 .alg.crypto = {
			.cra_name = "cbc(aes)",
			.cra_driver_name = "cbc-aes-sa2ul",
			.cra_priority = 30000,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
				     CRYPTO_ALG_KERN_DRIVER_ONLY |
				     CRYPTO_ALG_ASYNC |
				     CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct sa_tfm_ctx),
			.cra_alignmask = 0,
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_init = sa_aes_cra_init,
			.cra_exit = sa_aes_cra_exit,
			.cra_u.ablkcipher = {
					     .min_keysize = AES_MIN_KEY_SIZE,
					     .max_keysize = AES_MAX_KEY_SIZE,
					     .ivsize = AES_BLOCK_SIZE,
					     .setkey = sa_aes_cbc_setkey,
					     .encrypt = sa_aes_cbc_encrypt,
					     .decrypt = sa_aes_cbc_decrypt,
					}
			}
	},
	{	.type = CRYPTO_ALG_TYPE_ABLKCIPHER,
		.alg.crypto = {
			.cra_name = "ecb(aes)",
			.cra_driver_name = "ecb-aes-sa2ul",
			.cra_priority = 30000,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_KERN_DRIVER_ONLY |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct sa_tfm_ctx),
			.cra_alignmask = 0,
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_init = sa_aes_cra_init,
			.cra_exit = sa_aes_cra_exit,
			.cra_u.ablkcipher = {
				.min_keysize    = AES_MIN_KEY_SIZE,
				.max_keysize    = AES_MAX_KEY_SIZE,
				.setkey		= sa_aes_ecb_setkey,
				.encrypt	= sa_aes_cbc_encrypt,
				.decrypt	= sa_aes_cbc_decrypt,
			}
		}
	},
	{	.type = CRYPTO_ALG_TYPE_ABLKCIPHER,
		.alg.crypto = {
			.cra_name = "cbc(des3_ede)",
			.cra_driver_name = "cbc-des3-sa2ul",
			.cra_priority = 30000,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_KERN_DRIVER_ONLY |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize = DES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct sa_tfm_ctx),
			.cra_alignmask = 0,
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_init = sa_aes_cra_init,
			.cra_exit = sa_aes_cra_exit,
			.cra_u.ablkcipher = {
				.min_keysize    = 3 * DES_KEY_SIZE,
				.max_keysize    = 3 * DES_KEY_SIZE,
				.ivsize		= DES_BLOCK_SIZE,
				.setkey		= sa_3des_cbc_setkey,
				.encrypt	= sa_aes_cbc_encrypt,
				.decrypt	= sa_aes_cbc_decrypt,
			}
		}
	},
	{	.type = CRYPTO_ALG_TYPE_ABLKCIPHER,
		.alg.crypto = {
			.cra_name = "ecb(des3_ede)",
			.cra_driver_name = "ecb-des3-sa2ul",
			.cra_priority = 30000,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_KERN_DRIVER_ONLY |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize = DES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct sa_tfm_ctx),
			.cra_alignmask = 0,
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_init = sa_aes_cra_init,
			.cra_exit = sa_aes_cra_exit,
			.cra_u.ablkcipher = {
				.min_keysize    = 3 * DES_KEY_SIZE,
				.max_keysize    = 3 * DES_KEY_SIZE,
				.ivsize		= DES_BLOCK_SIZE,
				.setkey		= sa_3des_ecb_setkey,
				.encrypt	= sa_aes_cbc_encrypt,
				.decrypt	= sa_aes_cbc_decrypt,
			}
		}
	},
	/* AEAD algorithms */
	{.type	= CRYPTO_ALG_TYPE_AEAD,
		.alg.aead = {
				.base = {
				.cra_name = "authenc(hmac(sha1),cbc(aes))",
				.cra_driver_name =
					"authenc(hmac(sha1),cbc(aes))-keystone-sa",
				.cra_blocksize = AES_BLOCK_SIZE,
				.cra_flags = CRYPTO_ALG_TYPE_AEAD |
					CRYPTO_ALG_KERN_DRIVER_ONLY |
					CRYPTO_ALG_ASYNC,
				.cra_ctxsize = sizeof(struct sa_tfm_ctx),
				.cra_module = THIS_MODULE,
				.cra_alignmask = 0,
				.cra_priority = 3000,
			},
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = SHA1_DIGEST_SIZE,

			.init = sa_cra_init_aead,
			.exit = sa_exit_tfm_aead,
			.setkey = sa_aead_cbc_sha1_setkey,
			.encrypt = sa_aead_encrypt,
			.decrypt = sa_aead_decrypt,
		}
	},
	{.type	= CRYPTO_ALG_TYPE_AEAD,
		.alg.aead = {
				.base = {
				.cra_name = "authenc(hmac(sha256),cbc(aes))",
				.cra_driver_name =
					"authenc(hmac(sha256),cbc(aes))-keystone-sa",
				.cra_blocksize = AES_BLOCK_SIZE,
				.cra_flags = CRYPTO_ALG_TYPE_AEAD |
					CRYPTO_ALG_KERN_DRIVER_ONLY |
					CRYPTO_ALG_ASYNC,
				.cra_ctxsize = sizeof(struct sa_tfm_ctx),
				.cra_module = THIS_MODULE,
				.cra_alignmask = 0,
				.cra_priority = 3000,
			},
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = SHA256_DIGEST_SIZE,

			.init = sa_cra_init_aead,
			.exit = sa_exit_tfm_aead,
			.setkey = sa_aead_cbc_sha256_setkey,
			.encrypt = sa_aead_encrypt,
			.decrypt = sa_aead_decrypt,
		}
	},
};

static struct ahash_alg algs_sha[] = {
{
	.init		= sa_sham_init,
	.update		= sa_sham_update,
	.final		= sa_sham_final,
	.finup		= sa_sham_finup,
	.digest		= sa_sham_digest,
	.setkey		= sa_sham_sha1_setkey,
	.halg.digestsize	= SHA1_DIGEST_SIZE,
	.halg.statesize		= 128,
	.halg.base	= {
		.cra_name		= "hmac(sha1)",
		.cra_driver_name	= "sa-hmac-sha1",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
						CRYPTO_ALG_ASYNC |
						CRYPTO_ALG_KERN_DRIVER_ONLY |
						CRYPTO_ALG_NEED_FALLBACK,
		.cra_blocksize		= SHA1_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct sa_tfm_ctx),
		.cra_alignmask		= SA_ALIGN_MASK,
		.cra_module		= THIS_MODULE,
		.cra_init		= sa_sham_cra_sha1_init,
		.cra_exit		= sa_sham_cra_exit,
	}
},
{
	.init		= sa_sham_init,
	.update		= sa_sham_update,
	.final		= sa_sham_final,
	.finup		= sa_sham_finup,
	.digest		= sa_sham_digest,
	.setkey		= sa_sham_sha256_setkey,
	.halg.digestsize	= SHA256_DIGEST_SIZE,
	.halg.statesize		= 128,
	.halg.base	= {
		.cra_name		= "hmac(sha256)",
		.cra_driver_name	= "sa-hmac-sha256",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
						CRYPTO_ALG_ASYNC |
						CRYPTO_ALG_KERN_DRIVER_ONLY |
						CRYPTO_ALG_NEED_FALLBACK,
		.cra_blocksize		= SHA256_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct sa_tfm_ctx),
		.cra_alignmask		= SA_ALIGN_MASK,
		.cra_module		= THIS_MODULE,
		.cra_init		= sa_sham_cra_sha256_init,
		.cra_exit		= sa_sham_cra_exit,
	}
},
};

/* Register the algorithms in crypto framework */
void sa_register_algos(const struct device *dev)
{
	char *alg_name;
	u32 type;
	int i, err, num_algs = ARRAY_SIZE(sa_algs);

	for (i = 0; i < num_algs; i++) {
		type = sa_algs[i].type;
		if (type == CRYPTO_ALG_TYPE_AEAD) {
			alg_name = sa_algs[i].alg.aead.base.cra_name;
			err = crypto_register_aead(&sa_algs[i].alg.aead);
		} else if (type == CRYPTO_ALG_TYPE_ABLKCIPHER) {
			alg_name = sa_algs[i].alg.crypto.cra_name;
			err = crypto_register_alg(&sa_algs[i].alg.crypto);
		} else {
			dev_err(dev,
				"un-supported crypto algorithm (%d)",
				sa_algs[i].type);
			continue;
		}

		if (err)
			dev_err(dev, "Failed to register '%s'\n", alg_name);
		else
			sa_algs[i].registered = 1;
	}

	num_algs = ARRAY_SIZE(algs_sha);
	for (i = 0; i < num_algs; i++) {
		alg_name =  algs_sha[i].halg.base.cra_name;
		err = crypto_register_ahash(&algs_sha[i]);
		if (err)
			dev_err(dev, "Failed to register '%s'\n",
				alg_name);
	}
}

/* Unregister the algorithms in crypto framework */
void sa_unregister_algos(const struct device *dev)
{
	char *alg_name;
	u32 type;
	int i, err = 0, num_algs = ARRAY_SIZE(sa_algs);

	for (i = 0; i < num_algs; i++) {
		type = sa_algs[i].type;
		if (type == CRYPTO_ALG_TYPE_AEAD) {
			alg_name = sa_algs[i].alg.aead.base.cra_name;
			crypto_unregister_aead(&sa_algs[i].alg.aead);
		} else {
			alg_name = sa_algs[i].alg.crypto.cra_name;
			err = crypto_unregister_alg(&sa_algs[i].alg.crypto);
		}

		sa_algs[i].registered = 0;
	}

	num_algs = ARRAY_SIZE(algs_sha);
	for (i = 0; i < num_algs; i++) {
		alg_name =  algs_sha[i].halg.base.cra_name;
		err = crypto_unregister_ahash(&algs_sha[i]);
		if (err)
			dev_err(dev, "Failed to register '%s'\n",
				alg_name);
	}
}

static int sa_init_mem(struct sa_crypto_data *dev_data)
{
	struct device *dev = &dev_data->pdev->dev;
	/* Setup dma pool for security context buffers */
	dev_data->sc_pool = dma_pool_create("keystone-sc", dev,
					    SA_CTX_MAX_SZ, 64, 0);
	if (!dev_data->sc_pool) {
		dev_err(dev, "Failed to create dma pool");
		return -ENOMEM;
	}

	return 0;
}

static int sa_dma_init(struct sa_crypto_data *dd)
{
	int ret;
	struct dma_slave_config cfg;

	dd->dma_rx1 = NULL;
	dd->dma_tx = NULL;
	dd->dma_rx2 = NULL;

	ret = dma_coerce_mask_and_coherent(dd->dev, DMA_BIT_MASK(48));
	if (ret)
		return ret;

	dd->dma_rx1 = dma_request_chan(dd->dev, "rx1");
	if (IS_ERR(dd->dma_rx1)) {
		if (PTR_ERR(dd->dma_rx1) != -EPROBE_DEFER)
			dev_err(dd->dev, "Unable to request rx1 DMA channel\n");
		return PTR_ERR(dd->dma_rx1);
	}

	dd->dma_rx2 = dma_request_chan(dd->dev, "rx2");
	if (IS_ERR(dd->dma_rx2)) {
		dma_release_channel(dd->dma_rx1);
		if (PTR_ERR(dd->dma_rx1) != -EPROBE_DEFER)
			dev_err(dd->dev, "Unable to request rx2 DMA channel\n");
		return PTR_ERR(dd->dma_rx2);
	}

	dd->dma_tx = dma_request_chan(dd->dev, "tx");
	if (IS_ERR(dd->dma_tx)) {
		if (PTR_ERR(dd->dma_rx1) != -EPROBE_DEFER)
			dev_err(dd->dev, "Unable to request tx DMA channel\n");
		ret = PTR_ERR(dd->dma_tx);
		goto err_dma_tx;
	}

	memzero_explicit(&cfg, sizeof(cfg));

	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.src_maxburst = 4;
	cfg.dst_maxburst = 4;

	ret = dmaengine_slave_config(dd->dma_rx1, &cfg);
	if (ret) {
		dev_err(dd->dev, "can't configure IN dmaengine slave: %d\n",
			ret);
		return ret;
	}

	ret = dmaengine_slave_config(dd->dma_rx2, &cfg);
	if (ret) {
		dev_err(dd->dev, "can't configure IN dmaengine slave: %d\n",
			ret);
		return ret;
	}

	ret = dmaengine_slave_config(dd->dma_tx, &cfg);
	if (ret) {
		dev_err(dd->dev, "can't configure OUT dmaengine slave: %d\n",
			ret);
		return ret;
	}

	return 0;

err_dma_tx:
	dma_release_channel(dd->dma_rx1);
	dma_release_channel(dd->dma_rx2);

	return ret;
}

static int sa_ul_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	static void __iomem *saul_base;
	struct sa_crypto_data *dev_data;
	u32 val;
	int ret;

	dev_data = devm_kzalloc(dev, sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data)
		return -ENOMEM;

	sa_k3_dev = dev;
	dev_data->dev = dev;
	dev_data->pdev = pdev;
	platform_set_drvdata(pdev, dev_data);
	dev_set_drvdata(sa_k3_dev, dev_data);

	sa_init_mem(dev_data);
	ret = sa_dma_init(dev_data);
	if (ret)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	saul_base = devm_ioremap_resource(dev, res);

	val = SA_EEC_ENCSS_EN | SA_EEC_AUTHSS_EN | SA_EEC_CTXCACH_EN |
	    SA_EEC_CPPI_PORT_IN_EN | SA_EEC_CPPI_PORT_OUT_EN | SA_EEC_TRNG_EN;

	writel_relaxed(val, saul_base + SA_ENGINE_ENABLE_CONTROL);

	sa_register_algos(dev);

	return 0;
}

static int sa_ul_remove(struct platform_device *pdev)
{
	struct sa_crypto_data *dev_data = platform_get_drvdata(pdev);

	sa_unregister_algos(&pdev->dev);

	dma_release_channel(dev_data->dma_rx2);
	dma_release_channel(dev_data->dma_rx1);
	dma_release_channel(dev_data->dma_tx);

	dma_pool_destroy(dev_data->sc_pool);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id of_match[] = {
	{.compatible = "ti,sa2ul-crypto",},
	{},
};

MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver sa_ul_driver = {
	.probe = sa_ul_probe,
	.remove = sa_ul_remove,
	.driver = {
		   .name = "saul-crypto",
		   .of_match_table = of_match,
		   },
};
module_platform_driver(sa_ul_driver);
MODULE_LICENSE("GPL v2");
