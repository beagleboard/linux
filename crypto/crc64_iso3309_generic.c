// SPDX-License-Identifier: GPL-2.0-only

#include <linux/crc64.h>
#include <linux/module.h>
#include <crypto/internal/hash.h>
#include <asm/unaligned.h>

static int chksum_cra_init(struct crypto_tfm *tfm)
{
	u64 *key = crypto_tfm_ctx(tfm);

	*key = 0;
	return 0;
}

static int chksum_init(struct shash_desc *desc)
{
	u64 *key = crypto_shash_ctx(desc->tfm);
	u64 *crc = shash_desc_ctx(desc);

	*crc = *key;
	return 0;
}

static int chksum_update(struct shash_desc *desc, const u8 *data,
			 unsigned int length)
{
	u64 *crc = shash_desc_ctx(desc);

	*crc = crc64_iso3309_generic(*crc, data, length);
	return 0;
}

static int chksum_final(struct shash_desc *desc, u8 *out)
{
	u64 *crc = shash_desc_ctx(desc);

	put_unaligned_le64(*crc, out);
	return 0;
}

static int __chksum_finup(u64 crc, const u8 *data, unsigned int len, u8 *out)
{
	crc = crc64_iso3309_generic(crc, data, len);

	put_unaligned_le64(crc, out);
	return 0;
}

static int chksum_finup(struct shash_desc *desc, const u8 *data,
			unsigned int len, u8 *out)
{
	u64 *crc = shash_desc_ctx(desc);

	return __chksum_finup(*crc, data, len, out);
}

static int chksum_digest(struct shash_desc *desc, const u8 *data,
			 unsigned int length, u8 *out)
{
	u64 *key = crypto_shash_ctx(desc->tfm);

	return __chksum_finup(*key, data, length, out);
}

/*
 * Setting the seed allows arbitrary accumulators and flexible XOR policy
 */
static int chksum_setkey(struct crypto_shash *tfm, const u8 *key,
			 unsigned int keylen)
{
	u64 *mctx = crypto_shash_ctx(tfm);

	if (keylen != sizeof(u64))
		return -EINVAL;

	*mctx = get_unaligned_le64(key);
	return 0;
}

static struct shash_alg alg = {
	.digestsize	=	sizeof(u64),
	.setkey		=	chksum_setkey,
	.init		=	chksum_init,
	.update		=	chksum_update,
	.final		=	chksum_final,
	.finup		=	chksum_finup,
	.digest		=	chksum_digest,
	.descsize	=	sizeof(u64),
	.base		=	{
		.cra_name		=	CRC64_ISO3309_STRING,
		.cra_driver_name	=	"crc64-iso3309-generic",
		.cra_priority		=	200,
		.cra_flags		=       CRYPTO_ALG_OPTIONAL_KEY,
		.cra_blocksize		=	1,
		.cra_ctxsize		=       sizeof(u64),
		.cra_module		=	THIS_MODULE,
		.cra_init		=       chksum_cra_init,
	}
};

static int __init crc64_iso3309_init(void)
{
	return crypto_register_shash(&alg);
}

static void __exit crc64_iso3309_exit(void)
{
	crypto_unregister_shash(&alg);
}

module_init(crc64_iso3309_init);
module_exit(crc64_iso3309_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kamlesh Gurudasani <kamlesh@ti.com>");
MODULE_DESCRIPTION("ISO3309 model CRC64 calculation");
MODULE_ALIAS_CRYPTO("crc64-iso3309");
MODULE_ALIAS_CRYPTO("crc64-iso3309-generic");
