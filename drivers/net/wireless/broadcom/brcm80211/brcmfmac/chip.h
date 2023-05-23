// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2014 Broadcom Corporation
 */
#ifndef BRCMF_CHIP_H
#define BRCMF_CHIP_H

#include <linux/types.h>

#define CORE_CC_REG(base, field) \
		((base) + offsetof(struct chipcregs, field))

#define CORE_GCI_REG(base, field) \
		((base) + offsetof(struct chipgciregs, field))

struct brcmf_blhs;

/**
 * struct brcmf_chip - chip level information.
 *
 * @chip: chip identifier.
 * @chiprev: chip revision.
 * @cc_caps: chipcommon core capabilities.
 * @cc_caps_ext: chipcommon core extended capabilities.
 * @pmucaps: PMU capabilities.
 * @pmurev: PMU revision.
 * @rambase: RAM base address (only applicable for ARM CR4 chips).
 * @ramsize: amount of RAM on chip including retention.
 * @srsize: amount of retention RAM on chip.
 * @name: string representation of the chip identifier.
 * @blhs: bootlooder handshake handle.
 */
struct brcmf_chip {
	u32 chip;
	u32 chiprev;
	u32 cc_caps;
	u32 cc_caps_ext;
	u32 pmucaps;
	u32 pmurev;
	u32 rambase;
	u32 ramsize;
	u32 srsize;
	char name[12];
	struct brcmf_blhs *blhs;
};

/**
 * struct brcmf_core - core related information.
 *
 * @id: core identifier.
 * @rev: core revision.
 * @base: base address of core register space.
 */
struct brcmf_core {
	u16 id;
	u16 rev;
	u32 base;
};

/**
 * struct brcmf_buscore_ops - buscore specific callbacks.
 *
 * @read32: read 32-bit value over bus.
 * @write32: write 32-bit value over bus.
 * @prepare: prepare bus for core configuration.
 * @setup: bus-specific core setup.
 * @active: chip becomes active.
 *	The callback should use the provided @rstvec when non-zero.
 * @blhs_attach: attach bootloader handshake handle
 */
struct brcmf_buscore_ops {
	u32 (*read32)(void *ctx, u32 addr);
	void (*write32)(void *ctx, u32 addr, u32 value);
	int (*prepare)(void *ctx);
	int (*reset)(void *ctx, struct brcmf_chip *chip);
	int (*setup)(void *ctx, struct brcmf_chip *chip);
	void (*activate)(void *ctx, struct brcmf_chip *chip, u32 rstvec);
	int (*blhs_attach)(void *ctx, struct brcmf_blhs **blhs, u32 flag,
			   uint timeout, uint interval);
};

/**
 * struct brcmf_blhs - bootloader handshake handle related information.
 *
 * @d2h: offset of dongle to host register for the handshake.
 * @h2d: offset of host to dongle register for the handshake.
 * @init: bootloader handshake initialization.
 * @prep_fwdl: handshake before firmware download.
 * @post_fwdl: handshake after firmware download.
 * @post_nvramdl: handshake after nvram download.
 * @chk_validation: handshake for firmware validation check.
 * @post_wdreset: handshake after watchdog reset.
 * @read: read value with register offset for the handshake.
 * @write: write value with register offset for the handshake.
 */
struct brcmf_blhs {
	u32 d2h;
	u32 h2d;
	void (*init)(struct brcmf_chip *pub);
	int (*prep_fwdl)(struct brcmf_chip *pub);
	int (*post_fwdl)(struct brcmf_chip *pub);
	void (*post_nvramdl)(struct brcmf_chip *pub);
	int (*chk_validation)(struct brcmf_chip *pub);
	int (*post_wdreset)(struct brcmf_chip *pub);
	u32 (*read)(void *ctx, u32 addr);
	void (*write)(void *ctx, u32 addr, u32 value);
};

int brcmf_chip_get_raminfo(struct brcmf_chip *pub);
struct brcmf_chip *brcmf_chip_attach(void *ctx,
				     const struct brcmf_buscore_ops *ops);
void brcmf_chip_detach(struct brcmf_chip *chip);
struct brcmf_core *brcmf_chip_get_core(struct brcmf_chip *chip, u16 coreid);
struct brcmf_core *brcmf_chip_get_d11core(struct brcmf_chip *pub, u8 unit);
struct brcmf_core *brcmf_chip_get_chipcommon(struct brcmf_chip *chip);
struct brcmf_core *brcmf_chip_get_pmu(struct brcmf_chip *pub);
bool brcmf_chip_iscoreup(struct brcmf_core *core);
void brcmf_chip_coredisable(struct brcmf_core *core, u32 prereset, u32 reset);
void brcmf_chip_resetcore(struct brcmf_core *core, u32 prereset, u32 reset,
			  u32 postreset);
void brcmf_chip_set_passive(struct brcmf_chip *ci);
bool brcmf_chip_set_active(struct brcmf_chip *ci, u32 rstvec);
bool brcmf_chip_sr_capable(struct brcmf_chip *pub);
char *brcmf_chip_name(u32 chipid, u32 chiprev, char *buf, uint len);
void brcmf_chip_reset_watchdog(struct brcmf_chip *pub);
void brcmf_chip_ulp_reset_lhl_regs(struct brcmf_chip *pub);
void brcmf_chip_reset_pmu_regs(struct brcmf_chip *pub);
void brcmf_chip_set_default_min_res_mask(struct brcmf_chip *pub);

#endif /* BRCMF_AXIDMP_H */
