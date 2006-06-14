/*
 * linux/include/asm-arm/arch-omap/menelaus.h
 *
 * Functions to access Menelaus power management chip
 */

#ifndef __ASM_ARCH_MENELAUS_H
#define __ASM_ARCH_MENELAUS_H

extern int menelaus_mmc_register(void (*callback)(unsigned long data, u8 card_mask),
				  unsigned long data);
extern int menelaus_mmc_remove(void);
extern int menelaus_mmc_opendrain(int enable);

extern int menelaus_set_vmem(unsigned int mV);
extern int menelaus_set_vio(unsigned int mV);

#if defined(CONFIG_ARCH_OMAP24XX) && defined(CONFIG_MENELAUS)
#define omap_has_menelaus()	1
#else
#define omap_has_menelaus()	0
#endif

#endif

