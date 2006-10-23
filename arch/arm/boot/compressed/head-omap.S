/*
 * linux/arch/arm/boot/compressed/head-omap.S
 *
 * OMAP specific tweaks.  This is merged into head.S by the linker.
 *
 */

#include <linux/linkage.h>
#include <asm/mach-types.h>

		.section        ".start", "ax"

__OMAP_start:
#ifdef CONFIG_MACH_OMAP_PERSEUS2
		/* support for booting without u-boot */
		mov	r7, #(MACH_TYPE_OMAP_PERSEUS2 & ~0xf)
		orr	r7, r7, #(MACH_TYPE_OMAP_PERSEUS2 & 0xf)
#endif
