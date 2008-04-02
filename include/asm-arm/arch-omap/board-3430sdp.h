/*
 * linux/include/asm-arm/arch-omap/board-3430sdp.h
 *
 * Hardware definitions for TI OMAP3430 SDP board.
 *
 * Initial creation by Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ASM_ARCH_OMAP_3430SDP_H
#define __ASM_ARCH_OMAP_3430SDP_H

extern void sdp3430_usb_init(void);
extern void sdp_mmc_init(void);

#define DEBUG_BASE			0x08000000  /* debug board */

/* Placeholder for 3430SDP specific defines */

#define OMAP34XX_ETHR_START		DEBUG_BASE
#define OMAP34XX_ETHR_GPIO_IRQ_SDPV1	29
#define OMAP34XX_ETHR_GPIO_IRQ_SDPV2	6

/*
 * GPIO used for TSC2046, TI's Touchscreen controller
 */
#define OMAP34XX_TS_GPIO_IRQ_SDPV1     3
#define OMAP34XX_TS_GPIO_IRQ_SDPV2     2

/* NAND */
/* IMPORTANT NOTE ON MAPPING
 * 3430SDP - 34XX
 * ----------
 * NOR always on 0x04000000 for SDPV1
 * NOR always on 0x10000000 for SDPV2
 * MPDB always on 0x08000000
 * NAND always on 0x0C000000
 * OneNand Mapped to 0x20000000
 * Boot Mode(NAND/NOR). The other on CS1
 */
#define FLASH_BASE_SDPV1	0x04000000 /* NOR flash (64 Meg aligned) */
#define FLASH_BASE_SDPV2	0x10000000 /* NOR flash (256 Meg aligned) */
#define DEBUG_BASE		0x08000000 /* debug board */
#define NAND_BASE		0x0C000000 /* NAND flash */
#define ONENAND_MAP		0x20000000 /* OneNand flash */

/* various memory sizes */
#define FLASH_SIZE_SDPV1	SZ_64M
#define FLASH_SIZE_SDPV2	SZ_128M

#ifdef CONFIG_TWL4030_CORE

#define TWL4030_IRQNUM INT_34XX_SYS_NIRQ

/* TWL4030 Primary Interrupt Handler (PIH) interrupts */
#define	IH_TWL4030_BASE		IH_BOARD_BASE
#define	IH_TWL4030_END		(IH_TWL4030_BASE+8)

#define IH_TWL4030_PWRBASE     (IH_TWL4030_END)
#define IH_TWL4030_PWRBASE_END (IH_TWL4030_PWRBASE+8)

#ifdef CONFIG_TWL4030_GPIO

/* TWL4030 GPIO Interrupts */
#define IH_TWL4030_GPIO_BASE	(IH_TWL4030_PWRBASE_END)
#define IH_TWL4030_GPIO_END	(IH_TWL4030_GPIO_BASE+18)
#define NR_IRQS			(IH_TWL4030_GPIO_END)
#else
#define NR_IRQS			(IH_TWL4030_PWRBASE_END)
#endif /* CONFIG_I2C_TWL4030_GPIO */
#endif /* End of support for TWL4030 */
#endif /*  __ASM_ARCH_OMAP_3430SDP_H */

