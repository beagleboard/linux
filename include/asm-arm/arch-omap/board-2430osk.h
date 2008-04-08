/*
 * linux/include/asm-arm/arch-omap/board-2430osk.h
 *
 * Hardware definitions for TI OMAP2430 OSK board.
 *
 * Based on board-2330sdp.h
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

#ifndef __ASM_ARCH_OMAP_2430OSK_H
#define __ASM_ARCH_OMAP_2430OSK_H

/* Placeholder for 2430OSK specific defines */
#define OMAP24XX_ETHR_START		0x08000300
#define OMAP24XX_ETHR_GPIO_IRQ		149

#ifdef CONFIG_TWL4030_CORE

#define TWL4030_IRQNUM			INT_24XX_SYS_NIRQ

/* TWL4030 Primary Interrupt Handler (PIH) interrupts */
#define IH_TWL4030_BASE			IH_BOARD_BASE
#define IH_TWL4030_END			(IH_TWL4030_BASE+8)

#define IH_TWL4030_PWRBASE		(IH_TWL4030_END)
#define IH_TWL4030_PWRBASE_END		(IH_TWL4030_PWRBASE+8)

#ifdef CONFIG_TWL4030_GPIO

/* TWL4030 GPIO Interrupts */
#define IH_TWL4030_GPIO_BASE		(IH_TWL4030_PWRBASE_END)
#define IH_TWL4030_GPIO_END		(IH_TWL4030_GPIO_BASE+18)
#define NR_IRQS				(IH_TWL4030_GPIO_END)
#else
#define NR_IRQS				(IH_TWL4030_PWRBASE_END)
#endif /* CONFIG_I2C_TWL4030_GPIO */
#endif /* End of support for TWL4030 */

#endif /* __ASM_ARCH_OMAP_2430OSK_H */
