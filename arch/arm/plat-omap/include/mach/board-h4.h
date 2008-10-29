/*
 * arch/arm/plat-omap/include/mach/board-h4.h
 *
 * Hardware definitions for TI OMAP2420 H4 board.
 *
 * Initial creation by Dirk Behme <dirk.behme@de.bosch.com>
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

#ifndef __ASM_ARCH_OMAP_H4_H
#define __ASM_ARCH_OMAP_H4_H

/* MMC Prototypes */
extern void h4_mmc_init(void);

/* Placeholder for H4 specific defines */
#define OMAP24XX_ETHR_GPIO_IRQ		92

/* FPGA on debug board has 32 GPIOs:  16 dedicated to leds,
 * 8 outputs on a header, and 6 inputs from a DIP switch.
 */
#define H4_DEBUG_GPIO_BASE		OMAP_MAX_GPIO_LINES
#	define H4_DEBUG_GPIO_SW3_1	(H4_DEBUG_GPIO_BASE + 24)
#	define H4_DEBUG_GPIO_SW3_2	(H4_DEBUG_GPIO_BASE + 25)
#	define H4_DEBUG_GPIO_SW3_3	(H4_DEBUG_GPIO_BASE + 26)
#	define H4_DEBUG_GPIO_SW3_4	(H4_DEBUG_GPIO_BASE + 27)
#	define H4_DEBUG_GPIO_SW3_5	(H4_DEBUG_GPIO_BASE + 28)
#	define H4_DEBUG_GPIO_SW3_8	(H4_DEBUG_GPIO_BASE + 29)

/* H4 baseboard has 3 PCF8574 (8 bit) I2C GPIO expanders */
#define H4_U191_GPIO_BASE		(H4_DEBUG_GPIO_BASE + 32)
#	define H4_GPIO_IRDA_FIRSEL	(H4_U191_GPIO_BASE + 0)
#	define H4_GPIO_MODEM_MOD_EN	(H4_U191_GPIO_BASE + 1)
#	define H4_GPIO_WLAN_MOD_EN	(H4_U191_GPIO_BASE + 2)
#	define H4_GPIO_CAM_MODULE_EN	(H4_U191_GPIO_BASE + 3)
#	define H4_GPIO_HANDSET_EN	(H4_U191_GPIO_BASE + 4)
#	define H4_GPIO_LCD_ENBKL	(H4_U191_GPIO_BASE + 5)
#	define H4_GPIO_AUDIO_ENVDD	(H4_U191_GPIO_BASE + 6)
#	define H4_GPIO_LCD_ENVDD	(H4_U191_GPIO_BASE + 7)

#define H4_U192_GPIO_BASE		(H4_U191_GPIO_BASE + 8)
#	define H4_GPIO_IRDA_AGPSn	(H4_U192_GPIO_BASE + 0)
#	define H4_GPIO_AGPS_PWREN	(H4_U192_GPIO_BASE + 1)
#	define H4_GPIO_AGPS_RSTn	(H4_U192_GPIO_BASE + 2)
#	define H4_GPIO_AGPS_SLEEP	(H4_U192_GPIO_BASE + 3)
#	define H4_GPIO_AGPS_PA_XMT	(H4_U192_GPIO_BASE + 4)
#	define H4_GPIO_MODEM_SPR2	(H4_U192_GPIO_BASE + 5)
#	define H4_GPIO_MODEM_SPR1	(H4_U192_GPIO_BASE + 6)
#	define H4_GPIO_BT_ACLK_ENn	(H4_U192_GPIO_BASE + 7)

#define H4_U193_GPIO_BASE		(H4_U192_GPIO_BASE + 8)
#	define H4_GPIO_SPR0		(H4_U193_GPIO_BASE + 0)
#	define H4_GPIO_SPR1		(H4_U193_GPIO_BASE + 1)
#	define H4_GPIO_WLAN_SHUTDOWN	(H4_U193_GPIO_BASE + 2)
#	define H4_GPIO_WLAN_RESET	(H4_U193_GPIO_BASE + 3)
#	define H4_GPIO_WLAN_CLK_ENn	(H4_U193_GPIO_BASE + 4)
	/* 5, 6 not connected */
#	define H4_GPIO_CAM_RST		(H4_U193_GPIO_BASE + 7)

#endif /*  __ASM_ARCH_OMAP_H4_H */

