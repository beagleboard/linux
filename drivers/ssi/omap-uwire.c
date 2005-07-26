/*
 * BRIEF MODULE DESCRIPTION
 *
 *	uWire interface driver for the OMAP Platform
 *
 * Copyright 2003 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *	   source@mvista.com
 *
 * Ported to 2.6 uwire interface.
 * Copyright (C) 2004 Texas Instruments.
 *
 * Generalization patches by Juha Yrjölä <juha.yrjola@nokia.com>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED	  ``AS	IS'' AND   ANY	EXPRESS OR IMPLIED
 *  WARRANTIES,	  INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO	EVENT  SHALL   THE AUTHOR  BE	 LIABLE FOR ANY	  DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED	  TO, PROCUREMENT OF  SUBSTITUTE GOODS	OR SERVICES; LOSS OF
 *  USE, DATA,	OR PROFITS; OR	BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN	 CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/delay.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/arch/mux.h>
#include <asm/arch/omap730.h>	/* OMAP730_IO_CONF registers */

#include "omap-uwire.h"

/* uWire Registers: */
#define UWIRE_BASE    0xFFFB3000
#define UWIRE_IO_SIZE 0x20
#define UWIRE_TDR     0x00
#define UWIRE_RDR     0x00
#define UWIRE_CSR     0x01
#define UWIRE_SR1     0x02
#define UWIRE_SR2     0x03
#define UWIRE_SR3     0x04
#define UWIRE_SR4     0x05
#define UWIRE_SR5     0x06

static unsigned short uwire_flags[4];
static unsigned long uwire_base = io_p2v(UWIRE_BASE);
static spinlock_t uwire_lock;
static unsigned int uwire_idx_shift;

static inline void uwire_write_reg(int idx, u16 val)
{
	__raw_writew(val, uwire_base + (idx << uwire_idx_shift));
}

static inline u16 uwire_read_reg(int idx)
{
	return __raw_readw(uwire_base + (idx << uwire_idx_shift));
}

void omap_uwire_configure_mode(int cs, unsigned long flags)
{
	u16 w, val = 0;
	int shift, reg;

	BUG_ON(cs > 3);

	val = flags & 0x3f;
	if (flags & UWIRE_CLK_INVERTED)
		val ^= 0x03;
	if (cs & 1)
		shift = 6;
	else
		shift = 0;
	if (cs <= 1)
		reg = UWIRE_SR1;
	else
		reg = UWIRE_SR2;
	spin_lock(&uwire_lock);
	w = uwire_read_reg(reg);
	w &= ~(0x3f << shift);
	w |= val << shift;
	uwire_write_reg(reg, w);
	spin_unlock(&uwire_lock);

	uwire_flags[cs] = flags;
}

static int wait_uwire_csr_flag(u16 mask, u16 val, int might_not_catch)
{
	u16 w;
	int c = 0;
	unsigned long max_jiffies = jiffies + HZ;

	for (;;) {
		w = uwire_read_reg(UWIRE_CSR);
		if ((w & mask) == val)
			break;
		if (time_after(jiffies, max_jiffies)) {
			printk(KERN_ERR "%s: timeout. reg=%#06x mask=%#06x val=%#06x\n",
			       __FUNCTION__, w, mask, val);
			return -1;
		}
		c++;
		if (might_not_catch && c > 64)
			break;
	}
	return 0;
}

int omap_uwire_data_transfer(int cs, u16 tx_data, int tx_size, int rx_size,
			     u16 *rx_buf, int leave_cs_active)
{
	u16 ret = -1, w;
	u16 mask;

	BUG_ON(cs > 3);
	BUG_ON(rx_size && !rx_buf);

	spin_lock(&uwire_lock);

	if (wait_uwire_csr_flag(1 << 14, 0, 0))
		goto exit;

	if (uwire_flags[cs] & UWIRE_CLK_INVERTED)
		uwire_write_reg(UWIRE_SR4, 1);
	else
		uwire_write_reg(UWIRE_SR4, 0);

	w = cs << 10;
	w |= 1 << 12;				/* CS_CMD : activate CS */
	uwire_write_reg(UWIRE_CSR, w);

	/* Shift data to 16bit MSb and place it in TX register. */
	uwire_write_reg(UWIRE_TDR, tx_data << (16 - tx_size));

	if (wait_uwire_csr_flag(1 << 14, 0, 0))
		goto exit;

	w = rx_size | (tx_size << 5) | (cs << 10);
	w |= (1 << 12) | (1 << 13);
	/* Start uWire read/write */
	uwire_write_reg(UWIRE_CSR, w);

	/* Wait till read/write actually starts.
	 * This is needed at high (>=60MHz) MPU frequencies
	 * REVISIT: But occasionally we won't have time to catch it
	 */
	if (wait_uwire_csr_flag(1 << 14, 1 << 14, 1))
		goto exit;

	/* Wait for both transfers to be completed */
	mask = 1 << 14;			/* CSRB : reg busy */
	w = 0;
	if (rx_size) {
		mask |= 1 << 15;	/* RDRB : reg busy */
		w |= 1 << 15;
	}

	if (wait_uwire_csr_flag(mask, w, 0))
		goto exit;

	if (rx_size)
		*rx_buf = uwire_read_reg(UWIRE_RDR);

	if (!leave_cs_active)
		uwire_write_reg(UWIRE_CSR, cs << 10);

	ret = 0;

exit:
	spin_unlock(&uwire_lock);
	return ret;
}

static int __init omap_uwire_init(void)
{
	spin_lock_init(&uwire_lock);
	if (cpu_is_omap730())
		uwire_idx_shift = 1;
	else
		uwire_idx_shift = 2;

	uwire_write_reg(UWIRE_SR3, 1);
	if (machine_is_omap_h2()) {
		/* defaults: W21 SDO, U18 SDI, V19 SCL */
		omap_cfg_reg(N14_1610_UWIRE_CS0);
		omap_cfg_reg(N15_1610_UWIRE_CS1);
	}
	if (machine_is_omap_osk()) {
		/* this is the standard expansion connector usage, with
		 * the other chipselect pins for MPUIO2 and MPUIO4.
		 */
		omap_cfg_reg(N14_1610_UWIRE_CS0);
		omap_cfg_reg(P15_1610_UWIRE_CS3);
	}
	if (machine_is_omap_perseus2()) {
		/* configure pins: MPU_UW_nSCS1, MPU_UW_SDO, MPU_UW_SCLK */
		int val = omap_readl(OMAP730_IO_CONF_9) & ~0x00EEE000;
		omap_writel(val | 0x00AAA000, OMAP730_IO_CONF_9);
	}
	return 0;
}

static void __exit omap_uwire_exit(void)
{
}

subsys_initcall(omap_uwire_init);
module_exit(omap_uwire_exit);

EXPORT_SYMBOL(omap_uwire_configure_mode);
EXPORT_SYMBOL(omap_uwire_data_transfer);

MODULE_LICENSE("GPL");
