/*
 *  linux/drivers/i2c/i2c-omap1610.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      OMAP I2C register definitions
 *
 *  Copyright (C) 2004 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 * 
 *  HISTORY:
 * 
 *  20040824: Thiago Radicchi <trr@dcc.ufmg.br>  DCC-UFMG / iNdT
 *       Removed some ifdefs which broke compilation for some platforms.
 *       Added new defintion for interrupt vector.
 */

/* I2C Registers: */

#define OMAP_I2C_BASE        IO_ADDRESS(0xfffb3800)
#define OMAP_I2C_IOSIZE      (0x40)
#define OMAP_I2C_REV         (OMAP_I2C_BASE + 0x00)
#define OMAP_I2C_IE          (OMAP_I2C_BASE + 0x04)
#define OMAP_I2C_STAT        (OMAP_I2C_BASE + 0x08)
#define OMAP_I2C_IV          (OMAP_I2C_BASE + 0x0c)
#define OMAP_I2C_SYSS        (OMAP_I2C_BASE + 0x10)
#define OMAP_I2C_BUF         (OMAP_I2C_BASE + 0x14)
#define OMAP_I2C_CNT         (OMAP_I2C_BASE + 0x18)
#define OMAP_I2C_DATA        (OMAP_I2C_BASE + 0x1c)
#define OMAP_I2C_SYSC        (OMAP_I2C_BASE + 0x20)
#define OMAP_I2C_CON         (OMAP_I2C_BASE + 0x24)
#define OMAP_I2C_OA          (OMAP_I2C_BASE + 0x28)
#define OMAP_I2C_SA          (OMAP_I2C_BASE + 0x2c)
#define OMAP_I2C_PSC         (OMAP_I2C_BASE + 0x30)
#define OMAP_I2C_SCLL        (OMAP_I2C_BASE + 0x34)
#define OMAP_I2C_SCLH        (OMAP_I2C_BASE + 0x38)
#define OMAP_I2C_SYSTEST     (OMAP_I2C_BASE + 0x3c)

/* I2C Interrupt Enable Register (OMAP_I2C_IE): */

#define OMAP_I2C_IE_XRDY_IE  (1 << 4)	/* Transmit data ready interrupt enable */
#define OMAP_I2C_IE_RRDY_IE  (1 << 3)	/* Receive data ready interrupt enable */
#define OMAP_I2C_IE_ARDY_IE  (1 << 2)	/* Register access ready interrupt enable */
#define OMAP_I2C_IE_NACK_IE  (1 << 1)	/* No acknowledgment interrupt enable */
#define OMAP_I2C_IE_AL_IE    (1 << 0)	/* Arbitration lost interrupt enable */

/* I2C Status Register (OMAP_I2C_STAT): */

#define OMAP_I2C_STAT_SBD    (1 << 15)	/* Single byte data */
#define OMAP_I2C_STAT_BB     (1 << 12)	/* Bus busy */
#define OMAP_I2C_STAT_ROVR   (1 << 11)	/* Receive overrun */
#define OMAP_I2C_STAT_XUDF   (1 << 10)	/* Transmit underflow */
#define OMAP_I2C_STAT_AAS    (1 << 9)	/* Address as slave */
#define OMAP_I2C_STAT_AD0    (1 << 8)	/* Address zero */
#define OMAP_I2C_STAT_XRDY   (1 << 4)	/* Transmit data ready */
#define OMAP_I2C_STAT_RRDY   (1 << 3)	/* Receive data ready */
#define OMAP_I2C_STAT_ARDY   (1 << 2)	/* Register access ready */
#define OMAP_I2C_STAT_NACK   (1 << 1)	/* No acknowledgment interrupt enable */
#define OMAP_I2C_STAT_AL     (1 << 0)	/* Arbitration lost interrupt enable */

/* I2C Buffer Configuration Register (OMAP_I2C_BUF): */

#define OMAP_I2C_BUF_RDMA_EN         (1 << 15)	/* Receive DMA channel enable */
#define OMAP_I2C_BUF_XDMA_EN         (1 << 7)	/* Transmit DMA channel enable */

/* I2C Configuration Register (OMAP_I2C_CON): */

#define OMAP_I2C_CON_EN      (1 << 15)	/* I2C module enable */
#define OMAP_I2C_CON_RST     (0 << 15)  /* I2C module reset */
#define OMAP_I2C_CON_BE      (1 << 14)	/* Big endian mode */
#define OMAP_I2C_CON_STB     (1 << 11)	/* Start byte mode (master mode only) */
#define OMAP_I2C_CON_MST     (1 << 10)	/* Master/slave mode */
#define OMAP_I2C_CON_TRX     (1 << 9)	/* Transmitter/receiver mode (master mode only) */
#define OMAP_I2C_CON_XA      (1 << 8)	/* Expand address */
#define OMAP_I2C_CON_RM      (1 << 2)	/* Repeat mode (master mode only) */
#define OMAP_I2C_CON_STP     (1 << 1)	/* Stop condition (master mode only) */
#define OMAP_I2C_CON_STT     (1 << 0)	/* Start condition (master mode only) */

/* I2C System Test Register (OMAP_I2C_SYSTEST): */

#define OMAP_I2C_SYSTEST_ST_EN       (1 << 15)	/* System test enable */
#define OMAP_I2C_SYSTEST_FREE        (1 << 14)	/* Free running mode (on breakpoint) */
#define OMAP_I2C_SYSTEST_TMODE_MASK  (3 << 12)	/* Test mode select */
#define OMAP_I2C_SYSTEST_TMODE_SHIFT (12)	/* Test mode select */
#define OMAP_I2C_SYSTEST_SCL_I       (1 << 3)	/* SCL line sense input value */
#define OMAP_I2C_SYSTEST_SCL_O       (1 << 2)	/* SCL line drive output value */
#define OMAP_I2C_SYSTEST_SDA_I       (1 << 1)	/* SDA line sense input value */
#define OMAP_I2C_SYSTEST_SDA_O       (1 << 0)	/* SDA line drive output value */

/* I2C System Status register (OMAP_I2C_SYSS): */

#define OMAP_I2C_SYSS_RDONE          1	/* Reset Done */

/* I2C System Configuration Register (OMAP_I2C_SYSC): */

#define OMAP_I2C_SYSC_SRST           (1 << 1)	/* Soft Reset */
