/*
 * BRIEF MODULE DESCRIPTION
 *
 *	Infra-red driver for the OMAP1610-H2 and OMAP1710-H3 and H4 Platforms
 *	  (SIR/MIR/FIR modes)
 *	  (based on omap-sir.c)
 *
 * Copyright 2003 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *	   source@mvista.com
 *
 * Copyright 2004 Texas Instruments.
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
 *
 Modifications:
 Feb 2004, Texas Instruments
 - Ported to 2.6 kernel (Feb 2004).
 *
 Apr 2004, Texas Instruments
 - Added support for H3 (Apr 2004).
 Nov 2004, Texas Instruments
 - Added support for Power Management.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/rtnetlink.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

#include <net/irda/irda.h>
#include <net/irda/irmod.h>
#include <net/irda/wrapper.h>
#include <net/irda/irda_device.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <asm/serial.h>
#include <asm/mach-types.h>
#include <asm/dma.h>
#include <mach/mux.h>
#include <mach/gpio.h>
#include <mach/irda.h>

#define UART3_EFR_EN			(1 << 4)
#define UART3_MCR_EN_TCR_TLR		(1 << 6)

#define UART3_LCR_WL_8			(3 << 0)
#define UART3_LCR_SP2			(1 << 2)
#define UART3_LCR_DIVEN			(1 << 7)

#define UART3_FCR_FIFO_EN		(1 << 0)
#define UART3_FCR_FIFO_RX		(1 << 1)
#define UART3_FCR_FIFO_TX		(1 << 2)
#define UART3_FCR_FIFO_DMA1		(1 << 3)
#define UART3_FCR_FIFO_TX_TRIG16	(1 << 4)
#define UART3_FCR_FIFO_RX_TRIG16	(1 << 6)
#define UART3_FCR_CONFIG	(\
		UART3_FCR_FIFO_EN | UART3_FCR_FIFO_RX	|\
		UART3_FCR_FIFO_TX | UART3_FCR_FIFO_DMA1 |\
		UART3_FCR_FIFO_TX_TRIG16		|\
		UART3_FCR_FIFO_RX_TRIG16)

#define UART3_SCR_TX_TRIG1		(1 << 6)
#define UART3_SCR_RX_TRIG1		(1 << 7)

#define UART3_MDR1_RESET		(0x07)
#define UART3_MDR1_SIR			(1 << 0)
#define UART3_MDR1_MIR			(4 << 0)
#define UART3_MDR1_FIR			(5 << 0)
#define UART3_MDR1_SIP_AUTO		(1 << 6)

#define UART3_MDR2_TRIG1		(0 << 1)
#define UART3_MDR2_IRTX_UNDERRUN	(1 << 0)

#define UART3_ACERG_TX_UNDERRUN_DIS	(1 << 4)
#define UART3_ACERG_SD_MODE_LOW		(1 << 6)
#define UART3_ACERG_DIS_IR_RX		(1 << 5)

#define UART3_IER_EOF			(1 << 5)
#define UART3_IER_CTS			(1 << 7)

#define UART3_IIR_TX_STATUS		(1 << 5)
#define UART3_IIR_EOF			(0x80)

#define IS_FIR(omap_ir)		((omap_ir)->speed >= 4000000)

struct omap_irda {
	unsigned char open;
	int speed;		/* Current IrDA speed */
	int newspeed;

	struct net_device_stats stats;
	struct irlap_cb *irlap;
	struct qos_info qos;

	int rx_dma_channel;
	int tx_dma_channel;

	dma_addr_t rx_buf_dma_phys;	/* Physical address of RX DMA buffer */
	dma_addr_t tx_buf_dma_phys;	/* Physical address of TX DMA buffer */

	void *rx_buf_dma_virt;		/* Virtual address of RX DMA buffer */
	void *tx_buf_dma_virt;		/* Virtual address of TX DMA buffer */

	struct device *dev;
	struct omap_irda_config *pdata;
};

static void inline uart_reg_out(int idx, u8 val)
{
	omap_writeb(val, idx);
}

static u8 inline uart_reg_in(int idx)
{
	u8 b = omap_readb(idx);
	return b;
}

/* forward declarations */
extern void omap_stop_dma(int lch);
static int omap_irda_set_speed(struct net_device *dev, int speed);

static void omap_irda_start_rx_dma(struct omap_irda *omap_ir)
{
	/* Configure DMA */
	omap_set_dma_src_params(omap_ir->rx_dma_channel, 0x3, 0x0,
				omap_ir->pdata->src_start,
				0, 0);

	omap_enable_dma_irq(omap_ir->rx_dma_channel, 0x01);

	omap_set_dma_dest_params(omap_ir->rx_dma_channel, 0x0, 0x1,
				omap_ir->rx_buf_dma_phys,
				0, 0);

	omap_set_dma_transfer_params(omap_ir->rx_dma_channel, 0x0,
				IRDA_SKB_MAX_MTU, 0x1,
				0x0, omap_ir->pdata->rx_trigger, 0);

	omap_start_dma(omap_ir->rx_dma_channel);
}

static void omap_start_tx_dma(struct omap_irda *omap_ir, int size)
{
	/* Configure DMA */
	omap_set_dma_dest_params(omap_ir->tx_dma_channel, 0x03, 0x0,
				omap_ir->pdata->dest_start, 0, 0);

	omap_enable_dma_irq(omap_ir->tx_dma_channel, 0x01);

	omap_set_dma_src_params(omap_ir->tx_dma_channel, 0x0, 0x1,
				omap_ir->tx_buf_dma_phys,
				0, 0);

	omap_set_dma_transfer_params(omap_ir->tx_dma_channel, 0x0, size, 0x1,
				0x0, omap_ir->pdata->tx_trigger, 0);

	/* Start DMA */
	omap_start_dma(omap_ir->tx_dma_channel);
}

/* DMA RX callback - normally, we should not go here,
 * it calls only if something is going wrong
 */
static void omap_irda_rx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct net_device *dev = data;
	struct omap_irda *omap_ir = netdev_priv(dev);

	printk(KERN_ERR "RX Transfer error or very big frame\n");

	/* Clear interrupts */
	uart_reg_in(UART3_IIR);

	omap_ir->stats.rx_frame_errors++;

	uart_reg_in(UART3_RESUME);

	/* Re-init RX DMA */
	omap_irda_start_rx_dma(omap_ir);
}

/* DMA TX callback - calling when frame transfer has been finished */
static void omap_irda_tx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct net_device *dev = data;
	struct omap_irda *omap_ir = netdev_priv(dev);

	/*Stop DMA controller */
	omap_stop_dma(omap_ir->tx_dma_channel);
}

/*
 * Set the IrDA communications speed.
 * Interrupt have to be disabled here.
 */
static int omap_irda_startup(struct net_device *dev)
{
	struct omap_irda *omap_ir = netdev_priv(dev);

	/* FIXME: use clk_* apis for UART3 clock*/
	/* Enable UART3 clock and set UART3 to IrDA mode */
	if (machine_is_omap_h2() || machine_is_omap_h3())
		omap_writel(omap_readl(MOD_CONF_CTRL_0) | (1 << 31) | (1 << 15),
				MOD_CONF_CTRL_0);

	/* Only for H2?
	 */
	if (omap_ir->pdata->transceiver_mode && machine_is_omap_h2()) {
		/* Is it select_irda on H2 ? */
		omap_writel(omap_readl(FUNC_MUX_CTRL_A) | 7,
					FUNC_MUX_CTRL_A);
		omap_ir->pdata->transceiver_mode(omap_ir->dev, IR_SIRMODE);
	}

	uart_reg_out(UART3_MDR1, UART3_MDR1_RESET);	/* Reset mode */

	/* Clear DLH and DLL */
	uart_reg_out(UART3_LCR, UART3_LCR_DIVEN);

	uart_reg_out(UART3_DLL, 0);
	uart_reg_out(UART3_DLH, 0);
	uart_reg_out(UART3_LCR, 0xbf);	/* FIXME: Add #define */

	uart_reg_out(UART3_EFR, UART3_EFR_EN);
	uart_reg_out(UART3_LCR, UART3_LCR_DIVEN);

	/* Enable access to UART3_TLR and UART3_TCR registers */
	uart_reg_out(UART3_MCR, UART3_MCR_EN_TCR_TLR);

	uart_reg_out(UART3_SCR, 0);
	/* Set Rx trigger to 1 and Tx trigger to 1 */
	uart_reg_out(UART3_TLR, 0);

	/* Set LCR to 8 bits and 1 stop bit */
	uart_reg_out(UART3_LCR, 0x03);

	/* Clear RX and TX FIFO and enable FIFO */
	/* Use DMA Req for transfers */
	uart_reg_out(UART3_FCR, UART3_FCR_CONFIG);

	uart_reg_out(UART3_MCR, 0);

	uart_reg_out(UART3_SCR, UART3_SCR_TX_TRIG1 |
			UART3_SCR_RX_TRIG1);

	/* Enable UART3 SIR Mode,(Frame-length method to end frames) */
	uart_reg_out(UART3_MDR1, UART3_MDR1_SIR);

	/* Set Status FIFO trig to 1 */
	uart_reg_out(UART3_MDR2, 0);

	/* Enables RXIR input */
	/* and disable TX underrun */
	/* SEND_SIP pulse */
	uart_reg_out(UART3_ACREG, UART3_ACERG_SD_MODE_LOW |
			UART3_ACERG_TX_UNDERRUN_DIS);

	/* Enable EOF Interrupt only */
	uart_reg_out(UART3_IER, UART3_IER_CTS | UART3_IER_EOF);

	/* Set Maximum Received Frame size to 2048 bytes */
	uart_reg_out(UART3_RXFLL, 0x00);
	uart_reg_out(UART3_RXFLH, 0x08);

	uart_reg_in(UART3_RESUME);

	return 0;
}

static int omap_irda_shutdown(struct omap_irda *omap_ir)
{
	unsigned long flags;

	local_irq_save(flags);

	/* Disable all UART3 Interrupts */
	uart_reg_out(UART3_IER, 0);

	/* Disable UART3 and disable baud rate generator */
	uart_reg_out(UART3_MDR1, UART3_MDR1_RESET);

	/* set SD_MODE pin to high and Disable RX IR */
	uart_reg_out(UART3_ACREG, (UART3_ACERG_DIS_IR_RX |
			~(UART3_ACERG_SD_MODE_LOW)));

	/* Clear DLH and DLL */
	uart_reg_out(UART3_LCR, UART3_LCR_DIVEN);
	uart_reg_out(UART3_DLL, 0);
	uart_reg_out(UART3_DLH, 0);

	local_irq_restore(flags);

	return 0;
}

static irqreturn_t
omap_irda_irq(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct omap_irda *omap_ir = netdev_priv(dev);
	struct sk_buff *skb;

	u8 status;
	int w = 0;

	/* Clear EOF interrupt */
	status = uart_reg_in(UART3_IIR);

	if (status & UART3_IIR_TX_STATUS) {
		u8 mdr2 = uart_reg_in(UART3_MDR2);
		if (mdr2 & UART3_MDR2_IRTX_UNDERRUN)
			printk(KERN_ERR "IrDA Buffer underrun error\n");

		omap_ir->stats.tx_packets++;

		if (omap_ir->newspeed) {
			omap_irda_set_speed(dev, omap_ir->newspeed);
			omap_ir->newspeed = 0;
		}

		netif_wake_queue(dev);
		if (!(status & UART3_IIR_EOF))
			return IRQ_HANDLED;
	}

	/* Stop DMA and if there are no errors, send frame to upper layer */
	omap_stop_dma(omap_ir->rx_dma_channel);

	status = uart_reg_in(UART3_SFLSR);	/* Take a frame status */

	if (status != 0) {	/* Bad frame? */
		omap_ir->stats.rx_frame_errors++;
		uart_reg_in(UART3_RESUME);
	} else {
		/* We got a frame! */
		skb = dev_alloc_skb(IRDA_SKB_MAX_MTU);

		if (!skb) {
			printk(KERN_ERR "omap_sir: out of memory for RX SKB\n");
			return IRQ_HANDLED;
		}
		/*
		 * Align any IP headers that may be contained
		 * within the frame.
		 */

		skb_reserve(skb, 1);

		w = omap_get_dma_dst_pos(omap_ir->rx_dma_channel) -
						omap_ir->rx_buf_dma_phys;

		if (!IS_FIR(omap_ir))
			/* Copy DMA buffer to skb */
			memcpy(skb_put(skb, w - 2), omap_ir->rx_buf_dma_virt,
					w - 2);
		else
			/* Copy DMA buffer to skb */
			memcpy(skb_put(skb, w - 4), omap_ir->rx_buf_dma_virt,
					w - 4);

		skb->dev = dev;
		skb_reset_mac_header(skb);
		skb->protocol = htons(ETH_P_IRDA);
		omap_ir->stats.rx_packets++;
		omap_ir->stats.rx_bytes += skb->len;
		netif_receive_skb(skb);	/* Send data to upper level */
	}

	/* Re-init RX DMA */
	omap_irda_start_rx_dma(omap_ir);

	dev->last_rx = jiffies;

	return IRQ_HANDLED;
}

static int omap_irda_hard_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct omap_irda *omap_ir = netdev_priv(dev);
	int speed = irda_get_next_speed(skb);
	int mtt = irda_get_mtt(skb);
	int xbofs = irda_get_next_xbofs(skb);


	/*
	 * Does this packet contain a request to change the interface
	 * speed?  If so, remember it until we complete the transmission
	 * of this frame.
	 */
	if (speed != omap_ir->speed && speed != -1)
		omap_ir->newspeed = speed;

	if (xbofs) /* Set number of addtional BOFS */
		uart_reg_out(UART3_EBLR, xbofs + 1);

	/*
	 * If this is an empty frame, we can bypass a lot.
	 */
	if (skb->len == 0) {
		if (omap_ir->newspeed) {
			omap_ir->newspeed = 0;
			omap_irda_set_speed(dev, speed);
		}
		dev_kfree_skb(skb);
		return 0;
	}

	netif_stop_queue(dev);

	/* Copy skb data to DMA buffer */
	skb_copy_from_linear_data(skb, omap_ir->tx_buf_dma_virt, skb->len);

	/* Copy skb data to DMA buffer */
	omap_ir->stats.tx_bytes += skb->len;

	/* Set frame length */
	uart_reg_out(UART3_TXFLL, (skb->len & 0xff));
	uart_reg_out(UART3_TXFLH, (skb->len >> 8));

	if (mtt > 1000)
		mdelay(mtt / 1000);
	else
		udelay(mtt);

	/* Start TX DMA transfer */
	omap_start_tx_dma(omap_ir, skb->len);

	/* We can free skb now because it's already in DMA buffer */
	dev_kfree_skb(skb);

	dev->trans_start = jiffies;

	return 0;
}

static int
omap_irda_ioctl(struct net_device *dev, struct ifreq *ifreq, int cmd)
{
	struct if_irda_req *rq = (struct if_irda_req *)ifreq;
	struct omap_irda *omap_ir = netdev_priv(dev);
	int ret = -EOPNOTSUPP;


	switch (cmd) {
	case SIOCSBANDWIDTH:
		if (capable(CAP_NET_ADMIN)) {
			/*
			 * We are unable to set the speed if the
			 * device is not running.
			 */
			if (omap_ir->open)
				ret = omap_irda_set_speed(dev,
						rq->ifr_baudrate);
			else {
				printk(KERN_ERR "omap_ir: SIOCSBANDWIDTH:"
						" !netif_running\n");
				ret = 0;
			}
		}
		break;

	case SIOCSMEDIABUSY:
		ret = -EPERM;
		if (capable(CAP_NET_ADMIN)) {
			irda_device_set_media_busy(dev, TRUE);
			ret = 0;
		}
		break;

	case SIOCGRECEIVING:
		rq->ifr_receiving = 0;
		break;

	default:
		break;
	}

	return ret;
}

static struct net_device_stats *omap_irda_stats(struct net_device *dev)
{
	struct omap_irda *omap_ir = netdev_priv(dev);
	return &omap_ir->stats;
}

static int omap_irda_start(struct net_device *dev)
{
	struct omap_irda *omap_ir = netdev_priv(dev);
	int err;

	omap_ir->speed = 9600;

	err = request_irq(dev->irq, omap_irda_irq, 0, dev->name, dev);
	if (err)
		goto err_irq;

	/*
	 * The interrupt must remain disabled for now.
	 */
	disable_irq(dev->irq);

	/*  Request DMA channels for IrDA hardware */
	if (omap_request_dma(omap_ir->pdata->rx_channel, "IrDA Rx DMA",
			(void *)omap_irda_rx_dma_callback,
			dev, &(omap_ir->rx_dma_channel))) {
		printk(KERN_ERR "Failed to request IrDA Rx DMA\n");
		goto err_irq;
	}

	if (omap_request_dma(omap_ir->pdata->tx_channel, "IrDA Tx DMA",
			(void *)omap_irda_tx_dma_callback,
			dev, &(omap_ir->tx_dma_channel))) {
		printk(KERN_ERR "Failed to request IrDA Tx DMA\n");
		goto err_irq;
	}

	/* Allocate TX and RX buffers for DMA channels */
	omap_ir->rx_buf_dma_virt =
		dma_alloc_coherent(NULL, IRDA_SKB_MAX_MTU,
				&(omap_ir->rx_buf_dma_phys),
				GFP_KERNEL);

	if (!omap_ir->rx_buf_dma_virt) {
		printk(KERN_ERR "Unable to allocate memory for rx_buf_dma\n");
		goto err_irq;
	}

	omap_ir->tx_buf_dma_virt =
		dma_alloc_coherent(NULL, IRDA_SIR_MAX_FRAME,
				&(omap_ir->tx_buf_dma_phys),
				GFP_KERNEL);

	if (!omap_ir->tx_buf_dma_virt) {
		printk(KERN_ERR "Unable to allocate memory for tx_buf_dma\n");
		goto err_mem1;
	}

	/*
	 * Setup the serial port for the specified config.
	 */
	if (omap_ir->pdata->select_irda)
		omap_ir->pdata->select_irda(omap_ir->dev, IR_SEL);

	err = omap_irda_startup(dev);

	if (err)
		goto err_startup;

	omap_irda_set_speed(dev, omap_ir->speed = 9600);

	/*
	 * Open a new IrLAP layer instance.
	 */
	omap_ir->irlap = irlap_open(dev, &omap_ir->qos, "omap_sir");

	err = -ENOMEM;
	if (!omap_ir->irlap)
		goto err_irlap;

	/* Now enable the interrupt and start the queue */
	omap_ir->open = 1;

	/* Start RX DMA */
	omap_irda_start_rx_dma(omap_ir);

	enable_irq(dev->irq);
	netif_start_queue(dev);

	return 0;

err_irlap:
	omap_ir->open = 0;
	omap_irda_shutdown(omap_ir);
err_startup:
	dma_free_coherent(NULL, IRDA_SIR_MAX_FRAME,
			omap_ir->tx_buf_dma_virt, omap_ir->tx_buf_dma_phys);
err_mem1:
	dma_free_coherent(NULL, IRDA_SKB_MAX_MTU,
			omap_ir->rx_buf_dma_virt, omap_ir->rx_buf_dma_phys);
err_irq:
	free_irq(dev->irq, dev);
	return err;
}

static int omap_irda_stop(struct net_device *dev)
{
	struct omap_irda *omap_ir = netdev_priv(dev);

	disable_irq(dev->irq);

	netif_stop_queue(dev);

	omap_free_dma(omap_ir->rx_dma_channel);
	omap_free_dma(omap_ir->tx_dma_channel);

	if (omap_ir->rx_buf_dma_virt)
		dma_free_coherent(NULL, IRDA_SKB_MAX_MTU,
				omap_ir->rx_buf_dma_virt,
				omap_ir->rx_buf_dma_phys);
	if (omap_ir->tx_buf_dma_virt)
		dma_free_coherent(NULL, IRDA_SIR_MAX_FRAME,
				omap_ir->tx_buf_dma_virt,
				omap_ir->tx_buf_dma_phys);

	omap_irda_shutdown(omap_ir);

	/* Stop IrLAP */
	if (omap_ir->irlap) {
		irlap_close(omap_ir->irlap);
		omap_ir->irlap = NULL;
	}

	omap_ir->open = 0;

	/*
	 * Free resources
	 */
	free_irq(dev->irq, dev);

	return 0;
}

static int omap_irda_set_speed(struct net_device *dev, int speed)
{
	struct omap_irda *omap_ir = netdev_priv(dev);
	int divisor;
	unsigned long flags;

	/* Set IrDA speed */
	if (speed <= 115200) {

		local_irq_save(flags);

		/* SIR mode */
		if (omap_ir->pdata->transceiver_mode)
			omap_ir->pdata->transceiver_mode(omap_ir->dev,
							IR_SIRMODE);

		/* Set SIR mode */
		uart_reg_out(UART3_MDR1, 1);
		uart_reg_out(UART3_EBLR, 1);

		divisor = 48000000 / (16 * speed);	/* Base clock 48 MHz */

		uart_reg_out(UART3_LCR, UART3_LCR_DIVEN);
		uart_reg_out(UART3_DLL, (divisor & 0xff));
		uart_reg_out(UART3_DLH, (divisor >> 8));
		uart_reg_out(UART3_LCR, 0x03);

		uart_reg_out(UART3_MCR, 0);

		local_irq_restore(flags);
	} else if (speed <= 1152000) {

		local_irq_save(flags);

		/* Set MIR mode, auto SIP */
		uart_reg_out(UART3_MDR1, UART3_MDR1_MIR |
				UART3_MDR1_SIP_AUTO);

		uart_reg_out(UART3_EBLR, 2);

		divisor = 48000000 / (41 * speed);	/* Base clock 48 MHz */

		uart_reg_out(UART3_LCR, UART3_LCR_DIVEN);
		uart_reg_out(UART3_DLL, (divisor & 0xff));
		uart_reg_out(UART3_DLH, (divisor >> 8));
		uart_reg_out(UART3_LCR, 0x03);

		if (omap_ir->pdata->transceiver_mode)
			omap_ir->pdata->transceiver_mode(omap_ir->dev,
							IR_MIRMODE);

		local_irq_restore(flags);
	} else {
		local_irq_save(flags);

		/* FIR mode */
		uart_reg_out(UART3_MDR1, UART3_MDR1_FIR |
				UART3_MDR1_SIP_AUTO);

		if (omap_ir->pdata->transceiver_mode)
			omap_ir->pdata->transceiver_mode(omap_ir->dev,
							IR_FIRMODE);

		local_irq_restore(flags);
	}

	omap_ir->speed = speed;

	return 0;
}

#ifdef CONFIG_PM
/*
 * Suspend the IrDA interface.
 */
static int omap_irda_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct omap_irda *omap_ir = netdev_priv(dev);

	if (!dev)
		return 0;

	if (omap_ir->open) {
		/*
		 * Stop the transmit queue
		 */
		netif_device_detach(dev);
		disable_irq(dev->irq);
		omap_irda_shutdown(omap_ir);
	}
	return 0;
}

/*
 * Resume the IrDA interface.
 */
static int omap_irda_resume(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct omap_irda *omap_ir= netdev_priv(dev);

	if (!dev)
		return 0;

	if (omap_ir->open) {
		/*
		 * If we missed a speed change, initialise at the new speed
		 * directly.  It is debatable whether this is actually
		 * required, but in the interests of continuing from where
		 * we left off it is desireable.  The converse argument is
		 * that we should re-negotiate at 9600 baud again.
		 */
		if (omap_ir->newspeed) {
			omap_ir->speed = omap_ir->newspeed;
			omap_ir->newspeed = 0;
		}

		omap_irda_startup(dev);
		omap_irda_set_speed(dev, omap_ir->speed);
		enable_irq(dev->irq);

		/*
		 * This automatically wakes up the queue
		 */
		netif_device_attach(dev);
	}

	return 0;
}
#else
#define omap_irda_suspend	NULL
#define omap_irda_resume	NULL
#endif

static int omap_irda_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct omap_irda *omap_ir;
	struct omap_irda_config *pdata = pdev->dev.platform_data;
	unsigned int baudrate_mask;
	int err = 0;
	int irq = NO_IRQ;

	if (!pdata) {
		printk(KERN_ERR "IrDA Platform data not supplied\n");
		return -ENOENT;
	}

	if (!pdata->rx_channel || !pdata->tx_channel) {
		printk(KERN_ERR "IrDA invalid rx/tx channel value\n");
		return -ENOENT;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		printk(KERN_WARNING "no irq for IrDA\n");
		return -ENOENT;
	}

	dev = alloc_irdadev(sizeof(struct omap_irda));
	if (!dev)
		goto err_mem_1;


	omap_ir = netdev_priv(dev);
	omap_ir->dev = &pdev->dev;
	omap_ir->pdata = pdata;

	dev->hard_start_xmit	= omap_irda_hard_xmit;
	dev->open		= omap_irda_start;
	dev->stop		= omap_irda_stop;
	dev->do_ioctl		= omap_irda_ioctl;
	dev->get_stats		= omap_irda_stats;
	dev->irq		= irq;

	irda_init_max_qos_capabilies(&omap_ir->qos);

	baudrate_mask = 0;
	if (omap_ir->pdata->transceiver_cap & IR_SIRMODE)
		baudrate_mask |= IR_9600|IR_19200|IR_38400|IR_57600|IR_115200;
	if (omap_ir->pdata->transceiver_cap & IR_MIRMODE)
		baudrate_mask |= IR_57600 | IR_1152000;
	if (omap_ir->pdata->transceiver_cap & IR_FIRMODE)
		baudrate_mask |= IR_4000000 << 8;

	omap_ir->qos.baud_rate.bits &= baudrate_mask;
	omap_ir->qos.min_turn_time.bits = 7;

	irda_qos_bits_to_value(&omap_ir->qos);

	/* Any better way to avoid this? No. */
	if (machine_is_omap_h3() || machine_is_omap_h4())
		INIT_DELAYED_WORK(&omap_ir->pdata->gpio_expa, NULL);

	err = register_netdev(dev);
	if (!err)
		platform_set_drvdata(pdev, dev);
	else
		free_netdev(dev);

err_mem_1:
	return err;
}

static int omap_irda_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);

	if (pdev) {
		unregister_netdev(dev);
		free_netdev(dev);
	}
	return 0;
}

static struct platform_driver omapir_driver = {
	.probe		= omap_irda_probe,
	.remove		= omap_irda_remove,
	.suspend	= omap_irda_suspend,
	.resume		= omap_irda_resume,
	.driver		= {
		.name	= "omapirda",
	},
};

static char __initdata banner[] = KERN_INFO "OMAP IrDA driver initializing\n";

static int __init omap_irda_init(void)
{
	printk(banner);
	return platform_driver_register(&omapir_driver);
}

static void __exit omap_irda_exit(void)
{
	platform_driver_unregister(&omapir_driver);
}

module_init(omap_irda_init);
module_exit(omap_irda_exit);

MODULE_AUTHOR("MontaVista");
MODULE_DESCRIPTION("OMAP IrDA Driver");
MODULE_LICENSE("GPL");

