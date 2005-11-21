/*
 * BRIEF MODULE DESCRIPTION
 *
 *	Infra-red driver for the OMAP1610-H2 and OMAP1710-H3 Platforms
 *          (SIR/MIR/FIR modes)
 *          (based on omap-sir.c)
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

#include <linux/config.h>
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

#include <net/irda/irda.h>
#include <net/irda/irmod.h>
#include <net/irda/wrapper.h>
#include <net/irda/irda_device.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/serial.h>
#include <asm/mach-types.h>
#include <asm/dma.h>
#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <linux/i2c.h>

#ifdef CONFIG_MACH_OMAP_H3
#include <asm/arch/gpioexpander.h>
#endif

#define SIR_MODE 0
#define MIR_MODE 1
#define FIR_MODE 2

#define OMAP1610_H2_FIRSEL_GPIO 17

static int rx_state = 0;	/* RX state for IOCTL */

struct omap1610_irda {
	unsigned char open;
	int speed;		/* Current IrDA speed */
	int newspeed;

	struct net_device_stats stats;
	struct irlap_cb *irlap;
	struct qos_info qos;

	int rx_dma_channel;
	int tx_dma_channel;

	dma_addr_t rx_buf_dma_phys;	/* Physical adress of RX DMA buffer */
	dma_addr_t tx_buf_dma_phys;	/* Physical adress of TX DMA buffer */

	void *rx_buf_dma_virt;	/* Virtual adress of RX DMA buffer */
	void *tx_buf_dma_virt;	/* Virtual adress of TX DMA buffer */

	struct device *dev;
};

#define OMAP_IRDA_DEBUG	0

#if (OMAP_IRDA_DEBUG > 0)
#define DBG(format, args...) printk(KERN_ERR "%s(): " format, __FUNCTION__, ## args);
#define DBG_IRQ(format, args...) printk(KERN_ERR "%s(): " format, __FUNCTION__, ## args);
#else
#define DBG(format, args...)
#define DBG_IRQ(format, args...)
#endif

#if (OMAP_IRDA_DEBUG > 1)
#define __ECHO_IN printk(KERN_ERR "%s: enter\n",__FUNCTION__);
#define __ECHO_OUT printk(KERN_ERR "%s: exit\n",__FUNCTION__);
#else
#define __ECHO_IN
#define __ECHO_OUT
#endif

#ifdef OMAP1610_IR_HARDWARE_DEBUG_ENABLE
#define HDBG_DELAY 200

void hard_debug1(u16 i)
{
	for (; i; i--) {
		omap_writew(0x2000,
			    OMAP1610_GPIO1_BASE + OMAP1610_GPIO_CLEAR_DATAOUT);
		udelay(HDBG_DELAY);

		omap_writew(0x2000,
			    OMAP1610_GPIO1_BASE + OMAP1610_GPIO_SET_DATAOUT);
		udelay(HDBG_DELAY);
	}
}

void hard_debug2(u16 i)
{
	for (; i; i--) {
		omap_writew(0x8000,
			    OMAP1610_GPIO1_BASE + OMAP1610_GPIO_CLEAR_DATAOUT);
		udelay(HDBG_DELAY);

		omap_writew(0x8000,
			    OMAP1610_GPIO1_BASE + OMAP1610_GPIO_SET_DATAOUT);
		udelay(HDBG_DELAY);
	}
}

#define HDBG1(i) hard_debug1(i)
#define HDBG2(i) hard_debug2(i)
#else
#define HDBG1(i)
#define HDBG2(i)
#endif

/* forward declarations */

extern void irda_device_setup(struct net_device *dev);
extern void omap_stop_dma(int lch);
static int omap1610_irda_set_speed(struct net_device *dev, int speed);

static void omap1610_irda_start_rx_dma(struct omap1610_irda *si)
{
	/* Configure DMA */
	omap_set_dma_src_params(si->rx_dma_channel, 0x3, 0x0, (unsigned long)UART3_RHR,
				0, 0);

	omap_enable_dma_irq(si->rx_dma_channel, 0x01);

	omap_set_dma_dest_params(si->rx_dma_channel, 0x0, 0x1,
				 si->rx_buf_dma_phys,
				 0, 0);

	omap_set_dma_transfer_params(si->rx_dma_channel, 0x0, 4096, 0x1, 0x0, 0, 0);

	omap_start_dma(si->rx_dma_channel);
}

static void omap1610_start_tx_dma(struct omap1610_irda *si, int size)
{
	__ECHO_IN;

	/* Configure DMA */
	omap_set_dma_dest_params(si->tx_dma_channel, 0x03, 0x0, (unsigned long)UART3_THR,
					0, 0);
	omap_enable_dma_irq(si->tx_dma_channel, 0x01);

	omap_set_dma_src_params(si->tx_dma_channel, 0x0, 0x1,
				si->tx_buf_dma_phys,
				0, 0);

	omap_set_dma_transfer_params(si->tx_dma_channel, 0x0, size, 0x1, 0x0, 0, 0);

	HDBG1(1);

	/* Start DMA */
	omap_start_dma(si->tx_dma_channel);

	HDBG1(1);

	__ECHO_OUT;
}

/* DMA RX callback - normally, we should not go here, 
   it calls only if something is going wrong
 */

static void omap1610_irda_rx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct net_device *dev = data;
	struct omap1610_irda *si = dev->priv;

	printk(KERN_ERR "RX Transfer error or very big frame \n");

	/* Clear interrupts */
	omap_readb(UART3_IIR);

	si->stats.rx_frame_errors++;

	omap_readb(UART3_RESUME);

	/* Re-init RX DMA */
	omap1610_irda_start_rx_dma(si);

}

/* DMA TX callback - calling when frame transfer has been finished */

static void omap1610_irda_tx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct net_device *dev = data;
	struct omap1610_irda *si = dev->priv;

	__ECHO_IN;

	/*Stop DMA controller */
	omap_stop_dma(si->tx_dma_channel);

	__ECHO_OUT;

}

/*
 * Set the IrDA communications speed.
 * Interrupt have to be disabled here.
 */

static int omap1610_irda_startup(struct net_device *dev)
{
	__ECHO_IN;

	/* Enable UART3 clock and set UART3 to IrDA mode */
	omap_writel(omap_readl(MOD_CONF_CTRL_0) | (1 << 31) | (1 << 15),
		    MOD_CONF_CTRL_0);

	if (machine_is_omap_h2()) {
//              omap_cfg_reg(Y15_1610_GPIO17);
		omap_writel(omap_readl(FUNC_MUX_CTRL_A) | 7, FUNC_MUX_CTRL_A);

		omap_set_gpio_direction(OMAP1610_H2_FIRSEL_GPIO, 0);
		omap_set_gpio_dataout(OMAP1610_H2_FIRSEL_GPIO, 0);
	}

	omap_writeb(0x07, UART3_MDR1);	/* Put UART3 in reset mode */

	/* Clear DLH and DLL */
	omap_writeb(1 << 7, UART3_LCR);

	omap_writeb(0, UART3_DLL);
	omap_writeb(0, UART3_DLH);

	omap_writeb(0xbf, UART3_LCR);

	omap_writeb(1 << 4, UART3_EFR);

	omap_writeb(1 << 7, UART3_LCR);

	/* Enable access to UART3_TLR and UART3_TCR registers */
	omap_writeb(1 << 6, UART3_MCR);

	omap_writeb(0, UART3_SCR);

	/* Set Rx trigger to 1 and Tx trigger to 1 */
	omap_writeb(0, UART3_TLR);

	/* Set LCR to 8 bits and 1 stop bit */
	omap_writeb(0x03, UART3_LCR);

	/* Clear RX and TX FIFO and enable FIFO */
	/* Use DMA Req for transfers */

	omap_writeb((1 << 2) | (1 << 1) | (1 << 3) | (1 << 4) | (1 << 6) | 1,
		    UART3_FCR);

	omap_writeb(0, UART3_MCR);

	omap_writeb((1 << 7) | (1 << 6), UART3_SCR);

	/* Enable UART3 SIR Mode,(Frame-length method to end frames) */
	omap_writeb(1, UART3_MDR1);

	/* Set Status FIFO trig to 1 */
	omap_writeb(0, UART3_MDR2);

	/* Enables RXIR input */
	/* and disable TX underrun */
	/* SEND_SIP pulse */

	//   omap_writeb((1 << 7) | (1 << 6) | (1 << 4), UART3_ACREG);
	omap_writeb((1 << 6) | (1 << 4), UART3_ACREG);

	/* Enable EOF Interrupt only */
	omap_writeb((1 << 7) | (1 << 5), UART3_IER);

	/* Set Maximum Received Frame size to 2048 bytes */
	omap_writeb(0x00, UART3_RXFLL);
	omap_writeb(0x08, UART3_RXFLH);

	omap_readb(UART3_RESUME);

	__ECHO_OUT;

	return 0;

}

static int omap1610_irda_shutdown(struct omap1610_irda *si)
{
	/* Disable all UART3 Interrupts */
	omap_writeb(0, UART3_IER);

	/* Disable UART3 and disable baud rate generator */
	omap_writeb(0x07, UART3_MDR1);	/* Put UART3 in reset mode */

	omap_writeb((1 << 5), UART3_ACREG);	/* set SD_MODE pin to high and Disable RX IR */

	/* Clear DLH and DLL */
	omap_writeb(1 << 7, UART3_LCR);
	omap_writeb(0, UART3_DLL);
	omap_writeb(0, UART3_DLH);

	return 0;
}

static irqreturn_t
omap1610_irda_irq(int irq, void *dev_id, struct pt_regs *hw_regs)
{
	struct net_device *dev = dev_id;
	struct omap1610_irda *si = dev->priv;
	struct sk_buff *skb;

	u8 status;
	int w = 0;

	__ECHO_IN;

	/* Clear EOF interrupt */
	status = omap_readb(UART3_IIR);

	if (status & (1 << 5)) {
		u8 mdr2 = omap_readb(UART3_MDR2);
		HDBG1(2);
		if (mdr2 & 1)
			printk(KERN_ERR "IRDA Buffer underrun error");

		si->stats.tx_packets++;

		if (si->newspeed) {
			omap1610_irda_set_speed(dev, si->newspeed);
			si->newspeed = 0;
		}

		netif_wake_queue(dev);

		if (!(status & 0x80))
			return IRQ_HANDLED;
	}

	/* Stop DMA and if there are no errors, send frame to upper layer */

	omap_stop_dma(si->rx_dma_channel);

	status = omap_readb(UART3_SFLSR);	/* Take a frame status */

	if (status != 0) {	/* Bad frame? */
		si->stats.rx_frame_errors++;
		omap_readb(UART3_RESUME);
	} else {
		/* We got a frame! */
		skb = alloc_skb(4096, GFP_ATOMIC);

		if (!skb) {
			printk(KERN_ERR "omap_sir: out of memory for RX SKB\n");
			return IRQ_HANDLED;
		}
		/*
		 * Align any IP headers that may be contained
		 * within the frame.
		 */

		skb_reserve(skb, 1);

		w = OMAP_DMA_CDAC_REG(si->rx_dma_channel);
		w -= OMAP1_DMA_CDSA_L_REG(si->rx_dma_channel);

		if (si->speed != 4000000) {
			memcpy(skb_put(skb, w - 2), si->rx_buf_dma_virt, w - 2);	/* Copy DMA buffer to skb */
		} else {
			memcpy(skb_put(skb, w - 4), si->rx_buf_dma_virt, w - 4);	/* Copy DMA buffer to skb */
		}

		skb->dev = dev;
		skb->mac.raw = skb->data;
		skb->protocol = htons(ETH_P_IRDA);
		si->stats.rx_packets++;
		si->stats.rx_bytes += skb->len;
		netif_receive_skb(skb);	/* Send data to upper level */
	}

	/* Re-init RX DMA */
	omap1610_irda_start_rx_dma(si);

	dev->last_rx = jiffies;

	__ECHO_OUT;

	return IRQ_HANDLED;
}

static int omap1610_irda_hard_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct omap1610_irda *si = dev->priv;
	int speed = irda_get_next_speed(skb);
	int mtt = irda_get_mtt(skb);
	int xbofs = irda_get_next_xbofs(skb);

	__ECHO_IN;

	/*
	 * Does this packet contain a request to change the interface
	 * speed?  If so, remember it until we complete the transmission
	 * of this frame.
	 */
	if (speed != si->speed && speed != -1)
		si->newspeed = speed;

	if (xbofs) {
		/* Set number of addtional BOFS */
		omap_writeb(xbofs + 1, UART3_EBLR);
	}

	/*
	 * If this is an empty frame, we can bypass a lot.
	 */
	if (skb->len == 0) {
		if (si->newspeed) {
			si->newspeed = 0;
			omap1610_irda_set_speed(dev, speed);
		}
		dev_kfree_skb(skb);
		return 0;
	}

	netif_stop_queue(dev);

	/* Copy skb data to DMA buffer */

	memcpy(si->tx_buf_dma_virt, skb->data, skb->len);

	si->stats.tx_bytes += skb->len;

	/* Set frame length */

	omap_writeb((skb->len & 0xff), UART3_TXFLL);
	omap_writeb((skb->len >> 8), UART3_TXFLH);

	if (mtt > 1000)
		mdelay(mtt / 1000);
	else
		udelay(mtt);

	/* Start TX DMA transfer */

	omap1610_start_tx_dma(si, skb->len);

	/* We can free skb now because it's already in DMA buffer */

	dev_kfree_skb(skb);

	dev->trans_start = jiffies;

	__ECHO_OUT;

	return 0;
}

static int
omap1610_irda_ioctl(struct net_device *dev, struct ifreq *ifreq, int cmd)
{
	struct if_irda_req *rq = (struct if_irda_req *)ifreq;
	struct omap1610_irda *si = dev->priv;
	int ret = -EOPNOTSUPP;

	__ECHO_IN;

	switch (cmd) {
	case SIOCSBANDWIDTH:
		if (capable(CAP_NET_ADMIN)) {
			/*
			 * We are unable to set the speed if the
			 * device is not running.
			 */
			if (si->open) {
				ret =
				    omap1610_irda_set_speed(dev,
							    rq->ifr_baudrate);
			} else {
				printk
				    (KERN_ERR
				     "omap_irda_ioctl: SIOCSBANDWIDTH: !netif_running\n");
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
		rq->ifr_receiving = rx_state;
		break;

	default:
		break;
	}

	__ECHO_OUT;

	return ret;
}

static struct net_device_stats *omap1610_irda_stats(struct net_device *dev)
{
	struct omap1610_irda *si = dev->priv;
	return &si->stats;
}

static int omap1610_irda_start(struct net_device *dev)
{
	struct omap1610_irda *si = dev->priv;
	int err;
	unsigned long flags = 0;

#ifdef CONFIG_MACH_OMAP_H3
	u8 ioExpanderVal = 0;
#endif

	__ECHO_IN;
	si->speed = 9600;

	err = request_irq(dev->irq, omap1610_irda_irq, 0, dev->name, dev);
	if (err)
		goto err_irq;

	/*
	 * The interrupt must remain disabled for now.
	 */

	disable_irq(dev->irq);

	/*  Request DMA channels for IrDA hardware */

	if (omap_request_dma(OMAP_DMA_UART3_RX, "IrDA Rx DMA",
			     (void *)omap1610_irda_rx_dma_callback,
			     dev, &(si->rx_dma_channel))) {
		printk(KERN_ERR "Failed to request IrDA Rx DMA \n");
		goto err_irq;
	}

	if (omap_request_dma(OMAP_DMA_UART3_TX, "IrDA Tx DMA",
			     (void *)omap1610_irda_tx_dma_callback,
			     dev, &(si->tx_dma_channel))) {
		printk(KERN_ERR "Failed to request IrDA Tx DMA \n");
		goto err_irq;
	}

	/* Allocate TX and RX buffers for DMA channels */

	si->rx_buf_dma_virt =
	    dma_alloc_coherent(NULL, 4096, &(si->rx_buf_dma_phys), flags);

	si->tx_buf_dma_virt =
	    dma_alloc_coherent(NULL, 4096, &(si->tx_buf_dma_phys), flags);

	/*
	 * Setup the serial port for the specified config.
	 */

#ifdef CONFIG_MACH_OMAP_H3

	if ((err = read_gpio_expa(&ioExpanderVal, 0x26))) {
		printk(KERN_ERR "Error reading from I/O EXPANDER \n");
		return err;
	}

	ioExpanderVal |= 0x40;	/* 'P6' Enable IRDA_TX and IRDA_RX */

	if ((err = write_gpio_expa(ioExpanderVal, 0x26))) {
		printk(KERN_ERR "Error writing to I/O EXPANDER \n");
		return err;
	}
#endif
	err = omap1610_irda_startup(dev);

	if (err)
		goto err_startup;

	omap1610_irda_set_speed(dev, si->speed = 9600);

	/*
	 * Open a new IrLAP layer instance.
	 */

	si->irlap = irlap_open(dev, &si->qos, "omap_sir");

	err = -ENOMEM;
	if (!si->irlap)
		goto err_irlap;

	/* Now enable the interrupt and start the queue  */
	si->open = 1;

	/* Start RX DMA */

	omap1610_irda_start_rx_dma(si);

	enable_irq(dev->irq);
	netif_start_queue(dev);

	__ECHO_OUT;

	return 0;

      err_irlap:
	si->open = 0;
	omap1610_irda_shutdown(si);
      err_startup:
      err_irq:
	free_irq(dev->irq, dev);
	return err;
}

static int omap1610_irda_stop(struct net_device *dev)
{
	struct omap1610_irda *si = dev->priv;

	__ECHO_IN;

	disable_irq(dev->irq);

	netif_stop_queue(dev);

	omap_free_dma(si->rx_dma_channel);
	omap_free_dma(si->tx_dma_channel);

	dma_free_coherent(NULL, 4096, si->rx_buf_dma_virt, si->rx_buf_dma_phys);
	dma_free_coherent(NULL, 4096, si->tx_buf_dma_virt, si->tx_buf_dma_phys);

	omap1610_irda_shutdown(si);

	/* Stop IrLAP */
	if (si->irlap) {
		irlap_close(si->irlap);
		si->irlap = NULL;
	}

	si->open = 0;

	/*
	 * Free resources
	 */

	free_irq(dev->irq, dev);

	__ECHO_OUT;

	return 0;
}

#ifdef CONFIG_MACH_OMAP_H3

static void set_h3_gpio_expa(u8 FIR_SEL, u8 IrDA_INVSEL)
{
	u8 ioExpanderVal = 0;

	if (read_gpio_expa(&ioExpanderVal, 0x27) != 0) {
		printk(KERN_ERR "Error reading from I/O EXPANDER \n");
		return;
	}

	ioExpanderVal &= ~0x03;
	ioExpanderVal |= FIR_SEL << 1;
	ioExpanderVal |= IrDA_INVSEL << 0;

	if (write_gpio_expa(ioExpanderVal, 0x27) != 0) {
		printk(KERN_ERR "Error writing to I/O EXPANDER \n");
		return;
	}
	if (read_gpio_expa(&ioExpanderVal, 0x27) != 0) {
		printk(KERN_ERR "Error reading from I/O EXPANDER \n");
		return;
	}
}

int which_speed;

static void set_h3_gpio_expa_handler(void *data)
{
	int *mode = data;

	if (*mode == SIR_MODE)
		set_h3_gpio_expa(0, 1);
	else if (*mode == MIR_MODE)
		set_h3_gpio_expa(1, 1);
	else if (*mode == FIR_MODE)
		set_h3_gpio_expa(1, 1);
}

DECLARE_WORK(set_h3_gpio_expa_work, &set_h3_gpio_expa_handler, &which_speed);

static inline void set_h3_irda_mode(int mode)
{
	cancel_delayed_work(&set_h3_gpio_expa_work);
	which_speed = mode;
	schedule_work(&set_h3_gpio_expa_work);
}
#else
#define set_h3_irda_mode(x)
#endif

static int omap1610_irda_set_speed(struct net_device *dev, int speed)
{
	struct omap1610_irda *si = dev->priv;
	int divisor;

	__ECHO_IN;

	/* Set IrDA speed */
	if (speed <= 115200) {
		/* SIR mode */
		if (machine_is_omap_h2()) {
			omap_set_gpio_dataout(OMAP1610_H2_FIRSEL_GPIO, 0);
		}

		if (machine_is_omap_h3())
			set_h3_irda_mode(SIR_MODE);

		printk("Set SIR Mode! Speed: %d\n", speed);

		omap_writeb(1, UART3_MDR1);	/* Set SIR mode */

		omap_writeb(1, UART3_EBLR);

		divisor = 48000000 / (16 * speed);	/* Base clock 48 MHz */

		HDBG2(1);
		omap_writeb(1 << 7, UART3_LCR);

		omap_writeb((divisor & 0xFF), UART3_DLL);

		omap_writeb((divisor >> 8), UART3_DLH);

		omap_writeb(0x03, UART3_LCR);

		omap_writeb(0, UART3_MCR);

		HDBG2(1);

	} else if (speed <= 1152000) {
		/* MIR mode */
		printk("Set MIR Mode! Speed: %d\n", speed);

		omap_writeb((1 << 2) | (1 << 6), UART3_MDR1);	/* Set MIR mode with 
								   SIP after each frame */

		omap_writeb(2, UART3_EBLR);

		divisor = 48000000 / (41 * speed);	/* Base clock 48 MHz */

		omap_writeb(1 << 7, UART3_LCR);

		omap_writeb((divisor & 0xFF), UART3_DLL);

		omap_writeb((divisor >> 8), UART3_DLH);

		omap_writeb(0x03, UART3_LCR);

		if (machine_is_omap_h2())
			omap_set_gpio_dataout(OMAP1610_H2_FIRSEL_GPIO, 1);

		if (machine_is_omap_h3())
			set_h3_irda_mode(MIR_MODE);

	} else {
		/* FIR mode */

		printk("Set FIR Mode! Speed: %d\n", speed);

		omap_writeb((1 << 2) | (1 << 6) | 1, UART3_MDR1);	/* Set FIR mode
									   with SIP after each frame */
		if (machine_is_omap_h2())
			omap_set_gpio_dataout(OMAP1610_H2_FIRSEL_GPIO, 1);

		if (machine_is_omap_h3())
			set_h3_irda_mode(FIR_MODE);
	}

	si->speed = speed;

	__ECHO_OUT;

	return 0;

}

#ifdef CONFIG_PM
/*
 * Suspend the IrDA interface.
 */
static int omap1610_irda_suspend(struct device *_dev, u32 state, u32 level)
{
	struct net_device *dev = dev_get_drvdata(_dev);
	struct omap1610_irda *si = dev->priv;

	if (!dev || level != SUSPEND_DISABLE)
		return 0;

	if (si->open) {
		/*
		 * Stop the transmit queue
		 */
		netif_device_detach(dev);
		disable_irq(dev->irq);
		omap1610_irda_shutdown(si);
	}
	return 0;
}

/*
 * Resume the IrDA interface.
 */
static int omap1610_irda_resume(struct device *_dev, u32 level)
{
	struct net_device *dev = dev_get_drvdata(_dev);
	struct omap1610_irda *si= dev->priv;

	if (!dev || level != RESUME_ENABLE)
		return 0;

	if (si->open) {
		/*
		 * If we missed a speed change, initialise at the new speed
		 * directly.  It is debatable whether this is actually
		 * required, but in the interests of continuing from where
		 * we left off it is desireable.  The converse argument is
		 * that we should re-negotiate at 9600 baud again.
		 */
		if (si->newspeed) {
			si->speed = si->newspeed;
			si->newspeed = 0;
		}

		omap1610_irda_startup(dev);
		omap1610_irda_set_speed(dev, si->speed);
		enable_irq(dev->irq);

		/*
		 * This automatically wakes up the queue
		 */
		netif_device_attach(dev);
	}

	return 0;
}
#else
#define omap1610_irda_suspend	NULL
#define omap1610_irda_resume	NULL
#endif

static int omap1610_irda_probe(struct device *_dev)
{
	struct platform_device *pdev = to_platform_device(_dev);
	struct net_device *dev;
	struct omap1610_irda *si;
	unsigned int baudrate_mask;
	int err = 0;

	dev = alloc_irdadev(sizeof(struct omap1610_irda));
	if (!dev)
		goto err_mem_1;

	si = dev->priv;
	si->dev = &pdev->dev;
	dev->hard_start_xmit = omap1610_irda_hard_xmit;
	dev->open = omap1610_irda_start;
	dev->stop = omap1610_irda_stop;
	dev->do_ioctl = omap1610_irda_ioctl;
	dev->get_stats = omap1610_irda_stats;
	dev->irq = INT_UART3;

	irda_init_max_qos_capabilies(&si->qos);

	/*
	 *  OMAP1610  supports SIR, MIR, FIR modes,
	 *  but actualy supported modes depend on hardware implementation.
	 *  OMAP1610 Innovator supports only SIR and 
	 *  OMAP1610 H2 supports both SIR and FIR
	 */

	baudrate_mask =
	    IR_9600 | IR_19200 | IR_38400 | IR_57600 | IR_115200 | IR_576000 |
	    IR_1152000;

	if (machine_is_omap_h2() || machine_is_omap_h3()) {

		baudrate_mask |= (IR_4000000 << 8);
	}

	si->qos.baud_rate.bits &= baudrate_mask;
	si->qos.min_turn_time.bits = 7;

	irda_qos_bits_to_value(&si->qos);

	err = register_netdev(dev);
	if (!err)
		dev_set_drvdata(&pdev->dev, dev);
	else 
		free_netdev(dev);

      err_mem_1:
	return err;
}

static int omap1610_irda_remove(struct device *_dev)
{
	struct net_device *dev = dev_get_drvdata(_dev);
	
#ifdef CONFIG_MACH_OMAP_H3
	if (machine_is_omap_h3())
		cancel_delayed_work(&set_h3_gpio_expa_work);
#endif
	if (dev) {
		unregister_netdev(dev);
		free_netdev(dev);
	}
	return 0;
}

static struct device_driver omap1610ir_driver = {
	.name = "omap1610-ir",
	.bus = &platform_bus_type,
	.probe = omap1610_irda_probe,
	.remove = omap1610_irda_remove,
	.suspend = omap1610_irda_suspend,
	.resume = omap1610_irda_resume,
};

static int __init omap1610_irda_init(void)
{
	return driver_register(&omap1610ir_driver);

}

static void __exit omap1610_irda_exit(void)
{
	driver_unregister(&omap1610ir_driver);
}

module_init(omap1610_irda_init);
module_exit(omap1610_irda_exit);

MODULE_AUTHOR("MontaVista");
MODULE_DESCRIPTION("OMAP IrDA Driver");
MODULE_LICENSE("GPL");

