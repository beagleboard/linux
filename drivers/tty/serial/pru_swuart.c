// SPDX-License-Identifier: GPL-2.0-only
/*
 * UART driver for PRU software based UART (PRU SWUART)
 *
 * Copyright (C) 2019-2021 Texas Instruments Incorporated - https://www.ti.com/
 * Author: Bin Liu <b-liu@ti.com>
 */

#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pruss.h>
#include <linux/remoteproc.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>

#define PSUART_NAME		"ttySPRU"

#define PSUART_FW_MAGIC_NUMBER	0x54524155	/* "UART" */
#define MAX_UART_PORTS		3
#define DRV_TOTAL_PORTS		(MAX_UART_PORTS * 2)	/* 2 PRUs */
#define PORT_MMR_BASE		0x14
#define PORT_MMR_LEN		0x14
#define FIFO_BASE		0x100
#define FIFO_SIZE		256
#define BPC			2	/* bytes per char */

/* hw flow control threshold */
#define RX_FIFO_THRES_SHIFT	16
#define RX_FIFO_THRES		((FIFO_SIZE - 16) << RX_FIFO_THRES_SHIFT)
#define RX_FIFO_THRES_MASK	(0xff << RX_FIFO_THRES_SHIFT)

/* global registers */
#define PSUART_FW_MAGIC		0x00
#define PSUART_FW_VERSION		0x08
#define PSUART_FW_GCFG			0x50
#define PSUART_FW_INITED		BIT(1)

/* uart port registers */
#define PPORT_ENABLE		0x00
#define PPORT_STATUS		0x01
#define PPORT_ENABLED		((BIT(1)) | (BIT(0)))

#define PPORT_CFG		0x04
#define PPORT_FIFO_POS		0x08
#define PPORT_TXFIFO_POS	0x08	/* 16-bit register */
#define PPORT_TXFIFO_WRITE	0x09	/* 8-bit register */
#define PPORT_RXFIFO_POS	0x0a	/* 16-bit register */
#define PPORT_RXFIFO_READ	0x0a	/* 8-bit register */
#define PPORT_TX_CFG		0x0c
#define PPORT_TX_INTR_CTRL	0x0d	/* 8-bit register */
#define PPORT_RX_CFG		0x10
#define PPORT_RX_INTR_CTRL	0x11	/* 8-bit register */

/* PPORT_CFG register bits */
#define PPORT_CFG_HWFLOW_EN	BIT(12)
#define PPORT_CFG_PARADD	BIT(10)
#define PPORT_CFG_PAR_EN	BIT(9)
#define PPORT_CFG_CSTOPB	BIT(8)

#define PPORT_CFG_CS6		0x1
#define PPORT_CFG_CS7		0x2
#define PPORT_CFG_CS8		0x3
#define PPORT_CFG_CSSHIFT	4

#define PPORT_CFG_B600		0x1
#define PPORT_CFG_B1200		0x2
#define PPORT_CFG_B2400		0x3
#define PPORT_CFG_B4800		0x4
#define PPORT_CFG_B9600		0x5
#define PPORT_CFG_B19200	0x7
#define PPORT_CFG_B38400	0x9
#define PPORT_CFG_B57600	0xa
#define PPORT_CFG_B115200	0xb
#define PPORT_CFG_BSHIFT	0
#define PPORT_MAX_BAUD		115200

/* rx character info bits */
#define PPORT_RX_CHAR_PE	BIT(15)
#define PPORT_RX_CHAR_FE	BIT(14)

struct psuart_port {
	struct uart_port port;
	void __iomem *mbase;
	void __iomem *tx_fifo;
	void __iomem *rx_fifo;
};

struct pru_swuart {
	struct device *dev;
	struct rproc *pru;
	struct pruss *pruss;
	enum pruss_pru_id pru_id;
	struct pruss_mem_region mem;
};

struct pport_pins {
	u8 tx;
	u8 rx;
	u8 cts;
	u8 rts;
};

union fifo_pos {
	u16 pos;
	struct {
		u8 tail;	/* read pointer */
		u8 head;	/* write pointer */
	} s;
};

static struct psuart_port pports[DRV_TOTAL_PORTS];

static inline struct psuart_port *up_to_pport(struct uart_port *up)
{
	return  container_of(up, struct psuart_port, port);
}

static inline u32 psuart_readl(struct pru_swuart *pu, u32 reg)
{
	return readl(pu->mem.va + reg);
}

static inline void psuart_writel(struct pru_swuart *pu, u32 reg, u32 val)
{
	writel(val, pu->mem.va + reg);
}

static inline u8 pport_readb(struct psuart_port *pp, u32 reg)
{
	return readb(pp->mbase + reg);
}

static inline void pport_writeb(struct psuart_port *pp, u32 reg, u8 val)
{
	writeb(val, pp->mbase + reg);
}

static inline u16 pport_readw(struct psuart_port *pp, u32 reg)
{
	return readw(pp->mbase + reg);
}

static inline u32 pport_readl(struct psuart_port *pp, u32 reg)
{
	return readl(pp->mbase + reg);
}

static inline void pport_writel(struct psuart_port *pp, u32 reg, u32 val)
{
	writel(val, pp->mbase + reg);
}

static inline int pport_is_fifo_empty(union fifo_pos *pos)
{
	return pos->s.head == pos->s.tail;
}

static void pport_rx_chars(struct psuart_port *pp)
{
	struct uart_port *up = &pp->port;
	union fifo_pos fifo;
	u16 ch;
	int i, total;

	fifo.pos = pport_readw(pp, PPORT_RXFIFO_POS);
	total = CIRC_CNT(fifo.s.head, fifo.s.tail, FIFO_SIZE) / BPC;
	if (!total)
		return;

	for (i = 0; i < total; i++) {
		ch = readw(pp->rx_fifo + fifo.s.tail);
		fifo.s.tail += BPC;

		if (ch & PPORT_RX_CHAR_PE)
			up->icount.parity++;
		if (ch & PPORT_RX_CHAR_FE)
			up->icount.frame++;

		uart_insert_char(up, 0, 0, ch, TTY_NORMAL);
	}

	up->icount.rx += total;
	pport_writeb(pp, PPORT_RXFIFO_READ, fifo.s.tail);
	tty_flip_buffer_push(&up->state->port);
}

static void pport_tx_chars(struct psuart_port *pp)
{
	struct uart_port *up = &pp->port;
	union fifo_pos fifo;
	struct circ_buf *xmit = &up->state->xmit;
	int count;

	fifo.pos = pport_readw(pp, PPORT_TXFIFO_POS);
	count = CIRC_SPACE(fifo.s.head, fifo.s.tail, FIFO_SIZE) / BPC;
	if (!count)
		return;

	if (up->x_char) {
		writew(up->x_char, pp->tx_fifo + fifo.s.head);
		fifo.s.head += BPC;
		pport_writeb(pp, PPORT_TXFIFO_WRITE, fifo.s.head);
		up->icount.tx++;
		up->x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(up))
		return;

	do {
		writew(xmit->buf[xmit->tail], pp->tx_fifo + fifo.s.head);
		fifo.s.head += BPC;
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	pport_writeb(pp, PPORT_TXFIFO_WRITE, fifo.s.head);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(up);
}

static irqreturn_t pport_handle_irq(int irq, void *pru_port)
{
	struct psuart_port *pp = (struct psuart_port *)pru_port;
	int rx, tx;

	rx = pport_readb(pp, PPORT_RX_INTR_CTRL);
	if (rx) {
		pport_writeb(pp, PPORT_RX_INTR_CTRL, 0);
		pport_rx_chars(pp);
	}

	tx = pport_readb(pp, PPORT_TX_INTR_CTRL);
	if (tx) {
		pport_tx_chars(pp);
		pport_writeb(pp, PPORT_TX_INTR_CTRL, 1);
	}

	if (rx)
		pport_writeb(pp, PPORT_RX_INTR_CTRL, 1);

	return IRQ_HANDLED;
}

static unsigned int pport_tx_empty(struct uart_port *up)
{
	struct psuart_port *pp = up_to_pport(up);
	union fifo_pos tx;

	tx.pos = pport_readw(pp, PPORT_TXFIFO_POS);

	return pport_is_fifo_empty(&tx) ? TIOCSER_TEMT : 0;
}

static unsigned int pport_get_mctrl(struct uart_port *up)
{
	return up->mctrl;
}

/* the hardware flow control doesn't require any software assistance */
static void pport_set_mctrl(struct uart_port *up, unsigned int mctrl)
{
};

static void pport_stop_tx(struct uart_port *up)
{
	struct psuart_port *pp = up_to_pport(up);

	pport_writeb(pp, PPORT_TX_INTR_CTRL, 0);
}

static void pport_start_tx(struct uart_port *up)
{
	struct psuart_port *pp = up_to_pport(up);

	pport_writeb(pp, PPORT_TX_INTR_CTRL, 0);
	pport_tx_chars(pp);
	pport_writeb(pp, PPORT_TX_INTR_CTRL, 1);
}

static void pport_stop_rx(struct uart_port *up)
{
	struct psuart_port *pp = up_to_pport(up);

	pport_writeb(pp, PPORT_RX_INTR_CTRL, 0);
}

static void pport_start_rx(struct uart_port *up)
{
	struct psuart_port *pp = up_to_pport(up);

	pport_writeb(pp, PPORT_RX_INTR_CTRL, 0);
	pport_rx_chars(pp);
	pport_writeb(pp, PPORT_RX_INTR_CTRL, 1);
}

static void pport_throttle(struct uart_port *up)
{
	pport_stop_rx(up);
}

static void pport_unthrottle(struct uart_port *up)
{
	pport_start_rx(up);
}

/* line break is not supported */
static void pport_break_ctl(struct uart_port *up, int break_state)
{
}

/* software flow control currently not supported */
static void pport_set_termios(struct uart_port *up, struct ktermios *termios,
			struct ktermios *old)
{
	struct psuart_port *pp = up_to_pport(up);
	tcflag_t cflag;
	unsigned int baud;
	u32 cfg;

	/* reset all fields except hw flow control settings */
	cfg = pport_readl(pp, PPORT_CFG);
	cfg &= RX_FIFO_THRES_MASK;

	cflag = termios->c_cflag;
	switch (cflag & CSIZE) {
	case CS5:
		break;
	case CS6:
		cfg |= (PPORT_CFG_CS6 << PPORT_CFG_CSSHIFT);
		break;
	case CS7:
		cfg |= (PPORT_CFG_CS7 << PPORT_CFG_CSSHIFT);
		break;
	case CS8:
	default:
		cfg |= (PPORT_CFG_CS8 << PPORT_CFG_CSSHIFT);
		break;
	}

	if (cflag & PARENB) {
		cfg |= PPORT_CFG_PAR_EN;
		cfg |= (cflag & PARODD) ? PPORT_CFG_PARADD : 0;
	}

	cfg |= (cflag & CSTOPB) ? PPORT_CFG_CSTOPB : 0;

	if (cflag & CRTSCTS) {
		cfg |= PPORT_CFG_HWFLOW_EN;
		/*
		 * Setting TIOCM_CTS here to prevent core uart_change_speed()
		 * calls ops->stop_tx() when hw flow control is enabled.
		 */
		up->mctrl |= TIOCM_CTS;
	} else {
		up->mctrl &= ~TIOCM_CTS;
	}

	switch (cflag & CBAUD) {
	case B300:
		break;
	case B600:
		cfg |= (PPORT_CFG_B600 << PPORT_CFG_BSHIFT);
		break;
	case B1200:
		cfg |= (PPORT_CFG_B1200 << PPORT_CFG_BSHIFT);
		break;
	case B2400:
		cfg |= (PPORT_CFG_B2400 << PPORT_CFG_BSHIFT);
		break;
	case B4800:
		cfg |= (PPORT_CFG_B4800 << PPORT_CFG_BSHIFT);
		break;
	case B9600:
	default:
		cfg |= (PPORT_CFG_B9600 << PPORT_CFG_BSHIFT);
		break;
	case B19200:
		cfg |= (PPORT_CFG_B19200 << PPORT_CFG_BSHIFT);
		break;
	case B38400:
		cfg |= (PPORT_CFG_B38400 << PPORT_CFG_BSHIFT);
		break;
	case B57600:
		cfg |= (PPORT_CFG_B57600 << PPORT_CFG_BSHIFT);
		break;
	case B115200:
		cfg |= (PPORT_CFG_B115200 << PPORT_CFG_BSHIFT);
		break;
	}

	baud = uart_get_baud_rate(up, termios, old, 0, PPORT_MAX_BAUD);
	uart_update_timeout(up, cflag, baud);

	pport_writeb(pp, PPORT_ENABLE, 0);
	pport_writel(pp, PPORT_CFG, cfg);
	pport_writeb(pp, PPORT_ENABLE, 1);
}

static int pport_startup(struct uart_port *up)
{
	struct psuart_port *pp = up_to_pport(up);
	int timeout = 100;

	if (up->flags & UPF_HARD_FLOW) {
		/*
		 * CTS is a input-only pin in the firmware, so AUTOCTS is
		 * not supported.
		 */
		up->status |= UPSTAT_AUTORTS;
		pport_writel(pp, PPORT_CFG, RX_FIFO_THRES);
	}

	pport_writeb(pp, PPORT_ENABLE, 1);
	while (!(pport_readb(pp, PPORT_STATUS) & PPORT_ENABLED)) {
		if (--timeout < 0) {
			dev_err(up->dev, "failed to enable port\n");
			return 1;
		}
	}

	pport_start_rx(up);
	return 0;
};

static void pport_shutdown(struct uart_port *up)
{
	struct psuart_port *pp = up_to_pport(up);

	pport_writeb(pp, PPORT_ENABLE, 0);
};

static void pport_config_port(struct uart_port *up, int flags)
{
	up->type = PORT_PSUART;
	up->flags |= UPF_HARD_FLOW;
}

/* rs485 is unsupported */
static int pport_rs485_config(struct uart_port *up, struct serial_rs485 *rs485)
{
	return rs485->flags & SER_RS485_ENABLED ? -EOPNOTSUPP : 0;
}

static const struct uart_ops psuart_port_ops = {
	.tx_empty	= pport_tx_empty,
	.get_mctrl	= pport_get_mctrl,
	.set_mctrl	= pport_set_mctrl,
	.stop_tx	= pport_stop_tx,
	.start_tx	= pport_start_tx,
	.throttle	= pport_throttle,
	.unthrottle	= pport_unthrottle,
	.stop_rx	= pport_stop_rx,
	.break_ctl	= pport_break_ctl,
	.startup	= pport_startup,
	.shutdown	= pport_shutdown,
	.set_termios	= pport_set_termios,
	.config_port	= pport_config_port,
};

static struct uart_driver psuart_port_drv = {
	.owner		= THIS_MODULE,
	.driver_name	= "PRU-SWUART",
	.dev_name	= PSUART_NAME,
	.nr		= DRV_TOTAL_PORTS,
};

static int pport_config_port_pins(struct psuart_port *pp,
			struct device_node *np)
{
	struct device *dev = pp->port.dev;
	struct pport_pins *pins;
	int nr_pins;
	int ret;
	u32 val;

	nr_pins = of_property_count_u8_elems(np, "ti,pru-swuart-pins");

	/* CTS/RTS pins are optional */
	if (nr_pins != 2 && nr_pins != 4) {
		dev_err(dev, "unexpected number of pins\n");
		return -EINVAL;
	}
	pp->port.flags = (nr_pins == 4) ?  UPF_HARD_FLOW : 0;

	pins = devm_kmalloc(dev, sizeof(*pins), GFP_KERNEL);
	if (!pins)
		return -ENOMEM;

	/* set non-configured pin value to 0xff */
	memset(pins, 0xff, sizeof(*pins));
	ret = of_property_read_u8_array(np, "ti,pru-swuart-pins",
			(u8 *)pins, nr_pins);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "interrupts", &val);
	if (ret)
		return ret;

	val = pins->cts << 24 | pins->tx << 16 | (val & 0xff);
	pport_writel(pp, PPORT_TX_CFG, val);

	val = pins->rts << 24 | pins->rx << 16 | (val & 0xff);
	pport_writel(pp, PPORT_RX_CFG, val);

	return 0;
}

static int psuart_init_port(struct pru_swuart *pu, struct device_node *np,
			int index)
{
	struct psuart_port *pp;
	int port_id;
	int ret = 0;

	port_id = pu->pru_id * MAX_UART_PORTS + index;
	pp = &pports[port_id];
	if (pp->mbase) {
		dev_err(pu->dev, "Error: port[%d] is already initialized\n",
				index);
		return -EEXIST;
	}

	ret = of_irq_get(np, 0);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			dev_err(pu->dev, "port[%d]: failed to get irq (%d)\n",
					index, ret);
		return ret;
	}
	pp->port.irq = ret;

	ret = request_irq(pp->port.irq, pport_handle_irq,
			  IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
			  dev_name(pu->dev), pp);
	if (ret) {
		dev_err(pu->dev, "port[%d]: failed to request irq (%d)\n",
				index, ret);
		return ret;
	}

	pp->mbase = pu->mem.va + PORT_MMR_BASE + PORT_MMR_LEN * index;
	pp->tx_fifo = pu->mem.va + FIFO_BASE + FIFO_SIZE * 2 * index;
	pp->rx_fifo = pp->tx_fifo + FIFO_SIZE;

	pp->port.dev = pu->dev;
	pp->port.type = PORT_PSUART;
	pp->port.iotype = UPIO_MEM;
	pp->port.fifosize = FIFO_SIZE / BPC;
	pp->port.ops = &psuart_port_ops;
	pp->port.line = port_id;
	pp->port.rs485_config = pport_rs485_config;

	ret = pport_config_port_pins(pp, np);
	if (ret) {
		free_irq(pp->port.irq, pp);
		return ret;
	}

	ret = uart_add_one_port(&psuart_port_drv, &pp->port);
	if (ret) {
		dev_err(pu->dev, "adding port[%d] failed (%d)\n", index, ret);
		pp->mbase = NULL;
		free_irq(pp->port.irq, pp);
	}

	return ret;
}

static int psuart_init_pruss(struct device_node *np, struct pru_swuart *pu)
{
	u32 reg;
	int ret = 0;

	pu->pru = pru_rproc_get(np, 0, &pu->pru_id);
	if (IS_ERR(pu->pru)) {
		ret = PTR_ERR(pu->pru);
		if (ret != -EPROBE_DEFER)
			dev_err(pu->dev, "failed to get pru (%d)\n", ret);
		return ret;
	}

	pu->pruss = pruss_get(pu->pru);
	if (IS_ERR(pu->pruss)) {
		ret = PTR_ERR(pu->pruss);
		dev_err(pu->dev, "failed to get pruss handle (%d)\n", ret);
		goto put_pru;
	}

	ret = pruss_cfg_ocp_master_ports(pu->pruss, 1);
	if (ret) {
		dev_err(pu->dev, "failed to enable ocp master port (%d)\n",
				ret);
		goto put_pruss;
	}

	if (pu->pru_id >= PRUSS_NUM_PRUS) {
		dev_err(pu->dev, "invalid pru id (%d)\n", pu->pru_id);
		ret = -EINVAL;
		goto put_ocp;
	}

	ret = pruss_request_mem_region(pu->pruss,
			pu->pru_id ? PRUSS_MEM_DRAM1 : PRUSS_MEM_DRAM0,
			&pu->mem);
	if (ret) {
		dev_err(pu->dev, "failed to get pruss mem region (%d)\n", ret);
		goto put_ocp;
	}

	/* clear the mem region before firmware runs by rproc_boot() */
	memset_io(pu->mem.va, 0, pu->mem.size);

	ret = rproc_boot(pu->pru);
	if (ret) {
		dev_err(pu->dev, "failed to boot pru (%d)\n", ret);
		goto put_mem;
	}

	reg = psuart_readl(pu, PSUART_FW_MAGIC);
	if (reg != PSUART_FW_MAGIC_NUMBER) {
		dev_err(pu->dev, "invalid firmware magic number\n");
		ret = -EINVAL;
		goto put_rproc;
	}

	reg = psuart_readl(pu, PSUART_FW_VERSION);
	if (reg > 0x01000000) {
		dev_err(pu->dev, "unsupported firmware version(0x%x)\n",
				reg);
		ret = -EINVAL;
		goto put_rproc;
	}

	reg = psuart_readl(pu, PSUART_FW_GCFG);
	if (!(reg & PSUART_FW_INITED)) {
		dev_err(pu->dev, "failed to initialize firmware\n");
		ret = -EINVAL;
		goto put_rproc;
	}

	return ret;

put_rproc:
	rproc_shutdown(pu->pru);
put_mem:
	pruss_release_mem_region(pu->pruss, &pu->mem);
put_ocp:
	pruss_cfg_ocp_master_ports(pu->pruss, 0);
put_pruss:
	pruss_put(pu->pruss);
put_pru:
	pru_rproc_put(pu->pru);

	return ret;
}

static void psuart_free_pruss(struct pru_swuart *pu)
{
	rproc_shutdown(pu->pru);
	pruss_release_mem_region(pu->pruss, &pu->mem);
	pruss_cfg_ocp_master_ports(pu->pruss, 0);
	pruss_put(pu->pruss);
	pru_rproc_put(pu->pru);
}

static const struct of_device_id psuart_dt_ids[] = {
	{ .compatible = "ti,pru-swuart", },
	{},
};
MODULE_DEVICE_TABLE(of, psuart_dt_ids);

static int psuart_probe(struct platform_device *pdev)
{
	struct pru_swuart *pu;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *child;
	int id, ret;

	if (!np)
		return -ENODEV;	/* we don't support non DT */

	pu = devm_kzalloc(dev, sizeof(*pu), GFP_KERNEL);
	if (!pu)
		return -ENOMEM;

	platform_set_drvdata(pdev, pu);
	pu->dev = dev;

	ret = psuart_init_pruss(np, pu);
	if (ret < 0)
		return ret;

	for_each_available_child_of_node(np, child) {
		ret = of_property_read_u32(child, "reg", &id);

		if (ret || id < 0 || id >= MAX_UART_PORTS)
			continue;

		ret = psuart_init_port(pu, child, id);
		if (ret == -EPROBE_DEFER) {
			/*
			 * -EPROBE_DEFER could only happen on the first port
			 *  so no initialized ports to free.
			 */
			psuart_free_pruss(pu);
			return ret;
		} else if (ret) {
			dev_err(pu->dev, "init port[%d] failed(%d)\n", id, ret);
		}

	}
	return 0;
}

static int psuart_remove(struct platform_device *pdev)
{
	struct pru_swuart *pu = platform_get_drvdata(pdev);
	struct psuart_port *pp;
	int i;

	for (i = 0; i < MAX_UART_PORTS; i++) {
		pp = &pports[pu->pru_id * MAX_UART_PORTS + i];
		if (!pp->mbase)
			continue;

		uart_remove_one_port(&psuart_port_drv, &pp->port);
		pp->mbase = NULL;
		free_irq(pp->port.irq, pp);
	}

	psuart_free_pruss(pu);
	return 0;
}

static struct platform_driver psuart_driver = {
	.probe = psuart_probe,
	.remove = psuart_remove,
	.driver = {
		.name = "pru_swuart",
		.of_match_table = psuart_dt_ids,
	},
};

static int __init psuart_init(void)
{
	int ret;

	ret = uart_register_driver(&psuart_port_drv);
	if (ret)
		return ret;

	ret = platform_driver_register(&psuart_driver);
	if (ret)
		uart_unregister_driver(&psuart_port_drv);

	return ret;
}
module_init(psuart_init);

static void __exit psuart_exit(void)
{
	platform_driver_unregister(&psuart_driver);
	uart_unregister_driver(&psuart_port_drv);
}
module_exit(psuart_exit);

MODULE_AUTHOR("Bin Liu <b-liu@ti.com>");
MODULE_DESCRIPTION("PRU SWUART Driver");
MODULE_LICENSE("GPL v2");
