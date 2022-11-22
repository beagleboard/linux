// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>

/* Status Register */
#define LIGHT_MBOX_STA		0x0
#define LIGHT_MBOX_CLR		0x4
#define LIGHT_MBOX_MASK		0xc

/* Transmit/receive data register:
 * INFO0 ~ INFO6
 */
#define LIGHT_MBOX_INFO_NUM		8
#define LIGHT_MBOX_DATA_INFO_NUM	7
#define LIGHT_MBOX_INFO0	0x14
/* Transmit ack register: INFO7 */
#define LIGHT_MBOX_INFO7	0x30

/* Generate remote icu IRQ Register */
#define LIGHT_MBOX_GEN		0x10
#define LIGHT_MBOX_GEN_RX_DATA	BIT(6)
#define LIGHT_MBOX_GEN_TX_ACK	BIT(7)

#define LIGHT_MBOX_CHAN_RES_SIZE	0x1000
#define LIGHT_MBOX_CHANS		4
#define LIGHT_MBOX_CHAN_NAME_SIZE	20

#define LIGHT_MBOX_ACK_MAGIC		0xdeadbeaf

enum light_mbox_chan_type {
	LIGHT_MBOX_TYPE_TXRX,	/* Tx & Rx chan */
	LIGHT_MBOX_TYPE_DB,	/* Tx & Rx doorbell */
};

enum light_mbox_icu_cpu_id {
	LIGHT_MBOX_ICU_CPU0,		/* 910T */
	LIGHT_MBOX_ICU_CPU1,            /* 902 */
	LIGHT_MBOX_ICU_CPU2,            /* 906 */
	LIGHT_MBOX_ICU_CPU3,            /* 910R */
};

struct light_mbox_con_priv {
	enum light_mbox_icu_cpu_id	idx;
	enum light_mbox_chan_type	type;
	void __iomem			*comm_local_base;
	void __iomem			*comm_remote_base;
	char				irq_desc[LIGHT_MBOX_CHAN_NAME_SIZE];
	struct mbox_chan		*chan;
	struct tasklet_struct		txdb_tasklet;
};

struct light_mbox_priv {
	struct device			*dev;
	void __iomem			*local_icu[LIGHT_MBOX_CHANS];
	void __iomem			*remote_icu[LIGHT_MBOX_CHANS - 1];
	void __iomem			*cur_cpu_ch_base;
	enum light_mbox_icu_cpu_id	cur_icu_cpu_id;
	spinlock_t			mbox_lock; /* control register lock */

	struct mbox_controller		mbox;
	struct mbox_chan		mbox_chans[LIGHT_MBOX_CHANS];

	struct light_mbox_con_priv  	con_priv[LIGHT_MBOX_CHANS];
	struct clk			*clk;
	int				irq;
};

static struct light_mbox_priv *to_light_mbox_priv(struct mbox_controller *mbox)
{
	return container_of(mbox, struct light_mbox_priv, mbox);
}

static void light_mbox_write(struct light_mbox_priv *priv, u32 val, u32 offs)
{
	iowrite32(val, priv->cur_cpu_ch_base + offs);
}

static u32 light_mbox_read(struct light_mbox_priv *priv, u32 offs)
{
	return ioread32(priv->cur_cpu_ch_base + offs);
}

static u32 light_mbox_rmw(struct light_mbox_priv *priv, u32 off, u32 set, u32 clr)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&priv->mbox_lock, flags);
	val = light_mbox_read(priv, off);
	val &= ~clr;
	val |= set;
	light_mbox_write(priv, val, off);
	spin_unlock_irqrestore(&priv->mbox_lock, flags);

	return val;
}

static void light_mbox_chan_write(struct light_mbox_con_priv *cp, u32 val, u32 offs, bool is_remote)
{
	if (is_remote)
		iowrite32(val, cp->comm_remote_base + offs);
	else
		iowrite32(val, cp->comm_local_base + offs);
}

static u32 light_mbox_chan_read(struct light_mbox_con_priv *cp, u32 offs, bool is_remote)
{
	if (is_remote)
		return ioread32(cp->comm_remote_base + offs);
	else
		return ioread32(cp->comm_local_base + offs);
}

static void light_mbox_chan_rmw(struct light_mbox_con_priv *cp, u32 off, u32 set, u32 clr, bool is_remote)
{
	struct light_mbox_priv *priv = to_light_mbox_priv(cp->chan->mbox);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&priv->mbox_lock, flags);
	val = light_mbox_chan_read(cp, off, is_remote);
	val &= ~clr;
	val |= set;
	light_mbox_chan_write(cp, val, off, is_remote);
	spin_unlock_irqrestore(&priv->mbox_lock, flags);
}

static void light_mbox_chan_rd_data(struct light_mbox_con_priv *cp, void *data, bool is_remote)
{
	u32 off = LIGHT_MBOX_INFO0;
	u32 *arg = data;
	u32 i;

	/* read info0 ~ info6, totally 28 bytes
	 * requires data memory size is 28 bytes
	 */
	for (i = 0; i < LIGHT_MBOX_DATA_INFO_NUM; i++) {
		*arg = light_mbox_chan_read(cp, off, is_remote);
		off += 4;
		arg++;
	}
}

static void light_mbox_chan_wr_data(struct light_mbox_con_priv *cp, void *data, bool is_remote)
{
	u32 off = LIGHT_MBOX_INFO0;
	u32 *arg = data;
	u32 i;

	/* write info0 ~ info6, totally 28 bytes
	 * requires data memory is 28 bytes valid data
	 */
	for (i = 0; i < LIGHT_MBOX_DATA_INFO_NUM; i++) {
		light_mbox_chan_write(cp, *arg, off, is_remote);
		off += 4;
		arg++;
	}
}

static void light_mbox_chan_wr_ack(struct light_mbox_con_priv *cp, void *data, bool is_remote)
{
	u32 off = LIGHT_MBOX_INFO7;
	u32 *arg = data;

	light_mbox_chan_write(cp, *arg, off, is_remote);
}

static int light_mbox_chan_id_to_mapbit(struct light_mbox_con_priv *cp)
{
	struct light_mbox_priv *priv = to_light_mbox_priv(cp->chan->mbox);
	int mapbit = 0;
	int i;

	for (i = 0; i < LIGHT_MBOX_CHANS; i++) {
		if (i == cp->idx)
			return mapbit;

		if (i != priv->cur_icu_cpu_id)
			mapbit++;
	}

	if (i == LIGHT_MBOX_CHANS)
		dev_err(cp->chan->mbox->dev, "convert to mapbit failed\n");

	return 0;
}

static void light_mbox_txdb_tasklet(unsigned long data)
{
	struct light_mbox_con_priv *cp = (struct light_mbox_con_priv *)data;

	mbox_chan_txdone(cp->chan, 0);
}

static irqreturn_t light_mbox_isr(int irq, void *p)
{
	struct mbox_chan *chan = p;
	struct light_mbox_priv *priv = to_light_mbox_priv(chan->mbox);
	struct light_mbox_con_priv *cp = chan->con_priv;
	int mapbit = light_mbox_chan_id_to_mapbit(cp);
	u32 sta, dat[LIGHT_MBOX_DATA_INFO_NUM];
	u32 ack_magic = LIGHT_MBOX_ACK_MAGIC;
	u32 info0_data, info7_data;

	sta = light_mbox_read(priv, LIGHT_MBOX_STA);
	if (!(sta & BIT(mapbit)))
		return IRQ_NONE;

	/* clear chan irq bit in STA register */
	light_mbox_rmw(priv, LIGHT_MBOX_CLR, BIT(mapbit), 0);

	/* rx doorbell */
	if (cp->type == LIGHT_MBOX_TYPE_DB) {
		mbox_chan_received_data(cp->chan, NULL);
		return IRQ_HANDLED;
	}

	/* info0 is the protocol word, shoud not be zero! */
	info0_data = light_mbox_chan_read(cp, LIGHT_MBOX_INFO0, false);
	if (info0_data) {
		/* read info0~info6 data */
		light_mbox_chan_rd_data(cp, dat, false);

		/* clear local info0 */
		light_mbox_chan_write(cp, 0x0, LIGHT_MBOX_INFO0, false);

		/* notify remote cpu */
		light_mbox_chan_wr_ack(cp, &ack_magic, true);
		/* CPU1 902/906 use polling mode to monitor info7 */
		if (cp->idx != LIGHT_MBOX_ICU_CPU1 && cp->idx != LIGHT_MBOX_ICU_CPU2)
			light_mbox_chan_rmw(cp, LIGHT_MBOX_GEN, LIGHT_MBOX_GEN_TX_ACK, 0, true);

		/* transfer the data to client */
		mbox_chan_received_data(chan, (void *)dat);
	}

	/* info7 magic value mean the real ack signal, not generate bit7 */
	info7_data = light_mbox_chan_read(cp, LIGHT_MBOX_INFO7, false);
	if (info7_data == LIGHT_MBOX_ACK_MAGIC) {
		/* clear local info7 */
		light_mbox_chan_write(cp, 0x0, LIGHT_MBOX_INFO7, false);

		/* notify framework the last TX has completed */
		mbox_chan_txdone(chan, 0);
	}

	if (!info0_data && !info7_data)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

static int light_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct light_mbox_con_priv *cp = chan->con_priv;

	if (cp->type == LIGHT_MBOX_TYPE_DB)
		tasklet_schedule(&cp->txdb_tasklet);
	else
		light_mbox_chan_wr_data(cp, data, true);
	light_mbox_chan_rmw(cp, LIGHT_MBOX_GEN, LIGHT_MBOX_GEN_RX_DATA, 0, true);
	return 0;
}

static int light_mbox_startup(struct mbox_chan *chan)
{
	struct light_mbox_priv *priv = to_light_mbox_priv(chan->mbox);
	struct light_mbox_con_priv *cp = chan->con_priv;
	u32 data[8] = {0};
	int mask_bit;
	int ret;

	/* clear local and remote generate and info0~info7 */
	light_mbox_chan_rmw(cp, LIGHT_MBOX_GEN, 0x0, 0xff, true);
	light_mbox_chan_rmw(cp, LIGHT_MBOX_GEN, 0x0, 0xff, false);
	light_mbox_chan_wr_ack(cp, &data[7], true);
	light_mbox_chan_wr_ack(cp, &data[7], false);
	light_mbox_chan_wr_data(cp, &data[0], true);
	light_mbox_chan_wr_data(cp, &data[0], false);

	/* enable the chan mask */
	mask_bit = light_mbox_chan_id_to_mapbit(cp);
	light_mbox_rmw(priv, LIGHT_MBOX_MASK, BIT(mask_bit), 0);

	if (cp->type == LIGHT_MBOX_TYPE_DB)
		/* tx doorbell doesn't have ACK, rx doorbell requires isr */
		tasklet_init(&cp->txdb_tasklet, light_mbox_txdb_tasklet,
			     (unsigned long)cp);

	ret = request_irq(priv->irq, light_mbox_isr, IRQF_SHARED |
			  IRQF_NO_SUSPEND, cp->irq_desc, chan);
	if (ret) {
		dev_err(priv->dev,
			"Unable to acquire IRQ %d\n", priv->irq);
		return ret;
	}

	return 0;
}

static void light_mbox_shutdown(struct mbox_chan *chan)
{
	struct light_mbox_priv *priv = to_light_mbox_priv(chan->mbox);
	struct light_mbox_con_priv *cp = chan->con_priv;
	int mask_bit;

	/* clear the chan mask */
	mask_bit = light_mbox_chan_id_to_mapbit(cp);
	light_mbox_rmw(priv, LIGHT_MBOX_MASK, 0, BIT(mask_bit));

	free_irq(priv->irq, chan);
}

static const struct mbox_chan_ops light_mbox_ops = {
	.send_data = light_mbox_send_data,
	.startup = light_mbox_startup,
	.shutdown = light_mbox_shutdown,
};

static void light_mbox_init_generic(struct light_mbox_priv *priv)
{
	/* Set default configuration */
	light_mbox_write(priv, 0xff, LIGHT_MBOX_CLR);
	light_mbox_write(priv, 0x0, LIGHT_MBOX_MASK);
}

static struct mbox_chan *light_mbox_xlate(struct mbox_controller *mbox,
					  const struct of_phandle_args *sp)
{
	struct light_mbox_priv *priv = to_light_mbox_priv(mbox);
	struct light_mbox_con_priv *cp;
	u32 chan, type;

	if (sp->args_count != 2) {
		dev_err(mbox->dev, "Invalid argument count %d\n", sp->args_count);
		return ERR_PTR(-EINVAL);
	}

	chan = sp->args[0]; /* comm remote channel */
	type = sp->args[1]; /* comm channel type */

	if (chan >= mbox->num_chans) {
		dev_err(mbox->dev, "Not supported channel number: %d\n", chan);
		return ERR_PTR(-EINVAL);
	}

	if (chan == priv->cur_icu_cpu_id) {
		dev_err(mbox->dev, "Cannot communicate with yourself\n");
		return ERR_PTR(-EINVAL);
	}

	if (type > LIGHT_MBOX_TYPE_DB) {
		dev_err(mbox->dev, "Not supported the type for channel[%d]\n", chan);
		return ERR_PTR(-EINVAL);
	}

	cp = mbox->chans[chan].con_priv;
	cp->type = type;

	return &mbox->chans[chan];
}

static int light_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct light_mbox_priv *priv;
	struct resource *res;
	unsigned int remote_idx = 0;
	unsigned int i;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (of_property_read_u32(np, "icu_cpu_id", &priv->cur_icu_cpu_id)) {
		dev_err(dev, "icu_cpu_id is missing\n");
		return -EINVAL;
	}

	if (priv->cur_icu_cpu_id != LIGHT_MBOX_ICU_CPU0 &&
	    priv->cur_icu_cpu_id != LIGHT_MBOX_ICU_CPU3) {
		dev_err(dev, "icu_cpu_id is invalid\n");
		return -EINVAL;
	}

	priv->dev = dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "local_base");
	priv->local_icu[LIGHT_MBOX_ICU_CPU0] = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->local_icu[LIGHT_MBOX_ICU_CPU0]))
		return PTR_ERR(priv->local_icu[LIGHT_MBOX_ICU_CPU0]);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "remote_icu0");
	priv->remote_icu[0] = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->remote_icu[0]))
		return PTR_ERR(priv->remote_icu[0]);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "remote_icu1");
	priv->remote_icu[1] = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->remote_icu[1]))
		return PTR_ERR(priv->remote_icu[1]);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "remote_icu2");
	priv->remote_icu[2] = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->remote_icu[2]))
		return PTR_ERR(priv->remote_icu[2]);

	priv->local_icu[LIGHT_MBOX_ICU_CPU1] = priv->local_icu[LIGHT_MBOX_ICU_CPU0] +
					       LIGHT_MBOX_CHAN_RES_SIZE;
	priv->local_icu[LIGHT_MBOX_ICU_CPU2] = priv->local_icu[LIGHT_MBOX_ICU_CPU1] +
					       LIGHT_MBOX_CHAN_RES_SIZE;
	priv->local_icu[LIGHT_MBOX_ICU_CPU3] = priv->local_icu[LIGHT_MBOX_ICU_CPU2] +
					       LIGHT_MBOX_CHAN_RES_SIZE;

	priv->cur_cpu_ch_base = priv->local_icu[priv->cur_icu_cpu_id];

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return priv->irq;

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk)) {
		if (PTR_ERR(priv->clk) != -ENOENT)
			return PTR_ERR(priv->clk);

		priv->clk = NULL;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		return ret;
	}

	/* init the chans */
	for (i = 0; i < LIGHT_MBOX_CHANS; i++) {
		struct light_mbox_con_priv *cp = &priv->con_priv[i];

		cp->idx = i;
		cp->chan = &priv->mbox_chans[i];
		priv->mbox_chans[i].con_priv = cp;
		snprintf(cp->irq_desc, sizeof(cp->irq_desc),
			 "light_mbox_chan[%i]", cp->idx);

		cp->comm_local_base = priv->local_icu[i];
		if (i != priv->cur_icu_cpu_id) {
			cp->comm_remote_base = priv->remote_icu[remote_idx];
			remote_idx++;
		}
	}

	spin_lock_init(&priv->mbox_lock);

	priv->mbox.dev = dev;
	priv->mbox.ops = &light_mbox_ops;
	priv->mbox.chans = priv->mbox_chans;
	priv->mbox.num_chans = LIGHT_MBOX_CHANS;
	priv->mbox.of_xlate = light_mbox_xlate;
	priv->mbox.txdone_irq = true;

	platform_set_drvdata(pdev, priv);

	light_mbox_init_generic(priv);

	return devm_mbox_controller_register(dev, &priv->mbox);
}

static int light_mbox_remove(struct platform_device *pdev)
{
	struct light_mbox_priv *priv = platform_get_drvdata(pdev);

	clk_disable_unprepare(priv->clk);

	return 0;
}

static const struct of_device_id light_mbox_dt_ids[] = {
	{ .compatible = "thead,light-mbox" },
	{ },
};
MODULE_DEVICE_TABLE(of, light_mbox_dt_ids);

static struct platform_driver light_mbox_driver = {
	.probe		= light_mbox_probe,
	.remove		= light_mbox_remove,
	.driver = {
		.name	= "light_mbox",
		.of_match_table = light_mbox_dt_ids,
	},
};
module_platform_driver(light_mbox_driver);

MODULE_AUTHOR("fugang.duan <duanfugang.dfg@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead Light mailbox IPC driver");
MODULE_LICENSE("GPL v2");
