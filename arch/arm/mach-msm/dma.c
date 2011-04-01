/* linux/arch/arm/mach-msm/dma.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <mach/dma.h>

#define MSM_DMOV_CHANNEL_COUNT 16
#define MSM_DMOV_MAX_ADMS 2

#define MODULE_NAME "msm_dmov"

enum {
	MSM_DMOV_PRINT_ERRORS = 1,
	MSM_DMOV_PRINT_IO = 2,
	MSM_DMOV_PRINT_FLOW = 4
};

unsigned int msm_dmov_print_mask = MSM_DMOV_PRINT_ERRORS;

struct msm_dmov_conf {
	void __iomem *base;
	int channel_active;
	struct list_head ready_commands[MSM_DMOV_CHANNEL_COUNT];
	struct list_head active_commands[MSM_DMOV_CHANNEL_COUNT];
	spinlock_t lock;
	unsigned irq;
	struct clk *clk;
	struct clk *pclk;
};

static struct msm_dmov_conf dmov_conf[MSM_DMOV_MAX_ADMS];
static int nr_adms;

#if defined(CONFIG_ARCH_MSM7X30)
#define DMOV_SD_AARM DMOV_SD2
#define DMOV_SD_SIZE 0x400
#elif defined(CONFIG_ARCH_MSM8X60)
#define DMOV_SD_AARM DMOV_SD1
#define DMOV_SD_SIZE 0x800
#elif defined(CONFIG_ARCH_MSM8960)
#define DMOV_SD_AARM DMOV_SD0
#define DMOV_SD_SIZE 0x800
#else
#define DMOV_SD_AARM DMOV_SD3
#define DMOV_SD_SIZE 0x400
#endif

#define DMOV_ADDR(sd, off, ch) (((sd) * DMOV_SD_SIZE) + (off) + ((ch) << 2))

#define DMOV_SD0(off, ch) DMOV_ADDR(0, off, ch)
#define DMOV_SD1(off, ch) DMOV_ADDR(1, off, ch)
#define DMOV_SD2(off, ch) DMOV_ADDR(2, off, ch)
#define DMOV_SD3(off, ch) DMOV_ADDR(3, off, ch)

#define DMOV_CMD_PTR(ch)      DMOV_SD_AARM(0x000, ch)
#define DMOV_RSLT(ch)         DMOV_SD_AARM(0x040, ch)
#define DMOV_FLUSH0(ch)       DMOV_SD_AARM(0x080, ch)
#define DMOV_FLUSH1(ch)       DMOV_SD_AARM(0x0C0, ch)
#define DMOV_FLUSH2(ch)       DMOV_SD_AARM(0x100, ch)
#define DMOV_FLUSH3(ch)       DMOV_SD_AARM(0x140, ch)
#define DMOV_FLUSH4(ch)       DMOV_SD_AARM(0x180, ch)
#define DMOV_FLUSH5(ch)       DMOV_SD_AARM(0x1C0, ch)
#define DMOV_STATUS(ch)       DMOV_SD_AARM(0x200, ch)
#define DMOV_CONFIG(ch)       DMOV_SD_AARM(0x300, ch)
#define DMOV_ISR              DMOV_SD_AARM(0x380, 0)

#define MSM_DMOV_DPRINTF(mask, format, args...) \
	do { \
		if ((mask) & msm_dmov_print_mask) \
			printk(KERN_ERR format, args); \
	} while (0)
#define PRINT_ERROR(format, args...) \
	MSM_DMOV_DPRINTF(MSM_DMOV_PRINT_ERRORS, format, args);
#define PRINT_IO(format, args...) \
	MSM_DMOV_DPRINTF(MSM_DMOV_PRINT_IO, format, args);
#define PRINT_FLOW(format, args...) \
	MSM_DMOV_DPRINTF(MSM_DMOV_PRINT_FLOW, format, args);

static inline unsigned dmov_readl(unsigned addr, int adm)
{
	return readl(dmov_conf[adm].base + addr);
}

static inline void dmov_writel(unsigned val, unsigned addr, int adm)
{
	writel(val, dmov_conf[adm].base + addr);
}

#define DMOV_ID_TO_ADM(id)   ((id) / MSM_DMOV_CHANNEL_COUNT)
#define DMOV_ID_TO_CHAN(id)   ((id) % MSM_DMOV_CHANNEL_COUNT)
#define DMOV_CHAN_ADM_TO_ID(ch, adm) ((ch) + (adm) * MSM_DMOV_CHANNEL_COUNT)

int msm_dmov_stop_cmd(unsigned id, struct msm_dmov_cmd *cmd, int graceful)
{
	int adm = DMOV_ID_TO_ADM(id);
	int ch = DMOV_ID_TO_CHAN(id);

	if (!dmov_conf[adm].base)
		return -ENODEV;

	dmov_writel((graceful << 31), DMOV_FLUSH0(ch), adm);

	return 0;
}
EXPORT_SYMBOL(msm_dmov_stop_cmd);

static int msm_dmov_clocks_on(int adm)
{
	int ret = 0;

	if (!IS_ERR(dmov_conf[adm].clk)) {
		ret = clk_enable(dmov_conf[adm].clk);
		if (ret)
			return ret;
		if (!IS_ERR(dmov_conf[adm].pclk)) {
			ret = clk_enable(dmov_conf[adm].pclk);
			if (ret)
				clk_disable(dmov_conf[adm].clk);
		}
	}
	return ret;
}

static void msm_dmov_clocks_off(int adm)
{
	if (!IS_ERR(dmov_conf[adm].clk))
		clk_disable(dmov_conf[adm].clk);
	if (!IS_ERR(dmov_conf[adm].pclk))
		clk_disable(dmov_conf[adm].pclk);
}

int msm_dmov_enqueue_cmd(unsigned id, struct msm_dmov_cmd *cmd)
{
	unsigned long irq_flags;
	unsigned int status;
	int adm = DMOV_ID_TO_ADM(id);
	int ch = DMOV_ID_TO_CHAN(id);

	if (!dmov_conf[adm].base)
		return -ENODEV;

	spin_lock_irqsave(&dmov_conf[adm].lock, irq_flags);
	if (!dmov_conf[adm].channel_active)
		msm_dmov_clocks_on(adm);
	dsb();
	status = dmov_readl(DMOV_STATUS(ch), adm);
	if (list_empty(&dmov_conf[adm].ready_commands[ch]) &&
		(status & DMOV_STATUS_CMD_PTR_RDY)) {
		if (cmd->execute_func)
			cmd->execute_func(cmd);
		PRINT_IO("msm_dmov_enqueue_cmd(%d), start command, status %x\n",
			 id, status);
		list_add_tail(&cmd->list, &dmov_conf[adm].active_commands[ch]);
		if (!dmov_conf[adm].channel_active)
			enable_irq(dmov_conf[adm].irq);
		dmov_conf[adm].channel_active |= 1U << ch;
		dmov_writel(cmd->cmdptr, DMOV_CMD_PTR(ch), adm);
	} else {
		if (!dmov_conf[adm].channel_active)
			msm_dmov_clocks_off(adm);
		if (list_empty(&dmov_conf[adm].active_commands[ch]))
			PRINT_ERROR("msm_dmov_enqueue_cmd(%d), error datamover "
				    "stalled, status %x\n", id, status);
		PRINT_IO("msm_dmov_enqueue_cmd(%d), enqueue command, status "
			 "%x\n", id, status);
		list_add_tail(&cmd->list, &dmov_conf[adm].ready_commands[ch]);
	}
	spin_unlock_irqrestore(&dmov_conf[adm].lock, irq_flags);

	return 0;
}
EXPORT_SYMBOL(msm_dmov_enqueue_cmd);

int msm_dmov_flush(unsigned int id)
{
	unsigned long flags;
	int ch = DMOV_ID_TO_CHAN(id);
	int adm = DMOV_ID_TO_ADM(id);

	if (!dmov_conf[adm].base)
		return -ENODEV;

	spin_lock_irqsave(&dmov_conf[adm].lock, flags);
	/* XXX not checking if flush cmd sent already */
	if (!list_empty(&dmov_conf[adm].active_commands[ch])) {
		PRINT_IO("msm_dmov_flush(%d), send flush cmd\n", id);
		dmov_writel(DMOV_FLUSH_GRACEFUL, DMOV_FLUSH0(ch), adm);
	}
	spin_unlock_irqrestore(&dmov_conf[adm].lock, flags);

	return 0;
}
EXPORT_SYMBOL(msm_dmov_flush);

struct msm_dmov_exec_cmdptr_cmd {
	struct msm_dmov_cmd dmov_cmd;
	struct completion complete;
	unsigned id;
	unsigned int result;
	struct msm_dmov_errdata err;
};

static void
dmov_exec_cmdptr_complete_func(struct msm_dmov_cmd *_cmd,
			       unsigned int result,
			       struct msm_dmov_errdata *err)
{
	struct msm_dmov_exec_cmdptr_cmd *cmd = container_of(_cmd, struct msm_dmov_exec_cmdptr_cmd, dmov_cmd);
	cmd->result = result;
	if (result != 0x80000002 && err)
		memcpy(&cmd->err, err, sizeof(struct msm_dmov_errdata));

	complete(&cmd->complete);
}

int msm_dmov_exec_cmd(unsigned id, unsigned int cmdptr)
{
	struct msm_dmov_exec_cmdptr_cmd cmd;
	int adm = DMOV_ID_TO_ADM(id);

	if (!dmov_conf[adm].base)
		return -ENODEV;

	PRINT_FLOW("dmov_exec_cmdptr(%d, %x)\n", id, cmdptr);

	cmd.dmov_cmd.cmdptr = cmdptr;
	cmd.dmov_cmd.complete_func = dmov_exec_cmdptr_complete_func;
	cmd.dmov_cmd.execute_func = NULL;
	cmd.id = id;
	init_completion(&cmd.complete);

	msm_dmov_enqueue_cmd(id, &cmd.dmov_cmd);
	wait_for_completion(&cmd.complete);

	if (cmd.result != 0x80000002) {
		PRINT_ERROR("dmov_exec_cmdptr(%d): ERROR, result: %x\n", id, cmd.result);
		PRINT_ERROR("dmov_exec_cmdptr(%d):  flush: %x %x %x %x\n",
			id, cmd.err.flush[0], cmd.err.flush[1], cmd.err.flush[2], cmd.err.flush[3]);
		return -EIO;
	}
	PRINT_FLOW("dmov_exec_cmdptr(%d, %x) done\n", id, cmdptr);
	return 0;
}
EXPORT_SYMBOL(msm_dmov_exec_cmd);

static void msm_dmov_fill_errdata(struct msm_dmov_errdata *errdata, int ch,
				  int adm)
{
	errdata->flush[0] = dmov_readl(DMOV_FLUSH0(ch), adm);
	errdata->flush[1] = dmov_readl(DMOV_FLUSH1(ch), adm);
	errdata->flush[2] = dmov_readl(DMOV_FLUSH2(ch), adm);
	errdata->flush[3] = dmov_readl(DMOV_FLUSH3(ch), adm);
	errdata->flush[4] = dmov_readl(DMOV_FLUSH4(ch), adm);
	errdata->flush[5] = dmov_readl(DMOV_FLUSH5(ch), adm);
}

static int msm_dmov_irq_to_adm(unsigned irq)
{
	int i;
	for (i = 0; i < nr_adms; i++)
		if (dmov_conf[i].irq == irq)
			return i;
	PRINT_ERROR("msm_dmov_irq_to_adm: can't match ADM to IRQ %d\n", irq);
	return -EINVAL;
}

static irqreturn_t msm_datamover_irq_handler(int irq, void *dev_id)
{
	unsigned int int_status, mask, id;
	unsigned long irq_flags;
	unsigned int ch;
	unsigned int ch_status;
	unsigned int ch_result;
	struct msm_dmov_cmd *cmd;
	int adm = msm_dmov_irq_to_adm(irq);

	spin_lock_irqsave(&dmov_conf[adm].lock, irq_flags);

	int_status = dmov_readl(DMOV_ISR, adm); /* read and clear interrupt */
	PRINT_FLOW("msm_datamover_irq_handler: DMOV_ISR %x\n", int_status);

	while (int_status) {
		mask = int_status & -int_status;
		ch = fls(mask) - 1;
		id = DMOV_CHAN_ADM_TO_ID(ch, adm);
		PRINT_FLOW("msm_datamover_irq_handler %08x %08x id %d\n", int_status, mask, id);
		int_status &= ~mask;
		ch_status = dmov_readl(DMOV_STATUS(ch), adm);
		if (!(ch_status & DMOV_STATUS_RSLT_VALID)) {
			PRINT_FLOW("msm_datamover_irq_handler id %d, result not valid %x\n", id, ch_status);
			continue;
		}
		do {
			ch_result = dmov_readl(DMOV_RSLT(ch), adm);
			if (list_empty(&dmov_conf[adm].active_commands[ch])) {
				PRINT_ERROR("msm_datamover_irq_handler id %d, got result "
					"with no active command, status %x, result %x\n",
					id, ch_status, ch_result);
				cmd = NULL;
			} else
				cmd = list_entry(dmov_conf[adm].
					active_commands[ch].next, typeof(*cmd),
					list);
			PRINT_FLOW("msm_datamover_irq_handler id %d, status %x,"
				   " result %x\n", id, ch_status, ch_result);
			if (ch_result & DMOV_RSLT_DONE) {
				PRINT_FLOW("msm_datamover_irq_handler id %d, status %x\n",
					id, ch_status);
				PRINT_IO("msm_datamover_irq_handler id %d, got result "
					"for %p, result %x\n", id, cmd, ch_result);
				if (cmd) {
					list_del(&cmd->list);
					dsb();
					cmd->complete_func(cmd, ch_result, NULL);
				}
			}
			if (ch_result & DMOV_RSLT_FLUSH) {
				struct msm_dmov_errdata errdata;

				msm_dmov_fill_errdata(&errdata, ch, adm);
				PRINT_FLOW("msm_datamover_irq_handler id %d, status %x\n", id, ch_status);
				PRINT_FLOW("msm_datamover_irq_handler id %d, flush, result %x, flush0 %x\n", id, ch_result, errdata.flush[0]);
				if (cmd) {
					list_del(&cmd->list);
					dsb();
					cmd->complete_func(cmd, ch_result, &errdata);
				}
			}
			if (ch_result & DMOV_RSLT_ERROR) {
				struct msm_dmov_errdata errdata;

				msm_dmov_fill_errdata(&errdata, ch, adm);
				PRINT_ERROR("msm_datamover_irq_handler id %d, status %x\n", id, ch_status);
				PRINT_ERROR("msm_datamover_irq_handler id %d, error, result %x, flush0 %x\n", id, ch_result, errdata.flush[0]);
				if (cmd) {
					list_del(&cmd->list);
					dsb();
					cmd->complete_func(cmd, ch_result, &errdata);
				}
				/* this does not seem to work, once we get an error */
				/* the datamover will no longer accept commands */
				dmov_writel(0, DMOV_FLUSH0(ch), adm);
			}
			ch_status = dmov_readl(DMOV_STATUS(ch), adm);
			PRINT_FLOW("msm_datamover_irq_handler id %d, status %x\n", id, ch_status);
			if ((ch_status & DMOV_STATUS_CMD_PTR_RDY) &&
			    !list_empty(&dmov_conf[adm].ready_commands[ch])) {
				cmd = list_entry(dmov_conf[adm].
					ready_commands[ch].next, typeof(*cmd),
					list);
				list_del(&cmd->list);
				list_add_tail(&cmd->list, &dmov_conf[adm].
					      active_commands[ch]);
				if (cmd->execute_func)
					cmd->execute_func(cmd);
				PRINT_FLOW("msm_datamover_irq_handler id %d, start command\n", id);
				dmov_writel(cmd->cmdptr, DMOV_CMD_PTR(ch), adm);
			}
		} while (ch_status & DMOV_STATUS_RSLT_VALID);
		if (list_empty(&dmov_conf[adm].active_commands[ch]) &&
				list_empty(&dmov_conf[adm].ready_commands[ch]))
			dmov_conf[adm].channel_active &= ~(1U << ch);
		PRINT_FLOW("msm_datamover_irq_handler id %d, status %x\n", id, ch_status);
	}

	if (!dmov_conf[adm].channel_active) {
		disable_irq_nosync(dmov_conf[adm].irq);
		msm_dmov_clocks_off(adm);
	}

	spin_unlock_irqrestore(&dmov_conf[adm].lock, irq_flags);
	return IRQ_HANDLED;
}

static void __init msm_dmov_deinit_clocks(int adm)
{
	if (!IS_ERR(dmov_conf[adm].clk))
		clk_put(dmov_conf[adm].clk);
	if (!IS_ERR(dmov_conf[adm].pclk))
		clk_put(dmov_conf[adm].pclk);
}

#define PDEV_TO_ADM(pdev) \
({ \
	typeof(pdev) _pdev = pdev; \
	(_pdev->id == -1) ? 0 : _pdev->id; \
})

static int __devinit msm_dmov_init_clocks(struct platform_device *pdev)
{
	int ret = 0;
	int adm = PDEV_TO_ADM(pdev);

	dmov_conf[adm].clk = clk_get(&pdev->dev, "adm_clk");
	if (IS_ERR(dmov_conf[adm].clk)) {
		PRINT_ERROR("%s: Error getting adm_clk\n", __func__);
		ret = PTR_ERR(dmov_conf[adm].clk);
	}

	dmov_conf[adm].pclk = clk_get(&pdev->dev, "adm_pclk");
	/* pclk not present on all SoCs, don't return error on failure */

	return ret;
}

static int __devinit msm_dmov_conf_init(struct platform_device *pdev)
{
	int i;
	int adm = PDEV_TO_ADM(pdev);
	struct resource *irqres =
		platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	struct resource *memres =
		platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!irqres || !memres || !irqres->start)
		return -EINVAL;

	dmov_conf[adm].irq = irqres->start;

	dmov_conf[adm].base =
		ioremap_nocache(memres->start, resource_size(memres));
	if (!dmov_conf[adm].base)
		return -ENOMEM;

	dmov_conf[adm].lock = __SPIN_LOCK_UNLOCKED(dmov_lock);
	for (i = 0; i < MSM_DMOV_CHANNEL_COUNT; i++) {
		INIT_LIST_HEAD(&dmov_conf[adm].ready_commands[i]);
		INIT_LIST_HEAD(&dmov_conf[adm].active_commands[i]);
	}
	return 0;
}

static inline void __devinit msm_dmov_conf_free(int adm)
{
	iounmap(dmov_conf[adm].base);
	dmov_conf[adm].base = NULL;
}

static int __devinit msm_dmov_probe(struct platform_device *pdev)
{
	int i;
	int ret;
	int adm = PDEV_TO_ADM(pdev);

	ret = msm_dmov_conf_init(pdev);
	if (ret)
		return ret;

	for (i = 0; i < MSM_DMOV_CHANNEL_COUNT; i++)
		dmov_writel(DMOV_CONFIG_IRQ_EN | DMOV_CONFIG_FORCE_TOP_PTR_RSLT
			  | DMOV_CONFIG_FORCE_FLUSH_RSLT, DMOV_CONFIG(i), adm);

	ret = msm_dmov_init_clocks(pdev);
	if (ret)
		goto out_conf;

	ret = request_irq(dmov_conf[adm].irq, msm_datamover_irq_handler, 0,
			  "msmdatamover", NULL);
	if (ret)
		goto out_clock;
	disable_irq(dmov_conf[adm].irq);
	nr_adms++;

	return 0;

out_clock:
	msm_dmov_deinit_clocks(adm);
out_conf:
	msm_dmov_conf_free(adm);
	return ret;
}

static struct platform_driver msm_dmov_driver = {
	.probe = msm_dmov_probe,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init msm_init_datamover(void)
{
	return platform_driver_register(&msm_dmov_driver);
}

arch_initcall(msm_init_datamover);
