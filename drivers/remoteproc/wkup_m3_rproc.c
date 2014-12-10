/*
 * AMx3 Wkup M3 Remote Processor driver
 *
 * Copyright (C) 2014 Texas Instruments, Inc.
 *
 * Dave Gerlach <d-gerlach@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/firmware.h>
#include <linux/remoteproc.h>
#include <linux/omap-mailbox.h>
#include <linux/mailbox_client.h>
#include <linux/wkup_m3.h>
#include <linux/kthread.h>

#include <linux/platform_data/wkup_m3.h>

#include "remoteproc_internal.h"

#define WKUP_M3_WAKE_SRC_MASK		0xFF

#define WKUP_M3_STATUS_RESP_SHIFT	16
#define WKUP_M3_STATUS_RESP_MASK	(0xffff << 16)

#define WKUP_M3_FW_VERSION_SHIFT	0
#define WKUP_M3_FW_VERSION_MASK		0xffff

#define AM33XX_CTRL_IPC_REG_COUNT	0x8
#define AM33XX_CTRL_IPC_REG_OFFSET(m)	(0x4 + 4 * (m))

/* AM33XX M3_TXEV_EOI register */
#define AM33XX_CONTROL_M3_TXEV_EOI	0x00

#define AM33XX_M3_TXEV_ACK		(0x1 << 0)
#define AM33XX_M3_TXEV_ENABLE		(0x0 << 0)

#define WKUP_M3_DMEM_START		0x80000
#define WKUP_M3_AUXDATA_OFFSET		0x1000
#define WKUP_M3_AUXDATA_SIZE		0xFF

struct wkup_m3_rproc {
	struct rproc *rproc;

	void * __iomem dev_table_va;
	void * __iomem ipc_mem_base;
	struct platform_device *pdev;

	struct mbox_client mbox_client;
	struct mbox_chan *mbox;

	bool is_active;
	bool is_rtc_only;
};

static struct wkup_m3_rproc *m3_rproc_static;
struct wkup_m3_ops *wkup_m3_pm_ops;

static const struct wkup_m3_wakeup_src wakeups[] = {
	{.irq_nr = 16,	.src = "PRCM"},
	{.irq_nr = 35,	.src = "USB0_PHY"},
	{.irq_nr = 36,	.src = "USB1_PHY"},
	{.irq_nr = 40,	.src = "I2C0"},
	{.irq_nr = 41,	.src = "RTC Timer"},
	{.irq_nr = 42,	.src = "RTC Alarm"},
	{.irq_nr = 43,	.src = "Timer0"},
	{.irq_nr = 44,	.src = "Timer1"},
	{.irq_nr = 45,	.src = "UART"},
	{.irq_nr = 46,	.src = "GPIO0"},
	{.irq_nr = 48,	.src = "MPU_WAKE"},
	{.irq_nr = 49,	.src = "WDT0"},
	{.irq_nr = 50,	.src = "WDT1"},
	{.irq_nr = 51,	.src = "ADC_TSC"},
	{.irq_nr = 0,	.src = "Unknown"},
};

static void am33xx_txev_eoi(struct wkup_m3_rproc *m3_rproc)
{
	writel(AM33XX_M3_TXEV_ACK,
	       m3_rproc->ipc_mem_base + AM33XX_CONTROL_M3_TXEV_EOI);
}

static void am33xx_txev_enable(struct wkup_m3_rproc *m3_rproc)
{
	writel(AM33XX_M3_TXEV_ENABLE,
	       m3_rproc->ipc_mem_base + AM33XX_CONTROL_M3_TXEV_EOI);
}

static void wkup_m3_ctrl_ipc_write(struct wkup_m3_rproc *m3_rproc,
				   u32 val, int ipc_reg_num)
{
	if (ipc_reg_num < 0 || ipc_reg_num > AM33XX_CTRL_IPC_REG_COUNT)
		return;

	writel(val, m3_rproc->ipc_mem_base +
	       AM33XX_CTRL_IPC_REG_OFFSET(ipc_reg_num));
}

static unsigned int wkup_m3_ctrl_ipc_read(struct wkup_m3_rproc *m3_rproc,
					  int ipc_reg_num)
{
	if (ipc_reg_num < 0 || ipc_reg_num > AM33XX_CTRL_IPC_REG_COUNT)
		return 0;

	return readl(m3_rproc->ipc_mem_base +
		     AM33XX_CTRL_IPC_REG_OFFSET(ipc_reg_num));
}

static irqreturn_t wkup_m3_txev_handler(int irq, void *unused)
{
	am33xx_txev_eoi(m3_rproc_static);

	if (wkup_m3_pm_ops && wkup_m3_pm_ops->txev_handler)
		wkup_m3_pm_ops->txev_handler();

	am33xx_txev_enable(m3_rproc_static);

	return IRQ_HANDLED;
}

/**
 * wkup_m3_fw_version_clear - Clear FW version from ipc regs
 *
 * Invalidate M3 firmware version before hardreset.
 * Write invalid version in lower 4 nibbles of parameter
 * register (ipc_regs + 0x8).
 */

static void wkup_m3_fw_version_clear(void)
{
	int val;

	val = wkup_m3_ctrl_ipc_read(m3_rproc_static, 2);
	val &= (~WKUP_M3_FW_VERSION_MASK);
	wkup_m3_ctrl_ipc_write(m3_rproc_static, val, 2);
}

void wkup_m3_set_rtc_only_mode(void)
{
	m3_rproc_static->is_rtc_only = true;
}
EXPORT_SYMBOL(wkup_m3_set_rtc_only_mode);

static int wkup_m3_rproc_start(struct rproc *rproc)
{
	struct wkup_m3_rproc *m3_rproc = rproc->priv;
	struct platform_device *pdev = m3_rproc->pdev;
	struct device *dev = &pdev->dev;
	struct wkup_m3_platform_data *pdata = dev->platform_data;
	int ret;

	wkup_m3_fw_version_clear();

	ret = pdata->deassert_reset(pdev, pdata->reset_name);
	if (ret) {
		dev_err(dev, "Unable to reset wkup_m3!\n");
		return -ENODEV;
	}

	m3_rproc->mbox_client.dev = dev;
	m3_rproc->mbox_client.tx_done = NULL;
	m3_rproc->mbox_client.rx_callback = NULL;
	m3_rproc->mbox_client.tx_block = false;
	m3_rproc->mbox_client.knows_txdone = false;

	m3_rproc->mbox = mbox_request_channel(&m3_rproc->mbox_client, 0);

	if (IS_ERR(m3_rproc->mbox)) {
		dev_err(dev, "IPC Request for A8->M3 Channel failed!\n");
		ret = PTR_ERR(m3_rproc->mbox);
		m3_rproc->mbox = NULL;
		return ret;
	}

	if (wkup_m3_pm_ops && wkup_m3_pm_ops->rproc_ready &&
	    !m3_rproc_static->is_rtc_only)
		wkup_m3_pm_ops->rproc_ready(&m3_rproc_static->pdev->dev);

	m3_rproc_static->is_active = 1;

	return 0;
}

static int wkup_m3_rproc_stop(struct rproc *rproc)
{
	struct wkup_m3_rproc *m3_rproc = rproc->priv;
	struct platform_device *pdev = m3_rproc->pdev;
	struct device *dev = &pdev->dev;
	struct wkup_m3_platform_data *pdata = dev->platform_data;

	mbox_free_channel(m3_rproc_static->mbox);

	pdata->assert_reset(pdev, pdata->reset_name);

	return 0;
}

static struct rproc_ops wkup_m3_rproc_ops = {
	.start		= wkup_m3_rproc_start,
	.stop		= wkup_m3_rproc_stop,
};

/* Public Functions */

/**
 * wkup_m3_copy_aux_data - Copy auxillary data to special region of m3 dmem
 * @data - pointer to data
 * @sz - size of data to copy (limit 256 bytes)
 *
 * Copies any additional blob of data to the wkup_m3 dmem to be used by the
 * firmware
 */
unsigned long wkup_m3_copy_aux_data(const void *data, int sz)
{
	unsigned long aux_data_dev_addr;
	void *aux_data_addr;

	aux_data_dev_addr = WKUP_M3_DMEM_START + WKUP_M3_AUXDATA_OFFSET;
	aux_data_addr = rproc_da_to_va(m3_rproc_static->rproc,
				       aux_data_dev_addr,
				       WKUP_M3_AUXDATA_SIZE);
	memcpy(aux_data_addr, data, sz);

	return WKUP_M3_AUXDATA_OFFSET;
}

/**
 * wkup_m3_set_ops - Set callbacks for user of rproc
 * @ops - struct wkup_m3_ops *
 *
 * Registers callbacks to wkup_m3 to be invoked after rproc is ready to use
 * and after an interrupt is handled.
 */
void wkup_m3_set_ops(struct wkup_m3_ops *ops)
{
	wkup_m3_pm_ops = ops;

	if (m3_rproc_static && m3_rproc_static->is_active &&
	    wkup_m3_pm_ops && wkup_m3_pm_ops->rproc_ready)
		wkup_m3_pm_ops->rproc_ready(&m3_rproc_static->pdev->dev);
}

/**
 * wkup_m3_ping - Send a dummy msg to wkup_m3 to tell to to check IPC regs
 *
 * Returns the result of sending mbox msg or -EIO if no mbox handle is present
 */
int wkup_m3_ping(void)
{
	int ret;
	mbox_msg_t dummy_msg = 0;

	if (!m3_rproc_static->mbox) {
		dev_err(&m3_rproc_static->pdev->dev,
			"No IPC channel to communicate with wkup_m3!\n");
		return -EIO;
	}

	/*
	 * Write a dummy message to the mailbox in order to trigger the RX
	 * interrupt to alert the M3 that data is available in the IPC
	 * registers. We must enable the IRQ here and disable it after in
	 * the RX callback to avoid multiple interrupts being received
	 * by the CM3.
	 */
	ret = mbox_send_message(m3_rproc_static->mbox, (void *)dummy_msg);
	if (ret < 0) {
		pr_err("%s: mbox_send_message() failed: %d\n", __func__, ret);
		return ret;
	}

	mbox_client_txdone(m3_rproc_static->mbox, 0);
	return 0;
}

/**
 * wkup_m3_wake_src - Get the wakeup source info passed from wkup_m3
 * @wkup_m3_wakeup: struct wkup_m3_wakeup_src * gets assigned the
 *		    wakeup src value
 */
void wkup_m3_wake_src(struct wkup_m3_wakeup_src *wkup_m3_wakeup)
{
	unsigned int wakeup_src_idx;
	int j, val;

	val = wkup_m3_ctrl_ipc_read(m3_rproc_static, 6);

	wakeup_src_idx = val & WKUP_M3_WAKE_SRC_MASK;

	for (j = 0; j < ARRAY_SIZE(wakeups)-1; j++) {
		if (wakeups[j].irq_nr == wakeup_src_idx) {
			*wkup_m3_wakeup = wakeups[j];
			return;
		}
	}
	*wkup_m3_wakeup = wakeups[j];
}

/**
 * wkup_m3_pm_status - Return the status code from wkup_m3 after sleep event
 *
 * Returns an error code that indicates whether or not the dsired sleep
 * action was a success or not.
 */
int wkup_m3_pm_status(void)
{
	unsigned int i;
	int val;

	val = wkup_m3_ctrl_ipc_read(m3_rproc_static, 1);

	i = WKUP_M3_STATUS_RESP_MASK & val;
	i >>= __ffs(WKUP_M3_STATUS_RESP_MASK);

	return i;
}

/**
 * wkup_m3_fw_version_read - Return the fw version given by the wkup_m3
 *
 * After boot the fw version should be read to ensure it is compatible.
 */
int wkup_m3_fw_version_read(void)
{
	int val;

	val = wkup_m3_ctrl_ipc_read(m3_rproc_static, 2);

	return val & WKUP_M3_FW_VERSION_MASK;
}

/**
 * wkup_m3_set_cmd - write contents of struct to ipc regs
 * @ipc_regs: struct wkup_m3_ipc_regs *
 */
void wkup_m3_set_cmd(struct wkup_m3_ipc_regs *ipc_regs)
{
	wkup_m3_ctrl_ipc_write(m3_rproc_static, ipc_regs->reg0, 0);
	wkup_m3_ctrl_ipc_write(m3_rproc_static, ipc_regs->reg1, 1);
	wkup_m3_ctrl_ipc_write(m3_rproc_static, ipc_regs->reg2, 2);
	wkup_m3_ctrl_ipc_write(m3_rproc_static, ipc_regs->reg3, 3);
	wkup_m3_ctrl_ipc_write(m3_rproc_static, ipc_regs->reg4, 4);
	wkup_m3_ctrl_ipc_write(m3_rproc_static, ipc_regs->reg5, 5);
	wkup_m3_ctrl_ipc_write(m3_rproc_static, ipc_regs->reg6, 6);
	wkup_m3_ctrl_ipc_write(m3_rproc_static, ipc_regs->reg7, 7);
}

static void wkup_m3_rproc_loader_thread(struct rproc *rproc)
{
	struct wkup_m3_rproc *m3_rproc = rproc->priv;
	struct device *dev = &m3_rproc->pdev->dev;
	int ret;

	wait_for_completion(&rproc->firmware_loading_complete);

	ret = rproc_boot(rproc);
	if (ret)
		dev_err(dev, "rproc_boot failed\n");

	do_exit(0);
}
static int wkup_m3_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct wkup_m3_platform_data *pdata = dev->platform_data;
	struct wkup_m3_rproc *m3_rproc;
	struct rproc *rproc;
	int irq, ret;
	struct resource *res;
	struct task_struct *task;

	if (!wkup_m3_pm_ops)
		return -EPROBE_DEFER;

	pm_runtime_enable(&pdev->dev);

	ret = pm_runtime_get_sync(&pdev->dev);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "pm_runtime_get_sync() failed\n");
		return ret;
	}

	rproc = rproc_alloc(dev, "wkup_m3", &wkup_m3_rproc_ops,
			    "am335x-pm-firmware.elf", sizeof(*m3_rproc));
	if (!rproc)
		return -ENOMEM;

	m3_rproc = rproc->priv;
	m3_rproc->rproc = rproc;
	m3_rproc->pdev = pdev;

	m3_rproc_static = m3_rproc;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ipc_regs");
	m3_rproc->ipc_mem_base = devm_ioremap_resource(dev, res);
	if (!m3_rproc->ipc_mem_base) {
		dev_err(dev, "could not ioremap ipc_mem\n");
		ret = -EADDRNOTAVAIL;
		goto err;
	}

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource\n");
		ret = -ENXIO;
		goto err;
	}

	ret = devm_request_irq(dev, irq, wkup_m3_txev_handler,
			       IRQF_DISABLED, "wkup_m3_txev", m3_rproc);
	if (ret) {
		dev_err(dev, "request_irq failed\n");
		goto err;
	}

	if (!(pdata && pdata->deassert_reset)) {
		dev_err(dev, "Platform data missing deassert_reset!\n");
		ret = -ENODEV;
	}

	/* Register as a remoteproc device */
	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed\n");
		goto err;
	}

	/*
	 * Wait for firmware loading completion in a thread so we
	 * can boot the wkup_m3 as soon as it's ready without holding
	 * up kernel boot
	 */
	task = kthread_run((void *)wkup_m3_rproc_loader_thread, rproc,
			   "wkup_m3_rproc_loader");

	if (IS_ERR(task)) {
		dev_err(dev, "can't create rproc_loader thread\n");
		goto err;
	}

	return 0;

err:
	rproc_put(rproc);
	pm_runtime_put_sync(&pdev->dev);
	return ret;
}

static int wkup_m3_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);
	rproc_put(rproc);
	pm_runtime_put_sync(&pdev->dev);

	m3_rproc_static = NULL;

	return 0;
}

static int wkm3_suspend(struct device *dev)
{
	return 0;
}

static int wkm3_resume(struct device *dev)
{
	if (m3_rproc_static->is_rtc_only) {
		rproc_shutdown(m3_rproc_static->rproc);
		rproc_boot(m3_rproc_static->rproc);
	}

	m3_rproc_static->is_rtc_only = false;

	return 0;
}

static int wkup_m3_rpm_suspend(struct device *dev)
{
	return -EBUSY;
}

static int wkup_m3_rpm_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops wkup_m3_rproc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(wkm3_suspend, wkm3_resume)
	SET_RUNTIME_PM_OPS(wkup_m3_rpm_suspend, wkup_m3_rpm_resume, NULL)
};

static const struct of_device_id wkup_m3_rproc_of_match[] = {
	{ .compatible = "ti,am3353-wkup-m3", .data = NULL, },
	{ .compatible = "ti,am4372-wkup-m3", .data = NULL, },
	{},
};

static struct platform_driver wkup_m3_rproc_driver = {
	.probe = wkup_m3_rproc_probe,
	.remove = wkup_m3_rproc_remove,
	.driver = {
		.name = "wkup_m3",
		.owner = THIS_MODULE,
		.of_match_table = wkup_m3_rproc_of_match,
		.pm = &wkup_m3_rproc_pm_ops,
	},
};

module_platform_driver(wkup_m3_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("wkup m3 remote processor control driver");
