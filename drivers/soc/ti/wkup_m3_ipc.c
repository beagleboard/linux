/*
 * AMx3 Wkup M3 IPC driver
 *
 * Copyright (C) 2015 Texas Instruments, Inc.
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

#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/omap-mailbox.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/suspend.h>
#include <linux/wkup_m3_ipc.h>

#define AM33XX_CTRL_IPC_REG_COUNT	0x8
#define AM33XX_CTRL_IPC_REG_OFFSET(m)	(0x4 + 4 * (m))

/* AM33XX M3_TXEV_EOI register */
#define AM33XX_CONTROL_M3_TXEV_EOI	0x00

#define AM33XX_M3_TXEV_ACK		(0x1 << 0)
#define AM33XX_M3_TXEV_ENABLE		(0x0 << 0)

#define IPC_CMD_DS0			0x4
#define IPC_CMD_STANDBY			0xc
#define IPC_CMD_IDLE			0x10
#define IPC_CMD_RESET			0xe
#define DS_IPC_DEFAULT			0xffffffff
#define M3_VERSION_UNKNOWN		0x0000ffff
#define M3_BASELINE_VERSION		0x191
#define M3_STATUS_RESP_MASK		(0xffff << 16)
#define M3_FW_VERSION_MASK		0xffff
#define M3_WAKE_SRC_MASK		0xff

#define IPC_MEM_TYPE_SHIFT		(0x0)
#define IPC_MEM_TYPE_MASK		(0x7 << 0)
#define IPC_VTT_STAT_SHIFT		(0x3)
#define IPC_VTT_STAT_MASK		(0x1 << 3)
#define IPC_VTT_GPIO_PIN_SHIFT		(0x4)
#define IPC_VTT_GPIO_PIN_MASK		(0x3f << 4)
#define IPC_IO_ISOLATION_STAT_SHIFT	(10)
#define IPC_IO_ISOLATION_STAT_MASK	(0x1 << 10)

#define M3_STATE_UNKNOWN		0
#define M3_STATE_RESET			1
#define M3_STATE_INITED			2
#define M3_STATE_MSG_FOR_LP		3
#define M3_STATE_MSG_FOR_RESET		4

#define WKUP_M3_SD_FW_MAGIC		0x570C

#define WKUP_M3_DMEM_START		0x80000
#define WKUP_M3_AUXDATA_OFFSET		0x1000
#define WKUP_M3_AUXDATA_SIZE		0xFF

struct wkup_m3_ipc {
	struct rproc *rproc;

	void __iomem *ipc_mem_base;
	struct device *dev;

	int mem_type;
	unsigned long resume_addr;
	int vtt_conf;
	int isolation_conf;
	int state;

	unsigned long volt_scale_offsets;
	const char *sd_fw_name;

	struct completion sync_complete;
	struct mbox_client mbox_client;
	struct mbox_chan *mbox;
	bool is_rtc_only;
};

struct wkup_m3_scale_data_header {
	u16 magic;
	u8 sleep_offset;
	u8 wake_offset;
} __packed;

static struct wkup_m3_ipc m3_ipc_state;

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

/**
 * wkup_m3_copy_aux_data - Copy auxillary data to special region of m3 dmem
 * @data - pointer to data
 * @sz - size of data to copy (limit 256 bytes)
 *
 * Copies any additional blob of data to the wkup_m3 dmem to be used by the
 * firmware
 */
static unsigned long wkup_m3_copy_aux_data(const void *data, int sz)
{
	unsigned long aux_data_dev_addr;
	void *aux_data_addr;

	aux_data_dev_addr = WKUP_M3_DMEM_START + WKUP_M3_AUXDATA_OFFSET;
	aux_data_addr = rproc_da_to_va(m3_ipc_state.rproc,
				       aux_data_dev_addr,
				       WKUP_M3_AUXDATA_SIZE, 0);
	memcpy(aux_data_addr, data, sz);

	return WKUP_M3_AUXDATA_OFFSET;
}

static void wkup_m3_scale_data_fw_cb(const struct firmware *fw, void *context)
{
	unsigned long val, aux_base;
	struct wkup_m3_scale_data_header hdr;
	struct device *dev = (struct device *)context;

	if (!fw) {
		dev_err(dev, "Voltage scale fw name given but file missing.\n");
		return;
	}

	memcpy(&hdr, fw->data, sizeof(hdr));

	if (hdr.magic != WKUP_M3_SD_FW_MAGIC) {
		dev_err(dev, "PM: Voltage Scale Data binary does not appear valid.\n");
		goto release_sd_fw;
	}

	aux_base = wkup_m3_copy_aux_data(fw->data + sizeof(hdr),
					 fw->size - sizeof(hdr));

	val = (aux_base + hdr.sleep_offset);
	val |= ((aux_base + hdr.wake_offset) << 16);

	m3_ipc_state.volt_scale_offsets = val;

release_sd_fw:
	release_firmware(fw);
};

static int wkup_m3_init_scale_data(struct device *dev)
{
	int ret = 0;

	/*
	 * If no name is provided, user has already been warned, pm will
	 * still work so return 0
	 */

	if (!m3_ipc_state.sd_fw_name)
		return ret;

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				      m3_ipc_state.sd_fw_name, dev, GFP_ATOMIC,
				      dev, wkup_m3_scale_data_fw_cb);

	return ret;
}

static void am33xx_txev_eoi(struct wkup_m3_ipc *m3_ipc)
{
	writel(AM33XX_M3_TXEV_ACK,
	       m3_ipc->ipc_mem_base + AM33XX_CONTROL_M3_TXEV_EOI);
}

static void am33xx_txev_enable(struct wkup_m3_ipc *m3_ipc)
{
	writel(AM33XX_M3_TXEV_ENABLE,
	       m3_ipc->ipc_mem_base + AM33XX_CONTROL_M3_TXEV_EOI);
}

static void wkup_m3_ctrl_ipc_write(struct wkup_m3_ipc *m3_ipc,
				   u32 val, int ipc_reg_num)
{
	if (WARN(ipc_reg_num < 0 || ipc_reg_num > AM33XX_CTRL_IPC_REG_COUNT,
		 "ipc register operation out of range"))
		return;

	writel(val, m3_ipc->ipc_mem_base +
	       AM33XX_CTRL_IPC_REG_OFFSET(ipc_reg_num));
}

static unsigned int wkup_m3_ctrl_ipc_read(struct wkup_m3_ipc *m3_ipc,
					  int ipc_reg_num)
{
	if (WARN(ipc_reg_num < 0 || ipc_reg_num > AM33XX_CTRL_IPC_REG_COUNT,
		 "ipc register operation out of range"))
		return 0;

	return readl(m3_ipc->ipc_mem_base +
		     AM33XX_CTRL_IPC_REG_OFFSET(ipc_reg_num));
}

static int wkup_m3_fw_version_read(struct wkup_m3_ipc *m3_ipc)
{
	int val;

	val = wkup_m3_ctrl_ipc_read(m3_ipc, 2);

	return val & M3_FW_VERSION_MASK;
}

static irqreturn_t wkup_m3_txev_handler(int irq, void *ipc_data)
{
	struct wkup_m3_ipc *m3_ipc = ipc_data;
	struct device *dev = m3_ipc->dev;
	int ver = 0;

	am33xx_txev_eoi(m3_ipc);

	switch (m3_ipc->state) {
	case M3_STATE_RESET:
		ver = wkup_m3_fw_version_read(m3_ipc);

		if (ver == M3_VERSION_UNKNOWN ||
		    ver < M3_BASELINE_VERSION) {
			dev_warn(dev, "CM3 Firmware Version %x not supported\n",
				 ver);
		} else {
			dev_info(dev, "CM3 Firmware Version = 0x%x\n", ver);
		}

		m3_ipc->state = M3_STATE_INITED;
		wkup_m3_init_scale_data(dev);
		complete(&m3_ipc->sync_complete);
		break;
	case M3_STATE_MSG_FOR_RESET:
		m3_ipc->state = M3_STATE_INITED;
		complete(&m3_ipc->sync_complete);
		break;
	case M3_STATE_MSG_FOR_LP:
		complete(&m3_ipc->sync_complete);
		break;
	case M3_STATE_UNKNOWN:
		dev_warn(dev, "Unknown CM3 State\n");
	}

	am33xx_txev_enable(m3_ipc);

	return IRQ_HANDLED;
}

static int wkup_m3_ping(struct wkup_m3_ipc *m3_ipc)
{
	struct device *dev = m3_ipc->dev;
	mbox_msg_t dummy_msg = 0;
	int ret;

	if (!m3_ipc->mbox) {
		dev_err(dev,
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
	ret = mbox_send_message(m3_ipc->mbox, &dummy_msg);
	if (ret < 0) {
		dev_err(dev, "%s: mbox_send_message() failed: %d\n",
			__func__, ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&m3_ipc->sync_complete,
					  msecs_to_jiffies(500));
	if (!ret) {
		dev_err(dev, "MPU<->CM3 sync failure\n");
		m3_ipc->state = M3_STATE_UNKNOWN;
		return -EIO;
	}

	mbox_client_txdone(m3_ipc->mbox, 0);
	return 0;
}

static int wkup_m3_ping_noirq(struct wkup_m3_ipc *m3_ipc)
{
	struct device *dev = m3_ipc->dev;
	mbox_msg_t dummy_msg = 0;
	int ret;

	if (!m3_ipc->mbox) {
		dev_err(dev,
			"No IPC channel to communicate with wkup_m3!\n");
		return -EIO;
	}

	ret = mbox_send_message(m3_ipc->mbox, &dummy_msg);
	if (ret < 0) {
		dev_err(dev, "%s: mbox_send_message() failed: %d\n",
			__func__, ret);
		return ret;
	}

	mbox_client_txdone(m3_ipc->mbox, 0);
	return 0;
}

static int wkup_m3_is_available(struct wkup_m3_ipc *m3_ipc)
{
	return (m3_ipc->state != M3_STATE_RESET) && (m3_ipc->state != M3_STATE_UNKNOWN);
}

static void wkup_m3_set_vtt_gpio(int gpio)
{
	m3_ipc_state.vtt_conf = (1 << IPC_VTT_STAT_SHIFT) |
				(gpio << IPC_VTT_GPIO_PIN_SHIFT);
}

static void wkup_m3_set_io_isolation(void)
{
	m3_ipc_state.isolation_conf = (1 << IPC_IO_ISOLATION_STAT_SHIFT);
}

/* Public functions */
/**
 * wkup_m3_set_mem_type - Pass wkup_m3 which type of memory is in use
 * @mem_type: memory type value read directly from emif
 *
 * wkup_m3 must know what memory type is in use to properly suspend
 * and resume.
 */
void wkup_m3_set_mem_type(int mem_type)
{
	m3_ipc_state.mem_type = mem_type;
}
EXPORT_SYMBOL_GPL(wkup_m3_set_mem_type);

/**
 * wkup_m3_set_resume_address - Pass wkup_m3 resume address
 * @addr: Physical address from which resume code should execute
 */
void wkup_m3_set_resume_address(void *addr)
{
	m3_ipc_state.resume_addr = (unsigned long)addr;
}
EXPORT_SYMBOL_GPL(wkup_m3_set_resume_address);

void wkup_m3_set_rtc_only_mode(void)
{
	m3_ipc_state.is_rtc_only = true;
}
EXPORT_SYMBOL(wkup_m3_set_rtc_only_mode);

/**
 * wkup_m3_request_pm_status - Retrieve wkup_m3 status code after suspend
 *
 * Returns code representing the status of a low power mode transition.
 *	0 - Successful transition
 *	1 - Failure to transition to low power state
 */
int wkup_m3_request_pm_status(void)
{
	struct wkup_m3_ipc *m3_ipc = &m3_ipc_state;
	unsigned int i;
	int val;

	val = wkup_m3_ctrl_ipc_read(m3_ipc, 1);

	i = M3_STATUS_RESP_MASK & val;
	i >>= __ffs(M3_STATUS_RESP_MASK);

	return i;
}
EXPORT_SYMBOL_GPL(wkup_m3_request_pm_status);

/**
 * wkup_m3_prepare_low_power - Request preparation for transition to
 *			       low power state
 * @state: A kernel suspend state to enter, either MEM or STANDBY
 *
 * Returns 0 if preparation was successful, otherwise returns error code
 */
int wkup_m3_prepare_low_power(int state)
{
	struct wkup_m3_ipc *m3_ipc = &m3_ipc_state;
	struct device *dev = m3_ipc_state.dev;
	int m3_power_state;
	int ret = 0;

	if (!wkup_m3_is_available(m3_ipc))
		return -ENODEV;

	switch (state) {
	case PM_SUSPEND_MEM:
		wkup_m3_ctrl_ipc_write(m3_ipc, m3_ipc->volt_scale_offsets, 5);
		m3_power_state = IPC_CMD_DS0;
		break;
	case PM_SUSPEND_STANDBY:
		m3_power_state = IPC_CMD_STANDBY;
		wkup_m3_ctrl_ipc_write(m3_ipc, DS_IPC_DEFAULT, 5);
		break;
	case WKUP_M3_IDLE:
		m3_power_state = IPC_CMD_IDLE;
		wkup_m3_ctrl_ipc_write(m3_ipc, DS_IPC_DEFAULT, 5);
		break;
	default:
		return 1;
	}

	/* Program each required IPC register then write defaults to others */
	wkup_m3_ctrl_ipc_write(m3_ipc, m3_ipc_state.resume_addr, 0);
	wkup_m3_ctrl_ipc_write(m3_ipc, m3_power_state, 1);
	wkup_m3_ctrl_ipc_write(m3_ipc, m3_ipc_state.mem_type |
			       m3_ipc_state.vtt_conf |
			       m3_ipc_state.isolation_conf, 4);

	wkup_m3_ctrl_ipc_write(m3_ipc, DS_IPC_DEFAULT, 2);
	wkup_m3_ctrl_ipc_write(m3_ipc, DS_IPC_DEFAULT, 3);
	wkup_m3_ctrl_ipc_write(m3_ipc, DS_IPC_DEFAULT, 6);
	wkup_m3_ctrl_ipc_write(m3_ipc, DS_IPC_DEFAULT, 7);

	m3_ipc->state = M3_STATE_MSG_FOR_LP;

	if (state == WKUP_M3_IDLE)
		ret = wkup_m3_ping_noirq(m3_ipc);
	else
		ret = wkup_m3_ping(m3_ipc);

	if (ret) {
		dev_err(dev, "Unable to ping CM3\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(wkup_m3_prepare_low_power);

/**
 * wkup_m3_finish_low_power - Return m3 to reset state
 *
 * Returns 0 if reset was successful, otherwise returns error code
 */
int wkup_m3_finish_low_power(void)
{
	struct wkup_m3_ipc *m3_ipc = &m3_ipc_state;
	struct device *dev = m3_ipc_state.dev;
	int ret = 0;

	if (!wkup_m3_is_available(m3_ipc))
		return -ENODEV;

	wkup_m3_ctrl_ipc_write(m3_ipc, IPC_CMD_RESET, 1);
	wkup_m3_ctrl_ipc_write(m3_ipc, DS_IPC_DEFAULT, 2);

	m3_ipc->state = M3_STATE_MSG_FOR_RESET;

	ret = wkup_m3_ping(m3_ipc);
	if (ret) {
		dev_err(dev, "Unable to ping CM3\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(wkup_m3_finish_low_power);

/**
 * wkup_m3_wake_src - Get the wakeup source info passed from wkup_m3
 * @wkup_m3_wakeup: struct wkup_m3_wakeup_src * gets assigned the
 *		    wakeup src value
 */
const char *wkup_m3_request_wake_src(void)
{
	struct wkup_m3_ipc *m3_ipc = &m3_ipc_state;
	unsigned int wakeup_src_idx;
	int j, val;

	val = wkup_m3_ctrl_ipc_read(m3_ipc, 6);

	wakeup_src_idx = val & M3_WAKE_SRC_MASK;

	for (j = 0; j < ARRAY_SIZE(wakeups) - 1; j++) {
		if (wakeups[j].irq_nr == wakeup_src_idx)
			return wakeups[j].src;
	}
	return wakeups[j].src;
}
EXPORT_SYMBOL(wkup_m3_request_wake_src);

static void wkup_m3_rproc_boot_thread(struct rproc *rproc)
{
	struct device *dev = &rproc->dev;
	int ret;

	wait_for_completion(&rproc->firmware_loading_complete);

	init_completion(&m3_ipc_state.sync_complete);

	ret = rproc_boot(rproc);
	if (ret)
		dev_err(dev, "rproc_boot failed\n");

	do_exit(0);
}

static int wkup_m3_ipc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int irq, ret, temp;
	uint32_t rproc_phandle;
	struct rproc *m3_rproc;
	struct resource *res;
	struct task_struct *task;
	struct device_node *np = dev->of_node;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	m3_ipc_state.ipc_mem_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(m3_ipc_state.ipc_mem_base)) {
		dev_err(dev, "could not ioremap ipc_mem\n");
		ret = PTR_ERR(m3_ipc_state.ipc_mem_base);
		return ret;
	}

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource\n");
		ret = -ENXIO;
		return ret;
	}

	ret = devm_request_irq(dev, irq, wkup_m3_txev_handler,
			       0, "wkup_m3_txev", &m3_ipc_state);
	if (ret) {
		dev_err(dev, "request_irq failed\n");
		return ret;
	}

	m3_ipc_state.mbox_client.dev = dev;
	m3_ipc_state.mbox_client.tx_done = NULL;
	m3_ipc_state.mbox_client.rx_callback = NULL;
	m3_ipc_state.mbox_client.tx_block = false;
	m3_ipc_state.mbox_client.knows_txdone = false;

	m3_ipc_state.mbox = mbox_request_channel(&m3_ipc_state.mbox_client, 0);
	if (IS_ERR(m3_ipc_state.mbox)) {
		dev_err(dev, "IPC Request for A8->M3 Channel failed! %ld\n",
			PTR_ERR(m3_ipc_state.mbox));
		ret = PTR_ERR(m3_ipc_state.mbox);
		m3_ipc_state.mbox = NULL;
		return ret;
	}

	if (of_property_read_u32(dev->of_node, "ti,rproc", &rproc_phandle)) {
		dev_err(&pdev->dev, "could not get rproc phandle\n");
		ret = -ENODEV;
		goto err;
	}

	m3_rproc = rproc_get_by_phandle(rproc_phandle);
	if (!m3_rproc) {
		dev_err(&pdev->dev, "could not get rproc handle\n");
		ret = -EPROBE_DEFER;
		goto err;
	}

	m3_ipc_state.rproc = m3_rproc;
	m3_ipc_state.dev = dev;
	m3_ipc_state.state = M3_STATE_RESET;

	if (of_find_property(np, "ti,needs-vtt-toggle", NULL) &&
	    !(of_property_read_u32(np, "ti,vtt-gpio-pin", &temp))) {
		if (temp >= 0 && temp <= 31)
			wkup_m3_set_vtt_gpio(temp);
		else
			dev_warn(dev, "Invalid VTT GPIO(%d) pin\n", temp);
	}

	if (of_find_property(np, "ti,set-io-isolation", NULL))
		wkup_m3_set_io_isolation();

	ret = of_property_read_string(np, "ti,scale-data-fw",
				      &m3_ipc_state.sd_fw_name);
	if (ret) {
		dev_dbg(dev, "Voltage scaling data blob not provided from DT.\n");
	};

	/*
	 * Wait for firmware loading completion in a thread so we
	 * can boot the wkup_m3 as soon as it's ready without holding
	 * up kernel boot
	 */
	task = kthread_run((void *)wkup_m3_rproc_boot_thread, m3_rproc,
			   "wkup_m3_rproc_loader");

	if (IS_ERR(task)) {
		dev_err(dev, "can't create rproc_boot thread\n");
		goto err_put_rproc;
	}

	return 0;

err_put_rproc:
	rproc_put(m3_rproc);
err:
	mbox_free_channel(m3_ipc_state.mbox);
	return ret;
}

static int wkup_m3_ipc_remove(struct platform_device *pdev)
{
	mbox_free_channel(m3_ipc_state.mbox);

	rproc_shutdown(m3_ipc_state.rproc);
	rproc_put(m3_ipc_state.rproc);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int wkup_m3_ipc_resume(struct device *dev)
{
	if (m3_ipc_state.is_rtc_only) {
		rproc_shutdown(m3_ipc_state.rproc);
		rproc_boot(m3_ipc_state.rproc);
	}

	m3_ipc_state.is_rtc_only = false;

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops wkup_m3_ipc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(NULL, wkup_m3_ipc_resume)
};

static const struct of_device_id wkup_m3_ipc_of_match[] = {
	{ .compatible = "ti,am3352-wkup-m3-ipc", },
	{ .compatible = "ti,am4372-wkup-m3-ipc", },
	{},
};

static struct platform_driver wkup_m3_ipc_driver = {
	.probe = wkup_m3_ipc_probe,
	.remove = wkup_m3_ipc_remove,
	.driver = {
		.name = "wkup_m3_ipc",
		.of_match_table = wkup_m3_ipc_of_match,
		.pm = &wkup_m3_ipc_pm_ops,
	},
};

module_platform_driver(wkup_m3_ipc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("wkup m3 remote processor ipc driver");
MODULE_AUTHOR("Dave Gerlach <d-gerlach@ti.com>");
MODULE_DEVICE_TABLE(of, wkup_m3_ipc_of_match);
