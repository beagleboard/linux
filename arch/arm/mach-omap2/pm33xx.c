/*
 * AM33XX Power Management Routines
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/suspend.h>

#include <plat/prcm.h>
#include <plat/mailbox.h>
#include <plat/sram.h>
#include <plat/omap_hwmod.h>
#include <plat/omap_device.h>

#include <asm/suspend.h>
#include <asm/proc-fns.h>
#include <asm/sizes.h>

#include "pm.h"
#include "pm33xx.h"

void (*am33xx_do_wfi_sram)(void);

#define DS_MODE		DS0_ID	/* DS0/1_ID */

#ifdef CONFIG_SUSPEND
static int m3_state;
struct omap_mbox *m3_mbox;
void __iomem *ipc_regs;
void __iomem *m3_eoi;
void __iomem *m3_code;

static struct device *mpu_dev;
bool enable_deep_sleep = true;

static int global_suspend_flag = 0;

static suspend_state_t suspend_state = PM_SUSPEND_ON;

struct a8_wkup_m3_ipc_data {
	int resume_addr;
	int sleep_mode;
	int ipc_data1;
	int ipc_data2;
} am33xx_lp_ipc;

static int am33xx_set_low_power_state(struct a8_wkup_m3_ipc_data *);
static void am33xx_verify_lp_state(void);


static int am33xx_do_sram_idle(long unsigned int state)
{
	am33xx_do_wfi_sram();
	return 0;
}

static inline bool is_suspending(void)
{
	return (suspend_state != PM_SUSPEND_ON) && console_suspend_enabled;
}

static int am33xx_pm_suspend(void)
{
	int ret = 0;

	struct omap_hwmod *cpgmac_oh, *gpmc_oh, *usb_oh;

	cpgmac_oh	= omap_hwmod_lookup("cpgmac0");
	usb_oh		= omap_hwmod_lookup("usb_otg_hs");
	gpmc_oh		= omap_hwmod_lookup("gpmc");

	omap_hwmod_enable(cpgmac_oh);
	omap_hwmod_enable(usb_oh);
	omap_hwmod_enable(gpmc_oh);

	omap_hwmod_idle(cpgmac_oh);
	omap_hwmod_idle(usb_oh);
	omap_hwmod_idle(gpmc_oh);

	ret = cpu_suspend(0, am33xx_do_sram_idle);
	if (ret)
		pr_err("Could not suspend\n");

	return ret;
}

static int am33xx_pm_enter(suspend_state_t unused)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = am33xx_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int am33xx_pm_begin(suspend_state_t state)
{
	int ret = 0;

	am33xx_lp_ipc.resume_addr = DS_RESUME_ADDR;
	am33xx_lp_ipc.sleep_mode  = DS_MODE;
	am33xx_lp_ipc.ipc_data1	  = DS_IPC_DEFAULT;
	am33xx_lp_ipc.ipc_data2   = DS_IPC_DEFAULT;

	am33xx_set_low_power_state(&am33xx_lp_ipc);

	ret = omap_mbox_msg_send(m3_mbox, 0xABCDABCD);
	if (!ret) {
		pr_info("Message sent for entering %s\n",
			(DS_MODE == DS0_ID ? "DS0" : "DS1"));
		omap_mbox_msg_rx_flush(m3_mbox);
	}

	omap_mbox_disable_irq(m3_mbox, IRQ_RX);

	suspend_state = state;

	return ret;
}

static void am33xx_m3_state_machine_reset(void)
{
	int ret = 0;

	am33xx_lp_ipc.resume_addr = 0x0;
	am33xx_lp_ipc.sleep_mode  = 0xe;
	am33xx_lp_ipc.ipc_data1	  = DS_IPC_DEFAULT;
	am33xx_lp_ipc.ipc_data2   = DS_IPC_DEFAULT;

	am33xx_set_low_power_state(&am33xx_lp_ipc);

	ret = omap_mbox_msg_send(m3_mbox, 0xABCDABCD);
	if (!ret) {
		pr_debug("Message sent for resetting M3 state machine\n");
		omap_mbox_msg_rx_flush(m3_mbox);
	}
}

static void am33xx_pm_end(void)
{
	suspend_state = PM_SUSPEND_ON;

	/* Check the global suspend flag followed by the IPC register */
	am33xx_verify_lp_state();

	/* TODO: This should be handled via some MBX API */
	if (m3_mbox->ops->ack_irq)
		m3_mbox->ops->ack_irq(m3_mbox, IRQ_RX);

	omap_mbox_enable_irq(m3_mbox, IRQ_RX);

	/* M3 state machine will get reset in a successful iteration,
	 * for now we go ahead and reset it again to catch the bad
	 * iterations
	 */
	am33xx_m3_state_machine_reset();

	return;
}

static const struct platform_suspend_ops am33xx_pm_ops = {
	.begin		= am33xx_pm_begin,
	.end		= am33xx_pm_end,
	.enter		= am33xx_pm_enter,
	.valid		= suspend_valid_only_mem,
};

int am33xx_set_low_power_state(struct a8_wkup_m3_ipc_data *data)
{
	writel(data->resume_addr, ipc_regs);
	writel(data->sleep_mode, ipc_regs + 0x4);
	writel(data->ipc_data1, ipc_regs + 0x8);
	writel(data->ipc_data2, ipc_regs + 0xc);

	return 0;
}

static void am33xx_verify_lp_state(void)
{
	int status;

	if (global_suspend_flag) {
		pr_err("Kernel core reported suspend failure\n");
		goto clear_old_status;
	}

	/* If it's a failed transition and we check the old status,
	 * the failure will be erroneoulsy logged as a pass
	 * and the worst part is that the next WFI in the idle loop
	 * will be intercepted by M3 as a signal to cut-off
	 * the power to A8
	 *
	 * So, we MUST reset the M3 state machine even if the
	 * result is pass. Other option could be to clear the
	 * the CMD_STAT bits in the resume path and that also
	 * should be done
	 */
	status = readl(ipc_regs + 0x4);
	status &= 0xffff0000;

	if (status == 0x0)
		pr_info("DeepSleep transition passed\n");
	else if (status == 0x10000)
		pr_info("DeepSleep transition failed\n");
	else
		pr_info("Status = %0x\n", status);


clear_old_status:
	/* After decoding we write back the bad status */
	status = readl(ipc_regs + 0x4);
	status &= 0xffff0000;
	status |= 0x10000;
	writel(status, ipc_regs + 0x4);
}

/*
 * Dummy notifier for the mailbox
 * TODO: Can this be completely removed?
 */
int wkup_m3_mbox_msg(struct notifier_block *self, unsigned long len, void *msg)
{
	return 0;
}

static struct notifier_block wkup_m3_mbox_notifier = {
	.notifier_call = wkup_m3_mbox_msg,
};

/* Interrupt from M3 to A8 */
static irqreturn_t wkup_m3_txev_handler(int irq, void *unused)
{
	m3_state++;

	if (m3_eoi) {
		writel(0x1, m3_eoi);
		writel(0x0, m3_eoi);
		return IRQ_HANDLED;
	} else {
		pr_err("%s unexpected interrupt. "
		"Something is seriously wrong\n", __func__);
		return IRQ_NONE;
	}
}

/* Initiliaze WKUP_M3, load the binary blob and let it run */
static int wkup_m3_init(void)
{
	struct clk *m3_clk;
	struct omap_hwmod *wkup_m3_oh;
	const struct firmware *firmware;
	int ret = 0;

	wkup_m3_oh = omap_hwmod_lookup("wkup_m3");

	if (!wkup_m3_oh) {
		pr_err("%s: could not find omap_hwmod\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	ipc_regs = ioremap(A8_M3_IPC_REGS, 0x4*8);
	if (!ipc_regs) {
		pr_err("Could not ioremap the IPC area\b");
		ret = -ENOMEM;
		goto exit;
	}

	m3_eoi = ioremap(M3_TXEV_EOI, 0x4);
	if (!m3_eoi) {
		pr_err("Could not ioremap the EOI register\n");
		ret = -ENOMEM;
		goto err1;
	}

	/* Reserve the MBOX for sending messages to M3 */
	m3_mbox = omap_mbox_get("wkup_m3", &wkup_m3_mbox_notifier);
	if (IS_ERR(m3_mbox)) {
		pr_err("Could not reserve mailbox for A8->M3 IPC\n");
		ret = -ENODEV;
		goto err2;
	}

	/* Enable access to the M3 code and data area from A8 */
	m3_clk = clk_get(NULL, "wkup_m3_fck");
	if (IS_ERR(m3_clk)) {
		pr_err("%s failed to enable WKUP_M3 clock\n", __func__);
		goto err3;
	}

	if (clk_enable(m3_clk)) {
		pr_err("%s WKUP_M3: clock enable Failed\n", __func__);
		goto err4;
	}

	m3_code = ioremap(M3_UMEM, SZ_16K);
	if (!m3_code) {
		pr_err("%s Could not ioremap M3 code space\n", __func__);
		ret = -ENOMEM;
		goto err5;
	}

	/* Now try to load the firware */
	ret = request_firmware(&firmware, "cm3-firmware.bin", mpu_dev);
	if (ret < 0) {
		dev_err(mpu_dev, "request_firmware failed\n");
		goto err6;
	} else {
		memcpy(m3_code, firmware->data, firmware->size);
		pr_info("Copied the M3 firmware to UMEM\n");
	}

	ret = omap_hwmod_deassert_hardreset(wkup_m3_oh, "wkup_m3");
	if (ret) {
		pr_err("Could not deassert the reset for WKUP_M3\n");
		goto err6;
	}

	ret = request_irq(AM33XX_IRQ_M3_M3SP_TXEV, wkup_m3_txev_handler,
			  IRQF_DISABLED, "wkup_m3_txev", NULL);
	if (ret)
		pr_err("%s request_irq failed for 0x%x\n", __func__,
			AM33XX_IRQ_M3_M3SP_TXEV);
	else
		return 0;

err6:
	release_firmware(firmware);
	iounmap(m3_code);
err5:
	clk_disable(m3_clk);
err4:
	clk_put(m3_clk);
err3:
	omap_mbox_put(m3_mbox, &wkup_m3_mbox_notifier);
err2:
	iounmap(m3_eoi);
err1:
	iounmap(ipc_regs);
exit:
	return ret;
}
#endif /* CONFIG_SUSPEND */

/*
 * Push the minimal suspend-resume code to SRAM
 */
void am33xx_push_sram_idle(void)
{
	am33xx_do_wfi_sram = omap_sram_push(am33xx_do_wfi, am33xx_do_wfi_sz);
}

static int __init am33xx_pm_init(void)
{
	int ret;

	if (!cpu_is_am33xx())
		return -ENODEV;

	pr_info("Power Management for AM33XX family\n");

#ifdef CONFIG_SUSPEND
	mpu_dev = omap_device_get_by_hwmod_name("mpu");

	if (!mpu_dev) {
		pr_warning("%s: unable to get the mpu device\n", __func__);
		return -EINVAL;
	}

	ret = wkup_m3_init();

	if (ret) {
		pr_err("Could not initialise WKUP_M3. "
			"Power management will be compromised\n");
		enable_deep_sleep = false;
	}

	if (enable_deep_sleep)
		suspend_set_ops(&am33xx_pm_ops);
#endif /* CONFIG_SUSPEND */

	return ret;
}
late_initcall(am33xx_pm_init);
