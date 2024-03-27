// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2008-2010 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#include <linux/slab.h>
#include <linux/export.h>
#include <linux/firmware.h>

#include "debug.h"
#include "acx.h"
#include "boot.h"
#include "io.h"
#include "event.h"
#include "rx.h"
#include "tx.h"
#include "init.h"


#define CC33XX_BOOT_TIMEOUT 2000

struct hwinfo_bitmap
{
    u32 disable_5g                  : 1u;
    u32 disable_6g                  : 1u;
    u32 disable_ble                 : 1u;
    u32 disable_ble_m0plus          : 1u;
    u32 disable_m33                 : 1u;
    u64 udi                         : 64u;
    u32 pg_version                  : 4u;
    u32 metal_version               : 4u;
    u32 boot_rom_version            : 4u;
    u32 m3_rom_version              : 4u;
    u32 fuse_rom_structure_version  : 4u;
    u64 mac_address                 : 48u;
    u32 device_part_number          : 6u;
    u32 package_type                : 4u;
    u32 fw_rollback_protection_1    : 32u;
    u32 fw_rollback_protection_2    : 32u;
    u32 fw_rollback_protection_3    : 32u;
    u32 reserved                    : 13u;
} /* Aligned with boot code, must not be __packed */;


union hw_info
{
    struct hwinfo_bitmap	bitmap;
    u8 				bytes[sizeof (struct hwinfo_bitmap)] ;
};


int wlcore_boot_upload_nvs(struct cc33xx *wl);

int cc33xx_hw_init(struct cc33xx *wl);

/* Called from threaded irq context */
void cc33xx_handle_boot_irqs(struct cc33xx *wl, u32 pending_interrupts)
{
	if (WARN_ON(!wl->fw_download))
		return;

	cc33xx_debug(DEBUG_BOOT, "BOOT IRQs: 0x%x", pending_interrupts);

	atomic_or(pending_interrupts, &wl->fw_download->pending_irqs);
	complete(&wl->fw_download->wait_on_irq);
}

int wlcore_boot_run_firmware(struct cc33xx *wl)
{
	cc33xx_debug(DEBUG_CC33xx, "FW reset not implemented");


	cc33xx_debug(DEBUG_CC33xx, "Skipping wait for init complete");

	return 0;
}

static u8 * fetch_container(struct cc33xx *wl, const char* container_name,
						size_t *container_len)
{
	u8 *container_data=NULL;
	const struct firmware *container;
	int ret;

	ret = request_firmware(&container, container_name, wl->dev);

	if (ret < 0) {
		cc33xx_error("could not get container %s: (%d)",
				container_name, ret);
		return NULL;
	}

	if (container->size % 4) {
		cc33xx_error("container size is not word-aligned: %zu",
			     container->size);
		goto out;
	}


	*container_len = container->size;
	container_data = vmalloc(container->size);

	if (!container_data) {
		cc33xx_error("could not allocate memory for the container");
		goto out;
	}

	memcpy(container_data, container->data, container->size);

out:
	release_firmware(container);
	return container_data;
}

static int cc33xx_set_power_on(struct cc33xx *wl)
{
	int ret;

	msleep(CC33XX_PRE_POWER_ON_SLEEP);
	ret = cc33xx_power_on(wl);
	if (ret < 0)
		goto out;
	msleep(CC33XX_POWER_ON_SLEEP);
	cc33xx_io_reset(wl);
	cc33xx_io_init(wl);

out:
	return ret;
}


static int cc33xx_chip_wakeup(struct cc33xx *wl)
{
	int ret = 0;

	cc33xx_debug(DEBUG_BOOT, "Chip wakeup");

	ret = cc33xx_set_power_on(wl);
	if (ret < 0)
		goto out;

	/*
	 * For wl127x based devices we could use the default block
	 * size (512 bytes), but due to a bug in the sdio driver, we
	 * need to set it explicitly after the chip is powered on.  To
	 * simplify the code and since the performance impact is
	 * negligible, we use the same block size for all different
	 * chip types.
	 *
	 * Check if the bus supports blocksize alignment and, if it
	 * doesn't, make sure we don't have the quirk.
	 */
	if (!cc33xx_set_block_size(wl))
		wl->quirks &= ~WLCORE_QUIRK_TX_BLOCKSIZE_ALIGN;

	/* TODO: make sure the lower driver has set things up correctly */

out:
	return ret;
}

static int wait_for_boot_irq(struct cc33xx *wl,
				u32 boot_irq_mask, unsigned long timeout)
{
	int ret;
	u32 pending_irqs;
	struct cc33xx_fw_download *fw_download;

	fw_download = wl->fw_download;

	ret = wait_for_completion_interruptible_timeout(
			&fw_download->wait_on_irq, msecs_to_jiffies(timeout));

	// Fetch pending IRQs while clearing them in fw_download
	pending_irqs = atomic_fetch_and(0, &fw_download->pending_irqs);
	pending_irqs &= ~HINT_COMMAND_COMPLETE;

	reinit_completion(&fw_download->wait_on_irq);

	if (ret == 0){
		cc33xx_error("boot IRQ timeout");
		return -1;
	}
	else if (ret < 0){
		cc33xx_error("boot IRQ completion error %d", ret);
		return -2;
	}

	if (boot_irq_mask != pending_irqs){
		cc33xx_error("Unexpected IRQ received @ boot: 0x%x",
		pending_irqs);
		return -3;
	}

	return 0;
}

static int download_container(struct cc33xx *wl, u8 *container, size_t len)
{
	int ret;
	u8 *current_transfer;
	size_t current_transfer_size;
	u8 *const container_end = container + len;
	size_t max_transfer_size = wl->fw_download->max_transfer_size;
	bool is_last_transfer;

	current_transfer = container;

	while (current_transfer < container_end){
		current_transfer_size = container_end - current_transfer;
		current_transfer_size =
			min(current_transfer_size, max_transfer_size);

		is_last_transfer = (current_transfer + current_transfer_size >= container_end);

		ret = cmd_download_container_chunk(
			wl, current_transfer, current_transfer_size, is_last_transfer);

		current_transfer += current_transfer_size;

		if (ret < 0) {
			cc33xx_error("Chunk transfer failed");
			goto out;
		}
	}

out:
	return ret;
}

static int container_download_and_wait(
	struct cc33xx *wl,
	const char* container_name,
	const u32 irq_wait_mask)
{
	int ret=-1;
	u8 *container_data;
	size_t container_len;

	cc33xx_debug(DEBUG_BOOT,
		"Downloading %s to device", container_name);

	container_data = fetch_container(wl, container_name, &container_len);
	if (!container_data)
		return ret;

	ret = download_container(wl, container_data, container_len);
	if (ret < 0){
		cc33xx_error("Transfer error while downloading %s",
				container_name);
		goto out;
	}

	ret = wait_for_boot_irq(wl, irq_wait_mask, CC33XX_BOOT_TIMEOUT);

	if (ret < 0){
		cc33xx_error("%s boot signal timeout", container_name);
		goto out;
	}

	cc33xx_debug(DEBUG_BOOT, "%s loaded successfully", container_name);
	ret=0;

out:
	vfree(container_data);
	return ret;
}

static int fw_download_alloc(struct cc33xx *wl)
{
	if (WARN_ON(wl->fw_download != NULL))
		return -EFAULT;

	wl->fw_download = kzalloc(sizeof(*wl->fw_download), GFP_KERNEL);
	if (!wl->fw_download)
		return -ENOMEM;

	init_completion(&wl->fw_download->wait_on_irq);

	return 0;
}

static void fw_download_free(struct cc33xx *wl)
{
	if (WARN_ON(wl->fw_download == NULL))
		return;

	kfree(wl->fw_download);
	wl->fw_download = NULL;
}

int get_device_info(struct cc33xx *wl)
{
	int ret;
	union hw_info hw_info;
	u64 mac_address;

	ret = cmd_get_device_info(wl, hw_info.bytes, sizeof hw_info.bytes);
	if (ret < 0)
		return ret;

	cc33xx_debug(DEBUG_BOOT, "CC33XX device info: "
		"PG version: %d, Metal version: %d, Boot ROM version: %d "
		"M3 ROM version: %d, MAC address: 0x%llx, Device part number: %d",
		hw_info.bitmap.pg_version, hw_info.bitmap.metal_version,
		hw_info.bitmap.boot_rom_version, hw_info.bitmap.m3_rom_version,
		(u64) hw_info.bitmap.mac_address, hw_info.bitmap.device_part_number);

	wl->fw_download->max_transfer_size = 640;

	mac_address = hw_info.bitmap.mac_address;

	wl->fuse_rom_structure_version = hw_info.bitmap.fuse_rom_structure_version;
	wl->pg_version = hw_info.bitmap.pg_version;
	wl->device_part_number = hw_info.bitmap.device_part_number;
	wl->disable_5g = hw_info.bitmap.disable_5g;
	wl->disable_6g = hw_info.bitmap.disable_6g;

	wl->efuse_mac_address[5] = (u8) (mac_address);
	wl->efuse_mac_address[4] = (u8) (mac_address >> 8);
	wl->efuse_mac_address[3] = (u8) (mac_address >> 16);
	wl->efuse_mac_address[2] = (u8) (mac_address >> 24);
	wl->efuse_mac_address[1] = (u8) (mac_address >> 32);
	wl->efuse_mac_address[0] = (u8) (mac_address >> 40);

	return 0;
}

int cc33xx_init_fw(struct cc33xx *wl)
{
	int ret;
	wl->max_cmd_size = CC33XX_CMD_MAX_SIZE;

	ret = fw_download_alloc(wl);
	if (ret < 0)
		return ret;

	reinit_completion(&wl->fw_download->wait_on_irq);

	ret = cc33xx_chip_wakeup(wl);
	if (ret < 0)
		goto power_off;

	wlcore_enable_interrupts(wl);

	ret = wait_for_boot_irq(wl,
		HINT_ROM_LOADER_INIT_COMPLETE,
		CC33XX_BOOT_TIMEOUT);
	if (ret < 0)
		goto disable_irq;

	ret = get_device_info(wl);
	if (ret < 0)
		goto disable_irq;

	ret = container_download_and_wait(wl,
		SECOND_LOADER_NAME,
		HINT_SECOND_LOADER_INIT_COMPLETE);
	if (ret < 0)
		goto disable_irq;

	ret = container_download_and_wait(wl,
		FW_NAME,
		HINT_FW_WAKEUP_COMPLETE);
	if (ret < 0)
		goto disable_irq;

	ret = cc33xx_download_ini_params_and_wait(wl);

	if (ret < 0)
		goto disable_irq;

	ret = wait_for_boot_irq(wl, HINT_FW_INIT_COMPLETE , CC33XX_BOOT_TIMEOUT);


	if (ret < 0)
		goto disable_irq;

	/* Get static calibration data and send it to FW*/
	ret = download_static_calibration_data(wl);
	if (ret < 0)
		return ret;

	ret = cc33xx_hw_init(wl);
	if (ret < 0)
		goto disable_irq;

	/*
	 * Now we know if 11a is supported (info from the INI File), so disable
	 * 11a channels if not supported
	 */
	wl->enable_11a =wl->conf.core.enable_5ghz;

	cc33xx_debug(DEBUG_MAC80211, "11a is %ssupported",
		     wl->enable_11a ? "" : "not ");

	wl->state = WLCORE_STATE_ON;
	ret = 0;
	goto out;

disable_irq:
	wlcore_disable_interrupts_nosync(wl);

power_off:
	cc33xx_power_off(wl);

out:
	fw_download_free(wl);
	return ret;
}
