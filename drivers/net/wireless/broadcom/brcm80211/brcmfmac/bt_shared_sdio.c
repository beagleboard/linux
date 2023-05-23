// SPDX-License-Identifier: ISC

/* Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor
 * Corporation or one of its subsidiaries ("Cypress") and is protected by
 * and subject to worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA"). If no EULA applies, Cypress hereby grants
 * you a personal, nonexclusive, non-transferable license to copy, modify,
 * and compile the Software source code solely for use in connection with
 * Cypress's integrated circuit products. Any reproduction, modification,
 * translation, compilation, or representation of this Software except as
 * specified above is prohibited without the express written permission of
 * Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/kernel.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/card.h>
#include "bus.h"
#include "chipcommon.h"
#include "core.h"
#include "sdio.h"
#include "soc.h"
#include "fwil.h"

#define SDIOD_ADDR_BOUND		0x1000
#define SDIOD_ADDR_BOUND_MASK		0xfff

struct brcmf_bus *g_bus_if;

enum bus_owner {
	WLAN_MODULE = 0,
	BT_MODULE
};

struct btsdio_info {
	u32 bt_buf_reg_addr;
	u32 host_ctrl_reg_addr;
	u32 bt_ctrl_reg_addr;
	u32 bt_buf_addr;
	u32 wlan_buf_addr;
};

void brcmf_btsdio_int_handler(struct brcmf_bus *bus_if)
{
	struct brcmf_bt_dev *btdev = bus_if->bt_dev;

	if (btdev && btdev->bt_sdio_int_cb)
		btdev->bt_sdio_int_cb(btdev->bt_data);
}

int brcmf_btsdio_init(struct brcmf_bus *bus_if)
{
	if (!bus_if)
		return -EINVAL;

	g_bus_if = bus_if;
	return 0;
}

int brcmf_btsdio_attach(struct brcmf_bus *bus_if, void *btdata,
			void (*bt_int_fun)(void *data))
{
	struct brcmf_bt_dev *btdev;

	/* Allocate bt dev */
	btdev = kzalloc(sizeof(*btdev), GFP_ATOMIC);
	if (!btdev)
		return -ENOMEM;

	btdev->bt_data = btdata;
	btdev->bt_sdio_int_cb = bt_int_fun;
	bus_if->bt_dev = btdev;

	return 0;
}

void brcmf_btsdio_detach(struct brcmf_bus *bus_if)
{
	struct brcmf_bt_dev *btdev = bus_if->bt_dev;

	if (!btdev)
		return;

	if (btdev->bt_data)
		btdev->bt_data = NULL;
	if (btdev->bt_sdio_int_cb)
		btdev->bt_sdio_int_cb = NULL;
	if (bus_if->bt_dev) {
		bus_if->bt_dev = NULL;
		kfree(btdev);
	}
}

u8 brcmf_btsdio_bus_count(struct brcmf_bus *bus_if)
{
	struct brcmf_bt_dev *btdev = bus_if->bt_dev;

	if (!btdev)
		return 0;

	return btdev->use_count;
}

void *brcmf_bt_sdio_attach(void *btdata, void (*bt_int_fun)(void *data))
{
	int err;

	if (!g_bus_if) {
		brcmf_err("BTSDIO is not initialized\n");
		return NULL;
	}

	err = brcmf_btsdio_attach(g_bus_if, btdata, bt_int_fun);
	if (err) {
		brcmf_err("BTSDIO attach failed, err=%d\n", err);
		return NULL;
	}

	return (void *)g_bus_if;
}
EXPORT_SYMBOL(brcmf_bt_sdio_attach);

int brcmf_get_wlan_info(struct brcmf_bus *bus_if, struct btsdio_info *bs_info)
{
	struct brcmf_if *ifp;

	if (!bus_if || !bs_info)
		return -EINVAL;

	ifp = bus_if->drvr->iflist[0];

	bs_info->bt_buf_reg_addr = SI_ENUM_BASE + 0xC00 +
				     CHIPGCIREGOFFS(gci_input[6]);
	bs_info->host_ctrl_reg_addr = SI_ENUM_BASE + 0xC00 +
				      CHIPGCIREGOFFS(gci_output[3]);
	bs_info->bt_ctrl_reg_addr = SI_ENUM_BASE + 0xC00 +
				    CHIPGCIREGOFFS(gci_input[7]);
	brcmf_dbg(INFO, "BT buf reg addr: 0x%x\n",
		  bs_info->bt_buf_reg_addr);
	brcmf_dbg(INFO, "HOST ctrl reg addr: 0x%x\n",
		  bs_info->host_ctrl_reg_addr);
	brcmf_dbg(INFO, "BT ctrl reg addr: 0x%x\n",
		  bs_info->bt_ctrl_reg_addr);
	return 0;
}
EXPORT_SYMBOL(brcmf_get_wlan_info);

u32 brcmf_bus_reg_read(struct brcmf_bus *bus_if, u32 addr)
{
	struct brcmf_sdio_dev *sdiodev;
	int err = 0;
	u32 val;

	if (!bus_if)
		return -EINVAL;

	sdiodev = bus_if->bus_priv.sdio;

	sdio_claim_host(sdiodev->func1);
	val = brcmf_sdiod_readl(sdiodev, addr, &err);
	if (err) {
		brcmf_err("sdio reg read failed, err=%d\n", err);
		sdio_release_host(sdiodev->func1);
		return err;
	}
	sdio_release_host(sdiodev->func1);

	return val;
}
EXPORT_SYMBOL(brcmf_bus_reg_read);

void brcmf_bus_reg_write(struct brcmf_bus *bus_if, u32 addr, u32 val)
{
	struct brcmf_sdio_dev *sdiodev;
	int err = 0;

	if (!bus_if)
		return;

	sdiodev = bus_if->bus_priv.sdio;

	sdio_claim_host(sdiodev->func1);
	brcmf_sdiod_writel(sdiodev, addr, val, &err);
	if (err)
		brcmf_err("sdio reg write failed, err=%d\n", err);
	sdio_release_host(sdiodev->func1);
}
EXPORT_SYMBOL(brcmf_bus_reg_write);

int brcmf_membytes(struct brcmf_bus *bus_if, bool set, u32 address, u8 *data,
		   unsigned int size)
{
	struct brcmf_sdio_dev *sdiodev;
	int err = 0;
	u32 block1_offset;
	u32 block2_addr;
	u16 block1_size;
	u16 block2_size;
	u8 *block2_data;

	if (!bus_if || !data)
		return -EINVAL;

	sdiodev = bus_if->bus_priv.sdio;
	/* To avoid SDIO access crosses AXI 4k address boundaries crossing */
	if (((address & SDIOD_ADDR_BOUND_MASK) + size) > SDIOD_ADDR_BOUND) {
		brcmf_dbg(SDIO, "data cross 4K boundary\n");
		/* The 1st 4k packet */
		block1_offset = address & SDIOD_ADDR_BOUND_MASK;
		block1_size = (SDIOD_ADDR_BOUND - block1_offset);
		sdio_claim_host(sdiodev->func1);
		err = brcmf_sdiod_ramrw(sdiodev, set, address,
					data, block1_size);
		if (err) {
			brcmf_err("sdio memory access failed, err=%d\n", err);
			sdio_release_host(sdiodev->func1);
			return err;
		}
		/* The 2nd 4k packet */
		block2_addr = address + block1_size;
		block2_size = size - block1_size;
		block2_data = data + block1_size;
		err = brcmf_sdiod_ramrw(sdiodev, set, block2_addr,
					block2_data, block2_size);
		if (err)
			brcmf_err("sdio memory access failed, err=%d\n", err);
		sdio_release_host(sdiodev->func1);
	} else {
		sdio_claim_host(sdiodev->func1);
		err = brcmf_sdiod_ramrw(sdiodev, set, address, data, size);
		if (err)
			brcmf_err("sdio memory access failed, err=%d\n", err);
		sdio_release_host(sdiodev->func1);
	}
	return err;
}
EXPORT_SYMBOL(brcmf_membytes);

/* Function to enable the Bus Clock
 * This function is not callable from non-sleepable context
 */
int brcmf_bus_clk_enable(struct brcmf_bus *bus_if, enum bus_owner owner)
{
	struct brcmf_sdio_dev *sdiodev;
	struct brcmf_bt_dev *btdev;
	int err = 0;

	if (!bus_if)
		return -EINVAL;

	btdev = bus_if->bt_dev;
	sdiodev = bus_if->bus_priv.sdio;

	sdio_claim_host(sdiodev->func1);
	btdev->use_count++;
	sdio_release_host(sdiodev->func1);
	err = brcmf_sdio_sleep(sdiodev->bus, false);

	return err;
}
EXPORT_SYMBOL(brcmf_bus_clk_enable);

/* Function to disable the Bus Clock
 * This function is not callable from non-sleepable context
 */
int brcmf_bus_clk_disable(struct brcmf_bus *bus_if, enum bus_owner owner)
{
	struct brcmf_sdio_dev *sdiodev;
	struct brcmf_bt_dev *btdev;
	int err = 0;

	if (!bus_if)
		return -EINVAL;

	btdev = bus_if->bt_dev;
	sdiodev = bus_if->bus_priv.sdio;

	sdio_claim_host(sdiodev->func1);
	if (btdev->use_count != 0)
		btdev->use_count--;
	sdio_release_host(sdiodev->func1);
	err = brcmf_sdio_sleep(sdiodev->bus, true);

	return err;
}
EXPORT_SYMBOL(brcmf_bus_clk_disable);

/* Function to reset bt_use_count counter to zero.
 * This function is not callable from non-sleepable context
 */
void brcmf_bus_reset_bt_use_count(struct brcmf_bus *bus_if)
{
	struct brcmf_sdio_dev *sdiodev;
	struct brcmf_bt_dev *btdev;

	if (!bus_if)
		return;

	btdev = bus_if->bt_dev;
	sdiodev = bus_if->bus_priv.sdio;

	sdio_claim_host(sdiodev->func1);
	btdev->use_count = 0;
	sdio_release_host(sdiodev->func1);
}
EXPORT_SYMBOL(brcmf_bus_reset_bt_use_count);
