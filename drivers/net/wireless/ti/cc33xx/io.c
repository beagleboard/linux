// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#include "cc33xx.h"
#include "debug.h"
#include "io.h"
#include "tx.h"

bool cc33xx_set_block_size(struct cc33xx *cc)
{
	if (cc->if_ops->set_block_size) {
		cc->if_ops->set_block_size(cc->dev, CC33XX_BUS_BLOCK_SIZE);
		cc33xx_debug(DEBUG_CC33xx,
			     "Set BLKsize to %d", CC33XX_BUS_BLOCK_SIZE);
		return true;
	}

	cc33xx_debug(DEBUG_CC33xx, "Could not set BLKsize");
	return false;
}

void cc33xx_disable_interrupts_nosync(struct cc33xx *cc)
{
	cc->if_ops->disable_irq(cc->dev);
}

void cc33xx_irq(void *cookie);
void cc33xx_enable_interrupts(struct cc33xx *cc)
{
	cc->if_ops->enable_irq(cc->dev);

	cc33xx_debug(DEBUG_CC33xx, "IBI_WA: Read core status");
	cc33xx_irq(cc);
	cc33xx_debug(DEBUG_CC33xx, "IBI_WA: Core status processed");
}

void cc33xx_io_reset(struct cc33xx *cc)
{
	if (cc->if_ops->reset)
		cc->if_ops->reset(cc->dev);
}

void cc33xx_io_init(struct cc33xx *cc)
{
	if (cc->if_ops->init)
		cc->if_ops->init(cc->dev);
}

/* Raw target IO, address is not translated */
static int __must_check cc33xx_raw_write(struct cc33xx *cc, int addr,
					 void *buf, size_t len, bool fixed)
{
	int ret;

	if (test_bit(CC33XX_FLAG_IO_FAILED, &cc->flags) ||
	    WARN_ON((test_bit(CC33XX_FLAG_IN_ELP, &cc->flags) &&
		     addr != HW_ACCESS_ELP_CTRL_REG)))
		return -EIO;

	ret = cc->if_ops->write(cc->dev, addr, buf, len, fixed);
	if (ret && cc->state != CC33XX_STATE_OFF)
		set_bit(CC33XX_FLAG_IO_FAILED, &cc->flags);

	return ret;
}

int __must_check cc33xx_raw_read(struct cc33xx *cc, int addr,
				 void *buf, size_t len, bool fixed)
{
	int ret;

	if (test_bit(CC33XX_FLAG_IO_FAILED, &cc->flags) ||
	    WARN_ON((test_bit(CC33XX_FLAG_IN_ELP, &cc->flags) &&
		     addr != HW_ACCESS_ELP_CTRL_REG)))
		return -EIO;

	ret = cc->if_ops->read(cc->dev, addr, buf, len, fixed);
	if (ret && cc->state != CC33XX_STATE_OFF)
		set_bit(CC33XX_FLAG_IO_FAILED, &cc->flags);

	return ret;
}

int __must_check cc33xx_write(struct cc33xx *cc, int addr,
			      void *buf, size_t len, bool fixed)
{
	return cc33xx_raw_write(cc, addr, buf, len, fixed);
}

void claim_core_status_lock(struct cc33xx *cc)
{
	/* When accessing core-status data (read or write) the transport lock
	 * should be held.
	 */
	cc->if_ops->interface_claim(cc->dev);
}

void release_core_status_lock(struct cc33xx *cc)
{
	/* After accessing core-status data (read or write) the transport lock
	 * should be released.
	 */
	cc->if_ops->interface_release(cc->dev);
}

void cc33xx_power_off(struct cc33xx *cc)
{
	int ret = 0;

	if (!test_bit(CC33XX_FLAG_GPIO_POWER, &cc->flags))
		return;

	if (cc->if_ops->power)
		ret = cc->if_ops->power(cc->dev, false);
	if (!ret)
		clear_bit(CC33XX_FLAG_GPIO_POWER, &cc->flags);
}

int cc33xx_power_on(struct cc33xx *cc)
{
	int ret = 0;

	if (cc->if_ops->power)
		ret = cc->if_ops->power(cc->dev, true);
	if (ret == 0)
		set_bit(CC33XX_FLAG_GPIO_POWER, &cc->flags);

	return ret;
}
