// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2012 Intel, Inc.
 * Copyright (C) 2013 Intel, Inc.
 * Copyright (C) 2014 Linaro Limited
 * Copyright (C) 2011-2016 Google, Inc.
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

/* This source file contains the implementation of a special device driver
 * that intends to provide a *very* fast communication channel between the
 * guest system and the QEMU emulator.
 *
 * Usage from the guest is simply the following (error handling simplified):
 *
 *    int  fd = open("/dev/qemu_pipe",O_RDWR);
 *    .... write() or read() through the pipe.
 *
 * This driver doesn't deal with the exact protocol used during the session.
 * It is intended to be as simple as something like:
 *
 *    // do this _just_ after opening the fd to connect to a specific
 *    // emulator service.
 *    const char*  msg = "<pipename>";
 *    if (write(fd, msg, strlen(msg)+1) < 0) {
 *       ... could not connect to <pipename> service
 *       close(fd);
 *    }
 *
 *    // after this, simply read() and write() to communicate with the
 *    // service. Exact protocol details left as an exercise to the reader.
 *
 * This driver is very fast because it doesn't copy any data through
 * intermediate buffers, since the emulator is capable of translating
 * guest user addresses into host ones.
 *
 * Note that we must however ensure that each user page involved in the
 * exchange is properly mapped during a transfer.
 */

//#include "defconfig_test.h"

#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/acpi.h>
#include "goldfish_pipe_qemu.h"
#include "goldfish_pipe.h"

/*
 * Update this when something changes in the driver's behavior so the host
 * can benefit from knowing it
 * Notes:
 *	version 2 was an intermediate release and isn't supported anymore.
 *	version 3 is goldfish_pipe_v2 without DMA support.
 *	version 4 (current) is goldfish_pipe_v2 with DMA support.
 */
enum {
	PIPE_DRIVER_VERSION = 4,
	PIPE_CURRENT_DEVICE_VERSION = 2
};

static int goldfish_pipe_probe(struct platform_device *pdev)
{
	struct resource *r;
	char __iomem *base;
	int irq;
	int version;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r || resource_size(r) < PAGE_SIZE) {
		dev_err(&pdev->dev, "can't allocate i/o page\n");
		return -EINVAL;
	}
	base = devm_ioremap(&pdev->dev, r->start, PAGE_SIZE);
	if (!base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return -EINVAL;
	}

	r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!r)
		return -EINVAL;

	irq = r->start;

	/*
	 * Exchange the versions with the host device
	 *
	 * Note: v1 driver used to not report its version, so we write it before
	 *  reading device version back: this allows the host implementation to
	 *  detect the old driver (if there was no version write before read).
	 */
	writel(PIPE_DRIVER_VERSION, base + PIPE_V2_REG_VERSION);
	version = readl(base + PIPE_V2_REG_VERSION);

	if (version < PIPE_CURRENT_DEVICE_VERSION)
		return goldfish_pipe_device_v1_init(pdev, base, irq);
	else
		return goldfish_pipe_device_v2_init(pdev, base, irq);
}

static int goldfish_pipe_remove(struct platform_device *pdev)
{
	struct goldfish_pipe_dev_base *dev = platform_get_drvdata(pdev);

	return dev->deinit(dev, pdev);
}

static const struct acpi_device_id goldfish_pipe_acpi_match[] = {
	{ "GFSH0003", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, goldfish_pipe_acpi_match);

static const struct of_device_id goldfish_pipe_of_match[] = {
	{ .compatible = "google,android-pipe", },
	{},
};
MODULE_DEVICE_TABLE(of, goldfish_pipe_of_match);

static struct platform_driver goldfish_pipe_driver = {
	.probe = goldfish_pipe_probe,
	.remove = goldfish_pipe_remove,
	.driver = {
		.name = "goldfish_pipe",
		.of_match_table = goldfish_pipe_of_match,
		.acpi_match_table = ACPI_PTR(goldfish_pipe_acpi_match),
	}
};

module_platform_driver(goldfish_pipe_driver);
MODULE_AUTHOR("David Turner <digit@google.com>");
MODULE_LICENSE("GPL v2");
