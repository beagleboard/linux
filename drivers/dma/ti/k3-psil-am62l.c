// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2024-2025 Texas Instruments Incorporated - https://www.ti.com
 */

#include <linux/kernel.h>

#include "k3-psil-priv.h"

#define PSIL_PDMA_XY_TR(x, ch)					\
	{							\
		.thread_id = x,					\
		.ep_config = {					\
			.ep_type = PSIL_EP_PDMA_XY,		\
			.mapped_channel_id = ch,		\
			.default_flow_id = -1,			\
		},						\
	}

#define PSIL_PDMA_XY_PKT(x, ch)					\
	{							\
		.thread_id = x,					\
		.ep_config = {					\
			.ep_type = PSIL_EP_PDMA_XY,		\
			.mapped_channel_id = ch,		\
			.pkt_mode = 1,				\
			.default_flow_id = -1			\
		},						\
	}

#define PSIL_ETHERNET(x, ch, flow_base, flow_cnt)		\
	{							\
		.thread_id = x,					\
		.ep_config = {					\
			.ep_type = PSIL_EP_NATIVE,		\
			.pkt_mode = 1,				\
			.needs_epib = 1,			\
			.psd_size = 16,				\
			.mapped_channel_id = ch,		\
			.flow_start = flow_base,		\
			.flow_num = flow_cnt,			\
			.default_flow_id = flow_base,		\
		},						\
	}

#define PSIL_PDMA_MCASP(x, ch)				\
	{						\
		.thread_id = x,				\
		.ep_config = {				\
			.ep_type = PSIL_EP_PDMA_XY,	\
			.pdma_acc32 = 1,		\
			.pdma_burst = 1,		\
			.mapped_channel_id = ch,	\
		},					\
	}

/* PSI-L source thread IDs, used for RX (DMA_DEV_TO_MEM) */
static struct psil_ep am62l_src_ep_map[] = {
	/* PDMA_MAIN1 - UART0-6 */
	PSIL_PDMA_XY_PKT(0x4400, 0),
	PSIL_PDMA_XY_PKT(0x4401, 2),
	PSIL_PDMA_XY_PKT(0x4402, 4),
	PSIL_PDMA_XY_PKT(0x4403, 6),
	PSIL_PDMA_XY_PKT(0x4404, 8),
	PSIL_PDMA_XY_PKT(0x4405, 10),
	PSIL_PDMA_XY_PKT(0x4406, 12),
	/* PDMA_MAIN0 - SPI0 - CH0-3 */
	PSIL_PDMA_XY_TR(0x4300, 16),
	/* PDMA_MAIN0 - SPI1 - CH0-3 */
	PSIL_PDMA_XY_TR(0x4301, 24),
	/* PDMA_MAIN0 - SPI2 - CH0-3 */
	PSIL_PDMA_XY_TR(0x4302, 32),
	/* PDMA_MAIN0 - SPI3 - CH0-3 */
	PSIL_PDMA_XY_TR(0x4303, 40),
	/* PDMA_MAIN2 - MCASP0-2 */
	PSIL_PDMA_MCASP(0x4500, 48),
	PSIL_PDMA_MCASP(0x4501, 50),
	PSIL_PDMA_MCASP(0x4502, 52),
	/* PDMA_MAIN0 - AES */
	PSIL_PDMA_XY_TR(0x4700, 65),
	/* PDMA_MAIN0 - ADC */
	PSIL_PDMA_XY_TR(0x4503, 80),
	PSIL_PDMA_XY_TR(0x4504, 81),
	PSIL_ETHERNET(0x4600, 96, 96, 16),
};

/* PSI-L destination thread IDs, used for TX (DMA_MEM_TO_DEV) */
static struct psil_ep am62l_dst_ep_map[] = {
	/* PDMA_MAIN1 - UART0-6 */
	PSIL_PDMA_XY_PKT(0xC400, 1),
	PSIL_PDMA_XY_PKT(0xC401, 3),
	PSIL_PDMA_XY_PKT(0xC402, 5),
	PSIL_PDMA_XY_PKT(0xC403, 7),
	PSIL_PDMA_XY_PKT(0xC404, 9),
	PSIL_PDMA_XY_PKT(0xC405, 11),
	PSIL_PDMA_XY_PKT(0xC406, 13),
	/* PDMA_MAIN0 - SPI0 - CH0-3 */
	PSIL_PDMA_XY_TR(0xC300, 17),
	/* PDMA_MAIN0 - SPI1 - CH0-3 */
	PSIL_PDMA_XY_TR(0xC301, 25),
	/* PDMA_MAIN0 - SPI2 - CH0-3 */
	PSIL_PDMA_XY_TR(0xC302, 33),
	/* PDMA_MAIN0 - SPI3 - CH0-3 */
	PSIL_PDMA_XY_TR(0xC303, 41),
	/* PDMA_MAIN2 - MCASP0-2 */
	PSIL_PDMA_MCASP(0xC500, 49),
	PSIL_PDMA_MCASP(0xC501, 51),
	PSIL_PDMA_MCASP(0xC502, 53),
	/* PDMA_MAIN0 - SHA */
	PSIL_PDMA_XY_TR(0xC700, 64),
	/* PDMA_MAIN0 - AES */
	PSIL_PDMA_XY_TR(0xC701, 66),
	/* PDMA_MAIN0 - CRC32 - CH0-1 */
	PSIL_PDMA_XY_TR(0xC702, 67),
	/* CPSW3G */
	PSIL_ETHERNET(0xc600, 64, 64, 2),
	PSIL_ETHERNET(0xc601, 66, 66, 2),
	PSIL_ETHERNET(0xc602, 68, 68, 2),
	PSIL_ETHERNET(0xc603, 70, 70, 2),
	PSIL_ETHERNET(0xc604, 72, 72, 2),
	PSIL_ETHERNET(0xc605, 74, 74, 2),
	PSIL_ETHERNET(0xc606, 76, 76, 2),
	PSIL_ETHERNET(0xc607, 78, 78, 2),
};

struct psil_ep_map am62l_ep_map = {
	.name = "am62l",
	.src = am62l_src_ep_map,
	.src_count = ARRAY_SIZE(am62l_src_ep_map),
	.dst = am62l_dst_ep_map,
	.dst_count = ARRAY_SIZE(am62l_dst_ep_map),
};
