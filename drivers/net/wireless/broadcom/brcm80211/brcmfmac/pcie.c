// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2014 Broadcom Corporation
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/pci.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/bcma/bcma.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/kthread.h>
#include <asm/unaligned.h>

#include <soc.h>
#include <chipcommon.h>
#include <brcmu_utils.h>
#include <brcmu_wifi.h>
#include <brcm_hw_ids.h>

/* Custom brcmf_err() that takes bus arg and passes it further */
#define brcmf_err(bus, fmt, ...)					\
	do {								\
		if (IS_ENABLED(CONFIG_BRCMDBG) ||			\
		    IS_ENABLED(CONFIG_BRCM_TRACING) ||			\
		    net_ratelimit())					\
			__brcmf_err(bus, __func__, fmt, ##__VA_ARGS__);	\
	} while (0)

#include "debug.h"
#include "bus.h"
#include "commonring.h"
#include "msgbuf.h"
#include "pcie.h"
#include "firmware.h"
#include "chip.h"
#include "core.h"
#include "common.h"
#include "cfg80211.h"
#include "trxhdr.h"


enum brcmf_pcie_state {
	BRCMFMAC_PCIE_STATE_DOWN,
	BRCMFMAC_PCIE_STATE_UP
};

BRCMF_FW_DEF(43602, "brcmfmac43602-pcie");
BRCMF_FW_DEF(4350, "brcmfmac4350-pcie");
BRCMF_FW_DEF(4350C, "brcmfmac4350c2-pcie");
CY_FW_DEF(4356, "cyfmac4356-pcie");
CY_FW_DEF(43570, "cyfmac43570-pcie");
BRCMF_FW_DEF(4358, "brcmfmac4358-pcie");
CY_FW_DEF(4359, "cyfmac4359-pcie");
BRCMF_FW_DEF(4364, "brcmfmac4364-pcie");
BRCMF_FW_DEF(4365B, "brcmfmac4365b-pcie");
BRCMF_FW_DEF(4365C, "brcmfmac4365c-pcie");
BRCMF_FW_DEF(4366B, "brcmfmac4366b-pcie");
BRCMF_FW_DEF(4366C, "brcmfmac4366c-pcie");
BRCMF_FW_DEF(4371, "brcmfmac4371-pcie");
CY_FW_DEF(4355, "cyfmac54591-pcie");
CY_FW_TRXSE_DEF(55572, "cyfmac55572-pcie");
CY_FW_DEF(4373, "cyfmac4373-pcie");

static const struct brcmf_firmware_mapping brcmf_pcie_fwnames[] = {
	BRCMF_FW_ENTRY(BRCM_CC_43602_CHIP_ID, 0xFFFFFFFF, 43602),
	BRCMF_FW_ENTRY(BRCM_CC_43465_CHIP_ID, 0xFFFFFFF0, 4366C),
	BRCMF_FW_ENTRY(BRCM_CC_4350_CHIP_ID, 0x000000FF, 4350C),
	BRCMF_FW_ENTRY(BRCM_CC_4350_CHIP_ID, 0xFFFFFF00, 4350),
	BRCMF_FW_ENTRY(BRCM_CC_43525_CHIP_ID, 0xFFFFFFF0, 4365C),
	BRCMF_FW_ENTRY(BRCM_CC_4356_CHIP_ID, 0xFFFFFFFF, 4356),
	BRCMF_FW_ENTRY(BRCM_CC_43567_CHIP_ID, 0xFFFFFFFF, 43570),
	BRCMF_FW_ENTRY(BRCM_CC_43569_CHIP_ID, 0xFFFFFFFF, 43570),
	BRCMF_FW_ENTRY(BRCM_CC_43570_CHIP_ID, 0xFFFFFFFF, 43570),
	BRCMF_FW_ENTRY(BRCM_CC_4358_CHIP_ID, 0xFFFFFFFF, 4358),
	BRCMF_FW_ENTRY(BRCM_CC_4359_CHIP_ID, 0xFFFFFFFF, 4359),
	BRCMF_FW_ENTRY(BRCM_CC_4364_CHIP_ID, 0xFFFFFFFF, 4364),
	BRCMF_FW_ENTRY(BRCM_CC_4365_CHIP_ID, 0x0000000F, 4365B),
	BRCMF_FW_ENTRY(BRCM_CC_4365_CHIP_ID, 0xFFFFFFF0, 4365C),
	BRCMF_FW_ENTRY(BRCM_CC_4366_CHIP_ID, 0x0000000F, 4366B),
	BRCMF_FW_ENTRY(BRCM_CC_4366_CHIP_ID, 0xFFFFFFF0, 4366C),
	BRCMF_FW_ENTRY(BRCM_CC_43664_CHIP_ID, 0xFFFFFFF0, 4366C),
	BRCMF_FW_ENTRY(BRCM_CC_4371_CHIP_ID, 0xFFFFFFFF, 4371),
	BRCMF_FW_ENTRY(CY_CC_89459_CHIP_ID, 0xFFFFFFFF, 4355),
	BRCMF_FW_ENTRY(CY_CC_55572_CHIP_ID, 0xFFFFFFFF, 55572),
	BRCMF_FW_ENTRY(CY_CC_4373_CHIP_ID, 0xFFFFFFFF, 4373),
};

#define BRCMF_PCIE_REV_GE64(dev)	(brcmf_chip_get_core((dev)->ci, \
					 BCMA_CORE_PCIE2)->rev >= 64)

#define BRCMF_PCIE_READ_SHARED_TIMEOUT		5000 /* msec */
#define BRCMF_PCIE_FW_UP_TIMEOUT		5000 /* msec */

#define BRCMF_PCIE_REG_MAP_SIZE			(32 * 1024)

/* backplane addres space accessed by BAR0 */
#define	BRCMF_PCIE_BAR0_WINDOW			0x80
#define BRCMF_PCIE_BAR0_REG_SIZE		0x1000
#define	BRCMF_PCIE_BAR0_WRAPPERBASE		0x70

#define BRCMF_PCIE_BAR0_WRAPBASE_DMP_OFFSET	0x1000
#define BRCMF_PCIE_BAR0_PCIE_ENUM_OFFSET	0x2000
#define BRCMF_CYW55572_PCIE_BAR0_PCIE_ENUM_OFFSET	0x3000

#define BRCMF_PCIE_ARMCR4REG_BANKIDX		0x40
#define BRCMF_PCIE_ARMCR4REG_BANKPDA		0x4C

#define BRCMF_PCIE_REG_INTSTATUS		0x90
#define BRCMF_PCIE_REG_INTMASK			0x94
#define BRCMF_PCIE_REG_SBMBX			0x98

#define BRCMF_PCIE_REG_LINK_STATUS_CTRL		0xBC

#define BRCMF_PCIE_PCIE2REG_INTMASK		0x24
#define BRCMF_PCIE_PCIE2REG_MAILBOXINT(dev)	(BRCMF_PCIE_REV_GE64(dev) ? \
						 0xC30 : 0x48)
#define BRCMF_PCIE_PCIE2REG_MAILBOXMASK(dev)	(BRCMF_PCIE_REV_GE64(dev) ? \
						 0xC34 : 0x4C)
#define BRCMF_PCIE_PCIE2REG_CONFIGADDR		0x120
#define BRCMF_PCIE_PCIE2REG_CONFIGDATA		0x124
#define BRCMF_PCIE_PCIE2REG_H2D_MAILBOX_0	0x140
#define BRCMF_PCIE_PCIE2REG_H2D_MAILBOX_1	0x144
#define BRCMF_PCIE_PCIE2REG_DAR_D2H_MSG_0	0xA80
#define BRCMF_PCIE_PCIE2REG_DAR_H2D_MSG_0	0xA90

#define BRCMF_PCIE2_INTA			0x01
#define BRCMF_PCIE2_INTB			0x02

#define BRCMF_PCIE_INT_0			0x01
#define BRCMF_PCIE_INT_1			0x02
#define BRCMF_PCIE_INT_DEF			(BRCMF_PCIE_INT_0 | \
						 BRCMF_PCIE_INT_1)

#define BRCMF_PCIE_MB_INT_FN0_0			0x0100
#define BRCMF_PCIE_MB_INT_FN0_1			0x0200
#define BRCMF_PCIE_MB_INT_D2H0_DB0		0x10000
#define BRCMF_PCIE_MB_INT_D2H0_DB1		0x20000
#define BRCMF_PCIE_MB_INT_D2H1_DB0		0x40000
#define BRCMF_PCIE_MB_INT_D2H1_DB1		0x80000
#define BRCMF_PCIE_MB_INT_D2H2_DB0		0x100000
#define BRCMF_PCIE_MB_INT_D2H2_DB1		0x200000
#define BRCMF_PCIE_MB_INT_D2H3_DB0		0x400000
#define BRCMF_PCIE_MB_INT_D2H3_DB1		0x800000
#define BRCMF_PCIE_MB_INT_D2H_DB_ALL	(BRCMF_PCIE_MB_INT_D2H0_DB0 | \
					 BRCMF_PCIE_MB_INT_D2H0_DB1 | \
					 BRCMF_PCIE_MB_INT_D2H1_DB0 | \
					 BRCMF_PCIE_MB_INT_D2H1_DB1 | \
					 BRCMF_PCIE_MB_INT_D2H2_DB0 | \
					 BRCMF_PCIE_MB_INT_D2H2_DB1 | \
					 BRCMF_PCIE_MB_INT_D2H3_DB0 | \
					 BRCMF_PCIE_MB_INT_D2H3_DB1)

#define BRCMF_PCIE_MB_INT_D2H0_DB0_GE64		0x0001
#define BRCMF_PCIE_MB_INT_D2H0_DB1_GE64		0x0002
#define BRCMF_PCIE_MB_INT_D2H1_DB0_GE64		0x0004
#define BRCMF_PCIE_MB_INT_D2H1_DB1_GE64		0x0008
#define BRCMF_PCIE_MB_INT_D2H2_DB0_GE64		0x0010
#define BRCMF_PCIE_MB_INT_D2H2_DB1_GE64		0x0020
#define BRCMF_PCIE_MB_INT_D2H3_DB0_GE64		0x0040
#define BRCMF_PCIE_MB_INT_D2H3_DB1_GE64		0x0080
#define BRCMF_PCIE_MB_INT_D2H4_DB0_GE64		0x0100
#define BRCMF_PCIE_MB_INT_D2H4_DB1_GE64		0x0200
#define BRCMF_PCIE_MB_INT_D2H5_DB0_GE64		0x0400
#define BRCMF_PCIE_MB_INT_D2H5_DB1_GE64		0x0800
#define BRCMF_PCIE_MB_INT_D2H6_DB0_GE64		0x1000
#define BRCMF_PCIE_MB_INT_D2H6_DB1_GE64		0x2000
#define BRCMF_PCIE_MB_INT_D2H7_DB0_GE64		0x4000
#define BRCMF_PCIE_MB_INT_D2H7_DB1_GE64		0x8000
#define BRCMF_PCIE_MB_INT_D2H_DB_ALL_GE64 \
					(BRCMF_PCIE_MB_INT_D2H0_DB0_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H0_DB1_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H1_DB0_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H1_DB1_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H2_DB0_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H2_DB1_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H3_DB0_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H3_DB1_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H4_DB0_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H4_DB1_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H5_DB0_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H5_DB1_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H6_DB0_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H6_DB1_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H7_DB0_GE64 | \
					 BRCMF_PCIE_MB_INT_D2H7_DB1_GE64)

#define BRCMF_PCIE_MB_INT_D2H_DB(dev)	(BRCMF_PCIE_REV_GE64(dev) ? \
					 BRCMF_PCIE_MB_INT_D2H_DB_ALL_GE64 : \
					 BRCMF_PCIE_MB_INT_D2H_DB_ALL)

#define BRCMF_PCIE_SHARED_VERSION_6		6
#define BRCMF_PCIE_SHARED_VERSION_7		7
#define BRCMF_PCIE_MIN_SHARED_VERSION		5
#define BRCMF_PCIE_MAX_SHARED_VERSION		BRCMF_PCIE_SHARED_VERSION_7
#define BRCMF_PCIE_SHARED_VERSION_MASK		0x00FF
#define BRCMF_PCIE_SHARED_DMA_INDEX		0x10000
#define BRCMF_PCIE_SHARED_DMA_2B_IDX		0x100000
#define BRCMF_PCIE_SHARED_USE_MAILBOX		0x2000000
#define BRCMF_PCIE_SHARED_HOSTRDY_DB1		0x10000000

#define BRCMF_PCIE_FLAGS_HTOD_SPLIT		0x4000
#define BRCMF_PCIE_FLAGS_DTOH_SPLIT		0x8000

#define BRCMF_SHARED_MAX_RXBUFPOST_OFFSET	34
#define BRCMF_SHARED_RING_BASE_OFFSET		52
#define BRCMF_SHARED_RX_DATAOFFSET_OFFSET	36
#define BRCMF_SHARED_CONSOLE_ADDR_OFFSET	20
#define BRCMF_SHARED_HTOD_MB_DATA_ADDR_OFFSET	40
#define BRCMF_SHARED_DTOH_MB_DATA_ADDR_OFFSET	44
#define BRCMF_SHARED_RING_INFO_ADDR_OFFSET	48
#define BRCMF_SHARED_DMA_SCRATCH_LEN_OFFSET	52
#define BRCMF_SHARED_DMA_SCRATCH_ADDR_OFFSET	56
#define BRCMF_SHARED_DMA_RINGUPD_LEN_OFFSET	64
#define BRCMF_SHARED_DMA_RINGUPD_ADDR_OFFSET	68
#define BRCMF_SHARED_HOST_CAP_OFFSET		84

#define BRCMF_RING_H2D_RING_COUNT_OFFSET	0
#define BRCMF_RING_D2H_RING_COUNT_OFFSET	1
#define BRCMF_RING_H2D_RING_MEM_OFFSET		4
#define BRCMF_RING_H2D_RING_STATE_OFFSET	8

#define BRCMF_RING_MEM_BASE_ADDR_OFFSET		8
#define BRCMF_RING_MAX_ITEM_OFFSET		4
#define BRCMF_RING_LEN_ITEMS_OFFSET		6
#define BRCMF_RING_MEM_SZ			16
#define BRCMF_RING_STATE_SZ			8

#define BRCMF_DEF_MAX_RXBUFPOST			255

#define BRCMF_HOSTCAP_H2D_ENABLE_HOSTRDY	0x400
#define BRCMF_HOSTCAP_DS_NO_OOB_DW			0x1000

#define BRCMF_CONSOLE_BUFADDR_OFFSET		8
#define BRCMF_CONSOLE_BUFSIZE_OFFSET		12
#define BRCMF_CONSOLE_WRITEIDX_OFFSET		16

#define BRCMF_DMA_D2H_SCRATCH_BUF_LEN		8
#define BRCMF_DMA_D2H_RINGUPD_BUF_LEN		1024

#define BRCMF_D2H_DEV_D3_ACK			0x00000001
#define BRCMF_D2H_DEV_DS_ENTER_REQ		0x00000002
#define BRCMF_D2H_DEV_DS_EXIT_NOTE		0x00000004
#define BRCMF_D2H_DEV_FWHALT			0x10000000

#define BRCMF_H2D_HOST_D3_INFORM		0x00000001
#define BRCMF_H2D_HOST_DS_ACK			0x00000002
#define BRCMF_H2D_HOST_D0_INFORM_IN_USE		0x00000008
#define BRCMF_H2D_HOST_D0_INFORM		0x00000010

#define BRCMF_PCIE_MBDATA_TIMEOUT		msecs_to_jiffies(2000)

#define BRCMF_PCIE_CFGREG_STATUS_CMD		0x4
#define BRCMF_PCIE_CFGREG_PM_CSR		0x4C
#define BRCMF_PCIE_CFGREG_MSI_CAP		0x58
#define BRCMF_PCIE_CFGREG_MSI_ADDR_L		0x5C
#define BRCMF_PCIE_CFGREG_MSI_ADDR_H		0x60
#define BRCMF_PCIE_CFGREG_MSI_DATA		0x64
#define BRCMF_PCIE_CFGREG_REVID			0x6C
#define BRCMF_PCIE_CFGREG_LINK_STATUS_CTRL	0xBC
#define BRCMF_PCIE_CFGREG_LINK_STATUS_CTRL2	0xDC
#define BRCMF_PCIE_CFGREG_RBAR_CTRL		0x228
#define BRCMF_PCIE_CFGREG_PML1_SUB_CTRL1	0x248
#define BRCMF_PCIE_CFGREG_REG_BAR2_CONFIG	0x4E0
#define BRCMF_PCIE_CFGREG_REG_BAR3_CONFIG	0x4F4
#define BRCMF_PCIE_CFGREG_REVID_SECURE_MODE	BIT(31)
#define BRCMF_PCIE_LINK_STATUS_CTRL_ASPM_ENAB	3

/* Magic number at a magic location to find RAM size */
#define BRCMF_RAMSIZE_MAGIC			0x534d4152	/* SMAR */
#define BRCMF_RAMSIZE_OFFSET			0x6c

#define BRCMF_ENTROPY_SEED_LEN		64u
#define BRCMF_ENTROPY_NONCE_LEN		16u
#define BRCMF_ENTROPY_HOST_LEN		(BRCMF_ENTROPY_SEED_LEN + \
					 BRCMF_ENTROPY_NONCE_LEN)
#define BRCMF_NVRAM_OFFSET_TCM		4u
#define BRCMF_NVRAM_COMPRS_FACTOR	4u
#define BRCMF_NVRAM_RNG_SIGNATURE	0xFEEDC0DEu

struct brcmf_rand_metadata {
	u32 signature;
	u32 count;
};

struct brcmf_pcie_console {
	u32 base_addr;
	u32 buf_addr;
	u32 bufsize;
	u32 read_idx;
	u8 log_str[256];
	u8 log_idx;
};

struct brcmf_pcie_shared_info {
	u32 tcm_base_address;
	u32 flags;
	struct brcmf_pcie_ringbuf *commonrings[BRCMF_NROF_COMMON_MSGRINGS];
	struct brcmf_pcie_ringbuf *flowrings;
	u16 max_rxbufpost;
	u16 max_flowrings;
	u16 max_submissionrings;
	u16 max_completionrings;
	u32 rx_dataoffset;
	u32 htod_mb_data_addr;
	u32 dtoh_mb_data_addr;
	u32 ring_info_addr;
	struct brcmf_pcie_console console;
	void *scratch;
	dma_addr_t scratch_dmahandle;
	void *ringupd;
	dma_addr_t ringupd_dmahandle;
	u8 version;
};

struct brcmf_pcie_core_info {
	u32 base;
	u32 wrapbase;
};

struct brcmf_pciedev_info {
	enum brcmf_pcie_state state;
	bool in_irq;
	struct pci_dev *pdev;
	char fw_name[BRCMF_FW_NAME_LEN];
	char nvram_name[BRCMF_FW_NAME_LEN];
	void __iomem *regs;
	void __iomem *tcm;
	u32 ram_base;
	u32 ram_size;
	struct brcmf_chip *ci;
	u32 coreid;
	struct brcmf_pcie_shared_info shared;
	u8 hostready;
	bool use_mailbox;
	bool use_d0_inform;
	wait_queue_head_t mbdata_resp_wait;
	bool mbdata_completed;
	bool irq_allocated;
	bool wowl_enabled;
	u8 dma_idx_sz;
	void *idxbuf;
	u32 idxbuf_sz;
	dma_addr_t idxbuf_dmahandle;
	u16 (*read_ptr)(struct brcmf_pciedev_info *devinfo, u32 mem_offset);
	void (*write_ptr)(struct brcmf_pciedev_info *devinfo, u32 mem_offset,
			  u16 value);
	struct brcmf_mp_device *settings;
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
	ulong bar1_size;
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
#ifdef DEBUG
	u32 console_interval;
	bool console_active;
	struct timer_list timer;
#endif
};

struct brcmf_pcie_ringbuf {
	struct brcmf_commonring commonring;
	dma_addr_t dma_handle;
	u32 w_idx_addr;
	u32 r_idx_addr;
	struct brcmf_pciedev_info *devinfo;
	u8 id;
};

/**
 * struct brcmf_pcie_dhi_ringinfo - dongle/host interface shared ring info
 *
 * @ringmem: dongle memory pointer to ring memory location
 * @h2d_w_idx_ptr: h2d ring write indices dongle memory pointers
 * @h2d_r_idx_ptr: h2d ring read indices dongle memory pointers
 * @d2h_w_idx_ptr: d2h ring write indices dongle memory pointers
 * @d2h_r_idx_ptr: d2h ring read indices dongle memory pointers
 * @h2d_w_idx_hostaddr: h2d ring write indices host memory pointers
 * @h2d_r_idx_hostaddr: h2d ring read indices host memory pointers
 * @d2h_w_idx_hostaddr: d2h ring write indices host memory pointers
 * @d2h_r_idx_hostaddr: d2h ring reaD indices host memory pointers
 * @max_flowrings: maximum number of tx flow rings supported.
 * @max_submissionrings: maximum number of submission rings(h2d) supported.
 * @max_completionrings: maximum number of completion rings(d2h) supported.
 */
struct brcmf_pcie_dhi_ringinfo {
	__le32			ringmem;
	__le32			h2d_w_idx_ptr;
	__le32			h2d_r_idx_ptr;
	__le32			d2h_w_idx_ptr;
	__le32			d2h_r_idx_ptr;
	struct msgbuf_buf_addr	h2d_w_idx_hostaddr;
	struct msgbuf_buf_addr	h2d_r_idx_hostaddr;
	struct msgbuf_buf_addr	d2h_w_idx_hostaddr;
	struct msgbuf_buf_addr	d2h_r_idx_hostaddr;
	__le16			max_flowrings;
	__le16			max_submissionrings;
	__le16			max_completionrings;
};

static const u32 brcmf_ring_max_item[BRCMF_NROF_COMMON_MSGRINGS] = {
	BRCMF_H2D_MSGRING_CONTROL_SUBMIT_MAX_ITEM,
	BRCMF_H2D_MSGRING_RXPOST_SUBMIT_MAX_ITEM,
	BRCMF_D2H_MSGRING_CONTROL_COMPLETE_MAX_ITEM,
	BRCMF_D2H_MSGRING_TX_COMPLETE_MAX_ITEM,
	BRCMF_D2H_MSGRING_RX_COMPLETE_MAX_ITEM
};

static const u32 brcmf_ring_itemsize_pre_v7[BRCMF_NROF_COMMON_MSGRINGS] = {
	BRCMF_H2D_MSGRING_CONTROL_SUBMIT_ITEMSIZE,
	BRCMF_H2D_MSGRING_RXPOST_SUBMIT_ITEMSIZE,
	BRCMF_D2H_MSGRING_CONTROL_COMPLETE_ITEMSIZE,
	BRCMF_D2H_MSGRING_TX_COMPLETE_ITEMSIZE_PRE_V7,
	BRCMF_D2H_MSGRING_RX_COMPLETE_ITEMSIZE_PRE_V7
};

static const u32 brcmf_ring_itemsize[BRCMF_NROF_COMMON_MSGRINGS] = {
	BRCMF_H2D_MSGRING_CONTROL_SUBMIT_ITEMSIZE,
	BRCMF_H2D_MSGRING_RXPOST_SUBMIT_ITEMSIZE,
	BRCMF_D2H_MSGRING_CONTROL_COMPLETE_ITEMSIZE,
	BRCMF_D2H_MSGRING_TX_COMPLETE_ITEMSIZE,
	BRCMF_D2H_MSGRING_RX_COMPLETE_ITEMSIZE
};

static void brcmf_pcie_setup(struct device *dev, int ret,
			     struct brcmf_fw_request *fwreq);
static struct brcmf_fw_request *
brcmf_pcie_prepare_fw_request(struct brcmf_pciedev_info *devinfo);
static void brcmf_pcie_bus_console_init(struct brcmf_pciedev_info *devinfo);
static void brcmf_pcie_bus_console_read(struct brcmf_pciedev_info *devinfo,
					bool error);

static void
brcmf_pcie_fwcon_timer(struct brcmf_pciedev_info *devinfo, bool active);
static void brcmf_pcie_debugfs_create(struct device *dev);

#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
DEFINE_RAW_SPINLOCK(pcie_lock);
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */

static u32
brcmf_pcie_read_reg32(struct brcmf_pciedev_info *devinfo, u32 reg_offset)
{
	void __iomem *address = devinfo->regs + reg_offset;

	return (ioread32(address));
}


static void
brcmf_pcie_write_reg32(struct brcmf_pciedev_info *devinfo, u32 reg_offset,
		       u32 value)
{
	void __iomem *address = devinfo->regs + reg_offset;

	iowrite32(value, address);
}


static u8
brcmf_pcie_read_tcm8(struct brcmf_pciedev_info *devinfo, u32 mem_offset)
{
	void __iomem *address = devinfo->tcm + mem_offset;
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
	unsigned long flags;
	u8 value;

	raw_spin_lock_irqsave(&pcie_lock, flags);
	if ((address - devinfo->tcm) >= devinfo->bar1_size) {
		pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN,
				       devinfo->bar1_size);
		address = address - devinfo->bar1_size;
	}
	value = ioread8(address);
	pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN, 0x0);
	raw_spin_unlock_irqrestore(&pcie_lock, flags);

	return value;
#else
	return (ioread8(address));
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
}


static u16
brcmf_pcie_read_tcm16(struct brcmf_pciedev_info *devinfo, u32 mem_offset)
{
	void __iomem *address = devinfo->tcm + mem_offset;
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
	u16 value;
	unsigned long flags;

	raw_spin_lock_irqsave(&pcie_lock, flags);
	if ((address - devinfo->tcm) >= devinfo->bar1_size) {
		pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN,
				       devinfo->bar1_size);
		address = address - devinfo->bar1_size;
	}
	value = ioread16(address);
	pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN, 0x0);
	raw_spin_unlock_irqrestore(&pcie_lock, flags);

	return value;
#else
	return (ioread16(address));
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
}


static void
brcmf_pcie_write_tcm16(struct brcmf_pciedev_info *devinfo, u32 mem_offset,
		       u16 value)
{
	void __iomem *address = devinfo->tcm + mem_offset;
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
	unsigned long flags;

	raw_spin_lock_irqsave(&pcie_lock, flags);
	if ((address - devinfo->tcm) >= devinfo->bar1_size) {
		pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN,
				       devinfo->bar1_size);
		address = address - devinfo->bar1_size;
	}

	iowrite16(value, address);
	pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN, 0x0);
	raw_spin_unlock_irqrestore(&pcie_lock, flags);
#else
	iowrite16(value, address);
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
}


static u16
brcmf_pcie_read_idx(struct brcmf_pciedev_info *devinfo, u32 mem_offset)
{
	u16 *address = devinfo->idxbuf + mem_offset;

	return (*(address));
}


static void
brcmf_pcie_write_idx(struct brcmf_pciedev_info *devinfo, u32 mem_offset,
		     u16 value)
{
	u16 *address = devinfo->idxbuf + mem_offset;

	*(address) = value;
}


static u32
brcmf_pcie_read_tcm32(struct brcmf_pciedev_info *devinfo, u32 mem_offset)
{
	void __iomem *address = devinfo->tcm + mem_offset;
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
	u32 value;
	unsigned long flags;

	raw_spin_lock_irqsave(&pcie_lock, flags);
	if ((address - devinfo->tcm) >= devinfo->bar1_size) {
		pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN,
				       devinfo->bar1_size);
		address = address - devinfo->bar1_size;
	}
	value = ioread32(address);
	pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN, 0x0);
	raw_spin_unlock_irqrestore(&pcie_lock, flags);

	return value;
#else
	return (ioread32(address));
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
}


static void
brcmf_pcie_write_tcm32(struct brcmf_pciedev_info *devinfo, u32 mem_offset,
		       u32 value)
{
	void __iomem *address = devinfo->tcm + mem_offset;
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
	unsigned long flags;

	raw_spin_lock_irqsave(&pcie_lock, flags);
	if ((address - devinfo->tcm) >= devinfo->bar1_size) {
		pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN,
				       devinfo->bar1_size);
		address = address - devinfo->bar1_size;
	}
	iowrite32(value, address);
	pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN, 0x0);
	raw_spin_unlock_irqrestore(&pcie_lock, flags);
#else
	iowrite32(value, address);
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
}


static u32
brcmf_pcie_read_ram32(struct brcmf_pciedev_info *devinfo, u32 mem_offset)
{
	void __iomem *address = devinfo->tcm + devinfo->ci->rambase
		+ mem_offset;
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
	u32 value;
	unsigned long flags;

	raw_spin_lock_irqsave(&pcie_lock, flags);
	if ((address - devinfo->tcm) >= devinfo->bar1_size) {
		pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN,
				       devinfo->bar1_size);
		address = address - devinfo->bar1_size;
	}
	value = ioread32(address);
	pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN, 0x0);
	raw_spin_unlock_irqrestore(&pcie_lock, flags);

	return value;
#else
	return (ioread32(address));
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
}


static void
brcmf_pcie_write_ram32(struct brcmf_pciedev_info *devinfo, u32 mem_offset,
		       u32 value)
{
	void __iomem *address = devinfo->tcm + devinfo->ci->rambase
		+ mem_offset;
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
	unsigned long flags;

	raw_spin_lock_irqsave(&pcie_lock, flags);
	if ((address - devinfo->tcm) >= devinfo->bar1_size) {
		pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN,
				       devinfo->bar1_size);
		address = address - devinfo->bar1_size;
	}
	iowrite32(value, address);
	pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN, 0x0);
	raw_spin_unlock_irqrestore(&pcie_lock, flags);
#else
	iowrite32(value, address);
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
}


static void
brcmf_pcie_copy_mem_todev(struct brcmf_pciedev_info *devinfo, u32 mem_offset,
			  void *srcaddr, u32 len)
{
	void __iomem *address = devinfo->tcm + mem_offset;
	__le32 *src32;
	__le16 *src16;
	u8 *src8;
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
	unsigned long flags;
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */

	if (((ulong)address & 4) || ((ulong)srcaddr & 4) || (len & 4)) {
		if (((ulong)address & 2) || ((ulong)srcaddr & 2) || (len & 2)) {
			src8 = (u8 *)srcaddr;
			while (len) {
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
				raw_spin_lock_irqsave(&pcie_lock, flags);
				if ((address - devinfo->tcm) >=
				    devinfo->bar1_size) {
					pci_write_config_dword
						(devinfo->pdev,
						 BCMA_PCI_BAR1_WIN,
						 devinfo->bar1_size);
					address = address -
						devinfo->bar1_size;
				}
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
				iowrite8(*src8, address);
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
				raw_spin_unlock_irqrestore(&pcie_lock, flags);
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
				address++;
				src8++;
				len--;
			}
		} else {
			len = len / 2;
			src16 = (__le16 *)srcaddr;
			while (len) {
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
				raw_spin_lock_irqsave(&pcie_lock, flags);
				if ((address - devinfo->tcm) >=
					devinfo->bar1_size) {
					pci_write_config_dword
						(devinfo->pdev,
						BCMA_PCI_BAR1_WIN,
						devinfo->bar1_size);
					address = address -
						devinfo->bar1_size;
				}
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
				iowrite16(le16_to_cpu(*src16), address);
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
				raw_spin_unlock_irqrestore(&pcie_lock, flags);
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
				address += 2;
				src16++;
				len--;
			}
		}
	} else {
		len = len / 4;
		src32 = (__le32 *)srcaddr;
		while (len) {
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
			raw_spin_lock_irqsave(&pcie_lock, flags);
			if ((address - devinfo->tcm) >=
			    devinfo->bar1_size) {
				pci_write_config_dword
					(devinfo->pdev,
					 BCMA_PCI_BAR1_WIN,
					 devinfo->bar1_size);
				address = address - devinfo->bar1_size;
			}
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
			iowrite32(le32_to_cpu(*src32), address);
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
			raw_spin_unlock_irqrestore(&pcie_lock, flags);
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
			address += 4;
			src32++;
			len--;
		}
	}
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
	pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN, 0x0);
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
}


static void
brcmf_pcie_copy_dev_tomem(struct brcmf_pciedev_info *devinfo, u32 mem_offset,
			  void *dstaddr, u32 len)
{
	void __iomem *address = devinfo->tcm + mem_offset;
	__le32 *dst32;
	__le16 *dst16;
	u8 *dst8;
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
	unsigned long flags;
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */

	if (((ulong)address & 4) || ((ulong)dstaddr & 4) || (len & 4)) {
		if (((ulong)address & 2) || ((ulong)dstaddr & 2) || (len & 2)) {
			dst8 = (u8 *)dstaddr;
			while (len) {
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
				raw_spin_lock_irqsave(&pcie_lock, flags);
				if ((address - devinfo->tcm) >=
				    devinfo->bar1_size) {
					pci_write_config_dword
						(devinfo->pdev,
						BCMA_PCI_BAR1_WIN,
						devinfo->bar1_size);
					address = address -
						devinfo->bar1_size;
				}
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
				*dst8 = ioread8(address);
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
				raw_spin_unlock_irqrestore(&pcie_lock, flags);
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
				address++;
				dst8++;
				len--;
			}
		} else {
			len = len / 2;
			dst16 = (__le16 *)dstaddr;
			while (len) {
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
				raw_spin_lock_irqsave(&pcie_lock, flags);
				if ((address - devinfo->tcm) >=
				    devinfo->bar1_size) {
					pci_write_config_dword
						(devinfo->pdev,
						BCMA_PCI_BAR1_WIN,
						devinfo->bar1_size);
					address = address -
						devinfo->bar1_size;
				}
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
				*dst16 = cpu_to_le16(ioread16(address));
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
				raw_spin_unlock_irqrestore(&pcie_lock, flags);
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
				address += 2;
				dst16++;
				len--;
			}
		}
	} else {
		len = len / 4;
		dst32 = (__le32 *)dstaddr;
		while (len) {
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
			raw_spin_lock_irqsave(&pcie_lock, flags);
			if ((address - devinfo->tcm) >=
			    devinfo->bar1_size) {
				pci_write_config_dword
					(devinfo->pdev,
					BCMA_PCI_BAR1_WIN,
					devinfo->bar1_size);
				address = address - devinfo->bar1_size;
			}
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
			*dst32 = cpu_to_le32(ioread32(address));
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
			raw_spin_unlock_irqrestore(&pcie_lock, flags);
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
			address += 4;
			dst32++;
			len--;
		}
	}
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
	pci_write_config_dword(devinfo->pdev, BCMA_PCI_BAR1_WIN, 0x0);
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
}


#define WRITECC32(devinfo, reg, value) brcmf_pcie_write_reg32(devinfo, \
		CHIPCREGOFFS(reg), value)


static void
brcmf_pcie_select_core(struct brcmf_pciedev_info *devinfo, u16 coreid)
{
	const struct pci_dev *pdev = devinfo->pdev;
	struct brcmf_bus *bus = dev_get_drvdata(&pdev->dev);
	struct brcmf_core *core;
	u32 bar0_win;

	core = brcmf_chip_get_core(devinfo->ci, coreid);
	if (core) {
		bar0_win = core->base;
		pci_write_config_dword(pdev, BRCMF_PCIE_BAR0_WINDOW, bar0_win);
		if (pci_read_config_dword(pdev, BRCMF_PCIE_BAR0_WINDOW,
					  &bar0_win) == 0) {
			if (bar0_win != core->base) {
				bar0_win = core->base;
				pci_write_config_dword(pdev,
						       BRCMF_PCIE_BAR0_WINDOW,
						       bar0_win);
			}
		}
	} else {
		brcmf_err(bus, "Unsupported core selected %x\n", coreid);
	}
}


static void brcmf_pcie_reset_device(struct brcmf_pciedev_info *devinfo)
{
	struct brcmf_core *core;
	u16 cfg_offset[] = { BRCMF_PCIE_CFGREG_STATUS_CMD,
			     BRCMF_PCIE_CFGREG_PM_CSR,
			     BRCMF_PCIE_CFGREG_MSI_CAP,
			     BRCMF_PCIE_CFGREG_MSI_ADDR_L,
			     BRCMF_PCIE_CFGREG_MSI_ADDR_H,
			     BRCMF_PCIE_CFGREG_MSI_DATA,
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
			     BCMA_PCI_BAR1_WIN,
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
			     BRCMF_PCIE_CFGREG_LINK_STATUS_CTRL2,
			     BRCMF_PCIE_CFGREG_RBAR_CTRL,
			     BRCMF_PCIE_CFGREG_PML1_SUB_CTRL1,
			     BRCMF_PCIE_CFGREG_REG_BAR2_CONFIG,
			     BRCMF_PCIE_CFGREG_REG_BAR3_CONFIG };
	u32 i;
	u32 val;
	u32 lsc;

	if (!devinfo->ci)
		return;

	/* Disable ASPM */
	brcmf_pcie_select_core(devinfo, BCMA_CORE_PCIE2);
	pci_read_config_dword(devinfo->pdev, BRCMF_PCIE_REG_LINK_STATUS_CTRL,
			      &lsc);
	val = lsc & (~BRCMF_PCIE_LINK_STATUS_CTRL_ASPM_ENAB);
	pci_write_config_dword(devinfo->pdev, BRCMF_PCIE_REG_LINK_STATUS_CTRL,
			       val);

	/* Watchdog reset */
	if (devinfo->ci->blhs)
		devinfo->ci->blhs->init(devinfo->ci);
	brcmf_pcie_select_core(devinfo, BCMA_CORE_CHIPCOMMON);
	WRITECC32(devinfo, watchdog, 4);
	msleep(100);
	if (devinfo->ci->blhs)
		if (devinfo->ci->blhs->post_wdreset(devinfo->ci))
			return;


	/* Restore ASPM */
	brcmf_pcie_select_core(devinfo, BCMA_CORE_PCIE2);
	pci_write_config_dword(devinfo->pdev, BRCMF_PCIE_REG_LINK_STATUS_CTRL,
			       lsc);

	core = brcmf_chip_get_core(devinfo->ci, BCMA_CORE_PCIE2);
	if (core->rev <= 13) {
		for (i = 0; i < ARRAY_SIZE(cfg_offset); i++) {
			brcmf_pcie_write_reg32(devinfo,
					       BRCMF_PCIE_PCIE2REG_CONFIGADDR,
					       cfg_offset[i]);
			val = brcmf_pcie_read_reg32(devinfo,
				BRCMF_PCIE_PCIE2REG_CONFIGDATA);
			brcmf_dbg(PCIE, "config offset 0x%04x, value 0x%04x\n",
				  cfg_offset[i], val);
			brcmf_pcie_write_reg32(devinfo,
					       BRCMF_PCIE_PCIE2REG_CONFIGDATA,
					       val);
		}
	}
}


static void brcmf_pcie_attach(struct brcmf_pciedev_info *devinfo)
{
	u32 config;

	/* BAR1 window may not be sized properly */
	brcmf_pcie_select_core(devinfo, BCMA_CORE_PCIE2);
	brcmf_pcie_write_reg32(devinfo, BRCMF_PCIE_PCIE2REG_CONFIGADDR, 0x4e0);
	config = brcmf_pcie_read_reg32(devinfo, BRCMF_PCIE_PCIE2REG_CONFIGDATA);
	brcmf_pcie_write_reg32(devinfo, BRCMF_PCIE_PCIE2REG_CONFIGDATA, config);

	device_wakeup_enable(&devinfo->pdev->dev);
}


static int brcmf_pcie_bus_readshared(struct brcmf_pciedev_info *devinfo,
				     u32 nvram_csm)
{
	struct brcmf_bus *bus = dev_get_drvdata(&devinfo->pdev->dev);
	u32 loop_counter;
	u32 addr_le;
	u32 addr = 0;

	loop_counter = BRCMF_PCIE_READ_SHARED_TIMEOUT / 50;
	while ((addr == 0 || addr == nvram_csm) && (loop_counter)) {
		msleep(50);
		addr_le = brcmf_pcie_read_ram32(devinfo,
						devinfo->ci->ramsize - 4);
		addr = le32_to_cpu(addr_le);
		loop_counter--;
	}
	if (addr == 0 || addr == nvram_csm || addr < devinfo->ci->rambase ||
	    addr >= devinfo->ci->rambase + devinfo->ci->ramsize) {
		brcmf_err(bus, "Invalid shared RAM address 0x%08x\n", addr);
		return -ENODEV;
	}
	devinfo->shared.tcm_base_address = addr;
	brcmf_dbg(PCIE, "Shared RAM addr: 0x%08x\n", addr);

	brcmf_pcie_bus_console_init(devinfo);
	return 0;
}

static int brcmf_pcie_enter_download_state(struct brcmf_pciedev_info *devinfo)
{
	struct brcmf_bus *bus = dev_get_drvdata(&devinfo->pdev->dev);
	int err = 0;

	if (devinfo->ci->chip == BRCM_CC_43602_CHIP_ID) {
		brcmf_pcie_select_core(devinfo, BCMA_CORE_ARM_CR4);
		brcmf_pcie_write_reg32(devinfo, BRCMF_PCIE_ARMCR4REG_BANKIDX,
				       5);
		brcmf_pcie_write_reg32(devinfo, BRCMF_PCIE_ARMCR4REG_BANKPDA,
				       0);
		brcmf_pcie_write_reg32(devinfo, BRCMF_PCIE_ARMCR4REG_BANKIDX,
				       7);
		brcmf_pcie_write_reg32(devinfo, BRCMF_PCIE_ARMCR4REG_BANKPDA,
				       0);
	}

	if (devinfo->ci->blhs) {
		err = devinfo->ci->blhs->prep_fwdl(devinfo->ci);
		if (err) {
			brcmf_err(bus, "FW download preparation failed");
			return err;
		}

		if (!brcmf_pcie_bus_readshared(devinfo, 0))
			brcmf_pcie_bus_console_read(devinfo, false);
	}

	return err;
}


static int brcmf_pcie_exit_download_state(struct brcmf_pciedev_info *devinfo,
					  u32 resetintr)
{
	struct brcmf_core *core;

	if (devinfo->ci->chip == BRCM_CC_43602_CHIP_ID) {
		core = brcmf_chip_get_core(devinfo->ci, BCMA_CORE_INTERNAL_MEM);
		brcmf_chip_resetcore(core, 0, 0, 0);
	}

	if (devinfo->ci->blhs) {
		brcmf_pcie_bus_console_read(devinfo, false);
		devinfo->ci->blhs->post_nvramdl(devinfo->ci);
	} else {
		if (!brcmf_chip_set_active(devinfo->ci, resetintr))
			return -EINVAL;
	}

	return 0;
}


static int
brcmf_pcie_send_mb_data(struct brcmf_pciedev_info *devinfo, u32 htod_mb_data)
{
	struct brcmf_pcie_shared_info *shared;
	struct brcmf_bus *bus;
	int err;
	struct brcmf_core *core;
	u32 addr;
	u32 cur_htod_mb_data;
	u32 i;

	shared = &devinfo->shared;
	bus = dev_get_drvdata(&devinfo->pdev->dev);
	if (shared->version >= BRCMF_PCIE_SHARED_VERSION_6 &&
	    !devinfo->use_mailbox) {
		err = brcmf_msgbuf_tx_mbdata(bus->drvr, htod_mb_data);
		if (err) {
			brcmf_err(bus, "sendimg mbdata failed err=%d\n", err);
			return err;
		}
	} else {
		addr = shared->htod_mb_data_addr;
		cur_htod_mb_data = brcmf_pcie_read_tcm32(devinfo, addr);

		if (cur_htod_mb_data != 0)
			brcmf_dbg(PCIE, "MB transaction is already pending 0x%04x\n",
				  cur_htod_mb_data);

		i = 0;
		while (cur_htod_mb_data != 0) {
			msleep(10);
			i++;
			if (i > 100)
				return -EIO;
			cur_htod_mb_data = brcmf_pcie_read_tcm32(devinfo, addr);
		}

		brcmf_pcie_write_tcm32(devinfo, addr, htod_mb_data);
		pci_write_config_dword(devinfo->pdev, BRCMF_PCIE_REG_SBMBX, 1);

		/* Send mailbox interrupt twice as a hardware workaround */
		core = brcmf_chip_get_core(devinfo->ci, BCMA_CORE_PCIE2);
		if (core->rev <= 13)
			pci_write_config_dword(devinfo->pdev,
					       BRCMF_PCIE_REG_SBMBX, 1);
	}
	return 0;
}


static u32 brcmf_pcie_read_mb_data(struct brcmf_pciedev_info *devinfo)
{
	struct brcmf_pcie_shared_info *shared;
	u32 addr;
	u32 dtoh_mb_data;

	shared = &devinfo->shared;
	addr = shared->dtoh_mb_data_addr;
	dtoh_mb_data = brcmf_pcie_read_tcm32(devinfo, addr);
	brcmf_pcie_write_tcm32(devinfo, addr, 0);
	return dtoh_mb_data;
}

void brcmf_pcie_handle_mb_data(struct brcmf_bus *bus_if, u32 d2h_mb_data)
{
	struct brcmf_pciedev *buspub = bus_if->bus_priv.pcie;
	struct brcmf_pciedev_info *devinfo = buspub->devinfo;

	brcmf_dbg(INFO, "D2H_MB_DATA: 0x%04x\n", d2h_mb_data);

	if (d2h_mb_data & BRCMF_D2H_DEV_DS_ENTER_REQ) {
		brcmf_dbg(INFO, "D2H_MB_DATA: DEEP SLEEP REQ\n");
		brcmf_pcie_send_mb_data(devinfo, BRCMF_H2D_HOST_DS_ACK);
		brcmf_dbg(INFO, "D2H_MB_DATA: sent DEEP SLEEP ACK\n");
	}

	if (d2h_mb_data & BRCMF_D2H_DEV_DS_EXIT_NOTE)
		brcmf_dbg(INFO, "D2H_MB_DATA: DEEP SLEEP EXIT\n");
	if (d2h_mb_data & BRCMF_D2H_DEV_D3_ACK) {
		brcmf_dbg(INFO, "D2H_MB_DATA: D3 ACK\n");
		devinfo->mbdata_completed = true;
		wake_up(&devinfo->mbdata_resp_wait);
	}

	if (d2h_mb_data & BRCMF_D2H_DEV_FWHALT) {
		brcmf_dbg(INFO, "D2H_MB_DATA: FW HALT\n");
		brcmf_fw_crashed(&devinfo->pdev->dev);
	}
}

static void brcmf_pcie_bus_console_init(struct brcmf_pciedev_info *devinfo)
{
	struct brcmf_pcie_shared_info *shared;
	struct brcmf_pcie_console *console;
	u32 buf_addr;
	u32 addr;

	shared = &devinfo->shared;
	console = &shared->console;
	addr = shared->tcm_base_address + BRCMF_SHARED_CONSOLE_ADDR_OFFSET;
	console->base_addr = brcmf_pcie_read_tcm32(devinfo, addr);

	addr = console->base_addr + BRCMF_CONSOLE_BUFADDR_OFFSET;
	buf_addr = brcmf_pcie_read_tcm32(devinfo, addr);
	/* reset console index when buffer address is updated */
	if (console->buf_addr != buf_addr) {
		console->buf_addr = buf_addr;
		console->read_idx = 0;
	}
	addr = console->base_addr + BRCMF_CONSOLE_BUFSIZE_OFFSET;
	console->bufsize = brcmf_pcie_read_tcm32(devinfo, addr);

	brcmf_dbg(FWCON, "Console: base %x, buf %x, size %d\n",
		  console->base_addr, console->buf_addr, console->bufsize);
}

/**
 * brcmf_pcie_bus_console_read - reads firmware messages
 *
 * @error: specifies if error has occurred (prints messages unconditionally)
 */
static void brcmf_pcie_bus_console_read(struct brcmf_pciedev_info *devinfo,
					bool error)
{
	struct pci_dev *pdev = devinfo->pdev;
	struct brcmf_bus *bus = dev_get_drvdata(&pdev->dev);
	struct brcmf_pcie_console *console;
	u32 addr;
	u8 ch;
	u32 newidx;

	if (!error && !BRCMF_FWCON_ON())
		return;

	console = &devinfo->shared.console;
	addr = console->base_addr + BRCMF_CONSOLE_WRITEIDX_OFFSET;
	newidx = brcmf_pcie_read_tcm32(devinfo, addr);
	while (newidx != console->read_idx) {
		addr = console->buf_addr + console->read_idx;
		ch = brcmf_pcie_read_tcm8(devinfo, addr);
		console->read_idx++;
		if (console->read_idx == console->bufsize)
			console->read_idx = 0;
		if (ch == '\r')
			continue;
		console->log_str[console->log_idx] = ch;
		console->log_idx++;
		if ((ch != '\n') &&
		    (console->log_idx == (sizeof(console->log_str) - 2))) {
			ch = '\n';
			console->log_str[console->log_idx] = ch;
			console->log_idx++;
		}
		if (ch == '\n') {
			console->log_str[console->log_idx] = 0;
			if (error)
				__brcmf_err(bus, __func__, "CONSOLE: %s",
					    console->log_str);
			else
				pr_debug("CONSOLE: %s", console->log_str);
			console->log_idx = 0;
		}
	}
}


static void brcmf_pcie_intr_disable(struct brcmf_pciedev_info *devinfo)
{
	brcmf_pcie_write_reg32(devinfo,
			       BRCMF_PCIE_PCIE2REG_MAILBOXMASK(devinfo), 0);
}


static void brcmf_pcie_intr_enable(struct brcmf_pciedev_info *devinfo)
{
	brcmf_pcie_write_reg32(devinfo,
			       BRCMF_PCIE_PCIE2REG_MAILBOXMASK(devinfo),
			       BRCMF_PCIE_MB_INT_D2H_DB(devinfo) |
			       BRCMF_PCIE_MB_INT_FN0_0 |
			       BRCMF_PCIE_MB_INT_FN0_1);
}

static void brcmf_pcie_hostready(struct brcmf_pciedev_info *devinfo)
{
	if (devinfo->shared.flags & BRCMF_PCIE_SHARED_HOSTRDY_DB1)
		brcmf_pcie_write_reg32(devinfo,
				       BRCMF_PCIE_PCIE2REG_H2D_MAILBOX_1, 1);
}

static irqreturn_t brcmf_pcie_quick_check_isr(int irq, void *arg)
{
	struct brcmf_pciedev_info *devinfo = (struct brcmf_pciedev_info *)arg;

	if (brcmf_pcie_read_reg32(devinfo,
				  BRCMF_PCIE_PCIE2REG_MAILBOXINT(devinfo))) {
		brcmf_pcie_intr_disable(devinfo);
		brcmf_dbg(PCIE, "Enter\n");
		return IRQ_WAKE_THREAD;
	}
	return IRQ_NONE;
}


static irqreturn_t brcmf_pcie_isr_thread(int irq, void *arg)
{
	struct brcmf_pciedev_info *devinfo = (struct brcmf_pciedev_info *)arg;
	u32 status;
	u32 d2h_mbdata;
	struct pci_dev *pdev = devinfo->pdev;
	struct brcmf_bus *bus = dev_get_drvdata(&pdev->dev);

	devinfo->in_irq = true;
	status = brcmf_pcie_read_reg32(devinfo,
				       BRCMF_PCIE_PCIE2REG_MAILBOXINT(devinfo));
	brcmf_dbg(PCIE, "Enter %x\n", status);
	if (status) {
		brcmf_pcie_write_reg32(devinfo,
				       BRCMF_PCIE_PCIE2REG_MAILBOXINT(devinfo),
				       status);
		if (status & (BRCMF_PCIE_MB_INT_FN0_0 |
			      BRCMF_PCIE_MB_INT_FN0_1))	{
			d2h_mbdata = brcmf_pcie_read_mb_data(devinfo);
			brcmf_pcie_handle_mb_data(bus, d2h_mbdata);
		}

		if (status & BRCMF_PCIE_MB_INT_D2H_DB(devinfo)) {
			if (devinfo->state == BRCMFMAC_PCIE_STATE_UP)
				brcmf_proto_msgbuf_rx_trigger(
							&devinfo->pdev->dev);
		}
	}
	brcmf_pcie_bus_console_read(devinfo, false);
	if (devinfo->state == BRCMFMAC_PCIE_STATE_UP)
		brcmf_pcie_intr_enable(devinfo);
	devinfo->in_irq = false;
	return IRQ_HANDLED;
}


static int brcmf_pcie_request_irq(struct brcmf_pciedev_info *devinfo)
{
	struct pci_dev *pdev = devinfo->pdev;
	struct brcmf_bus *bus = dev_get_drvdata(&pdev->dev);

	brcmf_pcie_intr_disable(devinfo);

	brcmf_dbg(PCIE, "Enter\n");

	pci_enable_msi(pdev);
	if (request_threaded_irq(pdev->irq, brcmf_pcie_quick_check_isr,
				 brcmf_pcie_isr_thread, IRQF_SHARED,
				 "brcmf_pcie_intr", devinfo)) {
		pci_disable_msi(pdev);
		brcmf_err(bus, "Failed to request IRQ %d\n", pdev->irq);
		return -EIO;
	}
	devinfo->irq_allocated = true;
	return 0;
}


static void brcmf_pcie_release_irq(struct brcmf_pciedev_info *devinfo)
{
	struct pci_dev *pdev = devinfo->pdev;
	struct brcmf_bus *bus = dev_get_drvdata(&pdev->dev);
	u32 status;
	u32 count;

	if (!devinfo->irq_allocated)
		return;

	brcmf_pcie_intr_disable(devinfo);
	free_irq(pdev->irq, devinfo);
	pci_disable_msi(pdev);

	msleep(50);
	count = 0;
	while ((devinfo->in_irq) && (count < 20)) {
		msleep(50);
		count++;
	}
	if (devinfo->in_irq)
		brcmf_err(bus, "Still in IRQ (processing) !!!\n");

	status = brcmf_pcie_read_reg32(devinfo,
				       BRCMF_PCIE_PCIE2REG_MAILBOXINT(devinfo));
	brcmf_pcie_write_reg32(devinfo,
			       BRCMF_PCIE_PCIE2REG_MAILBOXINT(devinfo),
			       status);

	devinfo->irq_allocated = false;
}


static int brcmf_pcie_ring_mb_write_rptr(void *ctx)
{
	struct brcmf_pcie_ringbuf *ring = (struct brcmf_pcie_ringbuf *)ctx;
	struct brcmf_pciedev_info *devinfo = ring->devinfo;
	struct brcmf_commonring *commonring = &ring->commonring;

	if (devinfo->state != BRCMFMAC_PCIE_STATE_UP)
		return -EIO;

	brcmf_dbg(PCIE, "W r_ptr %d (%d), ring %d\n", commonring->r_ptr,
		  commonring->w_ptr, ring->id);

	devinfo->write_ptr(devinfo, ring->r_idx_addr, commonring->r_ptr);

	return 0;
}


static int brcmf_pcie_ring_mb_write_wptr(void *ctx)
{
	struct brcmf_pcie_ringbuf *ring = (struct brcmf_pcie_ringbuf *)ctx;
	struct brcmf_pciedev_info *devinfo = ring->devinfo;
	struct brcmf_commonring *commonring = &ring->commonring;

	if (devinfo->state != BRCMFMAC_PCIE_STATE_UP)
		return -EIO;

	brcmf_dbg(PCIE, "W w_ptr %d (%d), ring %d\n", commonring->w_ptr,
		  commonring->r_ptr, ring->id);

	devinfo->write_ptr(devinfo, ring->w_idx_addr, commonring->w_ptr);

	return 0;
}


static int brcmf_pcie_ring_mb_ring_bell(void *ctx)
{
	struct brcmf_pcie_ringbuf *ring = (struct brcmf_pcie_ringbuf *)ctx;
	struct brcmf_pciedev_info *devinfo = ring->devinfo;

	if (devinfo->state != BRCMFMAC_PCIE_STATE_UP)
		return -EIO;

	brcmf_dbg(PCIE, "RING !\n");
	/* Any arbitrary value will do, lets use 1 */
	brcmf_pcie_write_reg32(devinfo, BRCMF_PCIE_PCIE2REG_H2D_MAILBOX_0, 1);

	return 0;
}


static int brcmf_pcie_ring_mb_update_rptr(void *ctx)
{
	struct brcmf_pcie_ringbuf *ring = (struct brcmf_pcie_ringbuf *)ctx;
	struct brcmf_pciedev_info *devinfo = ring->devinfo;
	struct brcmf_commonring *commonring = &ring->commonring;

	if (devinfo->state != BRCMFMAC_PCIE_STATE_UP)
		return -EIO;

	commonring->r_ptr = devinfo->read_ptr(devinfo, ring->r_idx_addr);

	brcmf_dbg(PCIE, "R r_ptr %d (%d), ring %d\n", commonring->r_ptr,
		  commonring->w_ptr, ring->id);

	return 0;
}


static int brcmf_pcie_ring_mb_update_wptr(void *ctx)
{
	struct brcmf_pcie_ringbuf *ring = (struct brcmf_pcie_ringbuf *)ctx;
	struct brcmf_pciedev_info *devinfo = ring->devinfo;
	struct brcmf_commonring *commonring = &ring->commonring;

	if (devinfo->state != BRCMFMAC_PCIE_STATE_UP)
		return -EIO;

	commonring->w_ptr = devinfo->read_ptr(devinfo, ring->w_idx_addr);

	brcmf_dbg(PCIE, "R w_ptr %d (%d), ring %d\n", commonring->w_ptr,
		  commonring->r_ptr, ring->id);

	return 0;
}


static void *
brcmf_pcie_init_dmabuffer_for_device(struct brcmf_pciedev_info *devinfo,
				     u32 size, u32 tcm_dma_phys_addr,
				     dma_addr_t *dma_handle)
{
	void *ring;
	u64 address;

	ring = dma_alloc_coherent(&devinfo->pdev->dev, size, dma_handle,
				  GFP_KERNEL);
	if (!ring)
		return NULL;

	address = (u64)*dma_handle;
	brcmf_pcie_write_tcm32(devinfo, tcm_dma_phys_addr,
			       address & 0xffffffff);
	brcmf_pcie_write_tcm32(devinfo, tcm_dma_phys_addr + 4, address >> 32);

	return (ring);
}


static struct brcmf_pcie_ringbuf *
brcmf_pcie_alloc_dma_and_ring(struct brcmf_pciedev_info *devinfo, u32 ring_id,
			      u32 tcm_ring_phys_addr)
{
	void *dma_buf;
	dma_addr_t dma_handle;
	struct brcmf_pcie_ringbuf *ring;
	u32 size;
	u32 addr;
	const u32 *ring_itemsize_array;

	if (devinfo->shared.version < BRCMF_PCIE_SHARED_VERSION_7)
		ring_itemsize_array = brcmf_ring_itemsize_pre_v7;
	else
		ring_itemsize_array = brcmf_ring_itemsize;

	size = brcmf_ring_max_item[ring_id] * ring_itemsize_array[ring_id];
	dma_buf = brcmf_pcie_init_dmabuffer_for_device(devinfo, size,
			tcm_ring_phys_addr + BRCMF_RING_MEM_BASE_ADDR_OFFSET,
			&dma_handle);
	if (!dma_buf)
		return NULL;

	addr = tcm_ring_phys_addr + BRCMF_RING_MAX_ITEM_OFFSET;
	brcmf_pcie_write_tcm16(devinfo, addr, brcmf_ring_max_item[ring_id]);
	addr = tcm_ring_phys_addr + BRCMF_RING_LEN_ITEMS_OFFSET;
	brcmf_pcie_write_tcm16(devinfo, addr, ring_itemsize_array[ring_id]);

	ring = kzalloc(sizeof(*ring), GFP_KERNEL);
	if (!ring) {
		dma_free_coherent(&devinfo->pdev->dev, size, dma_buf,
				  dma_handle);
		return NULL;
	}
	brcmf_commonring_config(&ring->commonring, brcmf_ring_max_item[ring_id],
				ring_itemsize_array[ring_id], dma_buf);
	ring->dma_handle = dma_handle;
	ring->devinfo = devinfo;
	brcmf_commonring_register_cb(&ring->commonring,
				     brcmf_pcie_ring_mb_ring_bell,
				     brcmf_pcie_ring_mb_update_rptr,
				     brcmf_pcie_ring_mb_update_wptr,
				     brcmf_pcie_ring_mb_write_rptr,
				     brcmf_pcie_ring_mb_write_wptr, ring);

	return (ring);
}


static void brcmf_pcie_release_ringbuffer(struct device *dev,
					  struct brcmf_pcie_ringbuf *ring)
{
	void *dma_buf;
	u32 size;

	if (!ring)
		return;

	dma_buf = ring->commonring.buf_addr;
	if (dma_buf) {
		size = ring->commonring.depth * ring->commonring.item_len;
		dma_free_coherent(dev, size, dma_buf, ring->dma_handle);
	}
	kfree(ring);
}


static void brcmf_pcie_release_ringbuffers(struct brcmf_pciedev_info *devinfo)
{
	u32 i;

	for (i = 0; i < BRCMF_NROF_COMMON_MSGRINGS; i++) {
		brcmf_pcie_release_ringbuffer(&devinfo->pdev->dev,
					      devinfo->shared.commonrings[i]);
		devinfo->shared.commonrings[i] = NULL;
	}
	kfree(devinfo->shared.flowrings);
	devinfo->shared.flowrings = NULL;
	if (devinfo->idxbuf) {
		dma_free_coherent(&devinfo->pdev->dev,
				  devinfo->idxbuf_sz,
				  devinfo->idxbuf,
				  devinfo->idxbuf_dmahandle);
		devinfo->idxbuf = NULL;
	}
}


static int brcmf_pcie_init_ringbuffers(struct brcmf_pciedev_info *devinfo)
{
	struct brcmf_bus *bus = dev_get_drvdata(&devinfo->pdev->dev);
	struct brcmf_pcie_ringbuf *ring;
	struct brcmf_pcie_ringbuf *rings;
	u32 d2h_w_idx_ptr;
	u32 d2h_r_idx_ptr;
	u32 h2d_w_idx_ptr;
	u32 h2d_r_idx_ptr;
	u32 ring_mem_ptr;
	u32 i;
	u64 address;
	u32 bufsz;
	u8 idx_offset;
	struct brcmf_pcie_dhi_ringinfo ringinfo;
	u16 max_flowrings;
	u16 max_submissionrings;
	u16 max_completionrings;
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
	brcmf_pcie_copy_dev_tomem(devinfo, devinfo->shared.ring_info_addr,
				  &ringinfo, sizeof(ringinfo));
#else
	memcpy_fromio(&ringinfo, devinfo->tcm + devinfo->shared.ring_info_addr,
		      sizeof(ringinfo));
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */

	if (devinfo->shared.version >= 6) {
		max_submissionrings = le16_to_cpu(ringinfo.max_submissionrings);
		max_flowrings = le16_to_cpu(ringinfo.max_flowrings);
		max_completionrings = le16_to_cpu(ringinfo.max_completionrings);
	} else {
		max_submissionrings = le16_to_cpu(ringinfo.max_flowrings);
		max_flowrings = max_submissionrings -
				BRCMF_NROF_H2D_COMMON_MSGRINGS;
		max_completionrings = BRCMF_NROF_D2H_COMMON_MSGRINGS;
	}
	if (max_flowrings > 256) {
		brcmf_err(bus, "invalid max_flowrings(%d)\n", max_flowrings);
		return -EIO;
	}

	if (devinfo->dma_idx_sz != 0) {
		bufsz = (max_submissionrings + max_completionrings) *
			devinfo->dma_idx_sz * 2;
		devinfo->idxbuf = dma_alloc_coherent(&devinfo->pdev->dev, bufsz,
						     &devinfo->idxbuf_dmahandle,
						     GFP_KERNEL);
		if (!devinfo->idxbuf)
			devinfo->dma_idx_sz = 0;
	}

	if (devinfo->dma_idx_sz == 0) {
		d2h_w_idx_ptr = le32_to_cpu(ringinfo.d2h_w_idx_ptr);
		d2h_r_idx_ptr = le32_to_cpu(ringinfo.d2h_r_idx_ptr);
		h2d_w_idx_ptr = le32_to_cpu(ringinfo.h2d_w_idx_ptr);
		h2d_r_idx_ptr = le32_to_cpu(ringinfo.h2d_r_idx_ptr);
		idx_offset = sizeof(u32);
		devinfo->write_ptr = brcmf_pcie_write_tcm16;
		devinfo->read_ptr = brcmf_pcie_read_tcm16;
		brcmf_dbg(PCIE, "Using TCM indices\n");
	} else {
		memset(devinfo->idxbuf, 0, bufsz);
		devinfo->idxbuf_sz = bufsz;
		idx_offset = devinfo->dma_idx_sz;
		devinfo->write_ptr = brcmf_pcie_write_idx;
		devinfo->read_ptr = brcmf_pcie_read_idx;

		h2d_w_idx_ptr = 0;
		address = (u64)devinfo->idxbuf_dmahandle;
		ringinfo.h2d_w_idx_hostaddr.low_addr =
			cpu_to_le32(address & 0xffffffff);
		ringinfo.h2d_w_idx_hostaddr.high_addr =
			cpu_to_le32(address >> 32);

		h2d_r_idx_ptr = h2d_w_idx_ptr +
				max_submissionrings * idx_offset;
		address += max_submissionrings * idx_offset;
		ringinfo.h2d_r_idx_hostaddr.low_addr =
			cpu_to_le32(address & 0xffffffff);
		ringinfo.h2d_r_idx_hostaddr.high_addr =
			cpu_to_le32(address >> 32);

		d2h_w_idx_ptr = h2d_r_idx_ptr +
				max_submissionrings * idx_offset;
		address += max_submissionrings * idx_offset;
		ringinfo.d2h_w_idx_hostaddr.low_addr =
			cpu_to_le32(address & 0xffffffff);
		ringinfo.d2h_w_idx_hostaddr.high_addr =
			cpu_to_le32(address >> 32);

		d2h_r_idx_ptr = d2h_w_idx_ptr +
				max_completionrings * idx_offset;
		address += max_completionrings * idx_offset;
		ringinfo.d2h_r_idx_hostaddr.low_addr =
			cpu_to_le32(address & 0xffffffff);
		ringinfo.d2h_r_idx_hostaddr.high_addr =
			cpu_to_le32(address >> 32);

#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
		brcmf_pcie_copy_mem_todev(devinfo,
					  devinfo->shared.ring_info_addr,
					  &ringinfo, sizeof(ringinfo));
#else
		memcpy_toio(devinfo->tcm + devinfo->shared.ring_info_addr,
			    &ringinfo, sizeof(ringinfo));
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */
		brcmf_dbg(PCIE, "Using host memory indices\n");
	}

	ring_mem_ptr = le32_to_cpu(ringinfo.ringmem);

	for (i = 0; i < BRCMF_NROF_H2D_COMMON_MSGRINGS; i++) {
		ring = brcmf_pcie_alloc_dma_and_ring(devinfo, i, ring_mem_ptr);
		if (!ring)
			goto fail;
		ring->w_idx_addr = h2d_w_idx_ptr;
		ring->r_idx_addr = h2d_r_idx_ptr;
		ring->id = i;
		devinfo->shared.commonrings[i] = ring;

		h2d_w_idx_ptr += idx_offset;
		h2d_r_idx_ptr += idx_offset;
		ring_mem_ptr += BRCMF_RING_MEM_SZ;
	}

	for (i = BRCMF_NROF_H2D_COMMON_MSGRINGS;
	     i < BRCMF_NROF_COMMON_MSGRINGS; i++) {
		ring = brcmf_pcie_alloc_dma_and_ring(devinfo, i, ring_mem_ptr);
		if (!ring)
			goto fail;
		ring->w_idx_addr = d2h_w_idx_ptr;
		ring->r_idx_addr = d2h_r_idx_ptr;
		ring->id = i;
		devinfo->shared.commonrings[i] = ring;

		d2h_w_idx_ptr += idx_offset;
		d2h_r_idx_ptr += idx_offset;
		ring_mem_ptr += BRCMF_RING_MEM_SZ;
	}

	devinfo->shared.max_flowrings = max_flowrings;
	devinfo->shared.max_submissionrings = max_submissionrings;
	devinfo->shared.max_completionrings = max_completionrings;
	rings = kcalloc(max_flowrings, sizeof(*ring), GFP_KERNEL);
	if (!rings)
		goto fail;

	brcmf_dbg(PCIE, "Nr of flowrings is %d\n", max_flowrings);

	for (i = 0; i < max_flowrings; i++) {
		ring = &rings[i];
		ring->devinfo = devinfo;
		ring->id = i + BRCMF_H2D_MSGRING_FLOWRING_IDSTART;
		brcmf_commonring_register_cb(&ring->commonring,
					     brcmf_pcie_ring_mb_ring_bell,
					     brcmf_pcie_ring_mb_update_rptr,
					     brcmf_pcie_ring_mb_update_wptr,
					     brcmf_pcie_ring_mb_write_rptr,
					     brcmf_pcie_ring_mb_write_wptr,
					     ring);
		ring->w_idx_addr = h2d_w_idx_ptr;
		ring->r_idx_addr = h2d_r_idx_ptr;
		h2d_w_idx_ptr += idx_offset;
		h2d_r_idx_ptr += idx_offset;
	}
	devinfo->shared.flowrings = rings;

	return 0;

fail:
	brcmf_err(bus, "Allocating ring buffers failed\n");
	brcmf_pcie_release_ringbuffers(devinfo);
	return -ENOMEM;
}


static void
brcmf_pcie_release_scratchbuffers(struct brcmf_pciedev_info *devinfo)
{
	if (devinfo->shared.scratch)
		dma_free_coherent(&devinfo->pdev->dev,
				  BRCMF_DMA_D2H_SCRATCH_BUF_LEN,
				  devinfo->shared.scratch,
				  devinfo->shared.scratch_dmahandle);
	if (devinfo->shared.ringupd)
		dma_free_coherent(&devinfo->pdev->dev,
				  BRCMF_DMA_D2H_RINGUPD_BUF_LEN,
				  devinfo->shared.ringupd,
				  devinfo->shared.ringupd_dmahandle);
}

static int brcmf_pcie_init_scratchbuffers(struct brcmf_pciedev_info *devinfo)
{
	struct brcmf_bus *bus = dev_get_drvdata(&devinfo->pdev->dev);
	u64 address;
	u32 addr;

	devinfo->shared.scratch =
		dma_alloc_coherent(&devinfo->pdev->dev,
				   BRCMF_DMA_D2H_SCRATCH_BUF_LEN,
				   &devinfo->shared.scratch_dmahandle,
				   GFP_KERNEL);
	if (!devinfo->shared.scratch)
		goto fail;

	addr = devinfo->shared.tcm_base_address +
	       BRCMF_SHARED_DMA_SCRATCH_ADDR_OFFSET;
	address = (u64)devinfo->shared.scratch_dmahandle;
	brcmf_pcie_write_tcm32(devinfo, addr, address & 0xffffffff);
	brcmf_pcie_write_tcm32(devinfo, addr + 4, address >> 32);
	addr = devinfo->shared.tcm_base_address +
	       BRCMF_SHARED_DMA_SCRATCH_LEN_OFFSET;
	brcmf_pcie_write_tcm32(devinfo, addr, BRCMF_DMA_D2H_SCRATCH_BUF_LEN);

	devinfo->shared.ringupd =
		dma_alloc_coherent(&devinfo->pdev->dev,
				   BRCMF_DMA_D2H_RINGUPD_BUF_LEN,
				   &devinfo->shared.ringupd_dmahandle,
				   GFP_KERNEL);
	if (!devinfo->shared.ringupd)
		goto fail;

	addr = devinfo->shared.tcm_base_address +
	       BRCMF_SHARED_DMA_RINGUPD_ADDR_OFFSET;
	address = (u64)devinfo->shared.ringupd_dmahandle;
	brcmf_pcie_write_tcm32(devinfo, addr, address & 0xffffffff);
	brcmf_pcie_write_tcm32(devinfo, addr + 4, address >> 32);
	addr = devinfo->shared.tcm_base_address +
	       BRCMF_SHARED_DMA_RINGUPD_LEN_OFFSET;
	brcmf_pcie_write_tcm32(devinfo, addr, BRCMF_DMA_D2H_RINGUPD_BUF_LEN);
	return 0;

fail:
	brcmf_err(bus, "Allocating scratch buffers failed\n");
	brcmf_pcie_release_scratchbuffers(devinfo);
	return -ENOMEM;
}


static void brcmf_pcie_down(struct device *dev)
{
	struct brcmf_bus *bus_if = dev_get_drvdata(dev);
	struct brcmf_pciedev *pcie_bus_dev = bus_if->bus_priv.pcie;
	struct brcmf_pciedev_info *devinfo = pcie_bus_dev->devinfo;

	brcmf_pcie_fwcon_timer(devinfo, false);
}


static int brcmf_pcie_tx(struct device *dev, struct sk_buff *skb)
{
	return 0;
}


static int brcmf_pcie_tx_ctlpkt(struct device *dev, unsigned char *msg,
				uint len)
{
	return 0;
}


static int brcmf_pcie_rx_ctlpkt(struct device *dev, unsigned char *msg,
				uint len)
{
	return 0;
}


static void brcmf_pcie_wowl_config(struct device *dev, bool enabled)
{
	struct brcmf_bus *bus_if = dev_get_drvdata(dev);
	struct brcmf_pciedev *buspub = bus_if->bus_priv.pcie;
	struct brcmf_pciedev_info *devinfo = buspub->devinfo;

	brcmf_dbg(PCIE, "Configuring WOWL, enabled=%d\n", enabled);
	devinfo->wowl_enabled = enabled;
}


static size_t brcmf_pcie_get_ramsize(struct device *dev)
{
	struct brcmf_bus *bus_if = dev_get_drvdata(dev);
	struct brcmf_pciedev *buspub = bus_if->bus_priv.pcie;
	struct brcmf_pciedev_info *devinfo = buspub->devinfo;

	return devinfo->ci->ramsize - devinfo->ci->srsize;
}


static int brcmf_pcie_get_memdump(struct device *dev, void *data, size_t len)
{
	struct brcmf_bus *bus_if = dev_get_drvdata(dev);
	struct brcmf_pciedev *buspub = bus_if->bus_priv.pcie;
	struct brcmf_pciedev_info *devinfo = buspub->devinfo;

	brcmf_dbg(PCIE, "dump at 0x%08X: len=%zu\n", devinfo->ci->rambase, len);
	brcmf_pcie_copy_dev_tomem(devinfo, devinfo->ci->rambase, data, len);
	return 0;
}

static
int brcmf_pcie_get_fwname(struct device *dev, const char *ext, u8 *fw_name)
{
	struct brcmf_bus *bus_if = dev_get_drvdata(dev);
	struct brcmf_fw_request *fwreq;
	struct brcmf_fw_name fwnames[] = {
		{ ext, fw_name },
	};
	u32 chip;

	chip = bus_if->chip;
	fwreq = brcmf_fw_alloc_request(chip, bus_if->chiprev,
				       brcmf_pcie_fwnames,
				       ARRAY_SIZE(brcmf_pcie_fwnames),
				       fwnames, ARRAY_SIZE(fwnames));
	if (!fwreq)
		return -ENOMEM;

	kfree(fwreq);
	return 0;
}

static int brcmf_pcie_reset(struct device *dev)
{
	struct brcmf_bus *bus_if = dev_get_drvdata(dev);
	struct brcmf_pciedev *buspub = bus_if->bus_priv.pcie;
	struct brcmf_pciedev_info *devinfo = buspub->devinfo;
	struct brcmf_fw_request *fwreq;
	int err;

	brcmf_pcie_intr_disable(devinfo);

	brcmf_pcie_bus_console_read(devinfo, true);

	brcmf_detach(dev);

	brcmf_pcie_release_irq(devinfo);
	brcmf_pcie_release_scratchbuffers(devinfo);
	brcmf_pcie_release_ringbuffers(devinfo);
	brcmf_pcie_reset_device(devinfo);

	fwreq = brcmf_pcie_prepare_fw_request(devinfo);
	if (!fwreq) {
		dev_err(dev, "Failed to prepare FW request\n");
		return -ENOMEM;
	}

	err = brcmf_fw_get_firmwares(dev, fwreq, brcmf_pcie_setup);
	if (err) {
		dev_err(dev, "Failed to prepare FW request\n");
		kfree(fwreq);
	}

	return err;
}

static const struct brcmf_bus_ops brcmf_pcie_bus_ops = {
	.txdata = brcmf_pcie_tx,
	.stop = brcmf_pcie_down,
	.txctl = brcmf_pcie_tx_ctlpkt,
	.rxctl = brcmf_pcie_rx_ctlpkt,
	.wowl_config = brcmf_pcie_wowl_config,
	.get_ramsize = brcmf_pcie_get_ramsize,
	.get_memdump = brcmf_pcie_get_memdump,
	.get_fwname = brcmf_pcie_get_fwname,
	.reset = brcmf_pcie_reset,
	.debugfs_create = brcmf_pcie_debugfs_create,
};


static void
brcmf_pcie_adjust_ramsize(struct brcmf_pciedev_info *devinfo, u8 *data,
			  u32 data_len)
{
	__le32 *field;
	u32 newsize;

	if (data_len < BRCMF_RAMSIZE_OFFSET + 8)
		return;

	field = (__le32 *)&data[BRCMF_RAMSIZE_OFFSET];
	if (le32_to_cpup(field) != BRCMF_RAMSIZE_MAGIC)
		return;
	field++;
	newsize = le32_to_cpup(field);

	brcmf_dbg(PCIE, "Found ramsize info in FW, adjusting to 0x%x\n",
		  newsize);
	devinfo->ci->ramsize = newsize;
}


static void
brcmf_pcie_write_rand(struct brcmf_pciedev_info *devinfo, u32 nvram_csm)
{
	struct brcmf_rand_metadata rand_data;
	u8 rand_buf[BRCMF_ENTROPY_HOST_LEN];
	u32 count = BRCMF_ENTROPY_HOST_LEN;
	u32 address;

	address = devinfo->ci->rambase +
		  (devinfo->ci->ramsize - BRCMF_NVRAM_OFFSET_TCM) -
		  ((nvram_csm & 0xffff) * BRCMF_NVRAM_COMPRS_FACTOR) -
		  sizeof(rand_data);
	memset(rand_buf, 0, BRCMF_ENTROPY_HOST_LEN);
	rand_data.signature = cpu_to_le32(BRCMF_NVRAM_RNG_SIGNATURE);
	rand_data.count = cpu_to_le32(count);
	brcmf_pcie_copy_mem_todev(devinfo, address, &rand_data,
				  sizeof(rand_data));
	address -= count;
	get_random_bytes(rand_buf, count);
	brcmf_pcie_copy_mem_todev(devinfo, address, rand_buf, count);
}

static int
brcmf_pcie_init_share_ram_info(struct brcmf_pciedev_info *devinfo,
			       u32 sharedram_addr)
{
	struct brcmf_bus *bus = dev_get_drvdata(&devinfo->pdev->dev);
	struct brcmf_pcie_shared_info *shared;
	u32 addr;
	u32 host_cap;

	shared = &devinfo->shared;
	shared->tcm_base_address = sharedram_addr;

	shared->flags = brcmf_pcie_read_tcm32(devinfo, sharedram_addr);
	shared->version = (u8)(shared->flags & BRCMF_PCIE_SHARED_VERSION_MASK);
	brcmf_dbg(PCIE, "PCIe protocol version %d\n", shared->version);
	if ((shared->version > BRCMF_PCIE_MAX_SHARED_VERSION) ||
	    (shared->version < BRCMF_PCIE_MIN_SHARED_VERSION)) {
		brcmf_err(bus, "Unsupported PCIE version %d\n",
			  shared->version);
		return -EINVAL;
	}

	/* check firmware support dma indicies */
	if (shared->flags & BRCMF_PCIE_SHARED_DMA_INDEX) {
		if (shared->flags & BRCMF_PCIE_SHARED_DMA_2B_IDX)
			devinfo->dma_idx_sz = sizeof(u16);
		else
			devinfo->dma_idx_sz = sizeof(u32);
	}

	addr = sharedram_addr + BRCMF_SHARED_MAX_RXBUFPOST_OFFSET;
	shared->max_rxbufpost = brcmf_pcie_read_tcm16(devinfo, addr);
	if (shared->max_rxbufpost == 0)
		shared->max_rxbufpost = BRCMF_DEF_MAX_RXBUFPOST;

	addr = sharedram_addr + BRCMF_SHARED_RX_DATAOFFSET_OFFSET;
	shared->rx_dataoffset = brcmf_pcie_read_tcm32(devinfo, addr);

	addr = sharedram_addr + BRCMF_SHARED_HTOD_MB_DATA_ADDR_OFFSET;
	shared->htod_mb_data_addr = brcmf_pcie_read_tcm32(devinfo, addr);

	addr = sharedram_addr + BRCMF_SHARED_DTOH_MB_DATA_ADDR_OFFSET;
	shared->dtoh_mb_data_addr = brcmf_pcie_read_tcm32(devinfo, addr);

	addr = sharedram_addr + BRCMF_SHARED_RING_INFO_ADDR_OFFSET;
	shared->ring_info_addr = brcmf_pcie_read_tcm32(devinfo, addr);

	if (shared->version >= BRCMF_PCIE_SHARED_VERSION_6) {
		host_cap = shared->version;

		/* Disable OOB Device Wake based DeepSleep State Machine */
		host_cap |= BRCMF_HOSTCAP_DS_NO_OOB_DW;

		devinfo->hostready =
			((shared->flags & BRCMF_PCIE_SHARED_HOSTRDY_DB1)
			 == BRCMF_PCIE_SHARED_HOSTRDY_DB1);
		if (devinfo->hostready) {
			brcmf_dbg(PCIE, "HostReady supported by dongle.\n");
			host_cap |= BRCMF_HOSTCAP_H2D_ENABLE_HOSTRDY;
		}
		devinfo->use_mailbox =
			((shared->flags & BRCMF_PCIE_SHARED_USE_MAILBOX)
			 == BRCMF_PCIE_SHARED_USE_MAILBOX);
		devinfo->use_d0_inform = false;
		addr = sharedram_addr + BRCMF_SHARED_HOST_CAP_OFFSET;

		brcmf_pcie_write_tcm32(devinfo, addr, host_cap);
	} else {
		devinfo->use_d0_inform = true;
	}

	brcmf_dbg(PCIE, "max rx buf post %d, rx dataoffset %d\n",
		  shared->max_rxbufpost, shared->rx_dataoffset);

	brcmf_pcie_bus_console_init(devinfo);

	return 0;
}


static int brcmf_pcie_download_fw_nvram(struct brcmf_pciedev_info *devinfo,
					const struct firmware *fw, void *nvram,
					u32 nvram_len)
{
	struct brcmf_bus *bus = dev_get_drvdata(&devinfo->pdev->dev);
	struct trx_header_le *trx = (struct trx_header_le *)fw->data;
	u32 fw_size;
	u32 sharedram_addr;
	u32 sharedram_addr_written;
	u32 loop_counter;
	int err;
	u32 address;
	u32 resetintr;
	u32 nvram_lenw;
	u32 nvram_csm;

	brcmf_dbg(PCIE, "Halt ARM.\n");
	err = brcmf_pcie_enter_download_state(devinfo);
	if (err)
		return err;

	brcmf_dbg(PCIE, "Download FW %s\n", devinfo->fw_name);
	address = devinfo->ci->rambase;
	fw_size = fw->size;
	if (trx->magic == cpu_to_le32(TRX_MAGIC)) {
		address -= sizeof(struct trx_header_le);
		fw_size = le32_to_cpu(trx->len);
	}
	brcmf_pcie_copy_mem_todev(devinfo, address, (void *)fw->data, fw_size);

	resetintr = get_unaligned_le32(fw->data);
	release_firmware(fw);

	if (devinfo->ci->blhs) {
		brcmf_pcie_bus_console_read(devinfo, false);
		err = devinfo->ci->blhs->post_fwdl(devinfo->ci);
		if (err) {
			brcmf_err(bus, "FW download failed, err=%d\n", err);
			return err;
		}

		err = devinfo->ci->blhs->chk_validation(devinfo->ci);
		if (err) {
			brcmf_err(bus, "FW valication failed, err=%d\n", err);
			return err;
		}
	} else {
		/* reset last 4 bytes of RAM address. to be used for shared
		 * area. This identifies when FW is running
		 */
		brcmf_pcie_write_ram32(devinfo, devinfo->ci->ramsize - 4, 0);
	}

	if (nvram) {
		brcmf_dbg(PCIE, "Download NVRAM %s\n", devinfo->nvram_name);
		address = devinfo->ci->rambase + devinfo->ci->ramsize -
			  nvram_len;
		if (devinfo->ci->blhs)
			address -= 4;
		brcmf_pcie_copy_mem_todev(devinfo, address, nvram, nvram_len);

		/* Convert nvram_len to words to determine the length token */
		nvram_lenw = nvram_len / 4;
		nvram_csm = (~nvram_lenw << 16) | (nvram_lenw & 0x0000FFFF);
		brcmf_fw_nvram_free(nvram);
	} else {
		nvram_csm = 0;
		brcmf_dbg(PCIE, "No matching NVRAM file found %s\n",
			  devinfo->nvram_name);
	}

	if (devinfo->ci->chip == CY_CC_55572_CHIP_ID) {
		/* Write the length token to the last word of RAM address */
		brcmf_pcie_write_ram32(devinfo, devinfo->ci->ramsize - 4,
				       cpu_to_le32(nvram_csm));

		/* Write random numbers to TCM for randomizing heap address */
		brcmf_pcie_write_rand(devinfo, nvram_csm);
	}

	sharedram_addr_written = brcmf_pcie_read_ram32(devinfo,
						       devinfo->ci->ramsize -
						       4);
	brcmf_dbg(PCIE, "Bring ARM in running state\n");
	err = brcmf_pcie_exit_download_state(devinfo, resetintr);
	if (err)
		return err;

	if (!brcmf_pcie_bus_readshared(devinfo, nvram_csm))
		brcmf_pcie_bus_console_read(devinfo, false);

	brcmf_dbg(PCIE, "Wait for FW init\n");
	sharedram_addr = sharedram_addr_written;
	loop_counter = BRCMF_PCIE_FW_UP_TIMEOUT / 50;
	while ((sharedram_addr == sharedram_addr_written) && (loop_counter)) {
		msleep(50);
		sharedram_addr = brcmf_pcie_read_ram32(devinfo,
						       devinfo->ci->ramsize -
						       4);
		loop_counter--;
	}
	if (sharedram_addr == sharedram_addr_written) {
		brcmf_err(bus, "FW failed to initialize\n");
		return -ENODEV;
	}
	if (sharedram_addr < devinfo->ci->rambase ||
	    sharedram_addr >= devinfo->ci->rambase + devinfo->ci->ramsize) {
		brcmf_err(bus, "Invalid shared RAM address 0x%08x\n",
			  sharedram_addr);
		return -ENODEV;
	}
	brcmf_dbg(PCIE, "Shared RAM addr: 0x%08x\n", sharedram_addr);

	return (brcmf_pcie_init_share_ram_info(devinfo, sharedram_addr));
}


static int brcmf_pcie_get_resource(struct brcmf_pciedev_info *devinfo)
{
	struct pci_dev *pdev = devinfo->pdev;
	struct brcmf_bus *bus = dev_get_drvdata(&pdev->dev);
	int err;
	phys_addr_t  bar0_addr, bar1_addr;
	ulong bar1_size;

	err = pci_enable_device(pdev);
	if (err) {
		brcmf_err(bus, "pci_enable_device failed err=%d\n", err);
		return err;
	}

	pci_set_master(pdev);

	/* Bar-0 mapped address */
	bar0_addr = pci_resource_start(pdev, 0);
	/* Bar-1 mapped address */
	bar1_addr = pci_resource_start(pdev, 2);
	/* read Bar-1 mapped memory range */
	bar1_size = pci_resource_len(pdev, 2);
	if ((bar1_size == 0) || (bar1_addr == 0)) {
		brcmf_err(bus, "BAR1 Not enabled, device size=%ld, addr=%#016llx\n",
			  bar1_size, (unsigned long long)bar1_addr);
		return -EINVAL;
	}

	devinfo->regs = ioremap(bar0_addr, BRCMF_PCIE_REG_MAP_SIZE);
	devinfo->tcm = ioremap(bar1_addr, bar1_size);
#ifdef CONFIG_BRCMFMAC_PCIE_BARWIN_SZ
	devinfo->bar1_size = bar1_size;
#endif /* CONFIG_BRCMFMAC_PCIE_BARWIN_SZ */

	if (!devinfo->regs || !devinfo->tcm) {
		brcmf_err(bus, "ioremap() failed (%p,%p)\n", devinfo->regs,
			  devinfo->tcm);
		return -EINVAL;
	}
	brcmf_dbg(PCIE, "Phys addr : reg space = %p base addr %#016llx\n",
		  devinfo->regs, (unsigned long long)bar0_addr);
	brcmf_dbg(PCIE, "Phys addr : mem space = %p base addr %#016llx size 0x%x\n",
		  devinfo->tcm, (unsigned long long)bar1_addr,
		  (unsigned int)bar1_size);

	return 0;
}


static void brcmf_pcie_release_resource(struct brcmf_pciedev_info *devinfo)
{
	if (devinfo->tcm)
		iounmap(devinfo->tcm);
	if (devinfo->regs)
		iounmap(devinfo->regs);

	pci_disable_device(devinfo->pdev);
}

static u32 brcmf_pcie_buscore_blhs_read(void *ctx, u32 reg_offset)
{
	struct brcmf_pciedev_info *devinfo = (struct brcmf_pciedev_info *)ctx;

	brcmf_pcie_select_core(devinfo, BCMA_CORE_PCIE2);
	return brcmf_pcie_read_reg32(devinfo, reg_offset);
}

static void brcmf_pcie_buscore_blhs_write(void *ctx, u32 reg_offset, u32 value)
{
	struct brcmf_pciedev_info *devinfo = (struct brcmf_pciedev_info *)ctx;

	brcmf_pcie_select_core(devinfo, BCMA_CORE_PCIE2);
	brcmf_pcie_write_reg32(devinfo, reg_offset, value);
}

static u32 brcmf_pcie_buscore_prep_addr(const struct pci_dev *pdev, u32 addr)
{
	u32 ret_addr;

	ret_addr = addr & (BRCMF_PCIE_BAR0_REG_SIZE - 1);
	addr &= ~(BRCMF_PCIE_BAR0_REG_SIZE - 1);
	pci_write_config_dword(pdev, BRCMF_PCIE_BAR0_WINDOW, addr);

	return ret_addr;
}


static u32 brcmf_pcie_buscore_read32(void *ctx, u32 addr)
{
	struct brcmf_pciedev_info *devinfo = (struct brcmf_pciedev_info *)ctx;

	addr = brcmf_pcie_buscore_prep_addr(devinfo->pdev, addr);
	return brcmf_pcie_read_reg32(devinfo, addr);
}


static void brcmf_pcie_buscore_write32(void *ctx, u32 addr, u32 value)
{
	struct brcmf_pciedev_info *devinfo = (struct brcmf_pciedev_info *)ctx;

	addr = brcmf_pcie_buscore_prep_addr(devinfo->pdev, addr);
	brcmf_pcie_write_reg32(devinfo, addr, value);
}


static int brcmf_pcie_buscoreprep(void *ctx)
{
	return brcmf_pcie_get_resource(ctx);
}


static int brcmf_pcie_buscore_reset(void *ctx, struct brcmf_chip *chip)
{
	struct brcmf_pciedev_info *devinfo = (struct brcmf_pciedev_info *)ctx;
	u32 val;

	devinfo->ci = chip;
	brcmf_pcie_reset_device(devinfo);

	val = brcmf_pcie_read_reg32(devinfo,
				    BRCMF_PCIE_PCIE2REG_MAILBOXINT(devinfo));
	if (val != 0xffffffff)
		brcmf_pcie_write_reg32(devinfo,
				       BRCMF_PCIE_PCIE2REG_MAILBOXINT(devinfo),
				       val);

	return 0;
}


static void brcmf_pcie_buscore_activate(void *ctx, struct brcmf_chip *chip,
					u32 rstvec)
{
	struct brcmf_pciedev_info *devinfo = (struct brcmf_pciedev_info *)ctx;

	brcmf_pcie_write_tcm32(devinfo, 0, rstvec);
}


static int brcmf_pcie_buscore_blhs_attach(void *ctx, struct brcmf_blhs **blhs,
					  u32 flag, uint timeout, uint interval)
{
	struct brcmf_pciedev_info *devinfo = (struct brcmf_pciedev_info *)ctx;
	struct brcmf_bus *bus = dev_get_drvdata(&devinfo->pdev->dev);
	struct brcmf_blhs *blhsh;
	u32 regdata;
	u32 pcie_enum;
	u32 addr;

	if (devinfo->pdev->vendor != CY_PCIE_VENDOR_ID_CYPRESS)
		return 0;

	pci_read_config_dword(devinfo->pdev, BRCMF_PCIE_CFGREG_REVID, &regdata);
	if (regdata & BRCMF_PCIE_CFGREG_REVID_SECURE_MODE) {
		blhsh = kzalloc(sizeof(*blhsh), GFP_KERNEL);
		if (!blhsh)
			return -ENOMEM;

		blhsh->d2h = BRCMF_PCIE_PCIE2REG_DAR_D2H_MSG_0;
		blhsh->h2d = BRCMF_PCIE_PCIE2REG_DAR_H2D_MSG_0;
		blhsh->read = brcmf_pcie_buscore_blhs_read;
		blhsh->write = brcmf_pcie_buscore_blhs_write;

		/* Host indication for bootloarder to start the init */
		if (devinfo->pdev->device == CY_PCIE_55572_DEVICE_ID)
			pcie_enum = BRCMF_CYW55572_PCIE_BAR0_PCIE_ENUM_OFFSET;
		else
			pcie_enum = BRCMF_PCIE_BAR0_PCIE_ENUM_OFFSET;

		pci_read_config_dword(devinfo->pdev, PCI_BASE_ADDRESS_0,
				      &regdata);
		addr = regdata + pcie_enum + blhsh->h2d;
		brcmf_pcie_buscore_write32(ctx, addr, 0);

		addr = regdata + pcie_enum + blhsh->d2h;
		SPINWAIT_MS((brcmf_pcie_buscore_read32(ctx, addr) & flag) == 0,
			    timeout, interval);
		regdata = brcmf_pcie_buscore_read32(ctx, addr);
		if (!(regdata & flag)) {
			brcmf_err(bus, "Timeout waiting for bootloader ready\n");
			kfree(blhsh);
			return -EPERM;
		}
		*blhs = blhsh;
	}

	return 0;
}

static const struct brcmf_buscore_ops brcmf_pcie_buscore_ops = {
	.prepare = brcmf_pcie_buscoreprep,
	.reset = brcmf_pcie_buscore_reset,
	.activate = brcmf_pcie_buscore_activate,
	.read32 = brcmf_pcie_buscore_read32,
	.write32 = brcmf_pcie_buscore_write32,
	.blhs_attach = brcmf_pcie_buscore_blhs_attach,
};

#define BRCMF_PCIE_FW_CODE	0
#define BRCMF_PCIE_FW_NVRAM	1

static void brcmf_pcie_setup(struct device *dev, int ret,
			     struct brcmf_fw_request *fwreq)
{
	const struct firmware *fw;
	void *nvram;
	struct brcmf_bus *bus;
	struct brcmf_pciedev *pcie_bus_dev;
	struct brcmf_pciedev_info *devinfo;
	struct brcmf_commonring **flowrings;
	u32 i, nvram_len;

	bus = dev_get_drvdata(dev);
	pcie_bus_dev = bus->bus_priv.pcie;
	devinfo = pcie_bus_dev->devinfo;

	/* check firmware loading result */
	if (ret)
		goto fail;

	brcmf_pcie_attach(devinfo);

	fw = fwreq->items[BRCMF_PCIE_FW_CODE].binary;
	nvram = fwreq->items[BRCMF_PCIE_FW_NVRAM].nv_data.data;
	nvram_len = fwreq->items[BRCMF_PCIE_FW_NVRAM].nv_data.len;
	kfree(fwreq);

	ret = brcmf_chip_get_raminfo(devinfo->ci);
	if (ret) {
		brcmf_err(bus, "Failed to get RAM info\n");
		goto fail;
	}

	/* Some of the firmwares have the size of the memory of the device
	 * defined inside the firmware. This is because part of the memory in
	 * the device is shared and the devision is determined by FW. Parse
	 * the firmware and adjust the chip memory size now.
	 */
	brcmf_pcie_adjust_ramsize(devinfo, (u8 *)fw->data, fw->size);

	ret = brcmf_pcie_download_fw_nvram(devinfo, fw, nvram, nvram_len);
	if (ret) {
		if (devinfo->ci->blhs && !brcmf_pcie_bus_readshared(devinfo, 0))
			brcmf_pcie_bus_console_read(devinfo, true);
		goto fail;
	}

	devinfo->state = BRCMFMAC_PCIE_STATE_UP;

	ret = brcmf_pcie_init_ringbuffers(devinfo);
	if (ret)
		goto fail;

	ret = brcmf_pcie_init_scratchbuffers(devinfo);
	if (ret)
		goto fail;

	brcmf_pcie_select_core(devinfo, BCMA_CORE_PCIE2);
	ret = brcmf_pcie_request_irq(devinfo);
	if (ret)
		goto fail;

	/* hook the commonrings in the bus structure. */
	for (i = 0; i < BRCMF_NROF_COMMON_MSGRINGS; i++)
		bus->msgbuf->commonrings[i] =
				&devinfo->shared.commonrings[i]->commonring;

	flowrings = kcalloc(devinfo->shared.max_flowrings, sizeof(*flowrings),
			    GFP_KERNEL);
	if (!flowrings)
		goto fail;

	for (i = 0; i < devinfo->shared.max_flowrings; i++)
		flowrings[i] = &devinfo->shared.flowrings[i].commonring;
	bus->msgbuf->flowrings = flowrings;

	bus->msgbuf->rx_dataoffset = devinfo->shared.rx_dataoffset;
	bus->msgbuf->max_rxbufpost = devinfo->shared.max_rxbufpost;
	bus->msgbuf->max_flowrings = devinfo->shared.max_flowrings;

	init_waitqueue_head(&devinfo->mbdata_resp_wait);

	brcmf_pcie_intr_enable(devinfo);
	brcmf_pcie_hostready(devinfo);

	ret = brcmf_attach(&devinfo->pdev->dev, true);
	if (ret)
		goto fail;

	brcmf_pcie_bus_console_read(devinfo, false);

	brcmf_pcie_fwcon_timer(devinfo, true);

	return;

fail:
	brcmf_err(bus, "Dongle setup failed\n");
	brcmf_pcie_bus_console_read(devinfo, true);
	brcmf_fw_crashed(dev);
	device_release_driver(dev);
}

static struct brcmf_fw_request *
brcmf_pcie_prepare_fw_request(struct brcmf_pciedev_info *devinfo)
{
	struct brcmf_fw_request *fwreq;
	struct brcmf_fw_name fwnames[] = {
		{ ".bin", devinfo->fw_name },
		{ ".txt", devinfo->nvram_name },
	};
	u32 chip;

	if (devinfo->ci->blhs)
		fwnames[BRCMF_PCIE_FW_CODE].extension = ".trxse";

	chip = devinfo->ci->chip;
	fwreq = brcmf_fw_alloc_request(chip, devinfo->ci->chiprev,
				       brcmf_pcie_fwnames,
				       ARRAY_SIZE(brcmf_pcie_fwnames),
				       fwnames, ARRAY_SIZE(fwnames));
	if (!fwreq)
		return NULL;

	if (devinfo->ci->blhs)
		fwreq->items[BRCMF_PCIE_FW_CODE].type = BRCMF_FW_TYPE_TRXSE;
	else
		fwreq->items[BRCMF_PCIE_FW_CODE].type = BRCMF_FW_TYPE_BINARY;
	fwreq->items[BRCMF_PCIE_FW_NVRAM].type = BRCMF_FW_TYPE_NVRAM;
	fwreq->items[BRCMF_PCIE_FW_NVRAM].flags = BRCMF_FW_REQF_OPTIONAL;
	fwreq->board_type = devinfo->settings->board_type;
	/* NVRAM reserves PCI domain 0 for Broadcom's SDK faked bus */
	fwreq->domain_nr = pci_domain_nr(devinfo->pdev->bus) + 1;
	fwreq->bus_nr = devinfo->pdev->bus->number;

	return fwreq;
}

#ifdef DEBUG
static void
brcmf_pcie_fwcon_timer(struct brcmf_pciedev_info *devinfo, bool active)
{
	if (!active) {
		if (devinfo->console_active) {
			del_timer_sync(&devinfo->timer);
			devinfo->console_active = false;
		}
		return;
	}

	/* don't start the timer */
	if (devinfo->state != BRCMFMAC_PCIE_STATE_UP ||
	    !devinfo->console_interval || !BRCMF_FWCON_ON())
		return;

	if (!devinfo->console_active) {
		devinfo->timer.expires = jiffies + devinfo->console_interval;
		add_timer(&devinfo->timer);
		devinfo->console_active = true;
	} else {
		/* Reschedule the timer */
		mod_timer(&devinfo->timer, jiffies + devinfo->console_interval);
	}
}

static void
brcmf_pcie_fwcon(struct timer_list *t)
{
	struct brcmf_pciedev_info *devinfo = from_timer(devinfo, t, timer);

	if (!devinfo->console_active)
		return;

	brcmf_pcie_bus_console_read(devinfo, false);

	/* Reschedule the timer if console interval is not zero */
	mod_timer(&devinfo->timer, jiffies + devinfo->console_interval);
}

static int brcmf_pcie_console_interval_get(void *data, u64 *val)
{
	struct brcmf_pciedev_info *devinfo = data;

	*val = devinfo->console_interval;

	return 0;
}

static int brcmf_pcie_console_interval_set(void *data, u64 val)
{
	struct brcmf_pciedev_info *devinfo = data;

	if (val > MAX_CONSOLE_INTERVAL)
		return -EINVAL;

	devinfo->console_interval = val;

	if (!val && devinfo->console_active)
		brcmf_pcie_fwcon_timer(devinfo, false);
	else if (val)
		brcmf_pcie_fwcon_timer(devinfo, true);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(brcmf_pcie_console_interval_fops,
			brcmf_pcie_console_interval_get,
			brcmf_pcie_console_interval_set,
			"%llu\n");

static void brcmf_pcie_debugfs_create(struct device *dev)
{
	struct brcmf_bus *bus_if = dev_get_drvdata(dev);
	struct brcmf_pub *drvr = bus_if->drvr;
	struct brcmf_pciedev *pcie_bus_dev = bus_if->bus_priv.pcie;
	struct brcmf_pciedev_info *devinfo = pcie_bus_dev->devinfo;
	struct dentry *dentry = brcmf_debugfs_get_devdir(drvr);

	if (IS_ERR_OR_NULL(dentry))
		return;

	devinfo->console_interval = BRCMF_CONSOLE;

	debugfs_create_file("console_interval", 0644, dentry, devinfo,
			    &brcmf_pcie_console_interval_fops);
}

#else
void brcmf_pcie_fwcon_timer(struct brcmf_pciedev_info *devinfo, bool active)
{
}

static void brcmf_pcie_debugfs_create(struct device *dev)
{
}
#endif

static int
brcmf_pcie_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret;
	struct brcmf_fw_request *fwreq;
	struct brcmf_pciedev_info *devinfo;
	struct brcmf_pciedev *pcie_bus_dev;
	struct brcmf_bus *bus;

	brcmf_dbg(PCIE, "Enter %x:%x\n", pdev->vendor, pdev->device);

	ret = -ENOMEM;
	devinfo = kzalloc(sizeof(*devinfo), GFP_KERNEL);
	if (devinfo == NULL)
		return ret;

	devinfo->pdev = pdev;
	pcie_bus_dev = NULL;
	devinfo->ci = brcmf_chip_attach(devinfo, &brcmf_pcie_buscore_ops);
	if (IS_ERR(devinfo->ci)) {
		ret = PTR_ERR(devinfo->ci);
		devinfo->ci = NULL;
		goto fail;
	}

	pcie_bus_dev = kzalloc(sizeof(*pcie_bus_dev), GFP_KERNEL);
	if (pcie_bus_dev == NULL) {
		ret = -ENOMEM;
		goto fail;
	}

	devinfo->settings = brcmf_get_module_param(&devinfo->pdev->dev,
						   BRCMF_BUSTYPE_PCIE,
						   devinfo->ci->chip,
						   devinfo->ci->chiprev);
	if (!devinfo->settings) {
		ret = -ENOMEM;
		goto fail;
	}

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		ret = -ENOMEM;
		goto fail;
	}
	bus->msgbuf = kzalloc(sizeof(*bus->msgbuf), GFP_KERNEL);
	if (!bus->msgbuf) {
		ret = -ENOMEM;
		kfree(bus);
		goto fail;
	}

	/* hook it all together. */
	pcie_bus_dev->devinfo = devinfo;
	pcie_bus_dev->bus = bus;
	bus->dev = &pdev->dev;
	bus->bus_priv.pcie = pcie_bus_dev;
	bus->ops = &brcmf_pcie_bus_ops;
	bus->proto_type = BRCMF_PROTO_MSGBUF;
	bus->chip = devinfo->coreid;
	bus->wowl_supported = pci_pme_capable(pdev, PCI_D3hot);
	dev_set_drvdata(&pdev->dev, bus);

	ret = brcmf_alloc(&devinfo->pdev->dev, devinfo->settings);
	if (ret)
		goto fail_bus;

#ifdef DEBUG
	/* Set up the fwcon timer */
	timer_setup(&devinfo->timer, brcmf_pcie_fwcon, 0);
#endif
	fwreq = brcmf_pcie_prepare_fw_request(devinfo);
	if (!fwreq) {
		ret = -ENOMEM;
		goto fail_brcmf;
	}

	ret = brcmf_fw_get_firmwares(bus->dev, fwreq, brcmf_pcie_setup);
	if (ret < 0) {
		kfree(fwreq);
		goto fail_brcmf;
	}
	return 0;

fail_brcmf:
	brcmf_free(&devinfo->pdev->dev);
fail_bus:
	kfree(bus->msgbuf);
	kfree(bus);
fail:
	brcmf_err(NULL, "failed %x:%x\n", pdev->vendor, pdev->device);
	brcmf_pcie_release_resource(devinfo);
	if (devinfo->ci)
		brcmf_chip_detach(devinfo->ci);
	if (devinfo->settings)
		brcmf_release_module_param(devinfo->settings);
	kfree(pcie_bus_dev);
	kfree(devinfo);
	return ret;
}


static void
brcmf_pcie_remove(struct pci_dev *pdev)
{
	struct brcmf_pciedev_info *devinfo;
	struct brcmf_bus *bus;

	brcmf_dbg(PCIE, "Enter\n");

	bus = dev_get_drvdata(&pdev->dev);
	if (bus == NULL)
		return;

	devinfo = bus->bus_priv.pcie->devinfo;

	brcmf_pcie_fwcon_timer(devinfo, false);

	devinfo->state = BRCMFMAC_PCIE_STATE_DOWN;
	if (devinfo->ci)
		brcmf_pcie_intr_disable(devinfo);

	brcmf_detach(&pdev->dev);
	brcmf_free(&pdev->dev);

	kfree(bus->bus_priv.pcie);
	kfree(bus->msgbuf->flowrings);
	kfree(bus->msgbuf);
	kfree(bus);

	brcmf_pcie_release_irq(devinfo);
	brcmf_pcie_release_scratchbuffers(devinfo);
	brcmf_pcie_release_ringbuffers(devinfo);
	brcmf_pcie_reset_device(devinfo);
	brcmf_pcie_release_resource(devinfo);

	if (devinfo->ci)
		brcmf_chip_detach(devinfo->ci);
	if (devinfo->settings)
		brcmf_release_module_param(devinfo->settings);

	kfree(devinfo);
	dev_set_drvdata(&pdev->dev, NULL);
}


#ifdef CONFIG_PM


static int brcmf_pcie_pm_enter_D3(struct device *dev)
{
	struct brcmf_pciedev_info *devinfo;
	struct brcmf_bus *bus;
	struct brcmf_cfg80211_info *config;
	int retry = BRCMF_PM_WAIT_MAXRETRY;

	brcmf_dbg(PCIE, "Enter\n");

	bus = dev_get_drvdata(dev);
	devinfo = bus->bus_priv.pcie->devinfo;
	config = bus->drvr->config;

	while (retry &&
	       config->pm_state == BRCMF_CFG80211_PM_STATE_SUSPENDING) {
		usleep_range(10000, 20000);
		retry--;
	}
	if (!retry && config->pm_state == BRCMF_CFG80211_PM_STATE_SUSPENDING)
		brcmf_err(bus, "timed out wait for cfg80211 suspended\n");

	brcmf_pcie_fwcon_timer(devinfo, false);

	brcmf_bus_change_state(bus, BRCMF_BUS_DOWN);

	devinfo->mbdata_completed = false;
	brcmf_pcie_send_mb_data(devinfo, BRCMF_H2D_HOST_D3_INFORM);

	wait_event_timeout(devinfo->mbdata_resp_wait, devinfo->mbdata_completed,
			   BRCMF_PCIE_MBDATA_TIMEOUT);
	if (!devinfo->mbdata_completed) {
		brcmf_err(bus, "Timeout on response for entering D3 substate\n");
		brcmf_bus_change_state(bus, BRCMF_BUS_UP);
		return -EIO;
	}

	devinfo->state = BRCMFMAC_PCIE_STATE_DOWN;

	return 0;
}


static int brcmf_pcie_pm_leave_D3(struct device *dev)
{
	struct brcmf_pciedev_info *devinfo;
	struct brcmf_bus *bus;
	struct pci_dev *pdev;
	int err;

	brcmf_dbg(PCIE, "Enter\n");

	bus = dev_get_drvdata(dev);
	devinfo = bus->bus_priv.pcie->devinfo;
	brcmf_dbg(PCIE, "Enter, dev=%p, bus=%p\n", dev, bus);

	/* Check if device is still up and running, if so we are ready */
	if (brcmf_pcie_read_reg32(devinfo, BRCMF_PCIE_PCIE2REG_INTMASK) != 0) {
		brcmf_dbg(PCIE, "Try to wakeup device....\n");
		if (devinfo->use_d0_inform) {
			if (brcmf_pcie_send_mb_data(devinfo,
						    BRCMF_H2D_HOST_D0_INFORM))
				goto cleanup;
		} else {
			brcmf_pcie_hostready(devinfo);
		}

		brcmf_dbg(PCIE, "Hot resume, continue....\n");
		devinfo->state = BRCMFMAC_PCIE_STATE_UP;
		brcmf_pcie_select_core(devinfo, BCMA_CORE_PCIE2);
		brcmf_bus_change_state(bus, BRCMF_BUS_UP);
		brcmf_pcie_intr_enable(devinfo);
		if (devinfo->use_d0_inform) {
			brcmf_dbg(TRACE, "sending brcmf_pcie_hostready since use_d0_inform=%d\n",
				  devinfo->use_d0_inform);
			brcmf_pcie_hostready(devinfo);
		}

		brcmf_pcie_fwcon_timer(devinfo, true);
		return 0;
	}

cleanup:
	brcmf_chip_detach(devinfo->ci);
	devinfo->ci = NULL;
	pdev = devinfo->pdev;
	brcmf_pcie_remove(pdev);

	err = brcmf_pcie_probe(pdev, NULL);
	if (err)
		brcmf_err(bus, "probe after resume failed, err=%d\n", err);

	return err;
}


static const struct dev_pm_ops brcmf_pciedrvr_pm = {
	.suspend = brcmf_pcie_pm_enter_D3,
	.resume = brcmf_pcie_pm_leave_D3,
	.freeze = brcmf_pcie_pm_enter_D3,
	.restore = brcmf_pcie_pm_leave_D3,
};


#endif /* CONFIG_PM */


#define BRCMF_PCIE_DEVICE(dev_id)	{ BRCM_PCIE_VENDOR_ID_BROADCOM, dev_id,\
	PCI_ANY_ID, PCI_ANY_ID, PCI_CLASS_NETWORK_OTHER << 8, 0xffff00, 0 }
#define BRCMF_PCIE_DEVICE_SUB(dev_id, subvend, subdev)	{ \
	BRCM_PCIE_VENDOR_ID_BROADCOM, dev_id,\
	subvend, subdev, PCI_CLASS_NETWORK_OTHER << 8, 0xffff00, 0 }

#define BRCMF_PCIE_DEVICE_CY(dev_id)	{ CY_PCIE_VENDOR_ID_CYPRESS, dev_id,\
	PCI_ANY_ID, PCI_ANY_ID, PCI_CLASS_NETWORK_OTHER << 8, 0xffff00, 0 }

static const struct pci_device_id brcmf_pcie_devid_table[] = {
	BRCMF_PCIE_DEVICE(BRCM_PCIE_4350_DEVICE_ID),
	BRCMF_PCIE_DEVICE_SUB(0x4355, BRCM_PCIE_VENDOR_ID_BROADCOM, 0x4355),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_4354_RAW_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_4356_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_43567_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_43570_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_43570_RAW_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_4358_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_4359_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_43602_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_43602_2G_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_43602_5G_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_43602_RAW_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_4364_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_4365_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_4365_2G_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_4365_5G_DEVICE_ID),
	BRCMF_PCIE_DEVICE_SUB(0x4365, BRCM_PCIE_VENDOR_ID_BROADCOM, 0x4365),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_4366_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_4366_2G_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_4366_5G_DEVICE_ID),
	BRCMF_PCIE_DEVICE(BRCM_PCIE_4371_DEVICE_ID),
	BRCMF_PCIE_DEVICE(CY_PCIE_89459_DEVICE_ID),
	BRCMF_PCIE_DEVICE(CY_PCIE_89459_RAW_DEVICE_ID),
	BRCMF_PCIE_DEVICE(CY_PCIE_54591_DEVICE_ID),
	BRCMF_PCIE_DEVICE(CY_PCIE_54590_DEVICE_ID),
	BRCMF_PCIE_DEVICE(CY_PCIE_54594_DEVICE_ID),
	BRCMF_PCIE_DEVICE_CY(CY_PCIE_55572_DEVICE_ID),
	BRCMF_PCIE_DEVICE(CY_PCIE_4373_RAW_DEVICE_ID),
	BRCMF_PCIE_DEVICE(CY_PCIE_4373_DUAL_DEVICE_ID),
	BRCMF_PCIE_DEVICE(CY_PCIE_4373_2G_DEVICE_ID),
	BRCMF_PCIE_DEVICE(CY_PCIE_4373_5G_DEVICE_ID),
	{ /* end: all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, brcmf_pcie_devid_table);

static struct pci_driver brcmf_pciedrvr = {
	.node = {},
	.name = KBUILD_MODNAME,
	.id_table = brcmf_pcie_devid_table,
	.probe = brcmf_pcie_probe,
	.remove = brcmf_pcie_remove,
#ifdef CONFIG_PM
	.driver.pm = &brcmf_pciedrvr_pm,
#endif
	.driver.coredump = brcmf_dev_coredump,
};

void brcmf_pcie_register(void)
{
	int err;

	brcmf_dbg(PCIE, "Enter\n");
	err = pci_register_driver(&brcmf_pciedrvr);
	if (err)
		brcmf_err(NULL, "PCIE driver registration failed, err=%d\n",
			  err);
}

void brcmf_pcie_exit(void)
{
	brcmf_dbg(PCIE, "Enter\n");
	pci_unregister_driver(&brcmf_pciedrvr);
}
