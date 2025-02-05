// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2010 Broadcom Corporation
 */

#ifndef	BRCMFMAC_SDIO_H
#define	BRCMFMAC_SDIO_H

#include <linux/skbuff.h>
#include <linux/firmware.h>
#include "firmware.h"

#define SDIOD_FBR_SIZE		0x100

/* io_en */
#define SDIO_FUNC_ENABLE_1	0x02
#define SDIO_FUNC_ENABLE_2	0x04

/* io_rdys */
#define SDIO_FUNC_READY_1	0x02
#define SDIO_FUNC_READY_2	0x04

/* intr_status */
#define INTR_STATUS_FUNC1	0x2
#define INTR_STATUS_FUNC2	0x4

/* mask of register map */
#define REG_F0_REG_MASK		0x7FF
#define REG_F1_MISC_MASK	0x1FFFF

#define BRCMF_SDIO_REG_DAR_H2D_MSG_0	0x10030
#define BRCMF_SDIO_REG_DAR_D2H_MSG_0	0x10038

/* function 0 vendor specific CCCR registers */

#define SDIO_CCCR_BRCM_CARDCAP			0xf0
#define SDIO_CCCR_BRCM_CARDCAP_CMD14_SUPPORT	BIT(1)
#define SDIO_CCCR_BRCM_CARDCAP_CMD14_EXT	BIT(2)
#define SDIO_CCCR_BRCM_CARDCAP_CMD_NODEC	BIT(3)
#define SDIO_CCCR_BRCM_CARDCAP_SECURE_MODE	BIT(7)

/* Interrupt enable bits for each function */
#define SDIO_CCCR_IEN_FUNC0			BIT(0)
#define SDIO_CCCR_IEN_FUNC1			BIT(1)
#define SDIO_CCCR_IEN_FUNC2			BIT(2)

#define SDIO_CCCR_BRCM_CARDCTRL			0xf1
#define SDIO_CCCR_BRCM_CARDCTRL_WLANRESET	BIT(1)

#define SDIO_CCCR_BRCM_SEPINT			0xf2
#define SDIO_CCCR_BRCM_SEPINT_MASK		BIT(0)
#define SDIO_CCCR_BRCM_SEPINT_OE		BIT(1)
#define SDIO_CCCR_BRCM_SEPINT_ACT_HI		BIT(2)

/* function 1 miscellaneous registers */

/* sprom command and status */
#define SBSDIO_SPROM_CS			0x10000
/* sprom info register */
#define SBSDIO_SPROM_INFO		0x10001
/* sprom indirect access data byte 0 */
#define SBSDIO_SPROM_DATA_LOW		0x10002
/* sprom indirect access data byte 1 */
#define SBSDIO_SPROM_DATA_HIGH		0x10003
/* sprom indirect access addr byte 0 */
#define SBSDIO_SPROM_ADDR_LOW		0x10004
/* gpio select */
#define SBSDIO_GPIO_SELECT		0x10005
/* gpio output */
#define SBSDIO_GPIO_OUT			0x10006
/* gpio enable */
#define SBSDIO_GPIO_EN			0x10007
/* rev < 7, watermark for sdio device TX path */
#define SBSDIO_WATERMARK		0x10008
/* control busy signal generation */
#define SBSDIO_DEVICE_CTL		0x10009

/* SB Address Window Low (b15) */
#define SBSDIO_FUNC1_SBADDRLOW		0x1000A
/* SB Address Window Mid (b23:b16) */
#define SBSDIO_FUNC1_SBADDRMID		0x1000B
/* SB Address Window High (b31:b24)    */
#define SBSDIO_FUNC1_SBADDRHIGH		0x1000C
/* Frame Control (frame term/abort) */
#define SBSDIO_FUNC1_FRAMECTRL		0x1000D
/* ChipClockCSR (ALP/HT ctl/status) */
#define SBSDIO_FUNC1_CHIPCLKCSR		0x1000E
/* SdioPullUp (on cmd, d0-d2) */
#define SBSDIO_FUNC1_SDIOPULLUP		0x1000F
/* Write Frame Byte Count Low */
#define SBSDIO_FUNC1_WFRAMEBCLO		0x10019
/* Write Frame Byte Count High */
#define SBSDIO_FUNC1_WFRAMEBCHI		0x1001A
/* Read Frame Byte Count Low */
#define SBSDIO_FUNC1_RFRAMEBCLO		0x1001B
/* Read Frame Byte Count High */
#define SBSDIO_FUNC1_RFRAMEBCHI		0x1001C
/* MesBusyCtl (rev 11) */
#define SBSDIO_FUNC1_MESBUSYCTRL	0x1001D
/* Watermark for sdio device RX path */
#define SBSDIO_MESBUSY_RXFIFO_WM_MASK	0x7F
#define SBSDIO_MESBUSY_RXFIFO_WM_SHIFT	0
/* Enable busy capability for MES access */
#define SBSDIO_MESBUSYCTRL_ENAB		0x80
#define SBSDIO_MESBUSYCTRL_ENAB_SHIFT	7

/* Sdio Core Rev 12 */
#define SBSDIO_FUNC1_WAKEUPCTRL		0x1001E
#define SBSDIO_FUNC1_WCTRL_ALPWAIT_MASK		0x1
#define SBSDIO_FUNC1_WCTRL_ALPWAIT_SHIFT	0
#define SBSDIO_FUNC1_WCTRL_HTWAIT_MASK		0x2
#define SBSDIO_FUNC1_WCTRL_HTWAIT_SHIFT		1
#define SBSDIO_FUNC1_SLEEPCSR		0x1001F
#define SBSDIO_FUNC1_SLEEPCSR_KSO_MASK		0x1
#define SBSDIO_FUNC1_SLEEPCSR_KSO_SHIFT		0
#define SBSDIO_FUNC1_SLEEPCSR_KSO_EN		1
#define SBSDIO_FUNC1_SLEEPCSR_DEVON_MASK	0x2
#define SBSDIO_FUNC1_SLEEPCSR_DEVON_SHIFT	1

#define SBSDIO_FUNC1_MISC_REG_START	0x10000	/* f1 misc register start */
#define SBSDIO_FUNC1_MISC_REG_LIMIT	0x1001F	/* f1 misc register end */

/* function 1 OCP space */

/* sb offset addr is <= 15 bits, 32k */
#define SBSDIO_SB_OFT_ADDR_MASK		0x07FFF
#define SBSDIO_SB_OFT_ADDR_LIMIT	0x08000
/* with b15, maps to 32-bit SB access */
#define SBSDIO_SB_ACCESS_2_4B_FLAG	0x08000

/* Address bits from SBADDR regs */
#define SBSDIO_SBWINDOW_MASK		0xffff8000

#define SDIOH_READ              0	/* Read request */
#define SDIOH_WRITE             1	/* Write request */

#define SDIOH_DATA_FIX          0	/* Fixed addressing */
#define SDIOH_DATA_INC          1	/* Incremental addressing */

/* internal return code */
#define SUCCESS	0
#define ERROR	1

/* Packet alignment for most efficient SDIO (can change based on platform) */
#define BRCMF_SDALIGN	(1 << 6)

/* watchdog polling interval */
#define BRCMF_WD_POLL	msecs_to_jiffies(10)

/**
 * enum brcmf_sdiod_state - the state of the bus.
 *
 * @BRCMF_SDIOD_DOWN: Device can be accessed, no DPC.
 * @BRCMF_SDIOD_DATA: Ready for data transfers, DPC enabled.
 * @BRCMF_SDIOD_NOMEDIUM: No medium access to dongle possible.
 */
enum brcmf_sdiod_state {
	BRCMF_SDIOD_DOWN,
	BRCMF_SDIOD_DATA,
	BRCMF_SDIOD_NOMEDIUM
};

struct brcmf_sdreg {
	int func;
	int offset;
	int value;
};

struct brcmf_sdio;
struct brcmf_sdiod_freezer;

/* ULP SHM Offsets info */
struct ulp_shm_info {
	u32 m_ulp_ctrl_sdio;
	u32 m_ulp_wakeevt_ind;
	u32 m_ulp_wakeind;
	u32 m_ulp_phytxblk;
};

/* FMAC ULP state machine */
#define FMAC_ULP_IDLE		(0)
#define FMAC_ULP_ENTRY_RECV		(1)
#define FMAC_ULP_TRIGGERED		(2)

/* BRCMF_E_ULP event data */
#define FMAC_ULP_EVENT_VERSION		1
#define FMAC_ULP_DISABLE_CONSOLE		1 /* Disable console */
#define FMAC_ULP_UCODE_DOWNLOAD		2 /* Download ULP ucode file */
#define FMAC_ULP_ENTRY		3 /* Inform ulp entry to Host */

struct brcmf_ulp {
	uint ulp_state;
	struct ulp_shm_info ulp_shm_offset;
};

struct brcmf_ulp_event {
	u16 version;
	u16 ulp_dongle_action;
};

struct brcmf_sdio_dev {
	struct sdio_func *func1;
	struct sdio_func *func2;
	u32 sbwad;			/* Save backplane window address */
	struct brcmf_core *cc_core;	/* chipcommon core info struct */
	struct brcmf_sdio *bus;
	struct device *dev;
	struct brcmf_bus *bus_if;
	struct brcmf_mp_device *settings;
	bool oob_irq_requested;
	bool sd_irq_requested;
	bool irq_en;			/* irq enable flags */
	spinlock_t irq_en_lock;
	bool sg_support;
	uint max_request_size;
	ushort max_segment_count;
	uint max_segment_size;
	uint txglomsz;
	struct sg_table sgtable;
	char fw_name[BRCMF_FW_NAME_LEN];
	char nvram_name[BRCMF_FW_NAME_LEN];
	bool wowl_enabled;
	enum brcmf_sdiod_state state;
	struct brcmf_sdiod_freezer *freezer;
	struct brcmf_ulp fmac_ulp;
	bool ulp;
};

/* sdio core registers */
struct sdpcmd_regs {
	u32 corecontrol;		/* 0x00, rev8 */
	u32 corestatus;			/* rev8 */
	u32 PAD[1];
	u32 biststatus;			/* rev8 */

	/* PCMCIA access */
	u16 pcmciamesportaladdr;	/* 0x010, rev8 */
	u16 PAD[1];
	u16 pcmciamesportalmask;	/* rev8 */
	u16 PAD[1];
	u16 pcmciawrframebc;		/* rev8 */
	u16 PAD[1];
	u16 pcmciaunderflowtimer;	/* rev8 */
	u16 PAD[1];

	/* interrupt */
	u32 intstatus;			/* 0x020, rev8 */
	u32 hostintmask;		/* rev8 */
	u32 intmask;			/* rev8 */
	u32 sbintstatus;		/* rev8 */
	u32 sbintmask;			/* rev8 */
	u32 funcintmask;		/* rev4 */
	u32 PAD[2];
	u32 tosbmailbox;		/* 0x040, rev8 */
	u32 tohostmailbox;		/* rev8 */
	u32 tosbmailboxdata;		/* rev8 */
	u32 tohostmailboxdata;		/* rev8 */

	/* synchronized access to registers in SDIO clock domain */
	u32 sdioaccess;			/* 0x050, rev8 */
	u32 PAD[3];

	/* PCMCIA frame control */
	u8 pcmciaframectrl;		/* 0x060, rev8 */
	u8 PAD[3];
	u8 pcmciawatermark;		/* rev8 */
	u8 PAD[155];

	/* interrupt batching control */
	u32 intrcvlazy;			/* 0x100, rev8 */
	u32 PAD[3];

	/* counters */
	u32 cmd52rd;			/* 0x110, rev8 */
	u32 cmd52wr;			/* rev8 */
	u32 cmd53rd;			/* rev8 */
	u32 cmd53wr;			/* rev8 */
	u32 abort;			/* rev8 */
	u32 datacrcerror;		/* rev8 */
	u32 rdoutofsync;		/* rev8 */
	u32 wroutofsync;		/* rev8 */
	u32 writebusy;			/* rev8 */
	u32 readwait;			/* rev8 */
	u32 readterm;			/* rev8 */
	u32 writeterm;			/* rev8 */
	u32 PAD[40];
	u32 clockctlstatus;		/* rev8 */
	u32 PAD[7];

	u32 PAD[128];			/* DMA engines */

	/* SDIO/PCMCIA CIS region */
	char cis[512];			/* 0x400-0x5ff, rev6 */

	/* PCMCIA function control registers */
	char pcmciafcr[256];		/* 0x600-6ff, rev6 */
	u16 PAD[55];

	/* PCMCIA backplane access */
	u16 backplanecsr;		/* 0x76E, rev6 */
	u16 backplaneaddr0;		/* rev6 */
	u16 backplaneaddr1;		/* rev6 */
	u16 backplaneaddr2;		/* rev6 */
	u16 backplaneaddr3;		/* rev6 */
	u16 backplanedata0;		/* rev6 */
	u16 backplanedata1;		/* rev6 */
	u16 backplanedata2;		/* rev6 */
	u16 backplanedata3;		/* rev6 */
	u16 PAD[31];

	/* sprom "size" & "blank" info */
	u16 spromstatus;		/* 0x7BE, rev2 */
	u32 PAD[464];

	u16 PAD[0x80];
};

/* Register/deregister interrupt handler. */
int brcmf_sdiod_intr_register(struct brcmf_sdio_dev *sdiodev);
void brcmf_sdiod_intr_unregister(struct brcmf_sdio_dev *sdiodev);

/* SDIO device register access interface */
/* Accessors for SDIO Function 0 */
#define brcmf_sdiod_func0_rb(sdiodev, addr, r) \
	sdio_f0_readb((sdiodev)->func1, (addr), (r))

#define brcmf_sdiod_func0_wb(sdiodev, addr, v, ret) \
	sdio_f0_writeb((sdiodev)->func1, (v), (addr), (ret))

/* Accessors for SDIO Function 1 */
#define brcmf_sdiod_readb(sdiodev, addr, r) \
	sdio_readb((sdiodev)->func1, (addr), (r))

#define brcmf_sdiod_writeb(sdiodev, addr, v, ret) \
	sdio_writeb((sdiodev)->func1, (v), (addr), (ret))

u32 brcmf_sdiod_readl(struct brcmf_sdio_dev *sdiodev, u32 addr, int *ret);
void brcmf_sdiod_writel(struct brcmf_sdio_dev *sdiodev, u32 addr, u32 data,
			int *ret);

/* Buffer transfer to/from device (client) core via cmd53.
 *   fn:       function number
 *   flags:    backplane width, address increment, sync/async
 *   buf:      pointer to memory data buffer
 *   nbytes:   number of bytes to transfer to/from buf
 *   pkt:      pointer to packet associated with buf (if any)
 *   complete: callback function for command completion (async only)
 *   handle:   handle for completion callback (first arg in callback)
 * Returns 0 or error code.
 * NOTE: Async operation is not currently supported.
 */
int brcmf_sdiod_send_pkt(struct brcmf_sdio_dev *sdiodev,
			 struct sk_buff_head *pktq);
int brcmf_sdiod_send_buf(struct brcmf_sdio_dev *sdiodev, u8 *buf, uint nbytes);

int brcmf_sdiod_recv_pkt(struct brcmf_sdio_dev *sdiodev, struct sk_buff *pkt);
int brcmf_sdiod_recv_buf(struct brcmf_sdio_dev *sdiodev, u8 *buf, uint nbytes);
int brcmf_sdiod_recv_chain(struct brcmf_sdio_dev *sdiodev,
			   struct sk_buff_head *pktq, uint totlen);

/* Flags bits */

/* Four-byte target (backplane) width (vs. two-byte) */
#define SDIO_REQ_4BYTE	0x1
/* Fixed address (FIFO) (vs. incrementing address) */
#define SDIO_REQ_FIXED	0x2

/* Read/write to memory block (F1, no FIFO) via CMD53 (sync only).
 *   rw:       read or write (0/1)
 *   addr:     direct SDIO address
 *   buf:      pointer to memory data buffer
 *   nbytes:   number of bytes to transfer to/from buf
 * Returns 0 or error code.
 */
int brcmf_sdiod_ramrw(struct brcmf_sdio_dev *sdiodev, bool write, u32 address,
		      u8 *data, uint size);

/* Issue an abort to the specified function */
int brcmf_sdiod_abort(struct brcmf_sdio_dev *sdiodev, struct sdio_func *func);

void brcmf_sdiod_sgtable_alloc(struct brcmf_sdio_dev *sdiodev);
void brcmf_sdiod_change_state(struct brcmf_sdio_dev *sdiodev,
			      enum brcmf_sdiod_state state);
#ifdef CONFIG_PM_SLEEP
bool brcmf_sdiod_freezing(struct brcmf_sdio_dev *sdiodev);
void brcmf_sdiod_try_freeze(struct brcmf_sdio_dev *sdiodev);
void brcmf_sdiod_freezer_count(struct brcmf_sdio_dev *sdiodev);
void brcmf_sdiod_freezer_uncount(struct brcmf_sdio_dev *sdiodev);
#else
static inline bool brcmf_sdiod_freezing(struct brcmf_sdio_dev *sdiodev)
{
	return false;
}
static inline void brcmf_sdiod_try_freeze(struct brcmf_sdio_dev *sdiodev)
{
}
static inline void brcmf_sdiod_freezer_count(struct brcmf_sdio_dev *sdiodev)
{
}
static inline void brcmf_sdiod_freezer_uncount(struct brcmf_sdio_dev *sdiodev)
{
}
#endif /* CONFIG_PM_SLEEP */

int brcmf_sdiod_probe(struct brcmf_sdio_dev *sdiodev);
int brcmf_sdiod_remove(struct brcmf_sdio_dev *sdiodev);

struct brcmf_sdio *brcmf_sdio_probe(struct brcmf_sdio_dev *sdiodev);
void brcmf_sdio_remove(struct brcmf_sdio *bus);
void brcmf_sdio_isr(struct brcmf_sdio *bus, bool in_isr);

void brcmf_sdio_wd_timer(struct brcmf_sdio *bus, bool active);
void brcmf_sdio_wowl_config(struct device *dev, bool enabled);
int brcmf_sdio_sleep(struct brcmf_sdio *bus, bool sleep);
void brcmf_sdio_trigger_dpc(struct brcmf_sdio *bus);

/* SHM offsets */
#define M_DS1_CTRL_SDIO(ptr)	((ptr).ulp_shm_offset.m_ulp_ctrl_sdio)
#define M_WAKEEVENT_IND(ptr)	((ptr).ulp_shm_offset.m_ulp_wakeevt_ind)
#define M_ULP_WAKE_IND(ptr)		((ptr).ulp_shm_offset.m_ulp_wakeind)
#define M_DS1_PHYTX_ERR_BLK(ptr)	((ptr).ulp_shm_offset.m_ulp_phytxblk)

#define D11_BASE_ADDR			0x18001000
#define D11_AXI_BASE_ADDR		0xE8000000
#define D11_SHM_BASE_ADDR		(D11_AXI_BASE_ADDR + 0x4000)

#define D11REG_ADDR(offset)	(D11_BASE_ADDR + (offset))
#define D11IHR_ADDR(offset)	(D11_AXI_BASE_ADDR + 0x400 + (2 * (offset)))
#define D11SHM_ADDR(offset)	(D11_SHM_BASE_ADDR + (offset))

/* MacControl register */
#define D11_MACCONTROL_REG			D11REG_ADDR(0x120)
#define D11_MACCONTROL_REG_WAKE		0x4000000

/* HUDI Sequence SHM bits */
#define	C_DS1_CTRL_SDIO_DS1_SLEEP		0x1
#define	C_DS1_CTRL_SDIO_MAC_ON			0x2
#define	C_DS1_CTRL_SDIO_RADIO_PHY_ON	0x4
#define	C_DS1_CTRL_SDIO_DS1_EXIT		0x8
#define	C_DS1_CTRL_PROC_DONE			0x100
#define	C_DS1_CTRL_REQ_VALID			0x200

/* M_ULP_WAKEIND bits */
#define	C_WATCHDOG_EXPIRY	BIT(0)
#define	C_FCBS_ERROR		BIT(1)
#define	C_RETX_FAILURE		BIT(2)
#define	C_HOST_WAKEUP		BIT(3)
#define	C_INVALID_FCBS_BLOCK	BIT(4)
#define	C_HUDI_DS1_EXIT		BIT(5)
#define	C_LOB_SLEEP		BIT(6)
#define	C_DS1_PHY_TXERR		BIT(9)
#define	C_DS1_WAKE_TIMER	BIT(10)

#define PHYTX_ERR_BLK_SIZE		18
#define D11SHM_FIRST2BYTE_MASK		0xFFFF0000
#define D11SHM_SECOND2BYTE_MASK		0x0000FFFF
#define D11SHM_2BYTE_SHIFT		16

#define D11SHM_RD(sdh, offset, ret) \
	brcmf_sdiod_readl(sdh, D11SHM_ADDR(offset), ret)

/* SHM Read is motified based on SHM 4 byte alignment as SHM size is 2 bytes and
 * 2 byte is currently not working on FMAC
 * If SHM address is not 4 byte aligned, then right shift by 16
 * otherwise, mask the first two MSB bytes
 * Suppose data in address 7260 is 0x440002 and it is 4 byte aligned
 * Correct SHM value is 0x2 for this SHM offset and next SHM value is 0x44
 */
#define D11SHM_RDW(sdh, offset, ret) \
	((offset % 4) ? \
		(brcmf_sdiod_readl(sdh, D11SHM_ADDR(offset), ret) \
		>> D11SHM_2BYTE_SHIFT) : \
		(brcmf_sdiod_readl(sdh, D11SHM_ADDR(offset), ret) \
		& D11SHM_SECOND2BYTE_MASK))

/* SHM is of size 2 bytes, 4 bytes write will overwrite other SHM's
 * First read 4 bytes and then clear the required two bytes based on
 * 4 byte alignment, then update the required value and write the
 * 4 byte value now
 */
#define D11SHM_WR(sdh, offset, val, mask, ret) \
	do { \
		if ((offset) % 4) \
			val = (val & D11SHM_SECOND2BYTE_MASK) | \
				((mask) << D11SHM_2BYTE_SHIFT); \
		else \
			val = (mask) | (val & D11SHM_FIRST2BYTE_MASK); \
		brcmf_sdiod_writel(sdh, D11SHM_ADDR(offset), val, ret); \
	} while (0)
#define D11REG_WR(sdh, addr, val, ret) \
	brcmf_sdiod_writel(sdh, addr, val, ret)

#define D11REG_RD(sdh, addr, ret) \
	brcmf_sdiod_readl(sdh, addr, ret)

#endif /* BRCMFMAC_SDIO_H */
