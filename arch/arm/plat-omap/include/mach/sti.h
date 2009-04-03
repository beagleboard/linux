#ifndef __ASM_ARCH_OMAP_STI_H
#define __ASM_ARCH_OMAP_STI_H

#include <asm/io.h>

/*
 * STI/SDTI
 */
#define STI_REVISION		0x00
#define STI_SYSCONFIG		0x10
#define STI_SYSSTATUS		0x14
#define STI_IRQSTATUS		0x18
#define STI_IRQSETEN		0x1c

#if defined(CONFIG_ARCH_OMAP1)
#define STI_IRQCLREN		0x20
#define STI_ER			0x24
#define STI_DR			0x28
#define STI_RX_DR		0x2c
#define STI_RX_STATUS		0x30
#define STI_CLK_CTRL		0x34
#define STI_IOBOTT0		0x4c
#define STI_IOTOP0		0x50
#define STI_IOBOTT1		0x54
#define STI_IOTOP1		0x58
#define STI_SERIAL_CFG		0x60

#define STI_OCPT2_MATCH_INT	0
#define STI_OCPT1_MATCH_INT	1
#define STI_EMIFS_MATCH_INT	2
#define STI_EMIFF_MATCH_INT	3
#define STI_IO_MATCH_INT	4
#define STI_RX_INT		5
#define STI_DUMP_REQUEST_INT	6
#define STI_DUMP_UNDERRUN_INT	7
#define STI_WAKEUP_INT		9

#define STI_NR_IRQS	10

#define STI_IRQSTATUS_MASK	0x2ff

#define STI_RXFIFO_EMPTY	(1 << 0)

/*
 * We use the following enums to retain consistency with the STI "functional"
 * specification.
 */

/* STI_ER */
enum {
	UnlockStatMatch	= (1 <<  2), /* Unlock status match event regs */
	IOMPUStr1En1	= (1 <<  3), /* MPU IO match, strobe 1, window 1 */
	IOMPUStr0En1	= (1 <<  4), /* MPU IO match, strobe 0, window 1 */
	IOMPUStr1En0	= (1 <<  5), /* MPU IO match, strobe 1, window 0 */
	IOMPUStr0En0	= (1 <<  6), /* MPU IO match, strobe 0, window 0 */
	IODSPStr1En1	= (1 <<  7), /* DSP IO match, strobe 1, window 1 */
	IODSPStr0En1	= (1 <<  8), /* DSP IO match, strobe 0, window 1 */
	IODSPStr1En0	= (1 <<  9), /* DSP IO match, strobe 1, window 0 */
	IODSPStr0En0	= (1 << 10), /* DSP IO match, strobe 0, window 0 */
	MemMatchEn	= (1 << 11), /* Memory matched event */
	DSPCmdEn	= (1 << 12), /* DSP command write */
	MPUCmdEn	= (1 << 13), /* MPU command write */
	MemDumpEn	= (1 << 14), /* System memory dump */
	STIEn		= (1 << 15), /* Global trace enable */
};

#define STI_PERCHANNEL_SIZE	4

#define to_channel_address(channel) \
	(sti_channel_base + STI_PERCHANNEL_SIZE * (channel))

#elif defined(CONFIG_ARCH_OMAP2)

/* XTI interrupt bits */
enum {
	STI_WAKEUP_INT	= 0,
	STI_ETB_THRESHOLD_INT,
	STI_RX_INT,
	STI_DUMP_REQUEST_INT,
	STI_NR_IRQS,
};

/* XTI_TRACESELECT */
enum {
	CmdTimeStampEn	= (1 << 0),	/* Command write timestamps */
	WinTimeStampEn	= (1 << 1),	/* Window match timestamps */
	WinMatchEn	= (1 << 2),	/* Window match trace */
	DSPCmdEn	= (1 << 3),	/* DSP command write */
	MPUCmdEn	= (1 << 4),	/* MPU command write */
	MemDumpEn0	= (1 << 5),	/* System memory dump */
	MemDumpEn1	= (1 << 6),
	MemDumpEn2	= (1 << 7),
	ExtTriggerEn	= (1 << 8),	/* External trace trigger */
	STIEn		= (1 << 9),	/* System trace enable */
};

#define STI_IRQSTATUS_MASK	0x0f
#define STI_PERCHANNEL_SIZE	64

/* XTI registers */
#define XTI_SYSSTATUS		0x14
#define XTI_TRACESELECT		0x24
#define XTI_RXDATA		0x28
#define XTI_SCLKCRTL		0x2c
#define XTI_SCONFIG		0x30

/* STI Compatability */
#define STI_RX_STATUS		XTI_SYSSTATUS
#define STI_IRQCLREN		STI_IRQSETEN
#define STI_ER			XTI_TRACESELECT
#define STI_DR			XTI_TRACESELECT
#define STI_RX_DR		XTI_RXDATA
#define STI_CLK_CTRL		XTI_SCLKCRTL
#define STI_SERIAL_CFG		XTI_SCONFIG

#define STI_RXFIFO_EMPTY	(1 << 8)

#define to_channel_address(channel) \
	(sti_channel_base + STI_PERCHANNEL_SIZE * (channel))

#elif defined(CONFIG_ARCH_OMAP3)

#define STI_PERCHANNEL_SIZE	0x1000
#define to_channel_address(channel) \
	(sti_channel_base + STI_PERCHANNEL_SIZE * (channel) + 0x800)

#endif

/* arch/arm/plat-omap/sti/sti.c */
extern void __iomem *sti_base, *sti_channel_base;

int sti_request_irq(unsigned int irq, void *handler, unsigned long arg);
void sti_free_irq(unsigned int irq);
void sti_enable_irq(unsigned int irq);
void sti_disable_irq(unsigned int irq);
void sti_ack_irq(unsigned int irq);

int sti_trace_enable(int event);
void sti_trace_disable(int event);

void sti_channel_write_trace(int len, int id, void *data, unsigned int channel);

/* arch/arm/plat-omap/sti/sti-fifo.c */
int sti_read_packet(unsigned char *buf, int maxsize);

static inline unsigned long sti_readl(unsigned long reg)
{
	return __raw_readl(sti_base + reg);
}

static inline void sti_writel(unsigned long data, unsigned long reg)
{
	__raw_writel(data, sti_base + reg);
}

static inline void sti_channel_writeb(unsigned char data, unsigned int channel)
{
	__raw_writeb(data, to_channel_address(channel));
}

static inline void sti_channel_writel(unsigned long data, unsigned int channel)
{
	__raw_writel(data, to_channel_address(channel));
}

#define STI_TRACE_CONTROL_CHANNEL	253

static inline void sti_channel_flush(unsigned int channel)
{
	sti_channel_writeb(channel, STI_TRACE_CONTROL_CHANNEL);
}
#endif /* __ASM_ARCH_OMAP_STI_H */
