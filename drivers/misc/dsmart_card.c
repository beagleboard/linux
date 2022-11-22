// SPDX-License-Identifier: GPL-2.0-or-later
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dsmart_card_interface.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <asm/uaccess.h>

#define MAX_TRY_TIME_COUNT	100000
#define MAX_INT_TIMEOUT		(msecs_to_jiffies(1000))

#define RX_TX_FIFO	0x00
#define CONTROL		0x04
#define MSTATUS_MMASK	0x08	/* Each bit of the MSTATUS register is a source of a maskable interrupt. Writing logic '1' to the corresponding bit of MMASK register,
				   enables pending interrupt from the given source. */
#define CSTATUS		0x0c
#define CONFIG		0x10
#define MISC		0x14
#define CCLK_DIV	0x18
#define ETU		0x1c
#define EGT		0x20
#define WT0_BGT		0x24
#define WT1_CWT		0x28
#define WT2		0x2c
#define WT3		0x30
#define ERROR_MERROR	0x34
#define LENGTH		0x38

/* bit definition for CONFIG */
#define MIE		BIT(7) /* This bit is set to HIGH to globally enable interrupts, and cleared to LOW disables interrupts */
#define DMA_EN		BIT(6) /* HIGH state written into this bit enables the DMA mode */
#define BLOCK_ON	BIT(5) /* Active HIGH bit which is responsible for block mode activation. See 9.15 LENGTH register for details */
#define CLOCK_STOP	BIT(4) /* The value in which card clock (cclk pin) will be stopped in the power down mode. Automatically cleared when CRD_ERR is detected or STOP asserted */
#define C		BIT(3) /* These bits work as GPIO for pins c5v3 and c18. Used for class setting */
#define AB		BIT(2) /* These bits work as GPIO for pins c5v3 and c18. Used for class setting */
#define TMOD		BIT(1) /* HIGH state of this bit indicates that the T=1 protocol is used, else T=0 protocol. In the DSMART, it changes the error signaling of character repetition */
#define CRD_DET_EN	BIT(0) /* When this bit is HIGH CRD_DET bit in Card Status register will follow the state of the cdet pin */

/* bit definition for MSTATUS/MMASK */
#define CRD_EV_IRQ	BIT(7)
#define ERR_EV_IRQ	BIT(6)
#define TX_EV_IRQ	BIT(5)
#define RX_EV_IRQ	BIT(4)
#define TX_EM_IRQ	BIT(3)
#define BREAK_IRQ	BIT(2)
#define BLK_FIN_IRQ	BIT(1)
#define RX_AV_IRQ	BIT(0)

/* bit definition for CSTATUS */
#define ODDP		BIT(7)
#define SES_END_CARD_STATE	BIT(6)
#define TIMEOUT_CARD_STATE	BIT(5)
#define SYNC_ERR_CARD_STATE	BIT(4)
#define CONV_CARD_STATE		BIT(3)
#define CRD_ERR_CARD_STATE	BIT(2)
#define CRD_ACT_CARD_STATE	BIT(1)
#define CRD_DET_CARD_STATE	BIT(0)

/* bit definition for MISC */
#define TX_TR		BIT(6)
#define RX_TR		BIT(4)
#define TXFIFO_ON	BIT(3)
#define RXFIFO_ON	BIT(2)
#define CRC_MOD		BIT(1)
#define CRC_EN		BIT(0)
#define RX_TR_OFF	4
#define TX_TR_OFF	6
#define RX_TR_MSK	0x3
#define TX_TR_MSK	0x3

/* bit definition for CONTROL */
#define TX_END		BIT(7)
#define LAST		BIT(6)
#define PWD		BIT(5)
#define RT		BIT(4)
#define STOP		BIT(3)
#define CLR_ERR		BIT(2)
#define WRST		BIT(1)
#define START		BIT(0)

/* bit definition for WT0/BGT */
#define BIT_WT2		BIT(7)
#define WT1		BIT(6)
#define WT0		BIT(5)
#define BGT4		BIT(4)
#define BGT3		BIT(3)
#define BGT2		BIT(2)
#define BGT1		BIT(1)
#define BGT0		BIT(0)
#define WT0_OFF		5
#define BGT_OFF		0
#define WT0_MSK		0x7
#define BGT_MSK		0x1f

/* bit definition for WT1/CWT */
#define WT6		BIT(7)
#define WT5		BIT(6)
#define WT4		BIT(5)
#define BIT_WT3		BIT(4)
#define CWT3		BIT(3)
#define CWT2		BIT(2)
#define CWT1		BIT(1)
#define CWT0		BIT(0)
#define WT3_OFF		4
#define CWT_OFF		0
#define WT3_MSK		0xf
#define CWT_MSK		0xf

/* bit definition for ERROR/MERROR */
#define PAR_ERR_STATE	BIT(0)
#define CRC_ERR_STATE	BIT(1)
#define REP_ERR_STATE	BIT(2)
#define RX_OVER_ERR_STATE	BIT(3)
#define TX_FULL_ERR_STATE	BIT(4)
#define CWT_TIM_ERR_STATE	BIT(5)
#define RX_FULL_ERR_STATE	BIT(6)

#define MAX_DSMART_CARD_CWT	10080 /* in ETU */

#define CVOL_CLASS_MSK	0x3

#define DSMART_CARD_RCV_BUFFER_SIZE	400
#define DSMART_CARD_XMT_BUFFER_SIZE	300

#define DSMART_CARD_PRESENT_REMOVED	0
#define DSMART_CARD_PRESENT_DETECTED	1

#define DSMART_CARD_STATE_REMOVED		0
#define DSMART_CARD_STATE_DETECTED		1
#define DSMART_CARD_STATE_ACTIVATED		2
#define DSMART_CARD_STATE_RECEIVING		3
#define DSMART_CARD_STATE_RECEIVE_DONE		4
#define DSMART_CARD_STATE_ATR_RECEIVING		5
#define DSMART_CARD_STATE_ATR_RECEIVE_DONE	6

#define DSMART_CARD_STATE_XMTING		7

#define DSMART_CARD_FIFO_SIZE		16

enum trans_direction {
	RECEIVER_MOD,
	TRANSMITTER_MOD,
};

enum card_staus {
	DEACTIVATE,
	ACTIVATING,
	PSS_TRF,
	PSS_RECV,
	ACTIVATE,
};

enum dsmart_card_event {
	EVENT_CARD_DETECTED,
	EVENT_READ_COMPLETE,
	EVENT_WRITE_COMPLETE,
	EVENT_READ_ERROR,
	EVENT_WRITE_ERROR,
	EVENT_ACTIVATE_SUCCESS,
	EVENT_ACTIVATE_FAILED,
	EVENT_CARD_ERROR_DEACTIVATE,
	EVENT_CARD_SESSION_CLOSED,
	EVENT_RX_FULL,
	EVENT_CWT_TIME_OUT,
	EVENT_RX_OVER,
	EVENT_CRC_ERR,
	EVENT_PARITY_ERR,
	EVENT_SLAVE_ATR_DETECTED,
	EVENT_SLAVE_ATR_DONE,
};

struct dsmart_card {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	int irq;
	spinlock_t lock;
	u8 rcv_buffer[DSMART_CARD_RCV_BUFFER_SIZE];
	u8 xmt_buffer[DSMART_CARD_XMT_BUFFER_SIZE];
	struct completion xfer_done;
	enum card_staus card_sta;
	int protocol_type;
	int present;
	u32 expected_rcv_cnt;
	u32 rx_threshold;
	u32 tx_threshold;
	bool is_fixed_len;
	int rcv_head;
	int rcv_count;
	int xmt_head;
	int xmt_count;
	int xmt_remainning_count;
	int errval;
	struct dsmart_card_timing timing_data;
	int state;
	struct dsmart_card_baud baud_rate;
};

static inline void card_clk_div_set(void __iomem *base, u32 val)
{
	writel(val, base + CCLK_DIV);
}

static inline void card_vol_class_set(void __iomem *base, u32 val)
{
	u32 data = readl(base + CONFIG);
	data &= ~(CVOL_CLASS_MSK << AB);
	data |= val << AB;
	writel(data, base + CONFIG);
}

static inline void card_det_enable(void __iomem *base, bool en)
{
	u32 data = readl(base + CONFIG);

	if (en)
		data |= CRD_DET_EN;
	else
		data &= ~CRD_DET_EN;

	writel(data, base + CONFIG);
}

static inline void card_event_interrupt_enable(void __iomem *base, u32 mask, bool en)
{
	u32 data = readl(base + MSTATUS_MMASK);

	if (en)
		data |= mask;
	else
		data &= ~mask;

	writel(data, base + MSTATUS_MMASK);
}

static inline void card_glb_interrupt_enable(void __iomem *base, bool en)
{
	u32 data = readl(base + CONFIG);

	if (en)
		data |= MIE;
	else
		data &= ~MIE;
	writel(data, base + CONFIG);
}

static inline void card_odd_parity_set(void __iomem *base, bool en)
{
	u32 data = readl(base + CSTATUS);

	if (en)
		data |= ODDP;
	else /* EVEN Parity */
		data &= ~ODDP;

	writel(data, base + CSTATUS);
}

static inline void card_trans_protocol_set(void __iomem *base, int proto)
{
	u32 data = readl(base + CONFIG);

	if (proto == CARD_PROTOCOL_T0)
		data &= ~TMOD;
	else
		data |= TMOD;

	writel(data, base + CONFIG);
}

static inline void card_tx_fifo_enable(void __iomem *base, bool en)
{
	u32 data = readl(base + MISC);

	if (en)
		data |= TXFIFO_ON;
	else
		data &= ~TXFIFO_ON;

	writel(data, base + MISC);
}

static inline void card_rx_fifo_enable(void __iomem *base, bool en)
{
	u32 data = readl(base + MISC);

	if (en)
		data |= RXFIFO_ON;
	else
		data &= ~RXFIFO_ON;

	writel(data, base + MISC);
}

static inline void card_trans_direction_set(void __iomem *base, enum trans_direction dire)
{
	u32 data = readl(base + CONTROL);

	if (dire == RECEIVER_MOD)
		data &= ~RT;
	else
		data |= RT;

	writel(data, base + CONTROL);
}

static inline void card_warm_reset(void __iomem *base)
{
	u32 data = readl(base + CONTROL);

	data |= WRST;

	writel(data, base + CONTROL);
}

/*
 * shall not exceed WWT={960xDxWI}x1etu, where D=TA1 interface byte, WI=TC2 interface byte.
 * WWT value should be a priori converted to current ETU value written to the WT0-WT3 register
 */
static inline void card_wwt_time_set(void __iomem *base, u32 time)
{
	u32 data = readl(base + WT0_BGT);

	data &= ~(WT0_MSK << WT0_OFF);
	data |= ((time >> 0) & WT0_MSK) << WT0_OFF;
	writel(data, base + WT0_BGT);

	data = readl(base + WT1_CWT);
	data &= ~(WT3_MSK << WT3_OFF);
	data |= ((time >> 3) & WT3_MSK) << WT3_OFF;
	writel(data, base + WT1_CWT);

	data = ~0xff;
	data |= (time >> 7) & 0xff;
	writel(data, base + WT2);

	data = ~0xff;
	data |= (time >> 15) & 0xff;
	writel(data, base + WT3);
}

static inline void card_trans_start(void __iomem *base)
{
	writel(readl(base + CONTROL) | START, base + CONTROL);
}

/*
 *When set HIGH the interface begins the deactivation procedure. After card deactivation this signal is automatically cleared.
 *It can be also used to clear card detection or session end interrupt without invoking the activation procedure
 */
static inline void card_trans_stop(void __iomem *base)
{
	writel(readl(base + CONTROL) | STOP, base + CONTROL);
}

static inline void card_rx_threshold_set(void __iomem *base, u32 val)
{
	u32 data = readl(base + MISC);

	data &= ~(RX_TR_MSK << RX_TR_OFF);
	data |= (val & RX_TR_MSK) << RX_TR_OFF;
	writel(data, base + MISC);
}

static inline void card_tx_threshold_set(void __iomem *base, u32 val)
{
	u32 data = readl(base + MISC);

	data &= ~(TX_TR_MSK << TX_TR_OFF);
	data |= (val & TX_TR_MSK) << TX_TR_OFF;
	writel(data, base + MISC);
}

static inline u32 card_mstatus_get(void __iomem *base)
{
	return readl(base + MSTATUS_MMASK);
}

static inline u32 card_status_get(void __iomem *base)
{
	return readl(base + CSTATUS);
}

static inline void card_error_clear(void __iomem *base)
{
	writel(readl(base + CONTROL) | CLR_ERR, base + CONTROL);
}

static inline u32 card_error_status_get(void __iomem *base)
{
	return readl(base + ERROR_MERROR);
}

static inline void card_bgt_time_set(void __iomem *base, u32 time)
{
	u32 data = readl(base + WT0_BGT);

	data &= ~0x1f;
	data |= time & 0x1f;

	writel(data, base + WT0_BGT);
}

static inline void card_cwt_time_set(void __iomem *base, u32 time)
{
	u32 data = readl(base + WT1_CWT);

	data &= ~0xf;
	data |= time & 0xf;

	writel(data, base + WT1_CWT);
}

/* BWT={(2BWIx960)+11}x1etu. The result should be written into the WT0-WT3 registers */
static inline void card_bwt_time_set(void __iomem *base, u32 time)
{
	card_wwt_time_set(base, time);
}

/*
 * The value of 255 indicates that the minimum GUARD TIME (2 ETU for T=0 protocol and 1 ETU for T=1 protocol) should be used,
 * then the minimum character to character duration for subsequent transmissions shall be 12 ETUs if T=0 protocol is used,
 * or 11 ETUs if T=1 protocol is used. The exact value of EXTRA GUARD TIME (from 0 to 254) should be written into the guard time register.
 * Default value after reset is 1
 */

static inline void card_egt_time_set(void __iomem *base, u32 time) /* means char guard time */
{
	writel(time, base + EGT);
}

/* 1etu={1/f}x{Fi/Di} */
static inline void card_baud_rate_set(void __iomem *base, u32 fi, u32 di)
{
	u32 data = (fi & 0xf) << 4 | (di & 0xf);

	writel(data, base + ETU);
}

static inline u32 card_data_length_get(void __iomem *base)
{
	return readl(base + LENGTH);
}

static inline u32 card_fifo_data_get(void __iomem *base)
{
	return readl(base + RX_TX_FIFO);
}

static inline void card_fifo_data_set(void __iomem *base, u8 data)
{
	writel(data, base + RX_TX_FIFO);
}

static struct miscdevice dsmart_card_dev;

static void dsmart_card_rcv_read_fifo(struct dsmart_card *card)
{
	u32 i, data, len = card_data_length_get(card->base);

	pr_debug("%s, %d: len = %d\n", __func__, __LINE__, len);
	if (!len)
		card_fifo_data_get(card->base); //clear the fifo

	spin_lock(&card->lock);
	for (i = 0; i < len; i++) {
		data = card_fifo_data_get(card->base);
		card->rcv_buffer[card->rcv_head + card->rcv_count] = (u8)data;

		pr_debug("data%d = 0x%x , card->rcv_buffer[%d] = 0x%x ", i, data, card->rcv_head + card->rcv_count,
				card->rcv_buffer[card->rcv_head + card->rcv_count]);

		card->rcv_count++;
		if (card->rcv_head + card->rcv_count >= DSMART_CARD_RCV_BUFFER_SIZE) {
			pr_err("the software fifo is full, head:%d, count:%d\n", card->rcv_head,
					card->rcv_count);
			break;
		}
	}
	spin_unlock(&card->lock);
}

static void dsmart_card_xmt_write_fifo(struct dsmart_card *card)
{
	u32 i, data, len = card_data_length_get(card->base);
	u32 avail_size = 0;

	spin_lock(&card->lock);

	avail_size = DSMART_CARD_FIFO_SIZE - len;
	for (i = 0; i < avail_size; i++) {
		data = card->xmt_buffer[card->xmt_head + card->xmt_count];
		card_fifo_data_set(card->base, data);
		card->xmt_count++;
		if (card->xmt_head + card->xmt_count >= DSMART_CARD_XMT_BUFFER_SIZE) {
			pr_err("the software fifo is empty, head:%d, count:%d\n", card->xmt_head,
					card->xmt_count);
			break;
		}
	}

	spin_unlock(&card->lock);
}

static void dsmart_error_event_handle(struct dsmart_card *card)
{
	u32 err_status = card_error_status_get(card->base);

	if (err_status & RX_FULL_ERR_STATE) {
		dsmart_card_rcv_read_fifo(card);
		card->state = DSMART_CARD_STATE_RECEIVE_DONE;
		return;
	}

	if (err_status & TX_FULL_ERR_STATE) {
		//card->errval = -DSMART_CARD_E_TX_FULL;
		card->errval = DSMART_CARD_OK;
		return;
	}

	if (err_status & PAR_ERR_STATE) {
		card->errval = -DSMART_CARD_E_PAR_ERR;
		pr_err("par error occured\n");
	} else if (err_status & CRC_ERR_STATE) {
		card->errval = -DSMART_CARD_E_CRC_ERR;
		pr_err("crc error occured\n");
	} else if (err_status & REP_ERR_STATE) {
		card->errval = -DSMART_CARD_E_REP_ERR;
		pr_err("rep error occured\n");
	} else if (err_status & CWT_TIM_ERR_STATE) {
		card->errval = -DSMART_CARD_E_CWT_TIM;
		pr_err("cwt_tim error occured\n");
	} else if (err_status & RX_OVER_ERR_STATE) {
		card->errval = -DSMART_CARD_E_RX_OVER;
		pr_err("rx_over error occured\n");
	} else {
		pr_err("invalid error event number\n");
		return;
	}

	/* clear the error to stop interrupt */
	card_error_clear(card->base);
	card_event_interrupt_enable(card->base, ERR_EV_IRQ, false);

}

static void dsmart_card_event_handle(struct dsmart_card *card)
{
	u32 card_status = card_status_get(card->base);

	pr_debug("%s:%d, card event status = 0x%x\n", __func__, __LINE__, card_status);

	if (card_status & CRD_DET_CARD_STATE) {
		card->present = DSMART_CARD_PRESENT_DETECTED;
		card->state = DSMART_CARD_STATE_DETECTED;
		card->errval = DSMART_CARD_OK;

		pr_debug("%s:%d\n", __func__, __LINE__);

		card_event_interrupt_enable(card->base, CRD_EV_IRQ, false);
		complete(&card->xfer_done);
		/* just clear the card detection interrupt without invoking the activation procedure */
		//card_trans_stop(card->base);
	} else if (card_status & CRD_ACT_CARD_STATE) {
		pr_debug("%s:%d\n", __func__, __LINE__);

		complete(&card->xfer_done);

	} else if (card_status & TIMEOUT_CARD_STATE) {
		card->errval = -DSMART_CARD_E_ACT_TIMEOUT;
		card_error_clear(card->base);

		pr_debug("%s:%d\n", __func__, __LINE__);

	} else if (card_status & CRD_ERR_CARD_STATE) {
		card->errval = -DSMART_CARD_E_REMOVED;
		card_error_clear(card->base);

		pr_debug("%s:%d\n", __func__, __LINE__);
	} else if (card_status & SES_END_CARD_STATE) {
		pr_debug("the session is terminated\n");
	}
}

static irqreturn_t dsmart_card_handler(int irq, void *p)
{
	struct dsmart_card *card = (struct dsmart_card *)p;
	u32 card_event = card_mstatus_get(card->base);

	pr_debug("%s:%d, irq status = 0x%x\n", __func__, __LINE__, card_event);

	if (card_event & CRD_EV_IRQ) {
		dsmart_card_event_handle(card);
	}

	if (card_event & ERR_EV_IRQ) {
		dsmart_error_event_handle(card);
	}

	if (card_event & RX_AV_IRQ) {
		dsmart_card_rcv_read_fifo(card);
		pr_debug("%s:%d\n", __func__, __LINE__);
	}

	if (card_event & TX_EM_IRQ) {
		dsmart_card_xmt_write_fifo(card);
		pr_debug("%s:%d\n", __func__, __LINE__);
	}

	if (card_event & RX_EV_IRQ) {
		dsmart_card_rcv_read_fifo(card);
		pr_debug("%s:%d\n", __func__, __LINE__);
	}

	if (card_event & TX_EV_IRQ) {
		dsmart_card_xmt_write_fifo(card);
		pr_debug("%s:%d\n", __func__, __LINE__);
	}

	//complete(&card->xfer_done);

	return IRQ_HANDLED;
}

static int dsmart_card_check_timing_data(struct dsmart_card_timing *timing_data)
{

	pr_debug("wwt = 0x%d, cwt = 0x%d, bwt = 0x%d, bgt = 0x%d, egt = 0x%d\n",
			timing_data->wwt, timing_data->cwt, timing_data->bwt, timing_data->bgt, timing_data->egt);

	if (timing_data->wwt > 0x7fffff ||
			timing_data->cwt > 0xf ||
			timing_data->bwt > 0x7fffff ||
			timing_data->bgt > 0x1f ||
			timing_data->egt > 0xf) {
		pr_err("the timing value is out of scope\n");
		return -EINVAL;
	}

	return 0;
}

static int dsmart_card_baud_rate_check(struct dsmart_card_baud *rate)
{
	/*
	 * According to Table: The clock rate conversion Fi integer & Table: The baud rate dividion Di integer
	 */
	if (rate->fi == 7 || rate->fi >= 0xe || rate->di >= 0xa)
		return -EINVAL;

	return 0;
}

static void dsmart_card_baud_rate_set(struct dsmart_card *card)
{
	if (card->baud_rate.fi && card->baud_rate.di)
		card_baud_rate_set(card->base, card->baud_rate.fi, card->baud_rate.di);
}

static void dsmart_card_data_reset(struct dsmart_card *card)
{
	card->errval = DSMART_CARD_OK;
	card->rcv_count = 0;
	card->rcv_head = 0;
	card->xmt_count = 0;
	card->xmt_remainning_count = 0;
	card->xmt_head = 0;
	card->protocol_type = CARD_PROTOCOL_T0;

	card_event_interrupt_enable(card->base, 0xff, false);
	card_tx_fifo_enable(card->base, false);
	card_rx_fifo_enable(card->base, false);
	card_rx_threshold_set(card->base, 2);			   /* set minimum threshold for receiving atr */
	card_event_interrupt_enable(card->base, CRD_EV_IRQ, true); /* enable card & error event occur first */
	card_glb_interrupt_enable(card->base, true);

	/* set card working frequency as 1M clk, clk = pclk / {(CLK_DIV + 1) * 2}*/
#ifdef CONFIG_MAX_CCLK
	card_clk_div_set(card->base, 1);	//7
#else
	card_clk_div_set(card->base, 29);
#endif

	memset(card->rcv_buffer, 0, DSMART_CARD_RCV_BUFFER_SIZE);
	memset(card->xmt_buffer, 0, DSMART_CARD_XMT_BUFFER_SIZE);
	memset(&card->timing_data, 0, sizeof(card->timing_data));
	memset(&card->baud_rate, 0, sizeof(card->baud_rate));

	reinit_completion(&card->xfer_done);
}

static int dsmart_card_detect(struct dsmart_card *card)
{
	reinit_completion(&card->xfer_done);
	card_det_enable(card->base, true);
	/* check the card if detected */
	if (wait_for_completion_interruptible(&card->xfer_done)) {/* wait for CRD_DET bit set */
		pr_err("%s, %d: dsmart card detected timeout\n", __func__, __LINE__);
		return -ETIMEDOUT;
	}

	card->state = DSMART_CARD_STATE_DETECTED;

	pr_debug("%s:%d\n", __func__, __LINE__);

	return 0;
}

static int dsmart_card_start_data_rcv(struct dsmart_card *card, unsigned long arg)
{
	dsmart_card_baud_rate_set(card);
	card_trans_protocol_set(card->base, card->protocol_type);
	card_rx_fifo_enable(card->base, true);
	card_trans_direction_set(card->base, RECEIVER_MOD);
	card->state = DSMART_CARD_STATE_RECEIVING;
	card_event_interrupt_enable(card->base, RX_EV_IRQ | RX_AV_IRQ, true);

	card_trans_start(card->base);

#if 0
	reinit_completion(&card->xfer_done);

	pr_debug("%s:%d\n", __func__, __LINE__);

	/* wait RX_TD interrupt */
	if (wait_for_completion_interruptible(&card->xfer_done)) {
		pr_err("%s,%d: timeout in receiving normal data\n", __func__, __LINE__);
		return -DSMART_CARD_E_DATA_TIMEOUT;
	}
#endif
	pr_debug("%s:%d\n", __func__, __LINE__);

	return 0;
}

static int dsmart_card_atr_rcv(struct dsmart_card *card, unsigned long arg)
{
	int __maybe_unused i;
	u32 delay_cnt = 0;
	u32 card_status = card_status_get(card->base);

	card_tx_fifo_enable(card->base, true);
	card_trans_direction_set(card->base, RECEIVER_MOD);

	/*Set the cwt timer.Refer the setting of ATR on EMV4.3 book*/
	//card_wwt_time_set(card->base, MAX_DSMART_CARD_CWT);

	/* Set 12 ETUS */
	//card_egt_time_set(card->base, 0xff);

	card_trans_start(card->base);	/* activate the secquence */

	card_fifo_data_set(card->base, 0x1);
	card_fifo_data_set(card->base, 0x2);
	card_fifo_data_set(card->base, 0x3);
	card_trans_direction_set(card->base, TRANSMITTER_MOD);

	pr_debug("%s:%d\n", __func__, __LINE__);

#if 1
	while ((card_status & CRD_ACT_CARD_STATE) != CRD_ACT_CARD_STATE) {
		card_status = card_status_get(card->base);
		mdelay(1);
		delay_cnt++;
	}
#else
	card_event_interrupt_enable(card->base, CRD_EV_IRQ, true);
	reinit_completion(&card->xfer_done);
	if (wait_for_completion_interruptible(&card->xfer_done)) {	/* wait for successful activation */
		pr_err("%s, %d: dsmart card activate timeout\n", __func__, __LINE__);
		return -ETIMEDOUT;
	}
#endif
	pr_debug("delay_cnt = %d ms\n", delay_cnt);
	card->state = DSMART_CARD_STATE_ACTIVATED;

	pr_debug("%s:%d\n", __func__, __LINE__);

	card_event_interrupt_enable(card->base, RX_EV_IRQ, true);

	card_rx_fifo_enable(card->base, true);

	card->state = DSMART_CARD_STATE_ATR_RECEIVING;

	pr_debug("%s:%d\n", __func__, __LINE__);

	card_trans_direction_set(card->base, RECEIVER_MOD);

	pr_debug("%s:%d\n", __func__, __LINE__);

	msleep(5000); //how to confirm the atr length ?
	card->state = DSMART_CARD_STATE_ATR_RECEIVE_DONE;

#ifdef CONFIG_ATR_DATA_PRINT
	for (i = 0; i < card->rcv_count; i++) {
		pr_debug("0x%02x  ", card->rcv_buffer[i]);
	}
#endif

	pr_info("[%s:%d]set fcclk as 15M\n", __func__, __LINE__);
	card_clk_div_set(card->base, 1);

	return 0;
}

static int dsmart_card_start_data_xmt(struct dsmart_card *card, unsigned long arg)
{
	dsmart_card_baud_rate_set(card);
	card_trans_protocol_set(card->base, card->protocol_type);
	card_tx_fifo_enable(card->base, true);
	card_trans_direction_set(card->base, TRANSMITTER_MOD);
	card->state = DSMART_CARD_STATE_XMTING;
	card_event_interrupt_enable(card->base, TX_EV_IRQ | TX_EM_IRQ, true);

	card_trans_start(card->base);

	reinit_completion(&card->xfer_done);

	pr_debug("%s:%d\n", __func__, __LINE__);

	/* wait RX_TD interrupt */
	if (wait_for_completion_interruptible(&card->xfer_done)) {
		pr_err("%s,%d: timeout in receiving normal data\n", __func__, __LINE__);
		return -DSMART_CARD_E_DATA_TIMEOUT;
	}
	pr_debug("%s:%d\n", __func__, __LINE__);

	return 0;
}

static void dsmart_card_deactivate(struct dsmart_card *card)
{
	card_trans_stop(card);
}

static int dsmart_card_warm_reset(struct dsmart_card *card)
{
	card_warm_reset(card->base);
	pr_info("[%s:%d]set fcclk as 1M\n", __func__, __LINE__);
	card_clk_div_set(card->base, 29);
	card->errval = DSMART_CARD_OK;
	return dsmart_card_atr_rcv(card, 0);
}

static void dsmart_card_timing_counter_set(struct dsmart_card *card)
{
	if (card->timing_data.wwt && card->protocol_type == CARD_PROTOCOL_T0) {
		card->timing_data.cwt = card->timing_data.wwt;
		card->timing_data.bwt = card->timing_data.wwt;
		card_wwt_time_set(card->base, card->timing_data.cwt);
	}

	if (card->timing_data.bgt && card->protocol_type == CARD_PROTOCOL_T1)
		card_bgt_time_set(card->base, card->timing_data.bgt);

	if (card->timing_data.cwt && card->protocol_type == CARD_PROTOCOL_T1)
		card_cwt_time_set(card->base, card->timing_data.cwt);

	if (card->timing_data.egt)
		card_egt_time_set(card->base, card->timing_data.egt);
}

static int dsmart_card_open(struct inode *inode, struct file *filp)
{
	struct dsmart_card *dsmart_card = dev_get_drvdata(dsmart_card_dev.parent);

	filp->private_data = dsmart_card;
	spin_lock_init(&dsmart_card->lock);

	//dsmart_card_data_reset(dsmart_card);

	return 0;
}

static int dsmart_card_release(struct inode *inode, struct file *filp)
{
	/* struct dsmart_card *dsmart_card = (struct dsmart_card *)filp->private_data; */

	return 0;
}

static long dsmart_card_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int __maybe_unused i;
	struct dsmart_card *card = (struct dsmart_card *)filp->private_data;
	u32 copy_cnt;
	unsigned long flags;
	u32 xmt_size = 0;

	pr_debug("%s entering, cmd = 0x%x\n", __func__, cmd);

	switch (cmd) {
	case DSMART_CARD_IOCTL_SET_PROTOCOL:
		pr_debug("%s:%d\n", __func__, __LINE__);

		ret = copy_from_user(&card->protocol_type, (int *)arg, sizeof(int));
		if (ret)
			pr_err("protocol copy error\n");
		break;
	case DSMART_CARD_IOCTL_DEACTIVATE:
		pr_debug("%s:%d\n", __func__, __LINE__);

		dsmart_card_deactivate(card);

		break;
	case DSMART_CARD_IOCTL_COLD_RESET:
		pr_debug("%s:%d\n", __func__, __LINE__);

		card->present = DSMART_CARD_PRESENT_REMOVED;
		card->state = DSMART_CARD_STATE_REMOVED;
		dsmart_card_data_reset(card);
		ret = dsmart_card_detect(card);
		if (ret) {
			pr_err("no card found\n");
			ret = -DSMART_CARD_E_DATA_TIMEOUT;
			break;
		}

		pr_debug("%s:%d\n", __func__, __LINE__);

		ret = dsmart_card_atr_rcv(card, arg);
		break;
	case DSMART_CARD_IOCTL_WARM_RESET:
		pr_debug("%s:%d\n", __func__, __LINE__);
		card_clk_div_set(card->base, 29);
		ret = dsmart_card_warm_reset(card);
		break;
	case DSMART_CARD_IOCTL_SET_TIMING:

		pr_debug("%s:%d\n", __func__, __LINE__);

		ret = copy_from_user(&card->timing_data, (struct dsmart_card_timing *)arg,
				     sizeof(struct dsmart_card_timing));
		if (ret) {
			pr_err("timing copy error\n");
			break;
		}

		ret = dsmart_card_check_timing_data(&card->timing_data);
		break;
	case DSMART_CARD_IOCTL_SET_BAUD:

		pr_debug("%s:%d\n", __func__, __LINE__);

		ret = copy_from_user(&card->baud_rate, (struct dsmart_card_baud *)arg,
				     sizeof(struct dsmart_card_baud));
		if (ret) {
			pr_err("baud rate copy error\n");
			break;
		}

		ret = dsmart_card_baud_rate_check(&card->baud_rate);
		if (ret)
			pr_err("invalid baud rate value\n");

		break;
	case DSMART_CARD_IOCTL_SET_RX_THRESHOLD:

		pr_debug("%s:%d\n", __func__, __LINE__);

		ret = copy_from_user(&card->rx_threshold, (int *)arg, sizeof(int));
		if (ret) {
			pr_err("threshold copy error\n");
			break;
		}
		break;
	case DSMART_CARD_IOCTL_SET_TX_THRESHOLD:

		pr_debug("%s:%d\n", __func__, __LINE__);

		break;
	case DSMART_CARD_IOCTL_ATR_RCV:

		pr_debug("%s:%d\n", __func__, __LINE__);

		if (card->state != DSMART_CARD_STATE_ATR_RECEIVE_DONE) {
			pr_err("failed to activate sequence\n");
			ret = -DSMART_CARD_E_ACTIVATE_FAILED;
			goto report_error;
		}

		ret = copy_to_user(&(((struct dsmart_card_atr *)arg)->len), &card->rcv_count,
				   sizeof(card->rcv_count));
		if (ret) {
			pr_err("len copy error\n");
			ret = -DSMART_CARD_E_ACCESS;
			goto report_error;
		}

		ret = copy_to_user(((struct dsmart_card_atr __user *)arg)->atr_buffer, card->rcv_buffer, card->rcv_count);
		if (ret) {
			pr_err("failed to copy atr data to user\n");
			ret = -DSMART_CARD_E_ACCESS;
			goto report_error;
		}
#ifdef CONFIG_ATR_DATA_PRINT
		for (i = 0; i < card->rcv_count; i++) {
			pr_info("0x%02x  ", card->rcv_buffer[i]);
		}
#endif
		pr_debug("%s:%d\n", __func__, __LINE__);

report_error:
		pr_debug("%s:%d\n", __func__, __LINE__);

		ret = copy_to_user(&(((struct dsmart_card_atr __user *)arg)->errval), &card->errval,
				   sizeof(card->errval));
		if (ret) {
			pr_err("ATR error value copy error\n");
			ret = -DSMART_CARD_E_ACCESS;
			break;
		}

		card->rcv_count	= 0;
		card->rcv_head	= 0;
		card->errval	= 0;
		break;
	case DSMART_CARD_IOCTL_XMT:
		/*ret = dsmart_card_xmt(card);*/
		xmt_size = 0;
		pr_debug("%s:%d\n", __func__, __LINE__);

		ret = copy_from_user(&card->xmt_remainning_count, &(((struct dsmart_card_xmt __user *)arg)->xmt_length),
				sizeof(card->xmt_remainning_count));
		if (ret) {
			pr_err("get user xmt length error\n");
			ret = -DSMART_CARD_E_ACCESS;
			break;
		}

		if (card->xmt_remainning_count > DSMART_CARD_XMT_BUFFER_SIZE) {
			pr_err("the data to send is too big\n");
			ret = -EINVAL;
			break;
		}

		ret = copy_from_user(&card->xmt_buffer[card->xmt_head], ((struct dsmart_card_xmt __user *)arg)->xmt_buffer, xmt_size);
		if (ret) {
			pr_err("get user buffer data error\n");
			ret = -DSMART_CARD_E_ACCESS;
			break;
		}

		if (card->state != DSMART_CARD_STATE_XMTING) {
			dsmart_card_timing_counter_set(card);
			ret = dsmart_card_start_data_xmt(card, arg);
			if (ret) {
				pr_err("failed to xmt normal data\n");
				break;
			}
		}

		spin_lock_irqsave(&card->lock, flags);
		card->xmt_head += card->xmt_count;
		card->xmt_remainning_count -= card->xmt_count;
		card->xmt_count = 0;
		card->errval = 0;
		spin_unlock_irqrestore(&card->lock, flags);

		//dsmart_card_start_data_rcv(card, 0);

		break;
	case DSMART_CARD_IOCTL_RCV:

		pr_debug("%s:%d\n", __func__, __LINE__);

		ret = copy_from_user(&card->expected_rcv_cnt, &(((struct dsmart_card_rcv __user *)arg)->rcv_length),
				     sizeof(card->expected_rcv_cnt));
		if (ret) {
			pr_err("rcv data error\n");
			ret = -DSMART_CARD_E_ACCESS;
			break;
		}

		if (card->expected_rcv_cnt)
			card->is_fixed_len = true;

		if ((card->rcv_count >= card->expected_rcv_cnt) && card->rcv_count)
			goto copy_data;

		if (card->state != DSMART_CARD_STATE_RECEIVING) {
			dsmart_card_timing_counter_set(card);
			ret = dsmart_card_start_data_rcv(card, arg);
			if (ret) {
				pr_err("failed to receive normal data\n");
				break;
			}

			msleep(5000);
		}

		pr_debug("%s:%d\n", __func__, __LINE__);

copy_data:

		pr_debug("%s:%d\n", __func__, __LINE__);

		if (card->is_fixed_len)
			copy_cnt = card->rcv_count >= card->expected_rcv_cnt ? card->expected_rcv_cnt : card->rcv_count;
		else
			copy_cnt = card->rcv_count;

		ret = copy_to_user(&(((struct dsmart_card_rcv __user *)arg)->rcv_length), &copy_cnt, sizeof(copy_cnt));
		if (ret) {
			pr_err("length copy error\n");
			ret = -DSMART_CARD_E_ACCESS;
			break;
		}

		ret = copy_to_user(((struct dsmart_card_rcv __user *)arg)->rcv_buffer, &card->rcv_buffer[card->rcv_head], copy_cnt);
		if (ret) {
			pr_err("buffer copy error\n");
			ret = -DSMART_CARD_E_ACCESS;
			break;
		}

		ret = copy_to_user(&(((struct dsmart_card_rcv __user *)arg)->errval), &card->errval, sizeof(card->errval));
		if (ret) {
			pr_err("error value copy error\n");
			ret = -DSMART_CARD_E_ACCESS;
			break;
		}

		pr_debug("%s:%d, copy_cnt = %d, rcv_head = %d, rcv_count = %d\n", __func__, __LINE__, copy_cnt, card->rcv_head, card->rcv_count);

#ifdef CONFIG_ATR_DATA_PRINT
		for (i = 0; i < copy_cnt; i++) {
			pr_info("%s, %d, 0x%x  ", __func__, __LINE__, card->rcv_buffer[card->rcv_head + i]);
		}
#endif

		spin_lock_irqsave(&card->lock, flags);
		card->rcv_head += copy_cnt;
		card->rcv_count -= copy_cnt;
		card->errval = 0;
		spin_unlock_irqrestore(&card->lock, flags);
		break;
	default:
		pr_err("invalid ioctol command\n");
		return -EINVAL;
	}

	pr_debug("%s:%d\n", __func__, __LINE__);

	return ret;
}

static const struct file_operations dsmart_card_fops = {
	.owner	= THIS_MODULE,
	.open	= dsmart_card_open,
	.release = dsmart_card_release,
	.unlocked_ioctl	= dsmart_card_ioctl,
};

static struct miscdevice dsmart_card_dev = {
	MISC_DYNAMIC_MINOR,
	"dsmart_card",
	&dsmart_card_fops,
};

static int dsmart_card_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct dsmart_card *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0) {
		dev_err(&pdev->dev, "failed to get irq for dsmart card\n");
		return priv->irq;
	}

	/* optional clock, default open */
	priv->clk = devm_clk_get(&pdev->dev, "dsmart_card");
	if(IS_ERR(priv->clk))
		priv->clk = NULL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	platform_set_drvdata(pdev, priv);

	dsmart_card_dev.parent = &(pdev->dev);

	card_event_interrupt_enable(priv->base, 0xff, false);

	init_completion(&priv->xfer_done);

	ret = devm_request_irq(&pdev->dev, priv->irq, dsmart_card_handler, 0,
				"dsmart_card", priv);
	if (ret) {
		dev_err(&pdev->dev, "failed to register a dsmart card IRQ handler(%d)\n", ret);
		return ret;
	}

	ret = misc_register(&dsmart_card_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register a misc device\n");
		return ret;
	}

	dev_info(&pdev->dev, "initialized iso7816 smart card driver\n");

	return 0;
}

static int dsmart_card_remove(struct platform_device *pdev)
{
	misc_deregister(&dsmart_card_dev);

	return 0;
}

static const struct of_device_id dsmart_card_of_match[] = {
	{.compatible = "thead,light-iso7816-card"},
	{/* sentinel */},
};
MODULE_DEVICE_TABLE(of, dsmart_card_of_match);

static struct platform_driver dsmart_card_driver = {
	.driver = {
		.name = "iso7816-card-drv",
		.owner = THIS_MODULE,
		.of_match_table = dsmart_card_of_match,
	},
	.probe = dsmart_card_probe,
	.remove = dsmart_card_remove,
};

module_platform_driver(dsmart_card_driver);
MODULE_AUTHOR("wei.liu <lw32886@linux.alibaba.com>");
MODULE_DESCRIPTION("iso7816 smart card platform driver");
MODULE_LICENSE("GPL v2");
