/* mailbox.h */

#ifndef MAILBOX_H
#define MAILBOX_H

typedef u32 mbox_msg_t;
typedef void (mbox_receiver_t)(mbox_msg_t msg);

struct mbox;	/* contents are private */

struct mbox *mbox_get(const char *id);
extern int mbox_send(struct mbox *mbox_h, mbox_msg_t msg);
extern int register_mbox_receiver(struct mbox *mbox, unsigned char cmd,
				 mbox_receiver_t *rcv);
extern int unregister_mbox_receiver(struct mbox *mbox, unsigned char cmd,
				   mbox_receiver_t *rcv);
extern void enable_mbox_irq(struct mbox *mbox);
extern void disable_mbox_irq(struct mbox *mbox);
extern void mbox_init_seq(struct mbox *mbox);

/*
 * mailbox command: 0x00 - 0x7f
 * when a driver wants to use mailbox, it must reserve mailbox commands here.
 */
#define MBOX_CMD_MAX	0x80

/* DSP Gateway */
#define MBOX_CMD_DSP_WDSND	0x10
#define MBOX_CMD_DSP_WDREQ	0x11
#define MBOX_CMD_DSP_BKSND	0x20
#define MBOX_CMD_DSP_BKREQ	0x21
#define MBOX_CMD_DSP_BKYLD	0x23
#define MBOX_CMD_DSP_BKSNDP	0x24
#define MBOX_CMD_DSP_BKREQP	0x25
#define MBOX_CMD_DSP_TCTL	0x30
#define MBOX_CMD_DSP_TCTLDATA	0x31
#define MBOX_CMD_DSP_POLL	0x32
#define MBOX_CMD_DSP_WDT		0x50
#define MBOX_CMD_DSP_RUNLEVEL	0x51
#define MBOX_CMD_DSP_PM		0x52
#define MBOX_CMD_DSP_SUSPEND	0x53
#define MBOX_CMD_DSP_KFUNC	0x54
#define MBOX_CMD_DSP_TCFG	0x60
#define MBOX_CMD_DSP_TADD	0x62
#define MBOX_CMD_DSP_TDEL	0x63
#define MBOX_CMD_DSP_TSTOP	0x65
#define MBOX_CMD_DSP_DSPCFG	0x70
#define MBOX_CMD_DSP_REGRW	0x72
#define MBOX_CMD_DSP_GETVAR	0x74
#define MBOX_CMD_DSP_SETVAR	0x75
#define MBOX_CMD_DSP_ERR		0x78
#define MBOX_CMD_DSP_DBG		0x79

#endif /* MAILBOX_H */
