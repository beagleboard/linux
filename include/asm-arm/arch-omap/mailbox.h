/* mailbox.h */

#ifndef MAILBOX_H
#define MAILBOX_H

typedef u32 mbx_msg_t;
typedef void (mbx_receiver_t)(mbx_msg_t msg);

struct mbx;	/* contents are private */

struct mbx *mbx_get(const char *id);
extern int mbx_send(struct mbx *mbx_h, mbx_msg_t msg);
extern int register_mbx_receiver(struct mbx *mbx, unsigned char cmd,
				 mbx_receiver_t *rcv);
extern int unregister_mbx_receiver(struct mbx *mbx, unsigned char cmd,
				   mbx_receiver_t *rcv);
extern void enable_mbx_irq(struct mbx *mbx);
extern void disable_mbx_irq(struct mbx *mbx);
extern void mbx_init_seq(struct mbx *mbx);

/*
 * mailbox command: 0x00 - 0x7f
 * when a driver wants to use mailbox, it must reserve mailbox commands here.
 */
#define MBX_CMD_MAX	0x80

/* DSP Gateway */
#define MBX_CMD_DSP_WDSND	0x10
#define MBX_CMD_DSP_WDREQ	0x11
#define MBX_CMD_DSP_BKSND	0x20
#define MBX_CMD_DSP_BKREQ	0x21
#define MBX_CMD_DSP_BKYLD	0x23
#define MBX_CMD_DSP_BKSNDP	0x24
#define MBX_CMD_DSP_BKREQP	0x25
#define MBX_CMD_DSP_TCTL	0x30
#define MBX_CMD_DSP_TCTLDATA	0x31
#define MBX_CMD_DSP_POLL	0x32
#define MBX_CMD_DSP_WDT		0x50
#define MBX_CMD_DSP_RUNLEVEL	0x51
#define MBX_CMD_DSP_PM		0x52
#define MBX_CMD_DSP_SUSPEND	0x53
#define MBX_CMD_DSP_KFUNC	0x54
#define MBX_CMD_DSP_TCFG	0x60
#define MBX_CMD_DSP_TADD	0x62
#define MBX_CMD_DSP_TDEL	0x63
#define MBX_CMD_DSP_TSTOP	0x65
#define MBX_CMD_DSP_DSPCFG	0x70
#define MBX_CMD_DSP_REGRW	0x72
#define MBX_CMD_DSP_GETVAR	0x74
#define MBX_CMD_DSP_SETVAR	0x75
#define MBX_CMD_DSP_ERR		0x78
#define MBX_CMD_DSP_DBG		0x79

#endif /* MAILBOX_H */
