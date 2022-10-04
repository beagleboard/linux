#ifndef _DSMART_CARD_INTERFACE_H
#define _DSMART_CARD_INTERFACE_H

#define DSMART_CARD_OK			0
#define DSMART_CARD_E_ACCESS		1
#define DSMART_CARD_E_DATA_TIMEOUT	2
#define DSMART_CARD_E_NOCARD		3
#define DSMART_CARD_E_NOACT		4
#define DSMART_CARD_E_REMOVED		5
#define DSMART_CARD_E_NO_RX_EV		6
#define DSMART_CARD_E_NO_TX_EV		7
#define DSMART_CARD_E_NO_CRD_EV		8
#define DSMART_CARD_E_ACT_TIMEOUT	9
#define DSMART_CARD_E_DATA_RCV_FAILED	10
#define DSMART_CARD_E_ACTIVATE_FAILED	11
#define DSMART_CARD_E_TX_FULL		12
#define DSMART_CARD_E_PAR_ERR		13
#define DSMART_CARD_E_CRC_ERR		14
#define DSMART_CARD_E_REP_ERR		15
#define DSMART_CARD_E_CWT_TIM		16
#define DSMART_CARD_E_RX_OVER		17
#define DSMART_CARD_STATE_ERR_EVENT	18

#define CARD_PROTOCOL_T0	1
#define	CARD_PROTOCOL_T1	2

struct dsmart_card_atr {
	unsigned char atr_buffer[64];
	unsigned int len;	/* length of ATR received */
	int errval;
};

struct dsmart_card_rcv {
	unsigned char rcv_buffer[256];
	int rcv_length;
	int time_out;
	int errval;
};

struct dsmart_card_xmt {
	unsigned char xmt_buffer[256];
	int xmt_length;
	int time_out;
	int errval;
};

struct dsmart_card_timing {
	unsigned int wwt;
	unsigned int cwt;
	unsigned int bwt;
	unsigned int bgt;
	unsigned int egt;
};

struct dsmart_card_baud {
	unsigned char di;
	unsigned char fi;
};

/* ioctl encodings */
#define DSMART_CARD_BASE			0xc0
#define DSMART_CARD_IOCTL_SET_PROTOCOL		_IOR(DSMART_CARD_BASE, 1, int)
#define DSMART_CARD_IOCTL_DEACTIVATE		_IOR(DSMART_CARD_BASE, 2, int)
#define DSMART_CARD_IOCTL_COLD_RESET		_IOR(DSMART_CARD_BASE, 3, int)
#define DSMART_CARD_IOCTL_WARM_RESET		_IOR(DSMART_CARD_BASE, 4, int)
#define DSMART_CARD_IOCTL_SET_TIMING		_IOR(DSMART_CARD_BASE, 5, int)
#define DSMART_CARD_IOCTL_SET_BAUD		_IOR(DSMART_CARD_BASE, 6, int)
#define DSMART_CARD_IOCTL_SET_RX_THRESHOLD	_IOR(DSMART_CARD_BASE, 7, int)
#define DSMART_CARD_IOCTL_SET_TX_THRESHOLD	_IOR(DSMART_CARD_BASE, 8, int)
#define DSMART_CARD_IOCTL_XMT			_IOR(DSMART_CARD_BASE, 9, int)
#define DSMART_CARD_IOCTL_RCV			_IOR(DSMART_CARD_BASE, 10, int)
#define DSMART_CARD_IOCTL_ATR_RCV		_IOR(DSMART_CARD_BASE, 11, int)

#endif
