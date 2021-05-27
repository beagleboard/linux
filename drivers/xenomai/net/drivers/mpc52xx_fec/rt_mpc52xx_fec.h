/*
 * arch/ppc/5xxx_io/fec.h
 *
 * Header file for the MPC5xxx Fast Ethernet Controller driver
 *
 * Author: Dale Farnsworth <dfarnsworth@mvista.com>
 *
 * Copyright 2003 MontaVista Software
 *
 * 2003 (c) MontaVista, Software, Inc.  This file is licensed under the terms
 * of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

#ifndef __RT_MPC52XX_FEC_H_
#define __RT_MPC52XX_FEC_H_

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/mii.h>
#include <linux/skbuff.h>
#include <asm/mpc5xxx.h>
#include <bestcomm_api.h>

/* Define board specific options */
#define CONFIG_XENO_DRIVERS_NET_USE_MDIO
#define CONFIG_XENO_DRIVERS_NET_FEC_GENERIC_PHY
#define CONFIG_XENO_DRIVERS_NET_FEC_LXT971
#undef CONFIG_XENO_DRIVERS_NET_FEC_DP83847

/* Tunable constants */
#define MPC5xxx_FEC_RECV_BUFFER_SIZE	1518	/* max receive packet size */
#define MPC5xxx_FEC_RECV_BUFFER_SIZE_BC 2048	/* max receive packet size */
#define MPC5xxx_FEC_TBD_NUM		256	/* max transmit packets */
#define MPC5xxx_FEC_RBD_NUM		256	/* max receive packets */

struct mpc5xxx_fec {
	volatile u32 fec_id;			/* FEC + 0x000 */
	volatile u32 ievent;			/* FEC + 0x004 */
	volatile u32 imask;			/* FEC + 0x008 */

	volatile u32 reserved0[1];		/* FEC + 0x00C */
	volatile u32 r_des_active;		/* FEC + 0x010 */
	volatile u32 x_des_active;		/* FEC + 0x014 */
	volatile u32 r_des_active_cl;		/* FEC + 0x018 */
	volatile u32 x_des_active_cl;		/* FEC + 0x01C */
	volatile u32 ivent_set;			/* FEC + 0x020 */
	volatile u32 ecntrl;			/* FEC + 0x024 */

	volatile u32 reserved1[6];		/* FEC + 0x028-03C */
	volatile u32 mii_data;			/* FEC + 0x040 */
	volatile u32 mii_speed;			/* FEC + 0x044 */
	volatile u32 mii_status;		/* FEC + 0x048 */

	volatile u32 reserved2[5];		/* FEC + 0x04C-05C */
	volatile u32 mib_data;			/* FEC + 0x060 */
	volatile u32 mib_control;		/* FEC + 0x064 */

	volatile u32 reserved3[6];		/* FEC + 0x068-7C */
	volatile u32 r_activate;		/* FEC + 0x080 */
	volatile u32 r_cntrl;			/* FEC + 0x084 */
	volatile u32 r_hash;			/* FEC + 0x088 */
	volatile u32 r_data;			/* FEC + 0x08C */
	volatile u32 ar_done;			/* FEC + 0x090 */
	volatile u32 r_test;			/* FEC + 0x094 */
	volatile u32 r_mib;			/* FEC + 0x098 */
	volatile u32 r_da_low;			/* FEC + 0x09C */
	volatile u32 r_da_high;			/* FEC + 0x0A0 */

	volatile u32 reserved4[7];		/* FEC + 0x0A4-0BC */
	volatile u32 x_activate;		/* FEC + 0x0C0 */
	volatile u32 x_cntrl;			/* FEC + 0x0C4 */
	volatile u32 backoff;			/* FEC + 0x0C8 */
	volatile u32 x_data;			/* FEC + 0x0CC */
	volatile u32 x_status;			/* FEC + 0x0D0 */
	volatile u32 x_mib;			/* FEC + 0x0D4 */
	volatile u32 x_test;			/* FEC + 0x0D8 */
	volatile u32 fdxfc_da1;			/* FEC + 0x0DC */
	volatile u32 fdxfc_da2;			/* FEC + 0x0E0 */
	volatile u32 paddr1;			/* FEC + 0x0E4 */
	volatile u32 paddr2;			/* FEC + 0x0E8 */
	volatile u32 op_pause;			/* FEC + 0x0EC */

	volatile u32 reserved5[4];		/* FEC + 0x0F0-0FC */
	volatile u32 instr_reg;			/* FEC + 0x100 */
	volatile u32 context_reg;		/* FEC + 0x104 */
	volatile u32 test_cntrl;		/* FEC + 0x108 */
	volatile u32 acc_reg;			/* FEC + 0x10C */
	volatile u32 ones;			/* FEC + 0x110 */
	volatile u32 zeros;			/* FEC + 0x114 */
	volatile u32 iaddr1;			/* FEC + 0x118 */
	volatile u32 iaddr2;			/* FEC + 0x11C */
	volatile u32 gaddr1;			/* FEC + 0x120 */
	volatile u32 gaddr2;			/* FEC + 0x124 */
	volatile u32 random;			/* FEC + 0x128 */
	volatile u32 rand1;			/* FEC + 0x12C */
	volatile u32 tmp;			/* FEC + 0x130 */

	volatile u32 reserved6[3];		/* FEC + 0x134-13C */
	volatile u32 fifo_id;			/* FEC + 0x140 */
	volatile u32 x_wmrk;			/* FEC + 0x144 */
	volatile u32 fcntrl;			/* FEC + 0x148 */
	volatile u32 r_bound;			/* FEC + 0x14C */
	volatile u32 r_fstart;			/* FEC + 0x150 */
	volatile u32 r_count;			/* FEC + 0x154 */
	volatile u32 r_lag;			/* FEC + 0x158 */
	volatile u32 r_read;			/* FEC + 0x15C */
	volatile u32 r_write;			/* FEC + 0x160 */
	volatile u32 x_count;			/* FEC + 0x164 */
	volatile u32 x_lag;			/* FEC + 0x168 */
	volatile u32 x_retry;			/* FEC + 0x16C */
	volatile u32 x_write;			/* FEC + 0x170 */
	volatile u32 x_read;			/* FEC + 0x174 */

	volatile u32 reserved7[2];		/* FEC + 0x178-17C */
	volatile u32 fm_cntrl;			/* FEC + 0x180 */
	volatile u32 rfifo_data;		/* FEC + 0x184 */
	volatile u32 rfifo_status;		/* FEC + 0x188 */
	volatile u32 rfifo_cntrl;		/* FEC + 0x18C */
	volatile u32 rfifo_lrf_ptr;		/* FEC + 0x190 */
	volatile u32 rfifo_lwf_ptr;		/* FEC + 0x194 */
	volatile u32 rfifo_alarm;		/* FEC + 0x198 */
	volatile u32 rfifo_rdptr;		/* FEC + 0x19C */
	volatile u32 rfifo_wrptr;		/* FEC + 0x1A0 */
	volatile u32 tfifo_data;		/* FEC + 0x1A4 */
	volatile u32 tfifo_status;		/* FEC + 0x1A8 */
	volatile u32 tfifo_cntrl;		/* FEC + 0x1AC */
	volatile u32 tfifo_lrf_ptr;		/* FEC + 0x1B0 */
	volatile u32 tfifo_lwf_ptr;		/* FEC + 0x1B4 */
	volatile u32 tfifo_alarm;		/* FEC + 0x1B8 */
	volatile u32 tfifo_rdptr;		/* FEC + 0x1BC */
	volatile u32 tfifo_wrptr;		/* FEC + 0x1C0 */

	volatile u32 reset_cntrl;		/* FEC + 0x1C4 */
	volatile u32 xmit_fsm;			/* FEC + 0x1C8 */

	volatile u32 reserved8[3];		/* FEC + 0x1CC-1D4 */
	volatile u32 rdes_data0;		/* FEC + 0x1D8 */
	volatile u32 rdes_data1;		/* FEC + 0x1DC */
	volatile u32 r_length;			/* FEC + 0x1E0 */
	volatile u32 x_length;			/* FEC + 0x1E4 */
	volatile u32 x_addr;			/* FEC + 0x1E8 */
	volatile u32 cdes_data;			/* FEC + 0x1EC */
	volatile u32 status;			/* FEC + 0x1F0 */
	volatile u32 dma_control;		/* FEC + 0x1F4 */
	volatile u32 des_cmnd;			/* FEC + 0x1F8 */
	volatile u32 data;			/* FEC + 0x1FC */

	volatile u32 rmon_t_drop;		/* FEC + 0x200 */
	volatile u32 rmon_t_packets;		/* FEC + 0x204 */
	volatile u32 rmon_t_bc_pkt;		/* FEC + 0x208 */
	volatile u32 rmon_t_mc_pkt;		/* FEC + 0x20C */
	volatile u32 rmon_t_crc_align;		/* FEC + 0x210 */
	volatile u32 rmon_t_undersize;		/* FEC + 0x214 */
	volatile u32 rmon_t_oversize;		/* FEC + 0x218 */
	volatile u32 rmon_t_frag;		/* FEC + 0x21C */
	volatile u32 rmon_t_jab;		/* FEC + 0x220 */
	volatile u32 rmon_t_col;		/* FEC + 0x224 */
	volatile u32 rmon_t_p64;		/* FEC + 0x228 */
	volatile u32 rmon_t_p65to127;		/* FEC + 0x22C */
	volatile u32 rmon_t_p128to255;		/* FEC + 0x230 */
	volatile u32 rmon_t_p256to511;		/* FEC + 0x234 */
	volatile u32 rmon_t_p512to1023;		/* FEC + 0x238 */
	volatile u32 rmon_t_p1024to2047;	/* FEC + 0x23C */
	volatile u32 rmon_t_p_gte2048;		/* FEC + 0x240 */
	volatile u32 rmon_t_octets;		/* FEC + 0x244 */
	volatile u32 ieee_t_drop;		/* FEC + 0x248 */
	volatile u32 ieee_t_frame_ok;		/* FEC + 0x24C */
	volatile u32 ieee_t_1col;		/* FEC + 0x250 */
	volatile u32 ieee_t_mcol;		/* FEC + 0x254 */
	volatile u32 ieee_t_def;		/* FEC + 0x258 */
	volatile u32 ieee_t_lcol;		/* FEC + 0x25C */
	volatile u32 ieee_t_excol;		/* FEC + 0x260 */
	volatile u32 ieee_t_macerr;		/* FEC + 0x264 */
	volatile u32 ieee_t_cserr;		/* FEC + 0x268 */
	volatile u32 ieee_t_sqe;		/* FEC + 0x26C */
	volatile u32 t_fdxfc;			/* FEC + 0x270 */
	volatile u32 ieee_t_octets_ok;		/* FEC + 0x274 */

	volatile u32 reserved9[2];		/* FEC + 0x278-27C */
	volatile u32 rmon_r_drop;		/* FEC + 0x280 */
	volatile u32 rmon_r_packets;		/* FEC + 0x284 */
	volatile u32 rmon_r_bc_pkt;		/* FEC + 0x288 */
	volatile u32 rmon_r_mc_pkt;		/* FEC + 0x28C */
	volatile u32 rmon_r_crc_align;		/* FEC + 0x290 */
	volatile u32 rmon_r_undersize;		/* FEC + 0x294 */
	volatile u32 rmon_r_oversize;		/* FEC + 0x298 */
	volatile u32 rmon_r_frag;		/* FEC + 0x29C */
	volatile u32 rmon_r_jab;		/* FEC + 0x2A0 */

	volatile u32 rmon_r_resvd_0;		/* FEC + 0x2A4 */

	volatile u32 rmon_r_p64;		/* FEC + 0x2A8 */
	volatile u32 rmon_r_p65to127;		/* FEC + 0x2AC */
	volatile u32 rmon_r_p128to255;		/* FEC + 0x2B0 */
	volatile u32 rmon_r_p256to511;		/* FEC + 0x2B4 */
	volatile u32 rmon_r_p512to1023;		/* FEC + 0x2B8 */
	volatile u32 rmon_r_p1024to2047;	/* FEC + 0x2BC */
	volatile u32 rmon_r_p_gte2048;		/* FEC + 0x2C0 */
	volatile u32 rmon_r_octets;		/* FEC + 0x2C4 */
	volatile u32 ieee_r_drop;		/* FEC + 0x2C8 */
	volatile u32 ieee_r_frame_ok;		/* FEC + 0x2CC */
	volatile u32 ieee_r_crc;		/* FEC + 0x2D0 */
	volatile u32 ieee_r_align;		/* FEC + 0x2D4 */
	volatile u32 r_macerr;			/* FEC + 0x2D8 */
	volatile u32 r_fdxfc;			/* FEC + 0x2DC */
	volatile u32 ieee_r_octets_ok;		/* FEC + 0x2E0 */

	volatile u32 reserved10[6];		/* FEC + 0x2E4-2FC */

	volatile u32 reserved11[64];		/* FEC + 0x300-3FF */
};

#define MPC5xxx_FEC_MIB_DISABLE			0x80000000

#define MPC5xxx_FEC_IEVENT_HBERR		0x80000000
#define MPC5xxx_FEC_IEVENT_BABR			0x40000000
#define MPC5xxx_FEC_IEVENT_BABT			0x20000000
#define MPC5xxx_FEC_IEVENT_GRA			0x10000000
#define MPC5xxx_FEC_IEVENT_TFINT		0x08000000
#define MPC5xxx_FEC_IEVENT_MII			0x00800000
#define MPC5xxx_FEC_IEVENT_LATE_COL		0x00200000
#define MPC5xxx_FEC_IEVENT_COL_RETRY_LIM	0x00100000
#define MPC5xxx_FEC_IEVENT_XFIFO_UN		0x00080000
#define MPC5xxx_FEC_IEVENT_XFIFO_ERROR		0x00040000
#define MPC5xxx_FEC_IEVENT_RFIFO_ERROR		0x00020000

#define MPC5xxx_FEC_IMASK_HBERR			0x80000000
#define MPC5xxx_FEC_IMASK_BABR			0x40000000
#define MPC5xxx_FEC_IMASK_BABT			0x20000000
#define MPC5xxx_FEC_IMASK_GRA			0x10000000
#define MPC5xxx_FEC_IMASK_MII			0x00800000
#define MPC5xxx_FEC_IMASK_LATE_COL		0x00200000
#define MPC5xxx_FEC_IMASK_COL_RETRY_LIM		0x00100000
#define MPC5xxx_FEC_IMASK_XFIFO_UN		0x00080000
#define MPC5xxx_FEC_IMASK_XFIFO_ERROR		0x00040000
#define MPC5xxx_FEC_IMASK_RFIFO_ERROR		0x00020000

#define MPC5xxx_FEC_RCNTRL_MAX_FL_SHIFT		16
#define MPC5xxx_FEC_RCNTRL_LOOP			0x01
#define MPC5xxx_FEC_RCNTRL_DRT			0x02
#define MPC5xxx_FEC_RCNTRL_MII_MODE		0x04
#define MPC5xxx_FEC_RCNTRL_PROM			0x08
#define MPC5xxx_FEC_RCNTRL_BC_REJ		0x10
#define MPC5xxx_FEC_RCNTRL_FCE			0x20

#define MPC5xxx_FEC_TCNTRL_GTS			0x00000001
#define MPC5xxx_FEC_TCNTRL_HBC			0x00000002
#define MPC5xxx_FEC_TCNTRL_FDEN			0x00000004
#define MPC5xxx_FEC_TCNTRL_TFC_PAUSE		0x00000008
#define MPC5xxx_FEC_TCNTRL_RFC_PAUSE		0x00000010

#define MPC5xxx_FEC_ECNTRL_RESET		0x00000001
#define MPC5xxx_FEC_ECNTRL_ETHER_EN		0x00000002

#define MPC5xxx_FEC_RESET_DELAY			50 /* uS */


/* Receive & Transmit Buffer Descriptor definitions */
struct mpc5xxx_fec_bd {
	volatile u32 status;
	volatile u32 data;
};

/* Receive data buffer format */
struct mpc5xxx_rbuf {
	u8 data[MPC5xxx_FEC_RECV_BUFFER_SIZE_BC];
};

struct fec_queue {
	volatile struct mpc5xxx_fec_bd *bd_base;
	struct rtskb **skb_base;
	u16 last_index;
	u16 start_index;
	u16 finish_index;
};

#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO
#define MII_ADVERTISE_HALF	(ADVERTISE_100HALF | ADVERTISE_10HALF | \
				 ADVERTISE_CSMA)

#define MII_ADVERTISE_ALL	(ADVERTISE_100FULL | ADVERTISE_10FULL | \
				 MII_ADVERTISE_HALF)
#ifdef PHY_INTERRUPT
#define MII_ADVERTISE_DEFAULT   MII_ADVERTISE_ALL
#else
#define MII_ADVERTISE_DEFAULT   MII_ADVERTISE_HALF
#endif

typedef struct {
	uint mii_data;
	void (*funct)(uint mii_reg, struct rtnet_device *dev, uint data);
} phy_cmd_t;

typedef struct {
	uint id;
	char *name;

	const phy_cmd_t *config;
	const phy_cmd_t *startup;
	const phy_cmd_t *ack_int;
	const phy_cmd_t *shutdown;
} phy_info_t;
#endif	/* CONFIG_XENO_DRIVERS_NET_USE_MDIO */

struct mpc5xxx_fec_priv {
	int full_duplex;
	int tx_full;
	int r_tasknum;
	int t_tasknum;
	int r_irq;
	int t_irq;
	rtdm_irq_t irq_handle;
	rtdm_irq_t r_irq_handle;
	rtdm_irq_t t_irq_handle;
	u32 last_transmit_time;
	u32 last_receive_time;
	struct mpc5xxx_fec *fec;
	struct mpc5xxx_sram_fec *sram;
	struct mpc5xxx_gpio *gpio;
	struct mpc5xxx_sdma *sdma;
	struct fec_queue r_queue;
	struct rtskb *rskb[MPC5xxx_FEC_RBD_NUM];
	struct fec_queue t_queue;
	struct rtskb *tskb[MPC5xxx_FEC_TBD_NUM];
	rtdm_lock_t lock;
	unsigned long open_time;
	struct net_device_stats stats;
#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO
	uint phy_id;
	uint phy_id_done;
	uint phy_status;
	uint phy_speed;
	phy_info_t *phy;
	struct tq_struct phy_task;
	volatile uint sequence_done;
	uint link;
	uint phy_addr;

	struct tq_struct link_up_task;
	int duplex_change;
	int link_up;

	struct timer_list phy_timer_list;
	u16 old_status;
#endif	/* CONFIG_XENO_DRIVERS_NET_USE_MDIO */
};

struct mpc5xxx_sram_fec {
	volatile struct mpc5xxx_fec_bd tbd[MPC5xxx_FEC_TBD_NUM];
	volatile struct mpc5xxx_fec_bd rbd[MPC5xxx_FEC_RBD_NUM];
};

#define MPC5xxx_FEC_RBD_READY	0x40000000
#define MPC5xxx_FEC_RBD_RFD	0x08000000	/* receive frame done */

#define MPC5xxx_FEC_RBD_INIT	MPC5xxx_FEC_RBD_READY

#define MPC5xxx_FEC_TBD_READY	0x40000000
#define MPC5xxx_FEC_TBD_TFD	0x08000000	/* transmit frame done */
#define MPC5xxx_FEC_TBD_INT	0x04000000	/* Interrupt */

#define MPC5xxx_FEC_TBD_INIT	(MPC5xxx_FEC_TBD_INT | MPC5xxx_FEC_TBD_TFD | \
				 MPC5xxx_FEC_TBD_READY)



/* MII-related definitions */
#define MPC5xxx_FEC_MII_DATA_ST		0x40000000	/* Start frame */
#define MPC5xxx_FEC_MII_DATA_OP_RD	0x20000000	/* Perform read */
#define MPC5xxx_FEC_MII_DATA_OP_WR	0x10000000	/* Perform write */
#define MPC5xxx_FEC_MII_DATA_PA_MSK	0x0f800000	/* PHY Address mask */
#define MPC5xxx_FEC_MII_DATA_RA_MSK	0x007c0000	/* PHY Register mask */
#define MPC5xxx_FEC_MII_DATA_TA		0x00020000	/* Turnaround */
#define MPC5xxx_FEC_MII_DATA_DATAMSK	0x00000fff	/* PHY data mask */

#define MPC5xxx_FEC_MII_DATA_RA_SHIFT	0x12		/* MII reg addr bits */
#define MPC5xxx_FEC_MII_DATA_PA_SHIFT	0x17		/* MII PHY addr bits */

#define MPC5xxx_FEC_MII_SPEED		(5 * 2)

const char mpc5xxx_fec_name[] = "eth0";

struct mibCounters {
	unsigned int byteReceived;
	unsigned int byteSent;
	unsigned int framesReceived;
	unsigned int framesSent;
	unsigned int totalByteReceived;
	unsigned int totalFramesReceived;
	unsigned int broadcastFramesReceived;
	unsigned int multicastFramesReceived;
	unsigned int cRCError;
	unsigned int oversizeFrames;
	unsigned int fragments;
	unsigned int jabber;
	unsigned int collision;
	unsigned int lateCollision;
	unsigned int frames64;
	unsigned int frames65_127;
	unsigned int frames128_255;
	unsigned int frames256_511;
	unsigned int frames512_1023;
	unsigned int frames1024_MaxSize;
	unsigned int macRxError;
	unsigned int droppedFrames;
	unsigned int outMulticastFrames;
	unsigned int outBroadcastFrames;
	unsigned int undersizeFrames;
};

#define MPC5xxx_FEC_WATCHDOG_TIMEOUT  ((400*HZ)/1000)


#define MPC5xxx_FEC_FRAME_LAST		0x08000000	/* Last */
#define MPC5xxx_FEC_FRAME_M		0x01000000	/* M? */
#define MPC5xxx_FEC_FRAME_BC		0x00800000	/* Broadcast */
#define MPC5xxx_FEC_FRAME_MC		0x00400000	/* Multicast */
#define MPC5xxx_FEC_FRAME_LG		0x00200000	/* Length error */
#define MPC5xxx_FEC_FRAME_NO		0x00100000	/* Non-octet aligned frame error */
#define MPC5xxx_FEC_FRAME_CR		0x00040000	/* CRC frame error */
#define MPC5xxx_FEC_FRAME_OV		0x00020000	/* Overrun error */
#define MPC5xxx_FEC_FRAME_TR		0x00010000	/* Truncated error */



#endif	/* __RT_MPC52XX_FEC_H_ */
