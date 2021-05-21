/* rt2x00.h
 *
 * Copyright (C) 2004 - 2005 rt2x00-2.0.0-b3 SourceForge Project
 *	                     <http://rt2x00.serialmonkey.com>
 *               2006        rtnet adaption by Daniel Gregorek 
 *                           <dxg@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*
  Module: rt2x00
  Abstract: rt2x00 global information.
  Supported chipsets: RT2560
*/

#ifndef RT2X00_H
#define RT2X00_H

#include <linux/netdevice.h>
#include <linux/wireless.h>

#include <rtnet_port.h>
#include <rtwlan.h>

#define MAX_UNITS 2

/*
 * Module information.
 */
#define DRV_NAME "rt2x00"
#define DRV_VERSION "0.1"
#define DRV_AUTHOR "Daniel Gregorek <dxg@gmx.de>"
//#define CONFIG_RT2X00_DEBUG

/*
 * Debug defines.
 * The debug variable will be exported by the device specific module.
 * For this reason this variable must be set to extern to make it accessible
 * to the core module as well.
 */
#ifdef CONFIG_RT2X00_DEBUG
extern int rt2x00_debug_level;
#define DEBUG_PRINTK(__message...)                                             \
	do {                                                                   \
		rtdm_printk(__message);                                        \
	} while (0)
#else /* CONFIG_RT2X00_DEBUG */
#define DEBUG_PRINTK(__message...)                                             \
	do {                                                                   \
	} while (0)
#endif /* CONFIG_RT2X00_DEBUG */

/*
 * Various debug levels.
 * PANIC and ERROR indicates serious problems within the module,
 * these should never be ignored and thus we will always print the message.
 */
#define PANIC(__message, __args...)                                            \
	rtdm_printk(KERN_PANIC DRV_NAME "->%s: Panic - " __message,            \
		    __FUNCTION__, ##__args);
#define ERROR(__message, __args...)                                            \
	rtdm_printk(KERN_ERR DRV_NAME "->%s: Error - " __message,              \
		    __FUNCTION__, ##__args);
#define WARNING(__message, __args...)                                          \
	rtdm_printk(KERN_WARNING DRV_NAME "->%s: Warning - " __message,        \
		    __FUNCTION__, ##__args);
#define NOTICE(__message, __args...)                                           \
	rtdm_printk(KERN_NOTICE DRV_NAME "->%s: Notice - " __message,          \
		    __FUNCTION__, ##__args);
#define INFO(__message, __args...)                                             \
	rtdm_printk(KERN_INFO DRV_NAME "->%s: Info - " __message,              \
		    __FUNCTION__, ##__args);
#define DEBUG(__message, __args...)                                            \
	DEBUG_PRINTK(KERN_DEBUG DRV_NAME "->%s: Debug - " __message,           \
		     __FUNCTION__, ##__args);

/*
 * RT2x00 ring types.
 */

/*
 * Ring names.
 */
#define RING_RX 0x01 /* Ring used for receiving packets. */
#define RING_TX 0x02 /* Ring used for transmitting normal packets. */

/*
 * Ring sizes.
 */
#define DATA_FRAME_SIZE 2432
#define MGMT_FRAME_SIZE 256

/*
 * RT2x00 xmit flags.
 */
#define XMIT_IFS_SIFS 0x0001
#define XMIT_IFS_BACKOFF 0x0002
#define XMIT_IFS_NEW_BACKOFF 0x0004
#define XMIT_IFS_NONE 0x0008
#define XMIT_NEW_SEQUENCE 0x0010
#define XMIT_ACK 0x0020
#define XMIT_TIMESTAMP 0x0040
#define XMIT_RTS 0x0080
#define XMIT_OFDM 0x0100
#define XMIT_LONG_RETRY 0x0200
#define XMIT_MORE_FRAGS 0x0400
#define XMIT_SHORT_PREAMBLE 0x0800
#define XMIT_START 0x1000

/*
 * RT2x00 Statistics flags.
 */
#define STATS_TX_RESULT 0x01
#define STATS_TX_RETRY_COUNT 0x02
#define STATS_RX_CRC 0x10
#define STATS_RX_PHYSICAL 0x20
#define STATS_RX_QUALITY 0x40
#define STATS_RX_DROP 0x80

/*
 * TX result flags.
 */
#define TX_SUCCESS 0
#define TX_SUCCESS_RETRY 1
#define TX_FAIL_RETRY 2
#define TX_FAIL_INVALID 3
#define TX_FAIL_OTHER 4

/*
 * Channel type defines.
 */
#define CHANNEL_OFDM 0x01
#define CHANNEL_UNII_LOW 0x02
#define CHANNEL_HIPERLAN2 0x04
#define CHANNEL_UNII_HIGH 0x08

#define CHANNEL_OFDM_MIN 1
#define CHANNEL_OFDM_MAX 14
#define CHANNEL_UNII_LOW_MIN 36
#define CHANNEL_UNII_LOW_MAX 64
#define CHANNEL_HIPERLAN2_MIN 100
#define CHANNEL_HIPERLAN2_MAX 140
#define CHANNEL_UNII_HIGH_MIN 149
#define CHANNEL_UNII_HIGH_MAX 161

/*
 * Device 802.11abg capabilities.
 */
static struct _rt2x00_capabilities {
	u8 txpower[6];
	u8 bitrate[12];
} __attribute__ ((packed)) capabilities = {
    /*
     * tx-power.
     */
    .txpower = {
          3, 12, 25, 50, 75, 100,
      },

    /*
     * Bitrates
     */
    .bitrate = {
         2, 4, 11, 22,						/* CCK. */
         12, 18, 24, 36, 48, 72, 96, 108,			/* OFDM. */
     },
};

struct _rt2x00_config {
	u8 config_flags;
#define CONFIG_DROP_BCAST 0x0001
#define CONFIG_DROP_MCAST 0x0002
#define CONFIG_AUTORESP 0x0004

	u8 antenna_tx;
	u8 antenna_rx;

	u8 bssid[ETH_ALEN];
	u8 short_retry;
	u8 long_retry;

	u8 channel;
	u8 bitrate; /* 0.5Mbit/sec */
	u8 txpower; /* % */

	u8 bbpsens;

	/*
     * LED status
     */
	u8 led_status;

	u16 __pad2; /* For alignment only. */

	/*
     * Duration values in us.
     */
	u8 plcp;
	u8 sifs;
	u8 slot_time;

	/*
     * Configuration values that have to be updated to device.
     */
	u16 update_flags;
#define UPDATE_ALL_CONFIG 0xffff
#define UPDATE_BSSID 0x0001
#define UPDATE_PACKET_FILTER 0x0002
#define UPDATE_CHANNEL 0x0004
#define UPDATE_BITRATE 0x0008
#define UPDATE_RETRY 0x0010
#define UPDATE_TXPOWER 0x0020
#define UPDATE_ANTENNA 0x0040
#define UPDATE_DURATION 0x0080
#define UPDATE_PREAMBLE 0x0100
#define UPDATE_AUTORESP 0x0200
#define UPDATE_LED_STATUS 0x0400
#define UPDATE_BBPSENS 0x0800

} __attribute__((packed));

struct _rt2x00_core {
	/*
     * RT2x00 device status flags (atomic read/write access).
     */
	unsigned long flags;

#define DEVICE_ENABLED 0 /* Device has been opened. */
#define DEVICE_AWAKE 1 /* Device is not suspended. */
#define DEVICE_RADIO_ON 2 /* Device antenna is enabled. */
#define DEVICE_CONFIG_UPDATE 3 /* Device is updating configuration. */

	/*
     * Device handler.
     */
	struct _rt2x00_dev_handler *handler;

	/*
     * RTnet device we belong to.
     */
	struct rtnet_device *rtnet_dev;

	/*
     * RTwlan stack structure.
     */
	struct rtwlan_device *rtwlan_dev;

	/*
     * Device configuration.
     */
	struct _rt2x00_config config;

	void *priv;

} __attribute__((packed));

/*
 * Device specific handlers.
 */
struct _rt2x00_dev_handler {
	/*
     * Device specific module.
     */
	struct module *dev_module;

	/*
     * Initialization handlers.
     */
	int (*dev_probe)(struct _rt2x00_core *core, void *priv);
	int (*dev_remove)(struct _rt2x00_core *core);

	/*
     * Radio control.
     */
	int (*dev_radio_on)(struct _rt2x00_core *core);
	int (*dev_radio_off)(struct _rt2x00_core *core);

	/*
     * Configuration handlers.
     */
	int (*dev_update_config)(struct _rt2x00_core *core, u16 update_flags);

	/*
     * xmit handler.
     */
	int (*dev_xmit_packet)(struct _rt2x00_core *core, struct rtskb *rtskb,
			       u16 rate, u16 xmit_flags);

	/*
     * Handler for direct access to register from core.
     */
	int (*dev_register_access)(struct _rt2x00_core *core, int request,
				   u32 address, u32 *value);

} __attribute__((packed));

static inline void *rt2x00_priv(const struct _rt2x00_core *core)
{
	return core->priv;
}

/*
 * Duration calculations
 * The rate variable passed is: 2 * real_rate (in Mb/s).
 * Therefore length has to be multiplied with 8 to convert bytes to bits and  mulltiply the length
 * with 2 to compensate for the difference between real_rate and the rate variable.
 */
#define ACK_SIZE 14
#define IEEE80211_HEADER 24

static inline u16 get_duration(const unsigned int size, const u8 rate)
{
	return ((size * 8 * 2) / rate);
}

static inline u16 get_duration_res(const unsigned int size, const u8 rate)
{
	return ((size * 8 * 2) % rate);
}

static inline u16 get_preamble(const struct _rt2x00_config *config)
{
	return 144;
}

/*
 * Register handlers.
 * We store the position of a register field inside a field structure,
 * This will simplify the process of setting and reading a certain field
 * inside the register.
 */
struct _rt2x00_field16 {
	u16 bit_offset;
	u16 bit_mask;
} __attribute__((packed));

struct _rt2x00_field32 {
	u32 bit_offset;
	u32 bit_mask;
} __attribute__((packed));

#define FIELD16(__offset, __mask)                                              \
	((struct _rt2x00_field16){ (__offset), (__mask) })
#define FIELD32(__offset, __mask)                                              \
	((struct _rt2x00_field32){ (__offset), (__mask) })

static inline void rt2x00_set_field32(u32 *reg,
				      const struct _rt2x00_field32 field,
				      const u32 value)
{
	*reg &= cpu_to_le32(~(field.bit_mask));
	*reg |= cpu_to_le32((value << field.bit_offset) & field.bit_mask);
}

static inline void rt2x00_set_field32_nb(u32 *reg,
					 const struct _rt2x00_field32 field,
					 const u32 value)
{
	*reg &= ~(field.bit_mask);
	*reg |= (value << field.bit_offset) & field.bit_mask;
}

static inline u32 rt2x00_get_field32(const u32 reg,
				     const struct _rt2x00_field32 field)
{
	return (le32_to_cpu(reg) & field.bit_mask) >> field.bit_offset;
}

static inline u32 rt2x00_get_field32_nb(const u32 reg,
					const struct _rt2x00_field32 field)
{
	return (reg & field.bit_mask) >> field.bit_offset;
}

static inline void rt2x00_set_field16(u16 *reg,
				      const struct _rt2x00_field16 field,
				      const u16 value)
{
	*reg &= cpu_to_le16(~(field.bit_mask));
	*reg |= cpu_to_le16((value << field.bit_offset) & field.bit_mask);
}

static inline void rt2x00_set_field16_nb(u16 *reg,
					 const struct _rt2x00_field16 field,
					 const u16 value)
{
	*reg &= ~(field.bit_mask);
	*reg |= (value << field.bit_offset) & field.bit_mask;
}

static inline u16 rt2x00_get_field16(const u16 reg,
				     const struct _rt2x00_field16 field)
{
	return (le16_to_cpu(reg) & field.bit_mask) >> field.bit_offset;
}

static inline u16 rt2x00_get_field16_nb(const u16 reg,
					const struct _rt2x00_field16 field)
{
	return (reg & field.bit_mask) >> field.bit_offset;
}

/*
 * rf register sructure for channel selection.
 */
struct _rf_channel {
	u32 rf1;
	u32 rf2;
	u32 rf3;
	u32 rf4;
} __attribute__((packed));

/*
 * Chipset identification
 * The chipset on the device is composed of a RT and RF chip.
 * The chipset combination is important for determining device capabilities.
 */
struct _rt2x00_chip {
	u16 rt;
	u16 rf;
} __attribute__((packed));

/*
 * Set chipset data.
 * Some rf values for RT2400 devices are equal to rf values for RT2500 devices.
 * To prevent problems, all rf values will be masked to clearly seperate each chipset.
 */
static inline void set_chip(struct _rt2x00_chip *chipset, const u16 rt,
			    const u16 rf)
{
	INFO("Chipset detected - rt: %04x, rf: %04x.\n", rt, rf);

	chipset->rt = rt;
	chipset->rf = rf | (chipset->rt & 0xff00);
}

static inline char rt2x00_rt(const struct _rt2x00_chip *chipset, const u16 chip)
{
	return (chipset->rt == chip);
}

static inline char rt2x00_rf(const struct _rt2x00_chip *chipset, const u16 chip)
{
	return (chipset->rf == chip);
}

static inline u16 rt2x00_get_rf(const struct _rt2x00_chip *chipset)
{
	return chipset->rf;
}

/*
 * _data_ring
 * Data rings are used by the device to send and receive packets.
 * The data_addr is the base address of the data memory.
 * Device specifice information is pointed to by the priv pointer.
 * The index values may only be changed with the functions ring_index_inc()
 * and ring_index_done_inc().
 */
struct _data_ring {
	/*
     * Base address of packet ring.
     */
	dma_addr_t data_dma;
	void *data_addr;

	/*
     * Private device specific data.
     */
	void *priv;
	struct _rt2x00_core *core;

	/*
     * Current index values.
     */
	u8 index;
	u8 index_done;

	/*
     * Ring type set with RING_* define.
     */
	u8 ring_type;

	/*
     * Number of entries in this ring.
     */
	u8 max_entries;

	/*
     * Size of packet and descriptor in bytes.
     */
	u16 entry_size;
	u16 desc_size;

	/*
     * Total allocated memory size.
     */
	u32 mem_size;
} __attribute__((packed));

/*
 * Number of entries in a packet ring.
 */
#define RX_ENTRIES 8
#define TX_ENTRIES 8
#define ATIM_ENTRIES 1
#define PRIO_ENTRIES 2
#define BEACON_ENTRIES 1

/*
 * Initialization and cleanup routines.
 */
static inline void rt2x00_init_ring(struct _rt2x00_core *core,
				    struct _data_ring *ring, const u8 ring_type,
				    const u16 max_entries, const u16 entry_size,
				    const u16 desc_size)
{
	ring->core = core;
	ring->index = 0;
	ring->index_done = 0;
	ring->ring_type = ring_type;
	ring->max_entries = max_entries;
	ring->entry_size = entry_size;
	ring->desc_size = desc_size;
	ring->mem_size =
		ring->max_entries * (ring->desc_size + ring->entry_size);
}

static inline void rt2x00_deinit_ring(struct _data_ring *ring)
{
	ring->core = NULL;
	ring->index = 0;
	ring->index_done = 0;
	ring->ring_type = 0;
	ring->max_entries = 0;
	ring->entry_size = 0;
	ring->desc_size = 0;
	ring->mem_size = 0;
}

/*
 * Ring index manipulation functions.
 */
static inline void rt2x00_ring_index_inc(struct _data_ring *ring)
{
	ring->index = (++ring->index < ring->max_entries) ? ring->index : 0;
}

static inline void rt2x00_ring_index_done_inc(struct _data_ring *ring)
{
	ring->index_done =
		(++ring->index_done < ring->max_entries) ? ring->index_done : 0;
}

static inline void rt2x00_ring_clear_index(struct _data_ring *ring)
{
	ring->index = 0;
	ring->index_done = 0;
}

static inline u8 rt2x00_ring_empty(struct _data_ring *ring)
{
	return ring->index_done == ring->index;
}

static inline u8 rt2x00_ring_free_entries(struct _data_ring *ring)
{
	if (ring->index >= ring->index_done)
		return ring->max_entries - (ring->index - ring->index_done);
	else
		return ring->index_done - ring->index;
}

/*
 * Return PLCP value matching the rate.
 * PLCP values according to ieee802.11a-1999 p.14.
 */
static inline u8 rt2x00_get_plcp(const u8 rate)
{
	u8 counter = 0x00;
	u8 plcp[12] = {
		0x00, 0x01, 0x02, 0x03, /* CCK. */
		0x0b, 0x0f, 0x0a, 0x0e, 0x09, 0x0d, 0x08, 0x0c, /* OFDM. */
	};

	for (; counter < 12; counter++) {
		if (capabilities.bitrate[counter] == rate)
			return plcp[counter];
	}

	return 0xff;
}

#define OFDM_CHANNEL(__channel)                                                \
	((__channel) >= CHANNEL_OFDM_MIN && (__channel) <= CHANNEL_OFDM_MAX)
#define UNII_LOW_CHANNEL(__channel)                                            \
	((__channel) >= CHANNEL_UNII_LOW_MIN &&                                \
	 (__channel) <= CHANNEL_UNII_LOW_MAX)
#define HIPERLAN2_CHANNEL(__channel)                                           \
	((__channel) >= CHANNEL_HIPERLAN2_MIN &&                               \
	 (__channel) <= CHANNEL_HIPERLAN2_MAX)
#define UNII_HIGH_CHANNEL(__channel)                                           \
	((__channel) >= CHANNEL_UNII_HIGH_MIN &&                               \
	 (__channel) <= CHANNEL_UNII_HIGH_MAX)

/*
 * Return the index value of the channel starting from the first channel of the range.
 * Where range can be OFDM, UNII (low), HiperLAN2 or UNII (high).
 */
static inline int rt2x00_get_channel_index(const u8 channel)
{
	if (OFDM_CHANNEL(channel))
		return (channel - 1);

	if (channel % 4)
		return -EINVAL;

	if (UNII_LOW_CHANNEL(channel))
		return ((channel - CHANNEL_UNII_LOW_MIN) / 4);
	else if (HIPERLAN2_CHANNEL(channel))
		return ((channel - CHANNEL_HIPERLAN2_MIN) / 4);
	else if (UNII_HIGH_CHANNEL(channel))
		return ((channel - CHANNEL_UNII_HIGH_MIN) / 4);
	return -EINVAL;
}

/*
 * RT2x00 core module functions that can be used in the device specific modules.
 */
extern struct rtnet_device *
rt2x00_core_probe(struct _rt2x00_dev_handler *handler, void *priv,
		  u32 sizeof_dev);
extern void rt2x00_core_remove(struct rtnet_device *rtnet_dev);

#endif
