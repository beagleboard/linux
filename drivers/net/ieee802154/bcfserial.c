
/*
 *  bcfserial.c - Serial interface driver for BeagleConnect Freedom.
 */
#include <linux/circ_buf.h>
#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/serdev.h>
#include <linux/sched.h>
#include <linux/skbuff.h>

#include <net/cfg802154.h>
#include <net/mac802154.h>

#define DEBUG

#define BCFSERIAL_DRV_VERSION "0.1.0"
#define BCFSERIAL_DRV_NAME "bcfserial"

#define HDLC_FRAME	0x7E
#define HDLC_ESC	0x7D
#define HDLC_XOR	0x20

#define ADDRESS_CTRL	0x01
#define ADDRESS_WPAN	0x03
#define ADDRESS_CDC	0x05
#define ADDRESS_HW	0x41

#define MAX_PSDU		127
#define MAX_RX_XFER		(1 + MAX_PSDU + 2 + 1)	/* PHR+PSDU+CRC+LQI */
#define HDLC_HEADER_LEN 	2
#define PACKET_HEADER_LEN	8
#define CRC_LEN 		2
#define RX_HDLC_PAYLOAD		140
#define MAX_TX_HDLC		(1 + HDLC_HEADER_LEN + PACKET_HEADER_LEN + MAX_RX_XFER + CRC_LEN + 1)
#define MAX_RX_HDLC		(1 + RX_HDLC_PAYLOAD + CRC_LEN)
#define TX_CIRC_BUF_SIZE	1024

enum bcfserial_requests {
	RESET,
	TX,
	XMIT_ASYNC,
	ED,
	SET_CHANNEL,
	START,
	STOP,
	SET_SHORT_ADDR,
	SET_PAN_ID,
	SET_IEEE_ADDR,
	SET_TXPOWER,
	SET_CCA_MODE,
	SET_CCA_ED_LEVEL,
	SET_CSMA_PARAMS,
	SET_LBT,
	SET_FRAME_RETRIES,
	SET_PROMISCUOUS_MODE,
	GET_EXTENDED_ADDR,
	GET_SUPPORTED_CHANNELS,
};

struct bcfserial {
	struct serdev_device *serdev;
	struct ieee802154_hw *hw;

	struct work_struct tx_work;
	spinlock_t tx_producer_lock;
	spinlock_t tx_consumer_lock;
	struct circ_buf tx_circ_buf;
	struct sk_buff *tx_skb;
	u16 tx_crc;
	u8 tx_ack_seq;		/* current TX ACK sequence number */

	size_t response_size;
	u8 *response_buffer;

	u8 rx_in_esc;
	u8 rx_address;
	u16 rx_offset;
	u8 *rx_buffer;
};

// RX Packet Format:
// - WPAN RX PACKET:	[len] payload [lqi]
// - WPAN TX ACK:	[seq]
// - WPAN CAPABILITIES:	supported_channels_mask(4)
// - CDC:		printable_chars

static void bcfserial_serdev_write_locked(struct bcfserial *bcfserial)
{
	//must be locked already
	int head = smp_load_acquire(&bcfserial->tx_circ_buf.head);
	int tail = bcfserial->tx_circ_buf.tail;
	int count = CIRC_CNT_TO_END(head, tail, TX_CIRC_BUF_SIZE);
	int written;

	if (count >= 1) {
		written = serdev_device_write_buf(bcfserial->serdev, &bcfserial->tx_circ_buf.buf[tail], count);

		smp_store_release(&(bcfserial->tx_circ_buf.tail), (tail + written) & (TX_CIRC_BUF_SIZE - 1));
	}
}

static void bcfserial_append(struct bcfserial *bcfserial, u8 value)
{
	//must be locked already
	int head = bcfserial->tx_circ_buf.head;

	while(true)
	{
		int tail = READ_ONCE(bcfserial->tx_circ_buf.tail);

		if (CIRC_SPACE(head, tail, TX_CIRC_BUF_SIZE) >= 1) {

			bcfserial->tx_circ_buf.buf[head] = value;

			smp_store_release(&(bcfserial->tx_circ_buf.head),
					  (head + 1) & (TX_CIRC_BUF_SIZE - 1));
			return;
		} else {
			dev_dbg(&bcfserial->serdev->dev, "Tx circ buf full\n");
			usleep_range(3000,5000);
		}
	}
}

static void bcfserial_append_tx_frame(struct bcfserial *bcfserial)
{
	bcfserial->tx_crc = 0xFFFF;
	bcfserial_append(bcfserial, HDLC_FRAME);
}

static void bcfserial_append_escaped(struct bcfserial *bcfserial, u8 value)
{
        if (value == HDLC_FRAME || value == HDLC_ESC) {
                bcfserial_append(bcfserial, HDLC_ESC);
                value ^= HDLC_XOR;
        }
        bcfserial_append(bcfserial, value);
}

static void bcfserial_append_tx_u8(struct bcfserial *bcfserial, u8 value)
{
	bcfserial->tx_crc = crc_ccitt(bcfserial->tx_crc, &value, 1);
	bcfserial_append_escaped(bcfserial, value);
}

static void bcfserial_append_tx_buffer(struct bcfserial *bcfserial, const void *buffer, size_t len)
{
	size_t i;
	for (i=0; i<len; i++) {
		bcfserial_append_tx_u8(bcfserial, ((u8*)buffer)[i]);
	}
}

static void bcfserial_append_tx_le16(struct bcfserial *bcfserial, u16 value)
{
	value = cpu_to_le16(value);
	bcfserial_append_tx_buffer(bcfserial, (u8 *)&value, sizeof(u16));
}

static void bcfserial_append_tx_crc(struct bcfserial *bcfserial)
{
	bcfserial->tx_crc ^= 0xffff;
	bcfserial_append_escaped(bcfserial, bcfserial->tx_crc & 0xff);
	bcfserial_append_escaped(bcfserial, (bcfserial->tx_crc >> 8) & 0xff);
}

static void bcfserial_hdlc_send(struct bcfserial *bcfserial, u8 cmd, u16 value, u16 index, u16 length, const void* buffer)
{
	// HDLC_FRAME
	// 0 address : 0x01
	// 1 control : 0x03
	// 2 [bmRequestType] : 0x00
	// 3 cmd (TX, START, STOP, etc)
	// 4/5 value
	// 6/7 index
	// 8/9 length
	// contents
	// x/y crc
	// HDLC_FRAME

	spin_lock(&bcfserial->tx_producer_lock);

	bcfserial_append_tx_frame(bcfserial);
	bcfserial_append_tx_u8(bcfserial, 0x01); //address
	bcfserial_append_tx_u8(bcfserial, 0x03); //control
	bcfserial_append_tx_u8(bcfserial, 0x00); //ignored
	bcfserial_append_tx_u8(bcfserial, cmd);
	bcfserial_append_tx_le16(bcfserial, value);
	bcfserial_append_tx_le16(bcfserial, index);
	bcfserial_append_tx_le16(bcfserial, length);
	bcfserial_append_tx_buffer(bcfserial, buffer, length);
	bcfserial_append_tx_crc(bcfserial);
	bcfserial_append_tx_frame(bcfserial);

	spin_unlock(&bcfserial->tx_producer_lock);

	spin_lock(&bcfserial->tx_consumer_lock);
	bcfserial_serdev_write_locked(bcfserial);
	spin_unlock(&bcfserial->tx_consumer_lock);
}

static void bcfserial_hdlc_send_cmd(struct bcfserial *bcfserial, u8 cmd)
{
	bcfserial_hdlc_send(bcfserial, cmd, 0, 0, 0, NULL);
}

static void bcfserial_hdlc_send_ack(struct bcfserial *bcfserial, u8 address, u8 seq)
{
	// To make this a valid S-frame:
	// u8 ctrl = (((seq + 1) & 0x07) << 5) | 0x01;
	// TODO Fix control frame type bug here and in wpanusb_bc

	spin_lock(&bcfserial->tx_producer_lock);

	bcfserial_append_tx_frame(bcfserial);
	bcfserial_append_tx_u8(bcfserial, address); //address
	bcfserial_append_tx_u8(bcfserial, 0x00); //control
	bcfserial_append_tx_crc(bcfserial);
	bcfserial_append_tx_frame(bcfserial);

	spin_unlock(&bcfserial->tx_producer_lock);

	spin_lock(&bcfserial->tx_consumer_lock);
	bcfserial_serdev_write_locked(bcfserial);
	spin_unlock(&bcfserial->tx_consumer_lock);
}

static int bcfserial_hdlc_receive(struct bcfserial *bcfserial, u8 cmd, void *buffer, size_t count)
{
	int retries = 80;
	bcfserial->response_size = count;
	bcfserial->response_buffer = (u8*)buffer;
	bcfserial_hdlc_send_cmd(bcfserial, cmd);
	// TODO semaphore? give/take
	do {
		usleep_range(10000,10001);
	} while (bcfserial->response_size && retries--);
	bcfserial->response_buffer = NULL;
	if (bcfserial->response_size) {
		bcfserial->response_size = 0;
		return -EAGAIN;
	}
	return 0;
}

static int bcfserial_start(struct ieee802154_hw *hw)
{
	struct bcfserial *bcfserial = hw->priv;
	dev_dbg(&bcfserial->serdev->dev, "START\n");
	bcfserial_hdlc_send_cmd(bcfserial, START);
	return 0;
}

static void bcfserial_stop(struct ieee802154_hw *hw)
{
	struct bcfserial *bcfserial = hw->priv;
	dev_dbg(&bcfserial->serdev->dev, "STOP\n");
	bcfserial_hdlc_send_cmd(bcfserial, STOP);
}

static int bcfserial_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct bcfserial *bcfserial = hw->priv;

	if (bcfserial->tx_skb)
	{
		dev_err(&bcfserial->serdev->dev, "SKB not freed! %d\n", bcfserial->tx_ack_seq);
	}

	bcfserial->tx_skb = skb;
	bcfserial->tx_ack_seq++;
	if (!bcfserial->tx_ack_seq) {
		bcfserial->tx_ack_seq++;
	}

	dev_dbg(&bcfserial->serdev->dev, "XMIT %02x %d\n", bcfserial->tx_ack_seq, skb->len);

	bcfserial_hdlc_send(bcfserial, TX, 0, bcfserial->tx_ack_seq, skb->len, skb->data);

	return 0;
}

static int bcfserial_ed(struct ieee802154_hw *hw, u8 *level)
{
	struct bcfserial *bcfserial = hw->priv;
	dev_dbg(&bcfserial->serdev->dev, "ED\n");
	WARN_ON(!level);
	*level = 0xbe;
	return 0;
}

static int bcfserial_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct bcfserial *bcfserial = hw->priv;
	u8 buffer[2] = {page, channel};
	dev_dbg(&bcfserial->serdev->dev, "SET CHANNEL %u %u\n", page, channel);
	bcfserial_hdlc_send(bcfserial, SET_CHANNEL, 0, 0, 2, &buffer);
	return 0;
}

static int bcfserial_set_hw_addr_filt(struct ieee802154_hw *hw,
				      struct ieee802154_hw_addr_filt *filt,
				      unsigned long changed)
{
	struct bcfserial *bcfserial = hw->priv;

	if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
		u16 addr = le16_to_cpu(filt->short_addr);
		dev_dbg(&bcfserial->serdev->dev, "Short Address changed %x\n", addr);
		bcfserial_hdlc_send(bcfserial, SET_SHORT_ADDR, 0, 0, sizeof(addr), &addr);
	}

	if (changed & IEEE802154_AFILT_PANID_CHANGED) {
		u16 pan = le16_to_cpu(filt->pan_id);
		dev_dbg(&bcfserial->serdev->dev, "PAN ID changed %x\n", pan);
		bcfserial_hdlc_send(bcfserial, SET_PAN_ID, 0, 0, sizeof(pan), &pan);
	}

	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
		u64 ieee_addr = le64_to_cpu(filt->ieee_addr);
		dev_dbg(&bcfserial->serdev->dev, "IEEE Addr changed %llx\n", ieee_addr);
		bcfserial_hdlc_send(bcfserial, SET_IEEE_ADDR, 0, 0, sizeof(ieee_addr), &ieee_addr);
	}
	return 0;
}

static int bcfserial_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	struct bcfserial *bcfserial = hw->priv;
	dev_dbg(&bcfserial->serdev->dev, "SET TXPOWER\n");
	return -ENOTSUPP;
}

static int bcfserial_set_lbt(struct ieee802154_hw *hw, bool on)
{
	struct bcfserial *bcfserial = hw->priv;
	dev_dbg(&bcfserial->serdev->dev, "SET LBT\n");
	return -ENOTSUPP;
}

static int bcfserial_set_cca_mode(struct ieee802154_hw *hw,
			   const struct wpan_phy_cca *cca)
{
	struct bcfserial *bcfserial = hw->priv;
	dev_dbg(&bcfserial->serdev->dev, "SET CCA MODE\n");
	return -ENOTSUPP;
}

static int bcfserial_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm)
{
	struct bcfserial *bcfserial = hw->priv;
	dev_dbg(&bcfserial->serdev->dev, "SET CCA ED LEVEL\n");
	return -ENOTSUPP;
}

static int bcfserial_set_csma_params(struct ieee802154_hw *hw, u8 min_be, u8 max_be,
			      u8 retries)
{
	struct bcfserial *bcfserial = hw->priv;
	dev_dbg(&bcfserial->serdev->dev, "SET CSMA PARAMS\n");
	return -ENOTSUPP;
}

static int bcfserial_set_frame_retries(struct ieee802154_hw *hw, s8 retries)
{
	struct bcfserial *bcfserial = hw->priv;
	dev_dbg(&bcfserial->serdev->dev, "SET FRAME RETRIES\n");
	return -ENOTSUPP;
}

static int bcfserial_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
	struct bcfserial *bcfserial = hw->priv;
	dev_dbg(&bcfserial->serdev->dev, "SET PROMISCUOUS\n");
	return -ENOTSUPP;
}

static const struct ieee802154_ops bcfserial_ops = {
	.owner			= THIS_MODULE,
	.start			= bcfserial_start,
	.stop			= bcfserial_stop,
	.xmit_async		= bcfserial_xmit,
	.ed			= bcfserial_ed,
	.set_channel		= bcfserial_set_channel,
	.set_hw_addr_filt	= bcfserial_set_hw_addr_filt,
	.set_txpower		= bcfserial_set_txpower,
	.set_lbt		= bcfserial_set_lbt,
	.set_cca_mode		= bcfserial_set_cca_mode,
	.set_cca_ed_level	= bcfserial_set_cca_ed_level,
	.set_csma_params	= bcfserial_set_csma_params,
	.set_frame_retries	= bcfserial_set_frame_retries,
	.set_promiscuous_mode	= bcfserial_set_promiscuous_mode,
};

static void bcfserial_wpan_rx(struct bcfserial *bcfserial, const u8 *buffer, size_t count)
{
	struct sk_buff *skb;
	u8 len, lqi;

	if (count == 1) {
		// TX ACK
		dev_dbg(&bcfserial->serdev->dev, "TX ACK: 0x%02x:0x%02x\n", buffer[0], bcfserial->tx_ack_seq);

		if (buffer[0] == bcfserial->tx_ack_seq && bcfserial->tx_skb) {
			skb = bcfserial->tx_skb;
			bcfserial->tx_skb = NULL;
			ieee802154_xmit_complete(bcfserial->hw, skb, false);
		} else {
			dev_err(&bcfserial->serdev->dev, "unknown ack %u\n", bcfserial->tx_ack_seq);
		}
	} else if (bcfserial->response_size == count && bcfserial->response_buffer) {
		//TODO replace with semaphore
		dev_dbg(&bcfserial->serdev->dev, "Response size %u found\n", count);
		memcpy(bcfserial->response_buffer, buffer, count);
		bcfserial->response_size = 0;
	} else {
		// RX Packet
		dev_dbg(&bcfserial->serdev->dev, "RX Packet Len:%u LQI:%u\n", buffer[0], buffer[count-1]);
		len = buffer[0];
		lqi = buffer[count-1];

		if (len+2 != count) {
			dev_err(&bcfserial->serdev->dev, "RX Packet invalid length\n");
			return;
		}

		if (!ieee802154_is_valid_psdu_len(len)) {
			dev_err(&bcfserial->serdev->dev, "frame corrupted\n");
			return;
		}

		skb = dev_alloc_skb(IEEE802154_MTU);
		if (!skb) {
			dev_err(&bcfserial->serdev->dev, "failed to allocate sk_buff\n");
			return;
		}

		skb_put_data(skb, buffer+1, len);
		ieee802154_rx_irqsafe(bcfserial->hw, skb, lqi);
	}
}

static int bcfserial_tty_receive(struct serdev_device *serdev,
	const unsigned char *data, size_t count)
{
	struct bcfserial *bcfserial = serdev_device_get_drvdata(serdev);
	u16 crc_check = 0;
	size_t i;
	u8 c;


	for (i = 0; i < count; i++) {
		c = data[i];

		if (c == HDLC_FRAME) {
			if (bcfserial->rx_address != 0xFF) {
				crc_check = crc_ccitt(0xffff, &bcfserial->rx_address, 1);
				crc_check = crc_ccitt(crc_check, bcfserial->rx_buffer, bcfserial->rx_offset);

				if (crc_check == 0xf0b8) {
					if ((bcfserial->rx_buffer[0] & 1) == 0) {
						//I-Frame, send S-Frame ACK
						bcfserial_hdlc_send_ack(bcfserial, bcfserial->rx_address, (bcfserial->rx_buffer[0] >> 1) & 0x7);
					}

					if (bcfserial->rx_address == ADDRESS_WPAN) {
						bcfserial_wpan_rx(bcfserial, bcfserial->rx_buffer + 1, bcfserial->rx_offset - 3);
					}
					else if (bcfserial->rx_address == ADDRESS_CDC) {
						bcfserial->rx_buffer[bcfserial->rx_offset-2] = 0;
						dev_dbg(&bcfserial->serdev->dev, "> %s", bcfserial->rx_buffer+1);
					}
				}
				else {
					dev_err(&bcfserial->serdev->dev, "CRC Failed from %02x: 0x%04x\n", bcfserial->rx_address, crc_check);
				}
			}
			bcfserial->rx_offset = 0;
			bcfserial->rx_address = 0xFF;
		} else if (c == HDLC_ESC) {
			bcfserial->rx_in_esc = 1;
		} else {
			if (bcfserial->rx_in_esc) {
				c ^= 0x20;
				bcfserial->rx_in_esc = 0;
			}

			if (bcfserial->rx_address == 0xFF) {
				bcfserial->rx_address = c;
				if (bcfserial->rx_address == ADDRESS_WPAN ||
				   bcfserial->rx_address == ADDRESS_CDC ||
				   bcfserial->rx_address == ADDRESS_HW) {
				} else {
					bcfserial->rx_address = 0xFF;
				}
					bcfserial->rx_offset = 0;
			} else {
				if (bcfserial->rx_offset < MAX_RX_HDLC) {
					bcfserial->rx_buffer[bcfserial->rx_offset] = c;
					bcfserial->rx_offset++;
				} else {
					//buffer overflow
					dev_err(&bcfserial->serdev->dev, "RX Buffer Overflow\n");
					bcfserial->rx_address = 0xFF;
					bcfserial->rx_offset = 0;
				}
			}
		}
	}

	return count;
}

static void bcfserial_uart_transmit(struct work_struct *work)
{
	struct bcfserial *bcfserial = container_of(work, struct bcfserial, tx_work);

	spin_lock_bh(&bcfserial->tx_consumer_lock);
	bcfserial_serdev_write_locked(bcfserial);
	spin_unlock_bh(&bcfserial->tx_consumer_lock);
}

static void bcfserial_tty_wakeup(struct serdev_device *serdev)
{
	struct bcfserial *bcfserial = serdev_device_get_drvdata(serdev);

	schedule_work(&bcfserial->tx_work);
}

static struct serdev_device_ops bcfserial_serdev_ops = {
	.receive_buf = bcfserial_tty_receive,
	.write_wakeup = bcfserial_tty_wakeup,
};

static const struct of_device_id bcfserial_of_match[] = {
	{
	.compatible = "beagle,bcfserial",
	},
	{}
};
MODULE_DEVICE_TABLE(of, bcfserial_of_match);

static const s32 channel_powers[] = {
	300, 280, 230, 180, 130, 70, 0, -100, -200, -300, -400, -500, -700,
	-900, -1200, -1700,
};

static int bcfserial_get_device_capabilities(struct bcfserial *bcfserial)
{
	u32 valid_channels = 0;
	int ret = 0;
	struct ieee802154_hw *hw = bcfserial->hw;

	bcfserial_hdlc_send_cmd(bcfserial, RESET);

	ret = bcfserial_hdlc_receive(bcfserial, GET_SUPPORTED_CHANNELS, &valid_channels, sizeof(valid_channels));
	if (ret < 0) {
		return ret;
	}
	dev_dbg(&bcfserial->serdev->dev, "Supported Channels %x\n", valid_channels);

	/* FIXME: these need to come from device capabilities */
	hw->flags = IEEE802154_HW_TX_OMIT_CKSUM | IEEE802154_HW_AFILT;

	/* FIXME: these need to come from device capabilities */
	hw->phy->flags = WPAN_PHY_FLAG_TXPOWER;

	/* Set default and supported channels */
	hw->phy->current_page = 0;
	hw->phy->current_channel = ffs(valid_channels) - 1; //set to lowest valid channel
	hw->phy->supported.channels[0] = valid_channels;

	/* FIXME: these need to come from device capabilities */
	hw->phy->supported.tx_powers = channel_powers;
	hw->phy->supported.tx_powers_size = ARRAY_SIZE(channel_powers);
	hw->phy->transmit_power = hw->phy->supported.tx_powers[0];

	return ret;
}

static int bcfserial_probe(struct serdev_device *serdev)
{
	struct ieee802154_hw *hw;
	struct bcfserial *bcfserial = NULL;
	u32 speed = 115200;
	int ret;

	hw = ieee802154_alloc_hw(sizeof(struct bcfserial), &bcfserial_ops);
	if (!hw)
		return -ENOMEM;

	bcfserial = hw->priv;
	bcfserial->hw = hw;
	hw->parent = &serdev->dev;
	bcfserial->serdev = serdev;

	INIT_WORK(&bcfserial->tx_work, bcfserial_uart_transmit);

	spin_lock_init(&bcfserial->tx_producer_lock);
	spin_lock_init(&bcfserial->tx_consumer_lock);
	bcfserial->tx_circ_buf.head = 0;
	bcfserial->tx_circ_buf.tail = 0;
	bcfserial->tx_circ_buf.buf = devm_kmalloc(&serdev->dev, TX_CIRC_BUF_SIZE, GFP_KERNEL);

	bcfserial->rx_buffer = devm_kmalloc(&serdev->dev, MAX_RX_HDLC, GFP_KERNEL);
	bcfserial->rx_offset = 0;
	bcfserial->rx_address = 0xff;
	bcfserial->rx_in_esc = 0;

	serdev_device_set_drvdata(serdev, bcfserial);
	serdev_device_set_client_ops(serdev, &bcfserial_serdev_ops);

	ret = serdev_device_open(serdev);
	if (ret) {
		dev_err(&bcfserial->serdev->dev, "Unable to open device\n");
		goto fail_hw;
	}

	speed = serdev_device_set_baudrate(serdev, speed);
	dev_dbg(&bcfserial->serdev->dev, "Using baudrate %u\n", speed);

	serdev_device_set_flow_control(serdev, false);

	bcfserial_hdlc_send_ack(bcfserial, 0x41, 0x00);

	ret = bcfserial_get_device_capabilities(bcfserial);

	if (ret < 0) {
		// dev_err(&udev->dev, "Failed to get device capabilities");
		dev_err(&bcfserial->serdev->dev, "Failed to get device capabilities\n");
		goto fail;
	}

	ret = ieee802154_register_hw(hw);

	dev_info(&bcfserial->serdev->dev, "bcfserial started");
	if (ret)
		goto fail;

	return 0;

fail:
	dev_err(&bcfserial->serdev->dev, "Closing serial device on failure\n");
	serdev_device_close(serdev);
fail_hw:
	dev_err(&bcfserial->serdev->dev, "Failed to open bcfserial\n");
	ieee802154_free_hw(hw);
	return ret;
}

static void bcfserial_remove(struct serdev_device *serdev)
{
	struct bcfserial *bcfserial = serdev_device_get_drvdata(serdev);
	dev_info(&bcfserial->serdev->dev, "Closing serial device\n");
	ieee802154_unregister_hw(bcfserial->hw);
	flush_work(&bcfserial->tx_work);
	ieee802154_free_hw(bcfserial->hw);
	serdev_device_close(serdev);
}

static struct serdev_device_driver bcfserial_driver = {
	.probe = bcfserial_probe,
	.remove = bcfserial_remove,
	.driver = {
		.name = BCFSERIAL_DRV_NAME,
		.of_match_table = of_match_ptr(bcfserial_of_match),
	},
};

module_serdev_device_driver(bcfserial_driver);

MODULE_DESCRIPTION("WPAN serial driver for BeagleConnect");
MODULE_AUTHOR("Erik Larson <erik@statropy.com>");
MODULE_VERSION("0.1.1");
MODULE_LICENSE("GPL v2");
