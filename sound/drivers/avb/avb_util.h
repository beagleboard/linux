#ifndef AVB_UTIL_H
#define AVB_UTIL_H

#include <linux/uio.h>
#include <linux/string.h>
#include <linux/net.h>
#include <linux/if_packet.h>
#include <linux/rtnetlink.h>
#include <sound/pcm.h>

#define SND_AVB_DRIVER "snd_avb"

#define AVB_DEBUG

#define ETH_MSRP (0x22EA)

#define SND_AVB_NUM_CARDS (SNDRV_CARDS)

#ifdef CONFIG_PM_SLEEP
#define AVB_PM_OPS &avb_pm
#else
#define AVB_PM_OPS NULL
#endif

#define AVB_USE_HIGH_RES_TIMER

#define AVB_KERN_EMERG 0 /* system is unusable */
#define AVB_KERN_ALERT 1 /* action must be taken immediately */
#define AVB_KERN_CRIT 2 /* critical conditions */
#define AVB_KERN_ERR 3 /* error conditions */
#define AVB_KERN_WARN 4 /* warning conditions */
#define AVB_KERN_NOT 5 /* normal but significant condition */
#define AVB_KERN_INFO 6 /* informational */
#define AVB_KERN_DEBUG 7 /* debug-level messages */

#define AVB_WQ "AVBWQ"

#define AVB_DELAY_WORK_MSRP (0)
#define AVB_DELAY_WORK_AVTP (1)
#define AVB_DELAY_WORK_AVDECC (2)

#define AVB_MAX_TS_SLOTS (12)
#define AVB_AVTP_AAF_SAMPLES_PER_PACKET                                        \
	(192) /* 4ms * 48KHz i.e. Maxframes per jiffy for HZ=250 */
#define AVB_MSRP_ETH_FRAME_SIZE (2048)
#define AVB_MAX_ETH_FRAME_SIZE (AVB_MSRP_ETH_FRAME_SIZE)

#define MSRP_ATTRIBUTE_TYPE_TALKER_ADVERTISE_VECTOR (1)
#define MSRP_ATTRIBUTE_TYPE_TALKER_FAILED_VECTOR (2)
#define MSRP_ATTRIBUTE_TYPE_LISTENER_VECTOR (3)
#define MSRP_ATTRIBUTE_TYPE_DOMAIN_VECTOR (4)

#define MSRP_ATTRIBUTE_LEN_TALKER_ADVERTISE_VECTOR (25)
#define MSRP_ATTRIBUTE_LEN_TALKER_FAILED_VECTOR (34)
#define MSRP_ATTRIBUTE_LEN_LISTENER_VECTOR (8)
#define MSRP_ATTRIBUTE_LEN_DOMAIN_VECTOR (4)

#define MSRP_MAX_FRAME_SIZE_48KHZ_AUDIO (80)

#define MSRP_MAX_INTERVAL_FRAME_48KHZ_AUDIO (1)

#define MSRP_ATTRIBUTE_EVENT_NEW (0)
#define MSRP_ATTRIBUTE_EVENT_JOININ (1)
#define MSRP_ATTRIBUTE_EVENT_IN (2)
#define MSRP_ATTRIBUTE_EVENT_JOINMT (3)
#define MSRP_ATTRIBUTE_EVENT_MT (4)
#define MSRP_ATTRIBUTE_EVENT_LEAVE (5)

#define MSRP_DECLARATION_STATE_NONE (0)
#define MSRP_DECLARATION_STATE_IGNORE (0)
#define MSRP_DECLARATION_STATE_ASKING_FAILED (1)
#define MSRP_DECLARATION_STATE_READY (2)
#define MSRP_DECLARATION_STATE_READY_FAILED (3)
#define MSRP_DECLARATION_STATE_UNKNOWN (255)

#define MSRP_THREE_PACK(a, b, c) (u8)((((a * 6) + b) * 6) + c)
#define MSRP_THREE_PACK_GET_A(x) (u8)(x / 36)
#define MSRP_THREE_PACK_GET_B(x) (u8)((x / 6) % 6)
#define MSRP_THREE_PACK_GET_C(x) (u8)(x % 6)
#define MSRP_FOUR_PACK(a, b, c, d) (u8)((a * 64) + (b * 16) + (c * 4) + (d))
#define MSRP_FOUR_PACK_GET_A(x) (u8)(x / 64)

#define AVTP_PDU_COMMON_STREAM_HEADER_LENGTH (24)
#define AVTP_PDU_COMMON_CONTROL_HEADER_LENGTH (12)

#define AVB_AVTP_AAF_VERSION (0)

#define AVB_AVTP_SUBTYPE_AAF (2)
#define AVB_AVTP_SUBTYPE_ADP (0xFA)
#define AVB_AVTP_SUBTYPE_AECP (0xFB)
#define AVB_AVTP_SUBTYPE_ACMP (0xFC)
#define AVB_AVTP_SUBTYPE_MAAP (0xFE)

#define AVB_AVTP_AAF_FORMAT_USER_SP (0)
#define AVB_AVTP_AAF_FORMAT_32_BIT_FLOAT (1)
#define AVB_AVTP_AAF_FORMAT_32_BIT_INT (2)
#define AVB_AVTP_AAF_FORMAT_24_bit_INT (3)
#define AVB_AVTP_AAF_FORMAT_16_BIT_INT (4)
#define AVB_AVTP_AAF_FORMAT_32_BIT_AES3 (5)

#define AVB_AVTP_AAF_NSR_USER_SP (0x0)
#define AVB_AVTP_AAF_NSR_8_KHZ (0x1)
#define AVB_AVTP_AAF_NSR_16_KHZ (0x2)
#define AVB_AVTP_AAF_NSR_32_KHZ (0x3)
#define AVB_AVTP_AAF_NSR_44_1_KHZ (0x4)
#define AVB_AVTP_AAF_NSR_48_KHZ (0x5)
#define AVB_AVTP_AAF_NSR_88_2_KHZ (0x6)
#define AVB_AVTP_AAF_NSR_96_KHZ (0x7)
#define AVB_AVTP_AAF_NSR_176_4_KHZ (0x8)
#define AVB_AVTP_AAF_NSR_192_KHZ (0x9)
#define AVB_AVTP_AAF_NSR_24_KHZ (0xA)

#define AVB_ADP_MSGTYPE_ENTITY_AVAILABLE (0x0)
#define AVB_ADP_MSGTYPE_ENTITY_DEPARTING (0x1)
#define AVB_ADP_MSGTYPE_ENTITY_DISCOVER (0x2)

#define AVB_ADP_CONTROL_DATA_LENGTH (56)

#define AVB_AECP_MSGTYPE_AEM_COMMAND (0x00)
#define AVB_AECP_MSGTYPE_AEM_RESPONSE (0x01)

#define AVB_AEM_CMD_ENTITY_ACQUIRE (0x00)
#define AVB_AEM_CMD_ENTITY_LOCK (0x01)
#define AVB_AEM_CMD_ENTITY_AVAILABLE (0x02)
#define AVB_AEM_CMD_CTRL_AVAILABLE (0x03)
#define AVB_AEM_CMD_READ_DESCP (0x04)
#define AVB_AEM_CMD_WRITE_DESCP (0x05)
#define AVB_AEM_CMD_SET_CONFIG (0x06)
#define AVB_AEM_CMD_GET_CONFIG (0x07)
#define AVB_AEM_CMD_SET_STREAM_FORMAT (0x08)
#define AVB_AEM_CMD_SET_STREAM_INFO (0x0e)
#define AVB_AEM_CMD_GET_STREAM_INFO (0x0f)
#define AVB_AEM_CMD_REGISTER_UNSOLICITED_NOTIFICATION (0x24)
#define AVB_AEM_CMD_GET_COUNTERS (0x029)

#define AVB_AEM_DESCP_ENTITY (0x00)
#define AVB_AEM_DESCP_CONFIGURATION (0x01)
#define AVB_AEM_DESCP_AUDIO_UNIT (0x02)
#define AVB_AEM_DESCP_STREAM_IP (0x05)
#define AVB_AEM_DESCP_STREAM_OP (0x06)
#define AVB_AEM_DESCP_JACK_IP (0x07)
#define AVB_AEM_DESCP_JACK_OP (0x08)
#define AVB_AEM_DESCP_AVBINTERFACE (0x09)
#define AVB_AEM_DESCP_CLOCKSOURCE (0x0a)
#define AVB_AEM_DESCP_LOCALE (0x0c)
#define AVB_AEM_DESCP_STRINGS (0x0d)
#define AVB_AEM_DESCP_STREAM_PORT_IP (0x0e)
#define AVB_AEM_DESCP_STREAM_PORT_OP (0x0f)
#define AVB_AEM_DESCP_EXT_PORT_IP (0x10)
#define AVB_AEM_DESCP_EXT_PORT_OP (0x11)
#define AVB_AEM_DESCP_AUDIO_CLUSTER (0x14)
#define AVB_AEM_DESCP_AUDIO_MAP (0x17)
#define AVB_AEM_DESCP_CLOCK_DOMAIN (0x24)
#define AVB_AEM_DESCP_INVALID (0xffff)

#define AVB_AEM_RES_SUCCESS (0x00)
#define AVB_AEM_RES_NOT_IMPLEMENTED (0x01)
#define AVB_AEM_RES_NO_SUCH_DESCRIPTOR (0x02)

#define AVB_AEM_MAX_DESCP_COUNT (7)

#define AVB_AEM_STREAM_FORMAT_AVTP (0x02)
#define AVB_AEM_MAX_SUPP_FORMATS (6)

#define AVB_ACMP_MSGTYPE_CONNECT_TX_CMD (0x00)
#define AVB_ACMP_MSGTYPE_CONNECT_TX_RESP (0x01)
#define AVB_ACMP_MSGTYPE_DISCONNECT_TX_CMD (0x02)
#define AVB_ACMP_MSGTYPE_DISCONNECT_TX_RESP (0x03)
#define AVB_ACMP_MSGTYPE_GET_TX_STATE_CMD (0x04)
#define AVB_ACMP_MSGTYPE_GET_TX_STATE_RESP (0x05)
#define AVB_ACMP_MSGTYPE_CONNECT_RX_CMD (0x06)
#define AVB_ACMP_MSGTYPE_CONNECT_RX_RESP (0x07)
#define AVB_ACMP_MSGTYPE_DISCONNECT_RX_CMD (0x08)
#define AVB_ACMP_MSGTYPE_DISCONNECT_RX_RESP (0x09)
#define AVB_ACMP_MSGTYPE_GET_RX_STATE_CMD (0x0A)
#define AVB_ACMP_MSGTYPE_GET_RX_STATE_RESP (0x0B)
#define AVB_ACMP_MSGTYPE_GET_TX_CONN_CMD (0x0C)
#define AVB_ACMP_MSGTYPE_GET_TX_CONN_RESP (0x0D)

#define AVB_ACMP_STATUS_SUCCESS (0x00)
#define AVB_ACMP_STATUS_LISTENER_UNKNOWN_ID (0x01)
#define AVB_ACMP_STATUS_TALKER_UNKNOWN_ID (0x02)
#define AVB_ACMP_STATUS_TALKER_DEST_MAC_FAIL (0x03)
#define AVB_ACMP_STATUS_TALKER_NO_STREAM_IDX (0x04)
#define AVB_ACMP_STATUS_TALKER_NO_BANDWIDTH (0x05)
#define AVB_ACMP_STATUS_TALKER_EXCLUSIVE (0x06)
#define AVB_ACMP_STATUS_LISTENER_TALKER_TIMEOUT (0x07)
#define AVB_ACMP_STATUS_LISTENER_EXCLUSIVE (0x08)
#define AVB_ACMP_STATUS_STATE_UNAVAILABLE (0x09)
#define AVB_ACMP_STATUS_NOT_CONNECTED (0x0a)
#define AVB_ACMP_STATUS_NO_SUCH_CONNECTION (0x0b)
#define AVB_ACMP_STATUS_COULD_NOT_SEND_MESSAGE (0x0c)
#define AVB_ACMP_STATUS_TALKER_MISBEHAVING (0x0d)
#define AVB_ACMP_STATUS_LISTENER_MISBEHAVING (0x0e)
#define AVB_ACMP_STATUS_RFU (0x0f)
#define AVB_ACMP_STATUS_CONTROLLER_NOT_AUTHORIZED (0x10)
#define AVB_ACMP_STATUS_INCOMPATIBLE_REQUEST (0x11)
#define AVB_ACMP_STATUS_NOT_SUPPORTED (0x1f)

#define AVB_AVTP_AAF_HDR_GET_SV(hdr) ((hdr->h.f.b1.sv & 0x80) >> 7)
#define AVB_AVTP_AAF_HDR_SET_SV(hdr, val)                                      \
	(hdr->h.f.b1.sv = (hdr->h.f.b1.sv | ((val << 7) & 0x80)))
#define AVB_AVTP_AAF_HDR_GET_VER(hdr) ((hdr->h.f.b1.version & 0x70) >> 4)
#define AVB_AVTP_AAF_HDR_SET_VER(hdr, val)                                     \
	(hdr->h.f.b1.version = (hdr->h.f.b1.version | ((val << 4) & 0x70)))
#define AVB_AVTP_AAF_HDR_GET_MR(hdr) ((hdr->h.f.b1.mr & 0x08) >> 3)
#define AVB_AVTP_AAF_HDR_SET_MR(hdr, val)                                      \
	(hdr->h.f.b1.mr = (hdr->h.f.b1.mr | ((val << 3) & 0x08)))
#define AVB_AVTP_AAF_HDR_GET_TSV(hdr) (hdr->h.f.b1.ts_valid & 0x01)
#define AVB_AVTP_AAF_HDR_SET_TSV(hdr, val)                                     \
	(hdr->h.f.b1.ts_valid = (hdr->h.f.b1.ts_valid | (val & 0x01)))
#define AVB_AVTP_AAF_HDR_GET_TU(hdr) (hdr->h.f.b2.tu & 0x01)
#define AVB_AVTP_AAF_HDR_SET_TU(hdr, val)                                      \
	(hdr->h.f.b2.tu = (hdr->h.f.b2.tu | (val & 0x01)))
#define AVB_AVTP_AAF_HDR_GET_NSR(hdr) ((hdr->h.f.fsd1.nsr & 0xF0) >> 4)
#define AVB_AVTP_AAF_HDR_SET_NSR(hdr, val)                                     \
	(hdr->h.f.fsd1.nsr = (hdr->h.f.fsd1.nsr | ((val << 4) & 0xF0)))
#define AVB_AVTP_AAF_HDR_GET_CPF(hdr) (hdr->h.f.fsd1.cpf & 0x03)
#define AVB_AVTP_AAF_HDR_SET_CPF(hdr, val)                                     \
	(hdr->h.f.fsd1.cpf = (hdr->h.f.fsd1.cpf | (val & 0x03)))
#define AVB_AVTP_AAF_HDR_GET_SP(hdr) ((hdr->h.f.fsd2.sp & 0x10) >> 4)
#define AVB_AVTP_AAF_HDR_SET_SP(hdr, val)                                      \
	(hdr->h.f.fsd2.sp = (hdr->h.f.fsd2.sp | ((val << 4) & 0x10)))
#define AVB_AVTP_AAF_HDR_GET_EVT(hdr) (hdr->h.f.fsd2.evt & 0x0F)
#define AVB_AVTP_AAF_HDR_SET_EVT(hdr, val)                                     \
	(hdr->h.f.fsd2.evt = (hdr->h.f.fsd2.evt | (val & 0x0F)))

#define AVB_AVTPDU_CTRL_HDR_GET_SV(hdr) ((hdr->h.f.b1.sv & 0x80) >> 7)
#define AVB_AVTPDU_CTRL_HDR_SET_SV(hdr, val)                                   \
	(hdr->h.f.b1.sv = (hdr->h.f.b1.sv | ((val << 7) & 0x80)))
#define AVB_AVTPDU_CTRL_HDR_GET_VER(hdr) ((hdr->h.f.b1.version & 0x70) >> 4)
#define AVB_AVTPDU_CTRL_HDR_SET_VER(hdr, val)                                  \
	(hdr->h.f.b1.version = (hdr->h.f.b1.version | ((val << 4) & 0x70)))
#define AVB_AVTPDU_CTRL_HDR_GET_MSGTYPE(hdr) ((hdr->h.f.b1.msg_type & 0x0f))
#define AVB_AVTPDU_CTRL_HDR_SET_MSGTYPE(hdr, val)                              \
	(hdr->h.f.b1.msg_type = (hdr->h.f.b1.msg_type | ((val)&0x0f)))
#define AVB_AVTPDU_CTRL_HDR_GET_VALIDTIME(hdr)                                 \
	((hdr->h.f.b2.valid_time & 0xf8) >> 3)
#define AVB_AVTPDU_CTRL_HDR_SET_VALIDTIME(hdr, val)                            \
	(hdr->h.f.b2.valid_time =                                              \
		 (hdr->h.f.b2.valid_time | ((val << 3) & 0xf8)))
#define AVB_AVTPDU_CTRL_HDR_GET_DATALEN(hdr)                                   \
	((((hdr->h.f.b2.data_len & 0x0007) << 8) & 0xff00) |                   \
	 (hdr->h.f.data_len & 0x00ff))
#define AVB_AVTPDU_CTRL_HDR_SET_DATALEN(hdr, val)                              \
	((hdr->h.f.b2.data_len =                                               \
		  (hdr->h.f.b2.data_len | ((val >> 8) & 0x0007))),             \
	 (hdr->h.f.data_len = (val & 0x00ff)))

struct socketdata {
	int type;
	int if_idx;
	char srcmac[6];
	char destmac[6];
	struct socket *sock;
	struct iovec tx_iov;
	struct iovec rx_iov;
	struct msghdr tx_msg_hdr;
	struct sockaddr_ll tx_sock_address;
	struct msghdr rx_msg_hdr;
	struct sockaddr_ll rx_sock_address;
	char tx_buf[AVB_MAX_ETH_FRAME_SIZE];
	char rx_buf[AVB_MAX_ETH_FRAME_SIZE];
};

struct stream_info {
	int sr;
	int st;
	int seq_no;
	int socket_count;
	unsigned long int timer_val;
	unsigned long int last_timer_ts;
	unsigned long int start_ts;
	snd_pcm_uframes_t hw_idx;
	snd_pcm_uframes_t hwnw_idx;
	snd_pcm_uframes_t num_bytes_consumed;
	snd_pcm_uframes_t period_size;
	snd_pcm_uframes_t pending_tx_frames;
	snd_pcm_uframes_t frame_size;
	snd_pcm_uframes_t buffer_size;
	snd_pcm_uframes_t fill_size;
	snd_pcm_uframes_t prev_hw_idx;
	snd_pcm_uframes_t frame_count;
	snd_pcm_uframes_t accum_frame_count;
	struct snd_pcm_substream *substream;
	unsigned char *tmp_buf;
};

struct avb_card {
	struct socketdata sd;
	struct snd_card *card;
	struct snd_pcm *pcm[1];
	struct stream_info tx;
	struct stream_info rx;
};

#ifdef AVB_USE_HIGH_RES_TIMER
struct avb_hrtimer {
	struct hrtimer timer;
	struct avb_card *card;
};
#endif

bool avb_socket_init(struct socketdata *sd, int rx_timeout);
void avb_log(int level, char *fmt, ...);

#endif