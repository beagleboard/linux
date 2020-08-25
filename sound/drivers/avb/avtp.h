#ifndef AVTP_H
#define AVTP_H

#include "avb_util.h"

#pragma pack(push, 1)

struct avt_pdu_aaf_pcm_hdr {
	union th {
		struct tf {
			u8 sub_type;
			union tb1 {
				u8 sv; /* 1 bit stream valid indication */
				u8 version; /* 3 bits version */
				u8 mr; /* 1 bit media clock restart */
				u8 rsv; /* 2 bits reserved */
				u8 ts_valid; /* 1 bit timestamp valid */
			} b1;
			u8 seq_no;
			union tb2 {
				u8 rsv; /* 7 bit reserved data */
				u8 tu; /* 1 bit timestamp uncertain */
			} b2;
			u64 stream_id;
			u32 avtp_ts;
			u8 format;
			union tfsd1 {
				u8 nsr; /* 4 bits nominal sample rate */
				u8 rsv; /* 2 bits reserved data */
				u8 cpf; /* first 2 bits of channels per frame */
			} fsd1;
			u8 cpf; /* last 8 bits of channels per frame */
			u8 bit_depth;
			u16 stream_data_len;
			union tfsd2 {
				u8 rsv; /* 3 bits reserved data */
				u8 sp; /* 1 bit sparse timestamp */
				u8 evt; /* 4 bits event data */
			} fsd2;
			u8 rsv;
		} f;
		u8 bytes[AVTP_PDU_COMMON_STREAM_HEADER_LENGTH];
	} h;
};

#pragma pack(pop)

void avb_avtp_aaf_header_init(char *buf, struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *hw_params);

#endif