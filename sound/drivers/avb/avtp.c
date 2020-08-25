#include "avtp.h"

static int avb_get_avtp_aaf_format(int rt_format)
{
	int format = AVB_AVTP_AAF_FORMAT_USER_SP;

	if ((rt_format == SNDRV_PCM_FORMAT_FLOAT_LE) ||
	    (rt_format == SNDRV_PCM_FORMAT_FLOAT_BE))
		format = AVB_AVTP_AAF_FORMAT_32_BIT_FLOAT;
	else if ((rt_format == SNDRV_PCM_FORMAT_S32_LE) ||
		 (rt_format == SNDRV_PCM_FORMAT_S32_BE) ||
		 (rt_format == SNDRV_PCM_FORMAT_U32_LE) ||
		 (rt_format == SNDRV_PCM_FORMAT_U32_BE))
		format = AVB_AVTP_AAF_FORMAT_32_BIT_INT;
	else if ((rt_format == SNDRV_PCM_FORMAT_S24_LE) ||
		 (rt_format == SNDRV_PCM_FORMAT_S24_BE) ||
		 (rt_format == SNDRV_PCM_FORMAT_U24_LE) ||
		 (rt_format == SNDRV_PCM_FORMAT_U24_BE))
		format = AVB_AVTP_AAF_FORMAT_24_bit_INT;
	else if ((rt_format == SNDRV_PCM_FORMAT_S16_LE) ||
		 (rt_format == SNDRV_PCM_FORMAT_S16_BE) ||
		 (rt_format == SNDRV_PCM_FORMAT_U16_LE) ||
		 (rt_format == SNDRV_PCM_FORMAT_U16_BE))
		format = AVB_AVTP_AAF_FORMAT_16_BIT_INT;
	else
		format = AVB_AVTP_AAF_FORMAT_USER_SP;

	return format;
}

static int avb_get_avtp_aaf_nsr(int sample_rate)
{
	int nsr = AVB_AVTP_AAF_NSR_USER_SP;

	if (sample_rate == 8000)
		nsr = AVB_AVTP_AAF_NSR_8_KHZ;
	else if (sample_rate == 16000)
		nsr = AVB_AVTP_AAF_NSR_16_KHZ;
	else if (sample_rate == 32000)
		nsr = AVB_AVTP_AAF_NSR_32_KHZ;
	else if (sample_rate == 44100)
		nsr = AVB_AVTP_AAF_NSR_44_1_KHZ;
	else if (sample_rate == 48000)
		nsr = AVB_AVTP_AAF_NSR_48_KHZ;
	else if (sample_rate == 88200)
		nsr = AVB_AVTP_AAF_NSR_88_2_KHZ;
	else if (sample_rate == 96000)
		nsr = AVB_AVTP_AAF_NSR_96_KHZ;
	else if (sample_rate == 176400)
		nsr = AVB_AVTP_AAF_NSR_176_4_KHZ;
	else if (sample_rate == 192000)
		nsr = AVB_AVTP_AAF_NSR_192_KHZ;
	else if (sample_rate == 24000)
		nsr = AVB_AVTP_AAF_NSR_24_KHZ;
	else
		nsr = AVB_AVTP_AAF_NSR_USER_SP;

	return nsr;
}

void avb_avtp_aaf_header_init(char *buf, struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *hw_params)
{
	struct avb_card *avb_card = snd_pcm_substream_chip(substream);
	struct ethhdr *eh = (struct ethhdr *)&buf[0];
	struct avt_pdu_aaf_pcm_hdr *hdr =
		(struct avt_pdu_aaf_pcm_hdr *)&buf[sizeof(struct ethhdr)];

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_avtp_aaf_header_init");

	memset(buf, 0, AVB_MAX_ETH_FRAME_SIZE);

	eh->h_dest[0] = avb_card->sd.destmac[0];
	eh->h_dest[1] = avb_card->sd.destmac[1];
	eh->h_dest[2] = avb_card->sd.destmac[2];
	eh->h_dest[3] = avb_card->sd.destmac[3];
	eh->h_dest[4] = avb_card->sd.destmac[4];
	eh->h_dest[5] = avb_card->sd.destmac[5];
	eh->h_source[0] = avb_card->sd.srcmac[0];
	eh->h_source[1] = avb_card->sd.srcmac[1];
	eh->h_source[2] = avb_card->sd.srcmac[2];
	eh->h_source[3] = avb_card->sd.srcmac[3];
	eh->h_source[4] = avb_card->sd.srcmac[4];
	eh->h_source[5] = avb_card->sd.srcmac[5];

	eh->h_proto = htons(avb_card->sd.type);

	hdr->h.f.sub_type = AVB_AVTP_SUBTYPE_AAF;
	AVB_AVTP_AAF_HDR_SET_SV(hdr, 1);
	AVB_AVTP_AAF_HDR_SET_VER(hdr, AVB_AVTP_AAF_VERSION);
	AVB_AVTP_AAF_HDR_SET_MR(hdr, 0);
	AVB_AVTP_AAF_HDR_SET_TSV(hdr, 1);
	hdr->h.f.seq_no = 0;
	AVB_AVTP_AAF_HDR_SET_TU(hdr, 0);
	hdr->h.f.stream_id = 0;
	hdr->h.f.avtp_ts = 0;
	hdr->h.f.format = avb_get_avtp_aaf_format(substream->runtime->format);
	AVB_AVTP_AAF_HDR_SET_NSR(hdr,
				 avb_get_avtp_aaf_nsr(params_rate(hw_params)));
	AVB_AVTP_AAF_HDR_SET_CPF(hdr, params_channels(hw_params));
	hdr->h.f.bit_depth = substream->runtime->sample_bits;
	hdr->h.f.stream_data_len = 0;
	AVB_AVTP_AAF_HDR_SET_SP(hdr, 1);
	AVB_AVTP_AAF_HDR_SET_EVT(hdr, 0);
}
