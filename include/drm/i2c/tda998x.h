#ifndef __TDA998X_H__
#define __TDA998X_H__

#include <linux/types.h>

enum tda998x_audio_format {
	AFMT_I2S,
	AFMT_SPDIF,
};

struct tda998x_encoder_params {
	uint8_t audio_cfg;
	uint8_t audio_clk_cfg;
	enum tda998x_audio_format audio_format;
	int audio_sample_rate;
	uint8_t audio_frame[6];
	uint8_t swap_a, mirr_a;
	uint8_t swap_b, mirr_b;
	uint8_t swap_c, mirr_c;
	uint8_t swap_d, mirr_d;
	uint8_t swap_e, mirr_e;
	uint8_t swap_f, mirr_f;
	uint8_t i2s_fmt;
};

#endif
