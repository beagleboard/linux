#ifndef __TDA998X_H__
#define __TDA998X_H__

enum tda998x_audio_format {
	AFMT_I2S,
	AFMT_SPDIF,
};

struct tda998x_encoder_params {
	int audio_cfg;
	int audio_clk_cfg;
	enum tda998x_audio_format audio_format;
	int audio_sample_rate;
	char audio_frame[6];
	int swap_a, mirr_a;
	int swap_b, mirr_b;
	int swap_c, mirr_c;
	int swap_d, mirr_d;
	int swap_e, mirr_e;
	int swap_f, mirr_f;
};

#endif
