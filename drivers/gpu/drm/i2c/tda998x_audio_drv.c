/*
 * ALSA SoC codec driver for HDMI audio on NXP TDA998x series.
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Darren Etheridge <detheridge@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#include <linux/module.h>
#include <sound/soc.h>
#include <linux/of_device.h>

#define DRV_NAME "nxp-hdmi-audio-codec"

static struct snd_soc_codec_driver nxp_hdmi_codec;

static struct snd_soc_dai_driver nxp_hdmi_codec_dai = {
	.name = "nxp-hdmi-hifi",
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE,
	},
};

#ifdef CONFIG_OF
static const struct of_device_id nxptda_dt_ids[] = {
        { .compatible = "nxp,nxptda", },
        { }
};
MODULE_DEVICE_TABLE(of, nxptda_dt_ids);
#endif


static int nxp_hdmi_codec_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &nxp_hdmi_codec,
			&nxp_hdmi_codec_dai, 1);
}

static int nxp_hdmi_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver nxp_hdmi_codec_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(nxptda_dt_ids)
	},

	.probe		= nxp_hdmi_codec_probe,
	.remove		= nxp_hdmi_codec_remove,
};

module_platform_driver(nxp_hdmi_codec_driver);

MODULE_AUTHOR("Darren Etheridge <detheridge@ti.com>");
MODULE_DESCRIPTION("ASoC NXP Dummy HDMI codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
