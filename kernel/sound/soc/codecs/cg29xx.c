/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Roger Nilsson <roger.xr.nilsson@stericsson.com>,
 *         Ola Lilja <ola.o.lilja@stericsson.com>
 *         for ST-Ericsson.
 *
 * License terms:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */
 #include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/bitops.h>
/* #include <linux/mfd/cg2900_audio.h> */

#include "cg29xx.h"

#define CG29XX_NBR_OF_DAI	2
#define CG29XX_SUPPORTED_RATE_PCM (SNDRV_PCM_RATE_8000 | \
	SNDRV_PCM_RATE_16000)

#define CG29XX_SUPPORTED_RATE (SNDRV_PCM_RATE_8000 | \
	SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

#define CG29XX_SUPPORTED_FMT (SNDRV_PCM_FMTBIT_S16_LE)

enum cg29xx_dai_direction {
	CG29XX_DAI_DIRECTION_TX,
	CG29XX_DAI_DIRECTION_RX
};

static int cg29xx_dai_startup(
	struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai);

static int cg29xx_dai_prepare(
	struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai);

static int cg29xx_dai_hw_params(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *hw_params,
	struct snd_soc_dai *dai);

static void cg29xx_dai_shutdown(
	struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai);

static int cg29xx_set_dai_sysclk(
	struct snd_soc_dai *codec_dai,
	int clk_id,
	unsigned int freq, int dir);

static int cg29xx_set_dai_fmt(
	struct snd_soc_dai *codec_dai,
	unsigned int fmt);

static int cg29xx_set_tdm_slot(
	struct snd_soc_dai *dai,
	unsigned int tx_mask, unsigned int rx_mask,
	int slots,
	int slot_width);

static struct cg29xx_codec codec_private = {
	.session = 0,
};

static struct snd_soc_dai_ops cg29xx_dai_driver_dai_ops = {
	.startup = cg29xx_dai_startup,
	.prepare = cg29xx_dai_prepare,
	.hw_params = cg29xx_dai_hw_params,
	.shutdown = cg29xx_dai_shutdown,
	.set_sysclk = cg29xx_set_dai_sysclk,
	.set_fmt = cg29xx_set_dai_fmt,
	.set_tdm_slot = cg29xx_set_tdm_slot
};

struct snd_soc_dai_driver cg29xx_dai_driver[] = {
	{
	.name = "cg29xx-codec-dai.0",
	.id = 0,
	.playback = {
		.stream_name = "CG29xx.0 Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = CG29XX_SUPPORTED_RATE,
		.formats = CG29XX_SUPPORTED_FMT,
	},
	.capture = {
		.stream_name = "CG29xx.0 Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = CG29XX_SUPPORTED_RATE,
		.formats = CG29XX_SUPPORTED_FMT,
	},
	.ops = &cg29xx_dai_driver_dai_ops,
	.symmetric_rates = 1,
	},
	{
	.name = "cg29xx-codec-dai.1",
	.id = 1,
	.playback = {
		.stream_name = "CG29xx.1 Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = CG29XX_SUPPORTED_RATE_PCM,
		.formats = CG29XX_SUPPORTED_FMT,
	},
	.capture = {
		.stream_name = "CG29xx.1 Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = CG29XX_SUPPORTED_RATE_PCM,
		.formats = CG29XX_SUPPORTED_FMT,
	},
	.ops = &cg29xx_dai_driver_dai_ops,
	.symmetric_rates = 1,
	}
};
EXPORT_SYMBOL_GPL(cg29xx_dai_driver);


static struct cg29xx_codec_dai_data *get_dai_data_codec(struct snd_soc_codec *codec,
						int dai_id)
{
	struct cg29xx_codec_dai_data *codec_drvdata = snd_soc_codec_get_drvdata(codec);
	return &codec_drvdata[dai_id];
}

static struct cg29xx_codec_dai_data *get_dai_data(struct snd_soc_dai *codec_dai)
{
	return get_dai_data_codec(codec_dai->codec, codec_dai->id);
}

static int cg29xx_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				int clk_id,
				unsigned int freq, int dir)
{
	return 0;
}

static int cg29xx_set_dai_fmt(struct snd_soc_dai *codec_dai,
			unsigned int fmt)
{
	return 0;
}

static int cg29xx_set_tdm_slot(struct snd_soc_dai *codec_dai,
			unsigned int tx_mask,
			unsigned int rx_mask,
			int slots,
			int slot_width)
{
	return 0;
}




static int cg29xx_dai_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	return 0;
}

static int cg29xx_dai_prepare(struct snd_pcm_substream *substream,
			struct snd_soc_dai *codec_dai)
{
	return 0;
}

static void cg29xx_dai_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *codec_dai)
{
}

static int cg29xx_dai_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *hw_params,
				struct snd_soc_dai *codec_dai)
{
	return 0;
}

static unsigned int cg29xx_codec_read(struct snd_soc_codec *codec,
				unsigned int reg)
{
	return 0;
}

static int cg29xx_codec_write(struct snd_soc_codec *codec,
			unsigned int reg,
			unsigned int value)
{
	return 0;
}

static int cg29xx_codec_probe(struct snd_soc_codec *codec)
{
	pr_debug("%s: Enter (codec->name = %s).\n", __func__, codec->name);

	return 0;
}

static int cg29xx_codec_remove(struct snd_soc_codec *codec)
{
	pr_debug("%s: Enter (codec->name = %s).\n", __func__, codec->name);

	return 0;
}

static int cg29xx_codec_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	pr_debug("%s: Enter (codec->name = %s).\n", __func__, codec->name);

	return 0;
}

static int cg29xx_codec_resume(struct snd_soc_codec *codec)
{
	pr_debug("%s: Enter (codec->name = %s).\n", __func__, codec->name);

	return 0;
}

struct snd_soc_codec_driver cg29xx_codec_driver = {
	.probe = cg29xx_codec_probe,
	.remove = cg29xx_codec_remove,
	.suspend = cg29xx_codec_suspend,
	.resume = cg29xx_codec_resume,
	.read = cg29xx_codec_read,
	.write = cg29xx_codec_write,
};

static int __devinit cg29xx_codec_driver_probe(struct platform_device *pdev)
{
	int ret;
	pr_debug("%s: Enter.\n", __func__);

	pr_info("%s: Init codec private data..\n", __func__);

	platform_set_drvdata(pdev, NULL);

	pr_info("%s: Register codec.\n", __func__);
	ret = snd_soc_register_codec(&pdev->dev, &cg29xx_codec_driver, &cg29xx_dai_driver[0], 2);
	if (ret < 0) {
		pr_debug("%s: Error: Failed to register codec (ret = %d).\n",
			__func__,
			ret);
		snd_soc_unregister_codec(&pdev->dev);
		return ret;
	}

	return 0;
}

static int __devexit cg29xx_codec_driver_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static int cg29xx_codec_driver_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int cg29xx_codec_driver_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver cg29xx_codec_platform_driver = {
	.driver = {
		.name = "cg29xx-codec",
		.owner = THIS_MODULE,
	},
	.probe = cg29xx_codec_driver_probe,
	.remove = __devexit_p(cg29xx_codec_driver_remove),
	.suspend = cg29xx_codec_driver_suspend,
	.resume = cg29xx_codec_driver_resume,
};


static int __devinit cg29xx_codec_platform_driver_init(void)
{
	int ret;

	pr_debug("%s: Enter.\n", __func__);

	ret = platform_driver_register(&cg29xx_codec_platform_driver);
	if (ret != 0)
		pr_err("Failed to register CG29xx platform driver (%d)!\n", ret);

	return ret;
}

static void __exit cg29xx_codec_platform_driver_exit(void)
{
	pr_debug("%s: Enter.\n", __func__);

	platform_driver_unregister(&cg29xx_codec_platform_driver);
}


module_init(cg29xx_codec_platform_driver_init);
module_exit(cg29xx_codec_platform_driver_exit);

MODULE_LICENSE("GPLv2");
