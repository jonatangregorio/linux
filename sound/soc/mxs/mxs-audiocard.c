/*
 * asoc platform driver for Freescale's MXS ADC/DAC
 *
 * Copyright 2013 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 *
 * Loosely based on a driver provided by Freescale that has:
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */
#include <sound/soc.h>
#include <linux/of.h>
#include <linux/module.h>

#define DRIVER_NAME "mxs-audio"

static const struct snd_soc_ops mxs_audio_daiops = {

};

static struct snd_soc_dai_link mxs_audio_dailink[] = {
	{
		.name = "HiFi Tx",
		.stream_name = "HiFi Playback",
		.ops = &mxs_audio_daiops,
		.codec_dai_name = "mxs-audioout codec dai",
	},
};

static struct snd_soc_card mxs_audio_card = {
	.name = "mxs_audio",
	.owner = THIS_MODULE,
	.dai_link = mxs_audio_dailink,
	.num_links = ARRAY_SIZE(mxs_audio_dailink),
};

static int mxs_audio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *audioout;
	struct snd_soc_card *card = &mxs_audio_card;
	int ret;

	if (!np)
		return -ENODEV;

	audioout = of_parse_phandle(np, "audioout", 0);
	if (!audioout) {
		dev_err(&pdev->dev,
				"phandle for audioout missing or invalid\n");
		return -EINVAL;
	}

	mxs_audio_dailink[0].cpu_of_node = audioout;
	mxs_audio_dailink[0].codec_of_node = audioout;
	mxs_audio_dailink[0].platform_of_node = audioout;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);

	ret = snd_soc_register_card(card);
	if (ret)
		dev_err(&pdev->dev, "failed to register card (%d)\n", ret);

	return ret;
}

static const struct of_device_id mxs_audio_dt_ids[] = {
	{ .compatible = "fsl,imx23-audio", },
	{ /* sentinel */ }
};

static struct platform_driver mxs_audio_driver = {
	.probe = mxs_audio_probe,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mxs_audio_dt_ids,
	},
};
module_platform_driver(mxs_audio_driver);

MODULE_AUTHOR("Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>");
MODULE_DESCRIPTION("mxs audio");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
