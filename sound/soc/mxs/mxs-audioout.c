#define DEBUG
/*
 * audio driver for Freescale's MXS ADC/DAC
 *
 * Copyright 2013 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 *
 * Loosely based on a driver provided by Freescale that has:
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/stmp_device.h>
#include <linux/io.h>

#include <sound/soc.h>
#include <sound/pcm_params.h>

#include "mxs-pcm.h"

#define DRIVER_NAME	"mxs-audioout"

#define REG_AUDIOOUT_CTRL		0x000
#define REG_AUDIOOUT_CTRL_DMAWAIT_COUNT_MASK		(0x1f << 16)
#define REG_AUDIOOUT_CTRL_DMAWAIT_COUNT(val)		(((val) & 0x1f) << 16)
#define REG_AUDIOOUT_CTRL_WORD_LENGTH			(1 << 6)
#define REG_AUDIOOUT_CTRL_RUN				(1 << 0)

#define REG_AUDIOOUT_STAT		0x010
#define REG_AUDIOOUT_DACSRR		0x020
#define REG_AUDIOOUT_DACVOLUME		0x030
#define REG_AUDIOOUT_DACVOLUME_MUTE_LEFT		(1 << 24)
#define REG_AUDIOOUT_DACVOLUME_VOLUME_LEFT(val)		(((val) & 0xff) << 16)
#define REG_AUDIOOUT_DACVOLUME_MUTE_RIGHT		(1 << 8)
#define REG_AUDIOOUT_DACVOLUME_VOLUME_RIGHT(val)	(((val) & 0xff) << 0)

#define REG_AUDIOOUT_DACDEBUG		0x040
#define REG_AUDIOOUT_HPVOL		0x050
#define REG_AUDIOOUT_RESERVED		0x060

#define REG_AUDIOOUT_PWRDN		0x070
#define REG_AUDIOOUT_PWRDN_RESETDEFAULT			0x01001111
#define REG_AUDIOOUT_PWRDN_DAC				(1 << 12)
#define REG_AUDIOOUT_PWRDN_CAPLESS			(1 << 4)
#define REG_AUDIOOUT_PWRDN_HEADPHONE			(1 << 0)

#define REG_AUDIOOUT_REFCTRL		0x080
#define REG_AUDIOOUT_REFCTRL_RAISE_REF			(1 << 25)
#define REG_AUDIOOUT_REFCTRL_XTAL_BGR_BIAS		(1 << 24)
#define REG_AUDIOOUT_REFCTRL_BIAS_CTRL			(((val) & 0x3) << 16)

#define REG_AUDIOOUT_ANACTRL		0x090
#define REG_AUDIOOUT_ANACTRL_HP_HOLD_GND		(1 << 5)
#define REG_AUDIOOUT_ANACTRL_HP_CLASSAB			(1 << 4)

#define REG_AUDIOOUT_TEST		0x0a0
#define REG_AUDIOOUT_TEST_HP_I1_ADJ_MASK	(3 << 22)
#define REG_AUDIOOUT_TEST_HP_I1_ADJ_NOMINAL	(0 << 22)
#define REG_AUDIOOUT_TEST_HP_I1_ADJ_MINUS50	(1 << 22)
#define REG_AUDIOOUT_TEST_HP_I1_ADJ_PLUS100	(2 << 22)
#define REG_AUDIOOUT_TEST_HP_I1_ADJ_PLUS50	(3 << 22)

#define REG_AUDIOOUT_BISTCTRL		0x0b0
#define REG_AUDIOOUT_BISTSTAT0		0x0c0
#define REG_AUDIOOUT_BISTSTAT1		0x0d0

#define REG_AUDIOOUT_ANACLKCTRL		0x0e0
#define REG_AUDIOOUT_ANACLKCTRL_CLKGATE			(1 << 31)

#define REG_AUDIOOUT_DATA		0x0f0

#define REG_AUDIOOUT_SPEAKERCTRL	0x100
#define REG_AUDIOOUT_SPEAKERCTRL_MUTE		(1 << 24)

#define REG_AUDIOOUT_VERSION		0x200

struct mxs_audioout_ddata {
	void __iomem *base;
	struct clk *clk;
};

static int mxs_audioout_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc = (void *)kcontrol->private_value;
	struct snd_soc_dai *dai = snd_kcontrol_chip(kcontrol);
	struct mxs_audioout_ddata *ddata = snd_soc_dai_get_drvdata(dai);
	u32 reg, val, mask = (1 << fls(mc->max)) - 1;

	reg = readl(ddata->base + mc->reg);
	val = (reg >> mc->shift) & mask;
	if (mc->invert)
		val = mc->max - val;
	ucontrol->value.integer.value[0] = val;

	if (mc->shift != mc->rshift) {
		val = (reg >> mc->rshift) & mask;
		if (mc->invert)
			val = mc->max - val;
		ucontrol->value.integer.value[1] = val;
	}

	return 0;
}

static int mxs_audioout_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc = (void *)kcontrol->private_value;
	struct snd_soc_dai *dai = snd_kcontrol_chip(kcontrol);
	struct mxs_audioout_ddata *ddata = snd_soc_dai_get_drvdata(dai);
	u32 reg, val, mask = (1 << fls(mc->max)) - 1;

	reg = readl(ddata->base + mc->reg);

	val = ucontrol->value.integer.value[0] & mask;
	if (mc->invert)
		val = mc->max - val;

	reg &= ~(mask << mc->shift);
	reg |= val << mc->shift;

	if (mc->shift != mc->rshift) {
		val = ucontrol->value.integer.value[1] & mask;
		if (mc->invert)
			val = mc->max - val;
		reg &= ~(mask << mc->rshift);
		reg |= val << mc->rshift;
	}

	writel(reg, ddata->base + mc->reg);

	return 0;
}

static const struct snd_kcontrol_new mxs_audioout_component_controls[] = {
	SOC_DOUBLE_EXT("Master Volume", REG_AUDIOOUT_DACVOLUME, 16, 0, 0xff, 0, mxs_audioout_control_get, mxs_audioout_control_put),
	SOC_DOUBLE_EXT("Master Switch", REG_AUDIOOUT_DACVOLUME, 24, 8, 0x1, 1, mxs_audioout_control_get, mxs_audioout_control_put),
	SOC_DOUBLE_EXT("Headphone Volume", REG_AUDIOOUT_HPVOL, 8, 0, 0x7f, 1, mxs_audioout_control_get, mxs_audioout_control_put),
	SOC_SINGLE_EXT("Headphone Switch", REG_AUDIOOUT_HPVOL, 24, 1, 1, mxs_audioout_control_get, mxs_audioout_control_put),
};

static int mxs_audioout_component_dai_probe(struct snd_soc_dai *dai)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = snd_soc_add_dai_controls(dai, mxs_audioout_component_controls,
			ARRAY_SIZE(mxs_audioout_component_controls));
	return ret;
}

#ifndef SH_DIV
/*
 * stolen from <linux/jiffies.h>
 */
#define SH_DIV(NOM, DEN, LSH) ((((NOM) / (DEN)) << (LSH)) \
		+ ((((NOM) % (DEN)) << (LSH)) + (DEN) / 2) / (DEN))
#endif

static int mxs_audioout_setrate(struct mxs_audioout_ddata *ddata,
		unsigned int outrate)
{
	/*
	 * The output rate is calculated as follows:
	 * 	750000 * basemult / (divint.divfrac * (hold + 1))
	 * basemult = 1, 2, 4
	 * divint = 0 .. 31
	 * divfrac = 0 .. 16383
	 * hold = 0 .. 7
	 */
	unsigned basemult, hold, divint, divfrac;
	unsigned div13;

	basemult = DIV_ROUND_UP(outrate, 48000);
	if (basemult > 4)
		return 1;
	if (basemult == 3)
		basemult = 4;

	/*
	 * the MSB of divint is always set to achieve maximal precision,
	 * so div is in [16, 32) and we get:
	 * 	hold + 1 = 75e4 * basemult / 16 * outrate
	 */
	hold = 46875 * basemult / outrate;
	if (hold > 0)
		--hold;

	/* This only happens for outrate < 5209 */
	if (hold > 7)
		return 1;

	div13 = SH_DIV(750000 * basemult, outrate * (hold + 1), 13);
	divfrac = div13 & ((1 << 13) - 1);
	divint = div13 >> 13;

	/* This cannot happen */
	if (divint > 31)
		return 1;

	pr_debug("%s(%u) -> %x %x %x %x -> %x\n", __func__, outrate,
			basemult, hold, divint, divfrac,
			basemult << 28 | hold << 24 | divint << 16 | divfrac);

	writel(basemult << 28 | hold << 24 | divint << 16 | divfrac,
			ddata->base + REG_AUDIOOUT_DACSRR);

	return 0;
}

static int mxs_audioout_component_dai_trigger(
		struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *cpu_dai)
{
	struct mxs_audioout_ddata *ddata = snd_soc_dai_get_drvdata(cpu_dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		writel(REG_AUDIOOUT_CTRL_RUN, ddata->base + REG_AUDIOOUT_CTRL + STMP_OFFSET_REG_SET);
		/* unmute speaker */
		writel(REG_AUDIOOUT_SPEAKERCTRL_MUTE, ddata->base + REG_AUDIOOUT_SPEAKERCTRL + STMP_OFFSET_REG_CLR);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		writel(REG_AUDIOOUT_SPEAKERCTRL_MUTE, ddata->base + REG_AUDIOOUT_SPEAKERCTRL + STMP_OFFSET_REG_SET);
		/* delay? or poll cross zero bit? */
		writel(REG_AUDIOOUT_CTRL_RUN, ddata->base + REG_AUDIOOUT_CTRL + STMP_OFFSET_REG_CLR);
		break;
	}
	return 0;
}

static int mxs_audioout_component_dai_hw_params(
		struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *cpu_dai)
{
	struct mxs_audioout_ddata *ddata = snd_soc_dai_get_drvdata(cpu_dai);
	int ret;
	pr_info("%s\n", __func__);

	ret = mxs_audioout_setrate(ddata, params_rate(params));
	if (ret)
		return -ERANGE;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		writel(REG_AUDIOOUT_CTRL_WORD_LENGTH,
				ddata->base + REG_AUDIOOUT_CTRL + STMP_OFFSET_REG_SET);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		writel(REG_AUDIOOUT_CTRL_WORD_LENGTH,
				ddata->base + REG_AUDIOOUT_CTRL + STMP_OFFSET_REG_CLR);
		break;
	}

	return 0;
}

static const struct snd_soc_dai_ops mxs_audioout_component_dai_ops = {
	.trigger = mxs_audioout_component_dai_trigger,
	.hw_params = mxs_audioout_component_dai_hw_params,
};

static struct snd_soc_dai_driver mxs_audioout_component_dai = {
	.name = DRIVER_NAME " component dai",
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
	.probe = mxs_audioout_component_dai_probe,
	.ops = &mxs_audioout_component_dai_ops,
};

static const struct snd_soc_component_driver mxs_audioout_component = {
	.name = DRIVER_NAME,
};

static struct snd_soc_dai_driver mxs_audioout_codec_dai = {
	.name = DRIVER_NAME " codec dai",
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
};

static const struct snd_soc_codec_driver mxs_audioout_codec = {
};

static int mxs_audioout_probe(struct platform_device *pdev)
{
	struct mxs_audioout_ddata *ddata;
	struct resource *res;
	int ret;

	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata) {
		dev_err(&pdev->dev, "cannot allocate driver data");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get device resource\n");
		return -ENOENT;
	}

	ddata->base = devm_request_and_ioremap(&pdev->dev, res);
	if (!ddata->base) {
		dev_err(&pdev->dev, "cannot remap register set\n");
		return -EADDRNOTAVAIL;
	}

	ddata->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ddata->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return PTR_ERR(ddata->clk);
	}

	dev_set_drvdata(&pdev->dev, ddata);

	ret = stmp_reset_block(ddata->base + REG_AUDIOOUT_CTRL);
	if (ret) {
		dev_err(&pdev->dev, "failed to reset audioout block\n");
		goto err_reginit;
	}

	ret = clk_prepare_enable(ddata->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clk\n");
		goto err_reginit;
	}

	writel(REG_AUDIOOUT_CTRL_DMAWAIT_COUNT(0x1f),
			ddata->base + REG_AUDIOOUT_CTRL);

	writel(REG_AUDIOOUT_TEST_HP_I1_ADJ_PLUS100,
			ddata->base + REG_AUDIOOUT_TEST);
	writel(0x03383f30, ddata->base + REG_AUDIOOUT_REFCTRL);
	/* chumby only clears CTRL_CLKGATE here */
	/* clear ANACLKCTRL_CLKGATE */
	writel(REG_AUDIOOUT_ANACLKCTRL_CLKGATE, ddata->base + REG_AUDIOOUT_ANACLKCTRL + STMP_OFFSET_REG_CLR);

	/* set capless mode */
	writel(REG_AUDIOOUT_PWRDN_CAPLESS, ddata->base + REG_AUDIOOUT_PWRDN + STMP_OFFSET_REG_CLR);

	/* power up dac */
	writel(REG_AUDIOOUT_PWRDN_DAC, ddata->base + REG_AUDIOOUT_PWRDN + STMP_OFFSET_REG_CLR);

	/* power up HP */
	writel(0x00000020, ddata->base + REG_AUDIOOUT_ANACLKCTRL + STMP_OFFSET_REG_SET);
	writel(REG_AUDIOOUT_PWRDN_HEADPHONE, ddata->base + REG_AUDIOOUT_PWRDN + STMP_OFFSET_REG_CLR);
	writel(REG_AUDIOOUT_ANACTRL_HP_CLASSAB, ddata->base + REG_AUDIOOUT_ANACTRL + STMP_OFFSET_REG_SET);
	writel(REG_AUDIOOUT_ANACTRL_HP_HOLD_GND, ddata->base + REG_AUDIOOUT_ANACTRL + STMP_OFFSET_REG_CLR);

	/* speaker */
	writel(REG_AUDIOOUT_REFCTRL_XTAL_BGR_BIAS, ddata->base + REG_AUDIOOUT_PWRDN + STMP_OFFSET_REG_CLR);
	writel(REG_AUDIOOUT_SPEAKERCTRL_MUTE, ddata->base + REG_AUDIOOUT_SPEAKERCTRL);

	dev_dbg(&pdev->dev, "register component\n");
	ret = snd_soc_register_component(&pdev->dev, &mxs_audioout_component,
			&mxs_audioout_component_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to register component (%d)\n", ret);
		goto err_register_component;
	}

	dev_dbg(&pdev->dev, "register codec\n");
	ret = snd_soc_register_codec(&pdev->dev, &mxs_audioout_codec,
			&mxs_audioout_codec_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to register codec (%d)\n", ret);
		goto err_register_codec;
	}

	dev_dbg(&pdev->dev, "register pcm\n");
	ret = mxs_pcm_platform_register(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register PCM (%d)\n", ret);
		snd_soc_unregister_codec(&pdev->dev);
err_register_codec:
		snd_soc_unregister_component(&pdev->dev);
	}

err_register_component:
err_reginit:
	return ret;
}

static const struct of_device_id mxs_audioout_dt_ids[] = {
	{ .compatible = "fsl,imx23-audioout", },
	{ /* sentinel */ }
};

static struct platform_driver mxs_audioout_driver = {
	.probe = mxs_audioout_probe,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mxs_audioout_dt_ids,
	},
};
module_platform_driver(mxs_audioout_driver);

MODULE_AUTHOR("Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>");
MODULE_DESCRIPTION("mxs audioout");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
