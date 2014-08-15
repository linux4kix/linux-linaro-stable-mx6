/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *
 * Based on imx-sgtl5000.c
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2012 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_i2c.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/pinctrl/consumer.h>

#include "../codecs/wm8731.h"
#include "imx-audmux.h"
#include "imx-ssi.h"

#define DAI_NAME_SIZE	32
#define	WM8731_MCLK_FREQ	(24000000 / 2)

struct imx_wm8731_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	struct i2c_client *codec_dev;
	/* audio_clocking_data */
	struct clk *pll;
	struct clk *clock_root;
	long sysclk;
	long current_rate;
	/* apis */
	int (*clock_enable)(int enable,struct imx_wm8731_data *data);
};

static int imx_wm8731_init(struct snd_soc_pcm_runtime *rtd);
static int imx_hifi_hw_params_slv_mode(struct snd_pcm_substream *substream,
                                       struct snd_pcm_hw_params *params);
static void imx_hifi_shutdown(struct snd_pcm_substream *substream);

struct imx_priv {
	struct platform_device *pdev;
	struct imx_wm8731_data *data;
};

static struct imx_priv card_priv;

static struct snd_soc_ops imx_hifi_ops = {
	.shutdown	= imx_hifi_shutdown,
};

/* imx card dapm widgets */
static const struct snd_soc_dapm_widget imx_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack",       NULL),
	SND_SOC_DAPM_SPK("Ext Spk",             NULL),
	SND_SOC_DAPM_LINE("Line Jack",          NULL),
	SND_SOC_DAPM_MIC("Mic Jack",            NULL),
};

/* imx machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	{ "Headphone Jack",     NULL,   "LHPOUT" },
	{ "Headphone Jack",     NULL,   "RHPOUT" },

	{ "Ext Spk",            NULL,   "LOUT" },
	{ "Ext Spk",            NULL,   "ROUT" },

	{ "LLINEIN",            NULL,   "Line Jack" },
	{ "RLINEIN",            NULL,   "Line Jack" },

	{ "MICIN",              NULL,   "Mic Bias" },
	{ "Mic Bias",           NULL,   "Mic Jack"},
};

static int wm8731_slv_mode_init(struct imx_wm8731_data *data)
{
	struct clk *new_parent;
	struct clk *ssi_clk;
	struct i2c_client *codec_dev = data->codec_dev;

	new_parent = devm_clk_get(&codec_dev->dev, "pll4");
	if (IS_ERR(new_parent)) {
		pr_err("Could not get \"pll4\" clock \n");
		return PTR_ERR(new_parent);
	}

	ssi_clk = devm_clk_get(&codec_dev->dev, "imx-ssi.1");
	if (IS_ERR(ssi_clk)) {
		pr_err("Could not get \"imx-ssi.1\" clock \n");
		return PTR_ERR(ssi_clk);
	}

	clk_set_parent(ssi_clk, new_parent);

	data->pll = new_parent;
	data->clock_root = ssi_clk;
	data->current_rate = 0;

	data->sysclk = 0;

	return 0;
}

static int wm8731_slv_mode_clock_enable(int enable, struct imx_wm8731_data *data)
{
	long pll_rate;
	long rate_req;
	long rate_avail;

	if (!enable)
		return 0;

	if (data->sysclk == data->current_rate)
		return 0;

	switch (data->sysclk) {
		case 11289600:
			pll_rate = 632217600;
			break;

		case 12288000:
			pll_rate = 688128000;
			break;

		default:
			return -EINVAL;
	}

	rate_req = pll_rate;
	rate_avail = clk_round_rate(data->pll, rate_req);
	clk_set_rate(data->pll, rate_avail);

	rate_req = data->sysclk;
	rate_avail = clk_round_rate(data->clock_root,
								rate_req);
	clk_set_rate(data->clock_root, rate_avail);

	pr_info("%s: \"imx-ssi.1\" rate = %ld (= %ld)\n",
			__func__, rate_avail, rate_req);

	data->current_rate = data->sysclk;

	return 0;
}

static int imx_hifi_startup_slv_mode(struct snd_pcm_substream *substream)
{
	/*
	 * As SSI's sys clock rate depends on sampling rate,
	 * the clock enabling code is moved to imx_hifi_hw_params().
	 */
	return 0;
}

static int wm8731_mst_mode_init(struct imx_wm8731_data *data)
{
	long rate;
	struct clk *new_parent;
	struct clk *ssi_clk;
	struct i2c_client *codec_dev = data->codec_dev;

	new_parent = devm_clk_get(&codec_dev->dev, "cko2");
	if (IS_ERR(new_parent)) {
		pr_err("Could not get \"cko2\" clock \n");
		return PTR_ERR(new_parent);
	}

	ssi_clk = devm_clk_get(&codec_dev->dev, "cko");
	if (IS_ERR(ssi_clk)) {
		pr_err("Could not get \"cko\" clock \n");
		return PTR_ERR(ssi_clk);
	}

	rate = clk_round_rate(new_parent, WM8731_MCLK_FREQ);
	clk_set_rate(new_parent, rate);

	clk_set_parent(ssi_clk, new_parent);

	rate = clk_round_rate(ssi_clk, WM8731_MCLK_FREQ);
	clk_set_rate(ssi_clk, rate);

	pr_info("%s: \"CLKO\" rate = %ld (= %d)\n",
		__func__, rate, WM8731_MCLK_FREQ);

	data->pll = new_parent;
	data->clock_root = ssi_clk;
	data->sysclk = rate;

	return 0;
}

static int wm8731_mst_mode_clock_enable(int enable, struct imx_wm8731_data *data)
{
	struct clk *clko = data->clock_root;

	if (enable)
		clk_enable(clko);
	else
		clk_disable(clko);

	return 0;
}

static int imx_hifi_startup_mst_mode(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = codec_dai->codec->card;
	struct imx_wm8731_data *data = snd_soc_card_get_drvdata(card);

	if (!codec_dai->active)
		data->clock_enable(1,data);

	return 0;
}


static int imx_hifi_hw_params_slv_mode(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = codec_dai->codec->card;
	struct imx_wm8731_data *data = snd_soc_card_get_drvdata(card);
	
	u32 dai_format;
	snd_pcm_format_t sample_format;
	unsigned int channels;
	unsigned int tx_mask, rx_mask;
	unsigned int sampling_rate;
	unsigned int div_2, div_psr, div_pm;
	int ret;

	sampling_rate = params_rate(params);
	sample_format = params_format(params);
	
	channels = params_channels(params);
	printk("%s:%s  sampling rate = %u  channels = %u \n", __FUNCTION__,
		   (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "Playback" : "Capture"),
		   sampling_rate, channels);

	/* set CPU DAI configuration */
	switch (sampling_rate) {
		case 8000:
		case 32000:
		case 48000:
		case 96000:
			data->sysclk = 12288000;
			break;

		case 44100:
		case 88200:
			data->sysclk = 11289600;
			break;

		default:
			return -EINVAL;
	}

	wm8731_slv_mode_clock_enable(1,data);

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
		SND_SOC_DAIFMT_CBS_CFS;

	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set i.MX active slot mask */
	/* S[TR]CCR:DC */
	tx_mask = ~((1 << channels) - 1);
	rx_mask = tx_mask;
	snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask, rx_mask, 2, 32);

	/*
	 * SSI sysclk divider:
	 * div_2:	/1 or /2
	 * div_psr:	/1 or /8
	 * div_pm:	/1 .. /256
	 */
	div_2	= 0;
	div_psr	= 0;
	switch (sampling_rate) {
		case 8000:
			// 1x1x12
			div_pm	= 11;
			break;
		case 32000:
			// 1x1x3
			div_pm	= 2;
			break;
		case 48000:
			// 1x1x2
			div_pm	= 1;
			break;
		case 96000:
			// 1x1x1
			div_pm	= 0;
			break;
		case 44100:
			// 1x1x2
			div_pm	= 1;
			break;
		case 88200:
			// 1x1x1
			div_pm	= 0;
			break;
		default:
			return -EINVAL;
	}

	/* sync mode: a single clock controls both playback and capture */
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_2, (div_2 ? SSI_STCCR_DIV2 : 0));
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_PSR, (div_psr ? SSI_STCCR_PSR : 0));
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_PM, div_pm);

	/* set codec DAI configuration */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBS_CFS;

	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai,
				     WM8731_SYSCLK_MCLK,
				     data->sysclk,
				     SND_SOC_CLOCK_IN);

	if (ret < 0) {
		pr_err("Failed to set codec master clock to %u: %d \n",
		       data->sysclk, ret);
		return ret;
	}

	return 0;
}

static int imx_hifi_hw_params_mst_mode(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = codec_dai->codec->card;
	struct imx_wm8731_data *data = snd_soc_card_get_drvdata(card);
	u32 dai_format;
	unsigned int channels;
	unsigned int tx_mask, rx_mask;
	unsigned int sampling_rate;
	int ret;


	sampling_rate = params_rate(params);
	channels = params_channels(params);
	pr_debug("%s:%s  sampling rate = %u  channels = %u \n", __FUNCTION__,
		 (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "Playback" : "Capture"),
		 sampling_rate, channels);

	/* set cpu DAI configuration */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
		SND_SOC_DAIFMT_CBM_CFM;

	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set i.MX active slot mask */
	/* S[TR]CCR:DC */
	tx_mask = ~((1 << channels) - 1);
	rx_mask = tx_mask;
	snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask, rx_mask, 2, 32);

	/* set codec DAI configuration */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM;

	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai,
				     WM8731_SYSCLK_MCLK,
				     data->sysclk,
				     SND_SOC_CLOCK_IN);

	if (ret < 0) {
		pr_err("Failed to set codec master clock to %u: %d \n",
		       data->sysclk, ret);
		return ret;
	}

	return 0;
}

static void imx_hifi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = codec_dai->codec->card;
	struct imx_wm8731_data *data = snd_soc_card_get_drvdata(card);
	
	if (!codec_dai->active)
		data->clock_enable(0,data);
	
	return;
}

static int imx_wm8731_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	struct snd_soc_codec *codec = rtd->codec;

	/* Add imx specific widgets */
	ret = snd_soc_dapm_new_controls(&codec->dapm, imx_dapm_widgets,
									ARRAY_SIZE(imx_dapm_widgets));
	if (ret)
			goto out_retcode;

	/* Set up imx specific audio path audio_map */
	ret = snd_soc_dapm_add_routes(&codec->dapm, audio_map, ARRAY_SIZE(audio_map));
	if (ret)
			goto out_retcode;

	ret = snd_soc_dapm_enable_pin(&codec->dapm, "Headphone Jack");
	if (ret)
			goto out_retcode;

	ret = snd_soc_dapm_nc_pin(&codec->dapm, "Ext Spk");
	if (ret)
			goto out_retcode;

out_retcode:

	if (ret)
			pr_err("%s: failed with error code: %d \n", __FUNCTION__, ret);
	else
			pr_info("%s: success \n", __FUNCTION__);

	return ret;
}

/**
 * Configure AUDMUX interconnection between
 * _slave (CPU side) and _master (codec size)
 *
 * When SSI operates in master mode, 5-wire interconnect with
 * audio codec is required:
 * TXC  - BCLK
 * TXD  - DAC data
 * RXD  - ADC data
 * TXFS - {DAC|ADC}LRC, i.e. word clock
 * RXC  - MCLK, i.e. oversampling clock
 * Audmux is operated in asynchronous mode to enable 6-wire
 * interface (as opposed to 4-wire interface in sync mode).
 */
static int imx_audmux_config_slv_mode(int _slave, int _master)
{
	unsigned int ptcr, pdcr;
	int slave = _slave - 1;
	int master = _master - 1;

	ptcr = IMX_AUDMUX_V2_PTCR_SYN |
		IMX_AUDMUX_V2_PTCR_TFSDIR |
		IMX_AUDMUX_V2_PTCR_TFSEL(slave) |
		IMX_AUDMUX_V2_PTCR_RCLKDIR |
		IMX_AUDMUX_V2_PTCR_RCSEL(slave | 0x8) |
		IMX_AUDMUX_V2_PTCR_TCLKDIR |
		IMX_AUDMUX_V2_PTCR_TCSEL(slave);

	pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(slave);
	imx_audmux_v2_configure_port(master, ptcr, pdcr);
	ptcr = ptcr & ~IMX_AUDMUX_V2_PTCR_SYN;
	imx_audmux_v2_configure_port(master, ptcr, pdcr);

	ptcr = IMX_AUDMUX_V2_PTCR_SYN |
		IMX_AUDMUX_V2_PTCR_RCLKDIR |
		IMX_AUDMUX_V2_PTCR_RCSEL(master | 0x8) |
		IMX_AUDMUX_V2_PTCR_TCLKDIR |
		IMX_AUDMUX_V2_PTCR_TCSEL(master);

	pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(master);
	imx_audmux_v2_configure_port(slave, ptcr, pdcr);
	ptcr = ptcr & ~IMX_AUDMUX_V2_PTCR_SYN;
	imx_audmux_v2_configure_port(slave, ptcr, pdcr);

	return 0;
}

static int imx_audmux_config_mst_mode(int _slave, int _master)
{
	unsigned int ptcr, pdcr;
	int slave = _slave - 1;
	int master = _master - 1;

	ptcr = IMX_AUDMUX_V2_PTCR_SYN;
	ptcr |= IMX_AUDMUX_V2_PTCR_TFSDIR |
		IMX_AUDMUX_V2_PTCR_TFSEL(master) |
		IMX_AUDMUX_V2_PTCR_TCLKDIR |
		IMX_AUDMUX_V2_PTCR_TCSEL(master);
	pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(master);
	imx_audmux_v2_configure_port(slave, ptcr, pdcr);

	ptcr = IMX_AUDMUX_V2_PTCR_SYN;
	pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(slave);
	imx_audmux_v2_configure_port(master, ptcr, pdcr);

	return 0;
}

static int imx_wm8731_probe(struct platform_device *pdev)
{
	struct device_node *ssi_np, *codec_np;
	struct platform_device *ssi_pdev;
	struct imx_priv *priv = &card_priv;
	struct i2c_client *codec_dev;
	struct imx_wm8731_data *data;
	unsigned int src_port, ext_port;
	unsigned int ssi_mode;
	const char *ssi_mode_str;

	int ret;

	priv->pdev = pdev;
	
	ssi_np = of_parse_phandle(pdev->dev.of_node, "ssi-controller", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!ssi_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	ssi_pdev = of_find_device_by_node(ssi_np);
	if (!ssi_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EINVAL;
		goto fail;
	}
        
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev || !codec_dev->driver) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	card_priv.data = data;

	data->codec_dev = codec_dev;
	
	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.codec_dai_name = "wm8731-hifi";
	data->dai.codec_of_node = codec_np;
	data->dai.cpu_dai_name = dev_name(&ssi_pdev->dev);
	data->dai.platform_of_node = ssi_np;
	data->dai.ops = &imx_hifi_ops;
	data->dai.init = &imx_wm8731_init;
	
	ret = of_property_read_u32(pdev->dev.of_node, "src-port", &src_port);
	if (ret) {
		dev_err(&pdev->dev, "failed to get \"src-port\" value\n");
		ret = -EINVAL;
		goto fail;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "ext-port", &ext_port);
	if (ret) {
		dev_err(&pdev->dev, "failed to get \"ext-port\" value\n");
		ret = -EINVAL;
		goto fail;
	}

	ret = of_property_read_string(ssi_np, "fsl,mode", &ssi_mode_str);
	if (ret) {
		dev_err(&pdev->dev, "failed to get \"fsl,mode\" value\n");
		ret = -EINVAL;
		goto fail;
	}

	ssi_mode = strcmp(ssi_mode_str, "i2s-master");

	if (ssi_mode) {
		/* Master Mode */
		imx_audmux_config_mst_mode(src_port, ext_port);
		wm8731_mst_mode_init(data);
		data->clock_enable = wm8731_mst_mode_clock_enable;
		imx_hifi_ops.hw_params = imx_hifi_hw_params_mst_mode;
		imx_hifi_ops.startup = imx_hifi_startup_mst_mode;
	} else {
		/* Slave Mode */
		imx_audmux_config_slv_mode(src_port, ext_port);
		wm8731_slv_mode_init(data);
		data->clock_enable = wm8731_slv_mode_clock_enable;
		imx_hifi_ops.hw_params = imx_hifi_hw_params_slv_mode;
		imx_hifi_ops.startup = imx_hifi_startup_slv_mode;
	}
	
	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;
	
	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;

	data->card.num_links = 1;
	data->card.dai_link = &data->dai;

	data->card.dapm_widgets = imx_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_dapm_widgets);

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = snd_soc_register_card(&data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}
	
	return 0;

fail:

	if (ssi_np)
		of_node_put(ssi_np);

	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static int imx_wm8731_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id imx_wm8731_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-wm8731", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_wm8731_dt_ids);

static struct platform_driver imx_wm8731_driver = {
	.driver = {
		.name = "imx-wm8731",
		.owner = THIS_MODULE,
		.of_match_table = imx_wm8731_dt_ids,
	},
	.probe = imx_wm8731_probe,
	.remove = imx_wm8731_remove,
};
module_platform_driver(imx_wm8731_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale i.MX WM8731 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-wm8731");
