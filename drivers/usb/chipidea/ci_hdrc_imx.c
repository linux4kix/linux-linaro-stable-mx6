/*
 * Copyright 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2012 Marek Vasut <marex@denx.de>
 * on behalf of DENX Software Engineering GmbH
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
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <linux/usb/chipidea.h>
#include <linux/clk.h>
#include <linux/busfreq-imx6.h>

#include "ci.h"
#include "ci_hdrc_imx.h"

#define CI_HDRC_IMX_IMX28_WRITE_FIX		BIT(0)
#define CI_HDRC_IMX_SUPPORT_RUNTIME_PM		BIT(1)
#define CI_HDRC_IMX_HOST_QUIRK		BIT(2)

struct ci_hdrc_imx_platform_flag {
	unsigned int flags;
};

static const struct ci_hdrc_imx_platform_flag imx27_usb_data = {
};

static const struct ci_hdrc_imx_platform_flag imx23_usb_data = {
	.flags = CI_HDRC_IMX_HOST_QUIRK,
};

static const struct ci_hdrc_imx_platform_flag imx28_usb_data = {
	.flags = CI_HDRC_IMX_IMX28_WRITE_FIX |
		CI_HDRC_IMX_HOST_QUIRK,
};

static const struct ci_hdrc_imx_platform_flag imx6q_usb_data = {
	.flags = CI_HDRC_IMX_SUPPORT_RUNTIME_PM |
		CI_HDRC_IMX_HOST_QUIRK,
};

static const struct ci_hdrc_imx_platform_flag imx6sl_usb_data = {
	.flags = CI_HDRC_IMX_SUPPORT_RUNTIME_PM |
		CI_HDRC_IMX_HOST_QUIRK,
};

static const struct of_device_id ci_hdrc_imx_dt_ids[] = {
	{ .compatible = "fsl,imx6sl-usb", .data = &imx6sl_usb_data},
	{ .compatible = "fsl,imx6q-usb", .data = &imx6q_usb_data},
	{ .compatible = "fsl,imx28-usb", .data = &imx28_usb_data},
	{ .compatible = "fsl,imx23-usb", .data = &imx23_usb_data},
	{ .compatible = "fsl,imx27-usb", .data = &imx27_usb_data},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ci_hdrc_imx_dt_ids);

struct ci_hdrc_imx_data {
	struct usb_phy *phy;
	struct platform_device *ci_pdev;
	struct clk *clk;
	struct clk *clk_phy;
	struct imx_usbmisc_data *usbmisc_data;
	bool supports_runtime_pm;
	bool in_lpm;
};

/* Common functions shared by usbmisc drivers */

static struct imx_usbmisc_data *usbmisc_get_init_data(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct of_phandle_args args;
	struct imx_usbmisc_data *data;
	int ret;

	/*
	 * In case the fsl,usbmisc property is not present this device doesn't
	 * need usbmisc. Return NULL (which is no error here)
	 */
	if (!of_get_property(np, "fsl,usbmisc", NULL))
		return NULL;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return ERR_PTR(-ENOMEM);

	ret = of_parse_phandle_with_args(np, "fsl,usbmisc", "#index-cells",
					0, &args);
	if (ret) {
		dev_err(dev, "Failed to parse property fsl,usbmisc, errno %d\n",
			ret);
		return ERR_PTR(ret);
	}

	data->index = args.args[0];
	of_node_put(args.np);

	if (of_find_property(np, "disable-over-current", NULL))
		data->disable_oc = 1;

	if (of_find_property(np, "external-vbus-divider", NULL))
		data->evdo = 1;

	return data;
}

/* End of common functions shared by usbmisc drivers*/

static int ci_hdrc_imx_probe(struct platform_device *pdev)
{
	struct ci_hdrc_imx_data *data;
	struct ci_hdrc_platform_data pdata = {
		.name		= "ci_hdrc_imx",
		.capoffset	= DEF_CAPOFFSET,
		.flags		= CI_HDRC_REQUIRE_TRANSCEIVER |
				  CI_HDRC_DISABLE_STREAMING,
	};
	int ret;
	const struct of_device_id *of_id =
			of_match_device(ci_hdrc_imx_dt_ids, &pdev->dev);
	const struct ci_hdrc_imx_platform_flag *imx_platform_flag = of_id->data;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "Failed to allocate ci_hdrc-imx data!\n");
		return -ENOMEM;
	}

	data->usbmisc_data = usbmisc_get_init_data(&pdev->dev);
	if (IS_ERR(data->usbmisc_data))
		return PTR_ERR(data->usbmisc_data);

	data->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(data->clk)) {
		dev_err(&pdev->dev,
			"Failed to get clock, err=%ld\n", PTR_ERR(data->clk));
		return PTR_ERR(data->clk);
	}

	request_bus_freq(BUS_FREQ_HIGH);
	ret = clk_prepare_enable(data->clk);
	if (ret) {
		release_bus_freq(BUS_FREQ_HIGH);
		dev_err(&pdev->dev,
			"Failed to prepare or enable clock, err=%d\n", ret);
		return ret;
	}

	data->clk_phy = devm_clk_get(&pdev->dev, "phy");
	if (IS_ERR(data->clk_phy)) {
		data->clk_phy = NULL;
	} else {
		ret = clk_prepare_enable(data->clk_phy);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to enable clk_phy: %d\n", ret);
			goto err_clk;
		}
	}

	data->phy = devm_usb_get_phy_by_phandle(&pdev->dev, "fsl,usbphy", 0);
	if (IS_ERR(data->phy)) {
		ret = PTR_ERR(data->phy);
		goto err_clk_phy;
	}

	pdata.phy = data->phy;

	if (imx_platform_flag->flags & CI_HDRC_IMX_IMX28_WRITE_FIX)
		pdata.flags |= CI_HDRC_IMX28_WRITE_FIX;

	ret = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		goto err_clk;

	if (imx_platform_flag->flags & CI_HDRC_IMX_SUPPORT_RUNTIME_PM) {
		pdata.flags |= CI_HDRC_SUPPORTS_RUNTIME_PM;
		data->supports_runtime_pm = true;
	}

	if (imx_platform_flag->flags & CI_HDRC_IMX_HOST_QUIRK)
		pdata.flags |= CI_HDRC_IMX_EHCI_QUIRK;

	if (data->usbmisc_data) {
		ret = imx_usbmisc_init(data->usbmisc_data);
		if (ret) {
			dev_err(&pdev->dev, "usbmisc init failed, ret=%d\n",
					ret);
			goto err_clk;
		}
	}

	data->ci_pdev = ci_hdrc_add_device(&pdev->dev,
				pdev->resource, pdev->num_resources,
				&pdata);
	if (IS_ERR(data->ci_pdev)) {
		ret = PTR_ERR(data->ci_pdev);
		dev_err(&pdev->dev,
			"Can't register ci_hdrc platform device, err=%d\n",
			ret);
		goto err_clk;
	}

	/* usbmisc needs to know dr mode to choose wakeup setting */
	if (data->usbmisc_data)
		data->usbmisc_data->available_role =
			ci_hdrc_query_available_role(data->ci_pdev);

	if (data->usbmisc_data) {
		ret = imx_usbmisc_init_post(data->usbmisc_data);
		if (ret) {
			dev_err(&pdev->dev, "usbmisc post failed, ret=%d\n",
					ret);
			goto disable_device;
		}
	}

	if (data->usbmisc_data) {
		ret = imx_usbmisc_set_wakeup(data->usbmisc_data, false);
		if (ret) {
			dev_err(&pdev->dev, "usbmisc set_wakeup failed, ret=%d\n",
					ret);
			goto disable_device;
		}
	}

	platform_set_drvdata(pdev, data);

	device_set_wakeup_capable(&pdev->dev, true);

	if (data->supports_runtime_pm) {
		pm_runtime_set_active(&pdev->dev);
		pm_runtime_enable(&pdev->dev);
	}

	return 0;

disable_device:
	ci_hdrc_remove_device(data->ci_pdev);
err_clk_phy:
	if (data->clk_phy)
		clk_disable_unprepare(data->clk_phy);
err_clk:
	clk_disable_unprepare(data->clk);
	release_bus_freq(BUS_FREQ_HIGH);
	return ret;
}

static int ci_hdrc_imx_remove(struct platform_device *pdev)
{
	struct ci_hdrc_imx_data *data = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	ci_hdrc_remove_device(data->ci_pdev);
	if (data->clk_phy)
		clk_disable_unprepare(data->clk_phy);
	clk_disable_unprepare(data->clk);
	release_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

#ifdef CONFIG_PM
static int imx_controller_suspend(struct device *dev)
{
	struct ci_hdrc_imx_data *data = dev_get_drvdata(dev);
	int ret;

	dev_dbg(dev, "at %s\n", __func__);

	if (data->in_lpm)
		return 0;

	if (data->usbmisc_data) {
		ret = imx_usbmisc_set_wakeup(data->usbmisc_data, true);
		if (ret) {
			dev_err(dev,
				"usbmisc set_wakeup failed, ret=%d\n",
				ret);
			return ret;
		}
	}

	clk_disable_unprepare(data->clk);
	release_bus_freq(BUS_FREQ_HIGH);
	data->in_lpm = true;

	return 0;
}

static int imx_controller_resume(struct device *dev)
{
	struct ci_hdrc_imx_data *data = dev_get_drvdata(dev);
	int ret = 0;

	dev_dbg(dev, "at %s\n", __func__);

	if (!data->in_lpm)
		return 0;

	request_bus_freq(BUS_FREQ_HIGH);
	ret = clk_prepare_enable(data->clk);
	if (ret) {
		release_bus_freq(BUS_FREQ_HIGH);
		return ret;
	}

	data->in_lpm = false;

	if (data->usbmisc_data) {
		ret = imx_usbmisc_set_wakeup(data->usbmisc_data, false);
		if (ret) {
			dev_err(dev,
				"usbmisc set_wakeup failed, ret=%d\n",
				ret);
			ret = -EINVAL;
			goto clk_disable;
		}
	}

	return 0;

clk_disable:
	clk_disable_unprepare(data->clk);
	release_bus_freq(BUS_FREQ_HIGH);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int ci_hdrc_imx_suspend(struct device *dev)
{
	return imx_controller_suspend(dev);
}

static int ci_hdrc_imx_resume(struct device *dev)
{
	struct ci_hdrc_imx_data *data = dev_get_drvdata(dev);
	int ret;

	ret = imx_controller_resume(dev);
	if (!ret && data->supports_runtime_pm) {
		pm_runtime_disable(dev);
		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);
	}

	return ret;
}
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM_RUNTIME
static int ci_hdrc_imx_runtime_suspend(struct device *dev)
{
	return imx_controller_suspend(dev);
}

static int ci_hdrc_imx_runtime_resume(struct device *dev)
{
	return imx_controller_resume(dev);
}
#endif /* CONFIG_PM_RUNTIME */

#endif /* CONFIG_PM */
static const struct dev_pm_ops ci_hdrc_imx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ci_hdrc_imx_suspend, ci_hdrc_imx_resume)
	SET_RUNTIME_PM_OPS(ci_hdrc_imx_runtime_suspend,
			ci_hdrc_imx_runtime_resume, NULL)
};

static struct platform_driver ci_hdrc_imx_driver = {
	.probe = ci_hdrc_imx_probe,
	.remove = ci_hdrc_imx_remove,
	.driver = {
		.name = "imx_usb",
		.owner = THIS_MODULE,
		.of_match_table = ci_hdrc_imx_dt_ids,
		.pm = &ci_hdrc_imx_pm_ops,
	 },
};

module_platform_driver(ci_hdrc_imx_driver);

MODULE_ALIAS("platform:imx-usb");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CI HDRC i.MX USB binding");
MODULE_AUTHOR("Marek Vasut <marex@denx.de>");
MODULE_AUTHOR("Richard Zhao <richard.zhao@freescale.com>");
