/*
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/device_cooling.h>
#include <linux/fsl_otp.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/types.h>

#define REG_SET		0x4
#define REG_CLR		0x8
#define REG_TOG		0xc

#define MISC0				0x0150
#define MISC0_REFTOP_SELBIASOFF		(1 << 3)

#define TEMPSENSE0			0x0180
#define TEMPSENSE0_ALARM_VALUE_SHIFT	20
#define TEMPSENSE0_ALARM_VALUE_MASK	(0xfff << TEMPSENSE0_ALARM_VALUE_SHIFT)
#define TEMPSENSE0_TEMP_CNT_SHIFT	8
#define TEMPSENSE0_TEMP_CNT_MASK	(0xfff << TEMPSENSE0_TEMP_CNT_SHIFT)
#define TEMPSENSE0_FINISHED		(1 << 2)
#define TEMPSENSE0_MEASURE_TEMP		(1 << 1)
#define TEMPSENSE0_POWER_DOWN		(1 << 0)

#define TEMPSENSE1			0x0190
#define TEMPSENSE1_MEASURE_FREQ		0xffff

#define OCOTP_ANA1			0x04e0

#define OCOTP_TEMP_GRADE		0x480
#define OCOTP_TEMP_GRADE_SHIFT		5
#define OCOTP_TEMP_GRADE_AUT		0x3
#define OCOTP_TEMP_GRADE_IND		0x2
#define OCOTP_TEMP_GRADE_EXT		0x1
#define OCOTP_TEMP_GRADE_COM		0x0

/*
 * It defines the temperature in millicelsius for passive trip point
 * that will trigger cooling action when crossed.
 */
#define IMX_TEMP_MAX_PASSIVE		85000
#define IMX_TEMP_MIN_TRIP_DELTA		6000

#define IMX_POLLING_DELAY		3000 /* millisecond */
#define IMX_PASSIVE_DELAY		2000

#define FACTOR0				10000000
#define FACTOR1				15976
#define FACTOR2				4297157

#define IMX_TRIP_PASSIVE	0

struct imx_thermal_data {
	struct thermal_zone_device *tz;
	struct thermal_cooling_device *cdev[2];
	enum thermal_device_mode mode;
	struct regmap *tempmon;
	u32 c1, c2; /* See formula in imx_get_sensor_data() */
	unsigned long temp_passive;
	unsigned long temp_critical;
	unsigned long num_passive_trips;
	unsigned long temp_zone_delta;
	unsigned long alarm_temp;
	unsigned long last_temp;
	bool irq_enabled;
	int irq;
	struct clk *thermal_clk;
};

static void imx_set_alarm_temp(struct imx_thermal_data *data,
			       signed long alarm_temp)
{
	struct regmap *map = data->tempmon;
	int alarm_value;

	data->alarm_temp = alarm_temp;
	alarm_value = (data->c2 - alarm_temp) / data->c1;
	regmap_write(map, TEMPSENSE0 + REG_CLR, TEMPSENSE0_ALARM_VALUE_MASK);
	regmap_write(map, TEMPSENSE0 + REG_SET, alarm_value <<
			TEMPSENSE0_ALARM_VALUE_SHIFT);
}

static int imx_get_temp(struct thermal_zone_device *tz, unsigned long *temp)
{
	struct imx_thermal_data *data = tz->devdata;
	struct regmap *map = data->tempmon;
	unsigned int n_meas;
	unsigned long cur_state;
	bool wait;
	u32 val;

	if (data->mode == THERMAL_DEVICE_ENABLED) {
		/* Check if a measurement is currently in progress */
		regmap_read(map, TEMPSENSE0, &val);
		wait = !(val & TEMPSENSE0_FINISHED);
	} else {
		/*
		 * Every time we measure the temperature, we will power on the
		 * temperature sensor, enable measurements, take a reading,
		 * disable measurements, power off the temperature sensor.
		 */
		regmap_write(map, TEMPSENSE0 + REG_CLR, TEMPSENSE0_POWER_DOWN);
		regmap_write(map, TEMPSENSE0 + REG_SET, TEMPSENSE0_MEASURE_TEMP);

		wait = true;
	}

	/*
	 * According to the temp sensor designers, it may require up to ~17us
	 * to complete a measurement.
	 */
	if (wait)
		usleep_range(20, 50);

	regmap_read(map, TEMPSENSE0, &val);

	if (data->mode != THERMAL_DEVICE_ENABLED) {
		regmap_write(map, TEMPSENSE0 + REG_CLR, TEMPSENSE0_MEASURE_TEMP);
		regmap_write(map, TEMPSENSE0 + REG_SET, TEMPSENSE0_POWER_DOWN);
	}

	if ((val & TEMPSENSE0_FINISHED) == 0) {
		dev_dbg(&tz->device, "temp measurement never finished\n");
		return -EAGAIN;
	}

	n_meas = (val & TEMPSENSE0_TEMP_CNT_MASK) >> TEMPSENSE0_TEMP_CNT_SHIFT;

	/* See imx_get_sensor_data() for formula derivation */
	*temp = data->c2 - n_meas * data->c1;

	data->cdev[0]->ops->get_cur_state(data->cdev[0], &cur_state);

	/* Update alarm value to next higher trip point */
	if ((data->temp_passive < data->alarm_temp)  && 
	    (data->alarm_temp < data->temp_critical) &&
	    (cur_state < data->num_passive_trips)) {
		imx_set_alarm_temp(data, data->temp_passive + ((cur_state + 1) * data->temp_zone_delta));
		dev_dbg(&tz->device, "thermal alarm on: T < %lu\n",
			data->alarm_temp / 1000);
	}

	if (data->alarm_temp < data->temp_critical && *temp >= data->temp_passive + (data->num_passive_trips * data->temp_zone_delta))
		imx_set_alarm_temp(data, data->temp_critical);

	if (data->alarm_temp > data->temp_passive && *temp < data->temp_passive) {
		imx_set_alarm_temp(data, data->temp_passive);
		dev_dbg(&tz->device, "thermal alarm off: T < %lu\n",
			data->alarm_temp / 1000);
	}

	if (*temp != data->last_temp) {
		dev_dbg(&tz->device, "millicelsius: %ld\n", *temp);
		data->last_temp = *temp;
	}

	/* Reenable alarm IRQ if temperature below alarm temperature */
	if (!data->irq_enabled && *temp < data->alarm_temp) {
		data->irq_enabled = true;
		enable_irq(data->irq);
	}

	return 0;
}

static int imx_get_mode(struct thermal_zone_device *tz,
			enum thermal_device_mode *mode)
{
	struct imx_thermal_data *data = tz->devdata;

	*mode = data->mode;

	return 0;
}

static int imx_set_mode(struct thermal_zone_device *tz,
			enum thermal_device_mode mode)
{
	struct imx_thermal_data *data = tz->devdata;
	struct regmap *map = data->tempmon;

	if (mode == THERMAL_DEVICE_ENABLED) {
		tz->polling_delay = IMX_POLLING_DELAY;
		tz->passive_delay = IMX_PASSIVE_DELAY;

		regmap_write(map, TEMPSENSE0 + REG_CLR, TEMPSENSE0_POWER_DOWN);
		regmap_write(map, TEMPSENSE0 + REG_SET, TEMPSENSE0_MEASURE_TEMP);

		if (!data->irq_enabled) {
			data->irq_enabled = true;
			enable_irq(data->irq);
		}
	} else {
		regmap_write(map, TEMPSENSE0 + REG_CLR, TEMPSENSE0_MEASURE_TEMP);
		regmap_write(map, TEMPSENSE0 + REG_SET, TEMPSENSE0_POWER_DOWN);

		tz->polling_delay = 0;
		tz->passive_delay = 0;

		if (data->irq_enabled) {
			disable_irq(data->irq);
			data->irq_enabled = false;
		}
	}

	data->mode = mode;
	thermal_zone_device_update(tz);

	return 0;
}

static int imx_get_trip_type(struct thermal_zone_device *tz, int trip,
			     enum thermal_trip_type *type)
{
	struct imx_thermal_data *data = tz->devdata;
	*type = (trip < data->num_passive_trips) ? THERMAL_TRIP_PASSIVE :
					     THERMAL_TRIP_CRITICAL;
	return 0;
}

static int imx_get_crit_temp(struct thermal_zone_device *tz,
			     unsigned long *temp)
{
	struct imx_thermal_data *data = tz->devdata;

	*temp = data->temp_critical;
	return 0;
}

static int imx_get_trip_temp(struct thermal_zone_device *tz, int trip,
			     unsigned long *temp)
{
	struct imx_thermal_data *data = tz->devdata;

	*temp = (trip < data->num_passive_trips) ?
			data->temp_passive + (trip * data->temp_zone_delta) :
			data->temp_critical;
	return 0;
}

static int imx_set_trip_temp(struct thermal_zone_device *tz, int trip,
			     unsigned long temp)
{
	struct imx_thermal_data *data = tz->devdata;

	if (trip > IMX_TRIP_PASSIVE)
		return -EPERM;

	if (trip == IMX_TRIP_PASSIVE && temp > IMX_TEMP_MAX_PASSIVE)
		return -EINVAL;

	data->temp_passive = temp;
	data->temp_zone_delta = (data->temp_critical - data->temp_passive) / data->num_passive_trips;

	imx_set_alarm_temp(data, temp);

	return 0;
}

static int imx_bind(struct thermal_zone_device *tz,
		    struct thermal_cooling_device *cdev)
{
	int ret;

	ret = thermal_zone_bind_cooling_device(tz, IMX_TRIP_PASSIVE, cdev,
					       THERMAL_NO_LIMIT,
					       THERMAL_NO_LIMIT);
	if (ret) {
		dev_err(&tz->device,
			"binding zone %s with cdev %s failed:%d\n",
			tz->type, cdev->type, ret);
		return ret;
	}

	return 0;
}

static int imx_unbind(struct thermal_zone_device *tz,
		      struct thermal_cooling_device *cdev)
{
	int ret;

	ret = thermal_zone_unbind_cooling_device(tz, IMX_TRIP_PASSIVE, cdev);
	if (ret) {
		dev_err(&tz->device,
			"unbinding zone %s with cdev %s failed:%d\n",
			tz->type, cdev->type, ret);
		return ret;
	}

	return 0;
}

 int imx_get_trend(struct thermal_zone_device *tz,
	 int trip, enum thermal_trend *trend)
{
	struct imx_thermal_data *data = tz->devdata;
	int ret;
	unsigned long trip_temp, cur_state;

	ret = imx_get_trip_temp(tz, trip, &trip_temp);
	if (ret < 0)
		return ret;

	data->cdev[0]->ops->get_cur_state(data->cdev[0], &cur_state);

	if (tz->temperature > tz->last_temperature &&
	    tz->temperature > (data->temp_passive + (cur_state * data->temp_zone_delta))) {
		*trend = THERMAL_TREND_RAISING;
	} else if (tz->temperature < tz->last_temperature && cur_state) {
		if (tz->temperature <= (data->temp_passive - data->temp_zone_delta))
			*trend = THERMAL_TREND_DROP_FULL;
		else if (tz->temperature <= (data->temp_passive +
					     ((cur_state - 1) * data->temp_zone_delta)))
			*trend = THERMAL_TREND_DROPPING;
		else
			*trend = THERMAL_TREND_STABLE;
	} else {
		*trend = THERMAL_TREND_STABLE;
	}

	return 0;
}

static struct thermal_zone_device_ops imx_tz_ops = {
	.bind = imx_bind,
	.unbind = imx_unbind,
	.get_temp = imx_get_temp,
	.get_mode = imx_get_mode,
	.set_mode = imx_set_mode,
	.get_trip_type = imx_get_trip_type,
	.get_trip_temp = imx_get_trip_temp,
	.get_crit_temp = imx_get_crit_temp,
	.get_trend = imx_get_trend,
	.set_trip_temp = imx_set_trip_temp,
};

static int imx_get_sensor_data(struct platform_device *pdev)
{
	struct imx_thermal_data *data = platform_get_drvdata(pdev);
	struct regmap *map;
	int t1, n1;
	int ret;
	u32 val;
	u64 temp64;

	map = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
					      "fsl,tempmon-data");
	if (IS_ERR(map)) {
		ret = PTR_ERR(map);
		dev_err(&pdev->dev, "failed to get sensor regmap: %d\n", ret);
		return ret;
	}

	ret = regmap_read(map, OCOTP_ANA1, &val);
	if (ret) {
		dev_err(&pdev->dev, "failed to read sensor data: %d\n", ret);
		return ret;
	}

	if (val == 0 || val == ~0) {
		dev_err(&pdev->dev, "invalid sensor calibration data\n");
		return -EINVAL;
	}

	/*
	 * Sensor data layout:
	 *   [31:20] - sensor value @ 25C
	 * Use universal formula now and only need sensor value @ 25C
	 * slope = 0.4297157 - (0.0015976 * 25C fuse)
	 */
	n1 = val >> 20;
	t1 = 25; /* t1 always 25C */

	/*
	 * Derived from linear interpolation:
	 * slope = 0.4297157 - (0.0015976 * 25C fuse)
	 * slope = (FACTOR2 - FACTOR1 * n1) / FACTOR0
	 * (Nmeas - n1) / (Tmeas - t1) = slope
	 * We want to reduce this down to the minimum computation necessary
	 * for each temperature read.  Also, we want Tmeas in millicelsius
	 * and we don't want to lose precision from integer division. So...
	 * Tmeas = (Nmeas - n1) / slope + t1
	 * milli_Tmeas = 1000 * (Nmeas - n1) / slope + 1000 * t1
	 * milli_Tmeas = -1000 * (n1 - Nmeas) / slope + 1000 * t1
	 * Let constant c1 = (-1000 / slope)
	 * milli_Tmeas = (n1 - Nmeas) * c1 + 1000 * t1
	 * Let constant c2 = n1 *c1 + 1000 * t1
	 * milli_Tmeas = c2 - Nmeas * c1
	 */
	temp64 = FACTOR0;
	temp64 *= 1000;
	do_div(temp64, FACTOR1 * n1 - FACTOR2);
	data->c1 = temp64;
	data->c2 = n1 * data->c1 + 1000 * t1;

	return 0;
}

static void imx_set_thermal_defaults(struct imx_thermal_data *data)
{
	int ret;
	u32 val;

	ret = fsl_otp_readl(OCOTP_TEMP_GRADE, &val);

	if (ret) {
		/*
		 * Set the default passive cooling trip point,
		 * can be changed from userspace.
		 */
		data->temp_passive = IMX_TEMP_MAX_PASSIVE;

		/*
		 * The maximum die temperature set to 20 C higher than
		 * IMX_TEMP_MAX_PASSIVE.
		 */
		data->temp_critical = 1000 * 20 + data->temp_passive;
		data->temp_zone_delta = (data->temp_critical - data->temp_passive) / data->num_passive_trips;
	} else {
		val >>= OCOTP_TEMP_GRADE_SHIFT;
		val &= 0x3;

		switch (val) {
		case OCOTP_TEMP_GRADE_AUT:
			data->temp_critical = 125000;
			break;
		case OCOTP_TEMP_GRADE_IND:
		case OCOTP_TEMP_GRADE_EXT:
			data->temp_critical = 105000;
			break;
		case OCOTP_TEMP_GRADE_COM:
		default:
			data->temp_critical = 95000;
			break;
		}
		data->temp_passive = data->temp_critical - (IMX_TEMP_MIN_TRIP_DELTA * data->num_passive_trips);
		data->temp_zone_delta = IMX_TEMP_MIN_TRIP_DELTA;
	}

	pr_debug("THERMAL DEFAULTS: passive: %lu \
		 critical %lu trip_points: %lu \
		 zone_delta: %lu\n", 
		 data->temp_passive, data->temp_critical,
		 data->num_passive_trips, data->temp_zone_delta);
}

static irqreturn_t imx_thermal_alarm_irq(int irq, void *dev)
{
	struct imx_thermal_data *data = dev;

	disable_irq_nosync(irq);
	data->irq_enabled = false;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t imx_thermal_alarm_irq_thread(int irq, void *dev)
{
	struct imx_thermal_data *data = dev;

	dev_dbg(&data->tz->device, "THERMAL ALARM: T > %lu\n",
		data->alarm_temp / 1000);

	thermal_zone_device_update(data->tz);

	return IRQ_HANDLED;
}

static int imx_thermal_probe(struct platform_device *pdev)
{
	struct imx_thermal_data *data;
	struct cpumask clip_cpus;
	struct regmap *map;
	int measure_freq;
	int ret;

	if (!cpufreq_get_current_driver()) {
		dev_dbg(&pdev->dev, "no cpufreq driver!");
		return -EPROBE_DEFER;
	}
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	map = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "fsl,tempmon");
	if (IS_ERR(map)) {
		ret = PTR_ERR(map);
		dev_err(&pdev->dev, "failed to get tempmon regmap: %d\n", ret);
		return ret;
	}
	data->tempmon = map;

	data->irq = platform_get_irq(pdev, 0);
	if (data->irq < 0)
		return data->irq;

	ret = devm_request_threaded_irq(&pdev->dev, data->irq,
			imx_thermal_alarm_irq, imx_thermal_alarm_irq_thread,
			0, "imx_thermal", data);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request alarm irq: %d\n", ret);
		return ret;
	}

	data->irq_enabled = true;

	data->thermal_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(data->thermal_clk)) {
		dev_warn(&pdev->dev, "failed to get thermal clk!\n");
	} else {
		/*
		 * Thermal sensor needs clk on to get correct value, normally
		 * we should enable its clk before taking measurement and disable
		 * clk after measurement is done, but if alarm function is enabled,
		 * hardware will auto measure the temperature periodically, so we
		 * need to keep the clk always on for alarm function.
		 */
		ret = clk_prepare_enable(data->thermal_clk);
		if (ret)
			dev_warn(&pdev->dev, "failed to enable thermal clk: %d\n", ret);
	}

	platform_set_drvdata(pdev, data);

	ret = imx_get_sensor_data(pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to get sensor data\n");
		return ret;
	}

	/* Make sure sensor is in known good state for measurements */
	regmap_write(map, TEMPSENSE0 + REG_CLR, TEMPSENSE0_POWER_DOWN);
	regmap_write(map, TEMPSENSE0 + REG_CLR, TEMPSENSE0_MEASURE_TEMP);
	regmap_write(map, TEMPSENSE1 + REG_CLR, TEMPSENSE1_MEASURE_FREQ);
	regmap_write(map, MISC0 + REG_SET, MISC0_REFTOP_SELBIASOFF);
	regmap_write(map, TEMPSENSE0 + REG_SET, TEMPSENSE0_POWER_DOWN);

	cpumask_set_cpu(0, &clip_cpus);
	data->cdev[0] = cpufreq_cooling_register(&clip_cpus);
	if (IS_ERR(data->cdev[0])) {
		ret = PTR_ERR(data->cdev[0]);
		dev_err(&pdev->dev,
			"failed to register cpufreq cooling device: %d\n", ret);
		return ret;
	}

	data->cdev[0]->ops->get_max_state(data->cdev[0], &data->num_passive_trips);

	data->cdev[1] = devfreq_cooling_register(data->num_passive_trips + 1);
	if (IS_ERR(data->cdev[1])) {
		ret = PTR_ERR(data->cdev[1]);
		dev_err(&pdev->dev,
			"failed to register devfreq cooling device: %d\n", ret);
		return ret;
	}

	imx_set_thermal_defaults(data);

	data->tz = thermal_zone_device_register("imx_thermal_zone",
						data->num_passive_trips + 1,
						BIT(IMX_TRIP_PASSIVE), data,
						&imx_tz_ops, NULL,
						IMX_PASSIVE_DELAY,
						IMX_POLLING_DELAY);
	if (IS_ERR(data->tz)) {
		ret = PTR_ERR(data->tz);
		dev_err(&pdev->dev,
			"failed to register thermal zone device %d\n", ret);
		cpufreq_cooling_unregister(data->cdev[0]);
		devfreq_cooling_unregister(data->cdev[1]);
		return ret;
	}

	/* Enable measurements at ~ 10 Hz */
	regmap_write(map, TEMPSENSE1 + REG_CLR, TEMPSENSE1_MEASURE_FREQ);
	measure_freq = DIV_ROUND_UP(32768, 10); /* 10 Hz */
	regmap_write(map, TEMPSENSE1 + REG_SET, measure_freq);
	imx_set_alarm_temp(data, data->temp_passive);
	regmap_write(map, TEMPSENSE0 + REG_CLR, TEMPSENSE0_POWER_DOWN);
	regmap_write(map, TEMPSENSE0 + REG_SET, TEMPSENSE0_MEASURE_TEMP);

	data->mode = THERMAL_DEVICE_ENABLED;

	return 0;
}

static int imx_thermal_remove(struct platform_device *pdev)
{
	struct imx_thermal_data *data = platform_get_drvdata(pdev);
	struct regmap *map = data->tempmon;

	/* Disable measurements */
	regmap_write(map, TEMPSENSE0 + REG_SET, TEMPSENSE0_POWER_DOWN);
	if (!IS_ERR(data->thermal_clk))
		clk_disable_unprepare(data->thermal_clk);

	thermal_zone_device_unregister(data->tz);
	cpufreq_cooling_unregister(data->cdev[0]);
	devfreq_cooling_unregister(data->cdev[1]);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int imx_thermal_suspend(struct device *dev)
{
	struct imx_thermal_data *data = dev_get_drvdata(dev);
	struct regmap *map = data->tempmon;

	/*
	 * Need to disable thermal sensor, otherwise, when thermal core
	 * try to get temperature before thermal sensor resume, a wrong
	 * temperature will be read as the thermal sensor is powered
	 * down.
	 */
	regmap_write(map, TEMPSENSE0 + REG_CLR, TEMPSENSE0_MEASURE_TEMP);
	regmap_write(map, TEMPSENSE0 + REG_SET, TEMPSENSE0_POWER_DOWN);
	data->mode = THERMAL_DEVICE_DISABLED;

	return 0;
}

static int imx_thermal_resume(struct device *dev)
{
	struct imx_thermal_data *data = dev_get_drvdata(dev);
	struct regmap *map = data->tempmon;

	/* Enabled thermal sensor after resume */
	regmap_write(map, TEMPSENSE0 + REG_CLR, TEMPSENSE0_POWER_DOWN);
	regmap_write(map, TEMPSENSE0 + REG_SET, TEMPSENSE0_MEASURE_TEMP);
	data->mode = THERMAL_DEVICE_ENABLED;

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(imx_thermal_pm_ops,
			 imx_thermal_suspend, imx_thermal_resume);

static const struct of_device_id of_imx_thermal_match[] = {
	{ .compatible = "fsl,imx6q-tempmon", },
	{ /* end */ }
};
MODULE_DEVICE_TABLE(of, of_imx_thermal_match);

static struct platform_driver imx_thermal = {
	.driver = {
		.name	= "imx_thermal",
		.owner  = THIS_MODULE,
		.pm	= &imx_thermal_pm_ops,
		.of_match_table = of_imx_thermal_match,
	},
	.probe		= imx_thermal_probe,
	.remove		= imx_thermal_remove,
};
module_platform_driver(imx_thermal);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Thermal driver for Freescale i.MX SoCs");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-thermal");
