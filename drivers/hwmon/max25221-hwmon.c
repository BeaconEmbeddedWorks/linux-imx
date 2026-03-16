/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2010-2015 Freescale Semiconductor, Inc.
 * Copyright 2019 NXP
 */
/*
 * max25221.c
 *
 * Based on the MAX1619 driver.
 * Copyright (C) 2003-2004 Alexey Fisher <fishor@mail.ru>
 *                         Jean Delvare <khali@linux-fr.org>
 *
 * The MAX25221 is a sensor chip made by Maxim.
 * It reports up to two temperatures (its own plus up to
 * one external one).
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/mfd/max25221.h>
#include <linux/mod_devicetable.h>

/*
 * Functions declaration
 */
static int max25221_sensor_probe(struct platform_device *pdev);
static int max25221_sensor_remove(struct platform_device *pdev);

static const struct platform_device_id max25221_sns_id[] = {
	{ "max25221-sns", 0},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, max25221_sns_id);

/*
 * Driver data (common to all clients)
 */
static struct platform_driver max25221_sensor_driver = {
	.probe = max25221_sensor_probe,
	.remove = max25221_sensor_remove,
	.id_table = max25221_sns_id,
	.driver = {
		.name = "max25221_sensor",
	},
};

/*
 * Client data (each client gets its own)
 */
struct max25221_data {
	struct device *hwmon_dev;
};

/*
 * Sysfs stuff
 */
static ssize_t show_temp_input1(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	//max25221_reg_read(REG_MAX25221_INT_TEMP, &reg_val);
	return snprintf(buf, PAGE_SIZE, "%d\n", 25000);
}

static ssize_t show_temp_input2(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	//max25221_reg_read(REG_MAX25221_EXT_TEMP, &reg_val);
	return snprintf(buf, PAGE_SIZE, "%d\n", 25000);
}

static DEVICE_ATTR(temp1_input, S_IRUGO, show_temp_input1, NULL);
static DEVICE_ATTR(temp2_input, S_IRUGO, show_temp_input2, NULL);

static struct attribute *max25221_attributes[] = {
	&dev_attr_temp1_input.attr,
	&dev_attr_temp2_input.attr,
	NULL
};

static const struct attribute_group max25221_group = {
	.attrs = max25221_attributes,
};

/*
 * Real code
 */
static int max25221_sensor_probe(struct platform_device *pdev)
{
	struct max25221_data *data;
	int err;

	data = kzalloc(sizeof(struct max25221_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&pdev->dev.kobj, &max25221_group);
	if (err)
		goto exit_free;

	data->hwmon_dev = hwmon_device_register(&pdev->dev);
	
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove_files;
	}

	platform_set_drvdata(pdev, data);

	return 0;

exit_remove_files:
	sysfs_remove_group(&pdev->dev.kobj, &max25221_group);
exit_free:
	kfree(data);
exit:
	return err;
}

static int max25221_sensor_remove(struct platform_device *pdev)
{
	struct max25221_data *data = platform_get_drvdata(pdev);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&pdev->dev.kobj, &max25221_group);

	kfree(data);
	return 0;
}

static int __init sensors_max25221_init(void)
{
	return platform_driver_register(&max25221_sensor_driver);
}
module_init(sensors_max25221_init);

static void __exit sensors_max25221_exit(void)
{
	platform_driver_unregister(&max25221_sensor_driver);
}
module_exit(sensors_max25221_exit);

MODULE_DESCRIPTION("MAX25221 sensor driver");
MODULE_LICENSE("GPL");

