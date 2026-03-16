/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2010-2015 Freescale Semiconductor, Inc.
 * Copyright 2019 NXP
 */

/*!
 * @file pmic/core/max25221.c
 * @brief This file contains MAX25221 specific PMIC code. This implementaion
 * may differ for each PMIC chip.
 *
 * @ingroup PMIC_CORE
 */

/*
 * Includes
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/pmic_status.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max25221.h>
// #include <asm/mach-types.h>

static int max25221_detect(struct i2c_client *client,
			  struct i2c_board_info *info);
struct i2c_client *max25221_client;
static struct regulator *gpio_regulator;

static struct mfd_cell max25221_devs[] = {
	{ .name = "max25221-pmic", },
	{ .name = "max25221-sns", },
};

static const unsigned short normal_i2c[] = {0x48, I2C_CLIENT_END};

int max25221_reg_read(int reg_num, unsigned int *reg_val)
{
	int result;

	if (max25221_client == NULL)
		return PMIC_ERROR;

	result = i2c_smbus_read_byte_data(max25221_client, reg_num);
	if (result < 0) {
		dev_err(&max25221_client->dev,
			"Unable to read MAX25221 register via I2C\n");
		return PMIC_ERROR;
	}	

	*reg_val = result;
	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(max25221_reg_read);

int max25221_reg_write(int reg_num, const unsigned int reg_val)
{
	int result;

	if (max25221_client == NULL)
		return PMIC_ERROR;

	result = i2c_smbus_write_byte_data(max25221_client, reg_num, reg_val);
	if (result < 0) {
		dev_err(&max25221_client->dev,
			"Unable to write MAX25221 register via I2C\n");
		return PMIC_ERROR;
	}

	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(max25221_reg_write);


static struct max25221_platform_data *max25221_i2c_parse_dt_pdata(
					struct device *dev)
{
	struct max25221_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for pdata\n");
		return ERR_PTR(-ENOMEM);
	}

	return pdata;
}


static int max25221_probe(struct i2c_client *client)
{
	struct max25221 *max25221;
	struct max25221_platform_data *pdata = client->dev.platform_data;
	struct device_node *np = client->dev.of_node;
	int ret = 0;

	if (!np)
		return -ENODEV;

	gpio_regulator = devm_regulator_get(&client->dev, "SENSOR");
	if (!IS_ERR(gpio_regulator)) {
		ret = regulator_enable(gpio_regulator);
		if (ret) {
			dev_err(&client->dev, "gpio set voltage error\n");
			return ret;
		}
	}

	/* Create the PMIC data structure */
	max25221 = kzalloc(sizeof(struct max25221), GFP_KERNEL);
	if (max25221 == NULL)
		return -ENOMEM;

	/* Initialize the PMIC data structure */
	i2c_set_clientdata(client, max25221);
	max25221->dev = &client->dev;
	max25221->i2c_client = client;

	max25221_client = client;
	ret = max25221_detect(client, NULL);
	if (ret)
		goto err1;

	if (max25221->dev->of_node) {
		pdata = max25221_i2c_parse_dt_pdata(max25221->dev);
		if (IS_ERR(pdata)) {
			ret = PTR_ERR(pdata);
			goto err2;
		}
	}
	max25221->pdata = pdata;		

	mfd_add_devices(max25221->dev, -1, max25221_devs,
			ARRAY_SIZE(max25221_devs),
			NULL, 0, NULL);


	dev_info(&client->dev, "PMIC MAX25221 for LCD display\n");

	return ret;
err2:
	mfd_remove_devices(max25221->dev);
err1:
	if (!IS_ERR(gpio_regulator))
		regulator_disable(gpio_regulator);
	kfree(max25221);

	return ret;
}


static void max25221_remove(struct i2c_client *i2c)
{
	struct max25221 *max25221 = i2c_get_clientdata(i2c);

	mfd_remove_devices(max25221->dev);

	if (!IS_ERR(gpio_regulator))
		regulator_disable(gpio_regulator);
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int max25221_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	s32 chip_id;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	/* detection */
	chip_id = i2c_smbus_read_byte_data(client,REG_MAX25221_DEVICE);
	if ( chip_id != 0x20 && chip_id != 0x21) {
		dev_err(&adapter->dev,
			"Max25221 PMIC not found! chip_id=%d\n",chip_id);
		return -ENODEV;
	}

	if (info)
		strlcpy(info->type, "max25221_sensor", I2C_NAME_SIZE);

	return 0;
}

static const struct i2c_device_id max25221_id[] = {
       { "max25221", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, max25221_id);

static const struct of_device_id max25221_dt_ids[] = {
	{
		.compatible = "maxim,max25221",
		.data = (void *) &max25221_id[0],
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, max25221_dt_ids);


static struct i2c_driver max25221_driver = {
	.driver = {
		   .name = "max25221",
		   .owner = THIS_MODULE,
		   .of_match_table = max25221_dt_ids,
	},
	.probe = max25221_probe,
	.remove = max25221_remove,
	.id_table = max25221_id,
	.detect = max25221_detect,
	.address_list = &normal_i2c[0],
};

static int __init max25221_init(void)
{
	return i2c_add_driver(&max25221_driver);
}

static void __exit max25221_exit(void)
{
	i2c_del_driver(&max25221_driver);
}

/*
 * Module entry points
 */
subsys_initcall(max25221_init);
module_exit(max25221_exit);
