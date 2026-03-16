/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2010-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2019 NXP
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/max25221.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>


struct max25221_data {
	int num_regulators;
	struct max25221 *max25221;
	struct regulator_dev **rdev;
};


static int max25221_display_enable(struct regulator_dev *reg)
{
	unsigned int reg_val;
	struct max25221 *max25221 = rdev_get_drvdata(reg);

	dev_err(max25221->dev, "max25221_display_enable\n");
	max25221_reg_read(REG_MAX25221_FAULT1, &reg_val);	//read fault regs to clear
	max25221_reg_read(REG_MAX25221_FAULT2, &reg_val);

	max25221_reg_write(REG_MAX25221_REG_CTRL, 0);	//diable display to change values
	mdelay(25);
	max25221_reg_read(REG_MAX25221_FAULT1, &reg_val);
	max25221_reg_read(REG_MAX25221_FAULT2, &reg_val);
	
	max25221_reg_write(REG_MAX25221_CONFIG, 		0x07);	//
	max25221_reg_write(REG_MAX25221_DELAY_VCOM_LSB, 0xa9);	//vcom bit 1
	max25221_reg_write(REG_MAX25221_VCOM25, 		0x86);	//vcom top (-1.5 to -0.5) -0.65V
	max25221_reg_write(REG_MAX25221_AVDD_SET, 		0x0d);	//avdd (4.5 to 6.0)	5.5V	
	max25221_reg_write(REG_MAX25221_VGON, 			0x74);	//vgon (16 to 20) 18.0V	
	max25221_reg_write(REG_MAX25221_VGOFF, 			0x16);	//vgoff (-11 to -8) -10V

	max25221_reg_write(REG_MAX25221_REG_CTRL, 0x40);	//enable display

	mdelay(250);
	max25221_reg_read(REG_MAX25221_FAULT1, &reg_val);
	max25221_reg_read(REG_MAX25221_FAULT2, &reg_val);
	mdelay(250);
	max25221_reg_read(REG_MAX25221_FAULT1, &reg_val);
	max25221_reg_read(REG_MAX25221_FAULT2, &reg_val);
	mdelay(250);
	max25221_reg_read(REG_MAX25221_FAULT1, &reg_val);
	max25221_reg_read(REG_MAX25221_FAULT2, &reg_val);
	
	return true;
}

static int max25221_display_disable(struct regulator_dev *reg)
{
	struct max25221 *max25221 = rdev_get_drvdata(reg);

	dev_err(max25221->dev, "max25221_display_disable\n");
	max25221_reg_write(REG_MAX25221_REG_CTRL, 0);

	return 0;
}

static int max25221_display_is_enabled(struct regulator_dev *reg)
{
	struct max25221 *max25221 = rdev_get_drvdata(reg);
	int gpio = gpio_get_value(max25221->gpio_pmic_enable);

	if (gpio == 0)
		return 0;
	else
		return 1;
}

/*
 * Regulator operations
 */

static struct regulator_ops max25221_display_ops = {
	.enable = max25221_display_enable,
	.disable = max25221_display_disable,
	.is_enabled = max25221_display_is_enabled,
};




/*
 * Regulator descriptors
 */
static struct regulator_desc max25221_reg[MAX25221_NUM_REGULATORS] = {
{
	.name = "DISPLAY",
	.id = MAX25221_DISPLAY,
	.ops = &max25221_display_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},

};


#define CHECK_PROPERTY_ERROR_KFREE(prop) \
do { \
	int ret = of_property_read_u32(max25221->dev->of_node, \
					#prop, &max25221->prop); \
	if (ret < 0) { \
		return ret;	\
	}	\
} while (0);

static int max25221_pmic_dt_parse_pdata(struct platform_device *pdev,
					struct max25221_platform_data *pdata)
{
	struct max25221 *max25221 = dev_get_drvdata(pdev->dev.parent);
	struct device_node *pmic_np, *regulators_np, *reg_np;
	struct max25221_regulator_data *rdata;
	int i, ret;

	pmic_np = of_node_get(max25221->dev->of_node);
	if (!pmic_np) {
		dev_err(&pdev->dev, "could not find pmic sub-node\n");
		return -ENODEV;
	}

	regulators_np = of_find_node_by_name(pmic_np, "regulators");
	if (!regulators_np) {
		dev_err(&pdev->dev, "could not find regulators sub-node\n");
		return -EINVAL;
	}

	pdata->num_regulators = of_get_child_count(regulators_np);
	dev_err(&pdev->dev, "num_regulators %d\n", pdata->num_regulators);

	rdata = devm_kzalloc(&pdev->dev, sizeof(*rdata) *
				pdata->num_regulators, GFP_KERNEL);
	if (!rdata) {
		of_node_put(regulators_np);
		dev_err(&pdev->dev, "could not allocate memory for"
			"regulator data\n");
		return -ENOMEM;
	}

	pdata->regulators = rdata;
	for_each_child_of_node(regulators_np, reg_np) {
		for (i = 0; i < ARRAY_SIZE(max25221_reg); i++)
			if (!of_node_cmp(reg_np->name, max25221_reg[i].name))
				break;

		if (i == ARRAY_SIZE(max25221_reg)) {
			dev_err(&pdev->dev, "don't know how to configure"
				"regulator %s\n", reg_np->name);
			continue;
		}

		rdata->id = i;
		rdata->initdata = of_get_regulator_init_data(&pdev->dev,
							     reg_np,
							     &max25221_reg[i]);
		rdata->reg_node = reg_np;
		rdata++;
	}
	of_node_put(regulators_np);

err:
	return 0;

}

/*
 * Regulator init/probing/exit functions
 */
static int max25221_regulator_probe(struct platform_device *pdev)
{
	struct max25221 *max25221 = dev_get_drvdata(pdev->dev.parent);
	struct max25221_platform_data *pdata = max25221->pdata;
	struct max25221_data *priv;
	struct regulator_dev **rdev;
	struct regulator_config config = { };
	int size, i, ret = 0;

	dev_err(&pdev->dev, "max25221_regulator_probe\n");


	if (max25221->dev->of_node) {
		ret = max25221_pmic_dt_parse_pdata(pdev, pdata);
		if (ret)
			return ret;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(struct max25221_data),
			       GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	size = sizeof(struct regulator_dev *) * pdata->num_regulators;
	priv->rdev = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!priv->rdev)
		return -ENOMEM;

	dev_err(&pdev->dev, "max25221_regulator_probe: num_of_reg %d\n", pdata->num_regulators);		

	rdev = priv->rdev;
	priv->num_regulators = pdata->num_regulators;
	platform_set_drvdata(pdev, priv);

	for (i = 0; i < pdata->num_regulators; i++) {
		int id = pdata->regulators[i].id;

		config.dev = max25221->dev;
		config.init_data = pdata->regulators[i].initdata;
		config.driver_data = max25221;
		config.of_node = pdata->regulators[i].reg_node;

		rdev[i] = regulator_register(config.dev, &max25221_reg[id], &config);
		if (IS_ERR(rdev[i])) {
			ret = PTR_ERR(rdev[i]);
			dev_err(&pdev->dev, "regulator init failed for %d\n",
					id);
			rdev[i] = NULL;
			goto err;
		}
	}

	return 0;
err:
	while (--i >= 0)
		regulator_unregister(rdev[i]);
	return ret;
}

static int max25221_regulator_remove(struct platform_device *pdev)
{
	struct max25221_data *priv = platform_get_drvdata(pdev);
	struct regulator_dev **rdev = priv->rdev;
	int i;

	for (i = 0; i < priv->num_regulators; i++)
		regulator_unregister(rdev[i]);
	return 0;
}

static const struct platform_device_id max25221_pmic_id[] = {
	{ "max25221-pmic", 0},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, max25221_pmic_id);

static struct platform_driver max25221_regulator_driver = {
	.probe = max25221_regulator_probe,
	.remove = max25221_regulator_remove,
	.id_table = max25221_pmic_id,
	.driver = {
		.name = "max25221-pmic",
	},
};

static int __init max25221_regulator_init(void)
{
	return platform_driver_register(&max25221_regulator_driver);
}
subsys_initcall_sync(max25221_regulator_init);

static void __exit max25221_regulator_exit(void)
{
	platform_driver_unregister(&max25221_regulator_driver);
}
module_exit(max25221_regulator_exit);

/*
 * Parse user specified options (`max25221:')
 * example:
 *   max25221:pass=2,vcom=-1250000
 */
static int __init max25221_setup(char *options)
{
	return 1;
}

__setup("max25221:", max25221_setup);

/* Module information */
MODULE_DESCRIPTION("MAX25221 regulator driver");
MODULE_LICENSE("GPL");
