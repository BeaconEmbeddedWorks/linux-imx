/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2010-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2019 NXP
 */
#ifndef __LINUX_REGULATOR_MAX25221_H_
#define __LINUX_REGULATOR_MAX25221_H_

/*
 * PMIC Register Addresses
 */
enum {
    REG_MAX25221_DEVICE 			= 0x00,
    REG_MAX25221_TEMP				= 0X01,
    REG_MAX25221_REG_CTRL			= 0X02,	
    REG_MAX25221_FLTMASK1			= 0X03,	
    REG_MAX25221_FLTMASK2			= 0X04,	
    REG_MAX25221_FAULT1				= 0X05,
    REG_MAX25221_FAULT2				= 0X06,
    REG_MAX25221_CONFIG				= 0X07,
    REG_MAX25221_DELAY_VCOM_LSB		= 0X08,		
    REG_MAX25221_VCOM25				= 0X09,
    REG_MAX25221_VCOM_L				= 0X0A,
    REG_MAX25221_VCOM_H1			= 0X0B,	
    REG_MAX25221_VCOM_H2			= 0X0C,	
    REG_MAX25221_VTEMP25			= 0X0D,	
	REG_MAX25221_VTEMP_L			= 0X0E,	
    REG_MAX25221_VTEMP_H1			= 0X0F,	
    REG_MAX25221_VTEMP_H2			= 0X10,	
    REG_MAX25221_VCOM_MIN			= 0X11,	
    REG_MAX25221_VCOM_MAX			= 0X12,	
    REG_MAX25221_AVDD_SET			= 0X13,
	REG_MAX25221_VGON				= 0X14,
	REG_MAX25221_VGOFF				= 0X15

};
#define MAX25221_REG_NUM        21
#define MAX25221_MAX_REGISTER   0xFF

/*
 * Bitfield macros that use rely on bitfield width/shift information.
 */
#define BITFMASK(field) (((1U << (field ## _WID)) - 1) << (field ## _LSH))
#define BITFVAL(field, val) ((val) << (field ## _LSH))
#define BITFEXT(var, bit) ((var & BITFMASK(bit)) >> (bit ## _LSH))

/*
 * Shift and width values for each register bitfield
 */
#define EXT_TEMP_LSH    7
#define EXT_TEMP_WID    9

#define THERMAL_SHUTDOWN_LSH    0
#define THERMAL_SHUTDOWN_WID    1

#define INT_TEMP_LSH    7
#define INT_TEMP_WID    9

#define STAT_BUSY_LSH   0
#define STAT_BUSY_WID   1
#define STAT_OPEN_LSH   1
#define STAT_OPEN_WID   1
#define STAT_SHRT_LSH   2
#define STAT_SHRT_WID   1

#define PROD_REV_LSH    0
#define PROD_REV_WID    8

#define PROD_ID_LSH     0
#define PROD_ID_WID     8

#define DVR_LSH         0
#define DVR_WID         8

#define ENABLE_LSH      0
#define ENABLE_WID      1
#define VCOM_ENABLE_LSH 1
#define VCOM_ENABLE_WID 1

#define FAULT_FBPG_LSH      0
#define FAULT_FBPG_WID      1
#define FAULT_HVINP_LSH     1
#define FAULT_HVINP_WID     1
#define FAULT_HVINN_LSH     2
#define FAULT_HVINN_WID     1
#define FAULT_FBNG_LSH      3
#define FAULT_FBNG_WID      1
#define FAULT_HVINPSC_LSH   4
#define FAULT_HVINPSC_WID   1
#define FAULT_HVINNSC_LSH   5
#define FAULT_HVINNSC_WID   1
#define FAULT_OT_LSH        6
#define FAULT_OT_WID        1
#define FAULT_POK_LSH       7
#define FAULT_POK_WID       1

#define HVINP_LSH           0
#define HVINP_WID           4

#define CTRL_DVR_LSH        0
#define CTRL_DVR_WID        1
#define CTRL_TIMING_LSH     1
#define CTRL_TIMING_WID     1

#define TIMING1_LSH         0
#define TIMING1_WID         8
#define TIMING2_LSH         0
#define TIMING2_WID         8
#define TIMING3_LSH         0
#define TIMING3_WID         8
#define TIMING4_LSH         0
#define TIMING4_WID         8
#define TIMING5_LSH         0
#define TIMING5_WID         8
#define TIMING6_LSH         0
#define TIMING6_WID         8
#define TIMING7_LSH         0
#define TIMING7_WID         8
#define TIMING8_LSH         0
#define TIMING8_WID         8

struct max25221 {
	/* chip revision */
	int rev;

	struct device *dev;
	struct max25221_platform_data *pdata;

	/* Platform connection */
	struct i2c_client *i2c_client;

	/* Timings */
	unsigned int gvee_pwrup;
	unsigned int vneg_pwrup;
	unsigned int vpos_pwrup;
	unsigned int gvdd_pwrup;
	unsigned int gvdd_pwrdn;
	unsigned int vpos_pwrdn;
	unsigned int vneg_pwrdn;
	unsigned int gvee_pwrdn;

	/* GPIOs */
	int gpio_pmic_pwrgood;
	int gpio_pmic_vcom_ctrl;
	int gpio_pmic_enable;
	int gpio_pmic_v3p3;
	int gpio_pmic_intr;

	/* MAX25221 part variables */
	int pass_num;
	int vcom_uV;

	/* One-time VCOM setup marker */
	bool vcom_setup;

	/* powerup/powerdown wait time */
	int max_wait;
};

enum {
    /* In alphabetical order */
    MAX25221_DISPLAY, /* virtual master enable */
    MAX25221_NUM_REGULATORS,
};

/*
 * Declarations
 */
struct regulator_init_data;
struct max25221_regulator_data;

struct max25221_platform_data {
	unsigned int gvee_pwrup;
	unsigned int vneg_pwrup;
	unsigned int vpos_pwrup;
	unsigned int gvdd_pwrup;
	unsigned int gvdd_pwrdn;
	unsigned int vpos_pwrdn;
	unsigned int vneg_pwrdn;
	unsigned int gvee_pwrdn;
	int gpio_pmic_pwrgood;
	int gpio_pmic_vcom_ctrl;
	int gpio_pmic_enable;
	int gpio_pmic_v3p3;
	int gpio_pmic_intr;
	int pass_num;
	int vcom_uV;

	/* PMIC */
	struct max25221_regulator_data *regulators;
	int num_regulators;
};

struct max25221_regulator_data {
	int id;
	struct regulator_init_data *initdata;
	struct device_node *reg_node;
};

int max25221_reg_read(int reg_num, unsigned int *reg_val);
int max25221_reg_write(int reg_num, const unsigned int reg_val);

#endif
