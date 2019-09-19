// SPDX-License-Identifier: GPL-2.0
/*
 * drivers/rtc/rtc-pcf85363.c
 *
 * Driver for NXP PCF85363 real-time clock.
 *
 * Copyright (C) 2017 Eric Nelson
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/bcd.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

/* Quartz capacitance */
#define PCF85363_QUARTZCAP_7pF		0
#define PCF85363_QUARTZCAP_6pF		1
#define PCF85363_QUARTZCAP_12p5pF	2

/* Quartz drive strength */
#define PCF85363_QUARTZDRIVE_NORMAL	0
#define PCF85363_QUARTZDRIVE_LOW	1
#define PCF85363_QUARTZDRIVE_HIGH	2

/*
 * Date/Time registers
 */
#define DT_100THS	0x00
#define DT_SECS		0x01
#define DT_MINUTES	0x02
#define DT_HOURS	0x03
#define DT_DAYS		0x04
#define DT_WEEKDAYS	0x05
#define DT_MONTHS	0x06
#define DT_YEARS	0x07

/*
 * Alarm registers
 */
#define DT_SECOND_ALM1	0x08
#define DT_MINUTE_ALM1	0x09
#define DT_HOUR_ALM1	0x0a
#define DT_DAY_ALM1	0x0b
#define DT_MONTH_ALM1	0x0c
#define DT_MINUTE_ALM2	0x0d
#define DT_HOUR_ALM2	0x0e
#define DT_WEEKDAY_ALM2	0x0f
#define DT_ALARM_EN	0x10

/*
 * Time stamp registers
 */
#define DT_TIMESTAMP1	0x11
#define DT_TIMESTAMP2	0x17
#define DT_TIMESTAMP3	0x1d
#define DT_TS_MODE	0x23

/*
 * control registers
 */
#define CTRL_OFFSET	0x24
#define CTRL_OSCILLATOR	0x25
#define CTRL_BATTERY	0x26
#define CTRL_PIN_IO	0x27
#define CTRL_FUNCTION	0x28
#define CTRL_INTA_EN	0x29
#define CTRL_INTB_EN	0x2a
#define CTRL_FLAGS	0x2b
#define CTRL_RAMBYTE	0x2c
#define CTRL_WDOG	0x2d
#define CTRL_STOP_EN	0x2e
#define CTRL_RESETS	0x2f
#define CTRL_RAM	0x40

#define ALRM_SEC_A1E	BIT(0)
#define ALRM_MIN_A1E	BIT(1)
#define ALRM_HR_A1E	BIT(2)
#define ALRM_DAY_A1E	BIT(3)
#define ALRM_MON_A1E	BIT(4)
#define ALRM_MIN_A2E	BIT(5)
#define ALRM_HR_A2E	BIT(6)
#define ALRM_DAY_A2E	BIT(7)

#define INT_WDIE	BIT(0)
#define INT_BSIE	BIT(1)
#define INT_TSRIE	BIT(2)
#define INT_A2IE	BIT(3)
#define INT_A1IE	BIT(4)
#define INT_OIE		BIT(5)
#define INT_PIE		BIT(6)
#define INT_ILP		BIT(7)

#define FLAGS_TSR1F	BIT(0)
#define FLAGS_TSR2F	BIT(1)
#define FLAGS_TSR3F	BIT(2)
#define FLAGS_BSF	BIT(3)
#define FLAGS_WDF	BIT(4)
#define FLAGS_A1F	BIT(5)
#define FLAGS_A2F	BIT(6)
#define FLAGS_PIF	BIT(7)

#define PIN_IO_INTAPM	GENMASK(1, 0)
#define PIN_IO_INTAPM_SHIFT	0
#define PIN_IO_INTA_CLK	(0 << PIN_IO_INTAPM_SHIFT)
#define PIN_IO_INTA_BAT	(1 << PIN_IO_INTAPM_SHIFT)
#define PIN_IO_INTA_OUT	(2 << PIN_IO_INTAPM_SHIFT)
#define PIN_IO_INTA_HIZ	(3 << PIN_IO_INTAPM_SHIFT)

#define PIN_IO_TSPM	 GENMASK(3, 2)
#define PIN_IO_TSPM_SHIFT	2
#define PIN_IO_TS_DISABLE	(0x0 << PIN_IO_TSPM_SHIFT)
#define PIN_IO_TS_INTB_OUT	(0x1 << PIN_IO_TSPM_SHIFT)
#define PIN_IO_TS_CLK_OUT	(0x2 << PIN_IO_TSPM_SHIFT)
#define PIN_IO_TS_IN	(0x3 << PIN_IO_TSPM_SHIFT)

#define PIN_IO_CLKPM	BIT(7) /* 0 = enable CLK pin,1 = disable CLK pin */

#define STOP_EN_STOP	BIT(0)

#define RESET_CPR	0xa4

#define NVRAM_SIZE	0x40

#define DT_SECS_OS BIT(7)

#define CTRL_OSCILLATOR_CL_MASK	GENMASK(1, 0)
#define CTRL_OSCILLATOR_CL_SHIFT	0
#define CTRL_OSCILLATOR_OSCD_MASK	GENMASK(3, 2)
#define CTRL_OSCILLATOR_OSCD_SHIFT	2
#define CTRL_OSCILLATOR_LOWJ		BIT(4)

#define CTRL_FUNCTION_COF_OFF	0x7 /* No clock output */

enum pcf85363_irqpin {
	IRQPIN_INTA,
	IRQPIN_INTB,
	IRQPIN_MAX,
};

static const char *const pcf85363_irqpin_names[] = {
	[IRQPIN_INTA] = "INTA",
	[IRQPIN_INTB] = "INTB",
	[IRQPIN_MAX] = "",
};


struct pcf85363 {
	struct device *dev;
	struct rtc_device	*rtc;
	struct regmap		*regmap;
	int irq;
	u8 irq_type[IRQPIN_MAX];
};

struct pcf85x63_config {
	struct regmap_config regmap;
	unsigned int num_nvram;
};

static int pcf85363_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct pcf85363 *pcf85363 = dev_get_drvdata(dev);
	unsigned char buf[DT_YEARS + 1];
	int ret, len = sizeof(buf);

	/* read the RTC date and time registers all at once */
	ret = regmap_bulk_read(pcf85363->regmap, DT_100THS, buf, len);
	if (ret) {
		dev_err(dev, "%s: error %d\n", __func__, ret);
		return ret;
	}

	tm->tm_year = bcd2bin(buf[DT_YEARS]);
	/* adjust for 1900 base of rtc_time */
	tm->tm_year += 100;

	tm->tm_wday = buf[DT_WEEKDAYS] & 7;
	buf[DT_SECS] &= 0x7F;
	tm->tm_sec = bcd2bin(buf[DT_SECS]);
	buf[DT_MINUTES] &= 0x7F;
	tm->tm_min = bcd2bin(buf[DT_MINUTES]);
	tm->tm_hour = bcd2bin(buf[DT_HOURS]);
	tm->tm_mday = bcd2bin(buf[DT_DAYS]);
	tm->tm_mon = bcd2bin(buf[DT_MONTHS]) - 1;

	return 0;
}

static int pcf85363_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct pcf85363 *pcf85363 = dev_get_drvdata(dev);
	unsigned char tmp[11];
	unsigned char *buf = &tmp[2];
	int ret;

	tmp[0] = STOP_EN_STOP;
	tmp[1] = RESET_CPR;

	buf[DT_100THS] = 0;
	buf[DT_SECS] = bin2bcd(tm->tm_sec);
	buf[DT_MINUTES] = bin2bcd(tm->tm_min);
	buf[DT_HOURS] = bin2bcd(tm->tm_hour);
	buf[DT_DAYS] = bin2bcd(tm->tm_mday);
	buf[DT_WEEKDAYS] = tm->tm_wday;
	buf[DT_MONTHS] = bin2bcd(tm->tm_mon + 1);
	buf[DT_YEARS] = bin2bcd(tm->tm_year % 100);

	ret = regmap_bulk_write(pcf85363->regmap, CTRL_STOP_EN,
				tmp, 2);
	if (ret)
		return ret;

	ret = regmap_bulk_write(pcf85363->regmap, DT_100THS,
				buf, sizeof(tmp) - 2);
	if (ret)
		return ret;

	return regmap_write(pcf85363->regmap, CTRL_STOP_EN, 0);
}

static int pcf85363_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct pcf85363 *pcf85363 = dev_get_drvdata(dev);
	unsigned char buf[DT_MONTH_ALM1 - DT_SECOND_ALM1 + 1];
	unsigned int val;
	int ret;

	ret = regmap_bulk_read(pcf85363->regmap, DT_SECOND_ALM1, buf,
			       sizeof(buf));
	if (ret)
		return ret;

	alrm->time.tm_sec = bcd2bin(buf[0]);
	alrm->time.tm_min = bcd2bin(buf[1]);
	alrm->time.tm_hour = bcd2bin(buf[2]);
	alrm->time.tm_mday = bcd2bin(buf[3]);
	alrm->time.tm_mon = bcd2bin(buf[4]) - 1;

	ret = regmap_read(pcf85363->regmap, CTRL_INTA_EN, &val);
	if (ret)
		return ret;

	alrm->enabled =  !!(val & INT_A1IE);

	return 0;
}

static int _pcf85363_rtc_alarm_irq_enable(struct pcf85363 *pcf85363,
					  unsigned int enabled,
					  int irq_pin)
{
	unsigned int alarm1_flags = ALRM_SEC_A1E | ALRM_MIN_A1E | ALRM_HR_A1E |
				   ALRM_DAY_A1E | ALRM_MON_A1E;
	unsigned int alarm2_flags = ALRM_MIN_A2E | ALRM_HR_A2E | ALRM_DAY_A2E;
	unsigned int alarm_flags = 0;
	int ret, reg;
	u8 reg_val = 0, ctrl_flags = FLAGS_A1F;

	if (pcf85363->irq_type[irq_pin] & INT_A1IE) {
		alarm_flags = alarm1_flags;
		ctrl_flags = FLAGS_A1F;
	}

	if (pcf85363->irq_type[irq_pin] & INT_A2IE) {
		alarm_flags |= alarm2_flags;
		ctrl_flags |= FLAGS_A2F;
	}
	ret = regmap_update_bits(pcf85363->regmap, DT_ALARM_EN, alarm_flags,
				 enabled ? alarm_flags : 0);
	if (ret)
		return ret;

	reg = CTRL_INTA_EN;
	reg_val = INT_A1IE;
	if (pcf85363->irq_type[irq_pin]) {
		switch (irq_pin) {
		case IRQPIN_INTA:
			reg = CTRL_INTA_EN;
			reg_val = pcf85363->irq_type[irq_pin] &
				  (INT_A1IE | INT_A2IE);
			break;

		case IRQPIN_INTB:
			reg = CTRL_INTB_EN;
			reg_val = pcf85363->irq_type[irq_pin] &
				  (INT_A1IE | INT_A2IE);
			break;

		default:
			dev_err(pcf85363->dev, "Failed to enable some \
				interrupts on some interrupt output pins\n");
			return -EINVAL;
		}
	}
	ret = regmap_update_bits(pcf85363->regmap, reg,
				 reg_val, enabled ? reg_val : 0);
	if (ret || !enabled)
		return ret;

	/* clear current flags */
	return regmap_update_bits(pcf85363->regmap, CTRL_FLAGS, ctrl_flags, 0);
}

static int pcf85363_rtc_alarm_irq_enable(struct device *dev,
					 unsigned int enabled)
{
	struct pcf85363 *pcf85363 = dev_get_drvdata(dev);

	return _pcf85363_rtc_alarm_irq_enable(pcf85363, enabled, IRQPIN_INTA);
}

static int pcf85363_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct pcf85363 *pcf85363 = dev_get_drvdata(dev);
	unsigned char buf[DT_MONTH_ALM1 - DT_SECOND_ALM1 + 1];
	int ret;

	buf[0] = bin2bcd(alrm->time.tm_sec);
	buf[1] = bin2bcd(alrm->time.tm_min);
	buf[2] = bin2bcd(alrm->time.tm_hour);
	buf[3] = bin2bcd(alrm->time.tm_mday);
	buf[4] = bin2bcd(alrm->time.tm_mon + 1);

	/*
	 * Disable the alarm interrupt before changing the value to avoid
	 * spurious interrupts
	 */
	ret = _pcf85363_rtc_alarm_irq_enable(pcf85363, 0, IRQPIN_INTA);
	if (ret)
		return ret;

	ret = regmap_bulk_write(pcf85363->regmap, DT_SECOND_ALM1, buf,
				sizeof(buf));
	if (ret)
		return ret;

	return _pcf85363_rtc_alarm_irq_enable(pcf85363,
					      alrm->enabled,
					      IRQPIN_INTA);
}

static irqreturn_t pcf85363_rtc_handle_irq(int irq, void *dev_id)
{
	struct pcf85363 *pcf85363 = i2c_get_clientdata(dev_id);
	unsigned int flags;
	int err;

	err = regmap_read(pcf85363->regmap, CTRL_FLAGS, &flags);
	if (err)
		return IRQ_NONE;

	if (flags & FLAGS_A1F) {
		rtc_update_irq(pcf85363->rtc, 1, RTC_IRQF | RTC_AF);
		regmap_update_bits(pcf85363->regmap, CTRL_FLAGS, FLAGS_A1F, 0);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int pcf85363_osc_is_stopped(struct pcf85363 *pcf85363)
{
	unsigned int regval;
	int ret;

	ret = regmap_read(pcf85363->regmap, DT_SECS, &regval);
	if (ret)
		return ret;

	ret = regval & DT_SECS_OS ? 1 : 0;
	if (ret)
		dev_warn(pcf85363->dev, "Oscillator stop detected, date/time is not reliable.\n");

	return ret;
}

static int pcf85363_ioctl(struct device *dev,
			  unsigned int cmd, unsigned long arg)
{
	struct pcf85363 *pcf85363 = dev_get_drvdata(dev);
	int ret;

	switch (cmd) {
	case RTC_VL_READ:
		ret = pcf85363_osc_is_stopped(pcf85363);
		if (ret < 0)
			return ret;

		if (copy_to_user((void __user *)arg, &ret, sizeof(int)))
			return -EFAULT;
		return 0;

	case RTC_VL_CLR:
		return regmap_update_bits(pcf85363->regmap,
					  DT_SECS,
					  DT_SECS_OS, 0);
	default:
		return -ENOIOCTLCMD;
	}
}

static const struct rtc_class_ops rtc_ops = {
	.ioctl = pcf85363_ioctl,
	.read_time	= pcf85363_rtc_read_time,
	.set_time	= pcf85363_rtc_set_time,
};

static const struct rtc_class_ops rtc_ops_alarm = {
	.ioctl = pcf85363_ioctl,
	.read_time	= pcf85363_rtc_read_time,
	.set_time	= pcf85363_rtc_set_time,
	.read_alarm	= pcf85363_rtc_read_alarm,
	.set_alarm	= pcf85363_rtc_set_alarm,
	.alarm_irq_enable = pcf85363_rtc_alarm_irq_enable,
};

static int pcf85363_nvram_read(void *priv, unsigned int offset, void *val,
			       size_t bytes)
{
	struct pcf85363 *pcf85363 = priv;

	return regmap_bulk_read(pcf85363->regmap, CTRL_RAM + offset,
				val, bytes);
}

static int pcf85363_nvram_write(void *priv, unsigned int offset, void *val,
				size_t bytes)
{
	struct pcf85363 *pcf85363 = priv;

	return regmap_bulk_write(pcf85363->regmap, CTRL_RAM + offset,
				 val, bytes);
}

static int pcf85x63_nvram_read(void *priv, unsigned int offset, void *val,
			       size_t bytes)
{
	struct pcf85363 *pcf85363 = priv;
	unsigned int tmp_val;
	int ret;

	ret = regmap_read(pcf85363->regmap, CTRL_RAMBYTE, &tmp_val);
	(*(unsigned char *) val) = (unsigned char) tmp_val;

	return ret;
}

static int pcf85x63_nvram_write(void *priv, unsigned int offset, void *val,
				size_t bytes)
{
	struct pcf85363 *pcf85363 = priv;
	unsigned char tmp_val;

	tmp_val = *((unsigned char *)val);
	return regmap_write(pcf85363->regmap, CTRL_RAMBYTE,
				(unsigned int)tmp_val);
}

static const struct pcf85x63_config pcf_85263_config = {
	.regmap = {
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = 0x2f,
	},
	.num_nvram = 1
};

static const struct pcf85x63_config pcf_85363_config = {
	.regmap = {
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = 0x7f,
	},
	.num_nvram = 2
};

/*
 * On some boards the interrupt line may not be wired to the CPU but only to
 * a power supply circuit.
 * In that case no interrupt will be specified in the device tree but the
 * wakeup-source DT property may be used to enable wakeup programming in
 * sysfs
 */
static bool pcf85363_can_wakeup_machine(struct pcf85363 *pcf85363)
{
	return pcf85363->irq ||
		of_property_read_bool(pcf85363->dev->of_node, "wakeup-source");
}

static int pcf85363_init_hw(struct pcf85363 *pcf85363)
{
	struct device_node *np = pcf85363->dev->of_node;
	unsigned int regval = 0;
	u32 propval;
	int i, ret;

	/* Determine if oscilator has been stopped (probably low power) */
	ret = pcf85363_osc_is_stopped(pcf85363);
	if (ret < 0) {
		/* Log here since this is the first hw access on probe */
		dev_err(pcf85363->dev, "Unable to read register\n");

		return ret;
	}

	ret = regmap_read(pcf85363->regmap, CTRL_OSCILLATOR, &regval);
	if (ret)
		return ret;

	/* Set oscilator register */
	propval = PCF85363_QUARTZCAP_12p5pF;
	ret = of_property_read_u32(np, "quartz-load-femtofarads", &propval);
	if (!ret) {
		switch (propval) {
		case 6000:
			propval = PCF85363_QUARTZCAP_6pF;
			break;
		case 7000:
			propval = PCF85363_QUARTZCAP_7pF;
			break;
		case 12500:
			propval = PCF85363_QUARTZCAP_12p5pF;
			break;
		default:
			dev_info(pcf85363->dev, "invalid quartz-load-femtofarads, \
				use default value 12500\n");
			break;
		}
	}
	regval |= ((propval << CTRL_OSCILLATOR_CL_SHIFT)
		    & CTRL_OSCILLATOR_CL_MASK);

	propval = PCF85363_QUARTZDRIVE_NORMAL;
	ret = of_property_read_u32(np, "quartz-drive-strength-ohms", &propval);
	if (!ret) {
		switch (propval) {
		case 60000:
			propval = PCF85363_QUARTZDRIVE_LOW;
			break;
		case 100000:
			propval = PCF85363_QUARTZDRIVE_NORMAL;
			break;
		case 500000:
			propval = PCF85363_QUARTZDRIVE_HIGH;
			break;
		default:
			dev_info(pcf85363->dev, "invalid quartz-drive-strength-ohms, \
				 use default value 100000\n");
			break;
		}
	}
	regval |= ((propval << CTRL_OSCILLATOR_OSCD_SHIFT)
		    & CTRL_OSCILLATOR_OSCD_MASK);

	if (of_property_read_bool(np, "nxp,quartz-low-jitter"))
		regval |= CTRL_OSCILLATOR_LOWJ;

	ret = regmap_write(pcf85363->regmap, CTRL_OSCILLATOR, regval);
	if (ret)
		return ret;

	/* Set function register
	 * (100th second disabled
	 * no periodic interrupt
	 * real-time clock mode
	 * RTC stop is controlled by STOP bit only
	 * no clock output)
	 */
	ret = regmap_write(pcf85363->regmap, CTRL_FUNCTION,
			   CTRL_FUNCTION_COF_OFF);
	if (ret)
		return ret;

	/* Set all interrupts to disabled, level mode */
	ret = regmap_write(pcf85363->regmap, CTRL_INTA_EN,
			   INT_ILP);
	if (ret)
		return ret;
	ret = regmap_write(pcf85363->regmap, CTRL_INTB_EN,
			   INT_ILP);
	if (ret)
		return ret;

	/* Determine which interrupt pin the board uses */
	pcf85363->irq_type[IRQPIN_INTA] = INT_A1IE;
	pcf85363->irq_type[IRQPIN_INTB] = 0;
	for (i = 0; i < IRQPIN_MAX; i++) {
		const char *irq_output_pin;
		u32 irq_type = 0;

		if (pcf85363_can_wakeup_machine(pcf85363)) {
			if (!of_property_read_string_index(pcf85363->dev->of_node,
							   "nxp,rtc-interrupt-output-pin",
							   i, &irq_output_pin))
				if (!strncmp(pcf85363_irqpin_names[i],
					     irq_output_pin,
					     strlen(pcf85363_irqpin_names[i]))) {
					if (!of_property_read_u32_index(pcf85363->dev->of_node,
									"nxp,rtc-interrupt-type",
									i, &irq_type))
						pcf85363->irq_type[IRQPIN_INTA] = (u8)(0xff & irq_type);
				}
		}

		/* Setup IO pin config register */
		regval = PIN_IO_CLKPM; /* disable CLK pin*/
		if (pcf85363->irq_type[i]) {
			switch (i) {
			case IRQPIN_INTA:
				regval |= (PIN_IO_INTA_OUT | PIN_IO_TS_DISABLE);
				break;
			case IRQPIN_INTB:
				regval |= (PIN_IO_INTA_HIZ | PIN_IO_TS_INTB_OUT);
				break;
			default:
				dev_err(pcf85363->dev, "Failed to set interrupt out pin\n");
				return -EINVAL;
			}
			ret = regmap_write(pcf85363->regmap, CTRL_PIN_IO, regval);
		}
	}
	return ret;
}


static int pcf85363_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct pcf85363 *pcf85363;
	const struct pcf85x63_config *config = &pcf_85363_config;
	const void *data = of_device_get_match_data(&client->dev);
	static struct nvmem_config nvmem_cfg[] = {
		{
			.name = "pcf85x63-",
			.word_size = 1,
			.stride = 1,
			.size = 1,
			.reg_read = pcf85x63_nvram_read,
			.reg_write = pcf85x63_nvram_write,
		}, {
			.name = "pcf85363-",
			.word_size = 1,
			.stride = 1,
			.size = NVRAM_SIZE,
			.reg_read = pcf85363_nvram_read,
			.reg_write = pcf85363_nvram_write,
		},
	};
	int ret, i;

	if (data)
		config = data;

	pcf85363 = devm_kzalloc(&client->dev, sizeof(struct pcf85363),
				GFP_KERNEL);
	if (!pcf85363)
		return -ENOMEM;

	pcf85363->regmap = devm_regmap_init_i2c(client, &config->regmap);
	if (IS_ERR(pcf85363->regmap)) {
		dev_err(&client->dev, "regmap allocation failed\n");
		return PTR_ERR(pcf85363->regmap);
	}

	pcf85363->irq = client->irq;
	pcf85363->dev = &client->dev;
	i2c_set_clientdata(client, pcf85363);

	pcf85363->rtc = devm_rtc_allocate_device(pcf85363->dev);
	if (IS_ERR(pcf85363->rtc))
		return PTR_ERR(pcf85363->rtc);

	pcf85363->rtc->ops = &rtc_ops;
	pcf85363->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	pcf85363->rtc->range_max = RTC_TIMESTAMP_END_2099;

	ret = pcf85363_init_hw(pcf85363);
	if (ret)
		return ret;

	if (pcf85363->irq > 0) {
		regmap_write(pcf85363->regmap, CTRL_FLAGS, 0);
		ret = devm_request_threaded_irq(pcf85363->dev, pcf85363->irq,
						NULL, pcf85363_rtc_handle_irq,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						"pcf85363", client);
		if (ret) {
			dev_warn(&client->dev, "unable to request IRQ, alarms disabled\n");
			pcf85363->irq = 0;
		}
		else
			pcf85363->rtc->ops = &rtc_ops_alarm;
	}

	if (pcf85363_can_wakeup_machine(pcf85363))
		device_init_wakeup(pcf85363->dev, true);

	ret = rtc_register_device(pcf85363->rtc);

	for (i = 0; i < config->num_nvram; i++) {
		nvmem_cfg[i].priv = pcf85363;
		rtc_nvmem_register(pcf85363->rtc, &nvmem_cfg[i]);
	}

	/* We cannot support UIE mode if we do not have an IRQ line */
	if (!pcf85363->irq)
		pcf85363->rtc->uie_unsupported = 1;

	return ret;
}

static const struct of_device_id dev_ids[] = {
	{ .compatible = "nxp,pcf85263", .data = &pcf_85263_config },
	{ .compatible = "nxp,pcf85363", .data = &pcf_85363_config },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dev_ids);

static int pcf85363_remove(struct i2c_client *client)
{
	struct pcf85363 *pcf85363 = i2c_get_clientdata(client);

	if (pcf85363_can_wakeup_machine(pcf85363))
		device_init_wakeup(pcf85363->dev, false);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pcf85363_suspend(struct device *dev)
{
	struct pcf85363 *pcf85363 = dev_get_drvdata(dev);
	int ret = 0;

	if (device_may_wakeup(dev))
		ret = enable_irq_wake(pcf85363->irq);

	return ret;
}

static int pcf85363_resume(struct device *dev)
{
	struct pcf85363 *pcf85363 = dev_get_drvdata(dev);
	int ret = 0;

	if (device_may_wakeup(dev))
		ret = disable_irq_wake(pcf85363->irq);

	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(pcf85363_pm_ops, pcf85363_suspend,  pcf85363_resume);

static struct i2c_driver pcf85363_driver = {
	.driver	= {
		.name	= "pcf85363",
		.of_match_table = of_match_ptr(dev_ids),
		.pm = &pcf85363_pm_ops,
	},
	.probe	= pcf85363_probe,
	.remove	= pcf85363_remove,
};

module_i2c_driver(pcf85363_driver);

MODULE_AUTHOR("Eric Nelson");
MODULE_DESCRIPTION("pcf85263/pcf85363 I2C RTC driver");
MODULE_LICENSE("GPL");
