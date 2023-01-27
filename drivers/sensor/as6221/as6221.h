/*
 * Copyright (c) 2020 Innoseis BV
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_AS6221_AS6221_H_
#define ZEPHYR_DRIVERS_SENSOR_AS6221_AS6221_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>

#define AS6221_REG_TEMPERATURE          0x00

#define AS6221_REG_CONFIG   0x01

#define AS6221_ALERT_BIT     BIT(5)

#define AS6221_CONV_RATE_SHIFT  6
#define AS6221_CONV_RATE_MASK   (BIT_MASK(2) <<  AS6221_CONV_RATE_SHIFT)
#define AS6221_CONV_RATE_250    0
#define AS6221_CONV_RATE_1000   1
#define AS6221_CONV_RATE_4000   2
#define AS6221_CONV_RATE_8000   3
#define AS6221_CONV_RATE(cr)    ((cr) << AS6221_CONV_RATE_SHIFT)

#define AS6221_CONFIG_SHIFT		8
#define AS6221_CONFIG_MASK		(BIT_MASK(8) << AS6221_CONFIG_SHIFT)
#define AS6221_SLEEP_MODE		BIT(8)
#define AS6221_INT_MODE			BIT(9)
#define AS6221_POLARITY			BIT(10)
#define AS6221_CF_0				BIT(11)
#define AS6221_CF_1				BIT(12)
#define AS6221_SINGLE_SHOT      BIT(15)
#define AS6221_CONFIG_COMBO		(AS6221_SLEEP_MODE | AS6221_INT_MODE | AS6221_POLARITY | AS6221_CF_0 | AS6221_CF_1 | AS6221_SINGLE_SHOT)

#define AS6221_REG_TLOW         0x02
#define AS6221_REG_THIGH        0x03

/* scale in micro degrees Celsius */
#define AS6221_TEMP_SCALE       	78125
#define AS6221_TEMP_SCALE_FACTOR	10

struct as6221_data {
	int16_t sample;
	uint16_t config_reg;
};

struct as6221_config {
	const struct i2c_dt_spec bus;
	uint8_t cr;
#ifdef CONFIG_AS6221_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif /* CONFIG_AS6221_TRIGGER */
};

#endif
