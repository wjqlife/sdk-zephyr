/*
 * Copyright (c) 2016 Firmwave
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_as6221

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include "as6221.h"

LOG_MODULE_REGISTER(AS6221, CONFIG_SENSOR_LOG_LEVEL);

static int as6221_reg_read(const struct as6221_config *cfg,
			   uint8_t reg, uint16_t *val)
{
	if (i2c_burst_read_dt(&cfg->bus, reg, (uint8_t *)val, sizeof(*val)) < 0) {
		return -EIO;
	}

	*val = sys_be16_to_cpu(*val);

	return 0;
}

static int as6221_reg_write(const struct as6221_config *cfg,
			    uint8_t reg, uint16_t val)
{
	uint8_t buf[3];

	buf[0] = reg;
	sys_put_be16(val, &buf[1]);

	return i2c_write_dt(&cfg->bus, buf, sizeof(buf));
}

static uint16_t set_config_flags(struct as6221_data *data, uint16_t mask,
				 uint16_t value)
{
	return (data->config_reg & ~mask) | (value & mask);
}

static int as6221_update_config(const struct device *dev, uint16_t mask,
				uint16_t val)
{
	int rc;
	struct as6221_data *data = dev->data;
	const uint16_t new_val = set_config_flags(data, mask, val);

	rc = as6221_reg_write(dev->config, AS6221_REG_CONFIG, new_val);
	if (rc == 0) {
		data->config_reg = new_val;
	}

	return rc;
}

static int as6221_attr_set(const struct device *dev,
			   enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	uint16_t value, cr, reg_val;
	const struct as6221_config *cfg = dev->config;
	int32_t reg_temp_upper, reg_temp_lower, buf_temp_upper, buf_temp_lower;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	switch (attr) {
	case SENSOR_ATTR_LOWER_THRESH:
#if CONFIG_AS6221_CONFIGURATION_RUNTIME
		/* val1 stored the limit in million times*/
		/* range is -40 to 125 */
		if (as6221_reg_read(cfg, AS6221_REG_THIGH, &reg_val) < 0) {
			LOG_DBG("Failed to read upper temperature threshold!");
			return -EIO;
		}
		reg_temp_upper = (int32_t)reg_val * AS6221_TEMP_SCALE / AS6221_TEMP_SCALE_FACTOR;
		if ((val->val1 < -40000000) || (val->val1 > 125000000) || (val->val1>reg_temp_upper)) {
			LOG_DBG("Lower temperature threshold set value out of range!");
			return -EOVERFLOW;
		}
		else {
			reg_temp_lower = val->val1;
		}
		buf_temp_lower = reg_temp_lower * AS6221_TEMP_SCALE_FACTOR / AS6221_TEMP_SCALE;
		value = (uint16_t)(buf_temp_lower & 0xFFF0);
		if (as6221_reg_write(dev->config, AS6221_REG_TLOW, value) < 0) {
			LOG_DBG("Failed to write lower temperature threshold!");
			return -EIO;
		}
		break;
#endif
	case SENSOR_ATTR_UPPER_THRESH:
#if CONFIG_AS6221_CONFIGURATION_RUNTIME
		/* val1 stored the limit in million times*/
		/* range is -40 to 125 */
		if (as6221_reg_read(cfg, AS6221_REG_TLOW, &reg_val) < 0) {
			LOG_DBG("Failed to read lower temperature threshold!");
			return -EIO;
		}
		reg_temp_lower = (int32_t)reg_val * AS6221_TEMP_SCALE / AS6221_TEMP_SCALE_FACTOR;
		if ((val->val1 < -40000000) || (val->val1 > 125000000) || (val->val1<reg_temp_lower)) {
			LOG_DBG("Upper temperature threshold set value out of range!");
			return -ENOTSUP;
		}
		else {
			reg_temp_upper = val->val1;
		}
		buf_temp_upper = reg_temp_upper * AS6221_TEMP_SCALE_FACTOR / AS6221_TEMP_SCALE;
		value = (uint16_t)(buf_temp_upper & 0xFFF0);
		if (as6221_reg_write(dev->config, AS6221_REG_THIGH, value) < 0) {
			LOG_DBG("Failed to write upper temperature threshold!");
			return -EIO;
		}
		break;
#endif
	case SENSOR_ATTR_CONFIGURATION:
#if CONFIG_AS6221_CONFIGURATION_RUNTIME
		/* configuration bits order: SINGLE_SHOT(BIT15), CONSECUTIVE FAULTS1(BIT12), CONSECUTIVE FAULTS2(BIT11), POLARITY(BIT10), INTERRUPT MODE(BIT9), and SLEEP MODE(BIT8) */
		value = val->val1 & AS6221_CONFIG_COMBO;

		if (as6221_update_config(dev, AS6221_CONFIG_MASK, value) < 0) {
			LOG_DBG("Failed to update configuration!");
			return -EIO;
		}

		break;
#endif
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
#if CONFIG_AS6221_SAMPLING_FREQUENCY_RUNTIME
		/* conversion rate in mHz */
		cr = val->val1 * 1000 + val->val2 / 1000;

		/* the sensor supports 0.25Hz, 1Hz, 4Hz and 8Hz */
		/* conversion rate */
		switch (cr) {
		case 250:
			value = AS6221_CONV_RATE(AS6221_CONV_RATE_250);
			break;

		case 1000:
			value = AS6221_CONV_RATE(AS6221_CONV_RATE_1000);
			break;

		case 4000:
			value = AS6221_CONV_RATE(AS6221_CONV_RATE_4000);
			break;

		case 8000:
			value = AS6221_CONV_RATE(AS6221_CONV_RATE_8000);
			break;

		default:
			return -ENOTSUP;
		}

		if (as6221_update_config(dev, AS6221_CONV_RATE_MASK, value) < 0) {
			LOG_DBG("Failed to set conversion rate!");
			return -EIO;
		}

		break;
#endif

	default:
		return -ENOTSUP;
	}

	return 0;
}

static int as6221_attr_get(const struct device * dev, 
			   enum sensor_channel chan, 
			   enum sensor_attribute attr, 
			   struct sensor_value * val)
{
	struct as6221_data *drv_data = dev->data;
	const struct as6221_config *cfg = dev->config;
	uint16_t reg_val;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AMBIENT_TEMP);

	switch (attr) {
	case SENSOR_ATTR_LOWER_THRESH:
		if (as6221_reg_read(cfg, AS6221_REG_TLOW, &reg_val) < 0) {
			LOG_DBG("Failed to read lower temperature threshold!");
			return -EIO;
		}
		val->val1 = reg_val;
		break;
	case SENSOR_ATTR_UPPER_THRESH:
		if (as6221_reg_read(cfg, AS6221_REG_THIGH, &reg_val) < 0) {
			LOG_DBG("Failed to upper lower temperature threshold!");
			return -EIO;
		}
		val->val1 = reg_val;
		break;
	case SENSOR_ATTR_CONFIGURATION:
		if (as6221_reg_read(cfg, AS6221_REG_CONFIG, &reg_val) < 0) {
			LOG_DBG("Failed to read configuration setting!");
			return -EIO;
		}
		val->val1 = reg_val;
		break;
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		if (as6221_reg_read(cfg, AS6221_REG_CONFIG, &reg_val) < 0) {
			LOG_DBG("Failed to read conversion rate setting!");
			return -EIO;
		}
		val->val1 = reg_val & AS6221_CONV_RATE_MASK;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int as6221_trigger_set(const struct device *dev,
				   const struct sensor_trigger *trig,
				   sensor_trigger_handler_t handler)
{
	
}

static int as6221_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct as6221_data *drv_data = dev->data;
	const struct as6221_config *cfg = dev->config;
	uint16_t val;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AMBIENT_TEMP);

	if (as6221_reg_read(cfg, AS6221_REG_TEMPERATURE, &val) < 0) {
		LOG_DBG("Failed to read temperature register!");
		return -EIO;
	}
	
	drv_data->sample = (int16_t)val;

	return 0;
}

static int as6221_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct as6221_data *drv_data = dev->data;
	int32_t uval;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	uval = (int32_t)drv_data->sample * AS6221_TEMP_SCALE / AS6221_TEMP_SCALE_FACTOR;
	val->val1 = uval / 1000000;
	val->val2 = uval % 1000000;

	return 0;
}

static const struct sensor_driver_api as6221_driver_api = {
	.attr_set = as6221_attr_set,
	.attr_get = as6221_attr_get,
	.trigger_set = as6221_trigger_set,
	.sample_fetch = as6221_sample_fetch,
	.channel_get = as6221_channel_get,
};

int as6221_init(const struct device *dev)
{
	const struct as6221_config *cfg = dev->config;
	struct as6221_data *data = dev->data;

	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C dev %s not ready", cfg->bus.bus->name);
		return -EINVAL;
	}

	data->config_reg = AS6221_CONV_RATE(cfg->cr);

	return as6221_update_config(dev, 0, 0);
}


#define AS6221_INST(inst)						    \
	static struct as6221_data as6221_data_##inst;			    \
	static const struct as6221_config as6221_config_##inst = {	    \
		.bus = I2C_DT_SPEC_INST_GET(inst),			    \
		.cr = DT_INST_ENUM_IDX(inst, conversion_rate),		    \
IF_ENABLED(CONFIG_AS6221_TRIGGER,						\
		   (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }),))	\
	};								    \
									    \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, as6221_init, NULL, &as6221_data_##inst, \
			      &as6221_config_##inst, POST_KERNEL,	    \
			      CONFIG_SENSOR_INIT_PRIORITY, &as6221_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AS6221_INST)