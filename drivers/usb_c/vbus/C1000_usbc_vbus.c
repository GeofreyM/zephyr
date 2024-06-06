/*
 * Copyright(c) 2024 TRILYS SAS
 */

#include "/media/veracrypt2/zephyr_workspace/C1000/inc/BSP_analog.h"

/* Derived from zephyr/drivers/usb_c/vbus/usbc_vbus_adc.c
 * Original copyright :
 *
 * Copyright 2022 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT c1000_usb_c_vbus

#include <soc.h>
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/drivers/usb_c/usbc_pd.h>
#include <zephyr/drivers/usb_c/usbc_vbus.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#if !DT_NODE_EXISTS(DT_NODELABEL(vbus1))
#error "Overlay for vbus node not properly defined."
#endif

/**
 * @brief Driver config
 */
struct usbc_vbus_config {
};

/**
 * @brief Driver data
 */
struct usbc_vbus_data {
	int sample;
};

/**
 * @brief Reads and returns VBUS measured in mV
 *
 * @retval 0 on success
 * @retval -EIO on failure
 */
static int adc_vbus_measure(const struct device *dev, int *meas)
{
	__ASSERT(meas != NULL, "ADC VBUS meas must not be NULL");
	*meas = analog_usb_mv();
	return 0;
}

/**
 * @brief Checks if VBUS is at a particular level
 *
 * @retval true if VBUS is at the level voltage, else false
 */
static bool adc_vbus_check_level(const struct device *dev, enum tc_vbus_level level)
{
	int meas;
	int ret;

	ret = adc_vbus_measure(dev, &meas);
	if (ret) {
		return false;
	}

	switch (level) {
	case TC_VBUS_SAFE0V:
		return (meas < PD_V_SAFE_0V_MAX_MV);
	case TC_VBUS_PRESENT:
		return (meas >= PD_V_SAFE_5V_MIN_MV);
	case TC_VBUS_REMOVED:
		return (meas < TC_V_SINK_DISCONNECT_MAX_MV);
	}

	return false;
}

/**
 * @brief Sets pin to discharge VBUS
 *
 * @retval 0 on success
 * @retval -EIO on failure
 * @retval -ENOENT if enable pin isn't defined
 */
static int adc_vbus_discharge(const struct device *dev, bool enable)
{
	// TODO : disabling STo_USB would not do much given ideal diode U9/Q14 and Q13/Q15 internal
	// diodes. Maybe disable STo_ON?
	// TODO, if still not enough, add active discharge circuit
	return 0;
}

/**
 * @brief Sets pin to enable VBUS measurments
 *
 * @retval 0 on success
 * @retval -EIO on failure
 * @retval -ENOENT if enable pin isn't defined
 */
static int adc_vbus_enable(const struct device *dev, bool enable)
{
	return 0;
}

/**
 * @brief Initializes the ADC VBUS Driver
 *
 * @retval 0 on success
 * @retval -EIO on failure
 */
static int adc_vbus_init(const struct device *dev)
{

	// TODO add a check that ADC DMA is started ?

	return 0;
}

static const struct usbc_vbus_driver_api driver_api = {.measure = adc_vbus_measure,
						       .check_level = adc_vbus_check_level,
						       .discharge = adc_vbus_discharge,
						       .enable = adc_vbus_enable};

#define DRIVER_INIT(inst)                                                                          \
	static struct usbc_vbus_data drv_data_##inst;                                              \
	static const struct usbc_vbus_config drv_config_##inst = {};                               \
	DEVICE_DT_INST_DEFINE(inst, &adc_vbus_init, NULL, &drv_data_##inst, &drv_config_##inst,    \
			      POST_KERNEL, CONFIG_USBC_VBUS_INIT_PRIORITY, &driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INIT)