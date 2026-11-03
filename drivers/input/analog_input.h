/*
 * Driver d'entree analogique (joystick) : lit des canaux ADC et emet leur
 * deflexion comme evenements d'entree Zephyr.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>

#define ANALOG_INPUT_MAX_CHANNELS 8

struct analog_input_io_channel {
    struct adc_dt_spec adc_channel;
    uint16_t mv_mid;
    uint16_t mv_min_max;
    uint16_t mv_deadzone; // uint16 : une zone morte > 255 mV etait tronquee en uint8
    bool invert;
    bool report_on_change_only;
    uint8_t evt_type;
    uint16_t input_code;
};

struct analog_input_config {
    uint32_t sampling_hz;
    uint8_t io_channels_len;
    struct analog_input_io_channel io_channels[];
};

struct analog_input_data {
    const struct device *dev;
    struct k_timer sampling_timer;
    struct k_work sampling_work;
    bool calibrated;
    uint8_t adc_errors;
    int32_t prev[ANALOG_INPUT_MAX_CHANNELS];
};
