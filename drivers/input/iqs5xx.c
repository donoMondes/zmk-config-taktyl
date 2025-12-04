/*
 * Copyright (c) 2025 Mariano Uvalle
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_iqs5xx

#include <stdlib.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "iqs5xx.h"

LOG_MODULE_REGISTER(iqs5xx, CONFIG_INPUT_LOG_LEVEL);

struct k_mutex iqs5xx_mutex;

static int iqs5xx_read_reg16(const struct device *dev, uint16_t reg, uint16_t *val) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[2];
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
    int ret;

    ret = i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), buf, sizeof(buf));
    if (ret < 0) {
        return ret;
    }

    *val = (buf[0] << 8) | buf[1];
    return 0;
}

static int iqs5xx_write_reg16(const struct device *dev, uint16_t reg, uint16_t val) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[4] = {reg >> 8, reg & 0xFF, val >> 8, val & 0xFF};

    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

static int iqs5xx_read_reg8(const struct device *dev, uint16_t reg, uint8_t *val) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};

    return i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), val, 1);
}

static int iqs5xx_write_reg8(const struct device *dev, uint16_t reg, uint8_t val) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[3] = {reg >> 8, reg & 0xFF, val};

    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

static int iqs5xx_end_comm_window(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[3] = {IQS5XX_END_COMM_WINDOW >> 8, IQS5XX_END_COMM_WINDOW & 0xFF, 0x00};

    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

static void iqs5xx_button_release_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, button_release_work);

    k_mutex_lock(&iqs5xx_mutex, K_FOREVER);
    if (data->tap_in_progress) {
        input_report_key(data->dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
        data->tap_in_progress = false;
    }
    if (data->two_finger_tap_in_progress) {
        input_report_key(data->dev, RIGHT_BUTTON_CODE, 0, true, K_FOREVER);
        data->two_finger_tap_in_progress = false;
    }
    k_mutex_unlock(&iqs5xx_mutex);
}

static void iqs5xx_work_handler(struct k_work *work) {
    struct iqs5xx_data *data = CONTAINER_OF(work, struct iqs5xx_data, work);
    const struct device *dev = data->dev;
    const struct iqs5xx_config *config = dev->config;
    int ret;

    uint16_t addr;

    struct iqs5xx_sys_info sys_info;
    struct iqs5xx_gesture_data gesture_events;
    struct iqs5xx_touch touch_data;
    memset(&sys_info,0,sizeof(struct iqs5xx_sys_info));
    memset(&gesture_events,0,sizeof(struct iqs5xx_gesture_data));
    memset(&touch_data,0,sizeof(struct iqs5xx_touch));

    addr = TO_LE(IQS5XX_SYSTEM_INFO_0);

    // Read system info registers.
    ret = i2c_write_read_dt(&config->i2c, &addr, sizeof(addr), (uint8_t *)&sys_info,2);
    if (ret < 0) {
        LOG_ERR("Failed to read system info 0: %d", ret);
        iqs5xx_end_comm_window(dev);
    }

    addr = TO_LE(IQS5XX_GESTURE_EVENTS_0);
    ret = i2c_write_read_dt(&config->i2c, &addr, sizeof(addr), (uint8_t *)&gesture_events,2);
    if (ret < 0) {
        LOG_ERR("Failed to read gesture events: %d", ret);
        iqs5xx_end_comm_window(dev);
    }

    // Handle reset indication.
    if (sys_info.sys_info_0.show_reset) {
        LOG_INF("Device reset detected");
        // Acknowledge reset.
        iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONTROL_0, IQS5XX_ACK_RESET);
        iqs5xx_end_comm_window(dev);
    }

    k_mutex_lock(&iqs5xx_mutex, K_FOREVER);
    if (!gesture_events.gesture_events_0.scroll) {
        // Clear accumulators if we're not actively scrolling.
        data->scroll_x_acc = 0;
        data->scroll_y_acc = 0;
    }

    uint16_t button_code;
    bool button_pressed = false;
    if (gesture_events.gesture_events_0.single_tap
        && (!gesture_events.gesture_events_1.scroll
        || !gesture_events.gesture_events_1.zoom)) {
        button_pressed = true;
        data->tap_in_progress = true;
        button_code = INPUT_BTN_0;
    } 
    else if (gesture_events.gesture_events_1.two_finger_tap
        && (!gesture_events.gesture_events_1.scroll
        || !gesture_events.gesture_events_1.zoom)) {
        button_pressed = true;
        data->two_finger_tap_in_progress = true;
        button_code = INPUT_BTN_1;
    }

    addr = TO_LE(IQS5XX_REL_X);

    int16_t rel_x, rel_y;
    if (sys_info.sys_info_1.tp_movement 
        || gesture_events.gesture_events_0.scroll) {
        ret = i2c_write_read_dt(&config->i2c, &addr, sizeof(addr), (uint8_t *)&touch_data,4);
        if (ret < 0) {
            LOG_ERR("Failed to read relative touch data: %d", ret);
            iqs5xx_end_comm_window(dev);
        }
    }
    rel_x = (int16_t)AZOTEQ_IQS5XX_COMBINE_H_L_BYTES(touch_data.rel_x.h, touch_data.rel_x.l);
    rel_y = (int16_t)AZOTEQ_IQS5XX_COMBINE_H_L_BYTES(touch_data.rel_y.h, touch_data.rel_y.l);
    // Handle movement and gestures.
    //
    // Each one of these branches needs to send the last report it makes as
    // sync to ensure that the input subsystem processes things in order.
    if (gesture_events.gesture_events_0.press_and_hold 
        && !data->active_hold) {
        LOG_INF("Hold became active");
        input_report_key(dev, LEFT_BUTTON_CODE, 1, true, K_FOREVER);
        data->active_hold = true;
    } 
    else if (!gesture_events.gesture_events_0.press_and_hold 
        && data->active_hold) {
        LOG_INF("Hold became inactive");
        input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
        data->active_hold = false;
    } 
    else if (button_pressed) {
        // Cancel any pending release.
        k_work_cancel_delayable(&data->button_release_work);

        // Press the button immediately.
        input_report_key(dev, button_code, 1, true, K_FOREVER);

        // Schedule release after 100ms.
        k_work_schedule(&data->button_release_work, K_MSEC(100));
    } 
    else if (gesture_events.gesture_events_0.scroll) {
        // TODO: Expose this divisor.
        int16_t scroll_div = 32;

        // Only one scrolling direction is valid at a time.
        // End the communication right after reporting the movement.
        if (rel_x != 0) {
            // By default the x axis is already "natural".
            if (!config->natural_scroll_x) {
                rel_x *= -1;
            }
            data->scroll_x_acc += rel_x;
            if (abs(data->scroll_x_acc) >= scroll_div) {
                input_report_rel(dev, INPUT_REL_HWHEEL, data->scroll_x_acc / scroll_div, true,
                                K_FOREVER);
                data->scroll_x_acc %= scroll_div;
            }
            iqs5xx_end_comm_window(dev);
        }
        if (rel_y != 0) {
            if (config->natural_scroll_y) {
                rel_y *= -1;
            }
            data->scroll_y_acc += rel_y;
            if (abs(data->scroll_y_acc) >= scroll_div) {
                input_report_rel(dev, INPUT_REL_WHEEL, data->scroll_y_acc / scroll_div, true,
                                 K_FOREVER);
                data->scroll_y_acc %= scroll_div;
            }
            iqs5xx_end_comm_window(dev);
        }
    } 
    else if (sys_info.sys_info_1.tp_movement && !sys_info.sys_info_1.palm_detect) {
        if (rel_x != 0 || rel_y != 0) {
            input_report_rel(dev, INPUT_REL_X, rel_x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, rel_y, true, K_FOREVER);
        }
    }
    k_mutex_unlock(&iqs5xx_mutex);
    iqs5xx_end_comm_window(dev);
}

static void iqs5xx_rdy_handler(const struct device *port, struct gpio_callback *cb,
                               gpio_port_pins_t pins) {
    struct iqs5xx_data *data = CONTAINER_OF(cb, struct iqs5xx_data, rdy_cb);

    k_work_submit(&data->work);
}

static int iqs5xx_setup_device(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    int ret;


    // Change resolution
    ret = iqs5xx_write_reg16(dev, IQS5XX_RESOLUTION_X,config->resolution_x);
    ret |= iqs5xx_write_reg16(dev, IQS5XX_RESOLUTION_Y,config->resolution_y);

    // Change report rate value
    ret |= iqs5xx_write_reg16(dev, IQS5XX_REPORT_RATE_ACTIVE_MODE,config->report_rate_active_mode);
    ret |= iqs5xx_write_reg16(dev, IQS5XX_REPORT_RATE_IDLE_TOUCH_MODE,config->report_rate_active_mode*5);
    ret |= iqs5xx_write_reg16(dev, IQS5XX_REPORT_RATE_IDLE_MODE,config->report_rate_active_mode*5);

    // Enable event mode and trackpad events.
    ret |= iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_1,
                            IQS5XX_EVENT_MODE | IQS5XX_TP_EVENT | IQS5XX_GESTURE_EVENT);
    if (ret < 0) {
        LOG_ERR("Failed to configure event mode: %d", ret);
        return ret;
    }

    // Configure the palm reject area size threshold
    ret = iqs5xx_write_reg8(dev, IQS5XX_PALM_REJECT_THRESHOLD,config->palm_reject_threshold);
    if (ret < 0) {
        LOG_ERR("Failed to configure the plam reject threshold: %d", ret);
        return ret;
    }

    // Configure the palm reject timeout
    ret = iqs5xx_write_reg8(dev, IQS5XX_PALM_REJECT_TIMEOUT,config->palm_reject_timeout);
    if (ret < 0) {
        LOG_ERR("Failed to configure the palm reject timeout: %d", ret);
        return ret;
    }

    ret = iqs5xx_write_reg8(dev, IQS5XX_BOTTOM_BETA, config->bottom_beta);
    if (ret < 0) {
        LOG_ERR("Failed to set bottom beta: %d", ret);
        return ret;
    }

    ret = iqs5xx_write_reg8(dev, IQS5XX_STATIONARY_THRESH, config->stationary_threshold);
    if (ret < 0) {
        LOG_ERR("Failed to set bottom stationary threshold: %d", ret);
        return ret;
    }

    // TODO: Expose these through dts bindings.
    // Set filter settings with:
    // - IIR filter enabled
    // - MAV filter enabled
    // - IIR select disabled (dynamic IIR)
    // - ALP count filter enabled
    ret = iqs5xx_write_reg8(dev, IQS5XX_FILTER_SETTINGS,
                            IQS5XX_IIR_FILTER | IQS5XX_MAV_FILTER | IQS5XX_ALP_COUNT_FILTER);
    if (ret < 0) {
        LOG_ERR("Failed to configure filter settings: %d", ret);
        return ret;
    }

    uint8_t single_finger_gestures = 0;
    single_finger_gestures |= config->one_finger_tap ? IQS5XX_SINGLE_TAP : 0;
    single_finger_gestures |= config->press_and_hold ? IQS5XX_PRESS_AND_HOLD : 0;
    // Configure single finger gestures.
    ret = iqs5xx_write_reg8(dev, IQS5XX_SINGLE_FINGER_GESTURES_CONF, single_finger_gestures);
    if (ret < 0) {
        LOG_ERR("Failed to configure single finger gestures: %d", ret);
        return ret;
    }
    // Configure the hold time for the press and hold gesture.
    ret = iqs5xx_write_reg16(dev, IQS5XX_HOLD_TIME, config->press_and_hold_time);
    if (ret < 0) {
        LOG_ERR("Failed to configure the hold time: %d", ret);
        return ret;
    }

    uint8_t two_finger_gestures = 0;
    two_finger_gestures |= config->two_finger_tap ? IQS5XX_TWO_FINGER_TAP : 0;
    two_finger_gestures |= config->scroll ? IQS5XX_SCROLL : 0;
    two_finger_gestures |= IQS5XX_ZOOM;
    // Configure multi finger gestures.
    ret = iqs5xx_write_reg8(dev, IQS5XX_MULTI_FINGER_GESTURES_CONF, two_finger_gestures);
    if (ret < 0) {
        LOG_ERR("Failed to configure multi finger gestures: %d", ret);
        return ret;
    }

    // Configure the hold time for the press and hold gesture.
    ret |=iqs5xx_write_reg16(dev, IQS5XX_TAP_TIME, DEFAULT_TAP_TIME);
    ret |=iqs5xx_write_reg16(dev, IQS5XX_TAP_DISTANCE, DEFAULT_TAP_DISTANCE);
    ret |=iqs5xx_write_reg16(dev, IQS5XX_HOLD_TIME, DEFAULT_HOLD_TIME);
    ret |=iqs5xx_write_reg16(dev, IQS5XX_SWIPE_INIT_TIME, DEFAULT_SWIPE_INITIAL_TIME);
    ret |=iqs5xx_write_reg16(dev, IQS5XX_SWIPE_INIT_DISTANCE, DEFAULT_SWIPE_INITIAL_DISTANCE);
    ret |=iqs5xx_write_reg16(dev, IQS5XX_SWIPE_CONSEC_TIME, DEFAULT_SWIPE_CONSECUTIVE_TIME);
    ret |=iqs5xx_write_reg16(dev, IQS5XX_SWIPE_CONSEC_DISTANCE, DEFAULT_SWIPE_CONSECUTIVE_DISTANCE);
    ret |=iqs5xx_write_reg16(dev, IQS5XX_SCROLL_INIT_DISTANCE, DEFAULT_SCROLL_INITIAL_DISTANCE);
    ret |=iqs5xx_write_reg16(dev, IQS5XX_ZOOM_INIT_DISTANCE, DEFAULT_ZOOM_INITIAL_DISTANCE);
    ret |=iqs5xx_write_reg16(dev, IQS5XX_ZOOM_CONSEC_DISTANCE, DEFAULT_ZOOM_CONSECUTIVE_DISTANCE);
    if (ret < 0) {
        LOG_ERR("Failed to configure timing gesture settings: %d", ret);
        return ret;
    }

    // Configure axes.
    uint8_t xy_config = 0;
    xy_config |= config->flip_x ? IQS5XX_FLIP_X : 0;
    xy_config |= config->flip_y ? IQS5XX_FLIP_Y : 0;
    xy_config |= config->switch_xy ? IQS5XX_SWITCH_XY_AXIS : 0;
    xy_config |= config->palm_rejection ? IQS5XX_ALLOW_PALM_REJECT : 0;
    ret = iqs5xx_write_reg8(dev, IQS5XX_XY_CONFIG_0, xy_config);
    if (ret < 0) {
        LOG_ERR("Failed to configure axes: %d", ret);
        return ret;
    }

    // Configure system settings.
    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_0, IQS5XX_SETUP_COMPLETE | IQS5XX_WDT | IQS5XX_REATI);
    if (ret < 0) {
        LOG_ERR("Failed to configure system: %d", ret);
        return ret;
    }

    // End communication window.
    ret = iqs5xx_end_comm_window(dev);
    if (ret < 0) {
        LOG_ERR("Failed to end comm window during initialization: %d", ret);
        return ret;
    }

    return 0;
}

static int iqs5xx_init(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    struct iqs5xx_data *data = dev->data;
    int ret;

    if (!i2c_is_ready_dt(&config->i2c)) { 
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    k_mutex_init(&iqs5xx_mutex);

    data->dev = dev;
    k_work_init(&data->work, iqs5xx_work_handler);
    k_work_init_delayable(&data->button_release_work, iqs5xx_button_release_work_handler);

    // Configure reset GPIO if available.
    if (config->reset_gpio.port) {
        if (!gpio_is_ready_dt(&config->reset_gpio)) {
            LOG_ERR("Reset GPIO not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure reset GPIO: %d", ret);
            return ret;
        }

        // Reset the device.
        gpio_pin_set_dt(&config->reset_gpio, 1);
        k_msleep(1);
        gpio_pin_set_dt(&config->reset_gpio, 0);
        k_msleep(10);
    }

    // Configure RDY GPIO.
    if (!gpio_is_ready_dt(&config->rdy_gpio)) {
        LOG_ERR("RDY GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->rdy_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure RDY GPIO: %d", ret);
        return ret;
    }

    gpio_init_callback(&data->rdy_cb, iqs5xx_rdy_handler, BIT(config->rdy_gpio.pin));
    ret = gpio_add_callback(config->rdy_gpio.port, &data->rdy_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add RDY callback: %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_EDGE_RISING);
    if (ret < 0) {
        LOG_ERR("Failed to configure RDY interrupt: %d", ret);
        return ret;
    }

    // Wait for device to be ready.
    k_msleep(100);

    // Setup device configuration.
    ret = iqs5xx_setup_device(dev);
    if (ret < 0) {
        LOG_ERR("Failed to setup device: %d", ret);
        return ret;
    }

    data->initialized = true;
    LOG_INF("IQS5xx trackpad initialized");

    return 0;
}

// Replace CONFIG_INPUT_INIT_PRIORITY with the azoteq specific value.
#define IQS5XX_INIT(n)                                                                             \
    static struct iqs5xx_data iqs5xx_data_##n;                                                     \
    static const struct iqs5xx_config iqs5xx_config_##n = {                                        \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                            \
        .rdy_gpio = GPIO_DT_SPEC_INST_GET(n, rdy_gpios),                                           \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                               \
        .one_finger_tap = DT_INST_PROP(n, one_finger_tap),                                         \
        .press_and_hold = DT_INST_PROP(n, press_and_hold),                                         \
        .two_finger_tap = DT_INST_PROP(n, two_finger_tap),                                         \
        .scroll = DT_INST_PROP(n, scroll),                                                         \
        .natural_scroll_x = DT_INST_PROP(n, natural_scroll_x),                                     \
        .natural_scroll_y = DT_INST_PROP(n, natural_scroll_y),                                     \
        .press_and_hold_time = DT_INST_PROP_OR(n, press_and_hold_time, 250),                       \
        .switch_xy = DT_INST_PROP(n, switch_xy),                                                   \
        .flip_x = DT_INST_PROP(n, flip_x),                                                         \
        .flip_y = DT_INST_PROP(n, flip_y),                                                         \
        .report_rate_active_mode = DT_INST_PROP_OR(n, report_rate_active_mode, 10),                \
        .palm_rejection = DT_INST_PROP(n,palm_rejection),                                          \
        .palm_reject_threshold = DT_INST_PROP_OR(n, palm_reject_threshold, 100),                   \
        .palm_reject_timeout = DT_INST_PROP_OR(n, palm_reject_timeout, 1),                         \
        .resolution_x = DT_INST_PROP_OR(n, resolution_x, 3072),                                    \
        .resolution_y = DT_INST_PROP_OR(n, resolution_y, 2048),                                    \
        .bottom_beta = DT_INST_PROP_OR(n, bottom_beta, 5),                                         \
        .stationary_threshold = DT_INST_PROP_OR(n, stationary_threshold, 5),                       \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, iqs5xx_init, NULL, &iqs5xx_data_##n, &iqs5xx_config_##n, POST_KERNEL, \
                          CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS5XX_INIT)