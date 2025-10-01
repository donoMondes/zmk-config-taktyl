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

static uint8_t prev_points;
struct iqs5xx_point_data point_data[IQS5XX_INPUT_MAX_TOUCHES];
static struct iqs5xx_point_data prev_point_data[IQS5XX_INPUT_MAX_TOUCHES];

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

// static void iqs5xx_button_release_work_handler(struct k_work *work) {
//     struct k_work_delayable *dwork = k_work_delayable_from_work(work);
//     struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, button_release_work);

//     // TODO: This loop should only deactivate one button.
//     // Log a warning when that is not the case.
//     for (int i = 0; i <IQS5XX_INPUT_MAX_TOUCHES ; i++) {
//         /* We look for the prev_point in the current points list */
//         if(finger == num_fingers)
//         {
//             if(config->max_touch_number>1)
//             {
//                 input_report_abs(dev,INPUT_ABS_MT_SLOT,point_data[prev_finger].id,true,K_FOREVER);
//             }
//             input_report_abs(dev, INPUT_ABS_X, prev_point_data[prev_finger].abs_x, false, K_FOREVER);
//             input_report_abs(dev, INPUT_ABS_Y, prev_point_data[prev_finger].abs_y, false, K_FOREVER);
//             input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
//         }
//     }
// }

static void iqs5xx_work_handler(struct k_work *work) {
    struct iqs5xx_data *data = CONTAINER_OF(work, struct iqs5xx_data, work);
    const struct device *dev = data->dev;
    const struct iqs5xx_config *config = dev->config;
    uint8_t sys_info_0, sys_info_1, gesture_events_0, gesture_events_1, num_fingers;
    int ret;

    uint8_t prev_finger;
	uint8_t finger;

    // Read system info registers.
    ret = iqs5xx_read_reg8(dev, IQS5XX_SYSTEM_INFO_0, &sys_info_0);
    if (ret < 0) {
        LOG_ERR("Failed to read system info 0: %d", ret);
        goto end_comm;
    }

    ret = iqs5xx_read_reg8(dev, IQS5XX_SYSTEM_INFO_1, &sys_info_1);
    if (ret < 0) {
        LOG_ERR("Failed to read system info 1: %d", ret);
        goto end_comm;
    }

    ret = iqs5xx_read_reg8(dev, IQS5XX_GESTURE_EVENTS_0, &gesture_events_0);
    if (ret < 0) {
        LOG_ERR("Failed to read gesture events: %d", ret);
        goto end_comm;
    }

    ret = iqs5xx_read_reg8(dev, IQS5XX_GESTURE_EVENTS_1, &gesture_events_1);
    if (ret < 0) {
        LOG_ERR("Failed to read gesture events 1: %d", ret);
        goto end_comm;
    }

    // Handle reset indication.
    if (sys_info_0 & IQS5XX_SHOW_RESET) {
        LOG_INF("Device reset detected");
        // Acknowledge reset.
        iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONTROL_0, IQS5XX_ACK_RESET);
        goto end_comm;
    }

    bool tp_movement = (sys_info_1 & IQS5XX_TP_MOVEMENT) != 0;

    // Handle movement and gestures.
    if (tp_movement) {
        ret = iqs5xx_read_reg8(dev, IQS5XX_NUM_FINGERS, &num_fingers);
        if (ret < 0) {
            LOG_ERR("Failed to read number of fingers: %d", ret);
            goto end_comm;
        }
        for(uint8_t finger_idx = 0; finger_idx<num_fingers;finger_idx++)
        {
            if(config->max_touch_number>1)
            {
                // current absolute slot for absolute position
                point_data[finger_idx].id = finger_idx;
                input_report_abs(dev,INPUT_ABS_MT_SLOT,point_data[finger_idx].id,false,K_FOREVER);
                if(finger_idx<config->max_touch_number)
                {
                    uint16_t abs_x, abs_y;
                    uint16_t current_reg_abs_x = (uint16_t)(IQS5XX_ABS_X + (finger_idx * IQS5XX_NEXT_TOUCH_OFFSET));
                    uint16_t current_reg_abs_y = (uint16_t)(IQS5XX_ABS_Y + (finger_idx * IQS5XX_NEXT_TOUCH_OFFSET));
                    ret = iqs5xx_read_reg16(dev, current_reg_abs_x, &abs_x);
                    if (ret < 0) {
                        goto end_comm;
                    }
                    ret = iqs5xx_read_reg16(dev, current_reg_abs_y, &abs_y);
                    if (ret < 0) {
                        goto end_comm;
                    }
                    point_data[finger_idx].abs_x = abs_x;
                    point_data[finger_idx].abs_y = abs_y;

                    if(abs_x!=0 || abs_y!=0)
                    {
                        LOG_INF("ABSPOS[%d] , abs_x : %u, abs_y : %u",point_data[finger_idx].id,point_data[finger_idx].abs_x,point_data[finger_idx].abs_y);
                        input_report_abs(dev, INPUT_ABS_X, point_data[finger_idx].abs_x, false, K_FOREVER);
                        input_report_abs(dev, INPUT_ABS_Y, point_data[finger_idx].abs_y, false, K_FOREVER);
                        input_report_key(dev, INPUT_BTN_TOUCH, 1, true, K_FOREVER);
                    }
                }
            }
        }
    }
    
    // for(prev_finger = 0; prev_finger < prev_points ; prev_finger++)
    // {
    //     /* We look for the prev_point in the current points list */
	// 	for (finger = 0; finger < num_fingers; finger++) {
	// 		if (prev_point_data[prev_finger].id == point_data[finger].id) 
    //         {
	// 			break;
	// 		}
    //     }
	// 	if(finger == num_fingers)
    //     {
    //         if(config->max_touch_number>1)
    //         {
    //             input_report_abs(dev,INPUT_ABS_MT_SLOT,point_data[prev_finger].id,true,K_FOREVER);
    //         }
    //         input_report_abs(dev, INPUT_ABS_X, prev_point_data[prev_finger].abs_x, false, K_FOREVER);
    //         input_report_abs(dev, INPUT_ABS_Y, prev_point_data[prev_finger].abs_y, false, K_FOREVER);
    //         input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
    //     }
    // }

    memcpy(prev_point_data,point_data,sizeof(point_data));
    prev_points = num_fingers;

end_comm:
    // End communication window.
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

    // Enable event mode and trackpad events.
    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_1,
                            IQS5XX_EVENT_MODE | IQS5XX_TP_EVENT | IQS5XX_GESTURE_EVENT);
    if (ret < 0) {
        LOG_ERR("Failed to configure event mode: %d", ret);
        return ret;
    }

    // Change report rate value
    ret = iqs5xx_write_reg16(dev, IQS5XX_REPORT_RATE_ACTIVE_MODE,config->report_rate_active_mode);
    if (ret < 0) {
        LOG_ERR("Failed to change report rate: %d", ret);
        return ret;
    }

    // Change Maximum number of touch point
    ret = iqs5xx_write_reg8(dev, IQS5XX_MAX_MULTI_TOUCHES,config->max_touch_number);
    if (ret < 0) {
        LOG_ERR("Failed to configure maximum number of touch point: %d", ret);
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

    // Change resolution

    ret = iqs5xx_write_reg16(dev, IQS5XX_RESOLUTION_X,config->resolution_x);
    if (ret < 0) {
        LOG_ERR("Failed to change x axis resolution: %d", ret);
        return ret;
    }

    ret = iqs5xx_write_reg16(dev, IQS5XX_RESOLUTION_Y,config->resolution_y);
    if (ret < 0) {
        LOG_ERR("Failed to change y axis resolution: %d", ret);
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
    // Configure multi finger gestures.
    ret = iqs5xx_write_reg8(dev, IQS5XX_MULTI_FINGER_GESTURES_CONF, two_finger_gestures);
    if (ret < 0) {
        LOG_ERR("Failed to configure multi finger gestures: %d", ret);
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
    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_0, IQS5XX_SETUP_COMPLETE | IQS5XX_WDT);
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

    data->dev = dev;
    k_work_init(&data->work, iqs5xx_work_handler);
    // k_work_init_delayable(&data->button_release_work, iqs5xx_button_release_work_handler);

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
        .max_touch_number = DT_INST_PROP_OR(n, max_touch_number, 5),                               \
        .resolution_x = DT_INST_PROP_OR(n, resolution_x, 3072),                                    \
        .resolution_y = DT_INST_PROP_OR(n, resolution_y, 2048),                                    \
        .bottom_beta = DT_INST_PROP_OR(n, bottom_beta, 5),                                         \
        .stationary_threshold = DT_INST_PROP_OR(n, stationary_threshold, 5),                       \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, iqs5xx_init, NULL, &iqs5xx_data_##n, &iqs5xx_config_##n, POST_KERNEL, \
                          CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS5XX_INIT)
