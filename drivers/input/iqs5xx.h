/*
 * Azoteq IQS5xx (IQS550 / IQS572 / IQS525) - driver Windows Precision Touchpad.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

// ---------------------------------------------------------------------------
// Carte memoire (IQS5xx-B000)
// ---------------------------------------------------------------------------

#define IQS5XX_SYSTEM_INFO_0 0x000F
#define IQS5XX_NUM_FINGERS 0x0011 // Debut du bloc doigts (1 + 4 + 5 * 7 octets)

#define IQS5XX_SYSTEM_CONTROL_0 0x0431
#define IQS5XX_ACK_RESET BIT(7)

#define IQS5XX_SYSTEM_CONTROL_1 0x0432
#define IQS5XX_SUSPEND BIT(0)

// Intervalles de rapport (ms, 16 bits) : 0x057A + 2 * mode
#define IQS5XX_REPORT_RATE_ACTIVE 0x057A
#define IQS5XX_REPORT_RATE_IDLE_TOUCH 0x057C
#define IQS5XX_REPORT_RATE_IDLE 0x057E
#define IQS5XX_REPORT_RATE_LP1 0x0580
#define IQS5XX_REPORT_RATE_LP2 0x0582

// Temps (s, 8 bits) passe en mode Idle avant de descendre en LP1. 255 = jamais.
#define IQS5XX_TIMEOUT_IDLE 0x0586

#define IQS5XX_SYSTEM_CONFIG_0 0x058E
#define IQS5XX_SETUP_COMPLETE BIT(6)
#define IQS5XX_WDT BIT(5)
#define IQS5XX_REATI BIT(2)

#define IQS5XX_SYSTEM_CONFIG_1 0x058F
#define IQS5XX_EVENT_MODE BIT(0)
#define IQS5XX_TP_EVENT BIT(2)
#define IQS5XX_TOUCH_EVENT BIT(6)

#define IQS5XX_FILTER_SETTINGS 0x0632
#define IQS5XX_IIR_FILTER BIT(0)
#define IQS5XX_MAV_FILTER BIT(1)
#define IQS5XX_ALP_COUNT_FILTER BIT(3)

#define IQS5XX_BOTTOM_BETA 0x0637

#define IQS5XX_XY_CONFIG_0 0x0669
#define IQS5XX_FLIP_X BIT(0)
#define IQS5XX_FLIP_Y BIT(1)
#define IQS5XX_SWITCH_XY_AXIS BIT(2)
#define IQS5XX_ALLOW_PALM_REJECT BIT(3)

#define IQS5XX_MAX_MULTI_TOUCHES 0x066A
#define IQS5XX_PALM_REJECT_THRESHOLD 0x066C
#define IQS5XX_PALM_REJECT_TIMEOUT 0x066D
#define IQS5XX_RESOLUTION_X 0x066E
#define IQS5XX_RESOLUTION_Y 0x0670
#define IQS5XX_STATIONARY_THRESH 0x0672

// Gestes materiels : desactives en mode PTP (Windows reconnait les gestes).
#define IQS5XX_SINGLE_FINGER_GESTURES_CONF 0x06B7
#define IQS5XX_MULTI_FINGER_GESTURES_CONF 0x06B8

#define IQS5XX_END_COMM_WINDOW 0xEEEE

#define IQS5XX_MAX_TOUCHES 5

// ---------------------------------------------------------------------------
// Donnees lues
// ---------------------------------------------------------------------------

struct iqs5xx_sys_info {
    uint8_t info_0;
    uint8_t info_1;
} __packed;

#define IQS5XX_INFO0_SHOW_RESET BIT(7)
#define IQS5XX_INFO1_PALM_DETECT BIT(1)

// Un doigt, tel que lu (big endian).
struct iqs5xx_touch {
    uint8_t abs_x[2];
    uint8_t abs_y[2];
    uint8_t strength[2]; // 0 = slot libre
    uint8_t area;
} __packed;

// Bloc lu en une fois a partir de IQS5XX_NUM_FINGERS (40 octets).
struct iqs5xx_frame {
    uint8_t nb_fingers;
    uint8_t rel_x[2];
    uint8_t rel_y[2];
    struct iqs5xx_touch touch[IQS5XX_MAX_TOUCHES];
} __packed;

// ---------------------------------------------------------------------------
// Configuration et etat du driver
// ---------------------------------------------------------------------------

struct iqs5xx_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec rdy_gpio;
    struct gpio_dt_spec reset_gpio;

    bool switch_xy;
    bool flip_x;
    bool flip_y;

    bool palm_rejection;
    uint8_t palm_reject_threshold;
    uint8_t palm_reject_timeout;

    uint8_t max_touch_number;
    uint16_t resolution_x;
    uint16_t resolution_y;
    uint16_t max_x; // Clamp des coordonnees envoyees
    uint16_t max_y;

    uint8_t bottom_beta;
    uint8_t stationary_threshold;

    // Modes basse consommation geres par l'IQS5xx lui-meme.
    uint16_t report_rate_active;
    uint16_t report_rate_idle;
    uint16_t report_rate_lp1;
    uint16_t report_rate_lp2;
    uint8_t idle_timeout;
};

struct iqs5xx_data {
    const struct device *dev;
    struct gpio_callback rdy_cb;

    struct k_work frame_work;              // Lecture d'une trame (front RDY)
    struct k_work_delayable recovery_work; // Reset materiel + reconfiguration
    struct k_work_delayable rdy_watchdog;  // Rattrapage d'un front RDY manque

    bool ready;             // Configure, pret a lire des trames
    bool suspended;         // En veille (PM)
    uint8_t i2c_errors;     // Erreurs I2C consecutives
    uint8_t recovery_tries; // Tentatives de reconfiguration consecutives
    uint8_t rdy_high_polls; // Controles consecutifs avec RDY bloque a 1

    uint8_t prev_active_mask;
    uint16_t last_x[IQS5XX_MAX_TOUCHES];
    uint16_t last_y[IQS5XX_MAX_TOUCHES];
};
