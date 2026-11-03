/*
 * Azoteq IQS5xx (IQS550 / IQS572 / IQS525) - driver Windows Precision Touchpad.
 *
 * Lit les 5 doigts en position absolue et les envoie au listener ZMK en mode
 * "trackpad" (INPUT_ABS_MT_SLOT / INPUT_ABS_X / INPUT_ABS_Y / INPUT_BTN_TOUCH).
 *
 * Robustesse :
 *  - Chaque acces I2C est retente (l'IQS refuse les transferts en dehors de
 *    sa fenetre de communication).
 *  - Apres plusieurs erreurs I2C consecutives, ou si l'IQS signale qu'il a
 *    redemarre tout seul (SHOW_RESET : il a perdu sa configuration), le
 *    capteur est reinitialise (broche reset) puis reconfigure. En cas d'echec,
 *    nouvel essai avec un delai croissant (100 ms -> 5 s).
 *  - Un chien de garde rattrape un front RDY manque.
 *
 * Energie :
 *  - L'IQS5xx gere seul ses modes basse consommation (Idle -> LP1 -> LP2) et
 *    se reveille au toucher : les intervalles de ces modes sont configurables.
 *  - Quand ZMK passe en veille profonde, le capteur est suspendu (PM device).
 *
 * Tout tourne dans la workqueue systeme : les traitements sont serialises,
 * aucun mutex n'est necessaire.
 *
 * Copyright (c) 2025 Mariano Uvalle
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_iqs5xx

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

#include "iqs5xx.h"

LOG_MODULE_REGISTER(iqs5xx, CONFIG_INPUT_LOG_LEVEL);

#define IQS5XX_I2C_RETRIES 10
#define IQS5XX_I2C_RETRY_DELAY_US 250
// Erreurs I2C consecutives (apres retentatives) avant reinitialisation.
#define IQS5XX_MAX_I2C_ERRORS 3
#define IQS5XX_RESET_DELAY_MS 100
#define IQS5XX_RECOVERY_MAX_DELAY_MS 5000
#define IQS5XX_RDY_WATCHDOG_MS 500

// ---------------------------------------------------------------------------
// Acces I2C
// ---------------------------------------------------------------------------

static int iqs5xx_read(const struct device *dev, uint16_t reg, void *buf, size_t len) {
    const struct iqs5xx_config *config = dev->config;
    const uint8_t addr[2] = {reg >> 8, reg & 0xFF};
    int ret = -EIO;

    for (int i = 0; i < IQS5XX_I2C_RETRIES; i++) {
        ret = i2c_write_read_dt(&config->i2c, addr, sizeof(addr), buf, len);
        if (ret == 0) {
            return 0;
        }
        k_usleep(IQS5XX_I2C_RETRY_DELAY_US);
    }

    return ret;
}

static int iqs5xx_write(const struct device *dev, uint16_t reg, const uint8_t *val, size_t len) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[4] = {reg >> 8, reg & 0xFF};
    int ret = -EIO;

    if (len > 2) {
        return -EINVAL;
    }
    memcpy(&buf[2], val, len);

    for (int i = 0; i < IQS5XX_I2C_RETRIES; i++) {
        ret = i2c_write_dt(&config->i2c, buf, 2 + len);
        if (ret == 0) {
            return 0;
        }
        k_usleep(IQS5XX_I2C_RETRY_DELAY_US);
    }

    return ret;
}

static int iqs5xx_write8(const struct device *dev, uint16_t reg, uint8_t val) {
    return iqs5xx_write(dev, reg, &val, 1);
}

static int iqs5xx_write16(const struct device *dev, uint16_t reg, uint16_t val) {
    const uint8_t be[2] = {val >> 8, val & 0xFF};
    return iqs5xx_write(dev, reg, be, 2);
}

static int iqs5xx_end_comm_window(const struct device *dev) {
    return iqs5xx_write8(dev, IQS5XX_END_COMM_WINDOW, 0x00);
}

static inline uint16_t iqs5xx_be16(const uint8_t b[2]) { return ((uint16_t)b[0] << 8) | b[1]; }

// ---------------------------------------------------------------------------
// Emission des doigts
// ---------------------------------------------------------------------------

/*
 * Envoie les slots de emit_mask. Pour chacun : SLOT, X, Y, BTN_TOUCH. Seul le
 * dernier evenement porte sync=true, pour que ZMK envoie un seul rapport PTP
 * par trame (Contact Count correct, un seul Scan Time).
 */
static void iqs5xx_emit(const struct device *dev, uint8_t active_mask, uint8_t emit_mask) {
    struct iqs5xx_data *data = dev->data;

    if (emit_mask == 0) {
        return;
    }

    const uint8_t last_slot = 31 - __builtin_clz((uint32_t)emit_mask);

    for (uint8_t i = 0; i < IQS5XX_MAX_TOUCHES; i++) {
        if (!(emit_mask & BIT(i))) {
            continue;
        }

        const bool touching = (active_mask & BIT(i)) != 0;

        input_report_abs(dev, INPUT_ABS_MT_SLOT, i, false, K_FOREVER);
        input_report_abs(dev, INPUT_ABS_X, data->last_x[i], false, K_FOREVER);
        input_report_abs(dev, INPUT_ABS_Y, data->last_y[i], false, K_FOREVER);
        input_report_key(dev, INPUT_BTN_TOUCH, touching ? 1 : 0, i == last_slot, K_FOREVER);
    }

    data->prev_active_mask = active_mask;
}

// Leve tous les doigts encore poses (avant un reset, une veille...), sinon
// Windows garderait des doigts "fantomes".
static void iqs5xx_release_all(const struct device *dev) {
    struct iqs5xx_data *data = dev->data;

    iqs5xx_emit(dev, 0, data->prev_active_mask);
}

// ---------------------------------------------------------------------------
// Configuration du capteur
// ---------------------------------------------------------------------------

static int iqs5xx_setup_device(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    int ret = 0;

    // Acquitte l'indicateur de reset.
    ret |= iqs5xx_write8(dev, IQS5XX_SYSTEM_CONTROL_0, IQS5XX_ACK_RESET);

    // Resolution de sortie.
    ret |= iqs5xx_write16(dev, IQS5XX_RESOLUTION_X, config->resolution_x);
    ret |= iqs5xx_write16(dev, IQS5XX_RESOLUTION_Y, config->resolution_y);

    // Intervalles de rapport. Doigt pose immobile (idle touch) = meme vitesse
    // qu'en actif, pour ne pas ajouter de latence au debut d'un mouvement.
    ret |= iqs5xx_write16(dev, IQS5XX_REPORT_RATE_ACTIVE, config->report_rate_active);
    ret |= iqs5xx_write16(dev, IQS5XX_REPORT_RATE_IDLE_TOUCH, config->report_rate_active);
    ret |= iqs5xx_write16(dev, IQS5XX_REPORT_RATE_IDLE, config->report_rate_idle);
    ret |= iqs5xx_write16(dev, IQS5XX_REPORT_RATE_LP1, config->report_rate_lp1);
    ret |= iqs5xx_write16(dev, IQS5XX_REPORT_RATE_LP2, config->report_rate_lp2);
    ret |= iqs5xx_write8(dev, IQS5XX_TIMEOUT_IDLE, config->idle_timeout);

    // Mode evenement : RDY seulement quand des doigts bougent / se posent / se levent.
    ret |= iqs5xx_write8(dev, IQS5XX_SYSTEM_CONFIG_1,
                         IQS5XX_EVENT_MODE | IQS5XX_TP_EVENT | IQS5XX_TOUCH_EVENT);

    // Pas de gestes materiels : Windows interprete lui-meme les doigts.
    ret |= iqs5xx_write8(dev, IQS5XX_SINGLE_FINGER_GESTURES_CONF, 0);
    ret |= iqs5xx_write8(dev, IQS5XX_MULTI_FINGER_GESTURES_CONF, 0);

    ret |= iqs5xx_write8(dev, IQS5XX_MAX_MULTI_TOUCHES, config->max_touch_number);
    ret |= iqs5xx_write8(dev, IQS5XX_PALM_REJECT_THRESHOLD, config->palm_reject_threshold);
    ret |= iqs5xx_write8(dev, IQS5XX_PALM_REJECT_TIMEOUT, config->palm_reject_timeout);
    ret |= iqs5xx_write8(dev, IQS5XX_BOTTOM_BETA, config->bottom_beta);
    ret |= iqs5xx_write8(dev, IQS5XX_STATIONARY_THRESH, config->stationary_threshold);
    ret |= iqs5xx_write8(dev, IQS5XX_FILTER_SETTINGS,
                         IQS5XX_IIR_FILTER | IQS5XX_MAV_FILTER | IQS5XX_ALP_COUNT_FILTER);

    uint8_t xy_config = 0;
    xy_config |= config->flip_x ? IQS5XX_FLIP_X : 0;
    xy_config |= config->flip_y ? IQS5XX_FLIP_Y : 0;
    xy_config |= config->switch_xy ? IQS5XX_SWITCH_XY_AXIS : 0;
    xy_config |= config->palm_rejection ? IQS5XX_ALLOW_PALM_REJECT : 0;
    ret |= iqs5xx_write8(dev, IQS5XX_XY_CONFIG_0, xy_config);

    ret |= iqs5xx_write8(dev, IQS5XX_SYSTEM_CONFIG_0,
                         IQS5XX_SETUP_COMPLETE | IQS5XX_WDT | IQS5XX_REATI);

    ret |= iqs5xx_end_comm_window(dev);

    return ret < 0 ? -EIO : 0;
}

// ---------------------------------------------------------------------------
// Reinitialisation
// ---------------------------------------------------------------------------

// Delai avant la prochaine tentative : immediat la premiere fois, puis
// croissant (200 ms, 400 ms... 5 s) si le capteur retombe en panne aussitot.
static k_timeout_t iqs5xx_backoff(const struct iqs5xx_data *data) {
    if (data->recovery_tries == 0) {
        return K_NO_WAIT;
    }
    return K_MSEC(
        MIN(IQS5XX_RESET_DELAY_MS << data->recovery_tries, IQS5XX_RECOVERY_MAX_DELAY_MS));
}

static void iqs5xx_schedule_recovery(const struct device *dev) {
    struct iqs5xx_data *data = dev->data;

    data->ready = false;
    k_work_reschedule(&data->recovery_work, iqs5xx_backoff(data));
}

/*
 * Reset materiel puis configuration complete. Bloque la workqueue systeme
 * ~100 ms : n'arrive qu'au demarrage et en cas de probleme.
 */
static void iqs5xx_recovery_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, recovery_work);
    const struct device *dev = data->dev;
    const struct iqs5xx_config *config = dev->config;

    if (data->suspended) {
        return;
    }

    data->ready = false;
    iqs5xx_release_all(dev);

    // Compte les tentatives ; remis a zero a la premiere trame lue avec succes.
    data->recovery_tries = MIN(data->recovery_tries + 1, 10);

    if (config->reset_gpio.port) {
        gpio_pin_set_dt(&config->reset_gpio, 1);
        k_msleep(1);
        gpio_pin_set_dt(&config->reset_gpio, 0);
    }
    k_msleep(IQS5XX_RESET_DELAY_MS);

    int ret = iqs5xx_setup_device(dev);
    if (ret == 0) {
        data->ready = true;
        data->i2c_errors = 0;
        LOG_INF("IQS5xx configure");
        return;
    }

    // Echec : nouvel essai avec un delai croissant.
    LOG_WRN("Configuration IQS5xx echouee (%d), tentative %u", ret, data->recovery_tries);
    k_work_reschedule(&data->recovery_work, iqs5xx_backoff(data));
}

static void iqs5xx_i2c_error(const struct device *dev, const char *what, int ret) {
    struct iqs5xx_data *data = dev->data;

    data->i2c_errors++;
    LOG_WRN("Erreur I2C (%s) : %d [%u/%u]", what, ret, data->i2c_errors, IQS5XX_MAX_I2C_ERRORS);

    if (data->i2c_errors >= IQS5XX_MAX_I2C_ERRORS) {
        LOG_ERR("Trop d'erreurs I2C, reinitialisation de l'IQS5xx");
        iqs5xx_schedule_recovery(dev);
    } else {
        iqs5xx_end_comm_window(dev);
    }
}

// ---------------------------------------------------------------------------
// Lecture d'une trame
// ---------------------------------------------------------------------------

static void iqs5xx_frame_handler(struct k_work *work) {
    struct iqs5xx_data *data = CONTAINER_OF(work, struct iqs5xx_data, frame_work);
    const struct device *dev = data->dev;
    const struct iqs5xx_config *config = dev->config;
    struct iqs5xx_sys_info sys_info;
    struct iqs5xx_frame frame;
    int ret;

    if (!data->ready || data->suspended) {
        return;
    }

    ret = iqs5xx_read(dev, IQS5XX_SYSTEM_INFO_0, &sys_info, sizeof(sys_info));
    if (ret < 0) {
        iqs5xx_i2c_error(dev, "sys_info", ret);
        return;
    }

    // L'IQS a redemarre tout seul (WDT, baisse de tension...) : il a perdu sa
    // configuration, on la refait.
    if (sys_info.info_0 & IQS5XX_INFO0_SHOW_RESET) {
        LOG_WRN("Reset de l'IQS5xx detecte, reconfiguration");
        iqs5xx_schedule_recovery(dev);
        return;
    }

    ret = iqs5xx_read(dev, IQS5XX_NUM_FINGERS, &frame, sizeof(frame));
    if (ret < 0) {
        iqs5xx_i2c_error(dev, "trame", ret);
        return;
    }

    data->i2c_errors = 0;
    data->recovery_tries = 0;

    const uint8_t max_slots = MIN(config->max_touch_number, IQS5XX_MAX_TOUCHES);
    uint8_t active_mask = 0;

    // Paume detectee : tous les doigts sont consideres comme leves.
    if (!(sys_info.info_1 & IQS5XX_INFO1_PALM_DETECT)) {
        for (uint8_t i = 0; i < max_slots; i++) {
            const struct iqs5xx_touch *tp = &frame.touch[i];

            if (iqs5xx_be16(tp->strength) == 0) {
                continue;
            }

            data->last_x[i] = MIN(iqs5xx_be16(tp->abs_x), config->max_x);
            data->last_y[i] = MIN(iqs5xx_be16(tp->abs_y), config->max_y);
            active_mask |= BIT(i);
        }
    }

    // Doigts leves depuis la trame precedente : derniere position avec Tip = 0.
    const uint8_t lifted_mask = data->prev_active_mask & ~active_mask;
    iqs5xx_emit(dev, active_mask, active_mask | lifted_mask);

    iqs5xx_end_comm_window(dev);
}

static void iqs5xx_rdy_handler(const struct device *port, struct gpio_callback *cb,
                               gpio_port_pins_t pins) {
    struct iqs5xx_data *data = CONTAINER_OF(cb, struct iqs5xx_data, rdy_cb);

    k_work_submit(&data->frame_work);
}

/*
 * Si RDY reste a 1 sur deux controles consecutifs, un front a ete manque :
 * on lit la trame en attente pour debloquer le capteur.
 */
static void iqs5xx_rdy_watchdog_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, rdy_watchdog);
    const struct iqs5xx_config *config = data->dev->config;

    if (data->suspended) {
        return;
    }

    if (data->ready && gpio_pin_get_dt(&config->rdy_gpio) == 1) {
        if (++data->rdy_high_polls >= 2) {
            LOG_WRN("RDY bloque, lecture forcee");
            data->rdy_high_polls = 0;
            k_work_submit(&data->frame_work);
        }
    } else {
        data->rdy_high_polls = 0;
    }

    k_work_schedule(&data->rdy_watchdog, K_MSEC(IQS5XX_RDY_WATCHDOG_MS));
}

// ---------------------------------------------------------------------------
// Veille (PM device)
// ---------------------------------------------------------------------------

#ifdef CONFIG_PM_DEVICE
static int iqs5xx_pm_action(const struct device *dev, enum pm_device_action action) {
    struct iqs5xx_data *data = dev->data;
    const struct iqs5xx_config *config = dev->config;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        data->suspended = true;
        data->ready = false;
        gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_DISABLE);
        k_work_cancel_delayable(&data->rdy_watchdog);
        k_work_cancel_delayable(&data->recovery_work);
        iqs5xx_release_all(dev);

        // Suspension du capteur (quelques uA). Un echec ne doit pas bloquer la
        // mise en veille du clavier : on le signale seulement.
        if (iqs5xx_write8(dev, IQS5XX_SYSTEM_CONTROL_1, IQS5XX_SUSPEND) < 0) {
            LOG_WRN("Impossible de suspendre l'IQS5xx");
        }
        iqs5xx_end_comm_window(dev);
        return 0;

    case PM_DEVICE_ACTION_RESUME:
        data->suspended = false;
        gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_EDGE_RISING);
        // Le reset materiel sort le capteur de la suspension.
        iqs5xx_schedule_recovery(dev);
        k_work_schedule(&data->rdy_watchdog, K_MSEC(IQS5XX_RDY_WATCHDOG_MS));
        return 0;

    default:
        return -ENOTSUP;
    }
}
#endif // CONFIG_PM_DEVICE

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

static int iqs5xx_init(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    struct iqs5xx_data *data = dev->data;
    int ret;

    if (!i2c_is_ready_dt(&config->i2c)) {
        LOG_ERR("Bus I2C non pret");
        return -ENODEV;
    }

    data->dev = dev;
    k_work_init(&data->frame_work, iqs5xx_frame_handler);
    k_work_init_delayable(&data->recovery_work, iqs5xx_recovery_handler);
    k_work_init_delayable(&data->rdy_watchdog, iqs5xx_rdy_watchdog_handler);

    if (config->reset_gpio.port) {
        if (!gpio_is_ready_dt(&config->reset_gpio)) {
            LOG_ERR("GPIO reset non pret");
            return -ENODEV;
        }
        // GPIO_OUTPUT_INACTIVE : capteur hors reset (le pulse est fait dans recovery).
        ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Configuration GPIO reset impossible : %d", ret);
            return ret;
        }
    }

    if (!gpio_is_ready_dt(&config->rdy_gpio)) {
        LOG_ERR("GPIO RDY non pret");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->rdy_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Configuration GPIO RDY impossible : %d", ret);
        return ret;
    }

    gpio_init_callback(&data->rdy_cb, iqs5xx_rdy_handler, BIT(config->rdy_gpio.pin));
    ret = gpio_add_callback(config->rdy_gpio.port, &data->rdy_cb);
    if (ret < 0) {
        LOG_ERR("Ajout du callback RDY impossible : %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_EDGE_RISING);
    if (ret < 0) {
        LOG_ERR("Configuration interruption RDY impossible : %d", ret);
        return ret;
    }

    // Reset + configuration dans la workqueue systeme, pour ne pas ralentir le boot.
    iqs5xx_schedule_recovery(dev);
    k_work_schedule(&data->rdy_watchdog, K_MSEC(IQS5XX_RDY_WATCHDOG_MS));

    return 0;
}

#ifdef CONFIG_ZMK_TRACKPAD_LOGICAL_X
#define IQS5XX_MAX_X(n) CONFIG_ZMK_TRACKPAD_LOGICAL_X
#define IQS5XX_MAX_Y(n) CONFIG_ZMK_TRACKPAD_LOGICAL_Y
#else
#define IQS5XX_MAX_X(n) DT_INST_PROP(n, resolution_x)
#define IQS5XX_MAX_Y(n) DT_INST_PROP(n, resolution_y)
#endif

#define IQS5XX_INIT(n)                                                                             \
    static struct iqs5xx_data iqs5xx_data_##n;                                                     \
    static const struct iqs5xx_config iqs5xx_config_##n = {                                        \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                            \
        .rdy_gpio = GPIO_DT_SPEC_INST_GET(n, rdy_gpios),                                           \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                               \
        .switch_xy = DT_INST_PROP(n, switch_xy),                                                   \
        .flip_x = DT_INST_PROP(n, flip_x),                                                         \
        .flip_y = DT_INST_PROP(n, flip_y),                                                         \
        .palm_rejection = DT_INST_PROP(n, palm_rejection),                                         \
        .palm_reject_threshold = DT_INST_PROP(n, palm_reject_threshold),                           \
        .palm_reject_timeout = DT_INST_PROP(n, palm_reject_timeout),                               \
        .max_touch_number = DT_INST_PROP(n, max_touch_number),                                     \
        .resolution_x = DT_INST_PROP(n, resolution_x),                                             \
        .resolution_y = DT_INST_PROP(n, resolution_y),                                             \
        .max_x = IQS5XX_MAX_X(n),                                                                  \
        .max_y = IQS5XX_MAX_Y(n),                                                                  \
        .bottom_beta = DT_INST_PROP(n, bottom_beta),                                               \
        .stationary_threshold = DT_INST_PROP(n, stationary_threshold),                             \
        .report_rate_active = DT_INST_PROP(n, report_rate_active_mode),                            \
        .report_rate_idle = DT_INST_PROP(n, report_rate_idle_mode),                                \
        .report_rate_lp1 = DT_INST_PROP(n, report_rate_lp1_mode),                                  \
        .report_rate_lp2 = DT_INST_PROP(n, report_rate_lp2_mode),                                  \
        .idle_timeout = DT_INST_PROP(n, idle_timeout),                                             \
    };                                                                                             \
    PM_DEVICE_DT_INST_DEFINE(n, iqs5xx_pm_action);                                                 \
    DEVICE_DT_INST_DEFINE(n, iqs5xx_init, PM_DEVICE_DT_INST_GET(n), &iqs5xx_data_##n,              \
                          &iqs5xx_config_##n, POST_KERNEL,                                         \
                          CONFIG_INPUT_AZOTEQ_IQS5XX_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS5XX_INIT)
