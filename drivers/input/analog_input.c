/*
 * Driver d'entree analogique (joystick).
 *
 * Echantillonne chaque canal ADC a `sampling-hz`, applique point milieu, zone
 * morte, butee et inversion, puis emet la deflexion (en mV) comme evenement
 * d'entree (`evt-type` / `input-code`).
 *
 * Corrections par rapport a la version precedente :
 *  - lecture synchrone, un canal a la fois : plus d'attente infinie sur un
 *    signal ADC asynchrone, et plus d'ambiguite sur l'ordre des canaux (le
 *    SAADC range les resultats par numero de canal croissant, pas dans
 *    l'ordre du devicetree) ;
 *  - le travail d'echantillonnage n'est soumis qu'a sa propre workqueue (il
 *    etait aussi soumis a la workqueue systeme) ;
 *  - zone morte en uint16 (400 mV etait tronque a 144 mV) ;
 *  - en cas d'erreurs ADC repetees, les canaux sont reconfigures.
 *
 * Avec `report-on-change-only`, la valeur n'est emise que lorsqu'elle change,
 * y compris le retour a 0 au centre (utilise par joystick_arrows pour relacher
 * les fleches). Sinon, toute valeur non nulle est emise a chaque echantillon.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_analog_input

#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "analog_input.h"

LOG_MODULE_REGISTER(analog_input, CONFIG_ANALOG_INPUT_LOG_LEVEL);

// Erreurs ADC consecutives avant de reconfigurer les canaux.
#define ANALOG_INPUT_MAX_ADC_ERRORS 5

K_THREAD_STACK_DEFINE(analog_input_q_stack, CONFIG_ANALOG_INPUT_WORKQUEUE_STACK_SIZE);
static struct k_work_q analog_input_work_q;
static bool analog_input_work_q_started;

static int analog_input_setup_channels(const struct device *dev) {
    const struct analog_input_config *config = dev->config;
    int ret = 0;

    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        const struct adc_dt_spec *spec = &config->io_channels[i].adc_channel;

        if (!device_is_ready(spec->dev)) {
            LOG_ERR("ADC non pret pour le canal %u", i);
            return -ENODEV;
        }

        struct adc_channel_cfg channel_cfg = {
            .gain = ADC_GAIN_1_6,
            .reference = ADC_REF_INTERNAL,
            .acquisition_time = ADC_ACQ_TIME_DEFAULT,
            .channel_id = spec->channel_id,
#ifdef CONFIG_ADC_CONFIGURABLE_INPUTS
#ifdef CONFIG_ADC_NRFX_SAADC
            .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + spec->channel_id,
#else
            .input_positive = spec->channel_id,
#endif
#endif
        };

        int err = adc_channel_setup(spec->dev, &channel_cfg);
        if (err < 0) {
            LOG_ERR("Configuration du canal %u impossible : %d", i, err);
            ret = err;
        }
    }

    return ret;
}

// Lit un canal et renvoie sa tension en mV.
static int analog_input_read_mv(const struct device *dev, const struct adc_dt_spec *spec,
                                int32_t *mv) {
    struct analog_input_data *data = dev->data;
    int16_t raw = 0;

    struct adc_sequence seq = {
        .channels = BIT(spec->channel_id),
        .buffer = &raw,
        .buffer_size = sizeof(raw),
        .resolution = 12,
        .oversampling = 0,
        .calibrate = !data->calibrated, // Calibration du SAADC a la premiere lecture
    };

    int err = adc_read(spec->dev, &seq);
    if (err < 0) {
        return err;
    }
    data->calibrated = true;

    *mv = raw;
    return adc_raw_to_millivolts(adc_ref_internal(spec->dev), ADC_GAIN_1_6, seq.resolution, mv);
}

static int32_t analog_input_process(const struct analog_input_io_channel *ch, int32_t mv) {
    int32_t v = mv - ch->mv_mid;
    const int32_t dz = ch->mv_deadzone;

    if (v > 0) {
        v = (v < dz) ? 0 : v - dz;
    } else if (v < 0) {
        v = (v > -dz) ? 0 : v + dz;
    }

    if (ch->mv_min_max) {
        v = CLAMP(v, -(int32_t)ch->mv_min_max, (int32_t)ch->mv_min_max);
    }

    return ch->invert ? -v : v;
}

static void analog_input_sampling_handler(struct k_work *work) {
    struct analog_input_data *data = CONTAINER_OF(work, struct analog_input_data, sampling_work);
    const struct device *dev = data->dev;
    const struct analog_input_config *config = dev->config;

    int32_t values[ANALOG_INPUT_MAX_CHANNELS];
    bool report[ANALOG_INPUT_MAX_CHANNELS];
    int8_t last_report = -1;

    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        const struct analog_input_io_channel *ch = &config->io_channels[i];
        int32_t mv;

        int err = analog_input_read_mv(dev, &ch->adc_channel, &mv);
        if (err < 0) {
            if (++data->adc_errors >= ANALOG_INPUT_MAX_ADC_ERRORS) {
                LOG_WRN("Erreurs ADC repetees (%d), reconfiguration des canaux", err);
                data->adc_errors = 0;
                data->calibrated = false;
                analog_input_setup_channels(dev);
            }
            return;
        }

        values[i] = analog_input_process(ch, mv);

        if (ch->report_on_change_only) {
            report[i] = (values[i] != data->prev[i]);
        } else {
            report[i] = (values[i] != 0);
        }

        if (report[i]) {
            last_report = i;
        }
    }

    data->adc_errors = 0;

    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        if (!report[i]) {
            continue;
        }

        const struct analog_input_io_channel *ch = &config->io_channels[i];
        data->prev[i] = values[i];
        input_report(dev, ch->evt_type, ch->input_code, values[i], i == last_report, K_NO_WAIT);
    }
}

static void analog_input_timer_handler(struct k_timer *timer) {
    struct analog_input_data *data = CONTAINER_OF(timer, struct analog_input_data, sampling_timer);

    k_work_submit_to_queue(&analog_input_work_q, &data->sampling_work);
}

static int analog_input_init(const struct device *dev) {
    struct analog_input_data *data = dev->data;
    const struct analog_input_config *config = dev->config;

    if (config->io_channels_len > ANALOG_INPUT_MAX_CHANNELS) {
        LOG_ERR("Trop de canaux (%u > %u)", config->io_channels_len, ANALOG_INPUT_MAX_CHANNELS);
        return -EINVAL;
    }

    data->dev = dev;

    int err = analog_input_setup_channels(dev);
    if (err < 0) {
        return err;
    }

    if (!analog_input_work_q_started) {
        k_work_queue_start(&analog_input_work_q, analog_input_q_stack,
                           K_THREAD_STACK_SIZEOF(analog_input_q_stack),
                           CONFIG_ANALOG_INPUT_WORKQUEUE_PRIORITY, NULL);
        analog_input_work_q_started = true;
    }

    k_work_init(&data->sampling_work, analog_input_sampling_handler);
    k_timer_init(&data->sampling_timer, analog_input_timer_handler, NULL);

    if (config->sampling_hz > 0) {
        const uint32_t usec = 1000000UL / config->sampling_hz;
        k_timer_start(&data->sampling_timer, K_USEC(usec), K_USEC(usec));
    }

    return 0;
}

#define ANALOG_INPUT_CHANNEL_ENTRY(node_id)                                                        \
    {                                                                                              \
        .adc_channel = ADC_DT_SPEC_GET_BY_IDX(node_id, 0),                                         \
        .mv_mid = DT_PROP(node_id, mv_mid),                                                        \
        .mv_min_max = DT_PROP(node_id, mv_min_max),                                                \
        .mv_deadzone = DT_PROP(node_id, mv_deadzone),                                              \
        .invert = DT_PROP(node_id, invert),                                                        \
        .report_on_change_only = DT_PROP(node_id, report_on_change_only),                          \
        .evt_type = DT_PROP(node_id, evt_type),                                                    \
        .input_code = DT_PROP(node_id, input_code),                                                \
    }

#define ANALOG_INPUT_CHILD_COUNT(node) 1 +

#define ANALOG_INPUT_DEFINE(n)                                                                     \
    static struct analog_input_data analog_input_data_##n;                                         \
    static const struct analog_input_config analog_input_config_##n = {                            \
        .sampling_hz = DT_INST_PROP(n, sampling_hz),                                               \
        .io_channels_len = (DT_INST_FOREACH_CHILD(n, ANALOG_INPUT_CHILD_COUNT) 0),                 \
        .io_channels = {DT_INST_FOREACH_CHILD_SEP(n, ANALOG_INPUT_CHANNEL_ENTRY, (, ))},           \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, analog_input_init, NULL, &analog_input_data_##n,                      \
                          &analog_input_config_##n, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY,       \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(ANALOG_INPUT_DEFINE)
