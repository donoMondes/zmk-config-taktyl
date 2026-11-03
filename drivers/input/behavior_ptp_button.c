/*
 * &ptp_btn : boutons de souris envoyes dans le rapport Precision Touchpad.
 *
 * En mode PTP, Windows ignore la collection souris du meme appareil : &mkp
 * (qui passe par le rapport souris) ne clique plus. Ce behavior met a jour les
 * boutons du rapport PTP a la place. Le clic se fait a la position du curseur,
 * et un clic maintenu + glissement du doigt sur le pave fait un glisser-deposer.
 *
 * Parametre : LCLK, RCLK ou MCLK (dt-bindings/zmk/mouse.h), comme &mkp.
 *
 * Le behavior emet un evenement d'entree ; le rapport PTP est modifie dans le
 * callback, donc dans le thread d'entree, comme le reste du rapport PTP (le
 * listener du pave). Les deux ne peuvent pas s'ecraser mutuellement.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_ptp_button

#include <drivers/behavior.h>
#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/behavior.h>
#include <zmk/endpoints.h>
#include <zmk/mouse/hid.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define PTP_BUTTONS 3

static int behavior_ptp_button_init(const struct device *dev) { return 0; }

static void ptp_button_report(const struct device *dev, uint32_t mask, bool pressed) {
    for (int i = 0; i < PTP_BUTTONS; i++) {
        if (mask & BIT(i)) {
            WRITE_BIT(mask, i, 0);
            input_report_key(dev, INPUT_BTN_0 + i, pressed ? 1 : 0, (mask & BIT_MASK(PTP_BUTTONS)) == 0,
                             K_FOREVER);
        }
    }
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    ptp_button_report(zmk_behavior_get_binding(binding->behavior_dev), binding->param1, true);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    ptp_button_report(zmk_behavior_get_binding(binding->behavior_dev), binding->param1, false);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_ptp_button_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

// Execute dans le thread d'entree.
static void ptp_button_input_cb(struct input_event *evt) {
    struct zmk_hid_ptp_report *report = zmk_mouse_hid_get_ptp_report();

    if (evt->type != INPUT_EV_KEY) {
        return;
    }

    switch (evt->code) {
    case INPUT_BTN_0:
        report->body.button1 = evt->value ? 1 : 0;
        break;
    case INPUT_BTN_0 + 1:
        report->body.button2 = evt->value ? 1 : 0;
        break;
    case INPUT_BTN_0 + 2:
        report->body.button3 = evt->value ? 1 : 0;
        break;
    default:
        return;
    }

    if (evt->sync) {
        // Scan Time doit avancer meme sans doigt pose (unites de 100 us).
        report->body.scan_time = (uint16_t)((k_uptime_get_32() * 10) & 0xFFFF);
        zmk_endpoints_send_ptp_report();
    }
}

#define PTP_BTN_INST(n)                                                                            \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_ptp_button_init, NULL, NULL, NULL, POST_KERNEL,            \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_ptp_button_driver_api);                                      \
    INPUT_CALLBACK_DEFINE(DEVICE_DT_INST_GET(n), ptp_button_input_cb);

DT_INST_FOREACH_STATUS_OKAY(PTP_BTN_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
