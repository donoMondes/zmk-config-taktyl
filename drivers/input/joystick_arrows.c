/*
 * Joystick -> touches flechees.
 *
 * Ecoute les evenements d'un device d'entree (driver analogique) et
 * transforme la deflexion des axes X / Y en appuis / relachements des
 * fleches gauche / droite / haut / bas.
 *
 * - Axe X (INPUT_REL_X ou INPUT_ABS_X) : valeur > 0 -> DROITE, < 0 -> GAUCHE
 * - Axe Y (INPUT_REL_Y ou INPUT_ABS_Y) : valeur > 0 -> HAUT,   < 0 -> BAS
 *   (inverser le sens avec la propriete `invert` du canal analogique)
 *
 * Une touche est appuyee quand |valeur| >= threshold et relachee quand
 * |valeur| < release-threshold (hysteresis anti-rebond). Tant que le joystick
 * reste pousse, la touche reste enfoncee : c'est la repetition automatique du
 * systeme d'exploitation qui fait defiler.
 *
 * Les evenements ZMK sont emis depuis la workqueue systeme (comme le keymap),
 * pas depuis le thread d'entree.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_joystick_arrows

#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <dt-bindings/zmk/keys.h>
#include <zmk/events/keycode_state_changed.h>

LOG_MODULE_REGISTER(joystick_arrows, CONFIG_INPUT_LOG_LEVEL);

struct jarrow_msg {
    uint32_t keycode;
    bool pressed;
};

K_MSGQ_DEFINE(jarrow_msgq, sizeof(struct jarrow_msg), 16, 4);

static void jarrow_work_cb(struct k_work *work) {
    struct jarrow_msg msg;

    while (k_msgq_get(&jarrow_msgq, &msg, K_NO_WAIT) == 0) {
        raise_zmk_keycode_state_changed_from_encoded(msg.keycode, msg.pressed, k_uptime_get());
    }
}

static K_WORK_DEFINE(jarrow_work, jarrow_work_cb);

static void jarrow_send(uint32_t keycode, bool pressed) {
    struct jarrow_msg msg = {.keycode = keycode, .pressed = pressed};

    if (k_msgq_put(&jarrow_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Queue pleine, evenement fleche perdu");
        return;
    }
    k_work_submit(&jarrow_work);
}

struct jarrow_config {
    int32_t press_threshold;
    int32_t release_threshold;
};

struct jarrow_axis {
    // -1, 0 ou +1 : direction actuellement appuyee.
    int8_t dir;
};

struct jarrow_data {
    struct jarrow_axis x;
    struct jarrow_axis y;
};

static void jarrow_update_axis(const struct jarrow_config *cfg, struct jarrow_axis *axis,
                               int32_t value, uint32_t pos_key, uint32_t neg_key) {
    int8_t new_dir = axis->dir;

    if (value >= cfg->press_threshold) {
        new_dir = 1;
    } else if (value <= -cfg->press_threshold) {
        new_dir = -1;
    } else if (abs(value) < cfg->release_threshold) {
        new_dir = 0;
    }
    // Entre les deux seuils : on garde l'etat actuel (hysteresis).

    if (new_dir == axis->dir) {
        return;
    }

    if (axis->dir != 0) {
        jarrow_send(axis->dir > 0 ? pos_key : neg_key, false);
    }
    if (new_dir != 0) {
        jarrow_send(new_dir > 0 ? pos_key : neg_key, true);
    }

    axis->dir = new_dir;
}

static void jarrow_handle(const struct jarrow_config *cfg, struct jarrow_data *data,
                          struct input_event *evt) {
    if (evt->type != INPUT_EV_REL && evt->type != INPUT_EV_ABS) {
        return;
    }

    switch (evt->code) {
    case INPUT_REL_X: // == INPUT_ABS_X
        jarrow_update_axis(cfg, &data->x, evt->value, RIGHT_ARROW, LEFT_ARROW);
        break;
    case INPUT_REL_Y: // == INPUT_ABS_Y
        jarrow_update_axis(cfg, &data->y, evt->value, UP_ARROW, DOWN_ARROW);
        break;
    default:
        break;
    }
}

// Signature du callback d'entree de Zephyr 3.5 (branche ZMK de Pete Johanson).
// Sur Zephyr >= 3.7, ajouter un parametre `void *user_data`.
#define JARROW_INST(n)                                                                             \
    static const struct jarrow_config jarrow_config_##n = {                                        \
        .press_threshold = DT_INST_PROP(n, threshold),                                             \
        .release_threshold = DT_INST_PROP(n, release_threshold),                                   \
    };                                                                                             \
    static struct jarrow_data jarrow_data_##n;                                                     \
    static void jarrow_cb_##n(struct input_event *evt) {                                           \
        jarrow_handle(&jarrow_config_##n, &jarrow_data_##n, evt);                                  \
    }                                                                                              \
    INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, device)), jarrow_cb_##n);

DT_INST_FOREACH_STATUS_OKAY(JARROW_INST)
