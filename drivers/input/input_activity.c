/*
 * Maintient ZMK eveille quand on utilise le pave tactile ou le joystick.
 *
 * Dans cette branche de ZMK, seuls les appuis de touches (position_state_changed)
 * et les encodeurs (sensor_event) comptent comme activite. Sans ce module, le
 * clavier passe en veille profonde apres CONFIG_ZMK_IDLE_SLEEP_TIMEOUT
 * (15 min par defaut) meme si on utilise le pave ou le joystick : ils cessent
 * alors de fonctionner jusqu'au prochain appui de touche.
 *
 * Tout evenement d'entree (n'importe quel device) signale une activite, au
 * plus une fois par seconde, depuis la workqueue systeme.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/input/input.h>
#include <zephyr/kernel.h>

#include <zmk/event_manager.h>

// Defini (non static) dans zmk/app/src/activity.c. L'evenement n'est pas lu.
extern int activity_event_listener(const zmk_event_t *eh);

#define INPUT_ACTIVITY_MIN_INTERVAL_MS 1000

static void input_activity_work_cb(struct k_work *work) { activity_event_listener(NULL); }

static K_WORK_DEFINE(input_activity_work, input_activity_work_cb);

static void input_activity_cb(struct input_event *evt) {
    static int64_t last_ms;
    const int64_t now = k_uptime_get();

    if (now - last_ms < INPUT_ACTIVITY_MIN_INTERVAL_MS) {
        return;
    }
    last_ms = now;

    k_work_submit(&input_activity_work);
}

// dev = NULL : ecoute tous les devices d'entree.
INPUT_CALLBACK_DEFINE(NULL, input_activity_cb);
