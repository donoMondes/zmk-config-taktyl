# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_INPUT_AZOTEQ_IQS5XX iqs5xx.c)
zephyr_library_sources_ifdef(CONFIG_ANALOG_INPUT analog_input.c)

# Fichiers qui utilisent les en-tetes de ZMK (zmk/...) : compiles dans la cible `app`.
target_sources_ifdef(CONFIG_ZMK_JOYSTICK_ARROWS app PRIVATE joystick_arrows.c)
target_sources_ifdef(CONFIG_ZMK_INPUT_ACTIVITY app PRIVATE input_activity.c)
target_sources_ifdef(CONFIG_ZMK_BEHAVIOR_PTP_BUTTON app PRIVATE behavior_ptp_button.c)
