# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_INPUT_AZOTEQ_IQS5XX iqs5xx.c)
zephyr_library_sources_ifdef(CONFIG_ANALOG_INPUT analog_input.c)

# Compile dans la cible `app` de ZMK pour avoir acces aux en-tetes zmk/ (evenements, keys.h).
target_sources_ifdef(CONFIG_ZMK_JOYSTICK_ARROWS app PRIVATE joystick_arrows.c)
