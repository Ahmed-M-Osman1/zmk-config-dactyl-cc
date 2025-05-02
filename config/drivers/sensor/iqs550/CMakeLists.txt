# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_IQS550)
    zephyr_library()
    zephyr_library_sources(iqs550.c)
    zephyr_include_directories(.)
endif()
