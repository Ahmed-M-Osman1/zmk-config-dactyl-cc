# Copyright (c) 2024 Manus AI
# SPDX-License-Identifier: Apache-2.0

# Zephyr build system boilerplate
zephyr_library()

# Add the driver source file to the build
zephyr_library_sources(iqs5xx_manus.c)

