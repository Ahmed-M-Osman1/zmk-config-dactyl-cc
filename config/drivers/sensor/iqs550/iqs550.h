/*
 * Copyright (c) 2024 The ZMK Contributors
 * Based on QMK driver code:
 * Copyright 2023 Dasky (@daskygit)
 * Copyright 2023 George Norton (@george-norton)
 * Copyright 2024 Geek-rabb1t (@geek-rabb1t)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_IQS5XX_H_
#define ZEPHYR_DRIVERS_SENSOR_IQS5XX_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

/* IQS5xx I2C Address (7-bit) */
#define IQS5XX_I2C_ADDR_APP           0x74

/* IQS5xx Register Addresses (16-bit) */
#define IQS5XX_REG_PRODUCT_NUMBER       0x0000
#define IQS5XX_REG_PREVIOUS_CYCLE_TIME  0x000C // Start of base data block
#define IQS5XX_REG_GESTURE_EVENTS_0     0x000D
#define IQS5XX_REG_GESTURE_EVENTS_1     0x000E
#define IQS5XX_REG_SYSTEM_INFO_0        0x000F
#define IQS5XX_REG_SYSTEM_INFO_1        0x0010
#define IQS5XX_REG_NUM_FINGERS          0x0011
#define IQS5XX_REG_REL_X                0x0012
#define IQS5XX_REG_REL_Y                0x0014
#define IQS5XX_REG_ABS_X_F1             0x0016
#define IQS5XX_REG_ABS_Y_F1             0x0018
#define IQS5XX_REG_TOUCH_STR_F1         0x001A
#define IQS5XX_REG_TOUCH_AREA_F1        0x001C
// Finger 2-5 data follows similarly

#define IQS5XX_REG_SYSTEM_CONTROL_1     0x0432
#define IQS5XX_REG_REPORT_RATE_ACTIVE   0x057A
#define IQS5XX_REG_REPORT_RATE_IDLE_TCH 0x057C
#define IQS5XX_REG_REPORT_RATE_IDLE     0x057E
#define IQS5XX_REG_REPORT_RATE_LP1      0x0580
#define IQS5XX_REG_REPORT_RATE_LP2      0x0582
#define IQS5XX_REG_SYSTEM_CONFIG_0      0x058E
#define IQS5XX_REG_SYSTEM_CONFIG_1      0x058F
#define IQS5XX_REG_XY_CONFIG_0          0x0669
#define IQS5XX_REG_X_RESOLUTION         0x066E
#define IQS5XX_REG_Y_RESOLUTION         0x0670
#define IQS5XX_REG_SINGLE_FINGER_GESTURES 0x06B7 // Start of gesture config block

#define IQS5XX_REG_END_COMMS            0xEEEE

/* Product Numbers */
#define IQS550_PRODUCT_NUMBER           40
#define IQS525_PRODUCT_NUMBER           52
#define IQS572_PRODUCT_NUMBER           58

/* Charging Modes */
enum iqs5xx_charging_mode {
    IQS5XX_MODE_ACTIVE,
    IQS5XX_MODE_IDLE_TOUCH,
    IQS5XX_MODE_IDLE,
    IQS5XX_MODE_LP1,
    IQS5XX_MODE_LP2,
};

/* Data Structures (Based on QMK driver) */
struct iqs5xx_gesture_events_0 {
    uint8_t swipe_y_neg : 1;
    uint8_t swipe_y_pos : 1;
    uint8_t swipe_x_pos : 1;
    uint8_t swipe_x_neg : 1;
    uint8_t press_and_hold : 1;
    uint8_t single_tap : 1;
    uint8_t _unused : 2;
} __attribute__((packed));

struct iqs5xx_gesture_events_1 {
    uint8_t zoom : 1;
    uint8_t scroll : 1;
    uint8_t two_finger_tap : 1;
    uint8_t _unused : 5;
} __attribute__((packed));

struct iqs5xx_system_info_0 {
    uint8_t show_reset : 1;
    uint8_t alp_reati_occurred : 1;
    uint8_t alp_ati_error : 1;
    uint8_t reati_occurred : 1;
    uint8_t ati_error : 1;
    uint8_t charging_mode : 3;
} __attribute__((packed));

struct iqs5xx_system_info_1 {
    uint8_t switch_state : 1;
    uint8_t snap_toggle : 1;
    uint8_t rr_missed : 1;
    uint8_t too_many_fingers : 1;
    uint8_t palm_detect : 1;
    uint8_t tp_movement : 1;
    uint8_t _unused : 2;
} __attribute__((packed));

struct iqs5xx_finger_data {
    uint16_t absolute_x;
    uint16_t absolute_y;
    uint16_t touch_strength;
    uint8_t touch_area;
    uint8_t _pad0;
} __attribute__((packed));

struct iqs5xx_base_data {
    uint8_t previous_cycle_time;
    struct iqs5xx_gesture_events_0 gesture_events_0;
    struct iqs5xx_gesture_events_1 gesture_events_1;
    struct iqs5xx_system_info_0 system_info_0;
    struct iqs5xx_system_info_1 system_info_1;
    uint8_t number_of_fingers;
    int16_t relative_x;
    int16_t relative_y;
    struct iqs5xx_finger_data finger_1;
    struct iqs5xx_finger_data finger_2;
    struct iqs5xx_finger_data finger_3;
    struct iqs5xx_finger_data finger_4;
    struct iqs5xx_finger_data finger_5;
} __attribute__((packed));

/* System Config 0 */
struct iqs5xx_system_config_0 {
    uint8_t manual_control : 1;
    uint8_t setup_complete : 1;
    uint8_t wdt : 1;
    uint8_t sw_input_event : 1;
    uint8_t alp_reati : 1;
    uint8_t reati : 1;
    uint8_t sw_input_select : 1;
    uint8_t sw_input : 1;
} __attribute__((packed));

/* System Config 1 */
struct iqs5xx_system_config_1 {
    uint8_t prox_event : 1;
    uint8_t touch_event : 1;
    uint8_t snap_event : 1;
    uint8_t alp_prox_event : 1;
    uint8_t reati_event : 1;
    uint8_t tp_event : 1;
    uint8_t gesture_event : 1;
    uint8_t event_mode : 1;
} __attribute__((packed));

/* XY Config 0 */
struct iqs5xx_xy_config_0 {
    uint8_t _unused : 4;
    uint8_t palm_reject : 1;
    uint8_t switch_xy_axis : 1;
    uint8_t flip_y : 1;
    uint8_t flip_x : 1;
} __attribute__((packed));

/* System Control 1 */
struct iqs5xx_system_control_1 {
    uint8_t _unused : 6;
    uint8_t reset : 1;
    uint8_t suspend : 1;
} __attribute__((packed));

/* Device Configuration */
struct iqs5xx_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec irq_gpio;
    struct gpio_dt_spec reset_gpio;
    uint16_t x_resolution;
    uint16_t y_resolution;
    bool flip_x;
    bool flip_y;
    bool switch_xy;
};

/* Device Data */
struct iqs5xx_data {
    const struct device *dev;
    struct iqs5xx_base_data base_data;
    struct k_work work;
    struct gpio_callback gpio_cb;
    uint16_t product_number;
    bool initialized;
    int16_t last_x; // Store last reported absolute X for relative calculation if needed
    int16_t last_y; // Store last reported absolute Y for relative calculation if needed
};

#endif /* ZEPHYR_DRIVERS_SENSOR_IQS5XX_H_ */
