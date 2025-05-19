/*
 * Copyright (c) 2024 Manus AI
 * Adapted from osmakari/zmk (Copyright (c) 2022 osmakari)
 * SPDX-License-Identifier: MIT
 */

#ifndef ZMK_DRIVERS_SENSOR_AZOTEQ_IQS_MANUS_H_
#define ZMK_DRIVERS_SENSOR_AZOTEQ_IQS_MANUS_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

// Product Numbers (from osmakari/zmk, seems to be for IQS5xx series)
#define IQS550_PRODUCT_NUMBER 0x0041 // Example, actual might vary or not be checked strictly
#define IQS572_PRODUCT_NUMBER 0x0040 // Example
#define IQS525_PRODUCT_NUMBER 0x004F // Example

// --- Register Definitions (16-bit addresses, Big Endian) ---
// These are based on the osmakari/zmk driver which seems to be for IQS5xx series
// and may differ from other Azoteq datasheets if they are for different chip families.

// General Info & Control
#define IQS5XX_REG_PRODUCT_NUMBER           0x0000 // Product Number (16-bit)
#define IQS5XX_REG_SW_VERSION               0x0001 // Software Version (16-bit)
#define IQS5XX_REG_HW_VERSION               0x0002 // Hardware Version (16-bit)

#define IQS5XX_REG_SYSTEM_CONTROL_0         0x0010 // System Control 0 (8-bit)
#define IQS5XX_REG_SYSTEM_CONTROL_1         0x0011 // System Control 1 (8-bit)

#define IQS5XX_REG_SYSTEM_STATUS            0x0012 // System Status (8-bit)

// Report Rates
#define IQS5XX_REG_REPORT_RATE_ACTIVE       0x0020 // Report Rate Active Mode (ms) (16-bit)
#define IQS5XX_REG_REPORT_RATE_IDLE_TOUCH   0x0021 // Report Rate Idle Touch Mode (ms) (16-bit)
#define IQS5XX_REG_REPORT_RATE_IDLE         0x0022 // Report Rate Idle Mode (ms) (16-bit)

// Configuration Registers
#define IQS5XX_REG_SYSTEM_CONFIG_0          0x0030 // System Configuration 0 (8-bit)
#define IQS5XX_REG_SYSTEM_CONFIG_1          0x0031 // System Configuration 1 (8-bit)
#define IQS5XX_REG_XY_CONFIG_0              0x0032 // XY Configuration 0 (8-bit)
#define IQS5XX_REG_GESTURE_CONFIG_0         0x0038 // Gesture Configuration 0 (8-bit)
#define IQS5XX_REG_GESTURE_CONFIG_1         0x0039 // Gesture Configuration 1 (8-bit)

// Data Registers (Read Block)
#define IQS5XX_REG_PREVIOUS_CYCLE_TIME      0x0080 // Previous Cycle Time (ms) (16-bit)
#define IQS5XX_REG_SYSTEM_INFO_0            0x0081 // System Info 0 (8-bit)
#define IQS5XX_REG_NUMBER_OF_FINGERS        0x0082 // Number of Fingers (8-bit)
#define IQS5XX_REG_RELATIVE_X               0x0083 // Relative X (16-bit signed)
#define IQS5XX_REG_RELATIVE_Y               0x0085 // Relative Y (16-bit signed)

// Finger 1 Data (example, structure for up to 5 fingers might exist)
#define IQS5XX_REG_FINGER_1_ABS_X           0x0087 // Finger 1 Absolute X (16-bit)
#define IQS5XX_REG_FINGER_1_ABS_Y           0x0089 // Finger 1 Absolute Y (16-bit)
#define IQS5XX_REG_FINGER_1_TOUCH_STRENGTH  0x008B // Finger 1 Touch Strength (16-bit)
#define IQS5XX_REG_FINGER_1_AREA            0x008D // Finger 1 Area (8-bit)

// Gesture Events
#define IQS5XX_REG_GESTURE_EVENTS_0         0x00A0 // Gesture Events 0 (8-bit)
#define IQS5XX_REG_GESTURE_EVENTS_1         0x00A1 // Gesture Events 1 (8-bit)

// Command for ending communication window (write any value)
#define IQS5XX_REG_END_COMMS                0xEEEE

// --- Bitfield Structures for Registers (as per osmakari/zmk driver) ---

// SYSTEM_CONTROL_1 (0x0011)
union iqs5xx_system_control_1 {
    struct {
        uint8_t soft_reset : 1;
        uint8_t ack_reset : 1;
        uint8_t app_crc_recalc : 1;
        uint8_t event_mode_disable : 1; // Note: osmakari driver sets this to disable event mode
        uint8_t : 4; // Reserved
    };
    uint8_t value;
};

// SYSTEM_CONFIG_0 (0x0030)
union iqs5xx_system_config_0 {
    struct {
        uint8_t disable_active_rr : 1;
        uint8_t disable_idle_touch_rr : 1;
        uint8_t disable_idle_rr : 1;
        uint8_t auto_modes : 1;
        uint8_t reati_enable : 1; // ReATI (Automatic Tuning Implementation)
        uint8_t : 3; // Reserved
    };
    uint8_t value;
};

// SYSTEM_CONFIG_1 (0x0031)
union iqs5xx_system_config_1 {
    struct {
        uint8_t sleep_disable : 1;
        uint8_t suspend_disable : 1;
        uint8_t io_wakeup_disable : 1;
        uint8_t gesture_event_disable : 1;
        uint8_t tp_event_disable : 1;      // Touchpad event (relative, abs, strength)
        uint8_t finger_event_disable : 1;  // Individual finger data
        uint8_t event_mode_enable : 1;     // Event mode enable (RDY pin)
        uint8_t : 1; // Reserved
    };
    uint8_t value;
};

// XY_CONFIG_0 (0x0032)
union iqs5xx_xy_config_0 {
    struct {
        uint8_t palm_rejection_disable : 1;
        uint8_t switch_xy_axis : 1;
        uint8_t flip_x_axis : 1;
        uint8_t flip_y_axis : 1;
        uint8_t : 4; // Reserved
    };
    uint8_t value;
};

// GESTURE_EVENTS_0 (0x00A0)
union iqs5xx_gesture_events_0 {
    struct {
        uint8_t single_tap : 1;
        uint8_t tap_and_hold : 1;
        uint8_t swipe_x_neg : 1;
        uint8_t swipe_x_pos : 1;
        uint8_t swipe_y_neg : 1;
        uint8_t swipe_y_pos : 1;
        uint8_t : 2; // Reserved
    };
    uint8_t value;
};

// GESTURE_EVENTS_1 (0x00A1)
union iqs5xx_gesture_events_1 {
    struct {
        uint8_t two_finger_tap : 1;
        uint8_t scroll : 1;
        uint8_t zoom : 1;
        uint8_t : 5; // Reserved
    };
    uint8_t value;
};

// SYSTEM_INFO_0 (0x0081) - from data block
union iqs5xx_system_info_0 {
    struct {
        uint8_t show_reset : 1;
        uint8_t : 1; // Reserved
        uint8_t global_halt : 1;
        uint8_t : 1; // Reserved
        uint8_t gesture_active : 1;
        uint8_t event : 1; // Event occurred (RDY related)
        uint8_t : 2; // Reserved
    };
    uint8_t value;
};

// --- Data Structures for Driver --- 

// Structure for a single finger data (simplified from potential full report)
struct iqs5xx_finger_data {
    uint16_t absolute_x;     // Big Endian
    uint16_t absolute_y;     // Big Endian
    uint16_t touch_strength; // Big Endian
    uint8_t area;
    uint8_t id; // Not directly in this minimal block, but useful for multi-finger
} __attribute__((packed));

// Structure for the base data block read from the device
// (Matches IQS5XX_REG_PREVIOUS_CYCLE_TIME to GESTURE_EVENTS_1 range, approximately)
struct iqs5xx_base_data_block {
    uint16_t previous_cycle_time; // ms, Big Endian
    union iqs5xx_system_info_0 system_info_0;
    uint8_t number_of_fingers;
    int16_t relative_x;          // Big Endian
    int16_t relative_y;          // Big Endian
    struct iqs5xx_finger_data finger_1; // Example for first finger
    // Potentially more finger data here in a full report
    uint8_t reserved_block[24]; // Placeholder for other data up to gestures
    union iqs5xx_gesture_events_0 gesture_events_0;
    union iqs5xx_gesture_events_1 gesture_events_1;
} __attribute__((packed));

// Driver instance data
struct iqs5xx_manus_data {
    const struct device *dev; // Back-pointer to the device
    struct k_work work;       // Work item for processing interrupts
    struct gpio_callback gpio_cb;

    uint16_t product_number;
    bool initialized;

    struct iqs5xx_base_data_block base_data; // Buffer for data read from device

    // Store last absolute coordinates if needed for some logic (e.g. preventing jumps)
    uint16_t last_x;
    uint16_t last_y;
};

// Driver instance configuration
struct iqs5xx_manus_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec irq_gpio;
    struct gpio_dt_spec reset_gpio; // Optional
    bool flip_x;
    bool flip_y;
    bool switch_xy;
    // uint16_t x_resolution; // Not strictly needed for relative reporting
    // uint16_t y_resolution;
};

#endif /* ZMK_DRIVERS_SENSOR_AZOTEQ_IQS_MANUS_H_ */

