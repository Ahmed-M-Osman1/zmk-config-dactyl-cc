/*
 * Copyright (c) 2024 Manus AI
 * Adapted from osmakari/zmk (Copyright (c) 2022 osmakari)
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_iqs_manus // Our new compatible string

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "iqs5xx_manus.h"

// Define a unique log module name for our driver
LOG_MODULE_REGISTER(IQS_MANUS, CONFIG_AZOTEQ_IQS_MANUS_LOG_LEVEL);

// Forward declarations
static int iqs5xx_manus_init(const struct device *dev);
static void iqs5xx_manus_work_handler(struct k_work *work);
static void iqs5xx_manus_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);

// Helper functions for 16-bit register I2C communication (adapted from osmakari)
static int iqs5xx_read_reg16(const struct device *dev, uint16_t reg, uint8_t *buf, uint8_t len) {
    const struct iqs5xx_manus_config *config = dev->config;
    uint8_t reg_buf[2];
    sys_put_be16(reg, reg_buf); // Register address is Big Endian
    LOG_DBG("Attempting I2C Read: Reg=0x%04X, Len=%d", reg, len);
    int ret = i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), buf, len);
    if (ret != 0) {
        LOG_ERR("I2C Read from Reg 0x%04X failed: %d", reg, ret);
    } else {
        LOG_HEXDUMP_DBG(buf, len, "I2C Read Data:");
    }
    return ret;
}

static int iqs5xx_write_reg16(const struct device *dev, uint16_t reg, const uint8_t *buf, uint8_t len) {
    const struct iqs5xx_manus_config *config = dev->config;
    uint8_t write_buf[len + 2];
    sys_put_be16(reg, write_buf); // Register address is Big Endian
    memcpy(&write_buf[2], buf, len);
    LOG_DBG("Attempting I2C Write: Reg=0x%04X, Len=%d", reg, len);
    LOG_HEXDUMP_DBG(buf, len, "I2C Write Data:");
    int ret = i2c_write_dt(&config->i2c, write_buf, sizeof(write_buf));
    if (ret != 0) {
        LOG_ERR("I2C Write to Reg 0x%04X failed: %d", reg, ret);
    }
    return ret;
}

// Helper to end communication window (as per osmakari/IQS5xx datasheets)
static int iqs5xx_end_comms(const struct device *dev) {
    uint8_t end_byte = 0x01; // Value doesn\'t strictly matter, just need to write to the register
    LOG_DBG("Attempting to end I2C comms window (writing to 0x%04X)", IQS5XX_REG_END_COMMS);
    int ret = iqs5xx_write_reg16(dev, IQS5XX_REG_END_COMMS, &end_byte, 1);
    if (ret != 0) {
        LOG_ERR("Failed to end I2C comms window: %d", ret);
    }
    return ret;
}

// Work queue handler - Processes data when RDY pin goes high
static void iqs5xx_manus_work_handler(struct k_work *work) {
    struct iqs5xx_manus_data *data = CONTAINER_OF(work, struct iqs5xx_manus_data, work);
    const struct device *dev = data->dev;
    const struct iqs5xx_manus_config *config = dev->config;
    int ret;

    LOG_DBG("Work handler started for device 	ıqs5xx_manus@%x	", config->i2c.addr);

    // Read the entire base data block
    LOG_DBG("Reading base data block (size %u bytes) from REG 0x%04X", sizeof(data->base_data), IQS5XX_REG_PREVIOUS_CYCLE_TIME);
    ret = iqs5xx_read_reg16(dev, IQS5XX_REG_PREVIOUS_CYCLE_TIME, (uint8_t *)&data->base_data, sizeof(data->base_data));
    if (ret != 0) {
        LOG_ERR("Failed to read base data block: %d. Ending comms.", ret);
        iqs5xx_end_comms(dev); // Attempt to end comms even on error
        return;
    }

    // End communication window *after* successful data read
    ret = iqs5xx_end_comms(dev);
    if (ret != 0) {
        LOG_WRN("Failed to end comms window after data read: %d. Processing data anyway.", ret);
    }

    // Byte swap multi-byte fields from Big Endian (device) to native CPU order
    int16_t rel_x = sys_be16_to_cpu(data->base_data.relative_x);
    int16_t rel_y = sys_be16_to_cpu(data->base_data.relative_y);
    uint16_t abs_x = sys_be16_to_cpu(data->base_data.finger_1.absolute_x);
    uint16_t abs_y = sys_be16_to_cpu(data->base_data.finger_1.absolute_y);
    uint16_t strength = sys_be16_to_cpu(data->base_data.finger_1.touch_strength);

    LOG_DBG("Raw Data - RelX:%d, RelY:%d, AbsX:%u, AbsY:%u, Fingers:%u, Strength:%u, Sys0:0x%02x, Gest0:0x%02x, Gest1:0x%02x",
            rel_x, rel_y, abs_x, abs_y, data->base_data.number_of_fingers,
            strength, data->base_data.system_info_0.value,
            data->base_data.gesture_events_0.value, data->base_data.gesture_events_1.value);

    // Apply transformations based on config
    if (config->switch_xy) {
        int16_t temp_rel = rel_x;
        rel_x = rel_y;
        rel_y = temp_rel;
        LOG_DBG("Applied switch_xy: RelX=%d, RelY=%d", rel_x, rel_y);
    }
    if (config->flip_x) {
        rel_x = -rel_x;
        LOG_DBG("Applied flip_x: RelX=%d", rel_x);
    }
    if (config->flip_y) {
        rel_y = -rel_y;
        LOG_DBG("Applied flip_y: RelY=%d", rel_y);
    }

    // Determine button state (e.g., based on number of fingers or touch strength)
    // Simple example: treat any touch as button 1 (BTN_TOUCH or BTN_LEFT)
    bool button_pressed = (data->base_data.number_of_fingers > 0);

    // Report input events
    if (rel_x != 0) {
        LOG_DBG("Reporting INPUT_REL_X: %d", rel_x);
        input_report_rel(dev, INPUT_REL_X, rel_x, false, K_FOREVER);
    }
    if (rel_y != 0) {
        LOG_DBG("Reporting INPUT_REL_Y: %d", rel_y);
        input_report_rel(dev, INPUT_REL_Y, rel_y, false, K_FOREVER);
    }
    
    // Report BTN_TOUCH for general touch presence, or BTN_LEFT for primary click
    LOG_DBG("Reporting INPUT_BTN_TOUCH (state: %d)", button_pressed ? 1 : 0);
    input_report_key(dev, INPUT_BTN_TOUCH, button_pressed ? 1 : 0, true, K_FOREVER); // Report sync after all events

    // Gesture reporting (example)
    if (data->base_data.gesture_events_0.single_tap) {
        LOG_INF("Gesture: Single Tap Detected!");
        // Example: Simulate a left click for single tap
        input_report_key(dev, INPUT_BTN_LEFT, 1, false, K_FOREVER);
        input_report_key(dev, INPUT_BTN_LEFT, 0, true, K_FOREVER); // Sync after click release
    }
    if (data->base_data.gesture_events_1.two_finger_tap) {
        LOG_INF("Gesture: Two Finger Tap Detected!");
        // Example: Simulate a right click for two-finger tap
        input_report_key(dev, INPUT_BTN_RIGHT, 1, false, K_FOREVER);
        input_report_key(dev, INPUT_BTN_RIGHT, 0, true, K_FOREVER); // Sync after click release
    }
    // TODO: Add more gesture handling (scroll, zoom, swipes) as needed

    data->last_x = abs_x; // Store for potential future use
    data->last_y = abs_y;
    data->initialized = true; // Mark as initialized after first successful data read
    LOG_DBG("Work handler finished.");
}

// GPIO interrupt callback - Triggered by RDY pin
static void iqs5xx_manus_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct iqs5xx_manus_data *data = CONTAINER_OF(cb, struct iqs5xx_manus_data, gpio_cb);
    const struct device *dev = data->dev;
    const struct iqs5xx_manus_config *config = dev->config;

    LOG_DBG("GPIO callback triggered for pins: 0x%08x on port %s", pins, port->name);

    // Check if the specific IRQ pin triggered the callback
    if (!(pins & BIT(config->irq_gpio.pin))) {
        LOG_WRN("GPIO callback for unexpected pins: 0x%08x, expected for pin %d", pins, config->irq_gpio.pin);
        return;
    }

    // Check RDY pin state (should be high if interrupt is GPIO_ACTIVE_HIGH)
    int pin_state = gpio_pin_get_dt(&config->irq_gpio);
    LOG_DBG("RDY pin (Port %s, Pin %d) state: %d", config->irq_gpio.port->name, config->irq_gpio.pin, pin_state);
    if (pin_state > 0) { // If RDY is high (active)
         LOG_DBG("RDY pin is active, submitting work to queue.");
         k_work_submit(&data->work);
    } else {
        LOG_DBG("RDY pin is inactive despite interrupt, ignoring (debounce or edge case).");
    }
}

// Initialization function
static int iqs5xx_manus_init(const struct device *dev) {
    struct iqs5xx_manus_data *data = dev->data;
    const struct iqs5xx_manus_config *config = dev->config;
    int ret;
    uint8_t buf[2]; // General purpose buffer for 2-byte reads/writes

    LOG_INF("Initializing Azoteq IQS-Manus Touchpad (device 	ıqs5xx_manus@%x	)...", config->i2c.addr);
    data->dev = dev;
    data->initialized = false;

    // Check I2C bus readiness
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus 	%s	 is not ready!", config->i2c.bus->name);
        return -ENODEV;
    }
    LOG_DBG("I2C bus 	%s	 is ready.", config->i2c.bus->name);

    // Configure Reset GPIO if available
    if (config->reset_gpio.port) {
        LOG_DBG("Reset GPIO port detected (Port: %s, Pin: %d)", config->reset_gpio.port->name, config->reset_gpio.pin);
        if (!device_is_ready(config->reset_gpio.port)) {
            LOG_ERR("Reset GPIO port 	%s	 not ready!", config->reset_gpio.port->name);
            return -ENODEV;
        }
        LOG_DBG("Configuring reset GPIO pin %d on port %s as output active low.", config->reset_gpio.pin, config->reset_gpio.port->name);
        ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE); // Start inactive (high for active-low reset)
        if (ret) {
            LOG_ERR("Failed to configure reset GPIO: %d", ret);
            return ret;
        }
        // Perform hardware reset sequence
        LOG_INF("Performing hardware reset sequence...");
        gpio_pin_set_dt(&config->reset_gpio, 1); // Ensure high (inactive)
        k_sleep(K_MSEC(5)); // Small delay
        gpio_pin_set_dt(&config->reset_gpio, 0); // Set low (active reset)
        k_sleep(K_MSEC(20)); // Hold reset for a bit (datasheet/example dependent)
        gpio_pin_set_dt(&config->reset_gpio, 1); // Set high (release reset)
        k_sleep(K_MSEC(150)); // Wait for device to boot (osmakari uses ~100ms)
        LOG_INF("Hardware reset sequence completed.");
    } else {
        LOG_INF("No reset GPIO defined. Attempting software reset...");
        union iqs5xx_system_control_1 sys_ctrl1 = { .soft_reset = 1 };
        ret = iqs5xx_write_reg16(dev, IQS5XX_REG_SYSTEM_CONTROL_1, &sys_ctrl1.value, 1);
        if (ret != 0) {
            LOG_WRN("Failed to send software reset command: %d. Continuing...", ret);
        } else {
            LOG_INF("Software reset command sent.");
        }
        iqs5xx_end_comms(dev); // End comms after reset attempt
        k_sleep(K_MSEC(150)); // Wait for device to boot
    }

    // Attempt to wake device by reading product number (common practice)
    LOG_DBG("Attempting to wake device by reading product number (REG 0x%04X)...", IQS5XX_REG_PRODUCT_NUMBER);
    ret = iqs5xx_read_reg16(dev, IQS5XX_REG_PRODUCT_NUMBER, buf, sizeof(buf));
    if (ret != 0) {
        LOG_ERR("Failed to communicate with device after reset/wake attempt (Read Product Number): %d. Comms window might not be open.", ret);
        // Don\'t end comms here as it might not be ready or window already closed by error
        return -EIO;
    }
    iqs5xx_end_comms(dev); // End comms after successful read
    k_sleep(K_MSEC(10));   // Small delay after wake/first read
    LOG_DBG("Device wake-up read complete.");

    // Read Product Number again to confirm
    LOG_DBG("Reading product number again (REG 0x%04X)...", IQS5XX_REG_PRODUCT_NUMBER);
    ret = iqs5xx_read_reg16(dev, IQS5XX_REG_PRODUCT_NUMBER, buf, sizeof(buf));
    if (ret != 0) {
        LOG_ERR("Failed to read product number: %d", ret);
        iqs5xx_end_comms(dev);
        return -EIO;
    }
    iqs5xx_end_comms(dev);
    data->product_number = sys_be16_to_cpu(*(uint16_t *)buf);
    LOG_INF("Device Product Number: 0x%04X (%u)", data->product_number, data->product_number);

    // Basic check against known product numbers (optional, for info)
    if (data->product_number != IQS550_PRODUCT_NUMBER && 
        data->product_number != IQS572_PRODUCT_NUMBER && 
        data->product_number != IQS525_PRODUCT_NUMBER) {
        LOG_WRN("Product number 0x%04X does not match known IQS5xx values. Driver may still work if compatible.", data->product_number);
    }

    // --- Configuration Sequence (adapted from osmakari/zmk driver) ---
    LOG_INF("Starting touchpad configuration sequence...");

    // 1. Set Report Rate (Active Mode)
    uint16_t report_rate_ms = 10; // 10ms = 100Hz (common for touchpads)
    LOG_DBG("Setting active report rate (REG 0x%04X) to %u ms", IQS5XX_REG_REPORT_RATE_ACTIVE, report_rate_ms);
    sys_put_be16(report_rate_ms, buf); // Convert to Big Endian for device
    ret = iqs5xx_write_reg16(dev, IQS5XX_REG_REPORT_RATE_ACTIVE, buf, sizeof(buf));
    if (ret != 0) LOG_WRN("Failed to set active report rate: %d", ret);
    iqs5xx_end_comms(dev);

    // 2. Configure Event Mode (SYSTEM_CONFIG_1 - REG 0x0031)
    // We need event_mode_enable = 1 for RDY pin to work.
    // osmakari driver disables event_mode in SYSTEM_CONTROL_1, then enables specific events here.
    LOG_DBG("Configuring event mode (SYSTEM_CONFIG_1 - REG 0x%04X)...", IQS5XX_REG_SYSTEM_CONFIG_1);
    union iqs5xx_system_config_1 sys_cfg1;
    ret = iqs5xx_read_reg16(dev, IQS5XX_REG_SYSTEM_CONFIG_1, &sys_cfg1.value, 1);
    if (ret == 0) {
        LOG_DBG("Read System Config 1: 0x%02x", sys_cfg1.value);
        sys_cfg1.event_mode_enable = 1;     // Crucial: Enable event mode for RDY pin
        sys_cfg1.gesture_event_disable = 0; // Enable gesture events
        sys_cfg1.tp_event_disable = 0;      // Enable TP events (relative, abs, strength)
        sys_cfg1.finger_event_disable = 0;  // Enable Individual finger data (if supported/parsed)
        LOG_DBG("Writing System Config 1 with event_mode_enable=1: 0x%02x", sys_cfg1.value);
        ret = iqs5xx_write_reg16(dev, IQS5XX_REG_SYSTEM_CONFIG_1, &sys_cfg1.value, 1);
        if (ret != 0) LOG_WRN("Failed to write System Config 1 for event mode: %d", ret);
    } else {
        LOG_WRN("Failed to read System Config 1: %d. Event mode might not be set correctly.", ret);
    }
    iqs5xx_end_comms(dev);

    // 3. Set ReATI (Automatic Tuning Implementation) (SYSTEM_CONFIG_0 - REG 0x0030)
    LOG_DBG("Configuring ReATI (SYSTEM_CONFIG_0 - REG 0x%04X)...", IQS5XX_REG_SYSTEM_CONFIG_0);
    union iqs5xx_system_config_0 sys_cfg0;
    ret = iqs5xx_read_reg16(dev, IQS5XX_REG_SYSTEM_CONFIG_0, &sys_cfg0.value, 1);
    if (ret == 0) {
        LOG_DBG("Read System Config 0: 0x%02x", sys_cfg0.value);
        sys_cfg0.reati_enable = 1; // Enable ReATI
        LOG_DBG("Writing System Config 0 with reati_enable=1: 0x%02x", sys_cfg0.value);
        ret = iqs5xx_write_reg16(dev, IQS5XX_REG_SYSTEM_CONFIG_0, &sys_cfg0.value, 1);
        if (ret != 0) LOG_WRN("Failed to write System Config 0 for ReATI: %d", ret);
    } else {
        LOG_WRN("Failed to read System Config 0: %d. ReATI might not be set correctly.", ret);
    }
    iqs5xx_end_comms(dev);

    // 4. Set XY Config (Orientation, Palm Rejection) (XY_CONFIG_0 - REG 0x0032)
    LOG_DBG("Configuring XY settings (XY_CONFIG_0 - REG 0x%04X)...", IQS5XX_REG_XY_CONFIG_0);
    union iqs5xx_xy_config_0 xy_cfg0;
    ret = iqs5xx_read_reg16(dev, IQS5XX_REG_XY_CONFIG_0, &xy_cfg0.value, 1);
    if (ret == 0) {
        LOG_DBG("Read XY Config 0: 0x%02x", xy_cfg0.value);
        xy_cfg0.palm_rejection_disable = 0; // Enable palm rejection (0 = enabled)
        xy_cfg0.flip_x_axis = config->flip_x ? 1 : 0;
        xy_cfg0.flip_y_axis = config->flip_y ? 1 : 0;
        xy_cfg0.switch_xy_axis = config->switch_xy ? 1 : 0;
        LOG_DBG("Writing XY Config 0: 0x%02x (flip_x:%d, flip_y:%d, switch_xy:%d)", 
                  xy_cfg0.value, config->flip_x, config->flip_y, config->switch_xy);
        ret = iqs5xx_write_reg16(dev, IQS5XX_REG_XY_CONFIG_0, &xy_cfg0.value, 1);
        if (ret != 0) LOG_WRN("Failed to write XY Config 0: %d", ret);
    } else {
        LOG_WRN("Failed to read XY Config 0: %d. XY settings might not be applied.", ret);
    }
    iqs5xx_end_comms(dev);

    // 5. Gesture Configuration (Optional, can be expanded)
    // For now, we rely on the event_mode_enable and gesture_event_disable=0 in SYSTEM_CONFIG_1
    // to enable basic gesture event reporting if the chip supports it by default.
    LOG_INF("Gesture configuration: Relying on System Config 1 settings for gesture events.");

    // --- End Configuration --- 
    LOG_INF("Touchpad configuration sequence complete.");

    // Configure IRQ GPIO
    LOG_DBG("Configuring IRQ GPIO (Port: %s, Pin: %d, Flags: 0x%x)", 
              config->irq_gpio.port->name, config->irq_gpio.pin, config->irq_gpio.dt_flags);
    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO port 	%s	 not ready!", config->irq_gpio.port->name);
        return -ENODEV;
    }
    // Configure as input, no pull-up/down initially (assuming external or sensor handles it)
    ret = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (ret) {
        LOG_ERR("Failed to configure IRQ GPIO as input: %d", ret);
        return ret;
    }
    gpio_init_callback(&data->gpio_cb, iqs5xx_manus_gpio_callback, BIT(config->irq_gpio.pin));
    LOG_DBG("Adding GPIO callback for IRQ pin...");
    ret = gpio_add_callback(config->irq_gpio.port, &data->gpio_cb);
    if (ret) {
        LOG_ERR("Failed to add GPIO callback: %d", ret);
        return ret; // Cannot proceed without interrupt handling
    }
    // Configure interrupt (e.g., edge rising for active high RDY)
    LOG_DBG("Configuring GPIO interrupt for IRQ pin (edge to active)...");
    ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) {
        LOG_ERR("Failed to configure GPIO interrupt: %d", ret);
        gpio_remove_callback(config->irq_gpio.port, &data->gpio_cb); // Clean up callback
        return ret;
    }
    LOG_INF("IRQ GPIO (RDY pin) configured successfully.");

    // Initialize work queue item
    k_work_init(&data->work, iqs5xx_manus_work_handler);
    LOG_DBG("Work queue initialized.");

    LOG_INF("Azoteq IQS-Manus Touchpad initialized successfully for device 	ıqs5xx_manus@%x	", config->i2c.addr);
    return 0;
}

// Instantiate the driver for each compatible node in the device tree
#define IQS5XX_MANUS_INIT(inst) \
    static struct iqs5xx_manus_data iqs5xx_manus_data_##inst; \
    static const struct iqs5xx_manus_config iqs5xx_manus_config_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst), \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios), \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, reset_gpios), \
                    (.reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),), (.reset_gpio = {0},)) \
        .flip_x = DT_INST_PROP(inst, flip_x), \
        .flip_y = DT_INST_PROP(inst, flip_y), \
        .switch_xy = DT_INST_PROP(inst, switch_xy), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, &iqs5xx_manus_init, NULL, \
                          &iqs5xx_manus_data_##inst, &iqs5xx_manus_config_##inst, \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
                          NULL); // No specific API structure needed for input reporting

DT_INST_FOREACH_STATUS_OKAY(IQS5XX_MANUS_INIT)


