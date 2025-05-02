/*
 * Copyright (c) 2024 The ZMK Contributors
 * Based on QMK driver code:
 * Copyright 2023 Dasky (@daskygit)
 * Copyright 2023 George Norton (@george-norton)
 * Copyright 2024 Geek-rabb1t (@geek-rabb1t)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #define DT_DRV_COMPAT azoteq_iqs550

 #include <zephyr/device.h>
 #include <zephyr/drivers/i2c.h>
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/input/input.h>
 #include <zephyr/kernel.h>
 #include <zephyr/logging/log.h>
 #include <zephyr/sys/byteorder.h>
 #include <zephyr/sys/util.h>
 
 #include "iqs550.h" // Renamed to iqs550.h, but content is for iqs5xx
 
 #define LOG_LEVEL CONFIG_IQS550_LOG_LEVEL
 #define LOG_MODULE_NAME iqs550
 LOG_MODULE_REGISTER(LOG_MODULE_NAME);
 
 // Forward declarations
 static int iqs5xx_init(const struct device *dev);
 static void iqs5xx_work_handler(struct k_work *work);
 static void iqs5xx_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
 
 // Helper functions for 16-bit register I2C communication
 static int iqs5xx_read_reg16(const struct device *dev, uint16_t reg, uint8_t *buf, uint8_t len) {
     const struct iqs5xx_config *config = dev->config;
     uint8_t reg_buf[2];
     sys_put_be16(reg, reg_buf);
     int ret = i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), buf, len);
     if (ret != 0) {
         LOG_ERR("I2C Read failed: reg=0x%04X, ret=%d", reg, ret);
     }
     return ret;
 }
 
 static int iqs5xx_write_reg16(const struct device *dev, uint16_t reg, uint8_t *buf, uint8_t len) {
     const struct iqs5xx_config *config = dev->config;
     uint8_t write_buf[len + 2];
     sys_put_be16(reg, write_buf);
     memcpy(&write_buf[2], buf, len);
     int ret = i2c_write_dt(&config->i2c, write_buf, sizeof(write_buf));
     if (ret != 0) {
         LOG_ERR("I2C Write failed: reg=0x%04X, ret=%d", reg, ret);
     }
     return ret;
 }
 
 // Helper to end communication window
 static int iqs5xx_end_comms(const struct device *dev) {
     uint8_t end_byte = 0x01; // Value doesn't matter
     LOG_DBG("I2C End Comms Window");
     int ret = iqs5xx_write_reg16(dev, IQS5XX_REG_END_COMMS, &end_byte, 1);
     if (ret != 0) {
         LOG_ERR("I2C End Comms failed: %d", ret);
     }
     return ret;
 }
 
 // Work queue handler - Processes data when RDY pin goes high
 static void iqs5xx_work_handler(struct k_work *work) {
     struct iqs5xx_data *data = CONTAINER_OF(work, struct iqs5xx_data, work);
     const struct device *dev = data->dev;
     const struct iqs5xx_config *config = dev->config;
     int ret;
 
     LOG_DBG("Work handler started");
 
     // Read the entire base data block
     ret = iqs5xx_read_reg16(dev, IQS5XX_REG_PREVIOUS_CYCLE_TIME, (uint8_t *)&data->base_data, sizeof(data->base_data));
     if (ret != 0) {
         LOG_ERR("Failed to read base data block: %d", ret);
         // Attempt to end comms even on error, might help reset state
         iqs5xx_end_comms(dev);
         return;
     }
 
     // End communication window
     ret = iqs5xx_end_comms(dev);
     if (ret != 0) {
         LOG_WRN("Failed to end comms window: %d", ret);
         // Continue processing data anyway
     }
 
     // Byte swap multi-byte fields from Big Endian (device) to native
     int16_t rel_x = sys_be16_to_cpu(data->base_data.relative_x);
     int16_t rel_y = sys_be16_to_cpu(data->base_data.relative_y);
     uint16_t abs_x = sys_be16_to_cpu(data->base_data.finger_1.absolute_x);
     uint16_t abs_y = sys_be16_to_cpu(data->base_data.finger_1.absolute_y);
     uint16_t strength = sys_be16_to_cpu(data->base_data.finger_1.touch_strength);
 
     LOG_DBG("Processed Data - Rel: X=%d, Y=%d | Abs: X=%u, Y=%u | Fingers: %u | Strength: %u | Sys0: 0x%02x | Gest0: 0x%02x | Gest1: 0x%02x",
             rel_x, rel_y, abs_x, abs_y, data->base_data.number_of_fingers,
             strength, *(uint8_t *)&data->base_data.system_info_0,
             *(uint8_t *)&data->base_data.gesture_events_0, *(uint8_t *)&data->base_data.gesture_events_1);
 
     // Apply transformations based on config
     if (config->switch_xy) {
         int16_t temp = rel_x;
         rel_x = rel_y;
         rel_y = temp;
     }
     if (config->flip_x) {
         rel_x = -rel_x;
     }
     if (config->flip_y) {
         rel_y = -rel_y;
     }
 
     // Determine button state (e.g., based on number of fingers or touch strength)
     // Simple example: treat any touch as button 1 press
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
     LOG_DBG("Reporting INPUT_BTN_TOUCH: %d", button_pressed ? 1 : 0);
     input_report_key(dev, INPUT_BTN_TOUCH, button_pressed ? 1 : 0, true, K_FOREVER); // Report sync after all events
 
     // TODO: Implement gesture reporting (taps, swipes, scroll) based on gesture_events_0/1
     // Example: Check for single tap
     if (data->base_data.gesture_events_0.single_tap) {
         LOG_DBG("Single Tap Detected");
         // Send BTN_LEFT press and release?
         // input_report_key(dev, INPUT_BTN_LEFT, 1, false, K_FOREVER);
         // input_report_key(dev, INPUT_BTN_LEFT, 0, true, K_FOREVER);
     }
     // Example: Check for scroll
     if (data->base_data.gesture_events_1.scroll) {
         LOG_DBG("Scroll Detected - Use relative X/Y for scroll values?");
         // Need logic to differentiate scroll movement from cursor movement
         // input_report_rel(dev, INPUT_REL_WHEEL, scroll_value_y, false, K_FOREVER);
         // input_report_rel(dev, INPUT_REL_HWHEEL, scroll_value_x, true, K_FOREVER);
     }
 
     data->last_x = abs_x;
     data->last_y = abs_y;
     data->initialized = true;
     LOG_DBG("Work handler finished");
 }
 
 // GPIO interrupt callback - Triggered by RDY pin
 static void iqs5xx_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
     struct iqs5xx_data *data = CONTAINER_OF(cb, struct iqs5xx_data, gpio_cb);
 
     LOG_DBG("GPIO callback triggered for pins: 0x%08x", pins);
 
     // Check if RDY pin is actually high before submitting work
     // This helps debounce or handle spurious interrupts
     const struct iqs5xx_config *config = data->dev->config;
     int pin_state = gpio_pin_get_dt(&config->irq_gpio);
     LOG_DBG("RDY pin state: %d", pin_state);
     if (pin_state > 0) {
          LOG_DBG("Submitting work to queue");
          k_work_submit(&data->work);
     } else {
         LOG_DBG("IRQ triggered but RDY pin low, ignoring.");
     }
 }
 
 // Initialization function
 static int iqs5xx_init(const struct device *dev) {
     struct iqs5xx_data *data = dev->data;
     const struct iqs5xx_config *config = dev->config;
     int ret;
     uint8_t buf[2];
 
     LOG_INF("Initializing IQS5XX Touchpad...");
     data->dev = dev;
     data->initialized = false;
 
     // Check I2C readiness
     if (!device_is_ready(config->i2c.bus)) {
         LOG_ERR("I2C bus %s not ready", config->i2c.bus->name);
         return -ENODEV;
     }
     LOG_DBG("I2C bus %s is ready", config->i2c.bus->name);
 
     // Configure Reset GPIO if available
     if (config->reset_gpio.port) {
         LOG_DBG("Reset GPIO port detected");
         if (!device_is_ready(config->reset_gpio.port)) {
             LOG_ERR("Reset GPIO port not ready");
             return -ENODEV;
         }
         LOG_DBG("Configuring reset GPIO pin %d on port %s", config->reset_gpio.pin, config->reset_gpio.port->name);
         ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE); // Active low reset
         if (ret) {
             LOG_ERR("Failed to configure reset GPIO: %d", ret);
             return ret;
         }
         // Perform hardware reset sequence from QMK
         LOG_DBG("Performing hardware reset sequence...");
         gpio_pin_set_dt(&config->reset_gpio, 1); // Set high (inactive)
         k_sleep(K_MSEC(1));
         gpio_pin_set_dt(&config->reset_gpio, 0); // Set low (active)
         k_sleep(K_MSEC(10)); // QMK uses wait_ms(10)
         gpio_pin_set_dt(&config->reset_gpio, 1); // Set high (inactive)
         k_sleep(K_MSEC(100)); // Wait for device to boot
         LOG_INF("Hardware reset performed.");
     } else {
         LOG_DBG("No reset GPIO detected, attempting software reset...");
         // If no reset pin, try software reset?
         // QMK driver does a software reset via SYSTEM_CONTROL_1
         struct iqs5xx_system_control_1 sys_ctrl = { .reset = 1 };
         ret = iqs5xx_write_reg16(dev, IQS5XX_REG_SYSTEM_CONTROL_1, (uint8_t *)&sys_ctrl, sizeof(sys_ctrl));
         if (ret != 0) {
             LOG_WRN("Failed to send software reset command: %d", ret);
         } else {
             LOG_INF("Software reset command sent.");
         }
         iqs5xx_end_comms(dev); // End comms after reset attempt
         k_sleep(K_MSEC(100)); // Wait for device to boot
     }
 
     // Wake device (needed after reset)
     LOG_DBG("Attempting to wake device by reading product number...");
     ret = iqs5xx_read_reg16(dev, IQS5XX_REG_PRODUCT_NUMBER, buf, sizeof(buf));
     if (ret != 0) {
         LOG_ERR("Failed to communicate with device after reset (Read Product Number): %d", ret);
         // Don't end comms here as it might not be ready
         return -EIO;
     }
     iqs5xx_end_comms(dev); // End comms after successful read
     k_sleep(K_MSEC(5)); // Small delay after wake
     LOG_DBG("Device wake attempt complete.");
 
     // Read Product Number
     LOG_DBG("Reading product number again...");
     ret = iqs5xx_read_reg16(dev, IQS5XX_REG_PRODUCT_NUMBER, buf, sizeof(buf));
     if (ret != 0) {
         LOG_ERR("Failed to read product number: %d", ret);
         iqs5xx_end_comms(dev);
         return -EIO;
     }
     iqs5xx_end_comms(dev);
     data->product_number = sys_be16_to_cpu(*(uint16_t *)buf);
     LOG_INF("Product Number: %u", data->product_number);
 
     if (data->product_number != IQS550_PRODUCT_NUMBER && data->product_number != IQS572_PRODUCT_NUMBER && data->product_number != IQS525_PRODUCT_NUMBER) {
         LOG_WRN("Unknown product number %u", data->product_number);
         // Continue anyway, might be compatible
     }
 
     // --- Configuration Sequence (based on QMK driver) ---
     LOG_DBG("Starting configuration sequence...");
 
     // Set Report Rate (Active Mode)
     uint16_t report_rate_ms = 10; // 100Hz
     LOG_DBG("Setting active report rate to %u ms", report_rate_ms);
     sys_put_be16(report_rate_ms, buf);
     ret = iqs5xx_write_reg16(dev, IQS5XX_REG_REPORT_RATE_ACTIVE, buf, sizeof(buf));
     if (ret != 0) LOG_WRN("Failed to set active report rate: %d", ret);
     iqs5xx_end_comms(dev);
 
     // Set Event Mode (QMK disables this, but we need it for RDY pin)
     LOG_DBG("Configuring event mode...");
     struct iqs5xx_system_config_1 sys_cfg1;
     ret = iqs5xx_read_reg16(dev, IQS5XX_REG_SYSTEM_CONFIG_1, (uint8_t *)&sys_cfg1, sizeof(sys_cfg1));
     if (ret == 0) {
         LOG_DBG("Read System Config 1: 0x%02x", *(uint8_t *)&sys_cfg1);
         sys_cfg1.event_mode = 1;    // Enable event mode
         sys_cfg1.gesture_event = 1; // Enable gesture events
         sys_cfg1.tp_event = 1;      // Enable TP events
         sys_cfg1.touch_event = 1;   // Enable touch events (redundant?)
         LOG_DBG("Writing System Config 1: 0x%02x", *(uint8_t *)&sys_cfg1);
         ret = iqs5xx_write_reg16(dev, IQS5XX_REG_SYSTEM_CONFIG_1, (uint8_t *)&sys_cfg1, sizeof(sys_cfg1));
         if (ret != 0) LOG_WRN("Failed to enable event mode: %d", ret);
     } else {
         LOG_WRN("Failed to read System Config 1: %d", ret);
     }
     iqs5xx_end_comms(dev);
 
     // Set ReATI (Automatic Tuning Implementation)
     LOG_DBG("Configuring ReATI...");
     struct iqs5xx_system_config_0 sys_cfg0;
     ret = iqs5xx_read_reg16(dev, IQS5XX_REG_SYSTEM_CONFIG_0, (uint8_t *)&sys_cfg0, sizeof(sys_cfg0));
     if (ret == 0) {
         LOG_DBG("Read System Config 0: 0x%02x", *(uint8_t *)&sys_cfg0);
         sys_cfg0.reati = 1; // Enable ReATI
         LOG_DBG("Writing System Config 0: 0x%02x", *(uint8_t *)&sys_cfg0);
         ret = iqs5xx_write_reg16(dev, IQS5XX_REG_SYSTEM_CONFIG_0, (uint8_t *)&sys_cfg0, sizeof(sys_cfg0));
         if (ret != 0) LOG_WRN("Failed to enable ReATI: %d", ret);
     } else {
         LOG_WRN("Failed to read System Config 0: %d", ret);
     }
     iqs5xx_end_comms(dev);
 
     // Set XY Config (Orientation, Palm Rejection)
     LOG_DBG("Configuring XY settings...");
     struct iqs5xx_xy_config_0 xy_cfg0;
     ret = iqs5xx_read_reg16(dev, IQS5XX_REG_XY_CONFIG_0, (uint8_t *)&xy_cfg0, sizeof(xy_cfg0));
     if (ret == 0) {
         LOG_DBG("Read XY Config 0: 0x%02x", *(uint8_t *)&xy_cfg0);
         xy_cfg0.palm_reject = 1; // Enable palm rejection (QMK default)
         xy_cfg0.flip_x = config->flip_x;
         xy_cfg0.flip_y = config->flip_y;
         xy_cfg0.switch_xy_axis = config->switch_xy;
         LOG_DBG("Writing XY Config 0: 0x%02x", *(uint8_t *)&xy_cfg0);
         ret = iqs5xx_write_reg16(dev, IQS5XX_REG_XY_CONFIG_0, (uint8_t *)&xy_cfg0, sizeof(xy_cfg0));
         if (ret != 0) LOG_WRN("Failed to set XY config: %d", ret);
     } else {
         LOG_WRN("Failed to read XY Config 0: %d", ret);
     }
     iqs5xx_end_comms(dev);
 
     // Set Gesture Config (Enable specific gestures)
     // TODO: Read current config, modify based on Kconfig, write back
     // QMK enables tap, 2-finger tap, scroll by default
     // For now, skip writing gesture config, use device defaults or hope QMK settings persist
     LOG_INF("Skipping gesture configuration for now.");
 
     // --- End Configuration --- 
     LOG_DBG("Configuration sequence complete.");
 
     // Configure IRQ GPIO
     LOG_DBG("Configuring IRQ GPIO pin %d on port %s", config->irq_gpio.pin, config->irq_gpio.port->name);
     if (!device_is_ready(config->irq_gpio.port)) {
         LOG_ERR("IRQ GPIO port not ready");
         return -ENODEV;
     }
     ret = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
     if (ret) {
         LOG_ERR("Failed to configure IRQ GPIO: %d", ret);
         return ret;
     }
     gpio_init_callback(&data->gpio_cb, iqs5xx_gpio_callback, BIT(config->irq_gpio.pin));
     LOG_DBG("Adding GPIO callback...");
     ret = gpio_add_callback(config->irq_gpio.port, &data->gpio_cb);
     if (ret) {
         LOG_ERR("Failed to add GPIO callback: %d", ret);
         // Try removing callback if it was partially added?
         gpio_remove_callback(config->irq_gpio.port, &data->gpio_cb);
         return ret;
     }
     // Configure interrupt after adding callback
     LOG_DBG("Configuring GPIO interrupt...");
     ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE); // Active high RDY pin
     if (ret) {
         LOG_ERR("Failed to configure GPIO interrupt: %d", ret);
         gpio_remove_callback(config->irq_gpio.port, &data->gpio_cb);
         return ret;
     }
     LOG_DBG("IRQ GPIO configuration complete.");
 
     // Initialize work queue
     k_work_init(&data->work, iqs5xx_work_handler);
 
     // Wait for first report cycle? QMK doesn't seem to wait here.
     // k_sleep(K_MSEC(15)); // Wait slightly longer than report rate
 
     LOG_INF("Successfully initialized IQS%u touchpad", data->product_number);
     return 0;
 }
 
 // Instantiate the driver
 #define IQS5XX_INIT(inst) \
     static struct iqs5xx_data iqs5xx_data_##inst; \
     static const struct iqs5xx_config iqs5xx_config_##inst = { \
         .i2c = I2C_DT_SPEC_INST_GET(inst), \
         .irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios), \
         COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, reset_gpios), \
                     (.reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),), ()) \
         .x_resolution = DT_INST_PROP_OR(inst, x_resolution, 3072), /* Default for TPS65 */ \
         .y_resolution = DT_INST_PROP_OR(inst, y_resolution, 2048), /* Default for TPS65 */ \
         .flip_x = DT_INST_PROP(inst, flip_x), \
         .flip_y = DT_INST_PROP(inst, flip_y), \
         .switch_xy = DT_INST_PROP(inst, switch_xy), \
     }; \
     DEVICE_DT_INST_DEFINE(inst, &iqs5xx_init, NULL, \
                           &iqs5xx_data_##inst, &iqs5xx_config_##inst, \
                           POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
                           NULL); // No specific API needed for input reporting
 
 DT_INST_FOREACH_STATUS_OKAY(IQS5XX_INIT)