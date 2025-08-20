/*
 * DJI Camera Remote Control - Main Application Entry Point
 * 
 * This file contains the main application entry point and core system initialization
 * for the DJI camera remote control system running on M5StickC Plus2 hardware.
 * 
 * The system provides a Bluetooth Low Energy (BLE) interface to control DJI cameras
 * with features including:
 * - Camera connection management
 * - Shutter control (photo/video recording)
 * - Camera mode switching
 * - Sleep/wake functionality
 * - GPIO trigger support for external hardware
 * 
 * Hardware: M5StickC Plus2 (ESP32-based)
 * Display: 240x135 TFT LCD
 * Connectivity: Bluetooth Low Energy
 * 
 * Based on original DJI SDK implementation
 */

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "connect_logic.h"
#include "light_logic.h"
#include "m5stickc_plus2_hal.h"
#include "ui.h"

/**
 * @brief Main application entry point
 * 
 * This function serves as the ESP-IDF application entry point and implements
 * the complete system initialization sequence followed by the main event loop.
 * 
 * Initialization sequence:
 * 1. M5StickC Plus2 hardware (display, buttons, power management)
 * 2. RGB LED light system
 * 3. Bluetooth Low Energy subsystem
 * 4. User interface system (including GPIO triggers)
 * 
 * Main loop handles:
 * - Power button monitoring (3-second hold for shutdown)
 * - User input from physical buttons
 * - GPIO trigger processing from external hardware
 * - Display updates
 * 
 * @note This function never returns under normal operation
 */
void app_main(void) {
    static const char *TAG = "MAIN";
    int res = 0;

    /* 
     * HARDWARE INITIALIZATION PHASE
     * Initialize all hardware components in dependency order
     */
    
    /* Initialize M5StickC Plus2 hardware platform
     * This includes: display controller, button GPIO, power management,
     * I2C bus, SPI bus, and other core hardware peripherals
     */
    res = m5stickc_plus2_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize M5StickC Plus2 hardware");
        return;
    }
    ESP_LOGI(TAG, "M5StickC Plus2 hardware initialized");

    /* Initialize RGB LED light system
     * Sets up the WS2812 LED strip driver for status indication
     * Colors indicate: connection state, operation status, errors
     */
    res = init_light_logic();
    if (res != 0) {
        ESP_LOGE(TAG, "Failed to initialize light logic");
        return;
    }
    ESP_LOGI(TAG, "Light logic initialized");

    /* Initialize Bluetooth Low Energy subsystem
     * Configures ESP32 BLE stack for DJI camera communication
     * Sets up GATT client, advertising scanner, and connection management
     */
    ESP_LOGI(TAG, "Initializing Bluetooth...");
    res = connect_logic_ble_init();
    if (res != 0) {
        ESP_LOGE(TAG, "Failed to initialize Bluetooth");
        return;
    }
    ESP_LOGI(TAG, "Bluetooth initialized successfully");

    /* Initialize user interface system
     * Sets up: display rendering, screen management, button handlers,
     * GPIO triggers, camera state management, NVS storage
     */
    ui_init();
    ESP_LOGI(TAG, "UI system initialized");

    /* System ready - log operational information for user */
    ESP_LOGI(TAG, "System ready - Icon-based UI active!");
    ESP_LOGI(TAG, "Button A: Select/execute current option");
    ESP_LOGI(TAG, "Button B: Cycle through options");

    /*
     * MAIN APPLICATION EVENT LOOP
     * Handles all user input, system events, and display updates
     */
    
    /* Power button state tracking for shutdown detection */
    static uint32_t power_button_hold_time = 0;
    static bool power_button_was_pressed = false;
    
    /* Background reconnection timer - attempt every 15 seconds when disconnected */
    static uint32_t reconnection_timer = 0;
    static const uint32_t RECONNECTION_INTERVAL_MS = 15000;
    
    while (1) {
        /*
         * POWER MANAGEMENT
         * Monitor power button for 3-second hold to initiate shutdown
         */
        bool power_button_pressed = m5stickc_plus2_button_pwr_pressed();
        
        if (power_button_pressed && !power_button_was_pressed) {
            /* Power button press detected - start hold timer */
            power_button_hold_time = 0;
            power_button_was_pressed = true;
            ESP_LOGI(TAG, "Power button pressed - hold for 3s to shutdown");
            
        } else if (power_button_pressed && power_button_was_pressed) {
            /* Power button held - increment timer and check for shutdown threshold */
            power_button_hold_time += 50; // Increment by main loop delay (50ms)
            
            if (power_button_hold_time >= 3000) { // 3 second threshold
                ESP_LOGI(TAG, "Power button held for 3s - shutting down");
                
                /* Display shutdown message to user */
                ui_show_message("Shutting down...", M5_COLOR_RED, 1000);
                
                /* Disable power hold circuit to turn off device
                 * M5StickC Plus2 uses a power hold circuit that must be
                 * actively maintained to keep the device powered on
                 */
                gpio_set_level(M5_PWR_EN_PIN, 0);
                
                /* Infinite loop in case shutdown fails */
                while(1) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
        } else if (!power_button_pressed && power_button_was_pressed) {
            /* Power button released before shutdown threshold - reset timer */
            power_button_was_pressed = false;
            power_button_hold_time = 0;
        }

        /*
         * USER INPUT HANDLING
         * Process physical button presses with debouncing
         */
        
        /* Button A: Execute current screen function
         * Actions: Connect, Shutter, Mode change, Sleep, Wake
         */
        if (m5stickc_plus2_button_a_pressed()) {
            ESP_LOGI(TAG, "Button A pressed - selecting current option");
            ui_execute_current_screen();
            
            /* Wait for button release to prevent multiple triggers */
            while (m5stickc_plus2_button_a_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            vTaskDelay(pdMS_TO_TICKS(200)); // Additional debounce delay
        }

        /* Button B: Navigate to next screen
         * Cycles through: Connect → Shutter → Mode → Sleep → Wake → Connect...
         */
        if (m5stickc_plus2_button_b_pressed()) {
            ESP_LOGI(TAG, "Button B pressed - cycling to next option");
            ui_next_screen();
            
            /* Wait for button release to prevent multiple triggers */
            while (m5stickc_plus2_button_b_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            vTaskDelay(pdMS_TO_TICKS(200)); // Additional debounce delay
        }

        /*
         * GPIO TRIGGER PROCESSING
         * Handle external GPIO triggers with randomized delays
         * This allows external hardware to trigger camera functions
         */
        ui_process_pending_gpio_actions();

        /*
         * DISPLAY UPDATE
         * Refresh display if any UI elements have changed
         * Uses dirty flag system for efficiency
         */
        ui_update_display();

        /*
         * BACKGROUND RECONNECTION
         * Attempt automatic reconnection every 15 seconds when disconnected
         */
        reconnection_timer += 50; /* Increment by main loop delay (50ms) */
        
        if (reconnection_timer >= RECONNECTION_INTERVAL_MS) {
            reconnection_timer = 0; /* Reset timer */
            
            /* Attempt background reconnection through UI layer */
            ESP_LOGI(TAG, "Background reconnection timer triggered");
            int reconnect_result = ui_attempt_background_reconnection();
            if (reconnect_result == 0) {
                ESP_LOGI(TAG, "Background reconnection attempt completed successfully");
            } else {
                ESP_LOGI(TAG, "Background reconnection not needed or failed");
            }
        }

        /*
         * BACKGROUND TASKS
         * Light logic runs on its own FreeRTOS timer task
         * BLE operations run on ESP-IDF BLE stack tasks
         * No manual updates needed for these systems
         */

        /* Main loop timing - 50ms cycle time
         * Provides responsive UI while preventing excessive CPU usage
         */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}