/*
 * DJI Camera Remote Control - User Interface System
 * 
 * This file implements the complete user interface system for the DJI camera remote
 * control device, including:
 * 
 * - Display management and screen rendering
 * - Button-based navigation and function execution
 * - GPIO trigger system for external hardware integration
 * - Camera pairing and connection state management
 * - Persistent storage of camera information using NVS
 * - Device ID generation and management
 * - Status updates and real-time camera state tracking
 * 
 * The UI system supports multiple screens accessed via button navigation:
 * 1. Connect - Camera pairing and connection management
 * 2. Shutter - Photo capture and video recording control
 * 3. Mode - Camera mode switching (photo, video, timelapse, etc.)
 * 4. Sleep - Put camera into sleep mode
 * 5. Wake - Wake camera from sleep using BLE broadcast
 * 
 * GPIO Integration:
 * - G0 (pull LOW): Trigger shutter function
 * - G26 (pull HIGH): Trigger sleep function
 * - G25 (pull HIGH): Trigger wake function
 * 
 * All GPIO triggers include device ID-based randomized delays (1-100ms)
 * to prevent interference when multiple devices are used simultaneously.
 * 
 * Hardware: M5StickC Plus2 (ESP32-PICO-V3, 240x135 TFT LCD)
 * Framework: ESP-IDF v5.5 with FreeRTOS
 * 
 * Based on original DJI SDK implementation
 */

#include "ui.h"
#include "m5stickc_plus2_hal.h"
#include "command_logic.h"
#include "status_logic.h"
#include "enums_logic.h"
#include "connect_logic.h"
#include "data.h"
#include "ble.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "esp_random.h"
#include "driver/gpio.h"
#include "freertos/queue.h"

/* Logging tag for ESP_LOG functions */
#define TAG "UI"

/* NVS (Non-Volatile Storage) configuration for persistent data */
#define NVS_CAMERA_NAMESPACE "camera"      /* Namespace for camera pairing data */
#define NVS_CAMERA_KEY "paired_info"       /* Key for stored camera information */
#define NVS_DEVICE_NAMESPACE "device"      /* Namespace for device configuration */
#define NVS_DEVICE_ID_KEY "device_id"      /* Key for unique device identifier */

/* GPIO pin definitions for external hardware triggers
 * These pins allow external devices to trigger camera functions
 * with randomized delays based on the device's unique ID
 */
#define GPIO_SHUTTER_PIN    0   /* G0 - Pull LOW to trigger SHUTTER (photo/video) */
#define GPIO_SLEEP_PIN     26   /* G26 - Pull HIGH to trigger SLEEP mode */
#define GPIO_WAKE_PIN      25   /* G25 - Pull HIGH to trigger WAKE from sleep */

/* Enumeration of GPIO trigger types for external hardware integration
 * These correspond to the three main camera functions that can be
 * triggered via GPIO pins with randomized delays
 */
typedef enum {
    GPIO_TRIGGER_SHUTTER = 0,    /* Photo capture or video recording toggle */
    GPIO_TRIGGER_SLEEP,          /* Put camera into sleep mode */
    GPIO_TRIGGER_WAKE            /* Wake camera from sleep mode */
} gpio_trigger_type_t;

/* Camera information storage structure for persistent pairing data
 * This structure contains all necessary information to reconnect to
 * a previously paired camera without requiring manual pairing
 */
typedef struct {
    bool is_paired;                 /* True if camera has been successfully paired */
    char camera_name[64];           /* Camera BLE advertising name (e.g., "OsmoAction5Pro1C59") */
    uint8_t camera_mac[6];          /* Camera's BLE MAC address for targeted connection */
    uint32_t device_id;             /* Protocol-level device identifier from camera */
    uint8_t mac_addr_len;           /* Length of protocol MAC address (typically 6) */
    int8_t mac_addr[6];             /* Protocol-level MAC address from camera handshake */
    uint32_t fw_version;            /* Camera firmware version for compatibility checks */
    uint16_t verify_data;           /* Last successful verification code for reconnection */
    uint8_t camera_reserved;        /* Camera identifier number in multi-camera setups */
} stored_camera_t;

/* Global camera storage - holds information about the currently paired camera
 * This data is synchronized with NVS for persistence across power cycles
 */
static stored_camera_t g_stored_camera = {0};

/* Global DJI protocol connection parameters
 * These variables are used for camera communication and are either
 * generated randomly (device_id) or received from the camera during handshake
 */
uint32_t g_device_id = 0x12345678;                           /* Unique device identifier (randomized on first boot) */
uint8_t g_mac_addr_len = 6;                                  /* MAC address length (always 6 for BLE) */
int8_t g_mac_addr[6] = {0x38, 0x34, 0x56, 0x78, 0x9A, 0xBC}; /* Protocol MAC address from remote device */
uint32_t g_fw_version = 0x00;                                /* Firmware version for compatibility */
uint8_t g_verify_mode = 0;                                   /* Authentication mode: 0=reconnect, 1=pair */
uint16_t g_verify_data = 0;                                  /* Random verification code for security */
uint8_t g_camera_reserved = 0;                               /* Camera number in multi-camera environments */

/* Global user interface state management
 * Tracks current screen, display update requirements, and device-specific
 * scaling parameters for proper rendering on different M5StickC variants
 */
ui_state_t g_ui_state = {
    .current_screen = SCREEN_CONNECT,    /* Start on connection screen */
    .display_needs_update = true,        /* Force initial display update */
    .is_plus2_device = false,           /* Detected device type (set during init) */
    .scale_factor = 1.0f,               /* Display scaling factor for text/graphics */
    .scaled_text_size = 1               /* Text size multiplier for readability */
};

/* Screen layout configuration - positions and sizes for UI elements
 * Automatically configured based on detected device type and screen resolution
 */
screen_layout_t g_layout = {0};

/**
 * @brief Save camera pairing information to non-volatile storage
 * 
 * Stores complete camera information including BLE details and protocol
 * parameters to NVS for automatic reconnection after device restart.
 * 
 * @param camera Pointer to camera information structure to save
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t save_camera_to_nvs(const stored_camera_t* camera) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_CAMERA_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_set_blob(nvs_handle, NVS_CAMERA_KEY, camera, sizeof(stored_camera_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving camera to NVS: %s", esp_err_to_name(err));
    } else {
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error committing to NVS: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Camera info saved to NVS successfully");
        }
    }
    
    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Generate a unique random device identifier
 * 
 * Creates a 32-bit random device ID using ESP32's hardware random number
 * generator. This ID is used for DJI protocol communication and GPIO
 * trigger delay calculation to prevent interference between multiple devices.
 * 
 * @return 32-bit random device identifier (never zero)
 */
static uint32_t generate_random_device_id(void) {
    /* Use ESP32's hardware-based random number generator for cryptographic quality */
    uint32_t random_id = esp_random();
    
    /* Ensure ID is never zero to avoid protocol issues */
    if (random_id == 0) {
        random_id = 0x12345678;  /* Fallback pattern if hardware RNG fails */
    }
    
    ESP_LOGI(TAG, "Generated random device ID: 0x%08X", (unsigned int)random_id);
    return random_id;
}

/**
 * @brief Save device ID to non-volatile storage for persistence
 * 
 * Stores the device's unique identifier to NVS so it remains consistent
 * across power cycles and reboots. This ensures GPIO trigger delays and
 * protocol communication remain stable.
 * 
 * @param device_id The 32-bit device identifier to save
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t save_device_id_to_nvs(uint32_t device_id) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_DEVICE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening device NVS handle: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_set_u32(nvs_handle, NVS_DEVICE_ID_KEY, device_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving device ID to NVS: %s", esp_err_to_name(err));
    } else {
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error committing device ID to NVS: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Device ID 0x%08X saved to NVS successfully", (unsigned int)device_id);
        }
    }
    
    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Load device ID from non-volatile storage
 * 
 * Retrieves the previously stored device identifier from NVS.
 * Returns ESP_ERR_NVS_NOT_FOUND if this is the first boot.
 * 
 * @param device_id Pointer to store the loaded device ID
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if not found, ESP_ERR_* on other failures
 */
static esp_err_t load_device_id_from_nvs(uint32_t* device_id) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_DEVICE_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening device NVS handle for reading: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_get_u32(nvs_handle, NVS_DEVICE_ID_KEY, device_id);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "Device ID not found in NVS, this is a first boot");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error loading device ID from NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Device ID 0x%08X loaded from NVS successfully", (unsigned int)*device_id);
    }
    
    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Initialize device ID on system startup
 * 
 * Handles device ID initialization by either loading an existing ID from NVS
 * or generating a new random ID on first boot. The device ID is used for:
 * - DJI protocol communication
 * - GPIO trigger delay calculation
 * - Multi-device interference prevention
 */
static void initialize_device_id(void) {
    uint32_t stored_device_id;
    esp_err_t err = load_device_id_from_nvs(&stored_device_id);
    
    ESP_LOGI(TAG, "Device ID initialization starting, current g_device_id: 0x%08X", (unsigned int)g_device_id);
    
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // First boot - generate new random device ID
        ESP_LOGI(TAG, "First boot detected, generating random device ID...");
        g_device_id = generate_random_device_id();
        
        // Save to NVS for future boots
        err = save_device_id_to_nvs(g_device_id);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to save device ID to NVS, using session-only ID");
        }
    } else if (err == ESP_OK) {
        // Device ID found in NVS - use it
        g_device_id = stored_device_id;
        ESP_LOGI(TAG, "Using stored device ID: 0x%08X", (unsigned int)g_device_id);
    } else {
        // Error loading from NVS - use default but try to save random one
        ESP_LOGW(TAG, "Error loading device ID from NVS, generating new one");
        g_device_id = generate_random_device_id();
        save_device_id_to_nvs(g_device_id);  // Try to save for next time
    }
    
    ESP_LOGI(TAG, "Device ID initialization complete, final g_device_id: 0x%08X", (unsigned int)g_device_id);
}

/* GPIO control system components for external hardware integration
 * Implements thread-safe GPIO trigger processing with debouncing and delays
 */
static QueueHandle_t gpio_trigger_queue = NULL;                              /* FreeRTOS queue for GPIO events */
static TickType_t last_gpio_trigger[3] = {0, 0, 0};                         /* Per-pin debouncing timestamps */
static TickType_t last_global_gpio_trigger = 0;                             /* Global cooldown timestamp */
static volatile gpio_trigger_type_t pending_gpio_action = (gpio_trigger_type_t)-1; /* Pending action for main thread */

/**
 * @brief Calculate device-specific GPIO trigger delay
 * 
 * Generates a consistent but pseudo-random delay (1-100ms) based on the device's
 * unique ID. This prevents multiple devices from triggering simultaneously when
 * connected to the same external trigger source, reducing RF interference and
 * improving reliability in multi-device deployments.
 * 
 * @return Delay in milliseconds (1-100ms range)
 */
static uint32_t calculate_gpio_delay_ms(void) {
    /* Safety check: ensure device ID is initialized */
    if (g_device_id == 0) {
        ESP_LOGW(TAG, "Device ID not initialized, using default 50ms delay");
        return 50;
    }
    
    /* Use lower 16 bits of device ID as entropy source for consistent randomization */
    uint16_t id_hash = (uint16_t)(g_device_id & 0xFFFF);
    
    /* Map 16-bit hash (0-65535) to delay range (1-100ms) with minimum 1ms guarantee */
    uint32_t delay_ms = 1 + ((id_hash * 99) / 65535);
    
    ESP_LOGI(TAG, "GPIO trigger delay calculated: %lu ms (based on device ID 0x%08X)", 
             delay_ms, (unsigned int)g_device_id);
    
    return delay_ms;
}

/* GPIO debouncing and cooldown configuration
 * Aggressive timing to prevent multiple triggers from mechanical switch bounce
 * and to ensure stable operation with external hardware
 */
#define GPIO_DEBOUNCE_TIME_MS 1000      /* Per-pin debouncing period (1 second) */
#define GPIO_GLOBAL_COOLDOWN_MS 1000    /* Global cooldown between any GPIO triggers (1 second) */

/**
 * @brief GPIO interrupt service routine
 * 
 * Handles GPIO pin state changes by queuing trigger events for processing
 * in the main task context. The ISR must be kept minimal and fast, so all
 * actual processing is deferred to the GPIO monitor task.
 * 
 * @param arg Pointer to gpio_trigger_type_t indicating which trigger type
 */
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    gpio_trigger_type_t trigger_type = (gpio_trigger_type_t)(uintptr_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Send trigger type to queue for processing in main task
    xQueueSendFromISR(gpio_trigger_queue, &trigger_type, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Process GPIO trigger with debouncing and delay calculation
 * 
 * Implements comprehensive GPIO trigger processing including:
 * - Per-pin and global debouncing to prevent multiple triggers
 * - Device ID-based delay calculation for multi-device coordination
 * - Thread-safe pending action system for main thread execution
 * 
 * @param trigger_type Type of GPIO trigger to process
 */
static void process_gpio_trigger(gpio_trigger_type_t trigger_type) {
    TickType_t current_time = xTaskGetTickCount();
    TickType_t debounce_ticks = pdMS_TO_TICKS(GPIO_DEBOUNCE_TIME_MS);
    TickType_t global_cooldown_ticks = pdMS_TO_TICKS(GPIO_GLOBAL_COOLDOWN_MS);
    
    // Check for global cooldown (any GPIO trigger within 1 second)
    if ((current_time - last_global_gpio_trigger) < global_cooldown_ticks) {
        ESP_LOGW(TAG, "GPIO trigger %d ignored due to global cooldown", trigger_type);
        return;
    }
    
    // Check for per-pin debouncing
    if (trigger_type < 3 && (current_time - last_gpio_trigger[trigger_type]) < debounce_ticks) {
        ESP_LOGW(TAG, "GPIO trigger %d ignored due to pin debouncing", trigger_type);
        return;
    }
    
    // Check if there's already a pending action
    if (pending_gpio_action != (gpio_trigger_type_t)-1) {
        ESP_LOGW(TAG, "GPIO trigger %d ignored, action already pending", trigger_type);
        return;
    }
    
    // Update timestamps
    last_global_gpio_trigger = current_time;
    if (trigger_type < 3) {
        last_gpio_trigger[trigger_type] = current_time;
    }
    
    uint32_t delay_ms = calculate_gpio_delay_ms();
    
    ESP_LOGI(TAG, "GPIO trigger %d processing, delaying %lu ms", trigger_type, delay_ms);
    
    // Wait for the calculated delay
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    
    // Set the pending action for main thread to execute
    pending_gpio_action = trigger_type;
    ESP_LOGI(TAG, "GPIO trigger %d ready for execution by main thread", trigger_type);
}

/**
 * @brief GPIO monitoring task - processes queued GPIO events
 * 
 * FreeRTOS task that receives GPIO trigger events from the ISR queue
 * and processes them with appropriate delays and debouncing. Runs
 * continuously waiting for events.
 * 
 * @param pvParameters Unused task parameter (required by FreeRTOS)
 */
static void gpio_monitor_task(void* pvParameters) {
    gpio_trigger_type_t trigger_type;
    
    while (1) {
        if (xQueueReceive(gpio_trigger_queue, &trigger_type, portMAX_DELAY)) {
            process_gpio_trigger(trigger_type);
        }
    }
}

/**
 * @brief Initialize the complete GPIO trigger system
 * 
 * Sets up GPIO pins, interrupt handlers, debouncing system, and monitoring task.
 * Configures:
 * - G0 (SHUTTER): Input with pull-up, trigger on falling edge (pull to LOW)
 * - G26 (SLEEP): Input with pull-down, trigger on rising edge (pull to HIGH)
 * - G25 (WAKE): Input with pull-down, trigger on rising edge (pull to HIGH)
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t init_gpio_system(void) {
    esp_err_t ret = ESP_OK;
    
    ESP_LOGI(TAG, "Initializing GPIO trigger system");
    ESP_LOGI(TAG, "Current device ID: 0x%08X", (unsigned int)g_device_id);
    
    /* Create FreeRTOS queue for GPIO trigger events (ISR to task communication) */
    gpio_trigger_queue = xQueueCreate(10, sizeof(gpio_trigger_type_t));
    if (gpio_trigger_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create GPIO trigger queue");
        return ESP_FAIL;
    }
    
    /* Initialize debouncing timestamps and pending action state */
    for (int i = 0; i < 3; i++) {
        last_gpio_trigger[i] = 0;
    }
    last_global_gpio_trigger = 0;
    pending_gpio_action = (gpio_trigger_type_t)-1;
    
    /* Configure GPIO pins with appropriate pull resistors and interrupt types */
    gpio_config_t io_conf = {};
    
    /* G0 (SHUTTER) - Input with internal pull-up resistor
     * External circuit should pull pin LOW to trigger shutter function
     */
    io_conf.intr_type = GPIO_INTR_NEGEDGE;      /* Interrupt on falling edge (HIGH to LOW) */
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_SHUTTER_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure SHUTTER GPIO pin");
        return ret;
    }
    
    /* G26 (SLEEP) - Input with internal pull-down resistor
     * External circuit should pull pin HIGH to trigger sleep function
     */
    io_conf.intr_type = GPIO_INTR_POSEDGE;      /* Interrupt on rising edge (LOW to HIGH) */
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_SLEEP_PIN);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure SLEEP GPIO pin");
        return ret;
    }
    
    /* G25 (WAKE) - Input with internal pull-down resistor
     * External circuit should pull pin HIGH to trigger wake function
     */
    io_conf.intr_type = GPIO_INTR_POSEDGE;      /* Interrupt on rising edge (LOW to HIGH) */
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_WAKE_PIN);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure WAKE GPIO pin");
        return ret;
    }
    
    /* Install GPIO interrupt service if not already installed */
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  /* ESP_ERR_INVALID_STATE = already installed */
        ESP_LOGE(TAG, "Failed to install GPIO ISR service");
        return ret;
    }
    
    /* Register interrupt handlers for each GPIO pin with trigger type identification */
    ret = gpio_isr_handler_add(GPIO_SHUTTER_PIN, gpio_isr_handler, (void*)GPIO_TRIGGER_SHUTTER);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SHUTTER GPIO ISR handler");
        return ret;
    }
    
    ret = gpio_isr_handler_add(GPIO_SLEEP_PIN, gpio_isr_handler, (void*)GPIO_TRIGGER_SLEEP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SLEEP GPIO ISR handler");
        return ret;
    }
    
    ret = gpio_isr_handler_add(GPIO_WAKE_PIN, gpio_isr_handler, (void*)GPIO_TRIGGER_WAKE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add WAKE GPIO ISR handler");
        return ret;
    }
    
    /* Create FreeRTOS task for GPIO event processing with sufficient stack size */
    BaseType_t task_ret = xTaskCreate(gpio_monitor_task, "gpio_monitor", 4096, NULL, 10, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPIO monitor task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "GPIO trigger system initialized successfully");
    ESP_LOGI(TAG, "  G0 (SHUTTER): Pull LOW to trigger");
    ESP_LOGI(TAG, "  G26 (SLEEP): Pull HIGH to trigger");
    ESP_LOGI(TAG, "  G25 (WAKE): Pull HIGH to trigger");
    
    return ESP_OK;
}

/**
 * @brief Process pending GPIO actions from main thread
 * 
 * This function must be called regularly from the main application loop
 * to execute GPIO-triggered actions in the main thread context. This
 * ensures thread safety by avoiding UI function calls from GPIO tasks.
 */
void ui_process_pending_gpio_actions(void) {
    if (pending_gpio_action != (gpio_trigger_type_t)-1) {
        gpio_trigger_type_t action = pending_gpio_action;
        pending_gpio_action = (gpio_trigger_type_t)-1; // Clear the pending action first
        
        ESP_LOGI(TAG, "Executing pending GPIO action: %d", action);
        
        switch (action) {
            case GPIO_TRIGGER_SHUTTER:
                ui_screen_shutter();
                break;
                
            case GPIO_TRIGGER_SLEEP:
                ui_screen_sleep();
                break;
                
            case GPIO_TRIGGER_WAKE:
                ui_screen_wake();
                break;
        }
    }
}

/**
 * @brief Load camera pairing information from non-volatile storage
 * 
 * Retrieves previously stored camera information from NVS for automatic
 * reconnection capabilities. Returns ESP_ERR_NVS_NOT_FOUND if no camera
 * has been paired yet.
 * 
 * @param camera Pointer to structure to fill with loaded camera data
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if not found, ESP_ERR_* on other failures
 */
static esp_err_t load_camera_from_nvs(stored_camera_t* camera) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    size_t required_size = sizeof(stored_camera_t);
    
    err = nvs_open(NVS_CAMERA_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error opening NVS handle for reading: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_get_blob(nvs_handle, NVS_CAMERA_KEY, camera, &required_size);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Camera info loaded from NVS successfully");
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No camera info found in NVS");
        // Initialize with default values
        memset(camera, 0, sizeof(stored_camera_t));
    } else {
        ESP_LOGE(TAG, "Error loading camera from NVS: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Store camera information after successful pairing
 * 
 * Captures and stores complete camera information including BLE details
 * and protocol parameters for future automatic reconnection. Called
 * automatically after successful camera pairing.
 */
static void store_camera_info(void) {
    g_stored_camera.is_paired = true;
    
    /* Capture BLE connection information from currently connected camera */
    if (ble_get_connected_device_info(g_stored_camera.camera_name, 
                                      sizeof(g_stored_camera.camera_name), 
                                      g_stored_camera.camera_mac)) {
        ESP_LOGI(TAG, "Captured camera BLE info: %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
                 g_stored_camera.camera_name,
                 g_stored_camera.camera_mac[0], g_stored_camera.camera_mac[1], 
                 g_stored_camera.camera_mac[2], g_stored_camera.camera_mac[3],
                 g_stored_camera.camera_mac[4], g_stored_camera.camera_mac[5]);
    } else {
        ESP_LOGW(TAG, "Failed to get camera BLE info, using default values");
        strncpy(g_stored_camera.camera_name, "Unknown Camera", sizeof(g_stored_camera.camera_name) - 1);
        memset(g_stored_camera.camera_mac, 0, sizeof(g_stored_camera.camera_mac));
    }
    
    /* Store DJI protocol-level information obtained during connection handshake */
    g_stored_camera.device_id = g_device_id;
    g_stored_camera.mac_addr_len = g_mac_addr_len;
    memcpy(g_stored_camera.mac_addr, g_mac_addr, sizeof(g_mac_addr));
    g_stored_camera.fw_version = g_fw_version;
    g_stored_camera.verify_data = g_verify_data;
    g_stored_camera.camera_reserved = g_camera_reserved;
    
    /* Write all information to persistent NVS storage */
    esp_err_t err = save_camera_to_nvs(&g_stored_camera);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Camera information stored persistently for future auto-connect");
    } else {
        ESP_LOGE(TAG, "Failed to store camera info persistently");
    }
}

/**
 * @brief Load stored camera information for reconnection
 * 
 * Restores global protocol variables from stored camera information
 * to enable reconnection using previously established parameters.
 */
static void load_stored_camera_info(void) {
    if (g_stored_camera.is_paired) {
        g_device_id = g_stored_camera.device_id;
        g_mac_addr_len = g_stored_camera.mac_addr_len;
        memcpy(g_mac_addr, g_stored_camera.mac_addr, sizeof(g_stored_camera.mac_addr));
        g_fw_version = g_stored_camera.fw_version;
        g_verify_data = g_stored_camera.verify_data;
        g_camera_reserved = g_stored_camera.camera_reserved;
        ESP_LOGI(TAG, "Loaded stored camera info for reconnection");
    }
}

/**
 * @brief Attempt automatic connection to paired camera on startup
 * 
 * Called during system initialization to automatically connect to a
 * previously paired camera if one exists. Uses stored BLE and protocol
 * information to establish connection without user intervention.
 */
/**
 * @brief Perform complete camera reconnection (BLE + Protocol)
 * 
 * Internal function that handles both BLE and protocol reconnection.
 * Used by both startup auto-connect and background reconnection.
 * 
 * @param show_messages Whether to show UI status messages
 * @return int 0 on success, -1 on failure
 */
static int ui_perform_complete_reconnection(bool show_messages) {
    if (!g_stored_camera.is_paired) {
        ESP_LOGI(TAG, "No paired camera found, cannot reconnect");
        return -1;
    }
    
    ESP_LOGI(TAG, "Performing complete reconnection");
    if (show_messages) {
        ui_show_message("Reconnecting...", M5_COLOR_CYAN, 1000);
    }
    
    /* Configure BLE layer to target the specific paired camera */
    ble_set_target_device(g_stored_camera.camera_name, g_stored_camera.camera_mac);
    
    load_stored_camera_info();
    g_verify_mode = 0;  /* Use reconnection mode (no pairing required) */
    
    /* Initiate BLE connection with reconnection flag */
    int res = connect_logic_ble_connect(true);  /* is_reconnecting = true */
    if (res == 0) {
        if (show_messages) {
            ui_show_message("BLE Connected\nConnecting protocol...", M5_COLOR_BLUE, 1000);
        }
        
        /* Establish DJI protocol connection using stored parameters */
        res = connect_logic_protocol_connect(
            g_device_id,
            g_mac_addr_len,
            g_mac_addr,
            g_fw_version,
            g_verify_mode,
            g_verify_data,
            g_camera_reserved
        );
        
        if (res == 0) {
            if (show_messages) {
                ui_show_message("Reconnected!", M5_COLOR_GREEN, 1000);
            }
            /* Enable real-time camera status monitoring */
            subscript_camera_status(PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
            ESP_LOGI(TAG, "Complete reconnection successful");
            return 0;
        } else {
            if (show_messages) {
                ui_show_message("Protocol connect failed", M5_COLOR_RED, 1500);
            }
            ESP_LOGW(TAG, "Protocol connection failed during reconnection");
        }
    } else {
        if (show_messages) {
            ui_show_message("BLE connect failed", M5_COLOR_RED, 1500);
        }
        ESP_LOGW(TAG, "BLE connection failed during reconnection");
    }
    
    return -1;
}

void ui_auto_connect_on_startup(void) {
    if (g_stored_camera.is_paired) {
        ESP_LOGI(TAG, "Found paired camera, attempting auto-connect");
        ui_perform_complete_reconnection(true);
    } else {
        ESP_LOGI(TAG, "No paired camera found, manual pairing required");
    }
}

/**
 * @brief Attempt background reconnection without UI messages
 * 
 * Public function for background reconnection attempts. This function
 * performs a complete reconnection (BLE + protocol) without showing
 * UI messages, making it suitable for periodic background attempts.
 * 
 * @return int 0 on success, -1 on failure or not needed
 */
int ui_attempt_background_reconnection(void) {
    /* Only attempt reconnection if we're disconnected but initialized */
    connect_state_t current_state = connect_logic_get_state();
    if (current_state >= BLE_SEARCHING) {
        /* Already connected or connecting */
        return 0;
    }
    
    if (current_state < BLE_INIT_COMPLETE) {
        /* BLE not initialized, cannot reconnect */
        return -1;
    }
    
    if (!g_stored_camera.is_paired) {
        /* No paired camera to reconnect to */
        return -1;
    }
    
    ESP_LOGI(TAG, "Background reconnection attempt");
    return ui_perform_complete_reconnection(false);  /* No UI messages */
}

/**
 * @brief Initialize the complete user interface system
 * 
 * Performs comprehensive UI system initialization including:
 * - NVS (Non-Volatile Storage) initialization
 * - Device ID generation/loading
 * - Camera pairing data loading
 * - Data layer initialization
 * - Display configuration
 * - Automatic connection attempts
 * - GPIO trigger system setup
 */
void ui_init(void) {
    ESP_LOGI(TAG, "Initializing UI system");
    
    /* Initialize NVS flash storage for persistent data */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS initialized successfully");
    
    /* Initialize unique device identifier (random generation on first boot) */
    initialize_device_id();
    
    /* Load any previously paired camera information from persistent storage */
    esp_err_t load_err = load_camera_from_nvs(&g_stored_camera);
    if (load_err == ESP_OK && g_stored_camera.is_paired) {
        ESP_LOGI(TAG, "Found paired camera in storage: %s (MAC: %02X:%02X:%02X:%02X:%02X:%02X)", 
                 g_stored_camera.camera_name,
                 g_stored_camera.camera_mac[0], g_stored_camera.camera_mac[1],
                 g_stored_camera.camera_mac[2], g_stored_camera.camera_mac[3],
                 g_stored_camera.camera_mac[4], g_stored_camera.camera_mac[5]);
    } else {
        ESP_LOGI(TAG, "No paired camera found in storage");
    }
    
    /* Initialize DJI protocol data layer with status update callbacks */
    if (!is_data_layer_initialized()) {
        ESP_LOGI(TAG, "Initializing data layer...");
        data_init();
        data_register_status_update_callback(update_camera_state_handler);
        data_register_new_status_update_callback(update_new_camera_state_handler);
        if (!is_data_layer_initialized()) {
            ESP_LOGE(TAG, "Failed to initialize data layer");
            return;
        }
        ESP_LOGI(TAG, "Data layer initialized successfully");
    }
    
    /* Configure display scaling and layout for detected hardware */
    ui_detect_device_and_set_scale();
    g_ui_state.current_screen = SCREEN_CONNECT;
    g_ui_state.display_needs_update = true;
    
    /* Attempt automatic connection to previously paired camera */
    ui_auto_connect_on_startup();
    
    /* Perform initial display update */
    ui_update_display();
    
    /* Initialize GPIO trigger system after all other subsystems are ready */
    esp_err_t gpio_err = init_gpio_system();
    if (gpio_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize GPIO system: %s", esp_err_to_name(gpio_err));
        ESP_LOGW(TAG, "Continuing without GPIO triggers");
    }
}

/**
 * @brief Detect M5StickC device type and configure display scaling
 * 
 * Determines the specific M5StickC variant and configures appropriate
 * scaling factors for optimal display rendering. Currently configured
 * for M5StickC Plus2 (240x135 resolution).
 */
void ui_detect_device_and_set_scale(void) {
    /* M5StickC Plus2: 240x135 display, original M5StickC: 160x80 display
     * Currently targeting Plus2 hardware exclusively
     */
    g_ui_state.is_plus2_device = true;
    g_ui_state.scale_factor = 1.5f;
    g_ui_state.scaled_text_size = 2;
    
    ESP_LOGI(TAG, "Detected: M5StickC Plus2 (240x135)");
    ESP_LOGI(TAG, "Scale factor: %.1f, Text size: %d", 
             g_ui_state.scale_factor, g_ui_state.scaled_text_size);
    
    /* Configure screen layout coordinates for M5StickC Plus2 (240x135 resolution)
     * All positions calculated for optimal visual balance and readability
     */
    g_layout.icon_x = (240 - 32) / 2;           /* Center 32px icons horizontally (104px from left) */
    g_layout.icon_y = 25 + 12;                  /* Icon vertical position with offset */
    g_layout.text_x = 240 / 2;                  /* Center text horizontally (120px from left) */
    g_layout.text_y = g_layout.icon_y + 32 + 12; /* Text below icon with spacing for double-size text */
    g_layout.status_x = 220;                    /* Connection status indicator position */
    g_layout.status_y = 12;
    g_layout.connection_radius = 7;             /* Size of connection status indicator */
    g_layout.dots_y = 120;                      /* Screen indicator dots vertical position */
    g_layout.dots_spacing = 25;                 /* Horizontal spacing between screen dots */
    g_layout.dots_start_x = 45;                 /* Starting x position for screen dots */
    g_layout.instruct_x = 8;                    /* Instruction text position */
    g_layout.instruct_y = 8;
}

/**
 * @brief Draw bitmap icon at specified position
 * 
 * Renders a bitmap icon with specified dimensions and color using the
 * hardware-specific display driver functions.
 * 
 * @param x Horizontal position
 * @param y Vertical position  
 * @param bitmap Pointer to bitmap data array
 * @param w Width in pixels
 * @param h Height in pixels
 * @param color Foreground color (RGB565 format)
 */
void ui_draw_bitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {
    /* Use hardware-optimized bitmap rendering with black background */
    m5stickc_plus2_display_draw_bitmap(x, y, w, h, bitmap, color, M5_COLOR_BLACK);
}

/**
 * @brief Draw connection status indicator
 * 
 * Displays a colored rectangle indicating the current camera connection state:
 * - Green: Fully connected to camera (BLE + protocol)
 * - Red: Not connected or connection in progress
 */
void ui_draw_connection_status(void) {
    uint16_t color;
    connect_state_t state = connect_logic_get_state();
    
    /* Color coding: Green for fully connected, Red for any other state */
    if (state == PROTOCOL_CONNECTED) {
        color = M5_COLOR_GREEN;
    } else {
        color = M5_COLOR_RED;
    }
    
    /* Draw connection status as small filled rectangle in top-right corner */
    int size = g_layout.connection_radius * 2;
    m5stickc_plus2_display_fill_rect(g_layout.status_x - size/2, g_layout.status_y - size/2, 
                                     size, size, color);
}

/**
 * @brief Calculate text width for centering calculations
 * 
 * Estimates the rendered width of text based on character count and scaling.
 * Used for centering text on the display.
 * 
 * @param text Text string to measure
 * @param text_size Text scaling factor
 * @return Estimated text width in pixels
 */
int ui_get_text_width(const char* text, int text_size) {
    /* Calculate width based on fixed-width font assumption (8 pixels base * scale) */
    int char_width = 8 * text_size;
    return strlen(text) * char_width;
}

/**
 * @brief Update display with current UI state
 * 
 * Renders the complete user interface including:
 * - Connection status indicator
 * - Current screen icon and text
 * - Screen navigation dots
 * - Instruction text
 * 
 * Only updates when display_needs_update flag is set for efficiency.
 */
void ui_update_display(void) {
    if (!g_ui_state.display_needs_update) {
        return;
    }
    
    /* Clear entire display to prevent visual artifacts */
    m5stickc_plus2_display_fill_rect(0, 0, 240, 135, M5_COLOR_BLACK);
    
    /* Draw connection status indicator in top-right corner */
    ui_draw_connection_status();
    
    /* Draw screen navigation indicators at bottom of display
     * Shows current screen position and total available screens
     */
    for (int i = 0; i < SCREEN_COUNT; i++) {
        int x = g_layout.dots_start_x + (i * g_layout.dots_spacing);
        int y = g_layout.dots_y;
        int dot_size = g_ui_state.is_plus2_device ? 8 : 6;
        int inactive_size = g_ui_state.is_plus2_device ? 6 : 4;
        
        if (i == g_ui_state.current_screen) {
            /* Current screen - larger white rectangle */
            m5stickc_plus2_display_fill_rect(x - dot_size/2, y - dot_size/2, dot_size, dot_size, M5_COLOR_WHITE);
        } else {
            /* Other screens - smaller gray rectangles */
            m5stickc_plus2_display_fill_rect(x - inactive_size/2, y - inactive_size/2, inactive_size, inactive_size, M5_COLOR_DARKGREY);
        }
    }
    
    /* Get information for currently selected screen */
    const screen_info_t* screen = &screen_info[g_ui_state.current_screen];
    
    /* Draw screen-specific icon with color coding */
    ui_draw_bitmap(g_layout.icon_x, g_layout.icon_y, screen->icon, 32, 32, screen->color);
    
    /* Calculate centered text position for double-sized text rendering */
    int text_scale = 2;  /* Double size for better readability */
    int text_width = ui_get_text_width(screen->name, text_scale);
    int centered_text_x = g_layout.text_x - (text_width / 2);
    
    /* Draw screen name centered below icon */
    m5stickc_plus2_display_print_scaled(centered_text_x, g_layout.text_y, screen->name, M5_COLOR_WHITE, text_scale);
    
    /* Display button instructions in top-left corner */
    m5stickc_plus2_display_print(g_layout.instruct_x, g_layout.instruct_y, "A : Run   B: Next", M5_COLOR_GREY);
    
    g_ui_state.display_needs_update = false;
    ESP_LOGI(TAG, "Display updated - Screen: %s", screen->name);
}

/**
 * @brief Navigate to next screen in sequence
 * 
 * Cycles through available screens: Connect → Shutter → Mode → Sleep → Wake → Connect...
 * Called when Button B is pressed.
 */
void ui_next_screen(void) {
    g_ui_state.current_screen = (g_ui_state.current_screen + 1) % SCREEN_COUNT;
    g_ui_state.display_needs_update = true;
    ESP_LOGI(TAG, "Switched to screen: %d (%s)", 
             g_ui_state.current_screen, screen_info[g_ui_state.current_screen].name);
}

/**
 * @brief Execute function for currently selected screen
 * 
 * Calls the screen-specific function (connect, shutter, mode switch, etc.)
 * when Button A is pressed or GPIO trigger occurs.
 */
void ui_execute_current_screen(void) {
    const screen_info_t* screen = &screen_info[g_ui_state.current_screen];
    ESP_LOGI(TAG, "Executing function for screen: %s", screen->name);
    
    if (screen->execute_func) {
        screen->execute_func();
    }
    
    /* Schedule display update to reflect any changes */
    g_ui_state.display_needs_update = true;
}

/**
 * @brief Display temporary message with automatic timeout
 * 
 * Shows a status or feedback message for a specified duration, then
 * returns to normal UI display. Used for operation feedback.
 * 
 * @param message Text to display
 * @param color Text color (RGB565 format)
 * @param duration_ms Display duration in milliseconds
 */
void ui_show_message(const char* message, uint16_t color, int duration_ms) {
    m5stickc_plus2_display_clear(M5_COLOR_BLACK);
    
    /* Calculate message position based on device type */
    int msg_x = g_ui_state.is_plus2_device ? 40 : 30;
    int msg_y = g_ui_state.is_plus2_device ? 60 : 40;
    
    m5stickc_plus2_display_print(msg_x, msg_y, message, color);
    
    /* Block for specified duration (simple implementation) */
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    
    /* Schedule normal UI restore */
    g_ui_state.display_needs_update = true;
}

/**
 * @brief Display "not connected" error message
 * 
 * Convenience function to show connection error when user attempts
 * camera operations without an active connection.
 */
void ui_show_not_connected_message(void) {
    ui_show_message("Not Connected!", M5_COLOR_RED, 1500);
}

/**
 * @brief Attempt manual camera pairing as fallback
 * 
 * Called when automatic reconnection fails, initiating a fresh pairing
 * process that requires user interaction on the camera side.
 */
static void ui_try_manual_pairing(void) {
    ESP_LOGI(TAG, "Attempting manual pairing fallback");
    
    /* Switch to pairing mode for new device registration */
    g_verify_mode = 1;  /* Pairing mode requires camera-side confirmation */
    
    /* Generate random verification code for secure pairing */
    srand((unsigned int)time(NULL));
    g_verify_data = (uint16_t)(rand() % 10000);
    
    /* Start BLE connection in discovery mode (scan for any compatible camera) */
    int res = connect_logic_ble_connect(false);  /* is_reconnecting = false */
    if (res == 0) {
        ui_show_message("BLE Connected\nPress camera pair button", M5_COLOR_CYAN, 2000);
        
        /* Establish DJI protocol connection in pairing mode */
        res = connect_logic_protocol_connect(
            g_device_id,
            g_mac_addr_len,
            g_mac_addr,
            g_fw_version,
            g_verify_mode,
            g_verify_data,
            g_camera_reserved
        );
        
        if (res == 0) {
            ui_show_message("Manual Pair Success!", M5_COLOR_GREEN, 1500);
            /* Save new camera information for future auto-connect */
            store_camera_info();
            /* Enable real-time status monitoring */
            subscript_camera_status(PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
        } else {
            ui_show_message("Manual Pair Failed", M5_COLOR_RED, 1500);
        }
    } else {
        ui_show_message("Manual BLE Failed", M5_COLOR_RED, 1500);
    }
}

/**
 * @brief Handle camera connection screen activation
 * 
 * Manages camera pairing and reconnection logic:
 * - If camera already paired: attempts reconnection
 * - If no paired camera: initiates pairing process
 * - Handles fallback to manual pairing if auto-reconnect fails
 */
void ui_screen_connect(void) {
    ESP_LOGI(TAG, "Executing connect screen");
    
    connect_state_t state = connect_logic_get_state();
    
    if (state == PROTOCOL_CONNECTED) {
        ui_show_message("Already Connected!", M5_COLOR_GREEN, 1000);
        return;
    }
    
    if (g_stored_camera.is_paired) {
        /* Camera already paired - attempt automatic reconnection */
        ESP_LOGI(TAG, "Attempting to reconnect to paired camera (verify_mode=0)");
        ui_show_message("Reconnecting...", M5_COLOR_CYAN, 500);
        
        /* Configure BLE to target the specific paired camera */
        ble_set_target_device(g_stored_camera.camera_name, g_stored_camera.camera_mac);
        
        load_stored_camera_info();
        g_verify_mode = 0;  /* Reconnection mode - no pairing required */
        
        /* Initiate BLE connection to paired device */
        int res = connect_logic_ble_connect(true);  /* is_reconnecting = true */
        if (res == 0) {
            ui_show_message("BLE Connected\nConnecting protocol...", M5_COLOR_BLUE, 1000);
            
            /* Establish DJI protocol connection using stored credentials */
            res = connect_logic_protocol_connect(
                g_device_id,
                g_mac_addr_len,
                g_mac_addr,
                g_fw_version,
                g_verify_mode,
                g_verify_data,
                g_camera_reserved
            );
            
            if (res == 0) {
                ui_show_message("Reconnected!", M5_COLOR_GREEN, 1000);
                /* Enable real-time camera status monitoring */
                subscript_camera_status(PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
            } else {
                ui_show_message("Reconnect Failed\nTrying manual pairing...", M5_COLOR_ORANGE, 1500);
                /* Fallback to fresh pairing attempt */
                ui_try_manual_pairing();
            }
        } else {
            ui_show_message("BLE Failed\nTrying manual pairing...", M5_COLOR_ORANGE, 1500);
            /* Fallback to fresh pairing attempt */
            ui_try_manual_pairing();
        }
    } else {
        /* No paired camera found - initiate first-time pairing */
        ESP_LOGI(TAG, "First time pairing (verify_mode=1)");
        ui_show_message("Pairing Mode\nPress camera pair button", M5_COLOR_CYAN, 2000);
        
        g_verify_mode = 1;  /* Pairing mode requires user confirmation */
        
        /* Generate random verification code for secure pairing */
        srand((unsigned int)time(NULL));
        g_verify_data = (uint16_t)(rand() % 10000);
        
        /* Start BLE connection in discovery mode */
        int res = connect_logic_ble_connect(false);  /* is_reconnecting = false */
        if (res == 0) {
            ui_show_message("BLE Connected\nPairing...", M5_COLOR_BLUE, 1000);
            
            /* Establish DJI protocol connection in pairing mode */
            res = connect_logic_protocol_connect(
                g_device_id,
                g_mac_addr_len,
                g_mac_addr,
                g_fw_version,
                g_verify_mode,
                g_verify_data,
                g_camera_reserved
            );
            
            if (res == 0) {
                /* Successful pairing - save camera information for future use */
                store_camera_info();
                ui_show_message("Paired Successfully!", M5_COLOR_GREEN, 1500);
                /* Enable real-time camera status monitoring */
                subscript_camera_status(PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
            } else {
                ui_show_message("Pairing Failed", M5_COLOR_RED, 1500);
            }
        } else {
            ui_show_message("BLE Connect Failed", M5_COLOR_RED, 1500);
        }
    }
}

/**
 * @brief Handle shutter screen activation
 * 
 * Controls photo capture and video recording based on current camera mode:
 * - Photo mode: Always takes a photo
 * - Video modes: Toggles recording start/stop
 * 
 * Requires active camera connection to function.
 */
void ui_screen_shutter(void) {
    ESP_LOGI(TAG, "Executing shutter screen");
    
    connect_state_t state = connect_logic_get_state();
    if (state != PROTOCOL_CONNECTED) {
        ui_show_not_connected_message();
        return;
    }
    
    /* Check if camera status has been received for mode-aware operation */
    if (!camera_status_initialized) {
        ESP_LOGW(TAG, "Camera status not yet initialized, attempting command anyway");
        ui_show_message("Status Unknown\nTrying anyway...", M5_COLOR_ORANGE, 1000);
    }
    
    /* Determine shutter behavior based on current camera mode */
    camera_mode_t current_mode = (camera_mode_t)current_camera_mode;
    
    ESP_LOGI(TAG, "Current camera mode: %d, status: %d, recording: %s", 
             current_mode, current_camera_status, is_camera_recording() ? "yes" : "no");
    
    if (current_mode == CAMERA_MODE_PHOTO) {
        /* Photo mode - single shot capture */
        ESP_LOGI(TAG, "Taking photo in photo mode");
        record_control_response_frame_t* response = command_logic_start_record();
        if (response) {
            ui_show_message("Photo Taken", M5_COLOR_GREEN, 1000);
            free(response);
        } else {
            ui_show_message("Photo Failed", M5_COLOR_RED, 1500);
        }
    } else {
        /* Video modes - toggle recording state */
        bool is_recording = is_camera_recording();
        
        if (is_recording) {
            /* Stop current recording */
            ESP_LOGI(TAG, "Stopping recording in video mode");
            record_control_response_frame_t* response = command_logic_stop_record();
            if (response) {
                ui_show_message("Recording Stopped", M5_COLOR_YELLOW, 1000);
                free(response);
            } else {
                ui_show_message("Stop Failed", M5_COLOR_RED, 1500);
            }
        } else {
            /* Start new recording */
            ESP_LOGI(TAG, "Starting recording in video mode");
            record_control_response_frame_t* response = command_logic_start_record();
            if (response) {
                ui_show_message("Recording Started", M5_COLOR_GREEN, 1000);
                free(response);
            } else {
                ui_show_message("Start Failed", M5_COLOR_RED, 1500);
            }
        }
    }
}

/**
 * @brief Handle camera mode switching screen activation
 * 
 * Cycles through common camera modes in sequence:
 * Video → Photo → Timelapse → Slow Motion → Video...
 * 
 * Requires active camera connection to function.
 */
void ui_screen_mode(void) {
    ESP_LOGI(TAG, "Executing mode screen");
    
    connect_state_t state = connect_logic_get_state();
    if (state != PROTOCOL_CONNECTED) {
        ui_show_not_connected_message();
        return;
    }
    
    /* Determine next mode in cycling sequence */
    camera_mode_t current_mode = (camera_mode_t)current_camera_mode;
    camera_mode_t next_mode;
    const char* mode_name;
    
    /* Mode cycling logic - rotates through most commonly used modes */
    switch (current_mode) {
        case CAMERA_MODE_NORMAL:
            next_mode = CAMERA_MODE_PHOTO;
            mode_name = "Photo";
            break;
        case CAMERA_MODE_PHOTO:
            next_mode = CAMERA_MODE_TIMELAPSE;
            mode_name = "Timelapse";
            break;
        case CAMERA_MODE_TIMELAPSE:
            next_mode = CAMERA_MODE_SLOW_MOTION;
            mode_name = "Slow Motion";
            break;
        case CAMERA_MODE_SLOW_MOTION:
            next_mode = CAMERA_MODE_NORMAL;
            mode_name = "Video";
            break;
        default:
            next_mode = CAMERA_MODE_NORMAL;
            mode_name = "Video";
            break;
    }
    
    ESP_LOGI(TAG, "Switching from mode %d to mode %d (%s)", current_mode, next_mode, mode_name);
    
    camera_mode_switch_response_frame_t* response = command_logic_switch_camera_mode(next_mode);
    if (response) {
        char message[32];
        snprintf(message, sizeof(message), "Mode: %s", mode_name);
        ui_show_message(message, M5_COLOR_ORANGE, 1500);
        free(response);
    } else {
        ui_show_message("Mode Failed", M5_COLOR_RED, 1500);
    }
}

/**
 * @brief Handle camera sleep screen activation
 * 
 * Sends Camera Power Mode Settings command (CmdSet=0x00, CmdID=0x1A)
 * to put the camera into sleep mode for power conservation.
 * 
 * Requires active camera connection to function.
 */
void ui_screen_sleep(void) {
    ESP_LOGI(TAG, "Executing sleep screen");
    
    connect_state_t state = connect_logic_get_state();
    if (state != PROTOCOL_CONNECTED) {
        ui_show_not_connected_message();
        return;
    }
    
    ESP_LOGI(TAG, "Sending sleep command to camera");
    camera_power_mode_switch_response_frame_t* response = command_logic_power_mode_switch_sleep();
    if (response) {
        if (response->ret_code == 0x00) {
            ui_show_message("Camera Sleeping", M5_COLOR_CYAN, 1500);
            ESP_LOGI(TAG, "Camera successfully put to sleep");
        } else {
            ui_show_message("Sleep Failed", M5_COLOR_RED, 1500);
            ESP_LOGW(TAG, "Sleep command failed with ret_code: %d", response->ret_code);
        }
        free(response);
    } else {
        ui_show_message("Sleep Failed", M5_COLOR_RED, 1500);
        ESP_LOGE(TAG, "Failed to send sleep command");
    }
}

/**
 * @brief Handle camera wake screen activation
 * 
 * Sends BLE advertising broadcast with wake-up pattern to rouse the camera
 * from sleep mode. Uses the MAC address of the paired camera to ensure
 * targeted wake-up.
 * 
 * Format: [10, 0xff, 'W','K','P', MAC[5-0] reversed]
 * Duration: 2 seconds of broadcasting
 */
void ui_screen_wake(void) {
    ESP_LOGI(TAG, "Executing wake screen");
    
    /* Verify we have a paired camera with known MAC address */
    if (!g_stored_camera.is_paired) {
        ui_show_message("No Paired Camera", M5_COLOR_RED, 1500);
        ESP_LOGW(TAG, "No paired camera found for wake broadcast");
        return;
    }
    
    /* Check if we're currently disconnected */
    bool was_disconnected = (connect_logic_get_state() <= BLE_INIT_COMPLETE);
    
    ESP_LOGI(TAG, "Starting wake broadcast for paired camera");
    esp_err_t ret = ble_wake_camera(g_stored_camera.camera_mac);
    
    if (ret == ESP_OK) {
        ui_show_message("Wake Broadcast\nSent (2s)", M5_COLOR_YELLOW, 2000);
        ESP_LOGI(TAG, "Wake broadcast started successfully");
        
        /* If we were disconnected when sending wake, attempt reconnection after delay */
        if (was_disconnected) {
            ESP_LOGI(TAG, "Wake sent while disconnected, will attempt reconnection in 3 seconds");
            vTaskDelay(pdMS_TO_TICKS(3000));  /* Wait for camera to wake up */
            
            int reconnect_result = ui_perform_complete_reconnection(true);
            if (reconnect_result == 0) {
                ESP_LOGI(TAG, "Reconnection successful after wake broadcast");
            } else {
                ESP_LOGW(TAG, "Failed to reconnect after wake broadcast");
            }
        }
    } else {
        ui_show_message("Wake Failed", M5_COLOR_RED, 1500);
        ESP_LOGE(TAG, "Failed to start wake broadcast: %s", esp_err_to_name(ret));
    }
}