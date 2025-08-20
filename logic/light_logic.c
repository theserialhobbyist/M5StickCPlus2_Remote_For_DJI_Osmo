#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include "connect_logic.h"
#include "status_logic.h"

#define TAG "LOGIC_LIGHT"

#if defined(CONFIG_IDF_TARGET_ESP32) && defined(M5STICKC_PLUS2)
/* M5StickC Plus2 internal LED */
#define LED_GPIO 10               // M5StickC Plus2 internal LED on GPIO 10
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
/* ESP32-C6 LED */
#define LED_GPIO 8                // Use GPIO to control RGB LED
#else
/* Default LED */
#define LED_GPIO 2                // Default LED GPIO
#endif

#define LED_STRIP_LENGTH 1        // Set the number of LEDs to 1

// Create a led_strip handle
static led_strip_handle_t led_strip = NULL;

// Initialize RGB LED related configurations and settings
static void init_rgb_led(void) {
    // Configure LED
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_STRIP_LENGTH // Set the number of LEDs to 1
    };

    // Configure RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // Set RMT resolution to 10 MHz
        .flags.with_dma = false,           // Disable DMA
    };

    // Use &led_strip as the third parameter because it needs a pointer to led_strip_handle_t
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    led_strip_clear(led_strip); // Clear all LEDs and turn them off
    ESP_LOGI(TAG, "RGB LED initialized");
}

// Set RGB LED color
static void set_rgb_color(uint8_t red, uint8_t green, uint8_t blue) {
    led_strip_set_pixel(led_strip, 0, red, green, blue);  // Set LED color
    led_strip_refresh(led_strip); // Refresh LED Strip to update color
}

// Initialize variables needed for RGB LED status
uint8_t led_red = 0, led_green = 0, led_blue = 0;   // RGB values
bool led_blinking = false;                          // Whether to blink
bool current_led_on = false;                        // Current LED status (on or off)

// Function to update LED state
static void update_led_state() {
    connect_state_t current_connect_state = connect_logic_get_state();
    bool current_camera_recording = is_camera_recording();

    // Print current status
    // ESP_LOGI(TAG, "Current connect state: %d, Camera recording: %d", current_connect_state, current_camera_recording);

    led_blinking = false;  // LED does not blink by default

    switch (current_connect_state) {
        case BLE_NOT_INIT:
            led_red = 13;      // 255 * 0.05
            led_green = 0;
            led_blue = 0;      // Red indicates Bluetooth not initialized
            break;

        case BLE_INIT_COMPLETE:
            led_red = 13;      // 255 * 0.05
            led_green = 13;    // 255 * 0.05
            led_blue = 0;      // Yellow indicates Bluetooth initialization complete
            break;

        case BLE_SEARCHING:
            led_blinking = true;
            led_red = 0;
            led_green = 0;
            led_blue = 13;     // 255 * 0.05, Blue blinking indicates Bluetooth is searching
            break;

        case BLE_CONNECTED:
            led_red = 0;
            led_green = 0;
            led_blue = 13;     // 255 * 0.05, Blue indicates Bluetooth is connected
            break;

        case PROTOCOL_CONNECTED:
            if (current_camera_recording) {
                led_blinking = true;
                led_red = 0;
                led_green = 13;    // 255 * 0.05
                led_blue = 0;      // Green blinking indicates recording
            } else {
                led_red = 0;
                led_green = 13;    // 255 * 0.05
                led_blue = 0;      // Green indicates not recording
            }
            break;
            
        default:
            led_red = 0;
            led_green = 0;
            led_blue = 0;  // Turn off LED
            break;
    }
}

// Timer callback function for periodic LED state updates
static void led_state_timer_callback(TimerHandle_t xTimer) {
    update_led_state();
}

// Timer callback function for LED blinking effect
static void led_blink_timer_callback(TimerHandle_t xTimer) {
    if (led_blinking) {
        // If in blinking state and LED is currently on, turn it off
        if (current_led_on) {
            set_rgb_color(0, 0, 0);  // Turn off LED
        } else {
            // If LED is currently off, set it to RGB color
            set_rgb_color(led_red, led_green, led_blue);
        }
        current_led_on = !current_led_on;
    } else {
        // If not in blinking state, directly set to RGB color
        set_rgb_color(led_red, led_green, led_blue);
    }
}

// Initialize light logic, including LED state updates and blink timer
int init_light_logic() {
    init_rgb_led();
    
    // Create a timer that executes update_led_state every 500ms
    TimerHandle_t led_state_timer = xTimerCreate("led_state_timer", pdMS_TO_TICKS(500), pdTRUE, (void *)0, led_state_timer_callback);

    if (led_state_timer != NULL) {
        xTimerStart(led_state_timer, 0);
        ESP_LOGI(TAG, "LED state timer started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create LED state timer");
        return -1;
    }

    // Create another timer to control LED blinking state (on/off)
    TimerHandle_t led_blink_timer = xTimerCreate("led_blink_timer", pdMS_TO_TICKS(500), pdTRUE, (void *)0, led_blink_timer_callback);

    if (led_blink_timer != NULL) {
        xTimerStart(led_blink_timer, 0);
        ESP_LOGI(TAG, "LED blink timer started successfully");
        return 0;
    } else {
        ESP_LOGE(TAG, "Failed to create LED blink timer");
        return -1;
    }
}