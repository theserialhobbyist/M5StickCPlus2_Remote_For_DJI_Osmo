/*
 * DJI Camera Remote Control - M5StickC Plus2 Hardware Abstraction Layer
 * 
 * This file implements the complete hardware abstraction layer for the M5StickC Plus2
 * development board, providing unified interfaces for all hardware components:
 * 
 * Hardware Components:
 * - ST7789 TFT LCD Display (240x135, 16-bit color)
 * - Power management (hold circuit, backlight PWM)
 * - Physical buttons (A, B, Power)
 * - I2C bus (IMU, RTC communication)
 * - SPI bus (display communication)
 * - GPIO control and monitoring
 * 
 * Display Features:
 * - Hardware-accelerated bitmap rendering
 * - Text rendering with scalable 8x8 font
 * - Color fill operations with RGB565 format
 * - Transparent bitmap drawing
 * - Configurable brightness control
 * 
 * The HAL provides a clean interface for the UI layer while handling all
 * low-level hardware details, timing, and ESP-IDF driver integration.
 * 
 * Hardware: M5StickC Plus2 (ESP32-PICO-V3)
 * Display: ST7789 controller, 240x135 resolution
 * Framework: ESP-IDF v5.5 with ESP-LCD drivers
 * 
 * Based on M5Stack hardware specifications
 */

#include "m5stickc_plus2_hal.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "esp_heap_caps.h"

/* LCD support for M5StickC Plus2 */
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

/* 8x8 Pixel Font Definition
 * 
 * Compact monospace font for text rendering on the display.
 * Each character is 8x8 pixels, stored as 8 bytes (1 byte per row).
 * Bit pattern: 1 = foreground pixel, 0 = transparent/background
 * 
 * Character set: ASCII 32-122 (space through lowercase z)
 * Optimized for readability at small sizes on TFT displays
 */
static const uint8_t font8x8[][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // Space
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // !
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // " (empty for now)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // # (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // $ (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // % (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // & (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ' (empty)
    {0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00}, // (
    {0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00}, // )
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // * (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // + (empty)
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30}, // ,
    {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, // -
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, // .
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // / (empty)
    {0x3C,0x66,0x6E,0x76,0x66,0x66,0x3C,0x00}, // 0
    {0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00}, // 1
    {0x3C,0x66,0x06,0x0C,0x30,0x60,0x7E,0x00}, // 2
    {0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00}, // 3
    {0x0C,0x1C,0x3C,0x6C,0x7E,0x0C,0x0C,0x00}, // 4
    {0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00}, // 5
    {0x3C,0x66,0x60,0x7C,0x66,0x66,0x3C,0x00}, // 6
    {0x7E,0x66,0x0C,0x18,0x18,0x18,0x18,0x00}, // 7
    {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00}, // 8
    {0x3C,0x66,0x66,0x3E,0x06,0x66,0x3C,0x00}, // 9
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00}, // : - fine dots
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ; (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // < (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // = (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // > (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ? (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // @ (empty)
    {0x18,0x24,0x42,0x7E,0x42,0x42,0x42,0x00}, // A - finer
    {0x7C,0x42,0x42,0x7C,0x42,0x42,0x7C,0x00}, // B - finer
    {0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00}, // C
    {0x78,0x6C,0x66,0x66,0x66,0x6C,0x78,0x00}, // D
    {0x7E,0x40,0x40,0x78,0x40,0x40,0x7E,0x00}, // E - finer
    {0x7E,0x40,0x40,0x78,0x40,0x40,0x40,0x00}, // F - finer
    {0x3C,0x66,0x60,0x6E,0x66,0x66,0x3C,0x00}, // G
    {0x42,0x42,0x42,0x7E,0x42,0x42,0x42,0x00}, // H - finer
    {0x7E,0x18,0x18,0x18,0x18,0x18,0x7E,0x00}, // I
    {0x06,0x06,0x06,0x06,0x06,0x66,0x3C,0x00}, // J
    {0x66,0x6C,0x78,0x70,0x78,0x6C,0x66,0x00}, // K
    {0x40,0x40,0x40,0x40,0x40,0x40,0x7E,0x00}, // L - finer
    {0x63,0x77,0x7F,0x6B,0x63,0x63,0x63,0x00}, // M
    {0x42,0x62,0x72,0x5A,0x4E,0x46,0x42,0x00}, // N - finer
    {0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, // O
    {0x7C,0x66,0x66,0x7C,0x60,0x60,0x60,0x00}, // P
    {0x3C,0x66,0x66,0x66,0x66,0x3C,0x0E,0x00}, // Q
    {0x7C,0x42,0x42,0x7C,0x48,0x44,0x42,0x00}, // R - finer
    {0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00}, // S
    {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, // T - finer
    {0x42,0x42,0x42,0x42,0x42,0x42,0x3C,0x00}, // U - finer
    {0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00}, // V
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, // W
    {0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00}, // X
    {0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x00}, // Y
    {0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00}, // Z
    {0x0E,0x18,0x18,0x70,0x18,0x18,0x0E,0x00}, // [
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // \ (empty)
    {0x70,0x18,0x18,0x0E,0x18,0x18,0x70,0x00}, // ]
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ^ (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00}, // _
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ` (empty)
    {0x00,0x00,0x3C,0x06,0x3E,0x66,0x3E,0x00}, // a
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x7C,0x00}, // b
    {0x00,0x00,0x3C,0x66,0x60,0x66,0x3C,0x00}, // c
    {0x06,0x06,0x3E,0x66,0x66,0x66,0x3E,0x00}, // d
    {0x00,0x00,0x3C,0x66,0x7E,0x60,0x3C,0x00}, // e
    {0x1C,0x36,0x30,0x78,0x30,0x30,0x30,0x00}, // f
    {0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x3C}, // g
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x00}, // h
    {0x18,0x00,0x38,0x18,0x18,0x18,0x3C,0x00}, // i
    {0x06,0x00,0x0E,0x06,0x06,0x66,0x66,0x3C}, // j
    {0x60,0x60,0x66,0x6C,0x78,0x6C,0x66,0x00}, // k
    {0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // l
    {0x00,0x00,0x66,0x7F,0x7F,0x6B,0x63,0x00}, // m
    {0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x00}, // n
    {0x00,0x00,0x3C,0x66,0x66,0x66,0x3C,0x00}, // o
    {0x00,0x00,0x7C,0x66,0x66,0x7C,0x60,0x60}, // p
    {0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x06}, // q
    {0x00,0x00,0x7C,0x66,0x60,0x60,0x60,0x00}, // r
    {0x00,0x00,0x3E,0x60,0x3C,0x06,0x7C,0x00}, // s
    {0x10,0x30,0x7C,0x30,0x30,0x36,0x1C,0x00}, // t
    {0x00,0x00,0x66,0x66,0x66,0x66,0x3E,0x00}, // u
    {0x00,0x00,0x66,0x66,0x66,0x3C,0x18,0x00}, // v
    {0x00,0x00,0x63,0x6B,0x7F,0x7F,0x36,0x00}, // w
    {0x00,0x00,0x66,0x3C,0x18,0x3C,0x66,0x00}, // x
    {0x00,0x00,0x66,0x66,0x66,0x3E,0x06,0x3C}, // y
    {0x00,0x00,0x7E,0x0C,0x18,0x30,0x7E,0x00}, // z
};

/* Logging tag for ESP_LOG functions */
static const char *TAG = "M5STICKC_HAL";

/* ESP-LCD driver handles for display communication */
static esp_lcd_panel_handle_t panel_handle = NULL;  /* ST7789 panel handle */
static esp_lcd_panel_io_handle_t io_handle = NULL;  /* SPI I/O handle */

/* Forward declarations for internal helper functions */
static void draw_char_scaled(int x, int y, char c, uint16_t color, uint16_t bg_color, int scale);

/* I2C Bus Configuration for M5StickC Plus2
 * Used for IMU (MPU6886) and RTC (BM8563) communication
 * 400kHz fast mode for optimal performance
 */
static i2c_config_t i2c_conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = M5_I2C_SDA_PIN,       /* GPIO21 - I2C Data line */
    .scl_io_num = M5_I2C_SCL_PIN,       /* GPIO22 - I2C Clock line */
    .sda_pullup_en = GPIO_PULLUP_ENABLE, /* Internal pull-up resistors */
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 400000,         /* 400kHz for fast I2C communication */
};

/**
 * @brief Initialize all M5StickC Plus2 hardware components
 * 
 * Performs complete hardware initialization in the correct dependency order:
 * 1. Power management (enables device hold circuit)
 * 2. I2C bus (for IMU and RTC communication)
 * 3. Display subsystem (ST7789 TFT LCD)
 * 4. Button inputs (A, B, Power buttons)
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
int m5stickc_plus2_init(void) {
    ESP_LOGI(TAG, "Initializing M5StickC Plus2 hardware");
    
    /* Initialize power management first - enables device hold circuit and backlight */
    int ret = m5stickc_plus2_power_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize power management");
        return ret;
    }
    
    /* Initialize I2C bus for sensor communication (IMU, RTC) */
    ret = m5stickc_plus2_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return ret;
    }
    
    /* Initialize display subsystem (SPI, ST7789 controller) */
    ret = m5stickc_plus2_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize display");
        return ret;
    }
    
    /* Initialize button input handling */
    ret = m5stickc_plus2_buttons_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize buttons");
        return ret;
    }
    
    ESP_LOGI(TAG, "M5StickC Plus2 hardware initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize M5StickC Plus2 power management
 * 
 * Sets up the device power hold circuit and display backlight control.
 * The M5StickC Plus2 uses a power hold circuit that must be actively
 * maintained to keep the device powered after the power button is released.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
int m5stickc_plus2_power_init(void) {
    ESP_LOGI(TAG, "Initializing power management for M5StickC Plus2");
    
    /* Configure power enable pin to maintain device power
     * This pin must be held HIGH to keep the device powered on
     * Setting it LOW will immediately shut down the device
     */
    gpio_config_t pwr_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << M5_PWR_EN_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    esp_err_t ret = gpio_config(&pwr_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure power enable pin: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Enable power hold circuit - keeps device powered after button release */
    gpio_set_level(M5_PWR_EN_PIN, 1);
    ESP_LOGI(TAG, "Power hold enabled on GPIO %d", M5_PWR_EN_PIN);
    
    /* Configure display backlight control pin
     * Used for both on/off control and PWM brightness adjustment
     */
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << M5_LCD_BL_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure backlight pin: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Enable backlight at full brightness by default */
    gpio_set_level(M5_LCD_BL_PIN, 1);
    
    ESP_LOGI(TAG, "Power management initialized - PWR_EN=%d, BL=%d", M5_PWR_EN_PIN, M5_LCD_BL_PIN);
    return ESP_OK;
}

/**
 * @brief Initialize I2C bus for M5StickC Plus2
 * 
 * Configures I2C_NUM_0 for communication with onboard sensors:
 * - MPU6886 IMU (6-axis accelerometer/gyroscope)
 * - BM8563 RTC (real-time clock)
 * 
 * Uses 400kHz fast mode for optimal sensor communication performance.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
int m5stickc_plus2_i2c_init(void) {
    /* Configure I2C parameters (pins, speed, pull-ups) */
    int ret = i2c_param_config(I2C_NUM_0, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed");
        return ret;
    }
    
    /* Install I2C driver with master mode, no slave buffers */
    ret = i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C initialized on SDA=%d, SCL=%d", M5_I2C_SDA_PIN, M5_I2C_SCL_PIN);
    return ESP_OK;
}

int m5stickc_plus2_display_init(void) {
    ESP_LOGI(TAG, "Display init - Step 1: GPIO initialization only");
    ESP_LOGI(TAG, "Pins: MOSI=%d, SCLK=%d, CS=%d, DC=%d, RST=%d, BL=%d", 
             M5_LCD_MOSI_PIN, M5_LCD_SCLK_PIN, M5_LCD_CS_PIN, 
             M5_LCD_DC_PIN, M5_LCD_RST_PIN, M5_LCD_BL_PIN);
    
    // Step 1: Just configure the GPIO pins without SPI
    ESP_LOGI(TAG, "Configuring display control GPIOs...");
    
    // Configure reset pin
    gpio_config_t rst_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << M5_LCD_RST_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    esp_err_t ret = gpio_config(&rst_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure RST pin: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "RST pin configured");
    
    // Configure DC pin
    gpio_config_t dc_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << M5_LCD_DC_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ret = gpio_config(&dc_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DC pin: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "DC pin configured");
    
    // Configure CS pin
    gpio_config_t cs_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << M5_LCD_CS_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ret = gpio_config(&cs_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CS pin: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(M5_LCD_CS_PIN, 1);  // CS high (inactive)
    ESP_LOGI(TAG, "CS pin configured and set high");
    
    // Perform a simple reset sequence
    ESP_LOGI(TAG, "Performing display reset sequence...");
    gpio_set_level(M5_LCD_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(M5_LCD_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Display reset complete");
    
    // Step 2: Initialize SPI bus
    ESP_LOGI(TAG, "Step 2: Initializing SPI bus...");
    
    spi_bus_config_t buscfg = {
        .mosi_io_num = M5_LCD_MOSI_PIN,
        .miso_io_num = -1,  // Display doesn't use MISO
        .sclk_io_num = M5_LCD_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    
    // Try VSPI_HOST (SPI3) instead of SPI2_HOST
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);  // Use DMA channel 1
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "SPI bus initialized on VSPI_HOST");
    
    // Step 3: Create LCD panel IO handle
    ESP_LOGI(TAG, "Step 3: Creating LCD panel IO...");
    
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = M5_LCD_DC_PIN,
        .cs_gpio_num = M5_LCD_CS_PIN,
        .pclk_hz = 26 * 1000 * 1000,  // 26MHz for better performance
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)VSPI_HOST, &io_config, &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(ret));
        spi_bus_free(VSPI_HOST);
        return ret;
    }
    ESP_LOGI(TAG, "LCD panel IO created");
    
    // Step 4: Create ST7789 panel
    ESP_LOGI(TAG, "Step 4: Creating ST7789 panel...");
    
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = M5_LCD_RST_PIN,
        .color_space = ESP_LCD_COLOR_SPACE_BGR,  // Use BGR color space
        .bits_per_pixel = 16,
        .flags = {
            .reset_active_high = 0,
        },
        .vendor_config = NULL,
    };
    
    ret = esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ST7789 panel: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "ST7789 panel created");
    
    // Step 5: Initialize the panel
    ESP_LOGI(TAG, "Step 5: Initializing panel...");
    
    ret = esp_lcd_panel_reset(panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset panel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_lcd_panel_init(panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init panel: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Panel initialized");
    
    // Step 6: Configure display settings
    ESP_LOGI(TAG, "Step 6: Configuring display settings...");
    
    // Invert colors (ST7789 often needs this)
    esp_lcd_panel_invert_color(panel_handle, true);
    
    // M5StickC Plus2 orientation: swap XY for landscape mode and flip 180 degrees
    // The display is physically 240x135 when rotated
    esp_lcd_panel_swap_xy(panel_handle, true);  // Swap for landscape
    esp_lcd_panel_mirror(panel_handle, false, true);  // Mirror Y axis to flip 180 degrees
    
    // Set the gap (offset) for M5StickC Plus2's ST7789V2 in landscape, flipped 180 degrees
    // Adjust these values to eliminate static edges after flip
    esp_lcd_panel_set_gap(panel_handle, 40, 52);  // Adjusted Y offset to eliminate bottom static
    
    // Turn on display
    esp_lcd_panel_disp_on_off(panel_handle, true);
    
    ESP_LOGI(TAG, "Display initialization complete!");
    
    // Clear display to black
    m5stickc_plus2_display_clear(M5_COLOR_BLACK);
    
    return ESP_OK;
    
    /* Rest disabled
    // Configure display control pins
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << M5_LCD_DC_PIN) | (1ULL << M5_LCD_RST_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO pins");
        return ret;
    }
    */
}

/**
 * @brief Initialize button input handling
 * 
 * Configures GPIO pins for the three physical buttons on M5StickC Plus2:
 * - Button A (GPIO35): Primary action button
 * - Button B (GPIO37): Secondary/navigation button  
 * - Power Button (GPIO39): Power control and system functions
 * 
 * Note: GPIOs 35, 37, 39 are input-only pins on ESP32 and cannot use
 * internal pull-up resistors. M5StickC Plus2 has external pull-ups.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
int m5stickc_plus2_buttons_init(void) {
    /* Configure button pins as inputs
     * GPIOs 35, 37, 39 are input-only on ESP32 - cannot use internal pull-ups
     * M5StickC Plus2 hardware provides external pull-up resistors
     */
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << M5_BTN_A_PIN) | (1ULL << M5_BTN_B_PIN) | (1ULL << M5_BTN_PWR_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,  /* Cannot use internal pull-up on input-only pins */
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "Buttons initialized - A=%d, B=%d, PWR=%d", M5_BTN_A_PIN, M5_BTN_B_PIN, M5_BTN_PWR_PIN);
    return ESP_OK;
}

/**
 * @brief Check if Button A is currently pressed
 * 
 * Button A is the primary action button used for executing current screen functions.
 * Returns true when button is physically pressed (GPIO reads LOW).
 * 
 * @return true if button is pressed, false otherwise
 */
bool m5stickc_plus2_button_a_pressed(void) {
    return gpio_get_level(M5_BTN_A_PIN) == 0;
}

/**
 * @brief Check if Button B is currently pressed
 * 
 * Button B is used for navigation between different screens and functions.
 * Returns true when button is physically pressed (GPIO reads LOW).
 * 
 * @return true if button is pressed, false otherwise
 */
bool m5stickc_plus2_button_b_pressed(void) {
    return gpio_get_level(M5_BTN_B_PIN) == 0;
}

/**
 * @brief Check if Power Button is currently pressed
 * 
 * Power button is used for device shutdown (3-second hold) and system functions.
 * Returns true when button is physically pressed (GPIO reads LOW).
 * 
 * @return true if button is pressed, false otherwise
 */
bool m5stickc_plus2_button_pwr_pressed(void) {
    return gpio_get_level(M5_BTN_PWR_PIN) == 0;
}

/**
 * @brief Set display backlight brightness
 * 
 * Controls display brightness using PWM on the backlight pin.
 * Initializes LEDC PWM controller on first call for smooth brightness control.
 * 
 * @param brightness Brightness level (0-255, 0=off, 255=maximum)
 */
void m5stickc_plus2_display_set_brightness(uint8_t brightness) {
    /* Initialize PWM controller once for backlight control */
    static bool ledc_initialized = false;
    
    if (!ledc_initialized) {
        /* Configure LEDC timer for 8-bit PWM at 5kHz (flicker-free) */
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = LEDC_TIMER_0,
            .duty_resolution = LEDC_TIMER_8_BIT,  /* 8-bit resolution (0-255) */
            .freq_hz = 5000,                      /* 5kHz frequency to avoid flicker */
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&ledc_timer);
        
        /* Configure LEDC channel for backlight control */
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = M5_LCD_BL_PIN,            /* Backlight control pin */
            .duty = 0,
            .hpoint = 0,
        };
        ledc_channel_config(&ledc_channel);
        ledc_initialized = true;
    }
    
    /* Set PWM duty cycle for brightness control (0-255) */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    
    ESP_LOGI(TAG, "Display brightness set to %d/255", brightness);
}

/**
 * @brief Get the ESP-LCD panel handle for direct access
 * 
 * Returns the internal LCD panel handle for advanced operations
 * that require direct ESP-LCD API access.
 * 
 * @return ESP-LCD panel handle, or NULL if not initialized
 */
esp_lcd_panel_handle_t m5stickc_plus2_get_display_handle(void) {
    return panel_handle;
}

/**
 * @brief Clear entire display with solid color
 * 
 * Fills the complete display area (240x135) with the specified color.
 * Uses chunked rendering to optimize memory usage for large clears.
 * 
 * @param color RGB565 color value to fill the display with
 */
void m5stickc_plus2_display_clear(uint16_t color) {
    if (panel_handle) {
        /* Clear display using chunked approach to manage memory efficiently */
        const int chunk_height = 30;  /* Process 30 rows at a time */
        const int total_pixels = M5_LCD_H_RES * chunk_height;
        uint16_t *buffer = malloc(total_pixels * sizeof(uint16_t));
        
        if (buffer) {
            /* Fill buffer with solid color (RGB565 format, no byte swapping needed) */
            for (int i = 0; i < total_pixels; i++) {
                buffer[i] = color;
            }
            
            /* Render display in horizontal chunks for memory efficiency */
            for (int y = 0; y < M5_LCD_V_RES; y += chunk_height) {
                int height = (y + chunk_height > M5_LCD_V_RES) ? (M5_LCD_V_RES - y) : chunk_height;
                esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, y, M5_LCD_H_RES, y + height, buffer);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to draw bitmap at y=%d: %s", y, esp_err_to_name(ret));
                }
            }
            free(buffer);
        } else {
            ESP_LOGE(TAG, "Failed to allocate display buffer");
        }
    }
}

/**
 * @brief Draw single character at normal scale
 * 
 * Convenience wrapper for draw_char_scaled with scale factor of 1.
 * Renders an 8x8 pixel character from the built-in font.
 * 
 * @param x Horizontal position
 * @param y Vertical position
 * @param c Character to draw
 * @param color Foreground color (RGB565)
 * @param bg_color Background color (RGB565, unused for transparency)
 */
static void draw_char(int x, int y, char c, uint16_t color, uint16_t bg_color) {
    draw_char_scaled(x, y, c, color, bg_color, 1);
}

/**
 * @brief Draw character with scaling support
 * 
 * Renders a character from the 8x8 font with specified scaling factor.
 * Uses pixel-by-pixel rendering for transparency - only foreground pixels
 * are drawn, allowing background to show through.
 * 
 * @param x Horizontal position
 * @param y Vertical position  
 * @param c Character to draw (ASCII 32-122)
 * @param color Foreground color (RGB565)
 * @param bg_color Background color (unused - transparent rendering)
 * @param scale Scaling factor (1=8x8, 2=16x16, etc.)
 */
static void draw_char_scaled(int x, int y, char c, uint16_t color, uint16_t bg_color, int scale) {
    if (panel_handle == NULL) return;
    
    /* Bounds checking with scaling factor */
    int scaled_width = 8 * scale;
    int scaled_height = 8 * scale;
    if (x < 0 || y < 0 || x + scaled_width > M5_LCD_H_RES || y + scaled_height > M5_LCD_V_RES) {
        return;
    }
    
    /* Convert ASCII character to font table index */
    int idx = 0;
    if (c >= ' ' && c <= 'z') {
        idx = c - ' ';
    }
    
    /* Render character pixel-by-pixel with scaling and transparency */
    const int char_width = 8;
    const int char_height = 8;
    
    for (int row = 0; row < char_height; row++) {
        uint8_t line = font8x8[idx][row];
        for (int col = 0; col < char_width; col++) {
            /* Only draw foreground pixels - background remains transparent */
            if (line & (0x80 >> col)) {
                /* Draw scaled pixel block (scale x scale pixels) */
                for (int sy = 0; sy < scale; sy++) {
                    for (int sx = 0; sx < scale; sx++) {
                        int px = x + col * scale + sx;
                        int py = y + row * scale + sy;
                        if (px < M5_LCD_H_RES && py < M5_LCD_V_RES) {
                            uint16_t pixel_color = color;
                            esp_lcd_panel_draw_bitmap(panel_handle, px, py, px + 1, py + 1, &pixel_color);
                        }
                    }
                }
            }
        }
    }
}

/**
 * @brief Print text string at normal scale
 * 
 * Convenience wrapper for scaled text printing with scale factor of 1.
 * Renders text using the built-in 8x8 font.
 * 
 * @param x Starting horizontal position
 * @param y Starting vertical position
 * @param text Null-terminated string to print
 * @param color Text color (RGB565)
 */
void m5stickc_plus2_display_print(int x, int y, const char *text, uint16_t color) {
    m5stickc_plus2_display_print_scaled(x, y, text, color, 1);
}

/**
 * @brief Print text string with scaling support
 * 
 * Renders text with specified scaling factor, supporting:
 * - Newline characters for explicit line breaks
 * - Automatic word wrapping at screen edge
 * - Transparent rendering (background shows through)
 * 
 * @param x Starting horizontal position
 * @param y Starting vertical position
 * @param text Null-terminated string to print (supports \n)
 * @param color Text color (RGB565)
 * @param scale Scaling factor (1=8x8, 2=16x16, etc.)
 */
void m5stickc_plus2_display_print_scaled(int x, int y, const char *text, uint16_t color, int scale) {
    if (panel_handle == NULL || text == NULL) return;
    
    int cursor_x = x;
    const char *p = text;
    int char_width = 8 * scale;
    int char_height = 8 * scale;
    int line_spacing = char_height + 2;  /* Small spacing between lines */
    
    while (*p) {
        /* Handle explicit newline characters */
        if (*p == '\n') {
            cursor_x = x;
            y += line_spacing;  /* Move to next line */
        } else {
            /* Render character with transparency */
            draw_char_scaled(cursor_x, y, *p, color, M5_COLOR_BLACK, scale);
            cursor_x += char_width;  /* Advance cursor by scaled character width */
            
            /* Automatic word wrapping at screen edge */
            if (cursor_x > M5_LCD_H_RES - char_width) {
                cursor_x = x;
                y += line_spacing;
            }
        }
        p++;
    }
}

/**
 * @brief Draw monochrome bitmap with transparency
 * 
 * Renders a 1-bit bitmap (1=foreground, 0=transparent) at the specified position.
 * Only foreground pixels are drawn, allowing background to show through.
 * Automatically clips to screen boundaries.
 * 
 * @param x Horizontal position
 * @param y Vertical position
 * @param width Bitmap width in pixels
 * @param height Bitmap height in pixels
 * @param bitmap Pointer to bitmap data (1 bit per pixel, row-major order)
 * @param color Foreground color (RGB565)
 * @param bg_color Background color (unused - transparent rendering)
 */
void m5stickc_plus2_display_draw_bitmap(int x, int y, int width, int height, const uint8_t *bitmap, uint16_t color, uint16_t bg_color) {
    if (panel_handle == NULL || bitmap == NULL) return;
    if (x >= M5_LCD_H_RES || y >= M5_LCD_V_RES) return;
    
    /* Clip bitmap to screen boundaries */
    int draw_width = (x + width > M5_LCD_H_RES) ? M5_LCD_H_RES - x : width;
    int draw_height = (y + height > M5_LCD_V_RES) ? M5_LCD_V_RES - y : height;
    if (draw_width <= 0 || draw_height <= 0) return;
    
    /* Render bitmap pixel-by-pixel for transparency support */
    int byte_width = (width + 7) / 8;  /* Bytes per row (padded to byte boundary) */
    for (int row = 0; row < draw_height; row++) {
        for (int col = 0; col < draw_width; col++) {
            int bitmap_byte_idx = row * byte_width + col / 8;
            int bit_idx = 7 - (col % 8);  /* MSB first bit ordering */
            bool pixel_set = (bitmap[bitmap_byte_idx] >> bit_idx) & 1;
            
            /* Only draw foreground pixels - background remains transparent */
            if (pixel_set) {
                uint16_t pixel_color = color;
                esp_lcd_panel_draw_bitmap(panel_handle, x + col, y + row, x + col + 1, y + row + 1, &pixel_color);
            }
        }
    }
}

/**
 * @brief Draw filled circle (simplified as square)
 * 
 * Currently implemented as a filled square for simplicity and performance.
 * Most UI elements in the camera remote use rectangular shapes anyway.
 * 
 * @param x Center horizontal position
 * @param y Center vertical position
 * @param radius Circle radius in pixels
 * @param color Fill color (RGB565)
 */
void m5stickc_plus2_display_fill_circle(int x, int y, int radius, uint16_t color) {
    if (panel_handle == NULL || radius <= 0) return;
    
    /* Simple implementation: draw filled square instead of circle
     * This avoids complex circle drawing algorithms and potential artifacts
     * Most UI elements in this application use rectangular shapes
     */
    int size = radius * 2;
    m5stickc_plus2_display_fill_rect(x - radius, y - radius, size, size, color);
}

/**
 * @brief Draw filled rectangle
 * 
 * Renders a solid-colored rectangle using DMA-capable memory for optimal
 * performance. Automatically clips to screen boundaries.
 * 
 * @param x Horizontal position
 * @param y Vertical position
 * @param width Rectangle width in pixels
 * @param height Rectangle height in pixels
 * @param color Fill color (RGB565)
 */
void m5stickc_plus2_display_fill_rect(int x, int y, int width, int height, uint16_t color) {
    if (panel_handle == NULL) return;
    if (x >= M5_LCD_H_RES || y >= M5_LCD_V_RES) return;
    
    /* Clip rectangle to screen boundaries */
    int draw_width = (x + width > M5_LCD_H_RES) ? M5_LCD_H_RES - x : width;
    int draw_height = (y + height > M5_LCD_V_RES) ? M5_LCD_V_RES - y : height;
    if (draw_width <= 0 || draw_height <= 0) return;
    
    /* Allocate DMA-capable buffer for hardware-accelerated transfer */
    size_t buffer_size = draw_width * draw_height * sizeof(uint16_t);
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate rectangle buffer");
        return;
    }
    
    /* Fill buffer with solid color (RGB565 format) */
    for (int i = 0; i < draw_width * draw_height; i++) {
        buffer[i] = color;
    }
    
    /* Transfer buffer to display using hardware acceleration */
    esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + draw_width, y + draw_height, buffer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to draw rectangle: %s", esp_err_to_name(ret));
    }
    
    heap_caps_free(buffer);
}