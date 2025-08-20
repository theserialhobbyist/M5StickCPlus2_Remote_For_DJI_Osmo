/*
 * M5StickC Plus2 Hardware Abstraction Layer Header
 */

#ifndef M5STICKC_PLUS2_HAL_H
#define M5STICKC_PLUS2_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_lcd_types.h"

/* M5StickC Plus2 GPIO Pin Definitions (ESP32-PICO-V3-02) */

/* Display pins (ST7789V2) - M5StickC Plus2 verified pins */
#define M5_LCD_MOSI_PIN     15  /* SPI MOSI (GPIO15) */
#define M5_LCD_SCLK_PIN     13  /* SPI CLK (GPIO13) */
#define M5_LCD_CS_PIN       5   /* Chip Select (GPIO5) */
#define M5_LCD_DC_PIN       14  /* Data/Command (GPIO14) - CHANGED */
#define M5_LCD_RST_PIN      12  /* Reset (GPIO12) - CHANGED */
#define M5_LCD_BL_PIN       27  /* Backlight GPIO pin for Plus2 */

/* Display resolution - M5StickC Plus2 in landscape mode */
#define M5_LCD_H_RES        240
#define M5_LCD_V_RES        135

/* Button pins - M5StickC Plus2 actual pins */
#define M5_BTN_A_PIN        37  /* Main button (Home) - Note: GPIO37 is input-only on ESP32 */
#define M5_BTN_B_PIN        39  /* Side button - Note: GPIO39 is input-only on ESP32 */
#define M5_BTN_PWR_PIN      35  /* Power button - Note: GPIO35 is input-only on ESP32 */

/* I2C pins (for IMU, RTC, PMU) */
#define M5_I2C_SDA_PIN      21
#define M5_I2C_SCL_PIN      22


/* Power control */
#define M5_PWR_EN_PIN       4   /* Power enable for peripherals */

/* LED pin (internal red LED) */
#define M5_LED_PIN          10

/* Buzzer pin */
#define M5_BUZZER_PIN       2

/* IR transmitter pin */
#define M5_IR_PIN           9

/* IMU (MPU6886) I2C address */
#define MPU6886_I2C_ADDR    0x68

/* RTC (BM8563) I2C address */
#define BM8563_I2C_ADDR     0x51

/* Note: M5StickC Plus2 does NOT have AXP192 PMU (Plus1 did, Plus2 doesn't) */

/* Function prototypes */

/**
 * @brief Initialize all M5StickC Plus2 hardware
 * @return ESP_OK on success, error code otherwise
 */
int m5stickc_plus2_init(void);

/**
 * @brief Initialize power management
 * @return ESP_OK on success, error code otherwise
 */
int m5stickc_plus2_power_init(void);

/**
 * @brief Initialize I2C bus for peripherals
 * @return ESP_OK on success, error code otherwise
 */
int m5stickc_plus2_i2c_init(void);

/**
 * @brief Initialize display
 * @return ESP_OK on success, error code otherwise
 */
int m5stickc_plus2_display_init(void);

/**
 * @brief Initialize buttons
 * @return ESP_OK on success, error code otherwise
 */
int m5stickc_plus2_buttons_init(void);

/**
 * @brief Check if button A is pressed
 * @return true if pressed, false otherwise
 */
bool m5stickc_plus2_button_a_pressed(void);

/**
 * @brief Check if button B is pressed
 * @return true if pressed, false otherwise
 */
bool m5stickc_plus2_button_b_pressed(void);

/**
 * @brief Check if power button is pressed
 * @return true if pressed, false otherwise
 */
bool m5stickc_plus2_button_pwr_pressed(void);

/**
 * @brief Set display brightness
 * @param brightness Brightness level (0-255)
 */
void m5stickc_plus2_display_set_brightness(uint8_t brightness);

/**
 * @brief Get display panel handle
 * @return Display panel handle
 */
esp_lcd_panel_handle_t m5stickc_plus2_get_display_handle(void);

/**
 * @brief Clear display with specified color
 * @param color 16-bit RGB565 color
 */
void m5stickc_plus2_display_clear(uint16_t color);

/**
 * @brief Print text on display
 * @param x X coordinate
 * @param y Y coordinate
 * @param text Text to display
 * @param color Text color (RGB565)
 */
void m5stickc_plus2_display_print(int x, int y, const char *text, uint16_t color);

/**
 * @brief Print scaled text on display
 * @param x X coordinate
 * @param y Y coordinate
 * @param text Text to display
 * @param color Text color (RGB565)
 * @param scale Scale factor (1=normal, 2=double size, etc.)
 */
void m5stickc_plus2_display_print_scaled(int x, int y, const char *text, uint16_t color, int scale);

/**
 * @brief Draw a bitmap using proper buffer-based approach
 * @param x X coordinate
 * @param y Y coordinate
 * @param width Bitmap width
 * @param height Bitmap height
 * @param bitmap Bitmap data
 * @param color Color for set bits
 * @param bg_color Background color for unset bits
 */
void m5stickc_plus2_display_draw_bitmap(int x, int y, int width, int height, const uint8_t *bitmap, uint16_t color, uint16_t bg_color);

/**
 * @brief Draw a filled circle using buffer-based approach
 * @param x Center X coordinate
 * @param y Center Y coordinate
 * @param radius Circle radius
 * @param color Fill color (RGB565)
 */
void m5stickc_plus2_display_fill_circle(int x, int y, int radius, uint16_t color);

/**
 * @brief Draw a filled rectangle
 * @param x X coordinate
 * @param y Y coordinate
 * @param width Rectangle width
 * @param height Rectangle height
 * @param color Fill color (RGB565)
 */
void m5stickc_plus2_display_fill_rect(int x, int y, int width, int height, uint16_t color);

/* Color definitions (RBG565 - green and blue are swapped on this display) */
#define M5_COLOR_BLACK      0x0000
#define M5_COLOR_WHITE      0xFFFF
#define M5_COLOR_RED        0xF800  // R=31, B=0, G=0
#define M5_COLOR_GREEN      0x001F  // R=0, B=0, G=31 (swapped with blue)
#define M5_COLOR_BLUE       0x07E0  // R=0, B=63, G=0 (swapped with green)
#define M5_COLOR_YELLOW     0xF81F  // Red + Green (R=31, B=0, G=31)
#define M5_COLOR_CYAN       0x07FF  // Blue + Green (R=0, B=63, G=31)
#define M5_COLOR_MAGENTA    0xFFE0  // Red + Blue (R=31, B=63, G=0)
#define M5_COLOR_ORANGE     0xF81F  // Similar to yellow in this color space
#define M5_COLOR_PURPLE     0x8010  // Mix of red and blue
#define M5_COLOR_DARKGREY   0x39C6  // Dark grey for indicators
#define M5_COLOR_GREY       0x7BEF  // Medium grey for text

#endif /* M5STICKC_PLUS2_HAL_H */