/*
 * ui.h
 * Display and user interface functions for DJI Camera Remote
 * Inspired by M5StickC Insta360 Remote UI design
 */

#ifndef UI_H
#define UI_H

#include <stdint.h>
#include <stdbool.h>
#include "icons.h"
#include "connect_logic.h"

// UI Screen definitions
typedef enum {
    SCREEN_CONNECT = 0,         // Connect to camera
    SCREEN_SHUTTER,             // Camera shutter (start/stop recording or photo)
    SCREEN_MODE,                // Switch camera mode
    SCREEN_SLEEP,               // Sleep mode
    SCREEN_WAKE,                // Wake camera from sleep
    SCREEN_COUNT                // Total number of screens
} ui_screen_t;

// UI state structure
typedef struct {
    ui_screen_t current_screen;
    bool display_needs_update;
    bool is_plus2_device;
    float scale_factor;
    int scaled_text_size;
} ui_state_t;

// Screen layout structure for different device sizes
typedef struct {
    int icon_x, icon_y;           // Icon position
    int text_x, text_y;           // Text position  
    int status_x, status_y;       // Connection status position
    int dots_y, dots_spacing, dots_start_x;  // Page indicator dots
    int instruct_x, instruct_y;   // Instructions position
    int connection_radius;        // Status circle radius
} screen_layout_t;

// Global UI state
extern ui_state_t g_ui_state;
extern screen_layout_t g_layout;

// UI Function declarations
void ui_init(void);
void ui_detect_device_and_set_scale(void);
void ui_auto_connect_on_startup(void);
int ui_attempt_background_reconnection(void);
void ui_update_display(void);
void ui_next_screen(void);
void ui_execute_current_screen(void);
void ui_show_message(const char* message, uint16_t color, int duration_ms);
void ui_show_not_connected_message(void);
void ui_draw_bitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
void ui_draw_connection_status(void);
int ui_get_text_width(const char* text, int text_size);
void ui_process_pending_gpio_actions(void);

// Screen function prototypes
void ui_screen_connect(void);
void ui_screen_shutter(void);
void ui_screen_mode(void);
void ui_screen_sleep(void);
void ui_screen_wake(void);

// Screen information structure
typedef struct {
    const char* name;
    const uint8_t* icon;
    uint16_t color;
    void (*execute_func)(void);
} screen_info_t;

// Screen definitions
static const screen_info_t screen_info[SCREEN_COUNT] = {
    {"CONNECT", bluetooth_icon, ICON_BLUE, ui_screen_connect},
    {"SHUTTER", record_icon, ICON_RED, ui_screen_shutter},
    {"MODE", mode_icon, ICON_ORANGE, ui_screen_mode},
    {"SLEEP", sleep_icon, ICON_CYAN, ui_screen_sleep},
    {"WAKE", key_icon, ICON_YELLOW, ui_screen_wake}
};

#endif // UI_H