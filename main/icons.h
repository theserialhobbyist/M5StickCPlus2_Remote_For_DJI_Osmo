/*
 * icons.h
 * 32x32 bitmap icon data for DJI Camera Remote UI
 */

#ifndef ICONS_H
#define ICONS_H

#include <stdint.h>

// Icon declarations - defined in icons.c
extern const uint8_t bluetooth_icon[128];
extern const uint8_t record_icon[128];
extern const uint8_t mode_icon[128];
extern const uint8_t sleep_icon[128];
extern const uint8_t key_icon[128];

// Icon color definitions (RBG565 - swapped green/blue for this display)
#define ICON_BLUE    0x07E0   // Blue for Connect (was green in RGB)
#define ICON_RED     0xF800   // Red for Record (stays same)
#define ICON_ORANGE  0xFC00   // Orange for Mode Switch
#define ICON_YELLOW  0xF81F   // Yellow for Quick Switch (red + green)
#define ICON_WHITE   0xFFFF   // White default
#define ICON_CYAN    0x07FF   // Cyan alternate

// Color definitions are now in m5stickc_plus2_hal.h to avoid conflicts

#endif // ICONS_H