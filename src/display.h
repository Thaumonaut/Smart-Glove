#ifndef DISPLAY_H
#define DISPLAY_H

#include "lvgl.h"

// Display initialization
void display_init(void);

// UI elements
void display_update_accel(int16_t x, int16_t y, int16_t z);
void display_update_gyro(int16_t x, int16_t y, int16_t z);
void display_update_bluetooth_state(bool connected, bool playing, bool in_call,
                                     const char *track_title, const char *track_artist);
void display_show_message(const char *message);

// Health display (HR/SpO2 or raw if unavailable)
void display_update_health(bool valid, float heart_rate, float spo2,
                           uint32_t red_led, uint32_t ir_led);

// Gesture display (pass preformatted text e.g., "Call Me")
void display_update_gesture(const char *gesture_text);

// APDS9960 proximity/gesture display
void display_update_apds_proximity(uint8_t proximity);  // 0-255
void display_update_apds_gesture(const char *gesture_dir);

// Optional: change hardware offsets at runtime if module needs manual tuning
void display_set_offsets(int x_offset, int y_offset);

// LVGL timer handler (call from main loop)
void display_lvgl_tick(void);

#endif // DISPLAY_H
