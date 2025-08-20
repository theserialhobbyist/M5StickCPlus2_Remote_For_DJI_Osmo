#ifndef STATUS_LOGIC_H
#define STATUS_LOGIC_H

#include <stdint.h>

// Declaration of global variables for camera status (more can be added later)
extern uint8_t current_camera_mode;
extern uint8_t current_camera_status;
extern uint8_t current_video_resolution;
extern uint8_t current_fps_idx;
extern uint8_t current_eis_mode;
extern bool camera_status_initialized;

bool is_camera_recording();

void print_camera_status();

int subscript_camera_status(uint8_t push_mode, uint8_t push_freq);

void update_camera_state_handler(void *data);

void update_new_camera_state_handler(void *data);

#endif