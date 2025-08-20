#ifndef DJI_PROTOCOL_DATA_STRUCTURES_H
#define DJI_PROTOCOL_DATA_STRUCTURES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Define command and response frame structures
typedef struct __attribute__((packed)) {
    uint32_t device_id;            // Device ID
    uint8_t mode;                  // Mode, refer to camera status in camera status push
    uint8_t reserved[4];           // Reserved field
} camera_mode_switch_command_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t ret_code;              // Return code: 0 for success, non-zero for failure
    uint8_t reserved[4];           // Reserved field
} camera_mode_switch_response_frame_t;

typedef struct __attribute__((packed)) {
    uint16_t ack_result;           // Acknowledgment result
    uint8_t product_id[16];        // Product ID, e.g., DJI-RS3
    uint8_t sdk_version[];         // SDK version data (flexible array)
} version_query_response_frame_t;

typedef struct __attribute__((packed)) {
    uint32_t device_id;            // Device ID
    uint8_t record_ctrl;           // Recording control: 0 - Start, 1 - Stop
    uint8_t reserved[4];           // Reserved field
} record_control_command_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t ret_code;              // Return code (refer to common return codes)
} record_control_response_frame_t;


typedef struct __attribute__((packed)) {
    uint32_t device_id;            // Device ID
    uint8_t mac_addr_len;          // MAC address length
    int8_t mac_addr[16];           // MAC address
    uint32_t fw_version;           // Firmware version
    uint8_t conidx;                // Reserved field
    uint8_t verify_mode;           // Verification mode
    uint16_t verify_data;          // Verification data or result
    uint8_t reserved[4];           // Reserved field
} connection_request_command_frame;

typedef struct __attribute__((packed)) {
    uint32_t device_id;            // Device ID
    uint8_t ret_code;              // Return code
    uint8_t reserved[4];           // Reserved field
} connection_request_response_frame;

typedef struct __attribute__((packed)) {
    uint8_t push_mode;             // Push mode: 0-Off, 1-Single, 2-Periodic, 3-Periodic+Status change
    uint8_t push_freq;             // Push frequency in 0.1Hz, only 20 is allowed
    uint8_t reserved[4];           // Reserved field
} camera_status_subscription_command_frame;

typedef struct __attribute__((packed)) {
    uint8_t camera_mode;           // Camera mode: 0x00-Slow Motion, 0x01-Video, 0x02-Timelapse, 0x05-Photo, 0x0A-Hyperlapse, 0x1A-Live stream, 0x23-UVC live stream, 0x28-Low light video (Super night scene in Osmo Action 5 Pro), 0x34-Subject follow, others-Use new protocol (refer to New camera status push 1D06)
    uint8_t camera_status;         // Camera status: 0x00-Screen off, 0x01-Live view (including screen on but not recording), 0x02-Playback, 0x03-Photo/video in progress, 0x05-Pre-recording
    uint8_t video_resolution;      // Video resolution: 10-1080P, 16-4K 16:9, 45-2.7K 16:9, 66-1080P 9:16, 67-2.7K 9:16, 95-2.7K 4:3, 103-4K 4:3, 109-4K 9:16; Photo format (Osmo Action): 4-L, 3-M; Photo format (Osmo 360): 4-Ultra Wide 30MP, 3-Wide 20MP, 2-Standard 12MP
    uint8_t fps_idx;               // Frame rate: 1-24fps, 2-25fps, 3-30fps, 4-48fps, 5-50fps, 6-60fps, 10-100fps, 7-120fps, 19-200fps, 8-240fps; In Slow Motion mode: multiplier = fps/30; In photo mode: burst count (1-single photo, >1-burst count)
    uint8_t eis_mode;              // Electronic image stabilization mode: 0-Off, 1-RS, 2-HS, 3-RS+, 4-HB
    uint16_t record_time;          // Current recording time (including pre-recording duration) in seconds; In burst mode: burst time limit in milliseconds
    uint8_t fov_type;              // FOV type, reserved field
    uint8_t photo_ratio;           // Photo aspect ratio: 0-4:3, 1-16:9
    uint16_t real_time_countdown;  // Real-time countdown in seconds
    uint16_t timelapse_interval;   // In Timelapse mode: shooting interval in 0.1s (e.g., for 0.5s interval, value is 5); In Hyperlapse mode: shooting rate (0 for Auto option)
    uint16_t timelapse_duration;   // Time-lapse recording duration in seconds
    uint32_t remain_capacity;      // Remaining SD card capacity in MB
    uint32_t remain_photo_num;     // Remaining number of photos
    uint32_t remain_time;          // Remaining recording time in seconds
    uint8_t user_mode;             // User mode (invalid values treated as 0): 0-General mode, 1-Custom mode 1, 2-Custom mode 2, 3-Custom mode 3, 4-Custom mode 4, 5-Custom mode 5
    uint8_t power_mode;            // Power mode: 0-Normal working mode, 3-Sleep mode
    uint8_t camera_mode_next_flag; // Pre-switch flag: In pre-switch mode (e.g., QS), only shows mode name and icon without specific parameters. Indicates target mode to switch to; if not in pre-switch mode, equals camera_mode
    uint8_t temp_over;             // Camera error status: 0-Normal temperature, 1-Temperature warning (can record but high temperature), 2-High temperature (cannot record), 3-Overheating (will shut down)
    uint32_t photo_countdown_ms;   // Photo countdown parameter in milliseconds (remote controller converts to 0.5s, 1s, 2s, 3s, 5s, 10s options)
    uint16_t loop_record_sends;    // Loop recording duration in seconds (remote controller shows as off, max, 5m, 20m, 1h options, where off=0, max=65535)
    uint8_t camera_bat_percentage; // Camera battery percentage: 0-100%
} camera_status_push_command_frame;

typedef struct __attribute__((packed)) {
    uint8_t type_mode_name;        // Camera mode name type: 0x01
    uint8_t mode_name_length;      // Mode name length
    uint8_t mode_name[20];         // Mode name, ASCII code, max 20 bytes
    uint8_t type_mode_param;       // Camera mode parameter type: 0x02
    uint8_t mode_param_length;     // Mode parameter length, ASCII code, max 20 bytes
    uint8_t mode_param[20];        // Mode parameters
} new_camera_status_push_command_frame;

typedef struct __attribute__((packed)) {
    uint8_t key_code;              // Key code
    uint8_t mode;                  // Report mode: 0x00-Report key press/release status, 0x01-Report key events
    uint16_t key_value;            // Key event value: For mode 0: 0x00-Key pressed, 0x01-Key released; For mode 1: 0x00-Short press, 0x01-Long press, 0x02-Double click, 0x03-Triple click, 0x04-Quadruple click
} key_report_command_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t ret_code;             // Return code (refer to common return codes)
} key_report_response_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t power_mode;           // Power mode, 0 for normal mode, 3 for sleep mode
} camera_power_mode_switch_command_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t ret_code;             // Return code: 0 for success, non-zero for failure
} camera_power_mode_switch_response_frame_t;

#endif