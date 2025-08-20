#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"

#include "enums_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "dji_protocol_data_structures.h"

static const char *TAG = "LOGIC_STATUS";

// Global variables to store various camera status information
uint8_t current_camera_mode = 0;
uint8_t current_camera_status = 0;
uint8_t current_video_resolution = 0;
uint8_t current_fps_idx = 0;
uint8_t current_eis_mode = 0;
uint8_t current_user_mode = 0;
uint8_t current_camera_mode_next_flag = 0;
uint16_t current_record_time = 0;
uint16_t current_timelapse_interval = 0;
bool camera_status_initialized = false;

// Global variables for new camera status push command frame
uint8_t current_type_mode_name = 0;
uint8_t current_mode_name_length = 0;
uint8_t current_mode_name[20] = {0};
uint8_t current_type_mode_param = 0;
uint8_t current_mode_param_length = 0;
uint8_t current_mode_param[20] = {0};


/**
 * @brief Check if camera is recording
 * 
 * Check if camera is in recording or pre-recording state, and status is initialized.
 * 
 * @return bool Returns true if camera is recording, false otherwise
 */
bool is_camera_recording() {
    if ((current_camera_status == CAMERA_STATUS_PHOTO_OR_RECORDING || current_camera_status == CAMERA_STATUS_PRE_RECORDING) && camera_status_initialized) {
        return true;
    }
    return false;
}

/**
 * @brief Print current camera status (partial status, other status can be printed as needed)
 * 
 * Print camera mode, status, resolution, frame rate and electronic image stabilization mode.
 */
void print_camera_status() {
    if (!camera_status_initialized) {
        ESP_LOGW(TAG, "Camera status has not been initialized.");
        return;
    }

    const char *mode_str = camera_mode_to_string((camera_mode_t)current_camera_mode);
    const char *status_str = camera_status_to_string((camera_status_t)current_camera_status);
    const char *resolution_str = video_resolution_to_string((video_resolution_t)current_video_resolution);
    const char *fps_str = fps_idx_to_string((fps_idx_t)current_fps_idx);
    const char *eis_str = eis_mode_to_string((eis_mode_t)current_eis_mode);

    ESP_LOGI(TAG, "[1D02] =========== Camera Status Push ===========");
    ESP_LOGI(TAG, "  Mode: %s", mode_str);
    ESP_LOGI(TAG, "  Status: %s", status_str);
    ESP_LOGI(TAG, "  Resolution: %s (value: %d)", resolution_str, current_video_resolution);
    ESP_LOGI(TAG, "  FPS: %s", fps_str);
    ESP_LOGI(TAG, "  EIS: %s", eis_str);
    ESP_LOGI(TAG, "  User mode: %d", current_user_mode);
    ESP_LOGI(TAG, "  Camera mode next flag: %d", current_camera_mode_next_flag);
    ESP_LOGI(TAG, "  Record time: %d", current_record_time);
    ESP_LOGI(TAG, "  Timelapse interval: %d", current_timelapse_interval);
    ESP_LOGI(TAG, "=================================================");
}

/**
 * @brief Subscribe to camera status
 * 
 * @param push_mode Subscription mode
 * @param push_freq Subscription frequency
 * @return int Returns 0 on success, -1 on failure
 */
int subscript_camera_status(uint8_t push_mode, uint8_t push_freq) {
    ESP_LOGI(TAG, "Subscribing to Camera Status with push_mode: %d, push_freq: %d", push_mode, push_freq);

    if (connect_logic_get_state() != PROTOCOL_CONNECTED) {
        ESP_LOGE(TAG, "Protocol connection to the camera failed. Current connection state: %d", connect_logic_get_state());
        return -1;
    }

    uint16_t seq = generate_seq();

    camera_status_subscription_command_frame command_frame = {
        .push_mode = push_mode,
        .push_freq = push_freq,
        .reserved = {0, 0, 0, 0}
    };

    send_command(0x1D, 0x05, CMD_NO_RESPONSE, &command_frame, seq, 5000);

    return 0;
}

/**
 * @brief Update camera state machine (callback function)
 * 
 * Process and update various camera states, check for state changes and print updated information.
 * 
 * @param data Input camera status data
 */
void update_camera_state_handler(void *data) {
    if (!data) {
        ESP_LOGE(TAG, "logic_update_camera_state: Received NULL data.");
        return;
    }

    const camera_status_push_command_frame *parsed_data = (const camera_status_push_command_frame *)data;

    bool state_changed = false;

    // Check and update camera mode
    if (current_camera_mode != parsed_data->camera_mode) {
        current_camera_mode = parsed_data->camera_mode;
        ESP_LOGI(TAG, "Camera mode updated to: %d", current_camera_mode);
        state_changed = true;
    }

    // Check and update camera status
    if (current_camera_status != parsed_data->camera_status) {
        current_camera_status = parsed_data->camera_status;
        ESP_LOGI(TAG, "Camera status updated to: %d", current_camera_status);
        state_changed = true;
    }

    // Check and update video resolution
    if (current_video_resolution != parsed_data->video_resolution) {
        current_video_resolution = parsed_data->video_resolution;
        ESP_LOGI(TAG, "Video resolution updated to: %d", current_video_resolution);
        state_changed = true;
    }

    // Check and update frame rate
    if (current_fps_idx != parsed_data->fps_idx) {
        current_fps_idx = parsed_data->fps_idx;
        ESP_LOGI(TAG, "FPS index updated to: %d", current_fps_idx);
        state_changed = true;
    }

    // Check and update electronic image stabilization mode
    if (current_eis_mode != parsed_data->eis_mode) {
        current_eis_mode = parsed_data->eis_mode;
        ESP_LOGI(TAG, "EIS mode updated to: %d", current_eis_mode);
        state_changed = true;
    }

    // Check and update user mode
    if (current_user_mode != parsed_data->user_mode) {
        current_user_mode = parsed_data->user_mode;
        ESP_LOGI(TAG, "User mode updated to: %d", current_user_mode);
        state_changed = true;
    }

    // Check and update camera mode next flag
    if (current_camera_mode_next_flag != parsed_data->camera_mode_next_flag) {
        current_camera_mode_next_flag = parsed_data->camera_mode_next_flag;
        ESP_LOGI(TAG, "Camera mode next flag updated to: %d", current_camera_mode_next_flag);
        state_changed = true;
    }

    // Check and update record time
    if (current_record_time != parsed_data->record_time) {
        current_record_time = parsed_data->record_time;
        ESP_LOGI(TAG, "Record time updated to: %d", current_record_time);
        state_changed = true;
    }

    // Check and update timelapse interval
    if (current_timelapse_interval != parsed_data->timelapse_interval) {
        current_timelapse_interval = parsed_data->timelapse_interval;
        ESP_LOGI(TAG, "Timelapse interval updated to: %d", current_timelapse_interval);
        state_changed = true;
    }

    // If status not initialized, mark as initialized
    if (!camera_status_initialized) {
        camera_status_initialized = true;
        ESP_LOGI(TAG, "Camera state fully updated and marked as initialized.");
        state_changed = true;  // Force status print as this is initialization
    }

    // If state changed or first initialization, print current camera status
    if (state_changed) {
        print_camera_status();
    }

    free(data);
}

void update_new_camera_state_handler(void *data) {
    if (!data) {
        ESP_LOGE(TAG, "update_new_camera_state_handler: Received NULL data.");
        return;
    }

    const new_camera_status_push_command_frame *parsed_data = (const new_camera_status_push_command_frame *)data;

    ESP_LOGI(TAG, "[1D06] =============Osmo360============");

    // Update type_mode_name
    current_type_mode_name = parsed_data->type_mode_name;
    ESP_LOGI(TAG, "[1D06] Camera mode name type: 0x%02X", current_type_mode_name);
    
    // Update mode_name_length
    current_mode_name_length = parsed_data->mode_name_length;
    ESP_LOGI(TAG, "[1D06] Mode name length: %d", current_mode_name_length);
    
    // Update mode_name array
    memcpy(current_mode_name, parsed_data->mode_name, 20);

    // Ensure null termination for safe string printing
    char mode_name_str[21] = {0};
    memcpy(mode_name_str, parsed_data->mode_name, 20);
    ESP_LOGI(TAG, "[1D06] Mode name: %s", mode_name_str);
    
    // Update type_mode_param
    current_type_mode_param = parsed_data->type_mode_param;
    ESP_LOGI(TAG, "[1D06] Camera mode parameter type: 0x%02X", current_type_mode_param);
    
    // Update mode_param_length
    current_mode_param_length = parsed_data->mode_param_length;
    ESP_LOGI(TAG, "[1D06] Mode parameter length: %d", current_mode_param_length);
    
    // Update mode_param array
    memcpy(current_mode_param, parsed_data->mode_param, 20);

    // Ensure null termination for safe string printing
    char mode_param_str[21] = {0};
    memcpy(mode_param_str, parsed_data->mode_param, 20);
    ESP_LOGI(TAG, "[1D06] Mode parameters: %s", mode_param_str);

    ESP_LOGI(TAG, "[1D06] ================================");

    free(data);
}