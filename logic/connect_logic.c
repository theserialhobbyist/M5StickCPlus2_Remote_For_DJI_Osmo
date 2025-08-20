/*
 * DJI Camera Remote Control - Connection Logic Layer
 * 
 * This file implements the complete connection management system for DJI camera
 * communication, handling both BLE and protocol-level connections.
 * 
 * Connection Flow:
 * 1. BLE Initialization: Set up ESP32 BLE stack
 * 2. Device Discovery: Scan for and connect to target camera
 * 3. Service Discovery: Find DJI communication characteristics
 * 4. Protocol Handshake: Establish DJI protocol connection
 * 5. Maintenance: Handle disconnections and reconnection attempts
 * 
 * State Management:
 * - BLE_NOT_INIT: Initial state before BLE initialization
 * - BLE_INIT_COMPLETE: BLE ready, no connection
 * - BLE_SEARCHING: Actively scanning for cameras
 * - BLE_CONNECTED: BLE link established, protocol pending
 * - PROTOCOL_CONNECTED: Full connection, ready for commands
 * - BLE_DISCONNECTING: Graceful disconnection in progress
 * 
 * Connection Types:
 * - Pairing (verify_mode=1): Initial camera registration
 * - Reconnection (verify_mode=0): Automatic connection to known camera
 * - Wake-up: BLE advertising to rouse sleeping cameras
 * 
 * Error Handling:
 * - Automatic reconnection attempts on unexpected disconnection
 * - Timeout management for all connection phases
 * - State restoration on connection failures
 * 
 * Hardware: M5StickC Plus2 with ESP32 BLE capabilities
 * Protocol: DJI proprietary communication protocol
 * 
 * Based on DJI SDK implementation
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ble.h"
#include "data.h"
#include "enums_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "status_logic.h"
#include "dji_protocol_data_structures.h"

/* Logging tag for ESP_LOG functions */
#define TAG "LOGIC_CONNECT"

/* Global connection state tracking
 * Manages the current state of BLE and protocol connections
 * Used throughout the system to determine available operations
 */
static connect_state_t connect_state = BLE_NOT_INIT;

/**
 * @brief Get current connection state
 * 
 * Returns the current state of the connection system, allowing other
 * components to determine what operations are available and respond
 * appropriately to connection status changes.
 * 
 * @return connect_state_t Current connection state
 */
connect_state_t connect_logic_get_state(void) {
    return connect_state;
}

/**
 * @brief Handle camera disconnection events
 * 
 * Callback function triggered when the BLE connection to the camera is lost.
 * Implements sophisticated disconnection handling based on current state:
 * 
 * - Expected disconnections: Clean state reset
 * - Unexpected disconnections: Automatic reconnection attempt
 * - Failed reconnections: Graceful fallback to disconnected state
 * 
 * The function attempts one automatic reconnection for unexpected disconnections
 * to maintain seamless operation during temporary connection issues.
 */
void receive_camera_disconnect_handler() {
    switch (connect_state) {
        case BLE_SEARCHING:
            /* Already searching - no action needed */
            break;
        case BLE_INIT_COMPLETE:
            ESP_LOGI(TAG, "Already in DISCONNECTED state.");
            break;
        case BLE_DISCONNECTING: {
            ESP_LOGI(TAG, "Normal disconnection process.");
            /* Expected disconnection - clean state reset */
            connect_state = BLE_INIT_COMPLETE;
            camera_status_initialized = false;
            ESP_LOGI(TAG, "Current state: DISCONNECTED.");
            break;
        }
        case BLE_CONNECTED:
        case PROTOCOL_CONNECTED:
        default: {
            ESP_LOGW(TAG, "Unexpected disconnection from state: %d, attempting reconnection...", connect_state);
            
            /* Unexpected disconnection - attempt automatic reconnection */
            bool reconnected = false;
            ESP_LOGI(TAG, "Reconnection attempt...");
            if (connect_logic_ble_connect(true) == ESP_OK) {
                /* Wait up to 30 seconds for reconnection to complete */
                for (int j = 0; j < 300; j++) {
                    if (s_ble_profile.connection_status.is_connected) {
                        ESP_LOGI(TAG, "Reconnection successful");
                        reconnected = true;
                        return;  /* Successful reconnection - maintain current operation */
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }

            if (!reconnected) {
                ESP_LOGE(TAG, "Reconnection failed after 1 attempts");
                /* Reconnection failed - perform clean disconnection */
                connect_state = BLE_INIT_COMPLETE;
                camera_status_initialized = false;
                ble_disconnect();
                ESP_LOGI(TAG, "Current state: DISCONNECTED.");
            }
            break;
        }
    }
}

/**
 * @brief Initialize BLE subsystem for camera communication
 * 
 * Performs one-time initialization of the ESP32 BLE stack and prepares
 * the system for camera connections. This must be called before any
 * connection attempts.
 * 
 * Initialization includes:
 * - ESP32 BLE controller and host setup
 * - GATT client profile registration
 * - Service and characteristic UUID configuration
 * - Connection parameter setup
 * 
 * @return 0 on success, -1 on failure
 */
int connect_logic_ble_init() {
    esp_err_t ret;

    /* Initialize ESP32 BLE stack with DJI camera service configuration */
    ret = ble_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE, error: %s", esp_err_to_name(ret));
        return -1;
    }

    connect_state = BLE_INIT_COMPLETE;
    ESP_LOGI(TAG, "BLE init successfully");
    return 0;
}

/**
 * @brief Connect to BLE device
 * 
 * Execute the following steps: set callbacks, start scanning and attempt connection, wait for connection completion and characteristic handle discovery.
 * 
 * If connection fails, returns error and resets connection state.
 * 
 * @return int Returns 0 on success, -1 on failure
 */
int connect_logic_ble_connect(bool is_reconnecting) {
    connect_state = BLE_SEARCHING;

    esp_err_t ret;

    /* 1. Set a global Notify callback for receiving remote data and protocol parsing */
    ble_set_notify_callback(receive_camera_notify_handler);
    ble_set_state_callback(receive_camera_disconnect_handler);

    /* 2. Start scanning and attempt connection */
    ble_set_reconnecting(is_reconnecting);
    ret = ble_start_scanning_and_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start scanning and connect, error: 0x%x", ret);
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    /* 3. Wait up to 30 seconds to ensure BLE connection success */
    ESP_LOGI(TAG, "Waiting up to 10s for BLE to connect...");
    bool connected = false;
    for (int i = 0; i < 100; i++) { // 300 * 100ms = 30s
        if (s_ble_profile.connection_status.is_connected) {
            ESP_LOGI(TAG, "BLE connected successfully");
            connected = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!connected) {
        ESP_LOGW(TAG, "BLE connection timed out");
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    /* 4. Wait for characteristic handle discovery completion (up to 30 seconds) */
    ESP_LOGI(TAG, "Waiting up to 10s for characteristic handles discovery...");
    bool handles_found = false;
    for (int i = 0; i < 100; i++) { // 300 * 100ms = 30s
        if (s_ble_profile.handle_discovery.notify_char_handle_found && 
            s_ble_profile.handle_discovery.write_char_handle_found) {
            ESP_LOGI(TAG, "Required characteristic handles found");
            handles_found = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!handles_found) {
        ESP_LOGW(TAG, "Characteristic handles not found within timeout");
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    /* 5. Register notification */
    ret = ble_register_notify(s_ble_profile.conn_id, s_ble_profile.notify_char_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register notify, error: %s", esp_err_to_name(ret));
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    // Update state to BLE connected
    connect_state = BLE_CONNECTED;

    // Delay RGB light display
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "BLE connect successfully");
    return 0;
}

/**
 * @brief Disconnect BLE connection
 * 
 * Attempt to disconnect from BLE device.
 * 
 * @return int Returns 0 on success, -1 on failure
 */
int connect_logic_ble_disconnect(void) {
    connect_state_t old_state = connect_state;
    connect_state = BLE_DISCONNECTING;
    
    ESP_LOGI(TAG, "Disconnecting camera...");

    // Call BLE layer's ble_disconnect function
    esp_err_t ret = ble_disconnect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect camera, BLE error: %s", esp_err_to_name(ret));
        connect_state = old_state;
        return -1;
    }

    ESP_LOGI(TAG, "Camera disconnected successfully");
    return 0;
}

/**
 * @brief Establish DJI protocol connection with camera
 * 
 * Performs the complete DJI protocol handshake sequence over the established
 * BLE connection. This involves a complex bidirectional authentication process
 * that varies depending on whether this is a new pairing or reconnection.
 * 
 * Protocol Handshake Sequence:
 * 1. Send connection request with device credentials
 * 2. Handle camera response (may be response or command frame)
 * 3. Wait for camera's connection command with verification
 * 4. Send final connection response to complete handshake
 * 
 * Verification Modes:
 * - verify_mode=0: Reconnection to previously paired camera
 * - verify_mode=1: New pairing requiring camera-side confirmation
 * - verify_mode=2: Camera verification response
 * 
 * @param device_id Unique device identifier for this remote
 * @param mac_addr_len Length of MAC address (typically 6)
 * @param mac_addr Device MAC address for protocol identification
 * @param fw_version Firmware version for compatibility checking
 * @param verify_mode Authentication mode (0=reconnect, 1=pair)
 * @param verify_data Random verification code for security
 * @param camera_reserved Camera-specific identifier
 * @return 0 on successful protocol connection, -1 on failure
 */
int connect_logic_protocol_connect(uint32_t device_id, uint8_t mac_addr_len, const int8_t *mac_addr,
                                   uint32_t fw_version, uint8_t verify_mode, uint16_t verify_data,
                                   uint8_t camera_reserved) {
    ESP_LOGI(TAG, "%s: Starting protocol connection", __FUNCTION__);
    uint16_t seq = generate_seq();

    /* Construct DJI protocol connection request frame */
    connection_request_command_frame connection_request = {
        .device_id = device_id,
        .mac_addr_len = mac_addr_len,
        .fw_version = fw_version,
        .verify_mode = verify_mode,
        .verify_data = verify_data,
    };
    memcpy(connection_request.mac_addr, mac_addr, mac_addr_len);


    // STEP1: Send connection request command to camera
    ESP_LOGI(TAG, "Sending connection request to camera...");
    CommandResult result = send_command(0x00, 0x19, CMD_WAIT_RESULT, &connection_request, seq, 1000);

    /**** Connection issue: camera may return either response frame or command frame ****/

    if (result.structure == NULL) {
        // If a command frame is sent, execute this block of code

        // Directly call data_wait_for_result_by_cmd(0x00, 0x19, 30000, &received_seq, &parse_result, &parse_result_length);
        
        // If != OK, it means no message was received, timeout occurred
        
        // Otherwise, GOTO wait_for_camera_command label
        void *parse_result = NULL;
        size_t parse_result_length = 0;
        uint16_t received_seq = 0;
        esp_err_t ret = data_wait_for_result_by_cmd(0x00, 0x19, 1000, &received_seq, &parse_result, &parse_result_length);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Timeout or error waiting for camera connection command, GOTO Failed.");
            connect_logic_ble_disconnect();
            return -1;
        } else {
            // If data is received, skip parsing camera response and directly enter STEP3
            goto wait_for_camera_command;
        }
    }

    // STEP2: Parse the response returned from camera
    connection_request_response_frame *response = (connection_request_response_frame *)result.structure;
    if (response->ret_code != 0) {
        ESP_LOGE(TAG, "Connection handshake failed: unexpected response from camera, ret_code: %d", response->ret_code);
        free(response);
        connect_logic_ble_disconnect();
        return -1;
    }

    ESP_LOGI(TAG, "Handshake successful, waiting for the camera to actively send the connection command frame...");
    free(response);

    // STEP3: Wait for camera to send connection request
wait_for_camera_command:
    void *parse_result = NULL;
    size_t parse_result_length = 0;
    uint16_t received_seq = 0;
    esp_err_t ret = data_wait_for_result_by_cmd(0x00, 0x19, 30000, &received_seq, &parse_result, &parse_result_length);

    if (ret != ESP_OK || parse_result == NULL) {
        ESP_LOGE(TAG, "Timeout or error waiting for camera connection command");
        connect_logic_ble_disconnect();
        return -1;
    }

    // Parse the connection request command sent by camera
    connection_request_command_frame *camera_request = (connection_request_command_frame *)parse_result;

    if (camera_request->verify_mode != 2) {
        ESP_LOGE(TAG, "Unexpected verify_mode from camera: %d", camera_request->verify_mode);
        free(parse_result);
        connect_logic_ble_disconnect();
        return -1;
    }

    if (camera_request->verify_data == 0) {
        ESP_LOGI(TAG, "Camera approved the connection, sending response...");

        // Construct connection response frame
        connection_request_response_frame connection_response = {
            .device_id = device_id,
            .ret_code = 0,
        };
        memset(connection_response.reserved, 0, sizeof(connection_response.reserved));
        connection_response.reserved[0] = camera_reserved;

        ESP_LOGI(TAG, "Constructed connection response, sending...");

        // STEP4: Send connection response frame
        send_command(0x00, 0x19, ACK_NO_RESPONSE, &connection_response, received_seq, 5000);

        // Set connection state to protocol connected
        connect_state = PROTOCOL_CONNECTED;

        ESP_LOGI(TAG, "Connection successfully established with camera.");
        free(parse_result);
        return 0;
    } else {
        ESP_LOGW(TAG, "Camera rejected the connection, closing Bluetooth link...");
        free(parse_result);
        connect_logic_ble_disconnect();
        return -1;
    }
}

int connect_logic_ble_wakeup(void) {
    ESP_LOGI(TAG, "Attempting to wake up camera via BLE advertising");

    esp_err_t ret = ble_start_advertising();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start BLE advertising: %s", esp_err_to_name(ret));
        return -1;
    }

    ESP_LOGI(TAG, "BLE advertising started, attempting to wake up camera");
    return 0;
}