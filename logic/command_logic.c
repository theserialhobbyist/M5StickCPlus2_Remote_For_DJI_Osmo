/*
 * DJI Camera Remote Control - Command Logic Layer
 * 
 * This file implements all camera command operations for the DJI remote control system.
 * It provides a unified interface for sending commands to DJI cameras and handling
 * their responses through the established BLE connection.
 * 
 * Supported Command Categories:
 * - Recording Control: Start/stop photo and video recording
 * - Mode Management: Switch between camera modes (photo, video, timelapse, etc.)
 * - Power Management: Sleep and wake camera operations
 * - Status Subscription: Request real-time camera status updates
 * 
 * Command Flow:
 * 1. Command Construction: Build DJI protocol frames with proper headers
 * 2. Sequence Management: Generate unique sequence numbers for tracking
 * 3. Transmission: Send commands via BLE to camera
 * 4. Response Handling: Wait for and parse camera responses
 * 5. Error Management: Handle timeouts and error responses
 * 
 * Protocol Details:
 * - Command Set 0x00: General camera operations
 * - Command Set 0x01: Recording and mode control
 * - Command Set 0x05: Status and telemetry
 * 
 * All commands use the DJI proprietary protocol format with CRC validation
 * and sequence numbering for reliable communication.
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
#include "dji_protocol_parser.h"
#include "dji_protocol_data_structures.h"

/* Logging tag for ESP_LOG functions */
#define TAG "LOGIC_COMMAND"

/* Global sequence number generator for command tracking
 * Each command gets a unique sequence number to match with responses
 */
uint16_t s_current_seq = 0;

/**
 * @brief Generate unique sequence number for command tracking
 * 
 * Creates incremental sequence numbers used to match commands with their
 * responses in the DJI protocol. Each command must have a unique sequence
 * number to prevent confusion with concurrent or delayed responses.
 * 
 * @return Unique 16-bit sequence number
 */
uint16_t generate_seq(void) {
    return s_current_seq += 1;
}

/**
 * @brief General function for constructing data frames and sending commands
 *        构造数据帧并发送命令的通用函数
 *
 * @param cmd_set Command set, used to specify command category
 *                命令集，用于指定命令的类别
 * @param cmd_id Command ID, used to identify specific command
 *               命令 ID，用于标识具体命令
 * @param cmd_type Command type, indicates features like response requirement
 *                 命令类型，指示是否需要应答等特性
 * @param structure Data structure pointer, contains input data for command frame
 *                 数据结构体指针，包含命令帧所需的输入数据
 * @param seq Sequence number, used to match request and response
 *            序列号，用于匹配请求与响应
 * @param timeout_ms Timeout for waiting result (in milliseconds)
 *                   等待结果的超时时间（以毫秒为单位）
 * 
 * Note: The caller needs to free the dynamically allocated memory after using the returned structure.
 * 注意：调用方需要在使用完返回的结构体后释放动态分配的内存。
 * 
 * @return CommandResult Returns parsed structure pointer and data length on success, NULL pointer and length 0 on failure
 *                       成功返回解析后的结构体指针及数据长度，失败返回 NULL 指针及长度 0
 */
CommandResult send_command(uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, const void *input_raw_data, uint16_t seq, int timeout_ms) { 
    CommandResult result = { NULL, 0 };

    if(connect_logic_get_state() <= BLE_INIT_COMPLETE){
        ESP_LOGE(TAG, "BLE not connected");
        return result;
    }

    esp_err_t ret;

    // Create protocol frame
    // 创建协议帧
    size_t frame_length = 0;
    uint8_t *protocol_frame = protocol_create_frame(cmd_set, cmd_id, cmd_type, input_raw_data, seq, &frame_length);
    if (protocol_frame == NULL) {
        ESP_LOGE(TAG, "Failed to create protocol frame");
        return result;
    }

    ESP_LOGI(TAG, "Protocol frame created successfully, length: %zu", frame_length);

    // Print ByteArray format for debugging
    // 打印 ByteArray 格式，便于调试
    printf("\033[96m");  // 设置青色输出
    printf("TX: [");
    for (size_t i = 0; i < frame_length; i++) {
        printf("%02X", protocol_frame[i]);
        if (i < frame_length - 1) {
            printf(", ");
        }
    }
    printf("]\n");
    printf("\033[0m");
    printf("\033[0;32m");

    void *structure_data = NULL;
    size_t structure_data_length = 0;

    switch (cmd_type) {
        case CMD_NO_RESPONSE:
        case ACK_NO_RESPONSE:
            ret = data_write_without_response(seq, protocol_frame, frame_length);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send data frame (no response), error: %s", esp_err_to_name(ret));
                free(protocol_frame);
                return result;
            }
            ESP_LOGI(TAG, "Data frame sent without response.");
            break;

        case CMD_RESPONSE_OR_NOT:
        case ACK_RESPONSE_OR_NOT:
            ret = data_write_with_response(seq, protocol_frame, frame_length);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send data frame (with response), error: %s", esp_err_to_name(ret));
                free(protocol_frame);
                return result;
            }
            ESP_LOGI(TAG, "Data frame sent, waiting for response...");
            
            ret = data_wait_for_result_by_seq(seq, timeout_ms, &structure_data, &structure_data_length);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "No result received, but continuing (seq=0x%04X)", seq);
            }

            break;

        case CMD_WAIT_RESULT:
        case ACK_WAIT_RESULT:
            ret = data_write_with_response(seq, protocol_frame, frame_length);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send data frame (wait result), error: %s", esp_err_to_name(ret));
                free(protocol_frame);
                return result;
            }
            ESP_LOGI(TAG, "Data frame sent, waiting for result...");

            ret = data_wait_for_result_by_seq(seq, timeout_ms, &structure_data, &structure_data_length);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to get parse result for seq=0x%04X, error: 0x%x", seq, ret);
                free(protocol_frame);
                return result;
            }

            if (structure_data == NULL) {
                ESP_LOGE(TAG, "Parse result is NULL for seq=0x%04X", seq);
                free(protocol_frame);
                return result;
            }

            break;

        default:
            ESP_LOGE(TAG, "Invalid cmd_type: %d", cmd_type);
            free(protocol_frame);
            return result;
    }

    free(protocol_frame);
    ESP_LOGI(TAG, "Command executed successfully");

    result.structure = structure_data;
    result.length = structure_data_length;

    return result;
}

/**
 * @brief Switch camera mode
 *        切换相机模式
 *
 * @param mode Camera mode
 *             相机模式
 * 
 * @return camera_mode_switch_response_frame_t* Returns parsed structure pointer, NULL on error
 *                                              返回解析后的结构体指针，如果发生错误返回 NULL
 */
camera_mode_switch_response_frame_t* command_logic_switch_camera_mode(camera_mode_t mode) {
    ESP_LOGI(TAG, "%s: Switching camera mode to: %d", __FUNCTION__, mode);
    if (connect_logic_get_state() != PROTOCOL_CONNECTED) {
        ESP_LOGE(TAG, "Protocol connection to the camera failed. Current connection state: %d", connect_logic_get_state());
        return NULL;
    }

    uint16_t seq = generate_seq();

    camera_mode_switch_command_frame_t command_frame = {
        .device_id = g_device_id,
        .mode = mode,
        .reserved = {0x01, 0x47, 0x39, 0x36}  // Reserved field
                                              // 预留字段
    };

    ESP_LOGI(TAG, "Constructed command frame: device_id=0x%08X, mode=%d", (unsigned int)command_frame.device_id, command_frame.mode);

    CommandResult result = send_command(
        0x1D,
        0x04,
        CMD_RESPONSE_OR_NOT,
        &command_frame,
        seq,
        5000
    );

    if (result.structure == NULL) {
        ESP_LOGE(TAG, "Failed to send command or receive response");
        return NULL;
    }

    camera_mode_switch_response_frame_t *response = (camera_mode_switch_response_frame_t *)result.structure;

    ESP_LOGI(TAG, "Received response: ret_code=%d", response->ret_code);
    return response;
}

/**
 * @brief Query device version
 *        查询设备版本号
 *
 * This function sends a query command to get device version information.
 * 该函数通过发送查询命令，获取设备的版本号信息。
 * 
 * The returned version information includes acknowledgment result (`ack_result`),
 * product ID (`product_id`) and SDK version (`sdk_version`).
 * 返回的版本号信息包括应答结果 (`ack_result`)、产品 ID (`product_id`) 和 SDK 版本号 (`sdk_version`)。
 *
 * @return version_query_response_frame_t* Returns parsed version info structure, NULL on error
 *                                         返回解析后的版本信息结构体，如果发生错误返回 NULL
 */
version_query_response_frame_t* command_logic_get_version(void) {
    ESP_LOGI(TAG, "%s: Querying device version", __FUNCTION__);
    
    if (connect_logic_get_state() != PROTOCOL_CONNECTED) {
        ESP_LOGE(TAG, "Protocol connection to the camera failed. Current connection state: %d", connect_logic_get_state());
        return NULL;
    }

    uint16_t seq = generate_seq();

    CommandResult result = send_command(
        0x00,
        0x00,
        CMD_WAIT_RESULT,
        NULL,
        seq,
        5000
    );

    if (result.structure == NULL) {
        ESP_LOGE(TAG, "Failed to send command or receive response");
        return NULL;
    }

    version_query_response_frame_t *response = (version_query_response_frame_t *)result.structure;

    ESP_LOGI(TAG, "Version Query Response: ack_result=%u, product_id=%s, sdk_version=%.*s",
             response->ack_result, response->product_id, 
             (int)(result.length - (sizeof(uint16_t) + sizeof(response->product_id))),
             response->sdk_version);

    return response;
}

/**
 * @brief Start recording
 *        开始录制
 *
 * @return record_control_response_frame_t* Returns parsed response structure pointer, NULL on error
 *                                          返回解析后的应答结构体指针，如果发生错误返回 NULL
 */
record_control_response_frame_t* command_logic_start_record(void) {
    ESP_LOGI(TAG, "%s: Starting recording", __FUNCTION__);

    if (connect_logic_get_state() != PROTOCOL_CONNECTED) {
        ESP_LOGE(TAG, "Protocol connection to the camera failed. Current connection state: %d", connect_logic_get_state());
        return NULL;
    }

    uint16_t seq = generate_seq();

    record_control_command_frame_t command_frame = {
        .device_id = g_device_id,
        .record_ctrl = 0x00,
        .reserved = {0x00, 0x00, 0x00, 0x00}
    };

    CommandResult result = send_command(
        0x1D,
        0x03,
        CMD_RESPONSE_OR_NOT,
        &command_frame,
        seq,
        5000
    );

    if (result.structure == NULL) {
        ESP_LOGE(TAG, "Failed to send command or receive response");
        return NULL;
    }

    record_control_response_frame_t *response = (record_control_response_frame_t *)result.structure;

    ESP_LOGI(TAG, "Start Record Response: ret_code=%d", response->ret_code);

    return response;
}

/**
 * @brief Stop recording
 *        停止录制
 *
 * @return record_control_response_frame_t* Returns parsed response structure pointer, NULL on error
 *                                          返回解析后的应答结构体指针，如果发生错误返回 NULL
 */
record_control_response_frame_t* command_logic_stop_record(void) {
    ESP_LOGI(TAG, "%s: Stopping recording", __FUNCTION__);

    if (connect_logic_get_state() != PROTOCOL_CONNECTED) {
        ESP_LOGE(TAG, "Protocol connection to the camera failed. Current connection state: %d", connect_logic_get_state());
        return NULL;
    }

    uint16_t seq = generate_seq();

    record_control_command_frame_t command_frame = {
        .device_id = g_device_id,
        .record_ctrl = 0x01,
        .reserved = {0x00, 0x00, 0x00, 0x00}
    };

    CommandResult result = send_command(
        0x1D,
        0x03,
        CMD_RESPONSE_OR_NOT,
        &command_frame,
        seq,
        5000
    );

    if (result.structure == NULL) {
        ESP_LOGE(TAG, "Failed to send command or receive response");
        return NULL;
    }

    record_control_response_frame_t *response = (record_control_response_frame_t *)result.structure;

    ESP_LOGI(TAG, "Stop Record Response: ret_code=%d", response->ret_code);

    return response;
}


/**
 * @brief Quick switch mode key report
 *        快速切换模式按键上报
 *
 * @return key_report_response_frame_t* Returns parsed response structure pointer, NULL on error
 *                                      返回解析后的应答结构体指针，如果发生错误返回 NULL
 */
key_report_response_frame_t* command_logic_key_report_qs(void) {
    ESP_LOGI(TAG, "%s: Reporting key press for mode switch", __FUNCTION__);

    if (connect_logic_get_state() != PROTOCOL_CONNECTED) {
        ESP_LOGE(TAG, "Protocol connection to the camera failed. Current connection state: %d", connect_logic_get_state());
        return NULL;
    }

    uint16_t seq = generate_seq();

    key_report_command_frame_t command_frame = {
        .key_code = 0x02,          // QS key code for mode switch
                                   // QS按键码，模式切换
        .mode = 0x01,              // Fixed as 0x01
                                   // 固定为 0x01
        .key_value = 0x00,         // Fixed as 0x00, short press event
                                   // 固定为 0x00，短按事件
    };

    CommandResult result = send_command(
        0x00,
        0x11,
        CMD_RESPONSE_OR_NOT,
        &command_frame,
        seq,
        5000
    );

    if (result.structure == NULL) {
        ESP_LOGE(TAG, "Failed to send command or receive response");
        return NULL;
    }

    key_report_response_frame_t *response = (key_report_response_frame_t *)result.structure;

    ESP_LOGI(TAG, "Key Report Response: ret_code=%d", response->ret_code);

    return response;
}

/**
 * @brief Put camera to sleep mode
 *        将相机设置为睡眠模式
 *
 * @return camera_power_mode_switch_response_frame_t* Returns parsed response structure pointer, NULL on error
 *                                                    返回解析后的应答结构体指针，如果发生错误返回 NULL
 */
camera_power_mode_switch_response_frame_t* command_logic_power_mode_switch_sleep(void) {
    // Log message indicating power mode switch to sleep
    // 睡眠模式切换日志
    ESP_LOGI(TAG, "%s: Reporting power mode switch to sleep", __FUNCTION__);

    // Check if the protocol is connected
    // 检查协议是否已连接
    if (connect_logic_get_state() != PROTOCOL_CONNECTED) {
        ESP_LOGE(TAG, "Protocol connection to the camera failed. Current connection state: %d", connect_logic_get_state());
        return NULL;
    }

    uint16_t seq = generate_seq();  // Generate sequence number
                                    // 生成序列号

    // Create command frame with power mode set to sleep mode (0x03)
    // 创建命令帧，将电源模式设置为睡眠模式 (0x03)
    camera_power_mode_switch_command_frame_t command_frame = {
        .power_mode = 0x03,        // Set to 0x03 for sleep mode
                                   // 设置为 0x03 表示睡眠模式
    };

    // Send the command and receive the response
    // 发送命令并接收响应
    CommandResult result = send_command(
        0x00,
        0x1A,                 // Use CmdSet = 0x00 and CmdID = 0x1A for power mode switch
                              // 使用 CmdSet = 0x00 和 CmdID = 0x1A 进行电源模式切换
        CMD_RESPONSE_OR_NOT,
        &command_frame,
        seq,
        5000
    );

    // If no response structure is returned, the send or receive failed
    // 如果未返回响应结构体，则表示发送或接收失败
    if (result.structure == NULL) {
        ESP_LOGE(TAG, "Failed to send command or receive response");
        return NULL;
    }

    // Convert the returned data structure to the corresponding response frame
    // 将返回的数据结构转换为相应的响应帧
    camera_power_mode_switch_response_frame_t *response = (camera_power_mode_switch_response_frame_t *)result.structure;

    // Log the response information for power mode switch
    // 记录电源模式切换的响应信息
    ESP_LOGI(TAG, "Power Mode Switch Response: ret_code=%d", response->ret_code);

    return response;  // Return the parsed response frame
                      // 返回解析后的响应帧
}