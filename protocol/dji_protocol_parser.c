/*
 * DJI Camera Remote Control - Protocol Parser
 * 
 * This file implements the low-level DJI protocol parser for camera communication.
 * It handles the binary protocol format used by DJI cameras over BLE connections,
 * including frame parsing, validation, and data extraction.
 * 
 * DJI Protocol Frame Format:
 * [SOF][VER+LEN][CMDTYPE][ENC][RES][SEQ][CRC16][CMDSET][CMDID][DATA][CRC32]
 * 
 * Frame Components:
 * - SOF (1 byte): Start of Frame marker (0x55)
 * - VER+LEN (2 bytes): Protocol version and frame length
 * - CMDTYPE (1 byte): Command type and response flags
 * - ENC (1 byte): Encryption type (typically 0x00)
 * - RES (3 bytes): Reserved bytes (padding)
 * - SEQ (2 bytes): Sequence number for request/response matching
 * - CRC16 (2 bytes): Header checksum validation
 * - CMDSET (1 byte): Command category (0x00=general, 0x01=camera, 0x05=status)
 * - CMDID (1 byte): Specific command identifier
 * - DATA (variable): Command payload data
 * - CRC32 (4 bytes): Full frame checksum validation
 * 
 * Parser Features:
 * - Incremental parsing for streaming data
 * - CRC validation for data integrity
 * - Frame synchronization and error recovery
 * - Support for both command and response frames
 * 
 * The parser maintains state across multiple calls to handle partial frames
 * and provides robust error handling for corrupted or incomplete data.
 * 
 * Hardware: M5StickC Plus2 with ESP32 BLE capabilities
 * Protocol: DJI proprietary binary communication protocol
 * 
 * Based on DJI SDK implementation
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "custom_crc16.h"
#include "custom_crc32.h"

#include "dji_protocol_data_processor.h"
#include "dji_protocol_parser.h"

/* Logging tag for ESP_LOG functions */
#define TAG "DJI_PROTOCOL_PARSER"

/* DJI Protocol Frame Field Length Definitions
 * 
 * These constants define the byte lengths of each field in the DJI protocol frame.
 * Used for parsing incoming data and constructing outgoing frames.
 */

/* Start of Frame marker - always 0x55 */
#define PROTOCOL_SOF_LENGTH 1

/* Version and frame length field (combined) */
#define PROTOCOL_VER_LEN_LENGTH 2

/* Command type field - indicates request/response and other flags */
#define PROTOCOL_CMD_TYPE_LENGTH 1

/* Encryption type field - typically 0x00 for no encryption */
#define PROTOCOL_ENC_LENGTH 1

/* Reserved bytes for future protocol extensions */
#define PROTOCOL_RES_LENGTH 3

/* Sequence number for matching requests with responses */
#define PROTOCOL_SEQ_LENGTH 2

/* CRC-16 checksum for header validation */
#define PROTOCOL_CRC16_LENGTH 2

/* Command set field - categorizes command types */
#define PROTOCOL_CMD_SET_LENGTH 1

/* Command ID field - specific command within set */
#define PROTOCOL_CMD_ID_LENGTH 1

/* CRC-32 checksum for entire frame validation */
#define PROTOCOL_CRC32_LENGTH 4

/**
 * Define header length (excluding CmdSet, CmdID and payload)
 */
#define PROTOCOL_HEADER_LENGTH ( \
    PROTOCOL_SOF_LENGTH +        \
    PROTOCOL_VER_LEN_LENGTH +    \
    PROTOCOL_CMD_TYPE_LENGTH +   \
    PROTOCOL_ENC_LENGTH +        \
    PROTOCOL_RES_LENGTH +        \
    PROTOCOL_SEQ_LENGTH +        \
    PROTOCOL_CRC16_LENGTH +      \
    PROTOCOL_CMD_SET_LENGTH +    \
    PROTOCOL_CMD_ID_LENGTH)

/**
 * Define tail length (only includes CRC-32)
 */
#define PROTOCOL_TAIL_LENGTH PROTOCOL_CRC32_LENGTH

/**
 * Define total frame length macro (dynamic calculation, including DATA segment)
 */
#define PROTOCOL_FULL_FRAME_LENGTH(data_length) ( \
    PROTOCOL_HEADER_LENGTH +                      \
    (data_length) +                               \
    PROTOCOL_TAIL_LENGTH)



/**
 * Parse notification frame
 *
 * Takes raw frame data and length, returns parsed result in frame_out structure
 *
 * @param frame_data Raw frame data
 * @param frame_length Frame length
 * @param frame_out Output structure for parsed result
 *
 * @return 0 on success, negative value on failure
 */
int protocol_parse_notification(const uint8_t *frame_data, size_t frame_length, protocol_frame_t *frame_out)
{
    // Check minimum frame length
    if (frame_length < 16)
    { // SOF(1) + Ver/Length(2) + CmdType(1) + ENC(1) + RES(3) + SEQ(2) + CRC-16(2) + CRC-32(4)
        ESP_LOGE(TAG, "Frame too short to be valid");
        return -1;
    }

    // Check frame header (SOF)
    if (frame_data[0] != 0xAA)
    {
        ESP_LOGE(TAG, "Invalid SOF: 0x%02X", frame_data[0]);
        return -2;
    }

    // Parse Ver/Length
    uint16_t ver_length = (frame_data[2] << 8) | frame_data[1];
    uint16_t version = ver_length >> 10;            // High 6 bits for version
    uint16_t expected_length = ver_length & 0x03FF; // Low 10 bits for frame length

    if (expected_length != frame_length)
    {
        ESP_LOGE(TAG, "Frame length mismatch: expected %u, got %zu", expected_length, frame_length);
        return -3;
    }

    // Verify CRC-16
    uint16_t crc16_received = (frame_data[11] << 8) | frame_data[10];
    uint16_t crc16_calculated = calculate_crc16(frame_data, 10); // From SOF to SEQ
    if (crc16_received != crc16_calculated)
    {
        ESP_LOGE(TAG, "CRC-16 mismatch: received 0x%04X, calculated 0x%04X", crc16_received, crc16_calculated);
        return -4;
    }

    // Verify CRC-32
    uint32_t crc32_received = (frame_data[frame_length - 1] << 24) | (frame_data[frame_length - 2] << 16) |
                              (frame_data[frame_length - 3] << 8) | frame_data[frame_length - 4];
    uint32_t crc32_calculated = calculate_crc32(frame_data, frame_length - 4); // From SOF to DATA
    if (crc32_received != crc32_calculated)
    {
        ESP_LOGE(TAG, "CRC-32 mismatch: received 0x%08X, calculated 0x%08X", (unsigned int)crc32_received, (unsigned int)crc32_calculated);
        return -5;
    }

    // Fill parsing results into structure
    frame_out->sof = frame_data[0];
    frame_out->version = version;
    frame_out->frame_length = expected_length;
    frame_out->cmd_type = frame_data[3];
    frame_out->enc = frame_data[4];
    memcpy(frame_out->res, &frame_data[5], 3);
    frame_out->seq = (frame_data[9] << 8) | frame_data[8];
    frame_out->crc16 = crc16_received;

    // Process data segment (DATA)
    if (frame_length > 16)
    { // DATA segment exists
        frame_out->data = &frame_data[12];
        frame_out->data_length = frame_length - 16; // DATA length
    }
    else
    { // DATA segment is empty
        frame_out->data = NULL;
        frame_out->data_length = 0;
        ESP_LOGW(TAG, "DATA segment is empty");
    }

    frame_out->crc32 = crc32_received;

    ESP_LOGI(TAG, "Frame parsed successfully");
    return 0;
}

/**
 * @brief Parse data segment from protocol frame
 *
 * Takes DATA segment, length and command type, returns parsed result length through data_length_without_cmd_out
 *
 * @param data Raw data segment
 * @param data_length Length of data segment
 * @param cmd_type Command type
 * @param data_length_without_cmd_out Output parameter for data length without cmdSet&CmdID
 *
 * @return void* Pointer to parsed result structure, NULL on failure
 */
void *protocol_parse_data(const uint8_t *data, size_t data_length, uint8_t cmd_type, size_t *data_length_without_cmd_out)
{
    if (data == NULL || data_length < 2)
    {
        ESP_LOGE(TAG, "Invalid data segment: data is NULL or too short");
        return NULL;
    }

    uint8_t cmd_set = data[0];
    uint8_t cmd_id = data[1];

    // Find corresponding command descriptor
    const data_descriptor_t *descriptor = find_data_descriptor(cmd_set, cmd_id);

    if (descriptor == NULL)
    {
        ESP_LOGW(TAG, "No descriptor found for CmdSet 0x%02X and CmdID 0x%02X by trying structure descriptor", cmd_set, cmd_id);
        return NULL;
    }

    // Extract response frame data
    const uint8_t *response_data = &data[2];
    size_t response_length = data_length - 2;

    ESP_LOGI(TAG, "CmdSet: 0x%02X, CmdID: 0x%02X", cmd_set, cmd_id);

    void *response_struct = malloc(response_length);
    if (response_struct == NULL)
    {
        ESP_LOGE(TAG, "Memory allocation failed for parsed data");
        return NULL;
    }

    int result = -1;
    if (descriptor != NULL)
    {
        result = data_parser_by_structure(cmd_set, cmd_id, cmd_type, response_data, response_length, response_struct);
    }

    if (result == 0)
    {
        ESP_LOGI(TAG, "Data parsed successfully for CmdSet 0x%02X and CmdID 0x%02X", cmd_set, cmd_id);
        if (data_length_without_cmd_out != NULL)
        {
            *data_length_without_cmd_out = response_length;
        }
    }
    else if (result == -2)
    {
        ESP_LOGW(TAG, "Parser function is NULL for CmdSet 0x%02X and CmdID 0x%02X by trying structure descriptor", cmd_set, cmd_id);
        free(response_struct);
        return NULL;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to parse data for CmdSet 0x%02X and CmdID 0x%02X", cmd_set, cmd_id);
        free(response_struct);
        return NULL;
    }

    return response_struct;
}

/**
 * @brief Create protocol frame
 *
 * Creates a complete protocol frame with given parameters and data structure
 *
 * @param cmd_set Command set
 * @param cmd_id Command ID
 * @param cmd_type Command type
 * @param structure Pointer to data structure
 * @param seq Sequence number
 * @param frame_length_out Output parameter for total frame length
 *
 * @return uint8_t* Pointer to created frame buffer, NULL on failure
 */
uint8_t *protocol_create_frame(uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, const void *structure, uint16_t seq, size_t *frame_length_out)
{
    size_t data_length = 0;
    uint8_t *payload_data = NULL;

    // Create payload data from structure
    payload_data = data_creator_by_structure(cmd_set, cmd_id, cmd_type, structure, &data_length);

    // Handle empty payload case
    if (payload_data == NULL && data_length > 0)
    {
        ESP_LOGE(TAG, "Failed to create payload data with non-zero length");
        return NULL;
    }

    // Calculate total frame length
    *frame_length_out = PROTOCOL_HEADER_LENGTH + data_length + PROTOCOL_TAIL_LENGTH;

    // Allocate memory for complete frame
    uint8_t *frame = (uint8_t *)malloc(*frame_length_out);
    if (frame == NULL)
    {
        ESP_LOGE(TAG, "Memory allocation failed for protocol frame");
        free(payload_data);
        return NULL;
    }

    // Initialize frame content
    memset(frame, 0, *frame_length_out);

    // Fill protocol header
    size_t offset = 0;
    frame[offset++] = 0xAA; // SOF start byte

    // Ver/Length field
    uint16_t version = 0; // Fixed version number
    uint16_t ver_length = (version << 10) | (*frame_length_out & 0x03FF);
    frame[offset++] = ver_length & 0xFF;        // Ver/Length low byte
    frame[offset++] = (ver_length >> 8) & 0xFF; // Ver/Length high byte

    // Fill command type
    frame[offset++] = cmd_type;

    // ENC (no encryption, fixed 0)
    frame[offset++] = 0x00;

    // RES (reserved bytes, fixed 0)
    frame[offset++] = 0x00;
    frame[offset++] = 0x00;
    frame[offset++] = 0x00;

    // Sequence number
    frame[offset++] = seq & 0xFF;        // Low byte of sequence number
    frame[offset++] = (seq >> 8) & 0xFF; // High byte of sequence number

    // Calculate and fill CRC-16 (covers from SOF to SEQ)
    uint16_t crc16 = calculate_crc16(frame, offset);
    frame[offset++] = crc16 & 0xFF;        // CRC-16 low byte
    frame[offset++] = (crc16 >> 8) & 0xFF; // CRC-16 high byte

    // Fill command set and ID
    frame[offset++] = cmd_set;
    frame[offset++] = cmd_id;

    // Fill payload data
    memcpy(&frame[offset], payload_data, data_length);
    offset += data_length;

    // Calculate and fill CRC-32 (covers from SOF to DATA)
    uint32_t crc32 = calculate_crc32(frame, offset);
    frame[offset++] = crc32 & 0xFF;         // CRC-32 byte 1
    frame[offset++] = (crc32 >> 8) & 0xFF;  // CRC-32 byte 2
    frame[offset++] = (crc32 >> 16) & 0xFF; // CRC-32 byte 3
    frame[offset++] = (crc32 >> 24) & 0xFF; // CRC-32 byte 4

    // Free payload data
    free(payload_data);

    return frame;
}