#include <string.h>
#include <stdio.h>
#include "esp_log.h"

#include "dji_protocol_data_processor.h"

#define TAG "DJI_PROTOCOL_DATA_PROCESSOR"

/**
 * @brief Find data descriptor by command set and command ID
 * 
 * @param cmd_set Command set
 * @param cmd_id Command ID
 * @return Return pointer to found data descriptor, NULL if not found
 */
const data_descriptor_t *find_data_descriptor(uint8_t cmd_set, uint8_t cmd_id) {
    for (size_t i = 0; i < DATA_DESCRIPTORS_COUNT; ++i) {
        if (data_descriptors[i].cmd_set == cmd_set && data_descriptors[i].cmd_id == cmd_id) {
            return &data_descriptors[i];
        }
    }
    return NULL;
}

/**
 * @brief Parse data according to structure
 * 
 * @param cmd_set Command set
 * @param cmd_id Command ID
 * @param cmd_type Command type
 * @param data Data to be parsed
 * @param data_length Data length
 * @param structure_out Output structure pointer
 * @return Return 0 on success, -1 or -2 on failure
 */
int data_parser_by_structure(uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, const uint8_t *data, size_t data_length, void *structure_out) {
    ESP_LOGI(TAG, "Parsing CmdSet: 0x%02X, CmdID: 0x%02X, CmdType: 0x%02X", cmd_set, cmd_id, cmd_type);

    // Find corresponding descriptor
    const data_descriptor_t *descriptor = find_data_descriptor(cmd_set, cmd_id);

    // Check if parser function exists
    if (descriptor->parser == NULL) {
        ESP_LOGW(TAG, "Parser function is NULL for CmdSet: 0x%02X, CmdID: 0x%02X", cmd_set, cmd_id);
        return -2;
    }

    return descriptor->parser(data, data_length, structure_out, cmd_type);
}

/**
 * @brief Create data according to structure
 * 
 * @param cmd_set Command set
 * @param cmd_id Command ID
 * @param cmd_type Command type
 * @param structure Input structure pointer
 * @param data_length Output data length
 * @return Return pointer to created data buffer, NULL on failure
 */
uint8_t* data_creator_by_structure(uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, const void *structure, size_t *data_length) {
    // Find corresponding descriptor
    const data_descriptor_t *descriptor = find_data_descriptor(cmd_set, cmd_id);
    if (descriptor == NULL) {
        ESP_LOGW(TAG, "Descriptor not found for CmdSet: 0x%02X, CmdID: 0x%02X", cmd_set, cmd_id);
        return NULL;
    }

    // Check if creator function exists
    if (descriptor->creator == NULL) {
        ESP_LOGW(TAG, "Creator function is NULL for CmdSet: 0x%02X, CmdID: 0x%02X", cmd_set, cmd_id);
        return NULL;
    }

    return descriptor->creator(structure, data_length, cmd_type);
}