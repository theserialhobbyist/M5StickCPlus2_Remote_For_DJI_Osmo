#ifndef DJI_PROTOCOL_DATA_PROCESSOR_H
#define DJI_PROTOCOL_DATA_PROCESSOR_H

#include <stdint.h>
#include <stddef.h>

#include "dji_protocol_data_descriptors.h"

const data_descriptor_t *find_data_descriptor(uint8_t cmd_set, uint8_t cmd_id);

int data_parser_by_structure(uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, const uint8_t *data, size_t data_length, void *output);

uint8_t* data_creator_by_structure(uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, const void *structure, size_t *data_length);

#endif