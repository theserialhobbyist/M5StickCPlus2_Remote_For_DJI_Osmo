#ifndef __DATA_H__
#define __DATA_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

void data_init(void);

bool is_data_layer_initialized(void);

esp_err_t data_write_with_response(uint16_t seq, const uint8_t *raw_data, size_t raw_data_length);

esp_err_t data_write_without_response(uint16_t seq, const uint8_t *raw_data, size_t raw_data_length);

esp_err_t data_wait_for_result_by_seq(uint16_t seq, int timeout_ms, void **out_result, size_t *out_result_length);

esp_err_t data_wait_for_result_by_cmd(uint8_t cmd_set, uint8_t cmd_id, int timeout_ms, uint16_t *out_seq, void **out_result, size_t *out_result_length);

typedef void (*camera_status_update_cb_t)(void *data);
void data_register_status_update_callback(camera_status_update_cb_t callback);

typedef void (*new_camera_status_update_cb_t)(void *data);
void data_register_new_status_update_callback(new_camera_status_update_cb_t callback);

void receive_camera_notify_handler(const uint8_t *raw_data, size_t raw_data_length);

#endif
