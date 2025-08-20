#ifndef __BLE_H__
#define __BLE_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "esp_err.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"

/* Connection status structure */
typedef struct {
    bool is_connected; // Connection status
} connection_status_t;

/* Handle discovery status structure */
typedef struct {
    bool notify_char_handle_found; // Notify characteristic handle found
    bool write_char_handle_found;  // Write characteristic handle found
} handle_discovery_t;

/* Global profile structure to manage connection and characteristic information */
typedef struct {
    uint16_t conn_id;              // Connection ID
    esp_gatt_if_t gattc_if;        // GATT client interface

    /* Handles for characteristics we need to operate on */
    uint16_t notify_char_handle;   // Notify characteristic handle
    uint16_t write_char_handle;    // Write characteristic handle
    uint16_t read_char_handle;     // Read characteristic handle

    /* Start and end handles of the service */
    uint16_t service_start_handle; // Service start handle
    uint16_t service_end_handle;   // Service end handle

    /* Remote device address */
    esp_bd_addr_t remote_bda;      // Remote Bluetooth device address

    connection_status_t connection_status;     // Connection status
    handle_discovery_t handle_discovery;       // Handle discovery status
} ble_profile_t;

extern ble_profile_t s_ble_profile;

/**
 * @brief Notify callback function type for receiving data from remote
 *
 * @param data   Pointer to the notification data
 * @param length Length of the notification data
 */
typedef void (*ble_notify_callback_t)(const uint8_t *data, size_t length);

typedef void (*connect_logic_state_callback_t)(void);

esp_err_t ble_init();

esp_err_t ble_start_scanning_and_connect(void);

void ble_set_reconnecting(bool flag);

bool ble_get_reconnecting(void);

esp_err_t ble_reconnect(void);

esp_err_t ble_disconnect(void);

esp_err_t ble_read(uint16_t conn_id, uint16_t handle);

esp_err_t ble_write_without_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length);

esp_err_t ble_write_with_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length);

esp_err_t ble_register_notify(uint16_t conn_id, uint16_t char_handle);

esp_err_t ble_unregister_notify(uint16_t conn_id, uint16_t char_handle);

void ble_set_notify_callback(ble_notify_callback_t cb);

void ble_set_state_callback(connect_logic_state_callback_t cb);

esp_err_t ble_start_advertising(void);

// Functions to get connected camera information
const char* ble_get_connected_device_name(void);
const uint8_t* ble_get_connected_device_mac(void);
bool ble_get_connected_device_info(char* name, size_t name_size, uint8_t* mac);

// Function to set target device for reconnection
void ble_set_target_device(const char* name, const uint8_t* mac);

// Function to wake up camera using BLE advertising broadcast
esp_err_t ble_wake_camera(const uint8_t* camera_mac);

#endif