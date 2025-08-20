#ifndef __CONNECT_LOGIC_H__
#define __CONNECT_LOGIC_H__

typedef enum {
    BLE_NOT_INIT = -1,
    BLE_INIT_COMPLETE = 0,
    BLE_SEARCHING = 1,
    BLE_CONNECTED = 2,
    PROTOCOL_CONNECTED = 3,
    BLE_DISCONNECTING = 4,   // Actively disconnecting state
} connect_state_t;

connect_state_t connect_logic_get_state(void);

int connect_logic_ble_init();

int connect_logic_ble_connect(bool is_reconnecting);

int connect_logic_ble_disconnect(void);

int connect_logic_protocol_connect(uint32_t device_id, uint8_t mac_addr_len, const int8_t *mac_addr,
                                    uint32_t fw_version, uint8_t verify_mode, uint16_t verify_data,
                                    uint8_t camera_reserved);

int connect_logic_ble_wakeup(void);

#endif