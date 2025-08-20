/*
 * DJI Camera Remote Control - BLE Communication Layer
 * 
 * This file implements the complete Bluetooth Low Energy communication system
 * for connecting to and controlling DJI cameras. It provides the low-level
 * BLE interface that the connection logic layer uses for camera communication.
 * 
 * BLE Architecture:
 * - GATT Client role for connecting to DJI cameras (which act as servers)
 * - Service discovery for DJI-specific UUIDs
 * - Characteristic handling for data transmission and notifications
 * - Connection management with automatic reconnection capability
 * 
 * Key Features:
 * - Device scanning with RSSI-based best device selection
 * - Targeted reconnection to previously paired cameras
 * - Robust connection state management
 * - Wake-up advertising for sleeping cameras
 * - Data transmission with proper MTU handling
 * 
 * DJI Service Discovery:
 * - Scans for devices advertising DJI service UUIDs
 * - Discovers write and notify characteristics
 * - Handles GATT service enumeration
 * - Manages connection parameters optimization
 * 
 * Connection Modes:
 * - Initial pairing: Connects to any compatible DJI camera
 * - Reconnection: Targets specific camera by name and MAC address
 * - Wake broadcast: Sends advertising to wake sleeping cameras
 * 
 * The BLE layer provides callback-based notification to upper layers
 * for connection events and received data processing.
 * 
 * Hardware: M5StickC Plus2 with ESP32 BLE capabilities
 * Protocol: BLE GATT over DJI proprietary service UUIDs
 * 
 * Based on DJI SDK implementation
 */

#include <string.h>
#include "ble.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_timer.h"

/* Logging tag for ESP_LOG functions */
#define TAG "BLE"

/* Target device name for connection attempts
 * Stores the BLE advertising name of the camera to connect to
 */
static char s_remote_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = {0};

/* Connection state tracking
 * Prevents multiple simultaneous connection attempts
 */
static bool s_connecting = false;

/* Global callback for received data notifications
 * Called when data is received from the camera via BLE notifications
 */
static ble_notify_callback_t s_notify_cb = NULL;

/* Global callback for connection state changes
 * Notifies upper layers of connection and disconnection events
 */
static connect_logic_state_callback_t s_state_cb = NULL;

/* Device scanning and selection parameters
 * Implements RSSI-based best device selection during scanning
 */
#define MIN_RSSI_THRESHOLD -80          /* Minimum acceptable signal strength (-80 dBm) */
static esp_bd_addr_t best_addr = {0};   /* MAC address of device with strongest signal */
static int8_t best_rssi = -128;         /* RSSI of best device (initialized to weakest possible) */
static bool s_is_reconnecting = false;  /* True when attempting reconnection to known device */
static bool s_found_previous_device = false;  /* True when previously paired device is found during scan */

/* BLE GATT client profile
 * Stores all connection state, handles, and discovery information
 */
ble_profile_t s_ble_profile = {
    .conn_id = 0,
    .gattc_if = ESP_GATT_IF_NONE,
    .remote_bda = {0x60, 0x60, 0x1F, 0x60, 0x11, 0xE7},
    .notify_char_handle = 0,
    .write_char_handle = 0,
    .read_char_handle = 0,
    .service_start_handle = 0,
    .service_end_handle = 0,
    .connection_status = {
        .is_connected = false,
    },
    .handle_discovery = {
        .notify_char_handle_found = false,
        .write_char_handle_found = false,
    },
};

/* Define the Service/Characteristic UUIDs to filter, for search use */
#define REMOTE_TARGET_SERVICE_UUID   0xFFF0
#define REMOTE_NOTIFY_CHAR_UUID      0xFFF4
#define REMOTE_WRITE_CHAR_UUID       0xFFF5

static esp_bt_uuid_t s_filter_notify_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid.uuid16 = REMOTE_NOTIFY_CHAR_UUID,
};

static esp_bt_uuid_t s_filter_write_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid.uuid16 = REMOTE_WRITE_CHAR_UUID,
};

static esp_bt_uuid_t s_notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
};

/* Scan parameters, adjustable as needed */
static esp_ble_scan_params_t s_ble_scan_params = {
    .scan_type          = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval      = 0x50,
    .scan_window        = 0x30,
    .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE
};

/* Callback function declarations */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gattc_event_handler(esp_gattc_cb_event_t event,
                                esp_gatt_if_t gattc_if,
                                esp_ble_gattc_cb_param_t *param);

static TimerHandle_t scan_timer;

void scan_stop_timer_callback(TimerHandle_t xTimer) {
    esp_ble_gap_stop_scanning();
    ESP_LOGI(TAG, "Scan stopped after timeout");
}

static void trigger_scan_task(void) {
    ESP_LOGI(TAG, "esp_ble_gap_start_scanning...");
    esp_err_t ret = esp_ble_gap_start_scanning(4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start scanning: %s", esp_err_to_name(ret));
    }
    // Start a timer to stop scanning after 4 seconds
    scan_timer = xTimerCreate("scan_timer", pdMS_TO_TICKS(4000), pdFALSE, (void *)0, scan_stop_timer_callback);
    if (scan_timer != NULL) {
        xTimerStart(scan_timer, 0);
    }
}

/* -------------------------
 *  Initialization/Scan/Connection related interfaces
 * ------------------------- */

/**
 * @brief BLE client initialization
 *
 * @return esp_err_t
 *         - ESP_OK on success
 *         - Others on failure
 */
esp_err_t ble_init() {
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Release classic Bluetooth memory */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    /* Configure and initialize the Bluetooth controller */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "initialize controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start the BLE controller */
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "enable controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Initialize the Bluedroid stack */
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "init bluedroid failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Enable Bluedroid */
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "enable bluedroid failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register GAP callback */
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "gap register error, err code = %x", ret);
        return ret;
    }

    /* Register GATTC callback */
    ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "gattc register error, err code = %x", ret);
        return ret;
    }

    /* Register GATTC application (only one profile here, app_id = 0) */
    ret = esp_ble_gattc_app_register(0);
    if (ret) {
        ESP_LOGE(TAG, "gattc app register error, err code = %x", ret);
        return ret;
    }

    /* Set local MTU (optional) */
    esp_ble_gatt_set_local_mtu(500);

    ESP_LOGI(TAG, "ble_init success!");
    return ESP_OK;
}

/**
 * @brief Connect to a device with a specified name (if already scanning, it will automatically connect when the device is found)
 *
 * @note  This interface is for demonstration only. If you want to actively specify an address to connect, you can extend the interface yourself.
 * @return esp_err_t
 */
esp_err_t ble_start_scanning_and_connect(void) {
    // TODO: Add reconnection logic; current implementation has issues and needs to be fixed.
    if(ble_get_reconnecting()) {
        return ble_reconnect();
    }

    // Reset scan-related variables
    memset(best_addr, 0, sizeof(esp_bd_addr_t));
    best_rssi = -128;
    memset(s_remote_device_name, 0, ESP_BLE_ADV_NAME_LEN_MAX);
    s_is_reconnecting = false;
    s_found_previous_device = false;

    // Set scan parameters
    esp_err_t ret = esp_ble_gap_set_scan_params(&s_ble_scan_params);
    if (ret) {
        ESP_LOGE(TAG, "Set scan params error: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set scan params ok!");
    return ESP_OK;
}

static void try_to_connect(esp_bd_addr_t addr) {
    // Check if already connecting
    if (s_connecting) {
        ESP_LOGW(TAG, "Already in connecting state, please wait...");
        return;
    }

    // Check if the address is the initial value (all zeros)
    bool is_valid = false;
    for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
        if (addr[i] != 0) {
            is_valid = true;
            break;
        }
    }

    if (!is_valid) {
        ESP_LOGE(TAG, "Invalid device address (all zeros)");
        return;
    }

    s_connecting = true;
    ESP_LOGI(TAG, "Try to connect target device name = %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             s_remote_device_name,
             addr[0], addr[1], addr[2],
             addr[3], addr[4], addr[5]);

    // Do not call lightly, if you connect to a non-existent device address, you will have to wait a while before you can connect again
    esp_ble_gattc_open(s_ble_profile.gattc_if,
                       addr,
                       BLE_ADDR_TYPE_PUBLIC,
                       true);
}

void ble_set_reconnecting(bool flag) {
    s_is_reconnecting = flag;
}

bool ble_get_reconnecting(void) {
    return s_is_reconnecting;
}

/**
 * @brief Reconnect to the last connected device
 * 
 * @note Only applicable to non-active disconnection situations, as device information is not cleared
 * @return esp_err_t
 */
esp_err_t ble_reconnect(void) {
    // Check if there is a valid last connection address
    bool is_valid = false;
    for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
        if (best_addr[i] != 0) {
            is_valid = true;
            break;
        }
    }

    if (!is_valid) {
        ESP_LOGE(TAG, "No valid previous device address found");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Attempting to reconnect to previous device: %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             s_remote_device_name,
             best_addr[0], best_addr[1], best_addr[2],
             best_addr[3], best_addr[4], best_addr[5]);

    // Set reconnection mode flag
    s_is_reconnecting = true;
    s_found_previous_device = false;  // Reset discovery flag
    
    // Start scan task
    trigger_scan_task();
    
    return ESP_OK;
}

/**
 * @brief Disconnect (if connected)
 *
 * @return esp_err_t
 */
esp_err_t ble_disconnect(void) {
    if (s_ble_profile.connection_status.is_connected) {
        esp_ble_gattc_close(s_ble_profile.gattc_if, s_ble_profile.conn_id);
    }
    return ESP_OK;
}

/* -------------------------
 *  Read/Write and Notify related interfaces
 * ------------------------- */
/**
 * @brief Read a specified characteristic
 *
 * @param conn_id  Connection ID (obtained from callback events or internal management)
 * @param handle   Handle of the characteristic
 * @return esp_err_t
 */
esp_err_t ble_read(uint16_t conn_id, uint16_t handle) {
    if (!s_ble_profile.connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected, skip read");
        return ESP_FAIL;
    }
    /* Initiate GATTC read request */
    esp_err_t ret = esp_ble_gattc_read_char(s_ble_profile.gattc_if,
                                            conn_id,
                                            handle,
                                            ESP_GATT_AUTH_REQ_NONE);
    if (ret) {
        ESP_LOGE(TAG, "read_char failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Write characteristic (Write Without Response)
 *
 * @param conn_id   Connection ID
 * @param handle    Handle of the characteristic
 * @param data      Data to be written
 * @param length    Length of the data
 * @return esp_err_t
 */
esp_err_t ble_write_without_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length) {
    if (!s_ble_profile.connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected, skip write_without_response");
        return ESP_FAIL;
    }
    esp_err_t ret = esp_ble_gattc_write_char(s_ble_profile.gattc_if,
                                             conn_id,
                                             handle,
                                             length,
                                             (uint8_t *)data,
                                             ESP_GATT_WRITE_TYPE_NO_RSP,
                                             ESP_GATT_AUTH_REQ_NONE);
    if (ret) {
        ESP_LOGE(TAG, "write_char NO_RSP failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Write characteristic (Write With Response)
 *
 * @param conn_id   Connection ID
 * @param handle    Handle of the characteristic
 * @param data      Data to be written
 * @param length    Length of the data
 * @return esp_err_t
 */
esp_err_t ble_write_with_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length) {
    if (!s_ble_profile.connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected, skip write_with_response");
        return ESP_FAIL;
    }
    esp_err_t ret = esp_ble_gattc_write_char(s_ble_profile.gattc_if,
                                             conn_id,
                                             handle,
                                             length,
                                             (uint8_t *)data,
                                             ESP_GATT_WRITE_TYPE_RSP,
                                             ESP_GATT_AUTH_REQ_NONE);
    if (ret) {
        ESP_LOGE(TAG, "write_char RSP failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Register (enable) Notify
 *
 * @param conn_id   Connection ID
 * @param char_handle Handle of the characteristic to enable notification
 * @return esp_err_t
 */
esp_err_t ble_register_notify(uint16_t conn_id, uint16_t char_handle) {
    if (!s_ble_profile.connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected, skip register_notify");
        return ESP_FAIL;
    }
    /* Request to subscribe to notifications from the protocol stack */
    esp_err_t ret = esp_ble_gattc_register_for_notify(s_ble_profile.gattc_if,
                                                      s_ble_profile.remote_bda,
                                                      char_handle);
    if (ret) {
        ESP_LOGE(TAG, "register_notify failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Unregister (disable) Notify
 *
 * @note  This is just a demonstration logic. You need the Client Config descriptor handle of the characteristic to operate.
 *        If needed in actual development, you can directly save the descr handle previously, and then close it by writing 0x0000 here.
 *
 * @param conn_id   Connection ID
 * @param char_handle Handle of the characteristic to disable notification
 * @return esp_err_t
 */
esp_err_t ble_unregister_notify(uint16_t conn_id, uint16_t char_handle) {
    /* In fact, you need to get the corresponding descriptor handle and then write 0x0000 to disable it */
    /* This is just a demonstration of the process. If needed, you can save the descr handle during register_notify */
    ESP_LOGI(TAG, "ble_unregister_notify called (demo), not fully implemented");
    return ESP_OK;
}

/**
 * @brief Set global Notify callback (for receiving data)
 *
 * @param cb Callback function pointer
 */
void ble_set_notify_callback(ble_notify_callback_t cb) {
    s_notify_cb = cb;
}

/**
 * @brief Set global logic layer disconnection state callback
 *
 * @param cb Callback function pointer
 */
void ble_set_state_callback(connect_logic_state_callback_t cb) {
    s_state_cb = cb;
}

/* ----------------------------------------------------------------
 *   GAP & GATTC callback function implementation (simplified version)
 * ---------------------------------------------------------------- */

/* Determine whether it is a DJI camera advertisement */
static uint8_t bsp_link_is_dji_camera_adv(esp_ble_gap_cb_param_t *scan_result) {
    const uint8_t *ble_adv = scan_result->scan_rst.ble_adv;
    const uint8_t adv_len = scan_result->scan_rst.adv_data_len + scan_result->scan_rst.scan_rsp_len;

    for (int i = 0; i < adv_len; ) {
        const uint8_t len = ble_adv[i];
        
        if (len == 0 || (i + len + 1) > adv_len) break;

        const uint8_t type = ble_adv[i+1];
        const uint8_t *data = &ble_adv[i+2];
        const uint8_t data_len = len - 1;

        if (type == ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE) {
            if (data_len >= 5) {
                if (data[0] == 0xAA && data[1] == 0x08 && data[4] == 0xFA) {
                    return 1;
                }
            }
        }
        i += (len + 1);
    }
    return 0;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT");
        trigger_scan_task();
        break;

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "scan stopped");
        // After scanning ends, decide whether to connect based on reconnection mode and device discovery status
        if (best_rssi > -128) {
            if (!ble_get_reconnecting() || (ble_get_reconnecting() && s_found_previous_device)) {
                try_to_connect(best_addr);
                ESP_LOGI(TAG, "Connected to device: %02x:%02x:%02x:%02x:%02x:%02x",
                         best_addr[0], best_addr[1], best_addr[2], best_addr[3], best_addr[4], best_addr[5]);
            } else {
                ESP_LOGW(TAG, "In reconnection mode but target device not found");
            }
        } else {
            ESP_LOGW(TAG, "No suitable device found with sufficient signal strength");
        }
        s_is_reconnecting = false;
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *r = param;
        if (r->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            // Check if it is a DJI camera advertisement
            if (!bsp_link_is_dji_camera_adv(r)) {
                break;
            }
            // Get the complete name from the advertisement data
            uint8_t *adv_name = NULL;
            uint8_t adv_name_len = 0;
            adv_name = esp_ble_resolve_adv_data_by_type(r->scan_rst.ble_adv,
                                r->scan_rst.adv_data_len + r->scan_rst.scan_rsp_len,
                                ESP_BLE_AD_TYPE_NAME_CMPL,
                                &adv_name_len);

            // Prepare a safe string pointer for logging
            const char *adv_name_str = NULL;
            if (adv_name && adv_name_len > 0) {
                static char name_buf[64];
                size_t copy_len = adv_name_len < sizeof(name_buf) - 1 ? adv_name_len : sizeof(name_buf) - 1;
                memcpy(name_buf, adv_name, copy_len);
                name_buf[copy_len] = '\0';
                adv_name_str = name_buf;
            } else {
                adv_name_str = "NULL";
            }

            ESP_LOGI(TAG, "Found device: %s with RSSI: %d, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                adv_name_str,
                r->scan_rst.rssi,
                r->scan_rst.bda[0], r->scan_rst.bda[1], r->scan_rst.bda[2],
                r->scan_rst.bda[3], r->scan_rst.bda[4], r->scan_rst.bda[5]);

            // Compare names and record signal strength
            if (ble_get_reconnecting()) {
                // In reconnection mode, compare device addresses
                if (memcmp(best_addr, r->scan_rst.bda, sizeof(esp_bd_addr_t)) == 0) {
                    s_found_previous_device = true;
                    best_rssi = r->scan_rst.rssi;  // Update RSSI for reconnection
                    ESP_LOGI(TAG, "Found previous device: %s, RSSI: %d", adv_name_str, r->scan_rst.rssi);
                }
            } else {
                // In normal scan mode, record the device with the strongest signal
                if (r->scan_rst.rssi > best_rssi && r->scan_rst.rssi >= MIN_RSSI_THRESHOLD) {
                    best_rssi = r->scan_rst.rssi;
                    memcpy(best_addr, r->scan_rst.bda, sizeof(esp_bd_addr_t));
                    strncpy(s_remote_device_name, adv_name_str, sizeof(s_remote_device_name) - 1);
                    s_remote_device_name[sizeof(s_remote_device_name) - 1] = '\0';
                }
            }
        }
        break;
    }

    default:
        break;
    }
}

static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    switch (event) {
    case ESP_GATTC_REG_EVT: {
        // Handle GATT client registration event
        if (param->reg.status == ESP_GATT_OK) {
            s_ble_profile.gattc_if = gattc_if;
            ESP_LOGI(TAG, "GATTC register OK, app_id=%d, gattc_if=%d",
                     param->reg.app_id, gattc_if);
        } else {
            ESP_LOGE(TAG, "GATTC register failed, status=%d", param->reg.status);
        }
        break;
    }
    case ESP_GATTC_CONNECT_EVT: {
        // Handle connection event
        s_ble_profile.conn_id = param->connect.conn_id;
        s_ble_profile.connection_status.is_connected = true;
        memcpy(s_ble_profile.remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "Connected, conn_id=%d", s_ble_profile.conn_id);

        ESP_LOGI(TAG, "Connect to camera MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
            param->connect.remote_bda[0],
            param->connect.remote_bda[1],
            param->connect.remote_bda[2],
            param->connect.remote_bda[3],
            param->connect.remote_bda[4],
            param->connect.remote_bda[5]);

        // Initiate MTU request
        esp_ble_gattc_send_mtu_req(gattc_if, param->connect.conn_id);
        break;
    }
    case ESP_GATTC_OPEN_EVT: {
        // Handle connection open event
        s_connecting = false;
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Open failed, status=%d", param->open.status);
            break;
        }
        ESP_LOGI(TAG, "Open success, MTU=%u", param->open.mtu);
        break;
    }
    case ESP_GATTC_CFG_MTU_EVT: {
        // Handle MTU configuration event
        if (param->cfg_mtu.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Config MTU Error, status=%d", param->cfg_mtu.status);
        }
        ESP_LOGI(TAG, "MTU=%d", param->cfg_mtu.mtu);

        // Start service discovery after MTU configuration
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL);
        break;
    }
    case ESP_GATTC_SEARCH_RES_EVT: {
        // Handle service search result event
        if ((param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) &&
            (param->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_TARGET_SERVICE_UUID)) {
            s_ble_profile.service_start_handle = param->search_res.start_handle;
            s_ble_profile.service_end_handle   = param->search_res.end_handle;
            ESP_LOGI(TAG, "Service found: start=%d, end=%d",
                     s_ble_profile.service_start_handle,
                     s_ble_profile.service_end_handle);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
        // Handle service search complete event
        if (param->search_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Service search failed, status=%d", param->search_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "Service search complete, next get char by UUID");

        // Get notify characteristic handle
        uint16_t count = 1;
        esp_gattc_char_elem_t char_elem_result;
        esp_ble_gattc_get_char_by_uuid(gattc_if,
                                       s_ble_profile.conn_id,
                                       s_ble_profile.service_start_handle,
                                       s_ble_profile.service_end_handle,
                                       s_filter_notify_char_uuid,
                                       &char_elem_result,
                                       &count);
        if (count > 0) {
            s_ble_profile.notify_char_handle = char_elem_result.char_handle;
            s_ble_profile.handle_discovery.notify_char_handle_found = true;
            ESP_LOGI(TAG, "Notify Char found, handle=0x%x",
                     s_ble_profile.notify_char_handle);
        }

        // Get write characteristic handle
        count = 1;
        esp_gattc_char_elem_t write_char_elem_result;
        esp_ble_gattc_get_char_by_uuid(gattc_if,
                                       s_ble_profile.conn_id,
                                       s_ble_profile.service_start_handle,
                                       s_ble_profile.service_end_handle,
                                       s_filter_write_char_uuid,
                                       &write_char_elem_result,
                                       &count);
        if (count > 0) {
            s_ble_profile.write_char_handle = write_char_elem_result.char_handle;
            s_ble_profile.handle_discovery.write_char_handle_found = true;
            ESP_LOGI(TAG, "Write Char found, handle=0x%x",
                     s_ble_profile.write_char_handle);
        }

        break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        // Handle notification registration event
        if (param->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Notify register failed, status=%d", param->reg_for_notify.status);
            break;
        }
        ESP_LOGI(TAG, "Notify register success, handle=0x%x", param->reg_for_notify.handle);

        // Find descriptor and write 0x01 to enable notification
        uint16_t count = 1;
        esp_gattc_descr_elem_t descr_elem;
        esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                               s_ble_profile.conn_id,
                                               param->reg_for_notify.handle,
                                               s_notify_descr_uuid,
                                               &descr_elem,
                                               &count);
        if (count > 0 && descr_elem.handle) {
            uint16_t notify_en = 1;
            esp_ble_gattc_write_char_descr(gattc_if,
                                           s_ble_profile.conn_id,
                                           descr_elem.handle,
                                           sizeof(notify_en),
                                           (uint8_t *)&notify_en,
                                           ESP_GATT_WRITE_TYPE_RSP,
                                           ESP_GATT_AUTH_REQ_NONE);
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
        // Handle notification data event
        if (s_notify_cb) {
            s_notify_cb(param->notify.value, param->notify.value_len);
        }
        break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
        // Handle disconnection event
        s_ble_profile.connection_status.is_connected = false;
        s_ble_profile.handle_discovery.write_char_handle_found = false;
        s_ble_profile.handle_discovery.notify_char_handle_found = false;
        s_connecting = false;
        ESP_LOGI(TAG, "Disconnected, reason=0x%x", param->disconnect.reason);

        if (s_state_cb) {
            s_state_cb();
        }
        break;
    }
    default:
        break;
    }
}

// BLE Advertising Data Format
static uint8_t adv_data[] = {
    10, 0xff, 'W','K','P','1','2','3','4','5','6'
};

static esp_timer_handle_t adv_timer;

static void stop_adv_after_2s(void* arg) {
    esp_ble_gap_stop_advertising();
    ESP_LOGI(TAG, "Advertising stopped after 2 seconds");
}

esp_err_t ble_start_advertising() {
    // Check if remote_bda is initialized
    if (memcmp(s_ble_profile.remote_bda, "\x00\x00\x00\x00\x00\x00", 6) == 0) {
        ESP_LOGE(TAG, "Error: remote_bda not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < 6; i++) {
        // adv_data[8 + i] = s_ble_profile.remote_bda[5 - i];
        adv_data[5 + i] = s_ble_profile.remote_bda[5 - i];
    }

    ESP_LOGI(TAG, "Modified Advertising Data (with MAC):");
    ESP_LOG_BUFFER_HEX(TAG, adv_data, sizeof(adv_data));

    esp_ble_adv_params_t adv_params = {
        .adv_int_min = 0x20,
        .adv_int_max = 0x40,
        .adv_type = ADV_TYPE_IND,
        .channel_map = ADV_CHNL_ALL,
    };

    esp_err_t ret = esp_ble_gap_config_adv_data_raw(adv_data, sizeof(adv_data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set adv data.");
        return ret;
    }

    ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start advertising: %s", esp_err_to_name(ret));
        return ret;
    }

    if (adv_timer == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = &stop_adv_after_2s,
            .name = "adv_timer"
        };
        esp_timer_create(&timer_args, &adv_timer);
    }
    
    esp_timer_start_once(adv_timer, 2000000);  // 2000ms = 2,000,000us

    ESP_LOGI(TAG, "Advertising started (will auto-stop after 2s)");
    return ESP_OK;
}

// Functions to get connected camera information
const char* ble_get_connected_device_name(void) {
    if (s_ble_profile.connection_status.is_connected) {
        return s_remote_device_name;
    }
    return NULL;
}

const uint8_t* ble_get_connected_device_mac(void) {
    if (s_ble_profile.connection_status.is_connected) {
        return (const uint8_t*)s_ble_profile.remote_bda;
    }
    return NULL;
}

bool ble_get_connected_device_info(char* name, size_t name_size, uint8_t* mac) {
    if (!s_ble_profile.connection_status.is_connected || !name || !mac) {
        return false;
    }
    
    // Copy device name
    strncpy(name, s_remote_device_name, name_size - 1);
    name[name_size - 1] = '\0';
    
    // Copy MAC address
    memcpy(mac, s_ble_profile.remote_bda, 6);
    
    return true;
}

// Function to wake up camera using BLE advertising broadcast
esp_err_t ble_wake_camera(const uint8_t* camera_mac) {
    if (!camera_mac) {
        ESP_LOGE(TAG, "Camera MAC address is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Starting wake broadcast for camera: %02X:%02X:%02X:%02X:%02X:%02X",
             camera_mac[0], camera_mac[1], camera_mac[2], 
             camera_mac[3], camera_mac[4], camera_mac[5]);
    
    // Prepare wake-up advertising data
    // Format: [10, 0xff, 'W','K','P', MAC[5], MAC[4], MAC[3], MAC[2], MAC[1], MAC[0]]
    static uint8_t wake_adv_data[] = {
        10, 0xff, 'W','K','P','1','2','3','4','5','6'
    };
    
    // Fill in camera MAC address in reverse order (as per documentation)
    for (int i = 0; i < 6; i++) {
        wake_adv_data[5 + i] = camera_mac[5 - i];
    }
    
    ESP_LOGI(TAG, "Wake broadcast data: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             wake_adv_data[0], wake_adv_data[1], wake_adv_data[2], wake_adv_data[3],
             wake_adv_data[4], wake_adv_data[5], wake_adv_data[6], wake_adv_data[7],
             wake_adv_data[8], wake_adv_data[9], wake_adv_data[10]);
    
    // Set advertising data
    esp_err_t ret = esp_ble_gap_config_adv_data_raw(wake_adv_data, sizeof(wake_adv_data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure wake advertising data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure advertising parameters
    esp_ble_adv_params_t adv_params = {
        .adv_int_min = 0x20,
        .adv_int_max = 0x40,
        .adv_type = ADV_TYPE_IND,
        .channel_map = ADV_CHNL_ALL,
    };
    
    // Start advertising
    ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start wake advertising: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create timer to stop advertising after 2 seconds (as per documentation)
    if (adv_timer == NULL) {
        esp_timer_create_args_t timer_args = {
            .callback = &stop_adv_after_2s,
            .name = "wake_adv_timer"
        };
        esp_timer_create(&timer_args, &adv_timer);
    }
    
    esp_timer_start_once(adv_timer, 2000000);  // 2000ms = 2,000,000us
    
    ESP_LOGI(TAG, "Wake advertising started (will auto-stop after 2s)");
    return ESP_OK;
}

// Function to set target device for reconnection
void ble_set_target_device(const char* name, const uint8_t* mac) {
    if (name && mac) {
        // Set the target device name
        strncpy(s_remote_device_name, name, sizeof(s_remote_device_name) - 1);
        s_remote_device_name[sizeof(s_remote_device_name) - 1] = '\0';
        
        // Set the target MAC address
        memcpy(best_addr, mac, 6);
        
        ESP_LOGI(TAG, "Target device set: %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 s_remote_device_name,
                 best_addr[0], best_addr[1], best_addr[2],
                 best_addr[3], best_addr[4], best_addr[5]);
    }
}
