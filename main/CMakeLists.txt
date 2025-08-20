idf_component_register(SRCS "app_main.c" 
                            "ui.c"
                            "icons.c"
                            "../utils/crc/custom_crc16.c" 
                            "../utils/crc/custom_crc32.c"
                            "../protocol/dji_protocol_parser.c"
                            "../protocol/dji_protocol_data_processor.c"
                            "../protocol/dji_protocol_data_descriptors.c"
                            "../protocol/dji_protocol_data_structures.c"
                            "../ble/ble.c"
                            "../data/data.c"
                            "../logic/connect_logic.c"
                            "../logic/command_logic.c"
                            "../logic/status_logic.c"
                            "../logic/enums_logic.c"
                            "../logic/light_logic.c"
                            "m5stickc_plus2_hal.c"
                    PRIV_REQUIRES bt nvs_flash esp_driver_gpio led_strip esp_driver_spi esp_driver_i2c esp_lcd driver
                    INCLUDE_DIRS "." "../utils/crc" "../protocol" "../ble" "../data" "../logic")

# Define M5STICKC_PLUS2 for conditional compilation
target_compile_definitions(${COMPONENT_LIB} PUBLIC M5STICKC_PLUS2)
