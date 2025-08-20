/*
 * DJI Camera Remote Control - Protocol Data Structures
 * 
 * This file implements the data structure definitions and utility functions
 * for the DJI camera communication protocol. It defines the binary layouts
 * for all command and response frames used in camera communication.
 * 
 * Structure Categories:
 * - Connection Management: Authentication and session establishment
 * - Recording Control: Photo and video capture commands
 * - Mode Management: Camera mode switching operations
 * - Power Management: Sleep and wake functionality
 * - Status Reporting: Camera state and telemetry data
 * 
 * All structures use packed attributes to ensure precise binary layout
 * matching the DJI protocol specification. Byte order is little-endian
 * to match ESP32 architecture and DJI camera expectations.
 * 
 * Key Features:
 * - Zero-padding elimination with __attribute__((packed))
 * - Consistent field ordering for protocol compatibility
 * - Clear separation between command and response structures
 * - Support for variable-length data payloads
 * 
 * These structures are used by the protocol parser and command logic
 * layers to ensure proper communication with DJI cameras.
 * 
 * Hardware: M5StickC Plus2 with ESP32 BLE capabilities
 * Protocol: DJI proprietary binary communication protocol
 * 
 * Based on DJI SDK implementation
 */

#include "dji_protocol_data_structures.h"
