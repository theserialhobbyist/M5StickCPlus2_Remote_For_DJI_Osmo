## Description

![PXL_20250820_213936612](https://github.com/user-attachments/assets/153c29c2-81bd-451d-8bd2-8c7f80474369)

M5StickC Plus2 remote control for DJI Osmo Action 5 Pro cameras.

Switch modes, start/stop recording, capture photo, sleep, wake, automatic reconnection, and GPIO control. 

GPS not supported in this version!

More info here: https://serialhobbyism.com/open-source-diy-remote-for-dji-osmo-action-5-pro-cameras

Demo on YouTube: https://youtube.com/shorts/t92D7x2sBuA?feature=share

## Hardware Specifications

The M5StickC Plus2 features:
- **MCU**: ESP32-PICO-V3-02 (Dual-core ESP32, 240MHz)
- **Display**: 1.14" TFT LCD (135x240 pixels, ST7789V2 driver)
- **Flash**: 4MB
- **PSRAM**: None
- **Buttons**: 3 (Button A/Home, Button B/Side, Power)
- **IMU**: MPU6886 (6-axis)
- **RTC**: BM8563
- **PMU**: AXP192
- **Buzzer**: Built-in
- **IR Transmitter**: Built-in
- **LED**: Internal red LED
- **Grove Port**: For external GPS module connection

## Pin Mappings

### Display (ST7789V2)
- MOSI: GPIO 15
- SCLK: GPIO 13
- CS: GPIO 5
- DC: GPIO 14
- RST: GPIO 12
- Backlight: GPIO 27

### Buttons
- Button A (Home): GPIO 37
- Button B (Side): GPIO 39
- Power Button: GPIO 35

### I2C (IMU, RTC, PMU)
- SDA: GPIO 21
- SCL: GPIO 22

### Grove Port (UART for GPS)
- TX: GPIO 32
- RX: GPIO 33

### Other
- Internal LED: GPIO 19
- Buzzer: GPIO 2
- IR: GPIO 9
- Power Enable: GPIO 2

## Building the Project

### Prerequisites
1. Install ESP-IDF v5.0 or later
2. Set up the ESP-IDF environment:
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```
   Or on Windows:
   ```cmd
   %IDF_PATH%\export.bat
   ```

### Build Steps

#### IMPORTANT: Set the correct target first!

1. Clean any previous build:
   ```bash
   idf.py fullclean
   ```

2. **CRITICAL**: Set target to ESP32 for M5StickC Plus2:
   ```bash
   idf.py set-target esp32
   ```
   
   Note: M5StickC Plus2 uses ESP32-PICO-V3-02, which is a standard ESP32 chip.

3. Copy the M5StickC Plus2 specific configuration:
   ```bash
   cp sdkconfig.defaults.m5stickc_plus2 sdkconfig.defaults
   ```
   Or on Windows:
   ```cmd
   copy sdkconfig.defaults.m5stickc_plus2 sdkconfig.defaults
   ```

4. Configure the project (optional):
   ```bash
   idf.py menuconfig
   ```

5. Build the project:
   ```bash
   idf.py build
   ```

6. Flash to M5StickC Plus2:
   ```bash
   idf.py -p COM[X] flash monitor
   ```
   Replace `COM[X]` with your device's serial port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)

## Features

### Display
The M5StickC Plus2's display shows:
- Boot status and initialization progress
- BLE connection status
- GPS status
- Camera connection status

### Button Functions
- **Button A (GPIO 37)**: Main control button - single press for commands, long press for reconnection
- **Button B (GPIO 39)**: Status display and secondary functions
- **Power Button (GPIO 35)**: Power management

### LED Indicators
The internal LED (GPIO 19) provides visual feedback:
- Different colors/patterns indicate various states
- Controlled via the light_logic module

## Troubleshooting

### Display Issues
- Ensure the display driver (ST7789) is properly initialized
- Check backlight GPIO (27) is set high
- Verify SPI pins are correctly configured

### Button Not Responding
- M5StickC Plus2 buttons are active low (pressed = 0)
- Internal pull-ups are enabled
- Check GPIO 37 (Button A), GPIO 39 (Button B), GPIO 35 (Power)

### Build Errors
- Ensure ESP-IDF v5.0+ is installed
- Target must be set to esp32s3
- Clean build directory if switching from another target

## Power Management
The M5StickC Plus2 includes an AXP192 power management IC:
- I2C address: 0x34
- Controls battery charging, power rails, and system power
- Power enable pin: GPIO 2

## Additional Resources
- [M5StickC Plus2 Documentation](https://docs.m5stack.com/en/core/M5StickC%20Plus2)
- [ESP32-S3 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)
