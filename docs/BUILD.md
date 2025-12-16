# Build Instructions

## Prerequisites

1. Install PlatformIO Core or PlatformIO IDE
   - **PlatformIO Core**: `pip install platformio`
   - **PlatformIO IDE**: Install VS Code extension "PlatformIO IDE"

## Building the Project

### Using PlatformIO Core (Command Line)

```bash
# Navigate to project directory
cd crawling-bot-v2

# Build the project
pio run

# Upload to ESP32-S3
pio run --target upload

# Monitor serial output
pio device monitor -b 115200

# Build, upload, and monitor in one command
pio run --target upload && pio device monitor -b 115200
```

### Using PlatformIO IDE (VS Code)

1. Open the project folder in VS Code
2. PlatformIO will automatically detect the project
3. Use the PlatformIO toolbar at the bottom:
   - Click "Build" (✓) to compile
   - Click "Upload" (→) to flash to board
   - Click "Serial Monitor" to view output

## First-Time Setup

On first boot, the robot will prompt for a robot number (1-8) via Serial Monitor:

1. Connect to Serial Monitor at 115200 baud
2. Enter a number between 1 and 8
3. The number will be saved to EEPROM
4. Robot will create AP with SSID: `ESP32-AP-{number}`
5. OTA hostname will be: `ESP32-OTA-{number}`

## Hardware Connections

### I2C Devices (SDA/SCL - Default ESP32 pins)
- OLED Display (SSD1306) - Address: 0x3C
- MPU9250 AHRS Sensor - Address: 0x68

### Servos
- Servo Down - GPIO 32
- Servo Up - GPIO 33

### Power
- ESP32-S3 via USB-C or external 5V power
- Servos should have adequate power supply

## Troubleshooting

### Build Issues

1. **Platform not found**: Run `pio pkg install` to download ESP32 platform
2. **Library errors**: Dependencies are auto-installed from platformio.ini
3. **Compilation errors**: Ensure you have the latest PlatformIO version

### Runtime Issues

1. **OLED not detected**: Check I2C connections and address (0x3C)
2. **MPU9250 not found**: Verify I2C address (0x68) and connections
3. **Servos not moving**: Check power supply and GPIO pins
4. **OTA not working**: Ensure you're connected to the robot's AP

## Development Workflow

1. Make code changes
2. Build: `pio run`
3. Fix any compilation errors
4. Upload: `pio run --target upload`
5. Monitor serial output: `pio device monitor -b 115200`
6. Test functionality
7. For subsequent updates, use OTA when possible

## OTA Updates (After Initial Upload)

1. Connect to robot's WiFi AP: `ESP32-AP-{robot_number}`
2. Password: `12345678`
3. Upload via OTA:
   ```bash
   pio run --target upload --upload-port ESP32-OTA-{robot_number}.local
   ```

## Calibration

For best AHRS performance, calibrate the MPU9250:

1. Modify main.cpp to add calibration calls in setup():
   ```cpp
   ahrs.calibrateAccelGyro();  // Keep sensor still
   ahrs.calibrateMag();         // Wave in figure-8 pattern
   ```
2. Upload and follow serial instructions
3. Remove calibration calls after completion

## Testing

The health check runs automatically on startup and tests:
- OLED display
- AHRS sensor (orientation)
- Servo motors (movement)

Monitor the OLED display and serial output for status.
