# Quick Start Guide

Get your crawling bot up and running in minutes!

## Prerequisites

- ESP32-S3-DevKitM-1 board
- OLED 0.96" display (SSD1306, I2C)
- MPU9250 AHRS sensor (I2C)
- 2x Servo motors
- PlatformIO installed

## Hardware Setup

### Wiring

**I2C Connections (OLED & MPU9250):**
- SDA → GPIO 21 (default ESP32 SDA)
- SCL → GPIO 22 (default ESP32 SCL)
- VCC → 3.3V
- GND → GND

**Servos:**
- Servo Down → GPIO 32
- Servo Up → GPIO 33
- VCC → 5V (external power recommended)
- GND → GND

**Power:**
- ESP32-S3 via USB-C
- Servos via external 5V power supply (recommended for stable operation)

### I2C Addresses
- OLED Display: 0x3C
- MPU9250: 0x68

## Software Setup

### 1. Clone and Open Project

```bash
git clone https://github.com/amirali-lll/crawling-bot-v2.git
cd crawling-bot-v2
```

Open in VS Code with PlatformIO extension or use command line.

### 2. Build

```bash
pio run
```

### 3. Upload

Connect ESP32-S3 via USB and upload:

```bash
pio run --target upload
```

### 4. Monitor

```bash
pio device monitor -b 115200
```

### 5. Configure Robot Number

On first boot, the robot will ask for a number (1-8):
1. Open serial monitor (115200 baud)
2. Enter a number between 1 and 8
3. Press Enter

The number is saved to EEPROM and used for WiFi AP name.

## First Run

After uploading, the robot will:

1. **Initialize Display** - Shows "RL Robot V2"
2. **Setup Network** - Creates WiFi AP (ESP32-AP-{number})
3. **Initialize AHRS** - Calibrates MPU9250
4. **Initialize Servos** - Sets to initial position
5. **Health Check** - Tests all components
6. **Main Loop** - Displays status and sensor data

Watch the OLED display for status messages!

## WiFi Access Point

The robot creates a WiFi AP:
- **SSID:** `ESP32-AP-{robot_number}`
- **Password:** `12345678`
- **IP:** `192.168.4.1`

## OTA Updates (After First Upload)

1. Connect to robot's WiFi AP
2. Upload via OTA:
   ```bash
   pio run --target upload --upload-port ESP32-OTA-{number}.local
   ```

## Verify Everything Works

### On OLED Display

You should see:
```
RL Robot V2
Initializing...

Robot #X
Setup...

Init AHRS...
AHRS OK

Init Servos...
Servos OK

Health Check
...

Robot #X
Running...
Y:123 T:25C
```

### On Serial Monitor (115200 baud)

```
AP Started
AP SSID: ESP32-AP-1
IP Address: 192.168.4.1
OTA Ready
OTA Hostname: ESP32-OTA-1
MPU9250 initialized successfully
Training module initialized (empty - to be implemented)
```

### Physical Behavior

During health check:
1. Display shows "Health Check"
2. Servos move to 90° (test position)
3. Servos return to initial position (down=0°, up=180°)
4. Display shows orientation data

## Troubleshooting

### Display Not Working
- Check I2C wiring (SDA/SCL)
- Verify display address is 0x3C
- Try `i2cdetect` to scan I2C bus

### AHRS Failed
- Check MPU9250 wiring
- Verify sensor address is 0x68
- Ensure sensor has power (3.3V)

### Servos Not Moving
- Check power supply (5V, adequate current)
- Verify GPIO connections (32, 33)
- Check servo signal wires

### Can't Upload
- Install CH340 or CP2102 USB drivers
- Check USB cable (must support data)
- Verify board is in bootloader mode
- Try pressing BOOT button while uploading

### Robot Number Not Saving
- EEPROM initialization might have failed
- Try re-uploading firmware
- Check serial monitor for errors

## Next Steps

Now that your robot is running:

1. **Test AHRS** - Move the robot and watch orientation change
2. **Test Servos** - Modify code to test different positions
3. **Calibrate MPU9250** - For better accuracy (see BUILD.md)
4. **Implement Training** - Add your RL algorithm to Training library
5. **Customize Behavior** - Modify main loop for your use case

## Need Help?

Check these files:
- `README.md` - Full project documentation
- `BUILD.md` - Detailed build instructions
- `MIGRATION.md` - Migration from old code
- `EXAMPLES.md` - Code examples
- `lib/*/` - Library source code

## Safety Notes

⚠️ **Important:**
- Always use external power for servos (not USB power)
- Don't overload the ESP32 GPIO pins
- Keep wiring away from moving parts
- Test new code with servos disconnected first
- Monitor temperature - MPU9250 can get warm

## Have Fun!

Your crawling robot is ready to learn and explore! 🤖

Happy coding! 🚀
