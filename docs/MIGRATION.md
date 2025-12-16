# Migration Guide: Arduino to PlatformIO

## Summary of Changes

This document outlines the key differences between the old Arduino template and the new PlatformIO implementation.

## Hardware Changes

| Component | Old (Arduino) | New (PlatformIO) |
|-----------|---------------|------------------|
| Display | LCD 16x2 (I2C) | OLED 0.96" SSD1306 |
| Distance Sensor | SRF-05 Ultrasonic | MPU9250 AHRS (9-DOF IMU) |
| Board | ESP32 Dev Module | ESP32-S3-DevKitM-1 |
| Servos | 2x Servo Motors | 2x Servo Motors (same) |

## Code Architecture Changes

### Old Structure (Monolithic)
```
old-template.ino
└── All code in single file
    ├── Global variables
    ├── Helper functions
    ├── setup()
    └── loop()
```

### New Structure (Library-Based)
```
src/main.cpp              # Main application
lib/
├── Display/              # OLED display management
├── AHRS/                 # MPU9250 sensor & motion tracking
├── ServoControl/         # Servo motor control
├── Network/              # WiFi AP & OTA updates
└── Training/             # RL training (template)
```

## API Changes

### Display

**Old (LCD):**
```cpp
LiquidCrystal_I2C lcd(0x27, 16, 2);
lcd.init();
lcd.backlight();
lcd.clear();
lcd.print("Hello");
lcd.setCursor(0, 1);
lcd.print("World");
```

**New (OLED):**
```cpp
Display display;
display.begin();
display.clear();
display.print("Hello", 0, 0);
display.print("World", 0, 16);
```

### Distance/Motion Sensing

**Old (SRF-05):**
```cpp
NewPing sonar(triggerPin, echoPin, MAX_DISTANCE);
float distance = sonar.ping_cm();
if (distance == 0) {
    // No object detected
}
```

**New (MPU9250 AHRS):**
```cpp
AHRS ahrs;
ahrs.begin();
ahrs.update();  // Call in loop

// Motion detection
bool moving = ahrs.isMoving();

// Orientation
float roll = ahrs.getRoll();
float pitch = ahrs.getPitch();
float yaw = ahrs.getYaw();

// Displacement tracking
float dx = ahrs.getDisplacementX();
float dy = ahrs.getDisplacementY();

// Raw sensor data
float accelX = ahrs.getAccelX();
float gyroZ = ahrs.getGyroZ();
float magX = ahrs.getMagX();
```

### Servo Control

**Old:**
```cpp
Servo servodown;
Servo servoup;
servodown.attach(control_servo_down, 600, 2400);
servoup.attach(control_servo_up, 600, 2400);
servodown.write(90);

// Smooth movement
moveServoSmooth(servodown, 0, 90, 10);
```

**New:**
```cpp
ServoControl servos(SERVO_PIN_DOWN, SERVO_PIN_UP);
servos.begin();
servos.moveDown(90);

// Smooth movement
servos.moveDownSmooth(90, 10);
servos.setInitialPosition();
```

### Network & OTA

**Old:**
```cpp
// Separate functions
setup_ap();
setup_ota();

// Manual task creation
xTaskCreatePinnedToCore(otaTask, "OTATask", ...);
```

**New:**
```cpp
Network network(&display);
network.begin();          // Sets up AP and OTA
network.startOTATask();   // Starts FreeRTOS task
uint8_t num = network.getRobotNumber();
```

## Pin Definitions

**Old:**
```cpp
const int triggerPin = 5;        // SRF-05 trigger
const int echoPin = 17;          // SRF-05 echo
const int control_servo_down = 32;
const int control_servo_up = 33;
```

**New:**
```cpp
// config.h
#define SERVO_PIN_DOWN 32
#define SERVO_PIN_UP 33
// I2C pins are defaults (SDA: 21, SCL: 22)
// No ultrasonic sensor pins needed
```

## Feature Improvements

### 1. Enhanced Sensing Capabilities

The MPU9250 provides far more data than the SRF-05:

**Old capabilities:**
- Distance measurement only
- Limited to line-of-sight

**New capabilities:**
- 9-DOF motion sensing (accel, gyro, mag)
- Orientation (roll, pitch, yaw)
- Quaternion representation
- Displacement tracking
- Motion detection
- Temperature sensing
- No line-of-sight required

### 2. Better Display

**Old (LCD 16x2):**
- 16 characters × 2 lines
- Text only
- Backlight control

**New (OLED 0.96"):**
- 128×64 pixels
- Graphics capable
- Progress bars
- Better contrast
- More information density

### 3. Modular Architecture

**Benefits:**
- Easier to test individual components
- Better code organization
- Reusable libraries
- Simpler maintenance
- Clear separation of concerns

### 4. Training Framework

**Old:**
```cpp
void doTraining() {
    // TODO
}
```

**New:**
```cpp
Training training;
training.begin();
training.startTraining();
training.executeLearnedBehavior();
// Well-defined interface for future RL implementation
```

## Migration Steps for Developers

1. **Update platformio.ini** with new dependencies
2. **Create library structure** in `lib/` directory
3. **Port display code** from LCD to OLED API
4. **Replace distance sensing** with AHRS motion tracking
5. **Wrap servo control** in dedicated class
6. **Encapsulate networking** in Network class
7. **Update main.cpp** to use new libraries
8. **Test each module** independently
9. **Implement training** when ready (currently placeholder)

## Backward Compatibility Notes

This is a **breaking change** - the old code will not run on the new hardware without modification because:

1. Different display driver (LCD → OLED)
2. Different sensor (ultrasonic → IMU)
3. Different library dependencies
4. Different architecture

However, all **functionality** has been preserved or enhanced:
- ✅ Display output
- ✅ Motion/distance detection (improved)
- ✅ Servo control
- ✅ WiFi AP
- ✅ OTA updates
- ✅ Robot number configuration
- ✅ Health checks
- ✅ Training framework (ready for implementation)

## Build Process Changes

**Old:**
- Arduino IDE
- Manual library installation
- Board selection in IDE
- Upload via USB

**New:**
- PlatformIO (CLI or IDE)
- Automatic dependency management
- Board configured in platformio.ini
- Upload via USB or OTA

## Next Steps

After migration:
1. Test all hardware components
2. Calibrate MPU9250 for accurate readings
3. Implement training algorithms in Training library
4. Fine-tune servo movements for your robot's mechanics
5. Customize display output for your use case

## Questions?

Refer to:
- `README.md` - General project documentation
- `BUILD.md` - Build and deployment instructions
- Library source files for API details
