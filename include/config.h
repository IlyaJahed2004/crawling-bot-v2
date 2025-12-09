#ifndef CONFIG_H
#define CONFIG_H

// Pin Configuration
#define SERVO_PIN_DOWN 32
#define SERVO_PIN_UP 33

// I2C Configuration (using default ESP32 pins)
// SDA - GPIO 21 (default)
// SCL - GPIO 22 (default)

// Display Configuration
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDRESS 0x3C

// MPU9250 Configuration
#define MPU9250_ADDRESS 0x68

// Network Configuration
#define AP_PASSWORD "12345678"
#define SERIAL_BAUD_RATE 115200

// EEPROM Configuration
#define EEPROM_SIZE 1
#define ROBOT_NUM_ADDR 0
#define MIN_ROBOT_NUM 1
#define MAX_ROBOT_NUM 8

#endif // CONFIG_H
