# Code Examples

This directory contains example code snippets for using the various libraries.

## Display Examples

### Basic Text Display
```cpp
#include <Display.h>

Display display;

void setup() {
    display.begin();
    display.clear();
    display.print("Hello Robot!", 0, 0);
}
```

### Multi-line Display
```cpp
display.clear();
display.print("Robot Status:", 0, 0);
display.print("Active", 0, 16);
display.print("Battery: 95%", 0, 32);
```

### Progress Bar
```cpp
// Show OTA progress
for (int i = 0; i <= 100; i += 10) {
    display.drawProgressBar(i);
    delay(500);
}
```

## AHRS Examples

### Basic Motion Detection
```cpp
#include <AHRS.h>

AHRS ahrs;

void setup() {
    Serial.begin(115200);
    ahrs.begin();
}

void loop() {
    ahrs.update();
    
    if (ahrs.isMoving()) {
        Serial.println("Robot is moving!");
    }
    
    delay(100);
}
```

### Orientation Monitoring
```cpp
void loop() {
    ahrs.update();
    
    Serial.print("Roll: "); Serial.print(ahrs.getRoll());
    Serial.print(" Pitch: "); Serial.print(ahrs.getPitch());
    Serial.print(" Yaw: "); Serial.println(ahrs.getYaw());
    
    delay(100);
}
```

### Displacement Tracking
```cpp
void setup() {
    Serial.begin(115200);
    ahrs.begin();
    ahrs.resetDisplacement();  // Start from zero
}

void loop() {
    ahrs.update();
    
    Serial.print("Displacement X: ");
    Serial.print(ahrs.getDisplacementX());
    Serial.print(" m, Y: ");
    Serial.print(ahrs.getDisplacementY());
    Serial.println(" m");
    
    delay(500);
}
```

### Full Sensor Data
```cpp
void printSensorData() {
    ahrs.update();
    
    Serial.println("=== Accelerometer (m/s²) ===");
    Serial.printf("X: %.2f, Y: %.2f, Z: %.2f\n", 
                  ahrs.getAccelX(), ahrs.getAccelY(), ahrs.getAccelZ());
    
    Serial.println("=== Gyroscope (deg/s) ===");
    Serial.printf("X: %.2f, Y: %.2f, Z: %.2f\n", 
                  ahrs.getGyroX(), ahrs.getGyroY(), ahrs.getGyroZ());
    
    Serial.println("=== Magnetometer (uT) ===");
    Serial.printf("X: %.2f, Y: %.2f, Z: %.2f\n", 
                  ahrs.getMagX(), ahrs.getMagY(), ahrs.getMagZ());
    
    Serial.println("=== Orientation (degrees) ===");
    Serial.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", 
                  ahrs.getRoll(), ahrs.getPitch(), ahrs.getYaw());
    
    Serial.printf("Temperature: %.2f°C\n", ahrs.getTemperature());
}
```

### Calibration
```cpp
void setup() {
    Serial.begin(115200);
    ahrs.begin();
    
    // Calibrate accelerometer and gyroscope
    Serial.println("Keep sensor still for calibration...");
    ahrs.calibrateAccelGyro();
    
    // Calibrate magnetometer
    Serial.println("Wave sensor in figure-8 pattern...");
    ahrs.calibrateMag();
    
    Serial.println("Calibration complete!");
}
```

## Servo Control Examples

### Basic Movement
```cpp
#include <ServoControl.h>

ServoControl servos(32, 33);  // pins 32 and 33

void setup() {
    servos.begin();
    servos.moveDown(90);
    delay(1000);
    servos.moveUp(90);
}
```

### Smooth Movement
```cpp
void crawlForward() {
    // Move down servo from 0 to 90 smoothly
    servos.moveDownSmooth(90, 10);  // 10ms delay between steps
    delay(500);
    
    // Move up servo
    servos.moveUpSmooth(90, 10);
    delay(500);
    
    // Return to initial position
    servos.setInitialPosition();
}
```

### Position Tracking
```cpp
void loop() {
    Serial.print("Down servo: ");
    Serial.print(servos.getCurrentDownAngle());
    Serial.print("°, Up servo: ");
    Serial.print(servos.getCurrentUpAngle());
    Serial.println("°");
    
    delay(1000);
}
```

### Crawling Pattern
```cpp
void crawlCycle() {
    // Lift front
    servos.moveUpSmooth(45, 10);
    delay(200);
    
    // Push back
    servos.moveDownSmooth(135, 10);
    delay(200);
    
    // Lower front
    servos.moveUpSmooth(90, 10);
    delay(200);
    
    // Pull forward
    servos.moveDownSmooth(45, 10);
    delay(200);
    
    // Reset
    servos.setInitialPosition();
    delay(500);
}
```

## Network Examples

### Basic Setup
```cpp
#include <Network.h>
#include <Display.h>

Display display;
Network* network;

void setup() {
    Serial.begin(115200);
    display.begin();
    
    network = new Network(&display);
    network->begin();
    network->startOTATask();
    
    Serial.print("Robot #");
    Serial.println(network->getRobotNumber());
    Serial.print("Connect to: ");
    Serial.println(network->getSSID());
}
```

### Display Network Info
```cpp
void showNetworkInfo() {
    display.clear();
    display.print("Robot #", 0, 0);
    display.print(network->getRobotNumber());
    
    display.setCursor(0, 16);
    display.print(network->getSSID());
    
    display.setCursor(0, 32);
    display.print("OTA Ready");
}
```

## Training Examples (Template)

### Basic Training Structure
```cpp
#include <Training.h>

Training training;

void setup() {
    Serial.begin(115200);
    training.begin();
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        if (cmd == 't') {
            // Start training
            training.startTraining();
        } else if (cmd == 's') {
            // Stop training
            training.stopTraining();
        } else if (cmd == 'e') {
            // Execute learned behavior
            if (training.hasLearnedBehavior()) {
                training.executeLearnedBehavior();
            }
        }
    }
}
```

## Complete Integration Example

### Robot Control Loop
```cpp
#include <Arduino.h>
#include <Display.h>
#include <AHRS.h>
#include <ServoControl.h>
#include <Network.h>
#include <Training.h>

Display display;
AHRS ahrs;
ServoControl servos(32, 33);
Network* network;
Training training;

unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 100;  // 100ms

void setup() {
    Serial.begin(115200);
    
    display.begin();
    network = new Network(&display);
    network->begin();
    network->startOTATask();
    
    ahrs.begin();
    servos.begin();
    training.begin();
}

void loop() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastUpdate >= UPDATE_INTERVAL) {
        lastUpdate = currentTime;
        
        // Update sensors
        ahrs.update();
        
        // Display status
        updateDisplay();
        
        // Check for movement
        if (ahrs.isMoving()) {
            // Robot is being moved externally
            Serial.println("External motion detected");
        }
        
        // Execute learned behavior if training is done
        if (!training.isTraining() && training.hasLearnedBehavior()) {
            training.executeLearnedBehavior();
        }
    }
}

void updateDisplay() {
    display.clear();
    display.print("Robot #", 0, 0);
    display.print(network->getRobotNumber());
    
    display.setCursor(0, 16);
    display.print("Yaw: ");
    display.print((int)ahrs.getYaw());
    
    display.setCursor(0, 32);
    if (training.isTraining()) {
        display.print("Training...");
    } else {
        display.print("Ready");
    }
}
```

## Sensor-Based Movement Example

```cpp
void reactiveMovement() {
    ahrs.update();
    
    float pitch = ahrs.getPitch();
    
    // Adjust servos based on pitch (tilt)
    if (pitch > 10) {
        // Tilted forward - compensate
        servos.moveDownSmooth(60, 5);
    } else if (pitch < -10) {
        // Tilted backward - compensate
        servos.moveDownSmooth(120, 5);
    } else {
        // Level - normal position
        servos.moveDownSmooth(90, 5);
    }
}
```
