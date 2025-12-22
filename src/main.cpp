#include <Arduino.h>
#include <Display.h>
#include <AHRS.h>
#include <ServoControl.h>
#include <Network.h>
#include <Training.h>
#include <HealthCheck.h>

// Pin definitions
const uint8_t SERVO_PIN_DOWN = 16; // P1 on board
const uint8_t SERVO_PIN_UP = 15;   // P2 on board

// Global objects
Display display;
AHRS ahrs;
ServoControl servoControl(SERVO_PIN_DOWN, SERVO_PIN_UP);    // when we call this the constructor of servocontrol is called.
Network *network;
Training training;
HealthCheck healthCheck(&display, &ahrs, &servoControl);


// this major method: Runs once at boot.Used for initialization.
void setup()
{
    // This lets the robot print messages to your laptop.Used for debugging.
    Serial.begin(115200);
    delay(1000);

    // This turns on the OLED screen so you can show logs.
    display.begin();
    display.clear();
    display.print("RL Robot V2", 0, 0);

    display.setCursor(0, 16);
    display.print("Initializing...");
    delay(1000);
    
    // From now on you can upload code wirelessly.
    network = new Network(&display);
    network->begin();
    network->startOTATask();

    // Robot reads its ID from EEPROM.
    display.clear();
    display.print("Robot #", 0, 0);
    display.print(network->getRobotNumber());

    display.setCursor(0, 16);
    display.print("Setup...");
    delay(1000);

    // AHRS
    display.clear();
    display.print("Init AHRS...", 0, 0);

    if (ahrs.begin())      //ahrs.begin(): This activates the MPU9250 sensor and enables:orientation (roll/pitch/yaw),velocity,displacement,acceleration, movement detection
        display.print("AHRS OK", 0, 16);
    else
        display.print("AHRS Failed!", 0, 16);
    delay(1000);


    // Servos
    display.clear();
    display.print("Init Servos...", 0, 0);
    servoControl.begin();   // This attaches servo motors and moves them to their starting angle: (180,180) this makes the down servo to point the up and the up servo point to down.
    display.print("Servos OK", 0, 16);
    delay(1000);

    // Prepares RL system (currently empty but we will write it).
    training.begin();
 
    display.clear();
    display.print("Setup Complete", 0, 0);
    delay(2000);

    healthCheck.run();

    display.clear();   //clears the screen BUFFER
    display.print("Hello World", 0, 0);  //writes text to BUFFER
    display.refresh();  //shows it on OLED
    delay(2000);

    // Multiple servo moves
    servoControl.moveDown(60);
    delay(500);

    servoControl.moveUp(120);
    delay(700);

    servoControl.moveDown(90);
    servoControl.moveUp(90);
    delay(500);

    int fast_move_count = 2;
    for (int i = 0; i <fast_move_count; i++) {
        servoControl.moveDown(60);
        servoControl.moveUp(120);
        delay(400);
        servoControl.moveDown(90);
        servoControl.moveUp(90);
        delay(400);
    }

    // This clears displacement, speed averages.Useful before training starts.
    ahrs.resetMeasurement();
}
void loop()
{
    // Update AHRS:This reads sensors and calculates:acceleration,velocity,displacement,orientation,speed
    ahrs.update();



    // ===== MODE 1: Real-time continuous display =====
    // Uncomment this section to show current instantaneous values
    /*
    display.clear();
    // Line 1: Speed (cm/s)
    display.setCursor(0, 0);
    display.print("Spd: ");
    display.print(ahrs.getSpeed() * 100, 1);
    display.print(" cm/s");
    // Line 2: Acceleration (m/s^2)
    display.setCursor(0, 12);
    display.print("Acc: ");
    display.print(ahrs.getAccelMagnitude(), 2);
    display.print(" m/s2");
    // Line 3: Displacement X (cm)
    display.setCursor(0, 24);
    display.print("X: ");
    display.print(ahrs.getDisplacementX() * 100, 1);
    display.print(" cm");
    // Line 4: Displacement Y (cm)
    display.setCursor(0, 36);
    display.print("Y: ");
    display.print(ahrs.getDisplacementY() * 100, 1);
    display.print(" cm");

    display.refresh();
    delay(1000);
    */



    // ===== MODE 2: Interval measurement (every 2 seconds) =====
    // This shows average movement parameters since last measurement
    static unsigned long lastMeasurement = 0;
    unsigned long currentTime = millis();

    // if (currentTime - lastMeasurement >= 2000)
    // {
    //      Every 2 seconds
    //     lastMeasurement = currentTime;

    //     // Get measurement data
    //     AHRS::MovementSnapshot measurement = ahrs.getMeasurement();

    //     // Display the interval data
    //     display.clear();

    //     // Line 1: Distance moved in interval (cm)
    //     display.setCursor(0, 0);
    //     display.print("Dist: ");
    //     display.print(measurement.deltaDistance, 1);
    //     display.print(" cm");

    //     // Line 2: Average speed in interval (cm/s)
    //     display.setCursor(1, 0);
    //     display.print("Spd: ");
    //     display.print(measurement.avgSpeed, 1);
    //     display.print(" cm/s");

    //     // Line 3: Average acceleration in interval (m/s^2)
    //     display.setCursor(2, 0);
    //     display.print("Acc: ");
    //     display.print(measurement.avgAcceleration, 2);
    //     display.print(" m/s2");

    //     // Line 4: Time interval
    //     display.setCursor(3, 0);
    //     display.print("Time: ");
    //     display.print(measurement.deltaTime, 1);
    //     display.print(" s");

    //     display.refresh();

    //     // Reset for next measurement
    //     ahrs.resetMeasurement();

    //     // Print to serial for debugging
    //     Serial.print("Distance: ");
    //     Serial.print(measurement.deltaDistance);
    //     Serial.print(" cm, Speed: ");
    //     Serial.print(measurement.avgSpeed);
    //     Serial.print(" cm/s, Accel: ");
    //     Serial.print(measurement.avgAcceleration);
    //     Serial.println(" m/s2");
    // }

    // TODO: Implement main loop logic
    // - Read sensors
    // - Process data
    // - Execute training if active
    // - Execute learned behavior
    // - Control servos
}
