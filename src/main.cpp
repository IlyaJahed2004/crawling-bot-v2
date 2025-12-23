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
    static unsigned long lastStep = 0;
    static int steps = 0;
    static float avgReward = 0.0f;

    const int TRAINING_STEPS = 200;
    const unsigned long STEP_INTERVAL = 1200;

    if (millis() - lastStep < STEP_INTERVAL) return;
    lastStep = millis();

    // 1. Sensor update
    ahrs.update();

    // 2. Build discrete state from speed
    float speed = ahrs.getSpeed() * 100.0f; // cm/s
    uint8_t state;

    if (speed < 2.0f) state = 0;
    else if (speed < 8.0f) state = 1;
    else state = 2;

    // 3. Choose action
    uint8_t action = training.selectAction(state);

    // 4. Execute action (servo gait)
    if (action == 0) {
        servoControl.moveDown(70);
        servoControl.moveUp(110);
    } else if (action == 1) {
        servoControl.moveDown(90);
        servoControl.moveUp(90);
    } else {
        servoControl.moveDown(120);
        servoControl.moveUp(60);
    }

    delay(600);

    // 5. Observe result
    ahrs.update();
    float newSpeed = ahrs.getSpeed() * 100.0f;

    uint8_t nextState;
    if (newSpeed < 2.0f) nextState = 0;
    else if (newSpeed < 8.0f) nextState = 1;
    else nextState = 2;

    float reward = newSpeed;

    // 6. Learn
    training.updateQ(state, action, reward, nextState);

    // 7. Auto-freeze training
    if (steps >= TRAINING_STEPS && training.isTraining()) {
        training.stopTraining();
        training.saveModel();
    }

    avgReward = (avgReward * steps + reward) / (steps + 1);
    steps++;

    // 8. OLED feedback (CRITICAL)
    display.clear();

    display.setCursor(0, 0);
    display.print(training.isTraining() ? "MODE: TRAIN" : "MODE: EXEC");

    display.setCursor(0, 12);
    display.print("AvgR:");
    display.print(avgReward, 1);

    display.setCursor(0, 24);
    display.print("Act:");
    display.print(action);

    display.setCursor(64, 24);
    display.print("Spd:");
    display.print(newSpeed, 1);

    display.setCursor(0, 36);
    display.print("eps:");
    display.print(training.getEpsilon(), 2);

    display.refresh();
}

