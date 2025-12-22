// ServoControl is the blueprint:the class methods and properties are prototyped there:
#include "ServoControl.h"

// constructor of servocontrol class: Saves pin numbers,Sets starting angles (default 180°)
ServoControl::ServoControl(uint8_t pinDown, uint8_t pinUp) 
    : pinDown(pinDown), pinUp(pinUp), 
      currentDownAngle(INITIAL_DOWN_ANGLE), 
      currentUpAngle(INITIAL_UP_ANGLE) {
}

//implementing begin method: Connects servo objects to actual ESP32 pins, defines allowable pulse widths, moves both servos to initial angles:
void ServoControl::begin() {
    //servodown and servoup are servo objects which are mapped to real servos on the crawler robot.
    servoDown.attach(pinDown, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    servoUp.attach(pinUp, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    setInitialPosition();  //see its implementation.this just sets the servos to its default place.
}


//implementing movedown method: Moves the down servo directly to an angle (0–180), Fast movement with no smoothing
void ServoControl::moveDown(int angle) {
    angle = constrain(angle, 0, 180);   //constrain is a macro: If amt < low → return low,else if amt > high → return high, Otherwise → return amt
    servoDown.write(angle);             //If value < 500 → treat value as angleClamp angle between 0–180,Convert angle to pulse width using map(),Send final pulse width signal to servo
    currentDownAngle = angle;
}


// Same as moveDown but for the upper joint.
void ServoControl::moveUp(int angle) {
    angle = constrain(angle, 0, 180);
    servoUp.write(angle);
    currentUpAngle = angle;
}

// Move from current angle → target angle in steps of 2°, Delay between steps (default 10 ms), Creates a natural "slow motion" servo effect
void ServoControl::moveDownSmooth(int targetAngle, int stepDelay) {
    targetAngle = constrain(targetAngle, 0, 180);
    moveServoSmooth(servoDown, currentDownAngle, targetAngle, stepDelay);
}

void ServoControl::moveUpSmooth(int targetAngle, int stepDelay) {
    targetAngle = constrain(targetAngle, 0, 180);
    moveServoSmooth(servoUp, currentUpAngle, targetAngle, stepDelay);
}


void ServoControl::setInitialPosition() {
    //these two consntants are declared in servocontrol.h:INITIAL_DOWN_ANGLE,INITIAL_UP_ANGLE
    moveDown(INITIAL_DOWN_ANGLE);
    moveUp(INITIAL_UP_ANGLE);
}

void ServoControl::setTestPosition() {
    moveDown(90);
    moveUp(90);
}

int ServoControl::getCurrentDownAngle() {
    return currentDownAngle;
}

int ServoControl::getCurrentUpAngle() {
    return currentUpAngle;
}

void ServoControl::moveServoSmooth(Servo &servo, int &currentAngle, int targetAngle, int stepDelay) {
    if (currentAngle < targetAngle) {
        for (int angle = currentAngle; angle <= targetAngle; angle += 2) {
            servo.write(angle);
            delay(stepDelay);
        }
    } else {
        for (int angle = currentAngle; angle >= targetAngle; angle -= 2) {
            servo.write(angle);
            delay(stepDelay);
        }
    }
    // Ensure we reach the exact target angle
    servo.write(targetAngle);
    currentAngle = targetAngle;
}
