#include "AHRS.h"

AHRS::AHRS() : initialized(false), calibrated(false),
               velocityX(0), velocityY(0), velocityZ(0),
               displacementX(0), displacementY(0), displacementZ(0),
               lastUpdateTime(0),
               lastSnapshotX(0), lastSnapshotY(0), lastSnapshotZ(0),
               lastSnapshotTime(0), speedSum(0), accelSum(0), sampleCount(0)
{
    mpu = new MPU9250();
}

AHRS::~AHRS()
{
    delete mpu;
}

bool AHRS::begin()
{
    Wire.begin();

    if (!mpu->setup(0x68))
    {
        Serial.println("MPU9250 connection failed");
        initialized = false;
        return false;
    }

    Serial.println("MPU9250 initialized successfully");
    initialized = true;
    lastUpdateTime = millis();
    lastSnapshotTime = millis();

    return true;
}

void AHRS::update()
{
    if (!initialized)
        return;

    if (mpu->update())
    {
        unsigned long currentTime = millis();
        float dt = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds
        lastUpdateTime = currentTime;

        // Update velocity and displacement using acceleration
        // Note: This is a simple integration and may drift over time
        // For better accuracy, additional filtering should be applied
        if (dt > 0 && dt < 1.0)
        { // Sanity check
            float accelX = getAccelX();
            float accelY = getAccelY();
            float accelZ = getAccelZ();

            velocityX += accelX * dt;
            velocityY += accelY * dt;
            velocityZ += accelZ * dt;

            displacementX += velocityX * dt;
            displacementY += velocityY * dt;
            displacementZ += velocityZ * dt;

            // Accumulate for averaging
            speedSum += getSpeed();
            accelSum += getAccelMagnitude();
            sampleCount++;
        }
    }
}

bool AHRS::isMoving()
{
    if (!initialized)
        return false;

    float totalAccel = sqrt(
        getAccelX() * getAccelX() +
        getAccelY() * getAccelY() +
        getAccelZ() * getAccelZ());

    // Remove gravity (approximately 9.81 m/s^2)
    float netAccel = abs(totalAccel - 9.81);

    return netAccel > MOTION_THRESHOLD;
}

float AHRS::getDisplacementX()
{
    return displacementX;
}

float AHRS::getDisplacementY()
{
    return displacementY;
}

float AHRS::getDisplacementZ()
{
    return displacementZ;
}

void AHRS::resetDisplacement()
{
    velocityX = velocityY = velocityZ = 0;
    displacementX = displacementY = displacementZ = 0;
}

float AHRS::getVelocityX()
{
    return velocityX;
}

float AHRS::getVelocityY()
{
    return velocityY;
}

float AHRS::getVelocityZ()
{
    return velocityZ;
}

float AHRS::getSpeed()
{
    return sqrt(velocityX * velocityX + velocityY * velocityY + velocityZ * velocityZ);
}

float AHRS::getAccelMagnitude()
{
    float ax = getAccelX();
    float ay = getAccelY();
    float az = getAccelZ();
    return sqrt(ax * ax + ay * ay + az * az);
}

float AHRS::getRoll()
{
    return initialized ? mpu->getRoll() : 0;
}

float AHRS::getPitch()
{
    return initialized ? mpu->getPitch() : 0;
}

float AHRS::getYaw()
{
    return initialized ? mpu->getYaw() : 0;
}

float AHRS::getAccelX()
{
    return initialized ? mpu->getAccX() : 0;
}

float AHRS::getAccelY()
{
    return initialized ? mpu->getAccY() : 0;
}

float AHRS::getAccelZ()
{
    return initialized ? mpu->getAccZ() : 0;
}

float AHRS::getGyroX()
{
    return initialized ? mpu->getGyroX() : 0;
}

float AHRS::getGyroY()
{
    return initialized ? mpu->getGyroY() : 0;
}

float AHRS::getGyroZ()
{
    return initialized ? mpu->getGyroZ() : 0;
}

float AHRS::getMagX()
{
    return initialized ? mpu->getMagX() : 0;
}

float AHRS::getMagY()
{
    return initialized ? mpu->getMagY() : 0;
}

float AHRS::getMagZ()
{
    return initialized ? mpu->getMagZ() : 0;
}

float AHRS::getQuatW()
{
    return initialized ? mpu->getQuaternionW() : 1;
}

float AHRS::getQuatX()
{
    return initialized ? mpu->getQuaternionX() : 0;
}

float AHRS::getQuatY()
{
    return initialized ? mpu->getQuaternionY() : 0;
}

float AHRS::getQuatZ()
{
    return initialized ? mpu->getQuaternionZ() : 0;
}

float AHRS::getTemperature()
{
    return initialized ? mpu->getTemperature() : 0;
}

void AHRS::calibrateAccelGyro()
{
    if (!initialized)
        return;

    Serial.println("Calibrating Accel & Gyro... Keep the sensor still");
    mpu->calibrateAccelGyro();
    Serial.println("Accel & Gyro calibration complete");
}

void AHRS::calibrateMag()
{
    if (!initialized)
        return;

    Serial.println("Calibrating Magnetometer... Wave the sensor in a figure 8");
    mpu->calibrateMag();
    Serial.println("Magnetometer calibration complete");
    calibrated = true;
}

bool AHRS::isCalibrated()
{
    return calibrated;
}

AHRS::MovementSnapshot AHRS::getMeasurement()
{
    MovementSnapshot snapshot;

    unsigned long currentTime = millis();
    snapshot.deltaTime = (currentTime - lastSnapshotTime) / 1000.0; // Convert to seconds

    // Calculate distance traveled
    float dx = displacementX - lastSnapshotX;
    float dy = displacementY - lastSnapshotY;
    float dz = displacementZ - lastSnapshotZ;
    snapshot.deltaDistance = sqrt(dx * dx + dy * dy + dz * dz) * 100; // Convert m to cm

    // Calculate averages
    if (sampleCount > 0)
    {
        snapshot.avgSpeed = (speedSum / sampleCount) * 100; // Convert m/s to cm/s
        snapshot.avgAcceleration = accelSum / sampleCount;
    }
    else
    {
        snapshot.avgSpeed = 0;
        snapshot.avgAcceleration = 0;
    }

    return snapshot;
}

void AHRS::resetMeasurement()
{
    lastSnapshotX = displacementX;
    lastSnapshotY = displacementY;
    lastSnapshotZ = displacementZ;
    lastSnapshotTime = millis();
    speedSum = 0;
    accelSum = 0;
    sampleCount = 0;
}
