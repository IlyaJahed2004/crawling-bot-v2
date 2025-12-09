#ifndef NETWORK_H
#define NETWORK_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ESPmDNS.h>

// Forward declaration
class Display;

class Network {
public:
    Network(Display* display);
    void begin();
    void startOTATask();
    uint8_t getRobotNumber();
    const char* getSSID();
    const char* getHostname();
    
    static void otaTaskFunction(void* parameter);

private:
    Display* display;
    uint8_t robotNumber;
    char ssid[32];
    char otaHostname[32];
    TaskHandle_t otaTaskHandle;
    
    static const uint8_t EEPROM_SIZE = 1;
    static const uint8_t ROBOT_NUM_ADDR = 0;
    static const char* BASE_SSID;
    static const char* BASE_OTA_HOSTNAME;
    static const char* AP_PASSWORD;
    
    void setupAP();
    void setupOTA();
    uint8_t readRobotNumber();
    void saveRobotNumber(uint8_t number);
};

#endif // NETWORK_H
