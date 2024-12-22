#pragma once
#include <Arduino.h>
#include <vector>
#include <array>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct SensorName {
    std::array<uint8_t, 8> address;
    char friendlyName[32];
};

struct GlobalState {
    SemaphoreHandle_t mutex;
    bool dataUpdated;
    std::vector<std::array<uint8_t, 8>> sensorAddresses;
    std::vector<float> temperatures;
    std::vector<SensorName> sensorNames;
    bool hasSelectedSensor;
    size_t selectedSensorIndex;
    
    // Constructor to properly initialize members
    GlobalState() : 
        mutex(NULL),
        dataUpdated(false),
        relay1State(false),
        relay2State(false),
        mqttConnected(false),
        wifiConnected(false),
        lastStatePublish(0),
        lastSensorPublish(0),
        hasSelectedSensor(false),
        selectedSensorIndex(0)
    {
        // Early mutex initialization with error checking
        mutex = xSemaphoreCreateMutex();
        if (mutex == NULL) {
            Serial.println("FATAL: Failed to create mutex");
            ESP.restart();
        }
    }
    
    // Destructor to clean up
    ~GlobalState() {
        if (mutex != NULL) {
            vSemaphoreDelete(mutex);
            mutex = NULL;
        }
    }
    
    // Relay states
    bool relay1State;
    bool relay2State;
    
    // Connection states
    bool mqttConnected;
    bool wifiConnected;
    
    // Timing
    unsigned long lastStatePublish;
    unsigned long lastSensorPublish;
};

// Global instance declaration
extern GlobalState gState;

// Function declarations
void initializeGlobalState();
void updateRelayState(uint8_t pin, bool state);
bool getRelayState(uint8_t pin);