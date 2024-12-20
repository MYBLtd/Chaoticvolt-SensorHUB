#pragma once
#include <Arduino.h>
#include <vector>
#include <array>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct GlobalState {
    // Mutex for thread safety
    SemaphoreHandle_t mutex;
    
    // Constructor to properly initialize members
    GlobalState() : 
        mutex(NULL),
        dataUpdated(false),
        relay1State(false),
        relay2State(false),
        mqttConnected(false),
        wifiConnected(false),
        lastStatePublish(0),
        lastSensorPublish(0)
    {
        mutex = xSemaphoreCreateMutex();
        if (mutex == NULL) {
            Serial.println("Failed to create mutex");
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

    // Sensor data
    bool dataUpdated;
    std::vector<std::array<uint8_t, 8>> sensorAddresses;
    std::vector<float> temperatures;
    
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
