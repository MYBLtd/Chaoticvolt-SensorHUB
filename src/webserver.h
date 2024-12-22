#pragma once

#include <Arduino.h>
#include "global_state.h" 
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <vector>
#include <utility>
#include <AsyncEventSource.h>
#include <Preferences.h>
#include "onewire_manager.h"
#include <SPIFFS.h>
#include <ArduinoJson.h>  // Include at the top of webserver.cpp if not already present

// Forward declaration
class MQTTManager;

class TempWebServer : public AsyncWebServer {
private:
    AsyncEventSource events;
    MQTTManager* mqttManager;
    SemaphoreHandle_t networkMutex;
    TaskHandle_t taskHandle;
    Preferences preferences;

    void updateRelayState(uint8_t pin, bool state);
    void sendRelayStates();
    void loadSensorNames();
    void setupRelayEndpoints();
   
public:
    TempWebServer(uint16_t port = 80);
    ~TempWebServer();
    void begin();
    void sendTemperature(float temp);
    void sendSensorData(const std::vector<std::pair<String, float>>& sensors);
    void setMQTTManager(MQTTManager* manager);
    void getSensorData(AsyncWebServerRequest *request);
    void handleSetSensorName(AsyncWebServerRequest *request);
    void handleStyleRequest();
    bool saveSensorName(const std::array<uint8_t, 8>& address, const char* name);
};

extern TempWebServer webServer;