#pragma once
#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <vector>
#include <utility>
#include <AsyncEventSource.h>
#include "webserver.h"
#include "onewire_manager.h"  // Add this line

// Forward declaration
class MQTTManager;

class TempWebServer : public AsyncWebServer {
private:
    AsyncEventSource events;
    static const char DASHBOARD_HTML[] PROGMEM;
    static const char DASHBOARD_CSS[] PROGMEM;
    MQTTManager* mqttManager;

public:
    TempWebServer(uint16_t port = 80);
    void begin();
    void sendTemperature(float temp);
    void sendSensorData(const std::vector<std::pair<String, float>>& sensors);
    void setMQTTManager(MQTTManager* manager);
};

extern TempWebServer webServer;