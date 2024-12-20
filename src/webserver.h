#pragma once
#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <vector>
#include <utility>
#include <AsyncEventSource.h>
#include "webserver.h"
#include "onewire_manager.h"  // Add this line
#include <SPIFFS.h>  // Add this include

// Forward declaration
class MQTTManager;

class TempWebServer : public AsyncWebServer {
private:
    AsyncEventSource events;
    MQTTManager* mqttManager;
    SemaphoreHandle_t networkMutex;
    TaskHandle_t taskHandle;  // Don't use xTaskHandle name

    // Add this function declaration
    void updateRelayState(uint8_t pin, bool state);
    void sendRelayStates();  // Add this declaration

public:
    TempWebServer(uint16_t port = 80);
    void begin();
    void sendTemperature(float temp);
    void sendSensorData(const std::vector<std::pair<String, float>>& sensors);
    void setMQTTManager(MQTTManager* manager);

    // Add this function
    void handleStyleRequest() {
        on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
            if (SPIFFS.exists("/dashboard.css")) {
                request->send(SPIFFS, "/dashboard.css", "text/css");
            } else {
                Serial.println("CSS file not found in SPIFFS");
                request->send(404, "text/plain", "CSS file not found");
            }
        });
    }
};

extern TempWebServer webServer;