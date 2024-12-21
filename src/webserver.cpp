#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include "onewire_manager.h"
#include "webserver.h"
#include "mqtt_manager.h"
#include "config.h"
#include <SPIFFS.h>
#include <ArduinoJson.h>

// Constructor - add route handlers
TempWebServer::TempWebServer(uint16_t port) 
    : AsyncWebServer(port), events("/events"), mqttManager(nullptr) {
    
    networkMutex = xSemaphoreCreateMutex();
    if (networkMutex == NULL) {
        Serial.println("Failed to create network mutex");
        return;
    }

    // Setup routes
    on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (SPIFFS.exists("/dashboard.html")) {
            request->send(SPIFFS, "/dashboard.html", "text/html");
        } else {
            Serial.println("dashboard.html file not found in SPIFFS");
            request->send(404, "text/plain", "HTML file not found");
        }
    });
    // Add event handler
    addHandler(&events);
}

void TempWebServer::begin() {
    // Mount SPIFFS first
    if(!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    Serial.println("SPIFFS mounted successfully");

    // List files in SPIFFS
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while(file) {
        Serial.printf("Found file: %s, size: %d\n", file.name(), file.size());
        file = root.openNextFile();
    }

    // Setup event source
    events.onConnect([](AsyncEventSourceClient *client) {
        client->send("hello!", NULL, millis(), 1000);
    });
    
    // Add handlers
    addHandler(&events);

    // Add CORS headers
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    
    // Serve static files

    on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (SPIFFS.exists("/dashboard.html")) {
            request->send(SPIFFS, "/dashboard.html", "text/html");
        } else {
            Serial.println("dashboard.html file not found in SPIFFS");
            request->send(404, "text/plain", "HTML file not found");
        }
    });

    on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (SPIFFS.exists("/dashboard.css")) {
            request->send(SPIFFS, "/dashboard.css", "text/css");
        } else {
            Serial.println("CSS file not found in SPIFFS");
            request->send(404, "text/plain", "CSS file not found");
        }
    });

    on("/logo.png", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/logo.png", "image/png");
    });
        // Add relay control endpoints
    on("/relay1/set", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("state", true)) {
            String state = request->getParam("state", true)->value();
            bool newState = (state == "on" || state == "1" || state == "true");
            
            // Update global state
            updateRelayState(SYSTEM_RELAY1_PIN, newState);
            
            // Publish via MQTT
            if (mqttManager) {
                String topic = String(SYSTEM_NAME) + "/" + 
                              String(MQTT_CLIENT_ID) + "/" + 
                              SYSTEM_RELAY1_TOPIC;
                
                if (mqttManager->publishTopic(topic.c_str(), newState ? "on" : "off")) {
                    request->send(200, "text/plain", "OK");
                } else {
                    request->send(500, "text/plain", "Failed to publish");
                }
            } else {
                request->send(500, "text/plain", "MQTT not initialized");
            }
        }
    });

    on("/relay2/set", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("state", true)) {
            String state = request->getParam("state", true)->value();
            bool newState = (state == "on" || state == "1" || state == "true");
            
            // Update global state
            updateRelayState(SYSTEM_RELAY2_PIN, newState);
            
            // Publish via MQTT
            if (mqttManager) {
                String topic = String(SYSTEM_NAME) + "/" + 
                              String(MQTT_CLIENT_ID) + "/" + 
                              SYSTEM_RELAY2_TOPIC;
                
                if (mqttManager->publishTopic(topic.c_str(), newState ? "on" : "off")) {
                    request->send(200, "text/plain", "OK");
                } else {
                    request->send(500, "text/plain", "Failed to publish");
                }
            } else {
                request->send(500, "text/plain", "MQTT not initialized");
            }
        }
    });

    on("/relay1", HTTP_POST, [this](AsyncWebServerRequest *request) {
        // Get state parameter from request
        if (!request->hasParam("state", true)) {
            request->send(400, "text/plain", "Missing state parameter");
            return;
        }
        
        String stateParam = request->getParam("state", true)->value();
        bool newState = (stateParam == "on" || stateParam == "1" || stateParam == "true");
        
        this->updateRelayState(SYSTEM_RELAY1_PIN, newState);
        request->send(200, "text/plain", newState ? "ON" : "OFF");
    });

        on("/relay1/state", HTTP_GET, [this](AsyncWebServerRequest *request) {
        bool state = digitalRead(SYSTEM_RELAY1_PIN) == HIGH;
        request->send(200, "text/plain", state ? "ON" : "OFF");
    });

    on("/relay2/state", HTTP_GET, [this](AsyncWebServerRequest *request) {
        bool state = digitalRead(SYSTEM_RELAY2_PIN) == HIGH;
        request->send(200, "text/plain", state ? "ON" : "OFF");
    });

    on("/selectSensor", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("address", true)) {
            String address = request->getParam("address", true)->value();
            
            if (xSemaphoreTake(gState.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Find sensor index by address
                for (size_t i = 0; i < gState.sensorAddresses.size(); i++) {
                    if (sensorAddressToString(gState.sensorAddresses[i]) == address) {
                        gState.selectedSensorIndex = i;
                        gState.hasSelectedSensor = true;
                        break;
                    }
                }
                xSemaphoreGive(gState.mutex);
                request->send(200, "text/plain", "OK");
            } else {
                request->send(500, "text/plain", "Mutex error");
            }
        } else {
            request->send(400, "text/plain", "Missing address");
        }
    });

    // Add relay state monitoring task
    xTaskCreatePinnedToCore(
        [](void* parameter) {
            TempWebServer* server = (TempWebServer*)parameter;
            for(;;) {
                server->sendRelayStates();
                vTaskDelay(pdMS_TO_TICKS(1000));  // Update every second
            }
        },
        "RelayMonitor",
        4096,
        this,
        1,
        NULL,
        1
    );

    // Create sensor data monitoring task
    xTaskCreatePinnedToCore(
        [](void* parameter) {
            TempWebServer* server = (TempWebServer*)parameter;
            for(;;) {
                if (xSemaphoreTake(gState.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                    // Create vector of sensor data
                    std::vector<std::pair<String, float>> sensorData;
                    
                    // Populate sensor data from global state
                    for (size_t i = 0; i < gState.sensorAddresses.size(); i++) {
                        sensorData.push_back(std::make_pair(
                            String(sensorAddressToString(gState.sensorAddresses[i]).c_str()),
                            gState.temperatures[i]
                        ));
                    }
                    
                    // Call sendSensorData with the populated vector
                    server->sendSensorData(sensorData);
                    
                    xSemaphoreGive(gState.mutex);
                } else {
                    Serial.println("Failed to get mutex for sensor data");
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        },
        "WebMonitor",
        4096,
        this,
        1,
        NULL,
        1
    );

    // Start web server
    AsyncWebServer::begin();
    Serial.println("Web server started");
}

void TempWebServer::sendTemperature(float temp) {
    if (temp != DEVICE_DISCONNECTED_C) {
        String tempStr = String(temp, 1);
        events.send(tempStr.c_str(), "temperature", millis());
        Serial.printf("[WebServer] Temperature event sent: %s°C\n", tempStr.c_str());
    }
}

void TempWebServer::sendSensorData(const std::vector<std::pair<String, float>>& sensors) {
    if (sensors.empty()) return;

    DynamicJsonDocument doc(1024);
    JsonArray array = doc.to<JsonArray>();
    
    for(const auto& sensor : sensors) {
        JsonObject obj = array.createNestedObject();
        obj["address"] = sensor.first;
        obj["temp"] = sensor.second;
    }

    String jsonString;
    serializeJson(doc, jsonString);
    
    events.send(jsonString.c_str(), "sensors", millis());
    Serial.printf("Sending sensor data: %s\n", jsonString.c_str());
}

void TempWebServer::setMQTTManager(MQTTManager* manager) {
    mqttManager = manager;
    // Optional: Log successful connection
    Serial.println("[WebServer] MQTT manager initialized");
}

void sendSensorData(const std::vector<std::pair<String, float>>& sensorData) {
    // Implementation for sending sensor data
}

// Global instance (matches extern declaration)
TempWebServer webServer(80);

bool MQTTManager::publishTopic(const char* topic, const char* payload) {
    if (!mqttClient.connected()) {
        return false;
    }
    return mqttClient.publish(topic, payload, true);  // Retained message
}

void TempWebServer::updateRelayState(uint8_t pin, bool state) {
    // Remove the :: operator
    pinMode(pin, OUTPUT);
    digitalWrite(pin, state);
    
    // Log state change
    Serial.printf("Relay pin %d set to %s\n", pin, state ? "ON" : "OFF");
    
    // Notify connected clients
    sendRelayStates();
}

void TempWebServer::sendRelayStates() {
    StaticJsonDocument<200> doc;
    doc["relay1"] = digitalRead(SYSTEM_RELAY1_PIN) == HIGH;
    doc["relay2"] = digitalRead(SYSTEM_RELAY2_PIN) == HIGH;
    
    String jsonString;
    serializeJson(doc, jsonString);
    events.send(jsonString.c_str(), "relayStates", millis());
}
