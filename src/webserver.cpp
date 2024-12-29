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
    
    // Initialize preferences at construction
    preferences.begin("sensors", true);
    Serial.println("Preferences initialized in constructor");

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
    Serial.println("WebServer: Loading stored sensor names...");
    loadSensorNames();

    if(!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
        return;
    }

    on("/api/sensor/names", HTTP_GET, [this](AsyncWebServerRequest *request) {
        DynamicJsonDocument doc(1024);
        JsonArray array = doc.to<JsonArray>();
        
        if (xSemaphoreTake(gState.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            for (const auto& sensor : gState.sensorNames) {
                JsonObject obj = array.createNestedObject();
                obj["address"] = sensorAddressToString(sensor.address);
                obj["name"] = sensor.friendlyName;
            }
            xSemaphoreGive(gState.mutex);
        }
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

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

    // Add endpoint for setting sensor names
    on("/setSensorName", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handleSetSensorName(request);
    });

    on("/api/sensor/name", HTTP_POST, 
        std::bind(&TempWebServer::handleSetSensorName, this, std::placeholders::_1),
        NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            if (!request->_tempObject) {
                request->_tempObject = new String();
            }
            String* body = (String*)request->_tempObject;
            body->concat((char*)data, len);
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
            TickType_t lastWakeTime = xTaskGetTickCount();
            
            for(;;) {
                // Use longer mutex timeout and handle failure gracefully
                if (xSemaphoreTake(gState.mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                    std::vector<std::pair<String, float>> sensorData;
                    sensorData.reserve(gState.sensorAddresses.size()); // Preallocate
                    
                    // Quick copy under mutex lock
                    for (size_t i = 0; i < gState.sensorAddresses.size(); i++) {
                        sensorData.emplace_back(
                            sensorAddressToString(gState.sensorAddresses[i]),
                            gState.temperatures[i]
                        );
                    }
                    xSemaphoreGive(gState.mutex);
                    
                    // Process data outside mutex lock
                    server->sendSensorData(sensorData);
                }
                
                // Use vTaskDelayUntil for consistent timing
                vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000));
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

    // Add event source handler
    events.onConnect([](AsyncEventSourceClient *client) {
        Serial.println("Client connected to events");
    });
    
    // Register event source
    this->addHandler(&events);

    // Add sensor data endpoint
    on("/api/sensors", HTTP_GET, [this](AsyncWebServerRequest *request) {
        getSensorData(request);
    });
}

void TempWebServer::sendTemperature(float temp) {
    if (temp != DEVICE_DISCONNECTED_C) {
        String tempStr = String(temp, 1);
        events.send(tempStr.c_str(), "temperature", millis());
        Serial.printf("[WebServer] Temperature event sent: %s°C\n", tempStr.c_str());
    }
}

void TempWebServer::sendSensorData(const std::vector<std::pair<String, float>>& sensors) {
    DynamicJsonDocument doc(1024);
    JsonArray array = doc.to<JsonArray>();
    
    if (xSemaphoreTake(gState.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.println("Preparing sensor data to send...");
        
        for (const auto& sensor : sensors) {
            JsonObject obj = array.createNestedObject();
            obj["address"] = sensor.first;
            obj["temperature"] = sensor.second;
            
            // Add friendly name if exists
            for (const auto& name : gState.sensorNames) {
                if (sensorAddressToString(name.address) == sensor.first) {
                    obj["name"] = name.friendlyName;
                    break;
                }
            }
        }
        xSemaphoreGive(gState.mutex);
        
        String jsonStr;
        serializeJson(doc, jsonStr);
        Serial.printf("Sending sensor data: %s\n", jsonStr.c_str());
        events.send(jsonStr.c_str(), "sensors", millis());
    } else {
        Serial.println("Failed to acquire mutex for sending sensor data");
    }
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
    StaticJsonDocument<128> doc;
    doc["relay1"] = digitalRead(SYSTEM_RELAY1_PIN) == HIGH;
    doc["relay2"] = digitalRead(SYSTEM_RELAY2_PIN) == HIGH;
    
    String jsonString;
    serializeJson(doc, jsonString);
    events.send(jsonString.c_str(), "relayStates", millis());
}

void TempWebServer::loadSensorNames() {
    Serial.println("\n=== Starting Sensor Name Load ===");
    
    if (!preferences.begin("sensors", true)) {
        Serial.println("ERROR: Failed to begin preferences");
        return;
    }

    if (xSemaphoreTake(gState.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        gState.sensorNames.clear();  // Clear existing names
        size_t count = preferences.getUInt("count", 0);
        Serial.printf("Found %d stored sensor names\n", count);
        
        for (size_t i = 0; i < count; i++) {
            char addrKey[16], nameKey[16];
            snprintf(addrKey, sizeof(addrKey), "addr_%d", i);
            snprintf(nameKey, sizeof(nameKey), "name_%d", i);
            
            std::array<uint8_t, 8> address;
            if (preferences.getBytes(addrKey, address.data(), 8)) {
                String name = preferences.getString(nameKey, "");
                if (!name.isEmpty()) {
                    SensorName sensorName;
                    sensorName.address = address;
                    strlcpy(sensorName.friendlyName, name.c_str(), sizeof(sensorName.friendlyName));
                    gState.sensorNames.push_back(sensorName);
                    Serial.printf("Loaded sensor %d: %s = %s\n", i, 
                        sensorAddressToString(address).c_str(), 
                        name.c_str());
                }
            }
        }
        xSemaphoreGive(gState.mutex);
        Serial.printf("Successfully loaded %d sensor names\n", gState.sensorNames.size());
    } else {
        Serial.println("ERROR: Failed to acquire mutex in loadSensorNames");
    }

    preferences.end();
    Serial.println("=== Sensor Name Load Complete ===\n");
}

bool TempWebServer::saveSensorName(const std::array<uint8_t, 8>& address, const char* name) {
    Serial.println("\n=== Saving Sensor Name ===");
    
    if (!preferences.begin("sensors", false)) {
        Serial.println("ERROR: Failed to begin preferences");
        return false;
    }

    bool success = false;
    if (xSemaphoreTake(gState.mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        // First check if this address already exists
        auto it = std::find_if(gState.sensorNames.begin(), gState.sensorNames.end(),
            [&address](const SensorName& sn) { return sn.address == address; });

        if (it != gState.sensorNames.end()) {
            // Update existing name
            strlcpy(it->friendlyName, name, sizeof(it->friendlyName));
        } else {
            // Add new sensor name
            SensorName sensorName;
            sensorName.address = address;
            strlcpy(sensorName.friendlyName, name, sizeof(sensorName.friendlyName));
            gState.sensorNames.push_back(sensorName);
        }

        // Save all names to preferences
        preferences.clear(); // Clear existing preferences
        preferences.putUInt("count", gState.sensorNames.size());
        
        for (size_t i = 0; i < gState.sensorNames.size(); i++) {
            char addrKey[16], nameKey[16];
            snprintf(addrKey, sizeof(addrKey), "addr_%d", i);
            snprintf(nameKey, sizeof(nameKey), "name_%d", i);
            
            preferences.putBytes(addrKey, gState.sensorNames[i].address.data(), 8);
            preferences.putString(nameKey, gState.sensorNames[i].friendlyName);
            
            Serial.printf("Saved sensor %d: %s = %s\n", i, 
                sensorAddressToString(gState.sensorNames[i].address).c_str(),
                gState.sensorNames[i].friendlyName);
        }
        
        success = true;
        xSemaphoreGive(gState.mutex);
    } else {
        Serial.println("ERROR: Failed to acquire mutex in saveSensorName");
    }

    preferences.end();
    Serial.println("=== Sensor Name Save Complete ===\n");
    return success;
}

void TempWebServer::getSensorData(AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(1024);
    JsonArray array = doc.to<JsonArray>();
    
    if (xSemaphoreTake(gState.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (size_t i = 0; i < gState.sensorAddresses.size(); i++) {
            JsonObject sensor = array.createNestedObject();
            sensor["address"] = sensorAddressToString(gState.sensorAddresses[i]);
            sensor["temperature"] = gState.temperatures[i];
            
            // Add friendly name if exists
            for (const auto& name : gState.sensorNames) {
                if (name.address == gState.sensorAddresses[i]) {
                    sensor["name"] = name.friendlyName;
                    break;
                }
            }
        }
        xSemaphoreGive(gState.mutex);
    }
    
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

void TempWebServer::handleSetSensorName(AsyncWebServerRequest *request) {
    if (!request->hasHeader("Content-Type") || 
        request->getHeader("Content-Type")->value() != "application/json") {
        request->send(400, "text/plain", "Content-Type must be application/json");
        return;
    }
    
    if (!request->_tempObject) {
        request->send(400, "text/plain", "No body provided");
        return;
    }

    String body = *(String*)request->_tempObject;
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
        request->send(400, "text/plain", "Invalid JSON");
        return;
    }

    if (!doc.containsKey("address") || !doc.containsKey("name")) {
        request->send(400, "text/plain", "Missing address or name");
        return;
    }

    const char* addressStr = doc["address"];
    const char* name = doc["name"];
    
    std::array<uint8_t, 8> address;
    for (int i = 0; i < 8; i++) {
        String byteStr = String(addressStr).substring(i*2, i*2+2);
        address[i] = strtol(byteStr.c_str(), NULL, 16);
    }
    
    if (saveSensorName(address, name)) {
        request->send(200, "text/plain", "OK");
    } else {
        request->send(500, "text/plain", "Failed to save sensor name");
    }
}

void TempWebServer::setupRelayEndpoints() {
    this->on("/relay1/set", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("state", true)) {
            String state = request->getParam("state", true)->value();
            bool relayState = (state == "on" || state == "1");
            digitalWrite(SYSTEM_RELAY1_PIN, relayState);
            request->send(200, "text/plain", "OK");
            sendRelayStates();
        } else {
            request->send(400, "text/plain", "Missing state parameter");
        }
    });
    this->on("/relay2/set", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("state", true)) {
            String state = request->getParam("state", true)->value();
            bool relayState = (state == "on" || state == "1");
            digitalWrite(SYSTEM_RELAY2_PIN, relayState);
            request->send(200, "text/plain", "OK");
            sendRelayStates();
        } else {
            request->send(400, "text/plain", "Missing state parameter");
        }
    });
}

TempWebServer::~TempWebServer() {
    // AsyncEventSource will clean itself up automatically
    // No manual cleanup needed since it's not a pointer
}