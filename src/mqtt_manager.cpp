#include "mqtt_manager.h"
#include "config.h"
#include <Arduino.h>
#include <cstdio>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <vector>
#include <array>
#include <string>

// Initialize static member
MQTTManager* MQTTManager::instance = nullptr;

MQTTManager::MQTTManager() : 
    espClient(),
    mqttClient(espClient),
    lastPublish(0),
    lastSensorPublish(0),
    reconnectAttempts(0) {
    instance = this;  // Set instance pointer
}

void MQTTManager::begin() {
    setupSecureClient();
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
}

void MQTTManager::setupSecureClient() {
    // Complete reset
    mqttClient.disconnect();
    espClient.stop();
    delay(500); // Longer delay for cleanup
    
    espClient = WiFiClientSecure(); // Create new instance
    espClient.setCACert(let_encrypt_root_ca);
    espClient.setHandshakeTimeout(5000);  // Shorter timeout
    espClient.setTimeout(3000);
    
    mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
    mqttClient.setKeepAlive(5);  // More frequent keepalive
    mqttClient.setSocketTimeout(3);
}

void MQTTManager::loop() {
    if (oneWireMutex != NULL) {
        if(xSemaphoreTake(oneWireMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Access global data structures directly
            if (!temperatures.empty() && !sensorAddresses.empty()) {
                publishSensorData(sensorAddresses, temperatures);
            }
            xSemaphoreGive(oneWireMutex);
        }
    }
    mqttClient.loop();
}

bool MQTTManager::isConnected() {
    return mqttClient.connected();
}

void MQTTManager::publishState() {
    unsigned long now = millis();
    if (now - lastPublish >= STATE_PUBLISH_INTERVAL) {  // Use constant from config.h
        char state[128];
        snprintf(state, sizeof(state),
                "{\"heap\":%d,\"uptime\":%lu,\"connected\":%d}",
                ESP.getFreeHeap(),
                now / 1000,
                mqttClient.connected());

        String topic = String(SYSTEM_NAME) + "/" + String(MQTT_CLIENT_ID) + "/" + String(MQTT_STATE_TOPIC);
        mqttClient.publish(topic.c_str(), state, true);
        lastPublish = now;
    }
}

void MQTTManager::publishSensorData(const std::vector<std::array<uint8_t, 8>>& sensors,
                                  const std::vector<float>& values) {
    unsigned long now = millis();
    
    if (now - lastSensorPublish >= SENSOR_PUBLISH_INTERVAL && mqttClient.connected()) {
        for (size_t i = 0; i < sensors.size(); i++) {
            // Convert std::string to const char* before creating Arduino String
            String sensorTopic = String(SYSTEM_NAME) + "/" + 
                               String(MQTT_CLIENT_ID) + "/" + 
                               String(SENSOR_TOPIC_PATH) + "/" + 
                               String(sensorAddressToString(sensors[i]).c_str());
            
            // Create JSON payload for single sensor
            String payload = "{\"temp\":";
            payload += String(values[i], 2);
            payload += "}";
            
            bool success = mqttClient.publish(sensorTopic.c_str(), payload.c_str(), true);
            
            if (success) {
                Serial.printf("Published sensor %s: %.2f°C\n", 
                    sensorAddressToString(sensors[i]).c_str(), 
                    values[i]);
            }
        }
        lastSensorPublish = now;
    }
}

void MQTTManager::mqttReconnectTask(void* parameter) {
    const uint32_t MIN_RETRY_DELAY = 2000;
    uint32_t retryDelay = MIN_RETRY_DELAY;
    
    for(;;) {
        if (ETH.linkUp() && !instance->mqttClient.connected()) {
            Serial.printf("Reconnecting, heap: %d, retry delay: %d\n", 
                ESP.getFreeHeap(), retryDelay);
            
            instance->setupSecureClient();
            
            if (instance->mqttClient.connect(MQTT_CLIENT_ID, 
                                           MQTT_USERNAME, 
                                           MQTT_PASSWORD,
                                           "status",
                                           1,
                                           true,
                                           "offline",
                                           true)) {
                retryDelay = MIN_RETRY_DELAY;
                instance->reconnectAttempts = 0;
                instance->mqttClient.publish("status", "online", true);
            } else {
                instance->reconnectAttempts++;
                retryDelay *= 2;  // Exponential backoff
                if (instance->reconnectAttempts >= 5) {
                    ESP.restart();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(retryDelay));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void MQTTManager::printConnectionDetails() {
    if (!espClient.connected()) {
        Serial.println("Client not connected!");
        return;
    }
    
    IPAddress localIP = espClient.localIP();
    Serial.print("Local IP: ");
    Serial.println(localIP.toString());
}

bool MQTTManager::reconnect() {
    if (!waitForNetwork()) {
        logError("Network unavailable");
        return false;
    }
    
    // Reset SSL state
    espClient.stop();
    delay(100);
    
    // Check heap before SSL init
    int heapBefore = ESP.getFreeHeap();
    logError("Heap before SSL: %d", heapBefore);
    
    setupSecureClient();
    
    if (mqttClient.connect(MQTT_CLIENT_ID, 
                          MQTT_USERNAME, 
                          MQTT_PASSWORD,
                          "status",    // LWT topic
                          1,           // QoS
                          true,        // Retain
                          "offline")) { // LWT message
        logError("MQTT connected, heap after: %d", ESP.getFreeHeap());
        reconnectAttempts = 0;
        return true;
    }
    
    return false;
}

bool MQTTManager::waitForNetwork() {
    int timeout = 10;  // 10 second timeout
    while (!ETH.linkUp() && timeout > 0) {
        delay(1000);
        timeout--;
    }
    return ETH.linkUp();
}

void MQTTManager::reconnectWithBackoff() {
    static const uint32_t MIN_DELAY = 2000;  // 2 seconds
    static const uint32_t MAX_DELAY = 30000; // 30 seconds
    static uint32_t retryDelay = MIN_DELAY;
    
    while (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
        Serial.printf("Heap: %d, Attempting MQTT connection...\n", ESP.getFreeHeap());
        
        // Reset SSL connection
        espClient.stop();
        delay(100);
        setupSecureClient();
        
        if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
            Serial.println("MQTT Connected");
            reconnectAttempts = 0;
            retryDelay = MIN_DELAY;
            return;
        }
        
        reconnectAttempts++;
        int state = mqttClient.state();
        Serial.printf("Failed, rc=%d (%s), attempt %d/%d\n", 
                     state, 
                     getMQTTErrorString(state),
                     reconnectAttempts, 
                     MAX_RECONNECT_ATTEMPTS);
        
        // Exponential backoff
        retryDelay = min(retryDelay * 2, MAX_DELAY);
        Serial.printf("Next retry in %d ms\n", retryDelay);
        vTaskDelay(pdMS_TO_TICKS(retryDelay));
    }
    
    Serial.println("Max reconnection attempts reached, restarting...");
    ESP.restart();
}

const char* MQTTManager::getMQTTErrorString(int error) {
    switch (error) {
        case -4: return "MQTT_CONNECTION_TIMEOUT";
        case -3: return "MQTT_CONNECTION_LOST";
        case -2: return "MQTT_CONNECT_FAILED";
        case -1: return "MQTT_DISCONNECTED";
        case 0: return "MQTT_CONNECTED";
        case 1: return "MQTT_CONNECT_BAD_PROTOCOL";
        case 2: return "MQTT_CONNECT_BAD_CLIENT_ID";
        case 3: return "MQTT_CONNECT_UNAVAILABLE";
        case 4: return "MQTT_CONNECT_BAD_CREDENTIALS";
        case 5: return "MQTT_CONNECT_UNAUTHORIZED";
        default: return "MQTT_UNKNOWN";
    }
}

std::string MQTTManager::sensorAddressToString(const std::array<uint8_t, 8>& address) {
    char buffer[17];  // 8 bytes * 2 chars per byte + null terminator
    snprintf(buffer, sizeof(buffer),
             "%02X%02X%02X%02X%02X%02X%02X%02X",
             address[0], address[1], address[2], address[3],
             address[4], address[5], address[6], address[7]);
    return std::string(buffer);
}