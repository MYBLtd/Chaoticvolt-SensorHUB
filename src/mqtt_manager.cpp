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
    mqttClient(espClient),
    oneWireMutex(xSemaphoreCreateMutex()),
    lastPublish(0),
    reconnectAttempts(0)
{
    instance = this;
    
    if (oneWireMutex == NULL) {
        ESP.restart();  // Critical error - restart device
    }
    
    // Clear vectors
    temperatures.clear();
    sensorAddresses.clear();
    
    setupSecureClient();
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback([this](char* topic, byte* payload, unsigned int length) {
        this->callback(topic, payload, length);
    });
}

MQTTManager::~MQTTManager() {
    if (oneWireMutex != NULL) {
        vSemaphoreDelete(oneWireMutex);
        oneWireMutex = NULL;
    }
}

void MQTTManager::begin() {
    Serial.println("MQTT: Starting manager...");
    delay(1000);
    
    xTaskCreatePinnedToCore(
        [](void* parameter) {
            MQTTManager* manager = (MQTTManager*)parameter;
            TickType_t xLastWakeTime = xTaskGetTickCount();
            
            Serial.println("MQTT: Task started");
            
            for(;;) {              
                if (xSemaphoreTake(gState.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {                   
                    if (gState.dataUpdated) {
                        // Serial.printf("MQTT: Publishing %d sensor values\n", 
                        //     gState.sensorAddresses.size());
                        
                        manager->publishSensorData(
                            gState.sensorAddresses,
                            gState.temperatures
                        );
                        gState.dataUpdated = false;
                    } else {
                        // Serial.println("MQTT: No new data");
                    }
                    xSemaphoreGive(gState.mutex);
                } else {
                    // Serial.println("MQTT: Failed to get mutex");
                }
                
                vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
            }
        },
        "MQTT_Monitor",
        8192,
        this,
        2,
        NULL,
        1
    );
    
    // Serial.println("MQTT: Manager started");
}

void MQTTManager::setupSecureClient() {
    Serial.println("Setting up secure client...");
    
    // Reset previous connections
    espClient.stop();
    delay(100);
    
    espClient = WiFiClientSecure();
    
    // Debug certificate info
    Serial.printf("CA cert length: %d bytes\n", strlen(let_encrypt_root_ca));
    
    // Configure SSL with longer timeouts
    espClient.setCACert(let_encrypt_root_ca);
    espClient.setHandshakeTimeout(30000);  // 30 seconds
    espClient.setTimeout(15000);           // 15 seconds
    
    // Configure MQTT client
    mqttClient.setClient(espClient);
    mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
    mqttClient.setKeepAlive(30);          // 30 seconds
    mqttClient.setSocketTimeout(15);       // 15 seconds
    
    Serial.println("Secure client setup complete");
}

void MQTTManager::loop() {
    static unsigned long lastCheck = 0;
    unsigned long now = millis();
    
    // Add error checking
    if (!ETH.linkUp()) {
        Serial.println("Network link down");
        delay(1000);
        return;
    }
    
    // Check every second with error handling
    if (now - lastCheck >= 1000) {
        lastCheck = now;
        
        if (!mqttClient.connected()) {
            Serial.println("MQTT disconnected, attempting reconnect...");
            delay(100);  // Add delay before reconnect
            return;
        }
        
        try {
            mqttClient.loop();
        } catch (...) {
            Serial.println("Error in MQTT loop");
            delay(1000);
        }
    }
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
    
    if (!mqttClient.connected()) {
        Serial.println("MQTT: Not connected, attempting reconnect...");
        if (!reconnect()) {
            Serial.println("MQTT: Reconnection failed, skipping publish");
            return;
        }
    }
    
    // Serial.printf("MQTT: Publishing %d sensor values\n", sensors.size());
    
    for (size_t i = 0; i < sensors.size(); i++) {
        String sensorAddr = String(sensorAddressToString(sensors[i]).c_str());
        String topic = String(SYSTEM_NAME) + "/" + 
                      String(MQTT_CLIENT_ID) + "/" + 
                      String(MQTT_TOPIC_BASE) + "/" + 
                      sensorAddr;
        
        String payload = String(values[i], 2);
        
        bool success = mqttClient.publish(topic.c_str(), payload.c_str(), true);
        Serial.printf("MQTT: %s = %s (%s)\n", topic.c_str(), payload.c_str(), 
                     success ? "OK" : "FAILED");
    }
    
    lastSensorPublish = now;
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
    if (!ETH.linkUp()) {
        Serial.println("MQTT: Network link down");
        return false;
    }
    
    // Setup secure client before attempting connection
    setupSecureClient();
    
    Serial.printf("MQTT: Attempting connection to %s:%d...\n", MQTT_BROKER, MQTT_PORT);
    
    // Configure LWT (Last Will and Testament)
    if (mqttClient.connect(MQTT_CLIENT_ID, 
                          MQTT_USERNAME, 
                          MQTT_PASSWORD,
                          "status",    // LWT topic
                          1,           // QoS
                          true,        // Retain
                          "offline"))  // LWT message
    {
        Serial.println("MQTT: Connected");
        
        // Publish online status
        String statusTopic = String(SYSTEM_NAME) + "/" + String(MQTT_CLIENT_ID) + "/status";
        mqttClient.publish(statusTopic.c_str(), "online", true);
        
        return true;
    }
    
    Serial.printf("MQTT: Connection failed, rc=%d\n", mqttClient.state());
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

bool MQTTManager::connect() {
    if (!espClient.connected()) {
        Serial.println("SSL not connected, attempting connection...");
        if (!espClient.connect(MQTT_BROKER, MQTT_PORT)) {
            Serial.println("SSL connection failed");
            return false;
        }
    }
    
    if (!mqttClient.connected()) {
        return mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
    }
    
    return true;
}

void MQTTManager::callback(char* topic, byte* payload, unsigned int length) {
    // Convert payload to null-terminated string
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    
    // Log received message
    Serial.printf("Message arrived [%s]: %s\n", topic, message);
    
    // Handle the message here
    // Add your message handling logic
}