#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ETH.h>
#include <vector>
#include <string>
#include "config.h"
#include "certificates.h"

// Base topic paths
static const char* SENSOR_TOPIC = "sensors/temperature";
static const char* SENSOR_TOPIC_PATH = "sensors/temperature";

class MQTTManager {
public:
    // Singleton access method
    static MQTTManager& getInstance() {
        static MQTTManager instance;
        return instance;
    }
    
    // Static task function
    static void mqttReconnectTask(void* parameter);
    
    // Public methods
    void begin();
    void loop();
    void publishState();
    bool isConnected();
    void reconnectWithBackoff();
    bool reconnect();
    
    // Generic method to publish sensor readings
    template<typename T>
    void publishSensorData(const char* sensorId, T value) {
        if (!isConnected()) return;
        
        char topic[128];
        snprintf(topic, sizeof(topic), "%s/%s", SENSOR_TOPIC_PATH, sensorId);
        
        char payload[32];
        snprintf(payload, sizeof(payload), "%.2f", static_cast<float>(value));
        
        publishTopic(topic, payload);
    }
    
    void publishSensorData(const std::vector<std::array<uint8_t, 8>>& sensors,
                          const std::vector<float>& values);
    
    bool publishTopic(const char* topic, const char* payload);
    
private:
    static MQTTManager* instance;  // Add this declaration
    SemaphoreHandle_t oneWireMutex;
    std::vector<float> temperatures;
    std::vector<std::array<uint8_t, 8>> sensorAddresses;

    // Private constructor
    MQTTManager();
    
    // Delete copy constructor and assignment
    MQTTManager(const MQTTManager&) = delete;
    MQTTManager& operator=(const MQTTManager&) = delete;
    
    // Member variables
    WiFiClientSecure espClient;
    PubSubClient mqttClient;
    unsigned long lastPublish;
    uint8_t reconnectAttempts;
    char lastErrorMsg[128];
    unsigned long lastSensorPublish;
    
    // Private helper methods
    void setupSecureClient();
    void printConnectionDetails();
    const char* getMQTTErrorString(int error);
    void logError(const char* format, ...);
    bool waitForNetwork();
    
    // Helper function to convert sensor address to string
    std::string sensorAddressToString(const std::array<uint8_t, 8>& address);
};

#endif