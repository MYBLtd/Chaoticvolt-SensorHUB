#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <WiFi.h>
#include <ETH.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h>
#include "esp_task_wdt.h"
#include "config.h"
#include "mqtt_manager.h"
#include "onewire_manager.h"
#include "webserver.h"  // Add this line

// SSL/Certificate Handling
#include <WiFiClientSecure.h>


// Add near top of file after includes
#define MQTT_BUFFER_SIZE 1024
#define SSL_BUFFER_SIZE 4096



// Global variables with careful memory management
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER);

// Task Handles
TaskHandle_t mqttReconnectTaskHandle = NULL;
TaskHandle_t ntpUpdateTaskHandle = NULL;

// Function prototypes
void mqttTask(void *parameter);
void checkHeap();
void reconnectMQTT();
void setupEthernet();  // Add this line
void WiFiEvent(WiFiEvent_t event);  // Add this line
void initWebServer();  // Add this line

// Near other constants
#define STATE_TOPIC_PATH String(String(SYSTEM_NAME) + "/" + String(MQTT_CLIENT_ID) + "/" + String(MQTT_STATE_TOPIC))
#define SENSOR_TOPIC_PATH String(String(SYSTEM_NAME) + "/" + String(MQTT_CLIENT_ID) + "/" + String(MQTT_TOPIC_BASE))

// NTP Update Task with error handling
void ntpUpdateTask(void *pvParameters) {
    for (;;) {
        if (ETH.linkUp()) {
            timeClient.update();
            
            time_t epochTime = timeClient.getEpochTime();
            if (epochTime > 1000000000) {  // Basic sanity check
                struct tm* timeInfo = localtime(&epochTime);
                
                char timeBuffer[50];
                strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", timeInfo);
                Serial.println("NTP Time: " + String(timeBuffer));
            } else {
                Serial.println("NTP Time Update Failed");
            }
        } else {
            Serial.println("Cannot Update NTP - Ethernet Down");
        }
        
        // Update every 2 minutes
        vTaskDelay(pdMS_TO_TICKS(120000));
    }
}

// Ethernet Event Handler
void WiFiEvent(WiFiEvent_t event) {
    static uint8_t reconnectAttempts = 0;
    
    switch (event) {
        case ARDUINO_EVENT_ETH_START:
            Serial.println("ETH Started");
            ETH.setHostname("esp32-ethernet");
            break;
            
        case ARDUINO_EVENT_ETH_CONNECTED:
            Serial.println("ETH Connected");
            break;
            
        case ARDUINO_EVENT_ETH_GOT_IP:
            reconnectAttempts = 0;  // Reset counter on successful connection
            Serial.print("ETH MAC: ");
            Serial.print(ETH.macAddress());
            Serial.print(", IPv4: ");
            Serial.print(ETH.localIP());
            Serial.print(", Subnet: ");
            Serial.print(ETH.subnetMask());
            Serial.print(", Gateway: ");
            Serial.println(ETH.gatewayIP());
            break;
            
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            Serial.println("ETH Disconnected");
            if (++reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
                delay(RECONNECT_DELAY);
                // Trigger reconnection
                ETH.begin();
            } else {
                Serial.println("Max reconnection attempts reached. Restarting...");
                ESP.restart();
            }
            break;
            
        case ARDUINO_EVENT_ETH_STOP:
            Serial.println("ETH Stopped");
            break;
            
        default:
            Serial.printf("Unknown ETH event: %d\n", event);
            break;
    }
}

// Add this function before setup()
void setupEthernet() {
    // Initialize Ethernet event handler
    WiFi.onEvent(WiFiEvent);
    
    // Initialize ETH
    ETH.begin();
    
    // Wait for Ethernet connection (optional timeout)
    int timeout = 10; // 10 second timeout
    while (!ETH.linkUp() && timeout > 0) {
        Serial.println("Waiting for Ethernet connection...");
        delay(1000);
        timeout--;
    }
    
    if (!ETH.linkUp()) {
        Serial.println("Ethernet connection failed!");
        ESP.restart();
    }
    
    Serial.println("Ethernet connection established");
}

void setup() {
    // Get the singleton instance first
    MQTTManager& mqttManager = MQTTManager::getInstance();
    
    // Setup serial and network
    Serial.begin(115200);
    setupEthernet();
    
    // Create MQTT reconnect task
    xTaskCreate(
        MQTTManager::mqttReconnectTask,
        "MQTT Reconnect",
        MQTT_RECONNECT_STACK_SIZE,
        NULL,
        1,
        &mqttReconnectTaskHandle
    );
    
    // Initialize MQTT manager (call begin only once)
    mqttManager.begin();

    // In setup()
    initOneWire();
    initWebServer();
    
    xTaskCreate(
        TaskScanSensors,
        "ScanSensors",
        4096,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        TaskReadTemperature,
        "ReadTemp",
        4096,
        NULL,
        1,
        NULL
    );
}

// Add new function before loop()
void publishState() {
    static uint32_t lastPublish = 0;
    const uint32_t now = millis();
    
    if (now - lastPublish >= STATE_PUBLISH_INTERVAL) {
        char state[128];
        snprintf(state, sizeof(state),
                "{\"heap\":%d,\"uptime\":%lu,\"connected\":%d}",
                ESP.getFreeHeap(),
                now / 1000, // Uptime in seconds
                mqttClient.connected());
                
        mqttClient.publish(STATE_TOPIC_PATH.c_str(), state, true); // Retained message
        lastPublish = now;
    }
}

// Modify loop() to include state publishing
void loop() {
    // Get singleton instance
    MQTTManager& mqttManager = MQTTManager::getInstance();
    mqttManager.loop();
    vTaskDelay(pdMS_TO_TICKS(100));
}

// In your loop or task function
void checkHeap() {
    static uint32_t lastCheck = 0;
    const uint32_t CHECK_INTERVAL = 30000; // Check every 30 seconds
    
    if (millis() - lastCheck >= CHECK_INTERVAL) {
        uint32_t freeHeap = ESP.getFreeHeap();
        Serial.printf("Free heap: %u bytes\n", freeHeap);
        lastCheck = millis();
        
        if (freeHeap < 20000) {
            Serial.println("WARNING: Low memory condition!");
        }
    }
}

void mqttTask(void *parameter) {
    static uint32_t lastHeapCheck = 0;
    const TickType_t xDelay = pdMS_TO_TICKS(100);
    
    // Get reference to MQTTManager instance
    MQTTManager& mqttManager = MQTTManager::getInstance();
    
    for(;;) {
        if (!mqttManager.isConnected()) {
            mqttManager.reconnectWithBackoff();
        }
        
        mqttManager.loop();
        
        // Check heap every 5 seconds
        if (xTaskGetTickCount() - lastHeapCheck > pdMS_TO_TICKS(5000)) {
            checkHeap();
            lastHeapCheck = xTaskGetTickCount();
        }
        
        vTaskDelay(xDelay);
    }
}

// Add this function before setup()
void initWebServer() {
    webServer.setMQTTManager(&MQTTManager::getInstance());
    webServer.begin();
}


