#include "onewire_manager.h"
#include "mqtt_manager.h"
#include "webserver.h"
#include "config.h"
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <vector>
#include <array>

// Object instantiation
extern TempWebServer webServer;
OneWire oneWire(ONE_WIRE_BUS);  // Uses ONE_WIRE_BUS from config.h
DallasTemperature sensors(&oneWire);
std::vector<std::array<uint8_t, 8>> sensorAddresses;
std::vector<float> temperatures;
SemaphoreHandle_t oneWireMutex = NULL;

// Function declarations
String sensorAddressToString(const std::array<uint8_t, 8>& address);

void initOneWire() {
    oneWireMutex = xSemaphoreCreateMutex();
    sensors.begin();
}

// Scan for connected sensors
void scanSensors() {
    sensors.begin();  // Initialize sensors
    uint8_t deviceCount = sensors.getDeviceCount();
    sensorAddresses.clear();
    
    DeviceAddress tempAddr;
    for(uint8_t i = 0; i < min(deviceCount, MAX_SENSORS); i++) {
        if(sensors.getAddress(tempAddr, i)) {
            std::array<uint8_t, 8> addr;
            memcpy(addr.data(), tempAddr, 8);
            sensorAddresses.push_back(addr);
        }
    }
}

float getTemperature(int index) {
    if (index >= sensorAddresses.size()) {
        return -127.0;  // Error value
    }
    sensors.requestTemperatures();
    return sensors.getTempC((uint8_t*)sensorAddresses[index].data());
}

// Task for reading temperature sensors and publishing data
void TaskReadTemperature(void* parameter) {
    for(;;) {
        if(xSemaphoreTake(gState.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            sensors.requestTemperatures();
            gState.temperatures.clear();
            gState.sensorAddresses.clear();
            
            // Update temperatures in global state
            for(size_t i = 0; i < sensorAddresses.size(); i++) {
                float temp = sensors.getTempC((uint8_t*)sensorAddresses[i].data());
                if(temp != DEVICE_DISCONNECTED_C) {
                    gState.temperatures.push_back(temp);
                    gState.sensorAddresses.push_back(sensorAddresses[i]);
                }
            }
            
            // Set data updated flag
            gState.dataUpdated = true;
            Serial.println("Global state updated with new sensor data");
            
            xSemaphoreGive(gState.mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(READ_INTERVAL_MS));
    }
}

// Task for scanning and discovering new sensors
void TaskScanSensors(void* parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while(true) {
        if(xSemaphoreTake(oneWireMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            Serial.println("Scanning for sensors...");
            scanSensors();
            xSemaphoreGive(oneWireMutex);
            
            // Log found sensors
            Serial.printf("Found %d sensors:\n", sensorAddresses.size());
            for(size_t i = 0; i < sensorAddresses.size(); i++) {
                Serial.printf("Sensor %d: %s\n", i, 
                    sensorAddressToString(sensorAddresses[i]).c_str());
            }
        }
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SCAN_INTERVAL_MS));
    }
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

String sensorAddressToString(const std::array<uint8_t, 8>& address) {
    String addrStr = "";
    for (uint8_t i = 0; i < 8; i++) {
        if (address[i] < 16) addrStr += "0";
        addrStr += String(address[i], HEX);
    }
    return addrStr;
}


