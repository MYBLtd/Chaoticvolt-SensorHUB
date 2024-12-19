// onewire.h
#ifndef ONEWIRE_MANAGER_H
#define ONEWIRE_MANAGER_H

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <vector>
#include <array>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "config.h"

// Function declarations
void initOneWire();
void scanSensors();
void TaskScanSensors(void* parameter);
void TaskReadTemperature(void* parameter);
String sensorAddressToString(const std::array<uint8_t, 8>& addr);

// Global variables
extern OneWire oneWire;
extern DallasTemperature sensors;
extern std::vector<std::array<uint8_t, 8>> sensorAddresses;
extern std::vector<float> temperatures;
extern SemaphoreHandle_t oneWireMutex;

#endif