#include "7segDisplay.h"
#include <ETH.h>
#include <DallasTemperature.h>

SegDisplay::SegDisplay(uint8_t clk_pin, uint8_t dio_pin) : display(clk_pin, dio_pin) {
}

void SegDisplay::begin() {
    Serial.println("Initializing display...");
    display.begin();
    display.setBrightnessPercent(90);
    Serial.println("Display initialized");
}

void SegDisplay::showNumber(int num) {
    display.display(num);
}

void SegDisplay::showFloat(float num) {
    display.display(num);
}

void SegDisplay::showText(const char* text) {
    Serial.printf("Display: showing text '%s'\n", text);
    display.display(text);
}

void SegDisplay::setBrightness(uint8_t percent) {
    display.setBrightnessPercent(percent);
}

// Helper function to display IP address octets
void SegDisplay::showIPAddress(IPAddress ip) {
    // Get last octet of IP address
    int lastOctet = ip[3];
    
    // Format with leading zeros if needed
    char ipStr[5];
    snprintf(ipStr, sizeof(ipStr), "%03d", lastOctet);
    
    // Show on display
    showText(ipStr);
}

// Task to display sequence
void SegDisplay::displayTask(void* parameter) {
    SegDisplay* display = static_cast<SegDisplay*>(parameter);
    
    for(;;) {
        if (xSemaphoreTake(gState.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Check if we have any sensors at all
            if (!gState.temperatures.empty()) {
                // Use either selected sensor or first available one
                size_t sensorIndex = gState.hasSelectedSensor ? 
                    gState.selectedSensorIndex : 0;
                
                // Verify index is valid
                if (sensorIndex < gState.temperatures.size()) {
                    float temp = gState.temperatures[sensorIndex];
                    if (temp != DEVICE_DISCONNECTED_C) {
                        char tempStr[8];
                        snprintf(tempStr, sizeof(tempStr), "%.1f", temp);
                        display->showText(tempStr);
                        xSemaphoreGive(gState.mutex);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        continue;
                    }
                }
            }
            xSemaphoreGive(gState.mutex);
            display->showText("---");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Add to SegDisplay class
void SegDisplay::startDisplayTask() {
    xTaskCreatePinnedToCore(
        SegDisplay::displayTask,  // Use fully qualified name
        "DisplayTask",
        4096,
        this,
        1,  // Lower priority than relay control
        NULL,
        0   // Run on core 0
    );
}