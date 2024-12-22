#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class MutexGuard {
private:
    SemaphoreHandle_t& mutex;
    bool locked;
    
public:
    explicit MutexGuard(SemaphoreHandle_t& m, TickType_t timeout = pdMS_TO_TICKS(100)) 
        : mutex(m), locked(false) {
        locked = (xSemaphoreTake(mutex, timeout) == pdTRUE);
    }
    
    ~MutexGuard() {
        if (locked) {
            xSemaphoreGive(mutex);
        }
    }
    
    bool isLocked() const { return locked; }

    // Prevent copying
    MutexGuard(const MutexGuard&) = delete;
    MutexGuard& operator=(const MutexGuard&) = delete;
};