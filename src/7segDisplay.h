#pragma once

#include <Arduino.h>
#include <TM1637.h>
#include "global_state.h"

class SegDisplay {
private:
    TM1637 display;
    static constexpr unsigned long TOGGLE_INTERVAL = 3000;  // 3 seconds
    static void displayTask(void* parameter);

public:
    SegDisplay(uint8_t clk_pin, uint8_t dio_pin);
    void begin();
    void showNumber(int num);
    void showFloat(float num);
    void showText(const char* text);
    void setBrightness(uint8_t percent);
    void startDisplayTask();
    void showIPAddress(IPAddress ip);
};
