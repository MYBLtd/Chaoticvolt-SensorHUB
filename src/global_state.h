#ifndef GLOBAL_STATE_H
#define GLOBAL_STATE_H

#include <Arduino.h>

struct GlobalState {
    bool relay1State;
    bool relay2State;
    bool mqttConnected;
    bool wifiConnected;
    unsigned long lastStatePublish;
    unsigned long lastSensorPublish;
};

extern GlobalState gState;

void initializeGlobalState();
void updateRelayState(uint8_t relay, bool state);
bool getRelayState(uint8_t relay);

#endif
