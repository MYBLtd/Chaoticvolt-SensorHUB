#include "global_state.h"
#include "config.h"

GlobalState gState;  // Global instance definition

void initializeGlobalState() {
    gState.mutex = xSemaphoreCreateMutex();
    gState.relay1State = false;
    gState.relay2State = false;
    gState.mqttConnected = false;
    gState.wifiConnected = false;
    gState.lastStatePublish = 0;
    gState.lastSensorPublish = 0;
    gState.dataUpdated = false;
}

void updateRelayState(uint8_t relay, bool state) {
    if (relay == SYSTEM_RELAY1_PIN) {
        gState.relay1State = state;
    } else if (relay == SYSTEM_RELAY2_PIN) {
        gState.relay2State = state;
    }
}

bool getRelayState(uint8_t relay) {
    if (relay == SYSTEM_RELAY1_PIN) {
        return gState.relay1State;
    } else if (relay == SYSTEM_RELAY2_PIN) {
        return gState.relay2State;
    }
    return false;
}