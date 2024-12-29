#ifndef CONFIG_H
#define CONFIG_H

// MQTT Broker Configuration
#define SYSTEM_NAME "YourSystemName"
#define MQTT_CLIENT_ID "YourClientName"
#define MQTT_TOPIC_BASE "sensors"
#define MQTT_STATE_TOPIC "state"
#define MQTT_BROKER "url.mqtt.broker"
#define MQTT_RELAY1_SET_TOPIC "relay1"
#define MQTT_RELAY2_SET_TOPIC "relay2"
#define MQTT_PORT 12883
#define MQTT_USERNAME "YourUserName"
#define MQTT_PASSWORD "YourPassword"
#define WEBPAGE_HIGHLITED_SENSOR "28104a482819015c"
#define SYSTEM_RELAY1_TOPIC "relay1"
#define SYSTEM_RELAY2_TOPIC "relay2"

// Pin configurations
#define SYSTEM_RELAY1_PIN 32
#define SYSTEM_RELAY2_PIN 33
#define DISPLAY_CLK_PIN 16
#define DISPLAY_DIO_PIN 13

// Increased stack sizes and added some safeguards
#define MQTT_RECONNECT_STACK_SIZE 6144  // Increased from 4096
#define NTP_UPDATE_STACK_SIZE 4096
#define MQTT_RECONNECT_INTERVAL 15000   // 15 seconds between reconnection attempts
#define MAX_RECONNECT_ATTEMPTS 5
#define RECONNECT_DELAY 5000  // 5 seconds

// OneWire & Temperature Sensor Configuration
const uint8_t ONE_WIRE_BUS = 4;          // GPIO pin 4
const uint8_t MAX_SENSORS = 12;          // Max number of sensors
const uint32_t SCAN_INTERVAL_MS = 12000; // milliseconds
const uint32_t READ_INTERVAL_MS = 6000;  // milliseconds

// Increase task stack sizes
#define MQTT_TASK_STACK_SIZE (8192)
#define DEFAULT_TASK_PRIORITY (1)
#define MQTT_TASK_PRIORITY (2)        // Higher than default
#define ONEWIRE_TASK_PRIORITY (1)     // Standard priority

// NTP Configuration
#define NTP_SERVER "2.europe.pool.ntp.org"
#define GMT_OFFSET_SEC 3600     // +1 hour
#define DAYLIGHT_OFFSET_SEC 3600 // +1 hour DST

// Buffer sizes
#define MQTT_BUFFER_SIZE 1024
#define SSL_BUFFER_SIZE 4096


// Modify existing or add new timing constants
#undef STATE_PUBLISH_INTERVAL
#undef SENSOR_PUBLISH_INTERVAL

static constexpr uint32_t STATE_PUBLISH_INTERVAL = 4000;   // 4 seconds
static constexpr uint32_t SENSOR_PUBLISH_INTERVAL = 4000;  // 4 seconds

#endif
