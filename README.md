# Chaoticvolt-SensorHUB

A robust ESP32-based temperature sensor hub with MQTT connectivity, web interface, and 7-segment display support.
The web interface allows for real-time temperature monitoring, relay control, and setting sensor firendly names. This also allows for quick diagnostics of the system the sensors are connected to without the need for a mqtt serer or any other external service.

Included are full PCB design files created in easyEDA and exported to Gerber files, Altium Designer Format and pdf for manufacturing.
The code uses FreeRTOS for task management and is written in C++ using the Espressiv framework in PlatformIO.


## Features

- Ethernet connectivity for reliable network communication
- MQTT integration with SSL/TLS security
- Real-time temperature monitoring with Dallas OneWire sensors
- Web dashboard for sensor visualization
- 7-segment LED display for local temperature readings
- Dual relay control with MQTT and web interface
- NTP time synchronization
- mDNS support for easy device discovery
- SPIFFS file system for web assets

## Hardware Requirements

- ESP32 EVB board
- Dallas OneWire temperature sensors (DS18B20)
- TM1637 7-segment display
- Two relay modules
- Ethernet connection

## Pin Configuration

```cpp
#define SYSTEM_RELAY1_PIN 32
#define SYSTEM_RELAY2_PIN 33
#define DISPLAY_CLK_PIN 16
#define DISPLAY_DIO_PIN 13
